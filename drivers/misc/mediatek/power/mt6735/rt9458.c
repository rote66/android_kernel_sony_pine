/*
 *  Driver for Richtek RT9458 Charger 
 *
 *  Copyright (C) 2015 Richtek Technology Corp.
 *  shufan_lee <shufan_lee@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include "rt9458.h"

#ifdef CONFIG_RT_REGMAP
#include <mt-plat/charging.h>
#include "rt-regmap.h"
#endif

/*[Arima_8100][bozhi_lin] RID003699 OVP warning message to end-user 20170320 begin*/
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
static int rt9458_flag;
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
/*[Arima_8100][bozhi_lin] 20170320 end*/

/* ================= */
/* extern variable */
/* ================= */

kal_bool chargin_hw_init_done = KAL_FALSE; /* Used by MTK battery driver */

/* ================= */
/* internal variable */
/* ================= */

static const unsigned char rt9458_reg_addr[RT9458_REG_IDX_MAX] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x11,
};

/* E1 revision */
static const unsigned char rt9458_reg_def_val_e1[RT9458_REG_IDX_MAX] = {
	0x40, 0x40, 0x8E, 0x01, 0x00, 0x82, 0x02, 0x10,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42,
};

/* E2 revision */
static const unsigned char rt9458_reg_def_val_e2[RT9458_REG_IDX_MAX] = {
	0x40, 0x40, 0x8E, 0x02, 0x00, 0x82, 0x02, 0x10,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42,
};

/* Point to def val of current revision, e.g. rt9458_reg_def_val_e1
 * Assigned in rt9458_is_hw_exist
 * If there is no rt9458 detected, it will be NULL
 */
static const unsigned char *rt9458_reg_def_val = NULL;

#define RT9458_DBG_MSG_I2C 1
#define RT9458_DBG_MSG_OTH 1

#define RT9458_ICHG_NUM 8
#define RT9458_IEOC_NUM 8
#define RT9458_MIVR_NUM 7
#define RT9458_VPREC_NUM 8
#define RT9458_IPREC_NUM 4
#define RT9458_VOREG_NUM 64
#define RT9458_VMREG_NUM 16



/* Charging status name */
static const char *rt9458_chg_status_name[RT_CHG_STATUS_MAX] = {
	"ready", "progress", "done", "fault",
};

/* DTS configuration */
enum {
	RT9458_DTS_CFG_ICHG = 0,
	RT9458_DTS_CFG_AICR,
	RT9458_DTS_CFG_MIVR,
	RT9458_DTS_CFG_IEOC,
	RT9458_DTS_CFG_TE,
	RT9458_DTS_CFG_TE_SHDN,
	RT9458_DTS_CFG_MAX,
};
static const char *rt9458_dts_cfg_name[RT9458_DTS_CFG_MAX] = {
	"ichg", "aicr", "mivr", "ieoc", "te", "te-shdn",	
};

/* unit: mA */
static const u32 rt9458_ieoc_table[RT9458_IEOC_NUM] = {
	50, 100, 150, 200, 250, 300, 350, 400,
};

/* unit: mA */
static const u32 rt9458_ichg_table[RT9458_ICHG_NUM] = {
	500, 650, 800, 950, 1100, 1250, 1400, 1550,	
};

/* unit: mV */
static const u32 rt9458_mivr_table[RT9458_MIVR_NUM] = {
	4100, 4200, 4300, 4400, 4500, 4600, 4700,
};

/* unit: mV */
static const u32 rt9458_vprec_table[RT9458_VPREC_NUM] = {
	2000, 2200, 2400, 2600, 2800, 3000, 3000, 3000,	
};

/* unit: mA */
static const u32 rt9458_iprec_table[RT9458_IPREC_NUM] = {
	20, 40, 60, 60,
};

/* unit: mV */
static const u32 rt9458_battery_voreg_table[RT9458_VOREG_NUM] = {
	3500, 3520, 3540, 3560, 3580, 3600, 3620, 3640,
	3660, 3680, 3700, 3720, 3740, 3760, 3780, 3800,
	3820, 3840, 3860, 3880, 3900, 3920, 3940, 3960,
	3980, 4000, 4020, 4040, 4060, 4080, 4100, 4120,
	4140, 4160, 4180, 4200, 4220, 4240, 4260, 4280,
	4300, 4330, 4350, 4370, 4390, 4410, 4430, 4450,
	4450, 4450, 4450, 4450, 4450, 4450, 4450, 4450,
	4450, 4450, 4450, 4450, 4450, 4450, 4450, 4450,
};

/* unit: mV */
static const u32 rt9458_boost_voreg_table[RT9458_VOREG_NUM] = {
	4425, 4450, 4475, 4500, 4525, 4550, 4575, 4600,
	4625, 4650, 4675, 4700, 4725, 4750, 4775, 4800,
	4825, 4850, 4875, 4900, 4925, 4950, 4975, 5000,
	5025, 5050, 5075, 5100, 5125, 5150, 5175, 5200,
	5225, 5250, 5275, 5300, 5325, 5350, 5375, 5400,
	5425, 5450, 5475, 5500, 5525, 5550, 5575, 5600,
	5600, 5600, 5600, 5600, 5600, 5600, 5600, 5600,
	5600, 5600, 5600, 5600, 5600, 5600, 5600, 5600,
};

/* unit: mV */
static const u32 rt9458_battery_vmreg_table[RT9458_VMREG_NUM] = {
	4200, 4220, 4240, 4260, 4280, 4300, 4320, 4340,
	4360, 4380, 4400, 4430, 4450, 4450, 4450, 4450,	
};

/* unit: mV */
static const u32 rt9458_boost_vmreg_table[RT9458_VMREG_NUM] = {
	5300, 5325, 5350, 5375, 5400, 5425, 5450, 5475,
	5500, 5525, 5550, 5575, 5600, 5600, 5600, 5600,
};

#ifdef CONFIG_RT_REGMAP
RT_REG_DECL(RT9458_REG_CTRL1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_DEVID, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL4, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL5, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL6, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL7, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_IRQ1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_IRQ2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_IRQ3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_MASK1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_MASK2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_MASK3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9458_REG_CTRL8, 1, RT_VOLATILE, {});

static rt_register_map_t rt9458_regmap_map[RT9458_REG_IDX_MAX] = {
	RT_REG(RT9458_REG_CTRL1),	
	RT_REG(RT9458_REG_CTRL2),	
	RT_REG(RT9458_REG_CTRL3),	
	RT_REG(RT9458_REG_DEVID),	
	RT_REG(RT9458_REG_CTRL4),	
	RT_REG(RT9458_REG_CTRL5),	
	RT_REG(RT9458_REG_CTRL6),	
	RT_REG(RT9458_REG_CTRL7),	
	RT_REG(RT9458_REG_IRQ1),	
	RT_REG(RT9458_REG_IRQ2),	
	RT_REG(RT9458_REG_IRQ3),	
	RT_REG(RT9458_REG_MASK1),	
	RT_REG(RT9458_REG_MASK2),	
	RT_REG(RT9458_REG_MASK3),	
	RT_REG(RT9458_REG_CTRL8),	
};

static struct rt_regmap_properties rt9458_regmap_prop = {
	.name = "rt9458",
	.aliases = "rt9458",
	.register_num = RT9458_REG_IDX_MAX,
	.rm = rt9458_regmap_map,
	.rt_regmap_mode = RT_SINGLE_BYTE | RT_CACHE_DISABLE | RT_IO_PASS_THROUGH,
	.io_log_en = 1,
};
#endif /* CONFIG_RT_REGMAP */


struct rt9458_info {
	struct i2c_client *i2c;
	int dts_cfg[RT9458_DTS_CFG_MAX];
#ifdef CONFIG_RT_REGMAP
	struct rt_regmap_device *regmap_dev;
#endif
/*[Arima_8100][bozhi_lin] RID003699 OVP warning message to end-user 20170320 begin*/
	int irq_gpio;
	int irq;
	int vinovpi_flag;
/*[Arima_8100][bozhi_lin] 20170320 end*/
};

static struct rt9458_info g_rt9458_info = {
	.i2c = NULL,
	.dts_cfg = {500, 500, 4500, 150, 1, 1},
#ifdef CONFIG_RT_REGMAP
	.regmap_dev = NULL,
#endif
};

/* ========================= */
/* I2C operations */
/* ========================= */

#ifdef CONFIG_RT_REGMAP
static ssize_t rt9458_debug_write(struct file *file,
	const char __user *user_buffer, size_t count, loff_t *position)
{
	long unsigned int cmd = 0, data = 0;
	char buf[256] = "\0";
	char *pch = NULL;
	char tmp[128];
	int ret = 0, substr_len = 0;

	simple_write_to_buffer(buf, sizeof(buf), position, user_buffer, count);

	/* Parse cmd */
	pch = strchr(buf, ',');
	substr_len = pch ? (pch - buf) : strlen(buf);
	strncpy(tmp, buf, substr_len);
	tmp[substr_len] = '\0';
	ret = kstrtoul(tmp, 10, &cmd);

	/* Parse data */
	if (pch) {
		substr_len = strlen(buf) - substr_len - 1;
		strncpy(tmp, pch + 1, substr_len);
		ret = kstrtoul(tmp, 10, &data);
	}

	chr_control_interface(cmd, &data);
	return count;
}

static ssize_t rt9458_debug_read(struct file *file, char __user *user_buffer,
	size_t count, loff_t *position)
{
	char buf[256] = "\0";
	return simple_read_from_buffer(user_buffer, count, position,
		buf, strlen(buf));
}

static int rt9458_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static struct file_operations rt9458_dbgfs_ops = {
	.owner = THIS_MODULE,
	.open = rt9458_debug_open,
	.read = rt9458_debug_read,
	.write = rt9458_debug_write,
};

static int rt9458_regmap_read(void *client, u32 addr, int leng, void *dst)
{
	int ret = 0;

	struct i2c_client *i2c = (struct i2c_client *)client;
	ret = i2c_smbus_read_i2c_block_data(i2c, addr, leng, dst);
	
#if RT9458_DBG_MSG_I2C
	if (ret < 0)
		pr_info("%s: I2CR[0x%02X] failed\n", __func__, addr);
	else
		pr_info("%s: I2CR[0x%02X] = 0x%02X\n", __func__, addr, *((u8 *)dst));
#endif

	return ret;
}

static int rt9458_regmap_write(void *client, u32 addr, int leng, const void *src)
{
	int ret = 0;

	struct i2c_client *i2c = (struct i2c_client *)client;
	ret = i2c_smbus_write_i2c_block_data(i2c, addr, leng, src);	
	
#if RT9458_DBG_MSG_I2C
	if (ret < 0)
		pr_info("%s: I2CW[0x%02X] = 0x%02X failed\n", __func__, addr,
			*((u8 *)src));
	else
		pr_info("%s: I2CW[0x%02X] = 0x%02X\n", __func__, addr, *((u8 *)src));
#endif

	return ret;
}

static struct rt_regmap_fops rt9458_regmap_fops = {
	.read_device = rt9458_regmap_read,
	.write_device = rt9458_regmap_write,
};


static int rt9458_register_rt_regmap(void)
{
	int ret = 0;
	pr_info("%s: starts\n", __func__);
	g_rt9458_info.regmap_dev = rt_regmap_device_register(
		&rt9458_regmap_prop,
		&rt9458_regmap_fops,
		&(g_rt9458_info.i2c->dev),
		g_rt9458_info.i2c,
		&g_rt9458_info
	);

	if (!g_rt9458_info.regmap_dev) {
		dev_err(&g_rt9458_info.i2c->dev, "register regmap device failed\n");
		return -EINVAL;
	}

	ret = rt_regmap_add_debugfs(
		g_rt9458_info.regmap_dev,
		"chr_control_interface",
		S_IRUGO | S_IWUGO,
		NULL,
		&rt9458_dbgfs_ops
	); 

	if (ret < 0)
		pr_info("%s: add chr_control_interface to dbgfs failed\n", __func__);
	
	pr_info("%s: ends\n", __func__);
	return ret;
}
#else

static int rt9458_write_device(struct i2c_client *i2c, u32 reg, int leng,
	const void *src)
{
	int ret = 0;
	
	if (leng > 1) {
		ret = i2c_smbus_write_i2c_block_data(i2c, reg, leng, src);
	} else {
		ret = i2c_smbus_write_byte_data(i2c, reg, *((u8 *)src));
		if (ret < 0)
			return ret;
	}

#if RT9458_DBG_MSG_I2C
	if (ret < 0)
		pr_info("%s: I2CW[0x%02X] = 0x%02X failed\n", __func__, reg,
			*((u8 *)src));
	else
		pr_info("%s: I2CW[0x%02X] = 0x%02X\n", __func__, reg, *((u8 *)src));
#endif

	return ret;
}

static int rt9458_read_device(struct i2c_client *i2c, u32 reg, int leng,
	void *dst)
{
	int ret = 0;
	
	if (leng > 1) {
		ret = i2c_smbus_read_i2c_block_data(i2c, reg, leng, dst);
	} else {
		ret = i2c_smbus_read_byte_data(i2c, reg);
		if (ret < 0)
			return ret;

		*(u8 *)dst = (u8)ret;
	}

#if RT9458_DBG_MSG_I2C
	if (ret < 0)
		pr_info("%s: I2CR[0x%02X] failed\n", __func__, reg);
	else
		pr_info("%s: I2CR[0x%02X] = 0x%02X\n", __func__, reg, *((u8 *)dst));
#endif

	return ret;
}
#endif /* CONFIG_RT_REGMAP */

static int rt9458_i2c_write_byte(u8 cmd, u8 data)
{
	int ret = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt9458_regmap_write(g_rt9458_info.i2c, cmd, 1, &data);
#else
	ret = rt9458_write_device(g_rt9458_info.i2c, cmd, 1, &data);
#endif

	return ret;
}


static int rt9458_i2c_read_byte(u8 cmd)
{
	int ret = 0, ret_val = 0;

#ifdef CONFIG_RT_REGMAP
	ret = rt9458_regmap_read(g_rt9458_info.i2c, cmd, 1, &ret_val);
#else
	ret = rt9458_read_device(g_rt9458_info.i2c, cmd, 1, &ret_val);
#endif

	if (ret < 0)
		return ret;

	ret_val = ret_val & 0xFF;

	return ret_val;
}


static int rt9458_i2c_update_bits(u8 cmd, u8 data, u8 mask)
{
	int ret = 0;
	u8 reg_data = 0;

	ret = rt9458_i2c_read_byte(cmd);
	if (ret < 0)
		return ret;

	reg_data = ret & 0xFF;
	reg_data &= ~mask;
	reg_data |= (data & mask);

	return rt9458_i2c_write_byte(cmd, reg_data);
}

#define rt9458_set_bit(reg, mask) \
	rt9458_i2c_update_bits(reg, mask, mask)

#define rt9458_clr_bit(reg, mask) \
	rt9458_i2c_update_bits(reg, 0x00, mask)

/* ================== */
/* internal functions */
/* ================== */

static u8 rt9458_find_closest_reg_value(const u32 *value_table,
	const u32 table_size, const u32 target_value)
{
	u32 i = 0;

	/* Smaller than minimum supported value, use minimum one */
	if (target_value < value_table[0]) {
		return 0;
	}

	for (i = 0; i < table_size - 1; i++) {
		if (target_value >= value_table[i] && target_value < value_table[i + 1])
			return i;
	}

	/* Greater than maximum supported value, use maximum one */
	return table_size - 1; 
}

static int rt9458_set_aicr_reg_val(const u8 reg_aicr)
{
	int ret = 0;

	ret = rt9458_i2c_update_bits(
		RT9458_REG_CTRL2,
		reg_aicr << RT9458_SHIFT_AICR,
		RT9458_MASK_AICR
	);

	return ret;
}

static int rt9458_set_aicr_sel(const u8 sel)
{
	/* sel = 1, 500mA and 1A become 700mA */	
	int ret = 0;

	ret = rt9458_i2c_update_bits(
		RT9458_REG_CTRL6,
		sel << RT9458_SHIFT_AICR_SEL,
		RT9458_MASK_AICR_SEL
	);

	return ret;
}

static int rt9458_set_aicr_int(const u8 internal)
{
	/* internal = 1, use AICR setting in CTRL2
     * otherwise, AICR is decided by OTG_PinP
	 */	
	int ret = 0;

	ret = rt9458_i2c_update_bits(
		RT9458_REG_CTRL2,
		internal << RT9458_SHIFT_AICR_INT,
		RT9458_MASK_AICR_INT
	);

	return ret;	
}

static u8 rt9458_is_hw_exist(void)
{
	int ret = 0;
	u8 chip_revision = 0x01;
	
	ret = rt9458_i2c_read_byte(RT9458_REG_DEVID);
	if (ret < 0) {
		return 0;	
	}

	chip_revision = ret & 0xFF;
	if (chip_revision == rt9458_reg_def_val_e1[RT9458_REG_IDX_DEVID]) {
		rt9458_reg_def_val = rt9458_reg_def_val_e1;
		pr_info("%s: E1 revision\n", __func__);
	} else if (chip_revision == rt9458_reg_def_val_e2[RT9458_REG_IDX_DEVID]) {
		rt9458_reg_def_val = rt9458_reg_def_val_e2;
		pr_info("%s: E2 revision\n", __func__);
	} else {
		/* Unknown revision */
		return 0;	
	}

	return 1;			
}

static int rt9458_set_battery_vmreg(u32 vmreg)
{
	int ret = 0;
	u8 reg_vmreg = 0x00;

	reg_vmreg = rt9458_find_closest_reg_value(rt9458_battery_vmreg_table,
		RT9458_VMREG_NUM, vmreg);

#ifdef RT9458_DBG_MSG_OTH
	pr_info("%s: vmreg = %d\n", __func__,
		rt9458_battery_vmreg_table[reg_vmreg]);
#endif

	ret = rt9458_i2c_update_bits(
		RT9458_REG_CTRL7,
		reg_vmreg << RT9458_SHIFT_VMREG,
		RT9458_MASK_VMREG
	);

	return ret;
}

/* The following is implementation for interface of rt_charger */

/* Set register's value to default */
int rt_charger_hw_init(void)
{
	int ret = 0;
	int i = 0;

#if RT9458_DBG_MSG_OTH
	pr_info("%s: starts\n", __func__);
#endif

	for (i = 0; i < RT9458_REG_IDX_MAX; i++) {
		ret = rt9458_i2c_write_byte(
			rt9458_reg_addr[i],
			rt9458_reg_def_val[i]
		);

		if (ret < 0)
			return ret;
	}

	return ret;
}

int rt_charger_sw_init(void)
{
	int ret = 0;

#if RT9458_DBG_MSG_OTH
	pr_info("%s: starts\n", __func__);
#endif

	/* Set VMREG to maximum */
	ret = rt9458_set_battery_vmreg(4450);
	if (ret < 0)
		pr_info("%s: Set maximum vmreg failed\n", __func__);

	ret = rt_charger_set_ichg(g_rt9458_info.dts_cfg[RT9458_DTS_CFG_ICHG]);
	if (ret < 0)
		pr_info("%s: Set default ichg failed\n", __func__);

	ret = rt_charger_set_aicr(g_rt9458_info.dts_cfg[RT9458_DTS_CFG_AICR]);
	if (ret < 0)
		pr_info("%s: Set default aicr failed\n", __func__);

	ret = rt_charger_set_mivr(g_rt9458_info.dts_cfg[RT9458_DTS_CFG_MIVR]);
	if (ret < 0)
		pr_info("%s: Set default mivr failed\n", __func__);

	ret = rt_charger_set_ieoc(g_rt9458_info.dts_cfg[RT9458_DTS_CFG_IEOC]);
	if (ret < 0)
		pr_info("%s: Set default ieoc failed\n", __func__);

	ret = rt_charger_enable_te(g_rt9458_info.dts_cfg[RT9458_DTS_CFG_TE]);
	if (ret < 0)
		pr_info("%s: Set default te failed\n", __func__);

	ret = rt_charger_enable_te_shutdown(
		g_rt9458_info.dts_cfg[RT9458_DTS_CFG_TE_SHDN]
	);
	if (ret < 0)
		pr_info("%s: Set default te_shutdown failed\n", __func__);

	return ret;
}

int rt_charger_dump_register(void)
{
	int i = 0, ret = 0;
	u32 ichg = 0, aicr = 0, mivr = 0, ieoc = 0;
	u8 chg_enable = 0;
	enum rt_charging_status chg_status = RT_CHG_STATUS_READY;

	for (i = 0; i < RT9458_REG_IDX_MAX; i++) {
		ret = rt9458_i2c_read_byte(rt9458_reg_addr[i]);	
		if (ret < 0)
			return ret;
	}

	ret = rt_charger_get_ichg(&ichg);
	ret = rt_charger_get_mivr(&mivr);
	ret = rt_charger_get_aicr(&aicr);
	ret = rt_charger_get_ieoc(&ieoc);
	ret = rt_charger_is_charging_enable(&chg_enable);
	ret = rt_charger_get_charging_status(&chg_status);

	pr_info("%s: ICHG = %dmA, AICR = %dmA, MIVR = %dmV, IEOC = %dmA\n",
		__func__, ichg, aicr, mivr, ieoc);
	pr_info("%s: CHG_EN = %d, CHG_STATUS = %s\n",
		__func__, chg_enable, rt9458_chg_status_name[chg_status]);

	return ret;
}

int rt_charger_enable_charging(const u8 enable)
{
	int ret = 0;

#if RT9458_DBG_MSG_OTH
	pr_info("%s: enable charger = %d\n", __func__, enable);
#endif

	ret = (enable ? rt9458_set_bit(RT9458_REG_CTRL7, RT9458_MASK_CHG_EN)
					: rt9458_clr_bit(RT9458_REG_CTRL7, RT9458_MASK_CHG_EN));

	return ret;
}

int rt_charger_enable_hz(u8 enable)
{
	int ret = 0;

	ret = (enable ? rt9458_set_bit(RT9458_REG_CTRL2, RT9458_MASK_HZ_EN)
					: rt9458_clr_bit(RT9458_REG_CTRL2, RT9458_MASK_CHG_EN));

	return ret;
}

int rt_charger_enable_te(const u8 enable)
{
	int ret = 0;

#if RT9458_DBG_MSG_OTH
	pr_info("%s: enable te = %d\n", __func__, enable);
#endif

	ret = (enable ? rt9458_set_bit(RT9458_REG_CTRL2, RT9458_MASK_TE_EN)
					: rt9458_clr_bit(RT9458_REG_CTRL2, RT9458_MASK_TE_EN));

	return ret;
}

int rt_charger_enable_te_shutdown(const u8 enable)
{
	int ret = 0;

#if RT9458_DBG_MSG_OTH
	pr_info("%s: enable te-shdn = %d\n", __func__, enable);
#endif

	ret = (enable ? rt9458_set_bit(RT9458_REG_CTRL2, RT9458_MASK_TE_SHDN_EN)
					: rt9458_clr_bit(RT9458_REG_CTRL2, RT9458_MASK_TE_SHDN_EN));

	return ret;
}

int rt_charger_enable_timer(const u8 enable)
{
	int ret = 0;

	ret = (enable ? rt9458_set_bit(RT9458_REG_CTRL5, RT9458_MASK_TMR_EN)
					: rt9458_clr_bit(RT9458_REG_CTRL5, RT9458_MASK_TMR_EN));

	return ret;
}

int rt_charger_enable_otg(const u8 enable)
{
	int ret = 0;

	pr_info("%s: otg enable = %d\n", __func__, enable);

	ret = (enable ? rt9458_set_bit(RT9458_REG_CTRL2, RT9458_MASK_OPA_MODE)
					: rt9458_clr_bit(RT9458_REG_CTRL2, RT9458_MASK_OPA_MODE));

	return ret;
}

int rt_charger_enable_pumpX()
{
	return -ENOTSUPP;
}

int rt_charger_disable_aicr()
{
	int ret = 0;

	ret = rt9458_set_aicr_reg_val(0x00);		

	return ret;
}

int rt_charger_disable_mivr()
{
	int ret = 0;
	
	ret = rt9458_i2c_update_bits(
		RT9458_REG_CTRL8,
		0x08 << RT9458_SHIFT_MIVR,
		RT9458_MASK_MIVR
	);

	return ret;
}

int rt_charger_set_ichg(const u32 ichg)
{
	int ret = 0;

	/* Find corresponding reg value */
	u8 reg_ichg = rt9458_find_closest_reg_value(rt9458_ichg_table,
		RT9458_ICHG_NUM, ichg);
#ifdef RT9458_DBG_MSG_OTH
	pr_info("%s: ichg = %d\n", __func__, rt9458_ichg_table[reg_ichg]);
#endif
	ret = rt9458_i2c_update_bits(
		RT9458_REG_CTRL6,
		reg_ichg << RT9458_SHIFT_ICHG,
		RT9458_MASK_ICHG
	);

	return ret;
}

int rt_charger_set_ieoc(const u32 ieoc)
{
	int ret = 0;	

	/* Find corresponding reg value */
	u8 reg_ieoc = rt9458_find_closest_reg_value(rt9458_ieoc_table,
		RT9458_IEOC_NUM, ieoc);

#ifdef RT9458_DBG_MSG_OTH
	pr_info("%s: ieoc = %d\n", __func__, rt9458_ieoc_table[reg_ieoc]);
#endif
	ret = rt9458_i2c_update_bits(
		RT9458_REG_CTRL5,
		reg_ieoc << RT9458_SHIFT_IEOC,
		RT9458_MASK_IEOC
	);

	return ret;
}


int rt_charger_set_aicr(const u32 aicr)
{
	int ret = 0;	

#if RT9458_DBG_MSG_OTH
	pr_info("%s: aicr = %d\n", __func__, aicr);
#endif

	
	ret = rt9458_set_aicr_int(1);	
	if (ret < 0) {
#if RT9458_DBG_MSG_OTH
		pr_info("%s: set aicr_int failed\n", __func__);
#endif
		return ret;
	}
	
	if (aicr < 500) { /* AICR = 100 mA */
		ret = rt9458_set_aicr_reg_val(0x03);		
	} else if (aicr < 700) { /* AICR = 500 mA */
		ret = rt9458_set_aicr_sel(0);	
		if (ret < 0)
			return ret;
		ret = rt9458_set_aicr_reg_val(0x01);
	} else if (aicr < 1000) { /* AICR = 700 mA */
		ret = rt9458_set_aicr_sel(1);
		if (ret < 0)
			return ret;
		ret = rt9458_set_aicr_reg_val(0x01);
/*[Arima_8100][bozhi_lin] set ac charging current to 1.5A and enable high voltage battery 20160922 begin*/
	} else if (aicr < 1500) { /* AICR = 1000 mA */
		ret = rt9458_set_aicr_sel(0);
		if (ret < 0)
			return ret;
		ret = rt9458_set_aicr_reg_val(0x02);
	} else { /* AICR = 1500 mA */
		ret = rt9458_set_aicr_sel(0);
		if (ret < 0)
			return ret;
		ret = rt9458_set_aicr_reg_val(0x03);
/*[Arima_8100][bozhi_lin] 20160922 end*/
	}

	return ret;
}
	

int rt_charger_set_mivr(const u32 mivr)
{
	int ret = 0;	

	/* Find corresponding reg value */
	u8 reg_mivr = rt9458_find_closest_reg_value(rt9458_mivr_table,
		RT9458_MIVR_NUM, mivr);

#ifdef RT9458_DBG_MSG_OTH
	pr_info("%s: mivr = %d\n", __func__, rt9458_mivr_table[reg_mivr]);
#endif
	ret = rt9458_i2c_update_bits(
		RT9458_REG_CTRL8,
		reg_mivr << RT9458_SHIFT_MIVR,
		RT9458_MASK_MIVR
	);

	return ret;
}

int rt_charger_set_battery_voreg(const u32 voreg)
{
	int ret = 0;

	u8 reg_voreg = rt9458_find_closest_reg_value(
		rt9458_battery_voreg_table, RT9458_VOREG_NUM, voreg);

#ifdef RT9458_DBG_MSG_OTH
	pr_info("%s: bat voreg = %d\n", __func__,
		rt9458_battery_voreg_table[reg_voreg]);
#endif

	ret = rt9458_i2c_update_bits(
		RT9458_REG_CTRL3,
		reg_voreg << RT9458_SHIFT_VOREG,
		RT9458_MASK_VOREG
	);

	return ret;
}

int rt_charger_set_boost_voreg(const u32 voreg)
{
	int ret = 0;

	u8 reg_voreg = rt9458_find_closest_reg_value(
		rt9458_boost_voreg_table, RT9458_VOREG_NUM, voreg);

	ret = rt9458_i2c_update_bits(
		RT9458_REG_CTRL3,
		reg_voreg << RT9458_SHIFT_VOREG,
		RT9458_MASK_VOREG
	);

	return ret;
}

int rt_charger_set_ta_current_pattern(const u8 is_pump_up)
{
	return -ENOTSUPP;	
}

int rt_charger_get_charging_status(enum rt_charging_status *chg_stat)
{
	int ret = 0;

	ret = rt9458_i2c_read_byte(RT9458_REG_CTRL1);
	if (ret < 0) {
		return ret;
	}

	*chg_stat = (ret & RT9458_MASK_CHG_STAT) >> RT9458_SHIFT_CHG_STAT;

#if RT9458_DBG_MSG_OTH
	pr_info("%s: CHG_STATUS = %s\n",
		__func__, rt9458_chg_status_name[*chg_stat]);
#endif

	return ret;
}

int rt_charger_get_ichg(u32 *ichg)
{
	int ret = 0;
	u8 reg_ichg = 0;

	ret = rt9458_i2c_read_byte(RT9458_REG_CTRL6);
	if (ret < 0)
		return ret;

	reg_ichg = (ret & RT9458_MASK_ICHG) >> RT9458_SHIFT_ICHG;
	*ichg = rt9458_ichg_table[reg_ichg];

	return ret;
}

int rt_charger_get_ieoc(u32 *ieoc)
{
	int ret = 0;
	u8 reg_ieoc = 0;

	ret = rt9458_i2c_read_byte(RT9458_REG_CTRL5);
	if (ret < 0)
		return ret;

	reg_ieoc = (ret & RT9458_MASK_IEOC) >> RT9458_SHIFT_IEOC;
	*ieoc = rt9458_ieoc_table[reg_ieoc];

	return ret;
}

int rt_charger_get_aicr(u32 *aicr)
{
	int ret = 0;
	u8 reg_aicr_sel = 0, reg_aicr = 0;

	/* Read register value of aicr_sel */
	ret = rt9458_i2c_read_byte(RT9458_REG_CTRL6);
	if (ret < 0)
		return ret;
	reg_aicr_sel =
		((ret & RT9458_MASK_AICR_SEL) >> RT9458_SHIFT_AICR_SEL) & 0xFF;

	/* Read register value of aicr */
	ret = rt9458_i2c_read_byte(RT9458_REG_CTRL2);
	if (ret < 0)
		return ret;
	reg_aicr = ((ret & RT9458_MASK_AICR) >> RT9458_SHIFT_AICR) & 0xFF;


	switch (reg_aicr) {
		case 0:
			*aicr = 100;
			break;
		case 1:
			*aicr = reg_aicr_sel ? 700 : 500;
			break;
		case 2:
			*aicr = reg_aicr_sel ? 700 : 1000;
			break;
		case 3:
			*aicr = 0;
			break;
	}

	return ret;
}

int rt_charger_get_mivr(u32 *mivr)
{
	int ret = 0;
	u8 reg_mivr = 0;

	ret = rt9458_i2c_read_byte(RT9458_REG_CTRL8);
	if (ret < 0)
		return ret;
	reg_mivr = ((ret & RT9458_MASK_MIVR) >> RT9458_SHIFT_MIVR) & 0xFF;

	*mivr = rt9458_mivr_table[reg_mivr];

	return ret;
}

int rt_charger_get_battery_voreg(u32 *voreg)
{
	int ret = 0;
	u8 reg_voreg = 0;

	ret = rt9458_i2c_read_byte(RT9458_REG_CTRL3);
	if (ret < 0)
		return ret;

	reg_voreg = ((ret & RT9458_MASK_VOREG) >> RT9458_SHIFT_VOREG) & 0xFF;
	*voreg = rt9458_battery_voreg_table[reg_voreg];		

	return ret;
}

int rt_charger_get_boost_voreg(u32 *voreg)
{
	int ret = 0;
	u8 reg_voreg = 0;

	ret = rt9458_i2c_read_byte(RT9458_REG_CTRL3);
	if (ret < 0)
		return ret;

	reg_voreg = ((ret & RT9458_MASK_VOREG) >> RT9458_SHIFT_VOREG) & 0xFF;
	*voreg = rt9458_boost_voreg_table[reg_voreg];		

	return ret;
}

int rt_charger_is_charging_enable(u8 *enable)
{
	int ret = 0;

	ret = rt9458_i2c_read_byte(RT9458_REG_CTRL7);
	if (ret < 0)
		return ret;

	*enable = ((ret & RT9458_MASK_CHG_EN) >> RT9458_SHIFT_CHG_EN) & 0xFF;

	return ret;
}

int rt_charger_reset_wchdog_timer()
{
	return -ENOTSUPP;	
}

/*[Arima_8100][bozhi_lin] RID003699 OVP warning message to end-user 20170320 begin*/
int rt_charger_get_vinovpi(u32 *vinovpi)
{
	int ret = 0;

	ret = rt9458_i2c_read_byte(RT9458_REG_IDX_IRQ1);
	if (ret < 0)
		return ret;

	//*vinovpi = (ret & RT9458_MASK_VINOVPI) >> RT9458_SHIFT_VINOVPI;
	*vinovpi = g_rt9458_info.vinovpi_flag;

	return ret;
}
/*[Arima_8100][bozhi_lin] 20170320 end*/

/* ========================= */
/* i2c driver function */
/* ========================= */

static void rt9458_parse_dts_cfg(void)
{
	int i = 0, ret = 0;

	for (i = 0; i < RT9458_DTS_CFG_MAX; i++) {
#ifdef CONFIG_OF
		ret = of_property_read_u32(
			g_rt9458_info.i2c->dev.of_node,
			rt9458_dts_cfg_name[i],
			&g_rt9458_info.dts_cfg[i]
		);
		if (ret < 0) {
			pr_info("%s: no property[%s], set default = %d\n",
				__func__, rt9458_dts_cfg_name[i], g_rt9458_info.dts_cfg[i]);
		}
#endif /* CONFIG_OF */

#ifdef RT9458_DBG_MSG_OTH
		pr_info("%s: %s = %d\n", __func__, rt9458_dts_cfg_name[i],
			g_rt9458_info.dts_cfg[i]);
#endif
	}
}

/*[Arima_8100][bozhi_lin] RID003699 OVP warning message to end-user 20170320 begin*/
static int rt9458_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 4 };

	int gpio_val;
	u32 irq_1 = 0, irq_2 = 0, irq_3 = 0;

	sched_setscheduler(current, SCHED_RR, &param);
	do
	{
		gpio_val = __gpio_get_value(g_rt9458_info.irq_gpio);
		//pr_info("[B]%s(%d): g_rt9458_info.irq_gpio=%d\n", __func__, __LINE__, gpio_val);

		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, rt9458_flag != 0);
		rt9458_flag = 0;
		set_current_state(TASK_RUNNING);

		irq_1 = rt9458_i2c_read_byte(RT9458_REG_IDX_IRQ1);
		irq_2 = rt9458_i2c_read_byte(RT9458_REG_IDX_IRQ2);
		irq_3 = rt9458_i2c_read_byte(RT9458_REG_IDX_IRQ3);

		g_rt9458_info.vinovpi_flag = (irq_1 & RT9458_MASK_VINOVPI) >> RT9458_SHIFT_VINOVPI;

		pr_info("[B]%s(%d): irq_1=0x%02x, irq_2=0x%02x, irq_3=0x%02x, vinovpi_flag=%d\n", __func__, __LINE__, irq_1, irq_2, irq_3, g_rt9458_info.vinovpi_flag);

		enable_irq(g_rt9458_info.irq);
		gpio_val = __gpio_get_value(g_rt9458_info.irq_gpio);
		//pr_info("[B]%s(%d): g_rt9458_info.irq_gpio=%d\n", __func__, __LINE__, gpio_val);
	} while (!kthread_should_stop());

	return 0;
}

static irqreturn_t rt9458_eint_interrupt_handler(int irq, void *dev_id)
{
	//pr_info("[B]%s(%d): RT9458 interrupt has been triggered\n", __func__, __LINE__);
	rt9458_flag = 1;
	disable_irq_nosync(g_rt9458_info.irq);
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static int rt9458_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};

	g_rt9458_info.irq_gpio = of_get_named_gpio(g_rt9458_info.i2c->dev.of_node, "rt9458,irq_gpio", 0);
	pr_info("[B]%s(%d): g_rt9458_info.irq_gpio = %d\n", __func__, __LINE__, g_rt9458_info.irq_gpio);

	node = of_find_node_by_name(NULL, "charging_ic_rt9458");
	if (node) {
		of_property_read_u32_array(node,"debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		g_rt9458_info.irq = irq_of_parse_and_map(node, 0);
		pr_info("[B]%s(%d): g_rt9458_info.irq = %d\n", __func__, __LINE__, g_rt9458_info.irq);

		ret = request_irq(g_rt9458_info.irq, rt9458_eint_interrupt_handler,
				IRQF_TRIGGER_FALLING | IRQF_NO_THREAD |
				IRQF_NO_SUSPEND, "RT9458-irq", NULL);
		if (ret > 0)
			pr_info("[B]%s(%d): rt9458 request_irq IRQ LINE NOT AVAILABLE!.", __func__, __LINE__);
	} else {
		pr_info("[B]%s(%d): rt9458 request_irq can not find rt9458 eint device node!.", __func__, __LINE__);
	}
	return 0;
}
/*[Arima_8100][bozhi_lin] 20170320 end*/

static int rt9458_probe(struct i2c_client *i2c,
	const struct i2c_device_id *dev_id)
{
	int ret = 0;

	pr_info("%s: starts\n", __func__);	
	i2c_set_clientdata(i2c, &g_rt9458_info);

	g_rt9458_info.i2c = i2c;

	/* Is HW exist */
	if (!rt9458_is_hw_exist()) {
		pr_info("%s: no rt9458 exists\n", __func__);
		return -ENODEV;	
	}

	/* Parse default configuration from dts */
	rt9458_parse_dts_cfg();

/*[Arima_8100][bozhi_lin] RID003699 OVP warning message to end-user 20170320 begin*/
	thread = kthread_run(rt9458_event_handler, 0, "rt9458");
	if (IS_ERR(thread))
	{
		ret = PTR_ERR(thread);
		pr_err("failed to create kernel thread: %d\n", ret);
	}

	rt9458_irq_registration();
/*[Arima_8100][bozhi_lin] 20170320 end*/

#ifdef CONFIG_RT_REGMAP
	ret = rt9458_register_rt_regmap();
	if (ret < 0)
		return ret;
#endif

	ret = rt_charger_sw_init();
	if (ret < 0)
		pr_info("%s: set  failed\n", __func__);

	rt_charger_dump_register();
	chargin_hw_init_done = KAL_TRUE;
	pr_info("%s: ends\n", __func__);
	return ret;
}


static int rt9458_remove(struct i2c_client *client)
{
	int ret = 0;
	pr_info("%s: starts\n", __func__);

#ifdef CONFIG_RT_REGMAP
	rt_regmap_device_unregister(g_rt9458_info.regmap_dev);
#endif
	return ret;
}

static const struct i2c_device_id rt9458_i2c_id[] = {
	{"rt9458", 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id rt9458_of_match[] = {
	{ .compatible = "mediatek,swithing_charger", },
	{},
};
#else

#define RT9458_BUSNUM 1

static struct i2c_board_info rt9458_i2c_board_info __initdata = {
	I2C_BOARD_INFO("rt9458", RT9458_SALVE_ADDR)	
};  
#endif /* CONFIG_OF */


static struct i2c_driver rt9458_i2c_driver = {
	.driver = {
		.name = "rt9458",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = rt9458_of_match,
#endif
	},
	.probe = rt9458_probe,
	.remove = rt9458_remove,
	.id_table = rt9458_i2c_id,
};


static int __init rt9458_init(void)
{
	int ret = 0;

#ifdef CONFIG_OF
	pr_info("%s: with dts\n", __func__);
#else
	pr_info("%s: without dts\n", __func__);
	i2c_register_board_info(RT9458_BUSNUM, &rt9458_i2c_board_info, 1);
#endif

	ret = i2c_add_driver(&rt9458_i2c_driver);
	if (ret < 0)
		pr_info("%s: register i2c driver failed\n", __func__);

	return ret;
}
module_init(rt9458_init);


static void __exit rt9458_exit(void)
{
	i2c_del_driver(&rt9458_i2c_driver);
	return;
}
module_exit(rt9458_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("ShuFanLee <shufan_lee@richtek.com>");
MODULE_DESCRIPTION("RT9458 Charger Driver");
MODULE_VERSION("1.0.2_MTK");
