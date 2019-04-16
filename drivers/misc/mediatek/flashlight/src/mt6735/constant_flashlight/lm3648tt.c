/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>

/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME ""
#define PK_DBG_NONE(fmt, arg...) do {} while (0)
#define PK_DBG_FUNC(fmt, arg...) pr_err(TAG_NAME "[%s:%d]" fmt "\n", __func__ , __LINE__, ##arg)

#define DEBUG_LEDS_STROBE
#undef CDBG
#ifdef DEBUG_LEDS_STROBE
#define CDBG PK_DBG_FUNC
#else
#define CDBG PK_DBG_NONE
#endif

#define LM3648TT_REG_EN	0x01
#define LM3648TT_REG_IVFM	0x02
#define LM3648TT_REG_F_BRI	0x03
#define LM3648TT_REG_T_BRI	0x05
#define LM3648TT_REG_BOOST	0x07
#define LM3648TT_REG_TIME	0x08
#define LM3648TT_REG_TEMP	0x09

#define LM3648TT_DEF_FLASH_VALUE	41 // flash current = 983.25 mA = (Brightness Code x 23.45 mA) +  21.8 mA
#define LM3648TT_DEF_TORCH_VALUE	17 // torch current = 99.108 mA = (Brightness Code x  5.6  mA) + 3.908 mA
#define LM3648TT_MAX_FLASH_VALUE	63
#define LM3648TT_MAX_TORCH_VALUE	127

#define PCBA_HW_VER0 GPIO58
#define PCBA_HW_VER1 GPIO57
#define PCBA_HW_VER2 GPIO86

#define LM3644TT_CTRL_EN GPIO43
#define LM3644TT_CTRL_TH GPIO42

enum lm3648tt_led_mode {
	MODE_STDBY = 0,
	MODE_IR,
	MODE_TORCH,
	MODE_FLASH,
};

enum lm3648tt_led_en {
	EN_OFF = 0,
	EN_ON = 3,
};

enum sm31_hw_ver {
	HW_NOTSET = 0,
	HW_PDP,
	HW_DP,

	HW_ERROR
};

union lm3648tt_enable_reg {
	struct {
		u8 led_en:2;
		u8 mode:2;
		u8 torch_en:1;
		u8 flash_en:1;
		u8 type:1;
		u8 tx_en:1;
	} st;
	u8 value;
};

union lm3648tt_flash_bri_reg {
	struct {
		u8 bri:6;
		u8 must_be_0:1;
		u8 must_be_1:1;
	} st;
	u8 value;
};

union lm3648tt_torch_bri_reg {
	struct {
		u8 bri:7;
		u8 must_be_1:1;
	} st;
	u8 value;
};

union lm3648tt_timing_reg {
	struct {
		u8 flash:4;
		u8 torch:3;
		u8 rfu:1;
	} st;
	u8 value;
};

union lm3648tt_boost_reg {
	struct {
		u8 curr_limit:1;
		u8 freq_sel:1;
		u8 mode:1;
		u8 short_det:1;
		u8 fru:3;
		u8 sw_reset:1;
	} st;
	u8 value;
};

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */

static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_duty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);

static struct work_struct workTimeOut;

#define LM3648TT_MAX_DUTY	18
static int gIsTorch[LM3648TT_MAX_DUTY] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int gLedDuty[LM3648TT_MAX_DUTY] = { LM3648TT_DEF_TORCH_VALUE, LM3648TT_DEF_FLASH_VALUE, LM3648TT_DEF_FLASH_VALUE, LM3648TT_DEF_FLASH_VALUE, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

static void work_timeOutFunc(struct work_struct *data);

static struct i2c_client *g_i2c_client;

enum sm31_hw_ver g_hw_ver = HW_NOTSET;

struct lm3648tt_platform_data {
	u8 torch_pin_enable;	/* 1: TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1: TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct lm3648tt_chip_data {
	struct i2c_client *client;

	struct lm3648tt_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

static int lm3648tt_write_reg(u8 reg, u8 val)
{
	int ret = 0;
	struct lm3648tt_chip_data *chip = i2c_get_clientdata(g_i2c_client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(g_i2c_client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0) CDBG("failed writing at 0x%02x, ret=%d", reg, ret);
	else CDBG("Writing 0x%X(%d) to ADDR 0x%X, ret=%d", val, val, reg, ret);

	return ret;
}

static s32 lm3648tt_read_reg(u8 reg)
{
	s32 val = 0;
	struct lm3648tt_chip_data *chip = i2c_get_clientdata(g_i2c_client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(g_i2c_client, reg);
	mutex_unlock(&chip->lock);

	CDBG("Read 0x%X(%d) to ADDR 0x%X", val, val, reg);
	return val;
}

static int lm3648tt_chip_enable(u8 en)
{
	int val = 255, ret = 0;

	CDBG("BEGIN EN=%d HW_VER=%d", en, g_hw_ver);
	if (g_hw_ver == HW_PDP) {
		CDBG("END_PDP");
		return 0;
	}

	ret = mt_set_gpio_mode(LM3644TT_CTRL_EN, GPIO_MODE_GPIO);
	if (0 != ret) goto gpio_error;

	ret = mt_set_gpio_dir(LM3644TT_CTRL_EN, GPIO_DIR_OUT);
	if (0 != ret) goto gpio_error;

	if (0 == en) { // OFF
		ret = mt_set_gpio_out(LM3644TT_CTRL_EN, GPIO_OUT_ZERO);
	} else { // ON
		ret = mt_set_gpio_out(LM3644TT_CTRL_EN, GPIO_OUT_ONE);
	}
	if (0 != ret) goto gpio_error;

	val = mt_get_gpio_out(LM3644TT_CTRL_EN);

	CDBG("END CTRL_EN=%d", val);
	return 0;

gpio_error:
	CDBG("END_XX ret=%d", ret);
	return ret;
}

u8 g_flash_reg_current = LM3648TT_DEF_FLASH_VALUE;
u8 g_torch_reg_current = LM3648TT_DEF_TORCH_VALUE;
static void lm3648tt_init_reg(void)
{
	union lm3648tt_flash_bri_reg flash_reg;
	union lm3648tt_torch_bri_reg torch_reg;
	union lm3648tt_timing_reg timing_reg;

	// Timing Configuration Register(0x08)
	// bit07: RFU
	// bit06 ~ bit04: Torch Current Ramp Time
	// bit03 ~ bit00: Flash Time-Out Duration
	timing_reg.st.flash = 0xA; // Flash Time-Out Duration : 600 ms
	timing_reg.st.torch = 0x1; // Torch Current Ramp Time : 1 ms
	timing_reg.st.rfu = 0;
	lm3648tt_write_reg(LM3648TT_REG_TIME, timing_reg.value);

	// LED flash brightness set
	// Flash Brightness Register(0x03)
	// Bit 07 ~ 06: must set '10'
	// Bit 06 ~ 00: LED Flash Brightness Level
	flash_reg.st.must_be_1 = 1;
	flash_reg.st.must_be_0 = 0;
	flash_reg.st.bri = g_flash_reg_current;
	lm3648tt_write_reg(LM3648TT_REG_F_BRI, flash_reg.value);

	// LED torch brightness set
	// Torch Brightness Register(0x05)
	// Bit 07: must set '1'
	// Bit 06 ~ 00: LED Torch Brightness Level
	torch_reg.st.must_be_1 = 1;
	torch_reg.st.bri = g_torch_reg_current;
	lm3648tt_write_reg(LM3648TT_REG_T_BRI, torch_reg.value);
}

// ========================
//  Flash LED Attributes
// ========================
ssize_t lm3648tt_rt_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	union lm3648tt_enable_reg en_reg;

	CDBG("BEGIN");

	en_reg.value = 0;
	en_reg.st.mode = MODE_TORCH;
	en_reg.st.led_en = EN_ON;

	lm3648tt_chip_enable(1);
	lm3648tt_init_reg();
	lm3648tt_write_reg(LM3648TT_REG_EN, en_reg.value);

	CDBG("END");
	return 0;
}

ssize_t lm3648tt_rt_off_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	CDBG("BEGIN");

	lm3648tt_chip_enable(1);
	lm3648tt_write_reg(LM3648TT_REG_EN, 0x00);
	lm3648tt_chip_enable(0);

	CDBG("END");
	return 0;
}

ssize_t lm3648tt_ft_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	union lm3648tt_enable_reg en_reg;

	CDBG("BEGIN");

	en_reg.value = 0;
	en_reg.st.mode = MODE_FLASH;
	en_reg.st.led_en = EN_ON;

	lm3648tt_chip_enable(1);
	lm3648tt_init_reg();
	lm3648tt_write_reg(LM3648TT_REG_EN, en_reg.value);

	CDBG("END");

	return 0;
}

static u8 g_reg_addr;
ssize_t lm3648tt_addr_save(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	CDBG("BEGIN buf=%s", buf);

	if (sscanf(buf, "%hhu", &g_reg_addr) != 1) {
		CDBG("Input value error");
	}
	CDBG("END RegAddr=0x%02X", g_reg_addr);
	return count;
}
ssize_t lm3648tt_addr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	CDBG("BEGIN RegAddr=%d ", g_reg_addr);

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", g_reg_addr);

	CDBG("END");
	return rc;
}

ssize_t lm3648tt_data_save(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	u8 reg_data;
	CDBG("BEGIN buf=%s", buf);

	if (sscanf(buf, "%hhu", &reg_data) != 1) {
		CDBG("Input value error");
	}

	lm3648tt_chip_enable(1);
	lm3648tt_write_reg(g_reg_addr, reg_data);

	CDBG("END RegData = 0x%02X", reg_data);
	return count;
}
ssize_t lm3648tt_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 reg_data = 0;
	int rc = 0;
	CDBG("BEGIN");

	lm3648tt_chip_enable(1);
	reg_data = (u8)lm3648tt_read_reg(g_reg_addr);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", reg_data);

	CDBG("END RegData=0x%x, rc=%d", reg_data, rc);
	return rc;
}

ssize_t lm3648tt_t1_save(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	union lm3648tt_torch_bri_reg torch_reg;
	u8 read_value = 0;
	CDBG("BEGIN buf=%s", buf);

	if (sscanf(buf, "%hhu", &read_value) != 1) {
		CDBG("Input value error");
	}

	torch_reg.st.must_be_1 = 1;
	torch_reg.st.bri = read_value > LM3648TT_MAX_TORCH_VALUE ? LM3648TT_MAX_TORCH_VALUE : read_value;

	lm3648tt_chip_enable(1);
	lm3648tt_write_reg(LM3648TT_REG_T_BRI, torch_reg.value);

	CDBG("END");
	return count;
}
ssize_t lm3648tt_t1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	union lm3648tt_torch_bri_reg reg;
	int rc = 0;
	int mA = 0;
	CDBG("BEGIN");

	lm3648tt_chip_enable(1);
	reg.value = lm3648tt_read_reg(LM3648TT_REG_T_BRI);
	mA = (reg.st.bri * 56 + 39) / 10;
	rc = scnprintf(buf, PAGE_SIZE, "%d [%4dmA] MAX=%d\n", reg.st.bri, mA, LM3648TT_MAX_TORCH_VALUE);

	CDBG("END RegData=%d RegBri=%d rc=%d", reg.value, reg.st.bri, rc);
	return rc;
}

ssize_t lm3648tt_f1_save(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	union lm3648tt_flash_bri_reg flash_reg;
	u8 read_value = 0;
	CDBG("BEGIN buf=%s", buf);

	if (sscanf(buf, "%hhu", &read_value) != 1) {
		CDBG("Input value error");
	}

	flash_reg.st.must_be_1 = 1;
	flash_reg.st.must_be_0 = 0;
	flash_reg.st.bri = read_value > LM3648TT_MAX_FLASH_VALUE ? LM3648TT_MAX_FLASH_VALUE : read_value;

	lm3648tt_chip_enable(1);
	lm3648tt_write_reg(LM3648TT_REG_F_BRI, flash_reg.value);

	CDBG("END");
	return count;
}
ssize_t lm3648tt_f1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	union lm3648tt_flash_bri_reg reg;
	int rc = 0;
	int mA = 0;
	CDBG("BEGIN");

	lm3648tt_chip_enable(1);
	reg.value = lm3648tt_read_reg(LM3648TT_REG_F_BRI);
	mA = (reg.st.bri * 235 + 218) / 10;
	rc = scnprintf(buf, PAGE_SIZE, "%d [%4dmA] MAX=%d\n", reg.st.bri, mA, LM3648TT_MAX_FLASH_VALUE);

	CDBG("END RegData=%d RegBri=%d rc=%d", reg.value, reg.st.bri, rc);
	return rc;
}

// ========================
//  Flash LED Chip Drivers
// ========================

static enum sm31_hw_ver lm3648tt_get_hwver(void)
{
	int val0, val1, val2, val;
	enum sm31_hw_ver hw_ver = HW_NOTSET;

	CDBG("BEGIN");

	val0 = mt_get_gpio_in(PCBA_HW_VER0);
	val1 = mt_get_gpio_in(PCBA_HW_VER1);
	val2 = mt_get_gpio_in(PCBA_HW_VER2);
	CDBG("VAL 2 1 0 = %d %d %d", val2, val1, val0);

	val = val0 + val1 * 2 + val2 * 4;
	if (val == 7) hw_ver = HW_PDP;
	else hw_ver = HW_DP;

	CDBG("END HW_Ver=%d Val=%d", hw_ver, val);
	return hw_ver;
}

static int lm3648tt_chip_init(struct lm3648tt_chip_data *chip)
{
	CDBG("BEGIN");

	g_hw_ver = lm3648tt_get_hwver();

	if (g_hw_ver == HW_PDP) {
		// set gpio pin of torch
		mt_set_gpio_mode(GPIO42, GPIO_MODE_GPIO);
		mt_set_gpio_dir(GPIO42, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO42, GPIO_OUT_ZERO);

		// set gpio pin of strobe
		mt_set_gpio_mode(GPIO43, GPIO_MODE_GPIO);
		mt_set_gpio_dir(GPIO43, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO43, GPIO_OUT_ZERO);
	} else { // DP or newer
		// set gpio pin of torch
		mt_set_gpio_mode(LM3644TT_CTRL_TH, GPIO_MODE_GPIO);
		mt_set_gpio_dir(LM3644TT_CTRL_TH, GPIO_DIR_OUT);
		mt_set_gpio_out(LM3644TT_CTRL_TH, GPIO_OUT_ZERO);
	}

	CDBG("END");
	return 0;
}

static int lm3648tt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lm3648tt_chip_data *chip;
	struct lm3648tt_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	CDBG("BEGIN");

	g_i2c_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		CDBG("i2c functionality check fail.");
		return err;
	}

	chip = kzalloc(sizeof(struct lm3648tt_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		CDBG("Platform data does not exist");
		pdata = kzalloc(sizeof(struct lm3648tt_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (lm3648tt_chip_init(chip) < 0) goto err_chip_init;

	CDBG("END");
	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	CDBG("END_XX");
	return -ENODEV;
}

static int lm3648tt_remove(struct i2c_client *client)
{
	struct lm3648tt_chip_data *chip = i2c_get_clientdata(client);
	CDBG("BEGIN");

	if (chip->no_pdata)	kfree(chip->pdata);
	kfree(chip);

	CDBG("END");
	return 0;
}

#define LM3648TT_NAME "leds-lm3648tt"
static const struct i2c_device_id lm3648tt_id[] = {
	{LM3648TT_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id lm3648tt_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver lm3648tt_i2c_driver = {
	.driver = {
			.name = LM3648TT_NAME,
#ifdef CONFIG_OF
			.of_match_table = lm3648tt_of_match,
#endif
	},
	.probe = lm3648tt_probe,
	.remove = lm3648tt_remove,
	.id_table = lm3648tt_id,
};

struct lm3648tt_platform_data lm3648tt_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_lm3648tt = {
	.type = LM3648TT_NAME,
	.addr = 0x63,
	.platform_data = &lm3648tt_pdata,
};

static int __init lm3648tt_init(void)
{
	int ret = 0;
	CDBG("BEGIN");

	i2c_register_board_info(1, &i2c_lm3648tt, 1);
	ret = i2c_add_driver(&lm3648tt_i2c_driver);

	CDBG("END ret = %d", ret);
	return ret;
}

static void __exit lm3648tt_exit(void)
{
	i2c_del_driver(&lm3648tt_i2c_driver);
}

module_init(lm3648tt_init);
module_exit(lm3648tt_exit);

MODULE_DESCRIPTION("Flash driver for LM3648TT");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

int FL_Enable(void)
{
	union lm3648tt_enable_reg en_reg;
	CDBG("BEGIN");

	en_reg.value = 0;
	en_reg.st.led_en = EN_ON;
	if (gIsTorch[g_duty] == 1) en_reg.st.mode = MODE_TORCH; // reg_value = 0x0B;
	else en_reg.st.mode = MODE_FLASH; // reg_value = 0x0F;

	lm3648tt_chip_enable(1);
	lm3648tt_init_reg();
	lm3648tt_write_reg(LM3648TT_REG_EN, en_reg.value);

	CDBG("END");
	return 0;
}

int FL_Disable(void)
{
	CDBG("BEGIN");

	lm3648tt_chip_enable(1);
	lm3648tt_write_reg(LM3648TT_REG_EN, 0x00);
	lm3648tt_chip_enable(0);

	CDBG("END");
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	CDBG("BEGIN Duty=%d", duty);
	if (duty > LM3648TT_MAX_DUTY - 1) duty = LM3648TT_MAX_DUTY - 1;
	// if (duty < 0) duty = 0; // It's impossible for duty < 0 since its type is kal_uint32

	g_duty = duty;

	if (gIsTorch[g_duty] == 1) {
		g_torch_reg_current = gLedDuty[g_duty];
	} else {
		g_flash_reg_current = gLedDuty[g_duty];
	}

	CDBG("END");
	return 0;
}

int FL_Init(void)
{
	CDBG("BEGIN");
	FL_Disable();
	CDBG("END");
	return 0;
}
int FL_Uninit(void)
{
	CDBG("BEGIN");
	FL_Disable();
	CDBG("END");
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	CDBG("BEGIN");
	FL_Disable();
	CDBG("END");
}

enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}

static int lm3648tt_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	CDBG("BEGIN");

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));

	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		CDBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d", (int)arg);
		g_timeOutTimeMs = arg;
		break;

	case FLASH_IOC_SET_DUTY:
		CDBG("FLASHLIGHT_DUTY: %d", (int)arg);
		FL_dim_duty(arg);
		break;

	case FLASH_IOC_SET_STEP:
		CDBG("FLASH_IOC_SET_STEP: %d", (int)arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		CDBG("FLASHLIGHT_ONOFF: %d", (int)arg);
		if (arg == 1) {
			int s, ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;

	default:
		CDBG(" No such command");
		i4RetValue = -EPERM;
		break;
	}

	CDBG("END");
	return i4RetValue;
}

static int lm3648tt_open(void *pArg)
{
	int i4RetValue = 0;

	CDBG("BEGIN");

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}

	spin_lock_irq(&g_strobeSMPLock);

	if (strobe_Res) {
		CDBG("Busy!");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}

	spin_unlock_irq(&g_strobeSMPLock);

	CDBG("END");
	return i4RetValue;
}


static int lm3648tt_release(void *pArg)
{
	CDBG("BEGIN");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	CDBG("END");
	return 0;
}


FLASHLIGHT_FUNCTION_STRUCT lm3648tt_func = {
	lm3648tt_open,
	lm3648tt_release,
	lm3648tt_ioctl
};

MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	CDBG("BEGIN");
	if (pfFunc != NULL)	*pfFunc = &lm3648tt_func;

	CDBG("END");
	return 0;
}
EXPORT_SYMBOL(constantFlashlightInit);

/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{
	CDBG("BEGIN");

	CDBG("END");
	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
