/* Include from MTK */
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_hw.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/mtk_thermal_typedefs.h>

#ifdef CONFIG_MTK_LEGACY
#include <mt-plat/mt_gpio.h>
#include <cust_gpio_usage.h>
#endif

#include <linux/delay.h>
#include <linux/reboot.h>

#include "rt9458.h"


/* Necessary functions for integrating with MTK */
/* All of them are copied from original source code of MTK */

#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0

#if defined(GPIO_PWR_AVAIL_WLC)
/* K.S.? */
kal_uint32 wireless_charger_gpio_number = GPIO_PWR_AVAIL_WLC;
#else
kal_uint32 wireless_charger_gpio_number = 0;
#endif

#endif /* MTK_WIRELESS_CHARGER_SUPPORT */

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;
#endif

kal_bool charging_type_det_done = KAL_TRUE;

const kal_uint32 VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	    BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V,
	    BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V,
	    BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	    BATTERY_VOLT_10_500000_V
};

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT

#ifndef CUST_GPIO_VIN_SEL
#define CUST_GPIO_VIN_SEL 18
#endif

DISO_IRQ_Data DISO_IRQ;
int g_diso_state = 0;
int vin_sel_gpio_number = (CUST_GPIO_VIN_SEL | 0x80000000);
static char *DISO_state_s[8] = {
	"IDLE",
	"OTG_ONLY",
	"USB_ONLY",
	"USB_WITH_OTG",
	"DC_ONLY",
	"DC_WITH_OTG",
	"DC_WITH_USB",
	"DC_USB_OTG",
};
#endif /*CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT */

#ifdef CONFIG_MTK_BIF_SUPPORT
static int bif_inited;
#endif
static kal_uint32 charging_error;
kal_uint32 charging_value_to_parameter(const kal_uint32 *parameter, const kal_uint32 array_size,
				       const kal_uint32 val)
{
	if (val < array_size) {
		return parameter[val];
	} else {
		battery_log(BAT_LOG_CRTI, "Can't find the parameter \r\n");
		return parameter[0];
	}
}

kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size,
				       const kal_uint32 val)
{
	kal_uint32 i;

	battery_log(BAT_LOG_FULL, "array_size = %d \r\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	battery_log(BAT_LOG_CRTI, "NO register value match \r\n");
	/* TODO: ASSERT(0);    // not find the value */
	return 0;
}

static kal_uint32 bmt_find_closest_level(const kal_uint32 *pList, kal_uint32 number,
					 kal_uint32 level)
{
	kal_uint32 i;
	kal_uint32 max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		for (i = (number - 1); i != 0; i--) {	/* max value in the last element */
			if (pList[i] <= level) {
				battery_log(2, "zzf_%d<=%d     i=%d\n", pList[i], level, i);
				return pList[i];
			}
		}

		battery_log(BAT_LOG_CRTI, "Can't find closest level \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++) {	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];
		}

		battery_log(BAT_LOG_CRTI, "Can't find closest level \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}

static kal_uint32 is_chr_det(void)
{
	kal_uint32 val = 0;
	#if 1
	val = pmic_get_register_value(PMIC_RGS_CHRDET);
	#else
	val = pmic_get_register_value(MT6351_PMIC_RGS_CHRDET);
	#endif
	battery_log(BAT_LOG_CRTI, "[is_chr_det] %d\n", val);

	return val;
}

#ifdef CONFIG_MTK_BIF_SUPPORT
/* BIF related functions*/
#define BC (0x400)
#define SDA (0x600)
#define ERA (0x100)
#define WRA (0x200)
#define RRA (0x300)
#define WD  (0x000)
/*bus commands*/
#define BUSRESET (0x00)
#define RBL2 (0x22)
#define RBL4 (0x24)

/*BIF slave address*/
#define MW3790 (0x00)
#define MW3790_VBAT (0x0114)
#define MW3790_TBAT (0x0193)
void bif_reset_irq(void)
{
	kal_uint32 reg_val = 0;
	kal_uint32 loop_i = 0;

	pmic_set_register_value(PMIC_BIF_IRQ_CLR, 1);
	reg_val = 0;
	do {
		reg_val = pmic_get_register_value(PMIC_BIF_IRQ);

		if (loop_i++ > 50) {
			battery_log(BAT_LOG_CRTI, "[BIF][reset irq]failed.PMIC_BIF_IRQ 0x%x %d\n",
				    reg_val, loop_i);
			break;
		}
	} while (reg_val != 0);
	pmic_set_register_value(PMIC_BIF_IRQ_CLR, 0);
}

void bif_waitfor_slave(void)
{
	kal_uint32 reg_val = 0;
	int loop_i = 0;

	do {
		reg_val = pmic_get_register_value(PMIC_BIF_IRQ);

		if (loop_i++ > 50) {
			battery_log(BAT_LOG_CRTI,
				    "[BIF][waitfor_slave] failed. PMIC_BIF_IRQ=0x%x, loop=%d\n",
				    reg_val, loop_i);
			break;
		}
	} while (reg_val == 0);

	if (reg_val == 1)
		battery_log(BAT_LOG_FULL, "[BIF][waitfor_slave]OK at loop=%d.\n", loop_i);

}

int bif_powerup_slave(void)
{
	int bat_lost = 0;
	int total_valid = 0;
	int timeout = 0;
	int loop_i = 0;

	do {
		battery_log(BAT_LOG_FULL, "[BIF][powerup_slave] set BIF power up register\n");
		pmic_set_register_value(PMIC_BIF_POWER_UP, 1);

		battery_log(BAT_LOG_FULL, "[BIF][powerup_slave] trigger BIF module\n");
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 1);

		udelay(10);

		bif_waitfor_slave();

		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 0);

		pmic_set_register_value(PMIC_BIF_POWER_UP, 0);

		/*check_bat_lost(); what to do with this? */
		bat_lost = pmic_get_register_value(PMIC_BIF_BAT_LOST);
		total_valid = pmic_get_register_value(PMIC_BIF_TOTAL_VALID);
		timeout = pmic_get_register_value(PMIC_BIF_TIMEOUT);

		if (loop_i < 5) {
			loop_i++;
		} else {
			battery_log(BAT_LOG_CRTI, "[BIF][powerup_slave]Failed at loop=%d", loop_i);
			break;
		}
	} while (bat_lost == 1 || total_valid == 1 || timeout == 1);
	if (loop_i < 5) {
		battery_log(BAT_LOG_FULL, "[BIF][powerup_slave]OK at loop=%d", loop_i);
		bif_reset_irq();
		return 1;
	}

	return -1;
}

void bif_set_cmd(int bif_cmd[], int bif_cmd_len)
{
	int i = 0;
	int con_index = 0;
	kal_uint32 ret = 0;

	for (i = 0; i < bif_cmd_len; i++) {
		ret = pmic_config_interface(MT6351_PMIC_BIF_COMMAND_0_ADDR + con_index, bif_cmd[i], MT6351_PMIC_BIF_COMMAND_0_MASK, MT6351_PMIC_BIF_COMMAND_0_SHIFT);
		con_index += 0x2;
	}
}


int bif_reset_slave(void)
{
	kal_uint32 ret = 0;
	int bat_lost = 0;
	int total_valid = 0;
	int timeout = 0;
	int bif_cmd[1] = { 0 };
	int loop_i = 0;

	/*set command sequence */
	bif_cmd[0] = BC | BUSRESET;
	bif_set_cmd(bif_cmd, 1);

	do {
		/*command setting : 1 write command */
		ret = pmic_set_register_value(PMIC_BIF_TRASFER_NUM, 1);
		ret = pmic_set_register_value(PMIC_BIF_COMMAND_TYPE, 0);

		/*Command set trigger */
		ret = pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 1);

		udelay(10);
		/*Command sent; wait for slave */
		bif_waitfor_slave();

		/*Command clear trigger */
		ret = pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 0);
		/*check transaction completeness */
		bat_lost = pmic_get_register_value(PMIC_BIF_BAT_LOST);
		total_valid = pmic_get_register_value(PMIC_BIF_TOTAL_VALID);
		timeout = pmic_get_register_value(PMIC_BIF_TIMEOUT);

		if (loop_i < 50)
			loop_i++;
		else {
			battery_log(BAT_LOG_CRTI, "[BIF][bif_reset_slave]Failed at loop=%d",
				    loop_i);
			break;
		}
	} while (bat_lost == 1 || total_valid == 1 || timeout == 1);

	if (loop_i < 50) {
		battery_log(BAT_LOG_FULL, "[BIF][bif_reset_slave]OK at loop=%d", loop_i);
		/*reset BIF_IRQ */
		bif_reset_irq();
		return 1;
	}
	return -1;
}

/*BIF WRITE 8 transaction*/
int bif_write8(int addr, int *data)
{
	int ret = 1;
	int era, wra;
	int bif_cmd[4] = { 0, 0, 0, 0};
	int loop_i = 0;
	int bat_lost = 0;
	int total_valid = 0;
	int timeout = 0;

	era = (addr & 0xFF00) >> 8;
	wra = addr & 0x00FF;
	battery_log(BAT_LOG_FULL, "[BIF][bif_write8]ERA=%x, WRA=%x\n", era, wra);
	/*set command sequence */
	bif_cmd[0] = SDA | MW3790;
	bif_cmd[1] = ERA | era;	/*[15:8] */
	bif_cmd[2] = WRA | wra;	/*[ 7:0] */
	bif_cmd[3] = WD  | (*data & 0xFF);	/*data*/


	bif_set_cmd(bif_cmd, 4);
	do {
		/*command setting : 4 transactions for 1 byte write command(0) */
		pmic_set_register_value(PMIC_BIF_TRASFER_NUM, 4);
		pmic_set_register_value(PMIC_BIF_COMMAND_TYPE, 0);

		/*Command set trigger */
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 1);

		udelay(200);
		/*Command sent; wait for slave */
		bif_waitfor_slave();

		/*Command clear trigger */
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 0);
		/*check transaction completeness */
		bat_lost = pmic_get_register_value(PMIC_BIF_BAT_LOST);
		total_valid = pmic_get_register_value(PMIC_BIF_TOTAL_VALID);
		timeout = pmic_get_register_value(PMIC_BIF_TIMEOUT);

		if (loop_i <= 50)
			loop_i++;
		else {
			battery_log(BAT_LOG_CRTI,
		"[BIF][bif_write8] Failed. bat_lost = %d, timeout = %d, totoal_valid = %d\n",
		bat_lost, timeout, total_valid);
			ret = -1;
			break;
		}
	} while (bat_lost == 1 || total_valid == 1 || timeout == 1);

	if (ret == 1)
		battery_log(BAT_LOG_FULL, "[BIF][bif_write8] OK for %d loop(s)\n", loop_i);
	else
		battery_log(BAT_LOG_CRTI, "[BIF][bif_write8] Failed for %d loop(s)\n", loop_i);

	/*reset BIF_IRQ */
	bif_reset_irq();

	return ret;
}

/*BIF READ 8 transaction*/
int bif_read8(int addr, int *data)
{
	int ret = 1;
	int era, rra;
	int val = -1;
	int bif_cmd[3] = { 0, 0, 0 };
	int loop_i = 0;
	int bat_lost = 0;
	int total_valid = 0;
	int timeout = 0;

	battery_log(BAT_LOG_FULL, "[BIF][READ8]\n");

	era = (addr & 0xFF00) >> 8;
	rra = addr & 0x00FF;
	battery_log(BAT_LOG_FULL, "[BIF][bif_read8]ERA=%x, RRA=%x\n", era, rra);
	/*set command sequence */
	bif_cmd[0] = SDA | MW3790;
	bif_cmd[1] = ERA | era;	/*[15:8] */
	bif_cmd[2] = RRA | rra;	/*[ 7:0] */

	bif_set_cmd(bif_cmd, 3);
	do {
		/*command setting : 3 transactions for 1 byte read command(1) */
		pmic_set_register_value(PMIC_BIF_TRASFER_NUM, 3);
		pmic_set_register_value(PMIC_BIF_COMMAND_TYPE, 1);
		pmic_set_register_value(PMIC_BIF_READ_EXPECT_NUM, 1);

		/*Command set trigger */
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 1);

		udelay(200);
		/*Command sent; wait for slave */
		bif_waitfor_slave();

		/*Command clear trigger */
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 0);
		/*check transaction completeness */
		bat_lost = pmic_get_register_value(PMIC_BIF_BAT_LOST);
		total_valid = pmic_get_register_value(PMIC_BIF_TOTAL_VALID);
		timeout = pmic_get_register_value(PMIC_BIF_TIMEOUT);

		if (loop_i <= 50)
			loop_i++;
		else {
			battery_log(BAT_LOG_CRTI,
		"[BIF][bif_read16] Failed. bat_lost = %d, timeout = %d, totoal_valid = %d\n",
		bat_lost, timeout, total_valid);
			ret = -1;
			break;
		}
	} while (bat_lost == 1 || total_valid == 1 || timeout == 1);

	/*Read data */
	if (ret == 1) {
		val = pmic_get_register_value(PMIC_BIF_DATA_0);
		battery_log(BAT_LOG_FULL, "[BIF][bif_read8] OK d0=0x%x, for %d loop(s)\n",
			    val, loop_i);
	} else
		battery_log(BAT_LOG_CRTI, "[BIF][bif_read8] Failed for %d loop(s)\n", loop_i);

	/*reset BIF_IRQ */
	bif_reset_irq();

	*data = val;
	return ret;
}

/*bif read 16 transaction*/
int bif_read16(int addr)
{
	int ret = 1;
	int era, rra;
	int val = -1;
	int bif_cmd[4] = { 0, 0, 0, 0 };
	int loop_i = 0;
	int bat_lost = 0;
	int total_valid = 0;
	int timeout = 0;

	battery_log(BAT_LOG_FULL, "[BIF][READ]\n");

	era = (addr & 0xFF00) >> 8;
	rra = addr & 0x00FF;
	battery_log(BAT_LOG_FULL, "[BIF][bif_read16]ERA=%x, RRA=%x\n", era, rra);
	/*set command sequence */
	bif_cmd[0] = SDA | MW3790;
	bif_cmd[1] = BC | RBL2;	/* read back 2 bytes */
	bif_cmd[2] = ERA | era;	/*[15:8] */
	bif_cmd[3] = RRA | rra;	/*[ 7:0] */

	bif_set_cmd(bif_cmd, 4);
	do {
		/*command setting : 4 transactions for 2 byte read command(1) */
		pmic_set_register_value(PMIC_BIF_TRASFER_NUM, 4);
		pmic_set_register_value(PMIC_BIF_COMMAND_TYPE, 1);
		pmic_set_register_value(PMIC_BIF_READ_EXPECT_NUM, 2);

		/*Command set trigger */
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 1);

		udelay(200);
		/*Command sent; wait for slave */
		bif_waitfor_slave();

		/*Command clear trigger */
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 0);
		/*check transaction completeness */
		bat_lost = pmic_get_register_value(PMIC_BIF_BAT_LOST);
		total_valid = pmic_get_register_value(PMIC_BIF_TOTAL_VALID);
		timeout = pmic_get_register_value(PMIC_BIF_TIMEOUT);

		if (loop_i <= 50)
			loop_i++;
		else {
			battery_log(BAT_LOG_CRTI,
		"[BIF][bif_read16] Failed. bat_lost = %d, timeout = %d, totoal_valid = %d\n",
		bat_lost, timeout, total_valid);
			ret = -1;
			break;
		}
	} while (bat_lost == 1 || total_valid == 1 || timeout == 1);

	/*Read data */
	if (ret == 1) {
		int d0, d1;

		d0 = pmic_get_register_value(PMIC_BIF_DATA_0);
		d1 = pmic_get_register_value(PMIC_BIF_DATA_1);
		val = 0xFF & d1;
		val = val | ((d0 & 0xFF) << 8);
		battery_log(BAT_LOG_FULL, "[BIF][bif_read16] OK d0=0x%x, d1=0x%x for %d loop(s)\n",
			    d0, d1, loop_i);
	}

	/*reset BIF_IRQ */
	bif_reset_irq();


	return val;
}

void bif_ADC_enable(void){
	int reg = 0x18;

	bif_write8(0x0110, &reg);
	mdelay(50);
	
	reg = 0x98;
	bif_write8(0x0110, &reg);
	mdelay(50);

}

/* BIF init function called only at the first time*/
int bif_init(void)
{
	int pwr, rst;
	/*disable BIF interrupt */
	pmic_set_register_value(PMIC_INT_CON0_CLR, 0x4000);
	/*enable BIF clock */
	pmic_set_register_value(PMIC_TOP_CKPDN_CON2_CLR, 0x0070);

	/*enable HT protection */
	pmic_set_register_value(PMIC_RG_BATON_HT_EN, 1);

	/*change to HW control mode*/
	/*pmic_set_register_value(MT6351_PMIC_RG_VBIF28_ON_CTRL, 0);*/
	/*pmic_set_register_value(MT6351_PMIC_RG_VBIF28_EN, 1);*/
	mdelay(50);

	/*Enable RX filter function */
#if 1
	pmic_set_register_value(PMIC_BIF_RX_DEG_EN, 0x8000);
	pmic_set_register_value(PMIC_BIF_RX_DEG_WND, 0x17);
#else
	pmic_set_register_value(MT6351_PMIC_BIF_RX_DEG_EN, 0x8000);
	pmic_set_register_value(MT6351_PMIC_BIF_RX_DEG_WND, 0x17);
#endif
	pmic_set_register_value(PMIC_RG_BATON_EN, 0x1);
	pmic_set_register_value(PMIC_BATON_TDET_EN, 0x1);
	pmic_set_register_value(PMIC_RG_BATON_HT_EN_DLY_TIME, 0x1);


	/*wake up BIF slave */
	pwr = bif_powerup_slave();
	mdelay(10);
	rst = bif_reset_slave();

	/*pmic_set_register_value(MT6351_PMIC_RG_VBIF28_ON_CTRL, 1);*/
	mdelay(50);

	battery_log(BAT_LOG_CRTI, "[BQ25896][BIF_init] done.");

	if (pwr + rst == 2)
		return 1;

	return -1;
}
#endif

/* The following functions are for chr_control_interface */

static int charging_hw_init(void *data)
{
	int ret = 0;	

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	if (wireless_charger_gpio_number != 0) {
#ifdef CONFIG_MTK_LEGACY
		mt_set_gpio_mode(wireless_charger_gpio_number, 0);	/* 0:GPIO mode */
		mt_set_gpio_dir(wireless_charger_gpio_number, 0);	/* 0: input, 1: output */
#else
/* K.S. way here */
#endif
	}
#endif

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#ifdef CONFIG_MTK_LEGACY
	mt_set_gpio_mode(vin_sel_gpio_number, 0);	/* 0:GPIO mode */
	mt_set_gpio_dir(vin_sel_gpio_number, 0);	/* 0: input, 1: output */
#else
/* K.S. way here */
#endif
#endif

	return ret;
}

static int charging_dump_register(void *data)
{
	int ret = 0;

	ret = rt_charger_dump_register();

	return ret;
}

static int charging_enable(void *data)
{
	int ret = 0;

	u8 enable = *((u8 *)data);
	ret = rt_charger_enable_charging(enable);

	return ret;
}

static int charging_set_cv_voltage(void *data)
{
	int ret = 0;

	/* MTK's voltage unit : uV */
	/* Our voltage unit : mV */
	u32 voreg = *((u32 *)data);
	voreg /= 1000;

	ret = rt_charger_set_battery_voreg(voreg);
	if (ret < 0)
		return ret;

	ret = rt_charger_get_battery_voreg(&voreg);
	if (ret < 0)
		return ret;

	/* Set battery's voreg */
	/* This function is defined in switching_charger.c */
#if 0
	voreg *= 1000;
	battery_set_cv_voltage(voreg);
#endif	


	return ret;
}

static int charging_get_current(void *data)
{
	int ret = 0;
	u32 ichg = 0;

	ret = rt_charger_get_ichg(&ichg);

	/* MTK's current unit : 10uA */
	/* Our current unit : mA */
	ichg *= 100;
	*((u32 *)data) = ichg;
	
	return ret;
}

static int charging_set_current(void *data)
{
	int ret = 0;

	/* MTK's current unit : 10uA */
	/* Our current unit : mA */
	u32 ichg = *((u32 *)data);
	ichg /= 100;

	ret = rt_charger_set_ichg(ichg); 

	return ret;
}

static int charging_set_input_current(void *data)
{
	int ret = 0;

	/* MTK's current unit : 10uA */
	/* Our current unit : mA */
	u32 aicr = *((u32 *)data);
	aicr /= 100;

	ret = rt_charger_set_aicr(aicr);

	return ret;
}

static int charging_get_charging_status(void *data)
{
	int ret = 0;
	enum rt_charging_status chg_stat = RT_CHG_STATUS_READY;

	ret = rt_charger_get_charging_status(&chg_stat);
	
	/* Return is charging done or not */
	switch (chg_stat) {
		case RT_CHG_STATUS_READY:
		case RT_CHG_STATUS_PROGRESS:
		case RT_CHG_STATUS_FAULT:
			*((u32 *)data) = 0;
			break;
		case RT_CHG_STATUS_DONE:
			*((u32 *)data) = 1;
			break;
		default:
			*((u32 *)data) = 0;
			break;
	}

	return ret;
}

static int charging_reset_watch_dog_timer(void *data)
{
	int ret = 0;

	ret = rt_charger_reset_wchdog_timer();

	return ret;
}

static int charging_set_hv_threshold(void *data)
{
	int ret = 0;
	kal_uint32 set_hv_voltage;
	kal_uint32 array_size;
	kal_uint16 register_value;
	kal_uint32 voltage = *(kal_uint32 *)(data);

	array_size = GETARRAYNUM(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size,
		set_hv_voltage);
#if 1
	pmic_set_register_value(PMIC_RG_VCDT_HV_VTH, register_value);
/*[Arima_8100][bozhi_lin] set charger detection LV threshold to 4.2V 20170415 begin*/
	pmic_set_register_value(PMIC_RG_VCDT_LV_VTH, 0);
/*[Arima_8100][bozhi_lin] 20170415 end*/

#else
	pmic_set_register_value(MT6351_PMIC_RG_VCDT_HV_VTH, register_value);
#endif

	return ret;
}

static int charging_get_hv_status(void *data)
{
	int ret = 0;

#if 1
	*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
#else
	*(kal_bool *) (data) = pmic_get_register_value(MT6351_PMIC_RGS_VCDT_HV_DET);
#endif

	return ret;
}

static int charging_get_battery_status(void *data)
{
	int ret = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *)(data) = 0;
	battery_log(BAT_LOG_CRTI, "no battery for evb\n");
#else
	kal_uint32 val = 0;
#if 1
	val = pmic_get_register_value(PMIC_BATON_TDET_EN);
	battery_log(BAT_LOG_FULL, "[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
		pmic_set_register_value(PMIC_BATON_TDET_EN, 1);
		pmic_set_register_value(PMIC_RG_BATON_EN, 1);
		*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool *) (data) = KAL_FALSE;
	}
#else
	val = pmic_get_register_value(MT6351_PMIC_BATON_TDET_EN);
	battery_log(BAT_LOG_FULL, "[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
		pmic_set_register_value(MT6351_PMIC_BATON_TDET_EN, 1);
		pmic_set_register_value(MT6351_PMIC_RG_BATON_EN, 1);
		*(kal_bool *) (data) = pmic_get_register_value(MT6351_PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool *) (data) = KAL_FALSE;
	}
#endif
#endif

	return ret;
}

static int charging_get_charger_det_status(void *data)
{
	int ret = 0;

#if defined(CONFIG_MTK_FPGA)
	*(kal_bool *)(data) = 1;
	battery_log(BAT_LOG_CRTI, "chr exist for fpga\n");
#else
#if 1
	*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_CHRDET);
#else
	*(kal_bool *) (data) = pmic_get_register_value(MT6351_PMIC_RGS_CHRDET);
#endif
#endif

	return ret;
}

static int charging_get_charger_type(void *data)
{
	int ret = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	int wireless_state = 0;

	if (wireless_charger_gpio_number != 0) {
#ifdef CONFIG_MTK_LEGACY
		wireless_state = mt_get_gpio_in(wireless_charger_gpio_number);
#else
		/* K.S. way here*/
#endif
		if (wireless_state == WIRELESS_CHARGER_EXIST_STATE) {
			*(CHARGER_TYPE *) (data) = WIRELESS_CHARGER;
			battery_log(BAT_LOG_CRTI, "WIRELESS_CHARGER!\n");
			return ret;
		}
	} else {
		battery_log(BAT_LOG_CRTI, "wireless_charger_gpio_number=%d\n",
			wireless_charger_gpio_number);
	}

	if (g_charger_type != CHARGER_UNKNOWN &&
		g_charger_type != WIRELESS_CHARGER) {
		*(CHARGER_TYPE *) (data) = g_charger_type;
		battery_log(BAT_LOG_CRTI, "return %d!\n", g_charger_type);
		return ret;
	}
#endif /* MTK_WIRELESS_CHARGER_SUPPORT */

	if (is_chr_det() == 0) {
		g_charger_type = CHARGER_UNKNOWN;
		*(CHARGER_TYPE *) (data) = CHARGER_UNKNOWN;
		battery_log(BAT_LOG_CRTI, "%s: return CHARGER_UNKNOWN\n", __func__);
		return ret;
	}

	charging_type_det_done = KAL_FALSE;
	*(CHARGER_TYPE *) (data) = hw_charging_get_charger_type();
	charging_type_det_done = KAL_TRUE;
	g_charger_type = *(CHARGER_TYPE *) (data);
#endif

	return ret;
}

static int charging_get_is_pcm_timer_trigger(void *data)
{
	int ret = 0;

#if 1
	*(kal_bool *)(data) = KAL_FALSE;
#else
	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *)(data) = KAL_TRUE;
	else
		*(kal_bool *)(data) = KAL_FALSE;

	battery_log(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n",
		slp_get_wake_reason());
#endif

	return ret;
}

static int charging_set_platform_reset(void *data)
{
	int ret = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "%s\n", __func__);
	kernel_restart("battery service reboot system");
#endif

	return ret;
}

static int charging_get_platform_boot_mode(void *data)
{
	int ret = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	*(kal_uint32 *)(data) = get_boot_mode();
	battery_log(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
#endif

	return ret;
}

static int charging_set_power_off(void *data)
{
	int ret = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	/* Added dump_stack to see who the caller is */
	dump_stack();
	battery_log(BAT_LOG_CRTI, "%s\n", __func__);
	kernel_power_off();
#endif

	return ret;
}

static int charging_get_power_source(void *data)
{
	int ret = 0;

#if 0 
/* #if defined(MTK_POWER_EXT_DETECT) */
	if (MT_BOARD_PHONE == mt_get_board_type())
		*(kal_bool *) data = KAL_FALSE;
	else
		*(kal_bool *) data = KAL_TRUE;
#else
	*(kal_bool *)data = KAL_FALSE;
#endif

	return ret;
}

static int charging_get_csdac_full_flag(void *data)
{
	return -ENOTSUPP;	
}

static int charging_set_ta_current_pattern(void *data)
{
	int ret = 0;
	u8 is_pump_up = *((u8 *)data);

	ret = rt_charger_set_ta_current_pattern(is_pump_up);

	return ret;
}

static int charging_set_hiz_swchr(void *data)
{
	int ret = 0;
	u32 mivr = 0;
	u32 en = *((u32 *)data);

	if (en == 1)
		mivr = 15300;
	else
		mivr = 4500;

	ret = rt_charger_set_mivr(mivr);

	return ret;
}

static int charging_set_error_state(void *data)
{
	int ret = 0;

	charging_error = *(kal_uint32 *)(data);
	charging_set_hiz_swchr(&charging_error);

	return ret;
}

static int charging_diso_init(void *data)
{
	int ret = 0;

#if defined(MTK_DUAL_INPUT_CHARGER_SUPPORT)
	struct device_node *node;
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *) data;

	int ret_irq;
	/* Initialization DISO Struct */
	pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;

	pDISO_data->diso_state.pre_otg_state = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vdc_state = DISO_OFFLINE;

	pDISO_data->chr_get_diso_state = KAL_FALSE;

	pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;

	/* Initial AuxADC IRQ */
	DISO_IRQ.vdc_measure_channel.number = AP_AUXADC_DISO_VDC_CHANNEL;
	DISO_IRQ.vusb_measure_channel.number = AP_AUXADC_DISO_VUSB_CHANNEL;
	DISO_IRQ.vdc_measure_channel.period = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vusb_measure_channel.period = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vdc_measure_channel.debounce = AUXADC_CHANNEL_DEBOUNCE;
	DISO_IRQ.vusb_measure_channel.debounce = AUXADC_CHANNEL_DEBOUNCE;

	/* use default threshold voltage, if use high voltage,maybe refine */
	DISO_IRQ.vusb_measure_channel.falling_threshold = VBUS_MIN_VOLTAGE / 1000;
	DISO_IRQ.vdc_measure_channel.falling_threshold = VDC_MIN_VOLTAGE / 1000;
	DISO_IRQ.vusb_measure_channel.rising_threshold = VBUS_MIN_VOLTAGE / 1000;
	DISO_IRQ.vdc_measure_channel.rising_threshold = VDC_MIN_VOLTAGE / 1000;

	node = of_find_compatible_node(NULL, NULL, "mediatek,AUXADC");
	if (!node) {
		battery_log(BAT_LOG_CRTI, "[diso_adc]: of_find_compatible_node failed!!\n");
	} else {
		pDISO_data->irq_line_number = irq_of_parse_and_map(node, 0);
		battery_log(BAT_LOG_FULL, "[diso_adc]: IRQ Number: 0x%x\n",
			    pDISO_data->irq_line_number);
	}

	mt_irq_set_sens(pDISO_data->irq_line_number, MT_EDGE_SENSITIVE);
	mt_irq_set_polarity(pDISO_data->irq_line_number, MT_POLARITY_LOW);

	ret_irq = request_threaded_irq(pDISO_data->irq_line_number, diso_auxadc_irq_handler,
				   pDISO_data->irq_callback_func, IRQF_ONESHOT, "DISO_ADC_IRQ",
				   NULL);

	if (ret_irq) {
		battery_log(BAT_LOG_CRTI, "[diso_adc]: request_irq failed.\n");
	} else {
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		battery_log(BAT_LOG_FULL, "[diso_adc]: diso_init success.\n");
	}

#if defined(MTK_DISCRETE_SWITCH) && defined(MTK_DSC_USE_EINT)
	battery_log(BAT_LOG_CRTI, "[diso_eint]vdc eint irq registitation\n");
	mt_eint_set_hw_debounce(CUST_EINT_VDC_NUM, CUST_EINT_VDC_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_VDC_NUM, CUST_EINTF_TRIGGER_LOW, vdc_eint_handler, 0);
	mt_eint_mask(CUST_EINT_VDC_NUM);
#endif

#endif /* MTK_DUAL_INPUT_CHARGER_SUPPORT */

	return ret;	
}

static int charging_get_diso_state(void *data)
{
	int ret = 0;

#if defined(MTK_DUAL_INPUT_CHARGER_SUPPORT)
	int diso_state = 0x0;
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *) data;

	_get_diso_interrupt_state();
	diso_state = g_diso_state;
	battery_log(BAT_LOG_FULL, "[do_chrdet_int_task] current diso state is %s!\n",
		    DISO_state_s[diso_state]);
	if (((diso_state >> 1) & 0x3) != 0x0) {
		switch (diso_state) {
		case USB_ONLY:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
#ifdef MTK_DISCRETE_SWITCH
#ifdef MTK_DSC_USE_EINT
			mt_eint_unmask(CUST_EINT_VDC_NUM);
#else
			set_vdc_auxadc_irq(DISO_IRQ_ENABLE, 1);
#endif
#endif
			pDISO_data->diso_state.cur_vusb_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_ONLY:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_RISING);
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_WITH_USB:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
			pDISO_data->diso_state.cur_vusb_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_WITH_OTG:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_ONLINE;
			break;
		default:	/* OTG only also can trigger vcdt IRQ */
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_ONLINE;
			battery_log(BAT_LOG_FULL, " switch load vcdt irq triggerd by OTG Boost!\n");
			break;	/* OTG plugin no need battery sync action */
		}
	}

	if (DISO_ONLINE == pDISO_data->diso_state.cur_vdc_state)
		pDISO_data->hv_voltage = VDC_MAX_VOLTAGE;
	else
		pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;
#endif

	return ret;
	
}

static int charging_set_vindpm(void *data)
{
	/* MTK's mivr setting (It's actually register val from TI's BQ25896 */
	/* SWITCH_CHR_VINDPM_5V = 0x13 = 4.5V */
	/* SWITCH_CHR_VINDPM_7V = 0x25 = 6.3V */
	/* SWITCH_CHR_VINDPM_9V = 0x37 = 8.1V */
	/* SWITCH_CHR_VINDPM_12V = 0x54 = 11V */
	/* Our unit of votage is mV */
	int ret = 0;
#if defined(CONFIG_MTK_DYNAMIC_BAT_CV_SUPPORT)
	u32 mivr = 0;
	u8 reg_vindpm = *((u8 *)data);

	switch (reg_vindpm) {
		case SWITCH_CHR_VINDPM_5V:
			mivr = 4500;
			break;
		case SWITCH_CHR_VINDPM_7V:
			mivr = 6300;
			break;
		case SWITCH_CHR_VINDPM_9V:
			mivr = 8100;
			break;
		case SWITCH_CHR_VINDPM_12V:
			mivr = 11000;
			break;
		default:
			mivr = 4500;
			break;
	}

	ret = rt_charger_set_mivr(mivr);
#endif
	return ret;
}

static int charging_set_vbus_ovp_en(void *data)
{
	int ret = 0;
	kal_uint32 e = *(kal_uint32 *)data;

#if 1
	pmic_set_register_value(PMIC_RG_VCDT_HV_EN, e);
#else
	pmic_set_register_value(MT6351_PMIC_RG_VCDT_HV_EN, e);
#endif

	return ret;
}

static int charging_get_bif_vbat(void *data)
{
	int ret = 0;
#ifdef CONFIG_MTK_BIF_SUPPORT
	int vbat = 0;
	/* turn on VBIF28 regulator*/
	/* bif_init(); */

	/* change to HW control mode */
	/* pmic_set_register_value(MT6351_PMIC_RG_VBIF28_ON_CTRL, 0);
	pmic_set_register_value(MT6351_PMIC_RG_VBIF28_EN, 1); */

	bif_ADC_enable();

	vbat = bif_read16(MW3790_VBAT);
	*(kal_uint32 *) (data) = vbat;

	/*turn off LDO and change SW control back to HW control */
	/*pmic_set_register_value(MT6351_PMIC_RG_VBIF28_EN, 0);*/
	/*pmic_set_register_value(MT6351_PMIC_RG_VBIF28_ON_CTRL, 1);*/
#else
	*(kal_uint32 *)(data) = 0;
#endif
	return ret;	
}

static int charging_set_chrind_ck_pdn(void *data)
{
	int ret = 0;
#if 0
	kal_uint32 pwr_dn;

	pwr_dn = *(kal_uint32 *)data;
#if 1
	pmic_set_register_value(PMIC_CLK_DRV_CHRIND_CK_PDN, pwr_dn);
#else
	pmic_set_register_value(PMIC_RG_DRV_CHRIND_CK_PDN, pwr_dn);
#endif
#endif

	return ret;
}


static int charging_sw_init(void *data)
{
	int ret = 0;
	/*put here anything needed to be init upon battery_common driver probe*/
#ifdef CONFIG_MTK_BIF_SUPPORT
	int vbat;
	if (bif_inited != 1) {
		bif_init();
		charging_get_bif_vbat(&vbat);
		if (vbat != 0) {
			battery_log(BAT_LOG_CRTI, "[BIF]BIF battery detected.\n");
			bif_inited = 1;
		} else
			battery_log(BAT_LOG_CRTI, "[BIF]BIF battery _NOT_ detected.\n");
	}
#endif
	return ret;
}

static int charging_enable_safetytimer(void *data)
{
	int ret = 0;
	u8 timer_enable = *((u8 *)data);

	ret = rt_charger_enable_timer(timer_enable);

	return ret;
}


static int charging_get_bif_tbat(void *data)
{
	int ret = 0;

#ifdef CONFIG_MTK_BIF_SUPPORT
	int tbat = 0;
	int ret_read;
	int tried = 0;

	mdelay(50);

	if (bif_inited == 1) {
		do {
			bif_ADC_enable();
			ret_read = bif_read8(MW3790_TBAT, &tbat);
			tried++;
			mdelay(50);
			if (tried > 3)
				break;
		} while (ret_read != 1);

		if (tried <= 3)
			*(kal_int32 *)(data) = tbat;
		else
			ret = -ENOTSUPP;
	}
#endif

	return ret;	
}

static int charging_enable_otg(void *data)
{
	int ret = 0;
	u8 enable = *((u8 *)data);

	ret = rt_charger_enable_otg(enable);	

	return ret;
}

/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)
static int charging_enable_te_shutdown(void *data)
{
	int ret = 0;

	u8 enable = *((u8 *)data);
	ret = rt_charger_enable_te_shutdown(enable);

	return ret;
}
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/

/*[Arima_8100][bozhi_lin] RID003699 OVP warning message to end-user 20170320 begin*/
static int charging_get_vinovpi(void *data)
{
	int ret = 0;
	u32 vinovpi = 0;

	ret = rt_charger_get_vinovpi(&vinovpi);

	*((u32 *)data) = vinovpi;

	return ret;
}

static int charging_get_is_charging(void *data)
{
	int ret = 0;
	enum rt_charging_status chg_stat = RT_CHG_STATUS_READY;

	ret = rt_charger_get_charging_status(&chg_stat);
	
	/* Return is charging done or not */
	switch (chg_stat) {
		case RT_CHG_STATUS_READY:
		case RT_CHG_STATUS_FAULT:
			*((u32 *)data) = 0;
			break;
		case RT_CHG_STATUS_PROGRESS:
		case RT_CHG_STATUS_DONE:
			*((u32 *)data) = 1;
			break;
		default:
			*((u32 *)data) = 0;
			break;
	}

	return ret;
}
/*[Arima_8100][bozhi_lin] 20170320 end*/

static int(*charging_func[CHARGING_CMD_NUMBER]) (void *data);

kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	kal_int32 ret;
	static signed int init = -1;

	if (init == -1) {
		init = 0;
		charging_func[CHARGING_CMD_INIT] = charging_hw_init;
		charging_func[CHARGING_CMD_DUMP_REGISTER] = charging_dump_register;
		charging_func[CHARGING_CMD_ENABLE] = charging_enable;
		charging_func[CHARGING_CMD_SET_CV_VOLTAGE] = charging_set_cv_voltage;
		charging_func[CHARGING_CMD_GET_CURRENT] = charging_get_current;
		charging_func[CHARGING_CMD_SET_CURRENT] = charging_set_current;
		charging_func[CHARGING_CMD_SET_INPUT_CURRENT] = charging_set_input_current;
		charging_func[CHARGING_CMD_GET_CHARGING_STATUS] = charging_get_charging_status;
		charging_func[CHARGING_CMD_RESET_WATCH_DOG_TIMER] = charging_reset_watch_dog_timer;
		charging_func[CHARGING_CMD_SET_HV_THRESHOLD] = charging_set_hv_threshold;
		charging_func[CHARGING_CMD_GET_HV_STATUS] = charging_get_hv_status;
		charging_func[CHARGING_CMD_GET_BATTERY_STATUS] = charging_get_battery_status;
		charging_func[CHARGING_CMD_GET_CHARGER_DET_STATUS] = charging_get_charger_det_status;
		charging_func[CHARGING_CMD_GET_CHARGER_TYPE] = charging_get_charger_type;
		charging_func[CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER] = charging_get_is_pcm_timer_trigger;
		charging_func[CHARGING_CMD_SET_PLATFORM_RESET] = charging_set_platform_reset;
		charging_func[CHARGING_CMD_GET_PLATFORM_BOOT_MODE] = charging_get_platform_boot_mode;
		charging_func[CHARGING_CMD_SET_POWER_OFF] = charging_set_power_off;
		charging_func[CHARGING_CMD_GET_POWER_SOURCE] = charging_get_power_source;
		charging_func[CHARGING_CMD_GET_CSDAC_FALL_FLAG] = charging_get_csdac_full_flag;
		charging_func[CHARGING_CMD_SET_TA_CURRENT_PATTERN] = charging_set_ta_current_pattern;
		charging_func[CHARGING_CMD_SET_ERROR_STATE] = charging_set_error_state;
		charging_func[CHARGING_CMD_DISO_INIT] = charging_diso_init;
		charging_func[CHARGING_CMD_GET_DISO_STATE] = charging_get_diso_state;
		charging_func[CHARGING_CMD_SET_VINDPM] = charging_set_vindpm;
		charging_func[CHARGING_CMD_SET_VBUS_OVP_EN] = charging_set_vbus_ovp_en;
		charging_func[CHARGING_CMD_GET_BIF_VBAT] = charging_get_bif_vbat;
		charging_func[CHARGING_CMD_SET_CHRIND_CK_PDN] = charging_set_chrind_ck_pdn;
		charging_func[CHARGING_CMD_SW_INIT] = charging_sw_init;
		charging_func[CHARGING_CMD_ENABLE_SAFETY_TIMER] = charging_enable_safetytimer;
		charging_func[CHARGING_CMD_SET_HIZ_SWCHR] = charging_set_hiz_swchr;
		charging_func[CHARGING_CMD_GET_BIF_TBAT] = charging_get_bif_tbat;
		charging_func[CHARGING_CMD_ENABLE_OTG] = charging_enable_otg;
/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)
		charging_func[CHARGING_CMD_ENABLE_TE_SHUTDOWN] = charging_enable_te_shutdown;
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/
/*[Arima_8100][bozhi_lin] RID003699 OVP warning message to end-user 20170320 begin*/
		charging_func[CHARGING_CMD_GET_VINOVPI] = charging_get_vinovpi;
		charging_func[CHARGING_CMD_GET_IS_CHARGING] = charging_get_is_charging;
/*[Arima_8100][bozhi_lin] 20170320 end*/
	}

	if (cmd < CHARGING_CMD_NUMBER) {
		if (charging_func[cmd] != NULL) {
			ret = charging_func[cmd](data);
		} else {
			ret = -ENOTSUPP;
		}
	} else {
		ret = -ENOTSUPP;
	}

	return ret;
}
