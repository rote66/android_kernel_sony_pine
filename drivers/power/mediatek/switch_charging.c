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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *    switch_charging.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
 * Revision:   1.0
 * Modtime:   11 Aug 2005 10:28:16
 * Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc
 *
 * 03 05 2015 wy.chuang
 * [ALPS01921641] [L1_merge] for PMIC and charging
 * .
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/types.h>
#include <linux/kernel.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/battery_meter_hal.h>
#include <mt-plat/charging.h>
#include <mach/mt_charging.h>
#include <mt-plat/mt_boot.h>

#include "mtk_pep_intf.h"
#include "mtk_pep20_intf.h"

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#include <mt-plat/diso.h>
#include <mach/mt_diso.h>
#endif


/* ============================================================ // */
/* define */
/* ============================================================ // */
/* cut off to full */
#define POST_CHARGING_TIME (30*60)	/* 30mins */
#define FULL_CHECK_TIMES 6

#define STATUS_OK    0
#define STATUS_UNSUPPORTED    -1
#define STATUS_FAIL -2

/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)
#define FIRST_MAINTAIN_CHARGING_TIME		60 * 60 * 60 // 60 hours
#define SECOND_MAINTAIN_CHARGING_TIME		200 * 60 * 60 // 200 hours
#define TOTAL_MAINTAIN_CHARGING_TIME		(FIRST_MAINTAIN_CHARGING_TIME+SECOND_MAINTAIN_CHARGING_TIME)

#define V_FULL2CC_THRES			4150
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/

/*[Arima_8100][bozhi_lin] RID001582 - Soft charge 3.0 20161116 begin*/
#if defined(CONFIG_CHARGING_SOFTCHARE3_0)
#define V_SOFTCHARE3_0_THRES	4250
#define SOFTCHARE3_0_TEMP_POS_40_THRESHOLD	40
#define SOFTCHARE3_0_TEMP_POS_30_THRESHOLD	30
#define SOFTCHARE3_0_TEMP_POS_20_THRESHOLD	20

#define SOFTCHARE3_0_TOTAL_CHARGING_TIME	250 * 60 * 60 // 250 hours
#define SOFTCHARE3_0_TEMP_POS_40_FACTOR	(161 / 100) //1.61
#define SOFTCHARE3_0_TEMP_POS_30_FACTOR	(273 / 100) //2.73
#define SOFTCHARE3_0_TEMP_POS_20_FACTOR	(832 / 100) //8.32
#endif
/*[Arima_8100][bozhi_lin] 20161116 end*/

/* ============================================================ // */
/* global variable */
/* ============================================================ // */
unsigned int g_bcct_flag = 0;
unsigned int g_bcct_value = 0;
/*input-output curent distinction*/
unsigned int g_bcct_input_flag = 0;
unsigned int g_bcct_input_value = 0;
unsigned int g_full_check_count = 0;
CHR_CURRENT_ENUM g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
unsigned int g_usb_state = USB_UNCONFIGURED;
static bool usb_unlimited;
#if defined(CONFIG_MTK_HAFG_20)
/*[Arima_8100][bozhi_lin] set battery cv voltage to 4.35V 20161006 begin*/
#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
BATTERY_VOLTAGE_ENUM g_cv_voltage = BATTERY_VOLT_04_350000_V;
#else
BATTERY_VOLTAGE_ENUM g_cv_voltage = BATTERY_VOLT_04_200000_V;
#endif
/*[Arima_8100][bozhi_lin] 20161006 end*/
unsigned int get_cv_voltage(void)
{
	return g_cv_voltage;
}
#endif
DEFINE_MUTEX(g_ichg_aicr_access_mutex);
DEFINE_MUTEX(g_aicr_access_mutex);
DEFINE_MUTEX(g_ichg_access_mutex);
unsigned int g_aicr_upper_bound;
/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)

#else
static bool g_enable_dynamic_cv = true;
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/

 /* ///////////////////////////////////////////////////////////////////////////////////////// */
 /* // JEITA */
 /* ///////////////////////////////////////////////////////////////////////////////////////// */
#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
int g_temp_status = TEMP_POS_10_TO_POS_45;
kal_bool temp_error_recovery_chr_flag = KAL_TRUE;
#endif

/* ============================================================ // */
/* function prototype */
/* ============================================================ // */
void BATTERY_SetUSBState(int usb_state_value)
{
#if defined(CONFIG_POWER_EXT)
	battery_log(BAT_LOG_CRTI, "[BATTERY_SetUSBState] in FPGA/EVB, no service\r\n");
#else
	if ((usb_state_value < USB_SUSPEND) || ((usb_state_value > USB_CONFIGURED))) {
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY] BAT_SetUSBState Fail! Restore to default value\r\n");
		usb_state_value = USB_UNCONFIGURED;
	} else {
		battery_log(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Success! Set %d\r\n",
			    usb_state_value);
		g_usb_state = usb_state_value;
	}
#endif
}


unsigned int get_charging_setting_current(void)
{
	return g_temp_CC_value;
}

int mtk_get_dynamic_cv(unsigned int *cv)
{
	int ret = 0;
#ifdef CONFIG_MTK_BIF_SUPPORT
	u32 _cv;
	u32 vbat_bif = 0, vbat_auxadc = 0, vbat = 0;
	u32 retry_cnt = 0;
	u32 ircmp_volt_clamp = 0, ircmp_resistor = 0;

	if (!g_enable_dynamic_cv) {
		if (batt_cust_data.high_battery_voltage_support)
			_cv = BATTERY_VOLT_04_340000_V / 1000;
		else
			_cv = BATTERY_VOLT_04_200000_V / 1000;
		goto _out;
	}

	do {
		ret = battery_charging_control(CHARGING_CMD_GET_BIF_VBAT,
			&vbat_bif);
		vbat_auxadc = battery_meter_get_battery_voltage(KAL_TRUE);

		if (ret >= 0 && vbat_bif != 0 && vbat_bif < vbat_auxadc) {
			vbat = vbat_bif;
			battery_log(BAT_LOG_CRTI,
				"%s: use BIF vbat = %dmV, dV to auxadc = %dmV\n",
				__func__, vbat, vbat_auxadc - vbat_bif);
			break;
		}

		retry_cnt++;
	} while (retry_cnt < 5);

	if (retry_cnt == 5) {
		ret = 0;
		vbat = vbat_auxadc;
		battery_log(BAT_LOG_CRTI,
			"%s: use AUXADC vbat = %dmV, since BIF vbat = %d\n",
			__func__, vbat_auxadc, vbat_bif);
	}

	/* Adjust CV according to the obtained vbat */
	if (vbat >= 3400 && vbat < 4300) {
		_cv = 4550;
		battery_charging_control(CHARGING_CMD_SET_IRCMP_VOLT_CLAMP,
			&ircmp_volt_clamp);
		battery_charging_control(CHARGING_CMD_SET_IRCMP_RESISTOR,
			&ircmp_resistor);
	} else {
		if (batt_cust_data.high_battery_voltage_support)
			_cv = BATTERY_VOLT_04_340000_V / 1000;
		else
			_cv = BATTERY_VOLT_04_200000_V / 1000;

		/* Turn on IR compensation */
		ircmp_volt_clamp = 200;
		ircmp_resistor = 80;
		battery_charging_control(CHARGING_CMD_SET_IRCMP_VOLT_CLAMP,
			&ircmp_volt_clamp);
		battery_charging_control(CHARGING_CMD_SET_IRCMP_RESISTOR,
			&ircmp_resistor);

		/* Disable dynamic CV */
		g_enable_dynamic_cv = false;
	}

_out:
	*cv = _cv;
	battery_log(BAT_LOG_CRTI, "%s: CV = %dmV, enable dynamic cv = %d\n",
		__func__, _cv, g_enable_dynamic_cv);
#else
	ret = -ENOTSUPP;
#endif
	return ret;
}

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)

/*[Arima_8100][bozhi_lin] Charging enable JEITA battery temperature check 20161028 begin*/
#if defined(JEITA_SONY_CONFIG)
static BATTERY_VOLTAGE_ENUM select_jeita_cv(void)
{
	BATTERY_VOLTAGE_ENUM cv_voltage;

	if (g_temp_status == TEMP_ABOVE_POS_55) {
		cv_voltage = JEITA_TEMP_ABOVE_POS_55_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_45_TO_POS_55) {
		cv_voltage = JEITA_TEMP_POS_45_TO_POS_55_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_10_TO_POS_45) {
/*[Arima_8100][bozhi_lin] set battery cv voltage to 4.35V 20161006 begin*/
		if (batt_cust_data.high_battery_voltage_support)
			cv_voltage = BATTERY_VOLT_04_350000_V;
		else
			cv_voltage = JEITA_TEMP_POS_10_TO_POS_45_CV_VOLTAGE;
/*[Arima_8100][bozhi_lin] 20161006 end*/
	} else if (g_temp_status == TEMP_POS_5_TO_POS_10) {
		cv_voltage = JEITA_TEMP_POS_5_TO_POS_10_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_BELOW_POS_5) {
		cv_voltage = JEITA_TEMP_BELOW_POS_5_CV_VOLTAGE;
	} else {
		cv_voltage = BATTERY_VOLT_04_200000_V;
	}

	return cv_voltage;
}

PMU_STATUS do_jeita_state_machine(void)
{
	BATTERY_VOLTAGE_ENUM cv_voltage;
	PMU_STATUS jeita_status = PMU_STATUS_OK;

	/* JEITA battery temp Standard */

	if (BMT_status.temperature >= TEMP_POS_55_THRESHOLD) {
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY] Battery Over high Temperature(%d) !!\n\r",
			    TEMP_POS_55_THRESHOLD);

		g_temp_status = TEMP_ABOVE_POS_55;

		return PMU_STATUS_FAIL;
	} else if (BMT_status.temperature > TEMP_POS_45_THRESHOLD) {	/* control 45c to normal behavior */
		if ((g_temp_status == TEMP_ABOVE_POS_55)
		    && (BMT_status.temperature >= TEMP_POS_55_THRES_MINUS_X_DEGREE)) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
				    TEMP_POS_55_THRES_MINUS_X_DEGREE, TEMP_POS_55_THRESHOLD);

			jeita_status = PMU_STATUS_FAIL;
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
				    TEMP_POS_45_THRESHOLD, TEMP_POS_55_THRESHOLD);

			g_temp_status = TEMP_POS_45_TO_POS_55;
		}
	} else if (BMT_status.temperature >= TEMP_POS_10_THRESHOLD) {
		if (((g_temp_status == TEMP_POS_45_TO_POS_55)
		     && (BMT_status.temperature >= TEMP_POS_45_THRES_MINUS_X_DEGREE))
		    || ((g_temp_status == TEMP_POS_5_TO_POS_10)
			&& (BMT_status.temperature <= TEMP_POS_10_THRES_PLUS_X_DEGREE))) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature not recovery to normal temperature charging mode yet!!\n\r");
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Normal Temperature between %d and %d !!\n\r",
				    TEMP_POS_10_THRESHOLD, TEMP_POS_45_THRESHOLD);
			g_temp_status = TEMP_POS_10_TO_POS_45;
		}
	} else if (BMT_status.temperature >= TEMP_POS_5_THRESHOLD) {
		if ((g_temp_status == TEMP_BELOW_POS_5)
		    && (BMT_status.temperature <= TEMP_POS_5_THRES_PLUS_X_DEGREE)) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
				    TEMP_POS_5_THRESHOLD, TEMP_POS_5_THRES_PLUS_X_DEGREE);

			jeita_status = PMU_STATUS_FAIL;
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
				    TEMP_POS_5_THRESHOLD, TEMP_POS_10_THRESHOLD);

			g_temp_status = TEMP_POS_5_TO_POS_10;
		}
	} else {
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY] Battery below low Temperature(%d) !!\n\r",
			    TEMP_POS_5_THRESHOLD);
		g_temp_status = TEMP_BELOW_POS_5;

		jeita_status = PMU_STATUS_FAIL;
	}

	/* set CV after temperature changed */
	/* In normal range, we adjust CV dynamically */
	if (g_temp_status != TEMP_POS_10_TO_POS_45) {
		cv_voltage = select_jeita_cv();
		battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE,
			&cv_voltage);
#if defined(CONFIG_MTK_HAFG_20)
		g_cv_voltage = cv_voltage;
#endif
	}

	return jeita_status;
}


static void set_jeita_charging_current(void)
{
#if 1
	if (BMT_status.charger_type == STANDARD_HOST)
		return;
#endif

	if ((g_temp_status == TEMP_POS_5_TO_POS_10) || (g_temp_status == TEMP_POS_45_TO_POS_55)) {
		g_temp_CC_value = CHARGE_CURRENT_700_00_MA;
		g_temp_input_CC_value = CHARGE_CURRENT_700_00_MA;
		battery_log(BAT_LOG_CRTI, "[BATTERY] JEITA set charging current : %d\r\n",
			    g_temp_CC_value);
	}
}
#else
static BATTERY_VOLTAGE_ENUM select_jeita_cv(void)
{
	BATTERY_VOLTAGE_ENUM cv_voltage;

	if (g_temp_status == TEMP_ABOVE_POS_60) {
		cv_voltage = JEITA_TEMP_ABOVE_POS_60_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_45_TO_POS_60) {
		cv_voltage = JEITA_TEMP_POS_45_TO_POS_60_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_10_TO_POS_45) {
		if (batt_cust_data.high_battery_voltage_support)
			cv_voltage = BATTERY_VOLT_04_340000_V;
		else
			cv_voltage = JEITA_TEMP_POS_10_TO_POS_45_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_0_TO_POS_10) {
		cv_voltage = JEITA_TEMP_POS_0_TO_POS_10_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
		cv_voltage = JEITA_TEMP_NEG_10_TO_POS_0_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_BELOW_NEG_10) {
		cv_voltage = JEITA_TEMP_BELOW_NEG_10_CV_VOLTAGE;
	} else {
		cv_voltage = BATTERY_VOLT_04_200000_V;
	}

	return cv_voltage;
}

PMU_STATUS do_jeita_state_machine(void)
{
	BATTERY_VOLTAGE_ENUM cv_voltage;
	PMU_STATUS jeita_status = PMU_STATUS_OK;

	/* JEITA battery temp Standard */

	if (BMT_status.temperature >= TEMP_POS_60_THRESHOLD) {
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY] Battery Over high Temperature(%d) !!\n\r",
			    TEMP_POS_60_THRESHOLD);

		g_temp_status = TEMP_ABOVE_POS_60;

		return PMU_STATUS_FAIL;
	} else if (BMT_status.temperature > TEMP_POS_45_THRESHOLD) {	/* control 45c to normal behavior */
		if ((g_temp_status == TEMP_ABOVE_POS_60)
		    && (BMT_status.temperature >= TEMP_POS_60_THRES_MINUS_X_DEGREE)) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
				    TEMP_POS_60_THRES_MINUS_X_DEGREE, TEMP_POS_60_THRESHOLD);

			jeita_status = PMU_STATUS_FAIL;
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
				    TEMP_POS_45_THRESHOLD, TEMP_POS_60_THRESHOLD);

			g_temp_status = TEMP_POS_45_TO_POS_60;
		}
	} else if (BMT_status.temperature >= TEMP_POS_10_THRESHOLD) {
		if (((g_temp_status == TEMP_POS_45_TO_POS_60)
		     && (BMT_status.temperature >= TEMP_POS_45_THRES_MINUS_X_DEGREE))
		    || ((g_temp_status == TEMP_POS_0_TO_POS_10)
			&& (BMT_status.temperature <= TEMP_POS_10_THRES_PLUS_X_DEGREE))) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature not recovery to normal temperature charging mode yet!!\n\r");
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Normal Temperature between %d and %d !!\n\r",
				    TEMP_POS_10_THRESHOLD, TEMP_POS_45_THRESHOLD);
			g_temp_status = TEMP_POS_10_TO_POS_45;
		}
	} else if (BMT_status.temperature >= TEMP_POS_0_THRESHOLD) {
		if ((g_temp_status == TEMP_NEG_10_TO_POS_0 || g_temp_status == TEMP_BELOW_NEG_10)
		    && (BMT_status.temperature <= TEMP_POS_0_THRES_PLUS_X_DEGREE)) {
			if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
				battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
					    TEMP_POS_0_THRES_PLUS_X_DEGREE, TEMP_POS_10_THRESHOLD);
			}
			if (g_temp_status == TEMP_BELOW_NEG_10) {
				battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
					    TEMP_POS_0_THRESHOLD, TEMP_POS_0_THRES_PLUS_X_DEGREE);
				return PMU_STATUS_FAIL;
			}
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
				    TEMP_POS_0_THRESHOLD, TEMP_POS_10_THRESHOLD);

			g_temp_status = TEMP_POS_0_TO_POS_10;
		}
	} else if (BMT_status.temperature >= TEMP_NEG_10_THRESHOLD) {
		if ((g_temp_status == TEMP_BELOW_NEG_10)
		    && (BMT_status.temperature <= TEMP_NEG_10_THRES_PLUS_X_DEGREE)) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
				    TEMP_NEG_10_THRESHOLD, TEMP_NEG_10_THRES_PLUS_X_DEGREE);

			jeita_status = PMU_STATUS_FAIL;
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
				    TEMP_NEG_10_THRESHOLD, TEMP_POS_0_THRESHOLD);

			g_temp_status = TEMP_NEG_10_TO_POS_0;
		}
	} else {
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY] Battery below low Temperature(%d) !!\n\r",
			    TEMP_NEG_10_THRESHOLD);
		g_temp_status = TEMP_BELOW_NEG_10;

		jeita_status = PMU_STATUS_FAIL;
	}

	/* set CV after temperature changed */
	/* In normal range, we adjust CV dynamically */
	if (g_temp_status != TEMP_POS_10_TO_POS_45) {
		cv_voltage = select_jeita_cv();
		battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE,
			&cv_voltage);
#if defined(CONFIG_MTK_HAFG_20)
		g_cv_voltage = cv_voltage;
#endif
	}

	return jeita_status;
}


static void set_jeita_charging_current(void)
{
#ifdef CONFIG_USB_IF
	if (BMT_status.charger_type == STANDARD_HOST)
		return;
#endif

	if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
		g_temp_CC_value = CHARGE_CURRENT_350_00_MA;
		g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		battery_log(BAT_LOG_CRTI, "[BATTERY] JEITA set charging current : %d\r\n",
			    g_temp_CC_value);
	}
}
#endif
/*[Arima_8100][bozhi_lin] 20161028 end*/

#endif /* CONFIG_MTK_JEITA_STANDARD_SUPPORT */

bool get_usb_current_unlimited(void)
{
	if (BMT_status.charger_type == STANDARD_HOST || BMT_status.charger_type == CHARGING_HOST)
		return usb_unlimited;

		return false;
}

void set_usb_current_unlimited(bool enable)
{
	usb_unlimited = enable;
}

/*BQ25896 is the first switch chrager separating input and charge current
*/
unsigned int set_chr_input_current_limit(int current_limit)
{
#ifdef CONFIG_MTK_SWITCH_INPUT_OUTPUT_CURRENT_SUPPORT
	u32 power_path_enable = 1;
	CHR_CURRENT_ENUM chr_type_aicr = 0; /* 10uA */
	CHR_CURRENT_ENUM chr_type_ichg = 0;

	mutex_lock(&g_aicr_access_mutex);
	if (current_limit != -1) {
		g_bcct_input_flag = 1;
		if (current_limit < 100) {
			/* limit < 100, turn off power path */
			current_limit = 0;
			power_path_enable = 0;
			battery_charging_control(CHARGING_CMD_ENABLE_POWER_PATH,
				&power_path_enable);
		} else {
			/* Enable power path if it is disabled previously */
			if (g_bcct_input_value == 0) {
				power_path_enable = 1;
				battery_charging_control(CHARGING_CMD_ENABLE_POWER_PATH,
					&power_path_enable);
			}

			switch (BMT_status.charger_type) {
			case STANDARD_HOST:
				chr_type_aicr = batt_cust_data.usb_charger_current;
				break;
			case NONSTANDARD_CHARGER:
				chr_type_aicr = batt_cust_data.non_std_ac_charger_current;
				break;
			case STANDARD_CHARGER:
				chr_type_aicr = batt_cust_data.ac_charger_input_current;
				mtk_pep20_set_charging_current(&chr_type_ichg, &chr_type_aicr);
				mtk_pep_set_charging_current(&chr_type_ichg, &chr_type_aicr);
				break;
			case CHARGING_HOST:
				chr_type_aicr = batt_cust_data.charging_host_charger_current;
				break;
			case APPLE_2_1A_CHARGER:
				chr_type_aicr = batt_cust_data.apple_2_1a_charger_current;
				break;
			case APPLE_1_0A_CHARGER:
				chr_type_aicr = batt_cust_data.apple_1_0a_charger_current;
				break;
			case APPLE_0_5A_CHARGER:
				chr_type_aicr = batt_cust_data.apple_0_5a_charger_current;
				break;
			default:
				chr_type_aicr = CHARGE_CURRENT_500_00_MA;
				break;
			}
			chr_type_aicr /= 100;
			if (current_limit > chr_type_aicr)
				current_limit = chr_type_aicr;
		}

		g_bcct_input_value = current_limit;
	} else {
		/* Enable power path if it is disabled previously */
		if (g_bcct_input_value == 0) {
			power_path_enable = 1;
			battery_charging_control(CHARGING_CMD_ENABLE_POWER_PATH,
				&power_path_enable);
		}

		/* Change to default current setting */
		g_bcct_input_flag = 0;
	}

	battery_log(BAT_LOG_CRTI,
		"[BATTERY] set_chr_input_current_limit (%d)\n", current_limit);

	mutex_unlock(&g_aicr_access_mutex);
	return g_bcct_input_flag;
#else
	battery_log(BAT_LOG_CRTI,
		"[BATTERY] set_chr_input_current_limit _NOT_ supported\n");
	return 0;
#endif
}

static void mtk_select_ichg_aicr(void);
unsigned int set_bat_charging_current_limit(int current_limit)
{
	CHR_CURRENT_ENUM chr_type_ichg = 0;
	CHR_CURRENT_ENUM chr_type_aicr = 0;

	mutex_lock(&g_ichg_access_mutex);
	if (current_limit != -1) {
		g_bcct_flag = 1;
		switch (BMT_status.charger_type) {
		case STANDARD_HOST:
			chr_type_ichg = batt_cust_data.usb_charger_current;
			break;
		case NONSTANDARD_CHARGER:
			chr_type_ichg = batt_cust_data.non_std_ac_charger_current;
			break;
		case STANDARD_CHARGER:
			chr_type_ichg = batt_cust_data.ac_charger_current;
			mtk_pep20_set_charging_current(&chr_type_ichg, &chr_type_aicr);
			mtk_pep_set_charging_current(&chr_type_ichg, &chr_type_aicr);
			break;
		case CHARGING_HOST:
			chr_type_ichg = batt_cust_data.charging_host_charger_current;
			break;
		case APPLE_2_1A_CHARGER:
			chr_type_ichg = batt_cust_data.apple_2_1a_charger_current;
			break;
		case APPLE_1_0A_CHARGER:
			chr_type_ichg = batt_cust_data.apple_1_0a_charger_current;
			break;
		case APPLE_0_5A_CHARGER:
			chr_type_ichg = batt_cust_data.apple_0_5a_charger_current;
			break;
		default:
			chr_type_ichg = CHARGE_CURRENT_500_00_MA;
			break;
		}
		chr_type_ichg /= 100;
		if (current_limit > chr_type_ichg)
			current_limit = chr_type_ichg;
		g_bcct_value = current_limit;
	} else /* change to default current setting */
		g_bcct_flag = 0;

	mtk_select_ichg_aicr();

	battery_log(BAT_LOG_CRTI,
		"[BATTERY] set_bat_charging_current_limit (%d)\r\n",
		current_limit);

	mutex_unlock(&g_ichg_access_mutex);
	return g_bcct_flag;
}

int mtk_chr_reset_aicr_upper_bound(void)
{
	g_aicr_upper_bound = 0;
	return 0;
}

int set_chr_boost_current_limit(unsigned int current_limit)
{
	int ret = 0;

	ret = battery_charging_control(CHARGING_CMD_SET_BOOST_CURRENT_LIMIT,
		&current_limit);

	return ret;
}

int set_chr_enable_otg(unsigned int enable)
{
	int ret = 0;

	ret = battery_charging_control(CHARGING_CMD_ENABLE_OTG, &enable);

	return ret;
}

int mtk_chr_get_tchr(int *min_temp, int *max_temp)
{
	int ret = 0;
	int temp[2] = {0, 0};

	ret = battery_charging_control(CHARGING_CMD_GET_CHARGER_TEMPERATURE, temp);
	if (ret < 0)
		return ret;

	*min_temp = temp[0];
	*max_temp = temp[1];

	return ret;
}

int mtk_chr_get_soc(unsigned int *soc)
{
	if (BMT_status.SOC < 0) {
		*soc = 0;
		return -ENOTSUPP;
	}

	*soc = BMT_status.SOC;

	return 0;
}

int mtk_chr_get_ui_soc(unsigned int *ui_soc)
{
	/* UI_SOC2 is the one that shows on UI */
	if (BMT_status.UI_SOC2 < 0) {
		*ui_soc = 0;
		return -ENOTSUPP;
	}

	*ui_soc = BMT_status.UI_SOC2;

	return 0;
}

int mtk_chr_get_vbat(unsigned int *vbat)
{
	if (BMT_status.bat_vol < 0) {
		*vbat = 0;
		return -ENOTSUPP;
	}

	*vbat = BMT_status.bat_vol;

	return 0;
}

int mtk_chr_get_ibat(unsigned int *ibat)
{
	*ibat = BMT_status.IBattery / 10;
	return 0;
}

int mtk_chr_get_vbus(unsigned int *vbus)
{
	if (BMT_status.charger_vol < 0) {
		*vbus = 0;
		return -ENOTSUPP;
	}
	*vbus = BMT_status.charger_vol;

	return 0;
}

int mtk_chr_get_aicr(unsigned int *aicr)
{
	int ret = 0;
	u32 _aicr = 0; /* 10uA */

	ret = battery_charging_control(CHARGING_CMD_GET_INPUT_CURRENT, &_aicr);
	*aicr = _aicr / 100;

	return ret;
}

int mtk_chr_is_charger_exist(unsigned char *exist)
{
	*exist = (BMT_status.charger_exist ? 1 : 0);
	return 0;
}

static unsigned int charging_full_check(void)
{
	unsigned int status;

	battery_charging_control(CHARGING_CMD_GET_CHARGING_STATUS, &status);
	if (status == KAL_TRUE) {
		g_full_check_count++;
		if (g_full_check_count >= FULL_CHECK_TIMES)
			return KAL_TRUE;
		else
			return KAL_FALSE;
	}

	g_full_check_count = 0;
	return status;
}

static bool mtk_is_pep_series_connect(void)
{
	if (mtk_pep20_get_is_connect() ||
	    mtk_pep_get_is_connect()) {
		return true;
	}

	return false;
}

static int mtk_check_aicr_upper_bound(void)
{
	u32 aicr_upper_bound = 0; /* 10uA */

	if (mtk_is_pep_series_connect())
		return -EPERM;

	/* Check AICR upper bound gererated by AICL */
	aicr_upper_bound = g_aicr_upper_bound * 100;
	if (g_temp_input_CC_value > aicr_upper_bound && aicr_upper_bound > 0)
		g_temp_input_CC_value = aicr_upper_bound;

	return 0;
}

void select_charging_current(void)
{
	if (g_ftm_battery_flag) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] FTM charging : %d\r\n",
			    charging_level_data[0]);
		g_temp_CC_value = charging_level_data[0];

		if (g_temp_CC_value == CHARGE_CURRENT_450_00_MA) {
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		} else {
			g_temp_input_CC_value = CHARGE_CURRENT_MAX;
			g_temp_CC_value = batt_cust_data.ac_charger_current;

			battery_log(BAT_LOG_CRTI, "[BATTERY] set_ac_current \r\n");
		}
	} else {
		if (BMT_status.charger_type == STANDARD_HOST) {
/*[Arima_8100][bozhi_lin] RID001408 Invalid charger support 20161014 begin*/
//#ifdef CONFIG_USB_IF
#if 1
/*[Arima_8100][bozhi_lin] 20161014 end*/
			{
				g_temp_input_CC_value = CHARGE_CURRENT_MAX;
				if (g_usb_state == USB_SUSPEND)
					g_temp_CC_value = USB_CHARGER_CURRENT_SUSPEND;
				else if (g_usb_state == USB_UNCONFIGURED)
					g_temp_CC_value = batt_cust_data.usb_charger_current_unconfigured;
				else if (g_usb_state == USB_CONFIGURED)
					g_temp_CC_value = batt_cust_data.usb_charger_current_configured;
				else
					g_temp_CC_value = batt_cust_data.usb_charger_current_unconfigured;

				g_temp_input_CC_value = g_temp_CC_value;

				battery_log(BAT_LOG_CRTI,
					    "[BATTERY] STANDARD_HOST CC mode charging : %d on %d state\r\n",
					    g_temp_CC_value, g_usb_state);
			}
#else
			{
				g_temp_input_CC_value = batt_cust_data.usb_charger_current;
				g_temp_CC_value = batt_cust_data.usb_charger_current;
			}
#endif
		} else if (BMT_status.charger_type == NONSTANDARD_CHARGER) {
			g_temp_input_CC_value = batt_cust_data.non_std_ac_charger_current;
			g_temp_CC_value = batt_cust_data.non_std_ac_charger_current;

		} else if (BMT_status.charger_type == STANDARD_CHARGER) {
			if (batt_cust_data.ac_charger_input_current != 0)
				g_temp_input_CC_value = batt_cust_data.ac_charger_input_current;
			else
				g_temp_input_CC_value = batt_cust_data.ac_charger_current;

			g_temp_CC_value = batt_cust_data.ac_charger_current;
			mtk_pep_set_charging_current(&g_temp_CC_value, &g_temp_input_CC_value);
			mtk_pep20_set_charging_current(&g_temp_CC_value, &g_temp_input_CC_value);

		} else if (BMT_status.charger_type == CHARGING_HOST) {
			g_temp_input_CC_value = batt_cust_data.charging_host_charger_current;
			g_temp_CC_value = batt_cust_data.charging_host_charger_current;
		} else if (BMT_status.charger_type == APPLE_2_1A_CHARGER) {
			g_temp_input_CC_value = batt_cust_data.apple_2_1a_charger_current;
			g_temp_CC_value = batt_cust_data.apple_2_1a_charger_current;
		} else if (BMT_status.charger_type == APPLE_1_0A_CHARGER) {
			g_temp_input_CC_value = batt_cust_data.apple_1_0a_charger_current;
			g_temp_CC_value = batt_cust_data.apple_1_0a_charger_current;
		} else if (BMT_status.charger_type == APPLE_0_5A_CHARGER) {
			g_temp_input_CC_value = batt_cust_data.apple_0_5a_charger_current;
			g_temp_CC_value = batt_cust_data.apple_0_5a_charger_current;
		} else {
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
			g_temp_CC_value = CHARGE_CURRENT_500_00_MA;
		}

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE) {
			g_temp_input_CC_value = batt_cust_data.ac_charger_current;
			g_temp_CC_value = batt_cust_data.ac_charger_current;
		}
#endif

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
		set_jeita_charging_current();
#endif
	}

	mtk_check_aicr_upper_bound();
}

void select_charging_current_bcct(void)
{
/*BQ25896 is the first switch chrager separating input and charge current
* any switch charger can use this compile option which may be generalized
* to be CONFIG_SWITCH_INPUT_OUTPUT_CURRENT_SUPPORT
*/
#ifndef CONFIG_MTK_SWITCH_INPUT_OUTPUT_CURRENT_SUPPORT
	if ((BMT_status.charger_type == STANDARD_HOST) ||
	    (BMT_status.charger_type == NONSTANDARD_CHARGER)) {
		if (g_bcct_value < 100)
			g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (g_bcct_value < 500)
			g_temp_input_CC_value = CHARGE_CURRENT_100_00_MA;
		else if (g_bcct_value < 800)
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		else if (g_bcct_value == 800)
			g_temp_input_CC_value = CHARGE_CURRENT_800_00_MA;
		else
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
	} else if ((BMT_status.charger_type == STANDARD_CHARGER) ||
		   (BMT_status.charger_type == CHARGING_HOST)) {
		g_temp_input_CC_value = CHARGE_CURRENT_MAX;

		/* --------------------------------------------------- */
		/* set IOCHARGE */
		if (g_bcct_value < 550)
			g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (g_bcct_value < 650)
			g_temp_CC_value = CHARGE_CURRENT_550_00_MA;
		else if (g_bcct_value < 750)
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		else if (g_bcct_value < 850)
			g_temp_CC_value = CHARGE_CURRENT_750_00_MA;
		else if (g_bcct_value < 950)
			g_temp_CC_value = CHARGE_CURRENT_850_00_MA;
		else if (g_bcct_value < 1050)
			g_temp_CC_value = CHARGE_CURRENT_950_00_MA;
		else if (g_bcct_value < 1150)
			g_temp_CC_value = CHARGE_CURRENT_1050_00_MA;
		else if (g_bcct_value < 1250)
			g_temp_CC_value = CHARGE_CURRENT_1150_00_MA;
		else if (g_bcct_value == 1250)
			g_temp_CC_value = CHARGE_CURRENT_1250_00_MA;
		else
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		/* --------------------------------------------------- */

	} else {
		g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
	}
#else
/*[Arima_8100][bozhi_lin] only set charging current when bcct charging curent less than current charging current 20170223 begin*/
#if 1
	if (g_bcct_flag == 1) {
		if (g_temp_CC_value > (g_bcct_value * 100)) {
			g_temp_CC_value = g_bcct_value * 100;
		}
	}
	if (g_bcct_input_flag == 1) {
		if (g_temp_input_CC_value > (g_bcct_input_value * 100)) {
			g_temp_input_CC_value = g_bcct_input_value * 100;
		}
	}
#else
	if (g_bcct_flag == 1)
		g_temp_CC_value = g_bcct_value * 100;
	if (g_bcct_input_flag == 1)
		g_temp_input_CC_value = g_bcct_input_value * 100;
#endif
/*[Arima_8100][bozhi_lin] 20170223 end*/
#endif

	mtk_check_aicr_upper_bound();
}
#ifdef CONFIG_CHARGER_QNS
extern int battery_get_qns_current(void);
#endif

static void mtk_select_ichg_aicr(void)
{
	kal_bool enable_charger = KAL_TRUE;

#ifdef CONFIG_CHARGER_QNS
   int qns_current = battery_get_qns_current() * 100;
#endif
	mutex_lock(&g_ichg_aicr_access_mutex);

	/* Set Ichg, AICR */
	if (get_usb_current_unlimited()) {
		if (batt_cust_data.ac_charger_input_current != 0)
			g_temp_input_CC_value = batt_cust_data.ac_charger_input_current;
		else
			g_temp_input_CC_value = batt_cust_data.ac_charger_current;

		g_temp_CC_value = batt_cust_data.ac_charger_current;
		battery_log(BAT_LOG_FULL,
			"USB_CURRENT_UNLIMITED, use batt_cust_data.ac_charger_current\n");
	}
#ifndef CONFIG_MTK_SWITCH_INPUT_OUTPUT_CURRENT_SUPPORT
	else if (g_bcct_flag == 1) {
		select_charging_current_bcct();
		battery_log(BAT_LOG_FULL,
			"[BATTERY] select_charging_current_bcct !\n");
	} else {
		select_charging_current();
		battery_log(BAT_LOG_FULL,
			"[BATTERY] select_charging_current !\n");
	}
#else
	else if (g_bcct_flag == 1 || g_bcct_input_flag == 1) {
		select_charging_current();
/*[Arima_8100][bozhi_lin] set ac charging current to 1.5A and enable high voltage battery 20160922 begin*/
//<--[SM31][Thermal][JasonHsing] Reduce charging current for UC1-3c & UC2 20170109 BEGIN --
#if 1
		select_charging_current_bcct();
#endif
//-->[SM31][Thermal][JasonHsing] Reduce charging current for UC1-3c & UC2 20170109 END --
/*[Arima_8100][bozhi_lin] 20160922 end*/
		battery_log(BAT_LOG_FULL,
			"[BATTERY] select_charging_curret_bcct !\n");
	} else {
		select_charging_current();
		battery_log(BAT_LOG_FULL,
			"[BATTERY] select_charging_curret !\n");
	}
#endif
	battery_log(BAT_LOG_CRTI,
		"[BATTERY] Default CC mode charging : %d, input current = %d\n",
		g_temp_CC_value, g_temp_input_CC_value);

	battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT,
		&g_temp_input_CC_value);
#ifdef CONFIG_CHARGER_QNS
	if  ((qns_current != 0) && (qns_current < g_temp_CC_value)) {
		g_temp_CC_value = qns_current;
		battery_log(BAT_LOG_CRTI, "qns_current is less than current setting, use %dmA\r\n", qns_current/100);
	}
#endif
	battery_charging_control(CHARGING_CMD_SET_CURRENT,
		&g_temp_CC_value);

	/* For thermal, they need to enable charger immediately */
	if (g_temp_CC_value > 0 && g_temp_input_CC_value > 0)
		battery_charging_control(CHARGING_CMD_ENABLE, &enable_charger);

	/* If AICR < 300mA, stop PE+/PE+20 */
	if (g_temp_input_CC_value < CHARGE_CURRENT_300_00_MA) {
		if (mtk_pep20_get_is_enable()) {
			mtk_pep20_set_is_enable(false);
			if (mtk_pep20_get_is_connect())
				mtk_pep20_reset_ta_vchr();
		}
		if (mtk_pep_get_is_enable()) {
			mtk_pep_set_is_enable(false);
			if (mtk_pep_get_is_connect())
				mtk_pep_reset_ta_vchr();
		}
	} else if (g_bcct_input_flag == 0 && g_bcct_flag == 0) {
		if (!mtk_pep20_get_is_enable()) {
			mtk_pep20_set_is_enable(true);
			mtk_pep20_set_to_check_chr_type(true);
		}
		if (!mtk_pep_get_is_enable()) {
			mtk_pep_set_is_enable(true);
			mtk_pep_set_to_check_chr_type(true);
		}
	}

	mutex_unlock(&g_ichg_aicr_access_mutex);
}


static void mtk_select_cv(void)
{
	int ret = 0;
	u32 dynamic_cv = 0;
	BATTERY_VOLTAGE_ENUM cv_voltage;

#ifdef CONFIG_MTK_JEITA_STANDARD_SUPPORT
	/* If temperautre is abnormal, return not permitted */
	if (g_temp_status != TEMP_POS_10_TO_POS_45)
		return;
#endif

/*[Arima_8100][bozhi_lin] set battery cv voltage to 4.35V 20161006 begin*/
	if (batt_cust_data.high_battery_voltage_support)
		cv_voltage = BATTERY_VOLT_04_350000_V;
	else
		cv_voltage = BATTERY_VOLT_04_200000_V;
/*[Arima_8100][bozhi_lin] 20161006 end*/

/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)
	battery_log(BAT_LOG_CRTI, "[B]%s(%d): BMT_status.MAINTAIN_charging_time=%d, FIRST_MAINTAIN_CHARGING_TIME=%d, TOTAL_MAINTAIN_CHARGING_TIME=%d\n", __func__, __LINE__, BMT_status.MAINTAIN_charging_time, FIRST_MAINTAIN_CHARGING_TIME, TOTAL_MAINTAIN_CHARGING_TIME);
	if ((BMT_status.MAINTAIN_charging_time > FIRST_MAINTAIN_CHARGING_TIME) && (BMT_status.MAINTAIN_charging_time <= TOTAL_MAINTAIN_CHARGING_TIME)) {
/*[Arima_8100][bozhi_lin] charging maintenance implement: set cv voltage to 4.16V & 4.1V when temp out of normal 20170111 begin*/
		if (g_temp_status == TEMP_POS_10_TO_POS_45) {
			cv_voltage = BATTERY_VOLT_04_260000_V;
		} else {
			cv_voltage = BATTERY_VOLT_04_100000_V;
		}
/*[Arima_8100][bozhi_lin] 20170111 end*/
	}
	else if ((BMT_status.MAINTAIN_charging_time > 0) && (BMT_status.MAINTAIN_charging_time <= FIRST_MAINTAIN_CHARGING_TIME)) {
/*[Arima_8100][bozhi_lin] charging maintenance implement: set cv voltage to 4.16V & 4.1V when temp out of normal 20170111 begin*/
		if (g_temp_status == TEMP_POS_10_TO_POS_45) {
			cv_voltage = BATTERY_VOLT_04_300000_V;
		} else {
			cv_voltage = BATTERY_VOLT_04_160000_V;
		}
/*[Arima_8100][bozhi_lin] 20170111 end*/
	}
	else {
		#if defined(CONFIG_CHARGING_SOFTCHARE3_0)
			battery_log(BAT_LOG_CRTI, "[B]%s(%d): SOFTCHARGE3.0 (%d, %d, %d, %d, %d, %d, %d)\n", __func__, __LINE__,
				BMT_status.SOFTCHARE3_0_total_charging_time, 
				BMT_status.SOFTCHARE3_0_40_init_time, BMT_status.SOFTCHARE3_0_40_charging_time,
				BMT_status.SOFTCHARE3_0_30_init_time, BMT_status.SOFTCHARE3_0_30_charging_time,
				BMT_status.SOFTCHARE3_0_20_init_time, BMT_status.SOFTCHARE3_0_20_charging_time);
			/*[Arima_8100][bozhi_lin] RID001972 - Soft charge version 1.0 20161124 begin*/
			battery_log(BAT_LOG_CRTI, "[B]%s(%d): SOFTCHARGE1.0 (%d, %d)\n", __func__, __LINE__,
				BATTERY_DESIGN_CAPACITY_90_PERCENT, BMT_status.SOFTCHARE3_0_average_fcc_mah);
			/*[Arima_8100][bozhi_lin] 20161124 end*/
			if (BMT_status.SOFTCHARE3_0_total_charging_time > SOFTCHARE3_0_TOTAL_CHARGING_TIME) {
				cv_voltage = BATTERY_VOLT_04_300000_V;
			/*[Arima_8100][bozhi_lin] RID001972 - Soft charge version 1.0 20161124 begin*/
			} else if ((BMT_status.SOFTCHARE3_0_average_fcc_mah > 0) &&
				(BMT_status.SOFTCHARE3_0_average_fcc_mah <= BATTERY_DESIGN_CAPACITY_90_PERCENT)) {
				cv_voltage = BATTERY_VOLT_04_300000_V;
			/*[Arima_8100][bozhi_lin] 20161124 end*/
			} else {
				cv_voltage = BATTERY_VOLT_04_350000_V;
			}	
		#else
			cv_voltage = BATTERY_VOLT_04_350000_V;
		#endif
	}
#else
	#if defined(CONFIG_CHARGING_SOFTCHARE3_0)
		battery_log(BAT_LOG_CRTI, "[B]%s(%d): SOFTCHARGE3.0 (%d, %d, %d, %d, %d, %d, %d)\n", __func__, __LINE__,
			BMT_status.SOFTCHARE3_0_total_charging_time, 
			BMT_status.SOFTCHARE3_0_40_init_time, BMT_status.SOFTCHARE3_0_40_charging_time,
			BMT_status.SOFTCHARE3_0_30_init_time, BMT_status.SOFTCHARE3_0_30_charging_time,
			BMT_status.SOFTCHARE3_0_20_init_time, BMT_status.SOFTCHARE3_0_20_charging_time);
		/*[Arima_8100][bozhi_lin] RID001972 - Soft charge version 1.0 20161124 begin*/
		battery_log(BAT_LOG_CRTI, "[B]%s(%d): SOFTCHARGE1.0 (%d, %d)\n", __func__, __LINE__,
			BATTERY_DESIGN_CAPACITY_90_PERCENT, BMT_status.SOFTCHARE3_0_average_fcc_mah);
		/*[Arima_8100][bozhi_lin] 20161124 end*/
		if (BMT_status.SOFTCHARE3_0_total_charging_time > SOFTCHARE3_0_TOTAL_CHARGING_TIME) {
			cv_voltage = BATTERY_VOLT_04_300000_V;
		/*[Arima_8100][bozhi_lin] RID001972 - Soft charge version 1.0 20161124 begin*/
		} else if ((BMT_status.SOFTCHARE3_0_average_fcc_mah > 0) &&
			(BMT_status.SOFTCHARE3_0_average_fcc_mah <= BATTERY_DESIGN_CAPACITY_90_PERCENT)) {
			cv_voltage = BATTERY_VOLT_04_300000_V;
		/*[Arima_8100][bozhi_lin] 20161124 end*/
		} else {
			cv_voltage = BATTERY_VOLT_04_350000_V;
		}	
	#else
/*[Arima_8100][bozhi_lin] set battery cv voltage to 4.35V 20161006 begin*/
	if (batt_cust_data.high_battery_voltage_support)
		cv_voltage = BATTERY_VOLT_04_350000_V;
	else
		cv_voltage = BATTERY_VOLT_04_200000_V;
/*[Arima_8100][bozhi_lin] 20161006 end*/
	#endif
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/



	ret = mtk_get_dynamic_cv(&dynamic_cv);
	if (ret == 0) {
		cv_voltage = dynamic_cv * 1000;
		battery_log(BAT_LOG_FULL, "%s: set dynamic cv = %dmV\n",
			__func__, dynamic_cv);
	}

	battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);

#if defined(CONFIG_MTK_HAFG_20)
	g_cv_voltage = cv_voltage;
#endif
}

static void pchr_turn_on_charging(void)
{
	u32 charging_enable = KAL_TRUE;

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
	if (BMT_status.charger_exist)
		charging_enable = KAL_TRUE;
	else
		charging_enable = KAL_FALSE;
#endif

	if (BMT_status.bat_charging_state == CHR_ERROR) {
		battery_log(BAT_LOG_CRTI,
			"[BATTERY] Charger Error, turn OFF charging !\n");
		charging_enable = KAL_FALSE;
	} else if ((g_platform_boot_mode == META_BOOT) ||
		   (g_platform_boot_mode == ADVMETA_BOOT)) {
		battery_log(BAT_LOG_CRTI,
			"[BATTERY] In meta or advanced meta mode, disable charging.\n");
		charging_enable = KAL_FALSE;
	} else {
		/* HW initialization */
		battery_charging_control(CHARGING_CMD_INIT, NULL);
		battery_log(BAT_LOG_FULL, "charging_hw_init\n");

		/* PE+/PE+20 algorithm */
		mtk_pep20_start_algorithm();
		mtk_pep_start_algorithm();

		/* Select ICHG/AICR */
		mtk_select_ichg_aicr();

		if (g_temp_CC_value == CHARGE_CURRENT_0_00_MA ||
		    g_temp_input_CC_value == CHARGE_CURRENT_0_00_MA) {
			charging_enable = KAL_FALSE;
			battery_log(BAT_LOG_CRTI,
				"[BATTERY] charging current is set 0mA, turn off charging !\r\n");
		} else /* Set CV Voltage */
			mtk_select_cv();
	}

	/* enable/disable charging */
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	battery_log(BAT_LOG_FULL,
		"[BATTERY] pchr_turn_on_charging(), enable =%d !\r\n",
		charging_enable);
}


PMU_STATUS BAT_PreChargeModeAction(void)
{
#ifdef CONFIG_MTK_BIF_SUPPORT
	int ret = 0;
	bool bif_exist = false;
#endif
	unsigned int led_en = true;
/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)
	unsigned int te_shutdown_enable = KAL_FALSE;
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/

	battery_log(BAT_LOG_CRTI, "[BATTERY] Pre-CC mode charge, timer=%d on %d !!\n\r",
		    BMT_status.PRE_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time += BAT_TASK_PERIOD;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;
/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)
	BMT_status.MAINTAIN_charging_time = 0;
	
	te_shutdown_enable = KAL_TRUE;
	battery_charging_control(CHARGING_CMD_ENABLE_TE_SHUTDOWN,&te_shutdown_enable);
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/

#ifdef CONFIG_MTK_BIF_SUPPORT
	/* If defined BIF but not BIF's battery, stop charging */
	ret = battery_charging_control(CHARGING_CMD_GET_BIF_IS_EXIST,
		&bif_exist);
	if (!bif_exist) {
		battery_log(BAT_LOG_CRTI,
			"%s: define BIF but no BIF battery, disable charging\n",
			__func__);
		BMT_status.bat_charging_state = CHR_ERROR;
		return PMU_STATUS_OK;
	}
#endif

	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	/*  Enable charger */
	pchr_turn_on_charging();
#if defined(CONFIG_MTK_HAFG_20)
	if (BMT_status.UI_SOC2 == 100 && charging_full_check()) {
#else
	if (BMT_status.UI_SOC == 100) {
#endif
		BMT_status.bat_charging_state = CHR_BATFULL;
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	} else if (BMT_status.bat_vol > V_PRE2CC_THRES) {
		BMT_status.bat_charging_state = CHR_CC;
	}

	/* If it is not disabled by throttling,
	 * enable PE+/PE+20, if it is disabled
	 */
	if (g_bcct_input_flag && g_bcct_input_value < 300)
		return PMU_STATUS_OK;

	if (!mtk_pep20_get_is_enable()) {
		mtk_pep20_set_is_enable(true);
		mtk_pep20_set_to_check_chr_type(true);
	}
	if (!mtk_pep_get_is_enable()) {
		mtk_pep_set_is_enable(true);
		mtk_pep_set_to_check_chr_type(true);
	}

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_ConstantCurrentModeAction(void)
{
	unsigned int led_en = true;
/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)
	unsigned int te_shutdown_enable = KAL_FALSE;
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/

	battery_log(BAT_LOG_CRTI, "[BATTERY] CC mode charge, timer=%d on %d!!\n",
		    BMT_status.CC_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time += BAT_TASK_PERIOD;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;
/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)
	BMT_status.MAINTAIN_charging_time = 0;
	
	te_shutdown_enable = KAL_TRUE;
	battery_charging_control(CHARGING_CMD_ENABLE_TE_SHUTDOWN,&te_shutdown_enable);
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/

/*[Arima_8100][bozhi_lin] RID001582 - Soft charge 3.0 20161116 begin*/
//Change charging voltage from 4.35V to 4.30V when accumulated time of high voltage and high temperature on embedded battery reaches specified condition.
//a). 20deg.C~30deg.C & >4.25V (Accelerated factor : 8.32)
//b). 30deg.C~40deg.C & >4.25V (Accelerated factor : 2.73)
//c). >40deg.C        & >4.25V (Accelerated factor : 1.61)
//a) + b) + c) = 250[hour], then charging voltage 4.35V to 4.30V
#if defined(CONFIG_CHARGING_SOFTCHARE3_0)
	if (BMT_status.bat_vol > V_SOFTCHARE3_0_THRES) {
		if (BMT_status.temperature > SOFTCHARE3_0_TEMP_POS_40_THRESHOLD) {
			BMT_status.SOFTCHARE3_0_40_charging_time += BAT_TASK_PERIOD;
		} else if (BMT_status.temperature > SOFTCHARE3_0_TEMP_POS_30_THRESHOLD) {
			BMT_status.SOFTCHARE3_0_30_charging_time += BAT_TASK_PERIOD;
		} else if (BMT_status.temperature > SOFTCHARE3_0_TEMP_POS_20_THRESHOLD) {
			BMT_status.SOFTCHARE3_0_20_charging_time += BAT_TASK_PERIOD;
		}
		BMT_status.SOFTCHARE3_0_total_charging_time = 
			((BMT_status.SOFTCHARE3_0_40_init_time + BMT_status.SOFTCHARE3_0_40_charging_time) / SOFTCHARE3_0_TEMP_POS_40_FACTOR) +
			((BMT_status.SOFTCHARE3_0_30_init_time + BMT_status.SOFTCHARE3_0_30_charging_time) / SOFTCHARE3_0_TEMP_POS_30_FACTOR) +
			((BMT_status.SOFTCHARE3_0_20_init_time + BMT_status.SOFTCHARE3_0_20_charging_time) / SOFTCHARE3_0_TEMP_POS_20_FACTOR);
	}
#endif
/*[Arima_8100][bozhi_lin] 20161116 end*/

	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	/*  Enable charger */
	pchr_turn_on_charging();

/*[Arima_8100][bozhi_lin] add check battery capacity reach 100% will enter maintain 20170125 begin*/
/*[Arima_8100][bozhi_lin] add check battery capacity reach 100% will enter maintain 20161229 begin*/
#if 0
	if (BMT_status.UI_SOC2 == 100 && charging_full_check()) {
#else
	if (charging_full_check() == KAL_TRUE) {
#endif
/*[Arima_8100][bozhi_lin] 20161229 end*/
/*[Arima_8100][bozhi_lin] 20170125 end*/
		BMT_status.bat_charging_state = CHR_BATFULL;
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	}

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryFullAction(void)
{
	unsigned int led_en = false;
/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)
	unsigned int charging_enable = KAL_FALSE;
	unsigned int te_shutdown_enable = KAL_FALSE;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Battery full, enter MAINTAIN state, timer=%d\n",
		    BMT_status.MAINTAIN_charging_time);
#else
	battery_log(BAT_LOG_CRTI, "[BATTERY] Battery full !!\n\r");
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/

	BMT_status.bat_full = KAL_TRUE;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
/*[Arima_8100][bozhi_lin] add check battery capacity reach 100% will enter maintain 20170125 begin*/
/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)
	if ((BMT_status.UI_SOC2 != 100) && (BMT_status.MAINTAIN_charging_time == 0)) {
		BMT_status.MAINTAIN_charging_time = 0;
		battery_log(BAT_LOG_CRTI, "[BATTERY] BMT_status.UI_SOC2=%d, not reach 100, can not enter MAINTAIN state\n",
		    BMT_status.UI_SOC2);
	} else {
		BMT_status.MAINTAIN_charging_time += BAT_TASK_PERIOD;
	}
	BMT_status.bat_in_maintain_state = KAL_FALSE;
#else
	BMT_status.bat_in_recharging_state = KAL_FALSE;
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/
/*[Arima_8100][bozhi_lin] 20170125 end*/

	battery_log(BAT_LOG_FULL, "Turn off PWRSTAT LED\n");
	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	/*
	 * If CV is set to lower value by JEITA,
	 * Reset CV to normal value if temperture is in normal zone
	 */
	mtk_select_cv();

/*[Arima_8100][bozhi_lin] charging maintenance implement 20161206 begin*/
/*[Arima_8100][bozhi_lin] charging maintenance implement 20161104 begin*/
/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)
	//battery_log(BAT_LOG_CRTI, "[B]%s(%d): Battery Maintain !!\n", __func__, __LINE__);

	if (BMT_status.MAINTAIN_charging_time == BAT_TASK_PERIOD) {
		BMT_status.bat_in_maintain_state = KAL_TRUE;

		te_shutdown_enable = KAL_FALSE;
		battery_charging_control(CHARGING_CMD_ENABLE_TE_SHUTDOWN,&te_shutdown_enable);
	}

	if ((BMT_status.MAINTAIN_charging_time > TOTAL_MAINTAIN_CHARGING_TIME) ||
			(BMT_status.bat_vol < V_FULL2CC_THRES)) {
		te_shutdown_enable = KAL_TRUE;
		battery_charging_control(CHARGING_CMD_ENABLE_TE_SHUTDOWN,&te_shutdown_enable);

		/*  Disable charger */
		charging_enable = KAL_FALSE;
		battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

		if (charging_full_check() == KAL_FALSE) {
			battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Maintain Stop!! Enter CC Mode\n\r");

			BMT_status.bat_in_maintain_state = KAL_FALSE;
			BMT_status.bat_charging_state = CHR_CC;
		}
	}
#else
	if (charging_full_check() == KAL_FALSE) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Re-charging !!\n\r");

		BMT_status.bat_in_recharging_state = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_CC;
#ifndef CONFIG_MTK_HAFG_20
		battery_meter_reset();
#endif
		mtk_pep20_set_to_check_chr_type(true);
		mtk_pep_set_to_check_chr_type(true);
		g_enable_dynamic_cv = true;
	}
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/
/*[Arima_8100][bozhi_lin] 20161104 end*/
/*[Arima_8100][bozhi_lin] 20161206 end*/


	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryHoldAction(void)
{
	unsigned int charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Hold mode !!\n\r");

	if (BMT_status.bat_vol < TALKING_RECHARGE_VOLTAGE || g_call_state == CALL_IDLE) {
		BMT_status.bat_charging_state = CHR_CC;
		battery_log(BAT_LOG_CRTI, "[BATTERY] Exit Hold mode and Enter CC mode !!\n\r");
	}

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	return PMU_STATUS_OK;
}

/*[Arima_8100][bozhi_lin] FP22589: Battery Swelling mitigation for retail demo units 20161027 begin*/
#if defined(CONFIG_STOP_CHARGING_IN_DEMOAPP)
PMU_STATUS BAT_BatteryHoldDemoAction(void)
{
	unsigned int charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Hold Demo mode !!\n\r");

	if (batt_cust_data.stop_charging_in_demoapp) {
		if (BMT_status.UI_SOC2 < DEMOAPP_RECHARGE_SOC) {
			BMT_status.bat_charging_state = CHR_CC;
			battery_log(BAT_LOG_CRTI, "[BATTERY] Exit Hold Demo mode and Enter CC mode !!\n\r");
		}
	}

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	return PMU_STATUS_OK;
}
#endif
/*[Arima_8100][bozhi_lin] 20161027 end*/

PMU_STATUS BAT_BatteryStatusFailAction(void)
{
	unsigned int charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] BAD Battery status... Charging Stop !!\n\r");

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
/*[Arima_8100][bozhi_lin] Charging enable JEITA battery temperature check 20161028 begin*/
#if defined(JEITA_SONY_CONFIG)
	if ((g_temp_status == TEMP_ABOVE_POS_55) || (g_temp_status == TEMP_BELOW_POS_5))
		temp_error_recovery_chr_flag = KAL_FALSE;
#else
	if ((g_temp_status == TEMP_ABOVE_POS_60) || (g_temp_status == TEMP_BELOW_NEG_10))
		temp_error_recovery_chr_flag = KAL_FALSE;
#endif
/*[Arima_8100][bozhi_lin] 20161028 end*/

/*[Arima_8100][bozhi_lin] Charging enable JEITA battery temperature check 20161028 begin*/
#if defined(JEITA_SONY_CONFIG)
	if ((temp_error_recovery_chr_flag == KAL_FALSE) && (g_temp_status != TEMP_ABOVE_POS_55)
	    && (g_temp_status != TEMP_BELOW_POS_5)) {
		temp_error_recovery_chr_flag = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_PRE;
	}
#else
	if ((temp_error_recovery_chr_flag == KAL_FALSE) && (g_temp_status != TEMP_ABOVE_POS_60)
	    && (g_temp_status != TEMP_BELOW_NEG_10)) {
		temp_error_recovery_chr_flag = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_PRE;
	}
#endif
/*[Arima_8100][bozhi_lin] 20161028 end*/
#endif

	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
/*[Arima_8100][bozhi_lin] charging maintenance implement 20161102 begin*/
#if defined(CHARGING_MAINTAIN)
	BMT_status.MAINTAIN_charging_time = 0;
#endif
/*[Arima_8100][bozhi_lin] 20161102 end*/

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	/* Disable PE+/PE+20 */
	if (mtk_pep20_get_is_enable()) {
		mtk_pep20_set_is_enable(false);
		if (mtk_pep20_get_is_connect())
			mtk_pep20_reset_ta_vchr();
	}
	if (mtk_pep_get_is_enable()) {
		mtk_pep_set_is_enable(false);
		if (mtk_pep_get_is_connect())
			mtk_pep_reset_ta_vchr();
	}

	return PMU_STATUS_OK;
}


void mt_battery_charging_algorithm(void)
{
/*[Arima_8100][bozhi_lin] RID003699 OVP warning message to end-user 20170320 begin*/
	{
		unsigned int charging_vbus_ovp_enable;
		charging_vbus_ovp_enable = KAL_FALSE;
		battery_charging_control(CHARGING_CMD_SET_VBUS_OVP_EN, &charging_vbus_ovp_enable);
	}
/*[Arima_8100][bozhi_lin] 20170320 end*/

	battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, NULL);

	/* Generate AICR upper bound by AICL */
	if (!mtk_is_pep_series_connect()) {
		battery_charging_control(CHARGING_CMD_RUN_AICL,
			&g_aicr_upper_bound);
	}

	mtk_pep20_check_charger();
	mtk_pep_check_charger();
	switch (BMT_status.bat_charging_state) {
	case CHR_PRE:
		BAT_PreChargeModeAction();
		break;

	case CHR_CC:
		BAT_ConstantCurrentModeAction();
		break;

	case CHR_BATFULL:
		BAT_BatteryFullAction();
		break;

	case CHR_HOLD:
		BAT_BatteryHoldAction();
		break;

/*[Arima_8100][bozhi_lin] FP22589: Battery Swelling mitigation for retail demo units 20161027 begin*/
#if defined(CONFIG_STOP_CHARGING_IN_DEMOAPP)
	case CHR_HOLD_DEMO:
		BAT_BatteryHoldDemoAction();
		break;
#endif
/*[Arima_8100][bozhi_lin] 20161027 end*/

	case CHR_ERROR:
		BAT_BatteryStatusFailAction();
		break;
	}

/*[Arima_8100][bozhi_lin] RID003699 OVP warning message to end-user 20170320 begin*/
#if 0
	battery_charging_control(CHARGING_CMD_DUMP_REGISTER, NULL);
#endif
/*[Arima_8100][bozhi_lin] 20170320 end*/
}
