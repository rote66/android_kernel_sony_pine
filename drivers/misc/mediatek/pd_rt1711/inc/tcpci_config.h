/*
 * Copyright (C) 2016 Richtek Technology Corp.
 *
 * Author: TH <tsunghan_tsai@richtek.com>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_TCPC_CONFIG_H
#define __LINUX_TCPC_CONFIG_H

/* default config */

/*[Arima_8100][bozhi_lin] disable rt1711 pd_dbg_info 20170112 begin*/
//#define CONFIG_PD_DBG_INFO
/*[Arima_8100][bozhi_lin] 20170112 end*/

/* #define CONFIG_TYPEC_USE_DIS_VBUS_CTRL */
#define CONFIG_TYPEC_POWER_CTRL_INIT

#define CONFIG_TYPEC_CAP_TRY_SOURCE
#define CONFIG_TYPEC_CAP_TRY_SINK

/* #define CONFIG_TYPEC_CAP_DBGACC_SNK */
/* #define CONFIG_TYPEC_CAP_CUSTOM_SRC */

/* #define CONFIG_TYPEC_CHECK_CC_STABLE */
/* #define CONFIG_TYPEC_ATTACHED_SRC_SAFE0V_DELAY */
#define CONFIG_TYPEC_ATTACHED_SRC_SAFE0V_TIMEOUT

#define CONFIG_TYPEC_CHECK_LEGACY_CABLE
/* #define CONFIG_TYPEC_CHECK_SRC_UNATTACH_OPEN */

#define CONFIG_TYPEC_CAP_RA_DETACH
#define CONFIG_TYPEC_CAP_LPM_WAKEUP_WATCHDOG

#define CONFIG_TYPEC_CAP_POWER_OFF_CHARGE

#define CONFIG_TYPEC_CAP_ROLE_SWAP
#define CONFIG_TYPEC_CAP_ROLE_SWAP_TOUT		1200

/* #define CONFIG_TYPEC_CAP_AUTO_DISCHARGE */
#define CONFIG_TYPEC_CAP_AUTO_DISCHARGE_TOUT	50

#define CONFIG_TYPEC_CAP_AUDIO_ACC_SINK_VBUS

/* #define CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SNK */
/* #define CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SRC */

/*
 * USB 2.0 & 3.0 current
 * Unconfigured :	100 / 150 mA
 * Configured :		500 / 900 mA
 */

#define CONFIG_TYPEC_SNK_CURR_DFT		150
#define CONFIG_TYPEC_SRC_CURR_DFT		500
#define CONFIG_TYPEC_SNK_CURR_LIMIT		0

/* #define CONFIG_TCPC_FORCE_DISCHARGE_IC */
/* #define CONFIG_TCPC_FORCE_DISCHARGE_EXT */

/* #define CONFIG_TCPC_AUTO_DISCHARGE_IC */
/* #define CONFIG_TCPC_AUTO_DISCHARGE_EXT */

#define CONFIG_TCPC_VSAFE0V_DETECT
/* #define CONFIG_TCPC_VSAFE0V_DETECT_EXT */
#define CONFIG_TCPC_VSAFE0V_DETECT_IC

#define CONFIG_TCPC_LPM_CONFIRM
#define CONFIG_TCPC_LPM_POSTPONE

#define CONFIG_TCPC_LOW_POWER_MODE
/* #define CONFIG_TCPC_IDLE_MODE */
#define CONFIG_TCPC_CLOCK_GATING

/* #define CONFIG_TCPC_WATCHDOG_EN */
/* #define CONFIG_TCPC_INTRST_EN */
#define CONFIG_TCPC_I2CRST_EN

#define CONFIG_TCPC_SHUTDOWN_CC_DETACH
#define CONFIG_TCPC_SHUTDOWN_VBUS_DISABLE

/* debug config */
/* #define CONFIG_USB_PD_DBG_ALERT_STATUS */
/* #define CONFIG_USB_PD_DBG_SKIP_ALERT_HANDLER */

#endif /* __LINUX_TCPC_CONFIG_H */
