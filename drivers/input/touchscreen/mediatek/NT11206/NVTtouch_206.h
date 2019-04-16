 /*
 * Copyright (C) 2010 - 2016 Novatek, Inc.
 *
 * $Revision: 8441 $
 * $Date: 2017-01-03 17:00:50 +0800 (週二, 03 一月 2017) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/i2c.h>
#include <linux/input.h>

#define NVT_DEBUG 0

#define NVT_GPIO_AS_INT(pin) tpd_gpio_as_int(pin)
#define NVT_GPIO_OUTPUT(pin, level) tpd_gpio_output(pin, level)


//---INT trigger mode---
//#define EINTF_TRIGGER_RISING     0x00000001
//#define EINTF_TRIGGER_FALLING    0x00000002
//#define EINTF_TRIGGER_HIGH       0x00000004
//#define EINTF_TRIGGER_LOW        0x00000008
/*[Arima_8100][allen_yu] change interrupt trigger from high trigger to low trigger 20170413 begin*/
#define INT_TRIGGER_TYPE IRQF_TRIGGER_FALLING
/*[Arima_8100][allen_yu] 20170413 end*/

//---I2C driver info.---
#define NVT_I2C_NAME "NVT-ts"
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x62

#if NVT_DEBUG
#define NVT_LOG(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    pr_info("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"

//---Touch info.---
#define TOUCH_MAX_WIDTH 720
#define TOUCH_MAX_HEIGHT 1280
/*[Arima_8100][allen_yu] change support maximum fingers from 10 fingers to 5 fingers 20170413 begin*/
#define TOUCH_MAX_FINGER_NUM 5
/*[Arima_8100][allen_yu] 20170413 end*/
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif

//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 1
#define MT_PROTOCOL_B 1
#define WAKEUP_GESTURE 0
#if WAKEUP_GESTURE
extern const uint16_t gesture_key_array[];
#endif

/*[Arima_8100][bozhi_lin] upgrade Novatek nt11206 touch firmware to V07 20170503 begin*/
/*[Arima_8100][allen_yu] upgrade Novatek nt11206 touch firmware to V06 20170414 begin*/
/*[Arima_8100][allen_yu] upgrade Novatek nt11206 touch firmware to V05 20170413 begin*/
/*[Arima_8100][bozhi_lin] upgrade Novatek nt11206 touch firmware to V04 20170411 begin*/
/*[Arima_8100][bozhi_lin] upgrade Novatek nt11206 touch firmware to V03 20170313 begin*/
/*[Arima_8100][bozhi_lin] enable Novatek nt11206 touch driver firmware auto-upgrade 20170222 begin*/
#if 1
#define BOOT_UPDATE_FIRMWARE 1
#define BOOT_UPDATE_FIRMWARE_NAME "NT11206_INX_0550_HD_SM31_20170425_V07.bin"
#else
#define BOOT_UPDATE_FIRMWARE 0
#define BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw.bin"
#endif
/*[Arima_8100][bozhi_lin] 20170222 end*/
/*[Arima_8100][bozhi_lin] 20170313 end*/
/*[Arima_8100][bozhi_lin] 20170411 end*/
/*[Arima_8100][allen_yu] 20170413 end*/
/*[Arima_8100][allen_yu] 20170414 end*/
/*[Arima_8100][bozhi_lin] 20170503 end*/
#define POINT_DATA_CHECKSUM 0
#define MTK_REGULATOR 1

struct nvt_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct nvt_work;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	int8_t phys[32];
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t x_num;
	uint8_t y_num;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
/*[Arima_8100][allen_yu] 2nd Modify the setting of power-on 20170417 begin*/
#if MTK_REGULATOR
	bool power_on;
#endif
/*[Arima_8100][allen_yu] 20170417 end*/
	int32_t irq_gpio;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	struct mutex lock;
	/*[Arima_8100][allen_yu] 2nd Modify the setting of power-on 20170417 begin*/
	//bool power_on;
	/*[Arima_8100][allen_yu] 20170417 end*/
};

#if NVT_TOUCH_PROC
struct nvt_flash_data{
	rwlock_t lock;
	struct i2c_client *client;
};
#endif

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN	// normal run
} RST_COMPLETE_STATE;

#ifdef CONFIG_MTK_I2C_EXTENSION
#define TPD_SUPPORT_I2C_DMA         1
#else
#define TPD_SUPPORT_I2C_DMA         0
#endif

#define IIC_DMA_MAX_TRANSFER_SIZE     250
/*[Arima_8100][bozhi_lin] set Novatek nt11206 touch clock rate to 400KHz 20170308 begin*/
#define I2C_MASTER_CLOCK              400
/*[Arima_8100][bozhi_lin] 20170308 end*/

#define GTP_ADDR_LENGTH             1

#define _ERROR(e)      ((0x01 << e) | (0x01 << (sizeof(s32) * 8 - 1)))
#define ERROR          _ERROR(1)	/*for common use */
#define ERROR_IIC      _ERROR(2)	/*IIC communication error. */

#endif /* _LINUX_NVT_TOUCH_H */
