/*
 * Copyright (C) 2010 - 2016 Novatek, Inc.
 *
 * $Revision: 9298 $
 * $Date: 2017-02-06 16:30:33 +0800 (週一, 06 二月 2017) $
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/input/mt.h>
#include <linux/wakelock.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
/*[Arima_8100][allen_yu] 2nd Modify the setting of suspend sequence 20170418 begin*/
//static bool gtp_suspend = false;
/*[Arima_8100][allen_yu] 20170418 end*/
#include "tpd.h"
#include "NVTtouch_206.h"

#if NVT_TOUCH_EXT_PROC
extern int32_t nvt_extra_proc_init(void);
#endif

#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init(void);
#endif

struct nvt_ts_data *ts;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

#if WAKEUP_GESTURE
const uint16_t gesture_key_array[] = {
	KEY_POWER,  //GESTURE_WORD_C
	KEY_POWER,  //GESTURE_WORD_W
	KEY_POWER,  //GESTURE_WORD_V
	KEY_POWER,  //GESTURE_DOUBLE_CLICK
	KEY_POWER,  //GESTURE_WORD_Z
	KEY_POWER,  //GESTURE_WORD_M
	KEY_POWER,  //GESTURE_WORD_O
	KEY_POWER,  //GESTURE_WORD_e
	KEY_POWER,  //GESTURE_WORD_S
	KEY_POWER,  //GESTURE_SLIDE_UP
	KEY_POWER,  //GESTURE_SLIDE_DOWN
	KEY_POWER,  //GESTURE_SLIDE_LEFT
	KEY_POWER,  //GESTURE_SLIDE_RIGHT
};
#endif

static uint8_t bTouchIsAwake = 0;
static int tpd_flag = 0;
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

#if TPD_SUPPORT_I2C_DMA
static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
struct mutex dma_mutex;
DEFINE_MUTEX(dma_mutex);
#endif

/*******************************************************
Description:
	Novatek touchscreen i2c read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
#if TPD_SUPPORT_I2C_DMA
s32 i2c_dma_read(struct i2c_client *client, uint16_t addr, uint8_t offset, uint8_t *rxbuf, uint16_t len)
{
    int ret;
    uint8_t retry = 0;
	uint8_t buf[2] = {offset,0};

    struct i2c_msg msg[2] =
    {
        {
            .addr = (addr & I2C_MASK_FLAG),
            .flags = 0,
            .buf = buf,
            .len = 1,
            .timing = I2C_MASTER_CLOCK
        },
        {
            .addr = (addr & I2C_MASK_FLAG),
            .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = I2C_M_RD,
            .buf = (uint8_t*)gpDMABuf_pa,     
            .len = len,
            .timing = I2C_MASTER_CLOCK
        },
    };
    
    if (rxbuf == NULL)
        return -1;

    //dev_info(&ts->client->dev,"dma i2c read: 0x%04X, %d bytes(s)", addr, len);
    for (retry = 0; retry < 20; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg[0], 2);
        if (ret < 0)
        {
            continue;
        }
        memcpy(rxbuf, gpDMABuf_va, len);
        return 2;
    }
	
    dev_err(&ts->client->dev,"Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
    return ret;
}

s32 i2c_dma_write(struct i2c_client *client, uint16_t addr, uint8_t offset, uint8_t *txbuf, uint16_t len)
{
    int ret;
    s32 retry = 0;
    uint8_t *wr_buf = gpDMABuf_va;
    
    struct i2c_msg msg =
    {
        .addr = (addr & I2C_MASK_FLAG),
        .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
        .flags = 0,
        .buf = (uint8_t*)gpDMABuf_pa,
        .len = 1 + len,
        .timing = I2C_MASTER_CLOCK
    };
    
    wr_buf[0] = offset;

    if (txbuf == NULL)
        return -1;
    
    //dev_info(&client->dev,"dma i2c write: 0x%04X, %d bytes(s)", addr, len);
    memcpy(wr_buf+1, txbuf, len);
    for (retry = 0; retry < 20; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret < 0)
        {
            continue;
        }
        return 1;
    }
    dev_err(&client->dev,"Dma I2C Write Error: 0x%04X, %d byte(s), err-code: %d", offset, len, ret);
    return ret;
}

s32 i2c_read_bytes_dma(struct i2c_client *client, u16 addr, uint8_t offset, uint8_t *rxbuf, uint16_t len)
{
    uint16_t left = len;
    uint16_t read_len = 0;
    uint8_t *rd_buf = rxbuf;
    s32 ret = 0;    
    
    //dev_info(&client->dev,"Read bytes dma: 0x%04X, %d byte(s)", offset, len);
    while (left > 0)
    {
        if (left > IIC_DMA_MAX_TRANSFER_SIZE)
        {
            read_len = IIC_DMA_MAX_TRANSFER_SIZE;
        }
        else
        {
            read_len = left;
        }
        ret = i2c_dma_read(client, addr, offset, rd_buf, read_len);
        if (ret < 0)
        {
            dev_err(&client->dev,"dma read failed");
            return -1;
        }
        
        left -= read_len;
        offset += read_len;
        rd_buf += read_len;
    }
    return 2;
}

s32 i2c_write_bytes_dma(struct i2c_client *client, u16 addr, uint8_t offset, uint8_t *txbuf, uint16_t len)
{
    s32 ret = 0;
    s32 write_len = 0;
    s32 left = len;
    uint8_t *wr_buf = txbuf;
    
    //dev_info(&client->dev,"Write bytes dma: 0x%04X, %d byte(s)", offset, len);
    while (left > 0)
    {
        if (left > IIC_DMA_MAX_TRANSFER_SIZE)
        {
            write_len = IIC_DMA_MAX_TRANSFER_SIZE;
        }
        else
        {
            write_len = left;
        }
        ret = i2c_dma_write(client, addr, offset, wr_buf, write_len);
        
        if (ret < 0)
        {
            dev_err(&client->dev,"dma i2c write failed!");
            return -1;
        }
        
        left -= write_len;
        offset += write_len;
        wr_buf += write_len;
    }
    return 1;
}
#endif

#if TPD_SUPPORT_I2C_DMA
int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
    return i2c_read_bytes_dma(client, address, buf[0], &buf[1], len-1);
}
#else
int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msgs[2];
	int32_t ret = -1;
	int32_t retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len - 1;
	msgs[1].buf   = &buf[1];

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen i2c dummy read function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ_DUMMY(struct i2c_client *client, uint16_t address)
{
	uint8_t buf[8] = {0};
	int32_t ret = -1;

	ret = CTP_I2C_READ(client, address, buf, 2);
	if (ret < 0)
		NVT_ERR("CTP_I2C_READ_DUMMY failed.(%d)\n", ret);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
#if TPD_SUPPORT_I2C_DMA
int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
    return i2c_write_bytes_dma(client, address, buf[0], &buf[1], len-1);
}
#else
int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msg;
	int32_t ret = -1;
	int32_t retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}
#endif


/*******************************************************
Description:
	Novatek touchscreen IC hardware reset function.

return:
	n.a.
*******************************************************/
void nvt_hw_reset(void)
{
	//---trigger rst-pin to reset---
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 1);
	msleep(20);
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(20);
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 1);
	msleep(20);
}

/*******************************************************
Description:
	Novatek touchscreen set i2c debounce function.

return:
	n.a.
*******************************************************/
void nvt_set_i2c_debounce(void)
{
	uint8_t buf[8] = {0};
	uint8_t reg1_val = 0;
	uint8_t reg2_val = 0;
	uint32_t retry = 0;

	do {
		msleep(10);

		// set xdata index to 0x1F000
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF0;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		// set i2c debounce 34ns
		buf[0] = 0x15;
		buf[1] = 0x17;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		buf[0] = 0x15;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
		reg1_val = buf[1];

		// set schmitt trigger enable
		buf[0] = 0x3E;
		buf[1] = 0x07;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		buf[0] = 0x3E;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
		reg2_val = buf[1];
	} while (((reg1_val != 0x17) || (reg2_val != 0x07)) && (retry++ < 20));

	if(retry > 20) {
		NVT_ERR("set i2c debounce failed, reg1_val=0x%02X, reg2_val=0x%02X\n", reg1_val, reg2_val);
	}
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	uint8_t buf[4]={0};

	//---write i2c cmds to reset idle---
	buf[0]=0x00;
	buf[1]=0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(10);

	nvt_set_i2c_debounce();
}

/*[Arima_8100][bozhi_lin] fine tune nt11206 touch resume performance 20170504 begin*/
/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
	without i2c debounce function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle_no_i2c_debounce(void)
{
	uint8_t buf[4]={0};

	//---write i2c cmds to reset idle---
	buf[0]=0x00;
	buf[1]=0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(10);
}
/*[Arima_8100][bozhi_lin] 20170504 end*/

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	uint8_t buf[8] = {0};

	//---write i2c cmds to reset---
	buf[0] = 0x00;
	buf[1] = 0x69;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	// need 50ms delay after bootloader reset
	msleep(50);
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	for (i = 0; i < retry; i++) {
		//---set xdata index to 0x14700---
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0x47;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---clear fw status---
		buf[0] = 0x51;
		buf[1] = 0x00;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		//---read fw status---
		buf[0] = 0x51;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	for (i = 0; i < retry; i++) {
		//---set xdata index to 0x14700---
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0x47;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---read fw status---
		buf[0] = 0x51;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	while (1) {
		msleep(10);

		//---read reset state---
		buf[0] = 0x60;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] >= check_reset_state) && (buf[1] < 0xFF)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > 100)) {
			ret = -1;
			NVT_ERR("error, retry=%d, buf[1]=0x%02X\n", retry, buf[1]);
			break;
		}
	}

	return ret;
}

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_PROC
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTflash"

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash read function.

return:
	Executive outcomes. 2---succeed. -5,-14---failed.
*******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t str[68] = {0};
	int32_t ret = -1;
	int32_t retries = 0;
	int8_t i2c_wr = 0;

	if (count > sizeof(str)) {
		NVT_ERR("error count=%zu\n", count);
		return -EFAULT;
	}

	if (copy_from_user(str, buff, count)) {
		NVT_ERR("copy from user error\n");
		return -EFAULT;
	}

	i2c_wr = str[0] >> 7;

	if (i2c_wr == 0) {	//I2C write

		while (retries < 20) {
			ret = CTP_I2C_WRITE(ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 1)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else if (i2c_wr == 1) {	//I2C read
		while (retries < 20) {
			ret = CTP_I2C_READ(ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 2)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		// copy buff to user if i2c transfer
		if (retries < 20) {
			if (copy_to_user(buff, str, count))
				return -EFAULT;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else {
		NVT_ERR("Call error, str[0]=%d\n", str[0]);
		return -EFAULT;
	}
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash open function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		NVT_ERR("Failed to allocate memory for nvt flash data\n");
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash close function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev)
		kfree(dev);

	return 0;
}

static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.read = nvt_flash_read,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_proc_init(void)
{
	NVT_proc_entry = proc_create(DEVICE_NAME, 0444, NULL,&nvt_flash_fops);
	if (NVT_proc_entry == NULL) {
		NVT_ERR("Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("Succeeded!\n");
	}

	NVT_LOG("============================================================\n");
	NVT_LOG("Create /proc/NVTflash\n");
	NVT_LOG("============================================================\n");

	return 0;
}
#endif	//NVT_TOUCH_PROC

#if WAKEUP_GESTURE
#define GESTURE_WORD_C			12
#define GESTURE_WORD_W			13
#define GESTURE_WORD_V			14
#define GESTURE_DOUBLE_CLICK	15
#define GESTURE_WORD_Z			16
#define GESTURE_WORD_M			17
#define GESTURE_WORD_O			18
#define GESTURE_WORD_e			19
#define GESTURE_WORD_S			20
#define GESTURE_SLIDE_UP		21
#define GESTURE_SLIDE_DOWN		22
#define GESTURE_SLIDE_LEFT		23
#define GESTURE_SLIDE_RIGHT		24

static struct wake_lock gestrue_wakelock;
static uint8_t bWakeupByGesture = 0;

/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void nvt_ts_wakeup_gesture_report(uint8_t gesture_id)
{
	uint32_t keycode = 0;

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_WORD_C:
			NVT_LOG("Gesture : Word-C.\n");
			keycode = gesture_key_array[0];
			break;
		case GESTURE_WORD_W:
			NVT_LOG("Gesture : Word-W.\n");
			keycode = gesture_key_array[1];
			break;
		case GESTURE_WORD_V:
			NVT_LOG("Gesture : Word-V.\n");
			keycode = gesture_key_array[2];
			break;
		case GESTURE_DOUBLE_CLICK:
			NVT_LOG("Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
			break;
		case GESTURE_WORD_Z:
			NVT_LOG("Gesture : Word-Z.\n");
			keycode = gesture_key_array[4];
			break;
		case GESTURE_WORD_M:
			NVT_LOG("Gesture : Word-M.\n");
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			NVT_LOG("Gesture : Word-O.\n");
			keycode = gesture_key_array[6];
			break;
		case GESTURE_WORD_e:
			NVT_LOG("Gesture : Word-e.\n");
			keycode = gesture_key_array[7];
			break;
		case GESTURE_WORD_S:
			NVT_LOG("Gesture : Word-S.\n");
			keycode = gesture_key_array[8];
			break;
		case GESTURE_SLIDE_UP:
			NVT_LOG("Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			NVT_LOG("Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			NVT_LOG("Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			NVT_LOG("Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
			break;
		default:
			break;
	}

	if (keycode > 0) {
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);

		bWakeupByGesture = 1;
	}
}
#endif	//WAKEUP_GESTURE

#if MTK_REGULATOR
/*******************************************************
Description:
	Novatek touchscreen power on function.

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int32_t nvt_power_on(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

	if (ts->power_on) {
		NVT_LOG("Device already power on\n");
		return 0;
	}

	if (!IS_ERR(tpd->reg)) {
		ret = regulator_enable(tpd->reg);
		if (ret) {
			NVT_ERR("Regulator avdd enable failed ret=%d\n", ret);
			goto err_enable_avdd;
		}
	}

#if 0
	if (!IS_ERR(tpd->io_reg)) {
		ret = regulator_enable(tpd->io_reg);
		if (ret) {
			NVT_ERR("Regulator vdd enable failed ret=%d\n", ret);
			goto err_enable_vdd;
		}
	}
#endif

	ts->power_on = true;
	return 0;

/*[Arima_8100][bozhi_lin] fix coverity issue 20170306 begin*/
#if 0
err_enable_vdd:
	if (!IS_ERR(tpd->reg))
		regulator_disable(tpd->reg);
#endif
/*[Arima_8100][bozhi_lin] 20170306 end*/
err_enable_avdd:
	ts->power_on = false;
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen power off function.

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int32_t nvt_power_off(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

	if (!ts->power_on) {
		NVT_LOG("Device already power off\n");
		return 0;
	}

#if 0
	if (!IS_ERR(tpd->io_reg)) {
		ret = regulator_disable(tpd->io_reg);
		if (ret)
			NVT_ERR("Regulator vdd disable failed ret=%d\n", ret);
	}
#endif

	if (!IS_ERR(tpd->reg)) {
		ret = regulator_disable(tpd->reg);
		if (ret)
			NVT_ERR("Regulator avdd disable failed ret=%d\n", ret);
	}

	ts->power_on = false;
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen power initial function.

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int32_t nvt_power_init(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

	NVT_LOG("Device Tree get regulator!\n");

	/* AVDD */
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	if (IS_ERR(tpd->reg)) {
		ret = PTR_ERR(tpd->reg);
		NVT_ERR("Regulator get failed avdd ret=%d\n", ret);
		return -1;
	}

	ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);
	if (ret) {
		NVT_ERR("Failed to set vtouch voltage: %d!\n", ret);
		return -1;
	}

#if 0
	/* IOVDD */
	tpd->io_reg = regulator_get(tpd->tpd_dev, "vtouchio");
	if (IS_ERR(tpd->io_reg)) {
		ret = PTR_ERR(tpd->io_reg);
		NVT_ERR("Regulator get failed vdd ret=%d\n", ret);
		return -1;
	}

	ret = regulator_set_voltage(tpd->io_reg, 1800000, 1800000);
	if (ret) {
		NVT_ERR("Failed to set vtouchio voltage: %d!\n", ret);
		return -1;
	}
#endif

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen power remove function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_power_remove(struct nvt_ts_data *ts)
{
	regulator_put(tpd->reg);
#if 0
	regulator_put(tpd->io_reg);
#endif

	return 0;
}
#endif	//MTK_REGULATOR


#if POINT_DATA_CHECKSUM
/*******************************************************
Description:
	Novatek touchscreen check i2c packet checksum function.

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int32_t nvt_ts_point_data_checksum(struct i2c_client *client, uint8_t *buf, uint8_t length)
{
	uint8_t checksum = 0;
	int32_t i = 0;

	// Generate checksum
	for (i = 0; i < length; i++) {
		checksum += buf[i+1];
	}
	checksum = (~checksum + 1);

	// Compare ckecksum and dump fail data
	if (checksum != buf[length + 1]) {
		NVT_ERR("i2c packet checksum not match. (point_data[%d]=0x%02X, checksum=0x%02X)\n", (length+1), buf[length+1], checksum);

		for (i = 0; i < 10; i++) {
			NVT_ERR("%02X %02X %02X %02X %02X %02X\n", buf[1+i*6], buf[2+i*6], buf[3+i*6], buf[4+i*6], buf[5+i*6], buf[6+i*6]);
		}

		for (i = 0; i < (length - 60); i++) {
			NVT_ERR("%02X ", buf[1+60+i]);
		}

		return -1;
	}

	return 0;
}
#endif /* POINT_DATA_CHECKSUM */

#define POINT_DATA_LEN 64
/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 4 };

	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 2] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint8_t input_id = 0;
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};

	int32_t i = 0;
	int32_t finger_cnt = 0;

	sched_setscheduler(current, SCHED_RR, &param);
	do
	{
		set_current_state(TASK_INTERRUPTIBLE);

		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);

		mutex_lock(&ts->lock);
		memset(point_data, 0, POINT_DATA_LEN + 2);

		ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data, POINT_DATA_LEN + 2);
		if (ret < 0) {
			NVT_ERR("CTP_I2C_READ failed.(%d)\n", ret);
			goto XFER_ERROR;
		}
/*
		//--- dump I2C buf ---
		for (i = 0; i < 10; i++) {
			printk("%02X %02X %02X %02X %02X %02X  ", point_data[1+i*6], point_data[2+i*6], point_data[3+i*6], point_data[4+i*6], point_data[5+i*6], point_data[6+i*6]);
		}
		printk("\n");
*/
#if POINT_DATA_CHECKSUM
		ret = nvt_ts_point_data_checksum(ts->client, point_data, POINT_DATA_LEN);
		if (ret < 0) {
			goto XFER_ERROR;
		}
#endif /* POINT_DATA_CHECKSUM */

		finger_cnt = 0;
		input_id = (uint8_t)(point_data[1] >> 3);
		memset(press_id, 0, ts->max_touch_num);

		if (bTouchIsAwake == 0) {
#if WAKEUP_GESTURE
			nvt_ts_wakeup_gesture_report(input_id);
#endif
			enable_irq(ts->client->irq);
			mutex_unlock(&ts->lock);
			NVT_LOG("return for interrupt after suspend...\n");
			continue;
		}

#if MT_PROTOCOL_B
		for (i = 0; i < ts->max_touch_num; i++) {
			position = 1 + 6 * i;
			input_id = (uint8_t)(point_data[position + 0] >> 3);
			if (input_id > ts->max_touch_num)
				continue;

			if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
				input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
				input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
				input_w = (uint32_t)(point_data[position + 4]) + 10;
				if (input_w > 255)
					input_w = 255;

/*[Arima_8100][bozhi_lin] fix coverity issue 20170306 begin*/
#if 0
				if ((input_x < 0) || (input_y < 0))
					continue;
#endif
/*[Arima_8100][bozhi_lin] 20170306 end*/
				if ((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
					continue;

				press_id[input_id - 1] = 1;
				input_mt_slot(ts->input_dev, input_id - 1);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);

				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_w);
				finger_cnt++;
			}
		}

		for (i = 0; i < ts->max_touch_num; i++) {
			if (press_id[i] != 1) {
				input_mt_slot(ts->input_dev, i);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			}
		}

		input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));

#else /* #if MT_PROTOCOL_B */

		for (i = 0; i < ts->max_touch_num; i++) {
			position = 1 + 6 * i;
			input_id = (uint8_t)(point_data[position + 0] >> 3);

			if ((point_data[position] & 0x07) == 0x03) {	// finger up (break)
				continue;//input_report_key(ts->input_dev, BTN_TOUCH, 0);
			} else if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
				input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
				input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
				input_w = (uint32_t)(point_data[position + 4]) + 10;
				if (input_w > 255)
					input_w = 255;

				if ((input_x < 0) || (input_y < 0))
					continue;
				if ((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
					continue;

				press_id[input_id - 1] = 1;
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);
				input_report_key(ts->input_dev, BTN_TOUCH, 1);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_w);

				input_mt_sync(ts->input_dev);

				finger_cnt++;
			}
		}
		if (finger_cnt == 0) {
			input_report_key(ts->input_dev, BTN_TOUCH, 0);

			input_mt_sync(ts->input_dev);
		}
#endif	/* #if MT_PROTOCOL_B */


#if TOUCH_KEY_NUM > 0
		if (point_data[61] == 0xF8) {
			for (i = 0; i < ts->max_button_num; i++) {
				input_report_key(ts->input_dev, touch_key_array[i], ((point_data[62] >> i) & 0x01));
			}
		} else {
			for (i = 0; i < ts->max_button_num; i++) {
				input_report_key(ts->input_dev, touch_key_array[i], 0);
			}
		}
#endif

		input_sync(ts->input_dev);

XFER_ERROR:
		enable_irq(ts->client->irq);

		mutex_unlock(&ts->lock);

	} while (!kthread_should_stop());

	return 0;
}

/*******************************************************
Description:
	External interrupt service routine.

return:
	irq execute status.
*******************************************************/
static irqreturn_t nvt_ts_irq_handler(int32_t irq, void *dev_id)
{
	tpd_flag = 1;
	disable_irq_nosync(ts->client->irq);

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		wake_lock_timeout(&gestrue_wakelock, msecs_to_jiffies(5000));
	}
#endif

	wake_up_interruptible(&waiter);

	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Register interrupt handler

return:
	irq execute status.
*******************************************************/
static int nvt_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	NVT_LOG("Device Tree Tpd_irq_registration!\n");

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		//gpio_set_debounce(ints[0], ints[1]);

		ts->client->irq = irq_of_parse_and_map(node, 0);

		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);

		ret = request_irq(ts->client->irq, nvt_ts_irq_handler, ts->int_trigger_type, ts->client->name, ts);
		if (ret > 0) {
			ret = -1;
			NVT_ERR("tpd request_irq IRQ LINE NOT AVAILABLE!.\n");
		}
	} else {
		NVT_ERR("tpd request_irq can not find touch eint device node!.\n");
		ret = -1;
	}
	NVT_LOG("irq:%d, debounce:%d-%d:\n", ts->client->irq, ints[0], ints[1]);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen read chip id function.

return:
	Executive outcomes. 0x26---succeed.
*******************************************************/
static uint8_t nvt_ts_read_chipid(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_HW_Address);

	// reset idle to keep default addr 0x01 to read chipid
	buf[0] = 0x00;
	buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(10);

	for (retry = 5; retry > 0; retry--) {
		//write i2c index to 0x1F000
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF0;
		CTP_I2C_WRITE(ts->client, 0x01, buf, 3);

		//read hw chip id
		buf[0] = 0x00;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, 0x01, buf, 3);

		if (buf[1] == 0x26)
			break;
	}

	return buf[1];
}

/*[Arima_8100][bozhi_lin] Dynamic detect FocalTech and Novatek touch driver 20170222 begin*/
#if defined(TPD_REPORT_VENDOR_FW)
static uint8_t nvt_ts_read_fwver(void)
{
	uint8_t buf[64] = {0};
	int32_t retry = 0;

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_HW_Address);

	for (retry = 5; retry > 0; retry--) {
		//---set xdata index to 0x14700---
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0x47;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---read fw info---
		buf[0] = 0x78;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 17);

		if ((buf[1] + buf[2]) == 0xFF) {
			break;
		} else {
			buf[1] = 0;
		}
	}

	return buf[1];
}
#endif
/*[Arima_8100][bozhi_lin] 20170222 end*/

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t ret = 0;
#if ((TOUCH_KEY_NUM > 0) || WAKEUP_GESTURE)
	int32_t retry = 0;
#endif
/*[Arima_8100][bozhi_lin] Dynamic detect FocalTech and Novatek touch driver 20170222 begin*/
	uint8_t buf_recv[8] = { 0 };
/*[Arima_8100][bozhi_lin] 20170222 end*/

	NVT_LOG("start\n");

	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}
	ts->client = client;
	i2c_set_clientdata(client, ts);

/*[Arima_8100][bozhi_lin] Dynamic detect FocalTech and Novatek touch driver 20170222 begin*/
	if(client->addr != 0x62)
	{
		client->addr = 0x62;
		printk("[B]%s(%d): addr=0x%x\n", __func__, __LINE__, client->addr);
	}
/*[Arima_8100][bozhi_lin] 20170222 end*/

#if MTK_REGULATOR
	ts->power_on = false;

	//---request regulator for MTK---
	ret = nvt_power_init(ts);
	if (ret) {
		NVT_ERR("nvt power init failed\n");
		goto err_power_init;
	}
/*[Arima_8100][allen_yu] 2nd Modify the setting of power on sequence reset in Probe 20170418 begin*/
#if 0
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(10);
#endif
/*[Arima_8100][allen_yu] 20170418 end*/
	//---turn on regulator for MTK---
	ret = nvt_power_on(ts);
	if (ret) {
		NVT_ERR("nvt power on failed\n");
		goto err_nvt_power_on;
	}
#endif
/*[Arima_8100][allen_yu] 2nd Modify the setting of power on sequence reset in Probe 20170418 begin*/
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 0);	
/*[Arima_8100][allen_yu] 20170418 end*/
	//---request RST-pin & INT-pin---
	NVT_GPIO_AS_INT(GTP_INT_PORT);

	//---check i2c func.---
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		NVT_ERR("i2c_check_functionality failed. (no I2C_FUNC_I2C)\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	// need 10ms delay after POR(power on reset)
/*[Arima_8100][allen_yu] 2nd Modify the setting of power on sequence reset in Probe 20170418 begin*/	
#if 1
	//printk("[B]%s(%d): \n", __func__, __LINE__);
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 1);
	msleep(80);
	//printk("[B]%s(%d): \n", __func__, __LINE__);
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(2);

	// power on sequence reset, vdd3.3 high-->low-->high
	if (!IS_ERR(tpd->reg)) {
		ret = regulator_disable(tpd->reg);
		if (ret)
			NVT_ERR("Regulator avdd disable failed ret=%d\n", ret);
	}
	msleep(20);
	if (!IS_ERR(tpd->reg)) {
		ret = regulator_enable(tpd->reg);
		if (ret) {
			NVT_ERR("Regulator avdd enable failed ret=%d\n", ret);
		}
	}
	msleep(20);	

	//printk("[B]%s(%d): \n", __func__, __LINE__);
	//NVT_GPIO_OUTPUT(GTP_RST_PORT, 1);
	//msleep(5);
#else
	msleep(10);
#endif
/*[Arima_8100][allen_yu] 20170418 end*/
/*[Arima_8100][bozhi_lin] Dynamic detect FocalTech and Novatek touch driver 20170222 begin*/
	ret = i2c_master_recv( client, buf_recv, 4 );
	if (ret < 0)
	{
		printk("[B]%s(%d): not Novatek NT11206\n", __func__, __LINE__);
		goto err_check_functionality_failed;
	}
/*[Arima_8100][bozhi_lin] 20170222 end*/

#if TPD_SUPPORT_I2C_DMA
		tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
		gpDMABuf_va = (u8 *) dma_alloc_coherent(&tpd->dev->dev, IIC_DMA_MAX_TRANSFER_SIZE,
			&gpDMABuf_pa, GFP_KERNEL);
		if (!gpDMABuf_va) {
			TPD_DMESG("Allocate DMA I2C Buffer failed!");
			return -1;
		}
		memset(gpDMABuf_va, 0, IIC_DMA_MAX_TRANSFER_SIZE);
#endif
/*[Arima_8100][allen_yu] 2nd Modify the setting of power on sequence reset in Probe 20170418 begin*/
/*[Arima_8100][bozhi_lin] fine tune nt11206 touch resume performance 20170504 begin*/
	nvt_sw_reset_idle_no_i2c_debounce();
/*[Arima_8100][bozhi_lin] 20170504 end*/
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 1);
/*[Arima_8100][allen_yu] 20170418 end*/
	
	//---check chip id---
	ret = nvt_ts_read_chipid();
	if (ret != 0x26) {
		NVT_ERR("nvt_ts_read_chipid is not 0x26. ret=0x%02X\n", ret);
		ret = -EINVAL;
		goto err_chipid_failed;
	}

/*[Arima_8100][bozhi_lin] Dynamic detect FocalTech and Novatek touch driver 20170222 begin*/
	#if defined(TPD_REPORT_VENDOR_FW)
	{
		extern char *tpd_show_vendor_firmware;
		char buf[80]={0};
		sprintf(buf, "Novatek_0x%02X", nvt_ts_read_fwver());
		if (tpd_show_vendor_firmware == NULL) {
			tpd_show_vendor_firmware = kmalloc(strlen(buf) + 1, GFP_KERNEL);
		}
		if (tpd_show_vendor_firmware != NULL) {
			strcpy(tpd_show_vendor_firmware, buf);
		}
	}
	#endif
/*[Arima_8100][bozhi_lin] 20170222 end*/

	mutex_init(&ts->lock);

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread))
	{
		ret = PTR_ERR(thread);
		NVT_ERR("failed to create kernel thread: %d\n", ret);
		goto err_create_kthread_failed;
	}

	//---check input device---
	if (tpd->dev == NULL) {
		NVT_LOG("input device tpd->dev is NULL, allocate for ts->input_dev\n");
		//---allocate input device---
		ts->input_dev = input_allocate_device();
		if (ts->input_dev == NULL) {
			NVT_ERR("allocate input device failed\n");
			ret = -ENOMEM;
			goto err_input_dev_alloc_failed;
		}

		//---register input device---
		ret = input_register_device(ts->input_dev);
		if (ret) {
			NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
			goto err_input_register_device_failed;
		}
	} else {
		ts->input_dev = tpd->dev;
	}

	ts->abs_x_max = TOUCH_MAX_WIDTH;
	ts->abs_y_max = TOUCH_MAX_HEIGHT;
	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	ts->int_trigger_type = INT_TRIGGER_TYPE;


	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);    //pressure = 255

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
#if MT_PROTOCOL_B
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif	//MT_PROTOCOL_B
#endif	//TOUCH_MAX_FINGER_NUM > 1

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts->max_button_num; retry++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[retry]);
	}
#endif

#if WAKEUP_GESTURE
	for (retry = 0; retry < (sizeof(gesture_key_array) / sizeof(gesture_key_array[0])); retry++) {
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[retry]);
	}
	wake_lock_init(&gestrue_wakelock, WAKE_LOCK_SUSPEND, "poll-wake-lock");
#endif

	sprintf(ts->phys, "input/ts");
#if 0
	ts->input_dev->name = NVT_TS_NAME;
#endif
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;

	//---set int-pin & request irq---
	ret = nvt_irq_registration();


	if (ret != 0) {
		NVT_ERR("request irq failed. ret=%d\n", ret);
		goto err_int_request_failed;
	} else {
		disable_irq(client->irq);
		NVT_LOG("request irq %d succeed\n", client->irq);
	}

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
	if (!nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(4000));
#endif

	mutex_lock(&ts->lock);
	/*[Arima_8100][allen_yu] Fix the problem of power on sequence 20170407 begin*/
	//nvt_hw_reset();
	/*[Arima_8100][allen_yu] 20170407 end*/
	msleep(5);
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);
	mutex_unlock(&ts->lock);

	//---set device node---
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt flash proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_EXT_PROC
	ret = nvt_extra_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

	bTouchIsAwake = 1;
	tpd_load_status = 1;
	NVT_LOG("end\n");

	enable_irq(client->irq);

	return 0;

err_init_NVT_ts:
	free_irq(client->irq, ts);
#if BOOT_UPDATE_FIRMWARE
err_create_nvt_fwu_wq_failed:
#endif
err_int_request_failed:
err_input_register_device_failed:
	if (tpd->dev == NULL)
		input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_create_kthread_failed:
	mutex_destroy(&ts->lock);
err_chipid_failed:
err_check_functionality_failed:
#if MTK_REGULATOR
	nvt_power_off(ts);
err_nvt_power_on:
	nvt_power_remove(ts);
err_power_init:
#endif
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct i2c_client *client)
{
	mutex_destroy(&ts->lock);

	NVT_LOG("Removing driver...\n");

	free_irq(client->irq, ts);
	if (tpd->dev == NULL)
		input_unregister_device(ts->input_dev);
	i2c_set_clientdata(client, NULL);
	kfree(ts);

	return 0;
}

static int nvt_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, NVT_I2C_NAME);
	return 0;
}

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static const struct of_device_id nvt_match_table[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
#endif

static struct i2c_driver nvt_i2c_driver = {
	.probe  = nvt_ts_probe,
	.remove = nvt_ts_remove,
	.detect = nvt_i2c_detect,
	.driver.name = NVT_I2C_NAME,
	.driver = {
		.name  = NVT_I2C_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
	.id_table = nvt_ts_id,
};


static int nvt_local_init(void)
{
	int ret = 0;

	NVT_LOG("start\n");

	ret = i2c_add_driver(&nvt_i2c_driver);
	if (ret) {
		NVT_ERR("unable to add i2c driver.\n");
		return -1;
	}

	if (tpd_load_status == 0) {
		NVT_ERR("add error touch panel driver.\n");
		i2c_del_driver(&nvt_i2c_driver);
		return -1;
	}

	if (tpd_dts_data.use_tpd_button) {
		/*initialize tpd button data*/
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
				tpd_dts_data.tpd_key_dim_local);
	}

	tpd_type_cap = 1;

	NVT_LOG("end\n");

	return 0;
}


/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static void nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};
	/*[Arima_8100][allen_yu] 2nd Modify the setting of suspend sequence 20170418 begin*/
	/*[Arima_8100][allen_yu] 2nd Modify the setting of VDD in Suspend/Resume 20170407 begin*/
	//int32_t ret = 0;
	/*[Arima_8100][allen_yu] 20170222 end*/
	/*[Arima_8100][allen_yu] 20170418 end*/
#if MT_PROTOCOL_B
	uint32_t i = 0;
	/*[Arima_8100][allen_yu] 2nd Modify the setting of suspend sequence 20170418 begin*/
#endif
#if MTK_REGULATOR
	int32_t ret = 0;
	/*[Arima_8100][allen_yu] 20170418 end*/
#endif

	if (!bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		return;
	}

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	bTouchIsAwake = 0;

#if WAKEUP_GESTURE
	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	//---write i2c command to enter "wakeup gesture mode"---
	buf[0] = 0x50;
	buf[1] = 0x13;
	buf[2] = 0xFF;
	buf[3] = 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 4);

	enable_irq_wake(ts->client->irq);

	NVT_LOG("Enabled touch wakeup gesture\n");
#else
	disable_irq(ts->client->irq);

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	//---write i2c command to enter "deep sleep mode"---
	buf[0] = 0x50;
	buf[1] = 0x12;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
/*[Arima_8100][allen_yu] 2nd Modify the setting of suspend sequence 20170418 begin*/
	msleep(20);
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(10);
#if MTK_REGULATOR
	if (ts->power_on) {
		if (!IS_ERR(tpd->reg)) {
			ret = regulator_disable(tpd->reg);
			if (ret) {
				NVT_ERR("Regulator avdd disable failed ret=%d\n", ret);
			} else {
				ts->power_on = false;
			}
		} else {
			NVT_ERR("tpd->reg IS_ERR!\n");
		}
	}
#endif /* #if MTK_REGULATOR */
/*[Arima_8100][allen_yu] 20170418 end*/
#endif

	/* release all touches */
#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#if !MT_PROTOCOL_B
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

	msleep(50);

	mutex_unlock(&ts->lock);
	/*[Arima_8100][allen_yu] 2nd Modify the setting of suspend sequence 20170418 begin*/
	/*[Arima_8100][allen_yu] 2nd Modify the setting of VDD in Suspend/Resume 20170407 begin*/
	//if (!IS_ERR(tpd->reg)) {
		//ret = regulator_disable(tpd->reg);
		//if (ret)
			//NVT_ERR("Regulator avdd disable failed ret=%d\n", ret);
	//}
	/*[Arima_8100][allen_yu] 20170222 end*/
	/*[Arima_8100][allen_yu] 20170418 end*/
	NVT_LOG("end\n");
/*[Arima_8100][allen_yu] 2nd Modify the setting of suspend sequence 20170418 begin*/
	//gtp_suspend = true;
/*[Arima_8100][allen_yu] 20170418 end*/
	return;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static void nvt_ts_resume(struct device *dev)
{
	/*[Arima_8100][allen_yu] 2nd Modify the setting of resume sequence 20170418 begin*/
	/*[Arima_8100][allen_yu] 2nd Modify the setting of VDD in Suspend/Resume 20170407 begin*/
	//int32_t ret = 0;
	
	//if (!IS_ERR(tpd->reg)) {
		//ret = regulator_enable(tpd->reg);
		//if (ret) {
			//NVT_ERR("Regulator avdd enable failed ret=%d\n", ret);
		//}
	//}
	/*[Arima_8100][allen_yu] 20170222 end*/
	/*[Arima_8100][allen_yu] 20170418 end*/
#if WAKEUP_GESTURE
	uint8_t buf[4] = {0};
#endif
/*[Arima_8100][allen_yu] 2nd Modify the setting of resume sequence 20170418 begin*/
#if MTK_REGULATOR
	int32_t ret = 0;
#endif
/*[Arima_8100][allen_yu] 20170418 end*/

	if (bTouchIsAwake) {
		NVT_LOG("Touch is already resume\n");
		return;
	}

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");
/*[Arima_8100][allen_yu] 2nd Modify the setting of resume sequence 20170418 begin*/
	//gtp_suspend = false;
/*[Arima_8100][allen_yu] 20170418 end*/
#if WAKEUP_GESTURE
	if (bWakeupByGesture == 1) {
		bWakeupByGesture = 0;

		//---write i2c command to leave "wakeup gesture mode"---
		buf[0] = 0x50;
		buf[1] = 0x14;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
	} else {
		nvt_hw_reset();
		nvt_bootloader_reset();
		nvt_check_fw_reset_state(RESET_STATE_INIT);
	}
#else
/*[Arima_8100][allen_yu] 2nd Modify the setting of resume sequence 20170418 begin*/
#if MTK_REGULATOR
	if (!ts->power_on) {
		if (!IS_ERR(tpd->reg)) {
			ret = regulator_enable(tpd->reg);
			if (ret) {
				NVT_ERR("Regulator avdd enable failed ret=%d\n", ret);
			} else {
				ts->power_on = true;
			}
		} else {
			NVT_ERR("tpd->reg IS_ERR!\n");
		}
	}
#endif /* #if MTK_REGULATOR */
	msleep(20);
/*[Arima_8100][bozhi_lin] fine tune nt11206 touch resume performance 20170504 begin*/
	nvt_sw_reset_idle_no_i2c_debounce();
/*[Arima_8100][bozhi_lin] 20170504 end*/
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 1);
	msleep(10);
/*[Arima_8100][allen_yu] 20170418 end*/	
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);
	enable_irq(ts->client->irq);
#endif

	bTouchIsAwake = 1;

	mutex_unlock(&ts->lock);

	NVT_LOG("end\n");

	return ;
}

static struct device_attribute *novatek_attrs[] = {
};

static struct tpd_driver_t nvt_device_driver =
{
	.tpd_device_name = NVT_I2C_NAME,
	.tpd_local_init = nvt_local_init,
	.suspend = nvt_ts_suspend,
	.resume = nvt_ts_resume,
	.attrs = {
		.attr = novatek_attrs,
		.num  = ARRAY_SIZE(novatek_attrs),
	},
};

/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	NVT_LOG("start\n");

	tpd_get_dts_info();

	ret = tpd_driver_add(&nvt_device_driver);
	if (ret < 0){
		NVT_ERR("failed to add i2c driver");
		goto err_driver;
	}

	NVT_LOG("finished\n");

err_driver:
	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	tpd_driver_remove(&nvt_device_driver);
}

module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
