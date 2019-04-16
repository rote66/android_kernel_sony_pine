/*!
 * @section LICENSE
 * $license_gpl$
 *
 * @filename $filename$
 * @date     2016/05/09 13:44
 * @id       $id$
 * @version  0.2.0
 *
 * @brief    bma4xy Linux Driver
 */

#define DRIVER_VERSION "0.0.2.1"
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/atomic.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <asm/uaccess.h>
#include <cust_acc.h>
#include <accel.h>
//<--[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 BEGIN --
#include  <linux/dma-mapping.h>
//-->[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 END --

#include "bma4xy_driver.h"
#include "bma4xy.h"

#include <step_counter.h>

#define BMA4XY_DEV_NAME        "bma4xy_acc"
struct i2c_client *bma4xy_i2c_client = NULL;
static const struct i2c_device_id bma4xy_i2c_id[] = { {BMA4XY_DEV_NAME, 0}, {} };
#define COMPATIABLE_NAME "mediatek,bma4xy"
//<--[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 BEGIN --
static char   * I2CDMAWriteBuf = NULL;
static uintptr_t   I2CDMAWriteBuf_pa;  // = NULL;
static char   * I2CDMAReadBuf = NULL;
static uintptr_t  I2CDMAReadBuf_pa;   // = NULL;
#define   MAX_BUFFER_SIZE           255
//-->[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 END --

static int bma4xy_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int bma4xy_i2c_remove(struct i2c_client *client);
static int bma4xy_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int bma4xy_i2c_resume(struct i2c_client *client);
static int gsensor_local_init(void);
static int gsensor_remove(void);
static int gsensor_set_delay(u64 ns);


/*----------------------------------------------------------------------------*/
static struct data_resolution bma4xy_acc_data_resolution[1] = {
	{{0, 12}, 8192},	// +/-4G range
};
/*----------------------------------------------------------------------------*/
static struct data_resolution bma4xy_acc_offset_resolution = {{0, 12}, 8192};

#if 1
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               pr_err(GSE_TAG"%s\n", __func__)
#define GSE_ERR(fmt, args...)    pr_err(GSE_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    pr_err(GSE_TAG fmt, ##args)
#else
#define GSE_TAG
#define GSE_FUN(f)
#define GSE_ERR(fmt, args...)
#define GSE_LOG(fmt, args...)
#endif

static bool enable_status;
struct acc_hw accel_cust_1;
static struct acc_hw *hw = &accel_cust_1;
struct bma4xy_client_data *obj_i2c_data;
static bool sensor_power = true;
static int sensor_suspend;
static struct GSENSOR_VECTOR3D gsensor_gain;
static DEFINE_MUTEX(gsensor_mutex);

static int gsensor_init_flag = -1;	/* 0<==>OK -1 <==> fail */
//<--[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20170116 BEGIN --
static bool first_read_cali = true;
static const char g_sencer_dat[]="/data/prod/g-sencer.dat";
//-->[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20170116 END --


enum BMA4XY_SENSOR_INT_MAP {
	BMA4XY_UC_INTA = 0,
	BMA4XY_UC_INTB = 1,
	BMA4XY_UC_INTC = 2,
	BMA4XY_UC_INTD = 3,
	BMA4XY_UC_INTE = 4,
	BMA4XY_UC_INTF = 5,
	BMA4XY_UC_INTG = 6,
	BMA4XY_UC_INTH = 7,
	BMA4XY_FFULL = 8,
	BMA4XY_FWM = 9,
	BMA4XY_DRDY = 10,
};

enum BMA4XY_CONFIG_FUN {
	BMA4XY_SIG_MOTION = 0,
	BMA4XY_STEP_DETECTOR = 1,
	BMA4XY_STEP_COUNTER = 2,
	BMA4XY_TILT = 3,
	BMA4XY_PICKUP = 4,
	BMA4XY_GLANCE_DETECTOR = 5,
	BMA4XY_WAKEUP = 6,
	BMA4XY_ANY_MOTION = 7,
	BMA4XY_ORIENTATION = 8,
	BMA4XY_FLAT = 9,
	BMA4XY_TAP = 10,
	BMA4XY_HIGH_G = 11,
	BMA4XY_LOW_G = 12,
};

enum BMA4XY_INT_STATUS0 {
	SIG_MOTION_OUT = 0x01,
	STEP_DET_OUT = 0x02,
	TILT_OUT = 0x04,
	PICKUP_OUT = 0x08,
	GLANCE_OUT = 0x10,
	WAKEUP_OUT = 0x20,
	ANY_NO_MOTION_OUT = 0x40,
	ERROR_INT_OUT = 0x80,

};
enum BMA4XY_INT_STATUS1 {
	FIFOFULL_OUT = 0x01,
	FIFOWATERMARK_OUT = 0x02,
	MAG_DRDY_OUT = 0x20,
	ACC_DRDY_OUT = 0x80,
};
//<--[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 BEGIN --
static void bma4xy_i2c_dma_init(struct i2c_client *client)
{
	client->addr &= I2C_MASK_FLAG;
	client->ext_flag |= I2C_DMA_FLAG;
	client->ext_flag |= I2C_DIRECTION_FLAG;

	if( I2CDMAWriteBuf == NULL )
	{
	    #ifdef CONFIG_64BIT    
		I2CDMAWriteBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAWriteBuf_pa, GFP_KERNEL);
        #else
		I2CDMAWriteBuf = (char *)dma_alloc_coherent( NULL, MAX_BUFFER_SIZE, &I2CDMAWriteBuf_pa, GFP_KERNEL );
	    #endif
		if( I2CDMAWriteBuf == NULL )
		{
			GSE_ERR("%s : failed to allocate dma write buffer\n", __func__ );
		}
	}
	
	if( I2CDMAReadBuf == NULL )
	{
	    #ifdef CONFIG_64BIT 	
		I2CDMAReadBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL);
	    #else
		I2CDMAReadBuf = (char *)dma_alloc_coherent( NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL );
	    #endif
		if( I2CDMAReadBuf == NULL )
		{
			GSE_ERR("%s : failed to allocate dma read buffer\n", __func__ );
		}
	}

}
	
/*static void bma4xy_i2c_dma_remove(struct i2c_client *client)
{
	if( I2CDMAWriteBuf )
	{
	    #ifdef CONFIG_64BIT 	
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAWriteBuf, I2CDMAWriteBuf_pa);
	    #else
		dma_free_coherent( NULL, MAX_BUFFER_SIZE, I2CDMAWriteBuf, I2CDMAWriteBuf_pa );
	    #endif
		I2CDMAWriteBuf	  = NULL;
		I2CDMAWriteBuf_pa = 0;
	}
	
	if( I2CDMAReadBuf )
	{
	    #ifdef CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAReadBuf, I2CDMAReadBuf_pa);
	    #else
		dma_free_coherent( NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf, I2CDMAReadBuf_pa );
	    #endif
		I2CDMAReadBuf	  = NULL;
		I2CDMAReadBuf_pa  = 0;
	}

}	*/

//-->[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 END --
static int bma4xy_i2c_read(struct i2c_client *client,
	uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int32_t retry;

	struct i2c_msg msg[] = {
		{
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &reg_addr,
		},

		{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = len,
		.buf = data,
		},
	};
	for (retry = 0; retry < BMA4XY_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			usleep_range(BMA4XY_I2C_WRITE_DELAY_TIME * 1000,
				BMA4XY_I2C_WRITE_DELAY_TIME * 1000);
	}

	if (BMA4XY_MAX_RETRY_I2C_XFER <= retry) {
		GSE_ERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}
//<--[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 BEGIN --
#if 0
static int bma4xy_i2c_write(struct i2c_client *client,
	uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	int32_t retry;

	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = len + 1,
		.buf = NULL,
	};
	msg.buf = kmalloc(len + 1, GFP_KERNEL);
	if (!msg.buf) {
		GSE_ERR("Allocate mem failed\n");
		return -ENOMEM;
	}
	msg.buf[0] = reg_addr;
	memcpy(&msg.buf[1], data, len);
	for (retry = 0; retry < BMA4XY_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, &msg, 1) > 0)
			break;
		else
			usleep_range(BMA4XY_I2C_WRITE_DELAY_TIME * 1000,
				BMA4XY_I2C_WRITE_DELAY_TIME * 1000);
	}
	kfree(msg.buf);
	if (BMA4XY_MAX_RETRY_I2C_XFER <= retry) {
		GSE_ERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}
#endif
static s8 bma4xy_i2c_read_wrapper(uint8_t dev_addr,
	uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int ret;
	s32 retry = 0;
	u8 buffer[2];
	//int i;
	struct i2c_msg msg[2] = {
		{
			.addr = (bma4xy_i2c_client->addr & I2C_MASK_FLAG), 
			.flags = 0, 
			.buf = buffer, 
			.len = 2, 
			/* .timing = I2C_MASTER_CLOCK */ 
		}, 
		{
			.addr = (bma4xy_i2c_client->addr & I2C_MASK_FLAG), 
			.ext_flag = (bma4xy_i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG), 
			.flags = I2C_M_RD, 
			.buf = (u8 *) I2CDMAReadBuf_pa, 
			.len = len, 
			/* .timing = I2C_MASTER_CLOCK */ 
		}, 
	}; 

	//GSE_FUN();
	if (len < 8) {
		int err;
		bma4xy_i2c_client->addr = ((bma4xy_i2c_client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG));
		err = bma4xy_i2c_read(bma4xy_i2c_client, reg_addr, data, len);
		return err;
	}
	
	buffer[0] = reg_addr & 0xFF;

	if (data == NULL){
		printk("%s:data == NULL\n", __func__);
		return -1;
	}

	/* GTP_DEBUG("dma i2c read: 0x%04X, %d bytes(s)", addr, len); */
	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(bma4xy_i2c_client->adapter, &msg[0], 2);
		if (ret < 0)
			continue;
		memcpy(data, I2CDMAReadBuf, len);
#if 0
		printk("I2CDMAReadBuf:");
		for( i = 0; i < len; i++ )
		{
			printk("%02X ", *(data + i));
		}
		printk(" \n");
#endif
		return 0;
	}
	GSE_ERR("Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", reg_addr, len, ret);
	return ret;
}

static s8 bma4xy_i2c_write_wrapper(uint8_t dev_addr,
	uint8_t addr, uint8_t *buffer, uint8_t len)
{
	s32 ret = 0;
	s32 pos = 0;
	s32 transfer_length;
	u16 address = addr;
	//int i;

	struct i2c_msg msg = {
		.flags = !I2C_M_RD,
		.ext_flag = (bma4xy_i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.addr = (bma4xy_i2c_client->addr & I2C_MASK_FLAG),
		//.timing = I2C_MASTER_CLOCK,
		.buf = (u8 *)(uintptr_t)I2CDMAWriteBuf_pa,
	};

	while (pos != len) {
		if (len - pos > (250 - 1))
			transfer_length = 250 - 1;
		else
			transfer_length = len - pos;

		I2CDMAWriteBuf[0] = address & 0xFF;
		memcpy(&I2CDMAWriteBuf[1], &buffer[pos], transfer_length);
#if 0
		printk("I2CDMAWriteBuf:");
		for( i = 0; i < (len+1); i++ )
		{
			printk("%02X ", *(I2CDMAWriteBuf + i));
		}
		printk(" \n");
#endif
		msg.len = transfer_length + 1;
		if (1) {/*workround log too much*/
			ret = i2c_transfer(bma4xy_i2c_client->adapter, &msg, 1);
			if (ret != 1) {
				GSE_ERR("I2c Transfer error! (%d)", ret);
				ret = -1;
				break;
			}
		} else {
			ret = -1;
			break;
		}
		ret = 0;
		pos += transfer_length;
		address += transfer_length;
	}
	return ret;

}
#if 0
static s8 bma4xy_i2c_read_wrapper(uint8_t dev_addr,
	uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int err;
	err = bma4xy_i2c_read(bma4xy_i2c_client, reg_addr, data, len);
	return err;
}

static s8 bma4xy_i2c_write_wrapper(uint8_t dev_addr,
	uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	int err;
	err = bma4xy_i2c_write(bma4xy_i2c_client, reg_addr, data, len);
	return err;
}
#endif
//-->[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 END --

static void BMA4XY_power(struct acc_hw *hw, unsigned int on)
{
}

static int BMA4XY_SetDataResolution(struct bma4xy_client_data *obj)
{

	obj->reso = &bma4xy_acc_data_resolution[0];
	return 0;
}
static int BMA4XY_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8) * 10);

	if ((NULL == buf) || (bufsize <= 30))
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "BMA4XY Chip");
	return 0;
}

static int BMA4XY_ReadData(struct i2c_client *client, s16 data[BMA4XY_ACC_AXIS_NUM])
{
	int err = 0;
	struct bma4xy_accel_t raw_data;

	if(NULL == client)
		return -EINVAL;
	err = BMA4XY_CALL_API(read_accel_xyz)(&raw_data);
	if (err < 0)
		return err;
	data[BMA4XY_ACC_AXIS_X] = raw_data.x;
	data[BMA4XY_ACC_AXIS_Y] = raw_data.y;
	data[BMA4XY_ACC_AXIS_Z] = raw_data.z;
	return err;
}

static int BMA4XY_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct bma4xy_client_data *obj = (struct bma4xy_client_data *)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[BMA4XY_ACC_AXIS_NUM];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	if (NULL == buf)
		return -1;
	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	if (sensor_suspend == 1)
		return 0;

	res = BMA4XY_ReadData(client, obj->data);
	if (res != 0) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}

	obj->data[BMA4XY_ACC_AXIS_X] += obj->cali_sw[BMA4XY_ACC_AXIS_X];
	obj->data[BMA4XY_ACC_AXIS_Y] += obj->cali_sw[BMA4XY_ACC_AXIS_Y];
	obj->data[BMA4XY_ACC_AXIS_Z] += obj->cali_sw[BMA4XY_ACC_AXIS_Z];

	acc[obj->cvt.map[BMA4XY_ACC_AXIS_X]] =
	obj->cvt.sign[BMA4XY_ACC_AXIS_X] * obj->data[BMA4XY_ACC_AXIS_X];
	acc[obj->cvt.map[BMA4XY_ACC_AXIS_Y]] =
	obj->cvt.sign[BMA4XY_ACC_AXIS_Y] * obj->data[BMA4XY_ACC_AXIS_Y];
	acc[obj->cvt.map[BMA4XY_ACC_AXIS_Z]] =
	obj->cvt.sign[BMA4XY_ACC_AXIS_Z] * obj->data[BMA4XY_ACC_AXIS_Z];

	acc[BMA4XY_ACC_AXIS_X] =
	acc[BMA4XY_ACC_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[BMA4XY_ACC_AXIS_Y] =
	acc[BMA4XY_ACC_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[BMA4XY_ACC_AXIS_Z] =
	acc[BMA4XY_ACC_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;

	sprintf(buf, "%04x %04x %04x", acc[BMA4XY_ACC_AXIS_X], acc[BMA4XY_ACC_AXIS_Y],
		acc[BMA4XY_ACC_AXIS_Z]);
	if (atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		GSE_LOG("gsensor data: %s!\n", buf);
	return 0;
}

static int BMA4XY_ReadRawData(struct i2c_client *client, char *buf)
{
	int res = 0;
	struct bma4xy_client_data *obj = (struct bma4xy_client_data *)i2c_get_clientdata(client);

	if (!buf || !client)
		return -EINVAL;
	res = BMA4XY_ReadData(client, obj->data);
	if (0 != res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -EIO;
	}
	sprintf(buf, "BMA4XY_ReadRawData %04x %04x %04x", obj->data[BMA4XY_ACC_AXIS_X],
		obj->data[BMA4XY_ACC_AXIS_Y], obj->data[BMA4XY_ACC_AXIS_Z]);
	return 0;
}

static int BMA4XY_ReadOffset(struct i2c_client *client, s8 ofs[BMA4XY_ACC_AXIS_NUM])
{
	int err=0;
#ifdef SW_CALIBRATION
	ofs[0]=ofs[1]=ofs[2]=0x0;
#else
	err = bma4xy_i2c_read(client, BMA4XY_OFFSET_0_ADDR, ofs, BMI160_ACC_AXES_NUM);
	if(err) {
		GSE_ERR("error: %d\n", err);
	}
#endif
	GSE_LOG("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]);

	return err;
}

static int BMA4XY_ResetCalibration(struct i2c_client *client)
{
	struct bma4xy_client_data *priv = i2c_get_clientdata(client);
	int err=0;

	#ifdef SW_CALIBRATION

	#else
	u8 ofs[3]={0,0,0};
	err = bma4xy_i2c_write(client, BMA4XY_OFFSET_0_ADDR, ofs, 3);
	if(err) {
		GSE_ERR("error: %d\n", err);
	}
	#endif

	memset(priv->cali_sw, 0x00, sizeof(priv->cali_sw));
	memset(priv->offset, 0x00, sizeof(priv->offset));
	return err;
}

static int BMA4XY_ReadCalibration(struct i2c_client *client, int dat[BMA4XY_ACC_AXIS_NUM])
{
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int mul;
	GSE_FUN();
	#ifdef SW_CALIBRATION
	mul = 0;//only SW Calibration, disable HW Calibration
	#else
	err = BMA4XY_ReadOffset(client, obj->offset);
	if(err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = priv->reso->sensitivity/bma4xy_acc_offset_resolution.sensitivity;
	#endif

	dat[obj->cvt.map[BMA4XY_ACC_AXIS_X]] = obj->cvt.sign[BMA4XY_ACC_AXIS_X]*(obj->offset[BMA4XY_ACC_AXIS_X]*mul + obj->cali_sw[BMA4XY_ACC_AXIS_X]);
	dat[obj->cvt.map[BMA4XY_ACC_AXIS_Y]] = obj->cvt.sign[BMA4XY_ACC_AXIS_Y]*(obj->offset[BMA4XY_ACC_AXIS_Y]*mul + obj->cali_sw[BMA4XY_ACC_AXIS_Y]);
	dat[obj->cvt.map[BMA4XY_ACC_AXIS_Z]] = obj->cvt.sign[BMA4XY_ACC_AXIS_Z]*(obj->offset[BMA4XY_ACC_AXIS_Z]*mul + obj->cali_sw[BMA4XY_ACC_AXIS_Z]);

	return err;
}

static int BMA4XY_ReadCalibrationEx(struct i2c_client *client, int act[BMA4XY_ACC_AXIS_NUM],
				int raw[BMA4XY_ACC_AXIS_NUM])
{
	/*raw: the raw calibration data; act: the actual calibration data */
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;
	err = 0;
	#ifdef SW_CALIBRATION
	mul = 0;/* only SW Calibration, disable HW Calibration */
	#else
	err = BMA4XY_ReadOffset(client, obj->offset);
	if (err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = obj->reso->sensitivity / bma4xy_acc_offset_resolution.sensitivity;
	#endif

	raw[BMA4XY_ACC_AXIS_X] = obj->offset[BMA4XY_ACC_AXIS_X] * mul + obj->cali_sw[BMA4XY_ACC_AXIS_X];
	raw[BMA4XY_ACC_AXIS_Y] = obj->offset[BMA4XY_ACC_AXIS_Y] * mul + obj->cali_sw[BMA4XY_ACC_AXIS_Y];
	raw[BMA4XY_ACC_AXIS_Z] = obj->offset[BMA4XY_ACC_AXIS_Z] * mul + obj->cali_sw[BMA4XY_ACC_AXIS_Z];
	act[obj->cvt.map[BMA4XY_ACC_AXIS_X]] = obj->cvt.sign[BMA4XY_ACC_AXIS_X] * raw[BMA4XY_ACC_AXIS_X];
	act[obj->cvt.map[BMA4XY_ACC_AXIS_Y]] = obj->cvt.sign[BMA4XY_ACC_AXIS_Y] * raw[BMA4XY_ACC_AXIS_Y];
	act[obj->cvt.map[BMA4XY_ACC_AXIS_Z]] = obj->cvt.sign[BMA4XY_ACC_AXIS_Z] * raw[BMA4XY_ACC_AXIS_Z];
	return 0;
}

static int BMA4XY_WriteCalibration(struct i2c_client *client, int dat[BMA4XY_ACC_AXIS_NUM])
{
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[BMA4XY_ACC_AXIS_NUM], raw[BMA4XY_ACC_AXIS_NUM];

	err = BMA4XY_ReadCalibrationEx(client, cali, raw);
	if (0 != err) {/*offset will be updated in obj->offset */
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
	raw[BMA4XY_ACC_AXIS_X], raw[BMA4XY_ACC_AXIS_Y], raw[BMA4XY_ACC_AXIS_Z],
	obj->offset[BMA4XY_ACC_AXIS_X], obj->offset[BMA4XY_ACC_AXIS_Y], obj->offset[BMA4XY_ACC_AXIS_Z],
	obj->cali_sw[BMA4XY_ACC_AXIS_X], obj->cali_sw[BMA4XY_ACC_AXIS_Y], obj->cali_sw[BMA4XY_ACC_AXIS_Z]);
	
	/*calculate the real offset expected by caller */
	cali[BMA4XY_ACC_AXIS_X] += dat[BMA4XY_ACC_AXIS_X];
	cali[BMA4XY_ACC_AXIS_Y] += dat[BMA4XY_ACC_AXIS_Y];
	cali[BMA4XY_ACC_AXIS_Z] += dat[BMA4XY_ACC_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n",
		dat[BMA4XY_ACC_AXIS_X], dat[BMA4XY_ACC_AXIS_Y], dat[BMA4XY_ACC_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[BMA4XY_ACC_AXIS_X] =
	obj->cvt.sign[BMA4XY_ACC_AXIS_X] * (cali[obj->cvt.map[BMA4XY_ACC_AXIS_X]]);
	obj->cali_sw[BMA4XY_ACC_AXIS_Y] =
	obj->cvt.sign[BMA4XY_ACC_AXIS_Y] * (cali[obj->cvt.map[BMA4XY_ACC_AXIS_Y]]);
	obj->cali_sw[BMA4XY_ACC_AXIS_Z] =
	obj->cvt.sign[BMA4XY_ACC_AXIS_Z] * (cali[obj->cvt.map[BMA4XY_ACC_AXIS_Z]]);
#else
	int divisor = obj->reso->sensitivity / lsb;/* modified */

	obj->offset[BMA4XY_ACC_AXIS_X] =
	(s8) (obj->cvt.sign[BMA4XY_ACC_AXIS_X] * (cali[obj->cvt.map[BMA4XY_ACC_AXIS_X]]) / (divisor));
	obj->offset[BMA4XY_ACC_AXIS_Y] =
	(s8) (obj->cvt.sign[BMA4XY_ACC_AXIS_Y] * (cali[obj->cvt.map[BMA4XY_ACC_AXIS_Y]]) / (divisor));
	obj->offset[BMA4XY_ACC_AXIS_Z] =
	(s8) (obj->cvt.sign[BMA4XY_ACC_AXIS_Z] * (cali[obj->cvt.map[BMA4XY_ACC_AXIS_Z]]) / (divisor));

	/*convert software calibration using standard calibration */
	obj->cali_sw[BMA4XY_ACC_AXIS_X] =
	obj->cvt.sign[BMA4XY_ACC_AXIS_X] * (cali[obj->cvt.map[BMA4XY_ACC_AXIS_X]]) % (divisor);
	obj->cali_sw[BMA4XY_ACC_AXIS_Y] =
	obj->cvt.sign[BMA4XY_ACC_AXIS_Y] * (cali[obj->cvt.map[BMA4XY_ACC_AXIS_Y]]) % (divisor);
	obj->cali_sw[BMA4XY_ACC_AXIS_Z] =
	obj->cvt.sign[BMA4XY_ACC_AXIS_Z] * (cali[obj->cvt.map[BMA4XY_ACC_AXIS_Z]]) % (divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
		obj->offset[BMA4XY_ACC_AXIS_X] * divisor + obj->cali_sw[BMA4XY_ACC_AXIS_X],
		obj->offset[BMA4XY_ACC_AXIS_Y] * divisor + obj->cali_sw[BMA4XY_ACC_AXIS_Y],
		obj->offset[BMA4XY_ACC_AXIS_Z] * divisor + obj->cali_sw[BMA4XY_ACC_AXIS_Z],
		obj->offset[BMA4XY_ACC_AXIS_X], obj->offset[BMA4XY_ACC_AXIS_Y], obj->offset[BMA4XY_ACC_AXIS_Z],
		obj->cali_sw[BMA4XY_ACC_AXIS_X], obj->cali_sw[BMA4XY_ACC_AXIS_Y], obj->cali_sw[BMA4XY_ACC_AXIS_Z]);

	err = hwmsen_write_block(obj->client, BMA4XY_OFFSET_0_ADDR, obj->offset, BMA4XY_ACC_AXIS_NUM);
	if (err) {
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif
	mdelay(1);
	return err;
}

static int BMA4XY_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[2] = { 0 };
	int ret = 0;

	client->addr = ((client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG));
	ret = bma4xy_i2c_read(client, BMA4XY_CHIP_ID_ADDR, databuf, 1);
	if (ret < 0)
		goto exit_BMA4XY_CheckDeviceID;

	GSE_LOG("BMA4XY_CheckDeviceID %d done!\n ", databuf[0]);
	mdelay(1);
	return 0;
exit_BMA4XY_CheckDeviceID:
	if (ret < 0) {
		GSE_ERR("BMA4XY_CheckDeviceID %d failt!\n ", databuf[0]);
		return ret;
	}
	return ret;
}

static int BMA4XY_SetPowerMode(struct i2c_client *client, bool enable)
{
	int ret = 0;
	struct bma4xy_client_data *client_data = (struct bma4xy_client_data *)i2c_get_clientdata(client);

	if ((enable == 0) && (client_data->sigmotion_enable == 0)&&(client_data->stepdet_enable == 0)&&(
		client_data->stepcounter_enable == 0)) {
		ret = BMA4XY_CALL_API(set_accel_enable)(0);
		GSE_LOG("acc_op_mode %d", enable);
	}
	else if (enable == 1) {
		ret = BMA4XY_CALL_API(set_accel_enable)(1);
		GSE_LOG("acc_op_mode %d", enable);
	}
	if (ret < 0) {
		GSE_LOG("set power mode failed!\n");
		return ret;
	}
	sensor_power = enable;
	client_data->acc_pm = enable;
	mdelay(1);
	GSE_LOG("leave Sensor power status is sensor_power = %d\n",sensor_power);
	return ret;
}

static int BMA4XY_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	int err = 0;
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);

	err = BMA4XY_CALL_API(set_accel_range)(dataformat);
	if (err) {
		GSE_ERR("faliled");
		return -EIO;
	}
	return BMA4XY_SetDataResolution(obj);
}

static int BMA4XY_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	uint8_t data = 0;
	int ret = 0;
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);

	data = (uint8_t)bwrate;
	if (bwrate == 4)
		data = 0x74;
	else
		data |= 0xA0;
	ret = bma4xy_i2c_write_wrapper(obj->device.dev_addr, 0x40, &data, 1);
	if (ret) {
		GSE_ERR("faliled");
		return ret;
	}
	GSE_LOG("acc_odr =%d", data);
	return ret;
}

static void bma4xy_i2c_delay(u32 msec)
{
	if (msec <= 20)
		usleep_range(msec * 1000, msec * 1000);
	else
		msleep(msec);
}

uint64_t bma4xy_get_alarm_timestamp(void)
{
	uint64_t ts_ap;
	struct timespec tmp_time;
	get_monotonic_boottime(&tmp_time);
	ts_ap = (uint64_t)tmp_time.tv_sec * 1000000000 + tmp_time.tv_nsec;
	return ts_ap;
}

static int bma4xy_check_chip_id(struct bma4xy_client_data *client_data)
{
	int err = 0;
	uint8_t chip_id = 0;

	err = client_data->device.bus_read(client_data->device.dev_addr,
			BMA4XY_CHIP_ID_ADDR, &chip_id, 1);
	if (err) {
		GSE_ERR("error");
		return err;
	}
	GSE_LOG("read chip id result: %#x", chip_id);
	return err;
}

static ssize_t bma4xy_show_chip_id(struct device_driver *ddri, char *buf)
{
	uint8_t chip_id = 0;
	uint8_t rev_id = 0;
	int err = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = bma4xy_i2c_read_wrapper(client_data->device.dev_addr,
			BMA4XY_CHIP_ID_ADDR, &chip_id, 1);
	err += bma4xy_i2c_read_wrapper(client_data->device.dev_addr,
			BMA4XY_REV_ID_ADDR, &rev_id, 1);
	if (err) {
		GSE_ERR("falied");
		return err;
	}
	return snprintf(buf, 48, "chip_id=%x rev_id=%x\n", chip_id, rev_id);
}

static ssize_t bma4xy_show_acc_op_mode(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char acc_op_mode;

	err = BMA4XY_CALL_API(get_accel_enable)(&acc_op_mode);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 96, "1 mean enable now is %d\n", acc_op_mode);
}
static ssize_t bma4xy_store_acc_op_mode(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long op_mode;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);


	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;
	if (op_mode == 2 &&
		(client_data->sigmotion_enable == 0) &&
		(client_data->stepdet_enable == 0) &&
		(client_data->stepcounter_enable == 0) &&
		(client_data->tilt_enable == 0) &&
		(client_data->pickup_enable == 0) &&
		(client_data->glance_enable == 0) &&
		(client_data->wakeup_enable == 0)) {
			err = BMA4XY_CALL_API(set_accel_enable)(0);
			GSE_LOG("acc_op_mode %ld", op_mode);
		}
	else if (op_mode == 0) {
		err = BMA4XY_CALL_API(set_accel_enable)(1);
		GSE_LOG("acc_op_mode %ld", op_mode);
	}
	if (err) {
		GSE_ERR("failed");
		return err;
	} else {
		client_data->acc_pm = op_mode;
		return count;
	}
}

static ssize_t bma4xy_show_acc_value(struct device_driver *ddri, char *buf)
{
	struct bma4xy_accel_t data;
	int err;
	err = BMA4XY_CALL_API(read_accel_xyz)(&data);
	if (err < 0)
		return err;
	return snprintf(buf, 48, "%hd %hd %hd\n",
			data.x, data.y, data.z);
}

static ssize_t bma4xy_show_acc_range(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char acc_range;
	err = BMA4XY_CALL_API(get_accel_range)(&acc_range);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 16, "%d\n", acc_range);
}

static ssize_t bma4xy_store_acc_range(struct device_driver *ddri, const char *buf, size_t count)
{
	int err;
	unsigned long acc_range;
	uint8_t data = 0;
	err = kstrtoul(buf, 10, &acc_range);
	if (err)
		return err;
	data = (uint8_t)(acc_range);
	err = BMA4XY_CALL_API(set_accel_range)(data);
	if (err) {
		GSE_ERR("faliled");
		return -EIO;
	}
	return count;
}

static ssize_t bma4xy_show_acc_odr(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char acc_odr;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = BMA4XY_CALL_API(get_accel_output_data_rate)(&acc_odr);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	client_data->acc_odr = acc_odr;
	return snprintf(buf, 16, "%d\n", acc_odr);
}

static ssize_t bma4xy_store_acc_odr(struct device_driver *ddri, const char *buf, size_t count)
{
	int err;
	unsigned long acc_odr;
	uint8_t data = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = kstrtoul(buf, 10, &acc_odr);
	if (err)
		return err;

	data = (uint8_t)acc_odr;
	if (acc_odr == 4)
		data = 0x74;
	else
		data |= 0xA0;
	err = client_data->device.bus_write(client_data->device.dev_addr,
		0x40, &data, 1);
	if (err) {
		GSE_ERR("faliled");
		return -EIO;
	}
	GSE_ERR("acc_odr =%ld", acc_odr);
	return count;
}

static ssize_t bma4xy_show_selftest(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);


	return snprintf(buf, 16, "%d\n", client_data->selftest);
}
static ssize_t bma4xy_store_selftest(struct device_driver *ddri, const char *buf, size_t count)
{
	int err;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = BMA4XY_CALL_API(perform_accel_selftest)(
		&client_data->selftest);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}

static ssize_t bma4xy_show_foc(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	if (client_data == NULL) {
		GSE_ERR("Invalid client_data pointer");
		return -ENODEV;
	}
	return snprintf(buf, 64,
		"Use echo g_sign aixs > foc to begin foc\n");
}
static ssize_t bma4xy_store_foc(struct device_driver *ddri, const char *buf, size_t count)
{
	int g_value[3] = {0};
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	ssize_t ret;
	int err;

	if (client_data == NULL) {
		GSE_ERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = sscanf(buf, "%11d %11d %11d",
		&g_value[0], &g_value[1], &g_value[2]);
	GSE_LOG("g_value0=%d, g_value1=%d, g_value2=%d",
		g_value[0], g_value[1], g_value[2]);
	if (ret != 3) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	err = BMA4XY_CALL_API(configure_accel_foc)(g_value);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	GSE_LOG("FOC successsfully");
	return count;
}
static ssize_t bma4xy_show_config_function(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	if (client_data == NULL) {
		GSE_ERR("Invalid client_data pointer");
		return -ENODEV;
	}
	return snprintf(buf, PAGE_SIZE,
		"sig_motion=%d step_detector=%d step_counter=%d\n"
		"tilt=%d pickup=%d glance_detector=%d wakeup=%d\n"
		"any_motion=%d nomotion=%d\n"
		"orientation=%d flat=%d\n"
		"high_g=%d low_g=%d\n",
		client_data->sigmotion_enable, client_data->stepdet_enable,
		client_data->stepcounter_enable, client_data->tilt_enable,
		client_data->pickup_enable, client_data->glance_enable,
		client_data->wakeup_enable, client_data->anymotion_enable,
		client_data->nomotion_enable, client_data->orientation_enable,
		client_data->flat_enable, client_data->highg_enable,
		client_data->lowg_enable);
}

static ssize_t bma4xy_store_config_function(struct device_driver *ddri, const char *buf, size_t count)
{
	ssize_t ret;
	int config_func = 0;
	int enable = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	if (client_data == NULL) {
		GSE_ERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = sscanf(buf, "%11d %11d", &config_func, &enable);
	GSE_LOG("config_func = %d, enable=%d", config_func, enable);
	if (ret != 2) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	if (config_func < 0 || config_func > 14)
		return -EINVAL;
	switch (config_func) {
	#if defined(BMA422) || defined(BMA455)
	case BMA4XY_SIG_MOTION:
		if (BMA4XY_CALL_API(significant_motion_enable)(enable) < 0) {
			GSE_ERR("set BMA4XY_SIG_MOTION error");
			return -EINVAL;
		}
		client_data->sigmotion_enable = enable;
		return count;
	#endif
	#if defined(BMA422) || defined(BMA455) || defined(BMA421)
	case BMA4XY_STEP_DETECTOR:
		if (BMA4XY_CALL_API(step_detector_enable)(enable) < 0) {
			GSE_ERR("set BMA4XY_STEP_COUNTER error");
			return -EINVAL;
		}
		client_data->stepdet_enable = enable;
		return count;
	#endif
	#if defined(BMA422) || defined(BMA455) || defined(BMA421)
	case BMA4XY_STEP_COUNTER:
		if (BMA4XY_CALL_API(step_counter_enable)(enable) < 0) {
			GSE_ERR("set BMA4XY_STEP_COUNTER error");
			return -EINVAL;
		}
		client_data->stepcounter_enable = enable;
		return count;
	#endif
	#if defined(BMA422) || defined(BMA455)
	case BMA4XY_TILT:
		if (BMA4XY_CALL_API(tilt_enable)(enable) < 0) {
			GSE_ERR("set BMA4XY_TILT error");
			return -EINVAL;
		}
		client_data->tilt_enable = enable;
		return count;
	#endif
	#if defined(BMA422) || defined(BMA455)
	case BMA4XY_PICKUP:
		if (BMA4XY_CALL_API(pickup_enable)(enable) < 0) {
			GSE_ERR("set BMA4XY_PICKUP error");
			return -EINVAL;
		}
		client_data->pickup_enable = enable;
		return count;
	#endif
	#if defined(BMA422) || defined(BMA455)
	case BMA4XY_GLANCE_DETECTOR:
		if (BMA4XY_CALL_API(glance_enable)(enable) < 0) {
			GSE_ERR("set BMA4XY_GLANCE_DETECTOR error");
			return -EINVAL;
		}
		client_data->glance_enable = enable;
		return count;
	#endif
	#if defined(BMA422) || defined(BMA455)
	case BMA4XY_WAKEUP:
		if (BMA4XY_CALL_API(wakeup_enable)(
			enable) < 0) {
			GSE_ERR("set BMA4XY_WAKEUP error");
			return -EINVAL;
		}
		client_data->wakeup_enable = enable;
		return count;
	#endif
	#if defined(BMA420) || defined(BMA456)
	case BMA4XY_ANY_MOTION:
		if (BMA4XY_CALL_API(anymotion_enable_axis)(enable) < 0) {
			GSE_ERR("set BMA4XY_ANY_MOTION error");
			return -EINVAL;
		}
		client_data->anymotion_enable = enable;
		return count;
	#endif
	#if defined(BMA420) || defined(BMA456)
	case BMA4XY_ORIENTATION:
		if (BMA4XY_CALL_API(orientation_enable)(enable) < 0) {
			GSE_ERR("set BMA4XY_ORIENTATION error");
			return -EINVAL;
		}
		client_data->orientation_enable = enable;
		return count;
	#endif
	#if defined(BMA420) || defined(BMA456)
	case BMA4XY_FLAT:
		if (BMA4XY_CALL_API(flat_enable)(enable) < 0) {
			GSE_ERR("set BMA4XY_FLAT error");
			return -EINVAL;
		}
		client_data->flat_enable = enable;
		return count;
	#endif
	#if defined(BMA420) || defined(BMA456)
	case BMA4XY_TAP:
		if (BMA4XY_CALL_API(tap_enable)(
			enable, client_data->tap_type) < 0) {
			GSE_ERR("set BMA4XY_TAP error");
			return -EINVAL;
		}
		client_data->tap_enable = enable;
		return count;
	#endif
	#if defined(BMA420) || defined(BMA456)
	case BMA4XY_HIGH_G:
		if (BMA4XY_CALL_API(high_g_enable_axis)(enable) < 0) {
			GSE_ERR("set BMA4XY_HIGH_G error");
			return -EINVAL;
		}
		client_data->highg_enable = enable;
		return count;
	#endif
	#if defined(BMA420) || defined(BMA456)
	case BMA4XY_LOW_G:
		if (BMA4XY_CALL_API(low_g_enable)(enable) < 0) {
			GSE_ERR("set BMA4XY_LOW_G error");
			return -EINVAL;
		}
		client_data->lowg_enable = enable;
		return count;
	#endif
	default:
		GSE_ERR("Invalid sensor handle: %d", config_func);
		return -EINVAL;
	}
}
static ssize_t bma4xy_store_axis_remapping(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	struct bma4xy_axes_remap axis_remap_data;

	axis_remap_data.map_x_axis = (uint8_t)buf[0];
	axis_remap_data.map_x_axis_sign = (uint8_t)buf[1];
	axis_remap_data.map_y_axis = (uint8_t)buf[2];
	axis_remap_data.map_y_axis_sign = (uint8_t)buf[3];
	axis_remap_data.map_z_axis = (uint8_t)buf[4];
	axis_remap_data.map_z_axis_sign = (uint8_t)buf[5];
	GSE_LOG("map_x_axis = %d map_x_axis_sign=%d",
	axis_remap_data.map_x_axis, axis_remap_data.map_x_axis_sign);
	GSE_LOG("map_y_axis = %d map_y_axis_sign=%d",
	axis_remap_data.map_y_axis, axis_remap_data.map_y_axis_sign);
	GSE_LOG("map_z_axis = %d map_z_axis_sign=%d",
	axis_remap_data.map_z_axis, axis_remap_data.map_z_axis_sign);
	err = BMA4XY_CALL_API(remap_axes)(&axis_remap_data);
	if (err) {
		GSE_ERR("write failed");
		return -EIO;
	}
	return count;
}
static ssize_t bma4xy_show_fifo_length(struct device_driver *ddri, char *buf)
{
	int err;
	uint16_t fifo_bytecount = 0;

	err = BMA4XY_CALL_API(fifo_length)(&fifo_bytecount);
	if (err) {
		GSE_ERR("read falied");
		return err;
	}
	return snprintf(buf, 32, "%d\n", fifo_bytecount);
}
static ssize_t bma4xy_store_fifo_flush(struct device_driver *ddri, const char *buf, size_t count)
{
	int err;
	unsigned long enable;
	err = kstrtoul(buf, 10, &enable);
	if (err)
		return err;
	if (enable)
		err = BMA4XY_CALL_API(set_command_register)(0xb0);
	if (err) {
		GSE_ERR("write failed");
		return -EIO;
	}
	return count;
}
static ssize_t bma4xy_show_fifo_acc_enable(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char fifo_acc_enable;

	err = BMA4XY_CALL_API(get_fifo_accel_enable)(&fifo_acc_enable);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 16, "%d\n", fifo_acc_enable);
}
static ssize_t bma4xy_store_fifo_acc_enable(struct device_driver *ddri, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	unsigned char fifo_acc_enable;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	fifo_acc_enable = (unsigned char)data;
	err = BMA4XY_CALL_API(set_fifo_accel_enable)(fifo_acc_enable);
	if (err) {
		GSE_ERR("faliled");
		return -EIO;
	}
	client_data->fifo_acc_enable = fifo_acc_enable;
	return count;
}
static ssize_t bma4xy_show_int_enable(struct device_driver *ddri, char *buf)
{
	int err = 0;
	unsigned char int_enable1, int_enable2;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	err = bma4xy_i2c_read_wrapper(client_data->device.dev_addr,
		BMA4XY_INTR1_MAP, &int_enable1, 1);
	err += bma4xy_i2c_read_wrapper(client_data->device.dev_addr,
		BMA4XY_INTR2_MAP, &int_enable2, 1);
	if (err) {
		GSE_ERR("read error");
		return err;
	}
	return snprintf(buf, 96, "channel1=%x channel1=%x\n",
		int_enable1, int_enable2);
}
static ssize_t bma4xy_store_int_enable(struct device_driver *ddri, const char *buf, size_t count)
{
	int map_int, value;
	int err;
	ssize_t ret;

	ret = sscanf(buf, "%3d %3d", &map_int, &value);
	if (ret != 2) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	err = BMA4XY_CALL_API(set_output_enable)(map_int, value);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}

static ssize_t bma4xy_show_int_mapping(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char int1_config;
	unsigned char int2_config;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = bma4xy_i2c_read_wrapper(client_data->device.dev_addr,
		BMA4XY_INTR_MAP_1_ADDR, &int1_config, 1);
	if (err) {
		GSE_ERR("read error");
		return err;
	}
	err += bma4xy_i2c_read_wrapper(client_data->device.dev_addr,
		BMA4XY_INTR_MAP_2_ADDR, &int2_config, 1);
	if (err) {
		GSE_ERR("read error");
		return err;
	}
	return snprintf(buf, 32, "0x%x 0x%x\n", int1_config, int2_config);
}

static ssize_t bma4xy_store_int_mapping(struct device_driver *ddri, const char *buf, size_t count)
{
	int interrupt_type, map_int, value;
	ssize_t ret;

	ret = sscanf(buf, "%3d %3d %3d", &interrupt_type, &map_int, &value);
	if (ret != 3) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	if (interrupt_type < 0 || interrupt_type > 10)
		return -EINVAL;
	switch (interrupt_type) {
	case BMA4XY_FFULL:
		if (BMA4XY_CALL_API(set_intr_fifo_full)(map_int, value) < 0)
			return -EINVAL;
		return count;
	case BMA4XY_FWM:
		if (BMA4XY_CALL_API(set_intr_fifo_wm)(map_int, value) < 0)
			return -EINVAL;
		return count;
	case BMA4XY_DRDY:
		if (BMA4XY_CALL_API(set_intr_data_rdy)(map_int, value) < 0)
			return -EINVAL;
		return count;
	default:
		return count;
	}
}

static ssize_t bma4xy_show_load_config_stream(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	return snprintf(buf, 48, "config stream %s\n",
		client_data->config_stream_name);
}

int bma4xy_init_after_config_stream_load(
	struct bma4xy_client_data *client_data)
{
	int err = 0;
	uint8_t int_enable = 0x0a;
	uint8_t latch_enable = 0x01;
	uint8_t int1_map = 0xff;
	struct bma4xy_axes_remap axis_remap_data;

	err = bma4xy_write_reg(BMA4XY_INTR_MAP_1_ADDR, &int1_map, 1);
	bma4xy_i2c_delay(10);
	err += bma4xy_write_reg(BMA4XY_INTR1_OUT_CTRL_ADDR, &int_enable, 1);
	bma4xy_i2c_delay(1);
	err += bma4xy_write_reg(BMA4XY_INTR_LATCH_ADDR, &latch_enable, 1);
	bma4xy_i2c_delay(1);
	if (err)
		GSE_ERR("map and enable interrupr1 failed err=%d", err);
	memset(&axis_remap_data, 0, sizeof(axis_remap_data));
	axis_remap_data.map_x_axis = 0;
	axis_remap_data.map_x_axis_sign = 0;
	axis_remap_data.map_y_axis = 1;
	axis_remap_data.map_y_axis_sign = 1;
	axis_remap_data.map_z_axis = 2;
	axis_remap_data.map_z_axis_sign = 1;
	err = BMA4XY_CALL_API(remap_axes)(&axis_remap_data);
	if (err) {
		GSE_ERR("write axis_remap failed");
		return err;
	}
//<--[SM31][Sensors][JasonHsing] Significant motion threshold for CTS 20170313 BEGIN --
	err = BMA4XY_CALL_API(significant_motion_set_threshold)(614);
	if (err) {
		GSE_ERR("significant_motion_set_threshold failed");
		return err;
	}
	mdelay(1);
//-->[SM31][Sensors][JasonHsing] Significant motion threshold for CTS 20170313 END --
	return err;
}

int bma4xy_init_fifo_config(
	struct bma4xy_client_data *client_data)
{
	int err = 0;
	err = BMA4XY_CALL_API(set_fifo_header_enable)(BMA4XY_ENABLE);
	if (err)
		GSE_ERR("enable fifo header failed err=%d", err);
	err = BMA4XY_CALL_API(set_fifo_time_enable)(BMA4XY_ENABLE);
	if (err)
		GSE_ERR("enable fifo timer failed err=%d", err);
	return err;
}

int bma4xy_update_config_stream(
	struct bma4xy_client_data *client_data, int choose)
{
	char *name;
	int err = 0;
	uint8_t crc_check = 0;
	int8_t download_result;

	switch (choose) {
	case 1:
	name = "android.tbin";
	break;
	case 2:
	name = "legacy.tbin";
	break;
	default:
	GSE_ERR("no choose fw = %d,use dafault ", choose);
	name = "bma4xy_config_stream";
	break;
	}
	GSE_LOG("choose the config_stream %s", name);
	if ((choose == 1) || (choose == 2)) {
	} else if (choose == 3) {
		err = bma4xy_config_stream_data(&download_result);
		if (err)
			GSE_ERR("download config stream failer");
		bma4xy_i2c_delay(1000);
		err = bma4xy_read_reg(BMA4XY_GPIO_7_REG,
		&crc_check, BMA4XY_READ_LENGTH);
		if (err)
			GSE_ERR("reading CRC failer");
		if (crc_check != BMA4XY_TITAN_INITIALIZED)
			GSE_ERR("crc check error %x", crc_check);
	}
	return err;
}

static ssize_t bma4xy_store_load_config_stream(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long choose = 0;
	int err = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = kstrtoul(buf, 10, &choose);
	GSE_LOG("config_stream_choose %ld", choose);
	err = bma4xy_update_config_stream(client_data, choose);
	if (err) {
		GSE_ERR("config_stream load error");
		return count;
	}
	err = bma4xy_init_after_config_stream_load(client_data);
	if (err) {
		GSE_ERR("bma4xy_init_after_config_stream_load error");
		return count;
	}
	return count;
}
//<--[SM31][Sensors][JasonHsing] Modify G-sensor driver to reduce kernel initial time 20161020 BEGIN --
void bma4xy_load_config_stream(struct work_struct *work)
{
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	int choose = 0;
	int err = 0;
	choose = 3;
	err = bma4xy_update_config_stream(client_data, 3);
	if (err) {
		GSE_ERR("config_stream load error");
		//return err;
	}
	err = bma4xy_init_after_config_stream_load(client_data);
	if (err) {
		GSE_ERR("bma4xy_init_after_config_stream_load error");
		//return err;
	}
//<--[SM31][Sensors][JasonHsing] Significant motion skiptime for CTS 20161102 BEGIN --
	err = BMA4XY_CALL_API(significant_motion_set_skiptime)(300);
	if (err) {
		GSE_ERR("significant_motion_set_skiptime failed");
		//return err;
	}
	mdelay(1);
//-->[SM31][Sensors][JasonHsing] Significant motion skiptime for CTS 20161102 END --
}
//-->[SM31][Sensors][JasonHsing] Modify G-sensor driver to reduce kernel initial time 20161230 END --
static ssize_t bma4xy_show_fifo_data_out_frame(struct device_driver *ddri, char *buf)
{
	int err = 0;
	uint16_t fifo_bytecount = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	if (!client_data->fifo_mag_enable && !client_data->fifo_acc_enable) {
		GSE_ERR("no selsect sensor fifo\n");
		return -EINVAL;
	}
	err = BMA4XY_CALL_API(fifo_length)(&fifo_bytecount);
	if (err < 0) {
		GSE_ERR("read fifo_len err=%d", err);
		return -EINVAL;
	}
	if (fifo_bytecount == 0)
		return 0;
	err = client_data->device.bus_read(client_data->device.dev_addr,
		BMA4XY_FIFO_DATA_ADDR, buf,
		fifo_bytecount);
	if (err) {
		GSE_ERR("read fifo leght err");
		return -EINVAL;
	}
	return fifo_bytecount;
}

static ssize_t bma4xy_show_reg_sel(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	if (client_data == NULL) {
		GSE_ERR("Invalid client_data pointer");
		return -ENODEV;
	}
	return snprintf(buf, 64, "reg=0X%02X, len=%d\n",
		client_data->reg_sel, client_data->reg_len);
}

static ssize_t bma4xy_store_reg_sel(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	ssize_t ret;

	if (client_data == NULL) {
		GSE_ERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = sscanf(buf, "%11X %11d",
		&client_data->reg_sel, &client_data->reg_len);
	if (ret != 2) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	return count;
}

static ssize_t bma4xy_show_reg_val(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	ssize_t ret;
	u8 reg_data[128], i;
	int pos;

	if (client_data == NULL) {
		GSE_ERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = client_data->device.bus_read(client_data->device.dev_addr,
		client_data->reg_sel,
		reg_data, client_data->reg_len);
	if (ret < 0) {
		GSE_ERR("Reg op failed");
		return ret;
	}
	pos = 0;
	for (i = 0; i < client_data->reg_len; ++i) {
		pos += snprintf(buf + pos, 16, "%02X", reg_data[i]);
		buf[pos++] = (i + 1) % 16 == 0 ? '\n' : ' ';
	}
	if (buf[pos - 1] == ' ')
		buf[pos - 1] = '\n';
	return pos;
}

static ssize_t bma4xy_store_reg_val(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	ssize_t ret;
	u8 reg_data[128];
	int i, j, status, digit;

	if (client_data == NULL) {
		GSE_ERR("Invalid client_data pointer");
		return -ENODEV;
	}
	status = 0;
	for (i = j = 0; i < count && j < client_data->reg_len; ++i) {
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\t' ||
			buf[i] == '\r') {
			status = 0;
			++j;
			continue;
		}
		digit = buf[i] & 0x10 ? (buf[i] & 0xF) : ((buf[i] & 0xF) + 9);
		GSE_LOG("digit is %d", digit);
		switch (status) {
		case 2:
			++j; /* Fall thru */
		case 0:
			reg_data[j] = digit;
			status = 1;
			break;
		case 1:
			reg_data[j] = reg_data[j] * 16 + digit;
			status = 2;
			break;
		}
	}
	if (status > 0)
		++j;
	if (j > client_data->reg_len)
		j = client_data->reg_len;
	else if (j < client_data->reg_len) {
		GSE_ERR("Invalid argument");
		return -EINVAL;
	}
	GSE_LOG("Reg data read as");
	for (i = 0; i < j; ++i)
		GSE_LOG("%d", reg_data[i]);
	ret = client_data->device.bus_write(client_data->device.dev_addr,
		client_data->reg_sel,
		reg_data, client_data->reg_len);
	if (ret < 0) {
		GSE_ERR("Reg op failed");
		return ret;
	}
	return count;
}

static ssize_t bma4xy_show_driver_version(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);


	if (client_data == NULL) {
		GSE_ERR("Invalid client_data pointer");
		return -ENODEV;
	}
	return snprintf(buf, 128,
		"Driver version: %s Config_stream version :%s\n",
		DRIVER_VERSION, CONFIG_STREAM_VERSION);
}
static ssize_t bma4xy_show_config_stream_version(struct device_driver *ddri, char *buf)
{
	int err;
	struct bma4xy_config_stream_header header;
	memset(&header, 0, sizeof(header));
	err = BMA4XY_CALL_API(get_config_stream_header)(&header);
	if (err < 0)
		GSE_ERR("read failed");
	return snprintf(buf, PAGE_SIZE,
		"mager_version %d minor_version %d image_type %d\n",
		header.major_version, header.minor_version, header.image_type);
}
static ssize_t bma4xy_show_avail_sensor(struct device_driver *ddri, char *buf)
{
	uint16_t avail_sensor = 0;

	#if defined(BMA420)
		avail_sensor = 420;
	#elif defined(BMA421)
		avail_sensor = 421;
	#elif defined(BMA422)
		avail_sensor = 422;
	#elif defined(BMA455)
		avail_sensor = 455;
	#elif defined(BMA456)
		avail_sensor = 456;
	#endif
	return snprintf(buf, 32, "%d\n", avail_sensor);
}
#if defined(BMA422) || defined(BMA455)
static ssize_t bma4xy_show_sig_motion_threadhold(struct device_driver *ddri, char *buf)
{
	int err;
	uint16_t sig_mot_threadhold;

	err = BMA4XY_CALL_API(significant_motion_get_threshold)(
		&sig_mot_threadhold);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", sig_mot_threadhold);
}
static ssize_t bma4xy_store_sig_motion_threadhold(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long sig_mot_threadhold;

	err = kstrtoul(buf, 10, &sig_mot_threadhold);
	if (err)
		return err;
	GSE_LOG("sig_mot_threadhold %ld", sig_mot_threadhold);
	err = BMA4XY_CALL_API(significant_motion_set_threshold)(
		sig_mot_threadhold);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_sig_motion_skiptime(struct device_driver *ddri, char *buf)
{
	int err;
	uint16_t sig_mot_skiptime;

	err = BMA4XY_CALL_API(significant_motion_get_skiptime)(
		&sig_mot_skiptime);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", sig_mot_skiptime);
}
static ssize_t bma4xy_store_sig_motion_skiptime(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long sig_mot_skiptime;

	err = kstrtoul(buf, 10, &sig_mot_skiptime);
	if (err)
		return err;
	GSE_LOG("sig_mot_skiptime %ld", sig_mot_skiptime);
	err = BMA4XY_CALL_API(significant_motion_set_skiptime)(
		sig_mot_skiptime);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_sig_motion_prooftime(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char sig_mot_prooftime;

	err = BMA4XY_CALL_API(significant_motion_get_prooftime)(
		&sig_mot_prooftime);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", sig_mot_prooftime);
}
static ssize_t bma4xy_store_sig_motion_prooftime(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long sig_mot_prooftime;

	err = kstrtoul(buf, 10, &sig_mot_prooftime);
	if (err)
		return err;
	GSE_LOG("sig_mot_prooftime %ld", sig_mot_prooftime);
	err = BMA4XY_CALL_API(significant_motion_set_prooftime)(
		sig_mot_prooftime);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
#endif
#if defined(BMA422) || defined(BMA455)
static ssize_t bma4xy_show_tilt_threshold(struct device_driver *ddri, char *buf)
{
	int err;
	uint8_t tilt_threshold;

	err = BMA4XY_CALL_API(tilt_get_threshold)(&tilt_threshold);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", tilt_threshold);
}
static ssize_t bma4xy_store_tilt_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long tilt_threshold;

	err = kstrtoul(buf, 10, &tilt_threshold);
	if (err)
		return err;
	GSE_LOG("tilt_threshold %ld", tilt_threshold);
	err = BMA4XY_CALL_API(tilt_set_threshold)(tilt_threshold);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
#endif
#if defined(BMA421) || defined(BMA422) || defined(BMA455)
static ssize_t bma4xy_show_step_counter_val(struct device_driver *ddri, char *buf)
{
	int err = 0;
	uint32_t step_counter_val = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	err = BMA4XY_CALL_API(step_counter_output)(&step_counter_val);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	GSE_LOG("val %u", step_counter_val);
	if (client_data->err_int_trigger_num == 0) {
		client_data->step_counter_val = step_counter_val;
		GSE_LOG("report val %u", client_data->step_counter_val);
		err = snprintf(buf, 96, "%u\n", client_data->step_counter_val);
		client_data->step_counter_temp = client_data->step_counter_val;
	} else {
		GSE_LOG("after err report val %u",
			client_data->step_counter_val + step_counter_val);
		err = snprintf(buf, 96, "%u\n",
			client_data->step_counter_val + step_counter_val);
		client_data->step_counter_temp =
			client_data->step_counter_val + step_counter_val;
	}
	return err;
}
static ssize_t bma4xy_show_step_counter_watermark(struct device_driver *ddri, char *buf)
{
	int err = 0;
	uint16_t watermark;
	err = BMA4XY_CALL_API(step_counter_get_watermark)(&watermark);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", watermark);
}
static ssize_t bma4xy_store_step_counter_watermark(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long step_watermark;

	err = kstrtoul(buf, 10, &step_watermark);
	if (err)
		return err;
	GSE_LOG("watermark step_counter %ld", step_watermark);
	err = BMA4XY_CALL_API(step_counter_set_watermark)(step_watermark);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_store_step_counter_reset(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long reset_counter;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = kstrtoul(buf, 10, &reset_counter);
	if (err)
		return err;
	GSE_LOG("reset_counter %ld", reset_counter);
	err = BMA4XY_CALL_API(reset_step_counter)();
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	client_data->step_counter_val = 0;
	client_data->step_counter_temp = 0;
	return count;
}
#endif
#if defined(BMA420) || defined(BMA421) || defined(BMA422)
static ssize_t bma4xy_show_anymotion_threshold(struct device_driver *ddri, char *buf)
{
	int err;
	uint16_t anymon_threshold;

	err = BMA4XY_CALL_API(anymotion_get_threshold)(&anymon_threshold);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", anymon_threshold);
}
static ssize_t bma4xy_store_anymotion_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long anymon_threshold;

	err = kstrtoul(buf, 10, &anymon_threshold);
	if (err)
		return err;
	GSE_LOG("anymon_threshold %ld", anymon_threshold);
	err = BMA4XY_CALL_API(anymotion_set_threshold)(anymon_threshold);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_anymotion_duration(struct device_driver *ddri, char *buf)
{
	int err = 0;
	uint16_t anymon_duration;

	err = BMA4XY_CALL_API(anymotion_get_duration)(&anymon_duration);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", anymon_duration);
}
static ssize_t bma4xy_store_anymotion_duration(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long anymon_duration;

	err = kstrtoul(buf, 10, &anymon_duration);
	if (err)
		return err;
	GSE_LOG("anymon_duration %ld", anymon_duration);
	err = BMA4XY_CALL_API(anymotion_set_duration)(anymon_duration);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_store_anymotion_nomotion_sel(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long anynomotion_sel;

	err = kstrtoul(buf, 10, &anynomotion_sel);
	if (err)
		return err;
	GSE_LOG("anymon_duration %ld", anynomotion_sel);
	err = BMA4XY_CALL_API(anymotion_nomotion_selection)(anynomotion_sel);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
#endif
#if defined(BMA420) || defined(BMA456)
static ssize_t bma4xy_store_orientation_ud(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long orientation_ud;

	err = kstrtoul(buf, 10, &orientation_ud);
	if (err)
		return err;
	GSE_LOG("orientation_ud %ld", orientation_ud);
	err = BMA4XY_CALL_API(orientation_ud_enable)(orientation_ud);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_orientation_mode(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char orientation_mode;

	err = BMA4XY_CALL_API(orientation_get_mode)(&orientation_mode);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", orientation_mode);
}
static ssize_t bma4xy_store_orientation_mode(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long orientation_mode;

	err = kstrtoul(buf, 10, &orientation_mode);
	if (err)
		return err;
	GSE_LOG("orientation_mode %ld", orientation_mode);
	err = BMA4XY_CALL_API(orientation_set_mode)(orientation_mode);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_orientation_blocking(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char orientation_blocking;

	err = BMA4XY_CALL_API(orientation_get_blocking)(&orientation_blocking);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", orientation_blocking);
}
static ssize_t bma4xy_store_orientation_blocking(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long orientation_blocking;

	err = kstrtoul(buf, 10, &orientation_blocking);
	if (err)
		return err;
	GSE_LOG("orientation_blocking %ld", orientation_blocking);
	err = BMA4XY_CALL_API(orientation_set_blocking)(orientation_blocking);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_orientation_theta(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char orientation_theta;

	err = BMA4XY_CALL_API(orientation_get_theta)(&orientation_theta);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", orientation_theta);
}
static ssize_t bma4xy_store_orientation_theta(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long orientation_theta;

	err = kstrtoul(buf, 10, &orientation_theta);
	if (err)
		return err;
	GSE_LOG("orientation_theta %ld", orientation_theta);
	err = BMA4XY_CALL_API(orientation_set_theta)(orientation_theta);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_orientation_hysteresis(struct device_driver *ddri, char *buf)
{
	int err;
	uint16_t orientation_hysteresis;

	err = BMA4XY_CALL_API(orientation_get_hysteresis)(
		&orientation_hysteresis);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", orientation_hysteresis);
}
static ssize_t bma4xy_store_orientation_hysteresis(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long orientation_hysteresis;

	err = kstrtoul(buf, 10, &orientation_hysteresis);
	if (err)
		return err;
	GSE_LOG("orientation_hysteresis %ld", orientation_hysteresis);
	err = BMA4XY_CALL_API(orientation_set_hysteresis)(
		orientation_hysteresis);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
#endif
#if defined(BMA420) || defined(BMA456)
static ssize_t bma4xy_show_flat_theta(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char flat_theta;

	err = BMA4XY_CALL_API(flat_get_theta)(&flat_theta);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", flat_theta);
}
static ssize_t bma4xy_store_flat_theta(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long flat_theta;

	err = kstrtoul(buf, 10, &flat_theta);
	if (err)
		return err;
	GSE_LOG("flat_theta %ld", flat_theta);
	err = BMA4XY_CALL_API(flat_set_theta)(flat_theta);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_flat_holdtime(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char flat_holdtime;

	err = BMA4XY_CALL_API(flat_get_holdtime)(&flat_holdtime);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", flat_holdtime);
}
static ssize_t bma4xy_store_flat_holdtime(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long flat_holdtime;

	err = kstrtoul(buf, 10, &flat_holdtime);
	if (err)
		return err;
	GSE_LOG("flat_holdtime %ld", flat_holdtime);
	err = BMA4XY_CALL_API(flat_set_holdtime)(flat_holdtime);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_flat_hysteresis(struct device_driver *ddri, char *buf)
{
	int err;
	unsigned char flat_hysteresis;

	err = BMA4XY_CALL_API(flat_get_hysteresis)(&flat_hysteresis);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", flat_hysteresis);
}
static ssize_t bma4xy_store_flat_hysteresis(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long flat_hysteresis;

	err = kstrtoul(buf, 10, &flat_hysteresis);
	if (err)
		return err;
	GSE_LOG("flat_hysteresis %ld", flat_hysteresis);
	err = BMA4XY_CALL_API(flat_set_hysteresis)(flat_hysteresis);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
#endif
#if defined(BMA420) || defined(BMA456)
static ssize_t bma4xy_show_highg_threshold(struct device_driver *ddri, char *buf)
{
	int err;
	uint16_t highg_threshold;

	err = BMA4XY_CALL_API(high_g_get_threshold)(&highg_threshold);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", highg_threshold);
}
static ssize_t bma4xy_store_highg_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long highg_threshold;

	err = kstrtoul(buf, 10, &highg_threshold);
	if (err)
		return err;
	GSE_LOG("highg_threshold %ld", highg_threshold);
	err = BMA4XY_CALL_API(high_g_set_threshold)(highg_threshold);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_highg_hysteresis(struct device_driver *ddri, char *buf)
{
	int err;
	uint16_t highg_hysteresis;

	err = BMA4XY_CALL_API(high_g_get_hysteresis)(&highg_hysteresis);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", highg_hysteresis);
}
static ssize_t bma4xy_store_highg_hysteresis(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long highg_hysteresis;

	err = kstrtoul(buf, 10, &highg_hysteresis);
	if (err)
		return err;
	GSE_LOG("highg_hysteresis %ld", highg_hysteresis);
	err = BMA4XY_CALL_API(high_g_set_hysteresis)(highg_hysteresis);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_highg_duration(struct device_driver *ddri, char *buf)
{
	int err;
	uint16_t highg_duration;

	err = BMA4XY_CALL_API(high_g_get_duration)(&highg_duration);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", highg_duration);
}
static ssize_t bma4xy_store_highg_duration(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long highg_duration;

	err = kstrtoul(buf, 10, &highg_duration);
	if (err)
		return err;
	GSE_LOG("highg_duration %ld", highg_duration);
	err = BMA4XY_CALL_API(high_g_set_duration)(highg_duration);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
#endif
#if defined(BMA420) || defined(BMA456)
static ssize_t bma4xy_show_lowg_threshold(struct device_driver *ddri, char *buf)
{
	int err;
	uint16_t lowg_threshold;

	err = BMA4XY_CALL_API(low_g_get_threshold)(&lowg_threshold);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", lowg_threshold);
}
static ssize_t bma4xy_store_lowg_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long lowg_threshold;

	err = kstrtoul(buf, 10, &lowg_threshold);
	if (err)
		return err;
	GSE_LOG("lowg_threshold %ld", lowg_threshold);
	err = BMA4XY_CALL_API(low_g_set_threshold)(lowg_threshold);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_lowg_hysteresis(struct device_driver *ddri, char *buf)
{
	int err;
	uint16_t lowg_hysteresis;

	err = BMA4XY_CALL_API(low_g_get_hysteresis)(&lowg_hysteresis);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", lowg_hysteresis);
}
static ssize_t bma4xy_store_lowg_hysteresis(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long lowg_hysteresis;

	err = kstrtoul(buf, 10, &lowg_hysteresis);
	if (err)
		return err;
	GSE_LOG("lowg_hysteresis %ld", lowg_hysteresis);
	err = BMA4XY_CALL_API(low_g_set_hysteresis)(lowg_hysteresis);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
static ssize_t bma4xy_show_lowg_duration(struct device_driver *ddri, char *buf)
{
	int err;
	uint16_t lowg_duration;

	err = BMA4XY_CALL_API(low_g_get_duration)(&lowg_duration);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", lowg_duration);
}
static ssize_t bma4xy_store_lowg_duration(struct device_driver *ddri, const char *buf, size_t count)
{
	int err = 0;
	unsigned long lowg_duration;

	err = kstrtoul(buf, 10, &lowg_duration);
	if (err)
		return err;
	GSE_LOG("lowg_duration %ld", lowg_duration);
	err = BMA4XY_CALL_API(low_g_set_duration)(lowg_duration);
	if (err) {
		GSE_ERR("write failed");
		return err;
	}
	return count;
}
#endif
#if defined(BMA420) || defined(BMA456)
static ssize_t bma4xy_show_tap_type(struct device_driver *ddri, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	return snprintf(buf, 8, "%d\n", client_data->tap_type);
}
static ssize_t bma4xy_store_tap_type(struct device_driver *ddri, const char *buf, size_t count)
{
	int32_t ret = 0;
	unsigned long data;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = kstrtoul(buf, 16, &data);
	if (ret)
		return ret;
	client_data->tap_type = (uint8_t)data;
	return count;
}
#endif
int bma4xy_reinit_after_error_interrupt(
	struct bma4xy_client_data *client_data)
{
	int err = 0;
	uint8_t data = 0;
	int8_t download_result = 0;
	uint8_t crc_check = 0;
	client_data->err_int_trigger_num += 1;
	client_data->step_counter_val = client_data->step_counter_temp;
	/*reset the bma4xy*/
	err = BMA4XY_CALL_API(set_command_register)(0xB6);
	if (!err)
		GSE_LOG("reset chip");
	/*reinit the fifo config*/
	err = bma4xy_init_fifo_config(client_data);
	if (err)
		GSE_ERR("fifo init failed");
	/*reload the config_stream*/
	err = bma4xy_config_stream_data(&download_result);
	if (err)
		GSE_ERR("download config stream failer");
	bma4xy_i2c_delay(200);
	err = bma4xy_read_reg(BMA4XY_GPIO_7_REG,
	&crc_check, BMA4XY_READ_LENGTH);
	if (err)
		GSE_ERR("reading CRC failer");
	if (crc_check != BMA4XY_TITAN_INITIALIZED)
		GSE_ERR("crc check error %x", crc_check);
	/*reconfig interrupt and remap*/
	err = bma4xy_init_after_config_stream_load(client_data);
	if (err)
		GSE_ERR("reconfig interrupt and remap ");
	/*reinit the virtual sensor*/
	#if defined(BMA422) || defined(BMA455)
	if (client_data->sigmotion_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(significant_motion_enable)(
			BMA4XY_ENABLE) < 0)
			GSE_ERR("set BMA4XY_SIG_MOTION error");
	}
	bma4xy_i2c_delay(2);
	#endif
	#if defined(BMA422) || defined(BMA455) || defined(BMA421)
	if (client_data->stepdet_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(step_detector_enable)(BMA4XY_ENABLE) < 0)
			GSE_ERR("set BMA4XY_STEP_COUNTER error");
	}
	#endif
	bma4xy_i2c_delay(2);
	#if defined(BMA422) || defined(BMA455) || defined(BMA421)
	if (client_data->stepcounter_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(step_counter_enable)(BMA4XY_ENABLE) < 0)
			GSE_ERR("set BMA4XY_STEP_COUNTER error");
	}
	#endif
	bma4xy_i2c_delay(2);
	#if defined(BMA422) || defined(BMA455)
	if (client_data->tilt_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(tilt_enable)(BMA4XY_ENABLE) < 0)
			GSE_ERR("set BMA4XY_TILT error");
	}
	#endif
	#if defined(BMA422) || defined(BMA455)
	if (client_data->pickup_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(pickup_enable)(BMA4XY_ENABLE) < 0)
			GSE_ERR("set BMA4XY_PICKUP error");
	}
	bma4xy_i2c_delay(2);
	#endif
	#if defined(BMA422) || defined(BMA455)
	if (client_data->glance_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(glance_enable)(BMA4XY_ENABLE) < 0)
			GSE_ERR("set BMA4XY_GLANCE_DETECTOR error");
	}
	bma4xy_i2c_delay(2);
	#endif
	#if defined(BMA422) || defined(BMA455)
	if (client_data->wakeup_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(wakeup_enable)(BMA4XY_ENABLE) < 0)
			GSE_ERR("set BMA4XY_WAKEUP error");
	}
	bma4xy_i2c_delay(2);
	#endif
	#if defined(BMA420) || defined(BMA421) || defined(BMA422)
	if (client_data->anymotion_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(anymotion_enable_axis)(BMA4XY_ENABLE) < 0)
			GSE_ERR("set BMA4XY_ANY_MOTION error");
	}
	bma4xy_i2c_delay(2);
	#endif
	#if defined(BMA420) || defined(BMA456)
	if (client_data->orientation_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(orientation_enable)(BMA4XY_ENABLE) < 0)
			GSE_ERR("set BMA4XY_ORIENTATION error");
	}
	bma4xy_i2c_delay(2);
	#endif
	#if defined(BMA420) || defined(BMA456)
	if (client_data->flat_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(flat_enable)(BMA4XY_ENABLE) < 0)
			GSE_ERR("set BMA4XY_FLAT error");
	}
	bma4xy_i2c_delay(2);
	#endif
	#if defined(BMA420) || defined(BMA456)
	if (client_data->tap_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(tap_enable)(
			BMA4XY_ENABLE, client_data->tap_type) < 0)
			GSE_ERR("set BMA4XY_TAP error");
	}
	bma4xy_i2c_delay(2);
	#endif
	#if defined(BMA420) || defined(BMA456)
	if (client_data->highg_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(high_g_enable_axis)(BMA4XY_ENABLE) < 0)
			GSE_ERR("set BMA4XY_HIGH_G error");
	}
	#endif
	#if defined(BMA420) || defined(BMA456)
	if (client_data->lowg_enable == BMA4XY_ENABLE) {
		if (BMA4XY_CALL_API(low_g_enable)(BMA4XY_ENABLE) < 0)
			GSE_ERR("set BMA4XY_LOW_G error");
	}
	bma4xy_i2c_delay(2);
	#endif
	/*reinit acc*/
	if (client_data->acc_odr != 0) {
		data = client_data->acc_odr;
		if (data == 4)
			data = 0x74;
		else
			data |= 0xA0;
		err = client_data->device.bus_write(
			client_data->device.dev_addr,
			0x40, &data, 1);
		if (err)
			GSE_ERR("set acc_odr faliled");
		bma4xy_i2c_delay(2);
	}
	if (client_data->acc_pm == 0)
		err = BMA4XY_CALL_API(set_accel_enable)(1);
	if (err)
		GSE_ERR("set acc_op_mode failed");
	bma4xy_i2c_delay(2);
	err = BMA4XY_CALL_API(set_fifo_accel_enable)(
		client_data->fifo_acc_enable);
	if (err)
		GSE_ERR("set acc_fifo_enable faliled");
	bma4xy_i2c_delay(5);
	return 0;
}
#if defined(BMA4XY_ENABLE_INT1) || defined(BMA4XY_ENABLE_INT2)

static void bma4xy_uc_function_handle(
	struct bma4xy_client_data *client_data, uint8_t status)
{
	int err = 0;
#if defined(BMA420) || defined(BMA456)
	unsigned char uc_gpio[3] = {0};
#endif
	if(status & ERROR_INT_OUT) {
		err = bma4xy_reinit_after_error_interrupt(client_data);
		if (err)
			GSE_ERR("reinit failed");
	}
#if defined(BMA422) || defined(BMA421) || defined(BMA455)
//<--[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 BEGIN --
#if 1
	if ((status & STEP_DET_OUT) == 0x02)
		step_notify(TYPE_STEP_DETECTOR);
	if ((status & SIG_MOTION_OUT) == 0x01)
		step_notify(TYPE_SIGNIFICANT);
#endif
//-->[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 END --
#endif
#if defined(BMA420) || defined(BMA456)
	if (client_data->orientation_enable)
		err = bma4xy_read_reg(BMA4XY_GPIO_0_REG, &uc_gpio[0], 1);
	if (client_data->highg_enable)
		err += bma4xy_read_reg(BMA4XY_GPIO_1_REG, &uc_gpio[1], 1);
	err += bma4xy_read_reg(BMA4XY_GPIO_7_REG, &uc_gpio[2], 1);
	if (err) {
		GSE_ERR("read uc_gpio failed");
		return;
	}
	GSE_LOG("%d %d %d", uc_gpio[0], uc_gpio[1], uc_gpio[2]);
#endif
}

static void bma4xy_irq_work_func(struct work_struct *work)
{
	struct bma4xy_client_data *client_data = container_of(work,
		struct bma4xy_client_data, irq_work);
	unsigned char int_status[2] = {0, 0};
	int err = 0;

	int in_suspend_copy;
	in_suspend_copy = atomic_read(&client_data->in_suspend);

	/*read the interrut status two register*/
	err = client_data->device.bus_read(client_data->device.dev_addr,
				BMA4XY_INTR_STAT_0_ADDR, int_status, 2);
	if (err)
		return;
	GSE_LOG("int_status0 = 0x%x int_status1 =0x%x",
		int_status[0], int_status[1]);
	if (in_suspend_copy &&
		((int_status[0] & STEP_DET_OUT) == 0x02)) {
		return;
	}
	if (int_status[0])
		bma4xy_uc_function_handle(client_data, (uint8_t)int_status[0]);
}

static void bma4xy_delay_sigmo_work_func(struct work_struct *work)
{
	struct bma4xy_client_data *client_data =
	container_of(work, struct bma4xy_client_data,
	delay_work_sig.work);
	unsigned char int_status[2] = {0, 0};
	int err = 0;
	/*read the interrut status two register*/
	err = client_data->device.bus_read(client_data->device.dev_addr,
				BMA4XY_INTR_STAT_0_ADDR, int_status, 2);
	if (err)
		return;
	GSE_LOG("int_status0 = %x int_status1 =%x",
		int_status[0], int_status[1]);
//<--[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 BEGIN --
	if ((int_status[0] & STEP_DET_OUT) == 0x02)
		step_notify(TYPE_STEP_DETECTOR);
	if ((int_status[0] & SIG_MOTION_OUT) == 0x01)
		step_notify(TYPE_SIGNIFICANT);
//-->[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 END --
}

static irqreturn_t bma4xy_irq_handle(int irq, void *handle)
{
	struct bma4xy_client_data *client_data = handle;
	int in_suspend_copy;
	in_suspend_copy = atomic_read(&client_data->in_suspend);
	/*this only deal with SIG_motion CTS test*/
	if ((in_suspend_copy == 1) &&
		((client_data->sigmotion_enable == 1) &&
		(client_data->tilt_enable != 1) &&
		(client_data->pickup_enable != 1) &&
		(client_data->glance_enable != 1) &&
		(client_data->wakeup_enable != 1))) {
		wake_lock_timeout(&client_data->wakelock, HZ);
		schedule_delayed_work(&client_data->delay_work_sig,
			msecs_to_jiffies(50));
	} else if ((in_suspend_copy == 1) &&
		((client_data->sigmotion_enable == 1) ||
		(client_data->tilt_enable == 1) ||
		(client_data->pickup_enable == 1) ||
		(client_data->glance_enable == 1) ||
		(client_data->wakeup_enable == 1))) {
		wake_lock_timeout(&client_data->wakelock, HZ);
		schedule_work(&client_data->irq_work);
	} else
		schedule_work(&client_data->irq_work);
	return IRQ_HANDLED;
}

static int bma4xy_request_irq(struct bma4xy_client_data *client_data)
{
	int err = 0;
//<--[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 BEGIN --
#if 1
	GSE_FUN();
	client_data->gpio_pin = 66;
	err = gpio_request_one(client_data->gpio_pin,
				GPIOF_IN, "bma4xy_interrupt");
	if (err < 0) {
		printk(KERN_INFO "bma4xy_request_irq FAILED irq_num= %d",client_data->gpio_pin);
		return err;
	}
	err = gpio_direction_input(client_data->gpio_pin);
	if (err < 0) {
		printk(KERN_INFO "bma4xy_request_irq gpio_direction_input FAILED irq_num= %d",client_data->gpio_pin);
		return err;
	}
	client_data->IRQ = gpio_to_irq(client_data->gpio_pin);
	err = request_irq(client_data->IRQ, bma4xy_irq_handle,
			IRQF_TRIGGER_RISING,
			SENSOR_NAME, client_data);
	if (err < 0) {
		printk(KERN_INFO "ERROR request_irq irq_num= %d IRQ_num=%d",client_data->gpio_pin, client_data->IRQ);
		return err;
	}
#else
	struct device_node *node = NULL;
	int ret = 0;
	GSE_FUN();
	node = of_find_compatible_node(NULL, NULL, "mediatek,gyroscope");
	if (node) {
		/*touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
		client_data->IRQ = irq_of_parse_and_map(node, 0);
		ret = request_irq(client_data->IRQ, bma4xy_irq_handle,
			IRQF_TRIGGER_RISING,
			SENSOR_NAME, client_data);
			if (ret > 0)
				printk("bma4xy request_irq IRQ LINE NOT AVAILABLE");
	} else {
		printk("[%s] bma4xy request_irq can not find touch eint device node!.", __func__);
	}
	printk(KERN_INFO "bma4xy_request_irq irq_num= %d IRQ_num=%d",client_data->gpio_pin, client_data->IRQ);
#endif
//-->[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 END --
	INIT_WORK(&client_data->irq_work, bma4xy_irq_work_func);
	INIT_DELAYED_WORK(&client_data->delay_work_sig,
		bma4xy_delay_sigmo_work_func);
	return err;
}
#endif

static int bma4xy_init_client(struct i2c_client *client, int reset_cali)
{
	struct bma4xy_client_data*obj = i2c_get_clientdata(client);
	int res = 0;

	GSE_FUN();
	res = BMA4XY_CheckDeviceID(client);
	if (res) {
		GSE_ERR("check device ID failed");
		return res;
	}

	res = BMA4XY_SetBWRate(client, BMA4XY_ACCEL_OUTPUT_DATA_RATE_100HZ);
	if (res) {
		GSE_ERR("setBWRate failed");
		return res;
	}

	res = BMA4XY_SetDataFormat(client, 	BMA4XY_ACCEL_RANGE_4G);
	if (res) {
		GSE_ERR("SetDataFormat failed");
		return res;
	}
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	res = BMA4XY_SetPowerMode(client, 1);	/* false);// */
	if (res) {
		GSE_ERR("SetPowerMode failed");
		return res;
	}
	if (0 != reset_cali) {
		/*reset calibration only in power on */
		res = BMA4XY_ResetCalibration(client);
		if (res) {
			GSE_ERR("ResetCalibration failed");
			return res;
		}
	}
//<--[SM31][Sensors][JasonHsing] Modify G-sensor driver to reduce kernel initial time 20161020 BEGIN --
	/*load config_stream*/
	INIT_DELAYED_WORK(&obj->delay_load_config, bma4xy_load_config_stream);
	schedule_delayed_work(&obj->delay_load_config, msecs_to_jiffies(1000));
#if 0
	res= bma4xy_load_config_stream(obj);
	if (res)
		GSE_ERR("init failed after load config_stream");
#endif
//-->[SM31][Sensors][JasonHsing] Modify G-sensor driver to reduce kernel initial time 20161230 END --
	GSE_LOG("BMA4XY_init_client OK!\n");
//<--[SM31][Sensors][JasonHsing] Significant motion skiptime for CTS 20161102 BEGIN --
	res = BMA4XY_CALL_API(significant_motion_set_skiptime)(300);
	if (res) {
		GSE_ERR("significant_motion_set_skiptime failed");
		return res;
	}
	mdelay(1);
//-->[SM31][Sensors][JasonHsing] Significant motion skiptime for CTS 20161102 END --
	return 0;
}

static int bma4xy_open(struct inode *inode, struct file *file)
{
	file->private_data = bma4xy_i2c_client;

	if (file->private_data == NULL) {
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int bma4xy_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long bma4xy_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct bma4xy_client_data *obj = (struct bma4xy_client_data *)i2c_get_clientdata(client);
	char strbuf[BMA4XY_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	/* GSE_FUN(f); */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err) {
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case GSENSOR_IOCTL_INIT:
		bma4xy_init_client(client, 0);
		break;

	case GSENSOR_IOCTL_READ_CHIPINFO:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		BMA4XY_ReadChipInfo(client, strbuf, BMA4XY_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_SENSORDATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		BMA4XY_SetPowerMode(client, true);
		BMA4XY_ReadSensorData(client, strbuf, BMA4XY_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_GAIN:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D))) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		BMA4XY_ReadRawData(client, strbuf);
		if (copy_to_user(data, &strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_SET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		if (atomic_read(&obj->suspend)) {
			GSE_ERR("Perform calibration in suspend state!!\n");
			err = -EINVAL;
		} else {
			cali[BMA4XY_ACC_AXIS_X] =
			sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[BMA4XY_ACC_AXIS_Y] =
			sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[BMA4XY_ACC_AXIS_Z] =
			sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			err = BMA4XY_WriteCalibration(client, cali);
		}
		break;

	case GSENSOR_IOCTL_CLR_CALI:
		err = BMA4XY_ResetCalibration(client);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		err = BMA4XY_ReadCalibration(client, cali);
		if (0 != err)
			break;
		sensor_data.x = cali[BMA4XY_ACC_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		sensor_data.y = cali[BMA4XY_ACC_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		sensor_data.z = cali[BMA4XY_ACC_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		break;
	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}
	return err;
}
#ifdef CONFIG_COMPAT
static long bma4xy_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;

	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
			return err;
		}
		break;
	case COMPAT_GSENSOR_IOCTL_SET_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;
	case COMPAT_GSENSOR_IOCTL_GET_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;
	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
			return err;
		}
		break;
	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}
#endif

static const struct file_operations bma4xy_fops = {
	.owner = THIS_MODULE,
	.open = bma4xy_open,
	.release = bma4xy_release,
	.unlocked_ioctl = bma4xy_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = bma4xy_compat_ioctl,
#endif
};

static struct miscdevice bma4xy_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &bma4xy_fops,
};

static int bma4xy_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	GSE_LOG("suspend function entrance");
	enable_irq_wake(client_data->IRQ);
	atomic_set(&client_data->in_suspend, 1);
//<--[SM31][Sensors][JasonHsing] Fix the g-sensor can not into suspend 20161109 BEGIN --
	bma4xy_set_advance_power_save(1);
//-->[SM31][Sensors][JasonHsing] Fix the g-sensor can not into suspend 20161109 END --
	return err;
}

static int bma4xy_i2c_resume(struct i2c_client *client)
{
	int err = 0;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	GSE_LOG("resume function entrance");
	disable_irq_wake(client_data->IRQ);
	atomic_set(&client_data->in_suspend, 0);
//<--[SM31][Sensors][JasonHsing] Fix the g-sensor can not into suspend 20161109 BEGIN --
	bma4xy_set_advance_power_save(0);
//-->[SM31][Sensors][JasonHsing] Fix the g-sensor can not into suspend 20161109 END --
	return err;
}
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma4xy_i2c_client;
	char strbuf[BMA4XY_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	BMA4XY_ReadChipInfo(client, strbuf, BMA4XY_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma4xy_i2c_client;
	char strbuf[BMA4XY_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	BMA4XY_ReadSensorData(client, strbuf, BMA4XY_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	int err, len = 0, mul;
	int tmp[BMA4XY_ACC_AXIS_NUM];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	err = BMA4XY_ReadOffset(client, obj->offset);
	if (0 != err)
		return -EINVAL;

	err = BMA4XY_ReadCalibration(client, tmp);
	if (0 != err)
		return -EINVAL;

	mul = obj->reso->sensitivity / bma4xy_acc_offset_resolution.sensitivity;
	len +=
	snprintf(buf + len, PAGE_SIZE - len,
		"[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
		obj->offset[BMA4XY_ACC_AXIS_X], obj->offset[BMA4XY_ACC_AXIS_Y],obj->offset[BMA4XY_ACC_AXIS_Z],
		obj->offset[BMA4XY_ACC_AXIS_X],obj->offset[BMA4XY_ACC_AXIS_Y], obj->offset[BMA4XY_ACC_AXIS_Z]);
	len +=
	snprintf(buf + len, PAGE_SIZE - len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
		obj->cali_sw[BMA4XY_ACC_AXIS_X], obj->cali_sw[BMA4XY_ACC_AXIS_Y], obj->cali_sw[BMA4XY_ACC_AXIS_Z]);

	len +=
	snprintf(buf + len, PAGE_SIZE - len,
		"[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
		obj->offset[BMA4XY_ACC_AXIS_X] * mul + obj->cali_sw[BMA4XY_ACC_AXIS_X],
		obj->offset[BMA4XY_ACC_AXIS_Y] * mul + obj->cali_sw[BMA4XY_ACC_AXIS_Y],
		obj->offset[BMA4XY_ACC_AXIS_Z] * mul + obj->cali_sw[BMA4XY_ACC_AXIS_Z],
		tmp[BMA4XY_ACC_AXIS_X], tmp[BMA4XY_ACC_AXIS_Y], tmp[BMA4XY_ACC_AXIS_Z]);
	return len;
}

//<--[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20170116 BEGIN --
static bool read_cali_value(void)
{
	struct i2c_client *client = bma4xy_i2c_client;
	int err, x, y, z;
	int dat[BMA4XY_ACC_AXIS_NUM];
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	char buffer[100]= {0};

	fp = filp_open(g_sencer_dat, O_RDONLY, 0755);
	if (IS_ERR(fp))
	{
		GSE_ERR("[%s] NO g-sencer.dat calibration file\n", __func__);
	}
	else
	{
		pos = 0;
		fs = get_fs();
		set_fs(KERNEL_DS);
		vfs_read(fp, buffer, sizeof(buffer), &pos);
		filp_close(fp, NULL);
		set_fs(fs);
	}
	if (!strncmp(buffer, "rst", 3)) {
		err = BMA4XY_ResetCalibration(client);
		if (0 != err)
			GSE_ERR("reset offset err = %d\n", err);
	} else if (3 == sscanf(buffer, "%d %d %d", &x, &y, &z)) {
		dat[BMA4XY_ACC_AXIS_X] = x;
		dat[BMA4XY_ACC_AXIS_Y] = y;
		dat[BMA4XY_ACC_AXIS_Z] = z;

		err = BMA4XY_WriteCalibration(client, dat);
		if (0 != err)
			GSE_ERR("write calibration err = %d\n", err);
	} else {
		GSE_ERR("invalid format\n");
	}
	first_read_cali = false;
	GSE_ERR("read_cali_value Done!!\n");
	return 0;
}
static ssize_t read_cali_value_test(struct device_driver *ddri, char *buf)
{
	read_cali_value();
	return snprintf(buf, PAGE_SIZE, "read_cali_value Done\n");
}
//-->[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20170116 END --
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = bma4xy_i2c_client;
	int err, x, y, z;
	int dat[BMA4XY_ACC_AXIS_NUM];

	if (!strncmp(buf, "rst", 3)) {
		err = BMA4XY_ResetCalibration(client);
		if (0 != err)
			GSE_ERR("reset offset err = %d\n", err);
//<--[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20161118 BEGIN --
	} else if (3 == sscanf(buf, "%d %d %d", &x, &y, &z)) {
//-->[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20161118 END --
		dat[BMA4XY_ACC_AXIS_X] = x;
		dat[BMA4XY_ACC_AXIS_Y] = y;
		dat[BMA4XY_ACC_AXIS_Z] = z;

		err = BMA4XY_WriteCalibration(client, dat);
		if (0 != err)
			GSE_ERR("write calibration err = %d\n", err);
	} else {
		GSE_ERR("invalid format\n");
	}
	return count;
}
//<--[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20161118 BEGIN --
#define G_Sensor_Data_Count 20
static ssize_t show_do_calibration(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma4xy_i2c_client;
	s32 sum[BMA4XY_ACC_AXIS_NUM];
	char strbuf[BMA4XY_BUFSIZE];
	int i = 0;
	int count = 0;
	int retcode = 0;
	int data[BMA4XY_ACC_AXIS_NUM];
	ssize_t len = 0;
	int err=0;
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	GSE_FUN();

	BMA4XY_ResetCalibration(client);

	if(err!=0)
	{
		printk("show_do_calibration BMA4XY_ResetCalibration fail:%d\n", err);
		return 0;
	}
	sum[BMA4XY_ACC_AXIS_X] = 0;
	sum[BMA4XY_ACC_AXIS_Y] = 0;
	sum[BMA4XY_ACC_AXIS_Z] = 0;

	for(count=0; count<G_Sensor_Data_Count; count++)
	{
		data[BMA4XY_ACC_AXIS_X] = 0;
		data[BMA4XY_ACC_AXIS_Y] = 0;
		data[BMA4XY_ACC_AXIS_Z] = 0;

		retcode = BMA4XY_ReadSensorData(client, strbuf, BMA4XY_BUFSIZE);

		if(retcode !=0)
		{
			printk("show_do_calibration retcode:%d\n", retcode);
			return 0;
		}

		sscanf(strbuf, "%x %x %x", &data[BMA4XY_ACC_AXIS_X], &data[BMA4XY_ACC_AXIS_Y], &data[BMA4XY_ACC_AXIS_Z]);
		printk("1 show_do_calibration %d %d %d\n", data[BMA4XY_ACC_AXIS_X], data[BMA4XY_ACC_AXIS_Y], data[BMA4XY_ACC_AXIS_Z]);

		sum[BMA4XY_ACC_AXIS_X] += data[BMA4XY_ACC_AXIS_X];
		sum[BMA4XY_ACC_AXIS_Y] += data[BMA4XY_ACC_AXIS_Y];
		sum[BMA4XY_ACC_AXIS_Z] += data[BMA4XY_ACC_AXIS_Z];

		mdelay(50);
	}

	//len += snprintf(buf+len, PAGE_SIZE-len, "A %d %d %d  \n", sum[BMA4XY_ACC_AXIS_X], sum[BMA4XY_ACC_AXIS_Y], sum[BMA4XY_ACC_AXIS_Z]);

	sum[BMA4XY_ACC_AXIS_X] = sum[BMA4XY_ACC_AXIS_X] / G_Sensor_Data_Count;
	sum[BMA4XY_ACC_AXIS_Y] = sum[BMA4XY_ACC_AXIS_Y] / G_Sensor_Data_Count;
	sum[BMA4XY_ACC_AXIS_Z] = sum[BMA4XY_ACC_AXIS_Z] / G_Sensor_Data_Count;

	sum[BMA4XY_ACC_AXIS_Z] = sum[BMA4XY_ACC_AXIS_Z] - GRAVITY_EARTH_1000;

	for(i=0; i<BMA4XY_ACC_AXIS_NUM; i++)//remap coorindate
	{
		sum[i] = -(sum[i]);
	}

	//len += snprintf(buf+len, PAGE_SIZE-len, "B %d %d %d  \n", sum[BMA4XY_ACC_AXIS_X], sum[BMA4XY_ACC_AXIS_Y], sum[BMA4XY_ACC_AXIS_Z]);

	sum[BMA4XY_ACC_AXIS_X] = sum[BMA4XY_ACC_AXIS_X] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
	sum[BMA4XY_ACC_AXIS_Y] = sum[BMA4XY_ACC_AXIS_Y] * obj->reso->sensitivity / GRAVITY_EARTH_1000;
	sum[BMA4XY_ACC_AXIS_Z] = sum[BMA4XY_ACC_AXIS_Z] * obj->reso->sensitivity / GRAVITY_EARTH_1000;

	printk("2 show_do_calibration %d %d %d , sens=%d \n",
	sum[BMA4XY_ACC_AXIS_X], sum[BMA4XY_ACC_AXIS_Y], sum[BMA4XY_ACC_AXIS_Z], obj->reso->sensitivity);

	BMA4XY_WriteCalibration(client, sum);

	//len += snprintf(buf+len, PAGE_SIZE-len, "C %d %d %d  \n", sum[BMA4XY_ACC_AXIS_X], sum[BMA4XY_ACC_AXIS_Y], sum[BMA4XY_ACC_AXIS_Z]);

	len += snprintf(buf+len, PAGE_SIZE-len, "%d %d %d\n", sum[BMA4XY_ACC_AXIS_X], sum[BMA4XY_ACC_AXIS_Y], sum[BMA4XY_ACC_AXIS_Z]);

	return len;
}
//-->[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20161118 END --
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{

	return snprintf(buf, PAGE_SIZE, "not support\n");
}

static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	int trace;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (!kstrtoint(buf, 16, &trace))
		atomic_set(&obj->trace, trace);
	else
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (int)count);

	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (obj->hw) {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d %d (%d %d)\n",
				obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id,
				obj->hw->power_vol);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
	}
	return len;
}

static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	unsigned char acc_op_mode;
	int err = 0;

	err = BMA4XY_CALL_API(get_accel_enable)(&acc_op_mode);
	if (err) {
		GSE_ERR("read failed");
		return err;
	}
	if (sensor_power)
		GSE_LOG("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		GSE_LOG("G sensor is in standby mode, sensor_power = %d\n", sensor_power);
	return snprintf(buf, PAGE_SIZE, "%x\n", acc_op_mode);
}

static ssize_t show_chip_orientation(struct device_driver *ddri, char *pbBuf)
{
	ssize_t _tLength = 0;

	GSE_LOG("[%s] default direction: %d\n", __func__, hw->direction);

	_tLength = snprintf(pbBuf, PAGE_SIZE, "default direction = %d\n", hw->direction);

	return _tLength;
}

static ssize_t store_chip_orientation(struct device_driver *ddri, const char *pbBuf, size_t tCount)
{
	int _nDirection = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *_pt_i2c_obj = i2c_get_clientdata(client);

	if (NULL == _pt_i2c_obj)
		return 0;

	if (!kstrtoint(pbBuf, 10, &_nDirection)) {
		if (hwmsen_get_convert(_nDirection, &_pt_i2c_obj->cvt))
			GSE_ERR("ERR: fail to set direction\n");
	}

	GSE_LOG("[%s] set direction: %d\n", __func__, _nDirection);

	return tCount;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(powerstatus, S_IRUGO, show_power_status_value, NULL);
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation, store_chip_orientation);
static DRIVER_ATTR(chip_id, S_IRUGO, bma4xy_show_chip_id, NULL);
static DRIVER_ATTR(acc_op_mode, S_IWUSR | S_IRUGO,
	bma4xy_show_acc_op_mode, bma4xy_store_acc_op_mode);
static DRIVER_ATTR(acc_value, S_IRUGO, bma4xy_show_acc_value, NULL);
static DRIVER_ATTR(acc_range, S_IWUSR | S_IRUGO,
	bma4xy_show_acc_range, bma4xy_store_acc_range);
static DRIVER_ATTR(acc_odr, S_IWUSR | S_IRUGO,
	bma4xy_show_acc_odr, bma4xy_store_acc_odr);
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO,
	bma4xy_show_selftest, bma4xy_store_selftest);
static DRIVER_ATTR(avail_sensor, S_IRUGO,
	bma4xy_show_avail_sensor, NULL);
static DRIVER_ATTR(fifo_length, S_IRUGO,
	bma4xy_show_fifo_length, NULL);
static DRIVER_ATTR(acc_fifo_enable, S_IWUSR | S_IRUGO,
	bma4xy_show_fifo_acc_enable, bma4xy_store_fifo_acc_enable);
static DRIVER_ATTR(load_fw, S_IWUSR | S_IRUGO,
	bma4xy_show_load_config_stream, bma4xy_store_load_config_stream);
static DRIVER_ATTR(reg_sel, S_IWUSR | S_IRUGO,
	bma4xy_show_reg_sel, bma4xy_store_reg_sel);
static DRIVER_ATTR(reg_val, S_IWUSR | S_IRUGO,
	bma4xy_show_reg_val, bma4xy_store_reg_val);
static DRIVER_ATTR(int_map, S_IWUSR | S_IRUGO,
	bma4xy_show_int_mapping, bma4xy_store_int_mapping);
static DRIVER_ATTR(int_enable, S_IWUSR | S_IRUGO,
	bma4xy_show_int_enable, bma4xy_store_int_enable);
static DRIVER_ATTR(driver_version, S_IRUGO,
	bma4xy_show_driver_version, NULL);
static DRIVER_ATTR(config_stream_version, S_IRUGO,
	bma4xy_show_config_stream_version, NULL);
static DRIVER_ATTR(fifo_data_frame, S_IRUGO,
	bma4xy_show_fifo_data_out_frame, NULL);
static DRIVER_ATTR(fifo_flush, S_IWUSR | S_IRUGO,
	NULL, bma4xy_store_fifo_flush);
static DRIVER_ATTR(foc, S_IWUSR | S_IRUGO,
	bma4xy_show_foc, bma4xy_store_foc);
static DRIVER_ATTR(config_function, S_IWUSR | S_IRUGO,
	bma4xy_show_config_function, bma4xy_store_config_function);
static DRIVER_ATTR(axis_remapping, S_IWUSR | S_IRUGO,
	NULL, bma4xy_store_axis_remapping);
#if defined(BMA422) || defined(BMA455)
static DRIVER_ATTR(sig_threshold, S_IWUSR | S_IRUGO,
	bma4xy_show_sig_motion_threadhold, bma4xy_store_sig_motion_threadhold);
static DRIVER_ATTR(sig_skiptime, S_IWUSR | S_IRUGO,
	bma4xy_show_sig_motion_skiptime, bma4xy_store_sig_motion_skiptime);
static DRIVER_ATTR(sig_prooftime, S_IWUSR | S_IRUGO,
	bma4xy_show_sig_motion_prooftime, bma4xy_store_sig_motion_prooftime);
#endif
#if defined(BMA421) || defined(BMA422) || defined(BMA455)
static DRIVER_ATTR(step_counter_val, S_IRUGO,
	bma4xy_show_step_counter_val, NULL);
static DRIVER_ATTR(step_counter_watermark, S_IWUSR | S_IRUGO,
	bma4xy_show_step_counter_watermark,
	bma4xy_store_step_counter_watermark);
static DRIVER_ATTR(step_counter_reset, S_IWUSR | S_IRUGO,
	NULL, bma4xy_store_step_counter_reset);
#endif
#if defined(BMA422) || defined(BMA455)
static DRIVER_ATTR(tilt_threshold, S_IWUSR | S_IRUGO,
	bma4xy_show_tilt_threshold, bma4xy_store_tilt_threshold);
#endif
#if defined(BMA420) || defined(BMA456)
static DRIVER_ATTR(tap_type, S_IWUSR | S_IRUGO,
	bma4xy_show_tap_type, bma4xy_store_tap_type);
#endif

#if defined(BMA420) || defined(BMA421) || defined(BMA422)
static DRIVER_ATTR(anymotion_threshold, S_IWUSR | S_IRUGO,
	bma4xy_show_anymotion_threshold, bma4xy_store_anymotion_threshold);
static DRIVER_ATTR(anymotion_duration, S_IWUSR | S_IRUGO,
	bma4xy_show_anymotion_duration, bma4xy_store_anymotion_duration);
static DRIVER_ATTR(any_nomotion_sel, S_IWUSR | S_IRUGO,
	NULL, bma4xy_store_anymotion_nomotion_sel);
#endif
#if defined(BMA420) || defined(BMA456)
static DRIVER_ATTR(orientation_ud_en, S_IWUSR | S_IRUGO,
	NULL, bma4xy_store_orientation_ud);
static DRIVER_ATTR(orientation_mode, S_IWUSR | S_IRUGO,
	bma4xy_show_orientation_mode, bma4xy_store_orientation_mode);
static DRIVER_ATTR(orientation_blocking, S_IWUSR | S_IRUGO,
	bma4xy_show_orientation_blocking, bma4xy_store_orientation_blocking);
static DRIVER_ATTR(orientation_theta, S_IWUSR | S_IRUGO,
	bma4xy_show_orientation_theta, bma4xy_store_orientation_theta);
static DRIVER_ATTR(orientation_hysteresis, S_IWUSR | S_IRUGO,
	bma4xy_show_orientation_hysteresis,
	bma4xy_store_orientation_hysteresis);
#endif
#if defined(BMA420) || defined(BMA456)
static DRIVER_ATTR(flat_theta, S_IWUSR | S_IRUGO,
	bma4xy_show_flat_theta, bma4xy_store_flat_theta);
static DRIVER_ATTR(flat_hysteresis, S_IWUSR | S_IRUGO,
	bma4xy_show_flat_hysteresis, bma4xy_store_flat_hysteresis);
static DRIVER_ATTR(flat_holdtime, S_IWUSR | S_IRUGO,
	bma4xy_show_flat_holdtime, bma4xy_store_flat_holdtime);
#endif
#if defined(BMA420) || defined(BMA456)
static DRIVER_ATTR(highg_duration, S_IWUSR | S_IRUGO,
	bma4xy_show_highg_duration, bma4xy_store_highg_duration);
static DRIVER_ATTR(highg_hysteresis, S_IWUSR | S_IRUGO,
	bma4xy_show_highg_hysteresis, bma4xy_store_highg_hysteresis);
static DRIVER_ATTR(highg_threshold, S_IWUSR | S_IRUGO,
	bma4xy_show_highg_threshold, bma4xy_store_highg_threshold);
#endif
#if defined(BMA420) || defined(BMA456)
static DRIVER_ATTR(lowg_duration, S_IWUSR | S_IRUGO,
	bma4xy_show_lowg_duration, bma4xy_store_lowg_duration);
static DRIVER_ATTR(lowg_hysteresis, S_IWUSR | S_IRUGO,
	bma4xy_show_lowg_hysteresis, bma4xy_store_lowg_hysteresis);
static DRIVER_ATTR(lowg_threshold, S_IWUSR | S_IRUGO,
	bma4xy_show_lowg_threshold, bma4xy_store_lowg_threshold);
#endif
//<--[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20170116 BEGIN --
static DRIVER_ATTR(do_cali, S_IWUSR | S_IRUGO,
	show_do_calibration, NULL);
static DRIVER_ATTR(cali_test, S_IWUSR | S_IRUGO,
	read_cali_value_test, NULL);
//-->[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20170116 END --
/*----------------------------------------------------------------------------*/
static struct driver_attribute *bma4xy_attr_list[] = {
	&driver_attr_chipinfo,	/*chip information */
	&driver_attr_sensordata,	/*dump sensor data */
	&driver_attr_cali,	/*show calibration data */
	&driver_attr_firlen,	/*filter length: 0: disable, others: enable */
	&driver_attr_trace,	/*trace log */
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_orientation,
	&driver_attr_chip_id,
	&driver_attr_acc_op_mode,
	&driver_attr_acc_value,
	&driver_attr_acc_range,
	&driver_attr_acc_odr,
	&driver_attr_acc_fifo_enable,
	&driver_attr_fifo_length,
	&driver_attr_selftest,
	&driver_attr_avail_sensor,
	&driver_attr_foc,
	&driver_attr_driver_version,
	&driver_attr_load_fw,
	&driver_attr_fifo_data_frame,
	&driver_attr_fifo_flush,
	&driver_attr_reg_sel,
	&driver_attr_reg_val,
	&driver_attr_int_map,
	&driver_attr_int_enable,
	&driver_attr_config_function,
	&driver_attr_config_stream_version,
	&driver_attr_axis_remapping,
#if defined(BMA422) || defined(BMA455)
	&driver_attr_sig_threshold,
	&driver_attr_sig_skiptime,
	&driver_attr_sig_prooftime,
#endif
#if defined(BMA421) || defined(BMA422) || defined(BMA455)
	&driver_attr_step_counter_val,
	&driver_attr_step_counter_watermark,
	&driver_attr_step_counter_reset,
#endif
#if defined(BMA422) || defined(BMA455)
	&driver_attr_tilt_threshold,
#endif
#if defined(BMA420) || defined(BMA421) || defined(BMA422)
	&driver_attr_anymotion_threshold,
	&driver_attr_anymotion_duration,
	&driver_attr_any_nomotion_sel,
#endif
#if defined(BMA420) || defined(BMA456)
	&driver_attr_tap_type,
#endif
#if defined(BMA420) || defined(BMA456)
	&driver_attr_orientation_ud_en,
	&driver_attr_orientation_mode,
	&driver_attr_orientation_theta,
	&driver_attr_orientation_hysteresis,
	&driver_attr_orientation_blocking,
#endif
#if defined(BMA420) || defined(BMA456)
	&driver_attr_flat_theta,
	&driver_attr_flat_hysteresis,
	&driver_attr_flat_holdtime,
#endif
#if defined(BMA420) || defined(BMA456)
	&driver_attr_highg_threshold,
	&driver_attr_highg_hysteresis,
	&driver_attr_highg_duration,
#endif
#if defined(BMA420) || defined(BMA456)
	&driver_attr_lowg_threshold,
	&driver_attr_lowg_hysteresis,
	&driver_attr_lowg_duration,
#endif
//<--[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20170116 BEGIN --
	&driver_attr_do_cali,
	&driver_attr_cali_test,	
//-->[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20170116 END --
};

static int bma4xy_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(bma4xy_attr_list) / sizeof(bma4xy_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;
	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, bma4xy_attr_list[idx]);
		if (0 != err) {
			GSE_ERR("driver_create_file (%s) = %d\n", bma4xy_attr_list[idx]->attr.name,
				err);
			break;
		}
	}
	return err;
}

static int bma4xy_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(bma4xy_attr_list) / sizeof(bma4xy_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;
	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, bma4xy_attr_list[idx]);
	return err;
}

/*----------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int gsensor_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/*----------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
static int gsensor_enable_nodata(int en)
{
	int err = 0;
	if (((en == 0) && (sensor_power == false)) || ((en == 1) && (sensor_power == true))) {
		enable_status = sensor_power;
		GSE_LOG("Gsensor device have updated!\n");
	} else {
		enable_status = !sensor_power;
		if (atomic_read(&obj_i2c_data->suspend) == 0) {
			err = BMA4XY_SetPowerMode(obj_i2c_data->client, enable_status);
			GSE_LOG("Gsensor not in suspend BMA4XY_SetPowerMode!, enable_status = %d\n",
				enable_status);
		} else {
			GSE_LOG("Gsensor in suspend and can not enable or disable!enable_status = %d\n", enable_status);
		}
	}
	if (err) {
		GSE_ERR("gsensor_enable_nodata fail!\n");
		return -1;
	}
//<--[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20170116 BEGIN --
	if(first_read_cali == true){
		read_cali_value();
	}
//-->[SM31][Sensors][JasonHsing] G-sensor cailbration rule for PCBA 20170116 END --
	GSE_LOG("gsensor_enable_nodata OK!\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int gsensor_set_delay(u64 ns)
{
	int err = 0;
	int value;
	int sample_delay;
	value = (int)ns / 1000 / 1000;

	if (value <= 5)
		sample_delay = BMA4XY_ACCEL_OUTPUT_DATA_RATE_200HZ;
	else if (value <= 10)
		sample_delay = BMA4XY_ACCEL_OUTPUT_DATA_RATE_100HZ;
	else
		sample_delay = BMA4XY_ACCEL_OUTPUT_DATA_RATE_100HZ;

	err = BMA4XY_SetBWRate(obj_i2c_data->client, sample_delay);
	if (err) {
		GSE_ERR("Set delay parameter error!\n");
		return -1;
	}

	if (value >= 50) {
		atomic_set(&obj_i2c_data->filter, 0);
	} else {
	}
	GSE_LOG("gsensor_set_delay (%d)\n", value);
	return 0;
}

static int gsensor_get_data(int *x, int *y, int *z, int *status)
{
	char buff[BMA4XY_BUFSIZE];
	int ret;
	mutex_lock(&gsensor_mutex);
	BMA4XY_ReadSensorData(obj_i2c_data->client, buff, BMA4XY_BUFSIZE);
	mutex_unlock(&gsensor_mutex);
	ret = sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	return 0;
}

static struct acc_init_info bma4xy_init_info = {
	.name = BMA4XY_DEV_NAME,
	.init = gsensor_local_init,
	.uninit = gsensor_remove,
};

static int bma4xy_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct i2c_client *new_client;
	struct bma4xy_client_data *client_data = NULL;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};
	GSE_FUN();
	client_data = kzalloc(sizeof(struct bma4xy_client_data), GFP_KERNEL);
	if (NULL == client_data) {
		GSE_ERR("no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}
	memset(client_data, 0, sizeof(struct bma4xy_client_data));
	/* h/w init */
	client_data->device.bus_read = bma4xy_i2c_read_wrapper;
	client_data->device.burst_read = bma4xy_i2c_read_wrapper;
	client_data->device.bus_write = bma4xy_i2c_write_wrapper;
	client_data->device.burst_write = bma4xy_i2c_write_wrapper;
	client_data->device.delay_msec = bma4xy_i2c_delay;
	client_data->hw = hw;
	err = hwmsen_get_convert(client_data->hw->direction, &client_data->cvt);
	if (0 != err) {
		GSE_ERR("invalid direction: %d\n", client_data->hw->direction);
		goto exit_err_clean;
	}
	obj_i2c_data = client_data;
	client_data->client = client;
	client_data->client->addr = 0x18;	//forced change i2c slave addr
	printk("[Bma422] kernel: auto probe address 0x18\n");
	new_client = client_data->client;
//<--[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 BEGIN --
	bma4xy_i2c_dma_init(new_client);
//-->[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 END --
	i2c_set_clientdata(new_client, client_data);
	atomic_set(&client_data->trace, 0);
	atomic_set(&client_data->suspend, 0);
	bma4xy_i2c_client = new_client;
	/* check chip id */
	err = bma4xy_check_chip_id(client_data);
	if (!err) {
		GSE_LOG("Bosch Sensortec Device %s detected", SENSOR_NAME);
	} else {
		GSE_ERR("Bosch Sensortec Device not found, chip id mismatch");
		goto exit_err_clean;
	}
	err = BMA4XY_CALL_API(init)(&client_data->device);
	if (err)
		GSE_ERR("init failed\n");
	err = BMA4XY_CALL_API(set_command_register)(0xB6);
	if (err)
		GSE_LOG("reset chip failed\n");
	bma4xy_i2c_delay(10);

#if defined(BMA4XY_ENABLE_INT1) || defined(BMA4XY_ENABLE_INT2)
	err = bma4xy_request_irq(client_data);
	if (err < 0) {
		GSE_ERR("Request irq failed");
	}
#endif
	wake_lock_init(&client_data->wakelock, WAKE_LOCK_SUSPEND, "bma4xy");
	err = bma4xy_init_client(new_client, 1);
	if (err)
		GSE_ERR("bma4xy_device init cilent fail time\n");

	err = misc_register(&bma4xy_device);
	if (err) {
		GSE_ERR("bma4xy_device register failed\n");
		goto exit_err_clean;
	}
	err = bma4xy_create_attr(&bma4xy_init_info.platform_diver_addr->driver);
	if (err) {
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_err_clean;
	}
	ctl.open_report_data = gsensor_open_report_data;
	ctl.enable_nodata = gsensor_enable_nodata;
	ctl.set_delay = gsensor_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = false;
	err = acc_register_control_path(&ctl);
	if (err) {
		GSE_ERR("register acc control path err\n");
		goto exit_err_clean;
	}
	data.get_data = gsensor_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if (err) {
		GSE_ERR("register acc data path err\n");
		goto exit_err_clean;
	}
	err = batch_register_support_info(ID_ACCELEROMETER,
					ctl.is_support_batch, 102, 0);
	if (err) {
		GSE_ERR("register gsensor batch support err = %d\n", err);
		goto exit_err_clean;
	}

	gsensor_init_flag = 0;
	
	GSE_LOG("%s: OK\n", __func__);
	return 0;
exit_err_clean:
	if (err) {
		bma4xy_i2c_client = NULL;
		obj_i2c_data = NULL;
		if (client_data != NULL)
			kfree(client_data);
		return err;
	}
	return err;
}

static int bma4xy_i2c_remove(struct i2c_client *client)
{
	int err = 0;
	err = bma4xy_delete_attr(&bma4xy_init_info.platform_diver_addr->driver);
	if (err != 0)
		GSE_ERR("bma150_delete_attr fail: %d\n", err);

	err = misc_deregister(&bma4xy_device);
	if (0 != err)
		GSE_ERR("misc_deregister fail: %d\n", err);

	bma4xy_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;

}
#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor",},
	{ }
};
MODULE_DEVICE_TABLE(i2c, accel_of_match);
#endif


static struct i2c_driver bma4xy_i2c_driver = {
	.driver = {
		.name = SENSOR_NAME,
		.of_match_table = accel_of_match,
	},
	.probe = bma4xy_i2c_probe,
	.remove = bma4xy_i2c_remove,
	.suspend = bma4xy_i2c_suspend,
	.resume = bma4xy_i2c_resume,
	.id_table=bma4xy_i2c_id,
};

static int gsensor_local_init(void)
{
	GSE_FUN();
	if (i2c_add_driver(&bma4xy_i2c_driver)) {
		GSE_ERR("add driver error\n");
		return -1;
	}
	if (-1 == gsensor_init_flag)
		return -1;
	return 0;
}

static int gsensor_remove(void)
{
	GSE_FUN();
	BMA4XY_power(hw, 0);
	i2c_del_driver(&bma4xy_i2c_driver);
	return 0;
}

static int __init BMA4xy_init(void)
{
	//const char *name = "mediatek,bma4xy";
	const char *name = "mediatek,arima_gsensor";	//same compatible in dtsi
	
	GSE_FUN();
	hw = get_accel_dts_func(name, hw);
	if (!hw)
		GSE_ERR("get dts info fail\n");
	GSE_LOG("%s: i2c_number=%d\n", __func__, hw->i2c_num);
	acc_driver_add(&bma4xy_init_info);
	return 0;
}

static void __exit BMA4xy_exit(void)
{
	GSE_FUN();
}
module_init(BMA4xy_init);
module_exit(BMA4xy_exit);
MODULE_AUTHOR("contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA4XY SENSOR DRIVER");
MODULE_LICENSE("GPL v2");

