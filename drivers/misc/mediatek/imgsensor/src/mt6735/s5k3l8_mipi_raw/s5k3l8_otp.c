/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/
/*
 * Driver for CAM_CAL
 *
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"

typedef unsigned int kal_uint32;
typedef unsigned short kal_uint16;
typedef unsigned char kal_uint8;

#include "cam_cal.h"
#include "cam_cal_define.h"
#include "s5k3l8_otp.h"

/* #include <asm/system.h>  // for SMP */
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif


#define PFX "S5K3L8_OTP_FMT"

/* #define CAM_CALGETDLT_DEBUG */
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG


#define CAM_CALINF(fmt, arg...)    pr_debug("[%s] " fmt, __func__, ##arg)
#define CAM_CALDB(fmt, arg...)     pr_debug("[%s] " fmt, __func__, ##arg)
#define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __func__, ##arg)
#else
#define CAM_CALINF(x, ...)
#define CAM_CALDB(x, ...)
#define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __func__, ##arg)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); /* for SMP */


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)


/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 /* seanlin111208 */
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
/* static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO
(CAM_CAL_DRVNAME, S5K3L8_OTP_DEVICE_ID>>1)}; */

/* static struct i2c_client * g_pstI2Cclient = NULL; */


static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER, 0);
static struct cdev *g_pCAM_CAL_CharDrv;


static struct class *CAM_CAL_class;
static atomic_t g_CAM_CALatomic;

#define MAX_OTP_SIZE 64
static int s5k3l8_otp_read;
#if 0
typedef struct {
	u8     flag;
	u32   CaliVer;/* 0xff000b01 */
	u8     Module_id;/* 0x57 */
	u8     Lens_id;/* 0x20 */
	u8     Vcm_id;/*0x0A*/
	u8     Driver_id;/*0x02*/
	u8     AwbRGrMsb;
	u8     AwbBGrMsb;
	u8     AwbGbGrMsb;
	u8     AwbLsb;
	u8     UnitAwbR;
	u8     UnitAwbGr;
	u8     UnitAwbGb;
	u8     UnitAwbB;
	u8     GoldenAwbR;
	u8     GoldenAwbGr;
	u8     GoldenAwbGb;
	u8     GoldenAwbB;
	u8     AfMacroLsb;
	u8     AfMacroMsb;
	u8     AfInfiniteLsb;
	u8     AfInfiniteMsb;
	u8     flag2;
	u32   CaliVer2;/* 0xff000b01 */
	u8     Module_id2;/* 0x57 */
	u8     Lens_id2;/* 0x20 */
	u8     Vcm_id2;/*0x0A*/
	u8     Driver_id2;/*0x02*/
	u8     AwbRGrMsb2;
	u8     AwbBGrMsb2;
	u8     AwbGbGrMsb2;
	u8     AwbLsb2;
	u8     UnitAwbR2;
	u8     UnitAwbGr2;
	u8     UnitAwbGb2;
	u8     UnitAwbB2;
	u8     GoldenAwbR2;
	u8     GoldenAwbGr2;
	u8     GoldenAwbGb2;
	u8     GoldenAwbB2;
	u8     AfMacroLsb2;
	u8     AfMacroMsb2;
	u8     AfInfiniteLsb2;
	u8     AfInfiniteMsb2;
	u32   Reserved1;
	u32   Reserved2;
	u32   Reserved3;
	u16   Reserved4;
} S5K3L8_OTP_MTK_TYPE;

typedef union {
	u8 Data[MAX_OTP_SIZE];
	S5K3L8_OTP_MTK_TYPE       MtkOtpData;
} S5K3L8_OTP_DATA;
#endif

typedef struct {
	u8     flag;
	u32   CaliVer;/* 0xff000b01 */
	u8     Module_id;/* 0x57 */
	u8     Lens_id;/* 0x20 */
	u8     Vcm_id;/*0x0A*/
	u8     Driver_id;/*0x02*/
	u8     AwbRGrMsb;
	u8     AwbBGrMsb;
	u8     AwbGbGrMsb;
	u8     AwbLsb;
	u8     UnitAwbR;
	u8     UnitAwbGr;
	u8     UnitAwbGb;
	u8     UnitAwbB;
	u8     GoldenAwbR;
	u8     GoldenAwbGr;
	u8     GoldenAwbGb;
	u8     GoldenAwbB;
	u8     AfMacroLsb;
	u8     AfMacroMsb;
	u8     AfInfiniteLsb;
	u8     AfInfiniteMsb;
} S5K3L8_OTP_MTK_TYPE;

static u8 OTP_Data[MAX_OTP_SIZE];
static S5K3L8_OTP_MTK_TYPE		s5k3l8_otp_data;

#if 0
void otp_clear_flag(void)
{
	spin_lock(&g_CAM_CALLock);
	_otp_read = 0;
	spin_unlock(&g_CAM_CALLock);
}
#endif

//S5K3L8_OTP_DATA s5k3l8_otp_data = {{0} };
/*************************************************************************************************
* Function    :  start_read_otp
* Description :  before read otp , set the reading block setting
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  0, reading block setting err
		 1, reading block setting ok
**************************************************************************************************/

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1,S5K3L8_OTP_DEVICE_ID);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	iWriteRegI2C(pusendcmd, 4,S5K3L8_OTP_DEVICE_ID);
}

bool start_read_otp(void)
{
	CAM_CALERR("s5k3l8 start_read_otp\n");

	write_cmos_sensor(0x0100, 0x0100);
	write_cmos_sensor(0x0A02, 0x0100);   /* PAGE */
	write_cmos_sensor(0x0A00, 0x0100);

	Sleep(10);

	return 0;
}
int  stop_read_otp(void)
{
		CAM_CALERR("s5k3l8 stop_read_otp\n");
		write_cmos_sensor(0x0A00, 0x0000);
		
		return 0;
}


int read_s5k3l8_otp(u8 page, u16 offset, u8 *data)
{

	*data = read_cmos_sensor(offset);
	 CAM_CALERR("OTP read page 0x%x offset 0x%x  data 0x%x\n", page,offset,*data);
	return 1;
}

int read_s5k3l8_otp_size(u8 page, u16 offset, u8 *data, int size)
{
	int i = 0;

	for (i = 0; i < size; i++) {
		if (!read_s5k3l8_otp(page, offset + i, data + i))
			return -1;
	}
	CAM_CALERR("OTP read Done!\n");
	return 0;
}


int read_s5k3l8_otp_mtk_fmt(void)
{
	CAM_CALERR("OTP readed =%d\n", s5k3l8_otp_read);

	if (1 == s5k3l8_otp_read) {
		CAM_CALDB("OTP readed ! skip\n");
//[SM31][Camera] Modify for 3rd source Person Liu 20170317 S
	return (s5k3l8_otp_data.Module_id + s5k3l8_otp_data.Vcm_id);
//[SM31][Camera] Modify for 3rd source Person Liu 20170317 E
	}
	spin_lock(&g_CAM_CALLock);
	s5k3l8_otp_read = 1;
	spin_unlock(&g_CAM_CALLock);

	start_read_otp();

	read_s5k3l8_otp_size(0x01, 0x0A04, &OTP_Data[0x00], 64);

	if(OTP_Data[25] == 0x01){
		s5k3l8_otp_data.flag = OTP_Data[25];
		s5k3l8_otp_data.CaliVer =OTP_Data[27];
		s5k3l8_otp_data.Module_id= OTP_Data[30];
		s5k3l8_otp_data.Lens_id= OTP_Data[31];
		s5k3l8_otp_data.Vcm_id= OTP_Data[32];
		CAM_CALERR("S5k3l8 Use OTP Group 2\n");
	}
	else{
		s5k3l8_otp_data.flag = OTP_Data[0];
		s5k3l8_otp_data.CaliVer =OTP_Data[2];
		s5k3l8_otp_data.Module_id= OTP_Data[5];
		s5k3l8_otp_data.Lens_id= OTP_Data[6];
		s5k3l8_otp_data.Vcm_id= OTP_Data[7];
		CAM_CALERR("S5k3l8 Use OTP Group 1\n");
	}

	CAM_CALERR("Module_id = 0x%x\n",s5k3l8_otp_data.Module_id);
	CAM_CALERR("Lens_id = 0x%x\n",s5k3l8_otp_data.Lens_id);
	CAM_CALERR("Vcm_id = 0x%x\n",s5k3l8_otp_data.Vcm_id);

	stop_read_otp();
//[SM31][Camera] Modify for 3rd source Person Liu 20170317 S
	return (s5k3l8_otp_data.Module_id + s5k3l8_otp_data.Vcm_id);
//[SM31][Camera] Modify for 3rd source Person Liu 20170317 E
}


#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
	stCAM_CAL_INFO_STRUCT __user *data)
{
//	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data->u4Offset);
	err |= put_user(i, &data32->u4Offset);
	err |= get_user(i, &data->u4Length);
	err |= put_user(i, &data32->u4Length);
	/* Assume pointer is not change */
#if 0
	err |= get_user(p, &data->pu1Params);
	err |= put_user(p, &data32->pu1Params);
#endif
	return err;
}
static int compat_get_cal_info_struct(
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
	stCAM_CAL_INFO_STRUCT __user *data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data32->u4Offset);
	err |= put_user(i, &data->u4Offset);
	err |= get_user(i, &data32->u4Length);
	err |= put_user(i, &data->u4Length);
	err |= get_user(p, &data32->pu1Params);
	err |= put_user(compat_ptr(p), &data->pu1Params);

	return err;
}

static long s5k3l8otp_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;

	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
	stCAM_CAL_INFO_STRUCT __user *data;
	int err;

	CAM_CALERR("[CAMERA SENSOR] S5K3L8_OTP_DEVICE_ID,%p %p %x ioc size %d\n",
	filp->f_op , filp->f_op->unlocked_ioctl, cmd, _IOC_SIZE(cmd));

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {

	case COMPAT_CAM_CALIOC_G_READ: {
		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;
		err = compat_get_cal_info_struct(data32, data);
		if (err)
			return err;
		ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ, (unsigned long)data);
		err = compat_put_cal_info_struct(data32, data);

		if (err != 0)
			CAM_CALERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
		return ret;
	}
	default:
		return -ENOIOCTLCMD;
	}
}


#endif

#if 0
static int selective_read_region(u32 offset, BYTE *data, u16 i2c_id, u32 size)
{
	memcpy((void *)data, (void *)&OTP_Data[offset], size);
	CAM_CALERR("selective_read_region offset =%x size %d data read = %d\n", offset, size, *data);
	return size;
}
#endif
//Burst Read Data  iReadData(0x00,932,OTPData);
 int iReadData_S5K3L8(kal_uint16 ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{

// u8 readbuff;
 int i = 0;

 if(!s5k3l8_otp_read){
	read_s5k3l8_otp_mtk_fmt();
}

for(i=0;i<ui4_length;i++){

	pinputdata[i]=OTP_Data[ui4_offset+i];

	CAM_CALERR("pinputdata[%d] = 0x%x\n",i,pinputdata[i]);

}
   return 0;
}

/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode *a_pstInode,
			 struct file *a_pstFile,
			 unsigned int a_u4Command,
			 unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
	struct file *file,
	unsigned int a_u4Command,
	unsigned long a_u4Param
)
#endif
{
	int i4RetValue = 0;
	u8 *pBuff = NULL;
	u8 *pu1Params = NULL;
	stCAM_CAL_INFO_STRUCT *ptempbuf;
#ifdef CAM_CALGETDLT_DEBUG
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
#endif

	/*if (_IOC_NONE == _IOC_DIR(a_u4Command)) { LukeHu--150330=For Kernel Coding Style
	} else {*/
	if (_IOC_NONE != _IOC_DIR(a_u4Command)) {/*LukeHu++150330=For Kernel Coding Style*/
		pBuff = /*LukeHu++150330=For Kernel Coding Style*/kmalloc(sizeof(stCAM_CAL_INFO_STRUCT), GFP_KERNEL);

		if (NULL == pBuff) {
			CAM_CALERR(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}

		if (_IOC_WRITE & _IOC_DIR(a_u4Command)) {
			if (copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT))) {
				/* get input structure address */
				kfree(pBuff);
				CAM_CALERR("ioctl copy from user failed\n");
				return -EFAULT;
			}
		}
	}

	ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
	pu1Params = kmalloc(ptempbuf->u4Length, GFP_KERNEL);
	if (NULL == pu1Params) {
		kfree(pBuff);
		CAM_CALERR("ioctl allocate mem failed\n");
		return -ENOMEM;
	}


	if (copy_from_user((u8 *)pu1Params , (u8 *)ptempbuf->pu1Params, ptempbuf->u4Length)) {
		kfree(pBuff);
		kfree(pu1Params);
		CAM_CALERR(" ioctl copy from user failed\n");
		return -EFAULT;
	}

	switch (a_u4Command) {
	case CAM_CALIOC_S_WRITE:
/*CAM_CALDB("Write CMD\n");*/
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
		i4RetValue = 0;/* iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params); */
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
		CAM_CALDB("Write data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif
		break;
	case CAM_CALIOC_G_READ:
/*CAM_CALDB("[CAM_CAL] Read CMD\n");*/
		CAM_CALERR("Enter CAM_CALIOC_G_READ\n");
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
//		i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params, S5K3L8_OTP_DEVICE_ID,
//		ptempbuf->u4Length);
		iReadData_S5K3L8((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
		CAM_CALERR("Read data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif

		break;
	default:
		CAM_CALINF("[CAM_CAL] No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	if (_IOC_READ & _IOC_DIR(a_u4Command)) {
		/* copy data to user space buffer, keep other input paremeter unchange. */
		if (copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length)) {
			kfree(pBuff);
			kfree(pu1Params);
			CAM_CALERR("[CAM_CAL] ioctl copy to user failed\n");
			return -EFAULT;
		}
	}

	kfree(pBuff);
	kfree(pu1Params);
	return i4RetValue;
}


static u32 g_u4Opened;
/* #define */
/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
static int CAM_CAL_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	CAM_CALDB("CAM_CAL_Open\n");
	spin_lock(&g_CAM_CALLock);
	if (g_u4Opened) {
		spin_unlock(&g_CAM_CALLock);
		CAM_CALERR("Opened, return -EBUSY\n");
		return -EBUSY;
	} /*else {*//*LukeHu++150720=For check fo*/
	if (!g_u4Opened) {
		g_u4Opened = 1;
		atomic_set(&g_CAM_CALatomic, 0);
	}
	spin_unlock(&g_CAM_CALLock);
	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int CAM_CAL_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	spin_lock(&g_CAM_CALLock);

	g_u4Opened = 0;

	atomic_set(&g_CAM_CALatomic, 0);

	spin_unlock(&g_CAM_CALLock);

	return 0;
}

static const struct file_operations g_stCAM_CAL_fops = {
	.owner = THIS_MODULE,
	.open = CAM_CAL_Open,
	.release = CAM_CAL_Release,
	/* .ioctl = CAM_CAL_Ioctl */
#ifdef CONFIG_COMPAT
	.compat_ioctl = s5k3l8otp_Ioctl_Compat,
#endif
	.unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
/* #define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1 */

static inline int RegisterCAM_CALCharDrv(void)
{
	struct device *CAM_CAL_device = NULL;

	CAM_CALDB("RegisterCAM_CALCharDrv\n");
#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
	if (alloc_chrdev_region(&g_CAM_CALdevno, 0, 1, CAM_CAL_DRVNAME)) {
		CAM_CALERR(" Allocate device no failed\n");

		return -EAGAIN;
	}
#else
	if (register_chrdev_region(g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME)) {
		CAM_CALERR(" Register device no failed\n");

		return -EAGAIN;
	}
#endif

	/* Allocate driver */
	g_pCAM_CAL_CharDrv = cdev_alloc();

	if (NULL == g_pCAM_CAL_CharDrv) {
		unregister_chrdev_region(g_CAM_CALdevno, 1);

		CAM_CALERR(" Allocate mem for kobject failed\n");

		return -ENOMEM;
	}

	/* Attatch file operation. */
	cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

	g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1)) {
		CAM_CALERR(" Attatch file operation failed\n");

		unregister_chrdev_region(g_CAM_CALdevno, 1);

		return -EAGAIN;
	}

	CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
	if (IS_ERR(CAM_CAL_class)) {
		int ret = PTR_ERR(CAM_CAL_class);

		CAM_CALERR("Unable to create class, err = %d\n", ret);
		return ret;
	}
	CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

	return 0;
}

static inline void UnregisterCAM_CALCharDrv(void)
{
	/* Release char driver */
	cdev_del(g_pCAM_CAL_CharDrv);

	unregister_chrdev_region(g_CAM_CALdevno, 1);

	device_destroy(CAM_CAL_class, g_CAM_CALdevno);
	class_destroy(CAM_CAL_class);
}

static int CAM_CAL_probe(struct platform_device *pdev)
{

	return 0;/* i2c_add_driver(&CAM_CAL_i2c_driver); */
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
	/* i2c_del_driver(&CAM_CAL_i2c_driver); */
	return 0;
}

/* platform structure */
static struct platform_driver g_stCAM_CAL_Driver = {
	.probe              = CAM_CAL_probe,
	.remove     = CAM_CAL_remove,
	.driver             = {
		.name   = CAM_CAL_DRVNAME,
		.owner  = THIS_MODULE,
	}
};


static struct platform_device g_stCAM_CAL_Device = {
	.name = CAM_CAL_DRVNAME,
	.id = 0,
	.dev = {
	}
};

static int __init CAM_CAL_init(void)
{
	int i4RetValue = 0;

	CAM_CALDB("CAM_CAL_i2C_init\n");
	/* Register char driver */
	i4RetValue = RegisterCAM_CALCharDrv();
	if (i4RetValue) {
		CAM_CALDB(" register char device failed!\n");
		return i4RetValue;
	}
	CAM_CALDB(" Attached!!\n");

	/* i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1); */
	if (platform_driver_register(&g_stCAM_CAL_Driver)) {
		CAM_CALERR("failed to register 5k3l8otp driver\n");
		return -ENODEV;
	}

	if (platform_device_register(&g_stCAM_CAL_Device)) {
		CAM_CALERR("failed to register 5k3l8otp driver, 2nd time\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit CAM_CAL_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_init);
module_exit(CAM_CAL_exit);

