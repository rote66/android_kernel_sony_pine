/* BOSCH STEP COUNTER Sensor Driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <cust_acc.h>
#include <step_counter.h>
#include <bma4xy_driver.h>
#include <accel.h>

#define	STC_DEV_NAME	"bma4xy_step_counter"
#define	ENABLE						(1)
#define	DISABLE						(0)

#define COMPATIABLE_NAME "mediatek,m_step_c_pl"

/* power mode */
enum STC_POWERMODE_ENUM {
	STC_SUSPEND_MODE = 0x0,
	STC_NORMAL_MODE,
	STC_UNDEFINED_POWERMODE = 0xff
};

struct step_c_i2c_data {
	atomic_t trace;
	atomic_t suspend;
	atomic_t filter;
	uint8_t power_mode;
	uint8_t sigmotion_enable;
	uint8_t stepdet_enable;
	uint8_t stepcounter_enable;
	
};

static struct step_c_i2c_data *obj_i2c_step;
//static int step_c_set_powermode(int power_mode);

static int step_c_get_chip_type( void )
{
	STEP_C_LOG("[%s]chip id = %#x\n", __func__, 18);
	return 0;
}

int step_counter_enable(u8 enable)
{
	if (BMA4XY_CALL_API(step_counter_enable)(enable) < 0) {
		STEP_C_ERR("set BMA4XY_STEP_COUNTER error");
		return -EINVAL;
	}
	return 0;
}
/*
static int step_c_set_powermode(int power_mode)
{
	int err = 0;
	struct step_c_i2c_data *obj = obj_i2c_step;

	if (power_mode == 0 &&
		(obj->sigmotion_enable == 0) &&
		(obj->stepdet_enable == 0) &&
		(obj->stepcounter_enable == 0)) {
			err = BMA4XY_CALL_API(set_accel_enable)(0);
			STEP_C_ERR("acc_op_mode %d", power_mode);
		}
	else if (power_mode == 1) {
		err = BMA4XY_CALL_API(set_accel_enable)(1);
		STEP_C_ERR("acc_op_mode %d", power_mode);
	}
	if (err) {
		STEP_C_ERR("failed");
		return err;
	}
	return err;
}
*/
static int step_c_set_datarate(int datarate)
{
	return 0;
}

static int step_c_init_client(void)
{
	int err = 0;
	STEP_C_FUN();
	err = step_c_get_chip_type();
	if (err < 0) {
		STEP_C_ERR("get chip type failed, err = %d\n", err);
		return err;
	}
	
	err = step_c_set_datarate(0);
	if (err < 0) {
		STEP_C_ERR("set data_rate failed, err = %d\n", err);
		return err;
	}

	err = step_counter_enable(DISABLE);
	if (err < 0) {
		STEP_C_ERR("set step counter enable failed, err = %d\n", err);
		return err;
	}
	return 0;
}

static int step_c_open(struct inode *inode, struct file *file)
{
	file->private_data = obj_i2c_step;
	if (file->private_data == NULL) {
		STEP_C_ERR("file->private_data == NULL.\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int step_c_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long stc_c_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	return 0;
}

static const struct file_operations step_c_fops = {
	.owner = THIS_MODULE,
	.open = step_c_open,
	.release = step_c_release,
	.unlocked_ioctl = stc_c_unlocked_ioctl,
};

static struct miscdevice step_c_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma4xy_step_counter",
	.fops = &step_c_fops,
};

static int bma4xy_step_c_open_report_data(int open)
{
	return 0;
}

static int bma4xy_step_c_set_delay(u64 delay)
{
	return 0;
}
static int bma4xy_setp_d_set_selay(u64 delay)
{
	return 0;
}
static int bma4xy_step_c_enable_nodata(int en)
{
	int err = 0;
	uint8_t enable = 0;
	enable = (uint8_t)(en);

	if (BMA4XY_CALL_API(step_counter_enable)(en) < 0) {
		STEP_C_ERR("set BMA4XY_STEP_COUNTER error");
		return -EINVAL;
	}
	if (en == 1)
		err = BMA4XY_CALL_API(set_accel_enable)(1);
	if (err)
		STEP_C_ERR("set acc_op_mode failed");
	if ((en == 0) && (obj_i2c_data->sigmotion_enable== 0)&&
		(obj_i2c_data->stepdet_enable== 0)&&(obj_i2c_data->acc_pm== 0))
		err = BMA4XY_CALL_API(set_accel_enable)(0);
	if (err)
		STEP_C_ERR("set acc_op_mode failed");
	obj_i2c_data->stepcounter_enable = en;
	return err;
}

static int bma4xy_step_c_enable_significant(int en)
{
	int err = 0;
	uint8_t enable = 0;
	enable = (uint8_t)(en);
	if (BMA4XY_CALL_API(significant_motion_enable)(enable) < 0) {
		STEP_C_ERR("set BMA4XY_SIG_MOTION error");
		return -EINVAL;
	}
	if (en == 1)
		err = BMA4XY_CALL_API(set_accel_enable)(1);
	if (err)
		STEP_C_ERR("set acc_op_mode failed");
	if ((en == 0) && (obj_i2c_data->stepcounter_enable == 0) &&
		(obj_i2c_data->stepdet_enable== 0)&&(obj_i2c_data->acc_pm== 0))
		err = BMA4XY_CALL_API(set_accel_enable)(0);
	if (err)
		STEP_C_ERR("set acc_op_mode failed");
	obj_i2c_data->sigmotion_enable = en;
	return err;
}

static int bma4xy_step_c_enable_step_detect(int en)
{
	int err = 0;
	if (BMA4XY_CALL_API(step_detector_enable)(en) < 0) {
		STEP_C_ERR("set BMA4XY_STEP_COUNTER error");
		return -EINVAL;
	}
	if (en == 1)
		err = BMA4XY_CALL_API(set_accel_enable)(1);
	if (err)
		STEP_C_ERR("set acc_op_mode failed");
	if ((en == 0) && (obj_i2c_data->stepcounter_enable == 0) && (obj_i2c_data->sigmotion_enable== 0)&&(
		obj_i2c_data->acc_pm== 0))
		err = BMA4XY_CALL_API(set_accel_enable)(0);
	if (err)
		STEP_C_ERR("set acc_op_mode failed");
	obj_i2c_data->stepdet_enable= en;
	return err;
}

static int bma4xy_step_c_get_data(u32 *value, int *status)
{
	int err = 0;
	uint32_t step_counter_val = 0;
	err = BMA4XY_CALL_API(step_counter_output)(&step_counter_val);
	if (err) {
		STEP_C_ERR("read failed");
		return err;
	}
	*value = step_counter_val;
	*status = 1;
	STEP_C_LOG("step_c_get_data = %d\n", (int)(*value));
	return err;
}

static int bma4xy_stc_get_data_significant(u32 *value, int *status)
{
	return 0;
}

static int bma4xy_stc_get_data_step_d(u32 *value, int *status)
{
	return 0;
}

static int bma4xy_step_c_probe(void)
{
	struct step_c_i2c_data *obj;
	struct step_c_control_path ctl = {0};
	struct step_c_data_path data = {0};
	int err = 0;
//<--[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 BEGIN --
	if (obj_i2c_data == NULL){
				err = -1;
				return err;
	}
//-->[SM31][Sensors][JasonHsing] Significant motion for Full doze mode 20161020 END --
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	err = step_c_init_client();
	if (err)
		goto exit_init_client_failed;

	err = misc_register(&step_c_device);
	if (err) {
		STEP_C_ERR("misc device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;
	}

	ctl.open_report_data= bma4xy_step_c_open_report_data;
	ctl.enable_nodata = bma4xy_step_c_enable_nodata;
	ctl.enable_step_detect = bma4xy_step_c_enable_step_detect;
	ctl.enable_significant = bma4xy_step_c_enable_significant;
	ctl.step_c_set_delay = bma4xy_step_c_set_delay;
	ctl.step_d_set_delay = bma4xy_setp_d_set_selay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = false;
	ctl.is_report_input_direct = false;
	err =  step_c_register_control_path(&ctl);
	if(err) {
		STEP_C_ERR("step_c_register_control_path fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	data.get_data = bma4xy_step_c_get_data;
	data.vender_div = 1000;
	data.get_data_significant = bma4xy_stc_get_data_significant;
	data.get_data_step_d = bma4xy_stc_get_data_step_d;
	err = step_c_register_data_path(&data);
	if(err) {
		STEP_C_ERR("step_c_register_data_path fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	STEP_C_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&step_c_device);
exit_misc_device_register_failed:
exit_init_client_failed:
	kfree(obj);
exit:
	STEP_C_ERR("err = %d\n", err);
	return err;
}

static int bma4xy_stc_remove(void)
{
	STEP_C_FUN();
	return 0;
}
static int bma4xy_stc_local_init(void)
{
/*
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info info;
	printk(KERN_ERR "111111111111111111111111111bma4xy_stc_local_init");
	adapter = i2c_get_adapter(1);
	if(adapter == NULL){
	printk(KERN_ERR"bma4xy_stc_local_init error");
	}
	memset(&info,0,sizeof(struct i2c_board_info));
	info.addr = 0x19;
	strlcpy(info.type,"bma4xy_step_counter",I2C_NAME_SIZE);
	client = i2c_new_device(adapter,&info);
	strlcpy(client->name,"bma4xy_step_counter",I2C_NAME_SIZE);
*/
	STEP_C_FUN("bma4xy_stc_local_init.\n");
	if (bma4xy_step_c_probe()) {
		STEP_C_ERR("failed to register bma4xy step_c driver\n");
		return -ENODEV;
	}
	return 0;
}

static struct step_c_init_info bma4xy_stc_init_info = {
	.name = "bma4xy_step_counter",
	.init = bma4xy_stc_local_init,
	.uninit = bma4xy_stc_remove,
};

static int __init bma4xy_stec_init(void)
{
	STEP_C_FUN();
	step_c_driver_add(&bma4xy_stc_init_info);
	return 0;
}

static void __exit bma4xy_stc_exit(void)
{
	STEP_C_FUN();
}
module_init(bma4xy_stec_init);
module_exit(bma4xy_stc_exit);
MODULE_AUTHOR("contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA4XY SENSOR DRIVER");
MODULE_LICENSE("GPL v2");
