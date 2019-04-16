#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include "leds_sw.h"
#include "ktd2037.h"

#define CONFIG_MSMB_KTD2037_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_KTD2037_DEBUG
#define CDBG(fmt, args...) pr_err("[%s:%d]" fmt "\n", __func__, __LINE__, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define KTD2037_REG_EN_RST	0x00
#define KTD2037_REG_FLASH_PERIOD	0x01
#define KTD2037_REG_PWM1_TIMER	0x02
#define KTD2037_REG_PWM2_TIMER	0x03
#define KTD2037_REG_LED_EN	0x04
#define KTD2037_REG_TRISE_TFALL	0x05
#define KTD2037_REG_LED_BASE	0x06
#define KTD2037_REG_AUTOBLINQ	0x09

#define KTD2037_RED_MAX_CURRENT_REG	15
#define KTD2037_GRN_MAX_CURRENT_REG	15
#define KTD2037_BLU_MAX_CURRENT_REG	23
#define KTD2037_MAX_ON_MS	10000
#define KTD2037_MAX_OFF_MS	6400
#define KTD2037_MIN_PERIOD_MS	128

union ktd2037_reset_ctrl_reg {
	struct {
		u8 tctrl_reset:3;
		u8 enable:2;
		u8 time_scaling:2;
		u8 reserved:1;
	} st;
	u8 value;
};

union ktd2037_period_reg {
	struct {
		u8 period:7;
		u8 ramp:1;
	} st;
	u8 value;
};

union ktd2037_ch_ctrl_reg {
	struct {
		u8 led1_en_timer:2;
		u8 led2_en_timer:2;
		u8 led3_en_timer:2;
		u8 device_en:1;
		u8 not_used:1;
	} st;
	u8 value;
};

union ktd2037_blikq_reg {
	struct {
		u8 blinq_mode:1;
		u8 must_1:2;
		u8 non_used:4;
		u8 read_only:1;
	} st;
	u8 value;
};

union ktd2037_ramp_rate {
	struct {
		u8 rise:4;
		u8 fall:4;
	} st;
	u8 value;
};

typedef struct
{
    u8 leds_bri[3];

    u32 on_ms;
    u32 off_ms;

    union ktd2037_period_reg period;
    u8 percent_pwm1;
    u8 percent_pwm2;
    u8 leds_reg[3];
    union ktd2037_reset_ctrl_reg reset_ctrl;
    union ktd2037_ch_ctrl_reg ch_ctrl;
    union ktd2037_ramp_rate ramp_rate;

} ktd2037_led_data;

struct i2c_client *g_client = NULL;

ktd2037_led_data g_ktd2037_data;

// extern struct mt65xx_led_data *g_leds_data[MT65XX_LED_TYPE_TOTAL];

// ========================
//  I2C Access Functions
// ========================
static int ktd2037_write_i2c(u8 reg, u8 value)
{
	u8 data_buf[2] = { 0x00 };
	int ret = 0;

	data_buf[0] = reg;
	data_buf[1] = value;

	if (g_client == NULL) {
		CDBG("I2C Client was not initialized. Function not work!");
		return -1;
	}

	CDBG("Write I2C Addr=0x%02X Reg=0x%02X, Value=0x%02X", g_client->addr, reg, value);
	ret = i2c_master_send(g_client, (const char*)data_buf, 2);
	if (ret < 0) {
		CDBG("i2c_master_send(0x%02X, 0x%02X) failed and get ret=%d", data_buf[0], data_buf[1], ret);
		return -ENODEV;
	}

	return 0;
}

// AkenHsu: KTD2037 Cannot support read function since the I2C read format is different
#if 0
static int ktd2037_read_i2c(u8 reg, u8 *returnData)
{
	u8 data_buf[2] = { 0x00 };
	int ret = 0;

	data_buf[0] = reg;
	CDBG("BEGIN Read I2C Addr=0x%02X Reg=0x%02X", g_client->addr, reg);
	ret = i2c_master_send(g_client, (const char*)data_buf, 1);
	if (ret < 0) {
		CDBG("i2c_master_send failed and get ret=%d", ret);
		return ret;
	}

	ret = i2c_master_recv(g_client, returnData, 1);
	if (ret < 0) {
		CDBG("i2c_master_send failed and get ret=%d", ret);
		return ret;
	}
	CDBG("END Value=0x%02X", *returnData);

	return 0;
}
#endif

// ========================
//  Misc Functions
// ========================
static u8 ktd2037_get_brightness_reg(enum ktd2037_led_color color, u32 bri) {
	u8 reg = 0;
	u8 max_reg = 0;

	switch (color) {
		case LED_RED:
			max_reg = KTD2037_RED_MAX_CURRENT_REG;
			break;
		case LED_GRN:
			max_reg = KTD2037_GRN_MAX_CURRENT_REG;
			break;
		case LED_BLU:
			max_reg = KTD2037_BLU_MAX_CURRENT_REG;
			break;
		default:
			max_reg = KTD2037_RED_MAX_CURRENT_REG;
			break;
	}

	reg = bri * max_reg / 255;
	if (bri != 0) reg ++;
	if (reg > max_reg) reg = max_reg;

	CDBG("Brightness=%d, Reg=%d(0x%02X)", bri, reg, reg);
	return reg;
}

static u8 ktd2037_get_period_reg(u32 period) {
	u8 reg = 0;

	reg = period / 128;
	if (reg >= 1) reg -= 1;

	CDBG("Period=%d, Reg=%d(0x%02X)", period, reg, reg);
	return reg;
}

static u8 ktd2037_get_percent_reg(u32 off, u32 on) {
	u32 reg = 0;
	u32 period = on + off;

	reg = (on * 250) / (period);

	CDBG("Percent=%d/%d, Reg=%d(0x%02X)", on, period, reg, reg);
	return (u8)reg;
}

static void ktd2037_convert_blink(u32 on, u32 off, u8 *period_reg, u8 *persent_reg)
{
	u32 period = 0;

	CDBG("BEGIN On=%d, Off=%d", on, off);

	if (on == 0 || off == 0) {
		period_reg = 0;
		persent_reg = 0;
		CDBG("END on or off is 0");
		return;
	}
	if (on > KTD2037_MAX_ON_MS || off > KTD2037_MAX_OFF_MS) {
		period_reg = 0;
		persent_reg = 0;
		CDBG("END on or off is over range");
		return;
	}

	period = on + off < KTD2037_MIN_PERIOD_MS ? KTD2037_MIN_PERIOD_MS : on + off;
	CDBG("Actual Period = %d", period);

	*period_reg = ktd2037_get_period_reg(period);
	*persent_reg = ktd2037_get_percent_reg(off, on);

	CDBG("END period_reg=%d(0x%02X), persent_reg=%d(0x%02X)", *period_reg, *period_reg, *persent_reg, *persent_reg);
}

static int ktd2037_switch_blinq(u8 onoff)
{
	union ktd2037_blikq_reg blinq_reg;
	int rc = 0;

	CDBG("BEGIN");

	blinq_reg.st.must_1 = 3;
	blinq_reg.st.blinq_mode = onoff == 0 ? 0 : 1;

	rc = ktd2037_write_i2c(KTD2037_REG_AUTOBLINQ, blinq_reg.value);
	CDBG("END Write RegBlinQ=0x%02X rc=%d", blinq_reg.value, rc);
	return rc;
}

#if 0 // AKenHsu: no use now
static int ktd2037_init_chip(void)
{
	int i = 0;
	int rc = 0;

	CDBG("BEGIN");

	// Init g_ktd2037_data
	g_ktd2037_data.off_ms = 0;
	g_ktd2037_data.on_ms = 0;

	for (i = 0; i <= LED_BLU; i++) {
		g_ktd2037_data.leds_bri[i] = 0;
		g_ktd2037_data.leds_reg[i] = 0;
	}

	// Turn Off the Auto BlinQ function
	ktd2037_switch_blinq(0);

	// Init Period and PWM percent
	g_ktd2037_data.period.st.period = 0;
	g_ktd2037_data.period.st.ramp = 0;
	rc = ktd2037_write_i2c(KTD2037_REG_FLASH_PERIOD, g_ktd2037_data.ramp_rate.value);

	// Init percent PWM timers
	g_ktd2037_data.percent_pwm1 = 0;
	g_ktd2037_data.percent_pwm2 = 0;
	rc = ktd2037_write_i2c(KTD2037_REG_PWM1_TIMER, g_ktd2037_data.percent_pwm1);
	rc = ktd2037_write_i2c(KTD2037_REG_PWM2_TIMER, g_ktd2037_data.percent_pwm2);

	// Set LED Disable (REG 4)
	g_ktd2037_data.ch_ctrl.st.led1_en_timer = 0;
	g_ktd2037_data.ch_ctrl.st.led2_en_timer = 0;
	g_ktd2037_data.ch_ctrl.st.led3_en_timer = 0;
	g_ktd2037_data.ch_ctrl.st.device_en = 0;
	rc = ktd2037_write_i2c(KTD2037_REG_LED_EN, g_ktd2037_data.ramp_rate.value);

	// Set ramp rate (REG 5)
	g_ktd2037_data.ramp_rate.st.fall = 0;
	g_ktd2037_data.ramp_rate.st.rise = 0;
	rc = ktd2037_write_i2c(KTD2037_REG_TRISE_TFALL, g_ktd2037_data.ramp_rate.value);

	CDBG("END");
	return rc;
}
#endif

static int ktd2037_enable_chip(void)
{
	u8 is_blink = 0;
	int i = 0;

	CDBG("BEGIN");

	// Turn off all LEDs first of all
	g_ktd2037_data.ch_ctrl.st.led1_en_timer = 0;
	g_ktd2037_data.ch_ctrl.st.led2_en_timer = 0;
	g_ktd2037_data.ch_ctrl.st.led3_en_timer = 0;
	g_ktd2037_data.ch_ctrl.st.device_en = 0;
	ktd2037_write_i2c(KTD2037_REG_LED_EN, g_ktd2037_data.ch_ctrl.value);

	if (g_ktd2037_data.leds_reg[LED_RED] == 0 && g_ktd2037_data.leds_reg[LED_GRN] == 0 && g_ktd2037_data.leds_reg[LED_BLU] == 0) {
		CDBG("END_OO Turn OFF All LEDs.");
		return 0;
	}

	is_blink = g_ktd2037_data.off_ms == 0 || g_ktd2037_data.on_ms == 0 ? 0 : 1;

	// Set period and pwm percent
	if (is_blink) {
		ktd2037_write_i2c(KTD2037_REG_FLASH_PERIOD, g_ktd2037_data.period.value);
		ktd2037_write_i2c(KTD2037_REG_PWM1_TIMER, g_ktd2037_data.percent_pwm1);
	}

	// Set RGB
	for (i = LED_RED; i <= LED_BLU; i++) {
		if (g_ktd2037_data.leds_reg[i] == 0) { // LED is OFF
			switch (i) {
			case LED_RED:
				g_ktd2037_data.ch_ctrl.st.led1_en_timer = 0;
				break;
			case LED_GRN:
				g_ktd2037_data.ch_ctrl.st.led2_en_timer = 0;
				break;
			case LED_BLU:
				g_ktd2037_data.ch_ctrl.st.led3_en_timer = 0;
				break;
			} // end of switch
		} else { // LED is ON
			ktd2037_write_i2c(KTD2037_REG_LED_BASE + i, g_ktd2037_data.leds_reg[i]);
			switch (i) {
			case LED_RED:
				g_ktd2037_data.ch_ctrl.st.led1_en_timer = is_blink ? 2 : 1;
				break;
			case LED_GRN:
				g_ktd2037_data.ch_ctrl.st.led2_en_timer = is_blink ? 2 : 1;
				break;
			case LED_BLU:
				g_ktd2037_data.ch_ctrl.st.led3_en_timer = is_blink ? 2 : 1;
				break;
			} // end of switch
		} // end of if reg == 0
	} // end if for (i)
	g_ktd2037_data.ch_ctrl.st.device_en = 1;
	ktd2037_write_i2c(KTD2037_REG_LED_EN, g_ktd2037_data.ch_ctrl.value);

	CDBG("END Turn ON LEDs.");
	return 0;
}

// ========================
//  LED Attributes
// ========================
ssize_t ktd2037_config_save(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	u32 cfg_rgb = 0;
	u32 cfg_on = 0, cfg_off = 0;
	u8 period_reg = 0, percent_reg = 0;
	int i = 0;

	CDBG("BEGIN buf=%s", buf);

	if (sscanf(buf, "%x %d %d", &cfg_rgb, &cfg_on, &cfg_off) != 3) {
		CDBG("Input value error");
	}
	g_ktd2037_data.leds_bri[LED_RED] = (cfg_rgb & 0x00ff0000) >> 16;
	g_ktd2037_data.leds_bri[LED_GRN] = (cfg_rgb & 0x0000ff00) >> 8;
	g_ktd2037_data.leds_bri[LED_BLU] = (cfg_rgb & 0x000000ff);
	g_ktd2037_data.on_ms = cfg_on;
	g_ktd2037_data.off_ms = cfg_off;

	CDBG("R=%d G=%d B=%d OnOff=%d/%d",
		g_ktd2037_data.leds_bri[LED_RED], g_ktd2037_data.leds_bri[LED_GRN], g_ktd2037_data.leds_bri[LED_BLU], g_ktd2037_data.on_ms, g_ktd2037_data.off_ms);

	ktd2037_convert_blink(g_ktd2037_data.on_ms, g_ktd2037_data.off_ms, &period_reg, &percent_reg);
	g_ktd2037_data.period.st.period = period_reg;
	g_ktd2037_data.percent_pwm1 = percent_reg;

	for (i = LED_RED; i <= LED_BLU; i++) {
		g_ktd2037_data.leds_reg[i] = ktd2037_get_brightness_reg(i, g_ktd2037_data.leds_bri[i]);
	}

	ktd2037_enable_chip();

	CDBG("END R=0X%02X G=0X%02X B=0X%02X OnOff=%d/%d",
		g_ktd2037_data.leds_bri[LED_RED], g_ktd2037_data.leds_bri[LED_GRN], g_ktd2037_data.leds_bri[LED_BLU], g_ktd2037_data.on_ms, g_ktd2037_data.off_ms);
	return count;
}
ssize_t ktd2037_config_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	CDBG("BEGIN");

	rc = scnprintf(buf, PAGE_SIZE, "Usage: write \"RRGGBB msOn msOff\"\nRGB=%02x%02x%02x OnOff=%d/%d\n",
		g_ktd2037_data.leds_bri[LED_RED], g_ktd2037_data.leds_bri[LED_GRN], g_ktd2037_data.leds_bri[LED_BLU], g_ktd2037_data.on_ms, g_ktd2037_data.off_ms);

	CDBG("END rc=%d, R=0X%02X G=0X%02X B=0X%02X OnOff=%d/%d", rc,
		g_ktd2037_data.leds_bri[LED_RED], g_ktd2037_data.leds_bri[LED_GRN], g_ktd2037_data.leds_bri[LED_BLU], g_ktd2037_data.on_ms, g_ktd2037_data.off_ms);
	return rc;
}
static DEVICE_ATTR(config, 0664, ktd2037_config_show, ktd2037_config_save);

// For OFF charging mode
int ktd2037_rgb_led_off(void)
{
	CDBG("BEGIN");

	g_ktd2037_data.ch_ctrl.st.led1_en_timer = 0;
	g_ktd2037_data.ch_ctrl.st.led2_en_timer = 0;
	g_ktd2037_data.ch_ctrl.st.led3_en_timer = 0;
	g_ktd2037_data.ch_ctrl.st.device_en = 0;
	ktd2037_write_i2c(KTD2037_REG_LED_EN, g_ktd2037_data.ch_ctrl.value);

	CDBG("END");
	return 0;
}

int ktd2037_rgb_led_set(enum ktd2037_led_color led_color, u8 led_brightness)
{
	CDBG("BEGIN Color=%d Brightness=%d", led_color, led_brightness);
	g_ktd2037_data.leds_bri[led_color] = led_brightness;
	g_ktd2037_data.leds_reg[led_color] = ktd2037_get_brightness_reg(led_color, g_ktd2037_data.leds_bri[led_color]);

	ktd2037_enable_chip();
	CDBG("END");
	return 0;
}

static ssize_t ktd2037_red_save(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	u8 read_data;
	CDBG("BEGIN buf=%s", buf);

	if (sscanf(buf, "%hhu", &read_data) != 1) {
		CDBG("Input value error");
	}

	ktd2037_rgb_led_set(LED_RED, read_data);

	CDBG("END Red=0x%02X (%d)", g_ktd2037_data.leds_bri[LED_RED], g_ktd2037_data.leds_bri[LED_RED]);
	return count;
}
static ssize_t ktd2037_red_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	CDBG("BEGIN");

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", g_ktd2037_data.leds_bri[LED_RED]);

	CDBG("END Red=0x%02X (Reg=%d), rc = %d", g_ktd2037_data.leds_bri[LED_RED], g_ktd2037_data.leds_bri[LED_RED], rc);
	return rc;
}
static DEVICE_ATTR(red, 0664, ktd2037_red_show, ktd2037_red_save);

static ssize_t ktd2037_green_save(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	u8 read_data;
	CDBG("BEGIN buf=%s", buf);

	if (sscanf(buf, "%hhu", &read_data) != 1) {
		CDBG("Input value error");
	}

	ktd2037_rgb_led_set(LED_GRN, read_data);

	CDBG("END Green=0x%02X (%d)", g_ktd2037_data.leds_bri[LED_GRN], g_ktd2037_data.leds_bri[LED_GRN]);
	return count;
}
static ssize_t ktd2037_green_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	CDBG("BEGIN");

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", g_ktd2037_data.leds_bri[LED_GRN]);

	CDBG("END Green=0x%02X (Reg=%d), rc = %d", g_ktd2037_data.leds_bri[LED_GRN], g_ktd2037_data.leds_bri[LED_GRN], rc);
	return rc;
}
static DEVICE_ATTR(green, 0664, ktd2037_green_show, ktd2037_green_save);

static ssize_t ktd2037_blue_save(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	u8 read_data;
	CDBG("BEGIN buf=%s", buf);

	if (sscanf(buf, "%hhu", &read_data) != 1) {
		CDBG("Input value error");
	}

	ktd2037_rgb_led_set(LED_BLU, read_data);

	CDBG("END Blue=0x%02X (%d)", g_ktd2037_data.leds_bri[LED_BLU], g_ktd2037_data.leds_bri[LED_BLU]);
	return count;
}
static ssize_t ktd2037_blue_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	CDBG("BEGIN");

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", g_ktd2037_data.leds_bri[LED_BLU]);

	CDBG("END Blue=0x%02X (Reg=%d), rc = %d", g_ktd2037_data.leds_bri[LED_BLU], g_ktd2037_data.leds_bri[LED_BLU], rc);
	return rc;
}
static DEVICE_ATTR(blue, 0664, ktd2037_blue_show, ktd2037_blue_save);

static struct attribute *ktd2037_rgb_attrs[] = {
	&dev_attr_config.attr,
	&dev_attr_red.attr,
	&dev_attr_green.attr,
	&dev_attr_blue.attr,
	NULL
};
static struct attribute_group ktd2037_rgb_attr_group = {
	.attrs = ktd2037_rgb_attrs,
};

// ========================
//  Register Attributes
// ========================
static u8 g_reg_addr;
static ssize_t ktd2037_addr_save(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	CDBG("BEGIN buf=%s", buf);

	if (sscanf(buf, "%hhu", &g_reg_addr) != 1) {
		CDBG("Input value error");
	}
	CDBG("END RegAddr=0x%02X", g_reg_addr);
	return count;
}
static ssize_t ktd2037_addr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	CDBG("BEGIN RegAddr=%d", g_reg_addr);

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", g_reg_addr);

	CDBG("END");
	return rc;
}
static DEVICE_ATTR(addr, 0664, ktd2037_addr_show, ktd2037_addr_save);

static ssize_t ktd2037_data_save(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	u8 reg_data;
	CDBG("BEGIN buf=%s", buf);

	if (sscanf(buf, "%hhu", &reg_data) != 1) {
		CDBG("Input value error");
	}

	ktd2037_write_i2c(g_reg_addr, reg_data);

	CDBG("END RegData = 0x%02X", reg_data);
	return count;
}
static ssize_t ktd2037_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	u8 reg_data = 0;
//	int i2c_rc = 0, rc = 0;
	CDBG("BEGIN no use!");

//	i2c_rc = ktd2037_read_i2c(g_reg_addr, &reg_data);
//	rc = scnprintf(buf, PAGE_SIZE, "%d\n", reg_data);

//	CDBG("END RegData=0x%x, i2c_rc=%d, rc=%d", reg_data, i2c_rc, rc);
	return 0; //rc;
}
static DEVICE_ATTR(data, 0664, ktd2037_data_show, ktd2037_data_save);

static struct attribute *ktd2037_reg_attrs[] = {
	&dev_attr_addr.attr,
	&dev_attr_data.attr,
	NULL
};
static struct attribute_group ktd2037_reg_attr_group = {
	.attrs = ktd2037_reg_attrs,
};

static int ktd2037_create_attr(struct device *dev)
{
	int ret = 0;
	CDBG("BEGIN");

	ret = sysfs_create_group(&dev->kobj, &ktd2037_rgb_attr_group);

	ret = sysfs_create_group(&dev->kobj, &ktd2037_reg_attr_group);

	CDBG("END ret=%d", ret);
	return ret;
}

// ========================
//  I2C Driver
// ========================
#define I2C_KTD2037_SLAVE_ADDR 0x30
#define I2C_KTD2037_BUS 3 //I2C3

static struct i2c_client *ktd2037_i2c_client = NULL;
static struct i2c_board_info __initdata i2c_ktd2037 = { I2C_BOARD_INFO("ktd2037", I2C_KTD2037_SLAVE_ADDR) };

static const struct i2c_device_id ktd2037_i2c_ids[] =
{
  { "ktd2037", 0 },
  { },
};

static int ktd2037_i2c_probe(struct i2c_client *i2cclient, const struct i2c_device_id *id)
{
	int ret = 0;
	CDBG("BEGIN");

	if (!i2c_check_functionality(i2cclient->adapter, I2C_FUNC_I2C)) {
		CDBG("need I2C_FUNC_I2C.");
		ret = -ENODEV;
		goto error;
	}

	if (!i2c_check_functionality(i2cclient->adapter, I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {
		pr_err("i2c functionality check error\n");
		dev_err(&i2cclient->dev, "need I2C_FUNC_SMBUS_WRITE_I2C_BLOCK.\n");
		ret = -EIO;
		goto error;
	}

	g_client = i2cclient;

	ktd2037_create_attr(&(i2cclient->dev));

	ktd2037_switch_blinq(0);

	ktd2037_rgb_led_off();

	CDBG("END");

	return 0;

error:
	CDBG("END_XX ret=%d", ret);
	return ret;
}

static int ktd2037_i2c_remove(struct i2c_client *i2c)
{
	struct ktd2037 *ktd2037 = i2c_get_clientdata(i2c);

	CDBG("BEGIN");

	ktd2037_rgb_led_off();

	ktd2037_i2c_client = NULL;
	i2c_unregister_device(i2c);
	kfree(ktd2037);

	return 0;
}

static void ktd2037_i2c_shutdown(struct i2c_client *i2c)
{
	CDBG("BEGIN");

	// Turn off all LEDs
	ktd2037_rgb_led_off();

	CDBG("END");
}

static const struct of_device_id ktd2037_i2c_of_match[] = {
	{.compatible = "mediatek,led_ktd2037"},
	{},
};

static struct i2c_driver ktd2037_i2c_driver =
{
	.driver = {
		.name   = "ktd2037-i2c",
		.owner  = THIS_MODULE,
		.of_match_table = ktd2037_i2c_of_match,
	},
	.id_table = ktd2037_i2c_ids,
	.probe    = ktd2037_i2c_probe,
	.remove   = ktd2037_i2c_remove,
	.shutdown = ktd2037_i2c_shutdown,
};

static int leds_ktd2037_probe(struct platform_device *pdev)
{
	int ret = 0;

	CDBG("BEGIN platform_name=%s", pdev->name);

	ret = i2c_register_board_info(I2C_KTD2037_BUS, &i2c_ktd2037, 1);
	CDBG("i2c_register_board_info = %d", ret);

	if (i2c_add_driver(&ktd2037_i2c_driver)) {
		CDBG("add I2C driver error");
		return -1;
	} else {
		CDBG("add I2C driver success");
	}
	CDBG("END");

	return 0;
}

static const struct of_device_id ktd2037_platform_of_match[] = {
	{.compatible = "mediatek,ktd2037-platform"},
	{},
};

static struct platform_driver ktd2037_platform_driver = {
	.driver = {
		.name = "ktd2037-device",
		.owner = THIS_MODULE,
		.of_match_table = ktd2037_platform_of_match,
	},
	.probe = leds_ktd2037_probe,
};

static int __init leds_ktd2037_init(void)
{
	int ret;

	CDBG("BEGIN");

	ret = platform_driver_register(&ktd2037_platform_driver);

	if (ret) {
		CDBG("END_XX ret = %d", ret);
		return ret;
	}

	CDBG("END ret = %d", ret);
	return 0;
}

static void __exit leds_ktd2037_exit(void)
{
	i2c_del_driver(&ktd2037_i2c_driver);
}

module_init(leds_ktd2037_init);
module_exit(leds_ktd2037_exit);

MODULE_AUTHOR("ARIMA Inc.");
MODULE_DESCRIPTION("LED driver for KTD2037 chip");
MODULE_LICENSE("GPL");
MODULE_ALIAS("leds-ktd2037");

