/*
 * RGB-led driver for Maxim MAX77843
 *
 * Copyright (C) 2013 Maxim Integrated Product
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/mfd/max77843.h>
#include <linux/mfd/max77843-private.h>
#include <linux/leds-max77843-rgb.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/regmap.h>
#include <linux/sec_class.h>

#define SEC_LED_SPECIFIC

/* Registers */
/*defined max77843-private.h*//*
 max77843_led_reg {
	MAX77843_RGBLED_REG_LEDEN           = 0x30,
	MAX77843_RGBLED_REG_LED0BRT         = 0x31,
	MAX77843_RGBLED_REG_LED1BRT         = 0x32,
	MAX77843_RGBLED_REG_LED2BRT         = 0x33,
	MAX77843_RGBLED_REG_LED3BRT         = 0x34,
	MAX77843_RGBLED_REG_LEDRMP          = 0x36,
	MAX77843_RGBLED_REG_LEDBLNK         = 0x38,
	MAX77843_LED_REG_END,
};*/

/* MAX77843_REG_LED0BRT */
#define MAX77843_LED0BRT	0xFF

/* MAX77843_REG_LED1BRT */
#define MAX77843_LED1BRT	0xFF

/* MAX77843_REG_LED2BRT */
#define MAX77843_LED2BRT	0xFF

/* MAX77843_REG_LED3BRT */
#define MAX77843_LED3BRT	0xFF

/* MAX77843_REG_LEDBLNK */
#define MAX77843_LEDBLINKD	0xF0
#define MAX77843_LEDBLINKP	0x0F

/* MAX77843_REG_LEDRMP */
#define MAX77843_RAMPUP		0xF0
#define MAX77843_RAMPDN		0x0F

#define LED_R_MASK		0x00FF0000
#define LED_G_MASK		0x0000FF00
#define LED_B_MASK		0x000000FF
#define LED_MAX_CURRENT		0xFF

/* MAX77843_STATE*/
#define LED_DISABLE			0
#define LED_ALWAYS_ON			1
#define LED_BLINK			2

#define BASE_DYNAMIC_LED_CURRENT 0x14
#define BASE_LOW_POWER_CURRENT 0x05

#define LEDBLNK_ON(time)	((time < 100) ? 0 :			\
				(time < 500) ? time/100-1 :		\
				(time < 3250) ? (time-500)/250+4 : 15)

#define LEDBLNK_OFF(time)	((time < 500) ? 0x00 :			\
				(time < 5000) ? time/500 :		\
				(time < 8000) ? (time-5000)/1000+10 :	 \
				(time < 12000) ? (time-8000)/2000+13 : 15)

extern bool pu_checkBlackout(void);
extern bool sttg_pu_blockleds;

static bool sttg_fled_fade = 0;
static unsigned int sttg_fled_powermode = 0;
static unsigned int sttg_fled_high_r_gain = 215;
static unsigned int sttg_fled_high_g_gain = 210;
static unsigned int sttg_fled_high_b_gain = 80;
static unsigned int sttg_fled_low_r_gain = 15;
static unsigned int sttg_fled_low_g_gain = 10;
static unsigned int sttg_fled_low_b_gain = 7;
static unsigned int sttg_fled_charged_r_gain = 80;
static unsigned int sttg_fled_charged_g_gain = 255;
static unsigned int sttg_fled_charged_b_gain = 80;
static unsigned int sttg_fled_charging_r_gain = 255;
static unsigned int sttg_fled_charging_g_gain = 80;
static unsigned int sttg_fled_charging_b_gain = 80;
static unsigned int sttg_fled_missednoti_r_gain = 255;
static unsigned int sttg_fled_missednoti_g_gain = 255;
static unsigned int sttg_fled_missednoti_b_gain = 255;
static unsigned int sttg_fled_blink_on = 0;
static unsigned int sttg_fled_blink_off = 0;
static unsigned int sttg_fled_ramp_on = 0;
static unsigned int sttg_fled_ramp_off = 0;

static struct timer_list timer_alternatefrontled;
struct an30259a_data *leddata;
struct work_struct work_controlFrontLED;
static struct workqueue_struct *wq_controlFrontLED;
static unsigned int alternateFrontLED_duty1 = 0;
static unsigned int alternateFrontLED_r1 = 0;
static unsigned int alternateFrontLED_g1 = 0;
static unsigned int alternateFrontLED_b1 = 0;
static unsigned int alternateFrontLED_duty2 = 0;
static unsigned int alternateFrontLED_r2 = 0;
static unsigned int alternateFrontLED_g2 = 0;
static unsigned int alternateFrontLED_b2 = 0;
static unsigned int alternatefrontled_mode = 0;
static unsigned int wq_r = 0;
static unsigned int wq_g = 0;
static unsigned int wq_b = 0;
static bool flg_stop_incoming_leds = false;
static struct wake_lock fled_wake_lock;

static u8 led_dynamic_current = BASE_DYNAMIC_LED_CURRENT;
static u8 led_lowpower_mode = 0x0;

enum max77843_led_color {
	WHITE,
	RED,
	GREEN,
	BLUE,
};
enum max77843_led_pattern {
	PATTERN_OFF,
	CHARGING,
	CHARGING_ERR,
	MISSED_NOTI,
	LOW_BATTERY,
	FULLY_CHARGED,
	POWERING,
};

static struct device *led_dev;

struct max77843_rgb {
	struct led_classdev led[4];
	struct i2c_client *i2c;
	unsigned int delay_on_times_ms;
	unsigned int delay_off_times_ms;
};

#if defined (CONFIG_SEC_FACTORY)
#if defined(CONFIG_SEC_TRLTE_PROJECT) || defined(CONFIG_SEC_TBLTE_PROJECT)
static int jig_val;
extern int get_lcd_attached(void);

static int __init muic_get_jig_status(char *mode)
{
	if(get_option(&mode, &jig_val)) {
		printk(KERN_INFO "%s = %d\n", __func__, jig_val);
		return 0;
	}
	return -EINVAL;
}
early_param("uart_dbg", muic_get_jig_status);
#endif
#endif

static int max77843_rgb_number(struct led_classdev *led_cdev,
				struct max77843_rgb **p)
{
	const struct device *parent = led_cdev->dev->parent;
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(parent);
	int i;

	*p = max77843_rgb;

	for (i = 0; i < 4; i++) {
		if (led_cdev == &max77843_rgb->led[i]) {
			//pr_info("leds-max77843-rgb: %s, %d\n", __func__, i);
			return i;
		}
	}

	return -ENODEV;
}

static void max77843_rgb_set(struct led_classdev *led_cdev,
				unsigned int brightness)
{
	const struct device *parent = led_cdev->dev->parent;
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(parent);
	struct device *dev;
	int n;
	int ret;

	ret = max77843_rgb_number(led_cdev, &max77843_rgb);
	if (IS_ERR_VALUE(ret)) {
		dev_err(led_cdev->dev,
			"max77843_rgb_number() returns %d.\n", ret);
		return;
	}

	dev = led_cdev->dev;
	n = ret;

	if (brightness == LED_OFF) {
		/* Flash OFF */
		ret = max77843_update_reg(max77843_rgb->i2c,
					MAX77843_LED_REG_LEDEN, 0 , 3 << (2*n));
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "can't write LEDEN : %d\n", ret);
			return;
		}
	} else {
		/* Set current */
		ret = max77843_write_reg(max77843_rgb->i2c,
				MAX77843_LED_REG_LED0BRT + n, brightness);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "can't write LEDxBRT : %d\n", ret);
			return;
		}
		/* Flash ON */
		ret = max77843_update_reg(max77843_rgb->i2c,
				MAX77843_LED_REG_LEDEN, 0x55, 3 << (2*n));
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "can't write FLASH_EN : %d\n", ret);
			return;
		}
	}
}

static void max77843_rgb_set_state(struct led_classdev *led_cdev,
				unsigned int brightness, unsigned int led_state)
{
	const struct device *parent = led_cdev->dev->parent;
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(parent);
	struct device *dev;
	int n;
	int ret;

	//pr_info("leds-max77843-rgb: %s\n", __func__);

	ret = max77843_rgb_number(led_cdev, &max77843_rgb);

	if (IS_ERR_VALUE(ret)) {
		dev_err(led_cdev->dev,
			"max77843_rgb_number() returns %d.\n", ret);
		return;
	}

	dev = led_cdev->dev;
	n = ret;

	max77843_rgb_set(led_cdev, brightness);

	ret = max77843_update_reg(max77843_rgb->i2c,
			MAX77843_LED_REG_LEDEN, led_state << (2*n), 0x3 << 2*n);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "can't write FLASH_EN : %d\n", ret);
		return;
	}
}

static unsigned int max77843_rgb_get(struct led_classdev *led_cdev)
{
	const struct device *parent = led_cdev->dev->parent;
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(parent);
	struct device *dev;
	int n;
	int ret;
	u8 value;

	pr_info("leds-max77843-rgb: %s\n", __func__);

	ret = max77843_rgb_number(led_cdev, &max77843_rgb);
	if (IS_ERR_VALUE(ret)) {
		dev_err(led_cdev->dev,
			"max77843_rgb_number() returns %d.\n", ret);
		return 0;
	}
	n = ret;

	dev = led_cdev->dev;

	/* Get status */
	ret = max77843_read_reg(max77843_rgb->i2c,
				MAX77843_LED_REG_LEDEN, &value);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "can't read LEDEN : %d\n", ret);
		return 0;
	}
	if (!(value & (1 << n)))
		return LED_OFF;

	/* Get current */
	ret = max77843_read_reg(max77843_rgb->i2c,
				MAX77843_LED_REG_LED0BRT + n, &value);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "can't read LED0BRT : %d\n", ret);
		return 0;
	}

	return value;
}

static int max77843_rgb_ramp(struct device *dev, int ramp_up, int ramp_down)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	int value;
	int ret;

	pr_info("leds-max77843-rgb: %s\n", __func__);

	if (ramp_up <= 800) {
		ramp_up /= 100;
	} else {
		ramp_up = (ramp_up - 800) * 2 + 800;
		ramp_up /= 100;
	}

	if (ramp_down <= 800) {
		ramp_down /= 100;
	} else {
		ramp_down = (ramp_down - 800) * 2 + 800;
		ramp_down /= 100;
	}

	value = (ramp_down) | (ramp_up << 4);
	ret = max77843_write_reg(max77843_rgb->i2c,
					MAX77843_LED_REG_LEDRMP, value);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "can't write REG_LEDRMP : %d\n", ret);
		return -ENODEV;
	}

	return 0;
}

static int max77843_rgb_blink(struct device *dev,
				unsigned int delay_on, unsigned int delay_off)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	int value;
	int ret = 0;

	pr_info("leds-max77843-rgb: %s\n", __func__);

	if( delay_on > 3250 || delay_off > 12000 )
		return -EINVAL;
	else {
		value = (LEDBLNK_ON(delay_on) << 4) | LEDBLNK_OFF(delay_off);
		ret = max77843_write_reg(max77843_rgb->i2c,
					MAX77843_LED_REG_LEDBLNK, value);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "can't write REG_LEDBLNK : %d\n", ret);
			return -EINVAL;
		}
	}

	return ret;
}

#ifdef CONFIG_OF
static struct max77843_rgb_platform_data
			*max77843_rgb_parse_dt(struct device *dev)
{
	struct max77843_rgb_platform_data *pdata;
	struct device_node *np;
	int ret;
	int i;

	pr_info("leds-max77843-rgb: %s\n", __func__);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(pdata == NULL))
		return ERR_PTR(-ENOMEM);

	np = of_find_node_by_name(NULL, "rgb");
	if (unlikely(np == NULL)) {
		dev_err(dev, "rgb node not found\n");
		devm_kfree(dev, pdata);
		return ERR_PTR(-EINVAL);
	}

	for (i = 0; i < 4; i++)	{
		ret = of_property_read_string_index(np, "rgb-name", i,
						(const char **)&pdata->name[i]);

		pr_info("leds-max77843-rgb: %s, %s\n", __func__,pdata->name[i]);

		if (IS_ERR_VALUE(ret)) {
			devm_kfree(dev, pdata);
			return ERR_PTR(ret);
		}
	}

	return pdata;
}
#endif

static void max77843_rgb_reset(struct device *dev)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	max77843_rgb_set_state(&max77843_rgb->led[RED], LED_OFF, LED_DISABLE);
	max77843_rgb_set_state(&max77843_rgb->led[GREEN], LED_OFF, LED_DISABLE);
	max77843_rgb_set_state(&max77843_rgb->led[BLUE], LED_OFF, LED_DISABLE);
	max77843_rgb_ramp(dev, 0, 0);
}

static void controlFrontLED_work(struct work_struct *work)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(led_dev);
	
	if (!wq_r)
		max77843_rgb_set_state(&max77843_rgb->led[RED], LED_OFF, LED_DISABLE);
	else
		max77843_rgb_set_state(&max77843_rgb->led[RED], wq_r, LED_ALWAYS_ON);
	
	if (!wq_g)
		max77843_rgb_set_state(&max77843_rgb->led[GREEN], LED_OFF, LED_DISABLE);
	else
		max77843_rgb_set_state(&max77843_rgb->led[GREEN], wq_g, LED_ALWAYS_ON);
	
	if (!wq_b)
		max77843_rgb_set_state(&max77843_rgb->led[BLUE], LED_OFF, LED_DISABLE);
	else
		max77843_rgb_set_state(&max77843_rgb->led[BLUE], wq_b, LED_ALWAYS_ON);
}

void controlFrontLED(unsigned int r, unsigned int g, unsigned int b)
{
	// make sure nothing is out of range.
	if (r > 255)
		r = 255;
	
	if (g > 255)
		g = 255;
	
	if (b > 255)
		b = 255;
	
	wq_r = r;
	wq_g = g;
	wq_b = b;
	
	// if we're locked out and don't want leds on, turn them off.
	if (pu_checkBlackout() && sttg_pu_blockleds) {
		wq_r = 0;
		wq_g = 0;
		wq_b = 0;
	}
	
	pr_info("[LED/controlFrontLED] control led\n");
	
	queue_work_on(0, wq_controlFrontLED, &work_controlFrontLED);
	
}
EXPORT_SYMBOL(controlFrontLED);

static void timerhandler_alternatefrontled(unsigned long data)
{
	unsigned int tmp_duty = 0;
	
	if (!alternateFrontLED_duty1 || !alternateFrontLED_duty2) {
		// no duty cycle set one or more colors, just turn all the leds off and exit.
		
		// reset led.
		controlFrontLED(0, 0, 0);
		
		flg_stop_incoming_leds = false;
		
		// drop wakelock.
		wake_unlock(&fled_wake_lock);
		
		return;
	}
	
	if (!alternatefrontled_mode) {
		// mode was 0 (aka 1)
		
		//pr_info("[LED/timerhandler_alternatefrontled] turning on led2\n");
		
		// start color 2.
		controlFrontLED(alternateFrontLED_r2, alternateFrontLED_g2, alternateFrontLED_b2);
		tmp_duty = alternateFrontLED_duty2;
		alternatefrontled_mode = 1;
		
	} else {
		// mode was 1 (aka 2)
		
		//pr_info("[LED/timerhandler_alternatefrontled] turning on led1\n");
		
		// start color 1.
		controlFrontLED(alternateFrontLED_r1, alternateFrontLED_g1, alternateFrontLED_b1);
		tmp_duty = alternateFrontLED_duty1;
		alternatefrontled_mode = 0;
	}
	
	//pr_info("[LED/timerhandler_alternatefrontled] recycling in %d ms\n", tmp_duty);
	
	// start timer that will turn the other color on.
	mod_timer(&timer_alternatefrontled,
			  jiffies + msecs_to_jiffies(tmp_duty));
}

void alternateFrontLED(unsigned int duty1, unsigned int r1, unsigned int g1, unsigned int b1,
					   unsigned int duty2, unsigned int r2, unsigned int g2, unsigned int b2)
{
	if ((pu_checkBlackout() && sttg_pu_blockleds)
		|| !duty1 || !duty2) {
		// disable it if we're locked out and leds are blocked,
		// OR - if any duty cycle is 0
		
		pr_info("[LED/alternateFrontLED] stopping\n");
		del_timer(&timer_alternatefrontled);
		
		// reset led.
		controlFrontLED(0, 0, 0);
		
		flg_stop_incoming_leds = false;
		
		// drop wakelock.
		wake_unlock(&fled_wake_lock);
		
		return;
	}
	
	flg_stop_incoming_leds = true;
	
	// reset led.
	controlFrontLED(0, 0, 0);
	
	pr_info("[LED/alternateFrontLED] starting\n");
	
	// save timing and colors.
	alternateFrontLED_duty1 = duty1;
	alternateFrontLED_r1 = r1;
	alternateFrontLED_g1 = g1;
	alternateFrontLED_b1 = b1;
	
	alternateFrontLED_duty2 = duty2;
	alternateFrontLED_r2 = r2;
	alternateFrontLED_g2 = g2;
	alternateFrontLED_b2 = b2;
	
	// start first color.
	controlFrontLED(r1, g1, b1);
	alternatefrontled_mode = 0;
	
	// start timer that will turn the second color on.
	mod_timer(&timer_alternatefrontled,
			  jiffies + msecs_to_jiffies(duty1));
}
EXPORT_SYMBOL(alternateFrontLED);

void flashFrontLED(unsigned int duty1, unsigned int r1, unsigned int g1, unsigned int b1)
{
	// if we're locked out and don't want leds on, quit now.
	if (pu_checkBlackout() && sttg_pu_blockleds)
		return;
	
	// start wakelock so the flash doesn't get "stuck".
	wake_lock(&fled_wake_lock);
	
	// we don't want any incoming leds to override what we set.
	flg_stop_incoming_leds = true;
	
	// stop pending work.
	del_timer(&timer_alternatefrontled);
	
	// reset led.
	controlFrontLED(0, 0, 0);
	
	// no duty cycle for either color.
	alternateFrontLED_duty1 = 0;
	alternateFrontLED_duty2 = 0;
	
	// start first color.
	controlFrontLED(r1, g1, b1);
	
	// start timer that will turn it off.
	mod_timer(&timer_alternatefrontled,
			  jiffies + msecs_to_jiffies(duty1));
}
EXPORT_SYMBOL(flashFrontLED);

static ssize_t show_max77843_rgb_lowpower(struct device *dev,
									 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", led_lowpower_mode);
}

static ssize_t store_max77843_rgb_lowpower(struct device *dev,
					struct device_attribute *devattr,
					const char *buf, size_t count)
{
	int ret;
	u8 led_lowpower;

	ret = kstrtou8(buf, 0, &led_lowpower);
	if (ret != 0) {
		dev_err(dev, "fail to get led_lowpower.\n");
		return count;
	}
	
	if (!sttg_fled_powermode) {
		// don't apply override.
		
		led_lowpower_mode = led_lowpower;
		
		if (led_lowpower_mode == 1) {
			led_dynamic_current = BASE_LOW_POWER_CURRENT;
		} else {
			led_dynamic_current = BASE_DYNAMIC_LED_CURRENT;
		}
		
	} else if (sttg_fled_powermode == 1) {
		// low power always.
		
		led_lowpower_mode = 1;
		led_dynamic_current = BASE_LOW_POWER_CURRENT;
		
	} else if (sttg_fled_powermode == 2) {
		// high power always.
		
		led_lowpower_mode = 0;
		led_dynamic_current = BASE_DYNAMIC_LED_CURRENT;
	}
	
	pr_info("led_lowpower mode set to %i, led_dynamic_current set to %d\n", led_lowpower, led_dynamic_current);
	dev_dbg(dev, "led_lowpower mode set to %i\n", led_lowpower);

	return count;
}
static ssize_t store_max77843_rgb_brightness(struct device *dev,
					struct device_attribute *devattr,
					const char *buf, size_t count)
{
	int ret;
	u8 brightness;
	pr_info("leds-max77843-rgb: %s\n", __func__);

	ret = kstrtou8(buf, 0, &brightness);
	if (ret != 0) {
		dev_err(dev, "fail to get led_brightness.\n");
		return count;
	}

	led_lowpower_mode = 0;

	if (brightness > LED_MAX_CURRENT)
		brightness = LED_MAX_CURRENT;

	led_dynamic_current = brightness;

	dev_dbg(dev, "led brightness set to %i\n", brightness);

	return count;
}

static ssize_t store_max77843_rgb_pattern(struct device *dev,
					struct device_attribute *devattr,
					const char *buf, size_t count)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	unsigned int mode = 0;
	u8 led_r_brightness = 0;
	u8 led_g_brightness = 0;
	u8 led_b_brightness = 0;
	int delay_on_time = 0;
	int delay_off_time = 0;
	int ramp_delay_on_time = 0;
	int ramp_delay_off_time = 0;
	int ret;
	
	// if we're already controlling the leds,
	// OR - if we're locked out and don't want leds on, then quit now.
	if (flg_stop_incoming_leds || (pu_checkBlackout() && sttg_pu_blockleds))
		return count;
	
	pr_info("leds-max77843-rgb: %s, lowpower_mode : %d\n", __func__,led_lowpower_mode);

	ret = sscanf(buf, "%1d", &mode);
	if (ret == 0) {
		dev_err(dev, "fail to get led_pattern mode.\n");
		return count;
	}
	
	pr_info("[LED/store_max77843_rgb_pattern] mode: %d\n", mode);

	if (mode > POWERING)
		return count;

	/* Set all LEDs Off */
	max77843_rgb_reset(dev);
	if (mode == PATTERN_OFF)
		return count;

	switch (mode) {

	case CHARGING:
			
		if (led_lowpower_mode == 1) {
			if (sttg_fled_charging_r_gain && sttg_fled_low_r_gain)
				led_r_brightness = (sttg_fled_charging_r_gain * sttg_fled_low_r_gain) / 255;
			if (sttg_fled_charging_g_gain && sttg_fled_low_g_gain)
				led_g_brightness = (sttg_fled_charging_g_gain * sttg_fled_low_g_gain) / 255;
			if (sttg_fled_charging_b_gain && sttg_fled_low_b_gain)
				led_b_brightness = (sttg_fled_charging_b_gain * sttg_fled_low_b_gain) / 255;
		} else {
			if (sttg_fled_charging_r_gain && sttg_fled_low_r_gain)
				led_r_brightness = (sttg_fled_charging_r_gain * sttg_fled_high_r_gain) / 255;
			if (sttg_fled_charging_g_gain && sttg_fled_low_g_gain)
				led_g_brightness = (sttg_fled_charging_g_gain * sttg_fled_high_g_gain) / 255;
			if (sttg_fled_charging_b_gain && sttg_fled_low_b_gain)
				led_b_brightness = (sttg_fled_charging_b_gain * sttg_fled_high_b_gain) / 255;
		}
		
		if (led_r_brightness) {
			max77843_rgb_set_state(&max77843_rgb->led[RED], led_r_brightness, LED_ALWAYS_ON);
		}
		if (led_g_brightness) {
			max77843_rgb_set_state(&max77843_rgb->led[GREEN], led_g_brightness, LED_ALWAYS_ON);
		}
		if (led_b_brightness) {
			max77843_rgb_set_state(&max77843_rgb->led[BLUE], led_b_brightness, LED_ALWAYS_ON);
		}
			
		break;
			
	case CHARGING_ERR:
			
		max77843_rgb_blink(dev, 500, 500);
			
		if (led_lowpower_mode == 1)
			max77843_rgb_set_state(&max77843_rgb->led[RED], sttg_fled_low_r_gain, LED_BLINK);
		else
			max77843_rgb_set_state(&max77843_rgb->led[RED], sttg_fled_high_r_gain, LED_BLINK);
			
		break;
			
	case MISSED_NOTI:
			
		delay_on_time = 500;
		delay_off_time = 5000;
		
		if (sttg_fled_fade) {
			
			if (sttg_fled_ramp_on)
				ramp_delay_on_time = sttg_fled_ramp_on;
			else
				ramp_delay_on_time = delay_on_time;
			
			if (sttg_fled_ramp_off)
				ramp_delay_off_time = sttg_fled_ramp_off;
			else
				ramp_delay_off_time = delay_off_time;
			
			max77843_rgb_ramp(dev, ramp_delay_on_time, ramp_delay_off_time);
			
		} else {
			max77843_rgb_ramp(dev, 0, 0);
		}
			
		// override blink delays.
		if (sttg_fled_blink_on)
			delay_on_time = sttg_fled_blink_on;
		
		if (sttg_fled_blink_off)
			delay_off_time = sttg_fled_blink_off;
		
		max77843_rgb_blink(dev, delay_on_time, delay_off_time);
			
		if (led_lowpower_mode == 1) {
			if (sttg_fled_missednoti_r_gain && sttg_fled_low_r_gain)
				led_r_brightness = (sttg_fled_missednoti_r_gain * sttg_fled_low_r_gain) / 255;
			if (sttg_fled_missednoti_g_gain && sttg_fled_low_g_gain)
				led_g_brightness = (sttg_fled_missednoti_g_gain * sttg_fled_low_g_gain) / 255;
			if (sttg_fled_missednoti_b_gain && sttg_fled_low_b_gain)
				led_b_brightness = (sttg_fled_missednoti_b_gain * sttg_fled_low_b_gain) / 255;
		} else {
			if (sttg_fled_missednoti_r_gain && sttg_fled_low_r_gain)
				led_r_brightness = (sttg_fled_missednoti_r_gain * sttg_fled_high_r_gain) / 255;
			if (sttg_fled_missednoti_g_gain && sttg_fled_low_g_gain)
				led_g_brightness = (sttg_fled_missednoti_g_gain * sttg_fled_high_g_gain) / 255;
			if (sttg_fled_missednoti_b_gain && sttg_fled_low_b_gain)
				led_b_brightness = (sttg_fled_missednoti_b_gain * sttg_fled_high_b_gain) / 255;
		}
			
		if (led_r_brightness) {
			max77843_rgb_set_state(&max77843_rgb->led[RED], led_r_brightness, LED_BLINK);
		}
		if (led_g_brightness) {
			max77843_rgb_set_state(&max77843_rgb->led[GREEN], led_g_brightness, LED_BLINK);
		}
		if (led_b_brightness) {
			max77843_rgb_set_state(&max77843_rgb->led[BLUE], led_b_brightness, LED_BLINK);
		}
			
		break;
			
	case LOW_BATTERY:
			
		max77843_rgb_blink(dev, 500, 5000);
			
		if (led_lowpower_mode == 1)
			max77843_rgb_set_state(&max77843_rgb->led[RED], sttg_fled_low_r_gain, LED_BLINK);
		else
			max77843_rgb_set_state(&max77843_rgb->led[RED], sttg_fled_high_r_gain, LED_BLINK);
			
		break;
			
	case FULLY_CHARGED:
			
		if (led_lowpower_mode == 1) {
			if (sttg_fled_charged_r_gain && sttg_fled_low_r_gain)
				led_r_brightness = (sttg_fled_charged_r_gain * sttg_fled_low_r_gain) / 255;
			if (sttg_fled_charged_g_gain && sttg_fled_low_g_gain)
				led_g_brightness = (sttg_fled_charged_g_gain * sttg_fled_low_g_gain) / 255;
			if (sttg_fled_charged_b_gain && sttg_fled_low_b_gain)
				led_b_brightness = (sttg_fled_charged_b_gain * sttg_fled_low_b_gain) / 255;
		} else {
			if (sttg_fled_charged_r_gain && sttg_fled_low_r_gain)
				led_r_brightness = (sttg_fled_charged_r_gain * sttg_fled_high_r_gain) / 255;
			if (sttg_fled_charged_g_gain && sttg_fled_low_g_gain)
				led_g_brightness = (sttg_fled_charged_g_gain * sttg_fled_high_g_gain) / 255;
			if (sttg_fled_charged_b_gain && sttg_fled_low_b_gain)
				led_b_brightness = (sttg_fled_charged_b_gain * sttg_fled_high_b_gain) / 255;
		}
		
		if (led_r_brightness) {
			max77843_rgb_set_state(&max77843_rgb->led[RED], led_r_brightness, LED_ALWAYS_ON);
		}
		if (led_g_brightness) {
			max77843_rgb_set_state(&max77843_rgb->led[GREEN], led_g_brightness, LED_ALWAYS_ON);
		}
		if (led_b_brightness) {
			max77843_rgb_set_state(&max77843_rgb->led[BLUE], led_b_brightness, LED_ALWAYS_ON);
		}
			
		break;
			
	case POWERING:
			
		max77843_rgb_ramp(dev, 800, 800);
		max77843_rgb_blink(dev, 200, 200);
			
		if (led_lowpower_mode == 1)
			max77843_rgb_set_state(&max77843_rgb->led[BLUE], sttg_fled_low_b_gain, LED_ALWAYS_ON);
		else
			max77843_rgb_set_state(&max77843_rgb->led[BLUE], sttg_fled_high_b_gain, LED_ALWAYS_ON);

		max77843_rgb_set_state(&max77843_rgb->led[GREEN], led_dynamic_current, LED_BLINK);
			
		break;
			
	default:
		break;
	}

	return count;
}

static ssize_t store_max77843_rgb_blink(struct device *dev,
					struct device_attribute *devattr,
					const char *buf, size_t count)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	int led_brightness = 0;
	int delay_on_time = 0;
	int delay_off_time = 0;
	int ramp_delay_on_time = 0;
	int ramp_delay_off_time = 0;
	u8 led_r_brightness = 0;
	u8 led_g_brightness = 0;
	u8 led_b_brightness = 0;
	int ret;
	
	// if we're already controlling the leds,
	// OR - if we're locked out and don't want leds on, then quit now.
	if (flg_stop_incoming_leds || (pu_checkBlackout() && sttg_pu_blockleds))
		return count;
	
	pr_info("[LED/store_max77843_rgb_blink]\n");

	ret = sscanf(buf, "0x%8x %5d %5d", &led_brightness,
					&delay_on_time, &delay_off_time);
	if (ret == 0) {
		dev_err(dev, "fail to get led_blink value.\n");
		return count;
	}
	
	// override blink delays.
	if (sttg_fled_blink_on)
		delay_on_time = sttg_fled_blink_on;
	
	if (sttg_fled_blink_off)
		delay_off_time = sttg_fled_blink_off;

	/*Reset led*/
	max77843_rgb_reset(dev);

	led_r_brightness = (led_brightness & LED_R_MASK) >> 16;
	led_g_brightness = (led_brightness & LED_G_MASK) >> 8;
	led_b_brightness = led_brightness & LED_B_MASK;
	
	pr_info("[FLED] r: %d, g: %d, b: %d, dyncur: %d, lowpower: %d\n",
			led_r_brightness, led_g_brightness, led_b_brightness, led_dynamic_current, led_lowpower_mode);

	/* In user case, LED current is restricted to less than 2mA */
	
	if (led_lowpower_mode == 1) {
		led_r_brightness = (led_r_brightness * sttg_fled_low_r_gain) / 255;
		led_g_brightness = (led_g_brightness * sttg_fled_low_g_gain) / 255;
		led_b_brightness = (led_b_brightness * sttg_fled_low_b_gain) / 255;
	} else {
		led_r_brightness = (led_r_brightness * sttg_fled_high_r_gain) / 255;
		led_g_brightness = (led_g_brightness * sttg_fled_high_g_gain) / 255;
		led_b_brightness = (led_b_brightness * sttg_fled_high_b_gain) / 255;
	}
	
	pr_info("[FLED] now r: %d, g: %d, b: %d\n", led_r_brightness, led_g_brightness, led_b_brightness);
	
	/*if (led_r_brightness == led_g_brightness && led_r_brightness == led_b_brightness) {
		// neutral color.
		
		if (led_lowpower_mode == 1)
			led_b_brightness = ((led_r_brightness * sttg_fled_blueinwhite_pct) / 100);
		else
			led_b_brightness = ((led_r_brightness * sttg_fled_blueinwhite_pct) / 100);
		
		pr_info("[FLED] neutral (r: %d, g: %d), blue reduced to: %d\n", led_r_brightness, led_g_brightness, led_b_brightness);
	}*/

	if (led_r_brightness) {
		max77843_rgb_set_state(&max77843_rgb->led[RED], led_r_brightness, LED_BLINK);
	}
	if (led_g_brightness) {
		max77843_rgb_set_state(&max77843_rgb->led[GREEN], led_g_brightness, LED_BLINK);
	}
	if (led_b_brightness) {
		max77843_rgb_set_state(&max77843_rgb->led[BLUE], led_b_brightness, LED_BLINK);
	}
	
	/*Set LED blink mode*/
	
	if (sttg_fled_fade) {
		
		if (sttg_fled_ramp_on)
			ramp_delay_on_time = sttg_fled_ramp_on;
		else
			ramp_delay_on_time = delay_on_time;
		
		if (sttg_fled_ramp_off)
			ramp_delay_off_time = sttg_fled_ramp_off;
		else
			ramp_delay_off_time = delay_off_time;
		
		max77843_rgb_ramp(dev, ramp_delay_on_time, ramp_delay_off_time);
		
	} else {
		max77843_rgb_ramp(dev, 0, 0);
	}
	
	max77843_rgb_blink(dev, delay_on_time, delay_off_time);
	
	pr_info("[FLED] blink - on: %d, off: %d, rampon: %d, rampoff: %d, fademode: %d\n",
			delay_on_time, delay_off_time, ramp_delay_on_time, ramp_delay_off_time, sttg_fled_fade);

	pr_info("leds-max77843-rgb: %s\n", __func__);
	dev_dbg(dev, "led_blink is called, Color:0x%X Brightness:%i\n",
			led_brightness, led_dynamic_current);
	return count;
}

static ssize_t store_led_r(struct device *dev,
			struct device_attribute *devattr,
				const char *buf, size_t count)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	unsigned int brightness;
	char buff[10] = {0,};
	int cnt, ret;
	
	// if we're already controlling the leds,
	// OR - if we're locked out and don't want leds on, then quit now.
	if (flg_stop_incoming_leds || (pu_checkBlackout() && sttg_pu_blockleds))
		return count;
	
	pr_info("[LED/store_led_r]\n");

	cnt = count;
	cnt = (buf[cnt-1] == '\n') ? cnt-1 : cnt;
	memcpy(buff, buf, cnt);
	buff[cnt] = '\0';
	ret = kstrtouint(buff, 0, &brightness);
	if (ret != 0) {
		dev_err(dev, "fail to get brightness.\n");
		goto out;
	}
	pr_info("[FLED/red] was r: %d\n", brightness);
	
	if (led_lowpower_mode == 1)
		brightness = (brightness * sttg_fled_low_r_gain) / 255;
	else
		brightness = (brightness * sttg_fled_high_r_gain) / 255;
	
	pr_info("[FLED/red] now r: %d\n", brightness);
	if (brightness != 0) {
		max77843_rgb_set_state(&max77843_rgb->led[RED], brightness, LED_ALWAYS_ON);
	} else {
		max77843_rgb_set_state(&max77843_rgb->led[RED], LED_OFF, LED_DISABLE);
	}
out:
	pr_info("leds-max77843-rgb: %s\n", __func__);
	return count;
}
static ssize_t store_led_g(struct device *dev,
			struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	unsigned int brightness;
	char buff[10] = {0,};
	int cnt, ret;
	
	// if we're already controlling the leds,
	// OR - if we're locked out and don't want leds on, then quit now.
	if (flg_stop_incoming_leds || (pu_checkBlackout() && sttg_pu_blockleds))
		return count;
	
	pr_info("[LED/store_led_g]\n");

	cnt = count;
	cnt = (buf[cnt-1] == '\n') ? cnt-1 : cnt;
	memcpy(buff, buf, cnt);
	buff[cnt] = '\0';
	ret = kstrtouint(buff, 0, &brightness);
	if (ret != 0) {
		dev_err(dev, "fail to get brightness.\n");
		goto out;
	}
	pr_info("[FLED/green] was g: %d\n", brightness);
	
	if (led_lowpower_mode == 1)
		brightness = (brightness * sttg_fled_low_g_gain) / 255;
	else
		brightness = (brightness * sttg_fled_high_g_gain) / 255;
	
	pr_info("[FLED/green] now g: %d\n", brightness);
	if (brightness != 0) {
		max77843_rgb_set_state(&max77843_rgb->led[GREEN], brightness, LED_ALWAYS_ON);
	} else {
		max77843_rgb_set_state(&max77843_rgb->led[GREEN], LED_OFF, LED_DISABLE);
	}
out:
	pr_info("leds-max77843-rgb: %s\n", __func__);
	return count;
}
static ssize_t store_led_b(struct device *dev,
		struct device_attribute *devattr,
		const char *buf, size_t count)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	unsigned int brightness;
	char buff[10] = {0,};
	int cnt, ret;
	
	// if we're already controlling the leds,
	// OR - if we're locked out and don't want leds on, then quit now.
	if (flg_stop_incoming_leds || (pu_checkBlackout() && sttg_pu_blockleds))
		return count;
	
	pr_info("[LED/store_led_b]\n");

	cnt = count;
	cnt = (buf[cnt-1] == '\n') ? cnt-1 : cnt;
	memcpy(buff, buf, cnt);
	buff[cnt] = '\0';
	ret = kstrtouint(buff, 0, &brightness);
	if (ret != 0) {
		dev_err(dev, "fail to get brightness.\n");
		goto out;
	}
	pr_info("[FLED/blue] was b: %d\n", brightness);
	
	if (led_lowpower_mode == 1)
		brightness = (brightness * sttg_fled_low_b_gain) / 255;
	else
		brightness = (brightness * sttg_fled_high_b_gain) / 255;
	
	pr_info("[FLED/blue] now b: %d\n", brightness);
	if (brightness != 0) {
		max77843_rgb_set_state(&max77843_rgb->led[BLUE], brightness, LED_ALWAYS_ON);
	} else	{
		max77843_rgb_set_state(&max77843_rgb->led[BLUE], LED_OFF, LED_DISABLE);
	}
out:
	pr_info("leds-max77843-rgb: %s\n", __func__);
	return count;
}

static ssize_t show_fled_fade(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_fade);
}

static ssize_t store_fled_fade(struct device *dev,
								  struct device_attribute *attr,
								  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0) {
		
		if (data > 1)
			data = 1;
		
		sttg_fled_fade = data;
		pr_info("[FLED] STORE - sttg_fled_fade has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_powermode(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_powermode);
}

static ssize_t store_fled_powermode(struct device *dev,
									struct device_attribute *attr,
									const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 2) {
		
		sttg_fled_powermode = data;
		
		if (data == 1)
			led_lowpower_mode = 1;
		else if (data == 2)
			led_lowpower_mode = 0;
		
		pr_info("[FLED] STORE - sttg_fled_powermode has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_high_r_gain(struct device *dev,
									 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_high_r_gain);
}

static ssize_t store_fled_high_r_gain(struct device *dev,
									  struct device_attribute *attr,
									  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_high_r_gain = data;
		pr_info("[FLED] STORE - sttg_fled_high_r_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_high_g_gain(struct device *dev,
									 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_high_g_gain);
}

static ssize_t store_fled_high_g_gain(struct device *dev,
									  struct device_attribute *attr,
									  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_high_g_gain = data;
		pr_info("[FLED] STORE - sttg_fled_high_g_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_high_b_gain(struct device *dev,
									 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_high_b_gain);
}

static ssize_t store_fled_high_b_gain(struct device *dev,
									  struct device_attribute *attr,
									  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_high_b_gain = data;
		pr_info("[FLED] STORE - sttg_fled_high_b_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_low_r_gain(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_low_r_gain);
}

static ssize_t store_fled_low_r_gain(struct device *dev,
									 struct device_attribute *attr,
									 const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_low_r_gain = data;
		pr_info("[FLED] STORE - sttg_fled_low_r_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_low_g_gain(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_low_g_gain);
}

static ssize_t store_fled_low_g_gain(struct device *dev,
									 struct device_attribute *attr,
									 const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_low_g_gain = data;
		pr_info("[FLED] STORE - sttg_fled_low_g_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_low_b_gain(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_low_b_gain);
}

static ssize_t store_fled_low_b_gain(struct device *dev,
									 struct device_attribute *attr,
									 const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_low_b_gain = data;
		pr_info("[FLED] STORE - sttg_fled_low_b_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_charged_r_gain(struct device *dev,
									 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_charged_r_gain);
}

static ssize_t store_fled_charged_r_gain(struct device *dev,
									  struct device_attribute *attr,
									  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_charged_r_gain = data;
		pr_info("[FLED] STORE - sttg_fled_charged_r_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_charged_g_gain(struct device *dev,
									 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_charged_g_gain);
}

static ssize_t store_fled_charged_g_gain(struct device *dev,
									  struct device_attribute *attr,
									  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_charged_g_gain = data;
		pr_info("[FLED] STORE - sttg_fled_charged_g_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_charged_b_gain(struct device *dev,
									 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_charged_b_gain);
}

static ssize_t store_fled_charged_b_gain(struct device *dev,
									  struct device_attribute *attr,
									  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_charged_b_gain = data;
		pr_info("[FLED] STORE - sttg_fled_charged_b_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_charging_r_gain(struct device *dev,
										 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_charging_r_gain);
}

static ssize_t store_fled_charging_r_gain(struct device *dev,
										  struct device_attribute *attr,
										  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_charging_r_gain = data;
		pr_info("[FLED] STORE - sttg_fled_charging_r_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_charging_g_gain(struct device *dev,
										 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_charging_g_gain);
}

static ssize_t store_fled_charging_g_gain(struct device *dev,
										  struct device_attribute *attr,
										  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_charging_g_gain = data;
		pr_info("[FLED] STORE - sttg_fled_charging_g_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_charging_b_gain(struct device *dev,
										 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_charging_b_gain);
}

static ssize_t store_fled_charging_b_gain(struct device *dev,
										  struct device_attribute *attr,
										  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_charging_b_gain = data;
		pr_info("[FLED] STORE - sttg_fled_charging_b_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_missednoti_r_gain(struct device *dev,
										   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_missednoti_r_gain);
}

static ssize_t store_fled_missednoti_r_gain(struct device *dev,
									  struct device_attribute *attr,
									  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_missednoti_r_gain = data;
		pr_info("[FLED] STORE - sttg_fled_missednoti_r_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_missednoti_g_gain(struct device *dev,
										   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_missednoti_g_gain);
}

static ssize_t store_fled_missednoti_g_gain(struct device *dev,
									  struct device_attribute *attr,
									  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_missednoti_g_gain = data;
		pr_info("[FLED] STORE - sttg_fled_missednoti_g_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_missednoti_b_gain(struct device *dev,
										   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_missednoti_b_gain);
}

static ssize_t store_fled_missednoti_b_gain(struct device *dev,
									  struct device_attribute *attr,
									  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0 && data <= 255) {
		sttg_fled_missednoti_b_gain = data;
		pr_info("[FLED] STORE - sttg_fled_missednoti_b_gain has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_blink_on(struct device *dev,
								  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_blink_on);
}

static ssize_t store_fled_blink_on(struct device *dev,
								   struct device_attribute *attr,
								   const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0) {
		sttg_fled_blink_on = data;
		pr_info("[FLED] STORE - sttg_fled_blink_on has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_blink_off(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_blink_off);
}

static ssize_t store_fled_blink_off(struct device *dev,
									struct device_attribute *attr,
									const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0) {
		sttg_fled_blink_off = data;
		pr_info("[FLED] STORE - sttg_fled_blink_off has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_ramp_on(struct device *dev,
								  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_ramp_on);
}

static ssize_t store_fled_ramp_on(struct device *dev,
								  struct device_attribute *attr,
								  const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0) {
		sttg_fled_ramp_on = data;
		pr_info("[FLED] STORE - sttg_fled_ramp_on has been set to: %d\n", data);
	}
	
	return count;
}

static ssize_t show_fled_ramp_off(struct device *dev,
								  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sttg_fled_ramp_off);
}

static ssize_t store_fled_ramp_off(struct device *dev,
								   struct device_attribute *attr,
								   const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if (ret && data >= 0) {
		sttg_fled_ramp_off = data;
		pr_info("[FLED] STORE - sttg_fled_ramp_off has been set to: %d\n", data);
	}
	
	return count;
}

/* Added for led common class */
static ssize_t led_delay_on_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", max77843_rgb->delay_on_times_ms);
}

static ssize_t led_delay_on_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	unsigned int time;

	if (kstrtouint(buf, 0, &time)) {
		dev_err(dev, "can not write led_delay_on\n");
		return count;
	}

	max77843_rgb->delay_on_times_ms = time;

	return count;
}

static ssize_t led_delay_off_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", max77843_rgb->delay_off_times_ms);
}

static ssize_t led_delay_off_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	unsigned int time;

	if (kstrtouint(buf, 0, &time)) {
		dev_err(dev, "can not write led_delay_off\n");
		return count;
	}

	max77843_rgb->delay_off_times_ms = time;

	return count;
}

static ssize_t led_blink_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	const struct device *parent = dev->parent;
	struct max77843_rgb *max77843_rgb_num = dev_get_drvdata(parent);
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	unsigned int blink_set;
	int n = 0;
	int i;
	
	// if we're already controlling the leds,
	// OR - if we're locked out and don't want leds on, then quit now.
	if (flg_stop_incoming_leds || (pu_checkBlackout() && sttg_pu_blockleds))
		return count;
	
	pr_info("[LED/led_blink_store]\n");

	if (!sscanf(buf, "%1d", &blink_set)) {
		dev_err(dev, "can not write led_blink\n");
		return count;
	}

	if (!blink_set) {
		max77843_rgb->delay_on_times_ms = LED_OFF;
		max77843_rgb->delay_off_times_ms = LED_OFF;
	}

	for (i = 0; i < 4; i++) {
		if (dev == max77843_rgb_num->led[i].dev)
			n = i;
	}
	
	if (sttg_fled_fade) {
		max77843_rgb_ramp(max77843_rgb_num->led[n].dev->parent,
						   max77843_rgb->delay_on_times_ms,
						   max77843_rgb->delay_off_times_ms);
	}

	max77843_rgb_blink(max77843_rgb_num->led[n].dev->parent,
		max77843_rgb->delay_on_times_ms,
		max77843_rgb->delay_off_times_ms);
	max77843_rgb_set_state(&max77843_rgb_num->led[n], led_dynamic_current, LED_BLINK);

	pr_info("leds-max77843-rgb: %s\n", __func__);
	return count;
}

/* permission for sysfs node */
static DEVICE_ATTR(delay_on, 0640, led_delay_on_show, led_delay_on_store);
static DEVICE_ATTR(delay_off, 0640, led_delay_off_show, led_delay_off_store);
static DEVICE_ATTR(blink, 0640, NULL, led_blink_store);

#ifdef SEC_LED_SPECIFIC
/* below nodes is SAMSUNG specific nodes */
static DEVICE_ATTR(led_r, 0660, NULL, store_led_r);
static DEVICE_ATTR(led_g, 0660, NULL, store_led_g);
static DEVICE_ATTR(led_b, 0660, NULL, store_led_b);
/* led_pattern node permission is 222 */
/* To access sysfs node from other groups */
static DEVICE_ATTR(led_pattern, 0660, NULL, store_max77843_rgb_pattern);
static DEVICE_ATTR(led_blink, 0660, NULL,  store_max77843_rgb_blink);
static DEVICE_ATTR(led_brightness, 0660, NULL, store_max77843_rgb_brightness);
static DEVICE_ATTR(led_lowpower, 0660, show_max77843_rgb_lowpower, store_max77843_rgb_lowpower);
static DEVICE_ATTR(fled_fade, 0660, show_fled_fade, store_fled_fade);
static DEVICE_ATTR(fled_powermode, 0660, show_fled_powermode, store_fled_powermode);
static DEVICE_ATTR(fled_high_r_gain, 0660, show_fled_high_r_gain, store_fled_high_r_gain);
static DEVICE_ATTR(fled_high_g_gain, 0660, show_fled_high_g_gain, store_fled_high_g_gain);
static DEVICE_ATTR(fled_high_b_gain, 0660, show_fled_high_b_gain, store_fled_high_b_gain);
static DEVICE_ATTR(fled_low_r_gain, 0660, show_fled_low_r_gain, store_fled_low_r_gain);
static DEVICE_ATTR(fled_low_g_gain, 0660, show_fled_low_g_gain, store_fled_low_g_gain);
static DEVICE_ATTR(fled_low_b_gain, 0660, show_fled_low_b_gain, store_fled_low_b_gain);
static DEVICE_ATTR(fled_charged_r_gain, 0660, show_fled_charged_r_gain, store_fled_charged_r_gain);
static DEVICE_ATTR(fled_charged_g_gain, 0660, show_fled_charged_g_gain, store_fled_charged_g_gain);
static DEVICE_ATTR(fled_charged_b_gain, 0660, show_fled_charged_b_gain, store_fled_charged_b_gain);
static DEVICE_ATTR(fled_charging_r_gain, 0660, show_fled_charging_r_gain, store_fled_charging_r_gain);
static DEVICE_ATTR(fled_charging_g_gain, 0660, show_fled_charging_g_gain, store_fled_charging_g_gain);
static DEVICE_ATTR(fled_charging_b_gain, 0660, show_fled_charging_b_gain, store_fled_charging_b_gain);
static DEVICE_ATTR(fled_missednoti_r_gain, 0660, show_fled_missednoti_r_gain, store_fled_missednoti_r_gain);
static DEVICE_ATTR(fled_missednoti_g_gain, 0660, show_fled_missednoti_g_gain, store_fled_missednoti_g_gain);
static DEVICE_ATTR(fled_missednoti_b_gain, 0660, show_fled_missednoti_b_gain, store_fled_missednoti_b_gain);
static DEVICE_ATTR(fled_blink_on, 0660, show_fled_blink_on, store_fled_blink_on);
static DEVICE_ATTR(fled_blink_off, 0660, show_fled_blink_off, store_fled_blink_off);
static DEVICE_ATTR(fled_ramp_on, 0660, show_fled_ramp_on, store_fled_ramp_on);
static DEVICE_ATTR(fled_ramp_off, 0660, show_fled_ramp_off, store_fled_ramp_off);
#endif

static struct attribute *led_class_attrs[] = {
	&dev_attr_delay_on.attr,
	&dev_attr_delay_off.attr,
	&dev_attr_blink.attr,
	NULL,
};

static struct attribute_group common_led_attr_group = {
	.attrs = led_class_attrs,
};

#ifdef SEC_LED_SPECIFIC
static struct attribute *sec_led_attributes[] = {
	&dev_attr_led_r.attr,
	&dev_attr_led_g.attr,
	&dev_attr_led_b.attr,
	&dev_attr_led_pattern.attr,
	&dev_attr_led_blink.attr,
	&dev_attr_led_brightness.attr,
	&dev_attr_led_lowpower.attr,
	&dev_attr_fled_fade.attr,
	&dev_attr_fled_powermode.attr,
	&dev_attr_fled_high_r_gain.attr,
	&dev_attr_fled_high_g_gain.attr,
	&dev_attr_fled_high_b_gain.attr,
	&dev_attr_fled_low_r_gain.attr,
	&dev_attr_fled_low_g_gain.attr,
	&dev_attr_fled_low_b_gain.attr,
	&dev_attr_fled_charged_r_gain.attr,
	&dev_attr_fled_charged_g_gain.attr,
	&dev_attr_fled_charged_b_gain.attr,
	&dev_attr_fled_charging_r_gain.attr,
	&dev_attr_fled_charging_g_gain.attr,
	&dev_attr_fled_charging_b_gain.attr,
	&dev_attr_fled_missednoti_r_gain.attr,
	&dev_attr_fled_missednoti_g_gain.attr,
	&dev_attr_fled_missednoti_b_gain.attr,
	&dev_attr_fled_blink_on.attr,
	&dev_attr_fled_blink_off.attr,
	&dev_attr_fled_ramp_on.attr,
	&dev_attr_fled_ramp_off.attr,
	NULL,
};

static struct attribute_group sec_led_attr_group = {
	.attrs = sec_led_attributes,
};
#endif
static int max77843_rgb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max77843_rgb_platform_data *pdata;
	struct max77843_rgb *max77843_rgb;
	struct max77843_dev *max77843_dev = dev_get_drvdata(dev->parent);
	char temp_name[4][40] = {{0,},}, name[40] = {0,}, *p;
	int i, ret;

	pr_info("leds-max77843-rgb: %s\n", __func__);

#ifdef CONFIG_OF
	pdata = max77843_rgb_parse_dt(dev);
	if (unlikely(IS_ERR(pdata)))
		return PTR_ERR(pdata);
#else
	pdata = dev_get_platdata(dev);
#endif

	max77843_rgb = devm_kzalloc(dev, sizeof(struct max77843_rgb), GFP_KERNEL);
	if (unlikely(!max77843_rgb))
		return -ENOMEM;
	pr_info("leds-max77843-rgb: %s 1 \n", __func__);

	max77843_rgb->i2c = max77843_dev->i2c;

	for (i = 0; i < 4; i++) {
		ret = snprintf(name, 30, "%s", pdata->name[i])+1;
		if (1 > ret)
			goto alloc_err_flash;

		p = devm_kzalloc(dev, ret, GFP_KERNEL);
		if (unlikely(!p))
			goto alloc_err_flash;

		strcpy(p, name);
		strcpy(temp_name[i], name);
		max77843_rgb->led[i].name = p;
		max77843_rgb->led[i].brightness_set = max77843_rgb_set;
		max77843_rgb->led[i].brightness_get = max77843_rgb_get;
		max77843_rgb->led[i].max_brightness = LED_MAX_CURRENT;

		ret = led_classdev_register(dev, &max77843_rgb->led[i]);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "unable to register RGB : %d\n", ret);
			goto alloc_err_flash_plus;
		}
		ret = sysfs_create_group(&max77843_rgb->led[i].dev->kobj,
						&common_led_attr_group);
		if (ret < 0) {
			dev_err(dev, "can not register sysfs attribute\n");
			goto register_err_flash;
		}
	}

	led_dev = device_create(sec_class, NULL, 0, max77843_rgb, "led");
	if (IS_ERR(led_dev)) {
		dev_err(dev, "Failed to create device for samsung specific led\n");
		goto alloc_err_flash;
	}


	ret = sysfs_create_group(&led_dev->kobj, &sec_led_attr_group);
	if (ret < 0) {
		dev_err(dev, "Failed to create sysfs group for samsung specific led\n");
		goto alloc_err_flash;
	}

	platform_set_drvdata(pdev, max77843_rgb);

#if defined (CONFIG_SEC_FACTORY)
#if defined(CONFIG_SEC_TRLTE_PROJECT) || defined(CONFIG_SEC_TBLTE_PROJECT)
	if ( (jig_val == 0) && (get_lcd_attached() == 0) ) {
		pr_info("%s:Factory MODE - No OCTA, Battery BOOTING\n", __func__);
		max77843_rgb_set_state(&max77843_rgb->led[RED], BASE_DYNAMIC_LED_CURRENT, LED_ALWAYS_ON);
	}

#endif
#endif
	
	wq_controlFrontLED = alloc_workqueue("controlFrontLED_wq", WQ_HIGHPRI, 0);
	
	INIT_WORK(&work_controlFrontLED, controlFrontLED_work);
	
	setup_timer(&timer_alternatefrontled, timerhandler_alternatefrontled, 0);
	
	wake_lock_init(&fled_wake_lock, WAKE_LOCK_SUSPEND, "wake_plasma_fled");

	pr_info("leds-max77843-rgb: %s done\n", __func__);

	return 0;

register_err_flash:
	led_classdev_unregister(&max77843_rgb->led[i]);
alloc_err_flash_plus:
	devm_kfree(dev, temp_name[i]);
alloc_err_flash:
	while (i--) {
		led_classdev_unregister(&max77843_rgb->led[i]);
		devm_kfree(dev, temp_name[i]);
	}
	devm_kfree(dev, max77843_rgb);
	return -ENOMEM;
}

static int max77843_rgb_remove(struct platform_device *pdev)
{
	struct max77843_rgb *max77843_rgb = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < 4; i++)
		led_classdev_unregister(&max77843_rgb->led[i]);

	return 0;
}

static void max77843_rgb_shutdown(struct device *dev)
{
	struct max77843_rgb *max77843_rgb = dev_get_drvdata(dev);
	int i;

	if (!max77843_rgb->i2c)
		return;

	max77843_rgb_reset(dev);

	sysfs_remove_group(&led_dev->kobj, &sec_led_attr_group);

	for (i = 0; i < 4; i++){
		sysfs_remove_group(&max77843_rgb->led[i].dev->kobj,
						&common_led_attr_group);
		led_classdev_unregister(&max77843_rgb->led[i]);
	}
	devm_kfree(dev, max77843_rgb);
}
static struct platform_driver max77843_fled_driver = {
	.driver		= {
		.name	= "leds-max77843-rgb",
		.owner	= THIS_MODULE,
		.shutdown = max77843_rgb_shutdown,
	},
	.probe		= max77843_rgb_probe,
	.remove		= max77843_rgb_remove,
};

static int __init max77843_rgb_init(void)
{
	pr_info("leds-max77843-rgb: %s\n", __func__);
	return platform_driver_register(&max77843_fled_driver);
}
module_init(max77843_rgb_init);

static void __exit max77843_rgb_exit(void)
{
	platform_driver_unregister(&max77843_fled_driver);
}
module_exit(max77843_rgb_exit);

MODULE_ALIAS("platform:max77843-rgb");
MODULE_AUTHOR("Jeongwoong Lee<jell.lee@samsung.com>");
MODULE_DESCRIPTION("MAX77843 RGB driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
