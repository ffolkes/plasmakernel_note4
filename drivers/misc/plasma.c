/*
 * drivers/misc/plasma.c
 * A central implementation of various Plasma kernel features.
 *
 *
 * Copyright (c) 2014, ffolkes <ffolkess@gmail.com>
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/plasma.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/sched.h>
#ifdef CONFIG_POWERSUSPEND
#include <linux/powersuspend.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

// definitions.
#define DRIVER_AUTHOR "ffolkes <ffolkess@gmail.com>"
#define DRIVER_DESCRIPTION "Plasma kernel features"
#define DRIVER_VERSION "1.0"
#define LOGTAG "[plasma"

#define SCREEN_X_MAX		1440
#define SCREEN_Y_MAX		2560

// externals.
extern void zzmoove_boost(unsigned int screen_state, unsigned int max_cycles, unsigned int mid_cycles, unsigned int allcores_cycles, unsigned int input_cycles);
extern bool flg_power_suspended;
extern struct timeval time_power_suspended;
extern struct timeval time_pressed_power;
extern bool flg_tsp_always_on;

// variables.
static struct input_dev * vk_dev;
int plasmatest = 0;

//// touchwake.
static unsigned int sttg_tw_timeout = 0;  // 0 = disabled, >1 = milliseconds
static bool flg_tw_expired = true;
static bool flg_tw_skip = false;
static void tw_timeout_work(struct work_struct * work_tw_timeout);
static DECLARE_DELAYED_WORK(work_tw_timeout, tw_timeout_work);

//// generic tsp.
static int touch_x_start = -1;
static int touch_y_start = -1;
static int touch_w_start = -1;
static bool flg_onefinger = true;

//// swipe2wake.
static bool sttg_s2w_mode = false;
static int s2w_y_min = 1800;
static int s2w_y_max = SCREEN_Y_MAX;
static int s2w_start_x_width = 250;
static int s2w_end_x_min = 800;
static bool flg_s2w_inprogress = false;

//// arc2wake.
static bool sttg_a2w_mode = false;
static int a2w_start_x_min = 530;
static int a2w_start_x_max = 930;
static int a2w_start_y_height = 200;
static int a2w_end_y_min = 1350;
static int a2w_end_y_max = 1850;
static int a2w_end_x_width = 350;
static bool flg_a2w_inprogress = false;

//// sensors.
static bool flg_sensor_prox_detecting = false;

//// voice.
bool sttg_voice_enableturnoff = false;

static unsigned int vk_pressed_keycode = 0;
struct timeval time_vk_last_pressed;

DECLARE_WAIT_QUEUE_HEAD(vk_wq);

// work.
static void tw_timeout_work(struct work_struct * work_hu_precheck)
{
	
	pr_info(LOGTAG"/tw_timeout_work] touchwake - timed out\n");
	flg_tw_expired = true;
	return;
}

// functions.
void press_power(void)
{
	// if we're awake and turning off, don't trigger touchwake.
	if (!flg_power_suspended) {
		flg_tw_skip = true;
	}
	
	input_event(vk_dev, EV_KEY, KEY_POWER, 1);
	input_sync(vk_dev);
	msleep(10);
	input_event(vk_dev, EV_KEY, KEY_POWER, 0);
	input_sync(vk_dev);
}

void vk_press_button(int keycode, bool delayed, bool force, bool elastic, bool powerfirst)
{
	unsigned int delay = 10; // set default delay between down/up press
	
	// "force" mode will ignore practice_mode/block, so other projects
	// can use it regardless of tsp state.
	
	if (keycode == 0) {
		pr_info(LOGTAG"/vk] keycode was 0, aborting!\n");
		return;
	}
	
	/*if (!force && (wpmk_practice_mode || wpmk_block_touch)) {
		pr_info(LOGTAG"/vk] press blocked or practice mode enabled, aborting! wpmk_practice_mode: %d, wpmk_block_touch: %d, ctr: %d\n",
				wpmk_practice_mode, wpmk_block_touch, flg_ctr_tsp_touching);
		return;
	}*/
	
	// save the time.
	do_gettimeofday(&time_vk_last_pressed);
	
	//flg_elastic_mode = elastic;
	
	if (flg_power_suspended && keycode != KEY_POWER && powerfirst) {
		pr_info(LOGTAG"/vk] POWERFIRST requested. turning on...\n");
		press_power();
	}
	
	if (keycode >= 1000) {
		// this is a special mode that communicates with a userspace script.
		
		if (keycode >= 9000) {
			// no boost for special keycodes.
			
			// boost to max for 10 cycles, mid for 20.
			zzmoove_boost(0, 10, 20, 0, 0);
		}
		
		vk_pressed_keycode = keycode;
		wake_up_interruptible_all(&vk_wq);
		
		pr_info(LOGTAG"/vk] VIRTUALPRESS_%d\n", keycode);
		return;
	}
	
	pr_info(LOGTAG"/vk] PRESS - pressing keycode: %d with %d ms delay\n", keycode, delay);
	
	if (keycode >= 900 && keycode < 1000) {
		// send dummy virtualpress to userspace so it'll vibrate.
		
		vk_pressed_keycode = 1998;
		wake_up_interruptible_all(&vk_wq);
		
		pr_info(LOGTAG"/vk] VIRTUALPRESS_1998 VIBRATE\n");
	}
	
	if (keycode == 900) {
		
		//mdnie_toggle_negative();
		return;
		
	} else if (keycode == 901) {
		
		//mdnie_toggle_nightmode();
		return;
		
	} else if (keycode == 902) {
		
		//mdnie_toggle_graymode();
		return;
		
	} else if (keycode == 903) {
		
		//mdnie_toggle_blackout();
		return;
		
	} else if (keycode == 910) {
		
		//toggleRearLED(1);
		return;
		
	} else if (keycode == 911) {
		
		//toggleRearLED(7);
		return;
		
	} else if (keycode == 912) {
		
		//toggleRearLED(15);
		return;
		
	} else if (keycode == 920) {
		
		//input_report_key(input_dev_tk, 158, 0);
		//input_sync(input_dev_tk);
		return;
		
	} else if (keycode == 998) {
		
		// do nothing, dummy keycode so it'll vibrate.
		return;
		
	}
	
	// boost to max for 10 cycles.
	zzmoove_boost(0, 10, 0, 0, 0);
	
	if (delayed) {
		delay = 700;
	}
	
	// if the user is wanting to press power, better stick with the reliable way (gpio device)
	// that doesn't require a modified /system/usr/keylayout/sec_touchkey.kl file.
	if (keycode == KEY_POWER) {
		press_power();
		return;
	}
	
	input_report_key(vk_dev, keycode, 1);
	input_sync(vk_dev);
	msleep(delay);
	input_report_key(vk_dev, keycode, 0);
	input_sync(vk_dev);
}
EXPORT_SYMBOL(vk_press_button);

// utilities.
int do_timesince(struct timeval time_start)
{
	struct timeval time_now;
	int timesince = 0;
	
	do_gettimeofday(&time_now);
	
	timesince = (time_now.tv_sec - time_start.tv_sec) * MSEC_PER_SEC +
				(time_now.tv_usec - time_start.tv_usec) / USEC_PER_MSEC;
	
	return timesince;
}

void plasma_process_tsp_reset(void)
{
	pr_info(LOGTAG"/plasma_process_tsp_reset] - reset\n");
}
EXPORT_SYMBOL(plasma_process_tsp_reset);

void plasma_process_tsp_touch_enter(int finger, int touchcount)
{
	//pr_info(LOGTAG"/plasma_process_tsp_touch_enter] - finger: %d\n", finger);
	
	if (finger > 0)
		flg_onefinger = false;
	
	if (flg_power_suspended) {
		// the screen is off.
		
		if (!flg_tw_expired && sttg_tw_timeout > 0 && do_timesince(time_power_suspended) < sttg_tw_timeout) {
			zzmoove_boost(0, 10, 0, 10, 0);
			press_power();
			pr_info(LOGTAG"/plasma_process_tsp_touch_enter] - touchwake tap detected, waking\n");
		}
	}
}
EXPORT_SYMBOL(plasma_process_tsp_touch_enter);

int plasma_process_tsp_touch_move(int finger, int x, int y, int pressure)
{
	pr_info(LOGTAG"/plasma_process_tsp_touch_move] - finger: %d, x: %d, y: %d, pressure: %d\n", finger, x, y, pressure);
	
	// only use finger 0.
	if (flg_onefinger && finger == 0) {
		
		// initialize the swipe if needed.
		if (touch_x_start < 0) {
			// this is a new event.
			
			touch_x_start = x;
			touch_y_start = y;
			touch_w_start = pressure;
			
			pr_info(LOGTAG"/plasma_process_tsp_touch_move/init] - touch event initialized - x: %d, y: %d, pressure: %d\n", x, y, pressure);
		}
		
		// screen off stuff.
		if (flg_power_suspended) {
			
			// arc2wake.
			if (sttg_a2w_mode) {
				
				if (!flg_a2w_inprogress
					&& y > (SCREEN_Y_MAX - a2w_start_y_height)
					&& x > a2w_start_x_min
					&& x < a2w_start_x_max) {
					// this is a valid arc starting point.
					
					// check to make sure this is the first input event for this swipe.
					if (x == touch_x_start && y == touch_y_start) {
						flg_a2w_inprogress = true;
						pr_info(LOGTAG"/plasma_process_tsp_touch_move/a2w] ARC STARTED - x: %d, y: %d\n", x, y);
					}
					
				} else if (flg_a2w_inprogress
						   && (x > (SCREEN_X_MAX - a2w_end_x_width) || x < (a2w_end_x_width))  // x for right or left
						   && y > a2w_end_y_min
						   && y < a2w_end_y_max) {
					// this is a valid arc ending point.
					
					pr_info(LOGTAG"/plasma_process_tsp_touch_move/a2w] ARC ENDED - x: %d, y: %d\n", x, y);
					
					// boost to max for 5 cycles, allcores for 5.
					zzmoove_boost(0, 5, 0, 5, 0);
					
					// turn on.
					press_power();
				}
			}
			
			// swipe2wake.
			if (sttg_s2w_mode && !flg_a2w_inprogress) {
				
				if (!flg_s2w_inprogress
					&& x < s2w_start_x_width
					&& y > s2w_y_min
					&& y < s2w_y_max) {
					// this is a valid swipe starting point.
					
					flg_s2w_inprogress = true;
					pr_info(LOGTAG"/plasma_process_tsp_touch_move/s2w] SWIPE STARTED - x: %d, y: %d\n", x, y);
					
				} else if (flg_s2w_inprogress
						   && x > s2w_end_x_min
						   && y > s2w_y_min
						   && y < s2w_y_max) {
					// this is a valid swipe ending point.
					
					pr_info(LOGTAG"/plasma_process_tsp_touch_move/s2w] SWIPE ENDED - x: %d, y: %d\n", x, y);
					
					// boost to max for 5 cycles, allcores for 5.
					zzmoove_boost(0, 5, 0, 5, 0);
					
					// turn on.
					press_power();
					
				} else if (flg_s2w_inprogress
						   && (y < s2w_y_min || y > s2w_y_max)) {
					// swipe has gone too high or low, invalidate.
					
					flg_s2w_inprogress = false;
					pr_info(LOGTAG"/plasma_process_tsp_touch_move/s2w] SWIPE FAILED - x: %d, y: %d\n", x, y);
				}
			}
		}
	}
	
	return 1;
}
EXPORT_SYMBOL(plasma_process_tsp_touch_move);

void plasma_process_tsp_touch_exit(int finger, int touchcount)
{
	pr_info(LOGTAG"/plasma_process_tsp_touch_exit] - finger: %d, touchcount: %d\n", finger, touchcount);
	
	if (touchcount == 0) {
		
		// reset stuff.
		touch_x_start = -1;
		touch_y_start = -1;
		touch_w_start = -1;
		flg_s2w_inprogress = false;
		flg_a2w_inprogress = false;
		flg_onefinger = true;
	}
}
EXPORT_SYMBOL(plasma_process_tsp_touch_exit);

// suspend/resume.
static void plasma_power_suspend(struct power_suspend *h) {
	// suspend.
	pr_info(LOGTAG"] SUSPEND - \n");
	
	// check to see if we should let the tsp sleep.
	if (sttg_tw_timeout || sttg_s2w_mode || sttg_a2w_mode) {
		flg_tsp_always_on = true;
	} else {
		flg_tsp_always_on = false;
	}
	
	// touchwake.
	if (sttg_tw_timeout > 0 && !flg_sensor_prox_detecting && !flg_tw_skip) {
		// schedule work to turn touchwake off, only if prox isn't detecting anything (ie. in-call).
		
		if (do_timesince(time_pressed_power) < 1200) {
			// power was pressed less than 1200ms ago, assume it was turned off intentionally.
			flg_tw_expired = true;
		} else {
			flg_tw_expired = false;
			pr_info(LOGTAG"/SUSPEND] touchwake - schdeduling work in %d ms\n", sttg_tw_timeout);
			schedule_delayed_work(&work_tw_timeout, msecs_to_jiffies(sttg_tw_timeout));
		}
	}
}

static void plasma_power_resume(struct power_suspend *h) {
	// resume.
	pr_info(LOGTAG"/RESUME]\n");
	
	// cancel work.
	cancel_delayed_work_sync(&work_tw_timeout);
	
	// reset things.
	flg_s2w_inprogress = false;
	flg_a2w_inprogress = false;
	flg_tw_skip = false;
}

static struct power_suspend plasma_power_suspend_handler = {
	.suspend = plasma_power_suspend,
	.resume = plasma_power_resume,
};

// sensor stuff.
void plasma_sensor_prox_report(unsigned int detected)
{
	if (detected) {
		pr_info(LOGTAG"/plasma_sensor_prox_report] prox covered\n");
		flg_sensor_prox_detecting = true;
		
	} else {
		flg_sensor_prox_detecting = false;
		pr_info(LOGTAG"/plasma_sensor_prox_report] prox uncovered\n");
	}
}
EXPORT_SYMBOL(plasma_sensor_prox_report);

// sysfs stuff.
static ssize_t plasmatest_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", plasmatest);
}

static ssize_t plasmatest_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	// this is just a dummy sysfs to make triggering experiments easier.
	
	if(ret && data >= 0) {
		if (data == 2) {
			vk_press_button(1001, 0, 0, 0, 0);
		}
		if (data == 3) {
			pr_info(LOGTAG"] menu\n");
			vk_press_button(KEY_MENU, 0, 0, 0, 0);
		}
		pr_info(LOGTAG"] STORE - plasmatest has been set to: %d\n", data);
	}
	
	return size;
}

static ssize_t tw_timeout_show(struct device *dev,
							   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sttg_tw_timeout);
}

static ssize_t tw_timeout_store(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data >= 0) {
		sttg_tw_timeout = data;
		pr_info(LOGTAG"] STORE - sttg_tw_timeout has been set to: %d\n", data);
	}
	
	return size;
}

//// swipe2wake.
static ssize_t s2w_mode_show(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sttg_s2w_mode);
}

static ssize_t s2w_mode_store(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data >= 0) {
		
		if (data > 0)
			data = 1;
		
		sttg_s2w_mode = data;
		pr_info(LOGTAG"] STORE - sttg_s2w_mode has been set to: %d\n", data);
	}
	
	return size;
}

//// arc2wake.
static ssize_t a2w_mode_show(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sttg_a2w_mode);
}

static ssize_t a2w_mode_store(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data >= 0) {
		
		if (data > 0)
			data = 1;
		
		sttg_a2w_mode = data;
		pr_info(LOGTAG"] STORE - sttg_a2w_mode has been set to: %d\n", data);
	}
	
	return size;
}

//// voice.
static ssize_t voice_enableturnoff_show(struct device *dev,
							   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sttg_voice_enableturnoff);
}

static ssize_t voice_enableturnoff_store(struct device *dev,
										 struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int ret;
	unsigned int data;
	
	ret = sscanf(buf, "%u\n", &data);
	
	if(ret && data >= 0) {
		sttg_voice_enableturnoff = data;
		pr_info(LOGTAG"] STORE - sttg_voice_enableturnoff has been set to: %d\n", data);
	}
	
	return size;
}

static ssize_t vk_wait_for_virtualpress_show(struct device *dev,
											 struct device_attribute *attr, char *buf)
{
	int ret;
	char *s;
	
	ret = wait_event_interruptible(vk_wq, vk_pressed_keycode);
	
	if (ret)
		return ret; // interrupted
	
	s = buf;
	
	if (vk_pressed_keycode) {
		
		s += sprintf(buf, "%d", vk_pressed_keycode);
		vk_pressed_keycode = 0;
		return s - buf;
	}
	
	s += sprintf(buf, "0");
	return s - buf;
}

static DEVICE_ATTR(plasmatest, S_IWUSR | S_IRUGO, plasmatest_show, plasmatest_store);
static DEVICE_ATTR(tw_timeout, S_IWUSR | S_IRUGO, tw_timeout_show, tw_timeout_store);
static DEVICE_ATTR(s2w_mode, S_IWUSR | S_IRUGO, s2w_mode_show, s2w_mode_store);
static DEVICE_ATTR(a2w_mode, S_IWUSR | S_IRUGO, a2w_mode_show, a2w_mode_store);
static DEVICE_ATTR(voice_enableturnoff, S_IWUSR | S_IRUGO, voice_enableturnoff_show, voice_enableturnoff_store);
static DEVICE_ATTR(vk_wait_for_virtualpress, S_IRUGO | S_IWUSR, vk_wait_for_virtualpress_show, NULL);

static struct attribute *plasma_attrs[] = {
	&dev_attr_plasmatest.attr,
	&dev_attr_tw_timeout.attr,
	&dev_attr_s2w_mode.attr,
	&dev_attr_a2w_mode.attr,
	&dev_attr_voice_enableturnoff.attr,
	&dev_attr_vk_wait_for_virtualpress.attr,
	NULL,
};

static struct attribute_group plasma_attr_group = {
	.attrs = plasma_attrs,
};

// init stuff.
struct kobject *plasma_kobj;
EXPORT_SYMBOL_GPL(plasma_kobj);

static int __init plasma_init(void)
{
	int rc = 0;

	vk_dev = input_allocate_device();
	if (!vk_dev) {
		pr_err("Can't allocate plasma input dev\n");
		goto err_alloc_dev;
	}

	input_set_capability(vk_dev, EV_KEY, KEY_POWER);
	input_set_capability(vk_dev, EV_KEY, KEY_MENU);
	vk_dev->name = "plasma_vk";
	vk_dev->phys = "plasma_vk/input0";

	rc = input_register_device(vk_dev);
	if (rc) {
		pr_err("%s: input_register_device err=%d\n", __func__, rc);
		goto err_input_dev;
	}
	
	plasma_kobj = kobject_create_and_add("plasma", NULL);
	if (plasma_kobj == NULL) {
		pr_warn("%s: plasma_kobj create_and_add failed\n", __func__);
	}
	
	rc = sysfs_create_group(plasma_kobj, &plasma_attr_group);
	if (rc) {
		pr_warn("%s: Unable to create sysfs group, error: %d\n", __func__, rc);
	}
	
    /*rc = sysfs_create_file(plasma_kobj, &dev_attr_plasmatest.attr);
    if (rc) {
        pr_warn("%s: sysfs_create_file failed for plasmatest\n", __func__);
    }*/

#ifdef CONFIG_POWERSUSPEND
	register_power_suspend(&plasma_power_suspend_handler);
#endif

err_input_dev:
	input_free_device(vk_dev);
err_alloc_dev:
	pr_info(LOGTAG"%s done\n", __func__);

	return 0;
}

static void __exit plasma_exit(void)
{
    kobject_del(plasma_kobj);
#ifdef CONFIG_POWERSUSPEND
	unregister_power_suspend(&plasma_power_suspend_handler);
#endif
	input_unregister_device(vk_dev);
	input_free_device(vk_dev);
	return;
}

module_init(plasma_init);
module_exit(plasma_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_AUTHOR("ffolkes <ffolkess@gmail.com>");
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");
