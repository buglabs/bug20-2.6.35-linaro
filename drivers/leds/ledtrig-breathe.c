/*
 * LED Breathe Trigger
 *
 * Copyright (C) 2006 Atsushi Nemoto <anemo@mba.ocn.ne.jp>
 *
 * Based on Richard Purdie's ledtrig-timer.c and some arch's
 * CONFIG_BREATHE code.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/leds.h>
#include "leds.h"

struct breathe_trig_data {
	unsigned int phase;
	unsigned int brightness;
	struct timer_list timer;
};

static void led_breathe_function(unsigned long data)
{
	struct led_classdev *led_cdev = (struct led_classdev *) data;
	struct breathe_trig_data *breathe_data = led_cdev->trigger_data;
	
	/*
	 * Increments brightness from 2 -> LED_FULL, then
	 * from LED_HALF back to 2.
	 */

	if (breathe_data->brightness >= LED_FULL) {
	    breathe_data->phase = 1;
	  }
	else if (breathe_data->brightness <= 2) {
	    breathe_data->phase = 0;
	  }

	switch (breathe_data->phase) {
	case 0: // 'going up'
	        breathe_data->brightness++;
		break;
	case 1: // 'going down'
	        breathe_data->brightness--;
		break;
	}

	led_set_brightness(led_cdev, breathe_data->brightness);
	mod_timer(&breathe_data->timer, jiffies + msecs_to_jiffies(10));
}

static void breathe_trig_activate(struct led_classdev *led_cdev)
{
	struct breathe_trig_data *breathe_data;

	breathe_data = kzalloc(sizeof(*breathe_data), GFP_KERNEL);
	if (!breathe_data)
		return;

	led_cdev->trigger_data = breathe_data;
	setup_timer(&breathe_data->timer,
		    led_breathe_function, (unsigned long) led_cdev);
	breathe_data->phase = 0;
	breathe_data->brightness = 4;
	led_breathe_function(breathe_data->timer.data);
}

static void breathe_trig_deactivate(struct led_classdev *led_cdev)
{
	struct breathe_trig_data *breathe_data = led_cdev->trigger_data;

	if (breathe_data) {
		del_timer_sync(&breathe_data->timer);
		kfree(breathe_data);
	}
}

static struct led_trigger breathe_led_trigger = {
	.name     = "breathe",
	.activate = breathe_trig_activate,
	.deactivate = breathe_trig_deactivate,
};

static int __init breathe_trig_init(void)
{
	return led_trigger_register(&breathe_led_trigger);
}

static void __exit breathe_trig_exit(void)
{
	led_trigger_unregister(&breathe_led_trigger);
}

module_init(breathe_trig_init);
module_exit(breathe_trig_exit);

MODULE_AUTHOR("Atsushi Nemoto <anemo@mba.ocn.ne.jp>");
MODULE_DESCRIPTION("Breathe LED trigger");
MODULE_LICENSE("GPL");
