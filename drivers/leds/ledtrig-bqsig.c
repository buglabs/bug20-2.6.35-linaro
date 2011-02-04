/*
 * drivers/misc/bug_batt_low.c
 *
 * Battery status triggers for OMAP3 BUGBASE
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/bmi.h>
#include <linux/leds.h>

#include <plat/hardware.h>
#include <plat/gpio.h>
#include <plat/mux.h>
#include "leds.h"

#define BATT_LOW_GPIO      64

struct bqsig_device {
        struct platform_device  *pdev;
        struct work_struct       irq_handler_work;
	struct led_trigger	*batt_low_trig;
	struct led_trigger	*batt_normal_trig;
};

static struct platform_device *bug_bqsig_dev;

/*
 *    work queuing
 */

static inline struct bqsig_device *irq_handler_work_to_bqsig_device(struct work_struct *work)
{
  return container_of(work, struct bqsig_device, irq_handler_work);
}

static void irq_handler_work(struct work_struct *work)
{
        bool value;
  	struct bqsig_device *bqsig = container_of(work, struct bqsig_device, irq_handler_work);
	
	value = gpio_get_value(BATT_LOW_GPIO);

	//asserted 
	if (value) {
		led_trigger_event(bqsig->batt_normal_trig, LED_OFF);
		led_trigger_event(bqsig->batt_low_trig, LED_FULL);
		printk(KERN_INFO "WARNING: Low Battery!\n");
	}
	else {
		led_trigger_event(bqsig->batt_normal_trig, LED_FULL);
		led_trigger_event(bqsig->batt_low_trig, LED_OFF);
		printk(KERN_INFO "WARNING: Battery Level Safe\n");
	}
}

/*
 *    interrupt handler
 */

static irqreturn_t bqsig_irq_handler(int irq, void *dev_id)
{
  struct bqsig_device *bq_signal = dev_id;
  schedule_work(&bq_signal->irq_handler_work);
  return IRQ_HANDLED;
}

static void bqsig_low_trig_activate(struct led_classdev *led)
{	
	bool value;

	value = gpio_get_value(BATT_LOW_GPIO);
	
	if (value)
		led_set_brightness(led, LED_FULL);
	else
		led_set_brightness(led, LED_OFF);

	return;
}

static void bqsig_low_trig_deactivate(struct led_classdev *led)
{
	led_set_brightness(led, LED_OFF);
	return;
}

static void bqsig_normal_trig_activate(struct led_classdev *led)
{
	bool value;

	value = gpio_get_value(BATT_LOW_GPIO);
	
	if (value)
		led_set_brightness(led, LED_OFF);
	else
		led_set_brightness(led, LED_FULL);
	return;
}

static void bqsig_normal_trig_deactivate(struct led_classdev *led)
{
	led_set_brightness(led, LED_OFF);
	return;
}

/*
 *    probe/remove
 */

static int bug_bqsig_probe(struct platform_device *pdev)
{
        int err;
	//bool value;
	struct bqsig_device *bq_signal;

	bq_signal = kzalloc(sizeof(struct bqsig_device), GFP_KERNEL);
	bq_signal->pdev = pdev;
	
	// init work struct
	INIT_WORK(&bq_signal->irq_handler_work, irq_handler_work);

	// request dock presence pin
	err =  gpio_request(BATT_LOW_GPIO, "bq_batt_low");
	err |= gpio_direction_input(BATT_LOW_GPIO);
	gpio_set_debounce(BATT_LOW_GPIO, 5000);
	
	// request BATT_LOW irq
	err = request_irq(gpio_to_irq(BATT_LOW_GPIO), bqsig_irq_handler,
			  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "bq-battlow-signal", bq_signal); 
 
	platform_set_drvdata(pdev, bq_signal);

	// check for batt_low assertion on probe

	if (err < 0) {
	        printk(KERN_ERR "bq_signal: error during probe...\n");
		return -EINVAL;
	}

	led_trigger_register_simple("batt-low", &bq_signal->batt_low_trig);
	led_trigger_register_simple("batt-normal", &bq_signal->batt_normal_trig);
	
	if (bq_signal->batt_low_trig && bq_signal->batt_normal_trig) {
		bq_signal->batt_low_trig->activate = bqsig_low_trig_activate;
		bq_signal->batt_low_trig->deactivate = bqsig_low_trig_deactivate;
		bq_signal->batt_normal_trig->activate = bqsig_normal_trig_activate;
		bq_signal->batt_normal_trig->deactivate = bqsig_normal_trig_deactivate;
	}
	else
		printk(KERN_ERR "%s led triggers failed to register...\n", __FUNCTION__);

	printk(KERN_INFO "bug_bqsig: bug_bqsig_probe...\n");
	return 0;
}

static int bug_bqsig_remove(struct platform_device *pdev)
{
        struct bqsig_device *bq_signal;
	bq_signal = platform_get_drvdata(pdev);
	
	free_irq(gpio_to_irq(BATT_LOW_GPIO), bq_signal);
	gpio_free(BATT_LOW_GPIO);

	printk(KERN_INFO "bug_bqsig: bug_bqsig_remove...\n");
	return 0;
}

static int bug_bqsig_suspend(struct platform_device *pdev, pm_message_t state)
{
       return 0;
}

static int bug_bqsig_resume(struct platform_device *pdev)
{
       return 0;
}

static struct platform_driver bug_bqsig_drv = {
       .probe          = bug_bqsig_probe,
       .remove         = bug_bqsig_remove,
       .suspend        = bug_bqsig_suspend,
       .resume         = bug_bqsig_resume,
       .driver         = {
                               .name   = "bq-battlow-signal",
                       },
};

static int __init bug_bqsig_init(void)
{
  int ret; 

  bug_bqsig_dev = platform_device_alloc("bq-battlow-signal", -1);
  ret = platform_device_add(bug_bqsig_dev);

  ret = platform_driver_register(&bug_bqsig_drv);
  return ret;
}

static void __exit bug_bqsig_exit(void)
{
  platform_driver_unregister(&bug_bqsig_drv);
  platform_device_unregister(bug_bqsig_dev);
}

subsys_initcall(bug_bqsig_init);
module_exit(bug_bqsig_exit);

MODULE_AUTHOR("Bug Labs");
MODULE_DESCRIPTION("OMAP3 Bug BATT_LOW Signal Handler");
MODULE_LICENSE("GPL");
