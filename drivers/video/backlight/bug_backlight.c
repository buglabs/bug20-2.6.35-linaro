/*
 * drivers/video/backlight/bug_backlight.c
 *
 * Backlight driver for BUGBASE2
 *
 * Copyright (c) 2009, Texas Instruments Incorporated.
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
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/i2c/twl.h>
#include <plat/hardware.h>

/**
 * Name of the driver
 */
#define OMAPBL_DRVNAME "omap-backlight"

/**
 * Name of the device
 */
#define OMAPBL_DEVNAME "omap-backlight"

/**
 * Minimum intensity supported by the panel
 */
#define OMAPBL_MIN_INTENSITY   0
/**
 * Maximum intensity supported by the panel
 */
#define OMAPBL_MAX_INTENSITY   100

/**
 * Default intensity after boot-up
 */
#define OMAPBL_DEF_INTENSITY   70

/**
 * Flag indicating the driver status - suspended / running
 */
#define OMAPBL_SUSPENDED       0x01


#define TWL_PWM1_PWM1ON        0x00
#define TWL_PWM1_PWM1OFF       0x01

/**
 * Current backlight intensity
 */
static int panel_intensity;

/**
 * Backlight properties
 */
static struct backlight_properties omapbl_props;


/**
 * Backlight device
 */
struct backlight_device *omapbl_device;

/**
 * Backlight flags
 */
static unsigned long omapbl_flags;

static int omapbl_check_fb(struct backlight_device *bldev, struct fb_info *info)
{
	return 0;
}

static int omapbl_set_intensity(struct backlight_device *bd)
{
       int intensity = bd->props.brightness;
       int ret;
       u8 c;

       if (bd->props.power != FB_BLANK_UNBLANK)
               intensity = 0;
       if (bd->props.fb_blank != FB_BLANK_UNBLANK)
               intensity = 0;
       if (omapbl_flags & OMAPBL_SUSPENDED)
               intensity = 0;
       if (intensity == 0)
	       printk("Backlight turned off\n");
       
       c = ((125 * intensity) / 100) + 2;

#if 0
	   if (intensity > 100)
	   {
		   twl_i2c_read_u8(TWL4030_MODULE_INTBR, &c, 0x0c);
		   c &= ~(1 << 3);
		   twl_i2c_write_u8 (TWL4030_MODULE_INTBR, c, 0x0c);
		   return ret;
	   }
#endif
       /*
	* Program the OFF register of PWM1 with user values
	*/
       ret = twl_i2c_write_u8(TWL4030_MODULE_PWM1, c, TWL_PWM1_PWM1OFF);
       
       if (ret) {
	       printk("i2c write failed\n");
	       return ret;
       }
//     printk(KERN_INFO "Backlight altered with value %d.\n", intensity);

       panel_intensity = intensity;

       return 0;
}

static int omapbl_get_intensity(struct backlight_device *bd)
{
       return panel_intensity;
}

static struct backlight_ops omapbl_ops = {
       .get_brightness = omapbl_get_intensity,
       .update_status  = omapbl_set_intensity,
       .check_fb       = omapbl_check_fb,
};

static int omapbl_probe(struct platform_device *pdev)
{
  	u8 c;

	omapbl_device = backlight_device_register (OMAPBL_DRVNAME,
                                                       &pdev->dev,
                                                       NULL,
                                                       &omapbl_ops,
						       NULL);
	if (IS_ERR (omapbl_device))
		return PTR_ERR (omapbl_device);

	platform_set_drvdata(pdev, omapbl_device);

	omapbl_device->props.power              = FB_BLANK_UNBLANK;
	omapbl_device->props.max_brightness     = OMAPBL_MAX_INTENSITY;
	omapbl_device->props.brightness         = OMAPBL_DEF_INTENSITY;

	/* Enable PWM1 and PWM1_CLK in GPBR */
	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &c, 0x0c);
	c |= (1 << 3) | (1 << 1);
	twl_i2c_write_u8 (TWL4030_MODULE_INTBR, c, 0x0c);

	/* Select PWM1 output in PMBR1 */
	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &c, 0x0d);
	c |= 0x30;
	twl_i2c_write_u8 (TWL4030_MODULE_INTBR, c, 0x0d);

	omapbl_set_intensity(omapbl_device);

	printk(KERN_INFO "omap-backlight: device initialized.\n");

	return 0;
}

static int omapbl_remove(struct platform_device *pdev)
{
  u8 c;
  struct backlight_device *bd = platform_get_drvdata(pdev);
  
  omapbl_props.power = 0;
  omapbl_props.brightness = 0;
  backlight_update_status(bd);
  
  
  /* Disableable PWM1 and PWM1_CLK in GPBR */
  twl_i2c_read_u8(TWL4030_MODULE_INTBR, &c, 0x0c);
  c &= ~((1 << 3) | (1 << 1));
  twl_i2c_write_u8 (TWL4030_MODULE_INTBR, c, 0x0c);
  
  /* Restore default/gpio output in PMBR1 */
  twl_i2c_read_u8(TWL4030_MODULE_INTBR, &c, 0x0d);
  c &= ~(0x30);
  twl_i2c_write_u8 (TWL4030_MODULE_INTBR, c, 0x0d);
    
  backlight_device_unregister(bd);
  
  printk(KERN_INFO "omap-backlight: device unloaded.\n");
  
  return 0;
}

#ifdef CONFIG_PM
static int omapbl_suspend(struct platform_device *pdev, pm_message_t state)
{
       struct backlight_device *bd = platform_get_drvdata(pdev);

       printk(KERN_INFO "omap-backlight: suspending...\n");

       omapbl_flags |= OMAPBL_SUSPENDED;
       backlight_update_status(bd);

       return 0;
}

static int omapbl_resume(struct platform_device *pdev)
{
       struct backlight_device *bd = platform_get_drvdata(pdev);

       printk(KERN_INFO "omap-backlight: resuming...\n");

       omapbl_flags &= ~OMAPBL_SUSPENDED;
       backlight_update_status(bd);

       return 0;
}
#else
#define omapbl_suspend NULL
#define omapbl_resume  NULL
#endif

static struct platform_driver omap_backlight_drv = {
       .probe          = omapbl_probe,
       .remove         = omapbl_remove,
       .suspend        = omapbl_suspend,
       .resume         = omapbl_resume,
       .driver         = {
                               .name   = OMAPBL_DRVNAME,
                       },
};

static struct platform_device *omap_backlight_dev;

static int __init omapbl_init(void)
{
       int ret = platform_driver_register(&omap_backlight_drv);
       return ret;
}

static void __exit omapbl_exit(void)
{
       platform_device_unregister(omap_backlight_dev);
       platform_driver_unregister(&omap_backlight_drv);
}

module_init(omapbl_init);
module_exit(omapbl_exit);

MODULE_AUTHOR("Umesh K");
MODULE_DESCRIPTION("OMAP LCD Backlight driver");
MODULE_LICENSE("GPL");

