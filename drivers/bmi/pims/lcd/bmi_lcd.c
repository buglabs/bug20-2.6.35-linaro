/*
 * 	bmi_lcd.c
 *
 * 	BMI LCD device driver basic functionality
 *
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 *	Include files
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <plat/hardware.h>
#include <linux/i2c.h>
#include <linux/i2c/tsc2004.h>
#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_lcd.h>
#include <plat/display.h>
#include <linux/fb.h>
#include <linux/pm.h>

/*
 * 	Global variables
 */

static struct platform_device *bl_backlight_dev;
static struct omap_dss_device *this_disp;

static int tsc2004_init(void)
{
  int res;

  res = gpio_direction_input(10);
  
  omap_set_gpio_debounce(10, 1);
  omap_set_gpio_debounce_time(10, 0xff);
	
  return 1;
}

static int tsc2004_get_pendown_state(void)
{
  return !gpio_get_value(10);
}

static struct tsc2004_platform_data tsc_platform_data= {
  .model = 2004,
  .x_plate_ohms = 180,
  .get_pendown_state = tsc2004_get_pendown_state,
  .init_platform_hw = tsc2004_init,
};

static struct i2c_board_info tsc_info = {
	I2C_BOARD_INFO("tsc2004", 0x48),
	.platform_data = &tsc_platform_data,
};

static struct i2c_board_info acc_info = {
	I2C_BOARD_INFO("ml8953", 0x17),
};

// private device structure
struct bmi_lcd
{
  struct bmi_device	*bdev;			// BMI device
  struct cdev		cdev;			// control device
  struct device  	*class_dev;		// control class device
  int			open_flag;		// single open flag
  char			int_name[20];		// interrupt name
  struct i2c_client     *tsc;
  struct i2c_client     *acc;
};

struct bmi_lcd bmi_lcd;
static int major;	// control device major

/*
 *      sysfs interface
 */

static ssize_t bmi_lcd_suspend_show(struct device *dev, 
				    struct device_attribute *attr, char *buf)
{
    int len = 0;
    struct bmi_lcd *lcd = dev_get_drvdata(dev);

    int status = lcd->bdev->dev.power.status;
    
    if (status == DPM_ON)
      len += sprintf(buf+len, "0");      
    else
      len += sprintf(buf+len, "1");      
    
    len += sprintf(len+buf, "\n");
    return len;
}
	
static ssize_t bmi_lcd_suspend_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t len)
{
    struct bmi_lcd *lcd = dev_get_drvdata(dev);

    if (strchr(buf, '1') != NULL){
          lcd->bdev->dev.bus->pm->suspend(&lcd->bdev->dev);
    }
    else if (strchr(buf, '0') != NULL){
          lcd->bdev->dev.bus->pm->resume(&lcd->bdev->dev);
    }

    return len;
}

static DEVICE_ATTR(suspend, 0664, bmi_lcd_suspend_show, bmi_lcd_suspend_store);

/*
 * 	BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_lcd_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor      = BMI_VENDOR_BUG_LABS, 
		.product     = BMI_PRODUCT_LCD_SHARP_320X240, 
		.revision    = BMI_ANY, 
	}, 
	{ 0, },	  /* terminate list */
};

MODULE_DEVICE_TABLE(bmi, bmi_lcd_tbl);

int	bmi_lcd_probe (struct bmi_device *bdev);
void	bmi_lcd_remove (struct bmi_device *bdev);
int	bmi_lcd_resume (struct device *dev);
int	bmi_lcd_suspend (struct device *dev);

static struct dev_pm_ops bmi_lcd_pm =
{
	.resume = bmi_lcd_resume,
	.suspend = bmi_lcd_suspend,
};

// BMI driver structure
static struct bmi_driver bmi_lcd_driver = 
{
	.name     = "bmi_lcd", 
	.id_table = bmi_lcd_tbl, 
	.probe    = bmi_lcd_probe, 
	.remove   = bmi_lcd_remove,
	.pm       = &bmi_lcd_pm,
};

/*
 *	PIM functions
 */

// interrupt handler
/*
static irqreturn_t module_irq_handler(int irq, void *dummy)
{
	return IRQ_HANDLED;
}
*/

/*
 * 	BMI functions
 */

// probe - insert PIM
int bmi_lcd_probe(struct bmi_device *bdev)
{
	int err;
	int slot;
	struct bmi_lcd *lcd;
 	struct class *bmi_class;
      	struct i2c_adapter *adap;
	struct omap_dss_device *dssdev;

	struct fb_info *info;
	struct fb_var_screeninfo var;

	dev_t dev_id;
	int irq;

	err = 0;
	slot = bdev->slot->slotnum;
      	adap = bdev->slot->adap;
	lcd = &bmi_lcd;

	lcd->bdev = 0;
	lcd->open_flag = 0;

	//create sysfs entries
	err = sysfs_create_file(&bdev->dev.kobj, &dev_attr_suspend.attr);
	if (err < 0)
	        printk(KERN_ERR "Error creating SYSFS entries...\n");

	// Get display info and disable active display
	dssdev = NULL;
	this_disp = NULL;
        for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);

		if (dssdev->state)
		        dssdev->disable(dssdev);

		if (strnicmp(dssdev->name, "lcd", 3) == 0)
		        this_disp = dssdev;
	}
       
	// Resize the frame buffer
	info = registered_fb[0];
	var = info->var;

	var.xres = 320;
	var.yres = 240;
	var.xres_virtual = 320;
	var.yres_virtual = 240;
	var.activate = 128;               // Force update

	err = fb_set_var(info, &var);
	
	if (err)
	  printk(KERN_ERR "bmi_lcd.c: probe: error resizing omapfb");

	// Enable this display
	this_disp->enable(this_disp);

	// Create 1 minor device
	dev_id = MKDEV(major, slot); 

	// Create class device 
	bmi_class = bmi_get_class ();                            

	// bind driver and bmi_device 
	lcd->bdev = bdev;

  	gpio_direction_input(15);		

	tsc_info.irq = gpio_to_irq(10);
	lcd->tsc = i2c_new_device(bdev->slot->adap, &tsc_info);
	if (lcd->tsc == NULL)
	  printk(KERN_ERR "TSC NULL...\n");
	
	acc_info.irq = gpio_to_irq(15);
	lcd->acc = i2c_new_device(bdev->slot->adap, &acc_info);
	if (lcd->acc == NULL)
	  printk(KERN_ERR "ACC NULL...\n");

	bl_backlight_dev = platform_device_alloc("omap-backlight", -1);
	err = platform_device_add(bl_backlight_dev);

	if (err) {
	  platform_device_put(bl_backlight_dev);
	  printk(KERN_INFO "Backlight driver failed to load...");
	}	  

	bmi_device_set_drvdata (bdev, lcd);

	printk (KERN_INFO "bmi_lcd.c: probe...\n");

	// request PIM interrupt
	irq = bdev->slot->status_irq;
	sprintf (lcd->int_name, "bmi_lcd%d", slot);

	return 0;

//err1:	
	//bmi_device_set_drvdata (bdev, 0);
	//lcd->bdev = 0;
	//return -ENODEV;
}

// remove PIM
void bmi_lcd_remove(struct bmi_device *bdev)
{	
	int slot;
	struct bmi_lcd *lcd;
	struct class *bmi_class;
	int irq;
	int i;

	printk(KERN_INFO "bmi_lcd: Module Removed...\n");
	slot = bdev->slot->slotnum;
	lcd = &bmi_lcd;
	i2c_unregister_device(lcd->tsc);
	i2c_unregister_device(lcd->acc);
	irq = bdev->slot->status_irq;

	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	platform_device_unregister(bl_backlight_dev);
	
	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	lcd->class_dev = 0;

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	lcd->bdev = 0;

	// disable display
	this_disp->disable(this_disp);

	//remove sysfs entries
	sysfs_remove_file(&bdev->dev.kobj, &dev_attr_suspend.attr);


	return;
}


/*
 *	PM routines
 */

int bmi_lcd_resume(struct device *dev)
{
	struct bmi_device *bmi_dev;
	struct bmi_lcd *lcd;

	bmi_dev = to_bmi_device(dev);
	lcd = dev_get_drvdata(dev);

	printk(KERN_INFO "bmi_lcd: resume...\n");
	bmi_slot_spi_enable(bmi_dev->slot->slotnum);

	lcd->tsc->dev.bus->resume(&lcd->tsc->dev);
	lcd->acc->dev.bus->resume(&lcd->acc->dev);
	bl_backlight_dev->dev.bus->pm->resume(&bl_backlight_dev->dev);
	this_disp->enable(this_disp);

	return 0;
}

int bmi_lcd_suspend(struct device *dev)
{
	struct bmi_device *bmi_dev;
	struct bmi_lcd *lcd;

	bmi_dev = to_bmi_device(dev);
	lcd = dev_get_drvdata(dev);

	printk(KERN_INFO "bmi_lcd: suspend...\n");
	
	this_disp->disable(this_disp);
	lcd->tsc->dev.bus->suspend(&lcd->tsc->dev, PMSG_SUSPEND);
	lcd->acc->dev.bus->suspend(&lcd->acc->dev, PMSG_SUSPEND);
	bl_backlight_dev->dev.bus->pm->suspend(&bl_backlight_dev->dev);

	bmi_slot_spi_disable(bmi_dev->slot->slotnum);
	return 0;
}

/*
 *	module routines
 */

static void __exit bmi_lcd_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver (&bmi_lcd_driver);

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}

static int __init bmi_lcd_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI LCD Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_lcd_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}

	printk (KERN_INFO "bmi_lcd.c: BMI_LCD Driver...\n");

	return 0;
}


module_init(bmi_lcd_init);
module_exit(bmi_lcd_cleanup);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Buglabs Inc.");
MODULE_DESCRIPTION("BMI LCD device driver");
