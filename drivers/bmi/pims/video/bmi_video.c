/*
 * 	bmi_video.c
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
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <plat/hardware.h>
#include <linux/i2c.h>
#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>
//#include <linux/bmi/bmi_lcd.h>
#include <plat/display.h>
#include <linux/fb.h>
#include <linux/list.h>

//exported functions
#include <../drivers/video/omap2/displays/ths8200.h>
#include <../drivers/video/omap2/displays/tfp410.h>

//vmodes
#define DVI      0
#define VGA     1
#define OFF      2

u8 mode_list[] = {
  DVI,
  VGA,
  OFF,
};

//static LIST_HEAD(mode_list);

/*
 * 	Global variables
 */

//dss display structs
static struct omap_dss_device *dvi_disp;
static struct omap_dss_device *vga_disp;

//i2c video controller interfaces
static struct i2c_board_info tfp_info = {
  I2C_BOARD_INFO("tfp410p", 0x38),
};

static struct i2c_board_info ths_info = {
  I2C_BOARD_INFO("ths8200", 0x20),
};

//private device structure
struct bmi_video
{
  struct bmi_device    *bdev;			        // BMI device
  struct cdev		   cdev;			        // control device
  struct device 	   *class_dev;		        // control class device
  int		 	           open_flag;		        // single open flag
  char	  		   int_name[20];		// interrupt name
  struct i2c_client       *dvi_controller;
  struct i2c_client       *vga_controller;
  int                             vmode;

};

struct bmi_video bmi_video;
static int major;

//output enable prototypes
static void enable_dvi(struct bmi_video *video);
static void enable_vga(struct bmi_video *video);

/*
 *      sysfs interface
 */


static ssize_t bmi_video_vmode_show(struct device *dev, 
				    struct device_attribute *attr, char *buf)
{
    int i;
    int len = 0;

    struct bmi_video *video = dev_get_drvdata(dev);
    
    for (i = 0; i < 3; i++) {
        if (video->vmode == mode_list[i])
	    len += sprintf(buf+len, "[%d] ", mode_list[i]);
	else
	    len += sprintf(buf+len, "%d ", mode_list[i]);
    }
    
    len += sprintf(len+buf, "\n");
    return len;

    /*
    if (video->vmode == DVI)
        return snprintf(buf, PAGE_SIZE, "DVI(%d)\n", video->vmode);
    else if (video->vmode == VGA)
        return snprintf(buf, PAGE_SIZE, "VGA(%d)\n", video->vmode);    
    */
}
	
static ssize_t bmi_video_vmode_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t len)
{
    struct bmi_video *video = dev_get_drvdata(dev);

    if (strchr(buf, 'dvi') != NULL){
	video->vmode = DVI;
	enable_dvi(video);
    }
    else if (strchr(buf, 'vga') != NULL){
	video->vmode = VGA;
	enable_vga(video);
    }
    else if (strchr(buf, 't1') != NULL){
	tfp410_disable(video->dvi_controller);
    }
    else if (strchr(buf, 't2') != NULL){
	tfp410_enable(video->dvi_controller);
    }
    else if (strchr(buf, 't3') != NULL){
	ths8200_enable(video->vga_controller);
    }
    else if (strchr(buf, 't4') != NULL){
	ths8200_disable(video->vga_controller);
    }
    else if (strchr(buf, 't5') != NULL){
	ths8200_init(video->vga_controller);
    }
    else if (strchr(buf, 'off') != NULL){
        tfp410_disable(video->dvi_controller);
	ths8200_disable(video->vga_controller);
    }

    return len;
}

static DEVICE_ATTR(vmode, 0664, bmi_video_vmode_show, bmi_video_vmode_store);

/*
 * 	BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_video_tbl[] = 
{ 
	{ 
		.match_flags  = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor           = BMI_VENDOR_BUG_LABS, 
		.product          = BMI_PRODUCT_VIDEO, 
		.revision         = BMI_ANY, 
	}, 
	{ 0, },	  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_video_tbl);

int	bmi_video_probe (struct bmi_device *bdev);
void	bmi_video_remove (struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_video_driver = 
{
	.name      = "bmi_video", 
	.id_table  = bmi_video_tbl, 
	.probe      = bmi_video_probe, 
	.remove   = bmi_video_remove, 
};

/*
 *	PIM functions
 */

// interrupt handler
//static irqreturn_t module_irq_handler(int irq, void *dummy)
//{
//	return IRQ_HANDLED;
//}

/*
 * 	BMI functions
 */

// probe - insert PIM
int bmi_video_probe(struct bmi_device *bdev)
{
	int err;
	int slot;
	dev_t dev_id;
	int irq;

	struct bmi_video *video;
	struct class *bmi_class;
       	struct i2c_adapter *adap;
	struct omap_dss_device *dssdev;

	printk (KERN_INFO "bmi_video.c: probe...\n");	

	err = 0;
	slot = bdev->slot->slotnum;
      	adap = bdev->slot->adap;
	video = &bmi_video;

	video->bdev = 0;
	video->open_flag = 0;
	
	dssdev = NULL;
	dvi_disp = NULL;
	vga_disp = NULL;

        for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);

		if (dssdev->state)
		        dssdev->disable(dssdev);
		
		if (strnicmp(dssdev->name, "dvi", 3) == 0)
		        dvi_disp = dssdev;
		else if (strnicmp(dssdev->name, "vga", 3) == 0)
		        vga_disp = dssdev;
	}

	// bind driver and bmi_device 
	video->bdev = bdev;

	// create 1 minor device
	dev_id = MKDEV(major, slot); 

	// create class device 
	bmi_class = bmi_get_class ();                            

	// bind driver and bmi_device 
	video->bdev = bdev;

	err = sysfs_create_file(&bdev->dev.kobj, &dev_attr_vmode);
	if (err < 0)
	        printk(KERN_ERR "Error creating SYSFS entries...\n");

        tfp_info.irq = gpio_to_irq(10);
	video->dvi_controller = i2c_new_device(adap, &tfp_info);
	if (video->dvi_controller == NULL)
	        printk(KERN_ERR "TFP NULL...\n");

	video->vga_controller = i2c_new_device(adap, &ths_info);
	if (video->vga_controller == NULL)
		printk(KERN_ERR "THS NULL...\n");
	
	//default video mode: DVI
	video->vmode = DVI;

	//request PIM interrupt
	irq = bdev->slot->status_irq;
	sprintf (video->int_name, "bmi_video%d", slot);

	bmi_device_set_drvdata (bdev, video);

	enable_dvi(video);

	return 0;
	/*
 err1:	
	bmi_device_set_drvdata (bdev, 0);
	video->bdev = 0;

	return -ENODEV;
	*/
}

// remove PIM
void bmi_video_remove(struct bmi_device *bdev)
{	
	int irq;
	int i;
	int slot;

	struct bmi_video *video;
	struct class *bmi_class;

	printk(KERN_INFO "bmi_video: Module Removed...\n");

	slot = bdev->slot->slotnum;
	video = &bmi_video;

	//disable displays
	if (video->vmode == DVI)
	    tfp410_disable(video->dvi_controller);
	if (dvi_disp->state)
	        dvi_disp->disable(dvi_disp);

	if (video->vmode == VGA)
	    ths8200_disable(video->vga_controller);
	if (vga_disp->state)
	        vga_disp->disable(vga_disp);

	i2c_unregister_device(video->dvi_controller);
	i2c_unregister_device(video->vga_controller);
	irq = bdev->slot->status_irq;

	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	sysfs_remove_file(&bdev->dev.kobj, &dev_attr_vmode);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	video->class_dev = 0;

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	video->bdev = 0;

	return;
}

/*
 *	module routines
 */

static void enable_vga(struct bmi_video *video)
{
        int err;
        struct fb_info *info;
	struct fb_var_screeninfo var;
	
	printk (KERN_INFO "bmi_video.c: setting up VGA Output...\n");

	//disable dvi (tfp)
	tfp410_disable(video->dvi_controller);

	//disable dvi (dss)
	if (dvi_disp->state)
	    dvi_disp->disable(dvi_disp);

	//set omapfb
	info = registered_fb[0];
	var = info->var;	
	var.xres = 1024;
	var.yres = 768;
	var.xres_virtual = 1024;
	var.yres_virtual = 768;
        var.activate = 128;             //Force update

        err = fb_set_var(info, &var);
	if (err)
	        printk(KERN_ERR "bmi_video.c: enable_vga: error resizing omapfb");

	//enable vga (dss)
	if (vga_disp->state != 1)
	        vga_disp->enable(vga_disp);

	//init vga (ths)
	ths8200_init(video->vga_controller);
}

static void enable_dvi(struct bmi_video *video)
{
        int err;
	struct fb_info *info;
	struct fb_var_screeninfo var;

       	printk (KERN_INFO "bmi_video.c: setting up DVI Output...\n");

	//disable vga (tfp)
	ths8200_disable(video->vga_controller);

	//disable vga (dss)
	if (vga_disp->state)
	    vga_disp->disable(vga_disp);
	
	//set omapfb
	info = registered_fb[0];
	var = info->var;
	var.xres = 1280;
	var.yres = 1024;
	var.xres_virtual = 1280;
	var.yres_virtual = 1024;
        var.activate = 128;             //Force update

        err = fb_set_var(info, &var);	
	if (err)
	        printk(KERN_ERR "bmi_video.c: enable_dvi: error resizing omapfb");

	//enable dvi (dss)
	if (dvi_disp->state != 1)
	        dvi_disp->enable(dvi_disp);

	//init dvi (tfp)
	tfp410_init(video->dvi_controller);
}

static void __exit bmi_video_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver (&bmi_video_driver);

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}

static int __init bmi_video_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI VIDEO Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_video_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}

	printk (KERN_INFO "bmi_video.c: BMI_VIDEO Driver...\n");

	return 0;
}


module_init(bmi_video_init);
module_exit(bmi_video_cleanup);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Buglabs Inc.");
MODULE_DESCRIPTION("BMI video module device driver");
