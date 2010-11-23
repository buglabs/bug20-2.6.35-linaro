/*
 * Copyright 2008 EnCADIS Designs, Inc. All Rights Reserved.
 * Copyright 2008 Bug-Labs, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*-----------------------------------------------------------------------------
 *
 *      Part of BMI Motion Detector Accelerometer (MDACC) Kernel Module
 *
 *-----------------------------------------------------------------------------
 */

#include "ctl.h"
#include "mdacc.h"
#include <linux/bmi/bmi_mdacc.h>
#include <linux/ioctl.h>
#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>


#include <linux/module.h>

static int ctl_open (struct inode *, struct file *);
static int ctl_release (struct inode *, struct file *);
static int ctl_ioctl (struct inode *, struct file *, unsigned int, unsigned long);

struct file_operations ctl_fops = {
	.owner = THIS_MODULE, 
	.ioctl = ctl_ioctl, 
	.open = ctl_open, 
	.release = ctl_release, 
};

static int ctl_major;

int ctl_init (void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers

	retval = alloc_chrdev_region(&dev_id, 0, 4, "BMI MDACC Control Driver"); 
				     
	if (retval) {
		return -1;
	}
	ctl_major = MAJOR(dev_id);
	return 0;
}

void ctl_clean (void)
{
	dev_t dev_id;

	dev_id = MKDEV(ctl_major, 0); 
	unregister_chrdev_region(dev_id, 4);
	return;
}

int ctl_probe (struct ctl *ctl, int slot) 
{
	struct cdev *cdev;
	dev_t dev_id;
	int ret;
	struct class *bmi_class;

	cdev = &ctl->cdev;
	cdev_init (cdev, &ctl_fops);

	dev_id = MKDEV (ctl_major, slot); 
	ret = cdev_add (cdev, dev_id, 1);

	//Create class device 
	bmi_class = bmi_get_class ();                            

	ctl->class_dev = device_create (bmi_class, NULL, MKDEV(ctl_major, slot), ctl, "bmi_mdacc_ctl_m%i", slot+1);  
								     
	if (IS_ERR(ctl->class_dev)) {                                
		printk(KERN_ERR "Unable to create "                  
		       "class_device for bmi_mdacc_ctl_m%i; errno = %ld\n",
		       slot+1, PTR_ERR(ctl->class_dev));             
		ctl->class_dev = NULL;                               
	}

	return ret;
}

void ctl_remove (struct ctl *ctl, int slot) 
{
	struct class *bmi_class;

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(ctl_major, slot));

	ctl->class_dev = 0;

	cdev_del (&ctl->cdev);
	return;
}


static int ctl_open (struct inode *inode, struct file *file)
{
	struct ctl *ctl;
	
	ctl = container_of(inode->i_cdev, struct ctl, cdev);
	

	// Save ctl pointer for later.

	file->private_data = ctl;
	return 0;
}

static int ctl_release (struct inode *inode, struct file *file)
{
	struct ctl *ctl;

	ctl = container_of(inode->i_cdev, struct ctl, cdev);
	return 0;
}

static int ctl_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ctl *ctl;
	int slot;
	unsigned char tmp = 0;
	
	ctl = container_of(inode->i_cdev, struct ctl, cdev);
	slot = mdacc_get_slot_ctl (ctl);
	if (slot < 0) {
		return -ENODEV;
	}	

	switch (cmd) {

	case BMI_MDACC_CTL_RED_LED_OFF:
	  bmi_slot_gpio_set_value (slot, RED_LED, 1);
	 //bmi_slot_gpio_write_bit (slot, 3, 1); //Red LED Off
	 break;

	case BMI_MDACC_CTL_RED_LED_ON:
	  bmi_slot_gpio_set_value (slot, RED_LED, 0);
	  //bmi_slot_gpio_write_bit (slot, 3, 0); //Red LED On
	  break;

	case BMI_MDACC_CTL_GREEN_LED_OFF:
	  bmi_slot_gpio_set_value (slot, GREEN_LED, 1);
	  //bmi_slot_gpio_write_bit (slot, 2, 1); //Green LED Off
	  break;

	case BMI_MDACC_CTL_GREEN_LED_ON:
	  bmi_slot_gpio_set_value (slot, GREEN_LED, 0);
	  //bmi_slot_gpio_write_bit (slot, 2, 0); //Green LED On
	  break;
		
	default:
	  printk (KERN_ERR "ctl_ioctl() - error exit\n");
	  return -ENOTTY;
	}

	return 0;
}

