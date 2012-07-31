/*
 * 	bmi_voice.c
 *
 * 	BMI von Hippel device driver basic functionality
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

#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/hardware.h>

#include <linux/i2c.h>

#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_voice.h>

#define BMIVOICE_VERSION	"1.0"

/*
 * 	Global variables
 */

struct bmi_voice
{
  struct bmi_device	*bdev;			// BMI device
  struct cdev		cdev;			// control device
  struct device	*class_dev;		// control class device
  int			open_flag;		// single open flag
  char			int_name[20];		// interrupt name
};

static struct bmi_voice bmi_voice[4];	// per slot device structure
static int major;		// control device major

/*
 * 	BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_voice_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_VOICE, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },	  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_voice_tbl);

int	bmi_voice_probe (struct bmi_device *bdev);
void	bmi_voice_remove (struct bmi_device *bdev);
int	bmi_voice_resume (struct device *dev);
int	bmi_voice_suspend (struct device *dev);
static void	bmi_voice_reset(struct bmi_device *bdev);
static void	bmi_voice_toggle_power(struct bmi_device *bdev);

static struct dev_pm_ops bmi_voice_pm =
{
	.resume = bmi_voice_resume,
	.suspend = bmi_voice_suspend,
};

// BMI driver structure
static struct bmi_driver bmi_voice_driver = 
{
	.name = "bmi_voice", 
	.id_table = bmi_voice_tbl, 
	.probe   = bmi_voice_probe, 
	.remove  = bmi_voice_remove, 
	.pm  = &bmi_voice_pm,
};

/*
 *	control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *file)
{	
	struct bmi_voice *voice;

	voice = container_of (inode->i_cdev, struct bmi_voice, cdev);

	// Enforce single-open behavior

	if (voice->open_flag) {
		return -EBUSY; 
	}
	voice->open_flag = 1;

	// Save voice_dev pointer for later.

	file->private_data = voice;
	return 0;

}

// release
int cntl_release(struct inode *inode, struct file *file)
{	
	struct bmi_voice *voice;

	voice = (struct bmi_voice *)(file->private_data);
	voice->open_flag = 0;
	return 0;
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct bmi_voice *voice;
	int slot;
	int argnum;

	voice = (struct bmi_voice *)(file->private_data);

	// error if voice not present
	if(voice->bdev == 0)
		return -ENODEV;
	
	slot = voice->bdev->slot->slotnum;

	// ioctl's
	switch (cmd) {

	case BMI_VOICE_SET_EN_SUP:
		get_user(argnum, (int *)arg);
	  	bmi_slot_gpio_set_value (slot, VOICE_GPIO_EN_SUP, argnum);  
		break;
	case BMI_VOICE_SET_ON_OFF:
		get_user(argnum, (int *)arg);
	  	bmi_slot_gpio_set_value (slot, VOICE_GPIO_ON_OFF, argnum);  
		break;
	case BMI_VOICE_SET_RESET:
		get_user(argnum, (int *)arg);
	  	bmi_slot_gpio_set_value (slot, VOICE_GPIO_RESET, argnum);  
		break;
	case BMI_VOICE_GET_PWR_MON:
		argnum = bmi_slot_gpio_get_value(slot, VOICE_GPIO_PWR_MON);
		put_user(argnum, (int *)arg);
		break;
	case BMI_VOICE_RESET:
	  	bmi_slot_gpio_set_value (slot, VOICE_GPIO_RESET, 0);  
		msleep(5000);
	  	bmi_slot_gpio_set_value (slot, VOICE_GPIO_RESET, 1);  
		break;
	case BMI_VOICE_TOGGLE_POWER:
	  	bmi_slot_gpio_set_value (slot, VOICE_GPIO_ON_OFF, 0);  
		msleep(5000);
	  	bmi_slot_gpio_set_value (slot, VOICE_GPIO_ON_OFF, 1);  
		break;

	case BMI_VOICE_SUSPEND:
		voice->bdev->dev.bus->pm->suspend(&voice->bdev->dev);
		break;
	case BMI_VOICE_RESUME:
		voice->bdev->dev.bus->pm->resume(&voice->bdev->dev);
		break;
	default:
		return -ENOTTY;
	}

	return 0;
}

// control file operations
struct file_operations cntl_fops = {
	.owner = THIS_MODULE, 
	.ioctl = cntl_ioctl, 
	.open = cntl_open, 
	.release = cntl_release, 
};

/*
 *	PIM functions
 */

// interrupt handler
static irqreturn_t module_irq_handler(int irq, void *dummy)
{
	return IRQ_HANDLED;
}


// sysfs attribute - reset the the voice module
static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bmi_device *bdev;
	bdev = to_bmi_device(dev);	
	if (strlen(buf) > 0){
		bmi_voice_reset(bdev);
	}
	return count;
}

static void bmi_voice_reset(struct bmi_device *bdev)
{
	printk(KERN_DEBUG "voice - reset assert low\n");
	bmi_slot_gpio_set_value (bdev->slot->slotnum, VOICE_GPIO_RESET, 0);
	msleep(5000);
	printk(KERN_DEBUG "voice - reset clear high\n");
	bmi_slot_gpio_set_value (bdev->slot->slotnum, VOICE_GPIO_RESET, 1);
}

static DEVICE_ATTR(reset, 0664, NULL, reset_store);

// sysfs attribute - read the power status of the device
static ssize_t power_show(struct device *dev, struct device_attribute *attr,
		const char *buf)
{
	struct bmi_device *bdev;
	int value;
	bdev = to_bmi_device(dev);
	value = bmi_slot_gpio_get_value(bdev->slot->slotnum, 
			VOICE_GPIO_PWR_MON);
	return scnprintf(buf, PAGE_SIZE, "%d\n", value);
}

// sysfs attribute - toggle the power state of the voice module 
static ssize_t power_toggle_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bmi_device *bdev;
	bdev = to_bmi_device(dev);	
	if (strlen(buf) > 0){
		bmi_voice_toggle_power(bdev);	
	}
	return count;
}

static void bmi_voice_toggle_power(struct bmi_device *bdev)
{
	printk(KERN_DEBUG "voice - on_off assert low\n");
	bmi_slot_gpio_set_value (bdev->slot->slotnum, VOICE_GPIO_ON_OFF, 0);
	msleep(5000);
	printk(KERN_DEBUG "voice - on_off clear high\n");
	bmi_slot_gpio_set_value (bdev->slot->slotnum, VOICE_GPIO_ON_OFF, 1);
}

static DEVICE_ATTR(toggle_power, 0664, power_show, power_toggle_store);

// sysfs attribute - toggle the power state of the voice module 
static ssize_t enable_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bmi_device *bdev;
	int value;
	bdev = to_bmi_device(dev);	
	if (strlen(buf) > 0){
		if (buf[0] == '0') {
			printk(KERN_DEBUG "voice - disabling module power\n");
			bmi_slot_gpio_set_value (bdev->slot->slotnum,
				       	VOICE_GPIO_EN_SUP, 0);
		} else {
			printk(KERN_DEBUG "voice - enabling module power\n");
			bmi_slot_gpio_set_value (bdev->slot->slotnum,
				       	VOICE_GPIO_EN_SUP, 1);
		}
	}
	return count;
}

static DEVICE_ATTR(enable, 0664, NULL, enable_store);

/*
 * 	BMI functions
 */

// probe - insert PIM
int bmi_voice_probe(struct bmi_device *bdev)
{
	int err;
	int slot;
	struct bmi_voice *voice;
	struct cdev *cdev;
	struct class *bmi_class;
	dev_t dev_id;
	int irq;

	err = 0;
	slot = bdev->slot->slotnum;
	voice = &bmi_voice[slot];

	voice->bdev = 0;
	voice->open_flag = 0;
	
	// Create 1 minor device
	cdev = &voice->cdev;
	cdev_init (cdev, &cntl_fops);

	dev_id = MKDEV(major, slot); 
	err = cdev_add (cdev, dev_id, 1);
	if (err) {
		return err;
	}

	// Create class device 
	bmi_class = bmi_get_class ();                            
	voice->class_dev = device_create (bmi_class, NULL, MKDEV (major, slot), NULL, "bmi_voice_control_m%i", slot);  
								     
	if (IS_ERR(voice->class_dev)) {                                
		printk (KERN_ERR "Unable to create "                  
		       "class_device for bmi_voice_m%i; errno = %ld\n",
		       slot, PTR_ERR(voice->class_dev));             
		voice->class_dev = NULL;                               
		cdev_del (&voice->cdev);
		return -ENODEV;
	}                                                            

	// bind driver and bmi_device 
	voice->bdev = bdev;

	err = sysfs_create_file(&bdev->dev.kobj, &dev_attr_reset.attr);
	err = sysfs_create_file(&bdev->dev.kobj, &dev_attr_toggle_power.attr);
	err = sysfs_create_file(&bdev->dev.kobj, &dev_attr_enable.attr);
	if (err < 0){
		printk(KERN_ERR "Unable to add sysfs attribute \n");
	}

	printk (KERN_INFO "bmi_voice.c: probe slot %d\n", slot);

	bmi_device_set_drvdata (bdev, voice);
	// configure IOX
	  
	printk (KERN_INFO "bmi_voice.c: enabling module\n");
	// Initialize GPIOs (turn LED's on)
	
	bmi_slot_gpio_direction_out (slot, VOICE_GPIO_EN_SUP, 1);
	bmi_slot_gpio_direction_out (slot, VOICE_GPIO_ON_OFF, 1);
	bmi_slot_gpio_direction_out (slot, VOICE_GPIO_RESET, 1);	
	bmi_slot_gpio_direction_in (slot, VOICE_GPIO_PWR_MON);
	
	// request PIM interrupt
	irq = bdev->slot->status_irq;
	sprintf (voice->int_name, "bmi_voice%d", slot);
	if (request_irq(irq, &module_irq_handler, IRQF_TRIGGER_FALLING, voice->int_name, voice)) {
		printk (KERN_ERR "bmi_voice.c: Can't allocate irq %d or find voice in slot %d\n", 
			irq, slot); 
		//bmi_device_spi_cleanup(bdev);
		goto err1;

		//return -EBUSY;
	}
	return 0;

 err1:	
	voice->class_dev = NULL;                               
	cdev_del (&voice->cdev);
	device_destroy (bmi_class, MKDEV(major, slot));
	bmi_device_set_drvdata (bdev, 0);
	voice->bdev = 0;
	return -ENODEV;
}

// remove PIM
void bmi_voice_remove(struct bmi_device *bdev)
{	
	int slot;
	struct bmi_voice *voice;
	struct class *bmi_class;
	int irq;
	int i;

	printk(KERN_INFO "bmi_voice: Module Removed...\n");
	slot = bdev->slot->slotnum;
	voice = &bmi_voice[slot];

	irq = bdev->slot->status_irq;
	free_irq (irq, voice);

	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	voice->class_dev = 0;

	cdev_del (&voice->cdev);

	sysfs_remove_file(&bdev->dev.kobj, &dev_attr_reset.attr);
	sysfs_remove_file(&bdev->dev.kobj, &dev_attr_toggle_power.attr);
	sysfs_remove_file(&bdev->dev.kobj, &dev_attr_enable.attr);

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	voice->bdev = 0;

	return;
}

/*
 *	PM routines
 */

int bmi_voice_resume(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_voice: Resume..\n");
	return 0;
}

int bmi_voice_suspend(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_voice: Suspend..\n");
	return 0;
}

/*
 *	module routines
 */

static void __exit bmi_voice_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver (&bmi_voice_driver);

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}

static int __init bmi_voice_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI VOICE Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_voice_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}

	printk (KERN_INFO "bmi_voice.c: BMI_VOICE Driver v%s \n", BMIVOICE_VERSION);

	return 0;
}


module_init(bmi_voice_init);
module_exit(bmi_voice_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Buglabs Inc.");
MODULE_DESCRIPTION("BMI voice device driver");
MODULE_SUPPORTED_DEVICE("bmi_voice_control_mX");

