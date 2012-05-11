/*
 * 	bmi_rf4ce.c
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
#include <linux/bmi/bmi_rf4ce.h>

#define BMIRF4CE_VERSION	"1.0"

/*
 * 	Global variables
 */

struct bmi_rf4ce
{
  struct bmi_device	*bdev;			// BMI device
  struct cdev		cdev;			// control device
  struct device	*class_dev;		// control class device
  int			open_flag;		// single open flag
  char			int_name[20];		// interrupt name
};

static struct bmi_rf4ce bmi_rf4ce[4];	// per slot device structure
static int major;		// control device major

/*
 * 	BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_rf4ce_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_RF4CE, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },	  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_rf4ce_tbl);

int	bmi_rf4ce_probe (struct bmi_device *bdev);
void	bmi_rf4ce_remove (struct bmi_device *bdev);
int	bmi_rf4ce_resume (struct device *dev);
int	bmi_rf4ce_suspend (struct device *dev);
static void	bmi_rf4ce_reset(struct bmi_device *bdev);
static void	bmi_rf4ce_prog(struct bmi_device *bdev);

static struct dev_pm_ops bmi_rf4ce_pm =
{
	.resume = bmi_rf4ce_resume,
	.suspend = bmi_rf4ce_suspend,
};

// BMI driver structure
static struct bmi_driver bmi_rf4ce_driver = 
{
	.name = "bmi_rf4ce", 
	.id_table = bmi_rf4ce_tbl, 
	.probe   = bmi_rf4ce_probe, 
	.remove  = bmi_rf4ce_remove, 
	.pm  = &bmi_rf4ce_pm,
};

/*
 *	control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *file)
{	
	struct bmi_rf4ce *rf4ce;

	rf4ce = container_of (inode->i_cdev, struct bmi_rf4ce, cdev);

	// Enforce single-open behavior

	if (rf4ce->open_flag) {
		return -EBUSY; 
	}
	rf4ce->open_flag = 1;

	// Save rf4ce_dev pointer for later.

	file->private_data = rf4ce;
	return 0;

}

// release
int cntl_release(struct inode *inode, struct file *file)
{	
	struct bmi_rf4ce *rf4ce;

	rf4ce = (struct bmi_rf4ce *)(file->private_data);
	rf4ce->open_flag = 0;
	return 0;
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct bmi_rf4ce *rf4ce;
	int slot;

	rf4ce = (struct bmi_rf4ce *)(file->private_data);

	// error if rf4ce not present
	if(rf4ce->bdev == 0)
		return -ENODEV;
	
	slot = rf4ce->bdev->slot->slotnum;

	// ioctl's
	switch (cmd) {

	case BMI_RF4CE_RLEDOFF:
	  bmi_slot_gpio_set_value (slot, RF4CE_GPIO_RED_LED, 1); // Red LED=OFF 
		break;

	case BMI_RF4CE_RLEDON:
	  bmi_slot_gpio_set_value (slot, RF4CE_GPIO_RED_LED, 0); // Red LED=ON 
		break;

	case BMI_RF4CE_GLEDOFF:
	  bmi_slot_gpio_set_value (slot, RF4CE_GPIO_GREEN_LED, 1); // Green LED=OFF 
	  break;

	case BMI_RF4CE_GLEDON:
	  bmi_slot_gpio_set_value (slot, RF4CE_GPIO_GREEN_LED, 0); // Green LED=ON
	  break;
	case BMI_RF4CE_RESET:
		bmi_rf4ce_reset(rf4ce->bdev);
		break;

	case BMI_RF4CE_PROG:
		bmi_rf4ce_prog(rf4ce->bdev);
		break;
	case BMI_RF4CE_SUSPEND:
		rf4ce->bdev->dev.bus->pm->suspend(&rf4ce->bdev->dev);
		break;
	case BMI_RF4CE_RESUME:
		rf4ce->bdev->dev.bus->pm->resume(&rf4ce->bdev->dev);
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

// sysfs attribute - reset the PIC on the rf4ce module
static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bmi_device *bdev;
	bdev = to_bmi_device(dev);	
	if (strlen(buf) > 0){
		bmi_rf4ce_reset(bdev);
	}
	return count;
}

static void bmi_rf4ce_reset(struct bmi_device *bdev)
{
	//asserting GPIO_0 will ground MCLR on the PIC
	printk(KERN_DEBUG "rf4ce - reset assert\n");
	bmi_slot_gpio_set_value (bdev->slot->slotnum, GPIO_0, 1);
	//experimentally, should be >200ms - we've padded it
	msleep(1000);
	printk(KERN_DEBUG "rf4ce - reset clear\n");
	bmi_slot_gpio_set_value (bdev->slot->slotnum, GPIO_0, 0);
}

static DEVICE_ATTR(reset, 0664, NULL, reset_store);

// sysfs attribute - reset the pic and enable the bootloader
static ssize_t boot_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bmi_device *bdev;
	bdev = to_bmi_device(dev);	
	if (strlen(buf) > 0){
		bmi_rf4ce_prog(bdev);	
	}
	return count;
}

static void bmi_rf4ce_prog(struct bmi_device *bdev)
{
	printk(KERN_DEBUG "rf4ce - reset assert\n");
	bmi_slot_gpio_set_value (bdev->slot->slotnum, GPIO_0, 1);
	msleep(1000);
	printk(KERN_DEBUG "rf4ce - prog assert, reset clear\n");
	bmi_slot_gpio_set_value (bdev->slot->slotnum, GPIO_1, 1);
	// SW2 fall time measured at 75.2ns - this should be plenty
	udelay(2);
	bmi_slot_gpio_set_value (bdev->slot->slotnum, GPIO_0, 0);
	// no measurement or test for this value - just padding 
	msleep(500);
	printk(KERN_DEBUG "rf4ce - prog clear\n");
	bmi_slot_gpio_set_value (bdev->slot->slotnum, GPIO_1, 0);
}

static DEVICE_ATTR(enable_bootloader, 0664, NULL, boot_store);

/*
 * 	BMI functions
 */

// probe - insert PIM
int bmi_rf4ce_probe(struct bmi_device *bdev)
{
	int err;
	int slot;
	struct bmi_rf4ce *rf4ce;
	struct cdev *cdev;
	struct class *bmi_class;
	dev_t dev_id;
	int irq;

	err = 0;
	slot = bdev->slot->slotnum;
	rf4ce = &bmi_rf4ce[slot];

	rf4ce->bdev = 0;
	rf4ce->open_flag = 0;
	
	// Create 1 minor device
	cdev = &rf4ce->cdev;
	cdev_init (cdev, &cntl_fops);

	dev_id = MKDEV(major, slot); 
	err = cdev_add (cdev, dev_id, 1);
	if (err) {
		return err;
	}

	// Create class device 
	bmi_class = bmi_get_class ();                            
	rf4ce->class_dev = device_create (bmi_class, NULL, MKDEV (major, slot), NULL, "bmi_rf4ce_control_m%i", slot);  
								     
	if (IS_ERR(rf4ce->class_dev)) {                                
		printk (KERN_ERR "Unable to create "                  
		       "class_device for bmi_rf4ce_m%i; errno = %ld\n",
		       slot, PTR_ERR(rf4ce->class_dev));             
		rf4ce->class_dev = NULL;                               
		cdev_del (&rf4ce->cdev);
		return -ENODEV;
	}                                                            

	// bind driver and bmi_device 
	rf4ce->bdev = bdev;

	err = sysfs_create_file(&bdev->dev.kobj, &dev_attr_reset.attr);
	err = sysfs_create_file(&bdev->dev.kobj, &dev_attr_enable_bootloader.attr);
	if (err < 0){
		printk(KERN_ERR "Unable to add sysfs attribute \n");
	}

	printk (KERN_INFO "bmi_rf4ce.c: probe slot %d\n", slot);

	bmi_device_set_drvdata (bdev, rf4ce);
	// configure IOX
	  
	// Initialize GPIOs (turn LED's on)
	bmi_slot_gpio_direction_out (slot, RED_LED, 0);	// Red LED=ON
	bmi_slot_gpio_direction_out (slot, GREEN_LED, 0);	// Red LED=ON
	bmi_slot_gpio_direction_out (slot, GPIO_0, 0);	// Red LED=ON
	bmi_slot_gpio_direction_out (slot, GPIO_1, 0);	// Red LED=ON
	
	mdelay(200);
	
	// turn LED's off
	bmi_slot_gpio_set_value (slot, RED_LED, 1);
	bmi_slot_gpio_set_value (slot, GREEN_LED, 1);		// Red, Green LED=OFF

	// request PIM interrupt
	irq = bdev->slot->status_irq;
	sprintf (rf4ce->int_name, "bmi_rf4ce%d", slot);
	if (request_irq(irq, &module_irq_handler, IRQF_TRIGGER_FALLING, rf4ce->int_name, rf4ce)) {
		printk (KERN_ERR "bmi_rf4ce.c: Can't allocate irq %d or find rf4ce in slot %d\n", 
			irq, slot); 
		//bmi_device_spi_cleanup(bdev);
		goto err1;

		//return -EBUSY;
	}
	return 0;

 err1:	
	rf4ce->class_dev = NULL;                               
	cdev_del (&rf4ce->cdev);
	device_destroy (bmi_class, MKDEV(major, slot));
	bmi_device_set_drvdata (bdev, 0);
	rf4ce->bdev = 0;
	return -ENODEV;
}

// remove PIM
void bmi_rf4ce_remove(struct bmi_device *bdev)
{	
	int slot;
	struct bmi_rf4ce *rf4ce;
	struct class *bmi_class;
	int irq;
	int i;

	printk(KERN_INFO "bmi_rf4ce: Module Removed...\n");
	slot = bdev->slot->slotnum;
	rf4ce = &bmi_rf4ce[slot];

	irq = bdev->slot->status_irq;
	free_irq (irq, rf4ce);

	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	rf4ce->class_dev = 0;

	cdev_del (&rf4ce->cdev);

	sysfs_remove_file(&bdev->dev.kobj, &dev_attr_reset.attr);
	sysfs_remove_file(&bdev->dev.kobj, &dev_attr_enable_bootloader.attr);

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	rf4ce->bdev = 0;

	return;
}

/*
 *	PM routines
 */

int bmi_rf4ce_resume(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_rf4ce: Resume..\n");
	return 0;
}

int bmi_rf4ce_suspend(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_rf4ce: Suspend..\n");
	return 0;
}

/*
 *	module routines
 */

static void __exit bmi_rf4ce_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver (&bmi_rf4ce_driver);

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}

static int __init bmi_rf4ce_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI RF4CE Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_rf4ce_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}

	printk (KERN_INFO "bmi_rf4ce.c: BMI_RF4CE Driver v%s \n", BMIRF4CE_VERSION);

	return 0;
}


module_init(bmi_rf4ce_init);
module_exit(bmi_rf4ce_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Buglabs Inc.");
MODULE_DESCRIPTION("BMI rf4ce device driver");
MODULE_SUPPORTED_DEVICE("bmi_rf4ce_control_mX");

