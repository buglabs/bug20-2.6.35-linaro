/*
 * 	bmi_rfid.c
 *
 * 	BMI RFID device driver basic functionality
 * 	Copied from the VonHippel PIM driver
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
#include <linux/bmi/bmi_rfid.h>

#define BMIRFID_VERSION	"1.0"
#define CARD_LEN 	10

/*
 * 	Global variables
 */

struct bmi_rfid
{
  struct bmi_device	*bdev;			// BMI device
  struct cdev		cdev;			// control device
  struct device	*class_dev;		// control class device
  int			open_flag;		// single open flag
  char			int_name[20];		// interrupt name
  char 			last_card[CARD_LEN];	// Last card ID read from device
};

static struct bmi_rfid bmi_rfid[4];	// per slot device structure
static int major;		// control device major

/*
 * 	BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_rfid_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_RFID, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },	  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_rfid_tbl);

int	bmi_rfid_probe (struct bmi_device *bdev);
void	bmi_rfid_remove (struct bmi_device *bdev);
int	bmi_rfid_resume (struct device *dev);
int	bmi_rfid_suspend (struct device *dev);
static void	bmi_rfid_reset(struct bmi_device *bdev);
static ssize_t	bmi_rfid_card(struct bmi_rfid *rfid);

static struct dev_pm_ops bmi_rfid_pm =
{
	.resume = bmi_rfid_resume,
	.suspend = bmi_rfid_suspend,
};

// BMI driver structure
static struct bmi_driver bmi_rfid_driver = 
{
	.name = "bmi_rfid", 
	.id_table = bmi_rfid_tbl, 
	.probe   = bmi_rfid_probe, 
	.remove  = bmi_rfid_remove, 
	.pm  = &bmi_rfid_pm,
};

/*
 *	control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *file)
{	
	struct bmi_rfid *rfid;

	rfid = container_of (inode->i_cdev, struct bmi_rfid, cdev);

	// Enforce single-open behavior

	if (rfid->open_flag) {
		return -EBUSY; 
	}
	rfid->open_flag = 1;

	// Save rfid_dev pointer for later.

	file->private_data = rfid;
	return 0;

}

// release
int cntl_release(struct inode *inode, struct file *file)
{	
	struct bmi_rfid *rfid;

	rfid = (struct bmi_rfid *)(file->private_data);
	rfid->open_flag = 0;
	return 0;
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct bmi_rfid *rfid;
	int slot;

	rfid = (struct bmi_rfid *)(file->private_data);

	// error if rfid not present
	if(rfid->bdev == 0)
		return -ENODEV;
	
	slot = rfid->bdev->slot->slotnum;

	// ioctl's
	switch (cmd) {

	case BMI_RFID_RLEDOFF:
	  bmi_slot_gpio_set_value (slot, RFID_GPIO_RED_LED, 1); // Red LED=OFF 
		break;

	case BMI_RFID_RLEDON:
	  bmi_slot_gpio_set_value (slot, RFID_GPIO_RED_LED, 0); // Red LED=ON 
		break;

	case BMI_RFID_RESET:
		bmi_rfid_reset(rfid->bdev);
		break;
	case BMI_RFID_CARD:
		bmi_rfid_card(rfid);
		break;

	case BMI_RFID_SUSPEND:
		rfid->bdev->dev.bus->pm->suspend(&rfid->bdev->dev);
		break;
	case BMI_RFID_RESUME:
		rfid->bdev->dev.bus->pm->resume(&rfid->bdev->dev);
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

// sysfs attribute - reset the PIC on the rfid module
static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bmi_device *bdev;
	bdev = to_bmi_device(dev);	
	if (strlen(buf) > 0){
		bmi_rfid_reset(bdev);
	}
	return count;
}

static void bmi_rfid_reset(struct bmi_device *bdev)
{
	//asserting GPIO_0 will ground MCLR on the PIC
	printk(KERN_DEBUG "rfid - reset assert\n");
	bmi_slot_gpio_set_value (bdev->slot->slotnum, GPIO_0, 1);
	//experimentally, should be >200ms - we've padded it
	msleep(1000);
	printk(KERN_DEBUG "rfid - reset clear\n");
	bmi_slot_gpio_set_value (bdev->slot->slotnum, GPIO_0, 0);
}

static DEVICE_ATTR(reset, 0664, NULL, reset_store);

// sysfs attribute - read from the RFID card 
static ssize_t card_show(struct device *dev, struct device_attribute *attr,
			const char *buf)
{
	struct bmi_device *bdev;
	struct bmi_rfid *rfid;
	ssize_t ret = 0;
	bdev = to_bmi_device(dev);
	rfid = (struct bmi_rfid *)bmi_device_get_drvdata(bdev);
	ret = bmi_rfid_card(rfid);
	//TODO - figure out what to do if we don't actually retrieve a new card
	//AKA, if ret<0, figure out if we still want to return the last card.
	ret = scnprintf(buf, PAGE_SIZE, "%s\r\n", rfid->last_card);
	return ret;
}

static ssize_t bmi_rfid_card(struct bmi_rfid *rfid)
{
	//asserting GPIO_0 will ground MCLR on the PIC
	printk(KERN_DEBUG "rfid - read from card (NOT FULLY IMPLEMENTED)\n");
	strcpy(rfid->last_card, "HIMOM");
	//TODO - intelligent return value if device can't be opened...
	return 0;
}

static DEVICE_ATTR(card, S_IRUGO, card_show, NULL);
/*
 * 	BMI functions
 */

// probe - insert PIM
int bmi_rfid_probe(struct bmi_device *bdev)
{
	int err;
	int slot;
	struct bmi_rfid *rfid;
	struct cdev *cdev;
	struct class *bmi_class;
	dev_t dev_id;
	int irq;

	err = 0;
	slot = bdev->slot->slotnum;
	rfid = &bmi_rfid[slot];

	rfid->bdev = 0;
	rfid->open_flag = 0;

	memset(rfid->last_card, '\0', CARD_LEN);
	
	// Create 1 minor device
	cdev = &rfid->cdev;
	cdev_init (cdev, &cntl_fops);

	dev_id = MKDEV(major, slot); 
	err = cdev_add (cdev, dev_id, 1);
	if (err) {
		return err;
	}

	// Create class device 
	bmi_class = bmi_get_class ();                            
	rfid->class_dev = device_create (bmi_class, NULL, MKDEV (major, slot), NULL, "bmi_rfid_control_m%i", slot);  
								     
	if (IS_ERR(rfid->class_dev)) {                                
		printk (KERN_ERR "Unable to create "                  
		       "class_device for bmi_rfid_m%i; errno = %ld\n",
		       slot, PTR_ERR(rfid->class_dev));             
		rfid->class_dev = NULL;                               
		cdev_del (&rfid->cdev);
		return -ENODEV;
	}                                                            

	// bind driver and bmi_device 
	rfid->bdev = bdev;

	err = sysfs_create_file(&bdev->dev.kobj, &dev_attr_reset.attr);
	err = sysfs_create_file(&bdev->dev.kobj, &dev_attr_card.attr);
	if (err < 0){
		printk(KERN_ERR "Unable to add sysfs attribute \n");
	}

	printk (KERN_INFO "bmi_rfid.c: probe slot %d\n", slot);

	bmi_device_set_drvdata (bdev, rfid);
	// configure IOX
	  
	// Initialize GPIOs (turn LED's on)
	bmi_slot_gpio_direction_out (slot, RED_LED, 0);	// Red LED=ON
	bmi_slot_gpio_direction_out (slot, GPIO_0, 0);	// Red LED=ON
	bmi_slot_gpio_direction_out (slot, GPIO_1, 0);	// Red LED=ON
	
	mdelay(200);
	
	// turn LED's off
	bmi_slot_gpio_set_value (slot, RED_LED, 1);

	// request PIM interrupt
	irq = bdev->slot->status_irq;
	sprintf (rfid->int_name, "bmi_rfid%d", slot);
	if (request_irq(irq, &module_irq_handler, IRQF_TRIGGER_FALLING, rfid->int_name, rfid)) {
		printk (KERN_ERR "bmi_rfid.c: Can't allocate irq %d or find rfid in slot %d\n", 
			irq, slot); 
		//bmi_device_spi_cleanup(bdev);
		goto err1;

		//return -EBUSY;
	}
	return 0;

 err1:	
	rfid->class_dev = NULL;                               
	cdev_del (&rfid->cdev);
	device_destroy (bmi_class, MKDEV(major, slot));
	bmi_device_set_drvdata (bdev, 0);
	rfid->bdev = 0;
	return -ENODEV;
}

// remove PIM
void bmi_rfid_remove(struct bmi_device *bdev)
{	
	int slot;
	struct bmi_rfid *rfid;
	struct class *bmi_class;
	int irq;
	int i;

	printk(KERN_INFO "bmi_rfid: Module Removed...\n");
	slot = bdev->slot->slotnum;
	rfid = &bmi_rfid[slot];

	irq = bdev->slot->status_irq;
	free_irq (irq, rfid);

	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	rfid->class_dev = 0;

	cdev_del (&rfid->cdev);

	sysfs_remove_file(&bdev->dev.kobj, &dev_attr_reset.attr);
	sysfs_remove_file(&bdev->dev.kobj, &dev_attr_card.attr);

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	rfid->bdev = 0;

	return;
}

/*
 *	PM routines
 */

int bmi_rfid_resume(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_rfid: Resume..\n");
	return 0;
}

int bmi_rfid_suspend(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_rfid: Suspend..\n");
	return 0;
}

/*
 *	module routines
 */

static void __exit bmi_rfid_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver (&bmi_rfid_driver);

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}

static int __init bmi_rfid_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI RFID Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_rfid_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}

	printk (KERN_INFO "bmi_rfid.c: BMI_RFID Driver v%s \n", BMIRFID_VERSION);

	return 0;
}


module_init(bmi_rfid_init);
module_exit(bmi_rfid_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Buglabs Inc.");
MODULE_DESCRIPTION("BMI rfid device driver");
MODULE_SUPPORTED_DEVICE("bmi_rfid_control_mX");

