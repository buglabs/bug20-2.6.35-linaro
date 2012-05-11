/*
 * 	bmi_battery.c
 *
 * 	BMI battery device driver basic functionality
 *  taken from Von Hippel driver for expediency
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

#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/hardware.h>

#include <linux/i2c.h>

#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_battery.h>

#define BMIBATTERY_VERSION	"0.3"

/*
 * 	Global variables
 */

//static struct i2c_board_info iox_info = {
//  I2C_BOARD_INFO("BATTERY_IOX", BMI_IOX_I2C_ADDRESS),
//};

// private device structure
struct bmi_battery
{
  struct bmi_device	*bdev;			// BMI device
  struct cdev		cdev;			// control device
  struct device	*class_dev;		// control class device
  int			open_flag;		// single open flag
  char			int_name[20];		// interrupt name
//  struct i2c_client *iox;
};

static struct bmi_battery bmi_battery[4];	// per slot device structure
static int major;		// control device major

/*
 * 	BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_battery_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_BATTERY, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },	  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_battery_tbl);

int	bmi_battery_probe (struct bmi_device *bdev);
void	bmi_battery_remove (struct bmi_device *bdev);
int	bmi_battery_resume (struct device *dev);
int	bmi_battery_suspend (struct device *dev);

static struct dev_pm_ops bmi_battery_pm =
{
	.resume = bmi_battery_resume,
	.suspend = bmi_battery_suspend,
};

// BMI driver structure
static struct bmi_driver bmi_battery_driver = 
{
	.name = "bmi_battery", 
	.id_table = bmi_battery_tbl, 
	.probe   = bmi_battery_probe, 
	.remove  = bmi_battery_remove, 
	.pm  = &bmi_battery_pm,
};

/*
 * 	I2C set up
 */

// IOX
// read byte from I2C IO expander
/*static int ReadByte_IOX (struct i2c_client *client, unsigned char offset, unsigned char *data)
{
	int	ret = 0;	

	ret = i2c_master_send(client, &offset, 1);
	if (ret == 1)
	  ret = i2c_master_recv(client, data, 1);
	if (ret < 0)
	  printk (KERN_ERR "ReadByte_IOX() - i2c_transfer() failed...%d\n",ret);
	return ret;
}

// write byte to I2C IO expander
static int WriteByte_IOX (struct i2c_client *client, unsigned char offset, unsigned char data)
{
	int	ret = 0;
	unsigned char msg[2];
	
     	msg[0] = offset;
	msg[1] = data;
	ret = i2c_master_send(client, msg, sizeof(msg));
	
	if (ret < 0)
	  printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed...%d\n",ret);

	return ret;
}*/

/*
 *	control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *file)
{	
	struct bmi_battery *battery;

	battery = container_of (inode->i_cdev, struct bmi_battery, cdev);

	// Enforce single-open behavior

	if (battery->open_flag) {
		return -EBUSY; 
	}
	battery->open_flag = 1;

	// Save battery_dev pointer for later.

	file->private_data = battery;
	return 0;

}

// release
int cntl_release(struct inode *inode, struct file *file)
{	
	struct bmi_battery *battery;

	battery = (struct bmi_battery *)(file->private_data);
	battery->open_flag = 0;
	return 0;
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct i2c_adapter *adap;
//	unsigned char iox_data;
	int ret = 0;

	struct bmi_battery *battery;
	int slot;

	battery = (struct bmi_battery *)(file->private_data);

	// error if battery not present
	if(battery->bdev == 0)
		return -ENODEV;
	
	slot = battery->bdev->slot->slotnum;
	adap = battery->bdev->slot->adap;

	// ioctl's
	switch (cmd) {

	case BMI_BATTERY_MKGPIO_OUT:
		if ((arg < BATTERY_GPIO_0) || (arg > BATTERY_GPIO_3))
			return -EINVAL;
		bmi_slot_gpio_direction_out (slot, arg, 1);
		break;

	case BMI_BATTERY_MKGPIO_IN:
		if ((arg < BATTERY_GPIO_0) || (arg > BATTERY_GPIO_3))
			return -EINVAL;
		bmi_slot_gpio_direction_in (slot, arg);
		break;

	case BMI_BATTERY_SETGPIO:
		if ((arg < BATTERY_GPIO_0) || (arg > BATTERY_GPIO_3))
			return -EINVAL;
		bmi_slot_gpio_set_value (slot, arg, 0x1);
		break;

	case BMI_BATTERY_CLRGPIO:
		if ((arg < BATTERY_GPIO_0) || (arg > BATTERY_GPIO_3))
			return -EINVAL;
		bmi_slot_gpio_set_value (slot, arg, 0x0);
		break;
		
	case BMI_BATTERY_PRESENT:
		if ((arg < BATTERY_GPIO_0) || (arg > BATTERY_GPIO_1))
			return -EINVAL;
		return bmi_slot_gpio_get_value(slot, arg);
		
	case BMI_BATTERY_SUSPEND:
		battery->bdev->dev.bus->pm->suspend(&battery->bdev->dev);
		break;
	case BMI_BATTERY_RESUME:
		battery->bdev->dev.bus->pm->resume(&battery->bdev->dev);
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

/*
 * 	BMI functions
 */

// probe - insert PIM
int bmi_battery_probe(struct bmi_device *bdev)
{
	int err;
	int slot;
	struct bmi_battery *battery;
       	struct i2c_adapter *adap;
	struct cdev *cdev;
	struct class *bmi_class;
	dev_t dev_id;
	int irq;

	err = 0;
	slot = bdev->slot->slotnum;
      	adap = bdev->slot->adap;
	battery = &bmi_battery[slot];

	battery->bdev = 0;
	battery->open_flag = 0;
	
	// Create 1 minor device
	cdev = &battery->cdev;
	cdev_init (cdev, &cntl_fops);

	dev_id = MKDEV(major, slot); 
	err = cdev_add (cdev, dev_id, 1);
	if (err) {
		return err;
	}

	// Create class device 
	bmi_class = bmi_get_class ();                            
	battery->class_dev = device_create (bmi_class, NULL, MKDEV (major, slot), NULL, "bmi_battery_control_m%i", slot);  
								     
	if (IS_ERR(battery->class_dev)) {                                
		printk (KERN_ERR "Unable to create "                  
		       "class_device for bmi_battery_m%i; errno = %ld\n",
		       slot, PTR_ERR(battery->class_dev));             
		battery->class_dev = NULL;                               
		cdev_del (&battery->cdev);
		return -ENODEV;
	}                                                            

	// bind driver and bmi_device 
	battery->bdev = bdev;
	

	printk (KERN_INFO "bmi_battery.c: probe slot %d\n", slot);
//	battery->iox = i2c_new_device(bdev->slot->adap, &iox_info);
//	if (battery->iox == NULL)
//	  printk(KERN_ERR "IOX NULL...\n");

	bmi_device_set_drvdata (bdev, battery);
	
	// set battery detect lines as input, no pullup
	bmi_slot_gpio_direction_in(slot, GPIO_0);
	bmi_slot_gpio_direction_in(slot, GPIO_1);	
	bmi_slot_gpio_set_value (slot, GPIO_0, 0);
	bmi_slot_gpio_set_value (slot, GPIO_1, 0);
	
	//set AC_EN gpio line, then unbind it so that userland can play with it
	err = gpio_request(AC_EN_GPIO, "bug_battery");
	if (err) {
		printk(KERN_ERR "bmi_battery.c: Can't bind to AC_EN GPIO - battery module may not charge'");
	} else {
		printk(KERN_ERR "bmi_battery.c: Enabling module charge, gpio %d", AC_EN_GPIO);
		gpio_direction_output(AC_EN_GPIO, 1);
		gpio_free(AC_EN_GPIO);
	}
	
	

	// request PIM interrupt
	irq = bdev->slot->status_irq;
	sprintf (battery->int_name, "bmi_battery%d", slot);
	if (request_irq(irq, &module_irq_handler, IRQF_TRIGGER_FALLING, battery->int_name, battery)) {
		printk (KERN_ERR "bmi_battery.c: Can't allocate irq %d or find battery in slot %d\n", 
			irq, slot); 
		//bmi_device_spi_cleanup(bdev);
		goto err1;

		//return -EBUSY;
	}
	return 0;

 err1:	
	battery->class_dev = NULL;                               
	cdev_del (&battery->cdev);
	device_destroy (bmi_class, MKDEV(major, slot));
	bmi_device_set_drvdata (bdev, 0);
	battery->bdev = 0;
//	i2c_unregister_device(battery->iox);
	return -ENODEV;
}

// remove PIM
void bmi_battery_remove(struct bmi_device *bdev)
{	
	int slot;
	struct bmi_battery *battery;
	struct class *bmi_class;
	int irq;
	int i;

	printk(KERN_INFO "bmi_battery: Module Removed...\n");
	slot = bdev->slot->slotnum;
	battery = &bmi_battery[slot];

//	i2c_unregister_device(battery->iox);

	irq = bdev->slot->status_irq;
	free_irq (irq, battery);

	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	battery->class_dev = 0;

	cdev_del (&battery->cdev);

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	battery->bdev = 0;

	return;
}

/*
 *	PM routines
 */

int bmi_battery_resume(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_battery: Resume..\n");
	bmi_slot_uart_enable(bmi_dev->slot->slotnum);
	return 0;
}

int bmi_battery_suspend(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_battery: Suspend..\n");
	bmi_slot_uart_disable(bmi_dev->slot->slotnum);
	return 0;
}

/*
 *	module routines
 */

static void __exit bmi_battery_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver (&bmi_battery_driver);

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}

static int __init bmi_battery_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI BATTERY Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_battery_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}

	printk (KERN_INFO "bmi_battery.c: BMI_BATTERY Driver v%s \n", BMIBATTERY_VERSION);

	return 0;
}


module_init(bmi_battery_init);
module_exit(bmi_battery_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Buglabs Inc.");
MODULE_DESCRIPTION("BMI Battery device driver");
MODULE_SUPPORTED_DEVICE("bmi_battery_control_mX");

