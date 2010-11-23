/*
 * 	bmi_acnt.c
 *
 * 	Accenture BMI serial/accelerometer board device driver basic functionality
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

#include <linux/input.h>
#include <linux/input/adxl34x.h>

#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_accnt.h>

/*
 * 	Global variables
 */
#define BMI_ACC_I2C_ADDRESS 0x1D

static struct adxl34x_platform_data acc_plat_data = {
	.x_axis_offset = 0,
	.y_axis_offset = 0,
	.z_axis_offset = 0,
	.tap_threshold = 0x31,
	.tap_duration = 0x10,
	.tap_latency = 0x60,
	.tap_window = 0xF0,
	.tap_axis_control = ADXL_TAP_X_EN | ADXL_TAP_Y_EN | ADXL_TAP_Z_EN,
	.act_axis_control = 0xFF,
	.activity_threshold = 5,
	.inactivity_threshold = 3,
	.inactivity_time = 4,
	.free_fall_threshold = 0x7,
	.free_fall_time = 0x20,
	.data_rate = 0x8,
	.data_range = ADXL_FULL_RES,
 
	.ev_type = EV_ABS,
	.ev_code_x = ABS_X,		/* EV_REL */
	.ev_code_y = ABS_Y,		/* EV_REL */
	.ev_code_z = ABS_Z,		/* EV_REL */
 
	.ev_code_x = BTN_TOUCH,
	.ev_code_y = BTN_TOUCH,
	.ev_code_z = BTN_TOUCH,

 
	.ev_code_ff = KEY_F,
	.ev_code_act_inactivity = KEY_A,
	.power_mode = 0, //ADXL_AUTO_SLEEP | ADXL_LINK,
	.fifo_mode = ADXL_FIFO_STREAM,
};

static struct i2c_board_info acc_info = {
    I2C_BOARD_INFO("adxl34x", BMI_ACC_I2C_ADDRESS),
    .platform_data = &acc_plat_data,
};


// private device structure
struct bmi_accnt
{
  struct bmi_device	*bdev;			// BMI device
  struct cdev		cdev;			// control device
  struct device	        *class_dev;		// control class device
  int			open_flag;		// single open flag
  char			int_name[20];		// interrupt name
  struct i2c_client     *acc;
/*  
  wait_queue_head_t     acc_wait1_queue;        // Accelerometer interrupt wait queue
  unsigned char         acc_int1_en;            // Accelerometer interrupts are enabled
  unsigned char         acc_int1_fl;            // Accelerometer interrupt occurred/
  wait_queue_head_t     acc_wait2_queue;        // Accelerometer interrupt wait queue
  unsigned char         acc_int2_en;            // Accelerometer interrupts are enabled
  unsigned char         acc_int2_fl;            // Accelerometer interrupt occurred
*/
};

static struct bmi_accnt bmi_accnt[4];	// per slot device structure
static int major;		// control device major

/*
 * 	BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_accnt_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_ACCNT, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },	  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_accnt_tbl);

int	bmi_accnt_probe (struct bmi_device *bdev);
void	bmi_accnt_remove (struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_accnt_driver = 
{
	.name = "bmi_accnt", 
	.id_table = bmi_accnt_tbl, 
	.probe   = bmi_accnt_probe, 
	.remove  = bmi_accnt_remove, 
};

/*
 * 	I2C set up
 */



/*
 *	control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *file)
{	
	struct bmi_accnt *accnt;

	accnt = container_of (inode->i_cdev, struct bmi_accnt, cdev);

	// Enforce single-open behavior

	if (accnt->open_flag) {
		return -EBUSY; 
	}
	accnt->open_flag = 1;

	// Save accnt_dev pointer for later.

	file->private_data = accnt;
	return 0;

}

// release
int cntl_release(struct inode *inode, struct file *file)
{	
	struct bmi_accnt *accnt;

	accnt = (struct bmi_accnt *)(file->private_data);
	accnt->open_flag = 0;
	return 0;
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct i2c_adapter *adap;

	struct bmi_accnt *accnt;
	int slot;

	accnt = (struct bmi_accnt *)(file->private_data);

	// error if accnt not present
	if(accnt->bdev == 0)
		return -ENODEV;
	
	slot = accnt->bdev->slot->slotnum;
	adap = accnt->bdev->slot->adap;

	// ioctl's
	switch (cmd) {

	case BMI_ACCNT_RLEDOFF:
	  bmi_slot_gpio_set_value (slot, ACCNT_GPIO_RED_LED, 1); // Red LED=OFF 
		break;

	case BMI_ACCNT_RLEDON:
	  bmi_slot_gpio_set_value (slot, ACCNT_GPIO_RED_LED, 0); // Red LED=ON 
		break;

	case BMI_ACCNT_GLEDOFF:
	  bmi_slot_gpio_set_value (slot, ACCNT_GPIO_GREEN_LED, 1); // Green LED=OFF 
	  break;

	case BMI_ACCNT_GLEDON:
	  bmi_slot_gpio_set_value (slot, ACCNT_GPIO_GREEN_LED, 0); // Green LED=ON
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
int bmi_accnt_probe(struct bmi_device *bdev)
{
	int err;
	int slot;
	struct bmi_accnt *accnt;
       	struct i2c_adapter *adap;
	struct cdev *cdev;
	struct class *bmi_class;
	dev_t dev_id;
	int irq;

	err = 0;
	slot = bdev->slot->slotnum;
      	adap = bdev->slot->adap;
	accnt = &bmi_accnt[slot];

	accnt->bdev = 0;
	accnt->open_flag = 0;
	
	// Create 1 minor device
	cdev = &accnt->cdev;
	cdev_init (cdev, &cntl_fops);

	dev_id = MKDEV(major, slot); 
	err = cdev_add (cdev, dev_id, 1);
	if (err) {
		return err;
	}

	// Create class device 
	bmi_class = bmi_get_class ();                            
	accnt->class_dev = device_create (bmi_class, NULL, MKDEV (major, slot), NULL, "bmi_accnt_control_m%i", slot+1);  
								     
	if (IS_ERR(accnt->class_dev)) {                                
		printk (KERN_ERR "Unable to create "                  
		       "class_device for bmi_accnt_m%i; errno = %ld\n",
		       slot+1, PTR_ERR(accnt->class_dev));             
		accnt->class_dev = NULL;                               
		cdev_del (&accnt->cdev);
		return -ENODEV;
	}                                                            

	// bind driver and bmi_device 
	accnt->bdev = bdev;
	

	printk (KERN_INFO "bmi_accnt.c: probe slot %d\n", slot);

	bmi_device_set_drvdata (bdev, accnt);

	  
	// Initialize GPIOs (turn LED's on)
	bmi_slot_gpio_direction_out (slot, RED_LED, 0);	// Red LED=ON
	bmi_slot_gpio_direction_out (slot, GREEN_LED, 0);	// Red LED=ON
	
	mdelay(200);
	
	// turn LED's off
	bmi_slot_gpio_set_value (slot, RED_LED, 1);
	bmi_slot_gpio_set_value (slot, GREEN_LED, 1);		// Red, Green LED=OFF 		

	// add usb dependancy
	increment_usb_dep();

	irq = bdev->slot->status_irq;
	acc_info.irq = irq;
	accnt->acc = i2c_new_device(bdev->slot->adap, &acc_info);
	// request PIM interrupt
	sprintf (accnt->int_name, "bmi_accnt%d", slot);
	/*
	if (request_irq(irq, &module_irq_handler, 0, accnt->int_name, accnt)) {
		printk (KERN_ERR "bmi_accnt.c: Can't allocate irq %d or find von Hippel in slot %d\n", 
			irq, slot); 
		//bmi_device_spi_cleanup(bdev);
		goto err1;

		//return -EBUSY;
	}
	*/
	return 0;

 err1:	
	accnt->class_dev = NULL;                               
	cdev_del (&accnt->cdev);
	device_destroy (bmi_class, MKDEV(major, slot));
	bmi_device_set_drvdata (bdev, 0);
	accnt->bdev = 0;
	return -ENODEV;
}

// remove PIM
void bmi_accnt_remove(struct bmi_device *bdev)
{	
	int slot;
	struct bmi_accnt *accnt;
	struct class *bmi_class;
	int irq;
	int i;

	printk(KERN_INFO "bmi_accnt: Module Removed...\n");
	slot = bdev->slot->slotnum;
	accnt = &bmi_accnt[slot];

	irq = bdev->slot->status_irq;

	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	accnt->class_dev = 0;

	cdev_del (&accnt->cdev);
	i2c_unregister_device(accnt->acc);

	// remove usb dependency
	decrement_usb_dep();

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	accnt->bdev = 0;

	return;
}

/*
 *	module routines
 */

static void __exit bmi_accnt_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver (&bmi_accnt_driver);

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}

static int __init bmi_accnt_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI ACCNT Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_accnt_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}

	printk (KERN_INFO "bmi_accnt.c: BMI_ACCNT Driver\n");

	return 0;
}


module_init(bmi_accnt_init);
module_exit(bmi_accnt_cleanup);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Buglabs Inc.");
MODULE_DESCRIPTION("BMI Acclerometer device driver");
MODULE_SUPPORTED_DEVICE("bmi_accnt_control_mX");

