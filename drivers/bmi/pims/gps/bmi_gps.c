/*
 * 	bmi_gps.c
 *
 * 	BMI gps device driver
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
 * Include files
 */


//REWORK: Which are not needed ?
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/bmi.h>

#include <linux/bmi/bmi_gps.h>

//MTW
#include <linux/cdev.h>


#define BMIGPS_VERSION	"1.1"


// private device structure
struct bmi_gps
{
	struct bmi_device   *bdev;	// BMI device
	struct cdev 	     cdev;
	struct device *class_dev;
	int		     open_flag;	// single open flag
  struct i2c_client *iox;
};

static struct bmi_gps bmi_gps[4];
static int major;

/*
 * 	BMI set up
 */

	// BMI device ID table
static struct bmi_device_id bmi_gps_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_GPS_J32, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },					  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_gps_tbl);

int	bmi_gps_probe(struct bmi_device *bdev);
void	bmi_gps_remove(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_gps_driver = 
{
	.name = "bmi_gps", 
	.id_table = bmi_gps_tbl, 
	.probe   = bmi_gps_probe, 
	.remove  = bmi_gps_remove, 
};

/*
 * 	I2C set up
 */

// I2C Slave Address
#define BMI_IOX_I2C_ADDRESS	0x71	// 7-bit address

// I2C IOX register addresses
#define IOX_INPUT_REG		0x0
#define IOX_OUTPUT_REG		0x1
#define IOX_POLARITY_REG	0x2
#define IOX_CONTROL		0x3

static struct i2c_board_info iox_info = {
  I2C_BOARD_INFO("GPS_IOX", BMI_IOX_I2C_ADDRESS),
};

// read byte from I2C IO expander

static int ReadByte_IOX (struct i2c_client *client, unsigned char offset, unsigned char *data)
{
	int	ret = 0;	

	ret = i2c_master_send(client, &offset, 1);
	if (ret == 1)
	  ret = i2c_master_recv(client, data, 1);
	if (ret < 0) {
	  printk (KERN_ERR "ReadByte_IOX() - i2c_transfer() failed...%d\n",ret);
	  return ret;
	}
	return 0;
}

// write byte to I2C IO expander
static int WriteByte_IOX (struct i2c_client *client, unsigned char offset, unsigned char data)
{
	int	ret = 0;
	unsigned char msg[2];
	
     	msg[0] = offset;
	msg[1] = data;
	ret = i2c_master_send(client, msg, sizeof(msg));
	
	if (ret < 0) {
	  printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed...%d\n",ret);
	  return ret;
	}

	return 0;
}



/*
 * control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *file)
{	
	struct bmi_gps *gps;

	gps = container_of(inode->i_cdev, struct bmi_gps, cdev);

	// Enforce single-open behavior

	if (gps->open_flag) {
		return -EBUSY; 
	}
	gps->open_flag = 1;

	// Save gps_dev pointer for later.

	file->private_data = gps;
	return 0;

}

// release
int cntl_release(struct inode *inode, struct file *file)
{	
	struct bmi_gps *gps;

	gps = (struct bmi_gps *)(file->private_data);
	gps->open_flag = 0;
	return 0;
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct i2c_adapter *adap;
	unsigned char iox_data;
	
	struct bmi_gps *gps;
	int slot;

	gps = (struct bmi_gps *)(file->private_data);

	// error if gps not present
	if(gps->bdev == 0)
		return -ENODEV;
	
	slot = gps->bdev->slot->slotnum;
	adap = gps->bdev->slot->adap;

		// ioctl's
	switch (cmd) {
	case BMI_GPS_RLEDOFF:
	  bmi_slot_gpio_set_value (slot, RED_LED, 1); // Red LED=OFF 
		break;

	case BMI_GPS_RLEDON:
	  bmi_slot_gpio_set_value (slot, RED_LED, 0); // Red LED=ON 
		break;

	case BMI_GPS_GLEDOFF:
	  bmi_slot_gpio_set_value (slot, GREEN_LED, 1); // Greem LED=OFF 
		break;

	case BMI_GPS_GLEDON:
	  bmi_slot_gpio_set_value (slot, GREEN_LED, 0); // Greem LED=ON
		break;

	case BMI_GPS_SETBOOT:
		if(ReadByte_IOX (gps->iox, IOX_OUTPUT_REG, &iox_data))
			return -ENODEV;
		iox_data |= 0x08;
		if(WriteByte_IOX (gps->iox, IOX_OUTPUT_REG, iox_data))
			return -ENODEV;
		if(ReadByte_IOX (gps->iox, IOX_CONTROL, &iox_data))	// IOX[3]=BOOT=0, IOX[4]=WAKEUP=0
			return -ENODEV;
		iox_data |= 0x08;
		if(WriteByte_IOX (gps->iox, IOX_CONTROL, iox_data))	// IOX[3]=BOOT=0, IOX[4]=WAKEUP=0
			return -ENODEV;
		break;

	case BMI_GPS_CLRBOOT:
		if(ReadByte_IOX (gps->iox, IOX_OUTPUT_REG, &iox_data))
			return -ENODEV;
		iox_data &= ~0x08;
		if(WriteByte_IOX (gps->iox, IOX_OUTPUT_REG, iox_data))
			return -ENODEV;
		break;

	case BMI_GPS_SETWAKE:
		if(ReadByte_IOX (gps->iox, IOX_OUTPUT_REG, &iox_data))
			return -ENODEV;
		iox_data |= 0x10;
		if(WriteByte_IOX (gps->iox, IOX_OUTPUT_REG, iox_data))
			return -ENODEV;
		if(ReadByte_IOX (gps->iox, IOX_CONTROL, &iox_data))	// IOX[3]=BOOT=0, IOX[4]=WAKEUP=0
			return -ENODEV;
		iox_data |= 0x10;
		if(WriteByte_IOX (gps->iox, IOX_CONTROL, iox_data))	// IOX[3]=BOOT=0, IOX[4]=WAKEUP=0
				return -ENODEV;
			break;

	case BMI_GPS_CLRWAKE:
		if(ReadByte_IOX (gps->iox, IOX_OUTPUT_REG, &iox_data))
			return -ENODEV;
		iox_data &= ~0x10;
		if(WriteByte_IOX (gps->iox, IOX_OUTPUT_REG, iox_data))
			return -ENODEV;
		break;

	case BMI_GPS_SETRST:
	  bmi_slot_gpio_set_value (slot, GPIO_1, 0); // RST = 0;

		break;

	case BMI_GPS_CLRRST:
	  bmi_slot_gpio_set_value (slot, GPIO_1, 1); // RST=tristate 
		break;

	case BMI_GPS_GETSTAT:
		{
		int read_data;

		if(ReadByte_IOX (gps->iox, IOX_INPUT_REG, &iox_data))
			return -ENODEV;
		
		read_data = iox_data | (bmi_slot_gpio_get_all(slot) << 8);

		if(put_user(read_data, (int __user *) arg))
			return -EFAULT;
		}
		break;

	case BMI_GPS_ACTIVE_ANT :
		{
		printk(KERN_INFO "bmi_gps: ACTIVE_ANT Called\n");
		if(ReadByte_IOX (gps->iox, IOX_OUTPUT_REG, &iox_data))
			return -ENODEV;
		iox_data &= ~0x40;
		iox_data |=  0x80;
		if(WriteByte_IOX (gps->iox, IOX_OUTPUT_REG, iox_data))
			return -ENODEV;
		printk(KERN_INFO "bmi_gps: ACTIVE_ANT Success\n");
		}
		break;

	case BMI_GPS_PASSIVE_ANT:
		{
		if(ReadByte_IOX (gps->iox, IOX_OUTPUT_REG, &iox_data))
			return -ENODEV;
		iox_data &= ~0x80;
		iox_data |=  0x40;
		if(WriteByte_IOX (gps->iox, IOX_OUTPUT_REG, iox_data))
			return -ENODEV;
		}
		break;

	default:
		return -ENOTTY;
	}

	return 0;
}

// control file operations
struct file_operations gps_cntl_fops = {
	.owner = THIS_MODULE, 
	.ioctl = cntl_ioctl, 
	.open = cntl_open, 
	.release = cntl_release, 
};

/*
 *	Module functions
 */

/*
 * 	BMI functions
 */

int bmi_gps_probe(struct bmi_device *bdev)
{
	int err;
	int slot;
	struct bmi_gps *gps;
	struct i2c_adapter *adap;
	struct cdev *cdev;
	struct class *bmi_class;
	dev_t dev_id;

	err = 0;	
	slot = bdev->slot->slotnum;
	adap = bdev->slot->adap;
	gps = &bmi_gps[slot];

	gps->bdev = 0;
	gps->open_flag = 0;
	
	//Create 1 minor device
	cdev = &gps->cdev;
	cdev_init(cdev, &gps_cntl_fops);

	dev_id = MKDEV(major, slot); 
	err = cdev_add(cdev, dev_id, 1);
	if (err) {
	  return err;
	}

	//Create class device 
	bmi_class = bmi_get_class ();                            
	gps->class_dev = device_create(bmi_class, NULL, MKDEV(major, slot), gps, "bmi_gps_control_m%i", slot+1);  
								     
	if (IS_ERR(gps->class_dev)) {                                
		printk(KERN_ERR "Unable to create "                  
		       "class_device for bmi_gps_m%i; errno = %ld\n",
		       slot+1, PTR_ERR(gps->class_dev));             
		gps->class_dev = NULL;
		cdev_del (&gps->cdev);         
	}                                                            

	//bind driver and bmi_device 
	gps->bdev = bdev;
	bmi_device_set_drvdata(bdev, gps);

	printk(KERN_INFO "bmi_gps.c: probe slot %d\n", slot);

	// configure IOX - leave ouputs as inputs unless needed 
	
	gps->iox = i2c_new_device(bdev->slot->adap, &iox_info);
	if (gps->iox == NULL)
	  printk(KERN_ERR "IOX NULL...\n");
	if(WriteByte_IOX(gps->iox, IOX_OUTPUT_REG, 0x40) < 0) {  // 
	  device_destroy(bmi_class, MKDEV(major, slot));
	  goto err_lbl;
	}
	if(WriteByte_IOX(gps->iox, IOX_CONTROL, 0x3F) < 0) {     // IOX[4:3]=OUT, IOX[7:5,2:0]=IN
	  device_destroy(bmi_class, MKDEV(major, slot));	  
	  goto err_lbl;
	}


	// Initialize GPIOs (turn LED's on )

//	bmi_slot_gpio_configure_as_output (int slot, int gpio, int data)  

	bmi_slot_gpio_direction_out(slot, RED_LED, 0);
	bmi_slot_gpio_direction_out(slot, GREEN_LED, 0);
	bmi_slot_gpio_direction_out(slot, GPIO_1, 0); // Red, Green LEDS and GPIO_1 outputs

	//Enable uart transceiver
	bmi_slot_uart_enable (slot);

	mdelay(275);

	// release reset to J32 device
	bmi_slot_gpio_set_value(slot, RED_LED, 1);
	bmi_slot_gpio_set_value(slot,GREEN_LED, 1);
	bmi_slot_gpio_set_value(slot, GPIO_1, 1); 	// reset high, Red, Green LEDS off 

	return 0;

 err_lbl:
	gps->class_dev = NULL;
	cdev_del (&gps->cdev);
	bmi_device_set_drvdata (bdev, 0);
	i2c_unregister_device(gps->iox);
	return -ENODEV;
}


void bmi_gps_remove(struct bmi_device *bdev)
{	
	int slot;
	int i;
	struct bmi_gps *gps;
	struct class *bmi_class;

	slot = bdev->slot->slotnum;	
	gps = &bmi_gps[slot];

	dev_info(&bdev->dev, "gps removed..\n");
	//Disable uart transceiver
	bmi_slot_uart_disable (slot);
	i2c_unregister_device(gps->iox);
	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	bmi_class = bmi_get_class ();
	device_destroy(bmi_class, MKDEV(major, slot));

	gps->class_dev = NULL;

	cdev_del (&gps->cdev);

	//de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	gps->bdev = 0;

	return;
}





static void __exit bmi_gps_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver (&bmi_gps_driver);

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region(dev_id, 4);
	return;
}




static int __init bmi_gps_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers

	retval = alloc_chrdev_region(&dev_id, 0, 4, "BMI GPS Driver"); 
				     
	if (retval) {
		return -1;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_gps_driver);   

	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -1;  
	}
	printk("bmi_gps.c: BMI_GPS Driver v%s \n", BMIGPS_VERSION);

	return 0;
}


module_init(bmi_gps_init);
module_exit(bmi_gps_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter Giacomini <p.giacomini@encadis.com>");
MODULE_DESCRIPTION("BMI gps device driver");
MODULE_SUPPORTED_DEVICE("bmi_gps_control");

