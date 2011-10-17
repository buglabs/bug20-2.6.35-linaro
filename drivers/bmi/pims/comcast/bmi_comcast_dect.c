/*
 * drivers/bmi/pims/comcast/bmi_comcast_dect.c
 *
 * Copyright (C) 2011 Lane Brooks
 *
 * Contact: Lane Brooks <lane@brooksee.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#include <linux/bmi.h>
#include <linux/bmi/bmi-control.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/i2c/pca953x.h>
#include <linux/gpio.h>
#include <linux/bmi/bmi_comcast_dect.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>

#define SPI_SPEED 10000000
#define SPI_MODE  (SPI_MODE_0)

#define BMI_COMCAST_DECT_VERSION  "0.1.0.1"
static int major;		// control device major

// BMI device ID table
static struct bmi_device_id bmi_comcast_dect_tbl[] = 
{ 
	{ .match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
	  .vendor   = BMI_VENDOR_BUG_LABS, 
	  .product  = BMI_PRODUCT_COMCAST_DECT, 
	  .revision = BMI_ANY, 
	}, 
	{ 0, },				      /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_comcast_dect_tbl);

int	bmi_comcast_dect_probe(struct bmi_device *bdev);
void 	bmi_comcast_dect_remove(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_comcast_dect_driver = 
{
	.name = "bmi_comcast_dect", 
	.id_table = bmi_comcast_dect_tbl, 
	.probe   = bmi_comcast_dect_probe, 
	.remove  = bmi_comcast_dect_remove, 
	};


struct bmi_comcast_dect {
	struct bmi_device *bdev;		
	struct device	  *class_dev; // control class device
	struct cdev       cdev;			// control device
	int               open_flag;		// single open flag
	struct spi_board_info spi_board_info;
	struct spi_device *spi;	// SPI device
};

#define GPIO_DECT_RESETB        0
#define GPIO_IO_RESETB          1
#define GPIO_GREEN_LED          2
#define GPIO_RED_LED            3

// configure GPIO IO and states
static void init_gpios(struct bmi_comcast_dect *dect)
{
	// set states before turning on outputs
	int slot = dect->bdev->slot->slotnum;
	bmi_slot_gpio_direction_out(slot, GPIO_DECT_RESETB,    0);
	bmi_slot_gpio_direction_out(slot, GPIO_IO_RESETB,      0);
	bmi_slot_gpio_direction_out(slot, GPIO_GREEN_LED,      0);
	bmi_slot_gpio_direction_out(slot, GPIO_RED_LED,        1);

	// pull DECT out of reset
	bmi_slot_gpio_direction_out(slot, GPIO_DECT_RESETB,    1);
}

// deconfigure GPIOs
static void uninit_gpios(struct bmi_comcast_dect *dect)
{
 	int slot = dect->bdev->slot->slotnum;
	bmi_slot_gpio_direction_in(slot, GPIO_DECT_RESETB);
	bmi_slot_gpio_direction_in(slot, GPIO_IO_RESETB  );
	bmi_slot_gpio_direction_in(slot, GPIO_GREEN_LED  );
	bmi_slot_gpio_direction_in(slot, GPIO_RED_LED    );
}

static void comcast_dect_set_red_led(struct bmi_device *bdev, int on) 
{
	int slot = bdev->slot->slotnum;
	bmi_slot_gpio_direction_out(slot, GPIO_RED_LED, on ? 1 : 0);
}

static void comcast_dect_set_green_led(struct bmi_device *bdev, int on) 
{
	int slot = bdev->slot->slotnum;
	bmi_slot_gpio_direction_out(slot, GPIO_GREEN_LED, on ? 1 : 0);
}

static inline int spi_read_write(struct spi_device *spi, const u8 *wbuf, u8 *rbuf, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= wbuf,
			.rx_buf         = rbuf,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);
}

// spi registers
#define IODIRA   0x00
#define IPOLA    0x01
#define GPINTENA 0x02
#define DEFVALA  0x03
#define INTCONA  0x04
#define IOCON    0x05
#define GPPUA    0x06
#define INTFA    0x07
#define INTCAPA  0x08
#define GPIOA    0x09
#define OLATA    0x0A

static int bmi_dect_spi_set(struct spi_device *spi, int addr, int val) {
	int ret;
	unsigned char cmd[3], rcmd[3];
	cmd[0] = 0x40;
	cmd[1] = addr;
	cmd[2] = val;
	ret = spi_read_write(spi, cmd, rcmd, 3);
	printk(KERN_ERR "%s: addr=0x%x val=0x%x rval=0x%x\n", __func__, addr, val, rcmd[2]);
	return ret;
}

static int bmi_dect_spi_get(struct spi_device *spi, int addr, int *val) {
	int ret;
	unsigned char wcmd[3], rcmd[3];
	wcmd[0] = 0x41;
	wcmd[1] = addr;
	wcmd[2] = 0x00;
	ret = spi_read_write(spi, wcmd, rcmd, 3);
	if(ret < 0) {
		return ret;
	}
	*val = rcmd[2];
//	printk(KERN_ERR "%s: addr=0x%x val=0x%x\n", __func__, addr, *val);
	return ret;
}

static int bmi_dect_prog_seq(struct spi_device *spi, unsigned char *seq, int len) {
	int ret;
	unsigned char *cmd = kzalloc(len+2, GFP_KERNEL);
	cmd[0] = 0x40;
	cmd[1] = GPIOA;
	memcpy(cmd+2, seq, len);
	
	ret = spi_write(spi, cmd, len+2);
	kfree(cmd);
	return ret;
}

static int bmi_dect_program_init(struct bmi_comcast_dect *dect) {
	int ret  = 0;
	int slot = dect->bdev->slot->slotnum;

	// first reset the part
	bmi_slot_gpio_direction_out(slot, GPIO_IO_RESETB, 0);
	bmi_slot_gpio_direction_out(slot, GPIO_IO_RESETB, 1);

	// Set output latch values to all ones
	ret |= bmi_dect_spi_set(dect->spi, GPIOA,  0xFF);

        // I/O direction register, set RTCK and TDO as inputs, others as outputs
	ret |= bmi_dect_spi_set(dect->spi, IODIRA, 0x18); 

	// Set pull-ups on all outputs
	ret |= bmi_dect_spi_set(dect->spi, GPPUA,  ~0x18);

	// Configuration register, set non-open-drain driver & turn off addr inc
	ret |= bmi_dect_spi_set(dect->spi, IOCON,  0x20);

	return ret;
}
static int bmi_dect_program_uninit(struct bmi_comcast_dect *dect) {
	int slot = dect->bdev->slot->slotnum;
	bmi_slot_gpio_direction_out(slot, GPIO_IO_RESETB, 0);
	return 0;
}

/*
 *	control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *file)
{	
	struct bmi_comcast_dect *dect;

	dect = container_of (inode->i_cdev, struct bmi_comcast_dect, cdev);

	// Enforce single-open behavior
	if (dect->open_flag) {
		return -EBUSY; 
	}
	dect->open_flag = 1;

	// Save dect_dev pointer for later.
	file->private_data = dect;
	return 0;
}

// release
int cntl_release(struct inode *inode, struct file *file)
{	
	struct bmi_comcast_dect *dect;
	dect = (struct bmi_comcast_dect *)(file->private_data);
	dect->open_flag = 0;
	return 0;
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct bmi_comcast_dect *dect;
	int slot;
	dect = (struct bmi_comcast_dect *)(file->private_data);
	
	// error if dect not present
	if(dect->bdev == 0)
		return -ENODEV;
	
	slot = dect->bdev->slot->slotnum;

	// ioctl's
	switch (cmd) {

	case BMI_COMCAST_DECT_SUSPEND:
		dect->bdev->dev.bus->pm->suspend(&dect->bdev->dev);
		break;
	case BMI_COMCAST_DECT_RESUME:
		dect->bdev->dev.bus->pm->resume(&dect->bdev->dev);
		break;
	case BMI_COMCAST_DECT_RLEDOFF:
		comcast_dect_set_red_led(dect->bdev, 0);
		break;
	case BMI_COMCAST_DECT_RLEDON:
		comcast_dect_set_red_led(dect->bdev, 1);
		break;
	case BMI_COMCAST_DECT_GLEDOFF:
		comcast_dect_set_green_led(dect->bdev, 0);
		break;
	case BMI_COMCAST_DECT_GLEDON:
		comcast_dect_set_green_led(dect->bdev, 1);
		break;
	case BMI_COMCAST_DECT_PROGRAM_INIT:
		if(arg) {
			bmi_dect_program_init(dect);
		} else {
			bmi_dect_program_uninit(dect);
		}
		break;
	case BMI_COMCAST_DECT_PROGRAM_SEQ:
		return bmi_dect_prog_seq(dect->spi, 
					 ((struct dect_prog_seq*) arg)->seq, 
					 ((struct dect_prog_seq*) arg)->len);
		break;
	case BMI_COMCAST_DECT_PROGRAM_SET:
		return bmi_dect_spi_set(dect->spi, (arg >> 8) & 0xFF, arg & 0xFF);
	case BMI_COMCAST_DECT_PROGRAM_GET:
	{
		int val, ret;
		ret = bmi_dect_spi_get(dect->spi, (arg >> 8) & 0xFF, &val);
		if(ret < 0) {
			return ret;
		} else {
			return val;
		}
	}
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

int bmi_comcast_dect_probe(struct bmi_device *bdev)
{	
	struct cdev *cdev;
	struct bmi_comcast_dect *dect;
	struct class *bmi_class;
	dev_t dev_id;
	int ret=0;
	int slot = bdev->slot->slotnum;

	dect = kzalloc(sizeof(*dect), GFP_KERNEL);
	if (!dect)
	     return -1;
	
	bmi_device_set_drvdata(bdev, dect);
	dect->bdev = bdev;

	init_gpios(dect); 	// configure GPIO

	dect->spi_board_info.max_speed_hz = SPI_SPEED;
	dect->spi_board_info.bus_num = bdev->slot->spi_bus_num;
	dect->spi_board_info.chip_select = bdev->slot->spi_cs;
	dect->spi_board_info.mode = SPI_MODE;
	dect->spi = spi_new_device(spi_busnum_to_master(dect->spi_board_info.bus_num), &dect->spi_board_info) ;

	if (!dect->spi) {
		printk(KERN_WARNING "spi_new_device failed\n");
		goto err;
	}

	
	// Create 1 minor device
	cdev = &dect->cdev;
	cdev_init (cdev, &cntl_fops);
	dev_id = MKDEV(major, slot); 
	ret = cdev_add (cdev, dev_id, 1);
	if (ret) {
		goto err;
	}

	// Create class device 
	bmi_class = bmi_get_class ();                            
	dect->class_dev = device_create (bmi_class, NULL, 
					MKDEV (major, slot), NULL, 
					"bmi_dect%i", slot);  

	if (IS_ERR(dect->class_dev)) {                                
		printk (KERN_ERR "Unable to create "
		       "class_device for bmi_dect%i; errno = %ld\n",
		       slot, PTR_ERR(dect->class_dev));
		dect->class_dev = NULL;
		cdev_del (&dect->cdev);
		ret = -ENODEV;
		goto err;
	}



	return 0;

err:
	spi_unregister_device(dect->spi);
	kfree(dect);
	return ret;
}

void bmi_comcast_dect_remove(struct bmi_device *bdev)
{	
	struct class *bmi_class;
	struct bmi_comcast_dect *dect = bmi_device_get_drvdata (bdev);
	int slot = bdev->slot->slotnum;

	spi_unregister_device(dect->spi);
	uninit_gpios(dect);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	dect->class_dev = 0;

	cdev_del (&dect->cdev);

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	dect->bdev = 0;

	kfree (dect);
	return;
}

static __init int bmi_comcast_dect_init(void)
{	
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI Comcast DECT Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_comcast_dect_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}
	return 0;
}

static void __exit bmi_comcast_dect_cleanup(void)
{	
	dev_t dev_id;

//	UnRegister with BMI bus.

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}


module_init(bmi_comcast_dect_init);
module_exit(bmi_comcast_dect_cleanup);

MODULE_AUTHOR("Lane Brooks");
MODULE_DESCRIPTION("BMI Comcast Dect Driver");
MODULE_LICENSE("GPL");
