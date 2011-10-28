/*
 * drivers/bmi/pims/dlna_media/bmi_dlna_media.c
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
#include <linux/bmi/bmi_dlna_media.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/spi/flash.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>

#define BMI_DLNA_MEDIA_VERSION  "0.1.0.1"
static int major;		// control device major

// BMI device ID table
static struct bmi_device_id bmi_dlna_media_tbl[] = 
{ 
	{ .match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
	  .vendor   = BMI_VENDOR_BUG_LABS, 
	  .product  = BMI_PRODUCT_DLNA_MEDIA, 
	  .revision = BMI_ANY, 
	}, 
	{ 0, },				      /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_dlna_media_tbl);

int	bmi_dlna_media_probe(struct bmi_device *bdev);
void 	bmi_dlna_media_remove(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_dlna_media_driver = 
{
	.name = "bmi_dlna_media", 
	.id_table = bmi_dlna_media_tbl, 
	.probe   = bmi_dlna_media_probe, 
	.remove  = bmi_dlna_media_remove, 
	};


struct bmi_dlna_media {
	struct bmi_device *bdev;		
	struct i2c_client *iox;
	struct device	  *class_dev; // control class device
	struct cdev       cdev;			// control device
	int               open_flag;		// single open flag
	struct spi_board_info spi_board_info;
	struct spi_device *spi;	// SPI device
};

#define SPI_SPEED 25000000
#define SPI_MODE  (SPI_MODE_0)


/* setup IO expander */
#define GPIO_NUM (256-16)
#define GPIO_LED_RED   (GPIO_NUM + 5)
#define GPIO_LED_GREEN (GPIO_NUM + 6)

static int dlna_media_ioexp_gpio_setup(struct i2c_client *client,
					  unsigned gpio, unsigned ngpio,
					  void *context)
{
  return 0;
}

static int dlna_media_ioexp_gpio_teardown(struct i2c_client *client,
					     unsigned gpio, unsigned ngpio,
					     void *context)
{
  return 0;
}

static struct pca953x_platform_data dlna_media_ioexp_data = {
	.gpio_base  = GPIO_NUM,
	.setup	    = dlna_media_ioexp_gpio_setup,
	.teardown   = dlna_media_ioexp_gpio_teardown,
};

#define BMI_IOX_I2C_ADDRESS	0x26	// 7-bit address
static struct i2c_board_info dlna_media_i2c_boardinfo = {
	I2C_BOARD_INFO("pca9555", BMI_IOX_I2C_ADDRESS),
	//.irq = gpio_to_irq(63),
	.platform_data = &dlna_media_ioexp_data,
};


#define GPIO_SPI_SEL            0
#define GPIO_RESETB             1
#define GPIO_SPI_OE             2
#define GPIO_XC_STATUS2         3

static ssize_t store_gpio_value(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct bmi_dlna_media *med = dev_get_drvdata(dev);
	int value=0, addr=-1;
	int slot = med->bdev->slot->slotnum;
	sscanf(buf, "%d %d", &addr, &value);
	printk(KERN_ERR "%s addr=0x%x value=0x%x\n", __func__, addr, value);
	if(addr < 0) {
		printk(KERN_ERR "%s: addr is invalid %d\n", __func__, addr);
	} else if(addr < 4) {
		bmi_slot_gpio_direction_out(slot, addr, value);
	} else {
		int gaddr = addr-4+GPIO_NUM;
		if(value == 2) {
			gpio_direction_input(gaddr);
		} else {
			gpio_direction_output(gaddr,   value);
		}
	}
	return size;
}

static DEVICE_ATTR(gpio_value, S_IWUGO , NULL, store_gpio_value);

static void dlna_media_set_bootstraps(unsigned int mode)
{
	int i, bit;
	// set bootstrap pins
	for(i=0; i<16; i++) {
		bit = (mode & 0x1) ? 1 : 0;
		mode = mode >> 1;
		if(i>4 && i<8) continue; // skip gpio's not controller boot mode
		gpio_direction_output(GPIO_NUM + i, bit);
		//printk(KERN_ERR "%s: %d: gpio_direction_output(%d, %d)\n", __func__, i, GPIO_NUM + i, bit);
	}
}

// configure GPIO IO and states
static void init_gpios(struct bmi_dlna_media *med)
{
	// set states before turning on outputs
	int slot = med->bdev->slot->slotnum;
	bmi_slot_gpio_direction_out(slot, GPIO_SPI_SEL,  0);
	bmi_slot_gpio_direction_out(slot, GPIO_RESETB,   0);//hold vixs in reset
	bmi_slot_gpio_direction_in (slot, GPIO_SPI_OE);// SER_EN=OFF
	bmi_slot_gpio_direction_in (slot, GPIO_XC_STATUS2);

	/* initialize LEDS */
	gpio_direction_output(GPIO_LED_RED,   1);
	gpio_direction_output(GPIO_LED_GREEN, 0);

	dlna_media_set_bootstraps(BMI_DLNA_MEDIA_BOOT_SPI);
}

// deconfigure GPIOs
static void uninit_gpios(struct bmi_dlna_media *med)
{
 	int slot = med->bdev->slot->slotnum;
	bmi_slot_gpio_direction_in(slot, GPIO_SPI_SEL);
	bmi_slot_gpio_direction_in(slot, GPIO_RESETB);
	//bmi_slot_gpio_direction_in(slot, GPIO_SPI_OE);
}

static struct mtd_partition vixs_prom_partitions[] = {
	{ .name       = "bootloader(spi)",
	  .size       = 0x1000000,
	  .offset     = 0x0,
	  .mask_flags = 0x0,
	},
};

static struct flash_platform_data vixs_prom = {
	.name = "m25p80",
	.parts = vixs_prom_partitions,
	.nr_parts = 1,
	.type = "m25p128",
};

static int dlna_media_set_red_led(struct bmi_device *bdev, int on) 
{
	return gpio_direction_output(GPIO_LED_RED, on ? 1 : 0);
}

static int dlna_media_set_green_led(struct bmi_device *bdev, int on) 
{
	return gpio_direction_output(GPIO_LED_GREEN, on ? 1 : 0);
}

static int dlna_media_vixs_reboot(struct bmi_device *bdev, unsigned int mode)
{
	int slot = bdev->slot->slotnum;

	// put vixs chip in reset
	bmi_slot_gpio_direction_out(slot, GPIO_RESETB, 0);

	// if SPI flash boot mode, then give VIXS chip control of SPI flash
	if(mode == BMI_DLNA_MEDIA_BOOT_SPI) {
		bmi_slot_gpio_direction_out(slot, GPIO_SPI_SEL,  0);
	}

	dlna_media_set_bootstraps(mode);
	
	// wait 200ms
	mdelay(10);

	//printk(KERN_ERR "%s: xc_status = 0x%x\n", __func__, bmi_slot_gpio_get_value(slot, GPIO_XC_STATUS2));

	// remove vixs chip from reset
	bmi_slot_gpio_direction_out(slot, GPIO_RESETB, 1);

	// TODO: set bootstrap pins as inputs after boot is complete

	return 0;
}

/*
 *	control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *file)
{	
	struct bmi_dlna_media *med;

	med = container_of (inode->i_cdev, struct bmi_dlna_media, cdev);

	// Enforce single-open behavior
	if (med->open_flag) {
		return -EBUSY; 
	}
	med->open_flag = 1;

	// Save med_dev pointer for later.
	file->private_data = med;
	return 0;
}

// release
int cntl_release(struct inode *inode, struct file *file)
{	
	struct bmi_dlna_media *med;
	med = (struct bmi_dlna_media *)(file->private_data);
	med->open_flag = 0;
	return 0;
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct bmi_dlna_media *med;
	int slot;
	med = (struct bmi_dlna_media *)(file->private_data);
	
	// error if med not present
	if(med->bdev == 0)
		return -ENODEV;
	
	slot = med->bdev->slot->slotnum;

	// ioctl's
	switch (cmd) {

	case BMI_DLNA_MEDIA_SUSPEND:
		med->bdev->dev.bus->pm->suspend(&med->bdev->dev);
		break;
	case BMI_DLNA_MEDIA_RESUME:
		med->bdev->dev.bus->pm->resume(&med->bdev->dev);
		break;
	case BMI_DLNA_MEDIA_RLEDOFF:
		dlna_media_set_red_led(med->bdev, 0);
		break;
	case BMI_DLNA_MEDIA_RLEDON:
		dlna_media_set_red_led(med->bdev, 1);
		break;
	case BMI_DLNA_MEDIA_GLEDOFF:
		dlna_media_set_green_led(med->bdev, 0);
		break;
	case BMI_DLNA_MEDIA_GLEDON:
		dlna_media_set_green_led(med->bdev, 1);
		break;
	case BMI_DLNA_MEDIA_VIXS_REBOOT:
		dlna_media_vixs_reboot(med->bdev, (unsigned int) arg);
		break;
	case BMI_DLNA_MEDIA_SPI_FLASH_SEL:
		bmi_slot_gpio_direction_out(slot, GPIO_SPI_SEL,  arg ? 1 : 0);
		break;
	case BMI_DLNA_MEDIA_VIXS_RESET:
		bmi_slot_gpio_direction_out(slot, GPIO_RESETB, arg);
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

int bmi_dlna_media_probe(struct bmi_device *bdev)
{	
	struct cdev *cdev;
	struct bmi_dlna_media *med;
	struct class *bmi_class;
	dev_t dev_id;
	int ret=0, i;
	int slot = bdev->slot->slotnum;

	med = kzalloc(sizeof(*med), GFP_KERNEL);
	if (!med)
	     return -1;
	
	bmi_device_set_drvdata(bdev, med);
	med->bdev = bdev;

	med->iox  = i2c_new_device(bdev->slot->adap, &dlna_media_i2c_boardinfo);
	if(!med->iox) {
		printk(KERN_ERR "Error creating IO Expander device\n");
		return -1;
	}

	ret=0;
	for(i=GPIO_NUM; i<16+GPIO_NUM; i++) {
		ret |= gpio_request(i, "dlna_media");
		gpio_direction_input(i);
	}
	if(ret) {
		printk(KERN_ERR "Error requesting IOX gpio's\n");
		goto err;
	}
	init_gpios(med); 	// configure GPIO
	
	// These can be removed after driver stabalizes. They are for debug now.
	ret = device_create_file(&bdev->dev, &dev_attr_gpio_value);
	if(ret) {
		goto err;
	}

	// Create 1 minor device
	cdev = &med->cdev;
	cdev_init (cdev, &cntl_fops);
	dev_id = MKDEV(major, slot); 
	ret = cdev_add (cdev, dev_id, 1);
	if (ret) {
		goto err;
	}

	// Create class device 
	bmi_class = bmi_get_class ();                            
	med->class_dev = device_create (bmi_class, NULL, 
					MKDEV (major, slot), NULL, 
					"bmi_media%i", slot);  

	if (IS_ERR(med->class_dev)) {                                
		printk (KERN_ERR "Unable to create "
		       "class_device for bmi_media%i; errno = %ld\n",
		       slot, PTR_ERR(med->class_dev));
		med->class_dev = NULL;
		cdev_del (&med->cdev);
		return -ENODEV;
	}

	// SPI
	strcpy(med->spi_board_info.modalias, "m25p80");
	med->spi_board_info.max_speed_hz = SPI_SPEED;
	med->spi_board_info.bus_num = bdev->slot->spi_bus_num;
	med->spi_board_info.chip_select = bdev->slot->spi_cs;
	med->spi_board_info.platform_data = &vixs_prom;
	med->spi_board_info.mode = SPI_MODE;

	med->spi = spi_new_device(spi_busnum_to_master(med->spi_board_info.bus_num), &med->spi_board_info) ;
	if (!med->spi) {
		printk(KERN_WARNING "VH: spi_new_device failed\n");
		goto err;
	}
//	struct m25p *fdata = dev_get_drvdata(&spi->dev);
//	printk(KERN_ERR "%s: %d\n", __func__, fdata->mtd->index);
//	struct mtd_info *mtd = get_mtd_device(NULL, &fdata->mtd);
//	printk(KERN_ERR "%s: %s\n", __func__, mtd->name);
	

	dlna_media_vixs_reboot(bdev, BMI_DLNA_MEDIA_BOOT_SPI);
	return 0;

err:
	for(i=GPIO_NUM; i<GPIO_NUM+16; i++) {
		gpio_free(i);
	}
	i2c_unregister_device(med->iox);
	spi_unregister_device(med->spi);
	kfree(med);
	return ret;
}

void bmi_dlna_media_remove(struct bmi_device *bdev)
{	
	int i;
	struct class *bmi_class;
	struct bmi_dlna_media *med = bmi_device_get_drvdata (bdev);
	int slot = bdev->slot->slotnum;

	uninit_gpios(med);
	for(i=GPIO_NUM; i<GPIO_NUM+16; i++) {
		gpio_free(i);
	}
	i2c_unregister_device(med->iox);
	spi_unregister_device(med->spi);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	med->class_dev = 0;

	cdev_del (&med->cdev);

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	med->bdev = 0;

	kfree (med);
	printk(KERN_ERR "%s: exit\n", __func__);
	return;
}

static __init int bmi_dlna_media_init(void)
{	
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI Dlna Media Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_dlna_media_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}
	return 0;
}

static void __exit bmi_dlna_media_cleanup(void)
{	
	dev_t dev_id;

//	UnRegister with BMI bus.

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}


module_init(bmi_dlna_media_init);
module_exit(bmi_dlna_media_cleanup);

MODULE_AUTHOR("Lane Brooks");
MODULE_DESCRIPTION("BMI DLNA Media Driver");
MODULE_LICENSE("GPL");
