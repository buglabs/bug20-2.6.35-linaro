/*
 * 	bmi_gsm.c
 *
 * 	BMI GSM device driver
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

#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/system.h>
#include <linux/bmi.h>
#include <linux/bmi/bmi_gsm.h>

#define DEBUG
#undef DEBUG

#define BMIGSM_VERSION		"1.0"		// driver version


// private device structure
struct bmi_gsm
{
	struct bmi_device	*bdev;			// BMI device
	struct cdev		cdev;			// control device
	struct device	*class_dev;		// control class device
  int open_flag;	
};

static struct bmi_gsm bmi_gsm_priv[4];	// per slot device structure
static int major;		// control device major

/*
 * 	BMI set up
 */

	// BMI device ID table
static struct bmi_device_id bmi_gsm_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_GSM, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },					  /* terminate list */
};

MODULE_DEVICE_TABLE(bmi, bmi_gsm_tbl);


int	bmi_gsm_probe(struct bmi_device *bdev);
void	bmi_gsm_remove(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_gsm_driver = 
{
	.name = "bmi_gsm", 
	.id_table = bmi_gsm_tbl, 
	.probe   = bmi_gsm_probe, 
	.remove  = bmi_gsm_remove, 
};


/*
 * control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *file)
{	
	struct bmi_gsm *gsmod;

	gsmod = container_of (inode->i_cdev, struct bmi_gsm, cdev);

	// Enforce single-open behavior

	if (gsmod->open_flag) {
		return -EBUSY; 
	}
	gsmod->open_flag = 1;

	// Save gsm_dev pointer for later.

	file->private_data = gsmod;
	return 0;

}

// release
int cntl_release(struct inode *inode, struct file *file)
{	
	struct bmi_gsm *gsmod;

	gsmod = (struct bmi_gsm *)(file->private_data);
	gsmod->open_flag = 0;
	return 0;
}


// ioctl
int cntl_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, 
		   unsigned long arg)
{
  struct bmi_gsm *gsmod;
  int slot;


  gsmod = (struct bmi_gsm *)(filp->private_data);
  
  if(gsmod->bdev == 0)
    return -ENODEV;
  
  slot = gsmod->bdev->slot->slotnum;
  
  // ioctl's
  switch (cmd) {
  case BMI_GSM_RLEDOFF:
    bmi_slot_gpio_set_value(slot, RED_LED, 1); // Red LED=OFF
    break;
  case BMI_GSM_RLEDON:
    bmi_slot_gpio_set_value(slot, RED_LED, 0); // Red LED=ON
    break;
  case BMI_GSM_GLEDOFF:
    bmi_slot_gpio_set_value(slot, GREEN_LED, 1); // Green LED=OFF
    break;
  case BMI_GSM_GLEDON:
    bmi_slot_gpio_set_value(slot, GREEN_LED, 0); // Green LED=ON
    break;
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


// Probe
int bmi_gsm_probe(struct bmi_device *bdev)
{	
  int slot;
  int status;
  dev_t dev_id;
  struct cdev *cdev;
  struct class *bmi_class;
  struct bmi_gsm *gsmod;

  // add usb dependancy
  increment_usb_dep();


  slot = bdev->slot->slotnum;
  gsmod = &bmi_gsm_priv[slot];
  gsmod->bdev = 0;
  gsmod->open_flag = 0;

  cdev = &gsmod->cdev;
  cdev_init(cdev, &cntl_fops);
  dev_id = MKDEV(major, slot);
  status = cdev_add(cdev, dev_id, 1);
  if ( status)
    return status;

  bmi_class = bmi_get_class();
  gsmod->class_dev = device_create (bmi_class, NULL, MKDEV(major, slot), NULL, "bmi_gsm_ctl_m%i", slot + 1);

  if (IS_ERR(gsmod->class_dev))
    {
      printk(KERN_ERR "Unable to create class device for bmi_gsm_ctl_m%i...", slot + 1);
      gsmod->class_dev = NULL;
      cdev_del(&gsmod->cdev);
      return -ENODEV;
    }	
    
  // configure GPIO
  
  // set GPIO direction
  bmi_slot_gpio_direction_out(slot, RED_LED, 0);
  bmi_slot_gpio_direction_out(slot, GREEN_LED, 0);
  bmi_slot_gpio_direction_out(slot, GPIO_0, 1);
  
  // turn LED's on
  mdelay(500);
  // turn LED's off
  bmi_slot_gpio_set_value(slot, RED_LED, 1);
  bmi_slot_gpio_set_value(slot, GREEN_LED, 1);

  // Check if SIM is present...
  // gpio_reg = bmi_read_gpio_data_reg(slot);
  // turn mini-card on
  printk(KERN_INFO "Turning Sierra Wireless Card On...\n");
  bmi_slot_gpio_set_value(slot, GPIO_0, 0);

  // set up bdev/pbmi_gsm pointers
  gsmod->bdev = bdev;
  bmi_device_set_drvdata(bdev, &gsmod);
    
  return 0;
}	

// remove
void bmi_gsm_remove(struct bmi_device *bdev)
{	
  int slot;
  int i;
  struct bmi_gsm *gsmod;
  struct class *bmi_class;
  
  slot = bdev->slot->slotnum;

  gsmod = &bmi_gsm_priv[slot];
      
  for (i = 0; i < 4; i++)
    bmi_slot_gpio_direction_in(slot, i);  

  bmi_class = bmi_get_class ();
  device_destroy (bmi_class, MKDEV(major, slot));
  
  gsmod->class_dev = 0;
  
  cdev_del (&gsmod->cdev);
  
  // de-attach driver-specific struct from bmi_device structure 
  bmi_device_set_drvdata (bdev, 0);
  gsmod->bdev = 0;
  
  //de-attach driver-specific struct from bmi_device structure 
  bmi_device_set_drvdata (bdev, NULL);

  // remove usb dependency
  decrement_usb_dep();
  
  return;
}


static __init int bmi_gsm_init(void)
{	
    
  dev_t	dev_id;
  int	retval;
  
  // alloc char driver with 4 minor numbers
  retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI GSM Driver"); 
  if (retval) {
    return -ENODEV;
  }
  
  major = MAJOR(dev_id);
  retval = bmi_register_driver (&bmi_gsm_driver);   
  if (retval) {
    unregister_chrdev_region(dev_id, 4);
    return -ENODEV;  
  }
  
  printk("bmi_gsm.c: BMI_GSM Driver v%s \n", BMIGSM_VERSION);
  
  return 0;
}	

static void __exit bmi_gsm_clean(void)
{
  dev_t dev_id;
  
  bmi_unregister_driver (&bmi_gsm_driver);
  
  dev_id = MKDEV(major, 0);
  unregister_chrdev_region (dev_id, 4);
  return;
}

module_init(bmi_gsm_init);
module_exit(bmi_gsm_clean);

MODULE_AUTHOR("Matt Isaacs <izzy@buglabs.net>");
MODULE_DESCRIPTION("BMI gsm device driver");
MODULE_LICENSE("GPL");
