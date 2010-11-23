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

#include "md.h"
#include "mdacc.h"
#include "mon.h"
#include <linux/bmi/bmi_mdacc.h>
#include <linux/ioctl.h>
#include <linux/poll.h>

static int md_open (struct inode *, struct file *);
static int md_release (struct inode *, struct file *);
static int md_ioctl (struct inode *, struct file *, unsigned int, unsigned long);
static ssize_t md_read (struct file *, char __user *, size_t, loff_t *);
static unsigned int md_poll (struct file *, struct poll_table_struct *);

struct file_operations md_fops = {
	.owner = THIS_MODULE, 
	.ioctl = md_ioctl, 
	.open = md_open, 
	.release = md_release, 
	.read = md_read,
	.poll = md_poll,
};

static int md_major;

#define BMI_MOTION_DETECT_MASK   (BMI_MOTION_DETECT_STATUS         | \
				  BMI_MOTION_DETECT_LATCHED_STATUS | \
				  BMI_MOTION_DETECT_DELTA          | \
				  BMI_MOTION_DETECT_ENABLED)



void md_update (struct md *md, char data)
{

	unsigned char old_delta;
	unsigned char new_delta;
	unsigned char merge_bits;
	unsigned char new_bits;


	// Delta and Latched Status Bit Handling.

	//  MOTION_STATUS        Bit 3
	//  LATCHED_STATUS       Bit 2
	//  DELTA                Bit 1
	//  ENABLED              Bit 0


	// Handle bits individually

	old_delta = md->status & 0x02;
	new_delta = data & 0x02;

	if (!new_delta) {
		//preserve old latch bit, update delta bit
		merge_bits = (md->status & 0x06) | (data & 0x02);
	}
	else {
		//update latch bit and delta bit
		merge_bits = (md->status & 0x06) | (data & 0x06);
	}
	new_bits = data & 0x09;
	md->status = merge_bits | new_bits;

	if (!old_delta && new_delta) {
		md->ready = 1;
		//wake up anyone sleeping on our read wait queue
		wake_up_interruptible (&md->read_wait_queue);

	}
	return;
}


void md_clear_status (struct md *md)
{
	
	md->status &= ~(BMI_MOTION_DETECT_LATCHED_STATUS |
			BMI_MOTION_DETECT_DELTA);

	md->ready = 0;
	return;
	
}


int  md_init (void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers

	retval = alloc_chrdev_region(&dev_id, 0, 4, "BMI MDACC Motion Detector Driver"); 
				     
	if (retval) {
		return -1;
	}
	md_major = MAJOR(dev_id);
	return 0;
}

void md_clean (void)
{
	dev_t dev_id;

	dev_id = MKDEV(md_major, 0); 
	unregister_chrdev_region(dev_id, 4);
	return;
}

int  md_probe (struct md *md, int slot, struct mon *mon)
{
	struct cdev *cdev;
	dev_t dev_id;
	int ret;
	struct class *bmi_class;

	md->removed = 0;

	cdev = &md->cdev;
	cdev_init (cdev, &md_fops);

	dev_id = MKDEV (md_major, slot); 
	ret = cdev_add (cdev, dev_id, 1);

	//Create class device 
	bmi_class = bmi_get_class ();                            

	md->class_dev = device_create (bmi_class, NULL, MKDEV(md_major, slot), md, "bmi_mdacc_mot_m%i", slot+1);  
								     
	if (IS_ERR(md->class_dev)) {                                
		printk(KERN_ERR "Unable to create "                  
		       "class_device for bmi_mdacc_mot_m%i; errno = %ld\n",
		       slot+1, PTR_ERR(md->class_dev));             
		md->class_dev = NULL;                               
	}

	md->open_flag = 0;
	md->enabled = 0;
	md->status = 0;
	init_waitqueue_head (&md->read_wait_queue);
	md->mon = mon;
	return ret;
}

void md_remove (struct md *md, int slot )
{
	struct class *bmi_class;

	md->removed = 1;
	md->ready = -1;
	wake_up_interruptible (&md->read_wait_queue);

	cdev_del (&md->cdev);
	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(md_major, slot));
	md->class_dev = 0;
	return;
}


static int md_open (struct inode *inode, struct file *file)
{
	struct md *md;

	md = container_of(inode->i_cdev, struct md, cdev);
	
	//Enforce single open behavior
	if (md->open_flag) {
		return -EBUSY;
	}
	md->open_flag = 1;

	// Save md_drv pointer for later.
	file->private_data = md;
	return 0;
}

static int md_release (struct inode *inode, struct file *file)
{
	struct md *md;

	md = container_of(inode->i_cdev, struct md, cdev);
	md->open_flag = 0;

	//Enforce stop-on-close behavior.
	if (md->enabled) {
		mon_stop_motion (md->mon);
	}
	return 0;
}

static int md_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct md *md;
	md = container_of(inode->i_cdev, struct md, cdev);

	switch (cmd) {

	case BMI_MDACC_MOTION_DETECTOR_GET_STATUS:

		err = mon_status_request (md->mon);
		if (err){
			return -ENODEV;
		}

		//copy to user      
		err = copy_to_user((void*)(arg), &md->status, sizeof(md->status)); 
		md_clear_status (md);
		if (err) {
			return -EFAULT;
		}
		break;
	

	case BMI_MDACC_MOTION_DETECTOR_RUN:

		err = mon_start_motion (md->mon);

		if (err){
			return -ENODEV;
		}
		mon_status_request (md->mon);
		md->enabled = 1;
		break;
	
		
	case BMI_MDACC_MOTION_DETECTOR_STOP:

		err = mon_stop_motion (md->mon);
		if (err){
			return -ENODEV;
		}

		break;
		
	default:
		printk (KERN_ERR "md_ioctl() - error exit\n");
		return -ENOTTY;
	}
	return 0;
}

static ssize_t md_read (struct file *file, char __user *buf, size_t count, loff_t *ppos)
{

// 
// if fd is non-blocking and md is not enabled, return EWOULDBLOCK 
// if fd is non-blocking and md is enabled, and md is not ready, return EWOULDBLOCK 
// if fd is non-blocking and md is enabled, and md is ready, copy md->status to user, md_clear_status(),

// if fd is blocking and md is not enabled, avr_read_status,
//				            md_update,  copy md->status to user, md_clear_status()  

// if fs is blocking and md is enabled, and md is not ready, sleep until ready.
//                                                           when ready, copy md->status to user, md_clear_status()   


	int err;
	struct md *md = file->private_data;


	if (!md->enabled) {
		return -EAGAIN;
	}

	if (file->f_flags & O_NONBLOCK) {
		return -EAGAIN;
	}

	while (!md->ready) {
	      	
	  	if (file->f_flags & O_NONBLOCK)
	  		return -EAGAIN;
	  	if (wait_event_interruptible (md->read_wait_queue, (md->ready)))
	  		return -ERESTARTSYS;
		
		if(md->removed) {
		  return -1;
		}
	}

	err = copy_to_user (buf, &md->status, 1);
	md_clear_status (md);
	if (err) {
		return -EFAULT;    
	}
	return 1;
}

static unsigned int md_poll (struct file *file, struct poll_table_struct *table)
{
	unsigned int mask = 0;
	struct md *md = file->private_data; 

	poll_wait(file, &md->read_wait_queue, table);

	if (md->ready) {
		mask |= POLLIN | POLLRDNORM;	/* readable */
	}

	if (mdacc_check_bdev_md (md) ) {
		mask |= POLLHUP;	/* hang-up */
	}

	if (!md->enabled) {
		mask |= POLLHUP;	/* hang-up */
	}
	return mask;
}


