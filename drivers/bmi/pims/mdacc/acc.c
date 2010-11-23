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

#include "acc.h"
#include "mdacc.h"
#include "mon.h"
#include <linux/bmi/bmi_mdacc.h>
#include <linux/ioctl.h>
#include <linux/poll.h>

static int acc_open (struct inode *, struct file *);
static int acc_release (struct inode *, struct file *);
static int acc_ioctl (struct inode *, struct file *, unsigned int, unsigned long);
static ssize_t acc_read (struct file *, char __user *, size_t, loff_t *);
static unsigned int acc_poll (struct file *, struct poll_table_struct *);

struct file_operations acc_fops = {
	.owner = THIS_MODULE, 
	.ioctl = acc_ioctl, 
	.open = acc_open, 
	.release = acc_release, 
	.read = acc_read,
	.poll = acc_poll,
};

static int acc_major;

int acc_init (void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers

	retval = alloc_chrdev_region(&dev_id, 0, 4, "BMI MDACC Accelerometer Driver"); 
				     
	if (retval) {
		return -1;
	}
	acc_major = MAJOR(dev_id);
	return 0;
}

void acc_clean(void)
{
	dev_t dev_id;

	dev_id = MKDEV(acc_major, 0); 
	unregister_chrdev_region(dev_id, 4);
	return;
}

int  acc_probe (struct acc *acc, int slot, struct mon *mon) 
{
	struct cdev *cdev;
	dev_t dev_id;
	int ret;
	struct class *bmi_class;

	// initialize cdev

	cdev = &acc->cdev;
	cdev_init (cdev, &acc_fops);

	dev_id = MKDEV (acc_major, slot); 
	ret = cdev_add (cdev, dev_id, 1);

	//Create class device 

	bmi_class = bmi_get_class ();                            

	acc->class_dev = device_create (bmi_class, NULL, MKDEV(acc_major, slot), acc, "bmi_mdacc_acc_m%i", slot+1);  
								     
	if (IS_ERR(acc->class_dev)) {                                
		printk(KERN_ERR "Unable to create "                  
		       "class_device for bmi_mdacc_acc_m%i; errno = %ld\n",
		       slot+1, PTR_ERR(acc->class_dev));             
		acc->class_dev = NULL;                               
	}

	acc->open_flag = 0;
	acc->mon = mon;


	// initialize mdacc_accel_config

	acc->cfg.read_queue_size = 1;
	acc->cfg.read_queue_threshold = 1;
	acc->cfg.delay_mode = 0;
	acc->cfg.delay = 4000;
	acc->cfg.delay_resolution = 1;
	acc->cfg.sensitivity = 0; 
	acc->cfg.run = 0;
	

	// initialize cque

	acc->cque = cque_create  (acc->cfg.read_queue_size, acc->cfg.read_queue_threshold); 

	// initialize read_wait_queue

	init_waitqueue_head (&acc->read_wait_queue);
	return ret;
}

void acc_remove (struct acc *acc, int slot) 
{
	struct class *bmi_class;

	cque_destroy (acc->cque);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(acc_major, slot));

	acc->class_dev = 0;

	cdev_del (&acc->cdev);


	return;
}


static int acc_open (struct inode *inode, struct file *file)
{
	struct acc *acc;

	acc = container_of(inode->i_cdev, struct acc, cdev);
	
	//Enforce single open behavior
	if (acc->open_flag) {
		return -EBUSY;
	}
	acc->open_flag = 1;

	// Save acc_drv pointer for later.
	file->private_data = acc;
	return 0;
}

static int acc_release (struct inode *inode, struct file *file)
{
	struct acc *acc;

	acc = container_of(inode->i_cdev, struct acc, cdev);
	acc->open_flag = 0;

	//Enforce stop-on-close behavior.
	if (acc->cfg.run) {
		acc->cfg.run = 0;
		mon_stop_accel (acc->mon);
	}
	return 0;

}

static int check_config (struct mdacc_accel_config *config)
{
	int err = 0;

	if (!config) {
		err = 1;
		goto exit;
	}

	if (config->read_queue_size < 1) {
		err = 1;
	}
	if (config->read_queue_threshold > config->read_queue_size) {
		err = 1;
	}
	if (config->delay_mode) {
		switch (config->delay_resolution) 
		{
			case 0:
				err = 1;
				break;
			case 1:				     // 1 => 1 usec
				if (config->delay < 5000) {
					err = 1;
				}
				break;
			case 2:
				if (config->delay < 625) {  // 2 => 8 usec 
					err = 1;
				}
				break;
			case 3:
				if (config->delay < 79) {  // 3 => 64 usec  
					err = 1;
				}
				break;
			case 4:
				if (config->delay < 20) { // 4 => 256 usec
					err = 1;
				}
				break;
			case 5:
				if (config->delay < 5) { // 5 => 1024 usec
					err = 1;
				}
				break;
			default:
				err = 1;
				break;
		}
	}

	if (config->sensitivity > 3) {
		err = 1;
	}

exit:
	return err;
}

static int acc_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct acc *acc;
	acc = container_of(inode->i_cdev, struct acc, cdev);

	switch (cmd) {

	case BMI_MDACC_ACCELEROMETER_SET_CONFIG:
	{
		struct mdacc_accel_config new_cfg;

		err = copy_from_user ( (void*)&new_cfg, (void*)arg, sizeof (struct mdacc_accel_config) );
		if (err) {
			return -EFAULT;
		}

		err = check_config (&new_cfg);
		if (err) {
			return -EINVAL;
		}

		if (acc->cfg.run) {
			mon_stop_accel (acc->mon);
		}

		// take the mon semaphore
		down_interruptible (&acc->mon->sem);

		memcpy ( &acc->cfg, &new_cfg, sizeof (struct mdacc_accel_config) );
		cque_destroy (acc->cque);
		acc->cque = cque_create  (acc->cfg.read_queue_size, acc->cfg.read_queue_threshold); 

		// release the mon semaphore
		up (&acc->mon->sem);

		mon_set_config_accel( acc->mon, &acc->cfg);

	}
		break;

	case BMI_MDACC_ACCELEROMETER_GET_CONFIG:

		mon_get_config_accel( acc->mon, &acc->cfg);


		err = copy_to_user ( (void*)arg, &acc->cfg, sizeof (struct mdacc_accel_config) );
		if (err) {
			return -EFAULT;
		}
		break;

	case BMI_MDACC_ACCELEROMETER_RUN:

		acc->cfg.run = 1;
		err = mon_start_accel (acc->mon);
		if (err){
			return -ENODEV;
		}
		break;

		
	case BMI_MDACC_ACCELEROMETER_STOP:

		acc->cfg.run = 0;
		err = mon_stop_accel (acc->mon);
		if (err){
			return -ENODEV;
		}
		break;

	default:
		printk (KERN_ERR "acc_ioctl() - error exit\n");
		return -ENOTTY;
	}

	return 0;
}

static ssize_t acc_read (struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
// if fd is non-blocking and acc is not enabled, return EAGAIN
// if fd is non-blocking and acc is enabled, and cque is not ready, return EAGAIN
// if fd is non-blocking and acc is enabled, and cque is ready, read cque, copy to user,

// if fd is blocking and acc is not enabled, , return EAGAIN
//				            

// if fs is blocking and acc is enabled, and cque is not ready, sleep until ready.
//                                                           when ready, read cque, copy to user, 



	struct acc *acc = file->private_data;
	unsigned char temp[6];
	int bytes_read;

	if (count < 6) {
		return -EINVAL;
	}

	if (!acc->cfg.run) {
		return -EAGAIN;
	}

	while (!cque_is_ready_for_read(acc->cque)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible (acc->read_wait_queue, (cque_is_ready_for_read(acc->cque))))
			return -ERESTARTSYS;
	}

	//loop through 1 sample at a time.

	bytes_read = 0;

	while (count >= 6) {
	
		if (cque_read (acc->cque, &temp))
			break;
		if (copy_to_user (buf, &temp, 6))
			break;
		bytes_read += 6;
		buf += 6;
		count -= 6;
	}
	return bytes_read;
}


static unsigned int acc_poll (struct file *file, struct poll_table_struct *table)
{
	unsigned int mask = 0;
	struct acc *acc = file->private_data; 

	poll_wait(file, &acc->read_wait_queue, table);

	if (cque_is_ready_for_read( acc->cque) ) {
		mask |= POLLIN | POLLRDNORM;	/* readable */
	}
	if (mdacc_check_bdev_acc (acc) ) {
		mask |= POLLHUP;	/* hang-up */
	}
	if (!acc->cfg.run) {
		mask |= POLLHUP;	/* hang-up */
	}
	return mask;
}

