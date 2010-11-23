#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/bmi/bmi_camera.h>
#include "bug_camera.h"

#define BUG_CAMERA_VERSION  "1.0"


struct cam_ctl
{
  int slot;
  struct cdev cdev;
  struct device *class_dev;
};

// private device structure
struct bug_cam
{
	unsigned int	cam_cnt;		//number of cameras present
	unsigned int	active;			//active camera slot

	struct bmi_cam  *bcam[4];		//slot-specific behavior
							
	struct input_dev *input_dev;		// input device
  struct cam_ctl	cntl[4];		// control character device
	struct device *class_dev;
	int		cntl_devno;		// control device number
	int		open_flag;		// force single open
};

static struct bug_cam bug_cam;

static int cntl_open (struct inode *, struct file *);
static int cntl_release (struct inode *, struct file *);
static int cntl_ioctl(struct inode *, struct file *, unsigned int, unsigned long);
// control file operations

struct file_operations cam_ctl_fops = {
	.owner = THIS_MODULE, 
	.ioctl = cntl_ioctl, 
	.open = cntl_open, 
	.release = cntl_release, 
};

/*
 * control device operations
 */

static int cam_ctl_major;

int ctl_init (void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers

	retval = alloc_chrdev_region(&dev_id, 0, 4, "BUG Camera Control"); 
				     
	if (retval) {
		return -1;
	}
	cam_ctl_major = MAJOR(dev_id);
	return 0;
}

void ctl_clean (void)
{
	dev_t dev_id;

	dev_id = MKDEV(cam_ctl_major, 0); 
	unregister_chrdev_region(dev_id, 4);
	return;
}

int ctl_probe (struct cam_ctl *cam_ctl, int slot) 
{
	struct cdev *cdev;
	dev_t dev_id;
	int ret;
	struct class *bmi_class;

	cdev = &cam_ctl->cdev;
	cdev_init (cdev, &cam_ctl_fops);

	dev_id = MKDEV (cam_ctl_major, slot); 
	ret = cdev_add (cdev, dev_id, 1);

	//Create class device 
	bmi_class = bmi_get_bmi_class ();                            

	cam_ctl->class_dev = device_create (bmi_class, NULL, MKDEV(cam_ctl_major, slot), cam_ctl, "bmi_cam_ctl_m%i", slot+1);  
								     
	if (IS_ERR(cam_ctl->class_dev)) {                                
		printk(KERN_ERR "Unable to create "                  
		       "class_device for bmi_cam_ctl_m%i; errno = %ld\n",
		       slot+1, PTR_ERR(cam_ctl->class_dev));             
		cam_ctl->class_dev = NULL;                               
	}
	cam_ctl->slot = slot;

	return ret;
}

void ctl_remove (struct cam_ctl *cam_ctl, int slot) 
{
	struct class *bmi_class;

	bmi_class = bmi_get_bmi_class ();
	device_destroy (bmi_class, MKDEV(cam_ctl_major, slot));

	cam_ctl->class_dev = 0;

	cdev_del (&cam_ctl->cdev);
	return;
}

// open
static int cntl_open(struct inode *inode, struct file *filp)
{	
	if (bug_cam.open_flag) {
		return - EBUSY;
	}
	bug_cam.open_flag = 1;
	filp->private_data = &bug_cam;
	return 0;
}

// release
static int cntl_release(struct inode *inode, struct file *filp)
{	
	bug_cam.open_flag = 0;
	return 0;
}




static struct bmi_cam* get_selected (void)
{
	struct bmi_cam *bcam;

	if (bug_cam.active == -1) {
		return 0;
	}

	bcam = 	bug_cam.bcam [bug_cam.active];
	return bcam;

}


static int select_slot (int slot)
{
	struct bmi_cam *bcam;

	// validate slot number
	if ((slot < 0) || slot > 3) {

		printk(KERN_ERR
			"bug_camera.c: Invalid value (%d) for camera selection.\n", 
			 slot);

		return -EINVAL;
	}

	// error if this slot not registered
	if ( bug_cam.bcam [slot] == NULL ) {
		return -ENODEV;
	}

	// abort if this slot already active
	if (bug_cam.active == slot) {
		return 0;
	}

	// if another slot is active, deactivate it.
	if (bug_cam.active != -1) {
		bcam = 	bug_cam.bcam [bug_cam.active];
		if ((bcam) && (bcam->deactivate)) {
			bcam->deactivate (bcam);
		}
	}

	// activate this slot
	bcam = 	bug_cam.bcam [slot];
	if (bcam->activate) { 
		bcam->activate (bcam, bug_cam.input_dev);
	}
	bug_cam.active = slot;
	return 0;
}


// ioctl
static int cntl_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, 
	       unsigned long arg)
{	
	struct bmi_cam *bcam;
	int err;

	switch (cmd) {

	  // error if no camera selected. (active)

	case BMI_CAM_FLASH_LED_ON:

		bcam = 	get_selected();
		if ((bcam) && (bcam->cntl.flash_led_on)) {
			bcam->cntl.flash_led_on (bcam);
		}
		return 0;

	case BMI_CAM_FLASH_LED_OFF:

		bcam = 	get_selected();
		if ((bcam) && (bcam->cntl.flash_led_off)) {
			bcam->cntl.flash_led_off (bcam);
		}
		return 0;


	case BMI_CAM_FLASH_HIGH_BEAM:

		bcam = 	get_selected();
		if ((bcam) && (bcam->cntl.flash_high_beam)) {
			bcam->cntl.flash_high_beam (bcam);
		}
		return 0;

	case BMI_CAM_FLASH_LOW_BEAM:

		bcam = 	get_selected();
		if ((bcam) && (bcam->cntl.flash_low_beam)) {
			bcam->cntl.flash_low_beam (bcam);
		}
		return 0;

	case BMI_CAM_RED_LED_OFF:

		bcam = 	get_selected();
		if ((bcam) && (bcam->cntl.red_led_off)) {
			bcam->cntl.red_led_off (bcam);
		}
		return 0;

	case BMI_CAM_RED_LED_ON:

		bcam = 	get_selected();
		if ((bcam) && (bcam->cntl.red_led_on)) {
			bcam->cntl.red_led_on (bcam);
		}
		return 0;

	case BMI_CAM_GREEN_LED_OFF:

		bcam = 	get_selected();
		if ((bcam) && (bcam->cntl.green_led_off)) {
			bcam->cntl.green_led_off (bcam);
		}
		return 0;

	case BMI_CAM_GREEN_LED_ON:          

		bcam = 	get_selected();
		if ((bcam) && (bcam->cntl.green_led_on)) {
			bcam->cntl.green_led_on (bcam);
		}
		return 0;

	  case BMI_CAM_SELECT:

		err = select_slot (arg);
		return err;

	  case BMI_CAM_GET_SELECTED:

		return (int) bug_cam.active;

	  default:
		return -ENOTTY;
	}
	return 0;
}


void bug_camera_set_color(int bright, int saturation, int red, int green, int blue) 
{
	//delegate to active bmi_cam.

	struct bmi_cam *bcam;

	if (bug_cam.active == -1) {
		return;
	}

	bcam = 	bug_cam.bcam [bug_cam.active];

	if ((bcam) && (bcam->sensor.set_color)) {
		bcam->sensor.set_color (bcam, bright, saturation, red, green, blue);
	}
	return;

}

void bug_camera_get_color(int *bright, int *saturation, int *red, int *green, int *blue)
{
	//delegate to active bmi_cam.

	struct bmi_cam *bcam;

	if (bug_cam.active == -1) {
		return;
	}
	
	bcam = 	bug_cam.bcam [bug_cam.active];

	if ((bcam) && (bcam->sensor.get_color)) {
		bcam->sensor.get_color (bcam, bright, saturation, red, green, blue);
	}
	return;
}




void bug_camera_set_ae_mode (int ae_mode)
{
	//delegate to active bmi_cam.
	printk (KERN_ERR " bug_camera_set_ae_mode() - NOT IMPLEMENTED.\n");
}


void bug_camera_get_ae_mode (int *ae_mode)
{
	//delegate to active bmi_cam.
	printk (KERN_ERR " bug_camera_set_ae_mode() - NOT IMPLEMENTED.\n");
}


sensor_interface * bug_camera_config (int *frame_rate, int high_quality)
{
	//delegate to active bmi_cam.

	struct bmi_cam *bcam;
	sensor_interface *sensor_if = 0;

	if (bug_cam.active == -1) {
		return 0;
	}
	
	bcam = 	bug_cam.bcam [bug_cam.active];

	if ((bcam) && (bcam->sensor.config)) {
		sensor_if = bcam->sensor.config (bcam, frame_rate, high_quality);
	}
	return sensor_if;
}


sensor_interface * bug_camera_reset (void)
{

	//delegate to active bmi_cam.

	struct bmi_cam *bcam;
	sensor_interface *sensor_if = 0;

	if (bug_cam.active == -1) {
		return 0;
	}
	
	bcam = 	bug_cam.bcam [bug_cam.active];

	if ((bcam) && (bcam->sensor.reset)) {
		sensor_if = bcam->sensor.reset(bcam);
	}
	return sensor_if;
}

void bug_camera_link_enable (void)
{
	//delegate to active bmi_cam.

	struct bmi_cam *bcam;

	if (bug_cam.active == -1) {
		return;
	}
	
	bcam = 	bug_cam.bcam [bug_cam.active];

	if ((bcam) && (bcam->link.enable)) {
		bcam->link.enable(bcam);
	}
	return;
}

void bug_camera_link_disable (void)
{
	//delegate to active bmi_cam.

	struct bmi_cam *bcam;

	if (bug_cam.active == -1) {
		return;
	}
	
	bcam = 	bug_cam.bcam [bug_cam.active];

	if ((bcam) && (bcam->link.disable)) {
		bcam->link.disable(bcam);
	}
	return;
}


// Link point for bug_v4l2_capture 

struct camera_sensor camera_sensor_if = {

        set_color:  bug_camera_set_color,
        get_color:  bug_camera_get_color,
        config:     bug_camera_config,
        reset:      bug_camera_reset,
};


struct camera_link camera_link_if = {
	enable:     bug_camera_link_enable,
	disable:    bug_camera_link_disable,

};

int register_bug_camera( struct bmi_cam *bcam, int slot)
{
	printk (KERN_ERR "register_bug_camera() - slot = %d\n", slot);

	if (!bcam) {
		return -1;
	}
	if ((slot < 0) || (slot > 3)) {
		return -1;
	}
	if ( bug_cam.bcam [slot]) {
		return -1;
	}
	else {
		bug_cam.bcam [slot] = bcam;
		bug_cam.cam_cnt += 1;
	}

	if (ctl_probe(&bug_cam.cntl[slot], slot)) {
	    printk(KERN_ERR "\n");
	  }
	// Activate this camera if no other is active
	if ( bug_cam.active == -1) {
		bug_cam.active = slot;
		bcam->activate( bcam, bug_cam.input_dev);
	}
	
	return 0;

}

int unregister_bug_camera( struct bmi_cam *bcam, int slot)
{
	
	if (!bcam) {
		return -1;
	}
	if ((slot < 0) || (slot > 3)) {
		return -1;
	}
	if ( bug_cam.bcam [slot]  != bcam) {
		return -1;
	}
	else {
		bug_cam.bcam [slot] = 0;
		bug_cam.cam_cnt -= 1;
		
		// Deactivate this camera if active

		if (bug_cam.active == slot) {
			bcam->deactivate( bcam);
			bug_cam.active = -1;
		}

	}	
	return 0;
}

static __init int bug_camera_init(void)
{
	int err = 0;
	struct class *bmi_class;

	printk("BUG Camera Driver v%s \n", BUG_CAMERA_VERSION);

	memset (&bug_cam, 0, sizeof(struct bug_cam));

	//No camera is active.
	bug_cam.active = -1;

	//No cameras registered.
	bug_cam.cam_cnt = 0;

	// Allocate and Register input device.

	// allocate input device
	bug_cam.input_dev = input_allocate_device();
	if (!bug_cam.input_dev) {
		printk(KERN_ERR "bug_camera_init: Can't allocate input_dev\n"); 
		err = -ENOMEM;
		goto exit;
	}

	// set up input device
	bug_cam.input_dev->name = "bug_cam";
	bug_cam.input_dev->phys = "bug_cam";
	bug_cam.input_dev->id.bustype = BUS_BMI;
	//bug_cam.input_dev->private = &bug_cam;
	bug_cam.input_dev->evbit[0] = BIT(EV_KEY);
	bug_cam.input_dev->keybit[BIT_WORD(BN_SHUTTER)] = BIT_MASK(BN_SHUTTER)  |
					      BIT_MASK(BN_ZOOMIN)  |
					      BIT_MASK(BN_ZOOMOUT);
	// register input device 
	err = input_register_device (bug_cam.input_dev);
	if (err) {
		printk(KERN_ERR
			"bug_camera_init() - input_register_device failed.\n");
		input_free_device(bug_cam.input_dev);
		goto exit;
	}

	//Register character device.
	/*
	// allocate char device number
	err = alloc_chrdev_region (&bug_cam.cntl_devno, 0, 1, 
		     "bug_camera_control");
	if (err < 0) {
		printk(KERN_ERR "bug_camera_init(): alloc_chrdev_region failed.\n"); 
		goto err_exit;
	}

	// init and add control character device
	cdev_init (&bug_cam.cntl, &cntl_fops);
	
	err = cdev_add (&bug_cam.cntl, bug_cam.cntl_devno, 1);
	if (err )  {
		printk(KERN_ERR "bmi_camera_init(): cdev_add failed\n"); 
		goto err_exit2;
	}

	// create class device 
	bmi_class = bmi_get_bmi_class ();                            

	bug_cam.class_dev = device_create (bmi_class, NULL, bug_cam.cntl_devno, &bug_cam, "bug_camera_control");  
								     
	if (IS_ERR(bug_cam.class_dev)) {                                
		printk(KERN_ERR "Unable to create "                  
		       "class_device for bug_camera; errno = %ld\n",
		       PTR_ERR(bug_cam.class_dev));             
		bug_cam.class_dev = NULL;
		goto err_exit2;
	}
	*/
	goto exit;

err_exit2:
	unregister_chrdev_region (bug_cam.cntl_devno, 1);
err_exit:
	input_unregister_device (bug_cam.input_dev);
exit:
	return err;
}

static void __exit bug_camera_clean(void)
{	
  /*	struct class *bmi_class;

	bmi_class = bmi_get_bmi_class ();
	device_destroy (bmi_class, bug_cam.cntl_devno);*/

	bug_cam.class_dev = 0;
	// Unregister character device.
	unregister_chrdev_region (bug_cam.cntl_devno, 1);

	// Unregister input device.
	input_unregister_device (bug_cam.input_dev);

//	input_free_device (bug_cam.input_dev);
	return;
}


module_init(bug_camera_init);
module_exit(bug_camera_clean);

// Exported symbols
EXPORT_SYMBOL(camera_sensor_if);
EXPORT_SYMBOL(camera_link_if);
EXPORT_SYMBOL(register_bug_camera);
EXPORT_SYMBOL(unregister_bug_camera);

MODULE_AUTHOR("EnCADIS Design, Inc.");
MODULE_DESCRIPTION("Bug Camera Driver");
MODULE_LICENSE("GPL");

