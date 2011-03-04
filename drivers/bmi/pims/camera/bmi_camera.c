/*
 * drivers/bmi/pims/camera/bmi_camera.c
 *
 * Copyright (C) 2010 Lane Brooks
 *
 * Contact: Lane Brooks <dirjud@gmail.com>
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

/* We implement this as a bridge driver to bridge v4l2 and bmi. As
 * such we implement two drivers. One is an i2c v4l2 subdevice. It
 * doesn't need to be i2c, but the omap3-isp driver requires it, so we
 * create the bug_camera_subdev as a dummy i2c device. The second is a
 * platform driver called "bug_camera" that bug camera modules must
 * implement. The bug_camera interface is very similar to the V4L
 * interface. This module takes care of farming out V4L commands to
 * the appropriate camera module, so it is also a mux.
 *
 * To add new camera module, one must call the
 * bmi_register_camera()/bmi_unregister_camera() functions and fill
 * in the applicable ops in the bmi_camera_platform_data struct.
 */

#include <linux/bmi.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include "../../../drivers/media/video/isp/isp.h"
#include "../../../drivers/media/video/isp/ispreg.h"
#include <linux/gpio.h>

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/v4l2-subdev.h>
#include <linux/v4l2-mediabus.h>

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "bmi_camera.h"
#include <linux/bmi/bmi_camera.h>

#include "../../../arch/arm/mach-omap2/devices.h"
//extern struct platform_device omap3isp_device;
static int major;		// control device major


/* We declare this module as an i2c device simply because the
 * omap3-isp requires a i2c subdev. In reality this module is
 * simply a mux between any bug camera modules that plug
 * and register as such. It registers a 'fake' i2c address
 * only because it has to. */
static struct i2c_board_info bmi_camera_i2c_devices[] = {
	{
		I2C_BOARD_INFO("bug_camera_subdev", BMI_CAMERA_I2C_ADDR),
		.platform_data = NULL,
	},
};

static struct v4l2_subdev_i2c_board_info bmi_camera_primary_subdevs[] = {
	{
		.board_info = &bmi_camera_i2c_devices[0],
		.i2c_adapter_id = 3,
		.module_name = "bug_camera_subdev",
	},
	{ NULL, 0, NULL, },
};

static struct isp_subdevs_group bmi_camera_subdevs[] = {
	{ bmi_camera_primary_subdevs, ISP_INTERFACE_PARALLEL, },
	{ NULL, 0, },
};


static struct isp_platform_data bmi_isp_platform_data = {
	.parallel = {
		.data_lane_shift	= 3,
		.clk_pol		= 0,
		.bridge                 = ISPCTRL_PAR_BRIDGE_DISABLE,
		//.bridge                 = ISPCTRL_PAR_BRIDGE_LENDIAN, 
		//.bridge                 = ISPCTRL_PAR_BRIDGE_BENDIAN, 
	},
	.subdevs = bmi_camera_subdevs,
};


static struct bmi_camera_selector bmi_camera_sel;

static struct bmi_camera_platform_data *bmi_camera_get_selected_pdat(void)
{
	struct bmi_camera_platform_data *pdat = NULL;
	mutex_lock(&bmi_camera_sel.mutex);
	if(bmi_camera_sel.selected >= 0)
		pdat = bmi_camera_sel.pdat[bmi_camera_sel.selected];
	mutex_unlock(&bmi_camera_sel.mutex);
	return pdat;
}

static int bmi_camera_get_selected_slot(void) {
	int ret=0;
	mutex_lock(&bmi_camera_sel.mutex);
	ret = bmi_camera_sel.selected;
	mutex_unlock(&bmi_camera_sel.mutex);
	return ret;
}

static int bmi_camera_set_selected_slot(int slotnum) {
	int ret;
	if(slotnum < 0 || slotnum > 3)
		return -EINVAL;
	mutex_lock(&bmi_camera_sel.mutex);
	if(bmi_camera_sel.busy) {
		ret = -EBUSY;
	} else if(!bmi_camera_sel.pdat[slotnum]) {
		ret = -EINVAL;
	} else {
		bmi_camera_sel.selected = slotnum;
		ret = 0;
	}
	mutex_unlock(&bmi_camera_sel.mutex);
	return ret;
}

static int bmi_camera_select_available_slot(void) {
	int slotnum;
	mutex_lock(&bmi_camera_sel.mutex);
	if(bmi_camera_sel.busy) {
		mutex_unlock(&bmi_camera_sel.mutex);
		return -EBUSY;
	}
	bmi_camera_sel.selected = -1; // start by assuming no cameras available
	for(slotnum=0; slotnum<4; slotnum++) { // cycle to find next avail slot
		if(bmi_camera_sel.pdat[slotnum]) {
			bmi_camera_sel.selected = slotnum;
			break;
		}
	}
	mutex_unlock(&bmi_camera_sel.mutex);
	return 0;
}

int cntl_open(struct inode *inode, struct file *file)
{	
	struct bmi_camera_platform_data *pdat;
	pdat = container_of (inode->i_cdev, struct bmi_camera_platform_data, 
			     cdev);

	// Enforce single-open behavior
	if (pdat->open_flag) {
		return -EBUSY; 
	}
	pdat->open_flag = 1;

	// Save platform data pointer for later.
	file->private_data = pdat;
	return 0;
}

int cntl_release(struct inode *inode, struct file *file)
{	
	struct bmi_camera_platform_data *pdat;
	pdat = (struct bmi_camera_platform_data *)(file->private_data);
	pdat->open_flag = 0;
	return 0;
}

int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct bmi_camera_platform_data *pdat;
	int slotnum;

	pdat = (struct bmi_camera_platform_data *)(file->private_data);
	slotnum = pdat->bdev->slot->slotnum;

	// ioctl's
	switch (cmd) {
	case BMI_CAM_RLEDOFF:
		if(pdat->bmi_ops->set_red_led)
			return pdat->bmi_ops->set_red_led(pdat->bdev, 0);
		else
			return -EINVAL;

	case BMI_CAM_RLEDON:
		if(pdat->bmi_ops->set_red_led)
			return pdat->bmi_ops->set_red_led(pdat->bdev, 1);
		else
			return -EINVAL;

	case BMI_CAM_GLEDOFF:
		if(pdat->bmi_ops->set_green_led)
			return pdat->bmi_ops->set_green_led(pdat->bdev, 0);
		else
			return -EINVAL;

	case BMI_CAM_GLEDON:
		if(pdat->bmi_ops->set_green_led)
			return pdat->bmi_ops->set_green_led(pdat->bdev, 1);
		else
			return -EINVAL;

	case BMI_CAM_SUSPEND:
		if(pdat->bmi_ops->suspend)
			return pdat->bmi_ops->suspend(pdat->bdev);
		else
			return -EINVAL;

	case BMI_CAM_RESUME:
		if(pdat->bmi_ops->resume)
			return pdat->bmi_ops->resume(pdat->bdev);
		else
			return -EINVAL;

	default:
		if(pdat->bmi_ops->ioctl)
			return pdat->bmi_ops->ioctl(pdat->bdev,cmd,(void*) arg);
		else
			return -ENOTTY;
	}
	return 0;
}

static void bmi_camera_platform_release(struct device *dev)
{
}

int bmi_register_camera(struct bmi_device *bdev, 
			struct v4l2_subdev_ops *v4l2_ops,
			struct bmi_camera_ops *bmi_ops,
			struct module *mod) 
{
	struct file_operations *fops          = NULL;
	struct bmi_camera_platform_data *pdat = NULL;
	struct platform_device *pdev          = NULL;
	struct class *bmi_class;
	int ret, slotnum;
	dev_t dev_id;

	slotnum  = bdev->slot->slotnum;

        // control file operations
	fops = kzalloc(sizeof(*pdat), GFP_KERNEL);
	pdat = kzalloc(sizeof(*pdat), GFP_KERNEL);
	pdev = kzalloc(sizeof(*pdev), GFP_KERNEL);
	if (!fops || !pdat || !pdev) {
		ret = -1;
		goto err;
	}
	fops->owner   = mod;
	fops->ioctl   = cntl_ioctl;
	fops->open    = cntl_open;
	fops->release = cntl_release;

	pdat->bdev      = bdev;
	pdat->pdev      = pdev;
	pdat->v4l2_ops  = v4l2_ops;
	pdat->bmi_ops   = bmi_ops;
	pdev->name      = "bug_camera";
	pdev->id        = -1;
	pdev->num_resources = 0;
	pdev->resource  = NULL;
	pdev->dev.release = bmi_camera_platform_release;
	pdev->dev.platform_data = pdat;
	
	ret = platform_device_register(pdev);
	if(ret < 0) {
		goto err;
	}

	// Create 1 minor device
	cdev_init (&pdat->cdev, fops);
	dev_id = MKDEV(major, slotnum); 
	ret = cdev_add (&pdat->cdev, dev_id, 1);
	if (ret < 0) {
		goto err;
	}

	// Create class device 
	bmi_class = bmi_get_class ();                            
	pdat->class_dev = device_create (bmi_class, NULL, 
					 MKDEV (major, slotnum), NULL, 
					 "bmi_cam%i", slotnum);

	if (IS_ERR(pdat->class_dev)) {                                
		printk (KERN_ERR "Unable to create "                  
		       "class_device for bmi_cam%i; errno = %ld\n",
		       slotnum, PTR_ERR(pdat->class_dev));             
		pdat->class_dev = NULL;
		cdev_del (&pdat->cdev);
		ret = -ENODEV;
		goto err;
	}
	return 0;

err:
	kfree(pdat);
	kfree(pdev);
	kfree(fops);
	return ret;
}
EXPORT_SYMBOL(bmi_register_camera);

int bmi_unregister_camera(struct bmi_device *bdev)
{
	struct class *bmi_class;
	struct bmi_camera_platform_data *pdat;
	const struct file_operations *fops;
	int slotnum;
	if(!bdev)
		return -ENODEV;
	mutex_lock(&bmi_camera_sel.mutex);
	bmi_class = bmi_get_class ();
	for(slotnum=0; slotnum<4; slotnum++) {
		pdat = bmi_camera_sel.pdat[slotnum];
		if(pdat && pdat->bdev == bdev) {
			mutex_unlock(&bmi_camera_sel.mutex);
			platform_device_unregister (pdat->pdev);
			device_destroy (bmi_class, MKDEV(major, slotnum));
			fops = pdat->cdev.ops;
			cdev_del (&pdat->cdev);
			bmi_device_set_drvdata (bdev, 0);
			kfree(fops);
			kfree(pdat->pdev);
			kfree(pdat);
			return 0;
		}
	}
	mutex_unlock(&bmi_camera_sel.mutex);
	return -ENODEV;
}
EXPORT_SYMBOL(bmi_unregister_camera);

int bmi_camera_platform_probe(struct platform_device *pdev)
{
	struct bmi_camera_platform_data *pdat = pdev->dev.platform_data;
	struct bmi_device *bdev = pdat->bdev;
	int rval = 0;
	int slotnum = bdev->slot->slotnum;

	mutex_lock(&bmi_camera_sel.mutex);
	bmi_camera_sel.pdat[slotnum] = pdat;

	// first camera plugged in gets selected
	if(bmi_camera_sel.selected == -1)
		bmi_camera_sel.selected = slotnum;
	bmi_camera_sel.count++;

	mutex_unlock(&bmi_camera_sel.mutex);
	printk(KERN_INFO "%s registered bug camera on slot %d\n", __func__, slotnum);
	return rval;
}

int bmi_camera_platform_remove(struct platform_device *pdev)
{
	struct bmi_camera_platform_data *pdat = pdev->dev.platform_data;
	struct bmi_device *bdev = pdat->bdev;
	int slotnum = bdev->slot->slotnum;
	printk(KERN_INFO "%s unregistering bug camera from slot %d\n", __func__, slotnum);
	mutex_lock(&bmi_camera_sel.mutex);
	bmi_camera_sel.pdat[slotnum] = NULL;
	bmi_camera_sel.count--;
	bmi_camera_sel.selected = -1;
	bmi_camera_sel.busy = 0;
	mutex_unlock(&bmi_camera_sel.mutex);
	bmi_camera_select_available_slot();

	return 0;
}


#define CAM_OSC_EN  37
#define CAM_REN     34
#define CAM_RCLK_RF 38
#define CAM_BUF_OEN 98
#define CAM_LOCKB  167

int bmi_camera_is_serdes_locked() {
	int val;
	val = gpio_get_value(CAM_LOCKB);
	if(val < 0)
		return val;
	else
		return !val;
}
EXPORT_SYMBOL(bmi_camera_is_serdes_locked);

int bmi_camera_set_power_bugbase(int on) {
	if(on) {
		gpio_set_value(CAM_OSC_EN, 1);
		gpio_set_value(CAM_REN,    1);
	} else {
		gpio_set_value(CAM_OSC_EN, 0);
		gpio_set_value(CAM_REN,    0);
	}		
	return 0;
}

static int setup_gpio(unsigned gpio, int value) {
	int ret;
	ret = gpio_request(gpio,  "bug_camera");
	if(ret < 0)
		return ret;
	return gpio_direction_output(gpio, value);
}


static ssize_t show_slot(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", bmi_camera_get_selected_slot());
}

static ssize_t store_slot(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	sscanf(buf, "%d", &value);
	bmi_camera_set_selected_slot(value);
	return size;
}

static ssize_t show_serdes_locked(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", bmi_camera_is_serdes_locked());
}

static DEVICE_ATTR(slot, S_IWUGO | S_IRUGO, show_slot, store_slot);
static DEVICE_ATTR(serdes_locked, S_IRUGO, show_serdes_locked, NULL);

#define GET_SELECTED_DEV                                                 \
	struct v4l2_subdev_ops *v4l2_ops = NULL;                         \
	struct bmi_device *bdev = NULL;                                  \
	struct bmi_camera_sensor *sensor = to_bmi_camera_sensor(subdev); \
	struct bmi_camera_platform_data *pdat = bmi_camera_get_selected_pdat();\
	if(!pdat) return -EINVAL;                                        \
	v4l2_ops = pdat->v4l2_ops;                                       \
	bdev = pdat->bdev;                                               \
	sensor->bdev = bdev

static int bmi_camera_s_stream(struct v4l2_subdev *subdev, int streaming)
{
	int err=0;
	GET_SELECTED_DEV;
	mutex_lock(&bmi_camera_sel.mutex);
	if(v4l2_ops->video && v4l2_ops->video->s_stream)
		err = v4l2_ops->video->s_stream(subdev, streaming);

	if(streaming && err==0) {
		bmi_camera_sel.busy = 1;
	} else {
		bmi_camera_sel.busy = 0;
	}		
	mutex_unlock(&bmi_camera_sel.mutex);
	return err;
}

static int bmi_camera_log_status(struct v4l2_subdev *subdev) 
{
	GET_SELECTED_DEV;
	printk(KERN_INFO "BMI CAMERA STATUS : %s\n", __func__);
	printk(KERN_INFO " Input: Slot %d\n", bmi_camera_sel.selected);
	if(v4l2_ops && v4l2_ops->core && v4l2_ops->core->log_status)
		return v4l2_ops->core->log_status(subdev);
	else 
		return 0;
}

static int bmi_camera_set_config(struct v4l2_subdev *subdev, 
				 int irq, void *platform_data)
{
	return 0; // nothing to do for now
}

static int bmi_camera_query_ctrl(struct v4l2_subdev *subdev,
				  struct v4l2_queryctrl *a)
{
	struct bmi_camera_platform_data *pdat = bmi_camera_get_selected_pdat();

	switch (a->id) {
	case V4L2_CID_FLASH_STROBE:
		a->type = V4L2_CTRL_TYPE_MENU;
		sprintf(a->name, "Flash Strobe");
		a->minimum = 0;
		a->maximum = 2;
		a->default_value = 0;

		// now determine if the selected camera supports flash strobe
		if(pdat && pdat->bmi_ops && pdat->bmi_ops->set_flash_strobe) {
			a->flags = 0;
		} else {
			a->flags = V4L2_CTRL_FLAG_DISABLED;
		}
		break;
	default:
		// default is pass the query control onto the selected camera
		if(pdat && pdat->v4l2_ops && pdat->v4l2_ops->core &&
		   pdat->v4l2_ops->core->queryctrl) {
			return pdat->v4l2_ops->core->queryctrl(subdev, a);
		} else {
			return -EINVAL;
		}
	}
	return 0;
}

static int bmi_camera_query_menu(struct v4l2_subdev *subdev,
				  struct v4l2_querymenu *qm)
{
	struct bmi_camera_platform_data *pdat = bmi_camera_get_selected_pdat();

	switch (qm->id) {
	case V4L2_CID_FLASH_STROBE:
		switch (qm->index) {
		case FLASH_STROBE_OFF:
			sprintf(qm->name, "OFF");
			break;
		case FLASH_STROBE_TORCH_EN:
			sprintf(qm->name, "TORCH (LOW BEAM)");
			break;
		case FLASH_STROBE_HIGH_BEAM_EN:
			sprintf(qm->name, "PULSE (HIGH BEAM)");
			break;
		case 3:
			return -EINVAL;
		}
		break;
	default:
		// default is pass the query menu onto the selected camera
		if(pdat && pdat->v4l2_ops && pdat->v4l2_ops->core &&
		   pdat->v4l2_ops->core->querymenu) {
			return pdat->v4l2_ops->core->querymenu(subdev, qm);
		} else {
			return -EINVAL;
		}
	}
	return 0;
}

static int bmi_camera_get_ctrl(struct v4l2_subdev *subdev,
			       struct v4l2_control *vc)
{
	GET_SELECTED_DEV;
	if(v4l2_ops->core && v4l2_ops->core->g_ctrl)
		return v4l2_ops->core->g_ctrl(subdev, vc);
	return -EINVAL;
}

static int bmi_camera_set_ctrl(struct v4l2_subdev *subdev,
			       struct v4l2_control *vc)
{
	GET_SELECTED_DEV;
	if(vc->id == V4L2_CID_FLASH_STROBE) {
		if(pdat && pdat->bmi_ops && pdat->bmi_ops->set_flash_strobe)
			return pdat->bmi_ops->set_flash_strobe(bdev, vc->value);
		else
			return -EINVAL;
	} else if(v4l2_ops->core && v4l2_ops->core->s_ctrl) {
		return v4l2_ops->core->s_ctrl(subdev, vc);
	}
	return -EINVAL;
}

static int bmi_camera_set_power(struct v4l2_subdev *subdev, int on)
{
	GET_SELECTED_DEV;
	if(v4l2_ops->core && v4l2_ops->core->s_power)
		return v4l2_ops->core->s_power(subdev, on);
	return 0;
}

#define MAX_FMTS 1
static int bmi_camera_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	GET_SELECTED_DEV;
	if(v4l2_ops->pad && v4l2_ops->pad->enum_frame_size)
		return v4l2_ops->pad->enum_frame_size(subdev, fh, fse);
	return -EINVAL;
}

static int bmi_camera_enum_frame_interval(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_interval_enum *fie)
{
	GET_SELECTED_DEV;
	if(v4l2_ops->pad && v4l2_ops->pad->enum_frame_interval)
		return v4l2_ops->pad->enum_frame_interval(subdev, fh, fie);
	return -EINVAL;
}


static int bmi_camera_enum_mbus_code(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_pad_mbus_code_enum *code)
{
	GET_SELECTED_DEV;
	if(v4l2_ops->pad && v4l2_ops->pad->enum_mbus_code)
		return v4l2_ops->pad->enum_mbus_code(subdev, fh, code);
	return -EINVAL;
}

static int bmi_camera_get_pad_format(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_fh *fh, unsigned int pad,
				 struct v4l2_mbus_framefmt *fmt,
				 enum v4l2_subdev_format which)
{
	GET_SELECTED_DEV;
	if(v4l2_ops->pad && v4l2_ops->pad->get_fmt) {
		return 	v4l2_ops->pad->get_fmt(subdev, fh, pad, fmt, which);
	}
	return -EINVAL;
}

static int bmi_camera_set_pad_format(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_fh *fh, unsigned int pad,
				 struct v4l2_mbus_framefmt *fmt,
				 enum v4l2_subdev_format which)
{
	GET_SELECTED_DEV;
	if(v4l2_ops->pad && v4l2_ops->pad->set_fmt)
		return v4l2_ops->pad->set_fmt(subdev, fh, pad, fmt, which);
	return -EINVAL;
}

static int bmi_camera_get_frame_interval(struct v4l2_subdev *subdev,
					 struct v4l2_subdev_frame_interval *fi)
{
	GET_SELECTED_DEV;
	if(v4l2_ops->video && v4l2_ops->video->g_frame_interval)
		return v4l2_ops->video->g_frame_interval(subdev, fi);
	return -EINVAL;
}

static int bmi_camera_set_frame_interval(struct v4l2_subdev *subdev,
					 struct v4l2_subdev_frame_interval *fi)
{
	GET_SELECTED_DEV;
	if(v4l2_ops->video && v4l2_ops->video->s_frame_interval)
		return v4l2_ops->video->s_frame_interval(subdev, fi);
	return -EINVAL;
}

static int bmi_camera_query_capabilities(struct v4l2_capability *argp) {
	sprintf(argp->driver, "bug_camera");
	sprintf(argp->card, "bug_base");
	sprintf(argp->bus_info, "BMI");
	argp->version = KERNEL_VERSION(0,0,1);
	argp->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}

static int bmi_camera_enum_input(struct v4l2_input *argp) {
	if(argp->index >= 4)
		return -EINVAL;
	sprintf(argp->name, "BUG CAMERA SLOT %d", argp->index);
	argp->type = V4L2_INPUT_TYPE_CAMERA;
	argp->audioset = 0; // currently no audio
	argp->tuner = 0;
	argp->std = 0; // not sure what to put here

	// from the docs, status is only valid when this is the current input
	mutex_lock(&bmi_camera_sel.mutex);
	if(bmi_camera_sel.selected == argp->index) {
		argp->status = 
			(bmi_camera_sel.pdat[argp->index] ? 0x0 : 
			 V4L2_IN_ST_NO_ACCESS) |
			(bmi_camera_is_serdes_locked() ? 0x0 : 
			 V4L2_IN_ST_NO_SYNC);
	} else {
		argp->status = 0xFFFFFFFF;
	}
	mutex_unlock(&bmi_camera_sel.mutex);
	return 0;
}

static long bmi_camera_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch(cmd) {
	case VIDIOC_QUERYCAP:
		return bmi_camera_query_capabilities(arg);
	case VIDIOC_ENUMINPUT:
		return bmi_camera_enum_input(arg);
	case VIDIOC_S_INPUT:
		return bmi_camera_set_selected_slot(*((int*) arg));
	case VIDIOC_G_INPUT:
		*((int*) arg) = bmi_camera_get_selected_slot();
		return 0;
	case VIDIOC_LOG_STATUS:
		return bmi_camera_log_status(sd);
	default:
		return -1;
	}
}

static int bmi_camera_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int slotnum;
	struct bmi_camera_platform_data *pdat;
	mutex_lock(&bmi_camera_sel.mutex);
	for(slotnum=0; slotnum<4; slotnum++) {
		pdat = bmi_camera_sel.pdat[slotnum];
		if(pdat && pdat->bmi_ops->suspend) {
			pdat->bmi_ops->suspend(pdat->bdev);
		}
	}
	mutex_unlock(&bmi_camera_sel.mutex);
	return 0;
}
static int bmi_camera_resume(struct i2c_client *client)
{
	int slotnum;
	struct bmi_camera_platform_data *pdat;
	mutex_lock(&bmi_camera_sel.mutex);
	for(slotnum=0; slotnum<4; slotnum++) {
		pdat = bmi_camera_sel.pdat[slotnum];
		if(pdat && pdat->bmi_ops->resume) {
			pdat->bmi_ops->resume(pdat->bdev);
		}
	}
	mutex_unlock(&bmi_camera_sel.mutex);
	return 0;
}

static const struct v4l2_subdev_video_ops bmi_camera_video_ops = {
	.s_stream            = bmi_camera_s_stream,
	.g_frame_interval    = bmi_camera_get_frame_interval,
	.s_frame_interval    = bmi_camera_set_frame_interval,
};

static const struct v4l2_subdev_core_ops bmi_camera_core_ops = {
	.log_status   = bmi_camera_log_status,
	.s_config     = bmi_camera_set_config,
	.queryctrl    = bmi_camera_query_ctrl,
	.querymenu    = bmi_camera_query_menu,
	.g_ctrl       = bmi_camera_get_ctrl,
	.s_ctrl       = bmi_camera_set_ctrl,
	.s_power      = bmi_camera_set_power,
	.ioctl        = bmi_camera_ioctl,
};

static const struct v4l2_subdev_pad_ops bmi_camera_pad_ops = {
	.enum_mbus_code      = bmi_camera_enum_mbus_code,
        .enum_frame_size     = bmi_camera_enum_frame_size,
        .enum_frame_interval = bmi_camera_enum_frame_interval,
	.get_fmt             = bmi_camera_get_pad_format,
	.set_fmt             = bmi_camera_set_pad_format,
};

static const struct v4l2_subdev_ops bmi_camera_ops = {
	.core  = &bmi_camera_core_ops,
	.video = &bmi_camera_video_ops,
	.pad   = &bmi_camera_pad_ops,
};

static const struct media_entity_operations bmi_camera_entity_ops = {
	.set_power = v4l2_subdev_set_power,
};


static int bmi_camera_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct bmi_camera_sensor *sensor;
	int ret;

	sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (sensor == NULL)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&sensor->subdev, client, &bmi_camera_ops);

	sensor->pad.type = MEDIA_PAD_TYPE_OUTPUT;
	sensor->subdev.entity.ops = &bmi_camera_entity_ops;
	sensor->bdev = NULL;
	ret = media_entity_init(&sensor->subdev.entity, 1, &sensor->pad, 0);
	if (ret < 0) {
		kfree(sensor);
		return ret;
	}

	ret = device_create_file(&client->dev, &dev_attr_slot);
	ret |= device_create_file(&client->dev, &dev_attr_serdes_locked);
	return ret;
}

static int __exit bmi_camera_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct bmi_camera_sensor *sensor = to_bmi_camera_sensor(subdev);

	v4l2_device_unregister_subdev(&sensor->subdev);
	kfree(sensor);
	return 0;
}

static const struct i2c_device_id bmi_camera_subdev_id_table[] = {
	{ "bug_camera_subdev", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bmi_camera_subdev_id_table);

static struct i2c_driver bmi_camera_i2c_driver = {
	.driver		= {
		.name	= "bug_camera_subdev",
		.owner = THIS_MODULE,
	},
	.probe		= bmi_camera_probe,
	.remove		= __exit_p(bmi_camera_remove),
	.suspend	= bmi_camera_suspend,
	.resume		= bmi_camera_resume,
	.id_table	= bmi_camera_subdev_id_table,
};

static struct platform_driver bmi_camera_platform_driver = {
	.driver         = {
		.name   = "bug_camera",
		.owner  = THIS_MODULE,
	},
	.probe		= bmi_camera_platform_probe,
	.remove		= bmi_camera_platform_remove,
};

static __init int bmi_camera_init(void)
{
	dev_t	dev_id;
	int ret, i;
	// initialize slot selection code
	mutex_init(&bmi_camera_sel.mutex);
	bmi_camera_sel.selected = -1;
	bmi_camera_sel.count = 0;
	bmi_camera_sel.busy  = 0;
	for(i=0; i<4; i++) {
		bmi_camera_sel.pdat[i] = NULL;
	}

	ret = i2c_add_driver(&bmi_camera_i2c_driver);
	if (ret) {
		printk(KERN_ERR "%s: failed registering i2c driver\n",__func__);
		return ret;
	}

	ret = platform_driver_register(&bmi_camera_platform_driver);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed registering plaform\n", __func__);
	}

	omap3isp_device.dev.platform_data = &bmi_isp_platform_data;
	// unlock the mutex to prevent nested locking while registering
	ret = platform_device_register(&omap3isp_device);
	if(ret < 0) {
		printk(KERN_ERR "Error register omap3-isp\n");
		return ret;
	}

	ret = setup_gpio(CAM_OSC_EN, 1);
	if(ret < 0)
		goto err5;
	ret = setup_gpio(CAM_REN, 1);
	if(ret < 0)
		goto err4;
	ret = setup_gpio(CAM_RCLK_RF, 1);
	if(ret < 0)
		goto err3;
	ret = setup_gpio(CAM_BUF_OEN, 0);
	if(ret < 0)
		goto err2;
	ret = gpio_request(CAM_LOCKB, "bug_camera");
	if(ret < 0)
		goto err1;
	gpio_direction_input(CAM_LOCKB);


	// alloc char driver with 4 minor numbers
	ret = alloc_chrdev_region (&dev_id, 0, 4, "BMI CAMERA Driver"); 
	if (ret) {
		ret = -ENODEV;
		goto err1;
	}

	major = MAJOR(dev_id);

	bmi_camera_set_power_bugbase(1);

	return 0;

err1:
	gpio_free(CAM_LOCKB);
err2:
	gpio_free(CAM_BUF_OEN);
err3:
	gpio_free(CAM_RCLK_RF);
err4:
	gpio_free(CAM_REN);
err5:
	gpio_free(CAM_OSC_EN);
	return ret;
}

static void __exit bmi_camera_cleanup(void)
{	
	dev_t dev_id;
	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);

	bmi_camera_set_power_bugbase(0);
	gpio_direction_output(98, 1); // CAM_OE#
	gpio_set_value(CAM_OSC_EN, 0);
	gpio_set_value(CAM_REN,    0);

	gpio_free(CAM_OSC_EN);
	gpio_free(CAM_REN);
	gpio_free(CAM_RCLK_RF);
	gpio_free(CAM_BUF_OEN);
	gpio_free(CAM_LOCKB);

	platform_device_unregister(&omap3isp_device);
	platform_driver_unregister(&bmi_camera_platform_driver);
	i2c_del_driver(&bmi_camera_i2c_driver);
}

module_init(bmi_camera_init);
module_exit(bmi_camera_cleanup);

MODULE_AUTHOR("Lane Brooks");
MODULE_DESCRIPTION("BMI Camera Driver");
MODULE_LICENSE("GPL");
