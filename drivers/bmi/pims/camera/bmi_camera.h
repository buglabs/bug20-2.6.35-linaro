/*
 * drivers/bmi/pims/camera/bmi_camera_mux.h
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

#ifndef __BMI_CAMERA_H
#define __BMI_CAMERA_H
#include <media/v4l2-subdev.h>
#include <linux/bmi.h>
struct mutex;

struct bmi_camera_ops {
	int (*set_flash_strobe)(struct bmi_device *bdev, int on);
	int (*set_red_led)(struct bmi_device *bdev, int on);
	int (*set_green_led)(struct bmi_device *bdev, int on);
	int (*ioctl)(struct bmi_device *bdev, unsigned int cmd, void *argp);
	int (*suspend)(struct bmi_device *bdev);
	int (*resume)(struct bmi_device *bdev);
};

struct bmi_camera_sensor {
	struct v4l2_subdev subdev;
	struct media_entity_pad pad;
	struct bmi_device *bdev;
};

struct bmi_camera_platform_data {
	struct bmi_device      *bdev;
	struct platform_device *pdev;
	struct v4l2_subdev_ops *v4l2_ops;
	struct bmi_camera_ops  *bmi_ops;
	struct device          *class_dev;  // control class device
	struct cdev            cdev;	    // control device
	int                    open_flag;
};

#define to_bmi_camera_sensor(sd) container_of(sd, struct bmi_camera_sensor, subdev)

struct bmi_camera_selector {
	struct mutex mutex; /* atomic access to slot selection data */
	struct bmi_camera_platform_data *pdat[4];
	int selected;
	int count;
	int busy; // indicates selected device is in use and cannot be switched
};

// This is an arbitrary I2C address (see comment in .c file)
#define BMI_CAMERA_I2C_ADDR 0x38

extern int bmi_register_camera(struct bmi_device *bdev, 
			       struct v4l2_subdev_ops *ops,
			       struct bmi_camera_ops *bmi_ops,
			       struct module *mod);
extern int bmi_unregister_camera(struct bmi_device *bdev);
extern int bmi_camera_is_serdes_locked(void);



#endif
