/*
 * drivers/bmi/pims/camera/bmi_li3m02cm.c
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
#include <linux/bmi.h>
#include <linux/bmi/bmi-control.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/delay.h>

#include "bmi_camera.h"
#include <linux/bmi/bmi_camera.h>
#include "mt9t111.h"

#define BMI_LI3M02CM_VERSION  "0.1.0.5"

// BMI device ID table
static struct bmi_device_id bmi_li3m02cm_tbl[] = 
{ 
	{ .match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
	  .vendor   = BMI_VENDOR_BUG_LABS, 
	  .product  = BMI_PRODUCT_CAMERA_LI3M02CM, 
	  .revision = BMI_ANY, 
	}, 
	{ 0, },				      /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_li3m02cm_tbl);

int	bmi_li3m02cm_probe(struct bmi_device *bdev);
void 	bmi_li3m02cm_remove(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_li3m02cm_driver = 
{
	.name = "bmi_li3m02cm", 
	.id_table = bmi_li3m02cm_tbl, 
	.probe   = bmi_li3m02cm_probe, 
	.remove  = bmi_li3m02cm_remove, 
	};


struct bmi_li3m02cm {
	struct bmi_device *bdev;		
	struct i2c_client *iox;
	struct i2c_client *mt9t111;
	u8  sysfs_iox_i2c_addr;
	u16 sysfs_mt9t111_i2c_addr;
};

	// I2C Slave Address
#define BMI_IOX_I2C_ADDRESS	0x38	// 7-bit address

	// I2C IOX register addresses
#define IOX_INPUT_REG		0x0
#define IOX_OUTPUT_REG		0x1
#define IOX_POLARITY_REG	0x2
#define IOX_CONTROL		0x3

#define IOX_CAM_RESETB          0x80
#define IOX_GREEN_LED           0x40
#define IOX_SER_SYNC            0x20
#define IOX_FLASH_TORCHB        0x10
#define IOX_STROBE              0x08
#define IOX_CAM_STBY            0x04

#define GPIO_SER_EN             1
#define GPIO_FLASHON            2
#define GPIO_REDLED             3

static struct i2c_board_info iox_info = {
	I2C_BOARD_INFO("CAM_IOX", BMI_IOX_I2C_ADDRESS),
};
static struct i2c_board_info mt9t111_info = {
	I2C_BOARD_INFO(MT9T111_NAME, MT9T111_I2C_ADDR),
};

// read byte from I2C IO expander
static int ReadByte_IOX (struct i2c_client *client, 
			 unsigned char offset, 
			 unsigned char *data)
{
#ifdef REVB
        int    ret = 0;
        struct i2c_msg rmsg[2];

        /* Read Byte with Pointer */
        rmsg[0].addr = client->addr;
        rmsg[0].flags = 0;          /* write */
        rmsg[0].len = 1;
        rmsg[0].buf = &offset;

        rmsg[1].addr = client->addr;
        rmsg[1].flags = I2C_M_RD;   /* read */ 
        rmsg[1].len = 1;
        rmsg[1].buf = data;

        ret = i2c_transfer (client->adapter, rmsg, 2);

        if (ret == 2) {
		printk (KERN_ERR "ReadByte_IOX() - addr=0x%x data=0x%02X\n", offset, *data);
		
                ret = 0;
        }
        else {
                //Rework: add conditional debug messages here
		printk (KERN_ERR "ReadByte_IOX() - i2c_transfer failed\n");
                ret = -1;
        }
        return ret;
#else
	int     ret;
	if(offset == IOX_INPUT_REG || offset == IOX_OUTPUT_REG) {
		ret = i2c_master_recv(client, data, 1);
		if (ret < 0)
			printk (KERN_ERR "ReadByte_IOX() - i2c_transfer() failed...%d\n",ret);
		return ret;
	} else {
		*data = 0xFF;
		return 0;
	}
#endif
}

// write byte to I2C IO expander
static int WriteByte_IOX (struct i2c_client *client, 
			  unsigned char offset,
			  unsigned char data)
{
#ifdef REVB
	int	ret = 0;
	unsigned char msg[2];
	printk (KERN_ERR "%s - addr=0x%x data=0x%02X\n", __func__, offset, data);
	
     	msg[0] = offset;
	msg[1] = data;
	ret = i2c_master_send(client, msg, sizeof(msg));
	
	if (ret < 0)
	  printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed...%d\n",ret);

	return ret;
#else
	int     ret = 0;
	if(offset == IOX_INPUT_REG || offset == IOX_OUTPUT_REG) {
		ret = i2c_master_send(client, &data, 1);
		if (ret < 0)
			printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed...%d\n",ret);
	}
	return ret;
#endif
}

static ssize_t show_iox_value(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	unsigned char iox_data;
	int ret;
	if(!cam)
		return sprintf(buf, "NULL camera handle\n");

	ret = ReadByte_IOX (cam->iox, cam->sysfs_iox_i2c_addr, &iox_data);
	if(ret < 0) 
		return sprintf(buf, "%d\n",   ret);
	else
		return sprintf(buf, "0x%02x\n", iox_data);
}
static ssize_t store_iox_value(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	long value;
	ssize_t status;
	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
		WriteByte_IOX(cam->iox, cam->sysfs_iox_i2c_addr, value);
	}
	return size;
}

static ssize_t show_iox_addr(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	return sprintf(buf, "0x%02x\n", cam->sysfs_iox_i2c_addr);
}

static ssize_t store_iox_addr(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	long value;
	ssize_t status;
	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
		cam->sysfs_iox_i2c_addr = value;
	}
	return size;
}

static ssize_t show_mt9t111_value(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	u16 val;
	int ret;
	if(!cam)
		return sprintf(buf, "NULL camera handle\n");

	ret = mt9t111_read_reg(cam->mt9t111, cam->sysfs_mt9t111_i2c_addr, &val);
	if(ret < 0) 
		return sprintf(buf, "%d\n",   ret);
	else
		return sprintf(buf, "0x%04x\n", val);
}
static ssize_t store_mt9t111_value(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	long value;
	ssize_t status;
	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
		mt9t111_write_reg(cam->mt9t111, cam->sysfs_mt9t111_i2c_addr, value);
	}
	return size;
}

static ssize_t show_mt9t111_addr(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	return sprintf(buf, "0x%04x\n", cam->sysfs_mt9t111_i2c_addr);
}

static ssize_t store_mt9t111_addr(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	long value;
	ssize_t status;
	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
		cam->sysfs_mt9t111_i2c_addr = value;
	}
	return size;
}

static int __li3m02cm_set_power(struct bmi_device *bdev, int on);

static ssize_t show_power(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "?\n");
}

static ssize_t store_power(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	long value = 0;
	strict_strtol(buf, 0, &value);
	if(value)
		__li3m02cm_set_power(cam->bdev, 1);
	else
		__li3m02cm_set_power(cam->bdev, 0);
	return size;
}

static ssize_t show_format(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	struct v4l2_mbus_framefmt fmt;
	mt9t111_get_format(cam->mt9t111, &fmt);
	return sprintf(buf, "%d %d %d\n", fmt.width, fmt.height, fmt.code);
}

static ssize_t store_format(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	struct v4l2_mbus_framefmt fmt;
	sscanf(buf, "%d %d %d", &(fmt.width), &(fmt.height), &(fmt.code));
	if(fmt.width && fmt.height && fmt.code)
		mt9t111_set_format(cam->mt9t111, &fmt);
	return size;
}

static ssize_t show_context(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u16 value;
	int result;
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	mt9t111_write_reg(cam->mt9t111, 0x098E, 0x8401);
	mt9t111_read_reg( cam->mt9t111,  0x0990, &value);
	if(value == 3) { 
		result = 1; 
	} else if(value == 7) { 
		result = 2; 
	} else {
		result = -value;
	}
	return sprintf(buf, "%d\n", result);
}

static ssize_t store_context(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	sscanf(buf, "%d", &value);
	mt9t111_write_reg(cam->mt9t111, 0x098E, 0x8400);
	mt9t111_write_reg(cam->mt9t111, 0x0990, (u16) value);
	return size;
}


static DEVICE_ATTR(iox_value, S_IWUGO | S_IRUGO, show_iox_value, store_iox_value);
static DEVICE_ATTR(iox_addr,  S_IWUGO | S_IRUGO, show_iox_addr, store_iox_addr);
static DEVICE_ATTR(mt9t111_value, S_IWUGO | S_IRUGO, show_mt9t111_value, store_mt9t111_value);
static DEVICE_ATTR(mt9t111_addr,  S_IWUGO | S_IRUGO, show_mt9t111_addr, store_mt9t111_addr);
static DEVICE_ATTR(set_power, S_IWUGO | S_IRUGO, show_power, store_power);
static DEVICE_ATTR(format, S_IWUGO | S_IRUGO, show_format, store_format);
static DEVICE_ATTR(context, S_IWUGO | S_IRUGO, show_context, store_context);

// configure IOX IO and states
void configure_IOX(struct bmi_li3m02cm *cam)
{	
	WriteByte_IOX(cam->iox, IOX_OUTPUT_REG, 0);
	WriteByte_IOX(cam->iox, IOX_CONTROL,    0x00); /* all outputs */
	WriteByte_IOX(cam->iox, IOX_OUTPUT_REG, 0);
}

// configure GPIO IO and states
void configure_GPIO(struct bmi_li3m02cm *cam)
{
	// set states before turning on outputs
	int slot = cam->bdev->slot->slotnum;
	bmi_slot_gpio_direction_out(slot, GPIO_REDLED,  0); // Red LED=OFF
	bmi_slot_gpio_direction_out(slot, GPIO_FLASHON, 0); // Flash LED=OFF
	bmi_slot_gpio_direction_out(slot, GPIO_SER_EN,  0); // SER_EN=OFF
}

// deconfigure IOX and GPIO
void deconfigure_module(struct bmi_li3m02cm *cam)
{
 	int slot = cam->bdev->slot->slotnum;
	bmi_slot_gpio_direction_in(slot, 3);
	bmi_slot_gpio_direction_in(slot, 2);
	bmi_slot_gpio_direction_in(slot, 1);
}

#define subdev_to_bdev(subdev) (((struct bmi_camera_sensor *) to_bmi_camera_sensor(subdev))->bdev)
#define subdev_to_cam(subdev) ((struct bmi_li3m02cm*) bmi_device_get_drvdata(subdev_to_bdev(subdev)))

static int li3m02cm_set_power(struct v4l2_subdev *subdev, int on)
{
	return __li3m02cm_set_power(subdev_to_bdev(subdev), on);
}

static int __li3m02cm_set_power(struct bmi_device *bdev, int on)
{
	struct bmi_li3m02cm *cam = bmi_device_get_drvdata(bdev);
	unsigned char iox_data;
	int ret;
	int slot = bdev->slot->slotnum;

	// setup the gpio pins
	bmi_slot_gpio_direction_out(slot, GPIO_SER_EN,  on ? 1 : 0);

	// setup the io expander pins on the camera module board
	ret = ReadByte_IOX (cam->iox, IOX_OUTPUT_REG, &iox_data);
	if(ret < 0)
		return ret;

	iox_data &= ~IOX_CAM_RESETB; // put part in reset regardless
	iox_data |=  IOX_CAM_STBY;   // and standby
	ret = WriteByte_IOX (cam->iox, IOX_OUTPUT_REG, iox_data);
	if(ret < 0)
		goto error;

	// now take part out reset and standby if a turn on is requested
	if(on) {
		iox_data |=  IOX_CAM_RESETB; // take out of reset 
		iox_data &= ~IOX_CAM_STBY;   // and standby
		iox_data |=  IOX_SER_SYNC;   // turn on serial sync
		ret = WriteByte_IOX (cam->iox, IOX_OUTPUT_REG, iox_data);
		if(ret < 0)
			goto error;
	}

	// set power to the sensor
	ret = mt9t111_set_power(cam->mt9t111, on);
	if(ret < 0) {
		printk(KERN_ERR "%s error setting power to mt9t111 (err=%d)\n", __func__, ret);
		goto error;
	}
	ret = mt9t111_s_stream(cam->mt9t111, on);
	if(ret < 0)
		goto error;

	if(on) {
		u8 retry_count = 0;
		while(1) { // wait for SERDES to lock
			ret = bmi_camera_is_serdes_locked();
			if(ret < 0)
				goto error;
			if(ret) {
				break; // we are locked
			} else {
				if(retry_count++ >= 20) {
					printk(KERN_ERR "Camera SERDES won't lock");
					ret = -EBUSY;
					goto error;
				} else {            // if not locked,   
					mdelay(10); // wait & test again
				}
			}
		}
		// turn off the serial sync option and check if lock occurred
		iox_data &= ~IOX_SER_SYNC;  // turn off serial sync
		ret = WriteByte_IOX (cam->iox, IOX_OUTPUT_REG, iox_data);
		if(ret < 0)
			goto error;		

		printk(KERN_DEBUG "Camera SERDES LOCKED\n");
	}
	return 0;

error:
	printk(KERN_ERR "%s error (err=%d)\n", __func__, ret);
	return ret;
}

static int li3m02cm_get_frame_interval(struct v4l2_subdev *subdev,
				       struct v4l2_subdev_frame_interval *fi)
{
	return mt9t111_get_frame_interval(subdev_to_cam(subdev)->mt9t111, fi);
}

static int li3m02cm_set_frame_interval(struct v4l2_subdev *subdev,
				       struct v4l2_subdev_frame_interval *fi)
{
	return mt9t111_set_frame_interval(subdev_to_cam(subdev)->mt9t111, fi);
}

static int li3m02cm_log_status(struct v4l2_subdev *subdev)
{
	return mt9t111_log_status(subdev_to_cam(subdev)->mt9t111);
}

static int li3m02cm_query_ctrl(struct v4l2_subdev *subdev,
			       struct v4l2_queryctrl *a)
{
	return mt9t111_query_ctrl(subdev_to_cam(subdev)->mt9t111, a);
}

static int li3m02cm_query_menu(struct v4l2_subdev *subdev,
			       struct v4l2_querymenu *qm)
{
	return mt9t111_query_menu(subdev_to_cam(subdev)->mt9t111, qm);
}

static int li3m02cm_get_ctrl(struct v4l2_subdev *subdev,
			     struct v4l2_control *vc)
{
	return mt9t111_get_ctrl(subdev_to_cam(subdev)->mt9t111, vc);
}

static int li3m02cm_set_ctrl(struct v4l2_subdev *subdev,
			     struct v4l2_control *vc)
{
	return mt9t111_set_ctrl(subdev_to_cam(subdev)->mt9t111, vc);
}

static int li3m02cm_enum_frame_size(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_fh *fh,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	return mt9t111_enum_frame_size(subdev_to_cam(subdev)->mt9t111, fh, fse);
}

static int li3m02cm_enum_frame_ival(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_fh *fh,
				    struct v4l2_subdev_frame_interval_enum *fie)
{
	return mt9t111_enum_frame_ival(subdev_to_cam(subdev)->mt9t111, fh, fie);
}
static int li3m02cm_enum_mbus_code(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_pad_mbus_code_enum *code)
{
	return mt9t111_enum_mbus_code(subdev_to_cam(subdev)->mt9t111, fh, code);
}

static int li3m02cm_get_pad_format(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_fh *fh, unsigned int pad,
				   struct v4l2_mbus_framefmt *fmt,
				   enum v4l2_subdev_format which)
{
	struct v4l2_mbus_framefmt *format;
		
	switch (which) {
	case V4L2_SUBDEV_FORMAT_PROBE:
		format = v4l2_subdev_get_probe_format(fh, pad);
		*fmt = *format;
		break;
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return mt9t111_get_format(subdev_to_cam(subdev)->mt9t111, fmt);
	default: 
		return -EINVAL;
	}
	return 0;
}

static int li3m02cm_set_pad_format(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_fh *fh, unsigned int pad,
				   struct v4l2_mbus_framefmt *fmt,
				   enum v4l2_subdev_format which)
{
	return mt9t111_set_format(subdev_to_cam(subdev)->mt9t111, fmt);
}

static int li3m02cm_set_flash_strobe(struct bmi_device *bdev, int on) 
{
	int ret;
	unsigned char iox_data;
	struct bmi_li3m02cm *cam = bmi_device_get_drvdata(bdev);
	int slotnum = bdev->slot->slotnum;

	// always turn off strobe initially so that high-beam can
	// be enabled correctly.
	bmi_slot_gpio_direction_out(slotnum, GPIO_FLASHON, 0); 
	switch (on) {
	case FLASH_STROBE_OFF:
		break;
	case FLASH_STROBE_TORCH_EN:
	case FLASH_STROBE_HIGH_BEAM_EN:
		ret = ReadByte_IOX (cam->iox, IOX_OUTPUT_REG, &iox_data);
		if(ret < 0)
			return ret;
		if(on == FLASH_STROBE_TORCH_EN)
			iox_data &= ~IOX_FLASH_TORCHB;
		else
			iox_data |=  IOX_FLASH_TORCHB;
		ret = WriteByte_IOX (cam->iox, IOX_OUTPUT_REG, iox_data);
		if(ret < 0)
			return ret;
		bmi_slot_gpio_direction_out(slotnum, GPIO_FLASHON, 1);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int li3m02cm_set_red_led(struct bmi_device *bdev, int on) 
{
	int slotnum = bdev->slot->slotnum;
	bmi_slot_gpio_direction_out(slotnum, GPIO_REDLED,  on ? 1 : 0);
	return 0;
}

static int li3m02cm_set_green_led(struct bmi_device *bdev, int on) 
{
	int ret;
	unsigned char iox_data;
	struct bmi_li3m02cm *cam = bmi_device_get_drvdata(bdev);
	ret = ReadByte_IOX (cam->iox, IOX_OUTPUT_REG, &iox_data);
	if(ret < 0)
		return ret;
	if(on)
		iox_data |=  IOX_GREEN_LED;
	else
		iox_data &= ~IOX_GREEN_LED;
	return WriteByte_IOX (cam->iox, IOX_OUTPUT_REG, iox_data);
}

// We don't have any custom ioctl's for this board
#define li3m02cm_ioctl   NULL
#define li3m02cm_suspend NULL
#define li3m02cm_resume  NULL

static const struct v4l2_subdev_video_ops li3m02cm_video_ops = {
	.g_frame_interval    = li3m02cm_get_frame_interval,
	.s_frame_interval    = li3m02cm_set_frame_interval,
};

static const struct v4l2_subdev_core_ops li3m02cm_core_ops = {
	.queryctrl    = li3m02cm_query_ctrl,
	.querymenu    = li3m02cm_query_menu,
	.g_ctrl       = li3m02cm_get_ctrl,
	.s_ctrl       = li3m02cm_set_ctrl,
	.s_power      = li3m02cm_set_power,
	.ioctl        = NULL, // no custom ioctl's
	.log_status   = li3m02cm_log_status,
};

static const struct v4l2_subdev_pad_ops li3m02cm_pad_ops = {
	.enum_mbus_code      = li3m02cm_enum_mbus_code,
        .enum_frame_size     = li3m02cm_enum_frame_size,
        .enum_frame_interval = li3m02cm_enum_frame_ival,
	.get_fmt             = li3m02cm_get_pad_format,
	.set_fmt             = li3m02cm_set_pad_format,
};

static struct v4l2_subdev_ops li3m02cm_ops = {
	.core  = &li3m02cm_core_ops,
	.video = &li3m02cm_video_ops,
	.pad   = &li3m02cm_pad_ops,
};

static struct bmi_camera_ops li3m02cm_bmi_ops = {
	.set_flash_strobe = li3m02cm_set_flash_strobe,
	.set_red_led      = li3m02cm_set_red_led,
	.set_green_led    = li3m02cm_set_green_led,
	.ioctl            = li3m02cm_ioctl,
	.suspend          = li3m02cm_suspend,
	.resume           = li3m02cm_resume,
};

int bmi_li3m02cm_probe(struct bmi_device *bdev)
{	
	struct bmi_li3m02cm *cam;
	int ret=0;

	cam = kzalloc(sizeof(*cam), GFP_KERNEL);
	if (!cam)
	     return -1;
	
	bmi_device_set_drvdata(bdev, cam);
	cam->bdev = bdev;
	cam->iox     = i2c_new_device(bdev->slot->adap, &iox_info);
	cam->mt9t111 = i2c_new_device(bdev->slot->adap, &mt9t111_info);

	configure_GPIO(cam); 	// configure GPIO and IOX
	configure_IOX(cam);
	
        //register this as a bug_camera
	bmi_register_camera(bdev, &li3m02cm_ops, &li3m02cm_bmi_ops,THIS_MODULE);
	
	// These can be removed after driver stabalizes. They are for debug now.
	ret = device_create_file(&bdev->dev, &dev_attr_iox_value);
	ret = device_create_file(&bdev->dev, &dev_attr_iox_addr);
	ret = device_create_file(&bdev->dev, &dev_attr_mt9t111_value);
	ret = device_create_file(&bdev->dev, &dev_attr_mt9t111_addr);
	ret = device_create_file(&bdev->dev, &dev_attr_set_power);
	ret = device_create_file(&bdev->dev, &dev_attr_format);
	ret = device_create_file(&bdev->dev, &dev_attr_context);
	return ret;
}

void bmi_li3m02cm_remove(struct bmi_device *bdev)
{	
	struct bmi_li3m02cm *cam = bmi_device_get_drvdata (bdev);
	bmi_unregister_camera(bdev);
	i2c_unregister_device(cam->iox);
	i2c_unregister_device(cam->mt9t111);
	kfree (cam);
	return;
}

static __init int bmi_li3m02cm_init(void)
{	
//	Register with BMI bus.
	return  bmi_register_driver(&bmi_li3m02cm_driver); 
}

static void __exit bmi_li3m02cm_cleanup(void)
{	
//	UnRegister with BMI bus.
	bmi_unregister_driver(&bmi_li3m02cm_driver);
	return;
}


module_init(bmi_li3m02cm_init);
module_exit(bmi_li3m02cm_cleanup);

MODULE_AUTHOR("Lane Brooks");
MODULE_DESCRIPTION("LI3M02CM Camera Driver");
MODULE_LICENSE("GPL");
