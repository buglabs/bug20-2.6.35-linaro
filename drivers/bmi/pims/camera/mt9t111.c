/*
 * drivers/bmi/pims/camera/mt9t111.c
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

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "mt9t111.h"
#include "mt9t111_reg.h"

struct mt9t111_sensor {
	struct i2c_client *client;
	struct v4l2_mbus_framefmt format;
	struct v4l2_fract frame_interval;
	u8 test_pat_id;
	u8 colorfx_id;
	u8 streaming;
	u8 reset;
	
};

int mt9t111_read_reg(struct i2c_client *client, u16 reg, u16 *val)
{
	int ret;
	u8 data[2];
	mdelay(5);
	data[0] = (reg >> 8) & 0x00FF;
	data[1] = (reg >> 0) & 0x00FF;
	ret = i2c_master_send(client, data, 2); // send register addr first
	if (ret >= 0)
		ret = i2c_master_recv(client, data, 2); // read register value
	if (ret < 0) {
		printk(KERN_ERR "%s: i2c_transfer() failed...%d\n", __func__, ret);
	} else {
		*val = ((data[0] & 0x00ff) << 8) | (data[1] & 0x00ff);	
		//printk(KERN_ERR "//%s: Read register 0x%x. value = 0x%x\n", __func__, reg, *val);
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_read_reg);

/**
 * mt9t111_write_reg - Write a value to a register in an mt9t111 sensor device
 * @client: i2c driver client structure
 * @data_length: length of data to be read
 * @reg: register address / offset
 * @val: value to be written to specified register
 *
 * Write a value to a register in an mt9t111 sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
int mt9t111_write_reg(struct i2c_client *client, u16 reg, u16 val)
{
	struct i2c_msg msg[1];
	u8 data[4];
	int err;

	mdelay(5);
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 4;
	msg->buf = data;
	data[0] = (u8)((reg & 0xff00) >> 8);
	data[1] = (u8)(reg & 0x00ff);	
	data[2] = (u8)((val & 0xff00) >> 8);
	data[3] = (u8)(val & 0x00ff);
	err = i2c_transfer(client->adapter, msg, 1);
	if(err < 0)
		printk(KERN_ERR "%s error writing to addr=0x%x data=0x%x err=%d\n", __func__, reg, val, err);
	//else
		//printk(KERN_DEBUG "%s succeed writing to addr=0x%x data=0x%x err=%d\n", __func__, reg, val, err);


	return 0;
}
EXPORT_SYMBOL(mt9t111_write_reg);


/**
 * mt9t111_write_regs - Write registers to an mt9t111 sensor device
 * @client: i2c driver client structure
 * @reg_in: pointer to registers to write
 * @cnt: the number of registers 
 *
 * Write registers .
 * Returns zero if successful, or non-zero otherwise.
 */
static int
mt9t111_write_regs(struct i2c_client *client, struct mt9t111_regs *r, int cnt)
{
	int err = 0, i;
	u16 val;
	struct mt9t111_regs *reg = r;
	
	for (i=0;i<cnt;i++) {
		if(reg->mask != 0x0000 && reg->mask != 0xFFFF) { 
			// in this case, the user is applying a write mask,
			// so we first read the register in question to 
			// generate the correct data to write based on the mask.
			err = mt9t111_read_reg(client, reg->addr, &val);
			if (err < 0)
				return err;
			val = (val & ~reg->mask) | (reg->data & reg->mask);
		} else {
			val = reg->data;
		}
			
		if (reg->delay_time == 0) {
			err = mt9t111_write_reg(client, reg->addr, val);
		} else if (reg->addr != 0 || reg->data != 0) {
			err = mt9t111_write_reg(client, reg->addr, val);
			mdelay(reg->delay_time);
		} else 
			mdelay(reg->delay_time);
			
		if (err < 0)
			return err;
		reg++;
	}
	return err;
}

/* The mt9t111_write_var() and mt9t111_read_var() functions are
 * convenience methods for writing to the extended variable registers
 * on the mt9t111. */
static int mt9t111_write_var(struct i2c_client *client, u16 addr, u16 val) 
{
	int ret;
	ret = mt9t111_write_reg(client, 0x098E, addr);
	if(ret < 0)
		return ret;
	return mt9t111_write_reg(client, 0x0990, val);
}

static int mt9t111_read_var(struct i2c_client *client, u16 addr, u16 *val)
{
	int ret;
	ret = mt9t111_write_reg(client, 0x098E, addr);
	if(ret < 0)
		return ret;
	return mt9t111_read_reg(client, 0x0990, val);
}

int mt9t111_write_bits(struct i2c_client *client, u16 reg, u16 val, u16 mask)
{
	u16 tmp;
	int err = mt9t111_read_reg(client, reg, &tmp);
	if (err < 0)
		return err;
	tmp = (tmp & ~mask) | (val & mask);
	return mt9t111_write_reg(client, reg, tmp);
}

int mt9t111_write_var_bits(struct i2c_client *client, u16 reg, u16 val,u16 mask)
{
	u16 tmp;
	int err = mt9t111_read_var(client, reg, &tmp);
	if (err < 0)
		return err;
	tmp = (tmp & ~mask) | (val & mask);
	return mt9t111_write_var(client, reg, tmp);
}

static int mt9t111_detect(struct i2c_client *client) 
{
	u16 val;
	/* chip ID is at address 0 */
	if (mt9t111_read_reg(client, MT9T111_CHIP_ID, &val) < 0) {
		printk(KERN_ERR "%s: Error reading MT9T111 chip id.\n", __func__);
		return -ENODEV;
	}
	if (val != MT9T111_CHIP_ID_VALUE) {
		printk(KERN_ERR "%s: Chip ID mismatch received 0x%x expecting 0x%x\n",
		       __func__, val, MT9T111_CHIP_ID_VALUE);
		return -ENODEV;
	}
	printk(KERN_INFO "%s: Read MT9T111 CHIP ID = 0x%x", __func__, val);
	return 0;
}

#define MT9T111_APPLY_PATCH(client, x) mt9t111_write_regs(client, x, sizeof(x)/sizeof(x[0]));

static int mt9t111_refresh(struct i2c_client *client)
{
	int err;
	err = mt9t111_write_var(client, 0x8400, 0x0006); // Refresh Seq. Mode
	if(err < 0)
		return err;
	mdelay(500); // found a delay necessary, not sure why, though
//	return mt9t111_write_var(client, 0x8400, 0x0005); // Refresh Seq.
	return 0;
}

static int mt9t111_enable_pll(struct i2c_client *client)
{
	int i, err;
	unsigned short value; 

	err = MT9T111_APPLY_PATCH(client, pll_regs1);
	if(err < 0) {
		printk(KERN_ERR "%s error applying pll_regs1 patch (err=%d)\n", __func__, err);
		return err;
	}
	i=0;
	while(1) { // wait for MT9T111 to report that PLL is locked
		err = mt9t111_read_reg(client,0x0014,&value);
		if(err < 0) {
			printk(KERN_ERR "%s: error reading pll lock state\n", __func__);
			return err;
		}
		if (( value & 0x8000) != 0)
			break;
		if(i++ > 100) {
			printk(KERN_ERR "%s: can't get pll lock\n", __func__);
			return -EBUSY;
		}
		mdelay(2);
	}
	err = MT9T111_APPLY_PATCH(client, pll_regs2);
	if(err < 0) {
		printk(KERN_ERR "%s: error applying pll_regs2 patch (err=%d)\n", __func__, err);
		return err;
	}
	return 0;
}

#if 0
static int mt9t111_sw_reset(struct i2c_client *client)
{
	int err;
	u16 value;
	err = mt9t111_read_reg(client, 0x001A, &value);
	if(err < 0)
		return err;
	err = mt9t111_write_reg(client, 0x001A, value | 0x1);
	if(err < 0)
		return err;
	err = mt9t111_write_reg(client, 0x001A, value & ~0x1);
	if(err < 0)
		return err;
	return 0;
}

static int mt9t111_soft_standby(struct i2c_client *client, int on)
{
	int err,i;
	unsigned short value;

	err = mt9t111_read_reg(client, 0x0018, &value);
	if(on) 
		value |= 0x1;
	else
		value &= ~0x1;

	err = mt9t111_write_reg(client, 0x0018, value);
	
	// now wait until the standby_done state indicator switches
	i=0;
	while(1) {
		err = mt9t111_read_reg(client, 0x0018, &value);
		if(err < 0)
			return err;
		if ( (value & 0x4000) != (on) ? 0x4000 : 0x0000 ) 
			break;
		if(i++ > 100) {
			return -EBUSY;
		}
		mdelay(2);
	}
	return 0;
}
#endif

static int mt9t111_loaddefault(struct i2c_client *client)
{
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	int err;
	sensor->test_pat_id = 0;
	sensor->colorfx_id = 0;

	if(1) {
		err = mt9t111_enable_pll(client);
	} else {
		err = MT9T111_APPLY_PATCH(client, bypass_pll);
	}
	if(err < 0)
		return err;

	err = MT9T111_APPLY_PATCH(client, mt9t111_init_regs);
	if(err < 0)
		return err;

	sensor->reset = 1;
	//err = MT9T111_APPLY_PATCH(client, def_regs1);
	//if(err < 0)
	//	return err;
	//
	//err = MT9T111_APPLY_PATCH(client, patch_rev6);
	//if(err < 0)
	//	return err;
	//
	//err = MT9T111_APPLY_PATCH(client, def_regs2);
	//if(err < 0)
	//	return err;
	//
	//return mt9t111_refresh(client);
	//sensor->format.width = sensor->format.height = 1;
	return 0;
}

int mt9t111_set_power(struct i2c_client *client, int on)
{
	int ret = 0;
	if(on) {
		ret = mt9t111_detect(client);
		if(ret < 0)
			return ret;
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_set_power);

int mt9t111_s_stream(struct i2c_client *client, int streaming)
{
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	int ret;
	if(streaming) {
		ret = mt9t111_loaddefault(client);
		if(ret < 0)
			return ret;
		ret = mt9t111_set_format(client, &sensor->format);
		if(ret < 0)
			return ret;
		sensor->streaming = 1;
	} else {
		sensor->streaming = 0;
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_s_stream);


struct mt9t111_format {
	struct v4l2_frmsize_discrete min_res;
	struct v4l2_frmsize_discrete max_res;
	struct v4l2_fract interval;
};


struct mt9t111_format mt9t111_fmt[] = {
	{ {1040, 784}, {2048, 1536}, { 20, 69  } },
	{ {680,  514}, {1024,  768}, { 1,  10  } },
	{ { 32,   32}, { 664,  498}, { 1,  14  } },
};

u32 mt9t111_mbus_codes[] = {
	V4L2_MBUS_FMT_YUYV8_2X8_LE,
	V4L2_MBUS_FMT_YVYU8_2X8_LE,
	V4L2_MBUS_FMT_YUYV8_2X8_BE,
	V4L2_MBUS_FMT_YVYU8_2X8_BE,
	V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE,
	V4L2_MBUS_FMT_RGB555_2X8_PADHI_BE,
	V4L2_MBUS_FMT_RGB565_2X8_LE,
	V4L2_MBUS_FMT_RGB565_2X8_BE,
	V4L2_MBUS_FMT_SBGGR8_1X8,
	V4L2_MBUS_FMT_SBGGR10_1X10,
	V4L2_MBUS_FMT_GREY8_1X8,
	V4L2_MBUS_FMT_Y10_1X10,
	//V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_LE,
	//V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_LE,
	//V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_BE,
	//V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_BE,
	V4L2_MBUS_FMT_SGRBG8_1X8,
	V4L2_MBUS_FMT_SGRBG10_1X10,
	//V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8,   // 10b companded to 8b
	V4L2_MBUS_FMT_YUYV16_1X16,
	V4L2_MBUS_FMT_UYVY16_1X16,
	V4L2_MBUS_FMT_YVYU16_1X16,
	V4L2_MBUS_FMT_VYUY16_1X16,
	V4L2_MBUS_FMT_JPEG8,
};	

int mt9t111_set_format(struct i2c_client *client, struct v4l2_mbus_framefmt *fmt)
{
	int ret;
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);

	if(sensor->format.width  == fmt->width  &&
	   sensor->format.height == fmt->height &&
	   sensor->format.code   == fmt->code && 
	   !sensor->reset) {
		printk(KERN_INFO "%s: Sensor already in requested format (%dx%d).\n", __func__, fmt->width, fmt->height);
		return 0;
	}
	sensor->reset = 0;

	// clamp width and height to multiples of 16
	fmt->width  = (fmt->width  / 16) * 16;
	fmt->height = (fmt->height / 16) * 16;
	if(fmt->width  < 32) fmt->width  = 32;
	if(fmt->height < 32) fmt->height = 32;

	if(fmt->height > 768 || fmt->width > 1024) {
		printk(KERN_INFO "%s applying 2048x1536 patch\n", __func__);
		ret = MT9T111_APPLY_PATCH(client, fmt_2048x1536_5fps);

		mt9t111_write_var(client, 0x6800, fmt->width);
		mt9t111_write_var(client, 0x6802, fmt->height);
		mt9t111_write_var(client, 0x4802, (1536-fmt->height)/2+8);
		mt9t111_write_var(client, 0x4804, (2048-fmt->width )/2+8);
		mt9t111_write_var(client, 0x4806, (1536-fmt->height)/2+8+fmt->height-1+8);
		mt9t111_write_var(client, 0x4808, (2048-fmt->width )/2+8+fmt->width-1+8);
		mt9t111_write_var(client, 0x482b, fmt->width+8);
		mt9t111_write_var(client, 0x482d, fmt->height+8);
		sensor->frame_interval.denominator = mt9t111_fmt[0].interval.denominator;
		sensor->frame_interval.numerator   = mt9t111_fmt[0].interval.numerator;
	} else if(fmt->height > 498 || fmt->width > 664) {
		printk(KERN_INFO "%s applying 1024x768 patch\n", __func__);
		ret = MT9T111_APPLY_PATCH(client, fmt_1024x768_20fps);

		mt9t111_write_var(client, 0x6800, fmt->width);
		mt9t111_write_var(client, 0x6802, fmt->height);
		mt9t111_write_var(client, 0x4802, 768-fmt->height);
		mt9t111_write_var(client, 0x4804, 1024-fmt->width);
		mt9t111_write_var(client, 0x4806, 768-fmt->height+(fmt->height+8)*2-3);
		mt9t111_write_var(client, 0x4808, 1024-fmt->width+(fmt->width+8)*2-3);
		mt9t111_write_var(client, 0x482b, fmt->width+8);
		mt9t111_write_var(client, 0x482d, fmt->height+8);
		sensor->frame_interval.denominator = mt9t111_fmt[1].interval.denominator;
		sensor->frame_interval.numerator   = mt9t111_fmt[1].interval.numerator;
	} else {
		printk(KERN_INFO "%s applying 640x480 patch\n", __func__);
		ret = MT9T111_APPLY_PATCH(client, fmt_640x480_22fps);
		mt9t111_write_var(client, 0x6800, fmt->width);
		mt9t111_write_var(client, 0x6802, fmt->height);
		sensor->frame_interval.denominator = mt9t111_fmt[2].interval.denominator;
		sensor->frame_interval.numerator   = mt9t111_fmt[2].interval.numerator;
	}


	switch(fmt->code) {
	case V4L2_MBUS_FMT_GREY8_1X8:
	case V4L2_MBUS_FMT_Y10_1X10:
		// monochrome modes are the same as bayer mode except for
		// the pri_a_output_format (0x6807) register
		printk(KERN_INFO "%s applying MONOCHROME mode\n", __func__);
		ret = MT9T111_APPLY_PATCH(client, fmt_GBRG_regs);
		if(ret < 0)
			return ret;
		mt9t111_write_var(client, 0x6807, 0x200); // monochrome mode
		break;
	case V4L2_MBUS_FMT_JPEG8:
		printk(KERN_INFO "%s applying JPEG mode\n", __func__);
		ret = MT9T111_APPLY_PATCH(client, fmt_JPEG_regs);
		if(ret < 0)
			return ret;
		break;
	case V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE:
	case V4L2_MBUS_FMT_RGB555_2X8_PADHI_BE:
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
	case V4L2_MBUS_FMT_RGB565_2X8_BE:
		// same as YUV mode except for register (0x6807)
		printk(KERN_INFO "%s applying RGB555/565 mode\n", __func__);
		ret = MT9T111_APPLY_PATCH(client, fmt_YCrCb_regs);
		if(ret < 0)
			return ret;
		if(fmt->code == V4L2_MBUS_FMT_RGB565_2X8_BE ||
		   fmt->code == V4L2_MBUS_FMT_RGB555_2X8_PADHI_BE)
			// byte swap
			mt9t111_write_var(client, 0x6809, 0x2);
		if(fmt->code == V4L2_MBUS_FMT_RGB565_2X8_BE ||
		   fmt->code == V4L2_MBUS_FMT_RGB565_2X8_LE)
			mt9t111_write_var(client, 0x6807, 0x8);
		else
			mt9t111_write_var(client, 0x6807, 0x4);

	case V4L2_MBUS_FMT_YUYV8_2X8_LE:
	case V4L2_MBUS_FMT_YVYU8_2X8_LE:
	case V4L2_MBUS_FMT_YUYV8_2X8_BE:
	case V4L2_MBUS_FMT_YVYU8_2X8_BE:
	case V4L2_MBUS_FMT_YUYV16_1X16:
	case V4L2_MBUS_FMT_UYVY16_1X16:
	case V4L2_MBUS_FMT_YVYU16_1X16:
	case V4L2_MBUS_FMT_VYUY16_1X16:
		printk(KERN_INFO "%s applying YUV mode\n", __func__);
		ret = MT9T111_APPLY_PATCH(client, fmt_YCrCb_regs);
		if(ret < 0)
			return ret;
		if(fmt->code == V4L2_MBUS_FMT_YVYU16_1X16  ||
		   fmt->code == V4L2_MBUS_FMT_VYUY16_1X16  ||
		   fmt->code == V4L2_MBUS_FMT_YVYU8_2X8_LE ||
		   fmt->code == V4L2_MBUS_FMT_YVYU8_2X8_BE)
			// swap Cr/Cb
			mt9t111_write_var_bits(client, 0x6809, 0x1, 0x1);
		if(fmt->code == V4L2_MBUS_FMT_VYUY16_1X16 ||
		   fmt->code == V4L2_MBUS_FMT_UYVY16_1X16 ||
		   fmt->code ==  V4L2_MBUS_FMT_YUYV8_2X8_BE ||
		   fmt->code ==  V4L2_MBUS_FMT_YVYU8_2X8_BE)
			// byte swap
			mt9t111_write_var_bits(client, 0x6809, 0x2, 0x2);
		break;
	case V4L2_MBUS_FMT_SGRBG8_1X8:
	case V4L2_MBUS_FMT_SGRBG10_1X10:
	case V4L2_MBUS_FMT_SBGGR8_1X8:
	case V4L2_MBUS_FMT_SBGGR10_1X10:
		printk(KERN_INFO "%s applying GBRG mode\n", __func__);
		ret = MT9T111_APPLY_PATCH(client, fmt_GBRG_regs);
		if(ret < 0)
			return ret;
		if(fmt->code == V4L2_MBUS_FMT_SBGGR8_1X8 ||
		   fmt->code == V4L2_MBUS_FMT_SBGGR10_1X10)
			// set first pixel as B
			mt9t111_write_var(client, 0x6809, 0x80);
		break;
	default:
		printk(KERN_INFO "%s applying GBRG mode\n", __func__);
		ret = MT9T111_APPLY_PATCH(client, fmt_GBRG_regs);
		if(ret < 0)
			return ret;
		fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;
		break;
	}

	//mt9t111_write_var(client, 0x689B, fmt->width*2);
	//mt9t111_write_var(client, 0x689D, fmt->height+10);

	memcpy(&(sensor->format), fmt, sizeof(*fmt));
	return mt9t111_refresh(client);
}
EXPORT_SYMBOL(mt9t111_set_format);


int mt9t111_get_format(struct i2c_client *client, struct v4l2_mbus_framefmt *fmt)
{
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	memcpy(fmt, &sensor->format, sizeof(sensor->format));
	return 0;
}
EXPORT_SYMBOL(mt9t111_get_format);


int mt9t111_enum_mbus_code(struct i2c_client *client,
			   struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_pad_mbus_code_enum *code)
{
	if(code->pad != 0 || code->index >= ARRAY_SIZE(mt9t111_mbus_codes))
		return -EINVAL;

	code->code = mt9t111_mbus_codes[code->index];
	return 0;
}
EXPORT_SYMBOL(mt9t111_enum_mbus_code);

int mt9t111_enum_frame_size(struct i2c_client *client,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_frame_size_enum *fse)

{
	int i;
	if(fse->index >= ARRAY_SIZE(mt9t111_fmt))
		return -EINVAL;

	for(i=0; i<ARRAY_SIZE(mt9t111_mbus_codes); i++) {
		if(mt9t111_mbus_codes[i] == fse->code) {
			fse->min_width = mt9t111_fmt[fse->index].min_res.width;
			fse->max_width = mt9t111_fmt[fse->index].max_res.width;
			fse->min_height= mt9t111_fmt[fse->index].min_res.height;
			fse->max_height= mt9t111_fmt[fse->index].max_res.height;
			return 0;
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL(mt9t111_enum_frame_size);


int mt9t111_enum_frame_ival(struct i2c_client *client,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_frame_interval_enum *fie)
{
	int i;
	if(fie->index >= ARRAY_SIZE(mt9t111_fmt))
		return -EINVAL;

	for(i=0; i<ARRAY_SIZE(mt9t111_mbus_codes); i++) {
		if(mt9t111_mbus_codes[i] == fie->code) {
			fie->interval.numerator   = mt9t111_fmt[fie->index].interval.numerator;
			fie->interval.denominator = mt9t111_fmt[fie->index].interval.denominator;
			return 0;
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL(mt9t111_enum_frame_ival);

int mt9t111_get_frame_interval(struct i2c_client *client,
			       struct v4l2_subdev_frame_interval *fi)
{
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	memcpy(&fi->interval, &sensor->frame_interval, sizeof(sensor->frame_interval));
	return 0;
}
EXPORT_SYMBOL(mt9t111_get_frame_interval);

int mt9t111_set_frame_interval(struct i2c_client *client,
			       struct v4l2_subdev_frame_interval *fi)
{
	// It is possible to change the frame interval, but for now
	// we just leave it static based on the current resolution.
	// The change would require calculating the new number of vblank
        // rows and applying it. This would keep the flicker settings
	// static as well, which is good.
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	memcpy(&fi->interval, &sensor->frame_interval, sizeof(sensor->frame_interval));
	return 0;
}
EXPORT_SYMBOL(mt9t111_set_frame_interval);


static char *test_pats[] = {
	"Disabled",         
	"Walking 1's",      
	"Solid White",      
	"Grey Ramp",        
	"Color Bars",       
	"Black/White Bars", 
	"Pseudo Random",    
};
static char *fx[] = {
	"Disabled",         
	"Black & White",
	"Sepia",
	"Negative",
	"Solarize",
};

static int mt9t111_set_test_pattern(struct i2c_client *client, int id) {
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	int err = 0;
	sensor->test_pat_id = id;
	if(id == 0) { // disable test pattern
		printk(KERN_INFO "%s Disabling Test Pattern\n", __func__);
		err |= mt9t111_write_var(client, 0xE025, 0x0000); //select pat 0
		err |= mt9t111_write_var(client, 0x6003, 0x0000); //disable patt
                //enable lens correction, gamma, etc.
		err |= mt9t111_write_reg(client, 0x3210, 0x01B8); 

		// disable 8bit walking 1's test pattern
		err |= mt9t111_write_bits(client, 0x3C20, 0x00, 0x0030);

	} else if(id == 1) { // walking 1's test pattern

		printk(KERN_INFO "%s Enabling Walking 1's Test Pattern\n", __func__);
		err |= mt9t111_write_var(client, 0xE025, 0x0000); //select pat 0
		err |= mt9t111_write_var(client, 0x6003, 0x0000); //disable patt

		// enable 8bit walking 1's test pattern
		err |= mt9t111_write_bits(client, 0x3C20, 0x20, 0x0032);
		
		// Note the user must be in Bayer mode for this to work.

	} else { // all other test patterns
		int code;
		switch(id) {
		case 2: code = 1; break; // solid white
		case 3: code = 4; break; // gray ramp
		case 4: code = 6; break; // color bars
		case 5: code = 8; break; // black/white bars
		case 6: code = 10; break; // random
		default: code = 6; break;
		}
		printk(KERN_INFO "%s Enabling Test Pattern %d\n", __func__, code);
		// disable 8bit walking 1's test pattern
		err |= mt9t111_write_bits(client, 0x3C20, 0x00, 0x0030);

		err |= mt9t111_write_var(client, 0x6003, 0x100); //enable patt.
		err |= mt9t111_write_var(client, 0xE025, code);  //select patt.
		{
			u16 val;
			mt9t111_read_var(client, 0xE025, &val);
			printk(KERN_INFO "%s test pat code=0x%x\n", __func__, val);
		}
		// disable lens correction, gamma, etc
		err |= mt9t111_write_reg(client, 0x3210, 0x0000);
	}
	mt9t111_refresh(client);
	return err;
}

static int mt9t111_set_colorfx(struct i2c_client *client, int fx) {
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	switch (fx) {
	case V4L2_COLORFX_NONE:
		mt9t111_write_var(client, 0xE883, 0x0000);
		mt9t111_write_var(client, 0xEC83, 0x0000);
		break;
	case V4L2_COLORFX_BW:
		mt9t111_write_var(client, 0xE883, 0x0001);
		//mt9t111_write_var(client, 0xEC83, 0x0001);
		break;
	case V4L2_COLORFX_SEPIA:
		mt9t111_write_var(client, 0xE883, 0x0002);
		mt9t111_write_var(client, 0xEC83, 0x0002);
		break;
 	case V4L2_COLORFX_NEGATIVE:
		mt9t111_write_var(client, 0xE883, 0x0003);
		mt9t111_write_var(client, 0xEC83, 0x0003);
		break;
	case 4:
		//[Special Effect – Solarize w/ Strength Control]
		mt9t111_write_var(client, 0xE883, 0x0004);
		//mt9t111_write_var(client, 0xE884, 0x08);// SOLARIZATION_TH
		mt9t111_write_var(client, 0xEC83, 0x0004);
		//mt9t111_write_var(client, 0xEC84, 0x08);// SOLARIZATION_TH
		break;
	default:
		return -EINVAL;
	}
	sensor->colorfx_id = fx;
	return mt9t111_refresh(client);
}

int mt9t111_query_ctrl(struct i2c_client *client, struct v4l2_queryctrl *a)
{
	switch (a->id) {
	case V4L2_CID_TEST_PATTERN:
		a->type = V4L2_CTRL_TYPE_MENU;
		sprintf(a->name, "Test Pattern");
		a->minimum = 0;
		a->maximum = ARRAY_SIZE(test_pats)-1;
		a->default_value = 0;
		a->flags = 0;
		break;
	case V4L2_CID_COLORFX:
		a->type = V4L2_CTRL_TYPE_MENU;
		sprintf(a->name, "Color Effects");
		a->minimum = 0;
		a->maximum = 4;
		a->default_value = 0;
		a->flags = 0;
		break;
	case V4L2_CID_HFLIP:
		a->type = V4L2_CTRL_TYPE_BOOLEAN;
		sprintf(a->name, "Horizontal Mirror");
		a->default_value = 0;
		a->flags = 0;
		break;
	case V4L2_CID_VFLIP:
		a->type = V4L2_CTRL_TYPE_BOOLEAN;
		sprintf(a->name, "Vertical Flip");
		a->default_value = 0;
		a->flags = 0;
		break;
	case V4L2_CID_EXPOSURE:
		a->type = V4L2_CTRL_TYPE_INTEGER;
		sprintf(a->name, "Exposure Level");
		a->minimum = 0;
		a->maximum = 0xFF;
		a->default_value = 0x37;
		a->flags = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_query_ctrl);

int mt9t111_query_menu(struct i2c_client *client, struct v4l2_querymenu *qm)
{
	switch (qm->id) {
	case V4L2_CID_TEST_PATTERN:
		if(qm->index < ARRAY_SIZE(test_pats)) {
			strcpy(qm->name, test_pats[qm->index]);
		} else {
			return -EINVAL;
		}
		break;
	case V4L2_CID_COLORFX:
		if(qm->index < 10) {
			strcpy(qm->name, fx[qm->index]);
		} else {
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_query_menu);

int mt9t111_get_ctrl(struct i2c_client *client, struct v4l2_control *vc)
{
	u16 val;
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	switch (vc->id) {
	case V4L2_CID_TEST_PATTERN:
		vc->value = sensor->test_pat_id;
		break;
	case V4L2_CID_COLORFX:
		vc->value = sensor->colorfx_id;
		break;
	case V4L2_CID_HFLIP:
		mt9t111_read_var(client, 0x480C, &val);
		vc->value = val & 0x1;
		break;
	case V4L2_CID_VFLIP:
		mt9t111_read_var(client, 0x480C, &val);
		vc->value = (val >> 1) & 0x1;
		break;
	case V4L2_CID_EXPOSURE:
		mt9t111_read_var(client, 0xE81F, &val);
		vc->value = val;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_get_ctrl);

int mt9t111_set_ctrl(struct i2c_client *client, struct v4l2_control *vc)
{
	switch (vc->id) {
	case V4L2_CID_TEST_PATTERN:
		return mt9t111_set_test_pattern(client, vc->value);
	case V4L2_CID_COLORFX:
		return mt9t111_set_colorfx(client, vc->value);
	case V4L2_CID_HFLIP:
		mt9t111_write_var_bits(client, 0x480C, vc->value ? 1 : 0, 0x1);
		mt9t111_refresh(client);
		break;
	case V4L2_CID_VFLIP:
		mt9t111_write_var_bits(client, 0x480C, vc->value ? 2 : 0, 0x2);
		mt9t111_refresh(client);
		break;
	case V4L2_CID_EXPOSURE:
		mt9t111_write_var(client, 0xE81F, vc->value);
		mt9t111_refresh(client);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_set_ctrl);

static void log_reg(struct i2c_client *client, char *name, int type, u16 addr) {
	int err;
	u16 val;
	if(type == 0) {
		err = mt9t111_read_reg(client, addr, &val);
	} else {
		err = mt9t111_read_var(client, addr, &val);
	}
	printk(KERN_INFO "mt9t111: Addr=0%04x  Value=0x%04x  %s\n", addr, val, name);
	if(err < 0)
		printk(KERN_ERR "Error code = %d\n", err);
}

int mt9t111_log_status(struct i2c_client *client) {
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	if(!sensor->streaming) {
		printk(KERN_INFO "mt9t111: Not streaming\n");
	} else {
		log_reg(client, "ae_track_status",   1, 0x2800);
		log_reg(client, "ae_track_mode",     1, 0xA802);
		log_reg(client, "ae_track_algorithm",1, 0x2803);
		log_reg(client, "ae_track_max_black_level",1, 0x2807);
		log_reg(client, "ae_track_target",   1, 0xA80D);
		log_reg(client, "ae_track_gate",     1, 0xA80E);
		log_reg(client, "ae_track_current_average_y", 1, 0xA80F);
		log_reg(client, "ae_track_dampening", 1, 0xA810);
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_log_status);


/**
 * mt9t111_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int mt9t111_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct mt9t111_sensor *sensor;
	sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
	     return -1;

	i2c_set_clientdata(client, sensor);
	sensor->client = client;
	sensor->format.width        = 1024;
	sensor->format.height       = 768;
	sensor->format.code         = V4L2_MBUS_FMT_YUYV16_1X16;
	sensor->format.colorspace   = V4L2_COLORSPACE_SRGB;
	sensor->format.field        = V4L2_FIELD_NONE;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator= 22;
	sensor->test_pat_id = 0;
	sensor->colorfx_id  = 0;
	sensor->streaming   = 0;
	sensor->reset       = 1;
	return 0;
}


/**
 * mt9t111_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of mt9t111_probe().
 */
static int __exit mt9t111_remove(struct i2c_client *client)
{
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	kfree(sensor);
	return 0;
}


static const struct i2c_device_id mt9t111_id_table[] = {
	{ MT9T111_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9t111_id_table);

static struct i2c_driver mt9t111_i2c_driver = {
	.driver		= {
		.name	= MT9T111_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= mt9t111_probe,
	.remove		= __exit_p(mt9t111_remove),
	.suspend	= NULL,
	.resume		= NULL,
	.id_table	= mt9t111_id_table,
};

/**
 * mt9t111sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init mt9t111_init(void)
{
	int rval;

	rval = i2c_add_driver(&mt9t111_i2c_driver);
	if (rval)
		printk(KERN_ERR "%s: failed registering " MT9T111_NAME "\n",
		       __func__);
	return rval;
}


/**
 * mt9t111sensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of mt9t111sensor_init.
 */
static void __exit mt9t111_exit(void)
{
	i2c_del_driver(&mt9t111_i2c_driver);
}

module_init(mt9t111_init);
module_exit(mt9t111_exit);

MODULE_AUTHOR("Lane Brooks <dirjud@gmail.com>");
MODULE_DESCRIPTION("Aptina MT9T111 camera I2C sensor driver");
MODULE_LICENSE("GPL");
