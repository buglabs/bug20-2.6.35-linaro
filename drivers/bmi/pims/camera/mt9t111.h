/*
 * drivers/bmi/pims/camera/mt9t111.h
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

#ifndef MT9T111_H
#define MT9T111_H

#include <linux/i2c.h>

#define MT9T111_NAME		"mt9t111"
#define MT9T111_I2C_ADDR	(0x3D)
#define MT9T111_CHIP_ID 	(0x0000)
#define MT9T111_CHIP_ID_VALUE	(0x2680)
struct v4l2_mbus_framefmt;

extern int mt9t111_write_reg(struct i2c_client *client, u16 reg, u16 val);
extern int mt9t111_read_reg(struct i2c_client *client, u16 reg, u16 *val);
extern int mt9t111_set_power(struct i2c_client *client, int on);
extern int mt9t111_s_stream(struct i2c_client *client, int streaming);
extern int mt9t111_set_format(struct i2c_client *client, struct v4l2_mbus_framefmt *fmt);
extern int mt9t111_get_format(struct i2c_client *client, struct v4l2_mbus_framefmt *fmt);
extern int mt9t111_query_ctrl(struct i2c_client *client, struct v4l2_queryctrl *a);
extern int mt9t111_query_menu(struct i2c_client *client, struct v4l2_querymenu *qm);
extern int mt9t111_get_ctrl(  struct i2c_client *client, struct v4l2_control *vc);
extern int mt9t111_set_ctrl(  struct i2c_client *client, struct v4l2_control *vc);
extern int mt9t111_log_status(struct i2c_client *client);
extern int mt9t111_enum_mbus_code(struct i2c_client *client,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_pad_mbus_code_enum *code);
extern int mt9t111_enum_frame_size(struct i2c_client *client,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_frame_size_enum *fse);
extern int mt9t111_enum_frame_ival(struct i2c_client *client,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_frame_interval_enum *fie);
extern int mt9t111_get_frame_interval(struct i2c_client *client,
				      struct v4l2_subdev_frame_interval *fi);
extern int mt9t111_set_frame_interval(struct i2c_client *client,
				      struct v4l2_subdev_frame_interval *fi);

	
struct mt9t111_regs {
	u16 delay_time;
	u16 addr;
	u16 data;
	u16 mask; // If 0x0 or 0xFFFF, then just write occurs. Otherwise, read occurs first and mask is applied prior to write
};


#endif /* MT9T111_H */
