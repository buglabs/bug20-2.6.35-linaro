/*
 * 	ths8200.c
 *
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <plat/gpio.h>
#include <linux/i2c.h>
#include "ths8200.h"

//temp
#include <linux/i2c/twl.h>


/*
 * 	THS8200 init sequence
 */

u8 ths8200_init_seq[][2] = {
    {0x03, 0xC1},     /*chip_ctl*/
    {0x19, 0x03},     /*csc_offset3  CSC bypassed*/
    {0x1D, 0x00},     /*dtg_y_sync1*/
    {0x1E, 0x00},     /*dtg_y_sync2*/
    {0x1F, 0x00},     /*dtg_y_sync3*/
    {0x20, 0x00},     /*dtg_cbcr_sync1*/
    {0x21, 0x00},     /*dtg_cbcr_sync2*/
    {0x22, 0x00},     /*dtg_cbcr_sync3*/
    {0x23, 0x00},     /*dtg_y_sync_upper*/
    {0x24, 0x00},     /*dtg_cbcr_sync_upper*/
    {0x1C, 0x70},     /*dman_cntl   30bit 4:4:4 input*/

    /*
    //   800x600   //
    {0x34, 0x04},     /*dtg_total_pixel_msb 1056 pixels per line
    {0x35, 0x20},     /*dtg_total_pixel_lsb
    {0x39, 0x22},     /*dtg_frame_field_msb 628 line per frame/field
    {0x3A, 0x74},     /*dtg_frame_size_lsb
    */

    /*   1024x768   */
    {0x34, 0x05},     /*dtg_total_pixel_msb 1056 pixels per line*/
    {0x35, 0x40},     /*dtg_total_pixel_lsb*/
    {0x39, 0x30},     /*dtg_frame_field_msb 628 line per frame/field*/
    {0x3A, 0x26},     /*dtg_frame_size_lsb*/

    {0x36, 0x80},     /*dtg_linecnt_msb*/
    {0x37, 0x01},     /*dtg_linecnt_lsb*/
    {0x38, 0x87},     /*dtg_mode      VESA slave*/
    {0x3B, 0x74},     /*dtg_field_size_lsb*/
    {0x4A, 0x8C},     /*csm_mult_gy_msb*/
    {0x4B, 0x44},     /*csm_mult_bcb_rcr_msb*/
    {0x4C, 0x00},     /*csm_mult_gy_lsb*/
    {0x4D, 0x00},     /*csm_mult_bcb_lsb*/
    {0x4E, 0x00},     /*csm_mult_rcr_lsb*/
    {0x4F, 0xC0},     /*csm_modd*/
    {0x70, 0x80},     /*dtg_hlength_lsb   HSOUT, 128 pixels*/
    {0x71, 0x00},     /*dtg_hdly_msb*/
    {0x72, 0x01},     /*dtg_hdly_lsb*/
    {0x73, 0x05},     /*dtg_vlength_lsb  VSOUT=4 lines
					 ( use 1 more than desired width)*/
    {0x74, 0x00},     /*dtg_vdly_msb*/
    {0x75, 0x01},     /*dtg_vdly_lsb*/
    {0x76, 0x00},     /*dtg_vlength2_lsb*/
    {0x77, 0x07},     /*dtg_vdly2_msb*/
    {0x78, 0xFF},     /*dtg_vdly2_lsb*/
    {0x79, 0x00},     /*dtg_hs_in_dly_msb*/
    {0x7A, 0x00},     /*dtg_hs_in_dly_lsb ,to adjust horizontal alignment*/
    {0x7B, 0x00},     /*dtg_vs_in_dly_msb*/
    {0x7C, 0x00},     /*dtg_vs_in_dly_lsb*/
    {0x82, 0xDB},     /*pol_cntl  HSync/VSync polarities,
			 DE enabled in VESA mode, set to 0x5B  to disable DE*/

    // colorspace conversion off
    /*
      {0x04, 0x00}, {0x05, 0x00}, {0x06, 0x00}, {0x07, 0x00}, {0x08, 0x00},
      {0x09, 0x00}, {0x0a, 0x00}, {0x0b, 0x00}, {0x0c, 0x00}, {0x0d, 0x00},
      {0x0e, 0x00}, {0x0f, 0x00}, {0x10, 0x00}, {0x11, 0x00}, {0x12, 0x00},
      {0x13, 0x00}, {0x14, 0x00}, {0x15, 0x00}, {0x16, 0x00}, {0x17, 0x00},
      {0x18, 0x00}, {0x19, 0x00},
    */

    /*color bars
    {0x03, 0xa3},*/
};



u8 ths8200_init_seq_bad[][2] = {
    {0x03, 0x00},     //place ths8200 into reset
    {0x03, 0x01},     //take ths8200 out of reset

    //These seem to have no effect
    {0x1c, 0x38},     //bit 5: enables 4:4:4; bit 4: bypass CSC
    //{0x1c, 0x08},

    {0x38, 0x87},     //bit 7: DTG on; bit 3:0 VESA slave
    {0x36, 0x80},
  
    /*
    //////////  1280 x 1024  //////////
    {0x82, 0xcb},     //VS_OUT = pos; HS_OUT = pos; FID = neg; VS_IN = pos; HS_IN = pos
    {0x72, 0x01},     //0x72(7:0) - HS_OUT delay
    {0x70, 0x20},     //0x70(7:0) - HS_OUT duration (32)
    {0x75, 0x01},     //0x75(7:0) - VS_OUT delay, field 1
    {0x73, 0x07},     //0x73(7:0) - VS_OUT duration, field 1 (7)
    {0x79, 0x00},     //0x79(4:0)(msb) and 0x7a(7:0)(lsb) - DTG horizontal delay
    {0x7a, 0x00},
    {0x7b, 0x00},     //0x7b(2:0)(msb) and 0x7c(7:0)(lsb) - DTG vertical delay
    {0x7c, 0x00},
    {0x34, 0x05},     //0x34(4:0)(msb) and 0x35(7:0)(lsb) - total pixels/line: 1408
    {0x35, 0x80},
    {0x39, 0x40},     //0x39(6:4)(msb) and 0x3a(7:0)(lsb) - total lines/frame: 1045
    {0x3a, 0x15},
    */
  
    //////////  1024 x 768  //////////
    {0x82, 0xd3},     //VS_OUT = neg; HS_OUT = neg; FID = neg; VS_IN = pos; HS_IN = pos;
    {0x72, 0x01},     //0x72(7:0) - HS_OUT delay
    {0x70, 0x88},     //0x70(7:0) - HS_OUT duration (136)
    {0x75, 0x01},     //0x75(7:0) - VS_OUT delay, field 1
    {0x73, 0x06},     //0x73(7:0) - VS_OUT duration, field 1 (6)
    {0x79, 0x00},     //0x79(4:0)(msb) and 0x7a(7:0)(lsb) - DTG horizontal delay
    {0x7a, 0x00},
    {0x7b, 0x00},     //0x7b(2:0)(msb) and 0x7c(7:0)(lsb) - DTG vertical delay
    {0x7c, 0x00},
    {0x34, 0x04},     //0x34(4:0)(msb) and 0x35(7:0)(lsb) - total pixels/line: 1208
    {0x35, 0xb8},
    {0x39, 0x30},     //0x39(6:4)(msb) and 0x3a(7:0)(lsb) - total lines/frame: 800
    {0x3a, 0x20},

    /*
    //////////  800 x 600  //////////
    {0x82, 0xcb},     //VS_OUT = pos; HS_OUT = pos; FID = neg; VS_IN = pos; HS_IN = pos
    {0x72, 0x01},     //0x72(7:0) - HS_OUT delay
    {0x70, 0x80},       //0x70(7:0) - HS_OUT duration (128)
    {0x75, 0x01},     //0x75(7:0) - VS_OUT delay, field 1
    {0x73, 0x04},     //0x73(7:0) - VS_OUT duration, field 1 (4)
    {0x79, 0x00},     //0x79(4:0)(msb) and 0x7a(7:0)(lsb) - DTG horizontal delay
    {0x7a, 0x00},
    {0x7b, 0x00},     //0x7b(2:0)(msb) and 0x7c(7:0)(lsb) - DTG vertical delay
    {0x7c, 0x00},
    {0x34, 0x03},     //0x34(4:0)(msb) and 0x35(7:0)(lsb) - total pixels/line: 928
    {0x35, 0xa0},
    {0x39, 0x20},     //0x39(6:4)(msb) and 0x3a(7:0)(lsb) - total lines/frame: 624
    {0x3a, 0x70},
    */
 
    //second reset?
    {0x03, 0x00},
    {0x03, 0x01},

    /*colorspace conversion off*/
    /*
      {0x04, 0x00}, {0x05, 0x00}, {0x06, 0x00}, {0x07, 0x00}, {0x08, 0x00},
      {0x09, 0x00}, {0x0a, 0x00}, {0x0b, 0x00}, {0x0c, 0x00}, {0x0d, 0x00},
      {0x0e, 0x00}, {0x0f, 0x00}, {0x10, 0x00}, {0x11, 0x00}, {0x12, 0x00},
      {0x13, 0x00}, {0x14, 0x00}, {0x15, 0x00}, {0x16, 0x00}, {0x17, 0x00},
      {0x18, 0x00}, {0x19, 0x00},
    */

    /* color bars
     0x03, 0xa3},*/
};

static inline int ths8200_read(struct i2c_client *client, u8 reg)
{
    return i2c_smbus_read_byte_data(client, reg);
}

static inline int ths8200_write(struct i2c_client *client, u8 reg, u8 value)
{
    return i2c_smbus_write_byte_data(client, reg, value);
}

int ths8200_enable(struct i2c_client *client)
{
    /*ths8200_enable should only be used following
	ths8200_disable if THS has been initialized*/
    int err = 0;

    /* clear chip_pwdn bit (exit PD mode) */
    err |= ths8200_write(client, 0x03, 0x01);
    if (err < 0) {
        dev_err(&client->dev, "%s: Unable to communicate with THS8200...\n", __func__);
	return -EINVAL;
    }

    mdelay(1);
    return 0;
}
EXPORT_SYMBOL(ths8200_enable);

/* function puts ths8200 chip in standby by setting the
 *	reset  line high and leaving it so.*/
int ths8200_standby(int gpio)
{
    gpio_direction_output(gpio, 0);
    gpio_set_value(gpio, 1);
    mdelay(1);
	return 0;
}
EXPORT_SYMBOL(ths8200_standby);

/* function resets ths8200 chip using the reset line
 * the specified gpio line must already be setup by the caller
 * or some outside function or tool.*/
int ths8200_reset(int gpio, struct i2c_client *client)
{
	printk(KERN_DEBUG "resetting ths8200 via gpio %d", gpio);
	/* issue reset */
    gpio_direction_output(gpio , 0);
    gpio_set_value(gpio , 1);
    mdelay(1);
    gpio_set_value(gpio , 0);
    mdelay(1);
	return 0;
}
EXPORT_SYMBOL(ths8200_reset);

int ths8200_disable(struct i2c_client *client)
{
    int err = 0;

    /* attempt graceful powerdown:
      set chip_pwdn bit (enter PD mode) */
    err |= ths8200_write(client, 0x03, 0x04);
    if (err < 0) {
        dev_err(&client->dev, "%s: Unable to place THS8200"
			" into power down state...\n", __func__);
	return -EINVAL;
    }

    mdelay(1);
    return 0;
}
EXPORT_SYMBOL(ths8200_disable);

int ths8200_init(struct i2c_client *client)
{
    u8 c;
    int i;
    int err = 0;

    /* issue THS reset:  set reset low, (duration > 200ns)
      disable pwm*/
    twl_i2c_read_u8(TWL4030_MODULE_INTBR, &c, 0x0d);
    c &= ~(0x30);
    twl_i2c_write_u8 (TWL4030_MODULE_INTBR, c, 0x0d);
    /* set GPIO7 as output - pg 506 */
    twl_i2c_read_u8(TWL4030_MODULE_GPIO, &c, 0x03);
    c |= (0x80);
    twl_i2c_write_u8 (TWL4030_MODULE_GPIO, c, 0x03);
    /* ensure reest line is high */
    twl_i2c_read_u8(TWL4030_MODULE_GPIO, &c, 0x06);
    c |= (0x80);
    twl_i2c_write_u8 (TWL4030_MODULE_GPIO, c, 0x06);
    mdelay(1);
    /* pull reset line low (min duration 200ns) */
    c &= ~(0x80);
    twl_i2c_write_u8 (TWL4030_MODULE_GPIO, c, 0x06);
    mdelay(1);
    /* set reset line high again */
    c |= (0x80);
    twl_i2c_write_u8 (TWL4030_MODULE_GPIO, c, 0x06);
    mdelay(1);

    /* execute init sequence */
 	for (i = 0; i < (sizeof(ths8200_init_seq)/2); i++) {
		err |= ths8200_write(client,  ths8200_init_seq[i][0], 
			ths8200_init_seq[i][1]);
		if (err < 0)
			printk(KERN_ERR "%s :error in init seq step %d",
				 __func__, i);
	}
		
	if (err < 0) {
		dev_err(&client->dev, "%s: Error during THS8200 init\n",
		__func__);
		return -EINVAL;
	}

    return 0;
}
EXPORT_SYMBOL(ths8200_init);

void ths8200_cleanup(struct i2c_client *client)
{
	/*no cleanup necessassary*/
}

static int ths8200_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
    //printk(KERN_INFO "ths8200.c: probe...\n");
        
    return 0;
}

static int ths8200_remove(struct i2c_client *client)
{
    /*printk (KERN_INFO "ths8200.c: remove...\n");*/

    return 0;
}

static const struct i2c_device_id ths8200_id[] = {
    {"ths8200", 0},
    {},
};

MODULE_DEVICE_TABLE(i2c, ths8200_id);

static struct i2c_driver ths8200_driver = {
    .driver = {
	.owner	= THIS_MODULE,
	.name	= "ths8200",
    },
	.probe		= ths8200_probe,
	.remove		= ths8200_remove,
	.id_table	= ths8200_id,
};

static __init int init_ths8200(void)
{
    return i2c_add_driver(&ths8200_driver);
}

static __exit void exit_ths8200(void)
{
    i2c_del_driver(&ths8200_driver);
}

module_init(init_ths8200);
module_exit(exit_ths8200);

MODULE_LICENSE("GPL");
