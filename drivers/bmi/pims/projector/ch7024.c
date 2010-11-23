/*
 * Copyright 2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

 /*!
  * @file ch7024.c
  * @brief Driver for CH7024 TV encoder
  *
  * @ingroup Framebuffer
  */
//#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>

#include "ch7024.h"

#define DEBUG_CH7024

static int ch7024_found = 0;
static struct i2c_adapter *ch7024_adap = NULL;

static int i2c_ch7024_client_xfer( char *reg, int reg_len, char *buf, int num,
				  int tran_flag)
{
	struct i2c_msg msg[2];
	int ret;
	if ((ch7024_adap == NULL))
		{
		printk (KERN_ERR "ch7024_adap is NULL\n");
		return -1;
		}
	msg[0].addr = CH7024_I2C_ADDR;
	msg[0].len = reg_len;
	msg[0].buf = reg;
	msg[0].flags = tran_flag;
	msg[0].flags &= ~I2C_M_RD;

	msg[1].addr = CH7024_I2C_ADDR;
	msg[1].len = num;
	msg[1].buf = buf;

	msg[1].flags = tran_flag;
	if (tran_flag & I2C_M_RD) {
		msg[1].flags |= I2C_M_RD;
	} else {
		msg[1].flags &= ~I2C_M_RD;
	}

	ret = i2c_transfer(ch7024_adap, msg, 2);
	return ret;
}

static int bug_i2c_ch7024_polling_read(char *reg, int reg_len, char *buf, int num)
{
	return i2c_ch7024_client_xfer(reg, reg_len, buf, num,I2C_M_RD);
}

static int bug_i2c_ch7024_polling_write(char *reg, int reg_len, char *buf,
					int num)
{
	return i2c_ch7024_client_xfer(reg, reg_len, buf, num, 0);

}

static int ch7024_read_reg(u32 reg, u32 * word, u32 len)
{
	int i;
	u8 *wp = (u8 *) word;

	*word = 0;

	for (i = 0; i < len; i++) {
		int ret = bug_i2c_ch7024_polling_read((char *)&reg, 1, wp, 1);
		if (ret < 0)
			return ret;
		wp++;
		reg++;
	}
	return 0;
}

static int ch7024_write_reg(u32 reg, u32 word, u32 len)
{
	return bug_i2c_ch7024_polling_write((char *)&reg, 1, (u8 *) & word, len);
}

/**
 * PAL B/D/G/H/K/I clock and timting structures
 */
static struct ch7024_clock ch7024_clk_pal = {
	.A = 0x0,
	.P = 0x36b00,
	.N = 0x41eb00,
	.T = 0x3f,
	.PLLN1 = 0x0,
	.PLLN2 = 0x1b,
	.PLLN3 = 0x12,
};

static struct ch7024_input_timing ch7024_timing_pal = {
	.HTI = 950,
	.VTI = 560,
	.HAI = 640,
	.VAI = 480,
	.HW = 60,
	.HO = 250,
	.VW = 40,
	.VO = 40,
	.VOS = CH7024_VOS_PAL_BDGHKI,
};

/**
 * NTSC_M clock and timting structures
 * TODO: change values to work well.
 */
static struct ch7024_clock ch7024_clk_ntsc = {
	.A = 0x0,
	.P = 0x2ac90,
	.N = 0x36fc90,
	.T = 0x3f,
	.PLLN1 = 0x0,
	.PLLN2 = 0x1b,
	.PLLN3 = 0x12,
};

static struct ch7024_input_timing ch7024_timing_ntsc = {
	.HTI = 801,
	.VTI = 554,
	.HAI = 640,
	.VAI = 480,
	.HW = 60,
	.HO = 101,
	.VW = 20,
	.VO = 54,
	.VOS = CH7024_VOS_NTSC_M,
};

/**
 * ch7024_setup
 * initial the CH7024 chipset by setting register
 * @param:
 * 	vos: output video format
 * @return:
 * 	0 successful
 * 	otherwise failed
 */
int ch7024_setup(struct i2c_adapter *adap,int vos)
{
	struct ch7024_input_timing *ch_timing;
	struct ch7024_clock *ch_clk;
#ifdef DEBUG_CH7024
	int i, val;
#endif
	ch7024_adap = adap;
#if 0
	if (!ch7024_found) {
		printk(KERN_ERR "CH7024: no such device to setup!\n");
		return -ENODEV;
	}
#endif
	/* select output video format */
	if (vos == PROJOUT_FMT_PAL) {
		ch_timing = &ch7024_timing_pal;
		ch_clk = &ch7024_clk_pal;
		pr_debug("CH7024: change to PAL video\n");
	} else if (vos == PROJOUT_FMT_NTSC) {
		ch_timing = &ch7024_timing_ntsc;
		ch_clk = &ch7024_clk_ntsc;
		pr_debug("CH7024: change to NTSC video\n");
	} else if (vos == PROJOUT_FMT_QVGA) {
                ch_timing = &ch7024_timing_ntsc;
                ch_clk = &ch7024_clk_ntsc;
                pr_debug("CH7024: change to NTSC video\n");
        }
	  else {

		pr_debug("CH7024: no such video format.\n");
		return -EINVAL;
	}
	printk("Resetting Chrontel Card\n");
	ch7024_write_reg(CH7024_POWER, 0x0C, 1);	/* power on, disable DAC */
	ch7024_write_reg(CH7024_RESET, 0x00, 1);	/* Reset */
	ch7024_write_reg(CH7024_RESET, 0x03, 1);	/* Reset */
	mdelay(10);

	ch7024_write_reg(CH7024_POWER, 0x0C, 1);	/* power on, disable DAC */
	ch7024_write_reg(CH7024_XTAL, CH7024_XTAL_13MHZ, 1);	/* 13MHz cystal */
	ch7024_write_reg(CH7024_SYNC, 0x0D, 1);	/* Master mode, and TTL */
	ch7024_write_reg(CH7024_IDF1, 0x00, 1);
	ch7024_write_reg(CH7024_TVFILTER1, 0x00, 1);	/* set XCH=0 */

	/* set input clock and divider */
	/* set PLL */
	ch7024_write_reg(CH7024_PLL1, ch_clk->PLLN1, 1);
	ch7024_write_reg(CH7024_PLL2, ch_clk->PLLN2, 1);
	ch7024_write_reg(CH7024_PLL3, ch_clk->PLLN3, 1);

	/* set A register */
	ch7024_write_reg(CH7024_PCLK_A1, 0x00, 1);
	ch7024_write_reg(CH7024_PCLK_A2, 0x00, 1);
	ch7024_write_reg(CH7024_PCLK_A3, 0x00, 1);
	ch7024_write_reg(CH7024_PCLK_A4, 0x00, 1);
	/* set P register */
	ch7024_write_reg(CH7024_CLK_P1, (ch_clk->P >> 16) & 0xFF, 1);
	ch7024_write_reg(CH7024_CLK_P2, (ch_clk->P >> 8) & 0xFF, 1);
	ch7024_write_reg(CH7024_CLK_P3, ch_clk->P & 0xFF, 1);
	/* set N register */
	ch7024_write_reg(CH7024_CLK_N1, (ch_clk->N >> 16) & 0xFF, 1);
	ch7024_write_reg(CH7024_CLK_N2, (ch_clk->N >> 8) & 0xFF, 1);
	ch7024_write_reg(CH7024_CLK_N3, ch_clk->N & 0xFF, 1);
	/* set T register */
	ch7024_write_reg(CH7024_CLK_T, ch_clk->T & 0xFF, 1);

	/* set sub-carrier frequency generation method */
	ch7024_write_reg(CH7024_ACIV, 0x10, 1);	/* ACIV = 1, automatical SCF */
	/* TV out pattern and DAC switch */
	ch7024_write_reg(CH7024_OUT_FMT, (0x10 | ch_timing->VOS) & 0xFF, 1);

if (vos != PROJOUT_FMT_QVGA)
{

	/* input settings */
	ch7024_write_reg(CH7024_IDF2, 0x033, 1);
	/* HAI/HTI VAI */
	ch7024_write_reg(CH7024_IN_TIMING1, ((ch_timing->HTI >> 5) & 0x38) |
			 ((ch_timing->HAI >> 8) & 0x07), 1);
	ch7024_write_reg(CH7024_IN_TIMING2, ch_timing->HAI & 0xFF, 1);
	ch7024_write_reg(CH7024_IN_TIMING8, ch_timing->VAI & 0xFF, 1);
	/* HTI VTI */
	ch7024_write_reg(CH7024_IN_TIMING3, ch_timing->HTI & 0xFF, 1);
	ch7024_write_reg(CH7024_IN_TIMING9, ch_timing->VTI & 0xFF, 1);
	/* HW/HO(h) VW */
	ch7024_write_reg(CH7024_IN_TIMING4, ((ch_timing->HW >> 5) & 0x18) |
			 ((ch_timing->HO >> 8) & 0x7), 1);
	ch7024_write_reg(CH7024_IN_TIMING6, ch_timing->HW & 0xFF, 1);
	ch7024_write_reg(CH7024_IN_TIMING11, ch_timing->VW & 0x3F, 1);
	/* HO(l) VO/VAI/VTI */
	ch7024_write_reg(CH7024_IN_TIMING5, ch_timing->HO & 0xFF, 1);
	ch7024_write_reg(CH7024_IN_TIMING7, ((ch_timing->VO >> 4) & 0x30) |
			 ((ch_timing->VTI >> 6) & 0x0C) |
			 ((ch_timing->VAI >> 8) & 0x03), 1);
	ch7024_write_reg(CH7024_IN_TIMING10, ch_timing->VO & 0xFF, 1);

}
	/* adjust the brightness */
	ch7024_write_reg(CH7024_TVBRI, 0x90, 1);

	ch7024_write_reg(CH7024_OUT_TIMING1, 0x4, 1);
	ch7024_write_reg(CH7024_OUT_TIMING2, 0xe0, 1);

	if (vos == PROJOUT_FMT_PAL) {
		ch7024_write_reg(CH7024_V_POS1, 0x03, 1);
		ch7024_write_reg(CH7024_V_POS2, 0x7d, 1);
	} else {
		ch7024_write_reg(CH7024_V_POS1, 0x02, 1);
		ch7024_write_reg(CH7024_V_POS2, 0x7b, 1);
	}

        /* Set up the sub carrier frequency */
	if (vos == PROJOUT_FMT_PAL) {
	}
	else {
	/* We have crystal of 13MHz */
	ch7024_write_reg(CH7024_SC_FREQ4, 0x7E, 1);
	ch7024_write_reg(CH7024_SC_FREQ3, 0xEA, 1);
	ch7024_write_reg(CH7024_SC_FREQ2, 0x33, 1);
	ch7024_write_reg(CH7024_SC_FREQ1, 0x02, 1);

	}

#ifdef DEBUG_CH7024
	for (i = 0; i < CH7024_SC_FREQ4; i++) {

		ch7024_read_reg(i, &val, 1);
		pr_debug("CH7024, reg[0x%x] = %x\n", i, val);
	}
#endif
	return 0;
}

/**
 * ch7024_enable
 * Enable the ch7024 Power to begin TV encoder
 */
void ch7024_enable(struct i2c_adapter *adap)
{
	ch7024_adap = adap;
	if (ch7024_found) {
		ch7024_write_reg(CH7024_POWER, 0x00, 1);
		printk("CH7024 power on.\n");
	}
}

/**
 * ch7024_disable
 * Disable the ch7024 Power to stop TV encoder
 */
void ch7024_disable(struct i2c_adapter *adap)
{
	ch7024_adap = adap;
	if (ch7024_found) {
		ch7024_write_reg(CH7024_POWER, 0x0D, 1);
		printk("CH7024 power off.\n");
	}
}

int ch7024_dump (struct i2c_adapter *adap)
    {
    int i;
    u32 data;
    ch7024_adap = adap;
    for (i =0; i <= CH7024_SC_FREQ4; i++)
        {
        ch7024_read_reg(i, &data, 1);
        printk ("Offset :0%X Value :0%X\n", i, data);
        }
    ch7024_read_reg(0x62, &data, 1);
    printk ("Offset :0x62 Value :0%X\n", data);
    ch7024_read_reg(0x63, &data, 1);
    printk ("Offset :0x63 Value :0%X\n", data);
    ch7024_read_reg(0x7E, &data, 1);
    printk ("Offset :0x7E Value :0%X\n", data);
    return 0;
    }
EXPORT_SYMBOL(ch7024_dump);

int encoder_read_reg (struct i2c_adapter *adap, u32 offset, u32 *data)
    {
    int ret;
    ch7024_adap = adap;
    ret = ch7024_read_reg(offset, data, 1);
    if (ret < 0)
        {
        printk ("Encoder read register failed at offset 0x%X\n", offset);
        return ret;
        }
    return 0;

    }
EXPORT_SYMBOL(encoder_read_reg);

int encoder_write_reg (struct i2c_adapter *adap, u32 offset, u32 data)
    {
    int ret;
    ch7024_adap = adap;
    ret = ch7024_write_reg(offset, data, 1);
    if (ret < 0)
        {
        printk ("Encoder write2 register failed at offset 0x%X\n", offset);
        return ret;
        }
    return 0;

    }
EXPORT_SYMBOL(encoder_write_reg);

int ch7024_detect (struct i2c_adapter *adap)
{
	int ret;
	u32 id;
	ch7024_adap = adap;
	/*TODO client detection */
	ret = ch7024_read_reg(CH7024_DEVID, &id, 1);
	if (ret < 0 || id != CH7024_DEVICE_ID) {
		printk(KERN_ERR
		       "ch7024: TV encoder not present: %d, read ret %d\n", id,
		       ret);
		return -1;
	}
	printk(KERN_ERR "ch7024: TV encoder present: %x, read ret %x\n", id,
	       ret);
	ch7024_found = 1;
	return 0;

}
EXPORT_SYMBOL(ch7024_detect);

int ch7024_set_bright (struct i2c_adapter *adap,u32 val)
{
	ch7024_adap = adap;
	if (val & ~0xFF) {
		printk ("Brighness value is out of range[0-255] %d\n", val);
		return -1;
	} else {
		ch7024_write_reg(CH7024_TVBRI, val, 1);
	}
	return 0;
}
EXPORT_SYMBOL(ch7024_set_bright);

int ch7024_set_cont (struct i2c_adapter *adap,u32 val)
{
	ch7024_adap = adap;
	if (val & ~0x7F) {
		printk ("Contrast value is out of range[0-127] %d\n", val);
		return -1;
	} else {
		ch7024_write_reg(CH7024_TVCTA, val, 1);
	}
	return 0;
}
EXPORT_SYMBOL(ch7024_set_cont);

int ch7024_set_hue (struct i2c_adapter *adap,u32 val)
{
	ch7024_adap = adap;
	if (val & ~0x7F) {
		printk ("Hue value is out of range[0-127] %d\n", val);
		return -1;
	} else {
		ch7024_write_reg(CH7024_TVHUE, val, 1);
	}
	return 0;
}
EXPORT_SYMBOL(ch7024_set_hue);

int ch7024_set_sharp (struct i2c_adapter *adap,u32 val)
{
	ch7024_adap = adap;
	if (val & ~0x07) {
		printk ("Sharpness value is out of range[0-8] %d\n", val);
		return -1;
	} else {
		ch7024_write_reg(CH7024_TVSHARP, val, 1);
	}
	return 0;
}
EXPORT_SYMBOL(ch7024_set_sharp);

int ch7024_set_sat (struct i2c_adapter *adap,u32 val)
{
	ch7024_adap = adap;
	if (val & ~0x7F) {
		printk ("Saturation value is out of range[0-127] %d\n", val);
		return -1;
	} else {
		ch7024_write_reg(CH7024_TVSAT, val, 1);
	}
	return 0;
}
EXPORT_SYMBOL(ch7024_set_sat);

void ch7024_set_attr (struct i2c_adapter *adap,struct ch7024_attr *attributes)
{
	if (!attributes)
		return;
	ch7024_set_bright (adap, attributes->brghtness & 0xFF);
	ch7024_set_cont (adap, attributes->contrast & 0xFF);
	ch7024_set_hue (adap, attributes->hue & 0xFF);
	ch7024_set_sharp (adap, attributes->sharpness & 0xFF);
	ch7024_set_sat (adap,attributes->saturation & 0xFF);
	return;
}

EXPORT_SYMBOL(ch7024_set_attr);


EXPORT_SYMBOL(ch7024_setup);
EXPORT_SYMBOL(ch7024_enable);
EXPORT_SYMBOL(ch7024_disable);



