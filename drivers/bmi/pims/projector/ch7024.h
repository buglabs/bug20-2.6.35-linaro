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
 * @file ch7024.h
 * @brief Driver for CH7024 TV encoder
 *
 * @ingroup Framebuffer
 */
#ifndef _CH7024_H_
#define _CH7024_H_

#ifdef __KERNEL__


/* I2C bus id and device address of CH7024 chip */

#define CH7024_I2C_ADDR	0x76	/* 7bits I2C address */

/*!
 * CH7024 registers
 */
#define CH7024_DEVID		0x00
#define CH7024_REVID		0x01
#define CH7024_PG		    0x02

#define CH7024_RESET		0x03
#define CH7024_POWER		0x04
#define CH7024_TVHUE		0x05
#define CH7024_TVSAT		0x06
#define CH7024_TVCTA		0x07
#define CH7024_TVBRI		0x08
#define CH7024_TVSHARP		0x09
#define CH7024_OUT_FMT		0x0A
#define CH7024_XTAL		    0x0B
#define CH7024_IDF1		    0x0C
#define CH7024_IDF2		    0x0D
#define CH7024_SYNC		    0x0E
#define CH7024_TVFILTER1	0x0F
#define CH7024_TVFILTER2	0x10
#define CH7024_IN_TIMING1	0x11
#define CH7024_IN_TIMING2	0x12
#define CH7024_IN_TIMING3	0x13
#define CH7024_IN_TIMING4	0x14
#define CH7024_IN_TIMING5	0x15
#define CH7024_IN_TIMING6	0x16
#define CH7024_IN_TIMING7	0x17
#define CH7024_IN_TIMING8	0x18
#define CH7024_IN_TIMING9	0x19
#define CH7024_IN_TIMING10	0x1A
#define CH7024_IN_TIMING11	0x1B
#define CH7024_ACIV		    0x1C
#define CH7024_OUT_TIMING1	0x1E
#define CH7024_OUT_TIMING2	0x1F
#define CH7024_V_POS1		0x20
#define CH7024_V_POS2		0x21
#define CH7024_H_POS1		0x22
#define CH7024_H_POS2		0x23
#define CH7024_PCLK_A1		0x24
#define CH7024_PCLK_A2		0x25
#define CH7024_PCLK_A3		0x26
#define CH7024_PCLK_A4		0x27
#define CH7024_CLK_P1		0x28
#define CH7024_CLK_P2		0x29
#define CH7024_CLK_P3		0x2A
#define CH7024_CLK_N1		0x2B
#define CH7024_CLK_N2		0x2C
#define CH7024_CLK_N3		0x2D
#define CH7024_CLK_T		0x2E
#define CH7024_PLL1		    0x2F
#define CH7024_PLL2		    0x30
#define CH7024_PLL3	    	0x31
#define CH7024_SC_FREQ1		0x34
#define CH7024_SC_FREQ2		0x35
#define CH7024_SC_FREQ3		0x36
#define CH7024_SC_FREQ4		0x37
#define CH7024_DATA_IO		0x63

/*!
 * CH7024 register values
 */
/* video output formats */
#define CH7024_VOS_NTSC_M	0x0
#define CH7024_VOS_NTSC_J	0x1
#define CH7024_VOS_NTSC_443	0x2
#define CH7024_VOS_PAL_BDGHKI	0x3
#define CH7024_VOS_PAL_M	0x4
#define CH7024_VOS_PAL_N	0x5
#define CH7024_VOS_PAL_NC	0x6
#define CH7024_VOS_PAL_60	0x7
/* crystal predefined */
#define CH7024_XTAL_13MHZ	0x4
#define CH7024_XTAL_26MHZ	0xB
#define CH7024_XTAL_27MHZ	0xC

/* chip ID */
#define CH7024_DEVICE_ID	0x45

/* clock source define */
#define CLK_HIGH	0
#define CLK_LOW		1

/* CH7024 presets structs */
struct ch7024_clock {
	u32 A;
	u32 P;
	u32 N;
	u32 T;
	u8 PLLN1;
	u8 PLLN2;
	u8 PLLN3;
};

struct ch7024_input_timing {
	u32 HTI;
	u32 VTI;
	u32 HAI;
	u32 VAI;
	u32 HW;
	u32 HO;
	u32 VW;
	u32 VO;
	u32 VOS;
};

struct ch7024_attr{
	u32 brghtness;
	u32 sharpness;
	u32 hue;
	u32 contrast;
	u32 saturation;
};

/* function declare, used by bmi projector module */
int		ch7024_setup			(struct i2c_adapter *adap,int vos);
void 	ch7024_enable		(struct i2c_adapter *adap);
void 	ch7024_disable		(struct i2c_adapter *adap);
int 	ch7024_detect 		(struct i2c_adapter *adap);
void 	ch7024_set_attr 	(struct i2c_adapter *adap, struct ch7024_attr *attributes);
int 	ch7024_set_sat 		(struct i2c_adapter *adap, u32 val);
int 	ch7024_set_sharp 	(struct i2c_adapter *adap, u32 val);
int 	ch7024_set_hue 		(struct i2c_adapter *adap, u32 val);
int 	ch7024_set_cont 	(struct i2c_adapter *adap, u32 val);
int 	ch7024_set_bright 	(struct i2c_adapter *adap, u32 val);
int     ch7024_dump         (struct i2c_adapter *adap);
int     encoder_read_reg    (struct i2c_adapter *adap, u32 offset, u32 *data);
int     encoder_write_reg   (struct i2c_adapter *adap, u32 offset, u32 data);

#endif				/* __KERNEL__ */

/* output video format */
#define PROJOUT_FMT_PAL		0x01
#define PROJOUT_FMT_NTSC	0x02
#define PROJOUT_FMT_QVGA        0x03

#endif				/* _CH7024_H_ */
