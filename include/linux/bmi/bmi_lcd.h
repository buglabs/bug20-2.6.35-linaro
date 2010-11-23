/*
 * File:         include/linux/bmi/bmi_lcd.h
 * Author:       Peter Giacomini <p.giacomini@encadis.com>
 *
 * 		This is the application header file for the BMI bus lcd plug-in
 * 		module on the MX31 BUG platform.
 */

#ifndef BMI_LCD_H
#define BMI_LCD_H

#include <linux/input.h>
#include <linux/bmi/bmi_ioctl.h>

	// IOCTL commands for BMI LCD driver
#define BMI_LCD_RLEDOFF		_IOW(BMI_LCD_IOCTL, 0x1, __u32)		// turn off Red LED
#define BMI_LCD_RLEDON		_IOW(BMI_LCD_IOCTL, 0x2, __u32)		// turn on Red LED
#define BMI_LCD_GLEDOFF		_IOW(BMI_LCD_IOCTL, 0x3, __u32)		// turn off Green LED
#define BMI_LCD_GLEDON		_IOW(BMI_LCD_IOCTL, 0x4, __u32)		// turn on Green LED
#define BMI_LCD_VSYNC_DIS	_IOW(BMI_LCD_IOCTL, 0x5, __u32)		// Enable VSYNC output buffer
#define BMI_LCD_VSYNC_EN	_IOW(BMI_LCD_IOCTL, 0x6, __u32)		// Disable VSYNC output buffer
#define BMI_LCD_EN		_IOW(BMI_LCD_IOCTL, 0x7, __u32)		// Enable LCD component
#define BMI_LCD_DIS		_IOW(BMI_LCD_IOCTL, 0x8, __u32)		// Disable LCD component
#define BMI_LCD_SER_EN		_IOW(BMI_LCD_IOCTL, 0x9, __u32)		// Enable Seriallizer component
#define BMI_LCD_SER_DIS		_IOW(BMI_LCD_IOCTL, 0xa, __u32)		// Disable Seriallizer component
#define BMI_LCD_SETRST		_IOW(BMI_LCD_IOCTL, 0xb, __u32)		// Disable entire module
#define BMI_LCD_CLRRST		_IOW(BMI_LCD_IOCTL, 0xc, __u32)		// Enable entire module
#define BMI_LCD_SET_BL		_IOW(BMI_LCD_IOCTL, 0xd, __u32)		// Set IOX backlight bits [2:0]
#define BMI_LCD_GETSTAT		_IOR(BMI_LCD_IOCTL, 0xe, __u32)		// Get IOX state
#define BMI_LCD_ACTIVATE	_IOW(BMI_LCD_IOCTL, 0xf, __u32)		// Activate SER, TS, ACCEL
#define BMI_LCD_DEACTIVATE	_IOW(BMI_LCD_IOCTL, 0x10, __u32)	// Deactivate SER, TS, ACCEL
#define BMI_LCD_SUSPEND		_IOW(BMI_LCD_IOCTL, 0x11, __u32)	// Power down module
#define BMI_LCD_RESUME		_IOW(BMI_LCD_IOCTL, 0x12, __u32)	// Power up module

/*Izzy Additions*/
#define BMI_LCD_MIN_XC    0
#define BMI_LCD_MAX_XC    0x3fff
#define BMI_LCD_MIN_YC    0
#define BMI_LCD_MAX_YC    0x3fff

/*struct lcd_ctl
{
  int slot;
  struct cdev cdev;
  struct device *class_dev;
};
*/

//
// Orientation	- location of module 1-3 shorter edge (when facing LCD side) 
// when not FACEUP or FACEDOWN
//
// Note that orientation is only reported through bmi_lcd_ts[0-3]
//
#define ACC_PITCH_MSK	(0xFFFF0000)
#define ACC_ROLL_MSK	(0xFFFF)

	// touch screen input devices
enum {
    BMI_TS_M1,		// bmi_lcd_ts0	- slot 0
    BMI_TS_M2,		// bmi_lcd_ts1	- slot 1
    BMI_TS_M3,		// bmi_lcd_ts2	- slot 2
    BMI_TS_M4,		// bmi_lcd_ts3	- slot 3
    BMI_TS_M13,		// bmi_lcd_ts4	- slot 0 and 2
    BMI_TS_M24,		// bmi_lcd_ts5	- slot 1 and 3
    BMI_TS_M1234,	// bmi_lcd_ts6	- slot 0-3
    BMI_TS_NUM,
} lcd_ts_t;

#endif	/* BMI_LCD_H */

