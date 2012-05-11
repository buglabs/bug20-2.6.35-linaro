/*
 * File:         include/linux/bmi/bmi_battery.h
 * Author:       Andrew Tergis <andrew.tergis@buglabs.net>
 *
 * 		This is the application header file for the BMI bus battery plug in
 * 		ripped from the von hippel for expediency
 */

#ifndef BMI_BATTERY_H
#define BMI_BATTERY_H

#include <linux/bmi/bmi_ioctl.h>

// GPIO
#define AC_EN_GPIO	233

#define BATTERY_GPIO_3		3	// default to input
#define BATTERY_GPIO_2		2	// default to input
#define BATTERY_GPIO_1		1	// default to input
#define BATTERY_GPIO_0		0	// default to input

#define BATTERY_GPIO_LED_ON		0
#define BATTERY_GPIO_LED_OFF		1

// I2C
// I2C Slave Addresses
#define BMI_IOX_I2C_ADDRESS	0x71	// 7-bit address

// von hippel driver ioctl definitions
// IOCTLS 0x0-0x5 reserved for future compatibility with other modules
#define BMI_BATTERY_MKGPIO_OUT	_IOW(BMI_BATTERY_IOCTL, 0x6, unsigned int)		// make a GPIO bit an output
#define BMI_BATTERY_MKGPIO_IN	_IOW(BMI_BATTERY_IOCTL, 0x7, unsigned int)		// make a GPIO bit an input
#define BMI_BATTERY_SETGPIO		_IOW(BMI_BATTERY_IOCTL, 0x8, unsigned int)		// set a GPIO output to 1
#define BMI_BATTERY_CLRGPIO		_IOW(BMI_BATTERY_IOCTL, 0x9, unsigned int)		// set a GPIO output to 0
#define BMI_BATTERY_PRESENT		_IOW(BMI_BATTERY_IOCTL, 0x10, unsigned int)		// Check whether a battery is present
#define BMI_BATTERY_SUSPEND		_IOR(BMI_BATTERY_IOCTL, 0x16, unsigned int)
#define BMI_BATTERY_RESUME		_IOR(BMI_BATTERY_IOCTL, 0x17, unsigned int)

#endif	/* BMI_BATTERY_H */

