/*
 * File:         include/linux/bmi/bmi_rf4ce.h
 * Author:       Andrew Tergis <andrew.tergis@buglabs.net>
 *
 * 		This is the application header file for the BMI bus rf4ce plug-in
 * 		module.  This has been blatantly ripped off from the bmi_vh code.
 */

#ifndef BMI_RF4CE_H
#define BMI_RF4CE_H

#include <linux/bmi/bmi_ioctl.h>

// GPIO
#define RF4CE_GPIO_RED_LED		3	// default to input
#define RF4CE_GPIO_GREEN_LED	2	// default to input
#define RF4CE_GPIO_1		1	// default to input
#define RF4CE_GPIO_0		0	// default to input

// rf4ce driver ioctl definitions
#define BMI_RF4CE_RLEDOFF		_IOW(BMI_RF4CE_IOCTL, 0x1, unsigned int)		// Turn off red LED
#define BMI_RF4CE_RLEDON		_IOW(BMI_RF4CE_IOCTL, 0x2, unsigned int)		// Turn on red LED
#define BMI_RF4CE_GLEDOFF		_IOW(BMI_RF4CE_IOCTL, 0x3, unsigned int)		// Turn off green LED
#define BMI_RF4CE_GLEDON		_IOW(BMI_RF4CE_IOCTL, 0x4, unsigned int)		// Turn on green LED
#define BMI_RF4CE_RESET			_IOW(BMI_RF4CE_IOCTL, 0x5, unsigned int)		// Reset the PIC
#define BMI_RF4CE_PROG			_IOW(BMI_RF4CE_IOCTL, 0x6, unsigned int)		// Enable the PIC bootloader
#define BMI_RF4CE_SUSPEND		_IOR(BMI_RF4CE_IOCTL, 0x16, unsigned int)
#define BMI_RF4CE_RESUME		_IOR(BMI_RF4CE_IOCTL, 0x17, unsigned int)

#endif	/* BMI_RF4CE_H */

