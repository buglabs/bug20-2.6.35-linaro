/*
 * File:         include/linux/bmi/bmi_rfid.h
 * Author:       Andrew Tergis <andrew.tergis@buglabs.net>
 *
 * 		This is the application header file for the BMI bus rfid plug-in
 * 		module.  This has been blatantly ripped off from the bmi_vh code.
 */

#ifndef BMI_RFID_H
#define BMI_RFID_H

#include <linux/bmi/bmi_ioctl.h>

// GPIO
#define RFID_GPIO_RED_LED		3	// default to input
#define RFID_GPIO_GREEN_LED	2	// default to input
#define RFID_GPIO_1		1	// default to input
#define RFID_GPIO_0		0	// default to input

// rfid driver ioctl definitions
#define BMI_RFID_RLEDOFF		_IOW(BMI_RFID_IOCTL, 0x1, unsigned int)		// Turn off red LED
#define BMI_RFID_RLEDON			_IOW(BMI_RFID_IOCTL, 0x2, unsigned int)		// Turn on red LED
#define BMI_RFID_GLEDOFF		_IOW(BMI_RFID_IOCTL, 0x3, unsigned int)		// Turn off green LED
#define BMI_RFID_GLEDON			_IOW(BMI_RFID_IOCTL, 0x4, unsigned int)		// Turn on green LED
#define BMI_RFID_RESET			_IOW(BMI_RFID_IOCTL, 0x5, unsigned int)		// Reset the PIC
#define BMI_RFID_CARD			_IOR(BMI_RFID_IOCTL, 0x6, char *)		// Enable the PIC bootloader
#define BMI_RFID_SUSPEND		_IOR(BMI_RFID_IOCTL, 0x16, unsigned int)
#define BMI_RFID_RESUME			_IOR(BMI_RFID_IOCTL, 0x17, unsigned int)

#endif	/* BMI_RFID_H */

