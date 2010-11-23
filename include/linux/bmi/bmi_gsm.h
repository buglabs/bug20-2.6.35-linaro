/*
 * File:         include/linux/bmi/bmi_gsm.h
 * Author:       Matt Isaacs <izzy@buglabs.net>
 *
 * 		This is the application header file for the BMI bus GSM/UMTS plug-in
 * 		module on the MX31 BUG platform.
 */

#ifndef BMI_GSM_H
#define BMI_GSM_H

#include <linux/bmi/bmi_ioctl.h>

// GPIO
#define GSM_GPIO_RED_LED      	3	// default to input
#define GSM_GPIO_GREEN_LED	2	// default to input
#define GSM_GPIO_1		1	// default to input
#define GSM_GPIO_0		0	// default to input

#define GSM_GPIO_LED_ON		0
#define GSM_GPIO_LED_OFF	1


// von hippel driver ioctl definitions
#define BMI_GSM_RLEDOFF		_IOW(BMI_GSM_IOCTL, 0x1, unsigned int)		// Turn off red LED
#define BMI_GSM_RLEDON		_IOW(BMI_GSM_IOCTL, 0x2, unsigned int)		// Turn on red LED
#define BMI_GSM_GLEDOFF		_IOW(BMI_GSM_IOCTL, 0x3, unsigned int)		// Turn off green LED
#define BMI_GSM_GLEDON		_IOW(BMI_GSM_IOCTL, 0x4, unsigned int)		// Turn on green LED
#define BMI_GSM_GETSTAT		_IOR(BMI_GSM_IOCTL, 0x5, unsigned int *)		// READ IOX register
#define BMI_GSM_SUSPEND		_IOW(BMI_GSM_IOCTL, 0x6, unsigned int)		
#define BMI_GSM_RESUME 		_IOW(BMI_GSM_IOCTL, 0x7, unsigned int)		


#endif	/* BMI_GSM_H */

