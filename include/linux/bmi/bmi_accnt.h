#ifndef BMI_ACCNT_H
#define BMI_ACCNT_H

#include <linux/bmi/bmi_ioctl.h>

// GPIO
#define ACCNT_GPIO_RED_LED		3	// default to input
#define ACCNT_GPIO_GREEN_LED	2	// default to input
#define ACCNT_GPIO_1		1	// default to input
#define ACCNT_GPIO_0		0	// default to input

#define ACCNT_GPIO_LED_ON		0
#define ACCNT_GPIO_LED_OFF		1

// Accenture serial driver ioctl definitions
#define BMI_ACCNT_RLEDOFF		_IOW(BMI_ACCNT_IOCTL, 0x1, unsigned int)		// Turn off red LED
#define BMI_ACCNT_RLEDON		_IOW(BMI_ACCNT_IOCTL, 0x2, unsigned int)		// Turn on red LED
#define BMI_ACCNT_GLEDOFF		_IOW(BMI_ACCNT_IOCTL, 0x3, unsigned int)		// Turn off green LED
#define BMI_ACCNT_GLEDON		_IOW(BMI_ACCNT_IOCTL, 0x4, unsigned int)		// Turn on green LED

#endif

