/*
 * File:         include/linux/bmi/bmi_voice.h
 * Author:       Andrew Tergis <andrew.tergis@buglabs.net>
 *
 * 		This is the application header file for the BMI bus voice plug-in
 * 		module.  This has been blatantly ripped off from the bmi_vh code.
 */

#ifndef BMI_VOICE_H
#define BMI_VOICE_H

#include <linux/bmi/bmi_ioctl.h>

// GPIO
#define VOICE_GPIO_EN_SUP		0	// default to output 
#define VOICE_GPIO_ON_OFF		1	// default to output
#define VOICE_GPIO_RESET		2	// default to output
#define VOICE_GPIO_PWR_MON		3	// default to input

// voice driver ioctl definitions
#define BMI_VOICE_SET_EN_SUP		_IOW(BMI_VOICE_IOCTL, 0x1, unsigned int)
#define BMI_VOICE_SET_ON_OFF		_IOW(BMI_VOICE_IOCTL, 0x2, unsigned int)
#define BMI_VOICE_SET_RESET		_IOW(BMI_VOICE_IOCTL, 0x3, unsigned int)
#define BMI_VOICE_GET_PWR_MON		_IOR(BMI_VOICE_IOCTL, 0x4, unsigned int)
#define BMI_VOICE_RESET			_IOW(BMI_VOICE_IOCTL, 0x5, unsigned int)
#define BMI_VOICE_TOGGLE_POWER		_IOW(BMI_VOICE_IOCTL, 0x6, unsigned int)
#define BMI_VOICE_SUSPEND		_IOR(BMI_VOICE_IOCTL, 0x16, unsigned int)
#define BMI_VOICE_RESUME		_IOR(BMI_VOICE_IOCTL, 0x17, unsigned int)

#endif	/* BMI_VOICE_H */

