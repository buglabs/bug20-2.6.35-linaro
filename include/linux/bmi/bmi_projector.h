/*
 * File:         include/linux/bmi/bmi_projector.h
 * Author:      Suresh Rao
 *
 * 		This is the application header file for the BMI bus projector plug-in
 * 		module on the MX31 BUG platform.
 */

#ifndef BMI_PROJECTOR_H
#define BMI_PROJECTOR_H

#include <linux/input.h>
#include <linux/bmi/bmi_ioctl.h>

// IOCTL commands for BMI PROJECTOR driver
#define BMI_PROJECTOR_ON	_IOW(BMI_PROJECTOR_IOCTL, 0x1, __u32)	// turn on projector
#define BMI_PROJECTOR_MODE	_IOW(BMI_PROJECTOR_IOCTL, 0x2, __u32)	// turn on projector
#define BMI_PROJECTOR_OFF	_IOW(BMI_PROJECTOR_IOCTL, 0x3, __u32)	// turn off projector
#define BMI_PROJECTOR_BATTERY	_IOW(BMI_PROJECTOR_IOCTL, 0x4, __u32)	// Battery charger on to bug from  projector

// IOCTL commands for Encoder control
#define BMI_PROJECTOR_HUE		_IOW(BMI_PROJECTOR_IOCTL, 0x5, __u32)	// Hue control in Encoder
#define BMI_PROJECTOR_SATURATION	_IOW(BMI_PROJECTOR_IOCTL, 0x6, __u32)	// Saturation control in Encoder
#define BMI_PROJECTOR_BRIGHTNESS	_IOW(BMI_PROJECTOR_IOCTL, 0x7, __u32)	// Brightness control in Encoder
#define BMI_PROJECTOR_SHARPNESS		_IOW(BMI_PROJECTOR_IOCTL, 0x8, __u32)	// Sharpness control in Encoder
#define BMI_PROJECTOR_CONTRAST		_IOW(BMI_PROJECTOR_IOCTL, 0x9, __u32)	// Contrast control in Encoder

/* BMI_PROJECTOR_MODE settings */
#define PROJECTOR_ECONOMY_MODE	0x0
#define PROJECTOR_BRIGHT_MODE	0x1

#endif	/* BMI_PROJECTOR_H */

