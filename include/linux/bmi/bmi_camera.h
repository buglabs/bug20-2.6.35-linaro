/*
 * File:         include/linux/bmi/bmi_camera.h
 * Author:       Lane Brooks <dirjud@gmail.com>
 *
 */

#ifndef BMI_CAMERA_A_H
#define BMI_CAMERA_A_H

#include <linux/bmi/bmi_ioctl.h>
#include <linux/videodev2.h>

#define BMI_CAM_SUSPEND		_IOR(BMI_CAMERA_IOCTL, 0x1, unsigned int)
#define BMI_CAM_RESUME		_IOR(BMI_CAMERA_IOCTL, 0x2, unsigned int)
#define BMI_CAM_RLEDOFF		_IOW(BMI_CAMERA_IOCTL, 0x3, unsigned int)
#define BMI_CAM_RLEDON		_IOW(BMI_CAMERA_IOCTL, 0x4, unsigned int)
#define BMI_CAM_GLEDOFF		_IOW(BMI_CAMERA_IOCTL, 0x5, unsigned int)
#define BMI_CAM_GLEDON		_IOW(BMI_CAMERA_IOCTL, 0x6, unsigned int)

#define FLASH_STROBE_OFF          0
#define FLASH_STROBE_TORCH_EN     1
#define FLASH_STROBE_HIGH_BEAM_EN 2

#endif	/* BMI_CAMERA_A_H */

