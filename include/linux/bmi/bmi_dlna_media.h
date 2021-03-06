/*
 * File:         include/linux/bmi/bmi_camera.h
 * Author:       Lane Brooks <dirjud@gmail.com>
 *
 */

#ifndef BMI_DLNA_MEDIA_H
#define BMI_DLNA_MEDIA_H

#include <linux/bmi/bmi_ioctl.h>

#define BMI_DLNA_MEDIA_SUSPEND _IOR(BMI_DLNA_MEDIA_IOCTL, 0x1, unsigned int)
#define BMI_DLNA_MEDIA_RESUME  _IOR(BMI_DLNA_MEDIA_IOCTL, 0x2, unsigned int)
#define BMI_DLNA_MEDIA_RLEDOFF _IOW(BMI_DLNA_MEDIA_IOCTL, 0x3, unsigned int)
#define BMI_DLNA_MEDIA_RLEDON  _IOW(BMI_DLNA_MEDIA_IOCTL, 0x4, unsigned int)
#define BMI_DLNA_MEDIA_GLEDOFF _IOW(BMI_DLNA_MEDIA_IOCTL, 0x5, unsigned int)
#define BMI_DLNA_MEDIA_GLEDON  _IOW(BMI_DLNA_MEDIA_IOCTL, 0x6, unsigned int)
#define BMI_DLNA_MEDIA_VIXS_REBOOT    _IOW(BMI_DLNA_MEDIA_IOCTL, 0x7, unsigned int)
#define BMI_DLNA_MEDIA_SPI_FLASH_SEL  _IOW(BMI_DLNA_MEDIA_IOCTL, 0x8, unsigned int)
#define BMI_DLNA_MEDIA_SPI_FLASH_OE   _IOW(BMI_DLNA_MEDIA_IOCTL, 0x9, unsigned int)
#define BMI_DLNA_MEDIA_VIXS_RESET   _IOW(BMI_DLNA_MEDIA_IOCTL, 0xa, unsigned int)

#define BMI_DLNA_MEDIA_SPI_FLASH_BUG_SEL  1
#define BMI_DLNA_MEDIA_SPI_FLASH_VIXS_SEL 0


#define BMI_DLNA_MEDIA_MOSTRT  0x0001
#define BMI_DLNA_MEDIA_MOVAL   0x0002
#define BMI_DLNA_MEDIA_TSOV    0x0004
#define BMI_DLNA_MEDIA_TSOS    0x0008
#define BMI_DLNA_MEDIA_TSOD    0x0010
#define BMI_DLNA_MEDIA_MODAT0  0x0100
#define BMI_DLNA_MEDIA_MODAT1  0x0200
#define BMI_DLNA_MEDIA_MODAT2  0x0400
#define BMI_DLNA_MEDIA_MODAT3  0x0800
#define BMI_DLNA_MEDIA_MODAT4  0x1000
#define BMI_DLNA_MEDIA_MODAT5  0x2000
#define BMI_DLNA_MEDIA_MODAT6  0x4000
#define BMI_DLNA_MEDIA_MODAT7  0x8000


#define BMI_DLNA_MEDIA_BOOT_SPI (BMI_DLNA_MEDIA_TSOS | BMI_DLNA_MEDIA_MOVAL | BMI_DLNA_MEDIA_MODAT1 | BMI_DLNA_MEDIA_MODAT2)

#endif	/* BMI_DLNA_MEDIA_H */

