/*
 * File:         include/linux/bmi/bmi_comcast_dect.h
 * Author:       Lane Brooks <dirjud@gmail.com>
 *
 */

#ifndef BMI_DECT_H
#define BMI_DECT_H

#include <linux/bmi/bmi_ioctl.h>

#define BMI_DECT_SUSPEND _IOR(BMI_DECT_IOCTL, 0x1, unsigned int)
#define BMI_DECT_RESUME  _IOR(BMI_DECT_IOCTL, 0x2, unsigned int)
#define BMI_DECT_RLEDOFF _IOW(BMI_DECT_IOCTL, 0x3, unsigned int)
#define BMI_DECT_RLEDON  _IOW(BMI_DECT_IOCTL, 0x4, unsigned int)
#define BMI_DECT_GLEDOFF _IOW(BMI_DECT_IOCTL, 0x5, unsigned int)
#define BMI_DECT_GLEDON  _IOW(BMI_DECT_IOCTL, 0x6, unsigned int)
#define BMI_DECT_PROGRAM_INIT  _IOW(BMI_DECT_IOCTL, 0x7, unsigned int)
#define BMI_DECT_PROGRAM_SEQ  _IOW(BMI_DECT_IOCTL, 0x8, unsigned int)
#define BMI_DECT_PROGRAM_SET  _IOW(BMI_DECT_IOCTL, 0x9, unsigned int)
#define BMI_DECT_PROGRAM_GET  _IOW(BMI_DECT_IOCTL, 0xa, unsigned int)


struct dect_prog_seq {
  unsigned char *seq;
  int len;
};


#endif	/* BMI_DECT_H */

