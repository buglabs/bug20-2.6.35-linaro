/*
 * File:         include/linux/bmi/bmi_gps.h
 * Author:       Peter Giacomini <p.giacomini@encadis.com>
 *
 * 		This is the application header file for the BMI bus gps plug-in
 * 		module on the MX31 BUG platform.
 */

#ifndef BMI_GPS_H
#define BMI_GPS_H

#include <linux/bmi/bmi_ioctl.h>

	// IOCTL commands for BMI GPS driver
#define BMI_GPS_RLEDOFF     _IOW(BMI_GPS_IOCTL, 0x1, unsigned int)		// Turn off red LED
#define BMI_GPS_RLEDON      _IOW(BMI_GPS_IOCTL, 0x2, unsigned int)		// Turn on red LED
#define BMI_GPS_GLEDOFF     _IOW(BMI_GPS_IOCTL, 0x3, unsigned int)		// Turn off green LED
#define BMI_GPS_GLEDON      _IOW(BMI_GPS_IOCTL, 0x4, unsigned int)		// Turn on green LED
#define BMI_GPS_SETBOOT     _IOW(BMI_GPS_IOCTL, 0x5, unsigned int)		// Set BOOT to '1'
#define BMI_GPS_CLRBOOT     _IOW(BMI_GPS_IOCTL, 0x6, unsigned int)		// Set BOOT to '0'
#define BMI_GPS_SETWAKE     _IOW(BMI_GPS_IOCTL, 0x7, unsigned int)		// Set WAKE to '1'
#define BMI_GPS_CLRWAKE     _IOW(BMI_GPS_IOCTL, 0x8, unsigned int)		// Set WAKE to '0'
#define BMI_GPS_GETSTAT     _IOR(BMI_GPS_IOCTL, 0x9, unsigned int *)	// READ IOX register
#define BMI_GPS_SETRST      _IOW(BMI_GPS_IOCTL, 0xA, unsigned int)		// Set RESET to '0'
#define BMI_GPS_CLRRST      _IOW(BMI_GPS_IOCTL, 0xB, unsigned int)		// Set RESET to '1'
#define BMI_GPS_ACTIVE_ANT  _IOW(BMI_GPS_IOCTL, 0xC, unsigned int)		// Select Active Antenna
#define BMI_GPS_PASSIVE_ANT _IOW(BMI_GPS_IOCTL, 0xD, unsigned int)		// Select Passive Antenna
#define BMI_GPS_SUSPEND     _IOW(BMI_GPS_IOCTL, 0xE, unsigned int)
#define BMI_GPS_RESUME      _IOW(BMI_GPS_IOCTL, 0xF, unsigned int)

#endif	/* BMI_GPS_H */

