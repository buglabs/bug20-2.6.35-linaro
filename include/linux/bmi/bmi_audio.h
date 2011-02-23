/*
 * File:        include/linux/bmi/bmi_audio.h
 * Author:      Tim Harvey <tim.harvey@buglabs.com>
 *              Peter Giacomini <p.giacomini@encadis.com>
 *
 * This is the application header file for the BMI bus audio PIM 
 */

#ifndef BMI_AUDIO_H
#define BMI_AUDIO_H

#include <linux/bmi/bmi_ioctl.h>

// GETSTAT defines
typedef enum {
	GETSTAT_AMP	= 0x001,	// IOX bit 0 - amplifier off (active lo)
	GETSTAT_ISPARE	= 0x002,	// IOX bit 1 - spare
	GETSTAT_VOLP	= 0x004,	// IOX bit 2 - VOLP (I - interrupt)
	GETSTAT_VOLD	= 0x008,	// IOX bit 3 - VOLD (I - interrupt)
	GETSTAT_HP_INS	= 0x010,	// IOX bit 4 - HP_INS (I - interrupt)
	GETSTAT_MIC_INS	= 0x020,	// IOX bit 5 - MIC_INS (I - interrupt)
	GETSTAT_LI_INS	= 0x040,	// IOX bit 6 - LI_INS (I - interrupt)
	GETSTAT_LO_INS	= 0x080,	// IOX bit 7 - LO_INS (I - interrupt)
	GETSTAT_GSPARE	= 0x100,	// unused
	GETSTAT_RESET	= 0x200,	// CODEC reset
	GETSTAT_GREEN	= 0x400,	// green LED
	GETSTAT_RED	= 0x800,	// red LED
} BMI_AUDIO_GETSTAT;

// input event bit defintions
typedef enum {
	HEADPHONE_INSERTED         = 0x001,	// Detected headphone insertion
	MICROPHONE_INSERTED        = 0x002,	// Detected microphone insertion
	LINEOUT_INSERTED           = 0x004,	// Detected line out insertion
	LINEIN_INSERTED            = 0x008,	// Detected line in insertion
	VOLUME_DOWN                = 0x010,	// volume down button pressed
	VOLUME_UP                  = 0x020,	// volume up button pressed
} BMI_AUDIO_EVENT;

// IOCTL commands for BMI AUDIO driver
#define BMI_AUDIO_RLEDOFF    _IO(BMI_AUDIO_IOCTL, 0x1) // Turn off red LED
#define BMI_AUDIO_RLEDON     _IO(BMI_AUDIO_IOCTL, 0x2) // Turn on red LED
#define BMI_AUDIO_GLEDOFF    _IO(BMI_AUDIO_IOCTL, 0x3) // Turn off green LED
#define BMI_AUDIO_GLEDON     _IO(BMI_AUDIO_IOCTL, 0x4) // Turn on green LED
#define BMI_AUDIO_SPKOFF     _IO(BMI_AUDIO_IOCTL, 0x5) // Turn off speaker
#define BMI_AUDIO_SPKON      _IO(BMI_AUDIO_IOCTL, 0x6) // Turn on speaker
#define BMI_AUDIO_GETSTAT    _IOR(BMI_AUDIO_IOCTL, 0x9, unsigned int *)	// READ IOX register
#define BMI_AUDIO_SETRST     _IO(BMI_AUDIO_IOCTL, 0xA) // Set RESET to '0'
#define BMI_AUDIO_CLRRST     _IO(BMI_AUDIO_IOCTL, 0xB) // Set RESET to '1'
#define BMI_AUDIO_ACTIVATE   _IO(BMI_AUDIO_IOCTL, 0xC) // Activate a module for audio capture
#define BMI_AUDIO_DEACTIVATE _IO(BMI_AUDIO_IOCTL, 0xD) // Deactivate a module for audio capture
//#define BMI_AUDIO_WCODEC     _IOW(BMI_AUDIO_IOCTL, 0xE, struct codec_xfer *)	// write CODEC register
//#define BMI_AUDIO_RCODEC     _IOR(BMI_AUDIO_IOCTL, 0xF, struct codec_xfer *)	// read CODEC register
#define BMI_AUDIO_SUSPEND    _IO(BMI_AUDIO_IOCTL, 0x10)
#define BMI_AUDIO_RESUME     _IO(BMI_AUDIO_IOCTL, 0x11)

#endif	/* BMI_AUDIO_H */

