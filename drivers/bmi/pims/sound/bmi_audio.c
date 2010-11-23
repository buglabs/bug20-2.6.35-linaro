/*
 * 	bmi_audio.c
 *
 * 	BMI audio device driver for audio PIMs
 *
 * The BMI Audio driver and this API were developed to support
 * audio playback and record on the BUG audio PIMs.
 *
 * The following operating modes are supported:
 *
 *      Operating Mode                PIM
 *      ---------------------------- -------
 *      Stereo DAC Playback           Yes
 *      Stereo ADC Record             Yes
 *      Output Amplifier Control      Yes
 *      Input Mixer Control           Yes
 *
 * This file also implements the sound driver mixer interface for ALSA.
 *
 * Playback and Recording supports 11025, 22050, and 44100 kHz for stereo.
 *
 * Future Enhanement Options: 
 * 	48000-based rates
 * 	efx control
 * 	I2S -> TDM mode
 * 	power management
 */

/*
 * 	This code was derived from the following sources:
 *
 * @file	pmic_audio.c
 * @file	mxc-alsa-mixer.c
 * @file	mxc-alsa-pmic.c
 *
 * Copyright 2004-2006 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#undef CODEC	// disable CODEC access for code testing
#define CODEC	// enable CODEC access for code testing

#define DEBUG	// enable debug printk
#undef DEBUG	// disable debug printk

#ifdef DEBUG

#  define DDPRINTK(fmt, args...) printk(KERN_ERR"%s :: %d :: %s - " \
		  fmt, __FILE__,__LINE__,__FUNCTION__ , ## args)
#  define DPRINTK(fmt, args...) printk("%s: " fmt, __FUNCTION__ , ## args)
#  define PRINTK(fmt) printk(KERN_INFO fmt)

#  define FUNC_START DPRINTK(" func start\n")
#  define FUNC_END DPRINTK(" func end\n")

#  define FUNC_ERR printk(KERN_ERR"%s :: %d :: %s  err= %d \n", \
		  __FILE__,__LINE__,__FUNCTION__ ,err)

#else	// DEBUG

#define DDPRINTK(fmt, args...)  do {} while(0)
#define DPRINTK(fmt, args...)   do {} while(0)
#define PRINTK(fmt)   do {} while(0)

#endif	// DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/clk.h>

// BMI/BUG interfaces
#include <linux/bmi.h>
#include <linux/bmi/bmi-control.h>
#include <linux/bmi-ids.h>
#include <linux/bmi/bmi_ioctl.h>
#include <linux/bmi/bmi_audio.h>

// control interface - LED/RESET/MODULE ACTIVATION
#include <asm/ioctl.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/major.h>

// I2C interface - IOX/CODEC
#include <linux/i2c.h>
#include <mach/mxc_i2c.h>

// Input interface - BUTTONS/JACKS
#include <linux/input.h>
#include <linux/interrupt.h>

// includes from mxc-alsa-mixer.c:
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <linux/soundcard.h>
#include "../../../mxc/bug_audio/bug-alsa-common.h"

// includes from mxc-alsa-pmic.c:
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif	// CONFIG_PM

#include <mach/dma.h>
#include <mach/spba.h>
#include <mach/clock.h>
#include <asm/mach-types.h>

#include "../../../../arch/arm/mach-mx3/crm_regs.h"
#include "../../../mxc/ssi/ssi.h"
#include "../../../mxc/ssi/registers.h"
#include "../../../mxc/dam/dam.h"

// Global variables
static ushort fcc_test = 0;
static ushort output_ints = 0;

// BMI defines
#define BMIAUDIO_VERSION	"1.0"
#define MAX_STRG		(20)
#define WORK_DELAY		(1)	// input debounce

// BMI private device structure
struct bmi_audio
{
	struct bmi_device   *bdev;			// BMI device
	struct cdev 	     cdev;			// control character device
	struct device *class_dev;			// control device class
	unsigned int         active;			// PIM active
	unsigned int         irq;			// interrupt number
	char                 int_name[MAX_STRG];	// interrupt name for /proc
	struct input_dev     *input_dev;		// button/insertion input device
};

static struct bmi_audio bmi_audio[BMI_AUDIO_PIM_NUM];	// PIM structures
static int major;					// control device major

//
// I2C
//

// I2C Slave Addresses
#define BMI_IOX_I2C_ADDRESS	0x71	// 7-bit address
#define BMI_CODEC_I2C_ADDRESS	0x18	// 7-bit address

// I2C IOX register definitions
#define IOX_INPUT_REG		0x0	// IOX input data register
#define IOX_OUTPUT_REG		0x1	// IOX output data register
#define IOX_POLARITY_REG	0x2	// IOX polarity data register
#define IOX_CONTROL		0x3	// IOX direction control register
#define IOX_AMP			(0)	// bit 0 - amplifier off (O - low active)
#define IOX_SPARE		(1)	// bit 1 - spare
#define IOX_VOLP		(2)	// bit 2 - VOLP (I - interrupt)
#define IOX_VOLD		(3)	// bit 3 - VOLD (I - interrupt)
#define IOX_HP_INS		(4)	// bit 4 - HP_INS (I - interrupt)
#define IOX_MIC_INS		(5)	// bit 5 - MIC_INS (I - interrupt)
#define IOX_LI_INS		(6)	// bit 6 - LI_INS (I - interrupt)
#define IOX_LO_INS		(7)	// bit 7 - LO_INS (I - interrupt)

//
// I2C routines
//

// read byte from I2C IO expander
static int ReadByte_IOX (struct i2c_adapter *adap, unsigned char offset, unsigned char *data)
{
		int	ret = 0;
		struct i2c_msg rmsg[2];
		int	num_msgs;

		// Read Byte with Pointer
		rmsg[0].addr = BMI_IOX_I2C_ADDRESS;
		rmsg[0].flags = 0;	  // write
		rmsg[0].len = 1;
		rmsg[0].buf = &offset;

		rmsg[1].addr = BMI_IOX_I2C_ADDRESS;
		rmsg[1].flags = I2C_M_RD;   // read
		rmsg[1].len = 1;
		rmsg[1].buf = data;

		num_msgs = 2;
		ret = i2c_transfer (adap, rmsg, num_msgs);

		if (ret == 2) {
			ret = 0;
		}
		else {
			printk (KERN_ERR "ReadByte_IOX() - i2c_transfer() failed.\n");
			ret = -1;
		}
		return ret;
}

// write byte to I2C IO expander
static int WriteByte_IOX (struct i2c_adapter *adap, unsigned char offset, unsigned char data)
{
		int	ret = 0;
		struct i2c_msg wmsg[2];
		int	num_msgs;
		
		// Write Byte with Pointer
		wmsg[0].addr = BMI_IOX_I2C_ADDRESS;
		wmsg[0].flags = 0;	// write
		wmsg[0].len = 1;
		wmsg[0].buf = &offset;

		wmsg[1].addr = BMI_IOX_I2C_ADDRESS;
		wmsg[1].flags = 0;	// write
		wmsg[1].len = 1;
		wmsg[1].buf = &data;

		num_msgs = 2;

		ret = i2c_transfer (adap, wmsg, num_msgs);

		if (ret == 2) {
			ret = 0;
		}
		else {
			printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed.\n");
			ret = -1;
		}
		return ret;
}

#ifdef CODEC
// read byte from I2C CODEC
static int ReadByte_CODEC (struct i2c_adapter *adap, unsigned char offset, unsigned char *data)
{
		int	ret = 0;
		struct i2c_msg rmsg[2];
		int	num_msgs;

		// Read Byte with Pointer
		rmsg[0].addr = BMI_CODEC_I2C_ADDRESS;
		rmsg[0].flags = 0;	  // write
		rmsg[0].len = 1;
		rmsg[0].buf = &offset;

		rmsg[1].addr = BMI_CODEC_I2C_ADDRESS;
		rmsg[1].flags = I2C_M_RD;   // read
		rmsg[1].len = 1;
		rmsg[1].buf = data;

		num_msgs = 2;
		ret = i2c_transfer (adap, rmsg, num_msgs);

		if (ret == 2) {
			ret = 0;
		}
		else {
			printk (KERN_ERR "ReadByte_CODEC() - i2c_transfer() failed.\n");
			ret = -1;
		}
		return ret;
}

// write byte to I2C CODEC
static int WriteByte_CODEC (struct i2c_adapter *adap, unsigned char offset, unsigned char data)
{
		int	ret = 0;
		struct i2c_msg wmsg[2];
		int	num_msgs;
		
		// Write Byte with Pointer
		wmsg[0].addr = BMI_CODEC_I2C_ADDRESS;
		wmsg[0].flags = 0;	// write
		wmsg[0].len = 1;
		wmsg[0].buf = &offset;

		wmsg[1].addr = BMI_CODEC_I2C_ADDRESS;
		wmsg[1].flags = 0;	// write
		wmsg[1].len = 1;
		wmsg[1].buf = &data;

		num_msgs = 2;

		ret = i2c_transfer (adap, wmsg, num_msgs);

		if (ret == 2) {
			ret = 0;
		}
		else {
			printk (KERN_ERR "WriteByte_CODEC() - i2c_transfer() failed.\n");
			ret = -1;
		}
		return ret;
}
#endif	// CODEC

//
// control cdev routines
//

// open
int cntl_open (struct inode *inode, struct file *file)
{	
	struct bmi_audio *audio;

	audio = container_of (inode->i_cdev, struct bmi_audio, cdev);
	file->private_data = audio;
	return 0;

}

// release
int cntl_release (struct inode *inode, struct file *file)
{	
	file->private_data = 0;
	return 0;
}

static int configure_CODEC(struct i2c_adapter *adap, struct bmi_audio *audio);

// ioctl
int cntl_ioctl (struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct bmi_audio *audio = (struct bmi_audio *) (file->private_data);
	struct codec_xfer codec_xfer;
	struct i2c_adapter *adap;
	unsigned char iox_data;
	int slot;

	// error if audio/bdev not present
	if (audio == 0)
		return -ENODEV;

	if (audio->bdev == 0)
		return -ENODEV;
	
	adap = bmi_device_get_i2c_adapter (audio->bdev);
	slot = bmi_device_get_slot (audio->bdev);

	// get codec transfer structure
	if ((cmd == BMI_AUDIO_WCODEC) || (cmd == BMI_AUDIO_RCODEC)) {
		if (copy_from_user (&codec_xfer, (struct codec_xfer *) arg, sizeof(struct codec_xfer))) {
			printk (KERN_INFO "bmi_audio.c: ioctl(%d): copy_from_user error\n", slot);
			return -EFAULT;
		}
	}

	// ioctl's
	switch (cmd) {

		case BMI_AUDIO_RLEDOFF:
			bmi_slot_gpio_write_bit (slot, GPIO_RED, BMI_GPIO_ON); // Red LED=OFF 
			break;
	
		case BMI_AUDIO_RLEDON:
			bmi_slot_gpio_write_bit (slot, GPIO_RED, BMI_GPIO_OFF); // Red LED=ON 
			break;
	
		case BMI_AUDIO_GLEDOFF:
			bmi_slot_gpio_write_bit (slot, GPIO_GREEN, BMI_GPIO_ON); // Greem LED=OFF 
			break;
	
		case BMI_AUDIO_GLEDON:
			bmi_slot_gpio_write_bit (slot, GPIO_GREEN, BMI_GPIO_OFF); // Greem LED=ON
			break;
	
	case BMI_AUDIO_SPKON:
	  {
	    printk(KERN_INFO "BMI_AUDIO: SPKOFF Called...\n");
	    if (ReadByte_IOX (adap, IOX_OUTPUT_REG, &iox_data))
	      return -ENODEV;
	    printk(KERN_INFO "BMI_AUDIO: IOX Output Read: 0x%x\n", iox_data);
	    if (WriteByte_IOX (adap, IOX_OUTPUT_REG, (iox_data | (1 << IOX_AMP))))
	      return -ENODEV;
	  }
	  break;
	case BMI_AUDIO_SPKOFF:
	  {
	    printk(KERN_INFO "BMI_AUDIO: SPKON Called...\n");
	    if (ReadByte_IOX (adap, IOX_OUTPUT_REG, &iox_data))
	      return -ENODEV;
	    printk(KERN_INFO "BMI_AUDIO: IOX Output Read: 0x%x\n", iox_data);
	    if (WriteByte_IOX (adap, IOX_OUTPUT_REG, (iox_data & ~(1 << IOX_AMP))))
	      return -ENODEV;
	  }
	  break;
		case BMI_AUDIO_SETRST:
			bmi_slot_gpio_configure_as_output (slot, GPIO_RESET, BMI_GPIO_OFF); // RST = 0;
			break;
	
		case BMI_AUDIO_CLRRST:
			bmi_slot_gpio_configure_as_output (slot, GPIO_RESET, BMI_GPIO_ON); // RST = 1;
			break;
	
		case BMI_AUDIO_GETSTAT:
			{
			int read_data;
	
			if(ReadByte_IOX (adap, IOX_INPUT_REG, &iox_data))
				return -ENODEV;
			
			read_data = iox_data | (bmi_read_gpio_data_reg(slot) << 8);
	
			if(put_user(read_data, (int __user *) arg))
				return -EFAULT;
			}
			break;
	
		case BMI_AUDIO_ACTIVATE:
			audio->active = 0;
			switch (slot) {
				case 0:
					if (bmi_audio[2].active == 0) {
						audio->active = 1;
					}
					break;
				case 1:
					if (bmi_audio[3].active == 0) {
						audio->active = 1;
					}
					break;
				case 2:
					if (bmi_audio[0].active == 0) {
						audio->active = 1;
					}
					break;
				case 3:
					if (bmi_audio[1].active == 0) {
						audio->active = 1;
					}
					break;
			}
#ifdef CODEC
			// write page register
			if (WriteByte_CODEC (adap, 0x0, 0x0))
				return -ENODEV;
			if (audio->active) {	// power-up ADC
				if (WriteByte_CODEC (adap, CODEC_L1L_LPGA, CODEC_L_PGA(0x00) | CODEC_LX_PGA_PU))
					return -ENODEV;
				if (WriteByte_CODEC (adap, CODEC_L1R_RPGA, CODEC_L_PGA(0x00) | CODEC_LX_PGA_PU))
					return -ENODEV;
				configure_CODEC(adap, audio);
			} else {	// power-down ADC
				if (WriteByte_CODEC (adap, CODEC_L1L_LPGA, 0x38))
					return -ENODEV;
				if (WriteByte_CODEC (adap, CODEC_L1R_RPGA, 0x38))
					return -ENODEV;
			}
#endif	// CODEC

		case BMI_AUDIO_DEACTIVATE:
#ifdef CODEC
			// write page register
			if (WriteByte_CODEC (adap, 0x0, 0x0))
				return -ENODEV;
			// power-down ADC
			if (WriteByte_CODEC (adap, CODEC_L1L_LPGA, 0x38))
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_L1R_RPGA, 0x38))
				return -ENODEV;
#endif	// CODEC
			audio->active = 0;
			break;

		case BMI_AUDIO_WCODEC:
#ifdef CODEC
			// write page register
			if (WriteByte_CODEC (adap, 0x0, codec_xfer.page))
				return -ENODEV;
			// write codec register
			if (WriteByte_CODEC (adap, codec_xfer.reg, codec_xfer.data))
				return -ENODEV;
#endif	// CODEC
			break;

		case BMI_AUDIO_RCODEC:
#ifdef CODEC
			// write page register
			if (WriteByte_CODEC (adap, 0x0, codec_xfer.page))
				return -ENODEV;
			// read codec register
			if (ReadByte_CODEC (adap, codec_xfer.reg, &codec_xfer.data))
				return -ENODEV;
			if (copy_to_user ((struct codec_xfer *) arg, &codec_xfer, 
				    sizeof(struct codec_xfer)))
				return -EFAULT;
#endif	// CODEC
			break;

		default:
			return -ENOTTY;
	}

	return 0;
}

// control file operations
struct file_operations cntl_fops = {
	.owner = THIS_MODULE, 
	.ioctl = cntl_ioctl, 
	.open = cntl_open, 
	.release = cntl_release, 
};

// functionality from mxc-alsa-pmic.h:
#define DAM_PORT_4	port_4
#define DAM_PORT_5	port_5
#define TX_WATERMARK	0x4
#define RX_WATERMARK	0x6

// functionality from mxc-alsa-pmic.c:

/*
 * driver buffer policy.
 * Customize here if the sound is not correct
 */
#define MAX_BUFFER_SIZE  	(32*1024)
#define DMA_BUF_SIZE		(8*1024)
#define MIN_PERIOD_SIZE		64
#define MIN_PERIOD		2
#define MAX_PERIOD		255

#define AUD_MUX_CONF 		0x0031010
#define MASK_2_TS		0xfffffffc
#define SOUND_CARD13_NAME	"PIM_AUDIO13"
#define SOUND_CARD24_NAME	"PIM_AUDIO24"

// These defines enable DMA chaining for playback and capture respectively.
#define MXC_SOUND_PLAYBACK_CHAIN_DMA_EN	1
#define MXC_SOUND_CAPTURE_CHAIN_DMA_EN	1

// ID for the PIM cards
static char id13[] = "PIM_AUDIO13";	// slots 1 & 3
static char id24[] = "PIM_AUDIO24";	// slots 2 & 4

#define MXC_ALSA_MAX_PCM_DEV	1
#define MXC_ALSA_MAX_PLAYBACK	1
#define MXC_ALSA_MAX_CAPTURE	1

#define PLAYBACK_STREAM	(0)
#define CAPTURE_STREAM	(1)

#define NUM_CARDS	(2)

/*
 * This structure is the global configuration of the soundcard
 * that are accessed by the mixer as well as by the playback/recording
 * stream. This contains various settings.
 */
typedef struct audio_mixer_control {

	/*
	 * This variable holds the current volume for ouput devices
	 */
	int vol_for_output[OP_MAXDEV];

	/*
	 * This variable holds the current volume for input devices
	 */
	int vol_for_input[IP_MAXDEV];

	/*
	 * This variable holds the current volume for playback devices.
	 */
	int master_volume_out;

	/*
	 * These variables holds the current state of the jack indicators
	 */
	int hp_indicator;
	int mic_indicator;
	int li_indicator;
	int lo_indicator;

	/*
	 * Semaphore used to control the access to this structure.
	 */
	struct semaphore sem;

	/*
	 * These variables are set by PCM stream
	 */
	int codec_playback_active;
	int codec_capture_active;

} audio_mixer_control_t;

/*
 * This structure represents an audio stream in term of
 * channel DMA, HW configuration on CODEC and AudioMux/SSI
 */
typedef struct audio_stream {
	/*
	 * identification string
	 */
	char *id;

	/*
	 * numeric identification
	 */
	int stream_id;

	/*
	 * SSI ID on the ARM side
	 */
	int ssi;

	/*
	 * DAM port on the ARM side
	 */
	int dam_port;

	/*
	 * device identifier for DMA
	 */
	int dma_wchannel;

	/*
	 * we are using this stream for transfer now
	 */
	int active:1;

	/*
	 * current transfer period
	 */
	int period;

	/*
	 * current count of transfered periods
	 */
	int periods;

	/*
	 * are we recording - flag used to do DMA trans. for sync
	 */
	int tx_spin;

	/*
	 * Previous offset value for resume
	 */
	unsigned int old_offset;

	/*
	 * for locking in DMA operations
	 */
	spinlock_t dma_lock;

	/*
	 * Alsa substream pointer
	 */
	struct snd_pcm_substream *stream;

} audio_stream_t;

/*
 * This structure represents the CODEC sound card with its
 * streams and its shared parameters
 */
typedef struct snd_card_mxc_bmi_audio {
	/*
	 * ALSA sound card handle
	 */
	struct snd_card *card;

	/*
	 * ALSA pcm driver type handle
	 */
	struct snd_pcm *pcm[MXC_ALSA_MAX_PCM_DEV];

	/*
	 * playback & capture streams handle
	 * We can support a maximum of 1 playback streams
	 * We can support a maximum of 1 capture streams
	 */
	audio_stream_t s[MXC_ALSA_MAX_CAPTURE + MXC_ALSA_MAX_PLAYBACK];

} mxc_bmi_audio_t;

/*
 * bmi audio chip parameters for IP/OP and volume controls
 */
audio_mixer_control_t audio_mixer_control[NUM_CARDS];

/*
 * Global variable that represents the CODEC soundcard.
 */
mxc_bmi_audio_t *mxc_audio[NUM_CARDS] = { NULL, NULL };

/*
 * Supported playback rates array
 */
static unsigned int playback_rates_stereo[] = {
	8000,
	11025,
	12000,
	16000,
	22050,
	24000,
	32000,
	44100,
	48000,
	88200,
	96000
};

/*
 * Supported capture rates array
 */
static unsigned int capture_rates_stereo[] = {
	8000,
	11025,
	12000,
	16000,
	22050,
	24000,
	32000,
	44100,
	48000,
	88200,
	96000
};

/*
 * this structure represents the sample rates supported
 * by CODEC for playback operations on DAC.
 */
static struct snd_pcm_hw_constraint_list hw_playback_rates_stereo = {
	.count = ARRAY_SIZE(playback_rates_stereo),
	.list = playback_rates_stereo,
	.mask = 0,
};

/*
 * this structure represents the sample rates supported
 * by capture operations on ADC.
 */
static struct snd_pcm_hw_constraint_list hw_capture_rates_stereo = {
	.count = ARRAY_SIZE(capture_rates_stereo),
	.list = capture_rates_stereo,
	.mask = 0,
};

/*
 * This function configures audio multiplexer to support
 * audio data routing in CODEC slave mode.
 *
 * @param       ssi	SSI of the ARM to connect to the DAM.
 */
void configure_dam_bmi_master(int ssi)
{
	int source_port;
	int target_port;

	if (ssi == SSI1) {
		PRINTK("DAM: port 1 -> port 4\n");
		source_port = port_1;
		target_port = port_4;
	} else {
		PRINTK("DAM: port 2 -> port 5\n");
		source_port = port_2;
		target_port = port_5;
	}

	dam_reset_register (source_port);
	dam_reset_register (target_port);

	dam_select_mode (source_port, normal_mode);
	dam_select_mode (target_port, normal_mode);

	dam_set_synchronous (source_port, true);
	dam_set_synchronous (target_port, true);

	dam_select_RxD_source (source_port, target_port);
	dam_select_RxD_source (target_port, source_port);

	dam_select_TxFS_direction (source_port, signal_in);
	dam_select_TxFS_direction (target_port, signal_out);
	dam_select_TxFS_source (target_port, false, source_port);

	dam_select_TxClk_direction (source_port, signal_in);
	dam_select_TxClk_direction (target_port, signal_out);
	dam_select_TxClk_source (target_port, false, source_port);

	dam_select_RxFS_direction (source_port, signal_in);
	dam_select_RxFS_direction (target_port, signal_out);
	dam_select_RxFS_source (source_port, false, target_port);
	dam_select_RxFS_source (target_port, false, source_port);

	dam_select_RxClk_direction (source_port, signal_in);
	dam_select_RxClk_direction (target_port, signal_out);
	dam_select_RxClk_source (source_port, false, target_port);
	dam_select_RxClk_source (target_port, false, source_port);
}

void ssi_rx_sampleRate (int ssi, int samplerate) {
	struct clk *ssi_clk;

	ssi_rx_clock_divide_by_two (ssi, false);
	ssi_rx_clock_prescaler (ssi, false);
	ssi_rx_frame_rate (ssi, 2);

	ssi_clk = clk_get (NULL, "usb_pll.0");

	if (ssi == SSI1) {
		ssi_clk = clk_get (NULL, "ssi_clk.0");
	} else {
		ssi_clk = clk_get (NULL, "ssi_clk.1");
	}

	clk_set_rate (ssi_clk, clk_round_rate (ssi_clk, 11289600));
	switch (samplerate) {
		case 8000:
			ssi_rx_prescaler_modulus (ssi, 17);
			break;
		case 11025:
			ssi_rx_prescaler_modulus (ssi, 12);
			break;
		case 16000:
			ssi_rx_prescaler_modulus (ssi, 8);
			break;
		case 22050:
			ssi_rx_prescaler_modulus (ssi, 6);
			break;
		default:
			if (samplerate != 44100)
				printk (KERN_ERR 
					"ssi_rx_sampleRate(): samplerate=%d not supported (default to 44100).\n", 
					samplerate);

			ssi_rx_prescaler_modulus (ssi, 3);
			break;
	}
}

void ssi_tx_sampleRate (int ssi, int samplerate) {
	struct clk *ssi_clk;

	ssi_tx_clock_divide_by_two (ssi, false);
	ssi_tx_clock_prescaler (ssi, false);
	ssi_tx_frame_rate (ssi, 2);

	ssi_clk = clk_get (NULL, "usb_pll.0");

	if (ssi == SSI1) {
		ssi_clk = clk_get (NULL, "ssi_clk.0");
	} else {
		ssi_clk = clk_get (NULL, "ssi_clk.1");
	}

	clk_set_rate (ssi_clk, clk_round_rate (ssi_clk, 11289600));
	switch (samplerate) {
		case 8000:
			ssi_tx_prescaler_modulus (ssi, 17);
			break;
		case 11025:
			ssi_tx_prescaler_modulus (ssi, 12);
			break;
		case 16000:
			ssi_tx_prescaler_modulus (ssi, 8);
			break;
		case 22050:
			ssi_tx_prescaler_modulus (ssi, 6);
			break;
		default:
			if (samplerate != 44100)
				printk (KERN_ERR 
					"ssi_tx_sampleRate(): samplerate=%d not supported (default to 44100).\n", 
					samplerate);

			ssi_tx_prescaler_modulus (ssi, 3);
			break;
	}
}

/*
 * This function configures the SSI in order to receive audio
 * from CODEC (recording). Configuration of SSI consists mainly in
 * setting the following:
 *
 * 1) SSI to use (SSI1 or SSI2)
 * 2) SSI mode (normal or network. We use always network mode)
 * 3) SSI STCCR register settings, which control the sample rate (BCL and
 *    FS clocks)
 * 4) Watermarks for SSI FIFOs as well as timeslots to be used.
 * 5) Enable SSI.
 *
 * @param	substream	pointer to the structure of the current stream.
 */
void configure_ssi_rx (int ssi)
{
	DPRINTK("configure_ssi_rx: SSI%d\n", ssi + 1);

	// receive set up
	ssi_rx_shift_direction (ssi, ssi_msb_first);
	ssi_rx_clock_polarity (ssi, ssi_clock_on_falling_edge);
	ssi_rx_frame_sync_active (ssi, ssi_frame_sync_active_low);
	ssi_rx_frame_sync_length (ssi, ssi_frame_sync_one_word);
	ssi_rx_early_frame_sync (ssi, ssi_frame_sync_one_bit_before);
	ssi_rx_word_length (ssi, ssi_16_bits);
	ssi_rx_bit0 (ssi, false);

	// FIFO set up
	ssi_rx_fifo_full_watermark (ssi, ssi_fifo_0, RX_WATERMARK);
	ssi_rx_fifo_enable (ssi, ssi_fifo_0, true);
}

/*
 * This function configures the SSI in order to
 * send data to CODEC. Configuration of SSI consists
 * mainly in setting the following:
 *
 * 1) SSI to use (SSI1 or SSI2)
 * 2) SSI mode (normal for normal use e.g. playback, network for mixing)
 * 3) SSI STCCR register settings, which control the sample rate (BCL and
 *    FS clocks)
 * 4) Watermarks for SSI FIFOs as well as timeslots to be used.
 * 5) Enable SSI.
 *
 * @param	substream	pointer to the structure of the current stream.
 */
void configure_ssi_tx (int ssi)
{
	DPRINTK("configure_ssi_tx: SSI%d\n", ssi + 1);

	// disable SSI
	ssi_clock_off (ssi, false);

	// I2S master, 16 bits, 44.1 KHz
	// basic I2S settings
	ssi_network_mode (ssi, true);
	ssi_synchronous_mode (ssi, true);
	ssi_tx_shift_direction (ssi, ssi_msb_first);
	ssi_tx_clock_polarity (ssi, ssi_clock_on_falling_edge);
	ssi_tx_frame_sync_active (ssi, ssi_frame_sync_active_low);
	ssi_tx_frame_sync_length (ssi, ssi_frame_sync_one_word);
	ssi_tx_early_frame_sync (ssi, ssi_frame_sync_one_bit_before);
	ssi_tx_word_length (ssi, ssi_16_bits);
	ssi_tx_bit0 (ssi, false);

	// clocks are being provided by SSI
	ssi_tx_frame_direction (ssi, ssi_tx_rx_internally);
	ssi_tx_clock_direction (ssi, ssi_tx_rx_internally);

	// FIFO set up
	ssi_tx_fifo_enable (ssi, ssi_fifo_0, true);
	ssi_tx_fifo_empty_watermark (ssi, ssi_fifo_0, TX_WATERMARK);

	// Clocking
	ssi_tx_sampleRate (ssi, 44100);
	ssi_system_clock (ssi, true);

	// enable ssi
	ssi_i2s_mode (ssi, i2s_master);
	ssi_enable (ssi, true);
}

/*
 * This function configures number of channels for next audio operation
 * (recording/playback) Number of channels define if sound is stereo
 * or mono.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 */
void set_bmi_channels (struct snd_pcm_substream *substream)
{
	mxc_bmi_audio_t *chip;
	audio_stream_t *s;

	chip = snd_pcm_substream_chip (substream);
	s = &chip->s[substream->pstr->stream];

	ssi_tx_mask_time_slot (s->ssi, MASK_2_TS);
	ssi_rx_mask_time_slot (s->ssi, MASK_2_TS);
}

/*
 * This function configures the DMA channel used to transfer
 * audio from MCU to CODEC
 *
 * @param	substream	pointer to the structure of the current stream.
 * @param       callback        pointer to function that will be
 *                              called when a SDMA TX transfer finishes.
 *
 * @return              0 on success, -1 otherwise.
 */
static int
configure_write_channel (audio_stream_t *s, mxc_dma_callback_t callback, int stream_id)
{
	int ret = -1;
	int channel = -1;

	if (s->ssi == SSI1)
		channel = mxc_dma_request (MXC_DMA_SSI1_16BIT_TX0, "ALSA TX DMA1");
	else
		channel = mxc_dma_request (MXC_DMA_SSI2_16BIT_TX0, "ALSA TX DMA2");

	if (channel < 0) {
		PRINTK("error requesting a write dma channel\n");
		return -1;
	}

	ret = mxc_dma_callback_set (channel, (mxc_dma_callback_t) callback, (void *) s);
	if (ret != 0) {
		mxc_dma_free (channel);
		return -1;
	}
	s->dma_wchannel = channel;

	return 0;
}

/*
 * This function configures the DMA channel used to transfer
 * audio from CODEC to MCU
 *
 * @param	substream	pointer to the structure of the current stream.
 * @param       callback        pointer to function that will be
 *                              called when a SDMA RX transfer finishes.
 *
 * @return              0 on success, -1 otherwise.
 */
static int configure_read_channel (audio_stream_t *s, 
				mxc_dma_callback_t callback)
{
	int ret = -1;
	int channel = -1;

	if (s->ssi == SSI1)
		channel = mxc_dma_request (MXC_DMA_SSI1_16BIT_RX0, "ALSA RX DMA1");
	else
		channel = mxc_dma_request (MXC_DMA_SSI2_16BIT_RX0, "ALSA RX DMA2");

	if (channel < 0) {
		PRINTK("error requesting a read dma channel\n");
		return -1;
	}

	ret =
	    mxc_dma_callback_set (channel, (mxc_dma_callback_t) callback,
				 (void *) s);
	if (ret != 0) {
		mxc_dma_free (channel);
		return -1;
	}
	s->dma_wchannel = channel;

	return 0;
}

/*
 * This function frees the stream structure
 *
 * @param	s	pointer to the structure of the current stream.
 */
static void audio_dma_free (audio_stream_t *s)
{
	/*
	 * There is nothing to be done here since the dma channel has been
	 * freed either in the callback or in the stop method
	 */
}

/*
 * This function gets the dma pointer position during record.
 * Our DMA implementation does not allow to retrieve this position
 * when a transfer is active, so, it answers the middle of
 * the current period beeing transfered
 *
 * @param	s	pointer to the structure of the current stream.
 *
 */
static u_int audio_get_capture_dma_pos (audio_stream_t *s)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int offset;
  
	substream = s->stream;
	runtime = substream->runtime;
	offset = 0;

	// tx_spin value is used here to check if a transfer is active
	if (s->tx_spin) {
		offset = (runtime->period_size * (s->periods)) + 0;
		if (offset >= runtime->buffer_size)
			offset = 0;
		DPRINTK("MXC: audio_get_dma_pos offset  %d\n", offset);
	} else {
		offset = (runtime->period_size * (s->periods));
		if (offset >= runtime->buffer_size)
			offset = 0;
		DPRINTK("MXC: audio_get_dma_pos BIS offset  %d\n", offset);
	}

	return offset;
}

/*
 * This function gets the dma pointer position during playback.
 * Our DMA implementation does not allow to retrieve this position
 * when a transfer is active, so, it answers the middle of
 * the current period beeing transfered
 *
 * @param	s	pointer to the structure of the current stream.
 *
 */
static u_int audio_get_playback_dma_pos (audio_stream_t *s)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int offset;

	substream = s->stream;
	runtime = substream->runtime;
	offset = 0;

	// tx_spin value is used here to check if a transfer is active
	if (s->tx_spin) {
		offset = (runtime->period_size * (s->periods)) + 0;
		if (offset >= runtime->buffer_size)
			offset = 0;
		DPRINTK("MXC: audio_get_dma_pos offset  %d\n", offset);
	} else {
		offset = (runtime->period_size * (s->periods));
		if (offset >= runtime->buffer_size)
			offset = 0;
		DPRINTK("MXC: audio_get_dma_pos BIS offset  %d\n", offset);
	}

	return offset;
}

/*
 * This function stops the current dma transfer for playback
 * and clears the dma pointers.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 */
static void audio_playback_stop_dma (audio_stream_t *s)
{
	unsigned long flags;
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int offset;

	substream = s->stream;
	runtime = substream->runtime;
	dma_size = frames_to_bytes (runtime, runtime->period_size);
	offset = dma_size * s->periods;

	spin_lock_irqsave (&s->dma_lock, flags);

	PRINTK("MXC : audio_stop_dma active = 0\n");
	s->active = 0;
	s->period = 0;
	s->periods = 0;

	// this stops the dma channel and clears the buffer ptrs
	mxc_dma_disable (s->dma_wchannel);
	dma_unmap_single (NULL, runtime->dma_addr + offset, dma_size,
			 DMA_TO_DEVICE);

	spin_unlock_irqrestore (&s->dma_lock, flags);
}

/*
 * This function stops the current dma transfer for capture
 * and clears the dma pointers.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 */
static void audio_capture_stop_dma (audio_stream_t *s)
{
	unsigned long flags;
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int offset;

	substream = s->stream;
	runtime = substream->runtime;
	dma_size = frames_to_bytes (runtime, runtime->period_size);
	offset = dma_size * s->periods;

	spin_lock_irqsave (&s->dma_lock, flags);

	PRINTK("MXC : audio_stop_dma active = 0\n");
	s->active = 0;
	s->period = 0;
	s->periods = 0;

	// this stops the dma channel and clears the buffer ptrs
	mxc_dma_disable (s->dma_wchannel);
	dma_unmap_single (NULL, runtime->dma_addr + offset, dma_size,
			 DMA_FROM_DEVICE);

	spin_unlock_irqrestore (&s->dma_lock, flags);
}

/*
 * This function is called whenever a new audio block needs to be
 * transferred to CODEC. The function receives the address and the size
 * of the new block and start a new DMA transfer.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 */
static void audio_playback_dma (audio_stream_t *s)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size = 0;
	unsigned int offset;
	int ret = 0;
	mxc_dma_requestbuf_t dma_request;
	int device;

	substream = s->stream;
	runtime = substream->runtime;
	device = substream->pcm->device;

	DPRINTK("\nDMA direction %d\(0 is playback 1 is capture)\n",
		 s->stream_id);

	memset (&dma_request, 0, sizeof (mxc_dma_requestbuf_t));

	if (s->active) {
		if (ssi_get_status(s->ssi) & ssi_transmitter_underrun_0) {
			ssi_enable (s->ssi, false);
			ssi_transmit_enable (s->ssi, false);
			ssi_enable (s->ssi, true);
		}
		dma_size = frames_to_bytes (runtime, runtime->period_size);
		DPRINTK("s->period (%x) runtime->periods (%d)\n",
			 s->period, runtime->periods);
		DPRINTK("runtime->period_size (%d) dma_size (%d)\n",
			 (unsigned int) runtime->period_size,
			 runtime->dma_bytes);

		offset = dma_size * s->period;
		snd_assert (dma_size <= DMA_BUF_SIZE,);

		dma_request.src_addr = (dma_addr_t) (dma_map_single (NULL,
								    runtime->
								    dma_area +
								    offset,
								    dma_size,
								    DMA_TO_DEVICE));

		if (s->ssi == SSI1)
			dma_request.dst_addr = (dma_addr_t) (SSI1_BASE_ADDR + MXC_SSI1STX0);
		else
			dma_request.dst_addr = (dma_addr_t) (SSI2_BASE_ADDR + MXC_SSI2STX0);
		dma_request.num_of_bytes = dma_size;

		DPRINTK("MXC: Start DMA offset (%d) size (%d)\n", offset,
			 runtime->dma_bytes);

		mxc_dma_config (s->dma_wchannel, &dma_request, 1,
			       MXC_DMA_MODE_WRITE);
		ret = mxc_dma_enable (s->dma_wchannel);
		ssi_transmit_enable (s->ssi, true);
		ssi_enable (s->ssi, true);
		s->tx_spin = 1;	/* FGA little trick to retrieve DMA pos */

		if (ret) {
			DPRINTK("audio_process_dma: cannot queue DMA buffer\
								(%i)\n", ret);
			return;
		}
		s->period++;
		s->period %= runtime->periods;

#ifdef MXC_SOUND_PLAYBACK_CHAIN_DMA_EN
		if ((s->period > s->periods) && ((s->period - s->periods) > 1)) {
			PRINTK("audio playback chain dma: already double buffered\n");
			return;
		}

		if ((s->period < s->periods)
		    && ((s->period + runtime->periods - s->periods) > 1)) {
			PRINTK("audio playback chain dma: already double buffered\n");
			return;
		}

		if (s->period == s->periods) {
			PRINTK("audio playback chain dma: s->period == s->periods\n");
			return;
		}

		if (snd_pcm_playback_hw_avail(runtime) <
		    2 * runtime->period_size) {
			PRINTK("audio playback chain dma: available data is not enough\n");
			return;
		}

		PRINTK("audio playback chain dma:to set up the 2nd dma buffer\n");
		offset = dma_size * s->period;
		dma_request.src_addr = (dma_addr_t) (dma_map_single (NULL,
								    runtime->
								    dma_area +
								    offset,
								    dma_size,
								    DMA_TO_DEVICE));
		mxc_dma_config (s->dma_wchannel, &dma_request, 1,
			       MXC_DMA_MODE_WRITE);
		ret = mxc_dma_enable (s->dma_wchannel);

		s->period++;
		s->period %= runtime->periods;
#endif	// MXC_SOUND_PLAYBACK_CHAIN_DMA_EN
	}
}

/*
 * This function is called whenever a new audio block needs to be
 * transferred from CODEC. The function receives the address and the size
 * of the block that will store the audio samples and start a new DMA transfer.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 */
static void audio_capture_dma (audio_stream_t *s)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int offset;
	int ret = 0;
	mxc_dma_requestbuf_t dma_request;

	substream = s->stream;
	runtime = substream->runtime;

	DPRINTK("\nDMA direction %d\
		(0 is playback 1 is capture)\n", s->stream_id);

	memset (&dma_request, 0, sizeof (mxc_dma_requestbuf_t));

	if (s->active) {
		dma_size = frames_to_bytes (runtime, runtime->period_size);
		DPRINTK("s->period (%x) runtime->periods (%d)\n",
			 s->period, runtime->periods);
		DPRINTK("runtime->period_size (%d) dma_size (%d)\n",
			 (unsigned int) runtime->period_size,
			 runtime->dma_bytes);

		offset = dma_size * s->period;
		snd_assert (dma_size <= DMA_BUF_SIZE,);

		dma_request.dst_addr = (dma_addr_t) (dma_map_single(NULL,
								    runtime->
								    dma_area +
								    offset,
								    dma_size,
								    DMA_FROM_DEVICE));
		if (s->ssi == SSI1)
			dma_request.src_addr = (dma_addr_t) (SSI1_BASE_ADDR + MXC_SSI1SRX0);
		else
			dma_request.src_addr = (dma_addr_t) (SSI2_BASE_ADDR + MXC_SSI2SRX0);
		dma_request.num_of_bytes = dma_size;

		DPRINTK("MXC: Start DMA offset (%d) size (%d)\n", offset,
			 runtime->dma_bytes);

		mxc_dma_config (s->dma_wchannel, &dma_request, 1,
			       MXC_DMA_MODE_READ);
		ret = mxc_dma_enable (s->dma_wchannel);

		s->tx_spin = 1;	/* FGA little trick to retrieve DMA pos */

		if (ret) {
			DPRINTK("audio_process_dma: cannot queue DMA buffer\
								(%i)\n", ret);
			return;
		}
		s->period++;
		s->period %= runtime->periods;

#ifdef MXC_SOUND_CAPTURE_CHAIN_DMA_EN
		if ((s->period > s->periods) && ((s->period - s->periods) > 1)) {
			PRINTK("audio capture chain dma: already double buffered\n");
			return;
		}

		if ((s->period < s->periods)
		    && ((s->period + runtime->periods - s->periods) > 1)) {
			PRINTK("audio capture chain dma: already double buffered\n");
			return;
		}

		if (s->period == s->periods) {
			PRINTK("audio capture chain dma: s->period == s->periods\n");
			return;
		}

		if (snd_pcm_capture_hw_avail (runtime) <
		    2 * runtime->period_size) {
			PRINTK("audio capture chain dma: available data is not enough\n");
			return;
		}

		PRINTK("audio capture chain dma:to set up the 2nd dma buffer\n");
		offset = dma_size * s->period;
		dma_request.dst_addr = (dma_addr_t) (dma_map_single(NULL,
								    runtime->
								    dma_area +
								    offset,
								    dma_size,
								    DMA_FROM_DEVICE));
		mxc_dma_config (s->dma_wchannel, &dma_request, 1,
			       MXC_DMA_MODE_READ);
		ret = mxc_dma_enable (s->dma_wchannel);

		s->period++;
		s->period %= runtime->periods;
#endif	// MXC_SOUND_CAPTURE_CHAIN_DMA_EN
	}
}

/*
 * This is a callback which will be called
 * when a TX transfer finishes. The call occurs
 * in interrupt context.
 *
 * @param	dat	pointer to the structure of the current stream.
 *
 */
static void audio_playback_dma_callback (void *data, int error,
					unsigned int count)
{
	audio_stream_t *s;
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int previous_period;
	unsigned int offset;

	s = data;
	substream = s->stream;
	runtime = substream->runtime;
	previous_period = s->periods;
	dma_size = frames_to_bytes (runtime, runtime->period_size);
	offset = dma_size * previous_period;

	s->tx_spin = 0;
	s->periods++;
	s->periods %= runtime->periods;

	 // Give back to the CPU the access to the non cached memory
	dma_unmap_single (NULL, runtime->dma_addr + offset, dma_size,
			 DMA_TO_DEVICE);

	/*
	 * If we are getting a callback for an active stream then we inform
	 * the PCM middle layer we've finished a period
	 */
	if (s->active)
		snd_pcm_period_elapsed (s->stream);

	spin_lock (&s->dma_lock);

	// Trig next DMA transfer
	audio_playback_dma (s);

	spin_unlock (&s->dma_lock);
}

/*
 * This is a callback which will be called when a RX transfer finishes. The 
 * call occurs in interrupt context.
 *
 * @param	substream	pointer to the structure of the current stream.
 */
static void audio_capture_dma_callback (void *data, int error,
				       unsigned int count)
{
	audio_stream_t *s;
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int previous_period;
	unsigned int offset;

	s = data;
	substream = s->stream;
	runtime = substream->runtime;
	previous_period = s->periods;
	dma_size = frames_to_bytes (runtime, runtime->period_size);
	offset = dma_size * previous_period;

	s->tx_spin = 0;
	s->periods++;
	s->periods %= runtime->periods;

	// Give back to the CPU the access to the non cached memory
	dma_unmap_single (NULL, runtime->dma_addr + offset, dma_size,
			 DMA_FROM_DEVICE);

	/*
	 * If we are getting a callback for an active stream then we inform
	 * the PCM middle layer we've finished a period
	 */
	if (s->active)
		snd_pcm_period_elapsed (s->stream);

	spin_lock (&s->dma_lock);

	// Trig next DMA transfer
	audio_capture_dma (s);

	spin_unlock (&s->dma_lock);
}

/*
 * This function is a dispatcher of command to be executed
 * by the driver for playback.
 *
 * @param	substream	pointer to the structure of the current stream.
 * @param	cmd		command to be executed
 *
 * @return              0 on success, -1 otherwise.
 */
static int
snd_mxc_audio_playback_trigger (struct snd_pcm_substream *substream, int cmd)
{
	mxc_bmi_audio_t *chip;
	int stream_id = PLAYBACK_STREAM;
	audio_stream_t *s;
	int err;
	int device;

	device = substream->pcm->device;
	chip = snd_pcm_substream_chip (substream);
	stream_id = substream->pstr->stream;
	s = &chip->s[stream_id];
	err = 0;

	// note local interrupts are already disabled in the midlevel code
	spin_lock (&s->dma_lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		PRINTK("MXC: SNDRV_PCM_TRIGGER_START\n");
		s->tx_spin = 0;
		// requested stream startup
		s->active = 1;
		audio_playback_dma (s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		PRINTK("MXC: SNDRV_PCM_TRIGGER_STOP\n");
		// requested stream shutdown
		audio_playback_stop_dma (s);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		PRINTK("MXC : SNDRV_PCM_TRIGGER_SUSPEND active = 0\n");
		s->active = 0;
		s->periods = 0;
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		PRINTK("MXC: SNDRV_PCM_TRIGGER_RESUME\n");
		s->active = 1;
		s->tx_spin = 0;
		audio_playback_dma (s);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		PRINTK("MXC: SNDRV_PCM_TRIGGER_PAUSE_PUSH\n");
		s->active = 0;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		PRINTK("MXC: SNDRV_PCM_TRIGGER_PAUSE_RELEASE\n");
		s->active = 1;
		if (s->old_offset) {
			s->tx_spin = 0;
			audio_playback_dma (s);
			break;
		}
		break;
	default:
		err = -EINVAL;
		break;
	}
	spin_unlock (&s->dma_lock);
	return err;
}

/*
 * This function is a dispatcher of command to be executed
 * by the driver for capture.
 *
 * @param	substream	pointer to the structure of the current stream.
 * @param	cmd		command to be executed
 *
 * @return              0 on success, -1 otherwise.
 */
static int
snd_mxc_audio_capture_trigger (struct snd_pcm_substream *substream, int cmd)
{
	mxc_bmi_audio_t *chip;
	int stream_id;
	audio_stream_t *s;
	int err;

	chip = snd_pcm_substream_chip (substream);
	stream_id = substream->pstr->stream;
	s = &chip->s[stream_id];
	err = 0;

	// note local interrupts are already disabled in the midlevel code
	spin_lock (&s->dma_lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		PRINTK("MXC: SNDRV_PCM_TRIGGER_START\n");
		s->tx_spin = 0;
		// requested stream startup
		s->active = 1;
		audio_capture_dma (s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		PRINTK("MXC: SNDRV_PCM_TRIGGER_STOP\n");
		// requested stream shutdown
		audio_capture_stop_dma (s);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		PRINTK("MXC : SNDRV_PCM_TRIGGER_SUSPEND active = 0\n");
		s->active = 0;
		s->periods = 0;
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		PRINTK("MXC: SNDRV_PCM_TRIGGER_RESUME\n");
		s->active = 1;
		s->tx_spin = 0;
		audio_capture_dma (s);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		PRINTK("MXC: SNDRV_PCM_TRIGGER_PAUSE_PUSH\n");
		s->active = 0;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		PRINTK("MXC: SNDRV_PCM_TRIGGER_PAUSE_RELEASE\n");
		s->active = 1;
		if (s->old_offset) {
			s->tx_spin = 0;
			audio_capture_dma (s);
			break;
		}
		break;
	default:
		err = -EINVAL;
		break;
	}
	spin_unlock (&s->dma_lock);
	return err;
}

/*
 * This function configures the hardware to allow audio
 * playback operations. It is called by ALSA framework.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 * @return              0 on success, -1 otherwise.
 */
static int snd_mxc_audio_playback_prepare (struct snd_pcm_substream *substream)
{
	mxc_bmi_audio_t *chip;
	audio_stream_t *s;
	int ssi;
	int device = -1;
	int stream_id = PLAYBACK_STREAM;
	struct snd_pcm_runtime *runtime;

	device = substream->pcm->device;

	chip = snd_pcm_substream_chip (substream);
	runtime = substream->runtime;
	s = &chip->s[stream_id];
	ssi = s->ssi;

	configure_dam_bmi_master (ssi);
	configure_ssi_rx (ssi);
	ssi_rx_sampleRate (ssi, runtime->rate);
	configure_ssi_tx (ssi);
	ssi_tx_sampleRate (ssi, runtime->rate);
	set_bmi_channels (substream);
	ssi_interrupt_enable (ssi, ssi_tx_dma_interrupt_enable);
	ssi_interrupt_enable(ssi, ssi_tx_interrupt_enable);
	ssi_interrupt_enable (ssi, ssi_tx_fifo_0_empty);
	ssi_enable(ssi, true);

	s->period = 0;
	s->periods = 0;

	msleep (100);

	return 0;
}

/*
 * This function gets the current capture pointer position.
 * It is called by ALSA framework.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 */
static snd_pcm_uframes_t 
snd_mxc_audio_capture_pointer (struct snd_pcm_substream *substream)
{
	mxc_bmi_audio_t *chip;

	chip = snd_pcm_substream_chip (substream);
	return audio_get_capture_dma_pos (&chip->s[substream->pstr->stream]);
}

/*
 * This function gets the current playback pointer position.
 * It is called by ALSA framework.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 */
static snd_pcm_uframes_t
snd_mxc_audio_playback_pointer (struct snd_pcm_substream *substream)
{
	mxc_bmi_audio_t *chip;
	int device;
	int stream_id;
	device = substream->pcm->device;
	stream_id = PLAYBACK_STREAM;
	chip = snd_pcm_substream_chip (substream);
	return audio_get_playback_dma_pos (&chip->s[stream_id]);
}

/*
 * This structure reprensents the capabilities of the driver
 * in capture mode.
 * It is used by ALSA framework.
 */
static struct snd_pcm_hardware snd_mxc_bmi_capture = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = (SNDRV_PCM_RATE_8000_96000 | SNDRV_PCM_RATE_CONTINUOUS),
	.rate_min = 8000,
	.rate_max = 96000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = MAX_BUFFER_SIZE,
	.period_bytes_min = MIN_PERIOD_SIZE,
	.period_bytes_max = DMA_BUF_SIZE,
	.periods_min = MIN_PERIOD,
	.periods_max = MAX_PERIOD,
	.fifo_size = 0,
};

/*
 * This structure reprensents the capabilities of the driver
 * in playback mode for ST-Dac.
 * It is used by ALSA framework.
 */
static struct snd_pcm_hardware snd_mxc_bmi_playback_stereo = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = (SNDRV_PCM_RATE_8000_96000 | SNDRV_PCM_RATE_CONTINUOUS),
	.rate_min = 8000,
	.rate_max = 96000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = MAX_BUFFER_SIZE,
	.period_bytes_min = MIN_PERIOD_SIZE,
	.period_bytes_max = DMA_BUF_SIZE,
	.periods_min = MIN_PERIOD,
	.periods_max = MAX_PERIOD,
	.fifo_size = 0,
};

/*
 * This function opens a CODEC audio device in playback mode
 * It is called by ALSA framework.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 * @return              0 on success, -1 otherwise.
 */
static int snd_card_mxc_audio_playback_open (struct snd_pcm_substream *substream)
{
	mxc_bmi_audio_t *chip;
	struct snd_pcm_runtime *runtime;
	int stream_id = -1;
	int err;
	int device = -1;

	device = substream->pcm->device;

	chip = snd_pcm_substream_chip (substream);
	runtime = substream->runtime;
	stream_id = PLAYBACK_STREAM;

	err = -1;

	audio_mixer_control[chip->s->ssi].codec_playback_active = 1;

	chip->s[stream_id].stream = substream;

	runtime->hw = snd_mxc_bmi_playback_stereo;

	if ((err = snd_pcm_hw_constraint_integer (runtime,
						 SNDRV_PCM_HW_PARAM_PERIODS)) <
	    0)
		return err;
	if ((err = snd_pcm_hw_constraint_list (runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
		&hw_playback_rates_stereo)) < 0)
		return err;

	msleep (10);

	// setup DMA controller for playback
	if ((err =
	     configure_write_channel (&mxc_audio[chip->s->ssi]->s[stream_id],
				     audio_playback_dma_callback,
				     stream_id)) < 0)
		return err;

	return 0;
}

/*
 * This function closes an CODEC audio device for playback.
 * It is called by ALSA framework.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 * @return              0 on success, -1 otherwise.
 */
static int snd_card_mxc_audio_playback_close (struct snd_pcm_substream *substream)
{
	mxc_bmi_audio_t *chip;
	audio_stream_t *s;
	int ssi;
	int device, stream_id = -1;

	device = substream->pcm->device;
	stream_id = PLAYBACK_STREAM;

	chip = snd_pcm_substream_chip (substream);
	s = &chip->s[stream_id];
	ssi = s->ssi;

	audio_mixer_control[chip->s->ssi].codec_playback_active = 0;

	ssi_tx_fifo_enable (ssi, ssi_fifo_0, false);
	ssi_interrupt_disable (ssi, ssi_tx_interrupt_enable);
	ssi_interrupt_disable (ssi, ssi_tx_dma_interrupt_enable);
	ssi_interrupt_disable (ssi, ssi_tx_fifo_0_empty);
	mxc_dma_free ((mxc_audio[ssi]->s[stream_id]).dma_wchannel);

	chip->s[stream_id].stream = NULL;

	return 0;
}

/*
 * This function closes a audio device for capture.
 * It is called by ALSA framework.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 * @return              0 on success, -1 otherwise.
 */
static int snd_card_mxc_audio_capture_close (struct snd_pcm_substream *substream)
{
	mxc_bmi_audio_t *chip;
	audio_stream_t *s;
	int ssi;

	chip = snd_pcm_substream_chip (substream);
	s = &chip->s[substream->pstr->stream];
	ssi = s->ssi;

	audio_mixer_control[ssi].codec_capture_active = 0;

	ssi_rx_fifo_enable (ssi, ssi_fifo_0, false);
	ssi_interrupt_disable(ssi, ssi_rx_interrupt_enable);
	ssi_interrupt_disable (ssi, ssi_rx_dma_interrupt_enable);
	ssi_interrupt_disable (ssi, ssi_rx_fifo_0_full);
	mxc_dma_free ((mxc_audio[ssi]->s[1]).dma_wchannel);

	chip->s[substream->pstr->stream].stream = NULL;

	return 0;
}

/*
 * This function configure the Audio HW in terms of memory allocation.
 * It is called by ALSA framework.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 * @return              0 on success, -1 otherwise.
 */
static int snd_mxc_audio_hw_params (struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime;
	int ret;

	runtime = substream->runtime;
	ret =
	    snd_pcm_lib_malloc_pages (substream, params_buffer_bytes (hw_params));
	if (ret < 0)
		return ret;

	runtime->dma_addr = virt_to_phys (runtime->dma_area);

	DPRINTK("MXC: snd_mxc_audio_hw_params runtime->dma_addr 0x(%x)\n",
		 (unsigned int) runtime->dma_addr);
	DPRINTK("MXC: snd_mxc_audio_hw_params runtime->dma_area 0x(%x)\n",
		 (unsigned int) runtime->dma_area);
	DPRINTK("MXC: snd_mxc_audio_hw_params runtime->dma_bytes 0x(%x)\n",
		 (unsigned int) runtime->dma_bytes);

	return ret;
}

/*
 * This function frees the audio hardware at the end of playback/capture.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 * @return              0 on success, -1 otherwise.
 */
static int snd_mxc_audio_hw_free (struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages (substream);
}

/*
 * This function configures the hardware to allow audio
 * capture operations. It is called by ALSA framework.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 * @return              0 on success, -1 otherwise.
 */
static int snd_mxc_audio_capture_prepare (struct snd_pcm_substream *substream)
{
	mxc_bmi_audio_t *chip;
	audio_stream_t *s;
	struct snd_pcm_runtime *runtime;
	int ssi;

	chip = snd_pcm_substream_chip (substream);
	runtime = substream->runtime;
	s = &chip->s[substream->pstr->stream];
	ssi = s->ssi;

	DPRINTK("substream->pstr->stream %d\n", substream->pstr->stream);
	DPRINTK("SSI%d\n", ssi + 1);

	configure_dam_bmi_master (ssi);
	configure_ssi_tx (ssi);
	ssi_tx_sampleRate (ssi, runtime->rate);
	configure_ssi_rx (ssi);
	ssi_rx_sampleRate (ssi, runtime->rate);

	ssi_interrupt_enable (ssi, ssi_rx_fifo_0_full);
	ssi_interrupt_enable (ssi, ssi_rx_dma_interrupt_enable);
	ssi_interrupt_enable (ssi, ssi_rx_interrupt_enable);
	set_bmi_channels (substream);
	ssi_receive_enable (ssi, true);

	msleep(20);

	s->period = 0;
	s->periods = 0;

	return 0;
}

/*
 * This function opens an audio device in capture mode on Codec.
 * It is called by ALSA framework.
 *
 * @param	substream	pointer to the structure of the current stream.
 *
 * @return              0 on success, -1 otherwise.
 */
static int snd_card_mxc_audio_capture_open (struct snd_pcm_substream *substream)
{
	mxc_bmi_audio_t *chip;
	struct snd_pcm_runtime *runtime;
	int stream_id;
	int err;

	chip = snd_pcm_substream_chip (substream);
	runtime = substream->runtime;
	stream_id = substream->pstr->stream;
	err = -1;

	audio_mixer_control[chip->s->ssi].codec_capture_active = 1;

	chip->s[stream_id].stream = substream;

	if (stream_id == SNDRV_PCM_STREAM_CAPTURE) {
		runtime->hw = snd_mxc_bmi_capture;
	} else {
		return err;
	}

	if ((err = snd_pcm_hw_constraint_integer (runtime,
						 SNDRV_PCM_HW_PARAM_PERIODS)) <
	    0) {
		return err;
	}

	if ((err = snd_pcm_hw_constraint_list (runtime, 0,
					      SNDRV_PCM_HW_PARAM_RATE,
					      &hw_capture_rates_stereo)) < 0) {
		return err;
	}

	// setup DMA controller for Record
	err = configure_read_channel (&mxc_audio[chip->s->ssi]->s[SNDRV_PCM_STREAM_CAPTURE],
				     audio_capture_dma_callback);
	if (err < 0) {
		return err;
	}

	msleep (50);

	return 0;
}

/*
 * This structure is the list of operation that the driver
 * must provide for the capture interface
 */
static struct snd_pcm_ops snd_card_mxc_audio_capture_ops = {
	.open = snd_card_mxc_audio_capture_open,
	.close = snd_card_mxc_audio_capture_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_mxc_audio_hw_params,
	.hw_free = snd_mxc_audio_hw_free,
	.prepare = snd_mxc_audio_capture_prepare,
	.trigger = snd_mxc_audio_capture_trigger,
	.pointer = snd_mxc_audio_capture_pointer,
};

/*
 * This structure is the list of operation that the driver
 * must provide for the playback interface
 */
static struct snd_pcm_ops snd_card_mxc_audio_playback_ops = {
	.open = snd_card_mxc_audio_playback_open,
	.close = snd_card_mxc_audio_playback_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_mxc_audio_hw_params,
	.hw_free = snd_mxc_audio_hw_free,
	.prepare = snd_mxc_audio_playback_prepare,
	.trigger = snd_mxc_audio_playback_trigger,
	.pointer = snd_mxc_audio_playback_pointer,
};

/*
 * This functions initializes the capture audio device supported by
 * CODEC IC.
 *
 * @param	mxc_audio	pointer to the sound card structure
 * @param	device		SSI interface
 */
void init_device_capture (mxc_bmi_audio_t *mxc_audio, int device)
{
	audio_stream_t *audio_stream;

	audio_stream = &mxc_audio->s[SNDRV_PCM_STREAM_CAPTURE];	

	/* 
	 * These parameters defines the identity of
	 * the device (stereoadc or stereodac)
	 */
	if (device == 0) {
		audio_stream->ssi = SSI1;
		audio_stream->dam_port = DAM_PORT_4;
	} else {
		audio_stream->ssi = SSI2;
		audio_stream->dam_port = DAM_PORT_5;
	}
}

/*
 * This functions initializes the playback audio device supported by
 * CODEC IC.
 *
 * @param	mxc_audio	pointer to the sound card structure.
 * @param	device		SSI interface
 */
void init_device_playback (mxc_bmi_audio_t *mxc_audio, int device)
{
	audio_stream_t *audio_stream;
	audio_stream = &mxc_audio->s[0];

	/* These parameters defines the identity of
	 * the device (codec or stereodac)
	 */
	if (device == 0) {
		audio_stream->ssi = SSI1;
		audio_stream->dam_port = DAM_PORT_4;
	} else {
		audio_stream->ssi = SSI2;
		audio_stream->dam_port = DAM_PORT_5;
	}
}

void mxc_bmi_mixer_controls_init (mxc_bmi_audio_t *mxc_audio, int device)
{
	audio_mixer_control_t *audio_control;
	struct i2c_adapter *adap = 0;
	char iox_data[1];
	int i = 0;

	audio_control = &audio_mixer_control[device];

	memset (audio_control, 0, sizeof (audio_mixer_control_t));
	sema_init (&audio_control->sem, 1);

	for (i = 0; i < OP_MAXDEV; i++) {
		audio_control->vol_for_output[i] = 9;	// maximum gain
	}
	for (i = 0; i < IP_MAXDEV; i++) {
		audio_control->vol_for_input[i] = 4;	// gain = -6 dB
	}

	audio_control->master_volume_out = 127;

	if(device == 0) {
		if (bmi_audio[0].active == 1) {
			adap = bmi_device_get_i2c_adapter (bmi_audio[0].bdev);
		} else if (bmi_audio[2].active == 1) {
			adap = bmi_device_get_i2c_adapter (bmi_audio[2].bdev);
		}
	} else {
		if (bmi_audio[1].active == 1) {
			adap = bmi_device_get_i2c_adapter (bmi_audio[1].bdev);
		} else if (bmi_audio[3].active == 1) {
			adap = bmi_device_get_i2c_adapter (bmi_audio[3].bdev);
		}
	}

	if (adap) {
		// IOX status
		if (ReadByte_IOX (adap, IOX_INPUT_REG, iox_data) == 0) {
			audio_control->lo_indicator = (*iox_data >> IOX_LO_INS) & 0x1;
			audio_control->li_indicator = (*iox_data >> IOX_LI_INS) & 0x1;
			audio_control->mic_indicator = (*iox_data >> IOX_MIC_INS) & 0x1;
			audio_control->hp_indicator = (*iox_data >> IOX_HP_INS) & 0x1;
		}
	}
}

/*
 * This functions initializes the audio devices supported by
 * CODEC IC.
 *
 * @param	mxc_audio	pointer to the sound card structure.
 * @param	device	        SSi interface
 */
void mxc_bmi_audio_init (mxc_bmi_audio_t *mxc_audio, int device)
{
	mxc_audio->s[SNDRV_PCM_STREAM_PLAYBACK].id = "Audio out";
	mxc_audio->s[SNDRV_PCM_STREAM_PLAYBACK].stream_id = SNDRV_PCM_STREAM_PLAYBACK;
	mxc_audio->s[SNDRV_PCM_STREAM_CAPTURE].id = "Audio in";
	mxc_audio->s[SNDRV_PCM_STREAM_CAPTURE].stream_id = SNDRV_PCM_STREAM_CAPTURE;

	init_device_playback (mxc_audio, device);
	init_device_capture (mxc_audio, device);
}

/*
 * This function initializes the soundcard structure.
 *
 * @param	mxc_audio	pointer to the sound card structure.
 * @param	device		the device index (zero based)
 *
 * @return              0 on success, -1 otherwise.
 */
static int __init snd_card_mxc_audio_pcm (mxc_bmi_audio_t *mxc_audio, int device)
{
	struct snd_pcm *pcm;
	int err;

	// Create a new PCM instance with 1 capture stream and 1 playback substream
	if (device == 0) {
		if ((err = snd_pcm_new (mxc_audio->card, "PIM_AUDIO13", 0, 1, 1, &pcm)) < 0) {
			return err;
		}
	} else {
		if ((err = snd_pcm_new (mxc_audio->card, "PIM_AUDIO24", 0, 1, 1, &pcm)) < 0) {
			return err;
		}
	}

	/*
	 * this sets up our initial buffers and sets the dma_type to isa.
	 * isa works but I'm not sure why (or if) it's the right choice
	 * this may be too large, trying it for now
	 */
	snd_pcm_lib_preallocate_pages_for_all (pcm, SNDRV_DMA_TYPE_CONTINUOUS,
					      snd_dma_continuous_data
					      (GFP_KERNEL), MAX_BUFFER_SIZE * 2,
					      MAX_BUFFER_SIZE * 2);

	snd_pcm_set_ops (pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_card_mxc_audio_playback_ops);
	snd_pcm_set_ops (pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_card_mxc_audio_capture_ops);

	pcm->private_data = mxc_audio;
	pcm->info_flags = 0;
	if (device == 0)
		strncpy (pcm->name, SOUND_CARD13_NAME, sizeof (pcm->name));
	else
		strncpy (pcm->name, SOUND_CARD24_NAME, sizeof (pcm->name));
	mxc_audio->pcm[0] = pcm;
	mxc_bmi_audio_init (mxc_audio, device);
	mxc_bmi_mixer_controls_init (mxc_audio, device);

	return 0;
}

#if 0 //pjg - POWER_MANAGEMENT
#ifdef CONFIG_PM
/*
  * This function suspends all active streams.
  *
  * TBD
  *
  * @param	card	pointer to the sound card structure.
  * @param	state	requested state
  *
  * @return              0 on success, -1 otherwise.
  */
static int snd_mxc_audio_suspend (struct bmi_device *bdev,
				 pm_message_t state)
{
	struct snd_card *card = bmi_device_get_drvdata (bdev);
	mxc_bmi_audio_t *chip = card->private_data;

	snd_power_change_state (card, SNDRV_CTL_POWER_D3hot);
	snd_pcm_suspend_all (chip->pcm[0]);
	//mxc_alsa_audio_shutdown (chip);

	return 0;
}

/*
  * This function resumes all suspended streams.
  *
  * TBD
  *
  * @param	card	pointer to the sound card structure.
  * @param	state	requested state
  *
  * @return              0 on success, -1 otherwise.
  */
static int snd_mxc_audio_resume (struct bmi_device *bdev)
{
	struct snd_card *card = bmi_device_get_drvdata (bdev);

	snd_power_change_state (card, SNDRV_CTL_POWER_D0);

	return 0;
}
#endif	// CONFIG_PM
#endif //pjg - POWER_MANAGEMENT

/*
 * This function frees the sound card structure
 *
 * @param	card	pointer to the sound card structure.
 *
 * @return              0 on success, -1 otherwise.
 */
void snd_mxc_audio_free (struct snd_card *card)
{
	mxc_bmi_audio_t *chip;

	chip = card->private_data;
	audio_dma_free (&chip->s[SNDRV_PCM_STREAM_PLAYBACK]);
	audio_dma_free (&chip->s[SNDRV_PCM_STREAM_CAPTURE]);
	mxc_audio[chip->s->ssi] = NULL;
	card->private_data = NULL;
	kfree (chip);

}

/*
 * Input interrupt handler and support routines
 */

	// work handler
void bmiaudio_input_work (void *arg, int slot) {
	struct bmi_audio 	*audio = (struct bmi_audio *) arg;
	audio_mixer_control_t	*audio_control;
	struct i2c_adapter	*adap;
	unsigned char		iox_data[1];
	int			input_data;

	if (audio->bdev == 0) {
		printk (KERN_INFO 
			"bmi_audio.c: bmi_audio_input work called with no bdev (slot %d)\n", 
			slot);
		return;
	}

	if (bmi_device_present (audio->bdev) == 0) {
		printk (KERN_INFO 
			"bmi_audio.c: bmi_audio_input work called with no bdev active (slot %d)\n", 
			slot);
		return;
	}

	adap = bmi_device_get_i2c_adapter (audio->bdev);

	// IOX status
	if (ReadByte_IOX (adap, IOX_INPUT_REG, iox_data) != 0) {
		return;
	}
	input_data = 0;
	if ((*iox_data & GETSTAT_VOLP) == 0)
		input_data |= VOLUME_UP;
	if ((*iox_data & GETSTAT_VOLD) == 0)
		input_data |= VOLUME_DOWN;
	if ((*iox_data & GETSTAT_LI_INS) == GETSTAT_LI_INS)
		input_data |= LINEIN_INSERTED;
	if ((*iox_data & GETSTAT_MIC_INS) != GETSTAT_MIC_INS)
		input_data |= MICROPHONE_INSERTED;
	if (output_ints) {
		if ((*iox_data & GETSTAT_LO_INS) == GETSTAT_LO_INS)
			input_data |= LINEOUT_INSERTED;
		if ((*iox_data & GETSTAT_HP_INS) == GETSTAT_HP_INS)
			input_data |= HEADPHONE_INSERTED;
	}
	input_report_abs (audio->input_dev, ABS_MISC, input_data);
	input_sync (audio->input_dev);

	if ((slot == 0) || (slot == 2)) {
		audio_control = &audio_mixer_control[0];
	} else {
		audio_control = &audio_mixer_control[1];
	}

	if (down_interruptible (&audio_control->sem) == 0) {
		audio_control->lo_indicator = (*iox_data >> IOX_LO_INS) & 0x1;
		audio_control->li_indicator = (*iox_data >> IOX_LI_INS) & 0x1;
		audio_control->mic_indicator = (*iox_data >> IOX_MIC_INS) & 0x1;
		audio_control->hp_indicator = (*iox_data >> IOX_HP_INS) & 0x1;
		up (&audio_control->sem);
	}

	enable_irq (audio->irq);
	return;
}

void bmiaudio_input_work0 (struct work_struct *work) {
	bmiaudio_input_work (&bmi_audio[0], 0);
}

void bmiaudio_input_work1 (struct work_struct *work) {
	bmiaudio_input_work (&bmi_audio[1], 1);
}

void bmiaudio_input_work2 (struct work_struct *work) {
	bmiaudio_input_work (&bmi_audio[2], 2);
}

void bmiaudio_input_work3 (struct work_struct *work) {
	bmiaudio_input_work (&bmi_audio[3], 3);
}

DECLARE_DELAYED_WORK(bmiaudio_work0, bmiaudio_input_work0);
DECLARE_DELAYED_WORK(bmiaudio_work1, bmiaudio_input_work1);
DECLARE_DELAYED_WORK(bmiaudio_work2, bmiaudio_input_work2);
DECLARE_DELAYED_WORK(bmiaudio_work3, bmiaudio_input_work3);

static irqreturn_t module_irq_handler (int irq, void *dummy)
{
	disable_irq_nosync (irq);

	switch (irq) {
		case M1_IRQ:
			schedule_delayed_work (&bmiaudio_work0, WORK_DELAY);
			break;
		case M2_IRQ:
			schedule_delayed_work (&bmiaudio_work1, WORK_DELAY);
			break;
		case M3_IRQ:
			schedule_delayed_work (&bmiaudio_work2, WORK_DELAY);
			break;
		case M4_IRQ:
			schedule_delayed_work (&bmiaudio_work3, WORK_DELAY);
			break;
	}
	return IRQ_HANDLED;
}

/*
 * Configure the CODEC registers to the initial values
 */

static int configure_CODEC(struct i2c_adapter *adap, struct bmi_audio *audio)
{
	unsigned char codec_data[1];

	// page select = 0
	if (WriteByte_CODEC (adap, CODEC_PAGE_SEL, 0x00))
		goto nodev;
	if (ReadByte_CODEC (adap, CODEC_PAGE_SEL, codec_data))
		goto nodev;
	if (*codec_data != 0x00)
		goto nodev;

	// ADC,DAC SR = SR/1
	if (WriteByte_CODEC (adap, CODEC_SAMPLE_RATE, (CODEC_SR1 << CODEC_SR_SHIFT) | CODEC_SR1))
		goto nodev;

	// PLL disabled, Q = 2 => Fsref = 46875 Hz
	if (WriteByte_CODEC (adap, CODEC_PLLA, CODEC_PLLA_DIS | CODEC_PLLA_Q(2)))
		goto nodev;

	// CODEC datapath normal
	if (WriteByte_CODEC (adap, CODEC_DATAPATH, 
		    CODEC_DP_44 | CODEC_DP_L(CODEC_DP_REVERSE) | CODEC_DP_R(CODEC_DP_NORMAL)))
		goto nodev;

	// CODEC is slave
	if (WriteByte_CODEC (adap, CODEC_AIFA, CODEC_AIFA_BCLK_S | CODEC_AIFA_WCLK_S | 
		    CODEC_AIFA_DOUT_TS | CODEC_AIFA_CLK_F | CODEC_AIFA_FX_OFF))
		goto nodev;

	// CODEC is in I2S mode, WL = 32
	if (WriteByte_CODEC (adap, CODEC_AIFB, CODEC_AIFB_I2S | CODEC_AIFB_32))
		goto nodev;

	// HP outputs AC coupled
	if (WriteByte_CODEC (adap, CODEC_HS, CODEC_HS_COUPLED | CODEC_HS_ADIFF))
		goto nodev;

	// ADC PGA not muted, maximum gain
	if (WriteByte_CODEC (adap, CODEC_LADC_PGA, CODEC_ADC_PGA_G(0x3C)))	// 59 dB
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_RADC_PGA, CODEC_ADC_PGA_G(0x3C)))	// 59 dB
		goto nodev;

	// M3R -> [LR]PGA -6 dB gain, M3L muted
	if (WriteByte_CODEC (adap, CODEC_M3_LPGA, CODEC_M3_PGA_R(0x04) | CODEC_M3_PGA_LOFF))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_M3_RPGA, CODEC_M3_PGA_R(0x04) | CODEC_M3_PGA_LOFF))
		goto nodev;

	// L1 -> [LR]PGA gain = -6 dB
	// selectively activate ADC
	if (audio->active) {	
		if (WriteByte_CODEC (adap, CODEC_L1L_LPGA, CODEC_L_PGA(0x04) | CODEC_LX_PGA_PU))
			goto nodev;
		if (WriteByte_CODEC (adap, CODEC_L1R_RPGA, CODEC_L_PGA(0x04) | CODEC_LX_PGA_PU))
			goto nodev;
	} else {
		if (WriteByte_CODEC (adap, CODEC_L1L_LPGA, CODEC_L_PGA(0x0F)))
			goto nodev;
		if (WriteByte_CODEC (adap, CODEC_L1R_RPGA, CODEC_L_PGA(0x0F)))
			goto nodev;
	}

	// L2 -> [LR]PGA gain = -6 dB
	if (WriteByte_CODEC (adap, CODEC_L2L_LPGA, CODEC_L_PGA(0x04)))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_L2R_RPGA, CODEC_L_PGA(0x04)))
		goto nodev;

	// MIC Bias = 2V
	if (WriteByte_CODEC (adap, CODEC_MIC_BIAS, CODEC_MIC_BIAS_2V))
		goto nodev;

	// {LR]AGC A
	if (WriteByte_CODEC (adap, CODEC_MIC_LAGC_A, CODEC_MIC_AGC_EN))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_MIC_LAGC_A, CODEC_MIC_AGC_EN))
		goto nodev;

	// {LR]AGC B
	if (WriteByte_CODEC (adap, CODEC_MIC_LAGC_B, CODEC_MIC_AGC_MG(0x40)))	// 30 dB, 0xEC=59 dB
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_MIC_RAGC_B, CODEC_MIC_AGC_MG(0x40)))	// 30 dB, 0xEC=59 dB
		goto nodev;

	// DAC powered up
	if (WriteByte_CODEC (adap, CODEC_DAC_PWR, 
		    CODEC_DAC_PWR_L_EN | CODEC_DAC_PWR_R_EN | CODEC_DAC_PWR_HP_ISE))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_DAC_HPWR, CODEC_DAC_HPWR_HPL_DIFF))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_DAC_HPOS, CODEC_DAC_HPOS_SS_DIS))
		goto nodev;

	// DAC volume = max
	if (WriteByte_CODEC (adap, CODEC_DAC_LVOL, CODEC_DAC_VOL(0x0)))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_DAC_RVOL, CODEC_DAC_VOL(0x0)))
		goto nodev;

	// MIC3->speaker
	if(fcc_test) 
		if (WriteByte_CODEC (adap, CODEC_PGAR_HPLCOM, 0x80))
			goto nodev;

	// output switching volume = max
	if (WriteByte_CODEC (adap, CODEC_DACL1_HPL, CODEC_HP_EN | CODEC_HP_VOL(0x0)))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_DACL1_HPLCOM, CODEC_HP_EN | CODEC_HP_VOL(0x0)))
		goto nodev;

	if (WriteByte_CODEC (adap, CODEC_DACR1_HPR, CODEC_HP_EN | CODEC_HP_VOL(0x0)))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_DACR1_HPRCOM, CODEC_HP_EN | CODEC_HP_VOL(0x0)))
		goto nodev;

	if (WriteByte_CODEC (adap, CODEC_DACL1_LLOPM, CODEC_HP_EN | CODEC_HP_VOL(0x0)))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_DACR1_RLOPM, CODEC_HP_EN | CODEC_HP_VOL(0x0)))
		goto nodev;

	// output levels = max
	if (WriteByte_CODEC (adap, CODEC_HPLOUT, 
		    CODEC_HPX_LC(0x09) | CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_HPLCOM, 
		    CODEC_HPX_LC(0x09) | CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_HPROUT, 
		    CODEC_HPX_LC(0x09) | CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_HPRCOM, 
		    CODEC_HPX_LC(0x09) | CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_LLOPM, 
		    CODEC_HPX_LC(0x09) | CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_RLOPM, 
		    CODEC_HPX_LC(0x09) | CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
		goto nodev;

	// clocking
	if (WriteByte_CODEC (adap, CODEC_CLK, CODEC_CLK_CLKDIV))
		goto nodev;
	if (WriteByte_CODEC (adap, CODEC_CLKGEN, CODEC_CLKGEN_C_M))
		goto nodev;
	return (0);

nodev:
	WriteByte_CODEC (adap, CODEC_L1L_LPGA, 0x38);
	WriteByte_CODEC (adap, CODEC_L1R_RPGA, 0x38);
	return -ENODEV;
}

/*
 * This function initializes the driver in terms of BMI and
 * CODEC functionality.
 *
 * @return              0 on success, >0 otherwise.
 */
static int mxc_alsa_audio_probe (struct bmi_device *bdev)
{
	int rc = 0;
	struct bmi_audio *audio;
	struct i2c_adapter *adap;
	struct cdev *cdev;
	struct class *bmi_class;
	int slot;
	dev_t dev_id;
	int ssi;
	unsigned char iox_data[1];

	// bmi set-up
	slot = bmi_device_get_slot (bdev);
	adap = bmi_device_get_i2c_adapter (bdev);
	audio = &bmi_audio[slot];

	audio->bdev = 0;
	
	// Create 1 minor device
	cdev = &audio->cdev;
	cdev_init (cdev, &cntl_fops);

	dev_id = MKDEV(major, slot); 
	rc = cdev_add (cdev, dev_id, 1);
	if (rc)
		return rc;

	// Create class device 
	bmi_class = bmi_get_bmi_class ();                            
	audio->class_dev = device_create (bmi_class, NULL, MKDEV(major, slot), audio, "bmi_audio_ctrl_m%i", slot+1);  
								     
	if (IS_ERR(audio->class_dev)) {                                
		printk (KERN_ERR "Unable to create class_device for bmi_audio_m%i; errno = %ld\n",
		       slot+1, PTR_ERR(audio->class_dev));             
		cdev_del (&audio->cdev);
		audio->class_dev = NULL;                               
		return -ENODEV;
	}                                                            

	// bind driver and bmi_device 
	audio->bdev = bdev;

	// check for opposite side already active
	switch (slot) {
		case 0:
	    		ssi = 0;
			if (bmi_audio[2].active == 0) {
				mxc_audio[0]->card->dev = &bdev->dev;
				audio->active = 1;
			}
			break;
		case 1:
	    		ssi = 1;
			if (bmi_audio[3].active == 0) {
				mxc_audio[1]->card->dev = &bdev->dev;
				audio->active = 1;
			}
			break;
		case 2:
	    		ssi = 0;
			if (bmi_audio[0].active == 0) {
				mxc_audio[0]->card->dev = &bdev->dev;
				audio->active = 1;
			}
			break;
		case 3:
	    		ssi = 1;
			if (bmi_audio[1].active == 0) {
				mxc_audio[1]->card->dev = &bdev->dev;
				audio->active = 1;
			}
			break;
	}

	bmi_device_set_drvdata (bdev, &bmi_audio[slot]);

	// Initialize GPIOs (turn LED's on )
	// bmi_slot_gpio_configure_as_output (int slot, int gpio, int data)  
	bmi_slot_gpio_configure_as_output (slot, GPIO_RED, BMI_GPIO_OFF);	// Red LED=ON
	bmi_slot_gpio_configure_as_output (slot, GPIO_GREEN, BMI_GPIO_OFF);	// Green LED=ON
	bmi_slot_gpio_configure_as_output (slot, GPIO_RESET, BMI_GPIO_OFF);	// Assert RST = 0;
	bmi_slot_gpio_configure_as_input (slot, GPIO_SPARE);			// unused

	mdelay (200);

	// turn LED's off
	bmi_slot_gpio_write_bit (slot, GPIO_RED, BMI_GPIO_ON);		// Red LED=OFF 
	bmi_slot_gpio_write_bit (slot, GPIO_GREEN, BMI_GPIO_ON);	// Green LED=OFF 

	// release reset
	bmi_slot_gpio_write_bit (slot, GPIO_RESET, BMI_GPIO_ON);	// Reset = 1

	// configure IOX
	if (WriteByte_IOX (adap, IOX_OUTPUT_REG, 0x01)) 
		goto nodev;

	if (output_ints) {
		if (WriteByte_IOX (adap, IOX_CONTROL, 0xFC))    	 // IOX[1:0]=OUT, IOX[7:2]=IN
			goto nodev;
	} else {
		if (WriteByte_IOX (adap, IOX_CONTROL, 0x6C))    	 // IOX[7,4,1:0]=OUT, IOX[6:5,3:2]=IN
			goto nodev;
	}

	if (ReadByte_IOX (adap, IOX_INPUT_REG, iox_data))	// clear interrupts
		goto nodev;

	printk (KERN_INFO "bmi_audio.c: probe(%d) IOX = 0x%x\n", slot, *iox_data & 0xFF);

#ifdef CODEC	// CODEC
	// configure codec
	if (configure_CODEC(adap, audio))
		goto nodev;
#endif	// CODEC

	// set up input interrupt
	audio->irq = bmi_device_get_status_irq (bdev);
	snprintf (audio->int_name, sizeof (audio->int_name), "bmi_audio_stat_m%d", slot + 1);
	if  (request_irq (audio->irq, &module_irq_handler, 0, audio->int_name, &bmi_audio[slot])) {
		printk (KERN_ERR "bmi_audio.c: Can't allocate irq %d or find audio in slot %d\n", 
			audio->irq, slot + 1); 
		device_destroy (bmi_class, MKDEV(major, slot));
		cdev_del (&audio->cdev);
		audio->class_dev = NULL;                               
		return -EBUSY;
	}

       	// power stablization delay
        mdelay (500);
	return 0;

nodev:
	device_destroy (bmi_class, MKDEV(major, slot));
	cdev_del (&audio->cdev);
	audio->class_dev = NULL;                               
	return -ENODEV;
}

void mxc_alsa_audio_remove (struct bmi_device *bdev)
{
	struct bmi_audio *audio = (struct bmi_audio *) (bmi_device_get_drvdata (bdev));
	int slot = bmi_device_get_slot (bdev);
	struct class *bmi_class;

	// release sound card srtucture
	if (bmi_audio[slot].active && ((slot == 0) || (slot == 2)))
		mxc_audio[0]->card->dev = NULL;
	else if (bmi_audio[slot]. active && ((slot == 1) || (slot == 3)))
		mxc_audio[1]->card->dev = NULL;

	// remove input interrupt scheduled work
	free_irq (bmi_device_get_status_irq (bdev), &bmi_audio[slot]);
	switch(slot) {
		case 0: 
			cancel_delayed_work(&bmiaudio_work0);
			break;
		case 1: 
			cancel_delayed_work(&bmiaudio_work1);
			break;
		case 2: 
			cancel_delayed_work(&bmiaudio_work2);
			break;
		case 3: 
			cancel_delayed_work(&bmiaudio_work3);
			break;
	}


	// deconfigure GPIO
	bmi_slot_gpio_configure_all_as_inputs (slot);

	// remove class device
	bmi_class = bmi_get_bmi_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	// clean slot structure
	cdev_del (&audio->cdev);
	audio->class_dev = NULL;
	audio->bdev = NULL;
	audio->active = 0;

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);

	return;
}

// BMI device ID table
static struct bmi_device_id bmi_audio_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_AUDIO, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },	// terminate list
};
MODULE_DEVICE_TABLE(bmi, bmi_audio_tbl);

static struct bmi_driver bmi_audio_driver = {
	.name = "bmi_audio", 
	.id_table = bmi_audio_tbl, 
	.probe = mxc_alsa_audio_probe,
	.remove = mxc_alsa_audio_remove,
#if 0 //pjg - POWER_MANAGEMENT
#ifdef CONFIG_PM
	.suspend = snd_mxc_audio_suspend,
	.resume = snd_mxc_audio_resume,
#endif
#endif	//pjg - POWER_MANAGEMENT
	.driver = {
		   .name = "pim_ALSA",
		   },
};

// mxc-alsa-mixer.c

/*
  * These are the functions implemented in the ALSA PCM driver that
  * are used for mixer operations
  *
  */

	//
	// DAC Volume control
	//
	// PIM_AUDIO13
static int bmi_pb_volume_put0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];
	volume = 127 - volume;

	// get I2C dapter
	if (bmi_audio[0].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[0].bdev);
	} else if (bmi_audio[2].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[2].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_DAC_LVOL, CODEC_DAC_VOL(volume)))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_DAC_RVOL, CODEC_DAC_VOL(volume)))
			return -ENODEV;
	} else {
		return -1;
	}
#endif	// CODEC

	if (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	audio_mixer_control[0].master_volume_out = uvalue->value.integer.value[0];

	up (&audio_mixer_control[0].sem);

	return 0;
}
static int bmi_pb_volume_info0 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 127;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_pb_volume_get0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	char val[1];

	*val = 0x0;

	// get I2C dapter
	if (bmi_audio[0].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[0].bdev);
	} else if (bmi_audio[2].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[2].bdev);
	}

#ifdef CODEC
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (ReadByte_CODEC (adap, CODEC_DAC_LVOL, val))
			return -ENODEV;
	} else {
		*val = 0;
	}
#endif	// CODEC

	uvalue->value.integer.value[0] = 127 - ((int) *val);

#ifdef CODEC
	if (adap)
		return 0;
	else
		return -1;
#endif	// CODEC
	return 0;
}

struct snd_kcontrol_new bmi_control_pb_vol0 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Playback Volume",
	.index = 0x00,
	.info = bmi_pb_volume_info0,
	.get = bmi_pb_volume_get0,
	.put = bmi_pb_volume_put0,
	.private_value = 0xffab1,
};

// PIM_AUDIO24
static int bmi_pb_volume_put1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];
	volume = 127 - volume;

	// get I2C dapter
	if  (bmi_audio[1].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[1].bdev);
	} else if (bmi_audio[3].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[3].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_DAC_LVOL, CODEC_DAC_VOL(volume)))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_DAC_RVOL, CODEC_DAC_VOL(volume)))
			return -ENODEV;
	} else {
		return -1;
	}
#endif	// CODEC

	if (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	audio_mixer_control[1].master_volume_out = uvalue->value.integer.value[0];

	up (&audio_mixer_control[1].sem);

	return 0;
}
static int bmi_pb_volume_info1 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 127;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_pb_volume_get1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	char val[1];

	*val = 0x0;

	// get I2C dapter
	if (bmi_audio[1].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[1].bdev);
	} else if (bmi_audio[3].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[3].bdev);
	}

#ifdef CODEC
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (ReadByte_CODEC (adap, CODEC_DAC_LVOL, val))
			return -ENODEV;
	} else {
		*val = 0;
	}
#endif	// CODEC

	uvalue->value.integer.value[0] = 127 - ((int) *val);

#ifdef CODEC
	if (adap)
		return 0;
	else
		return -1;
#endif	// CODEC
	return 0;
}

struct snd_kcontrol_new bmi_control_pb_vol1 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Playback Volume",
	.index = 0x00,
	.info = bmi_pb_volume_info1,
	.get = bmi_pb_volume_get1,
	.put = bmi_pb_volume_put1,
	.private_value = 0xffab1,
};

//
// LI (L1) Volume control
//
// PIM_AUDIO13
static int bmi_li_volume_put0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];
	volume = 8 - volume;
	if (volume == 0)
		volume = 0xF;	// mute

	// get I2C dapter
	if (bmi_audio[0].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[0].bdev);
	} else if (bmi_audio[2].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[2].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_L1L_LPGA, CODEC_L_PGA(volume) | CODEC_LX_PGA_PU))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_L1R_RPGA, CODEC_L_PGA(volume) | CODEC_LX_PGA_PU))
			return -ENODEV;
	} else {
		return -1;
	}
#endif	// CODEC

	if (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	audio_mixer_control[0].vol_for_input[IP_LINEIN] = uvalue->value.integer.value[0];

	up (&audio_mixer_control[0].sem);

	return 0;
}
static int bmi_li_volume_info0 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 8;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_li_volume_get0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	if (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	uvalue->value.integer.value[0] = audio_mixer_control[0].vol_for_input[IP_LINEIN];

	up (&audio_mixer_control[0].sem);

	return 0;
}

struct snd_kcontrol_new bmi_control_li_vol0 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Linein Capture Volume/Mute",
	.index = 0x00,
	.info = bmi_li_volume_info0,
	.get = bmi_li_volume_get0,
	.put = bmi_li_volume_put0,
	.private_value = 0xffab3,
};

// PIM_AUDIO24
static int bmi_li_volume_put1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];
	volume = 8 - volume;
	if (volume == 0)
		volume = 0xF;	// mute

	// get I2C dapter
	if (bmi_audio[1].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[1].bdev);
	} else if (bmi_audio[3].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[3].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_L1L_LPGA, CODEC_L_PGA(volume) | CODEC_LX_PGA_PU))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_L1R_RPGA, CODEC_L_PGA(volume) | CODEC_LX_PGA_PU))
			return -ENODEV;
	} else {
		return -1;
	}
#endif	// CODEC

	if (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	audio_mixer_control[1].vol_for_input[IP_LINEIN] = uvalue->value.integer.value[0];

	up (&audio_mixer_control[1].sem);

	return 0;
}
static int bmi_li_volume_info1 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 8;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_li_volume_get1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	if  (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	uvalue->value.integer.value[0] = audio_mixer_control[1].vol_for_input[IP_LINEIN];

	up (&audio_mixer_control[1].sem);

	return 0;
}

struct snd_kcontrol_new bmi_control_li_vol1 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Linein Capture Volume/Mute",
	.index = 0x00,
	.info = bmi_li_volume_info1,
	.get = bmi_li_volume_get1,
	.put = bmi_li_volume_put1,
	.private_value = 0xffab3,
};

//
// MIC (L2) Volume control
//
// PIM_AUDIO13
static int bmi_mic_volume_put0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];
	volume = 8 - volume;
	if (volume == 0)
		volume = 0xF;	// mute

	// get I2C dapter
	if (bmi_audio[0].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[0].bdev);
	} else if (bmi_audio[2].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[2].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_L2L_LPGA, CODEC_L_PGA(volume)))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_L2R_RPGA, CODEC_L_PGA(volume)))
			return -ENODEV;
	} else {
		return -1;
	}
#endif	// CODEC

	if (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	audio_mixer_control[0].vol_for_input[IP_MIC] = uvalue->value.integer.value[0];

	up (&audio_mixer_control[0].sem);

	return 0;
}
static int bmi_mic_volume_info0 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 8;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_mic_volume_get0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	if (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	uvalue->value.integer.value[0] = audio_mixer_control[0].vol_for_input[IP_MIC];

	up (&audio_mixer_control[0].sem);

	return 0;
}

struct snd_kcontrol_new bmi_control_mic_vol0 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Mic Capture Volume/Mute",
	.index = 0x00,
	.info = bmi_mic_volume_info0,
	.get = bmi_mic_volume_get0,
	.put = bmi_mic_volume_put0,
	.private_value = 0xffab4,
};

// PIM_AUDIO24
static int bmi_mic_volume_put1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];
	volume = 8 - volume;
	if (volume == 0)
		volume = 0xF;	// mute

	// get I2C dapter
	if (bmi_audio[1].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[1].bdev);
	} else if (bmi_audio[3].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[3].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_L2L_LPGA, CODEC_L_PGA(volume)))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_L2R_RPGA, CODEC_L_PGA(volume)))
			return -ENODEV;
	} else {
		return -1;
	}
#endif	// CODEC

	if (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	audio_mixer_control[1].vol_for_input[IP_MIC] = uvalue->value.integer.value[0];

	up (&audio_mixer_control[1].sem);

	return 0;
}
static int bmi_mic_volume_info1 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 8;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_mic_volume_get1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	if (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	uvalue->value.integer.value[0] = audio_mixer_control[1].vol_for_input[IP_MIC];

	up (&audio_mixer_control[1].sem);

	return 0;
}

struct snd_kcontrol_new bmi_control_mic_vol1 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Mic Capture Volume/Mute",
	.index = 0x00,
	.info = bmi_mic_volume_info1,
	.get = bmi_mic_volume_get1,
	.put = bmi_mic_volume_put1,
	.private_value = 0xffab4,
};

//
// EMIC (L3) Volume control
//
// PIM_AUDIO13
static int bmi_emic_volume_put0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];
	volume = 8 - volume;
	if (volume == 0)
		volume = 0xF;	// mute

	// get I2C dapter
	if (bmi_audio[0].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[0].bdev);
	} else if (bmi_audio[2].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[2].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_M3_LPGA, CODEC_M3_PGA_R(volume)))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_M3_RPGA, CODEC_M3_PGA_R(volume)))
			return -ENODEV;
	} else {
		return -1;
	}
#endif	// CODEC

	if  (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	audio_mixer_control[0].vol_for_input[IP_EMIC] = uvalue->value.integer.value[0];

	up (&audio_mixer_control[0].sem);

	return 0;
}
static int bmi_emic_volume_info0 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 8;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_emic_volume_get0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	if (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	uvalue->value.integer.value[0] = audio_mixer_control[0].vol_for_input[IP_EMIC];

	up (&audio_mixer_control[0].sem);

	return 0;
}

struct snd_kcontrol_new bmi_control_emic_vol0 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Emic Capture Volume/Mute",
	.index = 0x00,
	.info = bmi_emic_volume_info0,
	.get = bmi_emic_volume_get0,
	.put = bmi_emic_volume_put0,
	.private_value = 0xffab5,
};

// PIM_AUDIO24
static int bmi_emic_volume_put1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];
	volume = 8 - volume;
	if (volume == 0)
		volume = 0xF;	// mute

	// get I2C dapter
	if (bmi_audio[1].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[1].bdev);
	} else if (bmi_audio[3].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[3].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_M3_LPGA, CODEC_M3_PGA_R(volume)))
			return -ENODEV;
		if (WriteByte_CODEC (adap, CODEC_M3_RPGA, CODEC_M3_PGA_R(volume)))
			return -ENODEV;
	} else {
		return -1;
	}
#endif	// CODEC

	if  (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	audio_mixer_control[1].vol_for_input[IP_EMIC] = uvalue->value.integer.value[0];

	up (&audio_mixer_control[1].sem);

	return 0;
}
static int bmi_emic_volume_info1 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 8;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_emic_volume_get1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	if  (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	uvalue->value.integer.value[0] = audio_mixer_control[1].vol_for_input[IP_EMIC];

	up (&audio_mixer_control[1].sem);

	return 0;
}

struct snd_kcontrol_new bmi_control_emic_vol1 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Emic Capture Volume/Mute",
	.index = 0x00,
	.info = bmi_emic_volume_info1,
	.get = bmi_emic_volume_get1,
	.put = bmi_emic_volume_put1,
	.private_value = 0xffab5,
};

//
// HEADPHONE (HP[LR]OUT) Volume control
//
// PIM_AUDIO13
static int bmi_hp_volume_put0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];

	// get I2C dapter
	if (bmi_audio[0].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[0].bdev);
	} else if (bmi_audio[2].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[2].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (volume == 0) {
			if (WriteByte_CODEC (adap, CODEC_HPLOUT, 0x0))	// mute
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_HPROUT, 0x0))	// mute
				return -ENODEV;
		} else {
			if (WriteByte_CODEC (adap, CODEC_HPLOUT, CODEC_HPX_LC(volume) 
		    		| CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_HPROUT, CODEC_HPX_LC(volume)
		    		| CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
				return -ENODEV;
		}
	} else {
		return -1;
	}
#endif	// CODEC

	if (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	audio_mixer_control[0].vol_for_output[OP_HEADPHONE] = uvalue->value.integer.value[0];

	up (&audio_mixer_control[0].sem);

	return 0;
}
static int bmi_hp_volume_info0 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 9;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_hp_volume_get0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	if  (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	uvalue->value.integer.value[0] = audio_mixer_control[0].vol_for_output[OP_HEADPHONE];

	up (&audio_mixer_control[0].sem);

	return 0;
}

struct snd_kcontrol_new bmi_control_hp_vol0 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Headphone Playback Volume/Mute",
	.index = 0x00,
	.info = bmi_hp_volume_info0,
	.get = bmi_hp_volume_get0,
	.put = bmi_hp_volume_put0,
	.private_value = 0xffab4,
};

	// PIM_AUDIO24
static int bmi_hp_volume_put1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];

	// get I2C dapter
	if (bmi_audio[1].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[1].bdev);
	} else if (bmi_audio[3].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[3].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (volume == 0) {
			if (WriteByte_CODEC (adap, CODEC_HPLOUT, 0x0))	// mute
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_HPROUT, 0x0))	// mute
				return -ENODEV;
		} else {
			if (WriteByte_CODEC (adap, CODEC_HPLOUT, CODEC_HPX_LC(volume)
		    		| CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_HPROUT, CODEC_HPX_LC(volume)
		    		| CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
				return -ENODEV;
		}
	} else {
		return -1;
	}
#endif	// CODEC

	if  (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	audio_mixer_control[1].vol_for_output[OP_HEADPHONE] = uvalue->value.integer.value[0];

	up (&audio_mixer_control[1].sem);

	return 0;
}
static int bmi_hp_volume_info1 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 9;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_hp_volume_get1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	if  (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	uvalue->value.integer.value[0] = audio_mixer_control[1].vol_for_output[OP_HEADPHONE];

	up (&audio_mixer_control[1].sem);

	return 0;
}

struct snd_kcontrol_new bmi_control_hp_vol1 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Headphone Playback Volume/Mute",
	.index = 0x00,
	.info = bmi_hp_volume_info1,
	.get = bmi_hp_volume_get1,
	.put = bmi_hp_volume_put1,
	.private_value = 0xffab4,
};

//
// Speaker (HP[LR]COM) Volume control
//
// PIM_AUDIO13
static int bmi_spkr_volume_put0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];

	// get I2C dapter
	if (bmi_audio[0].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[0].bdev);
	} else if (bmi_audio[2].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[2].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (volume == 0) {
			if (WriteByte_CODEC (adap, CODEC_HPLCOM, 0x0))
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_HPRCOM, 0x0))
				return -ENODEV;
		} else {
			if (WriteByte_CODEC (adap, CODEC_HPLCOM, CODEC_HPX_LC(volume) 
		    		| CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_HPRCOM, CODEC_HPX_LC(volume) 
		    		| CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
				return -ENODEV;
		}
	} else {
		return -1;
	}
#endif	// CODEC

	if (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	audio_mixer_control[0].vol_for_output[OP_SPEAKER] = uvalue->value.integer.value[0];

	up (&audio_mixer_control[0].sem);

	return 0;
}
static int bmi_spkr_volume_info0 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 9;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_spkr_volume_get0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	if (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	uvalue->value.integer.value[0] = audio_mixer_control[0].vol_for_output[OP_SPEAKER];

	up (&audio_mixer_control[0].sem);

	return 0;
}

struct snd_kcontrol_new bmi_control_spkr_vol0 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Speaker Playback Volume/Mute",
	.index = 0x00,
	.info = bmi_spkr_volume_info0,
	.get = bmi_spkr_volume_get0,
	.put = bmi_spkr_volume_put0,
	.private_value = 0xffab5,
};

// PIM_AUDIO24
static int bmi_spkr_volume_put1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];

	// get I2C dapter
	if (bmi_audio[1].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[1].bdev);
	} else if (bmi_audio[3].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[3].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (volume == 0) {
			if (WriteByte_CODEC (adap, CODEC_HPLCOM, 0x0))
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_HPRCOM, 0x0))
				return -ENODEV;
		} else {
			if (WriteByte_CODEC (adap, CODEC_HPLCOM, CODEC_HPX_LC(volume) 
		    			| CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_HPRCOM, CODEC_HPX_LC(volume) 
		    			| CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
				return -ENODEV;
		}
	} else {
		return -1;
	}
#endif	// CODEC

	if (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	audio_mixer_control[1].vol_for_output[OP_SPEAKER] = uvalue->value.integer.value[0];

	up (&audio_mixer_control[1].sem);

	return 0;
}
static int bmi_spkr_volume_info1 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 9;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_spkr_volume_get1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	if  (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	uvalue->value.integer.value[0] = audio_mixer_control[1].vol_for_output[OP_SPEAKER];

	up (&audio_mixer_control[1].sem);

	return 0;
}

struct snd_kcontrol_new bmi_control_spkr_vol1 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Speaker Playback Volume/Mute",
	.index = 0x00,
	.info = bmi_spkr_volume_info1,
	.get = bmi_spkr_volume_get1,
	.put = bmi_spkr_volume_put1,
	.private_value = 0xffab5,
};

//
// Line out ([LR]LOP) Volume control
//
// PIM_AUDIO13
static int bmi_lo_volume_put0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];

	// get I2C dapter
	if (bmi_audio[0].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[0].bdev);
	} else if (bmi_audio[2].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[2].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (volume == 0) {
			if (WriteByte_CODEC (adap, CODEC_LLOPM, 0x0))
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_RLOPM, 0x0))
				return -ENODEV;
		} else {
			if (WriteByte_CODEC (adap, CODEC_LLOPM, CODEC_HPX_LC(volume) 
		    			| CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_RLOPM, CODEC_HPX_LC(volume) 
		    			| CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
				return -ENODEV;
		}
	} else {
		return -1;
	}
#endif	// CODEC

	if (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	audio_mixer_control[0].vol_for_output[OP_LINEOUT] = uvalue->value.integer.value[0];

	up (&audio_mixer_control[0].sem);

	return 0;
}
static int bmi_lo_volume_info0 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 9;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_lo_volume_get0 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	if (down_interruptible (&audio_mixer_control[0].sem))
		return -EINTR;

	uvalue->value.integer.value[0] = audio_mixer_control[0].vol_for_output[OP_LINEOUT];

	up (&audio_mixer_control[0].sem);

	return 0;
}

struct snd_kcontrol_new bmi_control_lo_vol0 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Lineout Playback Volume/Mute",
	.index = 0x00,
	.info = bmi_lo_volume_info0,
	.get = bmi_lo_volume_get0,
	.put = bmi_lo_volume_put0,
	.private_value = 0xffab6,
};

// PIM_AUDIO24
static int bmi_lo_volume_put1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	struct i2c_adapter *adap = 0;
	int volume;

	// calculate register value
	volume = uvalue->value.integer.value[0];

	// get I2C dapter
	if (bmi_audio[1].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[1].bdev);
	} else if (bmi_audio[3].active) {
		adap = bmi_device_get_i2c_adapter (bmi_audio[3].bdev);
	}

#ifdef CODEC
	// set volume
	if (adap) {
		// write page register
		if (WriteByte_CODEC (adap, 0x0, 0x0))
			return -ENODEV;
		if (volume == 0) {
			if (WriteByte_CODEC (adap, CODEC_LLOPM, 0x0))
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_RLOPM, 0x0))
				return -ENODEV;
		} else {
			if (WriteByte_CODEC (adap, CODEC_LLOPM, CODEC_HPX_LC(volume) 
		    			| CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
				return -ENODEV;
			if (WriteByte_CODEC (adap, CODEC_RLOPM, CODEC_HPX_LC(volume) 
		    			| CODEC_HPX_EN | CODEC_HPX_PD | CODEC_HPX_PC))
				return -ENODEV;
		}
	} else {
		return -1;
	}
#endif	// CODEC

	if (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	audio_mixer_control[1].vol_for_output[OP_LINEOUT] = uvalue->value.integer.value[0];

	up (&audio_mixer_control[1].sem);

	return 0;
}
static int bmi_lo_volume_info1 (struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 9;
	uinfo->value.integer.step = 1;
	return 0;
}

static int bmi_lo_volume_get1 (struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	if (down_interruptible (&audio_mixer_control[1].sem))
		return -EINTR;

	uvalue->value.integer.value[0] = audio_mixer_control[1].vol_for_output[OP_LINEOUT];

	up (&audio_mixer_control[1].sem);

	return 0;
}

struct snd_kcontrol_new bmi_control_lo_vol1 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Lineout Playback Volume/Mute",
	.index = 0x00,
	.info = bmi_lo_volume_info1,
	.get = bmi_lo_volume_get1,
	.put = bmi_lo_volume_put1,
	.private_value = 0xffab6,
};

/*
  * This function registers the control components of ALSA Mixer
  * It is called by ALSA PCM init.
  *
  * @param	card	pointer to the ALSA sound card structure.
  * @param	device	SSI interface
  *
  * @return              0 on success, -ve otherwise.
  */
int bug_alsa_create_ctl (struct snd_card *card, void *p_value, int device)
{
	int rc = 0;

	if (device == 0) {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_pb_vol0, p_value))) < 0)
			return rc;
	} else {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_pb_vol1, p_value))) < 0)
			return rc;
	}

	if (device == 0) {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_li_vol0, p_value))) < 0)
			return rc;
	} else {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_li_vol1, p_value))) < 0)
			return rc;
	}

	if (device == 0) {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_mic_vol0, p_value))) < 0)
			return rc;
	} else {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_mic_vol1, p_value))) < 0)
			return rc;
	}

	if (device == 0) {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_emic_vol0, p_value))) < 0)
			return rc;
	} else {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_emic_vol1, p_value))) < 0)
			return rc;
	}

	if (device == 0) {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_hp_vol0, p_value))) < 0)
			return rc;
	} else {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_hp_vol1, p_value))) < 0)
			return rc;
	}

	if (device == 0) {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_spkr_vol0, p_value))) < 0)
			return rc;
	} else {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_spkr_vol1, p_value))) < 0)
			return rc;
	}

	if (device == 0) {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_lo_vol0, p_value))) < 0)
			return rc;
	} else {
		if ((rc =
			snd_ctl_add (card, snd_ctl_new1 (&bmi_control_lo_vol1, p_value))) < 0)
			return rc;
	}

	return 0;
}

/**************************************************************************
 * Module initialization and termination functions.
 *
 * Note that if this code is compiled into the kernel, then the
 * module_init() function will be called within the device_initcall()
 * group.
 **************************************************************************
 */

/*
 * @name Audio Driver Loading/Unloading Functions
 * These non-exported internal functions are used to support the audio
 * device driver initialization and de-initialization operations.
 */

/*
 * @brief This is the audio device driver initialization function.
 *
 * This function is called by the kernel when this device driver is first
 * loaded.
 */

char const input_name0[MAX_STRG] = "bmi_audio_status_m1";
char const input_name1[MAX_STRG] = "bmi_audio_status_m2";
char const input_name2[MAX_STRG] = "bmi_audio_status_m3";
char const input_name3[MAX_STRG] = "bmi_audio_status_m4";

static int __init bmi_audio_init (void)
{
	int		rc = 0;
	dev_t		dev_id;
	int		idn;
	int		iidn;
	struct snd_card	*card;
	struct snd_card	*card1;

	printk (KERN_INFO "BMI Audio driver loading...\n");

	// alloc char driver with 4 minor numbers
	rc = alloc_chrdev_region (&dev_id, 0, 4, "BMI AUDIO Driver"); 
	if (rc) {
		printk (KERN_ERR "bmi_audio_init: Can't allocate chrdev region\n"); 
		return -ENODEV;
	}
	major = MAJOR(dev_id);

	// Allocate and Register input devices - bmi_audio_status_m[1234]
	for (idn = BMI_AUDIO_M1; idn < BMI_AUDIO_PIM_NUM; idn++) {
		bmi_audio[idn].input_dev = input_allocate_device();
		if (!bmi_audio[idn].input_dev) {
			for (iidn = BMI_AUDIO_M1; iidn < idn; iidn++) 
				input_unregister_device (bmi_audio[iidn].input_dev);
			unregister_chrdev_region (dev_id, 4);
			printk (KERN_ERR "bmi_audio_init: Can't allocate input_dev[%d]\n", idn); 
			return -ENOMEM;
		}
	
		// set up input device
		switch (idn) {
		    case BMI_AUDIO_M1:
			bmi_audio[idn].input_dev->name = input_name0;
			bmi_audio[idn].input_dev->phys = input_name0;
			break;
		    case BMI_AUDIO_M2:
			bmi_audio[idn].input_dev->name = input_name1;
			bmi_audio[idn].input_dev->phys = input_name1;
			break;
		    case BMI_AUDIO_M3:
			bmi_audio[idn].input_dev->name = input_name2;
			bmi_audio[idn].input_dev->phys = input_name2;
			break;
		    case BMI_AUDIO_M4:
			bmi_audio[idn].input_dev->name = input_name3;
			bmi_audio[idn].input_dev->phys = input_name3;
			break;
		}
		bmi_audio[idn].input_dev->id.bustype = BUS_BMI;
		//bmi_audio[idn].input_dev->private = &bmi_audio[idn];
		bmi_audio[idn].input_dev->evbit[BIT_WORD(EV_ABS)] |= BIT_MASK(EV_ABS);
		bmi_audio[idn].input_dev->absbit[BIT_WORD(ABS_MISC)] |= BIT_MASK(ABS_MISC);

		// register input device 
		if (input_register_device (bmi_audio[idn].input_dev)) {
			printk (KERN_ERR "bmi_audio_init() - input_register_device failed.\n");
			for (iidn = BMI_AUDIO_M1; iidn < idn; iidn++) 
				input_unregister_device (bmi_audio[iidn].input_dev);
			unregister_chrdev_region (dev_id, 4);
			return -ENODEV;
		}
	}

		// clear bmi devices and active bits
	bmi_audio[0].bdev = NULL;
	bmi_audio[1].bdev = NULL;
	bmi_audio[2].bdev = NULL;
	bmi_audio[3].bdev = NULL;
	bmi_audio[0].active = 0;
	bmi_audio[1].active = 0;
	bmi_audio[2].active = 0;
	bmi_audio[3].active = 0;

	// allocate private structure
	mxc_audio[0] = kcalloc (1, sizeof (mxc_bmi_audio_t), GFP_KERNEL);
	if (mxc_audio[0] == NULL) {
		for (idn = BMI_AUDIO_M1; idn < BMI_AUDIO_PIM_NUM; idn++) 
			input_unregister_device (bmi_audio[idn].input_dev);
		unregister_chrdev_region (dev_id, 4);
		return -ENODEV;
	}

	// allocate private structure
	mxc_audio[1] = kcalloc (1, sizeof (mxc_bmi_audio_t), GFP_KERNEL);
	if (mxc_audio[1] == NULL) {
		kfree (mxc_audio[0]);
		for (idn = BMI_AUDIO_M1; idn < BMI_AUDIO_PIM_NUM; idn++) 
			input_unregister_device (bmi_audio[idn].input_dev);
		unregister_chrdev_region (dev_id, 4);
		return -ENODEV;
	}
	
	// register the soundcards
	// modules 1 and 3
	card = snd_card_new (1, id13, THIS_MODULE, sizeof (mxc_bmi_audio_t));
	if (card == NULL) {
		kfree (mxc_audio[1]);
		kfree (mxc_audio[0]);
		for (idn = BMI_AUDIO_M1; idn < BMI_AUDIO_PIM_NUM; idn++) 
			input_unregister_device (bmi_audio[idn].input_dev);
		unregister_chrdev_region (dev_id, 4);
		return -ENODEV;
	}

	card->private_data = (void *) mxc_audio[0];
	card->private_free = snd_mxc_audio_free;

	// register pcm
	mxc_audio[0]->card = card;
	if ((rc = snd_card_mxc_audio_pcm (mxc_audio[0], 0)) < 0) {
		snd_card_free (card);
		kfree (mxc_audio[1]);
		kfree (mxc_audio[0]);
		for (idn = BMI_AUDIO_M1; idn < BMI_AUDIO_PIM_NUM; idn++) 
			input_unregister_device (bmi_audio[idn].input_dev);
		unregister_chrdev_region (dev_id, 4);
		return -ENODEV;
	}

	// register mixer
	if (0 == bug_alsa_create_ctl (card, (void *) &audio_mixer_control[0], 0))
		printk (KERN_INFO "Control ALSA component registered\n");

	spin_lock_init (&(mxc_audio[0]->s[0].dma_lock));
	spin_lock_init (&(mxc_audio[0]->s[1].dma_lock));

	strcpy (card->driver, "PIM_AUDIO13");
	strcpy (card->shortname, "PIM13-audio");
	sprintf (card->longname, "PIM13 Freescale MX31");

	// register sound card
	if ((rc = snd_card_register (card)) == 0) {
		PRINTK(KERN_INFO "MXC PIM13 audio support initialized\n");
	} else {
		snd_card_free (card);
		kfree (mxc_audio[1]);
		kfree (mxc_audio[0]);
		for (idn = BMI_AUDIO_M1; idn < BMI_AUDIO_PIM_NUM; idn++) 
			input_unregister_device (bmi_audio[idn].input_dev);
		unregister_chrdev_region (dev_id, 4);
		return -ENODEV;
	}

	// modules 2 and 4
	card1 = snd_card_new (2, id24, THIS_MODULE, sizeof (mxc_bmi_audio_t));
	if (card1 == NULL) {
		snd_card_free (card);
		kfree (mxc_audio[1]);
		kfree (mxc_audio[0]);
		for (idn = BMI_AUDIO_M1; idn < BMI_AUDIO_PIM_NUM; idn++) 
			input_unregister_device (bmi_audio[idn].input_dev);
		unregister_chrdev_region (dev_id, 4);
		return -ENODEV;
	}

	card1->private_data = (void *) mxc_audio[1];
	card1->private_free = snd_mxc_audio_free;

	// register pcm
	mxc_audio[1]->card = card1;
	if ((rc = snd_card_mxc_audio_pcm (mxc_audio[1], 1)) < 0) {
		snd_card_free (card1);
		snd_card_free (card);
		kfree (mxc_audio[1]);
		kfree (mxc_audio[0]);
		for (idn = BMI_AUDIO_M1; idn < BMI_AUDIO_PIM_NUM; idn++) 
			input_unregister_device (bmi_audio[idn].input_dev);
		unregister_chrdev_region (dev_id, 4);
		return -ENODEV;
	}

	// register mixer
	if (0 == bug_alsa_create_ctl (card1, (void *) &audio_mixer_control[1], 1))
		printk (KERN_INFO "Control ALSA component registered\n");

	spin_lock_init (&(mxc_audio[1]->s[0].dma_lock));
	spin_lock_init (&(mxc_audio[1]->s[1].dma_lock));

	strcpy (card1->driver, "PIM_AUDIO24");
	strcpy (card1->shortname, "PIM24-audio");
	sprintf (card1->longname, "PIM24 Freescale MX31");

	// register sound card
	if ((rc = snd_card_register (card1)) == 0) {
		PRINTK(KERN_INFO "MXC PIM24 audio support initialized\n");
	} else {
		snd_card_free (card1);
		snd_card_free (card);
		kfree (mxc_audio[1]);
		kfree (mxc_audio[0]);
		for (idn = BMI_AUDIO_M1; idn < BMI_AUDIO_PIM_NUM; idn++) 
			input_unregister_device (bmi_audio[idn].input_dev);
		unregister_chrdev_region (dev_id, 4);
	}

	// register with BMI
	rc = bmi_register_driver (&bmi_audio_driver); 
	if (rc) {
		printk (KERN_ERR "bmi_audio.c: Can't register bmi_audio_driver\n");
		snd_card_free (card1);
		snd_card_free (card);
		kfree (mxc_audio[1]);
		kfree (mxc_audio[0]);
		for (idn = BMI_AUDIO_M1; idn < BMI_AUDIO_PIM_NUM; idn++) 
			input_unregister_device (bmi_audio[idn].input_dev);
		unregister_chrdev_region (dev_id, 4);
		return rc;
	}

	// turn on I2S transceiver
	bmi_activate_audio_ports();
	
	//configure DAM and SSI
	configure_dam_bmi_master (0);
	configure_dam_bmi_master (1);
	configure_ssi_rx (0);
	configure_ssi_rx (1);
	configure_ssi_tx (0);
	configure_ssi_tx (1);

	printk (KERN_INFO "bmi_audio.c: BMI_AUDIO Driver v%s \n", BMIAUDIO_VERSION);
	if(fcc_test)
		printk (KERN_INFO "bmi_audio.c: FCC Test mode enabled\n");
	if(output_ints)
		printk (KERN_INFO "bmi_audio.c: Output Jack Interrupts enabled\n");
	return 0;
}

/*
 * @brief This is the audio device driver de-initialization function.
 *
 * This function is called by the kernel when this device driver is about
 * to be unloaded.
 */
static void __exit bmi_audio_exit (void)
{
	dev_t dev_id;
	int idn;

	printk (KERN_INFO "BMI Audio driver unloading...\n");

	// delete scheduled work
	flush_scheduled_work ();

	// remove bmi functionality
	bmi_unregister_driver (&bmi_audio_driver);

	// free sound cards
	snd_card_free (mxc_audio[0]->card);
	snd_card_free (mxc_audio[1]->card);

	// free data structures
	kfree (mxc_audio[0]);
	kfree (mxc_audio[1]);

	// remove input devices
	for (idn = BMI_AUDIO_M1; idn < BMI_AUDIO_PIM_NUM; idn++) 
		input_unregister_device (bmi_audio[idn].input_dev);

	// remove control cdev
	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);

	// turn off I2S transceiver
	bmi_inactivate_audio_ports();
	
	printk (KERN_INFO "BMI Audio driver unloaded.\n");
	return;
}

/*
 * Module entry points and description information.
 */

module_init (bmi_audio_init);
module_exit (bmi_audio_exit);

module_param(fcc_test, ushort, S_IRUGO);
MODULE_PARM_DESC(fcc_test, "FCC Test code enable");

module_param(output_ints, ushort, S_IRUGO);
MODULE_PARM_DESC(fcc_test, "Output Jack Interrupts enable");

MODULE_DESCRIPTION("BMI driver for ALSA");
MODULE_AUTHOR("EnCADIS Design, Inc. <p.giacomini@encadis.com>");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("PIM_AUDIO13");
MODULE_SUPPORTED_DEVICE("PIM_AUDIO24");
MODULE_SUPPORTED_DEVICE("bmi_audio_ctrl_m[1234]");
MODULE_SUPPORTED_DEVICE("bmi_audio_status_m[1234]");

