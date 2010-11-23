/*
 * File:         include/linux/bmi/bmi_vh.h
 * Author:       Peter Giacomini <p.giacomini@encadis.com>
 *
 * 		This is the application header file for the BMI bus voh Hippel plug-in
 * 		module on the MX31 BUG platform.
 */

#ifndef BMI_VH_H
#define BMI_VH_H

#include <linux/bmi/bmi_ioctl.h>

// GPIO
#define VH_GPIO_RED_LED		3	// default to input
#define VH_GPIO_GREEN_LED	2	// default to input
#define VH_GPIO_1		1	// default to input
#define VH_GPIO_0		0	// default to input

#define VH_GPIO_LED_ON		0
#define VH_GPIO_LED_OFF		1

// I2C
// I2C Slave Addresses
#define BMI_IOX_I2C_ADDRESS	0x71	// 7-bit address
#define VH_RDAC_I2C_ADDRESS	0x18	// 7-bit address
#define VH_ADC_I2C_ADDRESS	0x24	// 7-bit address
#define VH_DAC_I2C_ADDRESS	0x58	// 7-bit address

// I2C IOX register addresses
#define IOX_INPUT_REG		0x0
#define IOX_OUTPUT_REG		0x1
#define IOX_POLARITY_REG	0x2
#define IOX_CONTROL		0x3

#define	VH_IOX_USB_FLG_N	7	// Input - H=normal, L=fault
#define	VH_IOX_USB_VEN		6	// output - H=power on, L=power off
#define	VH_IOX_B5		5	// set to output driven high to prevent interrupts
#define	VH_IOX_B4		4	// set to output driven high to prevent interrupts
#define	VH_IOX_B3		3	// set to output driven high to prevent interrupts
#define	VH_IOX_B2		2	// set to output driven high to prevent interrupts
#define	VH_IOX_B1		1	// set to output driven high to prevent interrupts
#define	VH_IOX_B0		0	// set to output driven high to prevent interrupts

// programmable LDO digital resistor
#define VH_RD_CMD_RDAC		0x00	// RDAC interface
#define VH_RD_CMD_EE		0x20	// EEPROM interface
	#define VH_TOL_HA	0x1E	// Tolerance MSB
	#define VH_TOL_LA	0x1F	// Tolerance LSB
#define VH_RD_CMD_WP		0x40	// EEPROM write protect
#define VH_RD_CMD_NOP		0x80	// NOP
#define VH_RD_CMD_ETOR		0xA0	// EEPROM -> RDAC
#define VH_RD_CMD_RTOE		0xC0	// RDAC -> EEPROM

// ADC
#define VH_ADC_W1_EN		0xA0			// Word 1 Enable
#define VH_ADC_W1_CH01		0x00			// diff - 0, 1
#define VH_ADC_W1_CH23		0x01			// diff - 2, 3
#define VH_ADC_W1_CH10		0x08			// diff - 1, 0
#define VH_ADC_W1_CH32		0x09			// diff - 3, 2
#define VH_ADC_W1_CH0		0x10			// single-ended - 0
#define VH_ADC_W1_CH1		0x18			// single-ended - 1
#define VH_ADC_W1_CH2		0x11			// single-ended - 2
#define VH_ADC_W1_CH3		0x19			// single-ended - 3
#define VH_ADC_W2_EN		0x80			// Word 2 Enable
#define VH_ADC_W2_IM		0x40			// internal temp
#define VH_ADC_W2_F(x)		(((x) % 0x3) << 4)	// rejection mode
#define VH_ADC_W2_SPD		0x08			// speed 2X
#define VH_ADC_W2_G(x)		((x) % 0x7)		// gain

struct vh_adc_wr {	// see the datasheet
	unsigned char w1;	// VH_ADC_W1_*
	unsigned char w2;	// VH_ADC_W2_*
};

// DAC
#define VH_DAC_W1_UA		0x00	// update DAC A output
#define VH_DAC_W1_UB		0x10	// update DAC B output
#define VH_DAC_W1_LA		0x40	// load DAC A input
#define VH_DAC_W1_LB		0x50	// load DAC B input
#define VH_DAC_W1_ALLA		0x80	// load DAC A input, update all outputs
#define VH_DAC_W1_ALLB		0x90	// load DAC B input, update all outputs
#define VH_DAC_W1_ALL		0xC0	// load all inputs, update all outputs
#define VH_DAC_W1_ALLI		0xD0	// load all inputs
#define VH_DAC_W1_UALL		0xE0	// update all - don't send data
#define VH_DAC_W1_EC		0xF0	// Extended command
	#define	VH_DAC_BCH	0x0C	// both channel A & B
	#define	VH_DAC_CHB	0x08	// channel B
	#define	VH_DAC_CHA	0x04	// channel A
	#define	VH_DAC_PD100K	0x03	// power down - 100K pull down
	#define	VH_DAC_PD1K	0x02	// power down - 1K pull down
	#define	VH_DAC_PDF	0x01	// power down - float
	#define	VH_DAC_PU	0x00	// power up
#define VH_DAC_W1_RDA		0xF1	// Read A
#define VH_DAC_W1_RDB		0xF2	// Read B

struct vh_dac_wr {
	unsigned char w1;	// cmd | d[7:3]
	unsigned char w2;	// (d[3:0] << 4) || (VH_DAC_CH* | VH_DAC_P*)
};

// SPI
#define BUF_MAX_SIZE	(20)

// SPI transfer structure
struct spi_xfer {
	unsigned char addr;
	unsigned char data[2];
} spi_xfer;

// von hippel driver ioctl definitions
#define BMI_VH_RLEDOFF		_IOW(BMI_VH_IOCTL, 0x1, unsigned int)		// Turn off red LED
#define BMI_VH_RLEDON		_IOW(BMI_VH_IOCTL, 0x2, unsigned int)		// Turn on red LED
#define BMI_VH_GLEDOFF		_IOW(BMI_VH_IOCTL, 0x3, unsigned int)		// Turn off green LED
#define BMI_VH_GLEDON		_IOW(BMI_VH_IOCTL, 0x4, unsigned int)		// Turn on green LED
#define BMI_VH_GETSTAT		_IOR(BMI_VH_IOCTL, 0x5, unsigned int *)		// READ IOX register
#define BMI_VH_MKGPIO_OUT	_IOW(BMI_VH_IOCTL, 0x6, unsigned int)		// make a GPIO bit an output
#define BMI_VH_MKGPIO_IN	_IOW(BMI_VH_IOCTL, 0x7, unsigned int)		// make a GPIO bit an input
#define BMI_VH_SETGPIO		_IOW(BMI_VH_IOCTL, 0x8, unsigned int)		// set a GPIO output to 1
#define BMI_VH_CLRGPIO		_IOW(BMI_VH_IOCTL, 0x9, unsigned int)		// set a GPIO output to 0
#define BMI_VH_MKIOX_OUT	_IOW(BMI_VH_IOCTL, 0xa, unsigned int)		// make a IOX bit an output
#define BMI_VH_MKIOX_IN		_IOW(BMI_VH_IOCTL, 0xb, unsigned int)		// make a IOX bit an input
#define BMI_VH_SETIOX		_IOW(BMI_VH_IOCTL, 0xc, unsigned int)		// set a IOX output to 1
#define BMI_VH_CLRIOX		_IOW(BMI_VH_IOCTL, 0xd, unsigned int)		// set a IOX output to 0
#define BMI_VH_SETRDAC		_IOW(BMI_VH_IOCTL, 0xe, unsigned int)		// set LDO RDAC resistance
#define BMI_VH_RDRDAC		_IOW(BMI_VH_IOCTL, 0xf, unsigned int *)		// read LDO RDAC resistance
#define BMI_VH_ADCWR		_IOW(BMI_VH_IOCTL, 0x10, struct vh_adc_wr *)	// write ADC
#define BMI_VH_ADCRD		_IOW(BMI_VH_IOCTL, 0x11, unsigned int *)	// read ADC
#define BMI_VH_DACWR		_IOW(BMI_VH_IOCTL, 0x12, struct vh_dac_wr *)	// write DAC
#define BMI_VH_DACRD		_IOW(BMI_VH_IOCTL, 0x13, unsigned int *)	// read DAC
#define BMI_VH_READ_SPI		_IOR(BMI_VH_IOCTL, 0x14, struct spi_xfer *)	// read SPI - requires SPI EEPROM
#define BMI_VH_WRITE_SPI	_IOR(BMI_VH_IOCTL, 0x15, struct spi_xfer *)	// write SPI - requires SPI EEPROM
#define BMI_VH_SUSPEND		_IOR(BMI_VH_IOCTL, 0x16, unsigned int)
#define BMI_VH_RESUME		_IOR(BMI_VH_IOCTL, 0x17, unsigned int)

#endif	/* BMI_VH_H */

