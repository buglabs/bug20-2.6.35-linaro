/*
 * Copyright 2008 EnCADIS Designs, Inc. All Rights Reserved.
 * Copyright 2008 Bug-Labs, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*-----------------------------------------------------------------------------
 *
 *      Part of BMI Motion Detector Accelerometer (MDACC) Kernel Module
 *
 *-----------------------------------------------------------------------------
 */
#ifndef MDACC_AVR_H
#define MDACC_AVR_H

#include <linux/device.h>
#include <linux/spi/spi.h>

struct avr_regs
{
	unsigned char timer_res;	// SPI Register 0 
	unsigned char timer_msb;        // SPI Register 1 
	unsigned char timer_lsb;        // SPI Register 2 
	unsigned char mode;             // SPI Register 3 

	unsigned char status;           // SPI Register 4 

	unsigned char adc0h;            // SPI Register 5  
	unsigned char adc0l;            // SPI Register 6  
	unsigned char adc1h;            // SPI Register 7                    
	unsigned char adc1l;            // SPI Register 8 
	unsigned char adc2h;            // SPI Register 9 
	unsigned char adc2l;            // SPI Register 10
};

int avr_read_adc (struct spi_device *spi, struct avr_regs *regs); 
int avr_read_status (struct spi_device *spi, struct avr_regs *regs);
int avr_read_status_and_adc (struct spi_device *spi, struct avr_regs *regs); 
int avr_read_mode  (struct spi_device *spi, struct avr_regs *regs); 
int avr_read_timer  (struct spi_device *spi, struct avr_regs *regs); 
int avr_read_timer_and_mode  (struct spi_device *spi, struct avr_regs *regs); 
int avr_write_mode (struct spi_device *spi, struct avr_regs *regs); 
int avr_write_timer (struct spi_device *spi, struct avr_regs *regs); 
int avr_write_timer_and_mode (struct spi_device *spi, struct avr_regs *regs); 

#endif


