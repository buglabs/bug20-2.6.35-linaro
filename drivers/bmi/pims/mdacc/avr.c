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



/*

Cmd Bit Definitions

Cmd bit 7  - R0/W1
Cmd bit 6  - Cnt 2
Cmd bit 5  - Cnt 1
Cmd bit 4  - Cnt 0
Cmd bit 3  - Address 3
Cmd bit 2  - Address 2
Cmd bit 1  - Address 1
Cmd bit 0  - Address 0
*/



#define AVR_CMD_READ_ADC              (0x65) 
#define AVR_CMD_READ_STATUS 	      (0x14)
#define AVR_CMD_READ_STATUS_AND_ADC   (0x74) 
#define AVR_CMD_READ_MODE             (0x13) 
#define AVR_CMD_READ_TIMER            (0x30) 
#define AVR_CMD_READ_TIMER_AND_MODE   (0x40) 
#define AVR_CMD_WRITE_MODE            (0x93) 
#define AVR_CMD_WRITE_TIMER           (0xB0) 
#define AVR_CMD_WRITE_TIMER_AND_MODE  (0xC0) 

int avr_read_adc (struct spi_device *spi, struct avr_regs *regs); 
int avr_read_status (struct spi_device *spi, struct avr_regs *regs);
int avr_read_status_and_adc (struct spi_device *spi, struct avr_regs *regs); 
int avr_read_mode  (struct spi_device *spi, struct avr_regs *regs); 
int avr_read_timer  (struct spi_device *spi, struct avr_regs *regs); 
int avr_read_timer_and_mode  (struct spi_device *spi, struct avr_regs *regs); 
int avr_write_mode (struct spi_device *spi, struct avr_regs *regs); 
int avr_write_timer (struct spi_device *spi, struct avr_regs *regs); 
int avr_write_timer_and_mode (struct spi_device *spi, struct avr_regs *regs); 


int avr_read_adc (struct spi_device *spi, struct avr_regs *regs)
{
	int			err;
	unsigned char		cmd;
	unsigned char		sync;
	unsigned char		dont_care;

	struct spi_transfer	x[7];
	struct spi_message	message;

	cmd = AVR_CMD_READ_ADC;
	sync = 0;

	dont_care = 0;

	spi_message_init(&message);
	memset(x, 0, sizeof x);

	x[0].len = 1;
	x[0].tx_buf = &cmd;
	x[0].rx_buf = &sync;
	x[0].delay_usecs = 400;
	x[0].bits_per_word = 8;
	spi_message_add_tail(&x[0], &message);

	x[1].len = 1;
	x[1].tx_buf = &dont_care;
	x[1].rx_buf = &regs->adc0h;
	x[1].delay_usecs = 400;
	x[1].bits_per_word = 8;
	spi_message_add_tail(&x[1], &message);

	x[2].len = 1;
	x[2].tx_buf = &dont_care;
	x[2].rx_buf = &regs->adc0l;
	x[2].delay_usecs = 400;
	x[2].bits_per_word = 8;
	spi_message_add_tail(&x[2], &message);

	x[3].len = 1;
	x[3].tx_buf = &dont_care;
	x[3].rx_buf = &regs->adc1h;
	x[3].delay_usecs = 400;
	x[3].bits_per_word = 8;
	spi_message_add_tail(&x[3], &message);

	x[4].len = 1;
	x[4].tx_buf = &dont_care;
	x[4].rx_buf = &regs->adc1l;
	x[4].delay_usecs = 400;
	x[4].bits_per_word = 8;
	spi_message_add_tail(&x[4], &message);

	x[5].len = 1;
	x[5].tx_buf = &dont_care;
	x[5].rx_buf = &regs->adc2h;
	x[5].delay_usecs = 400;
	x[5].bits_per_word = 8;
	spi_message_add_tail(&x[5], &message);

	x[6].len = 1;
	x[6].tx_buf = &dont_care;
	x[6].rx_buf = &regs->adc2l;
	x[6].delay_usecs = 400;
	x[6].bits_per_word = 8;
	spi_message_add_tail(&x[6], &message);

	err = spi_sync(spi, &message);

	return err;
}


int avr_read_status (struct spi_device *spi, struct avr_regs *regs)
{
	int			err;
	unsigned char		cmd;
	unsigned char		sync;
	unsigned char 		dont_care;
	struct spi_transfer	x[2];
	struct spi_message	message;

	cmd = AVR_CMD_READ_STATUS;
	sync = 0;

	dont_care = 0;;

	memset(x, 0, sizeof x);
	spi_message_init(&message);

	x[0].len = 1;
	x[0].tx_buf = &cmd;
	x[0].rx_buf = &sync;
	x[0].delay_usecs = 400;
	spi_message_add_tail(&x[0], &message);

	x[1].len = 1;
	x[1].tx_buf = &dont_care;
	x[1].rx_buf = &regs->status;
	x[1].delay_usecs = 400;
	spi_message_add_tail(&x[1], &message);

	err = spi_sync(spi, &message);
	
	return err;
}

int avr_read_status_and_adc (struct spi_device *spi, struct avr_regs *regs)
{
	int			err;
	unsigned char		cmd;
	unsigned char		sync;
	unsigned char 		dont_care;
	struct spi_transfer	x[8];
	struct spi_message	message;

	cmd = AVR_CMD_READ_STATUS_AND_ADC;
	sync = 0;
	dont_care = 0;

	memset(x, 0, sizeof x);
	spi_message_init(&message);

	x[0].len = 1;
	x[0].tx_buf = &cmd;
	x[0].rx_buf = &sync;
	x[0].delay_usecs = 400;
	spi_message_add_tail(&x[0], &message);

	x[1].len = 1;
	x[1].tx_buf = &dont_care;
	x[1].rx_buf = &regs->status;
	x[1].delay_usecs = 400;
	spi_message_add_tail(&x[1], &message);

	x[2].len = 1;
	x[2].tx_buf = &dont_care;
	x[2].rx_buf = &regs->adc0h;
	x[2].delay_usecs = 400;
	spi_message_add_tail(&x[2], &message);

	x[3].len = 1;
	x[3].tx_buf = &dont_care;
	x[3].rx_buf = &regs->adc0l;
	x[3].delay_usecs = 400;
	spi_message_add_tail(&x[3], &message);

	x[4].len = 1;
	x[4].tx_buf = &dont_care;
	x[4].rx_buf = &regs->adc1h;
	x[4].delay_usecs = 400;
	spi_message_add_tail(&x[4], &message);

	x[5].len = 1;
	x[5].tx_buf = &dont_care;
	x[5].rx_buf = &regs->adc1l;
	x[5].delay_usecs = 400;
	spi_message_add_tail(&x[5], &message);

	x[6].len = 1;
	x[6].tx_buf = &dont_care;
	x[6].rx_buf = &regs->adc2h;
	x[6].delay_usecs = 400;
	spi_message_add_tail(&x[6], &message);

	x[7].len = 1;
	x[7].tx_buf = &dont_care;
	x[7].rx_buf = &regs->adc2l;
	x[7].delay_usecs = 400;
	spi_message_add_tail(&x[7], &message);

	err = spi_sync(spi, &message);

	return err;
}

int avr_read_mode  (struct spi_device *spi, struct avr_regs *regs)
{
	int			err;
	unsigned char		cmd;
	unsigned char		sync;
	unsigned char		dont_care;

	struct spi_transfer	x[2];
	struct spi_message	message;


	cmd = AVR_CMD_READ_MODE;
	sync = 0;

	dont_care = 0;
	memset(x, 0, sizeof x);
	spi_message_init(&message);

	x[0].len = 1;
	x[0].tx_buf = &cmd;
	x[0].rx_buf = &sync;
	x[0].delay_usecs = 400;
	spi_message_add_tail(&x[0], &message);

	x[1].len = 1;
	x[1].tx_buf = &dont_care;
	x[1].rx_buf = &regs->mode;
	x[1].delay_usecs = 400;
	spi_message_add_tail(&x[1], &message);

	err = spi_sync(spi, &message);

	return err;
}

int avr_read_timer  (struct spi_device *spi, struct avr_regs *regs)
{
	int			err;
	unsigned char		cmd;
	unsigned char		sync;
	unsigned char 		dont_care;
	struct spi_transfer	x[4];
	struct spi_message	message;


	cmd = AVR_CMD_READ_TIMER;
	sync = 0;
	dont_care = 0;

	memset(x, 0, sizeof x);
	spi_message_init(&message);        

	x[0].len = 1;
	x[0].tx_buf = &cmd;
	x[0].rx_buf = &sync;
	x[0].delay_usecs = 400;
	spi_message_add_tail(&x[0], &message);

	x[1].len = 1;
	x[1].tx_buf = &dont_care;
	x[1].rx_buf = &regs->timer_res;
	x[1].delay_usecs = 400;
	spi_message_add_tail(&x[1], &message);

	x[2].len = 1;
	x[2].tx_buf = &dont_care;
	x[2].rx_buf = &regs->timer_msb;
	x[2].delay_usecs = 400;
	spi_message_add_tail(&x[2], &message);

	x[3].len = 1;
	x[3].tx_buf = &dont_care;
	x[3].rx_buf = &regs->timer_lsb;
	x[3].delay_usecs = 400;
	spi_message_add_tail(&x[3], &message);

	err = spi_sync(spi, &message);

	return err;
}
int avr_read_timer_and_mode  (struct spi_device *spi, struct avr_regs *regs)
{
	int			err;
	unsigned char		cmd;
	unsigned char		sync;
	unsigned char		dont_care;
	struct spi_transfer	x[5];
	struct spi_message	message;


	cmd = AVR_CMD_READ_TIMER_AND_MODE;
	sync = 0;
	dont_care = 0;

	memset(x, 0, sizeof x);
	spi_message_init(&message);

	x[0].len = 1;
	x[0].tx_buf = &cmd;
	x[0].rx_buf = &sync;
	x[0].delay_usecs = 400;
	spi_message_add_tail(&x[0], &message);

	x[1].len = 1;
	x[1].tx_buf = &dont_care;
	x[1].rx_buf = &regs->timer_res;
	x[1].delay_usecs = 400;
	spi_message_add_tail(&x[1], &message);

	x[2].len = 1;
	x[2].tx_buf = &dont_care;
	x[2].rx_buf = &regs->timer_msb;
	x[2].delay_usecs = 400;
	spi_message_add_tail(&x[2], &message);

	x[3].len = 1;
	x[3].tx_buf = &dont_care;
	x[3].rx_buf = &regs->timer_lsb;
	x[3].delay_usecs = 400;
	spi_message_add_tail(&x[3], &message);

	x[4].len = 1;
	x[4].tx_buf = &dont_care;
	x[4].rx_buf = &regs->mode;
	x[4].delay_usecs = 400;
	spi_message_add_tail(&x[4], &message);

	err = spi_sync(spi, &message);

	return err;
}

int avr_write_mode (struct spi_device *spi, struct avr_regs *regs)
{
	int			err;
	unsigned char		cmd;
	unsigned char		sync;
	struct spi_transfer	x[2];
	struct spi_message	message;

	cmd = AVR_CMD_WRITE_MODE;
	sync = 0;

	memset(x, 0, sizeof x);
	spi_message_init(&message);

	x[0].len = 1;
	x[0].tx_buf = &cmd;
	x[0].rx_buf = &sync;
	x[0].delay_usecs = 400;
	spi_message_add_tail(&x[0], &message);

	x[1].len = 1;
	x[1].tx_buf = &regs->mode;
	x[1].rx_buf = 0;
	x[1].delay_usecs = 400;
	spi_message_add_tail(&x[1], &message);

	err = spi_sync(spi, &message);

	return err;
}

int avr_write_timer (struct spi_device *spi, struct avr_regs *regs)
{
	int			err;
	unsigned char		cmd;
	unsigned char		sync;
	struct spi_transfer	x[4];
	struct spi_message	message;

	cmd = AVR_CMD_WRITE_TIMER;
	sync = 0;

	memset(x, 0, sizeof x);
	spi_message_init(&message);

	x[0].len = 1;
	x[0].tx_buf = &cmd;
	x[0].rx_buf = &sync;
	x[0].delay_usecs = 400;
	spi_message_add_tail(&x[0], &message);

	x[1].len = 1;
	x[1].tx_buf = &regs->timer_res;
	x[1].rx_buf = 0;
	x[1].delay_usecs = 400;
	spi_message_add_tail(&x[1], &message);

	x[2].len = 1;
	x[2].tx_buf = &regs->timer_msb;
	x[2].rx_buf = 0;
	x[2].delay_usecs = 400;
	spi_message_add_tail(&x[2], &message);

	x[3].len = 1;
	x[3].tx_buf = &regs->timer_lsb;
	x[3].rx_buf = 0;
	x[3].delay_usecs = 400;
	spi_message_add_tail(&x[3], &message);

	err = spi_sync(spi, &message);

	return err;
}

int avr_write_timer_and_mode (struct spi_device *spi, struct avr_regs *regs)
{
	int			err;
	unsigned char		cmd;
	unsigned char		sync;
	struct spi_transfer	x[5];
	struct spi_message	message;


	cmd = AVR_CMD_WRITE_TIMER_AND_MODE;
	sync = 0;

	memset(x, 0, sizeof x);
	spi_message_init(&message);

	x[0].len = 1;
	x[0].tx_buf = &cmd;
	x[0].rx_buf = &sync;
	x[0].delay_usecs = 400;
	spi_message_add_tail(&x[0], &message);

	x[1].len = 1;
	x[1].tx_buf = &regs->timer_res;
	x[1].rx_buf = 0;
	x[1].delay_usecs = 400;
	spi_message_add_tail(&x[1], &message);

	x[2].len = 1;
	x[2].tx_buf = &regs->timer_msb;
	x[2].rx_buf = 0;
	x[2].delay_usecs = 400;
	spi_message_add_tail(&x[2], &message);

	x[3].len = 1;
	x[3].tx_buf = &regs->timer_lsb;
	x[3].rx_buf = 0;
	x[3].delay_usecs = 400;
	spi_message_add_tail(&x[3], &message);

	x[4].len = 1;
	x[4].tx_buf = &regs->mode;
	x[4].rx_buf = 0;
	x[4].delay_usecs = 400;
	spi_message_add_tail(&x[4], &message);

	err = spi_sync(spi, &message);

	return err;
}

