/*
 * header file for Bug Module Interface (BMI) 
 * Bugduino module. This is a module containing an Arduino 
 * which is programmable/usable from the bug linux core
 */

/*    
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations: 
 *  
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */ 
    

#ifndef BMI_BUGDUINO_H
#define BMI_BUGDUINO_H 1


#include <linux/bmi/bmi_ioctl.h>



/* Laws of Nature */
/* --------------------------------------- */
#define BUGDUINO_TRUE 1
#define BUGDUINO_FALSE 0
#define BUGDUINO_HIGH 1
#define BUGDUINO_LOW 0
/* --------------------------------------- */

/* bmi_bugduino Meta-Data */
/* --------------------------------------- */
#define BUGDUINO_VERSION "1.0"
/* --------------------------------------- */

/* UART ports */
/* --------------------------------------- */
#define UART_SLOT_0 "/dev/ttyBMI0"
#define UART_SLOT_1 "/dev/ttyBMI1"
#define UART_SLOT_2 "/dev/ttyBMI2"
#define UART_SLOT_3 "/dev/ttyBMI3"
/* --------------------------------------- */

/* Buffer Sizes */
/* --------------------------------------- */
#define BUGDUINO_INTERRUPT_NAME_SIZE 32
#define BUGDUINO_SPI_BUFFER_SIZE 32
#define BUGDUINO_I2C_MESSAGE_SEND_SIZE 2
/* --------------------------------------- */

/* I2C */
/* --------------------------------------- */
/* address = i2c address of the device (7 MSb  (KEEP IN MIND THAT THE 
	I2C DRIVERS TAKE THE ADRESS IN THE 7 LSb) )
   offset = register to write to on the device
   data = data that is written or read */
struct bugduino_i2c_package {
  unsigned char address;
  unsigned char offset;
  unsigned char data;
};		       
/* --------------------------------------- */

/* IO Expander (IOX) */
/* --------------------------------------- */
/* I2C Address */
/* 7-bit address is stored in the lower 
   7 bits */
#define BUGDUINO_IOX_I2C_ADDRESS (0xE2 >> 1)

/* IOX Message Information */
#define BUGDUINO_IOX_MESSAGE_SIZE 2
#define BUGDUINO_IOX_MESSAGE_ADDRESS 0
#define BUGDUINO_IOX_MESSAGE_DATA 1

/* IOX Register Addresses */
#define BUGDUINO_IOX_REG_INPUT 0x0
#define BUGDUINO_IOX_REG_OUTPUT 0x1
#define BUGDUINO_IOX_REG_POLARITY 0x2
#define BUGDUINO_IOX_REG_CONTROL 0x3

/* IOX Pins */
#define BUGDUINO_IOX_PX( x ) ( 1 << (x) )
#define BUGDUINO_IOX_P7 ( BUGDUINO_IOX_P( 7 ) )
#define BUGDUINO_IOX_P6 ( BUGDUINO_IOX_P( 6 ) )
#define BUGDUINO_IOX_P5 ( BUGDUINO_IOX_P( 5 ) )
#define BUGDUINO_IOX_P4 ( BUGDUINO_IOX_P( 4 ) )
#define BUGDUINO_IOX_P3 ( BUGDUINO_IOX_P( 3 ) )
#define BUGDUINO_IOX_P2 ( BUGDUINO_IOX_P( 2 ) )
#define BUGDUINO_IOX_P1 ( BUGDUINO_IOX_P( 1 ) )
#define BUGDUINO_IOX_P0 ( BUGDUINO_IOX_P( 0 ) )
/* --------------------------------------- */

/* SPI */
/* --------------------------------------- */
#define BUGDUINO_SPI_SPEED 1000000
#define BUGDUINO_SPI_MODE SPI_MODE_2
/* --------------------------------------- */

/* GPIO reset */
/* --------------------------------------- */
#define BUGDUINO_GPIO_RESET_PIN 1
#define BUGDUINO_RESET_ON BUGDUINO_HIGH
#define BUGDUINO_RESET_OFF BUGDUINO_LOW
/* --------------------------------------- */

/* IOCTL definitions */
/* --------------------------------------- */
#define BMI_BUGDUINO_RESET _IOW( BMI_BUGDUINO_IOCTL, 0x1, unsigned int )
#define BMI_BUGDUINO_IOX_CTRL _IOW( BMI_BUGDUINO_IOCTL, 0x2, unsigned char )
#define BMI_BUGDUINO_IOX_WRITE _IOW( BMI_BUGDUINO_IOCTL, 0x3, unsigned char )
#define BMI_BUGDUINO_IOX_READ _IOR( BMI_BUGDUINO_IOCTL, 0x4, unsigned char * )
#define BMI_BUGDUINO_I2C_WRITE _IOW( BMI_BUGDUINO_IOCTL, 0x5, struct BUGDUINO_i2c_package * )
#define BMI_BUGDUINO_I2C_READ _IOR( BMI_BUGDUINO_IOCTL, 0x6, struct BUGDUINO_ioctl_package * )
#define BMI_BUGDUINO_SPI_XFER _IOW( BMI_BUGDUINO_IOCTL, 0x7, unsigned char * )
/* --------------------------------------- */

#endif /* BMI_BUGDUINO_H */
