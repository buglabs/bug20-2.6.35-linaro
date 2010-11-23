/*
 * Copyright 2007 EnCADIS Design, Inc. All Rights Reserved.
 * Copyright 2007 Bug-Labs, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef _BMI_CONTROL_H
#define _BMI_CONTROL_H

#define BMI_M1		(0x0)
#define BMI_M2		(0x1)
#define BMI_M3		(0x2)
#define BMI_M4		(0x3)
#define BMI_GPIO_IN	(0x0)
#define BMI_GPIO_OUT	(0x1)
#define BMI_GPIO_ON	(0x1)
#define BMI_GPIO_OFF	(0x0)

/*!
 * This function configures the UART function for the IOMUX pins.
 *
 * @param  port         a UART port number (0-5)
 * @param  no_irda      configure UART port for IRDA
 */
void bmi_gpio_uart_active(int port, int no_irda);

/*!
 * This function configures the UART function in the BMI.
 *
 * @param  port         a UART port number (0-5)
 */
void bmi_uart_active(int port);

/*!
 * This function configures the UART function in the BMI.
 *
 * @param  port         a UART port number (0-5)
 */
void bmi_uart_inactive(int port);

/*!
 * Setup GPIO for a CSPI device to be active
 *
 * @param  cspi_mod         an CSPI device
 */
void bmi_gpio_spi_active(int cspi_mod);

/*!
 * Setup BMI for a CSPI device to be active
 *
 * @param  cspi_mod         an CSPI device
 */
void bmi_spi_active(int cspi_mod);

/*!
 * Setup BMI for a CSPI device to be inactive
 *
 * @param  cspi_mod         an CSPI device
 */
void bmi_spi_inactive(int cspi_mod);

/*!
 * Setup GPIO for an I2C device to be active
 *
 * @param  i2c_num         an I2C device
 */
void bmi_gpio_i2c_active(int i2c_num);

/*!
 * Setup BMI for an I2C device to be active
 */
void bmi_i2c_active(void);

/*!
 * Setup BMI for an I2C device to be inactive
 */
void bmi_i2c_inactive(void);

/*
 * Setup GPIO for an I2S device to be active
 */
void bmi_gpio_activate_audio_ports(void);

/*!
 * Setup CPLD for an I2S device to be active
 */
void bmi_activate_audio_ports(void);

/*!
 * Setup CPLD for an I2S device to be inactive
 */
void bmi_inactivate_audio_ports(void);

/*!
 * Setup GPIO for the plug-in module LCD interface to be active
 */
void bmi_gpio_lcd_active(void);

/*!
 * Setup BMI for plug-in module LCD to be active
 *
 * @param  port LCD serializer (0 or 1)
 * @param  pllc LCD serializer PLL divisor (0-7)
 * @param  mode LCD serializer bus mode (LCD_MODE_I80 or LCD_MODE_M68)
 *
 */
void bmi_lcd_active(int port, int pllc, int mode);

/*!
 * Setup BMI for plug-in module LCD chip select to be active
 *
 * @param  cs LCD chip select (LCD_MxCS x = 1,2,3,4)
 *
 */
void bmi_lcd_cs_active(int cs);

/*!
 * Setup BMI for plug-in module LCD to be inactive
 *
 * @param  port LCD serializer (0 or 1)
 *
 */
void bmi_lcd_inactive(int port);

/*!
 * Setup BMI for plug-in module LCD chip select to be inactive
 *
 * @param  cs LCD chip select (LCD_MxCS x = 1,2,3,4)
 *
 */
void bmi_lcd_cs_inactive(int cs);

/*!
 * Setup pins for SLCD to be active
 *
 */
void bmi_slcd_gpio_config(void);

/*!
 * Setup GPIO for sensor to be active
 *
 */
void bmi_gpio_sensor_active(void);

/*!
 * Setup BMI for sensor to be active
 *
 * @param rclk_r pixclk edge (CAM_CLK_RISE or CAM_CLK_FALL)
 */
void bmi_sensor_active(int rclk_r);

/*!
 * Setup BMI for sensor to be inactive
 */
void bmi_sensor_inactive(void);

/*!
 * read BMI for sensor lock status
 *
 * 	@return	camera serializer lock status (1 == locked)
 */
int bmi_sensor_lock_status(void);

/*
 * USB Host 2 GPIO config
 *
 * @return 0
 */
int bmi_gpio_usbh2_active(void);

/*
 * USB Host 2 BMI config
 *
 * @param mtt - number of MTT's enabled in hub (USB_HUB_1_MTT or USB_HUB_4_MTT)
 */
void bmi_usbh2_active(int mtt);

/*
 * USB Host 2 BMI config inactive
 */
void bmi_usbh2_inactive(void);

/*
 * configure BMI Module GPIO direction
 *
 * @param module	plug-in module (BMI, x= 1,2,3,4)
 * @param bit		GPIO bit (0-3)
 * @param dir		GPIO bit (BMI_GPIO_IN or BMI_GPIO_OUT)
 */
void bmi_set_module_gpio_dir(int module, int bit, int dir);

/*
 * read BMI GPIO Direction register
 *
 * @param module	plug-in module (BMI_Mx, x= 1,2,3,4)
 * @return 			module GPIO direction (4 LSB)
 */
int bmi_read_gpio_direction_reg(int module);

/*
 * set BMI Module GPIO data
 *
 * @param module	plug-in module (BMI_Mx, x= 1,2,3,4)
 * @param bit		GPIO bit (0-3)
 * @param value		GPIO bit (0x0 or 0x1)
 */
void bmi_set_module_gpio_data(int module, int bit, int value);

/*
 * read BMI GPIO Data register
 *
 * @param module	plug-in module (BMI_Mx, x= 1,2,3,4)
 * @return 			module GPIO data (4 LSB)
 */
int bmi_read_gpio_data_reg(int module);

/*
 * set BMI Module battery enable
 *
 * @param module	plug-in module (BMI_Mx, x= 1,2,3,4)
 */
void bmi_set_module_battery_enable(int module);

/*
 * set BMI Module battery disable
 *
 * @param module	plug-in module (BMI_Mx, x= 1,2,3,4)
 */
void bmi_set_module_battery_disable(int module);

/*
 * read BMI module battery status
 *
 * @param module	plug-in module (BMI_Mx, x= 1,2,3,4)
 * @return 			state of module battery status bit
 */
int bmi_read_module_battery_status(int module);

/*
 * set BMI interrupt enable
 *
 * @param interrupt interrupt (INT_BUGRTC .. INT_M4_PRES) (defined in mx31bug.h)
 */
void bmi_interrupt_enable(int interrupt);

/*
 * set BMI interrupt disable
 *
 * @param interrupt interrupt (INT_BUGRTC .. INT_M4_PRES) (defined in mx31bug.h)
 */
void bmi_interrupt_disable(int interrupt);

/*
 * get BMI interrupt status
 *
 * @param interrupt interrupt (INT_BUGRTC .. INT_M4_PRES) (defined in mx31bug.h)
 * @return	1 if set, 0 otherwise
 */
int bmi_interrupt_status(int interrupt);

/*
 * clear BMI module present interrupt bit
 *
 * @param module	plug-in module (BMI_Mx, x= 1,2,3,4)
 */
void bmi_clear_module_present_interrupt(int module);

/*
 * enable I2C switches in BMI
 */
void bmi_i2c_sw_enable(void);

/*
 * disable I2C switches in BMI
 */
void bmi_i2c_sw_disable(void);

/*
 * read BMI module present status
 *
 * @param module	plug-in module (BMI_Mx, x= 1,2,3,4)
 * @return 		module present status (3 LSB = (OUT, IN, STATE_CHANGED))
 */
int bmi_read_module_present_status(int module);

/*
 *  BMI module present 
 *
 * @param module	plug-in module (BMI_Mx, x= 1,2,3,4)
 * @return 		module present (1 = present, 0 = not present )
 */
int bmi_module_present (struct bmi_device *bdev);


#endif // _BMI_CONTROL_H

