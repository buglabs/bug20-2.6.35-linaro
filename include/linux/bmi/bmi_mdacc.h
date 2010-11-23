/*-----------------------------------------------------------------------------
 *
 * File:     include/linux/bmi/bug_mdacc.h 
 *
 *-----------------------------------------------------------------------------
 * This file contains information needed by application programs that use the 
 * Bug Motion Detector Accelerometer (MDACC) Plug-In Module.
 *
 * The Bug Motion Detector Accelerometer (MDACC) Plug-In Module is a circuit 
 * board that contains the following devices:
 *
 *     a motion sensor, 
 *     a 3-axis accelerometer
 *     a micro-controller.
 *     1 Red LED
 *     1 Green LED
 *
 * The micro-controller behaves as an SPI-slave device. The host controls  
 * the operation of the micro-controller by issuing sequences of SPI messages. 
 * The micro-controller periodically samples the motion sensor and the 
 * accelerometer. The micro-controller generates an interrupts to the host 
 * processor. The micro-controller provides data to the host in response
 * to received SPI messages.
 * 
 * Application software can the MDACC Plug-in modules using the following 
 * device drivers:
 *
 *    BMI MDACC Control Driver                                                                                                      
 *    BMI MDACC Motion Detector Driver                                                                                              
 *    BMI MDACC Accelerometer Driver  
 *
 * These drivers allow for independent operation of MDACC peripheral devices.
 *
 * This file contains the interface definition for all 3 device drivers.
 *
 * ---------------------------------------------------------------------------
 *
 * Default Device Names:
 *
 * The following device nodes are created for each MDACC card present in the
 * system.
 *
 *   /dev/bmi_mdacc_ctl_mX   where X = 1,2,3,4 (bmi connector number)
 *   /dev/bmi_mdacc_mot_mX   where X = 1,2,3,4 (bmi connector number) 
 *   /dev/bmi_mdacc_acc_mX   where X = 1,2,3,4 (bmi connector number) 
 *
 * If the MDACC is not present in a given slot, the corresponding device nodes
 * are not created.
 *
 *----------------------------------------------------------------------------
 *
 *     BMI MDACC Control Driver 
 *
 *----------------------------------------------------------------------------
 *
 *  This character driver provides access to the Red and Green LEDs via  
 *  via the ioctl() system call. 
 * 
 *  Supported system calls: open(), close(), ioctl().
 *
 *  The following IOCTL commands are defined for this driver. 
 *
 *       BMI_MDACC_CTL_RED_LED_OFF
 *       BMI_MDACC_CTL_RED_LED_ON
 *       BMI_MDACC_CTL_GREEN_LED_OFF
 *       BMI_MDACC_CTL_GREEN_LED_ON
 *       BMI_MDACC_CTL_SUSPEND
 *       BMI_MDACC_CTL_RESUME
 *  
 *  Note that the 3rd argument to the ioctl system call are not used by the 
 *  ioctl commands listed above.
 *----------------------------------------------------------------------------
 */

/*----------------------------------------------------------------------------
 *
 *     BMI MDACC Motion Detector Driver
 *
 *----------------------------------------------------------------------------
 *
 * This character driver provides access to the motion sensor via the SPI
 * interface. This driver enforces single-open and stop-on-close behaviors.  
 *
 * 
 * Supported system calls: open(), close(), ioctl(). read(), select().
 *
 * BMI MDACC Motion Detector ioctl() interface
 *--------------------------------------------
 *
 * The following IOCTL commands are defined for this driver. 
 *
 *      BMI_MDACC_MOTION_DETECTOR_GET_STATUS 
 *      BMI_MDACC_MOTION_DETECTOR_RUN
 *      BMI_MDACC_MOTION_DETECTOR_STOP
 *
 *
 * The BMI_MDACC_MOTION_DETECTOR_RUN command sends an SPI message to the 
 * microcontroller to enable sampling of the motion detector status pin.
 * This command does not use the 3rd parameter to the ioctl system call.
 *
 * The BMI_MDACC_MOTION_DETECTOR_STOP command sends an SPI message to the 
 * microcontroller to halt sampling of the motion detector status pin.
 * This command does not use the 3rd parameter to the ioctl system call.
 * 
 * The BMI_MDACC_MOTION_DETECTOR_GET_STATUS command gets the motion detector
 * status byte that is maintained by the motion detector driver. 
 * The third argument to the ioctl system call should be the address of a 
 * the receive buffer that is 1 byte in size.
 * 
 * Motion Detect Status Bit Descriptions
 * ---------------------------------------
 *
 * The Motion Detect Status byte is returned by the system calls to the 
 * MDACC Motion Detector driver:
 *
 *       ioctl(BMI_MDACC_MOTION_GET_STATUS)
 *       read()
 *
 * The following bits are defined in the status byte.
 *
 * BMI_MOTION_DETECT_STATUS    
 * 
 * This bit is the present status of the the motion detector status pin.
 * A value of 1 indicates that motion is being detected.  
 * A value of 0 indicates that motion is not being detected.
 * 
 *
 * BMI_MOTION_DETECT_LATCHED_STATUS
 *
 * This bit is the latched status of the motion sensor. This bit is set to 1
 * when the BMI_MOTION_DETECT_STATUS bit changes from 0 to 1. This bit will 
 * be cleared as the result of an "ioctl(BMI_MDACC_MOTION_GET_STATUS)" or a 
 * read() system call.  
 *
 * BMI_MOTION_DETECT_DELTA
 *
 * This bit indicates that the motion detector status has changed from 1 to 0
 * or has changed from 0 to 1. This bit will be cleared as the result of an 
 * "ioctl(BMI_MDACC_MOTION_GET_STATUS)" or read() system calls. 
 * 
 * 
 * BMI_MOTION_DETECT_ENABLED
 *
 * This bits is the state of the motion detector sampling and status reporting
 * mechanism. A value of 1 indicates that the motion detector is enabled. A 
 * value of 0 indicates that the motion detector is disabled.
 *
 *
 * Motion Detect read() system call
 * --------------------------------
 *
 * The read() call for this driver allows the application program to read the
 * motion detector status only when the status has changed.
 * 
 * Prior to issuing a read() to this driver, the application must enabled the
 * motion detector using the "ioctl(BMI_MDACC_MOTION_DETECTOR_RUN)" command.
 * 
 * read parameters: 
 *
 *    buffer:   status byte destination address. 
 *    size:     1
 *
 * Motion Detect blocking read() behavior
 * --------------------------------------
 *
 * If the motion detector status HAS NOT changed, then the driver will sleep
 * waiting for the motion detect status to change.
 *
 * If the driver is awoken by a Motion Detect status change interrupt, the 
 * underlying hardware will be accessed (for a second time) and the motion 
 * detect status will be updated. 
 *  
 * The status data byte will be copied to the user-supplied buffer. The 
 * following bits will then be cleared in the motion detect status byte:
 *
 *     BMI_MOTION_DETECT_LATCHED_STATUS
 *     BMI_MOTION_DETECT_DELTA
 * 
 * The driver will then be marked as "not-ready-to-read".
 *
 * If the the driver is awoken by a signal, the driver will return failure (-1)
 * and errno will be set to ERESTARTSYS. 
 * 
 *  
 * Motion Detect non-blocking read() behavior
 * ------------------------------------------
 * If the motion detector status HAS NOT changed prior to the non-blocking 
 * read() system call, then the driver will return failure (-1) and errno will 
 * be set to EAGAIN.
 * 
 * If the motion detector status HAS changed prior to the non-blocking read() 
 * system call, the underlying hardware will be accessed and the motion detect 
 * status will be updated. The status data byte will be copied to the user
 * supplied buffer. The following bits will then be cleared in the motion 
 * detect status byte: 
 *
 *     BMI_MOTION_DETECT_LATCHED_STATUS
 *     BMI_MOTION_DETECT_DELTA
 * 
 * The driver will then be marked as "not-ready-to-read".
 *
 *
 * Motion Detect select() system call
 * -----------------------------------
 *
 * This driver supports select() for read only. Select for write and
 * exception is not supported.
 *
 * When a Motion Detect interrupt occurs, the file descriptor corresponding
 * to the Motion Detector driver will be marked as "ready for read".
 *
 * ---------------------------------------------------------------------------
 */


/*----------------------------------------------------------------------------
 *
 *     BMI MDACC Accelerometer Driver
 *
 *----------------------------------------------------------------------------
 *
 * This character driver provides access to the accelerometer data via an SPI
 * interface. This driver enforces single-open and stop-on-close behaviors.
 *
 * Supported system calls: open(), close(), ioctl(). read(), select().
 *
 *
 * MDACC Accelerometer Driver Configuration
 * -----------------------------------------
 *
 * The micro-controller on the MDACC Plug-In module uses an internal 10 bit A/D 
 * converter to sample the 3 analog output channels of the accelerometer 
 * device. The analog channels are sampled periodically at a rate that can be
 * configured by an application program. When a set of 3 channel samples has 
 * been acquired, the micro-controller generates an interrupt to the host 
 * processor. The host processor then issues SPI messages to the 
 * micro-controller to obtain the 3 channel sample set. 
 *
 * The MDACC Accelerometer Driver provides a read queue to store sample data 
 * until it can be read by the application program. The size of the read queue
 * and a read-queue "ready" threshold can both be specified by the application
 * program. 
 *
 * 
 * MDACC Accelerometer Driver ioctl() interface
 * --------------------------------------------

 * The following IOCTL commands are defined for this driver. 
 *
 *     BMI_MDACC_ACCELEROMETER_SET_CONFIG
 *     BMI_MDACC_ACCELEROMETER_GET_CONFIG
 *     BMI_MDACC_ACCELEROMETER_RUN
 *     BMI_MDACC_ACCELEROMETER_STOP
 * 
 *
 * BMI_MDACC_ACCELEROMETER_SET_CONFIG 
 *
 * This ioctl command transfers an mdacc_accel_config structure from the 
 * application program to the MDACC Accelerometer Driver. 
 * The third argument to this ioctl system call is the address of the an 
 * mdacc_accel_config structure. 
 * 
 * In the mdacc_accel_config structure, if the delay_mode field is 0,
 * the values of the delay and delay resolution fields are ignored and 
 * and a default delay value of 4 milliseconds is used.
 *
 * BMI_MDACC_ACCELEROMETER_GET_CONFIG 
 *
 * This ioctl command transfers an mdacc_accel_config structure from the 
 * MDACC Accelerometer Drive to the application program. 
 * The third argument to this ioctl system call is the address of the an 
 * mdacc_accel_config structure. 
 *
 *
 * BMI_MDACC_ACCELEROMETER_RUN
 *
 * This ioctl command will enable the accelerometer data aquistion in the MDACC
 * Accelerometer Driver and in the MDACC micro-controller. The behavior of this
 * ioctl command can also be invoked by an  

 *    "ioctl(BMI_MDACC_ACCELEROMETER_SET_CONFIG)" system call with 
 *    "mdacc_accel_config.run = 1".
 * 
 * BMI_MDACC_ACCELEROMETER_STOP
 *
 * This ioctl command will disable the accelerometer data aquistion in the MDACC
 * Accelerometer Driver and in the MDACC micro-controller. The behavior of this
 * ioctl command can also be invoked by an  
 *
 *    "ioctl(BMI_MDACC_ACCELEROMETER_SET_CONFIG)" system call with 
 *    "mdacc_accel_config.run = 0".
 *
 * Note that this behavior is also invoked in the close() system call if the
 * accelerometer had previously been enabled.
 *
 *
 * MDACC Accelerometer Driver read() interface
 * -------------------------------------------
 *
 * The read() call for this driver allows the application program to read 
 * motion detector status only when the status has changed.
 * 
 * Prior to issuing a read() to this driver, the application must enabled the
 * motion detector using the "ioctl(BMI_MDACC_MOTION_DETECTOR_RUN)" command.
 * 
 * read parameters: 
 *
 *     buffer:   address of an array of mdacc_accel_sample structures.
 *     size:     size of the mdacc_accel_sample array in bytes.
 *
 * Accelerometer blocking read() behavior
 * --------------------------------------
 *
 * If the accelerometer read queue DOES NOT contain at least "read-threshold"
 * number of sample set entries, then the driver will sleep.
 *
 * If the the driver is awoken by a signal, the driver will return failure (-1)
 * and errno will be set to ERESTARTSYS. 

 * Otherwise, the requested number of sample sets are removed from the driver 
 * read queue and copied to user space. The number of bytes transfers will be
 * returned to the application.
 *  
 * At the end of the transfer, if the number of read queue entries is below the 
 * read-threshold, the the driver will then be marked as "not-ready-to-read".
 * 
 *  
 * Accelerometer non-blocking read() behavior
 * ------------------------------------------
 * If the accelerometer read queue DOES NOT contain at least "read-threshold"
 * number of sample set entries, then the driver will return failure (-1) and 
 * errno will  be set to EAGAIN.
 *
 * Otherwise, the requested number of sample sets are removed from the driver 
 * read queue and copied to user space. The number of bytes transfers will be
 * returned to the application.
 *  
 * At the end of the transfer, if the number of read queue entries is below the 
 * read-threshold, the the driver will then be marked as "not-ready-to-read".
 * 
 *
 * Accelerometer select() system call
 * -----------------------------------
 *
 * This driver supports select() for read only. Select for write and
 * exception is not supported.
 *
 * When a data arrives and is inserted into the read queue and the number of 
 * queue entries meets or exceeds the read-threshold, the file descriptor 
 * corresponding to the Accelerometer driver will be marked as 
 * "ready for read". 
 * ---------------------------------------------------------------------------
 *
 *  Accelerometer Data Samples
 *  ---------------------------
 *
 *  The accelerometer analog outputs are sampled with a 10 bit A/D converter 
 *  using 2.9V as a reference. 
 *
 *  An accelerometer output of 1.45V corresponds to "0g". 
 *  
 *  The accelerometer outputs are scaled by the sensitivity settings.
 * 
 *  sensitivity   scale factor
 *  ---------------------------
 *  0 = 2.5G,     421 mV/G 
 *  1 = 3.3G,     316 mV/G 
 *  2 = 6.7G,     158 mV/G 
 *  3 = 10G,      105 mV/G 
 *
 *  The following equation converts an A/D sample to a G-Force value.
 *
 *  G-force = ( ((digital sample) * (X mV/bit)) - 1450 mV) / (scale factor )
 *
 * ---------------------------------------------------------------------------
 * 
 * Accelerometer Coordinate System. 
 *
 *
 *          z axis:   perpendicular to PCB.
 *          y axis:   parallel to the long edge of the connector.
 *          x axis:  perpendicular to the short edge of the connector.
 *         
 *           
 *                      Top View                              Side View
 *          
 *          +--------------------------------------+          +-+
 *          |               +x               LEDS  |          | |
 *          |     +------------------------+       |          | +-----+
 *          | -y  |  connector underneath  |  +y   |          |       |
 *          |     +------------------------+       |          | +-----+
 *          |               ____                   |          | |
 *          |             /      \                 |          | |
 *          |            |        |                |      -z  | |  +z
 *          |             \ ____ /                 |          | |
 *          |                                      |          | |
 *          |        motion sensor on top          |          | |
 *          |                                      |          | |
 *          |               -x                     |          | |
 *          +--------------------------------------+          +-+
 *
 * ---------------------------------------------------------------------------
 */
#ifndef LINUX_BMI_BMI_MDACC_H
#define LINUX_BMI_BMI_MDACC_H 

#include <linux/bmi/bmi_ioctl.h>

/* -------------------------
 *
 *  MDACC Control Driver
 *
 *--------------------------
 */
#define BMI_MDACC_CTL_RED_LED_OFF \
      	_IOW(BMI_MDACC_IOCTL, 0, char)     	// Turn off red LED

#define BMI_MDACC_CTL_RED_LED_ON \
       	_IOW(BMI_MDACC_IOCTL, 1, char)     	// Turn on red LED

#define BMI_MDACC_CTL_GREEN_LED_OFF \
	_IOW(BMI_MDACC_IOCTL, 2, char)     	// Turn off green LED

#define BMI_MDACC_CTL_GREEN_LED_ON \
     	_IOW(BMI_MDACC_IOCTL, 3, char)     	// Turn on green LED

#define BMI_MDACC_CTL_SUSPEND \
     	_IOW(BMI_MDACC_IOCTL, 4, char)     	// Turn on green LED

#define BMI_MDACC_CTL_RESUME \
     	_IOW(BMI_MDACC_IOCTL, 5, char)     	// Turn on green LED


/* -------------------------------
 *
 *  MDACC Motion Detector Driver
 *
 *--------------------------------
 */

/* Status Byte Bit Definitions */

#define BMI_MOTION_DETECT_STATUS         (1<<3)
#define BMI_MOTION_DETECT_LATCHED_STATUS (1<<2)
#define BMI_MOTION_DETECT_DELTA          (1<<1)
#define BMI_MOTION_DETECT_ENABLED        (1<<0)

/* Ioctl Commands */
	
#define BMI_MDACC_MOTION_DETECTOR_GET_STATUS \
		_IOR (BMI_MDACC_IOCTL, 4, char)

#define BMI_MDACC_MOTION_DETECTOR_RUN \
		_IOW (BMI_MDACC_IOCTL, 5, char)
		
#define BMI_MDACC_MOTION_DETECTOR_STOP \
		_IOW (BMI_MDACC_IOCTL, 6, char)


/* -------------------------------
 *
 *  MDACC Accelerometer Driver
 *
 *--------------------------------
 */
struct mdacc_accel_sample {

	unsigned short adc_0; //accelerometer channel Z, 10 bit, left justified 
                              //referenced to VCC = 2.9V

	unsigned short adc_1; //accelerometer channel Y, 10 bit, left justified. 
                              //referenced to VCC = 2.9V.

	unsigned short adc_2; //accelerometer channel X, 10 bit, left justified. 
                              //referenced to VCC = 2.9 V.
};


struct mdacc_accel_config {

	int read_queue_size;	// number of 6-byte sample sets.

	int read_queue_threshold; // number of 6-byte sample sets to queue 
				  // before ready.
	
	unsigned short delay;   // timer ticks between the start of 2 
				// sucessive sample sets. 

	unsigned char  delay_resolution; // timer tick resolution 
					 // 1 =    1 usec, 
					 // 2 =    8 usec, 
					 // 3 =   64 usec, 
					 // 4 =  256 usec,  
					 // 5 = 1024 usec

	unsigned char delay_mode;	//0 = default delay = 5 millisecond,
					//    ignore delay and delay_resolution
					//1 = configured delay

	unsigned char run;		//0 = sampling disabled
					//1 = sampling enabled

	unsigned char sensitivity;	// 0 = 2.5G, 421 mV/G 
					// 1 = 3.3G, 316 mV/G
					// 2 = 6.7G, 158 mV/G
					// 3 = 10G,  105 mV/G

};



#define BMI_MDACC_ACCELEROMETER_SET_CONFIG \
		_IOW (BMI_MDACC_IOCTL, 7, struct mdacc_accel_config)

#define BMI_MDACC_ACCELEROMETER_GET_CONFIG \
		_IOR (BMI_MDACC_IOCTL, 8, struct mdacc_accel_config)


#define BMI_MDACC_ACCELEROMETER_RUN \
		_IOW (BMI_MDACC_IOCTL, 9, char)
		
#define BMI_MDACC_ACCELEROMETER_STOP \
		_IOW (BMI_MDACC_IOCTL, 10, char)

#define BMI_MDACC_LAST_USED (10)
#endif
