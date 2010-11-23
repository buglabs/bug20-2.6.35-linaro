#ifndef BMI_EEPROM_DRIVER_H
#define BMI_EEPROM_DRIVER_H

/*******************************************************************************
 *  Driver description:
 *
 *  This driver provides operations that allow an application program to 
 *  read and write the inventory eeprom on Bug Labs Bug PlugIn peripheral 
 *  hardware modules.
 *
 *  This driver is a character driver.
 *
 *  Supported system calls
 *
 *  This driver supports the following system calls:
 *
 *  open()
 *
 *      During the open() call, only driver initialization and house keeping
 *      are performed. The hardware is not touched.
 *
 *  close()
 *
 *      During the close() system call, only driver house keeping is performed.
 *      The hardware is not touched.
 *
 *  ioctl()
 *
 *      All of the ioctl() calls for this driver take 2 or 3 parameters.
 *      They are:
 *              file descriptor         - obtained from open() call.
 *              ioctl command number    - described below.
 *              void pointer to struct  - ioctl command specific.

 *   ioctl() return values:
 *
 *      On success, all ioctl() calls return zero.
 *
 *      On error, all ioctl() calls return -1 and errno is set appropriatly.
 *      Additional error information may be returned in the ioctl command
 *      structure. See the ioctl command structure declarations for more
 *      information.
 *
 *******************************************************************************
 */
#ifdef __cplusplus
extern "C" {
#endif


/*
 * Include the standard type definitions.
 * The file to include depends on whether or not we are doing a kernel or
 * application build.
 */
#ifdef __KERNEL__
    #include   <linux/types.h>
#else
    #include   <sys/types.h>
    #include   <stdint.h>
#endif /* __KERNEL__ */


#ifdef __cplusplus
}
#endif

//REWORK: Add documentation.

//REWORK: Where should this file live so that applications can #include it ?


/*	bmi_eeprom_request

	offset: 0 - 255
	size:   1- 256
	offset + size must be <= 256

 */
struct bmi_eeprom_request {
	int return_code;
	int xfer_count;
	int size;
	int offset;
	__u8 data[256];
};


/*******************************************************************************
 *  Ioctl type definition:
 *
 *  The ioctl type (magic) number for this driver is BUG_EEPROM_IOC_TYPE
 *
 *******************************************************************************
 */

#define BUG_EEPROM_IOC_TYPE   0xFE

/*******************************************************************************
 *  Ioctl command definitions:
 *
 *  The ioctl calls supported by this driver are:
 *
 *******************************************************************************
 */

#define BUG_EEPROM_READ \
		_IOR (BUG_EEPROM_IOC_TYPE, 0, struct bmi_eeprom_request)
		
#define BUG_EEPROM_WRITE \
		_IOW (BUG_EEPROM_IOC_TYPE, 0, struct bmi_eeprom_request)
		
#endif
