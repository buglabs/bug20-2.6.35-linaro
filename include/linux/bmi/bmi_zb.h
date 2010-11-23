/*
 * File:    include/linux/bmi/bmi_gps.h
 * Author:  V. Thavisri <v.thavisri@encadis.com
 *
 * This is the application header file for the BMI ZigBee plug-in
 * module on the MX31 BUG platform.
 */

#ifndef BMI_ZBCNTL_H
#define BMI_ZBCNTL_H

#include <linux/bmi/bmi_ioctl.h>
#include <linux/sockios.h>

/* IOCTL commands for BMI ZB driver - char device portion */

#define BMI_ZB_RLEDOFF            _IO(BMI_ZIGBEE_IOCTL, 0x1)
#define BMI_ZB_RLEDON             _IO(BMI_ZIGBEE_IOCTL, 0x2)
#define BMI_ZB_GLEDOFF            _IO(BMI_ZIGBEE_IOCTL, 0x3)
#define BMI_ZB_GLEDON             _IO(BMI_ZIGBEE_IOCTL, 0x4)
#define BMI_ZB_RESET              _IO(BMI_ZIGBEE_IOCTL, 0x5)
#define BMI_ZB_SPI_SIG            _IO(BMI_ZIGBEE_IOCTL, 0x6)
#define BMI_ZB_LOOPBACK           _IO(BMI_ZIGBEE_IOCTL, 0x7)
#define BMI_ZB_STARTREQ           _IO(BMI_ZIGBEE_IOCTL, 0x8)
#define BMI_ZB_UPDATE_STATE       _IO(BMI_ZIGBEE_IOCTL, 0x9)
#define BMI_ZB_SUSPEND            _IO(BMI_ZIGBEE_IOCTL, 0xa)
#define BMI_ZB_RESUME             _IO(BMI_ZIGBEE_IOCTL, 0xb)
#define BMI_ZB_STATE              _IO(BMI_ZIGBEE_IOCTL, 0xc)


/* IOCTL commands for BMI ZB driver - network device portion */

#define SIOCSAPPREGISTER       (SIOCDEVPRIVATE + 1)
#define SIOCSALLOWBIND         (SIOCDEVPRIVATE + 2)
#define SIOCSPERMITJOINING     (SIOCDEVPRIVATE + 3)
#define SIOCGDEVICEINFO        (SIOCDEVPRIVATE + 4)
#define SIOCSRESET             (SIOCDEVPRIVATE + 5)
#define SIOCSBIND              (SIOCDEVPRIVATE + 6)
#define SIOCSZCOMMAND          (SIOCDEVPRIVATE + 7)       
#define SIOCSSTARTREQ          (SIOCDEVPRIVATE + 8)       
#define SIOCSFINDDEVICE        (SIOCDEVPRIVATE + 9)
#define SIOCSAFREGISTER        (SIOCDEVPRIVATE + 10)
#define SIOCSPOWERAMP          (SIOCDEVPRIVATE + 11)
#define SIOCDEBUG              (SIOCDEVPRIVATE + 15)

struct sockaddr_zb
{
	unsigned short z_family;
	int            z_ifindex;
	unsigned char  z_name[15];
	unsigned short z_protocol;
};

/* move this #define to include/linux/socket.h */
#define SOL_ZACCEL 275

#define Z_PACKET_SOCK  0
#define Z_CONTROL_SOCK 1
#define Z_NUM_SOCK     2
#define Z_NO_SOCK      0xFF

/* Device-Specification Configuration Parameters */
#define ZCD_NV_STARTUP_OPTION             0x03
#define ZCD_NV_LOGICAL_TYPE               0x87
#define ZCD_NV_POLL_RATE                  0x24
#define ZCD_NV_QUEUED_POLL_RATE           0x25
#define ZCD_NV_RESPONSE_POLL_RATE         0x26
#define ZCD_NV_POLL_FAILURE_RETRIES       0x29
#define ZCD_NV_INDIRECT_MSG_TIMEOUT       0x2B
#define ZCD_NV_APS_FRAME_RETRIES          0x43
#define ZCD_NV_APS_ACK_WAIT_DURATION      0x44
#define ZCD_NV_BINDING_TIME               0x46
#define ZCD_NV_USERDESC                   0x81

// Network-Specification Configuration Parameters
#define ZCD_NV_PANID                      0x83
#define ZCD_NV_CHANLIST                   0x84
#define ZCD_NV_PRECFGKEY                  0x62
#define ZCD_NV_PRECFGKEYS_ENABLE          0x63
#define ZCD_NV_SECURITY_MODE              0x64
#define ZCD_NV_BCAST_RETRIES              0x2E
#define ZCD_NV_PASSIVE_ACK_TIMEOUT        0x2F
#define ZCD_NV_BCAST_DELIVERY_TIME        0x30
#define ZCD_NV_ROUTE_EXPIRY_TIME          0x2C

#endif /* BMI_ZBCNTL_H */
