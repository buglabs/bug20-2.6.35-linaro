/*
 * File:         drivers/bmi/pims/zb/zaccel.h
 * Author:       V. Thavisri <v.thavisri@encadis.com>
 *
 * 		This is the header file for the CC2480 TI on
 * 		ZigBee module. It is derived from the following source
 *
 */


#ifndef _ZACCEL_H
#define _ZACCEL_H

#define RSPS_CMD                     0x40
/* SYS Interface commands */
#define SYS_RESET_REQ_0              0x41
#define SYS_RESET_REQ_1              0x00
#define SYS_RESET_IND                0x4180
#define	SYS_VER_0                    0x21
#define	SYS_VER_1                    0x02
#define SYS_OSAL_NV_READ             0x2108
#define SYS_OSAL_NV_WRITE            0x2109
#define SYS_OSAL_START_TIMER         0x210A
#define SYS_OSAL_TIMER_EXPIRED       0x4181
#define SYS_RANDOM                   0X210C
#define SYS_ADC_READ                 0x210D
#define SYS_GPIO                     0x210E
#define SYS_TEST_RF_0                0x41
#define SYS_TEST_RF_1                0x40
#define SYS_TEST_LOOPBACK_REQ        0x2141
#define SYS_TEST_LOOPBACK_REQ_0      0x21
#define SYS_TEST_LOOPBACK_REQ_1      0x41
#define SYS_TEST_LOOPBACK_RSP        0x6141
#define SYS_RF_POWER_AMP_0           0x21
#define SYS_RF_POWER_AMP_1           0x10
#define SYS_RF_POWER_AMP_RSP         0x6110

/* Configuration Interface Commands */
#define ZB_READ_CONFIG_REQ_0         0x26
#define ZB_READ_CONFIG_REQ_1         0x04
#define ZB_WRITE_CONFIG_REQ_0        0x26
#define ZB_WRITE_CONFIG_REQ_1        0x05
#define ZB_WRITE_CONFIG_RSP          0x6605

/* ZigBee PIM specific commands     */
#define ZB_DEVICE_INFO_CHG_0           0xFF
#define ZB_DEVICE_INFO_CHG_1           0x00

// Simple API Interface
#define ZB_APP_REGISTER_REQ_0        0x26
#define ZB_APP_REGISTER_REQ_1        0x0A
#define ZB_START_REQ_0               0x26
#define ZB_START_REQ_1               0x00
#define ZB_START_CONFIRM             0x4680
#define ZB_PERMIT_JOINING_REQ_0      0x26
#define ZB_PERMIT_JOINING_REQ_1      0x08
#define ZB_BIND_DEVICE_0             0x26
#define ZB_BIND_DEVICE_1             0x01
#define ZB_BIND_CONFIRM              0x4681
#define ZB_ALLOW_BIND_0              0x26
#define ZB_ALLOW_BIND_1              0x02
#define ZB_ALLOW_BIND_CONFIRM        0x4682
#define ZB_SEND_DATA_REQ_0           0x26
#define ZB_SEND_DATA_REQ_1           0x03
#define ZB_SEND_DATA_CONFIRM         0x4683
#define ZB_RCV_DATA_IND              0x4687
#define ZB_GET_DEVICE_INFO_0         0x26
#define ZB_GET_DEVICE_INFO_1         0x06
#define ZB_FIND_DEVICE_REQ_0         0x26
#define ZB_FIND_DEVICE_REQ_1         0x07
#define ZB_FIND_DEVICE_CONFIRM       0X4685

// AF Interface
#define AF_REGISTER_0                0x24
#define AF_REGISTER_1                0x00
#define AF_DATA_REQ_0                0x24
#define AF_DATA_REQ_1                0x01
#define AF_DATA_CONFIRM              0x4480
#define AF_INCOMING_MSG              0x4481

// ZDO Interface
#define ZDO_NWK_ADDR_REQ             0x2500
#define ZDO_IEEE_ADDR_REQ            0x2501
#define ZDO_NODE_DESC_REQ            0x2502
#define ZDO_NODE_DESC_RES            0x2582
#define ZDO_SIMPLE_DESC_REQ          0x2504
#define ZDO_SIMPLE_DESC_RSP          0x4584
#define ZDO_ACTIVE_EP_REQ            0x2505
#define ZDO_ACTIVE_EP_RSP            0x4585
#define ZDO_MATCH_DESC_REQ           0x2506
#define ZDO_MATCH_DESC_RSP           0x4586
#define ZDO_MATCH_DESC_RSP_SENT      0x45C2
#define ZDO_USER_DESC_REQ            0x2508
#define ZDO_USER_DESC_RSP            0x4588
#define ZDO_USER_DESC_SET            0x250B
#define ZDO_USER_DESC_CONF           0x4589
#define ZDO_END_DEVICE_ANNCE         0x250A
#define ZDO_END_DEVICE_ANNCE_IND     0x45C1
#define ZDO_END_DEVICE_BIND_REQ      0x2520
#define ZDO_END_DEVICE_BIND_RES_RSP  0x45A0
#define ZDO_BIND_REQ                 0x2521
#define ZDO_BIND_RSP                 0x45A1
#define ZDO_UNBIND_REQ               0x2522
#define ZDO_UNBIND_RSP               0x45A2
#define ZDO_MGMT_LQI_REQ             0x2531
#define ZDO_MGMT_LQI_RSP             0x45B1
#define ZDO_MGMT_LEAVE_REQ           0x2534
#define ZDO_MGMT_LEAVE_RSP           0x45B4
#define ZDO_MGMT_PERMIT_JOIN_REQ     0x2536
#define ZDO_MGMT_PERMIT_JOIN_RSP     0x45B6
#define ZDO_STATE_CHANGE_IND         0x45C0

// ZCD_NV_STARTUP_OPTION value
#define ZCD_STARTOPT_DEFAULT_CONFIG   0x01
#define ZCD_STARTOPT_DEFAULT_NETWORK  0x02
#define ZCD_STARTOPT_AUTO_START       0x04
#define ZCD_STARTOPT_VALID            0x02    // max valid option value
#define ZCD_STARTOPT_MASK             0x03

// ZCD_NV_LOGICAL_TYPE
#define ZB_COORDINATOR    0x00
#define ZB_ROUTER         0x01
#define ZB_ENDDEVICE      0x02
#define ZB_VALID_DEVICE   0x02
#define ZB_DEVICE_MASK    0x03
#define ZB_INVALID_DEVICE 0xFF

// ZB_GET_DEVICE_INFO parameters
#define ZB_DEVICE_STATE      0x00
#define ZB_DEVICE_IEEE_ADDR  0x01
#define ZB_DEVICE_SHORT_ADDR 0x02
#define ZB_PARENT_SHORT_ADDR 0x03
#define ZB_PARENT_IEEE_ADDR  0x04
#define ZB_DEVICE_CHANNEL    0x05
#define ZB_DEVICE_PANID      0x06
#define ZB_DEVICE_EXT_PANID  0x07
#define ZB_DEVICE_LAST       0x07

/* Z-Accel command return value */
#define Z_SUCCESS            0x00
#define Z_FAILURE            0x01
#define Z_INVALID_PARAM      0x02

#define Z_ALLOW_BIND        0xFF
#define Z_DENY_BIND         0x00

#define Z_PERMIT_JOIN       0xFF
#define Z_DENY_JOIN         0x00

#define Z_BIND_CREATE       0x01
#define Z_BIND_REMOVE       0x00

#define Z_CONFIG_OFFSET     6

#define SYS_RF_PA_MIN       25    /* valid power level 0 - 25 */

// ZB_DEVICE_STATE definition
typedef enum
{
  DEV_HOLD,               // Initialized - not started automatically
  DEV_INIT,               // Initialized - not connected to anything
  DEV_NWK_DISC,           // Discovering PAN's to join
  DEV_NWK_JOINING,        // Joining a PAN
  DEV_NWK_REJOIN,         // ReJoining a PAN, only for end devices
  DEV_END_DEVICE_UNAUTH,  // Joined but not yet authenticated by trust center
  DEV_END_DEVICE,         // Started as device after authentication
  DEV_ROUTER,             // Device joined, authenticated and is a router
  DEV_COORD_STARTING,     // Started as Zigbee Coordinator
  DEV_ZB_COORD,           // Started as Zigbee Coordinator
  DEV_NWK_ORPHAN          // Device has lost information about its parent..
} devStates_t;

// zstate - Z-Accel device state
#define ZACCEL_RESET           0
#define ZACCEL_WAIT_RELEASE    1
#define ZACCEL_RESET_IND       2
#define ZACCEL_START_REQ       3
#define ZACCEL_START_CNF       4
#define ZACCEL_START_FAIL      5
#define ZACCEL_APP_START       6
#define ZACCEL_TERMINATE       7
#define ZACCEL_POLL_DONE       8
#define ZACCEL_UNDEFINE        0xFF

/* periodic timer runs every 50 ms */
#define Z_TIMER                (50*HZ)/1000
#define Z_RESET_TIMEOUT        (20*HZ)
#define Z_CNF_TIMEOUT          (15*HZ)
#define Z_POLL_TIMER           (50*HZ)/1000
#define Z_SRDY_TIMEOUT         (4*HZ)

/* frequency to update zbinfo sysfs = (Z_TIMER * UPDATE_TICKS) */
#define UPDATE_TICKS       20

/* Reset option for ioctl */
#define Z_DONT_RESET           0
#define Z_HW_RESET             1
#define Z_SW_RESET             2

#define ZCMD_BUF       (256 + 3)
#define ZCMD_HEADER      3

struct zaccel_app_id
{
	unsigned char  endpoint;
	unsigned short profile_id;
	unsigned short device_id;
	unsigned char  device_ver;
	unsigned char  unused;
	unsigned char  icmd_num;
	
};
struct zaccel_app_struct
{
	struct zaccel_app_id info;
	unsigned char  commands[1];
};

struct zaccel_version
{
	unsigned char transportRev;
	unsigned char product;
	unsigned char majorRel;
	unsigned char minorRel;
	unsigned char hwRev;
};

struct zaccel_config
{
	unsigned char device;          // ZCD_NV_LOGICAL_TYPE
	unsigned long chanlist;
	unsigned short panid;
};

extern unsigned char zb_device_info_len[];
#define ZDEVICE_INFO_NUM   8

struct zaccel_device
{
	unsigned char state;
	unsigned char device_ieee[8];
	unsigned char device_short[2];
	unsigned char parent_short[2];
	unsigned char parent_ieee[8];
	unsigned char channel;
	unsigned char panid[2];
	unsigned char ext_panid[8];
};

#define ZBIND_NUM_MAX 64
struct zaccel_info
{
	unsigned char lastReset;
	struct zaccel_version ver;
	struct zaccel_device device;

	struct list_head xmt_list;
	wait_queue_head_t wait_queue;
	unsigned short msg_flag;
	unsigned short app_type;
};

#define NO_APP_TYPE         0
#define SAPI_TYPE           1
#define AF_INTERFACE_TYPE   2

#define ZBUF_MAX_SIZE   150

/* struct zaccel_info msg_flag bit definition */
#define START_CNF_BIT    0x0001
#define RESET_IND_BIT    0x0002

/* 
 * XMT_MSG_SIZE = Maximum length of the ZB_SEND_DATA_REQUEST content.
 * 2 bytes Destination, 2 bytes Command ID, 1 byte Handle
 * 1 byte Ack, 1 byte Radius, 1 byte len, up to 84 bytes data.
 */
#define XMT_MSG_SIZE  92

struct zaccel_xmt_msg
{
	struct list_head list;
	unsigned char len;
	unsigned char buf[XMT_MSG_SIZE];
};


#endif // _ZACCEL_H
