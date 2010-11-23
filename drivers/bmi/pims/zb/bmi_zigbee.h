/*
 * File:
 * Author:   V. Thavisri <v.thavisri@encadis.com>
 *
 *           Header file for the ZB module on the MX31 BUG platform.
 */
#ifndef BMI_ZB_H
#define BMI_ZB_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/bmi.h>
#include <linux/bmi/bmi-control.h>
#include <linux/bmi/bmi_ioctl.h>
#include <linux/bmi/bmi_zb.h>
#include "bmi_zaccel.h"

#define BMI_ZB_VERSION   "1.0"
#define VE_DEBUG

// GPIO
#define ZB_GPIO_RED_LED   3    
#define ZB_GPIO_GREEN_LED 2   
#define ZB_GPIO_MRDY      1  
#define ZB_GPIO_SS        0 

#define ZB_GPIO_LED_ON    0
#define ZB_GPIO_LED_OFF   1

#define BMI_IOX_I2C_ADDRESS 0x71
// I2C IOX register addressess
#define IOX_INPUT_REG           0x0
#define IOX_OUTPUT_REG          0x1
#define IOX_POLARITY_REG        0x2
#define IOX_CONTROL             0x3

#define ZB_IOX_RST              0x1
#define ZB_IOX_MRDY             0x2
#define ZB_IOX_SRDY             0x4

// SPI parameters
#define ZB_SPI_SPEED            2000000
#define ZB_SPI_BPW              8

// SPI state
// check the state when SRDY is asserted to determine which
// SPI transaction the host wants to perform.
#define ZB_SPI_POLL            0
#define ZB_SPI_WAIT_SRDY_L     1
#define ZB_SPI_SRDY_LOW        2
#define ZB_SPI_WAIT_SRDY_H     3
#define ZB_SPI_SRDY_HIGH       4
#define ZB_SPI_TIME2POLL       5
#define ZB_SPI_SRDY_EXP        6
#define ZB_SPI_WAIT_SRDY_Q     7

#define ZB_INT_POLL            0
#define ZB_INT_WAIT_SRDY       1
#define ZB_INT_SRDY_LOW        2

#define SPI_POLL_TYPE  0
#define SPI_REQ_TYPE   1

#define ZB_RESET      0
#define ZB_RELEASE    1

#define SPI_BUF_MAX_SIZE        ZBUF_MAX_SIZE
#define SPI_MSG_HEADER_SIZE     3

#define SPI_CMD_POLL            0x00
#define SPI_CMD_SREQ            0x20
#define SPI_CMD_AREQ            0x40
#define SPI_CMD_TYPE_MASK       (SPI_CMD_SREQ | SPI_CMD_AREQ)

#define SPI_CHK_SRDY_JIFFIES    1
#define SPI_CHK_SRDY_TIME       4000
#define SPI_SRDY_L_TIMEOUT      (HZ>>1)
#define SPI_SRDY_H_TIMEOUT      (HZ>>1)

// Zigbee network private information
struct net_zb
{
	struct net_device *dev;
	struct bmi_zb *zb;
	struct net_device_stats stats;
	unsigned char net_open;
	unsigned char socket[Z_NUM_SOCK];
};

struct bmi_zb
{
	struct bmi_device    *bdev;          
	struct cdev          cdev;            
	struct net_device    *netdev;
	struct device  *class_dev;       
	struct i2c_adapter    *adap;
	struct spi_device    *spi;          /* SPI device */

	struct work_struct   spi_work;      /* work to process Poll req */
	struct work_struct   xmt_work;      /* work to xmt to device */
	struct work_struct   sreq_work;     /* work to process sreq */
	struct work_struct   state_work;    /* work to check device state */

	struct delayed_work  srdy_work;     /* work to chk for SRDY */

	unsigned long start_time;
	unsigned long delay;

	struct workqueue_struct *spi_wq;
	struct workqueue_struct *srdy_wq;

	wait_queue_head_t    srdy_queue;
	unsigned char        srdy_state;
	unsigned char        enable;
	unsigned char        int_depth;

	wait_queue_head_t    sreq_queue;
	unsigned char        *sreq_buf;
	unsigned short       sreq_len;
	int                  sreq_ret;

	wait_queue_head_t    delay_queue;

	char                 int_name[20];  /* interrupt name */
	int                  slot;          /* base unit slot number */
	int                  open_flag;     /* single open flag */
	int                  irq;
	struct zaccel_info   z_info;
};

extern int  zb_Reset(struct bmi_zb *zb, unsigned char state);
extern void zb_StartDevice(struct bmi_zb *zb, unsigned char option);
extern int zb_StartRequest(struct bmi_zb *zb);
extern void zb_spi_poll(void *arg);
extern int zaccel_spi_req(struct bmi_zb *zb, unsigned char *data, unsigned short buf_len);
extern int zaccel_SRDY_poll(struct bmi_zb *zb, unsigned char polarity);
extern int zb_GetDeviceInfo(struct bmi_zb *zb, unsigned char param, unsigned char *buf);
extern unsigned char zb_WriteConfiguration(struct bmi_zb *zb, unsigned char configId, unsigned char len, unsigned char *data);
extern int zb_ReadConfiguration(struct bmi_zb *zb, unsigned char configId, 
                        unsigned char *buf);
extern int zb_zcommand(struct bmi_zb *zb, unsigned short cmd, unsigned char len,
                        unsigned char *buf);
extern void zb_SoftReset(struct bmi_zb *zb);
extern unsigned char zb_AppRegisterRequest(struct bmi_zb *zb, unsigned char len,
	                unsigned char *app_info);
extern unsigned char zb_PermitJoiningRequest(struct bmi_zb *zb, 
                        unsigned char *dest, unsigned char timeout);
extern void zb_AllowBind(struct bmi_zb *zb, unsigned char timeout);
extern void zb_BindRequest(struct bmi_zb *zb, unsigned char create, 
                        unsigned char *bind_info);
extern int zb_SendDataRequest(struct bmi_zb *zb, unsigned char *buf,
                        unsigned char len);
extern void zb_FindDeviceRequest(struct bmi_zb *zb, unsigned char *searchKey);
extern unsigned char zb_AFRegisterRequest(struct bmi_zb *zb, unsigned char len, unsigned char *app_info);
extern int zb_sysRFpowerAmp(struct bmi_zb *zb, unsigned char pa, unsigned char power);

extern void zaccel_cmd_proc(struct bmi_zb *zb,unsigned char *buf);
extern int zb_rx(struct net_device *dev, unsigned char *buf, 
                        unsigned char len, unsigned short type);
extern int z_sock_init(void);
extern int zdev_setopt(struct net_device *dev,int cmd, int len, 
	                unsigned char *buf);
extern int zdev_getopt(struct net_device *dev, int cmd, int *len, 
                        unsigned char *buf);

extern void zb_net_setup(struct net_device *dev);
extern void zaccel_getdev(struct bmi_zb *zb);

extern void z_sock_exit(void);
extern int z_sock_init(void);
extern void zb_create_sysfs(struct net_device *net);
extern void zb_remove_sysfs(struct net_device *net);
extern int init_zaccel(struct bmi_zb *zb);
extern void remove_zaccel(struct bmi_zb *zb);
extern int zb_ReadSRDY(struct bmi_zb *zb, int print);
extern void zb_ReadMRDY(struct bmi_zb *zb);
extern void zaccel_spi_poll(struct bmi_zb *zb);
extern int zb_SysTestLoopback(struct bmi_zb *zb);
extern void  zb_sysVersion(struct bmi_zb *zb);
extern int zaccel_get_chanlist(struct bmi_zb *zb);



#endif // BMI_ZB_H
