/*
 *
 *      bmi_zaccel.c
 *
 *      API to Zaccel device
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 * 
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/bmi/bmi_zb.h>
#include "bmi_zigbee.h"
#include "bmi_zaccel.h"

int reset_count = 0;

int zb_ReadConfiguration(struct bmi_zb *zb, unsigned char configId, unsigned char *buf)
{
	unsigned char data[128];
	unsigned char i, len;

	data[0] = 1;
	data[1] = ZB_READ_CONFIG_REQ_0;
	data[2] = ZB_READ_CONFIG_REQ_1;
	data[3] = configId;

	// Write to the device
	if(zaccel_spi_req(zb, data, 128) < 0)
	{
		return -1;
	}

	/* 
 	 * Return value has format shown below.
 	 * 	data[0] = Zaccetl return status
 	 * 	data[1] = configID
 	 * 	data[2] = len
 	 * 	data[3-130] = value
 	 */
	len = data[0];

	/* 
 	 * truncate output to 128 bytes if it is longer than 128 byte.
 	 * Zaccel data sheet indicates 0-128 bytes value.
 	 * Add 3 byte header to the total length
 	 */
	if(len > 128)
		len = 128;

	for(i = 0; i < (len + 3); i++)
	{
		buf[i] = data[i];
	}
	
	return(len);
}

unsigned char zb_WriteConfiguration(struct bmi_zb *zb, unsigned char configId, unsigned char len, unsigned char *value)
{
	unsigned char data[128];
	int rc;

	if(len > 128)
	{
		/* Len exceeds the max size */
		rc = -1;
	}
	data[0] = 2 + len;
	data[1] = ZB_WRITE_CONFIG_REQ_0;
	data[2] = ZB_WRITE_CONFIG_REQ_1;
	data[3] = configId;
	data[4] = len;

	memcpy((unsigned char *)(&data[5]),value,len);

	/* Write to the device. */
	zaccel_spi_req(zb, data, 128);

	/* read return status */
	rc = data[3];

	return rc;
}

void zb_SoftReset(struct bmi_zb *zb)
{
	unsigned char data[8];

	data[0] = 1;
	data[1] = SYS_RESET_REQ_0;
	data[2] = SYS_RESET_REQ_1;
	data[3] = 0;

	/* Write to the device. */
	zaccel_spi_req(zb, data, 8);
}

void zb_sysVersion(struct bmi_zb *zb)
{
	unsigned char data[16];

	/* SYS_VERSION command returns 5 bytes of data */

	data[0] = 0;
	data[1] = SYS_VER_0;
	data[2] = SYS_VER_1;

	/* Write to the device. */
	zaccel_spi_req(zb, data, 16);
}

int zb_sysRFpowerAmp(struct bmi_zb *zb, unsigned char pa, unsigned char power)
{
	unsigned char data[32];
	int rc = 0;

	if(power > SYS_RF_PA_MIN)
	{
		return -EINVAL;
	}

	data[0] = 2;
	data[1] = SYS_RF_POWER_AMP_0;
	data[2] = SYS_RF_POWER_AMP_1;
	data[3] = pa;
	data[4] = power;

	/* Write to the device */
	zaccel_spi_req(zb, data, 32);

	/* compare the return values */
	if((data[3] != pa) || (data[4] != power))
	{
		rc = -1;
	}

	return rc;
}

unsigned char zb_device_info_len[] = {1, 8, 2, 2, 8, 1, 2, 8};

int zb_GetDeviceInfo(struct bmi_zb *zb, unsigned char param,unsigned char *buf)
{
	unsigned char len;
	unsigned char data[32];
	unsigned char i;

	if(param > ZB_DEVICE_LAST)
	{
		printk(KERN_WARNING "zb: invalid device info %d\n",param);
		return -EINVAL;
	}

	data[0] = 1;
	data[1] = ZB_GET_DEVICE_INFO_0;
	data[2] = ZB_GET_DEVICE_INFO_1;
	data[3] = param;

	/* Write to the device. */
	zaccel_spi_req(zb, data, 32);

	/* data points to the value byte */
	len = zb_device_info_len[param];
	for(i = 0; i < len; i++)
	{
		buf[i] = data[i+4];
	}

	return(len);
}

void zb_FindDeviceRequest(struct bmi_zb *zb, unsigned char *searchKey)
{
	unsigned char data[16];
	
	data[0] = 8;
	data[1] = ZB_FIND_DEVICE_REQ_0;
	data[2] = ZB_FIND_DEVICE_REQ_1;

	memcpy(&data[3],searchKey,8);

	/* Write to the device. */
	zaccel_spi_req(zb, data, 16);
}

void zb_StartDevice(struct bmi_zb *zb, unsigned char option)
{
	zb_WriteConfiguration(zb,ZCD_NV_STARTUP_OPTION,1,&option);
	zb_Reset(zb,ZB_RESET);   
	udelay(10);
	zb_Reset(zb,ZB_RELEASE); 
}

int zb_StartRequest(struct bmi_zb *zb)
{
	unsigned char data[8];

	data[0] = 0;
	data[1] = ZB_START_REQ_0;
	data[2] = ZB_START_REQ_1;

	zb->z_info.msg_flag &= ~START_CNF_BIT;

	/* Write to the device */
	zaccel_spi_req(zb, data, 8);

	return 0;
}

unsigned char zb_PermitJoiningRequest(struct bmi_zb *zb, unsigned char *dest, unsigned char timeout)
{
	unsigned char data[8];
	unsigned char rc;

	data[0] = 3;
	data[1] = ZB_PERMIT_JOINING_REQ_0;
	data[2] = ZB_PERMIT_JOINING_REQ_1;
	data[3] = dest[0];           /* LSB of short address */
	data[4] = dest[1];           /* MSB of short address */
	data[5] = timeout;

	/* Write to the device */
	zaccel_spi_req(zb, data, 8);

	rc = data[3];

	if(rc != Z_SUCCESS)
	{
		printk(KERN_WARNING "zb_PermitJoiningReques invalid 0x%x\n",rc);
	}

	return rc;
}

void zb_AllowBind(struct bmi_zb *zb, unsigned char timeout)
{
	unsigned char data[8];

	data[0] = 1;
	data[1] = ZB_ALLOW_BIND_0;
	data[2] = ZB_ALLOW_BIND_1;
	data[3] = timeout;

	/* Write to the device */
	zaccel_spi_req(zb, data, 8);
}

unsigned char zb_AppRegisterRequest(struct bmi_zb *zb, unsigned char len, unsigned char *app_info)
{
	unsigned char *data;
	unsigned char rc;

	data = kmalloc((len + ZCMD_HEADER), GFP_KERNEL);

	data[0] = len;
	data[1] = ZB_APP_REGISTER_REQ_0;
	data[2] = ZB_APP_REGISTER_REQ_1;

	memcpy(&data[3],app_info,len);

	zaccel_spi_req(zb,data, (len + ZCMD_HEADER));

	/* Read return status */
	rc = data[3];
	if(rc == Z_SUCCESS)
	{
		zb->z_info.app_type = SAPI_TYPE;
	}

	kfree(data);

	return rc;
	
}

unsigned char zb_AFRegisterRequest(struct bmi_zb *zb, unsigned char len, unsigned char *app_info)
{
	unsigned char *data;
	unsigned char rc;

	data = kmalloc((len + ZCMD_HEADER), GFP_KERNEL);

	data[0] = len;
	data[1] = AF_REGISTER_0;
	data[2] = AF_REGISTER_1;

	memcpy(&data[3],app_info,len);

	zaccel_spi_req(zb,data, (len + ZCMD_HEADER));

	/* Read return status */
	rc = data[3];
	if(rc == Z_SUCCESS)
	{
		zb->z_info.app_type = AF_INTERFACE_TYPE;
	}

	kfree(data);

	return rc;
}

void zb_BindRequest(struct bmi_zb *zb, unsigned char create, unsigned char *bind_info)
{
	unsigned char data[16];

	data[0] = 0x0B;
	data[1] = ZB_BIND_DEVICE_0;
	data[2] = ZB_BIND_DEVICE_1;
	data[3] = create;
	
	memcpy(&data[4], bind_info, 10);

	zaccel_spi_req(zb,data, 16);
}

int zb_SendDataRequest(struct bmi_zb *zb, unsigned char *buf, unsigned char len)
{
	unsigned char *data;

	/*
	 * The user sends data to the remote ZigBee.
	 * Check if we use the SAPI or the AF send message.
	 * We expect that the user format the content of the
	 * message correctly.
	 */
	data = kmalloc((len + ZCMD_HEADER),GFP_KERNEL);
	data[0] = len;

	if(zb->z_info.app_type == AF_INTERFACE_TYPE)
	{
		data[1] = AF_DATA_REQ_0;
		data[2] = AF_DATA_REQ_1;
	}
	else
	{
		/* anything else will use SAPI */
		data[1] = ZB_SEND_DATA_REQ_0;
		data[2] = ZB_SEND_DATA_REQ_1;
	}
	
	memcpy((unsigned char *)(&data[3]), buf, len );

	zaccel_spi_req(zb,data,(len + ZCMD_HEADER));
	return 0;
}

int zb_SysTestLoopback(struct bmi_zb *zb)
{
	unsigned char *data;
	unsigned char i, len;
	unsigned char buf_len;
	unsigned char pattern[32];
	int rc;

	len = 4;
	buf_len = len + ZCMD_HEADER;
	data = kmalloc(buf_len,GFP_KERNEL);

	data[0] = len;
	data[1] = SYS_TEST_LOOPBACK_REQ_0;
	data[2] = SYS_TEST_LOOPBACK_REQ_1;

	/* Prepare test pattern */
	pattern[0] = 0xAA;
	pattern[1] = 0x55;
	pattern[2] = 0x07;
	pattern[3] = 0x5A;
	pattern[4] = 0x63;
	pattern[5] = 0x58;
	pattern[6] = 0x74;
	pattern[7] = 0x55;
	pattern[8] = 0x02;
	pattern[9] = 0x04;
	pattern[10] = 0x08;
	pattern[11] = 0x10;
	pattern[12] = 0x20;
	pattern[13] = 0xAA;
	pattern[14] = 0x55;

	memcpy(&data[3],pattern,len);

#define TEST_LOOPBACKx
#ifdef TEST_LOOPBACK
	printk("Loopback pattern: ");
	for(i = 0; i < buf_len; i++)
	{
		printk("%x ",data[i]);
	}
	printk(KERN_DEBUG "\n");
#endif

	zaccel_spi_req(zb,data,buf_len);

	/* Verify loopback results */
	if((data[1] == (SYS_TEST_LOOPBACK_REQ_0 | RSPS_CMD)) &&
	   (data[2] == SYS_TEST_LOOPBACK_REQ_1))
	{
		if(data[0] == len)
		{
			rc = 0;
			for(i = 0; i < len; i++)
			{
				if(data[ZCMD_HEADER + i] != pattern[i])
				{
					rc++;
					printk(KERN_WARNING "error: zb test byte %d expects %x, gets %x\n",i,pattern[i],data[ZCMD_HEADER + i]);
				}
			}
		}
		else
		{
			rc = -2;
		}
	}
	else
	{
		rc = -1;
	}

	return rc;
}

void zDeviceReport(struct bmi_zb *zb,unsigned char type, unsigned char len,unsigned char *buf)
{
	unsigned char *msg;

	msg = kmalloc((ZCMD_HEADER + len),GFP_KERNEL);

	msg[0] = len;
	msg[1] = ZB_DEVICE_INFO_CHG_0;
	msg[2] = ZB_DEVICE_INFO_CHG_0;
	memcpy(&msg[3],buf,len);

	len = ZCMD_HEADER + len;

#ifdef VE_OUT
	/* Send message to the application layer through control socket */
	zb_rx(zb->netdev,msg,len,Z_CONTROL_SOCK);
#endif

	kfree(msg);
}

/*
 * This routine send data originated by the user layer to
 * the Z-Accel.
 */
void zaccel_xmt(struct work_struct *work)
{
	struct bmi_zb *zb;
	struct zaccel_info *z_info;
	struct zaccel_xmt_msg *msg;

	zb = container_of(work, struct bmi_zb, xmt_work);
	z_info = &zb->z_info;
	
	while(!list_empty(&z_info->xmt_list))
	{
		msg = list_entry(z_info->xmt_list.next, struct zaccel_xmt_msg, list);

		zb_SendDataRequest(zb, msg->buf, msg->len);

		list_del(z_info->xmt_list.next);
		kfree(msg);
	}
}

/* 
 * zaccel_cmd_proc processes messages from the Zaccel.
 */

void zaccel_cmd_proc(struct bmi_zb *zb, unsigned char *buf)
{
	struct zaccel_info *z_info;
	struct net_zb *priv;
	unsigned char len;
	unsigned short cmd;
	int rc;

	z_info = &zb->z_info;

	len = buf[0] + ZCMD_HEADER;

	/* process the message from Z-Accel */

	cmd = ((((unsigned short)buf[1] << 8) & 0xFF00) | 
	        ((unsigned short)buf[2] & 0x00FF));

	switch(cmd)
	{
		case SYS_RESET_IND:

			if(buf[3] == 2)
			{
				/* count number of reset by watch-dog */
				reset_count++;
			}

			printk(KERN_INFO "zb%d SYS_RESET_IND %d \n",zb->slot,reset_count);
					/* rcv Z-Accel reset indication. */
			z_info->msg_flag |= RESET_IND_BIT;
			wake_up_interruptible(&z_info->wait_queue);
		
			zb_rx(zb->netdev,buf,len,Z_CONTROL_SOCK);
			break;

		case ZB_START_CONFIRM:
			/* 
 		 	 *	Z-Stack starts.  Device is ready to run applicaton.
 		 	 */
			z_info->msg_flag |= START_CNF_BIT;
			wake_up_interruptible(&z_info->wait_queue);
			break;

		case ZB_RCV_DATA_IND:
		case AF_INCOMING_MSG:
			/* 
 		 	 * Receive a packet from a remote device 
 		 	 * Send it to the user throught packet socket.
 		 	 */
			len = buf[0];

			if(zb->netdev)
			{
				rc = zb_rx(zb->netdev,&buf[3],len,Z_PACKET_SOCK);
				priv = netdev_priv(zb->netdev);
				if(rc >= 0)
				{
					priv->stats.rx_packets++;
					priv->stats.rx_bytes += len;
				}
				else
				{
					priv->stats.rx_dropped++;
				}
			}
			break;

		case ZB_BIND_CONFIRM:
			printk(KERN_DEBUG "Rec BIND CNF - cmd %x %x rc %x len %d\n",
			         buf[3],buf[4],buf[5],len);
			zb_rx(zb->netdev,buf,len,Z_CONTROL_SOCK);
			break;
			
		case ZB_SEND_DATA_CONFIRM:
			/* Return the results from zb_SendDataRequest */
			if(zb->netdev)
			{
				priv = netdev_priv(zb->netdev);
				if(buf[4] == Z_SUCCESS) 
					priv->stats.tx_packets++;
				else
				{
					printk(KERN_WARNING "ZB: send data failed 0x%x\n",buf[4]);
					priv->stats.tx_dropped++;
				}
				zb_rx(zb->netdev,buf,len,Z_CONTROL_SOCK);
			}
			break;

		case ZDO_STATE_CHANGE_IND:
			if((z_info->msg_flag & START_CNF_BIT) == 0)
			{
				z_info->msg_flag |= START_CNF_BIT;
				wake_up_interruptible(&z_info->wait_queue);
			}
			break;

		default:
				
			/* send data to the user application */
			zb_rx(zb->netdev,buf,len,Z_CONTROL_SOCK);
			break;
	}
}

int init_zaccel(struct bmi_zb *zb)
{
	struct zaccel_info *z_info;
	int rc = 0;

	z_info = &zb->z_info;
	memset(z_info,0, sizeof(struct zaccel_info));

	init_waitqueue_head(&z_info->wait_queue);

	INIT_LIST_HEAD(&z_info->xmt_list);

	INIT_WORK(&zb->xmt_work, zaccel_xmt);

	z_info->msg_flag = 0;
	z_info->app_type = NO_APP_TYPE;

	/* Release Z-Accel */
	zb_Reset(zb,ZB_RELEASE);
	
	printk(KERN_DEBUG "wait for ZACCEL_RESET_IND\n");
	/* Wait for the reset indication message from Z-Accel */
	rc = wait_event_interruptible_timeout(z_info->wait_queue,
	               ((z_info->msg_flag & RESET_IND_BIT) != 0),Z_RESET_TIMEOUT);

	/* 
 	 * Run Loopback test to test SPI Interface.  If this test fails, and
 	 * SPI interface is bad, we probably would not get the reset indication
 	 * message that we were waiting for previously.
 	 */

	if(rc == 0)
	{
		printk(KERN_ERR "bmi_zaccel: Z-Accel device failed\n");
		return -ENODEV;
	
	}
	else
	{
		/* a short delay to make sure that Z-Accel is done
		 * initializing.   Advice from TI ZigBee Forum discussion.
		 */
		mdelay(2000);
		/* SPI loopback test */
		rc  = zb_SysTestLoopback(zb);
		if(rc != 0)
		{
			printk(KERN_ERR "bmi_zaccel: SPI Loopback test FAILED %d\n",rc);
			return -ENODEV;
		}
	}

	/* print string for python factory test. Don't remove */
	printk(KERN_INFO "bmi_zaccel: SPI Loopback test PASSED\n");

	printk(KERN_INFO "bmi_zaccel: Z-Accel is ready\n");

	return 0;
}

void remove_zaccel(struct bmi_zb *zb)
{
	/* hold Zaccel reset */
	zb_Reset(zb,ZB_RESET);
}


void zaccel_getdev(struct bmi_zb *zb)
{
	struct zaccel_info *z_info;
	struct zaccel_device zdev;

	z_info = &zb->z_info;

	zb_GetDeviceInfo(zb,ZB_DEVICE_STATE,
		                    (unsigned char *)&zdev.state);
	zb_GetDeviceInfo(zb,ZB_DEVICE_IEEE_ADDR,
		                    (unsigned char *)&zdev.device_ieee);
	zb_GetDeviceInfo(zb,ZB_DEVICE_SHORT_ADDR,
		                    (unsigned char *)&zdev.device_short);
	zb_GetDeviceInfo(zb,ZB_PARENT_SHORT_ADDR,
		                    (unsigned char *)&zdev.parent_short);
	zb_GetDeviceInfo(zb,ZB_PARENT_IEEE_ADDR,
		                    (unsigned char *)&zdev.parent_ieee);
	zb_GetDeviceInfo(zb,ZB_DEVICE_CHANNEL,
		                    (unsigned char *)&zdev.channel);
	zb_GetDeviceInfo(zb,ZB_DEVICE_PANID,
		                    (unsigned char *)&zdev.panid);
	zb_GetDeviceInfo(zb,ZB_DEVICE_EXT_PANID,
		                    (unsigned char *)&zdev.ext_panid);
	
	z_info->device.state = zdev.state;
	memcpy(&z_info->device.device_ieee,&zdev.device_ieee,8);
	memcpy(&z_info->device.device_short,&zdev.device_short,2);
	memcpy(&z_info->device.parent_short,&zdev.parent_short,2);
	memcpy(&z_info->device.parent_ieee,&zdev.parent_ieee,8);
	z_info->device.channel = zdev.channel;
	memcpy(&z_info->device.panid,&zdev.panid,2);
	memcpy(&z_info->device.ext_panid,&zdev.ext_panid,8);
}
