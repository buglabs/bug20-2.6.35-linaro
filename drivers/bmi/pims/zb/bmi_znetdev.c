/*
 * bmi_znetdev.c
 *
 * This file contains the zaccel network device driver codes.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/if.h>
#include <linux/sysfs.h>
#include <linux/bmi/bmi_zb.h>
#include "bmi_zigbee.h"
#include "bmi_zaccel.h"

static int zb_open(struct net_device *dev)
{
	struct bmi_zb *zb;
	struct net_zb *priv;
	struct zaccel_device *zdev;

	printk(KERN_DEBUG "bmi_znetdev: zb_open\n");
	priv = netdev_priv(dev);
	zb = priv->zb;
	zdev = &zb->z_info.device;
	
	priv->net_open = 1;
	netif_start_queue(dev);
	return 0;
}

static int zb_close(struct net_device *dev)
{
	struct net_zb *priv;

	printk(KERN_DEBUG "bmi_znetdev: zb_close\n");
	priv = netdev_priv(dev);

	priv->net_open = 0;
	netif_stop_queue(dev);

	return 0;
}

static int zb_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{ 
	unsigned char len_chk;
	unsigned char offset;
	struct net_zb *priv;
	struct bmi_zb *zb;
	struct zaccel_info *z_info;

	unsigned char *zapp;
	unsigned char buf[32];
	unsigned short len;
	unsigned char type;
	unsigned char reason = 0;
	unsigned char *user_pt;
	unsigned char flag;
	unsigned char state;
	int i;
	int rc = 0;
	
	priv = netdev_priv(dev);
	zb = priv->zb;
	z_info = &zb->z_info;
	
	switch(cmd)
	{
		case SIOCSRESET:
			if(copy_from_user(buf, ifr->ifr_data, (2*sizeof(char))))
			{
				printk(KERN_ERR "BMI_ZB_ANY_REQ cannot get commands\n");
				return -EFAULT;
			}

			switch(buf[0])
			{
				case Z_HW_RESET:
					if(dev->flags & IFF_UP)
					{
						/* set flag to remember that IF is up previously */
						flag = IFF_UP;
						dev->flags &= ~IFF_UP;
						netif_stop_queue(dev);
					}

					zb_Reset(zb,ZB_RESET);
					udelay(10);
					zb_Reset(zb,ZB_RELEASE);

					break;

				case Z_SW_RESET:
					if(dev->flags & IFF_UP)
					{
						flag = IFF_UP;
						dev->flags &= ~IFF_UP;
						netif_stop_queue(dev);
					}

					zb_SoftReset(zb);
					break;

				case Z_DONT_RESET:
					return 0;

				default:
					printk(KERN_WARNING "zb ioctl reset - unknow reset type %d\n",buf[1]);
					return -EINVAL;
			}

			/* 
 			 * If reset the device, wait for it to come back and
 			 * start Z-stack.
 			 */
			rc = wait_event_interruptible_timeout(z_info->wait_queue,
			     ((z_info->msg_flag & RESET_IND_BIT) != 0),Z_RESET_TIMEOUT);

			if(rc == 0)
			{
				/* timeout and no indication received */
				reason = 1;
				printk(KERN_ERR "zb: wait for reset IND timeout\n");
				rc = -EAGAIN;
			}
			else
			{

#ifdef VE_OUT
				if(flag & IFF_UP)
				{
					netif_start_queue(dev);
					dev->flags |= IFF_UP;
				}
#endif

				rc = 0;
			}

			if(rc == -EAGAIN)
			{
				/* copy extra code to the user space */
				if(copy_to_user(ifr->ifr_data,(void *)&reason,1))
				{
					return -EFAULT;
				}
			}

			break;

		case SIOCGDEVICEINFO:
			
			if(copy_from_user(buf, ifr->ifr_data, 1))
			{
				printk(KERN_ERR "SIOCGDEVICEINFO cannot get parameter\n");
				return -EFAULT;
			}

			len = zb_GetDeviceInfo(zb,buf[0],buf);

			if(copy_to_user(ifr->ifr_data, buf, len))
			{
				return -EFAULT;
			}

			rc = (int)len;
			break;

		case SIOCSAPPREGISTER:
			if(copy_from_user(buf, ifr->ifr_data, sizeof(char)))
			{
				printk(KERN_ERR "SIOCSAPPREGISTER cannot get parameter\n");
				return -EFAULT;
			}

			len = (unsigned short)buf[0];

			/* 
 			 * Input Format:
 			 *      1-byte  message length
 			 *      x-bytes  message (see below comment for msg format.
 			 */

			zapp = kmalloc(len,GFP_KERNEL);
			if(zapp == NULL)
			{
				printk(KERN_WARNING "zb_ioctl no buf for cmd 0x%x\n",cmd);
				return -ENOMEM;
			}
			
			user_pt = (unsigned char *)ifr->ifr_data;
			if(copy_from_user((void *)zapp, &user_pt[1], (unsigned long)len))
			{
				kfree(zapp);
				return -EFAULT;
			}

			/* 
 			 * verify the length of the data.
 			 * Format of message is (number in parenthesis = num of bytes)
 			 *  
 			 *  zapp[offset]        content
 			 * 	0                   appEndpoint(1)
 			 * 	1                   appProfileID(2)
 			 * 	3                   deviceId(2)
 			 * 	5                   deviceVersion(1)
 			 * 	6                   unused(1)
 			 * 	7                   inputCommandNum(1)
 			 * 	  [8]                  [inputCommand(2),inputCommand(2), ...]
 			 * 	8 + (inputNum * 2)  outputCommandNum(1)
 			 * 	                       [outputCommand(2),outputCommand(2), ...]
 			 */

			/* 8 bytes between appEndpoint to inputCommandNum */
			len_chk = 8;

			/* 
 			 * Get the offset to outputCommandNum byte 
			 * and add the number of inputCommand bytes and outputCommandNum
			 * byte to the len_chk.
			 */
			offset = (zapp[7] << 1) + 8;
			len_chk += (zapp[7] << 1) + 1;

			/* Add the number of output command bytes */
			len_chk += (zapp[offset] << 1);

			if(len_chk != len)
			{
				/*
 				 * The len of the input declared is less than the
 				 * number of data required.
 				 */
				kfree(zapp);
				printk(KERN_WARNING "SIOCSAPPREGISTER: inconsistence len %d %d\n",len_chk,len);
				return -EINVAL;
			}

			buf[0] = zb_AppRegisterRequest(zb,len,zapp);

			if(buf[0] != 0)
			{
				rc = -1;
			}

			if(copy_to_user(ifr->ifr_data,buf,1))
			{
				printk(KERN_WARNING "SIOCAPP result dropped\n");
			}

			kfree(zapp);

			break;

		case SIOCSALLOWBIND:
			if(copy_from_user(buf, ifr->ifr_data, sizeof(char)))
			{
				printk(KERN_ERR "SIOCSALLOWBIND cannot get parameter\n");
				return -EFAULT;
			}

			/*
 			 * Input format:\
 			 *     1-bytes timeout
 			 */
			
			zb_AllowBind(zb,buf[0]);

			break;

		case SIOCSSTARTREQ:
			zb_StartRequest(zb);
			rc = wait_event_interruptible_timeout(z_info->wait_queue,
                      ((z_info->msg_flag & START_CNF_BIT) != 0),Z_CNF_TIMEOUT);

			if(rc == 0)
			{
				/* timeout - We didn't receive START_CNF message nor
				 * ZDO_STATE_CHANGE_IND.   This can happens if the
				 * end-device or router does not find a network.
				 * Check if the device state had changed from DEV_HOLD
				 * to anything else.   If it does, the stack has started.
				 */
				zb_GetDeviceInfo(zb,ZB_DEVICE_STATE,(unsigned char *)&state);
				if((state != DEV_HOLD) || (state != DEV_INIT))
				{
					printk(KERN_DEBUG "stack starts, state %d\n",state);
					rc = 0;
				}
				else
				{
					rc = -1;
					printk(KERN_DEBUG "Stack fails to start\n");
				}
			}
			else
			{
				printk(KERN_DEBUG "rcv ZB_START_CONFIRM\n");
				rc = 0;
			}

			break;

		case SIOCSPERMITJOINING:
			if(copy_from_user(buf, ifr->ifr_data, 3))
			{
				printk(KERN_ERR "SIOCSPERMITJOINING: read error\n");
				return -EFAULT;
			}
			
			/*
 			 * Input format:
 			 *     2-bytes 16-bit device address
 			 *     1-bytes timeout
 			 */
#ifdef VE_OUT
			printk(KERN_DEBUG "permitjoining address 0x%x timeout 0x%x\n",*(unsigned short *)buf,buf[2]);
#endif
			
			buf[0] = zb_PermitJoiningRequest(zb,&buf[0],buf[2]);

			if(buf[0] != 0)
			{
				rc = -1;
			}

			if(copy_to_user(ifr->ifr_data,buf,1))
			{
				printk(KERN_WARNING "SIOCSPERMIT result dropped\n");
			}

			break;

		case SIOCSBIND:
			if(copy_from_user(buf, ifr->ifr_data, 12))
			{
				printk(KERN_ERR "SIOCSBIND cannot get parameter\n");
				return -EFAULT;
			}

			if((buf[0] != Z_BIND_CREATE) && (buf[0] != Z_BIND_REMOVE))
			{
				return -EINVAL;
			}

			zb_BindRequest(zb,buf[0],&buf[1]);

			break;

		case SIOCSFINDDEVICE:
			if(copy_from_user(buf, ifr->ifr_data, 8))
			{
				printk(KERN_ERR "SIOCSFINDDEVICE cannot get parameter\n");
				return -EFAULT;
			}
			zb_FindDeviceRequest(zb,buf);
			break;

		case SIOCSAFREGISTER:
			if(copy_from_user(buf, ifr->ifr_data, sizeof(char)))
			{
				printk(KERN_ERR "SIOCSAPPREGISTER cannot get parameter\n");
				return -EFAULT;
			}

			len = (unsigned short)buf[0];

			/*
 			 * Input Format:
 			 *      1-byte  message length
 			 *      x-bytes  message (see below comment for msg format.
 			 */

			zapp = kmalloc(len,GFP_KERNEL);
			if(zapp == NULL)
			{
				printk(KERN_WARNING "zb_ioctl no buf for cmd 0x%x\n",cmd);
				return -ENOMEM;
			}

			user_pt = (unsigned char *)ifr->ifr_data;
			if(copy_from_user((void *)zapp, &user_pt[1], (unsigned long)len))
			{
				kfree(zapp);
				return -EFAULT;
			}

			buf[0] = zb_AFRegisterRequest(zb,len,zapp);
			if(buf[0] != 0)
			{
				rc = -1;
			}

			if(copy_to_user(ifr->ifr_data,buf,1))
			{
				printk(KERN_WARNING "SIOCAPP result dropped\n");
			}

			kfree(zapp);
			break;

		case SIOCSPOWERAMP:
			if(copy_from_user(buf, ifr->ifr_data, 2))
			{
				printk(KERN_ERR "SIOCSPOWERAMP cannot get parameter\n");
				return -EFAULT;
			}

			rc = zb_sysRFpowerAmp(zb,buf[0],buf[1]);
			break;

		case SIOCSZCOMMAND:

			/*
 			 * This command passes any Z-Accel command to the device and
 			 * returns the reply back to the user.
 			 * Message format:
 			 * 		1-byte  length of data field
 			 * 		2-bytes command
 			 * 		0-128 bytes data
 			 */
			if(copy_from_user(buf, ifr->ifr_data, 1))
			{
				printk(KERN_ERR "SIOCSZCOMMAND cannot get parameter\n");
				return -EFAULT;
			}

			user_pt = (unsigned char *)ifr->ifr_data;
			len = (unsigned short)buf[0] + 3;
			
			if(len  > ZCMD_BUF)
			{
				printk(KERN_WARNING "SIOCSZCOMMAND message is too long\n");
				return -EINVAL;
			}

			zapp = kmalloc(ZCMD_BUF,GFP_KERNEL);
			if(zapp == NULL)
			{
				printk(KERN_WARNING "SIOCSZCOMMAND no buf for the cmd\n");
				return -ENOMEM;
			}

			if(copy_from_user(zapp, ifr->ifr_data, len))
			{
				printk(KERN_ERR "SIOCSZCOMMAND cannot get parameter\n");
				return -EFAULT;
			}
			type = zapp[1] & SPI_CMD_TYPE_MASK;

#define VE_DEBUG
#ifdef VE_DEBUGx
			printk("len %d: ",len);
			for(i = 0; i < len; i++)
			{
				printk("%x ",zapp[i]);
			}
			printk("\n");
#endif

			zaccel_spi_req(zb,zapp,len);

			/*
			 * read the length of the data, if it's SREQ command. 
			 */
		
			if(type != SPI_CMD_AREQ)
			{
				len = zapp[0];

				if(copy_to_user(ifr->ifr_data,zapp,(len + 3)))
				{
					return -EFAULT;
				}
			}
			rc = 0;

			break;

		default:
			printk("zb_ioctl default cmd\n");
			rc = -EINVAL;
			break;
			
	}

	return rc;
}

/*
 * zb_rx passes data from Z-Accel to the user via socket
 */
int zb_rx(struct net_device *dev, unsigned char *buf, unsigned char len, unsigned short type)
{
	struct sk_buff *skb;
	struct net_zb *priv;

	if(!dev)
		return -EINVAL;
	

	priv = netdev_priv(dev);

	if(priv->socket[type] == Z_NO_SOCK)
	{
		return -EINVAL;
	}
	
	/* 
 	 * Check if the device is bound to a socket.
 	 * If not, drop the message.
 	 */

	dev->last_rx = jiffies;
	
	skb = dev_alloc_skb(len);
	if(skb)
	{
		memcpy(skb_put(skb,len),buf,len);

		/* 
 		 * Set the device to zigbee.
 		 * Set protocol number to zero, so the kernel will not
 		 * pass it through the protocol stack.
 		 */

		skb->dev = dev;
		skb->protocol = type;
		
		if(type == Z_PACKET_SOCK)
		{
			/*
 			 * Increment packet count and notify kernel of 
 			 * the new packet.
 			 */
			priv->stats.rx_packets++;
		}

		netif_rx(skb);
	}
	else
	{
		if(type == Z_PACKET_SOCK)
			priv->stats.rx_dropped++;
	}

	return 0;
}

int zb_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct bmi_zb *zb;
	struct net_zb *priv;
	struct zaccel_xmt_msg *msg;
	unsigned char *buf;

	priv = netdev_priv(dev);
	zb = priv->zb;

	if(skb->len > XMT_MSG_SIZE)
	{
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	msg = (struct zaccel_xmt_msg *)kmalloc(sizeof(struct zaccel_xmt_msg), GFP_KERNEL);
	buf = msg->buf;
	
	memcpy(buf,skb->data,skb->len);

	msg->len = skb->len;

	/* 
 	 * zb_tx is called by dev_queue_xmit, which requires atomic
 	 * operation.  We cannot call zb_SendDataRequest directly here
 	 * to schedule SPI transfer to the Z-Accel because 
 	 * it causes "BUG: scheduling while atomic exception."
 	 * We put the data in a message queue and 
 	 * schedule a work queue to call zb_SendDataRequest later.
 	 */

	list_add_tail(&msg->list, &zb->z_info.xmt_list);
	schedule_work(&zb->xmt_work);

	/* Move this statistic to a real packet transmission location later */
	priv->stats.tx_packets++;

	dev->trans_start = jiffies;

	/* Free sku buffer */
	dev_kfree_skb(skb);

	return 0;
}

struct net_device_stats *zb_stats(struct net_device *dev)
{
	struct net_zb *priv = netdev_priv(dev);

	return &priv->stats;
}

void zb_net_setup(struct net_device *dev)
{
	struct net_zb *priv;

	priv = netdev_priv(dev);

	priv->dev = dev;
	priv->socket[Z_PACKET_SOCK] = Z_NO_SOCK;
	priv->socket[Z_CONTROL_SOCK] = Z_NO_SOCK;

	dev->open = zb_open;
	dev->stop = zb_close;
	dev->hard_start_xmit = zb_tx;
	dev->tx_queue_len = 32;
	dev->get_stats = zb_stats;
	dev->do_ioctl = zb_ioctl;
	dev->flags = 0;
}

int zdev_setopt(struct net_device *dev,int cmd, int len, unsigned char *buf)
{
	struct net_zb *priv = netdev_priv(dev);
	struct bmi_zb *zb;
	int rc;

	zb = priv->zb;
	rc = (int)zb_WriteConfiguration(zb,(unsigned char)cmd,(unsigned char)len,buf);
	return rc;
}

int zdev_getopt(struct net_device *dev, int cmd, int *len, unsigned char *buf)
{
	struct net_zb *priv = netdev_priv(dev);
	struct bmi_zb *zb;

	zb = priv->zb;

	*len = (int)zb_ReadConfiguration(zb,(unsigned char)cmd,buf);
	if(*len != 0)
	{
		return 0;
	}
	else
	{
		return -EFAULT;
	}

}

static ssize_t show_device(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device *net = to_net_dev(dev);
	struct net_zb *priv = netdev_priv(net);
	struct bmi_zb *zb = priv->zb;
	int size = 0;
	int rc;
	unsigned char rbuf[8];

	rc = zb_ReadConfiguration(zb,ZCD_NV_LOGICAL_TYPE,rbuf);
	if(rc <= 0)
	{
			size = sprintf(buf,"Bad read.  Try again\n");
	}
	switch(rbuf[Z_CONFIG_OFFSET])
	{
		case ZB_COORDINATOR:
			size = sprintf(buf,"coordinator\n");
			break;

		case ZB_ROUTER:
			size = sprintf(buf,"router\n");
			break;

		case ZB_ENDDEVICE:
			size = sprintf(buf,"end-device\n");
			break;

		default:
			size = sprintf(buf,"device-error\n");
			break;
	}
	return size;
}

static ssize_t store_device(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	struct net_device *net = to_net_dev(dev);
	struct net_zb *priv = netdev_priv(net);
	struct bmi_zb *zb = priv->zb;
	int size;
	unsigned char device;

	if(memcmp(buf, "coordinator",(count-1)) == 0)
	{
		printk(KERN_DEBUG "zb: config to coordinator\n");
		device = ZB_COORDINATOR;
	}
	else if(memcmp(buf, "router",(count-1)) == 0)
	{
		printk(KERN_DEBUG "zb: config to router\n");
		device = ZB_ROUTER;
	}
	else if(memcmp(buf, "end-device",(count-1)) == 0)
	{
		printk(KERN_DEBUG "zb: config to end-device\n");
		device = ZB_ENDDEVICE;
	}
	else
	{
		printk(KERN_DEBUG "zb: invalid config\n");
		size = sprintf(buf,"Invalid device.  Try again\n");
		return count;
	}

	if(zb_WriteConfiguration(zb,ZCD_NV_LOGICAL_TYPE,1,&device) != 0)
	{
		size = sprintf(buf,"Bad write.  Try again\n");
	}

	return count;
}

static ssize_t store_chanlist(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	struct net_device *net = to_net_dev(dev);
	struct net_zb *priv = netdev_priv(net);
	struct bmi_zb *zb = priv->zb;
	int size;
	unsigned long chanlist;
	unsigned char rbuf[4];

	chanlist = simple_strtol(buf,NULL,16);
	rbuf[0] = (unsigned char)(chanlist) & 0xFF;
	rbuf[1] = (unsigned char)(chanlist >> 8) & 0xFF;
	rbuf[2] = (unsigned char)(chanlist >> 16) & 0xFF;
	rbuf[3] = (unsigned char)(chanlist >> 24) & 0xFF;

	if(zb_WriteConfiguration(zb,ZCD_NV_CHANLIST,4,(unsigned char *)rbuf) != 0)
	{
		size = sprintf(buf,"Bad write.  Try again\n");
	}
	
	return count;
}

static ssize_t show_chanlist(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device *net = to_net_dev(dev);
	struct net_zb *priv = netdev_priv(net);
	struct bmi_zb *zb = priv->zb;
	unsigned char rbuf[10];
	int size = 0;
	int rc;
	
	rc = zb_ReadConfiguration(zb,ZCD_NV_CHANLIST,rbuf);
	if(rc < 0)
	{
		size = sprintf(buf,"bad read.   Try again\n");
	}
	else
	{
		size  = sprintf(buf,"%0x ",rbuf[Z_CONFIG_OFFSET+3]);
		size += sprintf((buf+size),"%0x ",rbuf[Z_CONFIG_OFFSET+2]);
		size += sprintf((buf+size),"%0x ",rbuf[Z_CONFIG_OFFSET+1]);
		size += sprintf((buf+size),"%0x\n",rbuf[Z_CONFIG_OFFSET]);
	}
	return size;
}

static ssize_t store_initop(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	struct net_device *net = to_net_dev(dev);
	struct net_zb *priv = netdev_priv(net);
	struct bmi_zb *zb = priv->zb;
	unsigned char option;
	int size;

	option = (unsigned char)simple_strtol(buf,NULL,16);
	option &= ZCD_STARTOPT_MASK;

	if(zb_WriteConfiguration(zb,ZCD_NV_STARTUP_OPTION,1,(unsigned char *)&option) != 0)
	{
		size = sprintf(buf,"Bad write.   Try again\n");
	}
	
	return count;
}

static ssize_t show_initop(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device *net = to_net_dev(dev);
	struct net_zb *priv = netdev_priv(net);
	struct bmi_zb *zb = priv->zb;
	unsigned char rbuf[10];
	int size = 0;
	int rc;

	rc = zb_ReadConfiguration(zb,ZCD_NV_STARTUP_OPTION,(unsigned char *)&rbuf);
	if(rc < 0)
	{
		size = sprintf(buf,"bad read.   Try again\n");
	}
	else
	{
		size = sprintf(buf,"%0x\n",rbuf[Z_CONFIG_OFFSET]);
	}

	return size;
}

static ssize_t store_panid(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	struct net_device *net = to_net_dev(dev);
	struct net_zb *priv = netdev_priv(net);
	struct bmi_zb *zb = priv->zb;
	unsigned short panid;
	unsigned char rbuf[2];
	int size;

	panid = (unsigned short)simple_strtol(buf,NULL,16);
	rbuf[0] = (unsigned char)(panid) & 0xFF;
	rbuf[1] = (unsigned char)(panid >> 8) & 0xFF;

	if(zb_WriteConfiguration(zb,ZCD_NV_PANID,2,(unsigned char *)rbuf) != 0)
	{
		size = sprintf(buf,"Bad write read.   Try again\n");
	}
	
	printk(KERN_DEBUG "zb: config panid 0x%x\n",panid);
	return count;
}

static ssize_t show_panid(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device *net = to_net_dev(dev);
	struct net_zb *priv = netdev_priv(net);
	struct bmi_zb *zb = priv->zb;
	unsigned char rbuf[10];
	int size = 0;
	int rc;

	rc = zb_ReadConfiguration(zb,ZCD_NV_PANID,(unsigned char *)&rbuf);
	if(rc < 0)
	{
		size = sprintf(buf,"bad read.   Try again\n");
	}
	else
	{
		size = sprintf(buf,"%0x ",rbuf[Z_CONFIG_OFFSET+1]);
		size += sprintf((buf+size),"%0x\n",rbuf[Z_CONFIG_OFFSET]);
	}
	return size;
}

static ssize_t show_zbinfo(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device *net = to_net_dev(dev);
	struct net_zb *priv = netdev_priv(net);
	struct bmi_zb *zb = priv->zb;
	struct zaccel_device *z_dev = &zb->z_info.device;
	int size = 0;

	zaccel_getdev(zb);

	switch(z_dev->state)
	{
		case DEV_HOLD:
			size = sprintf(buf,"DEV_HOLD\n");
			break;

		case DEV_INIT:
			size = sprintf(buf,"DEV_INIT\n");
			break;

		case DEV_NWK_DISC:
			size = sprintf(buf,"DEV_NWK_DISC\n");
			break;

		case DEV_NWK_JOINING:
			size = sprintf(buf,"DEV_NWK_JOINING\n");
			break;

		case DEV_NWK_REJOIN:
			size = sprintf(buf,"DEV_NWK_REJOIN\n");
			break;

		case DEV_END_DEVICE_UNAUTH:
			size = sprintf(buf,"DEV_END_DEVICE_UNAUTH\n");
			break;

		case DEV_END_DEVICE:
			size = sprintf(buf,"DEV_END_DEVICE\n");
			break;

		case DEV_ROUTER:
			size = sprintf(buf,"DEV_ROUTER\n");
			break;

		case DEV_COORD_STARTING:
			size = sprintf(buf,"DEV_COORD_STARTING\n");
			break;

		case DEV_ZB_COORD:
			size = sprintf(buf,"DEV_ZB_COORD\n");
			break;

		case DEV_NWK_ORPHAN:
			size = sprintf(buf,"DEV_NWK_ORHAN\n");
			break;

		default:
			size = sprintf(buf,"UNKNOWN STATE %d\n",z_dev->state);
			break;
	}

	size += sprintf((buf+size),"device IEEE address:  %x:%x:%x:%x:%x:%x:%x:%x\n",
	         z_dev->device_ieee[7],z_dev->device_ieee[6],z_dev->device_ieee[5],
	         z_dev->device_ieee[4],z_dev->device_ieee[3],z_dev->device_ieee[2],
	         z_dev->device_ieee[1],z_dev->device_ieee[0]);
	size += sprintf((buf+size),"device short address: 0x%x %x\n",
	         z_dev->device_short[1],z_dev->device_short[0]);
	size += sprintf((buf+size),"parent short address: 0x%x %x\n",
	         z_dev->parent_short[1],z_dev->parent_short[0]);
	size += sprintf((buf+size),"parent IEEE address:  %x:%x:%x:%x:%x:%x:%x:%x\n",
	         z_dev->parent_ieee[7],z_dev->parent_ieee[6],z_dev->parent_ieee[5],
	         z_dev->parent_ieee[4],z_dev->parent_ieee[3],z_dev->parent_ieee[2],
	         z_dev->parent_ieee[1],z_dev->parent_ieee[0]);
	size += sprintf((buf+size),"channel:              0x%0x\n",z_dev->channel);
	size += sprintf((buf+size),"PAN ID:               0x%0x %0x\n",
	         z_dev->panid[1],z_dev->panid[0]);
	size += sprintf((buf+size),"extended PAN ID:      %x:%x:%x:%x:%x:%x:%x:%x\n",
	         z_dev->ext_panid[7],z_dev->ext_panid[6],z_dev->ext_panid[5],
	         z_dev->ext_panid[4],z_dev->ext_panid[3],z_dev->ext_panid[2],
	         z_dev->ext_panid[1],z_dev->ext_panid[0]);
	return size;
}


DEVICE_ATTR(device, (S_IRUGO | S_IWUSR), show_device, store_device);
DEVICE_ATTR(panid, (S_IRUGO | S_IWUSR), show_panid, store_panid);
DEVICE_ATTR(chanlist, (S_IRUGO | S_IWUSR), show_chanlist, store_chanlist);
DEVICE_ATTR(initop, (S_IRUGO | S_IWUSR), show_initop, store_initop);
DEVICE_ATTR(zbinfo, S_IRUGO, show_zbinfo, NULL);

void zb_create_sysfs(struct net_device *net)
{
	struct device *dev = &(net->dev);

	if(device_create_file(dev,&dev_attr_device) < 0)
		printk(KERN_WARNING "zb: failed to create device attribute\n");

	if(device_create_file(dev,&dev_attr_panid) < 0)
		printk(KERN_WARNING "zb: failed to create panid attribute\n");

	if(device_create_file(dev,&dev_attr_chanlist) < 0)
		printk(KERN_WARNING "zb: failed to create chanlist attribute\n");

	if(device_create_file(dev,&dev_attr_initop) < 0)
		printk(KERN_WARNING "zb: failed to create initop attribute\n");

	if(device_create_file(dev,&dev_attr_zbinfo) < 0)
		printk(KERN_WARNING "zb: failed to create zinfo attribute\n");
}

void zb_remove_sysfs(struct net_device *net)
{
	struct device *dev = &(net->dev);

	device_remove_file(dev,&dev_attr_device);
	device_remove_file(dev,&dev_attr_panid);
	device_remove_file(dev,&dev_attr_chanlist);
	device_remove_file(dev,&dev_attr_initop);
	device_remove_file(dev,&dev_attr_zbinfo);
}

