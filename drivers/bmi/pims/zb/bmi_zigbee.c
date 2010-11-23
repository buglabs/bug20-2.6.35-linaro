/*
 * 	bmi_zigbee.c
 *
 * 	BMI zigbee device driver
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 * 
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * Include files
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/if.h>
#include <asm/uaccess.h>
#include <linux/bmi/bmi_zb.h>
#include "bmi_zigbee.h"  
#include "bmi_zaccel.h" 

#define BMIZIGBEE_VERSION "1.1"

// Global variables

static struct bmi_zb bmi_zb[4];
static int major;

/*
 * 	BMI set up
 */

	// BMI device ID table
static struct bmi_device_id bmi_zb_tbl[] =
{
	{
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT,
		.vendor      = BMI_VENDOR_BUG_LABS,
		.product     = BMI_PRODUCT_ZIGBEE,
		.revision    = BMI_ANY,
	},
	{ 0, },    /* terminate list */
};

MODULE_DEVICE_TABLE(bmi, bmi_zb_tbl);

int  bmi_zb_probe(struct bmi_device *bdev);
void bmi_zb_remove(struct bmi_device *bdev);

static struct semaphore spi_sem;

// BMI driver structure
static struct bmi_driver bmi_zb_driver =
{
	.name     = "bmi_zb",
	.id_table = bmi_zb_tbl,
	.probe    = bmi_zb_probe,
	.remove   = bmi_zb_remove,
};

// IOX
// read byte from I2C IO expander
static int ReadByte_IOX(struct i2c_adapter *adap, unsigned char offset, unsigned char *data)
{
	int    ret = 0;
	struct i2c_msg rmsg[2];
	int    num_msgs;


	// Read Byte with Pointer
	rmsg[0].addr = BMI_IOX_I2C_ADDRESS;
	rmsg[0].flags = 0;                     // write
	rmsg[0].len = 1;
	rmsg[0].buf = &offset;

	rmsg[1].addr = BMI_IOX_I2C_ADDRESS;
	rmsg[1].flags = I2C_M_RD;              // read
	rmsg[1].len = 1;
	rmsg[1].buf = data;

	num_msgs = 2;
	ret = i2c_transfer(adap, rmsg, num_msgs);

	if(ret == 2)
	{
		ret = 0;
	}
	else
	{
		printk(KERN_ERR "ReadByte_IOX() - i2c_transfer() zb failed.\n");
		ret = -1;
	}
	return ret;
}

// IOX
// write byte from I2C IO expander
static int WriteByte_IOX(struct i2c_adapter *adap, unsigned char offset, unsigned char data)
{
	int    ret = 0;
	struct i2c_msg wmsg[2];
	int    num_msgs;

	// Write Byte with Pointer
	wmsg[0].addr = BMI_IOX_I2C_ADDRESS;
	wmsg[0].flags = 0;                     // write
	wmsg[0].len = 1;
	wmsg[0].buf = &offset;

	wmsg[1].addr = BMI_IOX_I2C_ADDRESS;
	wmsg[1].flags = 0;                     // write
	wmsg[1].len = 1;
	wmsg[1].buf = &data;

	num_msgs = 2;
	ret = i2c_transfer(adap, wmsg, num_msgs);
	
	if(ret == 2)
	{
		ret = 0;
	}
	else
	{
		printk(KERN_ERR "WriteByte_IOX() - i2c_transfer() zb failed.\n");
		ret = -1;
	}
	return ret;
}

// char device file operation controls the module LEDs and reset

// open
int cntl_open(struct inode *inode, struct file *file)
{
	struct bmi_zb *zb;
	
	zb = container_of(inode->i_cdev, struct bmi_zb, cdev);

	zb->open_flag++;

	// Save zb pointer for later.
	file->private_data = zb;
	return 0;
}

// release
int cntl_release(struct inode *inode, struct file *file)
{
	struct bmi_zb *zb;
	
	zb = (struct bmi_zb *)(file->private_data);
	zb->open_flag = 0;
	return 0;
}


// ioctl
int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct i2c_adapter *adap;
	struct bmi_zb *zb;
	unsigned char buf[80];
	int slot;
	int i;

	zb = (struct bmi_zb *)(file->private_data);
	
	// error if zb not present
	if(zb->bdev == 0)
		return -ENODEV;

	slot = zb->slot;
	adap = zb->adap;

	zb->adap = adap;

	switch(cmd)
	{
		case BMI_ZB_RLEDOFF:
			bmi_slot_gpio_write_bit(slot, ZB_GPIO_RED_LED, ZB_GPIO_LED_OFF);
			break;

		case BMI_ZB_RLEDON:
			bmi_slot_gpio_write_bit(slot, ZB_GPIO_RED_LED, ZB_GPIO_LED_ON);
			break;

		case BMI_ZB_GLEDOFF:
			bmi_slot_gpio_write_bit(slot, ZB_GPIO_GREEN_LED, ZB_GPIO_LED_OFF);
			break;
	
		case BMI_ZB_GLEDON:
			bmi_slot_gpio_write_bit(slot, ZB_GPIO_GREEN_LED, ZB_GPIO_LED_ON);
			break;

		/* 
		 * Below are unpublished commands, used for testing only.
		 */

		case BMI_ZB_RESET:
			i = (__user arg) & 0xF;

			if(i == 0)
			{
				printk("external reset\n");
				zb_Reset(zb,ZB_RESET);
				mdelay(10);
				zb_Reset(zb,ZB_RELEASE);
			}
			else if (i == 1)
			{
				printk("soft reset\n");
				zb_SoftReset(zb);
			}
			else if (i == 2)
			{
				printk("set startup option to default\n");
				buf[0] = 3;
				zb_WriteConfiguration(zb,ZCD_NV_STARTUP_OPTION,1,(unsigned char *)&buf[0]);
			}

			break;

		case BMI_ZB_SPI_SIG:
			zb_ReadSRDY(zb,1);
			zb_ReadMRDY(zb);
			break;

		case BMI_ZB_LOOPBACK:
			zb_SysTestLoopback(zb);
			break;

		case BMI_ZB_STARTREQ:
			zb_StartRequest(zb);
			break;

		case BMI_ZB_UPDATE_STATE:
			zaccel_getdev(zb);
			break;

		default:
			return -ENOTTY;
	}

	return 0;
			
}

struct file_operations zb_fops =
{
	.owner = THIS_MODULE,
	.ioctl = cntl_ioctl,
	.open = cntl_open,
	.release = cntl_release,
};


void set_MRDY(struct bmi_zb *zb)
{
	/* Assert MRDY and SS pins */
	bmi_slot_gpio_write_bit(zb->slot, ZB_GPIO_MRDY, 0);
	bmi_slot_gpio_write_bit(zb->slot, ZB_GPIO_SS, 0);
}


void clr_MRDY(struct bmi_zb *zb)
{
	/* De-assert MRDY and SS pins */
	bmi_slot_gpio_write_bit(zb->slot, ZB_GPIO_MRDY, 1);
	bmi_slot_gpio_write_bit(zb->slot, ZB_GPIO_SS, 1);
}

static unsigned long jiff_interval(unsigned long start_time)
{
	unsigned long tick;

	tick = jiffies;
	if(tick >= start_time)
	{
		tick = tick - start_time;
	}
	else
	{
		tick = (0xFFFFFFFF - start_time) + tick;
	}

	return tick;
}

void zenable_irq(struct bmi_zb *zb)
{
	zb->enable = 1;
#ifdef VE_OUT
	printk("e-IRQ %d depth %d\n",zb->irq,zb->int_depth);
#endif
	if(zb->int_depth != 0)
	{
		zb->int_depth--;
		enable_irq(zb->irq);
	}
	else
	{
		printk("zenable_irq: unbalance irq %d\n",zb->irq);
	}
}

int wait_for_srdy(struct bmi_zb *zb,unsigned char type)
{
	int rc = 0;
	int err = 0;

	zb->srdy_state = ZB_SPI_WAIT_SRDY_H;
	zb->start_time = jiffies;

	/*
	 * Queue work to poll for SRDY high.
	 * From experimenting in the lab, I found that sometimes it took longer
	 * for the SRDY to go high when the host sent an SREQ command
	 * than when the host responded to the device AREQ (SPI_POLL_TYPE) command.
	 * So we delay the work to poll SRDY in SPI_REQ_TYPE.
	 * This delay is more of a fine tune process, and can be changed
	 * as long as it doesn't exceed the SPI transaction processing
	 * to over 1 second.
	 */

	if(type == SPI_POLL_TYPE)
	{
		zb->delay = 0;
	}
	else if(type == SPI_REQ_TYPE)
	{
		zb->delay = SPI_CHK_SRDY_JIFFIES;
	}

	queue_delayed_work(zb->srdy_wq, &zb->srdy_work,zb->delay);

	/* 
 	 * I use wait_event_interruptible instead of
 	 * wait_event_interruptible_timeout here because we schedule
 	 * work queue to poll SRDY.  The work schedules another
 	 * work queue, if SRDY has not been changed.
 	 * Using wait_event_interruptible_timeout can cause a race
 	 * condition if the wait_event_interrupt_timeout timeout occurs while
 	 * the work queue is running.
 	 *
 	 * The code wakes up when SRDY is high or when the work queue
 	 * decides that it has waited too long.
 	 */
	err = wait_event_interruptible(zb->srdy_queue,
	                         ((zb->srdy_state == ZB_SPI_SRDY_HIGH) || 
	                           (zb->srdy_state == ZB_SPI_SRDY_EXP)));

	if(err != 0)
	{
		printk(KERN_WARNING "zb-%d wait_for_srdy err %d\n",zb->slot,err);
	}

	if(zb->srdy_state == ZB_SPI_SRDY_EXP)
	{
		rc = -1;
	}

	return rc;
}

void zaccel_spi_poll(struct bmi_zb *zb)
{
	struct spi_message msg;
	unsigned char buf[ZCMD_HEADER + 256];
	unsigned char len;
#define AREQ_DEBUGx
#ifdef AREQ_DEBUG
	int i;
#endif

	struct spi_transfer t =
	{
		.len = SPI_MSG_HEADER_SIZE,
		.cs_change = 0,
		.delay_usecs = 0,
	};
	
	down(&spi_sem);
	if(zb_ReadSRDY(zb,0) != 0)
	{
		/* 
 		 * check if SRDY is low.   If not, don't run the routine.
 		 *
 		 * This routine is the IRQ bottom half, scheduled in the IRQ
 		 * service routine.
 		 * Sometimes, the SRDY goes high before the routine is executed.
 		 * That happens when the zaccel_spi_req, which is scheduled to
 		 * run on the same queue, is executed before
 		 * zaccel_spi_poll is executed.   Sometimes,
 		 * the IRQ puts zaccel_spi_poll on the workqueue multiple times,
 		 * each time when it sees SRDY low.   
 		 */
#ifdef DEBUG_OUT
		printk("zaccel_spi_poll SRDY high, depth %d\n",zb->int_depth);
#endif
	
		if(zb->int_depth != 0)
		{
			/* If the interrupt is disabled, enable it */
			zenable_irq(zb);
		}

		up(&spi_sem);
		return;
	}

	set_MRDY(zb);

	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;

	t.tx_buf = (const void *)buf;
	t.rx_buf = buf;
	t.len = SPI_MSG_HEADER_SIZE;

	/* Send three bytes of POLL command */
	spi_message_init(&msg);
	spi_message_add_tail(&t, &msg);

	if((spi_sync(zb->spi, &msg) != 0) || (msg.status != 0))
	{
		/* error reading the data. */
		printk(KERN_WARNING "send spi_sync error %d\n",msg.status);
		clr_MRDY(zb);
		up(&spi_sem);
		zenable_irq(zb);
		return;
	}

	if(wait_for_srdy(zb,SPI_POLL_TYPE) < 0)
	{
		/* time out */
		printk(KERN_WARNING "%d - SRDY() high poll timeout\n",zb->slot);
		clr_MRDY(zb);
		up(&spi_sem);
		zenable_irq(zb);
		return;
	}

	/* Read three bytes to get the message length */
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;

	t.tx_buf = (const void *)buf;
	t.rx_buf = buf;
	t.len = SPI_MSG_HEADER_SIZE,
	t.cs_change = 0,
	t.delay_usecs = 0,

	spi_message_init(&msg);
	spi_message_add_tail(&t, &msg);

	if((spi_sync(zb->spi, &msg) != 0) || (msg.status != 0))
	{
		/* error reading the data. */
		printk(KERN_WARNING "rcv1 spi_sync error %d\n",msg.status);
		clr_MRDY(zb);
		up(&spi_sem);
		zenable_irq(zb);
		return;
	}

	/* buf[0] contains the length of the message */

	if((buf[0] != 0) && (buf[0] != 0xFF))
	{
		/* 
 		 * Read the rest of the message, if length != 0 */
		t.len = buf[0];

		t.rx_buf = &buf[ZCMD_HEADER];
		spi_message_init(&msg);

		spi_message_add_tail(&t, &msg);
		if((spi_sync(zb->spi, &msg) != 0) || (msg.status != 0))
		{
			// error reading the data.   Set length to zero.
			buf[0] = 0;
			printk(KERN_WARNING "rcv2 spi_sync error %d\n",msg.status);
		}
	}
	
	clr_MRDY(zb);
	up(&spi_sem);
	zenable_irq(zb);

	/* buf[0] has message length */
	len = buf[0];

	if((buf[0] != 0xFF) && (buf[0] != 0))
	{
#ifdef AREQ_DEBUG
		printk("AREQ-%d: ",zb->slot);
		for(i = 0; i < (len + 3); i++)
		{
			printk("%x ",buf[i]);
		}

		printk("\n");
#endif
	}
	else
	{
		if(buf[0] == 0xFF)
			printk(KERN_WARNING "invalid 0xFF\n");
		return;
	}

	zaccel_cmd_proc(zb,buf);
	return;
}

void zaccel_chk_srdy(struct work_struct *work)
{
	struct bmi_zb *zb;

	zb = container_of(work, struct bmi_zb, srdy_work);

	/*
	 * check SRDY value.   If it matches what we are waiting
	 * for, we wake up the work on the queue.
	 * If not, we check the timer for timeout.
	 * If the timer is not expired, queue another zb->srdy_work, to
	 * check for the signal next time.
	 * If the timer is expired, set srdy_state to indicate timeout
	 * and wake up the work.
	 */

	if((zb_ReadSRDY(zb,0) == 0) && (zb->srdy_state == ZB_SPI_WAIT_SRDY_L))
	{
		zb->srdy_state = ZB_SPI_SRDY_LOW;
		wake_up_interruptible(&zb->srdy_queue);
		return;
	}
	else if((zb_ReadSRDY(zb,0) == 1) && (zb->srdy_state == ZB_SPI_WAIT_SRDY_H))
	{
		zb->srdy_state = ZB_SPI_SRDY_HIGH;
		wake_up_interruptible(&zb->srdy_queue);
		return;
	}
	else if(jiff_interval(zb->start_time) < SPI_SRDY_H_TIMEOUT)
	{
		queue_delayed_work(zb->srdy_wq, &zb->srdy_work,zb->delay);
	}
	else
	{
		zb_ReadSRDY(zb,1);
		printk(KERN_WARNING "zaccel_chk_srdy state %d\n",zb->srdy_state);
		zb->srdy_state = ZB_SPI_SRDY_EXP;
		wake_up_interruptible(&zb->srdy_queue);
	}

}

/* work handler to process Z-Accel request */
void zaccel_poll_proc(struct work_struct *work)
{
	struct bmi_zb *zb;

	zb = container_of(work, struct bmi_zb, spi_work);

	zaccel_spi_poll(zb);
}

void zaccel_sreq_proc(struct work_struct *work)
{
	struct bmi_zb *zb;

	unsigned char buf[320];
	struct spi_transfer t =
	{
		.tx_buf = (const void *)buf,
		.rx_buf = buf,
		.cs_change = 0,
		.delay_usecs = 0,
	};

	unsigned char *rbuf;
	struct spi_message msg;
	unsigned char type;
	unsigned short len;
	int i;
	int rc = 0;

	zb = container_of(work, struct bmi_zb, sreq_work);

	down(&spi_sem);

	rbuf = zb->sreq_buf;
	len  =  (size_t)(rbuf[0] + ZCMD_HEADER);
	type =  rbuf[1] & SPI_CMD_TYPE_MASK;

	t.len = len;
	t.tx_buf = rbuf;

#define SREQ_DEBUGx
#ifdef SREQ_DEBUG
	printk("SREQ-%d: ",zb->slot);

	for(i = 0; i < len; i++)
	{
		printk("%x ",rbuf[i]);
	}
	printk("\n");
	
#endif
	
	set_MRDY(zb);

	zb->srdy_state = ZB_SPI_WAIT_SRDY_L;

	if(zb->enable == 1)
	{
		rc = wait_event_interruptible_timeout(zb->srdy_queue,zb->srdy_state == ZB_SPI_SRDY_LOW,SPI_SRDY_L_TIMEOUT);
		if(rc == 0)
		{
			printk(KERN_WARNING "bmi_zb-%d: SRDY low timeout\n",zb->slot);
			clr_MRDY(zb);
			zb->srdy_state = ZB_SPI_POLL;
			up(&spi_sem);
			zb->sreq_ret = -1;
			wake_up_interruptible(&zb->sreq_queue);
			return;
		}
	}
	else
	{
		/* 
 		 * The interrupt has been disabled, because SRDY is low
 		 * prior to enter this routine.   We proceed ahead with the
 		 * SPI transaction.
 		 */

		if(zb_ReadSRDY(zb,0) != 0)
		{
 		 	/* Double check that SRDY is low, if not, give warning */
			printk(KERN_WARNING "zaccel_req_proc interrupt disable SRDY high\n");
		}
	}

	i = 0;

	spi_message_init(&msg);
	spi_message_add_tail (&t, &msg);

	if(spi_sync(zb->spi, &msg) != 0 || msg.status != 0)
	{
		printk(KERN_WARNING "bmi_zb: a - spi_sync error %d\n",msg.status);
		zb->sreq_ret = -ENODEV;
		goto done;
	}

	if(wait_for_srdy(zb,SPI_REQ_TYPE) < 0)
	{
		printk(KERN_WARNING "bmi_zb: SRDY wait to go high timeout\n");
		zb->sreq_ret = -1;
		goto done;
	}

	if(type == SPI_CMD_AREQ)
	{
		zb->sreq_ret = 0;
		goto done;
	}

	buf[0] = 0;             /* Poll command has zero byte */
	buf[1] = 0;             /* Poll command */
	buf[2] = 0;             /* Poll command */

	t.tx_buf = buf;
	t.len = SPI_MSG_HEADER_SIZE;

	spi_message_init(&msg);

	spi_message_add_tail(&t, &msg);
	if(spi_sync(zb->spi, &msg) != 0 || msg.status != 0)
	{
		printk(KERN_WARNING "bmi_zb: b- spi_sync error %d\n",msg.status);
		zb->sreq_ret = -ENODEV;
		goto done;
	}

	/* Read the length of the data */
	if(buf[0] != 0)
	{
		// Set len to the length of the message and offset 
		// the buffer to after the header field.
		t.len = buf[0];

		t.tx_buf = (unsigned char *)buf + SPI_MSG_HEADER_SIZE;
		t.rx_buf = (unsigned char *)buf + SPI_MSG_HEADER_SIZE;
		spi_message_init(&msg);

		spi_message_add_tail(&t, &msg);

		rc = spi_sync(zb->spi, &msg);
		clr_MRDY(zb);

		if(rc != 0 || msg.status != 0)
		{
			printk(KERN_WARNING "bmi_zb: c - spi_sync error %d\n",msg.status);
			zb->sreq_ret = -ENODEV;
			goto done;
		}
	}

	clr_MRDY(zb);
	up(&spi_sem);
	zenable_irq(zb);

	/* 
 	 * copy data back to the buffer.  Make sure that we don't
 	 * copy more data than the space available.
 	 */

	if(buf[0] != 0xFF)
	{
		/* copy data to the return buffer, as much as the length
		 * of the data or the buffer size (zb->sreq_len)
		 */
		if(zb->sreq_len > (buf[0] + 3))
		{
			len = buf[0] + 3;
		}
		else
		{
			len = zb->sreq_len;
		}
		memcpy(rbuf,buf,len);
		zb->sreq_ret = zb->sreq_len;
	}
	else
	{
		zb->sreq_ret = -2;
	}

	/* We're done.   Wake up the SREQ work */
	zb->srdy_state = ZB_SPI_POLL;
	wake_up_interruptible(&zb->sreq_queue);

#define SRSP_DEBUGx
#ifdef SRSP_DEBUG
	printk("SRSP-%d: ",zb->slot);
	if(buf[0] != 0xFF)
	{
		for(i = 0; i < len; i++)
		{
				printk("%x ",buf[i]);
		}
	}
	else
	{
		printk("Invalid length\n");
	}
	printk("\n");
#endif

	/* 
 	 * Sometimes, back-to-back write configuration
 	 * can chock the Z-Accel, result in fail SPI transaction.
 	 * Delay the return (wait_event_interruptible_timeout
 	 * will timeout - no one wakes up the queue), to prevent the problem.
 	 */
	rc = 0;
	wait_event_interruptible_timeout(zb->delay_queue,(rc != 0), 5);

	return;

done:
	clr_MRDY(zb);
	zb->srdy_state = ZB_SPI_POLL;
	up(&spi_sem);
	zenable_irq(zb);
	wake_up_interruptible(&zb->sreq_queue);
	return;
}

int config_ports(struct bmi_zb *zb)
{
	struct i2c_adapter *adap;
	int slot;
	unsigned char iox_data;

	slot = zb->slot;
	adap = zb->adap;

	/* 
 	 * Configure GPIO_RED_LED and GPIO_GREEN_LED as output
	 * and set them to low -- LEDs on
	 */

	bmi_slot_gpio_configure_as_output(slot,ZB_GPIO_RED_LED,0);
	bmi_slot_gpio_configure_as_output(slot,ZB_GPIO_GREEN_LED,0);

	/* 
 	 * Configure GPIO_SS and GPIO_MRDY as output 
	 * and set them to high -- deassert
	 */

	bmi_slot_gpio_configure_as_output(slot,ZB_GPIO_MRDY,1);
	bmi_slot_gpio_configure_as_output(slot,ZB_GPIO_SS,1);

	/* 
 	 * Set ZB_RST to output port.
	 * Set LSR_MRDY_IOX and LSR_SRDY_IOX to input port.
	 */

	if(ReadByte_IOX(adap, IOX_CONTROL, &iox_data))
	{
		printk(KERN_ERR "Unable to ReadByte_IOX - zb slot %d\n",slot);
		return -ENODEV;
	}	

	/* Set SRDY and MRDY port to input and RST to output */
	iox_data |= (ZB_IOX_SRDY | ZB_IOX_MRDY);
	iox_data &= ~(ZB_IOX_RST);

	if(WriteByte_IOX(adap, IOX_CONTROL, iox_data))
	{
		printk(KERN_ERR "Unable to WriteByte_IOX - zb slot %d\n",slot);
		return -ENODEV;
	}	

	return 0;
}

// interrupt handler
static irqreturn_t module_irq_handler(int irq, void *dummy)
{
	struct bmi_zb *zb;

	disable_irq_nosync(irq);

	switch(irq)
	{
		case M1_IRQ:
			zb = &bmi_zb[0];
			break;
		case M2_IRQ:
			zb = &bmi_zb[1];
			break;
		case M3_IRQ:
			zb = &bmi_zb[2];
			break;
		case M4_IRQ:
			zb = &bmi_zb[3];
			break;
		default:
			return IRQ_HANDLED;
	}

	zb->enable = 0;
	zb->int_depth++;

	if(zb->srdy_state == ZB_INT_WAIT_SRDY)
	{
		/*
 		 * This interrupt is a part of host SREQ or AREQ.
		 * Set srdy flag to 0 to indicate that SRDY pin is asserted.
		 * Wake up the zb->spi_queue that is waiting for the event.
		 */
		zb->srdy_state = ZB_INT_SRDY_LOW;
		wake_up_interruptible(&zb->srdy_queue);
	}
	else
	{
		/* 
 		 * Z-Accel has an AREQ frame to send.
		 * Schedule SPI receive task to pull data out.
		 */
		queue_work(zb->spi_wq, &zb->spi_work);
	}

	return IRQ_HANDLED;
}

/*
 * 	BMI functions
 */

int bmi_zb_probe(struct bmi_device *bdev)
{
	struct bmi_zb *zb;
	struct i2c_adapter *adap;
	struct class *bmi_class;
	struct cdev *cdev;
	struct net_device *netdev;
	struct net_zb *priv;
	dev_t dev_id;
	int slot;
	int irq;
	int err;
	char name[IFNAMSIZ];

	err = 0;
	slot = bmi_device_get_slot(bdev);
	adap = bmi_device_get_i2c_adapter(bdev);

	zb = &bmi_zb[slot];

	zb->slot = slot;
	zb->adap = adap;
	
	dev_id = MKDEV(major, slot);

	// Initialize GPIOs, turn on Red and Green LEDs.
	if(config_ports(zb))
	{
		printk(KERN_ERR "Unable to configure ZB port pins slot %d\n",(slot+1));
		return -EFAULT;
	}

	// Hold Z-Accel reset
	zb_Reset(zb,ZB_RESET);

	// setup SPI
	printk(KERN_INFO "ZB SPI_MODE_2 clock %d\n",ZB_SPI_SPEED);
	if(bmi_device_spi_setup(bdev, ZB_SPI_SPEED, SPI_MODE_2, ZB_SPI_BPW))
	{
		printk(KERN_ERR "Unable to setup spi\n");
		return -EFAULT;
	}
	bmi_slot_spi_enable(slot);

	zb->spi = &bdev->spi;
	zb->srdy_state = ZB_SPI_POLL;

	INIT_WORK(&zb->spi_work, zaccel_poll_proc);
	INIT_WORK(&zb->sreq_work, zaccel_sreq_proc);
	INIT_DELAYED_WORK(&zb->srdy_work, zaccel_chk_srdy);

	init_waitqueue_head(&zb->sreq_queue);
	init_waitqueue_head(&zb->srdy_queue);
	init_waitqueue_head(&zb->delay_queue);

	/*
	 * create a thread to handle SPI access.
	 * spi_sem allows one ZigBee module to perform and complete its
	 * SPI transaction before other Zigbee does its SPI.
	 */
	if(slot == 0)
	{
		zb->spi_wq = create_singlethread_workqueue("zaccel_spi0");
		zb->srdy_wq = create_singlethread_workqueue("zaccel_srdy0");
	}
	else if(slot == 1)
	{
		zb->spi_wq = create_singlethread_workqueue("zaccel_spi1");
		zb->srdy_wq = create_singlethread_workqueue("zaccel_srdy1");
	}
	else if(slot == 2)
	{
		zb->spi_wq = create_singlethread_workqueue("zaccel_spi2");
		zb->srdy_wq = create_singlethread_workqueue("zaccel_srdy2");
	}
	else if(slot == 3)
	{
		zb->spi_wq = create_singlethread_workqueue("zaccel_spi3");
		zb->srdy_wq = create_singlethread_workqueue("zaccel_srdy3");
	}


	if((!zb->spi_wq) && (!zb->srdy_wq))
	{
		printk(KERN_ERR "ZB: create workqueue failed %d\n",slot);
		if(zb->spi_wq)
		{
			destroy_workqueue(zb->spi_wq);
		}

		if(zb->srdy_wq)
		{
			destroy_workqueue(zb->srdy_wq);
		}
			
		bmi_slot_spi_disable(slot);
		bmi_device_spi_cleanup(bdev);
		return -ENOMEM;
	}

	// request PIM interrupt
	irq = bmi_device_get_status_irq(bdev);
	zb->irq = irq;
	zb->int_depth = 0;

	sprintf(zb->int_name, "bmi_zb%d", slot);
	err = request_irq(irq, &module_irq_handler, 0, zb->int_name, zb);
	if(err)
	{
		printk(KERN_ERR "bmi_zb.c: Can't allocate irq %d nor find ZB in slot %d \n", irq, slot);
		destroy_workqueue(zb->spi_wq);
		destroy_workqueue(zb->srdy_wq);
		bmi_slot_spi_disable(slot);
		bmi_device_spi_cleanup(bdev);
		return -EBUSY;
	}

	printk(KERN_INFO "bmi_zb.c: ZIGBEE create class device\n");
	bmi_class = bmi_get_bmi_class();
	zb->class_dev = device_create(bmi_class, NULL, dev_id, zb, 
	                                       "bmi_zb%i", slot + 1);

	if(IS_ERR(zb->class_dev))
	{
		printk(KERN_ERR "Unable to create "
		                "class_device for bmi_zb_%i; errno = %ld\n",
		                slot+1, PTR_ERR(zb->class_dev));
		zb->class_dev = NULL;
		err = -ENODEV;
		goto error;
	}

	/* Allocate network device */
	sprintf(name, "zb%d",(slot+1));
	netdev = alloc_netdev(sizeof(struct net_zb),name,zb_net_setup);
	if(!netdev)
	{
		printk(KERN_ERR "zb%d: cannot register net device \n",slot);
		err = -ENOMEM;
		goto error0;
		
	}

	priv = netdev_priv(netdev);
	priv->zb = zb;

	err = init_zaccel(zb);
	if(err < 0)
	{
		printk(KERN_ERR "zb%d: Z-Accel device not start\n",(slot+1));
		goto error1;
	}

	err = register_netdev(netdev);
	if(err < 0)
	{
		printk(KERN_WARNING "zb: cannot register net device\n");
		goto error1;
	}

	zb_create_sysfs(netdev);

	zb->netdev = netdev;

	// bind driver and bmi_device
	zb->bdev = bdev;
	bmi_device_set_drvdata(bdev,zb);

	cdev = &zb->cdev;
	cdev_init(cdev, &zb_fops);
	err = cdev_add(cdev, dev_id, 1);
	if(err < 0)
	{
		printk(KERN_ERR "Unable to add cdev for ZigBee module\n");
		goto error2;
	}

	/* turn LED's off */
	bmi_set_module_gpio_data(slot, ZB_GPIO_RED_LED, ZB_GPIO_LED_OFF);
	bmi_set_module_gpio_data(slot, ZB_GPIO_GREEN_LED, ZB_GPIO_LED_OFF);

	return 0;

error2:
	zb_remove_sysfs(netdev);
	zb->bdev = NULL;
	bmi_device_set_drvdata(bdev,0);

	zb->netdev = NULL;
	unregister_netdev(netdev);

error1:
	remove_zaccel(zb);
	free_netdev(netdev);

error0:
	zb->class_dev = NULL;
	device_destroy(bmi_class,dev_id);

error:

	free_irq(irq, zb);
	zb->irq = 0;
	destroy_workqueue(zb->spi_wq);
	destroy_workqueue(zb->srdy_wq);
	bmi_slot_spi_disable(slot);
	bmi_device_spi_cleanup(bdev);

	printk(KERN_ERR "bmi_zb: modprobe error %d\n",err);
	return err;
}

/* remove PIM */
void bmi_zb_remove(struct bmi_device *bdev)
{
	int slot;
	int irq;
	struct bmi_zb *zb;
	struct class *bmi_class;
	struct net_device *netdev;

	slot = bmi_device_get_slot(bdev);
	zb = &bmi_zb[slot];

	/* Free the interrupt first.  This is to prevent stranded interrupt 
  	 * when we hold the Z-Accel reset.
  	 */
	irq = bmi_device_get_status_irq(bdev);
	free_irq(irq, zb);

	destroy_workqueue(zb->spi_wq);
	destroy_workqueue(zb->srdy_wq);
	
	remove_zaccel(zb);

	cdev_del(&zb->cdev);

	/* Unregister and deallocate net_device  */
	netdev = zb->netdev;
	zb_remove_sysfs(netdev);
	unregister_netdev(netdev);
	free_netdev(netdev);
	zb->netdev = NULL;

	zb->irq = 0;
	zb->spi = (struct spi_device *)NULL;
	bmi_slot_spi_disable(slot);
	bmi_device_spi_cleanup(bdev);

	bmi_slot_gpio_configure_all_as_inputs(slot);

	bmi_class = bmi_get_bmi_class();
	device_destroy(bmi_class, MKDEV(major,slot));
	zb->class_dev = 0;

	/* de-attach driver-specific struct from bmi_device structure */
	bmi_device_set_drvdata(bdev,0);
	zb->bdev = 0;

	printk(KERN_INFO "bmi_zb: remove completed\n");
	return;
}

int zaccel_spi_req(struct bmi_zb *zb, unsigned char *rbuf, unsigned short buf_len)
{
	int err;

	zb->sreq_buf = rbuf;
	zb->sreq_len = buf_len;

	queue_work(zb->spi_wq, &zb->sreq_work);
	zb->sreq_ret = 0xFF;
	err = wait_event_interruptible_timeout(zb->sreq_queue,zb->sreq_ret != 0xFF,(5*HZ));
	if(err == 0)
	{
		printk(KERN_ERR "zaccel_spi_req timeout %d\n",zb->sreq_ret);
		zb->sreq_ret = -ENODEV;
	}

	return zb->sreq_ret;
}

void zb_ReadMRDY(struct bmi_zb *zb)
{
	int mrdy;
	int ss;

	mrdy = bmi_slot_gpio_read_bit(zb->slot,ZB_GPIO_MRDY);
	ss = bmi_slot_gpio_read_bit(zb->slot,ZB_GPIO_SS);
	printk("zb-SPI: MRDY %d SS %d\n",mrdy,ss);
}

int zb_ReadSRDY(struct bmi_zb *zb, int print)
{
	int irq_pin;
	int value;

	irq_pin = bmi_slot_status_irq_state(zb->slot);
	
	if(irq_pin == 1)
		value = 0;
	else
		value = 1;

	if(print == 1)
		printk("zb-SPI: srdy %d\n",value); 
	return value;
}

/* Z-Accel hardware reset */
int zb_Reset(struct bmi_zb *zb, unsigned char state)
{
	unsigned char iox_data;

	if(ReadByte_IOX (zb->adap, IOX_OUTPUT_REG, &iox_data))
		return -ENODEV;

	if(state == ZB_RESET)
	{
		iox_data &= ~ZB_IOX_RST;
	}
	else if(state == ZB_RELEASE)
	{
		zb->z_info.msg_flag &= ~RESET_IND_BIT;
		iox_data |= ZB_IOX_RST;
	}

	if(WriteByte_IOX (zb->adap, IOX_OUTPUT_REG, iox_data))
		return -ENODEV;

	return 0;
}

static void __exit bmi_zb_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver(&bmi_zb_driver);

	/* Unregister PF_ZACCEL socket */
	z_sock_exit();

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region(dev_id, 4);

	return;
}

static int __init bmi_zb_init(void)
{
	dev_t dev_id;
	int   retval;

	// allocate char device for the module control.
	retval = alloc_chrdev_region(&dev_id, 0, 4, "BMI ZigBee Driver");
	if(retval)
	{
		printk(KERN_ERR "Unable to allocate zb chardev_region\n");
		return -1;
	}
	major = MAJOR(dev_id);

	/* Register PF_ZACCEL protocol socket */
	retval = z_sock_init();
	if(retval)
	{
		unregister_chrdev_region(dev_id, 4);
		printk(KERN_ERR "ZB: protocol register failed %d \n", retval);
		return -1;
	}

	init_MUTEX(&spi_sem);

	retval = bmi_register_driver(&bmi_zb_driver);
	if(retval)
	{
		z_sock_exit();
		unregister_chrdev_region(dev_id, 4);

		printk(KERN_ERR "ZB: bmi_unregister_driver failed %d\n", retval);
		return -1;
	}

	printk(KERN_INFO "bmi_zb.c: BMI_ZIGBEE Driver v%s 0x%x\n", BMI_ZB_VERSION,BMI_PRODUCT_ZIGBEE);

	return 0;
}


module_init(bmi_zb_init);
module_exit(bmi_zb_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("V. Thavisri <v.thavisri@encadis.com>");
MODULE_DESCRIPTION("BMI ZigBee device driver");
MODULE_SUPPORTED_DEVICE("bmi_zigbee_control");
