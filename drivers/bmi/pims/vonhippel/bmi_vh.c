/*
 * 	bmi_vh.c
 *
 * 	BMI von Hippel device driver basic functionality
 *
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
 *	Include files
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/hardware.h>

#include <linux/i2c.h>

#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_vh.h>

#define BMIVH_VERSION	"1.0"
#define RDAC_3_3V	(0xAC)	// 16.5K = 3.3V

/*
 * 	Global variables
 */

static struct i2c_board_info iox_info = {
  I2C_BOARD_INFO("VH_IOX", BMI_IOX_I2C_ADDRESS),
};

static struct i2c_board_info rdac_info = {
  I2C_BOARD_INFO("VH_RDAC", VH_RDAC_I2C_ADDRESS),
};

static struct i2c_board_info adc_info = {
  I2C_BOARD_INFO("VH_ADC", VH_ADC_I2C_ADDRESS),
};

static struct i2c_board_info dac_info = {
  I2C_BOARD_INFO("VH_DAC", VH_DAC_I2C_ADDRESS),
};


static ushort factory_test = 0;
static ushort fcc_test = 0;
static struct timer_list fcc_timer;
static int fcc_state = 0x3;

// private device structure
struct bmi_vh
{
  struct bmi_device	*bdev;			// BMI device
  struct cdev		cdev;			// control device
  struct device	*class_dev;		// control class device
  int			open_flag;		// single open flag
  char			int_name[20];		// interrupt name
  struct i2c_client *iox;
  struct i2c_client *rdac;
  struct i2c_client *dac;
  struct i2c_client *adc;
  struct spi_device *spi;	// SPI device
  struct spi_board_info vh_spi_info;
  char			rbuf[BUF_MAX_SIZE];	// SPI read buffer
  char			wbuf[BUF_MAX_SIZE];	// SPI write buffer
};

static struct bmi_vh bmi_vh[4];	// per slot device structure
static int major;		// control device major

/*
 * 	BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_vh_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_VON_HIPPEL, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },	  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_vh_tbl);

int	bmi_vh_probe (struct bmi_device *bdev);
void	bmi_vh_remove (struct bmi_device *bdev);
int	bmi_vh_resume (struct device *dev);
int	bmi_vh_suspend (struct device *dev);

static struct dev_pm_ops bmi_vh_pm =
{
	.resume = bmi_vh_resume,
	.suspend = bmi_vh_suspend,
};

// BMI driver structure
static struct bmi_driver bmi_vh_driver = 
{
	.name = "bmi_vh", 
	.id_table = bmi_vh_tbl, 
	.probe   = bmi_vh_probe, 
	.remove  = bmi_vh_remove, 
	.pm  = &bmi_vh_pm,
};

/*
 * 	I2C set up
 */

// IOX
// read byte from I2C IO expander
static int ReadByte_IOX (struct i2c_client *client, unsigned char offset, unsigned char *data)
{
	int	ret = 0;	

	ret = i2c_master_send(client, &offset, 1);
	if (ret == 1)
	  ret = i2c_master_recv(client, data, 1);
	if (ret < 0)
	  printk (KERN_ERR "ReadByte_IOX() - i2c_transfer() failed...%d\n",ret);
	return ret;
}

// write byte to I2C IO expander
static int WriteByte_IOX (struct i2c_client *client, unsigned char offset, unsigned char data)
{
	int	ret = 0;
	unsigned char msg[2];
	
     	msg[0] = offset;
	msg[1] = data;
	ret = i2c_master_send(client, msg, sizeof(msg));
	
	if (ret < 0)
	  printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed...%d\n",ret);

	return ret;
}

// RDAC
// read byte from I2C LDO RDAC
static int ReadByte_RDAC (struct i2c_client *client, unsigned char command, unsigned char *data)
{
	int	ret = 0;
	
	ret = i2c_master_send(client, &command, 1);
	
	if (ret < 0)
	  {
	    printk (KERN_ERR "ReadByte_RDAC() - i2c_master_send() failed...%d\n",ret);
	    return ret;
	  }
	ret = i2c_master_recv(client, data, 1);
	if (ret < 0)
	  printk (KERN_ERR "ReadByte_RDAC() - i2c_master_recv() failed...%d\n",ret);
	return ret;
}

// write byte to I2C LDO RDAC
static int WriteByte_RDAC (struct i2c_client *client, unsigned char command, 
				unsigned char data, int send_data)
{
	int	ret = 0;
	
	unsigned char msg[2];
	
     	msg[0] = command;
	msg[1] = data;
	
	if (send_data)
	  ret = i2c_master_send(client, msg, 2);
	else
	  ret = i2c_master_send(client, &msg[0], 1);
	if (ret < 0)
	  printk (KERN_ERR "WriteByte_RDAC() - i2c_transfer() failed...%d\n",ret);

	return ret;
}

// ADC
// read data from I2C ADC
static int ReadByte_ADC (struct i2c_client *client, unsigned char *data)
{
	int	ret = 0;
	
	ret = i2c_master_recv(client, data, 3);

	if (ret < 0)
	  printk (KERN_ERR "ReadByte_ADC() - i2c_transfer() failed...%d\n",ret);
	return ret;
}

// write command to I2C ADC
static int WriteByte_ADC (struct i2c_client *client, unsigned char w1, unsigned char w2)
{
	int	ret = 0;	
	unsigned char msg[2];
	
     	msg[0] = w1;
	msg[1] = w2;
	ret = i2c_master_send(client, msg, sizeof(msg));

	if (ret < 0)
	  printk (KERN_ERR "WriteByte_ADC() - i2c_transfer() failed...%d\n",ret);
	return ret;
}

// DAC
// read data from I2C DAC
static int ReadByte_DAC (struct i2c_client *client, unsigned char command, unsigned char *data)
{
	int	ret = 0;

	ret = i2c_master_send(client, &command, 1);
	if (ret == 1)
	  ret = i2c_master_recv(client, data, 2);

	if (ret < 0)
	  printk (KERN_ERR "ReadByte_DAC() - i2c_transfer() failed...%d\n",ret);
	return ret;
}

// write command to I2C DAC
static int WriteByte_DAC (struct i2c_client *client, unsigned char w1, unsigned char w2, int send_w2)
{
	int	ret = 0;
	unsigned char msg[2];
	
     	msg[0] = w1;
	msg[1] = w2;
	if (send_w2)
	  ret = i2c_master_send(client, msg, sizeof(msg));
	else
	  ret = i2c_master_send(client, &msg[0], 1);

	if (ret < 0)
	  printk (KERN_ERR "WriteByte_DAC() - i2c_transfer() failed...%d\n",ret);
	return ret;
}

/*
 *	control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *file)
{	
	struct bmi_vh *vh;

	vh = container_of (inode->i_cdev, struct bmi_vh, cdev);

	// Enforce single-open behavior

	if (vh->open_flag) {
		return -EBUSY; 
	}
	vh->open_flag = 1;

	// Save vh_dev pointer for later.

	file->private_data = vh;
	return 0;

}

// release
int cntl_release(struct inode *inode, struct file *file)
{	
	struct bmi_vh *vh;

	vh = (struct bmi_vh *)(file->private_data);
	vh->open_flag = 0;
	return 0;
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct i2c_adapter *adap;
	unsigned char iox_data;
	unsigned char rdac_data;
	//	unsigned char buf[4];
	int ret = 0;

	struct bmi_vh *vh;
	int slot;

	vh = (struct bmi_vh *)(file->private_data);

	// error if vh not present
	if(vh->bdev == 0)
		return -ENODEV;
	
	slot = vh->bdev->slot->slotnum;
	adap = vh->bdev->slot->adap;

	// ioctl's
	switch (cmd) {

	case BMI_VH_RLEDOFF:
	  bmi_slot_gpio_set_value (slot, VH_GPIO_RED_LED, 1); // Red LED=OFF 
		break;

	case BMI_VH_RLEDON:
	  bmi_slot_gpio_set_value (slot, VH_GPIO_RED_LED, 0); // Red LED=ON 
		break;

	case BMI_VH_GLEDOFF:
	  bmi_slot_gpio_set_value (slot, VH_GPIO_GREEN_LED, 1); // Green LED=OFF 
	  break;

	case BMI_VH_GLEDON:
	  bmi_slot_gpio_set_value (slot, VH_GPIO_GREEN_LED, 0); // Green LED=ON
	  break;

	case BMI_VH_GETSTAT:
		{
			int read_data;
	
			if (ReadByte_IOX (vh->iox, IOX_INPUT_REG, &iox_data) < 0)
				return -ENODEV;			
			read_data = iox_data | (bmi_slot_gpio_get_all(slot) << 8);

			if (put_user (read_data, (int __user *) arg))
				return -EFAULT;
		}
		break;

	case BMI_VH_MKGPIO_OUT:
		if ((arg < VH_GPIO_0) || (arg > VH_GPIO_RED_LED))
			return -EINVAL;
		bmi_slot_gpio_direction_out (slot, arg, 1);
		break;

	case BMI_VH_MKGPIO_IN:
		if ((arg < VH_GPIO_0) || (arg > VH_GPIO_RED_LED))
			return -EINVAL;
		bmi_slot_gpio_direction_in (slot, arg);
		break;

	case BMI_VH_SETGPIO:
		if ((arg < VH_GPIO_0) || (arg > VH_GPIO_RED_LED))
			return -EINVAL;
		bmi_slot_gpio_set_value (slot, arg, 0x1);
		break;

	case BMI_VH_CLRGPIO:
		if ((arg < VH_GPIO_0) || (arg > VH_GPIO_RED_LED))
			return -EINVAL;
		bmi_slot_gpio_set_value (slot, arg, 0x0);
		break;

	case BMI_VH_MKIOX_OUT:
		if ((arg < VH_IOX_B0) || (arg > VH_IOX_B5))
			return -EINVAL;
		{
			unsigned char read_data;
	
			if (ReadByte_IOX (vh->iox, IOX_CONTROL, &iox_data) < 0)
				return -ENODEV;
		
			read_data = iox_data & ~(0x1 << arg);

			if (WriteByte_IOX (vh->iox, IOX_CONTROL, read_data) < 0)
				return -ENODEV;
		}
		break;

	case BMI_VH_MKIOX_IN:
		if ((arg < VH_IOX_B0) || (arg > VH_IOX_B5))
			return -EINVAL;
		{
			unsigned char read_data;
	
			if (ReadByte_IOX (vh->iox, IOX_CONTROL, &iox_data) < 0)
				return -ENODEV;
		
			read_data = iox_data & (0x1 << arg);

			if (WriteByte_IOX (vh->iox, IOX_CONTROL, read_data) < 0)
				return -ENODEV;
		}
		break;

	case BMI_VH_SETIOX:
		if ((arg < VH_IOX_B0) || (arg > VH_IOX_USB_VEN))
			return -EINVAL;
		{
			unsigned char read_data;
	
			if (ReadByte_IOX (vh->iox, IOX_OUTPUT_REG, &iox_data) < 0)
				return -ENODEV;
		
			read_data = iox_data | (0x1 << arg);

			if (WriteByte_IOX (vh->iox, IOX_OUTPUT_REG, read_data) < 0)
				return -ENODEV;
		}
		break;

	case BMI_VH_CLRIOX:
		if ((arg < VH_IOX_B0) || (arg > VH_IOX_USB_VEN))
			return -EINVAL;
		{
			unsigned char read_data;
	
			if (ReadByte_IOX (vh->iox, IOX_OUTPUT_REG, &iox_data) < 0)
				return -ENODEV;
		
			read_data = iox_data & ~(0x1 << arg);

			if (WriteByte_IOX (vh->iox, IOX_OUTPUT_REG, read_data) < 0)
				return -ENODEV;
		}
		break;

	case BMI_VH_SETRDAC:
		rdac_data = (unsigned char) (arg & 0xFF);

		if (WriteByte_RDAC (vh->rdac, VH_RD_CMD_RDAC, rdac_data, 1) < 0)
			return -ENODEV;

		if (WriteByte_RDAC (vh->rdac, VH_RD_CMD_EE, rdac_data, 1) < 0)
			return -ENODEV;

		break;

	case BMI_VH_RDRDAC:

		if (ReadByte_RDAC (vh->rdac, VH_RD_CMD_RDAC, &rdac_data) < 0)
			return -ENODEV;

		if(copy_to_user((unsigned int *) arg, &rdac_data, sizeof(int)))
			ret = -EFAULT;
		else
			ret = 0;

		break;

	case BMI_VH_ADCWR:
		{
			struct vh_adc_wr *adc_wr = NULL;

			if ((adc_wr = kmalloc(sizeof(struct vh_adc_wr), GFP_KERNEL)) == NULL)
				return -ENOMEM;
			if (copy_from_user(adc_wr, (struct vh_adc_wr *)arg, sizeof(struct vh_adc_wr))) {
				kfree (adc_wr);
				return -EFAULT;
			}
			if (WriteByte_ADC (vh->adc, adc_wr->w1, adc_wr->w2) < 0) {
				kfree (adc_wr);
				return -ENODEV;
			}
			kfree (adc_wr);
		}
		break;

	case BMI_VH_ADCRD:
		{
			unsigned char adc_data[3];
			unsigned int ret_data;

			if (ReadByte_ADC(vh->adc, adc_data) < 0)	// read ADC conversion
				return -ENODEV;
	
			ret_data = (unsigned int) ((adc_data[0] << 16) | (adc_data[1] << 8) | adc_data[2]);
			if(copy_to_user((unsigned int *) arg, &ret_data, sizeof(int)))
				ret = -EFAULT;
			else
				ret = 0;
		}

		break;

	case BMI_VH_DACWR:
		{
			struct vh_dac_wr *dac_wr = NULL;

			if ((dac_wr = kmalloc(sizeof(struct vh_dac_wr), GFP_KERNEL)) == NULL)
				return -ENOMEM;
			if (copy_from_user(dac_wr, (struct vh_dac_wr *)arg, sizeof(struct vh_dac_wr))) {
				kfree (dac_wr);
				return -EFAULT;
			}
			if (dac_wr->w1 == VH_DAC_W1_UALL) {
				if (WriteByte_DAC (vh->dac, dac_wr->w1, dac_wr->w2, 0) < 0) {
					kfree (dac_wr);
					return -ENODEV;
				}
			} else {
				if (WriteByte_DAC (vh->dac, dac_wr->w1, dac_wr->w2, 1) < 0) {
					kfree (dac_wr);
					return -ENODEV;
				}
			}
			kfree (dac_wr);
		}
		break;

	case BMI_VH_DACRD:
		{
			unsigned char dac_data[2];
			unsigned int command;
			unsigned int ret_data;

			if (copy_from_user(&command, (unsigned int *)arg, sizeof(int))) {
				return -EFAULT;
			}

			if (!((command == VH_DAC_W1_RDA) || (command == VH_DAC_W1_RDB))) {
				return -EINVAL;
			}

			if (ReadByte_DAC(vh->dac, (unsigned char) command, dac_data) < 0) { // read DAC value
				return -ENODEV;
			}
	
			ret_data = (unsigned int) ((dac_data[0] << 8) | dac_data[1]);
			if(copy_to_user((unsigned int *) arg, &ret_data, sizeof(int))) {
				ret = -EFAULT;
			} else {
				ret = 0;
			}
		}

		break;
	case BMI_VH_SUSPEND:
		vh->bdev->dev.bus->pm->suspend(&vh->bdev->dev);
		break;
	case BMI_VH_RESUME:
		vh->bdev->dev.bus->pm->resume(&vh->bdev->dev);
		break;
	default:
		return -ENOTTY;
	}

	return 0;
}

// control file operations
struct file_operations cntl_fops = {
	.owner = THIS_MODULE, 
	.ioctl = cntl_ioctl, 
	.open = cntl_open, 
	.release = cntl_release, 
};

/*
 *	PIM functions
 */

// FCC test timer
void ftimer(unsigned long arg)
{
	struct bmi_vh *bmi_vh = (struct bmi_vh *) arg;
	int slot = bmi_vh->bdev->slot->slotnum;

	/*	bmi_set_module_gpio_data (slot, VH_GPIO_RED_LED, (fcc_state & 0x2) >> 1);
		bmi_set_module_gpio_data (slot, VH_GPIO_GREEN_LED, fcc_state & 0x1);*/
	fcc_state = (fcc_state + 1) % 4;
	del_timer (&fcc_timer);
	fcc_timer.expires = jiffies + (2 * HZ);
	add_timer (&fcc_timer);			
}

// interrupt handler
static irqreturn_t module_irq_handler(int irq, void *dummy)
{
  /*if (!factory_test) {
		printk (KERN_ERR "Von Hippel USB power error - slot ");
		switch (irq) {
			case M1_IRQ:
				printk (KERN_ERR "1 - powering off\n");
				bmi_slot_power_off (0);
				break;
			case M2_IRQ:
				printk (KERN_ERR "2 - powering off\n");
				bmi_slot_power_off (1);
				break;
			case M3_IRQ:
				printk (KERN_ERR "3 - powering off\n");
				bmi_slot_power_off (2);
				break;
			case M4_IRQ:
				printk (KERN_ERR "3 - powering off\n");
				bmi_slot_power_off (3);
				break;
		}
	}
	disable_irq(irq);*/
	return IRQ_HANDLED;
}

/*
 * 	BMI functions
 */

// probe - insert PIM
int bmi_vh_probe(struct bmi_device *bdev)
{
	int err;
	int slot;
	struct bmi_vh *vh;
       	struct i2c_adapter *adap;
	struct cdev *cdev;
	struct class *bmi_class;
	dev_t dev_id;
	int irq;
	unsigned char rdac_data[1];
	unsigned char adc_data[3];
	unsigned char dac_data[2];
	unsigned long speed = 1000000;
	unsigned char mode = SPI_MODE_2; // von Hippel chip select must be low active
	unsigned char bits_per_word = 32;
	unsigned char iox_data;
	unsigned char buf[4];
	struct spi_xfer spi_xfer;
	int gpio_int;

	err = 0;
	slot = bdev->slot->slotnum;
      	adap = bdev->slot->adap;
	vh = &bmi_vh[slot];

	vh->bdev = 0;
	vh->open_flag = 0;
	
	// Create 1 minor device
	cdev = &vh->cdev;
	cdev_init (cdev, &cntl_fops);

	dev_id = MKDEV(major, slot); 
	err = cdev_add (cdev, dev_id, 1);
	if (err) {
		return err;
	}

	// Create class device 
	bmi_class = bmi_get_class ();                            
	vh->class_dev = device_create (bmi_class, NULL, MKDEV (major, slot), NULL, "bmi_vh_control_m%i", slot+1);  
								     
	if (IS_ERR(vh->class_dev)) {                                
		printk (KERN_ERR "Unable to create "                  
		       "class_device for bmi_vh_m%i; errno = %ld\n",
		       slot+1, PTR_ERR(vh->class_dev));             
		vh->class_dev = NULL;                               
		cdev_del (&vh->cdev);
		return -ENODEV;
	}                                                            

	// bind driver and bmi_device 
	vh->bdev = bdev;
	

	printk (KERN_INFO "bmi_vh.c: probe slot %d\n", slot);
	vh->iox = i2c_new_device(bdev->slot->adap, &iox_info);
	if (vh->iox == NULL)
	  printk(KERN_ERR "IOX NULL...\n");
	vh->rdac = i2c_new_device(bdev->slot->adap, &rdac_info);
	if (vh->rdac == NULL)
	  printk(KERN_ERR "RDAC NULL...\n");
	vh->adc = i2c_new_device(bdev->slot->adap, &adc_info);
	if (vh->adc == NULL)
	  printk(KERN_ERR "ADC NULL...\n");
	vh->dac = i2c_new_device(bdev->slot->adap, &dac_info);
	if (vh->dac == NULL)
	  printk(KERN_ERR "DAC NULL...\n");
	
	// SPI
	strcpy(vh->vh_spi_info.modalias, "spidev");
	vh->vh_spi_info.max_speed_hz = speed;
	vh->vh_spi_info.bus_num = bdev->slot->spi_bus_num;
	vh->vh_spi_info.chip_select = bdev->slot->spi_cs;
	vh->vh_spi_info.mode = mode;

	vh->spi = spi_new_device(spi_busnum_to_master(vh->vh_spi_info.bus_num), &vh->vh_spi_info) ;
	if (!vh->spi)
	  printk(KERN_WARNING "VH: spi_new_device failed\n");

	bmi_device_set_drvdata (bdev, vh);
	// configure IOX
	if (factory_test) {
		if (WriteByte_IOX(vh->iox, IOX_OUTPUT_REG, 0x55) < 0) {  // all outputs high
		  goto err1;
		}
	
		if (WriteByte_IOX(vh->iox, IOX_CONTROL, 0xAA) < 0) {     // IOX[7,5,3,1]=IN, IOX[6,4,2,0]=OUT
		  goto err1;
		}
	} else {
		if (WriteByte_IOX(vh->iox, IOX_OUTPUT_REG, 0x7F) < 0) {  // USB power on, other outputs high
		  goto err1;
		}
	
		if (WriteByte_IOX(vh->iox, IOX_CONTROL, 0x80) < 0) {     // IOX[7]=IN, IOX[6:0]=OUT
		  goto err1;
		}
	}

	mdelay(100);

	// read RDAC
	if (ReadByte_RDAC(vh->rdac, VH_RD_CMD_RDAC, rdac_data) < 0) {  // read LDO RDAC register
	  goto err1;
	}

	printk (KERN_INFO "bmi_vh.c: probe RDAC = 0x%x\n", *rdac_data);

	if (factory_test) {

		mdelay(100);	// RDAC recovery time

		// set LDO voltage to 3.3V
		*rdac_data = (unsigned char) RDAC_3_3V;
	
		if (WriteByte_RDAC (vh->rdac, VH_RD_CMD_RDAC, *rdac_data, 1) < 0) {
		  goto err1;
		}
	
		mdelay(100);
	
		if (WriteByte_RDAC (vh->rdac, VH_RD_CMD_EE, *rdac_data, 1) < 0) {
		  goto err1;
		}
	
		mdelay(100);

		// read EEPROM
		if (ReadByte_RDAC(vh->rdac, VH_RD_CMD_EE, rdac_data) < 0) {  // read LDO EEPROM
		  goto err1;
		}

		printk (KERN_INFO "bmi_vh.c: probe EEPROM = 0x%x\n", *rdac_data);

		mdelay(100);
	}

	// read ADC
	if (ReadByte_ADC(vh->adc, adc_data) < 0) {  // read initial ADC conversion
	  goto err1;
	}

	printk (KERN_INFO "bmi_vh.c: probe ADC = 0x%x%x%x\n", adc_data[0], adc_data[1], adc_data[2]);

	if (factory_test) {

		// power up DAC
		if (WriteByte_DAC(vh->dac, VH_DAC_W1_EC, VH_DAC_BCH | VH_DAC_PU, 1) < 0) { 
		  goto err1;
		}

		// Write DAC data
		if (WriteByte_DAC(vh->dac, VH_DAC_W1_ALL | 0x0, 0xF0, 1) < 0) {  // write A, B, inputs and update
		  goto err1;
		}
	}

	// read DAC
	if (ReadByte_DAC(vh->dac, VH_DAC_W1_RDA, dac_data) < 0) {  // read initial DAC A value
	  goto err1;
	}

	printk (KERN_INFO "bmi_vh.c: probe DAC = 0x%x%x\n", dac_data[0], dac_data[1]);


	  
	// Initialize GPIOs (turn LED's on)
	bmi_slot_gpio_direction_out (slot, RED_LED, 0);	// Red LED=ON
	bmi_slot_gpio_direction_out (slot, GREEN_LED, 0);	// Red LED=ON
	
	mdelay(200);
	
	// turn LED's off
	bmi_slot_gpio_set_value (slot, RED_LED, 1);
	bmi_slot_gpio_set_value (slot, GREEN_LED, 1);		// Red, Green LED=OFF 		

	if (WriteByte_IOX(vh->iox, IOX_OUTPUT_REG, 0x70) < 0) {  // USB power on, IOX[3:0] low, other outputs high
	  printk (KERN_ERR "bmi_vh.c: probe() - write IOX failed\n");
	  //bmi_device_spi_cleanup(bdev);
	  goto err1;
	}
	

	// request PIM interrupt
	irq = bdev->slot->status_irq;
	sprintf (vh->int_name, "bmi_vh%d", slot);
	if (request_irq(irq, &module_irq_handler, 0, vh->int_name, vh)) {
		printk (KERN_ERR "bmi_vh.c: Can't allocate irq %d or find von Hippel in slot %d\n", 
			irq, slot); 
		//bmi_device_spi_cleanup(bdev);
		goto err1;

		//return -EBUSY;
	}

	// add usb dependency
	increment_usb_dep();

	if (fcc_test) {
		init_timer (&fcc_timer);
		fcc_timer.data = (unsigned long) &bmi_vh[slot];
		fcc_timer.expires = jiffies + (2 * HZ);
		fcc_timer.function = ftimer;
		add_timer (&fcc_timer);			
	}

	return 0;

 err1:	
	vh->class_dev = NULL;                               
	cdev_del (&vh->cdev);
	device_destroy (bmi_class, MKDEV(major, slot));
	bmi_device_set_drvdata (bdev, 0);
	vh->bdev = 0;
	i2c_unregister_device(vh->iox);
	i2c_unregister_device(vh->rdac);
	i2c_unregister_device(vh->adc);
	i2c_unregister_device(vh->dac);
	spi_unregister_device(vh->spi);
	return -ENODEV;
}

// remove PIM
void bmi_vh_remove(struct bmi_device *bdev)
{	
	int slot;
	struct bmi_vh *vh;
	struct class *bmi_class;
	int irq;
	int i;

	printk(KERN_INFO "bmi_vh: Module Removed...\n");
	slot = bdev->slot->slotnum;
	vh = &bmi_vh[slot];

	i2c_unregister_device(vh->iox);
	i2c_unregister_device(vh->rdac);
	i2c_unregister_device(vh->adc);
	i2c_unregister_device(vh->dac);
	spi_unregister_device(vh->spi);

	// remove usb dependency
	decrement_usb_dep();

	if (factory_test) {
		// disable uart transceiver
		bmi_slot_uart_disable (slot);
	}

	if (fcc_test)
		del_timer (&fcc_timer);

	irq = bdev->slot->status_irq;
	free_irq (irq, vh);

	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	vh->class_dev = 0;

	cdev_del (&vh->cdev);

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	vh->bdev = 0;

	return;
}

/*
 *	PM routines
 */

int bmi_vh_resume(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_vh: Resume..\n");
	bmi_slot_uart_enable(bmi_dev->slot->slotnum);
	bmi_slot_spi_enable(bmi_dev->slot->slotnum);
	return 0;
}

int bmi_vh_suspend(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_vh: Suspend..\n");
	bmi_slot_uart_disable(bmi_dev->slot->slotnum);
	bmi_slot_spi_disable(bmi_dev->slot->slotnum);
	return 0;
}

/*
 *	module routines
 */

static void __exit bmi_vh_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver (&bmi_vh_driver);

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}

static int __init bmi_vh_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI VH Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_vh_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}

	if(factory_test)
		printk (KERN_INFO "bmi_vh.c: Factory Test mode enabled\n");

	if(fcc_test)
		printk (KERN_INFO "bmi_vh.c: FCC Test mode enabled\n");

	printk (KERN_INFO "bmi_vh.c: BMI_VH Driver v%s \n", BMIVH_VERSION);

	return 0;
}


module_init(bmi_vh_init);
module_exit(bmi_vh_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Buglabs Inc.");
MODULE_DESCRIPTION("BMI von Hippel device driver");
MODULE_SUPPORTED_DEVICE("bmi_vh_control_mX");

