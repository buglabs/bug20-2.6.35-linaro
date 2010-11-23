/*
 * 	factory_test.c
 *
 * 	BIG factory test board device driver
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
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/arch/mxc_i2c.h>
#include <linux/bmi.h>
#include <linux/bmi/bmi-control.h>
#include <asm/arch/mx31bug_cpld.h>
#include <asm/arch/gpio.h>
#include <asm/arch/board-bug.h>
#include <asm/arch/mx31bug_gpio.h>
#include <asm/arch/mx31bug_cpld.h>
#include <../arch/arm/mach-mx3/iomux.h>

#define BUG_FACTORY_TEST_VERSION	"1.0"
#define BUF_MAX_SIZE			0x20
#define FT_ERR				-1
#define MX31_GPIO_PORT1			0

static int major;
	// IOX I2C transfer structure
struct iox_i2c_xfer {
    unsigned char addr;
    unsigned char offset;
    unsigned char data;
} iox_i2c_xfer;

	// SPI transfer structure
struct spi_xfer {
    unsigned char addr;
    unsigned char data[2];
} spi_xfer;

enum sig_bus {
    MX31,
    CPLD
};
	// MX31 signals
enum msig {
    CTS0,
    RTS0,
    CTS3,
    RTS3,
    CAM,
    VSYNC1,
    I2S_RXD0,
    I2S_RXD1,
    I2S_RXD2,
    I2S_RXD3
};

	// CPLD signals
enum csig {
    GPIO0,
    GPIO1,
    GPIO2,
    GPIO3,
    PRES0,
    PRES1,
    PRES2,
    PRES3,
    CAM_IF,
    CAM_LOCK_STATUS,
    LCD
};

	// MX31/CPLD signal structure
struct mc_signal {
    enum sig_bus bus;
    unsigned int signal;
    unsigned char funct;
    unsigned int value;
} mc_signal;

	// IOCTL commands for Factory Test Module
#define FT_READ_IOX	_IOR('f', 0x1, struct iox_i2c_xfer *)	// read IOX
#define FT_WRITE_IOX	_IOR('f', 0x2, struct iox_i2c_xfer *)	// write IOX
#define FT_READ_SPI	_IOR('f', 0x3, struct spi_xfer *)	// read SPI
#define FT_WRITE_SPI	_IOR('f', 0x4, struct spi_xfer *)	// write SPI
#define FT_SIGNAL	_IOR('f', 0x5, struct mc_signal *)	// MX31/CPLD signals

	// private device structure
struct bug_ft
{
	struct cdev		cdev;			// character device (4 minor numbers)
	unsigned int		active;			// at lease 1 bdev active
	struct bmi_device	*bdev[4];		// BMI device per slot
	int			open_flag[4];		// force single open on each device
	struct			spi_device *spi[4];	// SPI device
	char			rbuf[BUF_MAX_SIZE];	// SPI read buffer
	char			wbuf[BUF_MAX_SIZE];	// SPI write buffer
};

static struct bug_ft bug_ft;	// global private data

/*
 * 	SPI function
 */

static int spi_rw(struct spi_device *spi, u8 * buf, size_t len)
{
	struct spi_transfer t = {
		.tx_buf = (const void *)buf,
		.rx_buf = buf,
		.len = len,
		.cs_change = 0,
		.delay_usecs = 0,
	};
	struct spi_message m;

	spi_message_init(&m);

	spi_message_add_tail(&t, &m);
	if (spi_sync(spi, &m) != 0 || m.status != 0)
		return FT_ERR;

	return m.actual_length;
}

/*!
 * This function allows writing 1 register on a SPI device.
 *
 * @param        buf         pointer on the buffer
 * @param        count       size of the buffer
 * @return       This function returns the number of written bytes.
 */
static ssize_t spi_write_reg(struct bug_ft *priv, char *buf, int slot)
{
	int res = 0;

	memset(priv->wbuf, 0, BUF_MAX_SIZE);
	priv->wbuf[0] = buf[0];
	priv->wbuf[1] = buf[1];
	priv->wbuf[2] = buf[2];
	priv->wbuf[3] = buf[3];
	if (res > 0) {
		return -EFAULT;
	}
	res = spi_rw(priv->spi[slot], priv->wbuf, 1);

	return res;
}

/*!
 * This function allows reading 1 register from a SPI device.
 *
 * @param        buf         pointer on the buffer
 * @param        off         offset in the buffer
  
 * @return       This function returns the number of read bytes.
 */
static ssize_t spi_read_reg(struct bug_ft *priv, char *buf, int slot)
{
	spi_write_reg(priv, buf, slot);

	memset(priv->rbuf, 0, BUF_MAX_SIZE);
	buf[0] = priv->wbuf[3];
	buf[1] = priv->wbuf[2];
	buf[2] = priv->wbuf[1];
	buf[3] = priv->wbuf[0];

	return 4;
}

/*
 * 	BMI set up
 */

	// BMI device ID table
static struct bmi_device_id bug_ft_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_FACTORY_TEST, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },					  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bug_ft_tbl);

int	bug_ft_probe(struct bmi_device *bdev);
void	bug_ft_remove(struct bmi_device *bdev);

	// BMI driver structure
static struct bmi_driver bug_ft_driver = 
{
	.name = "bug_ft_control", 
	.id_table = bug_ft_tbl, 
	.probe   = bug_ft_probe, 
	.remove  = bug_ft_remove, 
};

/*
 * 	I2C set up
 *
 * 	IOX A on slots 1 & 3
 * 	IOX B on slots 0 & 4
 */

	// I2C IOX register addresses
#define IOX0			(0xE8)	// I2C port A
#define IOX1			(0xEA)	// I2C port A
#define IOX2			(0xEC)	// I2C port A
#define IOX3			(0xEE)	// I2C port A
#define IOX4			(0xE8)	// I2C port B
#define IOX5			(0xEA)	// I2C port B
#define IOX6			(0xEC)	// I2C port B
#define IOX7			(0xEE)	// I2C port B

	// I2C IOX register offset addresses
#define IOX_INPUT0_REG		0x0
#define IOX_INPUT1_REG		0x1
#define IOX_OUTPUT0_REG		0x2
#define IOX_OUTPUT1_REG		0x3
#define IOX_POLARITY0_REG	0x4
#define IOX_POLARITY1_REG	0x5
#define IOX_CONTROL0		0x6
#define IOX_CONTROL1		0x7

	// read byte from I2C IO expander
static int ReadByte_IOX(struct i2c_adapter *adap, unsigned char addr, 
	unsigned char offset, unsigned char *data)
{
		int	ret = 0;
		struct i2c_msg rmsg[2];
		int	num_msgs;

		rmsg[0].addr = addr;
		rmsg[0].flags = 0; /* write */
		rmsg[0].len = 1;
		rmsg[0].buf = &offset;

		rmsg[1].addr = addr;
		rmsg[1].flags = I2C_M_RD; /* read */ 
		rmsg[1].len = 1;
		rmsg[1].buf = data;

		num_msgs = 2;
		ret = i2c_transfer (adap, rmsg, num_msgs);

		if (ret == 2) {
			ret = 0;
		}
		else {
			printk (KERN_ERR "ReadByte_IOX() - i2c_transfer() failed.\n");
			ret = FT_ERR;
		}
		return ret;
}

	// write byte to I2C IO expander
static int WriteByte_IOX(struct i2c_adapter *adap, unsigned char addr,
	unsigned char offset, unsigned char data)
{
		int	ret = 0;
		struct i2c_msg wmsg[2];
		int	num_msgs;
		
		wmsg[0].addr = addr;
		wmsg[0].flags = 0;  /* write */
		wmsg[0].len = 1;
		wmsg[0].buf = &offset;

		wmsg[1].addr = addr;
		wmsg[1].flags = 0;   /* write */ 
		wmsg[1].len = 1;
		wmsg[1].buf = &data;

		num_msgs = 2;

		ret = i2c_transfer (adap, wmsg, num_msgs);

		if (ret == 2) {
			ret = 0;
		}
		else {
			printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed.\n");
			ret = FT_ERR;
		}
		return ret;
}

/*
 * control device operations
 */

// open
int bug_ft_open(struct inode *inode, struct file *filp)
{	
	int slot = MINOR(inode->i_rdev);

	if (bug_ft.open_flag[slot]) {
		return - EBUSY;
	}
	bug_ft.open_flag[slot] = 1;
	filp->private_data = &bug_ft;
	return 0;
}

// release
int bug_ft_release(struct inode *inode, struct file *filp)
{	
	int slot = MINOR(inode->i_rdev);

	bug_ft.open_flag[slot] = 0;
	return 0;
}

/*
 * ioctl and support functions
 */

	// do_mx31_sig
int do_mx31_sig(struct mc_signal * mc_signal)
{
	int read_value;

	switch(mc_signal->signal) {
		case CTS0:	// GPIO2_7
		    	if(mc_signal->funct != 'W') {
				printk("do_mx31_sig(): CTS0 is a Write-Only signal\n");
				return -1;
			}
			iomux_config_mux(MX31_PIN_CTS1, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
			mxc_set_gpio_direction(MX31_PIN_CTS1, 0);
			mxc_set_gpio_dataout(MX31_PIN_CTS1, mc_signal->value);
			break;
		case RTS0: // GPIO2_6
		    	if(mc_signal->funct != 'R') {
				printk("do_mx31_sig(): RTS0 is a Read-Only signal\n");
				return -1;
			}
			mxc_request_iomux(MX31_PIN_RTS1, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			read_value = mxc_get_gpio_datain(MX31_PIN_RTS1);
			mc_signal->value = read_value;
			break;
		case CTS3: // GPIO3_29
		    	if(mc_signal->funct != 'W') {
				printk("do_mx31_sig(): CTS3 is a Write-Only signal\n");
				return -1;
			}
			iomux_config_mux(MX31_PIN_ATA_DIOW, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
			mxc_set_gpio_direction(MX31_PIN_ATA_DIOW, 0);
			mxc_set_gpio_dataout(MX31_PIN_ATA_DIOW, mc_signal->value);
			break;
		case RTS3: // GPIO3_27
		    	if(mc_signal->funct != 'R') {
				printk("do_mx31_sig(): RTS3 is a Read-Only signal\n");
				return -1;
			}
			mxc_request_iomux(MX31_PIN_ATA_CS1, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			read_value = mxc_get_gpio_datain(MX31_PIN_ATA_CS1);
			mc_signal->value = read_value;
			break;
		case CAM: // camera interface -> GPIO
		    	if(mc_signal->funct != 'R') {
				printk("do_mx31_sig(): CAM is a Read-Only signal\n");
				return -1;
			}
			mxc_request_iomux(MX31_PIN_CSI_D8, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			mxc_request_iomux(MX31_PIN_CSI_D9, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			mxc_request_iomux(MX31_PIN_CSI_D10, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			mxc_request_iomux(MX31_PIN_CSI_D11, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			mxc_request_iomux(MX31_PIN_CSI_D12, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			mxc_request_iomux(MX31_PIN_CSI_D13, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			mxc_request_iomux(MX31_PIN_CSI_D14, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			mxc_request_iomux(MX31_PIN_CSI_D15, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			mxc_request_iomux(MX31_PIN_CSI_HSYNC, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			mxc_request_iomux(MX31_PIN_CSI_VSYNC, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);

			mc_signal->value = mxc_get_gpio_datain(MX31_PIN_CSI_HSYNC) << 9;
			mc_signal->value |= mxc_get_gpio_datain(MX31_PIN_CSI_VSYNC) << 8;
			mc_signal->value |= mxc_get_gpio_datain(MX31_PIN_CSI_D15) << 7;
			mc_signal->value |= mxc_get_gpio_datain(MX31_PIN_CSI_D14) << 6;
			mc_signal->value |= mxc_get_gpio_datain(MX31_PIN_CSI_D13) << 5;
			mc_signal->value |= mxc_get_gpio_datain(MX31_PIN_CSI_D12) << 4;
			mc_signal->value |= mxc_get_gpio_datain(MX31_PIN_CSI_D11) << 3;
			mc_signal->value |= mxc_get_gpio_datain(MX31_PIN_CSI_D10) << 2;
			mc_signal->value |= mxc_get_gpio_datain(MX31_PIN_CSI_D9) << 1;
			mc_signal->value |= mxc_get_gpio_datain(MX31_PIN_CSI_D8);

			mxc_request_iomux(MX31_PIN_CSI_D8, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
			mxc_request_iomux(MX31_PIN_CSI_D9, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
			mxc_request_iomux(MX31_PIN_CSI_D10, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
			mxc_request_iomux(MX31_PIN_CSI_D11, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
			mxc_request_iomux(MX31_PIN_CSI_D12, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
			mxc_request_iomux(MX31_PIN_CSI_D13, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
			mxc_request_iomux(MX31_PIN_CSI_D14, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
			mxc_request_iomux(MX31_PIN_CSI_D15, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
			mxc_request_iomux(MX31_PIN_CSI_HSYNC, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
			mxc_request_iomux(MX31_PIN_CSI_VSYNC, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
			break;
		case VSYNC1: // read VSYNC1 state
		    	if(mc_signal->funct != 'R') {
				printk("do_mx31_sig(): VSYNC1 is a Read-Only signal\n");
				return -1;
			}
			mxc_request_iomux(MX31_PIN_SRST0, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
			mc_signal->value = mxc_get_gpio_datain(MX31_PIN_SRST0);
			mxc_request_iomux(MX31_PIN_SRST0, OUTPUTCONFIG_FUNC, INPUTCONFIG_ALT2);
			break;
		case I2S_RXD0: // GPIO1_20
		case I2S_RXD2:
		    	if(mc_signal->funct != 'R') {
				printk("do_mx31_sig(): I2S_RXD[02] are Read-Only signals\n");
				return -1;
			}
			mxc_request_iomux(MX31_PIN_SRXD4, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			read_value = mxc_get_gpio_datain(MX31_PIN_SRXD4);
			mc_signal->value = read_value;
			break;
		case I2S_RXD1: // GPIO1_22
		case I2S_RXD3:
		    	if(mc_signal->funct != 'R') {
				printk("do_mx31_sig(): I2S_RXD[13] are Read-Only signals\n");
				return -1;
			}
			mxc_request_iomux(MX31_PIN_SRXD5, OUTPUTCONFIG_FUNC, INPUTCONFIG_GPIO);
			read_value = mxc_get_gpio_datain(MX31_PIN_SRXD5);
			mc_signal->value = read_value;
			break;
		default:
			printk("do_mx31_sig(): unknown signal\n");
			return -1;
	}
	return 0;
}

	// do_cpld_sig
int do_cpld_sig(struct mc_signal * mc_signal)
{
	int read_value;

	switch(mc_signal->signal) {
		case GPIO0:
		    	if(mc_signal->funct == 'R') {
					// set GPIO to input
				cpld_set_module_gpio_dir(CPLD_M1, 0, CPLD_GPIO_IN);
				cpld_set_module_gpio_dir(CPLD_M1, 1, CPLD_GPIO_IN);
				cpld_set_module_gpio_dir(CPLD_M1, 2, CPLD_GPIO_IN);
				cpld_set_module_gpio_dir(CPLD_M1, 3, CPLD_GPIO_IN);
					// read GPIO
				read_value = cpld_read_gpio_data_reg(CPLD_M1);
					// read interrupt
				read_value |= cpld_interrupt_status(INT_M1_INT) << 4;
				mc_signal->value = read_value;
			} else if(mc_signal->funct == 'H') {
					// write GPIO
				cpld_set_module_gpio_data(CPLD_M1, 0, mc_signal->value & 0x1);
				cpld_set_module_gpio_data(CPLD_M1, 1, (mc_signal->value & 0x2) >> 1);
				cpld_set_module_gpio_data(CPLD_M1, 2, (mc_signal->value & 0x4) >> 2);
				cpld_set_module_gpio_data(CPLD_M1, 3, (mc_signal->value & 0x8) >> 3);
					// set GPIO to output
				cpld_set_module_gpio_dir(CPLD_M1, 0, CPLD_GPIO_OUT);
				cpld_set_module_gpio_dir(CPLD_M1, 1, CPLD_GPIO_OUT);
				cpld_set_module_gpio_dir(CPLD_M1, 2, CPLD_GPIO_OUT);
				cpld_set_module_gpio_dir(CPLD_M1, 3, CPLD_GPIO_OUT);
			} else {
				printk("do_cpld_sig(): Illegal function for CPLD GPIO\n");
				return -1;
			}
			break;
		case GPIO1:
		    	if(mc_signal->funct == 'R') {
					// set GPIO to input
				cpld_set_module_gpio_dir(CPLD_M2, 0, CPLD_GPIO_IN);
				cpld_set_module_gpio_dir(CPLD_M2, 1, CPLD_GPIO_IN);
				cpld_set_module_gpio_dir(CPLD_M2, 2, CPLD_GPIO_IN);
				cpld_set_module_gpio_dir(CPLD_M2, 3, CPLD_GPIO_IN);
					// read GPIO
				read_value = cpld_read_gpio_data_reg(CPLD_M2);
					// read interrupt
				read_value |= cpld_interrupt_status(INT_M2_INT) << 4;
				mc_signal->value = read_value;
			} else if(mc_signal->funct == 'H') {
					// write GPIO
				cpld_set_module_gpio_data(CPLD_M2, 0, mc_signal->value & 0x1);
				cpld_set_module_gpio_data(CPLD_M2, 1, (mc_signal->value & 0x2) >> 1);
				cpld_set_module_gpio_data(CPLD_M2, 2, (mc_signal->value & 0x4) >> 2);
				cpld_set_module_gpio_data(CPLD_M2, 3, (mc_signal->value & 0x8) >> 3);
					// set GPIO to output
				cpld_set_module_gpio_dir(CPLD_M2, 0, CPLD_GPIO_OUT);
				cpld_set_module_gpio_dir(CPLD_M2, 1, CPLD_GPIO_OUT);
				cpld_set_module_gpio_dir(CPLD_M2, 2, CPLD_GPIO_OUT);
				cpld_set_module_gpio_dir(CPLD_M2, 3, CPLD_GPIO_OUT);
			} else {
				printk("do_cpld_sig(): Illegal function for CPLD GPIO\n");
				return -1;
			}
			break;
		case GPIO2:
		    	if(mc_signal->funct == 'R') {
					// set GPIO to input
				cpld_set_module_gpio_dir(CPLD_M3, 0, CPLD_GPIO_IN);
				cpld_set_module_gpio_dir(CPLD_M3, 1, CPLD_GPIO_IN);
				cpld_set_module_gpio_dir(CPLD_M3, 2, CPLD_GPIO_IN);
				cpld_set_module_gpio_dir(CPLD_M3, 3, CPLD_GPIO_IN);
					// read GPIO
				read_value = cpld_read_gpio_data_reg(CPLD_M3);
					// read interrupt
				read_value |= cpld_interrupt_status(INT_M3_INT) << 4;
				mc_signal->value = read_value;
			} else if(mc_signal->funct == 'H') {
					// write GPIO
				cpld_set_module_gpio_data(CPLD_M3, 0, mc_signal->value & 0x1);
				cpld_set_module_gpio_data(CPLD_M3, 1, (mc_signal->value & 0x2) >> 1);
				cpld_set_module_gpio_data(CPLD_M3, 2, (mc_signal->value & 0x4) >> 2);
				cpld_set_module_gpio_data(CPLD_M3, 3, (mc_signal->value & 0x8) >> 3);
					// set GPIO to output
				cpld_set_module_gpio_dir(CPLD_M3, 0, CPLD_GPIO_OUT);
				cpld_set_module_gpio_dir(CPLD_M3, 1, CPLD_GPIO_OUT);
				cpld_set_module_gpio_dir(CPLD_M3, 2, CPLD_GPIO_OUT);
				cpld_set_module_gpio_dir(CPLD_M3, 3, CPLD_GPIO_OUT);
			} else {
				printk("do_cpld_sig(): Illegal function for CPLD GPIO\n");
				return -1;
			}
			break;
		case GPIO3:
		    	if(mc_signal->funct == 'R') {
					// set GPIO to input
				cpld_set_module_gpio_dir(CPLD_M4, 0, CPLD_GPIO_IN);
				cpld_set_module_gpio_dir(CPLD_M4, 1, CPLD_GPIO_IN);
				cpld_set_module_gpio_dir(CPLD_M4, 2, CPLD_GPIO_IN);
				cpld_set_module_gpio_dir(CPLD_M4, 3, CPLD_GPIO_IN);
					// read GPIO
				read_value = cpld_read_gpio_data_reg(CPLD_M4);
					// read interrupt
				read_value |= cpld_interrupt_status(INT_M4_INT) << 4;
				mc_signal->value = read_value;
			} else if(mc_signal->funct == 'H') {
					// write GPIO
				cpld_set_module_gpio_data(CPLD_M4, 0, mc_signal->value & 0x1);
				cpld_set_module_gpio_data(CPLD_M4, 1, (mc_signal->value & 0x2) >> 1);
				cpld_set_module_gpio_data(CPLD_M4, 2, (mc_signal->value & 0x4) >> 2);
				cpld_set_module_gpio_data(CPLD_M4, 3, (mc_signal->value & 0x8) >> 3);
					// set GPIO to output
				cpld_set_module_gpio_dir(CPLD_M4, 0, CPLD_GPIO_OUT);
				cpld_set_module_gpio_dir(CPLD_M4, 1, CPLD_GPIO_OUT);
				cpld_set_module_gpio_dir(CPLD_M4, 2, CPLD_GPIO_OUT);
				cpld_set_module_gpio_dir(CPLD_M4, 3, CPLD_GPIO_OUT);
			} else {
				return -1;
			}
			break;
		case PRES0:
		    	if(mc_signal->funct != 'R') {
				printk("do_cpld_sig(): Illegal function for CPLD PRES signal\n");
				return -1;
			}
			read_value = (cpld_read_module_present_status(CPLD_M1) >> 2) & 0x1;
			mc_signal->value = read_value;
			break;
		case PRES1:
		    	if(mc_signal->funct != 'R') {
				printk("do_cpld_sig(): Illegal function for CPLD PRES signal\n");
				return -1;
			}
			read_value = (cpld_read_module_present_status(CPLD_M2) >> 2) & 0x1;
			mc_signal->value = read_value;
			break;
		case PRES2:
		    	if(mc_signal->funct != 'R') {
				printk("do_cpld_sig(): Illegal function for CPLD PRES signal\n");
				return -1;
			}
			read_value = (cpld_read_module_present_status(CPLD_M3) >> 2) & 0x1;
			mc_signal->value = read_value;
			break;
		case PRES3:
		    	if(mc_signal->funct != 'R') {
				printk("do_cpld_sig(): Illegal function for CPLD PRES signal\n");
				return -1;
			}
			read_value = (cpld_read_module_present_status(CPLD_M4) >> 2) & 0x1;
			mc_signal->value = read_value;
			break;
		case CAM_IF:
		    	if(mc_signal->funct != 'W') {
				printk("do_cpld_sig(): Illegal function for CPLD CAM_IF signal\n");
				return -1;
			}
			cpld_sensor_active(CAM_CLK_RISE);
			break;
		case CAM_LOCK_STATUS:
		    	if(mc_signal->funct != 'R') {
				printk("do_cpld_sig(): Illegal function for CPLD CAM_LOCK_STATUS signal\n");
				return -1;
			}
			read_value = (int) __raw_readw(CPLD_BASE_ADDRESS+CPLD_CAM);
			read_value = cpld_sensor_lock_status();
			mc_signal->value = read_value;
			break;
		case LCD:
		    	if(mc_signal->funct != 'W') {
				printk("do_cpld_sig(): Illegal function for CPLD LCD signal\n");
				return -1;
			}
			cpld_lcd_inactive(0);
			cpld_lcd_inactive(1);
			cpld_lcd_active(0, 0, LCD_MODE_I80);
			cpld_lcd_active(1, 0, LCD_MODE_I80);
			break;
		default:
			printk("do_cpld_sig(): unknown signal\n");
			return -1;
	}
	return 0;
}

	// ioctl
int bug_ft_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, 
		   unsigned long arg)
{	
	int slot = MINOR(inode->i_rdev);
	struct bug_ft *bug_ft = (struct bug_ft *) filp->private_data;
	struct i2c_adapter *adap = &bug_ft->bdev[slot]->adap;
	unsigned char iox_data[1];
	struct iox_i2c_xfer iox_i2c_xfer;
	struct spi_xfer spi_xfer;
	struct mc_signal mc_signal;
	u8 buf[4];
	int ret = 0;

		// error if no ft active.
	if(bug_ft->active == -1)
		return -ENODEV;
		
		// error if no BMI device
	if(bug_ft->bdev[slot] == 0)
		return -ENODEV;
		
		// get I2C transfer structure
	if((cmd == FT_READ_IOX) || (cmd == FT_WRITE_IOX))
		if(copy_from_user(&iox_i2c_xfer, (struct iox_i2c_xfer *) arg, sizeof(struct iox_i2c_xfer))) {
			printk(KERN_INFO "factory_test ioctl(%d): copy_from_user #1 = %d\n", slot, ret);
			return -EFAULT;
		}

		// get SPI transfer structure
	if((cmd == FT_READ_SPI) || (cmd == FT_WRITE_SPI))
		if(copy_from_user(&spi_xfer, (struct spi_xfer *) arg, sizeof(struct spi_xfer))) {
			printk(KERN_INFO "factory_test ioctl(%d): copy_from_user #1 = %d\n", slot, ret);
			return -EFAULT;
		}

		// get signal structure
	if(cmd == FT_SIGNAL)
		if(copy_from_user(&mc_signal, (struct mc_signal *) arg, sizeof(struct mc_signal))) {
			printk(KERN_INFO "factory_test ioctl(%d): copy_from_user #1 = %d\n", slot, ret);
			return -EFAULT;
		}

		// ioctl's
	switch (cmd) {
			// read IOX
		case FT_READ_IOX:
			ret = ReadByte_IOX(adap, iox_i2c_xfer.addr, iox_i2c_xfer.offset, iox_data);
			if(ret == 0) {
				iox_i2c_xfer.data = *iox_data;
				if(copy_to_user((struct iox_i2c_xfer *) arg, &iox_i2c_xfer, sizeof(struct iox_i2c_xfer)))
					ret = -EFAULT;
			}
			break;
			// write IOX
		case FT_WRITE_IOX:
			*iox_data = iox_i2c_xfer.data;
			ret = WriteByte_IOX(adap, iox_i2c_xfer.addr, iox_i2c_xfer.offset, *iox_data);
			if(ret == 0) {
				if(copy_to_user((struct iox_i2c_xfer *) arg, &iox_i2c_xfer, sizeof(struct iox_i2c_xfer))) {
					ret = -EFAULT;
				}
			}
			break;
			// read SPI
		case FT_READ_SPI:
				// READ
			buf[3] = 0xC0 | ((spi_xfer.addr & 0x3F) >> 1);
			buf[2] = 0x00 | ((spi_xfer.addr & 0x1) << 7);
			buf[1] = 0x00;
			buf[0] = 0x00;
			ret = spi_read_reg(bug_ft, buf, slot);
			if(ret == 4) {
				spi_xfer.data[1] = ((buf[2] & 0x7F) << 1) | ((buf[1] & 0x80) >> 7);
				spi_xfer.data[0] = ((buf[1] & 0x7F) << 1) | ((buf[0] & 0x80) >> 7);
				if(copy_to_user((struct spi_xfer *) arg, &spi_xfer, sizeof(struct spi_xfer)))
					ret = -EFAULT;
				else
					ret = 0;
			} else
				ret = FT_ERR;
			break;
			// write SPI
		case FT_WRITE_SPI:
				// EWEN
			buf[3] = 0x98;
			buf[2] = 0x00;
			buf[1] = 0x00;
			buf[0] = 0x00;
			ret = spi_write_reg(bug_ft, buf, slot);
			if(ret != 1) {
			    ret = FT_ERR;
			    break;
			}
				// WRITE
			buf[3] = 0xA0 | ((spi_xfer.addr & 0x3F) >> 1);
			buf[2] = 0x00 | ((spi_xfer.addr & 0x1) << 7) | (spi_xfer.data[1] >> 1);
			buf[1] = ((spi_xfer.data[1] & 0x1) << 7) | (spi_xfer.data[0] >> 1);
			buf[0] = spi_xfer.data[0] << 0x7;
			ret = spi_write_reg(bug_ft, buf, slot);
			if(ret == 1) {
				if(copy_to_user((struct spi_xfer *) arg, &spi_xfer, sizeof(struct spi_xfer)))
					ret = -EFAULT;
				else
					ret = 0;
			} else
				ret = FT_ERR;

			break;
		case FT_SIGNAL:
			if(mc_signal.bus == MX31) {
				ret = do_mx31_sig(&mc_signal);
				if((mc_signal.funct == 'R') && (ret == 0)) {
					if(copy_to_user((struct mc_signal *) arg, &mc_signal, sizeof(struct mc_signal))) {
						ret = -EFAULT;
					} else {
						ret = 0;
					}
				}
			 } else if(mc_signal.bus == CPLD) {
				ret = do_cpld_sig(&mc_signal);
				if((mc_signal.funct == 'R') && (ret == 0)) {
					if(copy_to_user((struct mc_signal *) arg, &mc_signal, sizeof(struct mc_signal))) {
						ret = -EFAULT;
					} else {
						ret = 0;
					}
				}
			} else {
				ret = FT_ERR;
			}
			break;
		default:
			return -ENOTTY;
	}

	return ret;
}

/*
 * 	BMI functions
 */

static const struct file_operations bug_ft_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= bug_ft_ioctl,
	.open		= bug_ft_open,
	.release	= bug_ft_release,
};

int bug_ft_probe(struct bmi_device *bdev)
{	
	int slot = bdev->info->slot;
	struct cdev *cdev_ptr;
	dev_t dev_id;
	int ret;
	unsigned long speed = 1000000;
	unsigned char mode = SPI_MODE_2 | SPI_CS_HIGH;
	unsigned char bits_per_word = 32;


	printk(KERN_INFO "factory_test.c: probe slot %d\n", slot);

	cdev_ptr = &bug_ft.cdev;
	cdev_init(cdev_ptr, &bug_ft_fops);

	dev_id = MKDEV(major, bdev->info->slot); 
	ret = cdev_add(cdev_ptr, dev_id, 1);
	if(ret)
	    return ret;

	switch(slot) {
		case 0:
			cpld_set_module_gpio_dir(CPLD_M1, 0, CPLD_GPIO_IN);
			cpld_set_module_gpio_dir(CPLD_M1, 1, CPLD_GPIO_IN);
			cpld_set_module_gpio_dir(CPLD_M1, 2, CPLD_GPIO_IN);
			cpld_set_module_gpio_dir(CPLD_M1, 3, CPLD_GPIO_IN);
			ret = bmi_device_spi_setup(bdev, speed, mode, bits_per_word);
			if (ret) {
				printk (KERN_ERR "bug_ft_probe() - bmi_device_spi_setup(0) failed.\n");
				return ret;
			}
			bug_ft.spi[0] = &bdev->spi;
			break;
		case 1:
			cpld_set_module_gpio_dir(CPLD_M2, 0, CPLD_GPIO_IN);
			cpld_set_module_gpio_dir(CPLD_M2, 1, CPLD_GPIO_IN);
			cpld_set_module_gpio_dir(CPLD_M2, 2, CPLD_GPIO_IN);
			cpld_set_module_gpio_dir(CPLD_M2, 3, CPLD_GPIO_IN);
			ret = bmi_device_spi_setup(bdev, speed, mode, bits_per_word);
			if (ret) {
				printk (KERN_ERR "bug_ft_probe() - bmi_device_spi_setup(2) failed.\n");
				return ret;
			}
			bug_ft.spi[1] = &bdev->spi;
			break;
		case 2:
			cpld_set_module_gpio_dir(CPLD_M3, 0, CPLD_GPIO_IN);
			cpld_set_module_gpio_dir(CPLD_M3, 1, CPLD_GPIO_IN);
			cpld_set_module_gpio_dir(CPLD_M3, 2, CPLD_GPIO_IN);
			cpld_set_module_gpio_dir(CPLD_M3, 3, CPLD_GPIO_IN);
			ret = bmi_device_spi_setup(bdev, speed, mode, bits_per_word);
			if (ret) {
				printk (KERN_ERR "bug_ft_probe() - bmi_device_spi_setup(3) failed.\n");
				return ret;
			}
			bug_ft.spi[2] = &bdev->spi;
			break;
		case 3:
			cpld_set_module_gpio_dir(CPLD_M4, 0, CPLD_GPIO_IN);
			cpld_set_module_gpio_dir(CPLD_M4, 1, CPLD_GPIO_IN);
			cpld_set_module_gpio_dir(CPLD_M4, 2, CPLD_GPIO_IN);
			cpld_set_module_gpio_dir(CPLD_M4, 3, CPLD_GPIO_IN);
			ret = bmi_device_spi_setup(bdev, speed, mode, bits_per_word);
			if (ret) {
				printk (KERN_ERR "bug_ft_probe() - bmi_device_spi_setup(3) failed.\n");
				return ret;
			}
			bug_ft.spi[3] = &bdev->spi;
			break;
	}
	if(ret) {
		cdev_del(cdev_ptr);
		return ret;
	}

	bmi_device_set_drvdata(bdev, &bug_ft);

	bug_ft.bdev[slot] = bdev;

	bug_ft.active = 1;

	return 0;
}

void bug_ft_remove(struct bmi_device *bdev)
{	
	struct bug_ft *bug_ft = (struct bug_ft*)(bmi_device_get_drvdata (bdev));
	int slot = bdev->info->slot;
	struct cdev *cdev_ptr;
	dev_t dev_id;

	switch(slot) {
		case 0:
			bmi_device_spi_cleanup(bdev);
			break;
		case 1:
			bmi_device_spi_cleanup(bdev);
			break;
		case 2:
			bmi_device_spi_cleanup(bdev);
			break;
		case 3:
			bmi_device_spi_cleanup(bdev);
			break;
	}

	bug_ft->bdev[slot] = 0;

		//de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata(&bdev[slot], 0);

	if((bug_ft->bdev[0]==0) && (bug_ft->bdev[1]==0) && 
		(bug_ft->bdev[2]==0) && (bug_ft->bdev[3]==0)) {
		bug_ft->active = -1;
		cdev_ptr = &bug_ft->cdev;
		dev_id = MKDEV(major, bdev->info->slot); 
		cdev_del(cdev_ptr);
	}

	return;
}

/*
 *	Module functions
 */

static __init int bug_ft_init(void)
{	
	dev_t	dev_id;
	int	retval;

		// No ft is active.
	bug_ft.active = -1;

		// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region(&dev_id, 0, 4, "BUG Factory Test Driver"); 
	if (retval) {
		return -1;
	}

	major = MAJOR(dev_id);

	printk("factory_test.c: BUG FACTORY TEST Driver v%s (major = %d)\n", BUG_FACTORY_TEST_VERSION, major);

	return  bmi_register_driver(&bug_ft_driver); 
}

static void __exit bug_ft_clean(void)
{	
	dev_t dev_id = MKDEV(major, 0);

	unregister_chrdev_region(dev_id, 4);
	bmi_unregister_driver(&bug_ft_driver);

	return;
}

module_init(bug_ft_init);
module_exit(bug_ft_clean);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter Giacomini <p.giacomini@encadis.com>");
MODULE_DESCRIPTION("BUG Factory Test board device driver");
MODULE_SUPPORTED_DEVICE("bug_ft_control");

