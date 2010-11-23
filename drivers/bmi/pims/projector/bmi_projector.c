/*
 * 	bmi_projector.c
 *
 * 	BMI PROJECTOR device driver
 *
 *              Derived from: bmi_lcd.c
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

#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/system.h>
#include <mach/mxc_i2c.h>
#include <mach/mx31bug_cpld.h>
#include <linux/bmi.h>
#include <linux/bmi/bmi-control.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_projector.h>
#include <mach/ipu.h>

#include "ch7024.h"

#define DEBUG_PROJECTOR
#undef DEBUG_PROJECTOR

#define BMIPROJECTOR_VERSION	"1.0"		// driver version
#define BMI_SLOT_NUM		(4)		// number of BMI slots
#define MAX_STRG		(40)		// Max string buffer size
#define	VSYNC_DISABLE		0x0
#define	VSYNC_ENABLE		0x1
#define PROJ_DEF_MODE		0x09		// Projector default mode of operation

	// projector
struct projector_interface {
	char			projector_type[MAX_STRG];	// text description of PROJECTOR type
	u8			suspended;		// power management state
	u8			rotation;		// screen rotation
	u8			disp;			// display number (DISP0 or DISP1)
	u8			addr_mode;		// display addressing mode
	u8			vsync_mode;		// VSYNC signal enable (VSYNC_ENABLE | VSYNC_DISABLE)
	u8			bus_if_type;		// bus type (XY | FullWoBE | FullWithBE)
	ipu_adc_sig_cfg_t	adc_sig;		// IPU ADC set-up parameters
	ipu_di_signal_cfg_t	di_sig;			// IPU DI set-up parameters
};

static struct projector_interface projector_interface = {
	.projector_type = "MXCFB_PROJECTOR",
	.suspended = 0,
	.rotation = IPU_ROTATE_NONE,
	.disp = DISP0,
	.vsync_mode = VSYNC_DISABLE,
	.bus_if_type = XY,
	.adc_sig = { 0, 0, 0, 0, 0, 0, 0, 0, IPU_ADC_BURST_WCS, IPU_ADC_IFC_MODE_SYS80_TYPE2,
			16, 0, 0, IPU_ADC_SER_NO_RW },
	.di_sig = { 0,0,0,0,0,0,0,0 },		//pjg - reserved for multiple Projector driver
};

extern void projector_config(int disp);
extern int projector_disp_on(int disp);
extern int projector_disp_regset(int disp, unsigned short reg, unsigned short val);

struct bmi_projector;
static int disp_mode_reg = 0x09;  // Bit 3 of GPIO control register should always be maintained high
//static int proj_mode = 0x1;  // Bit 3 of GPIO control register should always be maintained high

struct bmi_projector_ops {
	void *(*config) (int disp);				// Projector configuration/initialization
	void *(*reset) (int slot);				// Projector reset
	int *(*suspend) (struct bmi_projector *bprojector);	// power management
	int *(*resume) (struct bmi_projector *bprojector);	// power management
	int *(*disp_on) (int disp);				// display on
	int *(*disp_off) (int disp);				// display off
	int (*activate) (struct bmi_projector *projector, int slot);	// enable Projector
	int (*deactivate) (struct bmi_projector *projector, int slot);	// disable Projector
};

struct bmi_projector_ops bmi_projector_ops;

struct bmi_projector {
	struct projector_interface interface;		// pointer to this struct is returned by config()
	struct bmi_projector_ops projector_ops;		// function pointers
};

static struct bmi_projector bmi_projector;

int register_bmi_projector(struct bmi_projector *bprojector, int slot);
int unregister_bmi_projector(struct bmi_projector *bprojector, int slot);

	// private device structure
struct pbmi_projector
{
	int			open_flag;				// force single open
	unsigned int		projector_cnt;				// number of Projector's present
	unsigned int		active;					// indication of Projector's presence
	unsigned int		activated[BMI_SLOT_NUM];		// indication of Projector's presence
	int			proj_mode[BMI_SLOT_NUM];		// Indicate the state of projector on the slot
	//int			disp_mode_reg[BMI_SLOT_NUM];

	struct bmi_projector 		*bprojector[BMI_SLOT_NUM];	// BMI Projector structure - placeholder for multiple display types
	struct bmi_device	*bdev[BMI_SLOT_NUM];			// BMI device per slot
	unsigned int		interrupt[BMI_SLOT_NUM];		// input device interrupt handlers
	char			int_name[MAX_STRG];			// interrupt name
};

static struct pbmi_projector pbmi_projector;	// Projector device sructure

/*
 * 	BMI set up
 */

	// BMI device ID table
static struct bmi_device_id bmi_projector_tbl[] =
{
	{
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT,
		.vendor   = BMI_VENDOR_BUG_LABS,
		.product  = BMI_PRODUCT_PROJECTOR,
		.revision = BMI_ANY,
	},
	{ 0, },					  /* terminate list */
};

MODULE_DEVICE_TABLE(bmi, bmi_projector_tbl);

int	bmi_projector_probe(struct bmi_device *bdev);
void	bmi_projector_remove(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_projector_driver =
{
	.name = "bmi_projector",
	.id_table = bmi_projector_tbl,
	.probe   = bmi_projector_probe,
	.remove  = bmi_projector_remove,
};

void bmi_projector_config(struct bmi_projector *projector, int disp);

	// probe
int bmi_projector_probe(struct bmi_device *bdev)
{
	int slot = bdev->info->slot;
	struct i2c_adapter *adap;
	struct bmi_projector *projector;
	/*int first_time = 1;*/

	printk(KERN_INFO "bmi_projector.c: probe slot %d\n", slot);

	// check for opposite side already active
	switch(slot) {	// opposite side
		case 0:
			if(pbmi_projector.activated[2] == 1) {
				printk(KERN_INFO "bmi_projector.c: probe slot %d not allowed (slot 2 already active)\n", slot);
				bmi_slot_power_off(0);
				pbmi_projector.bdev[0] = bdev;
				return 0;
			}
			break;
		case 1:
			if(pbmi_projector.activated[3] == 1) {
				printk(KERN_INFO "bmi_projector.c: probe slot %d not allowed (slot 3 already active)\n", slot);
				bmi_slot_power_off(1);
				pbmi_projector.bdev[1] = bdev;
				return 0;
			}
			break;
		case 2:
			if(pbmi_projector.activated[0] == 1) {
				printk(KERN_INFO "bmi_projector.c: probe slot %d not allowed (slot 0 already active)\n", slot);
				bmi_slot_power_off(2);
				pbmi_projector.bdev[2] = bdev;
				return 0;
			}
			break;
		case 3:
			if(pbmi_projector.activated[1] == 1) {
				printk(KERN_INFO "bmi_projector.c: probe slot %d not allowed (slot 1 already active)\n", slot);
				bmi_slot_power_off(3);
				pbmi_projector.bdev[3] = bdev;
				return 0;
			}
			break;
	}

	adap = &bdev->adap;

//	bmi_slot_power_on(slot);

	mdelay(500);

	if (!ch7024_detect (adap))
	{
		/* setup for NTSC */
		ch7024_setup (adap, PROJOUT_FMT_NTSC);
#ifdef DEBUG_PROJECTOR
		printk ("\nFound encoder on slot %d \n", slot);
#endif
		ch7024_enable (adap);
	}
	else
	{
		printk ("\nError! Failed to detect encoder chip\n");
		return 0;
	}

	// reset serial link (master)
	if((slot == 0) || (slot == 2)) {
	  bmi_lcd_inactive(0);                  // We are using Same CPLD pins for projector
	} else {
	  bmi_lcd_inactive(1);
	}

	// FPGA PROG_0 - Active low signal
	bmi_set_module_gpio_data(slot, 0, 0);
	bmi_set_module_gpio_dir(slot, 0, BMI_GPIO_OUT);

	/* Reset the FPGA */
	bmi_set_module_gpio_data(slot, 1, 1);
	bmi_set_module_gpio_dir(slot, 0, BMI_GPIO_OUT);
	mdelay(100);
	bmi_set_module_gpio_data(slot, 1, 0);

	// unreset serial link (master)
	if((slot == 0) || (slot == 2)) {
		mdelay(2);
		bmi_lcd_active(0, 0x0, LCD_MODE_I80);
	} else {
		mdelay(2);
		bmi_lcd_active(1, 0x0, LCD_MODE_I80);
	}


	// set up bdev/pbmi_projector pointers
	bmi_device_set_drvdata(bdev, &pbmi_projector);
	pbmi_projector.bdev[slot] = bdev;

	// complete pbmi_projector set-up
	pbmi_projector.projector_cnt++;
	pbmi_projector.active = 1;
	pbmi_projector.activated[slot] = 1;
	pbmi_projector.proj_mode[slot] = 1;

	mdelay(100);

	projector = pbmi_projector.bprojector[slot];
	if((slot == 0) || (slot == 2)) {
		mdelay(2);
		bmi_projector_config(projector, 0);
		mdelay(2);
	} else {
		mdelay(2);
		bmi_projector_config(projector, 1);
		mdelay(2);
	}

	// Turn on Projctor
	disp_mode_reg = PROJ_DEF_MODE;
        projector_disp_regset((slot & 0x1), 0x1, disp_mode_reg);
		// check GPIO status
	printk(KERN_INFO "bmi_projector.c: slot %d gpio = %x\n", slot, bmi_read_gpio_data_reg(slot));
	printk(KERN_INFO "bmi_projector.c: Projector count = %d\n", pbmi_projector.projector_cnt);

	return 0;
}

	// remove
void bmi_projector_remove(struct bmi_device *bdev)
{
	int slot = bdev->info->slot;

	if(pbmi_projector.activated[slot] == 0)
	    return;

	bmi_set_module_gpio_dir (slot, 3, BMI_GPIO_IN);
	bmi_set_module_gpio_dir (slot, 2, BMI_GPIO_IN);
	bmi_set_module_gpio_dir (slot, 1, BMI_GPIO_IN);
	bmi_set_module_gpio_dir (slot, 0, BMI_GPIO_IN);

		//de-attach driver-specific struct from bmi_device structure
	bmi_device_set_drvdata (bdev, NULL);

		// deactivate
	pbmi_projector.activated[slot] = 0;
	pbmi_projector.bdev[slot] = 0;
	pbmi_projector.projector_cnt--;

	if((pbmi_projector.activated[0] == 0) && (pbmi_projector.activated[2] == 0)) {
		bmi_lcd_inactive(0); // disable serializer
	}

	if((pbmi_projector.activated[1] == 0) && (pbmi_projector.activated[3] == 0)) {
		bmi_lcd_inactive(1); // disable serializer
	}

	if((pbmi_projector.activated[0] == 0) && (pbmi_projector.activated[1] == 0) &&
		(pbmi_projector.activated[2] == 0) && (pbmi_projector.activated[3] == 0)) {
		pbmi_projector.active = -1;
	}

		// enable Projector on opposite side
	switch(slot) {
		case 0:
			if(pbmi_projector.bdev[2] != 0)
				bmi_projector_probe(pbmi_projector.bdev[2]);
			break;
		case 1:
			if(pbmi_projector.bdev[3] != 0)
				bmi_projector_probe(pbmi_projector.bdev[3]);
			break;
		case 2:
			if(pbmi_projector.bdev[0] != 0)
				bmi_projector_probe(pbmi_projector.bdev[0]);
			break;
		case 3:
			if(pbmi_projector.bdev[1] != 0)
				bmi_projector_probe(pbmi_projector.bdev[1]);
			break;
	}

	printk(KERN_INFO "bmi_projector.c: projector count = %d\n", pbmi_projector.projector_cnt);

	return;
}

/*
 * control device operations
 */

/*
 * control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *filp)
{
	if(pbmi_projector.open_flag) {
		return - EBUSY;
	}
	pbmi_projector.open_flag = 1;
	filp->private_data = &pbmi_projector;
	return 0;
}

// release
int cntl_release(struct inode *inode, struct file *filp)
{
	pbmi_projector.open_flag = 0;
	return 0;
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
		   unsigned long arg)
{
	struct i2c_adapter *adap;
	int slot = (__user arg) & 0xF;
	int disp = 0;
	int ret = 0;

		// error if no projector active.
	if(pbmi_projector.active == -1)
		return -ENODEV;

		// error if slot invalid
	if((slot < CPLD_M1) || (slot > CPLD_M4))
		return -ENODEV;

	disp = slot & 0x1;  // disp=0 for Slot 0 and 2, disp=1 for 1 and 3
#ifdef DEBUG_PROJECTOR
	printk (KERN_INFO "Slot No is:%d\n", slot);
	printk (KERN_INFO "Disp No is:%d\n", disp);
#endif

		// error if no projector in chosen slot
	if(pbmi_projector.bdev[slot] == 0)
		return -ENODEV;

		// i2c adapter
	adap = &pbmi_projector.bdev[slot]->adap;

	if( (cmd != BMI_PROJECTOR_ON) && (pbmi_projector.proj_mode[slot]/*proj_mode*/ == 0) )
	{
		printk(KERN_ERR "Project is in OFF state !!!\n");
		return -EINVAL;
	}

		// ioctl's
	switch (cmd) {
		case BMI_PROJECTOR_ON:
			{
				printk(KERN_INFO "BMI_PROJECTOR turning on\n");
				disp_mode_reg &= ~(0x3);
				disp_mode_reg |= 0x1;
				ch7024_enable(adap);
                        	projector_disp_regset(disp, 0x1, disp_mode_reg);
				mdelay(100);
                        	projector_disp_regset(disp, 0x0, 0x1);
				//proj_mode = 0x1;
				pbmi_projector.proj_mode[slot] = 0x1;
			}
			break;

		case BMI_PROJECTOR_MODE:
			{
				int mode = ((__user arg) & 0xF0) >> 4;
				printk(KERN_INFO "BMI_PROJECTOR setting mode to 0x%x \n",mode);

				disp_mode_reg &= ~(0x3);
				switch(mode)
				{
					case PROJECTOR_ECONOMY_MODE:
						disp_mode_reg |= 0x2;  //Economy mode
						break;
					case PROJECTOR_BRIGHT_MODE:
						disp_mode_reg |= 0x1;
						break;
					default:
						printk(KERN_ERR "Invalid Mode\n");
						return -EINVAL;
				}

                        	projector_disp_regset(disp, 0x1, disp_mode_reg);
			}
			break;

		case BMI_PROJECTOR_OFF:
			{
				disp_mode_reg &= ~(0x3);
				disp_mode_reg |= 0x3;
				ch7024_disable(adap);
#ifdef DEBUG_PROJECTOR
				printk (KERN_INFO "Mode reg value is:0x%X\n", disp_mode_reg);
#endif
                        	projector_disp_regset(disp, 0x1, disp_mode_reg);
				//proj_mode = 0x0;
				pbmi_projector.proj_mode[slot] = 0x0;
			}
			break;

		case BMI_PROJECTOR_BATTERY:
			{
				printk(KERN_INFO "BMI_PROJECTOR Staring Battery Charger for BUG\n");
				ch7024_disable(adap);
                        	projector_disp_regset(disp, 0x1, 0xF);
				//proj_mode = 0x0;
				pbmi_projector.proj_mode[slot] = 0x0;
			}

			break;
		case BMI_PROJECTOR_HUE:
			{
				int val = ((__user arg) >> 8) & 0xff;
				printk(KERN_INFO "BMI_PROJECTOR setting Hue to 0x%x\n",val);
				ret = ch7024_set_hue(adap, val);
			}
			break;
		case BMI_PROJECTOR_SATURATION:
			{
				int val = ((__user arg) >> 8) & 0xff;
				printk(KERN_INFO "BMI_PROJECTOR setting Saturation to 0x%x\n",val);
				ret = ch7024_set_sat(adap, val);
			}
			break;
		case BMI_PROJECTOR_CONTRAST:
			{
				int val = ((__user arg) >> 8) & 0xff;
				printk(KERN_INFO "BMI_PROJECTOR setting Contrast to 0x%x\n",val);
				ret = ch7024_set_cont(adap, val);
			}
			break;
		case BMI_PROJECTOR_BRIGHTNESS:
			{
				int val = ((__user arg) >> 8) & 0xff;
				printk(KERN_INFO "BMI_PROJECTOR setting Brightness to 0x%x\n",val);
				ret = ch7024_set_bright(adap, val);
			}
			break;
		case BMI_PROJECTOR_SHARPNESS:
			{
				int val = ((__user arg) >> 8) & 0xff;
				printk(KERN_INFO "BMI_PROJECTOR setting Sharpness to 0x%x\n",val);
				ret = ch7024_set_sharp(adap, val);
			}
			break;
		default:
			return -ENOTTY;
	}
	return ret;
}

	// control file operations
struct file_operations cntl_fops = {
	.owner = THIS_MODULE,
	.ioctl = cntl_ioctl,
	.open = cntl_open,
	.release = cntl_release,
};

	// BMI Projector fops
void bmi_projector_config(struct bmi_projector *projector, int disp)
{
	if(pbmi_projector.active == -1) {
		return;
	}

	if((projector) && (projector->projector_ops.config)) {
		projector->projector_ops.config(disp);
	}
}

void bmi_projector_reset(struct bmi_projector *projector, int slot)
{
	if(pbmi_projector.active == -1) {
		return;
	}

	if((projector) && (projector->projector_ops.reset)) {
		projector->projector_ops.reset(slot);
	}
}

int register_bmi_projector(struct bmi_projector *projector, int slot)	//pjg - placeholder for multiple Projector types
{
	if(!projector) {
		return -1;
	}
	if((slot < 0) || (slot > 3)) {
		return -1;
	}
	if(pbmi_projector.bprojector[slot]) {
		return -1;
	}
	else {
		pbmi_projector.bprojector[slot] = projector;
	}

	if(projector->projector_ops.activate) {
		projector->projector_ops.activate(projector, slot);
	}

	return 0;
}

int unregister_bmi_projector(struct bmi_projector *projector, int slot)	//pjg - placeholder for multiple projector types
{
	if (!projector) {
		return -1;
	}
	if ((slot < 0) || (slot > 3)) {
		return -1;
	}
	if (pbmi_projector.bprojector[slot] != projector) {
		return -1;
	}
	else {
		pbmi_projector.bprojector [slot] = 0;
		projector->projector_ops.deactivate(projector, slot);
	}
	return 0;
}

static struct miscdevice cntl_dev = {
	MISC_DYNAMIC_MINOR,
	"bmi_projector",
	&cntl_fops
};

static __init int bmi_projector_init(void)
{
	int rc = 0;

		// No projector is active.
	pbmi_projector.active = -1;
	pbmi_projector.activated[0] = 0;
	pbmi_projector.activated[1] = 0;
	pbmi_projector.activated[2] = 0;
	pbmi_projector.activated[3] = 0;
	pbmi_projector.proj_mode[0] = 0;
	pbmi_projector.proj_mode[1] = 0;
	pbmi_projector.proj_mode[2] = 0;
	pbmi_projector.proj_mode[3] = 0;

		// set up control character device - bmi_projector_control
	rc = misc_register(&cntl_dev);
	if(rc) {
		printk(KERN_ERR "bmi_projector.c: Can't allocate bmi_projector_control device\n");
		return rc;
	}

	pbmi_projector.projector_cnt = 0;

		// hardware specfic set-up
	bmi_projector.interface = projector_interface,
	bmi_projector_ops.config = (void(*)) &projector_config;
	bmi_projector_ops.reset = NULL;	//pjg - placeholder for multiple projector hardware types
	bmi_projector_ops.suspend = NULL;	//pjg - placeholder for multiple projector hardware types
	bmi_projector_ops.resume = NULL;	//pjg - placeholder for multiple projector hardware types
	bmi_projector_ops.disp_on = NULL;	//pjg - placeholder for multiple projector hardware types
	bmi_projector_ops.disp_off = NULL;	//pjg - placeholder for multiple projector hardware types
	bmi_projector_ops.activate = NULL;	//pjg - placeholder for multiple Projector hardware types
	bmi_projector_ops.deactivate = NULL;	//pjg - placeholder for multiple Projector hardware types
	bmi_projector.projector_ops = bmi_projector_ops;
	pbmi_projector.bprojector[0] = &bmi_projector;
	pbmi_projector.bprojector[1] = &bmi_projector;
	pbmi_projector.bprojector[2] = &bmi_projector;
	pbmi_projector.bprojector[3] = &bmi_projector;

	// register with BMI
	rc = bmi_register_driver(&bmi_projector_driver);
	if(rc) {
		printk(KERN_ERR "bmi_projector.c: Can't register bmi_projector_driver\n");

		misc_deregister(&cntl_dev);

		return rc;
	}

	printk("bmi_projector.c: BMI_Projector Driver v%s \n", BMIPROJECTOR_VERSION);

	return 0;
}

static void __exit bmi_projector_clean(void)
{

		// remove control device
	misc_deregister(&cntl_dev);

		// remove bmi driver
	bmi_unregister_driver(&bmi_projector_driver);

        return;
}

module_init(bmi_projector_init);
module_exit(bmi_projector_clean);

// Exported symbols
EXPORT_SYMBOL(register_bmi_projector);
EXPORT_SYMBOL(unregister_bmi_projector);


MODULE_AUTHOR("Suresh Rao");
MODULE_DESCRIPTION("BMI projector device driver");
MODULE_SUPPORTED_DEVICE("bmi_projector_control");
MODULE_LICENSE("GPL");

