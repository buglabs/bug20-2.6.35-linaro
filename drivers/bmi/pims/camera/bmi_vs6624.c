#include <linux/bmi.h>
#include <linux/bmi/bmi-control.h>
#include <linux/bmi/bmi_camera.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include "bug_camera.h"
#include "vs6624_access.h"


// BMI device ID table
static struct bmi_device_id bmi_vs6624_tbl[] = 
{ 
	{ .match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
	  .vendor   = BMI_VENDOR_BUG_LABS, 
	  .product  = BMI_PRODUCT_CAMERA_VS6624, 
	  .revision = BMI_ANY, 
	}, 
	{ 0, },				      /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_vs6624_tbl);

int	bmi_vs6624_probe(struct bmi_device *bdev);
void	bmi_vs6624_remove(struct bmi_device *bdev);
int	bmi_vs6624_suspend(struct bmi_device *bdev);
int	bmi_vs6624_resume(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_vs6624_driver = 
{
	.name = "bmi_vs6624", 
	.id_table = bmi_vs6624_tbl, 
	.probe   = bmi_vs6624_probe, 
	.remove  = bmi_vs6624_remove, 
	};


struct bmi_vs6624 {
	struct bmi_device *bdev;		
	struct bmi_cam    bcam;
	unsigned int	shutter;     // shutter button save state
	unsigned int	zoomin;      // zoomin button save state
	unsigned int	zoomout;     // zoom out button save state
	unsigned int	flash;	     // state of camera FLASH 
	int irq;
	struct input_dev *idev;
	struct work_struct work;

};

	// I2C Slave Address
#define BMI_IOX_I2C_ADDRESS	0x71	// 7-bit address

	// I2C IOX register addresses
#define IOX_INPUT_REG		0x0
#define IOX_OUTPUT_REG		0x1
#define IOX_POLARITY_REG	0x2
#define IOX_CONTROL		0x3


// read byte from I2C IO expander

static int ReadByte_IOX(struct i2c_adapter *adap, unsigned char offset, unsigned char *data)
{
        int    ret = 0;
        struct i2c_msg rmsg[2];
        int    num_msgs;

        /* Read Byte with Pointer */

        rmsg[0].addr = BMI_IOX_I2C_ADDRESS;
        rmsg[0].flags = 0;          /* write */
        rmsg[0].len = 1;
        rmsg[0].buf = &offset;

        rmsg[1].addr = BMI_IOX_I2C_ADDRESS;
        rmsg[1].flags = I2C_M_RD;   /* read */ 
        rmsg[1].len = 1;
        rmsg[1].buf = data;

        num_msgs = 2;
        ret = i2c_transfer (adap, rmsg, num_msgs);

        if (ret == 2) {
		printk (KERN_ERR "ReadByte_IOX() - data = %02X\n", *data);
		
                ret = 0;
        }
        else {
                //Rework: add conditional debug messages here
		printk (KERN_ERR "ReadByte_IOX() - i2c_transfer failed\n");
                ret = -1;
        }
        return ret;
}


// write byte to I2C IO expander
static int WriteByte_IOX(struct i2c_adapter *adap, unsigned char offset, unsigned char data)
{
        int    ret = 0;
        struct i2c_msg wmsg[2];
        int    num_msgs;
        
        /* Write Byte with Pointer */

        wmsg[0].addr = BMI_IOX_I2C_ADDRESS;
        wmsg[0].flags = 0;          /* write */
        wmsg[0].len = 1;
        wmsg[0].buf = &offset;

        wmsg[1].addr = BMI_IOX_I2C_ADDRESS;
        wmsg[1].flags = 0;	   /* write */ 
        wmsg[1].len = 1;
        wmsg[1].buf = &data;

        num_msgs = 2;

        
        ret = i2c_transfer (adap, wmsg, num_msgs);

        if (ret == 2) {
		printk (KERN_ERR "WriteByte_IOX() - data = %02X\n", data);
                ret = 0;
        }
        else {
                //Rework: add conditional debug messages here

                printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed.\n");
                ret = -1;
        }
        return ret;
}


/*
 * Input interrupt handler and support routines
 */


// work handler
void bmi_vs6624_buttons_work(void *arg)
{	
	struct i2c_adapter *adap;
	struct bmi_vs6624 *pim = (struct bmi_vs6624*)arg;

	unsigned char	iox_data;
	unsigned int 	test_value;
	int sync_flag = 0;
	int err;

	printk (KERN_ERR "bmi_vs6624_buttons_work() - enter\n");


	// avoid i2c i/o if camera hardware not present. 

	if (bmi_device_present (pim->bdev) == 0) {
		goto exit;
	}

	adap = bmi_device_get_i2c_adapter (pim->bdev);


	// read IOX data input
	err = ReadByte_IOX (adap, IOX_INPUT_REG, &iox_data);
	if (err) {
		goto exit;
	}

	// zoom in button
	test_value = !((iox_data & 0x2) >> 1);
	if (test_value != pim->zoomin) {
		printk (KERN_ERR "bmi_vs6624buttons_work() - report ZOOMIN\n");
		pim->zoomin = test_value;
		input_report_key (pim->idev, BN_ZOOMIN, test_value);
		sync_flag = 1;
	}


	// zoom out button
	test_value = !((iox_data & 0x4) >> 2);
	if (test_value != pim->zoomout) {
		printk (KERN_ERR "bmi_vs6624_buttons_work() - report ZOOMOUT\n");
		pim->zoomout = test_value;
		input_report_key (pim->idev, BN_ZOOMOUT, test_value);
		sync_flag = 1;
	}
#if 0
	// flash button
	test_value = (iox_data & 0x8) >> 3;
	if (test_value != pim->flash) {
		printk (KERN_ERR "bmi_vs6624_buttons_work() - report FLASH\n");
		pim->flash = test_value;
		input_report_key (pim->idev, BN_FLASH, test_value);
		sync_flag = 1;
	}
#endif

	if ( sync_flag ) {
		printk (KERN_ERR "bmi_vs6624_buttons_work() - input_sync()ing..\n");
		input_sync (pim->idev);
	}
exit:
	printk (KERN_ERR "bmi_vs6624_buttons_work() -exit\n");
	enable_irq (pim->irq);
	return;

}


// interrupt handler
static irqreturn_t module_irq_handler(int irq, void *dummy)
{
	struct bmi_vs6624 *pim = dummy;
	unsigned int test_value;

	int slot;

	printk (KERN_ERR "module_irq_handler() - enter\n");

	disable_irq_nosync(irq);

	slot = bmi_device_get_slot(pim->bdev);


	// shutter button on GPIO

	test_value = !(bmi_read_gpio_data_reg (slot) & 0x1);  

	if (!test_value == pim->shutter) {
		pim->shutter = test_value;
		printk (KERN_ERR "module_irq_handler() - report SHUTTER\n");
		input_report_key (pim->idev, BN_SHUTTER, test_value);
		input_sync (pim->idev);
	}

	// other buttons on I2C IOX
 	schedule_work (&pim->work);
	printk (KERN_ERR "module_irq_handler() - exit\n");
	return IRQ_HANDLED;
}

/*
 * control functions
 */


// configure IOX IO and states
void configure_IOX(struct bmi_vs6624 *cam)
{	
	struct i2c_adapter *adap;
	
	adap = bmi_device_get_i2c_adapter (cam->bdev);

	printk (KERN_ERR "configure_IOX() - enter\n");


	WriteByte_IOX (adap, IOX_OUTPUT_REG, 0x40);	      // CE=0, F_CHG=1,SYNC=0, TORCH=0
	WriteByte_IOX (adap, IOX_CONTROL, 0x0F);	      // IOX[7:4]=OUT, IOX[3:0]=IN
}

// configure GPIO IO and states
void configure_GPIO(struct bmi_vs6624 *cam)
{
	// set states before turning on outputs

	int slot;

	slot = bmi_device_get_slot (cam->bdev);

	bmi_set_module_gpio_data (slot, 3, 1); // Red LED=OFF
	bmi_set_module_gpio_data (slot, 2, 1); // Green LED=OFF
	bmi_set_module_gpio_data (slot, 1, 0); // SER_RST=0

	// configure direction
	bmi_set_module_gpio_dir (slot, 3, BMI_GPIO_OUT);
	bmi_set_module_gpio_dir (slot, 2, BMI_GPIO_OUT);
	bmi_set_module_gpio_dir (slot, 1, BMI_GPIO_OUT);
	bmi_set_module_gpio_dir (slot, 0, BMI_GPIO_IN); // SHUTTER
}

// deconfigure IOX and GPIO
void deconfigure_module(struct bmi_vs6624 *cam)
{
	int slot;
	struct i2c_adapter *adap;

	slot = bmi_device_get_slot (cam->bdev);
	adap = bmi_device_get_i2c_adapter (cam->bdev);


	if ( bmi_device_present (cam->bdev) ) {
		WriteByte_IOX (adap, IOX_CONTROL, 0xFF);
	}
	bmi_set_module_gpio_dir (slot, 3, BMI_GPIO_IN);
	bmi_set_module_gpio_dir (slot, 2, BMI_GPIO_IN);
	bmi_set_module_gpio_dir (slot, 1, BMI_GPIO_IN);
}


// configure serializer on plug-in module
void configure_serializer(struct bmi_vs6624 *cam) 
{	
	int slot = bmi_device_get_slot (cam->bdev);
	bmi_set_module_gpio_data (slot, 1, 1);		 // SER_RST=1                      
}

void deconfigure_serializer(struct bmi_vs6624 *cam)
{	
	int slot = bmi_device_get_slot (cam->bdev);
	bmi_set_module_gpio_data (slot, 1, 0);		// SER_RST=0                      
}

void enable_camera(struct bmi_vs6624 *cam) 
{       
	struct i2c_adapter *adap;
	unsigned char iox_data;

        printk (KERN_ERR "enable_camera() enter\n");

	adap = bmi_device_get_i2c_adapter (cam->bdev);

        // The first i2c read seems to mess everything up.

        ReadByte_IOX (adap, IOX_OUTPUT_REG, &iox_data);
        printk (KERN_ERR "enable_camera() iox_data = %02X\n", iox_data);

        WriteByte_IOX (adap, IOX_OUTPUT_REG, iox_data | 0x80);
        printk (KERN_ERR "enable_camera() exit\n");
}

// disable camera on plug-in module
void disable_camera(struct bmi_vs6624 *cam)
{       
	struct i2c_adapter *adap;
	unsigned char iox_data;

        printk (KERN_ERR "disable_camera() enter\n");

	adap = bmi_device_get_i2c_adapter (cam->bdev);

	if ( bmi_device_present(cam->bdev) ) {

		ReadByte_IOX (adap, IOX_OUTPUT_REG, &iox_data);
        	WriteByte_IOX (adap, IOX_OUTPUT_REG, iox_data & 0x70);
	}

        printk (KERN_ERR "disable_camera() exit\n");
}

// generate sync
void generate_camera_sync(struct i2c_adapter *adap)
{	
	unsigned char iox_data[0];

 	printk(KERN_INFO "generate_camera_sync() - enter\n");
	ReadByte_IOX (adap, IOX_OUTPUT_REG, iox_data);

 	printk(KERN_INFO "generate_camera_sync() - read  = %02X\n", iox_data[0]);
 	printk(KERN_INFO "generate_camera_sync() - write = %02X\n", *iox_data | 0x20);

	WriteByte_IOX (adap, IOX_OUTPUT_REG, *iox_data | 0x20);// SYNC = 1

 	printk(KERN_INFO "generate_camera_sync() - write = %02X\n", *iox_data & 0xD0);

	WriteByte_IOX (adap, IOX_OUTPUT_REG, *iox_data & 0xD0);// SYNC = 0
	udelay(20);				      // 60 MHz * 1024 = ~17 us sync time

 	printk(KERN_INFO "generate_camera_sync() - exit\n");
}

void set_sync(struct i2c_adapter *adap)
{
	unsigned char iox_data[0];

 	printk(KERN_INFO "set_sync() - enter\n");

	ReadByte_IOX (adap, IOX_OUTPUT_REG, iox_data);
	WriteByte_IOX (adap, IOX_OUTPUT_REG, *iox_data | 0x20);// SYNC = 1

 	printk(KERN_INFO "set_sync() - exit\n");
}

void clear_sync(struct i2c_adapter *adap)
{
	unsigned char iox_data[0];

 	printk(KERN_INFO "clear_sync() - enter\n");

	ReadByte_IOX (adap, IOX_OUTPUT_REG, iox_data);
	WriteByte_IOX (adap, IOX_OUTPUT_REG, *iox_data & 0xD0);// SYNC = 0

 	printk(KERN_INFO "clear_sync() - exit\n");
}


// check serializer lock
int check_camera_lock(void) 
{	
	return bmi_sensor_lock_status();
}

void bmi_vs6624_set_color(struct bmi_cam *cam, int bright, int saturation, int red, int green, int blue) 
{

	struct i2c_adapter *adap;
	struct bmi_vs6624 *bmi_vs6624;

	bmi_vs6624 = container_of(cam, struct bmi_vs6624, bcam);
	adap = &bmi_vs6624->bdev->adap;

	vs6624_set_color (adap, bright, saturation, red, green, blue);
	return;

}

void bmi_vs6624_get_color(struct bmi_cam *cam, int *bright, int *saturation, int *red, int *green, int *blue)
{
	struct i2c_adapter *adap;
	struct bmi_vs6624 *bmi_vs6624;

	bmi_vs6624 = container_of(cam, struct bmi_vs6624, bcam);
	adap = &bmi_vs6624->bdev->adap;

	vs6624_get_color (adap, bright, saturation, red, green, blue);
	return;
}




void bmi_vs6624_set_ae_mode (struct bmi_cam *cam, int ae_mode)
{
	printk (KERN_ERR "bmi_vs6624_set_ae_mode() - NOT IMPLEMENTED.\n");
}


void bmi_vs6624_get_ae_mode (struct bmi_cam *cam, int *ae_mode)
{
	printk (KERN_ERR "bmi_vs6624_set_ae_mode() - NOT IMPLEMENTED.\n");
}


sensor_interface * bmi_vs6624_config (struct bmi_cam *cam, int *frame_rate, int high_quality)
{
	//REWORK: Add code here
	struct i2c_adapter *adap;
	struct bmi_vs6624 *bmi_vs6624;
	int i;

	bmi_vs6624 = container_of(cam, struct bmi_vs6624, bcam);
	adap = &bmi_vs6624->bdev->adap;

	printk (KERN_ERR "bmi_vs6624_CONFIG() - enter\n");

        if(check_camera_lock()) {
                printk(KERN_INFO "bmi_vs6624_config(): camera serializer LOCKED\n");
        }
        else {
                printk(KERN_ERR "bmi_vs6624_config(): camera serializer NOT LOCKED\n");
        }

	//Bring up the serial link

	bmi_sensor_active(1);
	configure_serializer (bmi_vs6624);

	adap = &bmi_vs6624->bdev->adap;
        set_sync (adap);

        for (i = 0; i < 10; i++) {

                msleep(10);

                if(check_camera_lock()) {
                        printk(KERN_INFO "bmi_vs6624_config() - camera serializer locked, i = %d\n", i);
                        break;
                }
                else {
                        printk(KERN_ERR "bmi_vs6624_config() -  camera serializer did not lock,i = %d\n", i);
                }

        }
        clear_sync (adap);


        if(check_camera_lock()) {

                printk(KERN_INFO "bmi_vs6624_config(): camera serializer LOCKED\n");
        }
        else {
                printk(KERN_ERR "bmi_vs6624_config(): camera serializer NOT LOCKED\n");
        }


	printk (KERN_ERR "bmi_vs6624_CONFIG() - exit\n");
	return &cam->interface;

}


sensor_interface * bmi_vs6624_reset (struct bmi_cam *cam)
{
	//REWORK: Add code here
	//REWORK: What is a valid soft reset sequence ?
	struct i2c_adapter *adap;
	struct bmi_vs6624 *bmi_vs6624;

	printk (KERN_ERR "bmi_vs6624_RESET() - enter\n");

	bmi_vs6624 = container_of(cam, struct bmi_vs6624, bcam);


	adap = bmi_device_get_i2c_adapter (bmi_vs6624->bdev);

        if(check_camera_lock()) {
                printk(KERN_INFO "bmi_vs6624_reset(): camera serializer LOCKED\n");
        }
        else {
                printk(KERN_ERR "bmi_vs6624_reset(): camera serializer NOT LOCKED\n");
        }
	//disable the serial link

	deconfigure_serializer (bmi_vs6624);
	bmi_sensor_inactive();

        if(check_camera_lock()) {

                printk(KERN_INFO "bmi_vs6624_reset(): camera serializer LOCKED\n");
        }
        else {
                printk(KERN_ERR "bmi_vs6624_reset(): camera serializer NOT LOCKED\n");
        }

	printk (KERN_ERR "bmi_vs6624_RESET() - exit\n");

	return &cam->interface;
}

int bmi_vs6624_activate (struct bmi_cam *cam, struct input_dev *idev)
{
	//REWORK: Add code here
	int rc = 0;
	int i;
	struct i2c_adapter *adap;
	struct bmi_vs6624 *bmi_vs6624;

	printk (KERN_ERR "int bmi_vs6624_activate () - enter\n");

	bmi_vs6624 = container_of(cam, struct bmi_vs6624, bcam);


	//bmi_vs6624 struct fields
	bmi_vs6624->idev = idev;

	bmi_vs6624->shutter  = 0;
	bmi_vs6624->zoomin   = 0; 
	bmi_vs6624->zoomout  = 0;
	bmi_vs6624->flash    = 0;	

	printk (KERN_ERR "bmi_vs6624_activate () - irq = %d\n", bmi_vs6624->irq);

	// install button irq_handler
	if (request_irq(bmi_vs6624->irq, &module_irq_handler, 0, "bmi_cam_button", bmi_vs6624)) { 
		printk (KERN_ERR
			"bmi_vs6624_activate() - request_irq (irq = %d) failed.\n",
			bmi_vs6624->irq);  

			rc = -EBUSY;
			goto exit;
		}

	//Activate serial link
	bmi_sensor_active(1);		      // rising edge clock
	configure_serializer (bmi_vs6624);

	adap = &bmi_vs6624->bdev->adap;
        set_sync (adap);


        for (i = 0; i < 10; i++) {

                msleep(10);

                if(check_camera_lock()) {
                        printk(KERN_INFO "bmi_vs6624_activate() - camera serializer locked, i = %d\n", i);
                        break;
                }
                else {
                        printk(KERN_ERR "bmi_vs6624_activate() -  camera serializer did not lock,i = %d\n", i);
                }

        }
        clear_sync (adap);

        if(check_camera_lock()) {
                printk(KERN_INFO "bmi_vs6624_activate(): camera serializer LOCKED\n");
        }
        else {
                printk(KERN_ERR "bmi_vs6624_activate(): camera serializer NOT LOCKED\n");
        }


exit:
	printk (KERN_ERR "int bmi_vs6624_activate () - exit\n");
	return rc;
}

int bmi_vs6624_deactivate (struct bmi_cam *cam)
{
	//REWORK: Add code here

	struct bmi_vs6624 *bmi_vs6624;
	bmi_vs6624 = container_of(cam, struct bmi_vs6624, bcam);


	//De-activate serial link     
	deconfigure_serializer (bmi_vs6624);
	bmi_sensor_inactive();


	//uninstall button irq_handler
	free_irq(bmi_vs6624->irq, bmi_vs6624);


	return 0;
}


void bmi_vs6624_link_enable (struct bmi_cam *cam)
{
	//REWORK: Add code here

	int i;
	struct i2c_adapter *adap;
	struct bmi_vs6624 *bmi_vs6624;
	bmi_vs6624 = container_of(cam, struct bmi_vs6624, bcam);

	//Activate serial link
	bmi_sensor_active(1);		      // rising edge clock
	configure_serializer (bmi_vs6624);

	adap =	bmi_device_get_i2c_adapter (bmi_vs6624->bdev);

        set_sync (adap);


	//REWORK: Speed this up. (shorten delay)

        for (i = 0; i < 10; i++) {

                msleep(10);

                if(check_camera_lock()) {
                        printk(KERN_INFO "bmi_vs6624_activate() - camera serializer locked, i = %d\n", i);
                        break;
                }
                else {
                        printk(KERN_ERR "bmi_vs6624_activate() -  camera serializer did not lock,i = %d\n", i);
                }

        }
        clear_sync (adap);

        if(check_camera_lock()) {
                printk(KERN_INFO "bmi_vs6624_activate(): camera serializer LOCKED\n");
        }
        else {
                printk(KERN_ERR "bmi_vs6624_activate(): camera serializer NOT LOCKED\n");
        }

	return;
}


void bmi_vs6624_link_disable (struct bmi_cam *cam)
{
	struct bmi_vs6624 *bmi_vs6624;
	bmi_vs6624 = container_of(cam, struct bmi_vs6624, bcam);


	//De-activate serial link     
	deconfigure_serializer (bmi_vs6624);
	bmi_sensor_inactive();

	return;
}



int bmi_vs6624_red_led_off (struct bmi_cam *cam)
{
	struct bmi_vs6624 *bmi_vs6624;
	struct bmi_device *bdev;
	int slot;

	bmi_vs6624 = container_of(cam, struct bmi_vs6624, bcam);
	bdev = bmi_vs6624->bdev;

	slot = bmi_device_get_slot (bdev);

	bmi_set_module_gpio_data (slot, 3, 1);	// Red LED=OFF
	return 0;
}


int bmi_vs6624_red_led_on (struct bmi_cam *cam)
{
	struct bmi_vs6624 *bmi_vs6624;
	struct bmi_device *bdev;
	int slot;

	bmi_vs6624 = container_of(cam, struct bmi_vs6624, bcam);
	bdev = bmi_vs6624->bdev;

	slot = bmi_device_get_slot (bdev);

	bmi_set_module_gpio_data (slot, 3, 0);	// Red LED=ON
	return 0;
}


int bmi_vs6624_green_led_off (struct bmi_cam *cam)
{
	struct bmi_vs6624 *bmi_vs6624;
	struct bmi_device *bdev;
	int slot;

	bmi_vs6624 = container_of(cam, struct bmi_vs6624, bcam);
	bdev = bmi_vs6624->bdev;

	slot = bmi_device_get_slot (bdev);

	bmi_set_module_gpio_data (slot, 2, 1);	// Green LED=OFF
	return 0;
}


int bmi_vs6624_green_led_on (struct bmi_cam *cam)
{
	struct bmi_vs6624 *bmi_vs6624;
	struct bmi_device *bdev;
	int slot;

	bmi_vs6624 = container_of(cam, struct bmi_vs6624, bcam);
	bdev = bmi_vs6624->bdev;

	slot = bmi_device_get_slot (bdev);

	bmi_set_module_gpio_data (slot, 2, 0);	// Green LED=ON
	return 0;
}


int bmi_vs6624_probe(struct bmi_device *bdev)
{	

	//REWORK: Add code here

	int slot = bmi_device_get_slot(bdev);


	// allocate a driver-specific <this> structure

	struct bmi_vs6624 *bmi_vs6624 = kzalloc(sizeof(struct bmi_vs6624), GFP_KERNEL);
	if (!bmi_vs6624) {
	     return -1;
	}

	// attach <this> bmi_device structure (so we can find it later).
	bmi_device_set_drvdata(bdev, bmi_vs6624);


	// initialize bmi_vs6624 struct
	bmi_vs6624->bdev = bdev;

	// sensor interface struct fields
# if 1
	bmi_vs6624->bcam.interface.clk_mode   = 0;		  // gated          
	bmi_vs6624->bcam.interface.ext_vsync  = 1;		  // external vsync 
#else
	bmi_vs6624->bcam.interface.clk_mode   = 2;		  // progressive          
	bmi_vs6624->bcam.interface.ext_vsync  = 0;		  // internal vsync 
# endif
	bmi_vs6624->bcam.interface.Vsync_pol  = 0;		  // non-inverted   
	bmi_vs6624->bcam.interface.Hsync_pol  = 0; 		  // non-inverted   
	bmi_vs6624->bcam.interface.pixclk_pol = 0; 		  // non-inverted   
	bmi_vs6624->bcam.interface.data_pol   = 0; 		  // non-inverted   MTW=1
	bmi_vs6624->bcam.interface.data_width = 1; 		  // 8-bits         
	bmi_vs6624->bcam.interface.width      = 1280-1;		  // 1280 - SXGA    
	bmi_vs6624->bcam.interface.height     = 1024-1;		  // 1024 - SXGA    

	bmi_vs6624->bcam.interface.pixel_fmt  = IPU_PIX_FMT_UYVY; // YUV422         
	bmi_vs6624->bcam.interface.mclk	      = 12000000;	  // frequency/src  

	//bmi_camera_sensor struct fields

	bmi_vs6624->bcam.sensor.set_color   = bmi_vs6624_set_color;  
	bmi_vs6624->bcam.sensor.get_color   = bmi_vs6624_get_color;  
	bmi_vs6624->bcam.sensor.set_ae_mode = bmi_vs6624_set_ae_mode;  
	bmi_vs6624->bcam.sensor.get_ae_mode = bmi_vs6624_get_ae_mode;  
	bmi_vs6624->bcam.sensor.config	    = bmi_vs6624_config;  
	bmi_vs6624->bcam.sensor.reset	    = bmi_vs6624_reset;  

	//bmi_camera_link struct fields

	bmi_vs6624->bcam.link.enable	    = bmi_vs6624_link_enable;  
	bmi_vs6624->bcam.link.disable	    = bmi_vs6624_link_disable;  

	//bmi_camera_cntl struct fields

	bmi_vs6624->bcam.cntl.flash_led_off    = 0;
	bmi_vs6624->bcam.cntl.flash_led_off    = 0;
	bmi_vs6624->bcam.cntl.flash_high_beam  = 0;
	bmi_vs6624->bcam.cntl.flash_low_beam   = 0;
	bmi_vs6624->bcam.cntl.red_led_off      = bmi_vs6624_red_led_off;
	bmi_vs6624->bcam.cntl.red_led_on       = bmi_vs6624_red_led_on;
	bmi_vs6624->bcam.cntl.green_led_off    = bmi_vs6624_green_led_off;
	bmi_vs6624->bcam.cntl.green_led_on     = bmi_vs6624_green_led_on;


	//bmi_cam struct fields

	bmi_vs6624->bcam.activate   = bmi_vs6624_activate  ;
	bmi_vs6624->bcam.deactivate = bmi_vs6624_deactivate; 

	//bmi_vs6624 struct fields
	bmi_vs6624->shutter  = 0;
	bmi_vs6624->zoomin   = 0; 
	bmi_vs6624->zoomout  = 0;
	bmi_vs6624->flash    = 0;	

	bmi_vs6624->irq = bmi_device_get_status_irq (bdev);	

	//initialize struct work_struct
	INIT_WORK (&bmi_vs6624->work, bmi_vs6624_buttons_work, bmi_vs6624); 

	//Power stablization delay
  	mdelay(500);
	
	//Do one-time hw initialization (e.g. patch)

	// configure IOX
	configure_IOX (bmi_vs6624);

	// configure GPIO
	configure_GPIO (bmi_vs6624);

	// chip enable camera
	enable_camera (bmi_vs6624);

	vs6624_patch (&bmi_vs6624->bdev->adap);

	//register with bug_camera

	//REWORK: check return code
  	register_bug_camera (&bmi_vs6624->bcam, slot);

	return 0;
}

void bmi_vs6624_remove(struct bmi_device *bdev)
{	
	//REWORK: Add code here


	//get our <this> pointer    
	struct bmi_vs6624 *bmi_vs6624 = (struct bmi_vs6624*)(bmi_device_get_drvdata (bdev));
	int slot = bmi_device_get_slot (bdev);


	//unregister with bug_camera
  	unregister_bug_camera ( &bmi_vs6624->bcam, slot);

	//REWORK: Avoid I2c access if camera module is not present.

	disable_camera (bmi_vs6624);
	deconfigure_module (bmi_vs6624);

	
	flush_scheduled_work();

	//de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);

	//free driver-specific structure
	kfree (bmi_vs6624);
	return;
}


static __init int bmi_vs6624_init(void)
{	

//	Register with BMI bus.
	return  bmi_register_driver (&bmi_vs6624_driver); 

}

static void __exit bmi_vs6624_cleanup(void)
{	

//	UnRegister with BMI bus.
	bmi_unregister_driver (&bmi_vs6624_driver);
	return;
}


module_init(bmi_vs6624_init);
module_exit(bmi_vs6624_cleanup);

MODULE_AUTHOR("EnCADIS Design, Inc.");
MODULE_DESCRIPTION("VS6624 Camera Driver");
MODULE_LICENSE("GPL");

