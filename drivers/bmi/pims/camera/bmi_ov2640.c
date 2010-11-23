#include <linux/bmi.h>
#include <linux/bmi/bmi-control.h>
#include <linux/bmi/bmi_camera.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include "bug_camera.h"
#include "ov2640.h"

#define BMI_OV2640_VERSION  "1.0"

// BMI device ID table
static struct bmi_device_id bmi_ov2640_tbl[] = 
{ 
	{ .match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
	  .vendor   = BMI_VENDOR_BUG_LABS, 
	  .product  = BMI_PRODUCT_CAMERA_OV2640, 
	  .revision = BMI_ANY, 
	}, 
	{ 0, },				      /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_ov2640_tbl);

int	bmi_ov2640_probe(struct bmi_device *bdev);
void 	bmi_ov2640_remove(struct bmi_device *bdev);
int	bmi_ov2640_suspend(struct bmi_device *bdev);
int	bmi_ov2640_resume(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_ov2640_driver = 
{
	.name = "bmi_ov2640", 
	.id_table = bmi_ov2640_tbl, 
	.probe   = bmi_ov2640_probe, 
	.remove  = bmi_ov2640_remove, 
	};


struct bmi_ov2640 {
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

#define work_to_bmi_ov2640(w)  container_of(w, struct bmi_ov2640, work)

/* IOX Bits Definitions

	IOX Bit 7 CAM_RST*	        Output: 0 = Reset, 1 = Normal Operation
	IOX Bit 6 GRNLED	        Output: 0 = Green LED on, 1 = Green LED off
	IOX Bit 5 SER_SYNC		Output: 0 = Normal Operation, 1 = send SYNC
	IOX Bit 4 FLASH_TORCH*		Output: 0 = low beam, 1 = high beam
	IOX Bit 3 STROBE_R		I/O, not used.
	IOX Bit 2 ZOOMB			Input: Zoom OUT Button: 0 = depressed, 1 = released
	IOX Bit 1 ZOOMA			Input: Zoom IN  Button  0 = depressed, 1 = released 
	IOX Bit 0 GPIO0_SHUTTER*	Input: Shutter Button   0 = depressed, 1 = released 

   GPIO  Bits Definitions

	GPIO Bit 3 REDLED		Output: 0 = Red LED on, 1 = Red LED off
	GPIO Bit 2 FLASHON		Output: 0 = Flash LED off, 1 = Flash LED on
	GPIO Bit 1 SER_RST*		Output: 0 = Serializer Reset, 1 = Normal Operation
	GPIO Bit 0 GPIO0_SHUTTER* 	Input:  Shutter Button: 0 = depressed, 1 = released 


*/



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
                ret = 0;
        }
        else {
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
                ret = 0;
        }
        else {
                printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed.\n");
                ret = -1;
        }
        return ret;
}


/*
 * Input interrupt handler and support routines
 */


// work handler
void bmi_ov2640_buttons_work(struct work_struct * work)
{	
	struct i2c_adapter *adap;
	struct bmi_ov2640 *pim = work_to_bmi_ov2640(work);

	unsigned char	iox_data;
	unsigned int 	test_value;
	int sync_flag = 0;
	int err;


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
		pim->zoomin = test_value;
		input_report_key (pim->idev, BN_ZOOMIN, test_value);
		sync_flag = 1;
	}


	// zoom out button
	test_value = !((iox_data & 0x4) >> 2);
	if (test_value != pim->zoomout) {
		pim->zoomout = test_value;
		input_report_key (pim->idev, BN_ZOOMOUT, test_value);
		sync_flag = 1;
	}

	if ( sync_flag ) {
		input_sync (pim->idev);
	}
exit:
	enable_irq (pim->irq);
	return;

}


// interrupt handler
static irqreturn_t module_irq_handler(int irq, void *dummy)
{
	struct bmi_ov2640 *pim = dummy;
	unsigned int test_value;

	int slot;

	disable_irq_nosync(irq);

	slot = bmi_device_get_slot(pim->bdev);


	// shutter button on GPIO

	test_value = !(bmi_read_gpio_data_reg (slot) & 0x1);  

	if (!test_value == pim->shutter) {
		pim->shutter = test_value;
		input_report_key (pim->idev, BN_SHUTTER, test_value);
		input_sync (pim->idev);
	}

	// other buttons on I2C IOX
 	schedule_work (&pim->work);
	return IRQ_HANDLED;
}

/*
 * control functions
 */


// configure IOX IO and states
void configure_IOX(struct bmi_ov2640 *cam)
{	
	struct i2c_adapter *adap;
	
	adap = bmi_device_get_i2c_adapter (cam->bdev);

	WriteByte_IOX (adap, IOX_OUTPUT_REG, 0xC0);	      // CAMRST* = 1, GRNLED = Off
	WriteByte_IOX (adap, IOX_CONTROL,    0x0F);	      // IOX[7:4]=OUT, IOX[3:0]=IN
	return;

}

// configure GPIO IO and states
void configure_GPIO(struct bmi_ov2640 *cam)
{
	// set states before turning on outputs

	int slot;


	slot = bmi_device_get_slot (cam->bdev);

	bmi_set_module_gpio_data (slot, 3, 1); // Red LED=OFF
	bmi_set_module_gpio_data (slot, 2, 0); // Flash LED=OFF
	bmi_set_module_gpio_data (slot, 1, 0); // SER_RST=0

	// configure direction
	bmi_set_module_gpio_dir (slot, 3, BMI_GPIO_OUT);
	bmi_set_module_gpio_dir (slot, 2, BMI_GPIO_OUT);
	bmi_set_module_gpio_dir (slot, 1, BMI_GPIO_OUT);
	bmi_set_module_gpio_dir (slot, 0, BMI_GPIO_IN); // SHUTTER

	// This is needed.
	bmi_set_module_gpio_data (slot, 2, 0);	// Flash LED off.
	return;

}

// deconfigure IOX and GPIO
void deconfigure_module(struct bmi_ov2640 *cam)
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
void configure_serializer(struct bmi_ov2640 *cam) 
{	
	int slot = bmi_device_get_slot (cam->bdev);
	bmi_set_module_gpio_data (slot, 1, 1);		 // SER_RST=1                      
}

void deconfigure_serializer(struct bmi_ov2640 *cam)
{	
	int slot = bmi_device_get_slot (cam->bdev);
	bmi_set_module_gpio_data (slot, 1, 0);		// SER_RST=0                      
}

void enable_camera(struct bmi_ov2640 *cam) 
{       
	struct i2c_adapter *adap;
        unsigned char iox_data;

	adap = bmi_device_get_i2c_adapter (cam->bdev);

        ReadByte_IOX (adap, IOX_OUTPUT_REG, &iox_data);

	iox_data |=  (0x80);	  //Set CAM_RST* to 1.

        WriteByte_IOX (adap, IOX_OUTPUT_REG, iox_data);
	return;
}

// disable camera on plug-in module
void disable_camera(struct bmi_ov2640 *cam)
{       
	struct i2c_adapter *adap;
        unsigned char iox_data;

	if (cam == NULL)
	  {
	    printk(KERN_INFO "bmi_camera_ov2640: disable_camera: NULL Pointer on cam\n");
	    return;
	  }
	if (cam->bdev == NULL)
	  {
	    printk(KERN_INFO "bmi_camera_ov2640: disable_camera: NULL Pointer on cam->bdev\n");
	    return;
	  }
	adap = bmi_device_get_i2c_adapter (cam->bdev);

	if ( bmi_device_present(cam->bdev) ) {

	        ReadByte_IOX (adap, IOX_OUTPUT_REG, &iox_data);

		iox_data &= ~(0x80);	 //Set CAM_RST* to 0;

        	WriteByte_IOX (adap, IOX_OUTPUT_REG, iox_data);
	}
	return;
}

// generate sync
void generate_camera_sync(struct i2c_adapter *adap)
{	
	unsigned char iox_data[0];

	ReadByte_IOX (adap, IOX_OUTPUT_REG, iox_data);

	WriteByte_IOX (adap, IOX_OUTPUT_REG, *iox_data | 0x20);// SYNC = 1

	WriteByte_IOX (adap, IOX_OUTPUT_REG, *iox_data & 0xD0);// SYNC = 0
	udelay(20);				      // 60 MHz * 1024 = ~17 us sync time
	return;
}

void set_sync(struct i2c_adapter *adap)
{
	unsigned char iox_data[0];

	ReadByte_IOX (adap, IOX_OUTPUT_REG, iox_data);
	WriteByte_IOX (adap, IOX_OUTPUT_REG, *iox_data | 0x20);// SYNC = 1
	return;
}

void clear_sync(struct i2c_adapter *adap)
{
	unsigned char iox_data[0];

	ReadByte_IOX (adap, IOX_OUTPUT_REG, iox_data);
	WriteByte_IOX (adap, IOX_OUTPUT_REG, *iox_data & 0xD0);// SYNC = 0
	return;
}


// check serializer lock
int check_camera_lock(void) 
{	
	return bmi_sensor_lock_status();
}

void bmi_ov2640_set_color(struct bmi_cam *cam, int bright, int saturation, int red, int green, int blue) 
{

	struct i2c_adapter *adap;
	struct bmi_ov2640 *bmi_ov2640;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);
	adap = &bmi_ov2640->bdev->adap;

//	ov2640_set_color (adap, bright, saturation, red, green, blue);
	printk (KERN_ERR "bmi_ov2640_set_color() - NOT IMPLEMENTED.\n");
	return;

}

void bmi_ov2640_get_color(struct bmi_cam *cam, int *bright, int *saturation, int *red, int *green, int *blue)
{
	struct i2c_adapter *adap;
	struct bmi_ov2640 *bmi_ov2640;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);
	adap = &bmi_ov2640->bdev->adap;

//	ov2640_get_color (adap, bright, saturation, red, green, blue);
	printk (KERN_ERR "bmi_ov2640_get_color() - NOT IMPLEMENTED.\n");
	return;
}




void bmi_ov2640_set_ae_mode (struct bmi_cam *cam, int ae_mode)
{
	printk (KERN_ERR "bmi_ov2640_set_ae_mode() - NOT IMPLEMENTED.\n");
}


void bmi_ov2640_get_ae_mode (struct bmi_cam *cam, int *ae_mode)
{
	printk (KERN_ERR "bmi_ov2640_set_ae_mode() - NOT IMPLEMENTED.\n");
}


sensor_interface * bmi_ov2640_config (struct bmi_cam *cam, int *frame_rate, int high_quality)
{

	struct i2c_adapter *adap;
	struct bmi_ov2640 *bmi_ov2640;
	int i;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);
	adap = &bmi_ov2640->bdev->adap;

	//Bring up the serial link

	bmi_sensor_active(1);
	configure_serializer (bmi_ov2640);

	adap = &bmi_ov2640->bdev->adap;
        set_sync (adap);

        for (i = 0; i < 10; i++) {

                msleep(10);

                if(check_camera_lock()) {
                        break;
                }
                else {
                        printk(KERN_ERR "bmi_ov2640_config() -  camera serializer did not lock,i = %d\n", i);
                }

        }
	clear_sync(adap);


        if(!check_camera_lock()) {
                printk(KERN_ERR "bmi_ov2640_config(): camera serializer NOT LOCKED\n");
        }

	return &cam->interface;

}


sensor_interface * bmi_ov2640_reset (struct bmi_cam *cam)
{
	struct i2c_adapter *adap;
	struct bmi_ov2640 *bmi_ov2640;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);


	adap = bmi_device_get_i2c_adapter (bmi_ov2640->bdev);

	//disable the serial link

	deconfigure_serializer (bmi_ov2640);
	bmi_sensor_inactive();

	return &cam->interface;
}

int bmi_ov2640_activate (struct bmi_cam *cam, struct input_dev *idev)
{
	int rc = 0;
	int i;
	struct i2c_adapter *adap;
	struct bmi_ov2640 *bmi_ov2640;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);


	//bmi_ov2640 struct fields
	bmi_ov2640->idev = idev;

	bmi_ov2640->shutter  = 0;
	bmi_ov2640->zoomin   = 0; 
	bmi_ov2640->zoomout  = 0;
	bmi_ov2640->flash    = 0;	


	// install button irq_handler
	if (request_irq(bmi_ov2640->irq, &module_irq_handler, 0, "bmi_cam_button", bmi_ov2640)) { 
		printk (KERN_ERR
			"bmi_ov2640_activate() - request_irq (irq = %d) failed.\n",
			bmi_ov2640->irq);  

			rc = -EBUSY;
			goto exit;
		}

	//Activate serial link
	bmi_sensor_active(1);		      // rising edge clock
	configure_serializer (bmi_ov2640);

	adap = &bmi_ov2640->bdev->adap;
        set_sync (adap);


        for (i = 0; i < 10; i++) {

                msleep(10);

                if(check_camera_lock()) {
                        break;
                }
                else {
                        printk(KERN_ERR "bmi_ov2640_activate() -  camera serializer did not lock,i = %d\n", i);
                }

        }
        clear_sync (adap);

exit:
	return rc;
}

int bmi_ov2640_deactivate (struct bmi_cam *cam)
{
	struct bmi_ov2640 *bmi_ov2640;
	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);

	//De-activate serial link     
	deconfigure_serializer (bmi_ov2640);
	bmi_sensor_inactive();

	//uninstall button irq_handler
	free_irq(bmi_ov2640->irq, bmi_ov2640);
	return 0;
}


void bmi_ov2640_link_enable (struct bmi_cam *cam)
{
	int i;
	struct i2c_adapter *adap;
	struct bmi_ov2640 *bmi_ov2640;
	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);

	//Activate serial link
	bmi_sensor_active(1);		      // rising edge clock
	configure_serializer (bmi_ov2640);

	adap =	bmi_device_get_i2c_adapter (bmi_ov2640->bdev);

        set_sync (adap);


	//REWORK: Speed this up. (shorten delay)

        for (i = 0; i < 10; i++) {

                msleep(10);

                if(check_camera_lock()) {
                        break;
                }
                else {
                        printk(KERN_ERR "bmi_ov2640_activate() -  camera serializer did not lock,i = %d\n", i);
                }

        }
        clear_sync (adap);
	return;
}


void bmi_ov2640_link_disable (struct bmi_cam *cam)
{
	struct bmi_ov2640 *bmi_ov2640;
	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);


	//De-activate serial link     
	deconfigure_serializer (bmi_ov2640);
	bmi_sensor_inactive();

	return;
}


int bmi_ov2640_flash_led_off (struct bmi_cam *cam) 
{
	struct bmi_ov2640 *bmi_ov2640;
	struct bmi_device *bdev;
	int slot;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);
	bdev = bmi_ov2640->bdev;

	slot = bmi_device_get_slot (bdev);

	bmi_set_module_gpio_data (slot, 2, 0);	// Flash LED off.
	return 0;
}

int bmi_ov2640_flash_led_on (struct bmi_cam *cam) 
{
	struct bmi_ov2640 *bmi_ov2640;
	struct bmi_device *bdev;
	int slot;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);
	bdev = bmi_ov2640->bdev;

	slot = bmi_device_get_slot (bdev);

	bmi_set_module_gpio_data (slot, 2, 1);	// Flash LED on.
	return 0;
}

int bmi_ov2640_flash_high_beam (struct bmi_cam *cam) 
{
	unsigned char	data;
	struct bmi_ov2640 *bmi_ov2640;
	struct i2c_adapter *adap;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);
	adap = bmi_device_get_i2c_adapter (bmi_ov2640->bdev);

  	ReadByte_IOX (adap, IOX_INPUT_REG, &data);
	data |= 0x10;				       	//High Beam
	WriteByte_IOX (adap, IOX_OUTPUT_REG, data);

	return 0;
}

int bmi_ov2640_flash_low_beam (struct bmi_cam *cam)
{
	unsigned char	data;
	struct bmi_ov2640 *bmi_ov2640;
	struct i2c_adapter *adap;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);
	adap = bmi_device_get_i2c_adapter (bmi_ov2640->bdev);

  	ReadByte_IOX (adap, IOX_INPUT_REG, &data);
	data &= ~(0x10);			       	// Low Beam
	WriteByte_IOX (adap, IOX_OUTPUT_REG, data);

	return 0;
}

int bmi_ov2640_red_led_off (struct bmi_cam *cam)
{
	struct bmi_ov2640 *bmi_ov2640;
	struct bmi_device *bdev;
	int slot;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);
	bdev = bmi_ov2640->bdev;

	slot = bmi_device_get_slot (bdev);

	bmi_set_module_gpio_data (slot, 3, 1);	// Red LED=OFF
	return 0;
}


int bmi_ov2640_red_led_on (struct bmi_cam *cam)
{
	struct bmi_ov2640 *bmi_ov2640;
	struct bmi_device *bdev;
	int slot;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);
	bdev = bmi_ov2640->bdev;

	slot = bmi_device_get_slot (bdev);

	bmi_set_module_gpio_data (slot, 3, 0);	// Red LED=ON
	return 0;
}


int bmi_ov2640_green_led_off (struct bmi_cam *cam)
{
	struct bmi_ov2640 *bmi_ov2640;
	struct bmi_device *bdev;
	struct i2c_adapter *adap;
        unsigned char iox_data;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);
	bdev = bmi_ov2640->bdev;

	adap = bmi_device_get_i2c_adapter (bdev);

	if ( bmi_device_present(bdev) ) {

	        ReadByte_IOX (adap, IOX_OUTPUT_REG, &iox_data);

		iox_data |=  (0x40);     //Set GRNLED to 1.(Off)

        	WriteByte_IOX (adap, IOX_OUTPUT_REG, iox_data);
	}
	return 0;
}

int bmi_ov2640_green_led_on (struct bmi_cam *cam)
{
	struct bmi_ov2640 *bmi_ov2640;
	struct bmi_device *bdev;
	struct i2c_adapter *adap;
        unsigned char iox_data;

	bmi_ov2640 = container_of(cam, struct bmi_ov2640, bcam);
	bdev = bmi_ov2640->bdev;

	adap = bmi_device_get_i2c_adapter (bdev);

	if ( bmi_device_present(bdev) ) {

	        ReadByte_IOX (adap, IOX_OUTPUT_REG, &iox_data);

		iox_data &=  ~(0x40);     //Set GRNLED to 0.(On)

        	WriteByte_IOX (adap, IOX_OUTPUT_REG, iox_data & 0xF0);
	}
	return 0;
}


int bmi_ov2640_probe(struct bmi_device *bdev)
{	

	int slot = bmi_device_get_slot(bdev);

	// allocate a driver-specific <this> structure

	struct bmi_ov2640 *bmi_ov2640 = kzalloc(sizeof(struct bmi_ov2640), GFP_KERNEL);
	if (!bmi_ov2640) {
	     return -1;
	}
	
	// attach <this> bmi_device structure (so we can find it later).

	bmi_device_set_drvdata(bdev, bmi_ov2640);
	printk(KERN_INFO "bmi_ov2640: probe: bmi_ov2640 pointer 0x%p \n",bdev->dev.driver_data);

	// initialize bmi_ov2640 struct
	bmi_ov2640->bdev = bdev;

	// sensor interface struct fields

	bmi_ov2640->bcam.interface.clk_mode   = 0;		  // gated          
	bmi_ov2640->bcam.interface.ext_vsync  = 1;		  // external vsync 
	bmi_ov2640->bcam.interface.Vsync_pol  = 0;		  // non-inverted   
	bmi_ov2640->bcam.interface.Hsync_pol  = 0; 		  // non-inverted   
	bmi_ov2640->bcam.interface.pixclk_pol = 0; 		  // non-inverted   
	bmi_ov2640->bcam.interface.data_pol   = 0; 		  // non-inverted   
	bmi_ov2640->bcam.interface.data_width = 1; 		  // 8-bits         
	bmi_ov2640->bcam.interface.width      = 1600-1;		  // 1280 - SXGA  1600 - UXGA
	bmi_ov2640->bcam.interface.height     = 1200-1;		  // 1024 - SXGA  1200 - UXGA

	bmi_ov2640->bcam.interface.pixel_fmt  = IPU_PIX_FMT_UYVY; // YUV422         
	bmi_ov2640->bcam.interface.mclk	      = 12000000;	  // frequency/src  

	//bmi_camera_sensor struct fields

	bmi_ov2640->bcam.sensor.set_color   = bmi_ov2640_set_color;  
	bmi_ov2640->bcam.sensor.get_color   = bmi_ov2640_get_color;  
	bmi_ov2640->bcam.sensor.set_ae_mode = bmi_ov2640_set_ae_mode;  
	bmi_ov2640->bcam.sensor.get_ae_mode = bmi_ov2640_get_ae_mode;  
	bmi_ov2640->bcam.sensor.config	    = bmi_ov2640_config;  
	bmi_ov2640->bcam.sensor.reset	    = bmi_ov2640_reset;  

	//bmi_camera_link struct fields

	bmi_ov2640->bcam.link.enable	    = bmi_ov2640_link_enable;  
	bmi_ov2640->bcam.link.disable	    = bmi_ov2640_link_disable;  

	//bmi_camera_cntl struct fields

	bmi_ov2640->bcam.cntl.flash_led_off    = bmi_ov2640_flash_led_off;
	bmi_ov2640->bcam.cntl.flash_led_on     = bmi_ov2640_flash_led_on;
	bmi_ov2640->bcam.cntl.flash_high_beam  = bmi_ov2640_flash_high_beam;
	bmi_ov2640->bcam.cntl.flash_low_beam   = bmi_ov2640_flash_low_beam;
	bmi_ov2640->bcam.cntl.red_led_off      = bmi_ov2640_red_led_off;
	bmi_ov2640->bcam.cntl.red_led_on       = bmi_ov2640_red_led_on;
	bmi_ov2640->bcam.cntl.green_led_off    = bmi_ov2640_green_led_off;
	bmi_ov2640->bcam.cntl.green_led_on     = bmi_ov2640_green_led_on;


	//bmi_cam struct fields

	bmi_ov2640->bcam.activate   = bmi_ov2640_activate  ;
	bmi_ov2640->bcam.deactivate = bmi_ov2640_deactivate; 

	//bmi_ov2640 struct fields
	bmi_ov2640->shutter  = 0;
	bmi_ov2640->zoomin   = 0; 
	bmi_ov2640->zoomout  = 0;
	bmi_ov2640->flash    = 0;	

	bmi_ov2640->irq = bmi_device_get_status_irq (bdev);	

	//initialize struct work_struct
	INIT_WORK (&bmi_ov2640->work, bmi_ov2640_buttons_work); 

       //Power stablization delay
        mdelay(500);

	//Do one-time hw initialization (e.g. patch)

	// configure IOX
	configure_IOX (bmi_ov2640);

	// configure GPIO
	configure_GPIO (bmi_ov2640);

	// chip enable camera
	enable_camera (bmi_ov2640);

	ov2640_patch (&bmi_ov2640->bdev->adap);

	//register with bug_camera

	//REWORK: check return code
  	register_bug_camera (&bmi_ov2640->bcam, slot);

	return 0;
}

void bmi_ov2640_remove(struct bmi_device *bdev)
{	

	//get our <this> pointer    
	struct bmi_ov2640 *bmi_ov2640 = (struct bmi_ov2640*)(bmi_device_get_drvdata (bdev));
	int slot = bmi_device_get_slot (bdev);

	if (bdev == NULL)
	  {
	    printk(KERN_INFO "bmi_ov2640: bmi-ov2640_remove: NULL Pointer on bdev\n");	    
	    return;
	  }
	if (bmi_ov2640 == NULL)
	  {
	    printk(KERN_INFO "bmi_ov2640: bmi_ov2640_remove: NULL Pointer on bmi_ov2640\n");
	    printk(KERN_INFO "bmi_ov2640: bmi_ov2640_remove: bdev->epid.vendor: 0x%x\n",bdev->epid.vendor);
	    printk(KERN_INFO "bmi_ov2640: bmi_ov2640_remove: bdev->epid.product: 0x%x\n",bdev->epid.product);
	    printk(KERN_INFO "bmi_ov2640: bmi_ov2640_remove: bdev->epid.revision: 0x%x\n",bdev->epid.revision);
	    printk(KERN_INFO "bmi_ov3640: bmi_ov2640_remove: bmi_ov2640 pointer 0x%p \n",bdev->dev.driver_data);
	    return;
	  }
	//unregister with bug_camera
  	unregister_bug_camera ( &bmi_ov2640->bcam, slot);

	disable_camera (bmi_ov2640);
	deconfigure_module (bmi_ov2640);

	
	flush_scheduled_work();

	//de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);

	//free driver-specific structure
	kfree (bmi_ov2640);
	return;
}


static __init int bmi_ov2640_init(void)
{	
	 printk("BMI OV2640 Camera Sensor Driver v%s \n", BMI_OV2640_VERSION);

//	Register with BMI bus.
	return  bmi_register_driver (&bmi_ov2640_driver); 

}

static void __exit bmi_ov2640_cleanup(void)
{	

//	UnRegister with BMI bus.
	bmi_unregister_driver (&bmi_ov2640_driver);
	return;
}


module_init(bmi_ov2640_init);
module_exit(bmi_ov2640_cleanup);

MODULE_AUTHOR("EnCADIS Design, Inc.");
MODULE_DESCRIPTION("OV2640 Camera Driver");
MODULE_LICENSE("GPL");

