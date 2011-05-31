/*
 * 	bmi_video.c
 *
 *  Bug Labs BugVideo (bmi_video) driver. This is the kernel plugin for managing
 * video output and plug events for DVI/VGA video via the BugVideo module
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
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/gpio.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <plat/hardware.h>

#include <plat/display.h>
#include <linux/fb.h>
#include <linux/list.h>

#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_video.h>

//static LIST_HEAD(mode_list);

/*
 * 	Global variables
 */

//dss display structs
static struct omap_dss_device *g_dvi_disp;
static struct omap_dss_device *g_vga_disp;


//private device structure
struct bmi_video
{
  
	//Bug BMI specific data
	struct bmi_device    *bdev;                           // BMI device
	
	//general Linux OS data
	struct cdev		cdev;		// control device
	struct device		*class_dev;                  // control class device

	//hardware interface data
 	struct i2c_client       *dvi_controller; //device for tfp410 dvi controller
 	struct i2c_client       *vga_controller; //device for ths8200 vga controller
 	struct i2c_client	*gpio_controller; //GPIO chip on bmi_video
	struct i2c_client	*swap_eeprom;    //device for eeprom. See footnote 1.
	struct i2c_client	*eeprom_switch; //eeprom expander switch. footnote 1
	
	//internal state mgmt
 	int	vmode;
	u8	swap_eeprom_state; // see footnote 1
	u8*	vga_monitor_edid; //created by a library. we must free
	u8*	dvi_monitor_edid; //created by a library. we must free 

	//legacy	
  	char	int_name[20];                // interrupt name

};

struct bmi_video g_bmi_video;
static int major;


/* 
 * Setup and destroy functions called by gpio controller as it
 * is created/destroyed. Testing only 
*/
#ifdef BMI_VIDEO_DEBUG
int pca_test_platform_setup (struct i2c_client *client, unsigned gpio, 
	unsigned ngpio, void *context)
{
        //printk(KERN_ERR "i2c gpio chip setup callback");
        return 0;
}
int pca_test_platform_teardown(struct i2c_client *client, unsigned gpio, 
	unsigned ngpio, void *context)
{
        //printk(KERN_ERR "i2c gpio chip teardown callback");
        return 0;
}
#endif /*BMI_VIDEO_DEBUG*/

/* sets the LED on the bmi video hardware
 * @value - 0 = OFF. 1= GREEN, 2=RED
 * @returns 0 always
 */
int bmi_video_ledset(int value)
{
	int green = 0, red = 0;
	switch (value)
	{
		default: //falls through on purpose
		case VIDEO_LED_RED: { green = 1; red = 0; }
		break; 
		case VIDEO_LED_GREEN: { green = 0; red = 1; }
		break; 
		case VIDEO_LED_BROWN: { green = 0; red = 0; }
		break; 
		case VIDEO_LED_OFF: { green = 1; red = 1; }
		break;
	}
	gpio_set_value_cansleep(RED_LED_GPIO,red);
	gpio_set_value_cansleep(GREEN_LED_GPIO,green);
	return 0;
}


/* 
 * This function changes the bus configuration to switch which mode is 'on'
 * for addr 0x50. See footnote 1
 *
 * OFF: The Video module eeprom data (default)
 * VGA: Used to read VGA data from VESA eeprom
 * DVI: Used to read DVI data from DVI connector
 * @returns 0 on success, nonzero otherwise 
*/ 
int do_eeprom_swap(int mode, struct bmi_video *video)
{
	int err = -EINVAL; //dafault, invalid
	u8 gpioState = 1;
	u8 switchFlag = 0; 
 
	if(mode == OFF){
		gpioState = 1;//mod eeprom on 
		switchFlag = 0x00; // off, switch inactive
	} else if (mode == VGA) {
		gpioState = 0; //mod eeprom off
		switchFlag = 0x02; //bit 0 = chan 1 = vga    
	} else if (mode == DVI) {
		gpioState = 0; //mod eeprom off
		switchFlag = 0x01; //bit1 = ch1 = DVI
	} else {
		//leave i2c bus as-is
		printk(KERN_WARNING "invalid i2c addr swap mode");
		return -EINVAL; //invalid setting
    }
	//set i2c bus on
	gpio_set_value_cansleep(EEPROM_SELECT_GPIO, gpioState); 
	err = i2c_smbus_write_byte(video->eeprom_switch, switchFlag); 
    
	if(err < 0)
		printk(KERN_ERR "error setting i2cswitch %d", err);
	else
		video->swap_eeprom_state = mode;
	return err;
}

/*
 * General helper function to download EDID data from a i2c eeprom
 * and pass it to the caller.
 * on success returns a kmallco'd block of memory for the caller to free,
 * containing edid data. On failure, returns null
 */

u8* download_edid(struct i2c_client *client)
{
	struct i2c_adapter *adapter = 0;
	u8 *fb  = 0x00;

	if(client) {       
		adapter = client->adapter;
		if(adapter) {
			//TRICKY: used instead of fb_read_edid for i2c bug reasons
			fb = fb_do_probe_ddc_edid(adapter);
	 	}
		else            
			printk(KERN_ERR "i2c adapter null fail."); 
	}       
	else    
		printk(KERN_ERR " i2c client is null. read monitor data fail");
	return fb;
}

int edid_data_to_screen_info(u8* edid, struct fb_var_screeninfo* var)
{
	int err = 0;
	
	if(edid) {
		err = fb_parse_edid(edid, var);
		if(err)
			printk(KERN_ERR "fb_parse_edid error %d", err);
	}
	else {
		printk(KERN_ERR "edid data is null"); 
		err = -ENOTTY; //TOOD make a better error
	}
	return err;
}
	

/*int screen_info_to_bugfb_info(u8* edid_data,  i
*/

/* 
 * Scans for EDID data from connected monitors.
 * if edid data is available, it is saved in the video 
 * structure. If no edid data is available, that data is left as 
 * null.
 * NOTE: this will change video->swap_eeprom_state as a side effect
 * @param vmode - AUTO - scans VGA/EGA. VGA or EGA scans just VGA/EGA. OFF 
 *  			deletes existing EDID data and does not refetch it
 */ 
int scan_for_monitors(int mode, struct bmi_video* video)
{
	int err = 0;
	//for all modes, free the old monitor data.
	kfree(video->vga_monitor_edid);
	kfree(video->dvi_monitor_edid);

	if(mode == OFF)
		return 0;

	if(mode == AUTO || mode == DVI ) {
		err = do_eeprom_swap(DVI,video);
		if(err == 0) {
			video->dvi_monitor_edid = download_edid(video->swap_eeprom);
			if(video->dvi_monitor_edid == NULL)
				printk(KERN_INFO "dvi monitor data does not exist :(");
		}			
		else
			printk(KERN_ERR "could not get edid data from dvi %d", err);
	}		
	
	if(mode == AUTO || mode == VGA ) {
		err = do_eeprom_swap(VGA,video);
		if(err == 0) {
			video->vga_monitor_edid = download_edid(video->swap_eeprom);
			if(video->vga_monitor_edid == NULL)
				printk(KERN_INFO "vga monitor data does not exist :(");
		}			
		else
			printk(KERN_ERR "could not get edid data from vga %d", err);
	}		
	return err;
}


	

/*
 * sysfs output to show videomode. 
 */
static ssize_t bmi_video_vmode_show(struct device *dev, 
				    struct device_attribute *attr, char *buf)
{
    int i;
    int len = 0;

    struct bmi_video *video = dev_get_drvdata(dev);

	for(i = 0; i < sizeof(mode_list); i++) {
		if(video->vmode == mode_list[i])
	    	len += sprintf(buf+len, "[%s] ", mode_names_list[i]);
		else
		    len += sprintf(buf+len, "%s ", mode_names_list[i]);
	}
    len += sprintf(len+buf, "\n");
    return len;
}

/* 
	sysfs input, to set the current videomode 
*/	
static ssize_t bmi_video_vmode_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t len)
{
	struct bmi_video *video = dev_get_drvdata(dev);
	int err = 0; 
	
	if (strstr(buf, "dvi") != NULL || strstr(buf, "DVI") != NULL){
		err = scan_for_monitors(DVI, video);
		if(err)  {
			printk(KERN_ERR "scan_for_monitors DVI err %d",err);
			return len;
		}		
		enable_dvi(video); 
	}
	else if (strstr(buf, "vga") != NULL || strstr(buf, "VGA") != NULL){
		err = scan_for_monitors(VGA, video);
		if(err)  {
			printk(KERN_ERR "scan_for_monitors VGA err %d",err);
			return len;
		}		
		enable_vga(video);
	}
	else if (strstr(buf, "auto") != NULL || strstr(buf, "AUTO") != NULL){

		err = scan_for_monitors(AUTO, video);
		if(err)  {
			printk(KERN_ERR "scan_for_monitors err %d",err);
			return len;
		}
		if(video->dvi_monitor_edid) {	
			//printk(KERN_INFO "dvi monitor detected");
			enable_dvi(video);
		}	
		else if(video->vga_monitor_edid) {	
			//printk(KERN_INFO "vga monitor detected");
			enable_vga(video);
		}
		else {
			printk(KERN_INFO "no monitor detected");
			monitors_off(video);
		}		
	}
	//test functions 	
	else if (strstr(buf, "t1") != NULL){
		ths8200_reset(VGA_RESET_GPIO,video->vga_controller);
		tfp410_disable(video->dvi_controller);

	}
	else if (strstr(buf, "t2") != NULL){
		ths8200_reset(VGA_RESET_GPIO,video->vga_controller);
		tfp410_enable(video->dvi_controller);
	}
	else if (strstr(buf, "t3") != NULL){
		ths8200_enable(video->vga_controller);
	}
	else if (strstr(buf, "t4") != NULL){
		ths8200_disable(video->vga_controller);
	}
	else if (strstr(buf, "t5") != NULL){
		ths8200_init(video->vga_controller);
	}
	else if (strstr(buf, "off") != NULL || strstr(buf, "OFF") != NULL){
		ths8200_reset(VGA_RESET_GPIO,video->vga_controller);
		tfp410_disable(video->dvi_controller);
		ths8200_disable(video->vga_controller);
	}
	return len;
}

static DEVICE_ATTR(vmode, 0664, bmi_video_vmode_show, bmi_video_vmode_store);

static ssize_t bmi_video_vga_edid_show(struct device *dev, 
				    struct device_attribute *attr, char *buf)
{
    struct bmi_video *video = dev_get_drvdata(dev);
	struct fb_monspecs monspecs;	/* Current Monitor specs */
	if(video->vga_monitor_edid) {
		ssize_t len = 0;
		fb_edid_to_monspecs( video->vga_monitor_edid, &monspecs);
		len = fb_edid_show(&monspecs, buf, PAGE_SIZE, 5);
		fb_destroy_modedb(monspecs.modedb);
		return len; 
	}
	
	return snprintf(buf,PAGE_SIZE,"(none)\n");
}

static ssize_t bmi_video_dvi_edid_show(struct device *dev, 
				    struct device_attribute *attr, char *buf)
{
    struct bmi_video *video = dev_get_drvdata(dev);
	struct fb_monspecs monspecs;	/* Current Monitor specs */
	if(video->dvi_monitor_edid) {
		ssize_t len = 0;
		fb_edid_to_monspecs( video->dvi_monitor_edid, &monspecs);
		len = fb_edid_show(&monspecs, buf, PAGE_SIZE, 5);
		fb_destroy_modedb(monspecs.modedb);
		return len; 
	}
	
	return snprintf(buf,PAGE_SIZE,"(none)\n");
}




static DEVICE_ATTR(vga_edid, 0644, bmi_video_vga_edid_show, NULL);
static DEVICE_ATTR(dvi_edid, 0644, bmi_video_dvi_edid_show, NULL);



#ifdef BMI_VIDEO_DEBUG

#define GPIO_DIR_CANT_CHANGE (0)
#define GPIO_DIR_CAN_CHANGE (1)

void dbg_export_gpios_to_sysfs(void)
{

	int iRet = 0;
	printk(KERN_ERR "DEBUG: gpio_export to sysfs");
	iRet = gpio_export(DVI_RESET_GPIO, GPIO_DIR_CANT_CHANGE );
	iRet = gpio_export(I2C_SW_RESET_GPIO, GPIO_DIR_CANT_CHANGE); //to test
	iRet = gpio_export(GREEN_LED_GPIO, GPIO_DIR_CANT_CHANGE );
	iRet = gpio_export( RED_LED_GPIO,GPIO_DIR_CANT_CHANGE );
	iRet = gpio_export( EEPROM_SELECT_GPIO, GPIO_DIR_CANT_CHANGE );
}

void dbg_unexport_gpios_to_sysfs()
{
	printk(KERN_ERR "DEBUG: gpio_unexport from sysfs");
	gpio_unexport(DVI_RESET_GPIO);
	gpio_unexport(I2C_SW_RESET_GPIO);
	gpio_unexport(GREEN_LED_GPIO);
	gpio_unexport( RED_LED_GPIO);
	gpio_unexport( EEPROM_SELECT_GPIO);
}

#endif /* BMI_VIDEO_DEBUG*/


/*
 * 	BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_video_tbl[] = 
{ 
	{ 
		.match_flags  = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor           = BMI_VENDOR_BUG_LABS, 
		.product          = BMI_PRODUCT_VIDEO, 
		.revision         = BMI_ANY, 
	}, 
	{ 0, },	  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_video_tbl);

	int		bmi_video_probe (struct bmi_device *bdev);
	void	bmi_video_remove (struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_video_driver = 
{
	.name		= "bmi_video", 
	.id_table	= bmi_video_tbl, 
	.probe		= bmi_video_probe, 
	.remove		= bmi_video_remove, 
};

/*
 *	PIM functions
 */

// interrupt handler
//static irqreturn_t module_irq_handler(int irq, void *dummy)
//{
//	return IRQ_HANDLED;
//}

/*
 * 	BMI functions
 */
// probe - insert PIM
int bmi_video_probe(struct bmi_device *bdev)
{
	int err = 0;
	int slot;
	int irq;

	struct bmi_video *video;
	struct class *bmi_class;
 	struct i2c_adapter *adap;
	struct omap_dss_device *dssdev;

	printk (KERN_INFO "bmi_video.c: probe...\n");	

	slot = bdev->slot->slotnum;
	adap = bdev->slot->adap;
	video = &g_bmi_video;
	
	video->dvi_monitor_edid = NULL; 
	video->vga_monitor_edid = NULL; 

	video->bdev = 0;
	dssdev = NULL;
	g_dvi_disp = NULL;
	g_vga_disp = NULL;
	
	/* disable all video devices, store vga/dvi entries */
	for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);

		if (dssdev->state)
		        dssdev->driver->disable(dssdev);
		// our dss names are from buglabs board files 	
		if (strnicmp(dssdev->name, "dvi", 3) == 0)
		        g_dvi_disp = dssdev;
		else if (strnicmp(dssdev->name, "vga", 3) == 0)
		        g_vga_disp = dssdev;
	}

	// bind driver and bmi_device 
	video->bdev = bdev;

	// create class device 
	bmi_class = bmi_get_class();

	// -- grab eeprom addr
	if(bdev->slot->eeprom)
	{
		video->swap_eeprom = bdev->slot->eeprom;
		video->swap_eeprom_state = OFF; //set to read bmi board eeprom
		//printk(KERN_ERR "swizzling slot eeprom, addr %x", 
			// (unsigned int)video->swap_eeprom->addr);
	}
	else {
		printk(KERN_ERR "module eeprom is null. Module type is impossible");
		video->swap_eeprom = 0x00;
		video->swap_eeprom_state = OFF;
	}

	// -- grap i2c expander switch
	video->eeprom_switch = i2c_new_device(adap, &i2cswitch_info);
	if(video->eeprom_switch == NULL) {
		printk(KERN_ERR "i2c addr %x fail", i2cswitch_info.addr);
		err = -ENOTTY;
		goto probe_fail1;
	}
	// -- grab i2c gpio chip and set it up	
	if (gpio_is_valid(VIDEO_GPIO_BASE))
		video->gpio_controller = i2c_new_device(adap,&gpio_controller_info);
	else
	{
		printk(KERN_ERR "i2c addr %x in use", VIDEO_GPIO_BASE);
		err = -ENOTTY;
		goto probe_fail2;
	}
		
	if(video->gpio_controller == NULL)
	{
		printk(KERN_ERR "i2c addr %x fail", gpio_controller_info.addr);
		err = -ENOTTY;
		goto probe_fail2;
	}

	err = gpio_request_array(vid_gpios, ARRAY_SIZE(vid_gpios));
	if (err) {
		printk(KERN_ERR "GPIO's not requestable. Damm");
		err = -ENOTTY;
		goto probe_fail3;
	}
	
	//dbg_export_gpios_to_sysfs();

	//we have gpio's now, turn the LED to red since we are not initalized
	bmi_video_ledset(VIDEO_LED_RED);

	// -- grab i2c for the dvi controller
	video->dvi_controller = i2c_new_device(adap, &tfp_info);
	if (video->dvi_controller == NULL){
		printk(KERN_ERR "DVI monitor controller not found\n");
		err = -ENOTTY;
		goto probe_fail4;

	}

	video->vga_controller = i2c_new_device(adap, &ths_info);
	if (video->vga_controller == NULL) {
		printk(KERN_ERR "VGA monitor controller not found\n");
		err = -ENOTTY;
		goto probe_fail5;
	}
	
	//FUTURE: enable pulg event interrupt for GPIO
	//ths_info.irq = gpio_to_irq(DVI_MONITOR_SENSE_GPIO);
	
	// -- to get ths8201 on the bus OK, we need to do a reset cycle
	ths8200_disable(video->vga_controller);//TRICKY: does some setup -> 
		//didn't have time to debug why this works and 'init' doesn't
	ths8200_standby(VGA_RESET_GPIO);

	err = device_create_file(&bdev->dev, &dev_attr_vmode);
	if (err < 0)
		printk(KERN_ERR "Error creating SYSFS entries...\n");
		//TRCIKY: don't exit on this error, tolerate it

	err = scan_for_monitors(AUTO, video);

	//based on monitor's found, choose the best'
	if(video->dvi_monitor_edid) {	
		printk(KERN_INFO "dvi monitor detected");
		enable_dvi(video);
	}	
	else if(video->vga_monitor_edid) {	
		printk(KERN_INFO "vga monitor detected");
		enable_vga(video);
	}
	else {
		printk(KERN_ERR "no monitor detected");
		monitors_off(video);
	}			

	err = device_create_file(&bdev->dev, &dev_attr_vga_edid);
	if (err < 0)
		printk(KERN_ERR "Error creating SYSFS entries...\n");
		//TRCIKY: don't exit on this error, tolerate it

	err = device_create_file(&bdev->dev, &dev_attr_dvi_edid);
	if (err < 0)
		printk(KERN_ERR "Error creating SYSFS entries...\n");
		//TRCIKY: don't exit on this error, tolerate it


	//request PIM interrupt
	irq = bdev->slot->status_irq;
	sprintf (video->int_name, "bmi_video%d", slot);

	bmi_device_set_drvdata (bdev, video);
	// -- successfuly loaded, even if not everyting is perfect
	return 0;

// -- failure/error cleanup
probe_fail5:
	if(video->dvi_controller) {
		i2c_unregister_device(video->dvi_controller);
		video->dvi_controller = NULL;
	}
probe_fail4:
	//dbg_unexport_gpios_to_sysfs();
probe_fail3:
	if(video->gpio_controller) {
		i2c_unregister_device(video->gpio_controller);
		video->gpio_controller = NULL;
	}
probe_fail2:
	if(video->eeprom_switch) {
		i2c_unregister_device(video->eeprom_switch);
		video->eeprom_switch = NULL;
	}
probe_fail1:
	video->swap_eeprom = 0x00;
	video->swap_eeprom_state = OFF;

	return err;
}

/* 
 * Function is called when the video module hardwrae is physically removed
 */  
void bmi_video_remove(struct bmi_device *bdev)
{	
	int irq;
	int i;
	int slot;

	struct bmi_video *video;
	struct class *bmi_class;
	static struct omap_dss_device *dssdev;

	printk(KERN_DEBUG "bmi_video: Module Removed...\n");

	//Would turn LED red, but we can't trust the HW is connected still

	slot = bdev->slot->slotnum;
	video = &g_bmi_video;

	//removals have known unwind errors iff rmmod bmi_video_core while
	//device is plugged in
	device_remove_file(&bdev->dev, &dev_attr_vmode);
	device_remove_file(&bdev->dev, &dev_attr_vga_edid);
	device_remove_file(&bdev->dev, &dev_attr_dvi_edid);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	// -- disable displays
	monitors_off_safe(video);//safe for missing HW

	// -- disable all displays
	for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);

		if (dssdev->state)
		        dssdev->driver->disable(dssdev);
	}


	if(video->swap_eeprom){
		//we are just 'borrowing' this pointer,
		//do not unregister! see bmi_video_probe.
		video->swap_eeprom = NULL;
		video->swap_eeprom_state = OFF;
	}

	if(video->eeprom_switch) {
		i2c_unregister_device(video->eeprom_switch);
		video->eeprom_switch = NULL;
	}

	if(video->gpio_controller) {
		//dbg_unexport_gpios_to_sysfs();
		gpio_free_array(vid_gpios, ARRAY_SIZE(vid_gpios));
		i2c_unregister_device(video->gpio_controller);
		video->gpio_controller= NULL;
	}

	if(video->dvi_controller) {
		i2c_unregister_device(video->dvi_controller);
		video->dvi_controller = NULL;
	}

	printk(KERN_INFO "bmi_video: 9\n");
	if(video->vga_controller) {
		i2c_unregister_device(video->vga_controller);
		video->vga_controller = NULL;
	}
	irq = bdev->slot->status_irq;

	// -- legacy vodoo code. Left in for safety
	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	if (video->vga_monitor_edid) {
		kfree(video->vga_monitor_edid); //created by a library. we must free
		video->vga_monitor_edid = NULL;
	}
	if (video->dvi_monitor_edid) {
		kfree(video->dvi_monitor_edid); //created by a library. we must free
		video->dvi_monitor_edid = NULL;
	}
	//just in case, clear the global
	g_dvi_disp = NULL;
	g_vga_disp = NULL;

	video->class_dev = 0;

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	video->bdev = 0;

	printk(KERN_INFO "bmi_video_remove done ");

	return; //Clean, squeaky clean
}

/*
 * Function taks a mode list, and deletes entries that are
 * too large or too small to fit our monitor use.
 *
 *
 *  based on code from modedb.c
 *
 */
void fb_delete_outranged_videomode(struct list_head *head, int minx,int miny,
	int maxx, int maxy)
{
	struct list_head *pos, *n;
	struct fb_modelist *modelist;
	struct fb_videomode *mode;

	list_for_each_safe(pos, n, head) {
		modelist = list_entry(pos, struct fb_modelist, list);
		mode = &modelist->mode;
		if(mode->yres > maxy || mode->xres > maxx)	{
			//printk(KERN_INFO "deleting max %d x %d",mode->xres, mode->yres);
			list_del(pos);
			kfree(pos);
		}
		if(mode->yres < miny || mode->xres < minx){
			//printk(KERN_INFO "deleting min %d x %d",mode->xres, mode->yres);
			list_del(pos);
			kfree(pos);
		}
	}	
}


/*
 *	module routines
 */
static int enable_vga(struct bmi_video *video)
{
	int err;
	struct fb_info *info;
	struct fb_var_screeninfo var;
	struct omap_overlay_manager *mgr;
 	const struct fb_videomode* matched_mode;
	struct list_head *pos, *n;
	printk (KERN_INFO "bmi_video.c: setting up VGA Output...\n");

	// -- disable displays
	if (video->vmode == DVI) {
		ths8200_reset(VGA_RESET_GPIO,video->vga_controller);
		tfp410_disable(video->dvi_controller);
	}
	if (g_dvi_disp->state)
	        g_dvi_disp->driver->disable(g_dvi_disp);

	if (video->vmode == VGA){ 
	    ths8200_disable(video->vga_controller);
		ths8200_reset(VGA_RESET_GPIO,video->vga_controller);
	}
	if (g_vga_disp->state)
	        g_vga_disp->driver->disable(g_vga_disp);
	video->vmode = OFF;//set (temp) video mode

	mgr = omap_dss_get_overlay_manager(0);
	if(mgr == NULL) {
		printk(KERN_ERR "omapl_dss_overlay_mgr fail.");
		return -ENOTTY;
	}

	err = mgr->unset_device(mgr);
	if(err) {
		printk(KERN_INFO "unset device fail");
		//TRICKY: continue anway, we *wanted* no device set
	}

	//set omapfb
	info = registered_fb[0];
	if(info == NULL)  { 
		printk(KERN_ERR "no registered_fb[0]. Total video failure expected");
		return -ENOTTY;
	}
	var = info->var;	

	//-- test our monitor generated edid
	err =  edid_data_to_screen_info(video->vga_monitor_edid, &var);
	if(err) {
		printk(KERN_ERR "data_to_screen_info error %d", err);
		err = -ENOTTY;
		goto cont;
	}

	// get rid of the old list
	if(list_empty(&info->modelist) == 0) {
		fb_destroy_modelist(&info->modelist);

	}
	/* causes crashes. can't rely on info->monsepcs yet
	if(info->monspecs.modedb != NULL) {
		kfree(info->monspecs.modedb);
	}*/

	// -- convert the monitor EDID into something usable
	fb_edid_to_monspecs( video->vga_monitor_edid, &info->monspecs);
	fb_videomode_to_modelist(info->monspecs.modedb,
		info->monspecs.modedb_len, &info->modelist);

	fb_delete_outranged_videomode(&info->modelist,
		BUG_VIDEO_XMIN, BUG_VIDEO_YMIN, BUG_VIDEO_XMAX,  BUG_VIDEO_YMAX);

	// -- walk list of remaining modes, pick the best, delete errornous  ones
	list_for_each_safe(pos, n, &info->modelist)
	{
		matched_mode = fb_find_nearest_mode((const struct fb_videomode*)&var,
			pos);
		if(matched_mode)
		{
			fb_videomode_to_var(&var, matched_mode);
			//fb_printk_var_screeninfo(&var);
		}
		else {
			err = -ENOTTY;
			goto cont;
		}

		//used to test ACTIVATE_TEST always throws an error
		var.activate = FB_ACTIVATE_FORCE;
		err = fb_set_var(info, &var);

		//bad mode by erro, delete it
		if (err != 0) {
			printk(KERN_INFO "bad mode, err %d, delete it 1", err);
			fb_delete_videomode(matched_mode, pos);
			matched_mode = 0x00;
			var = info->var;
		}
		//bad mode by size. Vestigal code, may be removed FUTURE
		else if (var.xres > 1024 || var.yres > 768) {
			printk(KERN_INFO "bad mode, delete it 2");
			fb_delete_videomode(matched_mode, pos);
			matched_mode = 0x00;
			var = info->var;
		}
		else
			goto cont;
	}

cont:
	// -- fallback on failure to legacy settings code
	if(err)
	{
		printk(KERN_INFO"fb_find_best_mode FAILED: fallback");
		var.xres = 1024;
		var.yres = 768;
		var.xres_virtual = 1024;
		var.yres_virtual = 768;
	}

	//printk(KERN_ERR "fb_find_mode got %d", err);
	//fb_printk_var_screeninfo(&var);
	var.activate = FB_ACTIVATE_FORCE;

	err = fb_set_var(info, &var);

	// -- Ultimate last-chance fallback case
	if (err) {
        printk(KERN_ERR "bmi_video.c: enable_vga total failure %d",err);
		//TOTAL FAILURE. OMG
		bmi_video_ledset(VIDEO_LED_RED);
		return err;
	}

	//-- enable vga (dss)
	if (g_vga_disp->state != 1) {
		mgr->set_device(mgr, g_vga_disp);
		mgr->apply(mgr);
	        g_vga_disp->driver->enable(g_vga_disp);
	}
	//init vga (ths)
	ths8200_init(video->vga_controller); //FUTURE: rename 'enable' and 'init' func's for clarity
	video->vmode = VGA;
	bmi_video_ledset(VIDEO_LED_OFF);

	return err;
}

/* FUTURE: use for cases when montior has error
void monitor_error(struct bmi_video *video)
{
	bmi_video_ledset(VIDEO_LED_RED);
	//FUTURE: check data structure for errors
	video->vmode = OFF;
}
*/

static void monitors_off(struct bmi_video *video)
{
	struct fb_info *info;
	// -- disable displays
	if (video->vmode == DVI) {
		ths8200_reset(VGA_RESET_GPIO,video->vga_controller);
		tfp410_disable(video->dvi_controller);
	}
	if (g_dvi_disp->state)
	        g_dvi_disp->driver->disable(g_dvi_disp);

	if (video->vmode == VGA) {
	    ths8200_disable(video->vga_controller);
		ths8200_reset(VGA_RESET_GPIO,video->vga_controller);
	}
	if (g_vga_disp->state)
	        g_vga_disp->driver->disable(g_vga_disp);

	// clear out our modelist
	info = registered_fb[0];
	if(list_empty(&info->modelist) == 0) {
		fb_destroy_modelist(&info->modelist);
	}

	video->vmode = OFF;
	do_eeprom_swap(OFF, video);
	bmi_video_ledset(VIDEO_LED_GREEN);
}

/*
 * safe version of monitors_off to avoid i2c failure
 * if the device is physically removed when this runs
*/
static void monitors_off_safe(struct bmi_video *video)
{
	struct fb_info *info;
	// clear out our modelist
	info = registered_fb[0];
	if(list_empty(&info->modelist) == 0) {
		fb_destroy_modelist(&info->modelist);
	}

	video->vmode = OFF;
}

static int enable_dvi(struct bmi_video *video)
{
    int err;
	struct fb_info *info;
	struct fb_var_screeninfo var;
	struct omap_overlay_manager *mgr;
	struct fb_videomode* matched_mode;
	struct list_head *pos, *n;
	printk (KERN_INFO "bmi_video.c: setting up DVI Output...\n");

	// -- disable displays
	if (video->vmode == DVI) {
		ths8200_reset(VGA_RESET_GPIO,video->vga_controller);
		tfp410_disable(video->dvi_controller);
	}
	if (g_dvi_disp->state)
	        g_dvi_disp->driver->disable(g_dvi_disp);

	if (video->vmode == VGA) {
	    ths8200_disable(video->vga_controller);
		ths8200_reset(VGA_RESET_GPIO,video->vga_controller);
	}
	if (g_vga_disp->state)
	        g_vga_disp->driver->disable(g_vga_disp);
	video->vmode = OFF;//set (temp) video mode

	mgr = omap_dss_get_overlay_manager(0);
	if(mgr == NULL) {
		printk(KERN_ERR "omapl_dss_overlay_mgr fail.");
		return -ENOTTY;
	}

	err = mgr->unset_device(mgr);
	if(err) {
		printk(KERN_INFO "unset device fail");
		//TRICKY: continue anway, we *wanted* no device set
	}

	//set omapfb
	info = registered_fb[0];
	if(info == NULL)  {
		printk(KERN_ERR "no registered_fb[0]. Total video failure expected");
		return -ENOTTY;
	}
	var = info->var;

	//-- test our monitor generated edid
	err =  edid_data_to_screen_info(video->dvi_monitor_edid, &var);
	if(err) {
		printk(KERN_ERR "data_to_screen_info error %d", err);
		err = -ENOTTY;
		goto cont; //error case, use dvi defaults 
	}

	// get rid of the old list
	if(list_empty(&info->modelist) == 0) {
		fb_destroy_modelist(&info->modelist);
	}

	// -- convert the monitor EDID into something usable
	fb_edid_to_monspecs( video->dvi_monitor_edid, &info->monspecs);
	fb_videomode_to_modelist(info->monspecs.modedb, info->monspecs.modedb_len,
		&info->modelist);

	fb_delete_outranged_videomode(&info->modelist,
		BUG_VIDEO_XMIN, BUG_VIDEO_YMIN, BUG_VIDEO_XMAX, BUG_VIDEO_YMAX);

	// -- walk list of remaiing modes, pick the best, delete bad modes
	list_for_each_safe(pos, n, &info->modelist)
	{
		matched_mode = fb_find_nearest_mode((const struct fb_videomode*)&var,
			pos);
		if(matched_mode)
		{
			fb_videomode_to_var(&var, matched_mode);
			//printk(KERN_ERR "fb_find_best_mode loop results");
			//fb_printk_var_screeninfo(&var);
		}
		else {
			err = -ENOTTY;
			goto cont;
		}
		//used to test ACTIVATE_TEST always throws an error
		var.activate = FB_ACTIVATE_FORCE;
		err = fb_set_var(info, &var);

		//bad mode, delete it
		if (err != 0) {
			printk(KERN_INFO "bad mode, delete it 1 (err %d)",err);
			fb_delete_videomode(matched_mode, pos);
			matched_mode = 0x00;
			var = info->var;
		}
		else if (var.xres > 1024 || var.yres > 768) {
			printk(KERN_INFO "bad mode, delete it 2");
			fb_delete_videomode(matched_mode,pos);
			matched_mode = 0x00;
			var = info->var;
		}
		else
			goto cont;
	}

cont:
	// -- fallback on failure to legacy settings code
	if(err)
	{
		printk(KERN_ERR "fb_find_best_mode FAILED: fallback");
		var.xres = 1024;
		var.yres = 768;
		var.xres_virtual = 1024;
		var.yres_virtual = 768;
	}

	//printk(KERN_ERR "fb_find_mode got %d", err);
	//fb_printk_var_screeninfo(&var);
	var.activate = FB_ACTIVATE_FORCE;

	err = fb_set_var(info, &var);
	// -- Total failure. Ultimate fallback case
	if (err) {
		printk(KERN_ERR "bmi_video.c: enable_dvi total failure %d", err);
		//TOTAL FAILURE. OMG
		bmi_video_ledset(VIDEO_LED_RED);
		return err;
	}
	// -- enable dvi (dss)
	if (g_dvi_disp->state != 1) {
		mgr->set_device(mgr, g_dvi_disp);
		mgr->apply(mgr);
	        g_dvi_disp->driver->enable(g_dvi_disp);
	}
	//init dvi (tfp)
	tfp410_init(video->dvi_controller);//FUTURE: rename 'enable' and 'init' func's for clarity
	video->vmode = DVI; //set (temp) new video mode
	bmi_video_ledset(VIDEO_LED_OFF);

	return err;
}

static void __exit bmi_video_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver(&bmi_video_driver);
	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}

static int __init bmi_video_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI VIDEO Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_video_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}

	printk (KERN_INFO "bmi_video.c: BMI_VIDEO Driver...\n");

	return 0;
}


module_init(bmi_video_init);
module_exit(bmi_video_cleanup);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Buglabs Inc.");
MODULE_DESCRIPTION("BMI video module device driver");
MODULE_VERSION(BMI_VIDEO_VERSION);


/* footnote 1:
 Swap_eeprom is a i2c device at addr 0x50. Due to design outside of our 
 control, that addr may need to point to bug module EEPROM ('OFF'),
 DVI monitor data ('DVI') or VGA monitor data 'VGA'). swap_eeprom_state 
 value keeps track of where it currently points.  To change between these
 uses, we need to turn on/off the expander, and/or change a GPIO line. It's
 complex, annoying, but works.
*/
