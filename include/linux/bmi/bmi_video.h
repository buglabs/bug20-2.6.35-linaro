/*
 *      bmi_video.c
 *
 * This is the core of bmi_video, a kernel module that runs
 * the BugVideo 'Bug Modle Interface' (BMI), a runtime plug-in
 * hardware module that can run a VGA or DVI display.
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

#include <linux/i2c/pca953x.h>
/* exported functions */
#include <../drivers/video/omap2/displays/ths8200.h>
#include <../drivers/video/omap2/displays/tfp410.h>

struct bmi_video;

#define BMI_VIDEO_VERSION "3.0.2"
//---  Video Modes
#define DVI 	(0) /* DVI video running */
#define VGA     (1)  /* VGA video running */
#define OFF     (2)  /* no video running */
#define AUTO	(3)  /* scanning for video, will change to DVI/VGA/OFF after scan*/

u8 		mode_list[] 		= { DVI, VGA, OFF, AUTO, };
char*	mode_names_list[]	= { "DVI","VGA","OFF","AUTO" };


#define BUG_VIDEO_XMIN	(640)
#define BUG_VIDEO_YMIN	(480)
#define BUG_VIDEO_XMAX	(1024)
#define BUG_VIDEO_YMAX	(768)


// -- Hardware interface to video board info
#define DVI_MONITOR_SENSE_GPIO (15) /*raw GPIO for monitor sensing */
#define VGA_RESET_GPIO (10) /*raw GPIO line for VGA reset */

// -- GIPO's we register to the system for some specific uses
#define VIDEO_GPIO_BASE (240) /*see Documentation/Bugbase2/gpio for info */
#define DVI_RESET_GPIO ( VIDEO_GPIO_BASE +  0)
#define I2C_SW_RESET_GPIO (VIDEO_GPIO_BASE + 1) /* set low to disable i2c switch+*/
#define GREEN_LED_GPIO (VIDEO_GPIO_BASE + 2) /* green LED drain */
#define RED_LED_GPIO (VIDEO_GPIO_BASE + 3) /* red LED drain */
#define EEPROM_SELECT_GPIO (VIDEO_GPIO_BASE +  4) /*enable/disable expander to r+ead module EEPROM*/
#define VIDEO_GPIO_UNUSED1 (VIDEO_GPIO_BASE + 5)
#define VIDEO_GPIO_UNUSED2 (VIDEO_GPIO_BASE + 6)
#define VIDEO_GPIO_UNUSED3 (VIDEO_GPIO_BASE + 7)

// --  GPIO structure to init/translate i2c gpio's to general linux gpio's
static struct gpio vid_gpios[] = {
// let tfp410 use DVI_RESET_GPIO when we are done 
	{ DVI_RESET_GPIO, GPIOF_INIT_HIGH, "BMI_VID DVI RESET GP+IO"}, 
        { I2C_SW_RESET_GPIO,  GPIOF_INIT_HIGH, "BMI_VID I2C SW GPIO"}, //high is+ 'use'
        { GREEN_LED_GPIO,  GPIOF_INIT_HIGH, "BMI_VID GREEN LED GPIO"},//led drai+n
        { RED_LED_GPIO,  GPIOF_INIT_LOW, "BMI_VID RED LED GPIO"},//led drain
        { EEPROM_SELECT_GPIO, GPIOF_INIT_LOW, "BMI_VID EEP SEL GPIO"},
        { VIDEO_GPIO_UNUSED1, GPIOF_INIT_LOW, "BMI_VID UNUSED1"},
        { VIDEO_GPIO_UNUSED2, GPIOF_INIT_LOW, "BMI_VID UNUSED2"},
        { VIDEO_GPIO_UNUSED3, GPIOF_INIT_LOW, "BMI_VID UNUSED3"},
};


// -- LED contoller functions
#define VIDEO_LED_OFF	(0)
#define VIDEO_LED_RED	(1)
#define VIDEO_LED_GREEN (2)
#define VIDEO_LED_BROWN (3) /*greep + red*/

// --  functions to properly init the pca9554 as an i2c GPIO controller

//pre decleration of callback funcitons for pca953x chip setup tsts 
#ifdef BMI_VIDEO_DEBUG
int pca_test_platform_setup(struct i2c_client *client, unsigned gpio, 
	unsigned ngpio, void *context); 
int pca_test_platform_teardown(struct i2c_client *client, unsigned gpio,  
	unsigned ngpio, void *context); 
#else /*BMI_VIDEO_DEBUG not defined */
	#define pca_test_platform_teardown NULL
	#define pca_test_platform_setup NULL
#endif 

//output enable prototypes
static int enable_dvi(struct bmi_video *video);
static int enable_vga(struct bmi_video *video);

#ifdef BMI_VIDEO_DEBUG
static void dbg_export_gpios_to_sysfs(void);
static void dbg_unexport_gpios_to_sysfs(void);
#endif /*BMI_VIDEO_DEBUG*/

static int do_eeprom_swap(int mode, struct bmi_video* video);
static void monitors_off(struct bmi_video *video);
static void monitors_off_safe(struct bmi_video *video);



// GPIO expander specific data
static struct pca953x_platform_data pca9554_plat = {
        .gpio_base = VIDEO_GPIO_BASE,
        .setup = pca_test_platform_setup,
        .teardown = pca_test_platform_teardown,
        .invert = 0,
//      .context multiplex_eeprom,
};

//DVI/HDMI controller chip i2c interface
static struct i2c_board_info tfp_info = {
  I2C_BOARD_INFO("tfp410p", 0x38), //0x70 raw
};

//vga video encoder
static struct i2c_board_info ths_info = {
  I2C_BOARD_INFO("ths8200", 0x20),//0x40 raw
};

//gpio controller
static struct i2c_board_info gpio_controller_info= {
  I2C_BOARD_INFO("pca9554",0x21), //0x42 raw
  .platform_data = &pca9554_plat,
};

// i2c switch for swap eeprom
static struct i2c_board_info i2cswitch_info = {
  I2C_BOARD_INFO("pca9543",0x71), //0xE2 raw
};

