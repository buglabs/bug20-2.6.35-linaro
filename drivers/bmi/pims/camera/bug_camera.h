#ifndef BUG_CAMARA_H
#define BUG_CAMARA_H

#include <linux/bmi.h>
#include <mach/bug_v4l2_capture.h>


struct bmi_cam;
struct input_dev;

# if 1
struct bmi_camera_sensor {

	void (*set_color) (struct bmi_cam *bcam, int bright, int saturation, int red, int green,
			   int blue);

	void (*get_color) (struct bmi_cam *bcam, int *bright, int *saturation, int *red, int *green,
			   int *blue);

	void (*set_ae_mode) (struct bmi_cam *bcam, int ae_mode);
	void (*get_ae_mode) (struct bmi_cam *bcam, int *ae_mode);
	sensor_interface *(*config) (struct bmi_cam *bcam, int *frame_rate, int high_quality);
	sensor_interface *(*reset) (struct bmi_cam *bcam);
};

struct bmi_camera_link {

	void (*enable) (struct bmi_cam *bcam);
	void (*disable) (struct bmi_cam *bcam);

};
# endif

struct bmi_camera_cntl {

	int (*flash_led_on) (struct bmi_cam *bcam);         
	int (*flash_led_off) (struct bmi_cam *bcam);         
	int (*flash_high_beam) (struct bmi_cam *bcam); 
	int (*flash_low_beam ) (struct bmi_cam *bcam); 
	int (*red_led_off) (struct bmi_cam *bcam); 
	int (*red_led_on) (struct bmi_cam *bcam); 
	int (*green_led_off) (struct bmi_cam *bcam); 
	int (*green_led_on) (struct bmi_cam *bcam); 

};

struct bmi_cam {

	sensor_interface  interface;		// pointer to this struct is returned by config()

	struct bmi_camera_sensor sensor;	// v4l function pointers

	struct bmi_camera_link link;		// bug link control function pointers

	struct bmi_camera_cntl cntl;		// bmi camera control function pointers


	int (*activate) (struct bmi_cam *cam, struct input_dev *idev);

	int (*deactivate) (struct bmi_cam *cam);


};



int register_bug_camera( struct bmi_cam *bcam, int slot);
int unregister_bug_camera( struct bmi_cam *bcam, int slot);


#endif

