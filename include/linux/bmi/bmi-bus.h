#ifndef BMI_BUS_H
#define BMI_BUS_H

#include <linux/bmi.h>

#define BMI_MAX_SLOTS 1

struct bmi_bus {

	struct bmi_device slot[BMI_MAX_SLOTS];
};



extern struct bus_type bmi_bus_type;

struct bmi_device* bmi_get_bmi_device (int slot_num);


#endif	/* BMI_BUS_H */

