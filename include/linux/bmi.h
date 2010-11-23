#ifndef __LINUX_BMI_H
#define __LINUX_BMI_H

#include <linux/bmi-ids.h>
#include <linux/bmi/bmi-eeprom.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>

/* BMI bus device table constants */
#define BMI_ANY					0x0

#define RED_LED		3
#define GREEN_LED	2
#define GPIO_1		1
#define GPIO_0		0

extern struct dev_pm_ops bmi_dev_pm_ops;
extern struct device_attribute bmi_dev_attrs[];
struct bmi_slot;

struct slot_actions {
  int (*present)(struct bmi_slot*);
  void (*power_on)(struct bmi_slot*);
  void (*power_off)(struct bmi_slot*);
  void (*gpio_direction_out)(struct bmi_slot*, unsigned gpio, int value);	/*Configure gpios as inputs/ouputs*/
  void (*gpio_direction_in)(struct bmi_slot*, unsigned gpio);
  int (*gpio_get_value)(struct bmi_slot*, unsigned gpio);
  void (*gpio_set_value)(struct bmi_slot*, unsigned gpio, int value);
  void (*uart_enable)(struct bmi_slot*);
  void (*uart_disable)(struct bmi_slot*);
  void (*spi_enable)(struct bmi_slot*);
  void (*spi_disable)(struct bmi_slot*);
  void (*audio_enable)(struct bmi_slot*);
  void (*audio_disable)(struct bmi_slot*);
  void (*batt_enable)(struct bmi_slot*);
  void (*batt_disable)(struct bmi_slot*);
};

struct bmi_slot {
  int slotnum;
  char* name;
  struct bmi_device *bdev;
  struct module *owner;
  struct device slotdev;
  struct kref kref;
  struct mutex pres_mutex;
  struct list_head  event_list;
  unsigned int event_bits[1];

  int present;
  struct i2c_adapter *adap;
  struct i2c_client  *eeprom;
  
  
  //  struct spi_device spi;
  int spi_bus_num;
  int spi_cs;
  
  int present_irq;
  int status_irq;
  struct delayed_work  work;
  struct slot_actions* actions;
  
  void* slot_data;  
};


/* BMI Device */

struct bmi_device {
  int devnum;
  
  struct device dev;

  struct mutex        lock;
  
  
  int    present_irq_cnt;  
  int    state;	/* Make this an enum */

  struct bmi_slot *slot;

  struct bmi_eeprom_data *ident;
  unsigned short vendor;
  unsigned short product;
  unsigned short revision;
  
  struct bmi_driver *driver;	/* which driver has allocated this device */

};

#define to_bmi_device(n) container_of(n, struct bmi_device, dev);
#define work_to_slot(w)  container_of(container_of(w,		\
						     struct delayed_work, \
						     work),		\
					struct bmi_slot,		\
					work)


static inline void *bmi_device_get_drvdata (struct bmi_device *bdev)
{
	return dev_get_drvdata (&bdev->dev);
}

static inline void bmi_device_set_drvdata (struct bmi_device *bdev, void *data)
{
	dev_set_drvdata(&bdev->dev, data);
}


/* BMI Driver  */

struct bmi_driver {

	char *name;
	struct bmi_device_id *id_table;
	struct device_driver  driver;
	int  (*probe)(struct bmi_device *dev);
	void (*remove)(struct bmi_device *dev);
	struct dev_pm_ops* pm;
};

extern struct bus_type bmi_bus_type;

#define	to_bmi_driver(drv) container_of(drv,struct bmi_driver, driver)

int __must_check __bmi_register_driver(struct bmi_driver *, struct module *);
static inline int __must_check bmi_register_driver(struct bmi_driver *driver)
{
        return __bmi_register_driver(driver, THIS_MODULE);
}

void bmi_unregister_driver(struct bmi_driver *drv);

struct bmi_device *bmi_alloc_dev(struct bmi_slot *slot);
struct class* bmi_get_class (void);
int bmi_add_slot(struct bmi_slot *slot);
int bmi_del_slot(struct bmi_slot *slot);

#endif

/* USB device counting (perhaps this should be moved */
void increment_usb_dep(void);
void decrement_usb_dep(void);
