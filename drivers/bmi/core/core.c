#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/bmi.h>


static DEFINE_MUTEX(core_lock);

static struct class *bmi_class;


struct class* bmi_get_class (void)
{
	return bmi_class;
};
EXPORT_SYMBOL(bmi_get_class);


/**
 * bmi_device_get - increments the reference count of the bmi device structure
 * @dev: the device being referenced
 *
 * Each live reference to a device should be refcounted.
 *
 * Drivers for BMI devices should normally record such references in
 * their probe() methods, when they bind to a device, and release
 * them by calling bmi_dev_put(), in their disconnect() methods.
 *
 * A pointer to the device with the incremented reference counter is returned.
 */
struct bmi_device *bmi_dev_get(struct bmi_device *dev)
{
	if (dev)
		get_device(&dev->dev);
	return dev;
}


/**
 * bmi_device_put - release a use of the bmi device structure
 * @dev: device that's been disconnected
 *
 * Must be called when a user of a device is finished with it.  When the last
 * user of the device calls this function, the memory of the device is freed.
 */
void bmi_dev_put(struct bmi_device *dev)
{
	if (dev)
		put_device(&dev->dev);
}


/**
 * bmi_match_one_id - Tell if a BMI device structure has a matching
 *                        BMI device id structure
 * @id: single BMI device id structure to match
 * @bdev: the BMI device structure to match against
 *
 * Returns the matching bmi_device_id structure or %NULL if there is no match.
 */

static const struct bmi_device_id *bmi_match_one_id(const struct bmi_device_id *id,
						const struct bmi_device *bdev)
{
  if ((id->vendor == bdev->vendor) &&
      (id->product == bdev->product) &&
      ((id->revision == bdev->revision) || (id->revision == BMI_ANY)))
    return id;
  return NULL;
}


/**
 * bmi_match_id - See if a BMI device matches a given bmi_device_id table
 * @ids: array of BMI device id structures to search in
 * @bdev: the BMI device structure to match against.
 *
 * Used by a driver to check whether a BMI device present in the
 * system is in its list of supported devices.  Returns the matching
 * bmi_device_id structure or %NULL if there is no match.
 *
 */


const struct bmi_device_id *bmi_match_id(const struct bmi_device_id *ids,
					 struct bmi_device *bdev)
{
  if (ids) {
    while (ids->vendor)	{
      if (bmi_match_one_id(ids, bdev))
	return ids;
      ids++;
    }
  }
  return NULL;
}

/**
 * bmi_device_match - Tell if a BMI device structure has a matching BMI device id structure
 * @dev: the BMI device structure to match against
 * @drv: the device driver to search for matching PCI device id structures
 * 
 * Used by a driver to check whether a BMI device present in the
 * system is in its list of supported devices. Returns the matching
 * bmi_device_id structure or %NULL if there is no match.
 */


static int bmi_device_match(struct device *dev, struct device_driver *driver)
{
  struct bmi_device *bmi_dev = to_bmi_device(dev);
  struct bmi_driver *bmi_drv = to_bmi_driver(driver);
  const struct bmi_device_id *found_id;
  
  found_id = bmi_match_id(bmi_drv->id_table, bmi_dev);

  if (found_id)
    return 1;
  return 0;
}

/*
 * Uevent Generation for hotplug
 */

static int bmi_device_uevent(struct device *dev, struct kobj_uevent_env *env)
{
  struct bmi_device *bdev = to_bmi_device(dev);

  if (!dev)
    return -ENODEV;
  
  if (add_uevent_var(env, "BMIBUS_SLOT=%01X", bdev->slot->slotnum)) {
    return -ENOMEM;
  }
  if (add_uevent_var(env, "BMIBUS_VENDOR=%04X", bdev->vendor)) {
    return -ENOMEM;
  }
  if (add_uevent_var(env, "BMIBUS_PRODUCT=%04X", bdev->product)) {
    return -ENOMEM;
  }
  if (add_uevent_var(env, "BMIBUS_REV=%04X", bdev->revision)) {
    return -ENOMEM;
  }
  if (add_uevent_var(env, "MODALIAS=bmi:v%04Xp%04Xr%04X",
		     bdev->vendor, bdev->product,
		     bdev->revision)) {		  
    return -ENOMEM;
  }			
  return 0;
}


struct bmi_device *bmi_alloc_dev(struct bmi_slot *slot)
{
  struct bmi_device *dev;

  dev = kzalloc(sizeof(*dev), GFP_KERNEL);
  if (!dev) {
    printk(KERN_ERR "BMI: Couldn't Allocate bmi_device structure...\n");
    return NULL;
  }
  
  device_initialize(&dev->dev);
  dev->dev.bus = &bmi_bus_type;
  dev_set_name(&dev->dev, "bmi-dev-%d",slot->slotnum);
  dev->dev.parent = &slot->slotdev;
  dev->slot = slot;
  
  return dev;
}
    


/**
 * __bmi_probe()
 * @drv: driver to call to check if it wants the BMI device
 * @bmi_dev: BMI device being probed
 * 
 * returns 0 on success, else error.
 * side-effect: bmi_dev->driver is set to drv when drv claims bmi_dev.
 */
static int
__bmi_probe(struct bmi_driver *driver, struct bmi_device *bmi_dev)
{
  int error = 0;

  if (!bmi_dev->driver && driver->probe) {

    error = driver->probe(bmi_dev);
    dev_info(&bmi_dev->dev, "Probe returned: 0x%x\n", error);
    if (error >= 0) {
      // bmi_device -> bmi_driver (bmi-bus level )
      bmi_dev->driver = driver;
      error = 0;
    }
  }
  return error;
}

static int bmi_device_probe (struct device *dev)
{
  int error = 0;
  struct bmi_driver *drv;
  struct bmi_device *bmi_dev;
  
  //By this time, we have already been match()ed against a driver.
  
  // device -> device_driver. (driver-core level)
  
  drv = to_bmi_driver(dev->driver);
  bmi_dev = to_bmi_device(dev);
  
  
  bmi_dev_get(bmi_dev);
  
  error = __bmi_probe(drv, bmi_dev);
  if (error)   
    bmi_dev_put(bmi_dev);
  else
    kobject_uevent(&dev->kobj, KOBJ_ADD);
  
  return error;
}



static int bmi_device_remove (struct device *dev)
{
  struct bmi_device * bmi_dev;
  struct bmi_driver * driver;

  bmi_dev = to_bmi_device(dev);
  driver = bmi_dev->driver;

  printk(KERN_INFO "Device removal called...\n ");
  if (driver) {
    if (driver->remove)
      driver->remove(bmi_dev);
    bmi_dev->driver = NULL;
  }

  kobject_uevent(&dev->kobj, KOBJ_REMOVE);
  bmi_dev_put(bmi_dev);
  return 0;
}

static void bmi_device_shutdown(struct device * dev)
{
  return;
}

static int bmi_device_suspend (struct device * dev, pm_message_t state) 
{
	struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	printk(KERN_INFO "BMI: Bus suspend..\n");

	if (pm->suspend) {
		int error;

		error = pm->suspend(dev);
		suspend_report_result(pm->suspend, error);
		if (error)
			return error;
	}

	return 0;
}

static int bmi_device_resume (struct device * dev)
{
  printk(KERN_INFO "BMI: Bus resume..\n");
  return 0;
}

struct bus_type bmi_bus_type = {
	.name = "bmi",
	.match    = bmi_device_match,
	.uevent   = bmi_device_uevent,
	.probe    = bmi_device_probe,
	.remove	  = bmi_device_remove,
	.shutdown = bmi_device_shutdown,
	.suspend  = bmi_device_suspend,
	.dev_attrs = bmi_dev_attrs,
	.resume = bmi_device_resume,
	.pm	= &bmi_dev_pm_ops,
};

static int __init bmi_init(void)
{
  int ret = 0;

  ret = bus_register(&bmi_bus_type);
  if (ret) {
    printk(KERN_ERR "BMI: (bmi_init) - Bus registration failed...\n");
    return ret;
  }

  //  ret = class_register(&bmi_class);
  bmi_class = class_create(THIS_MODULE, "bmi");
  if (ret) {
    printk(KERN_ERR "BMI: (bmi_init) - Failed to register BMI Class...\n");
    bmi_class = NULL;
    bus_unregister(&bmi_bus_type);
  }
  return ret;  
}

static void __exit bmi_cleanup(void)
{
  bmi_class = NULL;
  bus_unregister(&bmi_bus_type);
}

//subsys_initcall(bmi_init);
module_init(bmi_init);
module_exit(bmi_cleanup);
