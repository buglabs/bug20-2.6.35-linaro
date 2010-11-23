#include <linux/bmi.h>

int __bmi_register_driver(struct bmi_driver *drv, struct module *owner)
{
	int error;

	/* initialize common driver fields */
	drv->driver.name = drv->name;
	drv->driver.bus = &bmi_bus_type;
	drv->driver.owner = owner;
	drv->driver.pm = drv->pm;
	/* register with core */
	error = driver_register(&drv->driver);

	return error;
}


void
bmi_unregister_driver(struct bmi_driver *drv)
{
	driver_unregister(&drv->driver);
}

EXPORT_SYMBOL(__bmi_register_driver);
EXPORT_SYMBOL(bmi_unregister_driver);

