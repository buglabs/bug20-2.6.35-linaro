#include <linux/bmi.h>
#include <linux/pm.h>

static int bmi_pm_prepare(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int error = 0;

	printk(KERN_INFO "BMI: Bus prepare..\n");
	if (drv && drv->pm && drv->pm->prepare)
		error = drv->pm->prepare(dev);

	return error;
}

static void bmi_pm_complete(struct device *dev)
{
	struct device_driver *drv = dev->driver;

	printk(KERN_INFO "BMI: Bus complete..\n");
	if (drv && drv->pm && drv->pm->complete)
		drv->pm->complete(dev);
}

static int bmi_pm_suspend (struct device * dev) 
{
	struct dev_pm_ops *pm;
	struct bmi_device *bmi_dev;

	pm = dev->driver ? dev->driver->pm : NULL;
	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "BMI: Bus suspend..\n");
	if (pm && pm->suspend) {
		int error;

		error = pm->suspend(dev);
		suspend_report_result(pm->suspend, error);
		if (error)
			return error;
	}

	bmi_slot_power_off(bmi_dev->slot->slotnum);

	return 0;
}

static int bmi_pm_resume (struct device * dev) 
{	
	struct dev_pm_ops *pm;
	struct bmi_device *bmi_dev;
	int error = 0;

	pm = dev->driver ? dev->driver->pm : NULL;
	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "BMI: Bus resume..\n");
	bmi_slot_power_on(bmi_dev->slot->slotnum);

	if (pm && pm->resume) {
		error = pm->resume(dev);
	}
	
	return error;
}

struct dev_pm_ops bmi_dev_pm_ops = {
	.prepare = bmi_pm_prepare,
	.complete = bmi_pm_complete,
	.suspend = bmi_pm_suspend,
	.resume = bmi_pm_resume,
	/*
	.freeze = bmi_pm_freeze,
	.thaw = bmi_pm_thaw,
	.poweroff = bmi_pm_poweroff,
	.restore = bmi_pm_restore,
	.suspend_noirq = bmi_pm_suspend_noirq,
	.resume_noirq = bmi_pm_resume_noirq,
	.freeze_noirq = bmi_pm_freeze_noirq,
	.thaw_noirq = bmi_pm_thaw_noirq,
	.poweroff_noirq = bmi_pm_poweroff_noirq,
	.restore_noirq = bmi_pm_restore_noirq,
	*/
};

