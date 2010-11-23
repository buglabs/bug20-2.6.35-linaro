#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/bmi.h>

#include <mach/mx31bug_cpld.h>
//#include <mach/mx31bug_gpio.h>
#include <mach/board-bugbase.h>

static int bl_present(struct bmi_slot* slot)
{
  int status;

  status = cpld_read_module_present_status(slot->slotnum);
  if (status & 0x04)
    return 1;
  else 
    return 0;
}

static void bl_power_on(struct bmi_slot* slot)
{
  //gpio_power_on_slot (slot->slotnum);
  return;
}

static void bl_power_off(struct bmi_slot* slot)
{
  //gpio_power_off_slot (slot->slotnum);
  return;
}

static void bl_gpio_config(struct bmi_slot* slot, int mask)	/*Configure gpios as inputs/ouputs*/
{
  int i;
  for (i = 0; i < 4; i++) {
    cpld_set_module_gpio_dir(slot->slotnum, i, (mask & 0x1));
    mask = mask >> 1;
  }
  return;
}

static int bl_gpio_get(struct bmi_slot* slot)
{
  return cpld_read_gpio_data_reg(slot->slotnum);
}

static void bl_gpio_set(struct bmi_slot* slot, int mask)
{
  int i;
  for (i = 0; i < 4; i++) {
    cpld_set_module_gpio_data(slot->slotnum, i, (mask & 0x1));
    mask = mask >> 1;
  }
  return;
}

static void bl_uart_enable(struct bmi_slot* slot)
{
  cpld_uart_active(slot->slotnum);
  return;
}

static void bl_uart_disable(struct bmi_slot* slot)
{
  cpld_uart_inactive(slot->slotnum);
  return;
}

static void bl_spi_enable(struct bmi_slot* slot)
{
  //REVIST:
  cpld_spi_active(0);
  return;
}

static void bl_spi_disable(struct bmi_slot* slot)
{
  //REVIST:
  cpld_spi_inactive(0);
  return;
}

static void bl_audio_enable(struct bmi_slot* slot)
{
  cpld_activate_audio_ports();
  return;
}

static void bl_audio_disable(struct bmi_slot* slot)
{
  cpld_inactivate_audio_ports();
  return;
}

static void bl_batt_enable(struct bmi_slot* slot)
{
  cpld_set_module_battery_enable(slot->slotnum);
  return;
}

static void bl_batt_disable(struct bmi_slot* slot)
{
  cpld_set_module_battery_disable(slot->slotnum);
  return;
}

/*
static int mxcbmi_probe(struct platform_device *pdev);
static int mxcbmi_slot_remove(struct platform_device *pdev);
static int mxcbmi_suspend(struct platform_device *pdev, pm_message_t state);
static int mxcbmi_resume(struct platform_device *pdev);
*/

struct slot_actions bl_actions = {
  .present = bl_present,
  .power_on = bl_power_on,
  .power_off = bl_power_off,
  .gpio_config = bl_gpio_config,
  .gpio_get = bl_gpio_get,
  .gpio_set = bl_gpio_set,
  .uart_enable = bl_uart_enable,
  .uart_disable = bl_uart_disable,
  .spi_enable = bl_spi_enable,
  .spi_disable = bl_spi_disable,
  .audio_enable = bl_audio_enable,
  .audio_disable = bl_audio_disable,
  .batt_enable = bl_batt_enable,
  .batt_disable = bl_batt_disable,
};
  
static int mxcbmi_slot_suspend(struct platform_device *pdev, pm_message_t state)
{
  return 0;
}

static int mxcbmi_slot_resume(struct platform_device *pdev)
{
  return 0;
}

static int mxcbmi_slot_probe(struct platform_device *pdev)
{
  struct bmi_slot *slot;
  struct resource *res, *irq_pres, *irq_stat;
  struct mxc_bmi_platform_data *bmi_plat_data = pdev->dev.platform_data;
  int ret = 0;
  
  printk(KERN_INFO "Buglabs BUGBase Slots Driver...\n"); 
  irq_pres = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
  if (!irq_pres) {
    dev_err(&pdev->dev, "No presence irq resource...\n");
    return -ENODEV;
  }
  irq_stat = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
  if (!irq_stat) {
    dev_err(&pdev->dev, "No status irq resource...\n");
    return -ENODEV;
  }

  slot = kzalloc(sizeof(struct bmi_slot), GFP_KERNEL);
  if (!slot) {
    ret = -ENOMEM;
    goto err_release;
  }

  
  slot->present_irq = irq_pres->start;
  slot->status_irq = irq_stat->start;
  slot->owner = THIS_MODULE;
  slot->name = "mxc_bug_slot";
  slot->slotdev.parent = &pdev->dev;
  slot->adap = i2c_get_adapter(2 + pdev->id);
  slot->actions = &bl_actions;
  slot->spi_bus_num = 1;
  slot->spi_cs = pdev->id;
  ret = bmi_add_slot(slot);
  if (ret) {
    printk(KERN_ERR "slots_bug: Trouble instantiating slot...%d\n", ret);
    goto err_release;
  }
  ret = 0;
 err_release:
  return ret;
}

static int mxcbmi_slot_remove(struct platform_device *pdev)
{
	struct bmi_slot *slot = platform_get_drvdata(pdev);
	//int id = pdev->id;

	bmi_del_slot(slot);
	platform_set_drvdata(pdev, NULL);
	kfree(slot);
	return 0;
}


static struct platform_driver mxcbmi_slot_driver = {
  .driver = {
    .name = "mxc_bmi_slot",
    .owner = THIS_MODULE,
  },
  .probe = mxcbmi_slot_probe,
  .remove = mxcbmi_slot_remove,
  .suspend = mxcbmi_slot_suspend,
  .resume = mxcbmi_slot_resume,
};

static int __init mxc_bmi_slot_init(void)
{
  /* Register the device driver structure. */
  return platform_driver_register(&mxcbmi_slot_driver);
}

/*!
 * This function is used to cleanup all resources before the driver exits.
 */
static void __exit mxc_bmi_slot_exit(void)
{
  platform_driver_unregister(&mxcbmi_slot_driver);
}

module_init(mxc_bmi_slot_init);
module_exit(mxc_bmi_slot_exit);

MODULE_AUTHOR("Matt Isaacs");
MODULE_DESCRIPTION("MXC BMI Slot Driver");
MODULE_LICENSE("GPL");
