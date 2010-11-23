#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/bmi.h>

#include <mach/board.h>

#define BMI_GPIO_0 139
#define BMI_GPIO_1 158
#define BMI_GPIO_2 137
#define BMI_GPIO_3 136

static int bl_present(struct bmi_slot* slot)
{
  unsigned gpio = irq_to_gpio(slot->present_irq);
  if (gpio_get_value(gpio))
    return 0;
  else
    return 1;
}

static void bl_power_on(struct bmi_slot* slot)
{
  return;
}

static void bl_power_off(struct bmi_slot* slot)
{
  return;
}

static void bl_gpio_config(struct bmi_slot* slot, int mask)	/*Configure gpios as inputs/ouputs*/
{
  int i;
  
  unsigned char *gpio = (unsigned char*) slot->slot_data;
  
  for (i = 0; i < 4 ; i++)
    {
      if ((mask >> i) & 0x1)
	gpio_direction_output(gpio[i], 0);
      else
	gpio_direction_input(gpio[i]);
    }
  return;
}

static int bl_gpio_get(struct bmi_slot* slot)
{
  int i;
  unsigned char *gpio = (unsigned char*) slot->slot_data;
  unsigned char ret = 0;
  
  for (i = 3; i > -1 ; i--)
    {
      ret = (ret << 1) | gpio_get_value(gpio[i]);
    }
  
  return ret;
}

static void bl_gpio_set(struct bmi_slot* slot, int mask)
{
  int i;
  unsigned char *gpio = (unsigned char*) slot->slot_data;

  for (i = 0; i < 4 ; i++)
    {
      if ((mask >> i) & 0x1)
	gpio_set_value(gpio[i], 1);
      else
	gpio_set_value(gpio[i], 0);
    }
  return;
}

static void bl_uart_enable(struct bmi_slot* slot)
{
  return;
}

static void bl_uart_disable(struct bmi_slot* slot)
{
  return;
}

static void bl_spi_enable(struct bmi_slot* slot)
{
  return;
}

static void bl_spi_disable(struct bmi_slot* slot)
{
  return;
}

static void bl_audio_enable(struct bmi_slot* slot)
{
  return;
}

static void bl_audio_disable(struct bmi_slot* slot)
{
  return;
}

static void bl_batt_enable(struct bmi_slot* slot)
{
  return;
}

static void bl_batt_disable(struct bmi_slot* slot)
{
  return;
}


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
  
static int omapbmi_slot_suspend(struct platform_device *pdev, pm_message_t state)
{
  return 0;
}

static int omapbmi_slot_resume(struct platform_device *pdev)
{
  return 0;
}

static int omapbmi_slot_probe(struct platform_device *pdev)
{
  struct bmi_slot *slot;
  struct resource *irq_pres, *irq_stat;
  // struct omap_bmi_platform_data *bmi_plat_data = pdev->dev.platform_data;
  int ret = 0;
  unsigned char* gpio;
  
  printk(KERN_INFO "Buglabs BeagleBUG Slots Driver...\n"); 
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
  
  ret = gpio_request(irq_stat->start, "BMI SINT");
  if (ret) {
    printk(KERN_ERR "slots_beagle: GPIO %d request failed...\n",irq_stat->start);
    goto err_release;
  }
  ret = gpio_request(irq_pres->start, "BMI PINT");
  if (ret) {
    printk(KERN_ERR "slots_beagle: GPIO %d request failed...\n",irq_pres->start);
    goto err_release;
  }
  
  ret = gpio_direction_input(irq_pres->start);
  
  gpio = kmalloc(4, GFP_KERNEL);
  gpio_request(139,"BMI_0");
  gpio_request(158,"BMI_1");
  gpio_request(137,"BMI_2");
  gpio_request(136,"BMI_3");
  
  gpio[0] = 139;
  gpio[1] = 158;
  gpio[2] = 137;
  gpio[3] = 136;
  
  slot->slot_data = (void*)gpio;
  slot->present_irq = gpio_to_irq(irq_pres->start);
  slot->status_irq = gpio_to_irq(irq_stat->start);
  slot->owner = THIS_MODULE;
  slot->name = "omap_bug_slot";
  slot->slotdev.parent = &pdev->dev;
  slot->adap = i2c_get_adapter(3);
  slot->actions = &bl_actions;
  slot->spi_bus_num = 3;
  slot->spi_cs = 0;
  
  
  ret = bmi_add_slot(slot);
  if (ret) {
    printk(KERN_ERR "slots_beagle: Trouble instantiating slot...%d\n", ret);
    goto err_release;
  }
  return 0;
 err_release:
  kfree(slot->slot_data);
  kfree(slot);
  return ret;
}

static int omapbmi_slot_remove(struct platform_device *pdev)
{
	struct bmi_slot *slot = platform_get_drvdata(pdev);
	//int id = pdev->id;

	bmi_del_slot(slot);
	platform_set_drvdata(pdev, NULL);
	kfree(slot->slot_data);
	kfree(slot);
	return 0;
}


static struct platform_driver omapbmi_slot_driver = {
  .driver = {
    .name = "omap_bmi_slot",
    .owner = THIS_MODULE,
  },
  .probe = omapbmi_slot_probe,
  .remove = omapbmi_slot_remove,
  .suspend = omapbmi_slot_suspend,
  .resume = omapbmi_slot_resume,
};

static int __init omap_bmi_slot_init(void)
{
  /* Register the device driver structure. */
  return platform_driver_register(&omapbmi_slot_driver);
}

/*!
 * This function is used to cleanup all resources before the driver exits.
 */
static void __exit omap_bmi_slot_exit(void)
{
  platform_driver_unregister(&omapbmi_slot_driver);
}

module_init(omap_bmi_slot_init);
module_exit(omap_bmi_slot_exit);

MODULE_AUTHOR("Matt Isaacs");
MODULE_DESCRIPTION("OMAP BMI Slot Driver");
MODULE_LICENSE("GPL");
