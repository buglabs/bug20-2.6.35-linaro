#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/bmi.h>
#include <linux/bmi/omap_bmi.h>
#include <plat/board.h>

#define IOEXP_START 	212
#define BMI_GPIO_0 	139
#define BMI_GPIO_1 	158
#define BMI_GPIO_2 	137
#define BMI_GPIO_3 	136

static int bl_present(struct bmi_slot* slot)
{
  unsigned gpio = irq_to_gpio(slot->present_irq);
  if (gpio_get_value_cansleep(gpio))
    return 0;
  else
    return 1;
}

static void bl_power_on(struct bmi_slot* slot)
{
  switch(slot->slotnum) {
  case 0:
    //power
    gpio_direction_output(IOEXP_START + 16,1);
    gpio_direction_output(IOEXP_START + 15,0);
    //spi + uart + gpio line buffer
    
    break;
  case 1:
    gpio_direction_output(IOEXP_START + 17,1);
    gpio_direction_output(IOEXP_START + 22,0);
    break;
  case 2:
    gpio_direction_output(IOEXP_START + 18,1);
    gpio_direction_output(IOEXP_START + 6,0);
    break;
  case 3:
    gpio_direction_output(IOEXP_START + 19,1);
    gpio_direction_output(IOEXP_START + 7,0);
    break;
  default:
    break;
  }
  return;
}

static void bl_power_off(struct bmi_slot* slot)
{
  switch(slot->slotnum) {
  case 0:
    gpio_direction_output(IOEXP_START + 16,0);
    gpio_direction_output(IOEXP_START + 15,1);
    break;
  case 1:
    gpio_direction_output(IOEXP_START + 17,0);
    gpio_direction_output(IOEXP_START + 22,1);
    break;
  case 2:
    gpio_direction_output(IOEXP_START + 18,0);
    gpio_direction_output(IOEXP_START + 6,1);
    break;
  case 3:
    gpio_direction_output(IOEXP_START + 19,0);
    gpio_direction_output(IOEXP_START + 7,1);
    break;
  default:
    break;
  }
  return;
}

static void bl_gpio_direction_out(struct bmi_slot* slot, unsigned gpio, int value)	/*Configure gpios as inputs/ouputs*/
{  
  unsigned char *gpios = (unsigned char*) slot->slot_data;
  
  gpio_direction_output(gpios[gpio], value);
  return;
}

static void bl_gpio_direction_in(struct bmi_slot* slot, unsigned gpio)
{
  unsigned char *gpios = (unsigned char*) slot->slot_data;
  
  gpio_direction_input(gpios[gpio]);
  return;
}

static int bl_gpio_get_value(struct bmi_slot* slot, unsigned gpio)
{
  unsigned char *gpios = (unsigned char*) slot->slot_data;
  unsigned int ret = 0;
  
  ret = gpio_get_value_cansleep(gpios[gpio]);
  return ret;
}

static void bl_gpio_set_value(struct bmi_slot* slot, unsigned gpio, int value)
{
  unsigned char *gpios = (u8*) slot->slot_data;

  gpio_set_value_cansleep(gpios[gpio], value);
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
  .gpio_direction_in = bl_gpio_direction_in,
  .gpio_direction_out = bl_gpio_direction_out,
  .gpio_get_value = bl_gpio_get_value,
  .gpio_set_value = bl_gpio_set_value,
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

static int omapbmi_slot_gpio_req(u8 * gpios)
{
	int i, res;
  
	for (i = 0; i < 4; i++) {
		if (gpios[i] < 0)
			break;
		res = gpio_request(gpios[i], "bmi_gpio");
		if (res) {
			printk(KERN_ERR "slots_bug: GPIO %d request err %d\n",
			gpios[i], res);
		} else {
			res =  gpio_direction_output(gpios[i], 0);
			if (res)
				printk(KERN_ERR "slots_bug: GPIO %d set 0 failed %d\n",
				gpios[i], res);
		}
	}
	return 0;
}

void omapbmi_slot_gpio_free(char* gpios)
{
  return;
}

static int omapbmi_slot_probe(struct platform_device *pdev)
{
	struct bmi_slot *slot;
	struct resource *irq_pres, *irq_stat;
  	struct omap_bmi_platform_data* slot_pdata;
  	int ret;

  
  	printk(KERN_INFO "Buglabs 2.0 BUG Slots Driver...\n"); 
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
  
  	//Request slot enable gpios
  	ret = gpio_request(IOEXP_START + 15, "slot0 buf");
  	ret = gpio_request(IOEXP_START + 16, "slot0 en");
  	ret = gpio_request(IOEXP_START + 22, "slot1 buf");
  	ret = gpio_request(IOEXP_START + 17, "slot1 en");
  	ret = gpio_request(IOEXP_START + 6,  "slot2 buf");
  	ret = gpio_request(IOEXP_START + 18, "slot2 en");
  	ret = gpio_request(IOEXP_START + 7,  "slot3 buf");
  	ret = gpio_request(IOEXP_START + 19, "slot3 en");
  	ret = gpio_request(irq_stat->start, "BMI SINT");
  	if (ret) {
    		printk(KERN_ERR "slots_bug: GPIO %d request failed...\n",irq_stat->start);
    		goto err_release;
  	}
	ret = gpio_request(irq_pres->start, "BMI PINT");
	if (ret) {
	  	gpio_free(irq_stat->start);
		printk(KERN_ERR "slots_bug: GPIO %d request failed...\n",irq_pres->start);
    		goto err_release;
  	}
  
	ret = gpio_direction_input(irq_pres->start);
  	gpio_set_debounce(irq_pres->start, 7936);
	slot_pdata = pdev->dev.platform_data;

  	omapbmi_slot_gpio_req(slot_pdata->gpios);
  
  
  	slot->slot_data = (void*)slot_pdata->gpios;
  	slot->present_irq = gpio_to_irq(irq_pres->start);
  	slot->status_irq = gpio_to_irq(irq_stat->start);
  	slot->owner = THIS_MODULE;
  	slot->name = "omap_bug_slot";
  	slot->slotdev.parent = &pdev->dev;
  	slot->actions = &bl_actions;
  	slot->spi_bus_num = 1;
  	slot->spi_cs = slot_pdata->spi_cs;
  	slot->adap = i2c_get_adapter(slot_pdata->i2c_bus_no);
	if (!slot->adap) {
		printk(KERN_ERR "slots_bug: i2c adapter NULL...\n");
		goto gpio_release;
	}
			
  	ret = bmi_add_slot(slot);
  	if (ret) {
    		printk(KERN_ERR "slots_bug: Trouble instantiating slot...%d\n", ret);
    	goto gpio_release;
  	}
  
  	//  disable_irq_nosync(slot->present_irq);
  	schedule_delayed_work(&slot->work, msecs_to_jiffies(1000));
  	return 0;
 gpio_release:
 	gpio_free(irq_stat->start);
 	gpio_free(irq_pres->start);
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
