#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/sc16is.h>
#include <asm/gpio.h>


#define SC16IS_DIRECTION 0xA
#define SC16IS_STATE 0XB

struct sc16is_gpio_chip {
  struct sc16is *sc16is;
  unsigned base;
  unsigned char reg_state;
  unsigned char reg_direction;
  struct gpio_chip gpio_chip;
  char **names;
};

static int sc16is_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
  struct sc16is_gpio_chip *chip;
  uint8_t reg_val;
  int ret;

  chip = container_of(gc, struct sc16is_gpio_chip, gpio_chip);

  reg_val = chip->reg_direction & ~(1u << off);
  ret = sc16is_write_reg(chip->sc16is, 0, SC16IS_DIRECTION, reg_val);
  if (ret)
    return ret;

  chip->reg_direction = reg_val;
  return 0;
}

static int sc16is_gpio_direction_output(struct gpio_chip *gc, unsigned off, int val)
{
  struct sc16is_gpio_chip *chip;
  uint8_t reg_val;
  int ret;

  chip = container_of(gc, struct sc16is_gpio_chip, gpio_chip);


  /* Set direction direction */
  reg_val = chip->reg_direction | (1u << off);
  ret = sc16is_write_reg(chip->sc16is, 0, SC16IS_DIRECTION, reg_val);
  if (ret)
    return ret;

  chip->reg_direction = reg_val;

  /* set output level */
  if (val)
    reg_val = chip->reg_state | (1u << off);
  else
    reg_val = chip->reg_state & ~(1u << off);

  ret = sc16is_write_reg(chip->sc16is, 0, SC16IS_STATE, reg_val);
  if (ret)
    return ret;

  chip->reg_state = reg_val;

  return 0;
}

static int sc16is_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
  struct sc16is_gpio_chip *chip;
  uint8_t reg_val;
  int ret;

  chip = container_of(gc, struct sc16is_gpio_chip, gpio_chip);

  ret = sc16is_read_reg(chip->sc16is, 0, SC16IS_STATE, &reg_val);
  if (ret < 0)
    return ret;

  return (reg_val & (1u << off)) ? 1 : 0;
}

static void sc16is_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
  struct sc16is_gpio_chip *chip;
  uint8_t reg_val;
  int ret;

  chip = container_of(gc, struct sc16is_gpio_chip, gpio_chip);

  if (val)
    reg_val = chip->reg_state | (1u << off);
  else
    reg_val = chip->reg_state & ~(1u << off);

  ret = sc16is_write_reg(chip->sc16is, 0, SC16IS_STATE, reg_val);
  if (ret)
    return;

  chip->reg_state = reg_val;
}

static void sc16is_setup_gpio(struct sc16is_gpio_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = sc16is_gpio_direction_input;
	gc->direction_output = sc16is_gpio_direction_output;
	gc->get = sc16is_gpio_get_value;
	gc->set = sc16is_gpio_set_value;
	gc->can_sleep = 1;

	gc->base = chip->base;
	gc->ngpio = gpios;
	gc->label = dev_name(&chip->sc16is->spi_dev->dev);
	gc->dev = &chip->sc16is->spi_dev->dev;
	gc->owner = THIS_MODULE;
	gc->names = chip->names;
}

static int sc16is_gpio_probe(struct platform_device *pdev)
{
  struct sc16is_gpio_platform_data *pdata = pdev->dev.platform_data;
  struct sc16is_gpio_chip *chip;
  int res = 0;
  
  printk(KERN_INFO "SC16IS: GPIO Probe Called...0x%x \n",pdata->gpio_base);
  printk(KERN_INFO "SC16IS: GPIO Setup...0x%x \n",pdata->setup);

  chip = kzalloc(sizeof(struct sc16is_gpio_chip), GFP_KERNEL);
  if (!chip)
    return -ENOMEM;
  chip->sc16is = dev_get_drvdata(pdev->dev.parent);
  chip->base = pdata->gpio_base;
  chip->names = pdata->names;

  sc16is_setup_gpio(chip,8);

  res = sc16is_read_reg(chip->sc16is, 0, SC16IS_STATE, &chip->reg_state);
  if (res)
    goto failed;
  
  res = sc16is_read_reg(chip->sc16is, 0, SC16IS_DIRECTION, &chip->reg_direction);
  if (res)
    goto failed;

  res = gpiochip_add(&chip->gpio_chip);
  if (res) {
    dev_err(&chip->sc16is->spi_dev->dev, "failed adding gpios\n");
    goto failed;
  }
  printk(KERN_INFO "Base 0x%x\n",chip->gpio_chip.base);
  printk(KERN_INFO "ngpio 0x%x\n",chip->gpio_chip.ngpio);
  if (pdata->setup) {
    printk(KERN_INFO "SC16IS GPIO Setup called");
    res = pdata->setup(chip->sc16is->spi_dev, chip->gpio_chip.base, chip->gpio_chip.ngpio, pdata->context);
  }
  return 0;

 failed:
  kfree(chip);
  return res;

}


static int sc16is_gpio_remove(struct platform_device *pdev)
{
  return 0;
}

struct platform_driver sc16is_gpio_driver = {
  .driver.name  = "sc16is-gpio",
  .driver.owner = THIS_MODULE,
  .probe        = sc16is_gpio_probe,
  .remove       = sc16is_gpio_remove,
};

static int __init sc16is_gpio_init(void)
{
  return platform_driver_register(&sc16is_gpio_driver);
}
subsys_initcall(sc16is_gpio_init);

static void __exit sc16is_gpio_exit(void)
{
  platform_driver_unregister(&sc16is_gpio_driver);
}
module_exit(sc16is_gpio_exit);

MODULE_AUTHOR("Buglabs, Inc.");
MODULE_DESCRIPTION("GPIO interface for SC16IS7X");
MODULE_LICENSE("GPL");
