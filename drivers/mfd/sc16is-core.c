#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hardirq.h>
#include <linux/mfd/core.h>
#include <linux/spi/spi.h>
#include <linux/spi/sc16is.h>

struct platform_device *gpios;
struct platform_device *uarts;


static void sc16is_complete(void *arg)
{
  printk(KERN_INFO "Complete..\n");
}


int sc16is_write_reg(struct sc16is *sc16is, unsigned char channel, unsigned char address, unsigned char data)
{
  struct spi_message	message;
  struct spi_transfer	x;
  int ret;
  unsigned char buf[2];

  spi_message_init(&message);
 
  message.complete = sc16is_complete;
  message.context = sc16is;
  memset(&x, 0, sizeof x);
  x.len = 2;
  spi_message_add_tail(&x, &message);

  buf[0] = (address << 3) | (channel << 1);
  buf[1] = data;
  
  x.tx_buf = buf;

  if (in_irq() || in_softirq())
    ret = spi_async(sc16is->spi_dev, &message);
  else
    ret = spi_sync(sc16is->spi_dev, &message);
  //ret = spi_write_then_read(sc16is->spi_dev,buf,2,NULL,0);
  
  if (ret < 0)
    dev_err(&sc16is->spi_dev->dev, "failed writing register %d\n", ret);
    
  return ret;

}

EXPORT_SYMBOL(sc16is_write_reg);

int sc16is_read_reg(struct sc16is *sc16is, unsigned char channel, unsigned char address, unsigned char *data)
{
  struct spi_message	message;
  struct spi_transfer	x[2];
  int ret;
  unsigned char buf[2];

  spi_message_init(&message);
 
  message.complete = sc16is_complete;
  message.context = sc16is;
  memset(&x, 0, sizeof x);
  x[0].len = 1;
  spi_message_add_tail(&x[0], &message);
  x[1].len = 1;
  spi_message_add_tail(&x[1], &message);

  buf[0] = 0x80 | (address << 3) | (channel << 1);
  x[0].tx_buf = &buf[0];
  x[1].rx_buf = &buf[1];

  if (in_irq() || in_softirq())
    ret = spi_async(sc16is->spi_dev, &message);
  else
    ret = spi_sync(sc16is->spi_dev, &message);
  //  ret = spi_write_then_read(sc16is->spi_dev,data,1,data,1);

  if (ret < 0) 
    dev_err(&sc16is->spi_dev->dev, "failed reading register\n");
  *data = buf[1];
  return ret;
}

EXPORT_SYMBOL(sc16is_read_reg);

void sc16is_dump_regs(struct sc16is *sc16is)
{
  unsigned char tmp;
  int i;
  int res;

  for (i =0; i <= 0xf; i++)
    {
      res = sc16is_read_reg(sc16is, 0, i, &tmp);
      if (res != 0) {
	printk(KERN_ERR "%d\n",res);
	return;
      }
      printk(KERN_INFO "0x%x : 0x%02x \n", i, tmp);
    }
  return;
}

EXPORT_SYMBOL(sc16is_dump_regs);


static int sc16is_add_subdevice_pdata(struct sc16is *sc16is,
				      const char *name, void *pdata, size_t data_size)
{
  struct mfd_cell cell = {
    .name = name,
    .platform_data = pdata,
    .data_size = data_size,
  };
  return mfd_add_devices(&sc16is->spi_dev->dev, -1, &cell, 1, NULL, 0);
}

static int sc16is_add_subdevice(struct sc16is *sc16is,
				const char *name)
{
  return sc16is_add_subdevice_pdata(sc16is, name, NULL, 0);
}

static int __devinit sc16is_probe(struct spi_device *spi)
{
  struct sc16is *sc16is;
  struct sc16is_platform_data *pdata = spi->dev.platform_data;
  int res = 0;

  spi->mode = SPI_MODE_0;
  spi->bits_per_word = 8;
  spi_setup(spi);


  printk(KERN_INFO "SC16IS: Probe Called...\n");
  if (pdata == NULL) { 
    dev_err(&spi->dev, "No platform data...\n");
    return -EINVAL;
  }

  if (pdata->gpios == NULL) { 
    dev_err(&spi->dev, "No uart platform data...\n");
    return -EINVAL;
  }

  if (pdata->uarts == NULL) { 
    dev_err(&spi->dev, "No uart platform data...\n");
    return -EINVAL;
  }
  sc16is = kzalloc(sizeof(*sc16is), GFP_KERNEL);
  if (!sc16is)
    return -ENOMEM;

  dev_set_drvdata(&spi->dev, sc16is);
  
  sc16is->spi_dev = spi;
  
  mutex_init(&sc16is->lock); //Note to self: have a gander at mc13783_lock function
  res = sc16is_add_subdevice_pdata(sc16is, "sc16is-gpio",
				   pdata->gpios, sizeof(struct sc16is_gpio_platform_data));
  res = sc16is_add_subdevice_pdata(sc16is, "sc16is-uart",
				   pdata->uarts, sizeof(struct sc16is_uart_platform_data));
  return 0;
}

static int __devexit sc16is_remove(struct spi_device *spi)
{
  platform_device_unregister(gpios);
  return 0;
}

static struct spi_driver sc16is_driver = {
	.driver = {
		.name	 = "sc16is",
		.owner	= THIS_MODULE,
	},
	.probe	 = sc16is_probe,
	.remove = __devexit_p(sc16is_remove),
};

static __init int sc16is_init(void)
{
  printk(KERN_INFO "SC16IS Initcall...\n");
	return spi_register_driver(&sc16is_driver);
}
subsys_initcall(sc16is_init);

static __exit void sc16is_exit(void)
{
	spi_unregister_driver(&sc16is_driver);
}
module_exit(sc16is_exit);

MODULE_DESCRIPTION("SC16IS SPI driver");
MODULE_AUTHOR("Matt Isaacs <izzy@buglabs.net>");
MODULE_LICENSE("GPL");
