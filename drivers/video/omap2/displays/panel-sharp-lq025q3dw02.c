
/*#define DEBUG*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/device.h>

#include <plat/display.h>
#include <plat/dma.h>
#include <plat/gpio.h>

static struct omap_video_timings sharp_spi_timings = {
  .x_res = 320,
  .y_res = 240,
  
  .pixel_clock	= 5000,
  .hsw		= 8, //3
  .hfp		= 4, //3
  .hbp		= 4, //7
  
  .vsw		= 2, //0
  .vfp		= 1, //0
  .vbp		= 1, //1
};


u16 panel_init_seq [] = {
	0x0028,
	0x0100,
	0x0106,

	0x002D,
	0x017F,
	0x0106,

	0x0001,
	0x010A,
	0x01EF,

	0x0002,
	0x0103,
	0x0100,

	0x0003,
	0x010A,
	0x010E,

	0x000B,
	0x01DC,
	0x0100,

	0x000C,
	0x0100,
	0x0105,

	0x000D,
	0x0100,
	0x0102,

	0x000E,
	0x012C,
	0x0100,

	0x000F,
	0x0100,
	0x0100,

	0x0016,
	0x019F,
	0x0188,

	0x0017,
	0x0100,
	0x0102,

	0x001E,
	0x0100,
	0x0100,

	0x0028,
	0x0100,
	0x0106,

	0x002C,
	0x01C8,
	0x018C,

	0x002E,
	0x01B9,
	0x0145,

	0x0030,
	0x0100,
	0x0104,

	0x0031,
	0x0104,
	0x0107,

	0x0032,
	0x0100,
	0x0102,

	0x0033,
	0x0101,
	0x0107,

	0x0034,
	0x0105,
	0x0107,

	0x0035,
	0x0100,
	0x0103,

	0x0036,
	0x0103,
	0x0107,

	0x0037,
	0x0107,
	0x0104,

	0x003A,
	0x011F,
	0x0109,

	0x003B,
	0x0109,
	0x010E,


	0x002D,
	0x017F,
	0x0104,	
};

struct sharp_panel_device {
	struct backlight_device *bl_dev;
	int		enabled;
	unsigned int	saved_bklight_level;

	struct spi_device	*spi;
	struct mutex		mutex;
	struct omap_dss_device	*dssdev;
};


static inline void panel_write(struct spi_device *spi,
			       const u8 *buf, int len)
{
  int r;
	struct spi_transfer t;

	t.bits_per_word = 9;	
	t.tx_buf = buf;
	t.len = len;

	r = spi_write (spi, buf, len);
	if (r < 0)
	{
		printk ("SPI transfer failed for LCD\n");
		return ;
	}
	return ;

}

static int sharp_spi_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT|OMAP_DSS_LCD_IVS|OMAP_DSS_LCD_IHS|OMAP_DSS_LCD_IPC;
	dssdev->panel.acb = 0x0;
	dssdev->panel.timings = sharp_spi_timings;
	//	md->dssdev = dssdev;
	return 0;
}

static void sharp_spi_panel_remove(struct omap_dss_device *dssdev)
{
}

static int sharp_spi_panel_enable(struct omap_dss_device *dssdev)
{
  int r = 0;
  int i;
  u16 data = 0x00;
  int nreset_gpio = dssdev->reset_gpio;
  struct sharp_panel_device *md = dev_get_drvdata(&dssdev->dev);

  //dev_info(&dssdev->dev, "spi_lcd_panel_enable...\n");

  mutex_lock(&md->mutex);

  if (dssdev->platform_enable)
    {
      dev_info(&dssdev->dev, "attempting platform enable...\n");
      r = dssdev->platform_enable(dssdev);
      if (r)
	{
	  dev_warn(&dssdev->dev, "platform enable failed...\n");
	  goto exit;
	}
      md->spi->bits_per_word = 9;
      spi_setup (md->spi);

      mdelay (1);
      if (gpio_is_valid(nreset_gpio))
	//dev_info(&dssdev->dev, "taking lcd out of reset...\n",nreset_gpio);
	gpio_direction_output(93, 1);
	udelay(100);
	gpio_direction_output(90,0);
	udelay(100);
      for (i =0; i<81; i++)
	{
	  data = panel_init_seq[i];
	  panel_write (md->spi, (u8 *)&data, 2);
	}
      mdelay (1);

    }

  msleep(50); // wait for power up

  if (md->enabled) {
    mutex_unlock(&md->mutex);
    return 0;
  }

  md->enabled = 1;

 exit:
  mutex_unlock(&md->mutex);
  return 0;
}

static void sharp_spi_panel_disable(struct omap_dss_device *dssdev)
{
	struct sharp_panel_device *md = dev_get_drvdata(&dssdev->dev);

	dev_info(&dssdev->dev, "sharp_spi_lcd_disable\n");

	mutex_lock(&md->mutex);

	if (!md->enabled) {
		mutex_unlock(&md->mutex);
		return;
	}

	md->enabled = 0;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	mutex_unlock(&md->mutex);
}

static int sharp_spi_panel_suspend (struct omap_dss_device *dssdev)
{
        printk(KERN_INFO "sharp_spi_panel_suspend\n");
	return 0;
}

static int sharp_spi_panel_resume (struct omap_dss_device *dssdev)
{
        printk(KERN_INFO "sharp_spi_panel_resume\n");
	return 0;
}

static int sharp_spi_panel_get_recommended_bpp(struct omap_dss_device *dssdev)
{
	return 16;
}

static struct omap_dss_driver sharp_spi_panel_driver = {
	.probe		= sharp_spi_panel_probe,
	.remove		= sharp_spi_panel_remove,

	.enable		= sharp_spi_panel_enable,
	.disable	= sharp_spi_panel_disable,
	.suspend	= sharp_spi_panel_suspend,
	.resume		= sharp_spi_panel_resume,

	.get_recommended_bpp = sharp_spi_panel_get_recommended_bpp,
	.driver         = {
		.name   = "sharp_spi_panel",
		.owner  = THIS_MODULE,
	},  
};

static int spi_lcd_probe(struct spi_device *spi)
{
  struct omap_dss_device *dssdev = spi->dev.platform_data;
  struct sharp_panel_device *md;

	dev_info(&spi->dev, "spi_lcd_probe\n");

	if (dssdev == NULL) {
		dev_err(&spi->dev, "missing dssdev\n");
		return -ENODEV;
	}

	md = kzalloc(sizeof(*md), GFP_KERNEL);
	if (md == NULL) {
		dev_err(&spi->dev, "out of memory\n");
		return -ENOMEM;
	}
	
	mutex_init(&md->mutex);
	md->spi = spi;
	dev_set_drvdata(&spi->dev, md);
	dev_set_drvdata(&dssdev->dev, md);

	omap_dss_register_driver(&sharp_spi_panel_driver);

	return 0;
}

static int spi_lcd_remove(struct spi_device *spi)
{
	struct sharp_panel_device *md = dev_get_drvdata(&spi->dev);

	dev_info(&spi->dev, "spi_lcd_remove\n");

	omap_dss_unregister_driver(&sharp_spi_panel_driver);

	kfree(md);

	return 0;
}

static struct spi_driver lcd_spi_driver = {
	.driver = {
		.name	= "spi-lcd",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= spi_lcd_probe,
	.remove	= __devexit_p(spi_lcd_remove),
};

static int __init spi_lcd_init(void)
{
	return spi_register_driver(&lcd_spi_driver);
}

static void __exit spi_lcd_exit(void)
{
	spi_unregister_driver(&lcd_spi_driver);
}

module_init(spi_lcd_init);
module_exit(spi_lcd_exit);

MODULE_LICENSE("GPL");
