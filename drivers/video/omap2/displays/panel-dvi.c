/*
 * Generic panel support
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>

#include <plat/display.h>

static struct omap_video_timings dvi_panel_timings = {
	/* 1280 x 1024 @ 60 Hz */
        /*
  	.x_res		= 1280,
  	.y_res		= 1024,
	.pixel_clock	= 108000,
	.hfp		= 48,
	.hbp		= 248,
	.hsw		= 112,
	.vfp		= 1,
	.vbp		= 38,
	.vsw		= 3,
        */

	/* 1024 x 768 @ 60 Hz */        
  	.x_res		= 1024,
  	.y_res		= 768,
	.pixel_clock	= 65000,
	.hfp		= 24,
	.hbp		= 160,
	.hsw		= 136,
	.vfp		= 3,
	.vbp		= 29,
	.vsw		= 6,

   	/* 800 x 600 @ 60Hz */
	/*
  	.x_res		= 800,
  	.y_res		= 600,
	.pixel_clock	= 40000,
	.hfp		= 40,
	.hbp		= 88,
	.hsw		= 128,
	.vfp		= 1,
	.vbp		= 23,
	.vsw		= 4,
	*/

	/* 640 x 480 @ 60 Hz  Reduced blanking VESA CVT 0.31M3-R */        
	/*
	.x_res          = 640,
	.y_res          = 480,
	.pixel_clock	= 23500,
	.hfp		= 48,
	.hsw		= 32,
	.hbp		= 80,
	.vfp		= 3,
	.vsw		= 4,
	.vbp		= 7,
	*/
};

static int dvi_panel_power_on(struct omap_dss_device *dssdev)
{
	int r;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void dvi_panel_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omapdss_dpi_display_disable(dssdev);
}

static int dvi_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = dvi_panel_timings;

	return 0;
}

static void dvi_panel_remove(struct omap_dss_device *dssdev)
{
}

static int dvi_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = dvi_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void dvi_panel_disable(struct omap_dss_device *dssdev)
{
	dvi_panel_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int dvi_panel_suspend(struct omap_dss_device *dssdev)
{
	dvi_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int dvi_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = dvi_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void dvi_panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void dvi_panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int dvi_panel_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver dvi_driver = {
	.probe		= dvi_panel_probe,
	.remove		= dvi_panel_remove,

	.enable		= dvi_panel_enable,
	.disable	= dvi_panel_disable,
	.suspend	= dvi_panel_suspend,
	.resume		= dvi_panel_resume,

	.set_timings	= dvi_panel_set_timings,
	.get_timings	= dvi_panel_get_timings,
	.check_timings	= dvi_panel_check_timings,

	.driver         = {
		.name   = "dvi_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init dvi_panel_drv_init(void)
{
	return omap_dss_register_driver(&dvi_driver);
}

static void __exit dvi_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&dvi_driver);
}

module_init(dvi_panel_drv_init);
module_exit(dvi_panel_drv_exit);
MODULE_LICENSE("GPL");
