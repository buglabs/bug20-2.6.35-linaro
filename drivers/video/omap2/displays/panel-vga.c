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

static struct omap_video_timings vga_panel_timings = {
	/* 1280 x 1024 @ 57 Hz */        
        /*
  	.x_res		= 1280,
  	.y_res		= 1024,
	.pixel_clock	= 86400,
	.hfp		= 80,
	.hbp		= 48,
	.hsw		= 32,
	.vfp		= 3,
	.vbp		= 18,
	.vsw		= 7,
	//Hsync polarity: +
	//Vsync polarity: +
	*/

	/* 1024 x 768 */
  	.x_res		= 1024,
  	.y_res		= 768,
	.pixel_clock	= 63500,
	.hfp		= 23,
	.hbp		= 159,
	.hsw		= 135,
	.vfp		= 2,
	.vbp		= 28,
	.vsw		= 5,
	//Hsync polarity: -
	//Vsync polarity: -

   	/* 800 x 600 */
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
	//Hsync polarity: +
	//Vsync polarity: +
	*/

	/* 640 x 480 */        
        /*
	.x_res          = 640,
	.y_res          = 480,
	.pixel_clock	= 25170,
	.hfp		= (16-1),
	.hbp		= (48-1),
	.hsw		= (96-1),
	.vfp		= (10-1),
	.vbp		= (33-1),
	.vsw		= (2-1),
	*/
};

static int vga_panel_probe(struct omap_dss_device *dssdev)
{
  //dssdev->panel.config =  OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | 
	                               OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_IPC;
	//dssdev->panel.acb = 32;
	//dssdev->panel.acb = 48;
	//dssdev->panel.recommended_bpp = 24;
	//dssdev->panel.timings = vga_panel_timings;
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = vga_panel_timings;

	return 0;
}

static void vga_panel_remove(struct omap_dss_device *dssdev)
{

}

static int vga_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->platform_enable)
		r = dssdev->platform_enable(dssdev);

	return r;
}

static void vga_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static int vga_panel_suspend(struct omap_dss_device *dssdev)
{
	vga_panel_disable(dssdev);
	return 0;
}

static int vga_panel_resume(struct omap_dss_device *dssdev)
{
	return vga_panel_enable(dssdev);
}

static struct omap_dss_driver vga_driver = {
	.probe		= vga_panel_probe,
	.remove		= vga_panel_remove,

	.enable		= vga_panel_enable,
	.disable	= vga_panel_disable,
	.suspend	= vga_panel_suspend,
	.resume		= vga_panel_resume,

	.driver         = {
		.name   = "vga_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init vga_panel_drv_init(void)
{
	return omap_dss_register_driver(&vga_driver);
}

static void __exit vga_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&vga_driver);
}

module_init(vga_panel_drv_init);
module_exit(vga_panel_drv_exit);
MODULE_LICENSE("GPL");
