/*
 * linux/arch/arm/mach-omap2/board-bugbase.c
 *
 * Copyright (C) 2011 Bug Labs
 *
 * Based on mach-omap2/board-omap3beagle.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <linux/i2c/twl.h>
#include <linux/i2c/pca953x.h>


#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/usb.h>
#include <plat/timer-gp.h>

#include "mux.h"
#include "hsmmc.h"
#include "sdram-micron-mt46h32m32lf-6.h"

extern void __init bugbase_peripherals_init(void);

/* DSS */
static void __init omap3_bug_display_init(void)
{
	int r;
	
	r = gpio_request(90,  "lcd_shutdown");
	r |= gpio_request(93,  "lcd_reset");
	r |= gpio_request(10,  "dvi_reset");
	r |= gpio_request(92,  "acc_reset");
	if (r) {
	  printk(KERN_INFO "display_init: gpio request failed...\n");
	}
	return;
}

static int omap3_bug_panel_enable_lcd(struct omap_dss_device *display)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	omap_mux_init_signal("dss_data18.mcspi3_clk", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data19.mcspi3_simo", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data20.gpio_90", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data21.mcspi3_cs0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data22.gpio_92", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data23.gpio_93", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("etk_d1.gpio_15", OMAP_PIN_INPUT_PULLUP);
/*
  	omap_cfg_reg (LCD_MCSPI3_CLK);
	omap_cfg_reg (LCD_MCSPI3_SIMO);
	omap_cfg_reg (LCD_SHUTDOWN);
	omap_cfg_reg (LCD_MCSPI3_CS);
	omap_cfg_reg (ACC_RESET);
	omap_cfg_reg (LCD_TP_RESET);
	omap_cfg_reg (ACC_INT);
*/
	gpio_direction_output(90,1);
	gpio_direction_output(92,1);

	return 0;
}

static void omap3_bug_panel_disable_lcd(struct omap_dss_device *display)
{
	//gpio_direction_output(VIDEO_PIM_SW_ENABLE, 1);

	// Mux these pins to safe mode
	printk(KERN_INFO "%s\n", __FUNCTION__);
	omap_mux_init_signal("dss_data18.safe_mode", 0);
	omap_mux_init_signal("dss_data19.safe_mode", 0);
	omap_mux_init_signal("dss_data20.safe_mode", 0);
	omap_mux_init_signal("dss_data21.safe_mode", 0);
	omap_mux_init_signal("dss_data22.safe_mode", 0);
	omap_mux_init_signal("dss_data23.safe_mode", 0);
	omap_mux_init_signal("etk_d1.gpio_15", 0);
/*
  	omap_cfg_reg (LCD_MCSPI3_CLK);

  	omap_cfg_reg (DSS_D18);
	omap_cfg_reg (DSS_D19);
	omap_cfg_reg (DSS_D20);
	omap_cfg_reg (DSS_D21);
	omap_cfg_reg (DSS_D21);
	omap_cfg_reg (DSS_D22);
	omap_cfg_reg (DSS_D23);
*/
	return;
}


struct omap_dss_device omap3_bug_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "sharp_spi_panel",
	.phy.dpi.data_lines = 18,
	.reset_gpio = 90,
	.platform_enable = omap3_bug_panel_enable_lcd,
	.platform_disable = omap3_bug_panel_disable_lcd,
};

static int omap3_bug_panel_enable_dvi(struct omap_dss_device *display)
{
	omap_mux_init_signal("dss_data18.dss_data18", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data19.dss_data19", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data20.dss_data20", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data21.dss_data21", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data22.dss_data22", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data23.dss_data23", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("etk_d1.gpio_15", 0);
	omap_mux_init_signal("sys_clkout1.gpio_10", OMAP_PIN_OUTPUT);

/*
	gpio_direction_output(VIDEO_PIM_ENABLE, 1);
	gpio_direction_output(VIDEO_PIM_SW_ENABLE, 0);
*/
	return 0;
}

static void omap3_bug_panel_disable_dvi(struct omap_dss_device *display)
{
	//gpio_direction_output(VIDEO_PIM_SW_ENABLE, 1);
	
	// Mux these pins to safe mode
	omap_mux_init_signal("dss_data18.safe_mode", 0);
	omap_mux_init_signal("dss_data19.safe_mode", 0);
	omap_mux_init_signal("dss_data20.safe_mode", 0);
	omap_mux_init_signal("dss_data21.safe_mode", 0);
	omap_mux_init_signal("dss_data22.safe_mode", 0);
	omap_mux_init_signal("dss_data23.safe_mode", 0);
	omap_mux_init_signal("etk_d1.gpio_15", 0);
	return;
}

static struct omap_dss_device omap3_bug_vga_device = {
	.type                = OMAP_DISPLAY_TYPE_DPI,
	.name                = "vga",
	.driver_name         = "vga_panel",
	.phy.dpi.data_lines  = 24,
	.platform_enable     = omap3_bug_panel_enable_dvi,
	.platform_disable    = omap3_bug_panel_disable_dvi,
};

static struct omap_dss_device omap3_bug_dvi_device = {
	.type                = OMAP_DISPLAY_TYPE_DPI,
	.name                = "dvi",
	.driver_name         = "generic_panel",
	.phy.dpi.data_lines  = 24,
	.platform_enable     = omap3_bug_panel_enable_dvi,
	.platform_disable    = omap3_bug_panel_disable_dvi,
};

struct omap_dss_device *omap3_bug_display_devices[] = {
        &omap3_bug_lcd_device,
	&omap3_bug_dvi_device,
	&omap3_bug_vga_device,
};

static struct omap_dss_board_info omap3_bug_dss_data = {
	.num_devices	     = ARRAY_SIZE(omap3_bug_display_devices),
	.devices	     = omap3_bug_display_devices,
	.default_device	     = &omap3_bug_lcd_device,
};

struct platform_device omap3_bug_dss_device = {
	.name	 	     = "omapdss",
	.id		     = -1,
	.dev                 = {
		.platform_data = &omap3_bug_dss_data,
	},
};

static void __init omap3_bug_init_irq(void)
{
	omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
			     mt46h32m32lf6_sdrc_params);
	omap_init_irq();
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(12);
#endif
	omap_gpio_init();
}

void gen_gpio_settings(void)
{
	int r;
	r =   gpio_request(107, "dock_rst");
	if (r) {
		printk(KERN_ERR "gen_gpio: failed to get dock_rst...\n");
		return;
	}
	gpio_direction_output(107, 1);

	r =   gpio_request(42, "spi_uart_rst");
	if (r) {
		printk(KERN_ERR "gen_gpio: failed to get spi_uart_rst...\n");
		return;
	}
	gpio_direction_output(42, 1);


	r =   gpio_request(109, "twl_msecure");
	if (r) {
		printk(KERN_ERR "gen_gpio: failed to get twl_msecure...\n");
		return;
	}
	gpio_direction_output(109, 1);

	r =   gpio_request(35, "mmc1_enable");
	if (r) {
		printk(KERN_ERR "gen_gpio: failed to get mmc1_enable...\n");
		return;
	}
	gpio_direction_output(35, 1);

	r =   gpio_request(108, "audio_mute");
	if (r) {
		printk(KERN_ERR "gen_gpio: failed to get audio_mute...\n");
		return;
	}
	gpio_direction_output(108, 1);

	return;

}

static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 126,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static void __init omap3_bug_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
			
	bugbase_peripherals_init();			
	omap_serial_init();
	omap3_bug_display_init();
	usb_musb_init(&musb_board_data);
	usb_ehci_init(&ehci_pdata);

		/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("csi2_dy0.gpio_113", OMAP_PIN_INPUT);
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);
	gen_gpio_settings();
}

MACHINE_START(BUG20, "OMAP3 BUGBase")
	/* Maintainer: Matt Isaacs - http://buglabs.net */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= omap3_bug_init_irq,
	.init_machine	= omap3_bug_init,
	.timer		= &omap_timer,
MACHINE_END
