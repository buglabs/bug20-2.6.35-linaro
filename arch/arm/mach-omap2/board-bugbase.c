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


/*
 * On/Off LEDs available on OMAP.
 */
static struct gpio_led gpio_leds[] = {
		{
			.name		    = "omap3bug:green:battery",
			.default_trigger    = "batt-normal",
			.gpio		    = 53,
			.active_low         = true,
			.default_state      = LEDS_GPIO_DEFSTATE_OFF,
		},
		{
			.name		    = "omap3bug:red:battery",
			.default_trigger    = "batt-low",
			.gpio		    = 54,
			.active_low         = true,
			.default_state      = LEDS_GPIO_DEFSTATE_OFF,
		},
};

static struct gpio_led_platform_data gpio_led_info = {
       .leds           = gpio_leds,
       .num_leds       = ARRAY_SIZE(gpio_leds),
};

struct platform_device leds_gpio = {
       .name   = "leds-gpio",
       .id     = -1,
       .dev    = {
               .platform_data  = &gpio_led_info,
       },
};

/*
 * PWM LEDs available on OMAP.
 */

static struct omap_pwm_led_platform_data omap_pwm_led_gpt8 = {
       .name                = "omap3bug:blue:bt",
       .intensity_timer     = 8,
       .blink_timer         = 0,
       .default_trigger     = "hci0",
       //.set_power           = set_power(&omap_pwm_led_gpt92, 0),
};

static struct platform_device bugbase_omap_pwm_gpt8 = {
       .name   = "omap_pwm_led",
       .id     = 0,
       .dev    = {
               .platform_data  = &omap_pwm_led_gpt8,
       },
};

static struct omap_pwm_led_platform_data omap_pwm_led_gpt9 = {
       .name                = "omap3bug:blue:battery",
       .intensity_timer     = 9,
       .blink_timer         = 0,
       .default_trigger     = "none",
       //.set_power           = set_power(&omap_pwm_led_gpt92, 0),
};

static struct platform_device bugbase_omap_pwm_gpt9 = {
       .name   = "omap_pwm_led",
       .id     = 1,
       .dev    = {
               .platform_data  = &omap_pwm_led_gpt9,
       },
};

static struct omap_pwm_led_platform_data omap_pwm_led_gpt10 = {
       .name                = "omap3bug:blue:wifi",
       .intensity_timer     = 10,
       .blink_timer         = 0,
       .default_trigger     = "none",
       //.set_power           = set_power(&omap_pwm_led_gpt92, 0),
};

static struct platform_device bugbase_omap_pwm_gpt10 = {
       .name   = "omap_pwm_led",
       .id     = 2,
       .dev    = {
               .platform_data  = &omap_pwm_led_gpt10,
       },
};

static struct omap_pwm_led_platform_data omap_pwm_led_gpt11 = {
       .name                = "omap3bug:blue:power",
       .intensity_timer     = 11,
       .blink_timer         = 0,
       .default_trigger     = "breathe",
       //.set_power           = set_power(&omap_pwm_led_gpt92, 0),
};

static struct platform_device bugbase_omap_pwm_gpt11 = {
       .name   = "omap_pwm_led",
       .id     = 3,
       .dev    = {
               .platform_data  = &omap_pwm_led_gpt11,
       },
};

/* DSS */
static void __init bugbase_omap_display_init(void)
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

static int bugbase_omap_panel_enable_lcd(struct omap_dss_device *display)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	omap_mux_init_signal("dss_data18.mcspi3_clk", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data19.mcspi3_simo", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data20.gpio_90", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data21.mcspi3_cs0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data22.gpio_92", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data23.gpio_93", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("etk_d1.gpio_15", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sys_clkout1.gpio_10", OMAP_PIN_INPUT_PULLUP);
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

static void bugbase_omap_panel_disable_lcd(struct omap_dss_device *display)
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


struct omap_dss_device bugbase_omap_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "sharp_spi_panel",
	.phy.dpi.data_lines = 18,
	.reset_gpio = 90,
	.platform_enable = bugbase_omap_panel_enable_lcd,
	.platform_disable = bugbase_omap_panel_disable_lcd,
};

static int bugbase_omap_panel_enable_dvi(struct omap_dss_device *display)
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

static void bugbase_omap_panel_disable_dvi(struct omap_dss_device *display)
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

static struct omap_dss_device bugbase_omap_vga_device = {
	.type                = OMAP_DISPLAY_TYPE_DPI,
	.name                = "vga",
	.driver_name         = "vga_panel",
	.phy.dpi.data_lines  = 24,
	.platform_enable     = bugbase_omap_panel_enable_dvi,
	.platform_disable    = bugbase_omap_panel_disable_dvi,
};

static struct omap_dss_device bugbase_omap_dvi_device = {
	.type                = OMAP_DISPLAY_TYPE_DPI,
	.name                = "dvi",
	.driver_name         = "generic_panel",
	.phy.dpi.data_lines  = 24,
	.platform_enable     = bugbase_omap_panel_enable_dvi,
	.platform_disable    = bugbase_omap_panel_disable_dvi,
};

struct omap_dss_device *bugbase_omap_display_devices[] = {
        &bugbase_omap_lcd_device,
	&bugbase_omap_dvi_device,
	&bugbase_omap_vga_device,
};

static struct omap_dss_board_info bugbase_omap_dss_data = {
	.num_devices	     = ARRAY_SIZE(bugbase_omap_display_devices),
	.devices	     = bugbase_omap_display_devices,
	.default_device	     = &bugbase_omap_lcd_device,
};

struct platform_device bugbase_omap_dss_device = {
	.name	 	     = "omapdss",
	.id		     = -1,
	.dev                 = {
		.platform_data = &bugbase_omap_dss_data,
	},
};

static void __init bugbase_omap_init_irq(void)
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

	r =   gpio_request(41, "spi_cs_1");
	if (r) {
		printk(KERN_ERR "gen_gpio: failed to get spi_cs_1...\n");
		return;
	}
	gpio_direction_output(41, 1);

	r =   gpio_request(40, "spi_cs_3");
	if (r) {
		printk(KERN_ERR "gen_gpio: failed to get spi_cs_3...\n");
		return;
	}
	gpio_direction_output(40, 1);

	r =   gpio_request(39, "spi_cs_4");
	if (r) {
		printk(KERN_ERR "gen_gpio: failed to get spi_cs_4...\n");
		return;
	}
	gpio_direction_output(39, 1);

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

static struct platform_device *bugbase_omap_devices[] __initdata = {
	&bugbase_omap_pwm_gpt8,
	&bugbase_omap_pwm_gpt9,
	&bugbase_omap_pwm_gpt10,
	&bugbase_omap_pwm_gpt11,

};

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

static void __init bugbase_omap_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	platform_add_devices(bugbase_omap_devices,
			ARRAY_SIZE(bugbase_omap_devices));
	bugbase_peripherals_init();			
	omap_serial_init();
	bugbase_omap_display_init();
	usb_musb_init(&musb_board_data);
	usb_ehci_init(&ehci_pdata);

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("csi2_dy0.gpio_113", OMAP_PIN_INPUT);
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);
	/* Muxing for base LEDs */
	omap_mux_init_signal("gpmc_ncs7.gpt8_pwm_evt", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_ncs4.gpt9_pwm_evt", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_ncs5.gpt10_pwm_evt", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_ncs6.gpt11_pwm_evt", OMAP_PIN_OUTPUT);
	
	omap_mux_init_gpio(53, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(54, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(64, OMAP_PIN_INPUT_PULLUP);
	gen_gpio_settings();
}

MACHINE_START(BUG20, "OMAP3 BUGBase")
	/* Maintainer: Matt Isaacs - http://buglabs.net */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= bugbase_omap_init_irq,
	.init_machine	= bugbase_omap_init,
	.timer		= &omap_timer,
MACHINE_END
