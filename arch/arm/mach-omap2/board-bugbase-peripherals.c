/*
 * linux/arch/arm/mach-omap2/board-bugbase-peripherals.c
 *
 * Copyright (C) 2011 Bug Labs
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi.h>
#include <linux/spi/sc16is.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/pca953x.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mmc/host.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/bmi/omap_bmi.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/leds_pwm.h>

#include <plat/mcspi.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/dma.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#include "mux.h"
#include "hsmmc.h"


#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE		SZ_128K

extern struct platform_device leds_gpio;
extern struct omap_dss_device bugbase_omap_lcd_device;
extern struct platform_device bugbase_omap_dss_device;

static int bug_twl_gpio_setup(struct device *dev,
               unsigned gpio, unsigned ngpio);

static int bug_ioexp_gpio_setup(struct i2c_client *client,
				     unsigned gpio, unsigned ngpio, void *context);
static int bug_ioexp_gpio_teardown(struct i2c_client *client,
				     unsigned gpio, unsigned ngpio, void *context);

static int bug_spi_uart_gpio_setup(struct spi_device *spi,
				     unsigned gpio, unsigned ngpio, void *context);

static struct mtd_partition bug_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data bug_nand_data = {
	.options	= NAND_BUSWIDTH_16,
	.parts		= bug_nand_partitions,
	.nr_parts	= ARRAY_SIZE(bug_nand_partitions),
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.nand_setup	= NULL,
	.dev_ready	= NULL,
};

static struct resource bug_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device bug_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &bug_nand_data,
	},
	.num_resources	= 1,
	.resource	= &bug_nand_resource,
};

static void __init bug_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		bug_nand_data.cs = nandcs;

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (platform_device_register(&bug_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}

/*
 * BMI Slot definitions.
 */

static struct resource bmi_slot1_resources[] = {
	[0] = {
		.start = 16,
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.start = 21,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource bmi_slot2_resources[] = {
	[0] = {
		.start = 14,
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.start = 15,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource bmi_slot3_resources[] = {
	[0] = {
		.start = 22,
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.start = 23,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource bmi_slot4_resources[] = {
	[0] = {
		.start = 12,
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.start = 13,
		.flags = IORESOURCE_IRQ,
	},
};

static struct omap_bmi_platform_data bmi_slot_pdata1 = {
	.gpios = {220, 221, 222, 223},
	.i2c_bus_no = 4,
	.spi_cs = 4,  
};

static struct omap_bmi_platform_data bmi_slot_pdata2 = {
	.gpios = {-1,},
	.i2c_bus_no = 5,
	.spi_cs = -1,  
};

static struct omap_bmi_platform_data bmi_slot_pdata3 = {
	.gpios = {216, 217, 224, 225},
	.i2c_bus_no = 6,
	.spi_cs = 5,  
};

static struct omap_bmi_platform_data bmi_slot_pdata4 = {
	.gpios = {212, 213, 214, 215},
	.i2c_bus_no = 7,

	.spi_cs = 6,  
};

static struct platform_device bmi_slot_devices[] = {
	{
		.name = "omap_bmi_slot",
		.id = 0,
		.num_resources = ARRAY_SIZE(bmi_slot1_resources),
		.resource = bmi_slot1_resources,
		.dev = {
			.platform_data = &bmi_slot_pdata1,
		},
	},
	{
		.name = "omap_bmi_slot",
		.id = 1,
		.num_resources = ARRAY_SIZE(bmi_slot2_resources),
		.resource = bmi_slot2_resources,
		.dev = {
			.platform_data = &bmi_slot_pdata2,
		},
	},
	{
		.name = "omap_bmi_slot",
		.id = 2,
		.num_resources = ARRAY_SIZE(bmi_slot3_resources),
		.resource = bmi_slot3_resources,
		.dev = {
			.platform_data = &bmi_slot_pdata3,
		},
	},
	{
		.name = "omap_bmi_slot",
		.id = 3,
		.num_resources = ARRAY_SIZE(bmi_slot4_resources),
		.resource = bmi_slot4_resources,
		.dev = {
			.platform_data = &bmi_slot_pdata4,
		},
	},    
};


static void omap_init_bmi_slots(void)

{
	int i;

	for (i = 0; i < ARRAY_SIZE(bmi_slot_devices); i++) {
		if (platform_device_register(&bmi_slot_devices[i]) < 0)
			dev_err(&bmi_slot_devices[i].dev,
					"Unable to register BMI slot\n");
	}
}


/*
 *  SPI Peripherals
 */

static int bug_spi_uart_gpio_setup(struct spi_device *spi, unsigned gpio, unsigned ngpio, void *context)
{
	int r;
  
	printk(KERN_INFO "spi_uart_gpio: Setting up gpios...\n");
	//bug_display_init();
	r =   gpio_request(gpio + 4, "wifi_en");  
	if (r) {
	  printk(KERN_ERR "spi_uart_gpio: failed to get wifi_en...\n");
	  return r;
	}
	gpio_direction_output(gpio+4, 1);

	mdelay(100);
	r =   gpio_request(157, "wifi_rst");
	if (r) {
	  printk(KERN_ERR "spi_uart_gpio: failed to get wifi_rst...\n");
	  return r;
	}
	gpio_direction_output(157, 1);

	r =   gpio_request(156, "bt_rst");
	if (r) {
	  printk(KERN_ERR "spi_uart_gpio: failed to get bt_rst...\n");
	  return r;
	}
	gpio_direction_output(156, 1);

	r =   gpio_request(163, "wifi_wakeup");
	if (r) {
	  printk(KERN_ERR "spi_uart_gpio: failed to get wifi_wakeup...\n");
	  return r;
	}
	gpio_direction_output(163, 0);
	  
	r =   gpio_request(235, "5V_en");
	gpio_direction_output(235,1);
	gpio_free(235);
	mdelay(100);
	gpio_set_value (163, 1);
	gpio_set_value (157, 0);
  
	mdelay(100);
	gpio_set_value (157, 1);
	gpio_set_value (156, 0);
	mdelay(100);
	gpio_set_value (156, 1);

	printk(KERN_INFO "spi_uart_gpio: Freeing gpios...");
	gpio_free(232);
	/*
	gpio_free(156);
	gpio_free(157);
	gpio_free(163);
	*/
	return 0;
}

static struct sc16is_gpio_platform_data bugbase_spi_gpio = {
  .gpio_base	= OMAP_MAX_GPIO_LINES + TWL4030_GPIO_MAX + 18,
  .setup	= bug_spi_uart_gpio_setup,
};


static struct sc16is_uart_platform_data bugbase_spi_uart = {
  .irq_pin = 36,
};

static struct sc16is_platform_data bugbase_sc_data = {
  .gpios = &bugbase_spi_gpio,
  .uarts = &bugbase_spi_uart,
};

static struct spi_board_info __initdata bug_spi_board_info[] = {
  {
    .modalias                   = "sc16is",
    .bus_num                    = 1,
    .chip_select                = 0,
    .mode                       = SPI_MODE_0,
    .max_speed_hz               = 15000000,
    .platform_data              = &bugbase_sc_data,
  },
  {
    .modalias			= "spi-lcd",
    .bus_num			= 3,
    .chip_select		= 0,
    .max_speed_hz		= 1000000,
    .controller_data		= NULL,
    .platform_data 		= &bugbase_omap_lcd_device, //&lcd_mcspi_config,
    .mode			= SPI_MODE_0,
  },
};

/*
 *  SD/MMC
 */

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= 192,
		.gpio_wp	= -EINVAL,
	},
	{
	  	.mmc 		= 2,
		.wires 		= 4,
		.gpio_cd	= 170,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc 		= 3,
		.wires 		= 1,
		.nonremovable	= true,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},

	{}	/* Terminator */
};

/*
 *  Regulators
 */

/* Supply enable for digital video outputs */
static struct regulator_consumer_supply bug_1_8_supplies[] = {
	{
		.supply= "vdds_dsi",
		.dev= &bugbase_omap_dss_device.dev,
	},
	//REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.2"),
};

static struct regulator_init_data bug_fixed_1_8_data = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.boot_on 		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.always_on 		= true,
	},
	.num_consumer_supplies= ARRAY_SIZE(bug_1_8_supplies),
	.consumer_supplies= bug_1_8_supplies,
};

static struct fixed_voltage_config bug_fixed_1_8_pdata = {
	.supply_name   = "V1.8",
	.microvolts    = 1800000,
	.init_data     = &bug_fixed_1_8_data,
	.gpio          = -1,
};

static struct platform_device bug_fixed_1_8 = {
	.name          = "reg-fixed-voltage",
	.id            = 1,
	.dev = {
		.platform_data = &bug_fixed_1_8_pdata,
	},
};

static struct regulator_consumer_supply bug_sd_supplies[] = {
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.1"),
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.2"),
};

static struct regulator_init_data bug_fixed_sd_data = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.boot_on 		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,

		.always_on 		= true,
	},
	.num_consumer_supplies= ARRAY_SIZE(bug_sd_supplies),
	.consumer_supplies= bug_sd_supplies,
};

static struct fixed_voltage_config bug_fixed_sd_pdata = {
	.supply_name   = "SD",
	.microvolts    = 3300000,
	.init_data     = &bug_fixed_sd_data,
	.gpio          = -1,
};

static struct platform_device bug_fixed_sd = {
	.name          = "reg-fixed-voltage",
	.id            = 0,
	.dev = {
		.platform_data = &bug_fixed_sd_pdata,
	},
};

static struct regulator_consumer_supply bug_vmmc1_supply = 
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.0");



static struct regulator_consumer_supply bug_vaux2_supply =
	REGULATOR_SUPPLY("hsusb1", "ehci-omap.0");

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data bug_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	//.num_consumer_supplies	= ARRAY_SIZE(bug_vmmc_supplies),
	//.consumer_supplies	= bug_vmmc_supplies,
	.num_consumer_supplies	= 1,
	//.consumer_supplies	= bug_vmmc_supplies,
	.consumer_supplies	= &bug_vmmc1_supply,
};

static struct regulator_init_data bug_vaux2 = {
	.constraints = {
		.name			= "VHUB",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &bug_vaux2_supply,
};


/*
 *  TPS65930 Setup.
 */

/*
 * PWM LEDs available on TPS.
 */

static struct led_pwm pwm_leds[] =
{
		{
			.name               = "omap3bug:red:wifi",
			.default_trigger    = "phy0radio",
			.pwm_id             = 0,
			.active_low         = true,
			.max_brightness     = LED_FULL,
			.pwm_period_ns      = 330,
		},
		{
			.name               = "omap3bug:green:wifi",
			.default_trigger    = "phy0assoc",
			.pwm_id             = 1,
			.active_low         = true,
			.max_brightness     = LED_FULL,
			.pwm_period_ns      = 330,
		},
};

static struct led_pwm_platform_data pwm_led_info =
{
		.leds = pwm_leds,
		.num_leds = ARRAY_SIZE(pwm_leds),
};

static struct platform_device leds_pwm =
{

                .name = "leds_pwm",
		.id = -1,
		.dev =
		{
                                   .platform_data = &pwm_led_info,
		},
};

static struct platform_device bug_twl_pwm_a = {
  .name = "twl4030_pwm",
  .id = 0,
};

static struct platform_device bug_twl_pwm_b = {
  .name = "twl4030_pwm",
  .id = 1,
};


static int bug_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	gpio_request(gpio + 1, "usb_hub");
	gpio_direction_output(gpio + 1, 1);
	gpio_free(gpio + 1);
	return 0;
}

static struct twl4030_gpio_platform_data bug_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= bug_twl_gpio_setup,
};


static struct twl4030_usb_data bug_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_ins sleep_on_seq[] __initdata = {
/*
 * Turn off everything
 */
	/* {MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0,
			RES_STATE_SLEEP), 4},*/
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, 0, 1,
			RES_STATE_SLEEP), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, 0, 2,
			RES_STATE_SLEEP), 2},
};

static struct twl4030_script sleep_on_script __initdata = {
	.script = sleep_on_seq,
	.size   = ARRAY_SIZE(sleep_on_seq),
	.flags  = TWL4030_SLEEP_SCRIPT,
};

static struct twl4030_ins wakeup_p12_seq[] __initdata = {
/*
 * Reenable everything
 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, 0, 1,
			RES_STATE_ACTIVE), 0x2},

};

static struct twl4030_script wakeup_p12_script __initdata = {
	.script	= wakeup_p12_seq,
	.size	= ARRAY_SIZE(wakeup_p12_seq),
	.flags	= TWL4030_WAKEUP12_SCRIPT,
};

static struct twl4030_ins wakeup_p3_seq[] __initdata = {
/*
 * Reenable everything
 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, 0, 2,
			RES_STATE_ACTIVE), 0x2},

};

static struct twl4030_script wakeup_p3_script __initdata = {
	.script	= wakeup_p3_seq,
	.size	= ARRAY_SIZE(wakeup_p3_seq),
	.flags	= TWL4030_WAKEUP3_SCRIPT,
};

static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset VDD1 regulator.
 * Reset VDD2 regulator.
 * Reset VPLL1 regulator.
 * Enable sysclk output.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, 0, 1, RES_STATE_ACTIVE),
		0x13},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_PP, 0, 3, RES_STATE_WRST), 0x13},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VDD1, RES_STATE_WRST), 0x13},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VDD2, RES_STATE_WRST), 0x13},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VPLL1, RES_STATE_WRST), 0x35},
	{MSG_SINGULAR(DEV_GRP_P3, RES_HFCLKOUT, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wrst_script __initdata = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] __initdata = {
	&wakeup_p12_script,
	&sleep_on_script,
	&wakeup_p3_script,
	&wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] __initdata = {
	{ .resource = RES_VDD1, .devgroup = -1,
	  .type = 1, .type2 = -1, .remap_off = RES_STATE_OFF,
	  .remap_sleep = RES_STATE_OFF
	},
	{ .resource = RES_VDD2, .devgroup = -1,
	  .type = 1, .type2 = -1, .remap_off = RES_STATE_OFF,
	  .remap_sleep = RES_STATE_OFF
	},
	{ .resource = RES_VPLL1, .devgroup = DEV_GRP_P1,
	  .type = 1, .type2 = -1, .remap_off = RES_STATE_OFF,
	  .remap_sleep = RES_STATE_OFF
	},
	{ .resource = RES_VPLL2, .devgroup = DEV_GRP_P1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VAUX1, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VAUX2, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VAUX3, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VAUX4, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VMMC1, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VMMC2, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VDAC, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VSIM, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VINTANA1, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = -1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VINTANA2, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VINTDIG, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = -1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VIO, .devgroup = DEV_GRP_P3,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_CLKEN, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = 1, .type2 = -1 , .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_REGEN, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_NRES_PWRON, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_SYSEN, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_32KCLKOUT, .devgroup = -1,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_RESET, .devgroup = -1,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_Main_Ref, .devgroup = -1,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ 0, 0},
};

static struct twl4030_power_data bugbase_scripts_data __initdata = {
	.scripts        = twl4030_scripts,
	.num = ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

static int bugbase_keymap[] = {
	KEY(0, 0, KEY_VIDEO_PREV),
};

static struct matrix_keymap_data bugbase_keymap_data = {
	.keymap			= bugbase_keymap,
	.keymap_size		= ARRAY_SIZE(bugbase_keymap),
};

static struct twl4030_keypad_data bugbase_kp_data = {
	.keymap_data	= &bugbase_keymap_data,
	.rows		= 1,
	.cols		= 1,
	.rep		= 1,
};

static struct twl4030_codec_audio_data bug_audio_data = {
	.audio_mclk = 26000000,
	.ramp_delay_value = 4,
};

static struct twl4030_codec_data bug_codec_data = {
	.audio_mclk = 26000000,
	.audio = &bug_audio_data,
};

static struct twl4030_platform_data bug_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.power		= &bugbase_scripts_data,
	.keypad		= &bugbase_kp_data,
	.usb		= &bug_usb_data,
	.gpio		= &bug_gpio_data,
	.codec		= &bug_codec_data,
	.vmmc1		= &bug_vmmc1,
	.vaux2		= &bug_vaux2,
};

/*
 *  I2C Setup.
 */
static int bug_ioexp_gpio_setup(struct i2c_client *client,
				     unsigned gpio, unsigned ngpio, void *context)
{
  int r;
  r =   gpio_request(gpio + 14, "lt_en");
  if (r) {
    printk(KERN_ERR "ioexp_gpio: failed to get lt_en...\n");
    return -1;
  }
  gpio_direction_output(gpio+14, 0);
  gpio_free(gpio + 14);
  return 0;
}

static int bug_ioexp_gpio_teardown(struct i2c_client *client,
				     unsigned gpio, unsigned ngpio, void *context)
{
  int r;
  r =  gpio_direction_output(gpio+14, 1);
  if (r) {
    printk(KERN_ERR "ioexp_gpio: failed to reset lt_en...\n");
    return -1;
  }
  gpio_free(gpio+14);
  return 0;
}

static struct pca953x_platform_data bug_ioexp_data = {
  .gpio_base	= OMAP_MAX_GPIO_LINES + TWL4030_GPIO_MAX + 2,
  .setup	= bug_ioexp_gpio_setup,
  .teardown	= bug_ioexp_gpio_teardown,
};

static struct i2c_board_info __initdata bug_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &bug_twldata,
	},
};

static struct i2c_board_info __initdata bug_i2c2_boardinfo[] = {
	{
	  I2C_BOARD_INFO("pca9555", 0x20),
	  //.irq = gpio_to_irq(63),
	  .platform_data = &bug_ioexp_data,
	},
	{
	  I2C_BOARD_INFO("bq27500", 0x55),
	},
};

static struct i2c_board_info __initdata bug_i2c3_boardinfo[] = {
	{
	  I2C_BOARD_INFO("pca9546",  0x70),
	},
};

static int __init bugbase_omap_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, bug_i2c_boardinfo,
			ARRAY_SIZE(bug_i2c_boardinfo));
	omap_register_i2c_bus(2, 100, bug_i2c2_boardinfo,
			ARRAY_SIZE(bug_i2c2_boardinfo));
	omap_register_i2c_bus(3, 100, bug_i2c3_boardinfo, 
			ARRAY_SIZE(bug_i2c3_boardinfo));

	return 0;
}

/*
 *  Platform Devices
 */


static struct platform_device *bugbase_peripheral_devices[] __initdata = {
	&bug_fixed_sd,
	&bug_fixed_1_8,
	&bugbase_omap_dss_device,
	&bug_twl_pwm_a,
	&bug_twl_pwm_b,
	&leds_pwm,
	&leds_gpio,
};

/*
 *  Init
 */

void __init bugbase_peripherals_init(void)
{
	bugbase_omap_i2c_init();
	spi_register_board_info(bug_spi_board_info,
				ARRAY_SIZE(bug_spi_board_info));
	platform_add_devices(bugbase_peripheral_devices,
			ARRAY_SIZE(bugbase_peripheral_devices));				
	bug_flash_init();
	omap2_hsmmc_init(mmc);
	omap_init_bmi_slots();
}

