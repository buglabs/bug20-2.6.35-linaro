#ifndef _OMAP2_MCSPI_H
#define _OMAP2_MCSPI_H

struct omap2_mcspi_platform_config {
	unsigned short	num_cs;
	unsigned short	num_gpio_cs;
	unsigned short	gpio_cs[];
};

struct omap2_mcspi_device_config {
	unsigned turbo_mode:1;

	/* Do we want one channel enabled at the same time? */
	unsigned single_channel:1;
};

#endif
