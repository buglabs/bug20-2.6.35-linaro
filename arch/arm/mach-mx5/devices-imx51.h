/*
 * Copyright (C) 2010 Jason Wang <jason77.wang at gmail.com>
 *
 * based on mach-mx3/devices-imx35.h which is
 * Copyright (C) 2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig at pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/mx51.h>
#include <mach/devices-common.h>

extern const struct imx_esdhc_imx_data imx51_esdhc_data[] __initconst;
#define imx51_add_esdhc(id, pdata)	\
	imx_add_esdhc(&imx51_esdhc_data[id], pdata)
