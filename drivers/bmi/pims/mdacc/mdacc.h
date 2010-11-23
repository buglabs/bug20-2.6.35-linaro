/*
 * Copyright 2008 EnCADIS Designs, Inc. All Rights Reserved.
 * Copyright 2008 Bug-Labs, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*-----------------------------------------------------------------------------
 *
 *      Part of BMI Motion Detector Accelerometer (MDACC) Kernel Module
 *
 *-----------------------------------------------------------------------------
 */

#ifndef MDACC_H
#define MDACC_H

struct acc;
struct ctl;
struct md;
struct mon;
struct spi_device;
struct bmi_device;

extern int mdacc_get_slot_mon (struct mon *mon);
extern int mdacc_get_slot_ctl (struct ctl *ctl);

extern struct bmi_device* mdacc_get_bdev_mon (struct mon *mon);
extern struct spi_device* mdacc_get_spi_mon (struct mon *mon);

int mdacc_check_bdev_md (struct md *md);
int mdacc_check_bdev_acc (struct acc *acc);

struct mon *mdacc_get_mon_md(struct md *md);
struct mon *mdacc_get_mon_acc(struct acc *acc);

#endif

