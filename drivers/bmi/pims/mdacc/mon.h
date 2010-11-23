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

#ifndef MDACC_MON_H
#define MDACC_MON_H

#include "avr.h"
#include "md.h"
#include "acc.h"

#include <linux/workqueue.h>

struct mon
{
	int irq;
	struct md *md;
	struct acc *acc;
  struct spi_device *spi;
  struct spi_board_info mon_spi_info;
	struct semaphore sem;
	int state;
	struct avr_regs regs;
	struct workqueue_struct *workqueue;
	struct work_struct work_item;
};

struct md;
struct acc;

int  mon_probe (struct mon *mon, const char *name, int irq, struct md *md, struct acc *acc);
void mon_remove (struct mon *mon);

int mon_start_motion (struct mon *mon); 
int mon_stop_motion (struct mon *mon); 

int mon_set_config_accel (struct mon *mon, struct mdacc_accel_config *cfg);
int mon_get_config_accel (struct mon *mon, struct mdacc_accel_config *cfg);
int mon_start_accel (struct mon *mon); 
int mon_stop_accel (struct mon *mon); 

int mon_status_request (struct mon *mon); 


#endif

