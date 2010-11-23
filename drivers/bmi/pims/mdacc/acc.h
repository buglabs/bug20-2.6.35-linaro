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

#ifndef MDACC_ACC_H
#define MDACC_ACC_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/bmi.h>
#include <linux/delay.h>
#include <linux/bmi/bmi_mdacc.h>
#include "cque.h"

struct mon;

struct acc
{
	struct cdev cdev;
	struct device *class_dev;
	int open_flag;
	struct mon *mon;
	struct mdacc_accel_config cfg;
	struct cque *cque;
	wait_queue_head_t read_wait_queue; 
};


extern int  acc_init (void);
extern void acc_clean(void);
extern int  acc_probe (struct acc *acc, int slot, struct mon *mon); 
extern void acc_remove(struct acc *acc, int slot);




#endif
