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

#ifndef MDACC_MD_H
#define MDACC_MD_H

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/bmi.h>
#include <linux/delay.h>

#include <linux/workqueue.h>

struct mon;

struct md
{
	struct cdev cdev;
	struct device *class_dev;
	int open_flag;
	int enabled;
	unsigned char status;
	int ready;
	wait_queue_head_t read_wait_queue;
	struct mon *mon;
        u8 removed;
};


extern int  md_init (void);
extern void md_clean(void);
extern int  md_probe (struct md *md, int slot, struct mon *mon);
extern void md_remove(struct md *md, int slot);


void md_update ( struct md *md, char data);
void md_clear_status (struct md *md );


#endif
