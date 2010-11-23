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

#ifndef MDACC_CTL_H
#define MDACC_CTL_H

#include <linux/kernel.h>

#include <linux/fs.h>
#include <linux/cdev.h>


struct ctl
{
	struct cdev cdev;
	struct device *class_dev;
};

extern int  ctl_init (void);
extern void ctl_clean(void);
extern int  ctl_probe (struct ctl *ctl, int slot); 
extern void ctl_remove(struct ctl *ctl, int slot); 


#endif


