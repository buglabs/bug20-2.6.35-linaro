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
#ifndef MDACC_CQUE_H
#define MDACC_CQUE_H

struct cque 
{
	int 	entry_size;
	int 	num_entries;
	int 	entry_cnt;
	int 	threshold;
	
	void 	*start;
	void 	*end;
	void 	*top;
	void	*bot;
};

struct cque *cque_create  (int num_entries, int threshold);
void cque_destroy (struct cque *cque);

int cque_write (struct cque *cque, void *data);
int cque_read (struct cque *cque, void *data); 
int cque_is_ready_for_read (struct cque *cque);

#endif
