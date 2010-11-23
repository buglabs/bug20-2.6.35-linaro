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

#include "cque.h"
#include <linux/slab.h>
#include <linux/byteorder/generic.h>

struct cque *cque_create  (int num_entries, int threshold)
{

	int entry_size;
	struct cque 	*cque;
	void 		*buf_base;
	int 		buf_size;

	entry_size = 6;
	cque = 0;
	buf_size = entry_size * num_entries;
	if (!buf_size) {
		return 0;
	}

	if (( threshold > num_entries) || 
	    ( threshold < 1)) {
		return 0;
	}

	cque = kmalloc (sizeof(cque), GFP_KERNEL);
	if (!cque) {
		return 0;
	}

	buf_base = kmalloc (buf_size, GFP_KERNEL); 
	if (!buf_base) {
		kfree (cque);
		return 0;
	}

	cque->entry_size  = entry_size;
	cque->num_entries = num_entries;
	cque->entry_cnt   = 0;
	cque->threshold   = threshold;
	cque->start       = buf_base;
	cque->end         = buf_base + buf_size - entry_size;
	cque->top         = buf_base; 
	cque->bot         = buf_base; 

	return cque;
}

void cque_destroy (struct cque *cque)
{

	if (cque) {
		kfree (cque->start);
		kfree (cque);
	}
	return;

}

int cque_write (struct cque *cque, void *data)
{
	int size;
	int err;

	size = 6;
	err = 1;
	if (cque) {
		//insert

		memcpy (cque->top, data, size);
		cque->top += size;
		if (cque->top > cque->end) {
			cque->top = cque->start;
		}

		
		if (cque->entry_cnt < cque->num_entries) {
			cque->entry_cnt++;
		}
		else {
			cque->bot += size; 
			if (cque->bot > cque->end) {
				cque->bot = cque->start;
			}
		}
		err = 0;
	}
	return err;	

}

int cque_read (struct cque *cque, void *data )
{
	int size;
	int err;

	size = 6;
	err = 1;
	if (cque) { 

		if (cque->entry_cnt) { 

			//remove

			//memcpy (data, cque->bot, size) with swab ;

			*((short*)(data))   = ntohs (*((short*)(cque->bot)));
			*((short*)(data+2)) = ntohs (*((short*)(cque->bot+2)));
			*((short*)(data+4)) = ntohs (*((short*)(cque->bot+4)));
			cque->bot += size;
			if (cque->bot > cque->end) {
				cque->bot = cque->start;
			}
			cque->entry_cnt--;
			err = 0;
		}
	}
	return err;	
}

int cque_is_ready_for_read (struct cque *cque)
{
	int ready;

	ready = 0;
	if ((cque) && (cque->entry_cnt >= cque->threshold)) { 
		ready = 1;	
	}
	return ready;
}


