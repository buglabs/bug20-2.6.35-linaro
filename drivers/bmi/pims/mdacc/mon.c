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

#include "mon.h"
#include "mdacc.h"
#include <linux/interrupt.h>

#include <linux/spi/spi.h>

#define work_to_mon(w)  container_of(w, struct mon, work_item)

enum 
{
	MON_STATE_IDLE,
	MON_STATE_MOTION_ONLY,
	MON_STATE_ACC_ONLY,
	MON_STATE_MOTION_AND_ACC
};

enum 
{
	MON_ACTION_START_ACC,
	MON_ACTION_START_MOTION,
	MON_ACTION_STOP_ACC,
	MON_ACTION_STOP_MOTION
};




/*
   Spi Mode Register
   --------------------------------
   Timer Enable           Bit  4
   ADC Poll Enable        Bit  3
   Motion Detect Enable   Bit  2
   GSEL1:0                Bits 1:0


   SPI Status Register
   ------------------------------------
   Not used                     Bit 7
   Timer Enabled                Bit 6
   Adc Complete                 Bit 5
   Adc Enabled                  Bit 4
   Motion Detect Status         Bit 3
   Motion Detect Latched Status Bit 2
   Motion Detect Delta          Bit 1
   Motion Detect Enabled        Bit 0



*/

static void mon_change_state (struct mon* mon, int action)
{
	switch (mon->state) {

	case MON_STATE_IDLE:

		switch (action) {

		case MON_ACTION_START_MOTION:
			mon->state = MON_STATE_MOTION_ONLY;
			break;

		case MON_ACTION_START_ACC:
			mon->state = MON_STATE_ACC_ONLY;
			break;
		}
		break;

	case MON_STATE_MOTION_ONLY:

		switch (action) {

		case MON_ACTION_STOP_MOTION:
			mon->state = MON_STATE_IDLE;
			break;

		case MON_ACTION_START_ACC:
			mon->state = MON_STATE_MOTION_AND_ACC;
			break;
		}
		break;
	
	case MON_STATE_ACC_ONLY:
	
		switch (action) {

		case MON_ACTION_STOP_ACC:
			mon->state = MON_STATE_IDLE;
			break;

		case MON_ACTION_START_MOTION:
			mon->state = MON_STATE_MOTION_AND_ACC;
			break;
		}
		break;

	case MON_STATE_MOTION_AND_ACC:

		switch (action) {

		case MON_ACTION_STOP_ACC:
			mon->state = MON_STATE_MOTION_ONLY;
			break;

		case MON_ACTION_STOP_MOTION:
			mon->state = MON_STATE_ACC_ONLY;
			break;
		}
		break;
	}
	return;
}

int mon_start_motion (struct mon *mon)
{
	int err = 0;
	struct spi_device *spi;

	spi = mon->spi;
	if (spi) {
	  if (down_interruptible (&mon->sem))
	    return -ERESTARTSYS;
	  mon->regs.mode |= 0x04;
	  avr_write_mode (spi, &mon->regs);
	  mon_change_state (mon, MON_ACTION_START_MOTION);
	  up (&mon->sem);
	}
	else {
	  printk (KERN_ERR "mon_start_motion() - FAIL - spi is 0.\n");
	  err = 1;
	}
	return err ;
}

int mon_stop_motion (struct mon *mon)
{
	int err = 0;
	struct spi_device *spi;

	spi = mon->spi;
	if (spi) {
	  if (down_interruptible (&mon->sem))
	    return -ERESTARTSYS;
	  mon->regs.mode &= ~(0x04);
	  avr_write_mode (spi, &mon->regs);
	  mon_change_state (mon, MON_ACTION_STOP_MOTION);
	  up (&mon->sem);
	}	
	else {
	  printk (KERN_ERR "mon_stop_motion() - FAIL - spi is 0.\n");
	  err = 1;
	}
	return err ;
}


int mon_set_config_accel (struct mon *mon, struct mdacc_accel_config *cfg)
{
	int err = 0;
	unsigned char tmp;
	struct spi_device *spi;

	spi = mon->spi;
	if (cfg && spi) {
	  if (down_interruptible (&mon->sem))
	    return -ERESTARTSYS;
	  if  (cfg->delay_mode) {
	    
	    mon->regs.timer_res = cfg->delay_resolution;
	    mon->regs.timer_msb = cfg->delay >> 8;
	    mon->regs.timer_lsb = cfg->delay;
	    mon->regs.mode |= 0x10;
	  }
	  else {
	    mon->regs.mode &= ~0x10; 
	  }

	  if  (cfg->run) {
	    mon->regs.mode |= 0x08;  
	    mon_change_state (mon, MON_ACTION_START_ACC);
	  }
	  else {
	    mon->regs.mode &= ~0x08;  
	    mon_change_state (mon, MON_ACTION_STOP_ACC);
	    
	  }
	  
	  tmp = mon->regs.mode & 0xFC;
	  tmp |= cfg->sensitivity & 0x03;
	  mon->regs.mode = tmp;
	  
	  if  (cfg->delay_mode) {
	    avr_write_timer_and_mode (spi, &mon->regs);  	
	  }
	  else {
	    avr_write_mode (spi, &mon->regs);  	
	  }
	  up (&mon->sem);
	}
	else {
	  printk (KERN_ERR "mon_set_config_accel() - FAIL - null pointer.\n");
	  err = 1;
	}
	return err ;
}

int mon_get_config_accel (struct mon *mon, struct mdacc_accel_config *cfg)
{
	int err = 0;
	struct spi_device *spi;
	
	spi = mon->spi;
	if (cfg && spi) {
	  if (down_interruptible (&mon->sem))
	    return -ERESTARTSYS;

	  avr_read_timer_and_mode (spi, &mon->regs);  	
	  
	  
	  if (mon->regs.mode & 0x10) {
	    cfg->delay_mode = 1;
	    cfg->delay_resolution = mon->regs.timer_res;
	    cfg->delay = mon->regs.timer_msb << 8 |  mon->regs.timer_lsb;
	  }
	  else {
	    cfg->delay_mode = 0;
	    cfg->delay_resolution = 1;
	    cfg->delay = 5000;
	  }
	  
	  if (mon->regs.mode & 0x08) {
	    cfg->run = 1;
	  }
	  else {
	    cfg->run = 0;
	  }
	  cfg->sensitivity = mon->regs.mode & 0x03;
	  up (&mon->sem);
	}	
	else {	
	  printk (KERN_ERR "mon_get_config_accel() - FAIL - null pointer.\n");
	  err = 1;
	}	
	return err ;
}

int mon_start_accel (struct mon *mon)
{			    
	int err = 0;
	struct spi_device *spi;
	
	spi = mon->spi;
	if (spi) {
	  if (down_interruptible (&mon->sem))
	    return -ERESTARTSYS;
	  mon->regs.mode |= 0x08;  
	  avr_write_mode (spi, &mon->regs);  	
	  mon_change_state (mon, MON_ACTION_START_ACC);
	  up (&mon->sem);
	}
	else {
	  printk (KERN_ERR "mon_start_accel() - FAIL - spi is 0.\n");
	  err = 1;
	}
	return err ;
}


int mon_stop_accel (struct mon *mon)
{
	int err = 0;
	
	struct spi_device *spi;

	spi = mon->spi;
	if (spi) {
	  if (down_interruptible (&mon->sem))
	    return -ERESTARTSYS;

	  mon->regs.mode &= ~0x08;  
	  avr_write_mode (spi, &mon->regs);  	
	  mon_change_state (mon, MON_ACTION_STOP_ACC);
	  up (&mon->sem);
	}
	else {
	  printk (KERN_ERR "mon_stop_accel() - FAIL - spi is 0.\n");
	  err = 1;
	}
	return err ;
}


int mon_status_request (struct mon* mon)
{
	struct spi_device *spi;
	int err = 0;


	spi = mon->spi;
	
	if (!spi) {	
		printk (KERN_ERR "mon_status_request() - FAIL - spi is 0.\n");
		err = 1;
	}
	else {

	  if (down_interruptible (&mon->sem))
	    return -ERESTARTSYS;
		//Read the avr status register

	  switch (mon->state) {


	  case MON_STATE_IDLE:

	    avr_read_status (spi, &mon->regs);

			//update md status
	    md_update (mon->md, mon->regs.status);
	    break;

	  case MON_STATE_MOTION_ONLY:
	
	    avr_read_status (spi, &mon->regs);

	    //update md status
	    md_update (mon->md, mon->regs.status);
	    break;


	  case MON_STATE_ACC_ONLY:

	    avr_read_status_and_adc (spi, &mon->regs);

	    cque_write (mon->acc->cque, &mon->regs.adc0h);
	    if (cque_is_ready_for_read (mon->acc->cque) ) {
	      wake_up_interruptible (&mon->acc->read_wait_queue);
	    }
	    break;

	  case MON_STATE_MOTION_AND_ACC:

	    avr_read_status (spi, &mon->regs);

	    //update md status
	    md_update (mon->md, mon->regs.status);

	    // adc complete status ?
	    if (mon->regs.status & 0x20) {
	      avr_read_adc (spi, &mon->regs); 
	      cque_write (mon->acc->cque, &mon->regs.adc0h);
	      if (cque_is_ready_for_read (mon->acc->cque) ) {
		wake_up_interruptible (&mon->acc->read_wait_queue);
	      }
	    }

	    break;

	  default:
	    printk (KERN_ERR "mon_work_handler() - invalid state.\n");
	
	  }
	  up (&mon->sem);
	}
	return err;
}

// work handler
static void mon_work_handler (struct work_struct * work)
{

	struct mon *mon = work_to_mon(work);

	if ( !mon_status_request(mon) ) {
		enable_irq (mon->irq);
	}
	return;
}

// interrupt handler
static irqreturn_t mon_irq_handler(int irq, void *dummy)
{
	struct mon *mon = dummy;

	disable_irq_nosync(irq);
 	schedule_work (&mon->work_item);
	return IRQ_HANDLED;
}

int mon_probe (struct mon *mon, const char *name, int irq, struct md *md, struct acc *acc)
{
	int err;
	struct bmi_device *bdev;
	unsigned long speed;

	unsigned char mode;
	unsigned char bits_per_word;

	err = 1;
	if (mon) {
		bdev = mdacc_get_bdev_mon (mon);
		speed = 250000;
//		speed = 125000;
		mode = 1;
		bits_per_word = 8;

		strcpy(mon->mon_spi_info.modalias, "bug_mdacc_spi");
		mon->mon_spi_info.max_speed_hz = speed;
		mon->mon_spi_info.bus_num = bdev->slot->spi_bus_num;
		mon->mon_spi_info.chip_select = bdev->slot->spi_cs;
		mon->mon_spi_info.mode = mode;		
		mon->spi = spi_new_device(spi_busnum_to_master(mon->mon_spi_info.bus_num), &mon->mon_spi_info) ;
		if (!mon->spi) {
			printk (KERN_ERR "mon_probe() - bmi_device_spi_setup() failed.\n");
			goto exit;
		}

		mon->irq = irq;
		mon->md = md;
		mon->acc = acc;
		init_MUTEX (&mon->sem);
		mon->state = MON_STATE_IDLE;
		memset (&mon->regs, 0, sizeof (struct avr_regs) );

		mon->workqueue = create_singlethread_workqueue (name);
		if (!mon->workqueue) {
			printk (KERN_ERR "mon_probe() - create_singlethread_workqueue() failed.\n");
			goto exit;
		}
		INIT_WORK(&mon->work_item, mon_work_handler);

		if (request_irq(irq, &mon_irq_handler, 0, name, mon)) { 
			printk (KERN_ERR "mon_probe() - request_irq (irq = %d) failed.\n", irq);
			destroy_workqueue( mon->workqueue );
			goto exit;
		}
		err = 0;
	}
exit:	
	return err;
}


void mon_remove (struct mon *mon)
{
	if (mon) {
		free_irq(mon->irq, mon);
		destroy_workqueue( mon->workqueue );
		spi_unregister_device(mon->spi);
	}
      	return;
}


