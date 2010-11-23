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
 *         BMI Motion Detector Accelerometer (MDACC) Kernel Module
 *
 * This kernel module contains the device drivers for the Bug MDACC Plug-In
 * Module. Refer to include/linux/bmi/bmi_mdacc.h for user-level device driver
 * programming information.
 *------------------------------------------------------------------------------
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/bmi.h>
#include <linux/delay.h>

#include <linux/workqueue.h>

#define BMI_MDACC_VERSION  "1.0" // driver version


#include "md.h"
#include "acc.h"
#include "ctl.h"
#include "mon.h"
#include "avr.h"
#include "cque.h"
#include "mdacc.h"


// private device structure
struct pim
{
  struct bmi_device *bdev;
  struct ctl ctl;
  struct md  md;
  struct acc acc;
  struct mon mon;
  char *name;
};

static struct pim bug_mdacc_pim[4];

// BMI device ID table
static struct bmi_device_id bmi_mdacc_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_MOT_ACCEL, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },					  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_mdacc_tbl);

int	bmi_mdacc_probe(struct bmi_device *bdev);
void	bmi_mdacc_remove(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_mdacc_driver = 
{
	.name     = "bmi_mdacc", 
	.id_table =  bmi_mdacc_tbl, 
	.probe    =  bmi_mdacc_probe, 
	.remove   =  bmi_mdacc_remove, 
};


// Support functions

int mdacc_get_slot_mon (struct mon *mon)
{
	struct pim *pim;
	int slot;

	pim = container_of(mon, struct pim, mon);

	if (pim->bdev == 0) {
		slot = -1;
	}
	else {
		slot = pim->bdev->slot->slotnum;
	}
	return slot;
}

int mdacc_get_slot_ctl (struct ctl *ctl)
{
	struct pim *pim;
	int slot;

	pim = container_of(ctl, struct pim, ctl);

	if (pim->bdev == 0) {
		slot = -1;
	}
	else {
		slot = pim->bdev->slot->slotnum;
	}
	return slot;
}


struct spi_device* mdacc_get_spi_mon (struct mon *mon)
{
	struct pim *pim;
	struct spi_device *spi = 0;

	/*
	pim = container_of(mon, struct pim, mon);

	if (pim->bdev) {
		spi = pim->bdev->slot->slotnum;
	}
	*/
	return spi;
}

struct bmi_device* mdacc_get_bdev_mon (struct mon *mon)
{
	struct pim *pim;

	pim = container_of(mon, struct pim, mon);
	return pim->bdev;
}

int mdacc_check_bdev_md (struct md *md)
{
	int err;
	struct pim *pim;

	err = 0;
	pim = container_of(md, struct pim, md);
	if (!pim->bdev) {
		err = 1;
	}
	return err;
}

int mdacc_check_bdev_acc (struct acc *acc)
{
	int err;
	struct pim *pim;

	err = 0;
	pim = container_of(acc, struct pim, acc);
	if (!pim->bdev) {
		err = 1;;
	}
	return err;
}

// BMI Functions

int bmi_mdacc_probe(struct bmi_device *bdev)
{	
	int slot;
	struct pim *pim;
	int irq;
	unsigned char tmp = 0;

	// Module GPIO use:
	//					0     	1
	// GPIO 3	Red LED 	 	On	Off
	// GPIO 2	Green LED 		On	Off
	// GPIO 1	AVR Reset		Reset 	Normal Operation
	// GPIO 0	Accel. Sleep Mode	Sleep   Normal Operation 

	slot = bdev->slot->slotnum;
	pim = &bug_mdacc_pim[slot];

	bmi_device_set_drvdata(bdev, pim);
	pim->bdev = bdev;


	// Setup GPIOs for this slot

	bmi_slot_gpio_direction_out(slot, RED_LED, 0);
	bmi_slot_gpio_direction_out(slot, GREEN_LED, 0);
	bmi_slot_gpio_direction_out(slot, GPIO_1, 0);
	bmi_slot_gpio_direction_out(slot, GPIO_0, 0);	 //Red   LED: On	
	bmi_slot_gpio_set_value(slot, GPIO_0, 1);

	bmi_slot_spi_enable(slot);

	//AVR Reset Active time
	mdelay(1);


	//Take AVR out of reset
	bmi_slot_gpio_set_value(slot, GPIO_0, 1);

	//AVR Reset Recovery time

	mdelay (100);


	switch (slot) {
	case 0:
		pim->name = "mdacc_m1";
		break;
	case 1:
		pim->name = "mdacc_m2";
		break;
	case 2:
		pim->name = "mdacc_m3";
		break;
	case 3:
		pim->name = "mdacc_m4";
		break;
	}

	irq = bdev->slot->status_irq;

	if (mon_probe (&pim->mon, pim->name, irq, &pim->md, &pim->acc) ) {
		printk (KERN_ERR "bmi_mdacc_probe() - mon_probe() failed.\n");
		goto exit1;
      	}
	if (md_probe (&pim->md, slot, &pim->mon) ) {
		printk (KERN_ERR "bmi_mdacc_probe() - md_probe() failed.\n");
		goto exit2;
      	}
	if (acc_probe (&pim->acc, slot, &pim->mon) ) {
		printk (KERN_ERR "bmi_mdacc_probe() - acc_probe() failed.\n");
		goto exit3;
      	}
	if (ctl_probe (&pim->ctl, slot) ) {
		printk (KERN_ERR "bmi_mdacc_probe() - ctl_probe() failed.\n");
		goto exit4;
      	}
	bmi_slot_gpio_set_value (slot, RED_LED, 1); //Red + Green LEDs Off
	return 0;

exit4:
	acc_remove (&pim->acc, slot);
exit3:
	md_remove (&pim->md, slot);
exit2:
	mon_remove (&pim->mon);
	
exit1:
	bmi_slot_spi_disable(slot);
	bmi_device_set_drvdata (bdev, 0);
	pim->bdev = 0;
	//	bmi_slot_gpio_write_bit (slot, 2, 1); //Green LED Off
	bmi_slot_gpio_set_value (slot, GREEN_LED, 1);
	return -1;
}

void bmi_mdacc_remove(struct bmi_device *bdev)
{	
	int slot;
	struct pim *pim;

	slot = bdev->slot->slotnum;
	pim = &bug_mdacc_pim[slot];

	ctl_remove (&pim->ctl, slot);
	acc_remove (&pim->acc, slot);
	md_remove (&pim->md, slot);
	mon_remove (&pim->mon);
	
	bmi_slot_spi_disable(slot);

	bmi_device_set_drvdata (bdev, 0);
#if 0
	pim->bdev = 0;
#endif

	return;
}



static __init int bmi_mdacc_init(void)
{	
	int rc = 0;

	acc_init();
	md_init();
	ctl_init();

//	Register with BMI bus.
	rc = bmi_register_driver(&bmi_mdacc_driver); 
	if(rc) {
		printk(KERN_ERR "bmi_mdacc_init() - bmi_register_driver failed\n");
		return rc;
	}

	printk("BMI MDACC Driver v%s \n", BMI_MDACC_VERSION);
	return 0;
}

static void __exit bmi_mdacc_clean(void)
{	
//	Unregister with BMI bus.
	bmi_unregister_driver(&bmi_mdacc_driver);

	ctl_clean();
	md_clean();
	acc_clean();
	return;
}

module_init(bmi_mdacc_init);
module_exit(bmi_mdacc_clean);

MODULE_AUTHOR("EnCADIS Design, Inc.");
MODULE_DESCRIPTION("BMI Motion Detector/Accelerometer Driver");
MODULE_LICENSE("GPL");







