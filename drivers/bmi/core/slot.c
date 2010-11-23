/* Matt Isaacs - Kick ass platform independant BMI implementation */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/freezer.h>
#include <linux/idr.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>

static DEFINE_MUTEX(slot_lock);
static DEFINE_IDR(bmi_slot_idr);

static LIST_HEAD(slot_event_list);
static DECLARE_WAIT_QUEUE_HEAD(kslotd_wait);

static struct i2c_board_info at24c02_info = {
  I2C_BOARD_INFO("at24c02", 0XA0 >> 1),
};

static void bmi_slot_work_handler(struct work_struct * work);

struct bmi_slot* bmi_get_slot(int slotnum)
{
  struct bmi_slot *slot;

  mutex_lock(&slot_lock);
  slot = (struct bmi_slot*)idr_find(&bmi_slot_idr, slotnum);
  if (slot && !try_module_get(slot->owner))
    slot = NULL;
  
  mutex_unlock(&slot_lock);

  return slot;
}

void bmi_slot_power_on (int num)
{
  struct bmi_slot *slot = bmi_get_slot(num);

  if (!slot) {
    printk(KERN_ERR "BMI: Slot %d doesn't exist...\n", num);
    return;
  }
  
  if (slot->actions->power_on)
    slot->actions->power_on(slot);
  else
    printk(KERN_INFO "BMI: Slot %d power is always on...\n", num);
  return;    
}
		
void bmi_slot_power_off (int num)
{
  struct bmi_slot *slot = bmi_get_slot(num);

  if (!slot) {
    printk(KERN_ERR "BMI: Slot %d doesn't exist...\n", num);
    return;
  }
  
  if (slot->actions->power_off)
    slot->actions->power_off(slot);
  else
    printk(KERN_INFO "BMI: Slot %d power is always on...\n", num);
  return;    
}

void bmi_slot_gpio_direction_out (int num, unsigned gpio, int value)
{
  struct bmi_slot *slot = bmi_get_slot(num);

  if (!slot) {
    printk(KERN_ERR "BMI: Slot %d doesn't exist...\n", num);
    return;
  }
  
  if (slot->actions->gpio_direction_out)
    slot->actions->gpio_direction_out(slot, gpio, value);
  else
    printk(KERN_INFO "BMI: Slot GPIO not configurable...\n");
  return;    

}
EXPORT_SYMBOL(bmi_slot_gpio_direction_out);

void bmi_slot_gpio_direction_in (int num, unsigned gpio)
{
  struct bmi_slot *slot = bmi_get_slot(num);

  if (!slot) {
    printk(KERN_ERR "BMI: Slot %d doesn't exist...\n", num);
    return;
  }
  
  if (slot->actions->gpio_direction_in)
    slot->actions->gpio_direction_in(slot, gpio);
  else
    printk(KERN_INFO "BMI: Slot GPIO not configurable...\n");
  return;    

}
EXPORT_SYMBOL(bmi_slot_gpio_direction_in);

int bmi_slot_gpio_get_value(int num, unsigned gpio)
{
  struct bmi_slot *slot = bmi_get_slot(num);

  if (!slot) {
    printk(KERN_ERR "BMI: Slot %d doesn't exist...\n", num);
    return -ENODEV;
  }
  
  if (slot->actions->gpio_get_value)
    return slot->actions->gpio_get_value(slot, gpio);
  
  printk(KERN_INFO "BMI: Slot GPIO not writable...\n");
  return -EIO;    
}
EXPORT_SYMBOL(bmi_slot_gpio_get_value);

void bmi_slot_gpio_set_value(int num, unsigned gpio, int value)
{
  struct bmi_slot *slot = bmi_get_slot(num);

  if (!slot) {
    printk(KERN_ERR "BMI: Slot %d doesn't exist...\n", num);
    return;
  }
  
  if (slot->actions->gpio_set_value)
    slot->actions->gpio_set_value(slot, gpio, value);
  else
    printk(KERN_INFO "BMI: Slot GPIO not writable...\n");
  return;    
}
EXPORT_SYMBOL(bmi_slot_gpio_set_value);

int bmi_slot_gpio_get_all (int num)
{
  struct bmi_slot *slot = bmi_get_slot(num);
  int i;
  int res = 0;
  // unsigned char *gpio = (unsigned char*) slot->slot_data;

  for (i = 3; i > -1 ; i--)
    { 
      res = (res << 1) | slot->actions->gpio_get_value(slot, i);
    }
  return res;
}

EXPORT_SYMBOL(bmi_slot_gpio_get_all);

// NOTE: When a plug-in module is removed, the gpios should be returned to inputs.
// All requested slot resourece should be released.
// The slot should be powered down.

void bmi_slot_gpio_configure_all_as_inputs (int slot)
{
	return;
}


void bmi_slot_uart_enable  (int num)
{
  struct bmi_slot *slot = bmi_get_slot(num);

  if (!slot) {
    printk(KERN_ERR "BMI: Slot %d doesn't exist...\n", num);
    return;
  }
  
  if (slot->actions->uart_enable)
    return slot->actions->uart_enable(slot);
  
  printk(KERN_INFO "BMI: UART always enabled...\n");
  return;
}
EXPORT_SYMBOL(bmi_slot_uart_enable);

void bmi_slot_uart_disable (int num)
{

	return;
}
EXPORT_SYMBOL(bmi_slot_uart_disable);

void bmi_slot_spi_enable  (int num)
{

	return;
}
EXPORT_SYMBOL(bmi_slot_spi_enable);

void bmi_slot_spi_disable (int num)
{
	return;
}
EXPORT_SYMBOL(bmi_slot_spi_disable);

void bmi_slot_audio_enable  (int num)
{

	return;
}
EXPORT_SYMBOL(bmi_slot_audio_enable);

void bmi_slot_audio_disable (int num)
{

	return;
}
EXPORT_SYMBOL(bmi_slot_audio_disable);

void bmi_slot_battery_enable (int num)
{

	return;
}
EXPORT_SYMBOL(bmi_slot_battery_enable);

void bmi_slot_battery_disable (int num)
{

	return;
}
EXPORT_SYMBOL(bmi_slot_battery_disable);

int bmi_slot_module_present (int num)
{
  struct bmi_slot *slot = bmi_get_slot(num);
  int ret;
  // slot->actions->gpio_set
  if (slot->actions->present != NULL)
    {
      ret = slot->actions->present(slot);
      return ret;
    }
  else
    printk(KERN_INFO "BMI: Slot Driver incomplete. No presence detection...\n");
  return 0;  
}

int bmi_slot_read_eeprom(struct bmi_slot *slot, u8* data)
{
  unsigned char i = 0;
  int ret;

  if (slot->eeprom == NULL) {
	printk(KERN_INFO "Can't get eeprom client...\n");
	ret = -EIO;
  }
  else {
    ret = i2c_master_send(slot->eeprom, &i, 1);
    if (ret == 1)
      ret = i2c_master_recv(slot->eeprom, data, sizeof(struct bmi_eeprom_data));
  }
  return ret;
}

int bmi_slot_status_irq_state (int slot)
{
	int state = 0;
	return state;
}


#define	DEBOUNCE_DELAY	msecs_to_jiffies(1000)

static irqreturn_t bmi_slot_irq_handler(int irq, void *dev_id)
{
  struct bmi_slot *slot = dev_id;
  
  //disable_irq_nosync(irq);
  printk(KERN_INFO " BMI: IRQ Triggered on slot: %d\n", slot->slotnum);
  schedule_delayed_work(&slot->work, DEBOUNCE_DELAY);
  return IRQ_HANDLED;
}

static void bmi_slot_work_handler(struct work_struct * work)
{
  struct bmi_slot *slot;
  struct bmi_device *bdev;
  int ret;
  struct bmi_eeprom_data *data;
  unsigned char* cdat;

  slot = work_to_slot(work);

  mutex_lock(&slot->pres_mutex);
  if (bmi_slot_module_present(slot->slotnum)) {
    if (!slot->present) {
      slot->present = 1;

      bmi_slot_power_on(slot->slotnum);

      slot->eeprom = i2c_new_device(slot->adap, &at24c02_info);
      data = kmalloc(sizeof(struct bmi_eeprom_data), GFP_KERNEL);
      ret = bmi_slot_read_eeprom(slot, (u8*)data);
 
      if (ret < 0)
	{
	  printk(KERN_INFO "BMI: EEPROM Trouble on Slot %d...\n",slot->slotnum);
	  
	  goto del_eeprom;
	}
      //Testing stuff here...get rid of this...
      else
	printk(KERN_INFO "BMI: EEPROM Found...\n");
      cdat = (char*)&data;  
      printk(KERN_INFO "SLOTS: Vendor:  0x%x\n",(data->vendor_msb<<8) | (data->vendor_lsb));
      printk(KERN_INFO "SLOTS: Product  0x%x\n",(data->product_msb<<8) | (data->product_lsb));
      printk(KERN_INFO "SLOTS: Revision 0x%x\n",(data->revision_msb<<8) | (data->revision_lsb));
      
      //Do new device allocation and hand it over to BMI...
      bdev = bmi_alloc_dev(slot);
      bdev->vendor = (data->vendor_msb<<8) | (data->vendor_lsb);
      bdev->product = (data->product_msb<<8) | (data->product_lsb);
      bdev->revision = (data->revision_msb<<8) | (data->revision_lsb);
      bdev->ident = data;
      //Report module plugin so that udev can load appropriate drivers
      ret = device_add(&bdev->dev);
      if (ret) {
	printk(KERN_ERR "SLOTS: Failed to add device...%d\n",ret);
	goto del_eeprom; //TODO: Memory allocated for by bmi_alloc_dev 
      }
      slot->bdev = bdev;
    }
  }
  else { 
    if (slot->present) {
      slot->present = 0;      
      printk(KERN_INFO "BMI: Module removed from slot %d...\n", slot->slotnum);
      if (slot->bdev == NULL) {
	printk(KERN_ERR "SLOTS: BMI Device NULL...\n");
	goto irqenbl;
      }
      //Call BMI device removal stuff here...
      data = slot->bdev->ident;
      slot->bdev->ident = NULL;
      device_del(&slot->bdev->dev);
      goto del_eeprom;
    }
  }
 irqenbl:
  mutex_unlock(&slot->pres_mutex);
  //enable_irq(slot->present_irq);
  return;
 del_eeprom:
  i2c_unregister_device(slot->eeprom);
  kfree(data);
  slot->bdev = NULL;
  slot->eeprom = NULL;
  goto irqenbl;
  
}

static int bmi_register_slot(struct bmi_slot *slot)
{
  int res = 0;
  struct class *class;

  //  mutex_init(&slot->state_lock);
  if (unlikely(WARN_ON(!bmi_bus_type.p)))
    return -EAGAIN;
  if (slot->actions == NULL) {
    printk(KERN_INFO "SLOTS: No Slot actions defined...\n");
    goto unlist;
  }
  mutex_init(&slot->pres_mutex);
  mutex_lock(&slot_lock);
  
  if (slot->slotdev.parent == NULL) {
    slot->slotdev.parent = &platform_bus;
    //debug message here
  }

  dev_set_name(&slot->slotdev, "bmi-%d", slot->slotnum);

  class = bmi_get_class();
  if (class == NULL) {
    printk(KERN_ERR "BMI Class doesn't exist...\n");
    goto unlist;
  } 
  slot->slotdev.class = class;
  res = device_register(&slot->slotdev);
  if (res) {
    printk(KERN_ERR "SLOT: Couldn't register slot... %d\n",res);
    goto unlist;
    //quit
  }

  //Request IRQ
  printk(KERN_ERR "SLOT: Requesting IRQ...\n");
  res = request_irq(slot->present_irq, bmi_slot_irq_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING , slot->name, (void*)slot);

  if (res) {
    printk(KERN_ERR "SLOT: IRQ Request failed...\n");
    goto unlist;
  }

  INIT_DELAYED_WORK(&slot->work, bmi_slot_work_handler);
 unlock:
    mutex_unlock(&slot_lock);
    return res;
 unlist:
    idr_remove(&bmi_slot_idr, slot->slotnum);
    goto unlock;

}


int bmi_add_slot(struct bmi_slot *slot)
{
  int slotnum = 0;
  int res = 0;
  
 retry:
  if (idr_pre_get(&bmi_slot_idr, GFP_KERNEL) == 0)
    return -ENOMEM;
  
  mutex_lock(&slot_lock);
  
  res = idr_get_new_above(&bmi_slot_idr, slot, 0, &slotnum);
  mutex_unlock(&slot_lock);
  if (res < 0) {
    if (res == -EAGAIN)
      goto retry;
    return res;
  }
  slot->slotnum = slotnum;
  return bmi_register_slot(slot);
}
EXPORT_SYMBOL(bmi_add_slot);


int bmi_del_slot(struct bmi_slot *slot)
{
  int res = 0;

  mutex_lock(&slot_lock);
  if (idr_find(&bmi_slot_idr, slot->slotnum)) {
    printk(KERN_ERR "BMI: Attempting to delete unregistered slot...\n");
    res = -EINVAL;
    goto unlock;
  }

  disable_irq_nosync(slot->present_irq);
  free_irq(slot->present_irq, slot);
  device_unregister(&slot->slotdev);  
  idr_remove(&bmi_slot_idr, slot->slotnum);
  memset(&slot->slotdev, 0, sizeof(slot->slotdev));
 
 unlock:
  mutex_unlock(&slot_lock);
  return res;
}
EXPORT_SYMBOL(bmi_del_slot);

