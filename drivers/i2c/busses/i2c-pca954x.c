/*
 * i2c-pca954x.c
 *
 * Written by: Frank Edelhaeuser <[EMAIL PROTECTED]>
 *
 * Copyright (C) 2008 Spansion Inc. (Frank Edelhaeuser)
 *
 * This module supports the PCA954x series of I2C multiplexers/switches
 * made by NXP, formerly Philips Semiconductors. This includes the
 * PCA9540, PCA9542, PCA9543, PCA9544, PCA9545, PCA9546, PCA9547,PCA9548
 * and derivatives.
 *
 * These chips are all controlled via the I2C bus itself, and all havea
 * single 8-bit register (normally at 0x70). The upstream "parent" busfans
 * out to two, four, or eight downstream busses or channels; which ofthese
 * are selected is determined by the chip type and register contents.
 * A mux can select only one sub-bus at a time; a switch can select any
 * combination simultaneously. However, here we use switches like amux.
 *
 * Based on:
 *   pca954x.c from Kumar Gala <[EMAIL PROTECTED]>
 * which was based on
 *   pca954x.c from Ken Harrenstien
 *   Copyright (C) 2004 Google, Inc. (Ken Harrenstien)
 * and
 *   i2c-virtual_cb.c from Brian Kuschak <[EMAIL PROTECTED]>
 * and
 *   pca9540.c from Jean Delvare <[EMAIL PROTECTED]>, which was
 *   based on pcf8574.c from the same project by Frodo Looijaard,
 *   Philip Edelbrock, Dan Eaton and Aurelien Jarno.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>


struct i2c_pca954x_chipdef {
  const char *name;
  u8 nchans;
  u8 enable; /* used for muxes only */
  enum muxtype { pca954x_ismux = 0, pca954x_isswi } muxtype;
};

static struct i2c_pca954x_chipdef i2c_pca954x_chips[] = {
  {
    .name = "pca9540",
    .nchans = 2,
    .enable = 0x4,
    .muxtype = pca954x_ismux,
  },
  {
    .name = "pca9542",
    .nchans = 2,
    .enable = 0x4,
    .muxtype = pca954x_ismux,
  },
  {
    .name = "pca9543",
    .nchans = 2,
    .enable = 0x0,
    .muxtype = pca954x_isswi,
  },
  {
    .name = "pca9544",
    .nchans = 4,
    .enable = 0x4,
    .muxtype = pca954x_ismux,
  },
  {
    .name = "pca9545",
    .nchans = 4,
    .enable = 0x0,
    .muxtype = pca954x_isswi,
  },
  {
    .name = "pca9546",
    .nchans = 4,
    .enable = 0x0,
    .muxtype = pca954x_isswi,
  },
  {
    .name = "pca9547",
    .nchans = 8,
    .enable = 0x8,
    .muxtype = pca954x_ismux,
  },
  {
    .name = "pca9548",
    .nchans = 8,
    .enable = 0x0,
    .muxtype = pca954x_isswi,
  },
};


struct i2c_pca954x_data {
  struct i2c_client *client;
  struct i2c_pca954x_chipdef *chip;
  int last_chan;
  int adapter_nr_first;
  struct i2c_adapter adapters[8];
  struct mutex lock;
};


static struct i2c_algorithm i2c_pca954x_algo;

static int i2c_pca954x_select_chan(struct i2c_client *client, int chan)
{
  struct i2c_pca954x_data *data = i2c_get_clientdata(client);
  u8 regval = 0;
  int ret = 0;

  /* We make switches look like muxes */
  if (data->chip->muxtype == pca954x_ismux)
    regval = chan | data->chip->enable;
  else
    regval = 1 << chan;

  /* Only select the channel if it's different from the last channel */
  if (data->last_chan != chan) {
    ret = i2c_smbus_write_byte(client, regval);
    if (ret < 0) {
      dev_err(&client->dev,
	      "cannot write to %s device at address %04X\n",
	      client->name,
	      client->addr);
    }
    data->last_chan = chan;
  }

  return ret;
}


static int i2c_pca954x_master_xfer(struct i2c_adapter *adapter,
				   struct i2c_msg *msgs,
				   int num)
{
  struct i2c_pca954x_data *data = i2c_get_adapdata(adapter);
  struct i2c_adapter *parent;
  int chan;
  int rc;
  
  mutex_lock(&data->lock);
  chan = i2c_adapter_id(adapter) - data->adapter_nr_first;
  rc = i2c_pca954x_select_chan(data->client, chan);
  if (rc >= 0) {
    parent = data->client->adapter;
    rc = parent->algo->master_xfer(parent, msgs, num);
  }
  mutex_unlock(&data->lock);

  return rc;
}


static int i2c_pca954x_smbus_xfer(struct i2c_adapter *adapter, u16
				  addr,
				  unsigned short flags, char read_write,
				  u8 command, int size,
				  union i2c_smbus_data *data)
{
  struct i2c_pca954x_data *dat = i2c_get_adapdata(adapter);
  struct i2c_adapter *parent;
  int chan;
  int rc;
  
  mutex_lock(&dat->lock);
  chan = i2c_adapter_id(adapter) - dat->adapter_nr_first;
  rc = i2c_pca954x_select_chan(dat->client, chan);
  if (rc >= 0) {
    parent = dat->client->adapter;
    rc = parent->algo->smbus_xfer(parent, addr, flags, read_write,
				  command, size, data);
  }
  mutex_unlock(&dat->lock);

  return rc;
}


static int i2c_pca954x_remove(struct i2c_client *client)
{
  struct i2c_pca954x_data *data = i2c_get_clientdata(client);
  int i;
  
  if (data) {
    /* Destroy all virtual I2C bus adapters we created */
    i = 0;
    while (i < ARRAY_SIZE(data->adapters)) {
      if ((i2c_get_adapdata(&data->adapters[i])) &&
	  (i2c_del_adapter(&data->adapters[i]) < 0))
	dev_err(&client->dev,
		"cannot destroy adapter %i\n",
		i);
      i++;
    }

    /* Free client data structure */
    i2c_set_clientdata(client, 0);
    kfree(data);
  }

  return 0;
}


static int __devinit i2c_pca954x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  struct i2c_pca954x_chipdef *chip;
  struct i2c_pca954x_data *data;
  struct i2c_adapter *adapter;
  char *name;
  int rc;
  int i;


  printk(KERN_INFO "PCA9546 Driver Probe...\n");
  /* Make sure parent adapter has I2C byte read/write functionality */
  if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
    dev_err(&client->dev,
	    "i2c bus does not support byte read/write\n");
    return -ENODEV;
  }

  /* Look up chip type in table */
  if (!client->name) {
    dev_err(&client->dev,
	    "chip type must be specified in board info\n");
    return -ENODEV;
  }
  chip = i2c_pca954x_chips;
  while (strncmp(chip->name, client->name, strlen(chip->name))) {
    chip++;
    if (chip >= &i2c_pca954x_chips[ARRAY_SIZE(i2c_pca954x_chips)]) {
      dev_err(&client->dev,
	      "unknown chip type (%s)\n",
	      client->name);
      return -ENODEV;
    }
  }

  /* Allocate client data structure */
  data = kzalloc(sizeof(*data), GFP_KERNEL);
  if (!data) {
    dev_err(&client->dev, "cannot allocate client device data\n");
    return -ENOMEM;
  }
  i2c_set_clientdata(client, data);
  mutex_init(&data->lock);
  data->client    = client;
  data->chip      = chip;
  data->last_chan = -1;

  /* Check if local bus number was specified with i2c_board_info.type */
  name = strchr(client->name, ',');
  if (name) {
    /* Bus number given e.g. "pxa9544,5" use bus 5 through 8 */
    data->adapter_nr_first = (int) simple_strtol(name, NULL, 0);
  } else {
    /* Bus number not given. Start at parent's number + 1 */
    data->adapter_nr_first = i2c_adapter_id(client->adapter) + 1;
  }

  /* Hook transfer proc's, copy other parent's proc */
  i2c_pca954x_algo.functionality = client->adapter->algo->functionality;
  if (client->adapter->algo->master_xfer)
    i2c_pca954x_algo.master_xfer = i2c_pca954x_master_xfer;
  if (client->adapter->algo->smbus_xfer)
    i2c_pca954x_algo.smbus_xfer = i2c_pca954x_smbus_xfer;
  
  /* Register virtual adapters for all local bus segments */
  i = 0;
  rc = 0;
  while (i < chip->nchans) {
    adapter = &data->adapters[i];
    i2c_set_adapdata(adapter, data);
    adapter->owner      = THIS_MODULE;
    adapter->class      = I2C_CLASS_HWMON;
    adapter->algo       = &i2c_pca954x_algo;
    adapter->dev.parent = &client->dev;
    adapter->nr         = data->adapter_nr_first + i;
    strncpy(adapter->name,
	    "PCA954X virtual I2C adapter",
	    sizeof(adapter->name));
    rc = i2c_add_numbered_adapter(adapter);
    //rc = i2c_add_adapter(adapter);
    if (rc == -EBUSY) {
      data->adapter_nr_first++;
      continue;
    }
    
    if (rc < 0) {
      i2c_set_adapdata(adapter, 0);
      dev_err(&client->dev,
	      "cannot create adapter for sub-channel %i, error: %i\n",
	      i, rc);
    }
    i++;
  }

  /* Cleanup on error */
  if (rc < 0)
    i2c_pca954x_remove(client);

  return rc;
}

static const struct i2c_device_id i2c_pca954x_id[] = {
	{ "pca9540", 0 },
	{ "pca9542", 0 },
	{ "pca9543", 0 },
	{ "pca9544", 0 },
	{ "pca9545", 0 },
	{ "pca9546", 0 },
	{ "pca9547", 0 },
	{ "pca9548", 0 },
	{ }
};


static struct i2c_driver i2c_pca954x_driver = {
  .driver = {
    .name = "i2c-pca954x",
  },
  .id_table = i2c_pca954x_id,
  .probe  = i2c_pca954x_probe,
  .remove = i2c_pca954x_remove,
};

static int __init i2c_pca954x_init(void)
{
  return i2c_add_driver(&i2c_pca954x_driver);
}

static void __exit i2c_pca954x_exit(void)
{
  i2c_del_driver(&i2c_pca954x_driver);
}

subsys_initcall(i2c_pca954x_init);
//module_init(i2c_pca954x_init);
module_exit(i2c_pca954x_exit);

MODULE_AUTHOR("Frank Edelhaeuser <[EMAIL PROTECTED]>");
MODULE_DESCRIPTION("PCA954X I2C bus multiplexing");
MODULE_LICENSE("GPL");
