/*
 * 	tfp410.c
 *
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 *	Include files
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <plat/gpio.h>
#include <linux/i2c.h>
#include "tfp410.h"

static int tfp410_write (struct i2c_client *client, unsigned char offset, unsigned char data)
{
    int err = 0;
    unsigned char msg[2];
	
    msg[0] = offset;
    msg[1] = data;
    err = i2c_master_send(client, msg, sizeof(msg));
	
    if (err < 0)
        printk (KERN_ERR "tfp410.c: tfp410_write() - i2c transfer failed...%d\n",err);

    return err;
}

int tfp410_enable(struct i2c_client *client)
{
    int err = 0;

    //bring reset line low
    gpio_direction_output(10, 0);
    gpio_set_value (10, 1);
    mdelay (1);
    gpio_set_value (10, 0);
    mdelay (1);

    //exit PD mode
    err |= tfp410_write(client, 0x08, 0xbd);
    mdelay (1);
    if (err < 0) {
        dev_err(&client->dev, "%s: Error during enable\n", __func__);
        return -EINVAL;
    }

    return 0;
}
EXPORT_SYMBOL(tfp410_enable);

int tfp410_disable(struct i2c_client *client)
{
    int err;

    //issue reset
    gpio_direction_output(10, 0);
    gpio_set_value (10, 1);
    mdelay (1);
    gpio_set_value (10, 0);
    mdelay (1);

    //attempt graceful powerdown
    err = tfp410_write(client, 0x08, 0xbc);
    if (err < 0) {
        dev_err(&client->dev, "%s: TFP410 may already be in RESET or PD mode...\n", __func__);
       return -EINVAL;
    }
    mdelay(1);    

    //hold reset line high
    gpio_direction_output(10, 0);
    gpio_set_value (10, 1);
    mdelay(1);    

    return 0;
}
EXPORT_SYMBOL(tfp410_disable);


int tfp410_init(struct i2c_client *client)
{
    int err = 0;

    //issue reset
    gpio_direction_output(10, 0);
    gpio_set_value (10, 1);
    mdelay (1);
    gpio_set_value (10, 0);
    mdelay (1);

    //init tfp
    err |= tfp410_write(client, 0x08, 0xbd);
    mdelay (1);
    err |= tfp410_write(client, 0x09, 0x98);
    mdelay (1);
    err |= tfp410_write(client, 0x33, 0x30);
    mdelay (1);

    if (err < 0) {
        dev_err(&client->dev, "%s: Error during TFP410 init\n", __func__);
	return -EINVAL;
    }

    return 0;
}
EXPORT_SYMBOL(tfp410_init);

static int tfp410p_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
    //printk (KERN_INFO "tfp410.c: probe...\n");

    return 0;
}

static int tfp410p_remove(struct i2c_client *client)
{
    //printk (KERN_INFO "tfp410.c: remove...\n");

	return 0;
}

static const struct i2c_device_id tfp410p_id[] = {
	{"tfp410p", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tfp410p_id);

static struct i2c_driver tfp410p_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tfp410p",
	},
	.probe		= tfp410p_probe,
	.remove		= tfp410p_remove,
	.id_table	= tfp410p_id,
};

static __init int init_tfp410p(void)
{
	return i2c_add_driver(&tfp410p_driver);
}

static __exit void exit_tfp410p(void)
{
	i2c_del_driver(&tfp410p_driver);
}

module_init(init_tfp410p);
module_exit(exit_tfp410p);

MODULE_LICENSE("GPL");
