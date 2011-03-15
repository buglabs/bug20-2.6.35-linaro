/**
 * ml8953.c - OKI ML8953 Accelerometer driver
 *
 * Copyright (C) 2010 Bug Labs Inc.
 *      Matt Isaacs <izzy@buglabs.net>
 *
 * Based an Analog Devices adxl34x accelerometer drivers.
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/input.h>	/* BUS_I2C */
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>
#include "ml8953.h"

#define ML8953_RANGE	32768

static int ml8953_smbus_read(struct i2c_client *client, unsigned char reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int ml8953_smbus_write(struct i2c_client *client,
			       unsigned char reg, unsigned char val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int ml8953_enable(struct ml8953 *ac)
{
	struct i2c_client *client = ac->client;
	unsigned char data[1];
	int error = 0;
	
	mutex_lock(&ac->mutex);
	if (ac->disabled) {
		*data = 0x0;
		if (ml8953_smbus_write(client, ACC_PAGESEL, *data))
			error = -ENODEV;
		// read device to verify existance
		*data = ml8953_smbus_read(client, ACC_CPURDY);

		// set TMD = 0x300 (~250 ms)
		*data = 0x5;
		if(ml8953_smbus_write(client, ACC_TMDH, *data))
			error = -ENODEV;

		*data = 0x0;
		if(ml8953_smbus_write(client, ACC_TMDL, *data))
			error = -ENODEV;

		// set INTOTM
		*data = 0x00;
		if(ml8953_smbus_write(client, ACC_INTOTM, *data))
			error = -ENODEV;

		// set GxAVE
		*data = 0x0;
		if(ml8953_smbus_write(client, ACC_GAAVE, *data))
			error = -ENODEV;

		// set GDTCT[01]
		*data = 0x00;
		if(ml8953_smbus_write(client, ACC_GDTCT0L, *data))
			error = -ENODEV;

		*data = 0x00;
		if(ml8953_smbus_write(client, ACC_GDTCT0H, *data))
			error = -ENODEV;

		*data = 0x00;
		if(ml8953_smbus_write(client, ACC_GDTCT1L, *data))
			error = -ENODEV;

		*data = 0x00;
		if(ml8953_smbus_write(client, ACC_GDTCT1H, *data))
			error = -ENODEV;

		// set MODE0

		*data = ACC_MODE0_PDOFF | ACC_MODE0_TMPOFF | ACC_MODE0_AGCON | ACC_MODE0_MAUTO | ACC_MODE0_GDET10;
		if(ml8953_smbus_write(client, ACC_MODE0, *data))
			error = -ENODEV;

		// set CFG
		*data = ACC_CFG_REGMD | ACC_CFG_INTLVL;
		if(ml8953_smbus_write(client, ACC_CFG, *data))
			error = -ENODEV;

		// Clear INTRQ
		*data = ml8953_smbus_read(client, ACC_INTRQ);

		// set INTMSK
		*data = 0xFE;
		if(ml8953_smbus_write(client, ACC_INTMSK, *data))
			error = -ENODEV;

		// set CTRL0
		*data = ACC_CTRL0_CGAUTO;
		if(ml8953_smbus_write(client, ACC_CTRL0, *data))
			error = -ENODEV;

		// write PAGESEL
		*data = 0x1;
		if(ml8953_smbus_write(client, ACC_PAGESEL, *data))
			error = -ENODEV;
	}
	if (!error)
		ac->disabled = 0;
	mutex_unlock(&ac->mutex);
	return error;
}

static int ml8953_disable(struct ml8953 *ac)
{
	struct i2c_client *client = ac->client;
	unsigned char data[1];
	int error = 0;

	if (!ac->disabled) {
		*data = 0x0;
		if (ml8953_smbus_write(client, ACC_PAGESEL, *data))
			error = -ENODEV;
		*data = 0x0;
		if(ml8953_smbus_write(client, ACC_INTMSK, *data))
			error = -ENODEV;

		cancel_work_sync(&ac->work);
		ac->disabled = 1;
	}
	return error;
}

static void ml8953_work(struct work_struct *work)
{
	struct ml8953 *ac = container_of(work, struct ml8953, work);
	struct i2c_client *client = ac->client;
	int pitch;
	int roll;
	short gx;
	short gy;
	short gz;
	char data[1];

	// orientation
	// read ROLL
	if (!ac->disabled) {
		*data = ml8953_smbus_read(client, ACC_ROLLH);
		roll = (0x0000 | *data) << 8;

		*data = ml8953_smbus_read(client, ACC_ROLLL);
		roll = roll | *data;
		// read PITCH
		*data = ml8953_smbus_read(client, ACC_PITCHH);
		pitch = (0x0000 | *data) << 8;

		*data = ml8953_smbus_read(client, ACC_PITCHL);
		pitch = pitch | *data;

		*data = ml8953_smbus_read(client, ACC_GAZH);
		ac->sample[0] = *data;
		gz = *data << 8;

		*data = ml8953_smbus_read(client, ACC_GAZL);
		ac->sample[1] = *data;
		gz = gz | *data;

		*data = ml8953_smbus_read(client, ACC_GAYH);
		ac->sample[2] = *data;
		gy = *data << 8;

		*data = ml8953_smbus_read(client, ACC_GAYL);
		ac->sample[3] = *data;
		gy = gy | *data;

		*data = ml8953_smbus_read(client, ACC_GAXH);
		ac->sample[4] = *data;
		gx = *data << 8;

		*data = ml8953_smbus_read(client, ACC_GAXL);
		ac->sample[5] = *data;
		gx = gx | *data;

		mutex_lock(&ac->mutex);
		ac->saved[0] = gx;
		ac->saved[1] = gy;
		ac->saved[2] = gz;
		mutex_unlock(&ac->mutex);
		// read STATUS
		*data = ml8953_smbus_read(client, ACC_STATUS);

		if((*data & 0x1) == 0) {

			// write PAGESEL
			*data = 0x0;
			if(!ml8953_smbus_write(client, ACC_PAGESEL, *data))

				// read INTRQ
				*data = ml8953_smbus_read(client, ACC_INTRQ);
		}

		// write PAGESEL
		*data = 0x1;
		ml8953_smbus_write(client, ACC_PAGESEL, *data);

		// report orientation

		//printk(KERN_DEBUG "ml8953_work: 0x%x\n", (pitch << 16) | roll);
		input_report_abs(ac->input, ABS_MISC, (pitch << 16) | roll);
		//printk(KERN_INFO "ml8953_work: enabling irq %d\n",client->irq);
	}
	input_sync(ac->input);
	msleep(10);
	//enable_irq(client->irq);
}

static irqreturn_t ml8953_irq(int irq, void *handle)
{
	struct ml8953 *ac = handle;

	//printk(KERN_INFO "ml8953_irq: %d\n", irq);
	//disable_irq_nosync(irq);
	schedule_work(&ac->work);

	return IRQ_HANDLED;

}

static int ml8953_input_open(struct input_dev *input)
{
	struct ml8953 *ac = input_get_drvdata(input);

	mutex_lock(&ac->mutex);
	ac->open = 1;
	mutex_unlock(&ac->mutex);

	ml8953_enable(ac);

	return 0;
}

static void ml8953_input_close(struct input_dev *input)
{
	struct ml8953 *ac = input_get_drvdata(input);

	ml8953_disable(ac);

	mutex_lock(&ac->mutex);
	ac->open = 0;
	mutex_unlock(&ac->mutex);
}

static ssize_t ml8953_position_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ml8953 *ac = dev_get_drvdata(dev);
	ssize_t count;

	mutex_lock(&ac->mutex);

	count = sprintf(buf, "(%d, %d, %d)\n",
			ac->saved[0], ac->saved[1], ac->saved[2]);
	mutex_unlock(&ac->mutex);
	return count;
}

static DEVICE_ATTR(position, 0444, ml8953_position_show, NULL);

static ssize_t ml8953_disable_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct ml8953 *ac = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ac->disabled);
}

static ssize_t ml8953_disable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct ml8953 *ac = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if (val)
		ml8953_disable(ac);
	else
		ml8953_enable(ac);

	return count;
}

static DEVICE_ATTR(disable, 0664, ml8953_disable_show, ml8953_disable_store);

static struct attribute *ml8953_attributes[] = {
	&dev_attr_position.attr,
	&dev_attr_disable.attr,
	NULL
};

static const struct attribute_group ml8953_attr_group = {
	.attrs = ml8953_attributes,
};

static int __devinit ml8953_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct ml8953 *ac;
	int error = 0;
	struct input_dev *input_dev;

	if (!client->irq) {
		dev_err(&client->dev, "no IRQ?\n");
		return -ENODEV;
	}

	error = i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA);
	if (!error) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}
	ac= kzalloc(sizeof(struct ml8953), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!input_dev || !ac)
		return -ENOMEM;
	ac->disabled = 1;
	ac->client = client;
	ac->input = input_dev;
	snprintf(ac->phys, sizeof(ac->phys),
		 "%s/input0", dev_name(&client->dev));

	INIT_WORK(&ac->work, ml8953_work);
	mutex_init(&ac->mutex);

	input_dev->name = "ML8953 Accelerometer";
	input_dev->phys = ac->phys;
	input_dev->dev.parent = &client->dev;
	input_dev->id.bustype = BUS_I2C;
	input_dev->open = ml8953_input_open;
	input_dev->close = ml8953_input_close;
	input_set_drvdata(input_dev, ac);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_Z, input_dev->absbit);
	__set_bit(ABS_MISC, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_X, -ML8953_RANGE, ML8953_RANGE, 3, 3);
	input_set_abs_params(input_dev, ABS_Y, -ML8953_RANGE, ML8953_RANGE, 3, 3);
	input_set_abs_params(input_dev, ABS_Z, -ML8953_RANGE, ML8953_RANGE, 3, 3);
	
	error = request_irq(client->irq, ml8953_irq,
			  IRQF_TRIGGER_FALLING, client->dev.driver->name, ac);
	if (error) {
		dev_err(&client->dev, "irq %d busy?\n", client->irq);
		goto free_input_dev;
	}
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&client->dev, "input device failed to register?\n");
		goto free_dev_irq;
	}

	error = sysfs_create_group(&client->dev.kobj, &ml8953_attr_group);
	if (error)
		goto free_dev_irq;
	i2c_set_clientdata(client, ac);
	ml8953_enable(ac);
	return 0;
free_dev_irq:
	free_irq(client->irq, ac);
free_input_dev:
	input_free_device(input_dev);
	kfree(ac);
	return error;
}

static int __devexit ml8953_i2c_remove(struct i2c_client *client)
{
	struct ml8953 *ac;

	ac = i2c_get_clientdata(client);
	free_irq(client->irq, ac);
	input_unregister_device(ac->input);
	dev_dbg(&client->dev, "unregistered accelerometer\n");
	kfree(ac);

	return 0;
}

#ifdef CONFIG_PM
static int ml8953_suspend(struct i2c_client *client, pm_message_t message)
{
	//ml8953_disable(i2c_get_clientdata(client));
	return 0;
}

static int ml8953_resume(struct i2c_client *client)
{
	ml8953_enable(i2c_get_clientdata(client));
	return 0;
}
#else
# define ml8953_suspend NULL
# define ml8953_resume  NULL
#endif

static const struct i2c_device_id ml8953_id[] = {
	{ "ml8953", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ml8953_id);

static struct i2c_driver ml8953_driver = {
	.driver = {
		.name = "ml8953",
		.owner = THIS_MODULE,
	},
	.probe    = ml8953_i2c_probe,
	.remove   = __devexit_p(ml8953_i2c_remove),
	.suspend  = ml8953_suspend,
	.resume   = ml8953_resume,
	.id_table = ml8953_id,
};

static int __init ml8953_i2c_init(void)
{
	return i2c_add_driver(&ml8953_driver);
}
module_init(ml8953_i2c_init);

static void __exit ml8953_i2c_exit(void)
{
	i2c_del_driver(&ml8953_driver);
}
module_exit(ml8953_i2c_exit);

MODULE_AUTHOR("Matt Isaacs <izzy@buglabs.net");
MODULE_DESCRIPTION("ML8953 Three-Axis Digital Accelerometer Driver");
MODULE_LICENSE("GPL");

