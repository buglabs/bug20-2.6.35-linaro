/*
    V4L2 device support.

    Copyright (C) 2008  Hans Verkuil <hverkuil@xs4all.nl>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/i2c.h>
#if defined(CONFIG_SPI)
#include <linux/spi/spi.h>
#endif
#include <linux/videodev2.h>
#include <media/v4l2-device.h>

int v4l2_device_register(struct device *dev, struct v4l2_device *v4l2_dev)
{
	if (v4l2_dev == NULL)
		return -EINVAL;

	INIT_LIST_HEAD(&v4l2_dev->subdevs);
	spin_lock_init(&v4l2_dev->lock);
	v4l2_dev->dev = dev;
	if (dev == NULL) {
		/* If dev == NULL, then name must be filled in by the caller */
		WARN_ON(!v4l2_dev->name[0]);
		return 0;
	}

	/* Set name to driver name + device name if it is empty. */
	if (!v4l2_dev->name[0])
		snprintf(v4l2_dev->name, sizeof(v4l2_dev->name), "%s %s",
			dev->driver->name, dev_name(dev));
	if (!dev_get_drvdata(dev))
		dev_set_drvdata(dev, v4l2_dev);
	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_device_register);

int v4l2_device_set_name(struct v4l2_device *v4l2_dev, const char *basename,
						atomic_t *instance)
{
	int num = atomic_inc_return(instance) - 1;
	int len = strlen(basename);

	if (basename[len - 1] >= '0' && basename[len - 1] <= '9')
		snprintf(v4l2_dev->name, sizeof(v4l2_dev->name),
				"%s-%d", basename, num);
	else
		snprintf(v4l2_dev->name, sizeof(v4l2_dev->name),
				"%s%d", basename, num);
	return num;
}
EXPORT_SYMBOL_GPL(v4l2_device_set_name);

void v4l2_device_disconnect(struct v4l2_device *v4l2_dev)
{
	if (v4l2_dev->dev == NULL)
		return;

	if (dev_get_drvdata(v4l2_dev->dev) == v4l2_dev)
		dev_set_drvdata(v4l2_dev->dev, NULL);
	v4l2_dev->dev = NULL;
}
EXPORT_SYMBOL_GPL(v4l2_device_disconnect);

void v4l2_device_unregister(struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd, *next;

	if (v4l2_dev == NULL)
		return;
	v4l2_device_disconnect(v4l2_dev);

	/* Unregister subdevs */
	list_for_each_entry_safe(sd, next, &v4l2_dev->subdevs, list) {
		v4l2_device_unregister_subdev(sd);
#if defined(CONFIG_I2C) || (defined(CONFIG_I2C_MODULE) && defined(MODULE))
		if (sd->flags & V4L2_SUBDEV_FL_IS_I2C) {
			struct i2c_client *client = v4l2_get_subdevdata(sd);

			/* We need to unregister the i2c client explicitly.
			   We cannot rely on i2c_del_adapter to always
			   unregister clients for us, since if the i2c bus
			   is a platform bus, then it is never deleted. */
			if (client)
				i2c_unregister_device(client);
		}
#endif
#if defined(CONFIG_SPI)
		if (sd->flags & V4L2_SUBDEV_FL_IS_SPI) {
			struct spi_device *spi = v4l2_get_subdevdata(sd);

			if (spi)
				spi_unregister_device(spi);
		}
#endif
	}
}
EXPORT_SYMBOL_GPL(v4l2_device_unregister);

int v4l2_device_register_subdev(struct v4l2_device *v4l2_dev,
				struct v4l2_subdev *sd)
{
	struct media_entity *entity = &sd->entity;
	struct video_device *vdev;
	int ret;

	/* Check for valid input */
	if (v4l2_dev == NULL || sd == NULL || !sd->name[0])
		return -EINVAL;

	/* Warn if we apparently re-register a subdev */
	WARN_ON(sd->v4l2_dev != NULL);

	if (!try_module_get(sd->owner))
		return -ENODEV;

	/* Register the entity. */
	if (v4l2_dev->mdev) {
		ret = media_device_register_entity(v4l2_dev->mdev, entity);
		if (ret < 0) {
			module_put(sd->owner);
			return ret;
		}
	}

	sd->v4l2_dev = v4l2_dev;
	spin_lock(&v4l2_dev->lock);
	list_add_tail(&sd->list, &v4l2_dev->subdevs);
	spin_unlock(&v4l2_dev->lock);

	/* Register the device node. */
	vdev = &sd->devnode;
	snprintf(vdev->name, sizeof(vdev->name), "subdev");
	vdev->parent = v4l2_dev->dev;
	vdev->fops = &v4l2_subdev_fops;
	vdev->release = video_device_release_empty;
	ret = video_register_device(vdev, VFL_TYPE_SUBDEV, -1);
	if (ret < 0) {
		spin_lock(&v4l2_dev->lock);
		list_del(&sd->list);
		spin_unlock(&v4l2_dev->lock);
		sd->v4l2_dev = NULL;
		if (v4l2_dev->mdev)
			media_device_unregister_entity(entity);
		module_put(sd->owner);
		return ret;
	}

	entity->v4l.major = VIDEO_MAJOR;
	entity->v4l.minor = vdev->minor;
	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_device_register_subdev);

void v4l2_device_unregister_subdev(struct v4l2_subdev *sd)
{
	struct v4l2_device *v4l2_dev;

	/* return if it isn't registered */
	if (sd == NULL || sd->v4l2_dev == NULL)
		return;

	v4l2_dev = sd->v4l2_dev;

	spin_lock(&v4l2_dev->lock);
	list_del(&sd->list);
	spin_unlock(&v4l2_dev->lock);
	sd->v4l2_dev = NULL;

	module_put(sd->owner);
	if (v4l2_dev->mdev)
		media_device_unregister_entity(&sd->entity);
	video_unregister_device(&sd->devnode);
}
EXPORT_SYMBOL_GPL(v4l2_device_unregister_subdev);

