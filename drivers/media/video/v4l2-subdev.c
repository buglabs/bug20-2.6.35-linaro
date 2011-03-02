/*
 *  V4L2 subdevice support.
 *
 *  Copyright (C) 2009  Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>

static int subdev_fh_init(struct v4l2_subdev_fh *fh, struct v4l2_subdev *sd)
{
	/* Allocate probe format and crop in the same memory block */
	fh->probe_fmt = kzalloc((sizeof(*fh->probe_fmt) +
				sizeof(*fh->probe_crop)) * sd->entity.num_pads,
				GFP_KERNEL);
	if (fh->probe_fmt == NULL)
		return -ENOMEM;

	fh->probe_crop = (struct v4l2_rect *)
		(fh->probe_fmt + sd->entity.num_pads);

	return 0;
}

static void subdev_fh_free(struct v4l2_subdev_fh *fh)
{
	kfree(fh->probe_fmt);
	fh->probe_fmt = NULL;
	fh->probe_crop = NULL;
}

static int subdev_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct media_entity *entity;
	struct v4l2_subdev_fh *subdev_fh;
	int ret;

	if (!sd->initialized)
		return -EAGAIN;

	subdev_fh = kzalloc(sizeof(*subdev_fh), GFP_KERNEL);
	if (subdev_fh == NULL)
		return -ENOMEM;

	ret = subdev_fh_init(subdev_fh, sd);
	if (ret) {
		kfree(subdev_fh);
		return ret;
	}

	ret = v4l2_fh_init(&subdev_fh->vfh, vdev);
	if (ret)
		goto err;

	if (sd->flags & V4L2_SUBDEV_USES_EVENTS) {
		ret = v4l2_event_init(&subdev_fh->vfh);
		if (ret)
			goto err;

		ret = v4l2_event_alloc(&subdev_fh->vfh, sd->nevents);
		if (ret)
			goto err;
	}

	v4l2_fh_add(&subdev_fh->vfh);
	file->private_data = &subdev_fh->vfh;

	entity = media_entity_get(&sd->entity);
	if (!entity) {
		ret = -EBUSY;
		goto err;
	}

	return 0;

err:
	v4l2_fh_del(&subdev_fh->vfh);
	v4l2_fh_exit(&subdev_fh->vfh);
	subdev_fh_free(subdev_fh);
	kfree(subdev_fh);

	return ret;
}

static int subdev_close(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct v4l2_fh *vfh = file->private_data;
	struct v4l2_subdev_fh *subdev_fh = to_v4l2_subdev_fh(vfh);

	media_entity_put(&sd->entity);

	v4l2_fh_del(vfh);
	v4l2_fh_exit(vfh);
	subdev_fh_free(subdev_fh);
	kfree(subdev_fh);
	file->private_data = NULL;

	return 0;
}

static long subdev_do_ioctl(struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct v4l2_fh *vfh = file->private_data;
	struct v4l2_subdev_fh *subdev_fh = to_v4l2_subdev_fh(vfh);

	switch (cmd) {
	case VIDIOC_QUERYCTRL:
		return v4l2_subdev_call(sd, core, queryctrl, arg);

	case VIDIOC_QUERYMENU:
		return v4l2_subdev_call(sd, core, querymenu, arg);

	case VIDIOC_G_CTRL:
		return v4l2_subdev_call(sd, core, g_ctrl, arg);

	case VIDIOC_S_CTRL:
		return v4l2_subdev_call(sd, core, s_ctrl, arg);

	case VIDIOC_G_EXT_CTRLS:
		return v4l2_subdev_call(sd, core, g_ext_ctrls, arg);

	case VIDIOC_S_EXT_CTRLS:
		return v4l2_subdev_call(sd, core, s_ext_ctrls, arg);

	case VIDIOC_TRY_EXT_CTRLS:
		return v4l2_subdev_call(sd, core, try_ext_ctrls, arg);

	case VIDIOC_DQEVENT:
		if (!(sd->flags & V4L2_SUBDEV_USES_EVENTS))
			return -ENOIOCTLCMD;

		return v4l2_event_dequeue(vfh, arg, file->f_flags & O_NONBLOCK);

	case VIDIOC_SUBSCRIBE_EVENT:
		return v4l2_subdev_call(sd, core, subscribe_event, vfh, arg);

	case VIDIOC_UNSUBSCRIBE_EVENT:
		return v4l2_subdev_call(sd, core, unsubscribe_event, vfh, arg);

	case VIDIOC_SUBDEV_G_FMT: {
		struct v4l2_subdev_pad_format *format = arg;

		if (format->which != V4L2_SUBDEV_FORMAT_PROBE &&
		    format->which != V4L2_SUBDEV_FORMAT_ACTIVE)
			return -EINVAL;

		if (format->pad >= sd->entity.num_pads)
			return -EINVAL;

		return v4l2_subdev_call(sd, pad, get_fmt, subdev_fh,
					format->pad, &format->format,
					format->which);
	}

	case VIDIOC_SUBDEV_S_FMT: {
		struct v4l2_subdev_pad_format *format = arg;

		if (format->which != V4L2_SUBDEV_FORMAT_PROBE &&
		    format->which != V4L2_SUBDEV_FORMAT_ACTIVE)
			return -EINVAL;

		if (format->pad >= sd->entity.num_pads)
			return -EINVAL;

		return v4l2_subdev_call(sd, pad, set_fmt, subdev_fh,
					format->pad, &format->format,
					format->which);
	}

	case VIDIOC_SUBDEV_G_FRAME_INTERVAL:
		return v4l2_subdev_call(sd, video, g_frame_interval, arg);

	case VIDIOC_SUBDEV_S_FRAME_INTERVAL:
		return v4l2_subdev_call(sd, video, s_frame_interval, arg);

	case VIDIOC_SUBDEV_G_CROP: {
		struct v4l2_subdev_pad_crop *crop = arg;

		if (crop->which != V4L2_SUBDEV_FORMAT_PROBE &&
		    crop->which != V4L2_SUBDEV_FORMAT_ACTIVE)
			return -EINVAL;

		if (crop->pad >= sd->entity.num_pads)
			return -EINVAL;

		return v4l2_subdev_call(sd, pad, get_crop, subdev_fh, crop);
	}

	case VIDIOC_SUBDEV_S_CROP: {
		struct v4l2_subdev_pad_crop *crop = arg;

		if (crop->which != V4L2_SUBDEV_FORMAT_PROBE &&
		    crop->which != V4L2_SUBDEV_FORMAT_ACTIVE)
			return -EINVAL;

		if (crop->pad >= sd->entity.num_pads)
			return -EINVAL;

		return v4l2_subdev_call(sd, pad, set_crop, subdev_fh, crop);
	}

	case VIDIOC_SUBDEV_ENUM_MBUS_CODE: {
		struct v4l2_subdev_pad_mbus_code_enum *code = arg;

		if (code->pad >= sd->entity.num_pads)
			return -EINVAL;

		return v4l2_subdev_call(sd, pad, enum_mbus_code, subdev_fh,
					code);
	}

	case VIDIOC_SUBDEV_ENUM_FRAME_SIZE: {
		struct v4l2_subdev_frame_size_enum *fse = arg;

		if (fse->pad >= sd->entity.num_pads)
			return -EINVAL;

		return v4l2_subdev_call(sd, pad, enum_frame_size, subdev_fh,
					fse);
	}

	case VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL: {
		struct v4l2_subdev_frame_interval_enum *fie = arg;

		if (fie->pad >= sd->entity.num_pads)
			return -EINVAL;

		return v4l2_subdev_call(sd, pad, enum_frame_interval, subdev_fh,
					fie);
	}

	default:
		return v4l2_subdev_call(sd, core, ioctl, cmd, arg);
	}

	return 0;
}

static long subdev_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, subdev_do_ioctl);
}

static unsigned int subdev_poll(struct file *file, poll_table *wait)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct v4l2_fh *fh = file->private_data;

	if (!(sd->flags & V4L2_SUBDEV_USES_EVENTS))
		return POLLERR;

	poll_wait(file, &fh->events->wait, wait);

	if (v4l2_event_pending(fh))
		return POLLPRI;

	return 0;
}

const struct v4l2_file_operations v4l2_subdev_fops = {
	.owner = THIS_MODULE,
	.open = subdev_open,
	.unlocked_ioctl = subdev_ioctl,
	.release = subdev_close,
	.poll = subdev_poll,
};

void v4l2_subdev_init(struct v4l2_subdev *sd, const struct v4l2_subdev_ops *ops)
{
	INIT_LIST_HEAD(&sd->list);
	BUG_ON(!ops);
	sd->ops = ops;
	sd->flags = 0;
	sd->name[0] = '\0';
	sd->grp_id = 0;
	sd->priv = NULL;
	sd->entity.name = sd->name;
	sd->entity.type = MEDIA_ENTITY_TYPE_SUBDEV;
	sd->initialized = 1;
}
EXPORT_SYMBOL(v4l2_subdev_init);

int v4l2_subdev_set_power(struct media_entity *entity, int power)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);

	dev_dbg(entity->parent->dev,
		"%s power%s\n", entity->name, power ? "on" : "off");

	return v4l2_subdev_call(sd, core, s_power, power);
}
EXPORT_SYMBOL_GPL(v4l2_subdev_set_power);
