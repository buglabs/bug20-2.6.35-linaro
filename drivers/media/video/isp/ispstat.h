/*
 * ispstat.h
 *
 * Copyright (C) 2009 Nokia Corporation
 *
 * Contact: Sakari Ailus <sakari.ailus@nokia.com>
 * 	    David Cohen <david.cohen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef ISPSTAT_H
#define ISPSTAT_H

#include <linux/types.h>
#include <plat/isp_user.h>
#include <plat/dma.h>
#include <media/v4l2-event.h>

#include "isp.h"
#include "ispvideo.h"

#define STAT_MAX_BUFS		5
#define STAT_NEVENTS		8

#define STAT_BUF_DONE		0	/* Buffer is ready */
#define STAT_NO_BUF		1	/* An error has occurred */
#define STAT_BUF_WAITING_DMA	2	/* Histogram only: DMA is running */

struct ispstat;

struct ispstat_buffer {
	unsigned long iommu_addr;
	struct iovm_struct *iovm;
	void *virt_addr;
	dma_addr_t dma_addr;
	struct timeval ts;
	u32 buf_size;
	u32 frame_number;
	u16 config_counter;
	u8 empty;
};

struct ispstat_ops {
	/*
	 * Validate new params configuration.
	 * new_conf->buf_size value must be changed to the exact buffer size
	 * necessary for the new configuration if it's smaller.
	 */
	int (*validate_params)(struct ispstat *stat, void *new_conf);

	/*
	 * Save new params configuration.
	 * stat->priv->buf_size value must be set to the exact buffer size for
	 * the new configuration.
	 * stat->update is set to 1 if new configuration is different than
	 * current one.
	 */
	void (*set_params)(struct ispstat *stat, void *new_conf);

	/* Apply stored configuration. */
	void (*setup_regs)(struct ispstat *stat);

	/* Used for specific operations during generic buf process task. */
	int (*buf_process)(struct ispstat *stat);
};

struct ispstat_pcr_bits {
	u32 base;
	u32 offset;
	u32 enable;
	u32 busy;
};

enum ispstat_state_t {
	ISPSTAT_DISABLED = 0,
	ISPSTAT_DISABLING,
	ISPSTAT_ENABLED,
	ISPSTAT_ENABLING,
};

struct ispstat {
	struct v4l2_subdev subdev;
	struct media_entity_pad pad;	/* sink pad */

	/* Control */
	unsigned configured:1;
	unsigned update:1;
	unsigned buf_processing:1;
	u8 inc_config;
	atomic_t buf_err;
	enum ispstat_state_t state;	/* enabling/disabling state */
	struct omap_dma_channel_params dma_config;
	struct isp_device *isp;
	void *priv;		/* pointer to priv config struct */
	struct mutex ioctl_lock; /* serialize private ioctl */

	const struct ispstat_ops *ops;
	const struct ispstat_pcr_bits *pcr;

	/* Buffer */
	u8 wait_acc_frames;
	u16 config_counter;
	u32 frame_number;
	u32 buf_size;
	u32 buf_alloc_size;
	int dma_ch;
	unsigned long event_type;
	struct ispstat_buffer *buf;
	struct ispstat_buffer *active_buf;
	struct ispstat_buffer *locked_buf;
};

struct ispstat_generic_config {
	/*
	 * Fields must be in the same order as in:
	 *  - isph3a_aewb_config
	 *  - isph3a_af_config
	 *  - isphist_config
	 */
	u32 buf_size;
	u16 config_counter;
};

int ispstat_config(struct ispstat *stat, void *new_conf);
int ispstat_request_statistics(struct ispstat *stat, struct ispstat_data *data);
int ispstat_init(struct ispstat *stat, const char *name,
		 const struct v4l2_subdev_ops *sd_ops);
void ispstat_free(struct ispstat *stat);
int ispstat_subscribe_event(struct v4l2_subdev *subdev, struct v4l2_fh *fh,
			    struct v4l2_event_subscription *sub);
int ispstat_unsubscribe_event(struct v4l2_subdev *subdev, struct v4l2_fh *fh,
			      struct v4l2_event_subscription *sub);
int ispstat_s_stream(struct v4l2_subdev *subdev, int enable);
void ispstat_pcr_enable(struct ispstat *stat, u8 enable);

int ispstat_busy(struct ispstat *stat);
void ispstat_suspend(struct ispstat *stat);
void ispstat_resume(struct ispstat *stat);
int ispstat_enable(struct ispstat *stat, u8 enable);
void ispstat_sbl_overflow(struct ispstat *stat);
void ispstat_isr(struct ispstat *stat);
void ispstat_isr_frame_sync(struct ispstat *stat);
void ispstat_dma_isr(struct ispstat *stat);
int ispstat_register_entities(struct ispstat *stat, struct v4l2_device *vdev);
void ispstat_unregister_entities(struct ispstat *stat);

#endif /* ISPSTAT_H */
