/*
 * isph3a.c
 *
 * H3A module for TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 *	Sergio Aguirre <saaguirre@ti.com>
 *	Troy Laramy
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/dma-mapping.h>
#include <linux/uaccess.h>

#include "isp.h"

/* Structure for saving/restoring h3a module registers */
static struct isp_reg isph3a_reg_list[] = {
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR, 0}, /* Should be the first one */
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AEWWIN1, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AEWINSTART, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AEWINBLK, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AEWSUBWIN, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AEWBUFST, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFPAX1, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFPAX2, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFPAXSTART, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFIIRSH, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFBUFST, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF010, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF032, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF054, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF076, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF098, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF0010, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF110, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF132, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF154, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF176, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF198, 0},
	{OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF1010, 0},
	{0, ISP_TOK_TERM, 0}
};

static void isph3a_print_status(struct isp_h3a_device *h3a);

static void __isph3a_aewb_enable(struct isp_h3a_device *h3a, u8 enable)
{
	struct isp_device *isp = to_isp_device(h3a);
	u32 pcr = isp_reg_readl(isp, OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR);

	if (enable)
		pcr |= ISPH3A_PCR_AEW_EN;
	else
		pcr &= ~ISPH3A_PCR_AEW_EN;
	isp_reg_writel(isp, pcr, OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR);
}

/**
 * isph3a_aewb_enable - Enables AE, AWB engine in the H3A module.
 * @enable: 1 - Enables the AE & AWB engine.
 *
 * Client should configure all the AE & AWB registers in H3A before this.
 **/
void isph3a_aewb_enable(struct isp_h3a_device *h3a, u8 enable)
{
	unsigned long irqflags;

	spin_lock_irqsave(h3a->lock, irqflags);

	if (!h3a->aewb_config_local.aewb_enable && enable) {
		spin_unlock_irqrestore(h3a->lock, irqflags);
		return;
	}

	__isph3a_aewb_enable(h3a, enable);
	h3a->enabled = enable;

	spin_unlock_irqrestore(h3a->lock, irqflags);
}

/**
 * isph3a_aewb_suspend - Suspend AE, AWB engine in the H3A module.
 **/
void isph3a_aewb_suspend(struct isp_h3a_device *h3a)
{
	unsigned long flags;

	spin_lock_irqsave(h3a->lock, flags);

	if (h3a->enabled)
		__isph3a_aewb_enable(h3a, 0);

	spin_unlock_irqrestore(h3a->lock, flags);
}

/**
 * isph3a_aewb_resume - Resume AE, AWB engine in the H3A module.
 **/
void isph3a_aewb_resume(struct isp_h3a_device *h3a)
{
	unsigned long flags;

	spin_lock_irqsave(h3a->lock, flags);

	if (h3a->enabled)
		__isph3a_aewb_enable(h3a, 1);

	spin_unlock_irqrestore(h3a->lock, flags);
}

int isph3a_aewb_busy(struct isp_h3a_device *h3a)
{
	struct isp_device *isp = to_isp_device(h3a);

	return isp_reg_readl(isp, OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR)
		& ISPH3A_PCR_BUSYAEAWB;
}

/**
 * isph3a_aewb_update_regs - Helper function to update h3a registers.
 **/
static void isph3a_aewb_config_registers(struct isp_h3a_device *h3a)
{
	struct isp_device *isp = to_isp_device(h3a);
	unsigned long irqflags;

	if (!h3a->aewb_config_local.aewb_enable)
		return;

	spin_lock_irqsave(h3a->lock, irqflags);

	isp_reg_writel(isp, h3a->buf_next->iommu_addr,
		       OMAP3_ISP_IOMEM_H3A, ISPH3A_AEWBUFST);

	if (!h3a->update) {
		spin_unlock_irqrestore(h3a->lock, irqflags);
		return;
	}

	isp_reg_writel(isp, h3a->regs.win1, OMAP3_ISP_IOMEM_H3A,
		       ISPH3A_AEWWIN1);
	isp_reg_writel(isp, h3a->regs.start, OMAP3_ISP_IOMEM_H3A,
		       ISPH3A_AEWINSTART);
	isp_reg_writel(isp, h3a->regs.blk, OMAP3_ISP_IOMEM_H3A,
		       ISPH3A_AEWINBLK);
	isp_reg_writel(isp, h3a->regs.subwin, OMAP3_ISP_IOMEM_H3A,
		       ISPH3A_AEWSUBWIN);
	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR,
		       ~ISPH3A_PCR_AEW_MASK, h3a->regs.pcr);

	ispstat_bufs_set_size(&h3a->stat, h3a->buf_size);
	h3a->update = 0;

	spin_unlock_irqrestore(h3a->lock, irqflags);
}

void isph3a_aewb_try_enable(struct isp_h3a_device *h3a)
{
	unsigned long irqflags;

	spin_lock_irqsave(h3a->lock, irqflags);
	if (!h3a->enabled && h3a->aewb_config_local.aewb_enable) {
		h3a->update = 1;
		h3a->buf_next = ispstat_buf_next(&h3a->stat);
		spin_unlock_irqrestore(h3a->lock, irqflags);
		isph3a_aewb_config_registers(h3a);
		isph3a_aewb_enable(h3a, 1);
	} else
		spin_unlock_irqrestore(h3a->lock, irqflags);
}

/**
 * isph3a_update_wb - Updates WB parameters.
 *
 * Needs to be called when no ISP Preview processing is taking place.
 **/
void isph3a_update_wb(struct isp_h3a_device *h3a)
{
	struct isp_device *isp = to_isp_device(h3a);

	if (h3a->wb_update) {
		/* FIXME: Get the preview crap out of here!!! */
		isppreview_config_whitebalance(&isp->isp_prev,
					       &h3a->h3awb_update);
		h3a->wb_update = 0;
	}
	return;
}
EXPORT_SYMBOL(isph3a_update_wb);

void isph3a_aewb_isr(struct isp_h3a_device *h3a)
{
	isph3a_aewb_enable(h3a, 0);
	/* If it's busy we can't process this buffer anymore */
	if (!isph3a_aewb_busy(h3a)) {
		isph3a_aewb_buf_process(h3a);
		isph3a_aewb_config_registers(h3a);
	} else {
		dev_dbg(to_device(h3a),
			"h3a: cannot process buffer, device is busy.\n");
	}
	isph3a_aewb_enable(h3a, 1);
}

/**
 * isph3a_aewb_stats_available - Check for stats available of specified frame.
 * @aewbdata: Pointer to return AE AWB statistics data
 *
 * Returns 0 if successful, or -1 if statistics are unavailable.
 **/
static int isph3a_aewb_get_stats(struct isp_h3a_device *h3a,
				 struct isph3a_aewb_data *aewbdata)
{
	struct ispstat_buffer *buf;

	buf = ispstat_buf_get(&h3a->stat,
			      (__force void __user *)
			      aewbdata->h3a_aewb_statistics_buf,
			      aewbdata->frame_number);

	if (IS_ERR(buf))
		return PTR_ERR(buf);

	aewbdata->ts = buf->ts;
	aewbdata->config_counter = buf->config_counter;
	aewbdata->frame_number = buf->frame_number;

	ispstat_buf_release(&h3a->stat);

	return 0;
}

/**
 * isph3a_aewb_buf_process - Process H3A AEWB buffer.
 */
int isph3a_aewb_buf_process(struct isp_h3a_device *h3a)
{
	isph3a_update_wb(h3a);
	if (likely(!h3a->buf_err &&
				h3a->aewb_config_local.aewb_enable)) {
		int ret;

		ret = ispstat_buf_queue(&h3a->stat);
		h3a->buf_next = ispstat_buf_next(&h3a->stat);
		return ret;
	} else {
		h3a->buf_err = 0;
		return -1;
	}
}

static int isph3a_aewb_validate_params(struct isp_h3a_device *h3a,
				       struct isph3a_aewb_config *user_cfg)
{
	if (unlikely(user_cfg->saturation_limit > MAX_SATURATION_LIM))
		return -EINVAL;

	if (unlikely(user_cfg->win_height < MIN_WIN_H ||
		     user_cfg->win_height > MAX_WIN_H ||
		     user_cfg->win_height & 0x01))
		return -EINVAL;

	if (unlikely(user_cfg->win_width < MIN_WIN_W ||
		     user_cfg->win_width > MAX_WIN_W ||
		     user_cfg->win_width & 0x01))
		return -EINVAL;

	if (unlikely(user_cfg->ver_win_count < 1 ||
		     user_cfg->ver_win_count > MAX_WINVC))
		return -EINVAL;

	if (unlikely(user_cfg->hor_win_count < 1 ||
		     user_cfg->hor_win_count > MAX_WINHC))
		return -EINVAL;

	if (unlikely(user_cfg->ver_win_start > MAX_WINSTART))
		return -EINVAL;

	if (unlikely(user_cfg->hor_win_start > MAX_WINSTART))
		return -EINVAL;

	if (unlikely(user_cfg->blk_ver_win_start > MAX_WINSTART))
		return -EINVAL;

	if (unlikely(user_cfg->blk_win_height < MIN_WIN_H ||
		     user_cfg->blk_win_height > MAX_WIN_H ||
		     user_cfg->blk_win_height & 0x01))
		return -EINVAL;

	if (unlikely(user_cfg->subsample_ver_inc < MIN_SUB_INC ||
		     user_cfg->subsample_ver_inc > MAX_SUB_INC ||
		     user_cfg->subsample_ver_inc & 0x01))
		return -EINVAL;

	if (unlikely(user_cfg->subsample_hor_inc < MIN_SUB_INC ||
		     user_cfg->subsample_hor_inc > MAX_SUB_INC ||
		     user_cfg->subsample_hor_inc & 0x01))
		return -EINVAL;

	return 0;
}

/**
 * isph3a_aewb_set_params - Helper function to check & store user given params.
 * @user_cfg: Pointer to AE and AWB parameters struct.
 *
 * As most of them are busy-lock registers, need to wait until AEW_BUSY = 0 to
 * program them during ISR.
 *
 * Returns 0 if successful, or -EINVAL if any of the parameters are invalid.
 **/
static void isph3a_aewb_set_params(struct isp_h3a_device *h3a,
				   struct isph3a_aewb_config *user_cfg)
{
	if (h3a->aewb_config_local.saturation_limit !=
						user_cfg->saturation_limit) {
		WRITE_SAT_LIM(h3a->regs.pcr, user_cfg->saturation_limit);
		h3a->aewb_config_local.saturation_limit =
			user_cfg->saturation_limit;
		h3a->update = 1;
	}

	if (h3a->aewb_config_local.alaw_enable != user_cfg->alaw_enable) {
		WRITE_ALAW(h3a->regs.pcr, user_cfg->alaw_enable);
		h3a->aewb_config_local.alaw_enable = user_cfg->alaw_enable;
		h3a->update = 1;
	}

	if (h3a->aewb_config_local.win_height != user_cfg->win_height) {
		WRITE_WIN_H(h3a->regs.win1, user_cfg->win_height);
		h3a->aewb_config_local.win_height = user_cfg->win_height;
		h3a->update = 1;
	}

	if (h3a->aewb_config_local.win_width != user_cfg->win_width) {
		WRITE_WIN_W(h3a->regs.win1, user_cfg->win_width);
		h3a->aewb_config_local.win_width = user_cfg->win_width;
		h3a->update = 1;
	}

	if (h3a->aewb_config_local.ver_win_count !=
						user_cfg->ver_win_count) {
		WRITE_VER_C(h3a->regs.win1, user_cfg->ver_win_count);
		h3a->aewb_config_local.ver_win_count =
						user_cfg->ver_win_count;
		h3a->update = 1;
	}

	if (h3a->aewb_config_local.hor_win_count !=
						user_cfg->hor_win_count) {
		WRITE_HOR_C(h3a->regs.win1, user_cfg->hor_win_count);
		h3a->aewb_config_local.hor_win_count =
						user_cfg->hor_win_count;
		h3a->update = 1;
	}

	if (h3a->aewb_config_local.ver_win_start !=
						user_cfg->ver_win_start) {
		WRITE_VER_WIN_ST(h3a->regs.start, user_cfg->ver_win_start);
		h3a->aewb_config_local.ver_win_start =
						user_cfg->ver_win_start;
		h3a->update = 1;
	}

	if (h3a->aewb_config_local.hor_win_start !=
						user_cfg->hor_win_start) {
		WRITE_HOR_WIN_ST(h3a->regs.start, user_cfg->hor_win_start);
		h3a->aewb_config_local.hor_win_start =
						user_cfg->hor_win_start;
		h3a->update = 1;
	}

	if (h3a->aewb_config_local.blk_ver_win_start !=
	    user_cfg->blk_ver_win_start) {
		WRITE_BLK_VER_WIN_ST(h3a->regs.blk,
				     user_cfg->blk_ver_win_start);
		h3a->aewb_config_local.blk_ver_win_start =
			user_cfg->blk_ver_win_start;
		h3a->update = 1;
	}

	if (h3a->aewb_config_local.blk_win_height !=
						user_cfg->blk_win_height) {
		WRITE_BLK_WIN_H(h3a->regs.blk, user_cfg->blk_win_height);
		h3a->aewb_config_local.blk_win_height =
						user_cfg->blk_win_height;
		h3a->update = 1;
	}

	if (h3a->aewb_config_local.subsample_ver_inc !=
	    user_cfg->subsample_ver_inc) {
		WRITE_SUB_VER_INC(h3a->regs.subwin,
				  user_cfg->subsample_ver_inc);
		h3a->aewb_config_local.subsample_ver_inc =
			user_cfg->subsample_ver_inc;
		h3a->update = 1;
	}

	if (h3a->aewb_config_local.subsample_hor_inc !=
	    user_cfg->subsample_hor_inc) {
		WRITE_SUB_HOR_INC(h3a->regs.subwin,
				  user_cfg->subsample_hor_inc);
		h3a->aewb_config_local.subsample_hor_inc =
			user_cfg->subsample_hor_inc;
		h3a->update = 1;
	}

	h3a->aewb_config_local.aewb_enable = user_cfg->aewb_enable;;
}

/**
 * isph3a_aewb_config - Configure AEWB regs, enable/disable H3A engine.
 * @aewbcfg: Pointer to AEWB config structure.
 *
 * Returns 0 if successful, -EINVAL if aewbcfg pointer is NULL, -ENOMEM if
 * was unable to allocate memory for the buffer, of other errors if H3A
 * callback is not set or the parameters for AEWB are invalid.
 **/
int isph3a_aewb_config(struct isp_h3a_device *h3a,
				struct isph3a_aewb_config *aewbcfg)
{
	struct isp_device *isp = to_isp_device(h3a);
	int ret = 0;
	int win_count = 0;
	unsigned int buf_size;
	unsigned long irqflags;

	if (NULL == aewbcfg) {
		dev_dbg(isp->dev, "h3a: Null argument in configuration\n");
		return -EINVAL;
	}

	ret = isph3a_aewb_validate_params(h3a, aewbcfg);
	if (ret)
		return ret;

	/* FIXME: This win_count handling looks really fishy. */
	win_count = aewbcfg->ver_win_count * aewbcfg->hor_win_count;
	win_count += aewbcfg->hor_win_count;
	ret = win_count / 8;
	win_count += win_count % 8 ? 1 : 0;
	win_count += ret;

	buf_size = win_count * AEWB_PACKET_SIZE;

	ret = ispstat_bufs_alloc(&h3a->stat, buf_size, 0);
	if (ret)
		return ret;

	spin_lock_irqsave(h3a->lock, irqflags);

	h3a->win_count = win_count;
	h3a->buf_size = buf_size;
	isph3a_aewb_set_params(h3a, aewbcfg);

	spin_unlock_irqrestore(h3a->lock, irqflags);

	isph3a_print_status(h3a);

	return 0;
}
EXPORT_SYMBOL(isph3a_aewb_config);

/**
 * isph3a_aewb_request_statistics - REquest statistics and update gains in AEWB
 * @aewbdata: Pointer to return AE AWB statistics data.
 *
 * This API allows the user to update White Balance gains, as well as
 * exposure time and analog gain. It is also used to request frame
 * statistics.
 *
 * Returns 0 if successful, -EINVAL when H3A engine is not enabled, or other
 * errors when setting gains.
 **/
int isph3a_aewb_request_statistics(struct isp_h3a_device *h3a,
				   struct isph3a_aewb_data *aewbdata)
{
	struct isp_device *isp = to_isp_device(h3a);
	unsigned long irqflags;
	int ret = 0;

	if (!h3a->aewb_config_local.aewb_enable) {
		dev_dbg(isp->dev, "h3a: engine not enabled\n");
		return -EINVAL;
	}

	DPRINTK_ISPH3A("isph3a_aewb_request_statistics: Enter "
		       "(frame req. => %d, current frame => %d,"
		       "update => %d)\n",
		       aewbdata->frame_number, h3a->stat.frame_number,
		       aewbdata->update);
	DPRINTK_ISPH3A("User data received: \n");
	DPRINTK_ISPH3A("Digital gain = 0x%04x\n", aewbdata->dgain);
	DPRINTK_ISPH3A("WB gain b *=   0x%04x\n", aewbdata->wb_gain_b);
	DPRINTK_ISPH3A("WB gain r *=   0x%04x\n", aewbdata->wb_gain_r);
	DPRINTK_ISPH3A("WB gain gb =   0x%04x\n", aewbdata->wb_gain_gb);
	DPRINTK_ISPH3A("WB gain gr =   0x%04x\n", aewbdata->wb_gain_gr);

	spin_lock_irqsave(h3a->lock, irqflags);

	if (aewbdata->update & SET_DIGITAL_GAIN)
		h3a->h3awb_update.dgain = (u16)aewbdata->dgain;
	if (aewbdata->update & SET_COLOR_GAINS) {
		h3a->h3awb_update.coef0 = (u8)aewbdata->wb_gain_gr;
		h3a->h3awb_update.coef1 = (u8)aewbdata->wb_gain_r;
		h3a->h3awb_update.coef2 = (u8)aewbdata->wb_gain_b;
		h3a->h3awb_update.coef3 = (u8)aewbdata->wb_gain_gb;
	}
	if (aewbdata->update & (SET_COLOR_GAINS | SET_DIGITAL_GAIN))
		h3a->wb_update = 1;

	spin_unlock_irqrestore(h3a->lock, irqflags);

	if (aewbdata->update & REQUEST_STATISTICS)
		ret = isph3a_aewb_get_stats(h3a, aewbdata);

	aewbdata->curr_frame = h3a->stat.frame_number;

	DPRINTK_ISPH3A("isph3a_aewb_request_statistics: "
		       "aewbdata->h3a_aewb_statistics_buf => %p\n",
		       aewbdata->h3a_aewb_statistics_buf);

	return ret;
}
EXPORT_SYMBOL(isph3a_aewb_request_statistics);

/**
 * isph3a_aewb_init - Module Initialisation.
 *
 * Always returns 0.
 **/
int isph3a_aewb_init(struct isp_device *isp)
{
	struct isp_h3a_device *h3a = &isp->isp_h3a;

	h3a->lock = &isp->h3a_lock;
	h3a->aewb_config_local.saturation_limit = AEWB_SATURATION_LIMIT;
	ispstat_init(isp, "H3A", &h3a->stat, H3A_MAX_BUFF, MAX_FRAME_COUNT);

	return 0;
}

/**
 * isph3a_aewb_cleanup - Module exit.
 **/
void isph3a_aewb_cleanup(struct isp_device *isp)
{
	ispstat_free(&isp->isp_h3a.stat);
}

/**
 * isph3a_print_status - Debug print. Values of H3A related registers.
 **/
static void isph3a_print_status(struct isp_h3a_device *h3a)
{
	DPRINTK_ISPH3A("ISPH3A_PCR = 0x%08x\n",
		       isp_reg_readl(to_isp_device(h3a),
				     OMAP3_ISP_IOMEM_H3A,
				     ISPH3A_PCR));
	DPRINTK_ISPH3A("ISPH3A_AEWWIN1 = 0x%08x\n",
		       isp_reg_readl(to_isp_device(h3a),
				     OMAP3_ISP_IOMEM_H3A,
				     ISPH3A_AEWWIN1));
	DPRINTK_ISPH3A("ISPH3A_AEWINSTART = 0x%08x\n",
		       isp_reg_readl(to_isp_device(h3a),
				     OMAP3_ISP_IOMEM_H3A,
				     ISPH3A_AEWINSTART));
	DPRINTK_ISPH3A("ISPH3A_AEWINBLK = 0x%08x\n",
		       isp_reg_readl(to_isp_device(h3a),
				     OMAP3_ISP_IOMEM_H3A,
				     ISPH3A_AEWINBLK));
	DPRINTK_ISPH3A("ISPH3A_AEWSUBWIN = 0x%08x\n",
		       isp_reg_readl(to_isp_device(h3a),
				     OMAP3_ISP_IOMEM_H3A,
				     ISPH3A_AEWSUBWIN));
	DPRINTK_ISPH3A("ISPH3A_AEWBUFST = 0x%08x\n",
		       isp_reg_readl(to_isp_device(h3a),
				     OMAP3_ISP_IOMEM_H3A,
				     ISPH3A_AEWBUFST));
	DPRINTK_ISPH3A("stats windows = %d\n", h3a->win_count);
	DPRINTK_ISPH3A("stats buf size = %d\n", h3a->stat.buf_size);
}

/**
 * isph3a_save_context - Saves the values of the h3a module registers.
 **/
void isph3a_save_context(struct isp_device *isp)
{
	DPRINTK_ISPH3A(" Saving context\n");
	isp_save_context(isp, isph3a_reg_list);
	/* Avoid enable during restore ctx */
	isph3a_reg_list[0].val &= ~(ISPH3A_PCR_AEW_EN | ISPH3A_PCR_AF_EN);
}
EXPORT_SYMBOL(isph3a_save_context);

/**
 * isph3a_restore_context - Restores the values of the h3a module registers.
 **/
void isph3a_restore_context(struct isp_device *isp)
{
	DPRINTK_ISPH3A(" Restoring context\n");
	isp_restore_context(isp, isph3a_reg_list);
}
EXPORT_SYMBOL(isph3a_restore_context);
