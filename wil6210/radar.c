// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2018,2020, The Linux Foundation. All rights reserved.
 */

#include <linux/moduleparam.h>
#include <linux/types.h>
#include "uapi/linux/wil6210_uapi.h"
#include "wil6210.h"
#include "wmi.h"
#include "radar.h"

bool radar_mode; /* = false; */
module_param(radar_mode, bool, 0444);
MODULE_PARM_DESC(radar_mode, " When enabled, driver will operate in radar only mode, networking operations will fail");

static u32 radar_fifo_max_size = 2 * 1024 * 1024;
module_param(radar_fifo_max_size, uint, 0444);
MODULE_PARM_DESC(radar_fifo_max_size, " Size of radar data buffer, default 2MB");

void wil_rdr_set_pulse_size(struct wil6210_priv *wil,
			    struct wmi_radar_config_select_event *evt)
{
	u32 pulse_size;

	if (!evt)
		return;

	if (evt->status != WMI_FW_STATUS_SUCCESS) {
		wil_err(wil, "Event status failure (%d)\n", evt->status);
		return;
	}

	pulse_size = le32_to_cpu(evt->pulse_size);
	wil_dbg_rdr(wil, "Setting radar pulse size to %u bytes\n", pulse_size);
	wil->rdr_ctx.pulse_size = pulse_size;
}

int wil_rdr_init(struct wil6210_priv *wil)
{
	struct device *dev = wil_to_dev(wil);

	/* Allocate FW control block */
	wil->rdr_ctx.fw_cb =
		dma_alloc_coherent(dev, sizeof(struct wil_rdr_fw_cb),
				    &wil->rdr_ctx.fw_cb_pa,
				    GFP_KERNEL);
	if (!wil->rdr_ctx.fw_cb)
		return -ENOMEM;

	wil->rdr_ctx.fw_cb->fw_tail = cpu_to_le32(1);

	spin_lock_init(&wil->rdr_ctx.sw_head_lock);

	return 0;
}

void wil_rdr_uninit(struct wil6210_priv *wil)
{
	struct device *dev = wil_to_dev(wil);

	if (!radar_mode)
		return;

	if (wil->rdr_ctx.fw_cb) {
		dma_free_coherent(dev, sizeof(struct wil_rdr_fw_cb),
				  wil->rdr_ctx.fw_cb, wil->rdr_ctx.fw_cb_pa);
		wil->rdr_ctx.fw_cb = NULL;
		wil->rdr_ctx.fw_cb_pa = 0;
	}

	wil_rdr_free_buffer(wil);
}

static int wil_rdr_send_base_addresses_to_fw(struct wil6210_priv *wil)
{
	int rc = 0;
	struct wil_rdr_ctx *ctx = &wil->rdr_ctx;
	struct wmi_radar_pci_control_cmd cmd = {
		.base_addr = cpu_to_le64(ctx->fifo_pa),
		.control_block_addr = cpu_to_le64(ctx->fw_cb_pa),
		.buffer_size = cpu_to_le32(ctx->fifo_size),
	};
	struct {
		struct wmi_cmd_hdr wmi;
		struct wmi_radar_pci_control_event evt;
	} __packed reply = {
		.evt = {.status = WMI_FW_STATUS_FAILURE},
	};

	wil_dbg_rdr(wil, "FIFO base address: pa 0x%llx\n", ctx->fifo_pa);
	wil_dbg_rdr(wil, "Control block base address: pa= 0x%llx\n",
		    ctx->fw_cb_pa);
	rc = wmi_call(wil, WMI_RADAR_PCI_CONTROL_CMDID, 0, &cmd,
		      sizeof(cmd), WMI_RADAR_PCI_CONTROL_EVENTID, &reply,
		      sizeof(reply), 100);
	if (!rc && reply.evt.status != WMI_FW_STATUS_SUCCESS) {
		wil_err(wil,
			"WMI_RADAR_PCI_CONTROL_CMDID failed, status 0x%02x\n",
			reply.evt.status);
		rc = -EINVAL;
	}

	return rc;
}

int wil_rdr_alloc_buffer(struct wil6210_priv *wil, u32 *fifo_size)
{
	int rc;
	struct wil_rdr_ctx *ctx = &wil->rdr_ctx;
	struct device *dev = wil_to_dev(wil);
	struct wil_rdr_fw_cb *cb = wil->rdr_ctx.fw_cb;

	/* Check that pulse_size is known */
	if (!ctx->pulse_size) {
		wil_err(wil,
			"Pulse size is unknown, cannot allocate radar FIFO\n");
		return -EFAULT;
	}
	wil_dbg_rdr(wil, "Pulse size is %u\n", ctx->pulse_size);

	/* In case buffer already exists, free it and initialize fw_cb */
	if (ctx->fifo) {
		wil_dbg_rdr(wil,
			    "Radar FIFO had already been allocated, reallocating\n");
		dma_free_coherent(dev, ctx->fifo_size, ctx->fifo, ctx->fifo_pa);
		memset(cb, 0, sizeof(struct wil_rdr_fw_cb));
		cb->fw_tail = cpu_to_le32(1);
	}

	/* Allocate FIFO */
	ctx->fifo = dma_alloc_coherent(dev, radar_fifo_max_size,
				       &ctx->fifo_pa,
				       GFP_KERNEL);
	if (!ctx->fifo)
		return -ENOMEM;

	/* Color the FIFO */
	memset(ctx->fifo, 0xB, radar_fifo_max_size);

	/* Trim fifo size to whole multiplicity of the pulse size */
	ctx->fifo_size = rounddown(radar_fifo_max_size, ctx->pulse_size);
	ctx->fifo_size_pulses = ctx->fifo_size / ctx->pulse_size;
	wil_dbg_rdr(wil, "fifo size=%u pulses (%u bytes)\n",
		    ctx->fifo_size_pulses, ctx->fifo_size);

	/* Return the fifo size */
	*fifo_size = ctx->fifo_size;

	/* Initialize sw_head and rcvd_pulses_cntr */
	ctx->sw_head = 0;
	wil_w(wil, RGF_RDR_SW_HEAD_PTR, ctx->sw_head);
	ctx->rcvd_pulses_cntr = 0;
	ctx->last_pulse_id = 0;
	ctx->last_burst_id = 0;

	/* Send base addresses to FW */
	rc = wil_rdr_send_base_addresses_to_fw(wil);
	if (rc) {
		wil_err(wil,
			"wil_rdr_send_base_addresses_to_fw() failed (%d)\n",
			rc);
		wil_rdr_free_buffer(wil);
		return rc;
	}

	return 0;
}

void wil_rdr_free_buffer(struct wil6210_priv *wil)
{
	struct device *dev = wil_to_dev(wil);

	if (!wil->rdr_ctx.fifo)
		return;

	dma_free_coherent(dev, radar_fifo_max_size, wil->rdr_ctx.fifo,
			  wil->rdr_ctx.fw_cb_pa);
	wil->rdr_ctx.fifo = NULL;
	wil->rdr_ctx.fifo_pa = 0;
}

static void wil_rdr_print_fw_cb(struct wil6210_priv *wil,
				struct wil_rdr_fw_cb *cb)
{
	struct wil_rdr_ctx *ctx = &wil->rdr_ctx;

	wil_dbg_rdr(wil, "Control Block: (FIFO size in pulses %d)\n",
		    ctx->fifo_size_pulses);
	wil_dbg_rdr(wil, " -- FW tail			[%d]\n",
		    le32_to_cpu(cb->fw_tail));
	wil_dbg_rdr(wil, " -- SW head			[%d]\n",
		    le32_to_cpu(cb->sw_head));
	wil_dbg_rdr(wil, " -- Last TSF Low		[%d]\n",
		    le32_to_cpu(cb->last_wr_pulse_tsf_low));
	wil_dbg_rdr(wil, " -- Pulse count		[%d]\n",
		    le32_to_cpu(cb->last_wr_pulse_count));
	wil_dbg_rdr(wil, " -- Updated bytes		[%d]\n",
		    le32_to_cpu(cb->last_wr_in_bytes));
	wil_dbg_rdr(wil, " -- Last pulse ID		[%d]\n",
		    le32_to_cpu(cb->last_wr_pulse_id));
	wil_dbg_rdr(wil, " -- Last burst ID		[%d]\n",
		    le32_to_cpu(cb->last_wr_burst_id));
	wil_dbg_rdr(wil, " -- Received pulses		[%llu]\n",
		    ctx->rcvd_pulses_cntr);
	wil_dbg_rdr(wil, " -- Host last pulse ID	[%d]\n",
		    ctx->last_pulse_id);
	wil_dbg_rdr(wil, " -- Host last burst ID	[%d]\n",
		    ctx->last_burst_id);
}

static inline u32 wil_rdr_cyclic_calc(s32 a, u32 size)
{
	return (a >= 0) ? a : (a + size);
}

int wil_ioc_memio_rdr_get_data(struct wil6210_priv *wil, void __user *data)
{
	struct wil_memio_rdr_block io;
	struct wil_rdr_ctx *ctx = &wil->rdr_ctx;
	struct wil_rdr_fw_cb cb;
	u32 size_pulses;
	u32 size_bytes;
	u32 sw_head_bytes;
	u32 old_sw_head = 0;
	u32 fix = (ctx->rcvd_pulses_cntr == 0) ? 1 : 0;
	void __user *dest;
	void *src;
	u32 length;
	ulong flags;
	int rc = 0;

	if (!ctx->fifo || ctx->fifo_size_pulses == 0)
		return -EFAULT;

	if (copy_from_user(&io, data, sizeof(io)))
		return -EFAULT;

	/* Cache control block */
	memcpy(&cb, ctx->fw_cb, sizeof(struct wil_rdr_fw_cb));

	/* Increment sw_head since it points to the last pulse that was read */
	spin_lock_irqsave(&ctx->sw_head_lock, flags);
	old_sw_head = ctx->sw_head;
	ctx->sw_head = (ctx->sw_head + 1 - fix) % ctx->fifo_size_pulses;
	sw_head_bytes = ctx->sw_head * ctx->pulse_size;

	/* Calculate how much data to read */
	size_pulses =
		wil_rdr_cyclic_calc(le32_to_cpu(cb.fw_tail) - ctx->sw_head,
				    ctx->fifo_size_pulses);
	if (size_pulses == 0) {
		if (ctx->last_pulse_id == le32_to_cpu(cb.last_wr_pulse_id) &&
		    ctx->last_burst_id == le32_to_cpu(cb.last_wr_burst_id)) {
			ctx->sw_head =
				wil_rdr_cyclic_calc(ctx->sw_head - 1 + fix,
						    ctx->fifo_size_pulses);
			goto bail;
		}
		size_pulses = ctx->fifo_size_pulses;
	}
	size_bytes = size_pulses * ctx->pulse_size;
	wil_dbg_rdr(wil,
		    "fw_tail=%u, sw_head=%u, size_pulses=%u, size_bytes = %u, fix=%u\n",
		    cb.fw_tail, ctx->sw_head, size_pulses, size_bytes, fix);
	wil_rdr_print_fw_cb(wil, &cb);

	wil_dbg_rdr(wil,
		    "buffer size = %u bytes, size to read = %u pulses (%u bytes)\n",
		    io.size, size_pulses, size_bytes);

	if (size_bytes > io.size) {
		size_bytes = rounddown(io.size, ctx->pulse_size);
		size_pulses = size_bytes / ctx->pulse_size;
		wil_dbg_rdr(wil, "Trimming size to %u pulses (%u bytes)\n",
			    size_pulses, size_bytes);
	}

	/* Copy the radar data to the user block */
	dest = io.block;
	src = ctx->fifo + sw_head_bytes;
	if (ctx->sw_head + size_pulses <= ctx->fifo_size_pulses) {
		length = size_bytes;
		wil_dbg_rdr(wil, "dest = %p, src = %p, length = %u bytes\n",
			    dest, src, length);
	} else {
		length = ctx->fifo_size - sw_head_bytes;
		wil_dbg_rdr(wil, "1. dest = %p, src = %p, length = %u bytes\n",
			    dest, src, length);
		if (copy_to_user(dest, src, length)) {
			rc = -EFAULT;
			goto bail;
		}

		dest = io.block + length;
		src = ctx->fifo;
		length = size_bytes - length;
		wil_dbg_rdr(wil, "2. dest = %p, src = %p, length = %u bytes\n",
			    dest, src, length);
	}
	if (copy_to_user(dest, src, length)) {
		rc = -EFAULT;
		goto bail;
	}

	/* Increment counter and save last burst and pulse IDs */
	ctx->rcvd_pulses_cntr += size_pulses;
	ctx->last_pulse_id = le32_to_cpu(cb.last_wr_pulse_id);
	ctx->last_burst_id = le32_to_cpu(cb.last_wr_burst_id);

	/* Update SW head pointer */
	ctx->sw_head = (ctx->sw_head + size_pulses - 1) % ctx->fifo_size_pulses;
	wil_dbg_rdr(wil, "sw_head: %u --> %u\n", old_sw_head, ctx->sw_head);

	/* Write SW head pointer to RGF */
	wil_w(wil, RGF_RDR_SW_HEAD_PTR, ctx->sw_head);

	rc = size_bytes;

bail:
	spin_unlock_irqrestore(&ctx->sw_head_lock, flags);

	return rc;
}

void wil_rdr_isr(struct wil6210_priv *wil)
{
	struct wil_rdr_ctx *ctx = &wil->rdr_ctx;
	u32 old_sw_head, sw_head_inc;
	ulong flags;

	if (ctx->fifo_size_pulses == 0)
		return;

	spin_lock_irqsave(&ctx->sw_head_lock, flags);

	old_sw_head = ctx->sw_head;
	sw_head_inc = le32_to_cpu(ctx->fw_cb->sw_head_inc);

	/* Increment sw_head */
	ctx->sw_head = (ctx->sw_head + sw_head_inc) % ctx->fifo_size_pulses;
	ctx->dropped_pulse_cnt += sw_head_inc;

	/* Write SW head pointer to RGF */
	wil_w(wil, RGF_RDR_SW_HEAD_PTR, ctx->sw_head);
	wil_dbg_rdr(wil, "radar isr: sw_head: %u --> %u\n", old_sw_head,
		    ctx->sw_head);

	spin_unlock_irqrestore(&ctx->sw_head_lock, flags);
}

