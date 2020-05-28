/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019, Facebook, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "wil6210_compat.h"
#include "wil_platform.h"

struct dpdk_plat_ctx {
	struct wil_platform_rops rops;
	void *wil_handle;
	struct device *dev;
	/* virtual addr and size for crash dump  */
	void __iomem *fw_io_mem_addr;
	u32 fw_io_mem_size;
};

static int dpdk_wil_notify_crash(struct dpdk_plat_ctx *ctx)
{
	int rc;

	dev_err(ctx->dev, "%s: FW error detected\n", __func__);

	if (ctx->rops.ramdump && ctx->wil_handle) {
		dev_err(ctx->dev, "%s: Copying fw dump\n", __func__);

		rc = ctx->rops.ramdump(ctx->wil_handle, NULL, 0);
		if (rc)
			dev_err(ctx->dev, "%s: Error, Ramdump failed : %d\n",
				__func__, rc);
	}

	if (!ctx->rops.fw_recovery) {
		dev_err(ctx->dev, "%s: No fw_recovery operation\n", __func__);
		goto crash;
	}

	rc = ctx->rops.fw_recovery(ctx->wil_handle);
	if (rc == 0)
		return 0;

crash:
	dev_err(ctx->dev, "WIGIG: Asserting Host...\n");
	BUG_ON(1);

	return 0;
}

static int ops_notify(void *handle, enum wil_platform_event evt)
{
	struct dpdk_plat_ctx *ctx = (struct dpdk_plat_ctx *)handle;
	int rc = 0;

	if (!ctx) {
		RTE_LOG(ERR, PMD, "%s: Context not found\n", __func__);
		return -ENODEV;
	}

	switch (evt) {
	case WIL_PLATFORM_EVT_FW_CRASH:
		rc = dpdk_wil_notify_crash(ctx);
		break;
	default:
		dev_dbg(ctx->dev, "%s: Unhandled event %d\n", __func__, evt);
		break;
	}

	return rc;
}

static void ops_uninit(void *handle)
{
	struct dpdk_plat_ctx *ctx = (struct dpdk_plat_ctx *)handle;

	if (!ctx) {
		RTE_LOG(ERR, PMD, "%s: Error Getting Context", __func__);
		return;
	}

	dev_info(ctx->dev, "%s: WIL6210 platform device uninit.\n", __func__);
	rte_free(ctx);
}

/* Do not create header file for just one function */
void *dpdk_wil_dev_init(struct device *dev, struct wil_platform_ops *ops,
			const struct wil_platform_rops *rops, void *wil_handle);

void *dpdk_wil_dev_init(struct device *dev, struct wil_platform_ops *ops,
			const struct wil_platform_rops *rops, void *wil_handle)
{
	struct dpdk_plat_ctx *ctx;

	memset(ops, 0, sizeof(*ops));

	dev_info(dev, "%s: DPDK platform device init.\n", __func__);

	ctx = rte_zmalloc("wil6210_plat", sizeof(*ctx), 0);
	if (!ctx)
		return NULL;
	ctx->dev = dev;

	/* subsystem restart */
	if (rops) {
		ctx->rops = *rops;
		ctx->wil_handle = wil_handle;
	}

	/* fill ops */
	ops->notify = ops_notify;
	ops->uninit = ops_uninit;

	return ctx;
}
