/*
 * Copyright (c) 2012-2015,2017,2018-2020 The Linux Foundation. All rights reserved.
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

#include "wil6210_ethdev.h"
#include "wmi.h"
#include "txrx.h"
#include "wil6210_pmc.h"

struct desc_alloc_info {
	dma_addr_t pa;
	void	  *va;
};

static int wil_is_pmc_allocated(struct pmc_ctx *pmc)
{
	return !!pmc->pring_va;
}

void wil_pmc_init(struct wil6210_priv *wil)
{
	memset(&wil->pmc, 0, sizeof(struct pmc_ctx));
	mutex_init(&wil->pmc.lock);
}

/**
 * Allocate the physical ring (p-ring) and the required
 * number of descriptors of required size.
 * Initialize the descriptors as required by pmc dma.
 * The descriptors' buffers dwords are initialized to hold
 * dword's serial number in the lsw and reserved value
 * PCM_DATA_INVALID_DW_VAL in the msw.
 */
void wil_pmc_alloc(struct wil6210_priv *wil,
		   int num_descriptors,
		   int descriptor_size)
{
	uint32_t i;
	struct pmc_ctx *pmc = &wil->pmc;
	struct wil6210_vif *vif = ndev_to_vif(wil->main_ndev);
	struct wmi_pmc_cmd pmc_cmd = {0};
	int last_cmd_err = -ENOMEM;
	dma_addr_t desc_pa;
	uint8_t *desc_va;

	mutex_lock(&pmc->lock);

	if (wil_is_pmc_allocated(pmc)) {
		/* sanity check */
		wil_err(wil, "ERROR pmc is already allocated\n");
		goto no_release_err;
	}
	if ((num_descriptors <= 0) || (descriptor_size <= 0)) {
		wil_err(wil,
			"Invalid params num_descriptors(%d), descriptor_size(%d)\n",
			num_descriptors, descriptor_size);
		last_cmd_err = -EINVAL;
		goto no_release_err;
	}

	if (num_descriptors > (1 << WIL_RING_SIZE_ORDER_MAX)) {
		wil_err(wil,
			"num_descriptors(%d) exceeds max ring size %d\n",
			num_descriptors, 1 << WIL_RING_SIZE_ORDER_MAX);
		last_cmd_err = -EINVAL;
		goto no_release_err;
	}

	if (num_descriptors > INT_MAX / descriptor_size) {
		wil_err(wil,
			"Overflow in num_descriptors(%d)*descriptor_size(%d)\n",
			num_descriptors, descriptor_size);
		last_cmd_err = -EINVAL;
		goto no_release_err;
	}

	pmc->num_descriptors = num_descriptors;
	pmc->descriptor_size = descriptor_size;

	wil_dbg_misc(wil, "pmc_alloc: %d descriptors x %d bytes each\n",
		     num_descriptors, descriptor_size);

	/* allocate descriptors info list in pmc context*/
	pmc->descriptors = calloc(num_descriptors,
				  sizeof(struct desc_alloc_info));
	if (!pmc->descriptors) {
		wil_err(wil, "ERROR allocating pmc skb list\n");
		goto no_release_err;
	}

	wil_dbg_misc(wil, "pmc_alloc: allocated descriptors info list %p\n",
		     pmc->descriptors);

	/* Allocate pring buffer and descriptors.
	 * vring->va should be aligned on its size rounded up to power of 2
	 * This is granted by the wil_dma_zalloc_coherent.
	 *
	 * HW has limitation that all vrings addresses must share the same
	 * upper 16 msb bits part of 48 bits address.
	 * This should be ok in DPDK, because we allocate from DPDK memory
	 * zones which are in the same huge page area and should have
	 * same upper 16 msb bits set.
	 */
	pmc->pring_va = wil_dma_zalloc_coherent(wil, "pmc-ring", 0,
			sizeof(struct vring_tx_desc) * num_descriptors,
			&pmc->pring_dmah);
	pmc->pring_pa = pmc->pring_dmah.dma_addr;

	wil_dbg_misc(wil,
		     "pmc_alloc: allocated pring %p => %lx. %zd x %d = total %zd bytes\n",
		     pmc->pring_va, pmc->pring_pa,
		     sizeof(struct vring_tx_desc),
		     num_descriptors,
		     sizeof(struct vring_tx_desc) * num_descriptors);

	if (!pmc->pring_va) {
		wil_err(wil, "ERROR allocating pmc pring\n");
		goto release_pmc_skb_list;
	}

	pmc->desc_va = wil_dma_zalloc_coherent(wil,
			"pmc-desc", 0,
			descriptor_size * num_descriptors,
			&pmc->desc_dmah);
	if (!pmc->desc_va) {
		wil_err(wil, "ERROR allocating pmc descriptors memory\n");
		goto release_pmc_pring;
	}

	/* initially, all descriptors are SW owned
	 * For Tx, Rx, and PMC, ownership bit is at the same location, thus
	 * we can use any
	 */
	desc_va = pmc->desc_va;
	desc_pa = pmc->desc_dmah.dma_addr;
	for (i = 0; i < num_descriptors; i++) {
		struct vring_tx_desc *_d = &pmc->pring_va[i];
		struct vring_tx_desc dd = {}, *d = &dd;
		int j = 0;

		pmc->descriptors[i].va = desc_va;
		pmc->descriptors[i].pa = desc_pa;

		for (j = 0; j < descriptor_size / sizeof(u32); j++) {
			uint32_t *p = (uint32_t *)pmc->descriptors[i].va + j;
			*p = PCM_DATA_INVALID_DW_VAL | j;
		}

		/* configure dma descriptor */
		d->dma.addr.addr_low =
			cpu_to_le32(lower_32_bits(pmc->descriptors[i].pa));
		d->dma.addr.addr_high =
			cpu_to_le16((u16)upper_32_bits(pmc->descriptors[i].pa));
		d->dma.status = 0; /* 0 = HW_OWNED */
		d->dma.length = cpu_to_le16(descriptor_size);
		d->dma.d0 = BIT(9) | RX_DMA_D0_CMD_DMA_IT;
		*_d = *d;

		desc_va += descriptor_size;
		desc_pa += descriptor_size;
	}

	wil_dbg_misc(wil, "pmc_alloc: allocated successfully\n");

	pmc_cmd.op = WMI_PMC_ALLOCATE;
	pmc_cmd.ring_size = cpu_to_le16(pmc->num_descriptors);
	pmc_cmd.mem_base = cpu_to_le64(pmc->pring_pa);

	wil_dbg_misc(wil, "pmc_alloc: send WMI_PMC_CMD with ALLOCATE op\n");
	pmc->last_cmd_status = wmi_send(wil,
					WMI_PMC_CMDID,
					vif->mid,
					&pmc_cmd,
					sizeof(pmc_cmd));
	if (pmc->last_cmd_status) {
		wil_err(wil,
			"WMI_PMC_CMD with ALLOCATE op failed with status %d",
			pmc->last_cmd_status);
		goto release_pmc_desc_mem;
	}

	mutex_unlock(&pmc->lock);

	return;

release_pmc_desc_mem:
	wil_err(wil, "exit on error: Releasing skbs...\n");
	wil_dma_free_coherent(wil, &pmc->desc_dmah);
	pmc->desc_va = NULL;

release_pmc_pring:
	wil_err(wil, "exit on error: Releasing pring...\n");
	wil_dma_free_coherent(wil, &pmc->pring_dmah);
	pmc->pring_va = NULL;

release_pmc_skb_list:
	wil_err(wil, "exit on error: Releasing descriptors info list...\n");
	free(pmc->descriptors);
	pmc->descriptors = NULL;

no_release_err:
	pmc->last_cmd_status = last_cmd_err;
	mutex_unlock(&pmc->lock);
}

/**
 * Traverse the p-ring and release all buffers.
 * At the end release the p-ring memory
 */
void wil_pmc_free(struct wil6210_priv *wil, int send_pmc_cmd)
{
	struct pmc_ctx *pmc = &wil->pmc;
	struct wil6210_vif *vif = ndev_to_vif(wil->main_ndev);
	struct wmi_pmc_cmd pmc_cmd = {0};

	mutex_lock(&pmc->lock);

	pmc->last_cmd_status = 0;

	if (!wil_is_pmc_allocated(pmc)) {
		wil_dbg_misc(wil,
			     "pmc_free: Error, can't free - not allocated\n");
		pmc->last_cmd_status = -EPERM;
		mutex_unlock(&pmc->lock);
		return;
	}

	if (send_pmc_cmd) {
		wil_dbg_misc(wil, "send WMI_PMC_CMD with RELEASE op\n");
		pmc_cmd.op = WMI_PMC_RELEASE;
		pmc->last_cmd_status =
				wmi_send(wil, WMI_PMC_CMDID, vif->mid,
					 &pmc_cmd, sizeof(pmc_cmd));
		if (pmc->last_cmd_status) {
			wil_err(wil,
				"WMI_PMC_CMD with RELEASE op failed, status %d",
				pmc->last_cmd_status);
			/* There's nothing we can do with this error.
			 * Normally, it should never occur.
			 * Continue to freeing all memory allocated for pmc.
			 */
		}
	}

	if (pmc->pring_va) {
		wil_dbg_misc(wil, "pmc_free: free pring va %p\n",
			     pmc->pring_va);
		wil_dma_free_coherent(wil, &pmc->pring_dmah);

		pmc->pring_va = NULL;
	} else {
		pmc->last_cmd_status = -ENOENT;
	}

	if (pmc->desc_va) {
		wil_dbg_misc(wil, "pmc_free: free descriptors va %p\n",
			     pmc->desc_va);
		wil_dma_free_coherent(wil, &pmc->desc_dmah);

		pmc->desc_va = NULL;
	} else {
		pmc->last_cmd_status = -ENOENT;
	}

	if (pmc->descriptors) {
		wil_dbg_misc(wil,
			     "pmc_free: free pmc descriptors info list %p\n",
			     pmc->descriptors);
		free(pmc->descriptors);
		pmc->descriptors = NULL;
	} else {
		pmc->last_cmd_status = -ENOENT;
	}

	mutex_unlock(&pmc->lock);
}

/**
 * Status of the last operation requested via debugfs: alloc/free/read.
 * 0 - success or negative errno
 */
int wil_pmc_last_cmd_status(struct wil6210_priv *wil)
{
	wil_dbg_misc(wil, "pmc_last_cmd_status: status %d\n",
		     wil->pmc.last_cmd_status);

	return wil->pmc.last_cmd_status;
}

static ssize_t
wil_pmc_calc_read_size(uint32_t total_size, size_t size, uint32_t offset,
	uint32_t base_size)
{
	/* enforce reading whole descriptors */
	if (offset % base_size)
		return -EINVAL;

	if (offset > total_size)
		return 0;

	size = min_t(size_t, size, total_size - offset);
	size /= base_size;
	size *= base_size;

	return size;
}

/*
 * context for PMC reader
 */
struct wil_pmc_reader_ctx {
	struct wil6210_priv *wil;
	uint32_t offset; /* read so far */
	uint32_t unit_size;
};

/**
 * read descriptors data or descriptors, starting from offset with
 * specified size.
 * if read_desc is true, read descriptors, otherwise read
 * descriptors data (payload)
 * To simplify the implementation, only allow reading whole
 * descriptors. Verify the offset is at descriptor start,
 * and align size.
 * Return the amount of bytes copied to the buffer.
 */
static ssize_t
__wil_pmc_read(struct wil_pmc_reader_ctx *pmcr, char *buf, size_t size,
	uint32_t offset, bool read_desc)
{
	struct wil6210_priv *wil = pmcr->wil;
	struct pmc_ctx *pmc = &wil->pmc;
	uint32_t total_size, unit_size, written, desc;
	ssize_t asize;

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc)) {
		wil_err(wil, "error, pmc is not allocated!\n");
		pmc->last_cmd_status = -EPERM;
		goto out;
	}


	/* enforce reading whole descriptors */
	total_size = pmc->num_descriptors * pmcr->unit_size;
	asize = wil_pmc_calc_read_size(total_size, size, offset,
				       pmcr->unit_size);
	if (asize < 0) {
		wil_err(wil, "invalid offset %d, must be at descriptor start\n",
			offset);
		pmc->last_cmd_status = -EINVAL;
		goto out;
	}

	pmc->last_cmd_status = 0;
	wil_dbg_misc(wil,
		     "pmc_read: size %d offset %d adjusted %d, read_desc %d\n",
		     (int)size, (int)offset, (int)asize, (int)read_desc);

	if (asize == 0)
		goto out;

	desc = offset / pmcr->unit_size;
	if (read_desc) {
		memcpy(buf, pmc->pring_va + desc, asize);
	} else {
		written = 0;
		while (written < asize) {
			memcpy(buf + written, pmc->descriptors[desc].va,
			       pmcr->unit_size);
			written += pmcr->unit_size;
			desc++;
		}
	}

out:
	mutex_unlock(&pmc->lock);
	return (pmc->last_cmd_status < 0) ? pmc->last_cmd_status : asize;
}

static ssize_t wil_pmc_read(void *ctx, char *buf, size_t size)
{
	struct wil_pmc_reader_ctx *pmcr = ctx;
	struct wil6210_priv *wil = pmcr->wil;
	ssize_t rc;

	rc = __wil_pmc_read(pmcr, buf, size, pmcr->offset, false);
	if (rc > 0)
		pmcr->offset += rc;

	return rc;
}

static ssize_t wil_pmc_read_size(void *ctx, size_t size)
{
	struct wil_pmc_reader_ctx *pmcr = ctx;
	struct wil6210_priv *wil = pmcr->wil;
	struct pmc_ctx *pmc = &wil->pmc;
	uint32_t total_size;
	ssize_t rc;

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc)) {
		wil_err(wil, "error, pmc is not allocated!\n");
		pmc->last_cmd_status = -EPERM;
		mutex_unlock(&pmc->lock);
		return -EPERM;
	}

	total_size = pmcr->unit_size * pmc->num_descriptors;
	rc = wil_pmc_calc_read_size(total_size, size, pmcr->offset,
		pmcr->unit_size);

	mutex_unlock(&pmc->lock);
	return rc;
}

static ssize_t wil_pmc_min_read_size(void *ctx)
{
	struct wil_pmc_reader_ctx *pmcr = ctx;

	return pmcr->unit_size;
}

static ssize_t wil_pmc_available(void *ctx)
{
	struct wil_pmc_reader_ctx *pmcr = ctx;
	struct wil6210_priv *wil = pmcr->wil;
	struct pmc_ctx *pmc = &wil->pmc;
	uint32_t total_size;
	ssize_t rc;

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc)) {
		wil_err(wil, "error, pmc is not allocated!\n");
		pmc->last_cmd_status = -EPERM;
		mutex_unlock(&pmc->lock);
		return -EPERM;
	}

	total_size = pmc->num_descriptors * pmcr->unit_size;
	if (total_size < pmcr->offset) {
		pmc->last_cmd_status = -EINVAL;
		mutex_unlock(&pmc->lock);
		return -EINVAL;
	}

	rc = total_size - pmcr->offset;

	mutex_unlock(&pmc->lock);
	return rc;
}

static ssize_t wil_pmcring_read(void *ctx, char *buf, size_t size)
{
	struct wil_pmc_reader_ctx *pmcr = ctx;
	struct wil6210_priv *wil = pmcr->wil;
	ssize_t rc;

	rc = __wil_pmc_read(pmcr, buf, size, pmcr->offset, true);
	if (rc > 0)
		pmcr->offset += rc;

	return rc;
}

static struct wil_pmc_reader_ops pmc_reader_ops = {
	.read = wil_pmc_read,
	.read_size = wil_pmc_read_size,
	.min_read_size = wil_pmc_min_read_size,
	.available = wil_pmc_available,
};

static struct wil_pmc_reader_ops pmcring_reader_ops = {
	.read = wil_pmcring_read,
	.read_size = wil_pmc_read_size,
	.min_read_size = wil_pmc_min_read_size,
	.available = wil_pmc_available,
};

int wil_pmc_alloc_pmc_reader(struct wil6210_priv *wil,
	struct wil_pmc_reader_ops *ops, void **ctx)
{
	struct pmc_ctx *pmc = &wil->pmc;
	struct wil_pmc_reader_ctx *pmcr;

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc)) {
		wil_err(wil, "error, pmc is not allocated!\n");
		pmc->last_cmd_status = -EPERM;
		mutex_unlock(&pmc->lock);
		return -EPERM;
	}

	pmcr = malloc(sizeof(struct wil_pmc_reader_ctx));
	if (!pmcr) {
		pmc->last_cmd_status = -ENOMEM;
		mutex_unlock(&pmc->lock);
		return -ENOMEM;
	}

	pmcr->wil = wil;
	pmcr->offset = 0;
	pmcr->unit_size = pmc->descriptor_size;

	*ctx = pmcr;
	*ops = pmc_reader_ops;

	mutex_unlock(&pmc->lock);
	return 0;
}

int wil_pmc_alloc_pmcring_reader(struct wil6210_priv *wil,
	struct wil_pmc_reader_ops *ops, void **ctx)
{
	struct pmc_ctx *pmc = &wil->pmc;
	struct wil_pmc_reader_ctx *pmcr;

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc)) {
		wil_err(wil, "error, pmc is not allocated!\n");
		pmc->last_cmd_status = -EPERM;
		mutex_unlock(&pmc->lock);
		return -EPERM;
	}

	pmcr = malloc(sizeof(struct wil_pmc_reader_ctx));
	if (!pmcr) {
		pmc->last_cmd_status = -ENOMEM;
		mutex_unlock(&pmc->lock);
		return -ENOMEM;
	}

	pmcr->wil = wil;
	pmcr->offset = 0;
	pmcr->unit_size = sizeof(struct vring_rx_desc);

	*ctx = pmcr;
	*ops = pmcring_reader_ops;

	mutex_unlock(&pmc->lock);
	return 0;
}

void wil_pmc_free_reader(void *ctx)
{
	free(ctx);
}