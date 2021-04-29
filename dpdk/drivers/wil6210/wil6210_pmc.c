/*
 * Copyright (c) 2012-2015,2017-2021 The Linux Foundation. All rights reserved.
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

static int wil_pmc_get_sw_head_reg(struct wil6210_priv *wil)
{
	/* get the PMC sw head address register */
	u32 pmc_ring_index = wil_r(wil, WIL_RGF_PMC_RING_INDEX);

	pmc_ring_index = WIL_GET_BITS(pmc_ring_index, 8, 11);
	/* pmc ring cannot be 0 as this is used for Rx ring */
	if (pmc_ring_index == 0 || pmc_ring_index >= WIL_PMC_RINGS_INDEX_MAX) {
		wil_err(wil, "Invalid pmc ring index=%d,\n", pmc_ring_index);

		return -EINVAL;
	}

	wil->pmc.sw_head_reg = WIL_RGF_PMC_RINGS_BASE_ADDRESS +
		pmc_ring_index * WIL_RGF_PMC_RING_BLOCK_SIZE;

	wil_dbg_misc(wil, "pmc ring index=%d, sw_head_reg=0x%x\n",
		     pmc_ring_index, wil->pmc.sw_head_reg);

	return 0;
}

static void wil_pmc_init_mem(struct wil6210_priv *wil)
{
	uint32_t i;
	struct pmc_ctx *pmc = &wil->pmc;
	dma_addr_t desc_pa;
	uint8_t *desc_va;

	/* initially, all descriptors are SW owned
	 * For Tx, Rx, and PMC, ownership bit is at the same location, thus
	 * we can use any
	 */
	desc_va = pmc->desc_va;
	desc_pa = pmc->desc_dmah.dma_addr;
	for (i = 0; i < pmc->num_descriptors; i++) {
		struct vring_tx_desc *_d = &pmc->pring_va[i];
		struct vring_tx_desc dd = {}, *d = &dd;
		int j = 0;

		pmc->descriptors[i].va = desc_va;
		pmc->descriptors[i].pa = desc_pa;

		for (j = 0; j < pmc->descriptor_size / sizeof(u32); j++) {
			uint32_t *p = (uint32_t *)pmc->descriptors[i].va + j;
			*p = PCM_DATA_INVALID_DW_VAL | j;
		}

		/* configure dma descriptor */
		d->dma.addr.addr_low =
			cpu_to_le32(lower_32_bits(pmc->descriptors[i].pa));
		d->dma.addr.addr_high =
			cpu_to_le16((u16)upper_32_bits(pmc->descriptors[i].pa));
		d->dma.status = 0; /* 0 = HW_OWNED */
		d->dma.length = cpu_to_le16(pmc->descriptor_size);
		d->dma.d0 = BIT(9) | RX_DMA_D0_CMD_DMA_IT;
		*_d = *d;

		desc_va += pmc->descriptor_size;
		desc_pa += pmc->descriptor_size;
	}
}

static int wil_pmc_mem_alloc(struct wil6210_priv *wil, int num_descriptors,
			     int descriptor_size)
{
	uint32_t i;
	struct pmc_ctx *pmc = &wil->pmc;
	struct device *dev = wil_to_dev(wil);
	int rc = -EINVAL;

	if (wil_is_pmc_allocated(pmc)) {
		/* sanity check */
		wil_info(wil, "WARNING: pmc is already allocated\n");
		return rc;
	}

	if ((num_descriptors <= 0) || (descriptor_size <= 0)) {
		wil_err(wil,
			"Invalid params num_descriptors(%d), descriptor_size(%d)\n",
			num_descriptors, descriptor_size);
		return rc;
	}

	if (num_descriptors > (1 << WIL_RING_SIZE_ORDER_MAX)) {
		wil_err(wil,
			"num_descriptors(%d) exceeds max ring size %d\n",
			num_descriptors, 1 << WIL_RING_SIZE_ORDER_MAX);
		return rc;
	}

	if (num_descriptors > INT_MAX / descriptor_size) {
		wil_err(wil,
			"Overflow in num_descriptors(%d)*descriptor_size(%d)\n",
			num_descriptors, descriptor_size);
		return rc;
	}

	pmc->num_descriptors = num_descriptors;
	pmc->descriptor_size = descriptor_size;

	wil_dbg_misc(wil, "pmc_mem_alloc: %d descriptors x %d bytes each\n",
		     num_descriptors, descriptor_size);

	/* allocate descriptors info list in pmc context*/
	pmc->descriptors = calloc(num_descriptors,
				  sizeof(struct desc_alloc_info));
	if (!pmc->descriptors) {
		wil_err(wil, "ERROR allocating pmc skb list\n");
		return -ENOMEM;
	}

	wil_dbg_misc(wil, "pmc_mem_alloc: allocated descriptors info list %p\n",
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
		     "pmc_mem_alloc: allocated pring %p => %lx. %zd x %d = total %zd bytes\n",
		     pmc->pring_va, pmc->pring_pa, sizeof(struct vring_tx_desc),
		     num_descriptors,
		     sizeof(struct vring_tx_desc) * num_descriptors);

	if (!pmc->pring_va) {
		wil_err(wil, "ERROR allocating pmc pring\n");
		rc = -ENOMEM;
		goto release_pmc_skb_list;
	}

	pmc->desc_va = wil_dma_zalloc_coherent(wil,
			"pmc-desc", 0,
			descriptor_size * num_descriptors,
			&pmc->desc_dmah);
	if (!pmc->desc_va) {
		wil_err(wil, "ERROR allocating pmc descriptors memory\n");
		rc = -ENOMEM;
		goto release_pmc_pring;
	}

	wil_pmc_init_mem(wil);

	wil_dbg_misc(wil, "pmc_mem_alloc: allocated successfully\n");

	return 0;

release_pmc_pring:
	wil_err(wil, "exit on error: Releasing pring...\n");
	wil_dma_free_coherent(wil, &pmc->pring_dmah);
	pmc->pring_va = NULL;

release_pmc_skb_list:
	wil_err(wil, "exit on error: Releasing descriptors info list...\n");
	free(pmc->descriptors);
	pmc->descriptors = NULL;
	return rc;
}

static int wil_pmc_mem_free(struct wil6210_priv *wil)
{
	struct pmc_ctx *pmc = &wil->pmc;
	struct wil6210_vif *vif = ndev_to_vif(wil->main_ndev);

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

	return (pmc->last_cmd_status < 0) ? pmc->last_cmd_status : 0;
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
	struct pmc_ctx *pmc = &wil->pmc;
	int rc;

	if (wil->pmc_continuous_mode) {
		wil_err(wil, "legacy PMC is not supported\n");
		pmc->last_cmd_status = -EINVAL;
		return;
	}

	mutex_lock(&pmc->lock);

	rc = wil_pmc_mem_alloc(wil, num_descriptors, descriptor_size);
	if (rc)
		goto out;

	rc = wmi_pmc_alloc(wil, pmc->pring_pa, num_descriptors);
	if (rc) {
		wil_pmc_mem_free(wil);
		goto out;
	}

out:
	pmc->last_cmd_status = rc;
	mutex_unlock(&pmc->lock);
}

/**
 * Traverse the p-ring and release all buffers.
 * At the end release the p-ring memory
 */
void wil_pmc_free(struct wil6210_priv *wil)
{
	struct pmc_ctx *pmc = &wil->pmc;
	int rc1 = 0, rc = 0;

	pmc->last_cmd_status = 0;

	if (wil->pmc_continuous_mode) {
		wil_err(wil, "legacy PMC is not supported\n");
		pmc->last_cmd_status = -EINVAL;
		return;
	}

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc))
		goto out;

	rc1 = wmi_pmc_free(wil);
	rc = wil_pmc_mem_free(wil);
	pmc->last_cmd_status = rc1 ? rc1 : rc;

out:
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

int wil_pmc_ext_get_data(struct wil6210_priv *wil, char *buffer,
			 size_t buffer_size, uint32_t *bytes,
			 bool *extra_data, uint32_t *first_desc,
			 uint32_t *last_desc)
{
	struct pmc_ctx *pmc = &wil->pmc;
	int i, rc = 0;
	uint32_t bytes_count = 0;

	if (!wil->pmc_continuous_mode) {
		wil_err(wil, "continuous PMC is not supported\n");
		return -EINVAL;
	}

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc)) {
		wil_err(wil, "error, pmc is not allocated!\n");
		rc = -EPERM;
		goto out;
	}

	if (!pmc->sw_head_reg) {
		wil_err(wil, "error, pmc is not ready!\n");
		rc = -EAGAIN;
		goto out;
	}

	/* read the hw tail */
	pmc->sw_head = wil_r(wil, pmc->sw_head_reg);
	wil_dbg_misc(wil, "PMC ext get data: sw_head=%d sw_tail=%d\n",
		     pmc->sw_head, pmc->sw_tail);

	if (pmc->sw_head >= pmc->num_descriptors) {
		wil_err(wil, "error, sw head=%d out of range\n",
			pmc->sw_head);
		rc = -EINVAL;
		goto out;
	}

	if (buffer_size && (!first_desc || !last_desc)) {
		wil_err(wil, "first and last desc should not be null\n");
		rc = -EINVAL;
		goto out;
	}

	*bytes = 0;
	*extra_data = 0;
	if (buffer_size)
		*first_desc = pmc->sw_tail;

	/* read the data into the buffer */
	for (i = pmc->sw_tail; i != pmc->sw_head;) {
		u16 length = le16_to_cpu(pmc->pring_va[i].dma.length);

		if (buffer_size) {
			/* check that there is a room for the current descriptor
			 * data.
			 * note that this check is done when buffer_size > 0.
			 */
			if (length + bytes_count > buffer_size) {
				*extra_data = 1;
				break;
			}

			/* copy PMC ext data from descriptor payload to the
			 * buffer. Room for the data was guaranteed.
			 */
			memcpy(buffer + bytes_count, pmc->descriptors[i].va,
			       length);
			wil_dbg_misc(wil, "PMC descriptor=%d data length=%d\n",
				     i, length);

			wil_hex_dump_misc("descriptor ", DUMP_PREFIX_OFFSET, 16,
					  1, &pmc->pring_va[i],
					  sizeof(struct vring_tx_desc), true);

			wil_hex_dump_misc("PMC ext data ", DUMP_PREFIX_OFFSET,
					  16, 1, pmc->descriptors[i].va, length,
					  true);
		}

		i++;
		i %= pmc->num_descriptors;
		bytes_count += length;
	}

	*bytes = bytes_count;

	if (buffer_size) {
		/* update the sw_tail to all the descriptors we went through */
		pmc->sw_tail = i;
		*last_desc = pmc->sw_tail;
	}
out:
	mutex_unlock(&pmc->lock);
	return rc;
}

/* wil_pmc_ext_get_data_manual: reads PMC data from the ring descriptors into
 * a buffer.
 */
int wil_pmc_ext_get_data_manual(struct wil6210_priv *wil, char *buffer,
				size_t buffer_size, uint32_t *bytes,
				uint32_t first_desc, uint32_t *last_desc)
{
	struct pmc_ctx *pmc = &wil->pmc;
	int i, rc = 0;
	uint32_t bytes_count = 0;
	uint16_t length;

	if (!wil->pmc_continuous_mode) {
		wil_err(wil, "continuous PMC not supported\n");
		return -EINVAL;
	}

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc)) {
		wil_dbg_misc(wil, "error, pmc is not ready!\n");
		rc = -EPERM;
		goto out;
	}

	if (!buffer_size || !buffer || !last_desc) {
		wil_err(wil, "last desc or buffer should not be null, buffer_size=%d\n",
			buffer_size);
		rc = -EINVAL;
		goto out;
	}

	if (first_desc >= pmc->num_descriptors) {
		wil_err(wil, "error, first_desc=%d, number of descriptors=%d\n",
			first_desc, pmc->num_descriptors);
		rc = -EINVAL;
		goto out;
	}

	/* read the data into the buffer */
	for (i = first_desc; bytes_count < buffer_size;) {
		length = le16_to_cpu(pmc->pring_va[i].dma.length);

		/* check that there is a room for the current descriptor data.
		 */
		if (length + bytes_count > buffer_size)
			break;

		/* copy PMC data from descriptor payload to the buffer. Room
		 * for the data was guaranteed.
		 */
		memcpy(buffer + bytes_count, pmc->descriptors[i].va, length);
		wil_dbg_misc(wil, "PMC descriptor=%d data length=%d\n", i,
			     length);

		wil_hex_dump_misc("descriptor ", DUMP_PREFIX_OFFSET, 16, 1,
				  &pmc->pring_va[i],
				  sizeof(struct vring_tx_desc), true);

		wil_hex_dump_misc("PMC data ", DUMP_PREFIX_OFFSET, 16, 1,
				  pmc->descriptors[i].va, length, true);

		i++;
		i %= pmc->num_descriptors;
		bytes_count += length;
	}
	*bytes = bytes_count;
	*last_desc = i;
out:
	mutex_unlock(&pmc->lock);
	return rc;
}

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

	if (wil->pmc_continuous_mode) {
		wil_err(wil, "legacy PMC is not supported\n");
		pmc->last_cmd_status = -EINVAL;
		return -EINVAL;
	}

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

	if (wil->pmc_continuous_mode) {
		wil_err(wil, "legacy PMC is not supported\n");
		pmc->last_cmd_status = -EINVAL;
		return -EINVAL;
	}

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
	struct wil6210_priv *wil = pmcr->wil;
	struct pmc_ctx *pmc = &wil->pmc;

	if (wil->pmc_continuous_mode) {
		wil_err(wil, "legacy PMC is not supported\n");
		pmc->last_cmd_status = -EINVAL;
		return -EINVAL;
	}

	return pmcr->unit_size;
}

static ssize_t wil_pmc_available(void *ctx)
{
	struct wil_pmc_reader_ctx *pmcr = ctx;
	struct wil6210_priv *wil = pmcr->wil;
	struct pmc_ctx *pmc = &wil->pmc;
	uint32_t total_size;
	ssize_t rc;

	if (wil->pmc_continuous_mode) {
		wil_err(wil, "legacy PMC is not supported\n");
		pmc->last_cmd_status = -EINVAL;
		return -EINVAL;
	}

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

void wil_pmc_ext_pre_config(struct wil6210_priv *wil)
{
	struct wmi_pmc_ext_host_memory_info info = {0};
	struct pmc_ctx *pmc = &wil->pmc;
	void __iomem *dst;
	int rc;

	if (!wil->pmc_continuous_mode || !wil->pmc_ext_host)
		return;

	dst = wmi_buffer_block(wil, pmc->pmc_ext_fw_info_address,
			       sizeof(struct wmi_pmc_ext_host_memory_info));
	if (!dst || !pmc->pmc_ext_fw_info_address) {
		wil_info(wil, "Invalid pmc fw info address 0x%x. Fallback into PMC FW mode",
			 le32_to_cpu(pmc->pmc_ext_fw_info_address));
		wil->pmc_ext_host = false;
		return;
	}

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc)) {
		rc = wil_pmc_mem_alloc(wil, 1 << wil->pmc_ext_ring_order,
				       WIL_PMC_EXT_PAYLOAD_SIZE_DEF);
		if (rc) {
			wil_err(wil, "PMC alloc failed. rc=%d\n", rc);
			mutex_unlock(&pmc->lock);
			return;
		}
	} else {
		wil_dbg_misc(wil,
			     "pmc was allocated - initialize the descriptors memory\n");
		wil_pmc_init_mem(wil);
	}

	pmc->sw_head = 0;
	pmc->sw_tail = 0;

	/* set fw PMC data before releasing fw cpu */
	info.canary_data = cpu_to_le32(WIL_PMC_INFO_SECTION_CANARY_VALUE);
	info.ring_base_addr = cpu_to_le64(pmc->pring_pa);
	info.ring_size = cpu_to_le16(pmc->num_descriptors);
	info.payload_size_bytes = cpu_to_le16(pmc->descriptor_size);

	wil_dbg_misc(wil,
		     "PMC ring_base_addr=%lx ring_size=%d payload_size=%d\n",
		     info.ring_base_addr, info.ring_size,
		     info.payload_size_bytes);

	wil_memcpy_toio_32(dst, &info, sizeof(struct wmi_pmc_ext_host_memory_info));

	mutex_unlock(&pmc->lock);
}

int wil_pmc_ext_post_config(struct wil6210_priv *wil)
{
	struct pmc_ctx *pmc = &wil->pmc;

	if (!wil->pmc_continuous_mode || !wil->pmc_ext_host)
		return 0;

	mutex_lock(&pmc->lock);

	if (!pmc->sw_head_reg && wil_pmc_get_sw_head_reg(wil)) {
		mutex_unlock(&pmc->lock);
		wil_pmc_ext_stop(wil);
		return -EINVAL;
	}
	mutex_unlock(&pmc->lock);

	return 0;
}

int wil_pmc_ext_alloc(struct wil6210_priv *wil, int num_descriptors,
		      int descriptor_size)
{
	struct pmc_ctx *pmc = &wil->pmc;
	int rc;

	if (!wil->pmc_continuous_mode) {
		wil_err(wil, "continuous PMC not supported\n");
		return -EINVAL;
	}

	mutex_lock(&pmc->lock);

	if (wil_is_pmc_allocated(pmc)) {
		wil_err(wil, "error, pmc is allocated!\n");
		mutex_unlock(&pmc->lock);
		return -EPERM;
	}

	pmc->sw_head = 0;
	pmc->sw_tail = 0;

	rc = wil_pmc_mem_alloc(wil, num_descriptors, descriptor_size);
	if (rc)
		goto out;

	rc = wmi_pmc_ext_start_host(wil, pmc->pring_pa, num_descriptors,
				    descriptor_size);
	if (rc) {
		wil_pmc_mem_free(wil);
		goto out;
	}

	if (!pmc->sw_head_reg && wil_pmc_get_sw_head_reg(wil)) {
		wil_err(wil, "error, could not get sw head register\n");
		wil_pmc_ext_stop(wil);
		rc = -EINVAL;
	}
out:
	if (!rc)
		wil->pmc_ext_host = true;

	mutex_unlock(&pmc->lock);

	return rc;
}

int wil_pmc_ext_free(struct wil6210_priv *wil)
{
	struct pmc_ctx *pmc = &wil->pmc;

	pmc->last_cmd_status = 0;

	wil_pmc_ext_stop(wil);

	return 0;
}

void wil_pmc_ext_stop(struct wil6210_priv *wil)
{
	struct pmc_ctx *pmc = &wil->pmc;

	if (!wil->pmc_continuous_mode)
		return;

	mutex_lock(&pmc->lock);

	wmi_pmc_ext_stop(wil);
	wil_pmc_mem_free(wil);
	wil->pmc_ext_host = false;

	mutex_unlock(&pmc->lock);
}

const char *wil_pmc_ext_get_status(struct wil6210_priv *wil)
{
	int status;

	if (!wil->pmc_continuous_mode)
		return "continuous PMC not supported";

	status = wmi_pmc_ext_get_status(wil);
	switch (status) {
	case WMI_PMC_STATUS_OFF:
		return "PMC_STATUS_OFF";
	case WMI_PMC_STATUS_ON_FW_MODE:
		return "PMC_STATUS_ON_FW_MODE";
	case WMI_PMC_STATUS_ON_HOST_MODE:
		return "PMC_STATUS_ON_HOST_MODE";
	default:
		return "unknown status";
	}
}
