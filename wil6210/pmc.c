/*
 * Copyright (c) 2012-2015,2017 Qualcomm Atheros, Inc.
 * Copyright (c) 2017,2019-2021, The Linux Foundation. All rights reserved.
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

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include "wmi.h"
#include "wil6210.h"
#include "txrx.h"
#include "pmc.h"

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
	u32 i;
	struct pmc_ctx *pmc = &wil->pmc;

	/* initially, all descriptors are SW owned
	 * For Tx, Rx, and PMC, ownership bit is at the same location, thus
	 * we can use any
	 */
	for (i = 0; i < pmc->num_descriptors; i++) {
		struct vring_tx_desc *_d = &pmc->pring_va[i];
		struct vring_tx_desc dd = {}, *d = &dd;
		int j = 0;

		for (j = 0; j < pmc->descriptor_size / sizeof(u32); j++) {
			u32 *p = (u32 *)pmc->descriptors[i].va + j;
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
	}
}

static int wil_pmc_mem_alloc(struct wil6210_priv *wil, int num_descriptors,
			     int descriptor_size)
{
	u32 i;
	struct pmc_ctx *pmc = &wil->pmc;
	struct device *dev = wil_to_dev(wil);
	int rc = -ENOMEM;

	if (wil_is_pmc_allocated(pmc)) {
		/* sanity check */
		wil_info(wil, "WARNING: pmc is already allocated\n");
		return 0;
	}

	if ((num_descriptors <= 0) || (descriptor_size <= 0)) {
		wil_err(wil,
			"Invalid params num_descriptors(%d), descriptor_size(%d)\n",
			num_descriptors, descriptor_size);
		return -EINVAL;
	}

	if (num_descriptors > (1 << WIL_RING_SIZE_ORDER_MAX)) {
		wil_err(wil,
			"num_descriptors(%d) exceeds max ring size %d\n",
			num_descriptors, 1 << WIL_RING_SIZE_ORDER_MAX);
		return -EINVAL;
	}

	if (num_descriptors > INT_MAX / descriptor_size) {
		wil_err(wil,
			"Overflow in num_descriptors(%d)*descriptor_size(%d)\n",
			num_descriptors, descriptor_size);
		return -EINVAL;
	}

	pmc->num_descriptors = num_descriptors;
	pmc->descriptor_size = descriptor_size;

	wil_dbg_misc(wil, "pmc_mem_alloc: %d descriptors x %d bytes each\n",
		     num_descriptors, descriptor_size);

	/* allocate descriptors info list in pmc context*/
	pmc->descriptors = kcalloc(num_descriptors,
				  sizeof(struct desc_alloc_info),
				  GFP_KERNEL);
	if (!pmc->descriptors)
		return -ENOMEM;

	wil_dbg_misc(wil, "pmc_alloc: allocated descriptors info list %p\n",
		     pmc->descriptors);

	/* Allocate pring buffer and descriptors.
	 * vring->va should be aligned on its size rounded up to power of 2
	 * This is granted by the dma_alloc_coherent.
	 *
	 * HW has limitation that all vrings addresses must share the same
	 * upper 16 msb bits part of 48 bits address. To workaround that,
	 * if we are using more than 32 bit addresses switch to 32 bit
	 * allocation before allocating vring memory.
	 *
	 * There's no check for the return value of dma_set_mask_and_coherent,
	 * since we assume if we were able to set the mask during
	 * initialization in this system it will not fail if we set it again
	 */
	if (wil->dma_addr_size > 32)
		dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));

	pmc->pring_va = dma_alloc_coherent(dev,
			sizeof(struct vring_tx_desc) * num_descriptors,
			&pmc->pring_pa,
			GFP_KERNEL);

	if (wil->dma_addr_size > 32)
		dma_set_mask_and_coherent(dev,
					  DMA_BIT_MASK(wil->dma_addr_size));

	wil_dbg_misc(wil,
		     "pmc_alloc: allocated pring %p => %pad. %zd x %d = total %zd bytes\n",
		     pmc->pring_va, &pmc->pring_pa,
		     sizeof(struct vring_tx_desc),
		     num_descriptors,
		     sizeof(struct vring_tx_desc) * num_descriptors);

	if (!pmc->pring_va) {
		wil_err(wil, "ERROR allocating pmc pring\n");
		rc = -ENOMEM;
		goto release_pmc_skb_list;
	}

	for (i = 0; i < num_descriptors; i++) {
		pmc->descriptors[i].va = dma_alloc_coherent(dev,
			descriptor_size,
			&pmc->descriptors[i].pa,
			GFP_KERNEL);

		if (unlikely(!pmc->descriptors[i].va)) {
			wil_err(wil, "ERROR allocating pmc descriptor %d", i);
			rc = -ENOMEM;
			goto release_pmc_skbs;
		}

	}
	wil_pmc_init_mem(wil);

	wil_dbg_misc(wil, "pmc_alloc: allocated successfully\n");

	return 0;

release_pmc_skbs:
	wil_err(wil, "exit on error: Releasing skbs...\n");
	for (i = 0; i < num_descriptors && pmc->descriptors[i].va; i++) {
		dma_free_coherent(dev,
				  descriptor_size,
				  pmc->descriptors[i].va,
				  pmc->descriptors[i].pa);

		pmc->descriptors[i].va = NULL;
	}
	wil_err(wil, "exit on error: Releasing pring...\n");

	dma_free_coherent(dev,
			  sizeof(struct vring_tx_desc) * num_descriptors,
			  pmc->pring_va,
			  pmc->pring_pa);

	pmc->pring_va = NULL;

release_pmc_skb_list:
	wil_err(wil, "exit on error: Releasing descriptors info list...\n");
	kfree(pmc->descriptors);
	pmc->descriptors = NULL;

	return rc;
}

static int wil_pmc_mem_free(struct wil6210_priv *wil)
{
	struct pmc_ctx *pmc = &wil->pmc;
	struct device *dev = wil_to_dev(wil);
	int rc = 0;

	if (pmc->pring_va) {
		size_t buf_size = sizeof(struct vring_tx_desc) *
				  pmc->num_descriptors;

		wil_dbg_misc(wil, "pmc_free: free pring va %p\n",
			     pmc->pring_va);
		dma_free_coherent(dev, buf_size, pmc->pring_va, pmc->pring_pa);

		pmc->pring_va = NULL;
	} else {
		rc = -ENOENT;
	}

	if (pmc->descriptors) {
		int i;

		for (i = 0;
		     i < pmc->num_descriptors && pmc->descriptors[i].va; i++) {
			dma_free_coherent(dev,
					  pmc->descriptor_size,
					  pmc->descriptors[i].va,
					  pmc->descriptors[i].pa);
			pmc->descriptors[i].va = NULL;
		}
		wil_dbg_misc(wil, "pmc_free: free descriptor info %d/%d\n", i,
			     pmc->num_descriptors);
		wil_dbg_misc(wil,
			     "pmc_free: free pmc descriptors info list %p\n",
			     pmc->descriptors);
		kfree(pmc->descriptors);
		pmc->descriptors = NULL;
	} else {
		rc = -ENOENT;
	}

	return rc;
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

	if (test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities)) {
		wil_err(wil, "legacy PMC is not supported\n");
		pmc->last_cmd_status = -EINVAL;
		return;
	}

	mutex_lock(&pmc->lock);

	rc = wil_pmc_mem_alloc(wil,  num_descriptors, descriptor_size);
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

	if (test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities)) {
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

/**
 * Read from required position up to the end of current descriptor,
 * depends on descriptor size configured during alloc request.
 */
ssize_t wil_pmc_read(struct file *filp, char __user *buf, size_t count,
		     loff_t *f_pos)
{
	struct wil6210_priv *wil = filp->private_data;
	struct pmc_ctx *pmc = &wil->pmc;
	size_t retval = 0;
	unsigned long long idx;
	loff_t offset;
	size_t pmc_size;

	if (test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities)) {
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

	pmc_size = pmc->descriptor_size * pmc->num_descriptors;

	wil_dbg_misc(wil,
		     "pmc_read: size %u, pos %lld\n",
		     (u32)count, *f_pos);

	pmc->last_cmd_status = 0;

	idx = *f_pos;
	do_div(idx, pmc->descriptor_size);
	offset = *f_pos - (idx * pmc->descriptor_size);

	if (*f_pos >= pmc_size) {
		wil_dbg_misc(wil,
			     "pmc_read: reached end of pmc buf: %lld >= %u\n",
			     *f_pos, (u32)pmc_size);
		pmc->last_cmd_status = -ERANGE;
		goto out;
	}

	wil_dbg_misc(wil,
		     "pmc_read: read from pos %lld (descriptor %llu, offset %llu) %zu bytes\n",
		     *f_pos, idx, offset, count);

	/* if no errors, return the copied byte count */
	retval = simple_read_from_buffer(buf,
					 count,
					 &offset,
					 pmc->descriptors[idx].va,
					 pmc->descriptor_size);
	*f_pos += retval;
out:
	mutex_unlock(&pmc->lock);

	return retval;
}

loff_t wil_pmc_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;
	struct wil6210_priv *wil = filp->private_data;
	struct pmc_ctx *pmc = &wil->pmc;
	size_t pmc_size;

	if (test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities)) {
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

	pmc_size = pmc->descriptor_size * pmc->num_descriptors;

	switch (whence) {
	case 0: /* SEEK_SET */
		newpos = off;
		break;

	case 1: /* SEEK_CUR */
		newpos = filp->f_pos + off;
		break;

	case 2: /* SEEK_END */
		newpos = pmc_size;
		break;

	default: /* can't happen */
		newpos = -EINVAL;
		goto out;
	}

	if (newpos < 0) {
		newpos = -EINVAL;
		goto out;
	}
	if (newpos > pmc_size)
		newpos = pmc_size;

	filp->f_pos = newpos;

out:
	mutex_unlock(&pmc->lock);

	return newpos;
}

int wil_pmcring_read(struct seq_file *s, void *data)
{
	struct wil6210_priv *wil = s->private;
	struct pmc_ctx *pmc = &wil->pmc;
	size_t pmc_ring_size =
		sizeof(struct vring_rx_desc) * pmc->num_descriptors;

	if (test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities)) {
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

	wil_dbg_misc(wil, "pmcring_read: size %zu\n", pmc_ring_size);

	seq_write(s, pmc->pring_va, pmc_ring_size);

	mutex_unlock(&pmc->lock);

	return 0;
}

/* wil_pmc_ext_get_data: reads PMC data from the ring descriptors into
 * a buffer. The function sets extra_data flag if there is more data to read.
 *
 * In case the buffer_size = 0, the function caluclates the data size which
 * is waiting in the PMC ring and return its value in bytes parameter,
 * first_desc and last_desc will not be used, caller should pass it null.
 * In case buffer_size > 0, first_desc and last_desc must be non-NULL.
 */
int wil_pmc_ext_get_data(struct wil6210_priv *wil, u8 *buffer, u32 buffer_size,
			 u32 *bytes, u8 *extra_data, u32 *first_desc,
			 u32 *last_desc)
{
	struct pmc_ctx *pmc = &wil->pmc;
	int i;
	u32 bytes_count = 0;

	if (!test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities)) {
		wil_err(wil, "continuous PMC not supported\n");
		return -EINVAL;
	}

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc) || !pmc->sw_head_reg) {
		wil_dbg_misc(wil, "error, pmc is not ready!\n");
		mutex_unlock(&pmc->lock);
		return -EPERM;
	}

	/* read the hw tail */
	pmc->sw_head = wil_r(wil, pmc->sw_head_reg);
	wil_dbg_misc(wil, "PMC get data: sw_head=%d sw_tail=%d\n",
		     pmc->sw_head, pmc->sw_tail);

	if (pmc->sw_head >= pmc->num_descriptors) {
		wil_err(wil, "error, sw head=%d out of range\n",
			pmc->sw_head);
		mutex_unlock(&pmc->lock);

		return -EINVAL;
	}

	if (buffer_size && (!first_desc || !last_desc)) {
		wil_err(wil, "first and last desc should not be null\n");
		mutex_unlock(&pmc->lock);

		return -EINVAL;
	}

	*bytes = 0;
	*extra_data = 0;
	if (buffer_size)
		*first_desc = pmc->sw_tail;

	if (pmc->sw_tail !=  pmc->sw_head)
		printk("vt02 tail=%d head=%d\n", pmc->sw_tail, pmc->sw_head);
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

			wil_hex_dump_misc("PMC data ", DUMP_PREFIX_OFFSET, 16,
					  1, pmc->descriptors[i].va, length,
					  true);
		}

		i++;
		i %= pmc->num_descriptors;
		bytes_count += length;
	}

	*bytes = bytes_count;
	if (buffer_size) {
		/* update the sw_tail for all the descriptors we went over */
		pmc->sw_tail = i;
		*last_desc = pmc->sw_tail;
	}

	mutex_unlock(&pmc->lock);

	return 0;
}

/* wil_pmc_ext_get_data_manual: reads PMC data from the ring descriptors into
 * a buffer.
 */
int wil_pmc_ext_get_data_manual(struct wil6210_priv *wil, u8 *buffer,
				u32 buffer_size, u32 *bytes, u32 first_desc,
				u32 *last_desc)
{
	struct pmc_ctx *pmc = &wil->pmc;
	int i;
	u32 bytes_count = 0;

	if (!test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities)) {
		wil_err(wil, "continuous PMC not supported\n");
		return -EINVAL;
	}

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc)) {
		wil_dbg_misc(wil, "error, pmc is not ready!\n");
		mutex_unlock(&pmc->lock);
		return -EPERM;
	}

	if (!buffer_size || !buffer || !last_desc) {
		wil_err(wil, "last desc or buffer should not be null, buffer_size=%d\n",
			buffer_size);
		mutex_unlock(&pmc->lock);

		return -EINVAL;
	}

	if (first_desc >= pmc->num_descriptors) {
		wil_err(wil, "error, first_desc=%d, number of descriptors=%d\n",
			first_desc, pmc->num_descriptors);
		mutex_unlock(&pmc->lock);

		return -EINVAL;
	}

	/* read the data into the buffer */
	for (i = first_desc; bytes_count < buffer_size;) {
		u16 length = le16_to_cpu(pmc->pring_va[i].dma.length);

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

	mutex_unlock(&pmc->lock);

	return 0;
}

void wil_pmc_ext_pre_config(struct wil6210_priv *wil)
{
	struct wmi_pmc_ext_host_memory_info info = {0};
	struct pmc_ctx *pmc = &wil->pmc;
	void __iomem *dst;
	int rc;

	if (!test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities))
		return;

	if (!wil->pmc_ext_host)
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
		     "PMC ring_base_addr=%llx ring_size=%d payload_size=%d\n",
		     info.ring_base_addr, info.ring_size,
		     info.payload_size_bytes);

	wil_memcpy_toio_32(dst,
			   &info, sizeof(struct wmi_pmc_ext_host_memory_info));

	mutex_unlock(&pmc->lock);
}

int wil_pmc_ext_post_config(struct wil6210_priv *wil)
{
	struct pmc_ctx *pmc = &wil->pmc;

	if (!test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities))
		return 0;

	if (!wil->pmc_ext_host)
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

	if (!test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities)) {
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
	wil_pmc_ext_stop(wil);

	return 0;
}

void wil_pmc_ext_stop(struct wil6210_priv *wil)
{
	struct pmc_ctx *pmc = &wil->pmc;

	if (!test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities))
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

	if (!test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities))
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

u32 wil_pmc_ext_get_data_size(struct wil6210_priv *wil)
{
	struct pmc_ctx *pmc = &wil->pmc;
	u32 size;

	if (!test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities)) {
		wil_dbg_misc(wil, "continuous PMC not supported\n");
		return 0;
	}

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc)) {
		wil_dbg_misc(wil, "PMC host mode is not enabled\n");
		mutex_unlock(&pmc->lock);
		return 0;
	}
	size = pmc->descriptor_size * pmc->num_descriptors;
	mutex_unlock(&pmc->lock);

	return size;
}

int wil_pmc_ext_copy_host_data(struct wil6210_priv *wil, void *dest, u32 size)
{
	struct pmc_ctx *pmc = &wil->pmc;
	int i, count;
	u32 num_of_desc, first_desc_index, bytes_count = 0;

	if (!test_bit(WMI_FW_CAPABILITY_PMC_LOG, wil->fw_capabilities)) {
		wil_dbg_misc(wil, "continuous PMC not supported\n");
		return 0;
	}

	mutex_lock(&pmc->lock);

	if (!wil_is_pmc_allocated(pmc)) {
		wil_err(wil, "PMC host mode is not enabled\n");
		mutex_unlock(&pmc->lock);
		return -EPERM;
	}

	if (size < pmc->descriptor_size) {
		wil_err(wil,
			"PMC can not copy host memory, available size = %u\n",
			size);
		mutex_unlock(&pmc->lock);
		return -EINVAL;
	}

	/* update the sw head, read the hw tail register, in case of
	 * sys_assert before FW_READY event, the sw_head register might be
	 * invalid and initialized to 0.
	 */
	if (pmc->sw_head_reg)
		pmc->sw_head = wil_r(wil, pmc->sw_head_reg);

	/* calculate the number of descriptors to copy */
	num_of_desc = size / pmc->descriptor_size;
	first_desc_index = pmc->sw_head;

	if (num_of_desc >= pmc->num_descriptors)
		num_of_desc = pmc->num_descriptors;
	else
		first_desc_index = (pmc->sw_head + pmc->num_descriptors -
				    num_of_desc) % pmc->num_descriptors;

	wil_dbg_misc(wil, "first_desc=%d, sw_head=%d num_of_desc=%d size=%d",
		     first_desc_index, pmc->sw_head, num_of_desc, size);

	for (i = first_desc_index, count = 0; count < num_of_desc;
	     count++, i++, i %= pmc->num_descriptors) {
		u16 length = le16_to_cpu(pmc->pring_va[i].dma.length);

		memcpy(dest + bytes_count, pmc->descriptors[i].va,
		       length);
		bytes_count += length;
	}

	wil_info(wil,
		 "Save PMC host data, total memory copied is %u",
		 bytes_count);
	mutex_unlock(&pmc->lock);

	return 0;
}
