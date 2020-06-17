/*
 * Copyright (c) 2014,2017,2019-2020 The Linux Foundation. All rights reserved.
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

#define wil_hex_dump_memio(prefix_str, buf, len) \
	print_hex_dump_debug(WIL_DBG_LOG_MIO, "DBG[MIO ]" prefix_str, \
			     DUMP_PREFIX_OFFSET, 16, 1, buf, len, true)
#define wil_dbg_memio(wil, fmt, arg...) wil_dbg(wil, MIO, "DBG[MIO ]" fmt, ##arg)

static void __iomem *wil_memio_addr(struct wil6210_priv *wil, uint32_t addr,
				  uint32_t size, enum wil_memio_op op)
{
	void __iomem *a;
	u32 off;

	switch (op & wil_mmio_addr_mask) {
	case wil_mmio_addr_linker:
		a = wmi_buffer(wil, cpu_to_le32(addr));
		break;
	case wil_mmio_addr_ahb:
		a = wmi_addr(wil, addr);
		break;
	case wil_mmio_addr_bar:
		a = wmi_addr(wil, addr + WIL6210_FW_HOST_OFF);
		break;
	default:
		wil_err(wil, "Unsupported address mode, op = 0x%08x\n", op);
		return NULL;
	}

	off = (u8 *)a - wil->csr;
	if (size > wil->bar_size - off) {
		wil_err(wil, "Requested block does not fit into memory: "
			"off = 0x%08x size = 0x%08x\n", off, size);
		return NULL;
	}

	return a;
}

int wil_memio_dword(struct wil6210_priv *wil, struct wil_memio *io)
{
	void __iomem *a;
	int rc;

	wil_dbg_memio(wil, "IO: addr = 0x%08x val = 0x%08x op = 0x%08x\n",
		      io->addr, io->val, io->op);

	a = wil_memio_addr(wil, io->addr, sizeof(u32), io->op);
	if (!a) {
		wil_err(wil, "invalid address 0x%08x, op = 0x%08x\n", io->addr,
			io->op);
		return -EINVAL;
	}

	rc = wil_mem_access_lock(wil);
	if (rc)
		return rc;

	/* operation */
	switch (io->op & wil_mmio_op_mask) {
	case wil_mmio_read:
		io->val = readl(a);
		break;
	case wil_mmio_write:
		writel(io->val, a);
		wmb(); /* make sure write propagated to HW */
		break;
	default:
		wil_err(wil, "Unsupported operation, op = 0x%08x\n", io->op);
		wil_mem_access_unlock(wil);
		return -EINVAL;
	}
	wil_mem_access_unlock(wil);

	return 0;
}

int wil_memio_block(struct wil6210_priv *wil, struct wil_memio_block *io)
{
	void *block;
	void __iomem *a;
	int rc = 0;

	wil_dbg_memio(wil, "IO: addr = 0x%08x size = 0x%08x op = 0x%08x\n",
		      io->addr, io->size, io->op);

	/* size */
	if (io->size % 4) {
		wil_err(wil, "size is not multiple of 4:  0x%08x\n", io->size);
		return -EINVAL;
	}

	a = wil_memio_addr(wil, io->addr, io->size, io->op);
	if (!a) {
		wil_err(wil, "invalid address 0x%08x, op = 0x%08x\n", io->addr,
			io->op);
		return -EINVAL;
	}

	rc = wil_mem_access_lock(wil);
	if (rc) {
		kfree(block);
		return rc;
	}

	/* operation */
	switch (io->op & wil_mmio_op_mask) {
	case wil_mmio_read:
		wil_memcpy_fromio_32(io->block, a, io->size);
		wil_hex_dump_memio("Read  ", io->block, io->size);
		break;
	case wil_mmio_write:
		wil_memcpy_toio_32(a, io->block, io->size);
		wmb(); /* make sure write propagated to HW */
		wil_hex_dump_memio("Write ", io->block, io->size);
		break;
	default:
		wil_err(wil, "Unsupported operation, op = 0x%08x\n", io->op);
		rc = -EINVAL;
		break;
	}

out_unlock:
	wil_mem_access_unlock(wil);
	return rc;
}
