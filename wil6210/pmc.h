/*
 * Copyright (c) 2012-2015 Qualcomm Atheros, Inc.
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
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

#define PCM_DATA_INVALID_DW_VAL (0xB0BA0000)
#define WIL_PMC_EXT_RING_ORDER_DEF (10)
#define WIL_PMC_EXT_PAYLOAD_SIZE_DEF (2048)

#define WIL_RGF_PMC_RING_INDEX (0x00886838)
#define WIL_RGF_PMC_RINGS_BASE_ADDRESS (0x00881310)
#define WIL_RGF_PMC_RING_BLOCK_SIZE (0x70)
#define WIL_PMC_RINGS_INDEX_MAX (12)
#define WIL_PMC_INFO_SECTION_CANARY_VALUE (0xBACA1234)

void wil_pmc_init(struct wil6210_priv *wil);
void wil_pmc_alloc(struct wil6210_priv *wil,
		   int num_descriptors, int descriptor_size);
void wil_pmc_free(struct wil6210_priv *wil);
int wil_pmc_ext_alloc(struct wil6210_priv *wil, int num_descriptors,
		      int descriptor_size);
int wil_pmc_ext_free(struct wil6210_priv *wil);
void wil_pmc_ext_stop(struct wil6210_priv *wil);
const char *wil_pmc_ext_get_status(struct wil6210_priv *wil);
int wil_pmc_last_cmd_status(struct wil6210_priv *wil);
ssize_t wil_pmc_read(struct file *, char __user *, size_t, loff_t *);
loff_t wil_pmc_llseek(struct file *filp, loff_t off, int whence);
int wil_pmcring_read(struct seq_file *s, void *data);
int wil_pmc_ext_get_data(struct wil6210_priv *wil, u8 *buffer, u32 buffer_size,
			 u32 *bytes, u8 *extra_data, u32 *first_desc,
			 u32 *last_desc);
int wil_pmc_ext_get_data_manual(struct wil6210_priv *wil, u8 *buffer,
				u32 buffer_size, u32 *bytes, u32 first_desc,
				u32 *last_desc);
void wil_pmc_ext_pre_config(struct wil6210_priv *wil);
int wil_pmc_ext_post_config(struct wil6210_priv *wil);

u32 wil_pmc_ext_get_data_size(struct wil6210_priv *wil);
int wil_pmc_ext_copy_host_data(struct wil6210_priv *wil, void *dest, u32 size);
