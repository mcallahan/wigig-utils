/*
 * Copyright (c) 2012-2015,2019-2020, The Linux Foundation. All rights reserved.
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
#ifndef WIL6210_PMC_H
#define WIL6210_PMC_H

#include <wil6210_ethdev.h>

#define PCM_DATA_INVALID_DW_VAL (0xB0BA0000)

/*
 * defines API for sequential reading of PMC data.
 * can be used for reading either PMC descriptors
 * or descriptors payload. Provided so callers can
 * use the same code for reading both types of PMC
 * data, and possibly more types in the future
 */
struct wil_pmc_reader_ops {
	ssize_t (*read)(void *ctx, char *buf, size_t size);
	ssize_t (*read_size)(void *ctx, size_t size);
	ssize_t (*min_read_size)(void *ctx);
	ssize_t (*available)(void *ctx);
};

void wil_pmc_init(struct wil6210_priv *wil);
void wil_pmc_alloc(struct wil6210_priv *wil,
		   int num_descriptors, int descriptor_size);
void wil_pmc_free(struct wil6210_priv *wil, int send_pmc_cmd);
int wil_pmc_last_cmd_status(struct wil6210_priv *wil);
int wil_pmc_alloc_pmc_reader(struct wil6210_priv *wil,
	struct wil_pmc_reader_ops *ops, void **ctx);
int wil_pmc_alloc_pmcring_reader(struct wil6210_priv *wil,
	struct wil_pmc_reader_ops *ops, void **ctx);
void wil_pmc_free_reader(void *ctx);

#endif /* WIL6210_PMC_H */