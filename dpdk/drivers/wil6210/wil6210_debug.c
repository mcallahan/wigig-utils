/*
 * Copyright (c) 2013,2016 Qualcomm Atheros, Inc.
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
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

#include "wil6210_ethdev.h"

struct wil_log_state {
	int log_type_idx;
	const char *log_type_str;
};

static struct wil_log_state wil_dbg_log[WIL_DBG_LOG_NUM] = {
    [ WIL_DBG_LOG_IRQ ] = { 0, "pmd.wil.irq" },
    [ WIL_DBG_LOG_TXRX ] = { 0, "pmd.wil.txrx" },
    [ WIL_DBG_LOG_WMI ] = { 0, "pmd.wil.wmi" },
    [ WIL_DBG_LOG_MISC ] = { 0, "pmd.wil.misc" },
    [ WIL_DBG_LOG_PM ] = { 0, "pmd.wil.pm" },
    [ WIL_DBG_LOG_IOC ] = { 0, "pmd.wil.ioc" },
    [ WIL_DBG_LOG_FW ] = { 0, "pmd.wil.fw" },
    [ WIL_DBG_LOG_UMAC ] = { 0, "pmd.wil.umac" },
};

RTE_INIT(wil_init_log);

#define WIL_DBG_LOG_LEVEL RTE_LOG_INFO

static void
wil_init_log(void)
{
	int i;

	for (i = 0; i < WIL_DBG_LOG_NUM; i++) {
		wil_dbg_log[i].log_type_idx = rte_log_register(
		    wil_dbg_log[i].log_type_str);
		if (wil_dbg_log[i].log_type_idx >= 0) {
			rte_log_set_level(wil_dbg_log[i].log_type_idx, RTE_LOG_ERR);
		}
	}
}

void wil_set_log_dbg_mask(u32 * mask) {
	int level, i;

	for (i = 0; i < WIL_DBG_LOG_NUM; i++) {
		level = rte_log_get_level(wil_dbg_log[i].log_type_idx);
		if (level >= WIL_DBG_LOG_LEVEL)
			*mask |= 1 << i; /* Enable this log */
	}
}

static inline int net_ratelimit(void)
{
	return 0;
}

static inline void _dbg_vprintf(int level, int ltype, const char *fmt, va_list args)
{
	rte_vlog(level, ltype, fmt, args);
}

void __dbg_printf(int level, int ltype, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	_dbg_vprintf(level, ltype, fmt, args);
	va_end(args);
}

void wil_dbg_trace(struct wil6210_priv *wil, int ltype, const char *fmt, ...)
{
	va_list args;

	if (ltype >= 0 && ltype < WIL_DBG_LOG_NUM &&
	    wil_dbg_log[ltype].log_type_idx >= 0) {
		va_start(args, fmt);
		_dbg_vprintf(WIL_DBG_LOG_LEVEL, wil_dbg_log[ltype].log_type_idx,
		    fmt, args);
		va_end(args);
	}
}

/*
 * Simplified hex dumping routine.
 * This routine is no match to the version in Linux kernel, but is enough
 * to make debuuging possible.
 */
static void
print_hex_dump(int level, int type, const char *prefix_str, int prefix_type,
    const void *buf, size_t len, bool ascii_)
{
	int i, j, linelen;
	char buffer[80];
	char temp[10];
	char *ascii, *hexdata;
	const uint8_t *data = buf;

	// Hex portion of the line is 10 (the padding) + 4 * 12 = 58 chars long
	// We add another two spaces padding and place the ASCII version...
	linelen = 48;
	hexdata = buffer;

	/* intentianly treat DUMP_PREFIX_ADDRESS as DUMP_PREFIX_OFFSET */
	if (prefix_type != DUMP_PREFIX_NONE) {
		linelen += 10;
		hexdata += 10;
	}

	if (ascii_) {
		ascii = buffer + linelen + 2;
		linelen += 16 + 2;
	}

	memset(buffer, ' ', linelen);
	buffer[linelen + 0] = '\n';
	buffer[linelen + 1] = '\0';

	if (prefix_type != DUMP_PREFIX_NONE) {
		snprintf(temp, 10, "%08x:", 0);
		memcpy(buffer, temp, 9);
	}

	for (i = 0, j = 0; i < len; i++, j++) {
		if (j == 16) {
			__dbg_printf(level, type, "%s%s", prefix_str, buffer);

			memset(buffer, ' ', linelen);
			if (prefix_type != DUMP_PREFIX_NONE) {
				snprintf(temp, sizeof(temp), "%08x:", i);
				memcpy(buffer, temp, 9);
			}
			j = 0;
		}

		snprintf(temp, 3, "%02x", data[i]);
		memcpy(hexdata + (j * 3), temp, 2);
		if (ascii_) {
		    if (data[i] > 31 && data[i] < 127)
			    ascii[j] = data[i];
		    else
			    ascii[j] = '.';
		}
	}

	if (j != 0)
		__dbg_printf(level, type, "%s%s", prefix_str, buffer);
}

void print_hex_dump_level(int level, const char *prefix_str, int prefix_type,
			  int rowsize, int groupsize,
			  const void *buf, size_t len, bool ascii_)
{
	print_hex_dump(level, RTE_LOGTYPE_PMD, prefix_str, prefix_type,
	    buf, len, ascii_);
}


void print_hex_dump_debug(int ltype, const char *prefix_str, int prefix_type,
			  int rowsize, int groupsize,
			  const void *buf, size_t len, bool ascii_)
{

	if (ltype >= 0 && ltype < WIL_DBG_LOG_NUM &&
	    wil_dbg_log[ltype].log_type_idx >= 0) {
	    print_hex_dump(WIL_DBG_LOG_LEVEL, wil_dbg_log[ltype].log_type_idx,
		    prefix_str, prefix_type, buf, len, ascii_);
	}
}
