/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
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

#ifndef __BACKPORT_H__
#define __BACKPORT_H__
#include <linux/version.h>

/* PART 1: define BACKPORT_XXXX macros based on kernel version macros
 * this is the only place where kernel version macros are allowed.
 * Use only BACKPORT_XXXX macros in the driver source code.
 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	#define BACKPORT_HAS_TIMER_SETUP
#else
	#undef BACKPORT_HAS_TIMER_SETUP
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0)
	#define BACKPORT_PROBE_RESP_HAS_ACK_SIGNAL
#else
	#undef BACKPORT_PROBE_RESP_HAS_ACK_SIGNAL
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	#define BACKPORT_HAS_SB_DEV_AND_FALLBACK
#else
	#undef BACKPORT_HAS_SB_DEV_AND_FALLBACK
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0)
	#undef BACKPORT_HAS_ZALLOC_COHERENT
#else
	#define BACKPORT_HAS_ZALLOC_COHERENT
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
	#define BACKPORT_HAS_ONLY_SB_DEV
#else
	#undef BACKPORT_HAS_ONLY_SB_DEV
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	#define BACKPORT_HAS_VENDOR_CMD_POLICY
#else
	#undef BACKPORT_HAS_VENDOR_CMD_POLICY
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
	#define BACKPORT_HAS_EDMG
#else
	#undef BACKPORT_HAS_EDMG
#endif

/* PART 2: Optionally include a platform specific header file
 * to override BACKPORT_XXXX macros.
 * This is useful when targeting an old kernel which backported
 * features from newer kernels.
 * Create file named backport_<id>.h
 * and pass <id> in the BACKPORT_OVERRIDE pre-processor macro
 */

#define __backport_header(x) #x
#define _backport_header(x) __backport_header(backport_##x.h)
#define backport_header(x) _backport_header(x)

#ifdef BACKPORT_OVERRIDE
#include backport_header(BACKPORT_OVERRIDE)
#endif /* BACKPORT_OVERRIDE */

/* PART 3: Further customize backports based on BACKPORT_XXXX macros.
 * Do not use kernel version macros here, only BACKPORT_XXXX macros.
 */

#endif /* __BACKPORT_H__ */
