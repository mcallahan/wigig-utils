/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019-2020, Facebook, Inc. All rights reserved.
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
#include <fcntl.h>
#include <float.h>
#include <math.h>
#include <unistd.h>
#include <poll.h>

#define WIL_FW_STR_BIN_FILE "fw_image_trace_string_load.bin"
#define WIL_UCODE_STR_BIN_FILE "ucode_image_trace_string_load.bin"

/* Little Endian formatted structure in device memory */
struct wil_fw_module_level_enable {
	uint error_level_enable : 1;
	uint warn_level_enable : 1;
	uint info_level_enable : 1;
	uint verbose_level_enable : 1;
	uint disable_new_line : 1;
	uint reserved0 : 3;
} __attribute__((packed));

/* Little Endian formatted structure in device memory */
struct wil_fw_log_trace_header {
	/* MSB of number of dwords following the header */
	uint params_dwords_msb : 2;
	/* the offset of the trace string in the strings sections */
	uint strring_offset : 18;
	/* module that outputs the trace */
	uint module : 4;
	/* 0 - Error, 1 - WARN, 2 - INFO, 3 - VERBOSE */
	uint level : 2;
	/* LSB of number of dwords following the header */
	uint params_dwords_lsb : 2;
	/* indicate if the printf uses %s */
	uint is_string : 1;
	/* should be 5 (2'101) in valid header */
	uint signature : 3;
} __attribute__((packed));

#define WIL_FW_HDR_SIGNATURE_MAGIC (5)

union wil_fw_log_event {
	struct wil_fw_log_trace_header hdr;
	u32 param;
} __attribute__((packed));

struct wil_fw_log_table_header {
	u32 write_ptr; /* incremented by trace producer every write */
	struct wil_fw_module_level_enable module_level_enable[16];
	union wil_fw_log_event evt[0];
} __attribute__((packed));

struct wil_fw_log_rgf_user_usage {
	uint offset : 29;
	uint log_size : 3;
};

static const size_t wil_fw_log_size_lut[] = {
	4 * 1024, 1 * 1024,  2 * 1024,  4 * 1024,
	8 * 1024, 16 * 1024, 32 * 1024, 64 * 1024,
};

typedef enum { fw_log_type, ucode_log_type } wil_log_t;
static const char *const log_type_strs[] = { "fw", "ucode" };

struct wil_fw_log_state {
	struct wil6210_priv *wil;

	void *log_buf; /* memory allocated for the log buf */
	char timestamp[25];
	char module_string[128];
	uint prev_module;
	char trailing_newline;
	char *str_buf;
	size_t str_sz;
	u32 rptr;
	u32 rptr_param_last;
	u32 wptr;

	size_t log_offset;
	size_t log_buf_entries;
	char *log_path;
	char str_path[WIL_FW_FILE_PATH_BUFSZ];
	wil_log_t log_type;
	FILE *fp;
};

enum { wil_fw_str_mask = 0xFFFFF };

/* string used in place of corrupted string parameter */
static const char *CORRUPTED_PARAM_MARK = "ZZZZ";

static const char *const levels[] = {
	"E",
	"W",
	"I",
	"V",
};

#define WIL_FW_MAX_PARAMS (7)

static const char *modules[16];

static inline size_t wil_fw_log_size(size_t entry_num)
{
	return sizeof(struct wil_fw_log_table_header) + entry_num * 4;
}

static size_t wil_fw_read_all(int f, char *buf, size_t n)
{
	size_t actual = 0, r;
	do {
		r = read(f, buf + actual, n - actual);
		actual += r;
	} while ((r > 0) && (actual < n));
	return actual;
}

static char
*wil_fw_read_strings(struct wil_fw_log_state *s, const char *name,
	size_t *size, size_t extra_space)
{
	int f = open(name, O_RDONLY);
	size_t sz = *size;
	size_t r;
	char *buf;
	struct wil6210_priv *wil = s->wil;

	wil_info(wil, "Using strings file %s for %s log printing\n", name,
		 log_type_strs[s->log_type]);

	if (f < 0) {
		wil_err(wil, "Error: unavailable strings file %s\n", name);
		return NULL;
	}

	if (!sz) {
		sz = lseek(f, 0, SEEK_END);
		lseek(f, 0, SEEK_SET);
	}
	buf = rte_zmalloc(NULL, sz + extra_space, 0);
	if (!buf) {
		wil_err(wil, "Error: unable to allocate string buffer %zd bytes\n",
			sz);
		return NULL;
	}
	r = wil_fw_read_all(f, buf, sz);
	close(f);
	if (r != sz) {
		wil_err(wil, "Error: from %s read %zd bytes out of %zd\n", name, r,
			sz);
		return NULL;
	}
	*size = sz;
	return buf;
}

static int
wil_fw_read_log(struct wil_fw_log_state *s, const char *fname)
{
	int f, rc;
	size_t r;
	void *data;
	struct wil6210_priv *wil = s->wil;
	struct wil_fw_log_table_header *h;
	u32 read_ptr = s->rptr;
	u32 write_ptr, offset;
	size_t off = s->log_offset;
	size_t log_buf_entries = s->log_buf_entries;
	size_t size = wil_fw_log_size(log_buf_entries);

	/* read from live device data instead of reading from a crash core dump */
	if (!fname) {
		rc = wil_mem_access_lock(wil);
		if (rc)
			return rc;
		data = (void *__force)wil->csr + off;

		/* copy just the header first */
		wil_memcpy_fromio_32((void *__force)(s->log_buf),
			(const void __iomem *__force)data,
			sizeof(struct wil_fw_log_table_header));

		/* calculate proper read pointer after read overrun/write overflow */
		h = s->log_buf;
		write_ptr = h->write_ptr;
		if (read_ptr > write_ptr) {
			read_ptr = write_ptr > log_buf_entries ?
				write_ptr - log_buf_entries : 0;
		}
		if ((write_ptr - read_ptr) >= log_buf_entries) {
			read_ptr = write_ptr - log_buf_entries;
		}
		if (write_ptr / log_buf_entries > read_ptr / log_buf_entries) {
			/* new portion of logs wraps, copy end of logs then beginning */
			offset = sizeof(struct wil_fw_log_table_header) +
				(read_ptr % log_buf_entries) * 4;
			wil_memcpy_fromio_32(
				(void *__force)(&h->evt[read_ptr % log_buf_entries]),
				(const void __iomem *__force)(data + offset),
				(log_buf_entries - read_ptr % log_buf_entries) * 4
			);
			offset = sizeof(struct wil_fw_log_table_header);
			wil_memcpy_fromio_32(
				(void *__force)(&h->evt[0]),
				(const void __iomem *__force)(data + offset),
				(write_ptr % log_buf_entries) * 4
			);
		} else {
			/* copy only new portion of logs, no wrapping */
			offset = sizeof(struct wil_fw_log_table_header) +
				(read_ptr % log_buf_entries) * 4;
			wil_memcpy_fromio_32(
				(void *__force)(&h->evt[read_ptr % log_buf_entries]),
				(const void __iomem *__force)(data + offset),
				(write_ptr - read_ptr) * 4
			);
		}

		wil_mem_access_unlock(wil);

		return 0;
	}

	/* copy data from core dump file into buffer */
	f = open(fname, O_RDONLY);
	if (f < 0) {
		wil_err(wil, "Error: memdump file %s\n", fname);
		return -1;
	}
	lseek(f, off, SEEK_SET);
	r = wil_fw_read_all(f, s->log_buf, size);

	close(f);
	if (r != size) {
		wil_err(wil, "Error: from %s read %zd bytes out of %zd\n", fname, r,
			size);
		return -1;
	}
	return 0;
}

static void wil_fw_logerr(struct wil_fw_log_state *s, FILE *f,
	const char *fmt, ...)
{
	va_list argp;

	if (!s->trailing_newline)
		putc('\n', f);
	fprintf(f, "%s[%6d/%6d] ", s->timestamp, s->rptr, s->wptr);
	va_start(argp, fmt);
	vfprintf(f, fmt, argp);
	va_end(argp);
	putc('\n', f);
	s->trailing_newline = 1;
}

static int wil_fw_print_trace(struct wil_fw_log_state *s,
	FILE *f, const char *fmt);

static void wil_fw_do_parse(struct wil_fw_log_state *s, FILE *f)
{
	union wil_fw_log_event evt;
	struct wil_fw_log_table_header *h = s->log_buf;
	size_t log_buf_entries = s->log_buf_entries;

	s->wptr = h->write_ptr;

	if (s->rptr > s->wptr) {
		wil_fw_logerr(s, f, "rptr overrun; try to rewind it back");
		s->rptr = s->wptr > log_buf_entries ?
			s->wptr - log_buf_entries : 0;
	}

	if ((s->wptr - s->rptr) >= log_buf_entries) {
		wil_fw_logerr(s, f, "wptr overflow; try to parse last wrap");
		s->rptr = s->wptr - log_buf_entries;
	}

	for (; s->rptr < s->wptr; s->rptr++) {
		const char *fmt;
		uint params_dwords;
		uint strring_offset;

		evt = h->evt[s->rptr % log_buf_entries];
		if (evt.hdr.signature != WIL_FW_HDR_SIGNATURE_MAGIC) {
			wil_fw_logerr(s, f, "Signature magic mismatch 0x%08x", evt.param);
			continue;
		}

		strring_offset = evt.hdr.strring_offset
				 << 2; /* promote to 20 bits */
		if (strring_offset > s->str_sz) {
			wil_fw_logerr(s, f, "String offset [%d/%lu] overflow",
			       strring_offset, s->str_sz);
			continue;
		}

		snprintf(s->module_string, sizeof(s->module_string),
			 "%s[%6u/%6u] %9s %s : ",
			 s->timestamp, s->rptr, s->wptr,
			 modules[evt.hdr.module],
			 levels[evt.hdr.level]);
		fmt = s->str_buf + strring_offset;
		params_dwords = evt.hdr.params_dwords_lsb << 0;
		params_dwords |= evt.hdr.params_dwords_msb << 2;

		if (params_dwords > 2 * WIL_FW_MAX_PARAMS) {
			wil_fw_logerr(s, f, "Params length (%d) exceeds max (%d) in %s",
			       params_dwords, WIL_FW_MAX_PARAMS, fmt);
			continue;
		}

		s->rptr_param_last = s->rptr + params_dwords;

		if (s->rptr_param_last > s->wptr) {
			wil_fw_logerr(s, f, "Params length (%d) overflows wptr (%d) in %s",
			       params_dwords, s->wptr, fmt);
			s->rptr_param_last = s->wptr;
		}

		s->rptr += 1;

		if (s->trailing_newline) {
			fputs(s->module_string, f);
			s->trailing_newline = 0;
		} else {
			/* Separate lines from different modules */
			if (s->prev_module != evt.hdr.module) {
				fprintf(f, "\n%s", s->module_string);
			} else if (strncmp("[NNL]", fmt, 5) == 0) {
				fmt += 5;
			} else if (!h->module_level_enable[s->prev_module]
					.disable_new_line) {
				fprintf(f, "\n%s", s->module_string);
			}
		}

		wil_fw_print_trace(s, f, fmt);
		s->prev_module = evt.hdr.module;

		if (s->rptr - 1 > s->rptr_param_last)
			wil_fw_logerr(s, f, "Params overflow (%d/%d)",
			       s->rptr - 1, s->rptr_param_last);
		else if (s->rptr - 1 != s->rptr_param_last)
			wil_fw_logerr(s, f, "Params underflow (%d/%d)",
			       s->rptr - 1, s->rptr_param_last);
		s->rptr = s->rptr_param_last;
	}

	fflush(f);
}

/* Start of vfprintf implementation taken from musl libc [1]
 * distributed under standard MIT license [2].
 *
 * [1] http://git.musl-libc.org/cgit/musl/tree/src/stdio/vfprintf.c
 * [2] http://git.musl-libc.org/cgit/musl/tree/COPYRIGHT
 */

/* Convenient bit representation for modifier flags,
 * which all fall within 31 codepoints of the space character.
 */
#define ALT_FORM (1U << ('#' - ' '))
#define ZERO_PAD (1U << ('0' - ' '))
#define LEFT_ADJ (1U << ('-' - ' '))
#define PAD_POS (1U << (' ' - ' '))
#define MARK_POS (1U << ('+' - ' '))
#define GROUPED (1U << ('\'' - ' '))

#define FLAGMASK (ALT_FORM | ZERO_PAD | LEFT_ADJ | PAD_POS | MARK_POS | GROUPED)

/* State machine to accept length modifiers + conversion specifiers.
 * Result is 0 on failure, or an argument type to pop on success.
 */
enum {
	BARE, LPRE, LLPRE, HPRE, HHPRE, BIGLPRE, ZTPRE, JPRE,
	STOP,
	PTR, INT, UINT, ULLONG, LONG, ULONG, SHORT, USHORT,
	CHAR, UCHAR, LLONG, SIZET, IMAX, UMAX, PDIFF, UIPTR,
	DBL, LDBL, NOARG, MAXSTATE
};

#define S(x)[(x) - 'A']

static const unsigned char states[]['z' - 'A' + 1] = {
	{
		/* 0: bare types */
		S('d') = INT,   S('i') = INT,  S('o') = UINT,  S('u') = UINT,
		S('x') = UINT,  S('X') = UINT, S('e') = DBL,   S('f') = DBL,
		S('g') = DBL,   S('a') = DBL,  S('E') = DBL,   S('F') = DBL,
		S('G') = DBL,   S('A') = DBL,  S('c') = CHAR,  S('C') = INT,
		S('s') = PTR,   S('S') = PTR,  S('p') = UIPTR, S('n') = PTR,
		S('m') = NOARG, S('l') = LPRE, S('h') = HPRE,  S('L') = BIGLPRE,
		S('z') = ZTPRE, S('j') = JPRE, S('t') = ZTPRE,
	},
	{
		/* 1: l-prefixed */
		S('d') = LONG,  S('i') = LONG,  S('o') = ULONG, S('u') = ULONG,
		S('x') = ULONG, S('X') = ULONG, S('e') = DBL,   S('f') = DBL,
		S('g') = DBL,   S('a') = DBL,   S('E') = DBL,   S('F') = DBL,
		S('G') = DBL,   S('A') = DBL,   S('c') = INT,   S('s') = PTR,
		S('n') = PTR,   S('l') = LLPRE,
	},
	{
		/* 2: ll-prefixed */
		S('d') = LLONG,  S('i') = LLONG, S('o') = ULLONG,
		S('u') = ULLONG, S('x') = ULLONG, S('X') = ULLONG,
		S('n') = PTR,
	},
	{
		/* 3: h-prefixed */
		S('d') = SHORT, S('i') = SHORT, S('o') = USHORT,
		S('u') = USHORT, S('x') = USHORT, S('X') = USHORT,
		S('n') = PTR, S('h') = HHPRE,
	},
	{
		/* 4: hh-prefixed */
		S('d') = CHAR, S('i') = CHAR, S('o') = UCHAR, S('u') = UCHAR,
		S('x') = UCHAR, S('X') = UCHAR, S('n') = PTR,
	},
	{
		/* 5: L-prefixed */
		S('e') = LDBL, S('f') = LDBL, S('g') = LDBL, S('a') = LDBL,
		S('E') = LDBL, S('F') = LDBL, S('G') = LDBL, S('A') = LDBL,
		S('n') = PTR,
	},
	{
		/* 6: z- or t-prefixed (assumed to be same size) */
		S('d') = PDIFF, S('i') = PDIFF, S('o') = SIZET, S('u') = SIZET,
		S('x') = SIZET, S('X') = SIZET, S('n') = PTR,
	},
	{
		/* 7: j-prefixed */
		S('d') = IMAX, S('i') = IMAX, S('o') = UMAX, S('u') = UMAX,
		S('x') = UMAX, S('X') = UMAX, S('n') = PTR,
	}
};

#define OOB(x) ((unsigned int)(x) - 'A' > 'z' - 'A')

union wil_fw_arg {
	uintmax_t i;
	double f;
	const void *p;
};

static union wil_fw_arg pop_arg(struct wil_fw_log_state *s, int type)
{
	struct wil_fw_log_table_header *h = s->log_buf;
	union wil_fw_arg arg = { .p = 0 };
	size_t log_buf_entries = s->log_buf_entries;

	switch (type) {
	case PTR:
		if (s->rptr > s->rptr_param_last ||
		    (h->evt[s->rptr % log_buf_entries].param & wil_fw_str_mask) >=
			    s->str_sz) {
			arg.p = CORRUPTED_PARAM_MARK;
			break;
		}

		arg.p = s->str_buf +
			(h->evt[s->rptr++ % log_buf_entries].param & wil_fw_str_mask);
		break;

	case CHAR:
	case SHORT:
	case INT:
	case LONG:
	case PDIFF:
		if (s->rptr > s->rptr_param_last) {
			arg.i = -1;
			break;
		}

		arg.i = (int32_t)h->evt[s->rptr++ % log_buf_entries].param;
		break;

	case UCHAR:
	case USHORT:
	case UINT:
	case ULONG:
	case UIPTR:
	case SIZET:
		if (s->rptr > s->rptr_param_last) {
			arg.i = -1;
			break;
		}

		arg.i = h->evt[s->rptr++ % log_buf_entries].param;
		break;

	case LLONG:
	case ULLONG:
	case IMAX:
	case UMAX:
		if (s->rptr + 1 > s->rptr_param_last) {
			arg.i = -1;
			break;
		}

		arg.i = (uintmax_t)h->evt[s->rptr++ % log_buf_entries].param << 0;
		arg.i |= (uintmax_t)h->evt[s->rptr++ % log_buf_entries].param
			 << 32;
		break;

	/* floating point types are not supported at the moment */
	case DBL:
	case LDBL:
		if (s->rptr + 1 > s->rptr_param_last) {
			arg.f = -1;
			break;
		}

		arg.f = -1.0;
		s->rptr += 2;
		break;
	}

	return arg;
}

static void out(FILE *f, const char *s, size_t l)
{
	if (!ferror(f))
		fwrite((void *)s, l, 1, f);
}

static void pad(FILE *f, char c, int w, int l, int fl)
{
	char pad[256];

	if (fl & (LEFT_ADJ | ZERO_PAD) || l >= w)
		return;
	l = w - l;
	memset(pad, c, (size_t)l > sizeof(pad) ? sizeof(pad) : (size_t)l);
	for (; (size_t)l >= sizeof(pad); l -= sizeof(pad))
		out(f, pad, sizeof(pad));
	out(f, pad, l);
}

static const char xdigits[16] = { "0123456789ABCDEF" };

static char *fmt_x(uintmax_t x, char *s, int lower)
{
	for (; x; x >>= 4)
		*--s = xdigits[(x & 15)] | lower;
	return s;
}

static char *fmt_o(uintmax_t x, char *s)
{
	for (; x; x >>= 3)
		*--s = '0' + (x & 7);
	return s;
}

static char *fmt_u(uintmax_t x, char *s)
{
	unsigned long y;

	for (; x > ULONG_MAX; x /= 10)
		*--s = '0' + x % 10;
	for (y = x; y; y /= 10)
		*--s = '0' + y % 10;
	return s;
}

static int fmt_fp(FILE *f, long double y, int w, int p, int fl, int t)
{
	uint32_t big[(LDBL_MANT_DIG + 28) / 29 + 1 // mantissa expansion
		     + (LDBL_MAX_EXP + LDBL_MANT_DIG + 28 + 8) /
			       9]; // exponent expansion
	uint32_t *a, *d, *r, *z;
	int e2 = 0, e, i, j, l;
	char buf[9 + LDBL_MANT_DIG / 4], *s;
	const char *prefix = "-0X+0X 0X-0x+0x 0x";
	int pl;
	char ebuf0[3 * sizeof(int)], *ebuf = &ebuf0[3 * sizeof(int)], *estr = 0;

	pl = 1;
	if (signbit(y))
		y = -y;
	else if (fl & MARK_POS)
		prefix += 3;
	else if (fl & PAD_POS)
		prefix += 6;
	else
		prefix++, pl = 0;

	if (!isfinite(y)) {
		char *s = (t & 32) ? "inf" : "INF";

		if (y != y)
			s = (t & 32) ? "nan" : "NAN";
		pad(f, ' ', w, 3 + pl, fl & ~ZERO_PAD);
		out(f, prefix, pl);
		out(f, s, 3);
		pad(f, ' ', w, 3 + pl, fl ^ LEFT_ADJ);
		return max(w, 3 + pl);
	}

	y = frexpl(y, &e2) * 2;
	if (y)
		e2--;

	if ((t | 32) == 'a') {
		long double round = 8.0;
		int re;

		if (t & 32)
			prefix += 9;
		pl += 2;

		if (p < 0 || p >= LDBL_MANT_DIG / 4 - 1)
			re = 0;
		else
			re = LDBL_MANT_DIG / 4 - 1 - p;

		if (re) {
			while (re--)
				round *= 16;
			if (*prefix == '-') {
				y = -y;
				y -= round;
				y += round;
				y = -y;
			} else {
				y += round;
				y -= round;
			}
		}

		estr = fmt_u(e2 < 0 ? -e2 : e2, ebuf);
		if (estr == ebuf)
			*--estr = '0';
		*--estr = (e2 < 0 ? '-' : '+');
		*--estr = t + ('p' - 'a');

		s = buf;
		do {
			int x = y;

			*s++ = xdigits[x] | (t & 32);
			y = 16 * (y - x);
			if (s - buf == 1 && (y || p > 0 || (fl & ALT_FORM)))
				*s++ = '.';
		} while (y);

		if (p > INT_MAX - 2 - (ebuf - estr) - pl)
			return -1;
		if (p && s - buf - 2 < p)
			l = (p + 2) + (ebuf - estr);
		else
			l = (s - buf) + (ebuf - estr);

		pad(f, ' ', w, pl + l, fl);
		out(f, prefix, pl);
		pad(f, '0', w, pl + l, fl ^ ZERO_PAD);
		out(f, buf, s - buf);
		pad(f, '0', l - (ebuf - estr) - (s - buf), 0, 0);
		out(f, estr, ebuf - estr);
		pad(f, ' ', w, pl + l, fl ^ LEFT_ADJ);
		return max(w, pl + l);
	}
	if (p < 0)
		p = 6;

	if (y)
		y *= 0x1p28, e2 -= 28;

	if (e2 < 0) {
		a = big;
		r = a;
		z = a;
	}
	else {
		a = big + sizeof(big) / sizeof(*big) - LDBL_MANT_DIG - 1;
		r = a;
		z = a;
	}

	do {
		*z = y;
		y = 1000000000 * (y - *z++);
	} while (y);

	while (e2 > 0) {
		uint32_t carry = 0;
		int sh = min(29, e2);

		for (d = z - 1; d >= a; d--) {
			uint64_t x = ((uint64_t)*d << sh) + carry;

			*d = x % 1000000000;
			carry = x / 1000000000;
		}
		if (carry)
			*--a = carry;
		while (z > a && !z[-1])
			z--;
		e2 -= sh;
	}
	while (e2 < 0) {
		uint32_t carry = 0, *b;
		int sh = min(9, -e2);
		int need = 1 + (p + LDBL_MANT_DIG / 3U + 8) / 9;

		for (d = a; d < z; d++) {
			uint32_t rm = *d & ((1 << sh) - 1);

			*d = (*d >> sh) + carry;
			carry = (1000000000 >> sh) * rm;
		}
		if (!*a)
			a++;
		if (carry)
			*z++ = carry;
		/* Avoid (slow!) computation past requested precision */
		b = (t | 32) == 'f' ? r : a;
		if (z - b > need)
			z = b + need;
		e2 += sh;
	}

	if (a < z)
		for (i = 10, e = 9 * (r - a); *a >= (uint32_t)i; i *= 10, e++)
			;
	else
		e = 0;

	/* Perform rounding: j is precision after the radix (possibly neg) */
	j = p - ((t | 32) != 'f') * e - ((t | 32) == 'g' && p);
	if (j < 9 * (z - r - 1)) {
		uint32_t x;
		/* We avoid C's broken division of negative numbers */
		d = r + 1 + ((j + 9 * LDBL_MAX_EXP) / 9 - LDBL_MAX_EXP);
		j += 9 * LDBL_MAX_EXP;
		j %= 9;
		for (i = 10, j++; j < 9; i *= 10, j++)
			;
		x = *d % i;
		/* Are there any significant digits past j? */
		if (x || d + 1 != z) {
			long double round = 2 / LDBL_EPSILON;
			long double small;

			if ((*d / i & 1) ||
			    (i == 1000000000 && d > a && (d[-1] & 1)))
				round += 2;
			if (x < (uint32_t)i / 2)
				small = 0x0.8p0;
			else if (x == (uint32_t)i / 2 && d + 1 == z)
				small = 0x1.0p0;
			else
				small = 0x1.8p0;
			if (pl && *prefix == '-')
				round *= -1, small *= -1;
			*d -= x;
			/* Decide whether to round by probing round+small */
			if (round + small != round) {
				*d = *d + i;
				while (*d > 999999999) {
					*d-- = 0;
					if (d < a)
						*--a = 0;
					(*d)++;
				}
				for (i = 10, e = 9 * (r - a); *a >= (uint32_t)i;
				     i *= 10, e++)
					;
			}
		}
		if (z > d + 1)
			z = d + 1;
	}
	for (; z > a && !z[-1]; z--)
		;

	if ((t | 32) == 'g') {
		if (!p)
			p++;
		if (p > e && e >= -4) {
			t--;
			p -= e + 1;
		} else {
			t -= 2;
			p--;
		}
		if (!(fl & ALT_FORM)) {
			/* Count trailing zeros in last place */
			if (z > a && z[-1])
				for (i = 10, j = 0; z[-1] % i == 0;
				     i *= 10, j++)
					;
			else
				j = 9;
			if ((t | 32) == 'f')
				p = min(p, max(0, 9 * (z - r - 1) - j));
			else
				p = min(p, max(0, 9 * (z - r - 1) + e - j));
		}
	}
	if (p > INT_MAX - 1 - (p || (fl & ALT_FORM)))
		return -1;
	l = 1 + p + (p || (fl & ALT_FORM));
	if ((t | 32) == 'f') {
		if (e > INT_MAX - l)
			return -1;
		if (e > 0)
			l += e;
	} else {
		estr = fmt_u(e < 0 ? -e : e, ebuf);
		while (ebuf - estr < 2)
			*--estr = '0';
		*--estr = (e < 0 ? '-' : '+');
		*--estr = t;
		if (ebuf - estr > INT_MAX - l)
			return -1;
		l += ebuf - estr;
	}

	if (l > INT_MAX - pl)
		return -1;
	pad(f, ' ', w, pl + l, fl);
	out(f, prefix, pl);
	pad(f, '0', w, pl + l, fl ^ ZERO_PAD);

	if ((t | 32) == 'f') {
		if (a > r)
			a = r;
		for (d = a; d <= r; d++) {
			char *s = fmt_u(*d, buf + 9);

			if (d != a)
				while (s > buf)
					*--s = '0';
			else if (s == buf + 9)
				*--s = '0';
			out(f, s, buf + 9 - s);
		}
		if (p || (fl & ALT_FORM))
			out(f, ".", 1);
		for (; d < z && p > 0; d++, p -= 9) {
			char *s = fmt_u(*d, buf + 9);

			while (s > buf)
				*--s = '0';
			out(f, s, min(9, p));
		}
		pad(f, '0', p + 9, 9, 0);
	} else {
		if (z <= a)
			z = a + 1;
		for (d = a; d < z && p >= 0; d++) {
			char *s = fmt_u(*d, buf + 9);

			if (s == buf + 9)
				*--s = '0';
			if (d != a) {
				while (s > buf)
					*--s = '0';
			} else {
				out(f, s++, 1);
				if (p > 0 || (fl & ALT_FORM))
					out(f, ".", 1);
			}
			out(f, s, min(buf + 9 - s, p));
			p -= buf + 9 - s;
		}
		pad(f, '0', p + 18, 18, 0);
		out(f, estr, ebuf - estr);
	}

	pad(f, ' ', w, pl + l, fl ^ LEFT_ADJ);

	return max(w, pl + l);
}

static int getint(char **s)
{
	int i;

	for (i = 0; isdigit(**s); (*s)++) {
		if ((uint)i > INT_MAX / 10U || **s - '0' > INT_MAX - 10 * i)
			i = -1;
		else
			i = 10 * i + (**s - '0');
	}
	return i;
}

static size_t string_length(const char *s, size_t n)
{
	size_t len = 0;

	if (!s)
		return 0;

	while (n-- > 0 && *s++)
		len++;
	return len;
}

static int
printf_core(struct wil_fw_log_state *log_s, FILE *f,
	const char *fmt, union wil_fw_arg *nl_arg, int *nl_type)
{
	char *a, *z, *s = (char *)fmt;
	unsigned int l10n = 0, fl;
	int w, p, xp;
	union wil_fw_arg arg = { .p = 0 };
	int argpos;
	unsigned int st, ps;
	int cnt = 0, l = 0;
	size_t i;
	char buf[sizeof(uintmax_t) * 3 + 3 + LDBL_MANT_DIG / 4];
	const char *prefix;
	int t, pl;
	wchar_t wc[2];
	const wchar_t *ws;
	char mb[4];

	for (;;) {
		/* This error is only specified for snprintf, but since it's
		 * unspecified for other forms, do the same. Stop immediately
		 * on overflow; otherwise %n could produce wrong results.
		 */
		if (l > INT_MAX - cnt)
			goto overflow;

		/* Consume newlines */
		for (; *s && *s == '\n'; s++) {
			if (f) {
				putc('\n', f);

				if (s[1])
					fputs(log_s->module_string, f);
				else
					log_s->trailing_newline = 1;
			}

			l++;
		}

		/* Update output count, end loop when fmt is exhausted */
		cnt += l;
		if (!*s)
			break;

		/* Handle literal text and %% format specifiers */
		for (a = s; *s && *s != '%' && *s != '\n'; s++)
			;
		for (z = s; s[0] == '%' && s[1] == '%'; z++, s += 2)
			;
		if (z - a > INT_MAX - cnt)
			goto overflow;
		l = z - a;
		if (f)
			out(f, a, l);
		if (l)
			continue;

		if (isdigit(s[1]) && s[2] == '$') {
			l10n = 1;
			argpos = s[1] - '0';
			s += 3;
		} else {
			argpos = -1;
			s++;
		}

		/* Read modifier flags */
		for (fl = 0;
		     (unsigned int)*s - ' ' < 32 &&
			 (FLAGMASK & (1U << (*s - ' ')));
		     s++)
			fl |= 1U << (*s - ' ');

		/* Read field width */
		if (*s == '*') {
			if (isdigit(s[1]) && s[2] == '$') {
				l10n = 1;
				nl_type[s[1] - '0'] = INT;
				w = nl_arg[s[1] - '0'].i;
				s += 3;
			} else if (!l10n) {
				w = f ? pop_arg(log_s, INT).i : 0;
				s++;
			} else {
				goto inval;
			}
			if (w < 0)
				fl |= LEFT_ADJ, w = -w;
		} else {
			w = getint(&s);
			if (w < 0)
				goto overflow;
		}

		/* Read precision */
		if (*s == '.' && s[1] == '*') {
			if (isdigit(s[2]) && s[3] == '$') {
				nl_type[s[2] - '0'] = INT;
				p = nl_arg[s[2] - '0'].i;
				s += 4;
			} else if (!l10n) {
				p = f ? pop_arg(log_s, INT).i : 0;
				s += 2;
			} else {
				goto inval;
			}
			xp = (p >= 0);
		} else if (*s == '.') {
			s++;
			p = getint(&s);
			xp = 1;
		} else {
			p = -1;
			xp = 0;
		}

		/* Format specifier state machine */
		st = 0;
		do {
			if (OOB(*s))
				goto inval;
			ps = st;
			st = states[st] S(*s++);
		} while (st - 1 < STOP);
		if (!st)
			goto inval;

		/* Check validity of argument type (nl/normal) */
		if (st == NOARG) {
			if (argpos >= 0)
				goto inval;
		} else {
			if (argpos >= 0)
				nl_type[argpos] = st, arg = nl_arg[argpos];
			else if (f)
				arg = pop_arg(log_s, st);
			else
				return 0;
		}

		if (!f)
			continue;

		z = buf + sizeof(buf);
		prefix = "-+   0X0x";
		pl = 0;
		t = s[-1];

		/* Transform ls,lc -> S,C */
		if (ps && (t & 15) == 3)
			t &= ~32;

		/* - and 0 flags are mutually exclusive */
		if (fl & LEFT_ADJ)
			fl &= ~ZERO_PAD;

		switch (t) {
		case 'n':
			switch (ps) {
			case BARE:
				*(int *)arg.p = cnt;
				break;
			case LPRE:
				*(long *)arg.p = cnt;
				break;
			case LLPRE:
				*(long long *)arg.p = cnt;
				break;
			case HPRE:
				*(unsigned short *)arg.p = cnt;
				break;
			case HHPRE:
				*(unsigned char *)arg.p = cnt;
				break;
			case ZTPRE:
				*(size_t *)arg.p = cnt;
				break;
			case JPRE:
				*(uintmax_t *)arg.p = cnt;
				break;
			}
			continue;
		case 'p':
			p = max((size_t)p, 2 * sizeof(void *));
			t = 'x';
			fl |= ALT_FORM;
		case 'x':
		case 'X':
			a = fmt_x(arg.i, z, t & 32);
			if (arg.i && (fl & ALT_FORM))
				prefix += (t >> 4), pl = 2;
			if (0) {
			case 'o':
				a = fmt_o(arg.i, z);
				if ((fl & ALT_FORM) && p < z - a + 1)
					p = z - a + 1;
			}
			if (0) {
			case 'd':
			case 'i':
				pl = 1;
				if (arg.i > INTMAX_MAX)
					arg.i = -arg.i;
				else if (fl & MARK_POS)
					prefix++;
				else if (fl & PAD_POS)
					prefix += 2;
				else
					pl = 0;
			case 'u':
				a = fmt_u(arg.i, z);
			}
			if (xp && p < 0)
				goto overflow;
			if (xp)
				fl &= ~ZERO_PAD;
			if (!arg.i && !p) {
				a = z;
				break;
			}
			p = max(p, z - a + !arg.i);
			break;
		case 'c':
			*(a = z - (p = 1)) = arg.i;
			fl &= ~ZERO_PAD;
			break;
		case 'm':
			if (1)
				a = strerror(errno);
			else
		case 's':
				a = (char *)(arg.p ? arg.p : "(null)");
			z = a + string_length(a, p < 0 ? INT_MAX : p);
			if (p < 0 && *z)
				goto overflow;
			p = z - a;
			fl &= ~ZERO_PAD;
			break;
		case 'C':
			wc[0] = arg.i;
			wc[1] = 0;
			arg.p = wc;
			p = -1;
		case 'S':
			ws = arg.p;
			for (i = l = 0;
			     i < (size_t)p &&
				 *ws &&
				 (l = wctomb(mb, *ws++)) >= 0 &&
			     l <= p - (int)i;
			     i += l)
				;
			if (l < 0)
				return -1;
			if (i > INT_MAX)
				goto overflow;
			p = i;
			pad(f, ' ', w, p, fl);
			ws = arg.p;
			for (i = 0;
				 i < 0U + p &&
				 *ws &&
				 i + (l = wctomb(mb, *ws++)) <= (size_t)p;
			     i += l)
				out(f, mb, l);
			pad(f, ' ', w, p, fl ^ LEFT_ADJ);
			l = w > p ? w : p;
			continue;
		case 'e':
		case 'f':
		case 'g':
		case 'a':
		case 'E':
		case 'F':
		case 'G':
		case 'A':
			if (xp && p < 0)
				goto overflow;
			l = fmt_fp(f, arg.f, w, p, fl, t);
			if (l < 0)
				goto overflow;
			continue;
		}

		if (p < z - a)
			p = z - a;
		if (p > INT_MAX - pl)
			goto overflow;
		if (w < pl + p)
			w = pl + p;
		if (w > INT_MAX - cnt)
			goto overflow;

		pad(f, ' ', w, pl + p, fl);
		out(f, prefix, pl);
		pad(f, '0', w, pl + p, fl ^ ZERO_PAD);
		pad(f, '0', p, z - a, 0);
		out(f, a, z - a);
		pad(f, ' ', w, pl + p, fl ^ LEFT_ADJ);

		l = w;
	}

	if (f)
		return cnt;
	if (!l10n)
		return 0;

	for (i = 1; i <= WIL_FW_MAX_PARAMS && nl_type[i]; i++)
		nl_arg[i] = pop_arg(log_s, nl_type[i]);

	for (; i <= WIL_FW_MAX_PARAMS && !nl_type[i]; i++)
		;

	if (i <= WIL_FW_MAX_PARAMS)
		goto inval;

	return 1;

inval:
	errno = EINVAL;
	return -1;
overflow:
	errno = EOVERFLOW;
	return -1;
}

/* End of vfprintf implementation
 */

static int wil_fw_print_trace(struct wil_fw_log_state *s,
	FILE *f, const char *fmt)
{
	int ret;
	int nl_type[WIL_FW_MAX_PARAMS + 1] = { 0 };
	union wil_fw_arg nl_arg[WIL_FW_MAX_PARAMS + 1];

	if (printf_core(s, 0, fmt, nl_arg, nl_type) < 0)
		return -1;

	ret = printf_core(s, f, fmt, nl_arg, nl_type);
	if (ferror(f))
		ret = -1;
	return ret;
}

static const char *wil_fw_next_mod(const char *mod)
{
	mod = strchr(mod, '\0') + 1;
	while (*mod == '\0')
		mod++;
	return mod;
}

static int
wil_fw_start_log(struct wil_fw_log_state *s, FILE *fp, char *fw_core_dump_path)
{
	const char *mod;
	size_t extra_space;
	int i, ret;
	struct wil6210_priv *wil = s->wil;
	size_t offset = s->log_offset;
	size_t log_buf_entries = s->log_buf_entries;

	fprintf(fp, "memdump file: <%s> offset: 0x%zx entries: %zd"
		" strings: <%s>\n",
		fw_core_dump_path, offset, log_buf_entries, s->str_path);

	/* reserve extra space at the end of string buffer for a special
	 * string that we write to the log when we encounter corrupted
	 * offsets for string parameters. This can happen when log is
	 * wrapped in the middle of a log message
	 */
	extra_space = strlen(CORRUPTED_PARAM_MARK) + 1;
	s->str_buf = wil_fw_read_strings(s, s->str_path, &s->str_sz,
		extra_space);
	if (!s->str_buf) {
		return -1;
	}
	mod = s->str_buf;
	snprintf(s->str_buf + s->str_sz, extra_space, "%s",
		CORRUPTED_PARAM_MARK);
	s->log_buf = rte_zmalloc(NULL, wil_fw_log_size(log_buf_entries), 0);
	if (!s->log_buf) {
		wil_err(wil, "Error: Unable to allocate log buffer %zd bytes\n",
			wil_fw_log_size(log_buf_entries));
		return -1;
	}
	struct wil_fw_log_table_header *h = s->log_buf;

	ret = wil_fw_read_log(s, fw_core_dump_path);
	if (ret) {
		return -1;
	}

	s->wptr = h->write_ptr;

	if ((s->wptr - s->rptr) >= log_buf_entries) {
		/* overflow; try to parse last wrap */
		s->rptr = s->wptr - log_buf_entries;
	}

	fprintf(fp, "  wptr = %u rptr = %u\n", s->wptr, s->rptr);
	for (i = 0; i < 16; i++, mod = wil_fw_next_mod(mod)) {
		modules[i] = mod;
		struct wil_fw_module_level_enable *m = &h->module_level_enable[i];

		fprintf(fp, "  %s[%2d] : %s%s%s%s%s\n", modules[i], i,
		       m->error_level_enable ? "E" : " ",
		       m->warn_level_enable ? "W" : " ",
		       m->info_level_enable ? "I" : " ",
		       m->verbose_level_enable ? "V" : " ",
		       m->disable_new_line ? "n" : " ");
	}
	return 0;
}

static int
wil_fw_start_opaque_log(struct wil_fw_log_state *s, FILE *f)
{
	int ret;
	size_t log_buf_entries = s->log_buf_entries;

	s->log_buf = rte_zmalloc(NULL, wil_fw_log_size(log_buf_entries), 0);
	if (!s->log_buf) {
		wil_err(s->wil, "Error: Unable to allocate log buffer %zd bytes\n",
			wil_fw_log_size(log_buf_entries));
		return -1;
	}

	ret = wil_fw_read_log(s, NULL);
	if (ret) {
		return -1;
	}

	ret = fwrite((void *__force)(s->log_buf), 1,
		sizeof(struct wil_fw_log_table_header), f);

	if (ret < sizeof(struct wil_fw_log_table_header)) {
		return -1;
	}
	return 0;
}

static void wil_fw_copy_opaque_log(struct wil_fw_log_state *s, FILE *f)
{
	struct wil_fw_log_table_header *h = s->log_buf;
	size_t log_buf_entries = s->log_buf_entries;

	s->wptr = h->write_ptr;
	if (s->rptr > s->wptr) {
		s->rptr = s->wptr > log_buf_entries ?
			s->wptr  - log_buf_entries : 0;
	}
	if ((s->wptr - s->rptr) >= log_buf_entries) {
		s->rptr = s->wptr - log_buf_entries;
	}
	if (s->wptr / log_buf_entries > s->rptr / log_buf_entries) {
		fwrite((void *__force)(&h->evt[s->rptr % log_buf_entries]),
			4, log_buf_entries - s->rptr % log_buf_entries, f);
		fwrite((void *__force)(&h->evt[0]),
			4, s->wptr % log_buf_entries, f);
	} else {
		fwrite((void *__force)(&h->evt[s->rptr % log_buf_entries]),
			4, s->wptr - s->rptr, f);
	}
	fflush(f);

	s->rptr = s->wptr;
}

static void wil_fw_log_state_set_str_path(struct wil_fw_log_state *s);

int
wil_fw_print_dump_logs(struct wil6210_priv *wil, char *fw_core_dump_path,
	bool only_logs)
{
	struct wil_fw_log_state log_s = {0};
	FILE *fp;
	char fw_trace_path[WIL_FW_FILE_PATH_BUFSZ];

	log_s.wil = wil;
	log_s.trailing_newline = 1;
	log_s.log_offset = only_logs ? 0 : wil->fw_log_offset;
	log_s.log_buf_entries = wil->fw_log_buf_entries;
	log_s.log_type = fw_log_type;
	wil_fw_log_state_set_str_path(&log_s);

	snprintf(fw_trace_path, WIL_FW_FILE_PATH_BUFSZ, "%s_trace.log",
		fw_core_dump_path);

	fp = fopen(fw_trace_path, "w+");
	if (!fp) {
		wil_err(wil, "error creating firmware trace file %s\n", fw_trace_path);
		return -1;
	}

	if (wil_fw_start_log(&log_s, fp, fw_core_dump_path)) {
		wil_err(wil, "error starting firmware log printing\n");
		fclose(fp);
		return -1;
	}

	wil_fw_do_parse(&log_s, fp);

	fprintf(fp, "\n");
	fclose(fp);

	wil_info(wil, "Firmware trace written to %s\n", fw_trace_path);
	return 0;
}

static void
wil_fw_log_state_set_str_path(struct wil_fw_log_state *s)
{
	char fw_path_param[WIL_FW_FILE_PATH_BUFSZ];
	struct wil6210_priv *wil = s->wil;
	int ret;

	if (s->log_type == ucode_log_type) {
		/* check if devarg for strings bin was set otherwise use default */
		if (strlen(wil->ucode_str_path) > 0) {
			strlcpy(s->str_path, wil->ucode_str_path,
				WIL_FW_FILE_PATH_BUFSZ);
		} else {
			/* search for file in custom fw search path */
			read_custom_fw_path(fw_path_param,
					    WIL_FW_FILE_PATH_BUFSZ);

			ret = snprintf(s->str_path, WIL_FW_FILE_PATH_BUFSZ,
				       "%s/%s", fw_path_param,
				       WIL_UCODE_STR_BIN_FILE);
			if (ret >= WIL_FW_FILE_PATH_BUFSZ) {
				wil_err(wil,
					"truncated ucode strings bin file name in custom path\n");
			}
			/* use default fw path if not found in custom fw search path */
			if (fw_path_param[0] == '\0' ||
			    access(s->str_path, F_OK) == -1) {
				ret = snprintf(s->str_path,
					       WIL_FW_FILE_PATH_BUFSZ, "%s/%s",
					       DEFAULT_FW_PATH,
					       WIL_UCODE_STR_BIN_FILE);
				if (ret >= WIL_FW_FILE_PATH_BUFSZ) {
					wil_err(wil,
						"truncated ucode strings bin file name in default path\n");
				}
			}
		}
	} else if (s->log_type == fw_log_type) {
		/* check if devarg for strings bin was set otherwise use default */
		if (strlen(wil->fw_str_path) > 0) {
			strlcpy(s->str_path, wil->fw_str_path,
				WIL_FW_FILE_PATH_BUFSZ);
		} else {
			/* search for file in custom fw search path */
			read_custom_fw_path(fw_path_param,
					    WIL_FW_FILE_PATH_BUFSZ);

			ret = snprintf(s->str_path, WIL_FW_FILE_PATH_BUFSZ,
				       "%s/%s", fw_path_param,
				       WIL_FW_STR_BIN_FILE);
			if (ret >= WIL_FW_FILE_PATH_BUFSZ) {
				wil_err(wil,
					"truncated fw strings bin file name in custom path\n");
			}
			/* use default fw path if not found in custom fw search path */
			if (fw_path_param[0] == '\0' ||
			    access(s->str_path, F_OK) == -1) {
				ret = snprintf(s->str_path,
					       WIL_FW_FILE_PATH_BUFSZ, "%s/%s",
					       DEFAULT_FW_PATH,
					       WIL_FW_STR_BIN_FILE);
				if (ret >= WIL_FW_FILE_PATH_BUFSZ) {
					wil_err(wil,
						"truncated fw strings bin file name in default path\n");
				}
			}
		}
	}
}

static void *
wil_fw_log_state_init(struct wil6210_priv *wil, wil_log_t log_type)
{
	struct wil_fw_log_state *s;

	s = rte_zmalloc("wil_fw_log_state", sizeof(*s), 0);
	if (!s) {
		return NULL;
	}

	s->wil = wil;
	s->trailing_newline = 1;
	s->log_type = log_type;

	if (log_type == ucode_log_type) {
		s->log_path = wil->ucode_log_path;
	} else if (log_type == fw_log_type) {
		s->log_path = wil->fw_log_path;
	}
	wil_fw_log_state_set_str_path(s);

	return s;
}

void wil_fw_log_polling_init(struct wil6210_priv *wil)
{
	if (strlen(wil->fw_log_path) > 0) {
		wil->fw_log_state = wil_fw_log_state_init(wil, fw_log_type);
		if (!wil->fw_log_state) {
			wil_err(wil, "fw log state init failed\n");
			wil->fw_log_path[0] = '\0';
			wil->fw_log_state = NULL;
		}
	} else {
		wil->fw_log_state = NULL;
	}

	if (strlen(wil->ucode_log_path) > 0) {
		wil->ucode_log_state = wil_fw_log_state_init(wil, ucode_log_type);
		if (!wil->ucode_log_state) {
			wil_err(wil, "ucode log state init failed\n");
			wil->ucode_log_path[0] = '\0';
			wil->ucode_log_state = NULL;
		}
	} else {
		wil->ucode_log_state = NULL;
	}
}

static int wil_fw_log_poll(struct wil6210_priv *wil)
{
	struct pollfd pfd;
	int i, rc;
	struct wil_fw_log_state *log_states[2];
	struct wil_fw_log_state *s;

	log_states[0] = wil->fw_log_state;
	log_states[1] = wil->ucode_log_state;

	/* shutdown pipe pollfd */
	pfd.fd = wil->log_pipe[0];
	pfd.events = POLLIN;

	/* if no shutdown from pipe before timeout, read log (every 1 second) */
	rc = poll(&pfd, 1, 1000);
	if (rc <= 0) {
		for (i = 0; i < 2; i++) {
			s = log_states[i];
			if (!s)
				continue;

			wil_fw_read_log(s, NULL);

			if (wil->opaque_log) {
				wil_fw_copy_opaque_log(s, s->fp);
			} else {
				wil_fw_do_parse(s, s->fp);
			}
			return 0;
		}
	}

	/* shutdown from pipe */
	if (pfd.revents & POLLIN) {
		return 1;
	}

	return 1;
}

static void *wil_fw_log_poll_worker_thread(void *arg)
{
	struct wil6210_priv *wil = arg;
	FILE *fp[2];
	struct wil_fw_log_state *log_states[2];
	struct wil_fw_log_state *s;
	int i;

	log_states[0] = wil->fw_log_state;
	log_states[1] = wil->ucode_log_state;

	for (i = 0; i < 2; i++) {
		s = log_states[i];
		if (!s)
			continue;

		if (s->log_type == ucode_log_type) {
			s->log_offset = wil->ucode_log_offset;
			s->log_buf_entries = wil->ucode_log_buf_entries;
		} else if (s->log_type == fw_log_type) {
			s->log_offset = wil->fw_log_offset;
			s->log_buf_entries = wil->fw_log_buf_entries;
		}

		/* use fw log path specified by devargs */
		if (wil->opaque_log) {
			/* overwrite if opaque log; cannot read more than one header */
			s->fp = fopen(s->log_path, "w+");
		} else {
			/* human readable logs, just append to previous logs */
			s->fp = fopen(s->log_path, "a+");
		}
		if (!fp) {
			wil_err(wil, "Error creating polling log file %s\n",
				s->log_path);
			return NULL;
		}

		wil_info(wil,
			"Writing to polling log file %s for %s logs, opaque: %d\n",
			s->log_path, log_type_strs[s->log_type], wil->opaque_log);

		if (wil->opaque_log) {
			if (wil_fw_start_opaque_log(s, s->fp)) {
				wil_err(wil,
					"Error starting opaque log writing for log file %s\n",
					s->log_path);
				return NULL;
			}
			wil_fw_copy_opaque_log(s, s->fp);
		} else {
			if (wil_fw_start_log(s, s->fp, NULL)) {
				wil_err(wil,
					"Error starting log printing for log file %s\n",
					s->log_path);
				return NULL;
			};
			wil_fw_do_parse(s, s->fp);
		}
	}

	for(;;) {
		if (wil_fw_log_poll(wil) != 0)
			break;
	}

	for (i = 0; i < 2; i++) {
		s = log_states[i];
		if (!log_states[i])
			continue;
		if (!wil->opaque_log)
			fprintf(s->fp, "\n");
		fclose(s->fp);
	}

	return NULL;
}

void wil_fw_log_stop_poll_worker(struct wil6210_priv *wil)
{
	int i, rc;
	char tmp;

	if (wil->log_pipe[1] >= 0) {
		/* Wake up worker and tell it to quit */
		do {
			rc = write(wil->log_pipe[1], &tmp, sizeof(tmp));
		} while (rc == -1 && (errno == EAGAIN || errno == EINTR));
		/* Wait for worker to exit */
		pthread_join(wil->log_poll_thread, NULL);
	}

	for (i = 0; i < 2; i++) {
		if (wil->log_pipe[i] >= 0) {
			close(wil->log_pipe[i]);
			wil->log_pipe[i] = -1;
		}
	}
}

int wil_fw_log_start_poll_worker(struct wil6210_priv *wil)
{
	int rc, i;
	rte_cpuset_t cpuset;

	/* pipe used for telling poller thread to shut down */
	rc = pipe2(wil->log_pipe, O_NONBLOCK | O_CLOEXEC);
	if (rc == -1) {
		wil_err(wil, "Failed to create fw/ucode log poller pipes\n");
		return rc;
	}

	rc = pthread_create(&wil->log_poll_thread, NULL,
		wil_fw_log_poll_worker_thread, wil);
	if (rc != 0) {
		wil_err(wil, "Failed to create fw/ucode log poller thread\n");
		goto close_pipe;
	}

	/* Run control threads on master lcore and set its name */
	CPU_ZERO(&cpuset);
	CPU_SET(rte_get_master_lcore(), &cpuset);
	rc = pthread_setaffinity_np(wil->log_poll_thread, sizeof(cpuset),
		&cpuset);
	if (rc != 0) {
		wil_err(wil,
			"Unable to set fw/ucode log poller thread affinity: %s\n",
			strerror(errno));
		wil_fw_log_stop_poll_worker(wil);
		return rc;
	}

	/* This function is free to fail */
	(void)rte_thread_setname(wil->log_poll_thread, "wil-fw-log-poll");

	return 0;

close_pipe:
	for (i = 0; i < 2; i++) {
		if (wil->log_pipe[i] >= 0) {
			close(wil->log_pipe[i]);
			wil->log_pipe[i] = -1;
		}
	}
}

void
wil_fw_not_ready_logs(struct wil6210_priv *wil)
{
	int rc;
	void *data;
	u32 write_ptr;

	wil_info(
		wil,
		"Attempting to get firmware logs since firmware was not ready\n");
	rc = wil_mem_access_lock(wil);
	if (rc) {
		wil_err(wil,
			"Memory access error, not fetching firmware logs\n");
		return;
	}

	/* copy log header's write pointer and check if it is a sensible value */
	data = (void *__force)wil->csr + wil->fw_log_offset;
	wil_memcpy_fromio_32((void *)&write_ptr,
			     (const void __iomem *__force)data,
			     sizeof(write_ptr));
	wil_mem_access_unlock(wil);

	/* log should not be empty, write_ptr should not be too large already */
	if (write_ptr == 0 || write_ptr > wil->fw_log_buf_entries * 2) {
		wil_err(wil,
			"Write pointer error, not fetching firmware logs\n");
		return;
	}

	wil_info(wil, "Creating firmware core with just firmware logs\n");
	wil_fw_copy_crash_dump(wil, wil_fw_log_size(wil->fw_log_buf_entries),
			       wil->fw_log_offset);
}

void
wil_fw_set_log_offset_entries(struct wil6210_priv *wil)
{
	struct wil_fw_log_rgf_user_usage rgf;
	u32 rgf_addr[] = { RGF_USER_USAGE_1, RGF_USER_USAGE_2 };
	int rc, i;
	void *data;
	struct fw_map *fw_peri, *uc_data, *rgf_map;

	fw_peri = wil_find_fw_mapping("fw_peri");
	if (!fw_peri) {
		wil_err(wil, "fw_peri section not found in fw_mapping\n");
		return;
	}
	uc_data = wil_find_fw_mapping("uc_data");
	if (!uc_data) {
		wil_err(wil, "uc_data section not found in fw_mapping\n");
		return;
	}

	rc = wil_mem_access_lock(wil);
	if (rc) {
		wil_err(wil,
			"Memory access error, cannot get fw/ucode log offset/entries\n");
		return;
	}

	for (i = 0; i < 2; i++) {
		data = (void *__force)wil->csr + HOSTADDR(rgf_addr[i]);
		wil_memcpy_fromio_32((void *)&rgf,
				     (const void __iomem *__force)data,
				     sizeof(rgf));

		if (rgf_addr[i] == RGF_USER_USAGE_1) {
			wil->fw_log_offset =
				rgf.offset - fw_peri->from +
				(fw_peri->host - WIL6210_FW_HOST_OFF);
			wil->fw_log_buf_entries =
				wil_fw_log_size_lut[rgf.log_size] / 4;
		} else {
			wil->ucode_log_offset =
				rgf.offset - uc_data->from +
				(uc_data->host - WIL6210_FW_HOST_OFF);
			wil->ucode_log_buf_entries =
				wil_fw_log_size_lut[rgf.log_size] / 4;
		}
	}

	wil_mem_access_unlock(wil);
}

void
wil_fw_set_log_level(struct wil6210_priv *wil)
{
	void *dst;
	u32 bitmask[4];
	u32 x;
	int rc;

	/* levels: 1 - E, 2 - W, 3 - I, 4 - V */
	if (wil->fw_log_level <= 0) {
		/* use default log level */
		return;
	}

	/* set bitmask, see struct module_level_enable */
	x = ~(0xffffffff << wil->fw_log_level);
	x = x | x << 0x8 | x << 0x10 | x << 0x18;
	bitmask[0] = x;
	bitmask[1] = x;
	bitmask[2] = x;
	bitmask[3] = 0x10000000 | x;

	rc = wil_mem_access_lock(wil);
	if (rc) {
		wil_err(wil,
			"Memory access error, cannot set fw/ucode log levels\n");
		return;
	}

	/* set all modules for both firmware and microcode logs to same level */
	dst = (void *__force)wil->csr + wil->fw_log_offset +
	      offsetof(struct wil_fw_log_table_header, module_level_enable);
	wil_memcpy_toio_32(dst, &bitmask, sizeof(bitmask));

	dst = (void *__force)wil->csr + wil->ucode_log_offset +
	      offsetof(struct wil_fw_log_table_header, module_level_enable);
	wil_memcpy_toio_32(dst, &bitmask, sizeof(bitmask));

	wil_mem_access_unlock(wil);
}
