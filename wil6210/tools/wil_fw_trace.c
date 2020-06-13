/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
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

#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <linux/sockios.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include <getopt.h>
#include <errno.h>
#include <err.h>
#include <time.h>
#include <stdarg.h>
#include <libgen.h>
#include <dirent.h>

#include "../uapi/linux/wil6210_uapi.h"

/*
 * Dumps firmware trace.
 *
 * Uses binary representation of 'strings' file,
 * it should be named fw_strings.bin and be in the current directory
 * Periodically reads peripheral memory like in blob_fw_peri on the debugfs
 * Name of peripheral memory file passed as parameter
 */

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

typedef uint32_t u32;
typedef int32_t s32;
typedef unsigned int uint;

struct module_level_enable { /* Little Endian */
	uint error_level_enable : 1;
	uint warn_level_enable : 1;
	uint info_level_enable : 1;
	uint verbose_level_enable : 1;
	uint disable_new_line : 1;
	uint reserved0 : 3;
} __attribute__((packed));

/* Little Endian */
struct log_trace_header {
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

#define HDR_SIGNATURE_MAGIC (5)

union log_event {
	struct log_trace_header hdr;
	u32 param;
} __attribute__((packed));

struct log_table_header {
	u32 write_ptr; /* incremented by trace producer every write */
	struct module_level_enable module_level_enable[16];
	union log_event evt[0];
} __attribute__((packed));

static char *strings_bin;
enum { str_mask = 0xFFFFF };

/* string used in place of corrupted string parameter */
static const char *CORRUPTED_PARAM_MARK = "ZZZZ";

#define RGF_USER_USAGE_1         0x880004
#define RGF_USER_USAGE_2         0x880008

#define BAR_HOST_ADDR            0x880000
#define FW_PERI_LINKER_ADDR      0x840000
#define FW_PERI_HOST_ADDR        0xa20000
#define UC_DATA_LINKER_ADDR      0x800000
#define UC_DATA_HOST_ADDR        0xa78000

/* Pre-mapped PCIe device BAR */
static volatile uint32_t *dev_data_buf;

static int restart_log;
static int ifc_ctl_sock = -1;

/* log parameters */
static size_t log_offset; /* =0; offset of log buf in the file */
static size_t log_buf_entries; /* =0 entries in the log buf */

static void *log_buf; /* memory allocated for the log buf */
static union log_event evt;
static char timestamp[25] = { '\0' };
static char module_string[128] = { '\0' };
static uint prev_module;
static char trailing_newline = 1;
static char *str_buf;
static size_t str_sz;
static u32 rptr; /* = 0; */
static u32 rptr_param_last; /* = 0; */
static u32 wptr; /* = 0; */
static const char *const levels[] = {
	"E",
	"W",
	"I",
	"V",
};

#define MAX_PARAMS (7)

static const char *modules[16];

static size_t read_all(int f, char *buf, size_t n)
{
	size_t actual = 0, r;
	do {
		r = read(f, buf + actual, n - actual);
		actual += r;
	} while ((r > 0) && (actual < n));
	return actual;
}

static char *read_strings(const char *name, size_t *size, size_t extra_space)
{
	int f = open(name, O_RDONLY);
	size_t sz = *size;
	size_t r;
	char *buf;

	if (f < 0)
		err(1, "Error: strings file %s", name);

	if (!sz) {
		sz = lseek(f, 0, SEEK_END);
		lseek(f, 0, SEEK_SET);
	}
	buf = malloc(sz + extra_space);
	if (!buf)
		errx(1, "Error: unable to allocate string buffer %zd bytes",
		     sz);
	r = read_all(f, buf, sz);
	close(f);
	if (r != sz)
		errx(1, "Error: from %s read %zd bytes out of %zd", name, r,
		     sz);
	*size = sz;
	return buf;
}

static int wil_ioblock(char *ifname, uint32_t addr, uint32_t size, uint32_t op,
		void *buf)
{
	int ret;
	struct wil_memio_block io = {
		.op = op,
		.addr = addr,
		.size = size,
		.block = buf,
	};
	struct ifreq ifr = {
		.ifr_data = &io,
	};

	/* init socket */
	if (ifc_ctl_sock < 0)
		ifc_ctl_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (ifc_ctl_sock < 0) {
		perror("socket");
		return ifc_ctl_sock;
	}

	/* set up interface name for request */
	strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
	ifr.ifr_name[IFNAMSIZ - 1] = 0;

	/* go */
	ret = ioctl(ifc_ctl_sock, WIL_IOCTL_MEMIO_BLOCK, &ifr);
	if (ret < 0)
		perror("ioctl");

	return ret;
}

static inline size_t log_size(size_t entry_num)
{
	return sizeof(struct log_table_header) + entry_num * 4;
}

static int read_log(const char *fname, char *ifname, void *buf, size_t off,
		    size_t size)
{
	int f, ret;
	uint32_t addr;

	if (dev_data_buf != NULL) {
		uint32_t volatile *data;

		data = dev_data_buf + off / 4;
		/* Manual copy word by word for aligned access */
		while (size >= 4) {
			*(uint32_t *)buf = *data++;
			buf = (char *)buf + 4;
			size -= 4;
		}
		return 0;
	}

	if (ifname != NULL) {
		if (restart_log)
			return -1;

		ret = wil_ioblock(ifname, log_offset + BAR_HOST_ADDR,
				  log_size(log_buf_entries),
				  wil_mmio_read, buf);
		if (ret)
			restart_log = 1;
		return ret;
	}

	f = open(fname, O_RDONLY);
	if (f < 0)
		err(1, "Error: memdump file %s", fname);
	lseek(f, off, SEEK_SET);
	size_t r = read_all(f, buf, size);

	close(f);
	if (r != size)
		errx(1, "Error: from %s read %zd bytes out of %zd", fname, r,
		     size);
	return 0;
}

static void logerr(const char *fmt, ...)
{
	va_list argp;

	if (!trailing_newline)
		putc('\n', stdout);
	fprintf(stdout, "%s[%6d/%6d] ", timestamp, rptr, wptr);
	va_start(argp, fmt);
	vfprintf(stdout, fmt, argp);
	va_end(argp);
	putc('\n', stdout);
	trailing_newline = 1;
}

static int print_trace(FILE * f, const char *fmt);

static void do_parse(int onlylogs)
{
	struct log_table_header *h = log_buf;

	if (restart_log)
		return;

	wptr = h->write_ptr;

	if (onlylogs) {
		/* ignore pointers and parse all entries */
		wptr = log_buf_entries;
	} else {
		if (rptr > wptr) {
			logerr("rptr overrun; try rewind it back");
			rptr = wptr > log_buf_entries ? wptr - log_buf_entries : 0;
		}
		if ((wptr - rptr) >= log_buf_entries) {
			logerr("wptr overflow; try to parse last wrap");
			rptr = wptr - log_buf_entries;
		}
	}

	for (; rptr < wptr; rptr++) {
		const char *fmt;
		uint params_dwords;
		uint strring_offset;

		evt = h->evt[rptr % log_buf_entries];
		if (evt.hdr.signature != HDR_SIGNATURE_MAGIC) {
			logerr("Signature magic mismatch 0x%08x", evt.param);
			continue;
		}

		strring_offset = evt.hdr.strring_offset
				 << 2; /* promote to 20 bits */
		if (strring_offset > str_sz) {
			logerr("String offset [%d/%lu] overflow",
			       strring_offset, str_sz);
			continue;
		}

		snprintf(module_string, sizeof(module_string),
			 "%s[%6u/%6u] %9s %s : ",
			 timestamp, rptr, wptr, modules[evt.hdr.module],
			 levels[evt.hdr.level]);
		fmt = str_buf + strring_offset;
		params_dwords = evt.hdr.params_dwords_lsb << 0;
		params_dwords |= evt.hdr.params_dwords_msb << 2;

		if (params_dwords > 2 * MAX_PARAMS) {
			logerr("Params length (%d) exceeds max (%d) in %s",
			       params_dwords, MAX_PARAMS, fmt);
			continue;
		}

		rptr_param_last = rptr + params_dwords;

		if (rptr_param_last > wptr) {
			logerr("Params length (%d) overflows wptr (%d) in %s",
			       params_dwords, wptr, fmt);
			rptr_param_last = wptr;
		}

		rptr += 1;

		if (trailing_newline) {
			fputs(module_string, stdout);
			trailing_newline = 0;
		} else {
			/* Separate lines from different modules */
			if (prev_module != evt.hdr.module) {
				printf("\n%s", module_string);
			} else if (strncmp("[NNL]", fmt, 5) == 0) {
				fmt += 5;
			} else if (!h->module_level_enable[prev_module]
					.disable_new_line) {
				printf("\n%s", module_string);
			}
		}

		print_trace(stdout, fmt);
		prev_module = evt.hdr.module;

		if (rptr - 1 > rptr_param_last)
			logerr("Params overflow (%d/%d)",
			       rptr - 1, rptr_param_last);
		else if (rptr - 1 != rptr_param_last)
			logerr("Params underflow (%d/%d)",
			       rptr - 1, rptr_param_last);
		rptr = rptr_param_last;
	}

	fflush(stdout);
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

union arg {
	uintmax_t i;
	double f;
	const void *p;
};

static union arg pop_arg(int type)
{
	struct log_table_header *h = log_buf;
	union arg arg = { .p = 0 };

	switch (type) {
	case PTR:
		if (rptr > rptr_param_last ||
		    (h->evt[rptr % log_buf_entries].param & str_mask) >=
			    str_sz) {
			arg.p = CORRUPTED_PARAM_MARK;
			break;
		}

		arg.p = str_buf +
			(h->evt[rptr++ % log_buf_entries].param & str_mask);
		break;

	case CHAR:
	case SHORT:
	case INT:
	case LONG:
	case PDIFF:
		if (rptr > rptr_param_last) {
			arg.i = -1;
			break;
		}

		arg.i = (int32_t)h->evt[rptr++ % log_buf_entries].param;
		break;

	case UCHAR:
	case USHORT:
	case UINT:
	case ULONG:
	case UIPTR:
	case SIZET:
		if (rptr > rptr_param_last) {
			arg.i = -1;
			break;
		}

		arg.i = h->evt[rptr++ % log_buf_entries].param;
		break;

	case LLONG:
	case ULLONG:
	case IMAX:
	case UMAX:
		if (rptr + 1 > rptr_param_last) {
			arg.i = -1;
			break;
		}

		arg.i = (uintmax_t)h->evt[rptr++ % log_buf_entries].param << 0;
		arg.i |= (uintmax_t)h->evt[rptr++ % log_buf_entries].param
			 << 32;
		break;

	/* floating point types are not supported at the moment */
	case DBL:
	case LDBL:
		if (rptr + 1 > rptr_param_last) {
			arg.f = -1;
			break;
		}

		arg.f = -1.0;
		rptr += 2;
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
		return MAX(w, 3 + pl);
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
		return MAX(w, pl + l);
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
		int sh = MIN(29, e2);

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
		int sh = MIN(9, -e2);
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
				p = MIN(p, MAX(0, 9 * (z - r - 1) - j));
			else
				p = MIN(p, MAX(0, 9 * (z - r - 1) + e - j));
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
			out(f, s, MIN(9, p));
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
			out(f, s, MIN(buf + 9 - s, p));
			p -= buf + 9 - s;
		}
		pad(f, '0', p + 18, 18, 0);
		out(f, estr, ebuf - estr);
	}

	pad(f, ' ', w, pl + l, fl ^ LEFT_ADJ);

	return MAX(w, pl + l);
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

static int printf_core(FILE *f, const char *fmt, union arg *nl_arg,
		       int *nl_type)
{
	char *a, *z, *s = (char *)fmt;
	unsigned int l10n = 0, fl;
	int w, p, xp;
	union arg arg = { .p = 0 };
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
					fputs(module_string, f);
				else
					trailing_newline = 1;
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
				w = f ? pop_arg(INT).i : 0;
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
				p = f ? pop_arg(INT).i : 0;
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
				arg = pop_arg(st);
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
			p = MAX((size_t)p, 2 * sizeof(void *));
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
			p = MAX(p, z - a + !arg.i);
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

	for (i = 1; i <= MAX_PARAMS && nl_type[i]; i++)
		nl_arg[i] = pop_arg(nl_type[i]);

	for (; i <= MAX_PARAMS && !nl_type[i]; i++)
		;

	if (i <= MAX_PARAMS)
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

static int print_trace(FILE *f, const char *fmt)
{
	int ret;
	int nl_type[MAX_PARAMS + 1] = { 0 };
	union arg nl_arg[MAX_PARAMS + 1];

	if (printf_core(0, fmt, nl_arg, nl_type) < 0)
		return -1;

	ret = printf_core(f, fmt, nl_arg, nl_type);
	if (ferror(f))
		ret = -1;
	return ret;
}

static const char *next_mod(const char *mod)
{
	mod = strchr(mod, '\0') + 1;
	while (*mod == '\0')
		mod++;
	return mod;
}

static char *get_rgf_path(const char *dump_file_path)
{
	static const char rgf_filename[] = "RGF_USER_USAGE_1";
	size_t dir_len;
	char *rgf_path;
	const char *last_dir = strrchr(dump_file_path, '/');

	last_dir = last_dir ? last_dir + 1 : dump_file_path;
	dir_len = last_dir - dump_file_path;
	rgf_path = malloc(dir_len + sizeof(rgf_filename));

	if (!rgf_path)
		err(1, "Failed to allocate memory for rgf_path");

	snprintf(rgf_path, dir_len + 1, "%s", dump_file_path);
	snprintf(rgf_path + dir_len, sizeof(rgf_filename), "%s", rgf_filename);
	return rgf_path;
}

struct RGF_USER_USAGE {
	uint offset : 29;
	uint log_size : 3;
};

static const size_t LOG_SIZE_FW_LUT[] = {
	4 * 1024, 1 * 1024,  2 * 1024,  4 * 1024,
	8 * 1024, 16 * 1024, 32 * 1024, 64 * 1024,
};

#define RGF_STRING_SIZE (12)

static struct RGF_USER_USAGE read_rgf_user_usage(const char *path)
{
	union {
		u32 buffer;
		struct RGF_USER_USAGE value;
	} rgf;
	char rgf_string[RGF_STRING_SIZE] = { '\0' };
	char *rgf_string_end;
	int f = open(path, O_RDONLY);

	if (f < 0)
		err(1, "Failed to open %s", path);

	if (read_all(f, rgf_string, RGF_STRING_SIZE) <= 0)
		err(1, "Failed to read RGF_USER_USAGE %s", path);

	close(f);
	rgf.buffer = (u32)strtoul(rgf_string, &rgf_string_end, 16);

	while (*rgf_string_end && isspace(*rgf_string_end))
		rgf_string_end++;
	if (*rgf_string_end)
		err(1, "Failed to parse RGF_USER_USAGE '%s'", rgf_string);

	return rgf.value;
}

#define KERNEL_DRIVER_NAME "wil6210"

/*
 * Check if this pci device is using the wil6210 kernel driver. If it is,
 * then find the interface name so we can read logs via ioctls instead of
 * from the sysfs pci resource.
 *
 * Return a pointer to malloced interface name if found, otherwise NULL.
 */
static char *find_ifname_by_driver(char *pcidev) {
	int len, ret;
	struct stat sb;
	char driver_fname[sizeof("/sys/bus/pci/devices/xxxx:xx:xx.x/driver")];
	char netif_fname[sizeof("/sys/bus/pci/devices/xxxx:xx:xx.x/net")];
	char link_name[255];
	char *ifname;
	DIR *dirp;
	struct dirent *dp;

	/* check if this device is using the kernel driver */
	len = snprintf(
		driver_fname,
		sizeof(driver_fname),
		"/sys/bus/pci/devices/%s/driver",
		pcidev);
	if (len != sizeof(driver_fname) - 1)
		errx(1, "malformed PCI device selector");

	ret = readlink(driver_fname, link_name, sizeof(link_name) - 1);
	if (ret < 0 || ret > sizeof(link_name) - 1)
		return NULL;

	link_name[ret] = '\0';
	if (strcmp(basename(link_name), KERNEL_DRIVER_NAME) != 0)
		return NULL;

	/* verified this device is using kernel driver, find interface name */
	len = snprintf(
		netif_fname,
		sizeof(netif_fname),
		"/sys/bus/pci/devices/%s/net",
		pcidev);
	if (len != sizeof(netif_fname) - 1)
		errx(1, "malformed PCI device selector");

	dirp = opendir(netif_fname);
	if (!dirp)
		return NULL;

	while ((dp = readdir(dirp))) {
		if (strcmp(dp->d_name, ".") != 0 && strcmp(dp->d_name, "..") != 0) {
			ifname = strdup(dp->d_name);
			if (!ifname)
				err(1, "insufficient memory");
			closedir(dirp);
			return ifname;
		}
	}
	closedir(dirp);
	return NULL;
}

static void open_pci_device(const char *devsel, size_t *log_offset,
    size_t *log_buf_entries, int ucode, int verbosity)
{
	union {
		u32 buffer;
		struct RGF_USER_USAGE value;
	} rgf;
	char fname[sizeof("/sys/bus/pci/devices/xxxx:xx:xx.x/resource0")];
	struct stat st;
	uint32_t volatile *data;
	uint32_t offset, x;
	int len, ret, f;

	len = snprintf(fname, sizeof(fname), "/sys/bus/pci/devices/%s/resource0",
	    devsel);
	if (len != sizeof(fname) - 1)
		errx(1, "malformed PCI device selector");

	f = open(fname, O_RDWR);
	if (f < 0)
		err(1, "Error: unable to open %s", fname);

	ret = fstat(f, &st);
	if (ret < 0)
		err(1, "Error: stat %s", fname);

	data = mmap(NULL, st.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, f, 0);
	if (data == MAP_FAILED)
		err(1, "Error: map file %s", fname);

	/* FW log information word is at 0x880004, that is offset 0x04 in BAR
	 * ucode log information is next word at 0x880008, offset 0x08 in BAR
	 *
	 * offset = rgf.value.offset - linker addr + (pci/host addr - bar addr)
	 */
	if (ucode == 0) {
		rgf.buffer = data[1];
		offset = rgf.value.offset - FW_PERI_LINKER_ADDR +
			(FW_PERI_HOST_ADDR - BAR_HOST_ADDR);
	} else {
		rgf.buffer = data[2];
		offset = rgf.value.offset - UC_DATA_LINKER_ADDR +
			(UC_DATA_HOST_ADDR - BAR_HOST_ADDR);
	}

	if (log_offset && !log_offset[0])
		log_offset[0] = offset;

	if (log_buf_entries && !log_buf_entries[0])
		log_buf_entries[0] =
			LOG_SIZE_FW_LUT[rgf.value.log_size] / 4;

	dev_data_buf = data;

	/* set log levels */
	data = data + offset / 4;
	if (verbosity >= 0) {
		/* Set bitmask, see struct module_level_enable */
		x = ~(0xffffffff << (verbosity + 1));
		x = x | x << 0x8 | x << 0x10 | x << 0x18;
		data[1] = x;
		data[2] = x;
		data[3] = x;
		data[4] = 0x10000000 | x;
	} else {
		/* Reset default log levels */
		data[1] = 0x01010101;
		data[2] = 0x01000101;
		data[3] = 0x01010101;
		data[4] = 0x1f010101;
	}
}

static void update_timestamp(void)
{
	int toffset;
	struct timespec ts;

	clock_gettime(CLOCK_REALTIME, &ts);
	toffset = strftime(timestamp, sizeof(timestamp), "%T",
			   gmtime(&ts.tv_sec));

	if (toffset > 0)
		snprintf(timestamp + toffset, sizeof(timestamp) - toffset,
			 ".%09ld UTC ", ts.tv_nsec);
	else
		snprintf(timestamp, sizeof(timestamp),
			 "00:00:00.000000000 UTC ");
}

static int if_get_log_offset_size(char *ifname, int ucode)
{
	struct RGF_USER_USAGE rgf;
	int ret;
	uint32_t rgf_addr, host_addr, linker_addr;

	if (ucode == 0) {
		rgf_addr = RGF_USER_USAGE_1;
		host_addr = FW_PERI_HOST_ADDR;
		linker_addr = FW_PERI_LINKER_ADDR;
	} else {
		rgf_addr = RGF_USER_USAGE_2;
		host_addr = UC_DATA_HOST_ADDR;
		linker_addr = UC_DATA_LINKER_ADDR;
	}

	ret = wil_ioblock(ifname, rgf_addr, sizeof(rgf),
			  wil_mmio_read, &rgf);
	if (ret) {
		restart_log = 1;
		return ret;
	}

	log_offset = rgf.offset - linker_addr + (host_addr - BAR_HOST_ADDR);
	log_buf_entries = LOG_SIZE_FW_LUT[rgf.log_size] / 4;
	return 0;
}

static int if_set_log_level(char *ifname, int verbosity)
{
	uint32_t bitmask[4];
	uint32_t x;
	int ret;

	if (verbosity < 0)
		return 0;

	/* Set bitmask, see struct module_level_enable */
	x = ~(0xffffffff << (verbosity + 1));
	x = x | x << 0x8 | x << 0x10 | x << 0x18;
	bitmask[0] = x;
	bitmask[1] = x;
	bitmask[2] = x;
	bitmask[3] = 0x10000000 | x;

	ret = wil_ioblock(
	    ifname,
	    log_offset + BAR_HOST_ADDR +
	        offsetof(struct log_table_header, module_level_enable),
	    sizeof(bitmask),
	    wil_mmio_write,
	    &bitmask);

	if (ret)
		restart_log = 1;

	return ret;
}

static void start_log(char *peri, char *pcidev, char *ifname, int ucode,
		      int verbosity, int enable_timestamp, int once, int onlylogs)
{
	int i, ret;
	const char *mod;

	restart_log = 0;

	if (ifname != NULL) {
		ret = if_get_log_offset_size(ifname, ucode);
		if (ret)
			return;

		ret = if_set_log_level(ifname, verbosity);
		if (ret)
			return;
	}

	if (!log_buf)
		log_buf = malloc(log_size(log_buf_entries));
	if (!log_buf)
		errx(1, "Error: Unable to allocate log buffer %zd bytes",
		     log_size(log_buf_entries));
	struct log_table_header *h = log_buf;

	mod = str_buf;

	if (enable_timestamp)
		update_timestamp();

	ret = read_log(peri, ifname, log_buf, log_offset,
		       log_size(log_buf_entries));
	if (ret)
		return;

	if (onlylogs) {
		wptr = log_buf_entries;
	} else {
		wptr = h->write_ptr;
		if ((wptr - rptr) >= log_buf_entries) {
			/* overflow; try to parse last wrap */
			rptr = wptr - log_buf_entries;
		}
	}

	if (peri)
		printf("memdump file: <%s> ", peri);
	if (pcidev)
		printf("pcidev: <%s> ", pcidev);
	if (ifname)
		printf("ifname: <%s> ", ifname);
	printf("offset: 0x%zx entries: %zd strings: <%s> once: %d\n",
	       log_offset, log_buf_entries, strings_bin, once);

	printf("  wptr = %u rptr = %u\n", wptr, rptr);
	for (i = 0; i < 16; i++, mod = next_mod(mod)) {
		modules[i] = mod;
		struct module_level_enable *m = &h->module_level_enable[i];

		printf("  %s[%2d] : %s%s%s%s%s\n", modules[i], i,
		       m->error_level_enable ? "E" : " ",
		       m->warn_level_enable ? "W" : " ",
		       m->info_level_enable ? "I" : " ",
		       m->verbose_level_enable ? "V" : " ",
		       m->disable_new_line ? "n" : " ");
	}
	return;
}


int main(int argc, char *argv[])
{
	int i, c;
	unsigned long x;
	char *pcidev = 0; /* PCIe device to access */
	char *ifname = 0; /* Interface name for device */
	char *peri = 0; /* file to read */
	char *rgf_path;
	char *endptr;
	unsigned long poll_interval_ns = 100UL * 1000000UL;
	struct timespec t1;
	struct RGF_USER_USAGE rgf_user_usage;
	static int ucode; /* = 0; */
	static int onlylogs; /* = 0; */
	static int enable_timestamp; /* = 0; */
	static int once; /* = 0; */
	static int help; /* = 0; */
	static int verbosity = -1;
	size_t extra_space;
	struct stat st;
	static struct option long_options[] = {
		{ "device", required_argument, NULL, 'd' },
		{ "memdump", required_argument, NULL, 'm' },
		{ "offset", required_argument, NULL, 'o' },
		{ "logsize", required_argument, NULL, 'l' },
		{ "strings", required_argument, NULL, 's' },
		{ "interval", required_argument, NULL, 'i' },
		{ "verbosity", required_argument, NULL, 'v' },
		{ "ucode", no_argument, &ucode, 1 },
		{ "onlylogs", no_argument, &onlylogs, 1 },
		{ "timestamp", no_argument, &enable_timestamp, 1 },
		{ "once", no_argument, &once, 1 },
		{ "help", no_argument, &help, 1 },
		{ 0, 0, 0, 0 }
	};
	do {
		c = getopt_long(argc, argv, "d:m:o:l:s:i:v:uOt1h", long_options, &i);
		switch (c) {
		case 'd': /* pcidev */
			pcidev = optarg;
			break;
		case 'm': /* memdump */
			peri = optarg;
			break;
		case 'o': /* offset */
			x = strtoul(optarg, &endptr, 0);
			log_offset = (size_t)x;
			if (*endptr != '\0')
				errx(1, "Unable to parse offset [%s]", optarg);
			if (log_offset != x)
				errx(1, "Offset too large: 0x%08lx", x);
			break;
		case 'l': /* logsize */
			x = strtoul(optarg, &endptr, 0);
			log_buf_entries = (size_t)x;
			if (*endptr != '\0')
				errx(1, "Unable to parse log size [%s]",
				     optarg);
			if (log_buf_entries != x)
				errx(1, "Log size too large: 0x%08lx", x);
			break;
		case 's': /* strings */
			strings_bin = optarg;
			break;
		case 'i': /* interval */
			poll_interval_ns = strtoul(optarg, &endptr, 0)
					 * 1000000UL;
			if (*endptr != '\0')
				errx(1, "Unable to parse poll interval [%s]",
				     optarg);
			break;
		case 'v': /* verbosity */
			if (strcmp(optarg, "ERROR") == 0) {
				verbosity = 0;
			} else if (strcmp(optarg, "WARN") == 0) {
				verbosity = 1;
			} else if (strcmp(optarg, "INFO") == 0) {
				verbosity = 2;
			} else if (strcmp(optarg, "VERBOSE") == 0) {
				verbosity = 3;
			}
			break;
		case 'u': /* ucode */
			ucode = 1;
			break;
		case 'O': /* onlylogs*/
			onlylogs = 1;
			break;
		case 't': /* timestamp */
			enable_timestamp = 1;
			break;
		case '1': /* once */
			once = 1;
			break;
		case 0:  /* flag */
		case -1: /* end of options */
			break;
		default:
			help = 1;
		}
	} while (c >= 0);

	/*
	 * Specifying the device option will read logs by sending ioctls to device
	 * if it is using the wil6210 kernel driver. If it is not using the kernel
	 * driver, logs are read from the sysfs pci resource. If not using the
	 * kernel driver (logs are read from pci resource), unloading the driver
	 * while this program runs will cause it to crash with SIGBUS error.
	 *
	 * Specifying the memdump option will read logs out of a memory dump.
	 * Log offset and log size need to be set, or the program will attempt
	 * to read them from file "RGF_USER_USAGE_1" in the memdump's directory.
	 * When using the option of onlylogs, log offset and log size do not to
	 * be set, assuming the whole memdump file consists of only logs.
	 */
	if (pcidev && peri)
		errx(1, "memdump and device options cannot be specified together");

	if (pcidev != NULL) {
		ifname = find_ifname_by_driver(pcidev);
		if (!ifname)
			open_pci_device(
				pcidev, &log_offset, &log_buf_entries, ucode, verbosity);
	}

	if (peri && (!log_buf_entries || !log_offset) && !onlylogs) {
		rgf_path = get_rgf_path(peri);
		rgf_user_usage = read_rgf_user_usage(rgf_path);
		free(rgf_path);

		if (!log_offset)
			log_offset = rgf_user_usage.offset - 0x840000;

		if (!log_buf_entries)
			log_buf_entries =
				LOG_SIZE_FW_LUT[rgf_user_usage.log_size] / 4;
	}

	if (onlylogs && peri) {
		stat(peri, &st);
		log_buf_entries = (st.st_size - sizeof(struct log_table_header)) / 4;
		log_offset = 0; /* no offset, file only contains logs */
		once = 1; /* read all logs in one iteration */
	}

	/* check that strings_bin, log_offset, and log_buf_entries are set when
	 * reading from device via sysfs pci resource or reading from dump
	 */
	help = help || (((pcidev && !ifname) || peri) &&
			!(strings_bin && (log_offset > 0 || onlylogs) &&
			  log_buf_entries > 0));
	/* check that strings_bin is set when ifname is set */
	help = help || (ifname && !strings_bin);

	if (help) {
		printf("Usage: %s [OPTION]...\n"
		       "Extract trace log from firmware\n"
		       "\n"
		       "Mandatory arguments to long options are mandatory for short options too.\n"
		       " The following switches are mandatory:\n"
		       "  -d, --device=FILE		PCIe device selector to read logs from\n"
		       "  -m, --memdump=FILE		File to read memory dump from\n"
		       "  -s, --strings=FILE		File with format strings\n"
		       "  The device and memdump parameters are mutually exclusive.\n"
		       " The following switches are optional:\n"
		       "  -l, --logsize=NUMBER		Log buffer size, entries.\n"
		       "				Default - read from RGF\n"
		       "  -o, --offset=NUMBER		Offset of the log buffer in memdump, bytes.\n"
		       "				Default - read from RGF\n"
		       "  -i, --interval=msec		Polling interval\n"
		       "				Default - 100 ms\n"
		       "  -v --verbosity={ERROR|WARN|INFO|VERBOSE}\n"
		       "				Configure log verbosity level for all modules when reading from device.\n"
		       "				Levels lower than the one specified will be enabled as well.\n"
		       "				Default - ERROR for all FW log modules except CALIBS (none) and FW_3P (VERBOSE)\n"
		       "  -u, --ucode			Read microcode (ucode) logs instead of firmware logs.\n"
		       "				Must read from device. Check to use appropriate strings file.\n"
		       "  -O, --onlylogs		Memdump file consists only of FW logs, with arbitrary length. No offset/logsize used.\n"
		       "  -t, --timestamp		Print timestamps\n"
		       "  -1, --once			Read and parse once and exit, otherwise\n"
		       "				keep reading infinitely\n",
		       argv[0]);
		exit(1);
	}

	if (clock_gettime(CLOCK_MONOTONIC, &t1) < 0)
		errx(1, "Unable to read the CLOCK");

	restart_log = 1;

	/* reserve extra space at the end of string buffer for a special
	 * string that we write to the log when we encounter corrupted
	 * offsets for string parameters. This can happen when log is
	 * wrapped in the middle of a log message
	 */
	extra_space = strlen(CORRUPTED_PARAM_MARK) + 1;
	str_buf = read_strings(strings_bin, &str_sz, extra_space);
	snprintf(str_buf + str_sz, extra_space, "%s", CORRUPTED_PARAM_MARK);

	for (;;) {
		if (restart_log)
			start_log(peri, pcidev, ifname, ucode, verbosity,
				  enable_timestamp, once, onlylogs);

		do_parse(onlylogs);
		if (once)
			break;

		t1.tv_sec += (t1.tv_nsec + poll_interval_ns) / 1000000000L;
		t1.tv_nsec = (t1.tv_nsec + poll_interval_ns) % 1000000000L;
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t1, NULL);
		clock_gettime(CLOCK_MONOTONIC, &t1);

		if (enable_timestamp)
			update_timestamp();
		read_log(peri, ifname, log_buf, log_offset, log_size(log_buf_entries));
	}
	return 0;
}
