// SPDX-License-Identifier: ISC
/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved. */

#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <linux/sockios.h>
#include <err.h>
#include <errno.h>
#include "../uapi/linux/wil6210_uapi.h"

static int mod_wr; /* = 0, operation mode */
static int addr_mode;
static uint32_t addr;
static uint32_t val;
static int addr_set, val_set;
static char *ifc = "wlan0";
static int no_nl;
static int help;

/**
 * wil_io - perform one 32-bit I/O operation
 * @ifname - interface name, for example "wlan0"
 * @addr - address, in internal (linker) format,
 *	will be ramapped if neccessary
 * @val - points to value, may be written to
 * @op - operation, 0 - read, 1 - write
 * return value - error code, 0 for success
 */
static int wil_io(char *ifname, uint32_t addr, uint32_t *val, uint32_t op)
{
	int ret;
	struct wil_memio io = {
		.op = op,
		.addr = addr,
		.val = *val,
	};
	struct ifreq ifr = {
		.ifr_data = &io,
	};

	/* init socket */
	int ifc_ctl_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (ifc_ctl_sock < 0) {
		perror("socket");
		return errno;
	}

	/* set up interface name for request */
	strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
	ifr.ifr_name[IFNAMSIZ - 1] = 0;

	/* go */
	ret = ioctl(ifc_ctl_sock, WIL_IOCTL_MEMIO, &ifr);
	if (ret < 0) {
		perror("ioctl");
		return errno;
	}

	*val = io.val;

	return ret;
}

int main(int argc, char *argv[])
{
	/* parameters parsing */
	int c, i, ret;
	unsigned long x;
	char *endptr;
	static struct option long_options[] = {
		{"address", required_argument, NULL, 'a'},
		{"linker", no_argument, &addr_mode, WIL_MMIO_ADDR_LINKER},
		{"ahb", no_argument, &addr_mode, WIL_MMIO_ADDR_AHB},
		{"bar", no_argument, &addr_mode, WIL_MMIO_ADDR_BAR},
		{"read", no_argument, &mod_wr, 0},
		{"write", required_argument, NULL, 'w'},
		{"ifc", required_argument, NULL, 'i'},
		{"no_nl", no_argument, &no_nl, 1},
		{"help", no_argument, &help, 1},
		{}
	};
	do {
		c = getopt_long(argc, argv, "a:v:i:rw:n", long_options, &i);
		switch (c) {
		case 0:
			break;
		case 'a': /* address */
			x = strtoul(optarg, &endptr, 0);
			addr = (uint32_t)x;
			if (*endptr != '\0')
				errx(1, "Unable to parse address [%s]", optarg);
			if (addr != x)
				errx(1, "Address 0x%08lx is too large", x);
			addr_set = 1;
			break;
		case 'w': /* value */
			x = strtoul(optarg, &endptr, 0);
			val = (uint32_t)x;
			if (*endptr != '\0')
				errx(1, "Unable to parse value [%s]", optarg);
			if (val != x)
				errx(1, "Value 0x%08lx do not fit into 32 bit",
				     x);
			val_set = 1;
			mod_wr = 1;
			break;
		case 'r':
			mod_wr = 0;
			break;
		case 'i': /* interface */
			ifc = optarg;
			break;
		case 'n':
			no_nl = 1;
			break;
		case -1: /* end of options */
			break;
		default:
			help = 1;
		}
	} while (c >= 0);
	/* parameters validation */
	if (!addr_set)
		help = 1;
	if (mod_wr != val_set)
		help = 1;
	if (addr % 4) {
		printf("Address should be dword aligned\n");
		help = 1;
	}
	if (help) {
		static char *help_str = "Usage: wil_mem [OPTION]...\n"
"Read/write card register\n"
" Read value printrd in hexa-decimal format\n"
"\n"
"Mandatory arguments to long options are mandatory for short options too.\n"
" The following switches are mandatory:\n"
"  -a, --address=NUMBER		Address to access\n"
" The following switches are optional:\n"
"  Address mode:\n"
"      --linker			Address is FW linker address (default)\n"
"      --ahb			Address is AHB address\n"
"      --bar			Address is BAR offset\n"
"  Operation:\n"
"  -r, --read			Read (default)\n"
"  -w, --write=NUMBER		Write. Value is mandatory\n"
"  Other:\n"
"  -i, --ifc=STRING		Network interface to use\n"
"				Default is wlan0\n"
"  -n, --no_nl			Do not append new line when printing\n"
"				read value\n";
		errx(1, "%s", help_str);
	}
	/* real business */
	ret = wil_io(ifc, addr, &val, mod_wr | addr_mode);
	if (ret)
		errx(ret, "Error accessing memory at %s: %d %s", ifc, ret,
		     strerror(ret));

	if (!mod_wr)
		printf("0x%08x%s", val, no_nl ? "" : "\n");

	return 0;
}
