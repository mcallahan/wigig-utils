// SPDX-License-Identifier: ISC
/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved. */

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <linux/sockios.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>

#include "../uapi/linux/wil6210_uapi.h"

int wil_ioblock(char *ifc, uint32_t addr, uint32_t size, uint32_t op,
		void *buf);

/**
 * wil_ioblock - perform one 32-bit I/O operation
 * @ifname - interface name, for example "wlan0"
 * @addr - address, in internal (linker) format,
 *	will be ramapped if neccessary
 * @val - points to value, may be written to
 * @op - operation, 0 - read, 1 - write
 * return value - error code, 0 for success
 */
int wil_ioblock(char *ifname, uint32_t addr, uint32_t size, uint32_t op,
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
	int ifc_ctl_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (ifc_ctl_sock < 0) {
		perror("socket");
		return ifc_ctl_sock;
	}

	/* set up interface name for request */
	strlcpy(ifr.ifr_name, ifname, IFNAMSIZ);
	ifr.ifr_name[IFNAMSIZ - 1] = 0;

	/* go */
	ret = ioctl(ifc_ctl_sock, WIL_IOCTL_MEMIO_BLOCK, &ifr);
	if (ret < 0)
		perror("ioctl");

	return ret;
}

/* sample code showing how to use wil_ioblock()
 * reading from the card demonstrated.
 * block read dumped to the stdout.
 * use like:
 * wil_memdump wlan0 0x840000 100 0 | od -A x -t x1z
 */
int main(int argc, char *argv[])
{
	char *ifname = argv[1];
	uint32_t a, s, op;
	void *buf;
	int ret;
	/* input check */
	if (argc != 5) {
		fprintf(stderr, "Usage: %s <wlan0> <addr> <value> <op>\n",
			argv[0]);
		return 1;
	}
	if ((sscanf(argv[2], "%i", &a) != 1) ||
	    (sscanf(argv[3], "%i", &s) != 1) ||
	    (sscanf(argv[4], "%i", &op) != 1)) {
		fprintf(stderr, "invalid arguments\n");
		return 1;
	}
	buf = malloc(s);
	if (!buf) {
		perror("malloc");
		return 1;
	}
	ret = wil_ioblock(ifname, a, s, op, buf);
	/* result */
	fprintf(stderr, "op=0x%08x addr=0x%08x size=0x%08x ret=%d\n",
		op, a, s, ret);
	if (s != write(STDOUT_FILENO, buf, s))
		perror("write");
	free(buf);
	return 0;
}
