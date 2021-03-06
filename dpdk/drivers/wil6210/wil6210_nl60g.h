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

#ifndef WIL6210_NL60G_H
#define WIL6210_NL60G_H

#include <sys/types.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <stdint.h>

#include <pthread.h>

#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <rte_pci.h>

#include <netlink/netlink.h>

#include "wil6210_ethdev.h"
#include "wil6210_pmc.h"

/* maximum number of nl60g ports(connections) */
#define NL60G_MAX_PORTS 4

/* maximum supported message size */
#define NL60G_MAX_MSG_SIZE	(2*1024*1024) /* 2MB */

struct nl60g_port {
	int fd;
	bool publish_nl_evt;
	struct nl_cb *cb;
	struct nl_sock *sk;
	struct nl_msg *reply;
	/* for reading PMC data and ring descriptors */
	struct wil_pmc_reader_ops pmc_reader, pmcring_reader;
	void *pmc_reader_ctx, *pmcring_reader_ctx;
	/* thread for processing requests on this socket */
	pthread_t req_thread;
	int exit_sockets[2];
	struct nl60g_state *nl60g;
};

struct nl60g_state {
	/* the root socket listening for connections */
	int fd;
	struct wil6210_priv *wil;
	struct rte_pci_addr pci_addr; /* for file name calculation */
	struct nl60g_port ports[NL60G_MAX_PORTS];
	int exit_sockets[2];
	pthread_t poll_thread;
	/* protects modifications to ports array */
	pthread_mutex_t ports_mutex;
};

void nl60g_fw_state_evt(struct nl60g_state *nl60g,
	enum wil_fw_state fw_state);
void nl60g_receive_wmi_evt(struct nl60g_state *nl60g, uint8_t *cmd, int len);

struct nl60g_state *nl60g_init(void);
int nl60g_start(struct nl60g_state *nl60g, struct wil6210_priv *wil,
	struct rte_pci_addr pci_addr);
void nl60g_stop(struct nl60g_state *nl60g);
void nl60g_fini(struct nl60g_state *nl60g);

#endif /* WIL6210_NL60G_H */
