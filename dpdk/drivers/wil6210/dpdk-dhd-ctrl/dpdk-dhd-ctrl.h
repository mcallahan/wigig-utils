/* SPDX-License-Identifier: ISC
 * Copyright (c) 2019-2020, Facebook, Inc. All rights reserved.
 */

#ifndef DPDK_DHD_CTR_H
#define DPDK_DHD_CTR_H

#include <sys/types.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <stdint.h>

#include <pthread.h>

#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <rte_pci.h>

struct dhd_state;

struct dhd_tx_queue {
	int queue_fd;
	uint8_t *data;
	struct iovec *rd;

	unsigned int framecount;
	unsigned int framenum;
	unsigned int frame_data_size;

	volatile unsigned long tx_pkts;
	volatile unsigned long err_pkts;
	volatile unsigned long tx_bytes;
};

struct dhd_rx_queue {
	int queue_fd;
	uint8_t *data;
	struct iovec *rd;

	struct rte_mempool *mb_pool;
	uint16_t in_port;

	unsigned int framecount;
	unsigned int framenum;
	unsigned int frame_data_size;

	volatile unsigned long rx_pkts;
	volatile unsigned long err_pkts;
	volatile unsigned long rx_bytes;
};

#define DHD_NUM_QUEUES	2

struct dhd_call_desc {
	uint32_t call_opcode;
	const void *call_data;
	uint16_t call_len;
	void *resp_data;
	uint16_t resp_len;
	void *call_priv;
};

/* Callback to handle synchronous requests from kernel */
typedef int dhd_sync_handler_t(void *ctx, struct dhd_call_desc *req);

/* Maxumum number of links per Wigid device instance */
#define DHD_MAX_PEERS 16
/* Maxumum number of Wigig instances supported */
#define DHD_MAX_PORTS 4
/* Total number of links supported */
#define DHD_TOTAL_LINKS (DHD_MAX_PORTS * DHD_MAX_PEERS)

struct dhd_state {
	int dhd_fd;
	int dhd_ifindex;
	int dhd_ifaddr[IFHWADDRLEN];
	char dhd_ifname[IFNAMSIZ];
	struct tpacket_req dhd_req;
	struct dhd_tx_queue dhd_txq[DHD_NUM_QUEUES];
	struct dhd_rx_queue dhd_rxq[DHD_NUM_QUEUES];
	int dhd_peer_link_id[DHD_MAX_PEERS];
	dhd_sync_handler_t *dhd_handler;
	void *dhd_handler_arg;
	int dhd_pipe[2];
	pthread_t dhd_poll_thread;
	pthread_mutex_t dhd_send_mutex;
};

struct dhd_state *dhd_init(void);
int dhd_attach(struct dhd_state *dhd, uint8_t mac_addr[],
	struct rte_pci_addr pci_addr);
int dhd_start(struct dhd_state *dhd);
int dhd_send_event(struct dhd_state *dhd, const void *buf, size_t len);
void dhd_stop(struct dhd_state*dhd);
void dhd_fini(struct dhd_state *dhd);
int dhd_pcie_retrain(struct dhd_state *dhd, struct rte_pci_addr pci_addr);

void dhd_rx_setup(void *dhd, uint16_t in_port, struct rte_mempool *mb_pool);
uint16_t dhd_rx_burst(void *dhd, struct rte_mbuf **bufs, uint16_t nb_pkts);
uint16_t dhd_tx_burst(void *dhd, struct rte_mbuf **bufs, uint16_t nb_pkts);

#endif /* DPDK_DHD_CTR_H */
