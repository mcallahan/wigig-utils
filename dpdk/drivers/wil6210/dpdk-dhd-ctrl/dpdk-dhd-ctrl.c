/* SPDX-License-Identifier: ISC
 * Copyright (c) 2019-2020, Facebook, Inc. All rights reserved.
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdatomic.h>

#include <err.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>

#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>

#include <rte_mbuf.h>
#include <rte_mempool.h>

#include "dpdk_dhd.h"
#include "dpdk-dhd-ctrl.h"

#include <wil6210_compat.h>

/* Compatibility with DPDK */
#ifndef RTE_LOG
#define RTE_LOG(level, type, args...)  fprintf(stderr, args)
#endif

static int
dhd_setup_queues(struct dhd_state *dhd)
{
	struct sockaddr_ll sockaddr;
	struct tpacket_req *req;
	struct dhd_rx_queue *rx_queue = NULL;
	struct dhd_tx_queue *tx_queue = NULL;
	int i, q, qsockfd, nb_queues = DHD_NUM_QUEUES;
	int rc, tpver, discard, qdisc_bypass;
	size_t rdsize;

	memset(&sockaddr, 0, sizeof(sockaddr));
	sockaddr.sll_family = AF_PACKET;
	sockaddr.sll_protocol = htons(DHD_ETH_TYPE_CFG);
	sockaddr.sll_ifindex = dhd->dhd_ifindex;

	req = &dhd->dhd_req;

	for (q = 0; q < nb_queues; q++) {
		/* Open an AF_PACKET socket for this queue... */
		qsockfd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
		if (qsockfd == -1) {
			rc = errno;
			RTE_LOG(ERR, PMD,
			        "%s: could not open AF_PACKET socket\n",
			        dhd->dhd_ifname);
			goto error;
		}

		rx_queue = &dhd->dhd_rxq[q];
		rx_queue->queue_fd = qsockfd;
		rx_queue->framecount = req->tp_frame_nr;
		rx_queue->frame_data_size = req->tp_frame_size;
		rx_queue->frame_data_size -= TPACKET2_HDRLEN -
			sizeof(struct sockaddr_ll) - sizeof(struct ethhdr) -
			sizeof(struct dhd_pkt_header);

		tpver = TPACKET_V2;
		rc = setsockopt(qsockfd, SOL_PACKET, PACKET_VERSION,
				&tpver, sizeof(tpver));
		if (rc == -1) {
			rc = errno;
			RTE_LOG(ERR, PMD,
				"%s: could not set PACKET_VERSION on AF_PACKET "
				"socket\n", dhd->dhd_ifname);
			goto error;
		}

		discard = 1;
		rc = setsockopt(qsockfd, SOL_PACKET, PACKET_LOSS,
				&discard, sizeof(discard));
		if (rc == -1) {
			rc = errno;
			RTE_LOG(ERR, PMD,
				"%s: could not set PACKET_LOSS on "
			        "AF_PACKET socket\n", dhd->dhd_ifname);
			goto error;
		}

		/*
		 * We create multiple sockets per device, all gobbling every
		 * single packet thet can lay ther eye on, so need to prevent
		 * sibling queues to steal packets injected into the kernel
		 * by anty other queue. The way we achive that is bys forcing
		 * packets to be forwarded into device_start_xmit directly,
		 * bypassing qdisc, where Linux kernel would normally let
		 * packet handlers to intercept the packets. By avoiding this
		 * path we do not give different AF_PACKET sockets the chance
		 * to steal each other's packets.
		 */
		qdisc_bypass = 1;
		rc = setsockopt(qsockfd, SOL_PACKET, PACKET_QDISC_BYPASS,
				&qdisc_bypass, sizeof(qdisc_bypass));
		if (rc == -1) {
			rc = errno;
			RTE_LOG(ERR, PMD,
				"%s: could not set PACKET_QDISC_BYPASS "
			        "on AF_PACKET socket\n", dhd->dhd_ifname);
			goto error;
		}

		rc = setsockopt(qsockfd, SOL_PACKET, PACKET_RX_RING, req, sizeof(*req));
		if (rc == -1) {
			rc = errno;
			RTE_LOG(ERR, PMD,
				"%s: could not set PACKET_RX_RING on AF_PACKET "
				"socket\n", dhd->dhd_ifname);
			goto error;
		}

		rc = setsockopt(qsockfd, SOL_PACKET, PACKET_TX_RING, req, sizeof(*req));
		if (rc == -1) {
			rc = errno;
			RTE_LOG(ERR, PMD,
				"%s: could not set PACKET_TX_RING on AF_PACKET "
				"socket\n", dhd->dhd_ifname);
			goto error;
		}

		rx_queue->data = mmap(NULL, 2 * req->tp_block_size * req->tp_block_nr,
				    PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED,
				    qsockfd, 0);
		if (rx_queue->data == MAP_FAILED) {
			RTE_LOG(ERR, PMD,
				"%s: call to mmap failed on AF_PACKET socket\n",
				dhd->dhd_ifname);
			rx_queue->data = NULL;
			goto error;
		}

		/* rdsize is same for both Tx and Rx */
		rdsize = req->tp_frame_nr * sizeof(*(rx_queue->rd));

		rx_queue->rd = calloc(1, rdsize);
		if (rx_queue->rd == NULL) {
			rc = errno;
			goto error;
		}
		for (i = 0; i < req->tp_frame_nr; ++i) {
			rx_queue->rd[i].iov_base = rx_queue->data +
			    (i * req->tp_frame_size);
			rx_queue->rd[i].iov_len = req->tp_frame_size;
		}

		tx_queue = &dhd->dhd_txq[q];
		tx_queue->queue_fd = qsockfd;
		tx_queue->framecount = req->tp_frame_nr;
		tx_queue->frame_data_size = req->tp_frame_size;
		tx_queue->frame_data_size -= TPACKET2_HDRLEN -
			sizeof(struct sockaddr_ll);

		tx_queue->data = rx_queue->data +
		    req->tp_block_size * req->tp_block_nr;

		tx_queue->rd = calloc(1, rdsize);
		if (tx_queue->rd == NULL) {
			rc = errno;
			goto error;
		}
		for (i = 0; i < req->tp_frame_nr; ++i) {
			tx_queue->rd[i].iov_base = tx_queue->data +
			    (i * req->tp_frame_size);
			tx_queue->rd[i].iov_len = req->tp_frame_size;
		}

		rc = bind(qsockfd, (const struct sockaddr*)&sockaddr, sizeof(sockaddr));
		if (rc == -1) {
			rc = errno;
			RTE_LOG(ERR, PMD,
				"%s: could not bind AF_PACKET socket\n",
			        dhd->dhd_ifname);
			goto error;
		}

		/* All queues, but first are for data */
		sockaddr.sll_protocol = htons(DHD_ETH_TYPE_DATA);
	}

	return 0;

error:
	if (rx_queue != NULL && rx_queue->rd != NULL) {
		free(rx_queue->rd);
		rx_queue->rd = NULL;
	}
	if (tx_queue != NULL && tx_queue->rd != NULL) {
		free(tx_queue->rd);
		tx_queue->rd = NULL;
	}
	return rc;
}

static int
dhd_dev_change_flags(struct dhd_state *dhd, uint32_t flags, uint32_t mask)
{
	struct ifreq ifr;
	int ret, s;

	s = socket(PF_INET, SOCK_DGRAM, 0);
	if (s < 0)
		return errno;

	snprintf(ifr.ifr_name, IFNAMSIZ, "%s", dhd->dhd_ifname);
	if (ioctl(s, SIOCGIFFLAGS, &ifr) < 0)
		goto out;
	ifr.ifr_flags &= mask;
	ifr.ifr_flags |= flags;
	if (ioctl(s, SIOCSIFFLAGS, &ifr) < 0)
		goto out;
	close(s);
	return 0;
out:
	ret = errno;
	close(s);
	return ret;
}

typedef int (*dhd_msg_fill_func)(struct dhd_state *dhd, void *arg, void *buf,
    size_t buf_size);

static int
dhd_send_locked(struct dhd_state *dhd, struct dhd_tx_queue *txq,
    dhd_msg_fill_func msg_func, void *msg_ctx)
{
	struct tpacket2_hdr *pkthdr;
	unsigned int framecount, framenum;
	struct pollfd pfd;
	uint8_t *pbuf;
	int rc;

	memset(&pfd, 0, sizeof(pfd));
	pfd.fd = txq->queue_fd;
	pfd.events = POLLOUT;
	pfd.revents = 0;

	/*
	 * Events and call responses come from different
	 * threads. Protect against collision
	 * */
	framecount = txq->framecount;
	framenum = txq->framenum;

	pkthdr = (struct tpacket2_hdr *)txq->rd[framenum].iov_base;
	if ((pkthdr->tp_status != TP_STATUS_AVAILABLE) == 0 &&
	    poll(&pfd, 1, -1) < 0)
		return EBUSY;

	pbuf = (uint8_t *)pkthdr + TPACKET2_HDRLEN -
	    sizeof(struct sockaddr_ll);
	rc = msg_func(dhd, msg_ctx, pbuf, txq->frame_data_size);
	if (rc < 0)
		return -rc;

	pkthdr->tp_len = rc;
	pkthdr->tp_snaplen = rc;

	pkthdr->tp_status = TP_STATUS_SEND_REQUEST;
	if (++framenum >= framecount)
		framenum = 0;
	txq->framenum = framenum;

	rc = sendto(txq->queue_fd, NULL, 0, MSG_DONTWAIT, NULL, 0);
	if (rc == -1) {
		rc = errno;
		RTE_LOG(ERR, PMD, "sendto failure: %s\n", strerror(rc));
		return rc;
	}
	return 0;
}

static int
dhd_send(struct dhd_state *dhd, struct dhd_tx_queue *txq,
    dhd_msg_fill_func msg_func, void *msg_ctx)
{
	int rc;

	pthread_mutex_lock(&dhd->dhd_send_mutex);
	rc  = dhd_send_locked(dhd, txq, msg_func, msg_ctx);
	pthread_mutex_unlock(&dhd->dhd_send_mutex);
	return rc;
}

static void *
dhd_fill_cfg_header(struct dhd_state *dhd, struct dhd_pkt_header *dhd_hdr,
    void *buffer, size_t buf_len)
{
	struct ethhdr ehdr;
	uint8_t *cur_p;
	size_t pkt_size;

	/* Make sure we can fit into the send buffer */
	pkt_size = sizeof(*dhd_hdr) + sizeof(ehdr) + dhd_hdr->cmd_len;
	if (pkt_size > buf_len)
		return NULL;
	/* Adjust length in header to cover whole packet */
	dhd_hdr->cmd_len = pkt_size;

	/* Fabricate the ethernet header */
	memset(ehdr.h_dest, 0xff, sizeof(ehdr.h_dest));
	memcpy(ehdr.h_source, dhd->dhd_ifaddr, sizeof(ehdr.h_source));
	ehdr.h_proto = htons(DHD_ETH_TYPE_CFG);

	/* Copy Ethernet header */
	cur_p = buffer;
	memcpy(cur_p, &ehdr, sizeof(ehdr));
	cur_p += sizeof(ehdr);

	/* Copy message header */
	memcpy(cur_p, dhd_hdr, sizeof(*dhd_hdr));
	cur_p += sizeof(*dhd_hdr);

	return cur_p;
}

struct dhd_msg_send_ctx {
	struct dhd_pkt_header hdr;
	const void *data;
	size_t len;
};

static int
dhd_fill_cfg_msg(struct dhd_state *dhd, void *arg, void *buf, size_t buf_len)
{
	struct dhd_msg_send_ctx *ctx;

	ctx = arg;
	buf = dhd_fill_cfg_header(dhd, &ctx->hdr, buf, buf_len);
	if (buf == NULL)
		return -EINVAL;
	memcpy(buf, ctx->data, ctx->len);
	return ctx->hdr.cmd_len;
}

int
dhd_send_event(struct dhd_state *dhd, const void *data, size_t len)
{
	struct dhd_msg_send_ctx ctx;

	memset(&ctx.hdr, 0, sizeof(ctx.hdr));
	ctx.hdr.cmd_code = DHD_CMDOP_EVENT;
	ctx.hdr.cmd_len = len;

	ctx.data = data;
	ctx.len = len;
	return dhd_send(dhd, &dhd->dhd_txq[0], dhd_fill_cfg_msg, &ctx);
}

static int
dhd_call_result(struct dhd_state *dhd, void *hdr, int result, const void *data,
    size_t len)
{
	struct dhd_msg_send_ctx ctx;

	/* Use incoming call header as a template */
	memcpy(&ctx.hdr, hdr, sizeof(ctx.hdr));
	ctx.hdr.cmd_result = result;
	ctx.hdr.cmd_len = len;

	ctx.data = data;
	ctx.len = len;
	return dhd_send(dhd, &dhd->dhd_txq[0], dhd_fill_cfg_msg, &ctx);
}

static void
dhd_rx_handler(struct dhd_state *dhd, uint8_t *data, size_t len)
{
	struct dhd_pkt_header dhd_hdr;
	struct ethhdr *ehdr;
	size_t pkt_meta_size;

	/* Validate data headers */
	pkt_meta_size = sizeof(dhd_hdr) + sizeof(*ehdr);
	if (len < pkt_meta_size) {
		RTE_LOG(ERR, PMD, "Packet length %ju is too short\n", len);
		return;
	}
	ehdr = (struct ethhdr *)data;
	if (ehdr->h_proto != htons(DHD_ETH_TYPE_CFG)) {
		RTE_LOG(ERR, PMD, "Unhandled packet type %04x\n",
		    ntohs(ehdr->h_proto));
		return;
	}

	/* Copy out our header */
	data += sizeof(*ehdr);
	memcpy(&dhd_hdr, data, sizeof(dhd_hdr));
	if (dhd_hdr.cmd_len != len) {
		RTE_LOG(ERR, PMD, "Invalid len in dhd header %u != %lu\n",
		    dhd_hdr.cmd_len, len);
		return;
	}
	/*
	 * Adjust data pointer and size to describe the packet
	 * payload.
	 */
	data += sizeof(dhd_hdr);
	len -= pkt_meta_size;

	if (dhd_hdr.cmd_code < DHD_CMDOP_FIRST ||
	    dhd_hdr.cmd_code > DHD_CMDOP_LAST) {
		RTE_LOG(ERR, PMD, "Unhandled cfg opcode %u\n",dhd_hdr.cmd_code);
	} else {
		struct dhd_call_desc req;
		int rc;

		req.call_opcode = dhd_hdr.cmd_code;
		req.call_data = data;
		req.call_len = len;
		req.resp_data = data;
		req.resp_len = dhd->dhd_rxq[0].frame_data_size;
		req.call_priv = &dhd_hdr;

		rc = -ENOENT;
		if (dhd->dhd_handler != NULL)
			rc = dhd->dhd_handler(dhd->dhd_handler_arg, &req);

		dhd_call_result(dhd, &dhd_hdr, rc, req.resp_data,
		    req.resp_len);
	}
}

static void
dhd_process_rxq(struct dhd_state *dhd, int qnum)
{
	struct dhd_rx_queue *rxq;
	struct tpacket2_hdr *pkthdr;
	unsigned int framecount, framenum;

	rxq = &dhd->dhd_rxq[qnum];

	framecount = rxq->framecount;
	framenum = rxq->framenum;

	for ( ; ; ) {
		/* Look for next packet from the kernel */
		pkthdr = (struct tpacket2_hdr *)rxq->rd[framenum].iov_base;
		if ((pkthdr->tp_status & TP_STATUS_USER) == 0)
			break;
		dhd_rx_handler(dhd, (uint8_t *)pkthdr +
		    pkthdr->tp_mac, pkthdr->tp_snaplen);
		/* Mark packet as free to use by the kernel */
		pkthdr->tp_status = TP_STATUS_KERNEL;
		if (++framenum >= framecount)
			framenum = 0;
	}
	rxq->framenum = framenum;
}

static int
dhd_poll(struct dhd_state *dhd)
{
	struct pollfd pfd[DHD_NUM_QUEUES + 1];
	int i, npoll, rc, nqueues;

	memset(&pfd, 0, sizeof(pfd));

	/* Wait for shutdown signal from parent */
	pfd[0].fd = dhd->dhd_pipe[0];
	pfd[0].events = POLLIN;
	npoll = 1;

	/* Do not poll for data queues */
	nqueues = 1;

	/* Wait for all queues at once */
	for (i = 0; i < nqueues; i++) {
		if (dhd->dhd_rxq[i].queue_fd < 0)
			break;
		pfd[npoll].fd = dhd->dhd_rxq[i].queue_fd;
		pfd[npoll].events = POLLIN;
		npoll++;
	}

	rc = poll(pfd, npoll, 2000);
	if (rc <= 0)
		return 0; /* Make this fatal? */

	/* Check of we were asked to shutdown */
	if (pfd[0].revents & POLLIN) {
		return 1;
	}

	/* Iterate over queues, process messages as necessary */
	for (i = 1; i < npoll; i++) {
		if (pfd[i].revents & POLLIN)
			dhd_process_rxq(dhd, i - 1);
	}
	return 0;
}

/*
 * Bulk RX APIs for data packets
 */
void
dhd_rx_setup(void *ctx, uint16_t in_port, struct rte_mempool *mb_pool)
{
	struct dhd_state *dhd;
	struct dhd_rx_queue *rxq;

	dhd = ctx;
	rxq = &dhd->dhd_rxq[1];

	rxq->mb_pool = mb_pool;
	rxq->in_port = in_port;
}

uint16_t
dhd_rx_burst(void *qdata, struct rte_mbuf **bufs, uint16_t nb_pkts)
{
	struct dhd_state *dhd;
	struct tpacket2_hdr *ppd;
	struct rte_mbuf *mbuf;
	uint8_t *pbuf;
	struct dhd_rx_queue *rxq;
	uint16_t num_rx = 0;
	unsigned long num_rx_bytes = 0;
	unsigned int framecount, framenum;
	unsigned int i, datalen, hdrlen;
	struct ethhdr *ehdr;
	struct dhd_data_header *pkthdr;

	if (unlikely(nb_pkts == 0))
		return 0;

	dhd = qdata;
	rxq = &dhd->dhd_rxq[1];

	framecount = rxq->framecount;
	framenum = rxq->framenum;
	for (i = 0; i < nb_pkts; i++) {
		/* point at the next incoming frame */
		ppd = (struct tpacket2_hdr *) rxq->rd[framenum].iov_base;
		if ((ppd->tp_status & TP_STATUS_USER) == 0)
			break;

		hdrlen = sizeof(*ehdr) + sizeof(*pkthdr);
		/* Check the packet size */
		if (ppd->tp_snaplen <= hdrlen) {
			goto next;
		}
		/* Check that this is indeed a data packet */
		ehdr = (struct ethhdr *)((uint8_t *)ppd + ppd->tp_mac);
		if (ehdr->h_proto != htons(DHD_ETH_TYPE_DATA)) {
		    RTE_LOG(ERR, PMD, "DHD RX: unexpected h_proto 0x%04x, len %u\n",
			ntohs(ehdr->h_proto), ppd->tp_snaplen);
		    goto next;
		}

		/* allocate the next mbuf */
		mbuf = rte_pktmbuf_alloc(rxq->mb_pool);
		if (unlikely(mbuf == NULL)) {
			break;
		}

		pkthdr = (struct dhd_data_header *)&ehdr[1];
		datalen = ppd->tp_snaplen - hdrlen;
		pbuf = (uint8_t *)ppd + ppd->tp_mac + hdrlen;

		/* packet will fit in the mbuf, go ahead and receive it */
		rte_pktmbuf_pkt_len(mbuf) = rte_pktmbuf_data_len(mbuf) = datalen;
		memcpy(rte_pktmbuf_mtod(mbuf, void *), pbuf, rte_pktmbuf_data_len(mbuf));

		mbuf->port = rxq->in_port;
		mbuf->udata64 = dhd->dhd_peer_link_id[pkthdr->peer_index];

		bufs[i] = mbuf;
		num_rx++;
		num_rx_bytes += mbuf->pkt_len;
next:
		ppd->tp_status = TP_STATUS_KERNEL;
		if (++framenum >= framecount)
			framenum = 0;
	}
	rxq->framenum = framenum;
	rxq->rx_pkts += num_rx;
	rxq->rx_bytes += num_rx_bytes;
	return num_rx;
}

/*
 * Callback to handle sending packets through a real NIC.
 */
uint16_t
dhd_tx_burst(void *qdata, struct rte_mbuf **bufs, uint16_t nb_pkts)
{
	struct dhd_state *dhd;
	struct tpacket2_hdr *ppd;
	struct rte_mbuf *mbuf;
	uint8_t *pbuf;
	unsigned int framecount, framenum;
	struct pollfd pfd;
	struct dhd_tx_queue *txq;
	uint16_t num_tx = 0;
	unsigned long num_tx_bytes = 0;
	unsigned int datalen, hdrlen;
	struct ethhdr *ehdr;
	struct dhd_data_header *pkthdr;
	int i;

	if (unlikely(nb_pkts == 0))
		return 0;

	dhd = qdata;
	txq = &dhd->dhd_txq[1];

	memset(&pfd, 0, sizeof(pfd));
	pfd.fd = txq->queue_fd;
	pfd.events = POLLOUT;
	pfd.revents = 0;

	framecount = txq->framecount;
	framenum = txq->framenum;
	ppd = (struct tpacket2_hdr *) txq->rd[framenum].iov_base;
	for (i = 0; i < nb_pkts; i++) {
		mbuf = *bufs++;

		hdrlen = sizeof(*ehdr) + sizeof(*pkthdr);
		datalen = mbuf->pkt_len + hdrlen;

		/* drop oversized packets */
		if (datalen > txq->frame_data_size) {
			rte_pktmbuf_free(mbuf);
			continue;
		}

		/* point at the next incoming frame */
		if ((ppd->tp_status != TP_STATUS_AVAILABLE) &&
		    (poll(&pfd, 1, -1) < 0))
			break;

		/* copy the tx frame data */
		pbuf = (uint8_t *) ppd + TPACKET2_HDRLEN -
			sizeof(struct sockaddr_ll);

		/* Fabricate dhd headers */
		ehdr = (struct ethhdr *)pbuf;
		memcpy(pbuf, rte_pktmbuf_mtod(mbuf, struct ethhdr *),
		    sizeof(*ehdr));
		ehdr->h_proto = htons(DHD_ETH_TYPE_DATA);
		pkthdr = (struct dhd_data_header *)&ehdr[1];

		/* Fill in the terraX unit number */
		pkthdr->peer_index = (int)mbuf->udata64;

		pbuf += hdrlen;

		struct rte_mbuf *tmp_mbuf = mbuf;
		while (tmp_mbuf) {
			uint16_t data_len = rte_pktmbuf_data_len(tmp_mbuf);
			memcpy(pbuf, rte_pktmbuf_mtod(tmp_mbuf, void*), data_len);
			pbuf += data_len;
			tmp_mbuf = tmp_mbuf->next;
		}

		ppd->tp_len = datalen;
		ppd->tp_snaplen = datalen;

		/* release incoming frame and advance ring buffer */
		ppd->tp_status = TP_STATUS_SEND_REQUEST;
		if (++framenum >= framecount)
			framenum = 0;
		ppd = (struct tpacket2_hdr *) txq->rd[framenum].iov_base;

		num_tx++;
		num_tx_bytes += datalen;
		rte_pktmbuf_free(mbuf);
	}

	/* kick-off transmits */
	if (sendto(txq->queue_fd, NULL, 0, MSG_DONTWAIT, NULL, 0) == -1) {
		/* error sending -- no packets transmitted */
		num_tx = 0;
		num_tx_bytes = 0;
	}

	txq->framenum = framenum;
	txq->tx_pkts += num_tx;
	txq->err_pkts += i - num_tx;
	txq->tx_bytes += num_tx_bytes;
	return i;
}

struct dhd_state *
dhd_init(void)
{
	struct dhd_state *dhd;
	int i;

	dhd = calloc(1, sizeof(*dhd));
	if (dhd == NULL)
		return NULL;
	dhd->dhd_fd = -1;
	dhd->dhd_ifindex = -1;
	dhd->dhd_pipe[0] = -1;
	dhd->dhd_pipe[1] = -1;

	for (i = 0; i < DHD_NUM_QUEUES; i++) {
		dhd->dhd_rxq[i].queue_fd = -1;
		dhd->dhd_txq[i].queue_fd = -1;
	}

	for (i = 0; i < DHD_MAX_PEERS; i++) {
		dhd->dhd_peer_link_id[i] = -1;
	}

	/* Numbers that need to be tuned */
	dhd->dhd_req.tp_block_size = 8192;
	dhd->dhd_req.tp_block_nr = 128;
	dhd->dhd_req.tp_frame_size = 8192;
	dhd->dhd_req.tp_frame_nr = 128;

	pthread_mutex_init(&dhd->dhd_send_mutex, 0);

	return dhd;
}

void
dhd_fini(struct dhd_state *dhd)
{
	struct tpacket_req *req;
	int i;

	if (dhd == NULL)
		return;

	req = &dhd->dhd_req;
	for (i = 0; i < DHD_NUM_QUEUES; i++) {
		free(dhd->dhd_rxq[i].rd);
		free(dhd->dhd_txq[i].rd);
		if (dhd->dhd_rxq[i].queue_fd >= 0)
			close(dhd->dhd_rxq[i].queue_fd);
		if (dhd->dhd_rxq[i].data != NULL)
			munmap(dhd->dhd_rxq[i].data,
			    2 * req->tp_block_size * req->tp_block_nr);
	}

	pthread_mutex_destroy(&dhd->dhd_send_mutex);
	close(dhd->dhd_fd);
	free(dhd);
}

static int
dhd_fd_set_flags(int fd, int flags)
{
	int rc;

	rc = fcntl(fd, F_GETFL);
	if (rc < 0)
		return rc;
	return fcntl(fd, F_SETFL, rc | flags);
}

int
dhd_attach(struct dhd_state *dhd, uint8_t mac_addr[],
	struct rte_pci_addr pci_addr)
{
	struct dhd_attach_request attach;
	int rc;

	dhd->dhd_fd = open("/dev/dhd", O_CLOEXEC | O_RDWR);
	if (dhd->dhd_fd == -1)
		return errno;

	memset(&attach, 0, sizeof(attach));
	memcpy(attach.mac_addr, mac_addr, sizeof(attach.mac_addr));
	attach.pci_domain = pci_addr.domain;
	attach.pci_bus = pci_addr.bus;
	attach.pci_devid = pci_addr.devid;
	attach.pci_function = pci_addr.function;

	rc = ioctl(dhd->dhd_fd, DPDK_DHD_ATTACH, &attach);
	if (rc < 0) {
		rc = errno;
		RTE_LOG(ERR, PMD, "Unable to attach to dhd: %s\n",
		    strerror(rc));
		return rc;
	}

	/*
	 * We know what interface to use for communication with the
	 * kernel, save the information for later.
	 */
	dhd->dhd_ifindex = attach.if_index;
	strlcpy(dhd->dhd_ifname, attach.if_name, IFNAMSIZ);
	memcpy(dhd->dhd_ifaddr, attach.mac_addr, sizeof(dhd->dhd_ifaddr));

	/* Setup socket queues for faster communication */
	rc = dhd_setup_queues(dhd);
	if (rc != 0)
		return rc;

	return 0;
}

static void *
dhd_poll_worker_thread(void *arg)
{
	struct dhd_state *dhd = arg;

	for (/*none*/; /*none*/; /*none*/) {
		if (dhd_poll(dhd) != 0)
			break;
	}
	return NULL;
}

static void
dhd_stop_poll_worker(struct dhd_state *dhd)
{
	int i, rc;
	char tmp;

	if (dhd->dhd_pipe[1] >= 0) {
		/* Wake up worker and tell it to quit */
		do {
			rc = write(dhd->dhd_pipe[1], &tmp, sizeof(tmp));
		} while (rc == -1 && errno == EAGAIN);

		/* Wait for worker to exit */
		pthread_join(dhd->dhd_poll_thread, NULL);
	}

	for (i = 0; i < 2; i++) {
		if (dhd->dhd_pipe[i] >= 0) {
			close(dhd->dhd_pipe[i]);
			dhd->dhd_pipe[i] = -1;
		}
	}
}

static int
dhd_start_poll_worker(struct dhd_state *dhd)
{
	rte_cpuset_t cpuset;
	int i, rc;

	/* Initialize pipes */
	rc = pipe(dhd->dhd_pipe);
	if (rc == -1) {
		rc = errno;
		RTE_LOG(ERR, PMD, "Unable to create pipes: %s\n",
		    strerror(rc));
		return rc;
	}
	for (i = 0; i < 2; i++) {
		rc = dhd_fd_set_flags(dhd->dhd_pipe[i], O_NONBLOCK | O_CLOEXEC);
		if (rc == -1) {
			rc = errno;
			RTE_LOG(ERR, PMD, "Unable to setup pipes: %s\n",
			    strerror(rc));
			goto fail;
		}
	}

	rc = pthread_create(&dhd->dhd_poll_thread, NULL,
	    dhd_poll_worker_thread, dhd);
	if (rc != 0) {
		RTE_LOG(ERR, PMD, "Unable to create poller thread: %s\n",
		    strerror(errno));
		goto fail;
	}

	/* Run control threads on master lcore and set its name */
	CPU_ZERO(&cpuset);
	CPU_SET(rte_get_master_lcore(), &cpuset);
	rc = pthread_setaffinity_np(dhd->dhd_poll_thread, sizeof(cpuset),
	    &cpuset);
	if (rc != 0) {
		RTE_LOG(ERR, PMD, "Unable to set poller thread affinity: %s\n",
		    strerror(errno));
		dhd_stop_poll_worker(dhd);
		return rc;
	}

	/* This function is free to fail */
	char n[32];
	snprintf(n, sizeof(n), "poll-%s", dhd->dhd_ifname);
	(void)rte_thread_setname(dhd->dhd_poll_thread, n);

	return 0;
fail:
	for (i = 0; i < 2; i++) {
		if (dhd->dhd_pipe[i] >= 0) {
			close(dhd->dhd_pipe[i]);
			dhd->dhd_pipe[i] = -1;
		}
	}
	return rc;
}

int
dhd_start(struct dhd_state *dhd)
{
	socklen_t optlen;
	int i, rc, soerror;

	/* Mark interface as UP */
	rc = dhd_dev_change_flags(dhd, IFF_UP, ~0u);
	if (rc != 0)
		return rc;

	/*
	 * We bind AF_SOCKET before device is up, and that sets the
	 * sticky socket error, this in turn causes the first sendto
	 * to fail on that socket. Clear socket error here instead and
	 * let the rest of the code to proceed undisturbed. Alternative
	 * is to bring interface UP defore binding to it or bind to it
	 * here, this code just happens to be written this way.
	 */
	for (i = 0; i < DHD_NUM_QUEUES; i++) {
		optlen = sizeof(soerror);
		(void)getsockopt(dhd->dhd_txq[i].queue_fd, SOL_SOCKET,
		    SO_ERROR, &soerror, &optlen);
	}

	/* Start polling */
	rc = dhd_start_poll_worker(dhd);
	if (rc != 0)
		return rc;

	/* We are ready to serve requests from the driver */
	rc = ioctl(dhd->dhd_fd, DPDK_DHD_START);
	if (rc < 0) {
		rc = errno;
		RTE_LOG(ERR, PMD, "Unable to start dhd: %s\n",
		    strerror(errno));
		return rc;
	}

	return 0;
}

void
dhd_stop(struct dhd_state *dhd)
{
	dhd_stop_poll_worker(dhd);
}

int
dhd_pcie_retrain(struct dhd_state *dhd, struct rte_pci_addr pci_addr)
{
	struct dhd_pcie_retrain_request retrain;
	int rc;

	retrain.domain = pci_addr.domain;
	retrain.bus = pci_addr.bus;
	retrain.devid = pci_addr.devid;
	retrain.function = pci_addr.function;

	rc = ioctl(dhd->dhd_fd, DPDK_DHD_PCIE_RETRAIN, &retrain);
	if (rc < 0) {
		rc = errno;
		RTE_LOG(ERR, PMD, "pcie_retrain failed: %s\n",
		    strerror(rc));
		return rc;
	}

	return 0;
}
