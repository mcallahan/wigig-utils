/*-
 *   BSD LICENSE
 *
 *   Copyright (C) IGEL Co.,Ltd.
 *   All rights reserved.
 *   Copyright (C) 2019-2020, Facebook, Inc. All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of IGEL Co.,Ltd. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <rte_mbuf.h>
#include <rte_ethdev.h>
#include <rte_ethdev_pci.h>
#include <rte_malloc.h>
#include <rte_memcpy.h>
#include <rte_bus_pci.h>
#include <rte_spinlock.h>

#include "wil6210_ethdev.h"
#include "slave.h"
#include "txrx.h"

struct pmd_internals;

struct ethdev_tx_queue {
	struct wil6210_vif *vif;
	struct wil_ring *ring;
};

struct pmd_internals {
	struct ethdev_tx_queue tx_ethdev_queues[WIL6210_MAX_TX_RINGS];
	struct ether_addr eth_addr;
	struct wil6210_priv *wil;
};


static struct rte_eth_link pmd_link = {
	.link_speed = ETH_SPEED_NUM_2_5G,
	.link_duplex = ETH_LINK_FULL_DUPLEX,
	.link_status = ETH_LINK_DOWN,
	.link_autoneg = ETH_LINK_FIXED,
};

struct rte_wil6210_xstats_name_off {
	char name[RTE_ETH_XSTATS_NAME_SIZE];
	unsigned offset;
};

struct rte_wil6210_xstats_name_off_acc {
	char name[RTE_ETH_XSTATS_NAME_SIZE];
	unsigned offset;
	unsigned acc_offset;
};

static const struct rte_wil6210_xstats_name_off rte_stats_ul_strings[] = {
	{ "rx_dropped", offsetof(struct wil_net_stats, rx_dropped) },
	{ "rx_non_data_frame",
	  offsetof(struct wil_net_stats, rx_non_data_frame) },
	{ "rx_short_frame", offsetof(struct wil_net_stats, rx_short_frame) },
	{ "rx_large_frame", offsetof(struct wil_net_stats, rx_large_frame) },
	{ "rx_mic_error", offsetof(struct wil_net_stats, rx_mic_error) },
	{ "rx_key_error", offsetof(struct wil_net_stats, rx_key_error) },
	{ "rx_amsdu_error", offsetof(struct wil_net_stats, rx_amsdu_error) },
	{ "wil_tx_ring_calls",
	  offsetof(struct wil_net_stats, wil_tx_ring_calls) },
	{ "wil_tx_ring_full",
	  offsetof(struct wil_net_stats, wil_tx_ring_full) },
	/* only add unsigned longs here */
};

static const struct rte_wil6210_xstats_name_off rte_port_stats_u64_strings[] = {
	{ "rx_burst_calls",
	  offsetof(struct wil6210_priv, count_rx_burst_calls) },
	{ "rx_burst_full",
	  offsetof(struct wil6210_priv, count_rx_burst_full) },
	{ "no_credits",
	  offsetof(struct wil6210_priv, no_credits) },
#ifdef DEBUG_LATENCY
	{ "rx_handler_time_nano",
	  offsetof(struct wil6210_priv, wil_rx_handler_time_nano) },
#endif
#ifdef ENABLE_PMC_STATS
	{ "tx_handler_time_nano",
	  offsetof(struct wil6210_priv, wil_tx_handler_time_nano) },
#endif
	/* only add u64's here */
};

static const
struct rte_wil6210_xstats_name_off rte_port_stats_u64_abs_strings[] = {
#ifdef DEBUG_LATENCY
	{ "max_rx_diff_nano",
		offsetof(struct wil6210_priv, max_rx_diff_nano) },
#endif
	/* only add u64's here */
};


/* Note: The stats below are not monotonic.  Since pktgen only displays the
 * rate of change and not the absolute value of stats, we accumulate the
 * values so they get properly reported by pktgen.
 */
static const struct rte_wil6210_xstats_name_off_acc rte_stats_atomic_strings[] = {
	{ "tx_pend_packets", offsetof(struct wil_net_stats, tx_pend_packets),
	  offsetof(struct wil_net_stats, tx_pend_packets_acc) },
	/* only add atomic_t's here */
};

/* stats collected on per port x per STA basis */
#define WIL6210_NB_UL_STATS                                                    \
	(sizeof(rte_stats_ul_strings) / sizeof(rte_stats_ul_strings[0]))

#define WIL6210_NB_ATOMIC_STATS                                                \
	(sizeof(rte_stats_atomic_strings) / sizeof(rte_stats_atomic_strings[0]))

/* stats collected on per port basis */
#define WIL6210_NB_U64_PORT_STATS                                              \
	(sizeof(rte_port_stats_u64_strings) / sizeof(rte_port_stats_u64_strings[0]))

#define WIL6210_NB_U64_ABS_PORT_STATS                                          \
	(sizeof(rte_port_stats_u64_abs_strings) ?                              \
		 (sizeof(rte_port_stats_u64_abs_strings) /                     \
		  sizeof(rte_port_stats_u64_abs_strings[0])) :                 \
		 0)

static inline struct pmd_internals *
eth_get_priv(struct rte_eth_dev *eth_dev)
{
	struct wil6210_priv *wil;

	wil = eth_dev->data->dev_private;
	return wil->eth_priv;
}

uint16_t wil6210_burst_rx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs);
uint16_t wil6210_burst_tx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs);
int wil6210_tx_feedback(void *q, uint32_t *avail, uint32_t *pending,
			uint32_t nb_peers);
int wil6210_tx_get_pipe_size(void *q, uint16_t *size, uint32_t nb_peers);

int wil6210_xstats_config(struct rte_eth_dev *dev, uint32_t flags);

static inline void
eth_set_priv(struct rte_eth_dev *eth_dev,
    	     struct pmd_internals *internals)
{
	struct wil6210_priv *wil;

	wil = eth_dev->data->dev_private;
	wil->eth_priv = internals;
}

uint16_t
wil6210_burst_rx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	if (unlikely(q == NULL || bufs == NULL))
		return 0;

	return wil_rx_burst(q, bufs, nb_bufs);
}

/* Forward declation */
static uint16_t wil_tx_burst(void *qdata, struct rte_mbuf **bufs,
			     uint16_t nb_pkts);

uint16_t
wil6210_burst_tx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	if (unlikely(q == NULL || bufs == NULL))
		return 0;

	return wil_tx_burst(q, bufs, nb_bufs);
}

/*
 * For each possible peer, reports the size of the TX ring.
 */
int
wil6210_tx_get_pipe_size(void *q, uint16_t *size,
		    uint32_t nb_peers)
{
	struct pmd_internals *internals = q;
	if (unlikely(q == NULL || size == NULL
			|| nb_peers > ARRAY_SIZE(internals->tx_ethdev_queues)))
		return -EINVAL;
	struct wil6210_priv *wil = internals->wil;

	for (int peer=0; peer < nb_peers; peer++) {
		*size++ = wil_tx_ring_config_size();
	}
	return 0;
}

/*
 * For each possible peer, reports the number of pending and
 * available entries within the Tx Ring.
 *
 * pending is optional and can be set to NULL
 */
int
wil6210_tx_feedback(void *q, uint32_t *avail, uint32_t *pending,
		    uint32_t nb_peers)
{
	struct pmd_internals *internals = q;
	if (unlikely(q == NULL || avail == NULL
			|| nb_peers > ARRAY_SIZE(internals->tx_ethdev_queues)))
		return -EINVAL;

	struct ethdev_tx_queue *txq;
	struct wil6210_priv *wil = NULL;
	struct wil_ring *ring;
	unsigned i;
	int rc = 0;

	wil = internals->wil;
	if (test_bit(wil_status_suspending, wil->status) ||
		test_bit(wil_status_suspended, wil->status) ||
		test_bit(wil_status_resuming, wil->status)) {
		wil_dbg_txrx(wil,
			 "suspend/resume in progress. block credits.\n");
		return -EAGAIN;
	}

	for (int peer=0; peer < nb_peers; peer++) {
		txq = &internals->tx_ethdev_queues[peer];

		ring = txq->ring;
		/* Unlocked access to ring. */
		if (unlikely(ring == NULL)) {
			*avail++ = 0;
			// once pending is being used, change unlikely to likely
			// for this NULL check across the whole function
			if (unlikely(pending != NULL)) {
				*pending++ = 0;
			}
			continue;
		}

		int ring_index = ring - wil->ring_tx;
		struct wil_ring_tx_data *txdata = &wil->ring_tx_data[ring_index];

		spin_lock(&txdata->lock);
		if (txdata->enabled) {
			int v = wil_ring_avail_tx(ring);
			*avail++ = v;
			rc += v;

			if (unlikely(pending != NULL)) {
				struct wil_sta_info *sta = &wil->sta[txdata->cid];
				*pending++ = atomic_read(&sta->stats.tx_pend_bytes);
			}
		} else {
			*avail++ = 0;

			if (unlikely(pending != NULL)) {
				*pending++ = 0;
			}
		}
		spin_unlock(&txdata->lock);
	}

	if (wil && rc == 0) {
		wil->no_credits ++;
	}
	return rc;
}


static uint16_t
wil6210_burst_rx_noop(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	return 0;
}

static uint16_t
wil6210_burst_tx_noop(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	return 0;
}

static int
wil6210_tx_feedback_noop(void *q, uint32_t *avail, uint32_t *pending,
			 uint32_t nb_peers)
{
	return -ENETDOWN;
}

static int
wil6210_send_queue_stats_noop(void *q, struct rte_eth_tx_pending_stats *pending,
			      uint32_t nb_peers)
{
	return -ENETDOWN;
}

int
wil6210_xstats_config(struct rte_eth_dev *dev, uint32_t flags)
{
	struct wil6210_priv *wil;

	if (unlikely(dev == NULL))
		return -EINVAL;
	wil = dev->data->dev_private;
	wil->xstats_flags = flags;

#ifdef ENABLE_PMC_STATS
	if (!(wil->xstats_flags &
			XSTATS_FLAGS_PMC_COUNTERS_3|XSTATS_FLAGS_PMC_COUNTERS_6)) {
		for (int i=0; i < PMC_NUM_COUNTER_INSTANCES; i++) {
			for (int j=0; j < RTE_PMC_NUM_COUNTERS; j++)
				wil->pmc[i][j] = 0;
		}
	}
#endif
	return 0;
}

static int
wil6210_send_queue_stats(void *q, struct rte_eth_tx_pending_stats *pending,
			 uint32_t nb_peers)
{
	struct pmd_internals *internals = q;
	struct wil6210_priv *wil = NULL;
	struct ethdev_tx_queue *txq;
	int peer, num_links;
	struct wil_ring *ring;
	int ring_index;
	struct wil_ring_tx_data *txdata;
	u8 cid;
	struct wil_sta_info *stas[WIL6210_MAX_TX_RINGS] = { NULL };
	struct wil_sta_info *sta;
	u32 bytes_pending[WIL6210_MAX_TX_RINGS];
	u32 arrival_rate[WIL6210_MAX_TX_RINGS];

	if (unlikely(q == NULL || pending == NULL ||
		     nb_peers > WIL6210_MAX_TX_RINGS))
		return -EINVAL;

	wil = internals->wil;

	if (!wil)
		return -EINVAL;

	if (test_bit(wil_status_suspending, wil->status) ||
	    test_bit(wil_status_suspended, wil->status) ||
	    test_bit(wil_status_resuming, wil->status)) {
		return -EAGAIN;
	}

	/* Organize per peer information */
	for (peer = 0; peer < nb_peers; peer++) {
		txq = &internals->tx_ethdev_queues[peer];

		ring = txq->ring;
		if (unlikely(ring == NULL)) {
			continue;
		}

		ring_index = ring - wil->ring_tx;
		txdata = &wil->ring_tx_data[ring_index];

		spin_lock(&txdata->lock);
		if (txdata->enabled) {
			cid = txdata->cid;
		} else {
			spin_unlock(&txdata->lock);
			continue;
		}
		spin_unlock(&txdata->lock);

		/* check if this is a peer with a link up*/
		sta = &wil->sta[cid];
		if (sta->status != wil_sta_connected || sta->mid != 0) {
			continue;
		}
		if (sta->peer_id != peer) {
			wil_err(wil,
				"index of ethdev_tx_queue (%u) does not match the peer_id of \
				its corresponding wil_sta_info (%u)\n",
				peer, sta->peer_id);
			continue;
		}

		/*
		 * wil_sta_info structs for each peer in wil6210_priv are ordered by
		 * CID, but we gather them by order of peer index to make them parallel
		 * with pending stats and form the queue stats ioctl
		 */
		stas[peer] = sta;
		bytes_pending[peer] = pending[peer].bytes;
		arrival_rate[peer] = pending[peer].arrival_rate;
	}

	return wil_send_queue_stats_ioctl_async(wil, bytes_pending,
						arrival_rate, stas);
}

static int
eth_dev_configure(struct rte_eth_dev *dev __rte_unused)
{
	return 0;
}

static int
eth_dev_start(struct rte_eth_dev *dev)
{
	struct wil6210_priv *wil;
	int rc;

	if (dev == NULL)
		return -EINVAL;

	wil = dev->data->dev_private;

	/* Setup DBG log enable bitmask.
	 * Note that in PUMA logs are configured at VPP init time,
	 * and cannot be set dynamically.
	 */
	wil_set_log_dbg_mask(&wil->dbg_log_enable_mask);

	/* Make sure there are no pending IRQs */
	wil6210_clear_irq(wil);
	rc = wil6210_init_irq(wil, 0);
	if (rc)
		goto release_irq;

	rc = wil_if_add(wil);
	if (rc)
		goto release_irq;

	dhd_rx_setup(wil->dhd, dev->data->port_id, wil->ring_rx.mpool);

	dev->data->dev_link.link_status = ETH_LINK_UP;

	if (wil->fw_log_state || wil->ucode_log_state)
		wil_fw_log_start_poll_worker(wil);

	dev->rx_pkt_burst = wil6210_burst_rx;
	dev->tx_pkt_burst = wil6210_burst_tx;
	dev->tx_get_feedback = wil6210_tx_feedback;
	dev->tx_send_pending_stats = wil6210_send_queue_stats;

	return 0;
release_irq:
	wil6210_fini_irq(wil, 0);
	return rc;
}

static void
eth_dev_stop(struct rte_eth_dev *dev)
{
	struct wil6210_priv *wil;

	if (dev == NULL)
		return;

	wil = dev->data->dev_private;

	wil_delete_peers(wil);

	dev->rx_pkt_burst = wil6210_burst_rx_noop;
	dev->tx_pkt_burst = wil6210_burst_tx_noop;
	dev->tx_get_feedback = wil6210_tx_feedback_noop;
	dev->tx_send_pending_stats = wil6210_send_queue_stats_noop;

	if (wil->fw_log_state || wil->ucode_log_state)
		wil_fw_log_stop_poll_worker(wil);

	wil_if_remove(wil);
	wil6210_fini_irq(wil, 0);

	dev->data->dev_link.link_status = ETH_LINK_DOWN;
}

static void
eth_dev_close(struct rte_eth_dev *dev)
{
	if (dev == NULL)
		return;

	wil6210_dev_uninit(dev);
}

static int
eth_rx_queue_setup(struct rte_eth_dev *dev, uint16_t rx_queue_id,
		uint16_t nb_rx_desc __rte_unused,
		unsigned int socket_id __rte_unused,
		const struct rte_eth_rxconf *rx_conf __rte_unused,
		struct rte_mempool *mb_pool)
{
	struct wil6210_priv *wil;

	if (dev == NULL || mb_pool == NULL)
		return -EINVAL;

	if (rx_queue_id >= dev->data->nb_rx_queues)
		return -ENODEV;

	wil = dev->data->dev_private;

	/* Hardware RX queues */
	wil->ring_rx.mpool = mb_pool;
	wil->ring_rx.port_id = dev->data->port_id;

	dev->data->rx_queues[rx_queue_id] = wil;
	return 0;
}

int wil_tx_ring(struct wil6210_priv *wil, struct wil6210_vif *vif,
		struct wil_ring *ring, struct rte_mbuf *skb);

static int
wil_eth_ring_find(struct wil6210_priv *wil, unsigned int cid)
{
	int i;
	int min_ring_id = wil_get_min_tx_ring_id(wil);

	for (i = min_ring_id; i < ARRAY_SIZE(wil->ring2cid_tid); i++) {
		if (wil->ring2cid_tid[i][0] == cid)
			return i;
	}
	return -1;
}

void
wil_eth_tx_ring_enable(struct wil6210_priv *wil, unsigned int cid,
		       unsigned int qid)
{
	struct pmd_internals *internals;
	struct ethdev_tx_queue *txq;
	int ring_id;

	ring_id = wil_eth_ring_find(wil, cid);
	if (ring_id < 0)
		return;

	internals = wil->eth_priv;
	txq = &internals->tx_ethdev_queues[qid];

	txq->ring = &wil->ring_tx[ring_id];
	wmb();
}

void
wil_eth_tx_ring_disable(struct wil6210_priv *wil, unsigned int cid,
		        unsigned int qid)
{
	struct pmd_internals *internals;
	struct ethdev_tx_queue *txq;

	internals = wil->eth_priv;
	txq = &internals->tx_ethdev_queues[qid];

	txq->ring = NULL;
	wmb();
}

static inline uint16_t
wil_tx_burst_peer(struct pmd_internals *internals, uint16_t peer,
		  struct rte_mbuf **bufs, uint16_t nb_pkts)
{
	struct ethdev_tx_queue *txq;
	struct wil6210_priv *wil;
	struct wil6210_vif *vif;
	struct wil_ring *ring;
	unsigned i;
	int rc;

	txq = &internals->tx_ethdev_queues[peer];

	wil = internals->wil;
	vif = txq->vif;
	ring = txq->ring;

	/*
	 * Note: unlocked access to ring pointer to verify
	 * if corresponding TX ring should be polled or not
	 */
	if (unlikely(ring == NULL)) {
		/*
		 * These are not accounted for as they really are not
		 * expected and we have no STA to keep the track of
		 * them.
		 */
		for (i = 0; i < nb_pkts; i++)
		    rte_pktmbuf_free(bufs[i]);
		return nb_pkts;
	}

	for (i = 0; i < nb_pkts; i++) {
		struct rte_mbuf *mbuf = *bufs++;

		rc = wil_tx_ring(wil, vif, ring, mbuf);
		if (unlikely(rc < 0)) {
			if (unlikely(rc != -ENOMEM)) {
				rte_pktmbuf_free(mbuf);
				i++; // Consume the packet.
			}
			break;
		}
	}

	return i;
}

static uint16_t
wil_tx_burst(void *qdata, struct rte_mbuf **tx_pkts, uint16_t nb_pkts)
{
	struct pmd_internals *internals;
	struct rte_mbuf *mbuf;
	uint16_t tx_done, tx_batch, tx_left, tx_total;

	internals = qdata;
	tx_total = 0;
	tx_left = nb_pkts;
	struct wil6210_priv *wil = internals->wil;

#ifdef ENABLE_PMC_STATS
	u64 prev_count[RTE_PMC_NUM_COUNTERS];
	/* Use first index of first peer */
	u64 start = rte_get_timer_cycles();
	if (wil->xstats_flags & XSTATS_FLAGS_PMC_COUNTERS_3) {
		prev_count[0] = rte_read_perf_counter_0();
		prev_count[1] = rte_read_perf_counter_1();
		prev_count[2] = rte_read_perf_counter_2();
	}
#endif

	while (tx_left > 0) {
		mbuf = tx_pkts[0];

		for (tx_batch = 1; tx_batch < tx_left; tx_batch++)
			if (mbuf->udata64 != tx_pkts[tx_batch]->udata64)
				break;

		tx_done = wil_tx_burst_peer(internals, mbuf->udata64, tx_pkts,
					    tx_batch);
		tx_total += tx_done;
		if (tx_done < tx_batch)
			break;

		tx_pkts += tx_done;
		tx_left -= tx_done;
	}

#ifdef ENABLE_PMC_STATS
	if (wil->xstats_flags & XSTATS_FLAGS_PMC_COUNTERS_3) {
		wil->pmc[PMC_COUNTERS_TX_OUTER][0] +=
			rte_pmc_sub_counters(prev_count[0], rte_read_perf_counter_0());
		wil->pmc[PMC_COUNTERS_TX_OUTER][1] +=
			rte_pmc_sub_counters(prev_count[1], rte_read_perf_counter_1());
		wil->pmc[PMC_COUNTERS_TX_OUTER][2] +=
			rte_pmc_sub_counters(prev_count[2], rte_read_perf_counter_2());
	}
	wil->wil_tx_handler_time_nano +=
		(rte_get_timer_cycles() - start) * wil->nano_per_cycle;
#endif

	return tx_total;
}

static int
eth_tx_queue_setup(struct rte_eth_dev *dev, uint16_t tx_queue_id,
		uint16_t nb_tx_desc __rte_unused,
		unsigned int socket_id __rte_unused,
		const struct rte_eth_txconf *tx_conf __rte_unused)
{
	struct wil6210_priv *wil;
	struct pmd_internals *internals;
	struct ethdev_tx_queue *txq;
	int i;

	if (dev == NULL)
		return -EINVAL;

	internals = eth_get_priv(dev);

	if (tx_queue_id >= dev->data->nb_tx_queues)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(internals->tx_ethdev_queues); i++) {
		txq = &internals->tx_ethdev_queues[i];

		wil = dev->data->dev_private;
		txq->vif = ndev_to_vif(wil->main_ndev);
	}

	dev->data->tx_queues[tx_queue_id] = internals;

	return 0;
}

static int eth_fw_version_get(struct rte_eth_dev *dev, char *fw_version,
			      size_t fw_size)
{
	struct wil6210_priv *wil;
	int ret;

	wil = dev->data->dev_private;

	ret = snprintf(fw_version, fw_size, "%s", wil->fw_version);
	ret += 1; /* add the size of '\0' */
	if (fw_size < (uint32_t)ret)
		return ret;
	else
		return 0;
}

static void
eth_dev_info(struct rte_eth_dev *dev, struct rte_eth_dev_info *dev_info)
{
	if (dev == NULL || dev_info == NULL)
		return;

	dev_info->max_mac_addrs = 1;
	dev_info->max_rx_pktlen = (uint32_t)-1;
	dev_info->max_rx_queues = 1;
	dev_info->max_tx_queues = 1;
	dev_info->min_rx_bufsize = 1;

	dev_info->rx_offload_capa = DEV_RX_OFFLOAD_CHECKSUM;
	dev_info->tx_offload_capa = DEV_TX_OFFLOAD_IPV4_CKSUM |
					DEV_TX_OFFLOAD_TCP_CKSUM |
					DEV_TX_OFFLOAD_UDP_CKSUM;
}

static int
eth_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *eth_stats)
{
	struct wil6210_priv *wil;
	struct wil_sta_info *sta;

	if (dev == NULL || eth_stats == NULL)
		return -EINVAL;

	wil = dev->data->dev_private;

	for (int cid = 0; cid < WIL6210_MAX_CID; cid++) {
		sta = &wil->sta[cid];
		if (sta == NULL || sta->status != wil_sta_connected || sta->mid != 0)
			continue;

		eth_stats->ipackets += sta->stats.rx_packets;
		eth_stats->opackets += sta->stats.tx_packets;
		eth_stats->ibytes += sta->stats.rx_bytes;
		eth_stats->obytes += sta->stats.tx_bytes;
		eth_stats->ierrors +=
			sta->stats.rx_dropped + sta->stats.rx_non_data_frame +
			sta->stats.rx_short_frame + sta->stats.rx_large_frame +
			sta->stats.rx_mic_error + sta->stats.rx_key_error +
			sta->stats.rx_amsdu_error;
		eth_stats->oerrors += sta->stats.tx_errors;
	}

	return 0;
}

static void
eth_stats_reset(struct rte_eth_dev *dev)
{
	if (dev == NULL)
		return;
}

static int
eth_xstats_get_names(struct rte_eth_dev *dev,
		     struct rte_eth_xstat_name *xstats_names,
		     __rte_unused unsigned limit)
{
	unsigned count = 0;
	unsigned pmc = 0;
#ifdef ENABLE_PMC_STATS
	pmc = RTE_PMC_NUM_COUNTERS * PMC_NUM_COUNTER_INSTANCES;
#endif

	/* API requires returning number of stats when xstats_names is NULL */
	if (xstats_names == NULL)
		return WIL6210_NB_U64_PORT_STATS + WIL6210_NB_U64_ABS_PORT_STATS + pmc
		       + WIL6210_MAX_CID *
		       (WIL6210_NB_UL_STATS + WIL6210_NB_ATOMIC_STATS + WIL_MCS_MAX);

	/* stats collected on per port basis */
	for (int t = 0; t < WIL6210_NB_U64_PORT_STATS; t++) {
		snprintf(xstats_names[count].name,
			 sizeof(xstats_names[count].name), "%s",
			 rte_port_stats_u64_strings[t].name);
		count++;
	}
	for (int t = 0; t < WIL6210_NB_U64_ABS_PORT_STATS; t++) {
		// non-monotonic stats
		// Note: _nonmon tells the caller that it shouldn't apply a diff on the
		// counter it has obtained from wil6210
		snprintf(xstats_names[count].name,
			 sizeof(xstats_names[count].name), "%s_nonmon",
			 rte_port_stats_u64_abs_strings[t].name);
		count++;
	}

#ifdef ENABLE_PMC_STATS
	/* List all instances of a given PMC_COUNTERS, for each counters */
	for (int t = 0; t < RTE_PMC_NUM_COUNTERS; t++) {
		snprintf(xstats_names[count].name,
			 sizeof(xstats_names[count].name), "rx_pmc_in1_%u", t);
		count++;
		snprintf(xstats_names[count].name,
			 sizeof(xstats_names[count].name), "rx_pmc_in2_%u", t);
		count++;
		snprintf(xstats_names[count].name,
			 sizeof(xstats_names[count].name), "rx_pmc_in3_%u", t);
		count++;
		snprintf(xstats_names[count].name,
			 sizeof(xstats_names[count].name), "rx_pmc_out_%u", t);
		count++;
		snprintf(xstats_names[count].name,
			 sizeof(xstats_names[count].name), "tx_pmc_in1_%u", t);
		count++;
		snprintf(xstats_names[count].name,
			 sizeof(xstats_names[count].name), "tx_pmc_in2_%u", t);
		count++;
		snprintf(xstats_names[count].name,
			 sizeof(xstats_names[count].name), "tx_pmc_in3_%u", t);
		count++;
		snprintf(xstats_names[count].name,
			 sizeof(xstats_names[count].name), "tx_pmc_out%u", t);
		count++;
	}
#endif

	/* stats collected on per port x per STA basis */
	for (int cid = 0; cid < WIL6210_MAX_CID; cid++) {
		for (int t = 0; t < WIL6210_NB_UL_STATS; t++) {
			snprintf(xstats_names[count].name,
				 sizeof(xstats_names[count].name), "cid%u_%s",
				 cid, rte_stats_ul_strings[t].name);
			count++;
		}
		for (int t = 0; t < WIL6210_NB_ATOMIC_STATS; t++) {
			snprintf(xstats_names[count].name,
				 sizeof(xstats_names[count].name), "cid%u_%s",
				 cid, rte_stats_atomic_strings[t].name);
			count++;
		}
		for (int t = 0; t < WIL_MCS_MAX; t++) {
			snprintf(xstats_names[count].name,
				 sizeof(xstats_names[count].name), "cid%u_rx_mcs%u",
				 cid, t);
			count++;
		}
	}

	return count;
}

static int
eth_xstats_get(struct rte_eth_dev *dev, struct rte_eth_xstat *xstats,
	       unsigned n)
{
	unsigned count = 0;
	struct wil6210_priv *wil;
	unsigned pmc = 0;
#ifdef ENABLE_PMC_STATS
	pmc = RTE_PMC_NUM_COUNTERS * PMC_NUM_COUNTER_INSTANCES;
#endif

	/* API requires returning number of stats when xstats is NULL */
	if (xstats == NULL)
		return WIL6210_NB_U64_PORT_STATS + WIL6210_NB_U64_ABS_PORT_STATS + pmc
		       + WIL6210_MAX_CID *
		       (WIL6210_NB_UL_STATS + WIL6210_NB_ATOMIC_STATS + WIL_MCS_MAX);

	wil = dev->data->dev_private;

	/* stats collected on per port basis */
	for (int t = 0; t < WIL6210_NB_U64_PORT_STATS; t++) {
		xstats[count].value = *(
			u64 *)(((char *)wil) +
						 rte_port_stats_u64_strings[t].offset);
		xstats[count].id = count;
		count++;
	}
	for (int t = 0; t < WIL6210_NB_U64_ABS_PORT_STATS; t++) {
		xstats[count].value = *(
			u64 *)(((char *)wil) +
						 rte_port_stats_u64_abs_strings[t].offset);
		// Reset the counter
		*(u64 *)(((char *)wil) +
						rte_port_stats_u64_abs_strings[t].offset) = 0;
		xstats[count].id = count;
		count++;
	}

#ifdef ENABLE_PMC_STATS
	/* List all instances of a given PMC_COUNTERS, for each counters */
	for (int u = 0; u < RTE_PMC_NUM_COUNTERS; u++) {
		for (int t = 0; t < PMC_NUM_COUNTER_INSTANCES; t++) {
			xstats[count].value = wil->pmc[t][u];
			if (wil->xstats_flags & XSTATS_FLAGS_PMC_COUNTERS_CLEAR)
				wil->pmc[t][u] = 0;
			count++;
		}
	}
#endif

	/* stats collected on per port x per STA basis */
	for (int cid = 0; cid < WIL6210_MAX_CID; cid++) {
		struct wil_sta_info *sta = &wil->sta[cid];
		for (int t = 0; t < WIL6210_NB_UL_STATS; t++) {
			if (sta->status != wil_sta_connected || sta->mid != 0)
				xstats[count].value = 0;
			else
				xstats[count].value = *(
					unsigned long *)(((char *)&sta->stats) +
							 rte_stats_ul_strings[t]
								 .offset);
			xstats[count].id = count;
			count++;
		}
		for (int t = 0; t < WIL6210_NB_ATOMIC_STATS; t++) {
			if (sta->status != wil_sta_connected || sta->mid != 0)
				xstats[count].value = 0;
			else {
				u64 *acc = (u64 *)(((char *)&sta->stats) +
						   rte_stats_atomic_strings[t]
							   .acc_offset);
				*acc += atomic_read((
						atomic_t
							*)(((char *)&sta->stats) +
							   rte_stats_atomic_strings[t]
								   .offset));
				xstats[count].value = *acc;
			}
			xstats[count].id = count;
			count++;
		}
		for (int t = 0; t < WIL_MCS_MAX; t++) {
			if (sta->status != wil_sta_connected || sta->mid != 0)
				xstats[count].value = 0;
			else {
				xstats[count].value = sta->stats.rx_per_mcs[t];
			}
			xstats[count].id = count;
			count++;
		}
		if (wil->xstats_flags & XSTATS_FLAGS_COUNTERS_CLEAR) {
			sta->stats.wil_tx_ring_calls = 0;
			sta->stats.wil_tx_ring_full = 0;
		}
	}

	if (wil->xstats_flags & XSTATS_FLAGS_COUNTERS_CLEAR) {
#ifdef ENABLE_PMC_STATS
		wil->wil_tx_handler_time_nano = 0;
#endif
#ifdef DEBUG_LATENCY
		wil->wil_rx_handler_time_nano = 0;
#endif
		wil->count_rx_burst_calls = 0;
		wil->count_rx_burst_full = 0;
		wil->no_credits = 0;
	}
	return count;
}

static void
eth_queue_release(void *q)
{
}

static int
eth_link_update(struct rte_eth_dev *dev __rte_unused,
		int wait_to_complete __rte_unused)
{
	return 0;
}

static const struct eth_dev_ops ops = {
	.dev_start = eth_dev_start,
	.dev_stop = eth_dev_stop,
	.dev_close = eth_dev_close,
	.dev_configure = eth_dev_configure,
	.dev_infos_get = eth_dev_info,
	.fw_version_get = eth_fw_version_get,
	.rx_queue_setup = eth_rx_queue_setup,
	.tx_queue_setup = eth_tx_queue_setup,
	.rx_queue_release = eth_queue_release,
	.tx_queue_release = eth_queue_release,
	.link_update = eth_link_update,
	.stats_get = eth_stats_get,
	.stats_reset = eth_stats_reset,
	.xstats_get_names = eth_xstats_get_names,
	.xstats_get = eth_xstats_get,
	.xstats_config = wil6210_xstats_config,
};

int
wil6210_eth_dev_init(struct rte_eth_dev *eth_dev)
{
	const unsigned nb_rx_queues = 1;
	const unsigned nb_tx_queues = 1;
	struct rte_eth_dev_data *data = NULL;
	struct pmd_internals *internals = NULL;
	struct rte_pci_device *dev = NULL;
	struct wil6210_priv *wil;

	dev = RTE_ETH_DEV_TO_PCI(eth_dev);
	if (dev->device.numa_node == SOCKET_ID_ANY)
		dev->device.numa_node = rte_socket_id();

	RTE_LOG(INFO, PMD, "Setting up ethdev on numa socket %u\n",
		dev->device.numa_node);

	internals = rte_zmalloc_socket("eth_priv", sizeof(*internals), 0,
		dev->device.numa_node);
	if (!internals)
		return -ENOMEM;

	data = eth_dev->data;
	data->nb_rx_queues = (uint16_t)nb_rx_queues;
	data->nb_tx_queues = (uint16_t)nb_tx_queues;
	data->dev_link = pmd_link;
	data->mac_addrs = &internals->eth_addr;

	wil = eth_dev->data->dev_private;
	internals->wil = wil;
	memcpy(&internals->eth_addr.addr_bytes, wil->main_ndev->dev_addr,
	    ETH_ALEN);

	eth_dev->dev_ops = &ops;

	/* set receive and transmit functions when device starts */
	eth_dev->rx_pkt_burst = wil6210_burst_rx_noop;
	eth_dev->tx_pkt_burst = wil6210_burst_tx_noop;
	eth_dev->tx_get_feedback = wil6210_tx_feedback_noop;
	eth_dev->tx_send_pending_stats = wil6210_send_queue_stats_noop;
	eth_dev->tx_get_pipe_size = wil6210_tx_get_pipe_size;

	eth_set_priv(eth_dev, internals);

	return 0;
}

int
wil6210_eth_dev_stop(struct rte_eth_dev *dev)
{
	struct pmd_internals *internals;
	RTE_LOG(INFO, PMD, "Closing ethdev on numa socket %u\n",
			rte_socket_id());

	internals = eth_get_priv(dev);
	rte_free(internals);
	return 0;
}
