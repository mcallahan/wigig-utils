// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2012-2017 Qualcomm Atheros, Inc.
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019-2020, Facebook, Inc. All rights reserved.
 */

#include "wil6210_ethdev.h"
#include "wmi.h"
#include "txrx.h"
#include "slave_i.h"

#include <rte_net.h>

#if defined(CONFIG_WIL6210_NSS_SUPPORT)
bool rx_align_2 = true;
#else
bool rx_align_2;
#endif
module_param(rx_align_2, bool, 0444);
MODULE_PARM_DESC(rx_align_2, " align Rx buffers on 4*n+2, default - no");

bool rx_large_buf;
module_param(rx_large_buf, bool, 0444);
MODULE_PARM_DESC(rx_large_buf, " allocate 8KB RX buffers, default - no");

#define WIL6210_MAX_HEADROOM_SIZE	(256)

uint headroom_size; /* = 0; */
MODULE_PARM_DESC(headroom_size,
		 " headroom size for rx skb allocation, default - 0");

void wil_tx_data_init(struct wil_ring_tx_data *txdata)
{
	spin_lock_bh(&txdata->lock);
	txdata->dot1x_open = 0;
	txdata->enabled = 0;
	txdata->idle = 0;
	txdata->last_idle = 0;
	txdata->begin = 0;
	txdata->agg_wsize = 0;
	txdata->agg_timeout = 0;
	txdata->agg_amsdu = 0;
	txdata->addba_in_progress = false;
	txdata->mid = U8_MAX;
	txdata->cid = U8_MAX;
	spin_unlock_bh(&txdata->lock);
}

void wil_netif_rx_any(struct rte_mbuf *skb, struct net_device *ndev)
{
#ifndef WIL6210_PMD
	gro_result_t rc = GRO_NORMAL;
	struct wil6210_vif *vif = ndev_to_vif(ndev);
	struct wil6210_priv *wil = ndev_to_wil(ndev);
	struct wireless_dev *wdev = vif_to_wdev(vif);
	unsigned int len = skb->len;
	int cid;
	int security;
	u8 *sa, *da = wil_skb_get_da(skb);
	/* here looking for DA, not A1, thus Rxdesc's 'mcast' indication
	 * is not suitable, need to look at data
	 */
	int mcast = is_multicast_ether_addr(da);
	struct wil_net_stats *stats;
	struct sk_buff *xmit_skb = NULL;
	static const char * const gro_res_str[] = {
		[GRO_MERGED]		= "GRO_MERGED",
		[GRO_MERGED_FREE]	= "GRO_MERGED_FREE",
		[GRO_HELD]		= "GRO_HELD",
		[GRO_NORMAL]		= "GRO_NORMAL",
		[GRO_DROP]		= "GRO_DROP",
	};

	wil->txrx_ops.get_netif_rx_params(skb, &cid, &security);

	stats = &wil->sta[cid].stats;

	skb_orphan(skb);

	if (security && (wil->txrx_ops.rx_crypto_check(wil, skb) != 0)) {
		rc = GRO_DROP;
		dev_kfree_skb(skb);
		stats->rx_replay++;
		goto stats;
	}

	if (vif->umac_vap)
		wil->umac_ops.rx_notify(wil->umac_handle, vif->umac_vap, skb);

	/* check errors reported by HW and update statistics */
	if (unlikely(wil->txrx_ops.rx_error_check(wil, skb, stats))) {
		dev_kfree_skb(skb);
		return;
	}

	if (wdev->iftype == NL80211_IFTYPE_STATION) {
		sa = wil_skb_get_sa(skb);
		if (mcast && ether_addr_equal(sa, ndev->dev_addr)) {
			/* mcast packet looped back to us */
			rc = GRO_DROP;
			dev_kfree_skb(skb);
			goto stats;
		}
	} else if (wdev->iftype == NL80211_IFTYPE_AP && !vif->ap_isolate) {
		if (mcast) {
			/* send multicast frames both to higher layers in
			 * local net stack and back to the wireless medium
			 */
			xmit_skb = skb_copy(skb, GFP_ATOMIC);
		} else {
			int xmit_cid = wil_find_cid(wil, vif->mid, da);

			if (xmit_cid >= 0) {
				/* The destination station is associated to
				 * this AP (in this VLAN), so send the frame
				 * directly to it and do not pass it to local
				 * net stack.
				 */
				xmit_skb = skb;
				skb = NULL;
			}
		}
	}
	if (xmit_skb) {
		/* Send to wireless media and increase priority by 256 to
		 * keep the received priority instead of reclassifying
		 * the frame (see cfg80211_classify8021d).
		 */
		xmit_skb->dev = ndev;
		xmit_skb->priority += 256;
		xmit_skb->protocol = htons(ETH_P_802_3);
		skb_reset_network_header(xmit_skb);
		skb_reset_mac_header(xmit_skb);
		wil_dbg_txrx(wil, "Rx -> Tx %d bytes\n", len);
		dev_queue_xmit(xmit_skb);
	}

	if (skb) { /* deliver to local stack */
		bool deliver_skb = true;
#if defined(CONFIG_WIL6210_NSS_SUPPORT)
		if (rx_align_2) {
			int rc1 = nss_virt_if_tx_buf(vif->nss_handle, skb);

			if (rc1) {
				wil_err_ratelimited(wil, "NSS Rx error: %d\n",
						    rc1);
			} else {
				rc = GRO_NORMAL;
				deliver_skb = false;
			}
		}
#endif /* #if defined(CONFIG_WIL6210_NSS_SUPPORT) */
		if (deliver_skb) {
			if (slave_mode) {
				rc = wil_slave_rx_data(vif, cid, skb);
			} else {
				skb->protocol = eth_type_trans(skb, ndev);
				skb->dev = ndev;
				rc = napi_gro_receive(&wil->napi_rx, skb);
			}
		}
		wil_dbg_txrx(wil, "Rx complete %d bytes => %s\n",
			     len, gro_res_str[rc]);
	}
stats:
	/* statistics. rc set to GRO_NORMAL for AP bridging */
	if (unlikely(rc == GRO_DROP)) {
		ndev->stats.rx_dropped++;
		stats->rx_dropped++;
		wil_dbg_txrx(wil, "Rx drop %d bytes\n", len);
	} else {
		ndev->stats.rx_packets++;
		stats->rx_packets++;
		ndev->stats.rx_bytes += len;
		stats->rx_bytes += len;
		if (mcast)
			ndev->stats.multicast++;
	}
#endif
}

/**
 * Sets the descriptor @d up for csum. The corresponding
 * @skb is used to obtain the protocol and headers length.
 * Returns the protocol: 0 - not TCP, 1 - TCPv4, 2 - TCPv6.
 * Note, if d==NULL, the function only returns the protocol result.
 *
 * It is very similar to previous wil_tx_desc_offload_setup_tso. This
 * is "if unrolling" to optimize the critical path.
 */

#define WIL_TX_CKSUM_OFFLOAD_MASK (             \
		PKT_TX_IP_CKSUM |                \
		PKT_TX_TCP_CKSUM |               \
		PKT_TX_UDP_CKSUM)

static int wil_tx_desc_offload_setup(struct vring_tx_desc *d,
				     struct rte_mbuf *mbuf)
{
	/*
	 * Expect checksum offload to be required in common forwarding
	 * case.
	 */
	if (unlikely((mbuf->ol_flags & WIL_TX_CKSUM_OFFLOAD_MASK) == 0))
		return 0;

#ifdef RTE_LIBRTE_ETHDEV_DEBUG
	{
		int ret = rte_validate_tx_offload(mbuf);
		if (ret != 0)
			return ret;
	}
#endif
	d->dma.b11 = ETH_HLEN; /* MAC header length */

	if (mbuf->ol_flags & PKT_TX_IPV4)
		d->dma.b11 |= BIT(DMA_CFG_DESC_TX_OFFLOAD_CFG_L3T_IPV4_POS);

	if (mbuf->ol_flags & PKT_TX_TCP_CKSUM) {
		d->dma.d0 |= (2 << DMA_CFG_DESC_TX_0_L4_TYPE_POS);
		/* L4 header len: TCP header length */
		d->dma.d0 |=
		(mbuf->l4_len & DMA_CFG_DESC_TX_0_L4_LENGTH_MSK);
	} else if (mbuf->ol_flags & PKT_TX_UDP_CKSUM) {
		/* L4 header len: UDP header length */
		d->dma.d0 |=
		(sizeof(struct udp_hdr) & DMA_CFG_DESC_TX_0_L4_LENGTH_MSK);
	}

	d->dma.ip_length = mbuf->l3_len;
	/* Enable TCP/UDP checksum */
	d->dma.d0 |= BIT(DMA_CFG_DESC_TX_0_TCP_UDP_CHECKSUM_EN_POS);
	/* Calculate pseudo-header */
	d->dma.d0 |= BIT(DMA_CFG_DESC_TX_0_PSEUDO_HEADER_CALC_EN_POS);
	return 0;
}

static inline void wil_tx_last_desc(struct vring_tx_desc *d)
{
	d->dma.d0 |= BIT(DMA_CFG_DESC_TX_0_CMD_EOP_POS) |
	      BIT(DMA_CFG_DESC_TX_0_CMD_MARK_WB_POS) |
	      BIT(DMA_CFG_DESC_TX_0_CMD_DMA_IT_POS);
}

static inline
void wil_tx_desc_set_nr_frags(struct vring_tx_desc *d, int nr_frags)
{
	d->mac.d[2] |= (nr_frags << MAC_CFG_DESC_TX_2_NUM_OF_DESCRIPTORS_POS);
}

static int __wil_tx_ring(struct wil6210_priv *wil, struct wil6210_vif *vif,
			 struct wil_ring *ring, struct rte_mbuf *skb)
{
	struct device *dev = wil_to_dev(wil);
	struct rte_mbuf *frag;
	struct vring_tx_desc dd, *d = &dd;
	volatile struct vring_tx_desc *_d;
	u32 swhead = ring->swhead;
	u32 pkt_len;
	int avail = wil_ring_avail_tx(ring);
	int nr_frags;
	uint f = 0;
	int ring_index = ring - wil->ring_tx;
	struct wil_ring_tx_data  *txdata = &wil->ring_tx_data[ring_index];
	u8 cid = txdata->cid;
	struct wil_net_stats *stats = (cid < WIL6210_MAX_CID) ?
		&wil->sta[cid].stats : NULL;
	uint i = swhead;
	dma_addr_t pa;
	int used;
	bool mcast = (ring_index == vif->bcast_ring);
	uint len = rte_pktmbuf_data_len(skb);

	nr_frags = skb->nb_segs - 1;
	wil_dbg_txrx(wil, "tx_ring: %d bytes to ring %d, nr_frags %d\n",
		     len, ring_index, nr_frags);

	if (stats)
		stats->wil_tx_ring_calls++;

	if (unlikely(!txdata->enabled))
		return -EINVAL;

	if (unlikely(avail < 1 + nr_frags)) {
		if (stats)
			stats->wil_tx_ring_full++;
		wil_dbg_txrx(wil,
				    "Tx ring[%2d] full. No space for %d fragments\n",
				    ring_index, 1 + nr_frags);
		return -ENOMEM;
	}
	_d = &ring->va[i].tx.legacy;

	pa = rte_mbuf_data_iova(skb);

	wil_dbg_txrx(wil, "Tx[%2d] skb %d bytes 0x%p -> %llx\n", ring_index,
		     len, rte_pktmbuf_mtod(skb, void*), (unsigned long long)pa);
	wil_hex_dump_txrx("Tx ", DUMP_PREFIX_OFFSET, 16, 1,
			  rte_pktmbuf_mtod(skb, void*), len, false);

	ring->ctx[i].mapped_as = wil_mapped_as_single;
	/* 1-st segment */
	wil->txrx_ops.tx_desc_map((union wil_tx_desc *)d, pa, len,
				   ring_index);
	if (unlikely(mcast)) {
		d->mac.d[0] |= BIT(MAC_CFG_DESC_TX_0_MCS_EN_POS); /* MCS 0 */
		if (unlikely(len > WIL_BCAST_MCS0_LIMIT)) /* set MCS 1 */
			d->mac.d[0] |= (1 << MAC_CFG_DESC_TX_0_MCS_INDEX_POS);
	}
	/* Process TCP/UDP checksum offloading */
	if (unlikely(wil_tx_desc_offload_setup(d, skb))) {
		wil_err(wil, "Tx[%2d] Failed to set cksum, drop packet\n",
			ring_index);
		goto dma_error;
	}

	ring->ctx[i].nr_frags = nr_frags;
	wil_tx_desc_set_nr_frags(d, nr_frags + 1);

	/* middle segments */
	frag = skb->next;
	for (; f < nr_frags; f++) {
		int len = rte_pktmbuf_data_len(frag);

		*_d = *d;
		wil_dbg_txrx(wil, "Tx[%2d] desc[%4d]\n", ring_index, i);
		wil_hex_dump_txrx("TxD ", DUMP_PREFIX_NONE, 32, 4,
				  (const void *)d, sizeof(*d), false);
		i = (swhead + f + 1) % ring->size;
		_d = &ring->va[i].tx.legacy;
		pa = rte_mbuf_data_iova(frag);
		ring->ctx[i].mapped_as = wil_mapped_as_page;
		wil->txrx_ops.tx_desc_map((union wil_tx_desc *)d,
					   pa, len, ring_index);
		/* no need to check return code -
		 * if it succeeded for 1-st descriptor,
		 * it will succeed here too
		 */
		wil_tx_desc_offload_setup(d, skb);
		frag = frag->next;
	}
	/* for the last seg only */
	d->dma.d0 |= BIT(DMA_CFG_DESC_TX_0_CMD_EOP_POS);
	d->dma.d0 |= BIT(DMA_CFG_DESC_TX_0_CMD_MARK_WB_POS);
	d->dma.d0 |= BIT(DMA_CFG_DESC_TX_0_CMD_DMA_IT_POS);
	*_d = *d;
	wil_dbg_txrx(wil, "Tx[%2d] desc[%4d]\n", ring_index, i);
	wil_hex_dump_txrx("TxD ", DUMP_PREFIX_NONE, 32, 4,
			  (const void *)d, sizeof(*d), false);

	/*
	 * Cache packet len into local variable to avoid dereferencing
	 * skb from this point on.
	 */
	pkt_len = skb->pkt_len;
	ring->ctx[i].skb = skb;

	/* performance monitoring */
	used = wil_ring_used_tx(ring);
	if (wil_val_in_range(wil->ring_idle_trsh,
			     used, used + nr_frags + 1)) {
		txdata->idle += get_cycles() - txdata->last_idle;
		wil_dbg_txrx(wil,  "Ring[%2d] not idle %d -> %d\n",
			     ring_index, used, used + nr_frags + 1);
	}

	/* Make sure to advance the head only after descriptor update is done.
	 * This will prevent a race condition where the completion thread
	 * will see the DU bit set from previous run and will handle the
	 * skb before it was completed.
	 */
	wmb();

	/* advance swhead */
	wil_ring_advance_head(ring, nr_frags + 1);
	wil_dbg_txrx(wil, "Tx[%2d] swhead %d -> %d\n", ring_index, swhead,
		     ring->swhead);
	//trace_wil6210_tx(ring_index, swhead, pkt_len, nr_frags);

	/* make sure all writes to descriptors (shared memory) are done before
	 * committing them to HW
	 */
	wmb();

	if (stats) {
		atomic_inc(&stats->tx_pend_packets);
		atomic_add(pkt_len, &stats->tx_pend_bytes);
	}

	wil_w(wil, ring->hwtail, ring->swhead);

	return 0;
 dma_error:
	/* unmap what we have mapped */
	nr_frags = f + 1; /* frags mapped + one for skb head */
	for (f = 0; f < nr_frags; f++) {
		struct wil_ctx *ctx;

		i = (swhead + f) % ring->size;
		ctx = &ring->ctx[i];
		_d = &ring->va[i].tx.legacy;
		*d = *_d;
		_d->dma.status = TX_DMA_STATUS_DU;
		wil->txrx_ops.tx_desc_unmap(dev,
					    (union wil_tx_desc *)d,
					    ctx);

		memset(ctx, 0, sizeof(*ctx));
	}

	return -EINVAL;
}

#ifdef WIL6210_PMD
#define skb_is_gso(skb) (0)
int wil_tx_ring(struct wil6210_priv *wil, struct wil6210_vif *vif,
		       struct wil_ring *ring, struct rte_mbuf *skb);
#endif /* WIL6210_PMD */
/*static*/ int wil_tx_ring(struct wil6210_priv *wil, struct wil6210_vif *vif,
		       struct wil_ring *ring, struct rte_mbuf *skb)
{
	int ring_index = ring - wil->ring_tx;
	struct wil_ring_tx_data *txdata = &wil->ring_tx_data[ring_index];
	int rc;

	spin_lock(&txdata->lock);

	if (test_bit(wil_status_suspending, wil->status) ||
	    test_bit(wil_status_suspended, wil->status) ||
	    test_bit(wil_status_resuming, wil->status)) {
		wil_dbg_txrx(wil,
			     "suspend/resume in progress. drop packet\n");
		spin_unlock(&txdata->lock);
		return -EINVAL;
	}

	rc = (skb_is_gso(skb) ? wil->txrx_ops.tx_ring_tso : __wil_tx_ring)
	     (wil, vif, ring, skb);

	spin_unlock(&txdata->lock);

	return rc;
}

/**
 * reverse_memcmp - Compare two areas of memory, in reverse order
 * @cs: One area of memory
 * @ct: Another area of memory
 * @count: The size of the area.
 *
 * Cut'n'paste from original memcmp (see lib/string.c)
 * with minimal modifications
 */
int reverse_memcmp(const void *cs, const void *ct, size_t count)
{
	const unsigned char *su1, *su2;
	int res = 0;

	for (su1 = cs + count - 1, su2 = ct + count - 1; count > 0;
	     --su1, --su2, count--) {
		res = *su1 - *su2;
		if (res)
			break;
	}
	return res;
}

void wil_update_net_queues(struct wil6210_priv *wil,
			   struct wil6210_vif *vif,
			   struct wil_ring *ring,
			   bool check_stop)
{
	spin_lock(&wil->net_queue_lock);
	//__wil_update_net_queues(wil, vif, ring, check_stop);
	spin_unlock(&wil->net_queue_lock);
}
