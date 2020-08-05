// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2012-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019, Facebook, Inc. All rights reserved.
 */

#include "wil6210_ethdev.h"
#include "txrx_edma.h"
#include "txrx.h"

#if 1
#define RTE_PMD_USE_PREFETCH
#endif

#ifdef RTE_PMD_USE_PREFETCH
#define rte_wil_prefetch(p)	rte_prefetch0(p)
#else
#define rte_wil_prefetch(p)	do {} while(0)
#endif

#ifdef RTE_PMD_PACKET_PREFETCH
#define rte_packet_prefetch(p) rte_prefetch1(p)
#else
#define rte_packet_prefetch(p)	do {} while(0)
#endif

/* Max number of entries (packets to complete) to update the hwtail of tx
 * status ring. Should be power of 2
 */
#define WIL_EDMA_TX_SRING_UPDATE_HW_TAIL 128
#define WIL_EDMA_MAX_DATA_OFFSET (2)
/* RX buffer size must be aligned to 4 bytes */
#define WIL_EDMA_RX_BUF_LEN_DEFAULT (2048)
#define MAX_INVALID_BUFF_ID_RETRY (3)

typedef struct skb_frag {
	int size;
} skb_frag_t;

static int
skb_headlen(struct rte_mbuf *skb)
{
	/*
	 * Amount of data in first mbuf of mbuf chain. Should be
	 * identical to pkt_len for non-segmented mbufs.
	 */
	return skb->data_len;
}

static inline void wil_tx_desc_unmap_edma(struct device *dev,
				   union wil_tx_desc *desc,
				   struct wil_ctx *ctx)
{
	/* Nothing */
}

static int wil_find_free_sring(struct wil6210_priv *wil)
{
	int i;

	for (i = 0; i < WIL6210_MAX_STATUS_RINGS; i++) {
		if (!wil->srings[i].va)
			return i;
	}

	return -EINVAL;
}

static void wil_sring_free(struct wil6210_priv *wil,
			   struct wil_status_ring *sring)
{
	size_t sz;

	if (!sring || !sring->va)
		return;

	sz = sring->elem_size * sring->size;

	wil_dbg_misc(wil, "status_ring_free, size(bytes)=%zu, 0x%p:0x%jx\n",
		     sz, sring->va, (uintmax_t)sring->pa);

	wil_dma_free_coherent(wil, &sring->dmah);
	sring->pa = 0;
	sring->va = NULL;
}

static int wil_sring_alloc(struct wil6210_priv *wil,
			   struct wil_status_ring *sring)
{
	size_t sz = sring->elem_size * sring->size;

	wil_dbg_misc(wil, "status_ring_alloc: size=%zu\n", sz);

	if (sz == 0) {
		wil_err(wil, "Cannot allocate a zero size status ring\n");
		return -EINVAL;
	}

	sring->swhead = 0;

	/* Status messages are allocated and initialized to 0. This is necessary
	 * since DR bit should be initialized to 0.
	 */
	sring->va = wil_dma_zalloc_coherent(wil, "sring",
	    sring - &wil->srings[0], sz, &sring->dmah);
	if (!sring->va)
		return -ENOMEM;
	sring->pa = sring->dmah.dma_addr;

	wil_dbg_misc(wil, "status_ring[%d] 0x%p:%#jx\n", sring->size, sring->va,
		     (uintmax_t)sring->pa);

	return 0;
}

static int wil_tx_init_edma(struct wil6210_priv *wil)
{
	int ring_id = wil_find_free_sring(wil);
	struct wil_status_ring *sring;
	int rc;
	u16 status_ring_size;

	if (wil->tx_status_ring_order < WIL_SRING_SIZE_ORDER_MIN ||
	    wil->tx_status_ring_order > WIL_SRING_SIZE_ORDER_MAX)
		wil->tx_status_ring_order = WIL_TX_SRING_SIZE_ORDER_DEFAULT;

	status_ring_size = 1 << wil->tx_status_ring_order;

	wil_dbg_misc(wil, "init TX sring: size=%u, ring_id=%u\n",
		     status_ring_size, ring_id);

	if (ring_id < 0)
		return ring_id;

	/* Allocate Tx status ring. Tx descriptor rings will be
	 * allocated on WMI connect event
	 */
	sring = &wil->srings[ring_id];

	sring->is_rx = false;
	sring->size = status_ring_size;
	sring->elem_size = sizeof(struct wil_ring_tx_status);
	rc = wil_sring_alloc(wil, sring);
	if (rc)
		return rc;

	rc = wil_wmi_tx_sring_cfg(wil, ring_id);
	if (rc)
		goto out_free;

	sring->desc_rdy_pol = 1;
	wil->tx_sring_idx = ring_id;

	return 0;
out_free:
	wil_sring_free(wil, sring);
	return rc;
}

/**
 * Allocate one skb for Rx descriptor RING
 */
static int wil_ring_alloc_skb_edma(struct wil6210_priv *wil,
				   struct wil_ring *ring, u32 i)
{
	unsigned int sz = wil->rx_buf_len;
	dma_addr_t pa;
	u16 buff_id;
	struct list_head *active = &wil->rx_buff_mgmt.active;
	struct list_head *free = &wil->rx_buff_mgmt.free;
	struct wil_rx_buff *rx_buff;
	struct wil_rx_buff *buff_arr = wil->rx_buff_mgmt.buff_arr;
	struct rte_mbuf *skb;
	struct wil_rx_enhanced_desc dd, *d = &dd;
	struct wil_rx_enhanced_desc *_d = (struct wil_rx_enhanced_desc *)
		(uintptr_t)&ring->va[i].rx.enhanced;

	if (unlikely(list_empty(free))) {
		wil->rx_buff_mgmt.free_list_empty_cnt++;
		return -EAGAIN;
	}

	skb = rte_pktmbuf_alloc(ring->mpool);
	if (unlikely(!skb)) {
		wil_err(wil, "Unable to alloc mbuf\n");
		return -ENOMEM;
	}

	/**
	 * Make sure that the network stack calculates checksum for packets
	 * which failed the HW checksum calculation
	 */
	//skb->ip_summed = CHECKSUM_NONE;

	pa = rte_mbuf_data_iova_default(skb);

	/* Get the buffer ID - the index of the rx buffer in the buff_arr */
	rx_buff = list_first_entry(free, struct wil_rx_buff, list);
	buff_id = rx_buff->id;

	/* Move a buffer from the free list to the active list */
	list_move(&rx_buff->list, active);

	buff_arr[buff_id].skb = skb;

	wil_desc_set_addr_edma(&d->dma.addr, &d->dma.addr_high_high, pa);
	d->dma.length = cpu_to_le16(sz);
	d->mac.buff_id = cpu_to_le16(buff_id);
	*_d = *d;

	return 0;
}

static inline
void wil_get_next_rx_status_msg(struct wil_status_ring *sring,
			        struct wil_rx_status_extended *msg)
{
	volatile struct wil_rx_status_extended *ptr;

	ptr = (volatile struct wil_rx_status_extended *)
	    (sring->va + (sring->elem_size * sring->swhead));
	*msg = *ptr;
}

static inline void wil_sring_advance_swhead(struct wil_status_ring *sring)
{
	sring->swhead = (sring->swhead + 1) % sring->size;
	if (sring->swhead == 0)
		sring->desc_rdy_pol = 1 - sring->desc_rdy_pol;
}

static int wil_rx_refill_edma(struct wil6210_priv *wil)
{
	struct wil_ring *ring = &wil->ring_rx;
	u32 next_head, swhead;
	int rc = 0;
	ring->swtail = *ring->edma_rx_swtail.va;

	swhead = ring->swhead;
	for (; next_head = wil_ring_next_head(ring),
	     (next_head != ring->swtail);
	     ring->swhead = next_head) {
		rc = wil_ring_alloc_skb_edma(wil, ring, ring->swhead);
		if (unlikely(rc)) {
			if (rc == -EAGAIN)
				wil_dbg_txrx(wil, "No free buffer ID found\n");
			else
				wil_err_ratelimited(wil,
						    "Error %d in refill desc[%d]\n",
						    rc, ring->swhead);
			break;
		}
	}

	if (swhead != ring->swhead) {
		/* make sure all writes to descriptors (shared memory) are done before
		 * committing them to HW
		 */
		wmb();

		wil_w(wil, ring->hwtail, ring->swhead);
	}

	return rc;
}

static void wil_move_all_rx_buff_to_free_list(struct wil6210_priv *wil,
					      struct wil_ring *ring)
{
	struct list_head *active = &wil->rx_buff_mgmt.active;

	while (!list_empty(active)) {
		struct wil_rx_buff *rx_buff =
			list_first_entry(active, struct wil_rx_buff, list);
		struct rte_mbuf *skb = rx_buff->skb;

		if (unlikely(!skb)) {
			wil_err(wil, "No Rx skb at buff_id %d\n", rx_buff->id);
		} else {
			rx_buff->skb = NULL;
			rte_mbuf_raw_free(skb);
		}

		/* Move the buffer from the active to the free list */
		list_move(&rx_buff->list, &wil->rx_buff_mgmt.free);
	}
}

static void wil_free_rx_buff_arr(struct wil6210_priv *wil)
{
	struct wil_ring *ring = &wil->ring_rx;

	if (!wil->rx_buff_mgmt.buff_arr)
		return;

	/* Move all the buffers to the free list in case active list is
	 * not empty in order to release all SKBs before deleting the array
	 */
	wil_move_all_rx_buff_to_free_list(wil, ring);

	kfree(wil->rx_buff_mgmt.buff_arr);
	wil->rx_buff_mgmt.buff_arr = NULL;
}

static int wil_init_rx_buff_arr(struct wil6210_priv *wil,
				size_t size)
{
	struct wil_rx_buff *buff_arr;
	struct list_head *active = &wil->rx_buff_mgmt.active;
	struct list_head *free = &wil->rx_buff_mgmt.free;
	int i;

	wil->rx_buff_mgmt.buff_arr = kcalloc(size + 1,
					     sizeof(struct wil_rx_buff),
					     GFP_KERNEL);
	if (!wil->rx_buff_mgmt.buff_arr)
		return -ENOMEM;

	/* Set list heads */
	INIT_LIST_HEAD(active);
	INIT_LIST_HEAD(free);

	/* Linkify the list.
	 * buffer id 0 should not be used (marks invalid id).
	 */
	buff_arr = wil->rx_buff_mgmt.buff_arr;
	for (i = 1; i <= size; i++) {
		list_add(&buff_arr[i].list, free);
		buff_arr[i].id = i;
	}

	wil->rx_buff_mgmt.size = size + 1;

	return 0;
}

static int wil_init_rx_sring(struct wil6210_priv *wil,
			     u16 status_ring_size,
			     size_t elem_size,
			     u16 ring_id)
{
	struct wil_status_ring *sring = &wil->srings[ring_id];
	int rc;

	wil_dbg_misc(wil, "init RX sring: size=%u, ring_id=%u\n",
		     status_ring_size, ring_id);

	memset(&sring->rx_data, 0, sizeof(sring->rx_data));

	sring->is_rx = true;
	sring->size = status_ring_size;
	sring->elem_size = elem_size;
	rc = wil_sring_alloc(wil, sring);
	if (rc)
		return rc;

	rc = wil_wmi_rx_sring_add(wil, ring_id);
	if (rc)
		goto out_free;

	sring->desc_rdy_pol = 1;

	return 0;
out_free:
	wil_sring_free(wil, sring);
	return rc;
}

static int wil_ring_alloc_desc_ring(struct wil6210_priv *wil,
				    struct wil_ring *ring)
{
	size_t sz = ring->size * sizeof(ring->va[0]);
	int ring_index;

	wil_dbg_misc(wil, "alloc_desc_ring:\n");

	BUILD_BUG_ON(sizeof(ring->va[0]) != 32);

	ring->swhead = 0;
	ring->swtail = 0;
	ring->ctx = kcalloc(ring->size, sizeof(ring->ctx[0]), GFP_KERNEL);
	if (!ring->ctx)
		goto err;

	if (!ring->is_rx)
		ring_index = ring - wil->ring_tx;
	else
		ring_index = 0;

	ring->va = wil_dma_zalloc_coherent(wil, "desc-ring", ring_index, sz,
	    &ring->dmah);
	if (!ring->va)
		goto err_free_ctx;
	ring->pa = ring->dmah.dma_addr;

	if (ring->is_rx) {
		sz = sizeof(*ring->edma_rx_swtail.va);
		ring->edma_rx_swtail.va =
			wil_dma_zalloc_coherent(wil, "rx-swtail", ring_index,
			    sz, &ring->edma_rx_swtail.dmah);
		if (!ring->edma_rx_swtail.va)
			goto err_free_va;
		ring->edma_rx_swtail.pa = ring->edma_rx_swtail.dmah.dma_addr;
	}

	wil_dbg_misc(wil, "%s ring[%d] 0x%p:%#jx 0x%p\n",
		     ring->is_rx ? "RX" : "TX",
		     ring->size, ring->va, (uintmax_t)ring->pa, ring->ctx);

	return 0;
err_free_va:
	wil_dma_free_coherent(wil, &ring->dmah);
	ring->va = NULL;
err_free_ctx:
	kfree(ring->ctx);
	ring->ctx = NULL;
err:
	return -ENOMEM;
}

static void wil_ring_free_edma(struct wil6210_priv *wil, struct wil_ring *ring)
{
	struct device *dev = wil_to_dev(wil);
	int ring_index = 0;

	if (!ring->va)
		return;

	lockdep_assert_held(&wil->mutex);
	if (ring->is_rx) {
		wil_dbg_misc(wil, "free Rx ring [%d] 0x%p:%#jx 0x%p\n",
			     ring->size, ring->va,
			     (uintmax_t)ring->pa, ring->ctx);

		wil_move_all_rx_buff_to_free_list(wil, ring);
		wil_dma_free_coherent(wil, &ring->edma_rx_swtail.dmah);
		goto out;
	}

	/* TX ring */
	ring_index = ring - wil->ring_tx;

	wil_dbg_misc(wil, "free Tx ring %d [%d] 0x%p:%#jx 0x%p\n",
		     ring_index, ring->size, ring->va,
		     (uintmax_t)ring->pa, ring->ctx);

	while (!wil_ring_is_empty(ring)) {
		struct wil_ctx *ctx;

		struct wil_tx_enhanced_desc dd, *d = &dd;
		struct wil_tx_enhanced_desc *_d =
			(struct wil_tx_enhanced_desc *)
			(uintptr_t)&ring->va[ring->swtail].tx.enhanced;

		ctx = &ring->ctx[ring->swtail];
		if (!ctx) {
			wil_dbg_txrx(wil,
				     "ctx(%d) was already completed\n",
				     ring->swtail);
			ring->swtail = wil_ring_next_tail(ring);
			continue;
		}
		*d = *_d;
		wil_tx_desc_unmap_edma(dev, (union wil_tx_desc *)d, ctx);
		if (ctx->skb)
			rte_pktmbuf_free(ctx->skb);
		ring->swtail = wil_ring_next_tail(ring);
	}

out:
	wil_dma_free_coherent(wil, &ring->dmah);
	kfree(ring->ctx);
	ring->pa = 0;
	ring->va = NULL;
	ring->ctx = NULL;
}

static int wil_init_rx_desc_ring(struct wil6210_priv *wil, u16 desc_ring_size,
				 int status_ring_id)
{
	struct wil_ring *ring = &wil->ring_rx;
	int rc;

	wil_dbg_misc(wil, "init RX desc ring\n");

	ring->size = desc_ring_size;
	ring->is_rx = true;
	rc = wil_ring_alloc_desc_ring(wil, ring);
	if (rc)
		return rc;

	rc = wil_wmi_rx_desc_ring_add(wil, status_ring_id);
	if (rc)
		goto out_free;

	return 0;
out_free:
	wil_ring_free_edma(wil, ring);
	return rc;
}

static void wil_get_reorder_params_edma(struct wil6210_priv *wil,
					struct rte_mbuf *skb, int *tid,
					int *cid, int *mid, u16 *seq,
					int *mcast, int *retry)
{
	struct wil_rx_status_extended *s = wil_skb_rxstatus(skb);

	*tid = wil_rx_status_get_tid(s);
	*cid = wil_rx_status_get_cid(s);
	*mid = wil_rx_status_get_mid(s);
	*seq = le16_to_cpu(wil_rx_status_get_seq(wil, s));
	*mcast = wil_rx_status_get_mcast(s);
	*retry = wil_rx_status_get_retry(s);
}

static void wil_get_netif_rx_params_edma(struct rte_mbuf *skb, int *cid,
					 int *security)
{
	struct wil_rx_status_extended *s = wil_skb_rxstatus(skb);

	*cid = wil_rx_status_get_cid(s);
	*security = wil_rx_status_get_security(s);
}

static int wil_rx_crypto_check_edma(struct wil6210_priv *wil,
				    struct rte_mbuf *skb)
{
	struct wil_rx_status_extended *st;
	int cid, tid, key_id, mc;
	struct wil_sta_info *s;
	struct wil_tid_crypto_rx *c;
	struct wil_tid_crypto_rx_single *cc;
	const u8 *pn;

	/* In HW reorder, HW is responsible for crypto check */
	if (wil->use_rx_hw_reordering)
		return 0;

	/* in full slave mode, handshake/key management handled by master */
	if (slave_mode == 2)
		return 0;

	st = wil_skb_rxstatus(skb);

	cid = wil_rx_status_get_cid(st);
	tid = wil_rx_status_get_tid(st);
	key_id = wil_rx_status_get_key_id(st);
	mc = wil_rx_status_get_mcast(st);
	s = &wil->sta[cid];
	c = mc ? &s->group_crypto_rx : &s->tid_crypto_rx[tid];
	cc = &c->key_id[key_id];
	pn = (u8 *)&st->ext.pn_15_0;

	if (!cc->key_set) {
		wil_err_ratelimited(wil,
				    "Key missing. CID %d TID %d MCast %d KEY_ID %d\n",
				    cid, tid, mc, key_id);
		return -EINVAL;
	}

	if (reverse_memcmp(pn, cc->pn, IEEE80211_GCMP_PN_LEN) <= 0) {
		wil_err_ratelimited(wil,
				    "Replay attack. CID %d TID %d MCast %d KEY_ID %d PN %6phN last %6phN\n",
				    cid, tid, mc, key_id, pn, cc->pn);
		return -EINVAL;
	}
	memcpy(cc->pn, pn, IEEE80211_GCMP_PN_LEN);

	return 0;
}

static bool wil_is_rx_idle_edma(struct wil6210_priv *wil)
{
	struct wil_status_ring *sring;
	struct wil_rx_status_extended msg1;
	void *msg = &msg1;
	u8 dr_bit;
	int i;

	for (i = 0; i < wil->num_rx_status_rings; i++) {
		sring = &wil->srings[i];
		if (!sring->va)
			continue;

		wil_get_next_rx_status_msg(sring, msg);
		dr_bit = wil_rx_status_get_desc_rdy_bit(msg);

		/* Check if there are unhandled RX status messages */
		if (dr_bit == sring->desc_rdy_pol)
			return false;
	}

	return true;
}

static void wil_rx_buf_len_init_edma(struct wil6210_priv *wil)
{
	/* RX buffer size must be aligned to 4 bytes */
	wil->rx_buf_len = rx_large_buf ?
		WIL_MAX_ETH_MTU : WIL_EDMA_RX_BUF_LEN_DEFAULT;
}

static int wil_rx_init_edma(struct wil6210_priv *wil, uint desc_ring_order)
{
	u16 status_ring_size, desc_ring_size = 1 << desc_ring_order;
	struct wil_ring *ring = &wil->ring_rx;
	int rc;
	size_t elem_size = wil->use_compressed_rx_status ?
		sizeof(struct wil_rx_status_compressed) :
		sizeof(struct wil_rx_status_extended);
	int i;

#ifdef DEBUG_LATENCY
	wil->cycle_per_sec = rte_get_timer_hz();
	if (wil->cycle_per_sec > 0)
		wil->nano_per_cycle = SEC_TO_NANO / wil->cycle_per_sec;
	else
		wil->nano_per_cycle = 1;
#endif

	/* In SW reorder one must use extended status messages */
	if (wil->use_compressed_rx_status && !wil->use_rx_hw_reordering) {
		wil_err(wil,
			"compressed RX status cannot be used with SW reorder\n");
		return -EINVAL;
	}
	if (wil->rx_status_ring_order <= desc_ring_order)
		/* make sure sring is larger than desc ring */
		wil->rx_status_ring_order = desc_ring_order + 1;
	if (wil->rx_buff_id_count <= desc_ring_size)
		/* make sure we will not run out of buff_ids */
		wil->rx_buff_id_count = desc_ring_size + 512;
	if (wil->rx_status_ring_order < WIL_SRING_SIZE_ORDER_MIN ||
	    wil->rx_status_ring_order > WIL_SRING_SIZE_ORDER_MAX)
		wil->rx_status_ring_order = WIL_RX_SRING_SIZE_ORDER_DEFAULT;

	status_ring_size = 1 << wil->rx_status_ring_order;

	wil_dbg_misc(wil,
		     "rx_init, desc_ring_size=%u, status_ring_size=%u, elem_size=%zu\n",
		     desc_ring_size, status_ring_size, elem_size);

	wil_rx_buf_len_init_edma(wil);

	/* Use debugfs dbg_num_rx_srings if set, reserve one sring for TX */
	if (wil->num_rx_status_rings > WIL6210_MAX_STATUS_RINGS - 1)
		wil->num_rx_status_rings = WIL6210_MAX_STATUS_RINGS - 1;

	wil_dbg_misc(wil, "rx_init: allocate %d status rings\n",
		     wil->num_rx_status_rings);

	rc = wil_wmi_cfg_def_rx_offload(wil, wil->rx_buf_len);
	if (rc)
		return rc;

	/* Allocate status ring */
	for (i = 0; i < wil->num_rx_status_rings; i++) {
		int sring_id = wil_find_free_sring(wil);

		if (sring_id < 0) {
			rc = -EFAULT;
			goto err_free_status;
		}
		rc = wil_init_rx_sring(wil, status_ring_size, elem_size,
				       sring_id);
		if (rc)
			goto err_free_status;
	}

	/* Allocate descriptor ring */
	rc = wil_init_rx_desc_ring(wil, desc_ring_size,
				   WIL_DEFAULT_RX_STATUS_RING_ID);
	if (rc)
		goto err_free_status;

	if (wil->rx_buff_id_count >= status_ring_size) {
		wil_info(wil,
			 "rx_buff_id_count %d exceeds sring_size %d. set it to %d\n",
			 wil->rx_buff_id_count, status_ring_size,
			 status_ring_size - 1);
		wil->rx_buff_id_count = status_ring_size - 1;
	}

	/* Allocate Rx buffer array */
	rc = wil_init_rx_buff_arr(wil, wil->rx_buff_id_count);
	if (rc)
		goto err_free_desc;

#ifndef WIL6210_PMD /* Postpone until RX queue is started */
	/* Fill descriptor ring with credits */
	rc = wil_rx_refill_edma(wil);
	if (rc)
		goto err_free_rx_buff_arr;
#endif

	return 0;
#ifndef WIL6210_PMD /* Postpone until RX queue is started */
err_free_rx_buff_arr:
	wil_free_rx_buff_arr(wil);
#endif
err_free_desc:
	wil_ring_free_edma(wil, ring);
err_free_status:
	for (i = 0; i < wil->num_rx_status_rings; i++)
		wil_sring_free(wil, &wil->srings[i]);

	return rc;
}

static int wil_ring_init_tx_edma(struct wil6210_vif *vif, int ring_id,
				 int size, int cid, int tid)
{
	struct wil6210_priv *wil = vif_to_wil(vif);
	int rc;
	struct wil_ring *ring = &wil->ring_tx[ring_id];
	struct wil_ring_tx_data *txdata = &wil->ring_tx_data[ring_id];

	lockdep_assert_held(&wil->mutex);

	wil_dbg_misc(wil,
		     "init TX ring: ring_id=%u, cid=%u, tid=%u, sring_id=%u\n",
		     ring_id, cid, tid, wil->tx_sring_idx);

	wil_tx_data_init(txdata);
	ring->size = size;
	rc = wil_ring_alloc_desc_ring(wil, ring);
	if (rc)
		goto out;

	wil->ring2cid_tid[ring_id][0] = cid;
	wil->ring2cid_tid[ring_id][1] = tid;
	if (!vif->privacy)
		txdata->dot1x_open = true;

	rc = wil_wmi_tx_desc_ring_add(vif, ring_id, cid, tid);
	if (rc) {
		wil_err(wil, "WMI_TX_DESC_RING_ADD_CMD failed\n");
		goto out_free;
	}

	if (txdata->dot1x_open && agg_wsize >= 0)
		wil_addba_tx_request(wil, ring_id, agg_wsize);

	return 0;
 out_free:
	spin_lock_bh(&txdata->lock);
	txdata->dot1x_open = false;
	txdata->enabled = 0;
	spin_unlock_bh(&txdata->lock);
	wil_ring_free_edma(wil, ring);
	wil->ring2cid_tid[ring_id][0] = max_assoc_sta;
	wil->ring2cid_tid[ring_id][1] = 0;

 out:
	return rc;
}

static int wil_tx_ring_modify_edma(struct wil6210_vif *vif, int ring_id,
				   int cid, int tid)
{
	struct wil6210_priv *wil = vif_to_wil(vif);

	wil_err(wil, "ring modify is not supported for EDMA\n");

	return -EOPNOTSUPP;
}

/* This function is used only for RX SW reorder */
static int wil_check_bar(struct wil6210_priv *wil, void *msg, int cid,
			 struct rte_mbuf *skb, struct wil_net_stats *stats)
{
	u8 ftype;
	u8 fc1;
	int mid;
	int tid;
	u16 seq;
	struct wil6210_vif *vif;

	ftype = wil_rx_status_get_frame_type(wil, msg);
	if (ftype == IEEE80211_FTYPE_DATA)
		return 0;

	fc1 = wil_rx_status_get_fc1(wil, msg);
	mid = wil_rx_status_get_mid(msg);
	tid = wil_rx_status_get_tid(msg);
	seq = le16_to_cpu(wil_rx_status_get_seq(wil, msg));
	vif = wil->vifs[mid];

	if (unlikely(!vif)) {
		wil_dbg_txrx(wil, "RX descriptor with invalid mid %d", mid);
		return -EAGAIN;
	}

	wil_dbg_txrx(wil,
		     "Non-data frame FC[7:0] 0x%02x MID %d CID %d TID %d Seq 0x%03x\n",
		     fc1, mid, cid, tid, seq);
	if (stats)
		stats->rx_non_data_frame++;
	if (wil_is_back_req(fc1)) {
		wil_dbg_txrx(wil,
			     "BAR: MID %d CID %d TID %d Seq 0x%03x\n",
			     mid, cid, tid, seq);
		wil_rx_bar(wil, vif, cid, tid, seq);
	} else {
		u32 sz = wil->use_compressed_rx_status ?
			sizeof(struct wil_rx_status_compressed) :
			sizeof(struct wil_rx_status_extended);

		/* print again all info. One can enable only this
		 * without overhead for printing every Rx frame
		 */
		wil_dbg_txrx(wil,
			     "Unhandled non-data frame FC[7:0] 0x%02x MID %d CID %d TID %d Seq 0x%03x\n",
			     fc1, mid, cid, tid, seq);
		wil_hex_dump_txrx("RxS ", DUMP_PREFIX_NONE, 32, 4,
				  (const void *)msg, sz, false);
		wil_hex_dump_txrx("Rx ", DUMP_PREFIX_OFFSET, 16, 1,
				  rte_pktmbuf_mtod(skb, void *), skb_headlen(skb), false);
	}

	return -EAGAIN;
}

static int wil_rx_error_check_edma(struct wil6210_priv *wil,
				   struct rte_mbuf *skb,
				   struct wil_net_stats *stats)
{
	int l2_rx_status;
	void *msg = wil_skb_rxstatus(skb);

	l2_rx_status = wil_rx_status_get_l2_rx_status(msg);
	if (l2_rx_status != 0) {
		wil_dbg_txrx(wil, "L2 RX error, l2_rx_status=0x%x\n",
			     l2_rx_status);
		/* Due to HW issue, KEY error will trigger a MIC error */
		if (l2_rx_status == WIL_RX_EDMA_ERROR_MIC) {
			wil_err_ratelimited(wil,
					    "L2 MIC/KEY error, dropping packet\n");
			stats->rx_mic_error++;
		}
		if (l2_rx_status == WIL_RX_EDMA_ERROR_KEY) {
			wil_err_ratelimited(wil,
					    "L2 KEY error, dropping packet\n");
			stats->rx_key_error++;
		}
		if (l2_rx_status == WIL_RX_EDMA_ERROR_REPLAY) {
			wil_err_ratelimited(wil,
					    "L2 REPLAY error, dropping packet\n");
			stats->rx_replay++;
		}
		if (l2_rx_status == WIL_RX_EDMA_ERROR_AMSDU) {
			wil_err_ratelimited(wil,
					    "L2 AMSDU error, dropping packet\n");
			stats->rx_amsdu_error++;
		}
		return -EFAULT;
	}

	skb->ol_flags = wil_rx_status_get_checksum(msg, stats);
	return 0;
}

static struct rte_mbuf *wil_sring_reap_rx_edma(struct wil6210_priv *wil,
					      struct wil_status_ring *sring,
					      uint16_t port_id)
{
	struct wil_rx_status_extended msg1;
	void *msg = &msg1;
	u16 buff_id;
	struct rte_mbuf *skb;
	struct wil_ring_rx_data *rxdata = &sring->rx_data;
	unsigned int sz = wil->rx_buf_len;
	struct wil_net_stats *stats = NULL;
	u16 dmalen;
	int cid;
	bool eop;
	u8 dr_bit;
	u8 data_offset;
	struct wil_rx_status_extended *s;
	u16 sring_idx = sring - wil->srings;

	RTE_BUILD_BUG_ON(sizeof(struct wil_rx_status_extended) > RTE_PKTMBUF_HEADROOM);

again:
	wil_get_next_rx_status_msg(sring, msg);
	dr_bit = wil_rx_status_get_desc_rdy_bit(msg);

	/* Completed handling all the ready status messages */
	if (dr_bit != sring->desc_rdy_pol)
		return NULL;

	/* Extract the buffer ID from the status message */
	buff_id = le16_to_cpu(wil_rx_status_get_buff_id(msg));

	while (!buff_id) {
		struct wil_rx_status_extended *s;
		int invalid_buff_id_retry = 0;

		wil_dbg_txrx(wil,
			     "buff_id is not updated yet by HW, (swhead 0x%x)\n",
			     sring->swhead);
		if (++invalid_buff_id_retry > MAX_INVALID_BUFF_ID_RETRY)
			break;

		/* Read the status message again */
		s = (struct wil_rx_status_extended *)
			(sring->va + (sring->elem_size * sring->swhead));
		*(struct wil_rx_status_extended *)msg = *s;
		buff_id = le16_to_cpu(wil_rx_status_get_buff_id(msg));
	}

	if (unlikely(!wil_val_in_range(buff_id, 1, wil->rx_buff_mgmt.size))) {
		wil_err(wil, "Corrupt buff_id=%d, sring->swhead=%d\n",
			buff_id, sring->swhead);
		wil_rx_status_reset_buff_id(sring);
		wil_sring_advance_swhead(sring);
		sring->invalid_buff_id_cnt++;
		goto again;
	}
	dmalen = le16_to_cpu(wil_rx_status_get_length(msg));

	//trace_wil6210_rx_status(wil, wil->use_compressed_rx_status, buff_id,
	//			msg);
	wil_dbg_txrx(wil, "Rx, buff_id=%u, sring_idx=%u, swhead=%d, dmalen=%u bytes\n",
		     buff_id, sring_idx, sring->swhead, dmalen);
	wil_hex_dump_txrx("RxS ", DUMP_PREFIX_NONE, 32, 4,
			  (const void *)msg, wil->use_compressed_rx_status ?
			  sizeof(struct wil_rx_status_compressed) :
			  sizeof(struct wil_rx_status_extended), false);

	/* Extract the SKB from the rx_buff management array */
	skb = wil->rx_buff_mgmt.buff_arr[buff_id].skb;
	if (!skb) {
		wil_err(wil, "No Rx skb at buff_id %d\n", buff_id);
		wil_rx_status_reset_buff_id(sring);
		/* Move the buffer from the active list to the free list */
		list_move_tail(&wil->rx_buff_mgmt.buff_arr[buff_id].list,
			       &wil->rx_buff_mgmt.free);
		wil_sring_advance_swhead(sring);
		sring->invalid_buff_id_cnt++;
		goto again;
	}
	wil->rx_buff_mgmt.buff_arr[buff_id].skb = NULL;

	wil_rx_status_reset_buff_id(sring);
	wil_sring_advance_swhead(sring);

	/* Move the buffer from the active list to the free list */
	list_move_tail(&wil->rx_buff_mgmt.buff_arr[buff_id].list,
		       &wil->rx_buff_mgmt.free);

	eop = wil_rx_status_get_eop(msg);

	cid = wil_rx_status_get_cid(msg);
	if (unlikely(!wil_val_in_range(cid, 0, max_assoc_sta))) {
		wil_err(wil, "Corrupt cid=%d, sring->swhead=%d\n",
			cid, sring->swhead);
		rxdata->skipping = true;
		goto skipping;
	}
	stats = &wil->sta[cid].stats;

	if (unlikely(dmalen > sz)) {
		wil_err(wil, "Rx size too large: %d bytes!\n", dmalen);
		stats->rx_large_frame++;
		rxdata->skipping = true;
	}

skipping:
	/* skipping indicates if a certain SKB should be dropped.
	 * It is set in case there is an error on the current SKB or in case
	 * of RX chaining: as long as we manage to merge the SKBs it will
	 * be false. once we have a bad SKB or we don't manage to merge SKBs
	 * it will be set to the !EOP value of the current SKB.
	 * This guarantees that all the following SKBs until EOP will also
	 * get dropped.
	 */
	if (unlikely(rxdata->skipping)) {
		rte_mbuf_raw_free(skb);
		if (rxdata->skb) {
			rte_pktmbuf_free(rxdata->skb);
			rxdata->skb = NULL;
		}
		rxdata->skipping = !eop;
		goto again;
	}

	/* Initialize the mbuf */
	skb->data_off = RTE_PKTMBUF_HEADROOM;
	rte_packet_prefetch(rte_pktmbuf_mtod(skb, char *));
	skb->pkt_len = dmalen;
	skb->data_len = dmalen;
	skb->port = port_id;

	if (!rxdata->skb) {
		rxdata->skb = skb;
	} else {
		struct rte_mbuf *seg;

		seg = rxdata->skb->userdata;
		if (seg != NULL)
			seg->next = skb;
		else
			rxdata->skb->next = skb;
		/* Attempt to chain mbuf as the fragment */
		rxdata->skb->nb_segs++;
		rxdata->skb->pkt_len += dmalen;
		/* Remember last mbuf in chain */
		rxdata->skb->userdata = skb;
	}

	if (!eop)
		goto again;

	/* reaching here rxdata->skb always contains a full packet */
	skb = rxdata->skb;
	skb->userdata = NULL;
	rxdata->skb = NULL;
	rxdata->skipping = false;

	if (unlikely(skb->pkt_len < ETH_HLEN)) {
		wil_dbg_txrx(wil, "Short frame, len = %d\n", skb->pkt_len);
		stats->rx_short_frame++;
		rte_pktmbuf_free(skb);
		goto again;
	}

	if (stats) {
		stats->last_mcs_rx = wil_rx_status_get_mcs(msg);
		if (stats->last_mcs_rx < ARRAY_SIZE(stats->rx_per_mcs))
			stats->rx_per_mcs[stats->last_mcs_rx]++;
	}

	if (!wil->use_rx_hw_reordering && !wil->use_compressed_rx_status &&
	    wil_check_bar(wil, msg, cid, skb, stats) == -EAGAIN) {
		rte_pktmbuf_free(skb);
		goto again;
	}

	/* Compensate for the HW data alignment according to the status
	 * message
	 */
	data_offset = wil_rx_status_get_data_offset(msg);
	if (data_offset == 0xFF ||
	    data_offset > WIL_EDMA_MAX_DATA_OFFSET) {
		wil_err(wil, "Unexpected data offset %d\n", data_offset);
		rte_pktmbuf_free(skb);
		goto again;
	}

	rte_pktmbuf_adj(skb, data_offset);

	wil_hex_dump_txrx("Rx ", DUMP_PREFIX_OFFSET, 16, 1,
			 rte_pktmbuf_mtod(skb, void *), skb_headlen(skb), false);

	/*
	 * Save message for future
	 */
	s = wil_skb_rxstatus(skb);
	memcpy(s, msg, sring->elem_size);

	return skb;
}


uint16_t wil_rx_burst(struct wil6210_priv *wil, struct rte_mbuf **rx_pkts,
    int16_t nb_pkts)
{
	struct wil_ring *ring = &wil->ring_rx;
	struct wil_status_ring *sring;
	struct rte_mbuf *mbuf;
	unsigned int nb_done;
	u32 swhead;
	int i;

#ifdef DEBUG_LATENCY
	u64 start = rte_get_timer_cycles() * wil->nano_per_cycle;
	u64 diff = start - wil->last_burst_rx_nano;
	if (diff > wil->max_rx_diff_nano) {
		wil->max_rx_diff_nano = diff;
	}
#endif
#ifdef ENABLE_PMC_STATS
	u64 prev_count[RTE_PMC_NUM_COUNTERS];
	if (wil->xstats_flags & XSTATS_FLAGS_PMC_COUNTERS_3) {
		prev_count[0] = rte_read_perf_counter_0();
		prev_count[1] = rte_read_perf_counter_1();
		prev_count[2] = rte_read_perf_counter_2();
	}
#endif
	wil->count_rx_burst_calls++;

	if (unlikely(!ring->va)) {
			wil_err(wil, "Burst Rx while Rx not yet initialized\n");
		return 0;
	}

	nb_done = 0;
	for (i = 0; i < wil->num_rx_status_rings; i++) {
		sring = &wil->srings[i];
		if (unlikely(!sring->va)) {
			wil_err(wil,
				"Rx IRQ while Rx status ring %d not yet initialized\n",
				i);
			continue;
		}
		swhead = sring->swhead;
		while (nb_done < nb_pkts &&
		       (NULL != (mbuf =
			wil_sring_reap_rx_edma(wil, sring, ring->port_id)))) {
			if (wil->use_rx_hw_reordering) {
				void *msg = wil_skb_rxstatus(mbuf);
				int mid = wil_rx_status_get_mid(msg);
				struct wil6210_vif *vif = wil->vifs[mid];
				struct wil_net_stats *stats;
				int cid, security;

				if (unlikely(!vif)) {
					wil_dbg_txrx(wil,
						     "RX desc invalid mid %d",
						     mid);
					rte_pktmbuf_free(mbuf);
					continue;
				}

				/* Get CID parameters */
				wil->txrx_ops.get_netif_rx_params(mbuf, &cid, &security);
				stats = &wil->sta[cid].stats;

				/* check errors reported by HW and update statistics */
				if (unlikely(wil->txrx_ops.rx_error_check(wil, mbuf, stats))) {
					rte_pktmbuf_free(mbuf);
					continue;
				}

				/*
				 * Translate PEER is to terra unit number
				 */
				mbuf->udata64 = wil->sta[cid].link_id;

				nb_done++;
				*rx_pkts++ = mbuf;
				stats->rx_packets++;
				stats->rx_bytes += mbuf->pkt_len;
			} else {
				wil_rx_reorder(wil, mbuf);
			}
		}

		if (sring->swhead != swhead)
			wil_w(wil, sring->hwtail, (sring->swhead - 1) % sring->size);
	}

	wil_rx_refill_edma(wil);

	/* Process TX completions */
	sring = &wil->srings[wil->tx_sring_idx];
	if (sring->va)
		(void)wil_tx_sring_handler(wil, sring);

#ifdef ENABLE_PMC_STATS
	if (wil->xstats_flags & XSTATS_FLAGS_PMC_COUNTERS_3) {
		wil->pmc[PMC_COUNTERS_RX_OUTER][0] +=
			rte_pmc_sub_counters(prev_count[0], rte_read_perf_counter_0());
		wil->pmc[PMC_COUNTERS_RX_OUTER][1] +=
			rte_pmc_sub_counters(prev_count[1], rte_read_perf_counter_1());
		wil->pmc[PMC_COUNTERS_RX_OUTER][2] +=
			rte_pmc_sub_counters(prev_count[2], rte_read_perf_counter_2());
	}
#endif
#ifdef DEBUG_LATENCY
	wil->last_burst_rx_nano = rte_get_timer_cycles() * wil->nano_per_cycle;
	wil->wil_rx_handler_time_nano += wil->last_burst_rx_nano - start;
#endif
	return nb_done;
}

void wil_rx_handle_edma(struct wil6210_priv *wil, int *quota)
{
	struct net_device *ndev;
	struct wil_ring *ring = &wil->ring_rx;
	struct wil_status_ring *sring;
	struct rte_mbuf *skb;
	int i;

	if (unlikely(!ring->va)) {
		wil_err(wil, "Rx IRQ while Rx not yet initialized\n");
		return;
	}
	wil_dbg_txrx(wil, "rx_handle\n");

	for (i = 0; i < wil->num_rx_status_rings; i++) {
		sring = &wil->srings[i];
		if (unlikely(!sring->va)) {
			wil_err(wil,
				"Rx IRQ while Rx status ring %d not yet initialized\n",
				i);
			continue;
		}

		while ((*quota > 0) &&
		       (NULL != (skb =
			wil_sring_reap_rx_edma(wil, sring, ring->port_id)))) {
			(*quota)--;
			if (wil->use_rx_hw_reordering) {
				void *msg = wil_skb_rxstatus(skb);
				int mid = wil_rx_status_get_mid(msg);
				struct wil6210_vif *vif = wil->vifs[mid];

				if (unlikely(!vif)) {
					wil_dbg_txrx(wil,
						     "RX desc invalid mid %d",
						     mid);
					rte_pktmbuf_free(skb);
					continue;
				}
				ndev = vif_to_ndev(vif);
				wil_netif_rx_any(skb, ndev);
			} else {
				wil_rx_reorder(wil, skb);
			}
		}

		wil_w(wil, sring->hwtail, (sring->swhead - 1) % sring->size);
	}

	wil_rx_refill_edma(wil);
}

static int wil_tx_desc_map_edma(union wil_tx_desc *desc,
				dma_addr_t pa,
				u32 len,
				int ring_index)
{
	struct wil_tx_enhanced_desc *d =
		(struct wil_tx_enhanced_desc *)&desc->enhanced;

	memset(d, 0, sizeof(struct wil_tx_enhanced_desc));

	wil_desc_set_addr_edma(&d->dma.addr, &d->dma.addr_high_high, pa);

	/* 0..6: mac_length; 7:ip_version 0-IP6 1-IP4*/
	d->dma.length = cpu_to_le16((u16)len);
	d->mac.d[0] = (ring_index << WIL_EDMA_DESC_TX_MAC_CFG_0_QID_POS);
	/* translation type:  0 - bypass; 1 - 802.3; 2 - native wifi;
	 * 3 - eth mode
	 */
	d->mac.d[2] = BIT(MAC_CFG_DESC_TX_2_SNAP_HDR_INSERTION_EN_POS) |
		      (0x3 << MAC_CFG_DESC_TX_2_L2_TRANSLATION_TYPE_POS);

	return 0;
}

static inline void
wil_get_next_tx_status_msg(struct wil_status_ring *sring,
			   struct wil_ring_tx_status *msg)
{
	volatile struct wil_ring_tx_status *_msg = (struct wil_ring_tx_status *)
		(sring->va + (sring->elem_size * sring->swhead));

	*msg = *_msg;
}

/**
 * Clean up transmitted skb's from the Tx descriptor RING.
 * Return number of descriptors cleared.
 */
int wil_tx_sring_handler(struct wil6210_priv *wil,
			 struct wil_status_ring *sring)
{
	struct net_device *ndev;
	struct device *dev = wil_to_dev(wil);
	struct wil_ring *ring = NULL;
	struct wil_ring_tx_data *txdata;
	/* Total number of completed descriptors in all descriptor rings */
	int desc_cnt = 0;
	int cid;
	struct wil_net_stats *stats;
	struct wil_tx_enhanced_desc *_d;
	unsigned int ring_id;
	unsigned int num_descs, num_statuses = 0;
	int i;
	u8 dr_bit; /* Descriptor Ready bit */
	struct wil_ring_tx_status msg;
	struct wil6210_vif *vif;
	int used_before_complete;
	int used_new;

	wil_get_next_tx_status_msg(sring, &msg);
	dr_bit = msg.desc_ready >> TX_STATUS_DESC_READY_POS;

	/* Process completion messages while DR bit has the expected polarity */
	while (dr_bit == sring->desc_rdy_pol) {
		num_descs = msg.num_descriptors;
		if (!num_descs) {
			wil_err(wil, "invalid num_descs 0\n");
			goto again;
		}

		/* Find the corresponding descriptor ring */
		ring_id = msg.ring_id;

		if (unlikely(ring_id >= WIL6210_MAX_TX_RINGS)) {
			wil_err(wil, "invalid ring id %d\n", ring_id);
			goto again;
		}
		ring = &wil->ring_tx[ring_id];
		if (unlikely(!ring->va)) {
			wil_err(wil, "Tx irq[%d]: ring not initialized\n",
				ring_id);
			goto again;
		}
		txdata = &wil->ring_tx_data[ring_id];
		if (unlikely(!txdata->enabled)) {
			wil_info(wil, "Tx irq[%d]: ring disabled\n", ring_id);
			goto again;
		}
		vif = wil->vifs[txdata->mid];
		if (unlikely(!vif)) {
			wil_dbg_txrx(wil, "invalid MID %d for ring %d\n",
				     txdata->mid, ring_id);
			goto again;
		}

		ndev = vif_to_ndev(vif);

		cid = wil->ring2cid_tid[ring_id][0];
		stats = (cid < max_assoc_sta ? &wil->sta[cid].stats : NULL);

		wil_dbg_txrx(wil,
			     "tx_status: completed desc_ring (%d), num_descs (%d)\n",
			     ring_id, num_descs);

		used_before_complete = wil_ring_used_tx(ring);

		for (i = 0 ; i < num_descs; ++i) {
			struct wil_ctx *ctx = &ring->ctx[ring->swtail];
			struct wil_tx_enhanced_desc dd, *d = &dd;
			u16 dmalen;
			struct rte_mbuf *skb = ctx->skb;

			_d = (struct wil_tx_enhanced_desc *)
				(uintptr_t)&ring->va[ring->swtail].tx.enhanced;
			*d = *_d;

			dmalen = le16_to_cpu(d->dma.length);
			//trace_wil6210_tx_status(&msg, ring->swtail, dmalen);
			wil_dbg_txrx(wil,
				     "TxC[%2d][%3d] : %d bytes, status 0x%02x\n",
				     ring_id, ring->swtail, dmalen,
				     msg.status);
			wil_hex_dump_txrx("TxS ", DUMP_PREFIX_NONE, 32, 4,
					  (const void *)&msg, sizeof(msg),
					  false);

			wil_tx_desc_unmap_edma(dev,
					       (union wil_tx_desc *)d,
					       ctx);

			if (skb) {
				if (likely(msg.status == 0)) {
					ndev->stats.tx_packets++;
					ndev->stats.tx_bytes += skb->pkt_len;
					if (stats) {
						stats->tx_packets++;
						stats->tx_bytes += skb->pkt_len;
					}
				} else {
					ndev->stats.tx_errors++;
					if (stats)
						stats->tx_errors++;
				}

				if (stats) {
					atomic_dec(
						&stats->tx_pend_packets);
					atomic_sub(skb->pkt_len,
						   &stats->tx_pend_bytes);
				}

				rte_pktmbuf_free(skb);
			}
			memset(ctx, 0, sizeof(*ctx));
			/* Make sure the ctx is zeroed before updating the tail
			 * to prevent a case where wil_tx_ring will see
			 * this descriptor as used and handle it before ctx zero
			 * is completed.
			 */
			wmb();

			ring->swtail = wil_ring_next_tail(ring);

			desc_cnt++;
		}

		/* performance monitoring */
		used_new = wil_ring_used_tx(ring);
		if (wil_val_in_range(wil->ring_idle_trsh,
				     used_new, used_before_complete)) {
			wil_dbg_txrx(wil, "Ring[%2d] idle %d -> %d\n",
				     ring_id, used_before_complete, used_new);
			txdata->last_idle = get_cycles();
		}

again:
		num_statuses++;
		if (num_statuses % WIL_EDMA_TX_SRING_UPDATE_HW_TAIL == 0)
			/* update HW tail to allow HW to push new statuses */
			wil_w(wil, sring->hwtail, sring->swhead);

		wil_sring_advance_swhead(sring);

		wil_get_next_tx_status_msg(sring, &msg);
		dr_bit = msg.desc_ready >> TX_STATUS_DESC_READY_POS;
	}

	/* shall we wake net queues? */
	if (desc_cnt)
		wil_update_net_queues(wil, vif, ring, false);

	if (num_statuses % WIL_EDMA_TX_SRING_UPDATE_HW_TAIL != 0)
		/* Update the HW tail ptr (RD ptr) */
		wil_w(wil, sring->hwtail, (sring->swhead - 1) % sring->size);

	return desc_cnt;
}

/**
 * Sets the descriptor @d up for csum and/or TSO offloading. The corresponding
 * @skb is used to obtain the protocol and headers length.
 * @tso_desc_type is a descriptor type for TSO: 0 - a header, 1 - first data,
 * 2 - middle, 3 - last descriptor.
 */
static void wil_tx_desc_offload_setup_tso_edma(struct wil_tx_enhanced_desc *d,
					       int tso_desc_type, bool is_ipv4,
					       int tcp_hdr_len,
					       int skb_net_hdr_len,
					       int mss)
{
	/* Number of descriptors */
	d->mac.d[2] |= 1;
	/* Maximum Segment Size */
	d->mac.tso_mss |= cpu_to_le16(mss >> 2);
	/* L4 header len: TCP header length */
	d->dma.l4_hdr_len |= tcp_hdr_len & DMA_CFG_DESC_TX_0_L4_LENGTH_MSK;
	/* EOP, TSO desc type, Segmentation enable,
	 * Insert IPv4 and TCP / UDP Checksum
	 */
	d->dma.cmd |= BIT(WIL_EDMA_DESC_TX_CFG_EOP_POS) |
		      tso_desc_type << WIL_EDMA_DESC_TX_CFG_TSO_DESC_TYPE_POS |
		      BIT(WIL_EDMA_DESC_TX_CFG_SEG_EN_POS) |
		      BIT(WIL_EDMA_DESC_TX_CFG_INSERT_IP_CHKSUM_POS) |
		      BIT(WIL_EDMA_DESC_TX_CFG_INSERT_TCP_CHKSUM_POS);
	/* Calculate pseudo-header */
	d->dma.w1 |= BIT(WIL_EDMA_DESC_TX_CFG_PSEUDO_HEADER_CALC_EN_POS) |
		     BIT(WIL_EDMA_DESC_TX_CFG_L4_TYPE_POS);
	/* IP Header Length */
	d->dma.ip_length |= skb_net_hdr_len;
	/* MAC header length and IP address family*/
	d->dma.b11 |= ETH_HLEN |
		      is_ipv4 << DMA_CFG_DESC_TX_OFFLOAD_CFG_L3T_IPV4_POS;
}

static int wil_tx_tso_gen_desc(struct wil6210_priv *wil, void *buff_addr,
			       int len, uint i, int tso_desc_type,
			       skb_frag_t *frag, struct wil_ring *ring,
			       struct rte_mbuf *skb, bool is_ipv4,
			       int tcp_hdr_len, int skb_net_hdr_len,
			       int mss, int *descs_used)
{
#ifndef WIL6210_PMD
	struct device *dev = wil_to_dev(wil);
	struct wil_tx_enhanced_desc *_desc = (struct wil_tx_enhanced_desc *)
		(uintptr_t)&ring->va[i].tx.enhanced;
	struct wil_tx_enhanced_desc desc_mem, *d = &desc_mem;
	int ring_index = ring - wil->ring_tx;
	dma_addr_t pa;

	if (len == 0)
		return 0;

	if (!frag) {
		pa = dma_map_single(dev, buff_addr, len, DMA_TO_DEVICE);
		ring->ctx[i].mapped_as = wil_mapped_as_single;
	} else {
		pa = skb_frag_dma_map(dev, frag, 0, len, DMA_TO_DEVICE);
		ring->ctx[i].mapped_as = wil_mapped_as_page;
	}
	if (unlikely(dma_mapping_error(dev, pa))) {
		wil_err(wil, "TSO: Skb DMA map error\n");
		return -EINVAL;
	}

	wil->txrx_ops.tx_desc_map((union wil_tx_desc *)d, pa,
				   len, ring_index);
	wil_tx_desc_offload_setup_tso_edma(d, tso_desc_type, is_ipv4,
					   tcp_hdr_len,
					   skb_net_hdr_len, mss);

	/* hold reference to skb
	 * to prevent skb release before accounting
	 * in case of immediate "tx done"
	 */
	if (tso_desc_type == wil_tso_type_lst)
		ring->ctx[i].skb = skb_get(skb);

	wil_hex_dump_txrx("TxD ", DUMP_PREFIX_NONE, 32, 4,
			  (const void *)d, sizeof(*d), false);

	*_desc = *d;
	(*descs_used)++;
#else
	(void)wil_tx_desc_offload_setup_tso_edma;
#endif /* WIL6210_PMD */

	return 0;
}

static int __wil_tx_ring_tso_edma(struct wil6210_priv *wil,
				  struct wil6210_vif *vif,
				  struct wil_ring *ring,
				  struct rte_mbuf *skb)
{
#ifndef WIL6210_PMD /* WIL6210_PMD */
	int ring_index = ring - wil->ring_tx;
	struct wil_ring_tx_data *txdata = &wil->ring_tx_data[ring_index];
	int nr_frags = skb_shinfo(skb)->nr_frags;
	int min_desc_required = nr_frags + 2; /* Headers, Head, Fragments */
	int used, avail = wil_ring_avail_tx(ring);
	int f, hdrlen, headlen;
	int gso_type;
	bool is_ipv4;
	u32 swhead = ring->swhead;
	int descs_used = 0; /* total number of used descriptors */
	int rc = -EINVAL;
	int tcp_hdr_len;
	int skb_net_hdr_len;
	int mss = skb_shinfo(skb)->gso_size;
	u8 cid = txdata->cid;
	struct wil_net_stats *stats = (cid < WIL6210_MAX_CID) ?
		&wil->sta[cid].stats : NULL;

	wil_dbg_txrx(wil, "tx_ring_tso: %d bytes to ring %d\n", skb->len,
		     ring_index);

	if (unlikely(!txdata->enabled))
		return -EINVAL;

	if (unlikely(avail < min_desc_required)) {
		wil_err_ratelimited(wil,
				    "TSO: Tx ring[%2d] full. No space for %d fragments\n",
				    ring_index, min_desc_required);
		return -ENOMEM;
	}

	gso_type = skb_shinfo(skb)->gso_type & (SKB_GSO_TCPV6 | SKB_GSO_TCPV4);
	switch (gso_type) {
	case SKB_GSO_TCPV4:
		is_ipv4 = true;
		break;
	case SKB_GSO_TCPV6:
		is_ipv4 = false;
		break;
	default:
		return -EINVAL;
	}

	if (skb->ip_summed != CHECKSUM_PARTIAL)
		return -EINVAL;

	/* tcp header length and skb network header length are fixed for all
	 * packet's descriptors - read them once here
	 */
	tcp_hdr_len = tcp_hdrlen(skb);
	skb_net_hdr_len = skb_network_header_len(skb);

	/* First descriptor must contain the header only
	 * Header Length = MAC header len + IP header len + TCP header len
	 */
	hdrlen = ETH_HLEN + tcp_hdr_len + skb_net_hdr_len;
	wil_dbg_txrx(wil, "TSO: process header descriptor, hdrlen %u\n",
		     hdrlen);
	rc = wil_tx_tso_gen_desc(wil, skb->data, hdrlen, swhead,
				 wil_tso_type_hdr, NULL, ring, skb,
				 is_ipv4, tcp_hdr_len, skb_net_hdr_len,
				 mss, &descs_used);
	if (rc)
		return -EINVAL;

	/* Second descriptor contains the head */
	headlen = skb_headlen(skb) - hdrlen;
	wil_dbg_txrx(wil, "TSO: process skb head, headlen %u\n", headlen);
	rc = wil_tx_tso_gen_desc(wil, skb->data + hdrlen, headlen,
				 (swhead + descs_used) % ring->size,
				 (nr_frags != 0) ? wil_tso_type_first :
				 wil_tso_type_lst, NULL, ring, skb,
				 is_ipv4, tcp_hdr_len, skb_net_hdr_len,
				 mss, &descs_used);
	if (rc)
		goto mem_error;

	/* Rest of the descriptors are from the SKB fragments */
	for (f = 0; f < nr_frags; f++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[f];
		int len = frag->size;

		wil_dbg_txrx(wil, "TSO: frag[%d]: len %u, descs_used %d\n", f,
			     len, descs_used);

		rc = wil_tx_tso_gen_desc(wil, NULL, len,
					 (swhead + descs_used) % ring->size,
					 (f != nr_frags - 1) ?
					 wil_tso_type_mid : wil_tso_type_lst,
					 frag, ring, skb, is_ipv4,
					 tcp_hdr_len, skb_net_hdr_len,
					 mss, &descs_used);
		if (rc)
			goto mem_error;
	}

	/* performance monitoring */
	used = wil_ring_used_tx(ring);
	if (wil_val_in_range(wil->ring_idle_trsh,
			     used, used + descs_used)) {
		txdata->idle += get_cycles() - txdata->last_idle;
		wil_dbg_txrx(wil,  "Ring[%2d] not idle %d -> %d\n",
			     ring_index, used, used + descs_used);
	}

	/* advance swhead */
	wil_ring_advance_head(ring, descs_used);
	wil_dbg_txrx(wil, "TSO: Tx swhead %d -> %d\n", swhead, ring->swhead);

	/* make sure all writes to descriptors (shared memory) are done before
	 * committing them to HW
	 */
	wmb();

	if (stats) {
		atomic_inc(&stats->tx_pend_packets);
		atomic_add(skb->len, &stats->tx_pend_bytes);
	}

	wil_w(wil, ring->hwtail, ring->swhead);

	return 0;

mem_error:
	while (descs_used > 0) {
		struct device *dev = wil_to_dev(wil);
		struct wil_ctx *ctx;
		int i = (swhead + descs_used - 1) % ring->size;
		struct wil_tx_enhanced_desc dd, *d = &dd;
		struct wil_tx_enhanced_desc *_desc =
			(struct wil_tx_enhanced_desc *)
			(uintptr_t)&ring->va[i].tx.enhanced;

		*d = *_desc;
		ctx = &ring->ctx[i];
		wil_tx_desc_unmap_edma(dev, (union wil_tx_desc *)d, ctx);
		memset(ctx, 0, sizeof(*ctx));
		descs_used--;
	}
	return rc;
#else
	(void)wil_tx_tso_gen_desc;
	return 0;
#endif /* WIL6210_PMD */
}

static int wil_ring_init_bcast_edma(struct wil6210_vif *vif, int ring_id,
				    int size)
{
	struct wil6210_priv *wil = vif_to_wil(vif);
	struct wil_ring *ring = &wil->ring_tx[ring_id];
	int rc;
	struct wil_ring_tx_data *txdata = &wil->ring_tx_data[ring_id];

	wil_dbg_misc(wil, "init bcast: ring_id=%d, sring_id=%d\n",
		     ring_id, wil->tx_sring_idx);

	lockdep_assert_held(&wil->mutex);

	wil_tx_data_init(txdata);
	ring->size = size;
	ring->is_rx = false;
	rc = wil_ring_alloc_desc_ring(wil, ring);
	if (rc)
		goto out;

	wil->ring2cid_tid[ring_id][0] = WIL6210_MAX_CID; /* CID */
	wil->ring2cid_tid[ring_id][1] = 0; /* TID */
	if (!vif->privacy)
		txdata->dot1x_open = true;

	rc = wil_wmi_bcast_desc_ring_add(vif, ring_id);
	if (rc)
		goto out_free;

	return 0;

 out_free:
	spin_lock_bh(&txdata->lock);
	txdata->enabled = 0;
	txdata->dot1x_open = false;
	spin_unlock_bh(&txdata->lock);
	wil_ring_free_edma(wil, ring);

out:
	return rc;
}

static void wil_tx_fini_edma(struct wil6210_priv *wil)
{
	struct wil_status_ring *sring = &wil->srings[wil->tx_sring_idx];

	wil_dbg_misc(wil, "free TX sring\n");

	wil_sring_free(wil, sring);
}

static void wil_rx_data_free(struct wil_status_ring *sring)
{
	if (!sring)
		return;

	rte_pktmbuf_free(sring->rx_data.skb);
	sring->rx_data.skb = NULL;
}

static void wil_rx_fini_edma(struct wil6210_priv *wil)
{
	struct wil_ring *ring = &wil->ring_rx;
	int i;

	wil_dbg_misc(wil, "rx_fini_edma\n");

	wil_ring_free_edma(wil, ring);

	for (i = 0; i < wil->num_rx_status_rings; i++) {
		wil_rx_data_free(&wil->srings[i]);
		wil_sring_free(wil, &wil->srings[i]);
	}

	wil_free_rx_buff_arr(wil);
}

void wil_init_txrx_ops_edma(struct wil6210_priv *wil)
{
	wil->txrx_ops.configure_interrupt_moderation =
		wil_configure_interrupt_moderation_edma;
	/* TX ops */
	wil->txrx_ops.ring_init_tx = wil_ring_init_tx_edma;
	wil->txrx_ops.ring_fini_tx = wil_ring_free_edma;
	wil->txrx_ops.ring_init_bcast = wil_ring_init_bcast_edma;
	wil->txrx_ops.tx_init = wil_tx_init_edma;
	wil->txrx_ops.tx_fini = wil_tx_fini_edma;
	wil->txrx_ops.tx_desc_map = wil_tx_desc_map_edma;
	wil->txrx_ops.tx_desc_unmap = wil_tx_desc_unmap_edma;
	wil->txrx_ops.tx_ring_tso = __wil_tx_ring_tso_edma;
	wil->txrx_ops.tx_ring_modify = wil_tx_ring_modify_edma;
	/* RX ops */
	wil->txrx_ops.rx_init = wil_rx_init_edma;
	wil->txrx_ops.wmi_addba_rx_resp = wmi_addba_rx_resp_edma;
	wil->txrx_ops.get_reorder_params = wil_get_reorder_params_edma;
	wil->txrx_ops.get_netif_rx_params = wil_get_netif_rx_params_edma;
	wil->txrx_ops.rx_crypto_check = wil_rx_crypto_check_edma;
	wil->txrx_ops.rx_error_check = wil_rx_error_check_edma;
	wil->txrx_ops.is_rx_idle = wil_is_rx_idle_edma;
	wil->txrx_ops.rx_fini = wil_rx_fini_edma;
}
