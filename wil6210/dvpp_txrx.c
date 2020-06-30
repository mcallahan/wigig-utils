// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2012-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/etherdevice.h>
#include <linux/moduleparam.h>
#include <linux/prefetch.h>
#include <linux/types.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#include "wil6210.h"
#include "txrx_edma.h"
#include "txrx.h"
#include "trace.h"

uint dvpp_inited;

/* Log rate limiting. */
static ktime_t _last_print;

/*
 * We use stubbed function, so as to allow for disabling the
 * ops asynchronously, while they can spuriously be called
 * from interrupt context. */

void stub_register_ops(dvpp_ops_t *ops) {};
int stub_port_state(void *context, unsigned int port, void *addr,
			   unsigned int enable) { return -ENODEV; };
int stub_pipe_state(unsigned int port, unsigned int flow, void *addr,
			   unsigned int enable) { return -ENODEV; };
void stub_port_free_mini(dvpp_desc_t *mini, u32 port) {};
int stub_port_alloc_mini(u32 port, dvpp_desc_t *mini) { return -ENODEV; };
void *stub_get_desc_kernel_address(dvpp_desc_t *b) { return 0; };

dvpp_platform_ops_t stub_dvpp_platform_ops = 	{
	.register_ops = stub_register_ops,
	.port_state = stub_port_state,
	.pipe_state = stub_pipe_state,
	.port_free_mini = stub_port_free_mini,
	.port_alloc_mini = stub_port_alloc_mini,
	.get_desc_kernel_address = stub_get_desc_kernel_address,
};

dvpp_platform_ops_t dvpp_platform_ops;

dvpp_platform_ops_t *dvpp_p_ops = &stub_dvpp_platform_ops;

static int dvpp_ring_alloc_buf_edma(struct wil6210_priv *wil,
				    struct wil_ring *ring, u32 i)
{
	struct device *dev = wil_to_dev(wil);
	unsigned int sz = wil->rx_buf_len;
	dma_addr_t pa;
	u16 buff_id;
	int ret;
	struct list_head *active = &wil->rx_buff_mgmt.active;
	struct list_head *free = &wil->rx_buff_mgmt.free;
	struct wil_rx_buff *rx_buff;
	struct wil_rx_buff *buff_arr = wil->rx_buff_mgmt.buff_arr;
	dvpp_desc_t _mini, *mini = &_mini;
	struct wil_rx_enhanced_desc dd, *d = &dd;
	struct wil_rx_enhanced_desc *_d =
		(struct wil_rx_enhanced_desc *)&ring->va[i].rx.enhanced;

	if (unlikely(list_empty(free))) {
		wil->rx_buff_mgmt.free_list_empty_cnt++;
		return -EAGAIN;
	}

	ret = dvpp_p_ops->port_alloc_mini(wil->dvpp_status.port_id, mini);
	if (unlikely(ret == 0)) {
		ktime_t now = ktime_get();
		if ((now - _last_print) > NSEC_PER_SEC) {
			wil_err(wil, "Alloc minibuf no buffers \n");
			_last_print = now;
		}
		return -ENOMEM;
	}

	if (unlikely(mini->data == 0)) {
		ktime_t now = ktime_get();
		if ((now - _last_print) > NSEC_PER_SEC) {
			wil_err(wil, "Alloc minibuf w/o data seg=%llx\n", mini->seg.desc);
			_last_print = now;
		}
	}

	pa = dma_map_single(dev, (void*)(mini->data + mini->seg.offset),
			wil->rx_buf_len, DMA_FROM_DEVICE);

	if (unlikely(dma_mapping_error(dev, pa))) {
		ktime_t now = ktime_get();
		if ((now - _last_print) > NSEC_PER_SEC) {
			wil_err(wil, "dma_mapping_error pa %p\n", (void*)pa);
			_last_print = now;
		}
		return -ENOMEM;
	}

	/* Get the buffer ID - the index of the rx buffer in the buff_arr */
	rx_buff = list_first_entry(free, struct wil_rx_buff, list);
	buff_id = rx_buff->id;

	/* Move a buffer from the free list to the active list */
	list_move(&rx_buff->list, active);

	buff_arr[buff_id].mini = *mini;

	wil_desc_set_addr_edma(&d->dma.addr, &d->dma.addr_high_high, pa);
	d->dma.length = cpu_to_le16(sz);
	d->mac.buff_id = cpu_to_le16(buff_id);

	*_d = *d;

	/* Save the physical address for later use in dma_unmap */
	buff_arr[buff_id].pa = pa;

	return 0;
}

/*
 * Call-back for when the User-Land will go away, hence the network
 * buffer memory used for Rx buffers goes away.
 *
 * Once dvpp_cancel_edma() is called, the Wigig peers must be disassociated
 * and reassociated so as to recover the Data-Path.
 *
 * TODO: implement proper "uncancel" or "unpause" DMA function.
 */
char fake_buf[2048];
int dvpp_cancel_edma(void *p)
{
	struct wil_ring *ring;
	int rc = 0, i;
	struct wil6210_priv *wil;
	struct wil_rx_enhanced_desc d;
	u64 pa = virt_to_phys(fake_buf);

	if (p == NULL)
		return rc;

	wil = p;

	ring = &wil->ring_rx;
	if (!ring)
		return rc;

	wil_info(wil,"will cancel ring port %u swtail %u swhead %u size %u\n",
		wil->dvpp_status.port_id, ring->swtail, ring->swhead, ring->size);

	for (i=0; i < ring->size; i++) {
		volatile struct wil_rx_enhanced_desc *_d =
			(struct wil_rx_enhanced_desc *)&ring->va[i].rx.enhanced;
		volatile u64 *p = (volatile u64 *)&ring->va[i].rx.enhanced.dma;
		volatile u64 *pp = (volatile u64 *)&d.dma;
		d = *_d;
		wil_desc_set_addr_edma(&d.dma.addr, &d.dma.addr_high_high, pa);
		wmb();
		*p++ = *pp++;
		*p = *pp;
		wmb();
	}

	return rc;
}

/*
 * Supplies the Rx ring with available network buffers.
 */
int dvpp_rx_refill_edma(struct wil6210_priv *wil)
{
	struct wil_ring *ring = &wil->ring_rx;
	u32 next_head;
	int rc = 0;
	int cnt = 0;
	ring->swtail = *ring->edma_rx_swtail.va;

	if (unlikely(wil->dvpp_status.enabled == 0))
		return 0;

	for (;
	     next_head = wil_ring_next_head(ring), (next_head != ring->swtail);
	     ring->swhead = next_head) {
		rc = dvpp_ring_alloc_buf_edma(wil, ring, ring->swhead);
		if (unlikely(rc)) {
			wil->refill_fail++;
			if (rc == -EAGAIN) {
				wil_dbg_txrx(wil, "DVPP No free buffer ID found\n");
			}
			break;
		} else {
			cnt++;
		}
	}

	if (cnt) {
		/* make sure all writes to descriptors (shared memory) are done before
		* committing them to HW
		 */
		wmb();

		wil_w(wil, ring->hwtail, ring->swhead);
	}
	return rc;
}

/*
 * Main Rx Loop.
 */
static u64*
dvpp_sring_reap_rx_edma(struct wil6210_priv *wil, struct wil_status_ring *sring,
			dvpp_desc_t *mini)
{
	struct device *dev = wil_to_dev(wil);
	struct wil_rx_status_extended msg1;
	void *msg = &msg1;
	u16 buff_id;
	dvpp_desc_t _mini = {};
	struct wil_ring_rx_data *rxdata = &sring->rx_data;
	unsigned int sz = wil->rx_buf_len;
	struct wil_net_stats *stats = NULL;
	u16 dmalen;
	int cid;
	bool eop;
	u8 dr_bit;
	u8 data_offset;
	u16 sring_idx = sring - wil->srings;

	if (unlikely(wil->dvpp_status.enabled == 0))
		return NULL;

again:
	wil_get_next_rx_status_msg(sring, &dr_bit, msg);

	/* Completed handling all the ready status messages */
	if (dr_bit != sring->desc_rdy_pol)
		return NULL;

	/* Extract the buffer ID from the status message */
	buff_id = le16_to_cpu(wil_rx_status_get_buff_id(msg));

	while (!buff_id) {
		struct wil_rx_status_extended *s;
		int invalid_buff_id_retry = 0;

		wil_dbg_txrx(
			wil,
			"buff_id is not updated yet by HW, (swhead 0x%x)\n",
			sring->swhead);
		if (++invalid_buff_id_retry > MAX_INVALID_BUFF_ID_RETRY)
			break;

		/* Read the status message again */
		s = (struct wil_rx_status_extended *)(sring->va +
						      (sring->elem_size *
						       sring->swhead));
		*(struct wil_rx_status_extended *)msg = *s;
		buff_id = le16_to_cpu(wil_rx_status_get_buff_id(msg));
	}

	if (unlikely(!wil_val_in_range(buff_id, 1, wil->rx_buff_mgmt.size))) {
		ktime_t now = ktime_get();
		if ((now - _last_print) > NSEC_PER_SEC) {
			wil_err(wil, "Corrupt buff_id=%d, sring->swhead=%d seg=%llx\n", buff_id,
				sring->swhead, _mini.seg.desc);
				_last_print = now;
		}
		wil_rx_status_reset_buff_id(sring);
		wil_sring_advance_swhead(sring);
		sring->invalid_buff_id_cnt++;
		goto again;
	}

	/* Extract the mini buf from the rx_buff management array */
	_mini = wil->rx_buff_mgmt.buff_arr[buff_id].mini;
	dvpp_desc_clear(&wil->rx_buff_mgmt.buff_arr[buff_id].mini);

	if (!_mini.data) {
		ktime_t now = ktime_get();
		if ((now - _last_print) > NSEC_PER_SEC) {
			wil_err(wil, "port %u No Rx buf at buff_id %d seg=%llx @ %p\n",
				wil->dvpp_status.port_id, buff_id,  _mini.seg.desc,
				&wil->rx_buff_mgmt.buff_arr[buff_id]);
			_last_print = now;
		}
		wil_rx_status_reset_buff_id(sring);
		/* Move the buffer from the active list to the free list */
		list_move_tail(&wil->rx_buff_mgmt.buff_arr[buff_id].list,
			       &wil->rx_buff_mgmt.free);
		wil_sring_advance_swhead(sring);
		sring->invalid_buff_id_cnt++;
		goto again;
	}

	wil_rx_status_reset_buff_id(sring);
	wil_sring_advance_swhead(sring);
	dma_unmap_single(dev, wil->rx_buff_mgmt.buff_arr[buff_id].pa + \
		_mini.seg.offset, sz, DMA_FROM_DEVICE);

	dmalen = le16_to_cpu(wil_rx_status_get_length(msg));

	wil_dbg_txrx(wil, "Rx, buff_id=%u, sring_idx=%u, dmalen=%u bytes\n",
		     buff_id, sring_idx, dmalen);

	/* Move the buffer from the active list to the free list */
	list_move_tail(&wil->rx_buff_mgmt.buff_arr[buff_id].list,
		       &wil->rx_buff_mgmt.free);

	eop = wil_rx_status_get_eop(msg);

	cid = wil_rx_status_get_cid(msg);

	if (unlikely(!wil_val_in_range(cid, 0, max_assoc_sta))) {
		wil_err(wil, "Corrupt cid=%d, sring->swhead=%d\n", cid,
			sring->swhead);
		rxdata->skipping = true;
		goto skipping;
	}
	stats = &wil->sta[cid].stats;

	if (unlikely(dmalen < ETH_HLEN)) {
		wil_dbg_txrx(wil, "Short frame, len = %d\n", dmalen);
		stats->rx_short_frame++;
		rxdata->skipping = true;
		goto skipping;
	}

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
		dvpp_p_ops->port_free_mini(&_mini, wil->dvpp_status.port_id);
		if (rxdata->mini.data) {
			dvpp_p_ops->port_free_mini(&rxdata->mini, wil->dvpp_status.port_id);
			dvpp_desc_clear(&rxdata->mini);
		}
		rxdata->skipping = !eop;
		goto again;
	}

	_mini.seg.len = dmalen;

	if (likely(!rxdata->mini.data)) {
		rxdata->mini = _mini;
	} else {
		/* TODO report multisegments. */
		wil_err(wil, "Error: DVPP RX multisegment packet\n");
		wil->dvpp_status.enabled = 0;
		wil->dvpp_status.error = DVPP_ERROR_MULTISEG_RX;
		return NULL;
	}

	if (unlikely(!eop))
		goto again;

	/* reaching here rxdata->skb always contains a full packet */
	*mini = rxdata->mini;
	dvpp_desc_clear(&rxdata->mini);
	rxdata->skipping = false;

	if (stats) {
		stats->last_mcs_rx = wil_rx_status_get_mcs(msg);
		if (stats->last_mcs_rx < ARRAY_SIZE(stats->rx_per_mcs))
			stats->rx_per_mcs[stats->last_mcs_rx]++;
	}

	/* Compensate for the HW data alignment according to the status
	 * message
	 */
	data_offset = wil_rx_status_get_data_offset(msg);
	if (data_offset == 0xFF || data_offset > WIL_EDMA_MAX_DATA_OFFSET) {
		wil_err(wil, "Unexpected data offset %d\n", data_offset);
		dvpp_p_ops->port_free_mini(mini, wil->dvpp_status.port_id);
		goto again;
	}
	mini->seg.offset += data_offset;

	mini->pipe_id = 0; /* TODO set peer ID */

	return (u64*)(mini->data);
}

int dvpp_rx_handle_edma(void *p, dvpp_desc_t *b, u32 n_pkts,
			u32 verbose)
{
	struct wil6210_priv *wil = p;
	int i;
	struct wil_status_ring *sring;
	int cnt = 0;
	struct wil_ring *ring = &wil->ring_rx;
	u64 * data;
	dvpp_desc_t *mini = b;

	if (unlikely(!ring->va)) {
		wil_err(wil, "Rx IRQ while Rx not yet initialized\n");
		return 0;
	}
	if (verbose)
		wil_info(wil, "%s: rings %u b %p\n", __FUNCTION__,
		       wil->num_rx_status_rings, b);
	for (i = 0; i < wil->num_rx_status_rings; i++) {
		sring = &wil->srings[i];
		if (unlikely(!sring->va)) {
			wil_err(wil,
				"Rx IRQ while Rx status ring %d not yet initialized\n",
				i);
			continue;
		}
		while ((cnt < n_pkts) &&
		       (NULL != (data = dvpp_sring_reap_rx_edma(wil, sring, mini++)))) {
			cnt++;
		}
		wil_w(wil, sring->hwtail, (sring->swhead - 1) % sring->size);
	}

	dvpp_rx_refill_edma(wil);

	return cnt;
}

/**
 * Clean up transmitted descriptors from the Tx descriptor RING.
 * Return number of descriptors cleared.
 */
int dvpp_tx_sring_handler(struct wil6210_priv *wil,
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

	wil_get_next_tx_status_msg(sring, &dr_bit, &msg);

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

			_d = (struct wil_tx_enhanced_desc *)
				&ring->va[ring->swtail].tx.enhanced;
			*d = *_d;

			dmalen = le16_to_cpu(d->dma.length);
			trace_wil6210_tx_status(&msg, ring->swtail, dmalen);
			wil_dbg_txrx(wil,
				     "TxC[%2d][%3d] : %d bytes, status 0x%02x\n",
				     ring_id, ring->swtail, dmalen,
				     msg.status);
			wil_hex_dump_txrx("TxS ", DUMP_PREFIX_NONE, 32, 4,
					  (const void *)&msg, sizeof(msg),
					  false);

			if (WIL_CTX_FLAGS(ctx) & WIL_CTX_FLAG_RESERVED_USED)
				txdata->tx_reserved_count++;

			wil_tx_desc_unmap_edma(dev,
					       (union wil_tx_desc *)d,
					       ctx);

			if (likely(msg.status == 0)) {
				ndev->stats.tx_packets++;
				ndev->stats.tx_bytes += ctx->desc.seg.len;
				if (stats) {
					stats->tx_packets++;
					stats->tx_bytes += ctx->desc.seg.len;
				}
			} else {
				ndev->stats.tx_errors++;
				if (stats)
					stats->tx_errors++;
			}
			if (stats) {
				atomic_dec(&stats->tx_pend_packets);
				atomic_sub(ctx->desc.seg.len, &stats->tx_pend_bytes);
			}
			dvpp_p_ops->port_free_mini(&ctx->desc, wil->dvpp_status.port_id);
			dvpp_desc_clear(&ctx->desc);

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

		wil_get_next_tx_status_msg(sring, &dr_bit, &msg);
	}

	if (num_statuses % WIL_EDMA_TX_SRING_UPDATE_HW_TAIL != 0)
		/* Update the HW tail ptr (RD ptr) */
		wil_w(wil, sring->hwtail, (sring->swhead - 1) % sring->size);

	return desc_cnt;
}

int dvpp_tx_complete(void *p)
{
	struct wil_status_ring *sring;
	struct wil6210_priv *wil = p;

	sring = &wil->srings[wil->tx_sring_idx];
	if (sring->va) {
		dvpp_tx_sring_handler(wil, sring);
	}
	return 0;
}

int dvpp_tx_batch(void *p, u32 pipe, dvpp_desc_t *bufs, u32 n_pkts,
		  u32 verbose)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)p;
	/* TODO proper flow to ring mapping, i.e. find the sta from the pipe */
	u32 ring_index =
		dvpp_pipe_to_ring_id(pipe);
	struct wil_ring *ring = &wil->ring_tx[ring_index];
	struct device *dev = wil_to_dev(wil);
	struct wil_ring_tx_data *txdata = &wil->ring_tx_data[ring_index];
	u8 cid = txdata->cid;
	struct wil_net_stats *stats =
		(cid < WIL6210_MAX_CID) ? &wil->sta[cid].stats : NULL;
	uint len;
	dma_addr_t pa;
	dvpp_desc_t *b;
	int nr_frags;
	bool mcast = false;
	struct vring_tx_desc dd, *d = &dd;
	volatile struct vring_tx_desc *_d;
	int num_sent = 0;
	int num_bytes_sent = 0; /* Statistics */
	int num_seg = 0;
	u32 swhead;
	int avail;

	spin_lock(&txdata->lock);
	if (test_bit(wil_status_suspending, wil->status) ||
	    test_bit(wil_status_suspended, wil->status) ||
	    test_bit(wil_status_resuming, wil->status)) {
		wil_dbg_txrx(wil, "suspend/resume in progress. drop packet\n");
		spin_unlock(&txdata->lock);
		return -EINVAL;
	}

	if (unlikely(!txdata->enabled)) {
		spin_unlock(&txdata->lock);
		return -EINVAL;
	}
	swhead = ring->swhead;

	/* Count number of buffers we can transmit */
	avail = wil_ring_avail_tx(ring);

	while (num_sent < n_pkts) {
		void *data;
		//b = bufs[num_sent];
		b = bufs++;
		data = dvpp_p_ops->get_desc_kernel_address(b);
		if (unlikely(data == 0)) {
			break;
		}
		// no account for multi frags yet
		nr_frags = 0;
		num_seg += 1 + nr_frags;
		if (num_seg > avail) {
			break;
		}

		len = b->seg.len;
		_d = &ring->va[swhead].tx.legacy;
		data += b->seg.offset;
		pa = dma_map_single(dev, data, len, DMA_TO_DEVICE);

		if (unlikely(dma_mapping_error(dev, pa))) {
			// We need to clean up what we sent
			goto dma_error;
		}

		ring->ctx[swhead].desc = *b;
		ring->ctx[swhead].desc.seg.mflags = wil_mapped_as_single;
		ring->ctx[swhead].desc.seg.flags = 0; //ctx_flags;
		/* 1-st segment */
		wil->txrx_ops.tx_desc_map((union wil_tx_desc *)d, pa, len,
					  ring_index);
		if (unlikely(mcast)) {
			d->mac.d[0] |=
				BIT(MAC_CFG_DESC_TX_0_MCS_EN_POS); /* MCS 0 */
			if (unlikely(len >
				     WIL_BCAST_MCS0_LIMIT)) /* set MCS 1 */
				d->mac.d[0] |=
					(1 << MAC_CFG_DESC_TX_0_MCS_INDEX_POS);
		}

		ring->ctx[swhead].desc.seg.flags = nr_frags;
		wil_tx_desc_set_nr_frags(d, nr_frags + 1);

		/* skip middle seggments */

		/* for the last seg only */
		d->dma.d0 |= BIT(DMA_CFG_DESC_TX_0_CMD_EOP_POS);
		d->dma.d0 |= BIT(DMA_CFG_DESC_TX_0_CMD_MARK_WB_POS);
		d->dma.d0 |= BIT(DMA_CFG_DESC_TX_0_CMD_DMA_IT_POS);
		*_d = *d;

		/* Maintains statistics */
		num_sent++;
		num_bytes_sent += len;

		/* advance swhead */
		swhead = (swhead + nr_frags + 1) % ring->size;
	}

	if (num_sent) {
		/* make sure all writes to descriptors (shared memory) are done before
		 * committing them to HW
		 */
		wmb();
		/* Update ring head */
		ring->swhead = swhead;

		/* Kick off DMA */
		wil_w(wil, ring->hwtail, ring->swhead);
		if (stats) {
			atomic_add(num_sent, &stats->tx_pend_packets);
			atomic_add(num_bytes_sent, &stats->tx_pend_bytes);
		}
	}

	spin_unlock(&txdata->lock);

	return num_sent;
dma_error:
	wil_err(wil, "%s: dma error\n", __FUNCTION__);
	return -EINVAL;
}

int dvpp_tx_avail(void *p, u32 *credit, u32 n_pipe)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)p;
	int i;
	struct wil_ring *ring;
	// Lock is not really needed here
	// TODO: make reading ring head and tail an atomic 64 bits read
	for (i = 0; i < n_pipe; i++) {
		ring = &wil->ring_tx[dvpp_pipe_to_ring_id(i)];
		if (ring->va)
			*credit++ = wil_ring_avail_tx(ring);
		else
			*credit++ = 0;
	}

	return i;
}

const struct platform_device_id dvpp_id_table[] = {
    {"direct-vpp", 0},
    {},
};

static int wil_dvpp_probe(struct platform_device *pdev)
{
	dvpp_platform_ops_t * ops = dev_get_platdata(&pdev->dev);
	if (ops == NULL) {
		printk(KERN_ERR "%s: error: DVPP and no ops!\n", __FUNCTION__);
		return -ENODEV;
	}

	/* Init ops */
	dvpp_platform_ops = *ops;
	dvpp_p_ops = &dvpp_platform_ops;
	printk(KERN_INFO "%s: will pdev %p register DVPP ops %p ops [%p %p]\n",
		__FUNCTION__,
		pdev, dvpp_p_ops->register_ops, ops->register_ops, ops->port_state);

	dvpp_p_ops->register_ops(&dvpp_ops);

	printk(KERN_INFO "%s: successfully registered DVPP\n", __FUNCTION__);
	dvpp_inited = 1;
	return 0;
}

static struct platform_driver dvpp_driver = {
	/* TODO: implement shutdown and remove. */
    .probe = wil_dvpp_probe,
    .id_table = dvpp_id_table,
    .driver =
	{
	    .name = "wil6210-direct-vpp",
	},
};

int wil_dvpp_init(void) {
	/* Attach to DVPP */
	int ret = platform_driver_register(&dvpp_driver);
	printk(KERN_INFO "%s: register dvpp driver\n", __FUNCTION__);
	if (ret != 0) {
		/* Continue without a DVPP interface. */
		printk(KERN_INFO "%s: didnt find DVPP, skip...\n", __FUNCTION__);
		goto done;
	}

done:
	return 0;
}


void wil_dvpp_clean(void) {
	/* Tell DVPP that we're going away */
	dvpp_p_ops->register_ops(NULL);
	/* Lose the DVPP ops */
	dvpp_p_ops = &stub_dvpp_platform_ops;
	/* Attach to DVPP */
	platform_driver_unregister(&dvpp_driver);
	printk(KERN_INFO "%s: unregister dvpp driver\n", __FUNCTION__);
}
