/*
 * Copyright (c) 2014-2017 Qualcomm Atheros, Inc.
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019-2020, Facebook, Inc. All rights reserved.
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

#include "wil6210_ethdev.h"
#include "txrx.h"

/* called in NAPI context */
void wil_rx_reorder(struct wil6210_priv *wil, struct rte_mbuf *skb)
__acquires(&sta->tid_rx_lock) __releases(&sta->tid_rx_lock)
{
	ASSERT(0);
}

/* process BAR frame, called in NAPI context */
void wil_rx_bar(struct wil6210_priv *wil, struct wil6210_vif *vif,
		u8 cid, u8 tid, u16 seq)
{
	ASSERT(0);
}

/* ADDBA processing */
static u16 wil_agg_size(struct wil6210_priv *wil, u16 req_agg_wsize)
{
	u16 max_agg_size = (slave_mode == 2) ? wil->max_agg_wsize :
		min_t(u16, wil->max_agg_wsize, wil->max_ampdu_size /
				 (wil->mtu_max + WIL_MAX_MPDU_OVERHEAD));

	if (!req_agg_wsize)
		return max_agg_size;

	return min(max_agg_size, req_agg_wsize);
}

/* Block Ack - Rx side (recipient) */
int wil_addba_rx_request(struct wil6210_priv *wil, u8 mid, u8 cid, u8 tid,
			 u8 dialog_token, __le16 ba_param_set,
			 __le16 ba_timeout, __le16 ba_seq_ctrl)
__acquires(&sta->tid_rx_lock) __releases(&sta->tid_rx_lock)
{
	u16 param_set = le16_to_cpu(ba_param_set);
	u16 agg_timeout = le16_to_cpu(ba_timeout);
	u16 seq_ctrl = le16_to_cpu(ba_seq_ctrl);
	struct wil_sta_info *sta;
	u16 agg_wsize = 0;
	/* bit 0: A-MSDU supported
	 * bit 1: policy (should be 0 for us)
	 * bits 2..5: TID
	 * bits 6..15: buffer size
	 */
	u16 req_agg_wsize = WIL_GET_BITS(param_set, 6, 15);
	bool agg_amsdu = wil->use_enhanced_dma_hw &&
		wil->use_rx_hw_reordering &&
		test_bit(WMI_FW_CAPABILITY_AMSDU, wil->fw_capabilities) &&
		wil->amsdu_en && (param_set & BIT(0));
	int ba_policy = param_set & BIT(1);
	u16 status = WLAN_STATUS_SUCCESS;
	u16 ssn = seq_ctrl >> 4;
	struct wil_tid_ampdu_rx *r __rte_unused;
	int rc = 0;

	/* sanity checks */
	if (cid >= max_assoc_sta) {
		wil_err(wil, "BACK: invalid CID %d\n", cid);
		rc = -EINVAL;
		goto out;
	}

	sta = &wil->sta[cid];
	if (sta->status != wil_sta_connected) {
		wil_err(wil, "BACK: CID %d not connected\n", cid);
		rc = -EINVAL;
		goto out;
	}

	wil_dbg_wmi(wil,
		    "ADDBA request for CID %d %pM TID %d size %d timeout %d AMSDU%s policy %d token %d SSN 0x%03x\n",
		    cid, sta->addr, tid, req_agg_wsize, agg_timeout,
		    agg_amsdu ? "+" : "-", !!ba_policy, dialog_token, ssn);

	/* apply policies */
	if (ba_policy) {
		wil_err(wil, "BACK requested unsupported ba_policy == 1\n");
		status = WLAN_STATUS_INVALID_QOS_PARAM;
	}
	if (status == WLAN_STATUS_SUCCESS) {
		if (req_agg_wsize == 0) {
			wil_dbg_misc(wil, "Suggest BACK wsize %d\n",
				     wil->max_agg_wsize);
			agg_wsize = wil->max_agg_wsize;
		} else {
			agg_wsize = min_t(u16,
					  wil->max_agg_wsize, req_agg_wsize);
		}
	}

	rc = wil->txrx_ops.wmi_addba_rx_resp(wil, mid, cid, tid, dialog_token,
					     status, agg_amsdu, agg_wsize,
					     agg_timeout);
	if (rc || (status != WLAN_STATUS_SUCCESS)) {
		wil_err(wil, "do not apply ba, rc(%d), status(%d)\n", rc,
			status);
		goto out;
	}

#ifndef WIL6210_PMD
	/* apply */
	r = wil_tid_ampdu_rx_alloc(wil, agg_wsize, ssn);
	spin_lock_bh(&sta->tid_rx_lock);
	wil_tid_ampdu_rx_free(wil, sta->tid_rx[tid]);
	sta->tid_rx[tid] = r;
	spin_unlock_bh(&sta->tid_rx_lock);
#endif

out:
	return rc;
}

/* BACK - Tx side (originator) */
int wil_addba_tx_request(struct wil6210_priv *wil, u8 ringid, u16 wsize)
{
	u8 agg_wsize = wil_agg_size(wil, wsize);
	u16 agg_timeout = 0;
	struct wil_ring_tx_data *txdata = &wil->ring_tx_data[ringid];
	int rc = 0;

	if (txdata->addba_in_progress) {
		wil_dbg_misc(wil, "ADDBA for vring[%d] already in progress\n",
			     ringid);
		goto out;
	}
	if (txdata->agg_wsize) {
		wil_dbg_misc(wil,
			     "ADDBA for vring[%d] already done for wsize %d\n",
			     ringid, txdata->agg_wsize);
		goto out;
	}
	txdata->addba_in_progress = true;
	rc = wmi_addba(wil, txdata->mid, ringid, agg_wsize, agg_timeout);
	if (rc) {
		wil_err(wil, "wmi_addba failed, rc (%d)", rc);
		txdata->addba_in_progress = false;
	}

out:
	return rc;
}
