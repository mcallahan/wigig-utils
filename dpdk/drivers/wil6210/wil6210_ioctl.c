/*
 * Copyright (c) 2020, Facebook, Inc. All rights reserved.
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

#define TG_SB_QUEUE_STATS 115

/* TG_SB_QUEUE_STATS */
struct tg_sb_queue_stats {
	uint32_t bytes_pending;
	uint32_t arrival_rate; // unit: bytes per millisecond
	uint8_t dst_mac_addr[6];
} __attribute__((__packed__));

/*
 * FB Driver - Firmware message type
 * This messages could be command to sent from Driver to Fw
 * Or Fw to Driver events
 */
struct fb_tg_if_event {
	uint8_t type;
	uint8_t pad[3]; // Ensure 4 byte alignment for data payloads
	union {
		/* include other ioctl types here */
		struct tg_sb_queue_stats queue_stats[WIL6210_MAX_TX_RINGS];
	} __attribute__((__packed__)) /* implicit */ data;
} __attribute__((__packed__));

static int
wil_try_ioctl_async(struct wil6210_priv *wil, u8 *req_buf, u16 req_len,
		    u8 *resp_buf, u16 *resp_len)
{
	struct wil6210_vif *vif;
	struct wmi_internal_fw_ioctl_cmd *cmd;
	struct {
		struct wmi_cmd_hdr wmi;
		struct wmi_internal_fw_ioctl_event evt;
	} __packed *reply;
	u16 cmd_len, reply_len, evt_len;
	int rc;

	vif = ndev_to_vif(wil->main_ndev);

	if (req_len > WMI_MAX_IOCTL_PAYLOAD_SIZE) {
		wil_err(wil, "request too large (%d, max %d)\n", req_len,
			WMI_MAX_IOCTL_PAYLOAD_SIZE);
		return -EINVAL;
	}

	cmd_len = sizeof(*cmd) + req_len;
	cmd = rte_malloc("wil6210", cmd_len, 0);
	if (!cmd)
		return -ENOMEM;
	reply_len = sizeof(*reply) + WMI_MAX_IOCTL_PAYLOAD_SIZE;
	reply = rte_malloc("wil6210", reply_len, 0);
	if (!reply) {
		rc = -ENOMEM;
		goto out_cmd;
	}
	cmd->code = 0;
	cmd->length = cpu_to_le16(req_len);
	memcpy(cmd->payload, req_buf, req_len);
	memset(reply, 0, sizeof(*reply));
	/* make default reply failure; will be overwritten if wmi call succeeds */
	reply->evt.status = WMI_FW_STATUS_FAILURE;

	wil->ioctl_resp_buf = resp_buf;
	wil->ioctl_resp_len = resp_len;

	rc = wmi_call_async(wil, WMI_INTERNAL_FW_IOCTL_CMDID, vif->mid, cmd,
			    cmd_len, WMI_INTERNAL_FW_IOCTL_EVENTID, reply,
			    reply_len, WIL6210_FW_RECOVERY_TO);

out_cmd:
	rte_free(cmd);
	return rc;
}

int
wil_send_queue_stats_ioctl_async(struct wil6210_priv *wil, u32 *bytes_pending,
				 u32 *arrival_rate, struct wil_sta_info **stas)
{
	u8 ioctl_req_buff[sizeof(struct fb_tg_if_event)];
	struct tg_sb_queue_stats queue_stats[WIL6210_MAX_TX_RINGS] = { { 0 } };
	int i, num_links, rc;
	struct fb_tg_if_event *ioctl;
	size_t stats_len, ioctl_len;
	struct wil_sta_info *sta;
	u32 tot_bytes_pending;
	char macstr[18];

	for (i = 0, num_links = 0; i < WIL6210_MAX_TX_RINGS; i++) {
		sta = stas[i];
		if (sta == NULL)
			continue;

		/* add bytes pending in the pmd to bytes pending from caller source */
		tot_bytes_pending = bytes_pending[i] +
				    atomic_read(&sta->stats.tx_pend_bytes);

		queue_stats[num_links].bytes_pending = tot_bytes_pending;
		queue_stats[num_links].arrival_rate = arrival_rate[i];
		memcpy(queue_stats[num_links].dst_mac_addr, sta->addr,
			sizeof(queue_stats[num_links].dst_mac_addr));

		wil_dbg_qstats(wil,
			"Queue stats message index %u, bytes_pending: %u, "
			"arrival_rate: %u, dst_mac_addr: %s\n",
			num_links,
			queue_stats[num_links].bytes_pending,
			queue_stats[num_links].arrival_rate,
			mac_to_str(queue_stats[num_links].dst_mac_addr, macstr));

		num_links++;
	}

	if (num_links == 0)
		return -ENODEV;

	ioctl = (struct fb_tg_if_event *)ioctl_req_buff;
	ioctl->type = TG_SB_QUEUE_STATS;

	stats_len = num_links * sizeof(struct tg_sb_queue_stats);
	memcpy(ioctl->data.queue_stats, queue_stats, stats_len);

	ioctl_len = offsetof(struct fb_tg_if_event, data) + stats_len;

	rc = wil_try_ioctl_async(wil, ioctl_req_buff, ioctl_len, NULL, NULL);
	if (rc)
		wil_dbg_qstats(wil, "async ioctl send failure rc=%u\n", rc);
	return rc;
}
