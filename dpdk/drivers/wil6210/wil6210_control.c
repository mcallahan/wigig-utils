/*
 * Copyright (c) 2013,2016 Qualcomm Atheros, Inc.
 * Copyright (c) 2018,2020 The Linux Foundation. All rights reserved.
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
#include "wil6210_nl60g.h"
#include "slave.h"
#include "slave_i.h"
#include "dpdk-dhd-ctrl.h"
#include "dpdk_dhd.h"

#include <poll.h>

static int wil_sync_handler(void *ctx, struct dhd_call_desc *req);

struct platform_device {
	struct dhd_state *dhd;
	struct wil6210_priv *wil;
	struct wil_slave_ops slave_ops;
	void *slave_ctx;
};

static struct dhd_state *dhd_list[DHD_MAX_PORTS];
struct dhd_link_info dhd_link_map[DHD_TOTAL_LINKS];

struct platform_device *
platform_device_alloc(const char *name, int unit)
{
	struct platform_device *pdev;
	int i;

	pdev = kzalloc(sizeof(*pdev), GFP_KERNEL);
	if (pdev == NULL)
		return NULL;

	pdev->dhd = dhd_init();
	if (pdev->dhd == NULL) {
		kfree(pdev);
		return NULL;
	}

	for (i = 0; i < ARRAY_SIZE(dhd_list); i++)
		if (dhd_list[i] == NULL)
			break;
	ASSERT(i < DHD_MAX_PORTS);
	dhd_list[i] = pdev->dhd;

	return pdev;
}

int
platform_device_add_data(struct platform_device *pdev, void *pdata,
    size_t datalen)
{
	struct wil_slave_platdata *slave_data = pdata;

	pdev->wil = wil_slave_priv(slave_data->dev_ctx);
	pdev->slave_ctx = slave_data->dev_ctx;
	pdev->slave_ops = *slave_data->ops;
	return 0;
}

void
platform_device_put(struct platform_device *pdev)
{
	int i;

	if (pdev->dhd != NULL) {
		for (i = 0; i <ARRAY_SIZE(dhd_list); i++)
			if (dhd_list[i] == pdev->dhd)
				dhd_list[i] = NULL;
		dhd_fini(pdev->dhd);
	}
	kfree(pdev);
}

int
platform_device_add(struct platform_device *pdev)
{
	struct wil6210_priv *wil;
	int rc;

	ASSERT(pdev != NULL);
	ASSERT(pdev->dhd != NULL);
	ASSERT(pdev->wil != NULL);

	wil = pdev->wil;

	rc = dhd_attach(pdev->dhd, wil->main_ndev->dev_addr,
		wil->pdev->addr /* pci_dev */);
	if (rc != 0)
		return -rc;

	pdev->dhd->dhd_handler = wil_sync_handler;
	pdev->dhd->dhd_handler_arg = pdev;
	wil->dhd = pdev->dhd;

	rc  = dhd_start(pdev->dhd);
	if (rc != 0)
		return -rc;
	return 0;
}

void
platform_device_unregister(struct platform_device *pdev)
{
	struct wil6210_priv *wil = pdev->wil;

	/* Stop the device */
	wil_info(wil, "Disconnecting from in-kernel interface %s\n",
	    pdev->dhd->dhd_ifname);
	dhd_stop(pdev->dhd);
	wil->dhd = NULL;
	/* Free the memory */
	platform_device_put(pdev);
}

static void
wil_master_rx_event(void *ctx, u16 id, u8 *evt, u32 len)
{
	struct platform_device *pdev = ctx;

	if (id != 0)
		return;
	dhd_send_event(pdev->dhd, evt, len);
}

static void
wil_master_connected(void *ctx, int tx_link_id, int rx_link_id,
		     const u8 *mac, u8 cid)
{
	struct platform_device *pdev = ctx;
	struct wil6210_priv *wil;
	char mac_str[18];

	ASSERT(pdev != NULL);
	ASSERT(pdev->dhd != NULL);
	ASSERT(pdev->wil != NULL);

	wil = pdev->wil;

	/* Remember links in sta structure */
	wil->sta[cid].rx_link = rx_link_id;
	wil->sta[cid].tx_link = tx_link_id;

	wil_info(wil, "Connected link: TX %d RX %d CID %u MAC %s\n",
	    tx_link_id, rx_link_id, cid, mac_to_str(mac, mac_str));
}

static void
wil_master_disconnected(void *ctx, u8 cid)
{
	struct platform_device *pdev = ctx;
	struct wil6210_priv *wil;

	ASSERT(pdev != NULL);
	ASSERT(pdev->dhd != NULL);
	ASSERT(pdev->wil != NULL);

	wil = pdev->wil;

	wil->sta[cid].rx_link = -1;
	wil->sta[cid].tx_link = -1;

	wil_info(wil, "Disconnected CID %u\n", cid);
}

static int
wil_master_rx_data(void *ctx, u8 cid, struct rte_mbuf *skb)
{
	return -ENOSPC; /* Something ridiculous */
}

static void
wil_master_flow_control(void *ctx, u8 cid, bool stop_tx)
{
}

static void
wil_master_set_channel(void *ctx, u8 channel)
{
	struct platform_device *pdev = ctx;
	struct wil6210_priv *wil;

	ASSERT(pdev != NULL);
	ASSERT(pdev->dhd != NULL);
	ASSERT(pdev->wil != NULL);

	wil = pdev->wil;
	wil_dbg_wmi(wil, "Set channel: %u\n", channel);
}

static void
wil_master_slave_going_down(void *ctx)
{
	struct platform_device *pdev = ctx;

	ASSERT(pdev != NULL);
	ASSERT(pdev->dhd != NULL);
	ASSERT(pdev->wil != NULL);

	wil_info(pdev->wil, "Going down: %s\n", pdev->dhd->dhd_ifname);
	pdev->slave_ops.unregister_master(pdev->slave_ctx);
}

static const struct wil_slave_rops slave_rops = {
	.api_version = WIL_SLAVE_API_VERSION,
	.rx_event = wil_master_rx_event,
	.connected = wil_master_connected,
	.disconnected = wil_master_disconnected,
	.rx_data = wil_master_rx_data,
	.flow_control = wil_master_flow_control,
	.set_channel = wil_master_set_channel,
	.slave_going_down = wil_master_slave_going_down,
};

static int
wil_add_dev_info(struct wil6210_priv *wil, const void *data, size_t len)
{
	const struct dhd_cmd_add_dev_req *req = data;
	struct dhd_state *dhd = wil->dhd;

	if (len != sizeof(*req))
		return EINVAL;

	if (req->dev_peer_index < 0 || req->dev_peer_index >= DHD_MAX_PEERS)
		return EINVAL;

	if (req->dev_name_unit < 0 || req->dev_name_unit >= DHD_TOTAL_LINKS)
		return EINVAL;

	dhd->dhd_peer_link_id[req->dev_peer_index] = req->dev_name_unit;

	dhd_link_map[req->dev_name_unit].dhd = dhd;
	dhd_link_map[req->dev_name_unit].dhd_port_id = wil->port_id;
	dhd_link_map[req->dev_name_unit].dhd_peer_id = req->dev_peer_index;
	dhd_link_map[req->dev_name_unit].dhd_peer_nameunit = req->dev_name_unit;
	dhd_link_map[req->dev_name_unit].dhd_peer_ifindex = req->dev_ifindex;
	strlcpy(dhd_link_map[req->dev_name_unit].dhd_peer_ifname, req->dev_ifname,
	    sizeof(dhd_link_map[req->dev_name_unit].dhd_peer_ifname));

	return 0;
}

static int
wil_add_link_info(struct wil6210_priv *wil, const void *data, size_t len)
{
	const struct dhd_cmd_add_link_req *req = data;
	uint16_t link_id;
	unsigned int i;

	if (len != sizeof(*req))
		return EINVAL;

	mutex_lock(&wil->mutex);

	if (req->peer_index < 0 || req->peer_index >= DHD_MAX_PEERS)
		return EINVAL;

	/* Find the STA associated with the link */
	for (i = 0; i < WIL6210_MAX_CID; i++) {
		/* Match the link */
		if (wil->sta[i].tx_link == req->tx_link_id &&
		    wil->sta[i].rx_link == req->rx_link_id)
			break;

	}
	if (i >= WIL6210_MAX_CID) {
		wil_err(wil,
		    "No STA for peer %d: tx link %u rx_link %u\n",
		    req->peer_index, req->tx_link_id, req->rx_link_id);
		goto done;
	}

	if (!terra_port_qid_to_link(wil->port_id, req->peer_index, &link_id)) {
		wil_err(wil,
		    "No link for peer %d: tx link %u rx_link %u\n",
		    req->peer_index, req->tx_link_id, req->rx_link_id);
		goto done;
	}
	wil_info(wil, "Add peer %d tx link %u rx_link %u CID %u\n",
	    req->peer_index, req->tx_link_id, req->rx_link_id, i);
	wil->sta[i].peer_id = req->peer_index;
	wil->sta[i].link_id = link_id;

	/* Enable fast path for the peer */
	wil_eth_tx_ring_enable(wil, i, wil->sta[i].peer_id);

	/* Notify API clients about the link coming up */
	wil_api_link_up(wil, wil->sta[i].peer_id, wil->sta[i].addr);
done:
	mutex_unlock(&wil->mutex);
	return 0;
}

static int
wil_del_link_info(struct wil6210_priv *wil, const void *data, size_t len)
{
	const struct dhd_cmd_del_link_req *req = data;
	unsigned int i;

	if (len != sizeof(*req))
		return EINVAL;

	if (req->peer_index < 0 || req->peer_index >= DHD_MAX_PEERS)
		return EINVAL;

	mutex_lock(&wil->mutex);
	/* Find the STA associated with the link */
	for (i = 0; i < WIL6210_MAX_CID; i++) {
		if (wil->sta[i].peer_id == req->peer_index)
			break;
	}
	if (i >= WIL6210_MAX_CID) {
		wil_err(wil, "No STA for peer %u\n",
		    req->peer_index);
		goto done;
	}

	wil_info(wil, "Del peer %u CID %u\n", req->peer_index, i);

	/* Notify API clients about the link drop */
	wil_api_link_down(wil, wil->sta[i].peer_id, wil->sta[i].addr);

	/* Disable fast path */
	wil_eth_tx_ring_disable(wil, i, wil->sta[i].peer_id);
done:
	mutex_unlock(&wil->mutex);
	return 0;
}

static int
wil_sync_handler(void *ctx, struct dhd_call_desc *req)
{
	struct platform_device *pdev = ctx;
	struct wil6210_priv *wil = pdev->wil;
	int rc;

	req->resp_len = 0;
	switch (req->call_opcode) {
	case DHD_CMDOP_REGISTER:
		rc = pdev->slave_ops.register_master(pdev->slave_ctx,
		    pdev, &slave_rops);
		if (rc == 0) {
			wil_info(wil, "Connected to in-kernel interface %s\n",
			    pdev->dhd->dhd_ifname);
		} else {
			wil_err(wil, "Unable to register master for %s\n",
			    pdev->dhd->dhd_ifname);
		}
		break;
	case DHD_CMDOP_IOCTL:
		rc = pdev->slave_ops.ioctl(pdev->slave_ctx, 0,
		       req->call_data, req->call_len, req->resp_data,
		       &req->resp_len);
		break;
	case DHD_CMDOP_ADD_DEV:
		rc = wil_add_dev_info(wil, req->call_data, req->call_len);
		break;
	case DHD_CMDOP_ADD_LINK:
		rc = wil_add_link_info(wil, req->call_data, req->call_len);
		break;
	case DHD_CMDOP_DEL_LINK:
		rc = wil_del_link_info(wil, req->call_data, req->call_len);
		break;
	case DHD_CMDOP_SET_KEY:
		rc = 0;
		break;
	default:
		rc = -ENOENT;
		break;
	}

	return rc;
}

bool terra_port_qid_to_link(uint16_t port_id, uint16_t qid, uint16_t *link_id)
{
	struct dhd_state *dhd;
	int i;

	for (i = 0; i < ARRAY_SIZE(dhd_link_map); i++) {
		dhd = dhd_link_map[i].dhd;

		if (dhd == NULL)
			continue;
		if (dhd_link_map[i].dhd_port_id != port_id)
			continue;
		if (dhd_link_map[i].dhd_peer_id != qid)
			continue;
		*link_id = dhd_link_map[i].dhd_peer_nameunit;
		return true;
	}

	return false;
}

bool terra_link_to_port_qid(uint16_t link_id, uint16_t *port_id, uint16_t *qid)
{
	if (link_id < ARRAY_SIZE(dhd_link_map)) {
		if (dhd_link_map[link_id].dhd != NULL) {
			*port_id = dhd_link_map[link_id].dhd_port_id;
			*qid = dhd_link_map[link_id].dhd_peer_id;
			return true;
		}
	}
	return false;
}

uint16_t read_terra_slowpath(struct rte_mbuf **rx_pkts, const uint16_t nb_pkts)
{
	struct dhd_state *dhd;
	uint16_t rx_done;
	struct pollfd pfd[DHD_MAX_PORTS + 1];
	struct dhd_state *dhds[DHD_MAX_PORTS + 1];
	int i, npoll, rc;

	for ( ; ; ) {

		memset(&pfd, 0, sizeof(pfd));

		npoll = 0;
		for (i = 0; i < DHD_MAX_PORTS; i++) {
			dhd = dhd_list[i];
			if (dhd == NULL)
				continue;
			if (dhd->dhd_rxq[1].queue_fd < 0)
				continue;

			if (npoll == 0) {
				/* Wait for shutdown signal from parent */
				pfd[0].fd = dhd->dhd_pipe[0];
				pfd[0].events = POLLIN;
				dhds[0] = NULL;
				npoll = 1;
			}

			pfd[npoll].fd = dhd->dhd_rxq[1].queue_fd;
			pfd[npoll].events = POLLIN;
			dhds[npoll] = dhd;
			npoll++;
		}

		rc = poll(pfd, npoll, 2000);
		if (rc < 0)
			return 0; /* Make this fatal? */
		if (rc == 0)
			continue;

		/* Check if we were asked to shutdown */
		if (pfd[0].revents & POLLIN)
			return 0;

		/* Iterate over queues, process messages as necessary */
		for (i = 1; i < npoll; i++) {
			if ((pfd[i].revents & POLLIN) == 0)
				continue;

			rx_done = dhd_rx_burst(dhds[i], rx_pkts, nb_pkts);
			if (rx_done > 0)
				return rx_done;
		}
	}
}

/*
 * Instead of forever waiting for packets or error, poll for poll_period
 * and return whatever packets are read
 */
uint16_t read_terra_slowpath_non_blocking(struct rte_mbuf **rx_pkts,
				const uint16_t nb_pkts,  uint16_t poll_period_ms)
{
	struct dhd_state *dhd;
	uint16_t rx_done;
	struct pollfd pfd[DHD_MAX_PORTS + 1];
	struct dhd_state *dhds[DHD_MAX_PORTS + 1];
	int i, npoll, rc;

	if (poll_period_ms > 2000)
		poll_period_ms = 2000; // Dont poll for > 2 seconds default

	for ( ; ; ) {

		memset(&pfd, 0, sizeof(pfd));

		npoll = 0;
		for (i = 0; i < DHD_MAX_PORTS; i++) {
			dhd = dhd_list[i];
			if (dhd == NULL)
				continue;
			if (dhd->dhd_rxq[1].queue_fd < 0)
				continue;

			if (npoll == 0) {
				/* Wait for shutdown signal from parent */
				pfd[0].fd = dhd->dhd_pipe[0];
				pfd[0].events = POLLIN;
				dhds[0] = NULL;
				npoll = 1;
			}

			pfd[npoll].fd = dhd->dhd_rxq[1].queue_fd;
			pfd[npoll].events = POLLIN;
			dhds[npoll] = dhd;
			npoll++;
		}

		rc = poll(pfd, npoll, poll_period_ms);
		if (rc < 0)
			return 0; /* Make this fatal? */
		if (rc == 0) {
			return 0;
		}

		/* Check if we were asked to shutdown */
		if (pfd[0].revents & POLLIN) {
			return 0;
		}

		/* Iterate over queues, process messages as necessary */
		for (i = 1; i < npoll; i++) {
			if ((pfd[i].revents & POLLIN) == 0)
				return 0;

			rx_done = dhd_rx_burst(dhds[i], rx_pkts, nb_pkts);
			if (rx_done > 0)
				return rx_done;
		}
	}
}

uint16_t write_terra_slowpath(struct rte_mbuf **tx_pkts, const uint16_t nb_pkts)
{
	struct dhd_link_info *li;
	struct rte_mbuf *mbuf;
	uint16_t tx_done, tx_batch, tx_left, tx_total;

	tx_total = 0;
	tx_left = nb_pkts;

	while (tx_left > 0) {
		mbuf = tx_pkts[0];

		/* Translate link_id to peer_id */
		li = &dhd_link_map[mbuf->udata64];
		mbuf->udata64 = li->dhd_peer_id;

		for (tx_batch = 1; tx_batch < tx_left; tx_batch++) {
			struct rte_mbuf *next = tx_pkts[tx_batch];
			if (li->dhd_peer_nameunit != (int)next->udata64)
				break;
			next->udata64 = (unsigned)li->dhd_peer_id;
		}

		tx_done = dhd_tx_burst(li->dhd, tx_pkts, tx_batch);
		tx_total += tx_done;
		if (tx_done < tx_batch)
			break;

		tx_pkts += tx_done;
		tx_left -= tx_done;
	}

	return tx_total;
}

void wil_nl_60g_fw_state_change(struct wil6210_priv *wil,
	enum wil_fw_state fw_state)
{
	wil_dbg_misc(wil, "fw_state change:%d => %d", wil->fw_state, fw_state);
	wil->fw_state = fw_state;
	nl60g_fw_state_evt(wil->nl60g, fw_state);
}

void wil_nl_60g_receive_wmi_evt(struct wil6210_priv *wil, u8 *cmd, int len)
{
	nl60g_receive_wmi_evt(wil->nl60g, cmd, len);
}
