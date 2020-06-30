/*
 * Copyright (c) 2013,2016 Qualcomm Atheros, Inc.
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019, Facebook, Inc. All rights reserved.
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

#include <rte_ethdev.h>
#include <rte_wigig_api.h>

#include "wil6210_ethdev.h"
#include "dpdk-dhd-ctrl.h"
#include "dpdk_dhd.h"

static void *
wil_api_device_lookup(uint8_t *mac)
{
	struct ether_addr mac_addr;
	uint32_t pid;

	/*
	 * Iterate over ports, looking for one that belongs to our
	 * driver and has the correct MAC
	 */
	for (pid = 0; pid < RTE_MAX_ETHPORTS; pid++) {
		if (rte_eth_devices[pid].state == RTE_ETH_DEV_UNUSED)
			continue;
		if (strcmp(rte_eth_devices[pid].device->driver->name,
		    "net_wil6210") != 0)
			continue;
		rte_eth_macaddr_get(pid, &mac_addr);
		if (memcmp(mac, mac_addr.addr_bytes, ETHER_ADDR_LEN) != 0)
			continue;
		/* Return pointer to rte_ethdev */
		return &rte_eth_devices[pid];
	}

	return NULL;
}

static int
wil_api_device_info(void *dev_opaque, struct rte_wigig_dev_info *wi)
{
	struct rte_eth_dev *ethdev;
	struct dhd_link_info *li;

	ethdev = dev_opaque;

	wi->port_id = ethdev->data->port_id;
	wi->num_links = 0;
	wi->data_fd = -1;

	li = &dhd_link_map[0];
	for (; li < &dhd_link_map[DHD_TOTAL_LINKS]; li++) {
		if (li->dhd == NULL || li->dhd_port_id != wi->port_id)
			continue;
		wi->link[li->dhd_peer_id].if_nameunit = li->dhd_peer_nameunit;
		wi->link[li->dhd_peer_id].if_peer_id = li->dhd_peer_id;
		strlcpy(wi->link[li->dhd_peer_id].if_name, li->dhd_peer_ifname,
			IFNAMSIZ);
		wi->num_links++;
		/* The device has links: pass in the queue fd to be polled */
		wi->data_fd = li->dhd->dhd_rxq[1].queue_fd;
	}

	return 0;
}

static int
wil_api_slowpath_tx(void *dev_opaque, struct rte_mbuf **tx_pkts, int nb_pkts)
{
	struct rte_eth_dev *ethdev;
	struct wil6210_priv *wil;

	ethdev = dev_opaque;

	/*
	 * Handle the case where caller does not guarantee that all
	 * packets in a vector are for the same BB card, fall back
	 * to generic implementation that knows how to sort packets
	 * out.
	 * This is the case for VPP nodes as implemented at the moment.
	 */
	if (ethdev == NULL)
		return write_terra_slowpath(tx_pkts, nb_pkts);

	wil = ethdev->data->dev_private;
	return dhd_tx_burst(wil->dhd, tx_pkts, nb_pkts);
}

static int
wil_api_slowpath_rx(void *dev_opaque, struct rte_mbuf **rx_pkts, int nb_pkts)
{
	struct rte_eth_dev *ethdev;
	struct wil6210_priv *wil;

	ethdev = dev_opaque;

	wil = ethdev->data->dev_private;
	return dhd_rx_burst(wil->dhd, rx_pkts, nb_pkts);
}

static void
wil_api_set_client_ops(void *dev_opaque, struct rte_wigig_client_ops *ops)
{
	struct rte_eth_dev *ethdev;
	struct wil6210_priv *wil;

	ethdev = dev_opaque;
	wil = ethdev->data->dev_private;

	wil->api_priv = ops;
}

static const struct rte_wigig_ops wigig_ops = {
    .device_lookup = wil_api_device_lookup,
    .set_client_ops = wil_api_set_client_ops,
    .device_info = wil_api_device_info,
    .slowpath_tx = wil_api_slowpath_tx,
    .slowpath_rx = wil_api_slowpath_rx,
};

static struct dhd_link_info *
wil_api_find_link_info(struct wil6210_priv *wil, unsigned int peer_id)
{
	struct dhd_link_info *li;

	ASSERT(wil->dhd != NULL);

	li = &dhd_link_map[0];
	for (; li < &dhd_link_map[DHD_TOTAL_LINKS]; li++) {
		if (li->dhd != wil->dhd || li->dhd_port_id != wil->port_id ||
		    li->dhd_peer_id != peer_id)
			continue;
		return li;
	}

	return NULL;
}


void wil_api_link_up(struct wil6210_priv *wil, unsigned int peer_id,
		     u8 addr[ETH_ALEN])
{
	struct rte_wigig_link_updown_info data;
	struct dhd_link_info *li;
	struct rte_wigig_client_ops *client_ops;

	if (wil->api_priv == NULL)
		return;

	li = wil_api_find_link_info(wil, peer_id);
	if (li == NULL)
		return;

	memset(&data, 0, sizeof(data));
	data.port_id = wil->port_id;
	data.if_nameunit = li->dhd_peer_nameunit;
	data.if_peer_id = peer_id;
	memcpy(data.if_peer_macaddr, addr, sizeof(data.if_peer_macaddr));

	client_ops = wil->api_priv;
	client_ops->link_up(&data);
}

void wil_api_link_down(struct wil6210_priv *wil, unsigned int peer_id,
		       u8 addr[ETH_ALEN])
{
	struct rte_wigig_link_updown_info data;
	struct dhd_link_info *li;
	struct rte_wigig_client_ops *client_ops;

	if (wil->api_priv == NULL)
		return;

	li = wil_api_find_link_info(wil, peer_id);
	if (li == NULL)
		return;

	memset(&data, 0, sizeof(data));
	data.port_id = wil->port_id;
	data.if_nameunit = li->dhd_peer_nameunit;
	data.if_peer_id = peer_id;
	memcpy(data.if_peer_macaddr, addr, sizeof(data.if_peer_macaddr));

	client_ops = wil->api_priv;
	client_ops->link_down(&data);
}

void wil_api_fw_recovery(struct wil6210_priv *wil)
{
	struct rte_wigig_recovery_info data;
	struct rte_wigig_client_ops *client_ops;

	if (wil->no_fw_recovery) {
		wil_info(wil, "FW recovery cancelled, no_fw_recovery\n");
		return;
	}

	client_ops = wil->api_priv;

	if (!client_ops || !client_ops->wigig_recovery) {
		wil_err(wil,
			"No firmware recovery op implemented by client, asserting...\n");
		BUG_ON(1);
		return;
	}

	data.port_id = wil->port_id;
	/* client is expected to stop and restart device to reset fw */
	client_ops->wigig_recovery(&data);
}

const struct rte_wigig_ops *
rte_wigig_get_ops(void)
{
	return &wigig_ops;
}
