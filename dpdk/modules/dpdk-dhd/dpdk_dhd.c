/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/* Copyright (c) 2020, Facebook, Inc. All rights reserved. */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/rtnetlink.h>
#include <linux/pci.h>

#include <fb_tg_backhaul_if.h>

#include "dpdk_dhd.h"

#define BH_MQ_BK        0
#define BH_MQ_BE        1
#define BH_MQ_VI        2
#define BH_MQ_VO        3

#define DHD_DEVICE "dhd"

#define PCI_ADDR_LEN 12

struct dhd_ioctl_info {
	const void *req_buf;
	void *res_buf;
	size_t req_len;
	size_t res_len;
	uint32_t code;
	int32_t result;
	uint32_t wait_seqno;
	uint32_t next_seqno;
	struct completion completion;
	bool dead;
};

struct dhd_peer_info {
	int32_t tx_link_id;
	int32_t rx_link_id;
	void __rcu *link_dev;
	uint64_t	rx_packets;
	uint64_t	tx_packets;
	uint64_t	rx_bytes;
	uint64_t	tx_bytes;
	uint64_t	rx_errors;
	uint64_t	tx_errors;
};
#define DHD_MAX_PEERS 16

struct dhd_bh_stats {
	uint64_t cfg_requests;
	uint64_t cfg_responses;
	uint64_t bh_ioctls;
	uint64_t add_links;
	uint64_t rm_links;
	uint64_t events;
};

typedef struct dhd_bh_info {
	struct net_device *bh_dev;
	struct mutex mtx_ioctl;
	spinlock_t bh_lock;
	struct dhd_ioctl_info ioctl;
	const struct tgd_bh_callback_ops __rcu *client_ops;
	void __rcu *client_ctxt;
	u8 mac_addr[ETH_ALEN];
	struct file *bh_file;
	struct platform_device *platform_dev;
	struct net_device_stats netdev_stats;
	struct dhd_bh_stats bh_stats;
	struct dhd_peer_info peer[DHD_MAX_PEERS];
	int pci_dev_index;
} dhd_bh_info_t;

static char* pci_order = NULL;
module_param(pci_order, charp, 0444);
MODULE_PARM_DESC(pci_order, " index interfaces for given PCI addresses");

static int
dhd_bh_dev_index(uint16_t domain, uint8_t bus, uint8_t devid, uint8_t function)
{
	int i;
	char* pos;
	char pci_addr[PCI_ADDR_LEN + 1];

	if (pci_order == NULL) {
		return -1;
	}

	snprintf(pci_addr, sizeof(pci_addr), "%04x:%02x:%02x.%d",
		domain, bus, devid, function);

	i = 0;
	for (pos = pci_order; *pos != '\0'; pos++) {
		if (*pos == ',') {
			i++;
		} else if (strncmp(pci_addr, pos, PCI_ADDR_LEN) == 0) {
			return i;
		}
	}
	return -1;
}

static int
dhd_bh_send_buf(dhd_bh_info_t *dhd, uint32_t cmd, uint32_t seqno,
    struct sk_buff *skb)
{
	struct net_device *out_dev;
	struct dhd_pkt_header dhd_hdr;
	struct ethhdr eth_hdr;
	uint8_t	*data;
	uint8_t	pkt_meta_size;

	out_dev = dhd->bh_dev;

	pkt_meta_size = sizeof(dhd_hdr) + sizeof(struct ethhdr);

	dhd_hdr.cmd_code = cmd;
	dhd_hdr.cmd_seqno = seqno;
	dhd_hdr.cmd_flags = 0;
	dhd_hdr.cmd_result = 0;
	dhd_hdr.cmd_reserved = 0;
	dhd_hdr.cmd_len = skb->len + pkt_meta_size;

	data = skb_push(skb, sizeof(dhd_hdr));
	memcpy(data, &dhd_hdr, sizeof(dhd_hdr));

	memset(eth_hdr.h_dest, 0xff, sizeof(eth_hdr.h_dest));
	memcpy(eth_hdr.h_source, out_dev->dev_addr, sizeof(eth_hdr.h_source));
	eth_hdr.h_proto = htons(DHD_ETH_TYPE_CFG);

	data = skb_push(skb, sizeof(eth_hdr));
	memcpy(data, &eth_hdr, sizeof(eth_hdr));

	skb_set_mac_header(skb, 0);
	skb_set_network_header(skb, sizeof(eth_hdr));
	skb->protocol = htons(DHD_ETH_TYPE_CFG);
	skb->dev = out_dev;

	return netif_rx(skb);
}

static void
dhd_bh_sync_terminate(dhd_bh_info_t *dhd, int result)
{
	unsigned long flags;

	spin_lock_irqsave(&dhd->bh_lock, flags);
	dhd->ioctl.dead = true;
	if (dhd->ioctl.code != 0) {
		netdev_err(dhd->bh_dev,
		    "Terminating call %u seqno %08x with %d\n",
		    dhd->ioctl.code, dhd->ioctl.wait_seqno, result);
		dhd->ioctl.result = result;
		complete(&dhd->ioctl.completion);
	}
	spin_unlock_irqrestore(&dhd->bh_lock, flags);
}

static void
dhd_bh_sync_complete(dhd_bh_info_t *dhd, struct dhd_pkt_header *dhd_hdr,
    void *data, size_t len)
{
	unsigned long flags;

	spin_lock_irqsave(&dhd->bh_lock, flags);
	if (dhd_hdr->cmd_code != dhd->ioctl.code || dhd->ioctl.code == 0) {
		netdev_err(dhd->bh_dev, "Unexpected opcode: got %u, expected %u\n",
		    dhd_hdr->cmd_code, dhd->ioctl.code);
		goto out;
	}
	if (dhd_hdr->cmd_seqno != dhd->ioctl.wait_seqno) {
		netdev_err(dhd->bh_dev, "Unexpected seqno: got %08x, expected %08x\n",
		    dhd_hdr->cmd_seqno, dhd->ioctl.wait_seqno);
		goto out;
	}
	/* At this point we know that this response is meant for us */
	dhd->ioctl.result = dhd_hdr->cmd_result;

	/* Copy response data */
	if (dhd->ioctl.res_buf != NULL) {
		if (len > dhd->ioctl.res_len) {
			netdev_dbg(dhd->bh_dev,
			    "ioctl response buffer overflow: %lu > %lu\n",
			    len, dhd->ioctl.res_len);
			len = dhd->ioctl.res_len;
		}
		memcpy(dhd->ioctl.res_buf, data, len);
	}

	netdev_dbg(dhd->bh_dev, "Received SYNC RESPONSE seqno %08x\n",
	    dhd_hdr->cmd_seqno);
	/* Cleanup saved call info */
	dhd->ioctl.code = 0;
	complete(&dhd->ioctl.completion);
	dhd->bh_stats.cfg_responses ++;
out:
	spin_unlock_irqrestore(&dhd->bh_lock, flags);
}

static void
dhd_bh_rx_event(dhd_bh_info_t *dhd, void *data, size_t len)
{
	const struct tgd_bh_callback_ops *ops;

	netdev_dbg(dhd->bh_dev, "Received EVENT %lu byte(s)\n", len);
	ops = rcu_dereference(dhd->client_ops);
	if (ops != NULL)
		ops->rx_event(dhd->client_ctxt, data, len);
	dhd->bh_stats.events ++;
}

static rx_handler_result_t
dhd_bh_cfg_rx_handler(struct sk_buff *skb)
{
	struct net_device *dev;
	struct ethhdr *ehdr;
	struct dhd_pkt_header dhd_hdr;
	dhd_bh_info_t *dhd;
	uint8_t	pkt_meta_size;
	unsigned int len;

	pkt_meta_size = sizeof(dhd_hdr) + sizeof(struct ethhdr);

	dev = skb->dev;
	dhd = netdev_priv(dev);
	len = skb->len;

	/* Verify if packet has sane size */
	if (len < pkt_meta_size) {
		netdev_err(dev, "Unexpected packet size %u\n", skb->len);
		goto drop;
	}

	ehdr = (struct ethhdr *)skb->data;
	BUG_ON(ehdr->h_proto != htons(DHD_ETH_TYPE_CFG));

	/* Copy out header from possibly unaligned location */
	memcpy(&dhd_hdr, skb_pull(skb, sizeof(*ehdr)), sizeof(dhd_hdr));
	/* Verify that header is sane */
	if (dhd_hdr.cmd_len != len || (dhd_hdr.cmd_code < DHD_CMDOP_FIRST ||
	    dhd_hdr.cmd_code > DHD_CMDOP_LAST)) {
		netdev_err(dev, "Unexpected packet opcode %u len %04x\n",
		   dhd_hdr.cmd_code, dhd_hdr.cmd_len);
		goto drop;
	}
	/* Point skb_data at payload */
	skb_pull(skb, sizeof(dhd_hdr));

	/* Handle the message */
	switch (dhd_hdr.cmd_code) {
	case DHD_CMDOP_IOCTL:
	case DHD_CMDOP_ADD_DEV:
	case DHD_CMDOP_ADD_LINK:
	case DHD_CMDOP_DEL_LINK:
	case DHD_CMDOP_SET_KEY:
	case DHD_CMDOP_REGISTER:
		dhd_bh_sync_complete(dhd, &dhd_hdr, skb->data, skb->len);
		break;
	case DHD_CMDOP_EVENT:
		dhd_bh_rx_event(dhd, skb->data, skb->len);
		break;
	default:
		goto drop;
	};

	/* FALLTHOUGH: consume packet in all cases */
drop:
	kfree_skb(skb);
	return RX_HANDLER_CONSUMED;
}

static rx_handler_result_t
dhd_bh_data_rx_handler(struct sk_buff *skb)
{
	struct net_device *dev, *link_dev;
	struct dhd_data_header dhd_hdr, *hdr;
	dhd_bh_info_t *dhd;
	uint8_t	pkt_meta_size;

	pkt_meta_size = sizeof(dhd_hdr) + sizeof(struct ethhdr);

	dev = skb->dev;
	dhd = netdev_priv(dev);

#if 0
	netdev_err(dev, "PKT RX\n");
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 16, 1, (void *)skb->data,
	    skb->len, 0);
#endif
	hdr = skb_pull(skb, sizeof(struct ethhdr));
	memcpy(&dhd_hdr, hdr, sizeof(*hdr));
	skb_pull(skb, sizeof(dhd_hdr));

	if (dhd_hdr.peer_index < 0 || dhd_hdr.peer_index >= DHD_MAX_PEERS) {
		netdev_err(dev, "invalid peer index %u\n", dhd_hdr.peer_index);
		goto drop;
	}

	link_dev = rcu_dereference(dhd->peer[dhd_hdr.peer_index].link_dev);
	if (link_dev == NULL) {
		netdev_err(skb->dev, "No link dev, peer %u\n", dhd_hdr.peer_index);
		dhd->peer[dhd_hdr.peer_index].rx_errors ++;
		goto drop;
	}

	skb->protocol = eth_type_trans(skb, link_dev);

	dhd->netdev_stats.rx_packets ++;
	dhd->netdev_stats.rx_bytes += skb->len;
	dhd->peer[dhd_hdr.peer_index].rx_packets ++;
	dhd->peer[dhd_hdr.peer_index].rx_bytes += skb->len;
	netif_rx(skb);
	return RX_HANDLER_CONSUMED;
drop:
	dhd->netdev_stats.rx_errors ++;
	kfree_skb(skb);
	return RX_HANDLER_CONSUMED;
}

static rx_handler_result_t
dhd_bh_rx_handler(struct sk_buff **pskb)
{
	struct ethhdr *ehdr;
	struct sk_buff *skb;

	skb = *pskb;

	/* Verify if packet has sane size */
	if (skb->len < sizeof(*ehdr)) {
		netdev_err(skb->dev, "Unexpected packet size %u\n", skb->len);
		goto drop;
	}

	/* See what type of packet we have got */
	ehdr = (struct ethhdr *)skb->data;
	if (ehdr->h_proto == htons(DHD_ETH_TYPE_CFG)) {
		return dhd_bh_cfg_rx_handler(skb);
	} else if (ehdr->h_proto == htons(DHD_ETH_TYPE_DATA)) {
		return dhd_bh_data_rx_handler(skb);
	}
	netdev_dbg(skb->dev, "Unexpected packet type %04x len %u\n",
	    ntohs(ehdr->h_proto), skb->len);
drop:
	kfree_skb(skb);
	return RX_HANDLER_CONSUMED;
}

static struct sk_buff *
dhd_bh_alloc_skb(dhd_bh_info_t *dhd)
{
	struct sk_buff *skb;

	/*
	 * Attempt our best to allocate SKBs from the kernel.
	 * TODO: This cannot fail, replace with some kind of pool
	 * allocation scheme.
	 */
	skb  = __netdev_alloc_skb(dhd->bh_dev, 7800, GFP_KERNEL);
	return skb;
}

static int
dhd_bh_call(dhd_bh_info_t *dhd, uint32_t opcode, const void *req_buf,
    uint req_len, void *resp_buf, uint resp_len)
{
	struct sk_buff *skb;
	unsigned long flags;
	uint8_t *data;
	int res;

	skb = dhd_bh_alloc_skb(dhd);
	if (skb == NULL) {
		netdev_err(dhd->bh_dev, "Unable to allocate buffer for ioctl\n");
		return -ENOMEM;
	}

	data = skb_put(skb, req_len);
	memcpy(data, req_buf, req_len);

	/* Only one ioctl caller at a time */
	mutex_lock(&dhd->mtx_ioctl);

	/* Remember data pointers */
	spin_lock_irqsave(&dhd->bh_lock, flags);
	if (unlikely(dhd->ioctl.dead)) {
		kfree_skb(skb);
		res = -ENXIO;
	} else {
		dhd->ioctl.code = opcode;
		dhd->ioctl.wait_seqno = dhd->ioctl.next_seqno++;
		dhd->ioctl.req_buf = req_buf;
		dhd->ioctl.req_len = req_len;
		dhd->ioctl.res_buf = resp_buf;
		dhd->ioctl.res_len = resp_len;
		reinit_completion(&dhd->ioctl.completion);
		netdev_dbg(dhd->bh_dev, "Sending ioctl seqno %08x\n",
		    dhd->ioctl.wait_seqno);
		res = 0;
	}
	spin_unlock_irqrestore(&dhd->bh_lock, flags);

	/* Send the request down */
	if (res == 0)
		res = dhd_bh_send_buf(dhd, opcode, dhd->ioctl.wait_seqno, skb);
	if (res == 0) {
		/* Wait for response with timeout */
		res = wait_for_completion_timeout(&dhd->ioctl.completion,
		    5 * HZ);
		if (res == 0)  {
			res = -ETIMEDOUT;
			dhd_bh_sync_terminate(dhd, res);
		} else
			res = dhd->ioctl.result;
	}

	dhd->bh_stats.cfg_requests ++;
	/* Release mutex */
	mutex_unlock(&dhd->mtx_ioctl);
	return res;
}

static int
dhd_bh_setup_netdev(void *dev, struct net_device *ndev,
    struct tgd_bh_netdev_desc *dev_desc)
{
	dhd_bh_info_t *dhd = dev;
	struct dhd_cmd_add_dev_req req;

	req.dev_name_unit = dev_desc->devNameUnit;
	req.dev_peer_index = dev_desc->devPeerIndex;
	req.dev_ifindex = ndev->ifindex;
	strlcpy(req.dev_ifname, netdev_name(ndev), sizeof(req.dev_ifname));

	return dhd_bh_call(dhd, DHD_CMDOP_ADD_DEV, &req, sizeof(req),
	    NULL, 0);
}

static int
dhd_bh_add_link_info(void *dev, struct tgd_bh_link_info_desc *ld)
{
	dhd_bh_info_t *dhd = dev;
	struct dhd_cmd_add_link_req req;
	struct net_device *ndev;

	ndev = ld->linkDev;

	req.tx_link_id = ld->txLinkId;
	req.rx_link_id = ld->rxLinkId;
	req.peer_index = ld->peerIndex;

	dhd->peer[ld->peerIndex].tx_link_id = req.tx_link_id;
	dhd->peer[ld->peerIndex].rx_link_id = req.rx_link_id;
	rcu_assign_pointer(dhd->peer[ld->peerIndex].link_dev, ndev);

	netdev_dbg(dhd->bh_dev, "Adding links peer %d tx %d rx %d ndev %p name %s\n",
	    req.peer_index, req.tx_link_id, req.rx_link_id, ndev, netdev_name(ndev));

	dhd->bh_stats.add_links ++;

	return dhd_bh_call(dhd, DHD_CMDOP_ADD_LINK, &req, sizeof(req),
	    NULL, 0);
}

static int
dhd_bh_del_link_info(void *dev, struct tgd_bh_link_info_desc *ld)
{
	dhd_bh_info_t *dhd = dev;
	struct dhd_cmd_del_link_req req;

	req.peer_index = ld->peerIndex;

	dhd->peer[ld->peerIndex].tx_link_id = -1;
	dhd->peer[ld->peerIndex].rx_link_id = -1;
	rcu_assign_pointer(dhd->peer[ld->peerIndex].link_dev, NULL);

	dhd->bh_stats.rm_links ++;

	return dhd_bh_call(dhd, DHD_CMDOP_DEL_LINK, &req, sizeof(req),
	    NULL, 0);
}

static int
dhd_bh_set_key(void *dev, int peer_index, const uint8_t *dest_mac,
    const uint8_t *key_data, uint key_len)
{
	dhd_bh_info_t *dhd = dev;
	struct sk_buff *skb;
	unsigned long flags;
	struct dhd_cmd_set_key *req;
	uint32_t opcode;
	int res;

	skb = dhd_bh_alloc_skb(dhd);
	if (skb == NULL) {
		netdev_err(dhd->bh_dev, "Unable to allocate buffer for ioctl\n");
		return -ENOMEM;
	}

	opcode = DHD_CMDOP_SET_KEY;
	req = skb_put(skb, sizeof(*req) + key_len);
	memcpy(req->mac_addr, dest_mac, ETH_ALEN);
	memcpy(req->key_data, key_data, key_len);
	req->key_len = key_len;
	req->peer_index = peer_index;

	/* Only one ioctl caller at a time */
	mutex_lock(&dhd->mtx_ioctl);

	/* Remember data pointers */
	spin_lock_irqsave(&dhd->bh_lock, flags);
	if (unlikely(dhd->ioctl.dead)) {
		kfree_skb(skb);
		res = -ENXIO;
	} else {
		dhd->ioctl.code = opcode;
		dhd->ioctl.wait_seqno = dhd->ioctl.next_seqno++;
		dhd->ioctl.req_buf = NULL;
		dhd->ioctl.req_len = 0;
		dhd->ioctl.res_buf = NULL;
		dhd->ioctl.res_len = 0;
		reinit_completion(&dhd->ioctl.completion);
		netdev_dbg(dhd->bh_dev, "Sending ioctl seqno %08x\n",
		    dhd->ioctl.wait_seqno);
		res = 0;
	}
	spin_unlock_irqrestore(&dhd->bh_lock, flags);

	/* Send the request down */
	if (res == 0)
		res = dhd_bh_send_buf(dhd, opcode, dhd->ioctl.wait_seqno, skb);
	if (res == 0) {
		/* Wait for response with timeout */
		res = wait_for_completion_timeout(&dhd->ioctl.completion,
		    5 * HZ);
		if (res == 0)  {
			res = -ETIMEDOUT;
			dhd_bh_sync_terminate(dhd, res);
		} else
			res = dhd->ioctl.result;
	}

	/* Release mutex */
	mutex_unlock(&dhd->mtx_ioctl);

	return res;
}

static void
dhd_bh_tx_data(void *dev, struct sk_buff *skb, struct tgd_bh_data_txd *txd)
{
	dhd_bh_info_t *dhd;
	struct net_device *out_dev;
	struct dhd_data_header dhd_hdr;
	struct ethhdr eth_hdr;
	uint8_t	*data;
	uint8_t	pkt_meta_size;

	dhd = dev;
	out_dev = dhd->bh_dev;

	pkt_meta_size = sizeof(dhd_hdr) + sizeof(struct ethhdr);

	dhd_hdr.peer_index = txd->peerIndex;
	dhd_hdr.link_id = txd->txLinkId;
	dhd_hdr.lifetime = txd->lifetime;

	data = skb_push(skb, sizeof(dhd_hdr));
	memcpy(data, &dhd_hdr, sizeof(dhd_hdr));

	memset(eth_hdr.h_dest, 0xff, sizeof(eth_hdr.h_dest));
	memcpy(eth_hdr.h_source, out_dev->dev_addr, sizeof(eth_hdr.h_source));
	eth_hdr.h_proto = htons(DHD_ETH_TYPE_DATA);

	data = skb_push(skb, sizeof(eth_hdr));
	memcpy(data, &eth_hdr, sizeof(eth_hdr));

	skb_set_mac_header(skb, 0);
	skb_set_network_header(skb, sizeof(eth_hdr));
	skb->protocol = htons(DHD_ETH_TYPE_DATA);
	skb->dev = out_dev;

	dhd->netdev_stats.tx_packets ++;
	dhd->netdev_stats.tx_bytes += skb->len;
	dhd->peer[dhd_hdr.link_id].tx_packets ++;
	dhd->peer[dhd_hdr.link_id].tx_bytes += skb->len;
	(void)netif_rx(skb);
}

static int
dhd_bh_link_stats(void *dev, int link_id, struct tgd_bh_link_stats *stats)
{
	dhd_bh_info_t *dhd = dev;
	memset(stats, 0, sizeof(*stats));
	stats->pkts_recved = dhd->peer[link_id].rx_packets;
	stats->bytes_recved = dhd->peer[link_id].rx_bytes;
	stats->rx_err = dhd->peer[link_id].rx_errors;
	stats->pkts_sent = dhd->peer[link_id].tx_packets;
	stats->bytes_sent = dhd->peer[link_id].tx_bytes;
	stats->tx_err = dhd->peer[link_id].tx_errors;
	return 0;
}

static int
dhd_bh_ioctl(void *dev, const uint8_t *req_buf, uint req_len,
    uint8_t *resp_buf, uint resp_len)
{
	dhd_bh_info_t *dhd = dev;

	dhd->bh_stats.bh_ioctls ++;
	return dhd_bh_call(dhd, DHD_CMDOP_IOCTL, req_buf, req_len,
	    resp_buf, resp_len);
}

static int
dhd_bh_register_client(void *plat_data, struct tgd_bh_client_info *ci,
    void **dhd_ctx)
{
	struct dhd_cmd_register_req req;
	dhd_bh_info_t *dhd;
	int rc;

	dhd = plat_data;

	/*
	 * Refuse to update client while one already is registered
	 */
	if (rcu_dereference(dhd->client_ctxt) != NULL) {
		netdev_err(dhd->bh_dev, "%s: Client already attached\n",
		    __FUNCTION__);
		rc = -EBUSY;
		goto exit;
	}

	/* Check version compatibility */
	if (ci->client_ops->api_version != TGD_BH_API_VERSION) {
		netdev_err(dhd->bh_dev,
			   "%s: api version mismatch. (0x%08x) != (0x%08x)\n",
			__FUNCTION__, TGD_BH_API_VERSION,
			ci->client_ops->api_version);
		rc = -EINVAL;
		goto exit;
	}

	if (ci->client_max_peers > DHD_MAX_PEERS) {
		netdev_err(dhd->bh_dev,
			   "%s: too many peers %u requested (max %u)\n",
			__FUNCTION__, ci->client_max_peers, DHD_MAX_PEERS);
		rc = -EINVAL;
		goto exit;
	}

	rcu_assign_pointer(dhd->client_ctxt, ci->client_ctx);
	rcu_assign_pointer(dhd->client_ops, ci->client_ops);

	/* Let the userland know that we are alive and ready */
	req.max_peers = ci->client_max_peers;
	rc = dhd_bh_call(dhd, DHD_CMDOP_REGISTER, &req, sizeof(req), NULL, 0);
	if (rc != 0) {
		rcu_assign_pointer(dhd->client_ctxt, NULL);
		rcu_assign_pointer(dhd->client_ops, NULL);
		goto exit;
	}

	*dhd_ctx = dhd;
	rc = 0;
exit:
	return rc;
}

static int
dhd_bh_unregister_client(void *dev)
{
	dhd_bh_info_t *dhd = dev;
	int ret;

	if (!dhd->client_ops) {
		netdev_err(dhd->bh_dev, "%s: client already unregistered\n",
		    __FUNCTION__);
		ret = -EINVAL;
		goto exit;
	}

	RCU_INIT_POINTER(dhd->client_ops, NULL);
	synchronize_net();
	RCU_INIT_POINTER(dhd->client_ctxt, NULL);
	ret = 0;
exit:
	netdev_info(dhd->bh_dev, "%s: bh %p, ret %d\n", __FUNCTION__, dhd, ret);
	return ret;
}


/* map user priority 0..7 to AC 0..3, see https://en.wikipedia.org/wiki/IEEE_802.11e-2005 */
static const unsigned char bh_prio_mq_map[BH_MQ_PRIO_NUM] = {
	BH_MQ_BE,	/* prio 0 (best effort)		--> best effort */
	BH_MQ_BK,	/* prio 1 (background)		--> background */
	BH_MQ_BK,	/* prio 2 (spare)		--> background */
	BH_MQ_BE,	/* prio 3 (excellent effort)	--> best effort */
	BH_MQ_VI,	/* prio 4 (controlled load)	--> video */
	BH_MQ_VI,	/* prio 4 (video)		--> video */
	BH_MQ_VO,	/* prio 4 (voice)		--> voice */
	BH_MQ_VO,	/* prio 4 (network control)	--> voice */
};

static const struct tgd_bh_ops dhd_bh_ops_tbl = {
	.api_version = TGD_BH_API_VERSION,
	.bh_prio_mq_map = &bh_prio_mq_map,
	.register_client = dhd_bh_register_client,
	.unregister_client = dhd_bh_unregister_client,
	.setup_netdev = dhd_bh_setup_netdev,
	.tx_data = dhd_bh_tx_data,
	.link_stats = dhd_bh_link_stats,
	.ioctl = dhd_bh_ioctl,
	.set_key = dhd_bh_set_key,
	.add_link_info = dhd_bh_add_link_info,
	.delete_link_info = dhd_bh_del_link_info,
};

static const struct tgd_bh_ops *dhd_bh_ops = &dhd_bh_ops_tbl;

static int dhd_bh_register_device(struct dhd_bh_info *bh)
{
	struct platform_device *bh_pdev;
	struct platform_device_info bh_dev;
	struct tgd_bh_platdata data;

	bh->platform_dev = NULL;

	memset(&bh_dev, 0, sizeof(bh_dev));
	memset(&data, 0, sizeof(data));

	bh_dev.name = TGD_BH_COMPATIBLE_STRING;
	bh_dev.id = bh->pci_dev_index >= 0 ? bh->pci_dev_index : PLATFORM_DEVID_AUTO;

	/* Wrap host pointers */
	data.drv_bh_ops = dhd_bh_ops;
	data.drv_bh_ctx = bh;
	ether_addr_copy(data.mac_addr, bh->mac_addr);

	bh_dev.data = &data;
	bh_dev.size_data = sizeof(data);

	bh_pdev = platform_device_register_full(&bh_dev);
	if (IS_ERR(bh_pdev))
		return PTR_ERR(bh_pdev);

	bh->platform_dev = bh_pdev;

	if (platform_get_drvdata(bh_pdev) == NULL)
		return -ENXIO;

	return 0;
}

static void dhd_bh_unregister_device(struct dhd_bh_info *bh)
{
	if (bh->platform_dev != NULL) {
		platform_device_unregister(bh->platform_dev);
		bh->platform_dev = NULL;
	}
}

/*
 * Virtual interface to be used to relay API calls between userland and kernel
 */

static netdev_tx_t dhd_bh_netdev_xmit(struct sk_buff *skb,
    struct net_device *dev)
{
	(void)dhd_bh_rx_handler(&skb);
	return NETDEV_TX_OK;
}

static struct net_device_stats *dhd_bh_netdev_get_stats(struct net_device *dev)
{
	dhd_bh_info_t *dhd = netdev_priv(dev);

	return &dhd->netdev_stats;
}

static struct dhd_bh_stats *dhd_bh_get_stats(struct net_device *dev)
{
	dhd_bh_info_t *dhd = netdev_priv(dev);

	return &dhd->bh_stats;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
static u16 dhd_bh_select_queue(struct net_device *dev, struct sk_buff *skb,
			       struct net_device *sb_dev)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
static u16 dhd_bh_select_queue(struct net_device *dev, struct sk_buff *skb,
			       struct net_device *sb_dev, select_queue_fallback_t fback)
#else
static u16 dhd_bh_select_queue(struct net_device *dev, struct sk_buff *skb,
			       void *accel_priv, select_queue_fallback_t fback)
#endif
{
	if (skb->protocol == htons(DHD_ETH_TYPE_CFG))
		return 0;
	return 1;
}

static const char dhd_stat_strings[][ETH_GSTRING_LEN] = {
	"rx_packets",
	"tx_packets",
	"rx_bytes",
	"tx_bytes",
	"rx_errors",
	"tx_errors",
	"cfg_requests",
	"cfg_responses",
	"bh_ioctls",
	"add_links",
	"rm_links",
	"events",
};

#define DHD_NUM_ETHTOOL_STATS \
			( sizeof(dhd_stat_strings) / sizeof(dhd_stat_strings[0]) )

static void dhd_get_strings(struct net_device *dev, u32 stringset,
			      u8 *data)
{
	if (stringset != ETH_SS_STATS)
		return;
	memcpy(data, dhd_stat_strings, sizeof(dhd_stat_strings));
}

static int dhd_get_sset_count(struct net_device *dev, int string_set)
{
	switch (string_set) {
	case ETH_SS_STATS:
		return DHD_NUM_ETHTOOL_STATS;
	}
	return -EOPNOTSUPP;
}

static void ethtool_op_get_dhd_stats(struct net_device *dev,
			struct ethtool_stats * ethtool_stats, u64 *data)
{
	struct dhd_bh_stats * b_stats = dhd_bh_get_stats(dev);
	struct net_device_stats * n_stats = dhd_bh_netdev_get_stats(dev);
	/* Report packet stats */
	data[0] = n_stats->rx_packets;
	data[1] = n_stats->tx_packets;
	data[2] = n_stats->rx_bytes;
	data[3] = n_stats->tx_bytes;
	data[4] = n_stats->rx_errors;
	data[5] = n_stats->tx_errors;
	data[6] = b_stats->cfg_requests;
	data[7] = b_stats->cfg_responses;
	data[8] = b_stats->bh_ioctls;
	data[9] = b_stats->add_links;
	data[10] = b_stats->rm_links;
	data[11] = b_stats->events;
}

static const struct ethtool_ops dhd_ethtool_ops = {
	.get_link = ethtool_op_get_link,
	.get_ethtool_stats = ethtool_op_get_dhd_stats,
	.get_strings = dhd_get_strings,
	.get_sset_count = dhd_get_sset_count,
};

static const struct net_device_ops dhd_bh_netdev_ops = {
	.ndo_start_xmit		= dhd_bh_netdev_xmit,
	.ndo_get_stats 		= dhd_bh_netdev_get_stats,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_select_queue	= dhd_bh_select_queue,
	.ndo_validate_addr	= eth_validate_addr,
};

static int dhd_bh_start(dhd_bh_info_t *dhd);
static void dhd_bh_stop(dhd_bh_info_t *dhd);
static void dhd_bh_cleanup(dhd_bh_info_t *dhd);

static void dhd_bh_netdev_free(struct net_device *dev)
{
}

/*
 * Extra headroom for packet metadata
 */
#define DHD_BH_KERN_HEADROM_SIZE 128

static void dhd_bh_netdev_setup(struct net_device *dev)
{
	ether_setup(dev);

	/* Initialize the device structure. */
	dev->netdev_ops = &dhd_bh_netdev_ops;
	dev->ethtool_ops = &dhd_ethtool_ops;

	dev->needs_free_netdev = true;
	dev->priv_destructor = dhd_bh_netdev_free;

	dev->mtu = 7800;
	dev->max_mtu = 7800;

	/* We do not want to see any wild traffic */
	dev->flags |= IFF_NOARP;
	dev->flags &= ~(IFF_MULTICAST | IFF_BROADCAST);

	/* Add some head room for this driver to prepend custom data to tx pkt */
	dev->needed_headroom += DHD_BH_KERN_HEADROM_SIZE;
}

static int dhd_bh_attach(struct file *file,
    struct dhd_attach_request *attach_req)
{
	dhd_bh_info_t *dhd;
	struct net_device *dev;
	int rc;
	char device_name[sizeof(DHD_DEVICE) + 3];

	int dev_index =	dhd_bh_dev_index(attach_req->pci_domain,
		attach_req->pci_bus,
		attach_req->pci_devid,
		attach_req->pci_function);

	/* Use dev_index if specified, otherwise autoassign index for device name */
	if (dev_index >= 0) {
		snprintf(device_name, sizeof(device_name), DHD_DEVICE "%d", dev_index);
	} else {
		snprintf(device_name, sizeof(device_name), DHD_DEVICE "%%d");
	}

	/* Allocate net device, including space for private structure */
	dev = alloc_netdev_mqs(sizeof(*dhd), device_name, NET_NAME_UNKNOWN,
			   dhd_bh_netdev_setup, 2, 1);
	if (dev == NULL)
		return -ENOMEM;

	dhd = netdev_priv(dev);
	dhd->bh_dev = dev;
	dhd->bh_file = file;
	mutex_init(&dhd->mtx_ioctl);

	ether_addr_copy(dhd->mac_addr, attach_req->mac_addr);
	ether_addr_copy(dev->dev_addr, attach_req->mac_addr);
	dhd->pci_dev_index = dev_index;

	spin_lock_init(&dhd->bh_lock);
	dhd->ioctl.next_seqno = 0xface0001;
	init_completion(&dhd->ioctl.completion);

	rtnl_lock();
#if 0
	rc = netdev_rx_handler_register(dev, dhd_bh_rx_handler, dev);
	if (rc < 0)
		goto fail;
#endif
	rc = register_netdevice(dev);
	if (rc < 0)
		goto fail;
	strlcpy(attach_req->if_name, netdev_name(dev),
		sizeof(attach_req->if_name));
	attach_req->if_index = dev->ifindex;
	file->private_data = dhd;
	rtnl_unlock();
	return 0;

fail:
	free_netdev(dev);
	rtnl_unlock();
	return rc;
}

static int dhd_bh_start(dhd_bh_info_t *dhd)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&dhd->bh_lock, flags);
	dhd->ioctl.dead = false;
	spin_unlock_irqrestore(&dhd->bh_lock, flags);

	/* Register this instance so that clients can bind to it */
	rc = dhd_bh_register_device(dhd);
	if (rc != 0)
		dhd_bh_stop(dhd);
	return rc;
}

static void dhd_bh_stop(dhd_bh_info_t *dhd)
{
	/* Terminate all ioctls */
	dhd_bh_sync_terminate(dhd, -ENXIO);

	/* Detach from the clients */
	if (dhd->platform_dev != NULL)
		dhd_bh_unregister_device(dhd);
}

#define LINK_RETRAIN_TIMEOUT	HZ

static int
dhd_bh_pcie_retrain(dhd_bh_info_t *dhd, struct dhd_pcie_retrain_request *req)
{
	unsigned int devfn = PCI_DEVFN(req->devid, req->function);
	struct pci_dev *dev = pci_get_domain_bus_and_slot(req->domain,
		req->bus, devfn);
	struct pci_dev *pdev;
	unsigned long start_jiffies;
	u16 reg16;

	netdev_info(dhd->bh_dev, "dhd retrain request for PCIe device %04x:%02x:%02x.%01x\n",
		    req->domain, req->bus, req->devid, req->function);
	if (!dev) {
		netdev_err(dhd->bh_dev, "PCIe device %04x:%02x:%02x.%01x not found\n",
			   req->domain, req->bus, req->devid, req->function);
		return -EINVAL;
	}

	if (!dev->bus || !dev->bus->self) {
		netdev_err(dhd->bh_dev, "RC of PCIe device %04x:%02x:%02x.%01x not found\n",
			   req->domain, req->bus, req->devid, req->function);
		return -EINVAL;
	}

	pdev = dev->bus->self;
	pcie_capability_read_word(pdev, PCI_EXP_LNKCTL, &reg16);
	reg16 |= PCI_EXP_LNKCTL_RL;
	pcie_capability_write_word(pdev, PCI_EXP_LNKCTL, reg16);
	if (pdev->clear_retrain_link) {
		/*
		 * Due to an erratum in some devices the Retrain Link bit
		 * needs to be cleared again manually to allow the link
		 * training to succeed.
		 */
		reg16 &= ~PCI_EXP_LNKCTL_RL;
		pcie_capability_write_word(pdev, PCI_EXP_LNKCTL, reg16);
	}

	/* Wait for link training end. Break out after waiting for timeout */
	start_jiffies = jiffies;
	for (;;) {
		pcie_capability_read_word(pdev, PCI_EXP_LNKSTA, &reg16);
		if (!(reg16 & PCI_EXP_LNKSTA_LT))
			break;
		if (time_after(jiffies, start_jiffies + LINK_RETRAIN_TIMEOUT))
			break;
		msleep(1);
	}

	if (reg16 & PCI_EXP_LNKSTA_LT) {
		netdev_err(dhd->bh_dev, "PCIe device %04x:%02x:%02x.%01x retrain timeout\n",
			   req->domain, req->bus, req->devid, req->function);
		return -ETIMEDOUT;
	}

	return 0;
}

static void dhd_bh_cleanup(dhd_bh_info_t *dhd)
{
	struct file *file = dhd->bh_file;

	/* Get rid of the client, if any */
	dhd_bh_stop(dhd);

	/* Remove kernel network interface and free all instance memory */
	unregister_netdev(dhd->bh_dev);
	file->private_data = NULL;
}

/*
 * File operations for miscellaneous device
 */
static int
dhd_open(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static int
dhd_release(struct inode *inode, struct file *file)
{
	dhd_bh_info_t *bh = file->private_data;

	if (bh != NULL) {
		dhd_bh_cleanup(bh);
		file->private_data = NULL;
	}
	return 0;
}

static long
dhd_ioctl(struct file *file, uint ioctl_num, ulong arg)
{
	dhd_bh_info_t *dhd = file->private_data;
	union {
		struct dhd_attach_request attach_req;
		struct dhd_pcie_retrain_request retrain_req;
	} u;
	int rc;

	switch (ioctl_num) {
	case DPDK_DHD_ATTACH:
		/* No instance should be attached already to this file */
		if (dhd != NULL) {
			rc = -EBUSY;
			break;
		}

		/* Get the attach parameters */
		if (copy_from_user(&u.attach_req, (void __user *)arg,
				   sizeof(u.attach_req))) {
			rc = -EFAULT;
			break;
		}
		rc = dhd_bh_attach(file, &u.attach_req);
		if (rc != 0)
			break;
		/* Return attach info back to the caller */
		rc = copy_to_user((void __user *)arg, &u.attach_req,
				  sizeof(u.attach_req)) ? -EFAULT : 0;
		if (rc != 0) {
			dhd_bh_cleanup(dhd);
			break;
		}
		break;
	case DPDK_DHD_START:
		if (dhd == NULL) {
			rc = -EINVAL;
			break;
		}
		rc = dhd_bh_start(dhd);
		break;
	case DPDK_DHD_STOP:
		if (dhd == NULL) {
			rc = -EINVAL;
			break;
		}
		dhd_bh_stop(dhd);
		rc = 0;
		break;
	case DPDK_DHD_PCIE_RETRAIN:
		if (dhd == NULL) {
			rc = -EINVAL;
			break;
		}

		/* Get the retrain parameters */
		if (copy_from_user(&u.retrain_req, (void __user *)arg,
				   sizeof(u.retrain_req))) {
			rc = -EFAULT;
			break;
		}
		rc = dhd_bh_pcie_retrain(dhd, &u.retrain_req);
		break;
	default:
		rc = -ENOTTY;
	}

	return rc;
}

static long
dhd_compat_ioctl(struct file *file, uint ioctl_num, ulong ioctl_param)
{
	dhd_bh_info_t *dhd = file->private_data;

	/* 32 bits app on 64 bits OS are not supported */
	netdev_dbg(dhd->bh_dev, "Not implemented.\n");
	return -EINVAL;
}

static const struct file_operations dhd_fops = {
	.owner = THIS_MODULE,
	.open = dhd_open,
	.release = dhd_release,
	.unlocked_ioctl = dhd_ioctl,
	.compat_ioctl = dhd_compat_ioctl,
};

static struct miscdevice dhd_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DHD_DEVICE,
	.fops = &dhd_fops,
};

static int __init
dhd_init(void)
{
	int rc;

	rc = misc_register(&dhd_misc);
	if (rc != 0)
		pr_err("%s: Misc registration failed\n", __FUNCTION__);
	return rc;
}

static void __exit
dhd_exit(void)
{
	misc_deregister(&dhd_misc);
}

module_init(dhd_init);
module_exit(dhd_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Facebook Inc.");
MODULE_DESCRIPTION("Kernel Module for DPDK dhd devices");
