/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
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
#include <linux/if.h>
#include "wil6210.h"
#include "wmi.h"
#include "slave.h"
#include "slave_i.h"

struct wil_slave_entry {
	struct wil6210_priv *wil;
	void *ctx; /* master driver context */
	struct wil_slave_rops rops;
};

static DEFINE_MUTEX(slave_lock);

#define MAX_SLAVES	2

static struct wil_slave_entry slaves[MAX_SLAVES];

static int find_free_slave_entry(void)
{
	int i;

	for (i = 0; i < MAX_SLAVES; i++)
		if (!slaves[i].wil)
			return i;

	return -ENOENT;
}

int wil_register_slave(struct wil6210_priv *wil)
{
	int i, rc = 0;

	mutex_lock(&slave_lock);

	i = find_free_slave_entry();
	if (i < 0) {
		wil_err(wil, "out of slave entries, %s not added\n",
			wil->main_ndev->name);
		rc = -ENOMEM;
		goto out;
	}
	slaves[i].wil = wil;
	slaves[i].ctx = NULL;
	wil->slave_ctx = &slaves[i];
	wil_info(wil, "added slave entry, interface %s\n",
		 wil->main_ndev->name);
out:
	mutex_unlock(&slave_lock);
	return rc;
}

void wil_unregister_slave(struct wil6210_priv *wil)
{
	int i;

	mutex_lock(&slave_lock);

	for (i = 0; i < MAX_SLAVES; i++) {
		if (slaves[i].wil == wil) {
			slaves[i].wil = NULL;
			wil->slave_ctx = NULL;
			/* TODO unregister the vendor driver if needed */
			goto out;
		}
	}

	wil_err(wil, "failed to remove slave, interface %s\n",
		wil->main_ndev->name);
out:
	mutex_unlock(&slave_lock);
}

static int wil_slave_ioctl(void *dev, u16 code, u8 *req_buf, u16 req_len,
			   u8 *resp_buf, u16 *resp_len)
{
	struct wil_slave_entry *slave = dev;
	struct wil6210_priv *wil = slave->wil;
	struct wil6210_vif *vif = ndev_to_vif(wil->main_ndev);
	struct wmi_internal_fw_ioctl_cmd *cmd;
	struct {
		struct wmi_cmd_hdr wmi;
		struct wmi_internal_fw_ioctl_event evt;
	} __packed * reply;
	u16 cmd_len, reply_len, evt_len;
	int rc;

	wil_dbg_misc(wil, "slave_ioctl, code %d\n", code);

	if (!resp_len)
		return -EINVAL;

	if (req_len > WMI_MAX_IOCTL_PAYLOAD_SIZE) {
		wil_err(wil, "request too large (%d, max %d)\n",
			req_len, WMI_MAX_IOCTL_PAYLOAD_SIZE);
		return -EINVAL;
	}

	cmd_len = sizeof(*cmd) + req_len;
	cmd = kmalloc(cmd_len, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;
	reply_len = sizeof(*reply) + WMI_MAX_IOCTL_REPLY_PAYLOAD_SIZE;
	reply = kmalloc(reply_len, GFP_KERNEL);
	if (!reply) {
		rc = -ENOMEM;
		goto out_cmd;
	}
	cmd->code = cpu_to_le16(code);
	cmd->length = cpu_to_le16(req_len);
	memcpy(cmd->payload, req_buf, req_len);
	memset(reply, 0, sizeof(*reply));
	reply->evt.status = WMI_FW_STATUS_FAILURE;
	rc = wmi_call(wil, WMI_INTERNAL_FW_IOCTL_CMDID, vif->mid,
		      cmd, cmd_len,
		      WMI_INTERNAL_FW_IOCTL_EVENTID, reply, reply_len,
		      WIL6210_FW_RECOVERY_TO);
	if (rc)
		goto out_reply;
	if (reply->evt.status) {
		wil_err(wil, "ioctl failed with status %d\n",
			reply->evt.status);
		rc = -EINVAL;
		goto out_reply;
	}
	evt_len = le16_to_cpu(reply->evt.length);
	if (evt_len > *resp_len) {
		wil_err(wil, "response buffer too short (have %d need %d)\n",
			*resp_len, evt_len);
		rc = -EINVAL;
		goto out_reply;
	}
	memcpy(resp_buf, &reply->evt.payload, evt_len);
	*resp_len = evt_len;
out_reply:
	kfree(reply);
out_cmd:
	kfree(cmd);
	return rc;
}

static struct wil_slave_ops slave_ops = {
	.api_version = WIL_SLAVE_API_VERSION,
	.ioctl = wil_slave_ioctl,
};

void wil_slave_evt_internal_fw_event(struct wil6210_vif *vif,
				     struct wmi_internal_fw_event_event *evt,
				     int len)
{
	struct wil6210_priv *wil = vif_to_wil(vif);
	struct wil_slave_entry *slave = wil->slave_ctx;
	void *master_ctx;

	if (!slave || len < sizeof(struct wmi_internal_fw_event_event))
		return;
	master_ctx = slave->ctx;
	if (!master_ctx) {
		wil_err(wil, "master not registered for interface %s\n",
			wil->main_ndev->name);
		return;
	}
	slave->rops.rx_event(master_ctx, le16_to_cpu(evt->id),
			     (u8 *)evt->payload, le16_to_cpu(evt->length));
}

void *wil_register_master(const char *ifname,
			  struct wil_slave_ops *ops,
			  const struct wil_slave_rops *rops, void *ctx)
{
	int i;
	void *ret;
	struct wil_slave_entry *slave = NULL;
	struct wil6210_priv *wil;
	struct net_device *ndev;

	if (!ifname || !ops || !rops)
		return ERR_PTR(-EINVAL);

	mutex_lock(&slave_lock);
	for (i = 0; i < MAX_SLAVES; i++) {
		if (slaves[i].wil) {
			wil = slaves[i].wil;
			ndev = wil->main_ndev;
			if (!strcmp(ndev->name, ifname)) {
				slave = &slaves[i];
				break;
			}
		}
	}

	if (!slave) {
		ret = ERR_PTR(-ENOENT);
		goto out;
	}
	if (ops->api_version != WIL_SLAVE_API_VERSION) {
		wil_err(wil, "mismatched slave API (expected %d have %d)\n",
			WIL_SLAVE_API_VERSION, ops->api_version);
		ret = ERR_PTR(-EINVAL);
		goto out;
	}

	*ops = slave_ops;
	slave->rops = *rops;
	slave->ctx = ctx;
	wil_info(wil, "registered master for interface %s\n", ifname);
	ret = slave;
out:
	mutex_unlock(&slave_lock);
	return ret;
}
EXPORT_SYMBOL(wil_register_master);

void wil_unregister_master(void *dev)
{
	int i;
	struct wil_slave_entry *slave;
	struct wil6210_priv *wil;

	if (!dev)
		return;

	mutex_lock(&slave_lock);
	slave = dev;
	i = slave - &slaves[0];
	if (i < 0 || i >= MAX_SLAVES)
		goto out;

	wil = slave->wil;
	if (!wil)
		goto out;

	slave->ctx = NULL;
	wil_info(wil, "unregistered master for interface %s\n",
		 wil->main_ndev->name);
out:
	mutex_unlock(&slave_lock);
}
EXPORT_SYMBOL(wil_unregister_master);
