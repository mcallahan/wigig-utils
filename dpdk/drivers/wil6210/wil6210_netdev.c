// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2012-2017 Qualcomm Atheros, Inc.
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019, Facebook, Inc. All rights reserved.
 */

#include "wil6210_ethdev.h"
#include "wil6210_nl60g.h"
#include "txrx.h"
#if defined(CONFIG_WIL6210_NSS_SUPPORT)
#include <nss_api_if.h>
#endif
#include "slave_i.h"

u8 slave_mode = 2;
module_param(slave_mode, byte, 0444);
MODULE_PARM_DESC(slave_mode,
		 " slave mode: 0=disabled,1=partial,2=full (default: 0)");

bool ac_queues; /* = false; */
module_param(ac_queues, bool, 0444);
MODULE_PARM_DESC(ac_queues, " enable access category for transmit packets. default false");

bool q_per_sta; /* = false; */
module_param(q_per_sta, bool, 0444);
MODULE_PARM_DESC(q_per_sta, " enable allocating tx queue(s) per station. default false");

bool wil_has_other_active_ifaces(struct wil6210_priv *wil,
				 struct net_device *ndev, bool up, bool ok)
{
#ifndef WIL6210_PMD
	int i;
	struct wil6210_vif *vif;
	struct net_device *ndev_i;

	for (i = 0; i < GET_MAX_VIFS(wil); i++) {
		vif = wil->vifs[i];
		if (vif) {
			ndev_i = vif_to_ndev(vif);
			if (ndev_i != ndev)
				if ((up && (ndev_i->flags & IFF_UP)) ||
				    (ok && netif_carrier_ok(ndev_i)))
					return true;
		}
	}
#endif

	return false;
}

bool wil_has_active_ifaces(struct wil6210_priv *wil, bool up, bool ok)
{
	/* use NULL ndev argument to check all interfaces */
	return wil_has_other_active_ifaces(wil, NULL, up, ok);
}

static void wil_vif_deinit(struct wil6210_vif *vif)
{
	//wil_ftm_deinit(vif);
	//del_timer_sync(&vif->scan_timer);
	//del_timer_sync(&vif->p2p.discovery_timer);
	//cancel_work_sync(&vif->disconnect_worker);
	//cancel_work_sync(&vif->p2p.discovery_expired_work);
	//cancel_work_sync(&vif->p2p.delayed_listen_work);
	//wil_probe_client_flush(vif);
	//cancel_work_sync(&vif->probe_client_worker);
}

void wil_vif_free(struct wil6210_vif *vif)
{
	//struct net_device *ndev = vif_to_ndev(vif);

	wil_vif_deinit(vif);
	//free_netdev(ndev);
}

#ifndef WIL6210_PMD
static void wil_ndev_destructor(struct net_device *ndev)
{
	struct wil6210_vif *vif = ndev_to_vif(ndev);

	wil_vif_deinit(vif);
}
#endif

static void wil_vif_init(struct wil6210_vif *vif)
{
	vif->bcast_ring = -1;
	vif->net_queue_stopped = 1;
}

static u8 wil_vif_find_free_mid(struct wil6210_priv *wil)
{
	u8 i;

	for (i = 0; i < GET_MAX_VIFS(wil); i++) {
		if (!wil->vifs[i])
			return i;
	}

	return U8_MAX;
}

struct wil6210_vif *
wil_vif_alloc(struct wil6210_priv *wil, const char *name,
	      unsigned char name_assign_type, enum nl80211_iftype iftype)
{
	struct net_device *ndev;
	struct wireless_dev *wdev;
	struct wil6210_vif *vif;
	u8 mid;

	mid = wil_vif_find_free_mid(wil);
	if (mid == U8_MAX) {
		wil_err(wil, "no available virtual interface\n");
		return ERR_PTR(-EINVAL);
	}

	if (mid == 0) {
		ndev = wil->main_ndev;
	} else {
		ASSERT(0);
	}

	vif = ndev_to_vif(ndev);
	vif->ndev = ndev;
	vif->wil = wil;
	vif->mid = mid;
	wil_vif_init(vif);

	wdev = &vif->wdev;
	wdev->wiphy = wil->wiphy;
	wdev->iftype = iftype;

	ndev->ieee80211_ptr = wdev;
	wdev->netdev = ndev;
	return vif;
}

#define NET_NAME_UNKNOWN 0
#define WLAN_REASON_DEAUTH_LEAVING 0

void *wil_if_alloc(struct wil6210_priv *wil)
{
	struct wil6210_vif *vif;
	struct device *dev;
	int rc = 0;

	dev = wil->wiphy->dev;

	rc = wil_priv_init(wil);
	if (rc) {
		dev_err(dev, "wil_priv_init failed\n");
		goto out_cfg;
	}
	wil_dbg_misc(wil, "if_alloc\n");

	vif = wil_vif_alloc(wil, "wlan%d", NET_NAME_UNKNOWN,
			    NL80211_IFTYPE_STATION);
	if (IS_ERR(vif)) {
		dev_err(dev, "wil_vif_alloc failed\n");
		rc = -ENOMEM;
		goto out_priv;
	}

	return wil;

out_priv:
	wil_priv_deinit(wil);

out_cfg:
#ifndef WIL6210_PMD
	wil_cfg80211_deinit(wil);
#endif

	return ERR_PTR(rc);
}

void wil_if_free(struct wil6210_priv *wil)
{
	struct net_device *ndev = wil->main_ndev;

	wil_dbg_misc(wil, "if_free\n");

	if (!ndev)
		return;

	wil_priv_deinit(wil);

#ifndef WIL6210_PMD
	wil_ndev_destructor(ndev);
	free_netdev(ndev);
#endif
}

int wil_vif_add(struct wil6210_priv *wil, struct wil6210_vif *vif)
{
	struct net_device *ndev = vif_to_ndev(vif);
	struct wireless_dev *wdev = vif_to_wdev(vif);
	bool any_active = wil_has_active_ifaces(wil, true, false);
	int rc = 0;


	ASSERT_RTNL();

	if (wil->vifs[vif->mid]) {
		dev_err(&ndev->dev, "VIF with mid %d already in use\n",
			vif->mid);
		return -EEXIST;
	}
	if (any_active && vif->mid != 0) {
		rc = wmi_port_allocate(wil, vif->mid, ndev->dev_addr,
				       wdev->iftype);
		if (rc)
			return rc;
	}
#ifndef WIL6210_PMD
	rc = register_netdevice(ndev);

	if (rc < 0) {
		dev_err(&ndev->dev, "Failed to register netdev: %d\n", rc);
		if (any_active && vif->mid != 0)
			wmi_port_delete(wil, vif->mid);
		return rc;
	}

	if (wil->umac_handle) {
		vif->umac_vap = wil->umac_ops.vap_add(wil->umac_handle, vif,
						      ndev);
		if (!vif->umac_vap) {
			unregister_netdevice(ndev);
			if (any_active && vif->mid != 0)
				wmi_port_delete(wil, vif->mid);
			return -ENOMEM;
		}
	}
#endif
	wil->vifs[vif->mid] = vif;
	return 0;
}

int wil_if_add(struct wil6210_priv *wil)
{
#ifndef WIL6210_PMD
	struct wiphy *wiphy = wil->wiphy;
	struct net_device *ndev = wil->main_ndev;
	struct wil6210_vif *vif = ndev_to_vif(ndev);
	int rc;

	wil_dbg_misc(wil, "entered");

	strlcpy(wiphy->fw_version, wil->fw_version, sizeof(wiphy->fw_version));

	rc = wiphy_register(wiphy);
	if (rc < 0) {
		wil_err(wil, "failed to register wiphy, err %d\n", rc);
		return rc;
	}

	init_dummy_netdev(&wil->napi_ndev);
	if (wil->use_enhanced_dma_hw) {
		netif_napi_add(&wil->napi_ndev, &wil->napi_rx,
			       wil6210_netdev_poll_rx_edma,
			       WIL6210_NAPI_BUDGET);
		netif_napi_add(&wil->napi_ndev,
			       &wil->napi_tx, wil6210_netdev_poll_tx_edma,
			       WIL6210_NAPI_BUDGET);
	} else {
		netif_napi_add(&wil->napi_ndev, &wil->napi_rx,
			       wil6210_netdev_poll_rx,
			       WIL6210_NAPI_BUDGET);
		netif_napi_add(&wil->napi_ndev,
			       &wil->napi_tx, wil6210_netdev_poll_tx,
			       WIL6210_NAPI_BUDGET);
	}

	wil_update_net_queues_bh(wil, vif, NULL, true);

	if (umac_mode) {
		wil->umac_handle = wil_umac_register(wil);
		if (!wil->umac_handle) {
			wil_err(wil, "wil_umac_register failed\n");
			rc = -ENOMEM;
			goto out_wiphy;
		}
	}

	rtnl_lock();
	rc = wil_vif_add(wil, vif);
	rtnl_unlock();
	if (rc < 0)
		goto umac_unreg;
	if (slave_mode) {
		rc = wil_register_slave(wil);
		if (rc)
			wil_err(wil, "failed to register slave, err %d\n", rc);
		/* continue even if failed */
	}
#else
	struct net_device *ndev = wil->main_ndev;
	struct wil6210_vif *vif = ndev_to_vif(ndev);
	int rc;

	rc = wil_vif_add(wil, vif);
	if (rc < 0)
		return rc;

	if (wil->nl60g) {
		rc = nl60g_start(wil->nl60g, wil, wil->pdev->addr);
		if (rc != 0)
			wil_err(wil, "failed to start nl60g, continue without\n");
	}

	if (slave_mode) {
		rc = wil_register_slave(wil);
		if (rc) {
			if (wil->nl60g)
				nl60g_stop(wil->nl60g);
			wil_err(wil, "failed to register slave, err %d\n", rc);
		}
		/* continue even if failed?? */
	}
#endif

	return rc;
#ifndef WIL6210_PMD
umac_unreg:
	wil_umac_unregister(wil);
out_wiphy:
	wiphy_unregister(wiphy);
	return rc;
#endif
}

void wil_vif_remove(struct wil6210_priv *wil, u8 mid)
{
	struct wil6210_vif *vif;
#ifndef WIL6210_PMD
	struct net_device *ndev;
#endif
	bool any_active = wil_has_active_ifaces(wil, true, false);

	ASSERT_RTNL();
	if (mid >= GET_MAX_VIFS(wil)) {
		wil_err(wil, "invalid MID: %d\n", mid);
		return;
	}

	vif = wil->vifs[mid];
	if (!vif) {
		wil_err(wil, "MID %d not registered\n", mid);
		return;
	}

#if defined(CONFIG_WIL6210_NSS_SUPPORT)
	if (vif->nss_handle)
		nss_virt_if_destroy_sync(vif->nss_handle);
#endif

	mutex_lock(&wil->mutex);
	wil6210_disconnect(vif, NULL, WLAN_REASON_DEAUTH_LEAVING);
	mutex_unlock(&wil->mutex);

#ifndef WIL6210_PMD
	if (vif->umac_vap) {
		wil->umac_ops.vap_remove(vif->umac_vap);
		vif->umac_vap = NULL;
	}

	ndev = vif_to_ndev(vif);
	/* during unregister_netdevice cfg80211_leave may perform operations
	 * such as stop AP, disconnect, so we only clear the VIF afterwards
	 */
	unregister_netdevice(ndev);
#endif

	if (any_active && vif->mid != 0)
		wmi_port_delete(wil, vif->mid);

	/* make sure no one is accessing the VIF before removing */
	mutex_lock(&wil->vif_mutex);
	wil->vifs[mid] = NULL;
	/* ensure NAPI code will see the NULL VIF */
	wmb();
#ifndef WIL6210_PMD
	if (test_bit(wil_status_napi_en, wil->status)) {
		napi_synchronize(&wil->napi_rx);
		napi_synchronize(&wil->napi_tx);
	}
#endif
	mutex_unlock(&wil->vif_mutex);

#ifndef WIL6210_PMD
	flush_work(&wil->wmi_event_worker);
	del_timer_sync(&vif->connect_timer);
	cancel_work_sync(&vif->disconnect_worker);
	wil_probe_client_flush(vif);
	cancel_work_sync(&vif->probe_client_worker);
#endif
	/* for VIFs, ndev will be freed by destructor after RTNL is unlocked.
	 * the main interface will be freed in wil_if_free, we need to keep it
	 * a bit longer so logging macros will work.
	 */
}

void wil_if_remove(struct wil6210_priv *wil)
{
	struct net_device *ndev = wil->main_ndev;
	struct wireless_dev *wdev = ndev->ieee80211_ptr;

	wil_dbg_misc(wil, "if_remove\n");

	if (slave_mode)
		wil_unregister_slave(wil);

	if (wil->nl60g)
		nl60g_stop(wil->nl60g);

	rtnl_lock();
	wil_vif_remove(wil, 0);
	rtnl_unlock();

#ifndef WIL6210_PMD
	wil_umac_unregister(wil);

	netif_napi_del(&wil->napi_tx);
	netif_napi_del(&wil->napi_rx);
#endif

	wiphy_unregister(wdev->wiphy);
}
