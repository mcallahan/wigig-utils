/* SPDX-License-Identifier: ISC */
/*
 * Copyright (c) 2018,2020, The Linux Foundation. All rights reserved.
 */

#ifndef WIL6210_RADAR_H
#define WIL6210_RADAR_H

#include "wil6210.h"

/**
 * enum qca_wlan_vendor_attr_radar - Radar related attributes
 *
 * @QCA_WLAN_VENDOR_ATTR_RADAR_FIFO_SIZE: Radar FIFO size, in bytes
 */
enum qca_wlan_vendor_attr_radar {
	QCA_WLAN_VENDOR_ATTR_RADAR_FIFO_SIZE,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_RADAR_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_RADAR_MAX =
		QCA_WLAN_VENDOR_ATTR_RADAR_AFTER_LAST - 1,
};

int wil_rdr_init(struct wil6210_priv *wil);
void wil_rdr_set_pulse_size(struct wil6210_priv *wil,
			    struct wmi_radar_config_select_event *evt);
int wil_rdr_alloc_buffer(struct wil6210_priv *wil, u32 *fifo_size);
void wil_rdr_free_buffer(struct wil6210_priv *wil);
int wil_ioc_memio_rdr_get_data(struct wil6210_priv *wil, void __user *data);
void wil_rdr_uninit(struct wil6210_priv *wil);
void wil_rdr_isr(struct wil6210_priv *wil);

#endif /* WIL6210_RADAR_H */
