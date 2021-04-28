/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2015 Intel Corporation. All rights reserved.
 *   All rights reserved.
 *   Copyright(c) 2019-2020, Facebook, Inc. All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/queue.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <fcntl.h>
#include <inttypes.h>
#include <rte_common.h>
#include <rte_byteorder.h>
#include <rte_cycles.h>

#include <rte_interrupts.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_pci.h>
#include <rte_bus_pci.h>
#include <rte_atomic.h>
#include <rte_branch_prediction.h>
#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_eal.h>
#include <rte_alarm.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_ethdev_pci.h>
#include <rte_string_fns.h>
#include <rte_malloc.h>
#include <rte_dev.h>
#include <rte_kvargs.h>
#include <cmdline_parse_etheraddr.h>

#include "wil6210_ethdev.h"

#ifdef RTE_LIBRTE_WIL6210_DEBUG_INIT
#define PMD_INIT_LOG(level, fmt, args...) \
	RTE_LOG(level, PMD, "%s(): " fmt "\n", __func__, ## args)
#define PMD_INIT_FUNC_TRACE() PMD_INIT_LOG(DEBUG, " >>")
#else
#define PMD_INIT_LOG(level, fmt, args...) do { } while(0)
#define PMD_INIT_FUNC_TRACE() do { } while(0)
#endif

/*
 * DPDK only capable of reflecting single MSI vector, so force driver
 * into that mode. This is just as well, as we only really care about
 * misc interrupt.
 */
static int n_msi = 1;
module_param(n_msi, int, 0444);
MODULE_PARM_DESC(n_msi,
    " Use MSI interrupt: 0 - use INTx, 1 - single, or 3 - (default) ");

bool ftm_mode;
module_param(ftm_mode, bool, 0444);
MODULE_PARM_DESC(ftm_mode, " Set factory test mode, default - false");

#define WIL_MAC_ADDR_ARG "mac-address"
#define WIL_FW_CORE_DUMP_PATH_ARG "fw-core-dump-path"
#define WIL_FW_STR_PATH_ARG "fw-strings"
#define WIL_FW_LOG_PATH_ARG "fw-log-path"
#define WIL_FW_OPAQUE_LOG_ARG "opaque-log"
#define WIL_CRASH_ON_FW_ERR "crash-on-fw-err"
#define WIL_UCODE_LOG_PATH_ARG "ucode-log-path"
#define WIL_UCODE_STR_PATH_ARG "ucode-strings"
#define WIL_FW_LOG_LEVEL_ARG "fw-log-level"
#define WIL_NO_FW_RECOVERY_ARG "no-fw-recovery"
#define WIL_MTU_MAX_ARG "mtu-max"
#define WIL_P2MP_CAPABLE_ARG "p2mp-capable"
#define WIL_NON_COMMERCIAL_RF_ARG "non-commercial-rf"
#define WIL_PCIE_EXPECTED_GEN_ARG "pcie-expected-gen"
#define WIL_PCIE_EXPECTED_LANES_ARG "pcie-expected-lanes"
#define WIL_PMC_EXT_HOST "pmc-ext-host"
#define WIL_PMC_EXT_RING_ORDER "pmc-ext-ring-order"

static const char *const devarg_keys[] = {
	WIL_MAC_ADDR_ARG,
	WIL_FW_CORE_DUMP_PATH_ARG,
	WIL_FW_STR_PATH_ARG,
	WIL_FW_LOG_PATH_ARG,
	WIL_FW_OPAQUE_LOG_ARG,
	WIL_CRASH_ON_FW_ERR,
	WIL_UCODE_LOG_PATH_ARG,
	WIL_UCODE_STR_PATH_ARG,
	WIL_FW_LOG_LEVEL_ARG,
	WIL_NO_FW_RECOVERY_ARG,
	WIL_MTU_MAX_ARG,
	WIL_P2MP_CAPABLE_ARG,
	WIL_NON_COMMERCIAL_RF_ARG,
	WIL_PCIE_EXPECTED_GEN_ARG,
	WIL_PCIE_EXPECTED_LANES_ARG,
	WIL_PMC_EXT_HOST,
	WIL_PMC_EXT_RING_ORDER,
	NULL /* last key must be NULL */
};

static int wil_set_mac_devarg(const char *key __rte_unused, const char *value,
			      void *arg);
static int wil_set_fw_core_dump_path(const char *key __rte_unused,
				     const char *value, void *arg);
static int wil_set_fw_str_path(const char *key __rte_unused, const char *value,
			       void *arg);
static int wil_set_fw_log_path(const char *key __rte_unused, const char *value,
			       void *arg);
static int wil_use_opaque_log(const char *key __rte_unused, const char *value,
			      void *arg);
static int wil_set_crash_on_fw_err(const char *key __rte_unused,
				   const char *value, void *arg);
static int wil_set_ucode_log_path(const char *key __rte_unused,
				  const char *value, void *arg);
static int wil_set_ucode_str_path(const char *key __rte_unused,
				  const char *value, void *arg);
static int wil_set_fw_log_level(const char *key __rte_unused,
				  const char *value, void *arg);
static int wil_set_no_fw_recovery(const char *key __rte_unused,
				  const char *value, void *arg);
static int wil_set_mtu_max(const char *key __rte_unused,
			   const char *value, void *arg);
static int wil_set_p2mp_capable(const char *key __rte_unused,
				const char *value, void *arg);
static int wil_set_non_commercial_rf(const char *key __rte_unused,
				     const char *value, void *arg);
static int wil_set_pcie_expected_gen(const char *key __rte_unused,
				     const char *value, void *arg);
static int wil_set_pcie_expected_lanes(const char *key __rte_unused,
				       const char *value, void *arg);
static int wil_set_pmc_ext_host(const char *key __rte_unused,
				const char *value, void *arg);
static int wil_set_pmc_ext_ring_order(const char *key __rte_unused,
				      const char *value, void *arg);

static const arg_handler_t devarg_handlers[] = {
	&wil_set_mac_devarg,     &wil_set_fw_core_dump_path,
	&wil_set_fw_str_path,    &wil_set_fw_log_path,
	&wil_use_opaque_log,     &wil_set_crash_on_fw_err,
	&wil_set_ucode_log_path, &wil_set_ucode_str_path,
	&wil_set_fw_log_level,   &wil_set_no_fw_recovery,
	&wil_set_mtu_max,	 &wil_set_p2mp_capable,
	&wil_set_non_commercial_rf, &wil_set_pcie_expected_gen,
	&wil_set_pcie_expected_lanes, &wil_set_pmc_ext_host,
	&wil_set_pmc_ext_ring_order,
};

static
int wil_set_capabilities(struct wil6210_priv *wil)
{
	const char *wil_fw_name;
	u32 jtag_id = wil_r(wil, RGF_USER_JTAG_DEV_ID);
	u8 chip_revision = (wil_r(wil, RGF_USER_REVISION_ID) &
			    RGF_USER_REVISION_ID_MASK);
	int platform_capa;
	struct fw_map *iccm_section, *sct;

	bitmap_zero(wil->hw_capa, hw_capa_last);
	bitmap_zero(wil->fw_capabilities, WMI_FW_CAPABILITY_MAX);
	bitmap_zero(wil->platform_capa, WIL_PLATFORM_CAPA_MAX);
	wil->wil_fw_name = ftm_mode ? WIL_FW_NAME_FTM_DEFAULT :
			   WIL_FW_NAME_DEFAULT;
	wil->chip_revision = chip_revision;

	switch (jtag_id) {
	case JTAG_DEV_ID_SPARROW:
		memcpy(fw_mapping, sparrow_fw_mapping,
		       sizeof(sparrow_fw_mapping));
		switch (chip_revision) {
		case REVISION_ID_SPARROW_D0:
			wil->hw_name = "Sparrow D0";
			wil->hw_version = HW_VER_SPARROW_D0;
			wil_fw_name = ftm_mode ? WIL_FW_NAME_FTM_SPARROW_PLUS :
				      WIL_FW_NAME_SPARROW_PLUS;

			if (wil_fw_verify_file_exists(wil, wil_fw_name))
				wil->wil_fw_name = wil_fw_name;
			sct = wil_find_fw_mapping("mac_rgf_ext");
			if (!sct) {
				wil_err(wil, "mac_rgf_ext section not found in fw_mapping\n");
				return -EINVAL;
			}
			memcpy(sct, &sparrow_d0_mac_rgf_ext, sizeof(*sct));
			break;
		case REVISION_ID_SPARROW_B0:
			wil->hw_name = "Sparrow B0";
			wil->hw_version = HW_VER_SPARROW_B0;
			break;
		default:
			wil->hw_name = "Unknown";
			wil->hw_version = HW_VER_UNKNOWN;
			break;
		}
		wil->rgf_fw_assert_code_addr = SPARROW_RGF_FW_ASSERT_CODE;
		wil->rgf_ucode_assert_code_addr = SPARROW_RGF_UCODE_ASSERT_CODE;
		break;
	case JTAG_DEV_ID_TALYN:
		wil->hw_name = "Talyn-MA";
		wil->hw_version = HW_VER_TALYN;
		memcpy(fw_mapping, talyn_fw_mapping, sizeof(talyn_fw_mapping));
		wil->rgf_fw_assert_code_addr = TALYN_RGF_FW_ASSERT_CODE;
		wil->rgf_ucode_assert_code_addr = TALYN_RGF_UCODE_ASSERT_CODE;
		if (wil_r(wil, RGF_USER_OTP_HW_RD_MACHINE_1) &
		    BIT_NO_FLASH_INDICATION)
			set_bit(hw_capa_no_flash, wil->hw_capa);
		wil_fw_name = ftm_mode ? WIL_FW_NAME_FTM_TALYN :
			      WIL_FW_NAME_TALYN;
		if (wil_fw_verify_file_exists(wil, wil_fw_name))
			wil->wil_fw_name = wil_fw_name;
		break;
	case JTAG_DEV_ID_TALYN_MB:
		wil->hw_name = "Talyn-MB";
		wil->hw_version = HW_VER_TALYN_MB;
		memcpy(fw_mapping, talyn_mb_fw_mapping,
		       sizeof(talyn_mb_fw_mapping));
		wil->rgf_fw_assert_code_addr = TALYN_RGF_FW_ASSERT_CODE;
		wil->rgf_ucode_assert_code_addr = TALYN_RGF_UCODE_ASSERT_CODE;
		set_bit(hw_capa_no_flash, wil->hw_capa);
		wil->use_enhanced_dma_hw = true;
		wil->use_rx_hw_reordering = true;
		wil->use_compressed_rx_status = true;
		wil_fw_name = ftm_mode ? WIL_FW_NAME_FTM_TALYN :
			      WIL_FW_NAME_TALYN;
		if (wil_fw_verify_file_exists(wil, wil_fw_name))
			wil->wil_fw_name = wil_fw_name;
		break;
	default:
		wil_err(wil, "Unknown board hardware, chip_id 0x%08x, chip_revision 0x%08x\n",
			jtag_id, chip_revision);
		wil->hw_name = "Unknown";
		wil->hw_version = HW_VER_UNKNOWN;
		return -EINVAL;
	}

	wil_init_txrx_ops(wil);

	iccm_section = wil_find_fw_mapping("fw_code");
	if (!iccm_section) {
		wil_err(wil, "fw_code section not found in fw_mapping\n");
		return -EINVAL;
	}
	wil->iccm_base = iccm_section->host;

	wil_info(wil, "Board hardware is %s, flash %sexist\n", wil->hw_name,
		 test_bit(hw_capa_no_flash, wil->hw_capa) ? "doesn't " : "");

	/* Get platform capabilities */
	if (wil->platform_ops.get_capa) {
		platform_capa =
			wil->platform_ops.get_capa(wil->platform_handle);
		memcpy(wil->platform_capa, &platform_capa,
		       min(sizeof(wil->platform_capa), sizeof(platform_capa)));
	}

	wil_info(wil, "platform_capa 0x%lx\n", *wil->platform_capa);

	/* extract FW capabilities from file without loading the FW */
	wil_request_firmware(wil, wil->wil_fw_name, false);
	wil_refresh_fw_capabilities(wil);

	return 0;
}

void wil_disable_irq(struct wil6210_priv *wil)
{
#ifndef WIL6210_PMD
	int irq = wil->pdev->irq;

	disable_irq(irq);
	if (wil->n_msi == 3) {
		disable_irq(irq + 1);
		disable_irq(irq + 2);
	}
#endif
}

void wil_enable_irq(struct wil6210_priv *wil)
{
#ifndef WIL6210_PMD
	int irq = wil->pdev->irq;

	enable_irq(irq);
	if (wil->n_msi == 3) {
		enable_irq(irq + 1);
		enable_irq(irq + 2);
	}
#endif
}

/*
 * Private structure per PCI device. Add all necessary bits to emulate
 * enough of wmi and linux functionality to make common rx/rx code
 * happy.
 */
struct wil6210_pci_priv
{
	struct wil6210_priv wil;
	struct wil6210_vif vif;
	struct net_device ndev;
	struct wiphy wiphy;
	struct rte_eth_dev *eth_dev;
};

static int
wil_set_mac_devarg(const char *key __rte_unused, const char *value,
	void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	struct net_device *ndev = wil->main_ndev;
	uint8_t mac[ETH_ALEN];
	int rc;

	rc = cmdline_parse_etheraddr(NULL, value, mac, ETH_ALEN);

	if (rc < 0 || !is_valid_ether_addr(mac))
		return -1;

	ether_addr_copy(ndev->dev_addr, mac);
	return 0;
}

static int
wil_set_fw_core_dump_path(const char *key __rte_unused, const char *value,
	void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	int ret;

	ret = snprintf(wil->fw_core_dump_path, WIL_FW_FILE_PATH_BUFSZ,
		"%s", value);
	/* check for string error or truncation */
	if (ret < 0 || ret >= WIL_FW_FILE_PATH_BUFSZ) {
		wil->fw_core_dump_path[0] = '\0';
		return -1;
	}
	return 0;
}

static int
wil_set_fw_str_path(const char *key __rte_unused, const char *value,
	void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	int ret;

	ret = snprintf(wil->fw_str_path, WIL_FW_FILE_PATH_BUFSZ,
		"%s", value);
	/* check for string error or truncation */
	if (ret < 0 || ret >= WIL_FW_FILE_PATH_BUFSZ) {
		wil->fw_str_path[0] = '\0';
		return -1;
	}
	return 0;
}

static int
wil_set_fw_log_path(const char *key __rte_unused, const char *value,
	void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	int ret;

	ret = snprintf(wil->fw_log_path, WIL_FW_FILE_PATH_BUFSZ,
		"%s", value);
	/* check for string error or truncation */
	if (ret < 0 || ret >= WIL_FW_FILE_PATH_BUFSZ) {
		wil->fw_log_path[0] = '\0';
		return -1;
	}
	return 0;
}

static int
wil_use_opaque_log(const char *key __rte_unused, const char *value, void * arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	/* log path must be specified to copy opaque logs */
	if (strcmp(value, "1") == 0 && strlen(wil->fw_log_path) > 0) {
		wil->opaque_log = true;
		return 0;
	}
	return 1;
}

static int
wil_set_crash_on_fw_err(const char *key __rte_unused, const char *value,
			void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	if (strcmp(value, "0") == 0) {
		wil->crash_on_fw_err = false;
		return 0;
	} else if (strcmp(value, "1") == 0) {
		wil->crash_on_fw_err = true;
		return 0;
	}
	return 1;
}

static int
wil_set_ucode_log_path(const char *key __rte_unused, const char *value,
		       void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	int ret;

	ret = snprintf(wil->ucode_log_path, WIL_FW_FILE_PATH_BUFSZ,
		"%s", value);
	/* check for string error or truncation */
	if (ret < 0 || ret >= WIL_FW_FILE_PATH_BUFSZ) {
		wil->ucode_log_path[0] = '\0';
		return -1;
	}
	return 0;
}

static int
wil_set_ucode_str_path(const char *key __rte_unused, const char *value,
		       void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	int ret;

	ret = snprintf(wil->ucode_log_path, WIL_FW_FILE_PATH_BUFSZ,
		"%s", value);
	/* check for string error or truncation */
	if (ret < 0 || ret >= WIL_FW_FILE_PATH_BUFSZ) {
		wil->ucode_str_path[0] = '\0';
		return -1;
	}
	return 0;
}

static int
wil_set_fw_log_level(const char *key __rte_unused, const char *value, void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	long val;

	errno = 0;
	val = strtol(value, NULL, 0);
	if (errno != 0 || val < 0) {
		return -1;
	}

	if (val > 4) {
		wil->fw_log_level = 4;
	} else {
		wil->fw_log_level = val;
	}
	return 0;
}

static int wil_set_no_fw_recovery(const char *key __rte_unused,
				  const char *value, void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	if (strcmp(value, "0") == 0) {
		wil->no_fw_recovery = false;
		return 0;
	} else if (strcmp(value, "1") == 0) {
		wil->no_fw_recovery = true;
		return 0;
	}
	return 1;
}

static int wil_set_mtu_max(const char *key __rte_unused,
			   const char *value, void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	unsigned long val;

	errno = 0;
	val = strtoul(value, NULL, 0);
	if (errno != 0)
		return -1;

	if (val == 0)
		return 0; /* skip */

	if (val < 68 || val > WIL_MAX_ETH_MTU)
		return -1;

	wil->mtu_max = val;
	return 0;
}

static int
wil_set_p2mp_capable(const char *key __rte_unused, const char *value,
		     void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;

	if (strcmp(value, "0") == 0) {
		wil->p2mp_capable = false;
		return 0;
	} else if (strcmp(value, "1") == 0) {
		wil->p2mp_capable = true;
		return 0;
	}
	return 1;
}

static int
wil_set_non_commercial_rf(const char *key __rte_unused, const char *value,
			  void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;

	if (strcmp(value, "0") == 0) {
		wil->non_commercial_rf = false;
		return 0;
	} else if (strcmp(value, "1") == 0) {
		wil->non_commercial_rf = true;
		return 0;
	}
	return 1;
}

static int
wil_set_pcie_expected_gen(const char *key __rte_unused, const char *value,
			  void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	long val;

	errno = 0;
	val = strtol(value, NULL, 0);
	if (errno != 0 || val < 0) {
		return -1;
	}

	wil->pcie_expected_gen = val;
	return 0;
}

static int
wil_set_pcie_expected_lanes(const char *key __rte_unused, const char *value,
			    void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	long val;

	errno = 0;
	val = strtol(value, NULL, 0);
	if (errno != 0 || val < 0) {
		return -1;
	}

	wil->pcie_expected_lanes = val;
	return 0;
}

static int
wil_set_pmc_ext_host(const char *key __rte_unused, const char *value, void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;

	if (strcmp(value, "0") == 0) {
		wil->pmc_ext_host = false;
		return 0;
	} else if (strcmp(value, "1") == 0) {
		wil->pmc_ext_host = true;
		return 0;
	}

	return -1;
}

static int
wil_set_pmc_ext_ring_order(const char *key __rte_unused, const char *value,
			   void *arg)
{
	struct wil6210_priv *wil = (struct wil6210_priv *)arg;
	unsigned long val;

	errno = 0;
	val = strtoul(value, NULL, 0);
	if (errno != 0 || val > U16_MAX)
		return -1;

	wil->pmc_ext_ring_order = val;
	return 0;
}

static void wil_process_devargs(struct rte_eth_dev *eth_dev)
{
	struct rte_kvargs *kvlist;
	int i, rc;
	const char *devarg;
	arg_handler_t handler;
	struct wil6210_pci_priv *pci_priv = eth_dev->data->dev_private;
	struct device *dev = &pci_priv->ndev.dev;
	struct wil6210_priv *wil = &pci_priv->wil;

	if (!eth_dev->device->devargs) {
		dev_dbg(dev, "no devargs to process\n");
		return;
	}

	kvlist = rte_kvargs_parse(eth_dev->device->devargs->args, devarg_keys);
	if (kvlist == NULL) {
		dev_err(dev, "error parsing devargs\n");
		return;
	}

	/* last key is always NULL */
	for (i = 0; i < ARRAY_SIZE(devarg_keys) - 1; i++) {
		devarg = devarg_keys[i];
		handler = devarg_handlers[i];

		if (rte_kvargs_count(kvlist, devarg) > 0) {
			rc = rte_kvargs_process(kvlist, devarg, handler, wil);
			if (rc) {
				dev_err(dev, "error processing devarg: %s\n", devarg);
			} else {
				dev_dbg(dev, "processed devarg: %s\n", devarg);
			}
		}
	}

	rte_kvargs_free(kvlist);
}

/* Bus ops */
static int wil_if_pcie_enable(struct wil6210_priv *wil)
{
	int rc = 0;

	wil_dbg_misc(wil, "if_pcie_enable\n");

	/* how many MSI interrupts to request? */
	switch (n_msi) {
	case 3:
	case 1:
		wil_dbg_misc(wil, "Setup %d MSI interrupts\n", n_msi);
		break;
	case 0:
		wil_dbg_misc(wil, "MSI interrupts disabled, use INTx\n");
		break;
	default:
		wil_err(wil, "Invalid n_msi=%d, default to 1\n", n_msi);
		n_msi = 1;
	}

	wil->n_msi = n_msi;

	/* need reset here to obtain MAC */
	mutex_lock(&wil->mutex);
	rc = wil_reset(wil, false);
	mutex_unlock(&wil->mutex);
	return rc;
}

static int wil_if_pcie_disable(struct wil6210_priv *wil)
{
	//struct rte_pci_device *pdev = wil->pdev;

	wil_dbg_misc(wil, "if_pcie_disable\n");

	//pci_clear_master(pdev);
	/* disable and release IRQ */
	wil6210_fini_irq(wil, 0);
	/* safe to call if no MSI */
	//pci_disable_msi(pdev);
	/* TODO: disable HW */
	return 0;
}

static int wil_platform_rop_ramdump(void *wil_handle, void *buf, uint32_t size)
{
	struct wil6210_priv *wil = wil_handle;

	if (!wil)
		return -EINVAL;

	return wil_fw_copy_crash_dump(wil, 0, 0);
}

static int wil_platform_rop_fw_recovery(void *wil_handle)
{
	struct wil6210_priv *wil = wil_handle;

	if (!wil)
		return -EINVAL;

	if (wil->crash_on_fw_err) {
		wil_err(wil, "crash-on-fw-err=1, crashing...\n");
		return -ECANCELED;
	}

	wil_fw_error_recovery(wil);
	return 0;
}

void wil_pci_linkdown_recovery_worker(struct work_struct *work)
{
#ifndef WIL6210_PMD
	struct wil6210_priv *wil = container_of(work, struct wil6210_priv,
						pci_linkdown_recovery_worker);
	int rc, i;
	struct wil6210_vif *vif;
	struct net_device *ndev = wil->main_ndev;

	wil_dbg_misc(wil, "starting pci_linkdown recovery\n");

	rtnl_lock();
	mutex_lock(&wil->mutex);
	down_write(&wil->mem_lock);
	clear_bit(wil_status_fwready, wil->status);
	set_bit(wil_status_pci_linkdown, wil->status);
	set_bit(wil_status_resetting, wil->status);
	up_write(&wil->mem_lock);

	if (test_and_clear_bit(wil_status_napi_en, wil->status)) {
		napi_disable(&wil->napi_rx);
		napi_disable(&wil->napi_tx);
	}

	mutex_unlock(&wil->mutex);
	rtnl_unlock();

	mutex_lock(&wil->mutex);

	mutex_lock(&wil->vif_mutex);
	wil_ftm_stop_operations(wil);
	wil_p2p_stop_radio_operations(wil);
	wil_abort_scan_all_vifs(wil, false);
	mutex_unlock(&wil->vif_mutex);

	for (i = 0; i < wil->max_vifs; i++) {
		vif = wil->vifs[i];
		if (vif) {
			cancel_work_sync(&vif->disconnect_worker);
			wil6210_disconnect(vif, NULL,
					   WLAN_REASON_DEAUTH_LEAVING);
		}
	}

	wmi_event_flush(wil);
	flush_workqueue(wil->wq_service);
	flush_workqueue(wil->wmi_wq);

	/* Recover PCIe */
	if (wil->platform_ops.pci_linkdown_recovery) {
		rc = wil->platform_ops.pci_linkdown_recovery(
			wil->platform_handle);
		if (rc) {
			wil_err(wil,
				"platform device failed to recover from pci linkdown (%d)\n",
				rc);
			mutex_unlock(&wil->mutex);
			goto out;
		}
	} else {
		wil_err(wil,
			"platform device doesn't support pci_linkdown recovery\n");
		mutex_unlock(&wil->mutex);
		goto out;
	}

	if (!ndev || !(ndev->flags & IFF_UP)) {
		wil_reset(wil, false);
		mutex_unlock(&wil->mutex);
	} else {
		mutex_unlock(&wil->mutex);
		wil->recovery_state = fw_recovery_pending;
		wil_fw_recovery(wil);
	}

out:
#endif /* WIL6210_PMD */
	return;
}

static int wil_platform_rop_notify(void *wil_handle,
				   enum wil_platform_notif notif)
{
	struct wil6210_priv *wil = wil_handle;

	if (!wil)
		return -EINVAL;

	switch (notif) {
	case WIL_PLATFORM_NOTIF_PCI_LINKDOWN:
		wil_info(wil, "received WIL_PLATFORM_NOTIF_PCI_LINKDOWN\n");
		clear_bit(wil_status_fwready, wil->status);
		set_bit(wil_status_resetting, wil->status);
		set_bit(wil_status_pci_linkdown, wil->status);

		if (wil->fw_state == WIL_FW_STATE_READY)
			wil_nl_60g_fw_state_change(wil,
						   WIL_FW_STATE_ERROR);
		else
			wil_nl_60g_fw_state_change(
				wil, WIL_FW_STATE_ERROR_BEFORE_READY);

		//  schedule_work(&wil->pci_linkdown_recovery_worker);
		break;
	default:
		break;
	}

	return 0;
}

static void wil_platform_ops_uninit(struct wil6210_priv *wil)
{
	if (wil->platform_ops.uninit)
		wil->platform_ops.uninit(wil->platform_handle);
	memset(&wil->platform_ops, 0, sizeof(wil->platform_ops));
}

/*
 * It returns 0 on success.
 */
static int
wil6210_dev_init(struct rte_eth_dev *eth_dev)
{
	struct device *dev;
	struct rte_pci_device *pdev;
	struct wil6210_pci_priv *pci_priv = eth_dev->data->dev_private;
	struct wil6210_priv *wil = &pci_priv->wil;
	const struct wil_platform_rops rops = {
		.ramdump = wil_platform_rop_ramdump,
		.fw_recovery = wil_platform_rop_fw_recovery,
		.notify = wil_platform_rop_notify,
	};
	u32 bar_size;
	int rc;

	PMD_INIT_FUNC_TRACE();

	pdev = RTE_ETH_DEV_TO_PCI(eth_dev);

	wil->pdev = pdev;
	wil->main_ndev = &pci_priv->ndev;
	wil->wiphy = &pci_priv->wiphy;
	wil->port_id = eth_dev->data->port_id;

	/*
	 * Populate names in compatibility objects, to make logs
	 * more informative.
	 */
	dev = &pci_priv->ndev.dev;
	snprintf(dev->name, sizeof(dev->name), "%s", pdev->device.name);
	snprintf(pci_priv->ndev.name, sizeof(pci_priv->ndev.name), "%s",
		 eth_dev->data->name);

	pci_priv->eth_dev = eth_dev;
	pci_priv->ndev.wil = wil;
	pci_priv->ndev.vif = &pci_priv->vif;
	pci_priv->ndev.ieee80211_ptr = &pci_priv->vif.wdev;
	pci_priv->wiphy.dev = &pci_priv->ndev.dev;
	pci_priv->wiphy.priv = wil;
	pci_priv->vif.wdev.priv = wil;
	pci_priv->vif.wdev.wiphy = wil->wiphy;

	rc = wil_priv_init(wil);
	if (rc) {
		dev_err(dev, "wil_priv_init failed: %d\n", rc);
		return rc;
	}
	wil_process_devargs(eth_dev);

	bar_size = pdev->mem_resource[0].len;
	if ((bar_size < WIL6210_MIN_MEM_SIZE) ||
	    (bar_size > WIL6210_MAX_MEM_SIZE)) {
		dev_err(dev, "Unexpected BAR0 size 0x%x\n",
			bar_size);
		rc = -ENODEV;
		goto wil_deinit;
	}

	wil = wil_if_alloc(wil);
	if (IS_ERR(wil)) {
		dev_err(dev, "Unable to init device data\n");
		rc = -ENODEV;
		goto wil_deinit;
	}

	wil->csr = pdev->mem_resource[0].addr;
	wil->bar_size = bar_size;

	wil_info(wil, "CSR at 0x0x%" PRIXPTR ", BAR size %u\n",
		 (uintptr_t)wil->csr, bar_size);

	wil->platform_handle =
		wil_platform_init(dev, &wil->platform_ops, &rops, wil);
	if (!wil->platform_handle) {
		rc = -ENODEV;
		wil_err(wil, "wil_platform_init failed\n");
		goto if_free;
	}

	rc = wil_set_capabilities(wil);
	if (rc != 0) {
		goto if_free;
	}

	rc = wil_if_pcie_enable(wil);
	if (rc != 0) {
		goto if_free;
	}

	wil_clear_fw_log_addr(wil);

	/* in case of WMI-only FW, perform full reset and FW loading */
	/* do FW loading in in DPDK case as well, no reason to wait */
	if (test_bit(WMI_FW_CAPABILITY_WMI_ONLY, wil->fw_capabilities)) {
		wil_dbg_misc(wil, "Loading WMI only FW\n");
		mutex_lock(&wil->mutex);
		rc = wil_reset(wil, true);
		mutex_unlock(&wil->mutex);
		if (rc) {
			wil_err(wil, "failed to load WMI only FW\n");
			goto if_free;
		}
	}

	wil_fw_log_polling_init(wil);

	return wil6210_eth_dev_init(eth_dev);
if_free:
	wil_platform_ops_uninit(wil);
wil_deinit:
	wil_priv_deinit(wil);
	return rc;
}

int
wil6210_dev_uninit(struct rte_eth_dev *eth_dev)
{
	struct wil6210_pci_priv *pci_priv = eth_dev->data->dev_private;
	struct wil6210_priv *wil = &pci_priv->wil;

	wil_dbg_misc(wil, "pcie_remove\n");

	wil_platform_ops_uninit(wil);
	wil->main_ndev = NULL;

	return wil6210_eth_dev_stop(eth_dev);
}

static int eth_wil6210_pci_probe(struct rte_pci_driver *pci_drv __rte_unused,
	struct rte_pci_device *pci_dev)
{
	return rte_eth_dev_pci_generic_probe(pci_dev, sizeof(struct wil6210_pci_priv),
	    wil6210_dev_init);
}

static int eth_wil6210_pci_remove(struct rte_pci_device *pci_dev)
{
	return rte_eth_dev_pci_generic_remove(pci_dev, wil6210_dev_uninit);
}

/*
 * The set of PCI devices this driver supports
 */
static const struct rte_pci_id pci_id_wil6210_map[] = {
	{ RTE_PCI_DEVICE(0x17cb, 0x1201) }, /* Talyn */
	{ .vendor_id = 0, /* sentinel */ },
};

static struct rte_pci_driver rte_wil6210_pmd = {
	.id_table = pci_id_wil6210_map,
	.drv_flags = RTE_PCI_DRV_NEED_MAPPING | RTE_PCI_DRV_INTR_LSC |
	    RTE_PCI_DRV_IOVA_AS_VA,
	.probe = eth_wil6210_pci_probe,
	.remove = eth_wil6210_pci_remove,
};

RTE_PMD_REGISTER_PCI(net_wil6210, rte_wil6210_pmd);
RTE_PMD_REGISTER_PCI_TABLE(net_wil6210, pci_id_wil6210_map);
RTE_PMD_REGISTER_KMOD_DEP(net_wil6210,
    "* igb_uio | uio_pci_generic | vfio-pci");
