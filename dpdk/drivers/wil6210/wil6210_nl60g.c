/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
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

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdatomic.h>

#include <err.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>

#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>

#include <rte_mbuf.h>
#include <rte_mempool.h>

#include <netlink/netlink.h>
#include <netlink/genl/genl.h>

#include "wil6210_ethdev.h"
#include "wil6210_pmc.h"
#include "wil6210_nl60g.h"

#include <wil6210_compat.h>

/* Compatibility with DPDK */
#ifndef RTE_LOG
#define RTE_LOG(level, type, args...)  fprintf(stderr, args)
#endif

/* directory to place local sockets */
#define NL60G_SOCKET_DIR "/dev/wil6210"

/*
 * definitions from nl80211.h (kernel) and qca-vendor.h
 * (wpa_supplicant). These are duplicated to reduce
 * build dependencies
 */
#define NL80211_ATTR_MAX 290

#define NL80211_CMD_VENDOR 103

#define QCA_NL80211_VENDOR_SUBCMD_UNSPEC 0

#define NL80211_ATTR_WIPHY 1
#define NL80211_ATTR_VENDOR_ID 195
#define NL80211_ATTR_VENDOR_SUBCMD 196
#define NL80211_ATTR_VENDOR_DATA 197

#define OUI_QCA 0x001374
#define QCA_WLAN_VENDOR_ATTR_MAX 33

/*
 * dummy nl80211 familty ID.
 * the family id is not checked, so any value
 * >= NLMSG_MIN_TYPE will work
 */
#define NL80211_FAMILTY_ID NLMSG_MIN_TYPE

/*
 * relevant structures for handling nl60g commands.
 * These must be in sync with the definitions in
 * wil6210/cfg80211.c
 */

enum nl60g_attr_wil {
	QCA_ATTR_FEATURE_FLAGS = 7,
	QCA_ATTR_TEST = 8,
	QCA_ATTR_PAD = 13,
	QCA_ATTR_WIL_MAX,
};

#define WIL_ATTR_60G_CMD_TYPE QCA_ATTR_FEATURE_FLAGS
#define WIL_ATTR_60G_BUF QCA_ATTR_TEST

static const struct
nla_policy nl60g_policy[QCA_ATTR_WIL_MAX + 1] = {
	[WIL_ATTR_60G_CMD_TYPE] = { .type = NLA_U32 },
	[WIL_ATTR_60G_BUF] = { .type = NLA_BINARY },
};

enum nl60g_cmd_type {
	NL_60G_CMD_FW_WMI,
	NL_60G_CMD_GENERIC,
	NL_60G_CMD_STATISTICS,
	NL_60G_CMD_REGISTER,
	NL_60G_CMD_FW_RMI,
	NL_60G_CMD_MEMIO,
	NL_60G_CMD_MEMIO_BLOCK,
	NL_60G_CMD_PMC,
	NL_60G_CMD_MAX,
};

enum nl60g_evt_type {
	NL_60G_EVT_DRIVER_ERROR,
	NL_60G_EVT_FW_ERROR,
	NL_60G_EVT_FW_WMI,
	NL_60G_EVT_DRIVER_SHUTOWN,
	NL_60G_EVT_DRIVER_DEBUG_EVENT,
	NL_60G_EVT_DRIVER_GENERIC,
	NL_60G_EVT_FW_RMI,
};

enum nl60g_generic_evt {
	NL_60G_GEN_EVT_FW_STATE,
};

struct nl60g_generic_event { /* NL_60G_EVT_DRIVER_GENERIC */
	uint32_t evt_id; /* wil_nl_60g_generic_evt */
};

struct nl60g_fw_state_event {
	struct nl60g_generic_event hdr;
	uint32_t fw_state; /* wil_fw_state */
};

enum nl60g_generic_cmd {
	NL_60G_GEN_FORCE_WMI_SEND,
	NL_60G_GEN_RADAR_ALLOC_BUFFER,
	NL_60G_GEN_FW_RESET,
	NL_60G_GEN_GET_DRIVER_CAPA,
	NL_60G_GEN_GET_FW_STATE,
	NL_60G_GEN_AUTO_RADAR_RX_CONFIG, /* automotive radar */
	NL_60G_GEN_GET_STA_INFO,
	NL_60G_GEN_GET_FW_CAPA,
};

enum nl60g_pmc_cmd {
	NL_60G_PMC_ALLOC,
	NL_60G_PMC_FREE,
	NL_60G_PMC_GET_DATA,
	NL_60G_PMC_GET_DESC_DATA,
};

/* this structure has to be packed because buf_len field
 * is mis-aligned. Looks like a mistake in the original definition
 * at wil6210/cfg80211.c, we keep it here for compatibility
 */
struct nl60g_send_receive_wmi {
	uint32_t cmd_id; /* enum wmi_command_id or enum wmi_event_id */
	uint8_t reserved[2];
	uint8_t dev_id; /* mid */
	uint16_t buf_len;
	uint8_t buf[];
} __attribute__((packed));

/* a single station entry for NL_60G_GET_STA_INFO,
 * the index in the array is the CID
 */
struct wil_nl_60g_sta_info_entry {
	uint8_t mac_addr[ETH_ALEN];
	uint16_t aid;
	uint8_t status; /* enum wil_sta_status */
	uint8_t mid;
	uint8_t reserved[2]; /* align to 32 bit structure size */
};

struct wil_nl_60g_sta_info {
	uint32_t num_cids;
	struct wil_nl_60g_sta_info_entry stations[];
};

struct nl60g_memio {
	uint32_t op; /* enum wil_memio_op */
	uint32_t addr; /* should be 32 bit aligned */
	uint32_t val; /* value to write/fill with value read */
};

struct nl60g_memio_block {
	uint32_t op; /* enum wil_memio_op */
	uint32_t addr; /* should be 32 bit aligned */
	uint32_t size; /* must be multiply of 4 */
	uint8_t buf[];
};

enum nl60g_driver_capa {
	NL_60G_DRIVER_CAPA_WMI_OVER_NL, /* NL command for WMI */
	NL_60G_DRIVER_CAPA_FW_STATE, /* notifications of FW state changes */
	/* not used, replaced by MEMIO_WRITE below */
	NL_60G_DRIVER_CAPA_IOCTL_WRITE,
	/* memio using NL, read dword/block */
	NL_60G_DRIVER_CAPA_MEMIO,
	/* memio using NL, write dword/block support */
	NL_60G_DRIVER_CAPA_MEMIO_WRITE,
	/* provides the GET_STA_INFO generic command */
	NL_60G_DRIVER_CAPA_GET_STA_INFO,
	NL_60G_DRIVER_CAPA_PMC_LEGACY_OVER_NL,
	NL_60G_DRIVER_CAPA_PMC_CONTINUOUS,
};

enum qca_wlan_vendor_driver_capa {
	QCA_WLAN_VENDOR_ATTR_DRIVER_CAPA,
};

enum qca_wlan_vendor_driver_fw_state {
	QCA_WLAN_VENDOR_ATTR_DRIVER_FW_STATE, /* wil_fw_state */
};

enum qca_wlan_vendor_nl60g_sta_info {
	QCA_WLAN_VENDOR_ATTR_STA_INFO,
};

enum qca_wlan_vendor_driver_fw_capa {
	/* wil->fw_capabilities bitmap. A variable length
	 * buffer, 32 bit units, little endian
	 */
	QCA_WLAN_VENDOR_ATTR_DRIVER_FW_CAPA,
};

enum qca_wlan_vendor_nl60g_memio {
	QCA_WLAN_VENDOR_ATTR_MEMIO,
	QCA_WLAN_VENDOR_ATTR_MEMIO_BLOCK,
};

/* NL_60G_PMC_GET_DATA and NL_60G_PMC_GET_DESC_DATA reply */
enum qca_wlan_vendor_pmc_get_data {
	/* total amount of bytes currently available for reading.
	 * will be returned when GET_DATA/GET_DESC_DATA is called
	 * with num_bytes == 0
	 */
	QCA_WLAN_VENDOR_ATTR_PMC_DATA_LENGTH,
	/* payload data */
	QCA_WLAN_VENDOR_ATTR_PMC_DATA,
	/* flag: will be set if more data is waiting in the PMC ring.
	 * user should issue more calls to GET_DATA or GET_DESC_DATA to receive
	 * the additional data
	 */
	QCA_WLAN_VENDOR_ATTR_PMC_MORE_DATA,
	/* This attribute stores the minimum amount of data that can be received
	 * in a single call to GET_DATA/GET_DESC_DATA.
	 * When this attribute is set, no data is returned (PMC_DATA will not
	 * be set) and the GET_DATA/GET_DESC_DATA call will fail with -NLE_FAILURE.
	 * The user is expected to retry the call with a bigger buffer.
	 */
	QCA_WLAN_VENDOR_ATTR_PMC_MIN_DATA_LENGTH,
};

struct nl60g_event {
	uint32_t evt_type; /* nl60g_evt_type */
	uint32_t buf_len;
	uint8_t reserved[9];
	uint8_t buf[];
};

/* for commands with sub-commands such as GENERIC and PMC */
struct nl60g_subcmd_hdr {
	uint32_t cmd_id;
};

struct nl60g_generic_force_wmi {
	struct nl60g_subcmd_hdr hdr;
	uint32_t enable;
};


struct nl60g_pmc_alloc {
	struct nl60g_subcmd_hdr hdr; /* contains command id */
	uint32_t num_desc; /* number of descriptors in the PMC ring */
	uint32_t payload_size;
};

/* used for NL_60G_PMC_GET_DATA and NL_60G_PMC_GET_DESC_DATA */
struct nl60g_pmc_get_data {
	struct nl60g_subcmd_hdr hdr; /* contians command id */
	/* number of bytes to read. 0 to get available bytes to read */
	uint32_t num_bytes;
};

struct nl60g_cmd_def {
	enum nl60g_cmd_type cmd;
	/* true if need WIL_ATTR_60G_BUF */
	bool need_buf;
	/* if need_buf is true, minimum expected buffer length */
	uint32_t len;
	/* subcommands if any */
	struct nl60g_subcmd_def *subcmds;
};

struct nl60g_subcmd_def {
	uint32_t cmd_id;
	/* total length needed including the command id, 0 to terminate list */
	uint32_t len;
};

static struct nl60g_subcmd_def nl60g_pmc_cmds[] = {
	{ NL_60G_PMC_ALLOC, sizeof(struct nl60g_pmc_alloc) },
	{ NL_60G_PMC_FREE, sizeof(struct nl60g_subcmd_hdr) },
	{ NL_60G_PMC_GET_DATA, sizeof(struct nl60g_pmc_get_data) },
	{ NL_60G_PMC_GET_DESC_DATA, sizeof(struct nl60g_pmc_get_data) },
	{ 0, 0 },
};

static struct nl60g_cmd_def nl60g_cmds[] = {
	{ NL_60G_CMD_REGISTER, true, sizeof(uint32_t), NULL },
	{ NL_60G_CMD_GENERIC, true, sizeof(struct nl60g_subcmd_hdr), NULL },
	{ NL_60G_CMD_FW_WMI, true,
	  offsetof(struct nl60g_send_receive_wmi, buf), NULL },
	{ NL_60G_CMD_MEMIO, true, sizeof(struct nl60g_memio), NULL },
	{ NL_60G_CMD_MEMIO_BLOCK, true, sizeof(struct nl60g_memio_block),
	  NULL },
	{ NL_60G_CMD_PMC, true, sizeof(struct nl60g_subcmd_hdr), nl60g_pmc_cmds },
	{ NL_60G_CMD_MAX, false, 0, NULL },
};

struct nl60g_state *nl60g_init(void)
{
	struct nl60g_state *nl60g;
	int i;

	nl60g = calloc(1, sizeof(*nl60g));
	if (nl60g == NULL)
		return NULL;
	nl60g->fd = -1;
	nl60g->exit_sockets[0] = nl60g->exit_sockets[1] = -1;

	for (i = 0; i < NL60G_MAX_PORTS; i++) {
		struct nl60g_port *port = &nl60g->ports[i];
		port->fd = -1;
		port->exit_sockets[0] = port->exit_sockets[1] = -1;
		port->nl60g = nl60g;
	}

	pthread_mutex_init(&nl60g->ports_mutex, 0);

	return nl60g;
}

void
nl60g_fini(struct nl60g_state *nl60g)
{
	if (nl60g == NULL)
		return;

	pthread_mutex_destroy(&nl60g->ports_mutex);
	free(nl60g);
}

/**
 * nl callback handler for disabling sequence number checking
 */
static int no_seq_check(struct nl_msg *msg, void *arg)
{
	return NL_OK;
}

/*
 * get local socket fd from nl_sock structure
 * the nl_sock->s_fd is not accessible, so we use
 * the port field of the local address to store the fd
 */
static int nl60g_get_local_sock_fd(struct nl_sock *sk)
{
	uint32_t port = nl_socket_get_local_port(sk);
	return (port == INT_MAX) ? -1 : (int)(port - 1);
}

/*
 * store local socket fd inside nl_sock structure
 * use local address port since nl_sock->s_fd is
 * not accessible
 */
static void nl60g_set_local_sock_fd(struct nl_sock *sk, int fd)
{
	/*
	 * 0 cannot be stored, otherwise nl_socket_get_local_port
	 * will return a random port
	 */
	uint32_t port = (fd < 0) ? INT_MAX : fd + 1;
	nl_socket_set_local_port(sk, port);
}

static int nl60g_recv_handler(struct nl_sock *sk, struct sockaddr_nl *nla,
	unsigned char **buf, struct ucred **creds)
{
	int avail, received, rc;
	struct iovec iov;
	struct msghdr msg;
	int s_fd = nl60g_get_local_sock_fd(sk);

	if (!buf || !nla)
		return -NLE_INVAL;

	/* nla is ignored since we use local connected socket */
	memset(nla, 0, sizeof(struct sockaddr_nl));
	msg.msg_name = NULL;
	msg.msg_namelen = 0;

	/* creds is also ignored since we use connected socket */
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	if (creds)
		*creds = NULL;

	rc = ioctl(s_fd, FIONREAD, &avail);
	if (rc < 0) {
		RTE_LOG(ERR, PMD, "failed F_IONREAD: %s\n", strerror(errno));
		return -NLE_FAILURE;
	}
	if (avail == 0)
		return -NLE_AGAIN;

	iov.iov_len = avail;
	iov.iov_base = malloc(avail); /* buffer freed by libnl */
	if (!iov.iov_base)
		return -NLE_NOMEM;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;

	rc = 0;
retry:
	received = recvmsg(s_fd, &msg, MSG_DONTWAIT);
	if (!received) {
		rc = 0;
		goto out;
	}

	if (received < 0) {
		if (errno == EINTR)
			goto retry;
		rc = -nl_syserr2nlerr(errno);
		goto out;
	}

	if (msg.msg_flags & MSG_TRUNC) {
		/* this should never happen */
		RTE_LOG(ERR, PMD, "truncated message received!!!\n");
		rc = -NLE_FAILURE;
		goto out;
	}

	rc = received;
out:
	if (rc > 0)
		*buf = iov.iov_base;
	else
		free(iov.iov_base);

	return rc;
}

static int nl60g_send_handler(struct nl_sock *sk, struct nl_msg *msg)
{
	struct iovec iov;
	struct nlmsghdr *nlh;
	struct msghdr mhdr;
	int rc;
	int s_fd = nl60g_get_local_sock_fd(sk);

	if (s_fd < 0)
		return -NLE_BAD_SOCK;

	nlh = nlmsg_hdr(msg);
	iov.iov_base = nlh;
	iov.iov_len = nlh->nlmsg_len;

	/* no need to fill in peer, connected socket */
	mhdr.msg_name = NULL;
	mhdr.msg_namelen = 0;
	mhdr.msg_iov = &iov;
	mhdr.msg_iovlen = 1;
	mhdr.msg_control = NULL;
	mhdr.msg_controllen = 0;

	/*
	 * this send can be called from WMI event handler so use non-blocking
	 * send to avoid delaying WMI event processing
	 */
	rc = sendmsg(s_fd, &mhdr, MSG_DONTWAIT);
	if (rc < 0)
		return -nl_syserr2nlerr(errno);
	return rc;
}

static bool nl60g_validate_cmd(uint32_t cmd, struct nlattr **tb, int *plen)
{
	int i;
	struct nl60g_cmd_def *def = NULL;
	struct nl60g_subcmd_hdr *gcmd;
	struct nl60g_subcmd_def *subcmds, *subcmd = NULL;

	if (!plen)
		return false;

	*plen = 0;

	for (i = 0; nl60g_cmds[i].cmd != NL_60G_CMD_MAX; i++) {
		if (nl60g_cmds[i].cmd == cmd) {
			def = &nl60g_cmds[i];
			break;
		}
	}

	if (!def)
		return false;

	if (!def->need_buf)
		return true;
	if (!tb[WIL_ATTR_60G_BUF])
		return false;
	*plen = nla_len(tb[WIL_ATTR_60G_BUF]);
	if (*plen < def->len)
		return false;

	subcmds = def->subcmds;
	if (!subcmds)
		return true;
	gcmd = (struct nl60g_subcmd_hdr *)nla_data(tb[WIL_ATTR_60G_BUF]);
	while (subcmds->len) {
		if (subcmds->cmd_id == gcmd->cmd_id) {
			subcmd = subcmds;
			break;
		}
		subcmds++;
	}

	if (!subcmd)
		return false;
	if (*plen < subcmd->len)
		return false;

	return true;
}

/*
 * allocate an nl_msg structure for sending
 * vendor event
 * starts a NLA80211_ATTR_VENDOR_DATA attribute so caller
 * can fill the reply.
 */
static struct nl_msg *nl60g_alloc_vendor_event(int len, struct nlattr **pvendor_data)
{
	struct nl_msg *msg;
	struct nlattr *vendor_data;

	if (!pvendor_data)
		return NULL;

	/* add extra 64 bytes for the attributes we add below,
	 * WIL_ATTR_60G_BUF header which caller always adds, and
	 * some spare. If caller adds other attributes besides the
	 * WIL_ATTR_60_BUF payload, it should be accounted for in len
	 */
	msg = nlmsg_alloc_size(64 + NLMSG_HDRLEN + len);
	if (!msg)
		return NULL;

	if (!genlmsg_put(msg,
		111, /* pid - dummy value so libnl will not try to allocate */
		0, /* sequence number (automatic) */
		NL80211_FAMILTY_ID, /* nl80211 familty (dummy) */
		0, /* user specific header length */
		0, /* flags */
		NL80211_CMD_VENDOR, /* command */
		0) /* protocol version */
	) {
		RTE_LOG(ERR, PMD, "failed to init vendor event\n");
		goto fail;
	}

	/* dummy WIPHY index for compatibility */
	if (nla_put_u32(msg, NL80211_ATTR_WIPHY, 1) ||
	    nla_put_u32(msg, NL80211_ATTR_VENDOR_ID, OUI_QCA) ||
	    nla_put_u32(msg, NL80211_ATTR_VENDOR_SUBCMD,
		        QCA_NL80211_VENDOR_SUBCMD_UNSPEC)) {
		RTE_LOG(ERR, PMD, "failed to setup vendor event\n");
		goto fail;
	}

	vendor_data = nla_nest_start(msg, NL80211_ATTR_VENDOR_DATA);
	if (!vendor_data) {
		RTE_LOG(ERR, PMD, "can't start vendor data for reply\n");
		goto fail;
	}
	*pvendor_data = vendor_data;

	return msg;
fail:
	nlmsg_free(msg);
	return NULL;
}

/*
 * allocate an nl_msg structure for sending
 * reply to command.
 * On success fills vendor_data with NL80211_ATTR_VENDOR_DATA attribute
 * so caller can close it after filling in the reply
 */
static struct nl_msg *nl60g_alloc_vendor_reply(int len, struct nl_msg *cmd,
	struct nlattr **pvendor_data)
{
	struct nl_msg *msg;
	struct nlmsghdr *replyhdr, *cmdhdr;

	msg = nl60g_alloc_vendor_event(len, pvendor_data);
	if (!msg)
		return NULL;

	cmdhdr = nlmsg_hdr(cmd);
	replyhdr = nlmsg_hdr(msg);

	replyhdr->nlmsg_pid = cmdhdr->nlmsg_pid;
	replyhdr->nlmsg_seq = cmdhdr->nlmsg_seq;

	return msg;
}

static void nl60g_send_60g_fw_state_evt(struct nl60g_port *port,
	enum wil_fw_state fw_state)
{
	struct nl_msg *vendor_event = NULL;
	struct nlattr *vendor_data = NULL;
	struct nl60g_event *evt;
	struct nl60g_fw_state_event *fw_state_event;
	int len = sizeof(*evt) + sizeof(*fw_state_event);

	if (!port->sk || !port->publish_nl_evt)
		return;

	RTE_LOG(INFO, PMD, "report fw_state event to user space (%d)\n",
		fw_state);

	evt = calloc(1, len);
	if (!evt)
		return;

	evt->evt_type = NL_60G_EVT_DRIVER_GENERIC;
	evt->buf_len = sizeof(*fw_state_event);

	fw_state_event = (struct nl60g_fw_state_event *)evt->buf;
	fw_state_event->hdr.evt_id = NL_60G_GEN_EVT_FW_STATE;
	fw_state_event->fw_state = fw_state;

	vendor_event = nl60g_alloc_vendor_event(len, &vendor_data);
	if (!vendor_event)
		goto out;

	if (nla_put(vendor_event, WIL_ATTR_60G_BUF, len, evt)) {
		RTE_LOG(ERR, PMD, "failed to fill WIL_ATTR_60G_BUF\n");
		goto out;
	}
	nla_nest_end(vendor_event, vendor_data);

	nl_send_auto(port->sk, vendor_event);

out:
	nlmsg_free(vendor_event);
	free(evt);
}

static void nl60g_send_60g_wmi_evt(
	struct nl60g_port *port, uint8_t *cmd, int len)
{
	struct nl60g_state *nl60g = port->nl60g;
	struct wil6210_priv *wil = nl60g->wil;
	struct nl_msg *vendor_event = NULL;
	struct nlattr *vendor_data = NULL;
	struct nl60g_event *evt;
	struct nl60g_send_receive_wmi *wmi_buf;
	struct wmi_cmd_hdr *wmi_hdr = (struct wmi_cmd_hdr *)cmd;
	int data_len, total_len;

	if (!port->sk || !port->publish_nl_evt)
		return;

	RTE_LOG(DEBUG, PMD, "report wmi event to user-space (0x%04x) [%d]\n",
		le16_to_cpu(wmi_hdr->command_id), len);

	data_len = len - sizeof(struct wmi_cmd_hdr);
	total_len = sizeof(*evt) + sizeof(*wmi_buf) + data_len;

	evt = calloc(1, total_len);
	if (!evt)
		return;

	evt->evt_type = NL_60G_EVT_FW_WMI;
	evt->buf_len = sizeof(*wmi_buf) + data_len;

	wmi_buf = (struct nl60g_send_receive_wmi *)evt->buf;

	wmi_buf->cmd_id = le16_to_cpu(wmi_hdr->command_id);
	wmi_buf->dev_id = wmi_hdr->mid;
	wmi_buf->buf_len = data_len;
	memcpy(wmi_buf->buf, cmd + sizeof(struct wmi_cmd_hdr), data_len);

	vendor_event = nl60g_alloc_vendor_event(data_len +
		sizeof(*evt) + sizeof(*wmi_buf), &vendor_data);
	if (!vendor_event)
		goto out;

	if (nla_put(vendor_event, WIL_ATTR_60G_BUF, total_len, evt)) {
		RTE_LOG(ERR, PMD, "failed to fill WIL_ATTR_60G_BUF\n");
		goto out;
	}
	nla_nest_end(vendor_event, vendor_data);

	nl_send_auto(port->sk, vendor_event);

out:
	nlmsg_free(vendor_event);
	free(evt);
}

void nl60g_fw_state_evt(struct nl60g_state *nl60g,
	enum wil_fw_state fw_state)
{
	int i;

	if (!nl60g)
		return;

	pthread_mutex_lock(&nl60g->ports_mutex);
	if (!nl60g->wil)
		goto out;
	for (i = 0; i < NL60G_MAX_PORTS; i++)
		nl60g_send_60g_fw_state_evt(&nl60g->ports[i], fw_state);
out:
	pthread_mutex_unlock(&nl60g->ports_mutex);
}

void nl60g_receive_wmi_evt(struct nl60g_state *nl60g, uint8_t *cmd, int len)
{
	int i;

	if (!nl60g)
		return;

	pthread_mutex_lock(&nl60g->ports_mutex);
	if (!nl60g->wil)
		goto out;
	for (i = 0; i < NL60G_MAX_PORTS; i++)
		nl60g_send_60g_wmi_evt(&nl60g->ports[i], cmd, len);
out:
	pthread_mutex_unlock(&nl60g->ports_mutex);
}


/*
 * allocate a reply message.
 * A generic NLMSG_ERROR message is used to report
 * both success and error
 */
static struct nl_msg *nl60g_alloc_reply_msg(void)
{
	int payload_size = sizeof(struct nlmsgerr);
	struct nl_msg *msg;
	struct nlmsghdr *hdr;

	msg = nlmsg_alloc();
	if (!msg)
		return NULL;

	/* use dummy non-0 PID so libnl will not try to allocate */
	hdr = nlmsg_put(msg, 111, 0, NLMSG_ERROR, payload_size, 0);
	if (!hdr) {
		nlmsg_free(msg);
		return NULL;
	}

	return msg;
}

/*
 * send a reply message with success or error code.
 * req is the request message. The header is copied
 * to the reply.
 * err is the error code to be reported (0 means success)
 * Not thread-safe and typically only called from the poll thread
 * which processes requests.
 */
static int nl60g_send_reply_msg(struct nl_sock *sk, struct nl_msg *req,
			        int err)
{
	struct nl_msg *reply;
	struct nlmsghdr *reqhdr = nlmsg_hdr(req);
	struct nlmsghdr *replyhdr;
	struct nlmsgerr *e;
	int minsize = sizeof(struct nlmsghdr) + sizeof(struct nlmsgerr);
	int rc;

	reply = nl60g_alloc_reply_msg();
	if (!reply)
		return -NLE_NOMEM;

	replyhdr = nlmsg_hdr(reply);
	if (replyhdr->nlmsg_len < minsize) {
		RTE_LOG(ERR, PMD, "reply too small! need %d have %d\n",
			minsize, (int)replyhdr->nlmsg_len);
		nlmsg_free(reply);
		return -NLE_NOMEM;
	}

	e = nlmsg_data(replyhdr);
	e->error = err;
	memcpy(&e->msg, reqhdr, sizeof(struct nlmsghdr));
	replyhdr->nlmsg_seq = 0; /* auto */

	rc = nl_send_auto(sk, reply);
	nlmsg_free(reply);
	return rc;
}

static void nl60g_fill_sta_info_entry(struct wil6210_priv *wil,
	struct wil_nl_60g_sta_info_entry *entry, int cid)
{
	struct wil_sta_info *p = &wil->sta[cid];
	uint8_t mid = (p->status != wil_sta_unused) ? p->mid : UINT8_MAX;
	uint16_t aid = (p->status == wil_sta_connected) ? p->aid : 0;

	entry->status = p->status;
	memcpy(entry->mac_addr, p->addr, ETH_ALEN);
	entry->mid = mid;
	entry->aid = aid;
	entry->reserved[0] = entry->reserved[1] = 0;
}

/* max amount of data we can return in single PMC message
 * adjusted to smaller value to be consistent with kernel driver
 */
#define NL60G_MAX_PMC_PAYLOAD	32720

/*
 * handle PMC GET_DATA/GET_DESC_DATA command
 */
static int
nl60g_handle_pmc_data_command(struct nl60g_port *port,
	struct nl_msg *msg, struct nl60g_subcmd_hdr *pmc_cmd)
{
	struct nl60g_state *nl60g = port->nl60g;
	struct wil6210_priv *wil = nl60g->wil;
	struct wil_pmc_reader_ops *reader;
	void *reader_ctx;
	struct nl60g_pmc_get_data *pmc_get_data_cmd;
	struct nl_msg *creply = NULL;
	struct nlattr *vendor_data, *data;
	int rc = 0;
	uint32_t num_bytes;
	ssize_t toread, read, remaining, min_size;
	bool is_get_data, more_data;

	pmc_get_data_cmd = (struct nl60g_pmc_get_data *)pmc_cmd;
	is_get_data = (pmc_cmd->cmd_id == NL_60G_PMC_GET_DATA);
	num_bytes = pmc_get_data_cmd->num_bytes;

	if (is_get_data) {
		reader = &port->pmc_reader;
		if (!port->pmc_reader_ctx)
			rc = wil_pmc_alloc_pmc_reader(wil, reader,
				&port->pmc_reader_ctx);
		reader_ctx = port->pmc_reader_ctx;
	} else {
		reader = &port->pmcring_reader;
		if (!port->pmcring_reader_ctx)
			rc = wil_pmc_alloc_pmcring_reader(wil, reader,
				&port->pmcring_reader_ctx);
		reader_ctx = port->pmcring_reader_ctx;
	}
	if (rc)
		return -nl_syserr2nlerr(rc);

	remaining = reader->available(reader_ctx);
	if (remaining < 0)
		return -nl_syserr2nlerr(remaining);
	min_size = reader->min_read_size(reader_ctx);
	if (min_size < 0)
		return -nl_syserr2nlerr(min_size);

	creply = nl60g_alloc_vendor_reply(NL60G_MAX_PMC_PAYLOAD,
		msg, &vendor_data);
	if (!creply)
		return -NLE_NOMEM;

	if (num_bytes == 0) {
		if (nla_put_u32(creply, QCA_WLAN_VENDOR_ATTR_PMC_DATA_LENGTH,
		                remaining)) {
			RTE_LOG(ERR, PMD, "fail to write data length\n");
			rc = -NLE_NOMEM;
			goto out_free;
		}
		goto out_send;
	}

	if (num_bytes < min_size) {
		if (nla_put_u32(creply,
				QCA_WLAN_VENDOR_ATTR_PMC_MIN_DATA_LENGTH,
				min_size)) {
			RTE_LOG(ERR, PMD, "fail to write min data length\n");
			rc = -NLE_NOMEM;
			goto out_free;
		}
		/* when requesting to read < min_len, return error */
		rc = -NLE_FAILURE;
		goto out_send;
	}

	more_data = false;
	toread = min_t(uint32_t, num_bytes, NL60G_MAX_PMC_PAYLOAD);
	toread = reader->read_size(reader_ctx, toread);
	if (toread < 0) {
		RTE_LOG(ERR, PMD, "internal error read_size: %d\n", (int)toread);
		rc = -NLE_NOMEM;
		goto out_free;
	}

	data = nla_reserve(creply, QCA_WLAN_VENDOR_ATTR_PMC_DATA, toread);
	if (!data) {
		RTE_LOG(ERR, PMD, "fail to write data\n");
		rc = -NLE_NOMEM;
		goto out_free;
	}
	read = reader->read(reader_ctx, nla_data(data), toread);
	if (toread != read) {
		RTE_LOG(ERR, PMD, "internal error filling PMC data, toread %d read %d\n",
			(int)toread, (int)read);
		rc = -NLE_NOMEM;
		goto out_free;
	}

	more_data = (read < remaining);
	if (more_data &&
	    nla_put_flag(creply, QCA_WLAN_VENDOR_ATTR_PMC_MORE_DATA)) {
		RTE_LOG(ERR, PMD, "fail to write more data flag\n");
		rc = -NLE_NOMEM;
		goto out_free;
	}

out_send:
	nla_nest_end(creply, vendor_data);
	nl_send_auto(port->sk, creply);
out_free:
	nlmsg_free(creply);
	return rc;
}

/*
 * handle PMC command
 */
static int
nl60g_handle_pmc_command(struct nl60g_port *port,
	struct nl_msg *msg, struct nlattr **tb)
{
	struct nl60g_state *nl60g = port->nl60g;
	struct wil6210_priv *wil = nl60g->wil;
	struct pmc_ctx *pmc = &wil->pmc;
	struct nl60g_subcmd_hdr *pmc_cmd;
	struct nl60g_pmc_alloc *pmc_alloc_cmd;
	uint32_t num_descs, payload_size;
	int rc = 0;

	pmc_cmd = (struct nl60g_subcmd_hdr *)nla_data(tb[WIL_ATTR_60G_BUF]);

	switch (pmc_cmd->cmd_id) {
	case NL_60G_PMC_ALLOC:
		pmc_alloc_cmd = (struct nl60g_pmc_alloc *)pmc_cmd;
		num_descs = pmc_alloc_cmd->num_desc;
		payload_size = pmc_alloc_cmd->payload_size;
		wil_pmc_alloc(wil, num_descs, payload_size);
		if (pmc->last_cmd_status)
			rc = -nl_syserr2nlerr(pmc->last_cmd_status);
		break;

	case NL_60G_PMC_FREE:
		wil_pmc_free(wil, true);
		if (pmc->last_cmd_status)
			rc = -nl_syserr2nlerr(pmc->last_cmd_status);
		wil_pmc_free_reader(port->pmc_reader_ctx);
		port->pmc_reader_ctx = NULL;
		wil_pmc_free_reader(port->pmcring_reader_ctx);
		port->pmcring_reader_ctx = NULL;
		break;

	case NL_60G_PMC_GET_DATA:
	case NL_60G_PMC_GET_DESC_DATA:
		rc = nl60g_handle_pmc_data_command(port, msg, pmc_cmd);
		break;
	default:
		RTE_LOG(ERR, PMD, "unknown PMC command: %d\n",
			(int)pmc_cmd->cmd_id);
		rc = -NLE_FAILURE;
		break;
	}

	return rc;
}

/*
 * handles requests received over the local socket.
 * based on the wil_nl_60g_handle_cmd function in
 * wil6210 driver
 */
static int nl60g_cmd_handler(struct nl_msg *msg, void *arg)
{
	struct nl60g_port *port = arg;
	struct nl60g_state *nl60g = port->nl60g;
	struct wil6210_priv *wil = nl60g->wil;
	struct nlattr *tb[NL80211_ATTR_MAX + 1];
	struct nlattr *tb2[QCA_ATTR_WIL_MAX + 1];
	struct genlmsghdr *gnlh = (struct genlmsghdr *)
		nlmsg_data(nlmsg_hdr(msg));
	struct nl60g_send_receive_wmi *wmi_cmd;
	struct nl60g_generic_force_wmi gen_force_wmi;
	int rc = 0, len = 0;
	uint32_t cmd, cmd_type, publish;

	nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0),
		  genlmsg_attrlen(gnlh, 0) , NULL);

	if (!tb[NL80211_ATTR_VENDOR_ID] ||
	    !tb[NL80211_ATTR_VENDOR_SUBCMD] ||
	    !tb[NL80211_ATTR_VENDOR_DATA])
		return NL_SKIP;

	if (nla_get_u32(tb[NL80211_ATTR_VENDOR_ID]) != OUI_QCA)
		return NL_SKIP;

	cmd = nla_get_u32(tb[NL80211_ATTR_VENDOR_SUBCMD]);
	if (cmd != QCA_NL80211_VENDOR_SUBCMD_UNSPEC) {
		RTE_LOG(ERR, PMD, "skip unknown cmd: %d\n", (int)cmd);
		return NL_SKIP;
	}

	if (nla_parse_nested(tb2, QCA_WLAN_VENDOR_ATTR_MAX,
	    tb[NL80211_ATTR_VENDOR_DATA], nl60g_policy)) {
		RTE_LOG(ERR, PMD, "fail to parse vendor data\n");
		rc = -NLE_INVAL;
		goto out;
	}

	if (!tb2[WIL_ATTR_60G_CMD_TYPE]) {
		RTE_LOG(ERR, PMD, "cmd type not found\n");
		rc = -NLE_INVAL;
		goto out;
	}

	cmd_type = nla_get_u32(tb2[WIL_ATTR_60G_CMD_TYPE]);
	if (!nl60g_validate_cmd(cmd_type, tb2, &len)) {
		RTE_LOG(ERR, PMD, "Invalid nl_60g_cmd spec\n");
		rc = -NLE_INVAL;
		goto out;
	}

	switch(cmd_type) {
	case NL_60G_CMD_REGISTER:
		memcpy(&publish, nla_data(tb2[WIL_ATTR_60G_BUF]),
		       sizeof(uint32_t));
		port->publish_nl_evt = (publish != 0);
		RTE_LOG(INFO, PMD, "Publish wmi event %s\n", publish ?
			"enabled" : "disabled");
		nl60g_send_60g_fw_state_evt(port, wil->fw_state);
		break;
	case NL_60G_CMD_GENERIC:
		memcpy(&gen_force_wmi, nla_data(tb2[WIL_ATTR_60G_BUF]),
		       sizeof(struct nl60g_subcmd_hdr));

		switch (gen_force_wmi.hdr.cmd_id) {
		case NL_60G_GEN_FORCE_WMI_SEND:
			if (len != sizeof(gen_force_wmi)) {
				RTE_LOG(ERR, PMD, "cmd buffer wrong len %d\n",
					len);
				rc = -EINVAL;
				break;
			}

			memcpy(&gen_force_wmi, nla_data(tb2[WIL_ATTR_60G_BUF]),
			       sizeof(gen_force_wmi));
			wil->force_wmi_send = gen_force_wmi.enable;

			RTE_LOG(INFO, PMD, "force sending wmi commands %d\n",
				wil->force_wmi_send);
			break;
		case NL_60G_GEN_FW_RESET:
			/* not supported */
			rc = -NLE_INVAL;
			break;

		case NL_60G_GEN_GET_DRIVER_CAPA:
		{
			struct nl_msg *creply;
			struct nlattr *vendor_data;
			uint32_t capa;

			creply = nl60g_alloc_vendor_reply(sizeof(capa), msg,
				&vendor_data);
			if (!creply) {
				rc = -NLE_NOMEM;
				break;
			}

			capa = BIT(NL_60G_DRIVER_CAPA_FW_STATE) |
			       BIT(NL_60G_DRIVER_CAPA_WMI_OVER_NL) |
			       BIT(NL_60G_DRIVER_CAPA_MEMIO) |
			       BIT(NL_60G_DRIVER_CAPA_MEMIO_WRITE) |
			       BIT(NL_60G_DRIVER_CAPA_GET_STA_INFO) |
			       BIT(NL_60G_DRIVER_CAPA_PMC_LEGACY_OVER_NL);
			if (nla_put_u32(creply,
			    QCA_WLAN_VENDOR_ATTR_DRIVER_CAPA,
			    capa)) {
				RTE_LOG(ERR, PMD, "fail to return driver capa\n");
				nlmsg_free(creply);
				rc = -NLE_NOMEM;
				break;
			}
			nla_nest_end(creply, vendor_data);

			nl_send_auto(port->sk, creply);
			nlmsg_free(creply);
			break;
		}
		case NL_60G_GEN_GET_FW_STATE:
		{
			struct nl_msg *creply;
			struct nlattr *vendor_data;
			uint32_t fw_state;

			creply = nl60g_alloc_vendor_reply(sizeof(fw_state),
				msg, &vendor_data);
			if (!creply) {
				rc = -NLE_NOMEM;
				break;
			}

			fw_state = wil->fw_state;
			if (nla_put_u32(creply,
					QCA_WLAN_VENDOR_ATTR_DRIVER_FW_STATE,
					fw_state)) {
				RTE_LOG(ERR, PMD, "fail to return fw_state\n");
				nlmsg_free(creply);
				rc = -NLE_NOMEM;
				break;
			}
			nla_nest_end(creply, vendor_data);

			nl_send_auto(port->sk, creply);
			nlmsg_free(creply);
			break;
		}
		case NL_60G_GEN_GET_STA_INFO:
		{
			struct nl_msg *creply;
			struct nlattr *vendor_data, *reply_attr;
			int num_sta = ARRAY_SIZE(wil->sta), i;
			int reply_len = sizeof(struct wil_nl_60g_sta_info) +
				num_sta * sizeof(struct wil_nl_60g_sta_info_entry);
			struct wil_nl_60g_sta_info *si;

			creply = nl60g_alloc_vendor_reply(reply_len, msg, &vendor_data);
			if (!creply) {
				rc = -NLE_NOMEM;
				break;
			}

			reply_attr = nla_reserve(creply,
				QCA_WLAN_VENDOR_ATTR_STA_INFO, reply_len);
			if (!reply_attr) {
				nlmsg_free(creply);
				rc = -NLE_NOMEM;
				break;
			}
			si = (struct wil_nl_60g_sta_info *)nla_data(reply_attr);
			si->num_cids = num_sta;
			for (i = 0; i < num_sta; i++) {
				nl60g_fill_sta_info_entry(
					wil, &si->stations[i], i);
			}

			nla_nest_end(creply, vendor_data);

			nl_send_auto(port->sk, creply);
			nlmsg_free(creply);
			break;
		}
		case NL_60G_GEN_GET_FW_CAPA:
		{
			struct nl_msg *creply;
			struct nlattr *vendor_data;

			creply = nl60g_alloc_vendor_reply(sizeof(wil->fw_capabilities),
				msg, &vendor_data);
			if (!creply) {
				rc = -NLE_NOMEM;
				break;
			}

			if (nla_put(creply,
				QCA_WLAN_VENDOR_ATTR_DRIVER_FW_CAPA,
				sizeof(wil->fw_capabilities),
				&wil->fw_capabilities)) {
				RTE_LOG(ERR, PMD, "fail to return fw_capabilities\n");
				nlmsg_free(creply);
				rc = -NLE_NOMEM;
				break;
			}
			nla_nest_end(creply, vendor_data);

			nl_send_auto(port->sk, creply);
			nlmsg_free(creply);
			break;
		}
		default:
			rc = -NLE_INVAL;
			RTE_LOG(ERR, PMD, "invalid generic_cmd id %d",
				gen_force_wmi.hdr.cmd_id);
		}
		break;
	case NL_60G_CMD_FW_WMI:
		len = nla_len(tb2[WIL_ATTR_60G_BUF]);
		wmi_cmd = (struct nl60g_send_receive_wmi *)
			nla_data(tb2[WIL_ATTR_60G_BUF]);

		if (len < wmi_cmd->buf_len + sizeof(struct nl60g_send_receive_wmi)) {
			RTE_LOG(ERR, PMD, "WMI buffer too small (%d)\n", len);
			rc = -NLE_INVAL;
			break;
		}

		RTE_LOG(DEBUG, PMD, "sending user-space command (0x%04x) [%d]\n",
			wmi_cmd->cmd_id, wmi_cmd->buf_len);

		if (wil->force_wmi_send)
			rc = wmi_force_send(wil, wmi_cmd->cmd_id, wmi_cmd->dev_id,
					    wmi_cmd->buf, wmi_cmd->buf_len);
		else
			rc = wmi_send(wil, wmi_cmd->cmd_id, wmi_cmd->dev_id,
				      wmi_cmd->buf, wmi_cmd->buf_len);


		if (rc)
			rc = -nl_syserr2nlerr(rc);

		break;
	case NL_60G_CMD_MEMIO:
	{
		struct nl_msg *creply;
		struct nlattr *vendor_data;
		struct nl60g_memio *nl_memio;
		struct wil_memio memio;

		nl_memio = (struct nl60g_memio *)
			nla_data(tb2[WIL_ATTR_60G_BUF]);
		memio.op = nl_memio->op;
		memio.addr = nl_memio->addr;
		memio.val = nl_memio->val;

		rc = wil_memio_dword(nl60g->wil, &memio);
		if (rc) {
			rc = -nl_syserr2nlerr(rc);
			break;
		}
		nl_memio->val = memio.val;

		creply = nl60g_alloc_vendor_reply(sizeof(struct nl60g_memio),
			msg, &vendor_data);
		if (!creply) {
			rc = -NLE_NOMEM;
			break;
		}

		if (nla_put(creply, QCA_WLAN_VENDOR_ATTR_MEMIO,
			sizeof(struct nl60g_memio), nl_memio)) {
			RTE_LOG(ERR, PMD, "fail to return CMD_MEMIO reply\n");
			nlmsg_free(creply);
			rc = -NLE_NOMEM;
			break;
		}
		nla_nest_end(creply, vendor_data);
		nl_send_auto(port->sk, creply);
		nlmsg_free(creply);
		break;
	}
	case NL_60G_CMD_MEMIO_BLOCK:
	{
		struct nl_msg *creply;
		struct nlattr *vendor_data, *block_attr;
		bool is_read;
		struct nl60g_memio_block *nl_memio_blk, *nl_memio_blk_reply;
		struct wil_memio_block memio_block;

		nl_memio_blk = (struct nl60g_memio_block *)
			nla_data(tb2[WIL_ATTR_60G_BUF]);
		memio_block.op = nl_memio_blk->op;
		memio_block.addr = nl_memio_blk->addr;
		memio_block.size = nl_memio_blk->size;

		/* pre-allocate the reply, it will save a memcpy for read */
		is_read = (memio_block.op & wil_mmio_op_mask) == wil_mmio_read;
		len = sizeof(struct nl60g_memio_block);
		if (is_read) {
			len += memio_block.size;
			if (len > USHRT_MAX) {
				rc = -NLE_MSGSIZE;
				break;
			}
		} else {
			int needed_len = len + memio_block.size;
			if (nla_len(tb2[WIL_ATTR_60G_BUF]) < needed_len) {
			    rc = -NLE_MSGSIZE;
			    break;
			}
		}

		creply = nl60g_alloc_vendor_reply(len, msg, &vendor_data);
		if (!creply) {
			rc = -NLE_NOMEM;
			break;
		}
		block_attr = nla_reserve(
			creply, QCA_WLAN_VENDOR_ATTR_MEMIO_BLOCK, len);
		if (!block_attr) {
			nlmsg_free(creply);
			rc = -NLE_NOMEM;
			break;
		}
		nl_memio_blk_reply = (struct nl60g_memio_block *)
			nla_data(block_attr);
		memio_block.block = is_read ? &nl_memio_blk_reply->buf :
			&nl_memio_blk->buf;

		rc = wil_memio_block(nl60g->wil, &memio_block);
		if (rc) {
			rc = -nl_syserr2nlerr(rc);
			nlmsg_free(creply);
			break;
		}

		nl_memio_blk_reply->op = memio_block.op;
		nl_memio_blk_reply->addr = memio_block.addr;
		nl_memio_blk_reply->size = memio_block.size;
		nla_nest_end(creply, vendor_data);
		nl_send_auto(port->sk, creply);
		nlmsg_free(creply);
		break;
	}
	case NL_60G_CMD_PMC:
		rc = nl60g_handle_pmc_command(port, msg, tb2);
		break;
	default:
		rc = -NLE_INVAL;
		wil_err(wil, "invalid nl_60g_cmd type %d", cmd_type);
		break;
	}
out:
	return nl60g_send_reply_msg(port->sk, msg, rc);
}

static int
nl60g_read_connection(struct nl60g_port *port)
{
	int rc;

	if (!port->sk) {
		return 0;
	}

	rc = nl_recvmsgs_report(port->sk, port->cb);
	if (rc < 0)
		RTE_LOG(ERR, PMD, "read error: %d\n", rc);

	return rc;
}

static void *
nl60g_req_worker_thread(void *arg)
{
	struct nl60g_port *port = arg;
	struct nl60g_state *nl60g = port->nl60g;
	struct pollfd pfd[2];
	int rc;

	memset(&pfd, 0, sizeof(pfd));
	pfd[0].fd = port->fd;
	pfd[0].events = POLLIN;
	pfd[1].fd = port->exit_sockets[1];
	pfd[1].events = POLLIN;

	for (/*none*/; /*none*/; /*none*/) {
		rc = poll(pfd, 2, 2000);
		if (rc == 0)
			continue;
		if (rc < 0) {
			if (errno == EINTR)
				continue;
			RTE_LOG(ERR, PMD, "poll error: %s\n", strerror(errno));
			/* TODO should we exit? */
			continue;
		}

		if (pfd[1].revents & POLLIN) {
			RTE_LOG(INFO, PMD, "request thread exit by request\n");
			break;
		}

		if (pfd[0].revents & POLLIN) {
			rc = nl60g_read_connection(port);
			if (rc <= 0) {
				/* error or connection terminated */
				break;
			}
		}

	}

	TEMP_FAILURE_RETRY(write(port->exit_sockets[1], "T", 1));
	return NULL;
}

static void
nl60g_accept_new_connection(struct nl60g_state *nl60g)
{
	int data_fd, index, i, rc;
	int blen = NL60G_MAX_MSG_SIZE;
	struct nl60g_port *port = NULL;
	bool success = false;

	data_fd = accept(nl60g->fd, NULL, NULL);
	if (data_fd == -1) {
		RTE_LOG(ERR, PMD, "accept error: %s\n", strerror(errno));
		return;
	}

	/* adjust socket buffers for supporting large memory blocks */
	if (setsockopt(data_fd, SOL_SOCKET, SO_RCVBUF, &blen,
	    sizeof(blen)) == -1) {
		RTE_LOG(ERR, PMD, "fail to set SO_RCVBUF: %s\n",
			strerror(errno));
		/* continue anyway */
	}
	if (setsockopt(data_fd, SOL_SOCKET, SO_SNDBUF, &blen,
	    sizeof(blen)) == -1) {
		RTE_LOG(ERR, PMD, "fail to set SO_SNDBUF: %s\n",
			strerror(errno));
		/* continue anyway */
	}

	index = -1;
	for (i = 0; i < NL60G_MAX_PORTS; i++) {
		if (nl60g->ports[i].fd == -1) {
			index = i;
			break;
		}
	}
	if (index == -1) {
		RTE_LOG(ERR, PMD, "too many active connections, rejecting new\n");
		close(data_fd);
		return;
	}

	port = &nl60g->ports[index];
	pthread_mutex_lock(&nl60g->ports_mutex);
	rc = socketpair(AF_UNIX, SOCK_STREAM, 0, port->exit_sockets);
	if (rc == -1) {
		rc = errno;
		RTE_LOG(ERR, PMD, "unable to create exit sockets: %s\n",
			strerror(rc));
		goto out;
	}
	port->cb = nl_cb_alloc(NL_CB_DEFAULT);
	if (port->cb == NULL) {
		RTE_LOG(ERR, PMD, "fail to alloc NL callback\n");
		goto out;
	}
	nl_cb_set(nl60g->ports[index].cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM,
		  no_seq_check, NULL);
	nl_cb_overwrite_recv(nl60g->ports[index].cb, nl60g_recv_handler);
	nl_cb_overwrite_send(nl60g->ports[index].cb, nl60g_send_handler);
	nl_cb_set(nl60g->ports[index].cb, NL_CB_VALID, NL_CB_CUSTOM,
		  nl60g_cmd_handler, port);

	nl60g->ports[index].sk = nl_socket_alloc_cb(nl60g->ports[index].cb);
	if (nl60g->ports[index].sk == NULL) {
		RTE_LOG(ERR, PMD, "fail to alloc NL socket\n");
		goto out;
	}

	nl60g->ports[index].reply = nl60g_alloc_reply_msg();
	if (!nl60g->ports[index].reply) {
		RTE_LOG(ERR, PMD, "fail to alloc reply\n");
		goto out;
	}

	nl60g_set_local_sock_fd(nl60g->ports[index].sk, data_fd);
	nl60g->ports[index].fd = data_fd;

	rc = pthread_create(&port->req_thread, NULL,
	    nl60g_req_worker_thread, port);
	if (rc != 0) {
		rc = errno;
		RTE_LOG(ERR, PMD, "Unable to create request thread: %s\n",
		    strerror(rc));
		goto out;
	}

	RTE_LOG(DEBUG, PMD, "accept new connection data fd %d index %d\n", data_fd, index);
	success = true;

out:
	if (!success) {
		nlmsg_free(port->reply);
		port->reply = NULL;
		nl_socket_free(port->sk);
		port->sk = NULL;
		nl_cb_put(port->cb);
		port->cb = NULL;
		if (port->exit_sockets[0] >= 0) {
			close(port->exit_sockets[0]);
			close(port->exit_sockets[1]);
			port->exit_sockets[0] = port->exit_sockets[1] = -1;
		}
		port->fd = -1;
		close(data_fd);
	}
	pthread_mutex_unlock(&nl60g->ports_mutex);
}

/*
 * called from nl60g_close_connection to free connection entry,
 * must be called with ports_mutex held
 */
static void
nl60g_free_connection(struct nl60g_port *port)
{
	nlmsg_free(port->reply);
	port->reply = NULL;
	nl_socket_free(port->sk);
	port->sk = NULL;
	nl_cb_put(port->cb);
	port->cb = NULL;
	close(port->fd);
	port->fd = -1;
	close(port->exit_sockets[0]);
	port->exit_sockets[0] = -1;
	close(port->exit_sockets[1]);
	port->exit_sockets[1] = -1;
	wil_pmc_free_reader(port->pmc_reader_ctx);
	port->pmc_reader_ctx = NULL;
	wil_pmc_free_reader(port->pmcring_reader_ctx);
	port->pmcring_reader_ctx = NULL;
}

static void
nl60g_close_connection(struct nl60g_port *port)
{
	struct nl60g_state *nl60g = port->nl60g;
	int i, j;

	/* perform some actions without the lock to avoid
	 * delaying other operations too much (such as sending events).
	 * nl60g_close_connection is only called from poll worker thread
	 * so no risk that other thread will modify port fields
	 */
	if (port->fd == -1)
		return;

	TEMP_FAILURE_RETRY(write(port->exit_sockets[0], "T", 1));
	pthread_join(port->req_thread, NULL);

	pthread_mutex_lock(&nl60g->ports_mutex);
	RTE_LOG(INFO, PMD, "nl60g_close_connection, fd %d\n", port->fd);

	nl60g_free_connection(port);
	pthread_mutex_unlock(&nl60g->ports_mutex);
}

static void
nl60g_close_all_connections(struct nl60g_state *nl60g)
{
	int i;

	for (i = 0; i < NL60G_MAX_PORTS; i++)
		nl60g_close_connection(&nl60g->ports[i]);
}

static void *
nl60g_poll_worker_thread(void *arg)
{
	struct nl60g_state *nl60g = arg;
	struct pollfd pfd[NL60G_MAX_PORTS + 2];
	int num_fd, i, rc, index;
	bool rebuild_poll = true;

	memset(&pfd, 0, sizeof(pfd));
	pfd[0].fd = nl60g->fd;
	pfd[0].events = POLLIN;
	pfd[1].fd = nl60g->exit_sockets[1];
	pfd[1].events = POLLIN;
	num_fd = 2;

	for (/*none*/; /*none*/; /*none*/) {
		if (rebuild_poll) {
			num_fd = 2;
			for (i = 0; i < NL60G_MAX_PORTS; i++) {
				if (nl60g->ports[i].exit_sockets[0]  < 0)
					continue;
				pfd[num_fd].fd = nl60g->ports[i].exit_sockets[0];
				pfd[num_fd].events = POLLIN;
				num_fd++;
			}
			rebuild_poll = false;
		}

		rc = poll(pfd, num_fd, 2000);
		if (rc == 0)
			continue;
		if (rc < 0) {
			if (errno == EINTR)
				continue;
			RTE_LOG(ERR, PMD, "poll error: %s\n", strerror(errno));
			/* TODO should we exit? */
			continue;
		}

		if (pfd[1].revents & POLLIN) {
			RTE_LOG(INFO, PMD, "poll thread exit by request\n");
			break;
		}

		if (pfd[0].revents & POLLIN) {
			nl60g_accept_new_connection(nl60g);
			rebuild_poll = true;
		}

		for (i = 2; i < num_fd; i++) {
			if (pfd[i].revents & POLLIN) {
				/* find out which port has this exit socket */
				for (index = 0; index < NL60G_MAX_PORTS; index++) {
					if (pfd[i].fd == nl60g->ports[index].
					    exit_sockets[0])
						break;
				}
				if (index >= NL60G_MAX_PORTS) {
					RTE_LOG(ERR, PMD, "internal error, index %d not mapped for close\n",
						i);
					continue;
				}
				nl60g_close_connection(&nl60g->ports[index]);
				rebuild_poll = true;
			}
		}
	}

	/* close all active connections, root socket will be closed outside */
	nl60g_close_all_connections(nl60g);

	return NULL;
}

static void
nl60g_stop_poll_worker(struct nl60g_state *nl60g)
{
	TEMP_FAILURE_RETRY(write(nl60g->exit_sockets[0], "T", 1));
	pthread_join(nl60g->poll_thread, NULL);
	close(nl60g->exit_sockets[0]);
	close(nl60g->exit_sockets[1]);
	nl60g->exit_sockets[0] = nl60g->exit_sockets[1] = -1;
}

static int
nl60g_start_poll_worker(struct nl60g_state *nl60g)
{
	rte_cpuset_t cpuset;
	int i, rc;

	rc = socketpair(AF_UNIX, SOCK_STREAM, 0, nl60g->exit_sockets);
	if (rc == -1) {
		rc = errno;
		RTE_LOG(ERR, PMD, "unable to create exit sockets: %s\n",
			strerror(rc));
		goto out;
	}


	rc = pthread_create(&nl60g->poll_thread, NULL,
	    nl60g_poll_worker_thread, nl60g);
	if (rc != 0) {
		rc = errno;
		RTE_LOG(ERR, PMD, "Unable to create poller thread: %s\n",
		    strerror(rc));
		goto out;
	}

	/* set affinity to the VPP master lcore */
	CPU_ZERO(&cpuset);
	CPU_SET(rte_get_master_lcore(), &cpuset);
	rc = pthread_setaffinity_np(nl60g->poll_thread, sizeof(cpuset),
		&cpuset);
	if (rc != 0) {
		RTE_LOG(ERR, PMD, "Unable to set poller thread affinity: %s\n",
			strerror(errno));
		nl60g_stop_poll_worker(nl60g);
		return rc;
	}

	/* This function is free to fail */
	(void)rte_thread_setname(nl60g->poll_thread, "nl60g-poll");

out:
	return rc;
}

static bool
nl60g_calc_socket_name(struct rte_pci_addr pci_addr, char *fname, size_t size)
{
	int written = snprintf(fname, size,
		NL60G_SOCKET_DIR "/wil6210_%d_%d_%d_%d",
		pci_addr.domain, pci_addr.bus, pci_addr.devid,
		pci_addr.function);

	return (written < (int)size);
}

int
nl60g_start(struct nl60g_state *nl60g, struct wil6210_priv *wil,
	struct rte_pci_addr pci_addr)
{
	char fname[64];
	struct sockaddr_un saddr;
	int rc;

	mkdir(NL60G_SOCKET_DIR, S_IRWXU | S_IRWXG ); /* 0770 */
	if (!nl60g_calc_socket_name(pci_addr, fname, sizeof(fname))) {
		RTE_LOG(ERR, PMD, "fail to assign socket name\n");
		return -1;
	}
	unlink(fname);
	nl60g->pci_addr = pci_addr;

	memset(&saddr, 0, sizeof(saddr));
	saddr.sun_family = AF_UNIX;
	strlcpy(saddr.sun_path, fname, sizeof(saddr.sun_path));
	nl60g->fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
	if (nl60g->fd == -1) {
		rc = errno;
		RTE_LOG(ERR, PMD, "unable to create local socket: %s\n",
			strerror(rc));
		return rc;
	}

	rc = bind(nl60g->fd, (const struct sockaddr *)&saddr, sizeof(saddr));
	if (rc == -1) {
		rc = errno;
		RTE_LOG(ERR, PMD, "unable to bind local socket: %s\n",
			strerror(rc));
		goto out;
	}

	rc = listen(nl60g->fd, NL60G_MAX_PORTS);
	if (rc == -1) {
		rc = errno;
		RTE_LOG(ERR, PMD, "unable to listen on local socket: %s\n",
			strerror(rc));
		goto out;
	}

	nl60g->wil = wil;
	rc = nl60g_start_poll_worker(nl60g);
	if (rc != 0)
		goto out;

	return 0;
out:
	close(nl60g->fd);
	nl60g->fd = -1;
	nl60g->wil = NULL;
	return rc;
}

void
nl60g_stop(struct nl60g_state *nl60g)
{
	char fname[64];

	if (nl60g->fd >= 0) {
		nl60g_stop_poll_worker(nl60g);
		close(nl60g->fd);
		nl60g->fd = -1;
		if (nl60g_calc_socket_name(nl60g->pci_addr, fname,
			sizeof(fname))) {
			unlink(fname);
		}
	}
	pthread_mutex_lock(&nl60g->ports_mutex);
	nl60g->wil = NULL;
	/* make sure other threads see nl60g->wil as NULL */
	wmb();
	pthread_mutex_unlock(&nl60g->ports_mutex);
}
