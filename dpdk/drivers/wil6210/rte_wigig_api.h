/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018 Facebook Inc.
 */

#ifndef _RTE_WIGIG_API_H_
#define _RTE_WIGIG_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/ioctl.h>
#include <stdint.h>
#include <linux/if.h>

struct rte_wigig_link_updown_info {
	uint32_t port_id;
	uint32_t if_nameunit;
	uint32_t if_peer_id;
	uint8_t  if_peer_macaddr[6];
};

struct rte_wigig_recovery_info {
	uint32_t port_id;
};

struct rte_wigig_client_ops {
	void (*link_up)(struct rte_wigig_link_updown_info *data);
	void (*link_down)(struct rte_wigig_link_updown_info *data);
	void (*wigig_recovery)(struct rte_wigig_recovery_info *data);
};

struct rte_wigig_link_info {
	uint32_t if_nameunit;
	uint32_t if_peer_id;
	char if_name[IFNAMSIZ];
};

#define RTE_WIGIG_MAX_PORTS	4
#define RTE_WIGIG_MAX_LINKS	16

struct rte_wigig_dev_info {
	int data_fd;
	uint32_t port_id;
	uint32_t num_links;
	struct rte_wigig_link_info link[RTE_WIGIG_MAX_LINKS];
};

struct rte_wigig_ops {
	void *(*device_lookup)(uint8_t *mac);
	void (*set_client_ops)(void *dev_opaque, struct rte_wigig_client_ops *ops);
	int (*device_info)(void *dev_opaque, struct rte_wigig_dev_info *di);
	int (*slowpath_tx)(void *dev_opaque, struct rte_mbuf **mbufs, int num_pkts);
	int (*slowpath_rx)(void *dev_opaque, struct rte_mbuf **mbufs, int num_pkts);
};

/*
 * Create a global symbol and corresponding typedef for the function
 * that DPDK clients can use to get the initial pointer to wigig-specific
 * methods.
 */
typedef const struct rte_wigig_ops *rte_wigig_get_ops_t(void);

/* Actual symbol */
extern rte_wigig_get_ops_t rte_wigig_get_ops;

#ifdef __cplusplus
}
#endif

#endif /* _RTE_WIGIG_API_H_ */
