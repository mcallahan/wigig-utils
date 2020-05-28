#ifndef DPDK_DHD_
#define DPDK_DHD_

#include <linux/types.h>
#include <linux/ioctl.h>

/* ioctl payloads */
struct dhd_attach_request {
	uint8_t mac_addr[IFHWADDRLEN];
	char if_name[IFNAMSIZ];
	int if_index;
	uint16_t pci_domain;
	uint8_t pci_bus;
	uint8_t pci_devid;
	uint8_t pci_function;
};

#define DHD_TYPE 'd'
#define DPDK_DHD_ATTACH _IOWR(DHD_TYPE, 1, struct dhd_attach_request)
#define DPDK_DHD_START _IO(DHD_TYPE, 2)
#define DPDK_DHD_STOP _IO(DHD_TYPE, 3)

/* Control packet payloads */
struct dhd_pkt_header {
	uint8_t cmd_code;
	uint8_t cmd_flags;
	uint16_t cmd_len;
	uint32_t cmd_seqno;
	int32_t cmd_result;
	uint32_t cmd_reserved;
	uint8_t cmd_payload[];
};

struct dhd_data_header {
	int16_t peer_index;
	int16_t link_id;
	uint32_t lifetime;
};

#define DHD_ETH_TYPE_DATA 0xFBCD
#define DHD_ETH_TYPE_CFG  0xFBCE

#define DHD_CMDOP_TX_DATA  0x01
#define DHD_CMDOP_RX_DATA  0x02
#define DHD_CMDOP_ADD_LINK 0x03
#define DHD_CMDOP_DEL_LINK 0x04
#define DHD_CMDOP_SET_KEY  0x05
#define DHD_CMDOP_IOCTL    0x06
#define DHD_CMDOP_EVENT    0x07
#define DHD_CMDOP_REGISTER 0x08
#define DHD_CMDOP_ADD_DEV  0x0a

#define DHD_CMDOP_FIRST    DHD_CMDOP_TX_DATA
#define DHD_CMDOP_LAST     DHD_CMDOP_ADD_DEV

struct dhd_cmd_register_req {
	uint32_t max_peers;
};

struct dhd_cmd_add_dev_req {
	int32_t dev_name_unit;
	int32_t dev_peer_index;
	int32_t dev_ifindex;
	char dev_ifname[IFNAMSIZ];
};

struct dhd_cmd_add_link_req {
	int32_t peer_index; /* Link index within single Wigig instance */
	int32_t tx_link_id; /* TX Link */
	int32_t rx_link_id; /* RX Link */
};

struct dhd_cmd_del_link_req {
	int32_t peer_index;
};

struct dhd_cmd_set_key {
	int32_t peer_index;
	uint8_t mac_addr[IFHWADDRLEN];
	uint16_t key_len;
	uint8_t key_data[];
};

#endif /* DPDK_DHD_ */
