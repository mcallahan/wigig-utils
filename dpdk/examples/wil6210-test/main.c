/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2016 Intel Corporation. All rights reserved.
 *   Copyright(c) 2019 Facebook Inc. Corporation. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/queue.h>
#include <netinet/in.h>
#include <setjmp.h>
#include <stdarg.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>

#include <rte_common.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_eal.h>
#include <rte_launch.h>
#include <rte_atomic.h>
#include <rte_cycles.h>
#include <rte_prefetch.h>
#include <rte_lcore.h>
#include <rte_per_lcore.h>
#include <rte_branch_prediction.h>
#include <rte_interrupts.h>
#include <rte_random.h>
#include <rte_debug.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_net.h>
#include <rte_version.h>

uint16_t read_terra_slowpath(struct rte_mbuf **rx_pkts, const uint16_t nb_pkts);
uint16_t write_terra_slowpath(struct rte_mbuf **tx_pkts, uint16_t nb_pkts);
bool terra_port_qid_to_link(uint16_t port_id, uint16_t qid, uint16_t *link_id);
bool terra_link_to_port_qid(uint16_t link_id, uint16_t *port_id, uint16_t *qid);

#define RX_QUEUES 1
#define TX_QUEUES 1

static volatile bool force_quit;

#define RTE_LOGTYPE_WILTEST RTE_LOGTYPE_USER1

#define NB_MBUF   8192

#define MAX_PKT_BURST 32
#define BURST_TX_DRAIN_US 100 /* TX drain every ~100us */
#define MEMPOOL_CACHE_SIZE 256

/*
 * Configurable number of RX/TX ring descriptors
 */
#define RTE_TEST_RX_DESC_DEFAULT 128
#define RTE_TEST_TX_DESC_DEFAULT 512
static uint16_t nb_rxd = RTE_TEST_RX_DESC_DEFAULT;
static uint16_t nb_txd = RTE_TEST_TX_DESC_DEFAULT;

/* ethernet addresses of ports */
static struct ether_addr wiltest_ports_eth_addr[RTE_MAX_ETHPORTS];

/* mask of enabled ports */
static uint32_t wiltest_enabled_port_mask = 0;

static unsigned int wiltest_rx_queue_per_lcore = 1;

#define MAX_RX_QUEUE_PER_LCORE 16
#define MAX_TX_QUEUE_PER_PORT 16
struct lcore_queue_conf {
  unsigned n_rx_port;
  unsigned rx_port_list[MAX_RX_QUEUE_PER_LCORE];
} __rte_cache_aligned;
struct lcore_queue_conf lcore_queue_conf[RTE_MAX_LCORE];

static struct rte_eth_dev_tx_buffer *tx_buffer[RTE_MAX_ETHPORTS][TX_QUEUES];

static const struct rte_eth_conf port_conf = {
	.rxmode = {
		.offloads = DEV_RX_OFFLOAD_CHECKSUM,
		.split_hdr_size = 0,
	},
	.txmode = {
		.mq_mode = ETH_MQ_TX_NONE,
		.offloads = DEV_TX_OFFLOAD_IPV4_CKSUM |
		    DEV_TX_OFFLOAD_UDP_CKSUM | DEV_TX_OFFLOAD_TCP_CKSUM,
	},
};

struct rte_mempool * wiltest_pktmbuf_pool = NULL;

/* Per-port statistics struct */
struct wiltest_port_statistics {
	uint64_t tx;
	uint64_t rx;
	uint64_t dropped;
} __rte_cache_aligned;
struct wiltest_port_statistics port_statistics[RTE_MAX_ETHPORTS];

#define MAX_TIMER_PERIOD 86400 /* 1 day max */
/* A tsc-based timer responsible for triggering statistics printout */
static uint64_t timer_period = MAX_TIMER_PERIOD;

/* Print out statistics on packets dropped */
static void
print_stats(void)
{
	uint64_t total_packets_dropped, total_packets_tx, total_packets_rx;
	unsigned portid;

	total_packets_dropped = 0;
	total_packets_tx = 0;
	total_packets_rx = 0;

	const char clr[] = { 27, '[', '2', 'J', '\0' };
	const char topLeft[] = { 27, '[', '1', ';', '1', 'H','\0' };

		/* Clear screen and move to top left */
	printf("%s%s", clr, topLeft);

	printf("\nPort statistics ====================================");

	for (portid = 0; portid < RTE_MAX_ETHPORTS; portid++) {
		/* skip disabled ports */
		if ((wiltest_enabled_port_mask & (1 << portid)) == 0)
			continue;
		printf("\nStatistics for port %u ------------------------------"
			   "\nPackets sent: %24"PRIu64
			   "\nPackets received: %20"PRIu64
			   "\nPackets dropped: %21"PRIu64,
			   portid,
			   port_statistics[portid].tx,
			   port_statistics[portid].rx,
			   port_statistics[portid].dropped);

		total_packets_dropped += port_statistics[portid].dropped;
		total_packets_tx += port_statistics[portid].tx;
		total_packets_rx += port_statistics[portid].rx;
	}
	printf("\nAggregate statistics ==============================="
		   "\nTotal packets sent: %18"PRIu64
		   "\nTotal packets received: %14"PRIu64
		   "\nTotal packets dropped: %15"PRIu64,
		   total_packets_tx,
		   total_packets_rx,
		   total_packets_dropped);
	printf("\n====================================================\n");
}

static inline void
wiltest_set_tx_offload(struct rte_mbuf *m)
{
	/*
	 * Parse the packet if sender did not do it for us. Then setup TX offload
	 * flags based on packet type.
	 */
	if (!m->packet_type) {
		struct rte_net_hdr_lens hdr_lens;

		m->packet_type = rte_net_get_ptype(m, &hdr_lens,
				RTE_PTYPE_L2_MASK | RTE_PTYPE_L3_MASK
				| RTE_PTYPE_L4_MASK);
		m->l2_len = hdr_lens.l2_len;
		m->l3_len = hdr_lens.l3_len;
		m->l4_len = hdr_lens.l4_len;
	}

	switch (m->packet_type & RTE_PTYPE_L3_MASK) {
	case RTE_PTYPE_L3_IPV4:
	case RTE_PTYPE_L3_IPV4_EXT:
		m->ol_flags |= PKT_TX_IPV4 | PKT_TX_IP_CKSUM;
		break;
	case RTE_PTYPE_L3_IPV6:
	case RTE_PTYPE_L3_IPV6_EXT:
		m->ol_flags |= PKT_TX_IPV6;
		break;
	default:
		break;
	}

	switch (m->packet_type & RTE_PTYPE_L4_MASK) {
	case RTE_PTYPE_L4_TCP:
		m->ol_flags |= PKT_TX_TCP_CKSUM;
		break;
	case RTE_PTYPE_L4_UDP:
		m->ol_flags |= PKT_TX_UDP_CKSUM;
		break;
	default:
		break;
	}

}

static void
wiltest_simple_forward(struct rte_mbuf *m)
{
	uint16_t dst_port, dst_queue;
	int sent;
	struct rte_eth_dev_tx_buffer *buffer;

	/* Cache this in routing module. Used here as an example */
	if (!terra_link_to_port_qid(m->udata64, &dst_port, &dst_queue)) {
		rte_pktmbuf_free(m);
		return;
	}

	if ((wiltest_enabled_port_mask & (1 << dst_port)) == 0) {
		printf("Fwd pkt to non enabled port %d -> Drop\n", dst_port);
		rte_pktmbuf_free(m);
		return;
	}

	/* Pass desired peer in udata64 */
	m->udata64 = dst_queue;

	buffer = tx_buffer[dst_port][0];
	sent = rte_eth_tx_buffer(dst_port, 0, buffer, m);
	if (sent)
		port_statistics[dst_port].tx += sent;
}

/* main processing loop */
static void
wiltest_main_loop(void)
{
	struct rte_mbuf *pkts_burst[MAX_PKT_BURST];
	struct rte_mbuf *m;
	int sent;
	unsigned lcore_id;
	uint64_t prev_tsc, diff_tsc, cur_tsc, timer_tsc;
	unsigned i, j, portid, qid, nb_rx;
	struct lcore_queue_conf *qconf;
	const uint64_t drain_tsc = (rte_get_tsc_hz() + US_PER_S - 1) / US_PER_S *
			BURST_TX_DRAIN_US;
	struct rte_eth_dev_tx_buffer *buffer;

	prev_tsc = 0;
	timer_tsc = 0;

	lcore_id = rte_lcore_id();
	qconf = &lcore_queue_conf[lcore_id];

	if (qconf->n_rx_port == 0) {
		RTE_LOG(INFO, WILTEST, "lcore %u has nothing to do\n", lcore_id);
		return;
	}

	RTE_LOG(INFO, WILTEST, "entering main loop on lcore %u\n", lcore_id);

	for (i = 0; i < qconf->n_rx_port; i++) {

		portid = qconf->rx_port_list[i];
		RTE_LOG(INFO, WILTEST, " -- lcoreid=%u portid=%u\n", lcore_id,
			portid);

	}

	while (!force_quit) {

		cur_tsc = rte_rdtsc();

		/*
		 * TX burst queue drain
		 */
		diff_tsc = cur_tsc - prev_tsc;
		if (unlikely(diff_tsc > drain_tsc)) {

			for (i = 0; i < qconf->n_rx_port; i++) {
				portid = qconf->rx_port_list[i];
			 	for (qid = 0; qid < TX_QUEUES; qid++) {
					buffer = tx_buffer[portid][qid];

					sent = rte_eth_tx_buffer_flush(portid, qid, buffer);
					if (sent)
						port_statistics[portid].tx += sent;
				}
			}

#if 0
			/* if timer is enabled */
			if (timer_period > 0) {

				/* advance the timer */
				timer_tsc += diff_tsc;

				/* if timer has reached its timeout */
				if (unlikely(timer_tsc >= timer_period)) {
					/* do this only on master core */
					if (lcore_id == rte_get_master_lcore()) {
						print_stats();
						/* reset the timer */
						timer_tsc = 0;
					}
				}
			}
#else
			(void)print_stats;
			(void)timer_tsc;
#endif

			prev_tsc = cur_tsc;
		}

		/*
		 * Read packet from RX queues
		 */
		for (i = 0; i < qconf->n_rx_port; i++) {

			portid = qconf->rx_port_list[i];
			for (qid = 0; qid < RX_QUEUES; qid++) {
				nb_rx = rte_eth_rx_burst(portid, qid,
							 pkts_burst, MAX_PKT_BURST);

				port_statistics[portid].rx += nb_rx;
				for (j = 0; j < nb_rx; j++) {
					m = pkts_burst[j];
					write_terra_slowpath(&m, 1);
				}
			}
		}
	}
}

static int
wiltest_launch_one_lcore(__attribute__((unused)) void *dummy)
{
	wiltest_main_loop();
	return 0;
}

/* display usage */
static void
wiltest_usage(const char *prgname)
{
	printf("%s [EAL options] -- -p PORTMASK [-q NQ]\n"
	       "  -p PORTMASK: hexadecimal bitmask of ports to configure\n"
	       "  -q NQ: number of queue (=ports) per lcore (default is 1)\n"
	       "  -T PERIOD: statistics will be refreshed each PERIOD seconds "
	       "(0 to disable, 10 default, 86400 maximum)\n",
	       prgname);
}

static int
wiltest_parse_portmask(const char *portmask)
{
	char *end = NULL;
	unsigned long pm;

	/* parse hexadecimal string */
	pm = strtoul(portmask, &end, 16);
	if ((portmask[0] == '\0') || (end == NULL) || (*end != '\0'))
		return -1;

	if (pm == 0)
		return -1;

	return pm;
}

static unsigned int
wiltest_parse_nqueue(const char *q_arg)
{
	char *end = NULL;
	unsigned long n;

	/* parse hexadecimal string */
	n = strtoul(q_arg, &end, 10);
	if ((q_arg[0] == '\0') || (end == NULL) || (*end != '\0'))
		return 0;
	if (n == 0)
		return 0;
	if (n >= MAX_RX_QUEUE_PER_LCORE)
		return 0;

	return n;
}

static int
wiltest_parse_timer_period(const char *q_arg)
{
	char *end = NULL;
	int n;

	/* parse number string */
	n = strtol(q_arg, &end, 10);
	if ((q_arg[0] == '\0') || (end == NULL) || (*end != '\0'))
		return -1;
	if (n >= MAX_TIMER_PERIOD)
		return -1;

	return n;
}

static const char short_options[] =
	"p:"  /* portmask */
	"q:"  /* number of queues */
	"T:"  /* timer period */
	;

enum {
	/* long options mapped to a short option */

	/* first long only option value must be >= 256, so that we won't
	 * conflict with short options */
	CMD_LINE_OPT_MIN_NUM = 256,
};

static const struct option lgopts[] = {
	{NULL, 0, 0, 0}
};

/* Parse the argument given in the command line of the application */
static int
wiltest_parse_args(int argc, char **argv)
{
	int opt, ret, timer_secs;
	char **argvopt;
	int option_index;
	char *prgname = argv[0];

	argvopt = argv;

	while ((opt = getopt_long(argc, argvopt, short_options,
				  lgopts, &option_index)) != EOF) {

		switch (opt) {
		/* portmask */
		case 'p':
			wiltest_enabled_port_mask = wiltest_parse_portmask(optarg);
			if (wiltest_enabled_port_mask == 0) {
				printf("invalid portmask\n");
				wiltest_usage(prgname);
				return -1;
			}
			break;

		/* nqueue */
		case 'q':
			wiltest_rx_queue_per_lcore = wiltest_parse_nqueue(optarg);
			if (wiltest_rx_queue_per_lcore == 0) {
				printf("invalid queue number\n");
				wiltest_usage(prgname);
				return -1;
			}
			break;

		/* timer period */
		case 'T':
			timer_secs = wiltest_parse_timer_period(optarg);
			if (timer_secs < 0) {
				printf("invalid timer period\n");
				wiltest_usage(prgname);
				return -1;
			}
			timer_period = timer_secs;
			break;

		/* long options */
		case 0:
			break;

		default:
			wiltest_usage(prgname);
			return -1;
		}
	}

	if (optind >= 0)
		argv[optind-1] = prgname;

	ret = optind-1;
	optind = 1; /* reset getopt lib */
	return ret;
}

/* Check the link status of all ports in up to 9s, and print them finally */
static void
check_all_ports_link_status(uint16_t port_num, uint32_t port_mask)
{
#define CHECK_INTERVAL 100 /* 100ms */
#define MAX_CHECK_TIME 90 /* 9s (90 * 100ms) in total */
	uint16_t portid;
	uint8_t count, all_ports_up, print_flag = 0;
	struct rte_eth_link link;

	printf("\nChecking link status");
	fflush(stdout);
	for (count = 0; count <= MAX_CHECK_TIME; count++) {
		if (force_quit)
			return;
		all_ports_up = 1;
		for (portid = 0; portid < port_num; portid++) {
			if (force_quit)
				return;
			if ((port_mask & (1 << portid)) == 0)
				continue;
			memset(&link, 0, sizeof(link));
			rte_eth_link_get_nowait(portid, &link);
			/* print link status if flag set */
			if (print_flag == 1) {
				if (link.link_status)
					printf(
					"Port%d Link Up. Speed %u Mbps - %s\n",
						portid, link.link_speed,
				(link.link_duplex == ETH_LINK_FULL_DUPLEX) ?
					("full-duplex") : ("half-duplex\n"));
				else
					printf("Port %d Link Down\n", portid);
				continue;
			}
			/* clear all_ports_up flag if any link down */
			if (link.link_status == ETH_LINK_DOWN) {
				all_ports_up = 0;
				break;
			}
		}
		/* after finally printing all link status, get out */
		if (print_flag == 1)
			break;

		if (all_ports_up == 0) {
			printf(".");
			fflush(stdout);
			rte_delay_ms(CHECK_INTERVAL);
		}

		/* set the print_flag if all ports up or timeout */
		if (all_ports_up == 1 || count == (MAX_CHECK_TIME - 1)) {
			print_flag = 1;
			printf("done\n");
		}
	}
}

static void
signal_handler(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		printf("\n\nSignal %d received, preparing to exit...\n",
				signum);
		force_quit = true;
	}
}

int
main(int argc, char **argv)
{
	struct lcore_queue_conf *qconf;
	struct rte_eth_dev_info dev_info;
	int ret;
	uint16_t nb_ports;
	uint16_t nb_ports_available;
	uint16_t portid, qid;
	unsigned lcore_id, rx_lcore_id;
	unsigned nb_ports_in_mask = 0;

	/* init EAL */
	ret = rte_eal_init(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Invalid EAL arguments\n");
	argc -= ret;
	argv += ret;

	force_quit = false;
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	/* parse application arguments (after the EAL ones) */
	ret = wiltest_parse_args(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Invalid WILTEST arguments\n");

	/* convert to number of cycles */
	timer_period *= rte_get_timer_hz();

	/* create the mbuf pool */
	wiltest_pktmbuf_pool = rte_pktmbuf_pool_create("mbuf_pool", NB_MBUF,
		MEMPOOL_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE,
		rte_socket_id());
	if (wiltest_pktmbuf_pool == NULL)
		rte_exit(EXIT_FAILURE, "Cannot init mbuf pool\n");

#if RTE_VERSION < RTE_VERSION_NUM(18, 1, 0, 0)
	nb_ports = rte_eth_dev_count();
#else
	nb_ports = rte_eth_dev_count_avail();
#endif
	if (nb_ports == 0)
		rte_exit(EXIT_FAILURE, "No Ethernet ports - bye\n");

	/* If no ports were explicitly enabled, enable all wil6210 ports,
	 * otherwise validate mask to include only wil6210 ports. */
	if (wiltest_enabled_port_mask == 0) {
		for (portid = 0; portid < nb_ports; portid++) {
			rte_eth_dev_info_get(portid, &dev_info);
			if (strcmp(dev_info.driver_name, "net_wil6210") != 0) {
				continue;
			}
			wiltest_enabled_port_mask |= (1 << portid);
			nb_ports_in_mask++;
		}
	} else {
		for (portid = 0; portid < nb_ports; portid++) {
			/* skip ports that are not enabled */
			if ((wiltest_enabled_port_mask & (1 << portid)) == 0)
				continue;

			rte_eth_dev_info_get(portid, &dev_info);
			if (strcmp(dev_info.driver_name, "net_wil6210") != 0) {
				printf("Skipping non-wil6210 port %u, driver %s\n", portid,
				    dev_info.driver_name);
				wiltest_enabled_port_mask &= ~(1 << portid);
				continue;
			}
			nb_ports_in_mask++;
		}
	}

	rx_lcore_id = 1;
	qconf = NULL;

	/* Initialize the port/queue configuration of each logical core */
	for (portid = 0; portid < nb_ports; portid++) {
		/* skip ports that are not enabled */
		if ((wiltest_enabled_port_mask & (1 << portid)) == 0)
			continue;

		/* get the lcore_id for this port */
		while (rte_lcore_is_enabled(rx_lcore_id) == 0 ||
		       lcore_queue_conf[rx_lcore_id].n_rx_port ==
		       wiltest_rx_queue_per_lcore) {
			rx_lcore_id++;
			if (rx_lcore_id >= RTE_MAX_LCORE)
				rte_exit(EXIT_FAILURE, "Not enough cores\n");
		}

		if (qconf != &lcore_queue_conf[rx_lcore_id])
			/* Assigned a new logical core in the loop above. */
			qconf = &lcore_queue_conf[rx_lcore_id];

		qconf->rx_port_list[qconf->n_rx_port] = portid;
		qconf->n_rx_port++;
		printf("Lcore %u: RX port %u\n", rx_lcore_id, portid);
	}

	nb_ports_available = nb_ports;

	/* Initialise each port */
	for (portid = 0; portid < nb_ports; portid++) {
		/* skip ports that are not enabled */
		if ((wiltest_enabled_port_mask & (1 << portid)) == 0) {
			printf("Skipping disabled port %u\n", portid);
			nb_ports_available--;
			continue;
		}
		/* init port */
		printf("Initializing port %u...\n", portid);
		fflush(stdout);
		ret = rte_eth_dev_configure(portid, RX_QUEUES, TX_QUEUES, &port_conf);
		if (ret < 0)
			rte_exit(EXIT_FAILURE, "Cannot configure device: err=%d, port=%u\n",
				  ret, portid);

		ret = rte_eth_dev_adjust_nb_rx_tx_desc(portid, &nb_rxd,
						       &nb_txd);
		if (ret < 0)
			rte_exit(EXIT_FAILURE,
				 "Cannot adjust number of descriptors: err=%d, port=%u\n",
				 ret, portid);

		rte_eth_macaddr_get(portid,&wiltest_ports_eth_addr[portid]);

		fflush(stdout);
		/* init RX queues */
		for (qid = 0; qid < RX_QUEUES; qid++) {
			ret = rte_eth_rx_queue_setup(portid, qid, nb_rxd,
						     rte_eth_dev_socket_id(portid),
						     NULL,
						     wiltest_pktmbuf_pool);
			if (ret < 0)
				rte_exit(EXIT_FAILURE,
					 "rte_eth_rx_queue_setup:err=%d, port=%u, qid=%d\n",
					 ret, portid, qid);
		}

		fflush(stdout);
		/* init TX queues */
		for (qid = 0; qid < TX_QUEUES; qid++) {
			struct rte_eth_dev_tx_buffer *buffer;

			ret = rte_eth_tx_queue_setup(portid, qid, nb_txd,
					rte_eth_dev_socket_id(portid),
					NULL);
			if (ret < 0)
				rte_exit(EXIT_FAILURE,
					"rte_eth_tx_queue_setup:err=%d, port=%u, qid %u\n",
					ret, portid, qid);

			/* Initialize TX buffers */
			buffer = rte_zmalloc_socket("tx_buffer",
					RTE_ETH_TX_BUFFER_SIZE(MAX_PKT_BURST), 0,
					rte_eth_dev_socket_id(portid));
			if (buffer == NULL)
				rte_exit(EXIT_FAILURE, "Cannot allocate buffer for tx on port %u\n",
					portid);

			rte_eth_tx_buffer_init(buffer, MAX_PKT_BURST);

			ret = rte_eth_tx_buffer_set_err_callback(buffer,
					rte_eth_tx_buffer_count_callback,
					&port_statistics[portid].dropped);
			if (ret < 0)
				rte_exit(EXIT_FAILURE,
				"Cannot set error callback for tx buffer on port %u\n",
					 portid);
			tx_buffer[portid][qid] = buffer;
		}

		/* Start device */
		ret = rte_eth_dev_start(portid);
		if (ret < 0)
			rte_exit(EXIT_FAILURE, "rte_eth_dev_start:err=%d, port=%u\n",
				  ret, portid);

		printf("done: \n");
		printf("Port %u, MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n\n",
				portid,
				wiltest_ports_eth_addr[portid].addr_bytes[0],
				wiltest_ports_eth_addr[portid].addr_bytes[1],
				wiltest_ports_eth_addr[portid].addr_bytes[2],
				wiltest_ports_eth_addr[portid].addr_bytes[3],
				wiltest_ports_eth_addr[portid].addr_bytes[4],
				wiltest_ports_eth_addr[portid].addr_bytes[5]);

		/* initialize port stats */
		memset(&port_statistics, 0, sizeof(port_statistics));
	}

	if (!nb_ports_available) {
		rte_exit(EXIT_FAILURE,
			"All available ports are disabled. Please set portmask.\n");
	}

	/* Wait for driver to finish handshake with the kernel */
	check_all_ports_link_status(nb_ports, wiltest_enabled_port_mask);

	ret = 0;
	/* launch per-lcore init on every lcore */
	rte_eal_mp_remote_launch(wiltest_launch_one_lcore, NULL, SKIP_MASTER);

	/* Loop, reading packets from kernel */
	while  (true) {
		struct rte_mbuf *m, *mbufs[16];
		uint16_t j, nb_rx;

		nb_rx = read_terra_slowpath(mbufs, MAX_PKT_BURST);
		if (nb_rx == 0)
			break;
		for (j = 0; j < nb_rx; j++) {
			m = mbufs[j];
			wiltest_simple_forward(m);
		}
	}

	/* Wait for other threads to shutdown */
	RTE_LCORE_FOREACH_SLAVE(lcore_id) {
		if (rte_eal_wait_lcore(lcore_id) < 0) {
			ret = -1;
			break;
		}
	}

	for (portid = 0; portid < nb_ports; portid++) {
		if ((wiltest_enabled_port_mask & (1 << portid)) == 0)
			continue;
		printf("Closing port %d...\n", portid);
		rte_eth_dev_stop(portid);
		rte_eth_dev_close(portid);
		printf("Done\n");
	}
	printf("Bye...\n");

	return ret;
}
