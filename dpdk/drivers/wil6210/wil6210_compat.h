/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2014-2017 Chelsio Communications.
 *   All rights reserved.
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
 *     * Neither the name of Chelsio Communications nor the names of its
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

#ifndef _WIL6210_COMPAT_H_
#define _WIL6210_COMPAT_H_

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#include <rte_common.h>
#include <rte_cycles.h>
#include <rte_byteorder.h>
#include <rte_ether.h>
#include <rte_spinlock.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_memcpy.h>
#include <rte_io.h>
#include <rte_interrupts.h>
#include <rte_bus_pci.h>
#include <rte_version.h>

#include <linux/ethtool.h>

/* Borrow list from dpaa and add missing macros only */
#include "wil6210_list.h"

#define __printf(a, b) __attribute__((format(printf, a, b)))

extern void __dbg_printf(int level, int ltype, const char *fmt, ...)
       	__printf(3, 4);

#define dev_printf(level, dev, fmt, args...) \
	__dbg_printf(RTE_LOG_ ## level, RTE_LOGTYPE_PMD, \
	             "%s: " fmt, (dev)->name, ## args)

#define dev_err(dev, args...) dev_printf(ERR, dev, args)
#define dev_info(dev, args...) dev_printf(INFO, dev, args)
#define dev_warn(dev, args...) dev_printf(WARNING, dev, args)
#define dev_dbg(dev, args...) dev_printf(DEBUG, dev, args)

#define dbg_printf(level, fmt, args...) \
	__dbg_printf(RTE_LOG_ ## level, RTE_LOGTYPE_PMD, \
	             fmt, ## args)

#define pr_err(fmt, args...) dbg_printf(ERR, fmt, args)
#define pr_warn(fmt, args...) dbg_printf(WARNING, fmt, args)
#define pr_info(fmt, args...) dbg_printf(INFO, fmt, rgs)
#define BUG() rte_panic("BUG at %s:%d", __func__, __LINE__)

#define ASSERT(x) do {\
	if (!(x)) \
		rte_panic("ASSERT: \"" #x "\" at %s:%d\n", \
			  __func__, __LINE__); \
} while (0)
#define BUG_ON(x) ASSERT(!(x))
#define BUILD_BUG_ON(x) RTE_BUILD_BUG_ON(x)

#ifndef WARN_ON
#define WARN_ON(x) do { \
	int ret = !!(x); \
	if (unlikely(ret)) \
		pr_warn("WARN_ON: \"" #x "\" at %s:%d\n", __func__, __LINE__); \
} while (0)
#endif

#define __iomem

#ifndef BIT
#define BIT(n) (1 << (n))
#endif

#define L1_CACHE_SHIFT  6
#define L1_CACHE_BYTES  BIT(L1_CACHE_SHIFT)

#define PAGE_SHIFT  12
#define WIL6210_ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))
#define PTR_ALIGN(p, a) ((typeof(p))WIL6210_ALIGN((unsigned long)(p), (a)))

#define VLAN_HLEN 4

#define rmb()     rte_rmb() /* dpdk rte provided rmb */
#define wmb()     rte_wmb() /* dpdk rte provided wmb */

typedef uint8_t   u8;
typedef int8_t    s8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef int32_t   s32;
typedef uint64_t  u64;
typedef int       bool;

typedef rte_iova_t  dma_addr_t;

#ifndef __le16
#define __le16	uint16_t
#endif
#ifndef __le32
#define __le32	uint32_t
#endif
#ifndef __le64
#define __le64	uint64_t
#endif
#ifndef __be16
#define __be16	uint16_t
#endif
#ifndef __be32
#define __be32	uint32_t
#endif
#ifndef __be64
#define __be64	uint64_t
#endif
#ifndef __u8
#define __u8	uint8_t
#endif
#ifndef __u16
#define __u16	uint16_t
#endif
#ifndef __u32
#define __u32	uint32_t
#endif
#ifndef __u64
#define __u64	uint64_t
#endif

#define FALSE	0
#define TRUE	1
#define false	0
#define true	1

#define min(a, b) RTE_MIN(a, b)
#define max(a, b) RTE_MAX(a, b)

/*
 * round up val _p to a power of 2 size _s
 */
#define wil6210_roundup(_p, _s) (((unsigned long)(_p) + (_s - 1)) & ~(_s - 1))

#undef container_of
#define container_of(ptr, type, member) ({ \
		typeof(((type *)0)->member)(*__mptr) = (ptr); \
		(type *)((char *)__mptr - offsetof(type, member)); })

#define ARRAY_SIZE(arr) ((ssize_t)RTE_DIM(arr))

#define upper_32_bits(n) ((uint32_t)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((uint32_t)(n))

#define cpu_to_be16(o) rte_cpu_to_be_16(o)
#define cpu_to_be32(o) rte_cpu_to_be_32(o)
#define cpu_to_be64(o) rte_cpu_to_be_64(o)
#define cpu_to_le16(o) rte_cpu_to_le_16(o)
#define cpu_to_le32(o) rte_cpu_to_le_32(o)
#define cpu_to_le64(o) rte_cpu_to_le_64(o)
#define be16_to_cpu(o) rte_be_to_cpu_16(o)
#define be32_to_cpu(o) rte_be_to_cpu_32(o)
#define be64_to_cpu(o) rte_be_to_cpu_64(o)
#define le16_to_cpu(o) rte_le_to_cpu_16(o)
#define le32_to_cpu(o) rte_le_to_cpu_32(o)
#define le64_to_cpu(o) rte_le_to_cpu_64(o)

#define le16_to_cpus(o) rte_le_to_cpu_16(o)
#define le32_to_cpus(o) rte_le_to_cpu_32(o)

#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#define DELAY(x) rte_delay_us(x)
#define udelay(x) DELAY(x)
#define msleep(x) DELAY(1000 * (x))
#define usleep_range(min, max) msleep(DIV_ROUND_UP(min, 1000))

#define jiffies 0
#define msecs_to_jiffies(m)	((ulong)(m))
#define jiffies_to_msecs(j)	((uint)(j))

static inline uint8_t hweight32(uint32_t word32)
{
	uint32_t res = word32 - ((word32 >> 1) & 0x55555555);

	res = (res & 0x33333333) + ((res >> 2) & 0x33333333);
	res = (res + (res >> 4)) & 0x0F0F0F0F;
	res = res + (res >> 8);
	return (res + (res >> 16)) & 0x000000FF;

} /* weight32 */

/**
 * wil6210_fls - find last (most-significant) bit set
 * @x: the word to search
 *
 * This is defined the same way as ffs.
 * Note wil6210_fls(0) = 0, wil6210_fls(1) = 1, wil6210_fls(0x80000000) = 32.
 */
static inline int wil6210_fls(int x)
{
	return x ? sizeof(x) * 8 - __builtin_clz(x) : 0;
}

/**
 * wil6210_ffs - find first bit set
 * @x: the word to search
 */
static inline int wil6210_ffs(int x)
{
	return x ? __builtin_ffs(x) : 0;
}

static inline unsigned long ilog2(unsigned long n)
{
	unsigned int e = 0;

	while (n) {
		if (n & ~((1 << 8) - 1)) {
			e += 8;
			n >>= 8;
			continue;
		}

		if (n & ~((1 << 4) - 1)) {
			e += 4;
			n >>= 4;
		}

		for (;;) {
			n >>= 1;
			if (n == 0)
				break;
			e++;
		}
	}

	return e;
}

static inline unsigned int readl(volatile void __iomem *addr)
{
	return rte_read32(addr);
}

static inline void writel(unsigned int val, volatile void __iomem *addr)
{
	rte_write32(val, addr);
}

static inline void writeq(u64 val, volatile void __iomem *addr)
{
	writel(val, addr);
	writel(val >> 32, (void *)((uintptr_t)addr + 4));
}

static inline void writel_relaxed(unsigned int val, volatile void __iomem *addr)
{
	rte_write32_relaxed(val, addr);
}

#define __raw_readl(p) (*(const volatile unsigned int *)(p))
#define __raw_writel(v, p) {*(volatile unsigned int *)(p) = (v); }

struct wireless_dev;
struct wiphy;
struct net_device;
struct cfg80211_mgmt_tx_params;
struct cfg80211_sched_scan_request;
struct station_info;
struct key_params;
struct rte_eth_dev;

size_t strlcat(char *dest, const char *src, size_t count);
size_t strlcpy(char *dest, const char *src, size_t size);

u32 crc32_le(u32 crc, const u8 *ptr, u32 length);

#define min_t(type, x, y) ({                    \
	type __min1 = (x);                      \
	type __min2 = (y);                      \
	__min1 < __min2 ? __min1 : __min2; })

#define max_t(type, x, y) ({                    \
	type __max1 = (x);                      \
	type __max2 = (y);                      \
	__max1 > __max2 ? __max1 : __max2; })

#define ENOTSUPP ENOTSUP
#define ETH_ALEN 6
#define U8_MAX	UINT8_MAX
#define S8_MAX	INT8_MAX
#define S8_MIN	INT8_MIN

#define __init
#define __user
#define __force
#define __acquires(x)
#define __releases(x)

#define __packed __rte_packed

#define prefetch(x) rte_prefetch0(x)

#define atomic_t rte_atomic32_t

#define atomic_t                rte_atomic32_t
#define atomic_read(v)          rte_atomic32_read(v)
#define atomic_set(v, i)        rte_atomic32_set(v, i)
#define atomic_inc(v)           rte_atomic32_inc(v)
#define atomic_dec(v)           rte_atomic32_dec(v)
#define atomic_add(i, v)        rte_atomic32_add(v, i)
#define atomic_sub(i, v)        rte_atomic32_sub(v, i)
#define atomic_inc_and_test(v)  rte_atomic32_inc_and_test(v)
#define atomic_dec_and_test(v)  rte_atomic32_dec_and_test(v)
#define atomic_inc_return(v)    rte_atomic32_add_return(v, 1)
#define atomic_dec_return(v)    rte_atomic32_sub_return(v, 1)
#define atomic_sub_and_test(i, v) (rte_atomic32_sub_return(v, i) == 0)

#define spinlock_t		rte_spinlock_t
#define __SPIN_LOCK_UNLOCKED(x)	RTE_SPINLOCK_INITIALIZER
#define DEFINE_SPINLOCK(x)	spinlock_t x = __SPIN_LOCK_UNLOCKED(x)
#define spin_lock_init(x)	rte_spinlock_init(x)
#define spin_lock_destroy(x)
#define spin_lock(x)		rte_spinlock_lock(x)
#define spin_unlock(x)		rte_spinlock_unlock(x)
#define spin_lock_irq(x)	spin_lock(x)
#define spin_unlock_irq(x)	spin_unlock(x)
#define spin_lock_irqsave(x, f) ({f = 0; spin_lock_irq(x);})
#define spin_unlock_irqrestore(x, f) ({spin_unlock_irq(x); f;})
#define spin_lock_bh(x)		rte_spinlock_lock(x)
#define spin_unlock_bh(x)	rte_spinlock_unlock(x)

#ifndef IRQ_HANDLED
typedef int irqreturn_t;
#define IRQ_HANDLED 1
#define IRQ_NONE 2
#define IRQ_WAKE_THREAD 3
#endif

typedef int netdev_tx_t;
#define NETDEV_TX_OK	0x00
#define NETDEV_TX_BUSY	0x10

#define MAX_ERRNO       4095
#define IS_ERR(x) (((unsigned long)x) >= (unsigned long)-MAX_ERRNO)
#define ERR_PTR(error) ((void *)(long)error)
#define PTR_ERR(error) ((long)(void *)error)
typedef unsigned long cycles_t;

#ifndef BITS_TO_LONGS
#define BITS_PER_LONG (__SIZEOF_LONG__ * 8)
#define BITS_TO_LONGS(bits) (((bits)+BITS_PER_LONG-1)/BITS_PER_LONG)
#endif

#ifndef DECLARE_BITMAP
#define DECLARE_BITMAP(name,bits) ulong name[BITS_TO_LONGS(bits)]
#endif

#define bitmap_zero(b,bits) memset(&(b), 0, sizeof(ulong) * BITS_TO_LONGS(bits))
#define bitmap_and(dst, src1, src2, bits) __bitmap_and(dst, src1, src2, bits)

unsigned long __bitmap_and(unsigned long *dst, const unsigned long *src1,
                               const unsigned long *src2, unsigned int bits);

#ifndef ALIGN
#define ALIGN(x, a) (((x) + ((__typeof__(x))(a) - 1)) & \
			~((__typeof__(x))(a) - 1))
#endif

enum {
	DUMP_PREFIX_NONE,
	DUMP_PREFIX_ADDRESS,
	DUMP_PREFIX_OFFSET
};

#define kmalloc(size, f) rte_malloc("wil6210", (size), 0)
#define kzalloc(size, f) rte_zmalloc("wil6210", (size), 0)
#define kcalloc(items, size, f)  rte_zmalloc("wil6210", ((size) * (items)), 0)
#define kfree rte_free

/* Bits */
/*
 * WARNING: non atomic version.
 */
static inline void
set_bit(unsigned long nr, void *addr)
{
	int *m = ((int *)addr) + (nr >> 5);
	*m |= 1 << (nr & 31);
}

static inline int
test_bit(int nr, const void *addr)
{
	return (1UL & (((const int *)addr)[nr >> 5] >> (nr & 31))) != 0UL;
}

/*
 * WARNING: non atomic version.
 */
static inline void
clear_bit(unsigned long nr, void *addr)
{
	int *m = ((int *)addr) + (nr >> 5);
	*m &= ~(1 << (nr & 31));
}

/*
 * WARNING: non atomic version.
 */
static inline int
test_and_clear_bit(unsigned long nr, void *addr)
{
	unsigned long mask = 1 << (nr & 0x1f);
	int *m = ((int *)addr) + (nr >> 5);
	int old = *m;

	*m = old & ~mask;
	return (old & mask) != 0;
}

/*
 * WARNING: non atomic version.
 */
static inline int
test_and_set_bit(unsigned long nr, void *addr)
{
	unsigned long mask = 1 << (nr & 0x1f);
	int *m = ((int *)addr) + (nr >> 5);
	int old = *m;

	*m = old | mask;
	return (old & mask) != 0;
}

/* Locks */
struct mutex {
	pthread_mutex_t lock;
};

#define DEFINE_MUTEX(_m)                      \
    struct mutex _m = (struct mutex) {        \
	    .lock = PTHREAD_MUTEX_INITIALIZER \
    }

static inline void mutex_init(struct mutex *m)
{
	pthread_mutex_init(&m->lock, NULL);
}

static inline void mutex_lock(struct mutex *m)
{
	pthread_mutex_lock(&m->lock);
}

static inline int mutex_trylock(struct mutex *m)
{
	return pthread_mutex_trylock(&m->lock);
}

static inline void mutex_unlock(struct mutex *m)
{
	pthread_mutex_unlock(&m->lock);
}

static inline int mutex_is_locked(struct mutex *m __rte_unused)
{
	return 1;
}

struct rw_semaphore {
	pthread_rwlock_t lock;
};

static inline void init_rwsem(struct rw_semaphore *sem)
{
	pthread_rwlock_init(&sem->lock, NULL);
}

static inline void down_read(struct rw_semaphore *sem)
{
	pthread_rwlock_rdlock(&sem->lock);
}

static inline bool down_read_trylock(struct rw_semaphore *sem)
{
	return (pthread_rwlock_tryrdlock(&sem->lock) == 0);
}

static inline void up_read(struct rw_semaphore *sem)
{
	pthread_rwlock_unlock(&sem->lock);
}

static inline void down_write(struct rw_semaphore *sem)
{
	pthread_rwlock_wrlock(&sem->lock);
}

static inline void up_write(struct rw_semaphore *sem)
{
	pthread_rwlock_unlock(&sem->lock);
}

static inline void
lockdep_assert_held(void *p __rte_unused)
{
}

#define NL80211_BAND_60GHZ	60		/* Arbitrary */
#define IEEE80211_GCMP_PN_LEN	6
/* 802.11ad extends maximum MSDU size for DMG (freq > 40Ghz) networks
 * to 7920 bytes, see 8.2.3 General frame format
 */
#define IEEE80211_MAX_DATA_LEN_DMG	7920
#define IEEE80211_FTYPE_DATA	0x8

#define IEEE80211_MAX_SSID_LEN	32

/* Constants used for BA */
#define WLAN_STATUS_SUCCESS 0
#define WLAN_STATUS_INVALID_QOS_PARAM 38

struct ieee80211_channel {
	int dummy;
};

struct timer_list {
	int dummy;
};

struct work_struct;
typedef void (*work_fn_t)(struct work_struct *work);

struct work_struct {
	pthread_mutex_t mutex;
	work_fn_t fn;
	bool running;
	struct list_head list;
	pthread_t thread;
};

void INIT_WORK(struct work_struct *work, work_fn_t fn);
void cancel_work_sync(struct work_struct *work);
bool schedule_work(struct work_struct *work, char *name);

struct workqueue_struct {
	pthread_mutex_t mutex;
	pthread_cond_t  cond;
	pthread_t thread;
	u32 queued;
	bool running;
	struct list_head work_list;
};

struct workqueue_struct *create_singlethread_workqueue(const char *name);
void destroy_workqueue(struct workqueue_struct *wq);
void flush_workqueue(struct workqueue_struct *wq);
bool queue_work(struct workqueue_struct *wq, struct work_struct *work);

struct completion {
	bool complete;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
};

void complete(struct completion *c);
void init_completion(struct completion *c);
void reinit_completion(struct completion *c);
int wait_for_completion_timeout(struct completion *c, ulong to);


struct wait_queue_head
{
	int dummy;
};
typedef struct wait_queue_head wait_queue_head_t;

void init_waitqueue_head(struct wait_queue_head *wq_head);
void wake_up_interruptible(struct wait_queue_head *wq);

void print_hex_dump_level(int level, const char *prefix_str, int prefix_type,
			  int rowsize, int groupsize, const void *buf,
			  size_t len, bool ascii);

void print_hex_dump_debug(int type, const char *prefix_str, int prefix_type,
			  int rowsize, int groupsize, const void *buf,
			  size_t len, bool ascii);

/* Wholesale linux types. To be removed */
enum nl80211_iftype {
	NL80211_IFTYPE_UNSPECIFIED,
	NL80211_IFTYPE_ADHOC,
	NL80211_IFTYPE_STATION,
	NL80211_IFTYPE_AP,
	NL80211_IFTYPE_AP_VLAN,
	NL80211_IFTYPE_WDS,
	NL80211_IFTYPE_MONITOR,
	NL80211_IFTYPE_MESH_POINT,
	NL80211_IFTYPE_P2P_CLIENT,
	NL80211_IFTYPE_P2P_GO,
	NL80211_IFTYPE_P2P_DEVICE,
	NL80211_IFTYPE_OCB,
	NL80211_IFTYPE_NAN,

	/* keep last */
	NUM_NL80211_IFTYPES,
	NL80211_IFTYPE_MAX = NUM_NL80211_IFTYPES - 1
};

/*
 * Minimal definitions to prevent re-arranging Linux driver structure
 * dependencies too much.
 */
struct platform_device;
struct platform_device *platform_device_alloc(const char *name, int unit);
int platform_device_add_data(struct platform_device *pdev, void *pdata,
    size_t datalen);
void platform_device_put(struct platform_device *pdev);
int platform_device_add(struct platform_device *pdev);
void platform_device_unregister(struct platform_device *pdev);
struct wil6210_priv;
struct wil6210_vif;

struct device {
	char name[RTE_DEV_NAME_MAX_LEN];
};

struct wiphy {
	struct device *dev;
	struct wil6210_priv *priv;
	uint8_t perm_addr[ETH_ALEN];
	char fw_version[ETHTOOL_FWVERS_LEN];
	uint8_t retry_short;
};

static inline int wiphy_register(struct wiphy *wiphy)
{
	return 0;
}

static inline void wiphy_unregister(struct wiphy *wiphy)
{
}

#define wiphy_priv(wiphy) (wiphy)->priv
#define wiphy_dev(wiphy) (wiphy)->dev

struct wireless_dev {
	struct wil6210_priv *priv;
	struct wiphy *wiphy;
	struct net_device *netdev;
	enum nl80211_iftype iftype;
};

#define wdev_priv(wdev) (wdev)->priv

enum {
	ARPHRD_ETHER = 1,
	ARPHRD_IEEE80211_RADIOTAP = 2,
};

struct netdev_stats {
	unsigned long	rx_packets;
	unsigned long	tx_packets;
	unsigned long	rx_bytes;
	unsigned long	tx_bytes;
	unsigned long	rx_errors;
	unsigned long	tx_errors;
	unsigned long	rx_dropped;
	unsigned long	tx_dropped;
};

struct net_device {
	struct device dev;
	uint8_t dev_addr[ETH_ALEN];
	uint8_t perm_addr[ETH_ALEN];
	struct netdev_stats stats;
	int type;
	char name[RTE_DEV_NAME_MAX_LEN];
	struct wireless_dev *ieee80211_ptr;
	struct wil6210_priv *wil;
	struct wil6210_vif  *vif;
};

#define netdev_priv(ndev) (ndev)->vif

#define ASSERT_RTNL()	((void)0)
#define rtnl_lock()
#define rtnl_unlock()

/* dirty */
#define is_valid_ether_addr(addr) \
	is_valid_assigned_ether_addr((const struct ether_addr *)&(addr)[0])
#define ether_addr_equal(addr1, addr2) \
	is_same_ether_addr((const struct ether_addr *)&(addr1)[0], \
	                   (const struct ether_addr *)&(addr2)[0])

#define module_param(param...)
#define MODULE_PARM_DESC(param...)

#define wait_event_interruptible(a, b) (0)

static inline void wil_ether_addr_copy(uint8_t *dst, const uint8_t *src)
{
	memcpy(dst, src, ETH_ALEN);
}
#define ether_addr_copy(_d, _s) wil_ether_addr_copy((_d), (_s))

static inline const char *
__mac_to_str(const uint8_t *addr, size_t len, char buf[18])
{
	snprintf(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X",
	    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
	buf[len - 1] = '\0';
	return buf;
}
#define mac_to_str(addr, buf) __mac_to_str((addr), sizeof(buf), buf)

typedef struct wil_dmamem_t {
	const struct rte_memzone	*dma_mz;
	void				*dma_base;
	dma_addr_t			dma_addr;
} dma_mem_t;

/* DMA APIs */
void *wil_dma_zalloc_coherent(struct wil6210_priv *dev, const char *name,
    uint16_t id, size_t sz, dma_mem_t *m);
void wil_dma_free_coherent(struct wil6210_priv *dev, dma_mem_t *m);

#define get_cycles() rte_get_timer_cycles()

void get_random_bytes(void *buf, int nbytes);

/*
 * definitions from kernel wil6210 UAPI (ioctl)
 * the definitions are used internally by the driver,
 * and the functionality is exposed using nl60g API.
 */

/**
 * operation to perform
 *
 * @wil_mmio_op_mask - bits defining operation,
 * @wil_mmio_addr_mask - bits defining addressing mode
 */
enum wil_memio_op {
	wil_mmio_read = 0,
	wil_mmio_write = 1,
	wil_mmio_op_mask = 0xff,
	wil_mmio_addr_linker = 0 << 8,
	wil_mmio_addr_ahb = 1 << 8,
	wil_mmio_addr_bar = 2 << 8,
	wil_mmio_addr_mask = 0xff00,
};

struct wil_memio {
	uint32_t op; /* enum wil_memio_op */
	uint32_t addr; /* should be 32-bit aligned */
	uint32_t val;
};

struct wil_memio_block {
	uint32_t op; /* enum wil_memio_op */
	uint32_t addr; /* should be 32-bit aligned */
	uint32_t size; /* should be multiple of 4 */
	void *block; /* block contents */
};



#endif /* _WIL6210_COMPAT_H_ */
