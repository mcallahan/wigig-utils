/*
 * Copyright (c) 2012-2017 Qualcomm Atheros, Inc.
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019, Facebook, Inc. All rights reserved.
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
#include <sys/time.h>
#include "wil6210_ethdev.h"
#include <rte_errno.h>
#include <unistd.h>

size_t strlcat(char *dest, const char *src, size_t count)
{
	size_t dsize = strlen(dest);
	size_t len = strlen(src);
	size_t res = dsize + len;

	dest += dsize;
	count -= dsize;
	if (len >= count)
		len = count-1;
	memcpy(dest, src, len);
	dest[len] = 0;
	return res;
}

size_t strlcpy(char *dest, const char *src, size_t size)
{
	size_t ret = strlen(src);

	if (size) {
		size_t len = (ret >= size) ? size - 1 : ret;
		memcpy(dest, src, len);
		dest[len] = '\0';
	}
	return ret;
}

u32 crc32_le(u32 crc, const u8 *ptr, u32 length)
{
	int i;

	while (length--) {
		crc ^= *ptr++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
	}
	return crc;
}

/* Very much soimplified version */
unsigned long __bitmap_and(unsigned long *dst, const unsigned long *src1,
			   const unsigned long *src2, unsigned int bits)
{

	unsigned int i;
	unsigned int num = BITS_TO_LONGS(bits);
	unsigned long result = 0;

	for (i = 0; i < num; i++)
		result |= (dst[i] = src1[i] & src2[i]);
	return result != 0;
}

void complete(struct completion *c)
{
	pthread_mutex_lock(&c->mutex);
	c->complete = true;
	pthread_cond_signal(&c->cond);
	pthread_mutex_unlock(&c->mutex);
}

void init_completion(struct completion *c)
{
	int retval;

	c->complete = false;
	retval = pthread_cond_init(&c->cond, NULL);
	ASSERT(retval == 0);
	retval = pthread_mutex_init(&c->mutex, NULL);
	ASSERT(retval == 0);
}

void reinit_completion(struct completion *c)
{
	c->complete = false;
}

int wait_for_completion_timeout(struct completion *c, ulong wait_ms)
{
	struct timespec ts;
	struct timeval tp;
	int retval = 0;

	gettimeofday(&tp, NULL);

	/* Convert from timeval to timespec */
	ts.tv_sec = tp.tv_sec;
	ts.tv_sec += wait_ms / 1000;
	ts.tv_nsec = tp.tv_usec * 1000;
	ts.tv_nsec += (wait_ms % 1000) * 1000 * 1000;
	if (ts.tv_nsec >= 1000000000ull) {
		ts.tv_sec += 1;
		ts.tv_nsec -= 1000000000ull;
	}

	pthread_mutex_lock(&c->mutex);
	while (!c->complete && retval == 0) {
		retval = pthread_cond_timedwait(&c->cond, &c->mutex, &ts);
	}
	pthread_mutex_unlock(&c->mutex);
	return retval ? 0 : wait_ms;
}

void INIT_WORK(struct work_struct *work, work_fn_t fn)
{
	int retval;

	INIT_LIST_HEAD(&work->list);
	work->running = false;
	work->fn = fn;
	retval = pthread_mutex_init(&work->mutex, NULL);
	ASSERT(retval == 0);
}

void cancel_work_sync(struct work_struct *work)
{
}

static void *work_runner(void *param)
{
	struct work_struct *work = param;

	pthread_mutex_lock(&work->mutex);
	/* check to make sure work was not cancelled */
	if (work->running)
		work->fn(work);

	work->running = false;
	pthread_mutex_unlock(&work->mutex);
	return NULL;
}

bool schedule_work(struct work_struct *work, char *name)
{
	int ret;
	rte_cpuset_t cpuset;

	ret = pthread_mutex_trylock(&work->mutex);
	if (ret != 0)
		return false;

	ret = pthread_create(&work->thread, NULL, work_runner, work);
	if (ret != 0)
		goto fail;

	/* Run control threads on master lcore and set its name */
	CPU_ZERO(&cpuset);
	CPU_SET(rte_get_master_lcore(), &cpuset);
	ret = pthread_setaffinity_np(work->thread, sizeof(cpuset),
				     &cpuset);
	if (ret != 0) {
		RTE_LOG(ERR, PMD, "Unable to set work thread affinity: %s\n",
			strerror(errno));
		goto fail;
	}
	(void)rte_thread_setname(work->thread, name);
	/* set work to be run */
	if (work->fn != NULL)
		work->running = true;

	pthread_mutex_unlock(&work->mutex);
	return true;
fail:
	/* cancel work from being run */
	work->running = false;
	pthread_mutex_unlock(&work->mutex);
	return false;
}

static void *
workqueue_runner(void *param)
{
	struct workqueue_struct *wq = param;
	struct work_struct *work;

	pthread_mutex_lock(&wq->mutex);
	while (wq->running) {
		work_fn_t fn;
		if (list_empty(&wq->work_list)) {
			pthread_cond_wait(&wq->cond, &wq->mutex);
		}
		if (!wq->running)
			break;
		if (list_empty(&wq->work_list))
			continue;
		/* Grab the next work from the list, wark it as pending */
		work = list_entry(wq->work_list.next, struct work_struct, list);
		pthread_mutex_lock(&work->mutex);
		if ((fn = work->fn) != NULL) {
			work->running = true;
			list_del(&work->list);
			INIT_LIST_HEAD(&work->list);
			wq->queued--;
		}
		pthread_mutex_unlock(&work->mutex);
		pthread_mutex_unlock(&wq->mutex);
		if (fn != NULL)
			fn(work);
		/* Mark work as complete */
		pthread_mutex_lock(&work->mutex);
		work->running = false;
		pthread_mutex_unlock(&work->mutex);
		pthread_mutex_lock(&wq->mutex);
	}
	pthread_mutex_unlock(&wq->mutex);
	return NULL;
}

struct workqueue_struct *
create_singlethread_workqueue(const char *name)
{
	struct workqueue_struct *wq;
	rte_cpuset_t cpuset;
	int retval;

	wq = rte_zmalloc("wil6210", sizeof(*wq), 0);
	if (wq == NULL)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&wq->work_list);
	retval = pthread_mutex_init(&wq->mutex, NULL);
	ASSERT(retval == 0);
	retval = pthread_cond_init(&wq->cond, NULL);
	ASSERT(retval == 0);
	wq->running = true;

	retval = pthread_create(&wq->thread, NULL,
	    workqueue_runner, wq);
	if (retval != 0) {
		wq->running = false;
		goto fail;
	}

	/* Run control threads on master lcore and set its name */
	CPU_ZERO(&cpuset);
	CPU_SET(rte_get_master_lcore(), &cpuset);
	retval = pthread_setaffinity_np(wq->thread, sizeof(cpuset), &cpuset);
	if (retval != 0) {
		RTE_LOG(ERR, PMD, "Unable to set wq thread affinity: %s\n",
		    strerror(errno));
		goto fail;
	}
	(void)rte_thread_setname(wq->thread, name);

	return wq;
fail:
	destroy_workqueue(wq);
	return NULL;
}

void
destroy_workqueue(struct workqueue_struct *wq)
{
	pthread_mutex_lock(&wq->mutex);
	if (wq->running) {
		wq->running = false;
		pthread_cond_signal(&wq->cond);
		pthread_mutex_unlock(&wq->mutex);
		pthread_join(wq->thread, NULL);
	} else {
		pthread_mutex_unlock(&wq->mutex);
	}
	pthread_mutex_destroy(&wq->mutex);
	pthread_cond_destroy(&wq->cond);
	rte_free(wq);
}

void flush_workqueue(struct workqueue_struct *wq)
{
}

bool queue_work(struct workqueue_struct *wq, struct work_struct *work)
{
	pthread_mutex_lock(&wq->mutex);
	pthread_mutex_lock(&work->mutex);
	if (list_empty(&work->list)) {
		list_add_tail(&work->list, &wq->work_list);
		wq->queued++;
		if (wq->work_list.next == &work->list) {
			pthread_cond_signal(&wq->cond);
		}
	}
	pthread_mutex_unlock(&work->mutex);
	pthread_mutex_unlock(&wq->mutex);
	return false;
}

void init_waitqueue_head(struct wait_queue_head *wq_head)
{
}

void wake_up_interruptible(struct wait_queue_head *wq)
{
}

/* DMA buffers */
void *wil_dma_zalloc_coherent(struct wil6210_priv *wil, const char *name,
    uint16_t id, size_t size, dma_mem_t *m)
{
	const struct rte_memzone *mz;
	char z_name[RTE_MEMZONE_NAMESIZE];
	unsigned flags;
	size_t align;

	memset(m, 0, sizeof(*m));

	snprintf(z_name, sizeof(z_name), "%s_%s_%#lx_%u",
		 "wl", name, (uintptr_t)wil, id);
	mz = rte_memzone_lookup(z_name);
	if (mz == NULL) {
		/*
		 * Simulate Linux' buddy allocator and return naturally
		 * aligned memory for DMA
		 */
		align = 1UL <<(1 +(63 -__builtin_clzl(size - 1)));

		/*
		 * Earlier DPDK versions had no way to guarantee that
		 * the allocated memory block allocated will be physically
		 * contiguous, so we enforce blocks to be allocated from
		 * huge pages and hope for the best.
		 */
		flags = RTE_MEMZONE_2MB | RTE_MEMZONE_1GB;
#ifdef RTE_MEMZONE_IOVA_CONTIG
		/*
		 * We can request contiguos memory explicitly and page size
		 * is just a mild preference of ours.
		 */
		flags |= RTE_MEMZONE_IOVA_CONTIG | RTE_MEMZONE_SIZE_HINT_ONLY;
#endif
		mz = rte_memzone_reserve_aligned(z_name, size, SOCKET_ID_ANY,
		    flags, align);
	}
	if (mz == NULL) {
		wil_err(wil, "cannot reserve DMA zone for %s:%u %#jx: %s",
			name, id, size, rte_strerror(rte_errno));
		return NULL;
	}

	m->dma_addr = mz->iova;
	if (m->dma_addr == RTE_BAD_IOVA) {
		(void)rte_memzone_free(mz);
		return NULL;
	}

	m->dma_mz = mz;
	m->dma_base = mz->addr;

	return m->dma_base;
}

void wil_dma_free_coherent(struct wil6210_priv *wil, dma_mem_t *m)
{
	int rc;

	rc = rte_memzone_free(m->dma_mz);
	if (rc != 0)
		wil_err(wil, "rte_memzone_free(() failed: %d", rc);

	memset(m, 0, sizeof(*m));
}

void get_random_bytes(void *buf, int nbytes)
{
	union {
		uint64_t value;
		uint8_t bytes[sizeof(uint64_t)];
	} u;
	uint8_t *cur = buf;
	int nchunk;

	while (nbytes > 0) {
		u.value = rte_rand();
		nchunk = min(nbytes, sizeof(u.value));
		memcpy(cur, u.bytes, nchunk);
		cur += nchunk;
		nbytes -= nchunk;
	}
}
