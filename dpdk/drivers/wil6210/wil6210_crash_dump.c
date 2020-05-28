/*
 * Copyright (c) 2015,2017 Qualcomm Atheros, Inc.
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
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

#include "wil6210_ethdev.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>

#define WIL_FW_CORE_DUMP_DEFAULT_PATH "/var/volatile/cores/wil6210_fw_core"
#define WIL_FW_NOT_READY_PATH "/var/volatile/cores/wil6210_fw_not_ready"

static int wil_fw_get_crash_dump_bounds(struct wil6210_priv *wil,
					u32 *out_dump_size, u32 *out_host_min)
{
	int i;
	const struct fw_map *map;
	u32 host_min, host_max, tmp_max;

	if (!out_dump_size)
		return -EINVAL;

	/* calculate the total size of the unpacked crash dump */
	BUILD_BUG_ON(ARRAY_SIZE(fw_mapping) == 0);
	map = &fw_mapping[0];
	host_min = map->host;
	host_max = map->host + (map->to - map->from);

	for (i = 1; i < ARRAY_SIZE(fw_mapping); i++) {
		map = &fw_mapping[i];

		if (!map->crash_dump)
			continue;

		if (map->host < host_min)
			host_min = map->host;

		tmp_max = map->host + (map->to - map->from);
		if (tmp_max > host_max)
			host_max = tmp_max;
	}

	*out_dump_size = host_max - host_min;
	if (out_host_min)
		*out_host_min = host_min;

	return 0;
}

int wil_fw_copy_crash_dump(struct wil6210_priv *wil, size_t log_size,
	size_t log_offset)
{
	int i, rc, fd;
	const struct fw_map *map;
	void *data, *dest;
	u32 host_min, dump_size, offset, len;
	char filepath[WIL_FW_FILE_PATH_BUFSZ];
	struct tm *t;
	time_t cur_time;
	char timestamp[21];
	struct rte_pci_addr p = wil->pdev->addr;
	bool only_logs;

	/* when log_size is specified, copy only logs section, otherwise copy
	 * entire crash dump */
	only_logs = log_size > 0;

	cur_time = time(NULL);
	t = gmtime(&cur_time);
	strftime(timestamp, sizeof(timestamp), "%FT%TZ", t);

	if (only_logs) {
		rc = snprintf(filepath, WIL_FW_FILE_PATH_BUFSZ,
			      "%s_%04x:%02x:%02x.%d_%s", WIL_FW_NOT_READY_PATH,
			      p.domain, p.bus, p.devid, p.function, timestamp);
	} else if (strlen(wil->fw_core_dump_path) > 0) {
		rc = snprintf(filepath, WIL_FW_FILE_PATH_BUFSZ,
			      "%s_%04x:%02x:%02x.%d_%s", wil->fw_core_dump_path,
			      p.domain, p.bus, p.devid, p.function, timestamp);
	} else {
		rc = snprintf(filepath, WIL_FW_FILE_PATH_BUFSZ,
			      "%s_%04x:%02x:%02x.%d_%s",
			      WIL_FW_CORE_DUMP_DEFAULT_PATH, p.domain, p.bus,
			      p.devid, p.function, timestamp);
	}

	if (rc >= WIL_FW_FILE_PATH_BUFSZ) {
		wil_err(wil, "truncated crash dump filename\n");
	}

	if (only_logs) {
		dump_size = log_size;
	} else if (wil_fw_get_crash_dump_bounds(wil, &dump_size, &host_min)) {
		wil_err(wil, "fail to obtain crash dump size\n");
		return -EINVAL;
	}

	fd = open(filepath, O_RDWR | O_CREAT | O_TRUNC, (mode_t)0600);
	if (fd == -1) {
		wil_err(wil, "error creating core dump file %s\n", filepath);
		return fd;
	}

	if (ftruncate(fd, dump_size)) {
		close(fd);
		wil_err(wil, "error setting core dump file length %d\n", dump_size);
		return -EINVAL;
	}

	dest = mmap(NULL, dump_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (dest == MAP_FAILED) {
		close(fd);
		wil_err(wil, "error mmapping core dump file\n");
		return -EINVAL;
	}
	close(fd);

	rc = wil_mem_access_lock(wil);
	if (rc) {
		munmap(dest, dump_size);
		return rc;
	}

	if (only_logs) {
		data = (void * __force)wil->csr + log_offset;
		wil_memcpy_fromio_32(dest, (const void __iomem * __force)data,
			dump_size);
	} else {
		/* copy to crash dump file */
		for (i = 0; i < ARRAY_SIZE(fw_mapping); i++) {
			map = &fw_mapping[i];

			if (!map->crash_dump)
				continue;

			data = (void * __force)wil->csr + HOSTADDR(map->host);
			len = map->to - map->from;
			offset = map->host - host_min;

			wil_dbg_misc(wil,
					"fw_copy_crash_dump: - dump %s, size %d, offset %d\n",
					fw_mapping[i].name, len, offset);

			wil_memcpy_fromio_32((void * __force)(dest + offset),
						(const void __iomem * __force)data, len);
		}
	}
	wil_mem_access_unlock(wil);

	if (msync(dest, dump_size, MS_SYNC)) {
		wil_err(wil, "error flushing mmapped core dump to file\n");
		munmap(dest, dump_size);
		return -EINVAL;
	}

	if (munmap(dest, dump_size)) {
		wil_err(wil, "error unmapping core dump file\n");
		return -EINVAL;
	}

	wil_info(wil, "Firmware core dumped, size %d bytes, location %s\n",
		dump_size, filepath);

	rc = wil_fw_print_dump_logs(wil, filepath, only_logs);
	if (rc) {
		wil_err(wil, "Unable to print firmware trace to file\n");
		return rc;
	}

	return 0;
}

void wil_fw_core_dump(struct wil6210_priv *wil)
{
#ifndef WIL6210_PMD
	void *fw_dump_data;
	u32 fw_dump_size;

	if (wil_fw_get_crash_dump_bounds(wil, &fw_dump_size, NULL)) {
		wil_err(wil, "fail to get fw dump size\n");
		return;
	}

	fw_dump_data = vzalloc(fw_dump_size);
	if (!fw_dump_data)
		return;

	if (wil_fw_copy_crash_dump(wil, fw_dump_data, fw_dump_size)) {
		vfree(fw_dump_data);
		return;
	}
	/* fw_dump_data will be free in device coredump release function
	 * after 5 min
	 */
	dev_coredumpv(wil_to_dev(wil), fw_dump_data, fw_dump_size, GFP_KERNEL);
	wil_info(wil, "fw core dumped, size %d bytes\n", fw_dump_size);
#endif
}
