/*
 * Copyright (c) 2014-2015,2017 Qualcomm Atheros, Inc.
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
#include "wil6210_ethdev.h"
#include "fw.h"

#define SYSFS_PATH_PARAM "/sys/module/firmware_class/parameters/path"

struct device;

/* Do not bother with _GNU_SOURCE */
int asprintf(char **strp, const char *fmt, ...);

struct firmware {
	size_t size;
	u8 *data;
	void *priv;
};

void read_custom_fw_path(char *path, size_t buf_size)
{
	FILE *fp;
	int i, ret;

	fp = fopen(SYSFS_PATH_PARAM, "r");
	if (fp != NULL) {
		ret = fread(path, sizeof(char), buf_size - 1, fp);
		if (ferror(fp) != 0) {
			path[0] = '\0';
		} else {
			// trim trailing whitespace
			path[ret] = '\0';
			for (i = strlen(path) - 1; i >= 0; i--) {
				if (!isspace(path[i]))
					break;
			}
			path[i + 1] = '\0';
		}
		fclose(fp);
	}
}

static int request_firmware(const struct firmware **fw,
		     const char *name,
		     struct device *device)
{
	FILE *fp;
	char *path;
	struct firmware *fwp;
	int error, i;
	char fw_path_param[WIL_FW_FILE_PATH_BUFSZ];
	char *fw_path[] = { fw_path_param, DEFAULT_FW_PATH };

	fp = NULL;
	path = NULL;
	fwp = NULL;

	fwp = malloc(sizeof *fwp);
	if (fwp == NULL) {
		return -ENOMEM;
	}
	memset(fwp, 0, sizeof(*fwp));

	/* read custom path from sysfs */
	read_custom_fw_path(fw_path_param, WIL_FW_FILE_PATH_BUFSZ);

	for (i = 0; i < ARRAY_SIZE(fw_path); i++) {
		/* skip unspecified custom path */
		if (!fw_path[i][0]) {
			error = -ENOENT;
			goto next;
		}

		error = 0;
		if (asprintf(&path, "%s/%s", fw_path[i], name) == -1) {
			error = -ENOMEM;
			goto next;
		}

		fp = fopen(path, "rb");
		if (fp == NULL) {
			RTE_LOG(ERR, PMD, "Unable to open file '%s'\n", path);
			error = -errno;
			goto next;
		}

		if (fseek(fp, 0, SEEK_END) != 0) {
			RTE_LOG(ERR, PMD, "Unable to seek file '%s'\n", path);
			error = -errno;
			goto next;
		}

		fwp->size = ftell(fp);
		if (fseek(fp, 0, SEEK_SET) != 0) {
			RTE_LOG(ERR, PMD, "Unable to seek file '%s'\n", path);
			error = -errno;
			goto next;
		}

		fwp->data = malloc(fwp->size);
		if (fwp->data == NULL) {
			RTE_LOG(ERR, PMD, "Unable to allocate buffer for file '%s'\n", path);
			error = -ENOMEM;
			goto next;
		}

		if (fread(fwp->data, fwp->size, 1, fp) != 1) {
			RTE_LOG(ERR, PMD, "Unable to read file '%s'\n", path);
			error = -ENOENT;
			goto next;
		}

		RTE_LOG(ERR, PMD, "Successfully loaded file '%s'\n", path);
next:
		if (path != NULL)
			free(path);
		if (fp != NULL)
			fclose(fp);
		if (error != 0) {
			if (fwp->data != NULL)
				free(fwp->data );
		} else {
			*fw = fwp;
			break;
		}
	}

	if (error != 0)
		free(fwp);
	return error;
}

static void release_firmware(const struct firmware *_fw)
{
	/* deconstify */
	struct firmware *fw = (void *)(uintptr_t)_fw;
	if (fw == NULL)
		return;
	if (fw->data != NULL)
		free(fw->data);
	free(fw);
}

static
void wil_memset_toio_32(volatile void __iomem *dst, u32 val,
			size_t count)
{
	volatile u32 __iomem *d = dst;

	for (count += 4; count > 4; count -= 4)
		__raw_writel(val, d++);
}

#include "fw_inc.c"
