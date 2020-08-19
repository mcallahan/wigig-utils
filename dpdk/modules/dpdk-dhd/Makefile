# SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause
# Copyright (c) 2020, Facebook, Inc. All rights reserved.

obj-m += dpdk-dhd.o

dpdk-dhd-y := dpdk_dhd.o

ccflags-y := \
	-Wall \
	-Werror \
	--param max-inline-insns-single=50 \
	-I$(PWD)/../../nl-driver-if

all:
	make -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean

modules_install:
	make -C $(KERNEL_SRC) M=$(PWD) modules_install
