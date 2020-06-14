#   BSD LICENSE
#
#   Copyright(c) 2010-2015 Intel Corporation. All rights reserved.
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     * Neither the name of Intel Corporation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

include $(RTE_SDK)/mk/rte.vars.mk

#
# library name
#
LIB = librte_pmd_wil6210.a

CFLAGS += -I$(SRCDIR)/wil6210
CFLAGS += -I$(SRCDIR)/dpdk-dhd-ctrl
CFLAGS += -I$(SRCDIR)/../../modules/dpdk-dhd
CFLAGS += -I$(SRCDIR)
#CFLAGS += -O3
CFLAGS += $(WERROR_FLAGS)

ifeq ($(CONFIG_RTE_TOOLCHAIN_ICC),y)
#
# CFLAGS for icc
#
CFLAGS_BASE_DRIVER = -wd174 -wd593 -wd869 -wd981 -wd2259

else ifeq ($(CONFIG_RTE_TOOLCHAIN_CLANG),y)
#
# CFLAGS for clang
#
CFLAGS_BASE_DRIVER = -Wno-unused-parameter -Wno-unused-value
CFLAGS_BASE_DRIVER += -Wno-strict-aliasing -Wno-format-extra-args

else
#
# CFLAGS for gcc
#
ifeq ($(shell test $(GCC_VERSION) -ge 44 && echo 1), 1)
CFLAGS+= -Wno-deprecated
endif
CFLAGS+= -Wno-unused-parameter -Wno-unused-value
CFLAGS+= -Wno-strict-aliasing -Wno-format-extra-args
CFLAGS+= -Wno-pointer-arith -Wno-sign-compare

# Use clear-on-read for ISR registers
CFLAGS+= -DCONFIG_WIL6210_ISR_COR

# Define macto used to stub kernel-only code out
CFLAGS+= -DWIL6210_PMD

endif
LDLIBS += -lrte_eal -lrte_mbuf -lrte_mempool -lrte_ring
LDLIBS += -lrte_ethdev -lrte_net -lrte_kvargs
LDLIBS += -lrte_bus_pci

VPATH += $(SRCDIR)/dpdk-dhd-ctrl

EXPORT_MAP := rte_pmd_wil6210_version.map

LIBABIVER := 1

# Driver is built as external library
ifeq ($(CONFIG_RTE_LIBRTE_WIL6210_PMD),)
CONFIG_RTE_LIBRTE_WIL6210_PMD := y
endif

#
# all source are stored in SRCS-y
#
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_api.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_compat.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_control.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_crash_dump.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_debug.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_ethdev.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_fw.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_fw_trace.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_interrupt.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_ioctl.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_main.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_netdev.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_nl60g.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_pcidev.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_platform.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_platform_dpdk.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_rx_reorder.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_slave.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_txrx.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_txrx_edma.c
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += wil6210_wmi.c

#
# Control path sources
#
SRCS-$(CONFIG_RTE_LIBRTE_WIL6210_PMD) += dpdk-dhd-ctrl.c

#
# External API
#
SYMLINK-$(CONFIG_RTE_LIBRTE_WIL6210_PMD)-include += rte_wigig_api.h

include $(RTE_SDK)/mk/rte.extlib.mk
