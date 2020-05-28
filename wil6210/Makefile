#
# Makefile for the wil6210 kernel module
#

M := $(CURDIR)
KSRC = $(KERNEL_SRC)

KBUILD_FLAGS ?= -C $(KSRC) M=$(M) \
	CONFIG_WIL6210=m CONFIG_WIL6210_ISR_COR=y CONFIG_WIL6210_DEBUGFS=y \
	CONFIG_WIL6210_WRITE_IOCTL=y
KBUILD_FLAGS += V=1

build:
	$(MAKE) $(KBUILD_FLAGS)

modules_install:
	$(MAKE) $(KBUILD_FLAGS) modules_install

clean:
	$(MAKE) $(KBUILD_FLAGS) clean