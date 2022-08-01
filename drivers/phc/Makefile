#Copyright (c) 2020 Marvell.
#SPDX-License-Identifier: GPL-2.0

MODULE_MAME = oct_ep_phc
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build

SRCS = ep_phc.c octeon_device.c

PCIE_HOST ?= $(OCTEONTX_HOST_ROOT)/drivers/legacy/modules/driver/src/host/linux/kernel/drv
INCLUDE_DIR = -I$(src) -I$(PCIE_HOST) \
	-I$(OCTEONTX_HOST_ROOT)/drivers/legacy/modules/driver/inc \
	-I$(OCTEONTX_HOST_ROOT)/drivers/legacy/modules/driver/src/common \
	-I$(OCTEONTX_HOST_ROOT)/drivers/legacy/modules/driver/src/host/osi \
	-I$(OCTEONTX_HOST_ROOT)/drivers/legacy/modules/driver/src/host/osi/octvf



ccflags-y := $(INCLUDE_DIR)
KBUILD_EXTRA_SYMBOLS := $(PCIE_HOST)/Module.symvers
ccflags-y +=  -Wno-error=date-time
OBJS =  $(SRCS:.c=.o)

obj-m += $(MODULE_MAME).o
$(MODULE_MAME)-y = $(OBJS)


all:
	make KBUILD_VERBOSE=1 -C $(KERNEL_DIR) M=`pwd` modules

clean:
	make -C $(KERNEL_DIR) M=`pwd` clean
	@rm -f ep_phc.o.ur-safe $(MODULE_NAME).mod
