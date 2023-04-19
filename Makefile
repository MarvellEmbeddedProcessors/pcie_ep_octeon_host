# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2020 Marvell.

export CONFIG_OCTEON_EP=m
export CONFIG_OCTEON_EP_VF=m

SUBDIRS=drivers/octeon_ep
SUBDIRS+=drivers/octeon_ep_vf
SUBDIRS+=drivers/phc
SUBDIRS+=drivers/octboot_net

all: $(SUBDIRS)
		for d in $(SUBDIRS); do \
				if test -d $$d; then $(MAKE) -s -C $$d || exit 1; fi; \
		done
clean:
		for d in $(SUBDIRS); do \
				if test -d $$d; then  $(MAKE) -s -C $$d $@ || exit 1; fi; \
		done
