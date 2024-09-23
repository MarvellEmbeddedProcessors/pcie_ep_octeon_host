/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2024 Marvell.
 */

#ifndef __MMIO_API_H_
#define __MMIO_API_H_

#include "mrvl_cuse.h"

static inline void mmio_memset(void *mmio_addr, int val, int size)
{
	uint8_t *baddr;

	baddr = (uint8_t *)mmio_addr;
	while (size--) {
		writeb(val, baddr);
		baddr++;
	}
}

static inline void mmio_memread(void *laddr, void const *mmio_addr,
				int size)
{
	uint64_t *qaddr;
	uint8_t  *baddr;
	int alignl, alignr;
	uint64_t *lqaddr;
	uint8_t *lbaddr;

	alignl = (uint64_t)laddr % 8;
	alignr = (uint64_t)mmio_addr % 8;
	qaddr = (uint64_t *)mmio_addr;
	lqaddr = (uint64_t *)laddr;
	if (alignl == 0 && alignr == 0) {
		while (size >= 8) {
			*lqaddr = readq(qaddr);
			size -= 8;
			lqaddr++;
			qaddr++;
		}
	}
	baddr = (uint8_t *)qaddr;
	lbaddr = (uint8_t *)lqaddr;
	while (size--) {
		*lbaddr = readb(baddr);
		baddr++;
		lbaddr++;
	}
}

static inline void mmio_memwrite(void *mmio_addr, void const *laddr,
				 int size)
{
	uint64_t *qaddr;
	uint8_t  *baddr;
	int alignl, alignr;
	uint64_t *lqaddr;
	uint8_t *lbaddr;

	alignl = (uint64_t)laddr % 8;
	alignr = (uint64_t)mmio_addr % 8;
	qaddr = (uint64_t *)mmio_addr;
	lqaddr = (uint64_t *)laddr;
	if (alignl == 0 && alignr == 0) {
		while (size >= 8) {
			writeq(*lqaddr, qaddr);
			size -= 8;
			lqaddr++;
			qaddr++;
		}
	}
	baddr = (uint8_t *)qaddr;
	lbaddr = (uint8_t *)lqaddr;
	while (size--) {
		writeb(*lbaddr, baddr);
		baddr++;
		lbaddr++;
	}
}

#endif /* _MMIO_API_ */
