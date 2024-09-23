/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2024 Marvell.
 */

#ifndef _MRVL_CUSE_H
#define _MRVL_CUSE_H

#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/virtio_ids.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#define MAX_MRVL_OCT_DEV 8
#define MAXEVENTS      32
#define MRVL_VENDOR_ID                  0x177d
#define MRVL_DEVICE_ID                  0xEF00

uint32_t time_read(void);
void kill_signal_callback_handler(int signum);
void poll_for_events_on_fds(void);

static inline uint64_t readq(const volatile void *addr)
{
	uint64_t value = *(const volatile uint64_t *)addr;
	__sync_synchronize();
	return value;
}

static inline void writeq(uint64_t value, volatile void *addr)
{
	*(volatile uint64_t *)addr = value;
	__sync_synchronize();
}

static inline uint32_t
readl(const volatile void *addr)
{
	uint32_t value = *(const volatile uint32_t *)addr;
	__sync_synchronize();
	return value;
}

static inline void
writel(uint32_t value, volatile void *addr)
{
	*(volatile uint32_t *)addr = value;
	__sync_synchronize();
}

static inline uint32_t
readb(const volatile void *addr)
{
	uint8_t value = *(const volatile uint8_t *)addr;
	__sync_synchronize();
	return value;
}

static inline void
writeb(uint8_t value, volatile void *addr)
{
	*(volatile uint8_t *)addr = value;
	__sync_synchronize();
}

typedef struct mrvl_oct_dev mrvl_oct_dev_t;
struct mrvl_oct_dev {
	/* Device name. */
	char dev_name[64];
	/* device. */
	void *dev;
#ifdef PHC_SUPPORT
	/* FUSE sessions */
	void *fuse_sessions;
	// struct fuse_session *fuse_sessions;
	pthread_t fuse_thread;
#endif
	int rev_id;
	int (*enable_device)(mrvl_oct_dev_t *oct, bool enable);
};

/*** PCI devices and access to their config space ***/
struct device {
	struct pci_dev *dev;
	uint8_t *config;                         /* Cached configuration space data */
};

struct device *scan_device(struct pci_dev *pdev, struct device *dev);

typedef struct mrvl_pcie_dev mrvl_pcie_dev_t;
struct mrvl_pcie_dev {
	mrvl_oct_dev_t oct;
	struct pci_access *pacc;
	int pacc_reset;
	struct device *d_lcl;
	/* Device info */
	int domain;
	uint16_t device_id;
	uint8_t bus;
	uint8_t dev;
	uint8_t func;
	int rev_id;

	int device_fd;

	/* VFIO info */
	int container_fd;
	int group_fd;

	int mmap_mode;
	const char *pci_path;
	/* BAR4 */
	char *bar4_addr;
	uint32_t bar4_size;
	char *bar2_addr;
	uint32_t bar2_size;
	char *bar0_addr;
	uint32_t bar0_size;
	struct pci_dev *pci_dev;
	int octdev_index;
	pthread_mutex_t dma_lock;
};

struct sk_buff {
	mrvl_pcie_dev_t *dev;
	uint64_t data;
	uint64_t dma;
	uint16_t len;
};

void *octboot_net_init_work(mrvl_pcie_dev_t **mrvl_pcidev);
int octboot_net_init(mrvl_pcie_dev_t *pci_dev);
int mrvl_pcie_dev_init(mrvl_pcie_dev_t **oct_dev);
#ifdef PHC_SUPPORT
int mrvl_fuse_init(mrvl_pcie_dev_t *mdev, int index);
#endif
int dma_unmap(mrvl_pcie_dev_t *dev, uint64_t *vaddr, uint64_t *dma_addr, int size);
uint64_t dma_alloc(mrvl_pcie_dev_t *dev, uint64_t *vaddr, uint64_t *dma_addr, int size);

#ifndef offsetof
#define offsetof(type, member)  ((size_t)&((type *)0)->member)
#endif

//#ifndef container_of
#define container_of(ptr, type, member) ({ \
	void *ptr1 = (void *)(ptr); \
	((type *)(ptr1 - offsetof(type, member))); })
#define DEBUG
#endif
