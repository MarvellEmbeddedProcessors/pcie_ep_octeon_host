/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2024 Marvell.
 */

#include <linux/vfio.h>
#include <pci/header.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <pci/pci.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <signal.h>

#include "mrvl_cuse.h"

#define SYS_CLASS_IOMMU_PATH        "/sys/class/iommu"
#define SYS_CLASS_VFIO_PCI_PATH     "/sys/module/vfio_pci"
#define SYS_BUS_PCI_PATH            "/sys/bus/pci/devices"
#define SYS_VFIO_PCI_PATH           "/sys/bus/pci/drivers/vfio-pci"
#define SYS_CLASS_UIO_PCI_PATH      "/sys/module/uio_pci_generic"
#define SYS_UIO_PCI_PATH            "/sys/bus/pci/drivers/uio_pci_generic"

#define PCIE_CMD_MAX_LENGTH	    128
#define MRVL_PCI_PATH_MAX           120
#define PCIE_NAME_LEN			32
#define PCIE_VFIO 1

extern mrvl_pcie_dev_t *oct_pcie_dev_array[MAX_MRVL_OCT_DEV];
extern int current_oct_pcie_dev_cnt;
volatile uint64_t iova_addr;
struct pci_access *pacc;
int bind;

uint64_t dma_alloc(mrvl_pcie_dev_t *dev, uint64_t *vaddr, uint64_t *dma_addr, int size)
{
	int ret, pagesz;
	struct vfio_group_status group_status = { .argsz = sizeof(group_status) };
	struct vfio_iommu_type1_dma_map dma_map = { .argsz = sizeof(dma_map) };
	struct vfio_iommu_type1_info iommu_info = { .argsz = sizeof(iommu_info) };

	pagesz = sysconf(_SC_PAGESIZE);
	if (size < pagesz)
		size = pagesz;
	/* Allocate some space and setup a DMA mapping */
	dma_map.vaddr = (uint64_t) mmap(0, size, PROT_READ | PROT_WRITE,
					MAP_PRIVATE | MAP_ANONYMOUS, 0, 0);

	if (dma_map.vaddr == MAP_FAILED) {
		printf("\n map failed \n");
		return 0;
	}

	dma_map.size = size;
	pthread_mutex_lock(&dev->dma_lock);
	dma_map.iova = iova_addr;
	iova_addr += size;
	pthread_mutex_unlock(&dev->dma_lock);
	dma_map.flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE;
	ret = ioctl(dev->container_fd, VFIO_IOMMU_MAP_DMA, &dma_map);
	if (ret) {
		printf("\n iommu error line %d ret = %d\n", __LINE__, ret);
		return 0;
	}

	*dma_addr = dma_map.iova;

	return dma_map.vaddr;
}

int dma_unmap(mrvl_pcie_dev_t *dev, uint64_t *vaddr, uint64_t *dma_addr, int size)
{
	int ret, pagesz;
	struct vfio_iommu_type1_dma_unmap dma_unmap = { .argsz = sizeof(dma_unmap) };

	pagesz = sysconf(_SC_PAGESIZE);
	if (size < pagesz)
		size = pagesz;
	dma_unmap.size = size;
	dma_unmap.iova = dma_addr;
	ret = ioctl(dev->container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap);
	munmap((void *)(vaddr), size);
}

int pcie_bind(mrvl_pcie_dev_t *dev, bool enable)
{
	char cmd[PCIE_CMD_MAX_LENGTH];
	int ret = -1;

	if (dev->mmap_mode == PCIE_VFIO) {

		snprintf(cmd, sizeof(cmd), "echo '%x %x' > %s/%s 2>/dev/null",
			MRVL_VENDOR_ID, MRVL_DEVICE_ID, dev->pci_path,
			enable ? "new_id" : "remove_id");
		ret = system(cmd);
	}

	return ret;
}

/* Memory map over VFIO. */
static int pcie_mmap_vfio(mrvl_pcie_dev_t *dev)
{
	int ret, group_id, container_fd = -1, group_fd = -1, device_fd = -1;
	struct vfio_irq_info irq = { .argsz = sizeof(irq) };
	char path[MRVL_PCI_PATH_MAX], name[64], *p;

	struct vfio_iommu_type1_info iommu_info = { .argsz = sizeof(iommu_info) };

	struct vfio_group_status group_status = {
	  .argsz = sizeof(group_status)
	};

	struct vfio_device_info device_info = {
	  .argsz = sizeof(device_info)
	};

	struct vfio_region_info region_info = {
	  .argsz = sizeof(region_info)
	};

	struct vfio_iommu_type1_dma_map dma_map = { .argsz = sizeof(dma_map) };

	container_fd = open("/dev/vfio/vfio", O_RDWR);
	if (container_fd < 0) {
		printf("Failed to open /dev/vfio/vfio, %d (%s)\n",
		       container_fd, strerror(errno));
		ret = container_fd;
		goto fail;
	}

	if (ioctl(container_fd, VFIO_GET_API_VERSION) != VFIO_API_VERSION) {
		printf("Unknown vfio API version\n");
		ret = -EPROTONOSUPPORT;
		goto fail;
	}

	if (!ioctl(container_fd, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU)) {
		printf("IOMMU version not supported\n");
		ret = -EPROTONOSUPPORT;
		goto fail;
	}

	/* Find the group_id. */
	snprintf(path, sizeof(path), "%s/%04x:%02x:%02x.%1u/iommu_group",
		SYS_BUS_PCI_PATH, dev->domain, dev->bus,
		dev->dev, dev->func);
	ret = readlink(path, name, sizeof(name));
	if (ret < 0 || !name[0] || ret >= sizeof(name)) {
		printf("%s: failed to read iommu link\n", path);
		goto fail;
	}
	printf("\n iommu  group = %s", name);
	name[ret] = 0;
	p = strrchr(name, '/');
	if (!p) {
		printf("Failed to find vfio group\n");
		ret = -ENOENT;
		goto fail;
	}
	group_id = atoi(p + 1);
	printf("\n group id = %d", group_id);
	snprintf(path, sizeof(path), "/dev/vfio/%d", group_id);
	group_fd = open(path, O_RDWR);
	if (group_fd < 0) {
		printf("Failed to open %s, %d (%s)\n",
			path, group_fd, strerror(errno));
		ret = group_fd;
		goto fail;
	}

	ret = ioctl(group_fd, VFIO_GROUP_GET_STATUS, &group_status);
	if (ret) {
		printf("VFIO_GROUP_GET_STATUS failed\n");
		goto fail;
	}

	if (!(group_status.flags & VFIO_GROUP_FLAGS_VIABLE)) {
		printf("VFIO group not viable\n");
		ret = -1;
		goto fail;
	}
	/* Add the group to the container */

	ret = ioctl(group_fd, VFIO_GROUP_SET_CONTAINER, &container_fd);

	if (ret) {
		printf("\n adding to conatiner failed line %d ret %d", ret, __LINE__);
		goto fail;
	}

	ret = ioctl(container_fd, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU);
	if (ret) {
		printf("\n VFIO_SET_IOMMU failed ret  %d\n", ret);
		goto fail;
	}

	/* Get addition IOMMU info */
	ret = ioctl(container_fd, VFIO_IOMMU_GET_INFO, &iommu_info);
	if (ret) {
		printf("\n iommu error line %d ret = %d\n", __LINE__, ret);
		goto fail;
	}

	snprintf(path, sizeof(path), "%04x:%02x:%02x.%d", dev->domain,
					dev->bus, dev->dev, dev->func);

	device_fd = ioctl(group_fd, VFIO_GROUP_GET_DEVICE_FD, path);
	if (device_fd < 0) {
		printf("Failed to get vfio device %s\n", path);
		ret = device_fd;
		goto fail;
	}

	ret = ioctl(device_fd, VFIO_DEVICE_GET_INFO, &device_info);
	if (ret) {
		printf("Failed to get vfio device info\n");
		goto fail;
	}

	if (!device_info.num_regions) {
		printf("Failed to get device region count\n");
		ret = -1;
		goto fail;
	}

	//printf("\n number of regions found = %d",device_info.num_regions);

	dev->device_fd = device_fd;
	dev->group_fd = group_fd;
	dev->container_fd = container_fd;

	region_info.index = VFIO_PCI_BAR4_REGION_INDEX;
	ret = ioctl(device_fd, VFIO_DEVICE_GET_REGION_INFO, &region_info);
	if (ret) {
		printf("Failed to get vfio region info\n");
		goto fail;
	}

	if (region_info.flags & VFIO_REGION_INFO_FLAG_MMAP) {
		void *map = mmap(NULL, (size_t)region_info.size,
				PROT_READ | PROT_WRITE,
				MAP_SHARED | MAP_LOCKED,
				device_fd,
				(off_t)region_info.offset);

		//printf("\n returned map = %p", map);
		if (map == MAP_FAILED) {
			printf("Vfio mmap failed\n");
			ret = -1;
			goto fail;
		} else {
			dev->bar4_addr = map;
			printf("\n dev %p bar4 addr = %p",dev, map);
		}
	}

        uint16_t cmd;
        cmd = pci_read_word(dev->pci_dev, PCI_COMMAND);
        cmd |= PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
        pci_write_word(dev->pci_dev, PCI_COMMAND, cmd);

	return 0;
fail:
	if (device_fd >= 0)
		close(device_fd);
	if (group_fd >= 0)
		close(group_fd);
	if (container_fd >= 0)
		close(container_fd);
	// We may to do unmapping here
	return ret;
}

/* Release pcie resource. */
void pcie_mmap_release(void)
{
	for (int i = 0; i < current_oct_pcie_dev_cnt; i++) {
		mrvl_pcie_dev_t *dev = oct_pcie_dev_array[i];
		void *addr = dev->bar4_addr;

		if (dev->bar4_addr) {
			dev->bar4_addr = NULL;
			__sync_synchronize();
			munmap((void *)addr, dev->bar4_size);
		}

		if (dev->device_fd >= 0) {
			close(dev->device_fd);
			dev->device_fd = -1;
		}

		if (dev->group_fd >= 0) {
			close(dev->group_fd);
			dev->group_fd = -1;
		}

		if (dev->container_fd >= 0) {
			close(dev->container_fd);
			dev->container_fd = -1;
		}

		free(dev);
	}
}

static int pcie_mmap(mrvl_pcie_dev_t *dev, bool enable)
{
	int ret;

	ret = pcie_mmap_vfio(dev);
	printf("\n vendor %4x device %4x\n", pci_read_word(dev->pci_dev, PCI_VENDOR_ID));
	return ret;
}

static int pcie_enable(mrvl_pcie_dev_t *dev, bool enable)
{
	int ret = 0;

	printf("\n dev %p\n", dev);
	if (!dev->device_id) {
		printf("\n func %s line %d failed\n", __func__,__LINE__);
		return -ENODEV;
	}

	/* Bind/unbind the device. */
	if (!bind) {
		ret = pcie_bind(dev, enable);
		bind = 1;
	}

	if (ret) {
		printf("\nFailed to bind the device \n");
		return ret;
	}
	ret = pcie_mmap(dev, true);

	return ret;
}

static int mrvl_pcie_dev_probe(struct pci_dev *pci_dev, mrvl_pcie_dev_t **octdev, struct device *d_lcl, struct pci_access *pacc)
{
	char dev_name[PCIE_NAME_LEN];
	mrvl_pcie_dev_t *dev;
	mrvl_oct_dev_t *oct;
	int ret = 0;

	snprintf(dev_name, sizeof(dev_name) - 1, "pcie-%04x:%02x:%02x.%x",
		pci_dev->domain, pci_dev->bus, pci_dev->dev, pci_dev->func);

	octdev[current_oct_pcie_dev_cnt]  = calloc(1, sizeof(mrvl_pcie_dev_t));
	dev = octdev[current_oct_pcie_dev_cnt];
	if (dev == NULL) {
		printf("\n Dev is NULL\n");
		goto out;
	}
	current_oct_pcie_dev_cnt++;
	dev->d_lcl = d_lcl;
	dev->pacc = pacc;
	dev->pacc_reset = 0;
	oct = &dev->oct;
	strcpy(oct->dev_name, dev_name);
	oct->enable_device = pcie_enable;
	dev->device_fd = -1;
	dev->group_fd = -1;
	dev->container_fd = -1;
	dev->pci_path = SYS_VFIO_PCI_PATH;
	dev->mmap_mode = PCIE_VFIO;

	oct->rev_id = pci_read_byte(pci_dev, PCI_REVISION_ID);

	dev->device_id = pci_dev->device_id;
	dev->domain = pci_dev->domain;
	dev->bus = pci_dev->bus;
	dev->dev = pci_dev->dev;
	dev->func = pci_dev->func;
	dev->pci_dev = pci_dev;
	printf("\n dev %2x:%2x:%2x:%2x", pci_dev->domain,
			pci_dev->bus, pci_dev->dev, pci_dev->func);
	pthread_mutex_init(&dev->dma_lock, NULL);

	ret = pcie_enable(dev, true);
	if (ret)
		goto out;

	return 0;

out:
	free(octdev[current_oct_pcie_dev_cnt]);
	return ret;
}

static bool is_vfio_available(void)
{
	struct dirent *d;
	DIR *dir;
	int ret;

	ret = system("modprobe vfio_pci");
	if (ret == -1)
		printf("Failed to load the vfio_pci module\n");
	dir = opendir(SYS_CLASS_VFIO_PCI_PATH);
	if (!dir) {
		printf("\n error : func %s line %d\n", __func__, __LINE__);
		return false;
	}
	closedir(dir);

	dir = opendir(SYS_CLASS_IOMMU_PATH);
	if (!dir) {
		printf("\n error : func %s line %d\n", __func__, __LINE__);
		return false;
	}
	while ((d = readdir(dir)) != NULL) {
	  if (strcmp(d->d_name, ".") && strcmp(d->d_name, ".."))
		break;
	}
	closedir(dir);

	return (d != NULL);
}

int mrvl_pcie_dev_init(mrvl_pcie_dev_t **oct_dev)
{
	bool mrvl_dev_avlbl = false;
	struct pci_dev *dev;
	struct device *d;
	int ret;

	if (!is_vfio_available()) {
		printf("\n VFIO not supported\n");
		return -ENOENT;
	}

	pacc = pci_alloc();
	if (!pacc) {
		printf("\n %s failed\n", __func__);
		return -ENOMEM;
	}

	pci_init(pacc);
	pci_scan_bus(pacc);

	/* Iterate over the devices */
	for (dev = pacc->devices; dev; dev = dev->next) {
		pci_fill_info(dev, PCI_FILL_IDENT | PCI_FILL_BASES | PCI_FILL_CLASS);

		if (dev->vendor_id != MRVL_VENDOR_ID || dev->device_id != MRVL_DEVICE_ID)
			continue;

		ret = mrvl_pcie_dev_probe(dev, oct_dev, NULL, pacc);

		if (ret)
			continue;

		mrvl_dev_avlbl = true;
	}

	if (!mrvl_dev_avlbl)
		return -ENODEV;

	return 0;
}
