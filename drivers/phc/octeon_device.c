/* Copyright (c) 2020 Marvell.
 * SPDX-License-Identifier: GPL-2.0
 */

#include "barmap.h"
#include "octeon_device.h"
#include "octeon_macros.h"
#include "octeon_mem_ops.h"
#include "oct_config_data.h"
#include "octeon_compat.h"
#include "octeon-pci.h"

/* All Octeon devices share the same PHC device ID */
#define OCTEON_PHC_PCIID_PF		0xef00177d

octeon_device_t *octeon_device[MAX_OCTEON_DEVICES];
uint32_t octeon_device_count = 0;

void octeon_free_device_mem(octeon_device_t *oct_dev)
{
	int i;

	for (i = 0; i < oct_dev->num_oqs; i++) {
		if (oct_dev->droq[i])
			cavium_free_virt(oct_dev->droq[i]);
	}

	for (i = 0; i < oct_dev->num_iqs; i++) {
		if (oct_dev->instr_queue[i])
			cavium_free_virt(oct_dev->instr_queue[i]);
	}

	i = oct_dev->octeon_id;
	cavium_free_virt(oct_dev);

	octeon_device[i] = NULL;
	octeon_device_count--;
}
octeon_device_t* octeon_allocate_device_mem(int pci_id)
{
	octeon_device_t *oct_dev;
	uint8_t *buf = NULL;
	int octdevsize = 0, configsize = 0, size;

	switch (pci_id) {


	case OCTEON_CN93XX_ID_PF:
	case OCTEON_CN3380_ID_PF:
	case OCTEON_CN98XX_ID_PF:
	case OCTEON_CN95O_ID_PF:
	case OCTEON_CN95N_ID_PF:
	    case 0xEF00:
		configsize = sizeof(octeon_cn93xx_pf_t);
		break;

	case OCTEON_CN10KA_ID_PF:
	case OCTEON_CNF10KA_ID_PF:
	case OCTEON_CNF10KB_ID_PF:
	case OCTEON_CN10KB_ID_PF:
		configsize = sizeof(octeon_cnxk_pf_t);
		break;

	default:
		cavium_print_msg("%s: Unknown PCI Device: 0x%x\n", __FUNCTION__,
				 pci_id);
		return(NULL);
	}

	if (configsize & 0x7)
		configsize += (8 - (configsize & 0x7));

	octdevsize = sizeof(octeon_device_t);
	if (octdevsize & 0x7)
		octdevsize += (8 - (octdevsize & 0x7));

	size =
		octdevsize + configsize +
		(sizeof(octeon_dispatch_t) * DISPATCH_LIST_SIZE);
	buf = cavium_alloc_virt(size);
	if (buf == NULL)
		return(NULL);

	cavium_memset(buf, 0, size);

	oct_dev = (octeon_device_t *)buf;
	oct_dev->chip = (void *)(buf + octdevsize);
	oct_dev->dispatch.dlist =
		(octeon_dispatch_t *)(buf + octdevsize + configsize);

	return(oct_dev);
}
DEFINE_SPINLOCK(allocate_lock);
octeon_device_t* octeon_allocate_device(int pci_id)
{
	int oct_idx = 0;
	octeon_device_t *oct_dev = NULL;

	spin_lock(&allocate_lock);

	for (oct_idx = 0; oct_idx < MAX_OCTEON_DEVICES; oct_idx++) {
		if (octeon_device[oct_idx] == NULL)
			break;
	}

	if (oct_idx == MAX_OCTEON_DEVICES) {
		cavium_error
			("OCT_PHC: Could not find empty slot for device pointer. octeon_device_count: %d MAX_OCTEON_DEVICES: %d\n",
			 octeon_device_count, MAX_OCTEON_DEVICES);
		goto out;
	}

	oct_dev = octeon_allocate_device_mem(pci_id);
	if (oct_dev == NULL) {
		cavium_error("OCT_PHC: Allocation failed for octeon device\n");
		goto out;
	}

	octeon_device_count++;
	octeon_device[oct_idx] = oct_dev;

	oct_dev->octeon_id = oct_idx;
	octeon_assign_dev_name(oct_dev);

out:
	spin_unlock(&allocate_lock);
	return(oct_dev);
}


void cavium_delete_proc(octeon_device_t *oct_dev)
{
	struct proc_dir_entry *root =
		(struct proc_dir_entry *)oct_dev->proc_root_dir;
	if (!root)
		return;
	proc_remove(root);

	return;
}

void octeon_unmap_pci_barx(octeon_device_t *oct_dev, int baridx)
{
	cavium_print(PRINT_DEBUG,
		     "OCT_PHC[%d]: Freeing PCI mapped regions for Bar%d\n",
		     oct_dev->octeon_id, baridx);

	if (oct_dev->mmio[baridx].done)
		iounmap((void *)oct_dev->mmio[baridx].hw_addr);

	if (oct_dev->mmio[baridx].start)
		pci_release_region(oct_dev->pci_dev, baridx * 2);
}

void octeon_destroy_resources(octeon_device_t *oct_dev)
{
	/* call the App handler to clear and destroy the queues created by the application. */

	switch (cavium_atomic_read(&oct_dev->status)) {
	case OCT_DEV_RUNNING:
	case OCT_DEV_DROQ_INIT_DONE:
	case OCT_DEV_INSTR_QUEUE_INIT_DONE:

#if __GNUC__ > 6
		__attribute__((__fallthrough__));
#endif

	case OCT_DEV_CORE_OK:
		cavium_atomic_set(&oct_dev->status, OCT_DEV_IN_RESET);

		oct_dev->app_mode = CVM_DRV_INVALID_APP;

		cavium_sleep_timeout(CAVIUM_TICKS_PER_SEC / 10);
#if __GNUC__ > 6
		__attribute__((__fallthrough__));
#endif

	case OCT_DEV_STOPPING:
#ifndef PCIE_AER
	case OCT_DEV_IN_RESET:
#endif
	case OCT_DEV_HOST_OK:


#ifndef PCIE_AER
		/* Delete the /proc device entries */
		cavium_delete_proc(oct_dev);
#endif
#if __GNUC__ > 6
		__attribute__((__fallthrough__));
#endif

#ifdef PCIE_AER
	case OCT_DEV_IN_RESET:
#endif

	case OCT_DEV_RESP_LIST_INIT_DONE:
		/* Disable the device */
		pci_disable_device(oct_dev->pci_dev);

#if __GNUC__ > 6
		__attribute__((__fallthrough__));
#endif

	case OCT_DEV_DISPATCH_INIT_DONE:
#if __GNUC__ > 6
		__attribute__((__fallthrough__));
#endif

	case OCT_DEV_PCI_MAP_DONE:
		octeon_unmap_pci_barx(oct_dev, 0);
		octeon_unmap_pci_barx(oct_dev, 1);
		if (OCTEON_CN9PLUS_PF(oct_dev->chip_id)) {
			octeon_unmap_pci_barx(oct_dev, 2);
		}

		cavium_print_msg("OCT_PHC[%d]: BAR unmapped.\n",
				 oct_dev->octeon_id);
#if __GNUC__ > 6
		__attribute__((__fallthrough__));
#endif

	case OCT_DEV_RESET_CLEANUP_DONE:
	case OCT_DEV_BEGIN_STATE:
		/* Nothing to be done here either */
		break;
	}                       /* end switch(oct_dev->status) */

}

static int cn93xx_get_pcie_qlmport(octeon_device_t *oct_dev)
{
	oct_dev->pcie_port = octeon_read_csr64(oct_dev, CN93XX_SDP_MAC_NUMBER) & 0xff;

	cavium_print_msg("OCT_PHC[%d]: CN9xxx uses PCIE Port %d\n",
			 oct_dev->octeon_id, oct_dev->pcie_port);
	/* If port is 0xff, PCIe read failed, return error */
	return(oct_dev->pcie_port == 0xff);
}

static void cn93xx_setup_reg_address(octeon_device_t *oct_dev)
{
	uint8_t cavium_iomem *bar0_pciaddr = oct_dev->mmio[0].hw_addr;

	oct_dev->reg_list.pci_win_wr_addr_hi =
		(uint32_t cavium_iomem *)(bar0_pciaddr +
					  CN93XX_SDP_WIN_WR_ADDR_HI);
	oct_dev->reg_list.pci_win_wr_addr_lo =
		(uint32_t cavium_iomem *)(bar0_pciaddr +
					  CN93XX_SDP_WIN_WR_ADDR_LO);
	oct_dev->reg_list.pci_win_wr_addr =
		(uint64_t cavium_iomem *)(bar0_pciaddr +
					  CN93XX_SDP_WIN_WR_ADDR64);

	oct_dev->reg_list.pci_win_rd_addr_hi =
		(uint32_t cavium_iomem *)(bar0_pciaddr +
					  CN93XX_SDP_WIN_RD_ADDR_HI);
	oct_dev->reg_list.pci_win_rd_addr_lo =
		(uint32_t cavium_iomem *)(bar0_pciaddr +
					  CN93XX_SDP_WIN_RD_ADDR_LO);
	oct_dev->reg_list.pci_win_rd_addr =
		(uint64_t cavium_iomem *)(bar0_pciaddr +
					  CN93XX_SDP_WIN_RD_ADDR64);

	oct_dev->reg_list.pci_win_wr_data_hi =
		(uint32_t cavium_iomem *)(bar0_pciaddr +
					  CN93XX_SDP_WIN_WR_DATA_HI);
	oct_dev->reg_list.pci_win_wr_data_lo =
		(uint32_t cavium_iomem *)(bar0_pciaddr +
					  CN93XX_SDP_WIN_WR_DATA_LO);
	oct_dev->reg_list.pci_win_wr_data =
		(uint64_t cavium_iomem *)(bar0_pciaddr +
					  CN93XX_SDP_WIN_WR_DATA64);

	oct_dev->reg_list.pci_win_rd_data_hi =
		(uint32_t cavium_iomem *)(bar0_pciaddr +
					  CN93XX_SDP_WIN_RD_DATA_HI);
	oct_dev->reg_list.pci_win_rd_data_lo =
		(uint32_t cavium_iomem *)(bar0_pciaddr +
					  CN93XX_SDP_WIN_RD_DATA_LO);
	oct_dev->reg_list.pci_win_rd_data =
		(uint64_t cavium_iomem *)(bar0_pciaddr +
					  CN93XX_SDP_WIN_RD_DATA64);
}


int octeon_map_pci_barx(octeon_device_t *oct_dev, int baridx, int max_map_len)
{
	unsigned long mapped_len = 0;

	if (pci_request_region(oct_dev->pci_dev, baridx * 2, DRIVER_NAME)) {
		cavium_error
			("OCT_PHC[%d]: pci_request_region failed for bar %d\n",
			 oct_dev->octeon_id, baridx);
		return(1);
	}

	oct_dev->mmio[baridx].start = pci_resource_start(oct_dev->pci_dev, baridx * 2);
	oct_dev->mmio[baridx].len = pci_resource_len(oct_dev->pci_dev, baridx * 2);

	mapped_len = oct_dev->mmio[baridx].len;
	if (!mapped_len)
		return(1);

	if (max_map_len && (mapped_len > max_map_len)) {
		mapped_len = max_map_len;
	}

	oct_dev->mmio[baridx].hw_addr =
		ioremap(oct_dev->mmio[baridx].start, mapped_len);
	oct_dev->mmio[baridx].mapped_len = mapped_len;

	cavium_print(PRINT_DEBUG,
		     "OCT_PHC[%d]: BAR%d start: 0x%lx mapped %lu of %lu bytes\n",
		     oct_dev->octeon_id, baridx, oct_dev->mmio[baridx].start,
		     mapped_len, oct_dev->mmio[baridx].len);

	/* VSR: delete below print; only for dev debug */
	printk("OCT_PHC[%d]: BAR%d start: 0x%lx mapped %lu of %lu bytes; hw_addr=0x%llx\n",
	       oct_dev->octeon_id, baridx, oct_dev->mmio[baridx].start,
	       mapped_len, oct_dev->mmio[baridx].len, (unsigned long long)oct_dev->mmio[baridx].hw_addr);

	if (!oct_dev->mmio[baridx].hw_addr) {
		cavium_error("OCT_PHC[%d]: error ioremap for bar %d\n",
			     oct_dev->octeon_id, baridx);
		return(1);
	}
	oct_dev->mmio[baridx].done = 1;

	return(0);
}
/* Routine to identify the Octeon device and to map the BAR address space */
int octeon_chip_specific_setup(octeon_device_t *oct_dev)
{
	uint32_t dev_id, rev_id;
	int ret;
	octeon_cn93xx_pf_t *cn93xx;

	OCTEON_READ_PCI_CONFIG(oct_dev, 0, &dev_id);
	OCTEON_READ_PCI_CONFIG(oct_dev, 8, &rev_id);
	oct_dev->rev_id = rev_id & 0xff;

	if (dev_id != OCTEON_PHC_PCIID_PF) {
	    cavium_error("OCT_PHC[%d]: Unknown device found (dev_id: %x)\n",
			 oct_dev->octeon_id, dev_id);
	    return(-1);
	}

	/*
	 * Read subsystem device ID to differentiate chips.
	 * The subsystem device IDs are the same as the 'primary'
	 * device IDs, so we can re-use those defines here.
	 * CN96XX is supported here for debug/development purposes only.
	 */
	OCTEON_READ_PCI_CONFIG(oct_dev, 0x2c, &dev_id);
	switch (dev_id) {
	case OCTEON_CN95N_PCIID_PF:
	case OCTEON_CN95O_PCIID_PF:
	case OCTEON_CN93XX_PCIID_PF:
		cavium_print_msg("OCT_PHC[%d]: CN93XX PASS%d.%d\n",
				 oct_dev->octeon_id, OCTEON_MAJOR_REV(oct_dev),
				 OCTEON_MINOR_REV(oct_dev));
		oct_dev->pf_num = oct_dev->octeon_id;
		/* Enable it to stop loading the driver for PF1 */
		oct_dev->sriov_info.num_vfs = 0;
		oct_dev->chip_id = OCTEON_CN93XX_ID_PF;

		cn93xx = (octeon_cn93xx_pf_t *)oct_dev->chip;

		cn93xx->oct = oct_dev;

		if (octeon_map_pci_barx(oct_dev, 0, 0))
			return(-1);

		/* TODO: It is not required */
		if (octeon_map_pci_barx(oct_dev, 1, MAX_BAR1_IOREMAP_SIZE)) {
			cavium_error("%s CN93XX BAR1 map failed\n", __FUNCTION__);
			octeon_unmap_pci_barx(oct_dev, 0);
			return(-1);
		}

		/* BAR index 2 is BAR 4 on Octeon   */
		if (octeon_map_pci_barx(oct_dev, 2, MAX_BAR1_IOREMAP_SIZE)) {
			cavium_error("%s CN93XX BAR2 map failed\n", __FUNCTION__);
			octeon_unmap_pci_barx(oct_dev, 0);
			octeon_unmap_pci_barx(oct_dev, 1);
			return(-1);
		}
		/* Update pcie port number in the device structure */
		if (cn93xx_get_pcie_qlmport(oct_dev)) {
			cavium_error("%s Invalid PCIe port\n", __FUNCTION__);
			ret = -1;
			goto free_barx;
		}

		cn93xx_setup_reg_address(oct_dev);
#if 0
		/* Firmware status CSR is supposed to be cleared by
		 * core domain reset, but due to a hw bug, it is not.
		 * Set it to RUNNING early in boot, so that unexpected resets
		 * leave it in a state that is not READY (1).
		 */
		printk("About to do OCTEON_PCI_WIN_WRITE\n");
		OCTEON_PCI_WIN_WRITE(oct_dev, CN93XX_PEMX_CFG_WR((u64)oct_dev->pcie_port),
				     0x84d0ull | (FW_STATUS_RUNNING << 32));
		printk("Completed OCTEON_PCI_WIN_WRITE\n");
#endif

#if 0
		ret = octeon_get_fw_info(oct_dev);
		if (ret != 0)
		goto free_barx;
#endif
		return(0);

#if 0
	case OCTEON_CN98XX_PCIID_PF:
		cavium_print_msg("OCTEON[%d]: CN98XX PASS%d.%d on %02x:%02x:%x\n",
				 oct_dev->octeon_id, OCTEON_MAJOR_REV(oct_dev),
				 OCTEON_MINOR_REV(oct_dev), oct_dev->pci_dev->bus->number,
				 PCI_SLOT(oct_dev->pci_dev->devfn),
				 PCI_FUNC(oct_dev->pci_dev->devfn));

		oct_dev->pf_num = oct_dev->octeon_id;
		/* Enable it to stop loading the driver for PF1 */
		oct_dev->sriov_info.num_vfs = num_vfs;
		oct_dev->chip_id = OCTEON_CN98XX_ID_PF;
		return setup_cn98xx_octeon_pf_device(oct_dev); //use 93xx PF setup for now

	case OCTEON_CN10KA_PCIID_PF:
	case OCTEON_CNF10KA_PCIID_PF:
	case OCTEON_CN10KB_PCIID_PF:
	case OCTEON_CNF10KB_PCIID_PF:
		cavium_print_msg("OCTEON[%d]: CNXK PASS%d.%d on %02x:%02x:%x\n",
				 oct_dev->octeon_id, OCTEON_MAJOR_REV(oct_dev),
				 OCTEON_MINOR_REV(oct_dev), oct_dev->pci_dev->bus->number,
				 PCI_SLOT(oct_dev->pci_dev->devfn),
				 PCI_FUNC(oct_dev->pci_dev->devfn));

		oct_dev->pf_num = oct_dev->octeon_id;
		oct_dev->sriov_info.num_vfs = num_vfs;
		oct_dev->chip_id = OCTEON_CN10KA_ID_PF;
		return setup_cnxk_octeon_pf_device(oct_dev);
#endif
	default:
		cavium_error("OCT_PHC: Unknown device found (subsystem_id: %x)\n",
			     dev_id);
	}
free_barx:
	octeon_unmap_pci_barx(oct_dev, 0);
	octeon_unmap_pci_barx(oct_dev, 1);
	octeon_unmap_pci_barx(oct_dev, 2);
	return(-1);
}



/*
 * Read 64 bits at a given offset within a bar4 region.
 */
#define OCTEON_BAR_4_MAPPING_SIZE	(4*1024*1024)
uint64_t octeon_pci_bar4_read64(octeon_device_t *oct_dev, int baridx, uint64_t bar_offset)
{
	if (baridx * OCTEON_BAR_4_MAPPING_SIZE + bar_offset + 8
	    > oct_dev->mmio[2].mapped_len) {
	    cavium_error("OCT_PHC[%d]: Invalid BAR4 index %d offset: 0x%llx, mapped len: 0x%x",
			 oct_dev->octeon_id, baridx, (unsigned long long)bar_offset, oct_dev->mmio[2].mapped_len);
	    return 0;
	}
	return(OCTEON_READ64(oct_dev->mmio[2].hw_addr + baridx * OCTEON_BAR_4_MAPPING_SIZE + bar_offset));

}
void octeon_pci_bar4_write64(octeon_device_t *oct_dev, int baridx, uint64_t bar_offset, uint64_t val)
{
	if (baridx * OCTEON_BAR_4_MAPPING_SIZE + bar_offset + 8
	    > oct_dev->mmio[2].mapped_len) {
	    cavium_error("OCT_PHC[%d]: Invalid BAR4 index %d offset: 0x%llx",
			 oct_dev->octeon_id, baridx, (unsigned long long)bar_offset);
	    return;
	}
	OCTEON_WRITE64(oct_dev->mmio[2].hw_addr + baridx * OCTEON_BAR_4_MAPPING_SIZE + bar_offset, val);

}
