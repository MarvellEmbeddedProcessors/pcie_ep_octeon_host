/*
 *   BSD LICENSE
 *
 *   Copyright(c) 2025  Marvell Octeon EP (EndPoint) Ethernet Driver..
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Marvell, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER(S) OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _OCTEP_MAIN_H_
#define  _OCTEP_MAIN_H_

#include "octep_tx.h"
#include "octep_rx.h"
#include "octep_ctrl_mbox.h"
#include "octep_config.h"

#ifndef PCI_VENDOR_ID_CAVIUM
#define PCI_VENDOR_ID_CAVIUM    0x177D
#endif

#define OCTEP_DRV_NAME      "octeon_ep"
#define OCTEP_DRV_STRING    "Marvell Octeon EndPoint NIC Driver"

#define  OCTEP_PCIID_CN93_PF  0xB200177d
#define  OCTEP_PCIID_CN93_VF  0xB203177d

#define  OCTEP_PCI_DEVICE_ID_CN98_PF 0xB100
#define  OCTEP_PCI_DEVICE_ID_CN98_VF 0xB103

#define  OCTEP_PCI_DEVICE_ID_CN93_PF 0xB200
#define  OCTEP_PCI_DEVICE_ID_CN93_VF 0xB203

#define  OCTEP_PCI_DEVICE_ID_CNF95O_PF 0xB600    //95O PF
#define  OCTEP_PCI_DEVICE_ID_CNF95O_VF 0xB603    //95O VF

#define  OCTEP_PCI_DEVICE_ID_CNF95N_PF 0xB400    //95N PF
#define  OCTEP_PCI_DEVICE_ID_CNF95N_VF 0xB403    //95N VF

#define  OCTEP_PCI_DEVICE_ID_CN10KA_PF  0xB900   //CN10KA PF
#define  OCTEP_PCI_DEVICE_ID_CNF10KA_PF 0xBA00   //CNF10KA PF
#define  OCTEP_PCI_DEVICE_ID_CNF10KB_PF 0xBC00   //CNF10KB PF
#define  OCTEP_PCI_DEVICE_ID_CN10KB_PF  0xBD00   //CN10KB PF

#define  OCTEP_MAX_QUEUES   63
#define  OCTEP_MAX_IQ       OCTEP_MAX_QUEUES
#define  OCTEP_MAX_OQ       OCTEP_MAX_QUEUES
#define  OCTEP_MAX_VF       128
#define OCTEP_MAX_MSIX_VECTORS OCTEP_MAX_OQ
#define  OCTEP_MMIO_REGIONS     3

#define  OCTEP_IQ_INTR_RESEND_BIT  59
#define  OCTEP_OQ_INTR_RESEND_BIT  59

#define  IQ_INSTR_PENDING(iq)  ((iq->host_write_index - iq->flush_index) & iq->ring_size_mask)
#define  IQ_INSTR_SPACE(iq)    (iq->max_count - IQ_INSTR_PENDING(iq))

#define BIT_ULL(nr)     (1ULL << (nr))
#define octep_ms_to_ticks(x)              \
	((hz > 1000) ? ((x) * (hz/1000)) : ((x) / (1000/hz)))

#define octep_mdelay(x) do {              \
	if (cold)                   \
	DELAY(1000 * (x));          \
	else                        \
	pause("Wait", octep_ms_to_ticks(x));  \
} while(0)

#define STATS_REFRESH_INTERVAL_MS 1000

#define OCTEP_DEV_STATE_OPEN 0x01
#define OCTEP_DEV_STATE_READ_STATS 0x02
#define OCTEP_DEV_STATE_DOWN_IN_PROGRESS 0x04

/*
 *  PCI address space information.
 *  Each of the 3 address spaces given by BAR0, BAR2 and BAR4 of
 *  Octeon gets mapped to different physical address spaces in
 *  the kernel.
 */
struct octep_mem_bus_space {
	struct resource		*pci_mem;
	bus_space_tag_t		tag;
	bus_space_handle_t	handle;
};

struct octep_reg_list {
	bus_size_t 	pci_win_wr_addr;
	bus_size_t	pci_win_rd_addr;
	bus_size_t 	pci_win_wr_data;
	bus_size_t	pci_win_rd_data;
};

/* Octeon mailbox data */
struct octep_mbox_data {
	uint32_t cmd;
	uint32_t total_len;
	uint32_t recv_len;
	uint32_t rsvd;
	uint64_t *data;
};

#define  OCTEP_MAX_VF       128
#define MAX_VF_PF_MBOX_DATA_SIZE 384
/* wrappers around work structs */
struct octep_pfvf_mbox_wk {
	struct taskqueue *tq;
	struct task work;
	void *ctxptr;
	uint64_t ctxul;
};

/* Octeon device mailbox */
struct octep_mbox {
	/* A mutex to protect access to this q_mbox. */
	struct mtx lock;
	uint32_t vf_id;
	uint32_t config_data_index;
	uint32_t message_len;
	uint32_t pf_vf_data_reg;
	uint32_t vf_pf_data_reg;
	struct octep_pfvf_mbox_wk wk;
	struct octep_device *oct;
	struct octep_mbox_data mbox_data;
	uint8_t config_data[MAX_VF_PF_MBOX_DATA_SIZE];
};

/* Tx/Rx queue vector per interrupt. */
struct octep_ioq_vector {
	char name[OCTEP_MSIX_NAME_SIZE];
	struct octep_device *octep_dev;
	struct octep_iq *iq;
	struct octep_oq *oq;
	cpuset_t        affinity_mask;
	struct resource *msix_res;
	void *tag;
	int vector;
	int num_msix_irqs;
	struct task     oq_task;
	struct taskqueue    *oq_taskqueue;
};

/* Hardware interface link state information. */
struct octep_iface_link_info {
	/* Bitmap of Supported link speeds/modes. */
	uint64_t supported_modes;

	/* Bitmap of Advertised link speeds/modes. */
	uint64_t advertised_modes;

	/* Negotiated link speed in Mbps. */
	uint32_t speed;

	/* MTU */
	uint16_t mtu;

	/* Autonegotation state. */
#define OCTEP_LINK_MODE_AUTONEG_SUPPORTED   BIT(0)
#define OCTEP_LINK_MODE_AUTONEG_ADVERTISED  BIT(1)
	uint8_t autoneg;

	/* Pause frames setting. */
#define OCTEP_LINK_MODE_PAUSE_SUPPORTED   BIT(0)
#define OCTEP_LINK_MODE_PAUSE_ADVERTISED  BIT(1)
	uint8_t pause;

	/* Admin state of the link (ifconfig <iface> up/down */
	uint8_t  admin_up;

	/* Operational state of the link: physical link is up down */
	uint8_t  oper_up;
};


/* The Octeon VF device specific info data structure.*/
struct octep_pfvf_info {
	uint8_t mac_addr[ETHER_ADDR_LEN];
	uint32_t flags;
	uint32_t mbox_version;
};

struct octep_hw_ops {
	void (*setup_iq_regs)(struct octep_device *oct, int q);
	int (*setup_oq_regs)(struct octep_device *oct, int q);
	void (*setup_mbox_regs)(struct octep_device *oct, int mbox);

	int  (*mbox_intr_handler)(void *ioq_vector);
	void  (*oei_intr_handler)(void *ioq_vector);
	int  (*ire_intr_handler)(void *ioq_vector);
	int  (*ore_intr_handler)(void *ioq_vector);
	int  (*vfire_intr_handler)(void *ioq_vector);
	int  (*vfore_intr_handler)(void *ioq_vector);
	int  (*dma_intr_handler)(void *ioq_vector);
	int  (*dma_vf_intr_handler)(void *ioq_vector);
	int  (*pp_vf_intr_handler)(void *ioq_vector);
	int  (*misc_intr_handler)(void *ioq_vector);
	int  (*rsvd_intr_handler)(void *ioq_vector);
	int (*soft_reset)(struct octep_device *oct);
	void (*reinit_regs)(struct octep_device *oct);
	uint32_t  (*update_iq_read_idx)(struct octep_iq *iq);

	void (*enable_interrupts)(struct octep_device *oct);
	void (*disable_interrupts)(struct octep_device *oct);
	void (*poll_non_ioq_interrupts)(struct octep_device *oct);

	void (*enable_io_queues)(struct octep_device *oct);
	void (*disable_io_queues)(struct octep_device *oct);
	void (*enable_iq)(struct octep_device *oct, int q);
	void (*disable_iq)(struct octep_device *oct, int q);
	void (*enable_oq)(struct octep_device *oct, int q);
	void (*disable_oq)(struct octep_device *oct, int q);
	void (*reset_io_queues)(struct octep_device *oct);
	void (*dump_registers)(struct octep_device *oct);
};

/*
 *  The Octeon device.
 *  Each Octeon device has this structure to represent all its
 *  components.
 */
struct octep_device {
	uint16_t sriov_pos;
	uint16_t num_vf_en;
	bool sriov_enabled;
	struct octep_config *conf;
	struct octep_mbox *mbox[OCTEP_MAX_VF];

	int max_vfs;
	/* Hardware Interface Link info like supported modes, aneg support */
	struct octep_iface_link_info link_info;

	/* offset for iface stats */
	uint32_t ctrl_mbox_ifstats_offset;

	/* control mbox over pf */
	struct octep_ctrl_mbox ctrl_mbox;
	int max_rx_pktlen;
	struct mtx lock;

	/* Mutex and condition variable for host-to-firmware requests */
	struct mtx ctrl_req_mtx;
	struct cv ctrl_req_cv;
	/* List of objects waiting for h2f response */
	TAILQ_HEAD(, octep_ctrl_net_wait_data) ctrl_req_wait_list;

	/* VFs info */
	struct octep_pfvf_info vf_info[OCTEP_MAX_VF];
	/* IOq information of it's corresponding MSI-X interrupt. */
	struct octep_ioq_vector *ioq_vector[OCTEP_MAX_QUEUES];

	struct task tx_timeout_task;       // Task for Tx timeout
	struct task ctrl_mbox_task;        // Task for control mailbox
	struct task dev_setup_task;        // Task for device setup
	struct callout intr_poll_callout;  // Callout for interrupt polling
	struct callout hb_callout;         // Callout for heartbeat
	volatile int status;               // Device status (atomic-like)
	volatile int hb_miss_cnt;          // Heartbeat miss count

	/* The input instruction queues */
	struct octep_iq *iq[OCTEP_MAX_IQ];

	/* The ROQ output queues  */
	struct octep_oq *oq[OCTEP_MAX_OQ];

	/* Enable non-ioq interrupt polling */
	bool poll_non_ioq_intr;

	int num_vfs;
	struct pci_iov_pf *iov;

	/* PCI device pointer */
	device_t	pdev;

	/* Octeon Chip type. */
	uint16_t	chip_id;

	uint16_t	rev_id;


	/* This device's PCIe port used for traffic. */
	uint16_t	pcie_port;

	uint16_t	flags;

	/* memory mapped io range */
	struct octep_mem_bus_space mem_bus_space[OCTEP_MMIO_REGIONS];

	struct octep_reg_list reg_list;

	/* Hardware operations */
	struct octep_hw_ops hw_ops;


	uint32_t	num_iqs;

	uint32_t	num_oqs;

	int		msix_on;

	struct ifnet *netdev;
	struct ifmedia ifmedia;
	uint32_t if_flags;
	int ifstate;
	uint64_t stats_last_update;
	/* Hardware Interface Tx statistics */
	struct octep_iface_tx_stats iface_tx_stats;
	/* Hardware Interface Rx statistics */
	struct octep_iface_rx_stats iface_rx_stats;

	bus_dma_tag_t       dma_tag;

	uint8_t mac_addr[ETHER_ADDR_LEN];

	/* IRQ info */
	u16 num_irqs;

	char *non_ioq_irq_names;
	struct resource **msix_res; /* Array for non-IOQ interrupts */
	void **tag;                /* Array for non-IOQ interrupt tags */
	int *aux_vector;           /* Array for non-IOQ vector IDs */
	int state;
	/* TX/RX process pkt budget */
	uint16_t    rx_budget;
	uint16_t    tx_budget;
};

/* Device status */
enum octep_dev_status {
	OCTEP_DEV_STATUS_INVALID,
	OCTEP_DEV_STATUS_ALLOC,
	OCTEP_DEV_STATUS_WAIT_FOR_FW,
	OCTEP_DEV_STATUS_INIT,
	OCTEP_DEV_STATUS_READY,
	OCTEP_DEV_STATUS_UNINIT
};

static inline uint16_t OCTEP_MAJOR_REV(struct octep_device *oct)
{
	uint16_t rev = (oct->rev_id & 0xC) >> 2;

	return (rev == 0) ? 1 : rev;
}

static inline uint16_t OCTEP_MINOR_REV(struct octep_device *oct)
{
	return (oct->rev_id & 0x3);
}


static inline uint8_t
octep_read_csr8(struct octep_device *oct, bus_size_t reg)
{

	return (bus_space_read_1(oct->mem_bus_space[0].tag,
							 oct->mem_bus_space[0].handle, reg));
}

static inline void
octep_write_csr8(struct octep_device *oct, bus_size_t reg, uint8_t val)
{

	bus_space_write_1(oct->mem_bus_space[0].tag,
					  oct->mem_bus_space[0].handle, reg, val);
}

static inline uint16_t
octep_read_csr16(struct octep_device *oct, bus_size_t reg)
{

	return (bus_space_read_2(oct->mem_bus_space[0].tag,
							 oct->mem_bus_space[0].handle, reg));
}

static inline void
octep_write_csr16(struct octep_device *oct, bus_size_t reg, uint16_t val)
{

	bus_space_write_2(oct->mem_bus_space[0].tag,
					  oct->mem_bus_space[0].handle, reg, val);
}

static inline uint32_t
octep_read_csr(struct octep_device *oct, bus_size_t reg)
{

	return (bus_space_read_4(oct->mem_bus_space[0].tag,
							 oct->mem_bus_space[0].handle, reg));
}

static inline void
octep_write_csr(struct octep_device *oct, bus_size_t reg, uint32_t val)
{

	bus_space_write_4(oct->mem_bus_space[0].tag,
					  oct->mem_bus_space[0].handle, reg, val);
}

static inline uint64_t
octep_read_csr64(struct octep_device *oct, bus_size_t reg)
{

#ifdef __i386__
	return (octep_read_csr(oct, reg) |
			((uint64_t)octep_read_csr(oct, reg + 4) << 32));
#else
	return (bus_space_read_8(oct->mem_bus_space[0].tag,
							 oct->mem_bus_space[0].handle, reg));
#endif
}

static inline void
octep_write_csr64(struct octep_device *oct, bus_size_t reg, uint64_t val)
{

#ifdef __i386__
	octep_write_csr(oct, reg, (uint32_t)val);
	octep_write_csr(oct, reg + 4, val >> 32);
#else
	bus_space_write_8(oct->mem_bus_space[0].tag,
					  oct->mem_bus_space[0].handle, reg, val);
#endif
}

//readl API
static inline uint32_t
octep_read_bar2_csr32(struct octep_device *oct, bus_size_t reg)
{
	return (bus_space_read_4(oct->mem_bus_space[2].tag,
							 oct->mem_bus_space[2].handle, reg));
}

//readq API
static inline uint64_t
octep_read_bar2_csr64(struct octep_device *oct, bus_size_t reg)
{
	return (bus_space_read_8(oct->mem_bus_space[2].tag,
							 oct->mem_bus_space[2].handle, reg));
}

//writel API
static inline void
octep_write_bar2_csr32(struct octep_device *oct, bus_size_t reg, uint32_t val)
{

	bus_space_write_4(oct->mem_bus_space[2].tag,
					  oct->mem_bus_space[2].handle, reg, val);
}

//writeq API
static inline void
octep_write_bar2_csr64(struct octep_device *oct, bus_size_t reg, uint64_t val)
{

#ifdef __i386__
	octep_write_bar2_csr32(oct, reg, (uint32_t)val);
	octep_write_bar2_csr32(oct, reg + 4, val >> 32);
#else
	bus_space_write_8(oct->mem_bus_space[2].tag,
					  oct->mem_bus_space[2].handle, reg, val);
#endif
}

/*
 * \brief unmaps a PCI BAR
 * @param oct Pointer to Octeon device
 * @param baridx bar index
 */
static inline void
octep_unmap_pci_barx(struct octep_device *oct, int baridx)
{

	if (oct->mem_bus_space[baridx].pci_mem != NULL) {
		bus_release_resource(oct->pdev, SYS_RES_MEMORY,
							 PCIR_BAR(baridx * 2),
							 oct->mem_bus_space[baridx].pci_mem);
		oct->mem_bus_space[baridx].pci_mem = NULL;
	}
}

/*
 * \brief maps a PCI BAR
 * @param oct Pointer to Octeon device
 * @param baridx bar index
 */
static inline int
octep_map_pci_barx(struct octep_device *oct, int baridx)
{
	int     rid = PCIR_BAR(baridx * 2);

	oct->mem_bus_space[baridx].pci_mem =
		bus_alloc_resource_any(oct->pdev, SYS_RES_MEMORY, &rid,
							   RF_ACTIVE);

	if (oct->mem_bus_space[baridx].pci_mem == NULL) {
		dev_err(oct->pdev, "Unable to allocate bus resource: memory\n");
		return (ENXIO);
	}

	/* Save bus_space values for READ/WRITE_REG macros */
	oct->mem_bus_space[baridx].tag =
		rman_get_bustag(oct->mem_bus_space[baridx].pci_mem);
	oct->mem_bus_space[baridx].handle =
		rman_get_bushandle(oct->mem_bus_space[baridx].pci_mem);

	dev_dbg(oct->pdev, "BAR%d Tag 0x%llx Handle 0x%llx\n",
			baridx, OCTEP_CAST64(oct->mem_bus_space[baridx].tag),
			OCTEP_CAST64(oct->mem_bus_space[baridx].handle));

	return (0);
}

static inline void
OCTEP_PCI_WIN_WRITE(struct octep_device *oct, u64 addr, u64 val)
{
	octep_write_csr64(oct, oct->reg_list.pci_win_wr_addr, addr);
	octep_write_csr64(oct, oct->reg_list.pci_win_wr_data, val);

	dev_dbg(oct->pdev,
			 "%s: reg: 0x%016lx val: 0x%016lx\n", __func__, addr, val);
}

static inline int
octep_ifstate_check(struct octep_device *oct, int state_flag)
{

	return (atomic_load_acq_int(&oct->state) & state_flag);
}


static inline void
octep_ifstate_set(struct octep_device *oct, int state_flag)
{

	atomic_store_rel_int(&oct->state,
						 (atomic_load_acq_int(&oct->state) | state_flag));
}

static inline void
octep_ifstate_reset(struct octep_device *oct, int state_flag)
{

	atomic_store_rel_int(&oct->state,
						 (atomic_load_acq_int(&oct->state) &
						  ~(state_flag)));
}

extern struct taskqueue *octep_tq;
int octep_device_setup(struct octep_device *oct);
void octep_device_setup_cn93_pf(struct octep_device *oct);
int octep_setup_iqs(struct octep_device *oct);
void octep_free_iqs(struct octep_device *oct);
int octep_setup_oqs(struct octep_device *oct);
void octep_free_oqs(struct octep_device *oct);
void octep_oq_dbell_init(struct octep_device *oct);
void octep_device_setup_cnxk_pf(struct octep_device *oct);
int octep_mq_start_locked(if_t ifp, struct octep_iq *iq);
int octep_iq_process_completions(struct octep_iq *iq, uint16_t budget);
int octep_oq_process_rx(struct octep_oq *oq, int budget);
int octep_oq_check_hw_for_pkts(struct octep_device *oct, struct octep_oq *oq);

#endif	/* _OCTEP_MAIN_H_ */
