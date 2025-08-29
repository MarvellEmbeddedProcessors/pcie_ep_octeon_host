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

#include "octep_bsd.h"
#include "octep_config.h"
#include "octep_main.h"
#include "octep_regs_cn9k_pf.h"
#include "octep_ctrl_net.h"

/* We will support 128 pf's in control mbox */
#define CTRL_MBOX_MAX_PF	128
#define CTRL_MBOX_SZ		((size_t)(0x400000 / CTRL_MBOX_MAX_PF))

#define PCI_DEVFN(slot, func)	((((slot) & 0x1f) << 3) | ((func) & 0x07))
#define PCI_SLOT(devfn)			(((devfn) >> 3) & 0x1f)
#define PCI_FUNC(devfn)			((devfn) & 0x07)
#define PCI_SRIOV_FUNC_LINK 0x12	/* Function Dependency Link */
#define PCI_EXT_CAP_ID_SRIOV	0x10	/* Single Root I/O Virtualization */

/* Names of Hardware non-queue generic interrupts */
static char *cn93_non_ioq_msix_names[] = {
	"epf_ire_rint",
	"epf_ore_rint",
	"epf_vfire_rint0",
	"epf_vfire_rint1",
	"epf_vfore_rint0",
	"epf_vfore_rint1",
	"epf_mbox_rint0",
	"epf_mbox_rint1",
	"epf_oei_rint",
	"epf_dma_rint",
	"epf_dma_vf_rint0",
	"epf_dma_vf_rint1",
	"epf_pp_vf_rint0",
	"epf_pp_vf_rint1",
	"epf_misc_rint",
	"epf_rsvd",
};

/* Reset Hardware Tx queue */
static int cn93_reset_iq(struct octep_device *oct, int q_no)
{
	struct octep_config *conf = oct->conf;
	u64 val = 0ULL;

	dev_dbg(oct->pdev, "Reset PF IQ-%d\n", q_no);

	/* Get absolute queue number */
	q_no += conf->pf_ring_cfg.srn;

	/* Disable the Tx/Instruction Ring */
	octep_write_csr64(oct, CN93_SDP_R_IN_ENABLE(q_no), val);

	/* clear the Instruction Ring packet/byte counts and doorbell CSRs */
	octep_write_csr64(oct, CN93_SDP_R_IN_CNTS(q_no), val);
	octep_write_csr64(oct, CN93_SDP_R_IN_INT_LEVELS(q_no), val);
	octep_write_csr64(oct, CN93_SDP_R_IN_PKT_CNT(q_no), val);
	octep_write_csr64(oct, CN93_SDP_R_IN_BYTE_CNT(q_no), val);
	octep_write_csr64(oct, CN93_SDP_R_IN_INSTR_BADDR(q_no), val);
	octep_write_csr64(oct, CN93_SDP_R_IN_INSTR_RSIZE(q_no), val);

	val = 0xFFFFFFFF;
	octep_write_csr64(oct, CN93_SDP_R_IN_INSTR_DBELL(q_no), val);

	return 0;
}

/* Reset Hardware Rx queue */
static void cn93_reset_oq(struct octep_device *oct, int q_no)
{
	u64 val = 0ULL;

	q_no += CFG_GET_PORTS_PF_SRN(oct->conf);

	/* Disable Output (Rx) Ring */
	octep_write_csr64(oct, CN93_SDP_R_OUT_ENABLE(q_no), val);

	/* Clear count CSRs */
	val = octep_read_csr(oct, CN93_SDP_R_OUT_CNTS(q_no));
	octep_write_csr(oct, CN93_SDP_R_OUT_CNTS(q_no), val);

	octep_write_csr64(oct, CN93_SDP_R_OUT_PKT_CNT(q_no), 0xFFFFFFFFFULL);
	octep_write_csr64(oct, CN93_SDP_R_OUT_SLIST_DBELL(q_no), 0xFFFFFFFF);
}

/* Reset all hardware Tx/Rx queues */
static void octep_reset_io_queues_cn93_pf(struct octep_device *oct)
{
	int q;

	dev_dbg(oct->pdev, "Reset OCTEP_CN93 PF IO Queues\n");

	for (q = 0; q < CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf); q++) {
		cn93_reset_iq(oct, q);
		cn93_reset_oq(oct, q);
	}
}


/* Initialize windowed addresses to access some hardware registers */
static void octep_setup_pci_window_regs_cn93_pf(struct octep_device *oct)
{
	oct->reg_list.pci_win_wr_addr = CN93_SDP_WIN_WR_ADDR64;
	oct->reg_list.pci_win_rd_addr = CN93_SDP_WIN_RD_ADDR64;
	oct->reg_list.pci_win_wr_data = CN93_SDP_WIN_WR_DATA64;
	oct->reg_list.pci_win_rd_data = CN93_SDP_WIN_RD_DATA64;
}

/* Configure Hardware mapping: inform hardware which rings belong to PF. */
static void octep_configure_ring_mapping_cn93_pf(struct octep_device *oct)
{
	struct octep_config *conf = oct->conf;
	u64 pf_srn = CFG_GET_PORTS_PF_SRN(oct->conf);
	int q;

	for (q = 0; q < CFG_GET_PORTS_ACTIVE_IO_RINGS(conf); q++) {
		u64 regval = 0;

		if (oct->pcie_port)
			regval = 8 << CN93_SDP_FUNC_SEL_EPF_BIT_POS;

		octep_write_csr64(oct, CN93_SDP_EPVF_RING(pf_srn + q), regval);

		regval = octep_read_csr64(oct, CN93_SDP_EPVF_RING(pf_srn + q));
		dev_dbg(oct->pdev, "Write SDP_EPVF_RING[0x%llx] = 0x%lx\n",
				 CN93_SDP_EPVF_RING(pf_srn + q), regval);
	}
}

/* Initialize configuration limits and initial active config 93xx PF. */
static void octep_init_config_cn93_pf(struct octep_device *oct)
{
	struct octep_config *conf = oct->conf;
	u64 val;
	int pos;
	u8 link = 0;

	/* Read ring configuration:
	 * PF ring count, number of VFs and rings per VF supported
	 */
	val = octep_read_csr64(oct, CN93_SDP_EPF_RINFO);
	conf->sriov_cfg.max_rings_per_vf = CN93_SDP_EPF_RINFO_RPVF(val);
	conf->sriov_cfg.active_rings_per_vf = conf->sriov_cfg.max_rings_per_vf;
	conf->sriov_cfg.max_vfs = CN93_SDP_EPF_RINFO_NVFS(val);
	conf->sriov_cfg.active_vfs = 0;
	conf->sriov_cfg.vf_srn = CN93_SDP_EPF_RINFO_SRN(val);

	val = octep_read_csr64(oct, CN93_SDP_MAC_PF_RING_CTL(oct->pcie_port));
	if (oct->chip_id == OCTEP_PCI_DEVICE_ID_CN98_PF) {
		conf->pf_ring_cfg.srn =  CN98_SDP_MAC_PF_RING_CTL_SRN(val);
		conf->pf_ring_cfg.max_io_rings = CN98_SDP_MAC_PF_RING_CTL_RPPF(val);
		conf->pf_ring_cfg.active_io_rings = conf->pf_ring_cfg.max_io_rings;
	} else {
		conf->pf_ring_cfg.srn =  CN93_SDP_MAC_PF_RING_CTL_SRN(val);
		conf->pf_ring_cfg.max_io_rings = CN93_SDP_MAC_PF_RING_CTL_RPPF(val);
		conf->pf_ring_cfg.active_io_rings = conf->pf_ring_cfg.max_io_rings;
	}
	dev_info(oct->pdev, "pf_srn=%u rpvf=%u nvfs=%u rppf=%u\n",
			 conf->pf_ring_cfg.srn, conf->sriov_cfg.active_rings_per_vf,
			 conf->sriov_cfg.active_vfs, conf->pf_ring_cfg.active_io_rings);

	conf->iq.num_descs = OCTEP_IQ_MAX_DESCRIPTORS;
	conf->iq.instr_type = OCTEP_64BYTE_INSTR;
	conf->iq.db_min = OCTEP_DB_MIN;
	conf->iq.intr_threshold = OCTEP_IQ_INTR_THRESHOLD;

	conf->oq.num_descs = OCTEP_OQ_MAX_DESCRIPTORS;
	conf->oq.buf_size = OCTEP_OQ_BUF_SIZE;
	conf->oq.refill_threshold = OCTEP_OQ_REFILL_THRESHOLD;
	conf->oq.oq_intr_pkt = OCTEP_OQ_INTR_PKT_THRESHOLD;
	conf->oq.oq_intr_time = OCTEP_OQ_INTR_TIME_THRESHOLD;

	conf->msix_cfg.non_ioq_msix = CN93_NUM_NON_IOQ_INTR;
	conf->msix_cfg.ioq_msix = conf->pf_ring_cfg.active_io_rings;
	conf->msix_cfg.non_ioq_msix_names = cn93_non_ioq_msix_names;

	if (pci_find_extcap(oct->pdev, PCI_EXT_CAP_ID_SRIOV, &pos) == 0) {
		link = pci_read_config(oct->pdev, pos + PCI_SRIOV_FUNC_LINK, 1);
		link = PCI_DEVFN(PCI_SLOT(pci_get_slot(oct->pdev)), link);
	}
	else {
		dev_info(oct->pdev,"cannot find SR-IOV PCIe cap\n");
		link = 0;
	}

	conf->ctrl_mbox_cfg.barmem_addr = CN93_PEM_BAR4_INDEX_OFFSET +
		(link * CTRL_MBOX_SZ);

	conf->fw_info.hb_interval = OCTEP_DEFAULT_FW_HB_INTERVAL;
	conf->fw_info.hb_miss_count = OCTEP_DEFAULT_FW_HB_MISS_COUNT;
}

/* Setup registers for a hardware Tx Queue	*/
static void octep_setup_iq_regs_cn93_pf(struct octep_device *oct, int iq_no)
{
	struct octep_iq *iq = oct->iq[iq_no];
	u32 reset_instr_cnt;
	u64 reg_val;

	iq_no += CFG_GET_PORTS_PF_SRN(oct->conf);
	reg_val = octep_read_csr64(oct, CN93_SDP_R_IN_CONTROL(iq_no));

	/* wait for IDLE to set to 1 */
	if (!(reg_val & CN93_R_IN_CTL_IDLE)) {
		do {
			reg_val = octep_read_csr64(oct, CN93_SDP_R_IN_CONTROL(iq_no));
		} while (!(reg_val & CN93_R_IN_CTL_IDLE));
	}

	reg_val |= CN93_R_IN_CTL_RDSIZE;
	reg_val |= CN93_R_IN_CTL_IS_64B;
	reg_val |= CN93_R_IN_CTL_ESR;
	octep_write_csr64(oct, CN93_SDP_R_IN_CONTROL(iq_no), reg_val);

	/* Write the start of the input queue's ring and its size  */
	octep_write_csr64(oct, CN93_SDP_R_IN_INSTR_BADDR(iq_no),
					  iq->desc_ring_dma);
	octep_write_csr64(oct, CN93_SDP_R_IN_INSTR_RSIZE(iq_no),
					  iq->max_count);

	/* Remember the doorbell & instruction count register addr
	 * for this queue
	 */
	iq->doorbell_reg = CN93_SDP_R_IN_INSTR_DBELL(iq_no);
	iq->inst_cnt_reg = CN93_SDP_R_IN_CNTS(iq_no);
	iq->intr_lvl_reg = CN93_SDP_R_IN_INT_LEVELS(iq_no);


	/* Store the current instruction counter (used in flush_iq calculation) */
	reset_instr_cnt = octep_read_csr(oct, iq->inst_cnt_reg);
	octep_write_csr(oct, iq->inst_cnt_reg, reset_instr_cnt);

	/* INTR_THRESHOLD is set to max(FFFFFFFF) to disable the INTR */
	reg_val = CFG_GET_IQ_INTR_THRESHOLD(oct->conf) & 0xffffffff;
	octep_write_csr64(oct, CN93_SDP_R_IN_INT_LEVELS(iq_no), reg_val);
}

/* Setup registers for a hardware Rx Queue	*/
static int octep_setup_oq_regs_cn93_pf(struct octep_device *oct, int oq_no)
{
	u64 reg_val;
	u64 oq_ctl = 0ULL;
	u32 time_threshold = 0;
	struct octep_oq *oq = oct->oq[oq_no];

	oq_no += CFG_GET_PORTS_PF_SRN(oct->conf);
	reg_val = octep_read_csr64(oct, CN93_SDP_R_OUT_CONTROL(oq_no));

	/* wait for IDLE to set to 1 */
	if (!(reg_val & CN93_R_OUT_CTL_IDLE)) {
		do {
			reg_val = octep_read_csr64(oct, CN93_SDP_R_OUT_CONTROL(oq_no));
		} while (!(reg_val & CN93_R_OUT_CTL_IDLE));
	}

	reg_val &= ~(CN93_R_OUT_CTL_IMODE);
	reg_val &= ~(CN93_R_OUT_CTL_ROR_P);
	reg_val &= ~(CN93_R_OUT_CTL_NSR_P);
	reg_val &= ~(CN93_R_OUT_CTL_ROR_I);
	reg_val &= ~(CN93_R_OUT_CTL_NSR_I);
	reg_val &= ~(CN93_R_OUT_CTL_ES_I);
	reg_val &= ~(CN93_R_OUT_CTL_ROR_D);
	reg_val &= ~(CN93_R_OUT_CTL_NSR_D);
	reg_val &= ~(CN93_R_OUT_CTL_ES_D);
	reg_val |= (CN93_R_OUT_CTL_ES_P);

	octep_write_csr64(oct, CN93_SDP_R_OUT_CONTROL(oq_no), reg_val);

	octep_write_csr64(oct, CN93_SDP_R_OUT_SLIST_BADDR(oq_no),
					  oq->desc_ring_dma);
	octep_write_csr64(oct, CN93_SDP_R_OUT_SLIST_RSIZE(oq_no),
					  oq->max_count);

	oq_ctl = octep_read_csr64(oct, CN93_SDP_R_OUT_CONTROL(oq_no));
	oq_ctl &= ~0x7fffffULL; //clear the ISIZE and BSIZE (22-0)
	oq_ctl |= (oq->buffer_size & 0xffff);	//populate the BSIZE (15-0)
	octep_write_csr64(oct, CN93_SDP_R_OUT_CONTROL(oq_no), oq_ctl);

	/* Get the mapped address of the pkt_sent and pkts_credit regs */
	oq->pkts_sent_reg = CN93_SDP_R_OUT_CNTS(oq_no);
	oq->pkts_credit_reg = CN93_SDP_R_OUT_SLIST_DBELL(oq_no);

	octep_write_csr64(oct, oq->pkts_credit_reg, oq->max_count);

	time_threshold = CFG_GET_OQ_INTR_TIME(oct->conf);
	reg_val = ((u64)time_threshold << 32) |
		CFG_GET_OQ_INTR_PKT(oct->conf);
	octep_write_csr64(oct, CN93_SDP_R_OUT_INT_LEVELS(oq_no), reg_val);
	return 0;
}

/* Setup registers for a PF mailbox */
static void octep_setup_mbox_regs_cn93_pf(struct octep_device *oct, int q_no)
{
	struct octep_mbox *mbox = oct->mbox[q_no];

	/* PF to VF DATA reg. PF writes into this reg */
	mbox->pf_vf_data_reg = CN93_SDP_MBOX_PF_VF_DATA(q_no);

	/* VF to PF DATA reg. PF reads from this reg */
	mbox->vf_pf_data_reg = CN93_SDP_MBOX_VF_PF_DATA(q_no);

}

/* Poll for mailbox messages from VF */
static void octep_poll_pfvf_mailbox(struct octep_device *oct)
{
	u32 vf, active_vfs, active_rings_per_vf, vf_mbox_queue;
	u64 reg0, reg1;

	reg0 = octep_read_csr64(oct, CN93_SDP_EPF_MBOX_RINT(0));
	reg1 = octep_read_csr64(oct, CN93_SDP_EPF_MBOX_RINT(1));
	if (reg0 || reg1) {
		/*
		 * dev_info(&oct->pdev->dev, "Received MBOX_RINT intr: reg0 0x%llx reg1 0x%llx\n",
		 * reg0, reg1);
		 */

		active_vfs = CFG_GET_ACTIVE_VFS(oct->conf);
		active_rings_per_vf = CFG_GET_ACTIVE_RPVF(oct->conf);
		for (vf = 0; vf < active_vfs; vf++) {
			vf_mbox_queue = vf * active_rings_per_vf;

			if (vf_mbox_queue < 64) {
				if (!(reg0 & (0x1UL << vf_mbox_queue)))
					continue;
			} else {
				if (!(reg1 & (0x1UL << (vf_mbox_queue - 64))))
					continue;
			}

			if (!oct->mbox[vf_mbox_queue]) {
				dev_err(oct->pdev, "bad mbox vf %d\n", vf);
				continue;
			}
			taskqueue_enqueue(oct->mbox[vf_mbox_queue]->wk.tq,
							  &oct->mbox[vf_mbox_queue]->wk.work);
		}
		if (reg0)
			octep_write_csr64(oct, CN93_SDP_EPF_MBOX_RINT(0), reg0);
		if (reg1)
			octep_write_csr64(oct, CN93_SDP_EPF_MBOX_RINT(1), reg1);
	}
}

/* PF-VF mailbox interrupt handler */
static int octep_pfvf_mbox_intr_handler_cn93_pf(void *dev)
{
	struct octep_device *oct = (struct octep_device *)dev;

	octep_poll_pfvf_mailbox(oct);
	return FILTER_HANDLED;
}

/* Poll OEI events like heartbeat */
static void octep_poll_oei_cn93_pf(struct octep_device *oct)
{
	u64 reg;
	int status;

	reg = octep_read_csr64(oct, CN93_SDP_EPF_OEI_RINT);
	if (reg) {
		octep_write_csr64(oct, CN93_SDP_EPF_OEI_RINT, reg);
		if (reg & CN93_SDP_EPF_OEI_RINT_DATA_BIT_MBOX)
		{
			status = atomic_load_acq_int(&oct->status);
			if (status == OCTEP_DEV_STATUS_INIT) {
				/* During setup, process synchronously */
				octep_ctrl_net_recv_fw_messages(oct);
			} else if (status == OCTEP_DEV_STATUS_READY) {
				/* After setup, use taskqueue */
				taskqueue_enqueue(octep_tq, &oct->ctrl_mbox_task);
			}
		}
		else if (reg & CN93_SDP_EPF_OEI_RINT_DATA_BIT_HBEAT)
			atomic_store_int(&oct->hb_miss_cnt, 0);
	}
}

/* OEI interrupt handler */
static void octep_oei_intr_handler_cn93_pf(void *dev)
{
	struct octep_device *oct = (struct octep_device *)dev;

	octep_poll_oei_cn93_pf(oct);
}

/* Process non-ioq interrupts required to keep pf interface running.
 * OEI_RINT is needed for control mailbox
 * MBOX_RINT is needed for pfvf mailbox
 */
static void octep_poll_non_ioq_interrupts_cn93_pf(struct octep_device *oct)
{
	octep_poll_pfvf_mailbox(oct);
	octep_poll_oei_cn93_pf(oct);
}

/* Interrupt handler for input ring error interrupts. */
static int octep_ire_intr_handler_cn93_pf(void *dev)
{
	struct octep_device *oct = (struct octep_device *)dev;
	u64 reg_val = 0;
	int i = 0;

	/* Check for IRERR INTR */
	reg_val = octep_read_csr64(oct, CN93_SDP_EPF_IRERR_RINT);
	if (reg_val) {
		dev_info(oct->pdev,
				 "received IRERR_RINT intr: 0x%lx\n", reg_val);
		octep_write_csr64(oct, CN93_SDP_EPF_IRERR_RINT, reg_val);

		for (i = 0; i < CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf); i++) {
			reg_val = octep_read_csr64(oct,
									   CN93_SDP_R_ERR_TYPE(i));
			if (reg_val) {
				dev_info(oct->pdev,
						 "Received err type on IQ-%d: 0x%lx\n",
						 i, reg_val);
				octep_write_csr64(oct, CN93_SDP_R_ERR_TYPE(i),
								  reg_val);
			}
		}
	}
	return FILTER_HANDLED;
}

/* Interrupt handler for output ring error interrupts. */
static int octep_ore_intr_handler_cn93_pf(void *dev)
{
	struct octep_device *oct = (struct octep_device *)dev;
	u64 reg_val = 0;
	int i = 0;

	/* Check for ORERR INTR */
	reg_val = octep_read_csr64(oct, CN93_SDP_EPF_ORERR_RINT);
	if (reg_val) {
		dev_info(oct->pdev,
				 "Received ORERR_RINT intr: 0x%lx\n", reg_val);
		octep_write_csr64(oct, CN93_SDP_EPF_ORERR_RINT, reg_val);
		for (i = 0; i < CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf); i++) {
			reg_val = octep_read_csr64(oct, CN93_SDP_R_ERR_TYPE(i));
			if (reg_val) {
				dev_info(oct->pdev,
						 "Received err type on OQ-%d: 0x%lx\n",
						 i, reg_val);
				octep_write_csr64(oct, CN93_SDP_R_ERR_TYPE(i),
								  reg_val);
			}
		}
	}
	return FILTER_HANDLED;
}

/* Interrupt handler for vf input ring error interrupts. */
static int octep_vfire_intr_handler_cn93_pf(void *dev)
{
	struct octep_device *oct = (struct octep_device *)dev;
	u64 reg_val = 0;

	/* Check for VFIRE INTR */
	reg_val = octep_read_csr64(oct, CN93_SDP_EPF_VFIRE_RINT(0));
	if (reg_val) {
		dev_info(oct->pdev,
				 "Received VFIRE_RINT intr: 0x%lx\n", reg_val);
		octep_write_csr64(oct, CN93_SDP_EPF_VFIRE_RINT(0), reg_val);
	}
	return FILTER_HANDLED;
}

/* Interrupt handler for vf output ring error interrupts. */
static int octep_vfore_intr_handler_cn93_pf(void *dev)
{
	struct octep_device *oct = (struct octep_device *)dev;
	u64 reg_val = 0;

	/* Check for VFORE INTR */
	reg_val = octep_read_csr64(oct, CN93_SDP_EPF_VFORE_RINT(0));
	if (reg_val) {
		dev_info(oct->pdev,
				 "Received VFORE_RINT intr: 0x%lx\n", reg_val);
		octep_write_csr64(oct, CN93_SDP_EPF_VFORE_RINT(0), reg_val);
	}
	return FILTER_HANDLED;
}


/* Interrupt handler for dpi dma related interrupts. */
static int octep_dma_intr_handler_cn93_pf(void *dev)
{
	struct octep_device *oct = (struct octep_device *)dev;
	u64 reg_val = 0;

	/* Check for DMA INTR */
	reg_val = octep_read_csr64(oct, CN93_SDP_EPF_DMA_RINT);
	if (reg_val) {
		octep_write_csr64(oct, CN93_SDP_EPF_DMA_RINT, reg_val);
	}
	return FILTER_HANDLED;
}

/* Interrupt handler for dpi dma transaction error interrupts for VFs  */
static int octep_dma_vf_intr_handler_cn93_pf(void *dev)
{
	struct octep_device *oct = (struct octep_device *)dev;
	u64 reg_val = 0;

	/* Check for DMA VF INTR */
	reg_val = octep_read_csr64(oct, CN93_SDP_EPF_DMA_VF_RINT(0));
	if (reg_val) {
		dev_info(oct->pdev,
				 "Received DMA_VF_RINT intr: 0x%lx\n", reg_val);
		octep_write_csr64(oct, CN93_SDP_EPF_DMA_VF_RINT(0), reg_val);
	}
	return FILTER_HANDLED;
}


/* Interrupt handler for pp transaction error interrupts for VFs  */
static int octep_pp_vf_intr_handler_cn93_pf(void *dev)
{
	struct octep_device *oct = (struct octep_device *)dev;
	u64 reg_val = 0;

	/* Check for PPVF INTR */
	reg_val = octep_read_csr64(oct, CN93_SDP_EPF_PP_VF_RINT(0));
	if (reg_val) {
		dev_info(oct->pdev,
				 "Received PP_VF_RINT intr: 0x%lx\n", reg_val);
		octep_write_csr64(oct, CN93_SDP_EPF_PP_VF_RINT(0), reg_val);
	}
	return FILTER_HANDLED;
}

/* Interrupt handler for mac related interrupts. */
static int octep_misc_intr_handler_cn93_pf(void *dev)
{
	struct octep_device *oct = (struct octep_device *)dev;
	u64 reg_val = 0;

	/* Check for MISC INTR */
	reg_val = octep_read_csr64(oct, CN93_SDP_EPF_MISC_RINT);
	if (reg_val) {
		dev_info(oct->pdev,
				 "Received MISC_RINT intr: 0x%lx\n", reg_val);
		octep_write_csr64(oct, CN93_SDP_EPF_MISC_RINT, reg_val);
	}
	return FILTER_HANDLED;
}

/* Interrupts handler for all reserved interrupts. */
static int octep_rsvd_intr_handler_cn93_pf(void *dev)
{
	struct octep_device *oct = (struct octep_device *)dev;

	dev_info(oct->pdev, "Reserved interrupts raised; Ignore\n");
	return FILTER_HANDLED;
}

/* soft reset of 98xx */
static int octep_soft_reset_cn98_pf(struct octep_device *oct)
{
	dev_info(oct->pdev, "CN98XX: skip soft reset\n");
	return 0;
}

/* soft reset of 93xx */
static int octep_soft_reset_cn93_pf(struct octep_device *oct)
{
	dev_info(oct->pdev, "CN93XX: Doing soft reset\n");

	octep_write_csr64(oct, CN93_SDP_WIN_WR_MASK_REG, 0xFF);

	/* Firmware status CSR is supposed to be cleared by
	 * core domain reset, but due to a hw bug, it is not.
	 * Set it to RUNNING right before reset so that it is not
	 * left in READY (1) state after a reset.  This is required
	 * in addition to the early setting to handle the case where
	 * the OcteonTX is unexpectedly reset, reboots, and then
	 * the module is removed.
	 */
	OCTEP_PCI_WIN_WRITE(oct, CN9K_PEMX_PFX_CSX_PFCFGX(0, 0, CN9K_PCIEEP_VSECST_CTL),
						FW_STATUS_DOWNING);
	/* Set core domain reset bit */
	OCTEP_PCI_WIN_WRITE(oct, CN93_RST_CORE_DOMAIN_W1S, 1);
	/* Wait for 100ms as Octeon resets. */
	DELAY(100000);
	/* clear core domain reset bit */
	OCTEP_PCI_WIN_WRITE(oct, CN93_RST_CORE_DOMAIN_W1C, 1);

	return 0;
}

/* Re-initialize Octeon hardware registers */
static void octep_reinit_regs_cn93_pf(struct octep_device *oct)
{
	u32 i;

	for (i = 0; i < CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf); i++)
		oct->hw_ops.setup_iq_regs(oct, i);

	for (i = 0; i < CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf); i++)
		oct->hw_ops.setup_oq_regs(oct, i);

	oct->hw_ops.enable_interrupts(oct);
	oct->hw_ops.enable_io_queues(oct);

	for (i = 0; i < CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf); i++)
		octep_write_csr(oct, oct->oq[i]->pkts_credit_reg,
						oct->oq[i]->max_count);
}

/* Enable all interrupts */
static void octep_enable_interrupts_cn93_pf(struct octep_device *oct)
{
	u64 intr_mask = 0ULL;
	int srn, num_rings, i;

	srn = CFG_GET_PORTS_PF_SRN(oct->conf);
	num_rings = CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf);

	for (i = 0; i < num_rings; i++)
		intr_mask |= (0x1ULL << (srn + i));

	octep_write_csr64(oct, CN93_SDP_EPF_IRERR_RINT_ENA_W1S, intr_mask);
	octep_write_csr64(oct, CN93_SDP_EPF_ORERR_RINT_ENA_W1S, intr_mask);
	octep_write_csr64(oct, CN93_SDP_EPF_OEI_RINT_ENA_W1S, -1ULL);

	octep_write_csr64(oct, CN93_SDP_EPF_VFIRE_RINT_ENA_W1S(0), -1ULL);
	octep_write_csr64(oct, CN93_SDP_EPF_VFORE_RINT_ENA_W1S(0), -1ULL);

	octep_write_csr64(oct, CN93_SDP_EPF_MISC_RINT_ENA_W1S, intr_mask);
	octep_write_csr64(oct, CN93_SDP_EPF_DMA_RINT_ENA_W1S, intr_mask);
	octep_write_csr64(oct, CN93_SDP_EPF_MBOX_RINT_ENA_W1S(0), -1ULL);
	octep_write_csr64(oct, CN93_SDP_EPF_MBOX_RINT_ENA_W1S(1), -1ULL);

	octep_write_csr64(oct, CN93_SDP_EPF_DMA_VF_RINT_ENA_W1S(0), -1ULL);
	octep_write_csr64(oct, CN93_SDP_EPF_PP_VF_RINT_ENA_W1S(0), -1ULL);
}

/* Disable all interrupts */
static void octep_disable_interrupts_cn93_pf(struct octep_device *oct)
{
	u64 reg_val, intr_mask = 0ULL;
	int srn, num_rings, i;

	srn = CFG_GET_PORTS_PF_SRN(oct->conf);
	num_rings = CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf);

	for (i = 0; i < num_rings; i++) {
		intr_mask |= (0x1ULL << (srn + i));
		reg_val = octep_read_csr64(oct, CN93_SDP_R_IN_INT_LEVELS(srn + i));
		reg_val &= ~(0x1ULL << 62);
		octep_write_csr64(oct, CN93_SDP_R_IN_INT_LEVELS(srn + i), reg_val);

		reg_val = octep_read_csr64(oct, CN93_SDP_R_OUT_INT_LEVELS(srn + i));
		reg_val &= ~(0x1ULL << 62);
		octep_write_csr64(oct, CN93_SDP_R_OUT_INT_LEVELS(srn + i), reg_val);
	}

	octep_write_csr64(oct, CN93_SDP_EPF_IRERR_RINT_ENA_W1C, intr_mask);
	octep_write_csr64(oct, CN93_SDP_EPF_ORERR_RINT_ENA_W1C, intr_mask);
	octep_write_csr64(oct, CN93_SDP_EPF_OEI_RINT_ENA_W1C, -1ULL);

	octep_write_csr64(oct, CN93_SDP_EPF_VFIRE_RINT_ENA_W1C(0), -1ULL);
	octep_write_csr64(oct, CN93_SDP_EPF_VFORE_RINT_ENA_W1C(0), -1ULL);

	octep_write_csr64(oct, CN93_SDP_EPF_MISC_RINT_ENA_W1C, intr_mask);
	octep_write_csr64(oct, CN93_SDP_EPF_DMA_RINT_ENA_W1C, intr_mask);
	octep_write_csr64(oct, CN93_SDP_EPF_MBOX_RINT_ENA_W1C(0), -1ULL);
	octep_write_csr64(oct, CN93_SDP_EPF_MBOX_RINT_ENA_W1C(1), -1ULL);

	octep_write_csr64(oct, CN93_SDP_EPF_DMA_VF_RINT_ENA_W1C(0), -1ULL);
	octep_write_csr64(oct, CN93_SDP_EPF_PP_VF_RINT_ENA_W1C(0), -1ULL);
}

/* Get new Octeon Read Index: index of descriptor that Octeon reads next. */
static u32 octep_update_iq_read_index_cn93_pf(struct octep_iq *iq)
{
	struct octep_device *oct = iq->oct_dev;
	u32 pkt_in_done =  octep_read_csr(oct, iq->inst_cnt_reg);
	u32 last_done, new_idx;

	last_done = pkt_in_done - iq->pkt_in_done;
	iq->pkt_in_done = pkt_in_done;

	new_idx = (iq->octep_read_index + last_done) % iq->max_count;

	return new_idx;
}

/* Enable a hardware Tx Queue */
static void octep_enable_iq_cn93_pf(struct octep_device *oct, int iq_no)
{
	u64 loop = hz;
	u64 reg_val;

	iq_no += CFG_GET_PORTS_PF_SRN(oct->conf);

	octep_write_csr64(oct, CN93_SDP_R_IN_INSTR_DBELL(iq_no), 0xFFFFFFFF);

	while (octep_read_csr64(oct, CN93_SDP_R_IN_INSTR_DBELL(iq_no)) &&
		   loop--) {
		pause("pause", 1);
	}

	reg_val = octep_read_csr64(oct,  CN93_SDP_R_IN_INT_LEVELS(iq_no));
	reg_val |= (0x1ULL << 62);
	octep_write_csr64(oct, CN93_SDP_R_IN_INT_LEVELS(iq_no), reg_val);

	reg_val = octep_read_csr64(oct, CN93_SDP_R_IN_ENABLE(iq_no));
	reg_val |= 0x1ULL;
	octep_write_csr64(oct, CN93_SDP_R_IN_ENABLE(iq_no), reg_val);
}

/* Enable a hardware Rx Queue */
static void octep_enable_oq_cn93_pf(struct octep_device *oct, int oq_no)
{
	u64 reg_val = 0ULL;

	oq_no += CFG_GET_PORTS_PF_SRN(oct->conf);

	reg_val = octep_read_csr64(oct,  CN93_SDP_R_OUT_INT_LEVELS(oq_no));
	reg_val |= (0x1ULL << 62);
	octep_write_csr64(oct, CN93_SDP_R_OUT_INT_LEVELS(oq_no), reg_val);

	octep_write_csr64(oct, CN93_SDP_R_OUT_SLIST_DBELL(oq_no), 0xFFFFFFFF);

	reg_val = octep_read_csr64(oct, CN93_SDP_R_OUT_ENABLE(oq_no));
	reg_val |= 0x1ULL;
	octep_write_csr64(oct, CN93_SDP_R_OUT_ENABLE(oq_no), reg_val);
}

/* Enable all hardware Tx/Rx Queues assined to PF */
static void octep_enable_io_queues_cn93_pf(struct octep_device *oct)
{
	u8 q;

	for (q = 0; q < CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf); q++) {
		octep_enable_iq_cn93_pf(oct, q);
		octep_enable_oq_cn93_pf(oct, q);
	}
}

/* Disable a hardware Tx Queue assined to PF */
static void octep_disable_iq_cn93_pf(struct octep_device *oct, int iq_no)
{
	u64 reg_val = 0ULL;

	iq_no += CFG_GET_PORTS_PF_SRN(oct->conf);

	reg_val = octep_read_csr64(oct, CN93_SDP_R_IN_ENABLE(iq_no));
	reg_val &= ~0x1ULL;
	octep_write_csr64(oct, CN93_SDP_R_IN_ENABLE(iq_no), reg_val);
}

/* Disable a hardware Rx Queue assined to PF */
static void octep_disable_oq_cn93_pf(struct octep_device *oct, int oq_no)
{
	u64 reg_val = 0ULL;

	oq_no += CFG_GET_PORTS_PF_SRN(oct->conf);
	reg_val = octep_read_csr64(oct, CN93_SDP_R_OUT_ENABLE(oq_no));
	reg_val &= ~0x1ULL;
	octep_write_csr64(oct, CN93_SDP_R_OUT_ENABLE(oq_no), reg_val);
}

/* Disable all hardware Tx/Rx Queues assined to PF */
static void octep_disable_io_queues_cn93_pf(struct octep_device *oct)
{
	int q = 0;

	for (q = 0; q < CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf); q++) {
		octep_disable_iq_cn93_pf(oct, q);
		octep_disable_oq_cn93_pf(oct, q);
	}
}

/**
 * octep_device_setup_cn93_pf() - Setup Octeon device.
 *
 * @oct: Octeon device private data structure.
 *
 * - initialize hardware operations.
 * - get target side pcie port number for the device.
 * - setup window access to hardware registers.
 * - set initial configuration and max limits.
 * - setup hardware mapping of rings to the PF device.
 */
void octep_device_setup_cn93_pf(struct octep_device *oct)
{
	oct->hw_ops.setup_iq_regs = octep_setup_iq_regs_cn93_pf;
	oct->hw_ops.setup_oq_regs = octep_setup_oq_regs_cn93_pf;
	oct->hw_ops.setup_mbox_regs = octep_setup_mbox_regs_cn93_pf;

	oct->hw_ops.mbox_intr_handler = octep_pfvf_mbox_intr_handler_cn93_pf;
	oct->hw_ops.oei_intr_handler = octep_oei_intr_handler_cn93_pf;
	oct->hw_ops.ire_intr_handler = octep_ire_intr_handler_cn93_pf;
	oct->hw_ops.ore_intr_handler = octep_ore_intr_handler_cn93_pf;
	oct->hw_ops.vfire_intr_handler = octep_vfire_intr_handler_cn93_pf;
	oct->hw_ops.vfore_intr_handler = octep_vfore_intr_handler_cn93_pf;
	oct->hw_ops.dma_intr_handler = octep_dma_intr_handler_cn93_pf;
	oct->hw_ops.dma_vf_intr_handler = octep_dma_vf_intr_handler_cn93_pf;
	oct->hw_ops.pp_vf_intr_handler = octep_pp_vf_intr_handler_cn93_pf;
	oct->hw_ops.misc_intr_handler = octep_misc_intr_handler_cn93_pf;
	oct->hw_ops.rsvd_intr_handler = octep_rsvd_intr_handler_cn93_pf;
	if (oct->chip_id == OCTEP_PCI_DEVICE_ID_CN98_PF)
		oct->hw_ops.soft_reset = octep_soft_reset_cn98_pf;
	else
		oct->hw_ops.soft_reset = octep_soft_reset_cn93_pf;
	oct->hw_ops.reinit_regs = octep_reinit_regs_cn93_pf;

	oct->hw_ops.enable_interrupts = octep_enable_interrupts_cn93_pf;
	oct->hw_ops.disable_interrupts = octep_disable_interrupts_cn93_pf;
	oct->hw_ops.poll_non_ioq_interrupts = octep_poll_non_ioq_interrupts_cn93_pf;

	oct->hw_ops.update_iq_read_idx = octep_update_iq_read_index_cn93_pf;

	oct->hw_ops.enable_iq = octep_enable_iq_cn93_pf;
	oct->hw_ops.enable_oq = octep_enable_oq_cn93_pf;
	oct->hw_ops.enable_io_queues = octep_enable_io_queues_cn93_pf;

	oct->hw_ops.disable_iq = octep_disable_iq_cn93_pf;
	oct->hw_ops.disable_oq = octep_disable_oq_cn93_pf;
	oct->hw_ops.disable_io_queues = octep_disable_io_queues_cn93_pf;
	oct->hw_ops.reset_io_queues = octep_reset_io_queues_cn93_pf;

	//oct->hw_ops.dump_registers = octep_dump_registers_cn93_pf;

	octep_setup_pci_window_regs_cn93_pf(oct);

	oct->pcie_port = octep_read_csr64(oct, CN93_SDP_MAC_NUMBER) & 0xff;
	dev_info(oct->pdev,
			 "Octeon device using PCIE Port %d\n", oct->pcie_port);

	octep_init_config_cn93_pf(oct);
	octep_configure_ring_mapping_cn93_pf(oct);

	if (oct->chip_id == OCTEP_PCI_DEVICE_ID_CN98_PF)
		return;

	/* Firmware status CSR is supposed to be cleared by
	 * core domain reset, but due to IPBUPEM-38842, it is not.
	 * Set it to RUNNING early in boot, so that unexpected resets
	 * leave it in a state that is not READY (1).
	 */
	OCTEP_PCI_WIN_WRITE(oct, CN9K_PEMX_PFX_CSX_PFCFGX(0, 0, CN9K_PCIEEP_VSECST_CTL),
						FW_STATUS_RUNNING);
}

