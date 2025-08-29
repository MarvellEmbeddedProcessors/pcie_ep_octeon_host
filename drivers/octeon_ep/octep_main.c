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
#include "octep_main.h"
#include "octep_config.h"
#include "octep_ctrl_net.h"
#include "octep_pfvf_mbox.h"
#include "octep_ioctl.h"

#define OCTEP_IQ_COMPLETION_BATCH (iq->max_count / 4)
#define OCTEP_INTR_POLL_TIME_MSECS 100
#define FW_STATUS_VSEC_ID  0xA3
#define FW_STATUS_READY 1ULL
#define FW_POLL_TIMEOUT_MS 30000
#ifndef PCIM_EXTCAP_VENDORSPECIFIC
#define PCIM_EXTCAP_VENDORSPECIFIC 0x0B  /* Vendor-Specific Extended Capability */
#endif

struct taskqueue *octep_tq;

struct octep_vendor_info {
	uint16_t vendor_id;
	uint16_t device_id;
};

static struct octep_vendor_info octep_pci_id_tbl[] = {
	{PCI_VENDOR_ID_CAVIUM, OCTEP_PCI_DEVICE_ID_CN98_PF},
	{PCI_VENDOR_ID_CAVIUM, OCTEP_PCI_DEVICE_ID_CN93_PF},
	{PCI_VENDOR_ID_CAVIUM, OCTEP_PCI_DEVICE_ID_CNF95O_PF},
	{PCI_VENDOR_ID_CAVIUM, OCTEP_PCI_DEVICE_ID_CNF95N_PF},
	{PCI_VENDOR_ID_CAVIUM, OCTEP_PCI_DEVICE_ID_CN10KA_PF},
	{PCI_VENDOR_ID_CAVIUM, OCTEP_PCI_DEVICE_ID_CNF10KA_PF},
	{PCI_VENDOR_ID_CAVIUM, OCTEP_PCI_DEVICE_ID_CNF10KB_PF},
	{PCI_VENDOR_ID_CAVIUM, OCTEP_PCI_DEVICE_ID_CN10KB_PF},
	{0, 0}
};


/**
 * octep_mbox_intr_handler() - common handler for pfvf mbox interrupts.
 *
 * @irq: Interrupt number.
 * @data: interrupt data.
 *
 * this is common handler for pfvf mbox interrupts.
 */
static void octep_mbox_intr_handler(void *data)
{
	struct octep_device *oct = data;
	oct->hw_ops.mbox_intr_handler(oct);

	return;
}

/**
 * octep_oei_intr_handler() - common handler for output endpoint interrupts.
 *
 * @irq: Interrupt number.
 * @data: interrupt data.
 *
 * this is common handler for all output endpoint interrupts.
 */
static void octep_oei_intr_handler(void *data)
{
	struct octep_device *oct = data;
	oct->hw_ops.oei_intr_handler(oct);

	return;
}

/**
 * octep_ire_intr_handler() - common handler for input ring error interrupts.
 *
 * @irq: Interrupt number.
 * @data: interrupt data.
 *
 * this is common handler for input ring error interrupts.
 */
static void octep_ire_intr_handler(void *data)
{
	struct octep_device *oct = data;
	oct->hw_ops.ire_intr_handler(oct);

	return;
}

/**
 * octep_ore_intr_handler() - common handler for output ring error interrupts.
 *
 * @irq: Interrupt number.
 * @data: interrupt data.
 *
 * this is common handler for output ring error interrupts.
 */
static void octep_ore_intr_handler(void *data)
{
	struct octep_device *oct = data;
	oct->hw_ops.ore_intr_handler(oct);

	return;
}


/**
 * octep_vfire_intr_handler() - common handler for vf input ring error interrupts.
 *
 * @irq: Interrupt number.
 * @data: interrupt data.
 *
 * this is common handler for vf input ring error interrupts.
 */
static void octep_vfire_intr_handler(void *data)
{
	struct octep_device *oct = data;
	oct->hw_ops.vfire_intr_handler(oct);

	return;
}

/**
 * octep_vfore_intr_handler() - common handler for vf output ring error interrupts.
 *
 * @irq: Interrupt number.
 * @data: interrupt data.
 *
 * this is common handler for vf output ring error interrupts.
 */
static void octep_vfore_intr_handler(void *data)
{
	struct octep_device *oct = data;
	oct->hw_ops.vfore_intr_handler(oct);

	return;
}

/**
 * octep_dma_intr_handler() - common handler for dpi dma related interrupts.
 *
 * @irq: Interrupt number.
 * @data: interrupt data.
 *
 * this is common handler for dpi dma related interrupts.
 */
static void octep_dma_intr_handler(void *data)
{
	struct octep_device *oct = data;
	oct->hw_ops.dma_intr_handler(oct);

	return;
}

/**
 * octep_dma_vf_intr_handler() - common handler for dpi dma transaction error interrupts for VFs.
 *
 * @irq: Interrupt number.
 * @data: interrupt data.
 *
 * this is common handler for dpi dma transaction error interrupts for VFs.
 */
static void octep_dma_vf_intr_handler(void *data)
{
	struct octep_device *oct = data;
	oct->hw_ops.dma_vf_intr_handler(oct);

	return;
}

/**
 * octep_pp_vf_intr_handler() - common handler for pp transaction error interrupts for VFs.
 *
 * @irq: Interrupt number.
 * @data: interrupt data.
 *
 * this is common handler for pp transaction error interrupts for VFs.
 */
static void octep_pp_vf_intr_handler(void *data)
{
	struct octep_device *oct = data;
	oct->hw_ops.pp_vf_intr_handler(oct);

	return;
}

/**
 * octep_misc_intr_handler() - common handler for mac related interrupts.
 *
 * @irq: Interrupt number.
 * @data: interrupt data.
 *
 * this is common handler for mac related interrupts.
 */
static void octep_misc_intr_handler(void *data)
{
	struct octep_device *oct = data;
	oct->hw_ops.misc_intr_handler(oct);

	return;
}

/**
 * octep_rsvd_intr_handler() - common handler for reserved interrupts (future use).
 *
 * @irq: Interrupt number.
 * @data: interrupt data.
 *
 * this is common handler for all reserved interrupts.
 */
static void octep_rsvd_intr_handler(void *data)
{
	struct octep_device *oct = data;
	oct->hw_ops.rsvd_intr_handler(oct);

	return;
}

static void
octep_update_pkt(struct octep_iq *iq, struct octep_oq *oq)
{
	struct octep_device *oct = iq->oct_dev;
	uint32_t pkts_pend = atomic_load_acq_int(&oq->pkts_pending);
	uint32_t last_pkt_count = atomic_load_acq_int(&oq->last_pkt_count);
	uint32_t pkts_processed = atomic_load_acq_int(&iq->pkts_processed);
	uint32_t pkt_in_done = atomic_load_acq_int(&iq->pkt_in_done);

	if (pkts_processed) {
		octep_write_csr(oct, iq->inst_cnt_reg, pkts_processed);
		octep_read_csr(oct, iq->inst_cnt_reg);
		atomic_store_rel_int(&iq->pkt_in_done, pkt_in_done - pkts_processed);
		atomic_store_rel_int(&iq->pkts_processed, 0);
	}

	if (last_pkt_count > pkts_pend) {
		octep_write_csr(oct, oq->pkts_sent_reg, last_pkt_count - pkts_pend);
		octep_read_csr(oct, oq->pkts_sent_reg);
		atomic_store_rel_int(&oq->last_pkt_count, pkts_pend);
	}
	wmb();
}

/**
 * octep_enable_ioq_irq() - Enable MSI-x interrupt of a Tx/Rx queue.
 *
 * @iq: Octeon Tx queue data structure.
 * @oq: Octeon Rx queue data structure.
 */
static void octep_enable_ioq_irq(struct octep_iq *iq, struct octep_oq *oq)
{
	struct octep_device  *oct = iq->oct_dev;

	octep_write_csr64(oct, oq->pkts_sent_reg, 1UL << OCTEP_OQ_INTR_RESEND_BIT);
	octep_write_csr64(oct, iq->inst_cnt_reg, 1UL << OCTEP_IQ_INTR_RESEND_BIT);
}

static int
octep_flush_iq(struct octep_device *oct, struct octep_iq *iq, uint16_t budget)
{
	uint32_t inst_processed = 0, tot_inst_processed = 0;
	int tx_done = 1;

	if (!mtx_trylock(&iq->iq_flush_running_lock)) {
		return tx_done;
	}

	mtx_lock(&iq->lock);
	iq->octep_read_index = oct->hw_ops.update_iq_read_idx(iq);

	do {
		if (iq->flush_index == iq->octep_read_index) {
			break;
		}

		inst_processed = octep_iq_process_completions(iq, budget - tot_inst_processed);
		if (inst_processed) {
			atomic_subtract_int(&iq->instr_pending, inst_processed);
			iq->stats.instr_processed += inst_processed;
		}
		tot_inst_processed += inst_processed;
		inst_processed = 0;

	} while (tot_inst_processed < budget);

	if (tot_inst_processed >= budget) {
		tx_done = 0;
	}

	mtx_unlock(&iq->lock);
	mtx_unlock(&iq->iq_flush_running_lock);

	return tx_done;
}

static void
octep_ioq_intr_handler(void *data)
{
	struct octep_ioq_vector *ioq_vector = (struct octep_ioq_vector *)data;
	struct octep_device *oct = ioq_vector->octep_dev;
	struct octep_oq *oq = ioq_vector->oq;
	struct octep_iq *iq = ioq_vector->iq;
	int rx_budget = oct->rx_budget; 
	int tx_budget = oct->tx_budget;
	int rx_done = 0, tx_done = 1;

	if (!ioq_vector || !oq || !iq || !oct->netdev || 
		!(if_getdrvflags(oct->netdev) & IFF_DRV_RUNNING) || oq->suspend) {
		return;
	}

	if (octep_oq_check_hw_for_pkts(oct, oq)) {
		mtx_lock(&oq->lock);
		rx_done = octep_oq_process_rx(oq, rx_budget);
		mtx_unlock(&oq->lock);
	}

	if (atomic_load_acq_int(&iq->instr_pending)) {
		tx_done = octep_flush_iq(oct, iq, tx_budget);
	}

	if ((oct->netdev != NULL) && (iq->br != NULL)) {
		if (mtx_trylock(&iq->enq_lock)) {

			if (!drbr_empty(oct->netdev, iq->br))
			{
				octep_mq_start_locked(oct->netdev, iq);
			}
			mtx_unlock(&iq->enq_lock);
		}
	}


	if (rx_done < rx_budget && tx_done) {
		octep_update_pkt(iq, oq);
		octep_enable_ioq_irq(iq, oq);
	} else {
		taskqueue_enqueue(ioq_vector->oq_taskqueue, &ioq_vector->oq_task);
	}
}


static void
octep_oq_bh(void *arg, int pending __unused)
{
	struct octep_ioq_vector *ioq_vector = (struct octep_ioq_vector *)arg;
	struct octep_oq *oq = ioq_vector->oq;
	struct octep_iq *iq = ioq_vector->iq;
	struct octep_device *oct = ioq_vector->octep_dev;
	int rx_budget = oct->rx_budget;
	int tx_budget = oct->tx_budget;
	int rx_done = 0, tx_done = 1;

	if (!oct->netdev || !(if_getdrvflags(oct->netdev) & IFF_DRV_RUNNING) || 
		oq->suspend) {
		return;
	}

	mtx_lock(&oq->lock);
	rx_done = octep_oq_process_rx(oq, rx_budget);
	mtx_unlock(&oq->lock);

	if (atomic_load_acq_int(&iq->instr_pending)) {
		tx_done = octep_flush_iq(oct, iq, tx_budget);
	}


	if (oq->suspend || (rx_done < rx_budget && tx_done)) {
		octep_update_pkt(iq, oq);
		octep_enable_ioq_irq(iq, oq);
	} else {
		taskqueue_enqueue(ioq_vector->oq_taskqueue, &ioq_vector->oq_task);
	}
}

/**
 * octep_iq_full_check() - check if a Tx queue is full.
 *
 * @iq: Octeon Tx queue data structure.
 *
 * Return: 0, if the Tx queue is not full.
 *         1, if the Tx queue is full.
 */
static int octep_iq_full_check(struct octep_iq *iq)
{
	uint32_t space;
	struct ifnet *ifp = iq->ifp;

	mtx_lock(&iq->post_lock);
	space = IQ_INSTR_SPACE(iq);
	if (space > OCTEP_WAKE_QUEUE_THRESHOLD) {
		mtx_unlock(&iq->post_lock);
		return 0;
	}

	/* Stop the queue if unable to send */
	if_setdrvflagbits(ifp, IFF_DRV_OACTIVE, 0);

	/* Process completions to potentially free up space */
	octep_iq_process_completions(iq, OCTEP_IQ_COMPLETION_BATCH);
	space = IQ_INSTR_SPACE(iq);

	/* Check again and restart the queue if enough space is available */
	if (space > OCTEP_WAKE_QUEUE_THRESHOLD) {
		if_setdrvflagbits(ifp, 0, IFF_DRV_OACTIVE);
		mtx_unlock(&iq->post_lock);
		iq->stats.restart_cnt++;
		return 0;
	}

	mtx_unlock(&iq->post_lock);
	iq->stats.tx_busy++;
	return 1;
}

static int octep_xmit(struct octep_device *oct, struct octep_iq *iq, 
					  struct mbuf **m_headp)
{
	struct mbuf *m_head = *m_headp;
	struct octep_tx_buffer *tx_buffer;
	struct octep_tx_sglist_desc *sglist;
	struct octep_tx_desc_hw *hw_desc;
	struct octep_instr_hdr *ih;
	bus_dma_segment_t segs[OCTEP_SGLIST_ENTRIES_PER_PKT];
	bus_dmamap_t map;
	uint64_t dptr;
	uint32_t iq_no;
	int nsegs, status, i;
	uint16_t wi;
	bool force_doorbell = true;

	if (m_head->m_pkthdr.len < ETHER_MIN_LEN) {
		if (m_length(m_head, NULL) < ETHER_MIN_LEN) {
			struct mbuf *m_new = m_dup(m_head, M_NOWAIT);
			if (m_new == NULL) {
				status = ENOBUFS;
				dev_err(iq->dev, "%s m_dup failed: ENOBUFS\n", __func__);
				goto drop_packet;
			}
			m_freem(m_head);
			*m_headp = m_new;
			m_head = m_new;
		}
	}

	if (!(if_getdrvflags(iq->ifp) & IFF_DRV_RUNNING)) {
		status = ENETDOWN;
		goto drop_packet;
	}

	if (octep_iq_full_check(iq)) {
		dev_err(iq->dev, "%s == Queue full: wi=%u, flush_idx=%u, pending=%u\n",
					  __func__, iq->host_write_index, iq->flush_index, atomic_load_acq_int(&iq->instr_pending));
		return ENOBUFS;
	}

	wi = iq->host_write_index;
	tx_buffer = &iq->buff_info[wi];
	hw_desc = &iq->desc_ring[wi];
	map = tx_buffer->map;
	iq_no = iq->q_no;

	status = bus_dmamap_load_mbuf_sg(iq->tx_dma_tag, map, m_head, segs, &nsegs, BUS_DMA_NOWAIT);
	if (status == EFBIG) {
		struct mbuf *m = m_defrag(m_head, M_NOWAIT);
		if (m == NULL) {
			dev_err(iq->dev, "%s == Defragment failed\n", __func__);
			status = ENOBUFS;
			goto drop_packet;
		}
		*m_headp = m;
		m_head = m;
		status = bus_dmamap_load_mbuf_sg(iq->tx_dma_tag, map, m_head, segs, &nsegs, BUS_DMA_NOWAIT);
	}
	if (status == ENOMEM) {
		dev_err(iq->dev, "%s == DMA map failed: ENOBUFS\n", __func__);
		return ENOBUFS;
	} else if (status) {
		dev_err(iq->dev, "%s == DMA map failed for IQ-%d: %d\n", __func__, iq_no, status);
		goto drop_packet;
	}

	bus_dmamap_sync(iq->tx_dma_tag, map, BUS_DMASYNC_PREWRITE);
	tx_buffer->mb = m_head;

	hw_desc->ih64 = 0;
	ih = &hw_desc->ih;
	ih->pkind = oct->conf->fw_info.pkind;
	ih->fsz = oct->conf->fw_info.fsz;
	ih->tlen = m_head->m_pkthdr.len + ih->fsz;

	if (nsegs == 1) {
		tx_buffer->gather = 0;
		dptr = segs[0].ds_addr;
		hw_desc->dptr = dptr;
	} else {
		sglist = tx_buffer->sglist;
		ih->gsz = nsegs;
		ih->gather = 1;
		tx_buffer->gather = 1;

		memset(sglist, 0, OCTEP_SGLIST_SIZE_PER_PKT);
		for (i = 0; i < nsegs; i++) {
			sglist[i >> 2].len[3 - (i & 3)] = segs[i].ds_len;
			sglist[i >> 2].dma_ptr[i & 3] = segs[i].ds_addr;
		}
		hw_desc->dptr = tx_buffer->sglist_dma;
	}

	if (oct->conf->fw_info.tx_ol_flags) {
		if (m_head->m_pkthdr.csum_flags & CSUM_TSO) {
			hw_desc->txm.ol_flags = OCTEP_TX_OFFLOAD_CKSUM | OCTEP_TX_OFFLOAD_TSO;
			hw_desc->txm.gso_size = m_head->m_pkthdr.tso_segsz;
			hw_desc->txm.gso_segs = howmany(m_head->m_pkthdr.len, m_head->m_pkthdr.tso_segsz);
		} else if (m_head->m_pkthdr.csum_flags & (CSUM_IP | CSUM_TCP | CSUM_UDP)) {
			hw_desc->txm.ol_flags = OCTEP_TX_OFFLOAD_CKSUM;
		}
		hw_desc->txm64[0] = htobe64(hw_desc->txm64[0]);
	}

	wi = (wi + 1) & iq->ring_size_mask;
	iq->host_write_index = wi;
	iq->fill_cnt++;
	atomic_add_int(&iq->instr_pending, 1);

	bus_dmamap_sync(iq->desc_dma_tag, iq->desc_dmamap, BUS_DMASYNC_PREWRITE);
	if (tx_buffer->gather)
		bus_dmamap_sync(iq->sglist_dma_tag, iq->sglist_dmamap, BUS_DMASYNC_PREWRITE);

	wmb();


	if (force_doorbell ||
		(IQ_INSTR_PENDING(iq) >= (iq->max_count - OCTEP_WAKE_QUEUE_THRESHOLD)) ||
		iq->fill_cnt >= iq->fill_threshold) {

		/* Final sync before doorbell */
		bus_dmamap_sync(iq->desc_dma_tag, iq->desc_dmamap, BUS_DMASYNC_PREWRITE);
		wmb();

		octep_write_csr(oct, iq->doorbell_reg, iq->fill_cnt);

		iq->stats.instr_posted += iq->fill_cnt;
		iq->fill_cnt = 0;
	}

	return 0;

drop_packet:
	m_freem(*m_headp);
	*m_headp = NULL;
	return status;
}

int octep_mq_start_locked(if_t ifp, struct octep_iq *iq)
{
	struct octep_device *oct = if_getsoftc(ifp);
	struct mbuf *m;
	int err = 0;

	if (!(if_getdrvflags(ifp) & IFF_DRV_RUNNING))
		return ENETDOWN;

	while ((m = drbr_peek(ifp, iq->br)) != NULL) {
		err = octep_xmit(oct, iq, &m);
		if (err) {
			if (m == NULL)
				drbr_advance(ifp, iq->br);
			else
				drbr_putback(ifp, iq->br, m);
			break;
		}
		drbr_advance(ifp, iq->br);
		ETHER_BPF_MTAP(ifp, m);
	}

	return err;
}

static int
octep_mq_start(if_t ifp, struct mbuf *m)
{
	struct octep_device *oct = if_getsoftc(ifp);
	struct octep_iq *iq;
	int err, q_no;

	if (M_HASHTYPE_GET(m) != M_HASHTYPE_NONE)
		q_no = m->m_pkthdr.flowid % oct->num_iqs;
	else
		q_no = curcpu % oct->num_iqs;

	iq = oct->iq[q_no];

	err = drbr_enqueue(ifp, iq->br, m);
	if (err)
		return err;

	if (mtx_trylock(&iq->enq_lock)) {
		err = octep_mq_start_locked(ifp, iq);
		mtx_unlock(&iq->enq_lock);
	}

	return err;
}

/**
 * octep_free_ioq_vectors() - Free all IOQ vector resources allocated for the Octeon device.
 *
 * @oct: Octeon device private data structure.
 */
static void octep_free_ioq_vectors(struct octep_device *oct)
{
	struct octep_ioq_vector *ioq_vector;
	int i;

	for (i = 0; i < oct->num_oqs; i++) {
		ioq_vector = oct->ioq_vector[i];
		if (ioq_vector) {
			/* Clean up taskqueue */
			if (ioq_vector->oq_taskqueue) {
				while (taskqueue_cancel(ioq_vector->oq_taskqueue, &ioq_vector->oq_task, NULL))
					taskqueue_drain(ioq_vector->oq_taskqueue, &ioq_vector->oq_task);
				taskqueue_free(ioq_vector->oq_taskqueue);
				ioq_vector->oq_taskqueue = NULL;
			}
			free(oct->ioq_vector[i], M_DEVBUF);
			oct->ioq_vector[i] = NULL;
		}
	}

	dev_info(oct->pdev, "Freed %d IOQ vectors\n", oct->num_oqs);
}

/**
 * octep_disable_msix() - Disable MSI-X interrupts and release associated resources.
 *
 * @oct: Octeon device private data structure.
 */
static void octep_disable_msix(struct octep_device *oct)
{
	if (oct->num_irqs > 0) {
		pci_release_msi(oct->pdev);
		oct->num_irqs = 0;
	}
	dev_info(oct->pdev, "Disabled MSI-X\n");
}


static int
octep_alloc_ioq_vectors(struct octep_device *oct)
{
	int i, cpu_num;
	struct octep_ioq_vector *ioq_vector;

	for (i = 0; i < oct->num_oqs; i++) {
		oct->ioq_vector[i] = malloc(sizeof(*oct->ioq_vector[i]), M_DEVBUF, M_ZERO | M_WAITOK);

		if (oct->ioq_vector[i] == NULL)
			goto free_ioq_vector;

		ioq_vector = oct->ioq_vector[i];
		ioq_vector->iq = oct->iq[i];
		ioq_vector->oq = oct->oq[i];
		ioq_vector->octep_dev = oct;
		cpu_num = i % mp_ncpus;
		CPU_SETOF(cpu_num, &ioq_vector->affinity_mask);

		NET_TASK_INIT(&ioq_vector->oq_task, 0, octep_oq_bh, (void *)ioq_vector);
		ioq_vector->oq_taskqueue = taskqueue_create_fast("oct_oq_task", M_NOWAIT,
														 taskqueue_thread_enqueue,
														 &ioq_vector->oq_taskqueue);
		if (ioq_vector->oq_taskqueue == NULL) {
			free(oct->ioq_vector[i], M_DEVBUF);
			oct->ioq_vector[i] = NULL;
			dev_err(oct->pdev, "Failed to create taskqueue for IOQ %d\n", i);
			goto free_ioq_vector;
		}
		taskqueue_start_threads_cpuset(&ioq_vector->oq_taskqueue, 1, PI_NET,
									   &ioq_vector->affinity_mask,
									   "oct_oq%d_task", i);
	}

	dev_info(oct->pdev, "Allocated %d IOQ vectors\n", oct->num_oqs);
	return 0;

free_ioq_vector:
	while (i) {
		i--;
		ioq_vector = oct->ioq_vector[i];
		if (ioq_vector->oq_taskqueue) {
			while (taskqueue_cancel(ioq_vector->oq_taskqueue, &ioq_vector->oq_task, NULL))
				taskqueue_drain(ioq_vector->oq_taskqueue, &ioq_vector->oq_task);
			taskqueue_free(ioq_vector->oq_taskqueue);
			ioq_vector->oq_taskqueue = NULL;
		}
		free(oct->ioq_vector[i], M_DEVBUF);
		oct->ioq_vector[i] = NULL;
	}
	return -1;
}

/* Enable MSI-X interrupts */
static int
octep_enable_msix_range(struct octep_device *oct)
{
	int num_msix, msix_allocated;

	num_msix = oct->num_oqs + CFG_GET_NON_IOQ_MSIX(oct->conf);
	msix_allocated = num_msix;

	if (pci_alloc_msix(oct->pdev, &msix_allocated) || msix_allocated != num_msix) {
		dev_err(oct->pdev, "Failed to enable %d MSI-X irqs; got only %d\n",
				num_msix, msix_allocated);
		if (msix_allocated > 0)
		{
			dev_err(oct->pdev, "Error : msix_allocated > 0\n");
			pci_release_msi(oct->pdev);
		}
		return -1;
	}

	oct->num_irqs = msix_allocated;
	dev_info(oct->pdev, "MSI-X enabled successfully\n");
	return 0;
}

static int
octep_request_irqs(struct octep_device *oct)
{
	struct octep_ioq_vector *ioq_vector;
	char **non_ioq_msix_names;
	int num_non_ioq_msix, ret, i, j;
	int res_id;

	num_non_ioq_msix = CFG_GET_NON_IOQ_MSIX(oct->conf);
	non_ioq_msix_names = CFG_GET_NON_IOQ_MSIX_NAMES(oct->conf);

	/* Allocate memory for non-IOQ interrupt names, resources, and tags */
	oct->non_ioq_irq_names = malloc(num_non_ioq_msix * OCTEP_MSIX_NAME_SIZE,
									M_DEVBUF, M_WAITOK | M_ZERO);
	if (!oct->non_ioq_irq_names) {
		dev_err(oct->pdev, "Failed to allocate non-IOQ IRQ names\n");
		goto alloc_err;
	}

	oct->msix_res = mallocarray(num_non_ioq_msix, sizeof(struct resource *),
								M_DEVBUF, M_WAITOK | M_ZERO);
	if (!oct->msix_res) {
		dev_err(oct->pdev, "Failed to allocate non-IOQ msix_res array\n");
		goto non_ioq_res_err;
	}

	oct->tag = mallocarray(num_non_ioq_msix, sizeof(void *),
						   M_DEVBUF, M_WAITOK | M_ZERO);
	if (!oct->tag) {
		dev_err(oct->pdev, "Failed to allocate non-IOQ tag array\n");
		goto non_ioq_tag_err;
	}

	oct->aux_vector = mallocarray(num_non_ioq_msix, sizeof(int),
								  M_DEVBUF, M_WAITOK | M_ZERO);
	if (!oct->aux_vector) {
		dev_err(oct->pdev, "Failed to allocate non-IOQ aux_vector array\n");
		goto non_ioq_vector_err;
	}

	/* Register non-IOQ interrupts */
	for (i = 0; i < num_non_ioq_msix; i++) {

		char *irq_name;

		irq_name = &oct->non_ioq_irq_names[i * OCTEP_MSIX_NAME_SIZE];

		snprintf(irq_name, OCTEP_MSIX_NAME_SIZE,
				 "%s-%s", device_get_nameunit(oct->pdev), non_ioq_msix_names[i]);
		res_id = i + 1;

		oct->msix_res[i] = bus_alloc_resource_any(oct->pdev, SYS_RES_IRQ, &res_id,
												  RF_SHAREABLE | RF_ACTIVE);
		if (!oct->msix_res[i]) {
			dev_err(oct->pdev, "Unable to allocate bus res for non-IOQ IRQ %d\n", i);
			goto non_ioq_irq_err;
		}

		/* Register handler based on interrupt name */
		if (strncmp(non_ioq_msix_names[i], "epf_mbox_rint", strlen("epf_mbox_rint")) == 0) {
			ret = bus_setup_intr(oct->pdev, oct->msix_res[i], INTR_TYPE_NET | INTR_MPSAFE,
								 NULL, octep_mbox_intr_handler, oct, &oct->tag[i]);
		} else if (strncmp(non_ioq_msix_names[i], "epf_oei_rint", strlen("epf_oei_rint")) == 0) {
			ret = bus_setup_intr(oct->pdev, oct->msix_res[i], INTR_TYPE_NET | INTR_MPSAFE,
								 NULL, octep_oei_intr_handler, oct, &oct->tag[i]);
		} else if (strncmp(non_ioq_msix_names[i], "epf_ire_rint", strlen("epf_ire_rint")) == 0) {
			ret = bus_setup_intr(oct->pdev, oct->msix_res[i], INTR_TYPE_NET | INTR_MPSAFE,
								 NULL, octep_ire_intr_handler, oct, &oct->tag[i]);
		} else if (strncmp(non_ioq_msix_names[i], "epf_ore_rint", strlen("epf_ore_rint")) == 0) {
			ret = bus_setup_intr(oct->pdev, oct->msix_res[i], INTR_TYPE_NET | INTR_MPSAFE,
								 NULL, octep_ore_intr_handler, oct, &oct->tag[i]);
		} else if (strncmp(non_ioq_msix_names[i], "epf_vfire_rint",strlen("epf_vfire_rint")) == 0) {
			ret = bus_setup_intr(oct->pdev, oct->msix_res[i], INTR_TYPE_NET | INTR_MPSAFE,
								 NULL, octep_vfire_intr_handler, oct, &oct->tag[i]);
		} else if (strncmp(non_ioq_msix_names[i], "epf_vfore_rint", strlen("epf_vfore_rint")) == 0) {
			ret = bus_setup_intr(oct->pdev, oct->msix_res[i], INTR_TYPE_NET | INTR_MPSAFE,
								 NULL, octep_vfore_intr_handler, oct, &oct->tag[i]);
		} else if (strncmp(non_ioq_msix_names[i], "epf_dma_rint", strlen("epf_dma_rint")) == 0) {
			ret = bus_setup_intr(oct->pdev, oct->msix_res[i], INTR_TYPE_NET | INTR_MPSAFE,
								 NULL, octep_dma_intr_handler, oct, &oct->tag[i]);
		} else if (strncmp(non_ioq_msix_names[i], "epf_dma_vf_rint", strlen("epf_dma_vf_rint")) == 0) {
			ret = bus_setup_intr(oct->pdev, oct->msix_res[i], INTR_TYPE_NET | INTR_MPSAFE,
								 NULL, octep_dma_vf_intr_handler, oct, &oct->tag[i]);
		} else if (strncmp(non_ioq_msix_names[i], "epf_pp_vf_rint", strlen("epf_pp_vf_rint")) == 0) {
			ret = bus_setup_intr(oct->pdev, oct->msix_res[i], INTR_TYPE_NET | INTR_MPSAFE,
								 NULL, octep_pp_vf_intr_handler, oct, &oct->tag[i]);
		} else if (strncmp(non_ioq_msix_names[i], "epf_misc_rint", strlen("epf_misc_rint")) == 0) {
			ret = bus_setup_intr(oct->pdev, oct->msix_res[i], INTR_TYPE_NET | INTR_MPSAFE,
								 NULL, octep_misc_intr_handler, oct, &oct->tag[i]);
		} else {
			ret = bus_setup_intr(oct->pdev, oct->msix_res[i], INTR_TYPE_NET | INTR_MPSAFE,
								 NULL, octep_rsvd_intr_handler, oct, &oct->tag[i]);
		}

		if (ret) {
			bus_release_resource(oct->pdev, SYS_RES_IRQ, res_id, oct->msix_res[i]);
			dev_err(oct->pdev,"Failed to register intr handler for non-ioq interrupt %d\n", i); 
			goto non_ioq_irq_err;
		}

		bus_describe_intr(oct->pdev, oct->msix_res[i], oct->tag[i], "aux%d", i);
		oct->aux_vector[i] = res_id;
	}

	/* Register IOQ interrupts */
	for (j = 0; j < oct->num_oqs; j++) {
		ioq_vector = oct->ioq_vector[j];
		res_id = j + num_non_ioq_msix + 1;

		snprintf(ioq_vector->name, OCTEP_MSIX_NAME_SIZE, "%s-q%d", device_get_nameunit(oct->pdev), j);
		ioq_vector->msix_res = bus_alloc_resource_any(oct->pdev, SYS_RES_IRQ, &res_id,
													  RF_SHAREABLE | RF_ACTIVE);
		if (!ioq_vector->msix_res) {
			dev_err(oct->pdev, "Unable to allocate bus res for Q-%d\n", j);
			goto ioq_irq_err;
		}

		ret = bus_setup_intr(oct->pdev, ioq_vector->msix_res, INTR_TYPE_NET | INTR_MPSAFE,
							 NULL, octep_ioq_intr_handler, ioq_vector, &ioq_vector->tag);
		if (ret) {
			bus_release_resource(oct->pdev, SYS_RES_IRQ, res_id, ioq_vector->msix_res);
			dev_err(oct->pdev, "Failed to setup IRQ for Q-%d\n", j);
			goto ioq_irq_err;
		}

		bus_describe_intr(oct->pdev, ioq_vector->msix_res, ioq_vector->tag, "rxtx%d", j);
		ioq_vector->vector = res_id;

		/* Set CPU affinity */
		int cpu_id = j % mp_ncpus;
		CPU_SETOF(cpu_id, &ioq_vector->affinity_mask);
		ret = bus_bind_intr(oct->pdev, ioq_vector->msix_res, cpu_id);
		if (ret)
			dev_err(oct->pdev, "Failed to bind IRQ for Q-%d to CPU %d\n", j, cpu_id);
	}

	return 0;

ioq_irq_err:
	while (j--) {
		ioq_vector = oct->ioq_vector[j];
		if (ioq_vector->tag) {
			bus_teardown_intr(oct->pdev, ioq_vector->msix_res, ioq_vector->tag);
			ioq_vector->tag = NULL;
		}
		if (ioq_vector->msix_res) {
			bus_release_resource(oct->pdev, SYS_RES_IRQ, ioq_vector->vector,
								 ioq_vector->msix_res);
			ioq_vector->msix_res = NULL;
		}
	}
non_ioq_irq_err:
	while (i--) {
		if (oct->tag[i]) {
			bus_teardown_intr(oct->pdev, oct->msix_res[i], oct->tag[i]);
			oct->tag[i] = NULL;
		}
		if (oct->msix_res[i]) {
			bus_release_resource(oct->pdev, SYS_RES_IRQ, oct->aux_vector[i],
								 oct->msix_res[i]);
			oct->msix_res[i] = NULL;
		}
	}
non_ioq_vector_err:
	free(oct->aux_vector, M_DEVBUF);
	oct->aux_vector = NULL;
non_ioq_tag_err:
	free(oct->tag, M_DEVBUF);
	oct->tag = NULL;
non_ioq_res_err:
	free(oct->msix_res, M_DEVBUF);
	oct->msix_res = NULL;
alloc_err:
	free(oct->non_ioq_irq_names, M_DEVBUF);
	oct->non_ioq_irq_names = NULL;
	return -1;
}

/**
 * octep_free_irqs() - Free all interrupt resources allocated for the Octeon device.
 *
 * @oct: Octeon device private data structure.
 */
static void octep_free_irqs(struct octep_device *oct)
{
	struct octep_ioq_vector *ioq_vector;
	int i, j;

	/* Free IOQ interrupts */
	for (j = 0; j < oct->num_oqs; j++) {
		ioq_vector = oct->ioq_vector[j];
		if (ioq_vector && ioq_vector->tag) {
			bus_teardown_intr(oct->pdev, ioq_vector->msix_res, ioq_vector->tag);
			ioq_vector->tag = NULL;
		}
		if (ioq_vector && ioq_vector->msix_res) {
			bus_release_resource(oct->pdev, SYS_RES_IRQ, ioq_vector->vector, ioq_vector->msix_res);
			ioq_vector->msix_res = NULL;
			ioq_vector->vector = 0;
		}
	}

	/* Free non-IOQ interrupts */
	for (i = 0; i < CFG_GET_NON_IOQ_MSIX(oct->conf); i++) {
		if (oct->tag && oct->tag[i]) {
			bus_teardown_intr(oct->pdev, oct->msix_res[i], oct->tag[i]);
			oct->tag[i] = NULL;
		}
		if (oct->msix_res && oct->msix_res[i]) {
			bus_release_resource(oct->pdev, SYS_RES_IRQ, oct->aux_vector[i], oct->msix_res[i]);
			oct->msix_res[i] = NULL;
		}
	}

	/* Free non-IOQ resource arrays */
	if (oct->aux_vector) {
		free(oct->aux_vector, M_DEVBUF);
		oct->aux_vector = NULL;
	}
	if (oct->tag) {
		free(oct->tag, M_DEVBUF);
		oct->tag = NULL;
	}
	if (oct->msix_res) {
		free(oct->msix_res, M_DEVBUF);
		oct->msix_res = NULL;
	}
	if (oct->non_ioq_irq_names) {
		free(oct->non_ioq_irq_names, M_DEVBUF);
		oct->non_ioq_irq_names = NULL;
	}

	dev_info(oct->pdev, "Freed all interrupt resources\n");
}


/**
 * octep_clean_irqs() - Free all interrupts and their resources.
 *
 * @oct: Octeon device private data structure.
 */
static void octep_clean_irqs(struct octep_device *oct)
{
	octep_free_irqs(oct);
	octep_disable_msix(oct);
	octep_free_ioq_vectors(oct);
}

static int
octep_setup_irqs(struct octep_device *oct)
{
	if (octep_alloc_ioq_vectors(oct))
		goto ioq_vector_err;

	if (octep_enable_msix_range(oct))
		goto enable_msix_err;

	if (octep_request_irqs(oct))
		goto request_irq_err;

	return 0;

request_irq_err:
	octep_disable_msix(oct);
enable_msix_err:
	octep_free_ioq_vectors(oct);
ioq_vector_err:
	return -1;
}

void octep_open(void *arg)
{
	struct octep_device *oct = arg;
	if_t ifp = oct->netdev;

	/* Check if interface is already running */
	if (if_getdrvflags(ifp) & IFF_DRV_RUNNING) {
		return;
	}

	if (oct->link_info.admin_up) {
		return;
	}

	/* Ensure device is in READY state */
	if (atomic_load_acq_int(&oct->status) != OCTEP_DEV_STATUS_READY) {
		dev_err(oct->pdev, "Device not ready, aborting open\n");
		return;
	}

	/* Clear running flag (redundant but safe) */
	if_setdrvflagbits(ifp, 0, IFF_DRV_RUNNING);

	/* Reset I/O queues */
	oct->hw_ops.reset_io_queues(oct);

	/* Setup input queues */
	if (octep_setup_iqs(oct)) {
		dev_err(oct->pdev, "Failed to setup input queues\n");
		goto setup_iq_err;
	}

	/* Setup output queues */
	if (octep_setup_oqs(oct)) {
		dev_err(oct->pdev, "Failed to setup output queues\n");
		goto setup_oq_err;
	}

	/* Setup interrupts */
	if (octep_setup_irqs(oct)) {
		dev_err(oct->pdev, "Failed to setup interrupts\n");
		goto setup_irq_err;
	}

	/* Set link state */
	oct->link_info.admin_up = 1;
	oct->poll_non_ioq_intr = false;

	octep_ctrl_net_set_rx_state(oct, OCTEP_CTRL_NET_INVALID_VFID, true, false);
	octep_ctrl_net_set_link_status(oct, OCTEP_CTRL_NET_INVALID_VFID, true, false);

	/* Enable interrupts */
	oct->hw_ops.enable_interrupts(oct);

	/* Enable I/O queues */
	oct->hw_ops.enable_io_queues(oct);

	/* Initialize output queue doorbells */
	octep_oq_dbell_init(oct);

	/* Check link status and update if up */
	if (octep_ctrl_net_get_link_status(oct, OCTEP_CTRL_NET_INVALID_VFID)) {
		if_link_state_change(ifp, LINK_STATE_UP);
	}

	/* Mark interface as running */
	if_setdrvflagbits(ifp, IFF_DRV_RUNNING, 0);
	octep_ifstate_set(oct, OCTEP_DEV_STATE_OPEN);

	dev_info(oct->pdev, "Started netdev ...\n");
	return;

setup_irq_err:
	octep_free_oqs(oct);
setup_oq_err:
	octep_free_iqs(oct);
setup_iq_err:
	octep_clean_irqs(oct);
	dev_err(oct->pdev, "Failed to open netdev\n");
}

int octep_stop(if_t ifp)
{
	struct octep_device *oct = if_getsoftc(ifp);

	oct->poll_non_ioq_intr = true;
	octep_ifstate_reset(oct, OCTEP_DEV_STATE_OPEN);
	if_link_state_change(ifp, LINK_STATE_DOWN);
	if_setdrvflagbits(ifp, 0, IFF_DRV_RUNNING);

	/* Disable interrupts and I/O queues */
	oct->hw_ops.disable_interrupts(oct);
	oct->hw_ops.disable_io_queues(oct);

	/* Clean up resources */
	octep_clean_irqs(oct);
	octep_free_oqs(oct);
	octep_free_iqs(oct);

	/* Reset link state */
	oct->link_info.admin_up = 0;
	octep_ctrl_net_set_rx_state(oct, OCTEP_CTRL_NET_INVALID_VFID, false, false);
	octep_ctrl_net_set_link_status(oct, OCTEP_CTRL_NET_INVALID_VFID, false, false);

	dev_info(oct->pdev, "Stopped netdev\n");
	return 0;
}

static void
octep_ifmedia_status(if_t ifp, struct ifmediareq *ifmr)
{
	struct octep_device *oct = if_getsoftc(ifp);
	uint64_t advertised_modes, supported_modes;

	/* Setup the default interface info. */
	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;


	advertised_modes = oct->link_info.advertised_modes;
	supported_modes = oct->link_info.supported_modes;
	OCTEP_SET_LINK_MODES_BITMAP(supported_modes);
	OCTEP_SET_LINK_MODES_BITMAP(advertised_modes);

	ifmr->ifm_status |= IFM_ACTIVE;
	ifmr->ifm_active |= IFM_FDX;


	if (oct->link_info.autoneg)
	{
		ifmr->ifm_active |= IFM_AUTO;
	}

	if (oct->link_info.pause & OCTEP_LINK_MODE_PAUSE_ADVERTISED)
	{
		ifmr->ifm_active |= IFM_ETH_TXPAUSE | IFM_ETH_RXPAUSE;
	}

}

static int
octep_ifmedia_update(if_t ifp)
{
	struct octep_device *oct = if_getsoftc(ifp);
	struct ifmedia *ifm = &oct->ifmedia;
	struct octep_iface_link_info link_info_new;
	uint64_t advertised = 0;
	uint8_t autoneg = 0;
	int error;

	if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER) {
		dev_err(oct->pdev, "Invalid media type (%d)\n", IFM_TYPE(ifm->ifm_media));
		return EINVAL;
	}

	if (ifm->ifm_media & IFM_HDX) {
		dev_err(oct->pdev, "Half duplex not supported\n");
		return EOPNOTSUPP;
	}

	memcpy(&link_info_new, &oct->link_info, sizeof(struct octep_iface_link_info));

	if (IFM_SUBTYPE(ifm->ifm_media) == IFM_AUTO) {
		if (!(oct->link_info.autoneg & OCTEP_LINK_MODE_AUTONEG_SUPPORTED)) {
			dev_err(oct->pdev, "Autonegotiation not supported\n");
			return EOPNOTSUPP;
		}
		autoneg = OCTEP_LINK_MODE_AUTONEG_SUPPORTED;
	}

	switch (IFM_SUBTYPE(ifm->ifm_media)) {
	case IFM_AUTO:
		advertised = oct->link_info.supported_modes;
		link_info_new.speed = 0;
		break;
	case IFM_10G_T:
		advertised |= BIT(OCTEP_LINK_MODE_10GBASE_T);
		link_info_new.speed = 10000;
		break;
	case IFM_10G_SR:
		advertised |= BIT(OCTEP_LINK_MODE_10GBASE_SR);
		link_info_new.speed = 10000;
		break;
	case IFM_10G_LR:
		advertised |= BIT(OCTEP_LINK_MODE_10GBASE_LR);
		link_info_new.speed = 10000;
		break;
	case IFM_10G_CR1:
		advertised |= BIT(OCTEP_LINK_MODE_10GBASE_CR);
		link_info_new.speed = 10000;
		break;
	case IFM_10G_KR:
		advertised |= BIT(OCTEP_LINK_MODE_10GBASE_KR);
		link_info_new.speed = 10000;
		autoneg = oct->link_info.autoneg & OCTEP_LINK_MODE_AUTONEG_SUPPORTED ? OCTEP_LINK_MODE_AUTONEG_SUPPORTED : 0;
		break;
	case IFM_25G_SR:
		advertised |= BIT(OCTEP_LINK_MODE_25GBASE_SR);
		link_info_new.speed = 25000;
		break;
	case IFM_25G_CR:
		advertised |= BIT(OCTEP_LINK_MODE_25GBASE_CR);
		link_info_new.speed = 25000;
		break;
	case IFM_25G_KR:
		advertised |= BIT(OCTEP_LINK_MODE_25GBASE_KR);
		link_info_new.speed = 25000;
		autoneg = oct->link_info.autoneg & OCTEP_LINK_MODE_AUTONEG_SUPPORTED ? OCTEP_LINK_MODE_AUTONEG_SUPPORTED : 0;
		break;
	case IFM_40G_CR4:
		advertised |= BIT(OCTEP_LINK_MODE_40GBASE_CR4);
		link_info_new.speed = 40000;
		break;
	case IFM_40G_KR4:
		advertised |= BIT(OCTEP_LINK_MODE_40GBASE_KR4);
		link_info_new.speed = 40000;
		autoneg = oct->link_info.autoneg & OCTEP_LINK_MODE_AUTONEG_SUPPORTED ? OCTEP_LINK_MODE_AUTONEG_SUPPORTED : 0;
		break;
	case IFM_40G_LR4:
		advertised |= BIT(OCTEP_LINK_MODE_40GBASE_LR4);
		link_info_new.speed = 40000;
		break;
	case IFM_40G_SR4:
		advertised |= BIT(OCTEP_LINK_MODE_40GBASE_SR4);
		link_info_new.speed = 40000;
		break;
	case IFM_50G_LR:
		advertised |= BIT(OCTEP_LINK_MODE_50GBASE_LR);
		link_info_new.speed = 50000;
		break;
	case IFM_50G_SR:
		advertised |= BIT(OCTEP_LINK_MODE_50GBASE_SR);
		link_info_new.speed = 50000;
		break;
	case IFM_50G_CR2:
		advertised |= BIT(OCTEP_LINK_MODE_50GBASE_CR2);
		link_info_new.speed = 50000;
		break;
	case IFM_50G_KR2:
		advertised |= BIT(OCTEP_LINK_MODE_50GBASE_KR2);
		link_info_new.speed = 50000;
		autoneg = oct->link_info.autoneg & OCTEP_LINK_MODE_AUTONEG_SUPPORTED ? OCTEP_LINK_MODE_AUTONEG_SUPPORTED : 0;
		break;
	case IFM_50G_SR2:
		advertised |= BIT(OCTEP_LINK_MODE_50GBASE_SR2);
		link_info_new.speed = 50000;
		break;
	case IFM_100G_CR4:
		advertised |= BIT(OCTEP_LINK_MODE_100GBASE_CR4);
		link_info_new.speed = 100000;
		break;
	case IFM_100G_KR4:
		advertised |= BIT(OCTEP_LINK_MODE_100GBASE_KR4);
		link_info_new.speed = 100000;
		autoneg = oct->link_info.autoneg & OCTEP_LINK_MODE_AUTONEG_SUPPORTED ? OCTEP_LINK_MODE_AUTONEG_SUPPORTED : 0;
		break;
	case IFM_100G_LR4:
		advertised |= BIT(OCTEP_LINK_MODE_100GBASE_LR4);
		link_info_new.speed = 100000;
		break;
	case IFM_100G_SR4:
		advertised |= BIT(OCTEP_LINK_MODE_100GBASE_SR4);
		link_info_new.speed = 100000;
		break;
	default:
		dev_err(oct->pdev, "Invalid media subtype (%d)\n", IFM_SUBTYPE(ifm->ifm_media));
		return EINVAL;
	}

	if (advertised & ~oct->link_info.supported_modes) {
		dev_err(oct->pdev, "Requested modes not supported\n");
		return EINVAL;
	}

	if (advertised == oct->link_info.advertised_modes &&
		link_info_new.speed == oct->link_info.speed &&
		autoneg == oct->link_info.autoneg) {
		return 0;
	}

	link_info_new.advertised_modes = advertised;
	link_info_new.autoneg = autoneg;

	error = octep_ctrl_net_set_link_info(oct, OCTEP_CTRL_NET_INVALID_VFID, &link_info_new, true);
	if (error) {
		dev_err(oct->pdev, "Failed to set link info: %d\n", error);
		return error;
	}

	memcpy(&oct->link_info, &link_info_new, sizeof(struct octep_iface_link_info));
	return 0;
}

static uint64_t
octep_get_counter(if_t ifp, ift_counter cnt)
{
	struct octep_device *oct = if_getsoftc(ifp);
	uint64_t counter = 0;
	int q;
	uint64_t current_ticks = ticks;
	int refresh_interval = octep_ms_to_ticks(STATS_REFRESH_INTERVAL_MS);

	/* Return 0 if interface is not running */
	if (!(if_getdrvflags(ifp) & IFF_DRV_RUNNING)) {
		return 0;
	}

	/* Check if stats need refreshing */
	mtx_lock(&oct->lock);
	if ((current_ticks - oct->stats_last_update) >= refresh_interval ||
		oct->stats_last_update == 0) {
		octep_ctrl_net_get_if_stats(oct, OCTEP_CTRL_NET_INVALID_VFID,
									&oct->iface_rx_stats, &oct->iface_tx_stats);
		oct->stats_last_update = current_ticks;
	}
	mtx_unlock(&oct->lock);

	switch (cnt) {
	case IFCOUNTER_IPACKETS:
		for (q = 0; q < oct->num_oqs; q++)
			counter += oct->oq[q]->stats.packets;
		break;
	case IFCOUNTER_OPACKETS:
		for (q = 0; q < oct->num_iqs; q++)
			counter += oct->iq[q]->stats.instr_completed;
		break;
	case IFCOUNTER_IBYTES:
		for (q = 0; q < oct->num_oqs; q++)
			counter += oct->oq[q]->stats.bytes;
		break;
	case IFCOUNTER_OBYTES:
		for (q = 0; q < oct->num_iqs; q++)
			counter += oct->iq[q]->stats.bytes_sent;
		break;
	case IFCOUNTER_IQDROPS:
		for (q = 0; q < oct->num_iqs; q++)
			counter += oct->iq[q]->stats.instr_dropped;
		break;
	case IFCOUNTER_OQDROPS:
		for (q = 0; q < oct->num_oqs; q++)
			counter += oct->oq[q]->stats.alloc_failures;
		break;
	case IFCOUNTER_IMCASTS:
		counter = oct->iface_rx_stats.mcast_pkts;
		break;
	case IFCOUNTER_OMCASTS:
		counter = oct->iface_tx_stats.mcst;
		break;
	case IFCOUNTER_COLLISIONS:
		counter = oct->iface_tx_stats.xscol + oct->iface_tx_stats.mcol + oct->iface_tx_stats.scol;
		break;
	case IFCOUNTER_IERRORS:
		counter = oct->iface_rx_stats.err_pkts;
		break;
	default:
		return if_get_counter_default(ifp, cnt);
	}

	return counter;
}

static void
octep_qflush(if_t ifp)
{
	struct octep_device *oct = if_getsoftc(ifp);
	struct mbuf *m;
	int i;

	mtx_lock(&oct->lock);
	for (i = 0; i < oct->num_iqs; i++) {
		struct octep_iq *iq = oct->iq[i];
		mtx_lock(&iq->enq_lock);
		while ((m = buf_ring_dequeue_sc(iq->br)) != NULL)
			m_freem(m);
		mtx_unlock(&iq->enq_lock);
	}
	mtx_unlock(&oct->lock);
}

static void
octep_tx_timeout_task(void *context, int pending)
{
	struct octep_device *oct = (struct octep_device *)context;
	struct ifnet *ifp = oct->netdev;

	mutex_lock(&oct->lock);
	if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		if_down(ifp);
		if_up(ifp);  /* Restart interface */
	}
	mutex_unlock(&oct->lock);
}

static void
octep_hb_timeout_task(void *context)
{
	struct octep_device *oct = (struct octep_device *)context;
	int status, miss_cnt;

	status = atomic_load_acq_int(&oct->status);
	if (status != OCTEP_DEV_STATUS_INIT && status != OCTEP_DEV_STATUS_READY)
		return;

	miss_cnt = atomic_fetchadd_int(&oct->hb_miss_cnt, 1) + 1;
	if (miss_cnt < oct->conf->fw_info.hb_miss_count) {
		callout_reset(&oct->hb_callout, (oct->conf->fw_info.hb_interval * hz) / 1000,
					  octep_hb_timeout_task, oct);
		return;
	}

	dev_info(oct->pdev, "Missed %u heartbeats. Carrier signal lost." 
			 "If a Virtual Function (VF) is active, reboot the board.\n", miss_cnt);
	if_link_state_change(oct->netdev, LINK_STATE_DOWN);
}

static void
octep_ctrl_mbox_task(void *context, int pending)
{
	struct octep_device *oct = (struct octep_device *)context;
	int status;

	status = atomic_load_acq_int(&oct->status);
	if (status != OCTEP_DEV_STATUS_INIT && status != OCTEP_DEV_STATUS_READY)
		return;

	octep_ctrl_net_recv_fw_messages(oct);
}

static void
octep_intr_poll_task(void *context)
{
	struct octep_device *oct = (struct octep_device *)context;
	int status;

	status = atomic_load_acq_int(&oct->status);
	if ((status != OCTEP_DEV_STATUS_INIT && status != OCTEP_DEV_STATUS_READY) ||
		!oct->poll_non_ioq_intr) {
		dev_info(oct->pdev, "Interrupt poll task stopped.\n");
		return;
	}

	oct->hw_ops.poll_non_ioq_interrupts(oct);
	callout_reset(&oct->intr_poll_callout, (OCTEP_INTR_POLL_TIME_MSECS * hz) / 1000,
				  octep_intr_poll_task, oct);
}

static bool
get_fw_ready_status(struct octep_device *oct)
{
	uint32_t pos;
	uint16_t vsec_id;
	uint8_t status;
	int error;

	/* Iterate through PCI extended capabilities */
	for (error = pci_find_extcap(oct->pdev, PCIM_EXTCAP_VENDORSPECIFIC, &pos);
		 error == 0;
		 error = pci_find_next_extcap(oct->pdev, PCIM_EXTCAP_VENDORSPECIFIC, pos, &pos)) {

		/* Read vendor-specific ID (2 bytes at offset +4) */
		vsec_id = pci_read_config(oct->pdev, pos + 4, 2);

		if (vsec_id != FW_STATUS_VSEC_ID)
			continue;

		/* Read firmware status (1 byte at offset +8) */
		status = pci_read_config(oct->pdev, pos + 8, 1);

		if (status == FW_STATUS_READY)
			return true;
	}

	return false;
}

static const char *octep_devid_to_str(struct octep_device *oct)
{
	switch (oct->chip_id) {
	case OCTEP_PCI_DEVICE_ID_CN98_PF:
		return "CN98XX";
	case OCTEP_PCI_DEVICE_ID_CN93_PF:
		return "CN93XX";
	case OCTEP_PCI_DEVICE_ID_CNF95O_PF:
		return "CNF95O";
	case OCTEP_PCI_DEVICE_ID_CNF95N_PF:
		return "CNF95N";
	case OCTEP_PCI_DEVICE_ID_CN10KA_PF:
		return "CN10KA";
	case OCTEP_PCI_DEVICE_ID_CNF10KA_PF:
		return "CNF10KA";
	case OCTEP_PCI_DEVICE_ID_CNF10KB_PF:
		return "CNF10KB";
	case OCTEP_PCI_DEVICE_ID_CN10KB_PF:
		return "CN10KB";
	default:
		return "Unsupported";
	}
}

static int
octep_sriov_init(device_t dev)
{
	nvlist_t *pf_schema, *vf_schema;
	int iov_pos, err;
	uint16_t total_vf_cnt;
	struct octep_device *oct = device_get_softc(dev);

	err = pci_find_extcap(dev, PCIZ_SRIOV, &iov_pos);
	if (err != 0) {
		dev_info(dev, "SR-IOV capability not found (err=%d)\n", err);
		return (err);
	}

	total_vf_cnt = pci_read_config(dev, iov_pos + PCIR_SRIOV_TOTAL_VFS, 2);
	if (total_vf_cnt == 0) {
		dev_info(dev, "SR-IOV Total VFs reported as 0\n");
		return (ENXIO);
	}

	oct->num_vf_en = total_vf_cnt;
	dev_dbg(dev, "SR-IOV Total VFs supported: %u\n", total_vf_cnt);

	pf_schema = pci_iov_schema_alloc_node();
	vf_schema = pci_iov_schema_alloc_node();
	if (pf_schema == NULL || vf_schema == NULL) {
		dev_info(dev, "Failed to allocate IOV schema\n");
		if (pf_schema) nvlist_destroy(pf_schema);
		if (vf_schema) nvlist_destroy(vf_schema);
		return (ENOMEM);
	}


	pci_iov_schema_add_unicast_mac(vf_schema, "mac-addr", 0, NULL);
	pci_iov_schema_add_bool(vf_schema, "allow-set-mac", IOV_SCHEMA_HASDEFAULT, TRUE);
	dev_info(dev, "IOV schema configured\n");
	err = pci_iov_attach(dev, pf_schema, vf_schema);
	if (err != 0) {
		dev_info(dev, "pci_iov_attach failed (err=%d)\n", err);
		nvlist_destroy(pf_schema);
		nvlist_destroy(vf_schema);
		return (err);
	}

	oct->sriov_enabled = true;
	dev_info(dev, "SR-IOV attached successfully\n");
	return (0);
}

int
octep_device_setup(struct octep_device *oct)
{
	device_t pdev = oct->pdev;
	int err, i;

	oct->conf = malloc(sizeof(*oct->conf), M_DEVBUF, M_WAITOK | M_ZERO);
	if (!oct->conf)
		return -ENOMEM;

	for (i = 0; i < OCTEP_MMIO_REGIONS; i++) {
		if (octep_map_pci_barx(oct, i)) {
			dev_err(pdev, "Failed to map BAR%d\n", i * 2);
			goto ioremap_err;
		}
	}

	oct->chip_id = pci_get_device(pdev);
	oct->rev_id = pci_get_revid(pdev);
	dev_info(pdev, "chip_id = 0x%x\n", oct->chip_id);

	dev_info(pdev, "Setting up OCTEON %s PF PASS%d.%d\n",
			 octep_devid_to_str(oct), OCTEP_MAJOR_REV(oct), OCTEP_MINOR_REV(oct));

	if (oct->chip_id == OCTEP_PCI_DEVICE_ID_CN98_PF ||
		oct->chip_id == OCTEP_PCI_DEVICE_ID_CN93_PF ||
		oct->chip_id == OCTEP_PCI_DEVICE_ID_CNF95O_PF ||
		oct->chip_id == OCTEP_PCI_DEVICE_ID_CNF95N_PF) {
		octep_device_setup_cn93_pf(oct);
	} else if (oct->chip_id == OCTEP_PCI_DEVICE_ID_CNF10KA_PF ||
			   oct->chip_id == OCTEP_PCI_DEVICE_ID_CN10KA_PF ||
			   oct->chip_id == OCTEP_PCI_DEVICE_ID_CNF10KB_PF ||
			   oct->chip_id == OCTEP_PCI_DEVICE_ID_CN10KB_PF) {
		octep_device_setup_cnxk_pf(oct);
	} else {
		dev_err(pdev, "%s: Unsupported device\n", __func__);
		goto unsupported_dev;
	}

	err = octep_ctrl_net_init(oct);
	if (err)
		return err;

	err = octep_setup_pfvf_mbox(oct);
	if (err) {
		dev_err(pdev, "PF-VF mailbox setup failed\n");
		octep_ctrl_net_uninit(oct);
		return err;
	}

	err = octep_sriov_init(pdev);
	if (err != 0) {
		dev_err(pdev, "Failed to initialize SR-IOV (err=%d)\n", err);
		return err;
	}
	dev_info(pdev, "SR-IOV initialized successfully\n");

	TASK_INIT(&oct->tx_timeout_task, 0, octep_tx_timeout_task, oct);
	TASK_INIT(&oct->ctrl_mbox_task, 0, octep_ctrl_mbox_task, oct);
	callout_init(&oct->intr_poll_callout, 1);
	callout_init(&oct->hb_callout, 1);

	oct->poll_non_ioq_intr = true;
	callout_reset(&oct->intr_poll_callout, (OCTEP_INTR_POLL_TIME_MSECS * hz) / 1000,
				  octep_intr_poll_task, oct);

	atomic_store_rel_int(&oct->hb_miss_cnt, 0);

	return 0;

ioremap_err:
	while (i > 0) {
		i--;
		octep_unmap_pci_barx(oct, i);
	}
unsupported_dev:
	free(oct->conf, M_DEVBUF);
	oct->conf = NULL;
	return -1;
}

static void
octep_device_cleanup(struct octep_device *oct)
{
	int i;

	dev_info(oct->pdev, "Cleaning up Octeon Device ...\n");

	callout_drain(&oct->hb_callout);
	callout_drain(&oct->intr_poll_callout);

	/* Cancel and drain all tasks */
	while (taskqueue_cancel(octep_tq, &oct->tx_timeout_task, NULL))
		taskqueue_drain(octep_tq, &oct->tx_timeout_task);
	while (taskqueue_cancel(octep_tq, &oct->ctrl_mbox_task, NULL))
		taskqueue_drain(octep_tq, &oct->ctrl_mbox_task);
	while (taskqueue_cancel(octep_tq, &oct->dev_setup_task, NULL))
		taskqueue_drain(octep_tq, &oct->dev_setup_task);

	/* Clean up interrupts and IOQ vectors */
	if (atomic_load_acq_int(&oct->status) >= OCTEP_DEV_STATUS_INIT) {
		dev_info(oct->pdev, "Cleaning up interrupts and IOQ vectors\n");
		octep_clean_irqs(oct);
	}

	oct->poll_non_ioq_intr = false;
	octep_delete_pfvf_mbox(oct);
	octep_ctrl_net_uninit(oct);

	dev_info(oct->pdev, "Calling soft_reset\n");
	//oct->hw_ops.soft_reset(oct);
	for (i = 0; i < OCTEP_MMIO_REGIONS; i++) {
		dev_info(oct->pdev, "octep_unmap_pci_barx\n");
		octep_unmap_pci_barx(oct, i);
	}

	/* Free configuration */
	if (oct->conf) {
		free(oct->conf, M_DEVBUF);
		oct->conf = NULL;
	}
}

static int
octep_ifnet_init(struct octep_device *oct)
{
	if_t ifp = oct->netdev;

	if (!oct || !ifp || !ifp->if_softc) {
		dev_err(oct->pdev, "Invalid oct or ifp: oct=%p, ifp=%p, ifp->if_softc=%p\n",
				oct, ifp, ifp ? ifp->if_softc : NULL);
		atomic_store_rel_int(&oct->status, OCTEP_DEV_STATUS_INIT);
		return -EINVAL;
	}

	if (atomic_load_acq_int(&oct->status) != OCTEP_DEV_STATUS_INIT) {
		dev_err(oct->pdev, "Device not in INIT state, aborting ifnet init\n");
		atomic_store_rel_int(&oct->status, OCTEP_DEV_STATUS_INIT);
		return -EINVAL;
	}

	ifmedia_init(&oct->ifmedia, IFM_IMASK, octep_ifmedia_update, octep_ifmedia_status);
	ifmedia_add(&oct->ifmedia, IFM_ETHER | IFM_10G_T | IFM_FDX, 0, NULL);
	ifmedia_add(&oct->ifmedia, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_set(&oct->ifmedia, IFM_ETHER | IFM_AUTO);

	oct->ifmedia.ifm_media = oct->ifmedia.ifm_cur->ifm_media;

	if_setflags(ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);

	if_setioctlfn(ifp, octep_ioctl);
	if_setgetcounterfn(ifp, octep_get_counter);
	if_settransmitfn(ifp, octep_mq_start);
	if_setqflushfn(ifp, octep_qflush);
	if_setinitfn(ifp, octep_open);

	if_setmtu(ifp, OCTEP_DEFAULT_MTU);

	if_setbaudrate(ifp, oct->link_info.speed * 1000000ULL);

	return 0;
}

static void
octep_dev_setup_task(void *context, int pending)
{
	struct octep_device *oct = (struct octep_device *)context;
	struct ifnet *ifp = oct->netdev;
	int err;
	int max_rx_pktlen;

	uint64_t start_time = ticks;
	uint64_t timeout_ticks = octep_ms_to_ticks(FW_POLL_TIMEOUT_MS);

	// Check if device is still valid
	if (!oct->pdev || atomic_load_acq_int(&oct->status) == OCTEP_DEV_STATUS_UNINIT) {
		dev_info(oct->pdev, "Device detached before setup task started\n");
		return;
	}

	atomic_store_rel_int(&oct->status, OCTEP_DEV_STATUS_WAIT_FOR_FW);

	while (true) {
		if (get_fw_ready_status(oct))
			break;

		/* Check for timeout */
		if ((ticks - start_time) >= timeout_ticks) {
			dev_info(oct->pdev, "Firmware polling timed out\n");
			atomic_store_rel_int(&oct->status, OCTEP_DEV_STATUS_ALLOC);
			return;
		}

		/* Check if detach has set status to UNINIT */
		if (atomic_load_acq_int(&oct->status) == OCTEP_DEV_STATUS_UNINIT) {
			dev_info(oct->pdev, "Setup task aborted due to detach\n");
			return;
		}

		octep_mdelay(1000);

	}

	atomic_store_rel_int(&oct->status, OCTEP_DEV_STATUS_INIT);
	err = octep_device_setup(oct);
	if (err) {
		dev_err(oct->pdev, "Device setup failed\n");
		atomic_store_rel_int(&oct->status, OCTEP_DEV_STATUS_ALLOC);
		return;
	}

	// Ensure device is still valid before proceeding
	if (!oct->pdev || atomic_load_acq_int(&oct->status) == OCTEP_DEV_STATUS_UNINIT) {
		dev_info(oct->pdev, "Device detached during setup\n");
		return;
	}

	octep_ctrl_net_get_info(oct, OCTEP_CTRL_NET_INVALID_VFID, &oct->conf->fw_info);
	dev_info(oct->pdev, "Heartbeat interval %u msecs Heartbeat miss count %u\n",
			 oct->conf->fw_info.hb_interval, oct->conf->fw_info.hb_miss_count);

	callout_reset(&oct->hb_callout, (oct->conf->fw_info.hb_interval * hz) / 1000,
				  octep_hb_timeout_task, oct);

	ifp->if_capabilities = IFCAP_HWCSUM | IFCAP_TSO4 | IFCAP_TSO6;
	if (OCTEP_TX_IP_CSUM(oct->conf->fw_info.tx_ol_flags))
		ifp->if_capabilities |= (IFCAP_TXCSUM | IFCAP_TXCSUM_IPV6);
	if (OCTEP_RX_IP_CSUM(oct->conf->fw_info.rx_ol_flags))
		ifp->if_capabilities |= IFCAP_RXCSUM;
	ifp->if_capenable = ifp->if_capabilities;

	max_rx_pktlen = octep_ctrl_net_get_mtu(oct, OCTEP_CTRL_NET_INVALID_VFID);
	if (max_rx_pktlen < 0) {
		dev_err(oct->pdev, "Failed to get max receive packet size; err = %d\n"
				, max_rx_pktlen);
		atomic_store_rel_int(&oct->status, OCTEP_DEV_STATUS_INIT);
		return;
	}

	oct->max_rx_pktlen = max_rx_pktlen - (ETHER_HDR_LEN + ETHER_CRC_LEN);
	octep_ctrl_net_get_mac_addr(oct, OCTEP_CTRL_NET_INVALID_VFID, oct->mac_addr);
	octep_ctrl_net_get_link_info(oct, OCTEP_CTRL_NET_INVALID_VFID, &oct->link_info);

	// Initialize ifnet
	if_initname(oct->netdev, device_get_name(oct->pdev), device_get_unit(oct->pdev));
	if_setsoftc(oct->netdev, oct);
	if_setdrvflagbits(oct->netdev, 0, IFF_DRV_RUNNING);
	if_setflags(oct->netdev, 0);
	if_link_state_change(oct->netdev, LINK_STATE_DOWN);

	// Ensure device is still in INIT state before proceeding
	if (atomic_load_acq_int(&oct->status) != OCTEP_DEV_STATUS_INIT) {
		dev_err(oct->pdev, "Device not in INIT state, aborting ifnet init\n");
		return;
	}

	err = octep_ifnet_init(oct);
	if (err) {
		dev_err(oct->pdev, "Failed to initialize ifnet\n");
		atomic_store_rel_int(&oct->status, OCTEP_DEV_STATUS_INIT);
		return;
	}

	atomic_store_rel_int(&oct->status, OCTEP_DEV_STATUS_READY);
	dev_info(oct->pdev, "Device setup successful\n");
}

static int
octep_probe(device_t dev)
{
	struct octep_vendor_info *tbl;
	uint16_t vendor_id, device_id;

	vendor_id = pci_get_vendor(dev);
	if (vendor_id != PCI_VENDOR_ID_CAVIUM)
		return (ENXIO);

	device_id = pci_get_device(dev);
	tbl = octep_pci_id_tbl;
	while (tbl->vendor_id) {
		if ((vendor_id == tbl->vendor_id) && (device_id == tbl->device_id)) {
			return (BUS_PROBE_DEFAULT);
		}
		tbl++;
	}

	return (ENXIO);
}

static int
octep_iov_init(device_t dev, uint16_t num_vfs, const nvlist_t *params)
{
	struct octep_device *oct = device_get_softc(dev);

	if (num_vfs == 0)
		return (ENXIO);

	CFG_GET_ACTIVE_VFS(oct->conf) = num_vfs;
	dev_info(dev, "SR-IOV initialized with %u VFs\n", num_vfs);
	return (0);
}

static void
octep_iov_uninit(device_t dev)
{
	struct octep_device *oct = device_get_softc(dev);
	CFG_GET_ACTIVE_VFS(oct->conf) = 0;
	dev_info(dev, "SR-IOV uninitialized\n");
}

static int
octep_iov_add_vf(device_t dev, uint16_t vfnum, const nvlist_t *params)
{
	struct octep_device *oct = device_get_softc(dev);

	if (vfnum >= oct->num_vf_en)
		return (EINVAL);

	dev_info(dev, "Added VF %u\n", vfnum);
	return (0);
}

static int
octep_attach(device_t dev)
{
	struct octep_device *oct;
	int err;

	oct = malloc(sizeof(*oct), M_DEVBUF, M_WAITOK | M_ZERO);
	if (!oct) {
		dev_err(dev, "Failed to allocate octep_device\n");
		return ENOMEM;
	}
	device_set_softc(dev, oct);
	oct->pdev = dev;
	oct->tx_budget = 64;
	oct->rx_budget = 64;

	err = pci_enable_busmaster(dev);
	if (err) {
		dev_err(dev, "Failed to enable PCI busmaster\n");
		goto err_free_oct;
	}

	err = bus_dma_tag_create(bus_get_dma_tag(dev), 1, 0, BUS_SPACE_MAXADDR,
							 BUS_SPACE_MAXADDR, NULL, NULL, BUS_SPACE_MAXSIZE,
							 0, BUS_SPACE_MAXSIZE, 0, NULL, NULL, &oct->dma_tag);
	if (err) {
		dev_err(dev, "Failed to create DMA tag\n");
		goto err_disable_busmaster;
	}

	oct->netdev = if_alloc(IFT_ETHER);
	if (!oct->netdev) {
		dev_err(dev, "Failed to allocate ifnet\n");
		err = ENOMEM;
		goto err_destroy_dma;
	}

	mutex_init(&oct->lock, "octep_lock", NULL, MTX_DEF);
	atomic_store_rel_int(&oct->status, OCTEP_DEV_STATUS_ALLOC);
	oct->stats_last_update = 0; /* Initialize stats timestamp */
	TASK_INIT(&oct->dev_setup_task, 0, octep_dev_setup_task, oct);
	dev_info(dev, "Device setup task queued\n");
	err = taskqueue_enqueue(octep_tq, &oct->dev_setup_task);
	if (err != 0) {
		dev_err(dev, "Failed to enqueue setup task: %d, running directly\n", err);
		mutex_lock(&oct->lock);
		octep_dev_setup_task(oct, 0); // Fallback: run directly
		mutex_unlock(&oct->lock);
	}

	// Wait for taskqueue to complete initialization
	dev_dbg(dev, "Wait for taskqueue to complete initialization\n");
	while (atomic_load_acq_int(&oct->status) != OCTEP_DEV_STATUS_READY) {
		if (atomic_load_acq_int(&oct->status) == OCTEP_DEV_STATUS_UNINIT) {
			dev_err(dev, "Device detached during setup\n");
			err = ENXIO; // Device not found
			goto err_destroy_ifnet;
		}

		pause("octwait", hz / 1000);

		if (atomic_load_acq_int(&oct->status) == OCTEP_DEV_STATUS_WAIT_FOR_FW ||
			atomic_load_acq_int(&oct->status) == OCTEP_DEV_STATUS_ALLOC) {
			dev_err(dev, "Setup task failed or timed out\n");
			return 0;
		}
	}

	ether_ifattach(oct->netdev,  oct->mac_addr);

	dev_info(dev, "OCTEP device attached successfully\n");
	return 0;

err_destroy_ifnet:
	if (oct->netdev) {
		if_free(oct->netdev);
		oct->netdev = NULL;
	}
err_destroy_dma:
	bus_dma_tag_destroy(oct->dma_tag);
err_disable_busmaster:
	pci_disable_busmaster(dev);
err_free_oct:
	device_set_softc(dev, NULL);
	free(oct, M_DEVBUF);
	return err;
}

static int
octep_detach(device_t dev)
{
	struct octep_device *oct = device_get_softc(dev);
	int status;

	if (!oct) {
		dev_info(dev, "No softc, nothing to detach\n");
		return 0;
	}

	dev_info(dev, "Removing device\n");

	status = atomic_load_acq_int(&oct->status);
	if (status <= OCTEP_DEV_STATUS_ALLOC)
		goto free_resources;

	if (status == OCTEP_DEV_STATUS_WAIT_FOR_FW) {
		dev_info(dev, "Cancelling setup task in WAIT_FOR_FW state\n");
		atomic_store_rel_int(&oct->status, OCTEP_DEV_STATUS_UNINIT);

		while (taskqueue_cancel(octep_tq, &oct->dev_setup_task, NULL))
			taskqueue_drain(octep_tq, &oct->dev_setup_task);
		goto free_resources;
	}

	/* Handle running tasks based on status */
	if (status == OCTEP_DEV_STATUS_INIT || status == OCTEP_DEV_STATUS_READY) {
		dev_info(dev, "Draining setup task\n");
		while (taskqueue_cancel(octep_tq, &oct->dev_setup_task, NULL))
			taskqueue_drain(octep_tq, &oct->dev_setup_task);

		if (oct->sriov_enabled) {
			pci_iov_detach(dev);
			oct->sriov_enabled = false;
		}

		if (oct->netdev && (oct->netdev->if_flags & IFF_UP)) {
			dev_info(dev, "Interface is up, bringing it down\n");
			if_down(oct->netdev);
		}

		if (oct->netdev) {
			dev_info(dev, "Detaching Ethernet interface\n");
			ether_ifdetach(oct->netdev);
		}

		octep_device_cleanup(oct);
	}

free_resources:
	if (oct->netdev)
		if_free(oct->netdev);
	if (oct->dma_tag)
		bus_dma_tag_destroy(oct->dma_tag);

	mutex_destroy(&oct->lock);
	pci_disable_busmaster(dev);
	atomic_store_rel_int(&oct->status, OCTEP_DEV_STATUS_UNINIT);
	free(oct, M_DEVBUF);

	device_set_softc(dev, NULL);

	dev_info(dev, "OCTEP device detached\n");
	return 0;
}

/* Module Event Handler */
static int
octep_modevent (module_t mod, int event, void *arg)
{
	int err = 0;

	switch (event) {
	case MOD_LOAD:
		printf("%s: Loading %s ...\n", OCTEP_DRV_NAME, OCTEP_DRV_STRING);
		octep_tq = taskqueue_create("octep_tq", M_WAITOK,
									taskqueue_thread_enqueue, &octep_tq);
		if (!octep_tq) {
			printf("%s: Failed to create taskqueue\n", OCTEP_DRV_NAME);
			return ENOMEM;
		}
		printf("%s: Starting taskqueue threads\n", OCTEP_DRV_NAME);
		taskqueue_start_threads(&octep_tq, 1, PI_NET, "%s_tq", OCTEP_DRV_NAME);
		printf("%s: Loaded successfully!\n", OCTEP_DRV_NAME);
		break;

	case MOD_UNLOAD:
		printf("%s: Unloading ...\n", OCTEP_DRV_NAME);
		if (octep_tq) {
			printf("%s: Draining all tasks in taskqueue\n", OCTEP_DRV_NAME);
			taskqueue_drain_all(octep_tq);
			printf("%s: Freeing taskqueue\n", OCTEP_DRV_NAME);
			taskqueue_free(octep_tq);
			octep_tq = NULL;
		}
		printf("%s: Unloading complete\n", OCTEP_DRV_NAME);
		break;

	default:
		err = EOPNOTSUPP;
		break;
	}
	return err;
}

static device_method_t octep_methods[] = {
	DEVMETHOD(device_probe, octep_probe),
	DEVMETHOD(device_attach, octep_attach),
	DEVMETHOD(device_detach, octep_detach),
	DEVMETHOD(pci_iov_init, octep_iov_init),
	DEVMETHOD(pci_iov_uninit, octep_iov_uninit),
	DEVMETHOD(pci_iov_add_vf, octep_iov_add_vf),
	DEVMETHOD_END
};

static driver_t octep_driver = {
	OCTEP_DRV_NAME,
	octep_methods,
	sizeof(struct octep_device)
};

static devclass_t octep_devclass;

DRIVER_MODULE(octep, pci, octep_driver, octep_devclass, octep_modevent, NULL);
MODULE_DEPEND(octep, pci, 1, 1, 1);
MODULE_VERSION(octep, 1);
