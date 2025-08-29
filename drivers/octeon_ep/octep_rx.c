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

static void
octep_oq_reset_indices(struct octep_oq *oq)
{
	mtx_lock(&oq->lock);
	oq->host_read_idx = 0;
	oq->host_refill_idx = 0;
	oq->refill_count = 0;
	atomic_store_int(&oq->pkts_pending, 0);
	oq->last_pkt_count = 0;
	mtx_unlock(&oq->lock);
}

static void
octep_dma_map_addr(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	bus_addr_t *addr = (bus_addr_t *)arg;
	if (error || nseg != 1)
		return;
	*addr = segs[0].ds_addr;
}

static int
octep_map_ring(struct octep_oq *oq, struct mbuf *mb, bus_dmamap_t map, uint64_t *dma_addr)
{
	int error;

	if (map == NULL) {
		oq->stats.rx_dma_mapping_err++;
		dev_err(oq->pdev, "Null DMA map in octep_map_ring for OQ-%d\n", oq->q_no);
		return EINVAL;
	}
	if (mb == NULL || mb->m_data == NULL) {
		oq->stats.alloc_failures++;
		dev_err(oq->pdev, "Invalid mbuf in octep_map_ring for OQ-%d\n", oq->q_no);
		return EINVAL;
	}

	error = bus_dmamap_load(oq->rx_buf_tag, map, mb->m_data, mb->m_len,
							octep_dma_map_addr, dma_addr, 0);
	if (error) {
		oq->stats.rx_dma_mapping_err++;
		dev_err(oq->pdev, "DMA mapping failed for OQ-%d: %d\n", oq->q_no, error);
		return error;
	}
	bus_dmamap_sync(oq->rx_buf_tag, map, BUS_DMASYNC_PREREAD);
	return 0;
}

int
octep_oq_check_hw_for_pkts(struct octep_device *oct, struct octep_oq *oq)
{
	uint32_t pkt_count, new_pkts;
	uint32_t last_pkt_count, pkts_pending;

	pkt_count = octep_read_csr(oct, oq->pkts_sent_reg);
	if (pkt_count == 0xFFFFFFFF) {
		octep_write_csr(oct, oq->pkts_sent_reg, pkt_count);
		pkt_count = 0;
		return 0;
	}

	last_pkt_count = atomic_load_acq_int(&oq->last_pkt_count);
	new_pkts = pkt_count - last_pkt_count;

	if (pkt_count < last_pkt_count) {
		new_pkts = pkt_count + (0xFFFFFFFFU - last_pkt_count) + 1;
	}

	if (pkt_count > 0xF0000000U) {
		octep_write_csr(oct, oq->pkts_sent_reg, pkt_count);
		pkt_count = octep_read_csr(oct, oq->pkts_sent_reg);
		if (pkt_count == 0xFFFFFFFF) {
			pkt_count = 0;
		}
		new_pkts += pkt_count;
	}

	atomic_store_rel_int(&oq->last_pkt_count, pkt_count);
	pkts_pending = atomic_load_acq_int(&oq->pkts_pending);
	atomic_store_rel_int(&oq->pkts_pending, pkts_pending + new_pkts);

	return new_pkts;
}

static inline uint32_t
octep_incr_index(uint32_t index, uint32_t count, uint32_t max)
{
	if ((index + count) >= max)
		index = index + count - max;
	else
		index += count;

	return (index);
}


static int
octep_oq_refill_alloc_descs(struct octep_oq *oq)
{
	struct octep_oq_desc_hw *desc_ring = oq->desc_ring;
	struct octep_rx_buffer *buf;
	struct mbuf *mb;
	uint64_t dma_addr;
	uint32_t refill_idx = oq->host_refill_idx;
	uint32_t desc_refilled = 0;
	int error = 0;

	while (oq->refill_count && (desc_refilled < oq->max_count)) {
		buf = &oq->buff_info[refill_idx];

		if (buf->buffer == NULL) {
			mb = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR, oq->buffer_size);
			if (mb == NULL) {
				oq->stats.alloc_failures++;
				dev_err(oq->pdev, "%s == mb == NULL\n", __func__);
				break;
			}

			mb->m_pkthdr.len = mb->m_len = oq->buffer_size;
			buf->buffer = mb;
			buf->data = mb->m_data;

			error = octep_map_ring(oq, mb, buf->dma_map, &dma_addr);
			if (error) {
				m_free(mb);
				buf->buffer = NULL;
				buf->data = NULL;
				dev_err(oq->pdev, "%s == DMA map error\n", __func__);
				break;
			}

			desc_ring[refill_idx].buffer_ptr = dma_addr;
			desc_ring[refill_idx].info_ptr = 0;
			bus_dmamap_sync(oq->rx_buf_tag, buf->dma_map, BUS_DMASYNC_PREREAD);

			desc_refilled++;
			oq->refill_count--;
		}

		refill_idx = octep_incr_index(refill_idx, 1, oq->max_count);
	}

	oq->host_refill_idx = refill_idx;
	return desc_refilled;
}

static int
octep_oq_refill(struct octep_oq *oq)
{
	int total_refilled = 0;

	total_refilled += octep_oq_refill_alloc_descs(oq);

	return total_refilled;
}

static void
octep_oq_free_ring_buffers(struct octep_oq *oq)
{
	struct octep_oq_desc_hw *desc_ring = oq->desc_ring;
	int i;

	mtx_lock(&oq->lock);
	if (!oq->desc_ring || !oq->buff_info) {
		mtx_unlock(&oq->lock);
		return;
	}

	for (i = 0; i < oq->max_count; i++) {
		if (oq->buff_info[i].buffer) {
			bus_dmamap_unload(oq->rx_buf_tag, oq->buff_info[i].dma_map);
			m_freem(oq->buff_info[i].buffer);
			oq->buff_info[i].buffer = NULL;
			oq->buff_info[i].data = NULL;
			oq->buff_info[i].len = 0;
			desc_ring[i].buffer_ptr = 0;
		}
	}

	octep_oq_reset_indices(oq);
	mtx_unlock(&oq->lock);
}

static int
octep_free_oq(struct octep_oq *oq)
{
	struct octep_device *oct = oq->oct_dev;
	int q_no = oq->q_no;
	int i;

	mtx_lock(&oq->lock);
	octep_oq_free_ring_buffers(oq);
	mtx_unlock(&oq->lock);

	if (oq->buff_info) {
		for (i = 0; i < oq->max_count; i++)
			bus_dmamap_destroy(oq->rx_buf_tag, oq->buff_info[i].dma_map);
		free(oq->buff_info, M_DEVBUF);
	}

	if (oq->desc_ring) {
		bus_dmamap_unload(oq->dma_tag, oq->dma_map);
		bus_dmamem_free(oq->dma_tag, oq->desc_ring, oq->dma_map);
		bus_dma_tag_destroy(oq->dma_tag);
		oq->desc_ring = NULL;
	}

	if (oq->rx_buf_tag) {
		bus_dma_tag_destroy(oq->rx_buf_tag);
		oq->rx_buf_tag = NULL;
	}

	if (oq->lro_enabled) {
		tcp_lro_free(&oq->lro);
		oq->lro_enabled = false;
	}

	mtx_destroy(&oq->lock);
	free(oq, M_DEVBUF);
	oct->oq[q_no] = NULL;
	oct->num_oqs--;
	return 0;
}

void octep_oq_dbell_init(struct octep_device *oct)
{
	int i;

	for (i = 0; i < oct->num_oqs; i++)
		octep_write_csr(oct, oct->oq[i]->pkts_credit_reg,
						oct->oq[i]->max_count);
}

void
octep_free_oqs(struct octep_device *oct)
{
	int i;

	for (i = 0; i < CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf); i++) {
		if (!oct->oq[i])
			continue;
		octep_free_oq(oct->oq[i]);
	}
	oct->num_oqs = 0;
}

static int
octep_oq_fill_ring_buffers(struct octep_oq *oq)
{
	struct mbuf *mb;
	uint64_t dma_addr;
	uint32_t i;
	int error = 0;
	struct octep_rx_buffer *buf;

	for (i = 0; i < oq->max_count; i++) {
		buf = &oq->buff_info[i];

		if (buf->buffer != NULL || buf->dma_map == NULL) {
			if (buf->buffer != NULL) {
				device_printf(oq->pdev, "octep: Buffer already exists at index %d\n", i);
			}
			if (buf->dma_map == NULL) {
				dev_err(oq->pdev,"octep: Missing DMA map for buffer %d\n", i);
			}
			continue;
		}

		mb = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR, oq->buffer_size);
		if (mb == NULL) {
			oq->stats.alloc_failures++;
			error = ENOMEM;
			break;
		}

		mb->m_pkthdr.len = mb->m_len = oq->buffer_size;
		buf->buffer = mb;
		buf->data = mb->m_data;

		error = octep_map_ring(oq, mb, buf->dma_map, &dma_addr);
		if (error) {
			m_free(mb);
			buf->buffer = NULL;
			buf->data = NULL;
			break;
		}

		oq->desc_ring[i].buffer_ptr = dma_addr;
		oq->desc_ring[i].info_ptr = 0;
	}

	if (error && i > 0) {
		while (i--) {
			buf = &oq->buff_info[i];
			if (buf->buffer) {
				bus_dmamap_unload(oq->dma_tag, buf->dma_map);
				m_free(buf->buffer);
				buf->buffer = NULL;
				buf->data = NULL;
				oq->desc_ring[i].buffer_ptr = 0;
			}
		}
	}

	return error;
}

static int
octep_setup_oq(struct octep_device *oct, int q_no)
{
	struct octep_oq *oq;
	bus_size_t desc_ring_size;
	bus_dma_tag_t dma_tag;
	bus_dmamap_t dma_map;
	void *desc_ring;
	int error, i;

	oq = malloc(sizeof(*oq), M_DEVBUF, M_ZERO | M_NOWAIT);
	if (!oq) {
		dev_err(oct->pdev, "Failed to allocate OQ-%d structure\n", q_no);
		goto create_oq_fail;
	}
	oct->oq[q_no] = oq;

	oq->oct_dev = oct;
	oq->pdev = oct->pdev;
	oq->ifp = oct->netdev;
	oq->q_no = q_no;
	oq->max_count = CFG_GET_OQ_NUM_DESC(oct->conf);
	oq->ring_size_mask = oq->max_count - 1;
	oq->buffer_size = CFG_GET_OQ_BUF_SIZE(oct->conf);
	oq->max_single_buffer_size = oq->buffer_size - OCTEP_OQ_RESP_HW_SIZE;

	if (oct->conf->fw_info.rx_ol_flags)
		oq->max_single_buffer_size -= OCTEP_OQ_RESP_HW_EXT_SIZE;

	oq->refill_threshold = CFG_GET_OQ_REFILL_THRESHOLD(oct->conf) / 4; /* More aggressive refill */

	mtx_init(&oq->lock, "octep_oq_lock", NULL, MTX_DEF);

	if (oct->netdev->if_capenable & IFCAP_LRO) {
		error = tcp_lro_init(&oq->lro);
		if (error) {
			dev_err(oq->pdev, "Failed to initialize LRO for OQ-%d\n", q_no);
		}
		oq->lro.ifp = oct->netdev;
		oq->lro_enabled = true;
	}

	desc_ring_size = oq->max_count * OCTEP_OQ_DESC_SIZE;
	error = bus_dma_tag_create(bus_get_dma_tag(oct->pdev), 8, 0, BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
							   NULL, NULL, desc_ring_size, 1, desc_ring_size,
							   BUS_DMA_COHERENT, NULL, NULL, &dma_tag);
	if (error) {
		dev_err(oq->pdev, "Failed to create DMA tag for OQ-%d\n", q_no);
		goto desc_dma_alloc_err;
	}

	error = bus_dmamem_alloc(dma_tag, &desc_ring, BUS_DMA_NOWAIT | BUS_DMA_ZERO, &dma_map);
	if (error || !desc_ring) {
		dev_err(oq->pdev, "Failed to allocate DMA memory for OQ-%d\n", q_no);
		bus_dma_tag_destroy(dma_tag);
		goto desc_dma_alloc_err;
	}

	error = bus_dmamap_load(dma_tag, dma_map, desc_ring, desc_ring_size,
							octep_dma_map_addr, &oq->desc_ring_dma, 0);
	if (error) {
		dev_err(oq->pdev, "Failed to map DMA memory for OQ-%d\n", q_no);
		bus_dmamem_free(dma_tag, desc_ring, dma_map);
		bus_dma_tag_destroy(dma_tag);
		goto desc_dma_alloc_err;
	}

	oq->desc_ring = desc_ring;
	oq->dma_tag = dma_tag;
	oq->dma_map = dma_map;

	error = bus_dma_tag_create(bus_get_dma_tag(oct->pdev), 64, 0, BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
							   NULL, NULL, oq->buffer_size, 1, oq->buffer_size,
							   BUS_DMA_COHERENT, NULL, NULL, &oq->rx_buf_tag);
	if (error) {
		dev_err(oq->pdev, "Failed to create RX buffer DMA tag for OQ-%d\n", q_no);
		goto rx_buf_tag_err;
	}

	oq->buff_info = malloc(oq->max_count * OCTEP_OQ_RECVBUF_SIZE, M_DEVBUF, M_ZERO | M_NOWAIT);
	if (!oq->buff_info) {
		dev_err(oct->pdev, "Failed to allocate buffer info for OQ-%d\n", q_no);
		goto buf_list_err;
	}

	for (i = 0; i < oq->max_count; i++) {
		error = bus_dmamap_create(oq->rx_buf_tag, 0, &oq->buff_info[i].dma_map);
		if (error) {
			dev_err(oct->pdev, "Failed to create DMA map for OQ-%d buffer %d\n", q_no, i);
			while (i) {
				i--;
				bus_dmamap_destroy(oq->rx_buf_tag, oq->buff_info[i].dma_map);
			}
			goto dma_map_err;
		}
	}

	if (octep_oq_fill_ring_buffers(oq)) {
		dev_err(oq->pdev, "Failed to fill ring buffers for OQ-%d\n", q_no);
		goto oq_fill_buff_err;
	}

	octep_oq_reset_indices(oq);

	if (oct->hw_ops.setup_oq_regs(oct, q_no)) {
		dev_err(oq->pdev, "Failed to setup OQ-%d registers\n", q_no);
		goto oq_fill_buff_err;
	}

	oct->num_oqs++;
	return 0;

oq_fill_buff_err:
	for (i = 0; i < oq->max_count; i++) {
		if (oq->buff_info[i].buffer) {
			bus_dmamap_unload(oq->rx_buf_tag, oq->buff_info[i].dma_map);
			m_freem(oq->buff_info[i].buffer);
		}
		bus_dmamap_destroy(oq->rx_buf_tag, oq->buff_info[i].dma_map);
	}
dma_map_err:
	free(oq->buff_info, M_DEVBUF);
	oq->buff_info = NULL;
buf_list_err:
	bus_dma_tag_destroy(oq->rx_buf_tag);
rx_buf_tag_err:
	bus_dmamap_unload(dma_tag, dma_map);
	bus_dmamem_free(dma_tag, oq->desc_ring, dma_map);
	bus_dma_tag_destroy(dma_tag);
desc_dma_alloc_err:
	oq->desc_ring = NULL;
	mtx_destroy(&oq->lock);
	free(oq, M_DEVBUF);
	oct->oq[q_no] = NULL;
create_oq_fail:
	return ENOMEM;
}

int
octep_setup_oqs(struct octep_device *oct)
{
	int i, retval = 0;

	oct->num_oqs = 0;
	for (i = 0; i < CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf); i++) {
		retval = octep_setup_oq(oct, i);
		if (retval) {
			dev_err(oct->pdev, "Failed to setup OQ-%d\n", i);
			goto oq_setup_err;
		}
		dev_dbg(oct->pdev, "Successfully setup OQ-%d\n", i);
	}

	return 0;

oq_setup_err:
	while (i) {
		i--;
		octep_free_oq(oct->oq[i]);
	}

	return -1;
}

int
octep_iq_process_completions(struct octep_iq *iq, uint16_t budget)
{
	struct octep_device *oct = iq->oct_dev;
	struct octep_tx_buffer *tx_buffer;
	uint32_t compl_pkts, compl_bytes, compl_sg;
	uint32_t fi;
	struct mbuf *mb;

	if (!iq->ifp || !(if_getdrvflags(iq->ifp) & IFF_DRV_RUNNING)) {
		return 0;
	}

	compl_pkts = 0;
	compl_bytes = 0;
	compl_sg = 0;
	fi = iq->flush_index;
	iq->octep_read_index = oct->hw_ops.update_iq_read_idx(iq);

	while (budget && (fi != iq->octep_read_index)) {
		tx_buffer = &iq->buff_info[fi];
		mb = tx_buffer->mb;

		fi = (fi + 1) & iq->ring_size_mask;
		compl_bytes += mb->m_pkthdr.len;
		compl_pkts++;
		budget--;

		if (!tx_buffer->gather) {
			bus_dmamap_unload(iq->desc_dma_tag, tx_buffer->map);
			m_freem(mb);
			tx_buffer->mb = NULL;
			continue;
		}

		compl_sg++;
		bus_dmamap_unload(iq->sglist_dma_tag, tx_buffer->map);
		tx_buffer->mb = NULL;
		m_freem(mb);
	}

	iq->pkts_processed += compl_pkts;
	iq->stats.instr_completed += compl_pkts;
	iq->stats.bytes_sent += compl_bytes;
	iq->stats.sgentry_sent += compl_sg;
	iq->flush_index = fi;

	return compl_pkts;
}

static int
octep_oq_process_rx_internal(struct octep_device *oct, struct octep_oq *oq, uint16_t pkts_to_process)
{
	struct octep_oq_resp_hw_ext *resp_hw_ext = NULL;
	struct octep_rx_buffer *buff_info;
	struct octep_oq_resp_hw *resp_hw;
	uint32_t pkt, rx_bytes, desc_used;
	uint16_t data_offset, rx_ol_flags;
	struct mbuf *mb, *head;
	uint32_t read_idx;
	int retry;

	bus_dmamap_sync(oq->dma_tag, oq->dma_map, BUS_DMASYNC_POSTREAD);

	read_idx = oq->host_read_idx;
	rx_bytes = 0;
	desc_used = 0;

	for (pkt = 0; pkt < pkts_to_process; pkt++) {
		buff_info = &oq->buff_info[read_idx];
		mb = buff_info->buffer;
		if (!mb) {
			oq->stats.alloc_failures++;
			break;
		}

		bus_dmamap_sync(oq->rx_buf_tag, buff_info->dma_map, BUS_DMASYNC_POSTREAD);
		resp_hw = (struct octep_oq_resp_hw *)buff_info->data;
		rmb();

		if (*((volatile uint64_t *)&resp_hw->length) == 0) {
			retry = 100;
			oq->stats.zero_length_packets++;

			while (retry-- && (*((volatile uint64_t *)&resp_hw->length) == 0)) {
				cpu_spinwait();
			}

			if (*((volatile uint64_t *)&resp_hw->length) == 0) {
				dev_err(oq->pdev, "OQ[%d]: ZERO_PKT_LEN pkt:%d SUSPENDED\n", oq->q_no, pkt);
				for (int i = 0; i < oct->num_oqs; i++) {
					oct->oq[i]->suspend = 1;
					oct->hw_ops.disable_iq(oct, i);
					oct->hw_ops.disable_oq(oct, i);
				}

				if (oq->ifp) {
					if_setdrvflagbits(oq->ifp, 0, IFF_DRV_RUNNING);
				}

				m_freem(mb);
				buff_info->buffer = NULL;
				buff_info->data = NULL;
				buff_info->len = 0;

				return pkt;
			}
		}

		buff_info->buffer = NULL;
		buff_info->data = NULL;
		buff_info->len = be64toh(resp_hw->length);

		if (oct->conf->fw_info.rx_ol_flags) {
			resp_hw_ext = (struct octep_oq_resp_hw_ext *)(resp_hw + 1);
			rx_ol_flags = resp_hw_ext->rx_ol_flags;
			buff_info->len -= OCTEP_OQ_RESP_HW_EXT_SIZE;
			data_offset = OCTEP_OQ_RESP_HW_SIZE + OCTEP_OQ_RESP_HW_EXT_SIZE;

			if (buff_info->len < 0 || buff_info->len > oq->buffer_size) {
				oq->stats.unexpected_packets++;
				m_freem(mb);
				read_idx = (read_idx + 1) & oq->ring_size_mask;
				desc_used++;
				continue;
			}
		} else {
			data_offset = OCTEP_OQ_RESP_HW_SIZE;
			rx_ol_flags = 0;
		}
		rx_bytes += buff_info->len;

		if (buff_info->len <= oq->max_single_buffer_size) {
			mb->m_data += data_offset;
			mb->m_len = buff_info->len;
			mb->m_pkthdr.len = buff_info->len;
			read_idx = (read_idx + 1) & oq->ring_size_mask;
			desc_used++;
		} else {
			head = mb;
			uint32_t data_len = buff_info->len - oq->max_single_buffer_size;

			mb->m_data += data_offset;
			mb->m_len = oq->max_single_buffer_size;
			mb->m_pkthdr.len = oq->max_single_buffer_size;
			read_idx = (read_idx + 1) & oq->ring_size_mask;
			desc_used++;

			while (data_len) {
				buff_info = &oq->buff_info[read_idx];
				mb = buff_info->buffer;
				if (!mb) {
					oq->stats.alloc_failures++;
					m_freem(head);
					break;
				}
				buff_info->buffer = NULL;
				buff_info->data = NULL;
				buff_info->len = 0;

				if (data_len < oq->buffer_size) {
					buff_info->len = data_len;
					data_len = 0;
				} else {
					buff_info->len = oq->buffer_size;
					data_len -= oq->buffer_size;
				}

				mb->m_len = buff_info->len;
				mb->m_pkthdr.len = buff_info->len;
				m_cat(head, mb);
				head->m_pkthdr.len += buff_info->len;

				read_idx = (read_idx + 1) & oq->ring_size_mask;
				desc_used++;
			}
			mb = head;
			if (data_len) {
				oq->stats.incomplete_packets++;

				continue;
			}
		}

		mb->m_pkthdr.rcvif = oq->ifp;
		mb->m_pkthdr.flowid = oq->q_no;
		ETHER_BPF_MTAP(oq->ifp, mb);
		if (oq->ifp->if_capenable & IFCAP_RXCSUM && OCTEP_RX_CSUM_VERIFIED(rx_ol_flags))
			mb->m_pkthdr.csum_flags |= CSUM_IP_CHECKED | CSUM_IP_VALID | CSUM_DATA_VALID;
		else
			mb->m_pkthdr.csum_flags = 0;

		(*oq->ifp->if_input)(oq->ifp, mb);
	}

	oq->host_read_idx = read_idx;
	oq->refill_count += desc_used;
	oq->stats.packets += pkt;
	oq->stats.bytes += rx_bytes;

	return pkt;
}



int
octep_oq_process_rx(struct octep_oq *oq, int budget)
{
	struct octep_device *oct = oq->oct_dev;
	uint32_t pkts_available, pkts_processed, total_pkts_processed = 0;
	uint32_t pkts_pending;

	if (oq->suspend) {
		return 0;
	}

	while (total_pkts_processed < budget) {
		if (oq->suspend) {
			return total_pkts_processed;
		}

		pkts_pending = atomic_load_acq_int(&oq->pkts_pending);
		if (pkts_pending == 0) {
			octep_oq_check_hw_for_pkts(oct, oq);
			pkts_pending = atomic_load_acq_int(&oq->pkts_pending);
			if (pkts_pending == 0) {
				break;
			}
		}

		pkts_available = MIN(budget - total_pkts_processed, pkts_pending);
		if (!pkts_available) {
			break;
		}

		pkts_processed = octep_oq_process_rx_internal(oct, oq, pkts_available);
		pkts_pending = atomic_load_acq_int(&oq->pkts_pending);
		atomic_store_rel_int(&oq->pkts_pending, pkts_pending - pkts_processed);
		total_pkts_processed += pkts_processed;
	}

	if (oq->refill_count >= oq->refill_threshold) {
		int desc_refilled = octep_oq_refill(oq);
		if (desc_refilled > 0) {
			wmb();
			octep_write_csr(oct, oq->pkts_credit_reg, desc_refilled);
		}
	}

	return total_pkts_processed;
}

