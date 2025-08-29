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


/* Reset various index of Tx queue data structure. */
static void octep_iq_reset_indices(struct octep_iq *iq)
{
	mtx_lock(&iq->lock);
	iq->fill_cnt = 0;
	iq->host_write_index = 0;
	iq->octep_read_index = 0;
	iq->flush_index = 0;
	iq->pkts_processed = 0;
	iq->pkt_in_done = 0;
	iq->instr_pending = 0;
	mtx_unlock(&iq->lock);
}

static void
octep_dma_map_addr (void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	bus_addr_t *addr = (bus_addr_t *)arg;
	if (error || nseg != 1)
		return;
	*addr = segs[0].ds_addr;
}

static int
octep_setup_iq(struct octep_device *oct, int q_no)
{
	u32 desc_ring_size, buff_info_size, sglist_size;
	struct octep_iq *iq;
	int i, error = 0; /* Initialize error to 0 */

	iq = malloc(sizeof(*iq), M_DEVBUF, M_ZERO | M_WAITOK);
	if (!iq) {
		dev_err(oct->pdev, "Failed to allocate IQ-%d\n", q_no);
		return ENOMEM;
	}
	oct->iq[q_no] = iq;

	iq->oct_dev = oct;
	iq->ifp = oct->netdev;
	iq->dev = oct->pdev;
	iq->q_no = q_no;
	iq->max_count = CFG_GET_IQ_NUM_DESC(oct->conf);
	iq->ring_size_mask = iq->max_count - 1;
	iq->fill_threshold = CFG_GET_IQ_DB_MIN(oct->conf);

	mtx_init(&iq->lock, "octep_iq_lock", NULL, MTX_DEF);
	mtx_init(&iq->enq_lock, "octep_iq_enq_lock", NULL, MTX_DEF);
	mtx_init(&iq->post_lock, "octep_iq_post_lock", NULL, MTX_DEF);
	mtx_init(&iq->iq_flush_running_lock, "octep_iq_flush_running_lock", NULL, MTX_DEF);

	desc_ring_size = OCTEP_IQ_DESC_SIZE * CFG_GET_IQ_NUM_DESC(oct->conf);
	error = bus_dma_tag_create(
		bus_get_dma_tag(oct->pdev), 8, 0, BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
		NULL, NULL, desc_ring_size, 1, desc_ring_size, BUS_DMA_COHERENT, NULL, NULL, &iq->desc_dma_tag);
	if (error) {
		dev_err(iq->dev, "Failed to create DMA tag for IQ-%d\n", q_no);
		goto desc_dma_alloc_err;
	}

	error = bus_dmamem_alloc(iq->desc_dma_tag, (void **)&iq->desc_ring,
							 BUS_DMA_NOWAIT | BUS_DMA_ZERO, &iq->desc_dmamap);
	if (error || !iq->desc_ring) {
		dev_err(iq->dev, "Failed to allocate DMA memory for IQ-%d\n", q_no);
		error = error ? error : ENOMEM;
		goto desc_dma_alloc_err;
	}

	error = bus_dmamap_load(iq->desc_dma_tag, iq->desc_dmamap, iq->desc_ring,
							desc_ring_size, &octep_dma_map_addr, &iq->desc_ring_dma, 0);
	if (error) {
		dev_err(iq->dev, "Failed to map DMA memory for IQ-%d\n", q_no);
		goto desc_dma_map_err;
	}

	sglist_size = OCTEP_SGLIST_SIZE_PER_PKT * CFG_GET_IQ_NUM_DESC(oct->conf);
	error = bus_dma_tag_create(
		bus_get_dma_tag(oct->pdev), 64, 0, BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
		NULL, NULL, sglist_size, 1, sglist_size, BUS_DMA_COHERENT, NULL, NULL, &iq->sglist_dma_tag);
	if (error) {
		dev_err(iq->dev, "Failed to create SGLIST DMA tag for IQ-%d\n", q_no);
		goto sglist_dma_alloc_err;
	}

	error = bus_dmamem_alloc(iq->sglist_dma_tag, (void **)&iq->sglist,
							 BUS_DMA_NOWAIT | BUS_DMA_ZERO, &iq->sglist_dmamap);
	if (error || !iq->sglist) {
		dev_err(iq->dev, "Failed to allocate SGLIST DMA memory for IQ-%d\n", q_no);
		error = error ? error : ENOMEM;
		goto sglist_dma_alloc_err;
	}

	error = bus_dmamap_load(iq->sglist_dma_tag, iq->sglist_dmamap, iq->sglist,
							sglist_size, &octep_dma_map_addr, &iq->sglist_dma, 0);
	if (error) {
		dev_err(iq->dev, "Failed to map SGLIST DMA memory for IQ-%d\n", q_no);
		goto sglist_dma_map_err;
	}

	error = bus_dma_tag_create(
		bus_get_dma_tag(oct->pdev), 1, 0, BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
		NULL, NULL, MAXPHYS, OCTEP_SGLIST_ENTRIES_PER_PKT, MAXPHYS, 0, NULL, NULL, &iq->tx_dma_tag);
	if (error) {
		dev_err(iq->dev, "Failed to create TX DMA tag for IQ-%d\n", q_no);
		goto sglist_dma_map_err;
	}

	buff_info_size = OCTEP_IQ_TXBUFF_INFO_SIZE * iq->max_count;
	iq->buff_info = malloc(buff_info_size, M_DEVBUF, M_ZERO | M_WAITOK);
	if (!iq->buff_info) {
		dev_err(iq->dev, "Failed to allocate buff info for IQ-%d\n", q_no);
		error = ENOMEM;
		goto buff_info_err;
	}

	for (i = 0; i < CFG_GET_IQ_NUM_DESC(oct->conf); i++) {
		struct octep_tx_buffer *tx_buffer;
		tx_buffer = &iq->buff_info[i];
		tx_buffer->sglist = &iq->sglist[i * OCTEP_SGLIST_ENTRIES_PER_PKT];
		tx_buffer->sglist_dma = iq->sglist_dma + (i * OCTEP_SGLIST_SIZE_PER_PKT);
		error = bus_dmamap_create(iq->tx_dma_tag, 0, &tx_buffer->map);
		if (error) {
			dev_err(iq->dev, "Failed to create DMA map for tx buffer %d\n", i);
			while (--i >= 0)
				bus_dmamap_destroy(iq->tx_dma_tag, iq->buff_info[i].map);
			goto buff_info_err;
		}
	}


	iq->br = buf_ring_alloc(OCTEP_BR_SIZE, M_DEVBUF, M_WAITOK, &iq->enq_lock);
	if (!iq->br) {
		dev_err(iq->dev, "Failed to allocate buffer ring for IQ-%d\n", q_no);
		error = ENOMEM;
		goto buff_info_err;
	}

	octep_iq_reset_indices(iq);
	oct->hw_ops.setup_iq_regs(oct, q_no);

	oct->num_iqs++;
	return 0;

buff_info_err:
	bus_dma_tag_destroy(iq->tx_dma_tag);
	free(iq->buff_info, M_DEVBUF);
sglist_dma_map_err:
	bus_dmamem_free(iq->sglist_dma_tag, iq->sglist, iq->sglist_dmamap);
sglist_dma_alloc_err:
	bus_dma_tag_destroy(iq->sglist_dma_tag);
desc_dma_map_err:
	bus_dmamem_free(iq->desc_dma_tag, iq->desc_ring, iq->desc_dmamap);
desc_dma_alloc_err:
	bus_dma_tag_destroy(iq->desc_dma_tag);
	mtx_destroy(&iq->post_lock);
	mtx_destroy(&iq->iq_flush_running_lock);
	mtx_destroy(&iq->enq_lock);
	mtx_destroy(&iq->lock);
	free(iq, M_DEVBUF);
	oct->iq[q_no] = NULL;
	return error;
}

static void
octep_free_iq(struct octep_iq *iq)
{
	struct octep_device *oct = iq->oct_dev;
	int q_no = iq->q_no;

	if (iq->br != NULL) {
		buf_ring_free(iq->br, M_DEVBUF);
		iq->br = NULL;
	}

	if (iq->buff_info) {
		int i;
		for (i = 0; i < CFG_GET_IQ_NUM_DESC(oct->conf); i++)
			if (iq->buff_info[i].map)
				bus_dmamap_destroy(iq->tx_dma_tag, iq->buff_info[i].map);
		free(iq->buff_info, M_DEVBUF);
		iq->buff_info = NULL;
	}

	if (iq->desc_ring) {
		bus_dmamap_unload(iq->desc_dma_tag, iq->desc_dmamap);
		bus_dmamem_free(iq->desc_dma_tag, iq->desc_ring, iq->desc_dmamap);
		bus_dma_tag_destroy(iq->desc_dma_tag);
		iq->desc_ring = NULL;
		iq->desc_dma_tag = NULL;
	}

	if (iq->sglist) {
		bus_dmamap_unload(iq->sglist_dma_tag, iq->sglist_dmamap);
		bus_dmamem_free(iq->sglist_dma_tag, iq->sglist, iq->sglist_dmamap);
		bus_dma_tag_destroy(iq->sglist_dma_tag);
		iq->sglist = NULL;
		iq->sglist_dma_tag = NULL;
	}

	if (iq->tx_dma_tag) {
		bus_dma_tag_destroy(iq->tx_dma_tag);
		iq->tx_dma_tag = NULL;
	}

	mtx_destroy(&iq->lock);
	mtx_destroy(&iq->enq_lock);
	mtx_destroy(&iq->post_lock);
	mtx_destroy(&iq->iq_flush_running_lock);
	free(iq, M_DEVBUF);
	oct->iq[q_no] = NULL;
	oct->num_iqs--;
}

int octep_setup_iqs(struct octep_device *oct)
{
	int i, error;

	oct->num_iqs = 0;
	for (i = 0; i < CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf); i++) {

		error = octep_setup_iq(oct, i);
		if (error) {
			dev_err(oct->pdev, "Failed to setup IQ(TxQ)-%d: error %d\n", i, error);
			goto iq_setup_err;
		}
		dev_dbg(oct->pdev, "Successfully setup IQ(TxQ)-%d\n", i);
	}

	return 0;

iq_setup_err:
	while (i--) {
		octep_free_iq(oct->iq[i]);
	}
	return ENOMEM;
}

/**
 * octep_free_iqs() - Free resources of all Tx queues.
 *
 * @oct: Octeon device private data structure.
 */
void octep_free_iqs(struct octep_device *oct)
{
	int i;

	for (i = 0; i < CFG_GET_PORTS_ACTIVE_IO_RINGS(oct->conf); i++) {
		if (!oct->iq[i])
			continue;
		octep_free_iq(oct->iq[i]);
		dev_dbg(oct->pdev,
				 "Successfully destroyed IQ(TxQ)-%d.\n", i);
	}
	oct->num_iqs = 0;
}

