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

#ifndef _OCTEP_RX_H_
#define _OCTEP_RX_H_

struct octep_oq_resp_hw_ext {
	/* Reserved. */
	u64 rsvd:48;

	/* offload flags */
	u16 rx_ol_flags;
} __packed;

#define  OCTEP_OQ_RESP_HW_EXT_SIZE   (sizeof(struct octep_oq_resp_hw_ext))

/* Length of Rx packet DMA'ed by Octeon to Host.
 * this is in bigendian; so need to be converted to cpu endian.
 * Octeon writes this at the beginning of Rx buffer (skb->data).
 */

struct octep_oq_resp_hw {
	uint64_t length; /* Packet length, 64-bit */
} __packed;

#define OCTEP_OQ_RESP_HW_SIZE   (sizeof(struct octep_oq_resp_hw))

/* Descriptor structure */
struct octep_oq_desc_hw {
	bus_addr_t buffer_ptr;
	uint64_t info_ptr;
} __packed;

#define OCTEP_OQ_DESC_SIZE    (sizeof(struct octep_oq_desc_hw))

/* Receive buffer structure */
struct octep_rx_buffer {
	struct mbuf *buffer;           /* Pointer to mbuf */
	void *data;          /* Pointer to packet data */
	uint32_t len;
	bus_dmamap_t dma_map;   /* DMA map for buffer */
};

#define OCTEP_OQ_RECVBUF_SIZE    (sizeof(struct octep_rx_buffer))

/* Rx offload flags */
#define OCTEP_RX_OFFLOAD_VLAN_STRIP BIT(0)
#define OCTEP_RX_OFFLOAD_IPV4_CKSUM BIT(1)
#define OCTEP_RX_OFFLOAD_UDP_CKSUM  BIT(2)
#define OCTEP_RX_OFFLOAD_TCP_CKSUM  BIT(3)

#define OCTEP_RX_OFFLOAD_CKSUM      (OCTEP_RX_OFFLOAD_IPV4_CKSUM | \
									 OCTEP_RX_OFFLOAD_UDP_CKSUM | \
									 OCTEP_RX_OFFLOAD_TCP_CKSUM)

#define OCTEP_RX_IP_CSUM(flags)     ((flags) & \
									 (OCTEP_RX_OFFLOAD_IPV4_CKSUM | \
									  OCTEP_RX_OFFLOAD_TCP_CKSUM | \
									  OCTEP_RX_OFFLOAD_UDP_CKSUM))

/* bit 0 is vlan strip */
#define OCTEP_RX_CSUM_IP_VERIFIED   BIT(1)
#define OCTEP_RX_CSUM_L4_VERIFIED   BIT(2)

#define OCTEP_RX_CSUM_VERIFIED(flags)   ((flags) & \
										 (OCTEP_RX_CSUM_L4_VERIFIED | \
										  OCTEP_RX_CSUM_IP_VERIFIED))

/* Hardware interface Rx statistics */
struct octep_iface_rx_stats {
	/* Received packets */
	u64 pkts;

	/* Octets of received packets */
	u64 octets;

	/* Received PAUSE and Control packets */
	u64 pause_pkts;

	/* Received PAUSE and Control octets */
	u64 pause_octets;

	/* Filtered DMAC0 packets */
	u64 dmac0_pkts;

	/* Filtered DMAC0 octets */
	u64 dmac0_octets;

	/* Packets dropped due to RX FIFO full */
	u64 dropped_pkts_fifo_full;

	/* Octets dropped due to RX FIFO full */
	u64 dropped_octets_fifo_full;

	/* Error packets */
	u64 err_pkts;

	/* Filtered DMAC1 packets */
	u64 dmac1_pkts;

	/* Filtered DMAC1 octets */
	u64 dmac1_octets;

	/* NCSI-bound packets dropped */
	u64 ncsi_dropped_pkts;

	/* NCSI-bound octets dropped */
	u64 ncsi_dropped_octets;
	/* Multicast packets received. */
	u64 mcast_pkts;

	/* Broadcast packets received. */
	u64 bcast_pkts;

};


/* Output Queue statistics. Each output queue has four stats fields. */
struct octep_oq_stats {
	/* Number of packets received from the Device. */
	u64 packets;

	/* Number of bytes received from the Device. */
	u64 bytes;

	/* Number of times failed to allocate buffers. */
	u64 alloc_failures;

	/* Number of packets for which data arrived late. */
	u64 pkts_delayed_data;

	uint64_t rx_dma_mapping_err;

	u64 zero_length_packets;

	u64 incomplete_packets;

	u64 unexpected_packets;

};

/* The Descriptor Ring Output Queue structure.
 * This structure has all the information required to implement a
 * Octeon OQ.
 */
struct octep_oq {
	struct octep_device *oct_dev;

	device_t pdev;

	if_t ifp;

	/* A lock to protect access to this ring. */
	struct mtx              lock;

	uint32_t                q_no;

	struct octep_rx_buffer *buff_info;
	/* The 8B aligned descriptor ring starts at this address. */
	struct octep_oq_desc_hw   *desc_ring;

	bus_dma_tag_t dma_tag;
	bus_dmamap_t dma_map;

	bus_dma_tag_t rx_buf_tag;
	/*
	 * Index in the ring where the driver will refill the descriptor's
	 * buffer
	 */
	uint32_t                refill_idx;

	/* Index in the ring where the driver should read the next packet */
	uint32_t host_read_idx;

	/* Packets pending to be processed */
	uint32_t pkts_pending;
	uint32_t last_pkt_count;
	/* Number of  descriptors in this ring. */
	uint32_t                max_count;
	uint32_t ring_size_mask;

	/* The number of descriptors pending refill. */
	uint32_t                refill_count;

	uint32_t host_refill_idx;

	uint32_t                refill_threshold;

	/* The size of each buffer pointed by the buffer pointer. */
	uint32_t                buffer_size;
	uint32_t max_single_buffer_size;

	/*
	 * Offset to packet credit register.
	 * Host writes number of info/buffer ptrs available to this register
	 */
	bus_size_t              pkts_credit_reg;

	/*
	 * Offset packet sent register.
	 * Octeon writes the number of packets DMA'ed to host memory
	 * in this register.
	 */
	bus_size_t              pkts_sent_reg;

	bus_addr_t              desc_ring_dma;

	struct lro_ctrl lro; 
	bool suspend;
	bool lro_enabled;

	struct octep_oq_stats stats;
};

#endif
