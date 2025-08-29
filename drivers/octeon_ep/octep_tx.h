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


#ifndef _OCTEP_TX_H_
#define _OCTEP_TX_H_

#include "octep_ctrl_mbox.h"


#define OCTEP_BR_SIZE 4096
/* Tx offload flags */
#define OCTEP_TX_OFFLOAD_VLAN_INSERT    BIT(0)
#define OCTEP_TX_OFFLOAD_IPV4_CKSUM BIT(1)
#define OCTEP_TX_OFFLOAD_UDP_CKSUM  BIT(2)
#define OCTEP_TX_OFFLOAD_TCP_CKSUM  BIT(3)
#define OCTEP_TX_OFFLOAD_SCTP_CKSUM BIT(4)
#define OCTEP_TX_OFFLOAD_TCP_TSO    BIT(5)
#define OCTEP_TX_OFFLOAD_UDP_TSO    BIT(6)

#define OCTEP_TX_OFFLOAD_CKSUM      (OCTEP_TX_OFFLOAD_IPV4_CKSUM | \
									 OCTEP_TX_OFFLOAD_UDP_CKSUM | \
									 OCTEP_TX_OFFLOAD_TCP_CKSUM)

#define OCTEP_TX_OFFLOAD_TSO        (OCTEP_TX_OFFLOAD_TCP_TSO | \
									 OCTEP_TX_OFFLOAD_UDP_TSO)

#define OCTEP_TX_IP_CSUM(flags)     ((flags) & \
									 (OCTEP_TX_OFFLOAD_IPV4_CKSUM | \
									  OCTEP_TX_OFFLOAD_TCP_CKSUM | \
									  OCTEP_TX_OFFLOAD_UDP_CKSUM))

#define OCTEP_TX_TSO(flags)     ((flags) & \
								 (OCTEP_TX_OFFLOAD_TCP_TSO | \
								  OCTEP_TX_OFFLOAD_UDP_TSO))

#define MAX_SKB_FRAGS 17

/* Hardware format for Scatter/Gather list */
struct octep_tx_sglist_desc {
	u16 len[4];
	bus_addr_t dma_ptr[4];
} __packed;

/* Each Scatter/Gather entry sent to hardwar hold four pointers.
 * So, number of entries required is (MAX_SKB_FRAGS + 1)/4, where '+1'
 * is for main skb which also goes as a gather buffer to Octeon hardware.
 * To allocate sufficient SGLIST entries for a packet with max fragments,
 * align by adding 3 before calcuating max SGLIST entries per packet.
 */
#define OCTEP_SGLIST_ENTRIES_PER_PKT ((MAX_SKB_FRAGS + 1 + 3) / 4)
#define OCTEP_SGLIST_SIZE_PER_PKT \
	(OCTEP_SGLIST_ENTRIES_PER_PKT * sizeof(struct octep_tx_sglist_desc))


struct octep_tx_buffer {
	struct mbuf *mb; /* Replace sk_buff with mbuf */
	bus_addr_t dma;
	struct octep_tx_sglist_desc *sglist;
	bus_addr_t sglist_dma;
	u8 gather;
	bus_dmamap_t map;
};

#define OCTEP_IQ_TXBUFF_INFO_SIZE (sizeof(struct octep_tx_buffer))

struct tx_mdata {

	/* offload flags */
	u16 ol_flags;

	/* gso size */
	u16 gso_size;

	/* gso flags */
	u16 gso_segs;

	/* reserved */
	u16 rsvd1;

	/* reserved */
	u64 rsvd2;
} __packed;

/* Hardware Tx Instruction Header */
struct octep_instr_hdr {
	/* Data Len */
	u64 tlen:16;

	/* Reserved */
	u64 rsvd:20;

	/* PKIND for SDP */
	u64 pkind:6;

	/* Front Data size */
	u64 fsz:6;

	/* No. of entries in gather list */
	u64 gsz:14;

	/* Gather indicator 1=gather*/
	u64 gather:1;

	/* Reserved3 */
	u64 reserved3:1;
} __packed;

struct octep_tx_desc_hw {
	bus_addr_t dptr;
	union {
		struct octep_instr_hdr ih; /* Assume this is defined elsewhere */
		u64 ih64;
	};
	union {
		u64 txm64[2];
		struct tx_mdata txm;
	};
	u64 exthdr[4];
} __packed;

#define OCTEP_IQ_DESC_SIZE (sizeof(struct octep_tx_desc_hw))

/* Hardware interface Tx statistics */
struct octep_iface_tx_stats {
	/* Total frames sent on the interface */
	u64 pkts;

	/* Total octets sent on the interface */
	u64 octs;

	/* Packets sent to a broadcast DMAC */
	u64 bcst;

	/* Packets sent to the multicast DMAC */
	u64 mcst;

	/* Packets dropped due to excessive collisions */
	u64 xscol;

	/* Packets dropped due to excessive deferral */
	u64 xsdef;

	/* Packets sent that experienced multiple collisions before successful
	 * transmission
	 */
	u64 mcol;

	/* Packets sent that experienced a single collision before successful
	 * transmission
	 */
	u64 scol;

	/* Packets sent with an octet count < 64 */
	u64 hist_lt64;

	/* Packets sent with an octet count == 64 */
	u64 hist_eq64;

	/* Packets sent with an octet count of 65▒~@~S127 */
	u64 hist_65to127;

	/* Packets sent with an octet count of 128▒~@~S255 */
	u64 hist_128to255;

	/* Packets sent with an octet count of 256▒~@~S511 */
	u64 hist_256to511;

	/* Packets sent with an octet count of 512▒~@~S1023 */
	u64 hist_512to1023;

	/* Packets sent with an octet count of 1024-1518 */
	u64 hist_1024to1518;

	/* Packets sent with an octet count of > 1518 */
	u64 hist_gt1518;

	/* Packets sent that experienced a transmit underflow and were
	 * truncated
	 */
	u64 undflw;

	/* Control/PAUSE packets sent */
	u64 ctl;
};

/* Input Queue statistics. Each input queue has four stats fields. */
struct octep_iq_stats {
	/* Instructions posted to this queue. */
	u64 instr_posted;

	/* Instructions copied by hardware for processing. */
	u64 instr_completed;

	/* Instructions that could not be processed. */
	u64 instr_dropped;

	/* Bytes sent through this queue. */
	u64 bytes_sent;

	/* Gather entries sent through this queue. */
	u64 sgentry_sent;

	/* Number of transmit failures due to TX_BUSY */
	u64 tx_busy;

	/* Number of times the queue is restarted */
	u64 restart_cnt;

	u64 instr_processed;
};

struct octep_iq {
	uint32_t q_no;
	struct octep_device    *oct_dev;
	struct ifnet *ifp;
	device_t dev;
	/* Index in input ring where driver should write the next packet */
	uint16_t host_write_index;
	/* Index in input ring where Octeon is expected to read next packet */
	uint16_t octep_read_index;

	/* This index aids in finding the window in the queue where Octeon
	 * has read the commands.
	 */
	uint16_t flush_index;

	/* Statistics for this input queue. */
	struct octep_iq_stats stats;

	/* DMA mapped base address of the input descriptor ring. */
	uint64_t               desc_ring_dma;
	struct octep_tx_desc_hw *desc_ring;

	/* Octeon doorbell register for the ring. */
	bus_size_t doorbell_reg;

	/* Octeon instruction count register for this ring. */
	bus_size_t inst_cnt_reg;


	/* interrupt level register for this ring */
	bus_size_t intr_lvl_reg;

	/* Maximum no. of instructions in this queue. */
	uint32_t max_count;
	uint32_t ring_size_mask;

	uint32_t pkt_in_done;
	uint32_t pkts_processed;

	volatile int        instr_pending; /* Added for pending instructions */
	struct mtx lock;       /* Added for general locking */
	struct mtx post_lock;  /* Command posting lock */
	struct mtx iq_flush_running_lock; /* Flush lock */

	uint32_t status;

	/* Number of instructions pending to be posted to Octeon. */
	uint32_t fill_cnt;

	/* The max. number of instructions that can be held pending by the
	 * driver before ringing doorbell.
	 */
	uint32_t fill_threshold;

	bus_dma_tag_t desc_dma_tag; /* DMA tags for memory management */
	bus_dmamap_t desc_dmamap;
	bus_dma_tag_t sglist_dma_tag;
	bus_dmamap_t sglist_dmamap;
	struct octep_tx_sglist_desc *sglist;
	bus_addr_t sglist_dma;
	struct octep_tx_buffer *buff_info;
	struct buf_ring *br;              /* Buffer ring for packet queuing */
	struct mtx enq_lock;              /* Mutex for buf_ring enqueue/dequeue */
	bus_dma_tag_t tx_dma_tag;
};


#endif
