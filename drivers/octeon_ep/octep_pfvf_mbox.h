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
#ifndef _OCTEP_PFVF_MBOX_H_
#define _OCTEP_PFVF_MBOX_H_

/* VF flags */
#define OCTEON_PFVF_FLAG_MAC_SET_BY_PF  BIT_ULL(0) /* PF has set VF MAC address */
#define OCTEON_SDP_16K_HW_FRS  16380UL
#define OCTEON_SDP_64K_HW_FRS  65531UL

/*
 * When a new command is implemented,PF Mbox version should be bumped.
 */
enum octep_pfvf_mbox_version {
	OCTEP_PFVF_MBOX_VERSION_V0,
	OCTEP_PFVF_MBOX_VERSION_V1,
	OCTEP_PFVF_MBOX_VERSION_V2,
	OCTEP_PFVF_MBOX_VERSION_V3,
};

#define OCTEP_PFVF_MBOX_VERSION_CURRENT OCTEP_PFVF_MBOX_VERSION_V3

enum octep_pfvf_mbox_opcode {
	OCTEP_PFVF_MBOX_CMD_VERSION,
	OCTEP_PFVF_MBOX_CMD_SET_MTU,
	OCTEP_PFVF_MBOX_CMD_SET_MAC_ADDR,
	OCTEP_PFVF_MBOX_CMD_GET_MAC_ADDR,
	OCTEP_PFVF_MBOX_CMD_GET_LINK_INFO,
	OCTEP_PFVF_MBOX_CMD_GET_STATS,
	OCTEP_PFVF_MBOX_CMD_SET_RX_STATE,
	OCTEP_PFVF_MBOX_CMD_SET_LINK_STATUS,
	OCTEP_PFVF_MBOX_CMD_GET_LINK_STATUS,
	OCTEP_PFVF_MBOX_CMD_GET_MTU,
	OCTEP_PFVF_MBOX_CMD_DEV_REMOVE,
	OCTEP_PFVF_MBOX_CMD_GET_FW_INFO,
	OCTEP_PFVF_MBOX_CMD_SET_OFFLOADS,
	OCTEP_PFVF_MBOX_NOTIF_LINK_STATUS,
	OCTEP_PFVF_MBOX_NOTIF_PF_FLR,
	OCTEP_PFVF_MBOX_CMD_MAX,
};

enum octep_pfvf_mbox_word_type {
	OCTEP_PFVF_MBOX_TYPE_CMD,
	OCTEP_PFVF_MBOX_TYPE_RSP_ACK,
	OCTEP_PFVF_MBOX_TYPE_RSP_NACK,
};

enum octep_pfvf_mbox_cmd_status {
	OCTEP_PFVF_MBOX_CMD_STATUS_NOT_SETUP = 1,
	OCTEP_PFVF_MBOX_CMD_STATUS_TIMEDOUT = 2,
	OCTEP_PFVF_MBOX_CMD_STATUS_NACK = 3,
	OCTEP_PFVF_MBOX_CMD_STATUS_BUSY = 4
};

enum octep_pfvf_mbox_state {
	OCTEP_PFVF_MBOX_STATE_IDLE = 0,
	OCTEP_PFVF_MBOX_STATE_BUSY = 1,
};


enum octep_pfvf_link_status {
	OCTEP_PFVF_LINK_STATUS_DOWN,
	OCTEP_PFVF_LINK_STATUS_UP,
};

enum octep_pfvf_link_speed {
	OCTEP_PFVF_LINK_SPEED_NONE,
	OCTEP_PFVF_LINK_SPEED_1000,
	OCTEP_PFVF_LINK_SPEED_10000,
	OCTEP_PFVF_LINK_SPEED_25000,
	OCTEP_PFVF_LINK_SPEED_40000,
	OCTEP_PFVF_LINK_SPEED_50000,
	OCTEP_PFVF_LINK_SPEED_100000,
	OCTEP_PFVF_LINK_SPEED_LAST,
};

enum octep_pfvf_link_duplex {
	OCTEP_PFVF_LINK_HALF_DUPLEX,
	OCTEP_PFVF_LINK_FULL_DUPLEX,
};

enum octep_pfvf_link_autoneg {
	OCTEP_PFVF_LINK_AUTONEG,
	OCTEP_PFVF_LINK_FIXED,
};

#define OCTEP_PFVF_MBOX_TIMEOUT_WAIT_COUNT  10000
#define OCTEP_PFVF_MBOX_TIMEOUT_MS     500
#define OCTEP_PFVF_MBOX_MAX_RETRIES    2
#define OCTEP_PFVF_MBOX_MAX_DATA_SIZE  6
#define OCTEP_PFVF_MBOX_MORE_FRAG_FLAG 1
#define OCTEP_PFVF_MBOX_WRITE_WAIT_TIME msecs_to_jiffies(1)

union octep_pfvf_mbox_word {
	u64 u64;
	struct {
		u64 opcode:8;
		u64 type:2;
		u64 rsvd:6;
		u64 data:48;
	} s;
	struct {
		u64 opcode:8;
		u64 type:2;
		u64 frag:1;
		u64 rsvd:5;
		u8 data[6];
	} s_data;
	struct {
		u64 opcode:8;
		u64 type:2;
		u64 rsvd:6;
		u64 version:48;
	} s_version;
	struct {
		u64 opcode:8;
		u64 type:2;
		u64 rsvd:6;
		u8 mac_addr[6];
	} s_set_mac;
	struct {
		u64 opcode:8;
		u64 type:2;
		u64 rsvd:6;
		u64 mtu:48;
	} s_set_mtu;
	struct {
		u64 opcode:8;
		u64 type:2;
		u64 rsvd:6;
		u64 mtu:48;
	} s_get_mtu;
	struct {
		u64 opcode:8;
		u64 type:2;
		u64 state:1;
		u64 rsvd:53;
	} s_link_state;
	struct {
		u64 opcode:8;
		u64 type:2;
		u64 status:1;
		u64 rsvd:53;
	} s_link_status;
	struct {
		u64 opcode:8;
		u64 type:2;
		u64 pkind:8;
		u64 fsz:8;
		u64 rx_ol_flags:16;
		u64 tx_ol_flags:16;
		u64 rsvd:6;
	} s_fw_info;
	struct {
		u64 opcode:8;
		u64 type:2;
		u64 rsvd:22;
		u64 rx_ol_flags:16;
		u64 tx_ol_flags:16;
	} s_offloads;
} __packed;


void octep_pfvf_mbox_work(void *context, int pending);
int octep_setup_pfvf_mbox(struct octep_device *oct);
void octep_delete_pfvf_mbox(struct octep_device *oct);
void octep_pfvf_notify(struct octep_device *oct, struct octep_ctrl_mbox_msg *msg);
int octep_send_notification(struct octep_device *oct, u32 vf_id,
			    union octep_pfvf_mbox_word cmd);
#endif
