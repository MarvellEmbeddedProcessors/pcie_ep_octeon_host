/* SPDX-License-Identifier: GPL-2.0 */
/* Marvell Octeon EP (EndPoint) Ethernet Driver
 *
 * Copyright (C) 2020 Marvell.
 *
 */
#ifndef _OCTEP_VF_MBOX_H_
#define _OCTEP_VF_MBOX_H_


#define OCTEP_VF_MBOX_VERSION 0

enum octep_pfvf_mbox_opcode {
	OCTEP_PFVF_MBOX_CMD_SET_MTU,
	OCTEP_PFVF_MBOX_CMD_SET_MAC_ADDR,
	OCTEP_PFVF_MBOX_CMD_GET_MAC_ADDR,
	OCTEP_PFVF_MBOX_CMD_START_QUEUE,
	OCTEP_PFVF_MBOX_CMD_STOP_QUEUE,
	OCTEP_PFVF_MBOX_CMD_GET_LINK,
	OCTEP_PFVF_MBOX_CMD_GET_STATS,
	OCTEP_PFVF_MBOX_CMD_LAST,
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

struct octep_pfvf_mbox_link {
	uint64_t link_status:1;
	uint64_t link_speed:8;
	uint64_t duplex:1;
	uint64_t autoneg:1;
	uint64_t rsvd:37;
} __packed;

#define OCTEP_PFVF_MBOX_TIMEOUT_MS     10
#define OCTEP_PFVF_MBOX_MAX_RETRIES    2
#define OCTEP_PFVF_MBOX_VERSION        0
#define OCTEP_PFVF_MBOX_MAX_DATA_SIZE  6
#define OCTEP_PFVF_MBOX_MAX_DATA_BUF_SIZE 256
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
		u64 link_status:1;
		u64 link_speed:8;
		u64 duplex:1;
		u64 autoneg:1;
		u64 rsvd:43;
	} s_get_link;
} __packed;

int octep_vf_setup_mbox(struct octep_vf_device *oct);
void octep_vf_delete_mbox(struct octep_vf_device *oct);
int octep_vf_mbox_send_cmd(struct octep_vf_device *oct, union octep_pfvf_mbox_word cmd,
			   union octep_pfvf_mbox_word *rsp);
int octep_vf_mbox_bulk_read(struct octep_vf_device *oct, enum octep_pfvf_mbox_opcode opcode,
			    u8 *data, int *size);
int octep_vf_mbox_set_mtu(struct octep_vf_device *oct, int mtu);
int octep_vf_mbox_set_mac_addr(struct octep_vf_device *oct, char *mac_addr);
int octep_vf_mbox_get_mac_addr(struct octep_vf_device *oct, char *mac_addr);
#endif
