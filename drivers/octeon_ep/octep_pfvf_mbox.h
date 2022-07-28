#ifndef _OCTEP_PFVF_MBOX_H_
#define _OCTEP_PFVF_MBOX_H_


#define OCTEP_PF_MBOX_VERSION 0

enum octep_vf_mbox_opcode {
	OCTEP_VF_MBOX_CMD_SET_MTU,
	OCTEP_VF_MBOX_CMD_SET_MAC_ADDR,
	OCTEP_VF_MBOX_CMD_GET_MAC_ADDR,
	OCTEP_VF_MBOX_CMD_START_QUEUE,
	OCTEP_VF_MBOX_CMD_STOP_QUEUE,
	OCTEP_VF_MBOX_CMD_GET_LINK,
	OCTEP_VF_MBOX_CMD_BULK_SEND,
	OCTEP_VF_MBOX_CMD_BULK_GET,
	OCTEP_VF_MBOX_CMD_LAST,
};

enum octep_vf_mbox_word_type {
	OCTEP_VF_MBOX_TYPE_CMD,
	OCTEP_VF_MBOX_TYPE_RSP_ACK,
	OCTEP_VF_MBOX_TYPE_RSP_NACK,
};

struct octep_vf_mbox_word_hdr {
	uint64_t version:3;
	uint64_t rsvd1:2;
	uint64_t opcode:5;
	uint64_t rsvd2:3;
	uint64_t id:1;
	uint64_t type:2;
} __packed;

enum octep_vf_link_status {
	OCTEP_VF_LINK_STATUS_DOWN,
	OCTEP_VF_LINK_STATUS_UP,
};

enum octep_vf_link_speed {
	OCTEP_VF_LINK_SPEED_NONE,
	OCTEP_VF_LINK_SPEED_100,
	OCTEP_VF_LINK_SPEED_1000,
	OCTEP_VF_LINK_SPEED_2500,
	OCTEP_VF_LINK_SPEED_5000,
	OCTEP_VF_LINK_SPEED_10000,
	OCTEP_VF_LINK_SPEED_20000,
	OCTEP_VF_LINK_SPEED_25000,
	OCTEP_VF_LINK_SPEED_40000,
	OCTEP_VF_LINK_SPEED_50000,
	OCTEP_VF_LINK_SPEED_100000,
	OCTEP_VF_LINK_SPEED_LAST,
};

enum octep_vf_link_duplex {
	OCTEP_VF_LINK_HALF_DUPLEX,
	OCTEP_VF_LINK_FULL_DUPLEX,
};

enum octep_vf_link_autoneg {
	OCTEP_VF_LINK_AUTONEG,
	OCTEP_VF_LINK_FIXED,
};

struct octep_vf_mbox_link {
	uint64_t link_status:1;
	uint64_t link_speed:8;
	uint64_t duplex:1;
	uint64_t autoneg:1;
	uint64_t rsvd:37;
} __packed;

#define OCTEP_VF_MBOX_TIMEOUT_MS     10
#define OCTEP_VF_MBOX_MAX_RETRIES    2
#define OCTEP_VF_MBOX_VERSION        1
#define OCTEP_VF_MBOX_MAX_DATA_SIZE  6
#define OCTEP_VF_MBOX_MORE_FRAG_FLAG 1

union octep_vf_mbox_word {
	u64 u64;
	struct {
		u64 version:3;
		u64 rsvd1:2;
		u64 opcode:5;
		u64 rsvd2:3;
		u64 id:1;
		u64 type:2;
		u64 data:48;
	} s;
	struct {
		u64 version:3;
		u64 rsvd1:2;
		u64 opcode:5;
		u64 rsvd2:2;
		u64 frag:1;
		u64 id:1;
		u64 type:2;
		u8 data[6];
	} s_data;
	struct {
		u64 version:3;
		u64 rsvd1:2;
		u64 opcode:5;
		u64 rsvd2:3;
		u64 id:1;
		u64 type:2;
		u8 mac_addr[6];
	} s_set_mac;
	struct {
		u64 version:3;
		u64 rsvd1:2;
		u64 opcode:5;
		u64 rsvd2:3;
		u64 id:1;
		u64 type:2;
		u64 mtu:48;
	} s_set_mtu;
	struct {
		u64 version:3;
		u64 rsvd1:2;
		u64 opcode:5;
		u64 rsvd2:3;
		u64 id:1;
		u64 type:2;
		u64 link_status:1;
		u64 link_speed:8;
		u64 duplex:1;
		u64 autoneg:1;
		u64 rsvd:37;
	} s_get_link;
} __packed;

void octep_vf_mbox_work(struct work_struct *work);
int octep_setup_vf_mbox(struct octep_device *oct);
void octep_delete_vf_mbox(struct octep_device *oct);
#endif
