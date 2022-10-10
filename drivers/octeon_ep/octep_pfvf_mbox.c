// SPDX-License-Identifier: GPL-2.0
/* Marvell Octeon EP (EndPoint) Ethernet Driver
 *
 * Copyright (C) 2020 Marvell.
 *
 */
#include <linux/types.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/etherdevice.h>

#include "octep_config.h"
#include "octep_main.h"
#include "octep_pfvf_mbox.h"

static void octep_pfvf_get_link_status(struct octep_device *oct, u32 vf_id,
				      union octep_pfvf_mbox_word cmd,
				      union octep_pfvf_mbox_word *rsp)
{
	rsp->s_get_link.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
	rsp->s_get_link.link_status =  OCTEP_PFVF_LINK_STATUS_UP;
	rsp->s_get_link.link_speed = OCTEP_PFVF_LINK_SPEED_10000;
	rsp->s_get_link.duplex = OCTEP_PFVF_LINK_FULL_DUPLEX;
	rsp->s_get_link.autoneg = OCTEP_PFVF_LINK_AUTONEG;
}

static void octep_pfvf_set_mtu(struct octep_device *oct, u32 vf_id,
			       union octep_pfvf_mbox_word cmd,
			       union octep_pfvf_mbox_word *rsp)
{
	rsp->s_set_mtu.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
}

static void octep_pfvf_set_mac_addr(struct octep_device *oct,  u32 vf_id,
				    union octep_pfvf_mbox_word cmd,
				    union octep_pfvf_mbox_word *rsp)
{
	u8 vf_mac_addr[6];
	int i;

	for (i = 0; i < OCTEP_PFVF_MBOX_MAX_DATA_SIZE; i++) {
		vf_mac_addr[i] = cmd.s_set_mac.mac_addr[i];
		oct->vf_info[vf_id].mac_addr[i] = cmd.s_set_mac.mac_addr[i];
	}

	rsp->s_set_mac.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
}

static void octep_pfvf_get_mac_addr(struct octep_device *oct,  u32 vf_id,
				    union octep_pfvf_mbox_word cmd,
				    union octep_pfvf_mbox_word *rsp)
{
	int i;

	rsp->s_set_mac.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
	for (i = 0; i < OCTEP_PFVF_MBOX_MAX_DATA_SIZE; i++) {
		rsp->s_set_mac.mac_addr[i] = oct->vf_info[vf_id].mac_addr[i];
	}
}

int octep_setup_pfvf_mbox(struct octep_device *oct)
{
	int i = 0, num_vfs = 0, rings_per_vf = 0;
	int ring = 0;

	num_vfs = oct->conf->sriov_cfg.active_vfs;
	rings_per_vf = oct->conf->sriov_cfg.max_rings_per_vf;

	for (i = 0; i < num_vfs; i++) {
		ring  = rings_per_vf * i;
		oct->mbox[ring] = vmalloc(sizeof(struct octep_mbox));

		if (oct->mbox[ring] == NULL)
			goto free_mbox;

		memset(oct->mbox[ring], 0, sizeof(struct octep_mbox));
		mutex_init(&oct->mbox[ring]->lock);
		INIT_WORK(&oct->mbox[ring]->wk.work, octep_pfvf_mbox_work);
		oct->mbox[ring]->wk.ctxptr = oct->mbox[i];

		if (oct->mbox[i] == NULL)
			oct->mbox[ring]->wk.ctxptr = oct->mbox[ring];

		oct->mbox[ring]->oct = oct;
		oct->mbox[ring]->vf_id = i;
		oct->hw_ops.setup_mbox_regs(oct, ring);
	}
	return 0;

free_mbox:
	while (i) {
		i--;
		ring  = rings_per_vf * i;
		cancel_work_sync(&oct->mbox[ring]->wk.work);
		vfree(oct->mbox[ring]);
		oct->mbox[ring] = NULL;
	}
	return 1;
}

void octep_delete_pfvf_mbox(struct octep_device *oct)
{
	int rings_per_vf = oct->conf->sriov_cfg.max_rings_per_vf;
	int num_vfs = oct->conf->sriov_cfg.active_vfs;
	int i = 0, ring = 0, vf_srn = 0;

	for (i = 0; i < num_vfs; i++) {
		ring  = vf_srn + rings_per_vf * i;
		if (!oct->mbox[ring])
			continue;

		if (work_pending(&oct->mbox[ring]->wk.work))
			cancel_work_sync(&oct->mbox[ring]->wk.work);
		vfree(oct->mbox[ring]);
		oct->mbox[ring] = NULL;
	}
}

static void octep_pfvf_pf_get_data(struct octep_device *oct,
				   struct octep_mbox *mbox, int vf_id,
				   union octep_pfvf_mbox_word cmd,
				   union octep_pfvf_mbox_word *rsp)
{
	int length = 0;
	int i = 0;

	rsp->s_data.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;

	if (cmd.s_data.frag != OCTEP_PFVF_MBOX_MORE_FRAG_FLAG) {
		mbox->config_data_index = 0;
		memset(mbox->config_data, 0, MAX_VF_PF_MBOX_DATA_SIZE);
		/* Based on the OPCODE CMD the PF driver
		 * specific API should be called to fetch
		 * the requested data
		 */
		*((int32_t *)rsp->s_data.data) = mbox->message_len;
		return;
	}

	if (mbox->message_len > OCTEP_PFVF_MBOX_MAX_DATA_SIZE)
		length = OCTEP_PFVF_MBOX_MAX_DATA_SIZE;
	else
		length = mbox->message_len;

	mbox->message_len -= length;

	for (i = 0; i < length; i++) {
		rsp->s_data.data[i] =
			mbox->config_data[mbox->config_data_index];
		mbox->config_data_index++;
	}
}

void octep_pfvf_mbox_work(struct work_struct *work)
{
	struct octep_pfvf_mbox_wk *wk = container_of(work, struct octep_pfvf_mbox_wk, work);
	union octep_pfvf_mbox_word cmd = { 0 };
	union octep_pfvf_mbox_word rsp = { 0 };
	struct octep_mbox *mbox = NULL;
	struct octep_device *oct = NULL;
	int vf_id;

	mbox = (struct octep_mbox *)wk->ctxptr;
	oct = (struct octep_device *)mbox->oct;
	vf_id = mbox->vf_id;

	mutex_lock(&mbox->lock);
	cmd.u64 = readq(mbox->vf_pf_data_reg);
	rsp.u64 = 0;

	switch (cmd.s.opcode) {
	case OCTEP_PFVF_MBOX_CMD_GET_LINK:
		octep_pfvf_get_link_status(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_PFVF_MBOX_CMD_SET_MTU:
		octep_pfvf_set_mtu(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_PFVF_MBOX_CMD_SET_MAC_ADDR:
		octep_pfvf_set_mac_addr(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_PFVF_MBOX_CMD_GET_MAC_ADDR:
		octep_pfvf_get_mac_addr(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_PFVF_MBOX_CMD_GET_STATS:
		octep_pfvf_pf_get_data(oct, mbox, vf_id, cmd, &rsp);
		break;
	default:
		dev_err(&oct->pdev->dev, "%s OCTEP_PFVF_MBOX_TYPE_RSP_NACK opcode:%d\n",
					  __func__, cmd.s.opcode);
		rsp.s.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
		break;
	}
	writeq(rsp.u64, mbox->vf_pf_data_reg);
	mutex_unlock(&mbox->lock);
}
