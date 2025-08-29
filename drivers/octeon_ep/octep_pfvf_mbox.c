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
#include "octep_regs_cnxk_pf.h"
#include "octep_pfvf_mbox.h"
#include "octep_ctrl_net.h"


/*
 * When a new command is implemented, the below table should be updated
 * with new command and it's version info.
 */

static u32 pfvf_cmd_versions[OCTEP_PFVF_MBOX_CMD_MAX] = {
	[0 ... OCTEP_PFVF_MBOX_CMD_DEV_REMOVE] = OCTEP_PFVF_MBOX_VERSION_V1,
	[OCTEP_PFVF_MBOX_CMD_GET_FW_INFO ... OCTEP_PFVF_MBOX_NOTIF_LINK_STATUS] =
		OCTEP_PFVF_MBOX_VERSION_V2,
	[OCTEP_PFVF_MBOX_NOTIF_PF_FLR] =
		OCTEP_PFVF_MBOX_VERSION_V3
};

static void octep_pfvf_validate_version(struct octep_device *oct,  u32 vf_id,
										union octep_pfvf_mbox_word cmd,
										union octep_pfvf_mbox_word *rsp)
{
	u32 vf_version = (u32)cmd.s_version.version;

	dev_dbg(oct->pdev, "VF id:%d VF version:%d PF version:%d\n",
			vf_id, vf_version, OCTEP_PFVF_MBOX_VERSION_CURRENT);
	if (vf_version < OCTEP_PFVF_MBOX_VERSION_CURRENT)
		rsp->s_version.version = vf_version;
	else
		rsp->s_version.version = OCTEP_PFVF_MBOX_VERSION_CURRENT;

	oct->vf_info[vf_id].mbox_version = rsp->s_version.version;
	dev_dbg(oct->pdev, "VF id:%d negotiated VF version:%d\n",
			vf_id, oct->vf_info[vf_id].mbox_version);

	rsp->s_version.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
}


static void octep_pfvf_get_link_status(struct octep_device *oct, u32 vf_id,
									   union octep_pfvf_mbox_word cmd,
									   union octep_pfvf_mbox_word *rsp)
{
	int status;

	status = octep_ctrl_net_get_link_status(oct, vf_id);
	if (status < 0) {
		rsp->s_link_status.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
		dev_err(oct->pdev, "Get VF link status failed via host control Mbox\n");
		return;
	}
	rsp->s_link_status.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
	rsp->s_link_status.status = status;
}

static void octep_pfvf_set_link_status(struct octep_device *oct, u32 vf_id,
									   union octep_pfvf_mbox_word cmd,
									   union octep_pfvf_mbox_word *rsp)
{
	int err;

	err = octep_ctrl_net_set_link_status(oct, vf_id, cmd.s_link_status.status, true);
	if (err) {
		rsp->s_link_status.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
		dev_err(oct->pdev, "Set VF link status failed via host control Mbox\n");
		return;
	}
	rsp->s_link_status.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
}

int
octep_send_notification(struct octep_device *oct, u32 vf_id,
						union octep_pfvf_mbox_word cmd)
{
	u32 max_rings_per_vf, vf_mbox_queue;
	struct octep_mbox *mbox;

	/* check if VF PF Mailbox is compatible for this notification */
	if (pfvf_cmd_versions[cmd.s.opcode] > oct->vf_info[vf_id].mbox_version) {
		dev_dbg(oct->pdev, "VF Mbox doesn't support Notification:%d on VF ver:%d\n",
				cmd.s.opcode, oct->vf_info[vf_id].mbox_version);
		return -EOPNOTSUPP;
	}

	max_rings_per_vf = CFG_GET_MAX_RPVF(oct->conf);
	vf_mbox_queue = vf_id * max_rings_per_vf;
	if (!oct->mbox[vf_mbox_queue]) {
		dev_err(oct->pdev, "Notif obtained for bad mbox vf %d\n", vf_id);
		return -EINVAL;
	}
	mbox = oct->mbox[vf_mbox_queue];

	mutex_lock(&mbox->lock);
	octep_write_csr64(oct, mbox->pf_vf_data_reg, cmd.u64);
	mutex_unlock(&mbox->lock);

	return 0;
}


static void octep_pfvf_set_rx_state(struct octep_device *oct, u32 vf_id,
									union octep_pfvf_mbox_word cmd,
									union octep_pfvf_mbox_word *rsp)
{
	int err;

	err = octep_ctrl_net_set_rx_state(oct, vf_id, cmd.s_link_state.state, true);
	if (err) {
		rsp->s_link_state.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
		dev_err(oct->pdev, "Set VF Rx link state failed via host control Mbox\n");
		return;
	}
	rsp->s_link_state.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
}

static void octep_pfvf_set_mtu(struct octep_device *oct, u32 vf_id,
							   union octep_pfvf_mbox_word cmd,
							   union octep_pfvf_mbox_word *rsp)
{
	int err;

	err = octep_ctrl_net_set_mtu(oct, vf_id, cmd.s_set_mtu.mtu, true);
	if (err) {
		rsp->s_set_mtu.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
		dev_err(oct->pdev, "Set VF MTU failed via host control Mbox\n");
		return;
	}
	rsp->s_set_mtu.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
}

static void octep_pfvf_get_mtu(struct octep_device *oct, u32 vf_id,
							   union octep_pfvf_mbox_word cmd,
							   union octep_pfvf_mbox_word *rsp)
{
	int max_rx_pktlen = oct->max_rx_pktlen + (ETHER_HDR_LEN + ETHER_CRC_LEN);

	rsp->s_set_mtu.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
	/* FIXME: next step is to get it from per vf_id structure stored in PF.
	 * Each VF may have different MTU setting
	 */
	rsp->s_get_mtu.mtu = max_rx_pktlen;
}

static void octep_pfvf_set_mac_addr(struct octep_device *oct,  u32 vf_id,
									union octep_pfvf_mbox_word cmd,
									union octep_pfvf_mbox_word *rsp)
{
	int err;

	if (oct->vf_info[vf_id].flags & OCTEON_PFVF_FLAG_MAC_SET_BY_PF) {
		dev_err(oct->pdev, "VF%d attampted to override administrative set MAC address\n",
				vf_id);
		rsp->s_set_mac.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
		return;
	}
	err = octep_ctrl_net_set_mac_addr(oct, vf_id, cmd.s_set_mac.mac_addr, true);
	if (err) {
		rsp->s_set_mac.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
		dev_err(oct->pdev, "Set VF MAC address failed via host control Mbox\n");
		return;
	}
	memcpy(oct->vf_info[vf_id].mac_addr, rsp->s_set_mac.mac_addr, ETHER_ADDR_LEN);
	rsp->s_set_mac.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
}

static void octep_pfvf_get_mac_addr(struct octep_device *oct,  u32 vf_id,
									union octep_pfvf_mbox_word cmd,
									union octep_pfvf_mbox_word *rsp)
{
	int err;

	if (oct->vf_info[vf_id].flags & OCTEON_PFVF_FLAG_MAC_SET_BY_PF) {
		dev_info(oct->pdev, "VF%d MAC addres set by PF\n", vf_id);
		memcpy(rsp->s_set_mac.mac_addr, oct->vf_info[vf_id].mac_addr,
			   ETHER_ADDR_LEN);
		rsp->s_set_mac.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
		return;
	}
	err = octep_ctrl_net_get_mac_addr(oct, vf_id, rsp->s_set_mac.mac_addr);
	if (err) {
		rsp->s_set_mac.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
		dev_err(oct->pdev, "Get VF%d MAC address failed via host control Mbox\n", 
				vf_id);
		return;
	}

	memcpy(oct->vf_info[vf_id].mac_addr, rsp->s_set_mac.mac_addr, ETHER_ADDR_LEN);
	rsp->s_set_mac.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
}

static void octep_pfvf_dev_remove(struct octep_device *oct,  u32 vf_id,
								  union octep_pfvf_mbox_word cmd,
								  union octep_pfvf_mbox_word *rsp)
{
	int err;

	err = octep_ctrl_net_dev_remove(oct, vf_id);
	if (err) {
		rsp->s.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
		dev_err(oct->pdev, "Failed to acknowledge fw of vf %d removal\n",
				vf_id);
		return;
	}
	rsp->s.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
}

static void octep_pfvf_get_fw_info(struct octep_device *oct,  u32 vf_id,
								   union octep_pfvf_mbox_word cmd,
								   union octep_pfvf_mbox_word *rsp)
{
	struct octep_fw_info fw_info;
	int err;

	err = octep_ctrl_net_get_info(oct, vf_id, &fw_info);
	if (err) {
		rsp->s_fw_info.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
		dev_err(oct->pdev, "Get VF info failed via host control Mbox\n");
		return;
	}

	rsp->s_fw_info.pkind = fw_info.pkind;
	rsp->s_fw_info.fsz = fw_info.fsz;
	rsp->s_fw_info.rx_ol_flags = fw_info.rx_ol_flags;
	rsp->s_fw_info.tx_ol_flags = fw_info.tx_ol_flags;

	rsp->s_fw_info.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
}

static void octep_pfvf_set_offloads(struct octep_device *oct, u32 vf_id,
									union octep_pfvf_mbox_word cmd,
									union octep_pfvf_mbox_word *rsp)
{
	struct octep_ctrl_net_offloads offloads = {
		.rx_offloads = cmd.s_offloads.rx_ol_flags,
		.tx_offloads = cmd.s_offloads.tx_ol_flags
	};
	int err;

	err = octep_ctrl_net_set_offloads(oct, vf_id, &offloads, true);
	if (err) {
		rsp->s_offloads.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
		dev_err(oct->pdev, "Set VF offloads failed via host control Mbox\n");
		return;
	}
	rsp->s_offloads.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;
}

int
octep_setup_pfvf_mbox(struct octep_device *oct)
{
	int i = 0, num_vfs = 0, rings_per_vf = 0;
	int ring = 0;

	num_vfs = oct->conf->sriov_cfg.max_vfs;
	rings_per_vf = oct->conf->sriov_cfg.max_rings_per_vf;

	for (i = 0; i < num_vfs; i++) {
		ring = rings_per_vf * i;
		oct->mbox[ring] = malloc(sizeof(*oct->mbox[ring]), M_DEVBUF, M_WAITOK | M_ZERO);
		if (!oct->mbox[ring])
			goto free_mbox;

		memset(oct->mbox[ring], 0, sizeof(struct octep_mbox));
		memset(&oct->vf_info[i], 0, sizeof(struct octep_pfvf_info));
		mutex_init(&oct->mbox[ring]->lock, "mbox_lock", NULL, MTX_DEF);

		oct->mbox[ring]->wk.tq = taskqueue_create("mbox_taskqueue", M_WAITOK,
												  taskqueue_thread_enqueue,
												  &oct->mbox[ring]->wk.tq);
		if (oct->mbox[ring]->wk.tq == NULL)
			goto free_mbox_cleanup;

		TASK_INIT(&oct->mbox[ring]->wk.work, 0, octep_pfvf_mbox_work, oct->mbox[ring]);
		taskqueue_start_threads(&oct->mbox[ring]->wk.tq, 1, PI_NET, "mbox_taskqueue_%d", i);

		oct->mbox[ring]->oct = oct;
		oct->mbox[ring]->vf_id = i;
		oct->hw_ops.setup_mbox_regs(oct, ring);
	}
	return 0;

free_mbox_cleanup:
	mutex_destroy(&oct->mbox[ring]->lock);
	free(oct->mbox[ring], M_DEVBUF);
	oct->mbox[ring] = NULL;

free_mbox:
	while (i > 0) {
		i--;
		ring = rings_per_vf * i;

		while (taskqueue_cancel(oct->mbox[ring]->wk.tq, &oct->mbox[ring]->wk.work, NULL))
			taskqueue_drain(oct->mbox[ring]->wk.tq, &oct->mbox[ring]->wk.work);

		taskqueue_free(oct->mbox[ring]->wk.tq);
		mutex_destroy(&oct->mbox[ring]->lock);
		free(oct->mbox[ring], M_DEVBUF);
		oct->mbox[ring] = NULL;
	}
	return -ENOMEM;
}

void
octep_delete_pfvf_mbox(struct octep_device *oct)
{
	int rings_per_vf = oct->conf->sriov_cfg.max_rings_per_vf;
	int num_vfs = oct->conf->sriov_cfg.active_vfs;
	int i = 0, ring = 0, vf_srn = 0;

	for (i = 0; i < num_vfs; i++) {
		ring = vf_srn + rings_per_vf * i;
		if (!oct->mbox[ring])
			continue;

		while (taskqueue_cancel(oct->mbox[ring]->wk.tq, &oct->mbox[ring]->wk.work, NULL))
			taskqueue_drain(oct->mbox[ring]->wk.tq, &oct->mbox[ring]->wk.work);

		taskqueue_free(oct->mbox[ring]->wk.tq);
		mutex_destroy(&oct->mbox[ring]->lock);
		free(oct->mbox[ring], M_DEVBUF);
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
	int err;
	struct octep_iface_link_info link_info;
	struct octep_iface_rx_stats rx_stats;
	struct octep_iface_tx_stats tx_stats;

	rsp->s_data.type = OCTEP_PFVF_MBOX_TYPE_RSP_ACK;

	if (cmd.s_data.frag != OCTEP_PFVF_MBOX_MORE_FRAG_FLAG) {
		mbox->config_data_index = 0;
		memset(mbox->config_data, 0, MAX_VF_PF_MBOX_DATA_SIZE);
		/* Based on the OPCODE CMD the PF driver
		 * specific API should be called to fetch
		 * the requested data
		 */
		switch (cmd.s.opcode) {
		case OCTEP_PFVF_MBOX_CMD_GET_LINK_INFO:
			memset(&link_info, 0, sizeof(link_info));
			err = octep_ctrl_net_get_link_info(oct, vf_id, &link_info);
			if (!err) {
				mbox->message_len = sizeof(link_info);
				*((int32_t *)rsp->s_data.data) = mbox->message_len;
				memcpy(mbox->config_data, (u8 *)&link_info, sizeof(link_info));
			} else {
				rsp->s_data.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
				return;
			}
			break;
		case OCTEP_PFVF_MBOX_CMD_GET_STATS:
			memset(&rx_stats, 0, sizeof(rx_stats));
			memset(&tx_stats, 0, sizeof(tx_stats));
			err = octep_ctrl_net_get_if_stats(oct, vf_id, &rx_stats, &tx_stats);
			if (!err) {
				mbox->message_len = sizeof(rx_stats) + sizeof(tx_stats);
				*((int32_t *)rsp->s_data.data) = mbox->message_len;
				memcpy(mbox->config_data, (u8 *)&rx_stats, sizeof(rx_stats));
				memcpy(mbox->config_data + sizeof(rx_stats), (u8 *)&tx_stats,
					   sizeof(tx_stats));

			} else {
				rsp->s_data.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
				return;
			}
			break;
		}
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

void octep_pfvf_notify(struct octep_device *oct, struct octep_ctrl_mbox_msg *msg)
{
	union octep_pfvf_mbox_word notif = { 0 };
	struct octep_ctrl_net_f2h_req *req;

	req = (struct octep_ctrl_net_f2h_req *)msg->sg_list[0].msg;
	switch (req->hdr.s.cmd) {
	case OCTEP_CTRL_NET_F2H_CMD_LINK_STATUS:
		notif.s_link_status.opcode = OCTEP_PFVF_MBOX_NOTIF_LINK_STATUS;
		notif.s_link_status.status = req->link.state;
		break;
	default:
		dev_info(oct->pdev,"Unknown mbox notif for vf: %u\n",
				 req->hdr.s.cmd);
		return;
	}

	notif.s.type = OCTEP_PFVF_MBOX_TYPE_CMD;
	octep_send_notification(oct, msg->hdr.s.vf_idx, notif);
}


void
octep_pfvf_mbox_work(void *context, int pending)
{
	struct octep_mbox *mbox = (struct octep_mbox *)context;
	union octep_pfvf_mbox_word cmd = { 0 };
	union octep_pfvf_mbox_word rsp = { 0 };
	struct octep_device *oct;
	int vf_id;

	oct = mbox->oct;
	vf_id = mbox->vf_id;

	mutex_lock(&mbox->lock);
	cmd.u64 = octep_read_csr64(oct, mbox->vf_pf_data_reg);
	if (cmd.u64 == 0xFFFFFFFFFFFFFFFFULL) {
		mutex_unlock(&mbox->lock);
		return;
	}

	rsp.u64 = 0;

	switch (cmd.s.opcode) {
	case OCTEP_PFVF_MBOX_CMD_VERSION:
		octep_pfvf_validate_version(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_PFVF_MBOX_CMD_GET_LINK_STATUS:
		octep_pfvf_get_link_status(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_PFVF_MBOX_CMD_SET_LINK_STATUS:
		octep_pfvf_set_link_status(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_PFVF_MBOX_CMD_SET_RX_STATE:
		octep_pfvf_set_rx_state(oct, vf_id, cmd, &rsp);
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
	case OCTEP_PFVF_MBOX_CMD_GET_LINK_INFO:
	case OCTEP_PFVF_MBOX_CMD_GET_STATS:
		octep_pfvf_pf_get_data(oct, mbox, vf_id, cmd, &rsp);
		break;
	case OCTEP_PFVF_MBOX_CMD_GET_MTU:
		octep_pfvf_get_mtu(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_PFVF_MBOX_CMD_DEV_REMOVE:
		octep_pfvf_dev_remove(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_PFVF_MBOX_CMD_GET_FW_INFO:
		octep_pfvf_get_fw_info(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_PFVF_MBOX_CMD_SET_OFFLOADS:
		octep_pfvf_set_offloads(oct, vf_id, cmd, &rsp);
		break;
	default:
		dev_err(oct->pdev, "PF-VF mailbox: invalid opcode %d\n", cmd.s.opcode);
		rsp.s.type = OCTEP_PFVF_MBOX_TYPE_RSP_NACK;
		break;
	}
	octep_write_csr64(oct, mbox->vf_pf_data_reg, rsp.u64);
	mutex_unlock(&mbox->lock);
}


