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

/* Control plane version */
#define OCTEP_CP_VERSION_CURRENT    OCTEP_CP_VERSION(1, 0, 0)
#define GENMASK(h, l) (((~0U) << (l)) & (~0U >> (sizeof(unsigned int) * 8 - 1 - (h))))

static const u32 req_hdr_sz = sizeof(union octep_ctrl_net_req_hdr);
static const u32 mtu_sz = sizeof(struct octep_ctrl_net_h2f_req_cmd_mtu);
static const u32 mac_sz = sizeof(struct octep_ctrl_net_h2f_req_cmd_mac);
static const u32 state_sz = sizeof(struct octep_ctrl_net_h2f_req_cmd_state);
static const u32 link_info_sz = sizeof(struct octep_ctrl_net_link_info);
static const u32 offloads_sz = sizeof(struct octep_ctrl_net_offloads);
static volatile int ctrl_net_msg_id;

static void init_send_req(struct octep_ctrl_mbox_msg *msg, void *buf,
			  u16 sz, int vfid)
{
	msg->hdr.s.flags = OCTEP_CTRL_MBOX_MSG_HDR_FLAG_REQ;
	msg->hdr.s.msg_id = atomic_fetchadd_int(&ctrl_net_msg_id, 1) &
		GENMASK(sizeof(msg->hdr.s.msg_id) * NBBY, 0);
	msg->hdr.s.sz = req_hdr_sz + sz;
	msg->sg_num = 1;
	msg->sg_list[0].msg = buf;
	msg->sg_list[0].sz = msg->hdr.s.sz;
	if (vfid != OCTEP_CTRL_NET_INVALID_VFID) {
		msg->hdr.s.is_vf = 1;
		msg->hdr.s.vf_idx = vfid;
	}
}

static int
send_mbox_req(struct octep_device *oct,
	      struct octep_ctrl_net_wait_data *d,
	      bool wait_for_response)
{
	int err, ret, cmd;

	cmd = d->data.req.hdr.s.cmd;
	if (octep_ctrl_net_h2f_cmd_versions[cmd] > oct->ctrl_mbox.max_fw_version ||
	    octep_ctrl_net_h2f_cmd_versions[cmd] < oct->ctrl_mbox.min_fw_version)
		return -EOPNOTSUPP;

	err = octep_ctrl_mbox_send((void *)oct, &oct->ctrl_mbox, &d->msg);
	if (err < 0)
		return err;

	if (!wait_for_response)
		return 0;

	d->done = 0;
	mutex_lock(&oct->ctrl_req_mtx);
	TAILQ_INSERT_TAIL(&oct->ctrl_req_wait_list, d, list);

	ret = cv_timedwait(&oct->ctrl_req_cv, &oct->ctrl_req_mtx, (2000 * hz) / 1000);

	TAILQ_REMOVE(&oct->ctrl_req_wait_list, d, list);
	mutex_unlock(&oct->ctrl_req_mtx);

	if (ret != 0)
	{
		if (ret == -EINTR)
		{
			return -EINTR;
		}
		return -EAGAIN;
	}

	if (d->done == 0) {
		dev_info(oct->pdev, "%s: signaled but done=0, potential bug\n", __func__);
		return -EAGAIN;
	}

	if (d->data.resp.hdr.s.reply != OCTEP_CTRL_NET_REPLY_OK)
		return -EAGAIN;

	return 0;
}

static int validate_fw_version(struct octep_ctrl_mbox *ctrl_mbox)
{
	if (ctrl_mbox->version < ctrl_mbox->min_fw_version ||
	    ctrl_mbox->version > ctrl_mbox->max_fw_version)
		return -EINVAL;

	return 0;
}

int
octep_ctrl_net_init(struct octep_device *oct)
{
	device_t pdev = oct->pdev;
	struct octep_ctrl_mbox *ctrl_mbox;
	int ret;

	/* Initialize mutex and condition variable */
	mutex_init(&oct->ctrl_req_mtx, "ctrl_req_mtx", NULL, MTX_DEF);
	cv_init(&oct->ctrl_req_cv, "ctrl_req_cv");

	/* Initialize the wait list */
	TAILQ_INIT(&oct->ctrl_req_wait_list);

	/* Initialize control mbox */
	ctrl_mbox = &oct->ctrl_mbox;
	ctrl_mbox->version = OCTEP_CP_VERSION_CURRENT;
	ctrl_mbox->barmem = CFG_GET_CTRL_MBOX_MEM_ADDR(oct->conf);
	ret = octep_ctrl_mbox_init((void *)oct, ctrl_mbox);
	if (ret) {
		dev_err(pdev, "Failed to initialize control mbox\n");
		goto init_fail;
	}

	dev_info(pdev, "Control plane versions host: %llx, firmware: %x:%x\n",
		 (unsigned long long)ctrl_mbox->version,
		 ctrl_mbox->min_fw_version, ctrl_mbox->max_fw_version);
	ret = validate_fw_version(ctrl_mbox);
	if (ret < 0) {
		dev_err(pdev, "Control plane version mismatch\n");
		octep_ctrl_mbox_uninit( (void *)oct, ctrl_mbox);
		ret = -EINVAL;
		goto init_fail;
	}
	oct->ctrl_mbox_ifstats_offset = ctrl_mbox->barmem_sz;

	return 0;

init_fail:
	cv_destroy(&oct->ctrl_req_cv);
	mutex_destroy(&oct->ctrl_req_mtx);
	return ret;
}

int octep_ctrl_net_get_link_status(struct octep_device *oct, int vfid)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;
	int err;

	init_send_req(&d.msg, (void *)req, state_sz, vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_LINK_STATUS;
	req->link.cmd = OCTEP_CTRL_NET_CMD_GET;
	err = send_mbox_req(oct, &d, true);
	if (err < 0)
		return err;

	return d.data.resp.link.state;
}

int octep_ctrl_net_set_link_status(struct octep_device *oct, int vfid, bool up,
				   bool wait_for_response)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;

	init_send_req(&d.msg, req, state_sz, vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_LINK_STATUS;
	req->link.cmd = OCTEP_CTRL_NET_CMD_SET;
	req->link.state = (up) ? OCTEP_CTRL_NET_STATE_UP :
		OCTEP_CTRL_NET_STATE_DOWN;

	return send_mbox_req(oct, &d, wait_for_response);
}

int octep_ctrl_net_set_rx_state(struct octep_device *oct, int vfid, bool up,
				bool wait_for_response)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;

	init_send_req(&d.msg, req, state_sz, vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_RX_STATE;
	req->link.cmd = OCTEP_CTRL_NET_CMD_SET;
	req->link.state = (up) ? OCTEP_CTRL_NET_STATE_UP :
		OCTEP_CTRL_NET_STATE_DOWN;

	return send_mbox_req(oct, &d, wait_for_response);
}

int octep_ctrl_net_get_mac_addr(struct octep_device *oct, int vfid, u8 *addr)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;
	int err;

	init_send_req(&d.msg, req, mac_sz, vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_MAC;
	req->link.cmd = OCTEP_CTRL_NET_CMD_GET;

	err = send_mbox_req(oct, &d, true);
	if (err < 0)
		return err;

	memcpy(addr, d.data.resp.mac.addr, ETHER_ADDR_LEN);

	return 0;
}

int octep_ctrl_net_set_mac_addr(struct octep_device *oct, int vfid, u8 *addr,
				bool wait_for_response)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;

	init_send_req(&d.msg, req, mac_sz, vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_MAC;
	req->mac.cmd = OCTEP_CTRL_NET_CMD_SET;
	memcpy(&req->mac.addr, addr, ETHER_ADDR_LEN);

	return send_mbox_req(oct, &d, wait_for_response);
}


int octep_ctrl_net_get_mtu(struct octep_device *oct, int vfid)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;
	int err;

	init_send_req(&d.msg, req, mtu_sz, vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_MTU;
	req->mtu.cmd = OCTEP_CTRL_NET_CMD_GET;

	err = send_mbox_req(oct, &d, true);
	if (err < 0)
		return err;

	return d.data.resp.mtu.val;
}

int octep_ctrl_net_set_mtu(struct octep_device *oct, int vfid, int mtu,
			   bool wait_for_response)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;

	init_send_req(&d.msg, req, mtu_sz, vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_MTU;
	req->mtu.cmd = OCTEP_CTRL_NET_CMD_SET;
	req->mtu.val = mtu;

	return send_mbox_req(oct, &d, wait_for_response);
}

int octep_ctrl_net_get_if_stats(struct octep_device *oct, int vfid,
				struct octep_iface_rx_stats *rx_stats,
				struct octep_iface_tx_stats *tx_stats)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;
	struct octep_ctrl_net_h2f_resp *resp;
	int err;

	init_send_req(&d.msg, req, 0, vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_GET_IF_STATS;
	err = send_mbox_req(oct, &d, true);
	if (err < 0)
		return err;

	resp = &d.data.resp;
	memcpy(rx_stats,
	       &resp->if_stats.rx_stats,
	       sizeof(struct octep_iface_rx_stats));
	memcpy(tx_stats,
	       &resp->if_stats.tx_stats,
	       sizeof(struct octep_iface_tx_stats));
	return 0;
}


int octep_ctrl_net_get_link_info(struct octep_device *oct, int vfid,
				 struct octep_iface_link_info *link_info)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;
	struct octep_ctrl_net_h2f_resp *resp;
	int err;

	init_send_req(&d.msg, req, link_info_sz, vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_LINK_INFO;
	req->link_info.cmd = OCTEP_CTRL_NET_CMD_GET;
	err = send_mbox_req(oct, &d, true);
	if (err < 0)
		return err;

	resp = &d.data.resp;
	link_info->supported_modes = resp->link_info.supported_modes;
	link_info->advertised_modes = resp->link_info.advertised_modes;
	link_info->autoneg = resp->link_info.autoneg;
	link_info->pause = resp->link_info.pause;
	link_info->speed = resp->link_info.speed;

	return 0;
}

int octep_ctrl_net_set_link_info(struct octep_device *oct, int vfid,
				 struct octep_iface_link_info *link_info,
				 bool wait_for_response)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;

	init_send_req(&d.msg, req, link_info_sz, vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_LINK_INFO;
	req->link_info.cmd = OCTEP_CTRL_NET_CMD_SET;
	req->link_info.info.advertised_modes = link_info->advertised_modes;
	req->link_info.info.autoneg = link_info->autoneg;
	req->link_info.info.pause = link_info->pause;
	req->link_info.info.speed = link_info->speed;

	return send_mbox_req(oct, &d, wait_for_response);
}

static int process_mbox_req(struct octep_device *oct,
			    struct octep_ctrl_mbox_msg *msg)
{
	return 0;
}



static int
process_mbox_resp(struct octep_device *oct, struct octep_ctrl_mbox_msg *msg)
{
	struct octep_ctrl_net_wait_data *pos, *n;

	mutex_lock(&oct->ctrl_req_mtx);
	TAILQ_FOREACH_SAFE(pos, &oct->ctrl_req_wait_list, list, n) {
		if (pos->msg.hdr.s.msg_id == msg->hdr.s.msg_id) {
			memcpy(&pos->data.resp, msg->sg_list[0].msg, msg->hdr.s.sz);
			pos->done = 1;
			cv_broadcast(&oct->ctrl_req_cv); /* Wake all waiters */
			break;
		}
	}
	mutex_unlock(&oct->ctrl_req_mtx);

	return 0;
}

static int process_mbox_notify(struct octep_device *oct,
			       struct octep_ctrl_mbox_msg *msg)
{
	struct octep_ctrl_net_f2h_req *req;
	int cmd;

	req = (struct octep_ctrl_net_f2h_req *)msg->sg_list[0].msg;
	cmd = req->hdr.s.cmd;

	/* check if we support this command */
	if (octep_ctrl_net_f2h_cmd_versions[cmd] > OCTEP_CP_VERSION_CURRENT ||
	    octep_ctrl_net_f2h_cmd_versions[cmd] < OCTEP_CP_VERSION_CURRENT)
		return -EOPNOTSUPP;

	if (msg->hdr.s.is_vf) {
		octep_pfvf_notify(oct, msg);
		return 0;
	}

	switch (cmd) {
	case OCTEP_CTRL_NET_F2H_CMD_LINK_STATUS:

#if 0
		if (netif_running(netdev)) {
			if (req->link.state) {
				octep_dev_info(oct, "netif_carrier_on\n");

				netif_carrier_on(netdev);
			} else {
				dev_info(&oct->pdev->dev, "netif_carrier_off\n");
				netif_carrier_off(netdev);
			}
		}
#endif
		break;
	default:
		dev_err(oct->pdev,"Unknown mbox req : %u\n", req->hdr.s.cmd);
		break;
	}

	return 0;
}

int octep_ctrl_net_recv_fw_messages(struct octep_device *oct)
{
	static u16 msg_sz = sizeof(union octep_ctrl_net_max_data);
	union octep_ctrl_net_max_data data = {0};
	struct octep_ctrl_mbox_msg msg = {0};
	int ret;

	msg.hdr.s.sz = msg_sz;
	msg.sg_num = 1;
	msg.sg_list[0].sz = msg_sz;
	msg.sg_list[0].msg = &data;
	while (true) {
		/* mbox will overwrite msg.hdr.s.sz so initialize it */
		msg.hdr.s.sz = msg_sz;
		ret = octep_ctrl_mbox_recv((void *)oct, &oct->ctrl_mbox, (struct octep_ctrl_mbox_msg *)&msg);
		if (ret < 0)
			break;

		if (msg.hdr.s.flags & OCTEP_CTRL_MBOX_MSG_HDR_FLAG_REQ)
			process_mbox_req(oct, &msg);
		else if (msg.hdr.s.flags & OCTEP_CTRL_MBOX_MSG_HDR_FLAG_RESP)
			process_mbox_resp(oct, &msg);
		else if (msg.hdr.s.flags & OCTEP_CTRL_MBOX_MSG_HDR_FLAG_NOTIFY)
			process_mbox_notify(oct, &msg);
	}

	return 0;
}

int octep_ctrl_net_get_info(struct octep_device *oct, int vfid,
			    struct octep_fw_info *info)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;
	struct octep_ctrl_net_h2f_resp *resp;
	int err;

	init_send_req(&d.msg, req, 0, vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_GET_INFO;
	req->link_info.cmd = OCTEP_CTRL_NET_CMD_GET;
	err = send_mbox_req(oct, &d, true);
	if (err < 0)
		return err;

	resp = &d.data.resp;
	memcpy(info, &resp->info.fw_info, sizeof (struct octep_fw_info));

	return 0;
}

int octep_ctrl_net_dev_remove(struct octep_device *oct, int vfid)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;

	dev_info(oct->pdev, "Sending dev_unload msg to fw\n");
	init_send_req(&d.msg, req, sizeof(int), vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_DEV_REMOVE;

	return send_mbox_req(oct, &d, false);
}

int octep_ctrl_net_set_offloads(struct octep_device *oct, int vfid,
				struct octep_ctrl_net_offloads *offloads,
				bool wait_for_response)
{
	struct octep_ctrl_net_wait_data d = {0};
	struct octep_ctrl_net_h2f_req *req = &d.data.req;

	init_send_req(&d.msg, req, offloads_sz, vfid);
	req->hdr.s.cmd = OCTEP_CTRL_NET_H2F_CMD_OFFLOADS;
	req->offloads.cmd = OCTEP_CTRL_NET_CMD_SET;
	req->offloads.offloads = *offloads;

	return send_mbox_req(oct, &d, wait_for_response);
}

int
octep_ctrl_net_uninit(struct octep_device *oct)
{
	struct octep_ctrl_net_wait_data *pos, *n;

	octep_ctrl_net_dev_remove(oct, OCTEP_CTRL_NET_INVALID_VFID);

	mutex_lock(&oct->ctrl_req_mtx);
	TAILQ_FOREACH_SAFE(pos, &oct->ctrl_req_wait_list, list, n) {
		pos->done = 1;
	}
	cv_broadcast(&oct->ctrl_req_cv); /* Wake all waiters */
	mutex_unlock(&oct->ctrl_req_mtx);

	octep_ctrl_mbox_uninit((void *)oct, &oct->ctrl_mbox);
	mutex_destroy(&oct->ctrl_req_mtx);
	cv_destroy(&oct->ctrl_req_cv);

	return 0;
}
