// SPDX-License-Identifier: GPL-2.0
/* Marvell Octeon EP (EndPoint) Ethernet Driver
 *
 * Copyright (C) 2020 Marvell.
 *
 */
#include <linux/string.h>
#include <linux/types.h>
#include <linux/etherdevice.h>
#include <linux/pci.h>

#include "octep_config.h"
#include "octep_main.h"
#include "octep_ctrl_net.h"

static const uint32_t req_hdr_sz = sizeof(struct octep_ctrl_net_req_hdr);
static const uint32_t mtu_sz = sizeof(struct octep_ctrl_net_h2f_req_cmd_mtu);
static const uint32_t mac_sz = sizeof(struct octep_ctrl_net_h2f_req_cmd_mac);
static const uint32_t state_sz = sizeof(struct octep_ctrl_net_h2f_req_cmd_state);
static const uint32_t link_info_sz = sizeof(struct octep_ctrl_net_link_info);
static const uint32_t get_stats_sz = sizeof(struct octep_ctrl_net_h2f_req_cmd_get_stats);

static void init_send_req(struct octep_ctrl_mbox_msg *msg, void *buf,
			  uint16_t sz)
{
	msg->hdr.s.flags = OCTEP_CTRL_MBOX_MSG_HDR_FLAG_REQ;
	msg->hdr.s.sz = req_hdr_sz + sz;
	msg->sg_num = 1;
	msg->sg_list[0].msg = buf;
	msg->sg_list[0].sz = msg->hdr.s.sz;
}

int octep_get_link_status(struct octep_device *oct)
{
	struct octep_ctrl_net_h2f_req req = {};
	struct octep_ctrl_net_h2f_resp *resp;
	struct octep_ctrl_mbox_msg msg = {};
	int err;

	init_send_req(&msg, (void *)&req, state_sz);
	req.hdr.cmd = OCTEP_CTRL_NET_H2F_CMD_LINK_STATUS;
	req.link.cmd = OCTEP_CTRL_NET_CMD_GET;

	err = octep_ctrl_mbox_send(&oct->ctrl_mbox, &msg, 1);
	if (err <= 0)
		return -EAGAIN;

	resp = (struct octep_ctrl_net_h2f_resp *)&req;
	return resp->link.state;
}

void octep_set_link_status(struct octep_device *oct, bool up)
{
	struct octep_ctrl_net_h2f_req req = {};
	struct octep_ctrl_mbox_msg msg = {};

	init_send_req(&msg, (void *)&req, state_sz);
	req.hdr.cmd = OCTEP_CTRL_NET_H2F_CMD_LINK_STATUS;
	req.link.cmd = OCTEP_CTRL_NET_CMD_SET;
	req.link.state = (up) ? OCTEP_CTRL_NET_STATE_UP : OCTEP_CTRL_NET_STATE_DOWN;

	octep_ctrl_mbox_send(&oct->ctrl_mbox, &msg, 1);
}

void octep_set_rx_state(struct octep_device *oct, bool up)
{
	struct octep_ctrl_net_h2f_req req = {};
	struct octep_ctrl_mbox_msg msg = {};

	init_send_req(&msg, (void *)&req, state_sz);
	req.hdr.cmd = OCTEP_CTRL_NET_H2F_CMD_RX_STATE;
	req.link.cmd = OCTEP_CTRL_NET_CMD_SET;
	req.link.state = (up) ? OCTEP_CTRL_NET_STATE_UP : OCTEP_CTRL_NET_STATE_DOWN;

	octep_ctrl_mbox_send(&oct->ctrl_mbox, &msg, 1);
}

int octep_get_mac_addr(struct octep_device *oct, u8 *addr)
{
	struct octep_ctrl_net_h2f_req req = {};
	struct octep_ctrl_net_h2f_resp *resp;
	struct octep_ctrl_mbox_msg msg = {};
	int err;

	init_send_req(&msg, (void *)&req, mac_sz);
	req.hdr.cmd = OCTEP_CTRL_NET_H2F_CMD_MAC;
	req.link.cmd = OCTEP_CTRL_NET_CMD_GET;

	err = octep_ctrl_mbox_send(&oct->ctrl_mbox, &msg, 1);
	if (err <= 0)
		return -EAGAIN;

	resp = (struct octep_ctrl_net_h2f_resp *)&req;
	memcpy(addr, resp->mac.addr, ETH_ALEN);

	return 0;
}

int octep_set_mac_addr(struct octep_device *oct, u8 *addr)
{
	struct octep_ctrl_net_h2f_req req = {};
	struct octep_ctrl_mbox_msg msg = {};

	init_send_req(&msg, (void *)&req, mac_sz);
	req.hdr.cmd = OCTEP_CTRL_NET_H2F_CMD_MAC;
	req.mac.cmd = OCTEP_CTRL_NET_CMD_SET;
	memcpy(&req.mac.addr, addr, ETH_ALEN);
	octep_ctrl_mbox_send(&oct->ctrl_mbox, &msg, 1);

	return 0;
}

int octep_set_mtu(struct octep_device *oct, int mtu)
{
	struct octep_ctrl_net_h2f_req req = {};
	struct octep_ctrl_mbox_msg msg = {};

	init_send_req(&msg, (void *)&req, mtu_sz);
	req.hdr.cmd = OCTEP_CTRL_NET_H2F_CMD_MTU;
	req.mtu.cmd = OCTEP_CTRL_NET_CMD_SET;
	req.mtu.val = mtu;

	octep_ctrl_mbox_send(&oct->ctrl_mbox, &msg, 1);

	return 0;
}

int octep_get_if_stats(struct octep_device *oct)
{
	void __iomem *iface_rx_stats;
	void __iomem *iface_tx_stats;
	struct octep_ctrl_net_h2f_req req = {};
	struct octep_ctrl_mbox_msg msg = {};
	int err;

	init_send_req(&msg, (void *)&req, get_stats_sz);
	req.hdr.cmd = OCTEP_CTRL_NET_H2F_CMD_GET_IF_STATS;
	req.mac.cmd = OCTEP_CTRL_NET_CMD_GET;
	req.get_stats.offset = oct->ctrl_mbox_ifstats_offset;

	err = octep_ctrl_mbox_send(&oct->ctrl_mbox, &msg, 1);
	if (err <= 0)
		return -EAGAIN;

	iface_rx_stats = oct->ctrl_mbox.barmem + oct->ctrl_mbox_ifstats_offset;
	iface_tx_stats = oct->ctrl_mbox.barmem + oct->ctrl_mbox_ifstats_offset +
			 sizeof(struct octep_iface_rx_stats);
	memcpy_fromio(&oct->iface_rx_stats, iface_rx_stats, sizeof(struct octep_iface_rx_stats));
	memcpy_fromio(&oct->iface_tx_stats, iface_tx_stats, sizeof(struct octep_iface_tx_stats));

	return err;
}

int octep_get_link_info(struct octep_device *oct)
{
	struct octep_ctrl_net_h2f_req req = {};
	struct octep_ctrl_net_h2f_resp *resp;
	struct octep_ctrl_mbox_msg msg = {};
	int err;

	init_send_req(&msg, (void *)&req, link_info_sz);
	req.hdr.cmd = OCTEP_CTRL_NET_H2F_CMD_LINK_INFO;
	req.mac.cmd = OCTEP_CTRL_NET_CMD_GET;

	err = octep_ctrl_mbox_send(&oct->ctrl_mbox, &msg, 1);
	if (err <= 0)
		return -EAGAIN;

	resp = (struct octep_ctrl_net_h2f_resp *)&req;
	oct->link_info.supported_modes = resp->link_info.supported_modes;
	oct->link_info.advertised_modes = resp->link_info.advertised_modes;
	oct->link_info.autoneg = resp->link_info.autoneg;
	oct->link_info.pause = resp->link_info.pause;
	oct->link_info.speed = resp->link_info.speed;

	return err;
}

int octep_set_link_info(struct octep_device *oct, struct octep_iface_link_info *link_info)
{
	struct octep_ctrl_net_h2f_req req = {};
	struct octep_ctrl_mbox_msg msg = {};

	init_send_req(&msg, (void *)&req, link_info_sz);
	req.hdr.cmd = OCTEP_CTRL_NET_H2F_CMD_LINK_INFO;
	req.link_info.cmd = OCTEP_CTRL_NET_CMD_SET;
	req.link_info.info.advertised_modes = link_info->advertised_modes;
	req.link_info.info.autoneg = link_info->autoneg;
	req.link_info.info.pause = link_info->pause;
	req.link_info.info.speed = link_info->speed;

	octep_ctrl_mbox_send(&oct->ctrl_mbox, &msg, 1);

	return 0;
}
