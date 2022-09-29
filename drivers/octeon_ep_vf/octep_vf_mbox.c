/* SPDX-License-Identifier: GPL-2.0 */
/* Marvell Octeon EP (EndPoint) VF Ethernet Driver
 *
 * Copyright (C) 2020 Marvell.
 *
 */
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include "octep_vf_config.h"
#include "octep_vf_main.h"

int octep_vf_setup_mbox(struct octep_vf_device *oct)
{
	int ring = 0;

	oct->mbox[ring] = vmalloc(sizeof(struct octep_vf_mbox));
	if (!oct->mbox[ring])
		return -1;

	memset(oct->mbox[ring], 0, sizeof(struct octep_vf_mbox));
	spin_lock_init(&oct->mbox[ring]->lock);

	oct->hw_ops.setup_mbox_regs(oct, ring);
	INIT_WORK(&oct->mbox[ring]->wk.work, octep_vf_mbox_work);
	oct->mbox[ring]->wk.ctxptr = oct;

	dev_info(&oct->pdev->dev, "%s setup vf mbox successfully\n", __func__);
	return 0;

}

void octep_vf_delete_mbox(struct octep_vf_device *oct)
{
	int ring = 0;
	if (work_pending(&oct->mbox[ring]->wk.work))
		cancel_work_sync(&oct->mbox[ring]->wk.work);
	vfree(oct->mbox[ring]);
	oct->mbox[ring] = NULL;
	dev_info(&oct->pdev->dev, "%s freed vf mbox struct.\n", __func__);
}

void octep_vf_mbox_work(struct work_struct *work)
{
	struct octep_vf_mbox_wk *wk = container_of(work, struct octep_vf_mbox_wk, work);
	struct octep_vf_mbox *mbox = NULL;
	struct octep_vf_device *oct = NULL;
	struct octep_vf_iface_link_info *link_info;
	u64 pf_vf_data;
	int ring = 0;

	oct = (struct octep_vf_device *)wk->ctxptr;
	link_info = &oct->link_info;
	mbox = oct->mbox[ring];
	pf_vf_data = readq(mbox->mbox_read_reg);
	if (!(pf_vf_data & OCTEP_PFVF_LINK_STATUS_DOWN))
		link_info->oper_up = OCTEP_PFVF_LINK_STATUS_DOWN;
	else if (pf_vf_data & OCTEP_PFVF_LINK_STATUS_UP)
		link_info->oper_up = OCTEP_PFVF_LINK_STATUS_UP;
}

inline void octep_vf_mbox_set_state(struct octep_vf_mbox *mbox, enum octep_pfvf_mbox_state state)
{
	unsigned long flags;

	spin_lock_irqsave(&mbox->lock, flags);
	mbox->state = state;
	spin_unlock_irqrestore(&mbox->lock, flags);
}

enum octep_pfvf_mbox_state octep_vf_mbox_get_state(struct octep_vf_mbox *mbox)
{
	unsigned long flags;
	enum octep_pfvf_mbox_state state;

	spin_lock_irqsave(&mbox->lock, flags);
	state = mbox->state;
	spin_unlock_irqrestore(&mbox->lock, flags);
	return state;
}

static int __octep_vf_mbox_send_cmd(struct octep_vf_device *oct, union octep_pfvf_mbox_word cmd,
				  union octep_pfvf_mbox_word *rsp)
{
	volatile uint64_t reg_val = 0ull;
	int count = 0;
	long timeout = OCTEP_PFVF_MBOX_WRITE_WAIT_TIME;
	struct octep_vf_mbox *mbox = oct->mbox[0];

	if (!mbox) {
		dev_err(&oct->pdev->dev, "%s Mbox is not initialized\n", __func__);
		return OCTEP_PFVF_MBOX_STATUS_NOT_SETUP;
	}

	cmd.s.type = OCTEP_PFVF_MBOX_TYPE_CMD;
	writeq(cmd.u64, mbox->mbox_write_reg);
	for (count = 0; count < OCTEP_PFVF_MBOX_TIMEOUT_MS; count++) {
		schedule_timeout_uninterruptible(timeout);
		reg_val = readq(mbox->mbox_write_reg);
		if (reg_val != cmd.u64) {
			rsp->u64 = reg_val;
			break;
		}
		count++;
	}
	if (count == OCTEP_PFVF_MBOX_TIMEOUT_MS) {
		dev_err(&oct->pdev->dev, "%s Timeout count:%d\n",
					 __func__, count);
		return OCTEP_PFVF_MBOX_STATUS_TIMEDOUT;
	}
	if (rsp->s.type != OCTEP_PFVF_MBOX_TYPE_RSP_ACK) {
		dev_err(&oct->pdev->dev, "%s Received Mbox NACK from PF\n", __func__);
		return OCTEP_PFVF_MBOX_STATUS_RCV_NACK;
	}
	rsp->u64 = reg_val;
	return 0;
}

int octep_vf_mbox_send_cmd(struct octep_vf_device *oct, union octep_pfvf_mbox_word cmd,
			   union octep_pfvf_mbox_word *rsp)
{
	int ring = 0, ret;
	struct octep_vf_mbox *mbox = oct->mbox[ring];

	if (!mbox) {
		dev_err(&oct->pdev->dev, "%s Mbox is not initialized\n", __func__);
		return OCTEP_PFVF_MBOX_STATUS_NOT_SETUP;
	}
	if (octep_vf_mbox_get_state(mbox) == OCTEP_PFVF_MBOX_STATE_BUSY) {
		dev_err(&oct->pdev->dev, "%s VF Mbox is in Busy state\n", __func__);
		return OCTEP_PFVF_MBOX_STATUS_BUSY;
	}
	octep_vf_mbox_set_state(mbox, OCTEP_PFVF_MBOX_STATE_BUSY);
	ret = __octep_vf_mbox_send_cmd(oct, cmd, rsp);
	octep_vf_mbox_set_state(mbox, OCTEP_PFVF_MBOX_STATE_IDLE);
	return ret;
}

int octep_vf_mbox_bulk_read(struct octep_vf_device *oct, enum octep_pfvf_mbox_opcode opcode,
			    u8 *data, int *size)
{
	union octep_pfvf_mbox_word cmd;
	union octep_pfvf_mbox_word rsp;
	int read_cnt, i = 0, ret;
	int data_len = 0, tmp_len = 0;
	int ring = 0;
	struct octep_vf_mbox *mbox = oct->mbox[ring];

	if (!mbox) {
		dev_err(&oct->pdev->dev, "%s Mbox is not initialized\n", __func__);
		return OCTEP_PFVF_MBOX_STATUS_NOT_SETUP;
	}
	if (octep_vf_mbox_get_state(mbox) == OCTEP_PFVF_MBOX_STATE_BUSY) {
		dev_err(&oct->pdev->dev, "%s VF Mbox is in Busy state\n", __func__);
		return OCTEP_PFVF_MBOX_STATUS_BUSY;
	}
	octep_vf_mbox_set_state(mbox, OCTEP_PFVF_MBOX_STATE_BUSY);
	cmd.u64 = 0;
	cmd.s_data.opcode = opcode;
	cmd.s_data.frag = 0;
	/* Send cmd to read data from PF */
	ret = __octep_vf_mbox_send_cmd(oct, cmd, &rsp);
	if (ret) {
		dev_err(&oct->pdev->dev, "%s send mbox cmd fail for data request\n", __func__);
		octep_vf_mbox_set_state(mbox, OCTEP_PFVF_MBOX_STATE_IDLE);
		return ret;
	}
	/*  PF sends the data length of requested CMD
	 *  in  ACK
	 */
	data_len = *((int32_t *)rsp.s_data.data);
	tmp_len = data_len;
	cmd.u64 = 0;
	rsp.u64 = 0;
	cmd.s_data.opcode = opcode;
	cmd.s_data.frag = 1;
	while (data_len) {
		ret = __octep_vf_mbox_send_cmd(oct, cmd, &rsp);
		if (ret) {
			dev_err(&oct->pdev->dev, "%s send mbox cmd fail for data request\n",
						  __func__);
			octep_vf_mbox_set_state(mbox, OCTEP_PFVF_MBOX_STATE_IDLE);
			mbox->mbox_data.data_index = 0;
			memset(mbox->mbox_data.recv_data, 0, OCTEP_PFVF_MBOX_MAX_DATA_BUF_SIZE);
			return ret;
		}
		if (data_len > OCTEP_PFVF_MBOX_MAX_DATA_SIZE) {
			data_len -= OCTEP_PFVF_MBOX_MAX_DATA_SIZE;
			read_cnt = OCTEP_PFVF_MBOX_MAX_DATA_SIZE;
		} else {
			read_cnt = data_len;
			data_len = 0;
		}
		for (i = 0; i < read_cnt; i++) {
			mbox->mbox_data.recv_data[mbox->mbox_data.data_index] =
				rsp.s_data.data[i];
			mbox->mbox_data.data_index++;
		}
		cmd.u64 = 0;
		rsp.u64 = 0;
		cmd.s_data.opcode = opcode;
		cmd.s_data.frag = 1;
	}
	memcpy(data, mbox->mbox_data.recv_data, tmp_len);
	*size = tmp_len;
	mbox->mbox_data.data_index = 0;
	memset(mbox->mbox_data.recv_data, 0, OCTEP_PFVF_MBOX_MAX_DATA_BUF_SIZE);
	octep_vf_mbox_set_state(mbox, OCTEP_PFVF_MBOX_STATE_IDLE);
	return 0;
}

int octep_vf_mbox_send_set_mtu(struct octep_vf_device *oct, int mtu)
{
	int frame_size = mtu + ETH_HLEN + ETH_FCS_LEN;
	union octep_pfvf_mbox_word cmd;
	union octep_pfvf_mbox_word rsp;
	int ret = 0;

	if (mtu < ETH_MIN_MTU || frame_size > ETH_MAX_MTU) {
		dev_err(&oct->pdev->dev, "%s MTU:%d MIN MTU:%d MAX MTU:%d\n",
			__func__, mtu, ETH_MIN_MTU, ETH_MAX_MTU);
		return -EINVAL;
	}

	cmd.u64 = 0;
	cmd.s_set_mtu.opcode = OCTEP_PFVF_MBOX_CMD_SET_MTU;
	cmd.s_set_mtu.mtu = mtu;

	ret = octep_vf_mbox_send_cmd(oct, cmd, &rsp);
	if (ret) {
		dev_err(&oct->pdev->dev, "%s Mbox send fail ret value:%d\n",
			__func__, ret);
		return ret;
	}
	if (rsp.s_set_mtu.type != OCTEP_PFVF_MBOX_TYPE_RSP_ACK) {
		dev_err(&oct->pdev->dev, "%s Received Mbox NACK from PF for MTU:%d\n",
			__func__, mtu);
		return -EINVAL;
	}

	return 0;
}

int octep_vf_mbox_set_mac_addr(struct octep_vf_device *oct, char *mac_addr)
{
	union octep_pfvf_mbox_word cmd;
	union octep_pfvf_mbox_word rsp;
	int i, ret;

	cmd.u64 = 0;
	cmd.s_set_mac.opcode = OCTEP_PFVF_MBOX_CMD_SET_MAC_ADDR;
	for (i = 0; i < ETH_ALEN; i++)
		cmd.s_set_mac.mac_addr[i] = mac_addr[i];
	ret = octep_vf_mbox_send_cmd(oct, cmd, &rsp);
	if (ret) {
		dev_err(&oct->pdev->dev, "%s Mbox send fail ret value:%d\n",
			__func__, ret);
		return ret;
	}
	if (rsp.s_set_mac.type != OCTEP_PFVF_MBOX_TYPE_RSP_ACK) {
		dev_err(&oct->pdev->dev, "%s received NACK\n", __func__);
		return -EINVAL;
	}
	return 0;
}

int octep_vf_mbox_get_mac_addr(struct octep_vf_device *oct, char *mac_addr)
{
	union octep_pfvf_mbox_word cmd;
	union octep_pfvf_mbox_word rsp;
	int i, ret;

	cmd.u64 = 0;
	cmd.s_set_mac.opcode = OCTEP_PFVF_MBOX_CMD_GET_MAC_ADDR;
	ret = octep_vf_mbox_send_cmd(oct, cmd, &rsp);
	if (ret) {
		dev_err(&oct->pdev->dev, "%s Mbox send fail ret value:%d\n",
			__func__, ret);
		return ret;
	}
	if (rsp.s_set_mac.type != OCTEP_PFVF_MBOX_TYPE_RSP_ACK) {
		dev_err(&oct->pdev->dev, "%s received NACK\n", __func__);
		return -EINVAL;
	}
	for (i = 0; i < ETH_ALEN; i++)
		mac_addr[i] = rsp.s_set_mac.mac_addr[i];
	return 0;
}
