/* */
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

static void octep_vf_get_link_status(struct octep_device *oct, u32 vf_id,
				      union octep_vf_mbox_word cmd,
				      union octep_vf_mbox_word *rsp)
{
	rsp->s_get_link.id = cmd.s_get_link.id;
	rsp->s_get_link.type = OCTEP_VF_MBOX_TYPE_RSP_ACK;
	rsp->s_get_link.link_status =  OCTEP_VF_LINK_STATUS_UP;
	rsp->s_get_link.link_speed = OCTEP_VF_LINK_SPEED_10000;
	rsp->s_get_link.duplex = OCTEP_VF_LINK_FULL_DUPLEX;
	rsp->s_get_link.autoneg = OCTEP_VF_LINK_AUTONEG;
	dev_info(&oct->pdev->dev, "%s cmd vf %d cmd_id %d\n",
		  __func__, vf_id, cmd.s_get_link.id);
}

static void octep_vf_set_mtu(struct octep_device *oct, u32 vf_id,
			      union octep_vf_mbox_word cmd,
			      union octep_vf_mbox_word *rsp)
{
	rsp->s_set_mtu.id = cmd.s_set_mtu.id;
	rsp->s_set_mtu.type = OCTEP_VF_MBOX_TYPE_RSP_ACK;
	dev_info(&oct->pdev->dev, "%s mtu cmd vf %d id %d mtu is %d\n",
		  __func__, vf_id, cmd.s_set_mtu.id, (int)cmd.s_set_mtu.mtu);
}

static void octep_vf_set_mac_addr(struct octep_device *oct,  u32 vf_id,
				   union octep_vf_mbox_word cmd,
				   union octep_vf_mbox_word *rsp)
{
	u8 vf_mac_addr[6];
	int i;

	for (i = 0; i < OCTEP_VF_MBOX_MAX_DATA_SIZE; i++) {
		vf_mac_addr[i] = cmd.s_set_mac.mac_addr[i];
		oct->vf_info[vf_id].mac_addr[i] = cmd.s_set_mac.mac_addr[i];
	}

	rsp->s_set_mac.id = cmd.s_set_mac.id;
	rsp->s_set_mac.type = OCTEP_VF_MBOX_TYPE_RSP_ACK;
	dev_info(&oct->pdev->dev, "%s %pM\n",  __func__, oct->vf_info[vf_id].mac_addr);
}

static void octep_vf_get_mac_addr(struct octep_device *oct,  u32 vf_id,
				   union octep_vf_mbox_word cmd,
				   union octep_vf_mbox_word *rsp)
{
	int i;

	rsp->s_set_mac.id = cmd.s_set_mac.id;
	rsp->s_set_mac.type = OCTEP_VF_MBOX_TYPE_RSP_ACK;
	for (i = 0; i < OCTEP_VF_MBOX_MAX_DATA_SIZE; i++) {
		rsp->s_set_mac.mac_addr[i] = oct->vf_info[vf_id].mac_addr[i];
	}
	dev_info(&oct->pdev->dev, "%s vf_info: %pM\n",  __func__, oct->vf_info[vf_id].mac_addr);
}


int octep_setup_vf_mbox(struct octep_device *oct)
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
		INIT_WORK(&oct->mbox[ring]->wk.work, octep_vf_mbox_work);
		oct->mbox[ring]->wk.ctxptr = oct->mbox[i];

		if (oct->mbox[i] == NULL)
			oct->mbox[ring]->wk.ctxptr = oct->mbox[ring];

		oct->mbox[ring]->oct = oct;
		oct->mbox[ring]->vf_id = i;
		oct->hw_ops.setup_mbox_regs(oct, ring);
		dev_info(&oct->pdev->dev, "%s num_vfs:%d ring:%d\n",
			 __func__, num_vfs, ring);
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

void octep_delete_vf_mbox(struct octep_device *oct)
{
	int rings_per_vf = oct->conf->sriov_cfg.max_rings_per_vf;
	int num_vfs = oct->conf->sriov_cfg.active_vfs;
	int i = 0, ring = 0, vf_srn = 0;

	for (i = 0; i < num_vfs; i++) {
		ring  = vf_srn + rings_per_vf * i;
		if (work_pending(&oct->mbox[ring]->wk.work))
			cancel_work_sync(&oct->mbox[ring]->wk.work);
		vfree(oct->mbox[ring]);
		oct->mbox[ring] = NULL;
	}
	dev_info(&oct->pdev->dev, "%s OCTEON_EP: freed mbox struct.\n",  __func__);
}

static void octep_vf_pf_get_data(struct octep_device *oct,
				  struct octep_mbox *mbox, int vf_id,
				  union octep_vf_mbox_word cmd,
				  union octep_vf_mbox_word *rsp)
{
	int length = 0;
	int i = 0;

	rsp->s_data.id = cmd.s_data.id;
	rsp->s_data.type = OCTEP_VF_MBOX_TYPE_RSP_ACK;
	dev_info(&oct->pdev->dev, "%s received\n", __func__);

	if (cmd.s_data.frag != OCTEP_VF_MBOX_MORE_FRAG_FLAG) {
		mbox->config_data_index = 0;
		memset(mbox->config_data, 0, MAX_VF_PF_MBOX_DATA_SIZE);
		/* Based on the OPCODE CMD the PF driver
		 * specific API should be called to fetch
		 * the requested data
		 */
		*((int32_t *)rsp->s_data.data) = mbox->message_len;
		dev_dbg(&oct->pdev->dev, "%s msg len %d:\n", __func__,
			mbox->message_len);
		return;
	}

	if (mbox->message_len > OCTEP_VF_MBOX_MAX_DATA_SIZE) {
		length = OCTEP_VF_MBOX_MAX_DATA_SIZE;
		dev_dbg(&oct->pdev->dev,
			"%s more to send data to VF\n", __func__);
	} else {
		length = mbox->message_len;
		dev_dbg(&oct->pdev->dev,
			"%s last to send data to VF\n", __func__);
	}

	mbox->message_len -= length;

	for (i = 0; i < length; i++) {
		rsp->s_data.data[i] =
			mbox->config_data[mbox->config_data_index];
		mbox->config_data_index++;
	}
}

static void octep_vf_pf_config_data(struct octep_device *oct,
				     struct octep_mbox *mbox,
				     int vf_id, union octep_vf_mbox_word cmd,
				     union octep_vf_mbox_word *rsp)
{
	int length;
	int i = 0;

	rsp->s_data.id = cmd.s_data.id;
	rsp->s_data.type = OCTEP_VF_MBOX_TYPE_RSP_ACK;
	dev_info(&oct->pdev->dev, "%s received\n", __func__);

	if ((cmd.s_data.frag == OCTEP_VF_MBOX_MORE_FRAG_FLAG) &&
	    (mbox->message_len == 0)) {
		length = *((int32_t *)cmd.s_data.data);
		mbox->message_len = length;
		mbox->config_data_index = 0;
		memset(mbox->config_data, 0, MAX_VF_PF_MBOX_DATA_SIZE);
		return;
	}

	if (cmd.s_data.frag == OCTEP_VF_MBOX_MORE_FRAG_FLAG) {
		for (i = 0; i < OCTEP_VF_MBOX_MAX_DATA_SIZE; i++) {
			mbox->config_data[mbox->config_data_index]
				= cmd.s_data.data[i];
			mbox->config_data_index++;
		}
		mbox->message_len -= OCTEP_VF_MBOX_MAX_DATA_SIZE;
	} else {
		for (i = 0; i < mbox->message_len; i++) {
			mbox->config_data[mbox->config_data_index]
				= cmd.s_data.data[i];
			mbox->config_data_index++;
		}
		/* Calls OPCODE specific configuration handler by passing
		 * received config data as input argument.
		 */
		mbox->config_data_index = 0;
		mbox->message_len = 0;
		memset(mbox->config_data, 0, MAX_VF_PF_MBOX_DATA_SIZE);
	}
}

void octep_vf_mbox_work(struct work_struct *work)
{
	struct octep_vf_mbox_wk *wk = container_of(work, struct octep_vf_mbox_wk, work);
	union octep_vf_mbox_word cmd = { 0 };
	union octep_vf_mbox_word rsp = { 0 };
	struct octep_mbox *mbox = NULL;
	struct octep_device *oct = NULL;
	int vf_id;

	mbox = (struct octep_mbox *)wk->ctxptr;
	oct = (struct octep_device *)mbox->oct;
	vf_id = mbox->vf_id;

	mutex_lock(&mbox->lock);
	cmd.u64 = readq(mbox->vf_pf_data_reg);
	rsp.u64 = 0;


	if (cmd.s.version != OCTEP_PF_MBOX_VERSION) {
		dev_info(&oct->pdev->dev,
			"%s mbox version mis match vf version %d pf version %d\n",
			__func__, cmd.s.version, OCTEP_PF_MBOX_VERSION);
		rsp.s.type = OCTEP_VF_MBOX_TYPE_RSP_NACK;
		goto done;
	}

	switch (cmd.s.opcode) {
	case OCTEP_VF_MBOX_CMD_GET_LINK:
		octep_vf_get_link_status(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_VF_MBOX_CMD_SET_MTU:
		octep_vf_set_mtu(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_VF_MBOX_CMD_SET_MAC_ADDR:
		octep_vf_set_mac_addr(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_VF_MBOX_CMD_GET_MAC_ADDR:
		octep_vf_get_mac_addr(oct, vf_id, cmd, &rsp);
		break;
	case OCTEP_VF_MBOX_CMD_BULK_SEND:
		octep_vf_pf_config_data(oct, mbox, vf_id, cmd, &rsp);
		break;
	case OCTEP_VF_MBOX_CMD_BULK_GET:
		octep_vf_pf_get_data(oct, mbox, vf_id, cmd, &rsp);
		break;
	default:
		dev_info(&oct->pdev->dev, "%s OCTEP_VF_MBOX_TYPE_RSP_NACK\n", __func__);
		rsp.s.type = OCTEP_VF_MBOX_TYPE_RSP_NACK;
		break;
	}
done:
	writeq(rsp.u64, mbox->vf_pf_data_reg);
	mutex_unlock(&mbox->lock);
}
