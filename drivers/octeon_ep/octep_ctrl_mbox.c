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
#include "octep_ctrl_mbox.h"

/* Timeout in msecs for message response */
#define OCTEP_CTRL_MBOX_MSG_TIMEOUT_MS          100
/* Time in msecs to wait for message response */
#define OCTEP_CTRL_MBOX_MSG_WAIT_MS         10

/* Size of mbox info in bytes */
#define OCTEP_CTRL_MBOX_INFO_SZ             256
/* Size of mbox host to fw queue info in bytes */
#define OCTEP_CTRL_MBOX_H2FQ_INFO_SZ            16
/* Size of mbox fw to host queue info in bytes */
#define OCTEP_CTRL_MBOX_F2HQ_INFO_SZ            16

#define OCTEP_CTRL_MBOX_TOTAL_INFO_SZ   (OCTEP_CTRL_MBOX_INFO_SZ + \
					 OCTEP_CTRL_MBOX_H2FQ_INFO_SZ + \
					 OCTEP_CTRL_MBOX_F2HQ_INFO_SZ)

#define OCTEP_CTRL_MBOX_INFO_MAGIC_NUM(m)   (m)
#define OCTEP_CTRL_MBOX_INFO_BARMEM_SZ(m)   ((m) + 8)
#define OCTEP_CTRL_MBOX_INFO_HOST_VERSION(m)    ((m) + 16)
#define OCTEP_CTRL_MBOX_INFO_HOST_STATUS(m) ((m) + 24)
#define OCTEP_CTRL_MBOX_INFO_FW_VERSION(m)  ((m) + 136)
#define OCTEP_CTRL_MBOX_INFO_FW_STATUS(m)   ((m) + 144)

#define OCTEP_CTRL_MBOX_H2FQ_INFO(m)    ((m) + OCTEP_CTRL_MBOX_INFO_SZ)
#define OCTEP_CTRL_MBOX_H2FQ_PROD(m)    (OCTEP_CTRL_MBOX_H2FQ_INFO(m))
#define OCTEP_CTRL_MBOX_H2FQ_CONS(m)    ((OCTEP_CTRL_MBOX_H2FQ_INFO(m)) + 4)
#define OCTEP_CTRL_MBOX_H2FQ_SZ(m)  ((OCTEP_CTRL_MBOX_H2FQ_INFO(m)) + 8)

#define OCTEP_CTRL_MBOX_F2HQ_INFO(m)    ((m) + \
					 OCTEP_CTRL_MBOX_INFO_SZ + \
					 OCTEP_CTRL_MBOX_H2FQ_INFO_SZ)
#define OCTEP_CTRL_MBOX_F2HQ_PROD(m)    (OCTEP_CTRL_MBOX_F2HQ_INFO(m))
#define OCTEP_CTRL_MBOX_F2HQ_CONS(m)    ((OCTEP_CTRL_MBOX_F2HQ_INFO(m)) + 4)
#define OCTEP_CTRL_MBOX_F2HQ_SZ(m)  ((OCTEP_CTRL_MBOX_F2HQ_INFO(m)) + 8)

static const u32 mbox_hdr_sz = sizeof(union octep_ctrl_mbox_msg_hdr);

static u32 octep_ctrl_mbox_circq_inc(u32 index, u32 inc, u32 sz)
{
	return (index + inc) % sz;
}

static u32 octep_ctrl_mbox_circq_space(u32 pi, u32 ci, u32 sz)
{
	return sz - (abs(pi - ci) % sz);
}

static u32 octep_ctrl_mbox_circq_depth(u32 pi, u32 ci, u32 sz)
{
	return (abs(pi - ci) % sz);
}

int octep_ctrl_mbox_init(void *dev, struct octep_ctrl_mbox *mbox)
{
	u64 magic_num, status, fw_versions;
	struct octep_device *oct = (struct octep_device *)dev;

	if (!mbox)
		return -EINVAL;

	if (!mbox->barmem) {
		dev_err(oct->pdev, "octep_ctrl_mbox : Invalid barmem %u\n"
			, mbox->barmem);
		return -EINVAL;
	}

	magic_num = octep_read_bar2_csr64(oct, 
					  (OCTEP_CTRL_MBOX_INFO_MAGIC_NUM(mbox->barmem)));
	if (magic_num != OCTEP_CTRL_MBOX_MAGIC_NUMBER) {
		dev_err(oct->pdev, "octep_ctrl_mbox : Invalid magic number %lx\n",
			magic_num);
		return -EINVAL;
	}

	status = octep_read_bar2_csr64(oct, (OCTEP_CTRL_MBOX_INFO_FW_STATUS(mbox->barmem)));
	if (status != OCTEP_CTRL_MBOX_STATUS_READY) {
		dev_err(oct->pdev, "octep_ctrl_mbox : Firmware is not ready.\n");
		return -EINVAL;
	}

	fw_versions = octep_read_bar2_csr64(oct, 
					    (OCTEP_CTRL_MBOX_INFO_FW_VERSION(mbox->barmem)));
	mbox->min_fw_version = ((fw_versions & 0xffffffff00000000ull) >> 32);
	mbox->max_fw_version = (fw_versions & 0xffffffff);
	mbox->barmem_sz = octep_read_bar2_csr32(oct, 
						(OCTEP_CTRL_MBOX_INFO_BARMEM_SZ(mbox->barmem)));

	octep_write_bar2_csr64(oct, OCTEP_CTRL_MBOX_INFO_HOST_STATUS(mbox->barmem),
			       OCTEP_CTRL_MBOX_STATUS_INIT);

	mbox->h2fq.sz = octep_read_bar2_csr32(oct,
					      (OCTEP_CTRL_MBOX_H2FQ_SZ(mbox->barmem)));
	mbox->h2fq.hw_prod = OCTEP_CTRL_MBOX_H2FQ_PROD(mbox->barmem);
	mbox->h2fq.hw_cons = OCTEP_CTRL_MBOX_H2FQ_CONS(mbox->barmem);
	mbox->h2fq.hw_q = mbox->barmem + OCTEP_CTRL_MBOX_TOTAL_INFO_SZ;

	mbox->f2hq.sz = octep_read_bar2_csr32(oct,
					      (OCTEP_CTRL_MBOX_F2HQ_SZ(mbox->barmem)));
	mbox->f2hq.hw_prod = OCTEP_CTRL_MBOX_F2HQ_PROD(mbox->barmem);
	mbox->f2hq.hw_cons = OCTEP_CTRL_MBOX_F2HQ_CONS(mbox->barmem);
	mbox->f2hq.hw_q = mbox->barmem +
		OCTEP_CTRL_MBOX_TOTAL_INFO_SZ +
		mbox->h2fq.sz;

	octep_write_bar2_csr64(oct,
			       OCTEP_CTRL_MBOX_INFO_HOST_VERSION(mbox->barmem), mbox->version);
	/* ensure ready state is seen after everything is initialized */
	wmb();
	octep_write_bar2_csr64(oct, OCTEP_CTRL_MBOX_INFO_HOST_STATUS(mbox->barmem),
			       OCTEP_CTRL_MBOX_STATUS_READY);

	return 0;
}

static int
octep_write_mbox_data(struct octep_device *oct, struct octep_ctrl_mbox_q *q, 
		      u32 *pi, u32 ci, void *buf, u32 w_sz, bus_space_tag_t tag,
		      bus_space_handle_t handle)
{
	u32 cp_sz;
	u8 *qbuf;

	/* Assumption: Caller has ensured enough write space */
	qbuf = (u8 *)q->hw_q + *pi;
	if (*pi < ci) {
		/* copy entire w_sz */
		bus_space_write_region_1(tag, handle, (bus_size_t)qbuf, buf, w_sz);
		*pi = octep_ctrl_mbox_circq_inc(*pi, w_sz, q->sz);
	} else {
		/* copy up to end of queue */
		cp_sz = min((q->sz - *pi), w_sz);
		bus_space_write_region_1(tag, handle, (bus_size_t)qbuf, buf, cp_sz);
		w_sz -= cp_sz;
		*pi = octep_ctrl_mbox_circq_inc(*pi, cp_sz, q->sz);
		if (w_sz) {
			/* roll over and copy remaining w_sz */
			buf = (void *)((u8 *)buf + cp_sz);
			qbuf = (u8 *)q->hw_q + *pi;
			bus_space_write_region_1(tag, handle, (bus_size_t)qbuf, buf, w_sz);
			*pi = octep_ctrl_mbox_circq_inc(*pi, w_sz, q->sz);
		}
	}

	return 0;
}

int
octep_ctrl_mbox_send(void *dev, struct octep_ctrl_mbox *mbox,
		     struct octep_ctrl_mbox_msg *msg)
{
	struct octep_device *oct = (struct octep_device *)dev;
	struct octep_ctrl_mbox_msg_buf *sg;
	struct octep_ctrl_mbox_q *q;
	u32 pi, ci, buf_sz, w_sz;
	int s;
	u64 status;

	if (!mbox || !msg)
	{
		return -EINVAL;
	}

	status = octep_read_bar2_csr64(oct,(OCTEP_CTRL_MBOX_INFO_FW_STATUS(mbox->barmem)));
	if (status != OCTEP_CTRL_MBOX_STATUS_READY) {
		device_printf(oct->pdev, "octep_ctrl_mbox : Firmware is not ready.\n");
		return -EIO;
	}

	mutex_lock(&mbox->h2fq_lock);
	q = &mbox->h2fq;
	pi = octep_read_bar2_csr32(oct,(q->hw_prod));
	ci = octep_read_bar2_csr32(oct,(q->hw_cons));
	if (pi == 0xFFFFFFFFU || ci == 0xFFFFFFFFU ) {
		mutex_unlock(&mbox->f2hq_lock);
		return -EIO;
	}

	if (octep_ctrl_mbox_circq_space(pi, ci, q->sz) < (msg->hdr.s.sz + mbox_hdr_sz)) {
		mutex_unlock(&mbox->f2hq_lock);
		return -EAGAIN;
	}

	octep_write_mbox_data(oct, q, &pi, ci, (void *)&msg->hdr, mbox_hdr_sz,
			      oct->mem_bus_space[2].tag, oct->mem_bus_space[2].handle);
	buf_sz = msg->hdr.s.sz;
	for (s = 0; ((s < msg->sg_num) && (buf_sz > 0)); s++) {
		sg = &msg->sg_list[s];
		w_sz = (sg->sz <= buf_sz) ? sg->sz : buf_sz;
		octep_write_mbox_data(oct, q, &pi, ci, sg->msg, w_sz,
				      oct->mem_bus_space[2].tag, oct->mem_bus_space[2].handle );
		buf_sz -= w_sz;
	}

	octep_write_bar2_csr32(oct, q->hw_prod, pi);
	mutex_unlock(&mbox->h2fq_lock);

	return 0;
}

static int
octep_read_mbox_data(struct octep_device *oct, struct octep_ctrl_mbox_q *q,
		     u32 pi, u32 *ci, void *buf, u32 r_sz, bus_space_tag_t tag,
		     bus_space_handle_t handle)
{
	u32 cp_sz;
	u8 *qbuf;

	/* Assumption: Caller has ensured enough read space */
	qbuf = (u8 *)q->hw_q + *ci;
	if (*ci < pi) {
		/* copy entire r_sz */
		bus_space_read_region_1(tag, handle, (bus_size_t)qbuf, buf, r_sz);
		*ci = octep_ctrl_mbox_circq_inc(*ci, r_sz, q->sz);
	} else {
		/* copy up to end of queue */
		cp_sz = min((q->sz - *ci), r_sz);
		bus_space_read_region_1(tag, handle, (bus_size_t)qbuf, buf, cp_sz);
		r_sz -= cp_sz;
		*ci = octep_ctrl_mbox_circq_inc(*ci, cp_sz, q->sz);
		if (r_sz) {
			/* roll over and copy remaining r_sz */
			buf = (void *)((u8 *)buf + cp_sz);
			qbuf = (u8 *)q->hw_q + *ci;
			bus_space_read_region_1(tag, handle, (bus_size_t)qbuf, buf, r_sz);
			*ci = octep_ctrl_mbox_circq_inc(*ci, r_sz, q->sz);
		}
	}

	return 0;
}


int
octep_ctrl_mbox_recv(void *dev, struct octep_ctrl_mbox *mbox,
		     struct octep_ctrl_mbox_msg *msg)
{
	struct octep_device *oct = (struct octep_device *)dev;
	struct octep_ctrl_mbox_msg_buf *sg;
	u32 pi, ci, r_sz, buf_sz, q_depth;
	struct octep_ctrl_mbox_q *q;
	int s;
	u64 status;

	if (!mbox || !msg)
		return -EINVAL;


	status = octep_read_bar2_csr64(oct,(OCTEP_CTRL_MBOX_INFO_FW_STATUS(mbox->barmem)));
	if (status != OCTEP_CTRL_MBOX_STATUS_READY) {
		device_printf(oct->pdev, "octep_ctrl_mbox : Firmware is not ready.\n");
		return -EIO;
	}

	mutex_lock(&mbox->f2hq_lock);
	q = &mbox->f2hq;
	pi = octep_read_bar2_csr32(oct,(q->hw_prod));
	ci = octep_read_bar2_csr32(oct,(q->hw_cons));
	if (pi == 0xFFFFFFFFU || ci == 0xFFFFFFFFU) {
		mutex_unlock(&mbox->f2hq_lock);
		return -EIO;
	}

	q_depth = octep_ctrl_mbox_circq_depth(pi, ci, q->sz);
	if (q_depth < mbox_hdr_sz) {
		mutex_unlock(&mbox->f2hq_lock);
		return -EAGAIN;
	}

	octep_read_mbox_data(oct, q, pi, &ci, (void *)&msg->hdr, mbox_hdr_sz,
			     oct->mem_bus_space[2].tag, oct->mem_bus_space[2].handle);
	buf_sz = msg->hdr.s.sz;

	for (s = 0; ((s < msg->sg_num) && (buf_sz > 0)); s++) {
		sg = &msg->sg_list[s];
		r_sz = (sg->sz <= buf_sz) ? sg->sz : buf_sz;
		octep_read_mbox_data(oct, q, pi, &ci, sg->msg, r_sz,
				     oct->mem_bus_space[2].tag, oct->mem_bus_space[2].handle);
		buf_sz -= r_sz;
	}
	octep_write_bar2_csr32(oct, q->hw_cons, ci);
	mutex_unlock(&mbox->f2hq_lock);

	status = octep_read_bar2_csr64(oct,(OCTEP_CTRL_MBOX_INFO_FW_STATUS(mbox->barmem)));
	if (status != OCTEP_CTRL_MBOX_STATUS_READY) {
		device_printf(oct->pdev, "octep_ctrl_mbox : Firmware is not ready.\n");
		return -EIO;
	}

	return 0;
}

int
octep_ctrl_mbox_uninit(void *dev, struct octep_ctrl_mbox *mbox)
{
	struct octep_device *oct = (struct octep_device *)dev;

	if (!mbox)
		return -EINVAL;
	if (!mbox->barmem)
		return -EINVAL;

	octep_write_bar2_csr64(oct, OCTEP_CTRL_MBOX_INFO_HOST_VERSION(mbox->barmem), 0);
	octep_write_bar2_csr64(oct, OCTEP_CTRL_MBOX_INFO_HOST_STATUS(mbox->barmem),
			       OCTEP_CTRL_MBOX_STATUS_INVALID);
	/* ensure uninit state is written before uninitialization */
	wmb();

	mutex_destroy(&mbox->h2fq_lock);
	mutex_destroy(&mbox->f2hq_lock);

	dev_info(oct->pdev, "Octep ctrl mbox : Uninit successful.\n");

	return 0;
}
