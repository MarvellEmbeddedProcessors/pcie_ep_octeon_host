/* Copyright (c) 2020 Marvell.
 * SPDX-License-Identifier: GPL-2.0
 */

/* Host PHC driver using PTP timer on Octeon EP device
 */

#include <linux/cpumask.h>
#include <linux/spinlock.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/circ_buf.h>
#include <linux/version.h>
#include <linux/sched/signal.h>
#include <linux/ptp_clock_kernel.h>

#include "octeon_main.h"
#include "cavium_release.h"
#include "octeon_hw.h"

int startup_set_ptp = 0;
module_param(startup_set_ptp, int, 0);
MODULE_PARM_DESC(startup_set_ptp, "Flag to set PTP clock to host clock at startup for testing");

void __iomem *nwa_bar0_internal_addr;

uint64_t octeon_pci_bar4_read64(octeon_device_t *oct_dev, int baridx, uint64_t bar_offset);
void octeon_pci_bar4_write64(octeon_device_t *oct_dev, int baridx, uint64_t bar_offset, uint64_t val);

int octeon_chip_specific_setup(octeon_device_t *oct_dev);

OCTEON_DRIVER_STATUS octeon_state;
#ifndef  DEFINE_PCI_DEVICE_TABLE
#define  DEFINE_PCI_DEVICE_TABLE(octeon_ep_phc_pci_tbl) struct pci_device_id octeon_ep_phc_pci_tbl[]
#endif

static DEFINE_PCI_DEVICE_TABLE(octeon_ep_phc_pci_tbl) = {
	/* Same devid for all PHC PFs, both cn9xxx and cn10k */
	{OCTEON_VENDOR_ID, 0xEF00, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0}
};

extern int octeon_ep_phc_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
extern void octeon_ep_phc_remove(struct pci_dev *pdev);
extern int octeon_ep_phc_sriov_configure(struct pci_dev *dev, int num_vfs);
static struct pci_driver octeon_ep_phc_pci_driver = {
	.name = "Octeon EP PHC",
	.id_table = octeon_ep_phc_pci_tbl,
	.probe = octeon_ep_phc_probe,
	.remove = octeon_ep_phc_remove,
};





/*
 * PTP clock operations
 * This driver only supports read-only operations on a PTP hardware clock
 * source that is owned and managed by software running on the Octeon
 * PCIe EP device.
 */
#ifdef PHC_DEBUG
static u64 prev_offset;
#endif
static int oct_ep_ptp_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct oct_ep_ptp_clock *ep_clk;
	struct timespec64 tspec;
	uint64_t ns = 0;

	ep_clk = container_of(ptp, struct oct_ep_ptp_clock, caps);
	preempt_disable_notrace();
	ns = octeon_pci_bar4_read64(ep_clk->oct_dev, CN93XX_MIO_PTP_BAR4_REGION,
			       CN93XX_MIO_PTP_CKOUT_THRESH_HI_OFFSET);
#ifdef PHC_DEBUG
	if (prev_offset && prev_offset != ns) {
	    printk("OCT_PHC: offset changed, prev: 0x%llx, current: 0x%llx\n",
		   (unsigned long long)prev_offset, (unsigned long long)ns);

	    prev_offset = ns;
	}
	if (!prev_offset)
	    prev_offset = ns;
#endif

	ns += octeon_pci_bar4_read64(ep_clk->oct_dev, CN93XX_MIO_PTP_BAR4_REGION,
			       CN93XX_MIO_PTP_CLOCK_HI_OFFSET);

	preempt_enable_notrace();
	tspec = ns_to_timespec64(ns);

	memcpy(ts, &tspec, sizeof(struct timespec64));
	return 0;
}
static int oct_ep_ptp_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	/* Nothing to do here, PTP hardware is enabled by EP */
	return 0;
}
static int oct_ep_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	return -ENOTSUPP;
}

static int oct_ep_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	return -ENOTSUPP;
}

static int oct_ep_ptp_settime(struct ptp_clock_info *ptp,
			   const struct timespec64 *ts)
{
	return -ENOTSUPP;
}


static const struct ptp_clock_info oct_ep_ptp_caps = {
	.owner		= THIS_MODULE,
	.name		= "Octeon EP PHC",
	.max_adj	= 1,
	.n_ext_ts	= 0,
	.n_pins		= 0,
	.pps		= 0,
	.adjfreq	= oct_ep_ptp_adjfreq,
	.adjtime	= oct_ep_ptp_adjtime,
	.gettime64	= oct_ep_ptp_gettime,
	.settime64	= oct_ep_ptp_settime,
	.enable		= oct_ep_ptp_enable,
};

static int __init phc_init(void)
{
	int ret;
	printk("phc_init\n");


	ret = pci_register_driver(&octeon_ep_phc_pci_driver);
	if (ret < 0) {
		cavium_error("OCT_PHC: pci_module_init() returned %d\n", ret);
		cavium_error
		    ("OCT_PHC: Your kernel may not be configured for hotplug\n");
		cavium_error("        and no Octeon devices were detected\n");
		return ret;
	}
	return 0;
}

static void __exit phc_exit(void)
{
	pci_unregister_driver(&octeon_ep_phc_pci_driver);
}

#define FW_STATUS_VSEC_ID 0xA3
#define FW_STATUS_READY 1
static u8 oct_get_fw_ready_status(octeon_device_t *oct_dev)
{
	u32 pos = 0;
	u16 vsec_id;
	u8 status = 0;

	while ((pos = pci_find_next_ext_capability(oct_dev->pci_dev, pos,
						   PCI_EXT_CAP_ID_VNDR))) {
		pci_read_config_word(oct_dev->pci_dev, pos + 4, &vsec_id);
		if (vsec_id == FW_STATUS_VSEC_ID) {
			pci_read_config_byte(oct_dev->pci_dev, (pos + 8), &status);
			cavium_print_msg("OCT_PHC[%d]:fw ready status %u\n",
					 oct_dev->octeon_id, status);
			return status;
		}
	}
	return 0;
}

/* OS-specific initialization for each Octeon device. */
static int octeon_pci_os_setup(octeon_device_t *oct_dev)
{

	/* setup PCI stuff first */
	if (pci_enable_device(oct_dev->pci_dev)) {
		cavium_error("OCT_PHC[%d]: pci_enable_device failed\n",
			     oct_dev->octeon_id);
		return 1;
	}

	/* Octeon device supports DMA into a 64-bit space */
	if (dma_set_mask_and_coherent(&oct_dev->pci_dev->dev, DMA_BIT_MASK(64))) {
		cavium_error("OCT_PHC[%d]: Unexpected DMA device capability\n",
			     oct_dev->octeon_id);
		return 1;
	}

	/* Enable PCI DMA Master. */
	pci_set_master(oct_dev->pci_dev);

	return 0;
}
/* Device initialization for each Octeon device. */
int octeon_device_init(octeon_device_t *oct_dev)
{
	int ret;
	octeon_poll_ops_t poll_ops;

	cavium_atomic_set(&oct_dev->status, OCT_DEV_BEGIN_STATE);

	/* Enable access to the octeon device and make its DMA capability
	   known to the OS. */
	if (octeon_pci_os_setup(oct_dev))
		return 1;

	ret  = octeon_chip_specific_setup(oct_dev);
	/* Identify the Octeon type and map the BAR address space. */
	if (ret == -1) {
		cavium_error("OCT_PHC[%d]: Chip specific setup failed\n",
			 oct_dev->octeon_id);
		return 1;
	}

	cavium_print_msg(" Chip specific setup completed\n");
	cavium_atomic_set(&oct_dev->status, OCT_DEV_PCI_MAP_DONE);

	cavium_spin_lock_init(&oct_dev->oct_lock);

	cavium_atomic_set(&oct_dev->status, OCT_DEV_DISPATCH_INIT_DONE);

#if 0
	/* Setup the /proc entries for this octeon device. */
	cavium_init_proc(oct_dev);
#endif

	oct_dev->app_mode = CVM_DRV_INVALID_APP;

	cavium_atomic_set(&oct_dev->status, OCT_DEV_HOST_OK);

	cavium_atomic_set(&oct_dev->hostfw_hs_state, HOSTFW_HS_INIT);

	return 0;
}
static void octeon_device_init_work(struct work_struct *work)
{
	octeon_device_t *oct_dev;
	struct cavium_delayed_wq *wq;
	u8 status;
	uint64_t tmp64;

	wq = container_of(work, struct cavium_delayed_wq, wk.work.work);
	oct_dev = (octeon_device_t *)wq->wk.ctxptr;

	cavium_atomic_set(&oct_dev->status, OCT_DEV_CHECK_FW);
	while (true) {
		status = oct_get_fw_ready_status(oct_dev);
		if (status == FW_STATUS_READY)
			break;

		schedule_timeout_interruptible(HZ * 1);
		if (cavium_atomic_read(&oct_dev->status) > OCT_DEV_RUNNING) {
			cavium_atomic_set(&oct_dev->status, OCT_DEV_STATE_INVALID);
			cavium_print_msg("OCT_PHC[%d]: Stopping firmware ready work.\n",
					 oct_dev->octeon_id);
			return;
		}
	}

	if (octeon_device_init(oct_dev)) {
		cavium_print_msg("OCT_PHC[%d]: ERROR: Octeon driver failed to load.\n",
				 oct_dev->octeon_id);
		return;
	}

	octeon_state = OCT_DRV_ACTIVE;

	oct_dev->oct_ep_ptp_clock->caps = oct_ep_ptp_caps;
	oct_dev->oct_ep_ptp_clock->oct_dev = oct_dev;

	oct_dev->oct_ep_ptp_clock->ptp_clock = ptp_clock_register(&oct_dev->oct_ep_ptp_clock->caps, NULL);
	if (!oct_dev->oct_ep_ptp_clock->ptp_clock) {
	    cavium_print_msg("OCT_PHC[%d]: ERROR: Octeon PHC driver failed to load.\n",
			     oct_dev->octeon_id);
	    return;
	}
	if (startup_set_ptp) {
	    /*
	     * For cases where the PTP clock is not set to an external
	     * reference, we want to set it to match the host clock at
	     * startup.  Without this the PTP clock will be off by decades,
	     * and phc2sys will not handle this.
	     * Note that this write may need an additional PCIe stream ID to
	     * be allowed in the EBF menu.
	     */
	    uint64_t kt = ktime_get_real_ns();
	    octeon_pci_bar4_write64(oct_dev, CN93XX_MIO_PTP_BAR4_REGION, CN93XX_MIO_PTP_CLOCK_HI_OFFSET, kt);
	    printk("OCT_PHC[%d]: Setting PTP_CLOCK_HI based on host time: %lld\n",
		   oct_dev->octeon_id, (unsigned long long)kt);
	}

	cavium_print_msg("OCT_PHC[%d]: Octeon PHC is ready\n",
			 oct_dev->octeon_id);
}

#ifdef PHC_DEBUG
static int poll_flag = 1;
static void octeon_device_poll(struct work_struct *work)
{
	octeon_device_t *oct_dev;
	struct cavium_delayed_wq *wq;
	u8 status;
	uint64_t tmp64;
	uint64_t kt;
	uint64_t kt1;
	uint64_t p_kt = 0;
	uint64_t p_ptp = 0;

	wq = container_of(work, struct cavium_delayed_wq, wk.work.work);
	oct_dev = (octeon_device_t *)wq->wk.ctxptr;

	cavium_print_msg("OCT_PHC[%d]: Octeon PHC debug poll loop started.\n",
			 oct_dev->octeon_id);
	while (1) {
		schedule_timeout_interruptible(HZ * 1);
		preempt_disable_notrace();
		kt1 = ktime_get_real_ns();
		tmp64 = octeon_pci_bar4_read64(oct_dev, CN93XX_MIO_PTP_BAR4_REGION, CN93XX_MIO_PTP_CKOUT_THRESH_HI_OFFSET);
		tmp64 += octeon_pci_bar4_read64(oct_dev, CN93XX_MIO_PTP_BAR4_REGION, CN93XX_MIO_PTP_CLOCK_HI_OFFSET);
		kt = ktime_get_real_ns();
		preempt_enable_notrace();
		printk("OCT_PHC[%d]: PTP_CLOCK_HI: %lld, kt/ptp diff: %lld, ptp int: %lld, kt int: %lld, int diff: %lld, lat: %lld\n",
		       oct_dev->octeon_id,
		       (unsigned long long)tmp64, (long long)(tmp64 - kt),
		       (long long)(tmp64 - p_ptp),
		       (long long)(kt - p_kt),
		       (long long)(tmp64 - p_ptp) - (long long)(kt - p_kt),
		       (long long)(kt - kt1));
		p_ptp = tmp64;
		p_kt = kt;
		if (cavium_atomic_read(&oct_dev->status) > OCT_DEV_RUNNING)
		    return;
	}
}
#endif

int octeon_ep_phc_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    octeon_device_t *oct_dev = NULL;
    u32 dev_id;

    oct_dev = octeon_allocate_device(pdev->device);
    if (oct_dev == NULL)
	    return -ENOMEM;

    oct_dev->oct_ep_ptp_clock = kmalloc(sizeof(struct oct_ep_ptp_clock ), GFP_KERNEL);
    if (oct_dev->oct_ep_ptp_clock == NULL)
	return -ENOMEM;


    /* Assign octeon_device for this device to the private data area. */
    pci_set_drvdata(pdev, oct_dev);

    /* set linux specific device pointer */
    oct_dev->pci_dev = (void *)pdev;

    OCTEON_READ_PCI_CONFIG(oct_dev, 0x2c, &dev_id);
    cavium_print_msg("OCT_PHC[%d]: Setting up PHC %x\n",
		     oct_dev->octeon_id, dev_id);

    oct_dev->dev_init_wq.wq = alloc_workqueue("dev_init_wq", WQ_MEM_RECLAIM, 0);
    oct_dev->dev_init_wq.wk.ctxptr = oct_dev;
    INIT_DELAYED_WORK(&oct_dev->dev_init_wq.wk.work, octeon_device_init_work);
    queue_delayed_work(oct_dev->dev_init_wq.wq, &oct_dev->dev_init_wq.wk.work, 0);

#ifdef PHC_DEBUG
    oct_dev->dev_poll_wq.wq = alloc_workqueue("dev_poll_wq", WQ_MEM_RECLAIM, 0);
    oct_dev->dev_poll_wq.wk.ctxptr = oct_dev;
    INIT_DELAYED_WORK(&oct_dev->dev_poll_wq.wk.work, octeon_device_poll);
    queue_delayed_work(oct_dev->dev_poll_wq.wq, &oct_dev->dev_poll_wq.wk.work, 0);
#endif

    return 0;

}
void octeon_ep_phc_remove(struct pci_dev *pdev)
{
    octeon_device_t *oct_dev = pci_get_drvdata(pdev);
    int oct_idx;

    oct_idx = oct_dev->octeon_id;

    cavium_print_msg("OCT_PHC[%d]: Stopping octeon device %d\n", oct_idx);
    if (cavium_atomic_read(&oct_dev->status) == OCT_DEV_CHECK_FW) {
	    cavium_atomic_set(&oct_dev->status, OCT_DEV_STOPPING);
	    while (true) {
		    if (cavium_atomic_read(&oct_dev->status) == OCT_DEV_STATE_INVALID)
			    return;

		    cavium_error("OCT_PHC[%d]: Waiting for firmware ready work to end.\n",
				 oct_idx);
		    schedule_timeout_interruptible(HZ * 1);
	    }
	    goto before_exit;
    }

    cavium_atomic_set(&oct_dev->status, OCT_DEV_STOPPING);

#ifdef PHC_DEBUG
    cancel_delayed_work_sync(&oct_dev->dev_poll_wq.wk.work);
    flush_workqueue(oct_dev->dev_poll_wq.wq);
    destroy_workqueue(oct_dev->dev_poll_wq.wq);
    oct_dev->dev_poll_wq.wq = NULL;
#endif
    cancel_delayed_work_sync(&oct_dev->dev_init_wq.wk.work);
    flush_workqueue(oct_dev->dev_init_wq.wq);
    destroy_workqueue(oct_dev->dev_init_wq.wq);
    oct_dev->dev_init_wq.wq = NULL;


    ptp_clock_unregister(oct_dev->oct_ep_ptp_clock->ptp_clock);
    kfree(oct_dev->oct_ep_ptp_clock);

    /* Reset the octeon device and cleanup all memory allocated for
       the octeon device by driver. */
    octeon_destroy_resources(oct_dev);

    /* This octeon device has been removed. Update the global
       data structure to reflect this. Free the device structure. */
    octeon_free_device_mem(oct_dev);

before_exit:
    cavium_print_msg("OCT_PHC[%d]: Octeon device removed\n", oct_idx);
}



module_init(phc_init);
module_exit(phc_exit);
MODULE_AUTHOR("Marvell Inc.");
MODULE_DESCRIPTION("OTX PCIe EP PHC");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
