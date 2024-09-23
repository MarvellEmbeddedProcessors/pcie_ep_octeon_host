/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2024 Marvell.
 */

#include <stdint.h>
#include "desc_queue.h"
#include "octboot_net.h"
#include "mmio_api.h"
#include <pci/pci.h>
#include <pthread.h>
#include "mrvl_cuse.h"
#include "extern.h"
#include <sys/ioctl.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include"mrvl_cuse.h"

#define CONFIG_SIZE 4096
#define OCTBOOT_NET_VERSION "1.0"
#define OCTBOOT_NET_VERSION_MAJOR 1
#define OCTBOOT_NET_VERSION_MINOR 0
#define ETH_ZLEN	60
#define OCTBOOT_NET_INIT_WQ_DELAY 2000000 // 2secs

pthread_mutex_t init_task_lock;
int net_init_task = 1;

/* Device status */
enum octboot_net_status {
	OCTBOOT_DEV_STATUS_READY,
	OCTBOOT_DEV_STATUS_PFFLR
};

/* Task running status */
enum octboot_task_status {
	TASK_STATUS_RUNNING,
};

struct octboot_net_struct {
	struct octboot_net_dev *mdev;
	struct pci_dev *octnet_pci_dev_arr;
	mrvl_pcie_dev_t *mrvl_pdev;
	int pci_bus;
	int pci_device;
	int pci_fn;
	int octboot_net_init_done;
	int initialized;
	atomic_int flr_status;
};

static struct octboot_net_struct octboot_struct[MAX_MRVL_OCT_DEV];

static struct octboot_net_dev *gmdev[MAX_MRVL_OCT_DEV];
static int octnet_num_device;
static int octboot_net_init_done[MAX_MRVL_OCT_DEV];

static void mgmt_init_work(void *bar4_addr, int index);
static int mdev_reinit_rings(struct octboot_net_dev *mdev);
static void change_host_status(struct octboot_net_dev *mdev, uint64_t status,
			bool ack_wait);
static int find_octboot_net_entry(mrvl_pcie_dev_t *dev);
static int mgmt_init_start[MAX_MRVL_OCT_DEV];

#define DEVICE_COUNT_RESOURCE 3

#define OCTBOOT_NET_MBOX_SIZE_WORDS 8
#define OCTBOOT_NET_MBOX_HOST_STATUS_CHANGE 1
#define OCTBOOT_NET_MBOX_TARGET_STATUS_CHANGE 2
#define OCTBOOT_NET_MBOX_OPCODE_INVALID 0xFF

#define OCTBOOT_NET_MBOX_TIMEOUT_MS 100
#define OCTBOOT_NET_MBOX_WAIT_MS 10
#define OCTBOOT_NET_MBOX_DBELL_ID 0
struct octboot_net_mbox_hdr {
	uint64_t opcode  :8;
	uint64_t id      :8;
	uint64_t req_ack :1;
	uint64_t sizew   :3; /* size in words excluding hdr */
	uint64_t rsvd    :44;
} __attribute__((packed));

union octboot_net_mbox_msg {
	uint64_t words[OCTBOOT_NET_MBOX_SIZE_WORDS];
	struct {
		 struct octboot_net_mbox_hdr hdr;
		 uint64_t data[7];
	} s;
} __attribute__((packed));

static void octboot_net_poll(void);

struct uboot_pcinet_barmap {
	uint64_t signature;
	uint64_t host_version;
	uint64_t host_status_reg;
	uint64_t host_mailbox_ack;
	uint64_t host_mailbox[MAX_MRVL_OCT_DEV];
	uint64_t target_version;
	uint64_t target_status_reg;
	uint64_t target_mailbox_ack;
	uint64_t target_mailbox[MAX_MRVL_OCT_DEV];
	uint64_t rx_descriptor_offset;
	uint64_t tx_descriptor_offset;
};

#define OCTBOOT_NET_MAXQ 1
#define OCTBOOT_NET_DESCQ_CLEAN 0
#define OCTBOOT_NET_DESCQ_READY 1
#define OCTBOOT_IFACE_NAME "octboot_net%d"
#define OCTBOOT_IFACE_NAME_SZ	15
#define OCTBOOT_NET_NUM_ELEMENTS 256
#define OCTBOOT_NET_SERVICE_TASK_US 1000 // 1msec
#define OCTBOOT_NET_SERVICE_TASK_US_FLR 6000000 // 6sec
struct tapdev {
	volatile int flags;
	char tap_name[20];
};

struct octboot_net_dev {
	mrvl_pcie_dev_t *dev;
	struct tapdev *tdev;
	int tap_fd;
	struct pci_dev *pdev;
	struct octboot_net_sw_descq rxq[OCTBOOT_NET_MAXQ];
	struct octboot_net_sw_descq txq[OCTBOOT_NET_MAXQ];
	bool  admin_up;
	uint8_t  *bar_map;
	uint32_t bar_map_size;
	uint32_t max_rxq;
	uint32_t num_rxq;
	uint32_t max_txq;
	uint32_t num_txq;
	uint32_t element_count;
	struct workqueue_struct *mgmt_wq;
	uint32_t *tq_cons_shdw_vaddr;
	uint64_t tq_cons_shdw_dma;
	uint32_t *rq_cons_shdw_vaddr;
	uint64_t rq_cons_shdw_dma;
	pthread_mutex_t mbox_lock;
	pthread_mutex_t service_task_lock;
	int service_task_state;
	uint32_t send_mbox_id;
	uint32_t recv_mbox_id;
	int      octboot_net_restart;
	uint8_t hw_addr[6];
	atomic_bool task_status;
	uint32_t task_run_poll_cnt;
};

static void *octboot_net_task(void *devm);

#define NPU_HANDSHAKE_SIGNATURE 0xABCDABCD
#define SIGNATURE_OFFSET 0x2000000 /* BAR4 index 8 is at this offset */
#define HOST_VERSION_OFFSET 0x2000008
#define HOST_STATUS_REG_OFFSET 0x2000080

#define OCTNET_HOST_DOWN                 0
#define OCTNET_HOST_READY                1
#define OCTNET_HOST_RUNNING              2
#define OCTNET_HOST_GOING_DOWN           3
#define OCTNET_HOST_FATAL                4

#define HOST_RESET_STATUS_REG_OFFSET 0x2000088
#define OCTNET_HOST_RESET_STATUS_BIT     0

#define HOST_MBOX_ACK_OFFSET 0x2000090
#define HOST_MBOX_OFFSET 0x2000098    /* Eight words at this offset */
#define TARGET_VERSION_OFFSET 0x2000060
#define TARGET_STATUS_REG_OFFSET 0x2000100


#define HOST_STATUS_REG(mdev)      (mdev->bar_map + HOST_STATUS_REG_OFFSET)
#define HOST_RESET_STATUS_REG(mdev) (mdev->bar_map + HOST_RESET_STATUS_REG_OFFSET)
#define HOST_VERSION_REG(mdev)      (mdev->bar_map + HOST_VERSION_OFFSET)
#define HOST_MBOX_ACK_REG(mdev)    (mdev->bar_map + HOST_MBOX_ACK_OFFSET)
#define HOST_MBOX_MSG_REG(mdev, i)    \
	(mdev->bar_map + HOST_MBOX_OFFSET + (i * 8))


#define OCTNET_TARGET_DOWN               0
#define OCTNET_TARGET_READY              1
#define OCTNET_TARGET_RUNNING            2
#define OCTNET_TARGET_GOING_DOWN         3
#define OCTNET_TARGET_FATAL              4


#define TARGET_MBOX_OFFSET 0x2000118
#define TARGET_MBOX_ACK_OFFSET 0x2000110
#define OCTNET_RX_DESC_OFFSET 0x20000B8
#define OCTNET_TX_DESC_OFFSET 0x20000c0

#define OCTNET_TX_DESCQ_OFFSET   0x2000400
#define OCTNET_RX_DESCQ_OFFSET   0x2010000

#define TX_DESCQ_OFFSET(mdev)     (mdev->bar_map + OCTNET_TX_DESCQ_OFFSET)
#define RX_DESCQ_OFFSET(mdev)     (mdev->bar_map + OCTNET_RX_DESCQ_OFFSET)

#define TARGET_STATUS_REG(mdev)        (mdev->bar_map + TARGET_STATUS_REG_OFFSET)
#define TARGET_VERSION_REG(mdev)        (mdev->bar_map + TARGET_VERSION_OFFSET)
#define TARGET_MBOX_MSG_REG(mdev, i)  \
	(mdev->bar_map + TARGET_MBOX_OFFSET + (i * 8))
#define TARGET_MBOX_ACK_REG(mdev)    \
	(mdev->bar_map + TARGET_MBOX_ACK_OFFSET)

typedef struct {
	/** PCI address to which the BAR is mapped. */
	unsigned long start;
	/** Length of this PCI address space. */
	unsigned long len;
	/** Length that has been mapped to phys. address space. */
	unsigned long mapped_len;
	/** The physical address to which the PCI address space is mapped. */
	void *hw_addr;
	/** Flag indicating the mapping was successful. */
	int done;
} octeon_mmio;

typedef struct {
	octeon_mmio mmio[3];
	/* struct npu_bar_map npu_memmap_info; */
	struct uboot_pcinet_barmap npu_memmap_info;
	void *bar4_addr;
	int signature_found;
	bool unavailable;
	bool enabled;
	struct pci_dev *pdev;
	int pci_saved_state;
} octboot_net_device_t;

octboot_net_device_t octboot_net_device[8];

int octboot_net_tx(uint32_t *buf, int len, struct octboot_net_dev *mdev);
int tap_alloc(char *dev, int flags);

int mrvl_pci_restore_state(mrvl_pcie_dev_t *mrvl_dev)
{
	struct pci_dev *dev = mrvl_dev->pci_dev;

	/* Restore config space of CONFIG_SIZE */
//	pci_setup_cache(dev, mrvl_dev->d_lcl->config, CONFIG_SIZE);
	pci_write_block(dev, 0, mrvl_dev->d_lcl->config, CONFIG_SIZE);
}

struct device *
scan_device(struct pci_dev *pdev, struct device *dev)
{

	if (dev)
		return dev;

	dev = malloc(sizeof(struct device));
	memset(dev, 0, sizeof(*dev));
	dev->dev = pdev;
	dev->config = malloc(CONFIG_SIZE);
	pci_setup_cache(pdev, dev->config, CONFIG_SIZE);
	if (!pci_read_block(pdev, 0, dev->config, CONFIG_SIZE))
		printf("\n Failed to save config space\n");
	return dev;
}

int mrvl_pci_save_state(mrvl_pcie_dev_t *mrvl_dev)
{
	struct device *d = NULL;
	struct pci_dev *dev = mrvl_dev->pci_dev;

	d = scan_device(dev, NULL);
	mrvl_dev->d_lcl = d;
}

static struct octboot_net_dev *find_event_producer_tap_fd(int fd)
{
	int i;
	int pci_bus, pci_device, pci_fn;

	for (i = 0; i < MAX_MRVL_OCT_DEV; i++) {
		if (octboot_struct[i].initialized &&
			octboot_struct[i].octboot_net_init_done &&
				octboot_struct[i].mdev->tap_fd == fd)
			return octboot_struct[i].mdev;
	}

	return NULL;
}

void dump_buffer(char buf[], int len)
{
	uint8_t data;

	for (int i = 0; i < len; i++) {
		if (i % 8 == 0)
			printf("\n");
		data = buf[i];
		printf("%02x ", data);
	}
}

void poll_for_events_on_fds(void)
{
	int ret, num;
	int fd, nread;
	struct octboot_net_dev *mdev;
	char buffer[2000];
	uint64_t c = 0;
	int q_num = 0;

	while (!term_reaceived) {
		num = epoll_wait(epollfd, events, MAXEVENTS, -1);
		if (num <= 0) {
			if (num < 0)
				printf("epoll_wait failed\n");
			continue;
		}

		for (int i = 0; i < num; i++) {

			fd = events[i].data.fd;
			if (fd == timerfd) {
				for (int j = 0; j < MAX_MRVL_OCT_DEV; j++) {
					if (octboot_struct[j].initialized && octboot_struct[j].octboot_net_init_done) {
						if (octboot_struct[j].mdev->service_task_state)
							octboot_net_task(octboot_struct[j].mdev);
					}
				}
				continue;
			}

			mdev = find_event_producer_tap_fd(fd);
			if (!mdev->service_task_state)
				continue;

			if (!mdev) {
				printf("\nwrong fd found\n");
				continue;
			}

			pthread_spin_lock(&mdev->txq[q_num].lock);
			nread = read(fd, buffer, sizeof(buffer));
			if (nread < 0) {
				perror("Reading from interface");
				pthread_spin_unlock(&mdev->txq[q_num].lock);
				continue;
			}
			octboot_net_tx((uint32_t*)buffer, nread, mdev);
			pthread_spin_unlock(&mdev->txq[q_num].lock);
		}
	}
}

/*
 * reset targets during module load
 * 0 = reset (default)
 * 1 = no reset
 */
bool is_flr_inprogress(int index)
{
	atomic_int status;

	if ((octboot_struct[index].octboot_net_init_done) && (octboot_struct[index].mdev)) {
		status = atomic_load(&octboot_struct[index].flr_status);
		if (status == OCTBOOT_DEV_STATUS_PFFLR) {
			printf("Device is in PFFLR State in init_task");
			return true;
		}
	return false;
	}
}

static bool octboot_net_task_busy(struct octboot_net_dev *mdev)
{
	return atomic_flag_test_and_set(&mdev->task_status);
}

static uint64_t get_host_status(struct octboot_net_dev *mdev)
{
	return readq(HOST_STATUS_REG(mdev));
}

static void set_host_reset_status(struct octboot_net_dev *mdev, bool set)
{
	uint64_t val;

	printf("reset host status to %s", set ? "true" : "false");
	val = readq(HOST_RESET_STATUS_REG(mdev));
	val &= ~(1 << OCTNET_HOST_RESET_STATUS_BIT);
	if (set)
		val |= (set << OCTNET_HOST_RESET_STATUS_BIT);
	printf("reset host status reg 0x%lx to val 0x%lx",
		 (uint64_t)HOST_RESET_STATUS_REG(mdev), val);
	writeq(val, HOST_RESET_STATUS_REG(mdev));
}

static uint64_t get_target_status(struct octboot_net_dev *mdev)
{
	return readq(TARGET_STATUS_REG(mdev));
}

static uint64_t get_target_version(struct octboot_net_dev *mdev)
{
	return readq(TARGET_VERSION_REG(mdev));
}

static uint64_t get_target_mbox_ack(struct octboot_net_dev *mdev)
{
	return readq(TARGET_MBOX_ACK_REG(mdev));
}

static void set_host_mbox_ack_reg(struct octboot_net_dev *mdev, uint32_t id)
{
	writeq(id, HOST_MBOX_ACK_REG(mdev));
}

static void mbox_send_msg(struct octboot_net_dev *mdev,
		union octboot_net_mbox_msg *msg)
{
	int i, id, ret;

	pthread_mutex_lock(&mdev->mbox_lock);
	mdev->send_mbox_id++;
	msg->s.hdr.id = mdev->send_mbox_id;
	id = msg->s.hdr.id;
	for (i = 1; i <= msg->s.hdr.sizew; i++)
		writeq(msg->words[i], HOST_MBOX_MSG_REG(mdev, i));

	/* write header at the end */
	printf("send mbox msg id:%d opcode:%d sizew: %d\n",
	       msg->s.hdr.id, msg->s.hdr.opcode, msg->s.hdr.sizew);
	writeq(msg->words[0], HOST_MBOX_MSG_REG(mdev, 0));

	/* more than 1 word mbox messages need explicit ack */
	if (msg->s.hdr.req_ack || msg->s.hdr.sizew) {
		time_t t0, t1;

		while ((ret = get_target_mbox_ack(mdev)) != id) {
			usleep(1000);
			time(&t1);
			if (difftime(t1, t0) > 3) {
				printf("octboot_net:mbox ack wait failed\n");
				break;
			}
		}
	}
	pthread_mutex_unlock(&mdev->mbox_lock);
}

static void octboot_net_restart(void)
{
	usleep(OCTBOOT_NET_INIT_WQ_DELAY);
	pthread_mutex_lock(&init_task_lock);
	net_init_task = 1;
	pthread_mutex_unlock(&init_task_lock);
}

static int mbox_check_msg_rcvd(struct octboot_net_dev *mdev,
			union octboot_net_mbox_msg *msg)
{
	unsigned int flags;
	int i, ret;

	flags = mdev->tdev->flags;

	if (!(flags | IFF_RUNNING))
		return 0;

	pthread_mutex_lock(&mdev->mbox_lock);
	msg->words[0] = readq(TARGET_MBOX_MSG_REG(mdev, 0));
	if (msg->s.hdr.opcode == OCTBOOT_NET_MBOX_OPCODE_INVALID) {
		ret = 0;

		/* If restart was already set, do not repeat process */
		if (mdev->octboot_net_restart) {
			pthread_mutex_unlock(&mdev->mbox_lock);
			return ret;
		}

		printf("Async or Sync reset of Octeon device\n");
		pthread_mutex_unlock(&mdev->mbox_lock);
		mdev->octboot_net_restart = true;

		/* set netdevice down */
		mdev->tdev->flags &= ~(IFF_RUNNING);
		change_host_status(mdev, OCTNET_HOST_GOING_DOWN, false);

		// Stop service loop, means processing tx/tx,mbox
		//set_tap_carrier(mdev->tdev->tap_name, 0);

		pthread_mutex_lock(&mdev->service_task_lock);
		mdev->service_task_state = 0;
		pthread_mutex_unlock(&mdev->service_task_lock);
		octboot_net_restart();

		/* Perform cleanup and return to looking for signature */
		return ret;
	}

	if (mdev->recv_mbox_id != msg->s.hdr.id) {

		/* new msg */
		printf("new mbox msg id:%d opcode:%d sizew: %d\n",
			msg->s.hdr.id, msg->s.hdr.opcode, msg->s.hdr.sizew);

		mdev->recv_mbox_id = msg->s.hdr.id;
		for (i = 1; i <= msg->s.hdr.sizew; i++)
			msg->words[i] = readq(TARGET_MBOX_MSG_REG(mdev, i));
		ret = 0;
	} else {
		ret = -ENOENT;
	}

	pthread_mutex_unlock(&mdev->mbox_lock);
	return ret;
}

static void change_host_status(struct octboot_net_dev *mdev, uint64_t status,
			bool ack_wait)
{
	union octboot_net_mbox_msg msg;

	printf("change host status from %ld to %ld\n",
		   readq(HOST_STATUS_REG(mdev)), status);

	writeq(status, HOST_STATUS_REG(mdev));
	memset(&msg, 0, sizeof(union octboot_net_mbox_msg));
	msg.s.hdr.opcode = OCTBOOT_NET_MBOX_HOST_STATUS_CHANGE;
	if (ack_wait)
		msg.s.hdr.req_ack = 1;
	mbox_send_msg(mdev, &msg);
}

static int find_octboot_net_entry(mrvl_pcie_dev_t *octnet_pci_dev)
{
	int i;
	int pci_bus, pci_device, pci_fn;

	pci_bus = octnet_pci_dev->bus;
	pci_device = octnet_pci_dev->dev;
	pci_fn = octnet_pci_dev->func;
	struct pci_dev *dev = octnet_pci_dev->pci_dev;

	for (i = 0; i < MAX_MRVL_OCT_DEV; i++) {
		if (octboot_struct[i].initialized &&
			octboot_struct[i].pci_bus == pci_bus &&
			octboot_struct[i].pci_device == pci_device &&
			octboot_struct[i].pci_fn == pci_fn) {
			return i;
		}
	}
	return -1;
}

static int add_octboot_net_entry(mrvl_pcie_dev_t *octnet_pci_dev)
{

	int i;
	int pci_bus, pci_device, pci_fn;

	pci_bus = octnet_pci_dev->bus;
	pci_device = octnet_pci_dev->dev;
	pci_fn = octnet_pci_dev->func;

	for (i = 0; i < MAX_MRVL_OCT_DEV; i++) {
		if (!octboot_struct[i].initialized) {
			octboot_struct[i].octnet_pci_dev_arr = octnet_pci_dev->pci_dev;
			octboot_struct[i].mrvl_pdev = octnet_pci_dev;
			octboot_struct[i].pci_bus = pci_bus;
			octboot_struct[i].pci_device = pci_device;
			octboot_struct[i].pci_fn = pci_fn;
			octboot_struct[i].initialized = true;
			return i;
		}
	}
	printf(&octnet_pci_dev->dev, "Error: exceeded max devices supported\n");
	return -ENOSPC;
}

static bool octboot_is_pci_bar_addr_reset(struct pci_dev *pdev)
{
	uint32_t bar0_base, bar1_base;

	bar0_base = pci_read_long(pdev, PCI_BASE_ADDRESS_0);
	bar1_base = pci_read_long(pdev, PCI_BASE_ADDRESS_1);
	if (!(bar0_base >> 16) && !bar1_base) {
		return 1;
	}

	return 0;
}

int pacc_dev_cache_refresh(mrvl_pcie_dev_t *mdev)
{
	struct pci_access *pacc = mdev->pacc;
        struct pci_dev *dev;
        struct device *d;
        int ret = 0;

	if (mdev->pacc_reset)
		pci_cleanup(pacc);

	pacc = pci_alloc();
        if (!pacc) {
                printf("\n %s failed\n", __func__);
                return -ENOMEM;
        }

	mdev->pacc = pacc;
	mdev->pacc_reset = 1;
        pci_init(pacc);
        pci_scan_bus(pacc);

        /* Iterate over the devices */
        for (dev = pacc->devices; dev; dev = dev->next) {
                pci_fill_info(dev, PCI_FILL_IDENT | PCI_FILL_BASES | PCI_FILL_CLASS);

                if ((dev->device_id != MRVL_DEVICE_ID) || ((dev->bus != mdev->bus) ||
				(dev->dev != mdev->dev) || (dev->func != mdev->func)))
                        continue;
		ret = octboot_is_pci_bar_addr_reset(dev);
		break;
        }

        return ret;
}

static bool octboot_is_pci_bar_accessible(octboot_net_device_t *octboot_dev)
{
	struct pci_dev *pdev = octboot_dev->pdev;
	uint64_t addr, signature;

	addr = (uint64_t)(octboot_dev->bar4_addr + SIGNATURE_OFFSET);

	/* all F's means BAR not accessible */
	return (*(uint64_t *)addr != -1ULL);
}

static int octboot_enable_device(octboot_net_device_t *octboot_dev, void *dev)
{
	struct pci_dev *pdev;
	int i, ret = 0;
	mrvl_pcie_dev_t *mpci_dev = dev;

	pdev = octboot_dev->pdev;

	if (pacc_dev_cache_refresh(mpci_dev)) {

		/* device was earlier not available; might be going through reset.
		 * now available; restore the config.
		 */
		printf("Device available but BAR addr is reset; restore config\n");
		mrvl_pci_restore_state(mpci_dev);
	}

	if (octboot_dev->bar4_addr && !octboot_is_pci_bar_accessible(octboot_dev)) {

		/* Device unavailable; may be going through reset */
		if (octboot_dev->enabled && !octboot_dev->unavailable) {
			printf("Device became unavailable\n");
			octboot_dev->unavailable = true;

			/* once device is unavailable, signature cannot be found */
			octboot_dev->signature_found = false;
		}
		return -EAGAIN;
	}
	/* If earlier unavailable, reset state to available */
	if (octboot_dev->unavailable) {
		printf("Device became available\n");
		octboot_dev->unavailable = false;
	}

	/* Enable the device only once; later just call restore state */
	if (octboot_dev->enabled) {
		printf("\ndevice already enabled...\n");
		return 0;


	}

	printf(&pdev->dev, "enabling device ...\n");

	octboot_dev->mmio[2].hw_addr = mpci_dev->bar4_addr;
	octboot_dev->mmio[2].done = 1;

	octboot_dev->bar4_addr = octboot_dev->mmio[2].hw_addr;
	octboot_dev->enabled = true;
	octboot_dev->signature_found = false;

	return 0;
}

void *octboot_net_init_work(mrvl_pcie_dev_t **mrvl_pcidev)
{
	mrvl_pcie_dev_t *octnet_pci_dev = NULL;
	int i, entry_idx, ret;

	/* need to loop for number of devices scanned array and pass accordingly */
	pthread_mutex_init(&init_task_lock, NULL);
	octnet_num_device = 0;

	while (!term_reaceived) {
		for (i = 0; i < current_oct_pcie_dev_cnt; i++) {
			octnet_pci_dev = mrvl_pcidev[i];
			entry_idx = find_octboot_net_entry(octnet_pci_dev);

			if (entry_idx == -1) {
				printf("\n entry idx = %d ", entry_idx);
				entry_idx = add_octboot_net_entry(octnet_pci_dev);

				if (entry_idx == -ENOSPC) {
					printf("Ignoring this device\n");
					continue;
				}

				printf("Device added at entry %d\n", entry_idx);
				octboot_net_device[entry_idx].pdev = octnet_pci_dev->pci_dev;
				octnet_num_device++;
			}

			if (is_flr_inprogress(entry_idx)) {
				printf("Main device init_task Device is in PFFLR State in init_task (devid=0x%x)\n",
					octnet_pci_dev->device_id);
				continue;
			}

			octboot_enable_device(&octboot_net_device[entry_idx], octnet_pci_dev);
			fflush(stdout);
		}

		pthread_mutex_lock(&init_task_lock);
		net_init_task = 0;
		pthread_mutex_unlock(&init_task_lock);
		octboot_net_poll();
		while (!net_init_task)
			sched_yield();
	}

	return 0;
}

static void octboot_net_poll(void)
{
	struct pci_dev *octnet_pci_device;
	int offset = SIGNATURE_OFFSET;
	uint64_t signature;
	void *bar4_addr;
	void *src;
	int i;

	for (i = 0; i < octnet_num_device; i++) {
		if (!octboot_net_device[i].bar4_addr ||
		    !octboot_net_device[i].pdev ||
		    octboot_net_device[i].unavailable)
			continue;

		octnet_pci_device = octboot_struct[i].octnet_pci_dev_arr;
		if (is_flr_inprogress(i)) {
			printf("Net poll Device is in PFFLR State in init_task");
			continue;
		}

		bar4_addr = octboot_net_device[i].bar4_addr;
		src = bar4_addr + offset;
		octnet_pci_device = octboot_struct[i].octnet_pci_dev_arr;
		memcpy(&octboot_net_device[i].npu_memmap_info, src,
			sizeof(struct uboot_pcinet_barmap));

		/* Check for signature */
		signature = octboot_net_device[i].npu_memmap_info.signature;
		printf("\n signature = 0x%lx\n", signature);

		if (signature == NPU_HANDSHAKE_SIGNATURE) {
			if (!octboot_net_device[i].signature_found) {

				/* Uboot is booting and requires a netdevice for tftp */
				printf("[Device-%d] Found valid signature 0x%lx\n", i, signature);
				octboot_net_device[i].signature_found = true;
				octboot_net_device[i].unavailable = false;

				/* Save state for future restoration */
				printf("saving pci state ...\n");
				if (!octboot_net_device[i].pci_saved_state) {
					mrvl_pci_save_state(octboot_struct[i].mrvl_pdev);
					octboot_net_device[i].pci_saved_state = 1;
				}
			}
		} else if (octboot_net_device[i].signature_found) {

			/* earlier valid signature found, but now read invalid signature */
			printf("[Device-%d] Found invalid signature 0x%lx\n", i, signature);
			octboot_net_device[i].signature_found = false;
			octboot_net_device[i].unavailable = true;
		}
	}

	/* Now that we have the signature, the next step is to create a
	 * netdevice
	 */
	for (i = 0; i < octnet_num_device; i++) {
		if ((octboot_net_device[i].signature_found == true) &&
						!mgmt_init_start[i]) {
			bar4_addr = octboot_net_device[i].bar4_addr;
			mgmt_init_work(bar4_addr, i);
			mgmt_init_start[i] = 1;
		}
		if ((octboot_net_device[i].signature_found == true) &&
				(mgmt_init_start[i]) &&
				octboot_net_init_done[i]) {
			struct octboot_net_dev *mdev = gmdev[i];
			int ret;

			/* This is restart */
			if (mdev->octboot_net_restart == true) {
				int flags;

				printf("This is restart of mgmt service task\n");
				change_host_status(mdev, OCTNET_HOST_GOING_DOWN, false);
				//set_tap_carrier(mdev->tdev->tap_name, 0);

				/* Add some flags or code to make tap down /unavailable */
				ret = mdev_reinit_rings(mdev);
				if (ret) {
					printf("restart of mgmt service task failed\n");
					printf("Please unload and load octboot_net module\n");
					change_host_status(mdev, OCTNET_HOST_FATAL, false);
					return;
				}
				change_host_status(mdev, OCTNET_HOST_READY, false);

				/* barrier to ensure the octboot_net_task thread  reads the
				 * updated flag
				 */
				flags = mdev->tdev->flags;
				flags |= IFF_RUNNING;
				mdev->tdev->flags = flags;
				mdev->octboot_net_restart = false;

				pthread_mutex_lock(&mdev->service_task_lock);
				mdev->service_task_state = 1;
				pthread_mutex_unlock(&mdev->service_task_lock);

			}
		}
	}

	for (i = 0; i < octnet_num_device; i++) {
		if (octboot_net_device[i].signature_found == false) {
			octboot_net_restart();
			return;
		}
	}
}

/* Here tap interface fd will be received and corresponding mdev is to be found */
int octboot_net_tx(uint32_t *buf, int len, struct octboot_net_dev *mdev)
{
	struct octboot_net_hw_desc_ptr ptr;
	struct octboot_net_sw_descq  *tq;
	uint32_t cur_cons_idx, cur_prod_idx;
	uint8_t *hw_desc_ptr;
	uint64_t dma;
	int idx = 0;
	int xmit_more;
	int bytes;
	struct sk_buff *skb;

	tq = &mdev->txq[idx];

	if (get_host_status(mdev) != OCTNET_HOST_RUNNING)
		goto err;
	bytes = len;
	cur_cons_idx = *tq->cons_idx_shadow;
	cur_prod_idx = tq->local_prod_idx;
	if (!octboot_net_circq_space(cur_prod_idx, cur_cons_idx, tq->mask)) {
		tq->errors++;
		printf("\n no space in tx ring\n");

		/* if we have accumulated skbs send them */
		if (tq->pending) {
			writel(tq->local_prod_idx, tq->hw_prod_idx);
			tq->pending = 0;
		}
		return -1;
	}

	memset(&ptr, 0, sizeof(struct octboot_net_hw_desc_ptr));
	ptr.hdr.s_mgmt_net.ptr_type = OCTBOOT_NET_DESC_PTR_DIRECT;
	ptr.hdr.s_mgmt_net.ptr_len = len;
	ptr.hdr.s_mgmt_net.total_len = len;

	/* Copy incoming buf or we need to read buf at already allocated dma`able buf */
	skb = tq->skb_list[cur_prod_idx];
	memcpy(skb->data, buf, len);
	ptr.ptr = skb->dma;
	hw_desc_ptr = tq->hw_descq +
		OCTBOOT_NET_DESC_ARR_ENTRY_OFFSET(cur_prod_idx);

	mmio_memwrite(hw_desc_ptr, &ptr, sizeof(struct octboot_net_hw_desc_ptr));
	cur_prod_idx = octboot_net_circq_inc(cur_prod_idx, tq->mask);
	tq->local_prod_idx =  cur_prod_idx;
	tq->pkts  += 1;
	tq->bytes += bytes;
	writel(tq->local_prod_idx, tq->hw_prod_idx);
	tq->pending = 0;
	return 0;
err:
	tq->errors++;
	return 0;
}

static void dump_hw_descq(struct octboot_net_hw_descq *descq)
{
	struct  octboot_net_hw_desc_ptr *ptr;
	int i, count;

	printf("prod_idx %u\n", descq->prod_idx);
	printf("cons_idx %u\n", descq->cons_idx);
	printf("num_entries %u\n", descq->num_entries);
	printf("shadow_cons_idx_addr 0x%lx\n",
		descq->shadow_cons_idx_addr);
	printf("shadow_prod_idx_addr 0x%lx\n",
		descq->shadow_prod_idx_addr);

	count = octboot_net_circq_depth(descq->prod_idx, descq->cons_idx, descq->num_entries - 1);
	for (i = 0; i <= count; i++) {
		ptr = &descq->desc_arr[i];
		printf("idx:%d is_frag:%d total_len:%d ptr_type:%d ptr_len:%d ptr:0x%lx\n", i,
			ptr->hdr.s_mgmt_net.is_frag,
			ptr->hdr.s_mgmt_net.total_len,
			ptr->hdr.s_mgmt_net.ptr_type,
			ptr->hdr.s_mgmt_net.ptr_len,
			ptr->ptr);
	}
}

static bool __handle_rxq(struct octboot_net_dev *mdev, int q_idx, int budget)
{
	struct octboot_net_sw_descq *rq = &mdev->rxq[q_idx];
	uint32_t cons_idx, prod_idx;
	struct octboot_net_hw_desc_ptr ptr;
	uint8_t *hw_desc_ptr;
	int count, start, i;
	struct sk_buff *skb;
	struct octboot_net_hw_descq *tmp_descq;
	int descq_tot_size;


	if (get_host_status(mdev) != OCTNET_HOST_RUNNING)
		return false;

	cons_idx = rq->local_cons_idx;
	prod_idx = *rq->cons_idx_shadow;
	if ((cons_idx == 0xFFFFFFFF) || (prod_idx == 0xFFFFFFFF)) {
		printf("$$$$ Rx Received All FFFFFFFF's Count:0x%x RQ mask:0x%x\n",
			(prod_idx-cons_idx), rq->mask);
		return false;
	}

	count = octboot_net_circq_depth(prod_idx,  cons_idx, rq->mask);

	if (!count)
		return false;

	if (count > budget)
		count = budget;

	start = cons_idx;
	for (i = 0; i <= count; i++) {
		skb = rq->skb_list[start];
		//dma_unmap(mdev->dev, skb->data, skb->dma, OCTBOOT_NET_RX_BUF_SIZE);
		hw_desc_ptr = rq->hw_descq +
				OCTBOOT_NET_DESC_ARR_ENTRY_OFFSET(start);
		/* this is not optimal metadata should probably be in the packet */
		mmio_memread(&ptr, hw_desc_ptr,
			     sizeof(struct octboot_net_hw_desc_ptr));

		if ((ptr.hdr.s_mgmt_net.total_len < ETH_ZLEN ||
		    ptr.hdr.s_mgmt_net.is_frag ||
		    ptr.hdr.s_mgmt_net.ptr_len != ptr.hdr.s_mgmt_net.ptr_len)) {
			/* dont handle frags now */
			rq->errors++;
			descq_tot_size = sizeof(struct octboot_net_hw_descq) +
					 (rq->element_count *
					  sizeof(struct octboot_net_hw_desc_ptr));
			tmp_descq = calloc(1, descq_tot_size);
			if (!tmp_descq) {
				printf("rx error kmalloc\n");
			} else {
				mmio_memread(tmp_descq, rq->hw_descq,
					     descq_tot_size);
				dump_hw_descq(tmp_descq);
				free(tmp_descq);
			}
		} else
			write(mdev->tap_fd, rq->dma_list[start], ptr.hdr.s_mgmt_net.total_len);

		start = octboot_net_circq_inc(start, rq->mask);
	}
	/* lists need to be updated before updating cons idx */
	cons_idx = octboot_net_circq_add(cons_idx, count, rq->mask);
	rq->local_cons_idx = cons_idx;
	return 0;
}

static int mdev_clean_tx_ring(struct octboot_net_dev *mdev, int q_idx)
{
	struct octboot_net_sw_descq *tq = &mdev->txq[q_idx];
	uint32_t cons_idx, prod_idx;
	struct sk_buff *skb;
	int i, count, start;
	int descq_tot_size;
	void *dma;

	if (tq->status == OCTBOOT_NET_DESCQ_CLEAN)
		return 0;
	cons_idx = tq->local_cons_idx;
	prod_idx = tq->local_prod_idx;
	count = octboot_net_circq_depth(prod_idx, cons_idx, tq->mask);
	descq_tot_size = sizeof(struct octboot_net_hw_descq) +
		(tq->element_count * sizeof(struct octboot_net_hw_desc_ptr));
	start = cons_idx;
	for (i = 0; i <= count; i++) {
		skb = tq->skb_list[start];
		dma = (void *)tq->dma_list[start];
		dma_unmap(mdev->dev, (uint64_t)dma, skb->dma, OCTBOOT_NET_RX_BUF_SIZE);
		free(skb);
		tq->skb_list[start] = NULL;
		tq->dma_list[start] = 0;
		start = octboot_net_circq_inc(start, tq->mask);
	}

	tq->local_cons_idx = tq->local_prod_idx = 0;
	*tq->cons_idx_shadow = 0;
	tq->status = OCTBOOT_NET_DESCQ_CLEAN;
	free(tq->skb_list);
	free(tq->dma_list);

	/* tq status need to be updated before memset */
	mmio_memset(tq->hw_descq, 0, descq_tot_size);
	return count;
}

static void mdev_clean_tx_rings(struct octboot_net_dev *mdev)
{
	int i;

	for (i = 0; i < mdev->num_txq && i < OCTBOOT_NET_MAXQ; i++)
		mdev_clean_tx_ring(mdev, i);
}

static int mdev_setup_tx_ring(struct octboot_net_dev *mdev, int q_idx)
{
	int element_count = mdev->element_count;
	struct octboot_net_hw_descq *descq;
	struct octboot_net_sw_descq *tq;
	struct octboot_net_hw_desc_ptr *ptr;
	int descq_tot_size;
	int count, ret, i = 0, j;
	struct sk_buff *skb;
	uint64_t  dma, data;


	descq_tot_size = sizeof(struct octboot_net_hw_descq) + (element_count *
		sizeof(struct octboot_net_hw_desc_ptr));
	descq = calloc(1, descq_tot_size);
	if (!descq) {
		perror("octboot_net: tq descq alloc failed\n");
		return -ENOMEM;
	}
	tq = &mdev->txq[q_idx];
	tq->priv = mdev;
	tq->q_num = q_idx;
	tq->local_prod_idx = 0;
	tq->local_cons_idx = 0;
	tq->pending = 0;
	tq->element_count = element_count;
	tq->mask = element_count - 1;
	descq->num_entries = element_count;
	tq->cons_idx_shadow = mdev->tq_cons_shdw_vaddr + q_idx;
	descq->shadow_cons_idx_addr = mdev->tq_cons_shdw_dma +
	(q_idx * sizeof(*mdev->tq_cons_shdw_vaddr));
	*tq->cons_idx_shadow = 0;
	tq->hw_descq = TX_DESCQ_OFFSET(mdev) + (q_idx * descq_tot_size);
	tq->hw_prod_idx = (uint32_t *)(tq->hw_descq +
		offsetof(struct octboot_net_hw_descq, prod_idx));
	tq->skb_list = calloc(1, sizeof(char *) * element_count);
	if (!tq->skb_list) {
		free(descq);
		printf("octboot_net: tq skb_list alloc  failed\n");
		return -ENOMEM;
	}
	tq->dma_list = calloc(1, sizeof(char *) * element_count);
	if (!tq->dma_list) {
		free(descq);
		free(tq->skb_list);
		perror("octboot_net: tq dma_list malloc failed\n");
		return -ENOMEM;
	}

	count = octboot_net_circq_space(tq->local_prod_idx, tq->local_cons_idx,
		  tq->mask);
	data = dma_alloc(mdev->dev, NULL, &dma, OCTBOOT_NET_RX_BUF_SIZE * (count + 1));
	for (i = 0; i <= count; i++) {
		skb = calloc(1, sizeof(struct sk_buff));  // check it
		if (!skb) {
			printf("octboot_net: skb alloc failed\n");
			ret = -ENOMEM;
			goto error;
		}

		skb->dev = mdev->dev;

		skb->data = data + i * OCTBOOT_NET_RX_BUF_SIZE;
		skb->dma = dma + i * OCTBOOT_NET_RX_BUF_SIZE;
		if (!skb->data) {
			printf("\n calloc failed, func %s line %d file %s",
					__func__, __LINE__, __FILE__);
			goto error;
		}

		ptr = &descq->desc_arr[i];
		memset(ptr, 0, sizeof(struct octboot_net_hw_desc_ptr));
		ptr->hdr.s_mgmt_net.ptr_type = OCTBOOT_NET_DESC_PTR_DIRECT;
		ptr->ptr = skb->dma;
		tq->skb_list[tq->local_cons_idx] = skb;
		tq->dma_list[tq->local_cons_idx] = skb->data;
		tq->local_cons_idx = octboot_net_circq_inc(tq->local_cons_idx,
			     tq->mask);
		descq->cons_idx = octboot_net_circq_inc(descq->cons_idx, tq->mask);
	}

	pthread_spin_init(&tq->lock, PTHREAD_PROCESS_SHARED);
	tq->status = OCTBOOT_NET_DESCQ_READY;

	/* tq status needs to be updated before memwrite */
	dump_hw_descq(descq);

	/* Add sync synchronize and  Do it with a for loop similar to mmi_memwrite() */
	mmio_memwrite(tq->hw_descq, descq, descq_tot_size);
	free(descq);
	return 0;

error:
	for (j = 0; j < i; j++) {
		skb = tq->skb_list[j];
		dma = tq->dma_list[j];
		if (skb) {
			dma_unmap(mdev->dev, skb->data, skb->dma, OCTBOOT_NET_RX_BUF_SIZE);
			free(skb);
		}
		tq->skb_list[j] = NULL;
		tq->dma_list[j] = 0;
	}
	tq->local_prod_idx = 0;
	tq->local_cons_idx = 0;
	free(descq);
	free(tq->skb_list);
	free(tq->dma_list);
	return 0;
}

static int mdev_setup_tx_rings(struct octboot_net_dev *mdev)
{
	int i, j, ret;

	for  (i = 0; i < mdev->num_txq && i < OCTBOOT_NET_MAXQ; i++) {
		ret = mdev_setup_tx_ring(mdev, i);
		if (ret)
			goto error;
	}
	return 0;
error:
	for (j = 0; j < i; j++)
		mdev_clean_tx_ring(mdev, j);
	return ret;
}

static void mdev_clean_rx_ring(struct octboot_net_dev *mdev, int q_idx)
{
	struct octboot_net_sw_descq *rq = &mdev->rxq[q_idx];
	int cons_idx, prod_idx;
	struct sk_buff *skb;
	int descq_tot_size;
	int start, count;
	int i;

	if (rq->status == OCTBOOT_NET_DESCQ_CLEAN)
		return;
	cons_idx = rq->local_cons_idx;
	prod_idx = rq->local_prod_idx;
	count = octboot_net_circq_depth(prod_idx, cons_idx, rq->mask);
	descq_tot_size = sizeof(struct octboot_net_hw_descq) +
		(rq->element_count * sizeof(struct octboot_net_hw_desc_ptr));
	start = cons_idx;
	for (i = 0; i <= count; i++) {
		skb = rq->skb_list[start];
		if (skb) {
			dma_unmap(mdev->dev, (uint64_t)rq->dma_list[start], skb->dma,
					 OCTBOOT_NET_RX_BUF_SIZE);
			free(skb);
			rq->skb_list[start] = NULL;
			rq->dma_list[start] = 0;
			start = octboot_net_circq_inc(start, rq->mask);
		}
	}
	rq->local_prod_idx = rq->local_cons_idx = 0;
	*rq->cons_idx_shadow = 0;
	free(rq->skb_list);
	free(rq->dma_list);
	rq->status = OCTBOOT_NET_DESCQ_CLEAN;

	/* rq needs to be updated before memset */
	mmio_memset(rq->hw_descq, 0, descq_tot_size);
}

static void mdev_clean_rx_rings(struct octboot_net_dev *mdev)
{
	int i;

	for (i = 0; i < mdev->num_rxq && i < OCTBOOT_NET_MAXQ; i++)
		mdev_clean_rx_ring(mdev, i);
}

static int mdev_setup_rx_ring(struct octboot_net_dev *mdev, int q_idx)
{
	int element_count = mdev->element_count;
	struct octboot_net_hw_desc_ptr *ptr;
	struct octboot_net_hw_descq *descq;
	struct octboot_net_sw_descq *rq;
	int i, j, ret, count;
	struct sk_buff *skb;
	int descq_tot_size;
	uint64_t  dma, data;

	rq = &mdev->rxq[q_idx];
	rq->priv = mdev;
	descq_tot_size = sizeof(struct octboot_net_hw_descq) + (element_count *
		      sizeof(struct octboot_net_hw_desc_ptr));
	descq = calloc(1, descq_tot_size);
	if (!descq) {
		printf("octboot_net: rq descq alloc failed\n");
		return -ENOMEM;
	}

	rq->local_prod_idx = 0;
	rq->local_cons_idx = 0;
	rq->element_count = element_count;
	rq->mask = element_count - 1;
	rq->q_num = q_idx;
	rq->skb_list = calloc(1, sizeof(char *) * element_count);
	if (!rq->skb_list) {
		free(descq);
		printf("octboot_net: rq skb_list  alloc failed\n");
		return -ENOMEM;
	}

	rq->dma_list = calloc(1, sizeof(char *) * element_count);
	if (!rq->dma_list) {
		free(descq);
		free(rq->skb_list);
		printf("octboot_net: rq dma_list  alloc failed\n");
		return -ENOMEM;
	}

	descq->num_entries = element_count;
	descq->buf_size = OCTBOOT_NET_RX_BUF_SIZE;
	rq->cons_idx_shadow = mdev->rq_cons_shdw_vaddr + q_idx;
	descq->shadow_cons_idx_addr = mdev->rq_cons_shdw_dma +
		(q_idx * sizeof(*rq->cons_idx_shadow));
	*rq->cons_idx_shadow = 0;
	count = octboot_net_circq_space(rq->local_prod_idx, rq->local_cons_idx,
		  rq->mask);
	data = dma_alloc(mdev->dev, 0, &dma, (count + 1) * OCTBOOT_NET_RX_BUF_SIZE);

	for (i = 0; i <= count; i++) {
		skb = calloc(1, sizeof(struct sk_buff));  // check it
		if (!skb) {
			printf("octboot_net: skb alloc failed\n");
			ret = -ENOMEM;
			goto error;
		}

		skb->dev = mdev->dev;
		skb->data = data + i * OCTBOOT_NET_RX_BUF_SIZE;
		skb->dma = dma + i * OCTBOOT_NET_RX_BUF_SIZE;

		if (!skb->data) {
			printf("octboot_net: dma mapping failed\n");
			free(skb);
			ret = -ENOENT;
			goto error;
		}

		ptr = &descq->desc_arr[rq->local_prod_idx];
		memset(ptr, 0, sizeof(struct octboot_net_hw_desc_ptr));
		ptr->hdr.s_mgmt_net.ptr_type = OCTBOOT_NET_DESC_PTR_DIRECT;
		ptr->ptr = skb->dma;
		rq->skb_list[rq->local_prod_idx] = skb;
		rq->dma_list[rq->local_prod_idx] = skb->data;
		rq->local_prod_idx = octboot_net_circq_inc(rq->local_prod_idx,
			     rq->mask);
		descq->prod_idx = octboot_net_circq_inc(descq->prod_idx, rq->mask);
	}

	rq->hw_descq = RX_DESCQ_OFFSET(mdev) + (q_idx * descq_tot_size);
	rq->hw_prod_idx = (uint32_t *)(rq->hw_descq +
		       offsetof(struct octboot_net_hw_descq, prod_idx));

	rq->status = OCTBOOT_NET_DESCQ_READY;

	/* rq needs to be updated before memwrite */
	pthread_spin_init(&rq->lock, PTHREAD_PROCESS_SHARED);
	dump_hw_descq(descq);
	mmio_memwrite(rq->hw_descq, descq, descq_tot_size);
	free(descq);
	return 0;
error:
	for (j = 0; j < i; j++) {
		skb = rq->skb_list[j];
		dma = rq->dma_list[j];
		if (skb) {
			dma_unmap(mdev->dev, skb->data, (uint64_t *)skb->dma,
							OCTBOOT_NET_RX_BUF_SIZE);
			free(skb);
		}
		rq->skb_list[j] = NULL;
		rq->dma_list[j] = 0;
	}
	rq->local_prod_idx = 0;
	rq->local_cons_idx = 0;
	free(descq);
	free(rq->skb_list);
	free(rq->dma_list);
	return ret;
}

static int mdev_setup_rx_rings(struct octboot_net_dev *mdev)
{
	int i, j, ret;

	for (i = 0; i < mdev->num_rxq && i < OCTBOOT_NET_MAXQ; i++) {
		ret = mdev_setup_rx_ring(mdev, i);
		if (ret)
			goto error;
	}
	return 0;
error:
	for (j = 0; j < i; j++)
		mdev_clean_rx_ring(mdev, j);
	return ret;
}

static int mdev_reinit_rings(struct octboot_net_dev *mdev)
{
	int ret;

	mdev_clean_tx_rings(mdev);
	mdev_clean_rx_rings(mdev);
	ret = mdev_setup_tx_rings(mdev);
	if (ret)
		return ret;
	ret = mdev_setup_rx_rings(mdev);
	if (ret)
		mdev_clean_tx_rings(mdev);
	return ret;
}

static int handle_target_status(struct octboot_net_dev *mdev)
{
	uint64_t target_status;
	uint64_t cur_status;
	int ret = 0;

	cur_status = get_host_status(mdev);
	target_status = get_target_status(mdev);
	printf("host status %lu\n", cur_status);
	printf("target status %lu\n", target_status);
	printf("\n current = %ld", cur_status);
	switch (cur_status) {
	case OCTNET_HOST_READY:
		if (target_status == OCTNET_TARGET_RUNNING) {
			printf("octboot_net: target running\n");
			change_host_status(mdev, OCTNET_HOST_RUNNING, false);
			//set_tap_carrier(mdev->tdev->tap_name, 1);
		}
		break;
	case OCTNET_HOST_RUNNING:
		target_status = get_target_status(mdev);
		if (target_status != OCTNET_TARGET_RUNNING) {
			printf("octboot_net: target stopped\n");
			change_host_status(mdev, OCTNET_HOST_GOING_DOWN,
						   false);
			//set_tap_carrier(mdev->tdev->tap_name, 0);
			ret = mdev_reinit_rings(mdev);
			if (ret) {
				change_host_status(mdev, OCTNET_HOST_FATAL,
						   false);
				return ret;
			}
			change_host_status(mdev, OCTNET_HOST_READY, false);
		}
		break;
	default:
		printf("octboot_net: unhandled state transition host_status:%lu target_status %lu\n",
		       cur_status, target_status);
		break;
	}
	return ret;
}

/* For every octbootnet we require one separate thread;*/
static void *octboot_net_task(void *devm)
{
	union octboot_net_mbox_msg msg;
	int ret;
	pthread_t thread;
	int q_num = 0, index;
	struct octboot_net_dev *mdev = (struct octboot_net_dev *)devm;

	atomic_flag_test_and_set(&mdev->task_status);

	index = find_octboot_net_entry(mdev->dev);
	if ((index >= 0) && is_flr_inprogress(index)) {
		atomic_flag_clear(&mdev->task_status);
		__sync_synchronize();
		perror("FLR in progress\n");
		return NULL;
	}
	ret = mbox_check_msg_rcvd(mdev, &msg);
	if (!ret) {
		switch (msg.s.hdr.opcode) {
		case OCTBOOT_NET_MBOX_TARGET_STATUS_CHANGE:
			handle_target_status(mdev);
			if (msg.s.hdr.req_ack)
				set_host_mbox_ack_reg(mdev, msg.s.hdr.id);
			break;
		case OCTBOOT_NET_MBOX_OPCODE_INVALID:
			/* Return from octboot_net_task */
			return NULL;
		default:
			break;
		}
	}

	if (pthread_spin_trylock(&mdev->rxq[q_num].lock)) {

		__handle_rxq(mdev, q_num, 2);
		pthread_spin_unlock(&mdev->rxq[q_num].lock);
	}

	index = find_octboot_net_entry(mdev->dev);
	if ((index >= 0) && is_flr_inprogress(index)) {
		printf(&(mdev->pdev)->dev, "net_task:End in PFFLR State");
		atomic_flag_clear(&mdev->task_status);
		__sync_synchronize();
		return NULL;
	}

	atomic_flag_clear(&mdev->task_status);
}

static void mgmt_init_work(void *bar4_addr, int index)
{
	uint32_t *tq_cons_shdw_vaddr, *rq_cons_shdw_vaddr;
	uint32_t *tq_cons_shdw_dma, *rq_cons_shdw_dma;
	int num_txq, num_rxq, max_rxq, max_txq, ret;
	char octboot_iface_name[OCTBOOT_IFACE_NAME_SZ];
	struct octboot_net_dev *mdev;
	struct pci_dev *octnet_pci_device;
	uint32_t host_version;
	uint32_t target_version;
	mrvl_pcie_dev_t *moct_dev;
	char tap_dev_name[16];

	moct_dev = octboot_struct[index].mrvl_pdev;
	octnet_pci_device = octboot_struct[index].octnet_pci_dev_arr;
	max_txq = num_txq = OCTBOOT_NET_MAXQ;
	max_rxq = num_rxq = OCTBOOT_NET_MAXQ;
	tq_cons_shdw_vaddr = (uint32_t *)dma_alloc(moct_dev, 0, &tq_cons_shdw_dma,
							(sizeof(uint32_t) * num_txq));
	if (tq_cons_shdw_vaddr == NULL) {
		printf("octboot_net: dma_alloc_coherent tq failed\n");
		ret = -ENOMEM;
		goto conf_err;
	}
	rq_cons_shdw_vaddr = (uint32_t *)dma_alloc(moct_dev, 0, (uint64_t *)&rq_cons_shdw_dma,
								(sizeof(uint32_t) * num_rxq));
	if (rq_cons_shdw_vaddr == NULL) {
		ret = -ENOMEM;
		printf("octboot_net: dma_alloc_coherent rq failed\n");
		goto tq_dma_free;
	}

	/* we support only single queue at this time */
	mdev = calloc(1, sizeof(struct octboot_net_dev));
	if (!mdev) {
		ret = -ENOMEM;
		printf("octboot_net: alloc_netdev failed\n");
		goto rq_dma_free;
	}
	mdev->tdev = calloc(1, sizeof(struct tapdev));

	/* We need to create TAP device here so that FDs
	 * can be easily tracked and also in proper flow. */
	snprintf(tap_dev_name, 16, "octboot_net%d", index);
	printf("\noctboot_net%d pci_info %02x:%02x.%x\n", index, moct_dev->bus,
					moct_dev->dev, moct_dev->func);
	int octnet_tapfd = tap_alloc(tap_dev_name, IFF_TAP | IFF_NO_PI);  /* tap interface */
	memcpy(mdev->tdev->tap_name, tap_dev_name, strlen(tap_dev_name));


	if (octnet_tapfd < 0) {
		perror("\nAllocating tap interface\n");
		goto free_net;
	}

	event.data.fd = octnet_tapfd;
	event.events = EPOLLIN;
	ret = epoll_ctl(epollfd, EPOLL_CTL_ADD, octnet_tapfd, &event);
	if (ret == -1) {
		perror("epoll_ctl failed\n");
		goto free_net;
	}

	mdev->tap_fd = octnet_tapfd;
	mdev->admin_up = false;
	mdev->bar_map = bar4_addr;
	mdev->dev = moct_dev;
	mdev->max_txq = max_txq;
	mdev->max_rxq = max_rxq;
	mdev->num_txq = num_txq;
	mdev->num_rxq = num_rxq;
	mdev->element_count = OCTBOOT_NET_NUM_ELEMENTS;
	mdev->tq_cons_shdw_vaddr = tq_cons_shdw_vaddr;
	mdev->tq_cons_shdw_dma   = (uint64_t)tq_cons_shdw_dma;
	mdev->rq_cons_shdw_vaddr = rq_cons_shdw_vaddr;
	mdev->rq_cons_shdw_dma   = (uint64_t)rq_cons_shdw_dma;
	ret = mdev_setup_tx_rings(mdev);
	if (ret) {
		printf("octboot_net setup tx rings failed\n");
		goto free_net;
	}

	ret = mdev_setup_rx_rings(mdev);
	if (ret) {
		printf("octboot_net: setup rx rings failed\n");
		goto clean_tx_ring;
	}

	mdev->send_mbox_id = 0;
	mdev->recv_mbox_id = 0;
	pthread_mutex_init(&mdev->mbox_lock, NULL);
	pthread_mutex_init(&mdev->service_task_lock, NULL);
	mdev->service_task_state = 1;

	change_host_status(mdev, OCTNET_HOST_READY, false);

	gmdev[index] = mdev;
	octboot_struct[index].mdev = mdev;
	octboot_struct[index].octboot_net_init_done = 1;
	octboot_net_init_done[index] = 1;
	host_version = ((OCTBOOT_NET_VERSION_MAJOR << 8)|OCTBOOT_NET_VERSION_MINOR);
	writeq(host_version, HOST_VERSION_REG(mdev));
	target_version = get_target_version(mdev);
	if ((host_version >> 8) == (target_version >> 8))
		printf("octboot_net driver compatible with uboot\n");
	else
		printf("octboot_net driver Incompatible with uboot\n");

	return;
destroy_mutex:
	pthread_mutex_destroy(&mdev->mbox_lock);
clean_rx_ring:
	mdev_clean_rx_rings(mdev);
clean_tx_ring:
	mdev_clean_tx_rings(mdev);
free_net:
	close(mdev->tap_fd);
rq_dma_free:
	dma_unmap(moct_dev,
		  rq_cons_shdw_vaddr,
		  rq_cons_shdw_dma,
		  (sizeof(uint32_t) * num_rxq));
tq_dma_free:
	dma_unmap(moct_dev,
		  tq_cons_shdw_vaddr,
		  tq_cons_shdw_dma,
		  (sizeof(uint32_t) * num_txq));
conf_err:
	printf("octboot_net: init failed; error = %d\n", ret);
	return;
}

void octboot_net_clean()
{
	int q_num = 0;
	struct octboot_net_dev *mdev;
	mrvl_pcie_dev_t *mpci;
	printf("\n %s called \n", __func__);

	for (int i = 0; i < MAX_MRVL_OCT_DEV; i++) {
		if(!octboot_struct[i].initialized)
			continue;
		if (!octboot_struct[i].octboot_net_init_done)
			continue;
		mdev = gmdev[i];
		mpci = octboot_struct[i].mrvl_pdev;
		change_host_status(mdev, OCTNET_HOST_GOING_DOWN, false);
		pthread_mutex_destroy(&mdev->mbox_lock);
		close(mdev->tap_fd);
		mdev_clean_rx_rings(mdev);
		mdev_clean_tx_rings(mdev);
		dma_unmap(mdev->dev,
			mdev->rq_cons_shdw_vaddr,
			mdev->rq_cons_shdw_dma,
			 (sizeof(uint64_t) * mdev->num_rxq));
		dma_unmap(mdev->dev,
			mdev->tq_cons_shdw_vaddr,
			mdev->tq_cons_shdw_dma,
			(sizeof(uint64_t) * mdev->num_txq));
		set_host_reset_status(mdev, true);
		if (mpci->pacc)
			pci_cleanup(mpci->pacc);
		gmdev[i] = NULL;
		pcie_bind(mpci, false);
	}

	if (pacc)
		pci_cleanup(pacc);
	pthread_mutex_destroy(&init_task_lock);
}
