/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2024 Marvell.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/epoll.h>
#include <sys/mount.h>
#include <sys/types.h>
#include <cuse_lowlevel.h>
#include <fuse_opt.h>
#include <pthread.h>
#include "mrvl_cuse.h"

#define		MIO_PTP_BAR4_REGION               14
#define		MIO_PTP_CLOCK_HI_OFFSET           0xf10
#define		OCTEON_BAR_4_MAPPING_SIZE       (4*1024*1024)

extern mrvl_pcie_dev_t *oct_pcie_dev_array[MAX_MRVL_OCT_DEV];

void reverse(char str[], int length)
{
	int start = 0;
	int end = length - 1;

	while (start < end) {
		char temp = str[start];

		str[start] = str[end];
		str[end] = temp;
		end--;
		start++;
	}
}

char *citoa(uint64_t num, char *str, int base)
{
	int i = 0;
	bool isNegative = false;

	if (num == 0) {
		str[i++] = '0';
		str[i] = '\0';
		return str;
	}

	while (num != 0) {
		int rem = num % base;
		str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
		num = num / base;
	}

	str[i] = '\0';

	// Reverse the string
	reverse(str, i);

	return str;
}

static void mrvl_fuse_open(fuse_req_t req, struct fuse_file_info *fi)
{
	fuse_reply_open(req, fi);
}

static void mrvl_fuse_read(fuse_req_t req, size_t size, off_t off,
			 struct fuse_file_info *fi)
{

	char buf[17] = {0};
	uint64_t timestamp;

	mrvl_pcie_dev_t *dev = fuse_req_userdata(req);

	if (!dev) {
		printf("\n dev is NULL func %s line %d\n",__func__,__LINE__);
		fuse_reply_buf(req, buf, 17);
		return;
	}

	/* Time should contain timestamp read from Bar4, so read BAR4 here*/
	timestamp = readq(dev->bar4_addr + MIO_PTP_BAR4_REGION *
				OCTEON_BAR_4_MAPPING_SIZE + MIO_PTP_CLOCK_HI_OFFSET);
	citoa(timestamp, buf, 16);
	fuse_reply_buf(req, buf, 17);
	printf("\n func %s line  %d buf %s\n",__func__, __LINE__, buf);
}

static const struct cuse_lowlevel_ops mrvl_fops = {
	.open = mrvl_fuse_open,
	.read = mrvl_fuse_read,
};

static void *cuse_worker(void *arg)
{
	struct fuse_session *se = arg;
	int ret;

	ret = fuse_session_loop(se);
	cuse_lowlevel_teardown(se);

	return (void *)(unsigned long)ret;
}

int mrvl_fuse_init(mrvl_pcie_dev_t *mdev, int index)
{
	int ret;
	int multithreaded = 0;
	char buf[128];
	const char *bufp[] = {buf};
	static const char * const argv[] = {"./daemon_test", "-f"};
	struct cuse_info ci = {.dev_info_argc = 1,
				.dev_info_argv = bufp,
				.flags = CUSE_UNRESTRICTED_IOCTL};
	static const struct cuse_lowlevel_ops *ops = &mrvl_fops;


	snprintf(buf, sizeof(buf), "DEVNAME=mrvl_ptp%d", index);
	printf("\n mrvl_ptp%d pci_device %02x:%02x:%x\n",
		       index, mdev->bus, mdev->dev, mdev->func);
	mdev->oct.fuse_sessions = cuse_lowlevel_setup(sizeof(argv)/sizeof(char *),
						(char **)argv,
						&ci, ops, &multithreaded, mdev);
	if (!mdev->oct.fuse_sessions) {
		printf("Failed to setup CUSE %s\n", buf);
		return -1;
	}

	ret = pthread_create(&mdev->oct.fuse_thread, NULL, cuse_worker,
				mdev->oct.fuse_sessions);
	if (ret) {
		cuse_lowlevel_teardown(mdev->oct.fuse_sessions);
		return ret;
	}

	return 0;
}

void mrvl_fuse_clean(mrvl_pcie_dev_t *mdev)
{
		pthread_kill(mdev->oct.fuse_thread, 0);
		cuse_lowlevel_teardown(mdev->oct.fuse_sessions);
}
