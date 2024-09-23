/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2024 Marvell.
 */

#include <linux/if.h>
#include <linux/if_tun.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <error.h>
#include "mrvl_cuse.h"
#include <sys/types.h>
#include <sys/timerfd.h>
#include <sys/stat.h>
#include <sys/epoll.h>
#include <linux/types.h>
#include <time.h>
#include <signal.h>

#define SIGNATURE_OFFSET  0x2000000
#define NPU_HANDSHAKE_SIGNATURE 0xABCDABCD
#define MAX_PKT_READ_LEN 1500

#define    MIO_PTP_BAR4_REGION               14
#define    MIO_PTP_CLOCK_CFG_OFFSET          0xf00
#define    MIO_PTP_CLOCK_HI_OFFSET           0xf10
#define    MIO_PTP_CKOUT_THRESH_HI_OFFSET    0xf38
#define    MIO_PTP_CLOCK_SEC_OFFSET          0xfd0
#define	   BAR_4_MAPPING_SIZE       (4*1024*1024)

volatile int epollfd;
volatile struct epoll_event events[MAXEVENTS];
volatile struct epoll_event event;
volatile int timerfd;
volatile int term_reaceived;

int current_oct_pcie_dev_cnt;
mrvl_pcie_dev_t *oct_pcie_dev_array[MAX_MRVL_OCT_DEV];
pthread_t octboot_init_thread, octboot_service_thread;

int set_tap_carrier(char *tap_name, int carrier)
{
	struct ifreq ifr = {};
	int fd;

	if ((fd = open("/dev/net/tun", O_RDWR)) < 0)
		return fd;
	strncpy(ifr.ifr_name, tap_name, IFNAMSIZ);
//	ifr.ifr_flags = IFF_NO_CARRIER;
	int r = ioctl(fd, TUNSETCARRIER, carrier);
	if (r != 0)
		printf("ioctl(TUNSETIFF error set carrier %d)\n", carrier);
}

int tap_alloc(char *dev, int flags)
{
	int fd, err;
	struct ifreq ifr;

	/* open tun/tap device */
	if ((fd = open("/dev/net/tun", O_RDWR)) < 0)
		return fd;

	memset(&ifr, 0, sizeof(ifr));
	ifr.ifr_flags = flags;
	if (*dev)
		strncpy(ifr.ifr_name, dev, IFNAMSIZ);

	/* create the device */
	if ((err = ioctl(fd, TUNSETIFF, (void *) &ifr)) < 0) {
		close(fd);
		return err;
	}

	return fd;
}

uint32_t time_read(void)
{
	mrvl_pcie_dev_t *oct_dev = oct_pcie_dev_array[0];

	return readl(oct_dev->bar4_addr + 0x3800000 + 0xf10);
}

static void mrvl_set_timer(int timerfd, int interval)
{
	struct itimerspec tspec;

	tspec.it_interval.tv_sec = 0;
	tspec.it_interval.tv_nsec = (long)interval * 1000000;
	tspec.it_value.tv_sec = 0;
	tspec.it_value.tv_nsec = tspec.it_interval.tv_nsec;
	timerfd_settime(timerfd, 0, &tspec, NULL);
}

void clean_up_resources()
{
#ifdef PHC_SUPPORT
	for (int i = 0; i < current_oct_pcie_dev_cnt; i++)
		mrvl_fuse_clean(oct_pcie_dev_array[i]);
#endif
	octboot_net_clean();
}

void sig_term_handler(int signum, siginfo_t *info, void *ptr)
{
	if (!term_reaceived) {
		term_reaceived  = 1;
		clean_up_resources();
	}
}

void catch_sigterm()
{
    static struct sigaction _sigact;

    memset(&_sigact, 0, sizeof(_sigact));
    _sigact.sa_sigaction = sig_term_handler;
    _sigact.sa_flags = SA_SIGINFO;

    sigaction(SIGTERM, &_sigact, NULL);
}


void mrvl_run_daemon(void)
{
	char buffer[MAX_PKT_READ_LEN];
	uint64_t signature;
	int ret, i;

	memset(&event, 0, sizeof(event));
	memset(events, 0, sizeof(events));
	epollfd = epoll_create1(EPOLL_CLOEXEC);
	if (epollfd == -1) {
		printf("epoll_create1 failed\n");
		goto out;
	}

	timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
	if (timerfd == -1) {
		perror("\n timerfd creation failed\n");
		goto out;
	}

	mrvl_set_timer(timerfd, 1);
	event.data.fd = timerfd;
	event.events = EPOLLIN | EPOLLOUT;
	ret = epoll_ctl(epollfd, EPOLL_CTL_ADD, timerfd, &event);
	if (ret == -1) {
		perror("epoll_ctl failed\n");
		goto out;
	}

	ret = mrvl_pcie_dev_init(&oct_pcie_dev_array[0]);

	if (ret) {
		perror("\n mrvl_pcie_dev_init failed");
		goto failed;
	}

	if (pthread_create(&octboot_init_thread, NULL, octboot_net_init_work, &oct_pcie_dev_array)) {
		printf("\n octboot_net_init_work thread creation failed\n");
		goto failed;
	}

#ifdef PHC_SUPPORT
	/* Now, init cuse dev for PHC and octboot_net dev */
	for (i = 0; i < current_oct_pcie_dev_cnt; i++) {

		/* Init fuse file system, a character device to be used for PHC
		 * Don't exit if octbootnet_dev init has already been done
		 */
		ret = mrvl_fuse_init(oct_pcie_dev_array[i], i);

		if (ret) {
			printf("\n PHC ptp%d dev init failed\n",
						current_oct_pcie_dev_cnt - 1);
			goto failed;
		}
	}
#endif

	catch_sigterm();

	/* Need to see any unbind required in failure case, TODO*/
	if (pthread_create(&octboot_service_thread, NULL, poll_for_events_on_fds, NULL)) {
		printf("\n Poll Thread creation failed\n");
		goto failed;
	}

	pthread_join(octboot_service_thread, NULL);
failed:
	for (i = 0; i < current_oct_pcie_dev_cnt; i++)
		free(oct_pcie_dev_array[i]);
out:
	if (epollfd >= 0)
		close(epollfd);
	if (timerfd >= 0)
		close(timerfd);
}

void main(void)
{

	/* Daemonize */
	pid_t p_id = 0;
	pid_t sid = 0;

	p_id = fork();

	if (p_id < 0) {
		perror("Can't create the child proccess");
		exit(1);
	}

	// exit the parent process
	if (p_id > 0)
		exit(0);

	/* Giving the permesions for the process  (0 means that any files or
	 * directories created in this process will have maximum permissions)
	 */
	umask(0);

	/* set new session */
	sid = setsid();
	if (sid < 0)
		exit(1);
	/* Change the current working directory to root. */
	chdir("/");
	/* Close stdin. stdout and stderr */
#ifndef DEBUG
	close(STDIN_FILENO);
	close(STDOUT_FILENO);
	close(STDERR_FILENO);
#endif
	mrvl_run_daemon();
	return;
}
