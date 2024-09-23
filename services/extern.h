/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2024 Marvell.
 */

#include "mrvl_cuse.h"
#include <sys/epoll.h>
volatile extern int epollfd;
volatile extern struct epoll_event events[MAXEVENTS];
volatile extern struct epoll_event event;
volatile extern int timerfd;
extern int current_oct_pcie_dev_cnt;
extern struct pci_access *pacc;
volatile extern int term_reaceived;
