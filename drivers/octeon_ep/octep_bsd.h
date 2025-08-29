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

#ifndef __OCTEP_BSD_H__
#define __OCTEP_BSD_H__

#include <sys/param.h>
#include <sys/gsb_crc32.h>
#include <sys/eventhandler.h>
#include <sys/socket.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sockio.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>

#include <net/if_types.h>
#include <net/if_vlan_var.h>

#include <netinet/in.h>
#include <netinet/tcp_lro.h>

#include <sys/bus.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <vm/vm.h>
#include <vm/pmap.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/smp.h>
#include <sys/kthread.h>
#include <sys/firmware.h>

#include <vm/vm_extern.h>
#include <vm/vm_kern.h>

#include <sys/nv.h>
#include <sys/iov_schema.h>
#include <dev/pci/pci_iov.h>

#ifndef unlikely
#define unlikely(x) __builtin_expect((x), 0)
#endif

#define mutex_init mtx_init
#define mutex_lock mtx_lock
#define mutex_unlock mtx_unlock
#define mutex_destroy mtx_destroy

#define OCTEP_CAST64(v)   ((long long)(long)(v))
#define BIT(nr)		(1UL << (nr))

#define dev_info(dev, format, args...)		\
	device_printf((dev), "Info: " format, ##args)
#define dev_warn(dev, format, args...)		\
	device_printf((dev), "Warn: " format, ##args)
#define dev_err(dev, format, args...)		\
	device_printf((dev), "Error: " format, ##args)

#ifdef OCTEP_DEBUG
#define dev_dbg(dev, format, args...)		\
	device_printf((dev), "Debug: " format, ##args)
#else
#define dev_dbg(dev, format, args...)	{do { } while (0); }
#endif

#endif	/* __OCTEP_BSD_H__ */
