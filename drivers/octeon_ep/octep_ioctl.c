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
#include "octep_ctrl_net.h"
#include "octep_pfvf_mbox.h"
#include "octep_ioctl.h"

/* ifnet and ioctl functions */
int
octep_change_mtu(if_t ifp, int new_mtu)
{
	struct octep_device *oct = if_getsoftc(ifp);
	struct octep_iface_link_info *link_info;
	int err = 0;

	link_info = &oct->link_info;

	if ((new_mtu < OCTEP_MIN_MTU_SIZE) || (new_mtu > oct->max_rx_pktlen)) {
		dev_err(oct->pdev, "Invalid MTU: %d Valid range from: %d to: %d\n", new_mtu,
				OCTEP_MIN_MTU_SIZE, oct->max_rx_pktlen);
		return (EINVAL);
	}

	if (link_info->mtu == new_mtu)
		return 0;

	mtx_lock(&oct->lock);
	err = octep_ctrl_net_set_mtu(oct, OCTEP_CTRL_NET_INVALID_VFID, new_mtu, true);
	if (!err) {
		oct->link_info.mtu = new_mtu;
		if_setmtu(ifp, new_mtu);
	}
	mtx_unlock(&oct->lock);

	return (err);
}

int
octep_set_capabilities(if_t ifp, int reqcap)
{
	struct octep_device *oct = if_getsoftc(ifp);
	struct octep_ctrl_net_offloads offloads = { 0 };
	int features, changed, error;

	features = reqcap & if_getcapabilities(ifp);
	if (features != reqcap) {
		dev_err(oct->pdev, "Unsupported capabilities requested: 0x%x\n",
				reqcap & ~if_getcapabilities(ifp));
		return (EINVAL);
	}

	mtx_lock(&oct->lock);

	changed = features ^ if_getcapenable(ifp);

	if (!changed)
	{
		mtx_unlock(&oct->lock);
		return 0;
	}

	if (features & (IFCAP_TXCSUM | IFCAP_TXCSUM_IPV6)) {
		offloads.tx_offloads |= OCTEP_TX_OFFLOAD_CKSUM;
	}
	if (features & (IFCAP_TSO4 | IFCAP_TSO6)) {
		offloads.tx_offloads |= OCTEP_TX_OFFLOAD_TSO;
	}
	if (features & (IFCAP_RXCSUM | IFCAP_RXCSUM_IPV6)) {
		offloads.rx_offloads |= OCTEP_RX_OFFLOAD_CKSUM;
	}

	error = octep_ctrl_net_set_offloads(oct, OCTEP_CTRL_NET_INVALID_VFID, &offloads, true);
	if (error) {
		dev_err(oct->pdev, "Failed to set offloads: %d\n", error);
		mtx_unlock(&oct->lock);
		return error;
	}

	if (changed & (IFCAP_TXCSUM | IFCAP_TXCSUM_IPV6)) {
		if_togglecapenable(ifp, IFCAP_TXCSUM);
		if_togglecapenable(ifp, IFCAP_TXCSUM_IPV6);
		if (if_getcapenable(ifp) & (IFCAP_TXCSUM | IFCAP_TXCSUM_IPV6)) {
			if_sethwassistbits(ifp, CSUM_IP | CSUM_TCP | CSUM_UDP | CSUM_TCP_IPV6 | CSUM_UDP_IPV6, 0);
		} else {
			if_sethwassistbits(ifp, 0, CSUM_IP | CSUM_TCP | CSUM_UDP | CSUM_TCP_IPV6 | CSUM_UDP_IPV6);
		}
	}
	if (changed & (IFCAP_RXCSUM | IFCAP_RXCSUM_IPV6)) {
		if_togglecapenable(ifp, IFCAP_RXCSUM);
		if_togglecapenable(ifp, IFCAP_RXCSUM_IPV6);
	}
	if (changed & (IFCAP_TSO4 | IFCAP_TSO6)) {
		if_togglecapenable(ifp, IFCAP_TSO4);
		if_togglecapenable(ifp, IFCAP_TSO6);
	}
	if (changed & IFCAP_LRO) {
		if_togglecapenable(ifp, IFCAP_LRO);
	}

	mtx_unlock(&oct->lock);

	return 0;
}



int
octep_ioctl(if_t ifp, u_long cmd, caddr_t data)
{
	struct octep_device *oct = if_getsoftc(ifp);
	struct ifreq *ifrequest = (struct ifreq *)data;
	int error = 0;

	mtx_lock(&oct->lock);
	switch (cmd) {
	case SIOCSIFADDR:
		if_setflagbits(ifp, IFF_UP, 0);
		error = ether_ioctl(ifp, cmd, data);
		break;
	case SIOCSIFMTU:
		error = octep_change_mtu(ifp, ifrequest->ifr_mtu);
		break;
	case SIOCSIFFLAGS:
		if (if_getflags(ifp) & IFF_UP) {
			if (if_getdrvflags(ifp) & IFF_DRV_RUNNING) {
				if ((if_getflags(ifp) ^ oct->if_flags) & (IFF_PROMISC | IFF_ALLMULTI)) {
					dev_info(oct->pdev, "Handle promiscuous/multicast changes not supported\n");
				}
			} else {
				octep_open(oct);
			}
		} else {
			if (if_getdrvflags(ifp) & IFF_DRV_RUNNING) {
				octep_stop(ifp);
			}
		}
		oct->if_flags = if_getflags(ifp);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if (if_getdrvflags(ifp) & IFF_DRV_RUNNING) {
			dev_dbg(oct->pdev, "ioctl: SIOCSIFMULTI\n");
		}
		break;
	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		error = ifmedia_ioctl(ifp, ifrequest, &oct->ifmedia, cmd);
		break;
	case SIOCSIFCAP:
		error = octep_set_capabilities(ifp, ifrequest->ifr_reqcap);
		break;
	default:
		error = ether_ioctl(ifp, cmd, data);
		break;
	}

	mtx_unlock(&oct->lock);

	return error;
}

