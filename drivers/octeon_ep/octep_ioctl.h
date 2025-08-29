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

#ifndef _OCTEP_IOCTL_H_
#define _OCTEP_IOCTL_H_

#define OCTEP_MIN_MTU_SIZE 64

int octep_change_mtu(if_t ifp, int new_mtu);
int octep_set_capabilities(if_t ifp, int reqcap);
int octep_ioctl(if_t ifp, u_long cmd, caddr_t data);
void octep_open(void *arg);
int octep_stop(struct ifnet *ifp);


/* Link modes */
enum octep_link_mode_bit_indices {
    OCTEP_LINK_MODE_10GBASE_T    = 0,
    OCTEP_LINK_MODE_10GBASE_R,
    OCTEP_LINK_MODE_10GBASE_CR,
    OCTEP_LINK_MODE_10GBASE_KR,
    OCTEP_LINK_MODE_10GBASE_LR,
    OCTEP_LINK_MODE_10GBASE_SR,
    OCTEP_LINK_MODE_25GBASE_CR,
    OCTEP_LINK_MODE_25GBASE_KR,
    OCTEP_LINK_MODE_25GBASE_SR,
    OCTEP_LINK_MODE_40GBASE_CR4,
    OCTEP_LINK_MODE_40GBASE_KR4,
    OCTEP_LINK_MODE_40GBASE_LR4,
    OCTEP_LINK_MODE_40GBASE_SR4,
    OCTEP_LINK_MODE_50GBASE_CR2,
    OCTEP_LINK_MODE_50GBASE_KR2,
    OCTEP_LINK_MODE_50GBASE_SR2,
    OCTEP_LINK_MODE_50GBASE_CR,
    OCTEP_LINK_MODE_50GBASE_KR,
    OCTEP_LINK_MODE_50GBASE_LR,
    OCTEP_LINK_MODE_50GBASE_SR,
    OCTEP_LINK_MODE_100GBASE_CR4,
    OCTEP_LINK_MODE_100GBASE_KR4,
    OCTEP_LINK_MODE_100GBASE_LR4,
    OCTEP_LINK_MODE_100GBASE_SR4,
    OCTEP_LINK_MODE_NBITS
};

#define OCTEP_SET_LINK_MODES_BITMAP(octep_speeds) \
{ \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_10GBASE_T)) \
    ifmr->ifm_active |= IFM_10G_T; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_10GBASE_R)) \
    ifmr->ifm_active |= IFM_10G_KR; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_10GBASE_CR)) \
    ifmr->ifm_active |= IFM_10G_CR1; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_10GBASE_KR)) \
    ifmr->ifm_active |= IFM_10G_KR; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_10GBASE_LR)) \
    ifmr->ifm_active |= IFM_10G_LR; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_10GBASE_SR)) \
    ifmr->ifm_active |= IFM_10G_SR; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_25GBASE_CR)) \
    ifmr->ifm_active |= IFM_25G_CR; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_25GBASE_KR)) \
    ifmr->ifm_active |= IFM_25G_KR; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_25GBASE_SR)) \
    ifmr->ifm_active |= IFM_25G_SR; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_40GBASE_CR4)) \
    ifmr->ifm_active |= IFM_40G_CR4; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_40GBASE_KR4)) \
    ifmr->ifm_active |= IFM_40G_KR4; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_40GBASE_LR4)) \
    ifmr->ifm_active |= IFM_40G_LR4; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_40GBASE_SR4)) \
    ifmr->ifm_active |= IFM_40G_SR4; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_50GBASE_CR2)) \
    ifmr->ifm_active |= IFM_50G_CR2; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_50GBASE_KR2)) \
    ifmr->ifm_active |= IFM_50G_KR2;\
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_50GBASE_SR2)) \
    ifmr->ifm_active |= IFM_50G_SR2;\
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_50GBASE_CR)) \
    ifmr->ifm_active |= IFM_50G_CR2; \
    if ((octep_speeds) & BIT(OCTEP_LINK_MODE_50GBASE_KR)) \
    ifmr->ifm_active |= IFM_50G_KR2; \
}

#endif
