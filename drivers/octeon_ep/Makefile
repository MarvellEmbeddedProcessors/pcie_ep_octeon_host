.PATH:  ${.CURDIR}

KMOD = octep_driver
CFLAGS += -I${.CURDIR}

SRCS = device_if.h bus_if.h pci_if.h pci_iov_if.h vnode_if.h opt_inet.h opt_inet6.h
SRCS += vnode_if_newproto.h vnode_if_typedef.h
SRCS += octep_main.c octep_cnxk_pf.c octep_pfvf_mbox.c octep_ctrl_mbox.c octep_ctrl_net.c octep_cn9k_pf.c octep_ioctl.c octep_tx.c octep_rx.c

.include <bsd.kmod.mk>
