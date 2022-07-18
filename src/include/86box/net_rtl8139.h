/*
 * 86Box     A hypervisor and IBM PC system emulator that specializes in
 *           running old operating systems and software designed for IBM
 *           PC systems and compatibles from 1981 through fairly recent
 *           system designs based on the PCI bus.
 *
 *           This file is part of the 86Box distribution.
 *
 *           Definitions for the RTL8129/RTL8139 ethernet controller.
 *
 * Authors: Jasmine Iwanek, <jasmine@iwanek.co.uk>
 *
 *          Copyright 2022-2025 Jasmine Iwanek.
 */
#ifndef NET_RTL8139_H
# define NET_RTL8139_H

enum {
    RTL8129   = 0,   /* 32-bit PCI Realtek 8129   */
    RTL8139C  = 1,   /* 32-bit PCI Realtek 8139C  */
    RTL8139CP = 2,   /* 32-bit PCI Realtek 8139C+ */
    RTL8139D  = 3    /* 32-bit PCI Realtek 8139D  */
};

#endif  /*NET_RTL8139_H*/
