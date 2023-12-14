/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of Intel 82815EP MCH Bridge
 *
 *
 *
 * Authors: Tiseno100,
 *          Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2022      Tiseno100.
 *          Copyright 2022-2023 Jasmine Iwanek.
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include "cpu.h"
#include "x86.h"
#include <86box/timer.h>
#include <86box/io.h>
#include <86box/device.h>
#include <86box/plat_unused.h>

#include <86box/mem.h>
#include <86box/pci.h>
#include <86box/smram.h>
#include <86box/spd.h>
#include <86box/chipset.h>

#define ENABLE_INTEL_815EP_LOG 1
#ifdef ENABLE_INTEL_815EP_LOG
int intel_815ep_do_log = ENABLE_INTEL_815EP_LOG;
static void
intel_815ep_log(const char *fmt, ...)
{
    va_list ap;

    if (intel_815ep_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define intel_815ep_log(fmt, ...)
#endif

typedef struct intel_815ep_t {
    uint8_t pci_conf[256];
    uint8_t pci_slot;

    smram_t *lsmm_segment;
} intel_815ep_t;

static void
intel_lsmm_segment_recalc(intel_815ep_t *dev, uint8_t val)
{
    intel_815ep_log("Intel 815EP MCH: LSMM update to status %d\n", val); /* Check the 815EP datasheet for status */

    switch (val) {
        case 0:
            smram_disable(dev->lsmm_segment);
            break;

        case 1:
            smram_enable(dev->lsmm_segment, 0x000a0000, 0x000a0000, 0x20000, 1, 0);
            break;

        case 2:
            smram_enable(dev->lsmm_segment, 0x000a0000, 0x000a0000, 0x20000, 0, 0);
            break;

        case 3:
            smram_enable(dev->lsmm_segment, 0x000a0000, 0x000a0000, 0x20000, 0, 1);
            break;
    }
}

static void
intel_pam_recalc(int addr, uint8_t val)
{
    int region = 0xc0000 + ((addr - 0x5a) << 15);

    if (addr == 0x59)
        mem_set_mem_state_both(0xf0000, 0x10000, ((val & 0x10) ? MEM_READ_INTERNAL : MEM_READ_EXTANY) | ((val & 0x20) ? MEM_WRITE_INTERNAL : MEM_WRITE_EXTANY));
    else {
        mem_set_mem_state_both(region, 0x4000, ((val & 0x01) ? MEM_READ_INTERNAL : MEM_READ_EXTANY) | ((val & 0x02) ? MEM_WRITE_INTERNAL : MEM_WRITE_EXTANY));
        mem_set_mem_state_both(region + 0x4000, 0x4000, ((val & 0x10) ? MEM_READ_INTERNAL : MEM_READ_EXTANY) | ((val & 0x20) ? MEM_WRITE_INTERNAL : MEM_WRITE_EXTANY));
    }
}

static void
intel_815ep_write(int func, int addr, uint8_t val, void *priv)
{
    intel_815ep_t *dev = (intel_815ep_t *) priv;

    intel_815ep_log("Intel 815EP MCH: dev->regs[%02x] = %02x\n", addr, val);

    switch (addr) {
        case 0x51:
            dev->pci_conf[addr] = val & 2; // Brute force to AGP Mode
            break;

        case 0x52:
        case 0x54:
            dev->pci_conf[addr] = val & ((addr & 4) ? 0x0f : 0xff);
            spd_write_drbs_intel_815ep(dev->pci_conf);
            break;

        case 0x59 ... 0x5f:
            dev->pci_conf[addr] = val;
            intel_pam_recalc(addr, val);
            break;

        case 0x70:
            dev->pci_conf[addr] = val;
            intel_lsmm_segment_recalc(dev, (val >> 2) & 3);
            break;

        default:
            dev->pci_conf[addr] = val;
            break;
    }
}

static uint8_t
intel_815ep_read(int func, int addr, void *priv)
{
    const intel_815ep_t *dev = (intel_815ep_t *) priv;

    intel_815ep_log("Intel 815EP MCH: dev->regs[%02x] (%02x)\n", addr, dev->pci_conf[addr]);
    return dev->pci_conf[addr];
}

static void
intel_815ep_reset(void *priv)
{
    intel_815ep_t *dev = (intel_815ep_t *) priv;
    memset(dev->pci_conf, 0x00, sizeof(dev->pci_conf)); /* Wash out the registers */

    /* VID - Vendor Identification Register */
    dev->pci_conf[0x00] = 0x86; /* Intel */
    dev->pci_conf[0x01] = 0x80;

    /* DID - Device Identification Register */
    dev->pci_conf[0x02] = 0x30; /* 815EP */
    dev->pci_conf[0x03] = 0x11;

    /* PCICMD - PCI Command Register */
    dev->pci_conf[0x04] = 0x06;

    /* PCISTS - PCI Status Register */
    dev->pci_conf[0x06] = 0x90;

    /* RID - Revision Identification Register */
    dev->pci_conf[0x08] = 0x02;

    /* BCC - Base Class Code Register */
    dev->pci_conf[0x0b] = 0x06;

    /* APBASE - Aperture Base Configuration Register */
    dev->pci_conf[0x10] = 0x03;

    /* CAPPTR - Capabilities Pointer */
    dev->pci_conf[0x34] = 0xa0;

    /* MCHCFG - MCH Configuration Register */
    dev->pci_conf[0x50] = 0x40;

    /* CAPID - Capability Identification */
    dev->pci_conf[0x88] = 0x09;
    dev->pci_conf[0x89] = 0xa0;
    dev->pci_conf[0x8a] = 0x04;
    dev->pci_conf[0x8b] = 0xf1;

    /* BUFF_SC - System Memory Buffer Strength Control Register */
    dev->pci_conf[0x92] = 0xff;
    dev->pci_conf[0x93] = 0xff;

    /* BUFF_SC2 - System Memory Buffer Strength Control Register 2 */
    dev->pci_conf[0x94] = 0xff;
    dev->pci_conf[0x95] = 0xff;

    /* ACAPID—AGP Capability Identifier Register */
    dev->pci_conf[0xa0] = 0x02;
    dev->pci_conf[0xa2] = 0x20;

    dev->pci_conf[0xa4] = 0x07;
    dev->pci_conf[0xa5] = 0x02;
    dev->pci_conf[0xa7] = 0x1f;

    for (int i = 0x58; i <= 0x5f; i++) /* Reset PAM to defaults */
        intel_pam_recalc(i, 0);

    intel_lsmm_segment_recalc(dev, 0); /* Reset LSMM SMRAM to defaults */
}

static void
intel_815ep_close(void *priv)
{
    intel_815ep_t *dev = (intel_815ep_t *) priv;

    smram_del(dev->lsmm_segment);
    free(dev);
}

static void *
intel_815ep_init(UNUSED(const device_t *info))
{
    intel_815ep_t *dev = (intel_815ep_t *) malloc(sizeof(intel_815ep_t));
    memset(dev, 0, sizeof(intel_815ep_t));

    /* Device */
    pci_add_card(PCI_ADD_NORTHBRIDGE, intel_815ep_read, intel_815ep_write, dev, &dev->pci_slot); /* Device 0: Intel 815EP MCH */

    /* AGP Bridge */
    device_add(&intel_815ep_agp_device);

    /* L2 Cache */
    cpu_cache_ext_enabled = 1;
    cpu_update_waitstates();

    /* LSMM SMRAM Segment */
    dev->lsmm_segment = smram_add();

    intel_815ep_reset(dev);
    return dev;
}

const device_t intel_815ep_device = {
    .name          = "Intel 815EP MCH Bridge",
    .internal_name = "intel_815ep",
    .flags         = DEVICE_PCI,
    .local         = 0,
    .init          = intel_815ep_init,
    .close         = intel_815ep_close,
    .reset         = intel_815ep_reset,
    { .available = NULL },
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
