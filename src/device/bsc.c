/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          IBM BSC emulation.
 *
 * Authors: Jasmine Iwanek, <jasmine@iwanek.co.uk>
 *
 *          Copyright 2022-2025 Jasmine Iwanek.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/machine.h>
#include <86box/io.h>
#include <86box/pic.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/bsc.h>
#include <86box/plat_unused.h>

bsc_port_t	bsc_ports[BSC_MAX];

static uint8_t      next_bsc_inst = 0;
static bsc_device_t bsc_devices[BSC_MAX];

#ifdef ENABLE_BSC_LOG
int bsc_do_log = ENABLE_BSC_LOG;

static void
bsc_log(const char *fmt, ...)
{
    va_list ap;

    if (bsc_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define bsc_log(fmt, ...)
#endif

void
bsc_transmit_period(UNUSED(bsc_t *dev))
{
    // Not yet implemented
}

static void
bsc_transmit_timer(UNUSED(void *priv))
{
    // Not yet implemented
}

static void
bsc_timeout_timer(UNUSED(void *priv))
{
    // Not yet implemented
}

static void
bsc_update_speed(UNUSED(bsc_t *dev))
{
    // Not yet implemented
}

void
bsc_write(UNUSED(uint16_t addr), UNUSED(uint8_t val), UNUSED(void *priv))
{
    // Not yet implemented
}

uint8_t
bsc_read(UNUSED(uint16_t addr), UNUSED(void *priv))
{
    return 0;
}

static void
bsc_speed_changed(void *priv)
{
    bsc_t *dev = (bsc_t *) priv;

    bsc_update_speed(dev);
}

static void
bsc_close(void *priv)
{
    bsc_t *dev = (bsc_t *) priv;

    next_bsc_inst--;

    free(dev);
}

void
bsc_setup(bsc_t *dev, uint16_t addr, uint8_t irq)
{
    bsc_log("Adding BSC port %i at %04X...\n", dev->inst, addr);

    if (dev == NULL)
        return;

    if (!bsc_ports[dev->inst].enabled)
        return;
#if 0
    if (dev->base_address != 0x0000)
        bsc_remove(dev);
#endif
    dev->base_address = addr;
    if (addr != 0x0000)
        io_sethandler(addr, 0x000a, bsc_read, NULL, NULL, bsc_write, NULL, NULL, dev);
    dev->irq = irq;
}

static void *
bsc_init(UNUSED(const device_t *info))
{
    bsc_t *dev = (bsc_t *) calloc(1, sizeof(bsc_t));

    dev->inst = next_bsc_inst;

    if (bsc_ports[next_bsc_inst].enabled) {
        bsc_log("Adding BSC port %i...\n", next_bsc_inst);
        memset(&(bsc_devices[next_bsc_inst]), 0, sizeof(bsc_device_t));
        dev->bd         = &(bsc_devices[next_bsc_inst]);
        dev->bd->bsc = dev;
#if 0
        bsc_reset_port(dev);
#endif
        if (next_bsc_inst == 1)
            bsc_setup(dev, BSC2_ADDR, BSC2_IRQ);
        else if (next_bsc_inst == 0)
            bsc_setup(dev, BSC1_ADDR, BSC1_IRQ);

        /* Default to 9600,N,7. */
        dev->dlab      = 96;
        dev->fcr       = 0x06;
        dev->clock_src = 1843200.0;
        bsc_transmit_period(dev);
        timer_add(&dev->transmit_timer, bsc_transmit_timer, dev, 0);
        timer_add(&dev->timeout_timer, bsc_timeout_timer, dev, 0);
    }

    return dev;
}

const device_t ibm_bsc_device = {
    .name          = "IBM Binary Synchronous Communications Adapter",
    .internal_name = "ibmbsc",
    .flags         = 0,
    .local         = 0,
    .init          = bsc_init,
    .close         = bsc_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = bsc_speed_changed,
    .force_redraw  = NULL,
    .config        = NULL
};
