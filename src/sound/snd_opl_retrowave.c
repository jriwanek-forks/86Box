/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          RetroWave OPL3 bridge.
 *
 *          Version: 1.0.1
 *
 * Version: @(#)snd_opl_retrowave.c    1.0.1    2024/12/14
 *
 * Authors: Fred N. van Kempen, <decwiz@yahoo.com>
 *          Miran Grca, <mgrca8@gmail.com>
 *          Alexey Khokholov (Nuke.YKT)
 *          Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2017-2020 Fred N. van Kempen.
 *          Copyright 2016-2020 Miran Grca.
 *          Copyright 2013-2018 Alexey Khokholov (Nuke.YKT)
 *          Copyright 2024-2025 Jasmine Iwanek.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/sound.h>
#include <86box/timer.h>
#include <86box/device.h>
#include "cpu.h"
#include <86box/snd_opl.h>
#include <86box/snd_opl_retrowave.h>
#include "RetroWave/RetroWaveLib/RetroWave_86Box.h"

#define WRBUF_SIZE  1024
#define WRBUF_DELAY 1
#define RSM_FRAC    10

#ifdef ENABLE_OPL_LOG
int retrowave_do_log = ENABLE_OPL_LOG;

static void
retrowave_log(const char *fmt, ...)
{
    va_list ap;

    if (retrowave_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define retrowave_log(fmt, ...)
#endif

uint16_t
retrowave_write_addr(void *priv, uint16_t port, uint8_t val)
{
    retrowave_drv_t *dev  = (retrowave_drv_t *) priv;
    uint16_t         addr = val;
//    printf("writeaddr: 0x%08x 0x%02x\n", port, val);

    switch (port & 3) {
        case 0:
            dev->port = 0;
        case 2:
            dev->port = 1;
            if (val == 0x05)
                addr |= 0x100;
    }

    return addr;
}

void
retrowave_write_reg(void *priv, uint16_t reg, uint8_t val)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;
//    printf("writereg: 0x%08x 0x%02x\n", reg, val);

    uint8_t real_reg = reg & 0xff;
    uint8_t real_val = val;

    if (dev->port)
        retrowave_opl3_queue_port1(&retrowave_global_context, real_reg, real_val);
    else
        retrowave_opl3_queue_port0(&retrowave_global_context, real_reg, real_val);
}

void
retrowave_generate_stream(void *priv, int32_t *sndptr, uint32_t num)
{
    retrowave_flush(&retrowave_global_context);
}

static void
retrowave_timer_tick(retrowave_drv_t *dev, int tmr)
{
    dev->timer_cur_count[tmr] = (dev->timer_cur_count[tmr] + 1) & 0xff;

    retrowave_log("Ticking timer %i, count now %02X...\n", tmr, dev->timer_cur_count[tmr]);

    if (dev->timer_cur_count[tmr] == 0x00) {
    dev->status |= ((STAT_TMR1_OVER >> tmr) & ~dev->timer_ctrl);
    dev->timer_cur_count[tmr] = dev->timer_count[tmr];

    retrowave_log("Count wrapped around to zero, reloading timer %i (%02X), status = %02X...\n", tmr, (STAT_TMR1_OVER >> tmr), dev->status);
    }

    timer_on_auto(&dev->timers[tmr], (tmr == 1) ? 320.0 : 80.0);
}

static void
retrowave_timer_control(retrowave_drv_t *dev, int tmr, int start)
{
    timer_on_auto(&dev->timers[tmr], 0.0);

    if (start) {
        retrowave_log("Loading timer %i count: %02X = %02X\n", tmr, dev->timer_cur_count[tmr], dev->timer_count[tmr]);
        dev->timer_cur_count[tmr] = dev->timer_count[tmr];
        if (dev->flags & FLAG_OPL3)
            retrowave_timer_tick(dev, tmr); /* Per the YMF 262 datasheet, OPL3 starts counting immediately, unlike OPL2. */
        else
            timer_on_auto(&dev->timers[tmr], (tmr == 1) ? 320.0 : 80.0);
    } else {
        retrowave_log("Timer %i stopped\n", tmr);
        if (tmr == 1) {
            dev->status &= ~STAT_TMR2_OVER;
        } else
            dev->status &= ~STAT_TMR1_OVER;
    }
}

static void
retrowave_timer_1(void *priv)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;

    retrowave_timer_tick(dev, 0);
}


static void
retrowave_timer_2(void *priv)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;

    retrowave_timer_tick(dev, 1);
}

static void
retrowave_drv_set_do_cycles(void *priv, int8_t do_cycles)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;

    if (do_cycles)
        dev->flags |= FLAG_CYCLES;
    else
        dev->flags &= ~FLAG_CYCLES;
}

static void *
retrowave_drv_init(const device_t *info)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) calloc(1, sizeof(retrowave_drv_t));
    dev->flags = FLAG_CYCLES;
    if (info->local == FM_YMF262)
        dev->flags |= FLAG_OPL3;
    else
        dev->status = 0x06;

    retrowave_init_86box("COM6");
    retrowave_opl3_reset(&retrowave_global_context);

    if (!!(dev->flags & FLAG_OPL3))
        info->reset(dev);

    timer_add(&dev->timers[0], retrowave_timer_1, dev, 0);
    timer_add(&dev->timers[1], retrowave_timer_2, dev, 0);

    return dev;
}

static void
retrowave_drv_close(void *priv)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;
    free(dev);
}

static void
retrowave_opl3_reset_device(void *priv)
{
    retrowave_opl3_reset(&retrowave_global_context);
}

static void
retrowave_drv_reset_buffer(void *priv)
{
    retrowave_drv_t* dev = (retrowave_drv_t*) priv;

    dev->pos = 0;
}

static int32_t *
retrowave_drv_update_opl2(void *priv)
{
    retrowave_drv_t* drv = (retrowave_drv_t*) priv;

    if (drv->pos >= music_pos_global)
        return drv->buffer;

    // FIXME: Remove Magic Numbers
    retrowave_generate_stream(drv->opl,
              &drv->buffer[drv->pos * 2],
              sound_pos_global - drv->pos);

    for (; drv->pos < music_pos_global; drv->pos++) {
        drv->buffer[drv->pos * 2] /= 2;
        drv->buffer[(drv->pos * 2) + 1] = drv->buffer[drv->pos * 2];
    }

    return drv->buffer;    
}

static int32_t *
retrowave_drv_update_opl3(void *priv)
{
    retrowave_drv_t* drv = (retrowave_drv_t*) priv;

    if (drv->pos >= music_pos_global)
        return drv->buffer;

    // FIXME: Remove Magic Numbers
    retrowave_generate_stream(drv->opl,
              &drv->buffer[drv->pos * 2],
              sound_pos_global - drv->pos);

    for (; drv->pos < music_pos_global; drv->pos++) {
        drv->buffer[drv->pos * 2] /= 2;
        drv->buffer[(drv->pos * 2) + 1] /= 2;
    }

    return drv->buffer;    
}

static int32_t *
retrowave_drv_update(void *priv)
{
    retrowave_drv_t* drv = (retrowave_drv_t*) priv;

    if(!!(drv->flags & FLAG_OPL3))
        retrowave_drv_update_opl3(drv); 
    else
        retrowave_drv_update_opl2(drv);

    return drv->buffer;
}

static uint8_t
__generic_retrowave_read(uint16_t port, retrowave_drv_t* dev) {
    uint8_t ret = 0xff;

    if ((port & 0x0003) == 0x0000) {
        ret = dev->status;
        if (dev->status & STAT_TMR_OVER)
            ret |= STAT_TMR_ANY;
    }

    retrowave_log("OPL ret = %02x, status = %02x\n", ret, dev->status);

    return ret;
}

static uint8_t
retrowave_drv_read(uint16_t port, void *priv)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;

    if (!!(dev->flags & FLAG_CYCLES))
        cycles -= ((int) (isa_timing * 8));

    retrowave_drv_update(dev);

    return __generic_retrowave_read(port, dev);
}

static void
retrowave_drv_write(uint16_t port, uint8_t val, void* priv)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;

    retrowave_drv_update(dev);

    // FIXME: Magic Numbers and Math with them is bad
    if((port & 0x0001) == 0x0001) {
        retrowave_write_reg(dev,port,val);

        switch(dev->port) {
            case 0x02: /* Timer 1 */
                dev->timer_count[0] = val;
                retrowave_log("Timer 0 count now: %i\n", dev->timer_count[0]);
                break;
            case 0x03:    /* Timer 2 */
                dev->timer_count[1] = val;
                retrowave_log("Timer 1 count now: %i\n", dev->timer_count[1]);
                break;
            case 0x04:    /* Timer control */
                if (val & CTRL_RESET) {
                    retrowave_log("Resetting timer status...\n");
                    dev->status &= ~STAT_TMR_OVER;
                } else {
                    dev->timer_ctrl = val;
                    retrowave_timer_control(dev, 0, val & CTRL_TMR1_START);
                    retrowave_timer_control(dev, 1, val & CTRL_TMR2_START);
                    retrowave_log("Status mask now %02X (val = %02X)\n", (val & ~CTRL_TMR_MASK) & CTRL_TMR_MASK, val);
                }
                break;
            /* exits in snd_opl_nuked.c but not in original daemon32 code*/
            // case 0x105:
            //     dev->opl.newm = val & 0x01;
            //     break;

            default:
                break;
        }
    } else {
        dev->port = retrowave_write_addr(&dev->opl, port, val) & 0x01ff;

        if (!(dev->flags & FLAG_OPL3))
            dev->port &= 0x00ff;
    }
}

const device_t ym3812_retrowave_device = {
    .name          = "Yamaha YM3812 OPL2 (RetroWave)",
    .internal_name = "ym3812_retrowave",
    .flags         = 0,
    .local         = FM_YM3812,
    .init          = retrowave_drv_init,
    .close         = retrowave_drv_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const device_t ymf262_retrowave_device = {
    .name          = "Yamaha YMF262 OPL3 (RetroWave)",
    .internal_name = "ymf262_retrowave",
    .flags         = 0,
    .local         = FM_YMF262,
    .init          = retrowave_drv_init,
    .close         = retrowave_drv_close,
    .reset         = retrowave_opl3_reset_device,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const fm_drv_t retrowave_opl_drv = {
    .read          = &retrowave_drv_read,
    .write         = &retrowave_drv_write,
    .update        = &retrowave_drv_update,
    .reset_buffer  = &retrowave_drv_reset_buffer,
    .set_do_cycles = &retrowave_drv_set_do_cycles,
    .priv          = NULL,
    .generate      = NULL,
};
