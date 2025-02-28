/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Emulation of IBM MCGA graphics card.
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          Miran Grca, <mgrca8@gmail.com>
 *          Cacodemon345
 *
 *          Copyright 2008-2019 Sarah Walker.
 *          Copyright 2016-2019 Miran Grca.
 *          Copyright 2022-2025 Cacodemon345.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#include <math.h>
#include <86box/86box.h>
#include "cpu.h"
#include <86box/io.h>
#include <86box/timer.h>
#include <86box/pit.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/device.h>
#include <86box/video.h>
#include <86box/vid_mcga.h>
#include <86box/vid_cga_comp.h>
#include <86box/plat_unused.h>

#define MCGA_RGB 0

static uint8_t crtcmask[32] = {
    0xff, 0xff, 0xff, 0xff, 0x7f, 0x1f, 0x7f, 0x7f, 0xf3, 0x1f, 0x7f, 0x1f, 0x3f, 0xff, 0x3f, 0xff,
    0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static video_timings_t timing_mcga = { .type = VIDEO_ISA, .write_b = 8, .write_w = 16, .write_l = 32, .read_b = 8, .read_w = 16, .read_l = 32 };

void mcga_recalctimings(mcga_t *mcga);

void
mcga_out(uint16_t addr, uint8_t val, void *priv)
{
    mcga_t *mcga = (mcga_t *) priv;
    uint8_t old;

    if ((addr >= 0x3d0) && (addr <= 0x3d7))
        addr = (addr & 0xff9) | 0x004;

    switch (addr) {
        case 0x3C7:
            mcga->read_index = val;
            break;
        case 0x3C8:
            mcga->write_index = val;
            break;
        case 0x3C9:
            {
                mcga->last_cycle = 0x0;
                mcga->pal[(3 * mcga->write_index) + mcga->write_times] = val;
                if (mcga->write_times == 2) {
                    mcga->write_index++;
                    mcga->write_times = 0;
                }
                mcga->write_times++;
            }
        case 0x3D4:
            mcga->crtcreg = val & 31;
            return;
        case 0x3D5:
            /* Do nothing on horz/vert registers if write-protect is set. */
            if ((mcga->crtc[0x10] & 0x80) && mcga->crtcreg <= 0x7) return;
            old                       = mcga->crtc[mcga->crtcreg];
            mcga->crtc[mcga->crtcreg] = val & crtcmask[mcga->crtcreg];
            if (old != val) {
                if ((mcga->crtcreg < 0xe) || (mcga->crtcreg > 0x10)) {
                    mcga->fullchange = changeframecount;
                    mcga_recalctimings(mcga);
                }
                if (mcga->crtcreg == 0x10) {
                     mem_mapping_set_addr(&mcga->mapping, (mcga->crtc[mcga->crtcreg] & 0x3) ? 0xA0000 : 0xB8000, (mcga->crtc[mcga->crtcreg] & 0x3) ? 64000 : 0x8000);
                }
            }
            return;
        case 0x3D8:
            old           = mcga->cgamode;
            mcga->cgamode = val;

            if (old ^ val) {
                if ((old ^ val) & 0x05)
                    update_cga16_color(val);

                mcga_recalctimings(mcga);
            }
            return;
        case 0x3DD: /* Extended Mode Control Register. */
            mcga->mcga_extmode = val;
            break;
        case 0x3D9:
            old = mcga->cgacol;
            mcga->cgacol = val;
            if (old ^ val)
                mcga_recalctimings(mcga);
            return;

        default:
            break;
    }
}

uint8_t
mcga_in(uint16_t addr, void *priv)
{
    mcga_t *mcga = (mcga_t *) priv;
    uint8_t ret  = 0xff;

    if ((addr >= 0x3d0) && (addr <= 0x3d7))
        addr = (addr & 0xff9) | 0x004;

    switch (addr) {
        case 0x3C6:
            return 0xFF;
        case 0x3C7:
            return mcga->last_cycle & 0x3;
        case 0x3C8:
            return mcga->read_index;
        case 0x3C9:
            {
                mcga->last_cycle = 0x3;
                ret = mcga->pal[(3 * mcga->read_index) + mcga->read_times];
                if (mcga->read_times == 2) {
                    mcga->read_index++;
                    mcga->read_times = 0;
                }
                mcga->read_times++;
            }
        case 0x3D4:
            ret = mcga->crtcreg;
            break;
        case 0x3D5:
            ret = (mcga->crtcreg == 0x12 && (mcga->crtc[0x11] & 0x80)) ? 0x2 : mcga->crtc[mcga->crtcreg];
            if (mcga->crtcreg == 0x10) {
                ret &= 0x13;
                ret |= (mcga->cgamode & 0x01) ? 0x80 : 0x00;
                ret |= (mcga->cgamode & 0x01) ? 0x08 : 0x00;
            }
            break;
        case 0x3D8:
            ret = mcga->cgamode;
            break;
        case 0x3DA:
            ret = mcga->cgastat;
            break;
        case 0x3DD: /* Extended Mode Control Register. */
            ret = mcga->mcga_extmode;
            break;

        default:
            break;
    }

    return ret;
}

void
mcga_waitstates(UNUSED(void *priv))
{
    int ws_array[16] = { 3, 4, 5, 6, 7, 8, 4, 5, 6, 7, 8, 4, 5, 6, 7, 8 };
    int ws           = ws_array[cycles & 0xf];

    cycles -= ws;
}

void
mcga_write(uint32_t addr, uint8_t val, void *priv)
{
    mcga_t *mcga = (mcga_t *) priv;

    mcga->vram[addr & 0xffff] = val;
    mcga_waitstates(mcga);
}

uint8_t
mcga_read(uint32_t addr, void *priv)
{
    mcga_t *mcga = (mcga_t *) priv;

    mcga_waitstates(mcga);
    return mcga->vram[addr & 0xffff];
}

void
mcga_recalctimings(mcga_t *mcga)
{
    double disptime;
    double _dispontime;
    double _dispofftime;

    if (mcga->cgamode & 1) {
        disptime    = (double) (mcga->crtc[0] + 1);
        _dispontime = (double) mcga->crtc[1];
    } else {
        disptime    = (double) ((mcga->crtc[0] + 1) << 1);
        _dispontime = (double) (mcga->crtc[1] << 1);
    }
    _dispofftime     = disptime - _dispontime;
    _dispontime      = _dispontime * ((mcga->crtc[0x10] & 0x10) ? VGACONST1 : CGACONST);
    _dispofftime     = _dispofftime * ((mcga->crtc[0x10] & 0x10) ? VGACONST1 : CGACONST);
    mcga->dispontime  = (uint64_t) (_dispontime);
    mcga->dispofftime = (uint64_t) (_dispofftime);
}

void
mcga_poll(void *priv)
{
    mcga_t  *mcga = (mcga_t *)priv;
    uint16_t ca  = (mcga->crtc[15] | (mcga->crtc[14] << 8)) & 0xffff;
    int      drawcursor;
    int      x;
#if 0
    uint8_t  c;
#endif
    int      xs_temp;
    int      ys_temp;
    int      oldvc;
    uint8_t  chr;
    uint8_t  attr;
#if 0
    uint8_t  border;
#endif
    uint16_t dat;
    int      cols[4];
    int      col;
    int      oldsc;

    if (!mcga->linepos) {
    timer_advance_u64(&mcga->timer, mcga->dispofftime);
    mcga->cgastat |= 1;
    mcga->linepos = 1;
    oldsc = mcga->sc;
    if ((mcga->crtc[8] & 3) == 3)
        mcga->sc = ((mcga->sc << 1) + mcga->oddeven) & 7;
    if (mcga->cgadispon) {
        if (mcga->displine < mcga->firstline) {
            mcga->firstline = mcga->displine;
            video_wait_for_buffer();
        }
        mcga->lastline = mcga->displine;
        for (uint8_t c = 0; c < 8; c++) {
            if ((mcga->cgamode & 0x12) == 0x12) {
                buffer32->line[(mcga->displine << 1)][c] =
                buffer32->line[(mcga->displine << 1) + 1][c] = 0;
                if (mcga->cgamode & 1) {
                    buffer32->line[(mcga->displine << 1)][c + (mcga->crtc[1] << 3) + 8] =
                    buffer32->line[(mcga->displine << 1) + 1][c + (mcga->crtc[1] << 3) + 8] = 0;
                } else {
                    buffer32->line[(mcga->displine << 1)][c + (mcga->crtc[1] << 4) + 8] =
                    buffer32->line[(mcga->displine << 1) + 1][c + (mcga->crtc[1] << 4) + 8] = 0;
                }
            } else {
                buffer32->line[(mcga->displine << 1)][c] =
                buffer32->line[(mcga->displine << 1) + 1][c] = (mcga->cgacol & 15) + 16;
                if (mcga->cgamode & 1) {
                    buffer32->line[(mcga->displine << 1)][c + (mcga->crtc[1] << 3) + 8] =
                    buffer32->line[(mcga->displine << 1) + 1][c + (mcga->crtc[1] << 3) + 8] = (mcga->cgacol & 15) + 16;
                } else {
                    buffer32->line[(mcga->displine << 1)][c + (mcga->crtc[1] << 4) + 8] =
                    buffer32->line[(mcga->displine << 1) + 1][c + (mcga->crtc[1] << 4) + 8] = (mcga->cgacol & 15) + 16;
                }
            }
        }
        if (mcga->cgamode & 1) {
            for (x = 0; x < mcga->crtc[1]; x++) {
                if (mcga->cgamode & 8) {
                    chr = mcga->charbuffer[x << 1];
                    attr = mcga->charbuffer[(x << 1) + 1];
                } else
                    chr = attr = 0;
                drawcursor = ((mcga->ma == ca) && mcga->con && mcga->cursoron);
                cols[1] = (attr & 15) + 16;
                if (mcga->cgamode & 0x20) {
                    cols[0] = ((attr >> 4) & 7) + 16;
                    if ((mcga->cgablink & 8) && (attr & 0x80) && !mcga->drawcursor)
                        cols[1] = cols[0];
                } else
                    cols[0] = (attr >> 4) + 16;
                if (drawcursor) {
                    for (uint8_t c = 0; c < 8; c++) {
                        buffer32->line[(mcga->displine << 1)][(x << 3) + c + 8] =
                        buffer32->line[(mcga->displine << 1) + 1][(x << 3) + c + 8] =
                            cols[(fontdat[chr + mcga->fontbase][mcga->sc & 7] & (1 << (c ^ 7))) ? 1 : 0] ^ 15;
                    }
                } else {
                    for (uint8_t c = 0; c < 8; c++) {
                        buffer32->line[(mcga->displine << 1)][(x << 3) + c + 8] =
                        buffer32->line[(mcga->displine << 1) + 1][(x << 3) + c + 8] =
                            cols[(fontdat[chr + mcga->fontbase][mcga->sc & 7] & (1 << (c ^ 7))) ? 1 : 0];
                    }
                }
                mcga->ma++;
            }
        } else if (!(mcga->cgamode & 2)) {
            for (x = 0; x < mcga->crtc[1]; x++) {
                if (mcga->cgamode & 8) {
                    chr  = mcga->vram[((mcga->ma << 1) & 0xffff)];
                    attr = mcga->vram[(((mcga->ma << 1) + 1) & 0xffff)];
                } else
                    chr = attr = 0;
                drawcursor = ((mcga->ma == ca) && mcga->con && mcga->cursoron);
                cols[1] = (attr & 15) + 16;
                if (mcga->cgamode & 0x20) {
                    cols[0] = ((attr >> 4) & 7) + 16;
                    if ((mcga->cgablink & 8) && (attr & 0x80))
                        cols[1] = cols[0];
                } else
                    cols[0] = (attr >> 4) + 16;
                mcga->ma++;
                if (drawcursor) {
                    for (uint8_t c = 0; c < 8; c++) {
                        buffer32->line[(mcga->displine << 1)][(x << 4) + (c << 1) + 8] =
                        buffer32->line[(mcga->displine << 1)][(x << 4) + (c << 1) + 1 + 8] =
                        buffer32->line[(mcga->displine << 1) + 1][(x << 4) + (c << 1) + 8] =
                        buffer32->line[(mcga->displine << 1) + 1][(x << 4) + (c << 1) + 1 + 8] =
                            cols[(fontdat[chr + mcga->fontbase][mcga->sc & 7] & (1 << (c ^ 7))) ? 1 : 0] ^ 15;
                    }
                } else {
                    for (uint8_t c = 0; c < 8; c++) {
                        buffer32->line[(mcga->displine << 1)][(x << 4) + (c << 1) + 8] =
                        buffer32->line[(mcga->displine << 1)][(x << 4) + (c << 1) + 1 + 8] =
                        buffer32->line[(mcga->displine << 1) + 1][(x << 4) + (c << 1) + 8] =
                        buffer32->line[(mcga->displine << 1) + 1][(x << 4) + (c << 1) + 1 + 8] =
                        cols[(fontdat[chr + mcga->fontbase][mcga->sc & 7] & (1 << (c ^ 7))) ? 1 : 0];
                    }
                }
            }
        } else if (!(mcga->cgamode & 16)) {
            cols[0] = (mcga->cgacol & 15) | 16;
            col = (mcga->cgacol & 16) ? 24 : 16;
            if (mcga->cgamode & 4) {
                cols[1] = col | 3;    /* Cyan */
                cols[2] = col | 4;    /* Red */
                cols[3] = col | 7;    /* White */
            } else if (mcga->cgacol & 32) {
                cols[1] = col | 3;    /* Cyan */
                cols[2] = col | 5;    /* Magenta */
                cols[3] = col | 7;    /* White */
            } else {
                cols[1] = col | 2;    /* Green */
                cols[2] = col | 4;    /* Red */
                cols[3] = col | 6;    /* Yellow */
            }
            for (x = 0; x < mcga->crtc[1]; x++) {
                if (mcga->cgamode & 8)
                    dat = (mcga->vram[((mcga->ma << 1) & 0x1fff) + ((mcga->sc & 1) * 0x2000)] << 8) | mcga->vram[((mcga->ma << 1) & 0x1fff) + ((mcga->sc & 1) * 0x2000) + 1];
                else
                    dat = 0;
                mcga->ma++;
                for (uint8_t c = 0; c < 8; c++) {
                    buffer32->line[(mcga->displine << 1)][(x << 4) + (c << 1) + 8] =
                    buffer32->line[(mcga->displine << 1)][(x << 4) + (c << 1) + 1 + 8] =
                    buffer32->line[(mcga->displine << 1) + 1][(x << 4) + (c << 1) + 8] =
                    buffer32->line[(mcga->displine << 1) + 1][(x << 4) + (c << 1) + 1 + 8] =
                        cols[dat >> 14];
                    dat <<= 2;
                }
            }
        } else {
            cols[0] = 0; cols[1] = (mcga->cgacol & 15) + 16;
            for (x = 0; x < mcga->crtc[1]; x++) {
                if (mcga->cgamode & 8)
                    dat = (mcga->vram[((mcga->ma << 1) & 0x1fff) + ((mcga->sc & 1) * 0x2000)] << 8) | mcga->vram[((mcga->ma << 1) & 0x1fff) + ((mcga->sc & 1) * 0x2000) + 1];
                else
                    dat = 0;
                mcga->ma++;
                for (uint8_t c = 0; c < 16; c++) {
                    buffer32->line[(mcga->displine << 1)][(x << 4) + c + 8] =
                    buffer32->line[(mcga->displine << 1) + 1][(x << 4) + c + 8] =
                        cols[dat >> 15];
                    dat <<= 1;
                }
            }
        }
    } else {
        cols[0] = ((mcga->cgamode & 0x12) == 0x12) ? 0 : (mcga->cgacol & 15) + 16;
        if (mcga->cgamode & 1) {
            hline(buffer32, 0, (mcga->displine << 1), ((mcga->crtc[1] << 3) + 16) << 2, cols[0]);
            hline(buffer32, 0, (mcga->displine << 1) + 1, ((mcga->crtc[1] << 3) + 16) << 2, cols[0]);
        } else {
            hline(buffer32, 0, (mcga->displine << 1), ((mcga->crtc[1] << 4) + 16) << 2, cols[0]);
            hline(buffer32, 0, (mcga->displine << 1) + 1, ((mcga->crtc[1] << 4) + 16) << 2, cols[0]);
        }
    }

    if (mcga->cgamode & 1)
        x = (mcga->crtc[1] << 3) + 16;
    else
        x = (mcga->crtc[1] << 4) + 16;

    mcga->sc = oldsc;
    if (mcga->vc == mcga->crtc[7] && !mcga->sc)
        mcga->cgastat |= 8;
    mcga->displine++;
    if (mcga->displine >= 360)
        mcga->displine = 0;
    } else {
    timer_advance_u64(&mcga->timer, mcga->dispontime);
    mcga->linepos = 0;
    if (mcga->vsynctime) {
        mcga->vsynctime--;
        if (!mcga->vsynctime)
            mcga->cgastat &= ~8;
    }
    if (mcga->sc == (mcga->crtc[11] & 31) || ((mcga->crtc[8] & 3) == 3 && mcga->sc == ((mcga->crtc[11] & 31) >> 1))) {
        mcga->con = 0;
        mcga->coff = 1;
    }
    if ((mcga->crtc[8] & 3) == 3 && mcga->sc == (mcga->crtc[9] >> 1))
        mcga->maback = mcga->ma;
    if (mcga->vadj) {
        mcga->sc++;
        mcga->sc &= 31;
        mcga->ma = mcga->maback;
        mcga->vadj--;
        if (!mcga->vadj) {
            mcga->cgadispon = 1;
            mcga->ma = mcga->maback = (mcga->crtc[13] | (mcga->crtc[12] << 8)) & 0xffff;
            mcga->sc = 0;
        }
    } else if (mcga->sc == mcga->crtc[9]) {
        mcga->maback = mcga->ma;
        mcga->sc = 0;
        oldvc = mcga->vc;
        mcga->vc++;
        mcga->vc &= 127;

        if (mcga->vc == mcga->crtc[6])
            mcga->cgadispon = 0;

        if (oldvc == mcga->crtc[4]) {
            mcga->vc = 0;
            mcga->vadj = mcga->crtc[5];
            if (!mcga->vadj) {
                mcga->cgadispon = 1;
                mcga->ma = mcga->maback = (mcga->crtc[13] | (mcga->crtc[12] << 8)) & 0xffff;
            }
            switch (mcga->crtc[10] & 0x60) {
                case 0x20:
                    mcga->cursoron = 0;
                    break;
                case 0x60:
                    mcga->cursoron = mcga->cgablink & 0x10;
                    break;
                default:
                    mcga->cursoron = mcga->cgablink & 0x08;
                    break;
            }
        }

        if (mcga->vc == mcga->crtc[7]) {
            mcga->cgadispon = 0;
            mcga->displine = 0;
            mcga->vsynctime = 16;
            if (mcga->crtc[7]) {
                if (mcga->cgamode & 1)
                    x = (mcga->crtc[1] << 3) + 16;
                else
                    x = (mcga->crtc[1] << 4) + 16;
                mcga->lastline++;

                xs_temp = x;
                ys_temp = (mcga->lastline - mcga->firstline) << 1;

                if ((xs_temp > 0) && (ys_temp > 0)) {
                    if (xs_temp < 64) xs_temp = 656;
                    if (ys_temp < 32) ys_temp = 400;
                    if (!enable_overscan)
                        xs_temp -= 16;

                    if ((mcga->cgamode & 8) && ((xs_temp != xsize) || (ys_temp != ysize) || video_force_resize_get())) {
                        xsize = xs_temp;
                        ysize = ys_temp;
                        set_screen_size(xsize, ysize + (enable_overscan ? 16 : 0));

                        if (video_force_resize_get())
                            video_force_resize_set(0);
                    }

                    if (enable_overscan) {
                            video_blit_memtoscreen_8(0, (mcga->firstline - 4) << 1,
                                         xsize, ((mcga->lastline - mcga->firstline) + 8) << 1);
                    } else {
                            video_blit_memtoscreen_8(8, mcga->firstline << 1,
                                         xsize, (mcga->lastline - mcga->firstline) << 1);
                    }
                }

                frames++;

                video_res_x = xsize;
                video_res_y = ysize;
                if (mcga->cgamode & 1) {
                    video_res_x /= 8;
                    video_res_y /= mcga->crtc[9] + 1;
                    video_bpp = 0;
                } else if (!(mcga->cgamode & 2)) {
                    video_res_x /= 16;
                    video_res_y /= mcga->crtc[9] + 1;
                    video_bpp = 0;
                } else if (!(mcga->cgamode & 16)) {
                    video_res_x /= 2;
                    video_bpp = 2;
                } else
                    video_bpp = 1;
            }
            mcga->firstline = 1000;
            mcga->lastline = 0;
            mcga->cgablink++;
            mcga->oddeven ^= 1;
        }
    } else {
        mcga->sc++;
        mcga->sc &= 31;
        mcga->ma = mcga->maback;
    }
    if (mcga->cgadispon)
        mcga->cgastat &= ~1;
    if ((mcga->sc == (mcga->crtc[10] & 31) || ((mcga->crtc[8] & 3) == 3 && mcga->sc == ((mcga->crtc[10] & 31) >> 1))))
        mcga->con = 1;
    if (mcga->cgadispon && (mcga->cgamode & 1)) {
        for (x = 0; x < (mcga->crtc[1] << 1); x++)
            mcga->charbuffer[x] = mcga->vram[(((mcga->ma << 1) + x) & 0xffff)];
    }
    }
}

void
mcga_init(mcga_t *mcga)
{
    timer_add(&mcga->timer, mcga_poll, mcga, 1);
    mcga->composite = 0;
}

void *
mcga_standalone_init(const device_t *info)
{
#if 0
    int     display_type;
#endif
    mcga_t *mcga = calloc(1, sizeof(mcga_t));

    video_inform(VIDEO_FLAG_TYPE_CGA, &timing_mcga);

#if 0
    display_type = device_get_config_int("display_type");
    mcga->snow_enabled = device_get_config_int("snow_enabled");
#endif

    mcga->vram = calloc(1, 0x10000);
    mcga->mcga_extmode |= 0x80; /* Readable DAC. */

    cga_comp_init(mcga->revision);
    timer_add(&mcga->timer, mcga_poll, mcga, 1);
    mem_mapping_add(&mcga->mapping, 0xb8000, 0x08000, mcga_read, NULL, NULL, mcga_write, NULL, NULL, NULL /*mcga->vram*/, MEM_MAPPING_EXTERNAL, mcga);
    io_sethandler(0x03c6, 0x0004, mcga_in, NULL, NULL, mcga_out, NULL, NULL, mcga);
    io_sethandler(0x03d0, 0x0010, mcga_in, NULL, NULL, mcga_out, NULL, NULL, mcga);

    overscan_x = overscan_y = 16;

    mcga->rgb_type = device_get_config_int("rgb_type");
    cga_palette    = (mcga->rgb_type << 1);
    cgapal_rebuild();

    return mcga;
}

void
mcga_close(void *priv)
{
    mcga_t *mcga = (mcga_t *) priv;

    free(mcga->vram);
    free(mcga);
}

void
mcga_speed_changed(void *priv)
{
    mcga_t *mcga = (mcga_t *) priv;

    mcga_recalctimings(mcga);
}

// clang-format off
const device_config_t mcga_config[] = {
#if 0
    {
        .name           = "display_type",
        .description    = "Display type",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = MCGA_RGB,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "RGB", .value = MCGA_RGB },
            { .description = ""                       }
        },
        .bios           = { { 0 } }
    },
#endif
    {
        .name           = "rgb_type",
        .description    = "RGB type",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Color",            .value = 0 },
            { .description = "Green Monochrome", .value = 1 },
            { .description = "Amber Monochrome", .value = 2 },
            { .description = "Gray Monochrome",  .value = 3 },
            { .description = "Color (no brown)", .value = 4 },
            { .description = ""                             }
        },
        .bios           = { { 0 } }
    },
#if 0
    {
        .name           = "snow_enabled",
        .description    = "Snow emulation",
        .type           = CONFIG_BINARY,
        .default_string = NULL,
        .default_int    = 1,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
#endif
    { .name = "", .description = "", .type = CONFIG_END }
};
// clang-format on

const device_t mcga_device = {
    .name          = "MCGA",
    .internal_name = "mcga",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = mcga_standalone_init,
    .close         = mcga_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = mcga_speed_changed,
    .force_redraw  = NULL,
    .config        = mcga_config
};
