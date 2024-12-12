/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Quadram Quadcolor I / I+II emulation
 *
 *
 *
 * Authors:  Benedikt Freisen, <https://pcem-emulator.co.uk/>
 *
 *          Copyright 2024      Benedikt Freisen.
 */

/* This has been derived from CGA emulation */
/* omissions: simulated snow (Quadcolor has dual-ported RAM), single and dual 8x16 font configuration */
/* additions: ports 0x3dd and 0x3de, 2nd char set, 2nd VRAM bank, hi-res bg color, Quadcolor II memory and mode */
/* assumptions: MA line 12 XORed with Bank Select, hi-res bg is also border color, QC2 mode has simple address counter */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <86box/86box.h>
#include <86box/io.h>
#include <86box/timer.h>
#include <86box/pit.h>
#include <86box/mem.h>
#include <86box/device.h>
#include <86box/video.h>
#include <86box/vid_quadcolor.h>
#include <86box/vid_cga_comp.h>
#include <86box/plat_unused.h>

#define CGA_RGB       0
#define CGA_COMPOSITE 1

#define COMPOSITE_OLD 0
#define COMPOSITE_NEW 1

static uint8_t crtcmask[32] = {
    0xff, 0xff, 0xff, 0xff, 0x7f, 0x1f, 0x7f, 0x7f, 0xf3, 0x1f, 0x7f, 0x1f, 0x3f, 0xff, 0x3f, 0xff,
    0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static video_timings_t timing_quadcolor = { .type = VIDEO_ISA, .write_b = 8, .write_w = 16, .write_l = 32, .read_b = 8, .read_w = 16, .read_l = 32 };

void quadcolor_recalctimings(quadcolor_t *quadcolor);

void
quadcolor_out(uint16_t addr, uint8_t val, void *priv)
{
    quadcolor_t *quadcolor = (quadcolor_t *) priv;
    uint8_t old;

    // pclog("CGA_OUT %04X %02X\n", addr, val);
    switch (addr) {
        case 0x3d0:
        case 0x3d2:
        case 0x3d4:
        case 0x3d6:
            quadcolor->crtcreg = val & 31;
            return;
        case 0x3d1:
        case 0x3d3:
        case 0x3d5:
        case 0x3d7:
            old = quadcolor->crtc[quadcolor->crtcreg];
            quadcolor->crtc[quadcolor->crtcreg] = val & crtcmask[quadcolor->crtcreg];
            if (old != val) {
                if (quadcolor->crtcreg < 0xe || quadcolor->crtcreg > 0x10) {
                    fullchange = changeframecount;
                    quadcolor_recalctimings(quadcolor);
                }
            }
            return;
        case 0x3d8:
            if (((quadcolor->cgamode ^ val) & 5) != 0) {
                quadcolor->cgamode = val;
                update_cga16_color(quadcolor->cgamode);
            }
            quadcolor->cgamode = val;
            return;
        case 0x3d9:
            quadcolor->cgacol = val;
            return;
        case 0x3dd:
            quadcolor->quadcolor_ctrl = val & 0x3f;
            /* helper variable that can be XORed onto the VRAM address to select the page to display */
            quadcolor->page_offset = (val & 0x10) << 8;
            /* in dual 8x8 font configuration, use fontbase 256 if "Character Set Select" bit is set */
            if (quadcolor->has_2nd_charset)
                quadcolor->fontbase = (val & 0x20) << 3;
            return;
        case 0x3de:
            if (quadcolor->has_quadcolor_2)
                quadcolor->quadcolor_2_oe = val & 0x10;
            return;
    }
}

uint8_t
quadcolor_in(uint16_t addr, void *priv)
{
    quadcolor_t *quadcolor = (quadcolor_t *) priv;

    // pclog("CGA_IN %04X\n", addr);
    switch (addr) {
        case 0x3D4:
            return quadcolor->crtcreg;
        case 0x3D5:
             return quadcolor->crtc[quadcolor->crtcreg];
        case 0x3DA:
            return quadcolor->cgastat;
    }
    return 0xFF;
}

void
quadcolor_write(uint32_t addr, uint8_t val, void *priv)
{
    quadcolor_t *quadcolor = (quadcolor_t *) priv;

    // pclog("CGA_WRITE %04X %02X\n", addr, val);
    quadcolor->vram[addr & 0x7fff] = val;
    egawrites++;
    cycles -= 4;
}

uint8_t
quadcolor_read(uint32_t addr, void *priv)
{
    quadcolor_t *quadcolor = (quadcolor_t *) priv;
    cycles -= 4;
    egareads++;
    // pclog("CGA_READ %04X\n", addr);
    return quadcolor->vram[addr & 0x7fff];
}

void
quadcolor_2_write(uint32_t addr, uint8_t val, void *priv)
{
    quadcolor_t *quadcolor = (quadcolor_t *) priv;

    quadcolor->vram_2[addr & 0xffff] = val;
}

uint8_t
quadcolor_2_read(uint32_t addr, void *priv)
{
    quadcolor_t *quadcolor = (quadcolor_t *) priv;

    return quadcolor->vram_2[addr & 0xffff];
}

void
quadcolor_recalctimings(quadcolor_t *quadcolor)
{
    double disptime;
    double _dispontime;
    double _dispofftime;

    pclog("Recalc - %i %i %i\n", quadcolor->crtc[0], quadcolor->crtc[1], quadcolor->cgamode & 1);
    if (quadcolor->cgamode & 1) {
        disptime = quadcolor->crtc[0] + 1;
        _dispontime = quadcolor->crtc[1];
    } else {
        disptime = (quadcolor->crtc[0] + 1) << 1;
        _dispontime = quadcolor->crtc[1] << 1;
    }
    _dispofftime = disptime - _dispontime;
    // printf("%i %f %f %f  %i %i\n",cgamode&1,disptime,dispontime,dispofftime,crtc[0],crtc[1]);
    _dispontime *= CGACONST;
    _dispofftime *= CGACONST;
    // printf("Timings - on %f off %f frame %f second
    // %f\n",dispontime,dispofftime,(dispontime+dispofftime)*262.0,(dispontime+dispofftime)*262.0*59.92);
    quadcolor->dispontime = (uint64_t)_dispontime;
    quadcolor->dispofftime = (uint64_t)_dispofftime;
}

static inline uint8_t get_next_qc2_pixel(quadcolor_t *quadcolor) {
    uint8_t pixel = (quadcolor->vram_2[quadcolor->qc2idx] & quadcolor->qc2mask) >> ((quadcolor->qc2mask = ~quadcolor->qc2mask) & 4);

    quadcolor->qc2idx += quadcolor->qc2mask >> 7;
    return quadcolor->quadcolor_2_oe ? pixel : 0;
}

void
quadcolor_poll(void *priv)
{
    quadcolor_t *quadcolor = (quadcolor_t *) priv;
    uint16_t ca = (quadcolor->crtc[15] | (quadcolor->crtc[14] << 8)) & 0x7fff;
    int drawcursor;
    int x, c;
    int oldvc;
    uint8_t chr;
    uint8_t attr;
    uint16_t dat;
    uint32_t cols[4];
    int col;
    int oldsc;

    if (!quadcolor->linepos) {
        timer_advance_u64(&quadcolor->timer, quadcolor->dispofftime);
        quadcolor->cgastat |= 1;
        quadcolor->linepos = 1;
        oldsc = quadcolor->sc;
        if ((quadcolor->crtc[8] & 3) == 3)
            quadcolor->sc = ((quadcolor->sc << 1) + quadcolor->oddeven) & 7;
        if (quadcolor->cgadispon) {
            if (quadcolor->displine < quadcolor->firstline) {
                quadcolor->firstline = quadcolor->displine;
                video_wait_for_buffer();
                // printf("Firstline %i\n",firstline);
            }
            quadcolor->lastline = quadcolor->displine;

            cols[0] = ((quadcolor->cgamode & 0x12) == 0x12) ? (quadcolor->quadcolor_ctrl & 15) : (quadcolor->cgacol & 15);  /* TODO: Is Quadcolor bg color actually relevant, here? */
            for (c = 0; c < 8; c++) {
                ((uint32_t *) buffer32->line[quadcolor->displine])[c] = cols[0];
                if (quadcolor->cgamode & 1)
                    ((uint32_t *) buffer32->line[quadcolor->displine])[c + (quadcolor->crtc[1] << 3) + 8] = cols[0];
                else
                    ((uint32_t *) buffer32->line[quadcolor->displine])[c + (quadcolor->crtc[1] << 4) + 8] = cols[0];
            }
            if (quadcolor->cgamode & 1) {  /* 80-column text */
                for (x = 0; x < quadcolor->crtc[1]; x++) {
                    if (quadcolor->cgamode & 8) {
                        chr = quadcolor->charbuffer[x << 1];
                        attr = quadcolor->charbuffer[(x << 1) + 1];
                    } else
                        chr = attr = 0;
                    drawcursor = ((quadcolor->ma == ca) && quadcolor->con && quadcolor->cursoron);
                    if (quadcolor->cgamode & 0x20) {
                        cols[1] = attr & 15;
                        cols[0] = (attr >> 4) & 7;
                        if ((quadcolor->cgablink & 8) && (attr & 0x80) && !quadcolor->drawcursor)
                            cols[1] = cols[0];
                    } else {
                        cols[1] = attr & 15;
                        cols[0] = attr >> 4;
                    }
                    if (drawcursor) {
                        for (c = 0; c < 8; c++)
                            ((uint32_t *) buffer32->line[quadcolor->displine])[(x << 3) + c + 8] =
                                (cols[(fontdat[chr + quadcolor->fontbase][quadcolor->sc & 7] & (1 << (c ^ 7)))
                                         ? 1
                                         : 0] ^
                                0xffffff) | get_next_qc2_pixel(quadcolor);
                    } else {
                        for (c = 0; c < 8; c++)
                            ((uint32_t *) buffer32->line[quadcolor->displine])[(x << 3) + c + 8] =
                                cols[(fontdat[chr + quadcolor->fontbase][quadcolor->sc & 7] & (1 << (c ^ 7)))
                                         ? 1
                                         : 0] | get_next_qc2_pixel(quadcolor);
                    }
                    quadcolor->ma++;
                }
            } else if (!(quadcolor->cgamode & 2)) {  /* not graphics (nor 80-column text) => 40-column text */
                for (x = 0; x < quadcolor->crtc[1]; x++) {
                    if (quadcolor->cgamode & 8) {
                        chr = quadcolor->vram[quadcolor->page_offset ^ ((quadcolor->ma << 1) & 0x7fff)];
                        attr = quadcolor->vram[quadcolor->page_offset ^ (((quadcolor->ma << 1) + 1) & 0x7fff)];
                    } else
                        chr = attr = 0;
                    drawcursor = ((quadcolor->ma == ca) && quadcolor->con && quadcolor->cursoron);
                    if (quadcolor->cgamode & 0x20) {
                        cols[1] = attr & 15;
                        cols[0] = (attr >> 4) & 7;
                        if ((quadcolor->cgablink & 8) && (attr & 0x80) && !quadcolor->drawcursor)
                            cols[1] = cols[0];
                    } else {
                        cols[1] = attr & 15;
                        cols[0] = attr >> 4;
                    }
                    quadcolor->ma++;
                    if (drawcursor) {
                        for (c = 0; c < 8; c++)
                            ((uint32_t *) buffer32->line[quadcolor->displine])[(x << 4) + (c << 1) + 8] =
                                ((uint32_t *)buffer32
                                     ->line[quadcolor->displine])[(x << 4) + (c << 1) + 1 + 8] =
                                    (cols[(fontdat[chr + quadcolor->fontbase][quadcolor->sc & 7] &
                                          (1 << (c ^ 7)))
                                             ? 1
                                             : 0] ^
                                    0xffffff) | get_next_qc2_pixel(quadcolor);
                    } else {
                        for (c = 0; c < 8; c++)
                            ((uint32_t *) buffer32->line[quadcolor->displine])[(x << 4) + (c << 1) + 8] =
                                ((uint32_t *) buffer32
                                     ->line[quadcolor->displine])[(x << 4) + (c << 1) + 1 + 8] =
                                    cols[(fontdat[chr + quadcolor->fontbase][quadcolor->sc & 7] &
                                          (1 << (c ^ 7)))
                                             ? 1
                                             : 0] | get_next_qc2_pixel(quadcolor);
                    }
                }
            } else if (!(quadcolor->cgamode & 16)) {  /* not hi-res (but graphics) => 4-color mode */
                cols[0] = quadcolor->cgacol & 15;
                col = (quadcolor->cgacol & 16) ? 8 : 0;
                if (quadcolor->cgamode & 4) {
                    cols[1] = col | 3;
                    cols[2] = col | 4;
                    cols[3] = col | 7;
                } else if (quadcolor->cgacol & 32) {
                    cols[1] = col | 3;
                    cols[2] = col | 5;
                    cols[3] = col | 7;
                } else {
                    cols[1] = col | 2;
                    cols[2] = col | 4;
                    cols[3] = col | 6;
                }
                for (x = 0; x < quadcolor->crtc[1]; x++) {
                    if (quadcolor->cgamode & 8)
                        dat = (quadcolor->vram[quadcolor->page_offset ^ (((quadcolor->ma << 1) & 0x1fff) + ((quadcolor->sc & 1) * 0x2000))] << 8) |
                              quadcolor->vram[quadcolor->page_offset ^ (((quadcolor->ma << 1) & 0x1fff) + ((quadcolor->sc & 1) * 0x2000) + 1)];
                    else
                        dat = 0;
                    quadcolor->ma++;
                    for (c = 0; c < 8; c++) {
                        ((uint32_t *) buffer32->line[quadcolor->displine])[(x << 4) + (c << 1) + 8] =
                            ((uint32_t *) buffer32->line[quadcolor->displine])[(x << 4) + (c << 1) + 1 + 8] =
                                cols[dat >> 14] | get_next_qc2_pixel(quadcolor);
                        dat <<= 2;
                    }
                }
            } else {  /* 2-color hi-res graphics mode */
                cols[0] = quadcolor->quadcolor_ctrl & 15;  /* background color (Quadcolor-specific) */
                cols[1] = quadcolor->cgacol & 15;
                for (x = 0; x < quadcolor->crtc[1]; x++) {
                    if (quadcolor->cgamode & 8)  /* video enabled */
                        dat = (quadcolor->vram[quadcolor->page_offset ^ (((quadcolor->ma << 1) & 0x1fff) + ((quadcolor->sc & 1) * 0x2000))] << 8) |
                              quadcolor->vram[quadcolor->page_offset ^ (((quadcolor->ma << 1) & 0x1fff) + ((quadcolor->sc & 1) * 0x2000) + 1)];
                    else
                        dat = quadcolor->quadcolor_ctrl & 15;  /* TODO: Is Quadcolor bg color actually relevant, here? Probably. See QC2 manual p.46 1. */
                    quadcolor->ma++;
                    for (c = 0; c < 16; c++) {
                        ((uint32_t *) buffer32->line[quadcolor->displine])[(x << 4) + c + 8] = cols[dat >> 15] | get_next_qc2_pixel(quadcolor);
                        dat <<= 1;
                    }
                }
            }
        } else {
            cols[0] = ((quadcolor->cgamode & 0x12) == 0x12) ? (quadcolor->quadcolor_ctrl & 15) : (quadcolor->cgacol & 15);  /* TODO: Is Quadcolor bg color actually relevant, here? */
            if (quadcolor->cgamode & 1)
                hline(buffer32, 0, quadcolor->displine, (quadcolor->crtc[1] << 3) + 16, cols[0]);
            else
                hline(buffer32, 0, quadcolor->displine, (quadcolor->crtc[1] << 4) + 16, cols[0]);
        }

        if (quadcolor->cgamode & 1)
            x = (quadcolor->crtc[1] << 3) + 16;
        else
            x = (quadcolor->crtc[1] << 4) + 16;

        if (quadcolor->composite) {
            for (c = 0; c < x; c++)
                buffer32->line[quadcolor->displine][c] = ((uint32_t *) buffer32->line[quadcolor->displine])[c] & 0xf;

            Composite_Process(quadcolor->cgamode, 0, x >> 2, buffer32->line[quadcolor->displine]);
        } else {
            for (c = 0; c < x; c++)
                ((uint32_t *) buffer32->line[quadcolor->displine])[c] =
                    cgapal[((uint32_t *) buffer32->line[quadcolor->displine])[c] & 0xf];
        }

        quadcolor->sc = oldsc;
        if (quadcolor->vc == quadcolor->crtc[7] && !quadcolor->sc)
            quadcolor->cgastat |= 8;
        quadcolor->displine++;
        if (quadcolor->displine >= 360)
            quadcolor->displine = 0;
    } else {
        timer_advance_u64(&quadcolor->timer, quadcolor->dispontime);
        quadcolor->linepos = 0;
        if (quadcolor->vsynctime) {
            quadcolor->vsynctime--;
            if (!quadcolor->vsynctime)
                quadcolor->cgastat &= ~8;
            quadcolor->qc2idx = 0;
            quadcolor->qc2mask = 0xf0;
        }
        if (quadcolor->sc == (quadcolor->crtc[11] & 31) || ((quadcolor->crtc[8] & 3) == 3 && quadcolor->sc == ((quadcolor->crtc[11] & 31) >> 1))) {
            quadcolor->con  = 0;
            quadcolor->coff = 1;
        }
        if ((quadcolor->crtc[8] & 3) == 3 && quadcolor->sc == (quadcolor->crtc[9] >> 1))
            quadcolor->maback = quadcolor->ma;
        if (quadcolor->vadj) {
            quadcolor->sc++;
            quadcolor->sc &= 31;
            quadcolor->ma = quadcolor->maback;
            quadcolor->vadj--;
            if (!quadcolor->vadj) {
                quadcolor->cgadispon = 1;
                quadcolor->ma = quadcolor->maback = (quadcolor->crtc[13] | (quadcolor->crtc[12] << 8)) & 0x7fff;
                quadcolor->sc                     = 0;
            }
        } else if (quadcolor->sc == quadcolor->crtc[9]) {
            quadcolor->maback = quadcolor->ma;
            quadcolor->sc = 0;
            oldvc = quadcolor->vc;
            quadcolor->vc++;
            quadcolor->vc &= 127;

            if (quadcolor->vc == quadcolor->crtc[6])
                quadcolor->cgadispon = 0;

            if (oldvc == quadcolor->crtc[4]) {
                quadcolor->vc = 0;
                quadcolor->vadj = quadcolor->crtc[5];
                if (!quadcolor->vadj) {
                    quadcolor->cgadispon = 1;
                    quadcolor->ma = quadcolor->maback = (quadcolor->crtc[13] | (quadcolor->crtc[12] << 8)) & 0x7fff;
                }
                if ((quadcolor->crtc[10] & 0x60) == 0x20)
                    quadcolor->cursoron = 0;
                else
                    quadcolor->cursoron = quadcolor->cgablink & 8;
            }

            if (quadcolor->vc == quadcolor->crtc[7]) {
                quadcolor->cgadispon = 0;
                quadcolor->displine = 0;
                quadcolor->vsynctime = 16;
                if (quadcolor->crtc[7]) {
                    if (quadcolor->cgamode & 1)
                        x = (quadcolor->crtc[1] << 3) + 16;
                    else
                        x = (quadcolor->crtc[1] << 4) + 16;
                    quadcolor->lastline++;
                    if (x != xsize || (quadcolor->lastline - quadcolor->firstline) != ysize) {
                        xsize = x;
                        ysize = quadcolor->lastline - quadcolor->firstline;
                        if (xsize < 64)
                            xsize = 656;
                        if (ysize < 32)
                            ysize = 200;
                        updatewindowsize(xsize, (ysize << 1) + 16);
                    }

                    video_blit_memtoscreen(0, quadcolor->firstline - 4, 0, (quadcolor->lastline - quadcolor->firstline) + 8,
                                   xsize, (quadcolor->lastline - quadcolor->firstline) + 8);
                    frames++;

                    video_res_x = xsize - 16;
                    video_res_y = ysize;
                    if (quadcolor->cgamode & 1) {
                        video_res_x /= 8;
                        video_res_y /= quadcolor->crtc[9] + 1;
                        video_bpp = 0;
                    } else if (!(quadcolor->cgamode & 2)) {
                        video_res_x /= 16;
                        video_res_y /= quadcolor->crtc[9] + 1;
                        video_bpp = 0;
                    } else if (!(quadcolor->cgamode & 16)) {
                        video_res_x /= 2;
                        video_bpp = 2;
                    } else
                        video_bpp = 1;
                }
                quadcolor->firstline = 1000;
                quadcolor->lastline = 0;
                quadcolor->cgablink++;
                quadcolor->oddeven ^= 1;
            }
        } else {
            quadcolor->sc++;
            quadcolor->sc &= 31;
            quadcolor->ma = quadcolor->maback;
        }
        if (quadcolor->cgadispon)
            quadcolor->cgastat &= ~1;
        if ((quadcolor->sc == (quadcolor->crtc[10] & 31) || ((quadcolor->crtc[8] & 3) == 3 && quadcolor->sc == ((quadcolor->crtc[10] & 31) >> 1))))
            quadcolor->con = 1;
        if (quadcolor->cgadispon && (quadcolor->cgamode & 1)) {
            for (x = 0; x < (quadcolor->crtc[1] << 1); x++)
                quadcolor->charbuffer[x] = quadcolor->vram[quadcolor->page_offset ^ (((quadcolor->ma << 1) + x) & 0x7fff)];
        }
    }
}

void
quadcolor_init(quadcolor_t *quadcolor)
{
    timer_add(&quadcolor->timer, quadcolor_poll, quadcolor, 1);
    quadcolor->composite = 0;
}

void *
quadcolor_standalone_init(UNUSED(const device_t *info))
{
    int display_type;
    int contrast;
    quadcolor_t *quadcolor = malloc(sizeof(quadcolor_t));

    memset(quadcolor, 0, sizeof(quadcolor_t));
    video_inform(VIDEO_FLAG_TYPE_CGA, &timing_quadcolor);

    display_type = device_get_config_int("display_type");
    quadcolor->composite = (display_type == CGA_COMPOSITE);
    quadcolor->revision = device_get_config_int("composite_type");
    quadcolor->has_2nd_charset = device_get_config_int("has_2nd_charset");
    quadcolor->has_quadcolor_2 = device_get_config_int("has_quadcolor_2");
    contrast = device_get_config_int("contrast");

    quadcolor->vram = malloc(0x8000);
    quadcolor->vram_2 = malloc(0x10000);

    cga_comp_init(quadcolor->revision);

    timer_add(&quadcolor->timer, quadcolor_poll, quadcolor, 1);
    mem_mapping_add(&quadcolor->mapping, 0xb8000, 0x08000, quadcolor_read, NULL, NULL, quadcolor_write, NULL, NULL, NULL, MEM_MAPPING_EXTERNAL,
            quadcolor);
    /* add mapping for vram_2 at 0xd0000, mirrored at 0xe0000 */
    if (quadcolor->has_quadcolor_2)
        mem_mapping_add(&quadcolor->mapping_2, 0xd0000, 0x20000, quadcolor_2_read, NULL, NULL, quadcolor_2_write, NULL, NULL, NULL, MEM_MAPPING_EXTERNAL,
                quadcolor);
    io_sethandler(0x03d0, 0x0010, quadcolor_in, NULL, NULL, quadcolor_out, NULL, NULL, quadcolor);

    cgapal_rebuild(display_type, contrast);

    return quadcolor;
}

void
quadcolor_close(void *priv)
{
    quadcolor_t *quadcolor = (quadcolor_t *) priv;

    free(quadcolor->vram);
    free(quadcolor->vram_2);
    free(quadcolor);
}

void
quadcolor_speed_changed(void *priv)
{
    quadcolor_t *quadcolor = (quadcolor_t *) priv;

    quadcolor_recalctimings(quadcolor);
}

// clang-format off
const device_config_t quadcolor_config[] = {
    {
        .name = "display_type",
        .description = "Display type",
        .type = CONFIG_SELECTION,
        .default_int = CGA_RGB,
        .selection = {
            {
                .description = "RGB",
                .value = CGA_RGB
            },
            {
                .description = "Composite",
                .value = CGA_COMPOSITE
            },
            {
                .description = ""
            }
        }
    },
    {
        .name = "composite_type",
        .description = "Composite type",
        .type = CONFIG_SELECTION,
        .default_int = COMPOSITE_OLD,
        .selection = {
            {
                .description = "Old",
                .value = COMPOSITE_OLD
            },
            {
                .description = "New",
                .value = COMPOSITE_NEW
            },
            {
                .description = ""
            }
        }
    },
    {
        .name = "rgb_type",
        .description = "RGB type",
        .type = CONFIG_SELECTION,
        .default_int = 5,
        .selection = {
            {
                .description = "Color (generic)",
                .value = 0
            },
            {
                .description = "Green Monochrome",
                .value = 1
            },
            {
                .description = "Amber Monochrome",
                .value = 2
            },
            {
                .description = "Gray Monochrome",
                .value = 3
            },
            {
                .description = "Color (no brown)",
                .value = 4
            },
            {
                .description = "Color (IBM 5153)",
                .value = 5
            },
            {
                .description = ""
            }
        }
    },
    {
        .name = "has_2nd_charset",
        .description = "Has secondary 8x8 character set",
        .type = CONFIG_BINARY,
        .default_int = 0
    },
    {
        .name = "has_quadcolor_2",
        .description = "Has Quadcolor II daughter board",
        .type = CONFIG_BINARY,
        .default_int = 1
    },
    {
        .name = "contrast",
        .description = "Alternate monochrome contrast",
        .type = CONFIG_BINARY,
        .default_int = 0
    },
    {
        .type = CONFIG_END
    }
};
// clang-format on

const device_t quadcolor_device = {
    .name          = "Quadram Quadcolor I / I+II",
    .internal_name = "quadcolor",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = quadcolor_standalone_init,
    .close         = quadcolor_close,
    .reset         = NULL,
    { .available   = NULL },
    .speed_changed = quadcolor_speed_changed,
    .force_redraw  = NULL,
    .config        = quadcolor_config
};
