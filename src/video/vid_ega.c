/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Emulation of the EGA and Chips & Technologies SuperEGA
 *          graphics cards.
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          Miran Grca, <mgrca8@gmail.com>
 *
 *          Copyright 2008-2019 Sarah Walker.
 *          Copyright 2016-2019 Miran Grca.
 */
#define _USE_MATH_DEFINES
#include <stdarg.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include "cpu.h"
#include <86box/io.h>
#include <86box/timer.h>
#include <86box/pic.h>
#include <86box/pit.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/device.h>
#include <86box/machine.h>
#include <86box/video.h>
#include <86box/vid_ati_eeprom.h>
#include <86box/vid_cga.h>
#include <86box/vid_cga_comp.h>
#include <86box/vid_ega.h>
#include <86box/plat_unused.h>

void ega_doblit(int wx, int wy, ega_t *ega);

#define BIOS_IBM_PATH    "roms/video/ega/ibm_6277356_ega_card_u44_27128.bin"
#define BIOS_CPQ_PATH    "roms/video/ega/108281-001.bin"
#define BIOS_SEGA_PATH   "roms/video/ega/lega.vbi"
#define BIOS_ATIEGA800P_PATH "roms/video/ega/ATI EGA Wonder 800+ N1.00.BIN"
#define BIOS_ISKRA_PATH  "roms/video/ega/143-02.bin", "roms/video/ega/143-03.bin"
#define BIOS_TSENG_PATH  "roms/video/ega/EGA ET2000.BIN"

static video_timings_t timing_ega = { .type = VIDEO_ISA, .write_b = 8, .write_w = 16, .write_l = 32, .read_b = 8, .read_w = 16, .read_l = 32 };
static uint8_t         ega_rotate[8][256];
static int             active             = 0;
uint32_t               pallook16[256];
uint32_t               pallook64[256];
static int             ega_type           = EGA_TYPE_IBM;

/* 3C2 controls default mode on EGA. On VGA, it determines monitor type (mono or colour):
    7=CGA mode (200 lines), 9=EGA mode (350 lines), 8=EGA mode (200 lines). */
int egaswitchread;
int egaswitches     = 9;
int update_overscan = 0;

#ifdef ENABLE_EGA_LOG
int ega_do_log = ENABLE_EGA_LOG;

static void
ega_log(const char *fmt, ...)
{
    va_list ap;

    if (ega_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define ega_log(fmt, ...)
#endif

uint8_t ega_in(uint16_t addr, void *priv);

void
ega_out(uint16_t addr, uint8_t val, void *priv)
{
    ega_t  *ega = (ega_t *) priv;
    uint8_t o;
    uint8_t old;
    int     type     = ega_type;
    int     atype    = ega->actual_type;
    uint8_t gdcmask  = (ega_type == EGA_SUPEREGA) ? 0xff : 0x0f;
    uint8_t crtcmask = (atype == EGA_SUPEREGA) ? 0xff : 0x1f;

    if (((((addr & 0xfff0) == 0x3d0) || ((addr & 0xfff0) == 0x2d0)) || (((addr & 0xfff0) == 0x3b0) || ((addr & 0xfff0) == 0x2b0))) && !(ega->miscout & 1))
        addr ^= 0x60;

    switch (addr) {
        case 0x1ce:
            ega->index = val;
            break;
        case 0x1cf:
            ega->regs[ega->index] = val;
            switch (ega->index) {
                case 0xb0:
                    ega_recalctimings(ega);
                    break;
                case 0xb3:
                    ati_eeprom_write((ati_eeprom_t *) ega->eeprom, val & 8, val & 2, val & 1);
                    break;

                default:
                    break;
            }
            break;

        case 0x2c0:
        case 0x3c0:
        case 0x2c1:
        case 0x3c1:
            if (atype == EGA_SUPEREGA)
                val &= 0x7f; /* Bit 7 indicates the flipflop status (read only) */
            if (!ega->attrff) {
                ega->attraddr = val & 31;
                if ((val & 0x20) != ega->attr_palette_enable) {
                    ega->fullchange          = 3;
                    ega->attr_palette_enable = val & 0x20;
                    ega_recalctimings(ega);
                }
            } else {
                if ((ega->attraddr == 0x13) && (ega->attrregs[0x13] != val))
                    ega->fullchange = changeframecount;
                uint8_t aidx        = ega->attraddr & 31;
                o                   = ega->attrregs[aidx];
                ega->attrregs[aidx] = val;
                if (ega->attraddr < 16)
                    ega->fullchange = changeframecount;
                int is_attr14 = ega->chipset && (ega->attraddr == 0x14);
                if ((ega->attraddr == 0x10) || is_attr14 || (ega->attraddr < 0x10)) {
                    for (uint8_t c = 0; c < 16; c++) {
                        if (ega->chipset) {
                            if (ega->attrregs[0x10] & 0x80)
                                ega->egapal[c] = (ega->attrregs[c] & 0xf) | ((ega->attrregs[0x14] & 0xf) << 4);
                            else
                                ega->egapal[c] = (ega->attrregs[c] & 0x3f) | ((ega->attrregs[0x14] & 0xc) << 4);
                        } else
                            ega->egapal[c] = ega->attrregs[c] & 0x3f;
                    }
                    ega->fullchange = changeframecount;
                }
                /* Recalculate timings on change of attribute register 0x11
                   (overscan border color) too. */
                if (ega->attraddr == 0x10) {
                    if (o != val)
                        ega_recalctimings(ega);
                } else if (ega->attraddr == 0x11) {
                    ega->overscan_color = ega->vres ? pallook16[val & 0x0f] : pallook64[val & 0x3f];
                    if (o != val)
                        ega_recalctimings(ega);
                } else if (ega->attraddr == 0x12)
                    ega->plane_mask = val & 0xf;
            }
            ega->attrff ^= 1;
            break;
        case 0x2c2:
        case 0x3c2:
            o                   = ega->miscout;
            egaswitchread       = (val & 0xc) >> 2;
            ega->vres           = !(val & 0x80);
            ega->pallook        = ega->vres ? pallook16 : pallook64;
            ega->vidclock       = val & 4;
            ega->miscout        = val;
            ega->overscan_color = ega->vres ? pallook16[ega->attrregs[0x11] & 0x0f] :
                                              pallook64[ega->attrregs[0x11] & 0x3f];

            uint16_t base_addr = 0x03a0;
#ifdef EGA_ALT_ADDR_SUPPORT
            if (ega->alt_addr == 1)
                base_addr = 0x02a0;
#endif
            if ((ega->priv_parent == NULL) && !ega->mono_io_disabled) {
                io_removehandler(base_addr, 0x0020, ega_in, NULL, NULL, ega_out, NULL, NULL, ega);
                if (!(val & 1))
                    io_sethandler(base_addr, 0x0020, ega_in, NULL, NULL, ega_out, NULL, NULL, ega);
            }
            ega_recalctimings(ega);
            if ((type == EGA_TYPE_COMPAQ) && !(val & 0x02))
                mem_mapping_disable(&ega->mapping);
            else switch (ega->gdcreg[6] & 0xc) {
                case 0x0: /*128k at A0000*/
                    mem_mapping_set_addr(&ega->mapping, 0xa0000, 0x20000);
                    break;
                case 0x4: /*64k at A0000*/
                    mem_mapping_set_addr(&ega->mapping, 0xa0000, 0x10000);
                    break;
                case 0x8: /*32k at B0000*/
                    mem_mapping_set_addr(&ega->mapping, 0xb0000, 0x08000);
                    break;
                case 0xC: /*32k at B8000*/
                    mem_mapping_set_addr(&ega->mapping, 0xb8000, 0x08000);
                    break;

                default:
                    break;
            }
            break;
        case 0x2c4:
        case 0x3c4:
            ega->seqaddr = val;
            break;
        case 0x2c5:
        case 0x3c5:
            o                                = ega->seqregs[ega->seqaddr & 0xf];
            ega->seqregs[ega->seqaddr & 0xf] = val;
            if (o != val && (ega->seqaddr & 0xf) == 1)
                ega_recalctimings(ega);
            switch (ega->seqaddr & 0xf) {
                case 1:
                    if (ega->scrblank && !(val & 0x20))
                        ega->fullchange = 3;
                    ega->scrblank = (ega->scrblank & ~0x20) | (val & 0x20);
                    break;
                case 2:
                    ega->writemask = val & 0xf;
                    break;
                case 3:
                    ega->charsetb = (((val >> 2) & 3) * 0x10000) + 2;
                    ega->charseta = ((val & 3) * 0x10000) + 2;
                    break;
                case 4:
                    ega->chain2_write = !(val & 4);
                    break;

                default:
                    break;
            }
            break;
        case 0x2c6:
        case 0x3c6:
            if (type == EGA_TYPE_COMPAQ)
                ega->ctl_mode = val;
            break;
        case 0x2ce:
        case 0x3ce:
            ega->gdcaddr = val;
            break;
        case 0x2cf:
        case 0x3cf: {
            uint8_t reg = ega->gdcaddr & gdcmask;
            ega->gdcreg[reg] = val;
            switch (reg) {
                case 2:
                    ega->colourcompare = val;
                    break;
                case 4:
                    ega->readplane = val & 3;
                    break;
                case 5:
                    ega->writemode   = val & 3;
                    ega->readmode    = val & 8;
                    ega->chain2_read = val & 0x10;
                    break;
                case 6:
                    if ((type == EGA_TYPE_COMPAQ) && !(ega->miscout & 0x02))
                        mem_mapping_disable(&ega->mapping);
                    else switch (val & 0xc) {
                        case 0x0: /*128k at A0000*/
                            mem_mapping_set_addr(&ega->mapping, 0xa0000, 0x20000);
                            break;
                        case 0x4: /*64k at A0000*/
                            mem_mapping_set_addr(&ega->mapping, 0xa0000, 0x10000);
                            break;
                        case 0x8: /*32k at B0000*/
                            mem_mapping_set_addr(&ega->mapping, 0xb0000, 0x08000);
                            break;
                        case 0xC: /*32k at B8000*/
                            mem_mapping_set_addr(&ega->mapping, 0xb8000, 0x08000);
                            break;

                        default:
                            break;
                    }
                    break;
                case 7:
                    ega->colournocare = val;
                    break;

                case 0xf8:
                    ega->la = val;
                    break;
                case 0xf9:
                    ega->lb = val;
                    break;
                case 0xfa:
                    ega->lc = val;
                    break;
                case 0xfb:
                    ega->ld = val;
                    break;

                default:
                    break;
            }
            break;
        }
        case 0x2d0:
        case 0x3d0:
        case 0x2d4:
        case 0x3d4:
            ega->crtcreg = ega->chipset ? (val & 0x3f) : val;
            return;
        case 0x2da:
        case 0x3da:
            /* Feature Control: bits 0-1 drive the FC0/FC1 feature connector pins. */
            ega->feat_ctrl = val & 0x03;
            break;
        case 0x2d1:
        case 0x3d1:
        case 0x2d5:
        case 0x3d5: {
            int idx = ega->crtcreg;

            if (ega->chipset) {
                if ((idx < 7) && (ega->crtc[0x11] & 0x80) && !(ega->regs[0xb4] & 0x80))
                    return;
                if ((idx == 7) && (ega->crtc[0x11] & 0x80) && !(ega->regs[0xb4] & 0x80))
                    val = (ega->crtc[7] & ~0x10) | (val & 0x10);
            } else {
                idx &= crtcmask;
                if ((idx >= 0x19) && (idx <= 0xf6))
                    return;
                if ((idx < 7) && (ega->crtc[0x11] & 0x80))
                    return;
                if ((idx == 7) && (ega->crtc[0x11] & 0x80))
                    val = (ega->crtc[7] & ~0x10) | (val & 0x10);
            }
            old            = ega->crtc[idx];
            ega->crtc[idx] = val;
            if (old != val) {
                if ((idx < 0xe) || (idx > 0x10)) {
                    if ((idx == 0xc) || (idx == 0xd)) {
                        ega->fullchange = 3;
                        ega->memaddr_latch   = ((ega->crtc[0xc] << 8) | ega->crtc[0xd]) +
                                          ((ega->crtc[8] & 0x60) >> 5);
                    } else {
                        ega->fullchange = changeframecount;
                        ega_recalctimings(ega);
                    }
                }
            }
            break;

        }

        default:
            break;
    }
}

uint8_t
ega_in(uint16_t addr, void *priv)
{
    ega_t  *ega      = (ega_t *) priv;
    uint8_t ret      = 0xff;
    int     type     = ega_type;
    int     atype    = ega->actual_type;
    uint8_t gdcmask  = (atype == EGA_SUPEREGA) ? 0xff : 0x0f;
    uint8_t crtcmask = (atype == EGA_SUPEREGA) ? 0xff : 0x1f;

    if (((((addr & 0xfff0) == 0x3d0) || ((addr & 0xfff0) == 0x2d0)) || (((addr & 0xfff0) == 0x3b0) || ((addr & 0xfff0) == 0x2b0))) && !(ega->miscout & 1))
        addr ^= 0x60;

    switch (addr) {
        case 0x1ce:
            ret = ega->index;
            break;
        case 0x1cf:
            switch (ega->index) {
                case 0xb7:
                    ret = ega->regs[ega->index] & ~8;
                    if (ati_eeprom_read((ati_eeprom_t *) ega->eeprom))
                        ret |= 8;
                    break;

                default:
                    ret = ega->regs[ega->index];
                    break;
            }
            break;

        case 0x2c0:
        case 0x3c0:
        case 0x2c1:
        case 0x3c1:
            if (type == EGA_TYPE_OTHER) {
                int data = (atype == EGA_SUPEREGA) ? (ega->attrff & 1) : (addr & 1);
                if (data)
                    ret = ega->attrregs[ega->attraddr];
                else
                    ret = ega->attraddr | ega->attr_palette_enable;
                if (atype == EGA_SUPEREGA)
                    /* Bit 7 indicates the flipflop status (read only) */
                    ret = (ret & 0x3f) | (ega->attrff ? 0x80 : 0x00);
            }
            break;
        case 0x2c2:
        case 0x3c2:
            ret = (egaswitches & (8 >> egaswitchread)) ? 0x10 : 0x00;
            /* Bits 5-6: FEAT0/FEAT1 inputs from the feature connector. */
            ret |= (ega->feat_in & 0x03) << 5;
            break;
        case 0x2c4:
        case 0x3c4:
            if (type == EGA_TYPE_OTHER) {
                if (atype == EGA_SUPEREGA)
                    ret = 0x1f | ((ega->miscout & 0x01) << 5);
                else
                    ret = ega->seqaddr;
            }
            break;
        case 0x2c5:
        case 0x3c5:
            if (type == EGA_TYPE_OTHER) {
                uint8_t idx = ega->seqaddr & 0xf;
                if (idx > 0x04)
                    ret = ega->chipset ? ega->seqregs[idx] : 0xff;
                else
                    ret = ega->seqregs[idx];
            }
            break;
        case 0x2c6:
        case 0x3c6:
            if (type == EGA_TYPE_COMPAQ)
                ret = ega->ctl_mode;
            break;
        case 0x2c8:
        case 0x3c8:
            if (type == EGA_TYPE_OTHER)
                ret = 2;
            break;
        case 0x2cc:
        case 0x3cc:
            if (type == EGA_TYPE_OTHER)
                ret = ega->miscout;
            break;
        case 0x2ce:
        case 0x3ce:
            if (type == EGA_TYPE_OTHER) {
                ret = ega->gdcaddr;
                if (atype == EGA_SUPEREGA) {
                    ret = (ret & 0x0f) | 0xe0;
                    if ((ega->gdcaddr & 0xe0) == 0xe0)
                        ret |= 0x10;
                }
            }
            break;
        case 0x2cf:
        case 0x3cf:
            if (type == EGA_TYPE_OTHER) {
                uint8_t gidx = ega->gdcaddr & gdcmask;
                switch (gidx) {
                    default:
                        ret = ega->gdcreg[gidx];
                        break;
                    case 0x09 ... 0xf7:
                        ret = ega->chipset ? ega->gdcreg[gidx] : 0xff;
                        break;
                    case 0xf8:
                        ret = ega->la;
                        break;
                    case 0xf9:
                        ret = ega->lb;
                        break;
                    case 0xfa:
                        ret = ega->lc;
                        break;
                    case 0xfb:
                        ret = ega->ld;
                        break;
                }
            }
            break;
        case 0x2d0:
        case 0x3d0:
        case 0x2d4:
        case 0x3d4:
            if (type == EGA_TYPE_OTHER) {
                ret = ega->crtcreg;
                if (atype == EGA_SUPEREGA) {
                    ret = (ret & 0x1f) | 0xc0;
                    if ((ega->crtcreg & 0xc0) == 0xc0)
                        ret |= 0x20;
                }
            }
            break;
        case 0x2d1:
        case 0x3d1:
        case 0x2d5:
        case 0x3d5:
            switch (ega->crtcreg & crtcmask) {
                case 0xc:
                case 0xd:
                case 0xe:
                case 0xf:
                    ret = ega->crtc[ega->crtcreg];
                    break;

                case 0x10:
                    if (type == EGA_TYPE_OTHER)
                        ret = ega->crtc[ega->crtcreg];
                    else
                        ret = ega->light_pen >> 8;
                    break;

                case 0x11:
                    if (type == EGA_TYPE_OTHER)
                        ret = ega->crtc[ega->crtcreg];
                    else
                        ret = ega->light_pen & 0xff;
                    break;

                case 0x19 ... 0xf6:
                    if (type == EGA_TYPE_OTHER)
                        ret = ega->chipset ? ega->crtc[ega->crtcreg] : 0xff;
                    break;

                default:
                    if (type == EGA_TYPE_OTHER)
                        ret = ega->crtc[ega->crtcreg];
                    break;
            }
            break;
        case 0x2da:
        case 0x3da:
            ega->attrff = 0;
            if (type == EGA_TYPE_COMPAQ) {
                ret = ega->status & 0xcf;
                switch ((ega->attrregs[0x12] >> 4) & 0x03) {
                    case 0x00:
                        /* 00 = Pri. Red (5), Pri. Blue (4) */
                        ret |= (ega->color_mux & 0x04) ? 0x20 : 0x00;
                        ret |= (ega->color_mux & 0x01) ? 0x10 : 0x00;
                        break;
                    case 0x01:
                    case 0x03:
                        /* 01 = Sec. Red (5), Sec. Green (4) */
                        /* 11 = Sec. Red (5), Sec. Green (4) */
                        ret |= (ega->color_mux & 0x20) ? 0x20 : 0x00;
                        ret |= (ega->color_mux & 0x10) ? 0x10 : 0x00;
                        break;
                    case 0x02:
                        /* 10 = Sec. Blue (5), Pri. Green (4) */
                        ret |= (ega->color_mux & 0x08) ? 0x20 : 0x00;
                        ret |= (ega->color_mux & 0x02) ? 0x10 : 0x00;
                        break;
                }
            } else {
                ega->status ^= 0x30; /* Fools IBM EGA video BIOS self-test. */
                ret = ega->status;
            }
            break;
        case 0x7c6:
            ret = 0xfd; /* EGA mode supported. */
            break;
        case 0xbc6:
            /* 0000 = None;
               0001 = Compaq Dual-Mode (DM) Monitor;
               0010 = RGBI Color Monitor;
               0011 = COMPAQ Color Monitor (RrGgBb) or Compatible;
               0100 - 1111 = Reserved. */
            ret = 0x01;
            break;
        case 0xfc6:
            ret = 0xfd;
            break;

        default:
            break;
    }

    return ret;
}

static void ega_pal_recalc(void *priv, double dot_hz, double line_dots, int vtotal);

static void
ega_recalctimings_common(ega_t *ega)
{
    int clksel;
    int color;

    double _dispontime;
    double _dispofftime;
    double disptime;
    double crtcconst;
    double mdiv = (ega->seqregs[1] & 1) ? 8.0 : 9.0;

    ega->vtotal     = ega->crtc[6];
    ega->dispend    = ega->crtc[0x12];
    ega->vsyncstart = ega->crtc[0x10];
    ega->split      = ega->crtc[0x18];

    if (ega->crtc[7] & 1)
        ega->vtotal |= 0x100;
    if ((ega->actual_type != EGA_SUPEREGA) && (ega->crtc[7] & 32))
        ega->vtotal |= 0x200;
    ega->vtotal += 2;

    if (ega->crtc[7] & 2)
        ega->dispend |= 0x100;
    if ((ega->actual_type != EGA_SUPEREGA) && (ega->crtc[7] & 64))
        ega->dispend |= 0x200;
    ega->dispend++;

    if (ega->crtc[7] & 4)
        ega->vsyncstart |= 0x100;
    if ((ega->actual_type != EGA_SUPEREGA) && (ega->crtc[7] & 128))
        ega->vsyncstart |= 0x200;
    ega->vsyncstart++;

    if (ega->crtc[7] & 0x10)
        ega->split |= 0x100;
    if (ega->crtc[9] & 0x40)
        ega->split |= 0x200;
    ega->split++;

    ega->hdisp = ega->crtc[1];
    ega->hdisp++;

    ega->rowoffset = ega->crtc[0x13];

    ega->linedbl  = ega->crtc[9] & 0x80;
    ega->rowcount = ega->crtc[9] & 0x1f;

    if (ega->actual_type == EGA_SUPEREGA) {
        switch ((ega->miscout >> 2) & 0x03) {
            case 0x00:
                crtcconst = (cpuclock / 16257000.0 * (double) (1ULL << 32));
                break;
            case 0x01:
                crtcconst = (cpuclock / (157500000.0 / 11.0) * (double) (1ULL << 32));
                break;
            default:
            case 0x02: case 0x03:
                crtcconst = (cpuclock / 25110000.0 * (double) (1ULL << 32));
                break;
        }

        crtcconst *= mdiv;
    } else if (ega_type == EGA_TYPE_COMPAQ) {
        color = (ega->miscout & 1);
        clksel = ((ega->miscout & 0xc) >> 2);

        if (color) {
            if (ega->vidclock)
                crtcconst = (cpuclock / 16257000.0 * (double) (1ULL << 32));
            else
                crtcconst = (cpuclock / (157500000.0 / 11.0) * (double) (1ULL << 32));
        } else {
            if (ega->vidclock)
                crtcconst = (cpuclock / 18981000.0 * (double) (1ULL << 32));
            else
                crtcconst = (cpuclock / 16872000.0 * (double) (1ULL << 32));
        }
        crtcconst *= mdiv;
    } else if (ega->eeprom) {
        clksel = ((ega->miscout & 0xc) >> 2) | ((ega->regs[0xbe] & 0x10) ? 4 : 0);

        switch (clksel) {
            case 0:
                crtcconst = (cpuclock / 25175000.0 * (double) (1ULL << 32));
                break;
            case 1:
                crtcconst = (cpuclock / 28322000.0 * (double) (1ULL << 32));
                break;
            case 4:
                crtcconst = (cpuclock / 14318181.0 * (double) (1ULL << 32));
                break;
            case 5:
                crtcconst = (cpuclock / 16257000.0 * (double) (1ULL << 32));
                break;
            case 7:
            default:
                crtcconst = (cpuclock / 36000000.0 * (double) (1ULL << 32));
                break;
        }
        crtcconst *= mdiv;
    } else {
        clksel = (ega->miscout >> 2) & 0x03;

        /* Clock select 10b takes the dot clock from the feature connector; with
           no card supplying one, keep the previous (bit 2 only) behaviour. */
        if ((clksel == 2) && (ega->ext_clock > 0.0))
            crtcconst = (cpuclock / ega->ext_clock * (double) (1ULL << 32));
        else if (ega->vidclock)
            crtcconst = (cpuclock / 16257000.0 * (double) (1ULL << 32));
        else
            crtcconst = (cpuclock / (157500000.0 / 11.0) * (double) (1ULL << 32));
        crtcconst *= mdiv;
    }

    /* Misc Output bit 4 turns off the IBM EGA's own monitor drivers; unless a
       feature card produces its own picture, the monitor goes dark. */
    ega->drivers_off = (ega->actual_type == EGA_IBM) && (ega->miscout & 0x10) &&
                       ((ega->feature_card == EGA_FEATURE_NONE) || (ega->feature_card == EGA_FEATURE_132COL));

    /* The PAL encoder works on buffer pixels, which always come at the master
       dot clock (a halved clock is drawn as doubled pixels). */
    if (ega->pal) {
        const int    clk = (ega->miscout >> 2) & 0x03;
        const double hz  = ((clk == 2) && (ega->ext_clock > 0.0)) ? ega->ext_clock :
                           (ega->vidclock ? 16257000.0 : (157500000.0 / 11.0));
        const double chars = (ega->seqregs[1] & 8) ? ((ega->crtc[0] + 2) << 1) : (ega->crtc[0] + 2);

        ega_pal_recalc(ega->pal, hz, chars * mdiv, ega->vtotal);
    }
    ega->dot_clock = crtcconst / mdiv;

    ega->interlace = 0;

    ega->memaddr_latch = (ega->crtc[0xc] << 8) | ega->crtc[0xd];

    ega->render = ega_render_blank;
    if (!ega->scrblank && ega->attr_palette_enable && !ega->drivers_off) {
        if (!(ega->gdcreg[6] & 1)) {
            if (ega->seqregs[1] & 8)
                ega->hdisp *= (ega->seqregs[1] & 1) ? 16 : 18;
            else
                ega->hdisp *= (ega->seqregs[1] & 1) ? 8 : 9;
            ega->render = ega_render_text;
        } else {
            ega->hdisp *= (ega->seqregs[1] & 8) ? 16 : 8;
            ega->render = ega_render_graphics;
        }
        ega->hdisp_old = ega->hdisp;
    }

    if (ega->chipset) {
        if (ega->hdisp >= 800) {
            ega->dispend <<= 1;
            ega->vtotal <<= 1;
            ega->split <<= 1;
            ega->vsyncstart <<= 1;
        }
    }

    overscan_y = (ega->rowcount + 1) << 1;

    if (overscan_y < 16)
        overscan_y = 16;

    overscan_x = (ega->seqregs[1] & 1) ? 16 : 18;

    if (ega->vres)
        overscan_y <<= 1;

    if (ega->seqregs[1] & 8)
        overscan_x <<= 1;

    ega->y_add = (overscan_y >> 1);
    ega->x_add = (overscan_x >> 1) - ega->scrollcache;

    if (ega->vres)
        ega->y_add >>= 1;

    if (ega->seqregs[1] & 8) {
        disptime     = (double) ((ega->crtc[0] + 2) << 1);
        _dispontime  = (double) ((ega->crtc[1] + 1) << 1);
    } else {
        disptime     = (double) (ega->crtc[0] + 2);
        _dispontime  = (double) (ega->crtc[1] + 1);
    }
    if ((ega->actual_type == EGA_SUPEREGA) && (ega->crtc[0x17] & 0x10) &&
        (ega->crtc[0xf9] & 0x01)) {
        disptime    *= 2.0;
        _dispontime *= 2.0;
    }
    _dispofftime = disptime - _dispontime;
    _dispontime *= crtcconst;
    _dispofftime *= crtcconst;

    ega->dispontime  = (uint64_t) (int64_t) (_dispontime);
    ega->dispofftime = (uint64_t) (int64_t) (_dispofftime);
    if (ega->dispontime < TIMER_USEC)
        ega->dispontime = TIMER_USEC;
    if (ega->dispofftime < TIMER_USEC)
        ega->dispofftime = TIMER_USEC;

    if (ega_type == EGA_TYPE_COMPAQ) {
        ega->dot_time  = (uint64_t) (int64_t) (ega->dot_clock);
        if (ega->dot_time < TIMER_USEC)
            ega->dot_time = TIMER_USEC;
        timer_disable(&ega->dot_timer);
        timer_set_delay_u64(&ega->dot_timer, ega->dot_time);
        ega->cca = 0;
        active   = 1;
        ega->dot = 0;
    }

    ega_recalc_remap_func(ega);
}

/* The overscan and size globals resolve through monitor_index_global, and this
   is reached from port writes made while it points at the primary display, so
   bind it to the monitor this card actually drives. */
void
ega_recalctimings(ega_t *ega)
{
    const int prev_monitor_index = monitor_index_global;

    monitor_index_global = ega->monitor_index;
    ega_recalctimings_common(ega);
    monitor_index_global = prev_monitor_index;
}

/* This is needed for the Compaq EGA so that it can pass the 3DA
   palette mux part of the self-test. */
void
ega_dot_poll(void *priv)
{
    ega_t          *ega           = (ega_t *) priv;
    static uint8_t  chr;
    static uint8_t  attr;
    const bool      doublewidth   = ((ega->seqregs[1] & 8) != 0);
    const bool      attrblink     = ((ega->attrregs[0x10] & 8) != 0);
    const bool      attrlinechars = (ega->attrregs[0x10] & 4);
    const bool      crtcreset     = ((ega->crtc[0x17] & 0x80) == 0);
    const bool      seq9dot       = ((ega->seqregs[1] & 1) == 0);
    const bool      blinked       = ega->blink & 0x10;
    const int       dwshift       = doublewidth ? 1 : 0;
    const int       dotwidth      = 1 << dwshift;
    const int       charwidth     = dotwidth * (seq9dot ? 9 : 8);
    const int       cursoron      = (ega->scanline == (ega->crtc[10] & 31));
    const int       cursoraddr    = (ega->crtc[0xe] << 8) | ega->crtc[0xf];
    uint32_t        addr;
    int             drawcursor;
    uint32_t        charaddr;
    static int      fg            = 0;
    static int      bg            = 0;
    static uint32_t dat           = 0x00000000;
    static int      cclock        = 0;
    static int      disptime;
    static int      _dispontime;
    static int      _dispofftime;

    if (ega->seqregs[1] & 8) {
        disptime    = ((ega->crtc[0] + 2) << 1);
        _dispontime = ((ega->crtc[1] + 1) << 1);
    } else {
        disptime    = (ega->crtc[0] + 2);
        _dispontime = (ega->crtc[1] + 1);
    }
    _dispofftime = disptime - _dispontime;

    timer_advance_u64(&ega->dot_timer, ega->dot_time);

    if (ega->render == ega_render_text)
        ega->color_mux = (dat & (0x100 >> (ega->dot >> dwshift))) ? fg : bg;
    else
        ega->color_mux = 0x00;

    addr = ega->remap_func(ega, ega->cca) & ega->vrammask;

    if (!crtcreset) {
        chr  = ega->vram[addr];
        attr = ega->vram[addr + 1];
    } else
        chr = attr = 0;

    drawcursor = ((ega->cca == cursoraddr) && cursoron && ega->cursoron);

    if (attr & 8)
        charaddr = ega->charsetb + (chr * 0x80);
    else
        charaddr = ega->charseta + (chr * 0x80);

    dat = ega->vram[charaddr + (ega->scanline << 2)];
    dat <<= 1;
    if ((chr & ~0x1F) == 0xC0 && attrlinechars)
        dat |= (dat >> 1) & 1;

    if (!active)
        dat = 0x200;

    if (drawcursor) {
        bg = ega->egapal[attr & 0x0f];
        fg = ega->egapal[attr >> 4];
    } else {
        fg = ega->egapal[attr & 0x0f];
        bg = ega->egapal[attr >> 4];
        if ((attr & 0x80) && attrblink) {
            bg = ega->egapal[(attr >> 4) & 7];
            if (blinked)
                fg = bg;
        }
    }

    ega->dot = (ega->dot + 1) % charwidth;

    if (ega->dot == 0) {
        ega->cca = (ega->cca + 4) & 0x3ffff;

        cclock++;

        if (active && (cclock == _dispofftime))
            active = 0;
        else if (!active && (cclock == _dispontime))
            active = 1;
    }
}

/* ---- PAL colour composite ------------------------------------------------
   86Box's CGA decoder is NTSC only, so the PAL card gets its own model, run on
   buffer pixels at the EGA's master dot clock:

   encoder: RGBI -> R'G'B' (2/3 per gun + 1/3 intensity) -> Y, U, V;
            composite = Y + U sin(wt) +/- V cos(wt), V switched every line,
            subcarrier phase carried across pixels, lines and frames.
   decoder: synchronous demodulation (2 sin, +/-2 cos) and a 1.3 MHz chroma
            low-pass, luma = composite minus the re-modulated chroma, then a
            PAL delay line (U and V averaged with the previous line), then RGB.

   There is no hue error to correct, so the delay line only adds the vertical
   chroma blur and the cancelling of cross-colour that a real PAL set shows. */
#define EGA_PAL_TAPS     15      /* Odd; chroma low-pass length. */
#define EGA_PAL_TRAP_TAPS 25     /* Odd; luma trap length (about -30 dB at the subcarrier). */
#define EGA_PAL_CHROMA_F 1300000.0
#define EGA_PAL_MAXW     2048

typedef struct ega_pal_t {
    double fs;            /* Sample (master dot clock) rate in Hz. */
    double step;          /* Subcarrier cycles per sample. */
    double cyc_per_line;  /* Subcarrier cycles per scanline. */
    double phase;         /* Subcarrier phase at the start of the current line, in cycles. */
    int    vsign;         /* PAL V switch for the current line. */

    float  lpf[EGA_PAL_TAPS];
    float  trap[EGA_PAL_TRAP_TAPS]; /* Luma notch at the subcarrier. */

    int    lock_mode;     /* EGA_PAL_LOCK_* */
    int    luma_mode;     /* EGA_PAL_LUMA_* */
    double h_rate;        /* Line rate of the signal (Hz). */
    double v_rate;        /* Field rate of the signal (Hz). */
    int    vtotal;        /* Scanlines per field. */
    int    h_locked;      /* The set's line timebase locks to the signal. */
    int    v_locked;      /* The set's field timebase locks to the signal. */
    double roll;          /* Vertical roll position, in fields (0..1). */
    uint32_t *roll_buf;   /* Frame copy for rolling. */
    size_t   roll_size;
    float  y[16], u[16], v[16];
    uint32_t rgb[16];     /* Steady-state colour, for flat border lines. */

    float  comp[EGA_PAL_MAXW], s[EGA_PAL_MAXW], c[EGA_PAL_MAXW];
    float  du[EGA_PAL_MAXW], dv[EGA_PAL_MAXW];
    float  prev_u[EGA_PAL_MAXW], prev_v[EGA_PAL_MAXW];
} ega_pal_t;

static uint32_t
ega_pal_yuv_to_rgb(float y, float u, float v)
{
    int r = (int) ((y + (1.13983f * v)) * 255.0f + 0.5f);
    int g = (int) ((y - (0.39465f * u) - (0.58060f * v)) * 255.0f + 0.5f);
    int b = (int) ((y + (2.03211f * u)) * 255.0f + 0.5f);

    r = (r < 0) ? 0 : ((r > 255) ? 255 : r);
    g = (g < 0) ? 0 : ((g > 255) ? 255 : g);
    b = (b < 0) ? 0 : ((b > 255) ? 255 : b);

    return (r << 16) | (g << 8) | b;
}

/* (Re)builds the chroma low-pass for a new sample rate. */
static void
ega_pal_set_rate(ega_pal_t *pal, double fs)
{
    const int    half = EGA_PAL_TAPS / 2;
    const double fc   = EGA_PAL_CHROMA_F / fs;
    double       sum  = 0.0;
    double       h[EGA_PAL_TAPS];

    if (fs == pal->fs)
        return;

    pal->fs   = fs;
    pal->step = EGA_PAL_FSC / fs;

    for (int n = 0; n < EGA_PAL_TAPS; n++) {
        const int    k    = n - half;
        const double sinc = (k == 0) ? (2.0 * fc) : (sin(2.0 * M_PI * fc * k) / (M_PI * k));
        const double win  = 0.54 - 0.46 * cos(2.0 * M_PI * n / (EGA_PAL_TAPS - 1)); /* Hamming */

        h[n] = sinc * win;
        sum += h[n];
    }
    for (int n = 0; n < EGA_PAL_TAPS; n++)
        pal->lpf[n] = (float) (h[n] / sum);

    /* Luma trap: unit impulse minus a band-pass from TRAP_LO to TRAP_HI (the
       difference of two windowed-sinc low-passes); unity gain at DC. */
    const double f1 = EGA_PAL_TRAP_LO / fs;
    const double f2 = MIN(EGA_PAL_TRAP_HI / fs, 0.499);

    for (int n = 0; n < EGA_PAL_TRAP_TAPS; n++) {
        const int    k   = n - (EGA_PAL_TRAP_TAPS / 2);
        const double win = 0.54 - 0.46 * cos(2.0 * M_PI * n / (EGA_PAL_TRAP_TAPS - 1));
        const double lp2 = (k == 0) ? (2.0 * f2) : (sin(2.0 * M_PI * f2 * k) / (M_PI * k));
        const double lp1 = (k == 0) ? (2.0 * f1) : (sin(2.0 * M_PI * f1 * k) / (M_PI * k));

        pal->trap[n] = (float) (((k == 0) ? 1.0 : 0.0) - ((lp2 - lp1) * win));
    }
}

static ega_pal_t *
ega_pal_init(int lock_mode, int luma_mode)
{
    ega_pal_t *pal = calloc(1, sizeof(ega_pal_t));

    pal->lock_mode = lock_mode;
    pal->luma_mode = luma_mode;
    pal->h_locked  = 1;
    pal->v_locked  = 1;

    for (int c = 0; c < 16; c++) {
        const float i = (c & 8) ? (1.0f / 3.0f) : 0.0f;
        const float r = ((c & 4) ? (2.0f / 3.0f) : 0.0f) + i;
        const float g = ((c & 2) ? (2.0f / 3.0f) : 0.0f) + i;
        const float b = ((c & 1) ? (2.0f / 3.0f) : 0.0f) + i;

        pal->y[c]   = (0.299f * r) + (0.587f * g) + (0.114f * b);
        pal->u[c]   = 0.492f * (b - pal->y[c]);
        pal->v[c]   = 0.877f * (r - pal->y[c]);
        pal->rgb[c] = ega_pal_yuv_to_rgb(pal->y[c], pal->u[c], pal->v[c]);
    }

    pal->vsign = 1;
    ega_pal_set_rate(pal, 157500000.0 / 11.0);

    return pal;
}

static void
ega_pal_recalc(void *priv, double dot_hz, double line_dots, int vtotal)
{
    ega_pal_t *pal = (ega_pal_t *) priv;
    double     v_max;

    ega_pal_set_rate(pal, dot_hz);
    pal->cyc_per_line = line_dots * EGA_PAL_FSC / dot_hz;

    pal->vtotal = (vtotal > 0) ? vtotal : 1;
    pal->h_rate = dot_hz / line_dots;
    pal->v_rate = pal->h_rate / pal->vtotal;

    switch (pal->lock_mode) {
        case EGA_PAL_LOCK_IDEAL:
            pal->h_locked = pal->v_locked = 1;
            break;

        default:
            v_max = (pal->lock_mode == EGA_PAL_LOCK_50HZ) ? EGA_PAL_TV_V_MAX_50HZ : EGA_PAL_TV_V_MAX_PAL60;

            pal->h_locked = (pal->h_rate >= EGA_PAL_TV_H_MIN) && (pal->h_rate <= EGA_PAL_TV_H_MAX);
            pal->v_locked = pal->h_locked && (pal->v_rate >= EGA_PAL_TV_V_MIN) && (pal->v_rate <= v_max);
            break;
    }

    if (pal->v_locked)
        pal->roll = 0.0;
}

/* Rolls the finished frame on a set whose field timebase isn't locked. The set
   free-runs at EGA_PAL_TV_V_FREE, so every signal field the picture slips by
   (v_rate - free) / v_rate of a field; the field's vertical blanking shows as a
   black bar travelling with it. rows is the visible height in buffer lines and
   field the height a whole field (blanking included) would take. */
static void
ega_pal_roll(ega_pal_t *pal, int rows, int field, int width)
{
    size_t need;
    int    offset;

    if (pal->v_locked || !pal->h_locked || (rows <= 0) || (width <= 0) || (buffer32 == NULL))
        return;

    rows  = MIN(rows, 2048);
    width = MIN(width, 2048);
    if (field <= rows)
        field = rows + (rows / 12) + 1; /* Always leave a visible blanking bar. */

    pal->roll += (pal->v_rate - EGA_PAL_TV_V_FREE) / pal->v_rate;
    pal->roll -= floor(pal->roll);
    offset = (int) (pal->roll * field);

    need = (size_t) rows * width;
    if (pal->roll_size < need) {
        free(pal->roll_buf);
        pal->roll_buf  = malloc(need * sizeof(uint32_t));
        pal->roll_size = pal->roll_buf ? need : 0;
        if (pal->roll_buf == NULL)
            return;
    }

    for (int i = 0; i < rows; i++)
        memcpy(&pal->roll_buf[(size_t) i * width], buffer32->line[i], width * sizeof(uint32_t));

    for (int i = 0; i < rows; i++) {
        const int v = (i + offset) % field;

        if (v < rows)
            memcpy(buffer32->line[i], &pal->roll_buf[(size_t) v * width], width * sizeof(uint32_t));
        else
            memset(buffer32->line[i], 0, width * sizeof(uint32_t));
    }
}

static void
ega_pal_close(ega_pal_t *pal)
{
    free(pal->roll_buf);
    free(pal);
}

/* Called once per scanline, displayed or not, so the subcarrier phase and the
   V switch run on through the blanking intervals as they do on real hardware. */
static void
ega_pal_next_line(ega_pal_t *pal)
{
    pal->phase += pal->cyc_per_line;
    pal->phase -= floor(pal->phase);
    pal->vsign = -pal->vsign;
}

/* Encodes and decodes one line of RGBI indices in place. */
static void
ega_pal_process(ega_pal_t *pal, uint32_t *p, int width)
{
    const int   half  = EGA_PAL_TAPS / 2;
    const float vsign = (float) pal->vsign;

    if (width > EGA_PAL_MAXW)
        width = EGA_PAL_MAXW;

    /* Encode. */
    for (int n = 0; n < width; n++) {
        const int    idx = p[n] & 0x0f;
        const double ph  = 2.0 * M_PI * (pal->phase + (n * pal->step));

        pal->s[n]    = (float) sin(ph);
        pal->c[n]    = (float) cos(ph);
        pal->comp[n] = pal->y[idx] + (pal->u[idx] * pal->s[n]) + (vsign * pal->v[idx] * pal->c[n]);
    }

    /* Demodulate and low-pass the chroma (edges clamp to the end samples). */
    for (int n = 0; n < width; n++) {
        float u = 0.0f;
        float v = 0.0f;

        for (int t = 0; t < EGA_PAL_TAPS; t++) {
            int k = n + t - half;

            k = (k < 0) ? 0 : ((k >= width) ? (width - 1) : k);
            u += pal->lpf[t] * 2.0f * pal->comp[k] * pal->s[k];
            v += pal->lpf[t] * 2.0f * vsign * pal->comp[k] * pal->c[k];
        }
        pal->du[n] = u;
        pal->dv[n] = v;
    }

    /* Luma, delay line, RGB. */
    for (int n = 0; n < width; n++) {
        float y;

        if (pal->luma_mode == EGA_PAL_LUMA_TRAP) {
            y = 0.0f;
            for (int t = 0; t < EGA_PAL_TRAP_TAPS; t++) {
                int k = n + t - (EGA_PAL_TRAP_TAPS / 2);

                k = (k < 0) ? 0 : ((k >= width) ? (width - 1) : k);
                y += pal->trap[t] * pal->comp[k];
            }
        } else
            y = pal->comp[n] - ((pal->du[n] * pal->s[n]) + (vsign * pal->dv[n] * pal->c[n]));
        const float u = 0.5f * (pal->du[n] + pal->prev_u[n]);
        const float v = 0.5f * (pal->dv[n] + pal->prev_v[n]);

        pal->prev_u[n] = pal->du[n];
        pal->prev_v[n] = pal->dv[n];

        p[n] = ega_pal_yuv_to_rgb(y, u, v);
    }
}

/* CGA mode register value the colour composite decoder is run with: 640x200
   graphics with the colour burst on, so it decodes the RGBI stream exactly as it
   comes from the EGA, with no CGA text-mode or B&W quirks applied. */
#define EGA_COMP_CGAMODE (CGA_MODE_FLAG_GRAPHICS | CGA_MODE_FLAG_VIDEO_ENABLE | CGA_MODE_FLAG_HIGHRES_GRAPHICS)

/* Post-processes one buffer line, of width pixels, for what the feature
   connector does to the picture. The line holds palette output, which with a
   composite card is the raw RGBI index (see ega_feature_init()). */
static void
ega_feature_process_line(ega_t *ega, int line, int width, int border)
{
    uint32_t *p;

    if ((line < 0) || (buffer32 == NULL))
        return;

    p = buffer32->line[line & 0x7ff];
    if (p == NULL)
        return;

    if (width > 2044)
        width = 2044;

    if (ega->drivers_off) {
        for (int x = 0; x < width; x++)
            p[x] = 0x00000000;
    } else if ((ega->feature_card == EGA_FEATURE_COLOR_COMP) && (width > 0))
        Composite_Process(EGA_COMP_CGAMODE, ega->overscan_color & 0x0f, (width + 3) >> 2, p);
    else if ((ega->feature_card == EGA_FEATURE_COLOR_COMP_PAL) && (width > 0)) {
        ega_pal_t *pal = (ega_pal_t *) ega->pal;

        /* No line lock, no picture. A flat border line decodes back to its own
           colour, and must not disturb the delay line, which holds the last
           picture line. */
        if (!pal->h_locked) {
            for (int x = 0; x < width; x++)
                p[x] = 0x00000000;
        } else if (border) {
            for (int x = 0; x < width; x++)
                p[x] = pal->rgb[p[x] & 0x0f];
        } else
            ega_pal_process(pal, p, width);
    }
}

static inline int
ega_feature_needs_processing(const ega_t *ega)
{
    return ega->drivers_off || EGA_FEATURE_IS_COMPOSITE(ega->feature_card);
}

static void
ega_poll_common(ega_t *ega)
{
    int      old_ma;
    int      wx = 640;
    int      wy = 350;

    if (!ega->linepos) {
        timer_advance_u64(&ega->timer, ega->dispofftime);
        ega->status |= 1;
        ega->linepos = 1;

        if (ega->pal)
            ega_pal_next_line((ega_pal_t *) ega->pal);

        if (ega->dispon) {
            ega->hdisp_on = 1;

            ega->memaddr &= ega->vrammask;
            if (ega->firstline == 2000) {
                ega->firstline = ega->displine;
                video_wait_for_buffer();
            }

            /* Text lines are normally only redrawn when something changed, but
               the composite decoder overwrites the buffer with its output, so
               the indices it works from have to be rendered every time. */
            if (EGA_FEATURE_IS_COMPOSITE(ega->feature_card))
                ega->fullchange = 2;

            old_ma = ega->memaddr;
            ega->displine *= ega->vres + 1;
            ega->y_add *= ega->vres + 1;
            for (int y = 0; y <= ega->vres; y++) {
                /* Render scanline */
                ega->render(ega);

                /* Render overscan */
                ega->x_add = (overscan_x >> 1);
                ega_render_overscan_left(ega);
                ega_render_overscan_right(ega);
                ega->x_add = (overscan_x >> 1) - ega->scrollcache;

                /* Feature connector output (composite encoding, drivers off). */
                if ((y > 0) && (ega->feature_card == EGA_FEATURE_COLOR_COMP_PAL) && !ega->drivers_off) {
                    /* A line-doubled 200-line scanline is one line on the wire:
                       repeat the decoded result rather than run the delay line
                       and the V switch a second time. */
                    const int dl = ega->displine + ega->y_add;

                    if ((dl > 0) && (buffer32 != NULL) && buffer32->line[dl & 0x7ff] && buffer32->line[(dl - 1) & 0x7ff])
                        memcpy(buffer32->line[dl & 0x7ff], buffer32->line[(dl - 1) & 0x7ff],
                               MIN(overscan_x + ega->hdisp + ega->scrollcache, 2044) * sizeof(uint32_t));
                } else if (ega_feature_needs_processing(ega))
                    ega_feature_process_line(ega, ega->displine + ega->y_add,
                                             ega->drivers_off ? (xsize + overscan_x) :
                                                                (overscan_x + ega->hdisp + ega->scrollcache), 0);

                if (y != ega->vres) {
                    ega->memaddr = old_ma;
                    ega->displine++;
                }
            }
            ega->displine /= ega->vres + 1;
            ega->y_add /= ega->vres + 1;

            if (ega->lastline < ega->displine)
                ega->lastline = ega->displine;
        }

        video_lightpen_check_trigger_strobe(8, ega->displine, 0, ega->firstline, (1. / (ega->dot_clock / (cpuclock * (double) (1ULL << 32)))) * ((ega->seqregs[1] & 1) ? 8.0 : 9.0), ega->monitor_index);
        ega->displine++;
        if (ega->interlace)
            ega->displine++;
        if ((ega->status & 8) && ((ega->displine & 15) == (ega->crtc[0x11] & 15)) && ega->vslines)
            ega->status &= ~8;
        ega->vslines++;
        if (ega->chipset) {
            if (ega->hdisp >= 800) {
                if (ega->displine > 2000)
                    ega->displine = 0;
            } else {
                if (ega->displine > 500)
                    ega->displine = 0;
            }
        } else {
            if (ega->displine > 500)
                ega->displine = 0;
        }
    } else {
        timer_advance_u64(&ega->timer, ega->dispontime);
        video_lightpen_hsync();

        if (ega->dispon)
            ega->status &= ~1;
        ega->hdisp_on = 0;

        ega->linepos = 0;
        if ((ega->scanline == (ega->crtc[11] & 31)) || (ega->scanline == ega->rowcount))
            ega->cursorvisible = 0;
        if (ega->dispon) {
            /* TODO: Verify real hardware behaviour for out-of-range fine vertical scroll */
            if (ega->linedbl && !ega->linecountff) {
                ega->linecountff = 1;
                ega->memaddr          = ega->memaddr_backup;
                ega->cca          = ega->memaddr_backup;
            }
            if (ega->scanline == (ega->crtc[9] & 31)) {
                ega->linecountff = 0;
                ega->scanline          = 0;

                ega->memaddr_backup += (ega->rowoffset << 3);
                if (ega->interlace)
                    ega->memaddr_backup += (ega->rowoffset << 3);
                ega->memaddr_backup &= ega->vrammask;
                ega->memaddr = ega->memaddr_backup;
                ega->cca = ega->memaddr_backup;
            } else {
                ega->linecountff = 0;
                ega->scanline++;
                ega->scanline &= 31;
                ega->memaddr = ega->memaddr_backup;
                ega->cca = ega->memaddr_backup;
            }
        }
        ega->real_vc++;
        if ((ega->actual_type != EGA_SUPEREGA) || !(ega->crtc[0xf9] & 0x02) ||
            !(ega->real_vc & 1))
            ega->vc++;
        if (ega->chipset) {
            if (ega->hdisp >= 800)
                ega->vc &= 1023;
            else
                ega->vc &= 511;
        } else
            ega->vc &= 511;
        if (ega->vc == ega->split) {
            // TODO: Implement the hardware bug where the first scanline is drawn twice when the split happens
            if (ega->interlace && ega->oddeven)
                ega->memaddr = ega->memaddr_backup = ega->rowoffset << 1;
            else
                ega->memaddr = ega->memaddr_backup = 0;
            ega->memaddr <<= 2;
            ega->cca = ega->memaddr;
            ega->memaddr_backup <<= 2;
            ega->scanline = 0;
        }
        if (ega->vc == ega->dispend) {
            ega->dispon = 0;
            uint32_t blink_delay = (ega->crtc[11] & 0x60) >> 5;
            if (ega->crtc[10] & 0x20)
                ega->cursoron = 0;
            else if (blink_delay == 2)
                ega->cursoron = ((ega->blink % 96) >= 48);
            else
                ega->cursoron = ega->blink & (16 + (16 * blink_delay));

            if (!(ega->gdcreg[6] & 1) && !(ega->blink & 15))
                ega->fullchange = 2;
            ega->blink = (ega->blink + 1) & 0x7f;

            if (ega->fullchange)
                ega->fullchange--;
        }
        if (ega->vc == ega->vsyncstart) {
            ega->dispon = 0;
            ega->status |= 8;
#if 0
            picint(1 << 2);
#endif
//            x = ega->hdisp;

            video_lightpen_vsync();
            if (ega->interlace && !ega->oddeven)
                ega->lastline++;
            if (ega->interlace && ega->oddeven)
                ega->firstline--;

            wx = ega->hdisp;

            if (ega->vres) {
                wy = (ega->lastline - ega->firstline) << 1;
                if ((ega->actual_type == EGA_SUPEREGA) && (ega->crtc[0xf9] & 0x02))
                    wy >>= 1;
                ega_doblit(wx, wy, ega);
            } else {
                wy = ega->lastline - ega->firstline;
                if ((ega->actual_type == EGA_SUPEREGA) && (ega->crtc[0xf9] & 0x02))
                    wy >>= 1;
                ega_doblit(wx, wy, ega);
            }

            frames++;

            ega->firstline = 2000;
            ega->lastline  = 0;

            ega->firstline_draw = 2000;
            ega->lastline_draw  = 0;

            ega->oddeven ^= 1;

            changeframecount = ega->interlace ? 3 : 2;
            ega->vslines     = 0;

            if (ega->interlace && ega->oddeven)
                ega->memaddr = ega->memaddr_backup = ega->memaddr_latch + (ega->rowoffset << 1);
            else
                ega->memaddr = ega->memaddr_backup = ega->memaddr_latch;
            ega->cursoraddr = (ega->crtc[0xe] << 8) | ega->crtc[0xf];

            ega->memaddr <<= 2;
            ega->memaddr_backup <<= 2;
            ega->cursoraddr <<= 2;
            ega->cca = ega->memaddr;
        }
        if (ega->vc == ega->vtotal) {
            ega->vc       = 0;
            ega->scanline       = (ega->crtc[0x8] & 0x1f);
            ega->dispon   = 1;
            ega->displine = (ega->interlace && ega->oddeven) ? 1 : 0;

            ega->scrollcache = (ega->attrregs[0x13] & 0x0f);
            if (ega->scrollcache >= 0x8)
                ega->scrollcache = 0;
            else
                ega->scrollcache++;

            if (ega->seqregs[1] & 8)
                ega->scrollcache <<= 1;

            ega->x_add = (overscan_x >> 1) - ega->scrollcache;

            ega->linecountff = 0;
        }
        if (ega->scanline == (ega->crtc[10] & 31))
            ega->cursorvisible = 1;
    }
}

void
ega_poll(void *priv)
{
    ega_t    *ega                = (ega_t *) priv;
    const int prev_monitor_index = monitor_index_global;

    /* This instance may be the secondary display; every blit and size macro
       used while rendering resolves through monitor_index_global. */
    monitor_index_global = ega->monitor_index;
    ega_poll_common(ega);
    monitor_index_global = prev_monitor_index;
}

void
ega_doblit(int wx, int wy, ega_t *ega)
{
    int       unscaled_overscan_y = ega->vres ? overscan_y >> 1 : overscan_y;
    int       y_add   = enable_overscan ? unscaled_overscan_y : 0;
    int       x_add   = enable_overscan ? overscan_x : 0;
    int       y_start = enable_overscan ? 0 : (unscaled_overscan_y >> 1);
    int       x_start = enable_overscan ? 0 : (overscan_x >> 1);
    int       bottom  = (unscaled_overscan_y >> 1);
    int       xs_temp;
    int       ys_temp;

    if (ega->vres) {
        y_add <<= 1;
        y_start <<= 1;
        bottom <<= 1;
    }

    if ((wx <= 0) || (wy <= 0))
        return;

    if (ega->vres)
        ega->y_add <<= 1;

    xs_temp = wx;
    ys_temp = wy + 1;
    if (ega->vres)
        ys_temp++;
    if (xs_temp < 64)
        xs_temp = 640;
    if (ys_temp < 32)
        ys_temp = 200;

    if ((ega->crtc[0x17] & 0x80) && ((xs_temp != xsize) || (ys_temp != ysize) || video_force_resize_get())) {
        /* Screen res has changed.. fix up, and let them know. */
        xsize = xs_temp;
        ysize = ys_temp;

        if ((xsize > 1984) || (ysize > 2016)) {
            /* 2048x2048 is the biggest safe render texture, to account for overscan,
               we suppress overscan starting from x 1984 and y 2016. */
            x_add             = 0;
            y_add             = 0;
            suppress_overscan = 1;
        } else
            suppress_overscan = 0;

        set_screen_size(xsize + x_add, ysize + y_add);

        if (video_force_resize_get())
            video_force_resize_set(0);
    }

    if ((wx >= 160) && ((wy + 1) >= 120)) {
        /* Draw (overscan_size - scroll size) lines of overscan on top and bottom. */
        for (int i = 0; i < ega->y_add; i++) {
            uint32_t *p = &buffer32->line[i & 0x7ff][0];

            for (int j = 0; j < (xsize + x_add); j++)
                p[j] = ega->overscan_color;

            if (ega_feature_needs_processing(ega))
                ega_feature_process_line(ega, i, xsize + x_add, 1);
        }

        for (int i = 0; i < bottom; i++) {
            uint32_t *p = &buffer32->line[(ysize + ega->y_add + i) & 0x7ff][0];

            for (int j = 0; j < (xsize + x_add); j++)
                p[j] = ega->overscan_color;

            if (ega_feature_needs_processing(ega))
                ega_feature_process_line(ega, ysize + ega->y_add + i, xsize + x_add, 1);
        }
    }

    /* A PAL television that can't hold the field rate rolls the picture. */
    if (ega->pal && (ega->feature_card == EGA_FEATURE_COLOR_COMP_PAL))
        ega_pal_roll((ega_pal_t *) ega->pal, y_start + ysize + y_add,
                     ega->vtotal << (ega->vres ? 1 : 0), xsize + x_add);

    video_blit_memtoscreen(x_start, y_start, xsize + x_add, ysize + y_add);

    if (ega->vres)
        ega->y_add >>= 1;
}

uint32_t
ega_remap_cpu_addr(uint32_t inaddr, ega_t *ega)
{
    int      a0mux;
    uint32_t addr = inaddr;

    // The CPU A0 line is multiplexed via a 3-to-8 mux.
    // Input bits are:
    // bit 0: 1 = 64K, 0 = 128K+ (from memory expansion connector)
    // bit 1: 1 = Odd/Even mode, 0 = normal mode (from GC reg 6 bit 1)
    // bit 2: 1 = 128K mapping, 0 = other mapping (from memory decode PROM)
    a0mux = 0;

    if (ega->gdcreg[6] & 2)
        a0mux |= 2;

    if (ega->vram_limit <= 64 * 1024)
        a0mux |= 1;

    switch (ega->gdcreg[6] & 0xC) {
        case 0x0: // 128K A000
            addr &= 0xFFFF;
            // TODO: Confirm the behaviour of this on actual hardware
            a0mux |= 4;
            break;
        case 0x4: // 64K A000
            addr &= 0xFFFF;
            break;
        case 0x8: // 32K B000
            addr &= 0x7FFF;
            break;
        case 0xC: // 32K B800
            addr &= 0x7FFF;
            break;

        default:
            break;
    }

    switch (a0mux) {
        case 0:
        case 1:
        case 4:
        case 5:
        case 7: // A0 becomes A0
            break;
        case 2:
            // A0 becomes the inversion of PGSEL (reg 0x3C2, miscout, bit 5)
            // That is, 1 selects the "low" 64k, and 0 selects the "high" 64k.
            addr &= ~1;
            addr |= (~ega->miscout >> 5) & 1;
            break;
        case 3: // A0 becomes A14
            addr &= ~1;
            addr |= (inaddr >> 14) & 1;
            break;
        case 6: // A0 becomes A16
            addr &= ~1;
            addr |= (inaddr >> 16) & 1;
            break;

        default:
            break;
    }

    // In 64k mode, only select the first 16Kword/64KB bank
    if (!(ega->seqregs[4] & 2)) {
        addr &= 0x3FFF;
    }
    return addr;
}

void
ega_write(uint32_t addr, uint8_t val, void *priv)
{
    ega_t  *ega = (ega_t *) priv;
    uint8_t vala;
    uint8_t valb;
    uint8_t valc;
    uint8_t vald;
    int     writemask2 = ega->writemask;

    cycles -= monitors[ega->monitor_index].mon_video_timing_write_b;

    if (ega->chain2_write)
        writemask2 &= 0x5 << (addr & 1);

    addr = ega_remap_cpu_addr(addr, ega);

    addr <<= 2;

    if (addr >= ega->vram_limit)
        return;

    if (!(ega->gdcreg[6] & 1))
        ega->fullchange = 2;

    switch (ega->writemode) {
        case 1:
            if (writemask2 & 1)
                ega->vram[addr] = ega->la;
            if (writemask2 & 2)
                ega->vram[addr | 0x1] = ega->lb;
            if (writemask2 & 4)
                ega->vram[addr | 0x2] = ega->lc;
            if (writemask2 & 8)
                ega->vram[addr | 0x3] = ega->ld;
            break;
        case 0:
            if (ega->gdcreg[3] & 7)
                val = ega_rotate[ega->gdcreg[3] & 7][val];

            if ((ega->gdcreg[8] == 0xff) && !(ega->gdcreg[3] & 0x18) && !ega->gdcreg[1]) {
                if (writemask2 & 1)
                    ega->vram[addr] = val;
                if (writemask2 & 2)
                    ega->vram[addr | 0x1] = val;
                if (writemask2 & 4)
                    ega->vram[addr | 0x2] = val;
                if (writemask2 & 8)
                    ega->vram[addr | 0x3] = val;
            } else {
                if (ega->gdcreg[1] & 1)
                    vala = (ega->gdcreg[0] & 1) ? 0xff : 0;
                else
                    vala = val;
                if (ega->gdcreg[1] & 2)
                    valb = (ega->gdcreg[0] & 2) ? 0xff : 0;
                else
                    valb = val;
                if (ega->gdcreg[1] & 4)
                    valc = (ega->gdcreg[0] & 4) ? 0xff : 0;
                else
                    valc = val;
                if (ega->gdcreg[1] & 8)
                    vald = (ega->gdcreg[0] & 8) ? 0xff : 0;
                else
                    vald = val;
                switch (ega->gdcreg[3] & 0x18) {
                    case 0: /*Set*/
                        if (writemask2 & 1)
                            ega->vram[addr] = (vala & ega->gdcreg[8]) | (ega->la & ~ega->gdcreg[8]);
                        if (writemask2 & 2)
                            ega->vram[addr | 0x1] = (valb & ega->gdcreg[8]) | (ega->lb & ~ega->gdcreg[8]);
                        if (writemask2 & 4)
                            ega->vram[addr | 0x2] = (valc & ega->gdcreg[8]) | (ega->lc & ~ega->gdcreg[8]);
                        if (writemask2 & 8)
                            ega->vram[addr | 0x3] = (vald & ega->gdcreg[8]) | (ega->ld & ~ega->gdcreg[8]);
                        break;
                    case 8: /*AND*/
                        if (writemask2 & 1)
                            ega->vram[addr] = (vala | ~ega->gdcreg[8]) & ega->la;
                        if (writemask2 & 2)
                            ega->vram[addr | 0x1] = (valb | ~ega->gdcreg[8]) & ega->lb;
                        if (writemask2 & 4)
                            ega->vram[addr | 0x2] = (valc | ~ega->gdcreg[8]) & ega->lc;
                        if (writemask2 & 8)
                            ega->vram[addr | 0x3] = (vald | ~ega->gdcreg[8]) & ega->ld;
                        break;
                    case 0x10: /*OR*/
                        if (writemask2 & 1)
                            ega->vram[addr] = (vala & ega->gdcreg[8]) | ega->la;
                        if (writemask2 & 2)
                            ega->vram[addr | 0x1] = (valb & ega->gdcreg[8]) | ega->lb;
                        if (writemask2 & 4)
                            ega->vram[addr | 0x2] = (valc & ega->gdcreg[8]) | ega->lc;
                        if (writemask2 & 8)
                            ega->vram[addr | 0x3] = (vald & ega->gdcreg[8]) | ega->ld;
                        break;
                    case 0x18: /*XOR*/
                        if (writemask2 & 1)
                            ega->vram[addr] = (vala & ega->gdcreg[8]) ^ ega->la;
                        if (writemask2 & 2)
                            ega->vram[addr | 0x1] = (valb & ega->gdcreg[8]) ^ ega->lb;
                        if (writemask2 & 4)
                            ega->vram[addr | 0x2] = (valc & ega->gdcreg[8]) ^ ega->lc;
                        if (writemask2 & 8)
                            ega->vram[addr | 0x3] = (vald & ega->gdcreg[8]) ^ ega->ld;
                        break;

                    default:
                        break;
                }
            }
            break;
        case 2:
            if (!(ega->gdcreg[3] & 0x18) && !ega->gdcreg[1]) {
                if (writemask2 & 1)
                    ega->vram[addr] = (((val & 1) ? 0xff : 0) & ega->gdcreg[8]) | (ega->la & ~ega->gdcreg[8]);
                if (writemask2 & 2)
                    ega->vram[addr | 0x1] = (((val & 2) ? 0xff : 0) & ega->gdcreg[8]) | (ega->lb & ~ega->gdcreg[8]);
                if (writemask2 & 4)
                    ega->vram[addr | 0x2] = (((val & 4) ? 0xff : 0) & ega->gdcreg[8]) | (ega->lc & ~ega->gdcreg[8]);
                if (writemask2 & 8)
                    ega->vram[addr | 0x3] = (((val & 8) ? 0xff : 0) & ega->gdcreg[8]) | (ega->ld & ~ega->gdcreg[8]);
            } else {
                vala = ((val & 1) ? 0xff : 0);
                valb = ((val & 2) ? 0xff : 0);
                valc = ((val & 4) ? 0xff : 0);
                vald = ((val & 8) ? 0xff : 0);
                switch (ega->gdcreg[3] & 0x18) {
                    case 0: /*Set*/
                        if (writemask2 & 1)
                            ega->vram[addr] = (vala & ega->gdcreg[8]) | (ega->la & ~ega->gdcreg[8]);
                        if (writemask2 & 2)
                            ega->vram[addr | 0x1] = (valb & ega->gdcreg[8]) | (ega->lb & ~ega->gdcreg[8]);
                        if (writemask2 & 4)
                            ega->vram[addr | 0x2] = (valc & ega->gdcreg[8]) | (ega->lc & ~ega->gdcreg[8]);
                        if (writemask2 & 8)
                            ega->vram[addr | 0x3] = (vald & ega->gdcreg[8]) | (ega->ld & ~ega->gdcreg[8]);
                        break;
                    case 8: /*AND*/
                        if (writemask2 & 1)
                            ega->vram[addr] = (vala | ~ega->gdcreg[8]) & ega->la;
                        if (writemask2 & 2)
                            ega->vram[addr | 0x1] = (valb | ~ega->gdcreg[8]) & ega->lb;
                        if (writemask2 & 4)
                            ega->vram[addr | 0x2] = (valc | ~ega->gdcreg[8]) & ega->lc;
                        if (writemask2 & 8)
                            ega->vram[addr | 0x3] = (vald | ~ega->gdcreg[8]) & ega->ld;
                        break;
                    case 0x10: /*OR*/
                        if (writemask2 & 1)
                            ega->vram[addr] = (vala & ega->gdcreg[8]) | ega->la;
                        if (writemask2 & 2)
                            ega->vram[addr | 0x1] = (valb & ega->gdcreg[8]) | ega->lb;
                        if (writemask2 & 4)
                            ega->vram[addr | 0x2] = (valc & ega->gdcreg[8]) | ega->lc;
                        if (writemask2 & 8)
                            ega->vram[addr | 0x3] = (vald & ega->gdcreg[8]) | ega->ld;
                        break;
                    case 0x18: /*XOR*/
                        if (writemask2 & 1)
                            ega->vram[addr] = (vala & ega->gdcreg[8]) ^ ega->la;
                        if (writemask2 & 2)
                            ega->vram[addr | 0x1] = (valb & ega->gdcreg[8]) ^ ega->lb;
                        if (writemask2 & 4)
                            ega->vram[addr | 0x2] = (valc & ega->gdcreg[8]) ^ ega->lc;
                        if (writemask2 & 8)
                            ega->vram[addr | 0x3] = (vald & ega->gdcreg[8]) ^ ega->ld;
                        break;

                    default:
                        break;
                }
            }
            break;

        default:
            break;
    }
}

/* For the memory viewer: the byte ega_read() returns in read mode 0, found
   without loading the latches or taking wait states. */
static uint8_t *
ega_debug_ptr(uint32_t addr, UNUSED(int write), void *priv)
{
    ega_t *ega       = (ega_t *) priv;
    int    readplane = ega->readplane;

    if (ega->chain2_read)
        readplane = (readplane & 2) | (addr & 1);

    addr = ega_remap_cpu_addr(addr, ega) << 2;
    if (addr >= ega->vram_limit)
        return NULL;
    if ((ega_type == EGA_TYPE_COMPAQ) && (ega->gdcreg[4] & 0x04))
        return NULL;

    return &ega->vram[addr | readplane];
}

uint8_t
ega_read(uint32_t addr, void *priv)
{
    ega_t  *ega = (ega_t *) priv;
    uint8_t temp;
    uint8_t temp2;
    uint8_t temp3;
    uint8_t temp4;
    int     readplane = ega->readplane;

    cycles -= monitors[ega->monitor_index].mon_video_timing_read_b;

    if (ega->chain2_read)
        readplane = (readplane & 2) | (addr & 1);

    addr = ega_remap_cpu_addr(addr, ega);

    addr <<= 2;

    if (addr >= ega->vram_limit)
        return 0xff;

    ega->la = ega->vram[addr];
    ega->lb = ega->vram[addr | 0x1];
    ega->lc = ega->vram[addr | 0x2];
    ega->ld = ega->vram[addr | 0x3];
    if (ega->readmode) {
        temp = ega->la;
        temp ^= (ega->colourcompare & 1) ? 0xff : 0;
        temp &= (ega->colournocare & 1) ? 0xff : 0;
        temp2 = ega->lb;
        temp2 ^= (ega->colourcompare & 2) ? 0xff : 0;
        temp2 &= (ega->colournocare & 2) ? 0xff : 0;
        temp3 = ega->lc;
        temp3 ^= (ega->colourcompare & 4) ? 0xff : 0;
        temp3 &= (ega->colournocare & 4) ? 0xff : 0;
        temp4 = ega->ld;
        temp4 ^= (ega->colourcompare & 8) ? 0xff : 0;
        temp4 &= (ega->colournocare & 8) ? 0xff : 0;
        return ~(temp | temp2 | temp3 | temp4);
    }

    if ((ega_type == EGA_TYPE_COMPAQ) && (ega->gdcreg[4] & 0x04))
        return 0xff;

    return ega->vram[addr | readplane];
}

void
ega_init(ega_t *ega, int monitor_type, int is_mono)
{
    /* For the memory viewer; here, as machines with the card built in map
       the same handlers. */
    mem_debug_add(ega_read, ega_write, ega_debug_ptr);

    ega->monitor_index = monitor_index_global;

    ega->vram     = calloc(1, 0x40000);
    ega->vrammask = 0x3ffff;

    for (uint16_t c = 0; c < 256; c++) {
        int e = c;
        for (uint8_t d = 0; d < 8; d++) {
            ega_rotate[d][c] = e;
            e                = (e >> 1) | ((e & 1) ? 0x80 : 0);
        }
    }

    if (is_mono) {
        for (uint16_t c = 0; c < 256; c++) {
            if (((c >> 3) & 3) == 0)
                pallook64[c] = pallook16[c] = makecol32(0, 0, 0);
            else
                switch (monitor_type >> 4) {
                    case DISPLAY_GREEN:
                        switch ((c >> 3) & 3) {
                            case 1:
                                pallook64[c] = pallook16[c] = makecol32(0x08, 0xc7, 0x2c);
                                break;
                            case 2:
                                pallook64[c] = pallook16[c] = makecol32(0x04, 0x8a, 0x20);
                                break;
                            case 3:
                                pallook64[c] = pallook16[c] = makecol32(0x34, 0xff, 0x5d);
                                break;

                            default:
                                break;
                        }
                        break;
                    case DISPLAY_AMBER:
                        switch ((c >> 3) & 3) {
                            case 1:
                                pallook64[c] = pallook16[c] = makecol32(0xef, 0x79, 0x00);
                                break;
                            case 2:
                                pallook64[c] = pallook16[c] = makecol32(0xb2, 0x4d, 0x00);
                                break;
                            case 3:
                                pallook64[c] = pallook16[c] = makecol32(0xff, 0xe3, 0x34);
                                break;

                            default:
                                break;
                        }
                        break;
                    case DISPLAY_WHITE:
                    default:
                        switch ((c >> 3) & 3) {
                            case 1:
                                pallook64[c] = pallook16[c] = makecol32(0xaf, 0xb3, 0xb0);
                                break;
                            case 2:
                                pallook64[c] = pallook16[c] = makecol32(0x7a, 0x81, 0x83);
                                break;
                            case 3:
                                pallook64[c] = pallook16[c] = makecol32(0xff, 0xfd, 0xed);
                                break;

                            default:
                                break;
                        }
                        break;
                }
        }
    } else {
        for (uint16_t c = 0; c < 256; c++) {
            pallook64[c] = makecol32(((c >> 2) & 1) * 0xaa, ((c >> 1) & 1) * 0xaa, (c & 1) * 0xaa);
            pallook64[c] += makecol32(((c >> 5) & 1) * 0x55, ((c >> 4) & 1) * 0x55, ((c >> 3) & 1) * 0x55);
            pallook16[c] = makecol32(((c >> 2) & 1) * 0xaa, ((c >> 1) & 1) * 0xaa, (c & 1) * 0xaa);
            pallook16[c] += makecol32(((c >> 4) & 1) * 0x55, ((c >> 4) & 1) * 0x55, ((c >> 4) & 1) * 0x55);
            if ((c & 0x17) == 6)
                pallook16[c] = makecol32(0xaa, 0x55, 0);
        }
    }

    ega->pallook = pallook16;

    for (uint16_t c = 0; c < 256; c++) {
        ega->mda_attr_to_color_table[c][0][0] = ega->mda_attr_to_color_table[c][1][0] = ega->mda_attr_to_color_table[c][1][1] = 16;
        if (c & 8)
            ega->mda_attr_to_color_table[c][0][1] = 15 + 16;
        else
            ega->mda_attr_to_color_table[c][0][1] = 7 + 16;
    }
    ega->mda_attr_to_color_table[0x70][0][1] = 16;
    ega->mda_attr_to_color_table[0x70][0][0] = ega->mda_attr_to_color_table[0x70][1][0] = ega->mda_attr_to_color_table[0x70][1][1] = 16 + 15;
    ega->mda_attr_to_color_table[0xF0][0][1]                                                       = 16;
    ega->mda_attr_to_color_table[0xF0][0][0] = ega->mda_attr_to_color_table[0xF0][1][0] = ega->mda_attr_to_color_table[0xF0][1][1] = 16 + 15;
    ega->mda_attr_to_color_table[0x78][0][1]                                                       = 16 + 7;
    ega->mda_attr_to_color_table[0x78][0][0] = ega->mda_attr_to_color_table[0x78][1][0] = ega->mda_attr_to_color_table[0x78][1][1] = 16 + 15;
    ega->mda_attr_to_color_table[0xF8][0][1]                                                       = 16 + 7;
    ega->mda_attr_to_color_table[0xF8][0][0] = ega->mda_attr_to_color_table[0xF8][1][0] = ega->mda_attr_to_color_table[0xF8][1][1] = 16 + 15;
    ega->mda_attr_to_color_table[0x00][0][1] = ega->mda_attr_to_color_table[0x00][1][1] = 16;
    ega->mda_attr_to_color_table[0x08][0][1] = ega->mda_attr_to_color_table[0x08][1][1] = 16;
    ega->mda_attr_to_color_table[0x80][0][1] = ega->mda_attr_to_color_table[0x80][1][1] = 16;
    ega->mda_attr_to_color_table[0x88][0][1] = ega->mda_attr_to_color_table[0x88][1][1] = 16;

    egaswitches = monitor_type & 0xf;

    ega->vram_limit = 256 * 1024;
    ega->vrammask   = ega->vram_limit - 1;

    /* Color Plane Enable defaults to all four planes enabled. */
    ega->attrregs[0x12] = 0x0f;
    ega->plane_mask     = 0x0f;

    ega->miscout |= 0x22;
    ega->oddeven_page = 0;

    ega->seqregs[4] |= 2;
    ega->extvram = 1;

    update_overscan = 0;

    ega->crtc[0] = 63;
    ega->crtc[6] = 255;

    ega->render_override = NULL;

    timer_add(&ega->timer, ega_poll, ega, 1);
    if (ega_type == EGA_TYPE_COMPAQ)
        timer_add(&ega->dot_timer, ega_dot_poll, ega, 1);
}

void
ega_set_type(void *priv, uint32_t local)
{
    ega_t *ega = (ega_t *) priv;

    if ((local == EGA_IBM) || (local == EGA_ISKRA) || (local == EGA_TSENG))
        ega_type = EGA_TYPE_IBM;
    else if (local == EGA_COMPAQ)
        ega_type = EGA_TYPE_COMPAQ;
    else
        ega_type = EGA_TYPE_OTHER;

    ega->actual_type = local;
    ega->chipset = 0;

    switch (local) {
        default:
            break;
        case EGA_ATI800P:
            ega->chipset = 1;
            break;
    }
}

/* Sets up the card on the IBM EGA's feature connector. Both composite cards take
   the R, G, B and I (secondary green in 350-line modes) pins off the connector,
   so the palettes are rebuilt to what they produce from those four lines. */
static void
ega_feature_init(ega_t *ega, int card)
{
    /* Brightest level of the mono tints, as in ega_init(). */
    static const uint32_t tint[3] = { 0xfffded, 0x34ff5d, 0xffe334 };

    ega->feature_card = card;
    ega->feat_ctrl    = 0;
    ega->feat_in      = 0; /* None of the supported cards drives FEAT0/FEAT1. */
    ega->ext_clock    = 0.0;

    switch (card) {
        case EGA_FEATURE_132COL:
            ega->ext_clock = EGA_FEATURE_132COL_CLOCK;
            break;

        case EGA_FEATURE_MONO_COMP_W:
        case EGA_FEATURE_MONO_COMP_G:
        case EGA_FEATURE_MONO_COMP_A: {
            const uint32_t t = tint[card - EGA_FEATURE_MONO_COMP_W];

            for (uint16_t c = 0; c < 256; c++) {
                /* RGBI to luminance through the card's resistor network: each
                   active line adds its conductance (see EGA_MONO_COMP_R_*). With
                   Holden's values intensity outweighs R+G+B together, giving 16
                   distinct grey levels, every bright colour above every normal one. */
                const double gr = 1.0 / EGA_MONO_COMP_R_RED;
                const double gg = 1.0 / EGA_MONO_COMP_R_GREEN;
                const double gb = 1.0 / EGA_MONO_COMP_R_BLUE;
                const double gi = 1.0 / EGA_MONO_COMP_R_INTENSITY;
                const double y  = (((c & 0x04) ? gr : 0.0) + ((c & 0x02) ? gg : 0.0) +
                                   ((c & 0x01) ? gb : 0.0) + ((c & 0x10) ? gi : 0.0)) /
                                  (gr + gg + gb + gi);

                pallook16[c] = pallook64[c] = makecol32((int) (((t >> 16) & 0xff) * y),
                                                        (int) (((t >> 8) & 0xff) * y),
                                                        (int) ((t & 0xff) * y));
            }
            break;
        }

        case EGA_FEATURE_COLOR_COMP_PAL:
            /* As for NTSC below, the palette output becomes the RGBI index, which
               ega_pal_process() encodes and decodes as PAL. */
            for (uint16_t c = 0; c < 256; c++)
                pallook16[c] = pallook64[c] = ((c >> 1) & 0x08) | (c & 0x07);

            ega->pal = ega_pal_init(device_get_config_int("pal_lock"), device_get_config_int("pal_luma"));
            break;

        case EGA_FEATURE_COLOR_COMP:
            /* The palette output becomes the raw RGBI index, which
               ega_feature_process_line() encodes and decodes as NTSC. */
            for (uint16_t c = 0; c < 256; c++)
                pallook16[c] = pallook64[c] = ((c >> 1) & 0x08) | (c & 0x07);

            /* The "new" CGA model mixes R, G and B into luminance, as a
               resistor-network RGBI encoder does. */
            cga_comp_init(1);
            update_cga16_color(EGA_COMP_CGAMODE, 0x00);
            break;

        default:
            break;
    }
}

/* Class of the adapter sharing the machine with this EGA, if it is one of the
   two the IBM EGA's switches know about, VIDEO_FLAG_TYPE_NONE otherwise. */
static int
ega_other_adapter(int monitor_index)
{
    int flags;

    if (monitor_index != 0)
        return video_get_primary_flags(machine, gfxcard[0]);

    for (uint8_t i = 1; i < GFXCARD_MAX; i++) {
        if ((gfxcard[i] == VID_NONE) || (gfxcard[i] == VID_INTERNAL) || (gfxcard[i] == VID_EXTERNAL))
            continue;

        flags = video_card_get_flags(gfxcard[i]);
        if ((flags == VIDEO_FLAG_TYPE_MDA) || (flags == VIDEO_FLAG_TYPE_CGA))
            return flags;
    }

    return VIDEO_FLAG_TYPE_NONE;
}

/* Works out the configuration switch setting the IBM EGA would have to be given
   for the monitor attached to it and the adapter, if any, sharing the machine
   (see the switch table above ega_ibm_config[]). */
static int
ega_auto_switches(int monitor_sw, int other, int is_secondary)
{
    const int mono = EGA_SW_IS_MONO(monitor_sw);

    if (other == VIDEO_FLAG_TYPE_MDA) {
        /* Two monochrome adapters cannot coexist. */
        if (mono) {
            ega_log("EGA: an MDA is installed, so the EGA must drive a colour monitor; "
                    "assuming an 80x25 colour display\n");
            monitor_sw = EGA_SW_CD80;
        }

        /* EGA primary with an MDA uses the same settings as the EGA alone; with the
           MDA primary, the colour settings move down to 0-3. */
        return is_secondary ? (monitor_sw - EGA_SW_CD40) : monitor_sw;
    }

    if (other == VIDEO_FLAG_TYPE_CGA) {
        /* Two colour adapters cannot coexist. */
        if (!mono) {
            ega_log("EGA: a CGA is installed, so the EGA must drive a monochrome monitor; "
                    "assuming a monochrome display\n");
        }

        /* The CGA is assumed to come up in 80x25; the 40x25 variants are available
           through the explicit switch settings. */
        return is_secondary ? EGA_SW_CGA80_PRI_MONO : EGA_SW_MONO;
    }

    return monitor_sw;
}

static void *
ega_standalone_init(const device_t *info)
{
    ega_t *ega = calloc(1, sizeof(ega_t));
    int    monitor_type;

    video_inform(VIDEO_FLAG_TYPE_SPECIAL, &timing_ega);

    overscan_x = 16;
    overscan_y = 28;
    ega->x_add = 8;
    ega->y_add = 14;

    if ((info->local == EGA_IBM) || (info->local == EGA_ISKRA) || (info->local == EGA_TSENG))
        ega_type = EGA_TYPE_IBM;
    else if (info->local == EGA_COMPAQ)
        ega_type = EGA_TYPE_COMPAQ;
    else
        ega_type = EGA_TYPE_OTHER;

    ega->actual_type = info->local;
    ega->chipset = 0;

    switch (info->local) {
        default:
        case EGA_IBM:
            rom_init(&ega->bios_rom, BIOS_IBM_PATH,
                     0xc0000, 0x8000, 0x7fff, 0, MEM_MAPPING_EXTERNAL);
            break;
        case EGA_COMPAQ:
            ega->ctl_mode = 0x21;
            rom_init(&ega->bios_rom, BIOS_CPQ_PATH,
                     0xc0000, 0x8000, 0x7fff, 0, MEM_MAPPING_EXTERNAL);
            break;
        case EGA_SUPEREGA:
            rom_init(&ega->bios_rom, BIOS_SEGA_PATH,
                     0xc0000, 0x8000, 0x7fff, 0, MEM_MAPPING_EXTERNAL);
            break;
        case EGA_ATI800P:
            rom_init(&ega->bios_rom, BIOS_ATIEGA800P_PATH,
                     0xc0000, 0x8000, 0x7fff, 0, MEM_MAPPING_EXTERNAL);
            ega->chipset = 1;
            break;
        case EGA_ISKRA:
            rom_init_interleaved(&ega->bios_rom, BIOS_ISKRA_PATH,
                                 0xc0000, 0x8000, 0x7fff, 0, MEM_MAPPING_EXTERNAL);
            break;
        case EGA_TSENG:
            rom_init(&ega->bios_rom, BIOS_TSENG_PATH,
                     0xc0000, 0x8000, 0x7fff, 0, MEM_MAPPING_EXTERNAL);
            break;
    }

    if ((ega->bios_rom.rom[0x3ffe] == 0xaa) && (ega->bios_rom.rom[0x3fff] == 0x55)) {
        for (uint16_t c = 0; c < 0x2000; c++) {
            uint8_t temp                  = ega->bios_rom.rom[c];
            ega->bios_rom.rom[c]          = ega->bios_rom.rom[0x3fff - c];
            ega->bios_rom.rom[0x3fff - c] = temp;
        }
    }

    monitor_type = ega->chipset ? 0x09 : device_get_config_int("monitor_type");

    /* Only the IBM EGA has the switches to share the machine with an MDA or a CGA. */
    int other = VIDEO_FLAG_TYPE_NONE;
    if (info->local == EGA_IBM) {
        int switches = device_get_config_int("switches");

        other = ega_other_adapter(monitor_index_global);

        if (switches == EGA_SW_AUTO)
            switches = ega_auto_switches(monitor_type & 0x0f, other, monitor_index_global != 0);

        ega_log("EGA: monitor %i, other adapter class %i, switches %X\n",
                monitor_index_global, other, switches);

        monitor_type = (monitor_type & 0xf0) | (switches & 0x0f);
    }

    ega_init(ega, monitor_type, EGA_SW_IS_MONO(monitor_type & 0x0f));

    if (info->local == EGA_IBM)
        ega_feature_init(ega, device_get_config_int("feature_card"));

    ega->vram_limit = device_get_config_int("memory") * 1024;
    ega->vrammask   = ega->vram_limit - 1;

    /* With an MDA in the machine, the 3Bx ports and the B0000 window are its own:
       start the EGA out decoding colour addresses only and in its 64 KB window, so
       that it does not answer the MDA's port and memory accesses before its BIOS
       has set it up. */
    ega->mono_io_disabled = (other == VIDEO_FLAG_TYPE_MDA);
    if (ega->mono_io_disabled)
        ega->miscout |= 0x01;

    mem_mapping_add(&ega->mapping, 0xa0000, ega->mono_io_disabled ? 0x10000 : 0x20000,
                    ega_read, NULL, NULL, ega_write, NULL, NULL, NULL, MEM_MAPPING_EXTERNAL, ega);
    if (ega_type == EGA_TYPE_COMPAQ)
        mem_mapping_disable(&ega->mapping);
    uint16_t addr = 0x03a0;
#ifdef EGA_ALT_ADDR_SUPPORT
    if (ega_type == EGA_TYPE_IBM) {
        addr = device_get_config_hex16("base") - 0x0020;
        if (addr == 0x02a0)
            ega->alt_addr = 1;
    }
#endif
    if (ega->mono_io_disabled)
        io_sethandler(addr + 0x0020, 0x0020, ega_in, NULL, NULL, ega_out, NULL, NULL, ega);
    else
        io_sethandler(addr, 0x0040, ega_in, NULL, NULL, ega_out, NULL, NULL, ega);

    if (ega->chipset) {
        io_sethandler(0x01ce, 0x0002, ega_in, NULL, NULL, ega_out, NULL, NULL, ega);
        ega->eeprom = calloc(1, sizeof(ati_eeprom_t));
        ati_eeprom_load((ati_eeprom_t *) ega->eeprom, "egawonder800p.nvr", 0);
    } else if (info->local == EGA_COMPAQ) {
        io_sethandler(0x0084, 0x0001, ega_in, NULL, NULL, ega_out, NULL, NULL, ega);
        io_sethandler(0x07c6, 0x0001, ega_in, NULL, NULL, ega_out, NULL, NULL, ega);
        io_sethandler(0x0bc6, 0x0001, ega_in, NULL, NULL, ega_out, NULL, NULL, ega);
        io_sethandler(0x0fc6, 0x0001, ega_in, NULL, NULL, ega_out, NULL, NULL, ega);
    }

    return ega;
}

static int
ega_standalone_available(void)
{
    return rom_present(BIOS_IBM_PATH);
}

static int
cpqega_standalone_available(void)
{
    return rom_present(BIOS_CPQ_PATH);
}

static int
sega_standalone_available(void)
{
    return rom_present(BIOS_SEGA_PATH);
}

static int
atiega800p_standalone_available(void)
{
    return rom_present(BIOS_ATIEGA800P_PATH);
}

static int
iskra_ega_standalone_available(void)
{
    return rom_present("roms/video/ega/143-02.bin") && rom_present("roms/video/ega/143-03.bin");
}

static int
et2000_standalone_available(void)
{
    return rom_present(BIOS_TSENG_PATH);
}

static void
ega_close(void *priv)
{
    ega_t *ega = (ega_t *) priv;

    if (ega->eeprom)
        free(ega->eeprom);
    if (ega->pal)
        ega_pal_close((ega_pal_t *) ega->pal);
    free(ega->vram);
    free(ega);
}

static void
ega_speed_changed(void *priv)
{
    ega_t *ega = (ega_t *) priv;

    ega_recalctimings(ega);
}

/* IBM EGA configuration switches (from the IBM EGA installation instructions):

   SW1 SW2 SW3 SW4  EGA monitor                       Other adapter           Value
   --- EGA alone, or EGA primary with an MDA (or a CGA for 0x0B) ---------------------
    ON OFF OFF  ON  Color 40x25                (5153) MDA, secondary   0110 0x06
   OFF OFF OFF  ON  Color 80x25                (5153) MDA, secondary   0111 0x07
    ON  ON  ON OFF  Enhanced Color - Normal    (5154) MDA, secondary   1000 0x08
   OFF  ON  ON OFF  Enhanced Color - Enhanced  (5154) MDA, secondary   1001 0x09
   OFF OFF  ON OFF  Monochrome                 (5151) CGA 80x25, sec.  1011 0x0B
    ON OFF  ON OFF  Monochrome                 (5151) CGA 40x25, sec.  1010 0x0A
   --- EGA secondary ------------------------------------------------------------------
    ON  ON  ON  ON  Color 40x25                (5153) MDA, primary     0000 0x00
   OFF  ON  ON  ON  Color 80x25                (5153) MDA, primary     0001 0x01
    ON OFF  ON  ON  Enhanced Color - Normal    (5154) MDA, primary     0010 0x02
   OFF OFF  ON  ON  Enhanced Color - Enhanced  (5154) MDA, primary     0011 0x03
    ON  ON OFF  ON  Monochrome                 (5151) CGA 40x25, pri.  0100 0x04
   OFF  ON OFF  ON  Monochrome                 (5151) CGA 80x25, pri.  0101 0x05

   0 = Switch closed (ON);
   1 = Switch open   (OFF).

   The "Monitor type" option holds the 0x06-0x0B (EGA alone) value plus the mono
   tint; with "Configuration switches" left on automatic, it is translated to the
   right row above from the slot the card is in and the adapter beside it. */
static const device_config_t ega_ibm_config[] = {
  // clang-format off
    {
        .name           = "memory",
        .description    = "Memory size",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 256,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description =  "32 KB", .value =  32 },
            { .description =  "64 KB", .value =  64 },
            { .description = "128 KB", .value = 128 },
            { .description = "256 KB", .value = 256 },
            { .description = ""                     }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "monitor_type",
        .description    = "Monitor type",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 9,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Monochrome (5151/MDA) (white)",             .value = 0x0B | (DISPLAY_WHITE << 4) },
            { .description = "Monochrome (5151/MDA) (green)",             .value = 0x0B | (DISPLAY_GREEN << 4) },
            { .description = "Monochrome (5151/MDA) (amber)",             .value = 0x0B | (DISPLAY_AMBER << 4) },
            { .description = "Color 40x25 (5153/CGA)",                    .value = 0x06                        },
            { .description = "Color 80x25 (5153/CGA)",                    .value = 0x07                        },
            { .description = "Enhanced Color - Normal Mode (5154/ECD)",   .value = 0x08                        },
            { .description = "Enhanced Color - Enhanced Mode (5154/ECD)", .value = 0x09                        },
            { .description = ""                                                                                }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "switches",
        .description    = "Configuration switches",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = EGA_SW_AUTO,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Automatic (from monitor and other adapter)",                              .value = EGA_SW_AUTO           },
            { .description = "SW1-4 On On On On: Secondary to MDA, Color 40x25 (5153)",                 .value = EGA_SW_MDA_PRI_CD40   },
            { .description = "SW1-4 Off On On On: Secondary to MDA, Color 80x25 (5153)",                .value = EGA_SW_MDA_PRI_CD80   },
            { .description = "SW1-4 On Off On On: Secondary to MDA, Enhanced Color - Normal (5154)",    .value = EGA_SW_MDA_PRI_ECD_N  },
            { .description = "SW1-4 Off Off On On: Secondary to MDA, Enhanced Color - Enhanced (5154)", .value = EGA_SW_MDA_PRI_ECD_E  },
            { .description = "SW1-4 On On Off On: Secondary to CGA 40x25, Monochrome (5151)",           .value = EGA_SW_CGA40_PRI_MONO },
            { .description = "SW1-4 Off On Off On: Secondary to CGA 80x25, Monochrome (5151)",          .value = EGA_SW_CGA80_PRI_MONO },
            { .description = "SW1-4 On Off Off On: Primary, Color 40x25 (5153)",                        .value = EGA_SW_CD40           },
            { .description = "SW1-4 Off Off Off On: Primary, Color 80x25 (5153)",                       .value = EGA_SW_CD80           },
            { .description = "SW1-4 On On On Off: Primary, Enhanced Color - Normal (5154)",             .value = EGA_SW_ECD_NORMAL     },
            { .description = "SW1-4 Off On On Off: Primary, Enhanced Color - Enhanced (5154)",          .value = EGA_SW_ECD_ENHANCED   },
            { .description = "SW1-4 On Off On Off: Primary, Monochrome (5151), CGA 40x25 secondary",    .value = EGA_SW_MONO_CGA40     },
            { .description = "SW1-4 Off Off On Off: Primary, Monochrome (5151)",                        .value = EGA_SW_MONO           },
            { .description = ""                                                                                        }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "feature_card",
        .description    = "Feature connector",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = EGA_FEATURE_NONE,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "None",                               .value = EGA_FEATURE_NONE        },
            { .description = "132-column clock card",              .value = EGA_FEATURE_132COL      },
            { .description = "Monochrome composite (white)",       .value = EGA_FEATURE_MONO_COMP_W },
            { .description = "Monochrome composite (green)",       .value = EGA_FEATURE_MONO_COMP_G },
            { .description = "Monochrome composite (amber)",       .value = EGA_FEATURE_MONO_COMP_A },
            { .description = "Color composite (NTSC)",             .value = EGA_FEATURE_COLOR_COMP     },
            { .description = "Color composite (PAL)",              .value = EGA_FEATURE_COLOR_COMP_PAL },
            { .description = ""                                                                     }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "pal_lock",
        .description    = "PAL TV sync",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = EGA_PAL_LOCK_PAL60,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Ideal (always locks)",          .value = EGA_PAL_LOCK_IDEAL },
            { .description = "PAL-60 capable TV",             .value = EGA_PAL_LOCK_PAL60 },
            { .description = "50 Hz-only TV (60 Hz rolls)",   .value = EGA_PAL_LOCK_50HZ  },
            { .description = ""                                                          }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "pal_luma",
        .description    = "PAL TV luma separation",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = EGA_PAL_LUMA_TRAP,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Chroma subtraction (sharp)",    .value = EGA_PAL_LUMA_COMB },
            { .description = "4.43 MHz luma trap (soft)",     .value = EGA_PAL_LUMA_TRAP },
            { .description = ""                                                         }
        },
        .bios           = { { 0 } }
    },
#ifdef EGA_ALT_ADDR_SUPPORT
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x03a0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "0x3C0", .value = 0x03a0 },
            { .description = "0x2C0", .value = 0x02a0 },
            { .description = ""                       }
        },
        .bios           = { { 0 } }
    },
#endif
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_config_t ega_config[] = {
  // clang-format off
    {
        .name           = "memory",
        .description    = "Memory size",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 256,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description =  "32 KB", .value =  32 },
            { .description =  "64 KB", .value =  64 },
            { .description = "128 KB", .value = 128 },
            { .description = "256 KB", .value = 256 },
            { .description = ""                     }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "monitor_type",
        .description    = "Monitor type",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 9,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Monochrome (5151/MDA) (white)",             .value = 0x0B | (DISPLAY_WHITE << 4) },
            { .description = "Monochrome (5151/MDA) (green)",             .value = 0x0B | (DISPLAY_GREEN << 4) },
            { .description = "Monochrome (5151/MDA) (amber)",             .value = 0x0B | (DISPLAY_AMBER << 4) },
            { .description = "Color 40x25 (5153/CGA)",                    .value = 0x06                        },
            { .description = "Color 80x25 (5153/CGA)",                    .value = 0x07                        },
            { .description = "Enhanced Color - Normal Mode (5154/ECD)",   .value = 0x08                        },
            { .description = "Enhanced Color - Enhanced Mode (5154/ECD)", .value = 0x09                        },
            { .description = ""                                                                                }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_config_t atiega800p_config[] = {
  // clang-format off
    {
        .name           = "memory",
        .description    = "Memory size",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 256,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description =  "32 KB", .value =  32 },
            { .description =  "64 KB", .value =  64 },
            { .description = "128 KB", .value = 128 },
            { .description = "256 KB", .value = 256 },
            { .description = ""                     }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

const device_t ega_device = {
    .name          = "IBM EGA",
    .internal_name = "ega",
    .flags         = DEVICE_ISA,
    .local         = EGA_IBM,
    .init          = ega_standalone_init,
    .close         = ega_close,
    .reset         = NULL,
    .available     = ega_standalone_available,
    .speed_changed = ega_speed_changed,
    .force_redraw  = NULL,
    .config        = ega_ibm_config
};

const device_t cpqega_device = {
    .name          = "Compaq EGA",
    .internal_name = "compaq_ega",
    .flags         = DEVICE_ISA,
    .local         = EGA_COMPAQ,
    .init          = ega_standalone_init,
    .close         = ega_close,
    .reset         = NULL,
    .available     = cpqega_standalone_available,
    .speed_changed = ega_speed_changed,
    .force_redraw  = NULL,
    .config        = ega_config
};

const device_t sega_device = {
    .name          = "Chips & Technologies SuperEGA",
    .internal_name = "superega",
    .flags         = DEVICE_ISA,
    .local         = EGA_SUPEREGA,
    .init          = ega_standalone_init,
    .close         = ega_close,
    .reset         = NULL,
    .available     = sega_standalone_available,
    .speed_changed = ega_speed_changed,
    .force_redraw  = NULL,
    .config        = ega_config
};

const device_t atiega800p_device = {
    .name          = "ATI EGA Wonder 800+",
    .internal_name = "egawonder800p",
    .flags         = DEVICE_ISA,
    .local         = EGA_ATI800P,
    .init          = ega_standalone_init,
    .close         = ega_close,
    .reset         = NULL,
    .available     = atiega800p_standalone_available,
    .speed_changed = ega_speed_changed,
    .force_redraw  = NULL,
    .config        = atiega800p_config
};

const device_t iskra_ega_device = {
    .name          = "Iskra EGA (Cyrillic ROM)",
    .internal_name = "iskra_ega",
    .flags         = DEVICE_ISA,
    .local         = EGA_ISKRA,
    .init          = ega_standalone_init,
    .close         = ega_close,
    .reset         = NULL,
    .available     = iskra_ega_standalone_available,
    .speed_changed = ega_speed_changed,
    .force_redraw  = NULL,
    .config        = ega_config
};

const device_t et2000_device = {
    .name          = "Tseng Labs ET2000",
    .internal_name = "et2000",
    .flags         = DEVICE_ISA,
    .local         = EGA_TSENG,
    .init          = ega_standalone_init,
    .close         = ega_close,
    .reset         = NULL,
    .available     = et2000_standalone_available,
    .speed_changed = ega_speed_changed,
    .force_redraw  = NULL,
    .config        = ega_config
};
