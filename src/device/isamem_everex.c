/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Emulation of the Everex EV-158A and EV-178 ISA memory boards.
 *
 *          EV-158A: up to 8 MB in 2 MB banks. Each 2 MB of EMS has its own
 *          gen-1 (Intel Above Board compatible) register set at consecutive
 *          I/O addresses from the table 208/218/258/268/2A8/2B8/2E8h.
 *
 *          EV-178: Everex "AMZ" enhanced register set - 64 mapping registers,
 *          one per 16 KB of the first megabyte, 10-bit page numbers (16 MB),
 *          with switch and configuration readbacks.
 *
 *          Register layouts are from the Everex EMM.SYS 4.01-4.05 drivers,
 *          see temporary/Everex_EMM_Spec.md. Both run Everex's own drivers.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2026 Jasmine Iwanek.
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
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/log.h>
#include <86box/isamem.h>
#include <86box/plat_unused.h>
#include "cpu.h"

#define EV_PG          0x4000u /* 16 KB */
#define EV158_SETS     4       /* gen-1 register sets (2 MB each) */
#define EV178_WINDOWS  64      /* 16 KB windows over 0-1 MB */
#define EV178_CONV_WIN 40      /* windows 0-27h = 0-640 KB */

#ifdef ENABLE_ISAMEM_EVEREX_LOG
int isamem_everex_do_log = ENABLE_ISAMEM_EVEREX_LOG;

static void
everex_log(void *priv, const char *fmt, ...)
{
    va_list ap;

    if (isamem_everex_do_log) {
        va_start(ap, fmt);
        log_out(priv, fmt, ap);
        va_end(ap);
    }
}
#else
#    define everex_log(priv, fmt, ...)
#endif

static const uint16_t ev_ports[7] = { 0x208, 0x218, 0x258, 0x268, 0x2a8, 0x2b8, 0x2e8 };

/* One 16 KB window, or a linear block (conventional fill / extended). */
typedef struct ev_win_t {
    uint32_t      addr;
    uint8_t      *ptr;    /* NULL = not decoded */
    uint32_t      size;
    int           active;
    mem_mapping_t mapping;
} ev_win_t;

typedef struct ev158_set_t {
    uint16_t base;
    uint8_t  reg[4];   /* raw mapping register, bit 7 enable, bits 6:0 page */
    uint8_t  latch;    /* frame latch bits 0-2 */
    uint32_t frame;
    ev_win_t win[4];
} ev158_set_t;

typedef struct everex_t {
    int       type;
    int       wide;
    uint8_t  *ram;
    uint32_t  ram_size;
    uint32_t  pages;

    ev_win_t  conv;
    ev_win_t  ext;

    /* EV-158A */
    int         nsets;
    ev158_set_t set[EV158_SETS];

    /* EV-178 */
    uint16_t  base;
    uint8_t   id_state;
    uint8_t   index;
    uint16_t  map[EV178_WINDOWS];
    ev_win_t  win[EV178_WINDOWS];
    uint8_t   sw8200;
    uint8_t   sw8201;
    uint8_t   cfgc200;

    void     *log;
} everex_t;

enum {
    EV_158A = 0,
    EV_178  = 1
};

/* ---------------------------------------------------------------- */
/* Memory                                                           */
/* ---------------------------------------------------------------- */
static uint8_t
ev_readb(uint32_t addr, void *priv)
{
    const ev_win_t *w = (ev_win_t *) priv;

    return w->ptr ? w->ptr[addr - w->addr] : 0xff;
}

static uint16_t
ev_readw(uint32_t addr, void *priv)
{
    const ev_win_t *w = (ev_win_t *) priv;

    return w->ptr ? *(uint16_t *) &w->ptr[addr - w->addr] : 0xffff;
}

static void
ev_writeb(uint32_t addr, uint8_t val, void *priv)
{
    const ev_win_t *w = (ev_win_t *) priv;

    if (w->ptr)
        w->ptr[addr - w->addr] = val;
}

static void
ev_writew(uint32_t addr, uint16_t val, void *priv)
{
    const ev_win_t *w = (ev_win_t *) priv;

    if (w->ptr)
        *(uint16_t *) &w->ptr[addr - w->addr] = val;
}

static void
ev_win_add(everex_t *dev, ev_win_t *w, uint32_t addr, uint32_t size)
{
    w->addr = addr;
    w->size = size;
    mem_mapping_add(&w->mapping, addr, size,
                    ev_readb, dev->wide ? ev_readw : NULL, NULL,
                    ev_writeb, dev->wide ? ev_writew : NULL, NULL,
                    NULL, MEM_MAPPING_EXTERNAL, w);
    mem_mapping_disable(&w->mapping);
}

/* Point a window at board RAM (or at nothing) and keep the mapping and the
   memory state in step. Conventional memory the board supplies replaces any
   system-board RAM decoded there. */
static void
ev_win_set(ev_win_t *w, uint32_t addr, uint8_t *ptr)
{
    if (w->active && (addr != w->addr)) {
        mem_mapping_disable(&w->mapping);
        if (w->addr < (mem_size << 10))
            mem_set_mem_state(w->addr, w->size, MEM_READ_INTERNAL | MEM_WRITE_INTERNAL);
        w->active = 0;
    }

    w->ptr = ptr;
    if (ptr != NULL) {
        if (!w->active || (addr != w->addr)) {
            w->addr = addr;
            mem_mapping_set_addr(&w->mapping, addr, w->size);
            mem_set_mem_state(addr, w->size, MEM_READ_EXTERNAL | MEM_WRITE_EXTERNAL);
        }
        mem_mapping_set_exec(&w->mapping, ptr);
        w->active = 1;
    } else if (w->active) {
        mem_mapping_disable(&w->mapping);
        if (w->addr < (mem_size << 10))
            mem_set_mem_state(w->addr, w->size, MEM_READ_INTERNAL | MEM_WRITE_INTERNAL);
        w->active = 0;
    }
    if (ptr == NULL)
        w->addr = addr;
}

static uint8_t *
ev_page(everex_t *dev, uint32_t page)
{
    return (page < dev->pages) ? dev->ram + (page * EV_PG) : NULL;
}

/* ---------------------------------------------------------------- */
/* EV-158A (gen-1 register sets)                                    */
/* ---------------------------------------------------------------- */

/* Register p drives the window of the frame whose A15:A14 equal p. */
static uint32_t
ev158_window_addr(uint32_t frame, int p)
{
    return frame + ((uint32_t) ((p - (int) (frame >> 14)) & 3) << 14);
}

static void
ev158_update(everex_t *dev, int s)
{
    ev158_set_t *set = &dev->set[s];

    for (int p = 0; p < 4; p++) {
        uint8_t  v   = set->reg[p];
        uint8_t *ptr = (v & 0x80) ? ev_page(dev, (uint32_t) s * 128 + (v & 0x7f)) : NULL;

        ev_win_set(&set->win[p], ev158_window_addr(set->frame, p), ptr);
    }
}

static uint8_t
ev158_in(uint16_t port, void *priv)
{
    everex_t *dev = (everex_t *) priv;
    uint8_t   ret = 0xff;

    for (int s = 0; s < dev->nsets; s++) {
        if ((port & 0x3ff) == dev->set[s].base)
            ret = dev->set[s].reg[port >> 14];
    }

    everex_log(dev->log, "EV-158A: in  %04X = %02X\n", port, ret);

    return ret;
}

static void
ev158_out(uint16_t port, uint8_t val, void *priv)
{
    everex_t *dev = (everex_t *) priv;
    int       p   = port >> 14;

    everex_log(dev->log, "EV-158A: out %04X, %02X\n", port, val);

    for (int s = 0; s < dev->nsets; s++) {
        ev158_set_t *set = &dev->set[s];

        if ((port & 0x3ff) == set->base) {
            set->reg[p] = val;
            ev158_update(dev, s);
        } else if (((port & 0x3ff) == (set->base + 1)) && (p < 3)) {
            set->latch = (uint8_t) ((set->latch & ~(1 << p)) | ((val >> 7) << p));
            set->frame = 0xc4000 + ((uint32_t) set->latch << 14);
            ev158_update(dev, s);
        }
    }
}

/* ---------------------------------------------------------------- */
/* EV-178 (AMZ enhanced registers)                                  */
/* ---------------------------------------------------------------- */
static void
ev178_update(everex_t *dev, int w)
{
    uint16_t v   = dev->map[w];
    uint8_t *ptr = (v & 0x800) ? ev_page(dev, v & 0x3ff) : NULL;

    ev_win_set(&dev->win[w], (uint32_t) w * EV_PG, ptr);
}

static uint8_t
ev178_in(uint16_t port, void *priv)
{
    everex_t *dev = (everex_t *) priv;
    uint8_t   ret = 0xff;
    int       odd = (port & 1);

    switch (port >> 14) {
        case 0:
            if (odd)
                ret = dev->index;
            else if ((dev->id_state >= 2) && (dev->id_state < 5)) {
                ret = (uint8_t) "AMZ"[dev->id_state - 2];
                dev->id_state++;
            } else
                dev->id_state = 0;
            break;
        case 1: {
            uint16_t v = dev->map[dev->index & 0x3f];

            ret = odd ? (v >> 8) : (v & 0xff);
            /* Bit 6 also steps the index after a high-byte read: EMML's
               page-map restore (4E01h) reads the windows it keeps and
               writes 0 to the rest in one auto-increment pass. */
            if (odd && (dev->index & 0x40))
                dev->index = (uint8_t) ((dev->index & 0xc0) | ((dev->index + 1) & 0x3f));
            break;
        }
        case 2:
            ret = odd ? dev->sw8201 : dev->sw8200;
            break;
        case 3:
            ret = odd ? 0xff : dev->cfgc200;
            break;
        default:
            break;
    }

    everex_log(dev->log, "EV-178: in  %04X = %02X\n", port, ret);

    return ret;
}

static void
ev178_out(uint16_t port, uint8_t val, void *priv)
{
    everex_t *dev = (everex_t *) priv;
    int       odd = (port & 1);

    everex_log(dev->log, "EV-178: out %04X, %02X\n", port, val);

    switch (port >> 14) {
        case 0:
            if (odd)
                dev->index = val;
            else if (val == 0xe5)
                dev->id_state = 1;
            else
                dev->id_state = ((dev->id_state == 1) && (val == 0x5a)) ? 2 : 0;
            break;
        case 1: {
            const int w = dev->index & 0x3f;

            if (odd)
                dev->map[w] = (uint16_t) ((dev->map[w] & 0x00ff) | (val << 8));
            else
                dev->map[w] = (uint16_t) ((dev->map[w] & 0xff00) | val);
            ev178_update(dev, w);
            /* Bit 6: auto-increment after the high byte (read or write). */
            if (odd && (dev->index & 0x40))
                dev->index = (uint8_t) ((dev->index & 0xc0) | ((dev->index + 1) & 0x3f));
            break;
        }
        default: /* B+8201h is written 00h by the driver; no effect known */
            break;
    }
}

/* ---------------------------------------------------------------- */
/* Device                                                           */
/* ---------------------------------------------------------------- */
static void *
everex_init(const device_t *info)
{
    everex_t *dev = (everex_t *) calloc(1, sizeof(everex_t));
    uint32_t  conv_kb;
    uint32_t  ext_kb;
    uint32_t  used;

    dev->log      = log_open("Everex");
    dev->type     = (int) info->local;
    dev->ram_size = (uint32_t) device_get_config_int("size") << 10;
    dev->wide     = is286 && (device_get_config_int("width") == 16);
    dev->ram      = (uint8_t *) calloc(1, dev->ram_size);
    dev->pages    = dev->ram_size / EV_PG;

    /* Conventional fill from the configured start up to 640 KB, then
       extended memory; both come from the start of board RAM. The EMS
       page numbering covers all board RAM, so the driver's start page
       (or the EV-178 switches) keeps EMS clear of them. */
    conv_kb = (uint32_t) device_get_config_int("conv_start");
    conv_kb = (conv_kb && (conv_kb < 640)) ? (640 - conv_kb) : 0;
    ext_kb  = is286 ? (uint32_t) device_get_config_int("ext_size") : 0;
    if (((conv_kb + ext_kb) << 10) > dev->ram_size)
        ext_kb = (dev->ram_size >> 10) > conv_kb ? (dev->ram_size >> 10) - conv_kb : 0;
    if ((conv_kb << 10) > dev->ram_size)
        conv_kb = dev->ram_size >> 10;
    used = 0;

    if (conv_kb) {
        ev_win_add(dev, &dev->conv, (640 - conv_kb) << 10, conv_kb << 10);
        ev_win_set(&dev->conv, (640 - conv_kb) << 10, dev->ram);
        isa_mem_size += conv_kb;
        used += conv_kb << 10;
    }
    if (ext_kb) {
        uint32_t ext_start = (uint32_t) device_get_config_int("ext_start") << 10;

        ev_win_add(dev, &dev->ext, ext_start, ext_kb << 10);
        ev_win_set(&dev->ext, ext_start, dev->ram + used);
        isa_mem_size += ext_kb;
        used += ext_kb << 10;
    }

    if (dev->type == EV_158A) {
        uint16_t base  = (uint16_t) device_get_config_hex16("base");
        int      first = 0;

        for (int i = 0; i < 7; i++)
            if (ev_ports[i] == base)
                first = i;

        /* One register set per 2 MB, at consecutive table addresses. */
        dev->nsets = (int) ((dev->ram_size + (2048u << 10) - 1) / (2048u << 10));
        if (dev->nsets > EV158_SETS)
            dev->nsets = EV158_SETS;
        if ((first + dev->nsets) > 7)
            dev->nsets = 7 - first;

        for (int s = 0; s < dev->nsets; s++) {
            ev158_set_t *set = &dev->set[s];

            set->base  = ev_ports[first + s];
            set->frame = 0xc4000;
            for (int p = 0; p < 4; p++) {
                ev_win_add(dev, &set->win[p], 0xc4000, EV_PG);
                io_sethandler(set->base + (p << 14), 2, ev158_in, NULL, NULL, ev158_out, NULL, NULL, dev);
            }
            everex_log(dev->log, "EV-158A: register set %d at %03X\n", s, set->base);
        }
    } else {
        int conv_start_blk = conv_kb ? (int) ((640 - conv_kb) / 128) : 0;
        int banks          = (int) ((dev->pages + 127) / 128);

        dev->base = (uint16_t) device_get_config_hex16("base");

        /* Switch readbacks are active low, in 128 KB units. */
        dev->sw8200  = (uint8_t) ((conv_kb ? 0x80 : 0x00) | (~conv_start_blk & 0x7f));
        /* B+8201h is the first board page given to EMS: everything below it
           is conventional fill and extended memory (EMM 4.05 at 5660h builds
           its page list from here). */
        dev->sw8201  = (uint8_t) (0x80 | (~((conv_kb + ext_kb) / 128) & 0x7f));
        dev->cfgc200 = (uint8_t) (0x08 | ((banks ? banks : 1) - 1));

        for (int w = 0; w < EV178_WINDOWS; w++)
            ev_win_add(dev, &dev->win[w], (uint32_t) w * EV_PG, EV_PG);

        /* The board supplies its conventional fill from power-on, through
           the window registers, from board page 0 up. The fill block set
           up above is replaced by the per-window registers. */
        if (conv_kb) {
            ev_win_set(&dev->conv, dev->conv.addr, NULL);
            for (int w = conv_start_blk * 8, pg = 0; w < EV178_CONV_WIN; w++, pg++) {
                dev->map[w] = (uint16_t) (0x800 | pg);
                ev178_update(dev, w);
            }
        }

        for (int i = 0; i < 4; i++)
            io_sethandler(dev->base + (i << 14), 2, ev178_in, NULL, NULL, ev178_out, NULL, NULL, dev);
    }

    everex_log(dev->log, "%s: %u KB, conventional %u KB, extended %u KB, %u pages\n",
               info->name, dev->ram_size >> 10, conv_kb, ext_kb, dev->pages);

    return dev;
}

static void
everex_close(void *priv)
{
    everex_t *dev = (everex_t *) priv;

    free(dev->ram);
    if (dev->log != NULL) {
        log_close(dev->log);
        dev->log = NULL;
    }
    free(dev);
}

// clang-format off
#define EV_CFG_BASE                                              \
    {                                                            \
        .name           = "base",                                \
        .description    = "Address",                             \
        .type           = CONFIG_HEX16,                          \
        .default_string = NULL,                                  \
        .default_int    = 0x0258,                                \
        .file_filter    = NULL,                                  \
        .spinner        = { 0 },                                 \
        .selection      = {                                      \
            { .description = "208H", .value = 0x0208 },          \
            { .description = "218H", .value = 0x0218 },          \
            { .description = "258H", .value = 0x0258 },          \
            { .description = "268H", .value = 0x0268 },          \
            { .description = "2A8H", .value = 0x02A8 },          \
            { .description = "2B8H", .value = 0x02B8 },          \
            { .description = "2E8H", .value = 0x02E8 },          \
            { .description = ""                      }           \
        },                                                       \
        .bios           = { { 0 } }                              \
    }

#define EV_CFG_SIZE(def, maxkb)                                  \
    {                                                            \
        .name           = "size",                                \
        .description    = "Memory size",                         \
        .type           = CONFIG_SPINNER,                        \
        .default_string = NULL,                                  \
        .default_int    = def,                                   \
        .file_filter    = NULL,                                  \
        .spinner        = { .min = 2048, .max = maxkb, .step = 2048 }, \
        .selection      = { { 0 } },                             \
        .bios           = { { 0 } }                              \
    }

#define EV_CFG_COMMON                                            \
    {                                                            \
        .name           = "conv_start",                          \
        .description    = "Fill base memory from (KB, 0 = no fill)", \
        .type           = CONFIG_SPINNER,                        \
        .default_string = NULL,                                  \
        .default_int    = 0,                                     \
        .file_filter    = NULL,                                  \
        .spinner        = { .min = 0, .max = 512, .step = 128 }, \
        .selection      = { { 0 } },                             \
        .bios           = { { 0 } }                              \
    },                                                           \
    {                                                            \
        .name           = "ext_start",                           \
        .description    = "Extended memory start (KB)",          \
        .type           = CONFIG_SPINNER,                        \
        .default_string = NULL,                                  \
        .default_int    = 1024,                                  \
        .file_filter    = NULL,                                  \
        .spinner        = { .min = 1024, .max = 15360, .step = 128 }, \
        .selection      = { { 0 } },                             \
        .bios           = { { 0 } }                              \
    },                                                           \
    {                                                            \
        .name           = "ext_size",                            \
        .description    = "Extended memory size (KB)",           \
        .type           = CONFIG_SPINNER,                        \
        .default_string = NULL,                                  \
        .default_int    = 0,                                     \
        .file_filter    = NULL,                                  \
        .spinner        = { .min = 0, .max = 15232, .step = 128 }, \
        .selection      = { { 0 } },                             \
        .bios           = { { 0 } }                              \
    },                                                           \
    {                                                            \
        .name           = "width",                               \
        .description    = "EMS transfer width",                  \
        .type           = CONFIG_SELECTION,                      \
        .default_string = NULL,                                  \
        .default_int    = 8,                                     \
        .file_filter    = NULL,                                  \
        .spinner        = { 0 },                                 \
        .selection      = {                                      \
            { .description = "8-bit",  .value =  8 },            \
            { .description = "16-bit", .value = 16 },            \
            { .description = ""                    }             \
        },                                                       \
        .bios           = { { 0 } }                              \
    }

static const device_config_t ev158a_config[] = {
    EV_CFG_SIZE(2048, 8192),
    EV_CFG_BASE,
    EV_CFG_COMMON,
    { .name = "", .description = "", .type = CONFIG_END }
};

static const device_config_t ev178_config[] = {
    EV_CFG_SIZE(2048, 16384),
    EV_CFG_BASE,
    EV_CFG_COMMON,
    { .name = "", .description = "", .type = CONFIG_END }
};
// clang-format on

const device_t ev158a_device = {
    .name          = "Everex EV-158A",
    .internal_name = "ev158a",
    .flags         = DEVICE_ISA,
    .local         = EV_158A,
    .init          = everex_init,
    .close         = everex_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ev158a_config
};

const device_t ev178_device = {
    .name          = "Everex EV-178",
    .internal_name = "ev178",
    .flags         = DEVICE_ISA,
    .local         = EV_178,
    .init          = everex_init,
    .close         = everex_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ev178_config
};
