/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Emulation of the Intel Above Board family of ISA memory
 *          boards: Above Board /PC, PS/PC, /AT, PS/AT, 286, PS/286,
 *          Plus, Plus I/O, Plus 8, Plus 8 I/O and Matched Memory Classic.
 *
 *          The hardware model lives in iab_core.h (shared with a test
 *          harness that runs Intel's own EMM.SYS and SETBOARD.EXE against
 *          it); this file is the 86Box glue: I/O handlers, 16 KB memory
 *          windows, extended/conventional memory and the configuration
 *          EEPROM (saved as an .nvr file).
 *
 *          Software-configured boards (286 and later) take their I/O
 *          address, conventional fill and extended range from the EEPROM.
 *          New boards start at Intel's factory default of 268h; run
 *          Intel SETBOARD, then do a hard reset so the board reloads its
 *          EEPROM. The "default" options below only seed a fresh EEPROM.
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
#include <86box/nvr.h>
#include <86box/log.h>
#include <86box/isamem.h>
#include <86box/plat_unused.h>
#include "cpu.h"
#include "iab_core.h"

/* Port 0x62 hack to allow the IAB PC driver to work on an AT class system */
//#define IAB_5170_HACK 1

#ifdef ENABLE_ISAMEM_IAB_LOG
int isamem_iab_do_log = ENABLE_ISAMEM_IAB_LOG;

static void
iab_log(void *priv, const char *fmt, ...)
{
    va_list ap;

    if (isamem_iab_do_log) {
        va_start(ap, fmt);
        log_out(priv, fmt, ap);
        va_end(ap);
    }
}
#else
#    define iab_log(priv, fmt, ...)
#endif

#define IAB_NWIN     36 /* 24 x 16 KB at 40000-9FFFF, 12 x 16 KB at C0000-EFFFF */
#define IAB_MAXRANGE 64

typedef struct iab_win_t {
    uint32_t      addr;
    uint8_t      *ptr; /* current page, NULL = not decoded */
    int           active;
    mem_mapping_t mapping;
} iab_win_t;

typedef struct iab_t {
    iab_core_t    core;
    int           wide;
    int           inst;
    char          nvr_name[64];

    uint16_t      ranges[IAB_MAXRANGE][2];
    int           nranges;

    iab_win_t     win[IAB_NWIN];

    mem_mapping_t ext_mapping;
    mem_mapping_t conv_mapping;
    int           ext_on;
    int           conv_on;

    int           diag_hack;
    void         *log;
} iab_t;

/* ---------------------------------------------------------------- */
/* Memory windows                                                   */
/* ---------------------------------------------------------------- */
static uint8_t
iab_win_readb(uint32_t addr, void *priv)
{
    const iab_win_t *w = (iab_win_t *) priv;

    return w->ptr ? w->ptr[addr & 0x3fff] : 0xff;
}

static uint16_t
iab_win_readw(uint32_t addr, void *priv)
{
    const iab_win_t *w = (iab_win_t *) priv;

    return w->ptr ? *(uint16_t *) &w->ptr[addr & 0x3ffe] : 0xffff;
}

static void
iab_win_writeb(uint32_t addr, uint8_t val, void *priv)
{
    const iab_win_t *w = (iab_win_t *) priv;

    if (w->ptr)
        w->ptr[addr & 0x3fff] = val;
}

static void
iab_win_writew(uint32_t addr, uint16_t val, void *priv)
{
    const iab_win_t *w = (iab_win_t *) priv;

    if (w->ptr)
        *(uint16_t *) &w->ptr[addr & 0x3ffe] = val;
}

/* Linear (extended / 286 conventional) RAM. The mapping's base is the
   address of ptr[0]; we keep it in the mapping's p pointer struct. */
typedef struct iab_lin_t {
    uint32_t base;
    uint8_t *ptr;
} iab_lin_t;

static iab_lin_t iab_lin[ISAMEM_MAX][2];

static uint8_t
iab_lin_readb(uint32_t addr, void *priv)
{
    const iab_lin_t *l = (iab_lin_t *) priv;

    return l->ptr[addr - l->base];
}

static uint16_t
iab_lin_readw(uint32_t addr, void *priv)
{
    const iab_lin_t *l = (iab_lin_t *) priv;

    return *(uint16_t *) &l->ptr[addr - l->base];
}

static void
iab_lin_writeb(uint32_t addr, uint8_t val, void *priv)
{
    const iab_lin_t *l = (iab_lin_t *) priv;

    l->ptr[addr - l->base] = val;
}

static void
iab_lin_writew(uint32_t addr, uint16_t val, void *priv)
{
    const iab_lin_t *l = (iab_lin_t *) priv;

    *(uint16_t *) &l->ptr[addr - l->base] = val;
}

static void
iab_update_windows(iab_t *dev)
{
    for (int i = 0; i < IAB_NWIN; i++) {
        iab_win_t *w   = &dev->win[i];
        uint8_t   *ptr = iab_window_ptr(&dev->core, w->addr >> 4);

        if (ptr == w->ptr && (ptr != NULL) == w->active)
            continue;
        w->ptr = ptr;
        if (ptr) {
            mem_mapping_set_exec(&w->mapping, ptr);
            if (!w->active) {
                /* Conventional memory the board is supplying replaces
                   any system-board RAM that might be decoded there. */
                mem_set_mem_state(w->addr, 0x4000, MEM_READ_EXTERNAL | MEM_WRITE_EXTERNAL);
                mem_mapping_enable(&w->mapping);
                w->active = 1;
            }
        } else if (w->active) {
            mem_mapping_disable(&w->mapping);
            if (w->addr < (mem_size << 10))
                mem_set_mem_state(w->addr, 0x4000, MEM_READ_INTERNAL | MEM_WRITE_INTERNAL);
            w->active = 0;
        }
    }
}

static void
iab_update_linear(iab_t *dev)
{
    iab_core_t *c = &dev->core;
    iab_lin_t  *l = iab_lin[dev->inst];

    if (dev->ext_on) {
        mem_mapping_disable(&dev->ext_mapping);
        dev->ext_on = 0;
    }
    if (dev->conv_on) {
        mem_mapping_disable(&dev->conv_mapping);
        dev->conv_on = 0;
    }

    if (c->ext_len && is286) {
        l[0].base = c->ext_base;
        l[0].ptr  = c->ext_ptr;
        mem_mapping_set_addr(&dev->ext_mapping, c->ext_base, c->ext_len);
        mem_mapping_set_exec(&dev->ext_mapping, c->ext_ptr);
        mem_set_mem_state(c->ext_base, c->ext_len, MEM_READ_EXTERNAL | MEM_WRITE_EXTERNAL);
        mem_mapping_enable(&dev->ext_mapping);
        dev->ext_on = 1;
        iab_log(dev->log, "IAB: extended memory %05X-%05X\n", c->ext_base, c->ext_base + c->ext_len - 1);
    }
    if (c->conv_len) {
        l[1].base = c->conv_base;
        l[1].ptr  = c->conv_ptr;
        mem_mapping_set_addr(&dev->conv_mapping, c->conv_base, c->conv_len);
        mem_mapping_set_exec(&dev->conv_mapping, c->conv_ptr);
        mem_set_mem_state(c->conv_base, c->conv_len, MEM_READ_EXTERNAL | MEM_WRITE_EXTERNAL);
        mem_mapping_enable(&dev->conv_mapping);
        dev->conv_on = 1;
        iab_log(dev->log, "IAB: conventional fill %05X-%05X\n", c->conv_base, c->conv_base + c->conv_len - 1);
    }
}

/* ---------------------------------------------------------------- */
/* I/O                                                              */
/* ---------------------------------------------------------------- */
static uint8_t
iab_in(uint16_t port, void *priv)
{
    iab_t  *dev = (iab_t *) priv;
    int     hit;
    uint8_t ret = iab_io_read(&dev->core, port, &hit);

    iab_log(dev->log, "IAB: in  %04X = %02X%s\n", port, ret, hit ? "" : " (not decoded)");

    return hit ? ret : 0xff;
}

static void
iab_out(uint16_t port, uint8_t val, void *priv)
{
    iab_t *dev = (iab_t *) priv;
    int    hit;

    iab_io_write(&dev->core, port, val, &hit);
    iab_log(dev->log, "IAB: out %04X, %02X%s\n", port, val, hit ? "" : " (not decoded)");
}

static void
iab_io_set(iab_t *dev, int set)
{
    for (int i = 0; i < dev->nranges; i++)
        io_handler(set, dev->ranges[i][0], dev->ranges[i][1],
                   iab_in, NULL, NULL, iab_out, NULL, NULL, dev);
}

static void
iab_event(void *ctx, int ev)
{
    iab_t *dev = (iab_t *) ctx;

    if (ev == IAB_EV_IO) {
        if (dev->nranges)
            iab_io_set(dev, 0);
        dev->nranges = iab_io_ranges(&dev->core, dev->ranges, IAB_MAXRANGE);
        iab_io_set(dev, 1);
        iab_log(dev->log, "IAB: I/O base %03X\n", dev->core.iobase);
    } else if (ev == IAB_EV_MAP)
        iab_update_windows(dev);
}

/* ---------------------------------------------------------------- */
/* EEPROM persistence                                               */
/* ---------------------------------------------------------------- */
static void
iab_eep_load(iab_t *dev)
{
    FILE *fp = nvr_fopen(dev->nvr_name, "rb");

    if (fp != NULL) {
        uint8_t buf[128];

        if (fread(buf, 1, sizeof(buf), fp) == sizeof(buf)) {
            for (int i = 0; i < 64; i++)
                dev->core.eep[i] = buf[i * 2] | (buf[i * 2 + 1] << 8);
        }
        fclose(fp);
    }
}

static void
iab_eep_save(iab_t *dev)
{
    FILE   *fp;
    uint8_t buf[128];

    if (!dev->core.has_eeprom)
        return;
    for (int i = 0; i < 64; i++) {
        buf[i * 2]     = dev->core.eep[i] & 0xff;
        buf[i * 2 + 1] = dev->core.eep[i] >> 8;
    }
    fp = nvr_fopen(dev->nvr_name, "wb");
    if (fp != NULL) {
        (void) fwrite(buf, 1, sizeof(buf), fp);
        fclose(fp);
    }
}

#ifdef IAB_5170_HACK
/*
 * Same workaround as isamem.c: the /PC driver samples port 62h after its
 * page tests and treats bit 6 as a hard failure. On an AT nothing claims
 * that port, it reads FFh and every bank looks bad. Only needed to run a
 * PC-mode board in an AT-class machine.
 */
static uint8_t
iab_diag_in(UNUSED(uint16_t port), UNUSED(void *priv))
{
    return 0xbf;
}
#endif

/* ---------------------------------------------------------------- */
/* Device                                                           */
/* ---------------------------------------------------------------- */
static void *
iab_init(const device_t *info)
{
    iab_t      *dev = (iab_t *) calloc(1, sizeof(iab_t));
    iab_core_t *c   = &dev->core;

    dev->log  = log_open("IAB");
    dev->inst = device_get_instance() - 1;
    if ((dev->inst < 0) || (dev->inst >= ISAMEM_MAX))
        dev->inst = 0;

    c->type   = (int) info->local;
    c->ram_kb = (uint32_t) device_get_config_int("size");
    if (c->type == IAB_PLUS8)
        c->piggy6 = device_get_config_int("piggyback");
    if ((c->type == IAB_AT) || (c->type == IAB_PSAT))
        c->at_mode = 1;
    else if ((c->type == IAB_PC) || (c->type == IAB_PSPC))
        c->at_mode = 0;
    else
        c->at_mode = (device_get_config_int("width") == 16);
    if (c->type == IAB_PLUS1 && device_get_config_int("revision") == 2)
        c->type = IAB_PLUS2;

    if (c->type < IAB_286) {
        c->sw_iobase = (uint16_t) device_get_config_hex16("base");
    } else {
        c->def_iobase       = (uint16_t) device_get_config_hex16("base");
        c->def_conv         = device_get_config_int("conv");
        c->def_ext_start_kb = (uint32_t) device_get_config_int("ext_start");
        if (c->def_ext_start_kb)
            c->def_ext_end_kb = c->def_ext_start_kb + (uint32_t) device_get_config_int("ext_size");
        snprintf(dev->nvr_name, sizeof(dev->nvr_name), "%s_%i.nvr", info->internal_name, dev->inst + 1);
        iab_eep_load(dev);
    }

    /* The 8-bit/16-bit choice decides whether we decode word cycles. */
    dev->wide = c->at_mode && is286;
    if (c->at_mode && !is286) {
        iab_log(dev->log, "IAB: AT-configured board in an 8-bit system\n");
    }

    for (int i = 0; i < IAB_NWIN; i++) {
        iab_win_t *w = &dev->win[i];

        w->addr = (i < 24) ? (0x40000 + i * 0x4000) : (0xc0000 + (i - 24) * 0x4000);
        mem_mapping_add(&w->mapping, w->addr, 0x4000,
                        iab_win_readb, dev->wide ? iab_win_readw : NULL, NULL,
                        iab_win_writeb, dev->wide ? iab_win_writew : NULL, NULL,
                        NULL, MEM_MAPPING_EXTERNAL, w);
        mem_mapping_disable(&w->mapping);
    }
    mem_mapping_add(&dev->ext_mapping, 0x100000, 0x4000,
                    iab_lin_readb, iab_lin_readw, NULL,
                    iab_lin_writeb, iab_lin_writew, NULL,
                    NULL, MEM_MAPPING_EXTERNAL, &iab_lin[dev->inst][0]);
    mem_mapping_disable(&dev->ext_mapping);
    mem_mapping_add(&dev->conv_mapping, 0x40000, 0x4000,
                    iab_lin_readb, dev->wide ? iab_lin_readw : NULL, NULL,
                    iab_lin_writeb, dev->wide ? iab_lin_writew : NULL, NULL,
                    NULL, MEM_MAPPING_EXTERNAL, &iab_lin[dev->inst][1]);
    mem_mapping_disable(&dev->conv_mapping);

    c->event = iab_event;
    c->ctx   = dev;
    if (iab_core_init(c) != 0) {
        log_close(dev->log);
        free(dev);
        return NULL;
    }
    iab_update_linear(dev);

    if (c->ext_len && is286)
        isa_mem_size += c->ext_len >> 10;
    if (c->conv_len)
        isa_mem_size += c->conv_len >> 10;

#ifdef IAB_5170_HACK
    if (((c->type == IAB_PC) || (c->type == IAB_PSPC)) && is286) {
        dev->diag_hack = 1;
        io_sethandler(0x0062, 1, iab_diag_in, NULL, NULL, NULL, NULL, NULL, dev);
    }
#endif

    iab_log(dev->log, "IAB: %s, %u KB, %s, %d bank(s), I/O %03X\n", info->name, c->ram_kb,
            c->at_mode ? "AT" : "PC", c->nbanks, c->iobase);

    return dev;
}

static void
iab_reset_dev(void *priv)
{
    iab_t *dev = (iab_t *) priv;

    /* A hard reset makes the board reload its EEPROM (new I/O address,
       conventional fill, extended range programmed by SETBOARD). */
    iab_eep_save(dev);
    iab_core_reset(&dev->core);
    iab_update_linear(dev);
}

static void
iab_close(void *priv)
{
    iab_t *dev = (iab_t *) priv;

#ifdef IAB_5170_HACK
    if (dev->diag_hack)
        io_removehandler(0x0062, 1, iab_diag_in, NULL, NULL, NULL, NULL, NULL, dev);
#endif
    iab_eep_save(dev);
    iab_core_free(&dev->core);
    if (dev->log != NULL) {
        log_close(dev->log);
        dev->log = NULL;
    }
    free(dev);
}

/* ---------------------------------------------------------------- */
/* Configuration                                                    */
/* ---------------------------------------------------------------- */
#define IAB_CFG_SIZE(def, maxkb)                         \
    {                                                  \
        .name           = "size",                      \
        .description    = "Memory size",               \
        .type           = CONFIG_SPINNER,              \
        .default_string = NULL,                        \
        .default_int    = def,                         \
        .file_filter    = NULL,                        \
        .spinner        = { .min = 128, .max = maxkb, .step = 128 }, \
        .selection      = { { 0 } },                   \
        .bios           = { { 0 } }                    \
    }

#define IAB_CFG_SWITCH_BASE                                      \
    {                                                            \
        .name           = "base",                                \
        .description    = "I/O address (switches)",              \
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

#define IAB_CFG_EEPROM_BASE                                      \
    {                                                            \
        .name           = "base",                                \
        .description    = "I/O address (new EEPROM)",            \
        .type           = CONFIG_HEX16,                          \
        .default_string = NULL,                                  \
        .default_int    = 0x0268,                                \
        .file_filter    = NULL,                                  \
        .spinner        = { 0 },                                 \
        .selection      = {                                      \
            { .description = "208H", .value = 0x0208 },          \
            { .description = "218H", .value = 0x0218 },          \
            { .description = "248H", .value = 0x0248 },          \
            { .description = "258H", .value = 0x0258 },          \
            { .description = "268H (factory)", .value = 0x0268 },\
            { .description = "2A8H", .value = 0x02A8 },          \
            { .description = "2B8H", .value = 0x02B8 },          \
            { .description = "2E8H", .value = 0x02E8 },          \
            { .description = ""                      }           \
        },                                                       \
        .bios           = { { 0 } }                              \
    }

#define IAB_CFG_WIDTH                                            \
    {                                                            \
        .name           = "width",                               \
        .description    = "Bus (PC selection plug)",             \
        .type           = CONFIG_SELECTION,                      \
        .default_string = NULL,                                  \
        .default_int    = 16,                                    \
        .file_filter    = NULL,                                  \
        .spinner        = { 0 },                                 \
        .selection      = {                                      \
            { .description = "8-bit (plug fitted)",  .value =  8 }, \
            { .description = "16-bit (no plug)",     .value = 16 }, \
            { .description = ""                                  }  \
        },                                                       \
        .bios           = { { 0 } }                              \
    }

#define IAB_CFG_EEPROM_DEFAULTS                                  \
    {                                                            \
        .name           = "conv",                                \
        .description    = "Conventional fill (new EEPROM)",      \
        .type           = CONFIG_SELECTION,                      \
        .default_string = NULL,                                  \
        .default_int    = 0,                                     \
        .file_filter    = NULL,                                  \
        .spinner        = { 0 },                                 \
        .selection      = {                                      \
            { .description = "None",        .value = 0 },        \
            { .description = "256K - 640K", .value = 1 },        \
            { .description = "256K - 512K", .value = 2 },        \
            { .description = "512K - 640K", .value = 3 },        \
            { .description = ""                        }         \
        },                                                       \
        .bios           = { { 0 } }                              \
    },                                                           \
    {                                                            \
        .name           = "ext_start",                           \
        .description    = "Extended memory start KB (new EEPROM, 0 = none)", \
        .type           = CONFIG_SPINNER,                        \
        .default_string = NULL,                                  \
        .default_int    = 0,                                     \
        .file_filter    = NULL,                                  \
        .spinner        = { .min = 0, .max = 15872, .step = 512 }, \
        .selection      = { { 0 } },                             \
        .bios           = { { 0 } }                              \
    },                                                           \
    {                                                            \
        .name           = "ext_size",                            \
        .description    = "Extended memory size KB (new EEPROM)",\
        .type           = CONFIG_SPINNER,                        \
        .default_string = NULL,                                  \
        .default_int    = 512,                                   \
        .file_filter    = NULL,                                  \
        .spinner        = { .min = 128, .max = 14336, .step = 128 }, \
        .selection      = { { 0 } },                             \
        .bios           = { { 0 } }                              \
    }

#define IAB_CFG_END { .name = "", .description = "", .type = CONFIG_END }

// clang-format off
static const device_config_t iab_pc_config[] = {
    IAB_CFG_SIZE(2048, 2048),
    IAB_CFG_SWITCH_BASE,
    IAB_CFG_END
};

static const device_config_t iab_at_config[] = {
    IAB_CFG_SIZE(2048, 4096),
    IAB_CFG_SWITCH_BASE,
    IAB_CFG_END
};

static const device_config_t iab_286_config[] = {
    IAB_CFG_SIZE(2048, 8192),
    IAB_CFG_EEPROM_BASE,
    IAB_CFG_WIDTH,
    IAB_CFG_EEPROM_DEFAULTS,
    IAB_CFG_END
};

static const device_config_t iab_plus_config[] = {
    IAB_CFG_SIZE(2048, 8192),
    {
        .name           = "revision",
        .description    = "ASIC revision",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 2,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Rev 1 (512K extended granularity)", .value = 1 },
            { .description = "Rev 2 (128K extended granularity)", .value = 2 },
            { .description = ""                                             }
        },
        .bios           = { { 0 } }
    },
    IAB_CFG_EEPROM_BASE,
    IAB_CFG_WIDTH,
    IAB_CFG_EEPROM_DEFAULTS,
    IAB_CFG_END
};

static const device_config_t iab_plus8_config[] = {
    IAB_CFG_SIZE(8192, 14336),
    {
        .name           = "piggyback",
        .description    = "6MB Piggyback Option",
        .type           = CONFIG_BINARY,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    IAB_CFG_EEPROM_BASE,
    IAB_CFG_WIDTH,
    IAB_CFG_EEPROM_DEFAULTS,
    IAB_CFG_END
};

static const device_config_t iab_mmc_config[] = {
    IAB_CFG_SIZE(8192, 8192),
    IAB_CFG_EEPROM_BASE,
    IAB_CFG_WIDTH,
    IAB_CFG_EEPROM_DEFAULTS,
    IAB_CFG_END
};
// clang-format on

#define IAB_DEVICE(var, nm, iname, flg, typ, cfg) \
    const device_t var = {                   \
        .name          = nm,                 \
        .internal_name = iname,              \
        .flags         = flg,                \
        .local         = typ,                \
        .init          = iab_init,           \
        .close         = iab_close,          \
        .reset         = iab_reset_dev,      \
        .available     = NULL,               \
        .speed_changed = NULL,               \
        .force_redraw  = NULL,               \
        .config        = cfg                 \
    }

// clang-format off
/* Internal names iabpc, iabat and iab286 are the ones the earlier isamem.c
   devices used, so existing VM configurations keep their card. */
IAB_DEVICE(iab_pc_device,    "Intel Above Board/PC",                  "iabpc",    DEVICE_ISA,   IAB_PC,    iab_pc_config);
IAB_DEVICE(iab_pspc_device,  "Intel Above Board PS/PC",               "iabpspc",  DEVICE_ISA,   IAB_PSPC,  iab_pc_config);
IAB_DEVICE(iab_at_device,    "Intel Above Board/AT",                  "iabat",    DEVICE_ISA16, IAB_AT,    iab_at_config);
IAB_DEVICE(iab_psat_device,  "Intel Above Board PS/AT",               "iabpsat",  DEVICE_ISA16, IAB_PSAT,  iab_at_config);
IAB_DEVICE(iab_286_device,   "Intel Above Board 286 / PS/286",        "iab286",   DEVICE_ISA,   IAB_286,   iab_286_config);
IAB_DEVICE(iab_plus_device,  "Intel Above Board Plus / Plus I/O",     "iabplus",  DEVICE_ISA,   IAB_PLUS1, iab_plus_config);
IAB_DEVICE(iab_plus8_device, "Intel Above Board Plus 8 / Plus 8 I/O", "iabplus8", DEVICE_ISA,   IAB_PLUS8, iab_plus8_config);
IAB_DEVICE(iab_mmc_device,   "Intel Matched Memory Classic",          "iabmmc",   DEVICE_ISA,   IAB_MMC,   iab_mmc_config);
// clang-format on
