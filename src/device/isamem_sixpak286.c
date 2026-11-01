/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          AST SixPak 286 memory section (AST p/n 500741).
 *
 *          See the header of sp286_init() for the register model.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2026 Jasmine Iwanek.
 *
 *          This program is free software; you can redistribute it and/or
 *          modify it under the terms of the GNU General Public License as
 *          published by the Free Software Foundation; either version 2 of
 *          the License, or (at your option) any later version.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include "cpu.h"
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/nvr.h>
#include <86box/plat.h>
#include <86box/isamem.h>

#ifdef ENABLE_SP286_LOG
int sp286_do_log = ENABLE_SP286_LOG;

static void
sp286_log(const char *fmt, ...)
{
    va_list ap;

    if (sp286_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define sp286_log(fmt, ...)
#endif

#define SP286_BASES    2
#define SP286_SETS     64            /* map sets (index high byte) */
#define SP286_WINDOWS  64            /* 16 KB windows in the first megabyte */
#define SP286_PAGE     (16 << 10)
#define SP286_BASE_RAM (2048 << 10)  /* RAM behind each base address */
#define SP286_NV_PORT  0x02c9
#define SP286_NV_RECS  16
#define SP286_ID       0xd0 /* REMM 4.41 also accepts 70h and E0h for related boards */

typedef struct sp286_t sp286_t;

typedef struct sp286_base_t {
    sp286_t *dev;
    int      idx;
    uint16_t iobase; /* 0 = not decoded */
    uint8_t  creg_index;
    uint8_t  creg[256];
    uint16_t map_index;
    uint8_t  sel;         /* compatible mode: RAMpage-style register selector at B+1 */
    uint8_t  raw[SP286_WINDOWS]; /* compatible mode: register bytes as written */
    uint8_t  sw4001;      /* compatible mode: B+4001 readback */
    uint16_t map[SP286_SETS][SP286_WINDOWS];
} sp286_base_t;

#define MAPPER_MODE(bs) ((bs)->creg[0x40] & 0x80)

typedef struct sp286_win_t {
    sp286_t      *dev;
    int           win;
    uint8_t      *ptr; /* current target, NULL = not driven */
    mem_mapping_t mapping;
} sp286_win_t;

typedef struct sp286_nvrec_t {
    uint16_t addr;
    uint8_t  data[16];
} sp286_nvrec_t;

struct sp286_t {
    uint8_t      *ram;
    /* linear (non-EMS) memory: board RAM from offset 0 */
    uint32_t      lin_size;
    uint32_t      lin_start; /* physical start address */
    mem_mapping_t lin_conv;
    mem_mapping_t lin_ext;
    uint32_t      lin_conv_base, lin_conv_len, lin_ext_base, lin_ext_len;

    uint32_t     ram_size;
    int          lane;
    int          wide;
    sp286_base_t base[SP286_BASES];
    sp286_win_t  win[SP286_WINDOWS];

    /* serial configuration memory on port 2C9h */
    int           nv_state; /* 0 key, 1 address, 2 data */
    int           nv_bits;
    uint16_t      nv_addr;
    int           nv_dir; /* 0 undecided, 1 read, 2 write */
    uint8_t       nv_buf[16];
    sp286_nvrec_t nv[SP286_NV_RECS];
    int           nv_count;
    int           nv_dirty;
    char          nv_fn[64];
};

/* The 64-bit key, sent LSB first, byte 0 first. */
static const uint8_t sp286_key[8] = { 0x20, 0x8c, 0xf2, 0xe2, 0xa4, 0xdd, 0xac, 0x7e };

static int
key_bit(int n)
{
    return (sp286_key[n >> 3] >> (n & 7)) & 1;
}

/* ---------------------------------------------------------------- */
/* Memory decode                                                    */
/* ---------------------------------------------------------------- */
static uint8_t *
sp286_target(sp286_t *dev, int w)
{
    for (int b = 0; b < SP286_BASES; b++) {
        sp286_base_t *bs = &dev->base[b];
        if (!bs->iobase)
            continue;
        uint16_t e = bs->map[bs->creg[0x20] & (SP286_SETS - 1)][w];
        if (!(e & 0x02))
            continue;
        uint32_t off = (uint32_t) b * SP286_BASE_RAM + (uint32_t) (e >> 2) * SP286_PAGE;
        if (off + SP286_PAGE <= dev->ram_size)
            return dev->ram + off;
    }
    return NULL;
}

static void
sp286_update_window(sp286_t *dev, int w)
{
    sp286_win_t *win = &dev->win[w];

    if (!win->dev)
        return;
    win->ptr = sp286_target(dev, w);
    if (win->ptr) {
        mem_mapping_set_exec(&win->mapping, win->ptr);
        mem_mapping_enable(&win->mapping);
    } else
        mem_mapping_disable(&win->mapping);
}

static void
sp286_update_all(sp286_t *dev)
{
    for (int w = 0; w < SP286_WINDOWS; w++)
        sp286_update_window(dev, w);

    /* In mapper mode the conventional backfill comes from the map table,
       which REMM rebuilds (as its mappable conventional pages). */
    if (dev->lin_conv_len) {
        if (MAPPER_MODE(&dev->base[0]))
            mem_mapping_disable(&dev->lin_conv);
        else
            mem_mapping_enable(&dev->lin_conv);
    }
}

static uint8_t
win_readb(uint32_t addr, void *priv)
{
    const sp286_win_t *win = (sp286_win_t *) priv;

    return win->ptr ? win->ptr[addr & 0x3fff] : 0xff;
}

static uint16_t
win_readw(uint32_t addr, void *priv)
{
    const sp286_win_t *win = (sp286_win_t *) priv;

    return win->ptr ? *(uint16_t *) &win->ptr[addr & 0x3fff] : 0xffff;
}

static void
win_writeb(uint32_t addr, uint8_t val, void *priv)
{
    const sp286_win_t *win = (sp286_win_t *) priv;

    if (win->ptr)
        win->ptr[addr & 0x3fff] = val;
}

static void
win_writew(uint32_t addr, uint16_t val, void *priv)
{
    const sp286_win_t *win = (sp286_win_t *) priv;

    if (win->ptr)
        *(uint16_t *) &win->ptr[addr & 0x3fff] = val;
}

/* ---------------------------------------------------------------- */
/* Register file                                                    */
/* ---------------------------------------------------------------- */
static sp286_base_t *
sp286_decode(sp286_t *dev, uint16_t port)
{
    for (int b = 0; b < SP286_BASES; b++)
        if (dev->base[b].iobase && ((port & 0x03ff) == dev->base[b].iobase ||
                                    (port & 0x03ff) == dev->base[b].iobase + 1))
            return &dev->base[b];
    return NULL;
}

static uint16_t *
sp286_entry(sp286_base_t *bs, int set, int w)
{
    return &bs->map[set & (SP286_SETS - 1)][w & (SP286_WINDOWS - 1)];
}

static uint8_t
sp286_in(uint16_t port, void *priv)
{
    sp286_t      *dev = (sp286_t *) priv;
    sp286_base_t *bs  = sp286_decode(dev, port);
    uint8_t       ret = 0xff;

    if (!bs)
        return ret;

    int odd = port & 1;
    int grp = (port >> 14) & 3;

    if (grp == 2) {
        /* Control register file, both modes. */
        ret = odd ? bs->creg[bs->creg_index] : bs->creg_index;
    } else if (MAPPER_MODE(bs)) {
        if (grp == 0) {
            uint16_t e = *sp286_entry(bs, bs->map_index >> 8, bs->map_index >> 2);
            ret        = odd ? (e >> 8) : (e & 0xff);
        } else if (grp == 3)
            ret = odd ? (bs->map_index >> 8) : (bs->map_index & 0xff);
    } else {
        /* RAMpage-compatible mode. */
        if (!odd) {
            int w = (bs->sel & 0x3f) + grp;
            if (w < SP286_WINDOWS) {
                uint16_t e = *sp286_entry(bs, bs->creg[0x20], w);
                ret        = ((e & 0x02) ? 0x80 : 0x00) | ((e >> 2) & 0x7f);
            }
        } else if (grp == 0)
            ret = bs->sel;
        else if (grp == 1)
            ret = bs->sw4001;
    }
    sp286_log("SP286: in  %04X = %02X\n", port, ret);
    return ret;
}

static void
sp286_out(uint16_t port, uint8_t val, void *priv)
{
    sp286_t      *dev = (sp286_t *) priv;
    sp286_base_t *bs  = sp286_decode(dev, port);

    if (!bs)
        return;

    sp286_log("SP286: out %04X, %02X\n", port, val);
    int odd = port & 1;
    int grp = (port >> 14) & 3;

    if (grp == 2) {
        if (!odd)
            bs->creg_index = val;
        else if (bs->creg_index != 0x00) {
            bs->creg[bs->creg_index] = val;
            if ((bs->creg_index == 0x20) || (bs->creg_index == 0x40))
                sp286_update_all(dev);
        }
    } else if (MAPPER_MODE(bs)) {
        if (grp == 0) {
            int       set = bs->map_index >> 8;
            int       w   = (bs->map_index >> 2) & (SP286_WINDOWS - 1);
            uint16_t *e   = sp286_entry(bs, set, w);
            *e            = odd ? ((*e & 0x00ff) | (val << 8)) : ((*e & 0xff00) | val);
            if ((set & (SP286_SETS - 1)) == (bs->creg[0x20] & (SP286_SETS - 1)))
                sp286_update_window(dev, w);
        } else if (grp == 3)
            bs->map_index = odd ? ((bs->map_index & 0x00ff) | (val << 8)) : ((bs->map_index & 0xff00) | val);
    } else {
        if (!odd) {
            int w = (bs->sel & 0x3f) + grp;
            if (w < SP286_WINDOWS) {
                bs->raw[w]                          = val;
                *sp286_entry(bs, bs->creg[0x20], w) = (val & 0x80) ? (uint16_t) (((val & 0x7f) << 2) | 0x02) : 0;
                sp286_update_window(dev, w);
            }
        } else if (grp == 0)
            bs->sel = val;
    }
}

/* ---------------------------------------------------------------- */
/* Serial configuration memory (port 2C9h)                          */
/* ---------------------------------------------------------------- */
static sp286_nvrec_t *
nv_find(sp286_t *dev, uint16_t addr, int create)
{
    for (int i = 0; i < dev->nv_count; i++)
        if (dev->nv[i].addr == addr)
            return &dev->nv[i];
    if (!create || dev->nv_count >= SP286_NV_RECS)
        return NULL;
    sp286_nvrec_t *r = &dev->nv[dev->nv_count++];
    r->addr          = addr;
    memset(r->data, 0xff, sizeof(r->data));
    return r;
}

static void
nv_reset(sp286_t *dev)
{
    dev->nv_state = 0;
    dev->nv_bits  = 0;
    dev->nv_dir   = 0;
}

static uint8_t
sp286_nv_in(uint16_t port, void *priv)
{
    sp286_t *dev = (sp286_t *) priv;
    uint8_t  ret = 0xff;

    (void) port;
    if ((dev->nv_state == 2) && (dev->nv_dir != 2)) {
        if (dev->nv_dir == 0) {
            const sp286_nvrec_t *r = nv_find(dev, dev->nv_addr, 0);
            if (r)
                memcpy(dev->nv_buf, r->data, 16);
            else
                memset(dev->nv_buf, 0xff, 16);
            dev->nv_dir  = 1;
            dev->nv_bits = 0;
        }
        if (!((dev->nv_buf[dev->nv_bits >> 3] >> (dev->nv_bits & 7)) & 1))
            ret &= ~(1 << dev->lane);
        if (++dev->nv_bits == 128)
            nv_reset(dev);
    } else
        nv_reset(dev); /* a read outside a data phase restarts the key */

    return ret;
}

static void sp286_apply(sp286_t *dev);
static void nv_save(sp286_t *dev);

static void
sp286_nv_out(uint16_t port, uint8_t val, void *priv)
{
    sp286_t *dev = (sp286_t *) priv;
    int      bit = (val >> dev->lane) & 1;

    (void) port;
    switch (dev->nv_state) {
        case 0:
            if (bit == key_bit(dev->nv_bits)) {
                if (++dev->nv_bits == 64) {
                    dev->nv_state = 1;
                    dev->nv_bits  = 0;
                    dev->nv_addr  = 0;
                }
            } else
                dev->nv_bits = (bit == key_bit(0)) ? 1 : 0;
            break;

        case 1:
            /* Two bytes, LSB first; the first byte is the high byte. */
            if (bit)
                dev->nv_addr |= 1 << ((dev->nv_bits < 8) ? (8 + dev->nv_bits) : (dev->nv_bits - 8));
            if (++dev->nv_bits == 16) {
                dev->nv_state = 2;
                dev->nv_bits  = 0;
                dev->nv_dir   = 0;
            }
            break;

        case 2:
            if (dev->nv_dir == 1) {
                /* A write during a read ends it and starts a new key. */
                nv_reset(dev);
                sp286_nv_out(port, val, priv);
                return;
            }
            if (dev->nv_dir == 0) {
                dev->nv_dir  = 2;
                dev->nv_bits = 0;
                memset(dev->nv_buf, 0, 16);
            }
            if (bit)
                dev->nv_buf[dev->nv_bits >> 3] |= 1 << (dev->nv_bits & 7);
            if (++dev->nv_bits == 128) {
                sp286_nvrec_t *r = nv_find(dev, dev->nv_addr, 1);
                if (r) {
                    memcpy(r->data, dev->nv_buf, 16);
                    dev->nv_dirty = 1;
                    nv_save(dev);
                }
                sp286_log("SP286: NV write %04X\n", dev->nv_addr);
                nv_reset(dev);
                sp286_apply(dev);
            }
            break;

        default:
            break;
    }
}

/*
 * Configuration record 0677h ("SixPak 286"), 16 bytes:
 *   byte 0     bits 7..3 checksum, bits 2..0 = 100b
 *   byte 1     00h
 *   bytes 2-7  36 bits, 6 per byte, as one bit stream that starts at
 *              byte 7 bit 0 and runs down to byte 2 bit 5. Fields, in
 *              stream order, LSB first:
 *                 5  f0   [?]
 *                 1  f1   base 2 enable
 *                 4  f2   base 2 I/O code
 *                 2  f3   base 1 I/O code, bits 1..0
 *                 2  f4   base 1 I/O code, bits 3..2
 *                 4  f5   base 2 linear size, as f6
 *                 4  f6   linear size: (rev4(f6) + 1) x 128 KB
 *                 2  f7   } base 2 linear memory: "memory below" it,
 *                 5  f8   } rev7(f7 | f8 << 2) x 128 KB; all ones = none
 *                         (f5 is then base 2's linear size code)
 *                 1  f9   1 = no linear memory (then f6 = 0, f10 = 3Fh)
 *                 6  f10  system memory below the board's linear memory,
 *                         rev6(f10) x 128 KB (SMART6 "Memory installed").
 *                         The linear memory starts there; at or above
 *                         640 KB it starts 384 KB higher, past the UMA.
 *   bytes 8-9  06h 77h, the product tag
 *   bytes 10-15 FFh
 * An I/O code is the bit-reversed middle nibble of the port:
 * 208h -> 0, 218h -> 8, 258h -> Ah, 268h -> 6, 2A8h -> 5, 2B8h -> Dh,
 * 2E8h -> 7. SMART6 parks a base with code 9 (298h, no board).
 * The checksum: c = 1; for i = 1..14: c = rol8(c ^ b[i] ^ b[i + 1]);
 * stored as (c & 1Fh) << 3.
 */
static const int sp286_fw[11] = { 5, 1, 4, 2, 2, 4, 4, 2, 5, 1, 6 };

static void
sp286_unpack(const uint8_t *rec, int *f)
{
    int k = 0;

    for (int i = 0; i < 11; i++) {
        f[i] = 0;
        for (int b = 0; b < sp286_fw[i]; b++, k++)
            if ((rec[7 - (k / 6)] >> (k % 6)) & 1)
                f[i] |= 1 << b;
    }
}

static void
sp286_pack(uint8_t *rec, const int *f)
{
    int k = 0;

    memset(rec, 0xff, 16);
    rec[1] = 0x00;
    for (int i = 2; i < 8; i++)
        rec[i] = 0x00;
    for (int i = 0; i < 11; i++)
        for (int b = 0; b < sp286_fw[i]; b++, k++)
            if ((f[i] >> b) & 1)
                rec[7 - (k / 6)] |= 1 << (k % 6);
    rec[8] = 0x06;
    rec[9] = 0x77;
}

static uint8_t
sp286_checksum(const uint8_t *rec)
{
    uint8_t c = 1;

    for (int i = 1; i < 15; i++) {
        c ^= rec[i] ^ rec[i + 1];
        c = (uint8_t) ((c << 1) | (c >> 7));
    }
    return c & 0x1f;
}

static int
rev4(int v)
{
    return ((v & 1) << 3) | ((v & 2) << 1) | ((v & 4) >> 1) | ((v & 8) >> 3);
}

static int
rev6(int v)
{
    int r = 0;

    for (int i = 0; i < 6; i++)
        if (v & (1 << i))
            r |= 1 << (5 - i);
    return r;
}

static uint8_t
lin_readb(uint32_t addr, void *priv)
{
    const sp286_t *dev = (sp286_t *) priv;
    uint32_t       off = (addr >= dev->lin_ext_base && dev->lin_ext_len) ? (addr - dev->lin_ext_base + dev->lin_conv_len) : (addr - dev->lin_conv_base);

    return (off < dev->ram_size) ? dev->ram[off] : 0xff;
}

static uint16_t
lin_readw(uint32_t addr, void *priv)
{
    return (uint16_t) (lin_readb(addr, priv) | (lin_readb(addr + 1, priv) << 8));
}

static void
lin_writeb(uint32_t addr, uint8_t val, void *priv)
{
    sp286_t *dev = (sp286_t *) priv;
    uint32_t off = (addr >= dev->lin_ext_base && dev->lin_ext_len) ? (addr - dev->lin_ext_base + dev->lin_conv_len) : (addr - dev->lin_conv_base);

    if (off < dev->ram_size)
        dev->ram[off] = val;
}

static void
lin_writew(uint32_t addr, uint16_t val, void *priv)
{
    lin_writeb(addr, val & 0xff, priv);
    lin_writeb(addr + 1, val >> 8, priv);
}

/* Place the linear memory: the part below 640 KB backfills conventional
   memory, the rest is extended memory from 1 MB (or higher). */
static void
sp286_set_linear(sp286_t *dev, uint32_t start, uint32_t size1, uint32_t size2)
{
    uint32_t size = size1 + size2;

    if (size > dev->ram_size)
        size = dev->ram_size;
    dev->lin_start     = start;
    dev->lin_size      = size;
    dev->lin_conv_base = start;
    dev->lin_conv_len  = 0;
    dev->lin_ext_base  = 0;
    dev->lin_ext_len   = 0;
    if (size && (start < 0xa0000)) {
        dev->lin_conv_len = 0xa0000 - start;
        if (dev->lin_conv_len > size)
            dev->lin_conv_len = size;
    }
    if (size > dev->lin_conv_len) {
        dev->lin_ext_base = (start < 0x100000) ? 0x100000 : start;
        dev->lin_ext_len  = size - dev->lin_conv_len;
        if (!is286)
            dev->lin_ext_len = 0; /* no extended memory on an 8088 */
    }

    if (dev->lin_conv_len) {
        mem_mapping_set_addr(&dev->lin_conv, dev->lin_conv_base, dev->lin_conv_len);
        mem_mapping_set_exec(&dev->lin_conv, dev->ram);
    } else
        mem_mapping_disable(&dev->lin_conv);
    if (dev->lin_ext_len) {
        mem_mapping_set_addr(&dev->lin_ext, dev->lin_ext_base, dev->lin_ext_len);
        mem_mapping_set_exec(&dev->lin_ext, dev->ram + dev->lin_conv_len);
        mem_set_mem_state(dev->lin_ext_base, dev->lin_ext_len, MEM_READ_EXTERNAL | MEM_WRITE_EXTERNAL);
    } else
        mem_mapping_disable(&dev->lin_ext);

    /* Mirror it in the control registers REMM reads on the D0h/E0h boards. */
    /* Per-base readbacks. Base 2 holds the linear memory past the first
       2 MB, which follows base 1's directly. */
    for (int b = 0; b < SP286_BASES; b++) {
        sp286_base_t *bs = &dev->base[b];
        uint32_t      sz = b ? size2 : size1;
        uint32_t      st = b ? (start + size1 + ((start < 0xa0000 && start + size1 > 0xa0000) ? 0x60000 : 0)) : start;

        /* Control registers 50h/60h, which REMM reads on these boards. */
        bs->creg[0x50] = sz ? (uint8_t) (st >> 17) : 0x7f;
        bs->creg[0x60] = sz ? (uint8_t) (((sz >> 17) - 1) & 0x3f) : 0x00;
        /* With no index written since reset, B+8001 reads a RAMpage-style
           switch byte: high nibble = linear size / 128 KB - 1 (Fh = none).
           SMART6 sizes the EMS part from there. [?: inferred from SMART6] */
        bs->creg[0xff] = (uint8_t) (sz ? ((((sz >> 17) - 1) & 0x0f) << 4) | 0x0f : 0xff);
        /* B+4001 reads a RAMpage AT-style start code in bits 7..1: below
           640 KB the start in 128 KB units, otherwise that minus 3; 7Fh
           means no linear memory. */
        if (!sz)
            bs->sw4001 = 0xfe;
        else if (st < 0xa0000)
            bs->sw4001 = (uint8_t) ((st >> 17) << 1);
        else
            bs->sw4001 = (uint8_t) ((((st >> 17) - 3) & 0x7f) << 1);
    }
    sp286_log("SP286: linear %u KB at %05X: conv %05X+%u KB, ext %06X+%u KB\n", size >> 10, start,
              dev->lin_conv_base, dev->lin_conv_len >> 10, dev->lin_ext_base, dev->lin_ext_len >> 10);
}

static uint16_t
sp286_code_to_port(int code)
{
    uint16_t port = (uint16_t) (0x0208 | (rev4(code) << 4));

    switch (port) {
        case 0x0208: case 0x0218: case 0x0258: case 0x0268:
        case 0x02a8: case 0x02b8: case 0x02e8:
            return port;
        default:
            return 0; /* parked */
    }
}

static void
sp286_set_io(sp286_t *dev, int b, uint16_t port)
{
    sp286_base_t *bs = &dev->base[b];

    if (bs->iobase == port)
        return;
    if (bs->iobase)
        for (int h = 0; h < 4; h++)
            io_removehandler(bs->iobase + (h << 14), 2,
                             sp286_in, NULL, NULL, sp286_out, NULL, NULL, dev);
    bs->iobase = port;
    if (port)
        for (int h = 0; h < 4; h++)
            io_sethandler(port + (h << 14), 2,
                          sp286_in, NULL, NULL, sp286_out, NULL, NULL, dev);
    sp286_update_all(dev);
}

/* The configuration memory is kept in <internal_name>_<board>.nvr:
   up to 16 records of {address (2 bytes, high first), data (16 bytes)}. */
static void
nv_load(sp286_t *dev)
{
    FILE   *fp = nvr_fopen(dev->nv_fn, "rb");
    uint8_t buf[18];

    if (fp == NULL)
        return;
    dev->nv_count = 0;
    while ((dev->nv_count < SP286_NV_RECS) && (fread(buf, 1, sizeof(buf), fp) == sizeof(buf))) {
        sp286_nvrec_t *r = &dev->nv[dev->nv_count++];
        r->addr          = (uint16_t) ((buf[0] << 8) | buf[1]);
        memcpy(r->data, buf + 2, 16);
    }
    fclose(fp);
}

static void
nv_save(sp286_t *dev)
{
    FILE   *fp = nvr_fopen(dev->nv_fn, "wb");
    uint8_t buf[18];

    if (fp == NULL)
        return;
    for (int i = 0; i < dev->nv_count; i++) {
        buf[0] = dev->nv[i].addr >> 8;
        buf[1] = dev->nv[i].addr & 0xff;
        memcpy(buf + 2, dev->nv[i].data, 16);
        (void) fwrite(buf, 1, sizeof(buf), fp);
    }
    fclose(fp);
    dev->nv_dirty = 0;
}

/* The board (re)loads its configuration from record 0677h. */
static void
sp286_apply(sp286_t *dev)
{
    const sp286_nvrec_t *r = nv_find(dev, 0x0677, 0);
    int                  f[11];

    if (!r || (r->data[8] != 0x06) || (r->data[9] != 0x77)) {
        sp286_set_io(dev, 0, 0);
        sp286_set_io(dev, 1, 0);
        sp286_set_linear(dev, 0, 0, 0);
        return;
    }
    sp286_unpack(r->data, f);
    for (int b = 0; b < SP286_BASES; b++) {
        dev->base[b].creg_index = 0xff; /* reads FFh until an index is written [?] */
        dev->base[b].creg[0x40] = 0x00; /* RAMpage-compatible mode */
    }
    sp286_set_io(dev, 0, sp286_code_to_port((f[4] << 2) | f[3]));
    sp286_set_io(dev, 1, (f[1] && (dev->ram_size > SP286_BASE_RAM)) ? sp286_code_to_port(f[2]) : 0);
    if (f[9])
        sp286_set_linear(dev, 0, 0, 0);
    else {
        uint32_t below = (uint32_t) rev6(f[10]) << 17;
        uint32_t size1 = (uint32_t) (rev4(f[6]) + 1) << 17;
        uint32_t size2 = 0;
        /* Base 2 linear part: f5 = size code, f7/f8 = its "memory below"
           (all ones = none). Only with base 1 fully linear. */
        if (!((f[7] == 3) && (f[8] == 0x1f)) && (size1 == SP286_BASE_RAM) && dev->base[1].iobase)
            size2 = (uint32_t) (rev4(f[5]) + 1) << 17;
        sp286_set_linear(dev, (below < 0xa0000) ? below : (below + 0x60000), size1, size2);
    }
    sp286_log("SP286: record %02X %02X %02X %02X %02X %02X %02X %02X, I/O %03X/%03X\n",
              r->data[0], r->data[1], r->data[2], r->data[3], r->data[4], r->data[5],
              r->data[6], r->data[7], dev->base[0].iobase, dev->base[1].iobase);
}

/* ---------------------------------------------------------------- */
static void *
sp286_init(const device_t *info)
{
    sp286_t *dev = (sp286_t *) calloc(1, sizeof(sp286_t));

    dev->ram_size = (uint32_t) device_get_config_int("size") << 10;
    dev->lane     = device_get_config_int("board") - 1;
    dev->wide     = is286;
    dev->ram      = (uint8_t *) calloc(1, dev->ram_size);

    for (int b = 0; b < SP286_BASES; b++) {
        sp286_base_t *bs = &dev->base[b];
        bs->dev          = dev;
        bs->idx          = b;
        bs->creg[0x00]   = SP286_ID;
        bs->creg[0x50]   = 0x7f; /* no linear memory */
        bs->creg[0xff]   = 0xff;
    }
    io_sethandler(SP286_NV_PORT, 1, sp286_nv_in, NULL, NULL, sp286_nv_out, NULL, NULL, dev);

    /* Factory record: both bases parked, invalid checksum, so SMART6
       autoconfigures the board. */
    {
        static const int fac[11] = { 0, 0, 9, 1, 2, 0, 0, 3, 0x1f, 1, 0x3f };
        sp286_nvrec_t   *r       = nv_find(dev, 0x0677, 1);
        sp286_pack(r->data, fac);
        r->data[0] = 0x04 | (uint8_t) (((sp286_checksum(r->data) ^ 1) & 0x1f) << 3);
    }
    snprintf(dev->nv_fn, sizeof(dev->nv_fn), "%s_%i.nvr", info->internal_name, dev->lane + 1);
    nv_load(dev);

    /* Windows 256-640 KB and C0000-EFFFF can be mapped. */
    for (int w = 0x10; w < 0x3c; w++) {
        if ((w >= 0x28) && (w < 0x30))
            continue; /* A0000-BFFFF: video */
        sp286_win_t *win = &dev->win[w];
        win->dev         = dev;
        win->win         = w;
        mem_mapping_add(&win->mapping, (uint32_t) w * SP286_PAGE, SP286_PAGE,
                        win_readb, dev->wide ? win_readw : NULL, NULL,
                        win_writeb, dev->wide ? win_writew : NULL, NULL,
                        NULL, MEM_MAPPING_EXTERNAL, win);
        mem_mapping_disable(&win->mapping);
    }

    mem_mapping_add(&dev->lin_conv, 0, 0, lin_readb, dev->wide ? lin_readw : NULL, NULL,
                    lin_writeb, dev->wide ? lin_writew : NULL, NULL, NULL, MEM_MAPPING_EXTERNAL, dev);
    mem_mapping_disable(&dev->lin_conv);
    mem_mapping_add(&dev->lin_ext, 0, 0, lin_readb, lin_readw, NULL,
                    lin_writeb, lin_writew, NULL, NULL, MEM_MAPPING_EXTERNAL, dev);
    mem_mapping_disable(&dev->lin_ext);

    sp286_apply(dev);

    sp286_log("SP286: %u KB, board %d, I/O %03X/%03X\n", dev->ram_size >> 10,
              dev->lane + 1, dev->base[0].iobase, dev->base[1].iobase);
    return dev;
}

static void
sp286_close(void *priv)
{
    sp286_t *dev = (sp286_t *) priv;

    if (dev->nv_dirty)
        nv_save(dev);
    free(dev->ram);
    free(dev);
}

static const device_config_t sp286_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 2048,
        .file_filter    = NULL,
        .spinner        = { .min = 512, .max = 4096, .step = 512 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "board",
        .description    = "Board number (jumper E9-E12)",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 1,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "1 (E9)",  .value = 1 },
            { .description = "2 (E10)", .value = 2 },
            { .description = "3 (E11)", .value = 3 },
            { .description = "4 (E12)", .value = 4 },
            { .description = ""                    }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

const device_t sixpak286_device = {
    .name          = "AST SixPak 286 (memory)",
    .internal_name = "a6pak286",
    .flags         = DEVICE_ISA16,
    .local         = 0,
    .init          = sp286_init,
    .close         = sp286_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = sp286_config
};
