/*
 * iab_core.h - Hardware model of the Intel Above Board family (ISA).
 *
 * Portable C99, no emulator dependencies. Included by the 86Box device
 * (isamem_iab.c) and by the Unicorn-based test harness. Every behaviour
 * here is taken from the reverse-engineered EMM.SYS / SETBOARD.EXE spec
 * (Intel_AboveBoard_EMM_Spec.md); section numbers refer to that file.
 *
 * Models:
 *   IAB_PC, IAB_PSPC   Gen-1 single-bank, DIP-switch boards     (2.1)
 *   IAB_AT, IAB_PSAT   two-bank AT boards (+ piggyback)        (2.2)
 *   IAB_286            Above Board 286 / PS/286, 16 registers   (2.3)
 *   IAB_PLUS1/2        Above Board Plus / Plus I/O, ASIC rev 1/2 (2.4)
 *   IAB_PLUS8          Above Board Plus 8 / Plus 8 I/O (+6 MB)   (2.4)
 *   IAB_MMC            Matched Memory Classic                  (2.4)
 *
 * The emulator glue must:
 *   - route every I/O access the board claims (iab_io_ranges) to
 *     iab_io_read / iab_io_write (full 16-bit port numbers!)
 *   - route memory accesses in the 16 KB windows the board may decode to
 *     iab_window_ptr(), re-evaluating after IAB_EV_MAP events
 *   - map ext_ptr/ext_base/ext_len as linear RAM (extended memory)
 */
#ifndef IAB_CORE_H
#define IAB_CORE_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

enum iab_type {
    IAB_PC = 0, /* Above Board /PC            (SETBOARD code 1, alias var. B) [G name] */
    IAB_PSPC,   /* Above Board PS/PC          (code 3, alias var. A)          [G name] */
    IAB_AT,     /* Above Board /AT            (code 2, var. B)                [G name] */
    IAB_PSAT,   /* Above Board PS/AT          (code 4, var. A)                [G name] */
    IAB_286,    /* Above Board 286 / PS/286   (codes 5/6) */
    IAB_PLUS1,  /* Above Board Plus rev 1     (codes 7/8) */
    IAB_PLUS2,  /* Above Board Plus rev 2     (codes 9/A) */
    IAB_PLUS8,  /* Above Board Plus 8         (codes B/C) */
    IAB_MMC,    /* Matched Memory Classic     (codes D/E) */
    IAB_NTYPES
};

#define IAB_PAGE       0x4000u
#define IAB_BANK_PAGES 128u
#define IAB_BANK_SIZE  (IAB_BANK_PAGES * IAB_PAGE) /* 2 MB */

/* Events reported to the glue */
#define IAB_EV_MAP 1 /* window mapping changed: re-query iab_window_ptr() */
#define IAB_EV_IO  2 /* I/O base changed (only at reset)                */

typedef struct iab_core {
    /* ---- configuration (set before iab_core_init) ---- */
    int      type;
    uint32_t ram_kb;       /* installed RAM */
    int      at_mode;      /* 1 = AT/16-bit (no PC selection plug)  */
    int      piggy6;       /* Plus 8 only: 6 MB piggyback attached   */
    uint16_t sw_iobase;    /* DIP-switch boards: I/O address (2x8h)  */
    /* defaults used to build an EEPROM image when none is loaded */
    uint16_t def_iobase;   /* 0 = factory 268h */
    int      def_conv;     /* 0 none, 1 256-640, 2 256-512, 3 512-640 */
    uint32_t def_ext_start_kb, def_ext_end_kb; /* 0 = none */
    const char *def_serial; /* 11 chars, NULL = generated */

    /* ---- derived / state ---- */
    uint8_t *ram;
    uint32_t ram_size;
    int      nbanks;
    uint16_t iobase;       /* active I/O address (2x8h) */
    uint8_t  latch[4];     /* frame-select latches, D7 of each write */

    /* legacy register files (Gen-1, 2-bank, 286): shared low bits */
    uint8_t  lo7[4];
    uint8_t  en[4][4];     /* [bank][pos] */

    /* Plus register file */
    uint8_t  preg[8][9][4];/* [vboard][blk][pos] */
    uint8_t  sig_n;        /* B+F bits 2..0 as written */
    uint8_t  enh;          /* B+F bits 7..6 */
    uint8_t  mem16_mask;   /* B+100F */
    uint8_t  mc_range;     /* B+400D */

    /* EEPROM */
    uint16_t eep[64];
    uint16_t eep_rd;
    uint8_t  eep_lat_lo, eep_lat_hi;
    int      eep_dirty;
    int      has_eeprom;

    /* extended memory decoded from EEPROM (AT boards only) */
    uint32_t ext_base, ext_len;
    uint8_t *ext_ptr;
    /* 286 conventional fill (Plus does it through the page registers) */
    uint32_t conv_base, conv_len;
    uint8_t *conv_ptr;

    void (*event)(void *ctx, int ev);
    void *ctx;
} iab_core_t;

/* ------------------------------------------------------------------ */
static inline int iab_is_plus(const iab_core_t *c) { return c->type >= IAB_PLUS1; }
static inline int iab_is_gen1(const iab_core_t *c) { return c->type == IAB_PC || c->type == IAB_PSPC; }
static inline int iab_is_2bank(const iab_core_t *c) { return c->type == IAB_AT || c->type == IAB_PSAT; }
static inline int iab_alias_a(const iab_core_t *c) { return c->type == IAB_PSPC || c->type == IAB_PSAT; }

static void iab_ev(iab_core_t *c, int ev) { if (c->event) c->event(c->ctx, ev); }

/* Virtual-board (bank) offsets, EMMAB8 init 8B46h */
static const uint16_t iab_bank_off[8] = { 0x0000, 0x2000, 0x1000, 0x3000, 0x0400, 0x2400, 0x1400, 0x3400 };
static const uint16_t iab_io_codes[8] = { 0x208, 0x218, 0x248, 0x258, 0x268, 0x2A8, 0x2B8, 0x2E8 };

/* CRC used by SETBOARD for the serial-number block (word 13h):
   CRC-16/CCITT-FALSE, stored byte-swapped. */
static uint16_t iab_serial_crc(const uint8_t *p, int n)
{
    uint16_t c = 0xffff;
    while (n--) {
        c ^= (uint16_t) (*p++) << 8;
        for (int i = 0; i < 8; i++)
            c = (c & 0x8000) ? (uint16_t) ((c << 1) ^ 0x1021) : (uint16_t) (c << 1);
    }
    return (uint16_t) ((c >> 8) | (c << 8));
}

/* ---- frame decoding (1.6) ---- */
static uint32_t iab_frame_seg(const iab_core_t *c)
{
    int f = (c->latch[3] << 3) | (c->latch[2] << 2) | (c->latch[1] << 1) | c->latch[0];
    if (iab_is_gen1(c))
        return 0xC400 + (f & 7) * 0x400;
    if (f == 0)
        return 0;
    if (c->at_mode) {
        if (f > 5) return 0;
        return 0xC000 + (f - 1) * 0x400;
    }
    if (f > 7) return 0;
    return 0xC800 + (f - 1) * 0x400;
}

/* ---- window -> memory pointer (NULL = board does not respond) ---- */
static int iab_blk(uint32_t seg)
{
    if (seg >= 0x4000 && seg < 0xA000) return (int) (seg >> 12) - 4; /* 0..5 */
    if (seg >= 0xC000 && seg < 0xF000) return (int) (seg >> 12) - 0xC + 6; /* 6..8 */
    return -1;
}

static uint8_t *iab_page_ptr(iab_core_t *c, int bank, uint8_t page)
{
    uint32_t off = (uint32_t) bank * IAB_BANK_SIZE + (uint32_t) page * IAB_PAGE;
    if (off + IAB_PAGE > c->ram_size)
        return NULL; /* unpopulated: floats */
    return c->ram + off;
}

/* seg must be 16 KB aligned (low 10 bits zero) */
static uint8_t *iab_window_ptr(iab_core_t *c, uint32_t seg)
{
    int pos = (int) ((seg >> 10) & 3);

    if (!iab_is_plus(c)) {
        uint32_t fs = iab_frame_seg(c);
        if (!fs || seg < fs || seg >= fs + 0x1000)
            return NULL;
        pos = (int) ((seg >> 10) & 3);
        for (int b = 0; b < c->nbanks; b++)
            if (c->en[b][pos])
                return iab_page_ptr(c, b, c->lo7[pos]);
        return NULL;
    }

    int blk = iab_blk(seg);
    if (blk < 0)
        return NULL;
    if (blk >= 6 && !(c->enh & 0xC0)) {
        /* compatibility mode: only the latch-selected frame, through the
           IOBASE (block 8) registers (2.4 "Two decode modes") */
        uint32_t fs = iab_frame_seg(c);
        if (!fs || seg < fs || seg >= fs + 0x1000)
            return NULL;
        blk = 8;
    }
    /* B+100F (mem16_mask) is stored but not used to gate decode: its
       MEMCS16 meaning is inferred only (spec 2.4). */
    for (int v = 0; v < c->nbanks; v++) {
        uint8_t r = c->preg[v][blk][pos];
        if (r & 0x80)
            return iab_page_ptr(c, v, r & 0x7f);
    }
    return NULL;
}

/* ---- EEPROM image ----
 *
 * Word map (Intel spec 2.7.2; SETBOARD 1.60 behaviour from the EEPROM
 * brute-force runs, spec 2.7.5). What the model acts on:
 *   w1  bits 12..10 I/O base, 15..14 conventional fill, 9..5 / 4..0
 *       extended start / end (512 KB units, end inclusive)
 *   w3  bits 14..13 / 12..11 extended start / end fine bits (128 KB),
 *       used when w6 bit 0 = 0, Plus rev 2 and later only
 *   w3F bits 15..8 = AAh: programmed (else the board answers at 268h)
 * Stored but with no effect in the model (timing only on real boards):
 *   w0  bits 2..0 bus timing (SETBOARD: 000 16-bit, 101 8-bit)
 *   w3F bits 3..1 bus speed (0 8-bit "any", 1 10 MHz, 2 12 MHz,
 *       3 >12 MHz, 7 6-8 MHz), bits 7..4 (kept as found by SETBOARD
 *       1.60, 0 when it programs a blank board; the 1987 286 SETBOARD
 *       writes 1111), bit 0 RAM speed (automatic setup 1, manual
 *       setup 0; forced 0 on the Plus 8 / MMC)
 *   w1  bit 13 (always cleared by SETBOARD), w3 bit 15 (board supplies
 *       conventional memory; EMM reads only w1)
 *   w2  scratch, w1Ah-23h factory data (never read by any Intel tool)
 *
 * TODO: serial/parallel ports of the I/O models (Plus I/O, Plus 8 I/O,
 *       PS/286) are not emulated. They would need:
 *         w14h capability: bit 0 parallel, bit 1 or 3 main serial,
 *              bit 2, 5 or 6 second serial (not offered on the Plus 8
 *              when a main serial bit is set)
 *         w0   bits 5..4 parallel (01 378h, 10 278h), bit 6 serial A
 *              enable, 8..7 address (11 3F8h, 01 2F8h, 10 3E8h, 00 2E8h),
 *              10..9 IRQ (00 2, 11 3, 10 4, 01 5), bit 11 serial B
 *              enable, 13..12 / 15..14 its address / IRQ
 *       w14h is left 0 so SETBOARD treats every board as port-less.
 */
static void iab_eep_default(iab_core_t *c)
{
    char sn[12];
    for (int i = 0; i < 64; i++) c->eep[i] = 0xffff;
    if (c->def_serial && strlen(c->def_serial) >= 11)
        memcpy(sn, c->def_serial, 11);
    else
        memcpy(sn, iab_is_plus(c) ? "M0000000001" : "E0000000001", 11);
    sn[11] = 0;

    uint16_t io = c->def_iobase ? c->def_iobase : 0x268;
    int ioc = 4;
    for (int i = 0; i < 8; i++) if (iab_io_codes[i] == io) ioc = i;

    int fine = (c->type >= IAB_PLUS2);
    uint16_t w0 = 0x0008 | (c->at_mode ? 0 : 5);   /* ports disabled */
    uint16_t w1 = (uint16_t) ((ioc << 10) | ((c->def_conv & 3) << 14));
    uint16_t w3 = 0, w6 = fine ? 0 : 1;
    if (c->def_ext_start_kb >= 1024 && c->def_ext_end_kb > c->def_ext_start_kb) {
        uint32_t s = c->def_ext_start_kb, e = c->def_ext_end_kb, last;
        if (!fine) { s &= ~511u; e = (e + 511) & ~511u; }
        w1 |= (uint16_t) (((s / 512) & 0x1f) << 5);
        last = e - 128;
        if (fine) {
            w1 |= (uint16_t) ((last / 512) & 0x1f);
            w3 |= (uint16_t) ((((s % 512) / 128) & 3) << 13);
            w3 |= (uint16_t) ((((last % 512) / 128) & 3) << 11);
        } else
            w1 |= (uint16_t) ((e / 512 - 1) & 0x1f);
    }
    if (c->def_conv && fine)
        w3 |= 0x8000;
    c->eep[0] = w0;
    c->eep[1] = w1;
    c->eep[2] = 0xffff;
    if (iab_is_plus(c)) { c->eep[3] = w3; c->eep[6] = w6; }
    for (int i = 0; i < 11; i++)
        c->eep[8 + i] = (uint16_t) ((uint8_t) sn[i] | ((uint8_t) ~sn[i] << 8));
    c->eep[0x13] = iab_serial_crc((const uint8_t *) sn, 11);
    c->eep[0x14] = 0x0000;                         /* no serial/parallel fitted (see TODO above) */
    if (c->type == IAB_286)
        c->eep[0x3f] = c->at_mode ? 0xAAFF : 0xAA01;
    else if (c->type == IAB_PLUS8 || c->type == IAB_MMC)
        c->eep[0x3f] = c->at_mode ? 0xAA0E : 0xAA00;
    else
        c->eep[0x3f] = c->at_mode ? 0xAA0F : 0xAA01;
}

/* ---- apply configuration (power-on / hard reset) ---- */
static void iab_core_reset(iab_core_t *c)
{
    memset(c->latch, 0, sizeof(c->latch));
    memset(c->lo7, 0, sizeof(c->lo7));
    memset(c->en, 0, sizeof(c->en));
    memset(c->preg, 0, sizeof(c->preg));
    c->sig_n = 0; c->enh = 0; c->mem16_mask = 0; c->mc_range = 0;
    c->ext_base = c->ext_len = 0; c->ext_ptr = NULL;
    c->conv_base = c->conv_len = 0; c->conv_ptr = NULL;

    if (!c->has_eeprom) {
        c->iobase = c->sw_iobase ? c->sw_iobase : 0x258;
        iab_ev(c, IAB_EV_IO);
        iab_ev(c, IAB_EV_MAP);
        return;
    }

    uint16_t w1 = c->eep[1];
    c->iobase = ((c->eep[0x3f] >> 8) == 0xAA) ? iab_io_codes[(w1 >> 10) & 7] : 0x268;

    /* conventional fill (word 1 bits 15..14) */
    int conv = (w1 >> 14) & 3;
    uint32_t fill_lo = 0, fill_hi = 0;
    if (conv == 1) { fill_lo = 0x40000; fill_hi = 0xA0000; }
    if (conv == 2) { fill_lo = 0x40000; fill_hi = 0x80000; }
    if (conv == 3) { fill_lo = 0x80000; fill_hi = 0xA0000; }
    if (conv && iab_is_plus(c)) {
        /* power-on map through bank-0 registers (EMMAB8 table 4FEEh) */
        for (uint32_t a = fill_lo; a < fill_hi; a += IAB_PAGE) {
            uint32_t seg = a >> 4;
            int blk = iab_blk(seg), pos = (int) ((seg >> 10) & 3);
            uint8_t v = (seg < 0x8000) ? (uint8_t) (0x90 + ((seg - 0x4000) >> 10))
                                       : (uint8_t) (0x80 + ((seg - 0x8000) >> 10));
            c->preg[0][blk][pos] = v;
        }
    } else if (conv && c->type == IAB_286 && c->ram_size >= fill_hi - fill_lo) {
        c->conv_base = fill_lo; c->conv_len = fill_hi - fill_lo; c->conv_ptr = c->ram;
    }

    /* extended memory (AT only): word 1 + fine bits in word 3 */
    if (c->at_mode) {
        uint32_t sf = (w1 >> 5) & 0x1f, ef = w1 & 0x1f;
        if (sf && ef) {
            /* the 128 KB fine bits exist from the Plus rev 2 (SETBOARD
               codes 9/A) on; rev 1 is 512 KB granular whatever w6 says */
            int fine = (c->type >= IAB_PLUS2) && !(c->eep[6] & 1);
            uint32_t s = sf * 512, e;
            if (fine) {
                s += ((c->eep[3] >> 13) & 3) * 128;
                e = ef * 512 + (((c->eep[3] >> 11) & 3) + 1) * 128;
            } else
                e = (ef + 1) * 512;
            if (e > s && s >= 1024) {
                uint32_t len = (e - s) * 1024;
                if (len <= c->ram_size) {
                    c->ext_base = s * 1024;
                    c->ext_len = len;
                    c->ext_ptr = c->ram + c->ram_size - len; /* top of board RAM */
                }
            }
        }
    }
    iab_ev(c, IAB_EV_IO);
    iab_ev(c, IAB_EV_MAP);
}

static int iab_core_init(iab_core_t *c)
{
    switch (c->type) {
        case IAB_PC: case IAB_PSPC: c->nbanks = 1; break;
        case IAB_AT: case IAB_PSAT: c->nbanks = 2; c->at_mode = 1; break;
        case IAB_PLUS8: c->nbanks = c->piggy6 ? 7 : 4; break;
        default: c->nbanks = 4; break;
    }
    if (c->ram_kb > (uint32_t) c->nbanks * 2048) c->ram_kb = (uint32_t) c->nbanks * 2048;
    c->ram_size = c->ram_kb * 1024;
    c->ram = (uint8_t *) calloc(1, c->ram_size ? c->ram_size : 1);
    if (!c->ram) return -1;
    c->has_eeprom = (c->type >= IAB_286);
    if (c->has_eeprom && (c->eep[0x3f] >> 8) != 0xAA && c->eep[0] == 0)
        iab_eep_default(c); /* glue may preload eep[] from a file first */
    iab_core_reset(c);
    return 0;
}

static void iab_core_free(iab_core_t *c) { free(c->ram); c->ram = NULL; }

/* ---- I/O ---- */
/* Fills up to max ranges {start,len}; returns count. */
static int iab_io_ranges(const iab_core_t *c, uint16_t (*r)[2], int max)
{
    int n = 0;
    uint16_t b = (uint16_t) (c->iobase & 0x2F0);
    for (int hi = 0; hi < 64 && n < max; hi++) {
        uint16_t p = (uint16_t) (hi << 10);
        if (iab_is_plus(c)) { r[n][0] = (uint16_t) (b + p); r[n][1] = 16; }
        else                 { r[n][0] = (uint16_t) (b + 8 + p); r[n][1] = 8; }
        n++;
    }
    return n;
}

static uint8_t iab_eep_io(iab_core_t *c, int low, int write, uint8_t val)
{
    if (write) {
        if (low == 4) c->eep_lat_lo = val;
        else if (low == 5) c->eep_lat_hi = val;
        else if (low == 6) {
            int a = val & 0x3f;
            if ((val & 0xC0) == 0x80) c->eep_rd = c->eep[a];
            else if ((val & 0xC0) == 0xC0) {
                /* data latches hold the complement (SETBOARD 2EC:00E8) */
                c->eep[a] = (uint16_t) ((uint8_t) ~c->eep_lat_lo | ((uint8_t) ~c->eep_lat_hi << 8));
                c->eep_dirty = 1;
            }
        }
        return 0xff;
    }
    if (low == 4) return (uint8_t) c->eep_rd;
    if (low == 5) return (uint8_t) (c->eep_rd >> 8);
    return 0xff;
}

static uint8_t iab_status9(const iab_core_t *c)
{
    uint8_t v = c->at_mode ? 0x20 : 0x00;
    if (c->type == IAB_PLUS8 && c->piggy6) v |= 0x10;
    if (c->type == IAB_MMC) v |= 0x18;
    return v;
}

static uint8_t iab_family(const iab_core_t *c)
{
    switch (c->type) {
        case IAB_PLUS1: return 0x00;
        case IAB_PLUS2: return 0x10;
        default: return 0x18;
    }
}

/* port is the full 16-bit address. *hit set to 1 if the board drives it */
static uint8_t iab_io_read(iab_core_t *c, uint16_t port, int *hit)
{
    uint16_t b = (uint16_t) (c->iobase & 0x2F0);
    int low = port & 0x0F;
    int pos = port >> 14;
    *hit = 0;
    if ((port & 0x03F0) != b) return 0xff;

    if (iab_is_plus(c)) {
        int vb_bits = port & 0x3C00, v = -1;
        if (low <= 8) {
            for (int i = 0; i < c->nbanks; i++) if (iab_bank_off[i] == vb_bits) v = i;
            if (v < 0) return 0xff;
            *hit = 1;
            return c->preg[v][low][pos];
        }
        if (low == 9) { *hit = 1; return iab_status9(c); }
        if ((port & 0x3C00) == 0) {
            if (low == 0xF && pos == 0) { *hit = 1; return (uint8_t) (c->enh | iab_family(c) | ((c->sig_n + 3) & 7)); }
            if (low == 0xD && pos == 1) { *hit = 1; return c->mc_range; }
            if (pos == 3 && low >= 0xC && low <= 0xE) { *hit = 1; return iab_eep_io(c, low - 8, 0, 0); }
        }
        return 0xff;
    }

    /* legacy boards: IOBASE-relative low offset */
    int l = low - 8;
    if (l < 0) return 0xff;
    if (c->type == IAB_286) {
        if (port & 0x0C00) return 0xff;
        int bank = (port >> 12) & 3;
        if (l == 0) { *hit = 1; return (uint8_t) ((c->en[bank][pos] << 7) | c->lo7[pos]); }
        if (l == 1) { *hit = 1; return (uint8_t) (0xDF | (c->at_mode ? 0x20 : 0)); }
        if (pos == 3 && bank == 0 && l >= 4 && l <= 6) { *hit = 1; return iab_eep_io(c, l, 0, 0); }
        return 0xff;
    }
    int bank = iab_is_2bank(c) ? ((port >> 13) & 1) : 0;
    int ok = iab_alias_a(c) ? (l != 1 || iab_is_2bank(c)) : (l <= 1);
    if (!ok) return 0xff;
    *hit = 1;
    return (uint8_t) ((c->en[bank][pos] << 7) | c->lo7[pos]);
}

static void iab_io_write(iab_core_t *c, uint16_t port, uint8_t val, int *hit)
{
    uint16_t b = (uint16_t) (c->iobase & 0x2F0);
    int low = port & 0x0F;
    int pos = port >> 14;
    *hit = 0;
    if ((port & 0x03F0) != b) return;

    if (iab_is_plus(c)) {
        int vb_bits = port & 0x3C00, v = -1;
        if (low <= 8) {
            for (int i = 0; i < c->nbanks; i++) if (iab_bank_off[i] == vb_bits) v = i;
            if (v < 0) return;
            *hit = 1;
            c->preg[v][low][pos] = val;
            iab_ev(c, IAB_EV_MAP);
            return;
        }
        if (low == 9) { *hit = 1; c->latch[pos] = val >> 7; iab_ev(c, IAB_EV_MAP); return; }
        if (low == 0xF && (port & 0xFC00) == 0) {
            *hit = 1; c->sig_n = val & 7; c->enh = val & 0xC0; iab_ev(c, IAB_EV_MAP); return;
        }
        if (low == 0xF && (port & 0xFC00) == 0x1000) { *hit = 1; c->mem16_mask = val; iab_ev(c, IAB_EV_MAP); return; }
        if (low == 0xD && (port & 0xFC00) == 0x4000) { *hit = 1; c->mc_range = val; return; }
        if ((port & 0xFC00) == 0xC000 && low >= 0xC && low <= 0xE) { *hit = 1; iab_eep_io(c, low - 8, 1, val); return; }
        return;
    }

    int l = low - 8;
    if (l < 0) return;
    if (l == 1) { *hit = 1; c->latch[pos] = val >> 7; iab_ev(c, IAB_EV_MAP); return; }
    if (c->type == IAB_286) {
        if (port & 0x0C00) return;
        int bank = (port >> 12) & 3;
        if (l == 0) {
            *hit = 1; c->lo7[pos] = val & 0x7f; c->en[bank][pos] = val >> 7; iab_ev(c, IAB_EV_MAP); return;
        }
        if (pos == 3 && bank == 0 && l >= 4 && l <= 6) { *hit = 1; iab_eep_io(c, l, 1, val); }
        return;
    }
    if (l == 0) {
        int bank = iab_is_2bank(c) ? ((port >> 13) & 1) : 0;
        *hit = 1; c->lo7[pos] = val & 0x7f; c->en[bank][pos] = val >> 7; iab_ev(c, IAB_EV_MAP);
    }
}

#endif /* IAB_CORE_H */
