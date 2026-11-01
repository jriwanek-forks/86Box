/*
 * VARCem   Virtual ARchaeological Computer EMulator.
 *          An emulator of (mostly) x86-based PC systems and devices,
 *          using the ISA,EISA,VLB,MCA  and PCI system buses, roughly
 *          spanning the era between 1981 and 1995.
 *
 *          Implementation of a memory expansion board for the ISA Bus.
 *
 *          Although modern systems use direct-connect local buses to
 *          connect the CPU with its memory, originally the main system
 *          bus(es) were used for that. Memory expension cards could add
 *          memory to the system through the ISA bus, using a variety of
 *          techniques.
 *
 *          The majority of these boards could provide some (additional)
 *          conventional (low) memory, extended (high) memory on 80286
 *          and higher systems, as well as EMS bank-switched memory.
 *
 *          This implementation uses the LIM 3.2 specifications for EMS.
 *
 *          With the EMS method, the system's standard memory is expanded
 *          by means of bank-switching. One or more 'frames' in the upper
 *          memory area (640K-1024K) are used as viewports into an array
 *          of RAM pages numbered 0 to N. Each page is defined to be 16KB
 *          in size, so, for a 1024KB board, 64 such pages are available.
 *          I/O control registers are used to set up the mappings. More
 *          modern boards even have multiple 'copies' of those registers,
 *          which can be switched very fast, to allow for multitasking.
 *
 * TODO:    The EV-159 is supposed to support 16b EMS transfers, but the
 *          EMM.sys driver for it doesn't seem to want to do that..
 *
 *          EV-125 (It supports backfill)
 *              https://theretroweb.com/expansioncard/documentation/50250.pdf
 *
 *          EV-158 (RAM 10000)
 *              http://web.archive.org/web/19961104093221/http://www.everex.com/supp/techlib/memmem.html
 *
 *          EV-178-1 (RAM 8000)
 *              https://theretroweb.com/expansioncards/s/everex-ev-178-1
 *
 * Authors: Fred N. van Kempen, <decwiz@yahoo.com>
 *          Jasmine Iwanek <jriwanek@gmail.com>
 *
 *          Copyright 2018      Fred N. van Kempen.
 *          Copyright 2022-2026 Jasmine Iwanek.
 *
 *          Licensing:
 *
 *          The portions of this file written by Fred N. van Kempen for
 *          VARCem remain under the BSD 3-clause licence below, and its
 *          notice must be retained in all copies.
 *
 *          The portions Copyright 2022-2026 Jasmine Iwanek are free
 *          software: you can redistribute them and/or modify them under
 *          the terms of the GNU General Public License as published by
 *          the Free Software Foundation, either version 2 of the
 *          License, or (at your option) any later version. They are
 *          distributed in the hope that they will be useful, but WITHOUT
 *          ANY WARRANTY; without even the implied warranty of
 *          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *          GNU General Public License for more details.
 *
 *          The file as a whole may therefore be redistributed and/or
 *          modified under the GNU General Public License, version 2 or
 *          later, provided the BSD notice below is kept.
 *
 *          BSD 3-clause licence (VARCem portions):
 *
 *          Redistribution and  use  in source  and binary forms, with
 *          or  without modification, are permitted  provided that the
 *          following conditions are met:
 *
 *          1. Redistributions of  source  code must retain the entire
 *             above notice, this list of conditions and the following
 *             disclaimer.
 *
 *          2. Redistributions in binary form must reproduce the above
 *             copyright  notice,  this list  of  conditions  and  the
 *             following disclaimer in  the documentation and/or other
 *             materials provided with the distribution.
 *
 *          3. Neither the  name of the copyright holder nor the names
 *             of  its  contributors may be used to endorse or promote
 *             products  derived from  this  software without specific
 *             prior written permission.
 *
 * THIS SOFTWARE  IS  PROVIDED BY THE  COPYRIGHT  HOLDERS AND CONTRIBUTORS
 * "AS IS" AND  ANY EXPRESS  OR  IMPLIED  WARRANTIES,  INCLUDING, BUT  NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE  ARE  DISCLAIMED. IN  NO  EVENT  SHALL THE COPYRIGHT
 * HOLDER OR  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL,  EXEMPLARY,  OR  CONSEQUENTIAL  DAMAGES  (INCLUDING,  BUT  NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES;  LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  AND ON  ANY
 * THEORY OF  LIABILITY, WHETHER IN  CONTRACT, STRICT  LIABILITY, OR  TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING  IN ANY  WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define ENABLE_ISAMEM_LOG 1
#define ISAMEM_DEBUG      1

/* Port 0x61/0x62 hack to allow the RAMpage XT drivers to work on an AT class
   system (isamem_iab.c has its own switch for the IAB PC driver) */
//#define IAB_5170_HACK     1

#ifdef ENABLE_ISAMEM_LOG
#include <stdarg.h>
#endif
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
//#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/machine.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/device.h>
#ifdef ENABLE_ISAMEM_LOG
#include <86box/log.h>
#endif
#include <86box/ui.h>
#include <86box/plat.h>
#include <86box/isamem.h>

#include "cpu.h"

#define ISAMEM_IBMXT_CARD      0
#define ISAMEM_GENXT_CARD      1
#define ISAMEM_RAMCARD_CARD    2
#define ISAMEM_SYSTEMCARD_CARD 3
#define ISAMEM_IBMAT_128K_CARD 4
#define ISAMEM_IBMAT_CARD      5
#define ISAMEM_GENAT_CARD      6
#define ISAMEM_P5PAK_CARD      7
#define ISAMEM_A6PAK_CARD      8
#define ISAMEM_EMS5150_CARD    9
#define ISAMEM_EV159_CARD      10 /* Ram 3000 Deluxe*/
#define ISAMEM_RAMPAGEXT_CARD    11
#define ISAMEM_RAMPAGEXTV1_CARD  12
/* 13 was ISAMEM_ABOVEBOARD286_CARD, now in isamem_iab.c */
#define ISAMEM_BRXT_CARD         14
#define ISAMEM_BRAT_CARD         15
#define ISAMEM_EV165A_CARD       16 /* Everex Maxi Magic EMS */
#define ISAMEM_LOTECH_EMS_CARD   17
#define ISAMEM_MPLUS2_CARD       18
#define ISAMEM_IBMPCJR_CARD      19
#define ISAMEM_GENPCJR_CARD      20
#define ISAMEM_JRIDE_CARD        21
/* 22, 23 were ISAMEM_ABOVEBOARDPC/AT_CARD, now in isamem_iab.c */
#define ISAMEM_RAMPAGEAT_CARD    24
#define ISAMEM_XTMAX_EMS_CARD    25
#define ISAMEM_GENUMB_CARD       26

#define RAM_TOPMEM             (640 << 10)  /* end of low memory */
#define RAM_UMAMEM             (384 << 10)  /* upper memory block */
#define RAM_EXTMEM             (1024 << 10) /* start of high memory */

#define EV159_BASE_MEM         (128 << 10)  /* size of EV-159 base memory in cs8220 mode*/
#define EV159_EXT_1536         (1536 << 10) /* start of EV-159 high memory in cs8220 mode*/
#define EV159_EXT_1024         (1024 << 10) /* start of EV-159 high memory in backfill mode*/

#define EMS_MAXSIZE            (2048 << 10) /* max EMS memory size */
#define EMS_EV159_MAXSIZE      (3072 << 10) /* max EMS memory size for EV-159 cards */
#define EMS_LOTECH_MAXSIZE     (4096 << 10) /* max EMS memory size for lotech cards */
#define EMS_XTMAX_MAXSIZE      (32768 << 10) /* max EMS memory size for XTMax cards */
#define EMS_PGSIZE             (16 << 10)   /* one page is this big */
#define EMS_MAXPAGE            4            /* number of viewport pages */

#define EXTRAM_CONVENTIONAL    0
#define EXTRAM_HIGH            1
#define EXTRAM_XMS             2

#define UMB_MAX_RANGES         2            /* Generic UMB card: RAM ranges */
#define UMB_GRAN               (16 << 10)   /* range granularity */

typedef struct emsreg_t {
    int8_t        enabled; /* 1=ENABLED */
    uint8_t       page;    /* page# in EMS RAM */
    uint8_t       frame;   /* (varies with board) */
    uint8_t       raw_page; /* register readback shadow */
    uint8_t      *addr;    /* start addr in EMS RAM */
    mem_mapping_t mapping; /* mapping entry for page */
    uint8_t      *ram;
    uint8_t      *frame_val;
    uint16_t     *ems_size;
    uint16_t     *ems_pages;
    uint32_t     *frame_addr;
    struct memdev_t *owner;
    void         *log;
} emsreg_t;

typedef struct ext_ram_t {
    uint32_t base;
    uint8_t *ptr;
} ext_ram_t;

typedef struct memdev_t {
    const char *name;
    uint8_t     board    : 6; /* board type */
    uint8_t     reserved : 2;

    uint8_t  flags;
#define FLAG_CONFIG        0x01 /* card is configured */
#define FLAG_RAMPAGE_DUAL  0x02 /* AST dual-page register banking */
#define FLAG_WIDE          0x10 /* card uses 16b mode */
#define FLAG_FAST          0x20 /* fast (<= 120ns) chips */
#define FLAG_EMS           0x40 /* card has EMS mode enabled */

    uint8_t  frame_val[2];
    uint8_t  rampage_switch_hi[EMS_MAXPAGE];

     /* AST RAMpage-family boards expose 64 mapping registers per selector
         bank.  Bit 6 of BASE+1 selects the alternate bank when dual-page
         mode is enabled; bit 7 remains the XT movable-frame enable. */
     uint8_t  rampage_map[128];
    uint8_t  rampage_map_select;
    uint8_t  rampage_board_select;
    uint8_t  rampage_mode;         /* BASE+4001 write latch (see rampage_eems_out) */
    uint16_t rampage_linear_kb;    /* RAMpage AT SW1-1..4: linear (non-EMS) memory */

    uint16_t total_size;    /* configured size in KB */
    uint16_t base_addr[2];  /* configured I/O address */

    uint32_t start_addr;    /* configured memory start */
    uint32_t frame_addr[2]; /* configured frame address */

    uint16_t ems_size[2];   /* EMS size in KB */
    uint16_t ems_pages[2];  /* EMS size in pages */
    uint32_t ems_start[2];  /* start of EMS in RAM */

    uint8_t *ram; /* allocated RAM buffer */

    ext_ram_t ext_ram[3]; /* structures for the mappings */

    mem_mapping_t low_mapping;  /* mapping for low mem */
    mem_mapping_t high_mapping; /* mapping for high mem */

    emsreg_t ems[EMS_MAXPAGE * 2]; /* EMS controller registers */

    /* XTMax memory manager (EMS part only) */
    uint16_t xtmax_ptr[EMS_MAXPAGE]; /* 16-bit page pointers, BASE+0..7 */
    uint16_t xtmax_seg;              /* EMS window segment, BASE+A/B   */
    uint16_t xtmax_win_seg;          /* window segment at last commit   */
    uint8_t  xtmax_count;            /* committed 16 KB windows         */
    uint16_t xtmax_umb_seg;          /* UMB/debug segment, BASE+D/E     */

    /* Generic UMB card: plain RAM decoded in the upper memory area */
    ext_ram_t     umb_ram[UMB_MAX_RANGES];
    mem_mapping_t umb_mapping[UMB_MAX_RANGES];

    void    *log;
} memdev_t;

#ifdef ENABLE_ISAMEM_LOG
uint8_t isamem_do_log = ENABLE_ISAMEM_LOG;

static void
isamem_log(void *priv, const char *fmt, ...)
{
    if (isamem_do_log) {
        va_list ap;

        va_start(ap, fmt);
        log_out(priv, fmt, ap);
        va_end(ap);
    }
}
#else
#    define isamem_log(priv, fmt, ...)
#endif

#if 0
static uint8_t
rampage_odd_readback(const emsreg_t *dev, int vpage)
{
    uint8_t ret = dev->frame & 0x0f;

    /* The old code models the board's active-low switch/status inputs
       through the odd register of the viewport pair.  Keep that behavior
       for the four normal viewport aliases. */
    if ((dev->owner != NULL) && (vpage == 2) && (vpage < EMS_MAXPAGE))
        ret |= dev->owner->rampage_switch_hi[vpage];

    return ret;
}
#endif

/* Recalculate the four visible 16-KB windows from the selected EEMS
   mapping-register set and group.  The supplied 1985 driver writes a set
   number to BASE+8001, a logical group to BASE+1, then a page value to one
   of BASE/+4000/+8000/+C000. */
static void
rampage_update_mappings(memdev_t *dev)
{
    const uint8_t first = dev->rampage_map_select & 0x3f;
    const uint8_t bank = ((dev->flags & FLAG_RAMPAGE_DUAL) &&
                          (dev->rampage_map_select & 0x40)) ? 64 : 0;
    const uint8_t bank_limit = (uint8_t) (bank + 64);

    /*
     * RAMpage/XT encodes the page-frame location in the same six-bit
     * selector used to choose the first logical mapping register.  The
     * observed values are: B2->C8000, B3->CC000, ..., B8->E0000.
     * Therefore:
     *
     *     frame = (selector & 3f) * 4000h
     *
     * Bit 7 is the frame-enable bit.  Do not leave a second hard-coded
     * frame alias installed: the hardware has one movable 64-KB frame.
     */
    /* The RAMpage AT has no frame switch either (AST RAMpage AT switch
       table); AST EMM 4.10 programs its frame through the selector exactly
       as on the XT boards, so it shares this path. */
    if ((dev->board == ISAMEM_RAMPAGEXT_CARD) ||
        (dev->board == ISAMEM_RAMPAGEXTV1_CARD) ||
        (dev->board == ISAMEM_RAMPAGEAT_CARD)) {
        const uint32_t frame = ((uint32_t) first) * EMS_PGSIZE;
        const int frame_enabled = !!(dev->rampage_map_select & 0x80);

        dev->frame_addr[0] = frame;

        isamem_log(dev->log,
                   "[%04X:%08X] RAMpage frame selector=%02X -> %05X enabled=%u\n",
                   CS, cpu_state.pc, dev->rampage_map_select, frame,
                   frame_enabled);

        for (uint8_t i = 0; i < EMS_MAXPAGE; i++) {
            const uint8_t reg = (uint8_t) (bank + first + i);
            const uint8_t val = (reg < bank_limit) ? dev->rampage_map[reg] : 0;
            const uint8_t page = val & 0x7f;
            const int enabled = frame_enabled && !!(val & 0x80) &&
                                (page < dev->ems_pages[0]);

            dev->ems[i].raw_page = val;
            dev->ems[i].page = page;
            dev->ems[i].enabled = enabled;
            dev->ems[i].addr = enabled ?
                dev->ram + dev->ems_start[0] + ((uint32_t) page * EMS_PGSIZE) :
                NULL;

            mem_mapping_set_addr(&dev->ems[i].mapping,
                                 frame + ((uint32_t) i * EMS_PGSIZE),
                                 EMS_PGSIZE);

            if (enabled) {
                mem_mapping_set_exec(&dev->ems[i].mapping, dev->ems[i].addr);
                mem_mapping_enable(&dev->ems[i].mapping);
            } else {
                mem_mapping_disable(&dev->ems[i].mapping);
            }
        }
        return;
    }

    /* Normal LIM-style cards retain their existing fixed frame handling. */
    for (uint8_t i = 0; i < EMS_MAXPAGE; i++) {
        const uint8_t reg = (uint8_t) (bank + first + i);
        const uint8_t val = (reg < bank_limit) ? dev->rampage_map[reg] : 0;
        const uint8_t page = val & 0x7f;
        const int enabled = !!(val & 0x80) &&
                            (page < dev->ems_pages[0]);

        dev->ems[i].raw_page = val;
        dev->ems[i].page = page;
        dev->ems[i].enabled = enabled;
        dev->ems[i].addr = enabled ?
            dev->ram + dev->ems_start[0] + ((uint32_t) page * EMS_PGSIZE) :
            NULL;

        mem_mapping_set_addr(&dev->ems[i].mapping,
                             dev->frame_addr[0] + ((uint32_t) i * EMS_PGSIZE),
                             EMS_PGSIZE);

        if (enabled) {
            mem_mapping_set_exec(&dev->ems[i].mapping, dev->ems[i].addr);
            mem_mapping_enable(&dev->ems[i].mapping);
        } else {
            mem_mapping_disable(&dev->ems[i].mapping);
        }
    }
}

/* BASE+8001 is an AST board-identification/status port.  The driver uses it
   as a small board-number latch: it writes 0Fh and expects that value back,
   then writes the board index and expects the low nibble to echo it. */
/*
 * RAMpage AT BASE+8001 high nibble: switches SW1-1..4, the amount of board
 * memory dedicated to linear (conventional + extended) memory, read with
 * ON = 0: (n + 1) * 128 KB, 0 = 128 KB ... F = 2048 KB (AST RAMpage AT
 * switch table; AST EMM 4.10 computes ((hi + 1) * 8) pages at 2B3Eh). With
 * no linear memory the start switches are all OFF and this is ignored.
 */
static uint8_t
rampage_board_nibble(const memdev_t *dev)
{
    const uint16_t units = dev->rampage_linear_kb / 128U;

    if (units == 0)
        return 0x0f;

    return (uint8_t) ((units - 1U) & 0x0fU);
}

static uint8_t
rampage_is_xt_family(const memdev_t *dev)
{
    return (uint8_t) ((dev->board == ISAMEM_RAMPAGEXT_CARD) ||
                      (dev->board == ISAMEM_RAMPAGEXTV1_CARD));
}

static uint8_t
rampage_eems_status(const memdev_t *dev)
{
    if (rampage_is_xt_family(dev))
        return dev->rampage_board_select;

    return (uint8_t) ((rampage_board_nibble(dev) << 4) |
                      (dev->rampage_board_select & 0x0f));
}

static uint8_t
rampage_4259_readback(const memdev_t *dev)
{
    if (rampage_is_xt_family(dev) && (dev->rampage_board_select == 0))
        return 0xff;

    return (uint8_t) ~dev->rampage_switch_hi[1];
}

static uint8_t
rampage_selector_bank(const memdev_t *dev)
{
    return ((dev->flags & FLAG_RAMPAGE_DUAL) &&
            (dev->rampage_map_select & 0x40)) ? 64 : 0;
}

static uint8_t
rampage_xt_base_readback(uint16_t base)
{
    switch (base) {
        case 0x0208:
            return 0x0f;
        case 0x0218:
            return 0x07;
        case 0x0258:
            return 0x0b;
        case 0x0268:
            return 0x09;
        case 0x02a8:
            return 0x04;
        case 0x02b8:
            return 0x00;
        case 0x02e8:
            return 0x01;
        default:
            return 0x07;
    }
}

/*
 * RAMpage AT switch block 2 as seen at BASE+4001 (stored inverted, see
 * rampage_4259_readback): SW2-1..7 = start address of the board's linear
 * memory in 128 KB units, SW2-1 the MSB, in bits 7:1; SW2-8 = parity check
 * in bit 0; switch ON reads 0. All seven start switches OFF (code 7Fh)
 * means all board memory is EMS. AST EMM 4.10 takes the code from bits 7:1
 * (2B4Bh: shr al,1) - the earlier layout in bits 6:0 made it read half the
 * start address and report "Memory size is overallocated".
 */
static uint8_t
rampage_at_sw2_readback(uint16_t start_kb, uint16_t linear_kb)
{
    uint8_t rd;

    if ((linear_kb == 0) || (start_kb >= 15360))
        rd = 0xfe; /* all EMS, parity on */
    else
        rd = (uint8_t) (((start_kb / 128U) & 0x7fU) << 1);

    return (uint8_t) ~rd;
}

static uint8_t
rampage_eems_in(uint16_t port, void *priv)
{
    memdev_t *dev = (memdev_t *) priv;
    const uint16_t base = dev->base_addr[0];
    const uint16_t off = (uint16_t) (port - base);
    uint8_t ret = 0xff;

    switch (off) {
        case 0x0000:
        case 0x4000:
        case 0x8000:
        case 0xc000: {
            const uint8_t window = (uint8_t) (off >> 14);
            const uint8_t bank = rampage_selector_bank(dev);
            const uint8_t reg = (uint8_t) (bank + (dev->rampage_map_select & 0x3f) + window);
            if (reg < (uint8_t) (bank + 64))
                ret = dev->rampage_map[reg];
            break;
        }

        case 0x0001:
        case 0x4001:
        case 0x8001:
        case 0xc001:
            if (off == 0x4001)
                ret = rampage_4259_readback(dev);
            else if (off == 0x8001)
                ret = rampage_eems_status(dev);
            else
                ret = dev->rampage_map_select;
            break;
        default:
            break;
    }

    isamem_log(dev->log,
               "[%04X:%08X] RAMpage read(%04X) = %02X sel=%02X board=%02X status=%02X\n",
               CS, cpu_state.pc, port, ret, dev->rampage_map_select,
               dev->rampage_board_select, rampage_eems_status(dev));
    return ret;
}

static void
rampage_eems_out(uint16_t port, uint8_t val, void *priv)
{
    memdev_t *dev = (memdev_t *) priv;
    const uint16_t base = dev->base_addr[0];
    const uint16_t off = (uint16_t) (port - base);

    switch (off) {
        case 0x0000:
        case 0x4000:
        case 0x8000:
        case 0xc000: {
            const uint8_t window = (uint8_t) (off >> 14);
            const uint8_t bank = rampage_selector_bank(dev);
            const uint8_t reg = (uint8_t) (bank + (dev->rampage_map_select & 0x3f) + window);
            if (reg < (uint8_t) (bank + 64)) {
                dev->rampage_map[reg] = val;
                dev->ems[window].raw_page = val;
                isamem_log(dev->log,
                           "[%04X:%08X] RAMpage TRACE OUT PAGE port=%04X reg=%02X val=%02X (bit7=%u page=%02X)\n",
                           CS, cpu_state.pc, port, reg, val, !!(val & 0x80), val & 0x7f);
                isamem_log(dev->log,
                           "[%04X:%08X] RAMpage map sel=%02X reg=%02X window=%u val=%02X page=%u en=%u\n",
                           CS, cpu_state.pc, dev->rampage_map_select, reg,
                           window, val, val & 0x7f, !!(val & 0x80));
                rampage_update_mappings(dev);
            }
            break;
        }

        case 0x0001:
        case 0x4001:
        case 0x8001:
        case 0xc001:
            if (off == 0x4001) {
                /* BASE+4001 is its own register, not a selector alias. AST
                   EMM 4.10 (REMM6) writes 01h/00h here around every page
                   access (or calls INT 67h AX=6101h/6102h instead) and
                   reports it as "Mode Value Set"; RAMpage EMM 2.08 (REMM5)
                   writes 0-3 and then rewrites BASE+1. Treating it as an
                   alias moved the frame away in the middle of REMM6's memory
                   test. Its function is not known, so it is only latched. */
                dev->rampage_mode = val;
                isamem_log(dev->log,
                           "[%04X:%08X] RAMpage MODE WRITE port=%04X val=%02X\n",
                           CS, cpu_state.pc, port, val);
            } else if (off == 0x8001) {
                     /* BASE+8001 stores the low-nibble board latch.  The high
                         nibble reflects installed 128-KB banks. */
                isamem_log(dev->log,
                           "[%04X:%08X] RAMpage EXT WRITE port=%04X off=%04X val=%02X high=%X low=%X prior_status=%02X prior_board=%02X\n",
                           CS, cpu_state.pc, port, off, val, val >> 4, val & 0x0f,
                                    rampage_eems_status(dev), dev->rampage_board_select);
                     dev->rampage_board_select = (uint8_t) (val & 0x0f);
            } else {
                     /* BASE+1 and its odd aliases select the first logical register
                         that the four even window ports expose. */
                dev->rampage_map_select = val;
                isamem_log(dev->log,
                           "[%04X:%08X] RAMpage EXT WRITE port=%04X off=%04X val=%02X (0259 selector bit7=%u bit6=%u group=%02X)\n",
                           CS, cpu_state.pc, port, off, val, !!(val & 0x80), !!(val & 0x40),
                           val & 0x3f);
                rampage_update_mappings(dev);
            }
            break;
        default:
            break;
    }
}

static uint8_t
rampage_sw1_readback(uint16_t base_kb)
{
    uint8_t sw15 = 1;
    uint8_t sw16 = 1;

    if (base_kb > 0) {
        if (base_kb <= 256) {
            sw15 = 0;
            sw16 = 1;
        } else if (base_kb <= 512) {
            sw15 = 1;
            sw16 = 0;
        } else {
            sw15 = 0;
            sw16 = 0;
        }
    }

    return (uint8_t) ((1U << 4) | (1U << 5) | (sw15 << 6) | (sw16 << 7));
}

static uint8_t
rampage_v1_4259_readback(uint16_t start_kb, uint16_t base_kb)
{
    uint8_t ret = 0xc0;

    switch (start_kb) {
        case 256:
            ret |= 0x10;
            break;
        case 512:
            ret |= 0x20;
            break;
        case 640:
            ret |= 0x30;
            break;
        default:
            break;
    }

    switch (base_kb) {
        case 256:
            ret = (uint8_t) ((ret & 0x3f) | 0x80);
            break;
        case 512:
            ret = (uint8_t) ((ret & 0x3f) | 0x40);
            break;
        case 768:
            ret &= 0x3f;
            break;
        default:
            break;
    }

    return ret;
}

static uint8_t
rampage_xt_switch_readback(uint16_t base, uint16_t start_kb, uint16_t base_kb)
{
    return (uint8_t) (rampage_xt_base_readback(base) |
                      rampage_v1_4259_readback(start_kb, base_kb));
}

static uint8_t
rampage_sw2_readback(uint16_t start_kb)
{
    return (uint8_t) (((start_kb / 64U) & 0x0fU) << 4);
}

/* Why this convoluted setup with the mem_dev stuff when it's much simpler
   to just pass the exec pointer as p as well, and then just use that. */
/* Read one byte from onboard RAM. */
static uint8_t
ram_readb(uint32_t addr, void *priv)
{
    ext_ram_t *dev = (ext_ram_t *) priv;
    uint8_t    ret = 0xff;

    /* Grab the data. */
    ret = *(uint8_t *) (dev->ptr + (addr - dev->base));

    return ret;
}

/* Read one word from onboard RAM. */
static uint16_t
ram_readw(uint32_t addr, void *priv)
{
    ext_ram_t *dev = (ext_ram_t *) priv;
    uint16_t   ret = 0xffff;

    /* Grab the data. */
    ret = *(uint16_t *) (dev->ptr + (addr - dev->base));

    return ret;
}

/* Write one byte to onboard RAM. */
static void
ram_writeb(uint32_t addr, uint8_t val, void *priv)
{
    ext_ram_t *dev = (ext_ram_t *) priv;

    /* Write the data. */
    *(uint8_t *) (dev->ptr + (addr - dev->base)) = val;
}

/* Write one word to onboard RAM. */
static void
ram_writew(uint32_t addr, uint16_t val, void *priv)
{
    ext_ram_t *dev = (ext_ram_t *) priv;

    /* Write the data. */
    *(uint16_t *) (dev->ptr + (addr - dev->base)) = val;
}

/* Read one byte from onboard paged RAM. */
static uint8_t
ems_readb(uint32_t addr, void *priv)
{
    emsreg_t *dev = (emsreg_t *) priv;
    uint8_t   ret = 0xff;

    /* Grab the data. */
    ret = *(uint8_t *) (dev->addr + (addr & 0x3fff));
#if ISAMEM_DEBUG
    if ((addr % 4096) == 0)
        isamem_log(dev->log, "[%04X:%08X] EMS readb(%06x) = %02x\n", CS, cpu_state.pc, addr & 0x3fff, ret);
#endif

    return ret;
}

/* Read one word from onboard paged RAM. */
static uint16_t
ems_readw(uint32_t addr, void *priv)
{
    emsreg_t *dev = (emsreg_t *) priv;
    uint16_t  ret = 0xffff;

    /* Grab the data. */
    ret = *(uint16_t *) (dev->addr + (addr & 0x3fff));
#if ISAMEM_DEBUG
    if ((addr % 4096) == 0)
        isamem_log(dev->log, "[%04X:%08X] EMS readw(%06x) = %04x\n", CS, cpu_state.pc, addr & 0x3fff, ret);
#endif

    return ret;
}

/* Write one byte to onboard paged RAM. */
static void
ems_writeb(uint32_t addr, uint8_t val, void *priv)
{
    emsreg_t *dev = (emsreg_t *) priv;

    /* Write the data. */
#if ISAMEM_DEBUG
    if ((addr % 4096) == 0)
        isamem_log(dev->log, "[%04X:%08X] EMS writeb(%06x, %02x)\n", CS, cpu_state.pc, addr & 0x3fff, val);
#endif
    *(uint8_t *) (dev->addr + (addr & 0x3fff)) = val;
}

/* Write one word to onboard paged RAM. */
static void
ems_writew(uint32_t addr, uint16_t val, void *priv)
{
    emsreg_t *dev = (emsreg_t *) priv;

    /* Write the data. */
#if ISAMEM_DEBUG
    if ((addr % 4096) == 0)
        isamem_log(dev->log, "[%04X:%08X] EMS writew(%06x, %04x)\n", CS, cpu_state.pc, addr & 0x3fff, val);
#endif
    *(uint16_t *) (dev->addr + (addr & 0x3fff)) = val;
}

/*
 * Address of the 16 KB window controlled by mapping register <vpage>.
 *
 * On the Intel-style boards (Above Board gen 1, Everex, BocaRAM, ...) the
 * register is selected by address bits A15:A14 of the I/O port, and it
 * drives whichever of the frame's four windows has the same A15:A14. The
 * frame need not be 64 KB aligned (Everex/Intel default C400h), so the
 * register for window 0 is not necessarily register 0:
 *     frame C400:  reg 1 -> C4000, reg 2 -> C8000, reg 3 -> CC000, reg 0 -> D0000
 * For a 64 KB aligned frame this is the same as frame + vpage * 16 KB.
 */
static uint32_t
ems_window_addr(uint32_t frame, int vpage)
{
    return frame + ((uint32_t) ((vpage - (int) (frame >> 14)) & 3) << 14);
}

static void
ems_map_page(emsreg_t *dev, uint8_t val, int vpage, int origport)
{
    dev->enabled = (val & 0x80);
    dev->page    = (val & 0x7f);

    if (dev->enabled && (dev->page < *dev->ems_pages)) {
        dev->addr = dev->ram + ((val & 0x7f) * EMS_PGSIZE);

        isamem_log(dev->log, "[%04X:%08X] (e) map port %04X, page %i, starting at %08X: %08X -> %08X\n",
                   CS, cpu_state.pc,
                   origport & (EMS_PGSIZE - 1), vpage, *dev->frame_addr,
                   ems_window_addr(*dev->frame_addr, vpage), dev->addr - dev->ram);
        mem_mapping_set_addr(&dev->mapping, ems_window_addr(*dev->frame_addr, vpage), EMS_PGSIZE);
        mem_mapping_set_exec(&dev->mapping, dev->addr);
        mem_mapping_enable(&dev->mapping);
    } else {
        isamem_log(dev->log, "[%04X:%08X] (d) map port %04X, page %i, starting at %08X: %08X -> N/A\n",
                   CS, cpu_state.pc,
                   origport & (EMS_PGSIZE - 1), vpage, *dev->frame_addr,
                   ems_window_addr(*dev->frame_addr, vpage));
        mem_mapping_disable(&dev->mapping);
    }
}

#if 0
static void
rampage_ems_map_page(emsreg_t *dev, uint8_t val, int vpage, int origport)
{
    dev->enabled = (val & 0x80);
    dev->page    = (val & 0x7f);

    uint32_t target_addr = *dev->frame_addr + (EMS_PGSIZE * vpage);

    if (dev->enabled && (dev->page < *dev->ems_pages)) {
        dev->addr = dev->ram + ((val & 0x7f) * EMS_PGSIZE);

        isamem_log(dev->log, "[%04X:%08X] (e) map port %04X, page %i, starting at %08X: %08X -> %08X\n",
                   CS, cpu_state.pc,
                   origport & (EMS_PGSIZE - 1), vpage, *dev->frame_addr,
                   target_addr, dev->addr - dev->ram);
                   
        mem_mapping_set_addr(&dev->mapping, target_addr, EMS_PGSIZE);
        mem_mapping_set_exec(&dev->mapping, dev->addr);
        mem_mapping_enable(&dev->mapping);
    } else {
        isamem_log(dev->log, "[%04X:%08X] (d) map port %04X, page %i, starting at %08X: %08X -> N/A\n",
                   CS, cpu_state.pc,
                   origport & (EMS_PGSIZE - 1), vpage, *dev->frame_addr,
                   target_addr);
                   
        mem_mapping_disable(&dev->mapping);
    }
}
#endif

/* Handle a READ operation from one of our registers. */
static uint8_t
ems_in(uint16_t port, void *priv)
{
    const emsreg_t *dev   = (emsreg_t *) priv;
    uint8_t         ret   = 0xff;
    /* Get the viewport page number. */
#ifdef ENABLE_ISAMEM_LOG
    int             vpage = (port / EMS_PGSIZE);
#endif

    port &= (EMS_PGSIZE - 1);

    switch (port & 0x0001) {
        case 0x0000: /* page number register */
            ret = dev->page;
            if (dev->enabled)
                ret |= 0x80;
            break;

        case 0x0001: /* W/O */
            break;

        default:
            break;
    }

    isamem_log(dev->log, "[%04X:%08X] read(%04x) = %02x) page=%d\n", CS, cpu_state.pc, port, ret, vpage);

    return ret;
}

/* Handle a READ operation from one of our registers. */
static uint8_t
consecutive_ems_in(uint16_t port, void *priv)
{
    const memdev_t *dev   = (memdev_t *) priv;
    uint8_t         ret   = 0xff;
    /* Get the viewport page number. */
    int             vpage = (port - dev->base_addr[0]);

    ret = dev->ems[vpage].page;
    if (dev->ems[vpage].enabled)
        ret |= 0x80;

    isamem_log(dev->log, "[%04X:%08X] read(%04x) = %02x) page=%d\n", CS, cpu_state.pc, port, ret, vpage);

    return ret;
}

#ifdef IAB_5170_HACK
/* Some EMS drivers sample XT/AT diagnostic ports and treat bit 6 as a hard
    failure signal after page tests. On machines where the probed port is
    otherwise unclaimed, 86Box returns 0xff and every tested bank looks bad.
    Force only that bit low and let read-handler combining preserve any other
    device-owned bits. */
static uint8_t
ems_diag_in(UNUSED(uint16_t port), UNUSED(void *priv))
{
    return 0xbf;
}
#endif

/* Handle a WRITE operation to one of our registers. */
static void
ems_out(uint16_t port, uint8_t val, void *priv)
{
    emsreg_t *dev   = (emsreg_t *) priv;
    /* Get the viewport page number. */
    int       vpage = (port / EMS_PGSIZE);

    int       origport = port;
    port &= (EMS_PGSIZE - 1);

    switch (port & 0x0001) {
        case 0x0000: /* page mapping registers */
            ems_map_page(dev, val, vpage, origport);
            break;

        case 0x0001: /* page frame registers */
            /*
             * The EV-159 EMM driver configures the frame address
             * by setting bits in these registers. The information
             * in their manual is unclear, but here is what was
             * found out by repeatedly changing EMM's config:
             *
             * 08 04 00  Address
             * -----------------
             * 00 00 00  C4000
             * 00 00 80  C8000
             * 00 80 00  CC000
             * 00 80 80  D0000
             * 80 00 00  D4000
             * 80 00 80  D8000
             * 80 80 00  DC000
             * 80 80 80  E0000
             */
            dev->frame = val;
            *dev->frame_val = (*dev->frame_val & ~(1 << vpage)) | ((val >> 7) << vpage);
            *dev->frame_addr = 0x000c4000 + (*dev->frame_val << 14);
            isamem_log(dev->log, "[%04X:%08X] map port %04X page %i: frame_addr = %08X\n", CS, cpu_state.pc, port, vpage, *dev->frame_addr);
            /* The board decodes the frame on every access, so windows that
               are already mapped move with it. */
            if (dev->owner != NULL) {
                emsreg_t *set = &dev->owner->ems[(dev - dev->owner->ems) & EMS_MAXPAGE];

                for (int i = 0; i < EMS_MAXPAGE; i++)
                    if (set[i].enabled && (set[i].page < *set[i].ems_pages))
                        mem_mapping_set_addr(&set[i].mapping, ems_window_addr(*dev->frame_addr, i), EMS_PGSIZE);
            }
#if 0
            /* Destroy the page registers. */
            for (uint8_t i = 0; i < 4; i ++) {
                isamem_log(dev->log, "    ");
                outb((port & 0x3ffe) + (i << 14), 0x00);
            }
#endif
            break;

        default:
            break;
    }
}


/* Handle a WRITE operation to one of our registers. */
static void
consecutive_ems_out(uint16_t port, uint8_t val, void *priv)
{
    memdev_t *dev   = (memdev_t *) priv;
    /* Get the viewport page number. */
    int       vpage = (port - dev->base_addr[0]);

    isamem_log(dev->log, "[%04X:%08X] write(%04x, %02x) to page mapping registers! (page=%d)\n", CS, cpu_state.pc, port, val, vpage);

    /* Set the page number. */
    dev->ems[vpage].enabled = 1;
    dev->ems[vpage].page    = val;

    /* Make sure we can do that.. FFh is the Lo-tech "window disabled" value
       (LTEMM DIS_EMS), even on a 4 MB board where page 255 would exist. */
    if (dev->flags & FLAG_CONFIG) {
        if ((val != 0xff) && (dev->ems[vpage].page < dev->ems_pages[0])) {
            /* Pre-calculate the page address in EMS RAM. */
            dev->ems[vpage].addr = dev->ram + dev->ems_start[0] + (val * EMS_PGSIZE);
        } else {
            /* That page does not exist. */
            dev->ems[vpage].enabled = 0;
        }

        if (dev->ems[vpage].enabled) {
            /* Update the EMS RAM address for this page. */
            mem_mapping_set_exec(&dev->ems[vpage].mapping,
                                 dev->ems[vpage].addr);

            /* Enable this page. */
            mem_mapping_enable(&dev->ems[vpage].mapping);
        } else {
            /* Disable this page. */
            mem_mapping_disable(&dev->ems[vpage].mapping);
        }
    }
}

/*
 * XTMax (MicroCoreLabs) memory manager, EMS part only.
 *
 * Register map, from XTMax.ino (rev 10) and the XTEMM driver:
 *   BASE+0..7  four 16-bit EMS page pointers, low byte first (the driver
 *              uses OUT DX,AX to BASE+2n). Read/write. Reset value FFFFh.
 *              A pointer selects the 16 KB page at pointer << 14; pages
 *              beyond the installed RAM read FFh and ignore writes, which
 *              is how FFFFh "disables" a window.
 *   BASE+A/B   EMS window segment (write only).
 *   BASE+C     write: number of 16 KB windows, and commit. Accepted only
 *              if segment >= A000h and segment + n*400h <= F000h.
 *   BASE+D/E   UMB base segment (write only). XTEMM writes the window
 *              segment here and reads BASE+F to verify the commit.
 *   BASE+F     read: region type at the BASE+D/E segment
 *              (0 unused, 1 RAM, 2 EMS window). Write: UMB commit - the
 *              UMB feature is not emulated, the write is ignored.
 * The firmware decodes window n as (segment << 4) | (n << 14), so the
 * segment should be 64 KB aligned; XTEMM's /P: values are.
 */
static void
xtmax_update_window(memdev_t *dev, int i)
{
    emsreg_t      *reg  = &dev->ems[i];
    const uint32_t page = dev->xtmax_ptr[i];

    if (i >= dev->xtmax_count) {
        reg->enabled = 0;
        mem_mapping_disable(&reg->mapping);
        return;
    }

    mem_mapping_set_addr(&reg->mapping,
                         ((uint32_t) dev->xtmax_win_seg << 4) | ((uint32_t) i * EMS_PGSIZE),
                         EMS_PGSIZE);

    if (page < dev->ems_pages[0]) {
        reg->enabled = 1;
        reg->addr    = dev->ram + dev->ems_start[0] + (page * EMS_PGSIZE);
        mem_mapping_set_exec(&reg->mapping, reg->addr);
    } else {
        reg->enabled = 0;
        reg->addr    = NULL;
        mem_mapping_disable(&reg->mapping);
    }
}

static uint8_t
xtmax_ems_in(uint16_t port, void *priv)
{
    const memdev_t *dev = (memdev_t *) priv;
    const int       reg = port - dev->base_addr[0];
    uint8_t         ret = 0xff;

    if (reg < 8)
        ret = (reg & 1) ? (dev->xtmax_ptr[reg >> 1] >> 8) : (dev->xtmax_ptr[reg >> 1] & 0xff);
    else if (reg == 15) {
        /* Region type at the UMB/debug segment, in 2 KB blocks. */
        const uint32_t blk  = dev->xtmax_umb_seg >> 7;
        const uint32_t from = dev->xtmax_win_seg >> 7;
        ret = (dev->xtmax_count && (blk >= from) && (blk < from + dev->xtmax_count * 8)) ? 2 : 0;
    }

    isamem_log(dev->log, "XTMax: in  %04X = %02X\n", port, ret);

    return ret;
}

static void
xtmax_ems_out(uint16_t port, uint8_t val, void *priv)
{
    memdev_t *dev = (memdev_t *) priv;
    const int reg = port - dev->base_addr[0];

    isamem_log(dev->log, "XTMax: out %04X, %02X\n", port, val);

    switch (reg) {
        case 0: case 1: case 2: case 3:
        case 4: case 5: case 6: case 7:
            if (reg & 1)
                dev->xtmax_ptr[reg >> 1] = (dev->xtmax_ptr[reg >> 1] & 0x00ff) | (val << 8);
            else
                dev->xtmax_ptr[reg >> 1] = (dev->xtmax_ptr[reg >> 1] & 0xff00) | val;
            xtmax_update_window(dev, reg >> 1);
            break;

        case 10:
            dev->xtmax_seg = (dev->xtmax_seg & 0xff00) | val;
            break;
        case 11:
            dev->xtmax_seg = (dev->xtmax_seg & 0x00ff) | (val << 8);
            break;

        case 12:
            if ((dev->xtmax_seg >= 0xa000) && (((uint32_t) dev->xtmax_seg + ((uint32_t) val << 10)) <= 0xf000)) {
                if (dev->xtmax_seg & 0x0fff)
                    isamem_log(dev->log, "XTMax: window segment %04X is not 64K aligned\n", dev->xtmax_seg);
                dev->xtmax_win_seg = dev->xtmax_seg;
                dev->xtmax_count   = (val > EMS_MAXPAGE) ? EMS_MAXPAGE : val;
                dev->frame_addr[0] = (uint32_t) dev->xtmax_win_seg << 4;
                isamem_log(dev->log, "XTMax: EMS window %05X, %d page(s)\n", dev->frame_addr[0], dev->xtmax_count);
                for (int i = 0; i < EMS_MAXPAGE; i++)
                    xtmax_update_window(dev, i);
            }
            break;

        case 13:
            dev->xtmax_umb_seg = (dev->xtmax_umb_seg & 0xff00) | val;
            break;
        case 14:
            dev->xtmax_umb_seg = (dev->xtmax_umb_seg & 0x00ff) | (val << 8);
            break;

        default: /* BASE+F UMB commit and unused registers */
            break;
    }
}

/* Initialize the device for use. */
static void *
isamem_init(const device_t *info)
{
    memdev_t *dev;
    uint32_t  k;
    uint32_t  t;
    uint32_t  addr;
    uint32_t  tot;
    /* EMS 3.2 cannot have more than 2048KB per board. */
    uint32_t  ems_max = EMS_MAXSIZE;
    uint8_t  *ptr;

    /* Find our device and create an instance. */
    dev = (memdev_t *) calloc(1, sizeof(memdev_t));
    dev->name  = info->name;
    dev->board = info->local;

#ifdef ENABLE_ISAMEM_LOG
    dev->log = log_open("ISAMEM");
#endif

    dev->base_addr[1]  = 0x0000;
    dev->frame_addr[1] = 0x00000000;

    /* Default RAMpage/EEMS state.  A fully populated original Rampage has
       32 alternate mapping-register sets; the driver derives this from the
       high nibble of BASE+8001. */
    dev->rampage_map_select = 0xB8;
    dev->rampage_board_select = 0;
    memset(dev->rampage_map, 0, sizeof(dev->rampage_map));

    /* Do per-board initialization. */
    tot = 0;
    switch (dev->board) {
        case ISAMEM_IBMXT_CARD:      /* IBM PC/XT Memory Expansion Card */
        case ISAMEM_GENXT_CARD:      /* Generic PC/XT Memory Expansion Card */
        case ISAMEM_RAMCARD_CARD:    /* Microsoft RAMCard for IBM PC */
        case ISAMEM_SYSTEMCARD_CARD: /* Microsoft SystemCard */
        case ISAMEM_P5PAK_CARD:      /* Paradise Systems 5-PAK */
        case ISAMEM_A6PAK_CARD:      /* AST SixPakPlus */
        case ISAMEM_MPLUS2_CARD:     /* AST MegaPlus II */
            dev->total_size = device_get_config_int("size");
            dev->start_addr = device_get_config_int("start");
            tot             = dev->total_size;
            break;

        case ISAMEM_IBMPCJR_CARD: /* IBM PCjr 128KB Memory Expansion */
            dev->total_size = 128;
            dev->start_addr = device_get_config_int("start");
            tot             = dev->total_size;
            break;

        case ISAMEM_GENPCJR_CARD: /* Generic PCjr Memory Expansion */
            dev->total_size = (uint16_t) device_get_config_int("size");
            dev->start_addr = device_get_config_int("start");
            tot             = dev->total_size;
            break;

        case ISAMEM_JRIDE_CARD: /* jr-IDE Memory Expansion */
            dev->total_size = 512;
            dev->start_addr = 128;
            tot             = dev->total_size;
            break;

        case ISAMEM_IBMAT_128K_CARD: /* IBM PC/AT 128K Memory Expansion Option */
            dev->total_size = 128;
            dev->start_addr = 512;
            tot             = dev->total_size;
            dev->flags |= FLAG_WIDE;
            break;

        case ISAMEM_IBMAT_CARD: /* IBM PC/AT Memory Expansion Card */
        case ISAMEM_GENAT_CARD: /* Generic PC/AT Memory Expansion Card */
            dev->total_size = device_get_config_int("size");
            dev->start_addr = device_get_config_int("start");
            tot             = dev->total_size;
            dev->flags |= FLAG_WIDE;
            break;

        case ISAMEM_EMS5150_CARD: /* Micro Mainframe EMS-5150(T) */
            dev->base_addr[0]  = device_get_config_hex16("base");
            dev->total_size    = device_get_config_int("size");
            dev->start_addr    = 0;
            dev->frame_addr[0] = 0xd0000;
            dev->flags        |= (FLAG_EMS | FLAG_CONFIG);
            break;

        case ISAMEM_EV159_CARD: /* Everex EV-159 RAM 3000 */
            /* The EV-159 cannot have more than 3072KB per board. */
            ems_max = EMS_EV159_MAXSIZE;
            dev->base_addr[0]  = device_get_config_hex16("base");
            dev->base_addr[1]  = device_get_config_hex16("base2");
            dev->total_size    = device_get_config_int("size");
            dev->start_addr    = device_get_config_int("start");
            tot                = device_get_config_int("length");
            if (!!device_get_config_int("width"))
                dev->flags    |= FLAG_WIDE;
            if (!!device_get_config_int("speed"))
                dev->flags    |= FLAG_FAST;
            if (!!device_get_config_int("ems"))
                dev->flags    |= FLAG_EMS;
            dev->frame_addr[0] = 0xd0000;
            dev->frame_addr[1] = 0xe0000;
            break;

        case ISAMEM_EV165A_CARD: /* Everex Maxi Magic EV-165A */
            dev->base_addr[0]  = device_get_config_hex16("base");
            dev->total_size    = device_get_config_int("size");
            dev->start_addr    = device_get_config_int("start");
            if ((dev->start_addr >= 256) && (dev->start_addr < 640))
                tot                = device_get_config_int("length");
            if (!!device_get_config_int("ems"))
                dev->flags    |= FLAG_EMS;
            dev->frame_addr[0] = 0xe0000;
            break;

        case ISAMEM_RAMPAGEXT_CARD:   /* AST RAMpage/XT */
        case ISAMEM_RAMPAGEXTV1_CARD: /* AST RAMpage/XT (/V=1 family) */
            dev->base_addr[0]  = device_get_config_hex16("base");
            dev->total_size    = device_get_config_int("size");
            dev->start_addr    = device_get_config_int("start");
            if (dev->start_addr < 640)
                tot                = device_get_config_int("length");
            dev->flags        |= (FLAG_EMS | FLAG_RAMPAGE_DUAL);
            dev->frame_addr[0] = 0xe0000;
            if (dev->board == ISAMEM_RAMPAGEXTV1_CARD)
                dev->rampage_switch_hi[1] = rampage_xt_switch_readback(dev->base_addr[0],
                                                                       (uint16_t) dev->start_addr,
                                                                       (uint16_t) tot);
            else
                dev->rampage_switch_hi[1] = rampage_sw2_readback((uint16_t) dev->start_addr);
            dev->rampage_switch_hi[2] = rampage_sw1_readback((uint16_t) tot);
            /* "In some cases, the total of existing PC memory and Rampage
               conventional memory may add up to more than 640K. In this
               situation, conventional memory stops at 640K and the balance
               becomes available as expanded memory" (AST switch notes for
               the RAMpage! and SixPakPremium). The switch readbacks above
               keep the setting as configured. */
            if ((dev->start_addr + tot) > 640)
                tot = (dev->start_addr < 640) ? (640 - dev->start_addr) : 0;
            if (!device_get_config_int("dual_page"))
                dev->flags &= ~FLAG_RAMPAGE_DUAL;
//              dev->rampage_switch_hi[1] = 0b11111111;
//             SW1-1   SW1-2   SW1-3   SW1-4
//        0K   OFF     OFF     OFF     OFF
//       64K    ON     OFF     OFF     OFF
//      128K   OFF      ON     OFF     OFF
//      192K    ON      ON     OFF     OFF
//      256K   OFF     OFF      ON     OFF
//      320K    ON     OFF      ON     OFF
//      384K   OFF      ON      ON     OFF
//      448K    ON      ON      ON     OFF
//      512K   OFF     OFF     OFF      ON
//      576K    ON     OFF     OFF      ON
//      640K   OFF      ON     OFF      ON
//               1111 = Invalid
//              dev->rampage_switch_hi[2] = 0b11110100;
            break;

        case ISAMEM_RAMPAGEAT_CARD: /* AST Rampage AT */
            dev->base_addr[0]  = device_get_config_hex16("base");
            dev->total_size    = device_get_config_int("size");
            dev->start_addr    = device_get_config_int("start");
            if ((dev->start_addr < 640) ||
                ((dev->start_addr >= 1024) && (dev->start_addr < 15360)))
                tot                = device_get_config_int("length");
            dev->flags        |= (FLAG_EMS | FLAG_WIDE | FLAG_RAMPAGE_DUAL);
            dev->frame_addr[0] = device_get_config_hex20("frame");
            dev->rampage_linear_kb    = (uint16_t) tot;
            dev->rampage_switch_hi[1] = rampage_at_sw2_readback((uint16_t) dev->start_addr, (uint16_t) tot);
            if (!device_get_config_int("dual_page"))
                dev->flags &= ~FLAG_RAMPAGE_DUAL;
            break;

        case ISAMEM_BRAT_CARD:       /* BocaRAM/AT */
            dev->base_addr[0]   = device_get_config_hex16("base");
            dev->total_size     = device_get_config_int("size");
            if (!!device_get_config_int("start"))
                dev->start_addr = device_get_config_int("start");
            if ((dev->start_addr >= 1024) && (dev->start_addr < 14336))
                tot                = device_get_config_int("length");
            dev->frame_addr[0]  = device_get_config_hex20("frame");
            dev->flags         |= FLAG_EMS;
            /* "width" holds 8 or 16, so test the value, not just non-zero. */
            if (device_get_config_int("width") == 16)
                dev->flags     |= FLAG_WIDE;
            if (!!device_get_config_int("speed"))
                dev->flags     |= FLAG_FAST;
            break;

        case ISAMEM_XTMAX_EMS_CARD: /* XTMax, EMS part only */
            ems_max = EMS_XTMAX_MAXSIZE;
            dev->base_addr[0]   = device_get_config_hex16("base");
            dev->total_size     = device_get_config_int("size");
            dev->start_addr     = 0;
            /* The window is placed by the driver (BASE+A..C). */
            dev->frame_addr[0]  = 0;
            for (uint8_t i = 0; i < EMS_MAXPAGE; i++)
                dev->xtmax_ptr[i] = 0xffff;
            dev->flags         |= (FLAG_EMS | FLAG_CONFIG);
            break;

        case ISAMEM_LOTECH_EMS_CARD: /* Lotech EMS */
            /* The Lotech EMS cannot have more than 4096KB per board. */
            ems_max = EMS_LOTECH_MAXSIZE;
            fallthrough;
        case ISAMEM_BRXT_CARD:   /* BocaRAM/XT */
            dev->base_addr[0]   = device_get_config_hex16("base");
            dev->total_size     = device_get_config_int("size");
            dev->start_addr     = 0;
            dev->frame_addr[0]  = device_get_config_hex20("frame");
            dev->flags         |= (FLAG_EMS | FLAG_CONFIG);
            break;

        default:
            break;
    }

    /* Fix up the memory start address. */
    dev->start_addr <<= 10;

    /* Say hello! */
    char log_suffix[48] = "";

    if (tot && (dev->total_size != tot))
        snprintf(log_suffix + strlen(log_suffix),
                 sizeof(log_suffix) - strlen(log_suffix),
                 ", %iKB for RAM", tot);
    if (dev->flags & FLAG_FAST)
        snprintf(log_suffix + strlen(log_suffix),
                 sizeof(log_suffix) - strlen(log_suffix),
                 ", FAST");
    if (dev->flags & FLAG_WIDE)
        snprintf(log_suffix + strlen(log_suffix),
                 sizeof(log_suffix) - strlen(log_suffix),
                 ", 16BIT");

    isamem_log(dev->log, "%s (%iKB%s)\n", info->name, dev->total_size,
               log_suffix);

    /* Force (back to) 8-bit bus if needed. */
    if ((!is286) && (dev->flags & FLAG_WIDE)) {
        isamem_log(dev->log, "not AT+ system, forcing 8-bit mode!\n");
        dev->flags &= ~FLAG_WIDE;
    }

    /* Allocate and initialize our RAM. */
    k        = dev->total_size << 10;
    dev->ram = (uint8_t *) calloc(1, k);
    ptr = dev->ram;

    /*
     * The 'Memory Start Address' switch indicates at which address
     * we should start adding memory. No memory is added if it is
     * set to 0.
     */
    tot <<= 10;
    addr = dev->start_addr;
    if (addr > 0 && tot > 0) {
        /* Adjust K for the RAM we will use. */
        k -= tot;

        /*
         * First, see if we have to expand the conventional
         * (low) memory area. This can extend up to 640KB,
         * so check this first.
         */
        t = (addr < RAM_TOPMEM) ? RAM_TOPMEM - addr : 0;

        /* Check for Everex EV-159 cards in CS8220 backfill mode. */
        if ((addr == RAM_TOPMEM) && (dev->board == ISAMEM_EV159_CARD)) {
            /* Reserve 128K RAM for base memory. */
            t = EV159_BASE_MEM;
            addr -= t;
        }

        if (t > 0) {
            /*
             * We need T bytes to extend that area.
             *
             * If the board doesn't have that much, grab
             * as much as we can.
             */
            if (t > tot)
                t = tot;
            isamem_log(dev->log, "RAM at %05iKB (%iKB)\n", addr >> 10, t >> 10);

            dev->ext_ram[EXTRAM_CONVENTIONAL].ptr  = ptr;
            dev->ext_ram[EXTRAM_CONVENTIONAL].base = addr;

            /* Create, initialize and enable the low-memory mapping. */
            mem_mapping_add(&dev->low_mapping, addr, t,
                            ram_readb,
                            (dev->flags & FLAG_WIDE) ? ram_readw : NULL,
                            NULL,
                            ram_writeb,
                            (dev->flags & FLAG_WIDE) ? ram_writew : NULL,
                            NULL,
                            ptr, MEM_MAPPING_EXTERNAL, &dev->ext_ram[EXTRAM_CONVENTIONAL]);

            /* Tell the memory system this is external RAM. */
            mem_set_mem_state(addr, t,
                              MEM_READ_EXTERNAL | MEM_WRITE_EXTERNAL);

            /* Update pointers. */
            ptr += t;
            tot -= t;
            addr += t;
        }

        /* Assign high memory address for EV-159 in backfill modes. */
        if ((addr == RAM_TOPMEM) && (dev->board == ISAMEM_EV159_CARD)) {
            if (dev->start_addr == RAM_TOPMEM) {
                addr = EV159_EXT_1536;
            } else {
                addr = EV159_EXT_1024;
            }
        }

        /* Skip to high memory if needed. */
        if ((addr == RAM_TOPMEM) && (tot >= RAM_UMAMEM)) {
            /*
             * We have more RAM available, but we are at the
             * top of conventional RAM. So, the next 384K are
             * skipped, and placed into different mappings so
             * they can be re-mapped later.
             */
            t = RAM_UMAMEM; /* 384KB */

            isamem_log(dev->log, "RAM at %05iKB (%iKB)\n", addr >> 10, t >> 10);

            dev->ext_ram[EXTRAM_HIGH].ptr  = ptr;
            dev->ext_ram[EXTRAM_HIGH].base = addr + tot;

            /* Update and enable the remap. */
            mem_mapping_set(&ram_remapped_mapping,
                            addr + tot, t,
                            ram_readb, ram_readw, NULL,
                            ram_writeb, ram_writew, NULL,
                            ptr, MEM_MAPPING_EXTERNAL,
                            &dev->ext_ram[EXTRAM_HIGH]);
            mem_mapping_disable(&ram_remapped_mapping);

            /* Tell the memory system this is external RAM. */
            mem_set_mem_state(addr + tot, t,
                              MEM_READ_EXTERNAL | MEM_WRITE_EXTERNAL);

            /* Update pointers. */
            ptr += t;
            tot -= t;
            addr += t;
        }
    }

    /*
     * Next, on systems that support it (80286 and up), we can add
     * (some of) our RAM to the system as Extended Memory, that is,
     * memory located above 1MB. This memory cannot be addressed in
     * real mode (so, not by DOS, for example) but it can be used in
     * protected mode.
     */
    if (is286 && addr > 0 && tot > 0) {
        t = tot;
        isamem_log(dev->log, "RAM at %05iKB (%iKB)\n", addr >> 10, t >> 10);

        dev->ext_ram[EXTRAM_XMS].ptr  = ptr;
        dev->ext_ram[EXTRAM_XMS].base = addr;

        /* Create, initialize and enable the high-memory mapping. */
        mem_mapping_add(&dev->high_mapping, addr, t,
                        ram_readb, ram_readw, NULL,
                        ram_writeb, ram_writew, NULL,
                        ptr, MEM_MAPPING_EXTERNAL, &dev->ext_ram[EXTRAM_XMS]);

        /* Tell the memory system this is external RAM. */
        mem_set_mem_state(addr, t, MEM_READ_EXTERNAL | MEM_WRITE_EXTERNAL);

        /* Update pointers. */
        ptr += t;
        tot -= t;
        addr += t;
    }

    isa_mem_size += dev->total_size - (k >> 10);

    /* If EMS is enabled, use the remainder for EMS. */
    if (dev->flags & FLAG_EMS) {
        t = k;

        if (t > ems_max)
            t = ems_max;

        /* Set up where EMS begins in local RAM, and how much we have. */
        dev->ems_start[0]     = ptr - dev->ram;
        if (dev->board == ISAMEM_RAMPAGEAT_CARD) {
            /* RAMpage AT: EMS page numbers cover the whole board, and the
               linear (conventional/extended) memory is board pages 0..n-1.
               AST EMM 4.10 skips those itself (8001 high nibble), so EMS
               must not start after the linear block or it is lost twice. */
            dev->ems_start[0] = 0;
            dev->ems_size[0]  = dev->total_size;
            dev->ems_pages[0] = (uint16_t) (((uint32_t) dev->total_size << 10) / EMS_PGSIZE);
        } else if ((dev->board == ISAMEM_EV159_CARD) && (t > (2 << 20))) {
            dev->ems_size[0]  = 2 << 10;
            dev->ems_pages[0] = (2 << 20) / EMS_PGSIZE;
        } else {
            dev->ems_size[0]  = t >> 10;
            dev->ems_pages[0] = t / EMS_PGSIZE;
        }
        isamem_log(dev->log, "EMS #1 enabled, I/O=%04XH, %iKB (%i pages)",
                   dev->base_addr[0], dev->ems_size[0], dev->ems_pages[0]);
        if (dev->frame_addr[0] > 0)
            isamem_log(dev->log, ", Frame[0]=%05XH", dev->frame_addr[0]);

        isamem_log(dev->log, "\n");

        if ((dev->board == ISAMEM_EV159_CARD) && (t > (2 << 20))) {
            dev->ems_start[1] = dev->ems_start[0] + (2 << 20);
            dev->ems_size[1]  = (t - (2 << 20)) >> 10;
            dev->ems_pages[1] = (t - (2 << 20)) / EMS_PGSIZE;
            isamem_log(dev->log, "EMS #2 enabled, I/O=%04XH, %iKB (%i pages)",
                       dev->base_addr[1], dev->ems_size[1], dev->ems_pages[1]);
            if (dev->frame_addr[1] > 0)
                isamem_log(dev->log, ", Frame[1]=%05XH", dev->frame_addr[1]);

            isamem_log(dev->log, "\n");
        }

        /*
         * For each supported page (we can have a maximum of 4),
         * create, initialize and disable the mappings, and set
         * up the I/O control handler.
         */
        for (uint8_t i = 0; i < EMS_MAXPAGE; i++) {
            dev->ems[i].ram        = dev->ram + dev->ems_start[0];
            dev->ems[i].frame_val  = &dev->frame_val[0];
            dev->ems[i].ems_size   = &dev->ems_size[0];
            dev->ems[i].ems_pages  = &dev->ems_pages[0];
            dev->ems[i].frame_addr = &dev->frame_addr[0];
            dev->ems[i].owner      = dev;
            dev->ems[i].log        = dev->log;

            /* Create and initialize a page mapping. */
            mem_mapping_add(&dev->ems[i].mapping,
                            dev->frame_addr[0] + (EMS_PGSIZE * i), EMS_PGSIZE,
                            ems_readb,
                            (dev->flags & FLAG_WIDE) ? ems_readw : NULL,
                            NULL,
                            ems_writeb,
                            (dev->flags & FLAG_WIDE) ? ems_writew : NULL,
                            NULL,
                            ptr, MEM_MAPPING_EXTERNAL,
                            &(dev->ems[i]));

            /* For now, disable it. */
            mem_mapping_disable(&dev->ems[i].mapping);

            /* Set up an I/O port handler.  RAMpage is not the same as the
               simple LIM 3.2 register pair: its four viewport ports are
               4000h apart and its odd ports include EEMS control/status. */
            if ((dev->board != ISAMEM_LOTECH_EMS_CARD) &&
                (dev->board != ISAMEM_XTMAX_EMS_CARD)) {
                if ((dev->board == ISAMEM_RAMPAGEXT_CARD) ||
                    (dev->board == ISAMEM_RAMPAGEXTV1_CARD) ||
                    (dev->board == ISAMEM_RAMPAGEAT_CARD)) {
                    io_sethandler(dev->base_addr[0] + (EMS_PGSIZE * i), 2,
                                  &rampage_eems_in, NULL, NULL,
                                  &rampage_eems_out, NULL, NULL, dev);
                } else {
                    io_sethandler(dev->base_addr[0] + (EMS_PGSIZE * i), 2,
                                  &ems_in, NULL, NULL, &ems_out,
                                  NULL, NULL, &(dev->ems[i]));
                }
            }

            if ((dev->board == ISAMEM_EV159_CARD) && (t > (2 << 20))) {
                dev->ems[i | 4].ram        = dev->ram + dev->ems_start[1];
                dev->ems[i | 4].frame_val  = &dev->frame_val[1];
                dev->ems[i | 4].ems_size   = &dev->ems_size[1];
                dev->ems[i | 4].ems_pages  = &dev->ems_pages[1];
                dev->ems[i | 4].frame_addr = &dev->frame_addr[1];
                dev->ems[i | 4].log        = dev->log;
                dev->ems[i | 4].owner      = dev;

                /* Create and initialize a page mapping. */
                mem_mapping_add(&dev->ems[i | 4].mapping,
                                dev->frame_addr[1] + (EMS_PGSIZE * i), EMS_PGSIZE,
                                ems_readb,
                                (dev->flags & FLAG_WIDE) ? ems_readw : NULL,
                                NULL,
                                ems_writeb,
                                (dev->flags & FLAG_WIDE) ? ems_writew : NULL,
                                NULL,
                                ptr + (2 << 20), MEM_MAPPING_EXTERNAL,
                                &(dev->ems[i | 4]));

                /* For now, disable it. */
                mem_mapping_disable(&dev->ems[i | 4].mapping);

                io_sethandler(dev->base_addr[1] + (EMS_PGSIZE * i), 2,
                              ems_in, NULL, NULL, ems_out, NULL, NULL, &(dev->ems[i | 4]));
            }
        }

        if (dev->board == ISAMEM_LOTECH_EMS_CARD)
            io_sethandler(dev->base_addr[0], 4,
                          consecutive_ems_in, NULL, NULL, consecutive_ems_out, NULL, NULL, dev);

        if (dev->board == ISAMEM_XTMAX_EMS_CARD)
            io_sethandler(dev->base_addr[0], 16,
                          xtmax_ems_in, NULL, NULL, xtmax_ems_out, NULL, NULL, dev);
    }

#ifdef IAB_5170_HACK
    if ((dev->board == ISAMEM_RAMPAGEXT_CARD) ||
        (dev->board == ISAMEM_RAMPAGEXTV1_CARD) ||
        (dev->board == ISAMEM_RAMPAGEAT_CARD)) {
        io_sethandler(0x0061, 1, ems_diag_in, NULL, NULL, NULL, NULL, NULL, dev);
        io_sethandler(0x0062, 1, ems_diag_in, NULL, NULL, NULL, NULL, NULL, dev);
    }
#endif

    /* Let them know our device instance. */
    return ((void *) dev);
}

/* Remove the device from the system. */
static void
isamem_close(void *priv)
{
    memdev_t *dev = (memdev_t *) priv;

#ifdef IAB_5170_HACK
    if ((dev->board == ISAMEM_RAMPAGEXT_CARD) ||
        (dev->board == ISAMEM_RAMPAGEXTV1_CARD) ||
        (dev->board == ISAMEM_RAMPAGEAT_CARD)) {
        io_removehandler(0x0061, 1, ems_diag_in, NULL, NULL, NULL, NULL, NULL, dev);
        io_removehandler(0x0062, 1, ems_diag_in, NULL, NULL, NULL, NULL, NULL, dev);
    }
#endif

    if (dev->ram != NULL)
        free(dev->ram);

#ifdef ENABLE_ISAMEM_LOG
    if (dev->log)
        log_close(dev->log);
#endif

    free(dev);
}

/*
 * Generic UMB expansion card.
 *
 * Plain RAM decoded in the upper memory area (A0000-FFFFF), with no
 * registers, as provided by simple UMB boards (and by memory cards with
 * UMA decode jumpers). DOS uses it through a UMB provider such as
 * USE!UMBS.SYS ("DEVICE=USE!UMBS.SYS D000-F000"), which only needs the
 * RAM to be writable; or DOS can be given it directly by a driver that
 * links it into the MCB chain.
 *
 * Up to two ranges, in 16 KB steps. The RAM is not cleared by a soft
 * reset, like real DRAM, and is not counted as conventional memory.
 * The ranges must not overlap option ROMs or video memory in use: the
 * card would answer as well, as real hardware would.
 */
static void *
genumb_init(const device_t *info)
{
    memdev_t *dev = (memdev_t *) calloc(1, sizeof(memdev_t));
    uint32_t  start[UMB_MAX_RANGES];
    uint32_t  len[UMB_MAX_RANGES];
    uint32_t  total = 0;
    uint8_t  *ptr;
    char      key[16];

    dev->name  = info->name;
    dev->board = info->local;
#ifdef ENABLE_ISAMEM_LOG
    dev->log = log_open("ISAMEM");
#endif

    if (device_get_config_int("width") == 16)
        dev->flags |= FLAG_WIDE;
    if ((!is286) && (dev->flags & FLAG_WIDE)) {
        isamem_log(dev->log, "not AT+ system, forcing 8-bit mode!\n");
        dev->flags &= ~FLAG_WIDE;
    }

    for (int i = 0; i < UMB_MAX_RANGES; i++) {
        snprintf(key, sizeof(key), "umb%i_start", i + 1);
        start[i] = ((uint32_t) device_get_config_int(key)) << 10;
        snprintf(key, sizeof(key), "umb%i_size", i + 1);
        len[i] = ((uint32_t) device_get_config_int(key)) << 10;

        /* Keep the range inside the UMA and on 16 KB boundaries. */
        start[i] &= ~(UMB_GRAN - 1);
        len[i] &= ~(UMB_GRAN - 1);
        if (start[i] < RAM_TOPMEM)
            start[i] = RAM_TOPMEM;
        if (start[i] >= RAM_EXTMEM)
            len[i] = 0;
        else if ((start[i] + len[i]) > RAM_EXTMEM)
            len[i] = RAM_EXTMEM - start[i];

        /* Clip a second range against the first. */
        for (int j = 0; j < i; j++) {
            if (!len[i] || !len[j])
                continue;
            if ((start[i] < start[j] + len[j]) && (start[j] < start[i] + len[i])) {
                if (start[i] >= start[j]) {
                    uint32_t end = start[i] + len[i];
                    start[i]     = start[j] + len[j];
                    len[i]       = (end > start[i]) ? (end - start[i]) : 0;
                } else
                    len[i] = start[j] - start[i];
                isamem_log(dev->log, "UMB range %i overlaps range %i, clipped\n", i + 1, j + 1);
            }
        }
        total += len[i];
    }

    dev->total_size = (uint16_t) (total >> 10);
    isamem_log(dev->log, "%s (%iKB%s)\n", info->name, dev->total_size,
               (dev->flags & FLAG_WIDE) ? ", 16BIT" : "");

    if (total == 0)
        return dev;

    dev->ram = (uint8_t *) calloc(1, total);
    ptr      = dev->ram;

    for (int i = 0; i < UMB_MAX_RANGES; i++) {
        if (!len[i])
            continue;

        isamem_log(dev->log, "UMB RAM at %05X-%05X (%iKB)\n",
                   start[i], start[i] + len[i] - 1, len[i] >> 10);

        dev->umb_ram[i].ptr  = ptr;
        dev->umb_ram[i].base = start[i];
        mem_mapping_add(&dev->umb_mapping[i], start[i], len[i],
                        ram_readb,
                        (dev->flags & FLAG_WIDE) ? ram_readw : NULL,
                        NULL,
                        ram_writeb,
                        (dev->flags & FLAG_WIDE) ? ram_writew : NULL,
                        NULL,
                        ptr, MEM_MAPPING_EXTERNAL, &dev->umb_ram[i]);
        ptr += len[i];
    }

    return dev;
}

static void
genumb_close(void *priv)
{
    memdev_t *dev = (memdev_t *) priv;

    for (int i = 0; i < UMB_MAX_RANGES; i++)
        if (dev->umb_ram[i].ptr)
            mem_mapping_disable(&dev->umb_mapping[i]);

    isamem_close(priv);
}


static const device_config_t ibmxt_32k_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 32,
        .file_filter    = NULL,
        .spinner        = {
            .min  =  32,
            .max  = 576,
            .step =  32
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 64,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 608,
            .step =  32
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t ibmxt_32k_device = {
    .name          = "IBM PC/XT 32K Memory Expansion Option",
    .internal_name = "ibmxt_32k",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_IBMXT_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ibmxt_32k_config
};

static const device_config_t ibmxt_64k_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 64,
        .file_filter    = NULL,
        .spinner        = {
            .min  =  64,
            .max  = 576,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 64,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 576,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t ibmxt_64k_device = {
    .name          = "IBM PC/XT 64K Memory Expansion Option",
    .internal_name = "ibmxt_64k",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_IBMXT_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ibmxt_64k_config
};

static const device_config_t ibmxt_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 128,
        .file_filter    = NULL,
        .spinner        = {
            .min  =  64,
            .max  = 576,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 256,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 576,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t ibmxt_device = {
    .name          = "IBM PC/XT 64/256K Memory Expansion Option",
    .internal_name = "ibmxt",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_IBMXT_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ibmxt_config
};

static const device_config_t genericxt_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 16,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 640,
            .step =  16
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 640,
            .step =  16
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_config_t genericpcjr_config[] = {
    // clang-format off
        {
                .name           = "size",
                .description    = "Memory size",
                .type           = CONFIG_SPINNER,
                .default_string = NULL,
                .default_int    = 128,
                .file_filter    = NULL,
                .spinner        = {
                        .min  =  64,
                        .max  = 512,
                        .step =  64
                },
                .selection      = { { 0 } },
                .bios           = { { 0 } }
        },
            {
                .name           = "start",
                .description    = "Start Address",
                .type           = CONFIG_SPINNER,
                .default_string = NULL,
                .default_int    = 128,
                .file_filter    = NULL,
                .spinner        = {
                    .min  =  64,
                    .max  = 576,
                    .step =  64
                },
                .selection      = { { 0 } },
                .bios           = { { 0 } }
            },
        { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

static const device_config_t ibmpcjr_config[] = {
    // clang-format off
        {
                .name           = "start",
                .description    = "Start Address",
                .type           = CONFIG_SPINNER,
                .default_string = NULL,
                .default_int    = 128,
                .file_filter    = NULL,
                .spinner        = {
                        .min  = 128,
                        .max  = 512,
                        .step = 128
                },
                .selection      = { { 0 } },
                .bios           = { { 0 } }
        },
        { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

// This also nicely accounts for the Everex EV-138
static const device_t genericxt_device = {
    .name          = "Generic PC/XT Memory Expansion",
    .internal_name = "genericxt",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_GENXT_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = genericxt_config
};

#define GENUMB_RANGE_CONFIG(n, def_start, def_size)              \
    {                                                            \
        .name           = "umb" #n "_start",                     \
        .description    = "Range " #n " start address (KB)",     \
        .type           = CONFIG_SPINNER,                        \
        .default_string = NULL,                                  \
        .default_int    = def_start,                             \
        .file_filter    = NULL,                                  \
        .spinner        = {                                      \
            .min  =  640,                                        \
            .max  = 1008,                                        \
            .step =   16                                         \
        },                                                       \
        .selection      = { { 0 } },                             \
        .bios           = { { 0 } }                              \
    },                                                           \
    {                                                            \
        .name           = "umb" #n "_size",                      \
        .description    = "Range " #n " size (KB, 0 = none)",    \
        .type           = CONFIG_SPINNER,                        \
        .default_string = NULL,                                  \
        .default_int    = def_size,                              \
        .file_filter    = NULL,                                  \
        .spinner        = {                                      \
            .min  =   0,                                         \
            .max  = 384,                                         \
            .step =  16                                          \
        },                                                       \
        .selection      = { { 0 } },                             \
        .bios           = { { 0 } }                              \
    }

static const device_config_t genumb_config[] = {
  // clang-format off
    /* Default D0000-EFFFF, the range in the USE!UMBS.SYS example. */
    GENUMB_RANGE_CONFIG(1, 832, 128),
    GENUMB_RANGE_CONFIG(2, 800,   0),
    {
        .name           = "width",
        .description    = "Bus Width",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 8,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "8-bit",  .value =  8 },
            { .description = "16-bit", .value = 16 },
            { .description = ""                    }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t genumb_device = {
    .name          = "Generic UMB Expansion",
    .internal_name = "genericumb",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_GENUMB_CARD,
    .init          = genumb_init,
    .close         = genumb_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = genumb_config
};

static const device_t ibmpcjr_device = {
    .name          = "IBM PCjr 128KB Memory Expansion",
    .internal_name = "ibmpcjr_mem",
    .flags         = DEVICE_SIDECAR,
    .local         = ISAMEM_IBMPCJR_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ibmpcjr_config
};

static const device_t genericpcjr_device = {
    .name          = "Generic PCjr Memory Expansion",
    .internal_name = "genericpcjr_mem",
    .flags         = DEVICE_SIDECAR,
    .local         = ISAMEM_GENPCJR_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = genericpcjr_config
};

static const device_t jride_mem_device = {
    .name          = "jr-IDE",
    .internal_name = "jride_mem",
    .flags         = DEVICE_SIDECAR,
    .local         = ISAMEM_JRIDE_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

static const device_config_t msramcard_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 64,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 256,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 624,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t msramcard_device = {
    .name          = "Microsoft RAMCard",
    .internal_name = "msramcard",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_RAMCARD_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = msramcard_config
};

static const device_config_t mssystemcard_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 64,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 256,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 624,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t mssystemcard_device = {
    .name          = "Microsoft SystemCard",
    .internal_name = "mssystemcard",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_SYSTEMCARD_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = mssystemcard_config
};

static const device_t ibmat_128k_device = {
    .name          = "IBM PC/AT 128KB Memory Expansion Option",
    .internal_name = "ibmat_128k",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_IBMAT_128K_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

static const device_config_t ibmat_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 512,
        .file_filter    = NULL,
        .spinner        = {
            .min  =     0,
            .max  = 12288,
            .step =   512
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 1024,
        .file_filter    = NULL,
        .spinner        = {
            .min  =     0,
            .max  = 15872,
            .step =   512
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t ibmat_device = {
    .name          = "IBM PC/AT Memory Expansion",
    .internal_name = "ibmat",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_IBMAT_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ibmat_config
};

static const device_config_t genericat_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 512,
        .file_filter    = NULL,
        .spinner        = {
            .min  =     0,
            .max  = 16384,
            .step =   128
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 1024,
        .file_filter    = NULL,
        .spinner        = {
            .min  =     0,
            .max  = 15872,
            .step =   128
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

// This also nicely accounts for the Everex EV-135
static const device_t genericat_device = {
    .name          = "Generic PC/AT Memory Expansion",
    .internal_name = "genericat",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_GENAT_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = genericat_config
};

static const device_config_t p5pak_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 128,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 384,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 512,
        .file_filter    = NULL,
        .spinner        = {
            .min  =  64,
            .max  = 576,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t p5pak_device = {
    .name          = "Paradise Systems 5-PAK",
    .internal_name = "p5pak",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_P5PAK_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = p5pak_config
};

static const device_config_t a6pak_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 64,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 384,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 256,
        .file_filter    = NULL,
        .spinner        = {
            .min  =  64,
            .max  = 512,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t a6pak_device = {
    .name          = "AST SixPakPlus",
    .internal_name = "a6pak",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_A6PAK_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = a6pak_config
};

static const device_config_t ems5150_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 256,
        .file_filter    = NULL,
        .spinner        = {
            .min  =    0,
            .max  = 2048,
            .step =   64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0x0000 },
            { .description = "208H",     .value = 0x0208 },
            { .description = "20AH",     .value = 0x020a },
            { .description = "20CH",     .value = 0x020c },
            { .description = "20EH",     .value = 0x020e },
            { .description = ""                          }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t ems5150_device = {
    .name          = "Micro Mainframe EMS-5150(T)",
    .internal_name = "ems5150",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_EMS5150_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ems5150_config
};

static const device_config_t ev159_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 512,
        .file_filter    = NULL,
        .spinner        = {
            .min  =    0,
            .max  = 3072,
            .step =  512
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = {
            .min  =     0,
            .max  = 16128,
            .step =   128
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "length",
        .description    = "Contiguous Size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = {
            .min  =     0,
            .max  = 16384,
            .step =   128
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "width",
        .description    = "I/O Width",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "8-bit",  .value = 0 },
            { .description = "16-bit", .value = 1 },
            { .description = ""                   }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "speed",
        .description    = "Transfer Speed",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Standard (150ns)",   .value = 0 },
            { .description = "High-Speed (120ns)", .value = 1 },
            { .description = ""                               }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "ems",
        .description    = "EMS mode",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0 },
            { .description = "Enabled",  .value = 1 },
            { .description = ""                     }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0258,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "208H", .value = 0x0208 },
            { .description = "218H", .value = 0x0218 },
            { .description = "258H", .value = 0x0258 },
            { .description = "268H", .value = 0x0268 },
            { .description = "2A8H", .value = 0x02A8 },
            { .description = "2B8H", .value = 0x02B8 },
            { .description = "2E8H", .value = 0x02E8 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "base2",
        .description    = "Address for > 2 MB",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0268,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "208H", .value = 0x0208 },
            { .description = "218H", .value = 0x0218 },
            { .description = "258H", .value = 0x0258 },
            { .description = "268H", .value = 0x0268 },
            { .description = "2A8H", .value = 0x02A8 },
            { .description = "2B8H", .value = 0x02B8 },
            { .description = "2E8H", .value = 0x02E8 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t ev159_device = {
    .name          = "Everex EV-159 RAM 3000 Deluxe",
    .internal_name = "ev159",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_EV159_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ev159_config
};

static const device_config_t ev165a_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 256,
        .file_filter    = NULL,
        .spinner        = {
            .min  =    0,
            .max  = 2048,
            .step =  256
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 64,
        .file_filter    = NULL,
        .spinner        = {
            .min  =  64,
            .max  = 640,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name = "length",
        .description = "Contiguous Size",
        .type = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = {
            .min  =    0,
            .max  = 2048,
            .step =  256
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "ems",
        .description    = "EMS mode",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0 },
            { .description = "Enabled",  .value = 1 },
            { .description = ""                     }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0258,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "208H", .value = 0x0208 },
            { .description = "218H", .value = 0x0218 },
            { .description = "258H", .value = 0x0258 },
            { .description = "268H", .value = 0x0268 },
            { .description = "2A8H", .value = 0x02A8 },
            { .description = "2B8H", .value = 0x02B8 },
            { .description = "2E8H", .value = 0x02E8 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t ev165a_device = {
    .name          = "Everex Maxi Magic EV-165A",
    .internal_name = "ev165a",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_EV165A_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ev165a_config
};

static const device_config_t brxt_config[] = {
  // clang-format off
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0268,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "208H", .value = 0x0208 },
            { .description = "218H", .value = 0x0218 },
            { .description = "258H", .value = 0x0258 },
            { .description = "268H", .value = 0x0268 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "frame",
        .description    = "Frame Address",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0xD0000,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "D000H", .value = 0xD0000 },
            { .description = "E000H", .value = 0xE0000 },
            { .description = ""                        }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 512,
        .file_filter    = NULL,
        .spinner        = {
            .min  =    0,
            .max  = 2048,
            .step =  512
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t brxt_device = {
    .name          = "BocaRAM/XT",
    .internal_name = "brxt",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_BRXT_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = brxt_config
};

static const device_config_t brat_config[] = {
  // clang-format off
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0268,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "208H", .value = 0x0208 },
            { .description = "218H", .value = 0x0218 },
            { .description = "258H", .value = 0x0258 },
            { .description = "268H", .value = 0x0268 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "frame",
        .description    = "Frame Address",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0xD0000,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "D000H",    .value = 0xD0000 },
            { .description = "E000H",    .value = 0xE0000 },
            { .description = ""                           }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "width",
        .description    = "I/O Width",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 8,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "8-bit",  .value =  8 },
            { .description = "16-bit", .value = 16 },
            { .description = ""                    }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "speed",
        .description    = "Transfer Speed",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Standard",   .value = 0 },
            { .description = "High-Speed", .value = 1 },
            { .description = ""                       }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 512,
        .file_filter    = NULL,
        .spinner        = {
            .min  =    0,
            .max  = 4096,
            .step =  512
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = {
            .min  =     0,
            .max  = 14336,
            .step =   512
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name = "length",
        .description = "Contiguous Size",
        .type = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = {
            .min  =    0,
            .max  = 4096,
            .step =  512
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t brat_device = {
    .name          = "BocaRAM/AT",
    .internal_name = "brat",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_BRAT_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = brat_config
};

static const device_config_t lotech_config[] = {
// clang-format off
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0260,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "260H", .value = 0x0260 },
            { .description = "264H", .value = 0x0264 },
            { .description = "268H", .value = 0x0268 },
            { .description = "26CH", .value = 0x026C },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "frame",
        .description    = "Frame Address",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0xe0000,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "C000H",    .value = 0xC0000 },
            { .description = "D000H",    .value = 0xD0000 },
            { .description = "E000H",    .value = 0xE0000 },
            { .description = ""                           }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 2048,
        .file_filter    = NULL,
        .spinner        = {
            .min  =  512,
            .max  = 4096,
            .step =  512
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
// clang-format on
};

static const device_t lotech_ems_device = {
    .name          = "Lo-tech EMS Board",
    .internal_name = "lotechems",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_LOTECH_EMS_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = lotech_config
};

// TODO: Dual Paging support
// TODO: Conventional memory suppport (Should be done now)
static const device_config_t rampagext_config[] = {
  // clang-format off
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0218,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "208H", .value = 0x0208 },
            { .description = "218H", .value = 0x0218 },
            { .description = "258H", .value = 0x0258 },
            { .description = "268H", .value = 0x0268 },
            { .description = "2A8H", .value = 0x02A8 },
            { .description = "2B8H", .value = 0x02B8 },
            { .description = "2E8H", .value = 0x02E8 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 256, /* Technically 128k, but banks 2-7 must be 256, headaches elsewise */
        .file_filter    = NULL,
        .spinner        = {
            .min  =  128,
            .max  = 2048,
            .step =  128
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 640,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 640,
            .step = 64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name = "length",
        .description = "Contiguous Size",
        .type = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 640,
            .step = 64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "dual_page",
        .description    = "Dual Page Mode",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 1,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0 },
            { .description = "Enabled",  .value = 1 },
            { .description = ""                     }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t rampagext_device = {
    .name          = "AST RAMpage/XT",
    .internal_name = "rampage",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_RAMPAGEXT_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = rampagext_config
};

/*
 * AST SixPakPremium (p/n 202094-001): a multifunction XT card whose memory
 * section is a RAMpage! (202073-002) - same EEMS registers, same switch
 * layout: start address SW1-1..4 in 64 KB steps, dual page SW1-7, parity
 * SW1-8, "memory used for base 640K" SW2-7/8 = 0/256/512/768 KB - so it uses
 * the RAMpage/XT model. 256 KB-1 MB on board, 2 MB with the Premium-Pak
 * daughterboard; LIM 3.2, EEMS and LIM 4.0 through AST REMM.SYS. Its serial,
 * parallel, game and clock functions are separate devices.
 */
static const device_config_t a6pakprem_config[] = {
  // clang-format off
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0218,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "208H", .value = 0x0208 },
            { .description = "218H", .value = 0x0218 },
            { .description = "258H", .value = 0x0258 },
            { .description = "268H", .value = 0x0268 },
            { .description = "2A8H", .value = 0x02A8 },
            { .description = "2B8H", .value = 0x02B8 },
            { .description = "2E8H", .value = 0x02E8 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 1024,
        .file_filter    = NULL,
        .spinner        = {
            .min  =  256,
            .max  = 2048,
            .step =  256
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Conventional Memory Starting Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 640,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   0,
            .max  = 640,
            .step =  64
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "length",
        .description    = "Memory used for Base 640K",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "0K",          .value =   0 },
            { .description = "Up to 256K",  .value = 256 },
            { .description = "Up to 512K",  .value = 512 },
            { .description = "Up to 768K",  .value = 768 },
            { .description = ""                          }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "dual_page",
        .description    = "Dual Page Mode",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 1,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0 },
            { .description = "Enabled",  .value = 1 },
            { .description = ""                     }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t a6pakprem_device = {
    .name          = "AST SixPakPremium",
    .internal_name = "a6pakprem",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_RAMPAGEXT_CARD, /* RAMpage! memory section, see above */
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = a6pakprem_config
};

static const device_config_t rampagextv1_config[] = {
  // clang-format off
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0218,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "208H", .value = 0x0208 },
            { .description = "218H", .value = 0x0218 },
            { .description = "258H", .value = 0x0258 },
            { .description = "268H", .value = 0x0268 },
            { .description = "2A8H", .value = 0x02A8 },
            { .description = "2B8H", .value = 0x02B8 },
            { .description = "2E8H", .value = 0x02E8 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 256,
        .file_filter    = NULL,
        .spinner        = {
            .min  =  128,
            .max  = 2048,
            .step =  128
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 640,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "0 KB",   .value =   0 },
            { .description = "256 KB", .value = 256 },
            { .description = "512 KB", .value = 512 },
            { .description = "640 KB", .value = 640 },
            { .description = ""                     }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "length",
        .description    = "Conventional Memory",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "0 KB",   .value =   0 },
            { .description = "256 KB", .value = 256 },
            { .description = "512 KB", .value = 512 },
            { .description = "768 KB", .value = 768 },
            { .description = ""                     }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "dual_page",
        .description    = "Dual Page Mode",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 1,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0 },
            { .description = "Enabled",  .value = 1 },
            { .description = ""                     }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t rampagextv1_device = {
    .name          = "AST RAMpage/XT (/V=1)",
    .internal_name = "rampagev1",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_RAMPAGEXTV1_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = rampagextv1_config
};

static const device_config_t rampageat_config[] = {
  // clang-format off
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0218,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "208H", .value = 0x0208 },
            { .description = "218H", .value = 0x0218 },
            { .description = "258H", .value = 0x0258 },
            { .description = "268H", .value = 0x0268 },
            { .description = "2A8H", .value = 0x02A8 },
            { .description = "2B8H", .value = 0x02B8 },
            { .description = "2E8H", .value = 0x02E8 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "frame",
        .description    = "Frame Address",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0xE0000,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "C000H", .value = 0xC0000 },
            { .description = "D000H", .value = 0xD0000 },
            { .description = "E000H", .value = 0xE0000 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 512,
        .file_filter    = NULL,
        .spinner        = {
            .min  =  512,
            .max  = 2048,
            .step =  128
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 1024,
        .file_filter    = NULL,
        .spinner        = {
            .min  =     0,
            .max  = 15360,
            .step =   128
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "length",
        .description    = "Contiguous Size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = {
            .min  =    0,
            .max  = 2048,
            .step =  128
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "dual_page",
        .description    = "Dual Page Mode",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 1,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0 },
            { .description = "Enabled",  .value = 1 },
            { .description = ""                     }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t rampageat_device = {
    .name          = "AST Rampage AT",
    .internal_name = "rampageat",
    .flags         = DEVICE_ISA16,
    .local         = ISAMEM_RAMPAGEAT_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = rampageat_config
};

static const device_config_t mplus2_config[] = {
  // clang-format off
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = "",
        .default_int    = 64,
        .file_filter    = "",
        .spinner        = {
            .min  = 0,
            .max  = 512,
            .step = 64
        },
        .selection      = { { 0 } }
    },
    {
        .name           = "start",
        .description    = "Start Address",
        .type           = CONFIG_SPINNER,
        .default_string = "",
        .default_int    = 256,
        .file_filter    = "",
        .spinner        = {
            .min  = 64,
            .max  = 576,
            .step = 64
        },
        .selection      = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t mplus2_device = {
    .name          = "AST MegaPlus II",
    .internal_name = "mplus2",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_MPLUS2_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = mplus2_config
};

static const device_config_t xtmax_ems_config[] = {
  // clang-format off
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0260,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            /* MMAN_BASE in the firmware; any multiple of 10h if rebuilt */
            { .description = "240H",           .value = 0x0240 },
            { .description = "250H",           .value = 0x0250 },
            { .description = "260H (default)", .value = 0x0260 },
            { .description = "270H",           .value = 0x0270 },
            { .description = ""                                }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "size",
        .description    = "Memory size",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 16384,
        .file_filter    = NULL,
        .spinner        = {
            .min  =   512,
            .max  = 32768,
            .step =   512
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_t xtmax_ems_device = {
    .name          = "XTMax (EMS)",
    .internal_name = "xtmax_ems",
    .flags         = DEVICE_ISA,
    .local         = ISAMEM_XTMAX_EMS_CARD,
    .init          = isamem_init,
    .close         = isamem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = xtmax_ems_config
};

static const struct {
    const device_t *dev;
} boards[] = {
    // clang-format off
    { &device_none         },
    // PCjr Sidecar Memory Expansion Cards
    { &ibmpcjr_device      },
    { &genericpcjr_device  },
    { &jride_mem_device    },
    // XT Ram Expansion Cards
    { &ibmxt_32k_device    },
    { &ibmxt_64k_device    },
    { &ibmxt_device        },
    { &genericxt_device    },
    { &msramcard_device    },
    { &mssystemcard_device },
    // AT RAM Expansion Cards
    { &ibmat_128k_device   },
    { &ibmat_device        },
    { &genericat_device    },
    // UMB Cards
    { &genumb_device       },
    // EMS Cards
    { &p5pak_device        },
    { &a6pak_device        },
    { &ems5150_device      },
    { &ev159_device        },
    { &ev165a_device       },
    { &ev158a_device       },
    { &ev178_device        },
    { &brxt_device         },
    { &brat_device         },
    { &rampagext_device    },
    { &rampagextv1_device  },
    { &a6pakprem_device    },
#ifdef USE_ISAMEM_RAMPAGE
    { &rampageat_device     },
#endif /* USE_ISAMEM_RAMPAGE */
    { &lotech_ems_device   },
    { &xtmax_ems_device    },
    { &mplus2_device       },
    // Intel Above Board family (isamem_iab.c)
    { &iab_pc_device       },
    { &iab_pspc_device     },
    { &iab_at_device       },
    { &iab_psat_device     },
    { &iab_286_device      },
    { &iab_plus_device     },
    { &iab_plus8_device    },
    { &iab_mmc_device      },
    { NULL                 }
    // clang-format on
};

void
isamem_reset(void)
{
    int k;

    /* We explicitly set to zero here or bad things happen */
    isa_mem_size = 0;

    for (uint8_t i = 0; i < ISAMEM_MAX; i++) {
        k = isamem_type[i];
        if (k == 0)
            continue;

        /* Add the instance to the system. */
        device_add_inst(boards[k].dev, i + 1);
    }
}

const char *
isamem_get_name(int board)
{
    if (boards[board].dev == NULL)
        return (NULL);

    return (boards[board].dev->name);
}

const char *
isamem_get_internal_name(int board)
{
    return device_get_internal_name(boards[board].dev);
}

int
isamem_get_from_internal_name(const char *str)
{
    int c = 0;

    while (boards[c].dev != NULL) {
        if (!strcmp(boards[c].dev->internal_name, str))
            return c;
        c++;
    }

    /* Not found. */
    return 0;
}

const device_t *
isamem_get_device(int board)
{
    /* Add the instance to the system. */
    return boards[board].dev;
}

int
isamem_has_config(int board)
{
    if (boards[board].dev == NULL)
        return 0;

    return (boards[board].dev->config ? 1 : 0);
}
