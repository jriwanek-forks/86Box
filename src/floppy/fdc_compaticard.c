/*
 * 86Box     A hypervisor and IBM PC system emulator that specializes in
 *           running old operating systems and software designed for IBM
 *           PC systems and compatibles from 1981 through fairly recent
 *           system designs based on the PCI bus.
 *
 *           This file is part of the 86Box distribution.
 *
 *           Emulation of Micro Solutions CompatiCard I/II/IV.
 *
 * Authors:  Jasmine Iwanek, <jasmine@iwanek.co.uk>
 *
 *           Copyright 2022-2025 Jasmine Iwanek.
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include "cpu.h"
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/machine.h>
#include <86box/timer.h>
#include <86box/fdd.h>
#include <86box/fdc.h>
#include <86box/fdc_ext.h>

#define DEVICE_COMPATICARD_I  0
#define DEVICE_COMPATICARD_II 1
#define DEVICE_COMPATICARD_IV 2

#define ROM_COMPATICARD_IV "roms/floppy/compaticard/ccivbios1.05.bin"

#define CCIV_BIOS_ADDR_DISABLED 0x07 /* 111 */
#define CCIV_BIOS_ADDR_CC000H   0x03 /* 011 */
#define CCIV_BIOS_ADDR_CE000H   0x05 /* 101 */
#define CCIV_BIOS_ADDR_D0000H   0x01 /* 001 */
#define CCIV_BIOS_ADDR_D8000H   0x06 /* 110 */
#define CCIV_BIOS_ADDR_DE000H   0x02 /* 010 */
#define CCIV_BIOS_ADDR_E8000H   0x04 /* 100 */
#define CCIV_BIOS_ADDR_EE000H   0x00 /* 000 */

#define CCIV_DMA_2_IRQ_FIXED 0x30
#define CCIV_DMA_1           0x20
#define CCIV_DMA_2           0x10
#define CCIV_DMA_3           0x00

#define CR_2_MASK 0x2f /* 00101111b */

typedef struct compaticard_s {
    rom_t   bios_rom;
    fdc_t  *fdc;
    /*
     * 7 - Reserved - Set to 0
     * 6 - Reserved - Set to 0
     * 5 - Programmable Pin 2 Logic I sets Pin 2 low (TODO)
     * 4 - Reserved - Set to 0
     * 3-0 - Data Transfer Rate Select (TODO)
     *       0000---250 Kbps
     *       0001-300 Kbps
     *       1111-500 Kbps
     */
    uint8_t cr_2;

    // Compaticard IV Switches
    uint8_t regs[2];
} compaticard_t;

#define ENABLE_COMPATICARD_LOG 1
#ifdef ENABLE_COMPATICARD_LOG
int compaticard_do_log = ENABLE_COMPATICARD_LOG;

static void
compaticard_log(const char *fmt, ...)
{
    va_list ap;

    if (compaticard_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define compaticard_log(fmt, ...)
#endif

static void
compaticard_out(uint16_t port, uint8_t val, void *priv)
{
#if 0
    compaticard_t *dev = (compaticard_t *) priv;
#endif

    compaticard_log("Write CompatiCard I/II %04X %02X\n", port, val);

#if 0
    dev->cr_2 = (val & CR_2_MASK);
#endif
}

static uint8_t
compaticard_in(uint16_t port, void *priv)
{
    compaticard_t *dev  = (compaticard_t *) priv;
    uint8_t        ret  = (dev->cr_2 & CR_2_MASK);

    compaticard_log("[%04X:%08X] Read CompatiCard I/II %04X %02X\n", CS, cpu_state.pc, port, ret);

    return ret;
}

static void
compaticardiv_out(uint16_t port, uint8_t val, void *priv)
{
#if 0
    compaticard_t *dev = (compaticard_t *) priv;
#endif

    compaticard_log("Write CompatiCard IV %04X %02X\n", port, val);

#if 0
    dev->regs[port] = val;
#endif
}

static uint8_t
compaticardiv_in(uint16_t port, void *priv)
{
    compaticard_t *dev  = (compaticard_t *) priv;
    uint8_t        ret  = dev->regs[0];

    compaticard_log("[%04X:%08X] Read CompatiCard IV %04X %02X\n", CS, cpu_state.pc, port, ret);

    return ~ret;
}

static void
compaticard_close(void *priv)
{
    compaticard_t *dev = (compaticard_t *) priv;

    free(dev);
}

static void *
compaticard_init(const device_t *info)
{
    compaticard_t *dev              = calloc(1, sizeof(compaticard_t));
    uint16_t       base_addr        = device_get_config_hex16("base");
    uint8_t        irq              = 6;
    uint8_t        dma              = 2;
    uint16_t       cr2_addr         = 0x0000; // Control Register 2

    // CompatiCard II has configurable IRQ and DMA
    if (info->local == DEVICE_COMPATICARD_II) {
        irq = device_get_config_int("irq");
        dma = device_get_config_int("dma");
    }

    switch (base_addr) {
        case FDC_SECONDARY_ADDR:
            cr2_addr = 0x772;
            dev->fdc = device_add(&fdc_xt_sec_device);
            break;

        case FDC_TERTIARY_ADDR:
            cr2_addr = 0x762;
            dev->fdc = device_add(&fdc_xt_ter_device);
            break;

        case FDC_QUATERNARY_ADDR:
            cr2_addr = 0x7e2;
            dev->fdc = device_add(&fdc_xt_qua_device);
            break;

        case FDC_PRIMARY_ADDR:
        default:
            cr2_addr = 0x7f2;
            dev->fdc = device_add(&fdc_xt_device);
            break;
    }

    fdc_set_irq(dev->fdc, irq);
    fdc_set_dma_ch(dev->fdc, dma);

    io_sethandler(cr2_addr, 0x0001,
                  compaticard_in, NULL, NULL,
                  compaticard_out, NULL, NULL,
                  dev);

    return dev;
}

static void *
compaticardiv_init(const device_t *info)
{
    compaticard_t *dev              = calloc(1, sizeof(compaticard_t));
    uint16_t       base_addr        = device_get_config_hex16("base");
    uint8_t        irq              = device_get_config_int("irq");
    uint8_t        dma              = device_get_config_int("dma");
    uint8_t        autoboot_enabled = 0;
    uint8_t        spindle_speed    = 0;

    uint32_t bios_addr = (uint32_t)(device_get_config_hex20("bios_addr") & 0x000fffff);
    uint8_t  bios_bits = 0x00;
    uint8_t  dma_bits  = 0x00;
    dev->regs[0]       = 0x00;
    compaticard_log("dev->regs[0] initial set was: %X\n", dev->regs[0]);
    dev->regs[1]       = 0x00;

    switch (base_addr) {
        case FDC_SECONDARY_ADDR:
            dev->fdc = device_add(&fdc_at_nsc_dp8477_sec_device);
            break;

        case FDC_TERTIARY_ADDR:
            dev->fdc = device_add(&fdc_at_nsc_dp8477_ter_device);
            break;

        case FDC_QUATERNARY_ADDR:
            dev->fdc = device_add(&fdc_at_nsc_dp8477_qua_device);
            break;

        case FDC_PRIMARY_ADDR:
        default:
            dev->fdc = device_add(&fdc_at_nsc_dp8477_device);
            break;
    }

    if (bios_addr != 0) {
        rom_init(&dev->bios_rom, ROM_COMPATICARD_IV, bios_addr, 0x2000, 0x1ffff, 0, MEM_MAPPING_EXTERNAL);
        switch (bios_addr) {
            case 0xcc000:
                bios_bits = CCIV_BIOS_ADDR_CC000H;
                break;

            case 0xce000:
                bios_bits = CCIV_BIOS_ADDR_CE000H;
                break;

            case 0xd0000:
                bios_bits = CCIV_BIOS_ADDR_D0000H;
                break;

            case 0xd8000:
                bios_bits = CCIV_BIOS_ADDR_D8000H;
                break;

            case 0xde000:
                bios_bits = CCIV_BIOS_ADDR_DE000H;
                break;

            case 0xe8000:
                bios_bits = CCIV_BIOS_ADDR_E8000H;
                break;

            case 0xee000:
                bios_bits = CCIV_BIOS_ADDR_EE000H;
                break;

            default:
                bios_bits = CCIV_BIOS_ADDR_DISABLED;
        }
    } else
        bios_bits = CCIV_BIOS_ADDR_DISABLED;

    dev->regs[0] |= bios_bits;
    compaticard_log("dev->regs[0] after setting bios_bits was: %X\n", dev->regs[0]);

    switch (dma) {
        case 1:
            dma_bits = CCIV_DMA_1;
            break;

        case 2:
            dma_bits = CCIV_DMA_2;
            break;

        case 3:
            dma_bits = CCIV_DMA_3;
            break;

        case 0x10:
            dma_bits = CCIV_DMA_2_IRQ_FIXED;
			irq      = 6;
    }

    dev->regs[0] |= dma_bits;
    compaticard_log("dev->regs[0] after setting dma_bits was: %X\n", dev->regs[0]);

    autoboot_enabled = device_get_config_int("autoboot_enabled");
    if (!autoboot_enabled) {
        dev->regs[0] |= 0x80;
        compaticard_log("dev->regs[0] after setting autoboot_enabled was: %X\n", dev->regs[0]);
    }

    spindle_speed = device_get_config_int("spindle_speed");
    if (spindle_speed) {
        dev->regs[0] |= 0x40;
        compaticard_log("dev->regs[0] after setting spindle_speed was: %X\n", dev->regs[0]);
    }

    fdc_set_irq(dev->fdc, irq);
    fdc_set_dma_ch(dev->fdc, dma);

    io_sethandler(base_addr, 0x0002,
                  compaticardiv_in, NULL, NULL,
                  compaticardiv_out, NULL, NULL,
                  dev);

    return dev;
}

static int compaticard_iv_available(void)
{
    return rom_present(ROM_COMPATICARD_IV);
}

static const device_config_t compaticard_i_config[] = {
// clang-format off
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = "",
        .default_int    = 0x3f0,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "0x3f0", .value = 0x3f0 },
            { .description = "0x370", .value = 0x370 },
            { .description = "0x360", .value = 0x360 },
            { .description = "0x3e0", .value = 0x3e0 },
            { .description = ""                      }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
// clang-format on
};

static const device_config_t compaticard_ii_config[] = {
// clang-format off
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = "",
        .default_int    = 0x3f0,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "0x3f0", .value = 0x3f0 },
            { .description = "0x370", .value = 0x370 },
            { .description = "0x360", .value = 0x360 },
            { .description = "0x3e0", .value = 0x3e0 },
            { .description = ""                      }
        }
    },
    {
        .name           = "irq",
        .description    = "IRQ",
        .type           = CONFIG_SELECTION,
        .default_string = "",
        .default_int    = 6,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "IRQ 2", .value = 2 },
            { .description = "IRQ 3", .value = 3 },
            { .description = "IRQ 4", .value = 4 },
            { .description = "IRQ 5", .value = 5 },
            { .description = "IRQ 6", .value = 6 },
            { .description = "IRQ 7", .value = 7 },
            { .description = ""                  }
        }
    },
    {
        .name           = "dma",
        .description    = "DMA channel",
        .type           = CONFIG_SELECTION,
        .default_string = "",
        .default_int    = 2,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "DMA 1", .value = 1 },
            { .description = "DMA 2", .value = 2 },
            { .description = "DMA 3", .value = 3 },
            { .description = ""                  }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
// clang-format on
};

static const device_config_t compaticard_iv_config[] = {
// clang-format off
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = "",
        .default_int    = 0x3f0,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "0x3f0", .value = 0x3f0 },
            { .description = "0x370", .value = 0x370 },
            { .description = "0x360", .value = 0x360 },
            { .description = "0x3e0", .value = 0x3e0 },
            { .description = ""                      }
        }
    },
    {
        .name           = "irq",
        .description    = "IRQ",
        .type           = CONFIG_SELECTION,
        .default_string = "",
        .default_int    = 6,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "IRQ 2", .value = 2 },
            { .description = "IRQ 3", .value = 3 },
            { .description = "IRQ 4", .value = 4 },
            { .description = "IRQ 5", .value = 5 },
            { .description = "IRQ 6", .value = 6 },
            { .description = "IRQ 7", .value = 7 },
            { .description = ""                  }
        }
    },
    {
        .name           = "dma",
        .description    = "DMA channel",
        .type           = CONFIG_SELECTION,
        .default_string = "",
        .default_int    = 2,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "DMA 1", .value = 1                  },
            { .description = "DMA 2", .value = 2                  },
            { .description = "DMA 3", .value = 3                  },
            { .description = "DMA 2 (Fixed IRQ 6)", .value = 0x10 },
            { .description = ""                                   }
        }
    },
    {
        .name           = "bios_addr",
        .description    = "BIOS Address:",
        .type           = CONFIG_HEX20,
        .default_string = "",
        .default_int    = 0xce000,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0       },
            { .description = "CC00H",    .value = 0xcc000 },
            { .description = "CE00H",    .value = 0xce000 },
            { .description = "D000H",    .value = 0xd0000 },
            { .description = "D800H",    .value = 0xd8000 },
            { .description = "DE00H",    .value = 0xde000 },
            { .description = "E800H",    .value = 0xe8000 },
            { .description = "EE00H",    .value = 0xee000 },
            { .description = ""                           }
        }
    },
    {
        .name           = "autoboot_enabled",
        .description    = "Enable Autoboot",
        .type           = CONFIG_BINARY,
        .default_string = "",
        .default_int    = 0
    },
    {
        .name           = "spindle_speed",
        .description    = "1.2MB Spindle Speed",
        .type           = CONFIG_SELECTION,
        .default_string = "",
        .default_int    = 0,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "Single Speed", .value = 0 },
            { .description = "Dual Speed",   .value = 1 },
            { .description = ""                         }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
// clang-format on
};

const device_t fdc_compaticard_i_device = {
    .name          = "Micro Solutions CompatiCard I",
    .internal_name = "compaticard_i",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = compaticard_init,
    .close         = compaticard_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = compaticard_i_config
};

const device_t fdc_compaticard_ii_device = {
    .name          = "Micro Solutions CompatiCard II",
    .internal_name = "compaticard_ii",
    .flags         = DEVICE_ISA,
    .local         = 1,
    .init          = compaticard_init,
    .close         = compaticard_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = compaticard_ii_config
};

const device_t fdc_compaticard_iv_device = {
    .name          = "Micro Solutions CompatiCard IV",
    .internal_name = "compaticard_iv",
    .flags         = DEVICE_ISA,
    .local         = 2,
    .init          = compaticardiv_init,
    .close         = compaticard_close,
    .reset         = NULL,
    .available     = compaticard_iv_available,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = compaticard_iv_config
};
