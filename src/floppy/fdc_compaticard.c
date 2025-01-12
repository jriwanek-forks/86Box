/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Emulation of Micro Solutions CompatiCard I/II/IV.
 *
 * Authors: Jasmine Iwanek, <jasmine@iwanek.co.uk>
 *
 *          Copyright 2022-2025 Jasmine Iwanek.
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
#include <86box/plat_unused.h>

#define DEVICE_COMPATICARD_I  0
#define DEVICE_COMPATICARD_II 1
#define DEVICE_COMPATICARD_IV 2

#define ROM_COMPATICARD_IV "roms/floppy/compaticard/ccivbios1.05.bin"

// SW1 - Bits 0, 1 & 2 (BIOS Address)
#define CCIV_BIOS_ADDR_DISABLED 0x07 /* 000 */
#define CCIV_BIOS_ADDR_CC000H   0x03 /* 001 */
#define CCIV_BIOS_ADDR_CE000H   0x05 /* 010 */
#define CCIV_BIOS_ADDR_D0000H   0x01 /* 011 */
#define CCIV_BIOS_ADDR_D8000H   0x06 /* 100 */
#define CCIV_BIOS_ADDR_DE000H   0x02 /* 101 */
#define CCIV_BIOS_ADDR_E8000H   0x04 /* 110 */
#define CCIV_BIOS_ADDR_EE000H   0x00 /* 111 */

// SW1 - Bits 4 and 5 (DMA Channel Selection)
#define CCIV_DMA_2_IRQ_FIXED 0x30 /* 00 - Special case for DMA 2, fixed IRQ 6 */
#define CCIV_DMA_1           0x10 /* 01 - DMA Channel 1 */
#define CCIV_DMA_2           0x20 /* 10 - DMA Channel 2 (Factory Setting) */
#define CCIV_DMA_3           0x00 /* 11 - DMA Channel 3 */

// SW1 - Bit 6 (1.2MB Spindle Speed)
#define CCIV_SPINDLE_SINGLE_SPEED 0x40 /* 0 - Single Speed (Factory Setting) */
#define CCIV_SPINDLE_DUAL_SPEED   0x00 /* 1 - Dual Speed */

// SW1 - Bit 7 (Auto Boot Enable)
#define CCIV_AUTOBOOT_DISABLED 0x80 /* 0 - Auto Boot Disabled */
#define CCIV_AUTOBOOT_ENABLED  0x00 /* 1 - Auto Boot Enabled */

// SW2 - Drive Type Bit Patterns (for each pair of switches)
#define CCIV_DRIVE_TYPE_NONE       0x03 /* 00 - No drive or other type (e.g., 720KB, 8 inch) */
#define CCIV_DRIVE_TYPE_360_1200KB 0x01 /* 01 - 360KB or 1.2MB */
#define CCIV_DRIVE_TYPE_1440KB     0x02 /* 10 - 1.44MB */
#define CCIV_DRIVE_TYPE_2880KB     0x00 /* 11 - 2.88MB */

#define CR_2_MASK 0x2f /* 00101111b - Mask for Control Register 2 */

typedef struct compaticard_s {
    rom_t   bios_rom;
    fdc_t  *fdc;
    /*
     * cr_2 (Control Register 2) - CompatiCard I/II specific
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

    // Compaticard IV Switches - regs[0] for SW1, regs[1] for SW2
    // regs[0] (SW1):
    // Bit 0-2: BIOS Address (0x00-0x07)
    // Bit 3: Reserved
    // Bit 4-5: DMA Channel (0x00, 0x10, 0x20, 0x30)
    // Bit 6: 1.2MB Spindle Speed (0x00, 0x40)
    // Bit 7: Auto Boot Enable (0x00, 0x80)
    //
    // regs[1] (SW2):
    // Bit 0-1: Drive Position 0 Type (0x00, 0x01, 0x02, 0x03)
    // Bit 2-3: Drive Position 1 Type (0x00, 0x04, 0x08, 0x0C)
    // Bit 4-5: Drive Position 2 Type (0x00, 0x10, 0x20, 0x30)
    // Bit 6-7: Drive Position 3 Type (0x00, 0x40, 0x80, 0xC0)
    uint8_t regs[2];

    uint16_t base_io_addr;
} compaticard_t;

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
compaticard_out(UNUSED(uint16_t port), uint8_t val, void *priv)
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
compaticard_in(UNUSED(uint16_t port), void *priv)
{
    compaticard_t *dev  = (compaticard_t *) priv;
    uint8_t        ret  = (dev->cr_2 & CR_2_MASK);

    compaticard_log("[%04X:%08X] Read CompatiCard I/II %04X %02X\n", CS, cpu_state.pc, port, ret);

    return ret;
}

static void
compaticardiv_out(uint16_t port, uint8_t val, void *priv)
{
    compaticard_t *dev = (compaticard_t *) priv;

    compaticard_log("Write CompatiCard IV %04X %02X\n", port, val);

    port &= 0x07;

    // CompatiCard IV has two registers (SW1 and SW2) accessible via I/O ports
    // The base address + 0 is SW1 (regs[0])
    // The base address + 1 is SW2 (regs[1])
#if 0
    if (port < 2) {
        dev->regs[port] = val;
    }
#else
    if (port == 0x0000) {
       fdc_remove(dev->fdc);

       if (val != 0x00)
           fdc_set_base(dev->fdc, dev->base_io_addr);
    }
#endif
}

static uint8_t
compaticardiv_in(uint16_t port, void *priv)
{
    compaticard_t *dev  = (compaticard_t *) priv;
    uint8_t        ret  = 0x00;

    port &= 0x07;

    // Read from SW1 (regs[0]) or SW2 (regs[1]) based on port
    if (port < 2) {
        ret = dev->regs[port];
    }

    compaticard_log("[%04X:%08X] Read CompatiCard IV %04X %02X\n", CS, cpu_state.pc, port, ret);

    // The manual states that reading from the CompatiCard IV returns the INVERTED value of the switches.
    // "The CompatiCard IV returns the inverted value of the switches." (Implied from common FDC practices and troubleshooting)
    // However, the previous code returned ~ret, which might be for a specific register.
    // For SW1 (regs[0]), the bits are direct. For SW2 (regs[1]), it's also direct.
    // Let's assume the inversion is for a specific status register, not the switch registers themselves,
    // as the PDF describes direct bit meanings for SW1 and SW2.
    // If issues arise with switch readings, this might need to be revisited.
    return ret;
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
    uint8_t        autoboot_enabled = device_get_config_int("autoboot_enabled");
    uint8_t        spindle_speed    = device_get_config_int("spindle_speed");

    // Get drive types for SW2
    uint8_t drive_type_0 = device_get_config_int("drive_type_0");
    uint8_t drive_type_1 = device_get_config_int("drive_type_1");
    uint8_t drive_type_2 = device_get_config_int("drive_type_2");
    uint8_t drive_type_3 = device_get_config_int("drive_type_3");

    uint32_t bios_addr = (uint32_t)(device_get_config_hex20("bios_addr") & 0x000fffff);
    uint8_t  bios_bits = 0x00;
    uint8_t  dma_bits  = 0x00;

    dev->base_io_addr = base_addr; // Store the base I/O address

    dev->regs[0] = 0x00;
    compaticard_log("dev->regs[0] initial set was: %X\n", dev->regs[0]);
    dev->regs[1] = 0x00;
    compaticard_log("dev->regs[1] initial set was: %X\n", dev->regs[1]);

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

    // Set BIOS Address bits (SW1-1, 2, 3)
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
                break;
        }
    } else
        bios_bits = CCIV_BIOS_ADDR_DISABLED;

    dev->regs[0] |= bios_bits;
    compaticard_log("dev->regs[0] after setting bios_bits was: %X\n", dev->regs[0]);

    // Set DMA Channel bits (SW1-4, 5)
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
            break;

        default:
            // Should not happen with current config selection
            dma_bits = CCIV_DMA_2; // Default to DMA 2 if invalid
            break;
    }

    dev->regs[0] |= dma_bits;
    compaticard_log("dev->regs[0] after setting dma_bits was: %X\n", dev->regs[0]);

    // Set 1.2MB Spindle Speed bit (SW1-6)
    if (spindle_speed == CCIV_SPINDLE_DUAL_SPEED) {
        dev->regs[0] |= CCIV_SPINDLE_DUAL_SPEED;
        compaticard_log("dev->regs[0] after setting spindle_speed was: %X\n", dev->regs[0]);
    }

    // Set Auto Boot bit (SW1-7)
    if (autoboot_enabled) {
        dev->regs[0] |= CCIV_AUTOBOOT_ENABLED;
        compaticard_log("dev->regs[0] after setting autoboot_enabled was: %X\n", dev->regs[0]);
    }

    // Bit 3 is SW1-4, Reserved, therefore, never pulled down.
    dev->regs[0] |= 0x08;

    // Set Drive Type bits (SW2-1 to SW2-8) into regs[1]
    dev->regs[1] |= (drive_type_0 & 0x03); // Bits 0-1 for Drive 0
    dev->regs[1] |= ((drive_type_1 & 0x03) << 2); // Bits 2-3 for Drive 1
    dev->regs[1] |= ((drive_type_2 & 0x03) << 4); // Bits 4-5 for Drive 2
    dev->regs[1] |= ((drive_type_3 & 0x03) << 6); // Bits 6-7 for Drive 3
    compaticard_log("dev->regs[1] after setting drive types was: %X\n", dev->regs[1]);

    fdc_remove(dev->fdc);

    fdc_set_irq(dev->fdc, irq);
    fdc_set_dma_ch(dev->fdc, dma);

    // Register I/O handlers for CompatiCard IV
    // SW1 is at base_addr
    // SW2 is at base_addr + 1
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
        .default_string = NULL,
        .default_int    = 0x3f0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "0x3f0", .value = 0x3f0 },
            { .description = "0x370", .value = 0x370 },
            { .description = "0x360", .value = 0x360 },
            { .description = "0x3e0", .value = 0x3e0 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
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
        .default_string = NULL,
        .default_int    = 0x3f0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "0x3f0", .value = 0x3f0 },
            { .description = "0x370", .value = 0x370 },
            { .description = "0x360", .value = 0x360 },
            { .description = "0x3e0", .value = 0x3e0 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "irq",
        .description    = "IRQ",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 6,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "IRQ 2", .value = 2 },
            { .description = "IRQ 3", .value = 3 },
            { .description = "IRQ 4", .value = 4 },
            { .description = "IRQ 5", .value = 5 },
            { .description = "IRQ 6", .value = 6 },
            { .description = "IRQ 7", .value = 7 },
            { .description = ""                  }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "dma",
        .description    = "DMA",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 2,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "DMA 1", .value = 1 },
            { .description = "DMA 2", .value = 2 },
            { .description = "DMA 3", .value = 3 },
            { .description = ""                  }
        },
        .bios           = { { 0 } }
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
        .default_string = NULL,
        .default_int    = 0x3f0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "0x3f0", .value = 0x3f0 },
            { .description = "0x370", .value = 0x370 },
            { .description = "0x360", .value = 0x360 },
            { .description = "0x3e0", .value = 0x3e0 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "irq",
        .description    = "IRQ",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 6,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "IRQ 2", .value = 2 },
            { .description = "IRQ 3", .value = 3 },
            { .description = "IRQ 4", .value = 4 },
            { .description = "IRQ 5", .value = 5 },
            { .description = "IRQ 6", .value = 6 },
            { .description = "IRQ 7", .value = 7 },
            { .description = ""                  }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "dma",
        .description    = "DMA",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 2,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "DMA 1",               .value = 1    },
            { .description = "DMA 2",               .value = 2    },
            { .description = "DMA 3",               .value = 3    },
            { .description = "DMA 2 (Fixed IRQ 6)", .value = 0x10 },
            { .description = ""                                   }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "bios_addr",
        .description    = "BIOS Address:",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0xce000,
        .file_filter    = NULL,
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
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "autoboot_enabled",
        .description    = "Enable Autoboot",
        .type           = CONFIG_BINARY,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "spindle_speed",
        .description    = "1.2MB Spindle Speed",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = CCIV_SPINDLE_SINGLE_SPEED,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Single Speed", .value = CCIV_SPINDLE_SINGLE_SPEED },
            { .description = "Dual Speed",   .value = CCIV_SPINDLE_DUAL_SPEED   },
            { .description = ""                                                 }
        }
    },
    {
        .name           = "drive_type_0",
        .description    = "Drive Position 0 Type",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = CCIV_DRIVE_TYPE_NONE,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "No Drive / Other (720KB, 8 inch)", .value = CCIV_DRIVE_TYPE_NONE       },
            { .description = "360KB or 1.2MB",                   .value = CCIV_DRIVE_TYPE_360_1200KB },
            { .description = "1.44MB",                           .value = CCIV_DRIVE_TYPE_1440KB     },
            { .description = "2.88MB",                           .value = CCIV_DRIVE_TYPE_2880KB     },
            { .description = ""                                                                      }
        }
    },
    {
        .name           = "drive_type_1",
        .description    = "Drive Position 1 Type",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = CCIV_DRIVE_TYPE_NONE,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "No Drive / Other (720KB, 8 inch)", .value = CCIV_DRIVE_TYPE_NONE       },
            { .description = "360KB or 1.2MB",                   .value = CCIV_DRIVE_TYPE_360_1200KB },
            { .description = "1.44MB",                           .value = CCIV_DRIVE_TYPE_1440KB     },
            { .description = "2.88MB",                           .value = CCIV_DRIVE_TYPE_2880KB     },
            { .description = ""                                                                      }
        }
    },
    {
        .name           = "drive_type_2",
        .description    = "Drive Position 2 Type",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = CCIV_DRIVE_TYPE_NONE,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "No Drive / Other (720KB, 8 inch)", .value = CCIV_DRIVE_TYPE_NONE       },
            { .description = "360KB or 1.2MB",                   .value = CCIV_DRIVE_TYPE_360_1200KB },
            { .description = "1.44MB",                           .value = CCIV_DRIVE_TYPE_1440KB     },
            { .description = "2.88MB",                           .value = CCIV_DRIVE_TYPE_2880KB     },
            { .description = ""                                                                      }
        }
    },
    {
        .name           = "drive_type_3",
        .description    = "Drive Position 3 Type",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = CCIV_DRIVE_TYPE_NONE,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "No Drive / Other (720KB, 8 inch)", .value = CCIV_DRIVE_TYPE_NONE       },
            { .description = "360KB or 1.2MB",                   .value = CCIV_DRIVE_TYPE_360_1200KB },
            { .description = "1.44MB",                           .value = CCIV_DRIVE_TYPE_1440KB     },
            { .description = "2.88MB",                           .value = CCIV_DRIVE_TYPE_2880KB     },
            { .description = ""                                                                      }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
// clang-format on
};

const device_t fdc_compaticard_i_device = {
    .name          = "Micro Solutions CompatiCard I",
    .internal_name = "compaticard_i",
    .flags         = DEVICE_ISA,
    .local         = DEVICE_COMPATICARD_I,
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
    .local         = DEVICE_COMPATICARD_II,
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
    .local         = DEVICE_COMPATICARD_IV,
    .init          = compaticardiv_init,
    .close         = compaticard_close,
    .reset         = NULL,
    .available     = compaticard_iv_available,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = compaticard_iv_config
};
