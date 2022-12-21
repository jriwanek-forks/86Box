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
 *
 *
 * Authors:  Jasmine Iwanek, <jasmine@iwanek.co.uk>
 *
 *           Copyright 2022-2024 Jasmine Iwanek.
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
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/machine.h>
#include <86box/timer.h>
#include <86box/fdd.h>
#include <86box/fdc.h>
#include <86box/fdc_ext.h>

#define BIOS_ADDR (uint32_t)(device_get_config_hex20("bios_addr") & 0x000fffff)
#define ROM_COMPATICARD_IV "roms/floppy/compaticard/ccivbios1.05.bin"

typedef struct
{
    rom_t bios_rom;
    fdc_t *fdc;
    uint8_t regs[1];
} compaticard_t;

static void
compaticard_out(uint16_t port, uint8_t val, void *priv)
{
    compaticard_t *dev = (compaticard_t *) priv;

    dev->regs[1] = val;
}

static uint8_t
compaticard_in(uint16_t port, void *priv)
{
    compaticard_t *dev = (compaticard_t *) priv;
    uint8_t     ret = 0xff;

    ret = dev->regs[1];

    return ret;
}

static void
compaticard_close(void *priv)
{
    compaticard_t *dev = (compaticard_t *)priv;

    free(dev);
}

static void *
compaticard_init(const device_t *info)
{
    compaticard_t *dev;

    dev = (compaticard_t *)malloc(sizeof(compaticard_t));
    memset(dev, 0, sizeof(compaticard_t));

    dev->fdc = device_add(&fdc_at_device);

    uint16_t base_addr = device_get_config_hex16("base");
    uint8_t irq = 6;
    uint8_t dma = 2;
    uint16_t cr2_addr = 0x7f2; // Control Register 2

    // Only on CompatiCard IV
    if ((info->local == 2) && (BIOS_ADDR != 0))
        rom_init(&dev->bios_rom, ROM_COMPATICARD_IV, BIOS_ADDR, 0x2000, 0x1ffff, 0, MEM_MAPPING_EXTERNAL);

    if (base_addr == FDC_SECONDARY_ADDR)
        cr2_addr = 0x772;
    else if (base_addr == FDC_TERTIARY_ADDR)
        cr2_addr = 0x762;
    else if (base_addr == FDC_QUATERNARY_ADDR)
        cr2_addr = 0x7e2;

    if (info->local >= 1) {
        irq = device_get_config_int("irq");
        dma = device_get_config_int("dma");
    }

    fdc_remove(dev->fdc);
    fdc_set_base(dev->fdc, base_addr);
    fdc_set_irq(dev->fdc, irq);
    fdc_set_dma_ch(dev->fdc, dma);

    io_sethandler(cr2_addr, 0x0001,
                  compaticard_in, NULL, NULL,
                  compaticard_out, NULL, NULL, dev);

    return dev;
}

static int compaticard_iv_available(void)
{
    return rom_present(ROM_COMPATICARD_IV);
}

static const device_config_t compaticard_i_config[] = {
// clang-format off
    {
        .name = "base",
        .description = "Address",
        .type = CONFIG_HEX16,
        .default_string = "",
        .default_int = 0x3f0,
        .file_filter = "",
        .spinner = { 0 },
        .selection = {
            {
                .description = "0x3f0",
                .value = 0x3f0
            },
            {
                .description = "0x370",
                .value = 0x370
            },
            {
                .description = "0x360",
                .value = 0x360
            },
            {
                .description = "0x3e0",
                .value = 0x3e0
            },
            { .description = "" }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
// clang-format on
};

static const device_config_t compaticard_ii_config[] = {
// clang-format off
    {
        .name = "base",
        .description = "Address",
        .type = CONFIG_HEX16,
        .default_string = "",
        .default_int = 0x3f0,
        .file_filter = "",
        .spinner = { 0 },
        .selection = {
            {
                .description = "0x3f0",
                .value = 0x3f0
            },
            {
                .description = "0x370",
                .value = 0x370
            },
            {
                .description = "0x360",
                .value = 0x360
            },
            {
                .description = "0x3e0",
                .value = 0x3e0
            },
            { .description = "" }
        }
    },
    {
        .name = "irq",
        .description = "IRQ",
        .type = CONFIG_SELECTION,
        .default_string = "",
        .default_int = 6,
        .file_filter = "",
        .spinner = { 0 },
        .selection = {
            {
                .description = "IRQ 2",
                .value = 2
            },
            {
                .description = "IRQ 3",
                .value = 3
            },
            {
                .description = "IRQ 4",
                .value = 4
            },
            {
                .description = "IRQ 5",
                .value = 5
            },
            {
                .description = "IRQ 6",
                .value = 6
            },
            {
                .description = "IRQ 7",
                .value = 7
            },
            { .description = "" }
        }
    },
    {
        .name = "dma",
        .description = "DMA channel",
        .type = CONFIG_SELECTION,
        .default_string = "",
        .default_int = 2,
        .file_filter = "",
        .spinner = { 0 },
        .selection = {
            {
                .description = "DMA 1",
                .value = 1
            },
            {
                .description = "DMA 2",
                .value = 2
            },
            {
                .description = "DMA 3",
                .value = 3
            },
            { .description = "" }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
// clang-format on
};

static const device_config_t compaticard_iv_config[] = {
// clang-format off
    {
        .name = "base",
        .description = "Address",
        .type = CONFIG_HEX16,
        .default_string = "",
        .default_int = 0x3f0,
        .file_filter = "",
        .spinner = { 0 },
        .selection = {
            {
                .description = "0x3f0",
                .value = 0x3f0
            },
            {
                .description = "0x370",
                .value = 0x370
            },
            {
                .description = "0x360",
                .value = 0x360
            },
            {
                .description = "0x3e0",
                .value = 0x3e0
            },
            { .description = "" }
        }
    },
    {
        .name = "irq",
        .description = "IRQ",
        .type = CONFIG_SELECTION,
        .default_string = "",
        .default_int = 6,
        .file_filter = "",
        .spinner = { 0 },
        .selection = {
            {
                .description = "IRQ 2",
                .value = 2
            },
            {
                .description = "IRQ 3",
                .value = 3
            },
            {
                .description = "IRQ 4",
                .value = 4
            },
            {
                .description = "IRQ 5",
                .value = 5
            },
            {
                .description = "IRQ 6",
                .value = 6
            },
            {
                .description = "IRQ 7",
                .value = 7
            },
            { .description = "" }
        }
    },
    {
        .name = "dma",
        .description = "DMA channel",
        .type = CONFIG_SELECTION,
        .default_string = "",
        .default_int = 2,
        .file_filter = "",
        .spinner = { 0 },
        .selection = {
            {
                .description = "DMA 1",
                .value = 1
            },
            {
                .description = "DMA 2",
                .value = 2
            },
            {
                .description = "DMA 3",
                .value = 3
            },
            { .description = "" }
        }
    },
    {
        .name = "bios_addr",
        .description = "BIOS Address:",
        .type = CONFIG_HEX20,
        .default_string = "",
        .default_int = 0xce000,
        .file_filter = "",
        .spinner = { 0 },
        .selection = {
            { .description = "Disabled", .value = 0 },
            { .description = "CC00H",    .value = 0xcc000 },
            { .description = "CE00H",    .value = 0xce000 },
            { .description = "D000H",    .value = 0xd0000 },
            { .description = "D800H",    .value = 0xd8000 },
            { .description = "DE00H",    .value = 0xde000 },
            { .description = "E000H",    .value = 0xe0000 },
            { .description = "E800H",    .value = 0xe8000 },
            { .description = "EE00H",    .value = 0xee000 },
            { .description = ""                           }
        }
    },
#if 0
    {
        .name = "autoboot_enabled",
        .description = "Enable Autoboot",
        .type = CONFIG_BINARY,
        .default_string = "",
        .default_int = 0
    },
#endif
    { .name = "", .description = "", .type = CONFIG_END }
// clang-format on
};

const device_t fdc_compaticard_i_device = {
    .name = "Micro Solutions CompatiCard I",
    .internal_name = "compaticard_i",
    .flags = DEVICE_ISA,
    .local = 0,
    .init = compaticard_init,
    .close = compaticard_close,
    .reset = NULL,
    { .available = NULL },
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = compaticard_i_config
};

const device_t fdc_compaticard_ii_device = {
    .name = "Micro Solutions CompatiCard II",
    .internal_name = "compaticard_ii",
    .flags = DEVICE_ISA,
    .local = 1,
    .init = compaticard_init,
    .close = compaticard_close,
    .reset = NULL,
    { .available = NULL },
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = compaticard_ii_config
};

const device_t fdc_compaticard_iv_device = {
    .name = "Micro Solutions CompatiCard IV",
    .internal_name = "compaticard_iv",
    .flags = DEVICE_ISA,
    .local = 2,
    .init = compaticard_init,
    .close = compaticard_close,
    .reset = NULL,
    { .available = compaticard_iv_available },
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = compaticard_iv_config
};
