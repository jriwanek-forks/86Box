/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          ISA ROM Board Emulation.
 *
 *
 *
 * Authors: Jasmine Iwanek
 *
 *          Copyright 2024-2025 Jasmine Iwanek
 */

/*
 * TODO:
 * https://github.com/monotech/ISA-DoubleROM
 * ISA FDC (https://github.com/skiselev/isa-fdc)
 * LBA Enhancer
 * Monster FDC
 * XT-IDE
 * Writable ROM Support
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/io.h>
#include <86box/device.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/plat_unused.h>

typedef struct rom_board_t
{
    uint32_t bios_addr;
    uint32_t bios_size;
    uint32_t bios_mask;
	
    rom_t    bios_rom;
} rom_board_t;

#define BIOS_MISC "roms/hdd/misc/bios.bin"

void
rom_board_close(void* priv)
{
    free(priv);

    return;
}

void *
rom_board_init(const device_t *info)
{
    rom_board_t *dev = (rom_board_t *) calloc(1, sizeof(rom_board_t));

    dev->bios_addr = device_get_config_hex20("bios_addr");
	dev->bios_size = device_get_config_hex20("bios_size");
	dev->bios_mask = dev->bios_size - 1;

    rom_init(&dev->bios_rom, BIOS_MISC,
             dev->bios_addr, dev->bios_size, dev->bios_mask, 0, MEM_MAPPING_EXTERNAL);

    return dev;
}

// clang-format off
static const device_config_t rom_board_config[] = {
    {
        .name           = "bios_addr",
        .description    = "BIOS Address",
        .type           = CONFIG_HEX20,
        .default_string = "",
        .default_int    = 0xc8000,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "C800H", .value = 0xc8000 },
            { .description = "CC00H", .value = 0xcc000 },
            { .description = "D000H", .value = 0xd0000 },
            { .description = "D400H", .value = 0xd4000 },
            { .description = "D800H", .value = 0xd8000 },
            { .description = "DC00H", .value = 0xdc000 },
            { .description = ""                           }
        },
    },
    {
        .name           = "bios_size",
        .description    = "BIOS Size",
        .type           = CONFIG_HEX20,
        .default_string = "",
        .default_int    = 0x4000,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "4K",  .value = 0x1000  },
            { .description = "8K",  .value = 0x2000  },
            { .description = "16K", .value = 0x4000  },
            { .description = "32K", .value = 0x80000 },
            { .description = "64K", .value = 0x10000 },
            { .description = ""                      }
        },
    },
    { .name = "", .description = "", .type = CONFIG_END }
};
// clang-format on

const device_t rom_board_device = {
    .name          = "Generic ROM Board",
    .internal_name = "romboard",
    .flags         = DEVICE_AT,
    .local         = 0,
    .init          = rom_board_init,
    .close         = rom_board_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = rom_board_config
};
