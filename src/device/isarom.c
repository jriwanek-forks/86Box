/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of ISA ROM card Expansions.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025 Jasmine Iwanek.
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
#include <86box/nvr.h>
#include <86box/plat_unused.h>
#include <86box/isarom.h>

#define ISAROM_CARD      0
#define ISAROM_CARD_DUAL 1

#ifdef ENABLE_ISAROM_LOG
int isarom_do_log = ENABLE_ISAROM_LOG;

static void
isarom_log(const char *fmt, ...)
{
    va_list ap;

    if (isarom_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define isarom_log(fmt, ...)
#endif

typedef struct isarom_t {
    rom_t       rom;
    uint16_t    rom_addr;
    const char *rom_fn;
    uint16_t    rom_size;
    uint16_t    rom_len;
    char        nvr_path[64];

    rom_t       rom2;
    uint16_t    rom2_addr;
    const char *rom2_fn;
    uint16_t    rom2_size;
    uint16_t    rom2_len;
    char        nvr2_path[64];
} isarom_t;

void
isarom_close(void *priv)
{
    isarom_t *dev = (isarom_t *) priv;

    if (dev->nvr_path[0] != 0x00) {
        FILE *fp = nvr_fopen(dev->nvr_path, "wb");
        if (fp != NULL) {
            fwrite(dev->rom.rom, 1, 0x2000, fp);
            fclose(fp);
        }
    }

    if (dev->nvr2_path[0] != 0x00) {
        FILE *fp = nvr_fopen(dev->nvr2_path, "wb");
        if (fp != NULL) {
            fwrite(dev->rom2.rom, 1, 0x2000, fp);
            fclose(fp);
        }
    }

    free(dev);

    return;
}

void *
isarom_init(UNUSED(const device_t *info))
{
    isarom_t *dev = (isarom_t *) calloc(1, sizeof(isarom_t));
    uint8_t   rom2_writes_enabled = 0;

    dev->rom_addr = device_get_config_hex20("bios_addr");
    dev->rom_fn   = device_get_config_string("bios_fn");
    dev->rom_size = 0x4000;
    dev->rom_len  = 0x3fff;
    uint8_t rom_writes_enabled = device_get_config_int("rom_writes_enabled");

    if (info->local == ISAROM_CARD_DUAL) {
        dev->rom2_addr = device_get_config_hex20("bios_addr2");
        dev->rom2_fn   = device_get_config_string("bios_fn2");
        dev->rom2_size = 0x4000;
        dev->rom2_len  = 0x3fff;
        rom2_writes_enabled = device_get_config_int("rom2_writes_enabled");
    }

    if (rom_writes_enabled) {
        mem_mapping_set_write_handler(&dev->rom.mapping, rom_write, rom_writew, rom_writel);
        sprintf(dev->nvr_path, "isarom_%i_1.nvr", device_get_instance());
        FILE *fp = nvr_fopen(dev->nvr_path, "rb");
        if (fp != NULL) {
            fread(dev->rom.rom, 1, 0x2000, fp);
            fclose(fp);
        }
    }

    if (dev->rom_addr != 0) 
        rom_init(&dev->rom, dev->rom_fn,
                 dev->rom_addr, dev->rom_size, dev->rom_len, 0, MEM_MAPPING_EXTERNAL);

    if (info->local == ISAROM_CARD_DUAL) {
        if (rom2_writes_enabled) {
            mem_mapping_set_write_handler(&dev->rom2.mapping, rom_write, rom_writew, rom_writel);
            sprintf(dev->nvr2_path, "isarom_%i_2.nvr", device_get_instance());
            FILE *fp = nvr_fopen(dev->nvr2_path, "rb");
            if (fp != NULL) {
                fread(dev->rom2.rom, 1, 0x2000, fp);
                fclose(fp);
            }
        }
    }

    if (dev->rom2_addr != 0) 
        rom_init(&dev->rom2, dev->rom2_fn,
                 dev->rom2_addr, dev->rom2_size, dev->rom2_len, 0, MEM_MAPPING_EXTERNAL);

    return dev;
}

// clang-format off
static const device_config_t isarom_config[] = {
    {
        .name           = "bios_fn",
        .description    = "BIOS File",
        .type           = CONFIG_FNAME,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = "Rom files (*.bin)|*.bin",
        .spinner        = { 0 },
        .selection      = { },
        .bios           = { { 0 } }
    },
    {
        .name           = "bios_addr",
        .description    = "BIOS Address (ROM #1)",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0x00000,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0x00000 },
            { .description = "C000H",    .value = 0xc0000 },
            { .description = "C200H",    .value = 0xc2000 },
            { .description = "C400H",    .value = 0xc4000 },
            { .description = "C600H",    .value = 0xc6000 },
            { .description = "C800H",    .value = 0xc8000 },
            { .description = "CA00H",    .value = 0xca000 },
            { .description = "CC00H",    .value = 0xcc000 },
            { .description = "CE00H",    .value = 0xce000 },
            { .description = "D000H",    .value = 0xd0000 },
            { .description = "D200H",    .value = 0xd2000 },
            { .description = "D400H",    .value = 0xd4000 },
            { .description = "D600H",    .value = 0xd6000 },
            { .description = "D800H",    .value = 0xd8000 },
            { .description = "DA00H",    .value = 0xda000 },
            { .description = "DC00H",    .value = 0xdc000 },
            { .description = "DE00H",    .value = 0xde000 },
            { .description = ""                           }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "rom_writes_enabled",
        .description    = "Enable BIOS extension ROM Writes",
        .type           = CONFIG_BINARY,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};

static const device_config_t isarom_dual_config[] = {
    {
        .name           = "bios_fn",
        .description    = "BIOS File (ROM #1)",
        .type           = CONFIG_FNAME,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = "Rom files (*.bin)|*.bin",
        .spinner        = { 0 },
        .selection      = { },
        .bios           = { { 0 } }
    },
    {
        .name           = "bios_addr",
        .description    = "BIOS Address (ROM #1)",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0x00000,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0x00000 },
            { .description = "C000H",    .value = 0xc0000 },
            { .description = "C200H",    .value = 0xc2000 },
            { .description = "C400H",    .value = 0xc4000 },
            { .description = "C600H",    .value = 0xc6000 },
            { .description = "C800H",    .value = 0xc8000 },
            { .description = "CA00H",    .value = 0xca000 },
            { .description = "CC00H",    .value = 0xcc000 },
            { .description = "CE00H",    .value = 0xce000 },
            { .description = "D000H",    .value = 0xd0000 },
            { .description = "D200H",    .value = 0xd2000 },
            { .description = "D400H",    .value = 0xd4000 },
            { .description = "D600H",    .value = 0xd6000 },
            { .description = "D800H",    .value = 0xd8000 },
            { .description = "DA00H",    .value = 0xda000 },
            { .description = "DC00H",    .value = 0xdc000 },
            { .description = "DE00H",    .value = 0xde000 },
            { .description = ""                           }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "rom_writes_enabled",
        .description    = "Enable BIOS extension ROM Writes (ROM #1)",
        .type           = CONFIG_BINARY,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "bios_fn2",
        .description    = "BIOS File (ROM #2)",
        .type           = CONFIG_FNAME,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = "Rom files (*.bin)|*.bin",
        .spinner        = { 0 },
        .selection      = { },
        .bios           = { { 0 } }
    },
    {
        .name           = "bios_addr2",
        .description    = "BIOS Address (ROM #2)",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0x00000,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0x00000 },
            { .description = "C000H",    .value = 0xc0000 },
            { .description = "C200H",    .value = 0xc2000 },
            { .description = "C400H",    .value = 0xc4000 },
            { .description = "C600H",    .value = 0xc6000 },
            { .description = "C800H",    .value = 0xc8000 },
            { .description = "CA00H",    .value = 0xca000 },
            { .description = "CC00H",    .value = 0xcc000 },
            { .description = "CE00H",    .value = 0xce000 },
            { .description = "D000H",    .value = 0xd0000 },
            { .description = "D200H",    .value = 0xd2000 },
            { .description = "D400H",    .value = 0xd4000 },
            { .description = "D600H",    .value = 0xd6000 },
            { .description = "D800H",    .value = 0xd8000 },
            { .description = "DA00H",    .value = 0xda000 },
            { .description = "DC00H",    .value = 0xdc000 },
            { .description = "DE00H",    .value = 0xde000 },
            { .description = ""                           }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "rom_writes_enabled2",
        .description    = "Enable BIOS extension ROM Writes (ROM #2)",
        .type           = CONFIG_BINARY,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};
// clang-format on

static const device_t isarom_device = {
    .name          = "Generic ISA ROM Board",
    .internal_name = "isarom",
    .flags         = DEVICE_ISA,
    .local         = ISAROM_CARD,
    .init          = isarom_init,
    .close         = isarom_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = isarom_config
};

static const device_t isarom_dual_device = {
    .name          = "Generic Dual ISA ROM Board",
    .internal_name = "isarom_dual",
    .flags         = DEVICE_ISA,
    .local         = ISAROM_CARD_DUAL,
    .init          = isarom_init,
    .close         = isarom_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = isarom_dual_config
};

static const struct {
    const device_t *dev;
} boards[] = {
    // clang-format off
    { &device_none        },
    { &isarom_device      },
    { &isarom_dual_device },
    { NULL                }
    // clang-format on
};

void
isarom_reset(void)
{
    for (uint8_t i = 0; i < ISAROM_MAX; i++) {
        if (isarom_type[i] == 0)
            continue;

        /* Add the device instance to the system. */
        device_add_inst(boards[isarom_type[i]].dev, i + 1);
    }
}

const char *
isarom_get_name(int board)
{
    if (boards[board].dev == NULL)
        return (NULL);

    return (boards[board].dev->name);
}

const char *
isarom_get_internal_name(int board)
{
    return device_get_internal_name(boards[board].dev);
}

int
isarom_get_from_internal_name(const char *str)
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
isarom_get_device(int board)
{
    /* Add the device instance to the system. */
    return boards[board].dev;
}

int
isarom_has_config(int board)
{
    if (boards[board].dev == NULL)
        return 0;

    return (boards[board].dev->config ? 1 : 0);
}
