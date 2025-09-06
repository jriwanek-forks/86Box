/*
 * 86Box     A hypervisor and IBM PC system emulator that specializes in
 *           running old operating systems and software designed for IBM
 *           PC systems and compatibles from 1981 through fairly recent
 *           system designs based on the PCI bus.
 *
 *           This file is part of the 86Box distribution.
 *
 *           USB MTD Emulation.
 *
 * Authors:  Cacodemon345
 *
 *           Copyright 2024-2025 Cacodemon345.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/io.h>
#include <86box/timer.h>
#include <86box/pcmcia.h>
#include <86box/mem.h>
#include <86box/pic.h>
#include <86box/device.h>
#include <86box/nvr.h>
#include <86box/plat.h>

#include "cpu.h"

static const uint8_t cis_data[] = {
	0x01, 0x3,
    0x04 | (0x06 << 4), (0x1F << 3) | 6, 0xFF,

    0x21, 2,
    0x01, 0x00,

    0x15, 13,
    0x04, 0x01,
    0x38, 0x36, 0x42, 0x6f, 0x78, 0x00,
    0x4d, 0x45, 0x4d, 0x00,
    0xFF,

    0x43, 2, 0, 0,

    0x18, 3,
    0x00, 0x00, 0xFF,

    0x1E, 6,
    2, 1, 1, 1, 1, 1,

    0x41, 21,
    0, 0,
    0, 0, 0, 0, 0x00, 0x00, 0x00, 0x04,
    0x00, 0x02,
    0x00, 0x00, 0x02, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00,

    0x42, 4,
    2, 1, 0xFF, 0xFF,

    0x46, 5,
    0, 0x44, 0x4f, 0x53, 0,

    0x14, 0x00,
	0xff, 0x00
};

typedef struct
{
    /* PCMCIA socket instance. */
    pcmcia_socket_t *pcmcia_socket;

    FILE *rw_file;

    plat_file_mapping_t map;
} pcmcia_mtd_t;

static void
mtd_pcmcia_write(uint32_t addr, uint8_t val, int reg, void *priv)
{
    pcmcia_mtd_t *mtd = priv;
    addr &= 0x3FFFFFF;

    if (reg == 0)
        addr &= ~1;
    else {
        mtd->map.mapped[addr] = val;
        return;
    }

    return;
}

static void
mtd_pcmcia_writew(uint32_t addr, uint16_t val, int reg, void *priv)
{
    mtd_pcmcia_write(addr, val & 0xFF, reg, priv);
    mtd_pcmcia_write(addr + 1, (val >> 8) & 0xFF, reg, priv);
}

static void *
mtd_mem_get_exec(uint32_t addr, void *priv)
{
    pcmcia_mtd_t *mtd = priv;

    return &mtd->map.mapped[addr & 0x3FFFFFF];
}

static uint8_t
mtd_pcmcia_read(uint32_t addr, int reg, void *priv)
{
    pcmcia_mtd_t *mtd = priv;
    addr &= 0x3FFFFFF;

    if (reg == 0)
        addr &= ~1;
    else {
        return mtd->map.mapped[addr];
    }

    addr &= 0x3FFFFFF;

    if ((addr >> 1) < sizeof(cis_data))
        return cis_data[(addr >> 1)];

    return 0xFF;
}

static uint16_t
mtd_pcmcia_readw(uint32_t addr, int reg, void *priv)
{
    return mtd_pcmcia_read(addr, reg, priv) | (mtd_pcmcia_read(addr + 1, reg, priv) << 8);
}

static void
mtd_reset(void *priv)
{
    //
}

static void
mtd_ata_mode(bool ata, pcmcia_socket_t *socket)
{
    //
}

static void *
mtd_init(const device_t *info)
{
    pcmcia_mtd_t    *dev  = calloc(1, sizeof(pcmcia_mtd_t));
    pcmcia_socket_t *slot = pcmcia_search_for_slots();
    if (!slot) {
        free(dev);
        return NULL;
    }

    dev->rw_file = nvr_fopen("pcmcia_mtd.bin", "rb+");
    if (!dev->rw_file) {
        dev->rw_file = nvr_fopen("pcmcia_mtd.bin", "wb+");
        if (dev->rw_file) {
            void *zeroed_ram = calloc(64, 1024 * 1024);
            fwrite(zeroed_ram, 64, 1024 * 1024, dev->rw_file);
            fseek(dev->rw_file, 0, SEEK_SET);
            free(zeroed_ram);
        }
    }

    if (!dev->rw_file) {
        free(dev);
        return NULL;
    }

    dev->map = plat_mmap_file(dev->rw_file);

    slot->mem_read       = mtd_pcmcia_read;
    slot->mem_write      = mtd_pcmcia_write;
    slot->mem_readw      = mtd_pcmcia_readw;
    slot->mem_writew     = mtd_pcmcia_writew;
    slot->reset          = mtd_reset;
    slot->ata_mode       = mtd_ata_mode;
    slot->mem_get_exec   = mtd_mem_get_exec;
    slot->device_name[0] = 0;
    strcpy(slot->device_name, info->name);

    slot->card_priv_unconnected = dev;
    dev->pcmcia_socket          = slot;
    pcmcia_socket_insert_card(slot, dev);
    slot->ready_changed(true, slot);

    return dev;
}

static void
mtd_close(void *priv)
{
    pcmcia_mtd_t *dev = priv;
    plat_munmap_file(&dev->map);
    fclose(dev->rw_file);
    free(dev);
}

const device_t pcmcia_mtd_device = {
    .name          = "Generic S-RAM MTD card",
    .internal_name = "mtd",
    .flags         = DEVICE_PCMCIA,
    .local         = 0,
    .init          = mtd_init,
    .close         = mtd_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
