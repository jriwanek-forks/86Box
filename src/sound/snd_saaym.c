#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/snd_cms.h>
#include <86box/snd_saaym.h>
#include <86box/sound.h>
#include <86box/plat_unused.h>

void
saaym_write(uint16_t addr, uint8_t val, void *priv)
{
    saaym_t *saaym = (saaym_t *) priv;

    switch (addr & 0xf) {
        case 0xe: /* SAAYM Register Select Port */
            saaym->addr = val & 31;
            break;

        case 0xf: /* SAAYM Data Port */
            /* TODO: Not yet Implemented */
            break;

        case 0x8: /* SAAYM Write Port */
        case 0x9: /* SAAYM Write Port */
            saaym->latched_data = val;
            break;

        default:
            cms_write(addr, val, priv);
            break;
    }
}

uint8_t
saaym_read(uint16_t addr, void *priv)
{
    const saaym_t *saaym = (saaym_t *) priv;

    switch (addr & 0xf) {
        case 0xc: /* SAAYM Read Port */
        case 0xd: /* SAAYM Read Port */
            return saaym->latched_data;
        case 0xe: /* YM2151 Register Select Port */
            return saaym->addr;

        default:
            cms_read(addr, (void *) &saaym->cms);
            break;
    }
    return 0xff;
}

void *
saaym_init(UNUSED(const device_t *info))
{
    saaym_t *saaym = malloc(sizeof(saaym_t));
    memset(saaym, 0, sizeof(saaym_t));

    memset(&saaym->cms, 0, sizeof(cms_t));

    uint16_t addr = device_get_config_hex16("base");
	/* Do this properly and pass through the CMS */
    io_sethandler(addr, 0x0010, saaym_read, NULL, NULL, saaym_write, NULL, NULL, saaym);
    sound_add_handler(cms_get_buffer, &saaym->cms);
    return saaym;
}

void
saaym_close(void *priv)
{
    saaym_t *saaym = (saaym_t *) priv;

    free(saaym);
}

static const device_config_t saaym_config[] = {
  // clang-format off
    {
        .name = "base",
        .description = "Address",
        .type = CONFIG_HEX16,
        .default_string = "",
        .default_int = 0x220,
        .file_filter = "",
        .spinner = { 0 },
        .selection = {
            {
                .description = "0x200",
                .value = 0x200
            },
            {
                .description = "0x210",
                .value = 0x210
            },
            {
                .description = "0x220",
                .value = 0x220
            },
            {
                .description = "0x230",
                .value = 0x230
            },
            {
                .description = "0x240",
                .value = 0x240
            },
            {
                .description = "0x250",
                .value = 0x250
            },
            {
                .description = "0x260",
                .value = 0x260
            },
            {
                .description = "0x270",
                .value = 0x270
            },
            {
                .description = ""
            }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

const device_t saaym_device = {
    .name          = "Texelec SAAYM",
    .internal_name = "saaym",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = saaym_init,
    .close         = saaym_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = saaym_config
};
