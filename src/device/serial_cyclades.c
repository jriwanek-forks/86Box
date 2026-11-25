#include <stdint.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/mem.h>
#include <86box/pic.h>
#include <86box/timer.h>
#include <86box/serial.h>

#define CYCLADES_CHIPS 8
#define CYCLADES_CHANNELS (CYCLADES_CHIPS * 4)
#define CYCLADES_WINDOW 0x2000

static const uint16_t chip_offset[CYCLADES_CHIPS] = {
    0x0000, 0x0400, 0x0800, 0x0c00, 0x0200, 0x0600, 0x0a00, 0x0e00
};

typedef struct cyclades_y_s {
    uint32_t base;
    uint8_t irq;
    uint8_t irq_pending;
    uint8_t irq_state;
    void *chip[CYCLADES_CHIPS];
    mem_mapping_t mapping;
} cyclades_y_t;

static void
cyclades_update_irq(void *priv)
{
    cyclades_y_t *dev = (cyclades_y_t *) priv;
    int active = 0;
    for (int i = 0; i < CYCLADES_CHIPS; i++)
        active |= cd1400_service_request(dev->chip[i]) != 0;

    if (active && !dev->irq_pending)
        picint(1u << dev->irq);
    dev->irq_pending = (uint8_t) active;
}

static uint8_t
cyclades_read(uint32_t addr, void *priv)
{
    cyclades_y_t *dev = (cyclades_y_t *) priv;
    uint32_t offset = addr - dev->base;
    for (int i = 0; i < CYCLADES_CHIPS; i++) {
        if ((offset >= chip_offset[i]) && (offset < chip_offset[i] + 0x100) && !(offset & 1))
            return cd1400_read((uint16_t) ((offset - chip_offset[i]) >> 1), dev->chip[i]);
    }
    return 0xff;
}

static void
cyclades_write(uint32_t addr, uint8_t val, void *priv)
{
    cyclades_y_t *dev = (cyclades_y_t *) priv;
    uint32_t offset = addr - dev->base;
    if (offset == 0x1400) {
        for (int i = 0; i < CYCLADES_CHIPS; i++)
            cd1400_write(0x05, 0x81, dev->chip[i]);
        dev->irq_pending = 0;
        picintc(1u << dev->irq);
        return;
    }
    if (offset == 0x1800) {
        dev->irq_pending = 0;
        picintc(1u << dev->irq);
        cyclades_update_irq(dev);
        return;
    }
    for (int i = 0; i < CYCLADES_CHIPS; i++) {
        if ((offset >= chip_offset[i]) && (offset < chip_offset[i] + 0x100) && !(offset & 1)) {
            cd1400_write((uint16_t) ((offset - chip_offset[i]) >> 1), val, dev->chip[i]);
            cyclades_update_irq(dev);
            return;
        }
    }
}

static void *
cyclades_init(const device_t *info)
{
    cyclades_y_t *dev = (cyclades_y_t *) calloc(1, sizeof(*dev));
    if (!dev)
        return NULL;
    dev->base = (uint32_t) device_get_config_hex20("base");
    dev->irq = (uint8_t) device_get_config_int("irq");
    if (dev->irq > 15)
        dev->irq = 5;

    for (int i = 0; i < CYCLADES_CHIPS; i++) {
        dev->chip[i] = cd1400_create();
        if (!dev->chip[i]) {
            for (int j = 0; j < i; j++)
                cd1400_destroy(dev->chip[j]);
            free(dev);
            return NULL;
        }
        cd1400_set_irq_callback(dev->chip[i], cyclades_update_irq, dev);
        for (int c = 0; c < 4; c++) {
            const int channel = i * 4 + c;
            cd1400_write(0x68, (uint8_t) (channel & 3), dev->chip[i]);
            serial_multiport_register_endpoint((uint8_t) channel,
                                               cd1400_endpoint(dev->chip[i], c), NULL);
        }
    }
    mem_mapping_add(&dev->mapping, dev->base, CYCLADES_WINDOW,
                    cyclades_read, NULL, NULL, cyclades_write, NULL, NULL,
                    NULL, MEM_MAPPING_EXTERNAL, dev);
    (void) info;
    return dev;
}

static void
cyclades_close(void *priv)
{
    cyclades_y_t *dev = (cyclades_y_t *) priv;
    if (!dev)
        return;
    mem_mapping_disable(&dev->mapping);
    picintc(1u << dev->irq);
    for (int i = 0; i < CYCLADES_CHANNELS; i++)
        serial_multiport_unregister_endpoint((uint8_t) i,
                                             cd1400_endpoint(dev->chip[i / 4], i & 3));
    for (int i = 0; i < CYCLADES_CHIPS; i++)
        cd1400_destroy(dev->chip[i]);
    free(dev);
}

static void
cyclades_reset(void *priv)
{
    cyclades_y_t *dev = (cyclades_y_t *) priv;
    dev->irq_pending = dev->irq_state = 0;
    picintc(1u << dev->irq);
    for (int i = 0; i < CYCLADES_CHIPS; i++) {
        cd1400_write(0x68, 0, dev->chip[i]);
        cd1400_write(0x05, 0x81, dev->chip[i]);
    }
}

static const device_config_t cyclades_config[] = {
    { .name = "base", .description = "Memory window", .type = CONFIG_HEX20, .default_int = 0xd0000,
      .selection = { { .description = "0xD0000", .value = 0xd0000 }, { .description = "0xD2000", .value = 0xd2000 },
                     { .description = "0xD4000", .value = 0xd4000 }, { .description = "0xD6000", .value = 0xd6000 },
                     { .description = "0xD8000", .value = 0xd8000 }, { .description = "0xDA000", .value = 0xda000 },
                     { .description = "0xDC000", .value = 0xdc000 }, { .description = "0xDE000", .value = 0xde000 },
                     { .description = "" } } },
    { .name = "irq", .description = "IRQ", .type = CONFIG_SELECTION, .default_int = 5,
      .selection = { { .description = "IRQ 3", .value = 3 }, { .description = "IRQ 4", .value = 4 },
                     { .description = "IRQ 5", .value = 5 }, { .description = "IRQ 7", .value = 7 },
                     { .description = "IRQ 10", .value = 10 }, { .description = "IRQ 11", .value = 11 },
                     { .description = "IRQ 12", .value = 12 }, { .description = "IRQ 15", .value = 15 },
                     { .description = "" } } },
    { .name = "", .type = CONFIG_END }
};

const device_t cyclades_cyclomy_device = {
    .name = "Cyclades Cyclom-Y ISA serial adapter",
    .internal_name = "cyclades_cyclomy",
    .flags = DEVICE_ISA16,
    .local = 0,
    .init = cyclades_init,
    .close = cyclades_close,
    .reset = cyclades_reset,
    .available = NULL,
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = cyclades_config
};
