#include <stdint.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/pic.h>
#include <86box/timer.h>
#include <86box/serial.h>

#define ECH_PORTS 32
#define ECH_PANELS 4
#define ECH_PANEL_SIZE 8
#define ECH_ID 0xa0
#define ECH_ID_MASK 0xe0
#define ECH_ENABLE 0x08
#define ECH_INT_ENABLE 0x01
#define ECH_INT_LEVEL 0x02
#define ECH_INT_PENDING 0x01
#define ECH_PANEL_PENDING 0x80
#define ECH_PANEL_ID_MASK 0x07
#define ECH_ADDR2_MASK 0x1e0

static const uint8_t stallion_irq_encode[16] = {
    0xff, 0xff, 0xff, 0x04, 0x06, 0x05, 0xff, 0x07,
    0xff, 0xff, 0x00, 0x02, 0x01, 0xff, 0xff, 0x03
};

typedef struct stallion_ech_s {
    uint16_t base;
    uint16_t panel_base;
    uint8_t irq;
    uint8_t control;
    uint8_t enabled;
    uint8_t level;
    uint8_t pending;
    uint8_t irq_state;
    uint8_t address[ECH_PANELS];
    void *chip[ECH_PORTS / 4];
} stallion_ech_t;

static int
stallion_ech_panel_pending(stallion_ech_t *dev, int panel)
{
    return cd1400_service_request(dev->chip[panel * 2]) ||
           cd1400_service_request(dev->chip[panel * 2 + 1]);
}

static void
stallion_ech_update_irq(void *priv)
{
    stallion_ech_t *dev = (stallion_ech_t *) priv;
    int active = 0;
    for (int p = 0; p < ECH_PANELS; p++)
        active |= stallion_ech_panel_pending(dev, p);
    active = active && dev->enabled && (dev->control & ECH_INT_ENABLE);

    if (dev->level) {
        dev->irq_state = (uint8_t) active;
        picintlevel(1u << dev->irq, &dev->irq_state);
    } else if (active && !dev->pending) {
        picint(1u << dev->irq);
    }
    dev->pending = (uint8_t) active;
}

static uint8_t
stallion_ech_io_read(uint16_t port, void *priv)
{
    stallion_ech_t *dev = (stallion_ech_t *) priv;
    if (port == dev->base + 1) {
        stallion_ech_update_irq(dev);
        return (uint8_t) (ECH_ID | (dev->pending ? ECH_INT_PENDING : 0));
    }
    if (port >= dev->panel_base && port < dev->panel_base + ECH_PANELS * ECH_PANEL_SIZE) {
        int panel = (port - dev->panel_base) / ECH_PANEL_SIZE;
        int offset = (port - dev->panel_base) % ECH_PANEL_SIZE;
        if (offset == 2) {
            stallion_ech_update_irq(dev);
            return (uint8_t) (panel | (stallion_ech_panel_pending(dev, panel) ? ECH_PANEL_PENDING : 0));
        }
        if ((offset == 4) && dev->enabled) {
            int chip = panel * 2 + ((dev->address[panel] & 0x80) ? 1 : 0);
            return cd1400_read(dev->address[panel] & 0x7f, dev->chip[chip]);
        }
    }
    return 0xff;
}

static void
stallion_ech_io_write(uint16_t port, uint8_t value, void *priv)
{
    stallion_ech_t *dev = (stallion_ech_t *) priv;
    if (port == dev->base) {
        dev->control = value;
        dev->enabled = 0;
        dev->pending = 0;
        dev->irq_state = 0;
        picintc(1u << dev->irq);
        return;
    }
    if (port == dev->base + 1) {
        dev->control = value;
        dev->enabled = !!(value & ECH_ENABLE);
        dev->level = !!(value & ECH_INT_LEVEL);
        stallion_ech_update_irq(dev);
        return;
    }
    if (port >= dev->panel_base && port < dev->panel_base + ECH_PANELS * ECH_PANEL_SIZE) {
        int panel = (port - dev->panel_base) / ECH_PANEL_SIZE;
        int offset = (port - dev->panel_base) % ECH_PANEL_SIZE;
        if (offset == 0)
            dev->address[panel] = value;
        else if ((offset == 4) && dev->enabled) {
            int chip = panel * 2 + ((dev->address[panel] & 0x80) ? 1 : 0);
            cd1400_write(dev->address[panel] & 0x7f, value, dev->chip[chip]);
            stallion_ech_update_irq(dev);
        }
    }
}

static void *
stallion_ech_init(const device_t *info)
{
    stallion_ech_t *dev = (stallion_ech_t *) calloc(1, sizeof(*dev));
    if (!dev)
        return NULL;
    dev->base = (uint16_t) device_get_config_hex16("base");
    dev->panel_base = (uint16_t) device_get_config_hex16("panel_base");
    dev->irq = (uint8_t) device_get_config_int("irq");
    if (dev->irq > 15 || stallion_irq_encode[dev->irq] == 0xff)
        dev->irq = 5;

    for (int i = 0; i < ECH_PORTS / 4; i++) {
        dev->chip[i] = cd1400_create();
        if (!dev->chip[i]) {
            for (int j = 0; j < i; j++)
                cd1400_destroy(dev->chip[j]);
            free(dev);
            return NULL;
        }
        cd1400_set_irq_callback(dev->chip[i], stallion_ech_update_irq, dev);
    }
    for (int i = 0; i < ECH_PORTS; i++) {
        const int chip = i / 4;
        const int channel = i & 3;
        cd1400_write(0x68, (uint8_t) ((i & 7) << 3), dev->chip[chip]);
        serial_multiport_register_endpoint((uint8_t) i,
                                           cd1400_endpoint(dev->chip[chip], channel), NULL);
    }
    io_sethandler(dev->base, 2, stallion_ech_io_read, NULL, NULL,
                  stallion_ech_io_write, NULL, NULL, dev);
    io_sethandler(dev->panel_base, ECH_PANELS * ECH_PANEL_SIZE,
                  stallion_ech_io_read, NULL, NULL,
                  stallion_ech_io_write, NULL, NULL, dev);
    (void) info;
    return dev;
}

static void
stallion_ech_close(void *priv)
{
    stallion_ech_t *dev = (stallion_ech_t *) priv;
    if (!dev)
        return;
    io_removehandler(dev->base, 2, stallion_ech_io_read, NULL, NULL,
                     stallion_ech_io_write, NULL, NULL, dev);
    io_removehandler(dev->panel_base, ECH_PANELS * ECH_PANEL_SIZE,
                     stallion_ech_io_read, NULL, NULL,
                     stallion_ech_io_write, NULL, NULL, dev);
    picintc(1u << dev->irq);
    for (int i = 0; i < ECH_PORTS; i++)
        serial_multiport_unregister_endpoint((uint8_t) i,
                                             cd1400_endpoint(dev->chip[i / 4], i & 3));
    for (int i = 0; i < ECH_PORTS / 4; i++)
        cd1400_destroy(dev->chip[i]);
    free(dev);
}

static void
stallion_ech_reset(void *priv)
{
    stallion_ech_t *dev = (stallion_ech_t *) priv;
    dev->enabled = dev->pending = dev->irq_state = 0;
    dev->control = dev->address[0] = dev->address[1] = dev->address[2] = dev->address[3] = 0;
    picintc(1u << dev->irq);
    for (int i = 0; i < ECH_PORTS / 4; i++) {
        cd1400_write(0x68, 0, dev->chip[i]);
        cd1400_write(0x05, 0x81, dev->chip[i]);
    }
}

static const device_config_t stallion_ech_config[] = {
    { .name = "base", .description = "Control I/O base", .type = CONFIG_HEX16, .default_int = 0x280,
      .selection = { { .description = "0x280", .value = 0x280 }, { .description = "0x288", .value = 0x288 },
                     { .description = "0x290", .value = 0x290 }, { .description = "0x298", .value = 0x298 },
                     { .description = "" } } },
    { .name = "panel_base", .description = "Panel I/O base", .type = CONFIG_HEX16, .default_int = 0x2c0,
      .selection = { { .description = "0x2C0", .value = 0x2c0 }, { .description = "0x300", .value = 0x300 },
                     { .description = "0x340", .value = 0x340 }, { .description = "0x380", .value = 0x380 },
                     { .description = "" } } },
    { .name = "irq", .description = "IRQ", .type = CONFIG_SELECTION, .default_int = 5,
      .selection = { { .description = "IRQ 3", .value = 3 }, { .description = "IRQ 4", .value = 4 },
                     { .description = "IRQ 5", .value = 5 }, { .description = "IRQ 7", .value = 7 },
                     { .description = "IRQ 10", .value = 10 }, { .description = "IRQ 11", .value = 11 },
                     { .description = "IRQ 12", .value = 12 }, { .description = "IRQ 15", .value = 15 },
                     { .description = "" } } },
    { .name = "", .type = CONFIG_END }
};

const device_t stallion_ech_device = {
    .name = "Stallion EasyConnection 8/32 AT serial adapter",
    .internal_name = "stallion_ech",
    .flags = DEVICE_ISA16,
    .local = 0,
    .init = stallion_ech_init,
    .close = stallion_ech_close,
    .reset = stallion_ech_reset,
    .available = NULL,
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = stallion_ech_config
};
