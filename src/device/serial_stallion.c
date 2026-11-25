/* Stallion EasyIO/8 ISA serial board (CD1400 based). */
#include <stdint.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/pic.h>
#include <86box/timer.h>
#include <86box/serial.h>

#define EIO_PORTS       8
#define EIO_BANK_SIZE   8
#define EIO_ADDR        0
#define EIO_DATA        4
#define EIO_RXACK       5
#define EIO_TXACK       6
#define EIO_MDACK       7
#define EIO_ID_8PORT    0x04
#define EIO_INTRPEND    0x08
#define EIO_INTLEVEL    0x08
#define EIO_0WS         0x10

static const uint8_t eio_irq_encode[16] = {
    0xff, 0xff, 0xff, 0x04, 0x06, 0x05, 0xff, 0x07,
    0xff, 0xff, 0x00, 0x02, 0x01, 0xff, 0xff, 0x03
};
static const int eio_irq_decode[8] = { 10, 12, 11, 15, 3, 5, 4, 7 };

typedef struct stallion_easyio_s {
    uint16_t base;
    uint8_t  address;
    uint8_t  control;
    uint8_t  irq;
    uint8_t  irq_state;
    uint8_t  irq_level;
    uint8_t  pending;
    void    *cd1400[2];
} stallion_easyio_t;

static void stallion_update_irq(void *priv);

static uint8_t
stallion_service(stallion_easyio_t *dev, int chip)
{
    return cd1400_service_request(dev->cd1400[chip]);
}

static void
stallion_update_irq(void *priv)
{
    stallion_easyio_t *dev = (stallion_easyio_t *) priv;
    uint8_t pending = stallion_service(dev, 0) | stallion_service(dev, 1);
    uint8_t active = (pending != 0);
    uint32_t irq_mask = 1u << dev->irq;

    if (dev->irq_level) {
        dev->irq_state = active;
        picintlevel(irq_mask, &dev->irq_state);
    } else if (active && !dev->pending) {
        picint(irq_mask);
    }
    dev->pending = active;
}

static uint8_t
stallion_ack(stallion_easyio_t *dev, uint8_t type)
{
    uint8_t ack = cd1400_service_ack(dev->cd1400[0], type);
    if (ack == 0xff)
        ack = cd1400_service_ack(dev->cd1400[1], type);
    stallion_update_irq(dev);
    return ack;
}

static uint8_t
stallion_read(uint16_t port, void *priv)
{
    stallion_easyio_t *dev = (stallion_easyio_t *) priv;
    switch (port - dev->base) {
        case EIO_ADDR:
            return 0xff;
        case 1:
            return 0xff;
        case 2:
            stallion_update_irq(dev);
            return (uint8_t) (EIO_ID_8PORT | (dev->pending ? EIO_INTRPEND : 0));
        case 3:
            return 0xff;
        case EIO_DATA: {
            int chip = (dev->address & 0x80) ? 1 : 0;
            uint8_t value = cd1400_read(dev->address & 0x7f, dev->cd1400[chip]);
            stallion_update_irq(dev);
            return value;
        }
        case EIO_RXACK:
            return stallion_ack(dev, 0x01);
        case EIO_TXACK:
            return stallion_ack(dev, 0x02);
        case EIO_MDACK:
            return stallion_ack(dev, 0x04);
        default:
            return 0xff;
    }
}

static void
stallion_write(uint16_t port, uint8_t value, void *priv)
{
    stallion_easyio_t *dev = (stallion_easyio_t *) priv;
    switch (port - dev->base) {
        case EIO_ADDR:
            dev->address = value;
            break;
        case 1: {
            uint8_t old_irq = dev->irq;
            uint8_t old_level = dev->irq_level;
            dev->control = value;
            dev->irq_level = !!(value & EIO_INTLEVEL);
            dev->irq = (uint8_t) eio_irq_decode[value & 7];
            if ((dev->irq != old_irq) || (dev->irq_level != old_level)) {
                picintc(1u << old_irq);
                dev->irq_state = 0;
                dev->pending = 0;
                stallion_update_irq(dev);
            }
            break;
        }
        case EIO_DATA: {
            int chip = (dev->address & 0x80) ? 1 : 0;
            cd1400_write(dev->address & 0x7f, value, dev->cd1400[chip]);
            stallion_update_irq(dev);
            break;
        }
        default:
            break;
    }
}

static void *
stallion_init(const device_t *info)
{
    stallion_easyio_t *dev = (stallion_easyio_t *) calloc(1, sizeof(*dev));
    int irq = device_get_config_int("irq");
    int vector;
    (void) info;

    if (dev == NULL)
        return NULL;
    dev->base = (uint16_t) device_get_config_hex16("base");
    if ((irq < 0) || (irq > 15) || (eio_irq_encode[irq] == 0xff))
        irq = 5;
    vector = eio_irq_encode[irq];
    dev->control = (uint8_t) (vector | EIO_0WS);
    dev->irq = (uint8_t) irq;
    dev->cd1400[0] = cd1400_create();
    dev->cd1400[1] = cd1400_create();
    if ((dev->cd1400[0] == NULL) || (dev->cd1400[1] == NULL)) {
        cd1400_destroy(dev->cd1400[0]);
        cd1400_destroy(dev->cd1400[1]);
        free(dev);
        return NULL;
    }
    cd1400_set_irq_callback(dev->cd1400[0], stallion_update_irq, dev);
    cd1400_set_irq_callback(dev->cd1400[1], stallion_update_irq, dev);

    /* Stallion's host driver assigns a board-wide channel number in LIVR.
       This keeps the CD1400 acknowledge byte useful to the board layer. */
    for (int port = 0; port < EIO_PORTS; port++) {
        int chip = port >> 2;
        cd1400_write(0x68, (uint8_t) (port << 3), dev->cd1400[chip]);
        cd1400_write(0x18, (uint8_t) (port << 3), dev->cd1400[chip]);
        serial_multiport_register_endpoint((uint8_t) port,
                                           cd1400_endpoint(dev->cd1400[chip], port & 3), NULL);
    }

    io_sethandler(dev->base, EIO_BANK_SIZE, stallion_read, NULL, NULL,
                  stallion_write, NULL, NULL, dev);
    return dev;
}

static void
stallion_close(void *priv)
{
    stallion_easyio_t *dev = (stallion_easyio_t *) priv;
    if (dev == NULL)
        return;
    io_removehandler(dev->base, EIO_BANK_SIZE, stallion_read, NULL, NULL,
                     stallion_write, NULL, NULL, dev);
    picintc(1u << dev->irq);
    for (int port = 0; port < EIO_PORTS; port++)
        serial_multiport_unregister_endpoint((uint8_t) port,
                                             cd1400_endpoint(dev->cd1400[port >> 2], port & 3));
    cd1400_destroy(dev->cd1400[0]);
    cd1400_destroy(dev->cd1400[1]);
    free(dev);
}

static void
stallion_reset(void *priv)
{
    stallion_easyio_t *dev = (stallion_easyio_t *) priv;
    picintc(1u << dev->irq);
    dev->pending = 0;
    dev->irq_state = 0;
    dev->address = 0;
    for (int chip = 0; chip < 2; chip++) {
        cd1400_write(0x68, 0, dev->cd1400[chip]);
        cd1400_write(0x05, 0x81, dev->cd1400[chip]);
    }
    stallion_update_irq(dev);
}

static const device_config_t stallion_easyio_config[] = {
    {
        .name = "base", .description = "I/O base address", .type = CONFIG_HEX16,
        .default_int = 0x2a0,
        .selection = {
            { .description = "0x200", .value = 0x200 },
            { .description = "0x280", .value = 0x280 },
            { .description = "0x2A0", .value = 0x2a0 },
            { .description = "0x300", .value = 0x300 },
            { .description = "0x320", .value = 0x320 },
            { .description = "0x3E0", .value = 0x3e0 },
            { .description = "0x3F0", .value = 0x3f0 },
            { .description = "" }
        }
    },
    {
        .name = "irq", .description = "IRQ", .type = CONFIG_SELECTION,
        .default_int = 5,
        .selection = {
            { .description = "IRQ 3", .value = 3 },
            { .description = "IRQ 4", .value = 4 },
            { .description = "IRQ 5", .value = 5 },
            { .description = "IRQ 7", .value = 7 },
            { .description = "IRQ 10", .value = 10 },
            { .description = "IRQ 11", .value = 11 },
            { .description = "IRQ 12", .value = 12 },
            { .description = "IRQ 15", .value = 15 },
            { .description = "" }
        }
    },
    { .name = "", .type = CONFIG_END }
};

const device_t stallion_easyio_device = {
    .name = "Stallion EasyIO/8 ISA serial adapter",
    .internal_name = "stallion_easyio",
    .flags = DEVICE_ISA16,
    .local = 0,
    .init = stallion_init,
    .close = stallion_close,
    .reset = stallion_reset,
    .available = NULL,
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = stallion_easyio_config
};
