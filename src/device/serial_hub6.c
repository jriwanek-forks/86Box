#include <stdint.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/pic.h>
#include <86box/timer.h>
#include <86box/serial.h>

#define HUB6_PORTS 12
#define HUB6_INDEX 0x302
#define HUB6_DATA  0x303
#define HUB6_IRQ   3

typedef struct hub6_serial_s {
    uint8_t selected;
    uint16_t irq_pending;
    serial_t *uart[HUB6_PORTS];
} hub6_serial_t;

static void
hub6_uart_irq(void *priv, int active)
{
    uintptr_t tagged = (uintptr_t) priv;
    hub6_serial_t *card = (hub6_serial_t *) (tagged & ~(uintptr_t) 0x0f);
    unsigned index = (unsigned) (tagged & 0x0f);
    if (active)
        card->irq_pending |= (uint16_t) (1u << index);
    else
        card->irq_pending &= (uint16_t) ~(1u << index);

    if (card->irq_pending)
        picint(1u << HUB6_IRQ);
    else
        picintc(1u << HUB6_IRQ);
}

static int
hub6_selected_port(uint8_t select)
{
    unsigned card = select >> 6;
    unsigned port = (select & 0x3f) >> 3;
    if ((card > 1) || (port > 5))
        return -1;
    return (int) (card * 6 + port);
}

static uint8_t
hub6_read(uint16_t port, void *priv)
{
    hub6_serial_t *card = (hub6_serial_t *) priv;
    if (port == HUB6_INDEX)
        return 0xff;

    int selected = hub6_selected_port(card->selected);
    if (selected < 0)
        return 0xff;
    return serial_read(card->selected & 7, card->uart[selected]);
}

static void
hub6_write(uint16_t port, uint8_t value, void *priv)
{
    hub6_serial_t *card = (hub6_serial_t *) priv;
    if (port == HUB6_INDEX) {
        card->selected = value;
        return;
    }

    int selected = hub6_selected_port(card->selected);
    if (selected >= 0)
        serial_write(card->selected & 7, value, card->uart[selected]);
}

static void *
hub6_serial_init(const device_t *info)
{
    hub6_serial_t *card = (hub6_serial_t *) calloc(1, sizeof(*card));
    (void) info;
    if (card == NULL)
        return NULL;

    for (unsigned i = 0; i < HUB6_PORTS; i++) {
        card->uart[i] = serial_create_board_uart(SERIAL_16550, 0x08, i, hub6_uart_irq,
                                                  (void *) ((uintptr_t) card | i));
        if (card->uart[i] == NULL) {
            while (i > 0)
                serial_destroy_board_uart(card->uart[--i]);
            free(card);
            return NULL;
        }
        /* These UARTs share the HUB6 selector/data window instead of owning
           individual I/O handlers. */
        serial_setup(card->uart[i], 0, HUB6_IRQ);
    }

    io_sethandler(HUB6_INDEX, 2, hub6_read, NULL, NULL, hub6_write, NULL, NULL, card);
    return card;
}

static void
hub6_serial_close(void *priv)
{
    hub6_serial_t *card = (hub6_serial_t *) priv;
    if (card == NULL)
        return;
    io_removehandler(HUB6_INDEX, 2, hub6_read, NULL, NULL, hub6_write, NULL, NULL, card);
    picintc(1u << HUB6_IRQ);
    for (unsigned i = 0; i < HUB6_PORTS; i++)
        serial_destroy_board_uart(card->uart[i]);
    free(card);
}

static void
hub6_serial_reset(void *priv)
{
    hub6_serial_t *card = (hub6_serial_t *) priv;
    card->selected = 0;
    card->irq_pending = 0;
    for (unsigned i = 0; i < HUB6_PORTS; i++)
        serial_reset_board_uart(card->uart[i]);
    picintc(1u << HUB6_IRQ);
}

const device_t hub6_serial_device = {
    .name = "HUB6 ISA serial adapter",
    .internal_name = "hub6_serial",
    .flags = DEVICE_ISA16,
    .local = 0,
    .init = hub6_serial_init,
    .close = hub6_serial_close,
    .reset = hub6_serial_reset,
    .available = NULL,
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = NULL
};
