#include <stdint.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/pic.h>
#include <86box/timer.h>
#include <86box/serial.h>

#define BOCA_PORTS 16
#define BOCA_IRQ 12
#define BOCA_BASE 0x100

typedef struct boca_serial_s {
    uint16_t irq_pending;
    serial_t *uart[BOCA_PORTS];
} boca_serial_t;

static void
boca_uart_irq(void *priv, int active)
{
    uintptr_t tagged = (uintptr_t) priv;
    boca_serial_t *card = (boca_serial_t *) (tagged & ~(uintptr_t) (BOCA_PORTS - 1));
    unsigned index = (unsigned) (tagged & (BOCA_PORTS - 1));
    if (active)
        card->irq_pending |= (uint16_t) (1u << index);
    else
        card->irq_pending &= (uint16_t) ~(1u << index);

    if (card->irq_pending)
        picint(1u << BOCA_IRQ);
    else
        picintc(1u << BOCA_IRQ);
}

static void *
boca_serial_init(const device_t *info)
{
    boca_serial_t *card = (boca_serial_t *) calloc(1, sizeof(*card));
    (void) info;
    if (card == NULL)
        return NULL;

    for (unsigned i = 0; i < BOCA_PORTS; i++) {
        card->uart[i] = serial_create_board_uart(SERIAL_16550, 0x08, i, boca_uart_irq,
                                                  (void *) ((uintptr_t) card | i));
        if (card->uart[i] == NULL) {
            while (i > 0)
                serial_destroy_board_uart(card->uart[--i]);
            free(card);
            return NULL;
        }
        serial_setup(card->uart[i], BOCA_BASE + i * 8, BOCA_IRQ);
    }
    return card;
}

static void
boca_serial_close(void *priv)
{
    boca_serial_t *card = (boca_serial_t *) priv;
    if (card == NULL)
        return;
    picintc(1u << BOCA_IRQ);
    for (unsigned i = 0; i < BOCA_PORTS; i++)
        serial_destroy_board_uart(card->uart[i]);
    free(card);
}

static void
boca_serial_reset(void *priv)
{
    boca_serial_t *card = (boca_serial_t *) priv;
    card->irq_pending = 0;
    for (unsigned i = 0; i < BOCA_PORTS; i++)
        serial_reset_board_uart(card->uart[i]);
    picintc(1u << BOCA_IRQ);
}

const device_t boca_serial_device = {
    .name = "Boca BB2016 16-port ISA serial adapter",
    .internal_name = "boca_bb2016",
    .flags = DEVICE_ISA16,
    .local = 0,
    .init = boca_serial_init,
    .close = boca_serial_close,
    .reset = boca_serial_reset,
    .available = NULL,
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = NULL
};
