#include <stdint.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/pic.h>
#include <86box/timer.h>
#include <86box/serial.h>

#define ACCENT_PORTS 2

typedef struct accent_async_s {
    uint8_t irq_pending;
    serial_t *uart[ACCENT_PORTS];
} accent_async_t;

static void
accent_uart_irq(void *priv, int active)
{
    uintptr_t tagged = (uintptr_t) priv;
    accent_async_t *card = (accent_async_t *) (tagged & ~(uintptr_t) 1);
    unsigned index = (unsigned) (tagged & 1);
    if (active)
        card->irq_pending |= 1u << index;
    else
        card->irq_pending &= (uint8_t) ~(1u << index);

    if (card->irq_pending)
        picint(1u << 4);
    else
        picintc(1u << 4);
}

static void *
accent_async_init(const device_t *info)
{
    static const uint16_t bases[ACCENT_PORTS] = { 0x330, 0x338 };
    accent_async_t *card = (accent_async_t *) calloc(1, sizeof(*card));
    (void) info;
    if (card == NULL)
        return NULL;

    for (unsigned i = 0; i < ACCENT_PORTS; i++) {
        card->uart[i] = serial_create_board_uart(SERIAL_8250, 0x08, accent_uart_irq,
                                                  (void *) ((uintptr_t) card | i));
        if (card->uart[i] == NULL) {
            while (i > 0)
                serial_destroy_board_uart(card->uart[--i]);
            free(card);
            return NULL;
        }
        serial_setup(card->uart[i], bases[i], 4);
    }
    return card;
}

static void
accent_async_close(void *priv)
{
    accent_async_t *card = (accent_async_t *) priv;
    if (card == NULL)
        return;
    picintc(1u << 4);
    for (unsigned i = 0; i < ACCENT_PORTS; i++)
        serial_destroy_board_uart(card->uart[i]);
    free(card);
}

static void
accent_async_reset(void *priv)
{
    accent_async_t *card = (accent_async_t *) priv;
    card->irq_pending = 0;
    for (unsigned i = 0; i < ACCENT_PORTS; i++)
        serial_reset_board_uart(card->uart[i]);
    picintc(1u << 4);
}

const device_t accent_async_device = {
    .name = "Accent Async ISA serial adapter",
    .internal_name = "accent_async",
    .flags = DEVICE_ISA16,
    .local = 0,
    .init = accent_async_init,
    .close = accent_async_close,
    .reset = accent_async_reset,
    .available = NULL,
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = NULL
};
