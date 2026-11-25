#include <stdint.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/pic.h>
#include <86box/timer.h>
#include <86box/serial.h>

#define EXAR_PORTS 4
#define EXAR_IRQ 5
#define EXAR_BASE 0x100

typedef struct exar_st16c554_s {
    uint8_t irq_pending;
    serial_t *uart[EXAR_PORTS];
} exar_st16c554_t;

static void
exar_uart_irq(void *priv, int active)
{
    uintptr_t tagged = (uintptr_t) priv;
    exar_st16c554_t *card = (exar_st16c554_t *) (tagged & ~(uintptr_t) 3);
    unsigned index = (unsigned) (tagged & 3);
    if (active)
        card->irq_pending |= 1u << index;
    else
        card->irq_pending &= (uint8_t) ~(1u << index);

    if (card->irq_pending)
        picint(1u << EXAR_IRQ);
    else
        picintc(1u << EXAR_IRQ);
}

static void *
exar_st16c554_init(const device_t *info)
{
    exar_st16c554_t *card = (exar_st16c554_t *) calloc(1, sizeof(*card));
    (void) info;
    if (card == NULL)
        return NULL;

    for (unsigned i = 0; i < EXAR_PORTS; i++) {
        card->uart[i] = serial_create_board_uart(SERIAL_16550, 0x08, i, exar_uart_irq,
                                                  (void *) ((uintptr_t) card | i));
        if (card->uart[i] == NULL) {
            while (i > 0)
                serial_destroy_board_uart(card->uart[--i]);
            free(card);
            return NULL;
        }
        serial_setup(card->uart[i], EXAR_BASE + i * 8, EXAR_IRQ);
    }
    return card;
}

static void
exar_st16c554_close(void *priv)
{
    exar_st16c554_t *card = (exar_st16c554_t *) priv;
    if (card == NULL)
        return;
    picintc(1u << EXAR_IRQ);
    for (unsigned i = 0; i < EXAR_PORTS; i++)
        serial_destroy_board_uart(card->uart[i]);
    free(card);
}

static void
exar_st16c554_reset(void *priv)
{
    exar_st16c554_t *card = (exar_st16c554_t *) priv;
    card->irq_pending = 0;
    for (unsigned i = 0; i < EXAR_PORTS; i++)
        serial_reset_board_uart(card->uart[i]);
    picintc(1u << EXAR_IRQ);
}

const device_t exar_st16c554_device = {
    .name = "Exar ST16C554/554D ISA quad UART adapter",
    .internal_name = "exar_st16c554",
    .flags = DEVICE_ISA16,
    .local = 0,
    .init = exar_st16c554_init,
    .close = exar_st16c554_close,
    .reset = exar_st16c554_reset,
    .available = NULL,
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = NULL
};
