#include <stdint.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/pic.h>
#include <86box/timer.h>
#include <86box/serial.h>

#define AST_PORTS 4
#define AST_IRQ_PORT(base) (((base) & 0x0fe0) | 0x001f)

typedef struct ast_fourport_s {
    uint16_t base;
    uint16_t irq_port;
    uint8_t irq;
    uint8_t irq_pending;
    uint8_t irq_enabled;
    serial_t *uart[AST_PORTS];
} ast_fourport_t;

static void
ast_update_irq(ast_fourport_t *card)
{
    if (card->irq_enabled && card->irq_pending)
        picint(1u << card->irq);
    else
        picintc(1u << card->irq);
}

static void
ast_uart_irq(void *priv, int active)
{
    uintptr_t port = (uintptr_t) priv;
    ast_fourport_t *card = (ast_fourport_t *) (port & ~(uintptr_t) 3);
    unsigned index = (unsigned) (port & 3);
    if (active)
        card->irq_pending |= 1u << index;
    else
        card->irq_pending &= (uint8_t) ~(1u << index);
    ast_update_irq(card);
}

static uint8_t
ast_read(uint16_t port, void *priv)
{
    ast_fourport_t *card = (ast_fourport_t *) priv;
    if (port == card->irq_port) {
        ast_update_irq(card);
        return (uint8_t) ((card->irq_enabled ? 0x80 : 0) | (card->irq_pending ? 1 : 0));
    }
    return 0xff;
}

static void
ast_write(uint16_t port, uint8_t value, void *priv)
{
    ast_fourport_t *card = (ast_fourport_t *) priv;
    if (port == card->irq_port) {
        card->irq_enabled = !!(value & 0x80);
        ast_update_irq(card);
    }
}

static void *
ast_fourport_init(const device_t *info)
{
    ast_fourport_t *card = (ast_fourport_t *) calloc(1, sizeof(*card));
    (void) info;
    if (card == NULL)
        return NULL;

    card->base = (uint16_t) device_get_config_hex16("base");
    card->irq = (card->base == 0x2a0) ? 5 : 9;
    card->irq_port = AST_IRQ_PORT(card->base);
    card->irq_enabled = 1;
    for (unsigned i = 0; i < AST_PORTS; i++) {
        card->uart[i] = serial_create_board_uart(SERIAL_8250, 0x04, i, ast_uart_irq,
                                                  (void *) ((uintptr_t) card | i));
        if (card->uart[i] == NULL) {
            while (i > 0)
                serial_destroy_board_uart(card->uart[--i]);
            free(card);
            return NULL;
        }
        serial_setup(card->uart[i], card->base + i * 8, card->irq);
    }
    io_sethandler(card->irq_port, 1, ast_read, NULL, NULL, ast_write, NULL, NULL, card);
    return card;
}

static void
ast_fourport_close(void *priv)
{
    ast_fourport_t *card = (ast_fourport_t *) priv;
    if (card == NULL)
        return;
    io_removehandler(card->irq_port, 1, ast_read, NULL, NULL, ast_write, NULL, NULL, card);
    picintc(1u << card->irq);
    for (unsigned i = 0; i < AST_PORTS; i++)
        serial_destroy_board_uart(card->uart[i]);
    free(card);
}

static void
ast_fourport_reset(void *priv)
{
    ast_fourport_t *card = (ast_fourport_t *) priv;
    card->irq_pending = 0;
    card->irq_enabled = 1;
    for (unsigned i = 0; i < AST_PORTS; i++)
        serial_reset_board_uart(card->uart[i]);
    ast_update_irq(card);
}

static const device_config_t ast_fourport_config[] = {
    {
        .name = "base", .description = "I/O base address", .type = CONFIG_HEX16,
        .default_int = 0x1a0,
        .selection = {
            { .description = "0x1A0 (IRQ 9)", .value = 0x1a0 },
            { .description = "0x2A0 (IRQ 5)", .value = 0x2a0 },
            { .description = "" }
        }
    },
    { .name = "", .type = CONFIG_END }
};

const device_t ast_fourport_device = {
    .name = "AST FourPort ISA serial adapter",
    .internal_name = "ast_fourport",
    .flags = DEVICE_ISA16,
    .local = 0,
    .init = ast_fourport_init,
    .close = ast_fourport_close,
    .reset = ast_fourport_reset,
    .available = NULL,
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = ast_fourport_config
};
