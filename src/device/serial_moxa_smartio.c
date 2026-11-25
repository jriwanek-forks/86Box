#include <stdint.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/pic.h>
#include <86box/timer.h>
#include <86box/serial.h>

#define MOXA_MAX_PORTS 8
#define MOXA_CFG_CS 0x04
#define MOXA_CFG_SK 0x01
#define MOXA_CFG_DO 0x02
#define MOXA_CFG_DI 0x08

typedef struct moxa_smartio_s moxa_smartio_t;
typedef struct moxa_irq_context_s {
    moxa_smartio_t *board;
    uint8_t channel;
} moxa_irq_context_t;

struct moxa_smartio_s {
    uint16_t base;
    uint16_t vector;
    uint8_t irq;
    uint8_t count;
    uint8_t asic_id;
    uint8_t irq_state[MOXA_MAX_PORTS];
    uint8_t irq_line_state;
    uint8_t program_stage;
    uint8_t program_mode;
    uint8_t config_sequence;
    uint8_t eeprom_cs;
    uint8_t eeprom_cmd_bits;
    uint16_t eeprom_command;
    uint16_t eeprom[14];
    uint8_t eeprom_cursor;
    serial_t *uart[MOXA_MAX_PORTS];
    moxa_irq_context_t irq_context[MOXA_MAX_PORTS];
};

static uint8_t
moxa_vector(moxa_smartio_t *dev)
{
    uint8_t value = (dev->count == 8) ? 0xff : 0x0f;
    for (int i = 0; i < dev->count; i++)
        if (dev->irq_state[i])
            value &= (uint8_t) ~(1u << i);
    return value;
}

static uint8_t
moxa_vector_read(uint16_t port, void *priv)
{
    (void) port;
    return moxa_vector((moxa_smartio_t *) priv);
}

static void
moxa_uart_irq(void *priv, int active)
{
    moxa_irq_context_t *context = (moxa_irq_context_t *) priv;
    moxa_smartio_t *dev = context->board;
    dev->irq_state[context->channel] = (uint8_t) active;
    uint8_t vector = moxa_vector(dev);
    int asserted = (dev->count == 8) ? (vector != 0xff) : (vector != 0x0f);
    dev->irq_line_state = (uint8_t) asserted;
    picintlevel(1u << dev->irq, &dev->irq_line_state);
}

static uint16_t
moxa_eeprom_value(moxa_smartio_t *dev, unsigned address)
{
    return (address < 14) ? dev->eeprom[address] : 0xffff;
}

static uint8_t
moxa_read(uint16_t port, void *priv)
{
    moxa_smartio_t *dev = (moxa_smartio_t *) priv;
    unsigned offset = port - dev->base;
    if (dev->program_mode && (offset == 0)) {
        if (dev->eeprom_cs && (dev->eeprom_cmd_bits >= 9)) {
            uint16_t word = moxa_eeprom_value(dev, dev->eeprom_command & 0x3f);
            uint8_t result = (dev->eeprom_cursor < 16 && (word & (0x8000u >> dev->eeprom_cursor))) ? MOXA_CFG_DI : 0;
            if (dev->eeprom_cursor < 16)
                dev->eeprom_cursor++;
            return result;
        }
        return 0;
    }
    if (dev->program_mode && (offset == 1) && (dev->program_stage == 7)) {
        dev->program_stage = 8;
        return dev->asic_id;
    }
    if (dev->program_mode && (offset == 2) && (dev->program_stage >= 8)) {
        return (dev->config_sequence++ == 0) ? 'M' : 1;
    }
    return serial_read(port, dev->uart[offset >> 3]);
}

static void
moxa_write(uint16_t port, uint8_t value, void *priv)
{
    moxa_smartio_t *dev = (moxa_smartio_t *) priv;
    unsigned offset = port - dev->base;
    if ((offset == 1) && dev->program_mode && (value == 0xa5)) {
        dev->program_mode = 0;
        dev->program_stage = 0;
        dev->eeprom_cs = 0;
        serial_write(port, value, dev->uart[0]);
        return;
    }
    if ((offset == 0) && dev->program_mode) {
        if ((value & MOXA_CFG_CS) && !dev->eeprom_cs) {
            dev->eeprom_cmd_bits = 0;
            dev->eeprom_command = 0;
            dev->eeprom_cursor = 0;
        }
        if ((value & MOXA_CFG_CS) && (value & MOXA_CFG_SK) && !dev->eeprom_cs) {
            /* The first high clock after CS is asserted begins command capture. */
            dev->eeprom_command = (uint16_t) ((dev->eeprom_command << 1) | !!(value & MOXA_CFG_DO));
            dev->eeprom_cmd_bits++;
        } else if ((value & MOXA_CFG_CS) && (value & MOXA_CFG_SK) && dev->eeprom_cs &&
                   (dev->eeprom_cmd_bits < 9)) {
            dev->eeprom_command = (uint16_t) ((dev->eeprom_command << 1) | !!(value & MOXA_CFG_DO));
            dev->eeprom_cmd_bits++;
        }
        if (!(value & MOXA_CFG_CS)) {
            dev->eeprom_cs = 0;
            dev->eeprom_cmd_bits = 0;
            dev->eeprom_cursor = 0;
        } else if (!(dev->eeprom_cmd_bits < 9)) {
            dev->eeprom_cs = 1;
        } else {
            dev->eeprom_cs = 1;
        }
        return;
    }
    if ((offset == 0) && !dev->program_mode && (value == 0) && (dev->program_stage < 3)) {
        if (++dev->program_stage == 3)
            return;
        return;
    }
    serial_write(port, value, dev->uart[offset >> 3]);
}

static uint8_t
moxa_probe_read(uint16_t port, void *priv)
{
    moxa_smartio_t *dev = (moxa_smartio_t *) priv;
    unsigned offset = port - dev->base;
    if ((offset == 0) && (dev->program_stage == 3)) {
        dev->program_stage = 4;
        return 0;
    }
    if ((offset == 0) && (dev->program_stage == 4)) {
        dev->program_stage = 5;
        return 0;
    }
    if ((offset == 0) && (dev->program_stage == 6)) {
        dev->program_stage = 7;
        return 0;
    }
    return moxa_read(port, priv);
}

static void
moxa_probe_write(uint16_t port, uint8_t value, void *priv)
{
    moxa_smartio_t *dev = (moxa_smartio_t *) priv;
    unsigned offset = port - dev->base;
    if ((offset == 0) && (value == 0) && (dev->program_stage == 5)) {
        dev->program_stage = 6;
        return;
    }
    moxa_write(port, value, priv);
}

static void
moxa_init_eeprom(moxa_smartio_t *dev)
{
    for (int i = 0; i < 14; i++)
        dev->eeprom[i] = 0;
    for (int i = 0; i < dev->count; i++)
        dev->eeprom[i + 1] = (uint16_t) (dev->base + i * 8);
    const uint16_t irq = (uint16_t) (dev->irq & 0x0f);
    const uint16_t irq_config = (uint16_t) ((irq << 12) | (irq << 8) | (irq << 4) | irq);
    dev->eeprom[9] = irq_config;
    dev->eeprom[10] = irq_config;
    dev->eeprom[11] = (uint16_t) (dev->base + 0x40);
    dev->eeprom[12] = 0xff80;
}

static void *
moxa_init(const device_t *info)
{
    moxa_smartio_t *dev = (moxa_smartio_t *) calloc(1, sizeof(*dev));
    if (!dev)
        return NULL;
    const int model = info->local;
    dev->base = (uint16_t) device_get_config_hex16("base");
    dev->irq = (uint8_t) device_get_config_int("irq");
    if (dev->irq > 15 || dev->irq == 0 || dev->irq == 1 || dev->irq == 2 || dev->irq == 6 || dev->irq == 8 || dev->irq == 13 || dev->irq == 14)
        dev->irq = 5;
    switch (model) {
        case 0: dev->count = 8; dev->asic_id = 1; break; /* C168 */
        case 1: dev->count = 4; dev->asic_id = 2; break; /* C104 */
        case 2: dev->count = 4; dev->asic_id = 5; break; /* CI-104J */
        case 3: dev->count = 4; dev->asic_id = 0x0b; break; /* C102 */
        case 4: dev->count = 4; dev->asic_id = 4; break; /* CI-132 */
        default: dev->count = 4; dev->asic_id = 3; break; /* CI-134 */
    }
    moxa_init_eeprom(dev);
    dev->vector = (uint16_t) (dev->base + 0x40);
    for (int i = 0; i < dev->count; i++) {
        dev->uart[i] = serial_create_board_uart(SERIAL_16550, 0x08, (uint8_t) i, moxa_uart_irq, dev);
        if (!dev->uart[i]) {
            for (int j = 0; j < i; j++)
                serial_destroy_board_uart(dev->uart[j]);
            free(dev);
            return NULL;
        }
        dev->irq_context[i].board = dev;
        dev->irq_context[i].channel = (uint8_t) i;
        dev->uart[i]->board_irq_priv = &dev->irq_context[i];
        if (i > 0)
            io_sethandler((uint16_t) (dev->base + i * 8), 8,
                          serial_read, NULL, NULL, serial_write, NULL, NULL, dev->uart[i]);
    }
    io_sethandler(dev->base, 8, moxa_probe_read, NULL, NULL,
                  moxa_probe_write, NULL, NULL, dev);
    io_sethandler(dev->vector, 1, moxa_vector_read, NULL, NULL, NULL, NULL, NULL, dev);
    (void) info;
    return dev;
}

static void
moxa_close(void *priv)
{
    moxa_smartio_t *dev = (moxa_smartio_t *) priv;
    if (!dev)
        return;
    io_removehandler(dev->base, 8, moxa_probe_read, NULL, NULL, moxa_probe_write, NULL, NULL, dev);
    io_removehandler(dev->vector, 1, moxa_vector_read, NULL, NULL, NULL, NULL, NULL, dev);
    for (int i = 0; i < dev->count; i++) {
        io_removehandler((uint16_t) (dev->base + i * 8), 8,
                         (i == 0) ? moxa_probe_read : serial_read,
                         NULL, NULL, (i == 0) ? moxa_probe_write : serial_write,
                         NULL, NULL, dev->uart[i]);
        serial_destroy_board_uart(dev->uart[i]);
    }
    free(dev);
}

static void
moxa_reset(void *priv)
{
    moxa_smartio_t *dev = (moxa_smartio_t *) priv;
    dev->program_stage = dev->program_mode = dev->eeprom_cs = 0;
    dev->config_sequence = 0;
    for (int i = 0; i < dev->count; i++) {
        dev->irq_state[i] = 0;
        serial_reset_board_uart(dev->uart[i]);
    }
    picintc(1u << dev->irq);
}

static const device_config_t moxa_config[] = {
    { .name = "base", .description = "UART I/O base", .type = CONFIG_HEX16, .default_int = 0x180,
      .selection = { { .description = "0x180", .value = 0x180 }, { .description = "0x200", .value = 0x200 },
                     { .description = "0x280", .value = 0x280 }, { .description = "0x300", .value = 0x300 },
                     { .description = "0x320", .value = 0x320 }, { .description = "" } } },
    { .name = "irq", .description = "IRQ", .type = CONFIG_SELECTION, .default_int = 5,
      .selection = { { .description = "IRQ 3", .value = 3 }, { .description = "IRQ 4", .value = 4 },
                     { .description = "IRQ 5", .value = 5 }, { .description = "IRQ 7", .value = 7 },
                     { .description = "IRQ 10", .value = 10 }, { .description = "IRQ 11", .value = 11 },
                     { .description = "IRQ 12", .value = 12 }, { .description = "IRQ 15", .value = 15 },
                     { .description = "" } } },
    { .name = "", .type = CONFIG_END }
};

#define MOXA_DEVICE(symbol, display, internal, model) \
const device_t symbol = { \
    .name = display, .internal_name = internal, .flags = DEVICE_ISA16, .local = model, \
    .init = moxa_init, .close = moxa_close, .reset = moxa_reset, .available = NULL, \
    .speed_changed = NULL, .force_redraw = NULL, .config = moxa_config \
}

MOXA_DEVICE(moxa_c168_isa_device, "Moxa C168 ISA SmartIO (8-port)", "moxa_c168_isa", 0);
MOXA_DEVICE(moxa_c104_isa_device, "Moxa C104 ISA SmartIO (4-port)", "moxa_c104_isa", 1);
MOXA_DEVICE(moxa_ci104j_isa_device, "Moxa CI-104J ISA SmartIO (4-port)", "moxa_ci104j_isa", 2);
MOXA_DEVICE(moxa_c102_isa_device, "Moxa C102 ISA SmartIO (4-port)", "moxa_c102_isa", 3);
MOXA_DEVICE(moxa_ci132_isa_device, "Moxa CI-132 ISA SmartIO (4-port)", "moxa_ci132_isa", 4);
MOXA_DEVICE(moxa_ci134_isa_device, "Moxa CI-134 ISA SmartIO (4-port)", "moxa_ci134_isa", 5);

#undef MOXA_DEVICE
