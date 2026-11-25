/*
 * Cirrus Logic CL-CD1400 uXART serial controller.
 *
 * This is the four-channel serial subset used by intelligent multiport
 * adapters. Board I/O decoding and IRQ aggregation belong to the board.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/char.h>
#include <86box/serial.h>

#define CD1400_CHANNELS 4
#define CD1400_FIFO     12

#define CCR_RESETCHAN   0x80
#define CCR_RESETFULL   0x81
#define CCR_TXFLUSHFIFO 0x82
#define CCR_CORCHANGE   0x4e
#define CCR_TXENABLE    0x18
#define CCR_TXDISABLE   0x14
#define CCR_RXENABLE    0x12
#define CCR_RXDISABLE   0x11

#define SRER_NNDT       0x01
#define SRER_TXEMPTY    0x02
#define SRER_TXDATA     0x04
#define SRER_RXDATA     0x10
#define SRER_MODEM      0x80
#define SVRR_RX         0x01
#define SVRR_TX         0x02
#define SVRR_MODEM      0x04
#define ACK_RX          0x03
#define ACK_TX          0x02
#define ACK_MODEM       0x01
#define RXSTAT_OVERRUN  0x01
#define ETC_CMD         0x00
#define ETC_STARTBREAK  0x81
#define ETC_STOPBREAK   0x83

#define CD1400_CLOCK    25000000.0
static const unsigned cd1400_prescalers[5] = { 8, 32, 128, 512, 2048 };

#define REG_CCR    0x05
#define REG_SRER   0x06
#define REG_COR1   0x08
#define REG_COR2   0x09
#define REG_COR3   0x0a
#define REG_CCSR   0x0b
#define REG_RDCR   0x0e
#define REG_LIVR   0x18
#define REG_RBPR   0x78
#define REG_RCOR   0x7c
#define REG_TBPR   0x72
#define REG_TCOR   0x76
#define REG_CAR    0x68
#define REG_GFRCR  0x40
#define REG_GCR    0x4b
#define REG_SVRR   0x67
#define REG_RICR   0x44
#define REG_TICR   0x45
#define REG_MICR   0x46
#define REG_RIR    0x6b
#define REG_TIR    0x6a
#define REG_MIR    0x69
#define REG_PPR    0x7e
#define REG_TDR    0x63
#define REG_RDSR   0x62
#define REG_MISR   0x4c
#define REG_EOSRR  0x60
#define REG_MIVR   0x41
#define REG_TIVR   0x42
#define REG_RIVR   0x43

typedef struct cd1400_channel_s {
    uint8_t reg[128];
    uint8_t rx[CD1400_FIFO];
    uint8_t rx_status[CD1400_FIFO];
    unsigned rx_head, rx_count;
    uint8_t rx_status_pending;
    uint8_t tx[CD1400_FIFO];
    unsigned tx_head, tx_count;
    uint8_t tx_busy;
    uint8_t etc_pending;
    uint8_t break_active;
    uint8_t modem_in;
    uint8_t modem_out;
    uint8_t modem_delta;
    uint8_t rx_enabled, tx_enabled;
    uint8_t rx_overrun;
    uint8_t rx_timeout;
    double rx_idle_us;
    pc_timer_t tx_timer;
    pc_timer_t rx_timer;
    serial_endpoint_t endpoint;
    struct cd1400_s *owner;
    unsigned number;
} cd1400_channel_t;

typedef struct cd1400_s {
    uint8_t global[128];
    uint8_t car;
    uint8_t service;
    uint8_t current_rx, current_tx, current_modem;
    uint8_t ack_type;
    uint8_t ack_channel;
    cd1400_channel_t channel[CD1400_CHANNELS];
} cd1400_t;

static void cd1400_reset(void *priv);
static void cd1400_tx_tick(void *priv);
static void cd1400_tx_start(cd1400_channel_t *ch);
static void cd1400_rx_tick(void *priv);

static int
cd1400_is_global(uint8_t reg)
{
    switch (reg) {
        case REG_GFRCR: case REG_CAR: case REG_GCR: case REG_SVRR:
        case REG_RICR: case REG_TICR: case REG_MICR: case REG_RIR:
        case REG_TIR: case REG_MIR: case REG_PPR: case REG_MIVR:
        case REG_TIVR: case REG_RIVR: case REG_TDR: case REG_RDSR:
        case REG_MISR: case REG_EOSRR:
            return 1;
        default:
            return 0;
    }
}

static double
cd1400_baud(const cd1400_channel_t *ch, int receive)
{
    unsigned cor = ch->reg[receive ? REG_RCOR : REG_TCOR] & 7;
    unsigned bpr = ch->reg[receive ? REG_RBPR : REG_TBPR];
    unsigned divider = (cor < 5) ? cd1400_prescalers[cor] : cd1400_prescalers[0];
    return CD1400_CLOCK / ((double) divider * (bpr + 1));
}

static double
cd1400_frame_us(const cd1400_channel_t *ch)
{
    uint8_t cor1 = ch->reg[REG_COR1];
    unsigned bits = 5 + (cor1 & 3);
    double frame = 1.0 + bits;
    unsigned stop = (cor1 >> 2) & 3;
    if (cor1 & 0x60)
        frame += 1.0;
    frame += (stop == 1) ? 1.5 : (stop >= 2) ? 2.0 : 1.0;
    return frame * 1000000.0 / cd1400_baud(ch, 0);
}

static void
cd1400_update_format(cd1400_channel_t *ch)
{
    uint8_t cor1 = ch->reg[REG_COR1];
    unsigned parity = 0;
    if (cor1 & 0x40)
        parity = (cor1 & 0x80) ? CHAR_COM_PARITY_ODD : CHAR_COM_PARITY_EVEN;
    if (ch->endpoint.be && ch->endpoint.be->set_format)
        ch->endpoint.be->set_format(ch->endpoint.be->priv,
            (int) (cd1400_baud(ch, 0) + 0.5), 5 + (cor1 & 3), parity,
            ((cor1 >> 2) & 3) ? 2 : 1);
}

static void
cd1400_update_modem_outputs(cd1400_channel_t *ch)
{
    uint32_t control = 0;
    if (ch->modem_out & 0x01) control |= CHAR_COM_DTR;
    if (ch->modem_out & 0x02) control |= CHAR_COM_RTS;
    if (ch->break_active) control |= CHAR_COM_BREAK;
    if (ch->endpoint.be && ch->endpoint.be->control)
        ch->endpoint.be->control(ch->endpoint.be->priv, control);
}

static void
cd1400_backend_changed(void *priv)
{
    cd1400_channel_t *ch = (cd1400_channel_t *) priv;
    cd1400_update_format(ch);
    cd1400_update_modem_outputs(ch);
}

static void
cd1400_update_requests(cd1400_t *dev)
{
    dev->service = 0;
    dev->current_rx = dev->current_tx = dev->current_modem = 0xff;
    for (unsigned i = 0; i < CD1400_CHANNELS; i++) {
        cd1400_channel_t *ch = &dev->channel[i];
        uint8_t enables = ch->reg[REG_SRER];
        unsigned threshold = ch->reg[REG_COR3] & 0x1f;
        if (threshold == 0) threshold = 1;
        if ((dev->current_rx == 0xff) && ch->rx_enabled &&
            ((ch->rx_count >= threshold) || ch->rx_timeout) && (enables & SRER_RXDATA))
            dev->current_rx = (uint8_t) i;
        if ((dev->current_tx == 0xff) && ch->tx_enabled &&
            (((enables & SRER_TXDATA) && (ch->tx_count <= 6)) ||
             ((enables & SRER_TXEMPTY) && (ch->tx_count == 0) && !ch->tx_busy)))
            dev->current_tx = (uint8_t) i;
        if ((dev->current_modem == 0xff) && ch->modem_delta && (enables & SRER_MODEM))
            dev->current_modem = (uint8_t) i;
    }
    if (dev->current_rx != 0xff) dev->service |= SVRR_RX;
    if (dev->current_tx != 0xff) dev->service |= SVRR_TX;
    if (dev->current_modem != 0xff) dev->service |= SVRR_MODEM;
}

static int
cd1400_can_receive(void *priv)
{
    cd1400_channel_t *ch = (cd1400_channel_t *) priv;
    return ch->rx_enabled && (ch->rx_count < CD1400_FIFO);
}

static void
cd1400_receive(void *priv, uint8_t byte)
{
    cd1400_channel_t *ch = (cd1400_channel_t *) priv;
    unsigned tail;
    if (!ch->rx_enabled)
        return;
    if (ch->rx_count == CD1400_FIFO) {
        ch->rx_overrun = 1;
        if (ch->rx_count)
            ch->rx_status[(ch->rx_head + ch->rx_count - 1) % CD1400_FIFO] |= RXSTAT_OVERRUN;
        return;
    }
    tail = (ch->rx_head + ch->rx_count) % CD1400_FIFO;
    ch->rx[tail] = byte;
    ch->rx_count++;
    ch->rx_timeout = 0;
    ch->rx_idle_us = 0.0;
    cd1400_update_requests(ch->owner);
}

static void
cd1400_set_line(void *priv, uint32_t line, int on)
{
    cd1400_channel_t *ch = (cd1400_channel_t *) priv;
    uint8_t bit = (line == CHAR_COM_CTS) ? 0x40 : (line == CHAR_COM_DSR) ? 0x10 :
                  (line == CHAR_COM_RI) ? 0x20 : (line == CHAR_COM_DCD) ? 0x80 : 0;
    uint8_t old = ch->modem_in;
    if (!bit) return;
    if (on) ch->modem_in |= bit;
    else ch->modem_in &= (uint8_t) ~bit;
    if (old != ch->modem_in) {
        ch->modem_delta |= (uint8_t) ((old ^ ch->modem_in) & (ch->reg[0x15] | ch->reg[0x16]));
        cd1400_update_requests(ch->owner);
    }
}

static int
cd1400_get_line(void *priv, uint32_t line)
{
    cd1400_channel_t *ch = (cd1400_channel_t *) priv;
    if (line == CHAR_COM_DTR) return !!(ch->modem_out & 1);
    if (line == CHAR_COM_RTS) return !!(ch->modem_out & 2);
    if (line == CHAR_COM_CTS) return !!(ch->modem_in & 0x40);
    if (line == CHAR_COM_DSR) return !!(ch->modem_in & 0x10);
    if (line == CHAR_COM_RI) return !!(ch->modem_in & 0x20);
    if (line == CHAR_COM_DCD) return !!(ch->modem_in & 0x80);
    return 0;
}

static const serial_port_ops_t cd1400_ops = {
    .can_receive = cd1400_can_receive,
    .receive = cd1400_receive,
    .set_line = cd1400_set_line,
    .get_line = cd1400_get_line
};

serial_endpoint_t *
cd1400_endpoint(void *priv, int channel)
{
    cd1400_t *dev = (cd1400_t *) priv;
    if ((dev == NULL) || (channel < 0) || (channel >= CD1400_CHANNELS))
        return NULL;
    return &dev->channel[channel].endpoint;
}

static uint8_t
cd1400_read_channel(cd1400_t *dev, cd1400_channel_t *ch, uint8_t reg)
{
    switch (reg) {
        /* Commands complete synchronously in this model. The hardware clears
           CCR after its internal sequencer has performed the command. */
        case REG_CCR: return 0;
        case REG_CCSR: {
            uint8_t status = 0;
            if (ch->rx_enabled) status |= 0x80;
            if (ch->rx_enabled && (ch->rx_count < CD1400_FIFO / 2)) status |= 0x40;
            if (ch->rx_enabled && (ch->rx_count >= CD1400_FIFO / 2)) status |= 0x20;
            if (ch->tx_enabled) status |= 0x08;
            if (ch->tx_enabled && (ch->tx_count < CD1400_FIFO / 2)) status |= 0x04;
            if (ch->tx_enabled && (ch->tx_count >= CD1400_FIFO / 2)) status |= 0x02;
            return status;
        }
        case REG_RDCR: return (uint8_t) ch->rx_count;
        case REG_RDSR:
            if (ch->rx_count) {
                uint8_t value;
                if (!ch->rx_status_pending && ch->rx_status[ch->rx_head]) {
                    ch->rx_status_pending = 1;
                    return ch->rx_status[ch->rx_head];
                }
                value = ch->rx[ch->rx_head];
                ch->rx_head = (ch->rx_head + 1) % CD1400_FIFO;
                ch->rx_count--;
                ch->rx_status_pending = 0;
                ch->rx_idle_us = 0.0;
                if (!ch->rx_count) {
                    ch->rx_timeout = 0;
                    cd1400_update_requests(dev);
                }
                return value;
            }
            return 0;
        default: return ch->reg[reg];
    }
}

uint8_t
cd1400_read(uint16_t addr, void *priv)
{
    cd1400_t *dev = (cd1400_t *) priv;
    uint8_t reg = addr & 0x7f;
    cd1400_channel_t *ch;
    if (dev == NULL)
        return 0xff;
    ch = &dev->channel[dev->car & 3];
    switch (reg) {
        case REG_GFRCR: return 0x48; /* CD1400 firmware revision */
        case REG_CAR: return dev->car;
        case REG_GCR: return dev->global[reg];
        case REG_SVRR: cd1400_update_requests(dev); return dev->service;
        case REG_RICR: cd1400_update_requests(dev); return dev->current_rx;
        case REG_TICR: cd1400_update_requests(dev); return dev->current_tx;
        case REG_MICR: cd1400_update_requests(dev); return dev->current_modem;
        case REG_RIR: return dev->global[REG_RIVR];
        case REG_TIR: return dev->global[REG_TIVR];
        case REG_MIR: return dev->global[REG_MIVR];
        case REG_PPR: return dev->global[reg];
        case REG_RDSR: return cd1400_read_channel(dev, ch, REG_RDSR);
        case REG_MISR: {
            uint8_t value = ch->modem_delta;
            ch->modem_delta = 0;
            cd1400_update_requests(dev);
            return value;
        }
        case 0x6c: return (uint8_t) (ch->modem_in | (ch->modem_out & 1));
        case 0x6d: return (uint8_t) (ch->modem_in | (ch->modem_out & 2));
        case REG_TDR: return 0;
        case REG_EOSRR: return 0;
        default:
            return cd1400_is_global(reg) ? dev->global[reg] : cd1400_read_channel(dev, ch, reg);
    }
}

void
cd1400_write(uint16_t addr, uint8_t val, void *priv)
{
    cd1400_t *dev = (cd1400_t *) priv;
    uint8_t reg = addr & 0x7f;
    cd1400_channel_t *ch;
    if (dev == NULL)
        return;
    ch = &dev->channel[dev->car & 3];
    switch (reg) {
        case REG_CAR:
            dev->car = val;
            break;
        case REG_CCR:
            switch (val) {
                case CCR_RESETCHAN:
                    memset(ch->reg, 0, sizeof(ch->reg));
                    ch->rx_head = ch->rx_count = 0;
                    ch->tx_head = ch->tx_count = 0;
                    ch->rx_enabled = ch->tx_enabled = 0;
                    ch->rx_overrun = 0;
                    ch->tx_busy = ch->etc_pending = ch->break_active = 0;
                    timer_disable(&ch->tx_timer);
                    cd1400_update_modem_outputs(ch);
                    break;
                case CCR_RESETFULL:
                    cd1400_reset(dev);
                    break;
                case CCR_TXFLUSHFIFO:
                    ch->tx_head = ch->tx_count = 0;
                    ch->tx_busy = ch->etc_pending = 0;
                    timer_disable(&ch->tx_timer);
                    break;
                case CCR_CORCHANGE:
                    /* Channel options are already live in this simple model. */
                    break;
                case 0x21: case 0x22: case 0x23: case 0x24: {
                    unsigned index = val & 3;
                    if (ch->tx_count < CD1400_FIFO) {
                        unsigned tail = (ch->tx_head + ch->tx_count) % CD1400_FIFO;
                        ch->tx[tail] = ch->reg[0x1a + index];
                        ch->tx_count++;
                        cd1400_tx_start(ch);
                    }
                    break;
                }
                case CCR_TXENABLE:
                    ch->tx_enabled = 1;
                    ch->reg[REG_CCSR] |= 0x08;
                    cd1400_tx_start(ch);
                    break;
                case CCR_TXDISABLE:
                    ch->tx_enabled = 0;
                    ch->tx_busy = 0;
                    timer_disable(&ch->tx_timer);
                    ch->reg[REG_CCSR] &= (uint8_t) ~0x08;
                    break;
                case CCR_RXENABLE:
                    ch->rx_enabled = 1;
                    ch->reg[REG_CCSR] |= 0x80;
                    break;
                case CCR_RXDISABLE:
                    ch->rx_enabled = 0;
                    ch->reg[REG_CCSR] &= (uint8_t) ~0x80;
                    break;
                default:
                    break;
            }
            break;
        case REG_EOSRR:
            if ((dev->ack_type == SVRR_MODEM) && (dev->ack_channel < CD1400_CHANNELS))
                dev->channel[dev->ack_channel].modem_delta = 0;
            dev->ack_type = 0;
            dev->ack_channel = 0xff;
            cd1400_update_requests(dev);
            break;
        case REG_TDR: {
            if (ch->tx_count < CD1400_FIFO) {
                unsigned tail = (ch->tx_head + ch->tx_count) % CD1400_FIFO;
                ch->tx[tail] = val;
                ch->tx_count++;
                cd1400_tx_start(ch);
            }
            cd1400_update_requests(dev);
            break;
        }
        case REG_MISR:
            ch->modem_delta &= (uint8_t) ~val;
            cd1400_update_requests(dev);
            break;
        case REG_RDSR:
            break; /* read-only virtual register */
        case 0x6c: /* MSVR1: DTR output plus modem inputs */
            ch->modem_out = (ch->modem_out & (uint8_t) ~1) | (val & 1);
            ch->reg[0x6c] = val & 1;
            cd1400_update_modem_outputs(ch);
            break;
        case 0x6d: /* MSVR2: RTS output plus modem inputs */
            ch->modem_out = (ch->modem_out & (uint8_t) ~2) | (val & 2);
            ch->reg[0x6d] = val & 2;
            cd1400_update_modem_outputs(ch);
            break;
        default:
            if (!cd1400_is_global(reg)) {
                ch->reg[reg] = val;
                if (reg == REG_COR1 || reg == REG_TBPR || reg == REG_TCOR || reg == REG_RBPR || reg == REG_RCOR)
                    cd1400_update_format(ch);
            } else
                dev->global[reg] = val;
            break;
    }
}

static void *
cd1400_init(const device_t *info)
{
    cd1400_t *dev = (cd1400_t *) calloc(1, sizeof(cd1400_t));
    (void) info;
    for (unsigned i = 0; i < CD1400_CHANNELS; i++) {
        cd1400_channel_t *ch = &dev->channel[i];
        ch->owner = dev;
        ch->number = i;
        serial_endpoint_init(&ch->endpoint, ch, &cd1400_ops);
        ch->endpoint.on_attach = cd1400_backend_changed;
        timer_add(&ch->tx_timer, cd1400_tx_tick, ch, 0);
        timer_add(&ch->rx_timer, cd1400_rx_tick, ch, 0);
        timer_set_delay_u64(&ch->rx_timer, 1000 * TIMER_USEC);
    }
    dev->global[REG_PPR] = 0xff;
    dev->global[REG_GCR] = 0;
    for (unsigned i = 0; i < CD1400_CHANNELS; i++) {
        dev->channel[i].reg[REG_RBPR] = 0x41;
        dev->channel[i].reg[REG_TBPR] = 0x41;
        dev->channel[i].reg[REG_RCOR] = 0x01;
        dev->channel[i].reg[REG_TCOR] = 0x81;
        dev->channel[i].modem_out = 0;
        cd1400_update_format(&dev->channel[i]);
        cd1400_update_modem_outputs(&dev->channel[i]);
    }
    return dev;
}

static void
cd1400_close(void *priv)
{
    cd1400_t *dev = (cd1400_t *) priv;
    if (dev == NULL)
        return;
    for (unsigned i = 0; i < CD1400_CHANNELS; i++) {
        timer_disable(&dev->channel[i].tx_timer);
        timer_disable(&dev->channel[i].rx_timer);
        serial_endpoint_detach(&dev->channel[i].endpoint);
    }
    free(priv);
}

static void
cd1400_reset(void *priv)
{
    cd1400_t *dev = (cd1400_t *) priv;
    for (unsigned i = 0; i < CD1400_CHANNELS; i++) {
        memset(dev->channel[i].reg, 0, sizeof(dev->channel[i].reg));
        dev->channel[i].rx_head = dev->channel[i].rx_count = 0;
        dev->channel[i].tx_head = dev->channel[i].tx_count = 0;
        dev->channel[i].rx_status_pending = 0;
        dev->channel[i].rx_timeout = 0;
        dev->channel[i].rx_idle_us = 0.0;
        dev->channel[i].tx_busy = 0;
        dev->channel[i].etc_pending = dev->channel[i].break_active = 0;
        dev->channel[i].rx_enabled = dev->channel[i].tx_enabled = 0;
        dev->channel[i].modem_delta = 0;
        dev->channel[i].rx_overrun = 0;
        timer_disable(&dev->channel[i].tx_timer);
        dev->channel[i].modem_out = 0;
        cd1400_update_modem_outputs(&dev->channel[i]);
    }
    memset(dev->global, 0, sizeof(dev->global));
    dev->global[REG_PPR] = 0xff;
    for (unsigned i = 0; i < CD1400_CHANNELS; i++) {
        dev->channel[i].reg[REG_RBPR] = 0x41;
        dev->channel[i].reg[REG_TBPR] = 0x41;
        dev->channel[i].reg[REG_RCOR] = 0x01;
        dev->channel[i].reg[REG_TCOR] = 0x81;
        cd1400_update_format(&dev->channel[i]);
    }
    dev->car = dev->service = 0;
    dev->ack_type = 0;
    dev->ack_channel = 0xff;
}

static void
cd1400_tx_tick(void *priv)
{
    cd1400_channel_t *ch = (cd1400_channel_t *) priv;
    if (!ch->tx_enabled || !ch->tx_count) {
        ch->tx_busy = 0;
        cd1400_update_requests(ch->owner);
        return;
    }
    uint8_t byte = ch->tx[ch->tx_head];
    ch->tx_head = (ch->tx_head + 1) % CD1400_FIFO;
    ch->tx_count--;
    if (ch->reg[REG_COR2] & 0x20) {
        if (ch->etc_pending) {
            if (byte == ETC_STARTBREAK)
                ch->break_active = 1;
            else if (byte == ETC_STOPBREAK)
                ch->break_active = 0;
            ch->etc_pending = 0;
            cd1400_update_modem_outputs(ch);
        } else if (byte == ETC_CMD) {
            ch->etc_pending = 1;
        } else if (ch->endpoint.be && ch->endpoint.be->write_byte) {
            ch->endpoint.be->write_byte(ch->endpoint.be->priv, byte);
        }
    } else if (ch->endpoint.be && ch->endpoint.be->write_byte) {
        ch->endpoint.be->write_byte(ch->endpoint.be->priv, byte);
    }
    ch->tx_busy = 0;
    cd1400_tx_start(ch);
    cd1400_update_requests(ch->owner);
}

static void
cd1400_tx_start(cd1400_channel_t *ch)
{
    if (!ch->tx_enabled || !ch->tx_count || ch->tx_busy)
        return;
    if ((ch->reg[REG_COR2] & 0x02) && ch->endpoint.be && ch->endpoint.be->status && !(ch->modem_in & 0x40))
        return;
    ch->tx_busy = 1;
    timer_on_auto(&ch->tx_timer, cd1400_frame_us(ch));
}

static void
cd1400_rx_tick(void *priv)
{
    cd1400_channel_t *ch = (cd1400_channel_t *) priv;
    if (ch->rx_enabled && ch->endpoint.be && ch->endpoint.be->read_byte) {
        int byte = ch->endpoint.be->read_byte(ch->endpoint.be->priv);
        if (byte >= 0)
            cd1400_receive(ch, (uint8_t) byte);
    } else if (!ch->rx_enabled && ch->endpoint.be && ch->endpoint.be->read_byte) {
        for (unsigned i = 0; i < CD1400_FIFO; i++)
            if (ch->endpoint.be->read_byte(ch->endpoint.be->priv) < 0)
                break;
    }
    if (ch->endpoint.be && ch->endpoint.be->status) {
        uint32_t status = ch->endpoint.be->status(ch->endpoint.be->priv);
        cd1400_set_line(ch, CHAR_COM_CTS, !!(status & CHAR_COM_CTS));
        cd1400_set_line(ch, CHAR_COM_DSR, !!(status & CHAR_COM_DSR));
        cd1400_set_line(ch, CHAR_COM_RI,  !!(status & CHAR_COM_RI));
        cd1400_set_line(ch, CHAR_COM_DCD, !!(status & CHAR_COM_DCD));
    }
    if (ch->rx_count) {
        double timeout_us = (double) ch->reg[0x21] * ((ch->owner->global[REG_PPR] + 1) * 20.48);
        if (timeout_us > 0.0) {
            ch->rx_idle_us += 1000.0;
            if (ch->rx_idle_us >= timeout_us)
                ch->rx_timeout = 1;
        }
    }
    cd1400_update_requests(ch->owner);
    timer_advance_u64(&ch->rx_timer, 1000 * TIMER_USEC);
}

uint8_t
cd1400_service_request(void *priv)
{
    cd1400_t *dev = (cd1400_t *) priv;
    if (dev == NULL)
        return 0;
    cd1400_update_requests(dev);
    return dev->service;
}

uint8_t
cd1400_service_ack(void *priv, uint8_t service_type)
{
    cd1400_t *dev = (cd1400_t *) priv;
    uint8_t channel = 0xff, ack = 0;
    if (dev == NULL)
        return 0xff;
    cd1400_update_requests(dev);
    if (service_type == SVRR_RX) {
        channel = dev->current_rx;
        ack = ACK_RX;
        if (channel != 0xff)
            dev->global[REG_RIVR] = (uint8_t) ((dev->channel[channel].reg[REG_LIVR] & 0xf8) | ACK_RX);
    } else if (service_type == SVRR_TX) {
        channel = dev->current_tx;
        ack = ACK_TX;
        if (channel != 0xff)
            dev->global[REG_TIVR] = (uint8_t) ((dev->channel[channel].reg[REG_LIVR] & 0xf8) | ACK_TX);
    } else if (service_type == SVRR_MODEM) {
        channel = dev->current_modem;
        ack = ACK_MODEM;
        if (channel != 0xff)
            dev->global[REG_MIVR] = (uint8_t) ((dev->channel[channel].reg[REG_LIVR] & 0xf8) | ACK_MODEM);
    }
    if (channel == 0xff)
        return 0xff;
    dev->ack_type = service_type;
    dev->ack_channel = channel;
    return (uint8_t) ((channel << 3) | ack);
}

const device_t cd1400_device = {
    .name = "Cirrus Logic CL-CD1400 uXART",
    .internal_name = "cd1400",
    .flags = 0,
    .local = 0,
    .init = cd1400_init,
    .close = cd1400_close,
    .reset = cd1400_reset,
    .available = NULL,
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = NULL
};
