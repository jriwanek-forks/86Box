/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          IBM SDLC emulation.
 *
 * Authors: Jasmine Iwanek, <jasmine@iwanek.co.uk>
 *
 *          Copyright 2022-2025 Jasmine Iwanek.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/machine.h>
#include <86box/io.h>
#include <86box/pic.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/sdlc.h>
#include <86box/plat_unused.h>

#define SDLC_IO_BASE          0x3a0 // Base I/O address for SDLC adapter
#define SDLC_IO_LEN           0x0d  // I/O length for full register set

#define SDLC_PPI_PORT_A       0x00  // 8255A-5 Port A: input (modem/timer status)
#define SDLC_PPI_PORT_B       0x01  // 8255A-5 Port B: output (modem control)
#define SDLC_PPI_PORT_C       0x02  // 8255A-5 Port C: mixed I/O (gating + data)
#define SDLC_PPI_CONTROL      0x03  // 8255A-5 Mode Set

#define SDLC_PIT_COUNTER_0    0x04  // 8253-5 PIT Counter 0
#define SDLC_PIT_COUNTER_1    0x05  // 8253-5 PIT Counter 1
#define SDLC_PIT_COUNTER_2    0x06  // 8253-5 PIT Counter 2
#define SDLC_PIT_MODE         0x07  // 8253-5 PIT Mode register

#define SDLC_8273_COMMAND     0x08  // 8273 SDLC Command/Status
#define SDLC_8273_PARAMETER   0x09  // 8273 SDLC Parameter/Result
#define SDLC_8273_TX_ACK      0x0A  // 8273 Transmit INT Acknowledge
#define SDLC_8273_RX_ACK      0x0B  // 8273 Receive INT Acknowledge
#define SDLC_8273_DATA        0x0C  // 8273 SDLC Data register

typedef struct {
    uint8_t port[3]; // Ports A, B, C

    uint8_t control;
} ppi8255_t;

typedef struct sdlc_s {
    // 8255A-5 PPI
    ppi8255_t ppi;
#if 0
    uint8_t ppi_ports[3]; // Ports A, B, C
    uint8_t ppi_control;
#endif

    // 8253-5 PIT
    uint8_t pit_counters[3];
    uint8_t pit_mode;

    // 8273 SDLC Controller
    uint8_t sdlc_cmd_status;
    uint8_t sdlc_param_result;
    uint8_t sdlc_tx_int_status;
    uint8_t sdlc_rx_int_status;
    uint8_t sdlc_data;

    // IRQ states
    uint8_t irq3; // Tx/Rx INT (IRQ 3)
    uint8_t irq4; // Modem & Timer INT (IRQ 4)
} sdlc_t;

#ifdef ENABLE_SDLC_LOG
int sdlc_do_log = ENABLE_SDLC_LOG;

static void
sdlc_log(const char *fmt, ...)
{
    va_list ap;

    if (sdlc_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define sdlc_log(fmt, ...)
#endif

static void
sdlc_update_irqs(sdlc_t *sdlc)
{
    sdlc_log("SDLC: update_irqs - IRQ3=%d, IRQ4=%d\n", sdlc->irq3, sdlc->irq4);

#if 0
    pic_set_irq_line(3, sdlc->irq3 ? 1 : 0);
    pic_set_irq_line(4, sdlc->irq4 ? 1 : 0);
#endif
}

// --- I/O Write ---
void
sdlc_write(uint16_t addr, uint8_t val, void *priv)
{
    sdlc_t *sdlc   = (sdlc_t *) priv;
    uint8_t offset = addr - SDLC_IO_BASE;

    switch (offset) {
        case SDLC_PPI_PORT_A:
            sdlc_log("SDLC: write - addr=0x%03x (PPI Port A) val=0x%02x\n", addr, val);
            sdlc->ppi.port[0] = val;
            break;

        case SDLC_PPI_PORT_B:
            sdlc_log("SDLC: write - addr=0x%03x (PPI Port B) val=0x%02x\n", addr, val);
            sdlc->ppi.port[1] = val;
            break;

        case SDLC_PPI_PORT_C:
            sdlc_log("SDLC: write - addr=0x%03x (PPI Port C) val=0x%02x\n", addr, val);
            sdlc->ppi.port[2] = val;
            break;

        case SDLC_PPI_CONTROL:
            sdlc_log("SDLC: write - addr=0x%03x (PPI Control) val=0x%02x\n", addr, val);
            sdlc->ppi.control = val;
            break;

        case SDLC_PIT_COUNTER_0:
        case SDLC_PIT_COUNTER_1:
        case SDLC_PIT_COUNTER_2:
        {
            int counter = offset - SDLC_PIT_COUNTER_0;
            sdlc_log("SDLC: write - addr=0x%03x (PIT Counter %d) val=0x%02x\n", addr, counter, val);
            sdlc->pit_counters[counter] = val;
            sdlc->irq4 = 1; // Timer INT for PIT1/PIT2
            sdlc_update_irqs(sdlc);
            break;
        }

        case SDLC_PIT_MODE:
            sdlc_log("SDLC: write - addr=0x%03x (PIT Mode Control) val=0x%02x\n", addr, val);
            sdlc->pit_mode = val;
            break;

        case SDLC_8273_COMMAND:
            sdlc_log("SDLC: write - addr=0x%03x (8273 Command) val=0x%02x\n", addr, val);
            sdlc->sdlc_cmd_status    = 0x01; // Simulated "command accepted"
            sdlc->sdlc_tx_int_status = 0x80; // Simulate TX Ready
            sdlc->sdlc_rx_int_status = 0x40; // Simulate RX Ready
            sdlc->irq3 = 1;
            sdlc_update_irqs(sdlc);
            break;

        case SDLC_8273_PARAMETER:
            sdlc_log("SDLC: write - addr=0x%03x (8273 Parameter) val=0x%02x\n", addr, val);
            sdlc->sdlc_param_result = val;
            break;

        case SDLC_8273_TX_ACK:
            sdlc_log("SDLC: write - addr=0x%03x (8273 TX ACK) val=0x%02x\n", addr, val);
            sdlc->sdlc_tx_int_status = val;
            sdlc->irq3 = 0;
            sdlc_update_irqs(sdlc);
            break;

        case SDLC_8273_RX_ACK:
            sdlc_log("SDLC: write - addr=0x%03x (8273 RX ACK) val=0x%02x\n", addr, val);
            sdlc->sdlc_rx_int_status = val;
            sdlc->irq3 = 0;
            sdlc_update_irqs(sdlc);
            break;

        case SDLC_8273_DATA:
            sdlc_log("SDLC: write - addr=0x%03x (8273 Data) val=0x%02x\n", addr, val);
            sdlc->sdlc_data = val;
            sdlc->sdlc_rx_int_status |= 0x01; // Simulate echo receive
            sdlc->irq3 = 1;
            sdlc_update_irqs(sdlc);
            break;

        default:
            sdlc_log("SDLC: write - addr=0x%03x (unknown offset 0x%02x) val=0x%02x\n", addr, offset, val);
            break;
    }
}

// --- I/O Read ---
uint8_t
sdlc_read(uint16_t addr, void *priv)
{
    sdlc_t *sdlc   = (sdlc_t *) priv;
    uint8_t offset = addr - SDLC_IO_BASE;
    uint8_t val    = 0xff;

    switch (offset) {
        case SDLC_PPI_PORT_A:
            val = sdlc->ppi.port[0];
            sdlc_log("SDLC: read - addr=0x%03x (PPI Port A) val=0x%02x\n", addr, val);
            break;

        case SDLC_PPI_PORT_B:
            val = sdlc->ppi.port[1];
            sdlc_log("SDLC: read - addr=0x%03x (PPI Port B) val=0x%02x\n", addr, val);
            break;

        case SDLC_PPI_PORT_C:
            val = sdlc->ppi.port[2];
            sdlc_log("SDLC: read - addr=0x%03x (PPI Port C) val=0x%02x\n", addr, val);
            break;

        case SDLC_PPI_CONTROL:
            val = sdlc->ppi.control;
            sdlc_log("SDLC: read - addr=0x%03x (PPI Control) val=0x%02x\n", addr, val);
            break;

        case SDLC_PIT_COUNTER_0:
        case SDLC_PIT_COUNTER_1:
        case SDLC_PIT_COUNTER_2:
        {
            int counter = offset - SDLC_PIT_COUNTER_0;
            val = sdlc->pit_counters[counter];
            sdlc_log("SDLC: read - addr=0x%03x (PIT Counter %d) val=0x%02x\n", addr, counter, val);
            break;
        }

        case SDLC_PIT_MODE:
            val = sdlc->pit_mode;
            sdlc_log("SDLC: read - addr=0x%03x (PIT Mode Control) val=0x%02x\n", addr, val);
            break;

        case SDLC_8273_COMMAND:
            val = sdlc->sdlc_cmd_status;
            sdlc_log("SDLC: read - addr=0x%03x (8273 Command) val=0x%02x\n", addr, val);
            break;

        case SDLC_8273_PARAMETER:
            val = sdlc->sdlc_param_result;
            sdlc_log("SDLC: read - addr=0x%03x (8273 Parameter) val=0x%02x\n", addr, val);
            break;

        case SDLC_8273_TX_ACK:
            val = sdlc->sdlc_tx_int_status;
            sdlc_log("SDLC: read - addr=0x%03x (8273 TX ACK) val=0x%02x\n", addr, val);
            break;

        case SDLC_8273_RX_ACK:
            val = sdlc->sdlc_rx_int_status;
            sdlc_log("SDLC: read - addr=0x%03x (8273 RX ACK) val=0x%02x\n", addr, val);
            break;

        case SDLC_8273_DATA:
            val = sdlc->sdlc_data;
            sdlc_log("SDLC: read - addr=0x%03x (8273 Data) val=0x%02x\n", addr, val);
            break;

        default:
            sdlc_log("SDLC: read - addr=0x%03x (unknown offset 0x%02x) val=0x%02x\n", addr, offset, val);
            break;
    }

    return val;
}

static void
sdlc_speed_changed(void *priv)
{
    // Not currently used, but reserved for future speed scaling.
#if 0
    sdlc_t *dev = (sdlc_t *) priv;

    sdlc_update_speed(dev);
#endif
}

// --- Reset ---
static void
sdlc_reset(void *priv)
{
    sdlc_t *dev = (sdlc_t *) priv;
    memset(dev, 0, sizeof(sdlc_t));
    dev->sdlc_cmd_status = 0x00;
    sdlc_update_irqs(dev);
}

static void
sdlc_close(void *priv)
{
    sdlc_t *dev = (sdlc_t *) priv;

    free(dev);
}

static void *
sdlc_init(UNUSED(const device_t *info))
{
    sdlc_t *dev = (sdlc_t *) calloc(1, sizeof(sdlc_t));

    io_sethandler(SDLC_IO_BASE, SDLC_IO_LEN,
                  sdlc_read, NULL, NULL,
                  sdlc_write, NULL, NULL,
                  dev);

    // TODO: IRQ

    return dev;
}

const device_t ibm_sdlc_device = {
    .name          = "IBM SDLC Adapter",
    .internal_name = "ibmsdlc",
    .flags         = 0,
    .local         = 0,
    .init          = sdlc_init,
    .close         = sdlc_close,
    .reset         = sdlc_reset,
    .available     = NULL,
    .speed_changed = sdlc_speed_changed,
    .force_redraw  = NULL,
    .config        = NULL
};
