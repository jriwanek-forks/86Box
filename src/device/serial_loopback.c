/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of serial port loopback devices.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#define ENABLE_SERIAL_LOOPBACK_LOG 1

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#ifdef ENABLE_SERIAL_LOOPBACK_LOG
#include <stdarg.h>
#define HAVE_STDARG_H
#endif
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/fifo.h>
#include <86box/fifo8.h>
#include <86box/timer.h>
#include <86box/serial.h>
#include <86box/serial_loopback.h>
#include <86box/plat_unused.h>

/* Standard UART Register Definitions */
#define MCR_DTR          0x01    /* Data Terminal Ready: Controls the DTR output pin */
#define MCR_RTS          0x02    /* Request To Send: Controls the RTS output pin */
#define MCR_LOOP         0x10    /* Local Loopback: Disconnects serial I/O and loops TX to RX internally */

typedef struct serial_loopback_s {
    serial_t   *serial;
    pc_timer_t  echo_timer;
    
    Fifo8       data_pending; // Buffer for bytes waiting to be echoed
    
    uint32_t    baudrate;
    uint8_t     port;
    uint8_t     type;
    int         dtr_state;
    int         rts_state;
} serial_loopback_t;

enum {
    PLUG_CHECKIT = 0,
    PLUG_NORTON,
    PLUG_PASSMARK
};

#ifdef ENABLE_SERIAL_LOOPBACK_LOG
int serial_loopback_do_log = ENABLE_SERIAL_LOOPBACK_LOG;

static void
serial_loopback_log(const char *fmt, ...)
{
    va_list ap;

    if (serial_loopback_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define serial_loopback_log(fmt, ...)
#endif

void
serial_loopback_init(void)
{
    for (uint8_t c = 0; c < (SERIAL_MAX - 1); c++) {
        if (serial_loopback_enabled[c]) {
            /* Instance n for COM n */
            device_add_inst(&serial_loopback_device, c + 1);
        }
    }
}

static void
loopback_sync_pins(serial_loopback_t *dev)
{
    if (!dev->serial)
        return;

    uint8_t mcr = dev->serial->mctrl;

    int rts = !!(mcr & MCR_RTS);
    int dtr = !!(mcr & MCR_DTR);

    switch (dev->type) {
        case PLUG_CHECKIT:
            /* RTS(7)->CTS(8) and DTR(4)->DCD(1), DSR(6), RI(9) */
            serial_set_cts(dev->serial, rts);
            serial_set_dcd(dev->serial, dtr);
            serial_set_dsr(dev->serial, dtr);
            serial_set_ri(dev->serial,  dtr);
            break;

        case PLUG_NORTON:
            /* RTS(7)->CTS(8) and DTR(4)->DCD(1), DSR(6), RI(9) */
            serial_set_cts(dev->serial, rts);
            serial_set_dcd(dev->serial, dtr);
            serial_set_dsr(dev->serial, dtr);
            serial_set_ri(dev->serial,  dtr);
            break;

        case PLUG_PASSMARK:
            /* RTS(7)->CTS(8) and DTR(4)->DCD(1) & DSR(6) */
            serial_set_cts(dev->serial, rts);
            serial_set_dcd(dev->serial, dtr);
            serial_set_dsr(dev->serial, dtr);
            serial_set_ri(dev->serial,  0); // Passmark doesn't bridge RI
            break;
    }

    serial_loopback_log("Loopback: Pins Synced (MCR:%02X RTS->CTS:%d DTR->DSR/DCD:%d)\n", mcr, rts, dtr);
}

static void
loopback_echo_timer_cb(void *priv)
{
    serial_loopback_log("Loopback: loopback_echo_timer_cb\n");
    serial_loopback_t *dev = (serial_loopback_t *) priv;

    if (!dev->serial) return;

    /* Check if UART is ready to receive a byte */
    if ((dev->serial->type >= SERIAL_16550) && dev->serial->fifo_enabled) {
        if (fifo_get_full(dev->serial->rcvr_fifo))
            goto reschedule;
    } else {
        if (dev->serial->lsr & 1) // Data Ready bit is still set
            goto reschedule;
    }

    /* If we have data, push one byte */
    if (fifo8_num_used(&dev->data_pending)) {
        serial_write_fifo(dev->serial, fifo8_pop(&dev->data_pending));
    }

reschedule:
    /* Standard 86Box serial timing: (1,000,000us / baud) * bits_per_frame */
    /* We use 10 bits (8-N-1) as a safe default for the echo delay */
    double interval = (1000000.0 / (double)dev->baudrate) * 10.0;
    timer_on_auto(&dev->echo_timer, interval);
}

static void
loopback_write(serial_t *s, void *priv, uint8_t txval)
{
    serial_loopback_log("Loopback: loopback_write\n");
    serial_loopback_t *dev = (serial_loopback_t *) priv;
    
    /* Push to buffer. The timer handler will move it to the UART. */
    if (fifo8_num_free(&dev->data_pending)) {
        fifo8_push(&dev->data_pending, txval);
    }
}

static void
loopback_rcr_cb(struct serial_s *serial, void *priv)
{
    serial_loopback_log("Loopback: loopback_rcr_cb\n");
    serial_loopback_t *dev = (serial_loopback_t *) priv;
    
    loopback_sync_pins(dev);
}

static void
loopback_dtr_cb(serial_t *serial, int status, void *priv)
{
    serial_loopback_log("Loopback: loopback_dtr_cb\n");
    serial_loopback_t *dev = (serial_loopback_t *) priv;

    loopback_sync_pins(dev);
}

static void
serial_loopback_dev_close(void *priv)
{
    serial_loopback_t *dev = (serial_loopback_t *) priv;
    timer_stop(&dev->echo_timer);
    fifo8_destroy(&dev->data_pending);
    free(dev);
}

static void *
serial_loopback_dev_init(const device_t *info)
{
    serial_loopback_t *dev = (serial_loopback_t *) calloc(1, sizeof(serial_loopback_t));
    
    dev->port     = device_get_config_int("port");
    dev->type     = device_get_config_int("type");
    dev->baudrate = 9600; // Default until UART tells us otherwise

    fifo8_create(&dev->data_pending, 256);

    timer_add(&dev->echo_timer, loopback_echo_timer_cb, dev, 0);

    /* Use serial_attach_ex_2 to get the DTR callback exactly like the modem */
    dev->serial = serial_attach_ex_2(dev->port, 
                                     loopback_rcr_cb,
                                     loopback_write,
                                     loopback_dtr_cb,
                                     dev);

    /* Start the echo loop */
    timer_on_auto(&dev->echo_timer, 1000000.0);

    return dev;
}

static const device_config_t serial_loopback_config[] = {
    // clang-format off
    {
        .name           = "type",
        .description    = "Type",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Checkit (IBM)", .value = PLUG_CHECKIT  },
            { .description = "Norton",        .value = PLUG_NORTON   },
            { .description = "Passmark",      .value = PLUG_PASSMARK },
            { .description = ""                                      }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

const device_t serial_loopback_device = {
    .name          = "Serial Loopback Plug",
    .internal_name = "serial_loopback",
    .flags         = DEVICE_COM,
    .local         = 0,
    .init          = serial_loopback_dev_init,
    .close         = serial_loopback_dev_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = serial_loopback_config
};
