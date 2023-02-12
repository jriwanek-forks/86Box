/*
 * 86Box        A hypervisor and IBM PC system emulator that specializes in
 *              running old operating systems and software designed for IBM
 *              PC systems and compatibles from 1981 through fairly recent
 *              system designs based on the PCI bus.
 *
 *              This file is part of the 86Box distribution.
 *
 *              Implementation of Serial passthrough device.
 *
 *
 * Authors:     Andreas J. Reichel <webmaster@6th-dimension.com>,
 *              Jasmine Iwanek <jasmine@iwanek.co.uk>
 *
 *              Copyright 2021      Andreas J. Reichel.
 *              Copyright 2021-2022 Jasmine Iwanek.
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/serial.h>
#include <86box/serial_passthrough.h>
#include <86box/plat_serial_passthrough.h>

#define ENABLE_SERIAL_PASSTHROUGH_LOG 1
#ifdef ENABLE_SERIAL_PASSTHROUGH_LOG
int serial_passthrough_do_log = ENABLE_SERIAL_PASSTHROUGH_LOG;

static void
serial_passthrough_log(const char *fmt, ...)
{
    va_list ap;

    if (serial_passthrough_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define serial_passthrough_log(fmt, ...)
#endif

void
serial_passthrough_init(void)
{
    int c;

    for (c = 0; c < SERIAL_MAX; c++) {
        if (serial_passthrough_enabled[c]) {
            /* Instance n for COM n */
            device_add_inst(&serial_passthrough_device, c + 1);
        }
    }
}

static void
serial_passthrough_timers_off(serial_passthrough_t *dev)
{
    timer_stop(&dev->host_to_serial_timer);
}

static void
serial_passthrough_write(serial_t *s, void *priv, uint8_t val)
{
    plat_serpt_write(priv, val);
}

static void
host_to_serial_cb(void *priv)
{
    serial_passthrough_t *dev = (serial_passthrough_t *) priv;

    uint8_t byte;

    /* write_fifo has no failure indication, but if we write to fast, the host
     * can never fetch the bytes in time, so check if the fifo is full if in
     * fifo mode or if lsr has bit 0 set if not in fifo mode */
    if ((dev->serial->type >= SERIAL_16550) && dev->serial->fifo_enabled) {
        if (dev->serial->rcvr_fifo_full) {
            goto no_write_to_machine;
        }
    } else {
        if (dev->serial->lsr & 1) {
            goto no_write_to_machine;
        }
    }
    if (plat_serpt_read(dev, &byte)) {
        // printf("got byte %02X\n", byte);
        serial_write_fifo(dev->serial, byte);
        // serial_set_dsr(dev->serial, 1);
    }
no_write_to_machine:
    // serial_device_timeout(dev->serial);
    timer_on_auto(&dev->host_to_serial_timer, (1000000.0 / dev->baudrate) * (double) dev->bits);
}

static void
serial_passthrough_rcr_cb(struct serial_s *serial, void *priv)
{
    serial_passthrough_t *dev = (serial_passthrough_t *) priv;

    timer_stop(&dev->host_to_serial_timer);
    /* FIXME: do something to dev->baudrate */
    timer_on_auto(&dev->host_to_serial_timer, (1000000.0 / dev->baudrate) * (double) dev->bits);
    // serial_clear_fifo(dev->serial);
}

static void
serial_passthrough_speed_changed(void *priv)
{
    serial_passthrough_t *dev = (serial_passthrough_t *) priv;

    timer_stop(&dev->host_to_serial_timer);
    /* FIXME: do something to dev->baudrate */
    timer_on_auto(&dev->host_to_serial_timer, (1000000.0 / dev->baudrate) * (double) dev->bits);
    // serial_clear_fifo(dev->serial);
}

static void
serial_passthrough_dev_close(void *priv)
{
    serial_passthrough_t *dev = (serial_passthrough_t *) priv;

    /* Detach passthrough device from COM port */
    if (dev && dev->serial && dev->serial->sd)
        memset(dev->serial->sd, 0, sizeof(serial_device_t));

    plat_serpt_close(dev);
    free(dev);
}

void
serial_passthrough_transmit_period(serial_t *serial, void *p, double transmit_period)
{
    serial_passthrough_t *dev = (serial_passthrough_t *) p;

    if (dev->mode != SERPT_MODE_HOSTSER) return;
    dev->baudrate = 1000000.0 / (transmit_period);
    
    serial_passthrough_speed_changed(p);
    plat_serpt_set_params(dev);
}

/* Initialize the device for use by the user. */
static void *
serial_passthrough_dev_init(const device_t *info)
{
    serial_passthrough_t *dev;

    dev = (serial_passthrough_t *) malloc(sizeof(serial_passthrough_t));
    memset(dev, 0, sizeof(serial_passthrough_t));
    dev->mode = device_get_config_int("mode");

    dev->port      = device_get_instance() - 1;
    dev->baudrate  = device_get_config_int("baudrate");
    dev->data_bits = device_get_config_int("data_bits");

    /* Attach passthrough device to a COM port */
    dev->serial = serial_attach_ex(dev->port, serial_passthrough_rcr_cb,
                                serial_passthrough_write, serial_passthrough_transmit_period, dev);

    strncpy(dev->host_serial_path, device_get_config_string("host_serial_path"), 1024);

    serial_passthrough_log("%s: port=COM%d\n", info->name, dev->port + 1);
    serial_passthrough_log("%s: baud=%f\n", info->name, dev->baudrate);
    serial_passthrough_log("%s: mode=%s\n", info->name, serpt_mode_names[dev->mode]);

    if (plat_serpt_open_device(dev)) {
        serial_passthrough_log("%s: not running\n", info->name);
        return NULL;
    }
    serial_passthrough_log("%s: running\n", info->name);

    memset(&dev->host_to_serial_timer, 0, sizeof(pc_timer_t));
    timer_add(&dev->host_to_serial_timer, host_to_serial_cb, dev, 1);
    serial_set_cts(dev->serial, 1);
    serial_set_dsr(dev->serial, 1);
    serial_set_dcd(dev->serial, 1);

    /* 1 start bit + data bits + stop bits (no parity assumed) */
    dev->bits = 1 + device_get_config_int("data_bits") + device_get_config_int("stop_bits");

    /* Return our private data to the I/O layer. */
    return dev;
}

const char *serpt_mode_names[SERPT_MODES_MAX] = {
    [SERPT_MODE_VCON]    = "vcon",
    [SERPT_MODE_TCP]     = "tcp",
    [SERPT_MODE_HOSTSER] = "hostser",
};

// clang-format off
static const device_config_t serial_passthrough_config[] = {
    {
        "mode", "Passthrough Mode", CONFIG_SELECTION, "", 0, "", { 0 },
        {
            {
#ifdef _WIN32
                "Named Pipe (server)", 0
#else
                "Pseudo Terminal/Virtual Console", 0
#endif
            },
            {
                "Host Serial Passthrough", 2
            },
            {
                ""
            }
        }
    },
    {
        .name = "host_serial_path",
        .description = "Host Serial Device",
        .type = CONFIG_SERPORT,
        .default_string = "",
        .file_filter = NULL,
        .spinner = {},
        .selection = {}
    },
    {
        .name = "data_bits",
        .description = "Data bits",
        .type = CONFIG_SELECTION,
        .default_string = "8",
        .default_int = 8,
        .file_filter = NULL,
        .spinner = {},
        .selection = {
            { "5", 5 },
            { "6", 6 },
            { "7", 7 },
            { "8", 8 }
        }
    },
    {
        .name = "stop_bits",
        .description = "Stop bits",
        .type = CONFIG_SELECTION,
        .default_string = "1",
        .default_int = 1,
        .file_filter = NULL,
        .spinner = {},
        .selection = {
            { "1", 1 },
            { "2", 2 }
        }
    },
    {
        "baudrate", "Baud Rate of Passthrough", CONFIG_SELECTION, "", 115200, "", { 0 }, {
            {
                "115200", 115200
            },
            {
                "57600", 57600
            },
            {
                "38400", 38400
            },
            {
                "19200", 19200
            },
            {
                "9600", 9600
            },
            {
                "4800", 4800
            },
            {
                "2400", 2400
            },
            {
                "1200", 1200
            },
            {
                "300", 300
            },
            {
                "150", 150
            }
        }
    },
    {
        "", "", -1
    }
};
// clang-format on

const device_t serial_passthrough_device = {
    .name  = "Serial Passthrough Device",
    .flags = 0,
    .local = 0,
    .init  = serial_passthrough_dev_init,
    .close = serial_passthrough_dev_close,
    .reset = NULL,
    { .poll = NULL },
    .speed_changed = serial_passthrough_speed_changed,
    .force_redraw  = NULL,
    .config        = &serial_passthrough_config[0]
};
