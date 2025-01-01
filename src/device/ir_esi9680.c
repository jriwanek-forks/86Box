/*
 * 86Box     A hypervisor and IBM PC system emulator that specializes in
 *           running old operating systems and software designed for IBM
 *           PC systems and compatibles from 1981 through fairly recent
 *           system designs based on the PCI bus.
 *
 *           This file is part of the 86Box distribution.
 *
 *           ESI-9680 JetEye PC dongle emulation.
 *
 *           The dongle takes its power from DTR and RTS, which also select
 *           its speed:
 *
 *               DTR  RTS  Speed
 *                0    0   Powered off
 *                0    1   9600 bps
 *                1    0   19200 bps
 *                1    1   115200 bps
 *
 * Authors:  Cacodemon345
 *
 *           Copyright 2024-2026 Cacodemon345.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/char.h>
#include <86box/log.h>
#include <86box/plat.h>
#include <86box/fifo8.h>
#include <86box/irda.h>

#define ESI9680_FIFO_SIZE 0x10000

/* IrDA SIR characters are always 8N1. */
#define ESI9680_CHAR_BITS 10.0

typedef struct esi9680_t {
    irda_device_t irda;
    void         *log;
    char_port_t  *port;
    pc_timer_t    timer;

    double   char_period; /* Microseconds per character, 0 when powered off. */
    uint32_t rx_arrived;  /* How much of the FIFO has arrived in full. */
    Fifo8    fifo;        /* FIFO buffer */
} esi9680_t;

#ifdef ENABLE_ESI9680_LOG
int esi9680_do_log = ENABLE_ESI9680_LOG;

static void
esi9680_log(void *priv, const char *fmt, ...)
{
    va_list ap;

    if (esi9680_do_log) {
        va_start(ap, fmt);
        log_out(priv, fmt, ap);
        va_end(ap);
    }
}
#else
#    define esi9680_log(priv, fmt, ...)
#endif

/* What has been sent is on its way until the timer says it has arrived. The
   UART polls for data much more often than characters arrive on the infrared
   link, and this keeps the link's own pace regardless of when it does. */
static int
esi9680_rx_pending(esi9680_t *esi9680)
{
    return (esi9680->char_period > 0.0) && (esi9680->rx_arrived < fifo8_num_used(&esi9680->fifo));
}

static void
esi9680_restart_timer(esi9680_t *esi9680)
{
    timer_stop(&esi9680->timer);

    if (esi9680_rx_pending(esi9680))
        timer_on_auto(&esi9680->timer, esi9680->char_period);
}

static void
esi9680_timer(void *priv)
{
    esi9680_t *esi9680 = (esi9680_t *) priv;

    esi9680->rx_arrived++;

    if (esi9680_rx_pending(esi9680))
        timer_on_auto(&esi9680->timer, esi9680->char_period);
}

static void
esi9680_receive(void *priv, uint8_t data)
{
    esi9680_t *esi9680 = (esi9680_t *) priv;

    if ((esi9680->char_period <= 0.0) || fifo8_is_full(&esi9680->fifo))
        return;

    fifo8_push(&esi9680->fifo, data);

    if (!timer_is_enabled(&esi9680->timer))
        timer_on_auto(&esi9680->timer, esi9680->char_period);
}

static size_t
esi9680_read(uint8_t *buf, size_t len, void *priv)
{
    esi9680_t *esi9680 = (esi9680_t *) priv;

    if ((len == 0) || (esi9680->rx_arrived == 0))
        return 0;

    buf[0] = fifo8_pop(&esi9680->fifo);
    esi9680->rx_arrived--;

    return 1;
}

static size_t
esi9680_write(uint8_t *buf, size_t len, void *priv)
{
    esi9680_t *esi9680 = (esi9680_t *) priv;

    if (esi9680->char_period > 0.0) {
        for (size_t i = 0; i < len; i++)
            irda_broadcast_data(&esi9680->irda, buf[i]);
    }

    return len;
}

static uint32_t
esi9680_status(void *priv)
{
    const esi9680_t *esi9680 = (esi9680_t *) priv;

    return (esi9680->char_period > 0.0) ? (CHAR_COM_CTS | CHAR_COM_DSR | CHAR_COM_DCD) : 0;
}

static void
esi9680_control(uint32_t flags, void *priv)
{
    esi9680_t *esi9680 = (esi9680_t *) priv;
    const int  dtr     = !!(flags & CHAR_COM_DTR);
    const int  rts     = !!(flags & CHAR_COM_RTS);
    const int  baud    = dtr ? (rts ? 115200 : 19200) : (rts ? 9600 : 0);
    double     period  = 0.0;

    if (baud != 0)
        period = (1000000.0 / (double) baud) * ESI9680_CHAR_BITS;

    if (period == esi9680->char_period)
        return;

    esi9680_log(esi9680->log, "DTR = %i, RTS = %i, %i bps\n", dtr, rts, baud);

    esi9680->char_period = period;

    /* Whatever had not reached the UART is lost with the power. */
    if (baud == 0) {
        fifo8_reset(&esi9680->fifo);
        esi9680->rx_arrived = 0;
    }

    esi9680_restart_timer(esi9680);
    char_update_status(esi9680->port);
}

static void
esi9680_reset(void *priv)
{
    esi9680_t *esi9680 = (esi9680_t *) priv;

    /* The lines stay as they are: the UART reports its own reset through them. */
    fifo8_reset(&esi9680->fifo);
    esi9680->rx_arrived = 0;
    timer_stop(&esi9680->timer);
}

static void
esi9680_speed_changed(void *priv)
{
    esi9680_restart_timer((esi9680_t *) priv);
}

static void *
esi9680_init(UNUSED(const device_t *info))
{
    esi9680_t *esi9680 = (esi9680_t *) calloc(1, sizeof(esi9680_t));

    fifo8_create(&esi9680->fifo, ESI9680_FIFO_SIZE);
    timer_add(&esi9680->timer, esi9680_timer, esi9680, 0);

    esi9680->irda.write = esi9680_receive;
    esi9680->irda.priv  = esi9680;
    irda_register_device(&esi9680->irda);

    esi9680->port = char_attach(0, esi9680_read, esi9680_write, esi9680_status,
                                esi9680_control, NULL, esi9680);
    esi9680->log  = char_log_open(esi9680->port, "ESI-9680");
    esi9680_log(esi9680->log, "init()\n");

    device_add_inst(&irda_obex_device, device_get_instance());

    return esi9680;
}

static void
esi9680_close(void *priv)
{
    esi9680_t *esi9680 = (esi9680_t *) priv;

    esi9680_log(esi9680->log, "close()\n");

    timer_stop(&esi9680->timer);
    irda_unregister_device(&esi9680->irda);

    log_close(esi9680->log);
    fifo8_destroy(&esi9680->fifo);
    free(esi9680);
}

const device_t esi9680_device = {
    .name          = "ESI-9680 JetEye PC",
    .internal_name = "esi9680",
    .flags         = DEVICE_COM | DEVICE_HOTPLUG,
    .local         = 0,
    .init          = esi9680_init,
    .close         = esi9680_close,
    .reset         = esi9680_reset,
    .available     = NULL,
    .speed_changed = esi9680_speed_changed,
    .force_redraw  = NULL,
    .config        = NULL
};
