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
 *
 *
 * Authors:  Cacodemon345
 *
 *           Copyright 2024 Cacodemon345.
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/serial.h>
#include <86box/plat.h>
#include <86box/fifo8.h>
#include <86box/irda.h>

typedef struct esi9680_t
{
    irda_device_t irda;
    serial_t* serial;
    pc_timer_t timer;

    int rts, dtr;
    Fifo8    fifo; /* FIFO buffer*/
} esi9680_t;

static const device_config_t esi9680_config[] = {
  // clang-format off
    {
        .name = "port",
        .description = "Serial Port",
        .type = CONFIG_SELECTION,
        .default_string = "",
        .default_int = 0,
        .file_filter = "",
        .spinner = { 0 },
        .selection = {
            { .description = "COM1", .value = 0 },
            { .description = "COM2", .value = 1 },
            { .description = "COM3", .value = 2 },
            { .description = "COM4", .value = 3 },
            { .description = ""                 }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};


void esi9680_update_baud_rate(esi9680_t* esi9680)
{
    if (!esi9680->dtr && !esi9680->rts) {
        timer_disable(&esi9680->timer);
        serial_set_cts(esi9680->serial, 0);
        serial_set_dcd(esi9680->serial, 0);
        serial_set_dsr(esi9680->serial, 0);
        return;
    }

    serial_set_cts(esi9680->serial, 1);
    serial_set_dcd(esi9680->serial, 1);
    serial_set_dsr(esi9680->serial, 1);

    if (!esi9680->dtr && esi9680->rts) {
        timer_on_auto(&esi9680->timer, (1. / 9600.) * 9.);
        return;
    }

    if (esi9680->dtr && !esi9680->rts) {
        timer_on_auto(&esi9680->timer, (1. / 19200.) * 9.);
        return;
    }
    
    if (esi9680->dtr && esi9680->rts) {
        timer_on_auto(&esi9680->timer, (1. / 115200.) * 9.);
        return;
    }
}

void esi9680_write_from_serial_host(UNUSED(struct serial_s *serial), void *priv, uint8_t val)
{
    esi9680_t* esi9680 = (esi9680_t*)priv;

    irda_broadcast_data(&esi9680->irda, val);
}

void esi9680_receive(void *priv, uint8_t data)
{
    esi9680_t* esi9680 = (esi9680_t*)priv;

    fifo8_push(&esi9680->fifo, data);
}

void esi9680_write_to_serial_host(void *priv)
{
    esi9680_t* esi9680 = (esi9680_t*)priv;

    if (!fifo8_is_empty(&esi9680->fifo))
        serial_write_fifo(esi9680->serial, fifo8_pop(&esi9680->fifo));
    esi9680_update_baud_rate(esi9680);
}

static void
esi9680_dtr_callback(UNUSED(struct serial_s *serial), int status, void *priv)
{
    esi9680_t* esi9680 = (esi9680_t*)priv;

    esi9680->dtr = status;
    esi9680_update_baud_rate(esi9680);
}

static void
esi9680_rts_callback(UNUSED(struct serial_s *serial), int status, void *priv)
{
    esi9680_t* esi9680 = (esi9680_t*)priv;

    esi9680->rts = status;
    esi9680_update_baud_rate(esi9680);
}

void* esi9680_init(const device_t* info)
{
    esi9680_t* esi9680 = calloc(1, sizeof(esi9680_t));
    esi9680->serial = serial_attach_ex_3(device_get_config_int("port"), esi9680_rts_callback, esi9680_write_from_serial_host, esi9680_dtr_callback, esi9680);
    esi9680->irda.write = esi9680_receive;
    esi9680->irda.priv = esi9680;
    fifo8_create(&esi9680->fifo, 0x10000);
    irda_register_device(&esi9680->irda);

    serial_set_cts(esi9680->serial, 0);
    serial_set_dcd(esi9680->serial, 0);
    serial_set_dsr(esi9680->serial, 0);

    extern device_t irda_obex_device;
    device_add(&irda_obex_device);

    return esi9680;
}

void esi9680_close(void* priv)
{
    esi9680_t* esi9680 = (esi9680_t*)priv;

    fifo8_destroy(&esi9680->fifo);
    /* Detach serial port from the dongle. */
    if (esi9680 && esi9680->serial && esi9680->serial->sd)
        memset(esi9680->serial->sd, 0, sizeof(serial_device_t));
    free(priv);
}

const device_t esi9680_device = {
    .name          = "ESI-9680 JetEye PC",
    .internal_name = "esi9680",
    .flags         = DEVICE_COM,
    .local         = 0,
    .init          = esi9680_init,
    .close         = esi9680_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = esi9680_config
};
