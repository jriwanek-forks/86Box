/*
 * 86Box     A hypervisor and IBM PC system emulator that specializes in
 *           running old operating systems and software designed for IBM
 *           PC systems and compatibles from 1981 through fairly recent
 *           system designs based on the PCI bus.
 *
 *           This file is part of the 86Box distribution.
 *
 *           IrDA emulation.
 *
 * Authors:  Cacodemon345
 *
 *           Copyright 2024-2026 Cacodemon345.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/log.h>
#include <86box/irda.h>

#define IRDA_MAX_DEVICES 256

#ifdef ENABLE_IRDA_LOG
int irda_do_log = ENABLE_IRDA_LOG;

static void *irda_log_priv;

static void
irda_log(const char *fmt, ...)
{
    va_list ap;

    if (irda_do_log) {
        /* The medium is not a device, so nothing else opens this. */
        if (irda_log_priv == NULL)
            irda_log_priv = log_open("IrDA");

        va_start(ap, fmt);
        log_out(irda_log_priv, fmt, ap);
        va_end(ap);
    }
}
#else
#    define irda_log(fmt, ...)
#endif

static irda_device_t *devices[IRDA_MAX_DEVICES];
static int            irda_count;

void
irda_reset(void)
{
    memset(devices, 0, sizeof(devices));
    irda_count = 0;
}

void
irda_register_device(irda_device_t *irda_device)
{
    if (irda_count >= IRDA_MAX_DEVICES) {
        irda_log("Too many devices\n");
        return;
    }

    devices[irda_count++] = irda_device;
}

void
irda_unregister_device(irda_device_t *irda_device)
{
    for (int i = 0; i < irda_count; i++) {
        if (devices[i] == irda_device) {
            irda_count--;
            memmove(&devices[i], &devices[i + 1], (irda_count - i) * sizeof(devices[0]));
            devices[irda_count] = NULL;
            return;
        }
    }
}

void
irda_broadcast_data(irda_device_t *broadcaster, uint8_t data)
{
    irda_log("Broadcast data: %02X\n", data);

    for (int i = 0; i < irda_count; i++) {
        if ((devices[i] != broadcaster) && (devices[i]->write != NULL))
            devices[i]->write(devices[i]->priv, data);
    }
}
