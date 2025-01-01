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

static irda_device_t* devices[256];
static uint8_t irda_count;

void irda_reset(void)
{
    memset(devices, 0, sizeof(devices));
    irda_count = 0;
}

void irda_register_device(irda_device_t* irda_device)
{
    devices[irda_count++] = irda_device;
}

void irda_broadcast_data(irda_device_t* broadcaster, uint8_t data)
{
    int i = 0;
    pclog("IrDA broadcast data: 0x%X\n", data);
    for (i = 0; i < irda_count; i++) {
        if (devices[i] != broadcaster && devices[i]->write)
            devices[i]->write(devices[i]->priv, data);
    }
}