/*
 * 86Box     A hypervisor and IBM PC system emulator that specializes in
 *           running old operating systems and software designed for IBM
 *           PC systems and compatibles from 1981 through fairly recent
 *           system designs based on the PCI bus.
 *
 *           This file is part of the 86Box distribution.
 *
 *           IrDA OBEX emulation.
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
#include "cpu.h"
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/serial.h>
#include <86box/plat.h>
#include <86box/fifo8.h>
#include <86box/irda.h>
#include <86box/random.h>

typedef struct irda_obex_t
{
    irda_device_t irda;

    uint32_t data_count;
    uint8_t data[0x10000];
    uint8_t resp_data[0x10000];
    uint8_t in_frame;
    uint8_t complement_next;

    uint8_t connection_address;
    uint32_t address, address_dest;
    uint8_t slot;
} irda_obex_t;

static void irda_obex_process_args(irda_obex_t* irda_obex, uint8_t* parameters)
{
    while (1) {
        switch (*parameters) {
            case 0x00:
            case 0xc1:
            case 0xc0:
            default:
                return;
            case 0x01:
            {
                uint8_t len = *(parameters + 1);
                parameters++;
                if (len) {
                    /* We ignore baud rate here. */
                    parameters += len;
                }
                break;
            }
        }
    }
}

void irda_obex_process_frame(irda_obex_t* irda_obex)
{
    pclog("Received frame of %u bytes\n", irda_obex->data_count);
    if (irda_obex->data_count == 0)
        return;
    pclog("Connection address: 0x%x\n", irda_obex->data[0]);
    pclog("Connection control: 0x%X\n", irda_obex->data[1]);

    if ((irda_obex->data[0] & 0xFE) != 0xFE && (irda_obex->data[0] & 0xFE) != (irda_obex->connection_address & 0xFE))
        return;

    if ((irda_obex->connection_address & 0xFE) == 0xFE && (irda_obex->data[1] & 3) == 3) {
        switch (irda_obex->data[1] & ~(0x10)) {
            case 0b0010111: /* XID */
            {
                irda_xid_info_t* xid = (irda_xid_info_t*)&irda_obex[2];
                if (xid->slot == 0 && (irda_obex->data[1] & 0x10) && (xid->dest == ~0 || xid->dest == irda_obex->address)) {
                    int i = 0;
                    irda_xid_info_t* resp_xid = (irda_xid_info_t*)&irda_obex->resp_data[2];
                    irda_obex->resp_data[0] = 0xFE;
                    irda_obex->resp_data[1] = 0b10111111;
                    irda_obex->address_dest = xid->source;
                    irda_obex->address = random_generate() | (random_generate() << 8) | (random_generate() << 16) | (random_generate() << 24);
                    memset(resp_xid, 0, sizeof(*resp_xid));
                    resp_xid->format = 1;
                    resp_xid->source = irda_obex->address;
                    resp_xid->dest = irda_obex->address_dest;
                    resp_xid->version = 0;
                    resp_xid->flags = xid->flags;
                    resp_xid->discovery_info[0] = 0x4;
                    resp_xid->discovery_info[1] = 0x20;
                    resp_xid->discovery_info[2] = 'I';
                    resp_xid->discovery_info[3] = 'r';
                    resp_xid->discovery_info[4] = 'D';
                    resp_xid->discovery_info[5] = 'A';
                    resp_xid->discovery_info[6] = ' ';
                    resp_xid->discovery_info[7] = 'O';
                    resp_xid->discovery_info[8] = 'B';
                    resp_xid->discovery_info[9] = 'E';
                    resp_xid->discovery_info[10] = 'X';
                    
                    for (i = 0; i < 11; i++) {
                        irda_broadcast_data(&irda_obex->irda, 0xC0);
                    }
                    for (i = 0; i < sizeof(*resp_xid) + 9 + 2; i++) {
                        irda_broadcast_data(&irda_obex->irda, irda_obex->resp_data[i]);
                    }

                    irda_broadcast_data(&irda_obex->irda, 0xC1);
                }
                break;
            }
        }
    }
}

void irda_obex_receive(void *priv, uint8_t data)
{
    irda_obex_t* irda_obex = (irda_obex_t*)priv;

    if (data == 0xFF && irda_obex->in_frame == 0)
        return;

    if (data == 0x7e && irda_obex->in_frame == 0) {
        irda_obex->in_frame = 2;
        return;
    }
    if (data == 0x7e && irda_obex->in_frame == 2) {
        irda_obex_process_frame(irda_obex);
        irda_obex->data_count = 0;
        irda_obex->in_frame = 0;
        return;
    }
    if (data == 0xc0 && irda_obex->in_frame != 2) {
        irda_obex->in_frame = 1;
        return;
    }
    if (data == 0xc1 && irda_obex->in_frame == 1) {
        irda_obex_process_frame(irda_obex);
        irda_obex->data_count = 0;
        irda_obex->in_frame = 0;
        return;
    }
    if (irda_obex->in_frame == 1 && data == 0x7d) {
        irda_obex->complement_next = 1;
        return;
    }
    if (irda_obex->in_frame == 1) {
        if (irda_obex->complement_next) {
            if (data == 0xc1) {
                /* Discard this frame. */
                irda_obex->data_count = 0;
                return;
            } else {
                irda_obex->data[irda_obex->data_count++] = data ^ (1 << 5);
                return;
            }
        } else {
            irda_obex->data[irda_obex->data_count++] = data;
            return;
        }
    } else if (irda_obex->in_frame == 2) {
        irda_obex->data[irda_obex->data_count++] = data;
        return;
    }
}

void* irda_obex_init(const device_t* info)
{
    irda_obex_t* irda_obex = (irda_obex_t*)calloc(1, sizeof(irda_obex_t));
    irda_obex->irda.write = irda_obex_receive;
    irda_obex->irda.priv = irda_obex;
    irda_register_device(&irda_obex->irda);
    irda_obex->connection_address = 0xFE;

    return irda_obex;
}

void irda_obex_close(void* priv)
{
    free(priv);
}

const device_t irda_obex_device = {
    .name          = "IrDA OBEX device",
    .internal_name = "irda_obex",
    .flags         = 0,
    .local         = 0,
    .init          = irda_obex_init,
    .close         = irda_obex_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};