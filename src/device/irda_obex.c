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
 *           Only IrLAP discovery is implemented for now: the device answers
 *           XID commands over IrDA SIR framing, so that it shows up to the
 *           guest, but it does not accept connections yet.
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
#include <86box/log.h>
#include <86box/plat.h>
#include <86box/irda.h>
#include <86box/random.h>

/* SIR framing. */
#define SIR_XBOF  0xff /* Extra beginning of frame */
#define SIR_BOF   0xc0 /* Beginning of frame */
#define SIR_EOF   0xc1 /* End of frame */
#define SIR_CE    0x7d /* Control escape */
#define SIR_TRANS 0x20 /* Transparency modifier */
#define SIR_XBOFS 10

#define FCS_INIT  0xffff
#define FCS_GOOD  0xf0b8
#define FCS_POLY  0x8408
#define FCS_SIZE  2

/* IrLAP frames. */
#define IRLAP_CBROADCAST 0xfe /* Broadcast connection address */
#define IRLAP_CMD_FRAME  0x01 /* Command/response bit of the address */
#define IRLAP_XID_CMD    0x2f
#define IRLAP_XID_RSP    0xaf
#define IRLAP_PF_BIT     0x10 /* Poll/final bit */
#define IRLAP_XID_FORMAT 0x01
#define IRLAP_BROADCAST  0xffffffff

/* Discovery information. */
#define HINT_COMPUTER  0x04
#define HINT_EXTENSION 0x80
#define HINT_OBEX      0x20
#define CHARSET_ASCII  0x00

#pragma pack(push, 1)
typedef struct irda_xid_frame_t {
    uint8_t  caddr;
    uint8_t  control;
    uint8_t  format;
    uint32_t source;
    uint32_t dest;
    uint8_t  flags;
    uint8_t  slot;
    uint8_t  version;
} irda_xid_frame_t;
#pragma pack(pop)

typedef struct irda_obex_t {
    irda_device_t irda;
    void         *log;

    uint32_t data_count;
    uint8_t  data[0x10000];
    uint8_t  in_frame;
    uint8_t  complement_next;

    uint8_t  connection_address;
    uint32_t address;
    uint32_t address_dest;
} irda_obex_t;

#ifdef ENABLE_IRDA_OBEX_LOG
int irda_obex_do_log = ENABLE_IRDA_OBEX_LOG;

static void
irda_obex_log(void *priv, const char *fmt, ...)
{
    va_list ap;

    if (irda_obex_do_log) {
        va_start(ap, fmt);
        log_out(priv, fmt, ap);
        va_end(ap);
    }
}
#else
#    define irda_obex_log(priv, fmt, ...)
#endif

static uint16_t
irda_obex_fcs(uint16_t fcs, uint8_t data)
{
    fcs ^= data;

    for (uint8_t i = 0; i < 8; i++)
        fcs = (fcs & 1) ? ((fcs >> 1) ^ FCS_POLY) : (fcs >> 1);

    return fcs;
}

static void
irda_obex_send_byte(irda_obex_t *irda_obex, uint8_t data)
{
    if ((data == SIR_BOF) || (data == SIR_EOF) || (data == SIR_CE)) {
        irda_broadcast_data(&irda_obex->irda, SIR_CE);
        data ^= SIR_TRANS;
    }

    irda_broadcast_data(&irda_obex->irda, data);
}

static void
irda_obex_send_frame(irda_obex_t *irda_obex, const uint8_t *frame, uint32_t len)
{
    uint16_t fcs = FCS_INIT;

    for (uint8_t i = 0; i < SIR_XBOFS; i++)
        irda_broadcast_data(&irda_obex->irda, SIR_XBOF);

    irda_broadcast_data(&irda_obex->irda, SIR_BOF);

    for (uint32_t i = 0; i < len; i++) {
        fcs = irda_obex_fcs(fcs, frame[i]);
        irda_obex_send_byte(irda_obex, frame[i]);
    }

    /* The FCS is sent complemented, low byte first. */
    fcs = ~fcs;
    irda_obex_send_byte(irda_obex, fcs & 0xff);
    irda_obex_send_byte(irda_obex, fcs >> 8);

    irda_broadcast_data(&irda_obex->irda, SIR_EOF);
}

static void
irda_obex_process_xid(irda_obex_t *irda_obex, const uint8_t *frame, uint32_t len)
{
    static const uint8_t info[] = { HINT_COMPUTER | HINT_EXTENSION, HINT_OBEX, CHARSET_ASCII,
                                    'I', 'r', 'D', 'A', ' ', 'O', 'B', 'E', 'X' };
    irda_xid_frame_t     xid;
    irda_xid_frame_t     resp_xid = { 0 };
    uint8_t              resp[sizeof(irda_xid_frame_t) + sizeof(info)];

    if (len < sizeof(xid))
        return;

    memcpy(&xid, frame, sizeof(xid));

    if ((xid.format != IRLAP_XID_FORMAT) || !(xid.control & IRLAP_PF_BIT))
        return;

    /* Any slot would do, so always take the first one. */
    if ((xid.slot != 0) || ((xid.dest != IRLAP_BROADCAST) && (xid.dest != irda_obex->address)))
        return;

    irda_obex->address_dest = xid.source;

    resp_xid.caddr   = IRLAP_CBROADCAST;
    resp_xid.control = IRLAP_XID_RSP | IRLAP_PF_BIT;
    resp_xid.format  = IRLAP_XID_FORMAT;
    resp_xid.source  = irda_obex->address;
    resp_xid.dest    = irda_obex->address_dest;
    resp_xid.flags   = xid.flags;
    resp_xid.slot    = xid.slot;

    memcpy(resp, &resp_xid, sizeof(resp_xid));
    memcpy(&resp[sizeof(resp_xid)], info, sizeof(info));

    irda_obex_send_frame(irda_obex, resp, sizeof(resp));
}

static void
irda_obex_process_frame(irda_obex_t *irda_obex)
{
    uint16_t fcs = FCS_INIT;
    uint32_t len;

    irda_obex_log(irda_obex->log, "Received frame of %u bytes\n", irda_obex->data_count);

    /* The address, the control field and the FCS are always there. */
    if (irda_obex->data_count < (2 + FCS_SIZE))
        return;

    for (uint32_t i = 0; i < irda_obex->data_count; i++)
        fcs = irda_obex_fcs(fcs, irda_obex->data[i]);

    if (fcs != FCS_GOOD) {
        irda_obex_log(irda_obex->log, "Discarding frame with bad FCS\n");
        return;
    }

    len = irda_obex->data_count - FCS_SIZE;

    irda_obex_log(irda_obex->log, "Connection address: %02X, control: %02X\n",
                  irda_obex->data[0], irda_obex->data[1]);

    if (((irda_obex->data[0] & IRLAP_CBROADCAST) != IRLAP_CBROADCAST) &&
        ((irda_obex->data[0] & IRLAP_CBROADCAST) != (irda_obex->connection_address & IRLAP_CBROADCAST)))
        return;

    /* Only commands are of interest in the normal disconnected mode. */
    if (((irda_obex->connection_address & IRLAP_CBROADCAST) == IRLAP_CBROADCAST) &&
        (irda_obex->data[0] & IRLAP_CMD_FRAME) &&
        ((irda_obex->data[1] & ~IRLAP_PF_BIT) == IRLAP_XID_CMD))
        irda_obex_process_xid(irda_obex, irda_obex->data, len);
}

static void
irda_obex_receive(void *priv, uint8_t data)
{
    irda_obex_t *irda_obex = (irda_obex_t *) priv;

    switch (data) {
        case SIR_BOF:
            /* This may be one of several, or abandon an unfinished frame. */
            irda_obex->in_frame        = 1;
            irda_obex->complement_next = 0;
            irda_obex->data_count      = 0;
            return;

        case SIR_EOF:
            /* A control escape right before the end aborts the frame. */
            if (irda_obex->in_frame && !irda_obex->complement_next)
                irda_obex_process_frame(irda_obex);

            irda_obex->in_frame        = 0;
            irda_obex->complement_next = 0;
            irda_obex->data_count      = 0;
            return;

        case SIR_CE:
            irda_obex->complement_next = irda_obex->in_frame;
            return;

        default:
            break;
    }

    if (!irda_obex->in_frame)
        return;

    if (irda_obex->complement_next) {
        data ^= SIR_TRANS;
        irda_obex->complement_next = 0;
    }

    /* Too long to be a frame, wait for the next one. */
    if (irda_obex->data_count >= sizeof(irda_obex->data)) {
        irda_obex->in_frame   = 0;
        irda_obex->data_count = 0;
        return;
    }

    irda_obex->data[irda_obex->data_count++] = data;
}

static void
irda_obex_reset(void *priv)
{
    irda_obex_t *irda_obex = (irda_obex_t *) priv;

    irda_obex->in_frame           = 0;
    irda_obex->complement_next    = 0;
    irda_obex->data_count         = 0;
    irda_obex->address_dest       = 0;
    irda_obex->connection_address = IRLAP_CBROADCAST;
}

static void *
irda_obex_init(UNUSED(const device_t *info))
{
    irda_obex_t *irda_obex = (irda_obex_t *) calloc(1, sizeof(irda_obex_t));

    irda_obex->irda.write = irda_obex_receive;
    irda_obex->irda.priv  = irda_obex;
    irda_obex->log        = log_open("IrDA OBEX");
    irda_register_device(&irda_obex->irda);

    irda_obex_reset(irda_obex);

    /* The device address must not be mistaken for the broadcast or null address. */
    do {
        irda_obex->address = (uint32_t) random_generate() | ((uint32_t) random_generate() << 8) |
                             ((uint32_t) random_generate() << 16) | ((uint32_t) random_generate() << 24);
    } while ((irda_obex->address == 0) || (irda_obex->address == IRLAP_BROADCAST));

    return irda_obex;
}

static void
irda_obex_close(void *priv)
{
    irda_obex_t *irda_obex = (irda_obex_t *) priv;

    irda_unregister_device(&irda_obex->irda);

    log_close(irda_obex->log);
    free(irda_obex);
}

const device_t irda_obex_device = {
    .name          = "IrDA OBEX device",
    .internal_name = "irda_obex",
    .flags         = DEVICE_COM | DEVICE_HOTPLUG, /* Comes and goes with its dongle. */
    .local         = 0,
    .init          = irda_obex_init,
    .close         = irda_obex_close,
    .reset         = irda_obex_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
