/*
 * 86Box     A hypervisor and IBM PC system emulator that specializes in
 *           running old operating systems and software designed for IBM
 *           PC systems and compatibles from 1981 through fairly recent
 *           system designs based on the PCI bus.
 *
 *           This file is part of the 86Box distribution.
 *
 *           Definitions for the IrDA medium shared by infrared devices.
 *
 * Authors:  Cacodemon345
 *
 *           Copyright 2024-2026 Cacodemon345.
 */
#ifndef EMU_IRDA_H
#define EMU_IRDA_H

/* Function definition. */
typedef void irda_write(void *priv, uint8_t data);

typedef struct irda_device_t {
    uint8_t  pri;      /* Primary or secondary device. */
    uint8_t  conn;     /* Physical number of connection. */
    uint32_t src_addr; /* Logical address of source station. */
    uint32_t dst_addr; /* Logical address of destination station. */

    irda_write *write; /* Write/receive function. */
    void       *priv;  /* Opaque pointer. */
} irda_device_t;

extern const device_t esi9680_device;
extern const device_t irda_obex_device;

extern void irda_register_device(irda_device_t *irda_device);
extern void irda_unregister_device(irda_device_t *irda_device);
extern void irda_broadcast_data(irda_device_t *broadcaster, uint8_t data);
extern void irda_reset(void);

#endif /*EMU_IRDA_H*/
