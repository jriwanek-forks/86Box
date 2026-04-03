/*
 * 86Box        A hypervisor and IBM PC system emulator that specializes in
 *              running old operating systems and software designed for IBM
 *              PC systems and compatibles from 1981 through fairly recent
 *              system designs based on the PCI bus.
 *
 *              This file is part of the 86Box distribution.
 *
 *              Definition of serial port loopback devices.
 *
 * Authors:     Jasmine Iwanek <jasmine@iwanek.co.uk>
 *
 *              Copyright 2025-2026 Jasmine Iwanek.
 */
#ifndef SERIAL_LOOPBACK_H
#define SERIAL_LOOPBACK_H

extern bool           serial_loopback_enabled[SERIAL_MAX - 1];
extern const device_t serial_loopback_device;

extern void serial_loopback_init(void);

#endif /*SERIAL_LOOPBACK_H*/
