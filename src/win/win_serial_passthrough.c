/*
 * 86Box        A hypervisor and IBM PC system emulator that specializes in
 *              running old operating systems and software designed for IBM
 *              PC systems and compatibles from 1981 through fairly recent
 *              system designs based on the PCI bus.
 *
 *              This file is part of the 86Box distribution.
 *
 *              Definitions for platform specific serial to host passthrough
 *
 *
 *
 * Authors:     Andreas J. Reichel <webmaster@6th-dimension.com>
 *
 *              Copyright 2021          Andreas J. Reichel
 */

#define _XOPEN_SOURCE 500
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <86box/86box.h>
#include <86box/log.h>
#include <86box/timer.h>
#include <86box/plat.h>
#include <86box/device.h>
#include <86box/serial_passthrough.h>
#include <86box/plat_serial_passthrough.h>


#define LOG_PREFIX "serial_passthrough: "


int
plat_serpt_read(void *p, uint8_t *data)
{
        serial_passthrough_t *dev = (serial_passthrough_t *)p;
        int res;

        switch (dev->mode) {
        case SERPT_MODE_VCON:
                break;
        default:
                break;
        }
        return 0;
}


void
plat_serpt_close(void *p)
{
        serial_passthrough_t *dev = (serial_passthrough_t *)p;

        close(dev->master_fd);
}


static void
plat_serpt_write_vcon(serial_passthrough_t *dev, uint8_t data)
{
        /* fd_set wrfds;
         * int res;
         */

        /* We cannot use select here, this would block the hypervisor! */
        /* FD_ZERO(&wrfds);
           FD_SET(ctx->master_fd, &wrfds);
        
           res = select(ctx->master_fd + 1, NULL, &wrfds, NULL, NULL);
        
           if (res <= 0) {
                return;
           }
        */

        /* just write it out */
        write(dev->master_fd, &data, 1);
}


void
plat_serpt_write(void *p, uint8_t data)
{
        serial_passthrough_t *dev = (serial_passthrough_t *)p;
        
        switch (dev->mode) {
        case SERPT_MODE_VCON:
                plat_serpt_write_vcon(dev, data);
                break;
        default:
                break;
        }
}


static int
open_pseudo_terminal(serial_passthrough_t *dev)
{
    return 0;
}


int
plat_serpt_open_device(void *p)
{
        serial_passthrough_t *dev = (serial_passthrough_t *)p;

        switch (dev->mode) {
        case SERPT_MODE_VCON:
                if (!open_pseudo_terminal(dev)) {
                        return 1;
                }
                break;
        default:
                break;

        }
        return 0;
}
