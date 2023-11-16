/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Universal Serial Bus emulation (currently dummy OHCI).
 *
 *
 *
 * Authors: Miran Grca, <mgrca8@gmail.com>
 *
 *          Copyright 2020 Miran Grca.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/fifo8.h>
#include <86box/timer.h>
#include <86box/usb.h>
#include "cpu.h"
#include <86box/plat_unused.h>

#ifdef ENABLE_USB_LOG
int usb_do_log = ENABLE_USB_LOG;

static void
usb_log(const char *fmt, ...)
{
    va_list ap;

    if (usb_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define usb_log(fmt, ...)
#endif

static usb_t* usb_device_inst[128];
static uint8_t usb_device_inst_cnt = 0;

extern uint8_t uhci_reg_read(uint16_t addr, void *priv);
extern uint16_t uhci_reg_readw(uint16_t addr, void *priv);
extern void uhci_reg_write(uint16_t addr, uint8_t val, void *priv);
extern void uhci_reg_writew(uint16_t addr, uint16_t val, void *priv);
extern void uhci_reset(usb_t *dev);
extern void uhci_frame_timer(void *opaque);
extern void uhci_detach(UHCIState *s, int index);

void
uhci_update_io_mapping(usb_t *dev, uint8_t base_l, uint8_t base_h, int enable)
{
    if (dev->uhci_enable && (dev->uhci_io_base != 0x0000))
        io_removehandler(dev->uhci_io_base, 0x20, uhci_reg_read, uhci_reg_readw, NULL, uhci_reg_write, uhci_reg_writew, NULL, dev);

    dev->uhci_io_base = base_l | (base_h << 8);
    dev->uhci_enable  = enable;

    if (dev->uhci_enable && (dev->uhci_io_base != 0x0000))
        io_sethandler(dev->uhci_io_base, 0x20, uhci_reg_read, uhci_reg_readw, NULL, uhci_reg_write, uhci_reg_writew, NULL, dev);
}

static uint8_t
ohci_mmio_read(uint32_t addr, void *priv)
{
    const usb_t  *dev = (usb_t *) priv;
    uint8_t       ret = 0x00;

    addr &= 0x00000fff;

    ret = dev->ohci_mmio[addr];

    if (addr == 0x101)
        ret = (ret & 0xfe) | (!!mem_a20_key);

    return ret;
}

static void
ohci_mmio_write(uint32_t addr, uint8_t val, void *priv)
{
    usb_t  *dev = (usb_t *) priv;
    uint8_t old;

    addr &= 0x00000fff;

    switch (addr) {
        case 0x04:
            if ((val & 0xc0) == 0x00) {
                /* UsbReset */
                dev->ohci_mmio[0x56] = dev->ohci_mmio[0x5a] = 0x16;
            }
            break;
        case 0x08: /* HCCOMMANDSTATUS */
            /* bit OwnershipChangeRequest triggers an ownership change (SMM <-> OS) */
            if (val & 0x08) {
                dev->ohci_mmio[0x0f] = 0x40;
                if ((dev->ohci_mmio[0x13] & 0xc0) == 0xc0)
                    smi_raise();
            }

            /* bit HostControllerReset must be cleared for the controller to be seen as initialized */
            if (val & 0x01) {
                memset(dev->ohci_mmio, 0x00, 4096);
                dev->ohci_mmio[0x00] = 0x10;
                dev->ohci_mmio[0x01] = 0x01;
                dev->ohci_mmio[0x48] = 0x02;
                val &= ~0x01;
            }
            break;
        case 0x0c:
            dev->ohci_mmio[addr] &= ~(val & 0x7f);
            return;
        case 0x0d:
        case 0x0e:
            return;
        case 0x0f:
            dev->ohci_mmio[addr] &= ~(val & 0x40);
            return;
        case 0x3b:
            dev->ohci_mmio[addr] = (val & 0x80);
            return;
        case 0x39:
        case 0x41:
            dev->ohci_mmio[addr] = (val & 0x3f);
            return;
        case 0x45:
            dev->ohci_mmio[addr] = (val & 0x0f);
            return;
        case 0x3a:
        case 0x3e:
        case 0x3f:
        case 0x42:
        case 0x43:
        case 0x46:
        case 0x47:
        case 0x48:
        case 0x4a:
            return;
        case 0x49:
            dev->ohci_mmio[addr] = (val & 0x1b);
            if (val & 0x02) {
                dev->ohci_mmio[0x55] |= 0x01;
                dev->ohci_mmio[0x59] |= 0x01;
            }
            return;
        case 0x4b:
            dev->ohci_mmio[addr] = (val & 0x03);
            return;
        case 0x4c:
        case 0x4e:
            dev->ohci_mmio[addr] = (val & 0x06);
            if ((addr == 0x4c) && !(val & 0x04)) {
                if (!(dev->ohci_mmio[0x58] & 0x01))
                    dev->ohci_mmio[0x5a] |= 0x01;
                dev->ohci_mmio[0x58] |= 0x01;
            }
            if ((addr == 0x4c) && !(val & 0x02)) {
                if (!(dev->ohci_mmio[0x54] & 0x01))
                    dev->ohci_mmio[0x56] |= 0x01;
                dev->ohci_mmio[0x54] |= 0x01;
            }
            return;
        case 0x4d:
        case 0x4f:
            return;
        case 0x50:
            if (val & 0x01) {
                if ((dev->ohci_mmio[0x49] & 0x03) == 0x00) {
                    dev->ohci_mmio[0x55] &= ~0x01;
                    dev->ohci_mmio[0x54] &= ~0x17;
                    dev->ohci_mmio[0x56] &= ~0x17;
                    dev->ohci_mmio[0x59] &= ~0x01;
                    dev->ohci_mmio[0x58] &= ~0x17;
                    dev->ohci_mmio[0x5a] &= ~0x17;
                } else if ((dev->ohci_mmio[0x49] & 0x03) == 0x01) {
                    if (!(dev->ohci_mmio[0x4e] & 0x02)) {
                        dev->ohci_mmio[0x55] &= ~0x01;
                        dev->ohci_mmio[0x54] &= ~0x17;
                        dev->ohci_mmio[0x56] &= ~0x17;
                    }
                    if (!(dev->ohci_mmio[0x4e] & 0x04)) {
                        dev->ohci_mmio[0x59] &= ~0x01;
                        dev->ohci_mmio[0x58] &= ~0x17;
                        dev->ohci_mmio[0x5a] &= ~0x17;
                    }
                }
            }
            return;
        case 0x51:
            if (val & 0x80)
                dev->ohci_mmio[addr] |= 0x80;
            return;
        case 0x52:
            dev->ohci_mmio[addr] &= ~(val & 0x02);
            if (val & 0x01) {
                if ((dev->ohci_mmio[0x49] & 0x03) == 0x00) {
                    dev->ohci_mmio[0x55] |= 0x01;
                    dev->ohci_mmio[0x59] |= 0x01;
                } else if ((dev->ohci_mmio[0x49] & 0x03) == 0x01) {
                    if (!(dev->ohci_mmio[0x4e] & 0x02))
                        dev->ohci_mmio[0x55] |= 0x01;
                    if (!(dev->ohci_mmio[0x4e] & 0x04))
                        dev->ohci_mmio[0x59] |= 0x01;
                }
            }
            return;
        case 0x53:
            if (val & 0x80)
                dev->ohci_mmio[0x51] &= ~0x80;
            return;
        case 0x54:
        case 0x58:
            old = dev->ohci_mmio[addr];

            if (val & 0x10) {
                if (old & 0x01) {
                    dev->ohci_mmio[addr] |= 0x10;
                    /* TODO: The clear should be on a 10 ms timer. */
                    dev->ohci_mmio[addr] &= ~0x10;
                    dev->ohci_mmio[addr + 2] |= 0x10;
                } else
                    dev->ohci_mmio[addr + 2] |= 0x01;
            }
            if (val & 0x08)
                dev->ohci_mmio[addr] &= ~0x04;
            if (val & 0x04)
                dev->ohci_mmio[addr] |= 0x04;
            if (val & 0x02) {
                if (old & 0x01)
                    dev->ohci_mmio[addr] |= 0x02;
                else
                    dev->ohci_mmio[addr + 2] |= 0x01;
            }
            if (val & 0x01) {
                if (old & 0x01)
                    dev->ohci_mmio[addr] &= ~0x02;
                else
                    dev->ohci_mmio[addr + 2] |= 0x01;
            }

            if (!(dev->ohci_mmio[addr] & 0x04) && (old & 0x04))
                dev->ohci_mmio[addr + 2] |= 0x04;
#if 0
            if (!(dev->ohci_mmio[addr] & 0x02))
                 dev->ohci_mmio[addr + 2] |= 0x02;
#endif
            return;
        case 0x55:
            if ((val & 0x02) && ((dev->ohci_mmio[0x49] & 0x03) == 0x00) && (dev->ohci_mmio[0x4e] & 0x02)) {
                dev->ohci_mmio[addr] &= ~0x01;
                dev->ohci_mmio[0x54] &= ~0x17;
                dev->ohci_mmio[0x56] &= ~0x17;
            }
            if ((val & 0x01) && ((dev->ohci_mmio[0x49] & 0x03) == 0x00) && (dev->ohci_mmio[0x4e] & 0x02)) {
                dev->ohci_mmio[addr] |= 0x01;
                dev->ohci_mmio[0x58] &= ~0x17;
                dev->ohci_mmio[0x5a] &= ~0x17;
            }
            return;
        case 0x59:
            if ((val & 0x02) && ((dev->ohci_mmio[0x49] & 0x03) == 0x00) && (dev->ohci_mmio[0x4e] & 0x04))
                dev->ohci_mmio[addr] &= ~0x01;
            if ((val & 0x01) && ((dev->ohci_mmio[0x49] & 0x03) == 0x00) && (dev->ohci_mmio[0x4e] & 0x04))
                dev->ohci_mmio[addr] |= 0x01;
            return;
        case 0x56:
        case 0x5a:
            dev->ohci_mmio[addr] &= ~(val & 0x1f);
            return;
        case 0x57:
        case 0x5b:
            return;

        default:
            break;
    }

    dev->ohci_mmio[addr] = val;
}

void
ohci_update_mem_mapping(usb_t *dev, uint8_t base1, uint8_t base2, uint8_t base3, int enable)
{
    if (dev->ohci_enable && (dev->ohci_mem_base != 0x00000000))
        mem_mapping_disable(&dev->ohci_mmio_mapping);

    dev->ohci_mem_base = ((base1 << 8) | (base2 << 16) | (base3 << 24)) & 0xfffff000;
    dev->ohci_enable   = enable;

    if (dev->ohci_enable && (dev->ohci_mem_base != 0x00000000))
        mem_mapping_set_addr(&dev->ohci_mmio_mapping, dev->ohci_mem_base, 0x1000);
}

uint8_t
usb_parse_control_endpoint(usb_device_t* usb_device, uint8_t* data, uint32_t *len, uint8_t pid_token, uint8_t endpoint, uint8_t underrun_not_allowed)
{
/* FIXME: This needs to be redone. */
#if 0
    usb_desc_setup_t* setup_packet = (usb_desc_setup_t*)data;
    uint8_t ret = USB_ERROR_STALL;
    
    if (endpoint != 0)
        return USB_ERROR_STALL;

    if (!usb_device->fifo.data) {
        fifo8_create(&usb_device->fifo, 4096);
    }
    //pclog("Control endpoint of device 0x%08X: Transfer (PID = 0x%X, len = %d)\n", usb_device, pid_token, *len);
    switch (pid_token) {
        case USB_PID_SETUP:
        {
            if (*len != 8) {
                return *len > 8 ? USB_ERROR_OVERRUN : USB_ERROR_UNDERRUN;
            }
            usb_device->setup_desc = *setup_packet;
            if (setup_packet->bmRequestType & 0x80) fifo8_reset(&usb_device->fifo);
            //pclog("bmRequestType = 0x%X, \n", usb_device->setup_desc.bmRequestType);
            //pclog("bRequest = 0x%X, \n", usb_device->setup_desc.bRequest);
            //pclog("wIndex = 0x%X, \n", usb_device->setup_desc.wIndex);
            //pclog("wLength = 0x%X, \n", usb_device->setup_desc.wLength);
            //pclog("wValue = 0x%X\n", usb_device->setup_desc.wValue);
            switch (setup_packet->bmRequestType & 0x1f) {
                case USB_SETUP_TYPE_INTERFACE:
                {
                    if (setup_packet->bmRequestType & 0x80) {
                        switch (setup_packet->bRequest) {
                            case USB_SETUP_GET_STATUS: {
                                ret = 0;
                                fifo8_push(&usb_device->fifo, 0x00);
                                fifo8_push(&usb_device->fifo, 0x00);
                                break;
                            }
                            case USB_SETUP_GET_INTERFACE: {
                                ret = 0;
                                fifo8_push(&usb_device->fifo, usb_device->interface_altsetting[setup_packet->wIndex]);
                                break;
                            }
                        }
                    } else {
                        switch (setup_packet->bRequest) {
                            case USB_SETUP_SET_FEATURE:
                            case USB_SETUP_CLEAR_FEATURE: {
                                ret = 0;
                                break;
                            }
                            case USB_SETUP_SET_INTERFACE: {
                                ret = 0;
                                usb_device->interface_altsetting[setup_packet->wIndex] = setup_packet->wValue;
                                break;
                            }
                        }
                    }
                    break;
                }
                case USB_SETUP_TYPE_ENDPOINT:
                    /* Consider it all handled. */
                    if (setup_packet->bRequest == USB_SETUP_GET_STATUS && (setup_packet->bmRequestType & 0x80)) {
                        fifo8_push(&usb_device->fifo, 0x00);
                        fifo8_push(&usb_device->fifo, 0x00);
                    }
                    return 0;
                case USB_SETUP_TYPE_DEVICE:
                    {
                        if (setup_packet->bmRequestType & 0x80) {
                            switch (setup_packet->bRequest) {
                                case USB_SETUP_GET_STATUS: {
                                    fifo8_push_all(&usb_device->fifo, (uint8_t*)&usb_device->status_bits, sizeof(uint16_t));
                                    ret = 0;
                                    break;
                                }
                                case USB_SETUP_GET_CONFIGURATION: {
                                    fifo8_push(&usb_device->fifo, usb_device->current_configuration);
                                    ret = 0;
                                    break;
                                }
                                case USB_SETUP_GET_DESCRIPTOR: {
                                    switch (setup_packet->wValue >> 8) {
                                        case 0x01: /* Device descriptor */
                                        {
                                            fifo8_push_all(&usb_device->fifo, (uint8_t*)&usb_device->device_desc, sizeof(usb_device->device_desc));
                                            ret = 0;
                                            break;
                                        }
                                        case 0x02: /* Configuration descriptor (with all associated descriptors) */
                                        {
                                            int i = 0;
                                            ret = 0;
                                            fifo8_push_all(&usb_device->fifo, (uint8_t*)&usb_device->conf_desc_items.conf_desc, usb_device->conf_desc_items.conf_desc.base.bLength);
                                            for (i = 0; i < 16; i++) {
                                                if (usb_device->conf_desc_items.other_descs[i] == NULL)
                                                    break;

                                                if (usb_device->conf_desc_items.other_descs[i]->bDescriptorType == 0xFF) {
                                                    fifo8_push_all(&usb_device->fifo, ((usb_desc_ptr_t*)usb_device->conf_desc_items.other_descs[i])->ptr, usb_device->conf_desc_items.other_descs[i]->bLength);
                                                }
                                                else
                                                    fifo8_push_all(&usb_device->fifo, (uint8_t*)usb_device->conf_desc_items.other_descs[i], usb_device->conf_desc_items.other_descs[i]->bLength);
                                            }
                                            break;
                                        }
                                        case 0x03: /* String descriptor */
                                        {
                                            ret = 0;
                                            if (!usb_device->string_desc[setup_packet->wValue & 0xff]) {
                                                return USB_ERROR_STALL;
                                            }
                                            fifo8_push_all(&usb_device->fifo, (uint8_t*)usb_device->string_desc[setup_packet->wValue & 0xff], usb_device->string_desc[setup_packet->wValue & 0xff]->base.bLength);
                                            break;
                                        }
                                        default:
                                            return USB_ERROR_STALL;
                                    }
                                    break;
                                }
                            }
                        } else {
                            switch (setup_packet->bRequest) {
                                case USB_SETUP_SET_FEATURE: {
                                    usb_device->status_bits |= (setup_packet->wValue & 0x1);
                                    ret = 0;
                                    break;
                                }
                                case USB_SETUP_CLEAR_FEATURE: {
                                    usb_device->status_bits &= ~(setup_packet->wValue & 0x1);
                                    ret = 0;
                                    break;
                                }
                                case USB_SETUP_SET_ADDRESS: {
                                    usb_device->address = setup_packet->wValue & 0xFF;
                                    pclog("Device address @ 0x%llX set to %d\n", (intptr_t)usb_device, usb_device->address);
                                    ret = 0;
                                    break;
                                }
                                case USB_SETUP_SET_CONFIGURATION: {
                                    usb_device->current_configuration = setup_packet->wValue & 0xFF;
                                    ret = 0;
                                    break;
                                }
                            }
                        }
                        break;
                    }
            }
            break;
        }
        case USB_PID_IN: {
            const uint8_t* buf = NULL;
            uint32_t used = 0;
            if (!(usb_device->setup_desc.bmRequestType & 0x80)) {
                if ((usb_device->setup_desc.wLength && fifo8_num_used(&usb_device->fifo) >= usb_device->setup_desc.wLength) || !usb_device->setup_desc.wLength) {
                    uint32_t len = 8;
                    return usb_device->device_process(usb_device->priv, (uint8_t*)&usb_device->setup_desc, &len, USB_PID_SETUP, 0, underrun_not_allowed);
                }
                else if (*len == 0)
                    return USB_ERROR_NO_ERROR;
                else
                    return USB_ERROR_STALL;
            }
            if (fifo8_num_used(&usb_device->fifo) == 0) {
                *len = 0;
                return USB_ERROR_NO_ERROR;
            }
            buf = fifo8_pop_buf(&usb_device->fifo, *len, len);
            memcpy(data, buf, *len);
            ret = 0;
            break;
        }
        case USB_PID_OUT: {
            if (!(usb_device->setup_desc.bmRequestType & 0x80)) {
                if (*len)
                    fifo8_push_all(&usb_device->fifo, data, *len);
                return 0;
            }
            if ((usb_device->setup_desc.bmRequestType & 0x80) && *len != 0)
                return USB_ERROR_STALL;
            return 0;
        }
    }
    return ret;
#else
    return USB_ERROR_STALL;
#endif
}

static void
usb_reset(void *priv)
{
    usb_t *dev = (usb_t *) priv;

    uhci_reset(dev);

    memset(dev->ohci_mmio, 0x00, 4096);
    dev->ohci_mmio[0x00] = 0x10;
    dev->ohci_mmio[0x01] = 0x01;
    dev->ohci_mmio[0x48] = 0x02;

    io_removehandler(dev->uhci_io_base, 0x20, uhci_reg_read, uhci_reg_readw, NULL, uhci_reg_write, uhci_reg_writew, NULL, dev);
    dev->uhci_enable = 0;

    mem_mapping_disable(&dev->ohci_mmio_mapping);
    dev->ohci_enable = 0;
}

static void
usb_close(void *priv)
{
    usb_t *dev = (usb_t *) priv;

    usb_device_inst[dev->inst_cnt] = NULL;
    usb_device_inst_cnt--;
    free(dev);
}

extern void uhci_attach(UHCIState *s, usb_device_t* device, int index);

/* FIXME: This needs a rewrite (with the detach counterpart) to support more than 2 ports. */
uint16_t usb_attach_device(usb_device_t* device, uint8_t bus_type)
{
    if (!usb_device_inst[0])
        return (uint16_t)-1;

    switch (bus_type) {
        case USB_BUS_UHCI:
            {
                uint8_t i = 0;
                for (i = 0; i < 2; i++)
                {
                    if (!usb_device_inst[0]->uhci_state.ports[i].dev)
                    {
                        uhci_attach(&usb_device_inst[0]->uhci_state, device, i);
                        return (i) | (bus_type) << 8;
                    }
                }
                break;
            }
        default:
            return (uint16_t)-1;
    }
    return (uint16_t)-1;
}

void
usb_detach_device(uint16_t port)
{
    if (!usb_device_inst[0])
        return;
    
    switch (port >> 8) {
        case USB_BUS_UHCI:
            {
                uhci_detach(&usb_device_inst[0]->uhci_state, port & 0xFF);
                break;
            }
        default:
            return;
    }
    return;
}

static void *
usb_init(UNUSED(const device_t *info), void* param)
{
    usb_t *dev;

    dev = (usb_t *) malloc(sizeof(usb_t));
    if (dev == NULL)
        return (NULL);
    memset(dev, 0x00, sizeof(usb_t));

    memset(dev->ohci_mmio, 0x00, 4096);
    dev->ohci_mmio[0x00] = 0x10;
    dev->ohci_mmio[0x01] = 0x01;
    dev->ohci_mmio[0x48] = 0x02;

    mem_mapping_add(&dev->ohci_mmio_mapping, 0, 0,
                    ohci_mmio_read, NULL, NULL,
                    ohci_mmio_write, NULL, NULL,
                    NULL, MEM_MAPPING_EXTERNAL, dev);

    timer_add(&dev->uhci_state.frame_timer, uhci_frame_timer, (void*)&dev->uhci_state, 0);

    if (param)
    {
        dev->params = *(usb_params_t*)param;
    }
    dev->uhci_state.dev = dev;
    usb_reset(dev);
    //uhci_attach(&dev->uhci_state, &dummy, 0);
    usb_device_inst[usb_device_inst_cnt] = dev;
    dev->inst_cnt = usb_device_inst_cnt;
    usb_device_inst_cnt++;

    return dev;
}

const device_t usb_device = {
    .name          = "Universal Serial Bus",
    .internal_name = "usb",
    .flags         = DEVICE_PCI | DEVICE_EXTPARAMS,
    .local         = 0,
    .init_ext      = usb_init,
    .close         = usb_close,
    .reset         = usb_reset,
    { .available = NULL },
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
