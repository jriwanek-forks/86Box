/*
 * 86Box     A hypervisor and IBM PC system emulator that specializes in
 *           running old operating systems and software designed for IBM
 *           PC systems and compatibles from 1981 through fairly recent
 *           system designs based on the PCI bus.
 *
 *           This file is part of the 86Box distribution.
 *
 *           CL-PD67xx Emulation.
 *
 * Authors:  Cacodemon345
 *
 *           Copyright 2024-2025 Cacodemon345.
 */
/* TODO: Dual-socket PD6722. */
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/io.h>
#include <86box/timer.h>
#include <86box/pcmcia.h>
#include <86box/mem.h>
#include <86box/pic.h>
#include <86box/device.h>

#include "cpu.h"

struct pcmcia_socket_pd67xx;

/* We will make the mappings use it as a pointer to avoid cluttering up code. */
typedef struct pd67xx_memory_map {
    struct pcmcia_socket_pd67xx *main_ptr;
    uint8_t                      map_num;
    mem_mapping_t                mapping;
    union {
        uint16_t addr;
        uint8_t  addr_b[2];
    } start __attribute__((packed));
    union {
        uint16_t addr;
        uint8_t  addr_b[2];
    } end __attribute__((packed));
    union {
        uint16_t addr;
        uint8_t  addr_b[2];
    } offset __attribute__((packed));
} pd67xx_memory_map;

typedef struct pcmcia_socket_pd67xx {
    uint8_t index;

    union {
        uint8_t interface_status;
        struct {
            uint8_t bvd       : 2;
            uint8_t cd        : 2;
            uint8_t wp        : 1;
            uint8_t ready     : 1;
            uint8_t power_on  : 1;
            uint8_t vpp_valid : 1;
        };
    };
    uint8_t power_control;
    uint8_t interrupt_general_control;
    uint8_t card_status;
    uint8_t management_interrupt_conf;
    uint8_t mapping_enable;
    uint8_t io_window_control;

    uint8_t misc_control_1;
    uint8_t misc_control_2;
    uint8_t fifo_control;
    uint8_t ata_control;

    io_range_t *ranges[2];
    uint16_t    io_offsets[2];

    pd67xx_memory_map mem_maps[5];

    uint8_t regs[0x40]; /* Only used for ignored registers. */

    bool    inserted;
    bool    status;
    bool    read_seq;
    uint8_t irq_state, mgmt_irq_state;

    pcmcia_socket_t socket;
} pcmcia_socket_pd67xx;

#ifdef ENABLE_PD6710_LOG
int pd6710_do_log = ENABLE_PD6710_LOG;

static void
pd6710_log(const char *fmt, ...)
{
    va_list ap;

    if (pd6710_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define pd6710_log(fmt, ...)
#endif

void pd67xx_power_recalc(pcmcia_socket_pd67xx* pd67xx);

void
pd67xx_mem_recalc(pd67xx_memory_map *mem_map)
{
    mem_mapping_disable(&mem_map->mapping);
    if (mem_map->main_ptr->mapping_enable & (1 << mem_map->map_num)) {
        uint32_t start = (mem_map->start.addr & 0x3fff) << 12;
        uint32_t end   = (mem_map->end.addr & 0x3fff) << 12;

        if (start < (16 << 12))
            start = (16 << 12);

        if (end < start)
            return;

        mem_mapping_set_addr(&mem_map->mapping, start, (end - start) + 4096);
        if (mem_map->main_ptr->socket.mem_get_exec)
            mem_mapping_set_exec(&mem_map->mapping, mem_map->main_ptr->socket.mem_get_exec(start + ((mem_map->offset.addr & 0x3fff) << 12), mem_map->main_ptr->socket.card_priv));
        else
            mem_mapping_set_exec(&mem_map->mapping, NULL);
    }
}

void
pd67xx_mem_recalc_all(pcmcia_socket_pd67xx *pd67xx)
{
    for (uint8_t i = 0; i < 5; i++)
        pd67xx_mem_recalc(&pd67xx->mem_maps[i]);
}

void
pd67xx_mgmt_interrupt(pcmcia_socket_pd67xx *pd67xx, int set)
{
    bool level = !!(pd67xx->misc_control_1 & (1 << 3));
    if ((pd67xx->interrupt_general_control & (1 << 4)) && set) {
        /* Assume that we are asserting I/O Channel Check low. */
        nmi_raise();
        return;
    }

    if (level && !!((pd67xx->management_interrupt_conf >> 4) & 0xF)) {
        if (set)
            picintlevel(1 << (pd67xx->management_interrupt_conf >> 4), &pd67xx->mgmt_irq_state);
        else
            picintclevel(1 << (pd67xx->management_interrupt_conf >> 4), &pd67xx->mgmt_irq_state);
    }

    if (set && !!((pd67xx->management_interrupt_conf >> 4) & 0xF))
        picint(1 << (pd67xx->management_interrupt_conf >> 4));
    else if (!!((pd67xx->management_interrupt_conf >> 4) & 0xF))
        picintc(1 << (pd67xx->management_interrupt_conf >> 4));
}

void
pd67xx_card_interrupt(bool set, bool level, pcmcia_socket_t *socket)
{
    pcmcia_socket_pd67xx *pd67xx = socket->socket_priv;
    if (pd67xx->misc_control_1 & (1 << 3))
        level = false;
    if (set && !!(pd67xx->interrupt_general_control & 0xF)) {
        if (level)
            picintlevel(1 << (pd67xx->interrupt_general_control), &pd67xx->irq_state);
        else
            picint(1 << (pd67xx->interrupt_general_control));
    } else if (!set && level)
        picintclevel(1 << (pd67xx->interrupt_general_control), &pd67xx->irq_state);
    else
        picintc(1 << (pd67xx->interrupt_general_control));
}

void
pd67xx_card_inserted(bool inserted, pcmcia_socket_t *socket)
{
    pcmcia_socket_pd67xx *pd67xx        = socket->socket_priv;
    bool                  signal_change = false;

    if (pd67xx->inserted ^ inserted)
        signal_change = true;

    pd67xx->inserted = inserted;
    pd67xx->cd       = inserted ? 0b11 : 0b00;

    if (signal_change && (pd67xx->management_interrupt_conf & 8)) {
        pd67xx->card_status |= 0x8;
        pd67xx_mgmt_interrupt(pd67xx, 1);
    }
    pd67xx_power_recalc(pd67xx);
}

void
pd67xx_status_changed(bool status, pcmcia_socket_t *socket)
{
    pcmcia_socket_pd67xx *pd67xx        = socket->socket_priv;
    bool                  signal_change = false;

    if (pd67xx->status ^ status)
        signal_change = true;

    pd67xx->status = status;
    pd67xx->bvd &= ~1;
    pd67xx->bvd |= status ? 0b1 : 0b0;

    if (signal_change && (pd67xx->management_interrupt_conf & 1)) {
        pd67xx->card_status |= 0x1;
        pd67xx_mgmt_interrupt(pd67xx, 1);
    }
}

void
pd67xx_ready_changed(bool status, pcmcia_socket_t *socket)
{
    pcmcia_socket_pd67xx *pd67xx        = socket->socket_priv;
    bool                  signal_change = false;

    if (pd67xx->interrupt_general_control & (1 << 5)) {
        pd67xx_card_interrupt(!status, true, socket);
        return;
    }

    if (pd67xx->ready ^ status)
        signal_change = true;

    pd67xx->ready = status;

    if (signal_change && (pd67xx->management_interrupt_conf & 4)) {
        pd67xx->card_status |= 0x4;
        pd67xx_mgmt_interrupt(pd67xx, 1);
    }
}

uint8_t
pd67xx_mem_read(uint32_t addr, void *priv)
{
    pd67xx_memory_map    *pd67xx_mem_map = (pd67xx_memory_map *) priv;
    pcmcia_socket_pd67xx *pd67xx         = pd67xx_mem_map->main_ptr;

    if (!pd67xx->socket.card_priv)
        return 0xFF;

    if (!pd67xx->socket.mem_read)
        return 0xFF;

    /* No outputs enabled. */
    if (!(pd67xx->power_control & (1 << 7)))
        return 0xFF;

    addr += (pd67xx_mem_map->offset.addr & 0x3fff) * 4096;
    addr &= 0x3FFFFFF;
    pd6710_log("PD67XX: Read from memory 0x%X (REG=%d)\n", addr, !(pd67xx_mem_map->offset.addr & (1 << 14)));
    return pd67xx->socket.mem_read(addr, !(pd67xx_mem_map->offset.addr & (1 << 14)), pd67xx->socket.card_priv);
}

void
pd67xx_mem_write(uint32_t addr, uint8_t val, void *priv)
{
    pd67xx_memory_map    *pd67xx_mem_map = (pd67xx_memory_map *) priv;
    pcmcia_socket_pd67xx *pd67xx         = pd67xx_mem_map->main_ptr;

    if (!pd67xx->socket.card_priv)
        return;

    if (!pd67xx->socket.mem_write)
        return;

    /* write-protected? */
    if (pd67xx_mem_map->offset.addr & 0x8000)
        return;

    /* No outputs enabled. */
    if (!(pd67xx->power_control & (1 << 7)))
        return;

    addr += (pd67xx_mem_map->offset.addr & 0x3fff) * 4096;
    addr &= 0x3FFFFFF;
    pd6710_log("PD67XX: Write 0x%02X to memory 0x%X (REG=%d)\n", val, addr, !(pd67xx_mem_map->offset.addr & (1 << 14)));
    return pd67xx->socket.mem_write(addr, val, !(pd67xx_mem_map->offset.addr & (1 << 14)), pd67xx->socket.card_priv);
}

uint16_t
pd67xx_mem_readw(uint32_t addr, void *priv)
{
    pd67xx_memory_map    *pd67xx_mem_map = (pd67xx_memory_map *) priv;
    pcmcia_socket_pd67xx *pd67xx         = pd67xx_mem_map->main_ptr;

    if (!pd67xx->socket.card_priv)
        return 0xFF;

    if (!pd67xx->socket.mem_readw)
        return 0xFF;

    /* No outputs enabled. */
    if (!(pd67xx->power_control & (1 << 7)))
        return 0xFFFF;

    addr += (pd67xx_mem_map->offset.addr & 0x3fff) * 4096;
    addr &= 0x3FFFFFF;
    pd6710_log("PD67XX: Read word from memory 0x%X (REG=%d)\n", addr, !(pd67xx_mem_map->offset.addr & (1 << 14)));
    return pd67xx->socket.mem_readw(addr, !(pd67xx_mem_map->offset.addr & (1 << 14)), pd67xx->socket.card_priv);
}

void
pd67xx_mem_writew(uint32_t addr, uint16_t val, void *priv)
{
    pd67xx_memory_map    *pd67xx_mem_map = (pd67xx_memory_map *) priv;
    pcmcia_socket_pd67xx *pd67xx         = pd67xx_mem_map->main_ptr;

    if (!pd67xx->socket.card_priv)
        return;

    if (!pd67xx->socket.mem_writew)
        return;

    /* write-protected? */
    if (pd67xx_mem_map->offset.addr & 0x8000)
        return;

    /* No outputs enabled. */
    if (!(pd67xx->power_control & (1 << 7)))
        return;

    addr += (pd67xx_mem_map->offset.addr & 0x3fff) * 4096;
    addr &= 0x3FFFFFF;
    pd6710_log("PD67XX: Write word 0x%04X to memory 0x%X (REG=%d)\n", val, addr, !(pd67xx_mem_map->offset.addr & (1 << 14)));
    return pd67xx->socket.mem_writew(addr, val, !(pd67xx_mem_map->offset.addr & (1 << 14)), pd67xx->socket.card_priv);
}

uint8_t
pd67xx_io_read_1(uint16_t port, void *priv)
{
    pcmcia_socket_pd67xx *pd67xx = priv;

    if (!pd67xx->socket.card_priv)
        return 0xFF;

    if (!pd67xx->socket.io_read)
        return 0xFF;

    /* No outputs enabled. */
    if (!(pd67xx->power_control & (1 << 7)))
        return 0xFF;

    port += pd67xx->io_offsets[0];
    return pd67xx->socket.io_read(port, pd67xx->socket.card_priv);
}

uint16_t
pd67xx_io_readw_1(uint16_t port, void *priv)
{
    pcmcia_socket_pd67xx *pd67xx = priv;

    if (!pd67xx->socket.card_priv)
        return 0xFF;

    if (!pd67xx->socket.io_read)
        return 0xFF;

    /* No outputs enabled. */
    if (!(pd67xx->power_control & (1 << 7)))
        return 0xFFFF;

    port += pd67xx->io_offsets[0];
    return pd67xx->socket.io_readw(port, pd67xx->socket.card_priv);
}

void
pd67xx_io_write_1(uint16_t port, uint8_t val, void *priv)
{
    pcmcia_socket_pd67xx *pd67xx = priv;

    if (!pd67xx->socket.card_priv)
        return;

    if (!pd67xx->socket.io_write)
        return;

    /* No outputs enabled. */
    if (!(pd67xx->power_control & (1 << 7)))
        return;

    port += pd67xx->io_offsets[0];
    return pd67xx->socket.io_write(port, val, pd67xx->socket.card_priv);
}

void
pd67xx_io_writew_1(uint16_t port, uint16_t val, void *priv)
{
    pcmcia_socket_pd67xx *pd67xx = priv;

    if (!pd67xx->socket.card_priv)
        return;

    if (!pd67xx->socket.io_writew)
        return;

    /* No outputs enabled. */
    if (!(pd67xx->power_control & (1 << 7)))
        return;

    port += pd67xx->io_offsets[0];
    return pd67xx->socket.io_write(port, val, pd67xx->socket.card_priv);
}

uint8_t
pd67xx_io_read_2(uint16_t port, void *priv)
{
    pcmcia_socket_pd67xx *pd67xx = priv;

    if (!pd67xx->socket.card_priv)
        return 0xFF;

    if (!pd67xx->socket.io_read)
        return 0xFF;

    /* No outputs enabled. */
    if (!(pd67xx->power_control & (1 << 7)))
        return 0xFF;

    port += pd67xx->io_offsets[1];
    return pd67xx->socket.io_read(port, pd67xx->socket.card_priv);
}

uint16_t
pd67xx_io_readw_2(uint16_t port, void *priv)
{
    pcmcia_socket_pd67xx *pd67xx = priv;

    if (!pd67xx->socket.card_priv)
        return 0xFF;

    if (!pd67xx->socket.io_read)
        return 0xFF;

    /* No outputs enabled. */
    if (!(pd67xx->power_control & (1 << 7)))
        return 0xFFFF;

    port += pd67xx->io_offsets[1];
    return pd67xx->socket.io_readw(port, pd67xx->socket.card_priv);
}

void
pd67xx_io_write_2(uint16_t port, uint8_t val, void *priv)
{
    pcmcia_socket_pd67xx *pd67xx = priv;

    if (!pd67xx->socket.card_priv)
        return;

    if (!pd67xx->socket.io_write)
        return;

    /* No outputs enabled. */
    if (!(pd67xx->power_control & (1 << 7)))
        return;

    port += pd67xx->io_offsets[1];
    return pd67xx->socket.io_write(port, val, pd67xx->socket.card_priv);
}

void
pd67xx_io_writew_2(uint16_t port, uint16_t val, void *priv)
{
    pcmcia_socket_pd67xx *pd67xx = priv;

    if (!pd67xx->socket.card_priv)
        return;

    if (!pd67xx->socket.io_writew)
        return;

    /* No outputs enabled. */
    if (!(pd67xx->power_control & (1 << 7)))
        return;

    port += pd67xx->io_offsets[1];
    return pd67xx->socket.io_write(port, val, pd67xx->socket.card_priv);
}

void
pd67xx_power_recalc(pcmcia_socket_pd67xx* pd67xx)
{
    if (pd67xx->socket.card_priv && (pd67xx->power_control & (1 << 4)) && (!(pd67xx->power_control & (1 << 5)) || ((pd67xx->power_control & (1 << 5)) && pd67xx->inserted)))
        pd67xx->interface_status |= (1 << 6);
    else
        pd67xx->interface_status &= ~(1 << 6);

    if (pd67xx->socket.card_priv && pd67xx->socket.power_status_change) {
        if (pd67xx->interface_status & (1 << 6)) {
            int vpp = 5;
            int vcc = (pd67xx->misc_control_1 & (1 << 1)) ? 3 : 5;

            switch (pd67xx->power_control & 3) {
                case 1:
                    vpp = 12;
                    break;
                case 2:
                    vpp = vcc;
                    break;
            }
            pd67xx->socket.power_status_change(vcc, vpp, &pd67xx->socket);
        } else if (pd67xx->socket.card_priv)
            pd67xx->socket.power_status_change(0, 0, &pd67xx->socket);
    }
}

void
pd67xx_port_write(uint16_t port, uint8_t val, void *priv)
{
    pcmcia_socket_pd67xx *pd67xx = priv;

    pd6710_log("PCIC write port 0x%04X (val 0x%04X)\n", port, val);

    if (!(port & 1))
        pd67xx->index = val;
    else {
        switch (pd67xx->index) {
            case 0x02:
                {
                    pd67xx->power_control = val;
                    pd67xx_power_recalc(pd67xx);
                    break;
                }
            case 0x03:
                {
                    bool reset                        = !(val & (1 << 6)) && (pd67xx->interrupt_general_control & (1 << 6));
                    pd67xx->interrupt_general_control = val;
                    if ((pd67xx->power_control & (1 << 7)) && reset) {
                        pd67xx->socket.reset(pd67xx->socket.card_priv);
                    }
                    break;
                }
            case 0x05:
                {
                    pd67xx->management_interrupt_conf = val;
                    break;
                }
            case 0x06:
                {
                    pd67xx->mapping_enable = val;
                    pd6710_log("PCIC Memory Enable: 0x%X\n", val);
                    pd67xx_mem_recalc_all(pd67xx);
                    pd67xx->ranges[0]->enable = !!(val & (1 << 6));
                    pd67xx->ranges[1]->enable = !!(val & (1 << 7));
                    break;
                }
            case 0x07:
                {
                    pd67xx->io_window_control = val;
                    break;
                }
            case 0x08:
                {
                    pd67xx->ranges[0]->start = (pd67xx->ranges[0]->start & 0xFF00) | val;
                    break;
                }
            case 0x09:
                {
                    pd67xx->ranges[0]->start = (pd67xx->ranges[0]->start & 0xFF) | (val << 8);
                    break;
                }
            case 0x0a:
                {
                    pd67xx->ranges[0]->end = (pd67xx->ranges[0]->start & 0xFF00) | val;
                    break;
                }
            case 0x0b:
                {
                    pd67xx->ranges[0]->end = (pd67xx->ranges[0]->start & 0xFF) | (val << 8);
                    break;
                }
            case 0x0c:
                {
                    pd67xx->ranges[1]->start = (pd67xx->ranges[1]->start & 0xFF00) | val;
                    break;
                }
            case 0x0d:
                {
                    pd67xx->ranges[1]->start = (pd67xx->ranges[1]->start & 0xFF) | (val << 8);
                    break;
                }
            case 0x0e:
                {
                    pd67xx->ranges[1]->end = (pd67xx->ranges[1]->start & 0xFF00) | val;
                    break;
                }
            case 0x0f:
                {
                    pd67xx->ranges[1]->end = (pd67xx->ranges[1]->start & 0xFF) | (val << 8);
                    break;
                }
            case 0x10 ... 0x15:
            case 0x18 ... 0x1D:
            case 0x20 ... 0x25:
            case 0x28 ... 0x2D:
            case 0x30 ... 0x35:
                {
                    uint8_t            mem_map_num = ((pd67xx->index - 0x10) >> 3);
                    pd67xx_memory_map *mem_map     = &pd67xx->mem_maps[mem_map_num];

                    switch (pd67xx->index & 0xF) {
                        case 0:
                            {
                                mem_map->start.addr_b[0] = val;
                                break;
                            }
                        case 1:
                            {
                                mem_map->start.addr_b[1] = val;
                                break;
                            }
                        case 2:
                            {
                                mem_map->end.addr_b[0] = val;
                                break;
                            }
                        case 3:
                            {
                                mem_map->end.addr_b[1] = val;
                                break;
                            }
                        case 4:
                            {
                                mem_map->offset.addr_b[0] = val;
                                break;
                            }
                        case 5:
                            {
                                mem_map->offset.addr_b[1] = val;
                                break;
                            }
                    }
                    pd67xx_mem_recalc(mem_map);
                    break;
                }

            case 0x36:
                {
                    pd67xx->io_offsets[0] = (pd67xx->io_offsets[0] & 0xFF00) | val;
                    break;
                }
            case 0x37:
                {
                    pd67xx->io_offsets[0] = (pd67xx->io_offsets[0] & 0xFF) | (val << 8);
                    break;
                }
            case 0x38:
                {
                    pd67xx->io_offsets[1] = (pd67xx->io_offsets[1] & 0xFF00) | val;
                    break;
                }
            case 0x39:
                {
                    pd67xx->io_offsets[1] = (pd67xx->io_offsets[1] & 0xFF) | (val << 8);
                    break;
                }
            case 0x3A ... 0x3F: /* Timing registers. */
            case 0x27:          /* Scratchpad */
                pd67xx->regs[pd67xx->index] = val;
                break;

            case 0x16:
                pd67xx->misc_control_1 = val;
                break;

            case 0x17:
                pd67xx->fifo_control = val & 0x7f;
                break;

            case 0x1E:
                pd67xx->misc_control_2 = val & ~(1 << 6); /* Bit 6 is reserved. */
                break;

            case 0x1F:
                pd67xx->read_seq = 0;
                break;
            
            case 0x26:
                {
                    bool changed = (val ^ pd67xx->ata_control) & 1;
                    pd67xx->ata_control = val;
                    if (changed && pd67xx->socket.ata_mode && pd67xx->socket.card_priv) {
                        pd67xx->socket.ata_mode(changed, &pd67xx->socket);
                    }
                    break;
                }
        }
    }
}

uint8_t
pd67xx_port_read(uint16_t port, void *priv)
{
    pcmcia_socket_pd67xx *pd67xx = priv;

    pd6710_log("PCIC read port 0x%04X\n", port);

    if (!(port & 1))
        return pd67xx->index;
    else {
        switch (pd67xx->index) {
            case 0x00:
                return 0b10000010;
            case 0x01: {
                uint8_t val = (pd67xx->interface_status & (1 << 6)) | (pd67xx->inserted ? 0b11101111 : 0);
                return val;
            }
            case 0x02:
                return pd67xx->power_control;
            case 0x03:
                return pd67xx->interrupt_general_control;
            case 0x04:
                {
                    uint8_t ret         = pd67xx->card_status;
                    pd67xx->card_status = 0;
                    pd67xx_mgmt_interrupt(pd67xx, 0);
                    return ret;
                }
            case 0x05:
                return pd67xx->management_interrupt_conf;
            case 0x06:
                return pd67xx->mapping_enable;
            case 0x07:
                {
                    return pd67xx->io_window_control;
                }
            case 0x08:
                {
                    return pd67xx->ranges[0]->start & 0xFF;
                }
            case 0x09:
                {
                    return (pd67xx->ranges[0]->start >> 8) & 0xFF;
                }
            case 0x0a:
                {
                    return pd67xx->ranges[0]->end & 0xFF;
                }
            case 0x0b:
                {
                    return (pd67xx->ranges[0]->end >> 8) & 0xFF;
                }
            case 0x0c:
                {
                    return pd67xx->ranges[1]->start & 0xFF;
                }
            case 0x0d:
                {
                    return (pd67xx->ranges[1]->start >> 8) & 0xFF;
                }
            case 0x0e:
                {
                    return pd67xx->ranges[1]->end & 0xFF;
                }
            case 0x0f:
                {
                    return (pd67xx->ranges[1]->end >> 8) & 0xFF;
                }

            case 0x10 ... 0x15:
            case 0x18 ... 0x1D:
            case 0x20 ... 0x25:
            case 0x28 ... 0x2D:
            case 0x30 ... 0x35:
                {
                    uint8_t            mem_map_num = ((pd67xx->index - 0x10) >> 3);
                    pd67xx_memory_map *mem_map     = &pd67xx->mem_maps[mem_map_num];

                    switch (pd67xx->index & 0xF) {
                        case 0:
                            {
                                return mem_map->start.addr_b[0];
                            }
                        case 1:
                            {
                                return mem_map->start.addr_b[1];
                            }
                        case 2:
                            {
                                return mem_map->end.addr_b[0];
                            }
                        case 3:
                            {
                                return mem_map->end.addr_b[1];
                            }
                        case 4:
                            {
                                return mem_map->offset.addr_b[0];
                            }
                        case 5:
                            {
                                return mem_map->offset.addr_b[1];
                            }
                    }
                    break;
                }

            case 0x36:
            case 0x37:
                return (pd67xx->io_offsets[0] >> (8 * (pd67xx->index & 1))) & 0xFF;
            case 0x38:
            case 0x39:
                return (pd67xx->io_offsets[1] >> (8 * (pd67xx->index & 1))) & 0xFF;

            /* Timing registers, not so relevant here right now. */
            case 0x3A ... 0x3F:
            case 0x27: /* Scratchpad. */
                return pd67xx->regs[pd67xx->index];

            /* All cards are 5 volts for now. */
            case 0x16:
                return pd67xx->misc_control_1 | 1;

            case 0x17:
                return pd67xx->fifo_control;

            case 0x1E:
                return pd67xx->misc_control_2;

            case 0x1F:
                {
                    uint8_t ret = 0b01110;
                    if (pd67xx->read_seq == false)
                        ret |= 0xC0;
                    pd67xx->read_seq ^= 1;
                    return ret;
                }
            
            case 0x26:
                return pd67xx->ata_control;

            default:
                return 0xFF;
        }
    }
    return 0xFF;
}

void
pd67xx_reset(void *priv)
{
    pcmcia_socket_pd67xx *pd67xx = priv;

    pd67xx_card_interrupt(false, false, &pd67xx->socket);
    pd67xx_card_interrupt(false, true, &pd67xx->socket);
    pd67xx_mgmt_interrupt(pd67xx, false);

    pd67xx->mapping_enable = 0;
    for (uint8_t i = 0; i < 5; i++) {
        pd67xx->mem_maps[i].start.addr = pd67xx->mem_maps[i].end.addr = pd67xx->mem_maps[i].offset.addr = 0;
        pd67xx->mem_maps[i].map_num                                                                     = i;
    }
    pd67xx_mem_recalc_all(pd67xx);

    pd67xx->power_control             = 0;
    pd67xx->misc_control_1            = 0;
    pd67xx->misc_control_2            = 0;
    pd67xx->fifo_control              = 0;
    pd67xx->interrupt_general_control = 0;
    pd67xx->management_interrupt_conf = 0;
    pd67xx->io_window_control         = 0;
    pd67xx->power_control             = 0xF0;

    memset(pd67xx->regs, 0, sizeof(pd67xx->regs));

    pd67xx->io_offsets[0] = pd67xx->io_offsets[1] = 0x0000;
    pd67xx->ranges[0]->enable                     = 0;
    pd67xx->ranges[1]->enable                     = 0;
    pd67xx->ranges[0]->start                      = 0;
    pd67xx->ranges[0]->end                        = 0;
    pd67xx->ranges[1]->start                      = 0;
    pd67xx->ranges[1]->end                        = 0;

    if (pd67xx->socket.reset && pd67xx->socket.card_priv)
        pd67xx->socket.reset(pd67xx->socket.card_priv);
}

void *
pd6710_init(const device_t *info)
{
    pcmcia_socket_pd67xx *pd67xx = calloc(1, sizeof(pcmcia_socket_pd67xx));

    pd67xx->ranges[0] = io_range_addhandler(0, 0, pd67xx_io_read_1, pd67xx_io_readw_1, NULL, pd67xx_io_write_1, pd67xx_io_writew_1, NULL, 0, pd67xx);
    pd67xx->ranges[1] = io_range_addhandler(0, 0, pd67xx_io_read_2, pd67xx_io_readw_2, NULL, pd67xx_io_write_2, pd67xx_io_writew_2, NULL, 0, pd67xx);

    for (uint8_t i = 0; i < 5; i++) {
        pd67xx->mem_maps[i].map_num  = i;
        pd67xx->mem_maps[i].main_ptr = pd67xx;
        mem_mapping_add(&pd67xx->mem_maps[i].mapping, 0, 0, pd67xx_mem_read, pd67xx_mem_readw, NULL, pd67xx_mem_write, pd67xx_mem_writew, NULL, NULL, MEM_MAPPING_EXTERNAL, &pd67xx->mem_maps[i]);
    }

    pd67xx->socket.socket_num     = 0;
    pd67xx->socket.powered_on     = 0;
    pd67xx->socket.interrupt      = pd67xx_card_interrupt;
    pd67xx->socket.card_inserted  = pd67xx_card_inserted;
    pd67xx->socket.ready_changed  = pd67xx_ready_changed;
    pd67xx->socket.status_changed = pd67xx_status_changed;
    pd67xx->socket.socket_priv    = pd67xx;

    io_sethandler((info->local == 1) ? 0x3e2 : 0x3e0, 2, pd67xx_port_read, NULL, NULL, pd67xx_port_write, NULL, NULL, pd67xx);

    pcmcia_register_socket(&pd67xx->socket);

    pd67xx_reset(pd67xx);

    return pd67xx;
}

void
pd6710_close(void *priv)
{
    free(priv);
}

const device_t pd6710_device = {
    .name          = "Cirrus Logic CL-PD6710",
    .internal_name = "pd6710",
    .flags         = DEVICE_ISA16,
    .local         = 0,
    .init          = pd6710_init,
    .close         = pd6710_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const device_t pd6710_alt_device = {
    .name          = "Cirrus Logic CL-PD6710 (Alternate I/O Address)",
    .internal_name = "pd6710_alt",
    .flags         = DEVICE_ISA16,
    .local         = 1,
    .init          = pd6710_init,
    .close         = pd6710_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
