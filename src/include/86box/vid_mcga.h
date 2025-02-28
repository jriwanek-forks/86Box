/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Emulation of the old and new IBM CGA graphics cards.
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          Miran Grca, <mgrca8@gmail.com>
 *          Cacodemon345
 *
 *          Copyright 2008-2018 Sarah Walker.
 *          Copyright 2016-2018 Miran Grca.
 *          Copyright 2022-2025 Cacodemon345.
 */

#ifndef VIDEO_MCGA_H
#define VIDEO_MCGA_H

#include <stdint.h>
#include <86box/mem.h>
#include <86box/timer.h>
typedef struct mcga_t {
    mem_mapping_t mapping;

    int     crtcreg;
    uint8_t crtc[32];

    uint8_t cgastat;

    uint8_t cgamode;
    uint8_t cgacol;

    int      fontbase;
    int      linepos;
    int      displine;
    int      sc;
    int      vc;
    int      cgadispon;
    int      con;
    int      coff;
    int      cursoron;
    int      cgablink;
    int      vsynctime;
    int      vadj;
    uint16_t ma;
    uint16_t maback;
    int      oddeven;

    uint64_t   dispontime;
    uint64_t   dispofftime;
    pc_timer_t timer;

    int firstline;
    int lastline;

    int drawcursor;

    int fullchange;

    uint8_t *vram;

    uint8_t charbuffer[256];
    uint8_t pal[256 * 3];
    uint8_t mcga_extmode;
    uint8_t read_index;
    uint8_t write_index;
    uint8_t last_cycle;

    int revision;
    int composite;
    int snow_enabled;
    int rgb_type;
    int read_times;
    int write_times;
} mcga_t;

void    mcga_init(mcga_t *mcga);
void    mcga_out(uint16_t addr, uint8_t val, void *priv);
uint8_t mcga_in(uint16_t addr, void *priv);
void    mcga_write(uint32_t addr, uint8_t val, void *priv);
uint8_t mcga_read(uint32_t addr, void *priv);
void    mcga_recalctimings(mcga_t *mcga);
void    mcga_poll(void *priv);

#ifdef EMU_DEVICE_H
extern const device_config_t mcga_config[];

extern const device_t mcga_device;
#endif

#endif	/*VIDEO_MCGA_H*/
