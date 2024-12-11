/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Quadram Quadcolor I / I+II emulation
 *
 *
 *
 * Authors:  Benedikt Freisen, <https://pcem-emulator.co.uk/>
 *
 *          Copyright 2024      Benedikt Freisen.
 */

#ifndef _VID_QUADCOLOR_H_
#define _VID_QUADCOLOR_H_

typedef struct quadcolor_t {
    mem_mapping_t mapping;
    mem_mapping_t mapping_2;

    int     crtcreg;
    uint8_t crtc[32];

    uint8_t cgastat;

    uint8_t cgamode;
	uint8_t cgacol;

    uint8_t  quadcolor_ctrl;
    uint8_t  quadcolor_2_oe;
    uint16_t page_offset;

    int fontbase;
    int linepos;
	int displine;
    int sc;
	int vc;
    int cgadispon;
    int con;
	int coff;
    int	cursoron;
	int cgablink;
    int vsynctime;
	int vadj;
    uint16_t ma;
	uint16_t maback;
    int oddeven;
    int qc2idx;
    uint8_t qc2mask;

    uint64_t dispontime;
	uint64_t dispofftime;
    pc_timer_t timer;

    int firstline;
	int lastline;

    int drawcursor;

    uint8_t *vram;
    uint8_t *vram_2;

    uint8_t charbuffer[256];

    int revision;
    int composite;
    int has_2nd_charset;
    int has_quadcolor_2;
} quadcolor_t;

void quadcolor_init(quadcolor_t *quadcolor);
void quadcolor_out(uint16_t addr, uint8_t val, void *priv);
uint8_t quadcolor_in(uint16_t addr, void *priv);
void quadcolor_write(uint32_t addr, uint8_t val, void *priv);
uint8_t quadcolor_read(uint32_t addr, void *priv);
void quadcolor_recalctimings(quadcolor_t *quadcolor);
void quadcolor_poll(void *priv);

#endif /* _VID_QUADCOLOR_H_ */
