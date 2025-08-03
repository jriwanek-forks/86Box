/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Covox Sound Master emulation.
 *
 * Authors: Roy Baer, <https://pcem-emulator.co.uk/>
 *          Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025 Roy Baer.
 *          Copyright 2025 Jasmine Iwanek.
 */
#ifndef _SOUND_CSM_H_
#define _SOUND_CSM_H_

typedef struct ay_3_89x0_s {
    int     type;
    int     last_written;
    uint8_t index;
    uint8_t regs[16];
    struct  ayumi chip;
} ay_3_89x0_t;

typedef struct csm_s {
    ay_3_89x0_t psg;
    uint8_t     pcm_sample;

    int16_t buffer[SOUNDBUFLEN * 10];
    int     pos;
} csm_t;

#endif /* _SOUND_CSM_H_ */
