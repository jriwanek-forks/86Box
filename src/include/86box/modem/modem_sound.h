/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and IBM PC systems and compatibles.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Modem speaker audio emulation interface.
 *
 * Authors: Xeon3D (original creator)
 *          Jasmine Iwanek (86Box port)
 *
 *          Copyright 2026 Xeon3D.
 *          Copyright 2026 Jasmine Iwanek.
 */
#ifndef EMU_MODEM_SOUND_H
#define EMU_MODEM_SOUND_H

#include <stdint.h>

#define MODEM_SOUND_DIAL_CONNECT_AFTER (1 << 9)

typedef struct modem_sound_t modem_sound_t;

enum {
    MODEM_SOUND_REGION_NORTH_AMERICA = 0,
    MODEM_SOUND_REGION_UK,
    MODEM_SOUND_REGION_EUROPE,
    MODEM_SOUND_REGION_JAPAN,
    MODEM_SOUND_REGION_COUNT
};

enum {
    MODEM_SOUND_HANGUP = 0,
    MODEM_SOUND_DIAL,
    MODEM_SOUND_ANSWER,
    MODEM_SOUND_CONNECT,
    MODEM_SOUND_BUSY,
    MODEM_SOUND_RINGING
};

modem_sound_t *modem_sound_init(void);
void modem_sound_close(modem_sound_t *sound);
void modem_sound_event(modem_sound_t *sound, int type, const char *number, int arg);
void modem_sound_speaker(modem_sound_t *sound, int mode, int level);
void modem_sound_set_region(modem_sound_t *sound, int region);
uint32_t modem_sound_dial_ms(const char *number, int s8, int pulse);
uint32_t modem_sound_ring_ms(modem_sound_t *sound);
uint32_t modem_sound_handshake_ms(void);

#endif
