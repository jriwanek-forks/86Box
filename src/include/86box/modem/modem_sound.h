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

typedef struct modem_sound_t modem_sound_t;

enum {
    MODEM_SOUND_HANGUP = 0,
    MODEM_SOUND_DIAL,
    MODEM_SOUND_ANSWER,
    MODEM_SOUND_CONNECT,
    MODEM_SOUND_BUSY
};

modem_sound_t *modem_sound_init(void);
void modem_sound_close(modem_sound_t *sound);
void modem_sound_event(modem_sound_t *sound, int type, const char *number, int arg);
void modem_sound_speaker(modem_sound_t *sound, int mode, int level);
uint32_t modem_sound_dial_ms(const char *number, int s8, int pulse);
uint32_t modem_sound_ring_ms(void);
uint32_t modem_sound_handshake_ms(void);

#endif
