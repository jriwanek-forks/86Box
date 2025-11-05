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

/* MODEM_SOUND_ANSWER argument: bit 0 is set on the modem that answers the call,
   bits 8 and up carry the DTE speed divided by 100 (used to pick the standard). */
#define MODEM_SOUND_ANSWER_ANSWERER    (1 << 0)
#define MODEM_SOUND_ANSWER_ARG(answerer, baud) \
    (((answerer) ? MODEM_SOUND_ANSWER_ANSWERER : 0) | ((int) ((baud) / 100) << 8))

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

/* Handshake sound setting. The values are saved in configuration files. */
enum {
    MODEM_SOUND_HANDSHAKE_SIMPLE = 0,
    MODEM_SOUND_HANDSHAKE_FULL   = 1, /* answer tone, negotiation and training */
    MODEM_SOUND_HANDSHAKE_OFF    = 2  /* silent, and the call connects without the delay */
};

/* Modulation standard whose handshake is imitated when the full handshake is on. */
enum {
    MODEM_SOUND_STD_AUTO = 0, /* chosen from the DTE speed */
    MODEM_SOUND_STD_V21,
    MODEM_SOUND_STD_V22,
    MODEM_SOUND_STD_V22BIS,
    MODEM_SOUND_STD_V32,
    MODEM_SOUND_STD_V32BIS,
    MODEM_SOUND_STD_V34,
    MODEM_SOUND_STD_K56FLEX, /* 1996-97 */
    MODEM_SOUND_STD_X2,      /* 1997 */
    MODEM_SOUND_STD_V90,     /* 1998 */
    MODEM_SOUND_STD_V92,     /* 2000 */
    MODEM_SOUND_STD_COUNT
};

modem_sound_t *modem_sound_init(void);
void modem_sound_close(modem_sound_t *sound);
void modem_sound_event(modem_sound_t *sound, int type, const char *number, int arg);
void modem_sound_speaker(modem_sound_t *sound, int mode, int level);
void modem_sound_set_region(modem_sound_t *sound, int region);
/* handshake is MODEM_SOUND_HANDSHAKE_*; standard is MODEM_SOUND_STD_*. */
void modem_sound_set_handshake(modem_sound_t *sound, int handshake, int standard);
int modem_sound_full(modem_sound_t *sound);
uint32_t modem_sound_dial_ms(const char *number, int s8, int pulse);
uint32_t modem_sound_ring_ms(modem_sound_t *sound);
uint32_t modem_sound_handshake_ms(void);
/* Handshake length for the given DTE speed: the fixed length for the simple handshake, 0 when off. */
uint32_t modem_sound_handshake_for(modem_sound_t *sound, uint32_t baud);
/* When a virtual ISP call should report CONNECT: before the end of the full handshake, so the
   last of the sound plays over link negotiation and authentication. */
uint32_t modem_sound_connect_for(modem_sound_t *sound, uint32_t baud);

#endif
