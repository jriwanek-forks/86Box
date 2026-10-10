/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Modem speaker audio emulation.
 *
 * Authors: Xeon3D (original creator)
 *          Jasmine Iwanek (86Box port)
 *
 *          Copyright 2026 Xeon3D.
 *          Copyright 2026 Jasmine Iwanek.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>

#include <86box/86box.h>
#include <86box/sound.h>
#include <86box/modem/modem_sound.h>

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

#define MODEM_SOUND_SLOTS 8
#define EVENT_QUEUE_SIZE 16
#define MAX_STEPS 256
#define DIAL_TONE_MS 800
#define RING_MS 2600
#define HANDSHAKE_MS 8000

typedef struct {
    int kind;
    int digit;
    uint32_t samples;
} dial_step_t;

typedef struct {
    int type;
    int arg;
    char number[128];
} sound_event_t;

enum { PH_IDLE, PH_DIAL, PH_RING, PH_HANDSHAKE, PH_ONLINE, PH_BUSY };
enum { STEP_TONE, STEP_DTMF, STEP_PULSE, STEP_SILENCE };

struct modem_sound_t {
    sound_event_t events[EVENT_QUEUE_SIZE];
    atomic_uint write_index;
    atomic_uint read_index;
    atomic_int mode;
    atomic_int level;
    int in_use;
    int handler_registered;
    int phase;
    int rate;
    int pulse;
    int s8;
    uint32_t sample;
    uint32_t step_start;
    int step_count;
    int step_index;
    int dial_connect_after;
    int pending_event;
    dial_step_t steps[MAX_STEPS];
    double phase1;
    double phase2;
    double phase3;
    uint32_t noise_state;
};

static modem_sound_t *modem_speakers[MODEM_SOUND_SLOTS];

static double
modem_oscillator(double *phase, double frequency, int rate)
{
    *phase += 2.0 * M_PI * frequency / rate;
    if (*phase >= 2.0 * M_PI)
        *phase -= 2.0 * M_PI;
    return sin(*phase);
}

static double
modem_noise(modem_sound_t *sound)
{
    sound->noise_state = sound->noise_state * 1664525u + 1013904223u;
    return ((double) (sound->noise_state >> 8) / 8388608.0) - 1.0;
}

static uint32_t
modem_ms_samples(modem_sound_t *sound, uint32_t milliseconds)
{
    return (uint32_t) (((uint64_t) milliseconds * (uint64_t) sound->rate) / 1000u);
}

static void
modem_add_step(modem_sound_t *sound, int kind, int digit, uint32_t ms)
{
    if (sound->step_count >= MAX_STEPS)
        return;
    sound->steps[sound->step_count++] = (dial_step_t) {
        .kind = kind,
        .digit = digit,
        .samples = modem_ms_samples(sound, ms)
    };
}

static void
modem_build_dial(modem_sound_t *sound, const char *number, int s8, int pulse)
{
    sound->step_count = 0;
    modem_add_step(sound, STEP_TONE, 0, DIAL_TONE_MS);
    for (const char *p = number; p && *p; p++) {
        const char c = *p;
        if (c == 'T' || c == 't')
            pulse = 0;
        else if (c == 'P' || c == 'p')
            pulse = 1;
        else if (c == ',')
            modem_add_step(sound, STEP_TONE, 0, (uint32_t) s8 * 1000u);
        else if (c == 'W' || c == 'w')
            modem_add_step(sound, STEP_TONE, 0, 1500);
        else if (c >= '0' && c <= '9') {
            if (pulse) {
                const int count = c == '0' ? 10 : c - '0';
                for (int i = 0; i < count; i++) {
                    modem_add_step(sound, STEP_PULSE, 0, 60);
                    modem_add_step(sound, STEP_SILENCE, 0, 40);
                }
                modem_add_step(sound, STEP_SILENCE, 0, 700);
            } else {
                modem_add_step(sound, STEP_DTMF, c, 90);
                modem_add_step(sound, STEP_SILENCE, 0, 90);
            }
        } else if (!pulse && (c == '*' || c == '#' || (c >= 'A' && c <= 'D'))) {
            modem_add_step(sound, STEP_DTMF, c, 90);
            modem_add_step(sound, STEP_SILENCE, 0, 90);
        }
    }
    modem_add_step(sound, STEP_SILENCE, 0, 600);
    sound->step_index = 0;
    sound->step_start = 0;
    sound->phase = PH_DIAL;
    sound->sample = 0;
}

uint32_t
modem_sound_dial_ms(const char *number, int s8, int pulse)
{
    uint32_t ms = DIAL_TONE_MS + 600;
    for (const char *p = number; p && *p; p++) {
        const char c = *p;
        if (c == 'T' || c == 't')
            pulse = 0;
        else if (c == 'P' || c == 'p')
            pulse = 1;
        else if (c == ',')
            ms += (uint32_t) s8 * 1000u;
        else if (c == 'W' || c == 'w')
            ms += 1500;
        else if (c >= '0' && c <= '9')
            ms += pulse ? (uint32_t) ((c == '0' ? 10 : c - '0') * 100 + 700) : 180;
        else if (!pulse && (c == '*' || c == '#' || (c >= 'A' && c <= 'D')))
            ms += 180;
    }
    return ms;
}

uint32_t modem_sound_ring_ms(void) { return RING_MS; }
uint32_t modem_sound_handshake_ms(void) { return HANDSHAKE_MS; }

void
modem_sound_event(modem_sound_t *sound, int type, const char *number, int arg)
{
    if (!sound)
        return;
    const unsigned write = atomic_load(&sound->write_index);
    const unsigned read = atomic_load(&sound->read_index);
    if (write - read >= EVENT_QUEUE_SIZE)
        return;
    sound_event_t *event = &sound->events[write % EVENT_QUEUE_SIZE];
    event->type = type;
    event->arg = arg;
    snprintf(event->number, sizeof(event->number), "%s", number ? number : "");
    atomic_store(&sound->write_index, write + 1);
}

void
modem_sound_speaker(modem_sound_t *sound, int mode, int level)
{
    if (!sound)
        return;
    atomic_store(&sound->mode, mode < 0 ? 0 : (mode > 3 ? 3 : mode));
    atomic_store(&sound->level, level < 0 ? 0 : (level > 3 ? 3 : level));
}

static void
modem_sound_apply_event(modem_sound_t *sound, int type)
{
    switch (type) {
        case MODEM_SOUND_ANSWER:
            sound->phase = PH_HANDSHAKE;
            sound->sample = 0;
            sound->phase1 = sound->phase2 = sound->phase3 = 0.0;
            break;
        case MODEM_SOUND_CONNECT:
            sound->phase = PH_ONLINE;
            sound->sample = 0;
            break;
        case MODEM_SOUND_BUSY:
            sound->phase = PH_BUSY;
            sound->sample = 0;
            break;
        case MODEM_SOUND_HANGUP:
        default:
            sound->phase = PH_IDLE;
            sound->sample = 0;
            break;
    }
}

static void
modem_sound_take_events(modem_sound_t *sound)
{
    const unsigned write = atomic_load(&sound->write_index);
    unsigned read = atomic_load(&sound->read_index);
    while (read != write) {
        const sound_event_t *event = &sound->events[read % EVENT_QUEUE_SIZE];
        if (event->type != MODEM_SOUND_DIAL && sound->phase == PH_DIAL
            && sound->step_index < sound->step_count) {
            /* Do not let a fast failure/hangup erase the dial tones before they play. */
            sound->pending_event = event->type;
            read++;
            continue;
        }
        switch (event->type) {
            case MODEM_SOUND_DIAL:
                modem_build_dial(sound, event->number, event->arg & 0xff, (event->arg >> 8) & 1);
                sound->dial_connect_after = !!(event->arg & MODEM_SOUND_DIAL_CONNECT_AFTER);
                sound->pending_event = -1;
                break;
            default:
                modem_sound_apply_event(sound, event->type);
                break;
        }
        read++;
    }
    atomic_store(&sound->read_index, read);
}

static double
modem_dtmf(modem_sound_t *sound, int digit)
{
    static const char keys[] = "123A456B789C*0#D";
    static const double rows[] = { 697, 770, 852, 941 };
    static const double cols[] = { 1209, 1336, 1477, 1633 };
    const char *key = strchr(keys, digit);
    if (!key)
        return 0.0;
    const int index = (int) (key - keys);
    return 0.5 * modem_oscillator(&sound->phase1, rows[index / 4], sound->rate)
         + 0.5 * modem_oscillator(&sound->phase2, cols[index % 4], sound->rate);
}

static double
modem_ringback(modem_sound_t *sound)
{
    const uint32_t ms = (uint32_t) (((uint64_t) sound->sample * 1000u) / sound->rate);
    return (ms % 5000 < 1000) ? modem_oscillator(&sound->phase1, 425, sound->rate) : 0.0;
}

static double
modem_busy_tone(modem_sound_t *sound)
{
    const uint32_t ms = (uint32_t) (((uint64_t) sound->sample * 1000u) / sound->rate);
    return (ms % 1000 < 500) ? modem_oscillator(&sound->phase1, 425, sound->rate) : 0.0;
}

static double
modem_sound_sample(modem_sound_t *sound, int mode)
{
    double value = 0.0;
    if (sound->phase == PH_DIAL) {
        if (sound->step_index < sound->step_count) {
            const dial_step_t *step = &sound->steps[sound->step_index];
            const uint32_t elapsed = sound->sample - sound->step_start;
            if (mode == 1 || mode == 2) {
                switch (step->kind) {
                    case STEP_TONE:
                        value = 0.5 * modem_oscillator(&sound->phase1, 350, sound->rate)
                              + 0.5 * modem_oscillator(&sound->phase2, 440, sound->rate);
                        break;
                    case STEP_DTMF: value = modem_dtmf(sound, step->digit); break;
                    case STEP_PULSE: value = elapsed < (uint32_t) (sound->rate / 400) ? 0.7 * modem_noise(sound) : 0.0; break;
                    default: break;
                }
            }
            if (elapsed + 1 >= step->samples) {
                sound->step_index++;
                sound->step_start = sound->sample + 1;
            }
        } else {
            if (sound->dial_connect_after) {
                sound->dial_connect_after = 0;
                sound->phase = PH_ONLINE;
                sound->sample = 0;
            } else if (sound->pending_event >= 0) {
                const int pending_event = sound->pending_event;
                sound->pending_event = -1;
                modem_sound_apply_event(sound, pending_event);
            } else {
                sound->phase = PH_RING;
                sound->sample = 0;
            }
        }
    } else if (sound->phase == PH_RING) {
        if (mode == 1 || mode == 2 || mode == 3)
            value = 0.6 * modem_ringback(sound);
    } else if (sound->phase == PH_BUSY) {
        const uint32_t ms = (uint32_t) (((uint64_t) sound->sample * 1000u) / sound->rate);
        if (ms >= 3000)
            sound->phase = PH_IDLE;
        else if (mode == 1 || mode == 2 || mode == 3)
            value = 0.6 * modem_busy_tone(sound);
    } else if (sound->phase == PH_HANDSHAKE) {
        if (mode == 1 || mode == 2 || mode == 3) {
            const uint32_t ms = (uint32_t) (((uint64_t) sound->sample * 1000u) / sound->rate);
            const double hiss = modem_noise(sound);
            value = ms < 1300 ? modem_oscillator(&sound->phase1, 2100, sound->rate)
                 : ms < 4000 ? 0.5 * modem_oscillator(&sound->phase1, 1200, sound->rate)
                             + 0.5 * modem_oscillator(&sound->phase2, 2400, sound->rate)
                 : 0.6 * hiss + 0.4 * hiss * modem_oscillator(&sound->phase3, 1800, sound->rate);
        }
    } else if (sound->phase == PH_ONLINE && mode == 2)
        value = 0.08 * modem_noise(sound);
    sound->sample++;
    return value;
}

static void
modem_sound_get_buffer(int32_t *buffer, uint16_t length, void *priv)
{
    modem_sound_t *sound = (modem_sound_t *) priv;
    if (!sound || !buffer || sound_sample_rate <= 0)
        return;
    sound->rate = sound_sample_rate;
    modem_sound_take_events(sound);
    const int mode = atomic_load(&sound->mode);
    const int level = atomic_load(&sound->level);
    static const double gains[4] = { 0.30, 0.30, 0.55, 0.85 };
    const double gain = 11000.0 * gains[level];
    for (uint16_t i = 0; i < length; i++) {
        double sample = modem_sound_sample(sound, mode);
           const int audible = (sound->phase == PH_DIAL && (mode == 1 || mode == 2))
                        || ((sound->phase == PH_RING || sound->phase == PH_BUSY
                            || sound->phase == PH_HANDSHAKE) && mode != 0)
                        || (sound->phase == PH_ONLINE && mode == 2);
        if (audible)
            sample += 0.01 * modem_noise(sound);
        if (mode == 0)
            sample = 0.0;
        const int32_t output = (int32_t) (sample * gain);
        buffer[(i << 1)] += output;
        buffer[(i << 1) + 1] += output;
    }
}

modem_sound_t *
modem_sound_init(void)
{
    for (int i = 0; i < MODEM_SOUND_SLOTS; i++) {
        if (!modem_speakers[i]) {
            modem_speakers[i] = (modem_sound_t *) calloc(1, sizeof(modem_sound_t));
            if (!modem_speakers[i])
                return NULL;
            atomic_init(&modem_speakers[i]->write_index, 0);
            atomic_init(&modem_speakers[i]->read_index, 0);
            atomic_init(&modem_speakers[i]->mode, 1);
            atomic_init(&modem_speakers[i]->level, 2);
            modem_speakers[i]->noise_state = 0x4d4f4445u + (uint32_t) i;
        }
        if (!modem_speakers[i]->in_use) {
            modem_sound_t *sound = modem_speakers[i];
            sound->in_use = 1;
            sound->phase = PH_IDLE;
            sound->step_count = sound->step_index = 0;
            sound->dial_connect_after = 0;
            sound->pending_event = -1;
            atomic_store(&sound->read_index, atomic_load(&sound->write_index));
            if (!sound->handler_registered) {
                sound_add_handler(modem_sound_get_buffer, sound);
                sound->handler_registered = 1;
            }
            return sound;
        }
    }
    return NULL;
}

void
modem_sound_close(modem_sound_t *sound)
{
    if (sound) {
        modem_sound_event(sound, MODEM_SOUND_HANGUP, NULL, 0);
        sound->in_use = 0;
    }
}
