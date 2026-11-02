/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Creative Labs GamePad-I and Blaster GamePad Cobra (GamePad-II)
 *          digital gameport protocol emulation, and the joystick side of
 *          the Cobra 2 USB, whose USB device is in usb_hid.c.
 *
 * Notes:   - The pad free-runs: it transmits for as long as it is powered and
 *            nothing is ever written to the port. Creative's BGAMEPAD.VXD and
 *            BGAMETST.EXE only ever read 201h in digital mode.
 *          - Each bit is a transition on exactly one of the pad's two button
 *            lines: 0 toggles the lower line, 1 toggles the upper line.
 *          - A frame is six 0 bits, which is the sync, followed by groups of
 *            a 1 marker and five data bits. The markers keep any other run of
 *            0 bits below six.
 *          - The drivers tell the models apart by counting groups:
 *              3   GamePad-I (10 buttons)
 *              5   GamePad-II (Cobra, 12 buttons and three spare bits)
 *              8   GamePad-II with a GamePad-I chained to it
 *              10  GamePad-II with a GamePad-II chained to it
 *          - Only a GamePad-II can host a chain, as the flag lives in its
 *            fifth group. Slots 0/1 are the two pads of a Y cable and slots
 *            2/3 are chained behind them, matching the driver's device IDs.
 *          - The bit rate is not documented anywhere. Linux gives up after
 *            45us without a bit and the Creative drivers tolerate far more,
 *            so COBRA_BIT_USEC is an estimate within that limit.
 *
 *          Copyright 2021-2025 Jasmine Iwanek.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/gameport.h>
#include <86box/plat_unused.h>

#define COBRA_BIT_USEC    20

#define COBRA_PAIRS       2
#define COBRA_SYNC_BITS   6
#define COBRA_GROUP_DATA  5
#define COBRA_GROUPS_I    3
#define COBRA_GROUPS_II   5
#define COBRA_MAX_BITS    (COBRA_SYNC_BITS + (2 * COBRA_GROUPS_II * (COBRA_GROUP_DATA + 1)))

/* Bit 0 of the data is an extension flag, which is clear on every known pad. */
#define COBRA_DATA_UP     (1U << 1)
#define COBRA_DATA_DOWN   (1U << 2)
#define COBRA_DATA_LEFT   (1U << 3)
#define COBRA_DATA_RIGHT  (1U << 4)
#define COBRA_DATA_BUTTON 5
#define COBRA_DATA_CHAIN  (1U << 20)

typedef struct cobra_pair_t {
    uint8_t bits[COBRA_MAX_BITS];
    uint8_t length;
    uint8_t pos;
    uint8_t lines;
} cobra_pair_t;

typedef struct cobra_data {
    cobra_pair_t pair[COBRA_PAIRS];
    uint64_t     last_tsc;
    int          groups;
    int          buttons;
} cobra_data;

static uint32_t
cobra_pad_data(const cobra_data *cobra, int slot)
{
    const joystick_state_t *state = &joystick_state[0][slot];
    uint32_t                data  = 0;

    if (state->axis[0] < -16384)
        data |= COBRA_DATA_LEFT;
    else if (state->axis[0] > 16384)
        data |= COBRA_DATA_RIGHT;

    if (state->axis[1] < -16384)
        data |= COBRA_DATA_UP;
    else if (state->axis[1] > 16384)
        data |= COBRA_DATA_DOWN;

    for (int button = 0; button < cobra->buttons; button++)
        if (state->button[button])
            data |= 1U << (button + COBRA_DATA_BUTTON);

    return data;
}

static void
cobra_append_pad(cobra_pair_t *pair, uint32_t data, int groups)
{
    for (int group = 0; group < groups; group++) {
        pair->bits[pair->length++] = 1;
        for (int bit = 0; bit < COBRA_GROUP_DATA; bit++)
            pair->bits[pair->length++] = (data >> ((group * COBRA_GROUP_DATA) + bit)) & 1;
    }
}

static void
cobra_build_frame(cobra_data *cobra, int index)
{
    cobra_pair_t *pair = &cobra->pair[index];

    pair->pos = 0;

    if (!JOYSTICK_PRESENT(0, index)) {
        pair->length = 0;
        pair->lines  = 3;
        return;
    }

    memset(pair->bits, 0, COBRA_SYNC_BITS);
    pair->length = COBRA_SYNC_BITS;

    if ((cobra->groups == COBRA_GROUPS_II) && JOYSTICK_PRESENT(0, index + COBRA_PAIRS)) {
        cobra_append_pad(pair, cobra_pad_data(cobra, index) | COBRA_DATA_CHAIN, cobra->groups);
        cobra_append_pad(pair, cobra_pad_data(cobra, index + COBRA_PAIRS), cobra->groups);
    } else
        cobra_append_pad(pair, cobra_pad_data(cobra, index), cobra->groups);
}

/* Bring the free-running transmitters up to the current time. Working this out
   on read rather than from a timer avoids a 50kHz event for an idle pad. */
static void
cobra_clock(cobra_data *cobra)
{
    uint64_t period = (TIMER_USEC * COBRA_BIT_USEC) >> 32;
    uint64_t bits;

    if (!period)
        period = 1;

    bits = (tsc - cobra->last_tsc) / period;
    cobra->last_tsc += bits * period;

    for (int index = 0; index < COBRA_PAIRS; index++) {
        cobra_pair_t *pair = &cobra->pair[index];
        uint64_t      left = bits;

        if (!pair->length)
            cobra_build_frame(cobra, index);
        else if (left >= pair->length) {
            /* Nobody was listening; only the current frame can be seen. */
            left %= pair->length;
            cobra_build_frame(cobra, index);
        }

        while (left-- && pair->length) {
            pair->lines ^= pair->bits[pair->pos] ? 2 : 1;

            if (++pair->pos == pair->length)
                cobra_build_frame(cobra, index);
        }
    }
}

static void *
cobra_init_model(int groups, int buttons)
{
    cobra_data *cobra = calloc(1, sizeof(*cobra));

    if (cobra) {
        cobra->groups   = groups;
        cobra->buttons  = buttons;
        cobra->last_tsc = tsc;

        for (int i = 0; i < COBRA_PAIRS; i++)
            cobra->pair[i].lines = 3;
    }

    return cobra;
}

static void *
cobra_init_gamepad(void)
{
    return cobra_init_model(COBRA_GROUPS_I, 10);
}

static void *
cobra_init_cobra(void)
{
    return cobra_init_model(COBRA_GROUPS_II, 12);
}

static void
cobra_close(void *priv)
{
    free(priv);
}

static uint8_t
cobra_read(void *priv)
{
    cobra_data *cobra     = priv;
    uint8_t     ret       = 0xf0;
    int         connected = 0;

    cobra_clock(cobra);

    for (int index = 0; index < COBRA_PAIRS; index++) {
        const cobra_pair_t *pair  = &cobra->pair[index];
        uint8_t             shift = (uint8_t) (4 + (index * 2));

        if (!pair->length)
            continue;

        connected = 1;

        if (!(pair->lines & 1))
            ret &= (uint8_t) ~(1U << shift);
        if (!(pair->lines & 2))
            ret &= (uint8_t) ~(1U << (shift + 1));
    }

    return connected ? ret : 0xff;
}

static void
cobra_write(UNUSED(void *priv))
{
}

static int
cobra_read_axis(UNUSED(void *priv), UNUSED(int axis))
{
    return AXIS_NOT_PRESENT;
}

static void
cobra_a0_over(UNUSED(void *priv))
{
}

/* The Cobra 2 is a USB device, so nothing is ever seen on a game port. */
static void *
cobra_usb_init(void)
{
    return NULL;
}

static void
cobra_usb_close(UNUSED(void *priv))
{
}

static uint8_t
cobra_usb_read(UNUSED(void *priv))
{
    return 0xff;
}

const joystick_t joystick_creative_gamepad = {
    .name          = "Creative Labs GamePad-I",
    .internal_name = "creative_gamepad",
    .init          = cobra_init_gamepad,
    .close         = cobra_close,
    .read          = cobra_read,
    .write         = cobra_write,
    .read_axis     = cobra_read_axis,
    .a0_over       = cobra_a0_over,
    .axis_count    = 2,
    .button_count  = 10,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "D-pad X", "D-pad Y" },
    .button_names  = { "Start", "Select", "TL", "TR", "X", "Y", "Z", "A", "B", "C" },
    .pov_names     = { NULL }
};

const joystick_t joystick_creative_cobra = {
    .name          = "Creative Labs Blaster GamePad Cobra",
    .internal_name = "creative_cobra",
    .init          = cobra_init_cobra,
    .close         = cobra_close,
    .read          = cobra_read,
    .write         = cobra_write,
    .read_axis     = cobra_read_axis,
    .a0_over       = cobra_a0_over,
    .axis_count    = 2,
    .button_count  = 12,
    .pov_count     = 0,
    .max_joysticks = 4,
    .axis_names    = { "D-pad X", "D-pad Y" },
    .button_names  = { "Start", "Select", "TL", "TR", "X", "Y", "Z", "A", "B", "C", "TL2", "TR2" },
    .pov_names     = { NULL }
};

const joystick_t joystick_creative_cobra2_usb = {
    .name          = "Creative Labs GamePad Cobra 2 USB",
    .internal_name = "creative_cobra2_usb",
    .init          = cobra_usb_init,
    .close         = cobra_usb_close,
    .read          = cobra_usb_read,
    .write         = cobra_write,
    .read_axis     = cobra_read_axis,
    .a0_over       = cobra_a0_over,
    .axis_count    = 3,
    .button_count  = 12,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "D-pad X", "D-pad Y", "Throttle" },
    .button_names  = { "A", "B", "C", "X", "Y", "Z", "L1", "R1", "Start", "Select", "L2", "R2" },
    .pov_names     = { NULL },
    .usb_device    = { &gamepad_cobra2_usb_device, &gamepad_cobra2_usb_2_device }
};
