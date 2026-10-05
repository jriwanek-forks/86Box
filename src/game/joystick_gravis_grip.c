/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Gravis GrIP gameport protocol emulation.
 *
 *          Copyright 2021-2025 Jasmine Iwanek.
 */
#include <stdint.h>
#include <stdlib.h>
#include <86box/gameport.h>
#include <86box/plat_unused.h>

#define GRIP_MODE_GPP 1
#define GRIP_MODE_BD  2
#define GRIP_MODE_XT  3
#define GRIP_MODE_DC  4

#define GRIP_GPP_LENGTH 24
#define GRIP_XT_CHUNK_LENGTH 20
#define GRIP_XT_CHUNKS 4

typedef struct grip_data {
    int      mode;
    uint32_t frame[MAX_JOYSTICKS][GRIP_XT_CHUNKS];
    uint8_t  lines[MAX_JOYSTICKS];
    uint8_t  bit_pos[MAX_JOYSTICKS];
    uint8_t  chunk[MAX_JOYSTICKS];
    uint8_t  delimiter[MAX_JOYSTICKS];
    uint8_t  started[MAX_JOYSTICKS];
} grip_data;

static int
grip_axis6(int value)
{
    return ((value + 32768) * 63) / 65535;
}

static unsigned
grip_hat_bits(int angle, unsigned offset)
{
    unsigned bits = 0;

    if (angle < 0)
        return 0;

    angle %= 360;
    if (angle >= 45 && angle < 135)
        bits |= 2;
    else if (angle >= 225 && angle < 315)
        bits |= 1;

    if (angle >= 135 && angle < 225)
        bits |= 4;
    else if (angle < 45 || angle >= 315)
        bits |= 8;

    return bits << offset;
}

static uint32_t
grip_gpp_data(const joystick_state_t *state)
{
    uint32_t data = 0x7c0000;
    static const uint8_t button_bits[] = { 0, 1, 2, 3, 5, 6, 7, 8, 10, 11 };

    for (int i = 0; i < 10; i++)
        if (state->button[i])
            data |= 1U << button_bits[i];

    if (state->axis[0] < -8192)
        data |= 1U << 16;
    else if (state->axis[0] > 8192)
        data |= 1U << 15;

    if (state->axis[1] < -8192)
        data |= 1U << 12;
    else if (state->axis[1] > 8192)
        data |= 1U << 13;

    return data;
}

static uint16_t
grip_build_payload(const joystick_state_t *state, int mode, int chunk)
{
    uint16_t data = 0;
    int axis_x = 0;
    int axis_y = 0;

    switch (chunk) {
        case 0:
            axis_x = grip_axis6(state->axis[0]);
            axis_y = 63 - grip_axis6(state->axis[1]);
            data = (uint16_t) ((axis_x << 2) | (axis_y << 8));
            break;

        case 1:
            if (mode == GRIP_MODE_XT) {
                data = (uint16_t) ((grip_axis6(state->axis[2]) << 2) |
                                   (grip_axis6(state->axis[3]) << 8));
            } else if (mode == GRIP_MODE_DC) {
                data = (uint16_t) ((grip_axis6(state->axis[2]) << 2) |
                                   (grip_axis6(state->axis[3]) << 8));
            }
            break;

        case 2:
            data = (uint16_t) (grip_hat_bits(state->pov[0], 0) |
                               (grip_axis6(state->axis[mode == GRIP_MODE_BD ? 2 : 4]) << 8));
            if (mode == GRIP_MODE_XT)
                data |= (uint16_t) grip_hat_bits(state->pov[1], 4);
            else if (mode == GRIP_MODE_DC)
                data |= 0x0010;
            break;

        case 3: {
            int button_count = mode == GRIP_MODE_BD ? 5 : mode == GRIP_MODE_XT ? 11 : 9;
            int first_bit = mode == GRIP_MODE_BD ? 4 : 3;

            if (mode == GRIP_MODE_XT)
                data |= 1;
            for (int i = 0; i < button_count; i++)
                if (state->button[i])
                    data |= (uint16_t) (1U << (first_bit + i));
            break;
        }
    }

    return data;
}

static uint32_t
grip_make_xt_chunk(const joystick_state_t *state, int mode, int chunk)
{
    uint32_t payload = grip_build_payload(state, mode, chunk) & 0x3fff;
    uint32_t packet = ((uint32_t) chunk << 18) | (payload << 4);

    for (uint32_t crc_bits = 0; crc_bits < 16; crc_bits++) {
        uint32_t candidate = packet | crc_bits;
        uint32_t crc = candidate ^ (candidate >> 7) ^ (candidate >> 14);
        uint32_t check = 0x25cb9e70U >> ((crc >> 2) & 0x1c);

        if (!((crc ^ check) & 0xf))
            return candidate;
    }

    return packet;
}

static void
grip_prepare_frame(grip_data *grip, int slot)
{
    const joystick_state_t *state = &joystick_state[0][slot];

    if (grip->mode == GRIP_MODE_GPP) {
        grip->frame[slot][0] = grip_gpp_data(state);
        return;
    }

    for (int chunk = 0; chunk < GRIP_XT_CHUNKS; chunk++)
        grip->frame[slot][chunk] = grip_make_xt_chunk(state, grip->mode, chunk);
}

static void *
grip_init_mode(int mode)
{
    grip_data *grip = calloc(1, sizeof(*grip));

    if (grip) {
        grip->mode = mode;
        for (int i = 0; i < MAX_JOYSTICKS; i++)
            grip->lines[i] = mode == GRIP_MODE_GPP ? 3 : 0;
    }

    return grip;
}

static void *
grip_init_gpp(void)
{
    return grip_init_mode(GRIP_MODE_GPP);
}

static void *
grip_init_bd(void)
{
    return grip_init_mode(GRIP_MODE_BD);
}

static void *
grip_init_xt(void)
{
    return grip_init_mode(GRIP_MODE_XT);
}

static void *
grip_init_dc(void)
{
    return grip_init_mode(GRIP_MODE_DC);
}

static void
grip_close(void *priv)
{
    free(priv);
}

static uint8_t
grip_read(void *priv)
{
    grip_data *grip = priv;
    uint8_t ret = 0xf0;
    int connected = 0;

    for (int slot = 0; slot < joystick_gravis_gamepad_pro.max_joysticks; slot++) {
        uint8_t shift = (uint8_t) (4 + slot * 2);

        if (!JOYSTICK_PRESENT(0, slot))
            continue;

        connected = 1;
        if (!grip->started[slot]) {
            grip_prepare_frame(grip, slot);
            grip->started[slot] = 1;
        } else if (grip->mode == GRIP_MODE_GPP) {
            uint8_t clock = grip->lines[slot] & 1;
            uint8_t bit = (uint8_t) ((grip->frame[slot][0] >> grip->bit_pos[slot]) & 1);

            clock = !clock;
            grip->lines[slot] = (uint8_t) ((grip->lines[slot] & 2) | clock);
            if (!clock && ++grip->bit_pos[slot] == GRIP_GPP_LENGTH) {
                grip->bit_pos[slot] = 0;
                grip_prepare_frame(grip, slot);
            }
            grip->lines[slot] = (uint8_t) ((grip->lines[slot] & 1) | (bit << 1));
        } else if (grip->delimiter[slot]) {
            grip->lines[slot] ^= 2;
            if (++grip->delimiter[slot] == 3) {
                grip->delimiter[slot] = 0;
                grip->bit_pos[slot] = 0;
                if (++grip->chunk[slot] == GRIP_XT_CHUNKS) {
                    grip->chunk[slot] = 0;
                    grip_prepare_frame(grip, slot);
                }
            }
        } else {
            uint8_t clock = grip->lines[slot] & 1;
            uint8_t bit = (uint8_t) ((grip->frame[slot][grip->chunk[slot]] >>
                                      (GRIP_XT_CHUNK_LENGTH - 1 - grip->bit_pos[slot])) & 1);

            clock = !clock;
            grip->lines[slot] = (uint8_t) ((bit << 1) | clock);
            if (++grip->bit_pos[slot] == GRIP_XT_CHUNK_LENGTH)
                grip->delimiter[slot] = 1;
        }

        if (!(grip->lines[slot] & 1))
            ret &= (uint8_t) ~(1U << shift);
        if (!(grip->lines[slot] & 2))
            ret &= (uint8_t) ~(1U << (shift + 1));
    }

    return connected ? ret : 0xff;
}

static void
grip_write(UNUSED(void *priv))
{
}

static int
grip_read_axis(UNUSED(void *priv), UNUSED(int axis))
{
    return AXIS_NOT_PRESENT;
}

static void
grip_a0_over(UNUSED(void *priv))
{
}

const joystick_t joystick_gravis_gamepad_pro = {
    .name          = "Gravis GamePad Pro",
    .internal_name = "gravis_gamepad_pro",
    .init          = grip_init_gpp,
    .close         = grip_close,
    .read          = grip_read,
    .write         = grip_write,
    .read_axis     = grip_read_axis,
    .a0_over       = grip_a0_over,
    .axis_count    = 2,
    .button_count  = 10,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "D-pad X", "D-pad Y" },
    .button_names  = { "Start", "Select", "R2", "Blue", "L2", "Yellow", "Red", "Green", "L1", "R1" },
    .pov_names     = { NULL }
};

const joystick_t joystick_gravis_blackhawk_digital = {
    .name          = "Gravis Blackhawk Digital",
    .internal_name = "gravis_blackhawk_digital",
    .init          = grip_init_bd,
    .close         = grip_close,
    .read          = grip_read,
    .write         = grip_write,
    .read_axis     = grip_read_axis,
    .a0_over       = grip_a0_over,
    .axis_count    = 3,
    .button_count  = 5,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis", "Throttle" },
    .button_names  = { "Thumb", "Thumb 2", "Trigger", "Top", "Base" },
    .pov_names     = { "POV" }
};

const joystick_t joystick_gravis_xterminator_digital = {
    .name          = "Gravis Xterminator Digital",
    .internal_name = "gravis_xterminator_digital",
    .init          = grip_init_xt,
    .close         = grip_close,
    .read          = grip_read,
    .write         = grip_write,
    .read_axis     = grip_read_axis,
    .a0_over       = grip_a0_over,
    .axis_count    = 5,
    .button_count  = 11,
    .pov_count     = 2,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis", "Brake", "Gas", "Throttle" },
    .button_names  = { "Trigger", "Thumb", "A", "B", "C", "X", "Y", "Z", "Select", "Start", "Mode" },
    .pov_names     = { "POV 1", "POV 2" }
};

const joystick_t joystick_gravis_xterminator_dualcontrol = {
    .name          = "Gravis Xterminator DualControl",
    .internal_name = "gravis_xterminator_dualcontrol",
    .init          = grip_init_dc,
    .close         = grip_close,
    .read          = grip_read,
    .write         = grip_write,
    .read_axis     = grip_read_axis,
    .a0_over       = grip_a0_over,
    .axis_count    = 5,
    .button_count  = 9,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "X axis", "Y axis", "RX axis", "RY axis", "Throttle" },
    .button_names  = { "Trigger", "Thumb", "Top", "Top 2", "Base", "Base 2", "Base 3", "Base 4", "Base 5" },
    .pov_names     = { "POV" }
};
