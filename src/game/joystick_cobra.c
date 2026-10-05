/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Creative Labs Blaster GamePad Cobra gameport protocol emulation.
 *
 *          Copyright 2021-2025 Jasmine Iwanek.
 */
#include <stdint.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/gameport.h>
#include <86box/plat_unused.h>

#define COBRA_LENGTH 36
#define COBRA_SYNC_MASK  0x04104107fULL
#define COBRA_SYNC_VALUE 0x041041040ULL

typedef struct cobra_data {
    uint64_t packet[MAX_JOYSTICKS];
    uint8_t  lines[MAX_JOYSTICKS];
    uint8_t  bit_pos[MAX_JOYSTICKS];
    uint8_t  started[MAX_JOYSTICKS];
} cobra_data;

static uint64_t
cobra_encode_packet(const joystick_state_t *state)
{
    uint32_t data = 0;
    uint64_t packet = 0;

    if (state->axis[0] < -16384)
        data |= 1U << 3;
    else if (state->axis[0] > 16384)
        data |= 1U << 4;

    if (state->axis[1] < -16384)
        data |= 1U << 1;
    else if (state->axis[1] > 16384)
        data |= 1U << 2;

    for (int button = 0; button < 12; button++)
        if (state->button[button])
            data |= 1U << (button + 5);

    packet |= (uint64_t) (data & 0x000001f) << 7;
    packet |= (uint64_t) ((data >> 5) & 0x000001f) << 13;
    packet |= (uint64_t) ((data >> 10) & 0x000001f) << 19;
    packet |= (uint64_t) ((data >> 15) & 0x000001f) << 25;
    packet |= (uint64_t) ((data >> 20) & 0x000001f) << 31;

    return (packet & ~COBRA_SYNC_MASK) | COBRA_SYNC_VALUE;
}

static void *
cobra_init(void)
{
    cobra_data *cobra = calloc(1, sizeof(*cobra));

    if (cobra)
        for (int i = 0; i < MAX_JOYSTICKS; i++)
            cobra->lines[i] = 3;

    return cobra;
}

static void
cobra_close(void *priv)
{
    free(priv);
}

static uint8_t
cobra_read(void *priv)
{
    cobra_data *cobra = priv;
    uint8_t ret = 0xf0;
    int connected = 0;

    for (int i = 0; i < joystick_creative_cobra.max_joysticks; i++) {
        uint8_t shift = (uint8_t) (4 + i * 2);

        if (!JOYSTICK_PRESENT(0, i))
            continue;

        connected = 1;
        if (!cobra->started[i]) {
            cobra->packet[i] = cobra_encode_packet(&joystick_state[0][i]);
            cobra->started[i] = 1;
        } else {
            uint8_t bit = (uint8_t) ((cobra->packet[i] >> cobra->bit_pos[i]) & 1);

            /* Cobra encodes each bit as a transition on exactly one of its lines. */
            cobra->lines[i] ^= bit ? 2 : 1;
            if (++cobra->bit_pos[i] == COBRA_LENGTH) {
                cobra->bit_pos[i] = 0;
                cobra->packet[i] = cobra_encode_packet(&joystick_state[0][i]);
            }
        }

        if (!(cobra->lines[i] & 1))
            ret &= (uint8_t) ~(1U << shift);
        if (!(cobra->lines[i] & 2))
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

const joystick_t joystick_creative_cobra = {
    .name          = "Creative Labs Blaster GamePad Cobra",
    .internal_name = "creative_cobra",
    .init          = cobra_init,
    .close         = cobra_close,
    .read          = cobra_read,
    .write         = cobra_write,
    .read_axis     = cobra_read_axis,
    .a0_over       = cobra_a0_over,
    .axis_count    = 2,
    .button_count  = 12,
    .pov_count     = 0,
    .max_joysticks = 2,
    .axis_names    = { "D-pad X", "D-pad Y" },
    .button_names  = { "Start", "Select", "TL", "TR", "X", "Y", "Z", "A", "B", "C", "TL2", "TR2" },
    .pov_names     = { NULL }
};
