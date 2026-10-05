/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the Logitech ADI gameport protocol.
 *
 * Notes:   The ADI activation sequence, product IDs, and packet formats
 *          are based on the protocol supported by Linux's adi.c driver.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2026 Jasmine Iwanek.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free  Software  Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is  distributed in the hope that it will be useful, but
 * WITHOUT   ANY  WARRANTY; without even   the implied warranty of
 * MERCHANTABILITY  or FITNESS  FOR A PARTICULAR  PURPOSE. See  the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the:
 *
 *   Free Software Foundation, Inc.
 *   59 Temple Place - Suite 330
 *   Boston, MA 02111-1307
 *   USA.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/gameport.h>
#include <86box/plat_unused.h>
#include <86box/timer.h>

#define ADI_INTERVAL_TIMEOUT_US 1000000
#define ADI_PACKET_TIMEOUT_US    200000
#define ADI_CLOCK_HALF_PERIOD_US      5
#define ADI_SEQUENCE_LENGTH           9
#define ADI_ID_PACKET_BITS           66
#define ADI_MAX_PACKET_BITS         128
#define ADI_FLAG_HAT               0x04
#define ADI_FLAG_10BIT             0x08

typedef struct adi_profile {
    const char *name;
    const char *internal_name;
    uint8_t     id;
    uint8_t     axis_count;
    uint8_t     button_count;
    uint8_t     pov_count;
    int8_t      pad_index;
} adi_profile;

typedef struct adi_data {
    pc_timer_t       interval_timer;
    pc_timer_t       packet_timer;
    pc_timer_t       reset_timer;
    const adi_profile *profile;
    uint128_t        packet[2];
    uint128_t        packet_mask[2];
    uint8_t          sequence[ADI_SEQUENCE_LENGTH];
    uint8_t          sequence_count;
    uint8_t          button_lines[2];
    uint8_t          packet_bits[2];
    uint8_t          marker_bits[2];
    uint8_t          marker_pending[2];
    uint8_t          digital_mode;
    uint8_t          has_last_write;
} adi_data;

static const adi_profile adi_profiles[] = {
    { "Logitech WingMan Extreme Digital", "logitech_wingman_extreme_digital", 0, 3, 6, 1, -1 },
    { "Logitech ThunderPad Digital", "logitech_thunderpad_digital", 1, 2, 7, 1, 4 },
    { "Logitech SideCar", "logitech_sidecar", 2, 6, 8, 0, -1 },
    { "Logitech CyberMan 2", "logitech_cyberman_2", 3, 6, 8, 0, -1 },
    { "Logitech WingMan Interceptor", "logitech_wingman_interceptor", 4, 3, 9, 3, -1 },
    { "Logitech WingMan Formula", "logitech_wingman_formula", 5, 3, 8, 3, -1 },
    { "Logitech WingMan GamePad", "logitech_wingman_gamepad", 6, 2, 7, 1, 0 },
    { "Logitech WingMan Extreme Digital 3D", "logitech_wingman_extreme_digital_3d", 7, 4, 7, 1, -1 },
    { "Logitech WingMan GamePad Extreme", "logitech_wingman_gamepad_extreme", 8, 2, 11, 1, -1 }
};

static const uint8_t adi_activation_sequence[ADI_SEQUENCE_LENGTH] = {
    4, 2, 3, 10, 6, 11, 7, 9, 11
};

static void
adi_interval_timer_over(UNUSED(void *priv))
{
}

static void
adi_packet_timer_over(void *priv)
{
    adi_data *adi = (adi_data *) priv;

    if (!adi->digital_mode)
        return;

    int packet_active = 0;
    for (uint8_t lane = 0; lane < 2; lane++) {
        if (adi->marker_pending[lane]) {
            adi->button_lines[lane] ^= adi->marker_bits[lane] ? 2 : 1;
            adi->marker_pending[lane] = 0;
        } else if (adi->packet_bits[lane]) {
            adi->button_lines[lane] ^= (adi->packet[lane] & adi->packet_mask[lane]) ? 2 : 1;
            adi->packet_bits[lane]--;
            adi->packet_mask[lane] = adi->packet_bits[lane] ? adi->packet_mask[lane] >> 1 : 0;
        }

        packet_active |= adi->marker_pending[lane] || adi->packet_bits[lane];
    }

    if (packet_active)
        timer_advance_u64(&adi->packet_timer, TIMER_USEC * ADI_CLOCK_HALF_PERIOD_US);
}

static void
adi_reset_timer_over(void *priv)
{
    adi_data *adi = (adi_data *) priv;

    adi->digital_mode = 0;
    adi->sequence_count = 0;
    adi->has_last_write = 0;
    memset(adi->packet_bits, 0, sizeof(adi->packet_bits));
    memset(adi->marker_pending, 0, sizeof(adi->marker_pending));
    timer_disable(&adi->packet_timer);
}

static void *
adi_init_profile(const adi_profile *profile)
{
    adi_data *adi = (adi_data *) calloc(1, sizeof(adi_data));

    adi->profile = profile;
    timer_add(&adi->interval_timer, adi_interval_timer_over, adi, 0);
    timer_add(&adi->packet_timer, adi_packet_timer_over, adi, 0);
    timer_add(&adi->reset_timer, adi_reset_timer_over, adi, 0);

    return adi;
}

static void *
adi_init_0(void)
{
    return adi_init_profile(&adi_profiles[0]);
}

static void *
adi_init_1(void)
{
    return adi_init_profile(&adi_profiles[1]);
}

static void *
adi_init_2(void)
{
    return adi_init_profile(&adi_profiles[2]);
}

static void *
adi_init_3(void)
{
    return adi_init_profile(&adi_profiles[3]);
}

static void *
adi_init_4(void)
{
    return adi_init_profile(&adi_profiles[4]);
}

static void *
adi_init_5(void)
{
    return adi_init_profile(&adi_profiles[5]);
}

static void *
adi_init_6(void)
{
    return adi_init_profile(&adi_profiles[6]);
}

static void *
adi_init_7(void)
{
    return adi_init_profile(&adi_profiles[7]);
}

static void *
adi_init_8(void)
{
    return adi_init_profile(&adi_profiles[8]);
}

static void
adi_close(void *priv)
{
    adi_data *adi = (adi_data *) priv;

    timer_disable(&adi->interval_timer);
    timer_disable(&adi->packet_timer);
    timer_disable(&adi->reset_timer);
    free(adi);
}

static int
adi_has_input(uint8_t joystick)
{
    return JOYSTICK_PRESENT(0, joystick);
}

static uint8_t
adi_read(void *priv)
{
    adi_data *adi = (adi_data *) priv;

    if (!adi_has_input(0) && !adi_has_input(1))
        return 0xff;

    if (adi->digital_mode)
        return (uint8_t) ((adi->button_lines[0] << 4) | (adi->button_lines[1] << 6));

    uint8_t result = 0xf0;
    for (uint8_t joystick = 0; joystick < 2; joystick++) {
        if (!adi_has_input(joystick))
            continue;

        for (uint8_t button = 0; button < 2; button++) {
            if (joystick_state[0][joystick].button[button])
                result &= (uint8_t) ~(0x10 << (joystick * 2 + button));
        }
    }

    return result;
}

static int
adi_read_axis(void *priv, int axis)
{
    adi_data *adi = (adi_data *) priv;

    if ((axis < 0) || (axis >= 4) || !adi_has_input(axis / 2) || adi->digital_mode)
        return AXIS_NOT_PRESENT;

    return joystick_state[0][axis / 2].axis[axis % 2];
}

static void
adi_a0_over(UNUSED(void *priv))
{
}

static void
adi_append_bits(uint128_t *packet, uint8_t *packet_bits, uint64_t value, uint8_t count)
{
    if (*packet_bits + count > ADI_MAX_PACKET_BITS)
        return;

    *packet |= (uint128_t) value << *packet_bits;
    *packet_bits += count;
}

static uint8_t
adi_hat_direction(int angle)
{
    if (angle < 0)
        return 0;

    return (uint8_t) ((((angle + 22) % 360 + 22) / 45) % 8 + 1);
}

static uint8_t
adi_hat_buttons(int angle)
{
    uint8_t direction = adi_hat_direction(angle);
    uint8_t buttons   = 0;

    if ((direction == 7) || (direction == 8) || (direction == 1))
        buttons |= 1 << 3;
    if ((direction == 1) || (direction == 2) || (direction == 3))
        buttons |= 1 << 2;
    if ((direction == 3) || (direction == 4) || (direction == 5))
        buttons |= 1 << 1;
    if ((direction == 5) || (direction == 6) || (direction == 7))
        buttons |= 1;

    return buttons;
}

static uint16_t
adi_axis_to_10bit(int axis)
{
    if (axis < -32768)
        axis = -32768;
    else if (axis > 32767)
        axis = 32767;

    return (uint16_t) (((int64_t) (axis + 32768) * 1023) / 65535);
}

static uint8_t
adi_build_packet(adi_data *adi, uint8_t joystick, int send_id, uint128_t *packet)
{
    const adi_profile *profile    = adi->profile;
    const joystick_state_t *state = &joystick_state[0][joystick];
    uint8_t packet_bits           = 0;

    *packet = 0;

    if (send_id) {
        uint8_t flags = ADI_FLAG_10BIT;

        if (profile->pov_count && (profile->pad_index < 0))
            flags |= ADI_FLAG_HAT;

        adi_append_bits(packet, &packet_bits, ADI_ID_PACKET_BITS, 10);
        adi_append_bits(packet, &packet_bits, profile->id, 8);
        adi_append_bits(packet, &packet_bits, flags, 4);
        adi_append_bits(packet, &packet_bits,
                        8 + profile->axis_count * 10 + profile->button_count +
                            ((profile->pad_index >= 0) ? 4 : 0) +
                            ((profile->pad_index < 0) ? profile->pov_count * 4 : 0),
                        10);
        adi_append_bits(packet, &packet_bits, profile->axis_count, 4);
        adi_append_bits(packet, &packet_bits,
                        profile->button_count + ((profile->pad_index >= 0) ? 4 : 0), 6);
        adi_append_bits(packet, &packet_bits,
                        ((profile->pov_count && (profile->pad_index < 0)) ? 8 : 0), 6);
        adi_append_bits(packet, &packet_bits, 0, 6);
        adi_append_bits(packet, &packet_bits,
                        (profile->pad_index < 0 && profile->pov_count)
                            ? profile->pov_count - 1
                            : 0,
                        4);
        adi_append_bits(packet, &packet_bits, 0, 4);
        adi_append_bits(packet, &packet_bits, 0, 4);
    } else {
        adi_append_bits(packet, &packet_bits, profile->id, 8);

        for (uint8_t axis = 0; axis < profile->axis_count; axis++)
            adi_append_bits(packet, &packet_bits, adi_axis_to_10bit(state->axis[axis]), 10);

        for (uint8_t button = 0; button < profile->button_count; button++) {
            if (button == profile->pad_index)
                adi_append_bits(packet, &packet_bits, adi_hat_buttons(state->pov[0]), 4);

            adi_append_bits(packet, &packet_bits, !!state->button[button], 1);
        }

        if (profile->pad_index >= profile->button_count)
            adi_append_bits(packet, &packet_bits, adi_hat_buttons(state->pov[0]), 4);

        if (profile->pad_index < 0) {
            for (uint8_t pov = 0; pov < profile->pov_count; pov++)
                adi_append_bits(packet, &packet_bits, adi_hat_direction(state->pov[pov]), 4);
        }
    }

    return packet_bits;
}

static void
adi_queue_lane(adi_data *adi, uint8_t lane, uint128_t packet, uint8_t first_bit, uint8_t bit_count,
               uint8_t marker)
{
    adi->packet[lane] = packet;
    adi->packet_mask[lane] = bit_count ? (uint128_t) 1 << first_bit : 0;
    adi->packet_bits[lane] = bit_count;
    adi->marker_bits[lane] = marker;
    adi->marker_pending[lane] = 1;
}

static void
adi_start_packets(adi_data *adi, int send_id)
{
    const int first_present  = adi_has_input(0);
    const int second_present = adi_has_input(1);

    memset(adi->packet_bits, 0, sizeof(adi->packet_bits));
    memset(adi->marker_pending, 0, sizeof(adi->marker_pending));
    adi->button_lines[0] = adi->button_lines[1] = 3;

    if (first_present && second_present) {
        for (uint8_t joystick = 0; joystick < 2; joystick++) {
            uint128_t packet;
            uint8_t   packet_bits = adi_build_packet(adi, joystick, send_id, &packet);

            adi_queue_lane(adi, joystick, packet, packet_bits - 1, packet_bits, 1);
        }
    } else if (first_present) {
        uint128_t packet;
        uint8_t   packet_bits = adi_build_packet(adi, 0, send_id, &packet);
        uint8_t   first_lane_bits = (packet_bits + 1) / 2;
        uint8_t   second_lane_bits = packet_bits / 2;

        adi_queue_lane(adi, 0, packet, packet_bits - 1, first_lane_bits, 1);
        adi_queue_lane(adi, 1, packet, second_lane_bits - 1, second_lane_bits, 0);
    } else if (second_present) {
        uint128_t packet;
        uint8_t   packet_bits = adi_build_packet(adi, 1, send_id, &packet);

        adi_queue_lane(adi, 1, packet, packet_bits - 1, packet_bits, 1);
    } else
        return;

    timer_set_delay_u64(&adi->packet_timer, TIMER_USEC * ADI_CLOCK_HALF_PERIOD_US);
}

static void
adi_append_interval(adi_data *adi, uint8_t interval)
{
    if (adi->sequence_count == ADI_SEQUENCE_LENGTH) {
        memmove(adi->sequence, adi->sequence + 1, ADI_SEQUENCE_LENGTH - 1);
        adi->sequence_count--;
    }

    adi->sequence[adi->sequence_count++] = interval;

    if ((adi->sequence_count == ADI_SEQUENCE_LENGTH) &&
        !memcmp(adi->sequence, adi_activation_sequence, ADI_SEQUENCE_LENGTH)) {
        adi->digital_mode = 1;
        adi->sequence_count = 0;
        timer_set_delay_u64(&adi->reset_timer, TIMER_USEC * ADI_PACKET_TIMEOUT_US);
        adi_start_packets(adi, 1);
    }
}

static void
adi_write(void *priv)
{
    adi_data *adi = (adi_data *) priv;

    if (adi->digital_mode) {
        timer_set_delay_u64(&adi->reset_timer, TIMER_USEC * ADI_PACKET_TIMEOUT_US);
        adi_start_packets(adi, 0);
        return;
    }

    if (adi->has_last_write) {
        uint64_t remaining = timer_get_remaining_us(&adi->interval_timer);
        uint64_t elapsed   = timer_is_enabled(&adi->interval_timer)
                                 ? ADI_INTERVAL_TIMEOUT_US - remaining
                                 : ADI_INTERVAL_TIMEOUT_US;
        uint64_t interval  = (elapsed < ADI_INTERVAL_TIMEOUT_US) ? (elapsed / 1000) : 255;

        adi_append_interval(adi, (uint8_t) interval);
    }

    adi->has_last_write = 1;
    timer_set_delay_u64(&adi->interval_timer, TIMER_USEC * ADI_INTERVAL_TIMEOUT_US);
}

#define ADI_AXIS_NAMES { "X axis", "Y axis", "Z axis", "R axis", "U axis", "V axis" }
#define ADI_BUTTON_NAMES { "Button 1", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6", \
                           "Button 7", "Button 8", "Button 9", "Button 10", "Button 11" }
#define ADI_POV_NAMES { "POV 1", "POV 2", "POV 3" }

#define JOYSTICK_ADI_COMMON \
    .close = adi_close, \
    .read = adi_read, \
    .write = adi_write, \
    .read_axis = adi_read_axis, \
    .a0_over = adi_a0_over, \
    .button_names = ADI_BUTTON_NAMES, \
    .pov_names = ADI_POV_NAMES

#define JOYSTICK_ADI_WINGMAN_EXTREME_DIGITAL \
    .init = adi_init_0, JOYSTICK_ADI_COMMON
#define JOYSTICK_ADI_THUNDERPAD_DIGITAL \
    .init = adi_init_1, JOYSTICK_ADI_COMMON
#define JOYSTICK_ADI_SIDECAR \
    .init = adi_init_2, JOYSTICK_ADI_COMMON
#define JOYSTICK_ADI_CYBERMAN_2 \
    .init = adi_init_3, JOYSTICK_ADI_COMMON
#define JOYSTICK_ADI_WINGMAN_INTERCEPTOR \
    .init = adi_init_4, JOYSTICK_ADI_COMMON
#define JOYSTICK_ADI_WINGMAN_FORMULA \
    .init = adi_init_5, JOYSTICK_ADI_COMMON
#define JOYSTICK_ADI_WINGMAN_GAMEPAD \
    .init = adi_init_6, JOYSTICK_ADI_COMMON
#define JOYSTICK_ADI_WINGMAN_EXTREME_DIGITAL_3D \
    .init = adi_init_7, JOYSTICK_ADI_COMMON
#define JOYSTICK_ADI_WINGMAN_GAMEPAD_EXTREME \
    .init = adi_init_8, JOYSTICK_ADI_COMMON

const joystick_t joystick_logitech_wingman = {
    .name = "Logitech WingMan Extreme Digital",
    .internal_name = "logitech_wingman_extreme_digital",
    JOYSTICK_ADI_WINGMAN_EXTREME_DIGITAL,
    .axis_count = 3, .button_count = 6, .pov_count = 1, .max_joysticks = 2,
    .axis_names = ADI_AXIS_NAMES
};

const joystick_t joystick_logitech_thunderpad = {
    .name = "Logitech ThunderPad Digital",
    .internal_name = "logitech_thunderpad_digital",
    JOYSTICK_ADI_THUNDERPAD_DIGITAL,
    .axis_count = 2, .button_count = 7, .pov_count = 1, .max_joysticks = 2,
    .axis_names = ADI_AXIS_NAMES
};

const joystick_t joystick_logitech_sidecar = {
    .name = "Logitech SideCar",
    .internal_name = "logitech_sidecar",
    JOYSTICK_ADI_SIDECAR,
    .axis_count = 6, .button_count = 8, .pov_count = 0, .max_joysticks = 2,
    .axis_names = ADI_AXIS_NAMES
};

const joystick_t joystick_logitech_cyberman_2 = {
    .name = "Logitech CyberMan 2",
    .internal_name = "logitech_cyberman_2",
    JOYSTICK_ADI_CYBERMAN_2,
    .axis_count = 6, .button_count = 8, .pov_count = 0, .max_joysticks = 2,
    .axis_names = ADI_AXIS_NAMES
};

const joystick_t joystick_logitech_wingman_interceptor = {
    .name = "Logitech WingMan Interceptor",
    .internal_name = "logitech_wingman_interceptor",
    JOYSTICK_ADI_WINGMAN_INTERCEPTOR,
    .axis_count = 3, .button_count = 9, .pov_count = 3, .max_joysticks = 2,
    .axis_names = ADI_AXIS_NAMES
};

const joystick_t joystick_logitech_wingman_formula = {
    .name = "Logitech WingMan Formula",
    .internal_name = "logitech_wingman_formula",
    JOYSTICK_ADI_WINGMAN_FORMULA,
    .axis_count = 3, .button_count = 8, .pov_count = 3, .max_joysticks = 2,
    .axis_names = { "Wheel", "Gas", "Brake" }
};

const joystick_t joystick_logitech_wingman_gamepad = {
    .name = "Logitech WingMan GamePad",
    .internal_name = "logitech_wingman_gamepad",
    JOYSTICK_ADI_WINGMAN_GAMEPAD,
    .axis_count = 2, .button_count = 7, .pov_count = 1, .max_joysticks = 2,
    .axis_names = ADI_AXIS_NAMES
};

const joystick_t joystick_logitech_wingman_extreme_3d = {
    .name = "Logitech WingMan Extreme Digital 3D",
    .internal_name = "logitech_wingman_extreme_digital_3d",
    JOYSTICK_ADI_WINGMAN_EXTREME_DIGITAL_3D,
    .axis_count = 4, .button_count = 7, .pov_count = 1, .max_joysticks = 2,
    .axis_names = ADI_AXIS_NAMES
};

const joystick_t joystick_logitech_wingman_gamepad_extreme = {
    .name = "Logitech WingMan GamePad Extreme",
    .internal_name = "logitech_wingman_gamepad_extreme",
    JOYSTICK_ADI_WINGMAN_GAMEPAD_EXTREME,
    .axis_count = 2, .button_count = 11, .pov_count = 1, .max_joysticks = 2,
    .axis_names = ADI_AXIS_NAMES
};
