/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the Logitech WingMan Extreme Digital.
 *
 * Notes:   The digital-mode activation sequence and packet layout are
 *          based on UniPCemu's joystick implementation and the Logitech
 *          ADI protocol.
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
 * WITHOUT   ANY  WARRANTY;  without even   the implied warranty of
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
#include <86box/gameport.h>
#include <86box/timer.h>

#define WINGMAN_INTERVAL_TIMEOUT_US 1000000
#define WINGMAN_PACKET_TIMEOUT_US    200000
#define WINGMAN_CLOCK_HALF_PERIOD_US      5
#define WINGMAN_SEQUENCE_LENGTH           9

typedef struct wingman_data {
    pc_timer_t interval_timer;
    pc_timer_t packet_timer;
    pc_timer_t reset_timer;
    uint64_t   packet;
    uint64_t   high_mask;
    uint32_t   low_mask;
    uint8_t    sequence[WINGMAN_SEQUENCE_LENGTH];
    uint8_t    sequence_count;
    uint8_t    button_lines[2];
    uint8_t    digital_mode;
    uint8_t    has_last_write;
} wingman_data;

static const uint8_t wingman_activation_sequence[WINGMAN_SEQUENCE_LENGTH] = {
    4, 2, 3, 10, 6, 11, 7, 9, 11
};

static void
wingman_interval_timer_over(void *priv)
{
    (void) priv;
}

static void
wingman_packet_timer_over(void *priv)
{
    wingman_data *wingman = (wingman_data *) priv;

    if (!wingman->digital_mode || !wingman->low_mask)
        return;

    if (wingman->packet & wingman->low_mask)
        wingman->button_lines[1] ^= 2;
    else
        wingman->button_lines[1] ^= 1;

    if (wingman->packet & wingman->high_mask)
        wingman->button_lines[0] ^= 2;
    else
        wingman->button_lines[0] ^= 1;

    wingman->low_mask >>= 1;
    wingman->high_mask >>= 1;

    if (wingman->low_mask)
        timer_advance_u64(&wingman->packet_timer, TIMER_USEC * WINGMAN_CLOCK_HALF_PERIOD_US);
}

static void
wingman_reset_timer_over(void *priv)
{
    wingman_data *wingman = (wingman_data *) priv;

    wingman->digital_mode = 0;
    wingman->sequence_count = 0;
    wingman->has_last_write = 0;
    wingman->low_mask = 0;
    wingman->high_mask = 0;
    timer_disable(&wingman->packet_timer);
}

static void *
wingman_init(void)
{
    wingman_data *wingman = (wingman_data *) calloc(1, sizeof(wingman_data));

    timer_add(&wingman->interval_timer, wingman_interval_timer_over, wingman, 0);
    timer_add(&wingman->packet_timer, wingman_packet_timer_over, wingman, 0);
    timer_add(&wingman->reset_timer, wingman_reset_timer_over, wingman, 0);

    return wingman;
}

static void
wingman_close(void *priv)
{
    wingman_data *wingman = (wingman_data *) priv;

    timer_disable(&wingman->interval_timer);
    timer_disable(&wingman->packet_timer);
    timer_disable(&wingman->reset_timer);
    free(wingman);
}

static int
wingman_has_input(void)
{
    return JOYSTICK_PRESENT(0, 0);
}

static uint8_t
wingman_read(void *priv)
{
    wingman_data *wingman = (wingman_data *) priv;

    if (!wingman_has_input())
        return 0xff;

    if (wingman->digital_mode)
        return (uint8_t) ((wingman->button_lines[0] << 4) | (wingman->button_lines[1] << 6));

    uint8_t result = 0xf0;
    for (uint8_t button = 0; button < 4; button++) {
        if (joystick_state[0][0].button[button])
            result &= (uint8_t) ~(0x10 << button);
    }

    return result;
}

static void
wingman_append_interval(wingman_data *wingman, uint8_t interval)
{
    if (wingman->sequence_count == WINGMAN_SEQUENCE_LENGTH) {
        memmove(wingman->sequence, wingman->sequence + 1, WINGMAN_SEQUENCE_LENGTH - 1);
        wingman->sequence_count--;
    }

    wingman->sequence[wingman->sequence_count++] = interval;

    if (wingman->sequence_count == WINGMAN_SEQUENCE_LENGTH &&
        !memcmp(wingman->sequence, wingman_activation_sequence, WINGMAN_SEQUENCE_LENGTH)) {
        wingman->digital_mode = 1;
        wingman->sequence_count = 0;
        timer_set_delay_u64(&wingman->reset_timer, TIMER_USEC * WINGMAN_PACKET_TIMEOUT_US);
    }
}

static uint64_t
wingman_axis_byte(int axis)
{
    return (uint64_t) (((uint16_t) axis) >> 8);
}

static uint8_t
wingman_hat_bits(int angle)
{
    uint8_t bits = 0;

    if ((angle >= 315) || (angle <= 45))
        bits |= 1 << 2;
    if ((angle >= 45) && (angle <= 135))
        bits |= 1 << 1;
    if ((angle >= 135) && (angle <= 225))
        bits |= 1 << 3;
    if ((angle >= 225) && (angle <= 315))
        bits |= 1;

    return bits;
}

static void
wingman_start_packet(wingman_data *wingman)
{
    const joystick_state_t *state = &joystick_state[0][0];
    uint8_t                 hat   = (state->pov[0] >= 0) ? wingman_hat_bits(state->pov[0]) : 0;

    wingman->packet = hat;
    for (uint8_t button = 0; button < 6; button++) {
        if (state->button[button])
            wingman->packet |= (uint64_t) 1 << (9 - button);
    }

    wingman->packet |= wingman_axis_byte(state->axis[2]) << 10;
    wingman->packet |= wingman_axis_byte(state->axis[1]) << 18;
    wingman->packet |= wingman_axis_byte(state->axis[0]) << 26;

    wingman->button_lines[0] = 0;
    wingman->button_lines[1] = 0;
    wingman->high_mask       = (uint64_t) 1 << 41;
    wingman->low_mask        = (uint32_t) 1 << 20;
    timer_set_delay_u64(&wingman->packet_timer, TIMER_USEC * WINGMAN_CLOCK_HALF_PERIOD_US);
}

static void
wingman_write(void *priv)
{
    wingman_data *wingman = (wingman_data *) priv;

    if (wingman->digital_mode) {
        timer_set_delay_u64(&wingman->reset_timer, TIMER_USEC * WINGMAN_PACKET_TIMEOUT_US);
        wingman_start_packet(wingman);
        return;
    }

    if (wingman->has_last_write) {
        uint64_t remaining = timer_get_remaining_us(&wingman->interval_timer);
        uint64_t elapsed   = (remaining < WINGMAN_INTERVAL_TIMEOUT_US)
                                 ? (WINGMAN_INTERVAL_TIMEOUT_US - remaining)
                                 : WINGMAN_INTERVAL_TIMEOUT_US;
        uint64_t interval  = (elapsed < WINGMAN_INTERVAL_TIMEOUT_US) ? (elapsed / 1000) : 255;

        wingman_append_interval(wingman, (uint8_t) interval);
    }

    wingman->has_last_write = 1;
    timer_set_delay_u64(&wingman->interval_timer, TIMER_USEC * WINGMAN_INTERVAL_TIMEOUT_US);
}

static int
wingman_read_axis(void *priv, int axis)
{
    wingman_data *wingman = (wingman_data *) priv;

    if ((axis < 0) || (axis >= 4) || !wingman_has_input() || wingman->digital_mode)
        return AXIS_NOT_PRESENT;

    if (axis < 3)
        return joystick_state[0][0].axis[axis];

    return AXIS_NOT_PRESENT;
}

static void
wingman_a0_over(void *priv)
{
    (void) priv;
}

const joystick_t joystick_logitech_wingman = {
    .name          = "Logitech WingMan Extreme Digital",
    .internal_name = "logitech_wingman_extreme_digital",
    .init          = wingman_init,
    .close         = wingman_close,
    .read          = wingman_read,
    .write         = wingman_write,
    .read_axis     = wingman_read_axis,
    .a0_over       = wingman_a0_over,
    .axis_count    = 3,
    .button_count  = 6,
    .pov_count     = 1,
    .max_joysticks = 1,
    .axis_names    = { "X axis", "Y axis", "Twist" },
    .button_names  = { "Button 1", "Button 2", "Button 3", "Button 4", "Button 5", "Button 6" },
    .pov_names     = { "Hat" }
};
