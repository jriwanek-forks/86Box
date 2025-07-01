/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 * running old operating systems and software designed for IBM
 * PC systems and compatibles from 1981 through fairly recent
 * system designs based on the PCI bus.
 *
 * This file is part of the 86Box distribution.
 *
 * Implementation of the Gravis GamePad Pro.
 *
 * Notes:   The Gravis GamePad Pro uses the proprietary Gravis GrIP protocol
 * over the standard PC gameport to achieve more buttons and
 * a true digital D-pad (POV hat) than the standard gameport
 * natively supports. This involves specific write sequences from
 * the PC and timed bit-stream responses from the gamepad.
 * This implementation is based on the general principles seen in
 * the SideWinder Pad emulation (joystick_sw_pad.c) due to their
 * shared characteristic of using proprietary digital protocols
 * over the gameport, rather than simple analog reads.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com> (initial structure based on existing 86Box files)
 * Original Gravis GrIP protocol reverse-engineered by others (details not available here).
 *
 * Copyright 2021-2025 Jasmine Iwanek.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free  Software  Foundation; either  version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is  distributed in the hope that it will be useful, but
 * WITHOUT   ANY  WARRANTY;  without  even   the  implied  warranty  of
 * MERCHANTABILITY  or FITNESS  FOR A PARTICULAR  PURPOSE. See  the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the:
 *
 * Free Software Foundation, Inc.
 * 59 Temple Place - Suite 330
 * Boston, MA 02111-1307
 * USA.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/gameport.h>
#include <86box/plat_unused.h> // For UNUSED macro.

// Forward declaration of joystick_if_t structure (defined in gameport.h).
typedef struct joystick_if_t joystick_if_t;

/**
 * @brief Private data structure for the Gravis GamePad Pro emulation.
 *
 * This structure holds state relevant to the Gravis GrIP protocol's
 * packet-based communication, including timers for bit sequencing and
 * the data being transmitted/received.
 */
typedef struct grav_data {
    pc_timer_t poll_timer;    /**< Timer for controlling the bit-stream polling rate. */
    int        poll_left;     /**< Number of bits/phases left to poll in current packet. */
    int        poll_clock;    /**< Simulated clock line for the proprietary protocol. */
    uint64_t   current_packet_data; /**< The raw data bits being sent by the gamepad. */
    int        protocol_mode; /**< Current protocol mode (e.g., for different packet types). */

    pc_timer_t handshake_timer; /**< Timer for detecting handshake/ID request timings. */
    int        id_request_pending; /**< Flag to indicate if an ID request is expected. */
} grav_data;

/**
 * @brief Timer callback for the Gravis GrIP polling sequence.
 *
 * This function simulates the bit-by-bit data transfer of the proprietary
 * protocol by advancing the internal clock and shifting out bits from
 * `current_packet_data`. The specific timing and bit-shifting logic
 * would be determined by reverse-engineering the actual GrIP protocol.
 *
 * @param priv Pointer to the `grav_data` private structure.
 */
static void
grav_timer_over(void *priv)
{
    grav_data *grav = (grav_data *) priv;

    // Simulate clock line toggling, crucial for serial protocols.
    grav->poll_clock = !grav->poll_clock;

    if (grav->poll_clock) {
        // Shift out bits of the current data packet.
        // The exact shift amount depends on the GrIP's bit transfer rate.
        grav->current_packet_data >>= 1; // Example: shift one bit per clock cycle.
        grav->poll_left--; // Decrement bits remaining.
    }

    // Schedule the next timer tick. Timing would be critical here for accuracy.
    if (grav->poll_left > 0) {
        // Adjust delay based on protocol requirements (e.g., 5us per bit).
        timer_set_delay_u64(&grav->poll_timer, TIMER_USEC * 5);
    } else {
        // Packet transfer complete, disable timer.
        timer_disable(&grav->poll_timer);
        grav->id_request_pending = 0; // Clear ID request after data transfer.
    }
}

/**
 * @brief Timer callback for Gravis GrIP ID/handshake timing.
 *
 * This timer is set after a write, and its expiration might trigger a
 * specific ID response sequence from the gamepad if certain conditions are met.
 *
 * @param priv Pointer to the `grav_data` private structure.
 */
static void
grav_handshake_timer_over(void *priv)
{
    grav_data *grav = (grav_data *) priv;
    timer_disable(&grav->handshake_timer);
    grav->id_request_pending = 0; // Handshake window closed.
}

/**
 * @brief Initializes the Gravis GamePad Pro interface.
 *
 * Allocates and initializes the private data structure `grav_data` and sets up
 * the necessary timers for protocol communication.
 *
 * @return void* A pointer to the allocated `grav_data` structure.
 */
static void *
grav_init(void)
{
    grav_data *grav = (grav_data *) calloc(1, sizeof(grav_data));
    if (grav == NULL) {
        // Handle allocation failure.
        return NULL;
    }

    timer_add(&grav->poll_timer, grav_timer_over, grav, 0);
    timer_add(&grav->handshake_timer, grav_handshake_timer_over, grav, 0);

    return grav;
}

/**
 * @brief Closes and cleans up resources for the Gravis GamePad Pro interface.
 *
 * Frees the allocated private data structure.
 *
 * @param priv Private data pointer to the `grav_data` structure.
 */
static void
grav_close(void *priv)
{
    grav_data *grav = (grav_data *) priv;
    if (grav) {
        free(grav);
    }
}

/**
 * @brief Reads the state of the gameport lines for Gravis GamePad Pro.
 *
 * This function is critical for the Gravis GrIP protocol. It simulates
 * the gamepad's response by setting the appropriate bits based on the
 * current state of the polling sequence. The PC's driver would read
 * these bits in a timed fashion to reconstruct the gamepad's full state.
 *
 * @param priv Private data pointer to the `grav_data` structure.
 * @return uint8_t A byte representing the gameport input lines (bits 4-7 for buttons).
 */
static uint8_t
grav_read(void *priv)
{
    grav_data *grav = (grav_data *) priv;
    uint8_t ret = 0xff; // All bits high (unpressed/inactive) by default.

    if (!JOYSTICK_PRESENT(0, 0)) {
        return 0xff; // No joystick connected, return default high.
    }

    // If a polling sequence is active, manipulate the button lines.
    // The specific bits driven low would depend on the Gravis protocol.
    if (timer_is_enabled(&grav->poll_timer)) {
        // Example: drive specific bits low based on `current_packet_data` and `poll_clock`.
        // This is where the actual Gravis GrIP data bits would be presented.
        // For demonstration, we'll mimic a simple serial output like Sidewinder.
        if (grav->poll_clock) {
            // If clock is high, drive B1 low (Pin 2)
            ret &= ~0x10;
        }

        // Present data bit on another line (e.g., B2/Pin 7)
        if (grav->current_packet_data & 1) {
            ret &= ~0x20; // Drive B2 low if current data bit is 1.
        }
        // Additional lines (B3/B4) could be used for clock, status, or other data bits.
        // This is highly speculative without the actual GrIP protocol.
        // e.g., if (grav->poll_clock) ret &= ~0x40; // Use B3 as another clock/data line.
    } else {
        // If no polling is active, revert to standard button states for axes.
        // This might be for initial detection or standard 4-button fallback.
        if (joystick_state[0][0].button[0]) ret &= ~0x10;
        if (joystick_state[0][0].button[1]) ret &= ~0x20;
        if (joystick_state[0][0].button[2]) ret &= ~0x40;
        if (joystick_state[0][0].button[3]) ret &= ~0x80;
    }

    return ret;
}

/**
 * @brief Handles write operations for Gravis GamePad Pro.
 *
 * For the Gravis GrIP protocol, write operations are crucial as they act
 * as commands or handshake signals from the PC to the gamepad. These writes
 * initiate the data polling sequences.
 *
 * @param priv Private data pointer to the `grav_data` structure.
 */
static void
grav_write(void *priv)
{
    grav_data *grav = (grav_data *) priv;

    if (!JOYSTICK_PRESENT(0, 0))
        return;

    // Check if a new polling sequence is starting (e.g., by checking if poll_left is 0).
    if (grav->poll_left == 0) {
        // This is a simplified example. A real GrIP protocol would involve checking
        // the address written to (0x201, 0x202, etc.) and specific data bits.

        // Assume any write to gameport initiates a new data packet transfer.
        // The exact packet size (e.g., 64 bits for all buttons/axes/POV) would be protocol-specific.
        // For demonstration, we'll create a dummy packet.

        grav->poll_clock = 1; // Start with clock high.
        grav->poll_left = 64; // Example: 64 bits of data.
        timer_set_delay_u64(&grav->poll_timer, TIMER_USEC * 10); // Initial delay.

        // Populate `current_packet_data` with the current joystick state.
        // This is where `joystick_state` (from host input) is encoded into the GrIP packet.
        uint64_t data_packet = 0;
        // Example: Encode D-pad (POV) state into first few bits.
        if (joystick_state[0][0].pov[0] != -1) {
            // For a digital D-pad, map angles to specific bit flags.
            // This is a simplified encoding example.
            if (joystick_state[0][0].pov[0] > 315 || joystick_state[0][0].pov[0] < 45) data_packet |= (1ULL << 0); // UP
            if (joystick_state[0][0].pov[0] >= 135 && joystick_state[0][0].pov[0] < 225) data_packet |= (1ULL << 1); // DOWN
            if (joystick_state[0][0].pov[0] >= 225 && joystick_state[0][0].pov[0] < 315) data_packet |= (1ULL << 2); // LEFT
            if (joystick_state[0][0].pov[0] >= 45 && joystick_state[0][0].pov[0] < 135) data_packet |= (1ULL << 3); // RIGHT
        }

        // Encode buttons (up to 10 for GamePad Pro).
        for (int i = 0; i < 10; i++) {
            if (joystick_state[0][0].button[i]) {
                data_packet |= (1ULL << (i + 4)); // Buttons after D-pad bits.
            }
        }

        // The Gravis GamePad Pro is primarily digital, its "axes" are typically
        // part of the digital protocol, not raw analog reads. We'll include X/Y
        // as 16-bit values.
        data_packet |= ((uint64_t)(joystick_state[0][0].axis[0] + 32768) & 0xFFFF) << 14; // X-axis
        data_packet |= ((uint64_t)(joystick_state[0][0].axis[1] + 32768) & 0xFFFF) << 30; // Y-axis

        grav->current_packet_data = data_packet; // Store the prepared packet.
        grav->id_request_pending = 1; // A handshake might be part of this write.
    }

    // Reset handshake timer. A new write might reset a timeout for ID request.
    timer_set_delay_u64(&grav->handshake_timer, TIMER_USEC * 10000); // Example timeout for ID request (10ms).
}

/**
 * @brief Reads a specific axis for the Gravis GamePad Pro.
 *
 * For the Gravis GamePad Pro, analog axis support might be limited or entirely
 * handled by the digital protocol. If a game attempts to read axes, this
 * function would provide appropriate (likely fixed or 0) values, or decode
 * axis data from the current digital packet if the protocol supports it.
 *
 * @param priv Private data pointer, unused.
 * @param axis The index of the axis to read.
 * @return int The axis value, or AXIS_NOT_PRESENT.
 */
static int
grav_read_axis(UNUSED(void *priv), UNUSED(int axis))
{
    // The Gravis GamePad Pro is primarily a digital gamepad.
    // Analog axis reads on these pins might return fixed values or 0,
    // or the driver might expect these to be part of the digital packet.
    if (!JOYSTICK_PRESENT(0, 0))
        return AXIS_NOT_PRESENT;

    // For a digital gamepad, axes (D-pad) are usually simulated with extreme values
    // if accessed directly as analog inputs.
    // Given its digital nature, returning 0 might be appropriate for un-mapped axes,
    // or specific values if a game expects "centered" on analog lines.
    return 0; // Return 0, as true analog read is not primary for this device.
}

/**
 * @brief Callback for Axis 0 (A0) timer overflow for Gravis GamePad Pro.
 *
 * This function is tied to the timing of the gameport's A0 line. For
 * proprietary protocols like GrIP, this might be used for specific handshake
 * or timing signals.
 *
 * @param priv Private data pointer to the `grav_data` structure.
 */
static void
grav_a0_over(void *priv)
{
    grav_data *grav = (grav_data *) priv;
    // When A0 overflows, it might be a trigger for the ID packet request or protocol state change.
    // If an ID request is pending, this timing might complete the handshake.
    if (grav->id_request_pending) {
        // This is where a specific ID response would be prepared if the GrIP
        // protocol expects it after a certain A0 timeout.
        // For now, simply disable the handshake timer.
        timer_disable(&grav->handshake_timer);
        grav->id_request_pending = 0;
    }
    // Set a new delay for the handshake timer, expecting another write soon.
    timer_set_delay_u64(&grav->handshake_timer, TIMER_USEC * 10000);
}

// =========================================================================
// Gravis GamePad Pro Definition
// =========================================================================

/**
 * @brief Defines the Gravis GamePad Pro joystick interface.
 *
 * This gamepad uses the proprietary Gravis GrIP protocol. It typically provides
 * digital X/Y (D-pad) input, 8-10 buttons, and a true POV hat (digital).
 * The emulation relies on complex timing and bit-stream decoding/encoding
 * within the `read` and `write` functions.
 *
 * DB-15 Pin Connections:
 * The GrIP protocol uses the standard gameport pins (X1, Y1, X2, Y2 for data,
 * and B1-B4 for control/clock/data) in a multiplexed, timed fashion.
 * Specific pin usage would depend on the exact protocol details, but it
 * leverages the existing infrastructure to transmit more data.
 *
 * X-Axis (D-Pad Horizontal): Controlled by protocol.
 * Y-Axis (D-Pad Vertical): Controlled by protocol.
 * Buttons: Multiple (e.g., 10) controlled by protocol.
 * POV Hat: Controlled by protocol.
 *
 * Note: The axis_count is set to 2 to represent the X/Y D-pad, but their
 * values are driven by the digital protocol, not direct analog reads.
 */
const joystick_if_t joystick_gravis_gamepad_pro = {
    .name          = "Gravis GamePad Pro",
    .internal_name = "gravis_gamepad_pro",
    .init          = grav_init,         // Custom initialization.
    .close         = grav_close,        // Custom cleanup.
    .read          = grav_read,         // Custom read for GrIP protocol.
    .write         = grav_write,        // Custom write for GrIP protocol.
    .read_axis     = grav_read_axis,    // Axis read for digital gamepad (might return 0).
    .a0_over       = grav_a0_over,      // Custom A0 overflow handling for handshake.
    .axis_count    = 2,                 // X, Y for D-pad (digital input).
    .button_count  = 10,                // Common for Gravis GamePad Pro.
    .pov_count     = 1,                 // Has a digital POV hat (D-pad).
    .max_joysticks = 1,                 // Typically a single device per port, or daisy-chained as one logical unit.
    .axis_names    = { "D-pad X", "D-pad Y" },
    .button_names  = { "Button 1", "Button 2", "Button 3", "Button 4",
                       "Button 5", "Button 6", "Button 7", "Button 8",
                       "Button 9", "Button 10" },
    .pov_names     = { "D-Pad" }
};
