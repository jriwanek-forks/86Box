/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the Gravis GamePad Pro and other Gravis GrIP devices.
 *
 * Notes:   The Gravis GamePad Pro uses the Gravis GrIP protocol
 *          over the standard PC gameport to achieve more buttons and
 *          a true digital D-pad (POV hat) than the standard gameport
 *          natively supports. This involves specific write sequences from
 *          the PC and timed bit-stream responses from the gamepad.
 *
 *          This implementation is based on the Linux kernel's `grip.c`
 *          driver.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2021-2025 Jasmine Iwanek.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/gameport.h>
#include <86box/plat_unused.h>

/**
 * @brief Private data structure for Gravis GrIP device emulation.
 *
 * This structure holds state relevant to the Gravis GrIP protocol's
 * packet-based communication, including timers for bit sequencing and
 * the combined data packet being transmitted.
 */
typedef struct grav_data {
    pc_timer_t poll_timer;           /**< Timer for controlling the bit-stream polling rate. */
    int        poll_bit_pos;         /**< Current bit position within the combined packet (0 to total_bits-1). */
    int        poll_clock;           /**< Simulated clock line state (used for specific output pins). 0=Low, 1=High. */
    uint8_t    packet_buffer[4 * 3]; /**< Buffer for up to 4 joysticks * 3 bytes/joystick = 12 bytes total (96 bits).
                                       * Note: GPP mode supports 2 joysticks, so only 6 bytes are used. */

    pc_timer_t handshake_timer;      /**< Timer for detecting handshake/ID request timings. */
    int        id_request_pending;   /**< Flag to indicate if an ID request is expected. */
} grav_data;

/**
 * @brief Timer callback for the Gravis GrIP polling sequence.
 *
 * This function simulates the bit-by-bit data transfer of the proprietary
 * protocol. It toggles a simulated clock line and advances the `poll_bit_pos`
 * to point to the next bit in the `packet_buffer` to be read by the emulated
 * PC's gameport driver.
 *
 * The total protocol packet length is 24 bits * max_joysticks for GPP mode.
 *
 * @param priv Pointer to the `grav_data` private structure.
 */
static void
grav_timer_over(void *priv)
{
    grav_data *grav = (grav_data *) priv;

    // The Linux `grip.c` driver's `grip_gpp_read_packet` implies that data is latched
    // when the clock line transitions from high to low.
    // We'll advance our bit position when our simulated clock goes high.
    grav->poll_clock = !grav->poll_clock; // Toggle simulated clock state (0=Low, 1=High).

    if (grav->poll_clock) { // Advance bit when clock goes high (start of a bit period).
        grav->poll_bit_pos++; // Move to the next bit in the combined stream.
    }

    // Schedule the next timer tick. Precise timing is critical for protocol accuracy.
    // Each clock phase (low or high) is assumed to take TIMER_USEC * 5.
    if (grav->poll_bit_pos < (joystick_gravis_gamepad_pro.max_joysticks * 24)) {
        timer_set_delay_u64(&grav->poll_timer, TIMER_USEC * 5); // 5us per clock phase.
    } else {
        // Packet transfer complete, disable timer.
        timer_disable(&grav->poll_timer);
        grav->id_request_pending = 0; // Clear ID request after data transfer completes.
        grav->poll_bit_pos = 0; // Reset bit position for the next packet request.
    }
}

/**
 * @brief Timer callback for Gravis GrIP ID/handshake timing.
 *
 * This timer is set after a write operation. If its expiration coincides with
 * a specific handshake sequence, it might trigger the gamepad to send its ID.
 *
 * @param priv Pointer to the `grav_data` private structure.
 */
static void
grav_handshake_timer_over(UNUSED(void *priv))
{
    grav_data *grav = (grav_data *) priv;
    timer_disable(&grav->handshake_timer);
    grav->id_request_pending = 0; // Handshake window closed or timed out.
}

/**
 * @brief Initializes the Gravis GrIP device interface.
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
    if (grav == NULL)
        return NULL;

    timer_add(&grav->poll_timer, grav_timer_over, grav, 0);
    timer_add(&grav->handshake_timer, grav_handshake_timer_over, grav, 0);

    return grav;
}

/**
 * @brief Closes and cleans up resources for the Gravis GrIP device interface.
 *
 * Frees the allocated private data structure.
 *
 * @param priv Private data pointer to the `grav_data` structure.
 */
static void
grav_close(void *priv)
{
    grav_data *grav = (grav_data *) priv;
    if (grav)
        free(grav);
}

/**
 * @brief Reads the state of the gameport lines for Gravis GrIP devices (GPP Mode).
 *
 * This function simulates the gamepad's response to the PC's polling.
 * It sets the appropriate bits on the gameport input lines (bits 4-7 of the
 * status byte) based on the current state of the serial polling sequence
 * (`grav->poll_clock` and the current bit from `grav->packet_buffer`).
 *
 * Based on the `grip.c` Linux driver for GamePad Pro:
 * - For joystick 0 (GrIP device 0): Pin 2 (B1, Bit 4) is the Clock, Pin 7 (B2, Bit 5) is the Data.
 * - For joystick 1 (GrIP device 1): Pin 10 (B3, Bit 6) is the Clock, Pin 14 (B4, Bit 7) is the Data.
 * - A logical '1' on the data line is represented by a LOW signal (inverted logic).
 *
 * @param priv Private data pointer to the `grav_data` structure.
 * @return uint8_t A byte representing the gameport input lines (bits 4-7 for buttons).
 */
static uint8_t
grav_read_gpp(void *priv)
{
    grav_data *grav = (grav_data *) priv;
    // Default state: all button lines high (unpressed/inactive), matching 0xF0 for gameport.
    uint8_t ret = 0xf0;

    if (!JOYSTICK_PRESENT(0, 0))
        return 0xff;

    // If a polling sequence is active (timer enabled), manipulate button lines
    // according to the GrIP protocol's serial data transfer.
    if (timer_is_enabled(&grav->poll_timer)) {
        // Determine which joystick's data we are currently outputting.
        // The GPP protocol has a 24-bit packet per joystick.
        int current_js = grav->poll_bit_pos / 24;

        // Ensure we don't read beyond the maximum supported joysticks.
        if (current_js >= joystick_gravis_gamepad_pro.max_joysticks)
            return 0xf0; // Default high if out of bounds.

        // Get the current bit from the packet_buffer for the active joystick.
        // The index within the 24-bit chunk for the current joystick.
        int bit_in_chunk = grav->poll_bit_pos % 24;
        // The byte index within the `packet_buffer` for the current joystick.
        int byte_idx = (current_js * 3) + (bit_in_chunk / 8);
        // The bit position within that byte.
        int bit_in_byte = bit_in_chunk % 8;

        uint8_t current_bit = (grav->packet_buffer[byte_idx] >> bit_in_byte) & 1;

        // Simulate the clock and data lines based on the current joystick's output pins.
        if (current_js == 0) { // First GamePad Pro (uses B1/B2 lines)
            // Clock on Pin 2 (B1, Bit 4). Drive low when simulated clock is 0 (LOW state).
            // This aligns with `grip.c`'s `(~v & u & 1)` where `u` is previous and `v` is current.
            // If `poll_clock` is 0 (current state is low), then drive the line low.
            if (!grav->poll_clock)
                ret &= ~0x10; // Clear bit 4 (0x10) to make Pin 2 low.

            // Data on Pin 7 (B2, Bit 5). Drive low if current_bit is 1 (inverted logic).
            if (current_bit == 1)
                ret &= ~0x20; // Clear bit 5 (0x20) to make Pin 7 low.

        } else if (current_js == 1) { // Second GamePad Pro (uses B3/B4 lines)
            // Clock on Pin 10 (B3, Bit 6). Drive low when simulated clock is 0.
            if (!grav->poll_clock)
                ret &= ~0x40; // Clear bit 6 (0x40) to make Pin 10 low.

            // Data on Pin 14 (B4, Bit 7). Drive low if current_bit is 1 (inverted logic).
            if (current_bit == 1)
                ret &= ~0x80; // Clear bit 7 (0x80) to make Pin 14 low.
        }
        // Note: The Linux driver's (i << 1) + 4 shift implies only two distinct addressable devices
        // on the 4 button lines for the GPP protocol. So, max_joysticks = 2 is appropriate here.
    } else {
        // If no polling is active, the gamepad might revert to a default state,
        // or a standard 4-button fallback mode for initial detection.
        // This assumes a non-polling read defaults to standard buttons (B1-B4).
        if (joystick_state[0][0].button[0])
            ret &= ~0x10; // B1 (Pin 2)
        if (joystick_state[0][0].button[1])
            ret &= ~0x20; // B2 (Pin 7)
        if (joystick_state[0][0].button[2])
            ret &= ~0x40; // B3 (Pin 10)
        if (joystick_state[0][0].button[3])
            ret &= ~0x80; // B4 (Pin 14)
    }

    return ret;
}

/**
 * @brief Handles write operations for Gravis GrIP devices (GPP Mode).
 *
 * For the Gravis GrIP protocol, write operations from the PC to the gameport
 * (typically 0x201) are crucial as they act as commands or handshake signals,
 * initiating the data polling sequences from the gamepad.
 *
 * This function prepares the combined 96-bit data packet (4 joysticks * 24 bits/joystick)
 * by encoding the current state of all emulated joysticks into `packet_buffer`.
 *
 * @param priv Private data pointer to the `grav_data` structure.
 */
static void
grav_write_gpp(void *priv)
{
    grav_data *grav = (grav_data *) priv;

    if (!JOYSTICK_PRESENT(0, 0)) // Check if at least one joystick is connected on the host side.
        return;

    // When the PC writes to the gameport, it signals the start of a polling cycle.
    // This is where the gamepad would prepare its data packet.
    if (!timer_is_enabled(&grav->poll_timer)) { // Only start if no packet is currently being sent.
        grav->poll_clock = 0; // Start with clock low (initial state for first bit transfer).
        grav->poll_bit_pos = 0; // Reset bit position to the beginning of the combined packet.
        // Set an initial delay before the first bit transfer/clock pulse.
        timer_set_delay_u64(&grav->poll_timer, TIMER_USEC * 10); // Example initial delay to prepare.

        // Initialize `packet_buffer` to all zeros. Unmapped joysticks will send 0s.
        memset(grav->packet_buffer, 0, sizeof(grav->packet_buffer));

        // Iterate through each emulated joystick slot (up to `max_joysticks`) and encode its state.
        for (int js = 0; js < joystick_gravis_gamepad_pro.max_joysticks; js++) {
            // Only encode data for joystick slots that have a physical host joystick mapped.
            if (!joystick_state[0][js].plat_joystick_nr) {
                 // If no physical joystick is mapped to this emulated slot, its 24-bit section
                 // in `packet_buffer` will remain zero (from the initial memset),
                 // effectively sending a "blank" or inactive state for that gamepad.
                 continue;
            }

            // --- Encode 24-bit packet for current joystick (js) into `single_joystick_packet` ---
            uint64_t single_joystick_packet = 0; // Use a temporary 64-bit var to build the 24-bit packet.

            // Start/framing bits (Bit 1-7 in image, corresponding to indices 0-6 in a 0-indexed 24-bit packet).
            // Values: 0111110 (LSB first for shifting, so bit 0:0, 1:1, 2:1, 3:1, 4:1, 5:1, 6:0).
            // NOTE: The Linux driver's sync word `0x7c0000` corresponds to these bits (0111110),
            // but the `0xfe4210` mask and XOR operation suggest a more complex scrambling.
            // For now, we encode directly as per the image. Future refinement might require
            // pre-scrambling `single_joystick_packet` if the Linux driver's sync word is definitive.
            single_joystick_packet |= (0ULL << 0); // Bit 1: 0
            single_joystick_packet |= (1ULL << 1); // Bit 2: 1
            single_joystick_packet |= (1ULL << 2); // Bit 3: 1
            single_joystick_packet |= (1ULL << 3); // Bit 4: 1
            single_joystick_packet |= (1ULL << 4); // Bit 5: 1
            single_joystick_packet |= (1ULL << 5); // Bit 6: 1
            single_joystick_packet |= (0ULL << 6); // Bit 7: 0

            // Digital buttons (based on joystick_gravis_gamepad_pro.button_names mapping).
            // Button states are 1 if pressed, 0 if not.
            // Bit 8 (index 7): Select -> joystick_state[0][js].button[0]
            if (joystick_state[0][js].button[0]) single_joystick_packet |= (1ULL << 7);
            // Bit 9 (index 8): Start -> joystick_state[0][js].button[1]
            if (joystick_state[0][js].button[1]) single_joystick_packet |= (1ULL << 8);
            // Bit 10 (index 9): R2 -> joystick_state[0][js].button[2]
            if (joystick_state[0][js].button[2]) single_joystick_packet |= (1ULL << 9);
            // Bit 11 (index 10): Blue -> joystick_state[0][js].button[3]
            if (joystick_state[0][js].button[3]) single_joystick_packet |= (1ULL << 10);

            // Frame bit (Bit 12 in image, index 11 in packet) is 0.
            // (No explicit setting needed as `single_joystick_packet` starts at 0).

            // Digital buttons continued
            // Bit 13 (index 12): L2 -> joystick_state[0][js].button[4]
            if (joystick_state[0][js].button[4]) single_joystick_packet |= (1ULL << 12);
            // Bit 14 (index 13): Green -> joystick_state[0][js].button[5]
            if (joystick_state[0][js].button[5]) single_joystick_packet |= (1ULL << 13);
            // Bit 15 (index 14): Yellow -> joystick_state[0][js].button[6]
            if (joystick_state[0][js].button[6]) single_joystick_packet |= (1ULL << 14);
            // Bit 16 (index 15): Red -> joystick_state[0][js].button[7]
            if (joystick_state[0][js].button[7]) single_joystick_packet |= (1ULL << 15);

            // Frame bit (Bit 17 in image, index 16 in packet) is 0.

            // Digital buttons continued
            // Bit 18 (index 17): L1 -> joystick_state[0][js].button[8]
            if (joystick_state[0][js].button[8]) single_joystick_packet |= (1ULL << 17);
            // Bit 19 (index 18): R1 -> joystick_state[0][js].button[9]
            if (joystick_state[0][js].button[9]) single_joystick_packet |= (1ULL << 18);

            // Frame bit (Bit 20 in image, index 19 in packet) is 0.

            // D-pad (POV) bits (Bit 21-24 in image, indices 20-23 in packet)
            // Convert POV angle (0-359 degrees) to individual Up/Down/Right/Left bits.
            // Assuming 0=Up, 90=Right, 180=Down, 270=Left for `joystick_state.pov`.
            if (joystick_state[0][js].pov[0] != -1) {
                double angle = (double)joystick_state[0][js].pov[0];
                // Up (Bit 21, index 20)
                if (angle >= 337.5 || angle < 22.5) single_joystick_packet |= (1ULL << 20);
                // Down (Bit 22, index 21)
                if (angle >= 157.5 && angle < 202.5) single_joystick_packet |= (1ULL << 21);
                // Right (Bit 23, index 22)
                if (angle >= 67.5 && angle < 112.5) single_joystick_packet |= (1ULL << 22);
                // Left (Bit 24, index 23)
                if (angle >= 247.5 && angle < 292.5) single_joystick_packet |= (1ULL << 23);

                // Handle diagonals (set both relevant bits if applicable)
                if (angle >= 22.5 && angle < 67.5) { single_joystick_packet |= (1ULL << 20) | (1ULL << 22); } // Up-Right
                if (angle >= 112.5 && angle < 157.5) { single_joystick_packet |= (1ULL << 21) | (1ULL << 22); } // Down-Right
                if (angle >= 202.5 && angle < 247.5) { single_joystick_packet |= (1ULL << 21) | (1ULL << 23); } // Down-Left
                if (angle >= 292.5 && angle < 337.5) { single_joystick_packet |= (1ULL << 20) | (1ULL << 23); } // Up-Left
            }

            // Copy the completed 24-bit packet (3 bytes) into the larger `packet_buffer`.
            // Each joystick's data is concatenated.
            int buffer_offset_bytes = js * 3; // Offset for current joystick's data.
            grav->packet_buffer[buffer_offset_bytes + 0] = (uint8_t)(single_joystick_packet & 0xFF);
            grav->packet_buffer[buffer_offset_bytes + 1] = (uint8_t)((single_joystick_packet >> 8) & 0xFF);
            grav->packet_buffer[buffer_offset_bytes + 2] = (uint8_t)((single_joystick_packet >> 16) & 0xFF);
        }
    }

    timer_set_delay_u64(&grav->handshake_timer, TIMER_USEC * 10000); // Example timeout (10ms).
}

/**
 * @brief Reads a specific axis for the Gravis GrIP devices.
 *
 * Gravis GrIP gamepads are primarily digital. Their D-pad input is
 * communicated via the proprietary digital protocol, not direct analog axis
 * readings on the gameport's potentiometer pins. Therefore, any attempts by
 * the emulated software to read these pins as analog axes should result in
 * a fixed (e.g., centered) value, or 0.
 *
 * @param priv Private data pointer, unused.
 * @param axis The index of the axis to read.
 * @return int The axis value (typically 0 for digital gamepads), or AXIS_NOT_PRESENT.
 */
static int
grav_read_axis(UNUSED(void *priv), UNUSED(int axis))
{
    if (!JOYSTICK_PRESENT(0, 0))
        return AXIS_NOT_PRESENT;

    return 0;
}

/**
 * @brief Placeholder for `read`, `write`, `read_axis`, `a0_over` for XT/BD/DC modes.
 *
 * These functions serve as **minimal placeholders for the complex XT/BD/DC protocols**.
 * They simulate very basic serial behavior (e.g., toggling a line) to prevent total
 * deadlock but **do NOT implement the full, intricate protocol logic**, including
 * CRC checks, variable packet lengths, or multi-chunk data decoding as seen in
 * the Linux `grip_xt_read_packet`.
 *
 * Full and accurate implementation of XT/BD/DC protocols would require a dedicated
 * state machine for each device, which is a significant undertaking beyond this stub.
 * These functions will effectively cause these devices to appear unresponsive or
 * return default values in emulated software expecting the full protocol.
 */
static uint8_t
grav_xt_bd_dc_read(void *priv)
{
    grav_data *grav = (grav_data *) priv; // Reusing grav_data for basic timing.
    uint8_t ret = 0xF0; // All buttons unpressed/inactive by default.

    if (timer_is_enabled(&grav->poll_timer)) {
        // Just toggle a data line (e.g., B2/Pin 7) to show some activity.
        if (grav->poll_clock) { // Simulate data toggling with clock.
            ret &= ~0x20; // Drive Pin 7 (B2) low.
        }
    }
    return ret;
}

static void
grav_xt_bd_dc_write(void *priv)
{
    grav_data *grav = (grav_data *) priv; // Reusing grav_data for basic timing.

    if (!timer_is_enabled(&grav->poll_timer)) {
        grav->poll_clock = 0;
        grav->poll_bit_pos = 0; // Simulate starting a new sequence.
        timer_set_delay_u64(&grav->poll_timer, TIMER_USEC * 64); // Use XT strobe time.
    }
    timer_set_delay_u64(&grav->handshake_timer, TIMER_USEC * 10000);
}

static int
grav_xt_bd_dc_read_axis(UNUSED(void *priv), UNUSED(int axis))
{
    return 0;
}

static void
grav_xt_bd_dc_a0_over(UNUSED(void *priv))
{
    // No specific action implemented for the placeholder.
    // The actual XT protocol might use A0 for specific handshakes.
}

/**
 * @brief Callback for Axis 0 (A0) timer overflow for Gravis GrIP devices.
 *
 * This function is tied to the timing of the gameport's A0 line. For
 * proprietary protocols like GrIP, this might be used for specific handshake
 * or timing signals, often indicating a completion of a timing cycle by the PC.
 *
 * @param priv Private data pointer to the `grav_data` structure.
 */
static void
grav_a0_over(void *priv)
{
    grav_data *grav = (grav_data *) priv;
    // When A0 overflows, it might be a trigger for the ID packet request or
    // a protocol state change. The GrIP protocol might expect certain timing
    // for its ID response.
    if (grav->id_request_pending) {
        // If an ID request was pending, the A0 overflow might complete the handshake.
        // The actual ID response would be returned on gameport read based on current state.
        timer_disable(&grav->handshake_timer);
        grav->id_request_pending = 0;
    }
    // Set a new delay for the handshake timer, in case another write/request follows.
    timer_set_delay_u64(&grav->handshake_timer, TIMER_USEC * 10000);
}

// =========================================================================
// Gravis GrIP Device Definitions
// =========================================================================

/**
 * @brief Defines the Gravis GamePad Pro joystick interface.
 *
 * This gamepad uses the proprietary Gravis GrIP protocol (GPP mode). It features
 * a digital D-pad (emulated as 2 axes X/Y), 10 buttons, and a true POV hat.
 * The emulation relies on complex timing and bit-stream encoding/decoding
 * within the `read` and `write` functions, based on the documented 24-bit protocol
 * and pin usage from the Linux `grip.c` driver.
 *
 * DB-15 Pin Connections:
 * Uses standard gameport button pins (B1/B2 for first device, B3/B4 for second)
 * for serial Clock/Data transmission.
 */
const joystick_t joystick_gravis_gamepad_pro = {
    .name          = "Gravis GamePad Pro",
    .internal_name = "gravis_gamepad_pro",
    .init          = grav_init,
    .close         = grav_close,
    .read          = grav_read_gpp,
    .write         = grav_write_gpp,
    .read_axis     = grav_read_axis,
    .a0_over       = grav_a0_over,
    .axis_count    = 2,
    .button_count  = 10,
    .pov_count     = 1,
    .max_joysticks = 2,
    .axis_names    = { "D-pad X", "D-pad Y" },
    .button_names  = { "Select", "Start", "R2", "Blue", "L2", "Green", "Yellow", "Red", "L1", "R1" },
    .pov_names     = { "D-Pad" }
};

/**
 * @brief Defines the Gravis Blackhawk Digital joystick interface.
 *
 * This device uses a more complex GrIP protocol variant (`GRIP_MODE_BD` in `grip.c`),
 * typically offering 3 axes (X, Y, Throttle) and 1 POV hat, along with 5 buttons.
 * **NOTE: This is a placeholder implementation.** The full protocol involves
 * complex chunking, CRC, and variable packet lengths as seen in `grip_xt_read_packet`
 * from Linux kernel, which is not implemented here.
 */
const joystick_t joystick_gravis_blackhawk_digital = {
    .name          = "Gravis Blackhawk Digital",
    .internal_name = "gravis_blackhawk_digital",
    .init          = grav_init,
    .close         = grav_close,
    .read          = grav_xt_bd_dc_read,
    .write         = grav_xt_bd_dc_write,
    .read_axis     = grav_xt_bd_dc_read_axis,
    .a0_over       = grav_xt_bd_dc_a0_over,
    .axis_count    = 3,
    .button_count  = 5,
    .pov_count     = 1,
    .max_joysticks = 1,
    .axis_names    = { "X axis", "Y axis", "Throttle" },
    .button_names  = { "Thumb", "Thumb 2", "Trigger", "Top", "Base" },
    .pov_names     = { "POV" }
};

/**
 * @brief Defines the Gravis Xterminator Digital joystick interface.
 *
 * This device uses another complex GrIP protocol variant (`GRIP_MODE_XT` in `grip.c`),
 * typically offering 5 axes (X, Y, Brake, Gas, Throttle) and 2 POV hats, along with 11 buttons.
 * **NOTE: This is a placeholder implementation.** The full protocol involves
 * complex chunking, CRC, and variable packet lengths as seen in `grip_xt_read_packet`
 * from Linux kernel, which is not implemented here.
 */
const joystick_t joystick_gravis_xterminator_digital = {
    .name          = "Gravis Xterminator Digital",
    .internal_name = "gravis_xterminator_digital",
    .init          = grav_init,
    .close         = grav_close,
    .read          = grav_xt_bd_dc_read,
    .write         = grav_xt_bd_dc_write,
    .read_axis     = grav_xt_bd_dc_read_axis,
    .a0_over       = grav_xt_bd_dc_a0_over,
    .axis_count    = 5,
    .button_count  = 11,
    .pov_count     = 2,
    .max_joysticks = 1,
    .axis_names    = { "X axis", "Y axis", "Brake", "Gas", "Throttle" },
    .button_names  = { "Trigger", "Thumb", "A", "B", "C", "X", "Y", "Z", "Select", "Start", "Mode" },
    .pov_names     = { "POV 1", "POV 2" }
};

/**
 * @brief Defines the Gravis Xterminator DualControl joystick interface.
 *
 * This device uses another complex GrIP protocol variant (`GRIP_MODE_DC` in `grip.c`),
 * typically offering 5 axes (X, Y, RX, RY, Throttle) and 1 POV hat, along with 9 buttons.
 * This is likely for a twin-stick or stick-and-pedals setup.
 * **NOTE: This is a placeholder implementation.** The full protocol involves
 * complex chunking, CRC, and variable packet lengths as seen in `grip_xt_read_packet`
 * from Linux kernel, which is not implemented here.
 */
const joystick_t joystick_gravis_xterminator_dualcontrol = {
    .name          = "Gravis Xterminator DualControl",
    .internal_name = "gravis_xterminator_dualcontrol",
    .init          = grav_init,
    .close         = grav_close,
    .read          = grav_xt_bd_dc_read,
    .write         = grav_xt_bd_dc_write,
    .read_axis     = grav_xt_bd_dc_read_axis,
    .a0_over       = grav_xt_bd_dc_a0_over,
    .axis_count    = 5,
    .button_count  = 9,
    .pov_count     = 1,
    .max_joysticks = 1,
    .axis_names    = { "X axis", "Y axis", "RX axis", "RY axis", "Throttle" },
    .button_names  = { "Trigger", "Thumb", "Top", "Top 2", "Base", "Base 2", "Base 3", "Base 4", "Base 5" },
    .pov_names     = { "POV" }
};
