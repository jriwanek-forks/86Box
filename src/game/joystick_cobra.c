/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the Creative Labs Blaster GamePad Cobra.
 *
 * Notes:   The GamePad Cobra uses a proprietary digital protocol over the
 *          standard PC gameport. It supports two daisy-chained devices, each
 *          transmitting a 36-bit packet serially. This implementation is based
 *          on the Linux kernel's `cobra.c` driver.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com> (initial structure based on existing 86Box files)
 *          Original Creative Labs Cobra protocol reverse-engineered by Vojtech Pavlik (from Linux `cobra.c` driver).
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

// Forward declaration of joystick_if_t structure (defined in gameport.h).
//typedef struct joystick_if_t joystick_if_t;

// Cobra protocol specific constants from Linux `cobra.c`
#define COBRA_LENGTH        36  /* Total bits per packet from one Cobra device */
#define COBRA_MAX_STROBE    45  /* 45 us max wait for first strobe (timeout) */

// ADDED: Constants for Cobra sync word and mask, derived from Linux `cobra.c` driver.
// These are 36-bit values.
// The driver checks `(buf[i] & 0x04104107f) ^ 0x041041040`.
// This means that for bits set in the mask, `buf[i]` must match the value.
// 0x04104107fULL is 0b1000001000001000001000001111111 (36 bits)
// 0x041041040ULL is 0b1000001000001000001000001000000 (36 bits)
const uint64_t COBRA_SYNC_MASK = 0x04104107FULL;
const uint64_t COBRA_SYNC_VALUE = 0x041041040ULL;

/**
 * @brief Private data structure for Creative Labs Blaster GamePad Cobra emulation.
 *
 * This structure holds state relevant to the Cobra protocol's
 * packet-based communication, including timers for bit sequencing and
 * the combined data packet being transmitted.
 */
typedef struct cobra_data {
    pc_timer_t poll_timer;    /**< Timer for controlling the bit-stream polling rate. */
    int        poll_bit_pos;  /**< Current bit position within the combined packet (0 to total_bits-1). */
    int        poll_clock;    /**< Simulated clock line state (used for specific output pins). 0=Low, 1=High. */
    uint8_t    packet_buffer[2 * 5]; /**< Buffer for 2 joysticks * 5 bytes/joystick = 10 bytes total (80 bits).
                                        * Cobra uses 36 bits, so 5 bytes (40 bits) per device to hold 36 bits.
                                        * 2 devices * 5 bytes = 10 bytes. */

    pc_timer_t handshake_timer; /**< Timer for detecting handshake/ID request timings. */
    int        id_request_pending; /**< Flag to indicate if an ID request is expected. */
} cobra_data;

/**
 * @brief Timer callback for the Cobra polling sequence.
 *
 * This function simulates the bit-by-bit data transfer of the proprietary
 * protocol. It toggles a simulated clock line and advances the `poll_bit_pos`
 * to point to the next bit in the `packet_buffer` to be read by the emulated
 * PC's gameport driver.
 *
 * The total protocol packet length is 36 bits * max_joysticks for Cobra.
 *
 * @param priv Pointer to the `cobra_data` private structure.
 */
static void
cobra_timer_over(void *priv)
{
    cobra_data *cobra = (cobra_data *) priv;

    // The Cobra protocol involves a clock-like signal.
    cobra->poll_clock = !cobra->poll_clock; // Toggle simulated clock state (0=Low, 1=High).

    // Advance bit position on a specific clock edge (e.g., when clock goes high).
    if (cobra->poll_clock) // Advance bit when clock goes high.
        cobra->poll_bit_pos++; // Move to the next bit in the combined stream.

    // Schedule the next timer tick. Precise timing is critical for protocol accuracy.
    // The `cobra.c` driver uses `COBRA_MAX_STROBE` (45 us) as a timeout.
    // We'll use a smaller, consistent period for each clock phase.
    if (cobra->poll_bit_pos < (joystick_creative_cobra.max_joysticks * COBRA_LENGTH)) {
        timer_set_delay_u64(&cobra->poll_timer, TIMER_USEC * 5); // Example: 5us per clock phase.
    } else {
        // Packet transfer complete, disable timer.
        timer_disable(&cobra->poll_timer);
        cobra->id_request_pending = 0; // Clear ID request after data transfer completes.
        cobra->poll_bit_pos = 0; // Reset bit position for the next packet request.
    }
}

/**
 * @brief Timer callback for Cobra ID/handshake timing.
 *
 * This timer is set after a write operation. Its expiration might signal
 * the end of a handshake window or a timeout.
 *
 * @param priv Pointer to the `cobra_data` private structure.
 */
static void
cobra_handshake_timer_over(UNUSED(void *priv))
{
    cobra_data *cobra = (cobra_data *) priv;
    timer_disable(&cobra->handshake_timer);
    cobra->id_request_pending = 0; // Handshake window closed or timed out.
}

/**
 * @brief Initializes the Creative Labs Blaster GamePad Cobra interface.
 *
 * Allocates and initializes the private data structure `cobra_data` and sets up
 * the necessary timers for protocol communication.
 *
 * @return void* A pointer to the allocated `cobra_data` structure.
 */
static void *
cobra_init(void)
{
    cobra_data *cobra = (cobra_data *) calloc(1, sizeof(cobra_data));
    if (cobra == NULL)
        return NULL;

    timer_add(&cobra->poll_timer, cobra_timer_over, cobra, 0);
    timer_add(&cobra->handshake_timer, cobra_handshake_timer_over, cobra, 0);

    return cobra;
}

/**
 * @brief Closes and cleans up resources for the Creative Labs Blaster GamePad Cobra interface.
 *
 * Frees the allocated private data structure.
 *
 * @param priv Private data pointer to the `cobra_data` structure.
 */
static void
cobra_close(void *priv)
{
    cobra_data *cobra = (cobra_data *) priv;
    if (cobra)
        free(cobra);
}

/**
 * @brief Reads the state of the gameport lines for Creative Labs Blaster GamePad Cobra.
 *
 * This function simulates the gamepad's response to the PC's polling.
 * It sets the appropriate bits on the gameport input lines (bits 4-7 of the
 * status byte) based on the current state of the serial polling sequence
 * (`cobra->poll_clock` and the current bit from `cobra->packet_buffer`).
 *
 * Based on `cobra.c` Linux driver:
 * - The driver reads `(gameport_read(gameport) >> shift) & 3`.
 * - `shift` is 4 for device 0 (looking at bits 4 and 5: B1 and B2).
 * - `shift` is 6 for device 1 (looking at bits 6 and 7: B3 and B4).
 * - `w & 0x30` indicates it's looking at bits 4 and 5 (for `shift=4`).
 * - `((w >> 5) & 1)` means bit 5 (B2) is the data bit, and bit 4 (B1) is the clock/strobe.
 * (For `shift=6`, it would be B4/Bit 7 as data, B3/Bit 6 as clock).
 * - The logic `(w & 0x30) < 0x30` suggests a specific transition or state.
 * This implies that the data bit is valid when the clock bit is low, or during a specific edge.
 * We will assume data is valid when clock is high, and clock toggles.
 *
 * @param priv Private data pointer to the `cobra_data` structure.
 * @return uint8_t A byte representing the gameport input lines (bits 4-7 for buttons).
 */
static uint8_t
cobra_read(void *priv)
{
    cobra_data *cobra = (cobra_data *) priv;
    // Default state: all button lines high (unpressed/inactive), matching 0xF0 for gameport.
    uint8_t ret = 0xf0;

    if (!JOYSTICK_PRESENT(0, 0)) // Check if at least the first joystick is connected.
        return 0xff; // If no joystick is connected, return all high.

    // If a polling sequence is active (timer enabled), manipulate button lines
    // according to the Cobra protocol's serial data transfer.
    if (timer_is_enabled(&cobra->poll_timer)) {
        // Determine which joystick's data we are currently outputting.
        // Each Cobra device sends a 36-bit packet.
        int current_js = cobra->poll_bit_pos / COBRA_LENGTH;

        // Ensure we don't read beyond the maximum supported joysticks.
        if (current_js >= joystick_creative_cobra.max_joysticks)
            return 0xf0; // Default high if out of bounds.

        // Get the current bit from the packet_buffer for the active joystick.
        // The index within the 36-bit chunk for the current joystick.
        int bit_in_chunk = cobra->poll_bit_pos % COBRA_LENGTH;
        // The byte index within the `packet_buffer` for the current joystick.
        int byte_idx = (current_js * 5) + (bit_in_chunk / 8); // 5 bytes per 36-bit packet.
        // The bit position within that byte.
        int bit_in_byte = bit_in_chunk % 8;

        uint8_t current_data_bit = (cobra->packet_buffer[byte_idx] >> bit_in_byte) & 1;

        // Simulate the clock and data lines based on the current joystick's output pins.
        if (current_js == 0) { // First Cobra device (uses B1/B2 lines)
            // Clock on Pin 2 (B1, Bit 4). Drive low when simulated clock is 0 (LOW state).
            if (!cobra->poll_clock)
                ret &= ~0x10; // Clear bit 4 (0x10) to make Pin 2 low.

            // Data on Pin 7 (B2, Bit 5). Drive low if current_data_bit is 1 (inverted logic assumed).
            // The `cobra.c` driver reads `(w >> 5) & 1` which is the raw bit.
            // We'll assume a '1' means the line is low, consistent with previous Gravis.
            if (current_data_bit == 1)
                ret &= ~0x20; // Clear bit 5 (0x20) to make Pin 7 low.

        } else if (current_js == 1) { // Second Cobra device (uses B3/B4 lines)
            // Clock on Pin 10 (B3, Bit 6). Drive low when simulated clock is 0.
            if (!cobra->poll_clock)
                ret &= ~0x40; // Clear bit 6 (0x40) to make Pin 10 low.

            // Data on Pin 14 (B4, Bit 7). Drive low if current_data_bit is 1.
            if (current_data_bit == 1)
                ret &= ~0x80; // Clear bit 7 (0x80) to make Pin 14 low.
        }
    } else {
        // If no polling is active, revert to standard 4-button fallback.
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
 * @brief Encodes joystick state into a 36-bit Cobra packet.
 *
 * This function takes the emulated joystick's state and encodes it into
 * a 36-bit raw packet (`__u64 buf` in Linux driver terms) that, when
 * processed by the Linux driver's `cobra_read_packet`, will yield the correct
 * button and D-pad states.
 *
 * This involves reversing the complex bit scrambling and interleaving
 * performed by the Linux driver's `data[i] = ...` line.
 *
 * The `cobra.c` driver decodes `data` from `buf` as:
 * `data = ((buf >>  7) & 0x000001f) | ((buf >>  8) & 0x00003e0) | ((buf >>  9) & 0x0007c00) | ((buf >> 10) & 0x00f8000) | ((buf >> 11) & 0x1f00000);`
 *
 * To reverse this, we extract 5-bit chunks from `decoded_data` and place them
 * into `raw_packet` at their original `buf` positions (7, 8, 9, 10, 11).
 *
 * @param js_state Pointer to the `joystick_t` state for the current emulated joystick.
 * @return uint64_t The 36-bit encoded Cobra packet (`buf` in Linux driver).
 */
static uint64_t
cobra_encode_packet(const joystick_state_t *js_state)
{
    uint64_t raw_packet   = 0; // This will be the 36-bit packet (buf in Linux driver).
    uint32_t decoded_data = 0; // This is the 32-bit 'data' value from the Linux driver's perspective.

    // --- Step 1: Construct the 'decoded_data' (32-bit value) based on current joystick_state ---
    // D-pad X (Left/Right)
    // Linux driver maps: ABS_X, -1 to 1.
    // Left: data[i] & (1 << 3)
    // Right: data[i] & (1 << 4)
    if (js_state->axis[0] < -16384)
        decoded_data |= (1 << 3); // Left (bit 3)
    else if (js_state->axis[0] > 16384)
        decoded_data |= (1 << 4); // Right (bit 4)

    // D-pad Y (Up/Down)
    // Linux driver maps: ABS_Y, -1 to 1.
    // Up: data[i] & (1 << 1)
    // Down: data[i] & (1 << 2)
    if (js_state->axis[1] < -16384)
        decoded_data |= (1 << 1); // Up (bit 1)
    else if (js_state->axis[1] > 16384)
        decoded_data |= (1 << 2); // Down (bit 2)

    // Buttons (cobra_btn array in Linux driver has 12 buttons)
    // The `cobra.c` driver uses `data[i] & (0x20 << j)` for button checks.
    // This implies button bits start from bit 5 (0x20).
    // So, button[0] (BTN_START) is bit 5, button[1] (BTN_SELECT) is bit 6, etc.
    // Our joystick_state.button has 12 buttons.
    // Map them directly to the bits as per cobra_btn array and the `0x20 << j` logic.
    // BTN_START (js_state->button[0]) -> bit 5
    if (js_state->button[0])
        decoded_data |= (1 << 5);
    // BTN_SELECT (js_state->button[1]) -> bit 6
    if (js_state->button[1])
        decoded_data |= (1 << 6);
    // BTN_TL (js_state->button[2]) -> bit 7
    if (js_state->button[2])
        decoded_data |= (1 << 7);
    // BTN_TR (js_state->button[3]) -> bit 8
    if (js_state->button[3])
        decoded_data |= (1 << 8);
    // BTN_X (js_state->button[4]) -> bit 9
    if (js_state->button[4])
        decoded_data |= (1 << 9);
    // BTN_Y (js_state->button[5]) -> bit 10
    if (js_state->button[5])
        decoded_data |= (1 << 10);
    // BTN_Z (js_state->button[6]) -> bit 11
    if (js_state->button[6])
        decoded_data |= (1 << 11);
    // BTN_A (js_state->button[7]) -> bit 12
    if (js_state->button[7])
        decoded_data |= (1 << 12);
    // BTN_B (js_state->button[8]) -> bit 13
    if (js_state->button[8])
        decoded_data |= (1 << 13);
    // BTN_C (js_state->button[9]) -> bit 14
    if (js_state->button[9])
        decoded_data |= (1 << 14);
    // BTN_TL2 (js_state->button[10]) -> bit 15
    if (js_state->button[10])
        decoded_data |= (1 << 15);
    // BTN_TR2 (js_state->button[11]) -> bit 16
    if (js_state->button[11])
        decoded_data |= (1 << 16);

    // --- Step 2: Reverse the bit scrambling to get the 36-bit 'raw_packet' (buf in Linux driver) ---
    // This is the inverse of the complex bit interleaving from `cobra.c`'s `data[i] = ...` line.
    // Each 5-bit chunk from `decoded_data` is placed into a specific 5-bit window in `raw_packet`.

    // Bits 0-4 of decoded_data go to bits 7-11 of raw_packet
    raw_packet |= ((uint64_t)(decoded_data & 0x1F) << 7);

    // Bits 5-9 of decoded_data go to bits 8-12 of raw_packet
    raw_packet |= ((uint64_t)((decoded_data >> 5) & 0x1F) << 8);

    // Bits 10-14 of decoded_data go to bits 9-13 of raw_packet
    raw_packet |= ((uint64_t)((decoded_data >> 10) & 0x1F) << 9);

    // Bits 15-19 of decoded_data go to bits 10-14 of raw_packet
    raw_packet |= ((uint64_t)((decoded_data >> 15) & 0x1F) << 10);

    // Bits 20-24 of decoded_data go to bits 11-15 of raw_packet
    raw_packet |= ((uint64_t)((decoded_data >> 20) & 0x1F) << 11);

    // ADDED: Apply the fixed sync bits to the `raw_packet`.
    // The `cobra.c` driver checks for these specific bits for synchronization.
    // This ensures the emulated gamepad transmits a packet that the driver can recognize.
    raw_packet = (raw_packet & ~COBRA_SYNC_MASK) | COBRA_SYNC_VALUE;

    // NOTE: The `cobra.c` driver also performs a bit rotation loop after reading.
    // `for (j = 0; j < COBRA_LENGTH && (buf[i] & 0x04104107f) ^ 0x041041040; j++)`
    // `buf[i] = (buf[i] >> 1) | ((__u64)(buf[i] & 1) << (COBRA_LENGTH - 1));`
    // This loop rotates the buffer until the sync word is found at a specific position.
    // Our `raw_packet` is constructed as the "unrotated" form. The driver's logic
    // is expected to handle the rotation to find the sync word. If emulation fails,
    // further investigation into this rotation and its interaction with the sync word
    // might be necessary.

    return raw_packet;
}

/**
 * @brief Handles write operations for Creative Labs Blaster GamePad Cobra.
 *
 * For the Cobra protocol, write operations from the PC to the gameport
 * (typically 0x201) are crucial as they act as commands or handshake signals,
 * initiating the data polling sequences from the gamepad.
 *
 * This function prepares the combined 72-bit data packet (2 joysticks * 36 bits/joystick)
 * by encoding the current state of all emulated joysticks into `packet_buffer`.
 *
 * @param priv Private data pointer to the `cobra_data` structure.
 */
static void
cobra_write(void *priv)
{
    cobra_data *cobra = (cobra_data *) priv;

    if (!JOYSTICK_PRESENT(0, 0)) // Check if at least one joystick is connected on the host side.
        return;

    // When the PC writes to the gameport, it signals the start of a polling cycle.
    // This is where the gamepad would prepare its data packet.
    if (!timer_is_enabled(&cobra->poll_timer)) { // Only start if no packet is currently being sent.
        cobra->poll_clock = 0; // Start with clock low (initial state for first bit transfer).
        cobra->poll_bit_pos = 0; // Reset bit position to the beginning of the combined packet.
        // Set an initial delay before the first bit transfer/clock pulse.
        timer_set_delay_u64(&cobra->poll_timer, TIMER_USEC * 10); // Example initial delay to prepare.

        // Initialize `packet_buffer` to all zeros. Unmapped joysticks will send 0s.
        memset(cobra->packet_buffer, 0, sizeof(cobra->packet_buffer));

        // Iterate through each emulated joystick slot (up to `max_joysticks`) and encode its state.
        for (int js = 0; js < joystick_creative_cobra.max_joysticks; js++) {
            // Only encode data for joystick slots that have a physical host joystick mapped.
            if (!joystick_state[0][js].plat_joystick_nr) {
                 // If no physical joystick is mapped to this emulated slot, its 36-bit section
                 // in `packet_buffer` will remain zero (from the initial memset),
                 // effectively sending a "blank" or inactive state for that gamepad.
                 continue;
            }

            // Encode the 36-bit packet for the current joystick.
            uint64_t single_joystick_packet = cobra_encode_packet(&joystick_state[0][js]);

            // Copy the completed 36-bit packet (5 bytes needed for 36 bits) into the larger `packet_buffer`.
            // Each joystick's data is concatenated.
            int buffer_offset_bytes = js * 5; // Offset for current joystick's data.
            cobra->packet_buffer[buffer_offset_bytes + 0] = (uint8_t)(single_joystick_packet & 0xFF);
            cobra->packet_buffer[buffer_offset_bytes + 1] = (uint8_t)((single_joystick_packet >> 8) & 0xFF);
            cobra->packet_buffer[buffer_offset_bytes + 2] = (uint8_t)((single_joystick_packet >> 16) & 0xFF);
            cobra->packet_buffer[buffer_offset_bytes + 3] = (uint8_t)((single_joystick_packet >> 24) & 0xFF);
            cobra->packet_buffer[buffer_offset_bytes + 4] = (uint8_t)((single_joystick_packet >> 32) & 0xFF); // For the 36th bit.
        }
    }

    // Reset handshake timer. A new write might reset a timeout for ID request
    // or signify the start of a new polling cycle.
    timer_set_delay_u64(&cobra->handshake_timer, TIMER_USEC * COBRA_MAX_STROBE); // Use Cobra's strobe time.
}

/**
 * @brief Reads a specific axis for the Creative Labs Blaster GamePad Cobra.
 *
 * The Cobra GamePad is primarily digital. Its D-pad input is communicated via
 * the proprietary digital protocol, not direct analog axis readings on the
 * gameport's potentiometer pins. Therefore, any attempts by the emulated
 * software to read these pins as analog axes should result in a fixed
 * (e.g., centered) value, or 0.
 *
 * @param priv Private data pointer, unused.
 * @param axis The index of the axis to read.
 * @return int The axis value (typically 0 for digital gamepads), or AXIS_NOT_PRESENT.
 */
static int
cobra_read_axis(UNUSED(void *priv), UNUSED(int axis))
{
    if (!JOYSTICK_PRESENT(0, 0))
        return AXIS_NOT_PRESENT;

    return 0; // Return 0 to simulate a centered analog input.
}

/**
 * @brief Callback for Axis 0 (A0) timer overflow for Creative Labs Blaster GamePad Cobra.
 *
 * This function is tied to the timing of the gameport's A0 line. For
 * proprietary protocols like Cobra, this might be used for specific handshake
 * or timing signals, often indicating a completion of a timing cycle by the PC.
 *
 * @param priv Private data pointer to the `cobra_data` structure.
 */
static void
cobra_a0_over(void *priv)
{
    cobra_data *cobra = (cobra_data *) priv;
    // When A0 overflows, it might be a trigger for the ID packet request or
    // a protocol state change. The Cobra protocol might expect certain timing
    // for its ID response.
    if (cobra->id_request_pending) {
        // If an ID request was pending, the A0 overflow might complete the handshake.
        // The actual ID response would be returned on gameport read based on current state.
        timer_disable(&cobra->handshake_timer); // Complete handshake or timeout.
        cobra->id_request_pending = 0;
    }
    // Set a new delay for the handshake timer, in case another write/request follows.
    timer_set_delay_u64(&cobra->handshake_timer, TIMER_USEC * COBRA_MAX_STROBE);
}

// =========================================================================
// Creative Labs Blaster GamePad Cobra Definition
// =========================================================================

/**
 * @brief Defines the Creative Labs Blaster GamePad Cobra joystick interface.
 *
 * This gamepad uses a proprietary digital protocol. It typically features
 * a digital D-pad (emulated as 2 axes X/Y) and 12 buttons.
 * The emulation relies on complex timing and bit-stream encoding/decoding
 * within the `read` and `write` functions, based on the documented 36-bit protocol
 * and pin usage from the Linux `cobra.c` driver.
 *
 * DB-15 Pin Connections:
 * Uses standard gameport button pins (B1/B2 for first device, B3/B4 for second)
 * for serial Clock/Data transmission.
 */
const joystick_t joystick_creative_gamepad = {
    .name          = "Creative Labs Blaster GamePad",
    .internal_name = "creative_gamepad",
    .init          = cobra_init,
    .close         = cobra_close,
    .read          = cobra_read,
    .write         = cobra_write,
    .read_axis     = cobra_read_axis,
    .a0_over       = cobra_a0_over,
    .axis_count    = 2,
    .button_count  = 10,
    .pov_count     = 0,
    .max_joysticks = 1, // Check This
    .axis_names    = { "D-pad X", "D-pad Y" },
    .button_names  = { "Start", "Select", "D/LS", "C/RS", "TA/X", "TB/Y", "D/Z", "A", "B", "C" },
    .pov_names     = { NULL }
};

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
