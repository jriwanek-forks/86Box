/*
 * 86Box    A hypervisor and IBM PC system emulator.
 *
 *          Down-level asynchronous RAS framing for modem emulation.
 */
#include <string.h>
#include <86box/modem/modem_ras.h>

static uint16_t
ras_crc_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t) byte << 8;
    for (unsigned int bit = 0; bit < 8; bit++)
        crc = (crc & 0x8000) ? (uint16_t) ((crc << 1) ^ 0x1021) : (uint16_t) (crc << 1);
    return crc;
}

static void
ras_rx_reset(modem_ras_decoder_t *decoder, uint8_t byte)
{
    decoder->state = (byte == MODEM_RAS_SYNC) ? MODEM_RAS_RX_ADDRESS : MODEM_RAS_RX_SYNC;
    decoder->length = 0;
    decoder->position = 0;
    decoder->crc = 0;
}

void
modem_ras_decoder_init(modem_ras_decoder_t *decoder, modem_ras_frame_callback_t callback, void *priv)
{
    if (!decoder)
        return;

    memset(decoder, 0, sizeof(*decoder));
    decoder->callback = callback;
    decoder->priv = priv;
    decoder->state = MODEM_RAS_RX_SYNC;
}

void
modem_ras_rx_byte(modem_ras_decoder_t *decoder, uint8_t byte)
{
    if (!decoder)
        return;

    switch (decoder->state) {
        case MODEM_RAS_RX_SYNC:
            if (byte == MODEM_RAS_SYNC)
                decoder->state = MODEM_RAS_RX_ADDRESS;
            break;

        case MODEM_RAS_RX_ADDRESS:
            if (byte != MODEM_RAS_BROADCAST && byte != MODEM_RAS_DESTINATION) {
                ras_rx_reset(decoder, byte);
                break;
            }
            decoder->address = byte;
            decoder->crc = ras_crc_update(0, byte);
            decoder->state = MODEM_RAS_RX_LENGTH_HIGH;
            break;

        case MODEM_RAS_RX_LENGTH_HIGH:
            decoder->length = (uint16_t) byte << 8;
            decoder->crc = ras_crc_update(decoder->crc, byte);
            decoder->state = MODEM_RAS_RX_LENGTH_LOW;
            break;

        case MODEM_RAS_RX_LENGTH_LOW:
            decoder->length |= byte;
            decoder->crc = ras_crc_update(decoder->crc, byte);
            if (!decoder->length || decoder->length > MODEM_RAS_MAX_PAYLOAD) {
                ras_rx_reset(decoder, byte);
                break;
            }
            decoder->position = 0;
            decoder->state = MODEM_RAS_RX_PAYLOAD;
            break;

        case MODEM_RAS_RX_PAYLOAD:
            decoder->payload[decoder->position++] = byte;
            decoder->crc = ras_crc_update(decoder->crc, byte);
            if (decoder->position == decoder->length)
                decoder->state = MODEM_RAS_RX_ETX;
            break;

        case MODEM_RAS_RX_ETX:
            if (byte != MODEM_RAS_ETX) {
                ras_rx_reset(decoder, byte);
                break;
            }
            decoder->crc = ras_crc_update(decoder->crc, byte);
            decoder->state = MODEM_RAS_RX_CRC_LOW;
            break;

        case MODEM_RAS_RX_CRC_LOW:
            decoder->received_crc_low = byte;
            decoder->state = MODEM_RAS_RX_CRC_HIGH;
            break;

        case MODEM_RAS_RX_CRC_HIGH:
            if ((((uint16_t) byte << 8) | decoder->received_crc_low) == decoder->crc
                && decoder->callback)
                decoder->callback(decoder->priv, decoder->address, decoder->payload,
                                  decoder->length);
            decoder->state = MODEM_RAS_RX_SYNC;
            decoder->length = 0;
            decoder->position = 0;
            decoder->crc = 0;
            break;
    }
}

bool
modem_ras_encode_frame(uint8_t address, const uint8_t *payload, size_t length,
                       uint8_t *output, size_t output_capacity, size_t *output_length)
{
    uint16_t crc;
    size_t   frame_length;

    if (output_length)
        *output_length = 0;
    if ((address != MODEM_RAS_BROADCAST && address != MODEM_RAS_DESTINATION)
        || !payload || !length || length > MODEM_RAS_MAX_PAYLOAD || !output || !output_length)
        return false;

    frame_length = length + 7;
    if (output_capacity < frame_length)
        return false;

    output[0] = MODEM_RAS_SYNC;
    output[1] = address;
    output[2] = (uint8_t) (length >> 8);
    output[3] = (uint8_t) length;
    memcpy(output + 4, payload, length);
    output[4 + length] = MODEM_RAS_ETX;

    crc = 0;
    for (size_t i = 1; i < length + 5; i++)
        crc = ras_crc_update(crc, output[i]);
    output[5 + length] = (uint8_t) crc;
    output[6 + length] = (uint8_t) (crc >> 8);
    *output_length = frame_length;
    return true;
}

bool
modem_ras_send_nbf(modem_ras_serial_push_t serial_push, void *priv,
                   const uint8_t *frame, int length)
{
    uint8_t output[MODEM_RAS_MAX_PAYLOAD + 7];
    size_t  output_length;

    if (!serial_push || !frame || length <= 0
        || !modem_ras_encode_frame(MODEM_RAS_DESTINATION, frame, (size_t) length,
                                   output, sizeof(output), &output_length))
        return false;

    serial_push(priv, output, (int) output_length);
    return true;
}
