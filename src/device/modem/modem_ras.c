/*
 * 86Box    A hypervisor and IBM PC system emulator.
 *
 *          Down-level asynchronous RAS framing for modem emulation.
 */
#include <string.h>
#include <86box/modem/modem_crypto.h>
#include <86box/modem/modem_ras.h>

#define MODEM_RAS_PROTOCOL_MESSAGE_LENGTH        13
#define MODEM_RAS_PADDED_PROTOCOL_MESSAGE_LENGTH 333
#define MODEM_RAS_RESPONSE_10_USERNAME_LENGTH    21
#define MODEM_RAS_RESPONSE_10_LENGTH             46
#define MODEM_RAS_RESPONSE_20_USERNAME_LENGTH    257
#define MODEM_RAS_RESPONSE_20_DOMAIN_LENGTH      16
#define MODEM_RAS_RESPONSE_20_LM_OFFSET          274
#define MODEM_RAS_RESPONSE_20_NT_OFFSET          298
#define MODEM_RAS_RESPONSE_20_USE_NT_OFFSET      322
#define MODEM_RAS_RESPONSE_20_LENGTH             326

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
modem_ras_decoder_init(modem_ras_decoder_t *decoder, modem_ras_frame_callback_t callback,
                       modem_ras_event_callback_t event_callback, void *priv)
{
    if (!decoder)
        return;

    memset(decoder, 0, sizeof(*decoder));
    decoder->callback = callback;
    decoder->event_callback = event_callback;
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
                if (decoder->event_callback)
                    decoder->event_callback(decoder->priv, MODEM_RAS_EVENT_INVALID_ADDRESS,
                                            byte, 0, byte, 0);
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
                if (decoder->event_callback)
                    decoder->event_callback(decoder->priv, MODEM_RAS_EVENT_INVALID_LENGTH,
                                            decoder->address, decoder->length, decoder->length,
                                            MODEM_RAS_MAX_PAYLOAD);
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
                if (decoder->event_callback)
                    decoder->event_callback(decoder->priv, MODEM_RAS_EVENT_INVALID_ETX,
                                            decoder->address, decoder->length, byte,
                                            MODEM_RAS_ETX);
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
        {
            const uint16_t received_crc = ((uint16_t) byte << 8) | decoder->received_crc_low;

            if (received_crc == decoder->crc) {
                if (decoder->event_callback)
                    decoder->event_callback(decoder->priv, MODEM_RAS_EVENT_FRAME_RECEIVED,
                                            decoder->address, decoder->length, 0, 0);
                if (decoder->callback)
                    decoder->callback(decoder->priv, decoder->address, decoder->payload,
                                      decoder->length);
            } else if (decoder->event_callback)
                decoder->event_callback(decoder->priv, MODEM_RAS_EVENT_BAD_CRC,
                                        decoder->address, decoder->length, received_crc,
                                        decoder->crc);
            decoder->state = MODEM_RAS_RX_SYNC;
            decoder->length = 0;
            decoder->position = 0;
            decoder->crc = 0;
            break;
        }
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

static uint16_t
ras_read_le16(const uint8_t *data)
{
    return (uint16_t) (data[0] | ((uint16_t) data[1] << 8));
}

static void
ras_write_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t) value;
    data[1] = (uint8_t) (value >> 8);
}

static uint32_t
ras_read_le32(const uint8_t *data)
{
    return (uint32_t) data[0] | ((uint32_t) data[1] << 8)
         | ((uint32_t) data[2] << 16) | ((uint32_t) data[3] << 24);
}

static bool
ras_zero_padding(const uint8_t *data, size_t start, size_t length)
{
    for (size_t i = start; i < length; i++) {
        if (data[i])
            return false;
    }
    return true;
}

typedef struct modem_ras_response_view_t {
    const uint8_t *username;
    size_t         username_length;
    const uint8_t *response;
    bool           use_nt_response;
} modem_ras_response_view_t;

static bool
ras_parse_response(const uint8_t *message, size_t length,
                   modem_ras_response_view_t *view)
{
    size_t username_length;
    size_t response_offset;
    size_t message_length;
    const uint8_t *terminator;

    if (!message || !view || !length)
        return false;

    switch (message[0]) {
        case MODEM_RAS_FRAME_RESPONSE:
            message_length = MODEM_RAS_RESPONSE_10_LENGTH;
            username_length = MODEM_RAS_RESPONSE_10_USERNAME_LENGTH;
            response_offset = 1 + username_length;
            if (length != message_length && length != MODEM_RAS_PADDED_PROTOCOL_MESSAGE_LENGTH)
                return false;
            break;

        case MODEM_RAS_FRAME_RESPONSE_20:
            message_length = MODEM_RAS_RESPONSE_20_LENGTH;
            username_length = MODEM_RAS_RESPONSE_20_USERNAME_LENGTH;
            response_offset = MODEM_RAS_RESPONSE_20_LM_OFFSET;
            if ((length != message_length && length != MODEM_RAS_PADDED_PROTOCOL_MESSAGE_LENGTH)
                || !memchr(message + 1 + username_length, 0,
                           MODEM_RAS_RESPONSE_20_DOMAIN_LENGTH))
                return false;
            {
                const uint32_t use_nt_response =
                    ras_read_le32(message + MODEM_RAS_RESPONSE_20_USE_NT_OFFSET);
                if (use_nt_response > 1)
                    return false;
                view->use_nt_response = use_nt_response != 0;
                if (view->use_nt_response)
                    response_offset = MODEM_RAS_RESPONSE_20_NT_OFFSET;
            }
            break;

        default:
            return false;
    }

    if (length > message_length && !ras_zero_padding(message, message_length, length))
        return false;

    terminator = (const uint8_t *) memchr(message + 1, 0, username_length);
    if (!terminator)
        return false;

    view->username = message + 1;
    view->username_length = (size_t) (terminator - view->username);
    view->response = message + response_offset;
    if (message[0] == MODEM_RAS_FRAME_RESPONSE)
        view->use_nt_response = false;
    return true;
}

static bool
ras_lm_response(const char *password, const uint8_t challenge[MODEM_RAS_CHALLENGE_LENGTH],
                uint8_t response[MODEM_RAS_RESPONSE_LENGTH])
{
    static const uint8_t magic[8] = { 'K', 'G', 'S', '!', '@', '#', '$', '%' };
    uint8_t upper_password[14] = { 0 };
    uint8_t password_hash[16];
    uint8_t padded_hash[21] = { 0 };
    size_t  password_length;

    if (!password || !challenge || !response)
        return false;

    password_length = strlen(password);
    if (password_length > sizeof(upper_password))
        password_length = sizeof(upper_password);
    for (size_t i = 0; i < password_length; i++) {
        uint8_t value = (uint8_t) password[i];
        if (value >= 0x80) {
            memset(upper_password, 0, sizeof(upper_password));
            return false;
        }
        if (value >= 'a' && value <= 'z')
            value = (uint8_t) (value - 'a' + 'A');
        upper_password[i] = value;
    }

    modem_des_encrypt_block(upper_password, magic, password_hash);
    modem_des_encrypt_block(upper_password + 7, magic, password_hash + 8);
    memcpy(padded_hash, password_hash, sizeof(password_hash));
    for (size_t i = 0; i < 3; i++)
        modem_des_encrypt_block(padded_hash + i * 7, challenge, response + i * 8);

    memset(upper_password, 0, sizeof(upper_password));
    memset(password_hash, 0, sizeof(password_hash));
    memset(padded_hash, 0, sizeof(padded_hash));
    return true;
}

static bool
ras_nt_response(const char *password, const uint8_t challenge[MODEM_RAS_CHALLENGE_LENGTH],
                uint8_t response[MODEM_RAS_RESPONSE_LENGTH])
{
    uint8_t unicode_password[128] = { 0 };
    uint8_t password_hash[MD4_DIGEST_LENGTH];
    uint8_t padded_hash[21] = { 0 };
    size_t  password_length;

    if (!password || !challenge || !response)
        return false;

    password_length = strlen(password);
    if (password_length > sizeof(unicode_password) / 2)
        return false;
    for (size_t i = 0; i < password_length; i++) {
        unicode_password[i * 2] = (uint8_t) password[i];
        unicode_password[i * 2 + 1] = 0;
    }

    modem_md4(unicode_password, password_length * 2, password_hash);
    memcpy(padded_hash, password_hash, sizeof(password_hash));
    for (size_t i = 0; i < 3; i++)
        modem_des_encrypt_block(padded_hash + i * 7, challenge, response + i * 8);

    memset(unicode_password, 0, sizeof(unicode_password));
    memset(password_hash, 0, sizeof(password_hash));
    memset(padded_hash, 0, sizeof(padded_hash));
    return true;
}

static bool
ras_response_matches_credentials(const modem_ras_response_view_t *view,
                                 const char *username, const char *password,
                                 const uint8_t challenge[MODEM_RAS_CHALLENGE_LENGTH])
{
    uint8_t expected[MODEM_RAS_RESPONSE_LENGTH];
    const size_t username_length = strlen(username);
    bool matches;

    if (view->username_length != username_length
        || memcmp(view->username, username, username_length))
        return false;

    if (!(view->use_nt_response ? ras_nt_response(password, challenge, expected)
                                : ras_lm_response(password, challenge, expected)))
        return false;

    matches = modem_constant_time_equal(view->response, expected, sizeof(expected));
    memset(expected, 0, sizeof(expected));
    return matches;
}

void
modem_ras_session_init(modem_ras_session_t *session)
{
    if (session)
        memset(session, 0, sizeof(*session));
}

modem_ras_session_result_t
modem_ras_session_process_message(modem_ras_session_t *session, const uint8_t *message,
                                  size_t length, const char *username, const char *password,
                                  modem_ras_message_send_t send, void *send_priv,
                                  modem_ras_random_bytes_t random_bytes)
{
    if (!session || !message || !send)
        return MODEM_RAS_SESSION_INVALID_MESSAGE;
    if (!username)
        username = "";
    if (!password)
        password = "";

    switch (session->state) {
        case MODEM_RAS_SESSION_WAIT_PROTOCOL:
        {
            uint8_t protocol_response[MODEM_RAS_PROTOCOL_MESSAGE_LENGTH] = { MODEM_RAS_FRAME_PROTOCOL };
            uint8_t challenge_frame[1 + MODEM_RAS_CHALLENGE_LENGTH] = { MODEM_RAS_FRAME_CHALLENGE };

            if ((length != sizeof(protocol_response)
                 && length != MODEM_RAS_PADDED_PROTOCOL_MESSAGE_LENGTH)
                || message[0] != MODEM_RAS_FRAME_PROTOCOL)
                return MODEM_RAS_SESSION_INVALID_MESSAGE;
            for (size_t i = sizeof(protocol_response); i < length; i++) {
                if (message[i] != 0)
                    return MODEM_RAS_SESSION_INVALID_MESSAGE;
            }
            if (!random_bytes)
                return MODEM_RAS_SESSION_RANDOM_FAILURE;

            session->client_version = ras_read_le16(message + 1);
            ras_write_le16(protocol_response + 1, MODEM_RAS_PROTOCOL_VERSION_20);
            if (!send(send_priv, protocol_response, sizeof(protocol_response)))
                return MODEM_RAS_SESSION_SEND_FAILURE;
            if (!random_bytes(session->challenge, sizeof(session->challenge)))
                return MODEM_RAS_SESSION_RANDOM_FAILURE;

            memcpy(challenge_frame + 1, session->challenge, sizeof(session->challenge));
            if (!send(send_priv, challenge_frame, sizeof(challenge_frame)))
                return MODEM_RAS_SESSION_SEND_FAILURE;

            session->state = MODEM_RAS_SESSION_WAIT_RESPONSE;
            return MODEM_RAS_SESSION_PROTOCOL_NEGOTIATED;
        }

        case MODEM_RAS_SESSION_WAIT_RESPONSE:
        {
            uint8_t result_frame[3] = { MODEM_RAS_FRAME_RESULT };
            modem_ras_response_view_t response;
            bool credentials_configured = username[0] || password[0];
            bool authenticated;

            if (!ras_parse_response(message, length, &response))
                return MODEM_RAS_SESSION_INVALID_MESSAGE;

            authenticated = !credentials_configured
                            || ras_response_matches_credentials(&response, username, password,
                                                                session->challenge);
            ras_write_le16(result_frame + 1, authenticated
                                                ? MODEM_RAS_RESULT_AUTHENTICATED
                                                : MODEM_RAS_RESULT_NOT_AUTHENTICATED);
            if (!send(send_priv, result_frame, sizeof(result_frame)))
                return MODEM_RAS_SESSION_SEND_FAILURE;

            session->state = authenticated ? MODEM_RAS_SESSION_AUTHENTICATED
                                           : MODEM_RAS_SESSION_REJECTED;
            return authenticated ? MODEM_RAS_SESSION_AUTHENTICATED_RESULT
                                 : MODEM_RAS_SESSION_AUTH_REJECTED;
        }

        case MODEM_RAS_SESSION_AUTHENTICATED:
            return MODEM_RAS_SESSION_POST_AUTH_UNIMPLEMENTED;

        case MODEM_RAS_SESSION_REJECTED:
            return MODEM_RAS_SESSION_INVALID_MESSAGE;
    }

    return MODEM_RAS_SESSION_INVALID_MESSAGE;
}
