/*
 * 86Box    A hypervisor and IBM PC system emulator.
 *
 *          Down-level asynchronous RAS framing for modem emulation.
 */
#ifndef EMU_MODEM_RAS_H
#define EMU_MODEM_RAS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODEM_RAS_MAX_PAYLOAD 1500
#define MODEM_RAS_SYNC        0x16
#define MODEM_RAS_ETX         0x03
#define MODEM_RAS_BROADCAST   0x01
#define MODEM_RAS_DESTINATION 0x02

#define MODEM_RAS_FRAME_PROTOCOL       100
#define MODEM_RAS_FRAME_CHALLENGE      101
#define MODEM_RAS_FRAME_RESPONSE       102
#define MODEM_RAS_FRAME_RESULT         104
#define MODEM_RAS_FRAME_CONFIGURATION_REQUEST 107
#define MODEM_RAS_FRAME_CONFIGURATION_RESULT  108
#define MODEM_RAS_FRAME_RESPONSE_20    109
#define MODEM_RAS_PROTOCOL_VERSION_20  20
#define MODEM_RAS_RESULT_AUTHENTICATED 0x6100
#define MODEM_RAS_RESULT_NOT_AUTHENTICATED 0x6200
#define MODEM_RAS_RESULT_PROJECTIONS_FAILED 0x7400
#define MODEM_RAS_RESULT_NAMES_ADDED   0x6500
#define MODEM_RAS_RESULT_CONFIGURATION_SUCCESS 0x7C00
#define MODEM_RAS_CHALLENGE_LENGTH     8
#define MODEM_RAS_RESPONSE_LENGTH      24
#define MODEM_RAS_CREDENTIAL_LENGTH    64

typedef void (*modem_ras_frame_callback_t)(void *priv, uint8_t address,
                                           const uint8_t *payload, size_t length);
typedef enum modem_ras_event_t {
    MODEM_RAS_EVENT_FRAME_RECEIVED,
    MODEM_RAS_EVENT_INVALID_ADDRESS,
    MODEM_RAS_EVENT_INVALID_LENGTH,
    MODEM_RAS_EVENT_INVALID_ETX,
    MODEM_RAS_EVENT_BAD_CRC
} modem_ras_event_t;
typedef void (*modem_ras_event_callback_t)(void *priv, modem_ras_event_t event,
                                           uint8_t address, uint16_t length,
                                           uint16_t actual, uint16_t expected);
typedef void (*modem_ras_serial_push_t)(void *priv, const uint8_t *data, int length);

typedef enum modem_ras_rx_state_t {
    MODEM_RAS_RX_SYNC,
    MODEM_RAS_RX_ADDRESS,
    MODEM_RAS_RX_LENGTH_HIGH,
    MODEM_RAS_RX_LENGTH_LOW,
    MODEM_RAS_RX_PAYLOAD,
    MODEM_RAS_RX_ETX,
    MODEM_RAS_RX_CRC_LOW,
    MODEM_RAS_RX_CRC_HIGH
} modem_ras_rx_state_t;

typedef enum modem_ras_session_state_t {
    MODEM_RAS_SESSION_WAIT_PROTOCOL,
    MODEM_RAS_SESSION_WAIT_RESPONSE,
    MODEM_RAS_SESSION_AUTHENTICATED,
    MODEM_RAS_SESSION_CONFIGURED,
    MODEM_RAS_SESSION_REJECTED
} modem_ras_session_state_t;

typedef enum modem_ras_session_result_t {
    MODEM_RAS_SESSION_PROTOCOL_NEGOTIATED,
    MODEM_RAS_SESSION_AUTHENTICATED_RESULT,
    MODEM_RAS_SESSION_AUTH_REJECTED,
    MODEM_RAS_SESSION_CONFIGURATION_ACCEPTED,
    MODEM_RAS_SESSION_CONFIGURATION_REJECTED,
    MODEM_RAS_SESSION_POST_CONFIGURATION_UNIMPLEMENTED,
    MODEM_RAS_SESSION_INVALID_MESSAGE,
    MODEM_RAS_SESSION_RANDOM_FAILURE,
    MODEM_RAS_SESSION_SEND_FAILURE
} modem_ras_session_result_t;

typedef struct modem_ras_session_t {
    modem_ras_session_state_t state;
    uint16_t                  client_version;
    uint8_t                   challenge[MODEM_RAS_CHALLENGE_LENGTH];
} modem_ras_session_t;

typedef bool (*modem_ras_message_send_t)(void *priv, const uint8_t *message, size_t length);
typedef bool (*modem_ras_random_bytes_t)(uint8_t *buffer, uint8_t length);

typedef struct modem_ras_decoder_t {
    modem_ras_frame_callback_t callback;
    modem_ras_event_callback_t event_callback;
    void                      *priv;
    modem_ras_rx_state_t       state;
    uint8_t                    address;
    uint16_t                   length;
    uint16_t                   position;
    uint16_t                   crc;
    uint8_t                    received_crc_low;
    uint8_t                    payload[MODEM_RAS_MAX_PAYLOAD];
} modem_ras_decoder_t;

void modem_ras_decoder_init(modem_ras_decoder_t *decoder,
                            modem_ras_frame_callback_t callback,
                            modem_ras_event_callback_t event_callback, void *priv);
void modem_ras_rx_byte(modem_ras_decoder_t *decoder, uint8_t byte);

bool modem_ras_encode_frame(uint8_t address, const uint8_t *payload, size_t length,
                            uint8_t *output, size_t output_capacity, size_t *output_length);
bool modem_ras_send_nbf(modem_ras_serial_push_t serial_push, void *priv,
                        const uint8_t *frame, int length);
void modem_ras_session_init(modem_ras_session_t *session);
/* Negotiates and authenticates a legacy RAS response using the configured credentials. */
modem_ras_session_result_t modem_ras_session_process_message(
    modem_ras_session_t *session, const uint8_t *message, size_t length,
    const char *username, const char *password,
    modem_ras_message_send_t send, void *send_priv,
    modem_ras_random_bytes_t random_bytes);

#ifdef __cplusplus
}
#endif

#endif /* EMU_MODEM_RAS_H */
