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

typedef void (*modem_ras_frame_callback_t)(void *priv, uint8_t address,
                                           const uint8_t *payload, size_t length);
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

typedef struct modem_ras_decoder_t {
    modem_ras_frame_callback_t callback;
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
                            modem_ras_frame_callback_t callback, void *priv);
void modem_ras_rx_byte(modem_ras_decoder_t *decoder, uint8_t byte);

bool modem_ras_encode_frame(uint8_t address, const uint8_t *payload, size_t length,
                            uint8_t *output, size_t output_capacity, size_t *output_length);
bool modem_ras_send_nbf(modem_ras_serial_push_t serial_push, void *priv,
                        const uint8_t *frame, int length);

#ifdef __cplusplus
}
#endif

#endif /* EMU_MODEM_RAS_H */
