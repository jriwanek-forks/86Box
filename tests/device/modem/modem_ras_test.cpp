#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <vector>
#include <86box/modem/modem_ras.h>

struct ras_capture_t {
    int                    frames = 0;
    uint8_t                address = 0;
    std::vector<uint8_t>   payload;
    std::vector<modem_ras_event_t> events;
    uint16_t               event_length = 0;
    uint16_t               event_actual = 0;
    uint16_t               event_expected = 0;
};

struct ras_message_capture_t {
    std::vector<std::vector<uint8_t>> messages;
};

static void
capture_ras_frame(void *priv, uint8_t address, const uint8_t *payload, size_t length)
{
    auto *capture = static_cast<ras_capture_t *>(priv);
    capture->frames++;
    capture->address = address;
    capture->payload.assign(payload, payload + length);
}

static void
capture_ras_event(void *priv, modem_ras_event_t event, uint8_t, uint16_t length,
                  uint16_t actual, uint16_t expected)
{
    auto *capture = static_cast<ras_capture_t *>(priv);
    capture->events.push_back(event);
    capture->event_length = length;
    capture->event_actual = actual;
    capture->event_expected = expected;
}

static void
capture_serial_bytes(void *priv, const uint8_t *data, int length)
{
    auto *capture = static_cast<std::vector<uint8_t> *>(priv);
    capture->assign(data, data + length);
}

static bool
capture_ras_message(void *priv, const uint8_t *message, size_t length)
{
    auto *capture = static_cast<ras_message_capture_t *>(priv);
    capture->messages.emplace_back(message, message + length);
    return true;
}

static bool
fill_ras_challenge(uint8_t *buffer, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
        buffer[i] = i;
    return true;
}

static bool
fail_ras_challenge(uint8_t *, uint8_t)
{
    return false;
}

static std::vector<uint8_t>
encode_ras_frame(uint8_t address, const std::vector<uint8_t> &payload)
{
    std::vector<uint8_t> output(payload.size() + 7);
    size_t output_length = 0;
    EXPECT_TRUE(modem_ras_encode_frame(address, payload.data(), payload.size(),
                                       output.data(), output.size(), &output_length));
    output.resize(output_length);
    return output;
}

TEST(ModemRas, EncodesTheDocumentedUncompressedFrameLayout)
{
    const uint8_t payload[] = { 0xF0, 0xF0, 0x03 };
    const uint8_t expected[] = { 0x16, 0x02, 0x00, 0x03, 0xF0, 0xF0, 0x03, 0x03, 0x1E, 0x97 };
    uint8_t output[sizeof(expected)] = {};
    size_t output_length = 0;

    ASSERT_TRUE(modem_ras_encode_frame(MODEM_RAS_DESTINATION, payload, sizeof(payload),
                                       output, sizeof(output), &output_length));
    ASSERT_EQ(output_length, sizeof(expected));
    EXPECT_EQ(std::vector<uint8_t>(output, output + output_length),
              std::vector<uint8_t>(expected, expected + sizeof(expected)));
}

TEST(ModemRas, SendsNbfPayloadAsDestinationFrame)
{
    const std::vector<uint8_t> payload = { 0xF0, 0xF0, 0x03 };
    std::vector<uint8_t> wire;

    ASSERT_TRUE(modem_ras_send_nbf(capture_serial_bytes, &wire, payload.data(),
                                   (int) payload.size()));
    EXPECT_EQ(wire, encode_ras_frame(MODEM_RAS_DESTINATION, payload));
}

TEST(ModemRas, NegotiatesVersionAndIssuesChallenge)
{
    const uint8_t protocol_request[13] = { MODEM_RAS_FRAME_PROTOCOL, 20, 0 };
    modem_ras_session_t session;
    ras_message_capture_t capture;
    modem_ras_session_init(&session);

    EXPECT_EQ(modem_ras_session_process_message(&session, protocol_request,
                                                sizeof(protocol_request), capture_ras_message,
                                                &capture, fill_ras_challenge),
              MODEM_RAS_SESSION_PROTOCOL_NEGOTIATED);
    ASSERT_EQ(capture.messages.size(), 2);
    ASSERT_EQ(capture.messages[0].size(), sizeof(protocol_request));
    EXPECT_EQ(capture.messages[0][0], MODEM_RAS_FRAME_PROTOCOL);
    EXPECT_EQ(capture.messages[0][1], MODEM_RAS_PROTOCOL_VERSION_20);
    EXPECT_EQ(capture.messages[0][2], 0);
    EXPECT_EQ(capture.messages[1][0], MODEM_RAS_FRAME_CHALLENGE);
    for (uint8_t i = 0; i < MODEM_RAS_CHALLENGE_LENGTH; i++)
        EXPECT_EQ(capture.messages[1][i + 1], i);
    EXPECT_EQ(session.client_version, 20);
    EXPECT_EQ(session.state, MODEM_RAS_SESSION_WAIT_RESPONSE);
}

TEST(ModemRas, RejectsChallengeResponseWithoutAuthenticating)
{
    const uint8_t protocol_request[13] = { MODEM_RAS_FRAME_PROTOCOL, 20, 0 };
    const uint8_t challenge_response[] = { MODEM_RAS_FRAME_RESPONSE_20 };
    modem_ras_session_t session;
    ras_message_capture_t capture;
    modem_ras_session_init(&session);

    ASSERT_EQ(modem_ras_session_process_message(&session, protocol_request,
                                                sizeof(protocol_request), capture_ras_message,
                                                &capture, fill_ras_challenge),
              MODEM_RAS_SESSION_PROTOCOL_NEGOTIATED);
    EXPECT_EQ(modem_ras_session_process_message(&session, challenge_response,
                                                sizeof(challenge_response), capture_ras_message,
                                                &capture, fill_ras_challenge),
              MODEM_RAS_SESSION_AUTH_REJECTED);
    ASSERT_EQ(capture.messages.size(), 3);
    EXPECT_EQ(capture.messages[2],
              (std::vector<uint8_t> { MODEM_RAS_FRAME_RESULT, 0x00, 0x62 }));
    EXPECT_EQ(session.state, MODEM_RAS_SESSION_REJECTED);
}

TEST(ModemRas, DecodesByteByByteAndPreservesPayloadBytes)
{
    const std::vector<uint8_t> payload = { 0xF0, 0xF0, 0x03, 0x16, 0x00, 0x7E };
    const auto wire = encode_ras_frame(MODEM_RAS_BROADCAST, payload);
    ras_capture_t capture;
    modem_ras_decoder_t decoder;
    modem_ras_decoder_init(&decoder, capture_ras_frame, capture_ras_event, &capture);

    for (uint8_t byte : wire)
        modem_ras_rx_byte(&decoder, byte);

    EXPECT_EQ(capture.frames, 1);
    EXPECT_EQ(capture.address, MODEM_RAS_BROADCAST);
    EXPECT_EQ(capture.payload, payload);
    EXPECT_EQ(capture.events, std::vector<modem_ras_event_t> { MODEM_RAS_EVENT_FRAME_RECEIVED });
}

TEST(ModemRas, DropsBadCrcAndAcceptsFollowingFrame)
{
    const std::vector<uint8_t> payload = { 0xF0, 0xF0, 0x03, 0x01 };
    auto invalid = encode_ras_frame(MODEM_RAS_DESTINATION, payload);
    invalid.back() ^= 0x80;
    const auto valid = encode_ras_frame(MODEM_RAS_DESTINATION, payload);
    ras_capture_t capture;
    modem_ras_decoder_t decoder;
    modem_ras_decoder_init(&decoder, capture_ras_frame, capture_ras_event, &capture);

    for (uint8_t byte : invalid)
        modem_ras_rx_byte(&decoder, byte);
    EXPECT_EQ(capture.frames, 0);
    ASSERT_EQ(capture.events.size(), 1);
    EXPECT_EQ(capture.events[0], MODEM_RAS_EVENT_BAD_CRC);
    EXPECT_EQ(capture.event_length, static_cast<uint16_t>(payload.size()));
    EXPECT_NE(capture.event_actual, capture.event_expected);

    for (uint8_t byte : valid)
        modem_ras_rx_byte(&decoder, byte);
    EXPECT_EQ(capture.frames, 1);
    EXPECT_EQ(capture.payload, payload);
    ASSERT_EQ(capture.events.size(), 2);
    EXPECT_EQ(capture.events[1], MODEM_RAS_EVENT_FRAME_RECEIVED);
}

TEST(ModemRas, ReportsInvalidAddressLengthAndEtx)
{
    const uint8_t invalid_address[] = { 0x16, 0x22 };
    const uint8_t oversized_length[] = { 0x16, 0x02, 0x05, 0xDD };
    auto invalid_etx = encode_ras_frame(MODEM_RAS_DESTINATION, { 0xF0, 0xF0, 0x03 });
    invalid_etx[7] = 0x04;
    ras_capture_t capture;
    modem_ras_decoder_t decoder;
    modem_ras_decoder_init(&decoder, capture_ras_frame, capture_ras_event, &capture);

    for (uint8_t byte : invalid_address)
        modem_ras_rx_byte(&decoder, byte);
    for (uint8_t byte : oversized_length)
        modem_ras_rx_byte(&decoder, byte);
    EXPECT_EQ(capture.frames, 0);
    ASSERT_EQ(capture.events.size(), 2);
    EXPECT_EQ(capture.event_actual, 0x05DD);
    EXPECT_EQ(capture.event_expected, MODEM_RAS_MAX_PAYLOAD);

    for (uint8_t byte : invalid_etx)
        modem_ras_rx_byte(&decoder, byte);
    EXPECT_EQ(capture.frames, 0);
    ASSERT_EQ(capture.events.size(), 3);
    EXPECT_EQ(capture.events[0], MODEM_RAS_EVENT_INVALID_ADDRESS);
    EXPECT_EQ(capture.events[1], MODEM_RAS_EVENT_INVALID_LENGTH);
    EXPECT_EQ(capture.events[2], MODEM_RAS_EVENT_INVALID_ETX);
}

TEST(ModemRas, EnforcesPayloadAndOutputCapacityLimits)
{
    std::vector<uint8_t> payload(MODEM_RAS_MAX_PAYLOAD, 0x55);
    std::vector<uint8_t> output(payload.size() + 7);
    size_t output_length = 0;

    EXPECT_TRUE(modem_ras_encode_frame(MODEM_RAS_DESTINATION, payload.data(), payload.size(),
                                        output.data(), output.size(), &output_length));
    EXPECT_EQ(output_length, output.size());
    EXPECT_FALSE(modem_ras_encode_frame(MODEM_RAS_DESTINATION, payload.data(), payload.size(),
                                        output.data(), output.size() - 1, &output_length));
    EXPECT_EQ(output_length, 0);
    payload.push_back(0x55);
    EXPECT_FALSE(modem_ras_encode_frame(MODEM_RAS_DESTINATION, payload.data(), payload.size(),
                                        output.data(), output.size(), &output_length));
    EXPECT_EQ(output_length, 0);
}
