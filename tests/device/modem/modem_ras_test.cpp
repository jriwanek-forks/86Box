#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <vector>
#include <86box/modem/modem_ras.h>

struct ras_capture_t {
    int                    frames = 0;
    uint8_t                address = 0;
    std::vector<uint8_t>   payload;
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
capture_serial_bytes(void *priv, const uint8_t *data, int length)
{
    auto *capture = static_cast<std::vector<uint8_t> *>(priv);
    capture->assign(data, data + length);
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

TEST(ModemRas, DecodesByteByByteAndPreservesPayloadBytes)
{
    const std::vector<uint8_t> payload = { 0xF0, 0xF0, 0x03, 0x16, 0x00, 0x7E };
    const auto wire = encode_ras_frame(MODEM_RAS_BROADCAST, payload);
    ras_capture_t capture;
    modem_ras_decoder_t decoder;
    modem_ras_decoder_init(&decoder, capture_ras_frame, &capture);

    for (uint8_t byte : wire)
        modem_ras_rx_byte(&decoder, byte);

    EXPECT_EQ(capture.frames, 1);
    EXPECT_EQ(capture.address, MODEM_RAS_BROADCAST);
    EXPECT_EQ(capture.payload, payload);
}

TEST(ModemRas, DropsBadCrcAndAcceptsFollowingFrame)
{
    const std::vector<uint8_t> payload = { 0xF0, 0xF0, 0x03, 0x01 };
    auto invalid = encode_ras_frame(MODEM_RAS_DESTINATION, payload);
    invalid.back() ^= 0x80;
    const auto valid = encode_ras_frame(MODEM_RAS_DESTINATION, payload);
    ras_capture_t capture;
    modem_ras_decoder_t decoder;
    modem_ras_decoder_init(&decoder, capture_ras_frame, &capture);

    for (uint8_t byte : invalid)
        modem_ras_rx_byte(&decoder, byte);
    EXPECT_EQ(capture.frames, 0);

    for (uint8_t byte : valid)
        modem_ras_rx_byte(&decoder, byte);
    EXPECT_EQ(capture.frames, 1);
    EXPECT_EQ(capture.payload, payload);
}

TEST(ModemRas, RejectsInvalidAddressAndOversizedPayload)
{
    const uint8_t invalid_address[] = { 0x16, 0x22 };
    const uint8_t oversized_length[] = { 0x16, 0x02, 0x05, 0xDD };
    ras_capture_t capture;
    modem_ras_decoder_t decoder;
    modem_ras_decoder_init(&decoder, capture_ras_frame, &capture);

    for (uint8_t byte : invalid_address)
        modem_ras_rx_byte(&decoder, byte);
    for (uint8_t byte : oversized_length)
        modem_ras_rx_byte(&decoder, byte);
    EXPECT_EQ(capture.frames, 0);
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
