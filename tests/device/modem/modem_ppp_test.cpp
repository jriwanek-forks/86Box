#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <cstring>
#include <gtest/gtest.h>
#include <vector>
#include <86box/modem/modem_cslip.h>
#include <86box/modem/modem_ppp.h>
#include <86box/modem/modem_mppe.h>
#include <86box/modem/modem_pap.h>

static void
discard_serial(void *, const uint8_t *, int)
{
}

static void
capture_serial(void *priv, const uint8_t *data, int len)
{
    auto *output = static_cast<std::vector<uint8_t> *>(priv);
    output->insert(output->end(), data, data + len);
}

static void
send_frame(ppp_ctx_t *ctx, const std::vector<uint8_t> &packet, bool corrupt_fcs = false,
           uint16_t protocol = PPP_PROTO_LCP)
{
    std::vector<uint8_t> raw = { PPP_ADDRESS, PPP_CONTROL,
                                 static_cast<uint8_t>(protocol >> 8),
                                 static_cast<uint8_t>(protocol) };
    raw.insert(raw.end(), packet.begin(), packet.end());
    uint16_t fcs = ppp_fcs16(raw.data(), static_cast<int>(raw.size()));
    raw.push_back(static_cast<uint8_t>(fcs));
    raw.push_back(static_cast<uint8_t>(fcs >> 8));
    if (corrupt_fcs)
        raw.back() ^= 1;

    ppp_rx_byte(ctx, PPP_FLAG);
    for (uint8_t byte : raw) {
        if (byte == PPP_FLAG || byte == PPP_ESCAPE) {
            ppp_rx_byte(ctx, PPP_ESCAPE);
            ppp_rx_byte(ctx, byte ^ 0x20);
        } else {
            ppp_rx_byte(ctx, byte);
        }
    }
    ppp_rx_byte(ctx, PPP_FLAG);
}

static void
capture_network(void *priv, const uint8_t *data, int len)
{
    auto *output = static_cast<std::vector<uint8_t> *>(priv);
    output->assign(data, data + len);
}

static void
capture_network_packets(void *priv, const uint8_t *data, int len)
{
    auto *output = static_cast<std::vector<std::vector<uint8_t>> *>(priv);
    output->emplace_back(data, data + len);
}

struct ras_capture_t {
    std::vector<uint8_t> serial;
    std::vector<std::vector<uint8_t>> packets;
};

static void
capture_ras_serial(void *priv, const uint8_t *data, int len)
{
    auto *capture = static_cast<ras_capture_t *>(priv);
    capture->serial.insert(capture->serial.end(), data, data + len);
}

static void
capture_ras_network(void *priv, const uint8_t *data, int len)
{
    auto *capture = static_cast<ras_capture_t *>(priv);
    capture->packets.emplace_back(data, data + len);
}

static std::vector<uint8_t>
make_tcp_packet(uint16_t ip_id, uint32_t sequence)
{
    std::vector<uint8_t> packet(40, 0);
    packet[0] = 0x45;
    packet[3] = static_cast<uint8_t>(packet.size());
    packet[4] = static_cast<uint8_t>(ip_id >> 8);
    packet[5] = static_cast<uint8_t>(ip_id);
    packet[8] = 64;
    packet[9] = 6;
    packet[12] = 10;
    packet[15] = 1;
    packet[16] = 10;
    packet[19] = 2;
    packet[20] = 0x04;
    packet[21] = 0xD2;
    packet[23] = 80;
    packet[24] = static_cast<uint8_t>(sequence >> 24);
    packet[25] = static_cast<uint8_t>(sequence >> 16);
    packet[26] = static_cast<uint8_t>(sequence >> 8);
    packet[27] = static_cast<uint8_t>(sequence);
    packet[32] = 0x50;
    packet[33] = 0x10;
    packet[34] = 0x10;
    uint32_t sum = 0;
    for (size_t pos = 0; pos < 20; pos += 2)
        sum += (static_cast<uint16_t>(packet[pos]) << 8) | packet[pos + 1];
    while (sum >> 16)
        sum = (sum & 0xFFFF) + (sum >> 16);
    uint16_t checksum = static_cast<uint16_t>(~sum);
    packet[10] = static_cast<uint8_t>(checksum >> 8);
    packet[11] = static_cast<uint8_t>(checksum);
    return packet;
}

static std::vector<uint8_t>
decode_captured_frame(const std::vector<uint8_t> &wire, uint16_t &protocol)
{
    std::vector<uint8_t> raw;
    for (size_t pos = 1; pos + 1 < wire.size(); pos++) {
        uint8_t byte = wire[pos];
        if (byte == PPP_ESCAPE)
            byte = wire[++pos] ^ 0x20;
        raw.push_back(byte);
    }
    if (raw.size() < 6)
        return {};
    protocol = static_cast<uint16_t>((raw[2] << 8) | raw[3]);
    return { raw.begin() + 4, raw.end() - 2 };
}

static ppp_ctx_t make_context();

TEST(ModemPpp, RequiresLowestSelectedMppeStrength)
{
    EXPECT_EQ(ppp_mppe_minimum_strength(0), 0);
    EXPECT_EQ(ppp_mppe_minimum_strength(CCP_MPPE_40), 40);
    EXPECT_EQ(ppp_mppe_minimum_strength(CCP_MPPE_56), 56);
    EXPECT_EQ(ppp_mppe_minimum_strength(CCP_MPPE_128), 128);
    EXPECT_EQ(ppp_mppe_minimum_strength(CCP_MPPE_40 | CCP_MPPE_128), 40);
    EXPECT_EQ(ppp_mppe_minimum_strength(CCP_MPPE_56 | CCP_MPPE_128), 56);
    EXPECT_EQ(ppp_mppe_minimum_strength(CCP_MPPE_KEY_BITS), 40);
}

TEST(ModemPpp, FcsMatchesStandardCheckValue)
{
    const uint8_t check[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    EXPECT_EQ(ppp_fcs16(check, sizeof(check)), 0x906Eu);
}

TEST(ModemPpp, MultilinkLcpOptionsAreOptIn)
{
    std::vector<uint8_t> unconfigured_wire;
    std::vector<uint8_t> configured_wire;
    std::vector<uint8_t> network;
    ppp_ctx_t *unconfigured = ppp_init(&unconfigured_wire, nullptr,
                                       capture_serial, capture_network);
    ppp_ctx_t *configured = ppp_init(&configured_wire, nullptr,
                                     capture_serial, capture_network);
    ASSERT_NE(unconfigured, nullptr);
    ASSERT_NE(configured, nullptr);

    ppp_start(unconfigured);
    ppp_multilink_configure(configured, "modem-pair");
    ppp_start(configured);

    uint16_t protocol = 0;
    auto ordinary_request = decode_captured_frame(unconfigured_wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_LCP);
    bool ordinary_has_mrru = false;
    for (size_t pos = 4; pos + 2 <= ordinary_request.size();) {
        uint8_t option_length = ordinary_request[pos + 1];
        ASSERT_GE(option_length, 2);
        ASSERT_LE(pos + option_length, ordinary_request.size());
        ordinary_has_mrru |= ordinary_request[pos] == LCP_OPT_MRRU;
        pos += option_length;
    }
    EXPECT_FALSE(ordinary_has_mrru);

    auto multilink_request = decode_captured_frame(configured_wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_LCP);
    bool has_mrru = false;
    bool has_short_sequence = false;
    bool has_endpoint = false;
    for (size_t pos = 4; pos + 2 <= multilink_request.size();) {
        uint8_t option_length = multilink_request[pos + 1];
        ASSERT_GE(option_length, 2);
        ASSERT_LE(pos + option_length, multilink_request.size());
        has_mrru |= multilink_request[pos] == LCP_OPT_MRRU;
        has_short_sequence |= multilink_request[pos] == LCP_OPT_SHORT_SEQUENCE;
        has_endpoint |= multilink_request[pos] == LCP_OPT_ENDPOINT_DISC;
        pos += option_length;
    }
    EXPECT_TRUE(has_mrru);
    EXPECT_TRUE(has_short_sequence);
    EXPECT_TRUE(has_endpoint);

    ppp_close(unconfigured);
    ppp_close(configured);
}

TEST(ModemPpp, SendsEscapedFrameWithValidFcs)
{
    ppp_ctx_t ctx = make_context();
    std::vector<uint8_t> wire;
    ctx.modem = &wire;
    ctx.serial_push = capture_serial;
    ctx.peer_accm = 1u << 0x11;
    const std::vector<uint8_t> payload = { PPP_FLAG, PPP_ESCAPE, 0x11 };

    ppp_send_frame(&ctx, PPP_PROTO_LCP, payload.data(), static_cast<int>(payload.size()));

    ASSERT_GE(wire.size(), 2u);
    EXPECT_EQ(wire.front(), PPP_FLAG);
    EXPECT_EQ(wire.back(), PPP_FLAG);
    std::vector<uint8_t> raw;
    for (size_t pos = 1; pos + 1 < wire.size(); pos++) {
        uint8_t byte = wire[pos];
        if (byte == PPP_ESCAPE) {
            ASSERT_LT(pos + 1, wire.size() - 1);
            byte = wire[++pos] ^ 0x20;
        } else {
            EXPECT_NE(byte, PPP_FLAG);
        }
        raw.push_back(byte);
    }

    ASSERT_GE(raw.size(), 6u);
    uint16_t received_fcs = static_cast<uint16_t>(raw[raw.size() - 2])
                          | (static_cast<uint16_t>(raw.back()) << 8);
    EXPECT_EQ(ppp_fcs16(raw.data(), static_cast<int>(raw.size() - 2)), received_fcs);
    const std::vector<uint8_t> expected = { PPP_ADDRESS, PPP_CONTROL, 0xC0, 0x21,
                                            PPP_FLAG, PPP_ESCAPE, 0x11 };
    EXPECT_TRUE(std::equal(expected.begin(), expected.end(), raw.begin()));
}

TEST(ModemPpp, EnforcesNegotiatedMruInBothDirections)
{
    ppp_ctx_t tx = make_context();
    std::vector<uint8_t> wire;
    tx.state = PPP_STATE_NETWORK;
    tx.modem = &wire;
    tx.serial_push = capture_serial;
    tx.peer_mru = 6;
    const std::vector<uint8_t> oversized = { 1, 2, 3, 4, 5 };

    ppp_send_frame(&tx, PPP_PROTO_IP, oversized.data(), static_cast<int>(oversized.size()));

    EXPECT_TRUE(wire.empty());

    ppp_ctx_t rx = make_context();
    std::vector<uint8_t> received;
    rx.state = PPP_STATE_NETWORK;
    rx.modem = &received;
    rx.network_send_ip = capture_network;
    rx.our_mru = 6;
    send_frame(&rx, oversized, false, PPP_PROTO_IP);
    EXPECT_TRUE(received.empty());
}

TEST(ModemPpp, NaksMruBelowRfcMinimum)
{
    ppp_ctx_t ctx = make_context();
    std::vector<uint8_t> wire;
    ctx.modem = &wire;
    ctx.serial_push = capture_serial;
    const std::vector<uint8_t> request = {
        PPP_CODE_CONFIGURE_REQUEST, 3, 0, 8,
        LCP_OPT_MRU, 4, 0, 64
    };

    send_frame(&ctx, request);

    uint16_t protocol = 0;
    const std::vector<uint8_t> response = decode_captured_frame(wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_LCP);
    ASSERT_EQ(response.size(), 8u);
    EXPECT_EQ(response[0], PPP_CODE_CONFIGURE_NAK);
    EXPECT_EQ(response[4], LCP_OPT_MRU);
    EXPECT_EQ(response[6], static_cast<uint8_t>(PPP_DEFAULT_MRU >> 8));
    EXPECT_EQ(response[7], static_cast<uint8_t>(PPP_DEFAULT_MRU));
    EXPECT_FALSE(ctx.lcp_ack_sent);
}

TEST(ModemPpp, SendsTerminateRequestWhenClosingActiveSession)
{
    std::vector<uint8_t> wire;
    ppp_ctx_t *ctx = ppp_init(&wire, nullptr, capture_serial, capture_network);
    ASSERT_NE(ctx, nullptr);
    ctx->state = PPP_STATE_NETWORK;

    ppp_close(ctx);

    uint16_t protocol = 0;
    const std::vector<uint8_t> payload = decode_captured_frame(wire, protocol);
    EXPECT_EQ(protocol, PPP_PROTO_LCP);
    ASSERT_EQ(payload.size(), 4u);
    EXPECT_EQ(payload[0], PPP_CODE_TERMINATE_REQUEST);
    EXPECT_EQ(payload[2], 0);
    EXPECT_EQ(payload[3], 4);
}

TEST(ModemPpp, LzsExtendedResynchronizesAfterCoherencyMismatch)
{
    ppp_ctx_t tx = make_context();
    ppp_ctx_t rx = make_context();
    std::vector<uint8_t> tx_wire;
    std::vector<uint8_t> rx_wire;
    std::vector<uint8_t> packet(128);
    for (size_t index = 0; index < packet.size(); index++)
        packet[index] = static_cast<uint8_t>((index / 8) % 4);

    tx.state = PPP_STATE_NETWORK;
    tx.modem = &tx_wire;
    tx.serial_push = capture_serial;
    tx.ccp_open = true;
    tx.ccp_tx_method = PPP_CCP_METHOD_LZS_EXTENDED;
    tx.peer_pfc = true;
    ASSERT_TRUE(ppp_ccp_codec_set(&tx, true, PPP_CCP_METHOD_LZS_EXTENDED));

    rx.state = PPP_STATE_NETWORK;
    rx.modem = &rx_wire;
    rx.serial_push = capture_serial;
    rx.network_send_ip = capture_network;
    rx.ccp_open = true;
    rx.ccp_rx_method = PPP_CCP_METHOD_LZS_EXTENDED;
    rx.our_pfc = true;
    ASSERT_TRUE(ppp_ccp_codec_set(&rx, false, PPP_CCP_METHOD_LZS_EXTENDED));

    ppp_send_frame(&tx, PPP_PROTO_IP, packet.data(), static_cast<int>(packet.size()));
    uint16_t protocol = 0;
    std::vector<uint8_t> payload = decode_captured_frame(tx_wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_MPPE);
    EXPECT_EQ(payload[0] & 0xA0, 0xA0);
    send_frame(&rx, payload, false, protocol);
    EXPECT_EQ(rx_wire, packet);

    tx_wire.clear();
    rx_wire.clear();
    ppp_send_frame(&tx, PPP_PROTO_IP, packet.data(), static_cast<int>(packet.size()));
    payload = decode_captured_frame(tx_wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_MPPE);
    EXPECT_EQ(payload[0] & 0x80, 0);
    ASSERT_EQ(payload[1], 1);
    payload[1] = 2;
    send_frame(&rx, payload, false, protocol);
    EXPECT_TRUE(rx.ccp_reset_pending);

    std::vector<uint8_t> reset = decode_captured_frame(rx_wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_CCP);
    ASSERT_EQ(reset.size(), 6u);
    EXPECT_EQ(reset[0], PPP_CODE_RESET_REQUEST);
    EXPECT_EQ(reset[4], 0);
    EXPECT_EQ(reset[5], 1);

    tx_wire.clear();
    send_frame(&tx, reset, false, PPP_PROTO_CCP);
    EXPECT_TRUE(tx_wire.empty());

    tx_wire.clear();
    ppp_send_frame(&tx, PPP_PROTO_IP, packet.data(), static_cast<int>(packet.size()));
    payload = decode_captured_frame(tx_wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_MPPE);
    EXPECT_NE(payload[0] & 0x80, 0);
    ASSERT_EQ(payload[1], 2);
    rx_wire.clear();
    send_frame(&rx, payload, false, protocol);
    EXPECT_FALSE(rx.ccp_reset_pending);
    EXPECT_EQ(rx_wire, packet);

    ppp_ccp_codec_close(&tx);
    ppp_ccp_codec_close(&rx);
}

TEST(ModemPpp, RoundTripsNegotiatedVjPacketsOverPpp)
{
    ppp_ctx_t tx = make_context();
    ppp_ctx_t rx = make_context();
    std::vector<uint8_t> wire;
    std::vector<uint8_t> received;
    tx.state = PPP_STATE_NETWORK;
    tx.modem = &wire;
    tx.serial_push = capture_serial;
    tx.vj_tx_ctx = cslip_init(nullptr);
    tx.vj_tx_enabled = true;
    tx.vj_tx_max_slot_id = IPCP_VJ_MAX_SLOT_ID;
    rx.state = PPP_STATE_NETWORK;
    rx.modem = &received;
    rx.network_send_ip = capture_network;
    rx.vj_rx_ctx = cslip_init(nullptr);
    rx.vj_rx_enabled = true;
    rx.vj_rx_max_slot_id = IPCP_VJ_MAX_SLOT_ID;
    ASSERT_NE(tx.vj_tx_ctx, nullptr);
    ASSERT_NE(rx.vj_rx_ctx, nullptr);

    auto first = make_tcp_packet(1, 100);
    ppp_wrap_ip(&tx, first.data(), static_cast<int>(first.size()));
    uint16_t protocol = 0;
    std::vector<uint8_t> payload = decode_captured_frame(wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_VJ_UNCOMPRESSED);
    ASSERT_LE(payload[9], IPCP_VJ_MAX_SLOT_ID);
    send_frame(&rx, payload, false, protocol);
    EXPECT_EQ(received, first);

    wire.clear();
    auto second = make_tcp_packet(2, 101);
    second[31] = 1;
    second[10] = 0;
    second[11] = 0;
    uint32_t sum = 0;
    for (size_t pos = 0; pos < 20; pos += 2)
        sum += (static_cast<uint16_t>(second[pos]) << 8) | second[pos + 1];
    while (sum >> 16)
        sum = (sum & 0xFFFF) + (sum >> 16);
    uint16_t checksum = static_cast<uint16_t>(~sum);
    second[10] = static_cast<uint8_t>(checksum >> 8);
    second[11] = static_cast<uint8_t>(checksum);
    ppp_wrap_ip(&tx, second.data(), static_cast<int>(second.size()));
    payload = decode_captured_frame(wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_VJ_COMPRESSED);
    send_frame(&rx, payload, false, protocol);
    EXPECT_EQ(received, second);

    cslip_close(tx.vj_tx_ctx);
    cslip_close(rx.vj_rx_ctx);
}

TEST(ModemPpp, DropsOversizedIpPacketBeforeVjCompression)
{
    ppp_ctx_t ctx = make_context();
    std::vector<uint8_t> wire;
    std::vector<uint8_t> oversized(PPP_MAX_FRAME + 1, 0);
    ctx.state = PPP_STATE_NETWORK;
    ctx.modem = &wire;
    ctx.serial_push = capture_serial;
    ctx.vj_tx_ctx = cslip_init(nullptr);
    ctx.vj_tx_enabled = true;
    ASSERT_NE(ctx.vj_tx_ctx, nullptr);

    ppp_wrap_ip(&ctx, oversized.data(), static_cast<int>(oversized.size()));

    EXPECT_TRUE(wire.empty());
    cslip_close(ctx.vj_tx_ctx);
}

TEST(ModemPpp, EncryptsNegotiatedPayloadAndRejectsPlaintext)
{
    ppp_ctx_t tx = make_context();
    ppp_ctx_t rx = make_context();
    std::vector<uint8_t> wire;
    std::vector<uint8_t> received;
    tx.state = PPP_STATE_NETWORK;
    tx.modem = &wire;
    tx.serial_push = capture_serial;
    tx.ccp_open = true;
    tx.mppe_keys_ready = true;
    tx.mppe_tx_enabled = true;
    ASSERT_TRUE(ppp_mppe_configure(&tx.mppe_tx, 128, false));
    rx.state = PPP_STATE_NETWORK;
    rx.modem = &received;
    rx.network_send_ip = capture_network;
    rx.ccp_open = true;
    rx.mppe_keys_ready = true;
    rx.mppe_rx_enabled = true;
    ASSERT_TRUE(ppp_mppe_configure(&rx.mppe_rx, 128, false));

    const std::vector<uint8_t> ip_packet = { 0x45, 0x00, 0x00, 0x04 };
    ppp_send_frame(&tx, PPP_PROTO_IP, ip_packet.data(), static_cast<int>(ip_packet.size()));

    uint16_t protocol = 0;
    std::vector<uint8_t> payload = decode_captured_frame(wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_MPPE);
    send_frame(&rx, payload, false, protocol);
    EXPECT_EQ(received, ip_packet);

    received.clear();
    send_frame(&rx, ip_packet, false, PPP_PROTO_IP);
    EXPECT_TRUE(received.empty());
}

TEST(ModemPpp, AppliesMppcBeforeMppeAndReversesBothOnReceive)
{
    ppp_ctx_t tx = make_context();
    ppp_ctx_t rx = make_context();
    std::vector<uint8_t> wire;
    std::vector<uint8_t> received;
    tx.state = PPP_STATE_NETWORK;
    tx.modem = &wire;
    tx.serial_push = capture_serial;
    tx.ccp_open = true;
    tx.ccp_tx_method = PPP_CCP_METHOD_MPPC;
    tx.mppe_keys_ready = true;
    tx.mppe_tx_enabled = true;
    ASSERT_TRUE(ppp_ccp_codec_set(&tx, true, PPP_CCP_METHOD_MPPC));
    ASSERT_TRUE(ppp_mppe_configure(&tx.mppe_tx, 128, false));
    rx.state = PPP_STATE_NETWORK;
    rx.modem = &received;
    rx.network_send_ip = capture_network;
    rx.ccp_open = true;
    rx.ccp_rx_method = PPP_CCP_METHOD_MPPC;
    rx.mppe_keys_ready = true;
    rx.mppe_rx_enabled = true;
    ASSERT_TRUE(ppp_ccp_codec_set(&rx, false, PPP_CCP_METHOD_MPPC));
    ASSERT_TRUE(ppp_mppe_configure(&rx.mppe_rx, 128, false));

    std::vector<uint8_t> ip_packet(128);
    for (size_t index = 0; index < ip_packet.size(); index++)
        ip_packet[index] = static_cast<uint8_t>((index / 8) % 4);
    ppp_send_frame(&tx, PPP_PROTO_IP, ip_packet.data(), static_cast<int>(ip_packet.size()));

    uint16_t protocol = 0;
    std::vector<uint8_t> payload = decode_captured_frame(wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_MPPE);
    send_frame(&rx, payload, false, protocol);
    EXPECT_EQ(received, ip_packet);
    ppp_ccp_codec_close(&tx);
    ppp_ccp_codec_close(&rx);
}

TEST(ModemPpp, TransportsStacLzsAndUsesNativeProtocolWhenItExpands)
{
    ppp_ctx_t tx = make_context();
    ppp_ctx_t rx = make_context();
    std::vector<uint8_t> wire;
    std::vector<uint8_t> received;
    tx.state = PPP_STATE_NETWORK;
    tx.modem = &wire;
    tx.serial_push = capture_serial;
    tx.ccp_open = true;
    tx.ccp_tx_method = PPP_CCP_METHOD_LZS;
    rx.state = PPP_STATE_NETWORK;
    rx.modem = &received;
    rx.network_send_ip = capture_network;
    rx.ccp_open = true;
    rx.ccp_rx_method = PPP_CCP_METHOD_LZS;
    ASSERT_TRUE(ppp_ccp_codec_set(&tx, true, PPP_CCP_METHOD_LZS));
    ASSERT_TRUE(ppp_ccp_codec_set(&rx, false, PPP_CCP_METHOD_LZS));

    std::vector<uint8_t> compressible(128);
    for (size_t index = 0; index < compressible.size(); index++)
        compressible[index] = static_cast<uint8_t>((index / 8) % 4);
    ppp_send_frame(&tx, PPP_PROTO_IP, compressible.data(),
                   static_cast<int>(compressible.size()));

    uint16_t protocol = 0;
    std::vector<uint8_t> payload = decode_captured_frame(wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_MPPE);
    send_frame(&rx, payload, false, protocol);
    EXPECT_EQ(received, compressible);

    wire.clear();
    received.clear();
    const std::vector<uint8_t> incompressible = { 0x45, 0xA3 };
    ppp_send_frame(&tx, PPP_PROTO_IP, incompressible.data(),
                   static_cast<int>(incompressible.size()));
    payload = decode_captured_frame(wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_IP);
    EXPECT_EQ(payload, incompressible);
    send_frame(&rx, payload, false, protocol);
    EXPECT_EQ(received, incompressible);

    ppp_ccp_codec_close(&tx);
    ppp_ccp_codec_close(&rx);
}

TEST(ModemPpp, ReplaysPapAckForDuplicateRequestAfterAuthentication)
{
    ppp_ctx_t ctx = make_context();
    std::vector<uint8_t> wire;
    ctx.state = PPP_STATE_NETWORK;
    ctx.modem = &wire;
    ctx.serial_push = capture_serial;
    ctx.auth_type = PPP_AUTH_PAP;
    ctx.auth_complete = true;
    const std::vector<uint8_t> request = {
        PAP_CODE_AUTHENTICATE_REQUEST, 9, 0, 8, 1, 'a', 1, 'b'
    };
    ctx.pap_last_request_len = static_cast<uint16_t>(request.size());
    std::memcpy(ctx.pap_last_request, request.data(), request.size());
    ctx.pap_last_request_valid = true;

    send_frame(&ctx, request, false, PPP_PROTO_PAP);

    uint16_t protocol = 0;
    const std::vector<uint8_t> response = decode_captured_frame(wire, protocol);
    EXPECT_EQ(protocol, PPP_PROTO_PAP);
    ASSERT_GE(response.size(), 2u);
    EXPECT_EQ(response[0], PAP_CODE_AUTHENTICATE_ACK);
    EXPECT_EQ(response[1], 9);
}

TEST(ModemPpp, TransportsV44AndUsesNativeProtocolWhenItExpands)
{
    ppp_ctx_t tx = make_context();
    ppp_ctx_t rx = make_context();
    std::vector<uint8_t> wire;
    std::vector<uint8_t> received;
    tx.state = PPP_STATE_NETWORK;
    tx.modem = &wire;
    tx.serial_push = capture_serial;
    tx.ccp_open = true;
    tx.ccp_tx_method = PPP_CCP_METHOD_V44;
    rx.state = PPP_STATE_NETWORK;
    rx.modem = &received;
    rx.network_send_ip = capture_network;
    rx.ccp_open = true;
    rx.ccp_rx_method = PPP_CCP_METHOD_V44;
    ASSERT_TRUE(ppp_ccp_codec_set(&tx, true, PPP_CCP_METHOD_V44));
    ASSERT_TRUE(ppp_ccp_codec_set(&rx, false, PPP_CCP_METHOD_V44));

    std::vector<uint8_t> compressible(128);
    std::vector<uint8_t> v44_plaintext = { static_cast<uint8_t>(PPP_PROTO_IP >> 8),
                                           static_cast<uint8_t>(PPP_PROTO_IP) };
    std::vector<uint8_t> v44_encoded(PPP_MAX_FRAME);
    std::vector<uint8_t> v44_decoded(PPP_MAX_FRAME);
    int v44_encoded_len;
    int v44_decoded_len;
    for (size_t index = 0; index < compressible.size(); index++)
        compressible[index] = static_cast<uint8_t>((index / 8) % 4);
    v44_plaintext.insert(v44_plaintext.end(), compressible.begin(), compressible.end());
    for (size_t length = 1; length <= v44_plaintext.size(); length++) {
        SCOPED_TRACE(::testing::Message() << "V.44 input length=" << length);
        ASSERT_TRUE(ppp_ccp_codec_compress(&tx, v44_plaintext.data(),
                                           static_cast<int>(length),
                                           v44_encoded.data(), v44_encoded.size(),
                                           &v44_encoded_len));
        ASSERT_TRUE(ppp_ccp_codec_decompress(&rx, v44_encoded.data(), v44_encoded_len,
                                             v44_decoded.data(), v44_decoded.size(),
                                             &v44_decoded_len)) << "input length=" << length;
        ASSERT_EQ(v44_decoded_len, length);
        EXPECT_TRUE(std::equal(v44_plaintext.begin(), v44_plaintext.begin() + length,
                               v44_decoded.begin()));
    }

    ppp_send_frame(&tx, PPP_PROTO_IP, compressible.data(),
                   static_cast<int>(compressible.size()));

    uint16_t protocol = 0;
    std::vector<uint8_t> payload = decode_captured_frame(wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_MPPE);
    send_frame(&rx, payload, false, protocol);
    EXPECT_EQ(received, compressible);

    wire.clear();
    received.clear();
    const std::vector<uint8_t> incompressible = { 0x45, 0xA3 };
    ppp_send_frame(&tx, PPP_PROTO_IP, incompressible.data(),
                   static_cast<int>(incompressible.size()));
    payload = decode_captured_frame(wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_IP);
    EXPECT_EQ(payload, incompressible);
    send_frame(&rx, payload, false, protocol);
    EXPECT_EQ(received, incompressible);

    ppp_ccp_codec_close(&tx);
    ppp_ccp_codec_close(&rx);
}

TEST(ModemPpp, DeliversFragmentedAndConcatenatedPredictor2Records)
{
    ppp_ctx_t tx = make_context();
    ppp_ctx_t rx = make_context();
    std::vector<std::vector<uint8_t>> received;
    std::array<uint8_t, 160> encoded{};
    std::vector<uint8_t> stream;
    int encoded_len;
    int first_encoded_len;

    tx.ccp_tx_method = PPP_CCP_METHOD_PREDICTOR2;
    ASSERT_TRUE(ppp_ccp_codec_set(&tx, true, PPP_CCP_METHOD_PREDICTOR2));
    ASSERT_TRUE(ppp_ccp_codec_set(&rx, false, PPP_CCP_METHOD_PREDICTOR2));
    rx.state = PPP_STATE_NETWORK;
    rx.ccp_open = true;
    rx.ccp_rx_method = PPP_CCP_METHOD_PREDICTOR2;
    rx.modem = &received;
    rx.network_send_ip = capture_network_packets;

    std::vector<uint8_t> first_packet = { 0x00, 0x21 };
    std::vector<uint8_t> second_packet = { 0x00, 0x21 };
    for (uint16_t index = 0; index < 128; index++) {
        first_packet.push_back(static_cast<uint8_t>((index / 8) % 4));
        second_packet.push_back(static_cast<uint8_t>(index * 37 + 11));
    }

    ASSERT_TRUE(ppp_ccp_codec_compress(&tx, first_packet.data(), first_packet.size(),
                                       encoded.data(), encoded.size(), &encoded_len));
    first_encoded_len = encoded_len;
    stream.insert(stream.end(), encoded.begin(), encoded.begin() + encoded_len);
    ASSERT_TRUE(ppp_ccp_codec_compress(&tx, second_packet.data(), second_packet.size(),
                                       encoded.data(), encoded.size(), &encoded_len));
    stream.insert(stream.end(), encoded.begin(), encoded.begin() + encoded_len);

    const size_t split = static_cast<size_t>(first_encoded_len / 2);
    ASSERT_GT(split, 2u);
    send_frame(&rx, { stream.begin(), stream.begin() + split }, false, PPP_PROTO_MPPE);
    EXPECT_TRUE(received.empty());
    EXPECT_FALSE(rx.ccp_reset_pending);

    send_frame(&rx, { stream.begin() + split, stream.end() }, false, PPP_PROTO_MPPE);
    ASSERT_EQ(received.size(), 2u);
    EXPECT_EQ(received[0], std::vector<uint8_t>(first_packet.begin() + 2, first_packet.end()));
    EXPECT_EQ(received[1], std::vector<uint8_t>(second_packet.begin() + 2, second_packet.end()));
    EXPECT_FALSE(rx.ccp_reset_pending);

    ppp_ccp_codec_close(&tx);
    ppp_ccp_codec_close(&rx);
}

TEST(ModemPpp, RecoversMppcAfterMalformedPacketReset)
{
    ppp_ctx_t tx = make_context();
    ppp_ctx_t rx = make_context();
    std::vector<uint8_t> tx_wire;
    std::vector<uint8_t> rx_wire;
    std::vector<uint8_t> received;
    tx.state = PPP_STATE_NETWORK;
    tx.modem = &tx_wire;
    tx.serial_push = capture_serial;
    tx.ccp_open = true;
    tx.ccp_tx_method = PPP_CCP_METHOD_MPPC;
    rx.state = PPP_STATE_NETWORK;
    rx.modem = &rx_wire;
    rx.serial_push = capture_serial;
    rx.network_send_ip = capture_network;
    rx.ccp_open = true;
    rx.ccp_rx_method = PPP_CCP_METHOD_MPPC;
    ASSERT_TRUE(ppp_ccp_codec_set(&tx, true, PPP_CCP_METHOD_MPPC));
    ASSERT_TRUE(ppp_ccp_codec_set(&rx, false, PPP_CCP_METHOD_MPPC));

    send_frame(&rx, { 0xA0, 0x00, 0x00, 0x80 }, false, PPP_PROTO_MPPE);
    ASSERT_TRUE(rx.ccp_reset_pending);
    uint16_t protocol = 0;
    std::vector<uint8_t> reset_request = decode_captured_frame(rx_wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_CCP);
    ASSERT_EQ(reset_request[0], PPP_CODE_RESET_REQUEST);

    tx_wire.clear();
    send_frame(&tx, reset_request, false, PPP_PROTO_CCP);
    std::vector<uint8_t> reset_ack = decode_captured_frame(tx_wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_CCP);
    ASSERT_EQ(reset_ack[0], PPP_CODE_RESET_ACK);

    rx_wire.clear();
    std::vector<uint8_t> stale_reset_ack = reset_ack;
    stale_reset_ack[1]++;
    send_frame(&rx, stale_reset_ack, false, PPP_PROTO_CCP);
    EXPECT_TRUE(rx.ccp_reset_pending);

    send_frame(&rx, reset_ack, false, PPP_PROTO_CCP);
    EXPECT_FALSE(rx.ccp_reset_pending);
    rx.modem = &received;
    rx.serial_push = discard_serial;

    const std::vector<uint8_t> ip_packet = { 0x45, 0x00, 0x00, 0x04 };
    tx_wire.clear();
    ppp_send_frame(&tx, PPP_PROTO_IP, ip_packet.data(), static_cast<int>(ip_packet.size()));
    std::vector<uint8_t> compressed = decode_captured_frame(tx_wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_MPPE);
    EXPECT_NE(compressed[0] & 0x80, 0);
    send_frame(&rx, compressed, false, PPP_PROTO_MPPE);
    EXPECT_EQ(received, ip_packet);

    ppp_ccp_codec_close(&tx);
    ppp_ccp_codec_close(&rx);
}

TEST(ModemPpp, DoesNotSendPlaintextBeforeCcpOpens)
{
    ppp_ctx_t ctx = make_context();
    std::vector<uint8_t> wire;
    ctx.state = PPP_STATE_NETWORK;
    ctx.modem = &wire;
    ctx.serial_push = capture_serial;
    ctx.mppe_keys_ready = true;
    const std::vector<uint8_t> ip_packet = { 0x45, 0x00, 0x00, 0x04 };

    ppp_send_frame(&ctx, PPP_PROTO_IP, ip_packet.data(), static_cast<int>(ip_packet.size()));

    EXPECT_TRUE(wire.empty());
}

TEST(ModemPpp, RequiredMppeNeedsMschapV2)
{
    ppp_ctx_t ctx = make_context();
    ctx.mppe_min_bits = 40;
    ctx.lcp_req_sent = false;

    ppp_start(&ctx);

    EXPECT_EQ(ctx.state, PPP_STATE_DEAD);
    EXPECT_FALSE(ctx.lcp_req_sent);
}

TEST(ModemPpp, FallsBackToPlaintextWhenPeerRejectsCcp)
{
    ppp_ctx_t ctx = make_context();
    std::vector<uint8_t> wire;
    ctx.state = PPP_STATE_IPCP_NEGOTIATE;
    ctx.ipcp_ack_sent = true;
    ctx.ipcp_ack_received = true;
    ctx.mppe_keys_ready = true;
    ctx.modem = &wire;
    ctx.serial_push = capture_serial;
    const std::vector<uint8_t> reject = {
        PPP_CODE_PROTOCOL_REJECT, 2, 0, 6,
        static_cast<uint8_t>(PPP_PROTO_CCP >> 8),
        static_cast<uint8_t>(PPP_PROTO_CCP)
    };

    send_frame(&ctx, reject);

    EXPECT_TRUE(ctx.ccp_plaintext_fallback);
    EXPECT_EQ(ctx.state, PPP_STATE_NETWORK);
    wire.clear();
    const std::vector<uint8_t> ip_packet = { 0x45, 0x00, 0x00, 0x04 };
    ppp_send_frame(&ctx, PPP_PROTO_IP, ip_packet.data(), static_cast<int>(ip_packet.size()));
    uint16_t protocol = 0;
    EXPECT_EQ(decode_captured_frame(wire, protocol), ip_packet);
    EXPECT_EQ(protocol, PPP_PROTO_IP);
}

static ppp_ctx_t
make_context()
{
    ppp_ctx_t ctx{};
    ctx.state = PPP_STATE_LCP_NEGOTIATE;
    ctx.our_mru = PPP_DEFAULT_MRU;
    ctx.peer_mru = PPP_DEFAULT_MRU;
    ctx.auth_type = PPP_AUTH_CHAP_MD5;
    ctx.serial_push = discard_serial;
    ctx.lcp_req_sent = true;
    ctx.lcp_request_len = 9;
    const uint8_t request[] = { PPP_CODE_CONFIGURE_REQUEST, 7, 0, 9,
                                LCP_OPT_AUTH_PROTO, 5, 0xC2, 0x23, CHAP_ALG_MD5 };
    std::memcpy(ctx.lcp_request, request, sizeof(request));
    return ctx;
}

TEST(ModemPpp, LcpIdentificationIsLoggedWithoutReply)
{
    ppp_ctx_t ctx = make_context();
    std::vector<uint8_t> wire;
    ctx.state = PPP_STATE_NETWORK;
    ctx.modem = &wire;
    ctx.serial_push = capture_serial;
    const std::vector<uint8_t> identification = {
        PPP_CODE_IDENTIFICATION, 1, 0, 13, 0x12, 0x34, 0x56, 0x78, 't', 'e', 's', 't', '\n'
    };

    send_frame(&ctx, identification);

    EXPECT_TRUE(wire.empty());
    EXPECT_EQ(ctx.state, PPP_STATE_NETWORK);
}

TEST(ModemPpp, LcpRetriesConfigureRequestAndStopsAfterLimit)
{
    ppp_ctx_t ctx = make_context();
    std::vector<uint8_t> wire;
    ctx.serial_push = capture_serial;
    ctx.modem = &wire;

    for (int retry = 0; retry < 11; retry++) {
        for (int millisecond = 0; millisecond < 3000; millisecond++)
            ppp_timer_tick(&ctx);
        if (retry < 10) {
            EXPECT_EQ(ctx.state, PPP_STATE_LCP_NEGOTIATE);
        }
    }

    EXPECT_EQ(ctx.state, PPP_STATE_DEAD);
    EXPECT_FALSE(ctx.lcp_req_sent);
    EXPECT_EQ(ctx.lcp_retries, 11);
    EXPECT_FALSE(wire.empty());
}

TEST(ModemPpp, AuthenticationTimesOutWhenPeerDoesNotRespond)
{
    ppp_ctx_t ctx = make_context();
    ctx.state = PPP_STATE_AUTH;

    for (int millisecond = 0; millisecond < 29999; millisecond++)
        ppp_timer_tick(&ctx);
    EXPECT_EQ(ctx.state, PPP_STATE_AUTH);

    ppp_timer_tick(&ctx);
    EXPECT_EQ(ctx.state, PPP_STATE_DEAD);
}

TEST(ModemPpp, ConfigureNakFallsBackInConfiguredOrder)
{
    ppp_ctx_t ctx = make_context();
    ctx.auth_type = PPP_AUTH_MSCHAPV2;
    ctx.lcp_request[8] = CHAP_ALG_MSCHAPV2;
    auto send_nak = [&ctx](const std::vector<uint8_t> &option) {
        std::vector<uint8_t> nak = { PPP_CODE_CONFIGURE_NAK, ctx.lcp_request[1], 0, 0 };
        nak.insert(nak.end(), option.begin(), option.end());
        nak[2] = static_cast<uint8_t>(nak.size() >> 8);
        nak[3] = static_cast<uint8_t>(nak.size());
        send_frame(&ctx, nak);
    };

    send_nak({ LCP_OPT_AUTH_PROTO, 5, 0xC2, 0x23, CHAP_ALG_MSCHAP });
    EXPECT_EQ(ctx.state, PPP_STATE_LCP_NEGOTIATE);
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_MSCHAP);
    EXPECT_EQ(ctx.lcp_request[18], CHAP_ALG_MSCHAP);

    send_nak({ LCP_OPT_AUTH_PROTO, 5, 0xC2, 0x23, CHAP_ALG_MD5 });
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_CHAP_MD5);
    EXPECT_EQ(ctx.lcp_request[18], CHAP_ALG_MD5);

    send_nak({ LCP_OPT_AUTH_PROTO, 4, 0xC0, 0x23 });
    EXPECT_EQ(ctx.state, PPP_STATE_LCP_NEGOTIATE);
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_PAP);
    EXPECT_EQ(ctx.lcp_request[15], 4);
    EXPECT_EQ(ctx.lcp_request[16], 0xC0);
    EXPECT_EQ(ctx.lcp_request[17], 0x23);
}

TEST(ModemPpp, ConfigureNakFallsBackThroughChapHashOrder)
{
    ppp_ctx_t ctx = make_context();
    ctx.auth_type = PPP_AUTH_CHAP_SHA512;
    ctx.lcp_request[8] = CHAP_ALG_SHA512;
    auto send_nak = [&ctx](const std::vector<uint8_t> &option) {
        std::vector<uint8_t> nak = { PPP_CODE_CONFIGURE_NAK, ctx.lcp_request[1], 0, 0 };
        nak.insert(nak.end(), option.begin(), option.end());
        nak[2] = static_cast<uint8_t>(nak.size() >> 8);
        nak[3] = static_cast<uint8_t>(nak.size());
        send_frame(&ctx, nak);
    };

    send_nak({ LCP_OPT_AUTH_PROTO, 5, 0xC2, 0x23, CHAP_ALG_SHA384 });
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_CHAP_SHA384);
    EXPECT_EQ(ctx.lcp_request[18], CHAP_ALG_SHA384);

    send_nak({ LCP_OPT_AUTH_PROTO, 5, 0xC2, 0x23, CHAP_ALG_SHA256 });
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_CHAP_SHA256);
    EXPECT_EQ(ctx.lcp_request[18], CHAP_ALG_SHA256);

    send_nak({ LCP_OPT_AUTH_PROTO, 5, 0xC2, 0x23, CHAP_ALG_SHA1 });
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_CHAP_SHA1);
    EXPECT_EQ(ctx.lcp_request[18], CHAP_ALG_SHA1);

    send_nak({ LCP_OPT_AUTH_PROTO, 5, 0xC2, 0x23, CHAP_ALG_MD5 });
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_CHAP_MD5);
    EXPECT_EQ(ctx.lcp_request[18], CHAP_ALG_MD5);

    send_nak({ LCP_OPT_AUTH_PROTO, 4, 0xC0, 0x23 });
    EXPECT_EQ(ctx.state, PPP_STATE_LCP_NEGOTIATE);
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_PAP);
    EXPECT_EQ(ctx.lcp_request[15], 4);
}

TEST(ModemPpp, ConfigureNakCannotDowngradeWhenMppeIsRequired)
{
    ppp_ctx_t ctx = make_context();
    ctx.auth_type = PPP_AUTH_MSCHAPV2;
    ctx.mppe_min_bits = 40;
    ctx.lcp_request[8] = CHAP_ALG_MSCHAPV2;
    const std::vector<uint8_t> nak = { PPP_CODE_CONFIGURE_NAK, 7, 0, 9,
                                       LCP_OPT_AUTH_PROTO, 5, 0xC2, 0x23,
                                       CHAP_ALG_MSCHAP };

    send_frame(&ctx, nak);

    EXPECT_EQ(ctx.state, PPP_STATE_DEAD);
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_MSCHAPV2);
}

TEST(ModemPpp, ConfigureRejectFallsBackFromChapMd5ToPap)
{
    ppp_ctx_t ctx = make_context();
    const std::vector<uint8_t> reject = { PPP_CODE_CONFIGURE_REJECT, 7, 0, 9,
                                          LCP_OPT_AUTH_PROTO, 5, 0xC2, 0x23, CHAP_ALG_MD5 };

    send_frame(&ctx, reject);

    EXPECT_EQ(ctx.state, PPP_STATE_LCP_NEGOTIATE);
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_PAP);
    EXPECT_TRUE(ctx.lcp_req_sent);
    EXPECT_FALSE(ctx.auth_complete);
}

TEST(ModemPpp, ConfigureRejectCannotDowngradeWhenMppeIsRequired)
{
    ppp_ctx_t ctx = make_context();
    ctx.auth_type = PPP_AUTH_MSCHAPV2;
    ctx.mppe_min_bits = 40;
    ctx.lcp_request[8] = CHAP_ALG_MSCHAPV2;
    const std::vector<uint8_t> reject = { PPP_CODE_CONFIGURE_REJECT, 7, 0, 9,
                                          LCP_OPT_AUTH_PROTO, 5, 0xC2, 0x23,
                                          CHAP_ALG_MSCHAPV2 };

    send_frame(&ctx, reject);

    EXPECT_EQ(ctx.state, PPP_STATE_DEAD);
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_MSCHAPV2);
}

TEST(ModemPpp, ConfigureRejectCannotDisablePapAuthentication)
{
    ppp_ctx_t ctx = make_context();
    ctx.auth_type = PPP_AUTH_PAP;
    ctx.lcp_request_len = 8;
    const uint8_t request[] = { PPP_CODE_CONFIGURE_REQUEST, 7, 0, 8,
                                LCP_OPT_AUTH_PROTO, 4, 0xC0, 0x23 };
    std::memcpy(ctx.lcp_request, request, sizeof(request));
    const std::vector<uint8_t> reject = { PPP_CODE_CONFIGURE_REJECT, 7, 0, 8,
                                          LCP_OPT_AUTH_PROTO, 4, 0xC0, 0x23 };

    send_frame(&ctx, reject);

    EXPECT_EQ(ctx.state, PPP_STATE_DEAD);
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_PAP);
}

TEST(ModemPpp, StartsOptionalCcpAfterPapWithoutMppeKeys)
{
    ppp_ctx_t ctx = make_context();
    std::vector<uint8_t> wire;
    ctx.state = PPP_STATE_AUTH;
    ctx.auth_type = PPP_AUTH_PAP;
    ctx.auth_complete = true;
    ctx.modem = &wire;
    ctx.serial_push = capture_serial;

    ppp_advance_state(&ctx);

    EXPECT_EQ(ctx.state, PPP_STATE_IPCP_NEGOTIATE);
    EXPECT_FALSE(ctx.mppe_keys_ready);
    EXPECT_TRUE(ctx.ccp_req_sent);
    EXPECT_EQ(ctx.ccp_request_method, PPP_CCP_METHOD_DEFLATE);
}

TEST(ModemPpp, IgnoresStaleConfigureNak)
{
    ppp_ctx_t ctx = make_context();
    const std::vector<uint8_t> nak = { PPP_CODE_CONFIGURE_NAK, 8, 0, 8,
                                       LCP_OPT_AUTH_PROTO, 4, 0xC0, 0x23 };

    send_frame(&ctx, nak);

    EXPECT_EQ(ctx.state, PPP_STATE_LCP_NEGOTIATE);
    EXPECT_EQ(ctx.auth_type, PPP_AUTH_CHAP_MD5);
}

TEST(ModemPpp, IgnoresFrameWithInvalidFcs)
{
    ppp_ctx_t ctx = make_context();
    const std::vector<uint8_t> nak = { PPP_CODE_CONFIGURE_NAK, 7, 0, 8,
                                       LCP_OPT_AUTH_PROTO, 4, 0xC0, 0x23 };

    send_frame(&ctx, nak, true);

    EXPECT_EQ(ctx.state, PPP_STATE_LCP_NEGOTIATE);
    EXPECT_TRUE(ctx.lcp_req_sent);
    EXPECT_FALSE(ctx.lcp_ack_received);
}

TEST(ModemPpp, RejectedPeerRequestClearsPreviousAck)
{
    ppp_ctx_t ctx = make_context();
    ctx.lcp_ack_sent = true;
    ctx.lcp_ack_received = true;
    const std::vector<uint8_t> request = { PPP_CODE_CONFIGURE_REQUEST, 3, 0, 8,
                                           LCP_OPT_AUTH_PROTO, 4, 0xC0, 0x23 };

    send_frame(&ctx, request);

    EXPECT_EQ(ctx.state, PPP_STATE_LCP_NEGOTIATE);
    EXPECT_FALSE(ctx.lcp_ack_sent);
}

TEST(ModemPpp, RejectsEndpointDiscriminatorWithoutMultilink)
{
    ppp_ctx_t ctx = make_context();
    std::vector<uint8_t> wire;
    ctx.modem = &wire;
    ctx.serial_push = capture_serial;
    const std::vector<uint8_t> request = {
        PPP_CODE_CONFIGURE_REQUEST, 3, 0, 8,
        LCP_OPT_ENDPOINT_DISC, 4, 1, 0x42
    };

    send_frame(&ctx, request);

    uint16_t protocol = 0;
    const std::vector<uint8_t> response = decode_captured_frame(wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_LCP);
    EXPECT_EQ(response, (std::vector<uint8_t> {
        PPP_CODE_CONFIGURE_REJECT, 3, 0, 8,
        LCP_OPT_ENDPOINT_DISC, 4, 1, 0x42
    }));
}

TEST(ModemPpp, NaksZeroOrLoopedBackMagicNumber)
{
    constexpr uint32_t local_magic = 0x12345678;
    const uint32_t invalid_magic[] = { 0, local_magic };

    for (size_t index = 0; index < std::size(invalid_magic); index++) {
        ppp_ctx_t ctx = make_context();
        std::vector<uint8_t> wire;
        ctx.modem = &wire;
        ctx.serial_push = capture_serial;
        ctx.our_magic = local_magic;
        const uint32_t magic = invalid_magic[index];
        const std::vector<uint8_t> request = {
            PPP_CODE_CONFIGURE_REQUEST, static_cast<uint8_t>(index + 1), 0, 10,
            LCP_OPT_MAGIC_NUMBER, 6,
            static_cast<uint8_t>(magic >> 24), static_cast<uint8_t>(magic >> 16),
            static_cast<uint8_t>(magic >> 8), static_cast<uint8_t>(magic)
        };

        send_frame(&ctx, request);

        uint16_t protocol = 0;
        const std::vector<uint8_t> response = decode_captured_frame(wire, protocol);
        ASSERT_EQ(protocol, PPP_PROTO_LCP);
        ASSERT_EQ(response.size(), 10u);
        EXPECT_EQ(response[0], PPP_CODE_CONFIGURE_NAK);
        EXPECT_EQ(response[4], LCP_OPT_MAGIC_NUMBER);
        const uint32_t suggested_magic = (static_cast<uint32_t>(response[6]) << 24)
                                       | (static_cast<uint32_t>(response[7]) << 16)
                                       | (static_cast<uint32_t>(response[8]) << 8)
                                       | response[9];
        EXPECT_NE(suggested_magic, 0u);
        EXPECT_NE(suggested_magic, local_magic);
    }

    ppp_ctx_t ctx = make_context();
    std::vector<uint8_t> wire;
    ctx.modem = &wire;
    ctx.serial_push = capture_serial;
    ctx.our_magic = local_magic;
    const uint32_t valid_magic = 0x87654321;
    const std::vector<uint8_t> request = {
        PPP_CODE_CONFIGURE_REQUEST, 3, 0, 10,
        LCP_OPT_MAGIC_NUMBER, 6,
        static_cast<uint8_t>(valid_magic >> 24), static_cast<uint8_t>(valid_magic >> 16),
        static_cast<uint8_t>(valid_magic >> 8), static_cast<uint8_t>(valid_magic)
    };

    send_frame(&ctx, request);

    uint16_t protocol = 0;
    const std::vector<uint8_t> response = decode_captured_frame(wire, protocol);
    ASSERT_EQ(protocol, PPP_PROTO_LCP);
    ASSERT_EQ(response.size(), 10u);
    EXPECT_EQ(response[0], PPP_CODE_CONFIGURE_ACK);
    EXPECT_EQ(ctx.peer_magic, valid_magic);
}

TEST(ModemPpp, IgnoresConfigureNakAfterAck)
{
    ppp_ctx_t ctx = make_context();
    std::vector<uint8_t> ack(ctx.lcp_request, ctx.lcp_request + ctx.lcp_request_len);
    ack[0] = PPP_CODE_CONFIGURE_ACK;
    send_frame(&ctx, ack);
    ASSERT_TRUE(ctx.lcp_ack_received);
    ASSERT_FALSE(ctx.lcp_req_sent);

    const std::vector<uint8_t> nak = { PPP_CODE_CONFIGURE_NAK, 7, 0, 8,
                                       LCP_OPT_AUTH_PROTO, 4, 0xC0, 0x23 };
    send_frame(&ctx, nak);

    EXPECT_EQ(ctx.auth_type, PPP_AUTH_CHAP_MD5);
    EXPECT_TRUE(ctx.lcp_ack_received);
    EXPECT_FALSE(ctx.lcp_req_sent);
}

TEST(ModemPpp, AppliesConfigureNakSuggestions)
{
    ppp_ctx_t ctx = make_context();
    ctx.auth_type = PPP_AUTH_NONE;
    ctx.our_mru = 1500;
    ctx.our_accm = 0xFFFF0000;
    ctx.request_pfc = true;
    ctx.request_acfc = true;
    const uint8_t request[] = { PPP_CODE_CONFIGURE_REQUEST, 7, 0, 18,
                                LCP_OPT_MRU, 4, 0x05, 0xDC,
                                LCP_OPT_ACCM, 6, 0xFF, 0xFF, 0, 0,
                                LCP_OPT_PFC, 2, LCP_OPT_ACFC, 2 };
    std::memcpy(ctx.lcp_request, request, sizeof(request));
    ctx.lcp_request_len = sizeof(request);
    const std::vector<uint8_t> nak = { PPP_CODE_CONFIGURE_NAK, 7, 0, 18,
                                       LCP_OPT_MRU, 4, 0x05, 0x78,
                                       LCP_OPT_ACCM, 6, 0, 0, 0, 0x0F,
                                       LCP_OPT_PFC, 2, LCP_OPT_ACFC, 2 };

    send_frame(&ctx, nak);

    EXPECT_EQ(ctx.our_mru, 1400);
    EXPECT_EQ(ctx.our_accm, 0xFFFF000Fu);
    EXPECT_FALSE(ctx.request_pfc);
    EXPECT_FALSE(ctx.request_acfc);
    ASSERT_EQ(ctx.lcp_request_len, 20);
    EXPECT_EQ(ctx.lcp_request[4], LCP_OPT_MRU);
    EXPECT_EQ(ctx.lcp_request[5], 4);
    EXPECT_EQ(ctx.lcp_request[6], 0x05);
    EXPECT_EQ(ctx.lcp_request[7], 0x78);
    EXPECT_EQ(ctx.lcp_request[8], LCP_OPT_ACCM);
    EXPECT_EQ(ctx.lcp_request[14], LCP_OPT_MAGIC_NUMBER);
}

TEST(ModemPpp, NegotiatedNt31RasUsesRasEnvelopeForIpv4)
{
    ppp_ctx_t tx = make_context();
    ppp_ctx_t rx = make_context();
    std::vector<uint8_t> wire;
    std::vector<uint8_t> received;
    std::vector<uint8_t> packet(128);
    tx.state = PPP_STATE_NETWORK;
    tx.modem = &wire;
    tx.serial_push = capture_serial;
    tx.ccp_open = true;
    tx.ccp_tx_method = PPP_CCP_METHOD_NT31RAS;
    rx.state = PPP_STATE_NETWORK;
    rx.modem = &received;
    rx.network_send_ip = capture_network;
    rx.ccp_open = true;
    rx.ccp_rx_method = PPP_CCP_METHOD_NT31RAS;
    for (size_t index = 0; index < packet.size(); index++)
        packet[index] = static_cast<uint8_t>((index / 8) % 4);
    ASSERT_TRUE(ppp_ccp_codec_set(&tx, true, PPP_CCP_METHOD_NT31RAS));
    ASSERT_TRUE(ppp_ccp_codec_set(&rx, false, PPP_CCP_METHOD_NT31RAS));

    for (int frame_index = 0; frame_index < 2; frame_index++) {
        wire.clear();
        ppp_send_frame(&tx, PPP_PROTO_IP, packet.data(), static_cast<int>(packet.size()));

        ASSERT_GT(wire.size(), 10u);
        EXPECT_EQ(wire[0], PPP_RAS_SYN);
        EXPECT_EQ(wire[1], PPP_RAS_SOH_DEST | PPP_RAS_SOH_TYPE | PPP_RAS_SOH_COMPRESS);
        EXPECT_EQ(wire[4], 0x08);
        EXPECT_EQ(wire[5], 0x00);
        for (uint8_t byte : wire)
            ppp_rx_byte(&rx, byte);
        EXPECT_EQ(received, packet);
    }
}

TEST(ModemPpp, Nt31RasFlushesAndRecoversAfterTicketGap)
{
    ppp_ctx_t tx = make_context();
    ppp_ctx_t rx = make_context();
    std::vector<uint8_t> tx_wire;
    ras_capture_t rx_capture;
    std::vector<uint8_t> packet(128);
    tx.state = PPP_STATE_NETWORK;
    tx.modem = &tx_wire;
    tx.serial_push = capture_serial;
    tx.ccp_open = true;
    tx.ccp_tx_method = PPP_CCP_METHOD_NT31RAS;
    tx.ccp_rx_method = PPP_CCP_METHOD_NT31RAS;
    rx.state = PPP_STATE_NETWORK;
    rx.modem = &rx_capture;
    rx.serial_push = capture_ras_serial;
    rx.network_send_ip = capture_ras_network;
    rx.ccp_open = true;
    rx.ccp_rx_method = PPP_CCP_METHOD_NT31RAS;
    ASSERT_TRUE(ppp_ccp_codec_set(&tx, true, PPP_CCP_METHOD_NT31RAS));
    ASSERT_TRUE(ppp_ccp_codec_set(&tx, false, PPP_CCP_METHOD_NT31RAS));
    ASSERT_TRUE(ppp_ccp_codec_set(&rx, false, PPP_CCP_METHOD_NT31RAS));
    for (size_t index = 0; index < packet.size(); index++)
        packet[index] = static_cast<uint8_t>((index / 8) % 4);

    ppp_send_frame(&tx, PPP_PROTO_IP, packet.data(), static_cast<int>(packet.size()));
    for (uint8_t byte : tx_wire)
        ppp_rx_byte(&rx, byte);
    ASSERT_EQ(rx_capture.packets.size(), 1u);

    tx_wire.clear();
    ppp_send_frame(&tx, PPP_PROTO_IP, packet.data(), static_cast<int>(packet.size()));
    tx_wire.back() ^= 1;
    for (uint8_t byte : tx_wire)
        ppp_rx_byte(&rx, byte);
    EXPECT_EQ(rx_capture.packets.size(), 1u);

    tx_wire.clear();
    ppp_send_frame(&tx, PPP_PROTO_IP, packet.data(), static_cast<int>(packet.size()));
    for (uint8_t byte : tx_wire)
        ppp_rx_byte(&rx, byte);
    ASSERT_EQ(rx_capture.serial.size(), 10u);
    EXPECT_EQ(rx_capture.serial[6], 0xFF);
    for (uint8_t byte : rx_capture.serial)
        ppp_rx_byte(&tx, byte);

    tx_wire.clear();
    ppp_send_frame(&tx, PPP_PROTO_IP, packet.data(), static_cast<int>(packet.size()));
    ASSERT_EQ(tx_wire[6], 32);
    for (uint8_t byte : tx_wire)
        ppp_rx_byte(&rx, byte);
    ASSERT_EQ(rx_capture.packets.size(), 2u);
    EXPECT_EQ(rx_capture.packets.back(), packet);
}