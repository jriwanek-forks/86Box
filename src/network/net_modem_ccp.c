/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          CCP negotiation and data compression for NT31 RAS, MPPC (RFC 2118),
 *          Deflate (RFC 1979), BSD-Compress (RFC 1977), Predictor (RFC 1978),
 *          and stateful/stateless 40-, 56-, and 128-bit MPPE (RFC 3078).
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2026 Jasmine Iwanek.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <86box/net_modem_ppp.h>
#include <86box/net_modem_mppe.h>

#define CCP_OPT_MPPE 18
#define CCP_OPT_PREDICTOR1 1
#define CCP_OPT_PREDICTOR2 2
#define CCP_OPT_BSD 21
#define CCP_OPT_DEFLATE 26
#define CCP_OPT_NT31RAS PPP_CCP_METHOD_NT31RAS
#define CCP_MPPE_STATELESS 0x01000000u
#define CCP_MPPE_128 0x00000040u
#define CCP_MPPE_56 0x00000080u
#define CCP_MPPE_40 0x00000020u
#define CCP_MPPE_KEY_BITS (CCP_MPPE_128 | CCP_MPPE_56 | CCP_MPPE_40)
#define CCP_MPPE_OFFER (CCP_MPPE_STATELESS | CCP_MPPE_KEY_BITS)
#define CCP_NT31RAS_FEATURES 0x0000000Fu

static uint32_t
ccp_nt31ras_window_size(uint32_t features)
{
    if (features & 0x00000008u)
        return 65536;
    if (features & 0x00000004u)
        return 32768;
    if (features & 0x00000002u)
        return 16384;
    if (features & 0x00000001u)
        return 8192;
    return 0;
}

static uint32_t
ccp_minimum_key_bit(uint8_t minimum_bits)
{
    if (minimum_bits >= 128)
        return CCP_MPPE_128;
    if (minimum_bits >= 56)
        return CCP_MPPE_56;
    if (minimum_bits >= 40)
        return CCP_MPPE_40;
    return 0;
}

static uint32_t
ccp_get_u32(const uint8_t *p)
{
    return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16)
         | ((uint32_t) p[2] << 8) | p[3];
}

static void
ccp_put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t) (value >> 24);
    p[1] = (uint8_t) (value >> 16);
    p[2] = (uint8_t) (value >> 8);
    p[3] = (uint8_t) value;
}

static uint32_t
ccp_get_le_u32(const uint8_t *p)
{
    return (uint32_t) p[0] | ((uint32_t) p[1] << 8)
         | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}

static void
ccp_put_le_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t) value;
    p[1] = (uint8_t) (value >> 8);
    p[2] = (uint8_t) (value >> 16);
    p[3] = (uint8_t) (value >> 24);
}

static void
ccp_update_state(ppp_ctx_t *ctx)
{
    ctx->ccp_open = !ctx->ccp_plaintext_fallback
                 && ctx->ccp_ack_sent && ctx->ccp_ack_received;
    ctx->mppe_tx_enabled = ctx->ccp_open && ctx->ccp_peer_mppe
                        && ctx->mppe_keys_ready;
    ctx->mppe_rx_enabled = ctx->ccp_open && ctx->ccp_ack_received
                        && ctx->mppe_keys_ready;
}

static void
ccp_send_request(ppp_ctx_t *ctx, uint32_t bits)
{
    uint8_t request[10] = {
        PPP_CODE_CONFIGURE_REQUEST, 0, 0, 10,
        CCP_OPT_MPPE, 6, 0, 0, 0, 0
    };

    request[1] = ctx->ccp_id++;
    ccp_put_u32(request + 6, bits);
    memcpy(ctx->ccp_request, request, sizeof(request));
    ctx->ccp_request_len = sizeof(request);
    ctx->ccp_request_id = request[1];
    ctx->ccp_request_bits = bits;
    ctx->ccp_request_method = PPP_CCP_METHOD_MPPE;
    ctx->ccp_req_sent = true;
    ppp_send_frame(ctx, PPP_PROTO_CCP, request, sizeof(request));
}

static bool
ccp_send_codec_request(ppp_ctx_t *ctx, uint8_t method)
{
    uint8_t request[28] = { PPP_CODE_CONFIGURE_REQUEST, 0, 0, 0 };
    int request_len = 4;

    if (!ctx || ctx->mppe_min_bits > 0)
        return false;
    switch (method) {
        case PPP_CCP_METHOD_PREDICTOR1:
            request[request_len++] = CCP_OPT_PREDICTOR1;
            request[request_len++] = 2;
            break;
        case PPP_CCP_METHOD_PREDICTOR2:
            request[request_len++] = CCP_OPT_PREDICTOR2;
            request[request_len++] = 2;
            break;
        case PPP_CCP_METHOD_BSD:
            request[request_len++] = CCP_OPT_BSD;
            request[request_len++] = 3;
            request[request_len++] = 0x29;
            break;
        case PPP_CCP_METHOD_MPPC:
            request[request_len++] = CCP_OPT_MPPE;
            request[request_len++] = 6;
            request[request_len++] = 0;
            request[request_len++] = 0;
            request[request_len++] = 0;
            request[request_len++] = 1;
            break;
        case PPP_CCP_METHOD_DEFLATE:
            request[request_len++] = CCP_OPT_DEFLATE;
            request[request_len++] = 4;
            request[request_len++] = 0x78;
            request[request_len++] = 0;
            break;
        case PPP_CCP_METHOD_NT31RAS:
            request[request_len++] = CCP_OPT_NT31RAS;
            request[request_len++] = 22;
            ccp_put_le_u32(request + request_len, CCP_NT31RAS_FEATURES);
            ccp_put_le_u32(request + request_len + 4, CCP_NT31RAS_FEATURES);
            ccp_put_le_u32(request + request_len + 8, PPP_DEFAULT_MRU);
            ccp_put_le_u32(request + request_len + 12, PPP_DEFAULT_MRU);
            ccp_put_le_u32(request + request_len + 16, 0);
            request_len += 20;
            break;
        default:
            return false;
    }

    request[2] = (uint8_t) (request_len >> 8);
    request[3] = (uint8_t) request_len;
    request[1] = ctx->ccp_id++;
    memcpy(ctx->ccp_request, request, (size_t) request_len);
    ctx->ccp_request_len = request_len;
    ctx->ccp_request_id = request[1];
    ctx->ccp_request_bits = 0;
    ctx->ccp_request_method = method;
    ctx->ccp_req_sent = true;
    ctx->ccp_rejected = false;
    ppp_send_frame(ctx, PPP_PROTO_CCP, request, request_len);
    return true;
}

static bool
ccp_try_next_codec(ppp_ctx_t *ctx)
{
    switch (ctx->ccp_request_method) {
        case PPP_CCP_METHOD_MPPE:
            return ccp_send_codec_request(ctx, PPP_CCP_METHOD_DEFLATE);
        case PPP_CCP_METHOD_DEFLATE:
            return ccp_send_codec_request(ctx, PPP_CCP_METHOD_BSD);
        case PPP_CCP_METHOD_BSD:
            return ccp_send_codec_request(ctx, PPP_CCP_METHOD_PREDICTOR2);
        case PPP_CCP_METHOD_PREDICTOR2:
            return ccp_send_codec_request(ctx, PPP_CCP_METHOD_PREDICTOR1);
        case PPP_CCP_METHOD_PREDICTOR1:
            return ccp_send_codec_request(ctx, PPP_CCP_METHOD_MPPC);
        case PPP_CCP_METHOD_MPPC:
            return ccp_send_codec_request(ctx, PPP_CCP_METHOD_NT31RAS);
        default:
            return false;
    }
}

static bool
ccp_response_matches_request(const ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    int total;

    if (!ctx->ccp_req_sent || pkt_len < 4 || pkt[1] != ctx->ccp_request_id)
        return false;
    total = (pkt[2] << 8) | pkt[3];
    if (total != pkt_len || total < 4)
        return false;
    return total == ctx->ccp_request_len
        && memcmp(pkt + 4, ctx->ccp_request + 4, (size_t) total - 4) == 0;
}

static bool
ccp_parse_mppe_nak(const uint8_t *pkt, int total, uint32_t *bits)
{
    uint32_t proposed;
    if (total != 10 || pkt[4] != CCP_OPT_MPPE || pkt[5] != 6)
        return false;
    proposed = ccp_get_u32(pkt + 6);
    if ((proposed & ~CCP_MPPE_OFFER) != 0
        || (proposed & CCP_MPPE_KEY_BITS) == 0
        || ((proposed & CCP_MPPE_KEY_BITS)
            & ((proposed & CCP_MPPE_KEY_BITS) - 1)) != 0)
        return false;
    *bits = proposed;
    return true;
}

static bool
ccp_retry_nt31ras_nak(ppp_ctx_t *ctx, const uint8_t *pkt, int total)
{
    uint32_t send_features;
    uint32_t recv_features;
    uint32_t old_send_features;
    uint32_t old_recv_features;
    uint32_t max_send;
    uint32_t max_recv;

    if (ctx->ccp_request_method != PPP_CCP_METHOD_NT31RAS || total != 26
        || pkt[4] != CCP_OPT_NT31RAS || pkt[5] != 22
        || ctx->ccp_request_len != 26 || ctx->ccp_retries >= 10)
        return false;

    send_features = ccp_get_le_u32(pkt + 6);
    recv_features = ccp_get_le_u32(pkt + 10);
    max_send = ccp_get_le_u32(pkt + 14);
    max_recv = ccp_get_le_u32(pkt + 18);
    old_send_features = ccp_get_le_u32(ctx->ccp_request + 6);
    old_recv_features = ccp_get_le_u32(ctx->ccp_request + 10);
    if (send_features == 0 || recv_features == 0
        || (send_features & ~old_send_features) != 0
        || (recv_features & ~old_recv_features) != 0
        || (send_features & ~CCP_NT31RAS_FEATURES) != 0
        || (recv_features & ~CCP_NT31RAS_FEATURES) != 0
        || (send_features == old_send_features && recv_features == old_recv_features)
        || max_send < 128 || max_send > PPP_MAX_FRAME
        || max_recv < 128 || max_recv > PPP_MAX_FRAME)
        return false;

    memcpy(ctx->ccp_request, pkt, (size_t) total);
    ctx->ccp_request[0] = PPP_CODE_CONFIGURE_REQUEST;
    ctx->ccp_request[1] = ctx->ccp_id++;
    ctx->ccp_request_id = ctx->ccp_request[1];
    ctx->ccp_request_len = total;
    ctx->ccp_request_bits = 0;
    ctx->ccp_req_sent = true;
    ctx->ccp_ack_received = false;
    ctx->ccp_retries++;
    ppp_send_frame(ctx, PPP_PROTO_CCP, ctx->ccp_request, total);
    return true;
}

static uint32_t
ccp_preferred_bits(uint32_t requested)
{
    uint32_t mode = requested & CCP_MPPE_STATELESS;
    uint32_t keys = requested & CCP_MPPE_KEY_BITS;
    uint32_t key = keys & CCP_MPPE_128 ? CCP_MPPE_128
                 : keys & CCP_MPPE_56 ? CCP_MPPE_56
                 : keys & CCP_MPPE_40 ? CCP_MPPE_40 : CCP_MPPE_128;
    return mode | key;
}

static uint8_t
ccp_key_bits_to_length(uint32_t bits)
{
    if (bits & CCP_MPPE_128)
        return 128;
    if (bits & CCP_MPPE_56)
        return 56;
    return 40;
}

static bool
ccp_request_options_valid(const uint8_t *pkt, int pkt_len)
{
    for (int pos = 4; pos < pkt_len;) {
        if (pos + 2 > pkt_len)
            return false;
        uint8_t option_len = pkt[pos + 1];
        if (option_len < 2 || pos + option_len > pkt_len)
            return false;
        pos += option_len;
    }
    return true;
}

static void
ccp_send_response(ppp_ctx_t *ctx, uint8_t code, uint8_t id,
                  const uint8_t *options, int options_len)
{
    uint8_t response[PPP_MAX_FRAME];
    int total = 4 + options_len;
    response[0] = code;
    response[1] = id;
    response[2] = (uint8_t) (total >> 8);
    response[3] = (uint8_t) total;
    if (options_len > 0)
        memcpy(response + 4, options, options_len);
    ppp_send_frame(ctx, PPP_PROTO_CCP, response, total);
}

static void
ccp_handle_config_request(ppp_ctx_t *ctx, const uint8_t *pkt, int total)
{
    uint8_t nak[PPP_MAX_FRAME];
    uint8_t rej[PPP_MAX_FRAME];
    int nak_len = 0;
    int rej_len = 0;
    int pos = 4;
    bool saw_mppe = false;
    bool saw_predictor1 = false;
    bool saw_predictor2 = false;
    bool saw_bsd = false;
    bool saw_deflate = false;
    bool saw_nt31ras = false;
    uint32_t selected_bits = 0;
    uint32_t selected_tx_window = 0;
    uint8_t selected_method = PPP_CCP_METHOD_NONE;

    ctx->ccp_ack_sent = false;
    ctx->ccp_peer_mppe = false;
    ccp_update_state(ctx);

    while (pos < total) {
        uint8_t option_len = pkt[pos + 1];
        if (pkt[pos] == CCP_OPT_PREDICTOR1 && option_len == 2 && !saw_predictor1
            && selected_method == PPP_CCP_METHOD_NONE) {
            saw_predictor1 = true;
            selected_method = PPP_CCP_METHOD_PREDICTOR1;
        } else if (pkt[pos] == CCP_OPT_PREDICTOR2 && option_len == 2 && !saw_predictor2
                   && selected_method == PPP_CCP_METHOD_NONE) {
            saw_predictor2 = true;
            selected_method = PPP_CCP_METHOD_PREDICTOR2;
        } else if (pkt[pos] == CCP_OPT_DEFLATE && option_len == 4 && !saw_deflate
                   && pkt[pos + 2] == 0x78 && pkt[pos + 3] == 0
                   && selected_method == PPP_CCP_METHOD_NONE) {
            saw_deflate = true;
            selected_method = PPP_CCP_METHOD_DEFLATE;
        } else if (pkt[pos] == CCP_OPT_BSD && option_len == 3 && !saw_bsd
                   && pkt[pos + 2] == 0x29
                   && selected_method == PPP_CCP_METHOD_NONE) {
            saw_bsd = true;
            selected_method = PPP_CCP_METHOD_BSD;
        } else if (pkt[pos] == CCP_OPT_NT31RAS && option_len == 22 && !saw_nt31ras
                   && selected_method == PPP_CCP_METHOD_NONE) {
            uint32_t send_features = ccp_get_le_u32(pkt + pos + 2);
            uint32_t recv_features = ccp_get_le_u32(pkt + pos + 6);
            uint32_t max_send = ccp_get_le_u32(pkt + pos + 10);
            uint32_t max_recv = ccp_get_le_u32(pkt + pos + 14);
            saw_nt31ras = true;
            if ((send_features & CCP_NT31RAS_FEATURES) != 0
                && (send_features & ~CCP_NT31RAS_FEATURES) == 0
                && (recv_features & CCP_NT31RAS_FEATURES) != 0
                && (recv_features & ~CCP_NT31RAS_FEATURES) == 0
                && max_send >= 128 && max_send <= PPP_MAX_FRAME
                && max_recv >= 128 && max_recv <= PPP_MAX_FRAME) {
                selected_method = PPP_CCP_METHOD_NT31RAS;
                selected_tx_window = ccp_nt31ras_window_size(recv_features);
            } else {
                memcpy(rej + rej_len, pkt + pos, option_len);
                rej_len += option_len;
            }
        } else if (pkt[pos] == CCP_OPT_MPPE && option_len == 6 && !saw_mppe) {
            uint32_t bits = ccp_get_u32(pkt + pos + 2);
            saw_mppe = true;
            if (bits == 1) {
                if (selected_method == PPP_CCP_METHOD_NONE)
                    selected_method = PPP_CCP_METHOD_MPPC;
                else {
                    memcpy(rej + rej_len, pkt + pos, option_len);
                    rej_len += option_len;
                }
            } else if (!ctx->mppe_keys_ready) {
                memcpy(rej + rej_len, pkt + pos, option_len);
                rej_len += option_len;
            } else if ((bits & ~CCP_MPPE_OFFER) != 0
                       || (bits & CCP_MPPE_KEY_BITS) == 0
                       || ((bits & CCP_MPPE_KEY_BITS)
                           & ((bits & CCP_MPPE_KEY_BITS) - 1)) != 0
                       || ccp_key_bits_to_length(bits) < ctx->mppe_min_bits) {
                uint32_t suggested = ccp_preferred_bits(bits);
                if (ctx->mppe_min_bits > 0
                    && ccp_key_bits_to_length(suggested) < ctx->mppe_min_bits)
                    suggested = (bits & CCP_MPPE_STATELESS)
                              | ccp_minimum_key_bit(ctx->mppe_min_bits);
                nak[nak_len++] = CCP_OPT_MPPE;
                nak[nak_len++] = 6;
                ccp_put_u32(nak + nak_len, suggested);
                nak_len += 4;
            } else {
                selected_bits = bits;
                if (selected_method == PPP_CCP_METHOD_NONE)
                    selected_method = PPP_CCP_METHOD_MPPE;
                else {
                    memcpy(rej + rej_len, pkt + pos, option_len);
                    rej_len += option_len;
                    selected_bits = 0;
                }
            }
        } else {
            memcpy(rej + rej_len, pkt + pos, option_len);
            rej_len += option_len;
        }
        pos += option_len;
    }

    if (rej_len > 0) {
        ccp_send_response(ctx, PPP_CODE_CONFIGURE_REJECT, pkt[1], rej, rej_len);
    } else if (nak_len > 0) {
        ccp_send_response(ctx, PPP_CODE_CONFIGURE_NAK, pkt[1], nak, nak_len);
    } else {
        ccp_send_response(ctx, PPP_CODE_CONFIGURE_ACK, pkt[1], pkt + 4, total - 4);
        ctx->ccp_ack_sent = true;
        ctx->ccp_peer_mppe = selected_bits != 0;
        ctx->ccp_tx_bits = selected_bits;
        if (selected_method != PPP_CCP_METHOD_MPPE
            && !ppp_ccp_codec_set_window(ctx, true, selected_method,
                                         selected_method == PPP_CCP_METHOD_NT31RAS
                                             ? selected_tx_window : 8192)) {
            ctx->state = PPP_STATE_DEAD;
            return;
        }
        if (selected_method == PPP_CCP_METHOD_NT31RAS)
            ctx->ccp_tx_window_size = selected_tx_window;
        if (selected_method == PPP_CCP_METHOD_MPPE)
            ppp_ccp_codec_set(ctx, true, PPP_CCP_METHOD_NONE);
        if (selected_bits != 0)
            ppp_mppe_configure(&ctx->mppe_tx, ccp_key_bits_to_length(selected_bits),
                               !(selected_bits & CCP_MPPE_STATELESS));
        ccp_update_state(ctx);
        ppp_advance_state(ctx);
    }
}

void
ppp_ccp_start(ppp_ctx_t *ctx)
{
    if (!ctx)
        return;
    ppp_ccp_codec_close(ctx);
    ctx->ccp_ack_sent = false;
    ctx->ccp_ack_received = false;
    ctx->ccp_peer_mppe = false;
    ctx->ccp_open = false;
    ctx->ccp_rejected = false;
    ctx->ccp_plaintext_fallback = false;
    ctx->ccp_retries = 0;
    ctx->ccp_req_sent = false;
    ctx->ccp_reset_pending = false;
    ctx->mppe_tx_enabled = false;
    ctx->mppe_rx_enabled = false;
    ctx->ras_tx_flush_pending = false;
    ctx->ccp_tx_window_size = 8192;
    ctx->ccp_rx_window_size = 8192;
    ctx->ras_tx_ticket_base = 0;
    ctx->ras_tx_ticket_next = 0;
    ctx->ras_rx_ticket_base = 0;
    ctx->ras_rx_ticket_next = 0;
    ctx->ras_rx_len = 0;
    ctx->ras_rx_expected = 0;
    ctx->ras_rx_in_frame = false;
    if (ctx->mppe_keys_ready) {
        uint32_t offer = CCP_MPPE_STATELESS | CCP_MPPE_KEY_BITS;
        if (ctx->mppe_min_bits >= 128)
            offer = CCP_MPPE_STATELESS | CCP_MPPE_128;
        else if (ctx->mppe_min_bits >= 56)
            offer = CCP_MPPE_STATELESS | CCP_MPPE_128 | CCP_MPPE_56;
        ccp_send_request(ctx, offer);
    } else if (ctx->mppe_min_bits > 0) {
        ctx->state = PPP_STATE_DEAD;
    } else {
        ccp_send_codec_request(ctx, PPP_CCP_METHOD_DEFLATE);
    }
}

void
ppp_ccp_process(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    if (!ctx || !pkt || pkt_len < 4)
        return;
    int total = (pkt[2] << 8) | pkt[3];
    if (total < 4 || total != pkt_len)
        return;

    switch (pkt[0]) {
        case PPP_CODE_CONFIGURE_REQUEST:
            if (ccp_request_options_valid(pkt, total))
                ccp_handle_config_request(ctx, pkt, total);
            break;

        case PPP_CODE_CONFIGURE_ACK:
            if (ccp_response_matches_request(ctx, pkt, total)) {
                ctx->ccp_req_sent = false;
                ctx->ccp_ack_received = true;
                if (ctx->ccp_request_bits != 0) {
                    ppp_mppe_configure(&ctx->mppe_rx,
                                       ccp_key_bits_to_length(ctx->ccp_request_bits),
                                       !(ctx->ccp_request_bits & CCP_MPPE_STATELESS));
                    ppp_ccp_codec_set(ctx, false, PPP_CCP_METHOD_NONE);
                } else if (ctx->ccp_request_method == PPP_CCP_METHOD_NT31RAS) {
                    uint32_t recv_features = ccp_get_le_u32(ctx->ccp_request + 10);
                    ctx->ccp_rx_window_size = ccp_nt31ras_window_size(recv_features);
                    if (!ctx->ccp_rx_window_size
                        || !ppp_ccp_codec_set_window(ctx, false,
                                                     ctx->ccp_request_method,
                                                     ctx->ccp_rx_window_size)) {
                        ctx->state = PPP_STATE_DEAD;
                        break;
                    }
                } else if (!ppp_ccp_codec_set(ctx, false, ctx->ccp_request_method)) {
                    ctx->state = PPP_STATE_DEAD;
                    break;
                }
                ccp_update_state(ctx);
                ppp_advance_state(ctx);
            }
            break;

        case PPP_CODE_CONFIGURE_NAK:
            {
                uint32_t proposed_bits;
                if (ctx->ccp_req_sent && pkt[1] == ctx->ccp_request_id
                    && ccp_parse_mppe_nak(pkt, total, &proposed_bits)
                    && (proposed_bits & ~ctx->ccp_request_bits) == 0
                    && ccp_key_bits_to_length(proposed_bits) >= ctx->mppe_min_bits
                    && ctx->ccp_retries++ < 10) {
                    ctx->ccp_req_sent = false;
                    ctx->ccp_ack_received = false;
                    ccp_send_request(ctx, proposed_bits);
                } else if (ctx->ccp_req_sent && pkt[1] == ctx->ccp_request_id
                           && ccp_retry_nt31ras_nak(ctx, pkt, total)) {
                    break;
                } else if (ctx->ccp_req_sent && pkt[1] == ctx->ccp_request_id
                           && ccp_try_next_codec(ctx)) {
                    break;
                } else if (ctx->ccp_req_sent && pkt[1] == ctx->ccp_request_id) {
                    ppp_ccp_fallback_plaintext(ctx);
                    ppp_advance_state(ctx);
                }
            }
            break;

        case PPP_CODE_CONFIGURE_REJECT:
            if (ccp_response_matches_request(ctx, pkt, total)) {
                ctx->ccp_rejected = true;
                if (!ccp_try_next_codec(ctx)) {
                    ppp_ccp_fallback_plaintext(ctx);
                    ppp_advance_state(ctx);
                }
            }
            break;

        case PPP_CODE_TERMINATE_REQUEST:
            ccp_send_response(ctx, PPP_CODE_TERMINATE_ACK, pkt[1], NULL, 0);
            ctx->ccp_ack_sent = false;
            ctx->ccp_ack_received = false;
            ctx->ccp_peer_mppe = false;
            ppp_ccp_fallback_plaintext(ctx);
            ppp_advance_state(ctx);
            break;

        case PPP_CODE_TERMINATE_ACK:
            ctx->ccp_ack_sent = false;
            ctx->ccp_ack_received = false;
            ctx->ccp_peer_mppe = false;
            ppp_ccp_fallback_plaintext(ctx);
            ppp_advance_state(ctx);
            break;

        case PPP_CODE_RESET_REQUEST:
            ccp_send_response(ctx, PPP_CODE_RESET_ACK, pkt[1], NULL, 0);
            ppp_mppe_request_rekey(&ctx->mppe_tx);
            if (ctx->ccp_tx_method != PPP_CCP_METHOD_NONE)
                ppp_ccp_codec_set_window(ctx, true, ctx->ccp_tx_method,
                                         ctx->ccp_tx_window_size
                                             ? ctx->ccp_tx_window_size : 8192);
            break;

        case PPP_CODE_RESET_ACK:
            if (total == 4 && ctx->ccp_reset_pending
                && pkt[1] == ctx->ccp_reset_request_id) {
                ctx->ccp_reset_pending = false;
                if (ctx->ccp_rx_method != PPP_CCP_METHOD_NONE)
                    ppp_ccp_codec_set_window(ctx, false, ctx->ccp_rx_method,
                                             ctx->ccp_rx_window_size
                                                 ? ctx->ccp_rx_window_size : 8192);
            }
            break;

        default:
            break;
    }
}

void
ppp_ccp_fallback_plaintext(ppp_ctx_t *ctx)
{
    if (!ctx)
        return;
    ctx->ccp_req_sent = false;
    ctx->ccp_ack_sent = false;
    ctx->ccp_ack_received = false;
    ctx->ccp_peer_mppe = false;
    ctx->ccp_open = false;
    ctx->ccp_plaintext_fallback = true;
    ctx->mppe_tx_enabled = false;
    ctx->mppe_rx_enabled = false;
    ctx->ccp_reset_pending = false;
    ctx->ras_rx_len = 0;
    ctx->ras_rx_expected = 0;
    ctx->ras_rx_in_frame = false;
    ppp_ccp_codec_close(ctx);
    if (ctx->mppe_min_bits > 0) {
        ctx->ccp_plaintext_fallback = false;
        ctx->state = PPP_STATE_DEAD;
    }
}