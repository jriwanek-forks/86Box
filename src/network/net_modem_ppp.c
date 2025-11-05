/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          PPP (Point-to-Point Protocol) implementation for modem emulation.
 *          HDLC-like framing (RFC 1662), FCS-16 CRC, and LCP negotiation
 *          (RFC 1661).
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <86box/net_modem_ppp.h>
#include <86box/net_modem_pap.h>
#include <86box/net_modem_chap.h>
#include <86box/net_modem_ipcp.h>

#ifdef ENABLE_MODEM_LOG
extern uint8_t modem_do_log;

static void
ppp_log(const char *fmt, ...)
{
    va_list ap;
    if (modem_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define ppp_log(fmt, ...)
#endif

/* FCS-16 lookup table (CRC-CCITT reflected, polynomial 0x8408) from RFC 1662 */
static const uint16_t fcs16_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
};

uint16_t
ppp_fcs16(const uint8_t *data, int len)
{
    uint16_t fcs = 0xFFFF;
    for (int i = 0; i < len; i++)
        fcs = (fcs >> 8) ^ fcs16_table[(fcs ^ data[i]) & 0xFF];
    return fcs ^ 0xFFFF;
}

/* Simple PRNG for magic numbers (doesn't need to be cryptographic) */
static uint32_t ppp_rand_state = 0x12345678;

static uint32_t
ppp_rand32(void)
{
    ppp_rand_state ^= ppp_rand_state << 13;
    ppp_rand_state ^= ppp_rand_state >> 17;
    ppp_rand_state ^= ppp_rand_state << 5;
    return ppp_rand_state;
}

/* Send raw HDLC-framed data to the serial line */
void
ppp_send_frame(ppp_ctx_t *ctx, uint16_t protocol, const uint8_t *data, int len)
{
    uint8_t  frame[PPP_MAX_FRAME * 2];
    uint8_t  raw[PPP_MAX_FRAME];
    int      raw_len = 0;
    int      out_len = 0;
    uint16_t fcs;

    /* Build unescaped frame: Address + Control + Protocol + Data */
    raw[raw_len++] = PPP_ADDRESS;
    raw[raw_len++] = PPP_CONTROL;
    raw[raw_len++] = (uint8_t) (protocol >> 8);
    raw[raw_len++] = (uint8_t) (protocol & 0xFF);
    if (data && len > 0) {
        memcpy(raw + raw_len, data, len);
        raw_len += len;
    }

    /* Calculate FCS over Address + Control + Protocol + Data */
    fcs = ppp_fcs16(raw, raw_len);
    raw[raw_len++] = (uint8_t) (fcs & 0xFF);
    raw[raw_len++] = (uint8_t) (fcs >> 8);

    /* HDLC escape and frame */
    frame[out_len++] = PPP_FLAG;
    for (int i = 0; i < raw_len; i++) {
        uint8_t c = raw[i];
        if (c == PPP_FLAG || c == PPP_ESCAPE || (c < 0x20 && (ctx->our_accm & (1u << c)))) {
            frame[out_len++] = PPP_ESCAPE;
            frame[out_len++] = c ^ 0x20;
        } else {
            frame[out_len++] = c;
        }
    }
    frame[out_len++] = PPP_FLAG;

    ctx->serial_push(ctx->modem, frame, out_len);
}

/* Build and send an LCP Configuration-Request */
static void
ppp_send_lcp_config_request(ppp_ctx_t *ctx)
{
    uint8_t pkt[64];
    int     len = 0;

    /* LCP header: Code(1) + Identifier(1) + Length(2) */
    pkt[0] = PPP_CODE_CONFIGURE_REQUEST;
    pkt[1] = ctx->lcp_id++;
    /* Length filled in later at offset 2-3 */
    len = 4;

    /* Option: MRU (type=1, len=4, value=1500) */
    pkt[len++] = LCP_OPT_MRU;
    pkt[len++] = 4;
    pkt[len++] = (uint8_t) (ctx->our_mru >> 8);
    pkt[len++] = (uint8_t) (ctx->our_mru & 0xFF);

    /* Option: ACCM (type=2, len=6, value=0x00000000) */
    pkt[len++] = LCP_OPT_ACCM;
    pkt[len++] = 6;
    pkt[len++] = (uint8_t) (ctx->our_accm >> 24);
    pkt[len++] = (uint8_t) (ctx->our_accm >> 16);
    pkt[len++] = (uint8_t) (ctx->our_accm >> 8);
    pkt[len++] = (uint8_t) (ctx->our_accm & 0xFF);

    /* Option: Auth Protocol (type=3) - if we want authentication */
    if (ctx->auth_type != PPP_AUTH_NONE) {
        pkt[len++] = LCP_OPT_AUTH_PROTO;
        if (ctx->auth_type == PPP_AUTH_PAP) {
            pkt[len++] = 4;
            pkt[len++] = (uint8_t) (PPP_AUTH_PROTO_PAP >> 8);
            pkt[len++] = (uint8_t) (PPP_AUTH_PROTO_PAP & 0xFF);
        } else {
            /* CHAP variants */
            pkt[len++] = 5;
            pkt[len++] = (uint8_t) (PPP_AUTH_PROTO_CHAP >> 8);
            pkt[len++] = (uint8_t) (PPP_AUTH_PROTO_CHAP & 0xFF);
            switch (ctx->auth_type) {
                case PPP_AUTH_CHAP_MD5:  pkt[len++] = CHAP_ALG_MD5;      break;
                case PPP_AUTH_MSCHAP:    pkt[len++] = CHAP_ALG_MSCHAP;   break;
                case PPP_AUTH_MSCHAPV2:  pkt[len++] = CHAP_ALG_MSCHAPV2; break;
                default:                 pkt[len++] = CHAP_ALG_MD5;      break;
            }
        }
    }

    /* Option: Magic Number (type=5, len=6) */
    pkt[len++] = LCP_OPT_MAGIC_NUMBER;
    pkt[len++] = 6;
    pkt[len++] = (uint8_t) (ctx->our_magic >> 24);
    pkt[len++] = (uint8_t) (ctx->our_magic >> 16);
    pkt[len++] = (uint8_t) (ctx->our_magic >> 8);
    pkt[len++] = (uint8_t) (ctx->our_magic & 0xFF);

    /* Fill in length */
    pkt[2] = (uint8_t) (len >> 8);
    pkt[3] = (uint8_t) (len & 0xFF);

    ppp_send_frame(ctx, PPP_PROTO_LCP, pkt, len);
    ctx->lcp_req_sent = true;
    ppp_log("PPP: Sent LCP Configure-Request (id=%d)\n", pkt[1]);
}

/* Handle LCP Configure-Request from peer */
static void
ppp_handle_lcp_config_request(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    uint8_t ack[PPP_MAX_FRAME];
    uint8_t nak[PPP_MAX_FRAME];
    uint8_t rej[PPP_MAX_FRAME];
    int     ack_len = 4, nak_len = 4, rej_len = 4;
    uint8_t id      = pkt[1];
    int     total   = (pkt[2] << 8) | pkt[3];
    int     pos     = 4;

    ppp_log("PPP: Received LCP Configure-Request (id=%d, len=%d)\n", id, total);

    while (pos < total && pos < pkt_len) {
        uint8_t opt_type = pkt[pos];
        uint8_t opt_len  = (pos + 1 < pkt_len) ? pkt[pos + 1] : 0;

        if (opt_len < 2 || pos + opt_len > pkt_len)
            break;

        switch (opt_type) {
            case LCP_OPT_MRU:
                if (opt_len == 4) {
                    ctx->peer_mru = (uint16_t) ((pkt[pos + 2] << 8) | pkt[pos + 3]);
                    memcpy(ack + ack_len, pkt + pos, opt_len);
                    ack_len += opt_len;
                } else {
                    memcpy(rej + rej_len, pkt + pos, opt_len);
                    rej_len += opt_len;
                }
                break;

            case LCP_OPT_ACCM:
                if (opt_len == 6) {
                    ctx->peer_accm = ((uint32_t) pkt[pos + 2] << 24)
                                   | ((uint32_t) pkt[pos + 3] << 16)
                                   | ((uint32_t) pkt[pos + 4] << 8)
                                   | (uint32_t) pkt[pos + 5];
                    memcpy(ack + ack_len, pkt + pos, opt_len);
                    ack_len += opt_len;
                } else {
                    memcpy(rej + rej_len, pkt + pos, opt_len);
                    rej_len += opt_len;
                }
                break;

            case LCP_OPT_MAGIC_NUMBER:
                if (opt_len == 6) {
                    ctx->peer_magic = ((uint32_t) pkt[pos + 2] << 24)
                                    | ((uint32_t) pkt[pos + 3] << 16)
                                    | ((uint32_t) pkt[pos + 4] << 8)
                                    | (uint32_t) pkt[pos + 5];
                    memcpy(ack + ack_len, pkt + pos, opt_len);
                    ack_len += opt_len;
                } else {
                    memcpy(rej + rej_len, pkt + pos, opt_len);
                    rej_len += opt_len;
                }
                break;

            case LCP_OPT_PFC:
                ctx->peer_pfc = true;
                memcpy(ack + ack_len, pkt + pos, opt_len);
                ack_len += opt_len;
                break;

            case LCP_OPT_ACFC:
                ctx->peer_acfc = true;
                memcpy(ack + ack_len, pkt + pos, opt_len);
                ack_len += opt_len;
                break;

            case LCP_OPT_AUTH_PROTO:
                /* We are the server; we don't accept the client requesting auth from us.
                   Reject this option so they don't try to authenticate us. */
                memcpy(rej + rej_len, pkt + pos, opt_len);
                rej_len += opt_len;
                break;

            default:
                /* Unknown option - reject */
                memcpy(rej + rej_len, pkt + pos, opt_len);
                rej_len += opt_len;
                break;
        }
        pos += opt_len;
    }

    if (rej_len > 4) {
        /* Send Configure-Reject */
        rej[0] = PPP_CODE_CONFIGURE_REJECT;
        rej[1] = id;
        rej[2] = (uint8_t) (rej_len >> 8);
        rej[3] = (uint8_t) (rej_len & 0xFF);
        ppp_send_frame(ctx, PPP_PROTO_LCP, rej, rej_len);
        ppp_log("PPP: Sent LCP Configure-Reject\n");
    } else if (nak_len > 4) {
        /* Send Configure-Nak */
        nak[0] = PPP_CODE_CONFIGURE_NAK;
        nak[1] = id;
        nak[2] = (uint8_t) (nak_len >> 8);
        nak[3] = (uint8_t) (nak_len & 0xFF);
        ppp_send_frame(ctx, PPP_PROTO_LCP, nak, nak_len);
        ppp_log("PPP: Sent LCP Configure-Nak\n");
    } else {
        /* Send Configure-Ack */
        ack[0] = PPP_CODE_CONFIGURE_ACK;
        ack[1] = id;
        ack[2] = (uint8_t) (ack_len >> 8);
        ack[3] = (uint8_t) (ack_len & 0xFF);
        ppp_send_frame(ctx, PPP_PROTO_LCP, ack, ack_len);
        ctx->lcp_ack_sent = true;
        ppp_log("PPP: Sent LCP Configure-Ack\n");
    }
}

/* Handle LCP Configure-Ack from peer */
static void
ppp_handle_lcp_config_ack(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    (void) pkt;
    (void) pkt_len;
    ppp_log("PPP: Received LCP Configure-Ack\n");
    ctx->lcp_ack_received = true;
    ppp_advance_state(ctx);
}

/* Handle LCP Configure-Nak from peer */
static void
ppp_handle_lcp_config_nak(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    int total = (pkt[2] << 8) | pkt[3];
    int pos   = 4;

    ppp_log("PPP: Received LCP Configure-Nak\n");

    while (pos < total && pos < pkt_len) {
        uint8_t opt_type = pkt[pos];
        uint8_t opt_len  = (pos + 1 < pkt_len) ? pkt[pos + 1] : 0;

        if (opt_len < 2 || pos + opt_len > pkt_len)
            break;

        switch (opt_type) {
            case LCP_OPT_AUTH_PROTO:
                /* Peer is suggesting a different auth protocol.
                   If we can accommodate, switch to it. */
                if (opt_len >= 4) {
                    uint16_t proto = (uint16_t) ((pkt[pos + 2] << 8) | pkt[pos + 3]);
                    if (proto == PPP_AUTH_PROTO_PAP) {
                        ctx->auth_type = PPP_AUTH_PAP;
                    } else if (proto == PPP_AUTH_PROTO_CHAP && opt_len >= 5) {
                        switch (pkt[pos + 4]) {
                            case CHAP_ALG_MD5:      ctx->auth_type = PPP_AUTH_CHAP_MD5; break;
                            case CHAP_ALG_MSCHAP:   ctx->auth_type = PPP_AUTH_MSCHAP;   break;
                            case CHAP_ALG_MSCHAPV2: ctx->auth_type = PPP_AUTH_MSCHAPV2; break;
                            default:                ctx->auth_type = PPP_AUTH_PAP;       break;
                        }
                    }
                }
                break;

            default:
                break;
        }
        pos += opt_len;
    }

    /* Resend Configure-Request with updated options */
    if (ctx->lcp_retries++ < 10) {
        ppp_send_lcp_config_request(ctx);
    }
}

/* Handle LCP Configure-Reject from peer */
static void
ppp_handle_lcp_config_reject(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    int total = (pkt[2] << 8) | pkt[3];
    int pos   = 4;

    ppp_log("PPP: Received LCP Configure-Reject\n");

    while (pos < total && pos < pkt_len) {
        uint8_t opt_type = pkt[pos];
        uint8_t opt_len  = (pos + 1 < pkt_len) ? pkt[pos + 1] : 0;

        if (opt_len < 2 || pos + opt_len > pkt_len)
            break;

        if (opt_type == LCP_OPT_AUTH_PROTO) {
            /* Peer rejected authentication - proceed without */
            ctx->auth_type     = PPP_AUTH_NONE;
            ctx->auth_complete = true;
        }
        pos += opt_len;
    }

    /* Resend without rejected options */
    if (ctx->lcp_retries++ < 10) {
        ppp_send_lcp_config_request(ctx);
    }
}

/* Handle LCP Echo-Request */
static void
ppp_handle_lcp_echo_request(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    uint8_t reply[PPP_MAX_FRAME];
    int     total = (pkt[2] << 8) | pkt[3];

    if (total > (int) sizeof(reply))
        total = sizeof(reply);

    memcpy(reply, pkt, total);
    reply[0] = PPP_CODE_ECHO_REPLY;

    /* Replace magic number with ours */
    if (total >= 8) {
        reply[4] = (uint8_t) (ctx->our_magic >> 24);
        reply[5] = (uint8_t) (ctx->our_magic >> 16);
        reply[6] = (uint8_t) (ctx->our_magic >> 8);
        reply[7] = (uint8_t) (ctx->our_magic & 0xFF);
    }

    ppp_send_frame(ctx, PPP_PROTO_LCP, reply, total);
    ppp_log("PPP: Sent LCP Echo-Reply\n");
}

/* Handle LCP Terminate-Request */
static void
ppp_handle_lcp_terminate_request(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    uint8_t reply[8];

    (void) pkt_len;

    ppp_log("PPP: Received LCP Terminate-Request\n");
    reply[0] = PPP_CODE_TERMINATE_ACK;
    reply[1] = pkt[1]; /* echo identifier */
    reply[2] = 0;
    reply[3] = 4;
    ppp_send_frame(ctx, PPP_PROTO_LCP, reply, 4);

    ctx->state = PPP_STATE_DEAD;
}

/* Process a complete LCP packet */
static void
ppp_process_lcp(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    if (pkt_len < 4)
        return;

    uint8_t code = pkt[0];

    switch (code) {
        case PPP_CODE_CONFIGURE_REQUEST:
            ppp_handle_lcp_config_request(ctx, pkt, pkt_len);
            ppp_advance_state(ctx);
            break;
        case PPP_CODE_CONFIGURE_ACK:
            ppp_handle_lcp_config_ack(ctx, pkt, pkt_len);
            break;
        case PPP_CODE_CONFIGURE_NAK:
            ppp_handle_lcp_config_nak(ctx, pkt, pkt_len);
            break;
        case PPP_CODE_CONFIGURE_REJECT:
            ppp_handle_lcp_config_reject(ctx, pkt, pkt_len);
            break;
        case PPP_CODE_ECHO_REQUEST:
            ppp_handle_lcp_echo_request(ctx, pkt, pkt_len);
            break;
        case PPP_CODE_TERMINATE_REQUEST:
            ppp_handle_lcp_terminate_request(ctx, pkt, pkt_len);
            break;
        case PPP_CODE_ECHO_REPLY:
        case PPP_CODE_DISCARD_REQUEST:
        case PPP_CODE_TERMINATE_ACK:
            /* Silently ignore */
            break;
        case PPP_CODE_PROTOCOL_REJECT:
            ppp_log("PPP: Received Protocol-Reject\n");
            break;
        default:
            ppp_log("PPP: Unknown LCP code %d\n", code);
            break;
    }
}

/* Advance the PPP state machine */
void
ppp_advance_state(ppp_ctx_t *ctx)
{
    switch (ctx->state) {
        case PPP_STATE_LCP_NEGOTIATE:
            if (ctx->lcp_ack_sent && ctx->lcp_ack_received) {
                ppp_log("PPP: LCP opened, moving to auth phase\n");
                if (ctx->auth_type != PPP_AUTH_NONE && !ctx->auth_complete) {
                    ctx->state = PPP_STATE_AUTH;
                    if (ctx->auth_type == PPP_AUTH_CHAP_MD5
                     || ctx->auth_type == PPP_AUTH_MSCHAP
                     || ctx->auth_type == PPP_AUTH_MSCHAPV2) {
                        ppp_chap_send_challenge(ctx);
                    }
                    /* PAP: server waits for client to send Authenticate-Request */
                } else {
                    ctx->auth_complete = true;
                    ctx->state         = PPP_STATE_IPCP_NEGOTIATE;
                    ppp_ipcp_send_config_request(ctx);
                }
            }
            break;

        case PPP_STATE_AUTH:
            if (ctx->auth_complete) {
                ppp_log("PPP: Auth complete, moving to IPCP\n");
                ctx->state = PPP_STATE_IPCP_NEGOTIATE;
                ppp_ipcp_send_config_request(ctx);
            }
            break;

        case PPP_STATE_IPCP_NEGOTIATE:
            if (ctx->ipcp_ack_sent && ctx->ipcp_ack_received) {
                ppp_log("PPP: IPCP opened, entering network phase\n");
                ctx->state = PPP_STATE_NETWORK;
            }
            break;

        default:
            break;
    }
}

/* Process a complete PPP frame (after HDLC un-escaping and FCS check) */
static void
ppp_process_frame(ppp_ctx_t *ctx, const uint8_t *frame, int frame_len)
{
    uint16_t protocol;
    int      data_offset;

    if (frame_len < 2)
        return;

    /* Check for Address and Control fields */
    if (frame[0] == PPP_ADDRESS && frame[1] == PPP_CONTROL) {
        if (frame_len < 4)
            return;
        protocol    = (uint16_t) ((frame[2] << 8) | frame[3]);
        data_offset = 4;
    } else if (ctx->peer_acfc) {
        /* Address/Control Field Compression - fields are omitted */
        if (frame[0] & 0x01) {
            /* Protocol Field Compression - single byte protocol */
            protocol    = frame[0];
            data_offset = 1;
        } else {
            protocol    = (uint16_t) ((frame[0] << 8) | frame[1]);
            data_offset = 2;
        }
    } else {
        ppp_log("PPP: Frame missing Address/Control fields\n");
        return;
    }

    const uint8_t *data     = frame + data_offset;
    int            data_len = frame_len - data_offset;

    ppp_log("PPP: Received frame proto=0x%04X len=%d (state=%d)\n", protocol, data_len, ctx->state);

    switch (protocol) {
        case PPP_PROTO_LCP:
            ppp_process_lcp(ctx, data, data_len);
            break;

        case PPP_PROTO_PAP:
            if (ctx->state == PPP_STATE_AUTH) {
                ppp_pap_process(ctx, data, data_len);
            }
            break;

        case PPP_PROTO_CHAP:
            if (ctx->state == PPP_STATE_AUTH) {
                ppp_chap_process(ctx, data, data_len);
            }
            break;

        case PPP_PROTO_IPCP:
            if (ctx->state >= PPP_STATE_IPCP_NEGOTIATE) {
                ppp_ipcp_process(ctx, data, data_len);
            }
            break;

        case PPP_PROTO_IP:
            if (ctx->state == PPP_STATE_NETWORK) {
                ctx->network_send_ip(ctx->modem, data, data_len);
            }
            break;

        default:
            /* Send Protocol-Reject for unknown protocols */
            if (ctx->state >= PPP_STATE_LCP_NEGOTIATE) {
                uint8_t reject[PPP_MAX_FRAME];
                int     rlen = 4;
                reject[0] = PPP_CODE_PROTOCOL_REJECT;
                reject[1] = ctx->lcp_id++;
                /* Include rejected protocol and up to 64 bytes of data */
                reject[rlen++] = (uint8_t) (protocol >> 8);
                reject[rlen++] = (uint8_t) (protocol & 0xFF);
                int copy = (data_len > 64) ? 64 : data_len;
                memcpy(reject + rlen, data, copy);
                rlen += copy;
                reject[2] = (uint8_t) (rlen >> 8);
                reject[3] = (uint8_t) (rlen & 0xFF);
                ppp_send_frame(ctx, PPP_PROTO_LCP, reject, rlen);
                ppp_log("PPP: Sent Protocol-Reject for 0x%04X\n", protocol);
            }
            break;
    }
}

/* Process a byte received from the serial line (HDLC framing) */
void
ppp_rx_byte(ppp_ctx_t *ctx, uint8_t byte)
{
    if (byte == PPP_FLAG) {
        if (ctx->rx_in_frame && ctx->rx_len > 2) {
            /* End of frame - check FCS */
            uint16_t calc_fcs = ppp_fcs16(ctx->rx_buf, ctx->rx_len - 2);
            uint16_t recv_fcs = (uint16_t) (ctx->rx_buf[ctx->rx_len - 2])
                              | ((uint16_t) (ctx->rx_buf[ctx->rx_len - 1]) << 8);

            if (calc_fcs == recv_fcs) {
                ppp_process_frame(ctx, ctx->rx_buf, ctx->rx_len - 2);
            } else {
                ppp_log("PPP: FCS error (calc=0x%04X recv=0x%04X)\n", calc_fcs, recv_fcs);
            }
        }
        /* Start new frame */
        ctx->rx_len      = 0;
        ctx->rx_in_frame = true;
        ctx->rx_escaped  = false;
        return;
    }

    if (!ctx->rx_in_frame)
        return;

    if (byte == PPP_ESCAPE) {
        ctx->rx_escaped = true;
        return;
    }

    if (ctx->rx_escaped) {
        byte ^= 0x20;
        ctx->rx_escaped = false;
    }

    if (ctx->rx_len < PPP_MAX_FRAME)
        ctx->rx_buf[ctx->rx_len++] = byte;
}

/* Wrap an IP packet in PPP HDLC framing for delivery to the guest */
void
ppp_wrap_ip(ppp_ctx_t *ctx, const uint8_t *ip_pkt, int len)
{
    if (ctx->state != PPP_STATE_NETWORK)
        return;

    ppp_send_frame(ctx, PPP_PROTO_IP, ip_pkt, len);
}

/* Initialize PPP context */
ppp_ctx_t *
ppp_init(void *modem,
         void (*serial_push)(void *, const uint8_t *, int),
         void (*network_send_ip)(void *, const uint8_t *, int))
{
    ppp_ctx_t *ctx = (ppp_ctx_t *) calloc(1, sizeof(ppp_ctx_t));
    if (!ctx)
        return NULL;

    ctx->modem           = modem;
    ctx->serial_push     = serial_push;
    ctx->network_send_ip = network_send_ip;
    ctx->state           = PPP_STATE_DEAD;
    ctx->our_mru         = PPP_DEFAULT_MRU;
    ctx->peer_mru        = PPP_DEFAULT_MRU;
    ctx->our_accm        = 0xFFFFFFFF; /* Send all control chars escaped initially */
    ctx->peer_accm       = 0xFFFFFFFF;
    ctx->our_magic       = ppp_rand32();
    ctx->auth_type       = PPP_AUTH_NONE;

    /* Default IP configuration (SLiRP defaults) */
    ctx->our_ip  = 0x0A000202; /* 10.0.2.2 */
    ctx->peer_ip = 0x0A00020F; /* 10.0.2.15 */
    ctx->dns1    = 0x0A000203; /* 10.0.2.3 */
    ctx->dns2    = 0x0A000203; /* 10.0.2.3 */

    return ctx;
}

void
ppp_close(ppp_ctx_t *ctx)
{
    if (ctx) {
        if (ctx->state >= PPP_STATE_LCP_NEGOTIATE && ctx->state < PPP_STATE_DEAD) {
            /* Send Terminate-Request */
            uint8_t pkt[4];
            pkt[0] = PPP_CODE_TERMINATE_REQUEST;
            pkt[1] = ctx->lcp_id++;
            pkt[2] = 0;
            pkt[3] = 4;
            ppp_send_frame(ctx, PPP_PROTO_LCP, pkt, 4);
        }
        free(ctx);
    }
}

/* Start PPP negotiation (called when entering PPP mode) */
void
ppp_start(ppp_ctx_t *ctx)
{
    ctx->state            = PPP_STATE_LCP_NEGOTIATE;
    ctx->lcp_ack_sent     = false;
    ctx->lcp_ack_received = false;
    ctx->lcp_req_sent     = false;
    ctx->lcp_retries      = 0;
    ctx->rx_in_frame      = false;
    ctx->rx_len           = 0;
    ctx->rx_escaped       = false;

    ppp_send_lcp_config_request(ctx);
}
