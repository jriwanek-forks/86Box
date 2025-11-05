/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          IPCP (Internet Protocol Control Protocol) implementation for
 *          PPP modem emulation. Server-side: assigns IP address and DNS
 *          to client.
 *          RFC 1332 (IPCP), RFC 1877 (DNS Extensions).
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <86box/net_modem_ipcp.h>

#ifdef ENABLE_MODEM_LOG
extern uint8_t modem_do_log;

static void
ipcp_log(const char *fmt, ...)
{
    va_list ap;
    if (modem_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define ipcp_log(fmt, ...)
#endif

static inline void
ipcp_put_ip(uint8_t *p, uint32_t ip)
{
    p[0] = (uint8_t) (ip >> 24);
    p[1] = (uint8_t) (ip >> 16);
    p[2] = (uint8_t) (ip >> 8);
    p[3] = (uint8_t) ip;
}

static inline uint32_t
ipcp_get_ip(const uint8_t *p)
{
    return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16)
         | ((uint32_t) p[2] << 8) | (uint32_t) p[3];
}

/* Send IPCP Configure-Request (we request our server IP) */
void
ppp_ipcp_send_config_request(ppp_ctx_t *ctx)
{
    uint8_t pkt[32];
    int     len = 4;

    pkt[0] = PPP_CODE_CONFIGURE_REQUEST;
    pkt[1] = ctx->ipcp_id++;

    /* Option: IP Address (our server IP) */
    pkt[len++] = IPCP_OPT_IP_ADDRESS;
    pkt[len++] = 6;
    ipcp_put_ip(pkt + len, ctx->our_ip);
    len += 4;

    pkt[2] = (uint8_t) (len >> 8);
    pkt[3] = (uint8_t) (len & 0xFF);

    ppp_send_frame(ctx, PPP_PROTO_IPCP, pkt, len);
    ctx->ipcp_req_sent = true;
    ipcp_log("IPCP: Sent Configure-Request (our_ip=%d.%d.%d.%d)\n",
             (ctx->our_ip >> 24) & 0xFF, (ctx->our_ip >> 16) & 0xFF,
             (ctx->our_ip >> 8) & 0xFF, ctx->our_ip & 0xFF);
}

/* Handle IPCP Configure-Request from peer */
static void
ipcp_handle_config_request(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    uint8_t ack[PPP_MAX_FRAME];
    uint8_t nak[PPP_MAX_FRAME];
    uint8_t rej[PPP_MAX_FRAME];
    int     ack_len = 4, nak_len = 4, rej_len = 4;
    uint8_t id      = pkt[1];
    int     total   = (pkt[2] << 8) | pkt[3];
    int     pos     = 4;

    ipcp_log("IPCP: Received Configure-Request (id=%d)\n", id);

    while (pos < total && pos < pkt_len) {
        uint8_t opt_type = pkt[pos];
        uint8_t opt_len  = (pos + 1 < pkt_len) ? pkt[pos + 1] : 0;

        if (opt_len < 2 || pos + opt_len > pkt_len)
            break;

        switch (opt_type) {
            case IPCP_OPT_IP_ADDRESS:
                if (opt_len == 6) {
                    uint32_t requested_ip = ipcp_get_ip(pkt + pos + 2);
                    if (requested_ip == ctx->peer_ip || requested_ip == 0) {
                        /* Accept the request or offer our configured IP */
                        if (requested_ip == 0) {
                            /* Peer is requesting an IP - NAK with our assigned IP */
                            nak[nak_len++] = IPCP_OPT_IP_ADDRESS;
                            nak[nak_len++] = 6;
                            ipcp_put_ip(nak + nak_len, ctx->peer_ip);
                            nak_len += 4;
                        } else {
                            memcpy(ack + ack_len, pkt + pos, opt_len);
                            ack_len += opt_len;
                        }
                    } else {
                        /* Different IP requested - NAK with our preference */
                        nak[nak_len++] = IPCP_OPT_IP_ADDRESS;
                        nak[nak_len++] = 6;
                        ipcp_put_ip(nak + nak_len, ctx->peer_ip);
                        nak_len += 4;
                    }
                } else {
                    memcpy(rej + rej_len, pkt + pos, opt_len);
                    rej_len += opt_len;
                }
                break;

            case IPCP_OPT_DNS_PRIMARY:
                if (opt_len == 6) {
                    uint32_t requested_dns = ipcp_get_ip(pkt + pos + 2);
                    if (requested_dns == ctx->dns1) {
                        memcpy(ack + ack_len, pkt + pos, opt_len);
                        ack_len += opt_len;
                    } else {
                        /* NAK with our DNS */
                        nak[nak_len++] = IPCP_OPT_DNS_PRIMARY;
                        nak[nak_len++] = 6;
                        ipcp_put_ip(nak + nak_len, ctx->dns1);
                        nak_len += 4;
                    }
                } else {
                    memcpy(rej + rej_len, pkt + pos, opt_len);
                    rej_len += opt_len;
                }
                break;

            case IPCP_OPT_DNS_SECONDARY:
                if (opt_len == 6) {
                    uint32_t requested_dns = ipcp_get_ip(pkt + pos + 2);
                    if (requested_dns == ctx->dns2) {
                        memcpy(ack + ack_len, pkt + pos, opt_len);
                        ack_len += opt_len;
                    } else {
                        nak[nak_len++] = IPCP_OPT_DNS_SECONDARY;
                        nak[nak_len++] = 6;
                        ipcp_put_ip(nak + nak_len, ctx->dns2);
                        nak_len += 4;
                    }
                } else {
                    memcpy(rej + rej_len, pkt + pos, opt_len);
                    rej_len += opt_len;
                }
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
        rej[0] = PPP_CODE_CONFIGURE_REJECT;
        rej[1] = id;
        rej[2] = (uint8_t) (rej_len >> 8);
        rej[3] = (uint8_t) (rej_len & 0xFF);
        ppp_send_frame(ctx, PPP_PROTO_IPCP, rej, rej_len);
        ipcp_log("IPCP: Sent Configure-Reject\n");
    } else if (nak_len > 4) {
        nak[0] = PPP_CODE_CONFIGURE_NAK;
        nak[1] = id;
        nak[2] = (uint8_t) (nak_len >> 8);
        nak[3] = (uint8_t) (nak_len & 0xFF);
        ppp_send_frame(ctx, PPP_PROTO_IPCP, nak, nak_len);
        ipcp_log("IPCP: Sent Configure-Nak\n");
    } else {
        ack[0] = PPP_CODE_CONFIGURE_ACK;
        ack[1] = id;
        ack[2] = (uint8_t) (ack_len >> 8);
        ack[3] = (uint8_t) (ack_len & 0xFF);
        ppp_send_frame(ctx, PPP_PROTO_IPCP, ack, ack_len);
        ctx->ipcp_ack_sent = true;
        ipcp_log("IPCP: Sent Configure-Ack\n");
    }
}

/* Handle IPCP Configure-Ack from peer */
static void
ipcp_handle_config_ack(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    (void) pkt;
    (void) pkt_len;
    ipcp_log("IPCP: Received Configure-Ack\n");
    ctx->ipcp_ack_received = true;
    ppp_advance_state(ctx);
}

/* Handle IPCP Configure-Nak from peer */
static void
ipcp_handle_config_nak(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    int total = (pkt[2] << 8) | pkt[3];
    int pos   = 4;

    ipcp_log("IPCP: Received Configure-Nak\n");

    while (pos < total && pos < pkt_len) {
        uint8_t opt_type = pkt[pos];
        uint8_t opt_len  = (pos + 1 < pkt_len) ? pkt[pos + 1] : 0;

        if (opt_len < 2 || pos + opt_len > pkt_len)
            break;

        if (opt_type == IPCP_OPT_IP_ADDRESS && opt_len == 6) {
            ctx->our_ip = ipcp_get_ip(pkt + pos + 2);
            ipcp_log("IPCP: Peer suggests our IP = %d.%d.%d.%d\n",
                     (ctx->our_ip >> 24) & 0xFF, (ctx->our_ip >> 16) & 0xFF,
                     (ctx->our_ip >> 8) & 0xFF, ctx->our_ip & 0xFF);
        }
        pos += opt_len;
    }

    if (ctx->ipcp_retries++ < 10)
        ppp_ipcp_send_config_request(ctx);
}

/* Handle IPCP Configure-Reject from peer */
static void
ipcp_handle_config_reject(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    (void) pkt;
    (void) pkt_len;
    ipcp_log("IPCP: Received Configure-Reject\n");

    /* Resend without rejected options - for simplicity, just ack ourselves */
    ctx->ipcp_ack_received = true;
    ppp_advance_state(ctx);
}

void
ppp_ipcp_process(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    if (pkt_len < 4)
        return;

    uint8_t code = pkt[0];

    switch (code) {
        case PPP_CODE_CONFIGURE_REQUEST:
            ipcp_handle_config_request(ctx, pkt, pkt_len);
            ppp_advance_state(ctx);
            break;
        case PPP_CODE_CONFIGURE_ACK:
            ipcp_handle_config_ack(ctx, pkt, pkt_len);
            break;
        case PPP_CODE_CONFIGURE_NAK:
            ipcp_handle_config_nak(ctx, pkt, pkt_len);
            break;
        case PPP_CODE_CONFIGURE_REJECT:
            ipcp_handle_config_reject(ctx, pkt, pkt_len);
            break;
        case PPP_CODE_TERMINATE_REQUEST:
            {
                uint8_t reply[4];
                reply[0] = PPP_CODE_TERMINATE_ACK;
                reply[1] = pkt[1];
                reply[2] = 0;
                reply[3] = 4;
                ppp_send_frame(ctx, PPP_PROTO_IPCP, reply, 4);
            }
            break;
        default:
            ipcp_log("IPCP: Unknown code %d\n", code);
            break;
    }
}
