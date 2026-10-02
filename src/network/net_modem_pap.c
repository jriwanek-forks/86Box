/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          PAP (Password Authentication Protocol) implementation for PPP
 *          modem emulation. Server-side processing: receives and validates
 *          Authenticate-Request from the peer.
 *          RFC 1334.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <86box/net_modem_pap.h>
#include <86box/log.h>

#ifdef ENABLE_MODEM_LOG
extern uint8_t modem_do_log;

static void
pap_log(void *priv, const char *fmt, ...)
{
    va_list ap;
    if (modem_do_log) {
        va_start(ap, fmt);
        log_out(priv, fmt, ap);
        va_end(ap);
    }
}
#else
#    define pap_log(priv, fmt, ...)
#endif

/* Send a PAP Authenticate-Ack or Authenticate-Nak */
static void
ppp_pap_send_response(ppp_ctx_t *ctx, uint8_t id, bool success)
{
    uint8_t     pkt[64];
    const char *msg     = success ? "Welcome" : "Authentication failed";
    uint8_t     msg_len = (uint8_t) strlen(msg);
    int         len     = 5 + msg_len;

    pkt[0] = success ? PAP_CODE_AUTHENTICATE_ACK : PAP_CODE_AUTHENTICATE_NAK;
    pkt[1] = id;
    pkt[2] = (uint8_t) (len >> 8);
    pkt[3] = (uint8_t) (len & 0xFF);
    pkt[4] = msg_len;
    memcpy(pkt + 5, msg, msg_len);

    ppp_send_frame(ctx, PPP_PROTO_PAP, pkt, len);
    pap_log(ctx->log, "PAP: Sent %s (id=%d)\n", success ? "Ack" : "Nak", id);
}

void
ppp_pap_process(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    if (pkt_len < 4)
        return;

    uint8_t code  = pkt[0];
    uint8_t id    = pkt[1];
    int     total = (pkt[2] << 8) | pkt[3];

    if (code != PAP_CODE_AUTHENTICATE_REQUEST) {
        pap_log(ctx->log, "PAP: Unexpected code %d\n", code);
        return;
    }

    if (total < 6 || total > pkt_len)
        return;

    /* Parse: Peer-ID-Length(1) + Peer-ID + Passwd-Length(1) + Passwd */
    int     pos         = 4;
    uint8_t peer_id_len = pkt[pos++];

    if (pos + peer_id_len + 1 > total)
        return;

    char peer_id[64];
    int  copy = (peer_id_len < sizeof(peer_id) - 1) ? peer_id_len : (int) sizeof(peer_id) - 1;
    memcpy(peer_id, pkt + pos, copy);
    peer_id[copy] = '\0';
    pos += peer_id_len;

    uint8_t passwd_len = pkt[pos++];
    if (pos + passwd_len > total)
        return;

    char passwd[64];
    copy = (passwd_len < sizeof(passwd) - 1) ? passwd_len : (int) sizeof(passwd) - 1;
    memcpy(passwd, pkt + pos, copy);
    passwd[copy] = '\0';

    pap_log(ctx->log, "PAP: Authenticate-Request user='%s'\n", peer_id);

    /* Validate credentials */
    bool ok = false;
    if (ctx->username[0] == '\0' && ctx->password[0] == '\0') {
        /* No credentials configured - accept anyone */
        ok = true;
    } else {
        if (strcmp(peer_id, ctx->username) == 0 && strcmp(passwd, ctx->password) == 0)
            ok = true;
    }

    /* Securely clear password from stack */
    memset(passwd, 0, sizeof(passwd));

    ppp_pap_send_response(ctx, id, ok);

    if (ok) {
        ctx->auth_complete = true;
        ppp_advance_state(ctx);
    }
}
