/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          CHAP (Challenge-Handshake Authentication Protocol) for PPP
 *          modem emulation. Supports CHAP/MD5 (RFC 1994), MS-CHAP
 *          (RFC 2433), and MS-CHAPv2 (RFC 2759).
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#ifndef NET_MODEM_CHAP_H
#define NET_MODEM_CHAP_H

//#include <86box/net_modem_ppp.h>

/* CHAP codes */
#define CHAP_CODE_CHALLENGE 1
#define CHAP_CODE_RESPONSE  2
#define CHAP_CODE_SUCCESS   3
#define CHAP_CODE_FAILURE   4

void ppp_chap_send_challenge(ppp_ctx_t *ctx);
void ppp_chap_process(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len);

#endif /* NET_MODEM_CHAP_H */
