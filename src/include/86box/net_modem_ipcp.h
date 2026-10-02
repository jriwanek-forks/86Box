/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          IPCP (Internet Protocol Control Protocol) for PPP modem
 *          emulation. RFC 1332, RFC 1877.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#ifndef NET_MODEM_IPCP_H
#define NET_MODEM_IPCP_H

#include <86box/net_modem_ppp.h>

void ppp_ipcp_send_config_request(ppp_ctx_t *ctx);
void ppp_ipcp_process(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len);

#endif /* NET_MODEM_IPCP_H */
