/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          PPP (Point-to-Point Protocol) definitions for modem emulation.
 *          HDLC-like framing, FCS-16, LCP negotiation.
 *          RFC 1661 (PPP), RFC 1662 (PPP in HDLC-like Framing).
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#ifndef NET_MODEM_PPP_H
#define NET_MODEM_PPP_H

//#include <stdint.h>
//#include <stdbool.h>
//#include <stddef.h>

/* PPP HDLC framing constants */
#define PPP_FLAG     0x7E
#define PPP_ESCAPE   0x7D
#define PPP_ADDRESS  0xFF
#define PPP_CONTROL  0x03

/* PPP protocol numbers */
#define PPP_PROTO_IP   0x0021
#define PPP_PROTO_IPCP 0x8021
#define PPP_PROTO_LCP  0xC021
#define PPP_PROTO_PAP  0xC023
#define PPP_PROTO_CHAP 0xC223

/* LCP/IPCP packet codes */
#define PPP_CODE_CONFIGURE_REQUEST  1
#define PPP_CODE_CONFIGURE_ACK      2
#define PPP_CODE_CONFIGURE_NAK      3
#define PPP_CODE_CONFIGURE_REJECT   4
#define PPP_CODE_TERMINATE_REQUEST  5
#define PPP_CODE_TERMINATE_ACK      6
#define PPP_CODE_CODE_REJECT        7
#define PPP_CODE_PROTOCOL_REJECT    8
#define PPP_CODE_ECHO_REQUEST       9
#define PPP_CODE_ECHO_REPLY         10
#define PPP_CODE_DISCARD_REQUEST    11

/* LCP option types */
#define LCP_OPT_MRU           1
#define LCP_OPT_ACCM          2
#define LCP_OPT_AUTH_PROTO     3
#define LCP_OPT_QUALITY_PROTO  4
#define LCP_OPT_MAGIC_NUMBER   5
#define LCP_OPT_PFC            7
#define LCP_OPT_ACFC           8

/* IPCP option types */
#define IPCP_OPT_IP_ADDRESS   3
#define IPCP_OPT_DNS_PRIMARY  129
#define IPCP_OPT_DNS_SECONDARY 131

/* Auth protocol values for LCP option 3 */
#define PPP_AUTH_PROTO_PAP   0xC023
#define PPP_AUTH_PROTO_CHAP  0xC223

/* CHAP algorithm values */
#define CHAP_ALG_MD5       5
#define CHAP_ALG_MSCHAP    0x80
#define CHAP_ALG_MSCHAPV2  0x81

/* PPP state machine */
typedef enum {
    PPP_STATE_DEAD,
    PPP_STATE_LCP_NEGOTIATE,
    PPP_STATE_AUTH,
    PPP_STATE_IPCP_NEGOTIATE,
    PPP_STATE_NETWORK,
    PPP_STATE_TERMINATING
} ppp_state_t;

/* Authentication method */
typedef enum {
    PPP_AUTH_NONE,
    PPP_AUTH_PAP,
    PPP_AUTH_CHAP_MD5,
    PPP_AUTH_MSCHAP,
    PPP_AUTH_MSCHAPV2
} ppp_auth_type_t;

/* Maximum PPP frame size */
#define PPP_MAX_FRAME  2048
#define PPP_DEFAULT_MRU 1500

/* PPP context */
typedef struct ppp_ctx_t {
    ppp_state_t     state;

    /* LCP negotiated parameters */
    uint16_t        our_mru;
    uint16_t        peer_mru;
    uint32_t        our_accm;
    uint32_t        peer_accm;
    uint32_t        our_magic;
    uint32_t        peer_magic;
    bool            peer_pfc;
    bool            peer_acfc;

    /* LCP state tracking */
    uint8_t         lcp_id;
    bool            lcp_ack_sent;
    bool            lcp_ack_received;
    int             lcp_retries;
    bool            lcp_req_sent;

    /* Authentication */
    ppp_auth_type_t auth_type;
    uint8_t         auth_id;
    char            username[64];
    char            password[64];
    bool            auth_complete;

    /* CHAP challenge state */
    uint8_t         chap_challenge[16];
    uint8_t         chap_challenge_len;

    /* IPCP state tracking */
    uint8_t         ipcp_id;
    bool            ipcp_ack_sent;
    bool            ipcp_ack_received;
    int             ipcp_retries;
    bool            ipcp_req_sent;

    /* IP configuration */
    uint32_t        our_ip;
    uint32_t        peer_ip;
    uint32_t        dns1;
    uint32_t        dns2;

    /* HDLC frame assembly (receiving from serial) */
    uint8_t         rx_buf[PPP_MAX_FRAME];
    int             rx_len;
    bool            rx_in_frame;
    bool            rx_escaped;

    /* Back-pointer and callbacks */
    void           *modem;
    void          (*serial_push)(void *modem, const uint8_t *data, int len);
    void          (*network_send_ip)(void *modem, const uint8_t *ip_pkt, int len);
} ppp_ctx_t;

/* PPP core functions */
ppp_ctx_t *ppp_init(void *modem,
                    void (*serial_push)(void *, const uint8_t *, int),
                    void (*network_send_ip)(void *, const uint8_t *, int));
void       ppp_close(ppp_ctx_t *ctx);
void       ppp_start(ppp_ctx_t *ctx);
void       ppp_rx_byte(ppp_ctx_t *ctx, uint8_t byte);
void       ppp_wrap_ip(ppp_ctx_t *ctx, const uint8_t *ip_pkt, int len);

/* PPP frame sending (used by sub-protocols) */
void       ppp_send_frame(ppp_ctx_t *ctx, uint16_t protocol, const uint8_t *data, int len);
void       ppp_advance_state(ppp_ctx_t *ctx);

/* FCS-16 CRC */
uint16_t   ppp_fcs16(const uint8_t *data, int len);

#endif /* NET_MODEM_PPP_H */
