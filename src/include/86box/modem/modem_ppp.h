/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          PPP (Point-to-Point Protocol) definitions for modem emulation.
 *          HDLC-like framing, FCS-16, LCP (RFC 1661/1662), IPCP (RFC 1332),
 *          CCP (RFC 1962), Stac LZS (RFC 1974), MPPC (RFC 2118), BSD-Compress
 *          (RFC 1977), Deflate (RFC 1979), Predictor (RFC 1978), MPPE (RFC 3078), PAP (RFC 1334),
 *          CHAP (RFC 1994), EAP (RFC 3748), and Van Jacobson compression.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#ifndef MODEM_PPP_H
#define MODEM_PPP_H

#ifdef __cplusplus
extern "C" {
#endif

struct cslip_ctx_t;

#define PPP_MPPE_KEY_LENGTH 16
#define PPP_PAP_MAX_REQUEST_LENGTH (4 + 1 + 255 + 1 + 255)

typedef struct {
    uint8_t s[256];
    uint8_t i;
    uint8_t j;
} ppp_mppe_rc4_state_t;

typedef struct ppp_mppe_state_t {
    uint8_t  start_key[PPP_MPPE_KEY_LENGTH];
    uint8_t  mschapv1_weak_start_key[8];
    uint8_t  session_key[PPP_MPPE_KEY_LENGTH];
    ppp_mppe_rc4_state_t rc4;
    uint16_t next_count;
    uint16_t last_count;
    uint8_t  key_bits;
    uint8_t  key_length;
    bool     stateful;
    bool     receive_initialized;
    bool     force_rekey;
    bool     discard;
    bool     reset_requested;
    bool     mschapv1_key_derivation;
} ppp_mppe_state_t;

/* PPP HDLC framing constants */
#define PPP_FLAG      0x7E       /* Async HDLC Flag Sequence (RFC 1662) */
#define PPP_ESCAPE    0x7D       /* Asynchronous Control Octet Escape (RFC 1662) */
#define PPP_TRANS     0x20       /* Control Octet Transparency Modifier (XOR value, RFC 1662) */
#define PPP_ADDRESS   0xFF       /* All-Stations Address Field (RFC 1662) */
#define PPP_CONTROL   0x03       /* Unnumbered Information (UI) Control Field (RFC 1662) */
#define PPP_INITFCS   0xFFFF     /* Initial FCS 16-bit CRC Seed (RFC 1662) */
#define PPP_GOODFCS   0xF0B8     /* Valid 16-bit FCS Checksum Constant (RFC 1662) */
#define PPP_INITFCS32 0xFFFFFFFF /* Initial FCS 32-bit CRC Seed (RFC 1662) */
#define PPP_GOODFCS32 0xDEBB20E3 /* Valid 32-bit FCS Checksum Constant (RFC 1662) */

/* PPP Protocol Numbers */
#define PPP_PROTO_IP              0x0021 /* Internet Protocol (RFC 1661) */
#define PPP_PROTO_VJ_COMPRESSED   0x002D /* Van Jacobson Compressed TCP/IP (RFC 1144 / RFC 1332) */
#define PPP_PROTO_VJ_UNCOMPRESSED 0x002F /* Van Jacobson Uncompressed TCP/IP (RFC 1144 / RFC 1332) */
#define PPP_PROTO_IPV6            0x0057 /* Internet Protocol Version 6 (RFC 5072) */
#define PPP_PROTO_COMPRESSED      0x00FB /* Compressed Datagram / Standard CCP Encapsulated Payload (RFC 1962) */
#define PPP_PROTO_MPPE            0x00FD /* Compressed Datagram / MPPE Payload (RFC 1962 / RFC 2419) */
#define PPP_PROTO_IPCP            0x8021 /* Internet Protocol Control Protocol (RFC 1332) */
#define PPP_PROTO_MPLSCP          0x802B /* MPLS Control Protocol (RFC 3032) */
#define PPP_PROTO_IPV6CP          0x8057 /* IPv6 Control Protocol (RFC 5072) */
#define PPP_PROTO_CCP             0x80FD /* Compression Control Protocol (RFC 1962) */
#define PPP_PROTO_LCP             0xC021 /* Link Control Protocol (RFC 1661) */
#define PPP_PROTO_PAP             0xC023 /* Password Authentication Protocol (RFC 1334) */
#define PPP_PROTO_SPAP            0xC027 /* Shiva Password Authentication Protocol */
#define PPP_PROTO_CBCP            0xC029 /* Callback Control Protocol (RFC 1570) */
#define PPP_PROTO_BAP             0xC02D /* Bandwidth Allocation Protocol (RFC 2125) */
#define PPP_PROTO_CHAP            0xC223 /* Challenge Handshake Authentication Protocol (RFC 1994) */
#define PPP_PROTO_EAP             0xC227 /* Extensible Authentication Protocol (RFC 2284 / RFC 3748) */
#define PPP_PROTO_BACP            0xC22B /* Bandwidth Allocation Control Protocol (RFC 2125) */

/* LCP/IPCP Packet Codes */
#define PPP_CODE_VENDOR_EXTENSION   0 /* Vendor-Specific Code (RFC 2153) */
#define PPP_CODE_CONFIGURE_REQUEST  1 /* Configure-Request (RFC 1661) */
#define PPP_CODE_CONFIGURE_ACK      2 /* Configure-Ack (RFC 1661) */
#define PPP_CODE_CONFIGURE_NAK      3 /* Configure-Nak (RFC 1661) */
#define PPP_CODE_CONFIGURE_REJECT   4 /* Configure-Reject (RFC 1661) */
#define PPP_CODE_TERMINATE_REQUEST  5 /* Terminate-Request (RFC 1661) */
#define PPP_CODE_TERMINATE_ACK      6 /* Terminate-Ack (RFC 1661) */
#define PPP_CODE_CODE_REJECT        7 /* Code-Reject (RFC 1661) */
#define PPP_CODE_PROTOCOL_REJECT    8 /* Protocol-Reject (LCP Only, RFC 1661) */
#define PPP_CODE_ECHO_REQUEST       9 /* Echo-Request (LCP Only, RFC 1661) */
#define PPP_CODE_ECHO_REPLY        10 /* Echo-Reply (LCP Only, RFC 1661) */
#define PPP_CODE_DISCARD_REQUEST   11 /* Discard-Request (LCP Only, RFC 1661) */
#define PPP_CODE_IDENTIFICATION    12 /* Identification (LCP Only, RFC 1570) */
#define PPP_CODE_TIME_REMAINING    13 /* Time-Remaining (LCP Only, RFC 1570) */
#define PPP_CODE_RESET_REQUEST     14 /* Reset-Request (CCP/NCP Specific, RFC 1962) */
#define PPP_CODE_RESET_ACK         15 /* Reset-Ack (CCP/NCP Specific, RFC 1962) */
#define PPP_CODE_SKIP_ALGORITHM    16 /* Skip-Algorithm (LCP/CCP Extensions) */
#define PPP_CODE_BONDING_COMM      17 /* Bandwidth Allocation Protocol (BAP) / Bonding Request */

/* LCP option types */
#define LCP_OPT_VENDOR_SPECIFIC        0 /* Vendor-Specific (RFC 2153) */
#define LCP_OPT_MRU                    1 /* Maximum-Receive-Unit (RFC 1661) */
#define LCP_OPT_ACCM                   2 /* Async-Control-Character-Map (RFC 1662) */
#define LCP_OPT_AUTH_PROTO             3 /* Authentication-Protocol (RFC 1661) */
#define LCP_OPT_QUALITY_PROTO          4 /* Quality-Protocol (RFC 1661) */
#define LCP_OPT_MAGIC_NUMBER           5 /* Magic-Number (RFC 1661) */
#define LCP_OPT_QUALITY_PROTO_OLD      6 /* Deprecated Quality-Protocol (RFC 1172) */
#define LCP_OPT_PFC                    7 /* Protocol-Field-Compression (RFC 1661) */
#define LCP_OPT_ACFC                   8 /* Address-and-Control-Field-Compression (RFC 1661) */
#define LCP_OPT_FCS_ALTERNATIVES       9 /* FCS-Alternatives (RFC 1570) */
#define LCP_OPT_SELF_DESCRIBING_PAD   10 /* Self-Describing-Pad (RFC 1570) */
#define LCP_OPT_NUMBERED_MODE         11 /* Numbered-Mode (RFC 1663) */
#define LCP_OPT_MULTILINK_PROCEDURE   12 /* Deprecated Multi-Link-Procedure (RFC 1351) */
#define LCP_OPT_CALLBACK              13 /* Callback (RFC 1570) */
#define LCP_OPT_CONNECT_TIME          14 /* Deprecated Connect-Time (RFC 1570) */
#define LCP_OPT_COMPOUND_FRAMES       15 /* Deprecated Compound-Frames (RFC 1570) */
#define LCP_OPT_NOMINAL_DATA_ENCAP    16 /* Deprecated Nominal-Data-Encapsulation (RFC 1570) */
#define LCP_OPT_MRRU                  17 /* Multilink Maximum-Receive-Reconstructed-Unit (RFC 1990) */
#define LCP_OPT_SHORT_SEQUENCE        18 /* Multilink Short-Sequence-Number-Header (RFC 1990) */
#define LCP_OPT_ENDPOINT_DISC         19 /* Multilink Endpoint Discriminator (RFC 1990) */
#define LCP_OPT_PROPRIETARY           20 /* Proprietary (RFC 1570) */
#define LCP_OPT_DCE_IDENTIFIER        21 /* DCE-Identifier (RFC 1976) */
#define LCP_OPT_MULTILINK_PLUS        22 /* Multi-Link-Plus-Procedure (RFC 1934) */
#define LCP_OPT_BACP_LINK_DISC        23 /* Link Discriminator for BACP (RFC 2125) */
#define LCP_OPT_LCP_AUTH              24 /* LCP-Authentication-Option (RFC 1570 / RFC 2484) */
#define LCP_OPT_COBS                  25 /* Consistent Overhead Byte Stuffing (RFC 2823) */
#define LCP_OPT_PREFIX_ELISION        26 /* Prefix elision (RFC 2686) */
#define LCP_OPT_MULTILINK_HEADER      27 /* Multilink header format (RFC 2686) */
#define LCP_OPT_INTERNATIONALIZATION  28 /* Internationalization (RFC 2484) */
#define LCP_OPT_SIMPLE_SONET_LINK     29 /* Simple Data Link on SONET/SDH (RFC 2823) */
#define LCP_OPT_LOG_OR_STATUS_COLLECT 30 /* Management-Option / Log or Status Collection */

/* IPCP option types */
#define IPCP_OPT_IP_ADDRESSES            1 /* Deprecated IP-Addresses (RFC 1172 / RFC 1332) */
#define IPCP_OPT_IP_COMPRESSION          2 /* IP-Compression-Protocol (RFC 1332) */
#define IPCP_OPT_IP_ADDRESS              3 /* IP-Address (RFC 1332)*/
#define IPCP_OPT_MOBILE_IPV4             4 /* Mobile-IPv4 (RFC 2290)*/
#define IPCP_OPT_DNS_PRIMARY           129 /* Primary DNS Server Address (RFC 1877) */
#define IPCP_OPT_NBNS_PRIMARY          130 /* Primary NetBIOS Name Server Address (RFC 1877) */
#define IPCP_OPT_DNS_SECONDARY         131 /* Secondary DNS Server Address (RFC 1877) */
#define IPCP_OPT_NBNS_SECONDARY        132 /* Secondary NetBIOS Name Server Address (RFC 1877) */
#define IPCP_OPT_IP_ADDRESS_ALLOCATION 144 /* IP-Address-Allocation / AT&T (RFC 2153) */

#define IPCP_VJ_MAX_SLOT_ID  15 /* Maximum Van Jacobson header compression slot index (RFC 1144, 16 total slots: 0-15) */
#define IPCP_VJ_COMP_SLOT_ID  1 /* Slot ID Compression flag bit in IPCP Option 2 (RFC 1332, 1 = allow slot ID compression) */

/* Auth protocol values for LCP option 3 */
#define PPP_AUTH_PROTO_SPAP_OLD            0xC021 /* Shiva Password Authentication Protocol (Early/Alternate Assignment) */
#define PPP_AUTH_PROTO_PAP                 0xC023 /* Password Authentication Protocol */
#define PPP_AUTH_PROTO_SHIVA_PAP           0xC027 /* Shiva Password Authentication Protocol */
#define PPP_AUTH_PROTO_DES_AUTH            0xC221 /* Individual Software Products DES Authentication Protocol */
#define PPP_AUTH_PROTO_CHAP                0xC223 /* Challenge-Handshake Authentication Protocol */
#define PPP_AUTH_PROTO_RSA                 0xC225 /* RSA Authentication Protocol */
#define PPP_AUTH_PROTO_EAP                 0xC227 /* Extensible Authentication Protocol */
#define PPP_AUTH_PROTO_MITSUBISHI_SIEP     0xC229 /* Mitsubishi Security Info Exchange Protocol */
#define PPP_AUTH_PROTO_VSAP                0xC05B /* Vendor-Specific Authentication Protocol */
#define PPP_AUTH_PROTO_PUBLIC_KEY          0xC26F /* Stampede Technologies Public Key Authentication Protocol */
#define PPP_AUTH_PROTO_PROPRIETARY_C281    0xC281 /* Proprietary Authentication Protocol */
#define PPP_AUTH_PROTO_PROPRIETARY_C283    0xC283 /* Proprietary Authentication Protocol */
#define PPP_AUTH_PROTO_NORTEL_MSAP         0xC285 /* Nortel Proprietary Authentication Protocol (MSAP) */
#define PPP_AUTH_PROTO_PROPRIETARY_NODE_ID 0xC481 /* Proprietary Node ID Authentication Protocol */

/* CHAP algorithm values */
#define CHAP_ALG_MD5        5   /* CHAP with MD5 (RFC 1994) */
#define CHAP_ALG_SHA1       6   /* CHAP with SHA-1 */
#define CHAP_ALG_SHA256     7   /* CHAP with SHA-256 */
#define CHAP_ALG_SHA3_256   8   /* CHAP with SHA3-256 */
#define CHAP_ALG_SHA384     9   /* CHAP with SHA-384 */
#define CHAP_ALG_SHA3_384  10   /* CHAP with SHA3-384 */
#define CHAP_ALG_SHA512    11   /* CHAP with SHA-512 */
#define CHAP_ALG_SHA3_512  12   /* CHAP with SHA3-512 */
#define CHAP_ALG_MSCHAP    0x80 /* Microsoft CHAP version 1 (RFC 2433) */
#define CHAP_ALG_MSCHAPV2  0x81 /* Microsoft CHAP version 2 (RFC 2759) */

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
    PPP_AUTH_MSCHAPV2,
    PPP_AUTH_CHAP_SHA1,
    PPP_AUTH_CHAP_SHA256,
    PPP_AUTH_CHAP_SHA384,
    PPP_AUTH_CHAP_SHA512,
    PPP_AUTH_EAP
} ppp_auth_type_t;

typedef enum {
    PPP_EAP_STATE_IDLE,
    PPP_EAP_STATE_IDENTITY,
    PPP_EAP_STATE_MD5_CHALLENGE
} ppp_eap_state_t;

struct ppp_mppp_bundle_t;

/* Maximum PPP frame size */
#define PPP_MAX_FRAME   2048 /* Maximum receive unit buffer size for raw HDLC/PPP packet framing */
#define PPP_DEFAULT_MRU 1500 /* Default Maximum Receive Unit length (RFC 1661) */

#define PPP_CCP_METHOD_NONE           0   /* OUI-based / Proprietary Compression (RFC 1962) */
#define PPP_CCP_METHOD_PREDICTOR1     1   /* Predictor Type 1 (RFC 1978) */
#define PPP_CCP_METHOD_PREDICTOR2     2   /* Predictor Type 2 (RFC 1978) */
#define PPP_CCP_METHOD_PUDDLEJUMPER   3   /* Puddle Jumper (RFC 1962) - Not yet used */
#define PPP_CCP_METHOD_HPPPC         16   /* Hewlett-Packard Packet-by-Packet Compression (RFC 1962) - Not yet implemented */
#define PPP_CCP_METHOD_LZS           17   /* Stac Electronics LZS (RFC 1974) */
#define PPP_CCP_METHOD_MPPE          18   /* Microsoft Point-to-Point Encryption / Compression (RFC 2118 / RFC 3078) */
#define PPP_CCP_METHOD_MPPC          19   /* Incorrect - Value 19 is reserved for Gandalf FZA */
#define PPP_CCP_METHOD_GANDALF_FZA   19   /* Gandalf FZA Compression (RFC 1962) - Not yet implemented */
#define PPP_CCP_METHOD_V42BIS        20   /* V.42bis compression, draft assignment (RFC 1962) - Not yet implemented */
#define PPP_CCP_METHOD_BSD           21   /* BSD Compress / LZW (RFC 1977) */
#define PPP_CCP_METHOD_MZS           22   /* Magnalink Variable Resource Compression / MZS (RFC 1962) - Not yet implemented */
#define PPP_CCP_METHOD_DCP           23   /* DCP Data Compression Protocol (RFC 1962) */
#define PPP_CCP_METHOD_LZS_DCP       23   /* Alias for PPP_CCP_METHOD_DCP */
#define PPP_CCP_METHOD_DEC           24   /* Digital Equipment Corporation Data Compression (RFC 1962) - Not yet implemented */
#define PPP_CCP_METHOD_deflate_draft 25   /* Early Deflate draft designation - Not yet implemented */
#define PPP_CCP_METHOD_DEFLATE       26   /* Deflate Compression (RFC 1979) */
#define PPP_CCP_METHOD_V44           27   /* ITU-T V.44 Packet Method Compression (RFC 3051) */
#define PPP_CCP_METHOD_V42BIS_ALT    28   /* V.42bis compression, formal assignment - Not yet implemented */
#define PPP_CCP_METHOD_LZJH          29   /* Hughes Network Systems LZJH Compression - Not yet implemented */
// 0x80 (128): Stac LZS alternative framing used across early WAN hardware (e.g., Ascend and Shiva routers) to handle LSB-first bit-ordering differences. (Check this is true)
#define PPP_CCP_METHOD_LZS_ALT       0x80 /* Stac Electronics LZS alternative bit-order framing (Type 128) - Not yet implemented, check validity */
// 0x81 (129): Microsoft legacy MPPC/MPPE negotiated via alternate option type field in early Windows dial-up client releases. (Check this is true)
#define PPP_CCP_METHOD_MPPE_ALT      0x81 /* Microsoft MPPE/MPPC alternative framing (Type 129) - Not yet implemented, check validity */
#define PPP_CCP_METHOD_LZS_EXTENDED  0x91 /* Stac LZS Extended / Ascend Proprietary (Type 145) */
#define PPP_CCP_METHOD_NT31RAS      254   /* Microsoft Windows NT 3.1 RAS Proprietary Compression */

#define PPP_RAS_SYN          0x16
#define PPP_RAS_ETX          0x03
#define PPP_RAS_SOH_DEST     0x02
#define PPP_RAS_SOH_TYPE     0x80
#define PPP_RAS_SOH_COMPRESS 0x40
#define PPP_RAS_IP_TYPE      0x0800

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
    bool            our_pfc;
    bool            our_acfc;
    bool            peer_pfc;
    bool            peer_acfc;
    bool            request_pfc;
    bool            request_acfc;
    char            multilink_group[64];
    bool            multilink_request_mrru;
    bool            multilink_request_short_sequence;
    bool            multilink_request_endpoint;
    bool            multilink_our_mrru;
    bool            multilink_peer_mrru;
    bool            multilink_our_short_sequence;
    bool            multilink_peer_short_sequence;
    bool            multilink_our_endpoint;
    bool            multilink_peer_endpoint;
    uint16_t        multilink_our_mrru_value;
    uint16_t        multilink_peer_mrru_value;
    uint8_t         multilink_peer_endpoint_data[32];
    uint8_t         multilink_peer_endpoint_length;
    struct ppp_mppp_bundle_t *multilink_bundle;

    /* LCP state tracking */
    uint8_t         lcp_id;
    bool            lcp_ack_sent;
    bool            lcp_ack_received;
    bool            ccp_reset_pending;
    int             lcp_retries;
    uint16_t        lcp_timeout_ms;
    bool            lcp_req_sent;
    uint8_t         lcp_request[64];
    uint8_t         lcp_request_len;

    /* Authentication */
    ppp_auth_type_t auth_type;
    uint8_t         auth_id;
    char            username[64];
    char            password[64];
    uint16_t        auth_timeout_ms;
    bool            auth_complete;
    uint16_t        pap_last_request_len;
    uint8_t         pap_last_request[PPP_PAP_MAX_REQUEST_LENGTH];
    bool            pap_last_request_valid;
    bool            mppe_keys_ready;
    bool            mppe_tx_enabled;
    bool            mppe_rx_enabled;
    uint8_t         mppe_min_bits;
    ppp_mppe_state_t mppe_tx;
    ppp_mppe_state_t mppe_rx;

    /* CCP state tracking */
    uint8_t         ccp_id;
    uint8_t         ccp_request_id;
    uint8_t         ccp_reset_id;
    bool            ccp_req_sent;
    bool            ccp_ack_sent;
    bool            ccp_ack_received;
    bool            ccp_peer_mppe;
    bool            ccp_open;
    bool            ccp_rejected;
    bool            ccp_plaintext_fallback;
    uint8_t         ccp_reset_request_id;
    uint32_t        ccp_request_bits;
    uint32_t        ccp_tx_bits;
    uint8_t         ccp_request_method;
    uint8_t         ccp_tx_method;
    uint8_t         ccp_rx_method;
    uint32_t        ccp_tx_window_size;
    uint32_t        ccp_rx_window_size;
    uint8_t         ccp_tx_bsd_bits;
    uint8_t         ccp_rx_bsd_bits;
    void           *ccp_tx_codec_state;
    void           *ccp_rx_codec_state;
    int             ccp_retries;
    uint8_t         ccp_request[32];
    uint8_t         ccp_request_len;

    /* NT31 RAS framing and coherency tickets. */
    bool            ras_tx_flush_pending;
    uint8_t         ras_tx_ticket_base;
    uint8_t         ras_tx_ticket_next;
    uint8_t         ras_rx_ticket_base;
    uint8_t         ras_rx_ticket_next;
    uint8_t         ras_rx_buf[PPP_MAX_FRAME + 272];
    int             ras_rx_len;
    int             ras_rx_expected;
    bool            ras_rx_in_frame;

    /* CHAP challenge state */
    uint8_t         chap_challenge[16];
    uint8_t         chap_challenge_len;

    /* EAP authenticator state */
    ppp_eap_state_t eap_state;
    uint8_t         eap_id;
    uint8_t         eap_request_id;
    uint8_t         eap_challenge[16];

    /* IPCP state tracking */
    uint8_t         ipcp_id;
    uint8_t         ipcp_request_id;
    bool            ipcp_ack_sent;
    bool            ipcp_ack_received;
    int             ipcp_retries;
    bool            ipcp_req_sent;
    uint32_t        ipcp_request_ip;
    bool            ipcp_vj_request;
    bool            vj_tx_enabled;
    bool            vj_rx_enabled;
    uint8_t         vj_tx_max_slot_id;
    uint8_t         vj_rx_max_slot_id;
    bool            vj_tx_comp_slot_id;
    bool            vj_rx_comp_slot_id;
    struct cslip_ctx_t *vj_tx_ctx;
    struct cslip_ctx_t *vj_rx_ctx;

    /* IP configuration */
    uint32_t        our_ip;
    uint32_t        peer_ip;
    uint32_t        dns1;
    uint32_t        dns2;
    uint32_t        wins1;
    uint32_t        wins2;

    /* HDLC frame assembly (receiving from serial) */
    uint8_t         rx_buf[PPP_MAX_FRAME];
    int             rx_len;
    bool            rx_in_frame;
    bool            rx_escaped;

    /* Back-pointer and callbacks */
    void           *modem;
    void           *log;
    void          (*serial_push)(void *modem, const uint8_t *data, int len);
    void          (*network_send_ip)(void *modem, const uint8_t *ip_pkt, int len);
} ppp_ctx_t;

/* PPP core functions */
ppp_ctx_t *ppp_init(void *modem, void *log,
                    void (*serial_push)(void *, const uint8_t *, int),
                    void (*network_send_ip)(void *, const uint8_t *, int));
void       ppp_close(ppp_ctx_t *ctx);
void       ppp_start(ppp_ctx_t *ctx);
void       ppp_multilink_configure(ppp_ctx_t *ctx, const char *group);
bool       ppp_multilink_is_member(const ppp_ctx_t *ctx);
bool       ppp_multilink_is_owner(const ppp_ctx_t *ctx);
void       ppp_timer_tick(ppp_ctx_t *ctx);
void       ppp_rx_byte(ppp_ctx_t *ctx, uint8_t byte);
void       ppp_wrap_ip(ppp_ctx_t *ctx, const uint8_t *ip_pkt, int len);

/* PPP frame sending (used by sub-protocols) */
void       ppp_send_frame(ppp_ctx_t *ctx, uint16_t protocol, const uint8_t *data, int len);
void       ppp_advance_state(ppp_ctx_t *ctx);
bool       ppp_random_bytes(uint8_t *buffer, uint8_t len);
void       ppp_ccp_start(ppp_ctx_t *ctx);
void       ppp_ccp_process(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len);
void       ppp_ccp_fallback_plaintext(ppp_ctx_t *ctx);
const char *ppp_ccp_method_name(uint8_t method);
bool       ppp_ccp_codec_set(ppp_ctx_t *ctx, bool transmit, uint8_t method);
bool       ppp_ccp_codec_set_window(ppp_ctx_t *ctx, bool transmit, uint8_t method,
                                    uint32_t window_size);
bool       ppp_ccp_codec_set_lzs(ppp_ctx_t *ctx, bool transmit, uint8_t method,
                                 uint16_t history_count, uint8_t check_mode);
void       ppp_ccp_codec_close(ppp_ctx_t *ctx);
bool       ppp_ccp_codec_compress(ppp_ctx_t *ctx, const uint8_t *input, int input_len,
                                  uint8_t *output, int output_capacity, int *output_len);
bool       ppp_ccp_codec_decompress(ppp_ctx_t *ctx, const uint8_t *input, int input_len,
                                    uint8_t *output, int output_capacity, int *output_len);
void       ppp_ccp_codec_flush(ppp_ctx_t *ctx, bool transmit);
bool       ppp_ccp_codec_last_frame_flushed(ppp_ctx_t *ctx, bool transmit);

/* FCS-16 CRC */
uint16_t   ppp_fcs16(const uint8_t *data, int len);

#ifdef __cplusplus
}
#endif

#endif /* MODEM_PPP_H */
