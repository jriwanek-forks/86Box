/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          PPP (Point-to-Point Protocol) implementation for modem emulation.
 *          HDLC-like and NT31 RAS framing, FCS-16 (RFC 1662), LCP (RFC 1661), PAP,
 *          CHAP, EAP, IPCP, CCP codecs (Stac LZS, MPPC, Deflate, BSD-Compress, Predictor),
 *          MPPE, and Van Jacobson compression.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <86box/net_modem_ppp.h>
#include <86box/net_modem_mppe.h>
#include <86box/net_modem_pap.h>
#include <86box/net_modem_chap.h>
#include <86box/net_modem_eap.h>
#include <86box/net_modem_ipcp.h>
#include <86box/net_modem_cslip.h>
#include <86box/net_modem_mppp.h>
#include <86box/log.h>
#include "net_modem_debug.h"
#ifdef _WIN32
#    include <windows.h>
#    include <bcrypt.h>
#endif

#define PPP_LCP_TIMEOUT_MS 3000
#define PPP_LCP_MAX_RETRIES 10
#define PPP_AUTH_TIMEOUT_MS 30000

struct ppp_mppp_bundle_t {
    char                 group[64];
    ppp_auth_type_t      auth_type;
    char                 username[64];
    uint8_t              peer_endpoint[32];
    uint8_t              peer_endpoint_length;
    uint16_t             rx_mrru;
    uint16_t             tx_mrru;
    bool                 rx_short_sequence;
    bool                 tx_short_sequence;
    ppp_ctx_t           *owner;
    ppp_ctx_t           *links[PPP_MPPP_MAX_LINKS];
    size_t               link_count;
    ppp_mppp_sender_t    sender;
    ppp_mppp_reassembler_t reassembler;
    struct ppp_mppp_bundle_t *next;
};

static struct ppp_mppp_bundle_t *ppp_mppp_bundles;
static void ppp_process_frame(ppp_ctx_t *ctx, const uint8_t *frame, int frame_len,
                              bool reassembled);

#ifdef ENABLE_MODEM_LOG
extern uint8_t modem_do_log;

static void
ppp_log(void *priv, const char *fmt, ...)
{
    va_list ap;
    if (modem_do_log) {
        va_start(ap, fmt);
        log_out(priv, fmt, ap);
        va_end(ap);
    }
}
#else
#    define ppp_log(priv, fmt, ...)
#endif

static bool
ppp_mppp_send_fragment(void *opaque, const uint8_t *fragment, size_t length)
{
    ppp_ctx_t *ctx = (ppp_ctx_t *) opaque;

    if (!ctx || ctx->state == PPP_STATE_DEAD || length > PPP_MAX_FRAME - 6)
        return false;

    ppp_send_frame(ctx, PPP_PROTO_MULTILINK, fragment, (int) length);
    return true;
}

static void
ppp_mppp_deliver_packet(void *opaque, const uint8_t *packet, size_t length)
{
    struct ppp_mppp_bundle_t *bundle = (struct ppp_mppp_bundle_t *) opaque;
    uint8_t frame[PPP_MAX_FRAME + 2];

    if (!bundle || !bundle->owner || length < 1 || length > sizeof(frame) - 2)
        return;
    if ((packet[0] & 1) ? !bundle->owner->our_pfc : length < 2)
        return;

    frame[0] = PPP_ADDRESS;
    frame[1] = PPP_CONTROL;
    memcpy(frame + 2, packet, length);
    ppp_process_frame(bundle->owner, frame, (int) length + 2, true);
}

static void
ppp_mppp_bundle_free(struct ppp_mppp_bundle_t *bundle)
{
    if (!bundle)
        return;

    ppp_mppp_reassembler_close(&bundle->reassembler);
    free(bundle);
}

static bool
ppp_multilink_join(ppp_ctx_t *ctx)
{
    struct ppp_mppp_bundle_t *bundle = ppp_mppp_bundles;
    size_t header_length;
    size_t max_fragment_payload;
    bool new_bundle = false;

    if (!ctx || !ctx->multilink_group[0]
        || !ctx->multilink_our_mrru || !ctx->multilink_peer_mrru)
        return false;

    header_length = ppp_mppp_header_size(ctx->multilink_peer_short_sequence);
    if (ctx->peer_mru <= header_length + 2) {
        ppp_log(ctx->log, "PPP: Physical MRU is too small for Multilink headers\n");
        ctx->state = PPP_STATE_DEAD;
        return false;
    }
    max_fragment_payload = ctx->peer_mru - header_length - 2;
    if (max_fragment_payload > PPP_MAX_FRAME - 6 - header_length)
        max_fragment_payload = PPP_MAX_FRAME - 6 - header_length;

    while (bundle && strcmp(bundle->group, ctx->multilink_group) != 0)
        bundle = bundle->next;

    if (bundle) {
        if (bundle->link_count >= PPP_MPPP_MAX_LINKS
            || bundle->auth_type != ctx->auth_type
            || strcmp(bundle->username, ctx->username) != 0
            || bundle->peer_endpoint_length != ctx->multilink_peer_endpoint_length
            || memcmp(bundle->peer_endpoint, ctx->multilink_peer_endpoint_data,
                      bundle->peer_endpoint_length) != 0
            || bundle->rx_mrru != ctx->multilink_our_mrru_value
            || bundle->tx_mrru != ctx->multilink_peer_mrru_value
            || bundle->rx_short_sequence != ctx->multilink_our_short_sequence
            || bundle->tx_short_sequence != ctx->multilink_peer_short_sequence) {
            ppp_log(ctx->log, "PPP: Multilink link does not match the active bundle\n");
            ctx->state = PPP_STATE_DEAD;
            return false;
        }
    } else {
        bundle = (struct ppp_mppp_bundle_t *) calloc(1, sizeof(*bundle));
        if (!bundle) {
            ppp_log(ctx->log, "PPP: Could not allocate Multilink bundle\n");
            ctx->state = PPP_STATE_DEAD;
            return false;
        }
        new_bundle = true;

        memcpy(bundle->group, ctx->multilink_group, sizeof(bundle->group));
        bundle->auth_type = ctx->auth_type;
        memcpy(bundle->username, ctx->username, sizeof(bundle->username));
        bundle->peer_endpoint_length = ctx->multilink_peer_endpoint_length;
        memcpy(bundle->peer_endpoint, ctx->multilink_peer_endpoint_data,
               bundle->peer_endpoint_length);
        bundle->rx_mrru = ctx->multilink_our_mrru_value;
        bundle->tx_mrru = ctx->multilink_peer_mrru_value;
        bundle->rx_short_sequence = ctx->multilink_our_short_sequence;
        bundle->tx_short_sequence = ctx->multilink_peer_short_sequence;

        if (!ppp_mppp_sender_init(&bundle->sender, bundle->tx_mrru,
                                  bundle->tx_short_sequence)
            || !ppp_mppp_reassembler_init(&bundle->reassembler, bundle->rx_mrru,
                                         bundle->rx_short_sequence)) {
            ppp_mppp_bundle_free(bundle);
            ctx->state = PPP_STATE_DEAD;
            return false;
        }

        bundle->owner = ctx;
    }

    if (!ppp_mppp_sender_add_link(&bundle->sender, ppp_mppp_send_fragment, ctx,
                                  max_fragment_payload)) {
        ppp_log(ctx->log, "PPP: Could not attach physical link to Multilink bundle\n");
        if (new_bundle)
            ppp_mppp_bundle_free(bundle);
        ctx->state = PPP_STATE_DEAD;
        return false;
    }

    bundle->links[bundle->link_count++] = ctx;
    ctx->multilink_bundle = bundle;
    if (new_bundle) {
        bundle->next = ppp_mppp_bundles;
        ppp_mppp_bundles = bundle;
    }
    ppp_log(ctx->log, "PPP: Joined Multilink bundle '%s' with %u link(s)\n",
            bundle->group, (unsigned) bundle->link_count);
    return true;
}

static void
ppp_multilink_detach(ppp_ctx_t *ctx)
{
    struct ppp_mppp_bundle_t *bundle;
    struct ppp_mppp_bundle_t **bundle_pos;

    if (!ctx || !ctx->multilink_bundle)
        return;

    bundle = ctx->multilink_bundle;
    ctx->multilink_bundle = NULL;
    ppp_mppp_sender_remove_link(&bundle->sender, ctx);

    for (size_t index = 0; index < bundle->link_count; index++) {
        if (bundle->links[index] != ctx)
            continue;
        bundle->link_count--;
        if (index != bundle->link_count)
            bundle->links[index] = bundle->links[bundle->link_count];
        bundle->links[bundle->link_count] = NULL;
        break;
    }

    if (bundle->owner == ctx) {
        for (size_t index = 0; index < bundle->link_count; index++) {
            bundle->links[index]->multilink_bundle = NULL;
            bundle->links[index]->state = PPP_STATE_DEAD;
        }
        bundle->link_count = 0;
    }

    if (bundle->link_count != 0) {
        ppp_mppp_reassembler_reset(&bundle->reassembler);
        return;
    }

    for (bundle_pos = &ppp_mppp_bundles; *bundle_pos && *bundle_pos != bundle;
         bundle_pos = &(*bundle_pos)->next)
        ;
    if (*bundle_pos)
        *bundle_pos = bundle->next;
    ppp_mppp_bundle_free(bundle);
}

void
ppp_multilink_configure(ppp_ctx_t *ctx, const char *group)
{
    if (!ctx)
        return;

    ctx->multilink_group[0] = '\0';
    if (group && group[0]) {
        strncpy(ctx->multilink_group, group, sizeof(ctx->multilink_group) - 1);
        ctx->multilink_group[sizeof(ctx->multilink_group) - 1] = '\0';
    }
}

bool
ppp_multilink_is_member(const ppp_ctx_t *ctx)
{
    return ctx && ctx->multilink_bundle;
}

bool
ppp_multilink_is_owner(const ppp_ctx_t *ctx)
{
    return ctx && (!ctx->multilink_bundle || ctx->multilink_bundle->owner == ctx);
}

#ifdef ENABLE_MODEM_LOG
static const char *
ppp_auth_name(ppp_auth_type_t auth_type)
{
    switch (auth_type) {
        case PPP_AUTH_NONE:       return "none";
        case PPP_AUTH_PAP:        return "PAP";
        case PPP_AUTH_CHAP_MD5:   return "CHAP-MD5";
        case PPP_AUTH_MSCHAP:     return "MS-CHAP";
        case PPP_AUTH_MSCHAPV2:   return "MS-CHAPv2";
        case PPP_AUTH_CHAP_SHA1:  return "CHAP-SHA1";
        case PPP_AUTH_CHAP_SHA256: return "CHAP-SHA256";
        case PPP_AUTH_CHAP_SHA384: return "CHAP-SHA384";
        case PPP_AUTH_CHAP_SHA512: return "CHAP-SHA512";
        case PPP_AUTH_EAP:        return "EAP-MD5";
        default:                  return "unknown";
    }
}

static const char *
ppp_protocol_name(uint16_t protocol)
{
    switch (protocol) {
        case PPP_PROTO_IP:             return "IPv4";
        case PPP_PROTO_IPCP:           return "IPCP";
        case PPP_PROTO_LCP:            return "LCP";
        case PPP_PROTO_PAP:            return "PAP";
        case PPP_PROTO_CHAP:           return "CHAP";
        case PPP_PROTO_EAP:            return "EAP";
        case PPP_PROTO_CCP:            return "CCP";
        case PPP_PROTO_MPPE:           return "MPPE";
        case PPP_PROTO_MULTILINK:      return "Multilink";
        case PPP_PROTO_VJ_COMPRESSED:  return "VJ-compressed IPv4";
        case PPP_PROTO_VJ_UNCOMPRESSED:return "VJ-uncompressed IPv4";
        case PPP_AUTH_PROTO_SHIVA_PAP: return "Shiva PAP";
        case PPP_AUTH_PROTO_RSA:       return "RSA Authentication";
        case PPP_AUTH_PROTO_MITSUBISHI_SIEP: return "Mitsubishi SIEP";
        case PPP_AUTH_PROTO_VSAP:      return "Vendor-Specific Authentication";
        case PPP_AUTH_PROTO_PROPRIETARY_C281: return "Proprietary Authentication (0xC281)";
        case PPP_AUTH_PROTO_PROPRIETARY_C283: return "Proprietary Authentication (0xC283)";
        case PPP_AUTH_PROTO_PROPRIETARY_NODE_ID: return "Proprietary Node ID Authentication";
        default:                       return "unknown";
    }
}

#if defined(ENABLE_MODEM_LOG) && defined(ENABLE_MODEM_DEBUG)
static const char *
ppp_chap_algorithm_name(uint8_t algorithm)
{
    switch (algorithm) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:                 return "Reserved";
        case CHAP_ALG_MD5:      return "CHAP with MD5";
        case CHAP_ALG_SHA1:     return "CHAP with SHA-1";
        case CHAP_ALG_SHA256:   return "CHAP with SHA-256";
        case CHAP_ALG_SHA3_256: return "CHAP with SHA3-256";
        case CHAP_ALG_SHA384:   return "CHAP with SHA-384";
        case CHAP_ALG_SHA3_384: return "CHAP with SHA3-384";
        case CHAP_ALG_SHA512:   return "CHAP with SHA-512";
        case CHAP_ALG_SHA3_512: return "CHAP with SHA3-512";
        case CHAP_ALG_MSCHAP:   return "MS-CHAP";
        case CHAP_ALG_MSCHAPV2: return "MS-CHAP-2";
        default:                return "Unassigned or unsupported";
    }
}
#endif

static const char *
ppp_state_name(ppp_state_t state)
{
    switch (state) {
        case PPP_STATE_DEAD:          return "dead";
        case PPP_STATE_LCP_NEGOTIATE: return "LCP-negotiation";
        case PPP_STATE_AUTH:          return "authentication";
        case PPP_STATE_IPCP_NEGOTIATE:return "IPCP-negotiation";
        case PPP_STATE_NETWORK:       return "network";
        case PPP_STATE_TERMINATING:   return "terminating";
        default:                      return "unknown";
    }
}
#endif

#if defined(ENABLE_MODEM_LOG) && defined(ENABLE_MODEM_DEBUG)
static const char *
ppp_lcp_code_name(uint8_t code)
{
    switch (code) {
        case PPP_CODE_CONFIGURE_REQUEST: return "Configure-Request";
        case PPP_CODE_CONFIGURE_ACK:     return "Configure-Ack";
        case PPP_CODE_CONFIGURE_NAK:     return "Configure-Nak";
        case PPP_CODE_CONFIGURE_REJECT:  return "Configure-Reject";
        case PPP_CODE_TERMINATE_REQUEST: return "Terminate-Request";
        case PPP_CODE_TERMINATE_ACK:     return "Terminate-Ack";
        case PPP_CODE_CODE_REJECT:       return "Code-Reject";
        case PPP_CODE_PROTOCOL_REJECT:   return "Protocol-Reject";
        case PPP_CODE_ECHO_REQUEST:      return "Echo-Request";
        case PPP_CODE_ECHO_REPLY:        return "Echo-Reply";
        case PPP_CODE_DISCARD_REQUEST:   return "Discard-Request";
        default:                         return "unknown";
    }
}

static const char *
ppp_lcp_option_name(uint8_t option)
{
    switch (option) {
        case LCP_OPT_MRU:            return "MRU";
        case LCP_OPT_ACCM:           return "ACCM";
        case LCP_OPT_AUTH_PROTO:     return "Authentication-Protocol";
        case LCP_OPT_QUALITY_PROTO:  return "Quality-Protocol";
        case LCP_OPT_MAGIC_NUMBER:   return "Magic-Number";
        case LCP_OPT_PFC:            return "Protocol-Field-Compression";
        case LCP_OPT_ACFC:           return "Address-Control-Field-Compression";
        case LCP_OPT_CALLBACK:       return "Callback";
        case LCP_OPT_MRRU:           return "MRRU";
        case LCP_OPT_SHORT_SEQUENCE: return "Short-Sequence-Number-Header";
        case LCP_OPT_ENDPOINT_DISC:  return "Endpoint-Discriminator";
        default:                     return "unknown";
    }
}
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

bool
ppp_random_bytes(uint8_t *buffer, uint8_t len)
{
#ifdef _WIN32
    return BCryptGenRandom(NULL, buffer, (ULONG) len, BCRYPT_USE_SYSTEM_PREFERRED_RNG) == 0;
#else
    FILE   *random_source = fopen("/dev/urandom", "rb");
    size_t  remaining     = len;

    if (!random_source)
        return false;

    while (remaining > 0) {
        size_t bytes_read = fread(buffer, 1, remaining, random_source);
        if (bytes_read == 0) {
            fclose(random_source);
            return false;
        }
        buffer += bytes_read;
        remaining -= bytes_read;
    }

    return fclose(random_source) == 0;
#endif
}

static uint16_t
ppp_ras_crc16(const uint8_t *data, int length)
{
    uint16_t crc = 0;
    for (int position = 0; position < length; position++) {
        crc ^= (uint16_t) data[position] << 8;
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000) ? (uint16_t) ((crc << 1) ^ 0x1021)
                                 : (uint16_t) (crc << 1);
    }
    return crc;
}

static void
ppp_ras_send_flush(ppp_ctx_t *ctx)
{
    uint8_t frame[10] = {
        PPP_RAS_SYN,
        PPP_RAS_SOH_DEST | PPP_RAS_SOH_TYPE | PPP_RAS_SOH_COMPRESS,
        0, 3,
        (uint8_t) (PPP_RAS_IP_TYPE >> 8), (uint8_t) PPP_RAS_IP_TYPE,
        0xFF, PPP_RAS_ETX, 0, 0
    };
    if (!ctx || !ctx->serial_push)
        return;
    uint16_t crc = ppp_ras_crc16(frame + 1, sizeof(frame) - 3);
    frame[8] = (uint8_t) crc;
    frame[9] = (uint8_t) (crc >> 8);
    ctx->serial_push(ctx->modem, frame, sizeof(frame));
}

static bool
ppp_ras_send_ip(ppp_ctx_t *ctx, const uint8_t *ip_packet, int ip_length)
{
    uint8_t compressed[PPP_MAX_FRAME + 256];
    uint8_t frame[PPP_MAX_FRAME + 272];
    int compressed_length;
    int body_length;
    int frame_length;
    uint8_t ticket;

    if (!ctx || !ip_packet || ip_length <= 0 || ip_length > PPP_MAX_FRAME
        || ip_length > ctx->peer_mru)
        return false;
    if (ctx->ras_tx_flush_pending) {
        ppp_ccp_codec_flush(ctx, true);
        ctx->ras_tx_flush_pending = false;
    }
    if (!ppp_ccp_codec_compress(ctx, ip_packet, ip_length, compressed,
                                sizeof(compressed), &compressed_length))
        return false;

    if (ppp_ccp_codec_last_frame_flushed(ctx, true)) {
        ctx->ras_tx_ticket_base = ctx->ras_tx_ticket_base >= 96
                                ? 0 : (uint8_t) (ctx->ras_tx_ticket_base + 16);
        ticket = ctx->ras_tx_ticket_base;
        ctx->ras_tx_ticket_next = (uint8_t) (ticket + 1);
    } else {
        ticket = ctx->ras_tx_ticket_next;
        ctx->ras_tx_ticket_next++;
        if ((ctx->ras_tx_ticket_next & 0x0F) == 0)
            ctx->ras_tx_ticket_next = (uint8_t) (ctx->ras_tx_ticket_base + 1);
    }

    body_length = compressed_length + 3;
    frame[0] = PPP_RAS_SYN;
    frame[1] = PPP_RAS_SOH_DEST | PPP_RAS_SOH_TYPE | PPP_RAS_SOH_COMPRESS;
    frame[2] = (uint8_t) (body_length >> 8);
    frame[3] = (uint8_t) body_length;
    frame[4] = (uint8_t) (PPP_RAS_IP_TYPE >> 8);
    frame[5] = (uint8_t) PPP_RAS_IP_TYPE;
    frame[6] = ticket;
    memcpy(frame + 7, compressed, (size_t) compressed_length);
    frame[7 + compressed_length] = PPP_RAS_ETX;
    frame_length = compressed_length + 10;
    uint16_t crc = ppp_ras_crc16(frame + 1, frame_length - 3);
    frame[frame_length - 2] = (uint8_t) crc;
    frame[frame_length - 1] = (uint8_t) (crc >> 8);
    ctx->serial_push(ctx->modem, frame, frame_length);
    return true;
}

static void
ppp_process_ras_frame(ppp_ctx_t *ctx, const uint8_t *frame, int frame_length)
{
    if (!ctx || !frame || frame_length < 10
        || frame[0] != PPP_RAS_SYN
           || frame[1] != (PPP_RAS_SOH_DEST | PPP_RAS_SOH_TYPE | PPP_RAS_SOH_COMPRESS))
        return;

    int body_length = (frame[2] << 8) | frame[3];
    if (body_length < 3 || body_length + 7 != frame_length
        || frame[frame_length - 3] != PPP_RAS_ETX)
        return;
    uint16_t expected_crc = ppp_ras_crc16(frame + 1, frame_length - 3);
    uint16_t received_crc = (uint16_t) frame[frame_length - 2]
                          | ((uint16_t) frame[frame_length - 1] << 8);
    if (expected_crc != received_crc
        || (((uint16_t) frame[4] << 8) | frame[5]) != PPP_RAS_IP_TYPE)
        return;

    uint8_t ticket = frame[6];
    if (ticket & 0x80) {
        ctx->ras_tx_flush_pending = true;
        if (ticket == 0xFF)
            return;
        ticket ^= 0x80;
    }

    const uint8_t *payload = frame + 7;
    int payload_length = frame_length - 10;
    if (ticket == 0x7E) {
        if (ctx->ccp_open && ctx->ccp_rx_method == PPP_CCP_METHOD_NT31RAS
            && ctx->state == PPP_STATE_NETWORK && payload_length > 0
            && payload_length <= ctx->our_mru)
            ctx->network_send_ip(ctx->modem, payload, payload_length);
        return;
    }
    if (ticket >= 0x7E)
        return;

    if ((ticket & 0x0F) == 0) {
        ctx->ras_rx_ticket_base = ticket;
        ctx->ras_rx_ticket_next = (uint8_t) (ticket + 1);
        ppp_ccp_codec_flush(ctx, false);
    } else if (ticket != ctx->ras_rx_ticket_next) {
        ctx->ras_rx_ticket_next = 0;
        ppp_ccp_codec_flush(ctx, false);
        ppp_ras_send_flush(ctx);
        return;
    } else {
        ctx->ras_rx_ticket_next++;
        if ((ctx->ras_rx_ticket_next & 0x0F) == 0)
            ctx->ras_rx_ticket_next = (uint8_t) (ctx->ras_rx_ticket_base + 1);
    }

    if (!ctx->ccp_open || ctx->ccp_rx_method != PPP_CCP_METHOD_NT31RAS)
        return;

    uint8_t ip_packet[PPP_MAX_FRAME];
    int ip_length;
    if (!ppp_ccp_codec_decompress(ctx, payload, payload_length,
                                  ip_packet, sizeof(ip_packet), &ip_length)) {
        ppp_ras_send_flush(ctx);
        return;
    }
    if (ctx->state == PPP_STATE_NETWORK && ip_length > 0 && ip_length <= ctx->our_mru)
        ctx->network_send_ip(ctx->modem, ip_packet, ip_length);
}

/* Send raw HDLC-framed data to the serial line */
void
ppp_send_frame(ppp_ctx_t *ctx, uint16_t protocol, const uint8_t *data, int len)
{
    uint8_t  frame[((PPP_MAX_FRAME + 264) * 2) + 2];
    uint8_t  raw[PPP_MAX_FRAME + 264];
    uint8_t  encrypted[PPP_MAX_FRAME * 2];
    uint8_t  plaintext[PPP_MAX_FRAME];
    int      raw_len = 0;
    int      out_len = 0;
    uint16_t fcs;
    size_t   encrypted_len = 0;
    uint16_t wire_protocol = protocol;
    const uint8_t *wire_data = data;
    int      wire_len = len;
    bool     compress_ac = ctx->state >= PPP_STATE_AUTH
                        && ctx->peer_acfc && protocol != PPP_PROTO_LCP;
    bool     compress_protocol = ctx->state >= PPP_STATE_AUTH
                              && ctx->peer_pfc && (protocol & 0xFF00) == 0
                              && (protocol & 1) != 0;
    bool     multilink_frame = protocol == PPP_PROTO_MULTILINK;

    if (len < 0 || len > PPP_MAX_FRAME - 6) {
        ppp_log(ctx->log, "PPP: Dropping oversized frame (%d bytes)\n", len);
        return;
    }

    if (ctx->mppe_keys_ready
        && ((!ctx->ccp_open && !ctx->ccp_plaintext_fallback)
            || ctx->state != PPP_STATE_NETWORK)
        && protocol >= PPP_PROTO_IP && protocol <= 0x00FA && !multilink_frame)
        return;

    if (!multilink_frame && ctx->mppe_tx_enabled
        && protocol >= PPP_PROTO_IP && protocol <= 0x00FA) {
        if (len > PPP_MAX_FRAME - 10) {
            ppp_log(ctx->log, "PPP: Dropping oversized MPPE frame (%d bytes)\n", len);
            return;
        }
        plaintext[0] = (uint8_t) (protocol >> 8);
        plaintext[1] = (uint8_t) protocol;
        if (len > 0) {
            if (!data)
                return;
            memcpy(plaintext + 2, data, (size_t) len);
        }
        if (!ppp_mppe_encrypt(&ctx->mppe_tx, plaintext, (size_t) len + 2,
                              encrypted, sizeof(encrypted), &encrypted_len))
            return;
        wire_protocol = PPP_PROTO_MPPE;
        wire_data = encrypted;
        wire_len = (int) encrypted_len;
        compress_protocol = ctx->state >= PPP_STATE_AUTH && ctx->peer_pfc;
    } else if (!multilink_frame && ctx->ccp_open
               && ctx->ccp_tx_method == PPP_CCP_METHOD_NT31RAS
               && protocol == PPP_PROTO_IP) {
        (void) ppp_ras_send_ip(ctx, data, len);
        return;
    } else if (!multilink_frame && ctx->ccp_open
               && (ctx->ccp_tx_method == PPP_CCP_METHOD_PREDICTOR1
                || ctx->ccp_tx_method == PPP_CCP_METHOD_PREDICTOR2
                || ctx->ccp_tx_method == PPP_CCP_METHOD_LZS
                || ctx->ccp_tx_method == PPP_CCP_METHOD_DEFLATE
                || ctx->ccp_tx_method == PPP_CCP_METHOD_MPPC
                || ctx->ccp_tx_method == PPP_CCP_METHOD_BSD)
               && protocol >= PPP_PROTO_IP && protocol <= 0x00FA) {
        int compressed_len;
        int plaintext_len;
        if ((ctx->ccp_tx_method == PPP_CCP_METHOD_BSD
             || ctx->ccp_tx_method == PPP_CCP_METHOD_DEFLATE)
            && protocol < 0x0100) {
            plaintext[0] = (uint8_t) protocol;
            plaintext_len = len + 1;
            if (len > 0) {
                if (!data)
                    return;
                memcpy(plaintext + 1, data, (size_t) len);
            }
        } else {
            plaintext[0] = (uint8_t) (protocol >> 8);
            plaintext[1] = (uint8_t) protocol;
            plaintext_len = len + 2;
            if (len > 0) {
                if (!data)
                    return;
                memcpy(plaintext + 2, data, (size_t) len);
            }
        }
        if (!ppp_ccp_codec_compress(ctx, plaintext, plaintext_len, encrypted,
                                    sizeof(encrypted), &compressed_len))
            return;
        if (ctx->ccp_tx_method == PPP_CCP_METHOD_LZS
            && compressed_len + (ctx->peer_pfc ? 1 : 2)
               >= len + (compress_protocol ? 1 : 2)) {
            wire_protocol = protocol;
            wire_data = data;
            wire_len = len;
        } else {
            wire_protocol = PPP_PROTO_MPPE;
            wire_data = encrypted;
            wire_len = compressed_len;
            compress_protocol = ctx->state >= PPP_STATE_AUTH && ctx->peer_pfc;
        }
    }

    if (!multilink_frame && ctx->multilink_bundle
        && ctx->multilink_bundle->owner == ctx
        && protocol >= PPP_PROTO_IP && protocol <= 0x00FA) {
        if (wire_len < 0
            || !ppp_mppp_sender_send(&ctx->multilink_bundle->sender, wire_protocol,
                                     wire_data, (size_t) wire_len))
            ppp_log(ctx->log, "PPP: Could not send packet over Multilink bundle\n");
        return;
    }

    if (wire_len + (compress_protocol ? 1 : 2) > ctx->peer_mru) {
        ppp_log(ctx->log, "PPP: Dropping frame larger than peer MRU\n");
        return;
    }

    /* Build unescaped frame with only negotiated header compression. */
    if (!compress_ac) {
        raw[raw_len++] = PPP_ADDRESS;
        raw[raw_len++] = PPP_CONTROL;
    }
    if (compress_protocol) {
        raw[raw_len++] = (uint8_t) wire_protocol;
    } else {
        raw[raw_len++] = (uint8_t) (wire_protocol >> 8);
        raw[raw_len++] = (uint8_t) (wire_protocol & 0xFF);
    }
    if (wire_len > 0) {
        if (!wire_data)
            return;
        memcpy(raw + raw_len, wire_data, (size_t) wire_len);
        raw_len += wire_len;
    }

    /* Calculate FCS over Address + Control + Protocol + Data */
    fcs = ppp_fcs16(raw, raw_len);
    raw[raw_len++] = (uint8_t) (fcs & 0xFF);
    raw[raw_len++] = (uint8_t) (fcs >> 8);

    /* HDLC escape and frame */
    frame[out_len++] = PPP_FLAG;
    for (int i = 0; i < raw_len; i++) {
        uint8_t c = raw[i];
        if (c == PPP_FLAG || c == PPP_ESCAPE || (c < 0x20 && (ctx->peer_accm & (1u << c)))) {
            frame[out_len++] = PPP_ESCAPE;
            frame[out_len++] = c ^ 0x20;
        } else {
            frame[out_len++] = c;
        }
    }
    frame[out_len++] = PPP_FLAG;

    MODEM_DEBUG_LOG(ctx->log, "PPP: TX frame protocol=%s (0x%04X) payload=%d "
                    "wire-protocol=%s (0x%04X) wire-payload=%d framed=%d "
                    "transform=%s ccp-open=%u ccp-method=%s MPPE-tx=%u "
                    "acfc=%u pfc=%u\n",
                    ppp_protocol_name(protocol), (unsigned) protocol, len,
                    ppp_protocol_name(wire_protocol), (unsigned) wire_protocol,
                    wire_len, out_len,
                    wire_protocol == PPP_PROTO_MPPE
                        ? (ctx->mppe_tx_enabled ? "MPPE"
                                                : ppp_ccp_method_name(ctx->ccp_tx_method))
                        : "none",
                    (unsigned) ctx->ccp_open,
                    ppp_ccp_method_name(ctx->ccp_tx_method),
                    (unsigned) ctx->mppe_tx_enabled,
                    (unsigned) compress_ac, (unsigned) compress_protocol);
    ctx->serial_push(ctx->modem, frame, out_len);
}

static uint32_t
ppp_multilink_endpoint_id(const ppp_ctx_t *ctx)
{
    uint32_t hash = 2166136261u;

    for (const char *pos = ctx->multilink_group; *pos; pos++) {
        hash ^= (uint8_t) *pos;
        hash *= 16777619u;
    }

    return hash;
}

/* Build and send an LCP Configuration-Request */
static void
ppp_send_lcp_config_request(ppp_ctx_t *ctx)
{
    uint8_t pkt[64];
    int     len = 0;

    ctx->lcp_ack_received = false;

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

    if (ctx->request_pfc) {
        pkt[len++] = LCP_OPT_PFC;
        pkt[len++] = 2;
    }

    if (ctx->request_acfc) {
        pkt[len++] = LCP_OPT_ACFC;
        pkt[len++] = 2;
    }

    /* Option: Auth Protocol (type=3) - if we want authentication */
    if (ctx->auth_type != PPP_AUTH_NONE) {
        pkt[len++] = LCP_OPT_AUTH_PROTO;
        if (ctx->auth_type == PPP_AUTH_PAP || ctx->auth_type == PPP_AUTH_EAP) {
            uint16_t auth_proto = ctx->auth_type == PPP_AUTH_PAP
                                ? PPP_AUTH_PROTO_PAP : PPP_AUTH_PROTO_EAP;

            pkt[len++] = 4;
            pkt[len++] = (uint8_t) (auth_proto >> 8);
            pkt[len++] = (uint8_t) (auth_proto & 0xFF);
        } else {
            /* CHAP variants */
            pkt[len++] = 5;
            pkt[len++] = (uint8_t) (PPP_AUTH_PROTO_CHAP >> 8);
            pkt[len++] = (uint8_t) (PPP_AUTH_PROTO_CHAP & 0xFF);
            switch (ctx->auth_type) {
                case PPP_AUTH_CHAP_MD5:  pkt[len++] = CHAP_ALG_MD5;      break;
                case PPP_AUTH_CHAP_SHA1: pkt[len++] = CHAP_ALG_SHA1;     break;
                case PPP_AUTH_CHAP_SHA256: pkt[len++] = CHAP_ALG_SHA256; break;
                case PPP_AUTH_CHAP_SHA384: pkt[len++] = CHAP_ALG_SHA384; break;
                case PPP_AUTH_CHAP_SHA512: pkt[len++] = CHAP_ALG_SHA512; break;
                case PPP_AUTH_MSCHAP:    pkt[len++] = CHAP_ALG_MSCHAP;   break;
                case PPP_AUTH_MSCHAPV2:  pkt[len++] = CHAP_ALG_MSCHAPV2; break;
                default:                 pkt[len++] = CHAP_ALG_MD5;      break;
            }
        }
    }

    if (ctx->multilink_request_mrru) {
        pkt[len++] = LCP_OPT_MRRU;
        pkt[len++] = 4;
        pkt[len++] = (uint8_t) (ctx->multilink_our_mrru_value >> 8);
        pkt[len++] = (uint8_t) ctx->multilink_our_mrru_value;
    }

    if (ctx->multilink_request_short_sequence) {
        pkt[len++] = LCP_OPT_SHORT_SEQUENCE;
        pkt[len++] = 2;
    }

    if (ctx->multilink_request_endpoint) {
        uint32_t endpoint_id = ppp_multilink_endpoint_id(ctx);
        pkt[len++] = LCP_OPT_ENDPOINT_DISC;
        pkt[len++] = 7;
        pkt[len++] = 0;
        pkt[len++] = (uint8_t) (endpoint_id >> 24);
        pkt[len++] = (uint8_t) (endpoint_id >> 16);
        pkt[len++] = (uint8_t) (endpoint_id >> 8);
        pkt[len++] = (uint8_t) endpoint_id;
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

    memcpy(ctx->lcp_request, pkt, len);
    ctx->lcp_request_len = (uint8_t) len;
    ppp_send_frame(ctx, PPP_PROTO_LCP, pkt, len);
    ctx->lcp_req_sent = true;
    ctx->lcp_timeout_ms = 0;
    ppp_log(ctx->log, "PPP: Sent LCP Configure-Request (id=%d)\n", pkt[1]);
#if defined(ENABLE_MODEM_LOG) && defined(ENABLE_MODEM_DEBUG)
        uint16_t auth_protocol = 0;
        if (ctx->auth_type == PPP_AUTH_PAP)
            auth_protocol = PPP_AUTH_PROTO_PAP;
        else if (ctx->auth_type == PPP_AUTH_EAP)
            auth_protocol = PPP_AUTH_PROTO_EAP;
        else if (ctx->auth_type >= PPP_AUTH_CHAP_MD5 && ctx->auth_type <= PPP_AUTH_CHAP_SHA512)
            auth_protocol = PPP_AUTH_PROTO_CHAP;

        MODEM_DEBUG_LOG(ctx->log, "LCP: local MRU=%u ACCM=0x%08X PFC=%s ACFC=%s "
                        "authentication=%s protocol=%s (0x%04X)\n",
                        (unsigned) ctx->our_mru, ctx->our_accm,
                        ctx->request_pfc ? "on" : "off", ctx->request_acfc ? "on" : "off",
                        ppp_auth_name(ctx->auth_type),
                        auth_protocol ? ppp_protocol_name(auth_protocol) : "none",
                        (unsigned) auth_protocol);
    #endif
}

/* Handle LCP Configure-Request from peer */
static bool
ppp_handle_lcp_config_request(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    uint8_t ack[PPP_MAX_FRAME];
    uint8_t nak[PPP_MAX_FRAME];
    uint8_t rej[PPP_MAX_FRAME];
    int     ack_len = 4, nak_len = 4, rej_len = 4;
    uint8_t id      = pkt[1];
    int     total   = (pkt[2] << 8) | pkt[3];
    int     pos     = 4;
    uint16_t peer_mru = PPP_DEFAULT_MRU;
    uint32_t peer_accm = 0xFFFFFFFF;
    uint32_t peer_magic = 0;
    bool    peer_pfc = false;
    bool    peer_acfc = false;
    bool    peer_mrru_enabled = false;
    bool    peer_short_sequence = false;
    bool    peer_endpoint_enabled = false;
    uint16_t peer_mrru_value = 0;
    uint8_t peer_endpoint_data[32] = { 0 };
    uint8_t peer_endpoint_length = 0;

    if (total < 4 || total > pkt_len)
        return false;

    for (int option_pos = 4; option_pos < total;) {
        if (option_pos + 2 > total)
            return false;
        uint8_t option_len = pkt[option_pos + 1];
        if (option_len < 2 || option_pos + option_len > total)
            return false;
        option_pos += option_len;
    }

    ctx->lcp_ack_sent = false;
    ppp_log(ctx->log, "PPP: Received LCP Configure-Request (id=%d, len=%d)\n", id, total);

    while (pos < total) {
        uint8_t opt_type = pkt[pos];
        uint8_t opt_len  = pkt[pos + 1];

        MODEM_DEBUG_LOG(ctx->log, "LCP: peer option=%s (type=%u) length=%u\n",
                ppp_lcp_option_name(opt_type), (unsigned) opt_type,
                (unsigned) opt_len);
    #if defined(ENABLE_MODEM_LOG) && defined(ENABLE_MODEM_DEBUG)
        if (opt_type == LCP_OPT_AUTH_PROTO && opt_len >= 4) {
            uint16_t auth_protocol = (uint16_t) ((pkt[pos + 2] << 8) | pkt[pos + 3]);
            if (auth_protocol == PPP_AUTH_PROTO_CHAP && opt_len == 5) {
                uint8_t algorithm = pkt[pos + 4];
                MODEM_DEBUG_LOG(ctx->log,
                                "LCP: peer authentication=%s (0x%04X) algorithm=%s (%u)\n",
                                ppp_protocol_name(auth_protocol), (unsigned) auth_protocol,
                                ppp_chap_algorithm_name(algorithm), (unsigned) algorithm);
            } else {
                MODEM_DEBUG_LOG(ctx->log, "LCP: peer authentication=%s (0x%04X)\n",
                                ppp_protocol_name(auth_protocol), (unsigned) auth_protocol);
            }
        }
#endif

        switch (opt_type) {
            case LCP_OPT_MRU:
                if (opt_len == 4) {
                    uint16_t requested_mru = (uint16_t) ((pkt[pos + 2] << 8) | pkt[pos + 3]);
                    if (requested_mru < 128) {
                        nak[nak_len++] = LCP_OPT_MRU;
                        nak[nak_len++] = 4;
                        nak[nak_len++] = (uint8_t) (PPP_DEFAULT_MRU >> 8);
                        nak[nak_len++] = (uint8_t) PPP_DEFAULT_MRU;
                    } else {
                        peer_mru = requested_mru;
                        memcpy(ack + ack_len, pkt + pos, opt_len);
                        ack_len += opt_len;
                    }
                } else {
                    memcpy(rej + rej_len, pkt + pos, opt_len);
                    rej_len += opt_len;
                }
                break;

            case LCP_OPT_ACCM:
                if (opt_len == 6) {
                    peer_accm = ((uint32_t) pkt[pos + 2] << 24)
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
                    peer_magic = ((uint32_t) pkt[pos + 2] << 24)
                               | ((uint32_t) pkt[pos + 3] << 16)
                               | ((uint32_t) pkt[pos + 4] << 8)
                               | (uint32_t) pkt[pos + 5];
                    if (peer_magic == 0 || peer_magic == ctx->our_magic) {
                        uint32_t suggested_magic = ctx->our_magic + 1;
                        if (suggested_magic == 0)
                            suggested_magic = 1;
                        nak[nak_len++] = LCP_OPT_MAGIC_NUMBER;
                        nak[nak_len++] = 6;
                        nak[nak_len++] = (uint8_t) (suggested_magic >> 24);
                        nak[nak_len++] = (uint8_t) (suggested_magic >> 16);
                        nak[nak_len++] = (uint8_t) (suggested_magic >> 8);
                        nak[nak_len++] = (uint8_t) suggested_magic;
                    } else {
                        memcpy(ack + ack_len, pkt + pos, opt_len);
                        ack_len += opt_len;
                    }
                } else {
                    memcpy(rej + rej_len, pkt + pos, opt_len);
                    rej_len += opt_len;
                }
                break;

            case LCP_OPT_PFC:
                if (opt_len == 2) {
                    peer_pfc = true;
                    memcpy(ack + ack_len, pkt + pos, opt_len);
                    ack_len += opt_len;
                } else {
                    memcpy(rej + rej_len, pkt + pos, opt_len);
                    rej_len += opt_len;
                }
                break;

            case LCP_OPT_ACFC:
                if (opt_len == 2) {
                    peer_acfc = true;
                    memcpy(ack + ack_len, pkt + pos, opt_len);
                    ack_len += opt_len;
                } else {
                    memcpy(rej + rej_len, pkt + pos, opt_len);
                    rej_len += opt_len;
                }
                break;

            case LCP_OPT_AUTH_PROTO:
                /* We are the server; we don't accept the client requesting auth from us.
                   Reject this option so they don't try to authenticate us. */
                memcpy(rej + rej_len, pkt + pos, opt_len);
                rej_len += opt_len;
                break;

            case LCP_OPT_CALLBACK:
                /* Callback requires dial-back support, which this modem does not provide. */
                memcpy(rej + rej_len, pkt + pos, opt_len);
                rej_len += opt_len;
                break;

            case LCP_OPT_MRRU:
                if (ctx->multilink_group[0] && opt_len == 4) {
                    uint16_t requested_mrru = (uint16_t) ((pkt[pos + 2] << 8) | pkt[pos + 3]);
                    if (requested_mrru < 128 || requested_mrru > PPP_MAX_FRAME) {
                        uint16_t suggested_mrru = PPP_DEFAULT_MRU + 2;
                        nak[nak_len++] = LCP_OPT_MRRU;
                        nak[nak_len++] = 4;
                        nak[nak_len++] = (uint8_t) (suggested_mrru >> 8);
                        nak[nak_len++] = (uint8_t) suggested_mrru;
                    } else {
                        peer_mrru_enabled = true;
                        peer_mrru_value = requested_mrru;
                        memcpy(ack + ack_len, pkt + pos, opt_len);
                        ack_len += opt_len;
                    }
                } else {
                    memcpy(rej + rej_len, pkt + pos, opt_len);
                    rej_len += opt_len;
                }
                break;

            case LCP_OPT_SHORT_SEQUENCE:
                if (ctx->multilink_group[0] && opt_len == 2) {
                    peer_short_sequence = true;
                    memcpy(ack + ack_len, pkt + pos, opt_len);
                    ack_len += opt_len;
                } else {
                    memcpy(rej + rej_len, pkt + pos, opt_len);
                    rej_len += opt_len;
                }
                break;

            case LCP_OPT_ENDPOINT_DISC:
                if (ctx->multilink_group[0] && opt_len >= 3 && opt_len <= 34) {
                    peer_endpoint_enabled = true;
                    peer_endpoint_length = (uint8_t) (opt_len - 2);
                    memcpy(peer_endpoint_data, pkt + pos + 2, peer_endpoint_length);
                    memcpy(ack + ack_len, pkt + pos, opt_len);
                    ack_len += opt_len;
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
        /* Send Configure-Reject */
        rej[0] = PPP_CODE_CONFIGURE_REJECT;
        rej[1] = id;
        rej[2] = (uint8_t) (rej_len >> 8);
        rej[3] = (uint8_t) (rej_len & 0xFF);
        ppp_send_frame(ctx, PPP_PROTO_LCP, rej, rej_len);
        ppp_log(ctx->log, "PPP: Sent LCP Configure-Reject\n");
    } else if (nak_len > 4) {
        /* Send Configure-Nak */
        nak[0] = PPP_CODE_CONFIGURE_NAK;
        nak[1] = id;
        nak[2] = (uint8_t) (nak_len >> 8);
        nak[3] = (uint8_t) (nak_len & 0xFF);
        ppp_send_frame(ctx, PPP_PROTO_LCP, nak, nak_len);
        ppp_log(ctx->log, "PPP: Sent LCP Configure-Nak\n");
    } else {
        /* Send Configure-Ack */
        ack[0] = PPP_CODE_CONFIGURE_ACK;
        ack[1] = id;
        ack[2] = (uint8_t) (ack_len >> 8);
        ack[3] = (uint8_t) (ack_len & 0xFF);
        ppp_send_frame(ctx, PPP_PROTO_LCP, ack, ack_len);
        ctx->peer_mru = peer_mru;
        ctx->peer_accm = peer_accm;
        ctx->peer_magic = peer_magic;
        ctx->peer_pfc = peer_pfc;
        ctx->peer_acfc = peer_acfc;
         ctx->multilink_peer_mrru = peer_mrru_enabled;
         ctx->multilink_peer_mrru_value = peer_mrru_value;
         ctx->multilink_peer_short_sequence = peer_short_sequence;
         ctx->multilink_peer_endpoint = peer_endpoint_enabled;
         ctx->multilink_peer_endpoint_length = peer_endpoint_length;
         memcpy(ctx->multilink_peer_endpoint_data, peer_endpoint_data,
             sizeof(ctx->multilink_peer_endpoint_data));
        ctx->lcp_ack_sent = true;
        ppp_log(ctx->log, "PPP: Sent LCP Configure-Ack\n");
    }

    return true;
}

/* Handle LCP Configure-Ack from peer */
static void
ppp_handle_lcp_config_ack(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    int total;
    int pos = 4;

    if (pkt_len < 4 || !ctx->lcp_req_sent)
    {
        MODEM_DEBUG_LOG(ctx->log, "LCP: ignored Configure-Ack (length=%d request-pending=%u)\n",
                        pkt_len, (unsigned) ctx->lcp_req_sent);
        return;
    }

    total = (pkt[2] << 8) | pkt[3];
    if (total != pkt_len || total != ctx->lcp_request_len
        || pkt[1] != ctx->lcp_request[1]) {
        MODEM_DEBUG_LOG(ctx->log, "LCP: ignored Configure-Ack id=%u length=%d expected id=%u length=%u\n",
                        (unsigned) pkt[1], total, (unsigned) ctx->lcp_request[1],
                        (unsigned) ctx->lcp_request_len);
        return;
    }
    if (memcmp(pkt + 4, ctx->lcp_request + 4, total - 4) != 0) {
        MODEM_DEBUG_LOG(ctx->log, "LCP: ignored Configure-Ack id=%u because options differ from request\n",
                        (unsigned) pkt[1]);
        return;
    }

    ctx->our_pfc  = false;
    ctx->our_acfc = false;
    ctx->multilink_our_mrru = false;
    ctx->multilink_our_short_sequence = false;
    ctx->multilink_our_endpoint = false;
    while (pos < total) {
        uint8_t opt_type = pkt[pos];
        uint8_t opt_len  = pkt[pos + 1];

        if (opt_len < 2 || pos + opt_len > total)
            return;

        if (opt_type == LCP_OPT_PFC && opt_len == 2)
            ctx->our_pfc = true;
        else if (opt_type == LCP_OPT_ACFC && opt_len == 2)
            ctx->our_acfc = true;
        else if (opt_type == LCP_OPT_MRRU && opt_len == 4)
            ctx->multilink_our_mrru = true;
        else if (opt_type == LCP_OPT_SHORT_SEQUENCE && opt_len == 2)
            ctx->multilink_our_short_sequence = true;
        else if (opt_type == LCP_OPT_ENDPOINT_DISC && opt_len >= 3)
            ctx->multilink_our_endpoint = true;

        pos += opt_len;
    }

    ppp_log(ctx->log, "PPP: Received LCP Configure-Ack\n");
    ctx->lcp_ack_received = true;
    ctx->lcp_req_sent = false;
    ctx->lcp_timeout_ms = 0;
    ppp_advance_state(ctx);
}

static bool
ppp_lcp_response_matches_request(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len,
                                 bool require_exact_options)
{
    int total;
    int pos = 4;
    int req_pos = 4;

    if (pkt_len < 4 || !ctx->lcp_req_sent || ctx->lcp_request_len < 4
        || ctx->lcp_request_len > sizeof(ctx->lcp_request)
        || pkt[1] != ctx->lcp_request[1])
        return false;

    total = (pkt[2] << 8) | pkt[3];
    if (total != pkt_len || total <= 4)
        return false;

    while (pos < total) {
        uint8_t opt_type = pkt[pos];
        uint8_t opt_len = pkt[pos + 1];

        if (opt_len < 2 || pos + opt_len > total)
            return false;

        while (req_pos < ctx->lcp_request_len
               && ctx->lcp_request[req_pos] != opt_type) {
            uint8_t req_len = ctx->lcp_request[req_pos + 1];

            if (req_len < 2 || req_pos + req_len > ctx->lcp_request_len)
                return false;
            req_pos += req_len;
        }

        if (req_pos >= ctx->lcp_request_len)
            return false;

        uint8_t req_len = ctx->lcp_request[req_pos + 1];
        if (req_len < 2 || req_pos + req_len > ctx->lcp_request_len
            || (req_len != opt_len && opt_type != LCP_OPT_AUTH_PROTO)
            || (require_exact_options
                && (req_len != opt_len
                    || memcmp(ctx->lcp_request + req_pos, pkt + pos, opt_len) != 0)))
            return false;

        req_pos += req_len;
        pos += opt_len;
    }

    return true;
}

/* Handle LCP Configure-Nak from peer */
static void
ppp_handle_lcp_config_nak(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    int      total = (pkt[2] << 8) | pkt[3];
    int      pos = 4;
    uint16_t our_mru = ctx->our_mru;
    uint32_t our_accm = ctx->our_accm;
    uint32_t our_magic = ctx->our_magic;
    bool     request_pfc = ctx->request_pfc;
    bool     request_acfc = ctx->request_acfc;
    bool     magic_nak = false;

    if (!ppp_lcp_response_matches_request(ctx, pkt, pkt_len, false))
    {
        MODEM_DEBUG_LOG(ctx->log, "LCP: ignored Configure-Nak id=%u (does not match outstanding request id=%u)\n",
                        (unsigned) pkt[1], (unsigned) ctx->lcp_request[1]);
        return;
    }

    while (pos < total) {
        uint8_t opt_type = pkt[pos];
        uint8_t opt_len  = pkt[pos + 1];

        switch (opt_type) {
            case LCP_OPT_MRU:
                if (opt_len != 4)
                    return;
                our_mru = (uint16_t) ((pkt[pos + 2] << 8) | pkt[pos + 3]);
                if (our_mru < 128 || our_mru > PPP_MAX_FRAME)
                    return;
                break;

            case LCP_OPT_MRRU:
                if (opt_len != 4)
                    return;
                ctx->multilink_our_mrru_value = (uint16_t) ((pkt[pos + 2] << 8)
                                                            | pkt[pos + 3]);
                if (ctx->multilink_our_mrru_value < 128
                    || ctx->multilink_our_mrru_value > PPP_MAX_FRAME)
                    return;
                break;

            case LCP_OPT_SHORT_SEQUENCE:
                if (opt_len != 2)
                    return;
                ctx->multilink_request_short_sequence = false;
                break;

            case LCP_OPT_ENDPOINT_DISC:
                if (opt_len < 3 || opt_len > 34)
                    return;
                ctx->multilink_request_endpoint = false;
                break;

            case LCP_OPT_ACCM:
                if (opt_len != 6)
                    return;
                our_accm |= ((uint32_t) pkt[pos + 2] << 24)
                          | ((uint32_t) pkt[pos + 3] << 16)
                          | ((uint32_t) pkt[pos + 4] << 8)
                          | (uint32_t) pkt[pos + 5];
                break;

            case LCP_OPT_MAGIC_NUMBER:
                if (opt_len != 6)
                    return;
                magic_nak = true;
                break;

            case LCP_OPT_PFC:
                if (opt_len != 2)
                    return;
                request_pfc = false;
                break;

            case LCP_OPT_ACFC:
                if (opt_len != 2)
                    return;
                request_acfc = false;
                break;

            case LCP_OPT_AUTH_PROTO:
                /* A peer cannot weaken the configured authentication method. */
                if (opt_len >= 4) {
                    uint16_t proto = (uint16_t) ((pkt[pos + 2] << 8) | pkt[pos + 3]);
                    ppp_auth_type_t suggested_auth = PPP_AUTH_NONE;

                    if (proto == PPP_AUTH_PROTO_PAP && opt_len == 4) {
                        suggested_auth = PPP_AUTH_PAP;
                    } else if (proto == PPP_AUTH_PROTO_EAP && opt_len == 4) {
                        suggested_auth = PPP_AUTH_EAP;
                    } else if (proto == PPP_AUTH_PROTO_CHAP && opt_len == 5) {
                        switch (pkt[pos + 4]) {
                            case CHAP_ALG_MD5:      suggested_auth = PPP_AUTH_CHAP_MD5; break;
                            case CHAP_ALG_SHA1:     suggested_auth = PPP_AUTH_CHAP_SHA1; break;
                            case CHAP_ALG_SHA256:   suggested_auth = PPP_AUTH_CHAP_SHA256; break;
                            case CHAP_ALG_SHA384:   suggested_auth = PPP_AUTH_CHAP_SHA384; break;
                            case CHAP_ALG_SHA512:   suggested_auth = PPP_AUTH_CHAP_SHA512; break;
                            case CHAP_ALG_MSCHAP:   suggested_auth = PPP_AUTH_MSCHAP;   break;
                            case CHAP_ALG_MSCHAPV2: suggested_auth = PPP_AUTH_MSCHAPV2; break;
                            default:                break;
                        }
                    }

                    if (suggested_auth != ctx->auth_type) {
                        ppp_log(ctx->log, "PPP: Peer suggested %s instead of configured %s; refusing authentication downgrade\n",
                            ppp_auth_name(suggested_auth), ppp_auth_name(ctx->auth_type));
                        ctx->state = PPP_STATE_DEAD;
                        ctx->lcp_req_sent = false;
                        return;
                    }
                }
                return;

            default:
                return;
        }
        pos += opt_len;
    }

    if (magic_nak) {
        uint8_t random[4];
        if (!ppp_random_bytes(random, sizeof(random))) {
            ctx->state = PPP_STATE_DEAD;
            ctx->lcp_req_sent = false;
            return;
        }
        our_magic = ((uint32_t) random[0] << 24) | ((uint32_t) random[1] << 16)
                  | ((uint32_t) random[2] << 8) | (uint32_t) random[3];
    }

    ctx->our_mru = our_mru;
    ctx->our_accm = our_accm;
    ctx->our_magic = our_magic;
    ctx->request_pfc = request_pfc;
    ctx->request_acfc = request_acfc;
    ctx->lcp_req_sent = false;
    ctx->lcp_ack_received = false;
    ppp_log(ctx->log, "PPP: Received LCP Configure-Nak\n");

    if (ctx->lcp_retries++ < PPP_LCP_MAX_RETRIES) {
        ppp_send_lcp_config_request(ctx);
    } else {
        ppp_log(ctx->log, "PPP: LCP negotiation failed after Configure-Nak retry limit\n");
        ctx->state = PPP_STATE_DEAD;
    }
}

/* Handle LCP Configure-Reject from peer */
static void
ppp_handle_lcp_config_reject(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    int total = (pkt[2] << 8) | pkt[3];
    int pos   = 4;

    if (!ppp_lcp_response_matches_request(ctx, pkt, pkt_len, true)) {
        MODEM_DEBUG_LOG(ctx->log, "LCP: ignored Configure-Reject id=%u (does not match outstanding request id=%u)\n",
                        (unsigned) pkt[1], (unsigned) ctx->lcp_request[1]);
        return;
    }

    ctx->lcp_req_sent = false;
    ctx->lcp_ack_received = false;
    ppp_log(ctx->log, "PPP: Received LCP Configure-Reject\n");

    while (pos < total && pos < pkt_len) {
        uint8_t opt_type = pkt[pos];
        uint8_t opt_len  = (pos + 1 < pkt_len) ? pkt[pos + 1] : 0;

        if (opt_len < 2 || pos + opt_len > pkt_len)
            break;

        if (opt_type == LCP_OPT_AUTH_PROTO) {
                ppp_log(ctx->log, "PPP: Peer rejected configured %s authentication; authentication is required\n",
                    ppp_auth_name(ctx->auth_type));
            ctx->state = PPP_STATE_DEAD;
            return;
        } else if (opt_type == LCP_OPT_PFC) {
            ctx->request_pfc = false;
            ctx->our_pfc     = false;
        } else if (opt_type == LCP_OPT_ACFC) {
            ctx->request_acfc = false;
            ctx->our_acfc     = false;
        } else if (opt_type == LCP_OPT_MRRU) {
            ctx->multilink_request_mrru = false;
            ctx->multilink_our_mrru = false;
        } else if (opt_type == LCP_OPT_SHORT_SEQUENCE) {
            ctx->multilink_request_short_sequence = false;
            ctx->multilink_our_short_sequence = false;
        } else if (opt_type == LCP_OPT_ENDPOINT_DISC) {
            ctx->multilink_request_endpoint = false;
            ctx->multilink_our_endpoint = false;
        }
        pos += opt_len;
    }

    /* Resend without rejected options */
    if (ctx->lcp_retries++ < PPP_LCP_MAX_RETRIES) {
        ppp_send_lcp_config_request(ctx);
    } else {
        ppp_log(ctx->log, "PPP: LCP negotiation failed after Configure-Reject retry limit\n");
        ctx->state = PPP_STATE_DEAD;
    }
}

/* Handle LCP Echo-Request */
static void
ppp_handle_lcp_echo_request(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    uint8_t reply[PPP_MAX_FRAME];
    int     total = (pkt[2] << 8) | pkt[3];

    if (total < 4 || total > pkt_len)
        return;

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
    ppp_log(ctx->log, "PPP: Sent LCP Echo-Reply\n");
}

/* Handle LCP Terminate-Request */
static void
ppp_handle_lcp_terminate_request(ppp_ctx_t *ctx, const uint8_t *pkt, int pkt_len)
{
    uint8_t reply[8];

    (void) pkt_len;

    ppp_log(ctx->log, "PPP: Received LCP Terminate-Request\n");
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
    int total = (pkt[2] << 8) | pkt[3];
    if (total < 4 || total > pkt_len)
        return;
    pkt_len = total;

    MODEM_DEBUG_LOG(ctx->log, "LCP: received %s (code=%u) id=%u length=%u\n",
                    ppp_lcp_code_name(code), (unsigned) code, (unsigned) pkt[1],
                    (unsigned) ((pkt[2] << 8) | pkt[3]));

    switch (code) {
        case PPP_CODE_CONFIGURE_REQUEST:
            if (ppp_handle_lcp_config_request(ctx, pkt, pkt_len))
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
            if (total >= 6 && pkt[4] == (uint8_t) (PPP_PROTO_CCP >> 8)
                && pkt[5] == (uint8_t) PPP_PROTO_CCP) {
                ppp_log(ctx->log, "PPP: Peer rejected CCP; continuing without compression or MPPE\n");
                ppp_ccp_fallback_plaintext(ctx);
                ppp_advance_state(ctx);
            } else if (total >= 6) {
                ppp_log(ctx->log, "PPP: Peer rejected protocol %s (0x%04X)\n",
                        ppp_protocol_name((uint16_t) (((uint16_t) pkt[4] << 8) | pkt[5])),
                        (unsigned) (((uint16_t) pkt[4] << 8) | pkt[5]));
            } else {
                ppp_log(ctx->log, "PPP: Received malformed Protocol-Reject\n");
            }
            break;
        default:
            ppp_log(ctx->log, "PPP: Unknown LCP code %d\n", code);
            break;
    }
}

/* Advance the PPP state machine */
void
ppp_advance_state(ppp_ctx_t *ctx)
{
    if (ctx->state == PPP_STATE_DEAD)
        return;

    switch (ctx->state) {
        case PPP_STATE_LCP_NEGOTIATE:
            if (ctx->lcp_ack_sent && ctx->lcp_ack_received) {
                ppp_log(ctx->log, "PPP: LCP opened, moving to auth phase\n");
                if (ctx->auth_type != PPP_AUTH_NONE && !ctx->auth_complete) {
                    ctx->state = PPP_STATE_AUTH;
                    ctx->auth_timeout_ms = 0;
                    if (ctx->auth_type == PPP_AUTH_EAP) {
                        ppp_eap_start(ctx);
                    } else if (ctx->auth_type == PPP_AUTH_CHAP_MD5
                     || ctx->auth_type == PPP_AUTH_CHAP_SHA1
                     || ctx->auth_type == PPP_AUTH_CHAP_SHA256
                     || ctx->auth_type == PPP_AUTH_CHAP_SHA384
                     || ctx->auth_type == PPP_AUTH_CHAP_SHA512
                     || ctx->auth_type == PPP_AUTH_MSCHAP
                     || ctx->auth_type == PPP_AUTH_MSCHAPV2) {
                        ppp_chap_send_challenge(ctx);
                    }
                    /* PAP: server waits for client to send Authenticate-Request */
                } else {
                    ctx->auth_complete = true;
                    bool multilink_joined = ppp_multilink_join(ctx);
                    if (ctx->state == PPP_STATE_DEAD)
                        break;
                    if (multilink_joined && !ppp_multilink_is_owner(ctx)) {
                        ctx->state = PPP_STATE_NETWORK;
                        ppp_log(ctx->log, "PPP: Multilink link ready; bundle owner handles IPCP\n");
                        break;
                    }
                    ctx->state         = PPP_STATE_IPCP_NEGOTIATE;
                    ppp_ipcp_send_config_request(ctx);
                }
            }
            break;

        case PPP_STATE_AUTH:
            if (ctx->auth_complete) {
                if (ctx->mppe_min_bits > 0
                    && (!ctx->mppe_keys_ready || ctx->auth_type != PPP_AUTH_MSCHAPV2)) {
                    ppp_log(ctx->log, "PPP: Required MPPE keys unavailable\n");
                    ctx->state = PPP_STATE_DEAD;
                    break;
                }
                ppp_log(ctx->log, "PPP: Auth complete, moving to IPCP\n");
                bool multilink_joined = ppp_multilink_join(ctx);
                if (ctx->state == PPP_STATE_DEAD)
                    break;
                if (multilink_joined && !ppp_multilink_is_owner(ctx)) {
                    ctx->state = PPP_STATE_NETWORK;
                    ctx->auth_timeout_ms = 0;
                    ppp_log(ctx->log, "PPP: Multilink link ready; bundle owner handles IPCP\n");
                    break;
                }
                ctx->state = PPP_STATE_IPCP_NEGOTIATE;
                ctx->auth_timeout_ms = 0;
                if (ctx->mppe_keys_ready)
                    ppp_ccp_start(ctx);
                ppp_ipcp_send_config_request(ctx);
            }
            break;

        case PPP_STATE_IPCP_NEGOTIATE:
            if (ctx->ipcp_ack_sent && ctx->ipcp_ack_received) {
                if ((!ctx->mppe_keys_ready && ctx->mppe_min_bits == 0)
                    || ctx->ccp_open
                    || (ctx->ccp_plaintext_fallback && ctx->mppe_min_bits == 0)) {
                    ppp_log(ctx->log, "PPP: IPCP opened, entering network phase\n");
                    ctx->state = PPP_STATE_NETWORK;
                }
            }
            break;

        default:
            break;
    }
}

void
ppp_timer_tick(ppp_ctx_t *ctx)
{
    if (ctx->state == PPP_STATE_AUTH) {
        if (++ctx->auth_timeout_ms >= PPP_AUTH_TIMEOUT_MS) {
            ppp_log(ctx->log, "PPP: Authentication timed out waiting for peer\n");
            ctx->state = PPP_STATE_DEAD;
        }
        return;
    }

    if (ctx->state != PPP_STATE_LCP_NEGOTIATE || !ctx->lcp_req_sent)
        return;

    if (++ctx->lcp_timeout_ms < PPP_LCP_TIMEOUT_MS)
        return;

    ctx->lcp_timeout_ms = 0;
    if (ctx->lcp_retries++ < PPP_LCP_MAX_RETRIES) {
        ppp_log(ctx->log, "PPP: LCP Configure-Request timed out; retrying (%d/%d)\n",
                ctx->lcp_retries, PPP_LCP_MAX_RETRIES);
        ppp_send_lcp_config_request(ctx);
    } else {
        ppp_log(ctx->log, "PPP: LCP negotiation timed out after %d retries\n",
                PPP_LCP_MAX_RETRIES);
        ctx->state = PPP_STATE_DEAD;
        ctx->lcp_req_sent = false;
    }
}

/* Process a complete PPP frame (after HDLC un-escaping and FCS check) */
static void
ppp_ccp_send_reset_request(ppp_ctx_t *ctx)
{
    uint8_t reset_request[4] = {
        PPP_CODE_RESET_REQUEST, ctx->ccp_reset_id++, 0, 4
    };

    ctx->ccp_reset_request_id = reset_request[1];
    ctx->ccp_reset_pending = true;
    ppp_send_frame(ctx, PPP_PROTO_CCP, reset_request, sizeof(reset_request));
}

/* Process a complete PPP frame (after HDLC un-escaping and FCS check) */
static void
ppp_process_frame(ppp_ctx_t *ctx, const uint8_t *frame, int frame_len, bool reassembled)
{
    uint16_t protocol;
    int      data_offset;
    bool     ac_present = false;
    bool     compression_active = ctx->state >= PPP_STATE_AUTH;

    if (frame_len < 1)
        return;

    /* Check for Address and Control fields */
    if (frame_len >= 2 && frame[0] == PPP_ADDRESS && frame[1] == PPP_CONTROL) {
        ac_present  = true;
        data_offset = 2;
    } else if (compression_active && ctx->our_acfc) {
        data_offset = 0;
    } else {
        ppp_log(ctx->log, "PPP: Frame missing Address/Control fields\n");
        return;
    }

    if (frame_len - data_offset
        > (reassembled ? ctx->multilink_our_mrru_value : ctx->our_mru))
        return;

    if (data_offset >= frame_len)
        return;

    if (frame[data_offset] & 0x01) {
        if (!compression_active || !ctx->our_pfc)
            return;
        protocol = frame[data_offset++];
    } else {
        if (data_offset + 1 >= frame_len)
            return;
        protocol = (uint16_t) ((frame[data_offset] << 8) | frame[data_offset + 1]);
        data_offset += 2;
    }

    if (protocol == PPP_PROTO_LCP && !ac_present)
        return;

    const uint8_t *data     = frame + data_offset;
    int            data_len = frame_len - data_offset;
    uint16_t       wire_protocol = protocol;
    int            wire_data_len = data_len;
#if defined(ENABLE_MODEM_LOG) && defined(ENABLE_MODEM_DEBUG)
    const char    *decode_method = "none";
#endif
    uint8_t        decrypted[PPP_MAX_FRAME];
    uint8_t        decompressed[PPP_MAX_FRAME];

    MODEM_DEBUG_LOG(ctx->log, "PPP: RX wire-protocol=%s (0x%04X) wire-payload=%d "
                    "state=%s CCP-open=%u CCP-method=%s MPPE-rx=%u\n",
                    ppp_protocol_name(wire_protocol), (unsigned) wire_protocol,
                    wire_data_len, ppp_state_name(ctx->state),
                    (unsigned) ctx->ccp_open,
                    ppp_ccp_method_name(ctx->ccp_rx_method),
                    (unsigned) ctx->mppe_rx_enabled);

    if (protocol == PPP_PROTO_MULTILINK) {
        if (ctx->multilink_bundle)
            ppp_mppp_reassembler_input(&ctx->multilink_bundle->reassembler,
                                       data, (size_t) data_len,
                                       ppp_mppp_deliver_packet,
                                       ctx->multilink_bundle);
        else
            ppp_log(ctx->log, "PPP: Received Multilink frame outside a bundle\n");
        return;
    }

    if (protocol == PPP_PROTO_MPPE && ctx->mppe_rx_enabled) {
        size_t decrypted_len;
        if (!ctx->mppe_rx_enabled
            || !ppp_mppe_decrypt(&ctx->mppe_rx, data, (size_t) data_len,
                                 decrypted, sizeof(decrypted), &decrypted_len)) {
            if (ctx->mppe_rx.reset_requested) {
                uint8_t reset_request[4] = {
                    PPP_CODE_RESET_REQUEST, ctx->ccp_reset_id++, 0, 4
                };
                ppp_send_frame(ctx, PPP_PROTO_CCP, reset_request, sizeof(reset_request));
                ctx->mppe_rx.reset_requested = false;
            }
            return;
        }
    #if defined(ENABLE_MODEM_LOG) && defined(ENABLE_MODEM_DEBUG)
        decode_method = "MPPE";
    #endif
        if (decrypted_len < 2)
            return;
        protocol = (uint16_t) (((uint16_t) decrypted[0] << 8) | decrypted[1]);
        data = decrypted + 2;
        data_len = (int) decrypted_len - 2;
        if (protocol < PPP_PROTO_IP || protocol > 0x00FA)
            return;
    } else if (protocol == PPP_PROTO_MPPE
               && ctx->ccp_open
               && (ctx->ccp_rx_method == PPP_CCP_METHOD_PREDICTOR1
                   || ctx->ccp_rx_method == PPP_CCP_METHOD_PREDICTOR2
                   || ctx->ccp_rx_method == PPP_CCP_METHOD_LZS
                   || ctx->ccp_rx_method == PPP_CCP_METHOD_DEFLATE
                         || ctx->ccp_rx_method == PPP_CCP_METHOD_MPPC
                         || ctx->ccp_rx_method == PPP_CCP_METHOD_BSD)) {
        int decompressed_len;
                #if defined(ENABLE_MODEM_LOG) && defined(ENABLE_MODEM_DEBUG)
                    decode_method = ppp_ccp_method_name(ctx->ccp_rx_method);
                #endif
        if (ctx->ccp_reset_pending
            && (ctx->ccp_rx_method == PPP_CCP_METHOD_PREDICTOR2
                || ctx->ccp_rx_method == PPP_CCP_METHOD_MPPC))
            return;
        if (ctx->ccp_rx_method == PPP_CCP_METHOD_PREDICTOR2) {
            bool first_segment = true;
            for (;;) {
                if (!ppp_ccp_codec_decompress(ctx, first_segment ? data : NULL,
                                              first_segment ? data_len : 0,
                                              decompressed, sizeof(decompressed),
                                              &decompressed_len)) {
                    if (!ctx->ccp_reset_pending)
                        ppp_ccp_send_reset_request(ctx);
                    return;
                }
                first_segment = false;
                if (decompressed_len == 0)
                    return;
                if (decompressed_len < 2) {
                    if (!ctx->ccp_reset_pending)
                        ppp_ccp_send_reset_request(ctx);
                    return;
                }

                uint16_t decoded_protocol = (uint16_t) (((uint16_t) decompressed[0] << 8)
                                                       | decompressed[1]);
                if (decoded_protocol < PPP_PROTO_IP || decoded_protocol > 0x00FA)
                    continue;

                uint8_t decoded_frame[PPP_MAX_FRAME + 4] = {
                    PPP_ADDRESS, PPP_CONTROL,
                    (uint8_t) (decoded_protocol >> 8), (uint8_t) decoded_protocol
                };
                memcpy(decoded_frame + 4, decompressed + 2, (size_t) decompressed_len - 2);
                ppp_process_frame(ctx, decoded_frame, decompressed_len + 2, reassembled);
            }
        }
        if (!ppp_ccp_codec_decompress(ctx, data, data_len, decompressed,
                                      sizeof(decompressed), &decompressed_len)
            || decompressed_len < 2) {
            if (!ctx->ccp_reset_pending && ctx->ccp_rx_method != PPP_CCP_METHOD_LZS)
                ppp_ccp_send_reset_request(ctx);
            return;
        }
        if (ctx->ccp_rx_method == PPP_CCP_METHOD_BSD
            || ctx->ccp_rx_method == PPP_CCP_METHOD_DEFLATE) {
            if (decompressed[0] & 1) {
                protocol = decompressed[0];
                data = decompressed + 1;
                data_len = decompressed_len - 1;
            } else {
                if (decompressed_len < 2)
                    return;
                protocol = (uint16_t) (((uint16_t) decompressed[0] << 8) | decompressed[1]);
                data = decompressed + 2;
                data_len = decompressed_len - 2;
            }
        } else {
            protocol = (uint16_t) (((uint16_t) decompressed[0] << 8) | decompressed[1]);
            data = decompressed + 2;
            data_len = decompressed_len - 2;
        }
        if (protocol < PPP_PROTO_IP || protocol > 0x00FA)
            return;
    } else if (protocol == PPP_PROTO_MPPE) {
        return;
    } else if (ctx->mppe_keys_ready && !ctx->ccp_plaintext_fallback && ctx->ccp_open
               && protocol >= PPP_PROTO_IP && protocol <= 0x00FA
               && ctx->mppe_rx_enabled) {
        return;
    } else if (ctx->mppe_keys_ready && !ctx->ccp_plaintext_fallback && !ctx->ccp_open
               && protocol >= PPP_PROTO_IP && protocol <= 0x00FA) {
        return;
    }

            ppp_log(ctx->log, "PPP: Received frame proto=%s (0x%04X) len=%d (state=%s)\n",
                ppp_protocol_name(protocol), protocol, data_len, ppp_state_name(ctx->state));
        if (protocol != wire_protocol || data_len != wire_data_len) {
            MODEM_DEBUG_LOG(ctx->log, "PPP: RX decoded protocol=%s (0x%04X) payload=%d "
                            "from-wire=%s (0x%04X) wire-payload=%d transform=%s\n",
                            ppp_protocol_name(protocol), (unsigned) protocol, data_len,
                            ppp_protocol_name(wire_protocol), (unsigned) wire_protocol,
                            wire_data_len, decode_method);
        } else {
            MODEM_DEBUG_LOG(ctx->log, "PPP: RX protocol=%s (0x%04X) payload=%d "
                            "address-control=%u compression=%u state=%s\n",
                            ppp_protocol_name(protocol), (unsigned) protocol, data_len,
                            (unsigned) ac_present, (unsigned) compression_active,
                            ppp_state_name(ctx->state));
        }

    switch (protocol) {
        case PPP_PROTO_LCP:
            ppp_process_lcp(ctx, data, data_len);
            break;

        case PPP_PROTO_PAP:
            if (ctx->state == PPP_STATE_AUTH) {
                ctx->auth_timeout_ms = 0;
                ppp_pap_process(ctx, data, data_len);
            } else {
                MODEM_DEBUG_LOG(ctx->log, "PPP: Ignoring PAP packet outside auth phase (state=%s length=%d)\n",
                                ppp_state_name(ctx->state), data_len);
            }
            break;

        case PPP_PROTO_CHAP:
            if (ctx->state == PPP_STATE_AUTH) {
                ctx->auth_timeout_ms = 0;
                ppp_chap_process(ctx, data, data_len);
            } else {
                MODEM_DEBUG_LOG(ctx->log, "PPP: Ignoring CHAP packet outside auth phase (state=%s length=%d)\n",
                                ppp_state_name(ctx->state), data_len);
            }
            break;

        case PPP_PROTO_EAP:
            if (ctx->state == PPP_STATE_AUTH && ctx->auth_type == PPP_AUTH_EAP) {
                ctx->auth_timeout_ms = 0;
                ppp_eap_process(ctx, data, data_len);
            }
            break;

        case PPP_PROTO_IPCP:
            if (ctx->state >= PPP_STATE_IPCP_NEGOTIATE) {
                ppp_ipcp_process(ctx, data, data_len);
            }
            break;

        case PPP_PROTO_CCP:
            if (ctx->state >= PPP_STATE_IPCP_NEGOTIATE && ctx->state < PPP_STATE_TERMINATING)
                ppp_ccp_process(ctx, data, data_len);
            break;

        case PPP_PROTO_IP:
            if (ctx->state == PPP_STATE_NETWORK
                && (!ctx->multilink_bundle || ppp_multilink_is_owner(ctx))) {
                ctx->network_send_ip(ctx->modem, data, data_len);
            }
            break;

        case PPP_PROTO_VJ_COMPRESSED:
        case PPP_PROTO_VJ_UNCOMPRESSED:
            if (ctx->state == PPP_STATE_NETWORK && ctx->vj_rx_enabled && ctx->vj_rx_ctx
                && (!ctx->multilink_bundle || ppp_multilink_is_owner(ctx))) {
                uint8_t ip_packet[PPP_MAX_FRAME + VJ_MAX_HDR];
                uint8_t normalized[PPP_MAX_FRAME];
                int     vj_type = protocol == PPP_PROTO_VJ_COMPRESSED
                                ? VJ_TYPE_COMPRESSED_TCP : VJ_TYPE_UNCOMPRESSED_TCP;

                if (protocol == PPP_PROTO_VJ_UNCOMPRESSED) {
                    if (data_len < 20 || data[9] > ctx->vj_rx_max_slot_id)
                        break;
                    memcpy(normalized, data, data_len);
                    normalized[9] |= VJ_TYPE_UNCOMPRESSED_TCP;
                    data = normalized;
                }

                ctx->vj_rx_ctx->num_slots = (int) ctx->vj_rx_max_slot_id + 1;
                int ip_len = cslip_decompress(ctx->vj_rx_ctx, data, data_len,
                                              ip_packet, vj_type);
                if (ip_len > 0)
                    ctx->network_send_ip(ctx->modem, ip_packet, ip_len);
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
                ppp_log(ctx->log, "PPP: Sent Protocol-Reject for 0x%04X\n", protocol);
            }
            break;
    }
}

/* Process a byte received from the serial line (HDLC framing) */
void
ppp_rx_byte(ppp_ctx_t *ctx, uint8_t byte)
{
    if (ctx->ras_rx_in_frame) {
        if (ctx->ras_rx_len >= (int) sizeof(ctx->ras_rx_buf)) {
            ctx->ras_rx_in_frame = false;
            ctx->ras_rx_len = 0;
            ctx->ras_rx_expected = 0;
            return;
        }
        ctx->ras_rx_buf[ctx->ras_rx_len++] = byte;
        if (ctx->ras_rx_len == 4) {
            int body_length = (ctx->ras_rx_buf[2] << 8) | ctx->ras_rx_buf[3];
            ctx->ras_rx_expected = body_length + 7;
            if (body_length < 3 || ctx->ras_rx_expected > (int) sizeof(ctx->ras_rx_buf)) {
                ctx->ras_rx_in_frame = false;
                ctx->ras_rx_len = 0;
                ctx->ras_rx_expected = 0;
                return;
            }
        }
        if (ctx->ras_rx_expected > 0 && ctx->ras_rx_len == ctx->ras_rx_expected) {
            ppp_process_ras_frame(ctx, ctx->ras_rx_buf, ctx->ras_rx_len);
            ctx->ras_rx_in_frame = false;
            ctx->ras_rx_len = 0;
            ctx->ras_rx_expected = 0;
        }
        return;
    }

    if (ctx->ccp_rx_method == PPP_CCP_METHOD_NT31RAS && ctx->rx_len == 0
        && byte == PPP_RAS_SYN) {
        ctx->ras_rx_buf[0] = byte;
        ctx->ras_rx_len = 1;
        ctx->ras_rx_expected = 0;
        ctx->ras_rx_in_frame = true;
        return;
    }

    if (byte == PPP_FLAG) {
        if (ctx->rx_in_frame && ctx->rx_len > 2) {
            /* End of frame - check FCS */
            uint16_t calc_fcs = ppp_fcs16(ctx->rx_buf, ctx->rx_len - 2);
            uint16_t recv_fcs = (uint16_t) (ctx->rx_buf[ctx->rx_len - 2])
                              | ((uint16_t) (ctx->rx_buf[ctx->rx_len - 1]) << 8);

            if (calc_fcs == recv_fcs) {
                MODEM_DEBUG_LOG(ctx->log, "PPP: HDLC frame complete length=%d FCS valid\n", ctx->rx_len);
                ppp_process_frame(ctx, ctx->rx_buf, ctx->rx_len - 2, false);
            } else {
                ppp_log(ctx->log, "PPP: FCS error (calc=0x%04X recv=0x%04X)\n", calc_fcs, recv_fcs);
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
    if (!ip_pkt || len <= 0 || len > PPP_MAX_FRAME - 6) {
        ppp_log(ctx->log, "PPP: Dropping invalid IP packet (%d bytes)\n", len);
        return;
    }
    if (ctx->mppe_keys_ready && !ctx->ccp_open && !ctx->ccp_plaintext_fallback)
        return;

    if (ctx->vj_tx_enabled && ctx->vj_tx_ctx && len > 0
        && ctx->ccp_tx_method != PPP_CCP_METHOD_NT31RAS) {
        uint8_t compressed[PPP_MAX_FRAME + VJ_MAX_HDR];
        int     type = VJ_TYPE_IP;

        ctx->vj_tx_ctx->num_slots = (int) ctx->vj_tx_max_slot_id + 1;
        ctx->vj_tx_ctx->compress_slot_id = ctx->vj_tx_comp_slot_id;
        int compressed_len = cslip_compress(ctx->vj_tx_ctx, ip_pkt, len, compressed, &type);
        if (compressed_len > 0 && type == VJ_TYPE_COMPRESSED_TCP) {
            ppp_send_frame(ctx, PPP_PROTO_VJ_COMPRESSED, compressed, compressed_len);
            return;
        }
        if (compressed_len > 0 && type == VJ_TYPE_UNCOMPRESSED_TCP) {
            compressed[9] &= VJ_MAX_SLOTS - 1;
            ppp_send_frame(ctx, PPP_PROTO_VJ_UNCOMPRESSED, compressed, compressed_len);
            return;
        }
    }

    ppp_send_frame(ctx, PPP_PROTO_IP, ip_pkt, len);
}

/* Initialize PPP context */
ppp_ctx_t *
ppp_init(void *modem, void *log,
         void (*serial_push)(void *, const uint8_t *, int),
         void (*network_send_ip)(void *, const uint8_t *, int))
{
    uint8_t   magic[4];
    ppp_ctx_t *ctx = (ppp_ctx_t *) calloc(1, sizeof(ppp_ctx_t));
    if (!ctx)
        return NULL;

    if (!ppp_random_bytes(magic, sizeof(magic))) {
        free(ctx);
        return NULL;
    }

    ctx->modem           = modem;
    ctx->log             = log;
    ctx->serial_push     = serial_push;
    ctx->network_send_ip = network_send_ip;
    ctx->state           = PPP_STATE_DEAD;
    ctx->our_mru         = PPP_DEFAULT_MRU;
    ctx->peer_mru        = PPP_DEFAULT_MRU;
    ctx->our_accm        = 0xFFFFFFFF; /* Send all control chars escaped initially */
    ctx->peer_accm       = 0xFFFFFFFF;
    ctx->our_magic       = ((uint32_t) magic[0] << 24) | ((uint32_t) magic[1] << 16)
                         | ((uint32_t) magic[2] << 8) | (uint32_t) magic[3];
    ctx->auth_type       = PPP_AUTH_NONE;
    ctx->vj_tx_ctx       = cslip_init(log);
    ctx->vj_rx_ctx       = cslip_init(log);
    if (!ctx->vj_tx_ctx || !ctx->vj_rx_ctx) {
        cslip_close(ctx->vj_tx_ctx);
        cslip_close(ctx->vj_rx_ctx);
        ctx->vj_tx_ctx = NULL;
        ctx->vj_rx_ctx = NULL;
    }

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
        ppp_multilink_detach(ctx);
        if (ctx->state != PPP_STATE_DEAD && ctx->state != PPP_STATE_TERMINATING) {
            /* Send Terminate-Request */
            uint8_t pkt[4];
            pkt[0] = PPP_CODE_TERMINATE_REQUEST;
            pkt[1] = ctx->lcp_id++;
            pkt[2] = 0;
            pkt[3] = 4;
            ppp_send_frame(ctx, PPP_PROTO_LCP, pkt, 4);
        }
        cslip_close(ctx->vj_tx_ctx);
        cslip_close(ctx->vj_rx_ctx);
        ppp_ccp_codec_close(ctx);
        free(ctx);
    }
}

/* Start PPP negotiation (called when entering PPP mode) */
void
ppp_start(ppp_ctx_t *ctx)
{
    if (ctx->mppe_min_bits > 0 && ctx->auth_type != PPP_AUTH_MSCHAPV2) {
        ppp_log(ctx->log, "PPP: Required MPPE needs MS-CHAPv2 authentication\n");
        ctx->state = PPP_STATE_DEAD;
        return;
    }

    ctx->state            = PPP_STATE_LCP_NEGOTIATE;
    ctx->lcp_ack_sent     = false;
    ctx->lcp_ack_received = false;
    ctx->lcp_req_sent     = false;
    ctx->lcp_retries      = 0;
    ctx->lcp_timeout_ms   = 0;
    ctx->auth_complete    = false;
    ctx->auth_timeout_ms  = 0;
    ctx->our_pfc          = false;
    ctx->our_acfc         = false;
    ctx->peer_pfc         = false;
    ctx->peer_acfc        = false;
    ctx->request_pfc      = true;
    ctx->request_acfc     = true;
    ctx->multilink_request_mrru = ctx->multilink_group[0] != '\0';
    ctx->multilink_request_short_sequence = ctx->multilink_group[0] != '\0';
    ctx->multilink_request_endpoint = ctx->multilink_group[0] != '\0';
    ctx->multilink_our_mrru = false;
    ctx->multilink_peer_mrru = false;
    ctx->multilink_our_short_sequence = false;
    ctx->multilink_peer_short_sequence = false;
    ctx->multilink_our_endpoint = false;
    ctx->multilink_peer_endpoint = false;
    ctx->multilink_our_mrru_value = PPP_DEFAULT_MRU + 2;
    ctx->multilink_peer_mrru_value = PPP_DEFAULT_MRU + 2;
    ctx->multilink_peer_endpoint_length = 0;
    ctx->rx_in_frame      = false;
    ctx->rx_len           = 0;
    ctx->rx_escaped       = false;
    ctx->eap_state        = PPP_EAP_STATE_IDLE;
    ctx->eap_id           = 0;
    ctx->eap_request_id   = 0;
    ctx->ipcp_ack_sent    = false;
    ctx->ipcp_ack_received = false;
    ctx->ipcp_req_sent    = false;
    ctx->ipcp_retries     = 0;
    ctx->ipcp_vj_request  = ctx->vj_tx_ctx && ctx->vj_rx_ctx;
    ctx->vj_tx_enabled    = false;
    ctx->vj_rx_enabled    = false;
    ctx->vj_tx_max_slot_id = IPCP_VJ_MAX_SLOT_ID;
    ctx->vj_rx_max_slot_id = IPCP_VJ_MAX_SLOT_ID;
    ctx->vj_tx_comp_slot_id = true;
    ctx->vj_rx_comp_slot_id = true;
    ctx->ccp_req_sent = false;
    ctx->ccp_ack_sent = false;
    ctx->ccp_ack_received = false;
    ctx->ccp_peer_mppe = false;
    ctx->ccp_open = false;
    ctx->ccp_rejected = false;
    ctx->ccp_plaintext_fallback = false;
    ctx->mppe_tx_enabled = false;
    ctx->mppe_rx_enabled = false;

    ppp_send_lcp_config_request(ctx);
}
