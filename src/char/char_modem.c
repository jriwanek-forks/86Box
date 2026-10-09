/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Hayes AT-compliant modem over the character-device API.
 *
 * Authors: The DOSBox Team
 *          Cacodemon345
 *          Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2002-2021 The DOSBox Team.
 *          Copyright 2022      The DOSBox Staging Team.
 *          Copyright 2024      Cacodemon345.
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <wchar.h>
#include <stdbool.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/thread.h>
#include <86box/fifo.h>
#include <86box/fifo8.h>
#include <86box/timer.h>
#include <86box/char.h>
#include <86box/plat.h>
#include <86box/network.h>
#include <86box/log.h>
#include <86box/version.h>
#include <86box/plat_unused.h>
#include <86box/plat_netsocket.h>
#ifndef _WIN32
#    include <arpa/inet.h>
#endif
#include <86box/modem/modem_ppp.h>
#include <86box/modem/modem_cslip.h>
#include <86box/modem/modem_slip.h>
#include <86box/modem/modem_slip_auth.h>
#include <86box/modem/modem_sound.h>
#include <86box/sound.h>
#include <86box/modem/modem_voice.h>
#include <86box/modem/modem_debug.h>

/* Keep the COM-device entry points distinct from the network modem. */
#define modem_send_res char_modem_send_res
#define modem_enter_idle_state char_modem_enter_idle_state
#define modem_enter_connected_state char_modem_enter_connected_state
#define modem_reset char_modem_reset
#define modem_dial char_modem_dial
#define modem_dtr_callback_timer char_modem_dtr_callback_timer
#define modem_dtr_callback char_modem_dtr_callback
#define modem_process_telnet char_modem_process_telnet
#define modem_device char_modem_device
#define modem_close char_modem_close
#define trim char_modem_trim

#ifdef ENABLE_MODEM_LOG
uint8_t char_modem_do_log = ENABLE_MODEM_LOG;

static void
modem_log(void *priv, const char *fmt, ...)
{
    va_list ap;

    if (char_modem_do_log) {
        va_start(ap, fmt);
        log_out(priv, fmt, ap);
        va_end(ap);
    }
}
#else
#    define modem_log(priv, fmt, ...)
#endif

#ifdef ENABLE_MODEM_LOG
static const char *
modem_ethertype_name(uint16_t ethertype)
{
    switch (ethertype) {
        case 0x0800: return "IPv4";
        case 0x0806: return "ARP";
        case 0x8137: return "IPX";
        case 0x86DD: return "IPv6";
        case 0x88CC: return "LLDP";
        default:     return "Unknown";
    }
}
#endif

#if defined(ENABLE_MODEM_LOG) && defined(ENABLE_MODEM_DEBUG)
static const char *
modem_ipv4_protocol_name(uint8_t protocol)
{
    switch (protocol) {
        case 1:   return "ICMP";
        case 2:   return "IGMP";
        case 4:   return "IPv4 encapsulation";
        case 6:   return "TCP";
        case 17:  return "UDP";
        case 41:  return "IPv6 encapsulation";
        case 47:  return "GRE";
        case 50:  return "ESP";
        case 51:  return "AH";
        case 58:  return "ICMPv6";
        case 89:  return "OSPF";
        case 132: return "SCTP";
        default:  return "Unknown";
    }
}

static const char *
modem_ipv4_port_name(uint16_t port)
{
    switch (port) {
        case 20:  return "FTP-data";
        case 21:  return "FTP-control";
        case 22:  return "SSH";
        case 23:  return "Telnet";
        case 25:  return "SMTP";
        case 53:  return "DNS";
        case 67:  return "DHCP-server";
        case 68:  return "DHCP-client";
        case 80:  return "HTTP";
        case 110: return "POP3";
        case 123: return "NTP";
        case 137: return "NetBIOS-NS";
        case 138: return "NetBIOS-DGM";
        case 139: return "NetBIOS-SSN";
        case 143: return "IMAP";
        case 161: return "SNMP";
        case 443: return "HTTPS";
        case 445: return "SMB";
        default:  return NULL;
    }
}

static const char *
modem_ipv4_icmp_type_name(uint8_t type)
{
    if (type >= 20 && type <= 29)
        return "Reserved for Robustness Experiment";
    if (type >= 44 && type <= 252)
        return "Unassigned";

    switch (type) {
        case 0:   return "Echo Reply";
        case 1:   return "Unassigned";
        case 2:   return "Unassigned";
        case 3:   return "Destination Unreachable";
        case 4:   return "Source Quench (Deprecated)";
        case 5:   return "Redirect";
        case 6:   return "Alternate Host Address (Deprecated)";
        case 7:   return "Unassigned";
        case 8:   return "Echo Request";
        case 9:   return "Router Advertisement";
        case 10:  return "Router Solicitation";
        case 11:  return "Time Exceeded";
        case 12:  return "Parameter Problem";
        case 13:  return "Timestamp Request";
        case 14:  return "Timestamp Reply";
        case 15:  return "Information Request (Deprecated)";
        case 16:  return "Information Reply (Deprecated)";
        case 17:  return "Address Mask Request (Deprecated)";
        case 18:  return "Address Mask Reply (Deprecated)";
        case 19:  return "Reserved for Security";
        case 30:  return "Traceroute (Deprecated)";
        case 31:  return "Datagram Conversion Error (Deprecated)";
        case 32:  return "Mobile Host Redirect (Deprecated)";
        case 33:  return "IPv6 Where-Are-You (Deprecated)";
        case 34:  return "IPv6 I-Am-Here (Deprecated)";
        case 35:  return "Mobile Registration Request (Deprecated)";
        case 36:  return "Mobile Registration Reply (Deprecated)";
        case 37:  return "Domain Name Request (Deprecated)";
        case 38:  return "Domain Name Reply (Deprecated)";
        case 39:  return "SKIP (Deprecated)";
        case 40:  return "Photuris";
        case 41:  return "Mobility Protocol Message";
        case 42:  return "Extended Echo Request";
        case 43:  return "Extended Echo Reply";
        case 253: return "RFC 3692 Experiment 1";
        case 254: return "RFC 3692 Experiment 2";
        case 255: return "Reserved";
        default:  return "Unknown";
    }
}

static const char *
modem_ipv4_icmp_code_name(uint8_t type, uint8_t code)
{
    if ((type == 0 || type == 8 || type == 10) && code == 0)
        return "No additional code";
    if (type == 3 && code == 3)
        return "Port Unreachable";

    return NULL;
}

static void
modem_debug_log_ipv4(void *log, const char *direction, const uint8_t *packet, int len)
{
    int      header_len;
    uint16_t total_len;
    uint16_t fragment;
    uint16_t fragment_offset;
    uint8_t  protocol;
    const char *destination_label = "";

    if (len < 20 || (packet[0] >> 4) != 4) {
        MODEM_DEBUG_LOG(log, "%s: packet length=%d (not IPv4)\n", direction, len);
        return;
    }

    header_len = (packet[0] & 0x0F) * 4;
    if (header_len < 20 || len < header_len) {
        MODEM_DEBUG_LOG(log, "%s: malformed IPv4 header length=%d packet=%d\n",
                        direction, header_len, len);
        return;
    }

    total_len = (uint16_t) (((uint16_t) packet[2] << 8) | packet[3]);
    if (total_len < header_len) {
        MODEM_DEBUG_LOG(log, "%s: malformed IPv4 total length=%u header=%d packet=%d\n",
                        direction, (unsigned) total_len, header_len, len);
        return;
    }

    protocol = packet[9];
    fragment = (uint16_t) (((uint16_t) packet[6] << 8) | packet[7]);
    fragment_offset = fragment & 0x1FFF;
    if (packet[16] == 224 && packet[17] == 0 && packet[18] == 0 && packet[19] == 2)
        destination_label = " (all-routers multicast)";
    else if (packet[16] >= 224 && packet[16] <= 239)
        destination_label = " (multicast)";

    MODEM_DEBUG_LOG(log, "%s: IPv4 length=%d total=%u header=%d tos=0x%02X ttl=%u id=%u "
                    "protocol=%s (%u) %u.%u.%u.%u -> %u.%u.%u.%u%s\n",
                    direction, len, (unsigned) total_len, header_len, (unsigned) packet[1],
                    (unsigned) packet[8], (unsigned) (((uint16_t) packet[4] << 8) | packet[5]),
                    modem_ipv4_protocol_name(protocol), (unsigned) protocol,
                    (unsigned) packet[12], (unsigned) packet[13],
                    (unsigned) packet[14], (unsigned) packet[15],
                    (unsigned) packet[16], (unsigned) packet[17],
                    (unsigned) packet[18], (unsigned) packet[19], destination_label);

    if (fragment & 0x7FFF) {
        MODEM_DEBUG_LOG(log, "%s: IPv4 fragmentation offset=%u more-fragments=%s "
                        "dont-fragment=%s\n",
                        direction, (unsigned) fragment_offset,
                        (fragment & 0x2000) ? "yes" : "no",
                        (fragment & 0x4000) ? "yes" : "no");
    }

    if (fragment_offset == 0 && protocol == 1 && total_len >= header_len + 2
        && len >= header_len + 2) {
        uint8_t code = packet[header_len + 1];
        const char *code_name = modem_ipv4_icmp_code_name(packet[header_len], code);
        MODEM_DEBUG_LOG(log, "%s: ICMP type=%u (%s) code=%u%s%s%s\n", direction,
                        (unsigned) packet[header_len],
                        modem_ipv4_icmp_type_name(packet[header_len]),
                        (unsigned) code, code_name ? " (" : "",
                        code_name ? code_name : "", code_name ? ")" : "");
    } else if (fragment_offset == 0 && (protocol == 6 || protocol == 17)) {
        int minimum_len = protocol == 6 ? 20 : 8;
        if (total_len >= header_len + minimum_len && len >= header_len + minimum_len) {
            uint16_t source_port = (uint16_t) (((uint16_t) packet[header_len] << 8)
                                               | packet[header_len + 1]);
            uint16_t dest_port = (uint16_t) (((uint16_t) packet[header_len + 2] << 8)
                                             | packet[header_len + 3]);
            const char *source_name = modem_ipv4_port_name(source_port);
            const char *dest_name = modem_ipv4_port_name(dest_port);

            MODEM_DEBUG_LOG(log, "%s: %s source-port=%u%s%s%s destination-port=%u%s%s%s\n",
                            direction, modem_ipv4_protocol_name(protocol),
                            (unsigned) source_port, source_name ? " (" : "",
                            source_name ? source_name : "", source_name ? ")" : "",
                            (unsigned) dest_port, dest_name ? " (" : "",
                            dest_name ? dest_name : "", dest_name ? ")" : "");
        }
    }
}
#else
#    define modem_debug_log_ipv4(...) ((void) 0)
#endif

typedef enum ResTypes {
    ResNONE,
    ResOK,
    ResERROR,
    ResCONNECT,
    ResRING,
    ResBUSY,
    ResNODIALTONE,
    ResNOCARRIER,
    ResNOANSWER
} ResTypes;

enum modem_types {
    MODEM_TYPE_NONE  = 0,
    MODEM_TYPE_SLIP  = 1,
    MODEM_TYPE_PPP   = 2,
    MODEM_TYPE_TCPIP = 3,
    MODEM_TYPE_CSLIP = 4,
    MODEM_TYPE_RAS   = 5
};

#ifdef ENABLE_MODEM_LOG
static const char *
modem_connection_type_name(int type)
{
    switch (type) {
        case MODEM_TYPE_NONE:  return "None";
        case MODEM_TYPE_SLIP:  return "SLIP";
        case MODEM_TYPE_PPP:   return "PPP";
        case MODEM_TYPE_TCPIP: return "TCP/IP";
        case MODEM_TYPE_CSLIP: return "CSLIP";
        case MODEM_TYPE_RAS:   return "Microsoft RAS";
        default:               return "Unknown";
    }
}
#endif

typedef enum modem_mode_t {
    MODEM_MODE_COMMAND = 0,
    MODEM_MODE_DATA    = 1,
    MODEM_MODE_FAX_TX  = 2,
    MODEM_MODE_FAX_WAIT = 3,
    MODEM_MODE_VOICE_TX = 4,
    MODEM_MODE_VOICE_RX = 5,
    MODEM_MODE_VOICE_TR = 6
} modem_mode_t;

typedef enum modem_fax_support_t {
    MODEM_FAX_SUPPORT_DISABLED = 0,
    MODEM_FAX_SUPPORT_CLASS_0  = 1 << 0,
    MODEM_FAX_SUPPORT_CLASS_1  = 1 << 1,
    MODEM_FAX_SUPPORT_CLASS_8  = 1 << 2
} modem_fax_support_t;

typedef enum modem_fax_transfer_status_t {
    MODEM_FAX_TRANSFER_ACTIVE,
    MODEM_FAX_TRANSFER_COMPLETE,
    MODEM_FAX_TRANSFER_ABORTED
} modem_fax_transfer_status_t;

typedef enum modem_slip_stage_t {
    MODEM_SLIP_STAGE_USERNAME,
    MODEM_SLIP_STAGE_PASSWORD
} modem_slip_stage_t;

#define COMMAND_BUFFER_SIZE 512
#define NUMBER_BUFFER_SIZE  128
#define MODEM_LOCAL_DIAL_SOUND_NUMBER "+18007160023"
#define MODEM_SOUND_DIAL_CONNECT_AFTER (1 << 9)
#define PHONEBOOK_SIZE      256
#define MODEM_REGS          100
#define MODEM_VOICE_PLAYBACK_BUFFER_SIZE (VOICE_LINE_RATE * 2)

typedef struct modem_phonebook_entry_t {
    char phone[NUMBER_BUFFER_SIZE];
    char address[NUMBER_BUFFER_SIZE];
} modem_phonebook_entry_t;

typedef struct modem_t {
    void      *log;
    uint8_t   mac[6];
    char_port_t *char_port;
    uint32_t char_status;
    bool char_dtr;
    bool char_rts;
    uint32_t  baudrate;

    modem_mode_t mode;

    uint8_t    esc_character_expected;
    pc_timer_t host_to_serial_timer;
    pc_timer_t dtr_timer;
    pc_timer_t cmdpause_timer;

    uint8_t  tx_pkt_ser_line[0x10000]; /* SLIP-encoded, or raw TCP/IP data. */
    uint32_t tx_count;

    Fifo8   rx_data; /* Data received from the network. */
    uint8_t reg[MODEM_REGS];

    Fifo8 data_pending; /* Data yet to be sent to the host. */
    Fifo8 char_tx_data; /* Bytes delivered by the char API read callback. */

    char     cmdbuf[COMMAND_BUFFER_SIZE];
    char     prevcmdbuf[COMMAND_BUFFER_SIZE];
    char     numberinprogress[NUMBER_BUFFER_SIZE];
    char     lastnumber[NUMBER_BUFFER_SIZE];
    char     dial_sound_number[NUMBER_BUFFER_SIZE];
    uint32_t cmdpos;
    uint32_t port;
    int      plusinc;
    int      flowcontrol;
    int      in_warmup;
    int      dtrmode;
    int      dcdmode;

    bool     connected;
    bool     ringing;
    bool     echo;
    bool     numericresponse;
    bool     tcpIpMode;
    bool     tcpIpConnInProgress;
    bool     cooldown;
    bool     telnet_mode;
    bool     dtrstate;
    uint32_t tcpIpConnCounter;
    bool     tcpIpSocketConnected;
    bool     call_progress_active;
    bool     call_answer_sound_started;
    uint32_t call_progress_elapsed_ms;
    uint32_t call_answer_at_ms;
    uint32_t call_connect_at_ms;
    int      pulse_dial;
    int      speaker_mode;
    int      speaker_level;
    modem_sound_t *sound;

    int doresponse;
    int cmdpause;
    int listen_port;
    int ringtimer;

    SOCKET serversocket;
    SOCKET clientsocket;
    SOCKET waitingclientsocket;

    struct {
        bool    binary[2];
        bool    echo[2];
        bool    supressGA[2];
        bool    timingMark[2];
        bool    inIAC;
        bool    recCommand;
        uint8_t command;
    } telClient;

    modem_phonebook_entry_t entries[PHONEBOOK_SIZE];
    uint32_t                entries_num;

    netcard_t *card;

    /* Protocol mode */
    int              connection_type;  /* MODEM_TYPE_SLIP, MODEM_TYPE_PPP, etc. */
    bool             ppp_active;       /* Currently in PPP mode */
    ppp_ctx_t       *ppp_ctx;          /* PPP context (when in PPP mode) */
    cslip_ctx_t     *cslip_ctx;        /* CSLIP context (when VJ compression enabled) */
    bool             cslip_enabled;    /* CSLIP VJ compression active */

    /* SLIP authentication */
    slip_auth_ctx_t *slip_auth_ctx;

    uint8_t fax_support;
    uint8_t fax_class;
    bool    fax_tx_pending_dle;
    bool    fax_rx_active;
    bool    fax_rx_pending_dle;
    bool    fax_rx_complete_pending;
    bool    fax_rx_frame_waiting;
    bool    fax_rx_aborted;
    uint16_t fax_wait_ticks;

    /* Rockwell voice mode; the TCP connection carries framed G.711 audio. */
    uint8_t voice_class;
    bool voice_dle_pending;
    bool voice_rx_draining;
    uint8_t voice_tx_frame[VOICE_FRAME_SAMPLES * 2];
    size_t voice_tx_count;
    voice_deframer_t voice_deframer;
    rv_adpcm_t voice_adpcm_tx;
    rv_adpcm_t voice_adpcm_rx;
    voice_resampler_t voice_resample_tx;
    voice_resampler_t voice_resample_rx;
    voice_resampler_t voice_capture_resample;
    voice_resampler_t voice_playback_resample;
    bool voice_capture_started;
    /* Voice frame handling and sound handlers run serially on the emulator
       event loop; no audio callback accesses this state on a worker thread. */
    int16_t voice_playback_buffer[MODEM_VOICE_PLAYBACK_BUFFER_SIZE];
    uint32_t voice_playback_head;
    uint32_t voice_playback_tail;
    uint32_t voice_playback_count;
    int voice_rate;
    int voice_bits;
    int voice_media_type;
    int voice_vls;
    bool voice_v253;
    bool voice_pcm_pending;
    uint8_t voice_pcm_lsb;
    voice_silence_t voice_silence;
    int voice_params[8]; /* VSS, VSP, VRN, VRA, VBT, VSD, VGT, VGR */
    int voice_vtd;
    int voice_v253_sensitivity;
    bool voice_silence_reported;

    /* PPP authentication settings */
    int              ppp_auth_type;    /* ppp_auth_type_t value from config */
    int              ppp_encryption;   /* Minimum required MPPE key strength */
    char             ppp_multilink_group[64];
    char             username[64];
    char             password[64];
    uint32_t         ppp_wins1;
    uint32_t         ppp_wins2;
    uint32_t         ppp_dns1;
    uint32_t         ppp_dns2;
    int              modem_identity;
} modem_t;

enum {
    MODEM_IDENTITY_GENERIC = 0,
    MODEM_IDENTITY_SUPRAEXPRESS = 1
};

#define MREG_AUTOANSWER_COUNT 0
#define MREG_RING_COUNT       1
#define MREG_ESCAPE_CHAR      2
#define MREG_CR_CHAR          3
#define MREG_LF_CHAR          4
#define MREG_BACKSPACE_CHAR   5
#define MREG_GUARD_TIME       12
#define MREG_DTR_DELAY        25

static void modem_do_command(modem_t *modem, int repeat);
static void modem_accept_incoming_call(modem_t *modem);
static void modem_enter_idle_state(modem_t *modem);
void modem_send_res(modem_t *modem, const ResTypes response);
static void modem_send_line(modem_t *modem, const char *line);
static void modem_send_number(modem_t *modem, uint32_t val);
static void fifo8_resize_2x(Fifo8 *fifo);
static uint32_t modem_scan_number(char **scan);

static void
modem_voice_frame_in(int type, const uint8_t *payload, size_t len, void *priv)
{
    modem_t *modem = (modem_t *) priv;
    int16_t  line[VOICE_FRAME_MAX];
    int16_t  serial[VOICE_FRAME_MAX * 2];
    uint8_t  encoded[VOICE_FRAME_MAX * 4];
    size_t   samples = 0;

    if (!len || !modem->voice_class
        || (modem->mode != MODEM_MODE_VOICE_RX && modem->mode != MODEM_MODE_VOICE_TR
            && !modem->voice_rx_draining))
        return;
    if (type == VOICE_FRAME_DTMF && (len == 1 || len == 3)) {
        double f1, f2;
        if (voice_dtmf_freqs((char) payload[0], &f1, &f2)) {
            voice_tone_t tone;
            uint32_t duration = len == 3
                                  ? (uint32_t) payload[1] | ((uint32_t) payload[2] << 8)
                                  : 100;
            if (duration < 40) duration = 40;
            if (duration > 5000) duration = 5000;
            voice_tone_start(&tone, f1, f2, duration, 6000.0);
            while (tone.left) {
                int16_t tone_samples[160] = { 0 };
                uint8_t audio[160];
                voice_tone_mix(&tone, tone_samples, sizeof(tone_samples) / sizeof(tone_samples[0]));
                for (size_t i = 0; i < sizeof(audio); i++)
                    audio[i] = voice_ulaw_encode(tone_samples[i]);
                modem_voice_frame_in(VOICE_FRAME_AUDIO, audio, sizeof(audio), priv);
            }
            for (int gap = 0; gap < 3; gap++) {
                uint8_t silence[160];
                memset(silence, voice_ulaw_encode(0), sizeof(silence));
                modem_voice_frame_in(VOICE_FRAME_AUDIO, silence, sizeof(silence), priv);
            }
        }
        return;
    }
    if (type == VOICE_FRAME_AUDIO || type == VOICE_FRAME_ALAW
        || type == VOICE_FRAME_S8 || type == VOICE_FRAME_U8) {
        if (len > VOICE_FRAME_MAX)
            return;
        for (size_t i = 0; i < len; i++) {
            if (type == VOICE_FRAME_AUDIO)
                line[i] = voice_ulaw_decode(payload[i]);
            else if (type == VOICE_FRAME_ALAW)
                line[i] = voice_alaw_decode(payload[i]);
            else if (type == VOICE_FRAME_S8)
                line[i] = (int16_t) ((int8_t) payload[i] * 256);
            else
                line[i] = (int16_t) (((int) payload[i] - 128) * 256);
        }
        samples = len;
    } else if (type == VOICE_FRAME_S16) {
        if ((len & 1) || len / 2 > VOICE_FRAME_MAX)
            return;
        samples = len / 2;
        for (size_t i = 0; i < samples; i++)
            line[i] = (int16_t) ((uint16_t) payload[i * 2]
                                 | ((uint16_t) payload[i * 2 + 1] << 8));
    } else
        return;
    {
        int16_t playback[VOICE_FRAME_MAX * 6];
        size_t playback_samples = voice_resample(&modem->voice_playback_resample,
                                                  line, samples, playback,
                                                  sizeof(playback) / sizeof(playback[0]));
        for (size_t i = 0; i < playback_samples; i++) {
            /* Keep playback latency bounded: discard the oldest sample on overrun. */
            if (modem->voice_playback_count == MODEM_VOICE_PLAYBACK_BUFFER_SIZE) {
                modem->voice_playback_tail = (modem->voice_playback_tail + 1)
                                           % MODEM_VOICE_PLAYBACK_BUFFER_SIZE;
                modem->voice_playback_count--;
            }
            modem->voice_playback_buffer[modem->voice_playback_head] = playback[i];
            modem->voice_playback_head = (modem->voice_playback_head + 1)
                                       % MODEM_VOICE_PLAYBACK_BUFFER_SIZE;
            modem->voice_playback_count++;
        }
    }
    voice_silence_feed(&modem->voice_silence, line, samples);
    if (modem->voice_silence.quiet_ms == 0)
        modem->voice_silence_reported = false;
    if (modem->voice_params[1] > 0
        && modem->voice_silence.quiet_ms >= (uint32_t) modem->voice_params[1] * 100u) {
        if (!modem->voice_silence_reported) {
            while (fifo8_num_free(&modem->rx_data) < 2)
                fifo8_resize_2x(&modem->rx_data);
            fifo8_push(&modem->rx_data, 0x10);
            fifo8_push(&modem->rx_data,
                       modem->voice_v253 && modem->voice_silence.heard_ms ? 'q' : 's');
            modem->voice_silence_reported = true;
        }
        if (modem->voice_params[5])
            return;
    }
    samples = voice_resample(&modem->voice_resample_rx, line, samples, serial,
                             sizeof(serial) / sizeof(serial[0]));
    if (modem->voice_v253 || modem->voice_media_type != VOICE_FRAME_AUDIO) {
        size_t bytes = 0;
        for (size_t i = 0; i < samples; i++) {
            if (modem->voice_media_type == VOICE_FRAME_AUDIO)
                encoded[bytes++] = voice_ulaw_encode(serial[i]);
            else if (modem->voice_media_type == VOICE_FRAME_ALAW)
                encoded[bytes++] = voice_alaw_encode(serial[i]);
            else if (modem->voice_media_type == VOICE_FRAME_S8)
                encoded[bytes++] = (uint8_t) (serial[i] >> 8);
            else if (modem->voice_media_type == VOICE_FRAME_U8)
                encoded[bytes++] = (uint8_t) ((serial[i] >> 8) + 128);
            else {
                encoded[bytes++] = (uint8_t) serial[i];
                encoded[bytes++] = (uint8_t) ((uint16_t) serial[i] >> 8);
            }
        }
        samples = bytes;
    } else
        samples = rv_adpcm_encode(&modem->voice_adpcm_rx, serial, samples,
                                  encoded, sizeof(encoded));
    while (samples * 2 > fifo8_num_free(&modem->rx_data))
        fifo8_resize_2x(&modem->rx_data);
    for (size_t i = 0; i < samples; i++) {
        fifo8_push(&modem->rx_data, encoded[i]);
        if (encoded[i] == 0x10)
            fifo8_push(&modem->rx_data, encoded[i]);
    }
}

static void
modem_voice_send_audio(modem_t *modem, const uint8_t *audio, size_t len)
{
    uint8_t frame[VOICE_FRAME_HDR + VOICE_FRAME_MAX];
    size_t  frame_len;

    if (!modem->connected || !modem->tcpIpMode || !len || len > VOICE_FRAME_MAX)
        return;
    frame_len = voice_frame(frame, modem->voice_media_type, audio, len);
    if (frame_len > sizeof(modem->tx_pkt_ser_line) - modem->tx_count)
        return;
    memcpy(modem->tx_pkt_ser_line + modem->tx_count, frame, frame_len);
    modem->tx_count += (uint32_t) frame_len;
}

static void
modem_voice_capture(int16_t *buffer, int len, void *priv)
{
    modem_t *modem = (modem_t *) priv;
    int16_t line[VOICE_FRAME_MAX];
    uint8_t audio[VOICE_FRAME_MAX];
    size_t  line_count = 0;

    if (!modem->voice_capture_started || len <= 0
        || (modem->mode != MODEM_MODE_VOICE_TX && modem->mode != MODEM_MODE_VOICE_TR))
        return;

    for (int pos = 0; pos < len;) {
        size_t count = (size_t) MIN(len - pos, 128);
        size_t samples = voice_resample(&modem->voice_capture_resample,
                                        buffer + pos, count, line + line_count,
                                        (sizeof(line) / sizeof(line[0])) - line_count);
        line_count += samples;
        pos += (int) count;
    }

    size_t bytes = 0;
    for (size_t i = 0; i < line_count; i++) {
        if (modem->voice_media_type == VOICE_FRAME_AUDIO)
            audio[bytes++] = voice_ulaw_encode(line[i]);
        else if (modem->voice_media_type == VOICE_FRAME_ALAW)
            audio[bytes++] = voice_alaw_encode(line[i]);
        else if (modem->voice_media_type == VOICE_FRAME_S8)
            audio[bytes++] = (uint8_t) (line[i] >> 8);
        else if (modem->voice_media_type == VOICE_FRAME_U8)
            audio[bytes++] = (uint8_t) ((line[i] >> 8) + 128);
        else {
            audio[bytes++] = (uint8_t) line[i];
            audio[bytes++] = (uint8_t) ((uint16_t) line[i] >> 8);
        }
    }
    if (bytes)
        modem_voice_send_audio(modem, audio, bytes);
}

static void
modem_voice_capture_set(modem_t *modem, bool enabled)
{
    if (modem->voice_capture_started == enabled)
        return;

    modem->voice_capture_started = enabled;
    if (enabled) {
        sound_in_start_input();
        int rate = al_capture_get_rate();
        if (rate <= 0)
            rate = SOUND_FREQ;
        voice_resampler_init(&modem->voice_capture_resample,
                             (uint32_t) rate, VOICE_LINE_RATE);
    } else
        sound_in_stop_input();
}

static void
modem_voice_playback(int32_t *buffer, uint16_t len, void *priv)
{
    modem_t *modem = (modem_t *) priv;

    /* An empty queue is an audio underrun; emit silence for the missing sample. */
    for (uint16_t i = 0; i < len; i++) {
        int32_t sample = 0;
        if (modem->voice_playback_count) {
            sample = modem->voice_playback_buffer[modem->voice_playback_tail] / 2;
            modem->voice_playback_tail = (modem->voice_playback_tail + 1)
                                       % MODEM_VOICE_PLAYBACK_BUFFER_SIZE;
            modem->voice_playback_count--;
        }
        buffer[i * 2] += sample;
        buffer[i * 2 + 1] += sample;
    }
}

static void
modem_voice_send_digit(modem_t *modem, char digit, uint16_t duration_ms)
{
    uint8_t frame[VOICE_FRAME_HDR + 3];
    uint8_t event[3] = { (uint8_t) digit, (uint8_t) duration_ms,
                         (uint8_t) (duration_ms >> 8) };
    size_t frame_len;

    if (!modem->connected || !modem->tcpIpMode || !modem->voice_class)
        return;
    frame_len = voice_frame(frame, VOICE_FRAME_DTMF, event, sizeof(event));
    if (frame_len > sizeof(modem->tx_pkt_ser_line) - modem->tx_count)
        return;
    memcpy(modem->tx_pkt_ser_line + modem->tx_count, frame, frame_len);
    modem->tx_count += (uint32_t) frame_len;
}

static bool
modem_voice_send_tone(modem_t *modem, double f1, double f2, uint32_t duration_ms)
{
    voice_tone_t tone;
    uint8_t samples[160];
    uint8_t frame[VOICE_FRAME_HDR + sizeof(samples)];

    if (!modem->connected || !modem->tcpIpMode || !modem->voice_class)
        return false;
    if (duration_ms > 5000)
        duration_ms = 5000;
    voice_tone_start(&tone, f1, f2, duration_ms, 6000.0);
    while (tone.left) {
        int16_t pcm[sizeof(samples)];
        size_t frame_len;
        memset(pcm, 0, sizeof(pcm));
        voice_tone_mix(&tone, pcm, sizeof(pcm) / sizeof(pcm[0]));
        for (size_t i = 0; i < sizeof(samples); i++)
            samples[i] = voice_ulaw_encode(pcm[i]);
        frame_len = voice_frame(frame, VOICE_FRAME_AUDIO, samples, sizeof(samples));
        if (frame_len > sizeof(modem->tx_pkt_ser_line) - modem->tx_count)
            return false;
        memcpy(modem->tx_pkt_ser_line + modem->tx_count, frame, frame_len);
        modem->tx_count += (uint32_t) frame_len;
    }
    return true;
}

static bool
modem_voice_send_vts(modem_t *modem, const char *sequence, uint32_t unit_ms,
                     uint32_t default_ms)
{
    struct vts_event { char digit; double f1, f2; uint32_t duration; bool tone; } events[64];
    size_t count = 0;
    uint32_t total_ms = 0;
    const char *p = sequence;

    if (!modem->connected || !modem->tcpIpMode || !modem->voice_class || !*p)
        return false;
    while (*p) {
        struct vts_event *event;
        if (count == sizeof(events) / sizeof(events[0]))
            return false;
        event = &events[count];
        memset(event, 0, sizeof(*event));
        if (*p == '{') {
            char digit;
            char *end;
            uint32_t length;
            p++;
            digit = *p++;
            if (!voice_dtmf_freqs(digit, &event->f1, &event->f2) || *p++ != ',')
                return false;
            end = (char *) p;
            length = modem_scan_number(&end);
            if (end == p || *end != '}' || length > 5000u / unit_ms) return false;
            p = end + 1;
            event->digit = digit;
            event->duration = length ? length * unit_ms : default_ms;
        } else if (*p == '[') {
            char *end;
            uint32_t f1, f2, length;
            p++;
            end = (char *) p;
            f1 = modem_scan_number(&end);
            if (end == p || *end++ != ',') return false;
            p = end;
            end = (char *) p;
            f2 = modem_scan_number(&end);
            if (end == p || *end++ != ',') return false;
            p = end;
            end = (char *) p;
            length = modem_scan_number(&end);
            if (end == p || *end != ']' || f1 > 4000 || f2 > 4000
                || length > 5000u / unit_ms) return false;
            p = end + 1;
            event->tone = true;
            event->f1 = f1;
            event->f2 = f2;
            event->duration = length * unit_ms;
        } else {
            event->digit = *p++;
            if (!voice_dtmf_freqs(event->digit, &event->f1, &event->f2))
                return false;
            event->duration = default_ms;
        }
        if (event->duration > 5000)
            event->duration = 5000;
        if (!event->duration) event->duration = 100;
        total_ms += event->duration;
        if (total_ms > 5000) return false;
        count++;
    }

    for (size_t i = 0; i < count; i++) {
        struct vts_event *event = &events[i];
        if (event->tone) {
            if (!modem_voice_send_tone(modem, event->f1, event->f2, event->duration))
                return false;
        } else
            modem_voice_send_digit(modem, event->digit, (uint16_t) event->duration);
    }
    return true;
}

static void
modem_voice_result(modem_t *modem, const char *result)
{
    if (modem->doresponse == 1 || modem->doresponse == 2)
        return;
    if (modem->numericresponse)
        modem_send_number(modem, 1);
    else
        modem_send_line(modem, result);
}

static void
modem_voice_tx_byte(modem_t *modem, uint8_t byte)
{
    int16_t decoded[16];
    int16_t line[64];
    size_t  n, m;

    modem->cmdpause = 0;
    if (modem->voice_dle_pending) {
        modem->voice_dle_pending = false;
        if (byte == '^' && modem->mode == MODEM_MODE_VOICE_TR) {
            if (modem->voice_tx_count)
                modem_voice_send_audio(modem, modem->voice_tx_frame, modem->voice_tx_count);
            modem->voice_tx_count = 0;
            modem->mode = MODEM_MODE_COMMAND;
            modem_voice_capture_set(modem, false);
            modem->voice_rx_draining = true;
            modem_send_res(modem, ResOK);
            return;
        }
        if (byte == 0x03) {
            if (modem->voice_tx_count)
                modem_voice_send_audio(modem, modem->voice_tx_frame, modem->voice_tx_count);
            modem->voice_tx_count = 0;
            modem->mode = MODEM_MODE_COMMAND;
            modem_voice_capture_set(modem, false);
            modem_send_res(modem, ResOK);
            return;
        }
        if (byte == 0x18) {
            modem->voice_tx_count = 0;
            modem->mode = MODEM_MODE_COMMAND;
            modem_voice_capture_set(modem, false);
            modem_send_res(modem, ResOK);
            return;
        }
        if (byte != 0x10)
            return;
    } else if (byte == 0x10) {
        modem->voice_dle_pending = true;
        return;
    }

    if (!modem->voice_v253 && modem->voice_media_type == VOICE_FRAME_AUDIO) {
        n = rv_adpcm_decode(&modem->voice_adpcm_tx, &byte, 1, decoded,
                            sizeof(decoded) / sizeof(decoded[0]));
    } else if (modem->voice_media_type == VOICE_FRAME_S16) {
        if (!modem->voice_pcm_pending) {
            modem->voice_pcm_lsb = byte;
            modem->voice_pcm_pending = true;
            return;
        }
        decoded[0] = (int16_t) ((uint16_t) modem->voice_pcm_lsb | ((uint16_t) byte << 8));
        modem->voice_pcm_pending = false;
        n = 1;
    } else {
        decoded[0] = modem->voice_media_type == VOICE_FRAME_AUDIO ? voice_ulaw_decode(byte)
                  : modem->voice_media_type == VOICE_FRAME_ALAW ? voice_alaw_decode(byte)
                  : modem->voice_media_type == VOICE_FRAME_S8 ? (int16_t) ((int8_t) byte * 256)
                  : (int16_t) (((int) byte - 128) * 256);
        n = 1;
    }
    m = voice_resample(&modem->voice_resample_tx, decoded, n, line,
                       sizeof(line) / sizeof(line[0]));
    for (size_t i = 0; i < m; i++) {
        if (modem->voice_media_type == VOICE_FRAME_AUDIO)
            modem->voice_tx_frame[modem->voice_tx_count++] = voice_ulaw_encode(line[i]);
        else if (modem->voice_media_type == VOICE_FRAME_ALAW)
            modem->voice_tx_frame[modem->voice_tx_count++] = voice_alaw_encode(line[i]);
        else if (modem->voice_media_type == VOICE_FRAME_S8)
            modem->voice_tx_frame[modem->voice_tx_count++] = (uint8_t) (line[i] >> 8);
        else if (modem->voice_media_type == VOICE_FRAME_U8)
            modem->voice_tx_frame[modem->voice_tx_count++] = (uint8_t) ((line[i] >> 8) + 128);
        else {
            modem->voice_tx_frame[modem->voice_tx_count++] = (uint8_t) line[i];
            modem->voice_tx_frame[modem->voice_tx_count++] = (uint8_t) ((uint16_t) line[i] >> 8);
        }
        if (modem->voice_tx_count >= (modem->voice_media_type == VOICE_FRAME_S16
                                          ? VOICE_FRAME_SAMPLES * 2 : VOICE_FRAME_SAMPLES)) {
            modem_voice_send_audio(modem, modem->voice_tx_frame, modem->voice_tx_count);
            modem->voice_tx_count = 0;
        }
    }
}

static bool
modem_voice_start(modem_t *modem, modem_mode_t mode)
{
    if (!modem->voice_class || !modem->connected || !modem->tcpIpMode
        || modem->telnet_mode)
        return false;

    memset(&modem->voice_deframer, 0, sizeof(modem->voice_deframer));
    rv_adpcm_init(&modem->voice_adpcm_tx, modem->voice_bits);
    rv_adpcm_init(&modem->voice_adpcm_rx, modem->voice_bits);
    voice_resampler_init(&modem->voice_resample_tx,
                         (uint32_t) modem->voice_rate, VOICE_LINE_RATE);
    voice_resampler_init(&modem->voice_resample_rx,
                         VOICE_LINE_RATE, (uint32_t) modem->voice_rate);
    voice_resampler_init(&modem->voice_playback_resample,
                         VOICE_LINE_RATE, SOUND_FREQ);
    modem->voice_playback_head = modem->voice_playback_tail = modem->voice_playback_count = 0;
    modem->voice_tx_count = 0;
    modem->voice_dle_pending = false;
    modem->voice_pcm_pending = false;
    voice_silence_init(&modem->voice_silence, modem->voice_params[0]);
    modem->voice_silence_reported = false;
    modem->mode = mode;
    modem->voice_rx_draining = mode == MODEM_MODE_VOICE_RX || mode == MODEM_MODE_VOICE_TR;
    modem_voice_capture_set(modem, mode == MODEM_MODE_VOICE_TX || mode == MODEM_MODE_VOICE_TR);
    modem_voice_result(modem, "CONNECT");
    return true;
}

static void
modem_voice_set_line(modem_t *modem, int vls)
{
    modem->voice_vls = vls;
    if (vls == 0 && modem->connected) {
        modem_send_res(modem, ResNOCARRIER);
        modem_enter_idle_state(modem);
    } else if (vls != 0 && !modem->connected
               && modem->waitingclientsocket != (SOCKET) -1) {
        modem_accept_incoming_call(modem);
    }
}

static void
modem_ppp_check_dead(modem_t *modem)
{
    if (modem->ppp_active && modem->ppp_ctx
        && modem->ppp_ctx->state == PPP_STATE_DEAD) {
        modem_log(modem->log, "PPP session ended\n");
        modem_send_res(modem, ResNOCARRIER);
        modem_enter_idle_state(modem);
    }
}

extern ssize_t local_getline(char **buf, size_t *bufsiz, FILE *fp);

// https://stackoverflow.com/a/122974
char *
trim(char *str)
{
    size_t len    = 0;
    char  *frontp = str;
    char  *endp   = NULL;

    if (str == NULL) {
        return NULL;
    }
    if (str[0] == '\0') {
        return str;
    }

    len  = strlen(str);
    endp = str + len;

    /* Move the front and back pointers to address the first non-whitespace
     * characters from each end.
     */
    while (isspace((unsigned char) *frontp)) {
        ++frontp;
    }
    if (endp != frontp) {
        while (isspace((unsigned char) *(--endp)) && endp != frontp) { }
    }

    if (frontp != str && endp == frontp)
        *str = '\0';
    else if (str + len - 1 != endp)
        *(endp + 1) = '\0';

    /* Shift the string so that it starts at str so that if it's dynamically
     * allocated, we can still free it on the returned pointer.  Note the reuse
     * of endp to mean the front of the string buffer now.
     */
    endp = str;
    if (frontp != str) {
        while (*frontp) {
            *endp++ = *frontp++;
        }
        *endp = '\0';
    }

    return str;
}

static void
modem_read_phonebook_file(modem_t *modem, const char *path)
{
    char  *buf  = NULL;
    char  *buf2 = NULL;
    size_t size = 0;

    modem->entries_num = 0;

    if (!path || path[0] == '\0')
        return;

    FILE *file = plat_fopen(path, "r");
    if (!file)
        return;

    modem_log(modem->log, "Modem: Reading phone book file %s...\n", path);
    while (local_getline(&buf, &size, file) != -1) {
        modem_phonebook_entry_t entry = { { 0 }, { 0 } };
        buf[strcspn(buf, "\r\n")]     = '\0';

        /* Remove surrounding whitespace from the input line and find the address part. */
        buf  = trim(buf);
        buf2 = &buf[strcspn(buf, " \t")];

        /* Remove surrounding whitespace and any extra text from the address part, then store it. */
        buf2                       = trim(buf2);
        buf2[strcspn(buf2, " \t")] = '\0';
        strncpy(entry.address, buf2, sizeof(entry.address) - 1);

        /* Split the line to get the phone number part, then store it. */
        buf2[0] = '\0';
        strncpy(entry.phone, buf, sizeof(entry.phone) - 1);

        if ((entry.phone[0] == '\0') || (entry.address[0] == '\0')) {
            /* Appears to be a bad line. */
            modem_log(modem->log, "Modem: Skipped a bad line\n");
            continue;
        }

        if (strspn(entry.phone, "01234567890*=,;#+>") != strlen(entry.phone)) {
            /* Invalid characters. */
            modem_log(modem->log, "Modem: Invalid character in phone number %s\n", entry.phone);
            continue;
        }

        modem_log(modem->log, "Modem: Mapped phone number %s to address %s\n", entry.phone, entry.address);
        modem->entries[modem->entries_num++] = entry;
        if (modem->entries_num >= PHONEBOOK_SIZE)
            break;
    }
    fclose(file);
}

static void
modem_echo(modem_t *modem, uint8_t c)
{
    if (modem->echo && fifo8_num_free(&modem->data_pending))
        fifo8_push(&modem->data_pending, c);
}

static uint32_t
modem_scan_number(char **scan)
{
    char     c   = 0;
    uint32_t ret = 0;
    while (1) {
        c = **scan;
        if (c == 0)
            break;
        if (c >= '0' && c <= '9') {
            ret *= 10;
            ret += c - '0';
            *scan = *scan + 1;
        } else
            break;
    }
    return ret;
}

static uint8_t
modem_fetch_character(char **scan)
{
    uint8_t c = **scan;
    *scan     = *scan + 1;
    return c;
}

static void
modem_speed_changed(void *priv)
{
    modem_t *dev = (modem_t *) priv;
    if (!dev)
        return;

    timer_stop(&dev->host_to_serial_timer);
    /* FIXME: do something to dev->baudrate */
    timer_on_auto(&dev->host_to_serial_timer, (1000000.0 / (double) dev->baudrate) * 9);
}

static void
modem_send_line(modem_t *modem, const char *line)
{
    fifo8_push(&modem->data_pending, modem->reg[MREG_CR_CHAR]);
    fifo8_push(&modem->data_pending, modem->reg[MREG_LF_CHAR]);
    fifo8_push_all(&modem->data_pending, (uint8_t *) line, strlen(line));
    fifo8_push(&modem->data_pending, modem->reg[MREG_CR_CHAR]);
    fifo8_push(&modem->data_pending, modem->reg[MREG_LF_CHAR]);
}

static void
modem_send_number(modem_t *modem, uint32_t val)
{
    fifo8_push(&modem->data_pending, modem->reg[MREG_CR_CHAR]);
    fifo8_push(&modem->data_pending, modem->reg[MREG_LF_CHAR]);

    fifo8_push(&modem->data_pending, val / 100 + '0');
    val = val % 100;
    fifo8_push(&modem->data_pending, val / 10 + '0');
    val = val % 10;
    fifo8_push(&modem->data_pending, val + '0');

    fifo8_push(&modem->data_pending, modem->reg[MREG_CR_CHAR]);
    fifo8_push(&modem->data_pending, modem->reg[MREG_LF_CHAR]);
}

static void
modem_ppp_serial_push(void *priv, const uint8_t *data, int len)
{
    modem_t *modem = (modem_t *) priv;

    while (len >= (int) fifo8_num_free(&modem->rx_data))
        fifo8_resize_2x(&modem->rx_data);

    fifo8_push_all(&modem->rx_data, (uint8_t *) data, (uint32_t) len);
}

static void
modem_ppp_network_send_ip(void *priv, const uint8_t *ip_pkt, int len)
{
    modem_t *modem = (modem_t *) priv;
    uint8_t *buf   = calloc(len + 14, 1);

    if (!buf)
        return;

    modem_debug_log_ipv4(modem->log, "PPP modem->network", ip_pkt, len);

    buf[0] = buf[1] = buf[2] = buf[3] = buf[4] = buf[5] = 0xFF;
    buf[6] = buf[7] = buf[8] = buf[9] = buf[10] = buf[11] = 0xFC;
    buf[12] = 0x08;
    buf[13] = 0x00;
    memcpy(buf + 14, ip_pkt, len);
    network_tx(modem->card, buf, len + 14);
    free(buf);
}

static void
process_tx_packet(modem_t *modem, uint8_t *p, uint32_t len)
{
    uint8_t *processed_tx_packet = calloc(len, 1);
    size_t   received;
    bool     decoded_ok;

    if (!processed_tx_packet)
        return;

    decoded_ok = slip_decode_frame_logged(modem->log, p, len, processed_tx_packet,
                                          len, &received);
    if (!decoded_ok || received == 0) {
        free(processed_tx_packet);
        return;
    }

    {
        uint8_t *ip_data    = processed_tx_packet;
        int      ip_len     = (int) received;
        uint8_t *decomp_buf = NULL;

        /* CSLIP: VJ decompress if enabled */
        if (modem->cslip_enabled && modem->cslip_ctx) {
            decomp_buf = calloc((size_t) ip_len + VJ_MAX_HDR, 1);
            if (!decomp_buf) {
                free(processed_tx_packet);
                return;
            }
            ip_len = cslip_decompress_packet(modem->cslip_ctx, ip_data, ip_len,
                                             decomp_buf);
            if (ip_len <= 0) {
                free(decomp_buf);
                free(processed_tx_packet);
                return;
            }
            ip_data = decomp_buf;
        }
        modem_debug_log_ipv4(modem->log, "SLIP modem->network", ip_data, ip_len);

        uint8_t *buf = calloc(ip_len + 14, 1);
        if (!buf) {
            free(decomp_buf);
            free(processed_tx_packet);
            return;
        }

        buf[0] = buf[1] = buf[2] = buf[3] = buf[4] = buf[5] = 0xFF;
        buf[6] = buf[7] = buf[8] = buf[9] = buf[10] = buf[11] = 0xFC;
        buf[12]                                               = 0x08;
        buf[13]                                               = 0x00;
        memcpy(buf + 14, ip_data, ip_len);
        network_tx(modem->card, buf, ip_len + 14);
        free(buf);
        if (decomp_buf)
            free(decomp_buf);
    }
    free(processed_tx_packet);
    return;
}

static modem_fax_transfer_status_t
modem_fax_transfer_status(bool *pending_dle, uint8_t data)
{
    if (*pending_dle) {
        *pending_dle = false;
        if (data == 0x03)
            return MODEM_FAX_TRANSFER_COMPLETE;
        if (data == 0x18)
            return MODEM_FAX_TRANSFER_ABORTED;
        if (data == 0x10)
            return MODEM_FAX_TRANSFER_ACTIVE;
    }

    if (data == 0x10)
        *pending_dle = true;

    return MODEM_FAX_TRANSFER_ACTIVE;
}

static bool
modem_fax_rate_supported(uint32_t rate)
{
    switch (rate) {
        case 3:
        case 24:
        case 48:
        case 72:
        case 73:
        case 74:
        case 96:
        case 97:
        case 98:
        case 121:
        case 122:
        case 145:
        case 146:
            return true;
        default:
            return false;
    }
}

static void
modem_data_mode_process_byte(modem_t *modem, uint8_t data)
{
    if (modem->mode != MODEM_MODE_FAX_TX && !modem->fax_rx_active
        && !modem->fax_rx_complete_pending && modem->reg[MREG_ESCAPE_CHAR] <= 127) {
        if (modem->plusinc >= 1 && modem->plusinc <= 3 && modem->reg[MREG_ESCAPE_CHAR] == data) {
            modem->plusinc++;
        } else {
            modem->plusinc = 0;
        }
    }
    modem->cmdpause = 0;

    if (modem->slip_auth_ctx && modem->slip_auth_ctx->ignore_lf) {
        if (data == '\n') {
            slip_auth_rx_byte(modem->slip_auth_ctx, data);
            return;
        }
        modem->slip_auth_ctx->ignore_lf = false;
    }

    /* SLIP auth in progress - route bytes to auth handler */
    if (modem->slip_auth_ctx && modem->slip_auth_ctx->active) {
        if (slip_auth_rx_byte(modem->slip_auth_ctx, data)) {
            if (modem->slip_auth_ctx->state == SLIP_AUTH_DONE_OK) {
                /* Auth succeeded - enter SLIP data mode */
                modem_log(modem->log, "SLIP auth succeeded, entering data mode\n");
            } else {
                /* Auth failed - hang up */
                modem_log(modem->log, "SLIP auth failed, disconnecting\n");
                modem_enter_idle_state(modem);
            }
        }
        return;
    }

    /* PPP mode - route bytes to PPP HDLC framing */
    if (modem->ppp_active && modem->ppp_ctx) {
        ppp_rx_byte(modem->ppp_ctx, data);
        modem_ppp_check_dead(modem);
        return;
    }

    if (modem->tx_count < 0x10000 && modem->connected) {
        modem->tx_pkt_ser_line[modem->tx_count++] = data;
        if (data == SLIP_END && !modem->tcpIpMode) {
            process_tx_packet(modem, modem->tx_pkt_ser_line, (uint32_t) modem->tx_count);
            modem->tx_count = 0;
        }
    }
}

static void
host_to_modem_cb(void *priv)
{
    modem_t *modem = (modem_t *) priv;
    if (modem->in_warmup || !modem->char_port
        || !fifo8_num_free(&modem->char_tx_data)
        || (modem->flowcontrol == 3 && !modem->char_rts))
        goto no_write_to_machine;

    if ((modem->mode == MODEM_MODE_DATA || modem->mode == MODEM_MODE_VOICE_RX
         || modem->mode == MODEM_MODE_VOICE_TR || modem->voice_rx_draining)
        && fifo8_num_used(&modem->rx_data)
        && !modem->cooldown
        && (!(modem->fax_rx_active || modem->fax_rx_complete_pending)
            || !fifo8_num_used(&modem->data_pending))) {
        fifo8_push(&modem->char_tx_data, fifo8_pop(&modem->rx_data));
    } else if (fifo8_num_used(&modem->data_pending)) {
        uint8_t val = fifo8_pop(&modem->data_pending);
        fifo8_push(&modem->char_tx_data, val);
    }

    if (modem->fax_rx_complete_pending && !fifo8_num_used(&modem->rx_data)) {
        modem->fax_rx_complete_pending = false;
        modem->mode = MODEM_MODE_COMMAND;
        modem->cmdpause = 0;
        modem->plusinc = 0;
        modem_send_res(modem, modem->fax_rx_aborted ? ResNOCARRIER : ResOK);
        modem->fax_rx_aborted = false;
    }

    if (modem->voice_rx_draining && !fifo8_num_used(&modem->rx_data))
        modem->voice_rx_draining = false;

    if (fifo8_num_used(&modem->data_pending) == 0) {
        modem->cooldown = false;
    }

no_write_to_machine:
    if (modem->mode == MODEM_MODE_VOICE_RX || modem->mode == MODEM_MODE_VOICE_TR
        || modem->voice_rx_draining) {
        const int bits_per_sample = (!modem->voice_v253
                                     && modem->voice_media_type == VOICE_FRAME_AUDIO)
                                      ? modem->voice_bits
                                      : (modem->voice_media_type == VOICE_FRAME_S16 ? 16 : 8);
        const double voice_byte_us = 1000000.0 * 8.0
                                   / ((double) modem->voice_rate * (double) bits_per_sample);
        timer_on_auto(&modem->host_to_serial_timer, voice_byte_us);
    } else
        timer_on_auto(&modem->host_to_serial_timer,
                      (1000000.0 / (double) modem->baudrate) * (double) 9);
}

static void
modem_write_byte(void *priv, uint8_t txval)
{
    modem_t *modem = (modem_t *) priv;

    if (modem->mode == MODEM_MODE_COMMAND) {
        if (modem->cmdpos < 2) {
            // Ignore everything until we see "AT" sequence.
            if (modem->cmdpos == 0 && toupper(txval) != 'A') {
                return;
            }

            if (modem->cmdpos == 1 && toupper(txval) != 'T') {
                if (txval == '/') {
                    // Repeat the last command.
                    modem_echo(modem, txval);
                    modem_log(modem->log, "Repeat last command (%s)\n", modem->prevcmdbuf);
                    modem_do_command(modem, 1);
                } else {
                    modem_echo(modem, modem->reg[MREG_BACKSPACE_CHAR]);
                    modem->cmdpos = 0;
                }
                return;
            }
        } else {
            // Now entering command.
            if (txval == modem->reg[MREG_BACKSPACE_CHAR]) {
                if (modem->cmdpos > 2) {
                    modem_echo(modem, txval);
                    modem->cmdpos--;
                }
                return;
            }

            if (txval == modem->reg[MREG_LF_CHAR]) {
                return; // Real modem doesn't seem to skip this?
            }

            if (txval == modem->reg[MREG_CR_CHAR]) {
                modem_echo(modem, txval);
                modem_do_command(modem, 0);
                return;
            }
        }

        if (modem->cmdpos < 99) {
            modem_echo(modem, txval);
            modem->cmdbuf[modem->cmdpos] = txval;
            modem->cmdpos++;
        }
    } else if (modem->mode != MODEM_MODE_FAX_WAIT) {
        if (modem->mode == MODEM_MODE_VOICE_TX || modem->mode == MODEM_MODE_VOICE_TR)
            modem_voice_tx_byte(modem, txval);
        else if (modem->mode == MODEM_MODE_VOICE_RX) {
            uint8_t tail[4];
            size_t  tail_len = rv_adpcm_flush(&modem->voice_adpcm_rx, tail, sizeof(tail));

            /* Rockwell stops receive mode when the DTE sends any byte. */
            while (fifo8_num_free(&modem->rx_data) < tail_len * 2 + 2)
                fifo8_resize_2x(&modem->rx_data);
            for (size_t i = 0; i < tail_len; i++) {
                fifo8_push(&modem->rx_data, tail[i]);
                if (tail[i] == 0x10)
                    fifo8_push(&modem->rx_data, tail[i]);
            }
            fifo8_push(&modem->rx_data, 0x10);
            fifo8_push(&modem->rx_data, 0x03);
            modem->mode = MODEM_MODE_COMMAND;
            modem_send_res(modem, ResOK);
        } else
            modem_data_mode_process_byte(modem, txval);
        if (modem->mode == MODEM_MODE_FAX_TX) {
            const modem_fax_transfer_status_t status =
                modem_fax_transfer_status(&modem->fax_tx_pending_dle, txval);
            if (status != MODEM_FAX_TRANSFER_ACTIVE) {
                modem->mode = MODEM_MODE_COMMAND;
                modem->cmdpause = 0;
                modem->plusinc = 0;
                modem_send_res(modem, status == MODEM_FAX_TRANSFER_ABORTED
                                          ? ResNOCARRIER : ResOK);
            }
        }
    }
}

void
modem_send_res(modem_t *modem, const ResTypes response)
{
    char        response_str_connect[256] = { 0 };
    const char *response_str              = NULL;
    uint32_t    code                      = -1;

    snprintf(response_str_connect, sizeof(response_str_connect), "CONNECT %u", modem->baudrate);

    switch (response) {
        case ResOK:
            code         = 0;
            response_str = "OK";
            break;
        case ResCONNECT:
            code         = 1;
            response_str = response_str_connect;
            break;
        case ResRING:
            code         = 2;
            response_str = "RING";
            break;
        case ResNOCARRIER:
            code         = 3;
            response_str = "NO CARRIER";
            break;
        case ResERROR:
            code         = 4;
            response_str = "ERROR";
            break;
        case ResNODIALTONE:
            code         = 6;
            response_str = "NO DIALTONE";
            break;
        case ResBUSY:
            code         = 7;
            response_str = "BUSY";
            break;
        case ResNOANSWER:
            code         = 8;
            response_str = "NO ANSWER";
            break;
        case ResNONE:
            return;
    }

    if (modem->doresponse != 1) {
        if (modem->doresponse == 2 && (response == ResRING || response == ResCONNECT || response == ResNOCARRIER)) {
            return;
        }
        modem_log(modem->log, "Modem response: %s\n", response_str);
        if (modem->numericresponse && code != ~0) {
            modem_send_number(modem, code);
        } else if (response_str != NULL) {
            modem_send_line(modem, response_str);
        }
    }
}

void
modem_enter_idle_state(modem_t *modem)
{
    modem_voice_capture_set(modem, false);
    modem->voice_playback_head = modem->voice_playback_tail = modem->voice_playback_count = 0;
    modem_sound_event(modem->sound, MODEM_SOUND_HANGUP, NULL, 0);
    timer_disable(&modem->dtr_timer);
    modem->connected           = false;
    modem->ringing             = false;
    modem->mode                = MODEM_MODE_COMMAND;
    modem->fax_class           = 0;
    modem->fax_tx_pending_dle   = false;
    modem->fax_rx_active        = false;
    modem->fax_rx_pending_dle   = false;
    modem->fax_rx_complete_pending = false;
    modem->fax_rx_frame_waiting = false;
    modem->fax_rx_aborted       = false;
    modem->fax_wait_ticks       = 0;
    modem->voice_rx_draining    = false;
    modem->voice_tx_count       = 0;
    modem->voice_dle_pending    = false;
    memset(&modem->voice_deframer, 0, sizeof(modem->voice_deframer));
    modem->in_warmup           = 0;
    modem->tcpIpConnInProgress = 0;
    modem->tcpIpConnCounter    = 0;
    modem->tcpIpSocketConnected = false;
    modem->call_progress_active = false;
    modem->call_answer_sound_started = false;
    modem->call_progress_elapsed_ms = 0;
    modem->call_answer_at_ms = 0;
    modem->call_connect_at_ms = 0;

    /* Clean up PPP state */
    if (modem->ppp_ctx) {
        ppp_close(modem->ppp_ctx);
        modem->ppp_ctx = NULL;
    }
    modem->ppp_active = false;

    /* Clean up SLIP auth state */
    if (modem->slip_auth_ctx) {
        slip_auth_close(modem->slip_auth_ctx);
        modem->slip_auth_ctx = NULL;
    }

    if (modem->waitingclientsocket != (SOCKET) -1)
        plat_netsocket_close(modem->waitingclientsocket);

    if (modem->clientsocket != (SOCKET) -1)
        plat_netsocket_close(modem->clientsocket);

    modem->clientsocket = modem->waitingclientsocket = (SOCKET) -1;
    if (modem->serversocket != (SOCKET) -1) {
        modem->waitingclientsocket = plat_netsocket_accept(modem->serversocket);
        while (modem->waitingclientsocket != (SOCKET) -1) {
            plat_netsocket_close(modem->waitingclientsocket);
            modem->waitingclientsocket = plat_netsocket_accept(modem->serversocket);
        }
        plat_netsocket_close(modem->serversocket);
        modem->serversocket = (SOCKET) -1;
    }

    if (modem->waitingclientsocket != (SOCKET) -1)
        plat_netsocket_close(modem->waitingclientsocket);

    modem->waitingclientsocket = (SOCKET) -1;
    modem->tcpIpMode           = false;
    modem->tcpIpConnInProgress = false;

    if (modem->listen_port) {
        modem->serversocket = plat_netsocket_create_server(NET_SOCKET_TCP, modem->listen_port);
        if (modem->serversocket == (SOCKET) -1) {
            modem_log(modem->log, "Failed to set up server on port %d\n", modem->listen_port);
        }
    }

    modem->char_status = CHAR_COM_CTS | CHAR_COM_DSR
                       | (!modem->dcdmode ? CHAR_COM_DCD : 0);
    char_update_status(modem->char_port);
}

void
modem_enter_connected_state(modem_t *modem)
{
    modem_sound_event(modem->sound, MODEM_SOUND_CONNECT, NULL, 0);
    if (modem->voice_class)
        modem_voice_result(modem, "VCON");
    else
        modem_send_res(modem, ResCONNECT);
    modem->mode      = MODEM_MODE_DATA;
    modem->ringing   = false;
    modem->connected = true;
    modem->tcpIpMode = true;
    modem->cooldown  = true;
    modem->tx_count  = 0;
    plat_netsocket_close(modem->serversocket);
    modem->serversocket = -1;
    memset(&modem->telClient, 0, sizeof(modem->telClient));

    modem->char_status = CHAR_COM_CTS | CHAR_COM_DSR | CHAR_COM_DCD;
    char_update_status(modem->char_port);
}

void
modem_reset(modem_t *modem)
{
    modem->dcdmode = 1;
    modem_enter_idle_state(modem);
    modem->cmdpos              = 0;
    modem->cmdbuf[0]           = 0;
    modem->prevcmdbuf[0]       = 0;
    modem->lastnumber[0]       = 0;
    modem->numberinprogress[0] = 0;
    modem->flowcontrol         = 0;
    modem->cmdpause            = 0;
    modem->plusinc             = 0;
    modem->dtrmode             = 2;
    modem->speaker_mode        = 1;
    modem->speaker_level       = 2;
    modem->pulse_dial          = 0;
    modem->voice_class         = 0;
    modem->voice_rate          = 7200;
    modem->voice_bits          = 4;
    modem->voice_media_type    = VOICE_FRAME_AUDIO;
    modem->voice_v253          = false;
    modem->voice_pcm_pending   = false;
    memset(modem->voice_params, 0, sizeof(modem->voice_params));
    modem->voice_params[0] = 2;
    modem->voice_params[1] = 50;
    modem->voice_params[3] = 50;
    modem->voice_params[4] = 1;
    modem->voice_params[6] = modem->voice_params[7] = 128;
    modem->voice_vtd = 10;
    modem->voice_v253_sensitivity = 128;
    modem->voice_silence_reported = false;
    voice_silence_init(&modem->voice_silence, modem->voice_params[0]);
    modem_sound_speaker(modem->sound, modem->speaker_mode, modem->speaker_level);

    memset(&modem->reg, 0, sizeof(modem->reg));
    modem->reg[MREG_AUTOANSWER_COUNT] = 0; // no autoanswer
    modem->reg[MREG_RING_COUNT]       = 1;
    modem->reg[MREG_ESCAPE_CHAR]      = '+';
    modem->reg[MREG_CR_CHAR]          = '\r';
    modem->reg[MREG_LF_CHAR]          = '\n';
    modem->reg[MREG_BACKSPACE_CHAR]   = '\b';
    modem->reg[MREG_GUARD_TIME]       = 50;
    modem->reg[MREG_DTR_DELAY]        = 5;
    modem->reg[8]                     = 2;

    modem->echo            = true;
    modem->doresponse      = 0;
    modem->numericresponse = false;
}

void
modem_dial(modem_t *modem, const char *str)
{
    char sound_number[NUMBER_BUFFER_SIZE];
    snprintf(sound_number, sizeof(sound_number), "%s",
             modem->dial_sound_number[0] ? modem->dial_sound_number : str);
    modem->dial_sound_number[0] = 0;
    modem->tcpIpConnCounter = 0;
    modem->tcpIpMode        = false;
    if (!strcmp(str, "0.0.0.0") || !strcmp(str, "0000")) {
        modem_log(modem->log, "Entering local IP mode (type=%s (%d))\n",
              modem_connection_type_name(modem->connection_type), modem->connection_type);
        modem_enter_connected_state(modem);
        modem->numberinprogress[0] = 0;

        if (modem->connection_type == MODEM_TYPE_PPP
            || modem->connection_type == MODEM_TYPE_RAS) {
            /* PPP mode */
            modem->ppp_active = true;
            modem->tcpIpMode  = false;
            modem->ppp_ctx    = ppp_init(modem, modem->log, modem_ppp_serial_push, modem_ppp_network_send_ip);
            if (!modem->ppp_ctx) {
                modem_log(modem->log, "PPP initialization failed\n");
                modem_enter_idle_state(modem);
                modem_send_res(modem, ResNOCARRIER);
                return;
            }

            if (modem->card && modem->card->slirp_host_ip) {
                modem->ppp_ctx->our_ip = modem->card->slirp_host_ip;
                modem->ppp_ctx->peer_ip = modem->card->slirp_dhcp_ip;
                modem->ppp_ctx->dns1 = modem->card->slirp_dns_ip;
                modem->ppp_ctx->dns2 = modem->card->slirp_dns_ip;
            }

            ppp_multilink_configure(modem->ppp_ctx,
                                    modem->connection_type == MODEM_TYPE_PPP
                                        ? modem->ppp_multilink_group : NULL);

            /* Configure PPP auth from device settings */
            modem->ppp_ctx->auth_type = (ppp_auth_type_t) modem->ppp_auth_type;
            modem->ppp_ctx->mppe_min_bits = (uint8_t) modem->ppp_encryption;
            memcpy(modem->ppp_ctx->username, modem->username, sizeof(modem->ppp_ctx->username));
            memcpy(modem->ppp_ctx->password, modem->password, sizeof(modem->ppp_ctx->password));
            modem->ppp_ctx->wins1 = modem->ppp_wins1;
            modem->ppp_ctx->wins2 = modem->ppp_wins2;
            if (modem->ppp_dns1)
                modem->ppp_ctx->dns1 = modem->ppp_dns1;
            if (modem->ppp_dns2)
                modem->ppp_ctx->dns2 = modem->ppp_dns2;
            ppp_start(modem->ppp_ctx);
            if (modem->ppp_ctx->state == PPP_STATE_DEAD) {
                modem_log(modem->log, "PPP startup failed\n");
                modem_enter_idle_state(modem);
                modem_send_res(modem, ResNOCARRIER);
                return;
            }
        } else {
            /* SLIP or CSLIP mode */
            modem->tcpIpMode    = false;
            modem->cslip_enabled = (modem->connection_type == MODEM_TYPE_CSLIP);

            /* A configured username enables SLIP authentication; the password may be empty. */
            if (modem->username[0]) {
                modem->slip_auth_ctx = slip_auth_init(modem, modem->log, modem_ppp_serial_push,
                                                      modem->username, modem->password);
                if (!modem->slip_auth_ctx) {
                    modem_log(modem->log, "SLIP authentication initialization failed\n");
                    modem_enter_idle_state(modem);
                    modem_send_res(modem, ResNOCARRIER);
                    return;
                }
                slip_auth_start(modem->slip_auth_ctx);
            }
        }
        modem_sound_event(modem->sound, MODEM_SOUND_DIAL, MODEM_LOCAL_DIAL_SOUND_NUMBER,
                          (modem->reg[8] & 0xff) | (modem->pulse_dial << 8)
                              | MODEM_SOUND_DIAL_CONNECT_AFTER);
    } else {
        char buf[NUMBER_BUFFER_SIZE] = "";
        strncpy(buf, str, sizeof(buf) - 1);
        strncpy(modem->lastnumber, str, sizeof(modem->lastnumber) - 1);
        modem_log(modem->log, "Connecting to %s...\n", buf);

        // Scan host for port
        uint16_t port;
        char    *hasport = strrchr(buf, ':');
        if (hasport) {
            *hasport++ = 0;
            port       = (uint16_t) atoi(hasport);
        } else {
            port = 23;
        }

        modem->numberinprogress[0] = 0;
        modem->call_progress_active = true;
        modem->call_answer_sound_started = false;
        modem->call_progress_elapsed_ms = 0;
        modem->call_answer_at_ms = modem_sound_dial_ms(sound_number, modem->reg[8], modem->pulse_dial)
                       + modem_sound_ring_ms();
        modem->call_connect_at_ms = modem->call_answer_at_ms + modem_sound_handshake_ms();
        modem_sound_event(modem->sound, MODEM_SOUND_DIAL, sound_number,
                  (modem->reg[8] & 0xff) | (modem->pulse_dial << 8));
        modem->clientsocket        = plat_netsocket_create(NET_SOCKET_TCP);
        if (modem->clientsocket == -1) {
            modem_log(modem->log, "Failed to create client socket\n");
            modem_send_res(modem, ResNOCARRIER);
            modem_enter_idle_state(modem);
            return;
        }

        if (-1 == plat_netsocket_connect(modem->clientsocket, buf, port)) {
            modem_log(modem->log, "Failed to connect to %s\n", buf);
            modem_send_res(modem, ResNOCARRIER);
            modem_enter_idle_state(modem);
            modem_sound_event(modem->sound, MODEM_SOUND_BUSY, NULL, 0);
            return;
        }
        modem->tcpIpConnInProgress = 1;
        modem->tcpIpConnCounter    = 0;
    }
}

static bool
is_next_token(const char *a, size_t N, const char *b)
{
    // Is 'b' at least as long as 'a'?
    size_t N_without_null = N - 1;
    if (strnlen(b, N) < N_without_null)
        return false;
    return (strncmp(a, b, N_without_null) == 0);
}

static bool
is_exact_token(const char *a, size_t N, const char *b)
{
    return is_next_token(a, N, b) && b[N - 1] == '\0';
}

static const char *
modem_get_address_from_phonebook(modem_t *modem, const char *input)
{
    int i = 0;
    for (i = 0; i < modem->entries_num; i++) {
        if (strcmp(input, modem->entries[i].phone) == 0)
            return modem->entries[i].address;
    }

    return NULL;
}

/* Process the Rockwell voice parameters shared by the net and char modems. */
static int
modem_voice_rockwell_param(modem_t *modem, char **scanbuf)
{
    static const struct {
        const char *name;
        int index, min, max;
        const char *range;
    } params[] = {
        { "VSS", 0, 0, 3, "0-3" }, { "VSP", 1, 0, 255, "0-255" },
        { "VRN", 2, 0, 255, "0-255" }, { "VRA", 3, 0, 255, "0-255" },
        { "VBT", 4, 0, 255, "0-255" }, { "VSD", 5, 0, 1, "0-1" },
        { "VGT", 6, 0, 255, "0-255" }, { "VGR", 7, 0, 255, "0-255" }
    };
    char response[48];

    for (size_t i = 0; i < sizeof(params) / sizeof(params[0]); i++) {
        const size_t name_len = strlen(params[i].name);
        if (strncmp(*scanbuf, params[i].name, name_len) != 0
            || ((*scanbuf)[name_len] != '?' && (*scanbuf)[name_len] != '='
                && (*scanbuf)[name_len] != '\0'))
            continue;

        char *arg = *scanbuf + name_len;
        int *value = &modem->voice_params[params[i].index];
        if (*arg == '?') {
            arg++;
            if (*arg) goto invalid;
            snprintf(response, sizeof(response), "#%s: %d", params[i].name, *value);
            modem_send_line(modem, response);
        } else if (*arg == '=' && arg[1] == '?') {
            arg += 2;
            if (*arg) goto invalid;
            snprintf(response, sizeof(response), "#%s: (%s)", params[i].name, params[i].range);
            modem_send_line(modem, response);
        } else if (*arg == '=') {
            arg++;
            char *end = arg;
            uint32_t number = modem_scan_number(&end);
            if (end == arg || number < (uint32_t) params[i].min
                || number > (uint32_t) params[i].max)
                goto invalid;
            *value = (int) number;
            while (*end == ',') {
                end++;
                if (!isdigit((unsigned char) *end)) goto invalid;
                (void) modem_scan_number(&end);
            }
            if (*end) goto invalid;
            if (params[i].index == 0)
                voice_silence_init(&modem->voice_silence, *value);
        } else if (*arg) {
            goto invalid;
        } else {
            snprintf(response, sizeof(response), "#%s: %d", params[i].name, *value);
            modem_send_line(modem, response);
        }
        modem_send_res(modem, ResOK);
        return 1;
invalid:
        modem_send_res(modem, ResERROR);
        return 1;
    }

    {
        static const char *const compat[] = { "BDR", "VTD", "VSB", "VSK", "TL", "RG", "VCI", "VSM" };
        for (size_t i = 0; i < sizeof(compat) / sizeof(compat[0]); i++) {
            size_t n = strlen(compat[i]);
            if (strncmp(*scanbuf, compat[i], n) != 0
                || ((*scanbuf)[n] != '?' && (*scanbuf)[n] != '=' && (*scanbuf)[n] != '\0'))
                continue;
            char *arg = *scanbuf + n;
            if (*arg == '?') {
                if (arg[1]) goto compat_invalid;
                snprintf(response, sizeof(response), "#%s: 0", compat[i]);
                modem_send_line(modem, response);
            } else if (*arg == '=' && arg[1] == '?') {
                if (arg[2]) goto compat_invalid;
                snprintf(response, sizeof(response), "#%s: (0-255)", compat[i]);
                modem_send_line(modem, response);
            } else if (*arg == '=') {
                arg++;
                if (!isdigit((unsigned char) *arg)) goto compat_invalid;
                (void) modem_scan_number(&arg);
                while (*arg == ',') {
                    arg++;
                    if (!isdigit((unsigned char) *arg)) goto compat_invalid;
                    (void) modem_scan_number(&arg);
                }
                if (*arg) goto compat_invalid;
            } else if (*arg) goto compat_invalid;
            modem_send_res(modem, ResOK);
            return 1;
compat_invalid:
            modem_send_res(modem, ResERROR);
            return 1;
        }
    }
    return 0;
}

static int
modem_voice_v253_param(modem_t *modem, char **scanbuf)
{
    static const struct { const char *name; int index; } params[] = {
        { "VGT", 6 }, { "VGR", 7 }, { "VRN", 2 }, { "VRA", 3 },
        { "VBT", 4 }, { "VSP", 1 }
    };
    char response[48];

    if (strncmp(*scanbuf, "VIP", 3) == 0
        && ((*scanbuf)[3] == '?' || (*scanbuf)[3] == '=' || (*scanbuf)[3] == '\0')) {
        char *arg = *scanbuf + 3;
        if (*arg == '?') {
            if (arg[1]) goto vip_invalid;
            modem_send_line(modem, "+VIP: 0");
        } else if (*arg == '=' && arg[1] == '?') {
            if (arg[2]) goto vip_invalid;
            modem_send_line(modem, "+VIP: (0)");
        } else {
            if (*arg == '=') arg++;
            while (*arg) {
                if (!isdigit((unsigned char) *arg)) goto vip_invalid;
                (void) modem_scan_number(&arg);
                if (*arg == ',') arg++;
                else if (*arg) goto vip_invalid;
            }
            memset(modem->voice_params, 0, sizeof(modem->voice_params));
            modem->voice_params[0] = 2;
            modem->voice_params[1] = 50;
            modem->voice_params[3] = 50;
            modem->voice_params[4] = 1;
            modem->voice_params[6] = modem->voice_params[7] = 128;
            modem->voice_vtd = 10;
            modem->voice_v253_sensitivity = 128;
            voice_silence_init(&modem->voice_silence, modem->voice_params[0]);
        }
        modem_send_res(modem, ResOK);
        return 1;
vip_invalid:
        modem_send_res(modem, ResERROR);
        return 1;
    }

    if (strncmp(*scanbuf, "VSD", 3) == 0
        && ((*scanbuf)[3] == '?' || (*scanbuf)[3] == '=' || (*scanbuf)[3] == '\0')) {
        char *arg = *scanbuf + 3;
        if (*arg == '?') {
            if (arg[1]) goto invalid;
            snprintf(response, sizeof(response), "+VSD: %d,%d",
                     modem->voice_v253_sensitivity, modem->voice_params[1]);
            modem_send_line(modem, response);
        } else if (*arg == '=' && arg[1] == '?') {
            if (arg[2]) goto invalid;
            modem_send_line(modem, "+VSD: (0-255),(0-255)");
        } else if (*arg == '=') {
            char *end = arg + 1;
            uint32_t sensitivity = modem_scan_number(&end);
            if (end == arg + 1 || sensitivity > 255) goto invalid;
            int period = modem->voice_params[1];
            if (*end == ',') {
                char *period_start = ++end;
                uint32_t value = modem_scan_number(&end);
                if (end == period_start || value > 255) goto invalid;
                period = (int) value;
            }
            if (*end) goto invalid;
            modem->voice_v253_sensitivity = (int) sensitivity;
            modem->voice_params[0] = (int) sensitivity / 64;
            modem->voice_params[1] = period;
            voice_silence_init(&modem->voice_silence, modem->voice_params[0]);
        } else if (*arg) goto invalid;
        modem_send_res(modem, ResOK);
        return 1;
invalid:
        modem_send_res(modem, ResERROR);
        return 1;
    }

    if (strncmp(*scanbuf, "VTD", 3) == 0
        && ((*scanbuf)[3] == '?' || (*scanbuf)[3] == '=' || (*scanbuf)[3] == '\0')) {
        int *value = &modem->voice_vtd;
        char *arg = *scanbuf + 3;
        if (*arg == '?') {
            if (arg[1]) goto vtd_invalid;
            snprintf(response, sizeof(response), "+VTD: %d", *value);
            modem_send_line(modem, response);
        } else if (*arg == '=' && arg[1] == '?') {
            if (arg[2]) goto vtd_invalid;
            modem_send_line(modem, "+VTD: (0-255)");
        } else if (*arg == '=') {
            char *end = arg + 1;
            uint32_t number = modem_scan_number(&end);
            if (end == arg + 1 || number > 255 || *end) goto vtd_invalid;
            *value = (int) number;
        } else if (*arg) goto vtd_invalid;
        modem_send_res(modem, ResOK);
        return 1;
vtd_invalid:
        modem_send_res(modem, ResERROR);
        return 1;
    }

    for (size_t i = 0; i < sizeof(params) / sizeof(params[0]); i++) {
        const size_t name_len = strlen(params[i].name);
        if (strncmp(*scanbuf, params[i].name, name_len) != 0
            || ((*scanbuf)[name_len] != '?' && (*scanbuf)[name_len] != '='
                && (*scanbuf)[name_len] != '\0'))
            continue;
        int *value = &modem->voice_params[params[i].index];
        char *arg = *scanbuf + name_len;
        if (*arg == '?') {
            if (arg[1]) goto generic_invalid;
            snprintf(response, sizeof(response), "+%s: %d", params[i].name, *value);
            modem_send_line(modem, response);
        } else if (*arg == '=' && arg[1] == '?') {
            if (arg[2]) goto generic_invalid;
            snprintf(response, sizeof(response), "+%s: (0-255)", params[i].name);
            modem_send_line(modem, response);
        } else if (*arg == '=') {
            char *end = arg + 1;
            uint32_t number = modem_scan_number(&end);
            if (end == arg + 1 || number > 255) goto generic_invalid;
            *value = (int) number;
            while (*end == ',') {
                end++;
                if (!isdigit((unsigned char) *end)) goto generic_invalid;
                (void) modem_scan_number(&end);
            }
            if (*end) goto generic_invalid;
        } else if (*arg) goto generic_invalid;
        modem_send_res(modem, ResOK);
        return 1;
generic_invalid:
        modem_send_res(modem, ResERROR);
        return 1;
    }

    {
        static const char *const noop[] = {
            "VIT", "VNH", "VDR", "VEM", "VGM", "VGS", "VRL", "VPR"
        };
        for (size_t i = 0; i < sizeof(noop) / sizeof(noop[0]); i++) {
            size_t n = strlen(noop[i]);
            if (strncmp(*scanbuf, noop[i], n) != 0
                || ((*scanbuf)[n] != '?' && (*scanbuf)[n] != '=' && (*scanbuf)[n] != '\0'))
                continue;
            char *arg = *scanbuf + n;
            if (*arg == '?') {
                if (arg[1]) goto noop_invalid;
                snprintf(response, sizeof(response), "+%s: 0", noop[i]);
                modem_send_line(modem, response);
            } else if (*arg == '=' || !*arg) {
                while (*arg == '=') {
                    arg++;
                    if (!isdigit((unsigned char) *arg)) goto noop_invalid;
                    (void) modem_scan_number(&arg);
                    while (*arg == ',') {
                        arg++;
                        if (!isdigit((unsigned char) *arg)) goto noop_invalid;
                        (void) modem_scan_number(&arg);
                    }
                }
                if (*arg) goto noop_invalid;
            } else goto noop_invalid;
            modem_send_res(modem, ResOK);
            return 1;
noop_invalid:
            modem_send_res(modem, ResERROR);
            return 1;
        }
    }
    return 0;
}

static void
modem_do_command(modem_t *modem, int repeat)
{
    int   i       = 0;
    char *scanbuf = NULL;

    if (repeat) {
        /* Handle the case of A/ being invoked without a previous command to run */
        if (modem->prevcmdbuf[0] == '\0') {
            modem_send_res(modem, ResOK);
            return;
        }
        /* Load the stored previous command line */
        strncpy(modem->cmdbuf, modem->prevcmdbuf, sizeof(modem->cmdbuf) - 1);
        modem->cmdbuf[COMMAND_BUFFER_SIZE - 1] = '\0';
    } else {
        /* Store the command line to be recalled */
        strncpy(modem->prevcmdbuf, modem->cmdbuf, sizeof(modem->prevcmdbuf) - 1);
        modem->prevcmdbuf[COMMAND_BUFFER_SIZE - 1] = '\0';
        modem->cmdbuf[modem->cmdpos] = modem->prevcmdbuf[modem->cmdpos] = '\0';
    }
    modem->cmdpos = 0;
    for (i = 0; i < sizeof(modem->cmdbuf); i++) {
        modem->cmdbuf[i] = toupper(modem->cmdbuf[i]);
    }

    /* AT command set interpretation */
    if ((modem->cmdbuf[0] != 'A') || (modem->cmdbuf[1] != 'T')) {
        modem_send_res(modem, ResERROR);
        return;
    }

    modem_log(modem->log, "Command received: %s (doresponse = %d)\n", modem->cmdbuf, modem->doresponse);
    MODEM_DEBUG_LOG(modem->log, "AT parser: command length=%u repeat=%d\n",
                    (unsigned) strlen(modem->cmdbuf), repeat);

    scanbuf = &modem->cmdbuf[2];

    while (1) {
        char chr = modem_fetch_character(&scanbuf);
        switch (chr) {
            case '#':
                if (modem_voice_rockwell_param(modem, &scanbuf))
                    return;
                if (is_exact_token("CLS=8", sizeof("CLS=8"), scanbuf)) {
                    if (!(modem->fax_support & MODEM_FAX_SUPPORT_CLASS_8)) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    modem->voice_class = 8;
                    modem->fax_class = 8;
                    modem->voice_v253 = false;
                    modem->voice_media_type = VOICE_FRAME_AUDIO;
                    modem->voice_rate = 7200;
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_exact_token("CLS=0", sizeof("CLS=0"), scanbuf)) {
                    modem->voice_class = 0;
                    if (modem->fax_class == 8)
                        modem->fax_class = 0;
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_exact_token("CLS?", sizeof("CLS?"), scanbuf)) {
                    modem_send_line(modem, modem->voice_class ? "#CLS: 8" : "#CLS: 0");
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_exact_token("CLS=?", sizeof("CLS=?"), scanbuf)) {
                    modem_send_line(modem, (modem->fax_support & MODEM_FAX_SUPPORT_CLASS_8)
                                               ? "#CLS: (0,8)" : "#CLS: (0)");
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_exact_token("VLS?", sizeof("VLS?"), scanbuf)) {
                    char response[16];
                    snprintf(response, sizeof(response), "#VLS: %d", modem->voice_vls);
                    modem_send_line(modem, response);
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_exact_token("VLS=?", sizeof("VLS=?"), scanbuf)) {
                    modem_send_line(modem, "#VLS: (0,1,2,3,4,6)");
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_next_token("VLS=", sizeof("VLS="), scanbuf)) {
                    scanbuf += 4;
                    const uint32_t vls = modem_scan_number(&scanbuf);
                    if ((vls != 0 && vls != 1 && vls != 2 && vls != 3
                         && vls != 4 && vls != 6) || *scanbuf != '\0') {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    if (vls == 0)
                        modem_voice_set_line(modem, 0);
                    else if (vls == 1 || vls == 3 || vls == 4 || vls == 6)
                        modem_voice_set_line(modem, 1);
                    else
                        modem->voice_vls = (int) vls;
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_next_token("VTS=", sizeof("VTS="), scanbuf)) {
                    scanbuf += 4;
                    if (!modem_voice_send_vts(modem, scanbuf, 100,
                            (uint32_t) MAX(40, modem->voice_params[4] * 100))) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_exact_token("VBS=?", sizeof("VBS=?"), scanbuf)) {
                    modem_send_line(modem, "#VBS: (2-4,8,16)");
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_exact_token("VSR=?", sizeof("VSR=?"), scanbuf)) {
                    modem_send_line(modem, "#VSR: (7200,8000,11025)");
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_exact_token("VBS?", sizeof("VBS?"), scanbuf)) {
                    char response[16];
                    int format_bits = modem->voice_media_type == VOICE_FRAME_U8 ? 8
                                    : modem->voice_media_type == VOICE_FRAME_S16 ? 16
                                    : modem->voice_bits;
                    snprintf(response, sizeof(response), "#VBS: %d", format_bits);
                    modem_send_line(modem, response);
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_exact_token("VSR?", sizeof("VSR?"), scanbuf)) {
                    char response[20];
                    snprintf(response, sizeof(response), "#VSR: %d", modem->voice_rate);
                    modem_send_line(modem, response);
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_next_token("VBS=", sizeof("VBS="), scanbuf)) {
                    scanbuf += 4;
                    const uint32_t bits = modem_scan_number(&scanbuf);
                    if (((bits < 2 || bits > 4) && bits != 8 && bits != 16)
                        || *scanbuf != '\0') {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    if (bits <= 4) {
                        modem->voice_bits = (int) bits;
                        modem->voice_media_type = VOICE_FRAME_AUDIO;
                    } else
                        modem->voice_media_type = bits == 8 ? VOICE_FRAME_U8 : VOICE_FRAME_S16;
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_next_token("VSR=", sizeof("VSR="), scanbuf)) {
                    scanbuf += 4;
                    const uint32_t rate = modem_scan_number(&scanbuf);
                    if ((rate != 7200 && rate != 8000 && rate != 11025) || *scanbuf != '\0') {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    modem->voice_rate = (int) rate;
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_exact_token("VTX", sizeof("VTX"), scanbuf)
                           || is_exact_token("VRX", sizeof("VRX"), scanbuf)) {
                    const bool transmit = scanbuf[1] == 'T';
                    if (!modem_voice_start(modem, transmit ? MODEM_MODE_VOICE_TX
                                                           : MODEM_MODE_VOICE_RX)) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    return;
                }
                modem_send_res(modem, ResERROR);
                return;
            case '+':
                if (modem_voice_v253_param(modem, &scanbuf))
                    return;
                if (is_next_token("FCLASS", sizeof("FCLASS"), scanbuf)
                    && is_exact_token("FCLASS=?", sizeof("FCLASS=?"), scanbuf)) {
                    scanbuf += 8;
                    if (modem->fax_support == MODEM_FAX_SUPPORT_DISABLED) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    char classes[32] = "+FCLASS: (";
                    bool first = true;
                    if (modem->fax_support & MODEM_FAX_SUPPORT_CLASS_0) {
                        strcat(classes, "0");
                        first = false;
                    }
                    if (modem->fax_support & MODEM_FAX_SUPPORT_CLASS_1) {
                        strcat(classes, first ? "1" : ",1");
                        first = false;
                    }
                    if (modem->fax_support & MODEM_FAX_SUPPORT_CLASS_8)
                        strcat(classes, first ? "8" : ",8");
                    strcat(classes, ")");
                    modem_send_line(modem, classes);
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_next_token("FCLASS", sizeof("FCLASS"), scanbuf)
                           && is_exact_token("FCLASS?", sizeof("FCLASS?"), scanbuf)) {
                    char response[16];
                    scanbuf += 7;
                    snprintf(response, sizeof(response), "+FCLASS: %u",
                             (unsigned) modem->fax_class);
                    modem_send_line(modem, response);
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_next_token("FCLASS", sizeof("FCLASS"), scanbuf)
                           && (is_exact_token("FCLASS=0", sizeof("FCLASS=0"), scanbuf)
                               || is_exact_token("FCLASS=0.0", sizeof("FCLASS=0.0"), scanbuf))) {
                    scanbuf += 8;
                    if (!(modem->fax_support & MODEM_FAX_SUPPORT_CLASS_0)) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    modem->fax_class = 0;
                    modem->voice_class = 0;
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_next_token("FCLASS", sizeof("FCLASS"), scanbuf)
                           && (is_exact_token("FCLASS=1", sizeof("FCLASS=1"), scanbuf)
                               || is_exact_token("FCLASS=1.0", sizeof("FCLASS=1.0"), scanbuf))) {
                    scanbuf += 8;
                    if (!(modem->fax_support & MODEM_FAX_SUPPORT_CLASS_1)) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    modem->fax_class = 1;
                    modem->voice_class = 0;
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_next_token("FCLASS", sizeof("FCLASS"), scanbuf)
                           && (is_exact_token("FCLASS=8", sizeof("FCLASS=8"), scanbuf)
                               || is_exact_token("FCLASS=8.0", sizeof("FCLASS=8.0"), scanbuf))) {
                    scanbuf += 8;
                    if (!(modem->fax_support & MODEM_FAX_SUPPORT_CLASS_8)) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    modem->fax_class = 8;
                    modem->voice_class = 8;
                    modem->voice_v253 = true;
                    modem->voice_media_type = VOICE_FRAME_AUDIO;
                    modem->voice_rate = 8000;
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_next_token("FCLASS=", sizeof("FCLASS="), scanbuf)) {
                    /* Catch any other FCLASS assignment attempt. */
                    scanbuf += 8;
                    modem_send_res(modem, ResERROR);
                    return;
                } else if (is_next_token("VLS", sizeof("VLS"), scanbuf)) {
                    scanbuf += 3;
                    if (is_exact_token("?", sizeof("?"), scanbuf)) {
                        char response[24];
                        snprintf(response, sizeof(response), "+VLS: %d", modem->voice_vls);
                        modem_send_line(modem, response);
                    } else if (is_exact_token("=?", sizeof("=?"), scanbuf)) {
                        modem_send_line(modem, "+VLS: (0,1,2,3,4,5,6,7,8,9,11,13)");
                    } else if (scanbuf[0] == '=') {
                        char *end = scanbuf + 1;
                        if (!isdigit((unsigned char) *end)
                            || strlen(end) > 2) {
                            modem_send_res(modem, ResERROR);
                            return;
                        }
                        const uint32_t vls = modem_scan_number(&end);
                        if (end == scanbuf + 1 || *end) {
                            modem_send_res(modem, ResERROR);
                            return;
                        }
                        if (vls != 0 && vls != 1 && vls != 2 && vls != 3
                            && vls != 4 && vls != 5 && vls != 6 && vls != 7
                            && vls != 8 && vls != 9 && vls != 11 && vls != 13) {
                            modem_send_res(modem, ResERROR);
                            return;
                        }
                        if (vls == 0)
                            modem_voice_set_line(modem, 0);
                        else if (vls == 1 || vls == 3 || vls == 5 || vls == 7
                                 || vls == 9 || vls == 13)
                            modem_voice_set_line(modem, 1);
                        else
                            modem->voice_vls = vls; /* Host audio routes are not attached. */
                    } else {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_next_token("VSM", sizeof("VSM"), scanbuf)) {
                    scanbuf += 3;
                    if (is_exact_token("?", sizeof("?"), scanbuf)) {
                        char response[32];
                        int format = modem->voice_media_type == VOICE_FRAME_AUDIO ? 4
                                   : modem->voice_media_type == VOICE_FRAME_ALAW ? 5
                                   : modem->voice_media_type == VOICE_FRAME_S8 ? 0
                                   : modem->voice_media_type == VOICE_FRAME_U8 ? 1 : 2;
                        snprintf(response, sizeof(response), "+VSM: %d,%d,0,0",
                                 format, modem->voice_rate);
                        modem_send_line(modem, response);
                    } else if (is_exact_token("=?", sizeof("=?"), scanbuf)) {
                        modem_send_line(modem, "+VSM: 0,\"SIGNED PCM\",8,0,(7200,8000,11025),(0),(0)");
                        modem_send_line(modem, "+VSM: 1,\"UNSIGNED PCM\",8,0,(7200,8000,11025),(0),(0)");
                        modem_send_line(modem, "+VSM: 2,\"SIGNED PCM\",16,0,(7200,8000,11025),(0),(0)");
                        modem_send_line(modem, "+VSM: 4,\"ULAW\",8,0,(8000),(0),(0)");
                        modem_send_line(modem, "+VSM: 5,\"ALAW\",8,0,(8000),(0),(0)");
                        modem_send_line(modem, "+VSM: 128,\"8-BIT LINEAR\",8,0,(7200,8000,11025),(0),(0)");
                        modem_send_line(modem, "+VSM: 129,\"16-BIT LINEAR\",16,0,(7200,8000,11025),(0),(0)");
                    } else if (scanbuf[0] == '=') {
                        int format;
                        int rate = modem->voice_rate;
                        scanbuf++;
                        char *format_start = scanbuf;
                        format = (int) modem_scan_number(&scanbuf);
                        if (scanbuf[0] == ',') {
                            scanbuf++;
                            rate = (int) modem_scan_number(&scanbuf);
                        }
                        if (scanbuf == format_start
                            || (rate != 7200 && rate != 8000 && rate != 11025)) {
                            modem_send_res(modem, ResERROR);
                            return;
                        }
                        while (*scanbuf == ',') {
                            scanbuf++;
                            char *option_start = scanbuf;
                            (void) modem_scan_number(&scanbuf);
                            if (scanbuf == option_start) {
                                modem_send_res(modem, ResERROR);
                                return;
                            }
                        }
                        if (*scanbuf) {
                            modem_send_res(modem, ResERROR);
                            return;
                        }
                        switch (format) {
                            case 0: modem->voice_media_type = VOICE_FRAME_S8; modem->voice_v253 = true; break;
                            case 1: case 128: modem->voice_media_type = VOICE_FRAME_U8; modem->voice_v253 = true; break;
                            case 2: case 129: modem->voice_media_type = VOICE_FRAME_S16; modem->voice_v253 = true; break;
                            case 4: modem->voice_media_type = VOICE_FRAME_AUDIO; modem->voice_v253 = true; break;
                            case 5: modem->voice_media_type = VOICE_FRAME_ALAW; modem->voice_v253 = true; break;
                            default:
                                modem_send_res(modem, ResERROR);
                                return;
                        }
                        if ((format == 4 || format == 5) && rate != 8000) {
                            modem_send_res(modem, ResERROR);
                            return;
                        }
                        modem->voice_rate = rate;
                    } else {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_exact_token("VTX?", sizeof("VTX?"), scanbuf)
                           || is_exact_token("VRX?", sizeof("VRX?"), scanbuf)
                           || is_exact_token("VTR?", sizeof("VTR?"), scanbuf)) {
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_exact_token("VTX", sizeof("VTX"), scanbuf)
                           || is_exact_token("VRX", sizeof("VRX"), scanbuf)
                           || is_exact_token("VTR", sizeof("VTR"), scanbuf)) {
                    modem_mode_t voice_mode = MODEM_MODE_VOICE_TX;
                    if (scanbuf[1] == 'R')
                        voice_mode = MODEM_MODE_VOICE_RX;
                    else if (scanbuf[1] == 'T' && scanbuf[2] == 'R')
                        voice_mode = MODEM_MODE_VOICE_TR;
                    if (!modem_voice_start(modem, voice_mode)) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    return;
                } else if (is_next_token("VTS=", sizeof("VTS="), scanbuf)) {
                    scanbuf += 4;
                    if (!modem_voice_send_vts(modem, scanbuf, 10,
                            (uint32_t) MAX(40, modem->voice_vtd * 10))) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    modem_send_res(modem, ResOK);
                    return;
                } else if (is_next_token("FTS=", sizeof("FTS="), scanbuf)
                           || is_next_token("FRS=", sizeof("FRS="), scanbuf)) {
                    const char *wait_start;
                    scanbuf += 4;
                    wait_start = scanbuf;
                    const uint32_t wait = modem_scan_number(&scanbuf);
                    if (!(modem->fax_support & MODEM_FAX_SUPPORT_CLASS_1)
                        || modem->fax_class != 1 || !modem->connected
                        || !modem->tcpIpMode || modem->telnet_mode
                        || scanbuf == wait_start || wait > 255 || *scanbuf != '\0') {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    if (wait) {
                        modem->fax_wait_ticks = (uint16_t) (wait * 10);
                        modem->mode = MODEM_MODE_FAX_WAIT;
                        modem->cmdpause = 0;
                        modem->plusinc = 0;
                    } else {
                        modem_send_res(modem, ResOK);
                    }
                    return;
                } else if (is_next_token("FTH=", sizeof("FTH="), scanbuf)
                           || is_next_token("FTM=", sizeof("FTM="), scanbuf)
                           || is_next_token("FRH=", sizeof("FRH="), scanbuf)
                           || is_next_token("FRM=", sizeof("FRM="), scanbuf)) {
                    const bool transmit = scanbuf[1] == 'T';
                    scanbuf += 4;
                    const uint32_t rate = modem_scan_number(&scanbuf);
                    if (!(modem->fax_support & MODEM_FAX_SUPPORT_CLASS_1)
                        || modem->fax_class != 1 || !modem->connected
                        || !modem->tcpIpMode || modem->telnet_mode
                        || !modem_fax_rate_supported(rate)
                        || *scanbuf != '\0') {
                        modem_send_res(modem, ResERROR);
                        return;
                    }

                    if (transmit) {
                        modem->fax_tx_pending_dle = false;
                        modem->mode = MODEM_MODE_FAX_TX;
                    } else {
                        modem->fax_rx_active = !modem->fax_rx_frame_waiting;
                        modem->fax_rx_complete_pending = modem->fax_rx_frame_waiting;
                        modem->fax_rx_frame_waiting = false;
                        if (modem->fax_rx_active)
                            modem->fax_rx_aborted = false;
                        modem->mode = MODEM_MODE_DATA;
                    }
                    modem->cmdpause = 0;
                    modem->plusinc = 0;
                    modem_send_res(modem, ResCONNECT);
                    return;
                } else if (is_next_token("NET", sizeof("NET"), scanbuf)) {
                    // only walk the pointer ahead if the command matches
                    scanbuf += 3;
                    const uint32_t requested_mode = modem_scan_number(&scanbuf);

                    // If the mode isn't valid then stop parsing
                    if (requested_mode != 1 && requested_mode != 0) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    // Inform the user on changes
                    if (modem->telnet_mode != !!requested_mode) {
                        modem->telnet_mode = !!requested_mode;
                    }
                    break;
                }
                modem_send_res(modem, ResERROR);
                return;
            case 'D':
                { // Dial.
                    char        buffer[NUMBER_BUFFER_SIZE];
                    char        obuffer[NUMBER_BUFFER_SIZE];
                    char       *foundstr   = &scanbuf[0];
                    const char *mappedaddr = NULL;
                    size_t      i          = 0;

                    if ((foundstr[0] == 'T' && foundstr[1] == 'P') || (foundstr[0] == 'P' && foundstr[1] == 'T')) { // Make win16 dialer happy
                        modem->pulse_dial = foundstr[1] == 'P';
                        foundstr += 2;
                    } else if (*foundstr == 'T' || *foundstr == 'P') { // Tone/pulse dialing
                        modem->pulse_dial = *foundstr == 'P';
                        foundstr++;
                    } else if (*foundstr == 'L') { // Redial last number
                        if (modem->lastnumber[0] == 0)
                            modem_send_res(modem, ResERROR);
                        else {
                            modem_log(modem->log, "Redialing number %s\n", modem->lastnumber);
                            modem_dial(modem, modem->lastnumber);
                        }
                        return;
                    }

                    if ((!foundstr[0] && !modem->numberinprogress[0]) || ((strlen(modem->numberinprogress) + strlen(foundstr)) > (NUMBER_BUFFER_SIZE - 1))) {
                        // Check for empty or too long strings
                        modem_send_res(modem, ResERROR);
                        modem->numberinprogress[0] = 0;
                        return;
                    }

                    foundstr = trim(foundstr);

                    // Check for ; and return to command mode if found
                    char *semicolon = strchr(foundstr, ';');
                    if (semicolon != NULL) {
                        modem_log(modem->log, "Semicolon found in number, returning to command mode\n");
                        strncat(modem->numberinprogress, foundstr, strcspn(foundstr, ";"));
                        scanbuf = semicolon + 1;
                        break;
                    } else {
                        strcat(modem->numberinprogress, foundstr);
                        foundstr = modem->numberinprogress;
                    }

                    modem_log(modem->log, "Dialing number %s\n", foundstr);
                    char dial_sound_number[NUMBER_BUFFER_SIZE];
                    snprintf(dial_sound_number, sizeof(dial_sound_number), "%s", foundstr);
                    mappedaddr = modem_get_address_from_phonebook(modem, foundstr);
                    if (mappedaddr) {
                        snprintf(modem->dial_sound_number, sizeof(modem->dial_sound_number), "%s",
                                 dial_sound_number);
                        modem_dial(modem, mappedaddr);
                        return;
                    }

                    if (strlen(foundstr) >= 12) {
                        // Check if supplied parameter only consists of digits
                        bool   isNum = true;
                        size_t fl    = strlen(foundstr);
                        for (i = 0; i < fl; i++)
                            if (foundstr[i] < '0' || foundstr[i] > '9')
                                isNum = false;
                        if (isNum && (fl > (NUMBER_BUFFER_SIZE - 5))) {
                            // Check if the number is long enough to cause buffer
                            // overflows during the number => IP transformation
                            modem_send_res(modem, ResERROR);
                            modem->numberinprogress[0] = 0;
                            return;
                        } else if (isNum) {
                            // Parameter is a number with at least 12 digits => this cannot
                            // be a valid IP/name
                            // Transform by adding dots
                            size_t       j        = 0;
                            const size_t foundlen = strlen(foundstr);
                            for (i = 0; i < foundlen; i++) {
                                buffer[j++] = foundstr[i];
                                // Add a dot after the third, sixth and ninth number
                                if (i == 2 || i == 5 || i == 8)
                                    buffer[j++] = '.';
                                // If the string is longer than 12 digits,
                                // interpret the rest as port
                                if (i == 11 && foundlen > 12)
                                    buffer[j++] = ':';
                            }
                            buffer[j] = 0;
                            foundstr  = buffer;

                            // Remove Zeros from beginning of octets
                            size_t k         = 0;
                            size_t foundlen2 = strlen(foundstr);
                            for (i = 0; i < foundlen2; i++) {
                                if (i == 0 && foundstr[0] == '0')
                                    continue;
                                if (i == 1 && foundstr[0] == '0' && foundstr[1] == '0')
                                    continue;
                                if (foundstr[i] == '0' && foundstr[i - 1] == '.')
                                    continue;
                                if (foundstr[i] == '0' && foundstr[i - 1] == '0' && foundstr[i - 2] == '.')
                                    continue;
                                obuffer[k++] = foundstr[i];
                            }
                            obuffer[k] = 0;
                            foundstr   = obuffer;
                        }
                    }
                    snprintf(modem->dial_sound_number, sizeof(modem->dial_sound_number), "%s",
                             dial_sound_number);
                    modem_dial(modem, foundstr);
                    return;
                }
            case 'I': // Modem identification strings
                {
                    const uint32_t index = modem_scan_number(&scanbuf);
                    if (modem->modem_identity == MODEM_IDENTITY_SUPRAEXPRESS) {
                        static const char *const supra_info[] = {
                            "1794",
                            "168",
                            "OK",
                            "SupraExpress 56e PRO",
                            "Diamond Multimedia SupraExpress 56e PRO",
                            "Country Code: 00",
                            "RCVDL56ACF/SP Rev 1.100",
                            "V1.100-V90_2M_DLS"
                        };
                        if (index < sizeof(supra_info) / sizeof(supra_info[0]))
                            modem_send_line(modem, supra_info[index]);
                    } else {
                        switch (index) {
                            case 3:
                                modem_send_line(modem, "86Box Emulated Modem Firmware V1.00");
                                break;
                            case 4:
                                modem_send_line(modem, "Modem compiled for 86Box version " EMU_VERSION);
                                break;
                        }
                    }
                }
                break;
            case 'E': // Echo on/off
                switch (modem_scan_number(&scanbuf)) {
                    case 0:
                        modem->echo = false;
                        break;
                    case 1:
                        modem->echo = true;
                        break;
                }
                break;
            case 'V':
                switch (modem_scan_number(&scanbuf)) {
                    case 0:
                        modem->numericresponse = true;
                        break;
                    case 1:
                        modem->numericresponse = false;
                        break;
                }
                break;
            case 'H': // Hang up
                switch (modem_scan_number(&scanbuf)) {
                    case 0:
                        modem->numberinprogress[0] = 0;
                        if (modem->connected) {
                            modem_send_res(modem, ResNOCARRIER);
                            modem_enter_idle_state(modem);
                            return;
                        }
                        // else return ok
                }
                break;
            case 'O': // Return to data mode
                switch (modem_scan_number(&scanbuf)) {
                    case 0:
                        if (modem->connected) {
                            modem->mode = MODEM_MODE_DATA;
                            return;
                        } else {
                            modem_send_res(modem, ResERROR);
                            return;
                        }
                }
                break;
            case 'T': // Tone Dial
            case 'P': // Pulse Dial
                modem->pulse_dial = chr == 'P';
                break;
            case 'M': // Monitor speaker mode
                {
                    const uint32_t mode = modem_scan_number(&scanbuf);
                    if (mode > 3) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    modem->speaker_mode = (int) mode;
                    modem_sound_speaker(modem->sound, modem->speaker_mode, modem->speaker_level);
                }
                break;
            case 'L': // Speaker volume
                {
                    const uint32_t level = modem_scan_number(&scanbuf);
                    if (level > 3) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    modem->speaker_level = (int) level;
                    modem_sound_speaker(modem->sound, modem->speaker_mode, modem->speaker_level);
                }
                break;
            case 'W':
            case 'X':
                modem_scan_number(&scanbuf);
                break;
            case 'A': // Answer call
                {
                    if (modem->waitingclientsocket == -1) {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                    modem_accept_incoming_call(modem);
                    break;
                }
                return;
            case 'Z':
                { // Reset and load profiles
                    // scan the number away, if any
                    modem_scan_number(&scanbuf);
                    if (modem->connected)
                        modem_send_res(modem, ResNOCARRIER);
                    modem_reset(modem);
                    break;
                }
            case ' ': // skip space
                break;
            case 'Q':
                {
                    // Response options
                    // 0 = all on, 1 = all off,
                    // 2 = no ring and no connect/carrier in answermode
                    const uint32_t val = modem_scan_number(&scanbuf);
                    if (!(val > 2)) {
                        modem->doresponse = val;
                        break;
                    } else {
                        modem_send_res(modem, ResERROR);
                        return;
                    }
                }

            case 'S':
                { // Registers
                    const uint32_t index = modem_scan_number(&scanbuf);
                    if (index >= MODEM_REGS) {
                        modem_send_res(modem, ResERROR);
                        return; // goto ret_none;
                    }

                    while (scanbuf[0] == ' ')
                        scanbuf++; // skip spaces

                    if (scanbuf[0] == '=') { // set register
                        scanbuf++;
                        while (scanbuf[0] == ' ')
                            scanbuf++; // skip spaces
                        const uint32_t val = modem_scan_number(&scanbuf);
                        modem->reg[index]  = val;
                        break;
                    } else if (scanbuf[0] == '?') { // get register
                        modem_send_number(modem, modem->reg[index]);
                        scanbuf++;
                        break;
                    }
                    // else
                    // LOG_MSG("SERIAL: Port %" PRIu8 " print reg %" PRIu32
                    //         " with %" PRIu8 ".",
                    //         GetPortNumber(), index, reg[index]);
                }
                break;
            case '&':
                { // & escaped commands
                    char cmdchar = modem_fetch_character(&scanbuf);
                    switch (cmdchar) {
                        case 'C':
                            {
                                const uint32_t val = modem_scan_number(&scanbuf);
                                if (val < 2)
                                    modem->dcdmode = val;
                                else {
                                    modem_send_res(modem, ResERROR);
                                    return;
                                }
                                break;
                            }
                        case 'K':
                            {
                                const uint32_t val = modem_scan_number(&scanbuf);
                                if (val < 5)
                                    modem->flowcontrol = val;
                                else {
                                    modem_send_res(modem, ResERROR);
                                    return;
                                }
                                break;
                            }
                        case 'D':
                            {
                                const uint32_t val = modem_scan_number(&scanbuf);
                                if (val < 4)
                                    modem->dtrmode = val;
                                else {
                                    modem_send_res(modem, ResERROR);
                                    return;
                                }
                                break;
                            }
                        case '\0':
                            // end of string
                            modem_send_res(modem, ResERROR);
                            return;
                    }
                    break;
                }
                break;
            case '\\':
                { // \ escaped commands
                    char cmdchar = modem_fetch_character(&scanbuf);
                    switch (cmdchar) {
                        case 'N':
                            // error correction stuff - not emulated
                            if (modem_scan_number(&scanbuf) > 5) {
                                modem_send_res(modem, ResERROR);
                                return;
                            }
                            break;
                        case '\0':
                            // end of string
                            modem_send_res(modem, ResERROR);
                            return;
                    }
                    break;
                }
            case '%': // % escaped commands
                // Windows 98 modem prober sends unknown command AT%V
                modem_send_res(modem, ResERROR);
                return;
            case '\0':
                modem_send_res(modem, ResOK);
                return;
        }
    }
}

void
modem_dtr_callback_timer(void *priv)
{
    modem_t *dev = (modem_t *) priv;
    if (dev->connected) {
        switch (dev->dtrmode) {
            case 1:
                modem_log(dev->log, "DTR dropped, returning to command mode (dtrmode = %i)\n", dev->dtrmode);
                dev->mode = MODEM_MODE_COMMAND;
                break;
            case 2:
                modem_log(dev->log, "DTR dropped, hanging up (dtrmode = %i)\n", dev->dtrmode);
                modem_send_res(dev, ResNOCARRIER);
                modem_enter_idle_state(dev);
                break;
            case 3:
                modem_log(dev->log, "DTR dropped, resetting modem (dtrmode = %i)\n", dev->dtrmode);
                modem_send_res(dev, ResNOCARRIER);
                modem_reset(dev);
                break;
        }
    }
}

static size_t
modem_char_read(uint8_t *buf, size_t len, void *priv)
{
    modem_t *modem = (modem_t *) priv;
    size_t count = 0;

    while (count < len && !fifo8_is_empty(&modem->char_tx_data))
        buf[count++] = fifo8_pop(&modem->char_tx_data);
    return count;
}

static size_t
modem_char_write(uint8_t *buf, size_t len, void *priv)
{
    for (size_t i = 0; i < len; i++)
        modem_write_byte(priv, buf[i]);
    return len;
}

static uint32_t
modem_char_status(void *priv)
{
    return ((modem_t *) priv)->char_status;
}

static void
modem_char_control(uint32_t flags, void *priv)
{
    modem_t *modem = (modem_t *) priv;
    const bool dtr = !!(flags & CHAR_COM_DTR);

    modem->char_rts = !!(flags & CHAR_COM_RTS);
    if (modem->dtrstate == dtr)
        return;
    modem->dtrstate = dtr;
    if (dtr)
        timer_disable(&modem->dtr_timer);
    else if (!timer_is_enabled(&modem->dtr_timer))
        timer_on_auto(&modem->dtr_timer, 1000000);
}

static void
modem_char_port_config(void *priv)
{
    modem_t *modem = (modem_t *) priv;
    if (modem->char_port && modem->char_port->com.baud)
        modem->baudrate = modem->char_port->com.baud;
    modem_speed_changed(modem);
}

static void
fifo8_resize_2x(Fifo8 *fifo)
{
    uint32_t pos  = 0;
    uint32_t size = fifo->capacity * 2;
    uint32_t used = fifo8_num_used(fifo);
    if (!used)
        return;

    uint8_t *temp_buf = calloc(size, 1);
    if (!temp_buf) {
        fatal("modem: Out Of Memory!\n");
    }
    while (!fifo8_is_empty(fifo)) {
        temp_buf[pos] = fifo8_pop(fifo);
        pos++;
    }
    pos = 0;
    fifo8_destroy(fifo);
    fifo8_create(fifo, size);
    fifo8_push_all(fifo, temp_buf, used);
    free(temp_buf);
}

#define TEL_CLIENT 0
#define TEL_SERVER 1
void
modem_process_telnet(modem_t *modem, uint8_t *data, uint32_t size)
{
    uint32_t i = 0;
    for (i = 0; i < size; i++) {
        uint8_t c = data[i];
        if (modem->telClient.inIAC) {
            if (modem->telClient.recCommand) {
                modem_log(modem->log, "modem_process_telnet: received command %i, option %i\n", modem->telClient.command, c);

                if ((c != 0) && (c != 1) && (c != 3)) {
                    /* Reject anything we don't recognize */
                    if (modem->telClient.command == 251 || modem->telClient.command == 252) {
                        modem_data_mode_process_byte(modem, 0xff);
                        modem_data_mode_process_byte(modem, 254);
                        modem_data_mode_process_byte(modem, c); /* Don't do crap! */
                    } else if (modem->telClient.command == 253 || modem->telClient.command == 254) {
                        modem_data_mode_process_byte(modem, 0xff);
                        modem_data_mode_process_byte(modem, 252);
                        modem_data_mode_process_byte(modem, c); /* We won't do crap! */
                    }
                }
                switch (modem->telClient.command) {
                    case 251: /* Will */
                        if (c == 0)
                            modem->telClient.binary[TEL_SERVER] = true;
                        if (c == 1)
                            modem->telClient.echo[TEL_SERVER] = true;
                        if (c == 3)
                            modem->telClient.supressGA[TEL_SERVER] = true;
                        break;
                    case 252: /* Won't */
                        if (c == 0)
                            modem->telClient.binary[TEL_SERVER] = false;
                        if (c == 1)
                            modem->telClient.echo[TEL_SERVER] = false;
                        if (c == 3)
                            modem->telClient.supressGA[TEL_SERVER] = false;
                        break;
                    case 253: /* Do */
                        if (c == 0) {
                            modem->telClient.binary[TEL_CLIENT] = true;
                            modem_data_mode_process_byte(modem, 0xff);
                            modem_data_mode_process_byte(modem, 251);
                            modem_data_mode_process_byte(modem, 0); /* Will do binary transfer */
                        }
                        if (c == 1) {
                            modem->telClient.echo[TEL_CLIENT] = false;
                            modem_data_mode_process_byte(modem, 0xff);
                            modem_data_mode_process_byte(modem, 252);
                            modem_data_mode_process_byte(modem, 1); /* Won't echo (too lazy) */
                        }
                        if (c == 3) {
                            modem->telClient.supressGA[TEL_CLIENT] = true;
                            modem_data_mode_process_byte(modem, 0xff);
                            modem_data_mode_process_byte(modem, 251);
                            modem_data_mode_process_byte(modem, 3); /* Will Suppress GA */
                        }
                        break;
                    case 254: /* Don't */
                        if (c == 0) {
                            modem->telClient.binary[TEL_CLIENT] = false;
                            modem_data_mode_process_byte(modem, 0xff);
                            modem_data_mode_process_byte(modem, 252);
                            modem_data_mode_process_byte(modem, 0); /* Won't do binary transfer */
                        }
                        if (c == 1) {
                            modem->telClient.echo[TEL_CLIENT] = false;
                            modem_data_mode_process_byte(modem, 0xff);
                            modem_data_mode_process_byte(modem, 252);
                            modem_data_mode_process_byte(modem, 1); /* Won't echo (fine by me) */
                        }
                        if (c == 3) {
                            modem->telClient.supressGA[TEL_CLIENT] = true;
                            modem_data_mode_process_byte(modem, 0xff);
                            modem_data_mode_process_byte(modem, 251);
                            modem_data_mode_process_byte(modem, 3); /* Will Suppress GA (too lazy) */
                        }
                        break;
                    default:
                        break;
                }
                modem->telClient.inIAC      = false;
                modem->telClient.recCommand = false;
                continue;
            } else {
                if (c == 249) {
                    /* Go Ahead received */
                    modem->telClient.inIAC = false;
                    continue;
                }
                modem->telClient.command    = c;
                modem->telClient.recCommand = true;

                if ((modem->telClient.binary[TEL_SERVER]) && (c == 0xff)) {
                    /* Binary data with value of 255 */
                    modem->telClient.inIAC      = false;
                    modem->telClient.recCommand = false;
                    fifo8_push(&modem->rx_data, 0xff);
                    continue;
                }
            }
        } else {
            if (c == 0xff) {
                modem->telClient.inIAC = true;
                continue;
            }
            fifo8_push(&modem->rx_data, c);
        }
    }
}

static bool
modem_queue_slip_frame(modem_t *modem, const uint8_t *packet, size_t packet_len)
{
    if (packet_len > (((size_t) -1) - 2) / 2)
        return false;

    size_t frame_capacity = packet_len * 2 + 2;
    size_t frame_len;
    uint8_t *frame = (uint8_t *) malloc(frame_capacity);
    if (!frame)
        return false;

    bool encoded = slip_encode_frame_logged(modem->log, packet, packet_len,
                                           frame, frame_capacity, &frame_len);
    if (encoded) {
        while (frame_len > fifo8_num_free(&modem->rx_data))
            fifo8_resize_2x(&modem->rx_data);
        for (size_t pos = 0; pos < frame_len; pos++)
            fifo8_push(&modem->rx_data, frame[pos]);
    }
    free(frame);
    return encoded;
}

static int
modem_rx(void *priv, uint8_t *buf, int io_len)
{
    modem_t *modem = (modem_t *) priv;

    if (!buf || io_len < 14)
        return 0;

    if (modem->tcpIpMode)
        return 0;

    if (!modem->connected) {
        /* Drop packet. */
        modem_log(modem->log, "Dropping %d bytes (EtherType %s 0x%04X)\n",
                  io_len - 14,
                  modem_ethertype_name((uint16_t) (((uint16_t) buf[12] << 8) | buf[13])),
                  (unsigned) (((uint16_t) buf[12] << 8) | buf[13]));
        return 0;
    }

    /* PPP mode: wrap IP packet in PPP HDLC framing */
    if (modem->ppp_active && modem->ppp_ctx) {
        if (!ppp_multilink_is_owner(modem->ppp_ctx))
            return 0;

        if (!(buf[12] == 0x08 && buf[13] == 0x00)) {
            modem_log(modem->log, "PPP: Dropping %d bytes (non-IP EtherType %s 0x%04X)\n",
                      io_len - 14,
                      modem_ethertype_name((uint16_t) (((uint16_t) buf[12] << 8) | buf[13])),
                      (unsigned) (((uint16_t) buf[12] << 8) | buf[13]));
            return 0; /* Non-IP */
        }

        modem_debug_log_ipv4(modem->log, "PPP network->modem", buf + 14, io_len - 14);

        while ((io_len) >= (int) (fifo8_num_free(&modem->rx_data) / 2))
            fifo8_resize_2x(&modem->rx_data);

        modem_log(modem->log, "PPP: Receiving %d bytes (EtherType IPv4 0x0800)\n", io_len - 14);
        ppp_wrap_ip(modem->ppp_ctx, buf + 14, io_len - 14);
        return 1;
    }

    while ((io_len) >= (int) (fifo8_num_free(&modem->rx_data) / 2)) {
        fifo8_resize_2x(&modem->rx_data);
    }

    if (!(buf[12] == 0x08 && buf[13] == 0x00)) {
        modem_log(modem->log, "Dropping %d bytes (non-IP EtherType %s 0x%04X)\n",
                  io_len - 14,
                  modem_ethertype_name((uint16_t) (((uint16_t) buf[12] << 8) | buf[13])),
                  (unsigned) (((uint16_t) buf[12] << 8) | buf[13]));
        return 0;
    }

    modem_log(modem->log, "Receiving %d bytes (EtherType IPv4 0x0800)\n", io_len - 14);
    /* Strip the Ethernet header. */
    io_len -= 14;
    buf += 14;
    modem_debug_log_ipv4(modem->log, "SLIP network->modem", buf, io_len);

    /* CSLIP: VJ compress before SLIP encoding */
    if (modem->cslip_enabled && modem->cslip_ctx) {
        uint8_t *comp_buf = calloc(io_len + VJ_MAX_HDR, 1);
        int      comp_type;
        if (!comp_buf)
            return 0;
        int      comp_len = cslip_compress_logged(modem->cslip_ctx, buf, io_len,
                              comp_buf, &comp_type);

        if (comp_len > 0) {
            bool queued = modem_queue_slip_frame(modem, comp_buf, (size_t) comp_len);
            free(comp_buf);
            return queued ? 1 : 0;
        }
        free(comp_buf);
        return 0;
    }

    return modem_queue_slip_frame(modem, buf, (size_t) io_len) ? 1 : 0;
}

static void
modem_accept_incoming_call(modem_t *modem)
{
    if (modem->waitingclientsocket != -1) {
        modem->clientsocket        = modem->waitingclientsocket;
        modem->waitingclientsocket = -1;
        modem_enter_connected_state(modem);
        modem->in_warmup = 250;
    } else {
        modem_enter_idle_state(modem);
    }
}

static void
modem_cmdpause_timer_callback(void *priv)
{
    modem_t *modem            = (modem_t *) priv;
    uint32_t guard_threshold = 0;
    timer_on_auto(&modem->cmdpause_timer, 1000);

    if (modem->mode == MODEM_MODE_FAX_WAIT && modem->fax_wait_ticks) {
        modem->fax_wait_ticks--;
        if (!modem->fax_wait_ticks) {
            modem->mode = MODEM_MODE_COMMAND;
            modem->cmdpause = 0;
            modem->plusinc = 0;
            modem_send_res(modem, ResOK);
        }
    }

    if (modem->ppp_active && modem->ppp_ctx) {
        ppp_timer_tick(modem->ppp_ctx);
        modem_ppp_check_dead(modem);
    }

    if (modem->tcpIpConnInProgress) {
        do {
            int status = plat_netsocket_connected(modem->clientsocket);

            if (status == -1) {
                plat_netsocket_close(modem->clientsocket);
                modem->clientsocket = -1;
                modem_enter_idle_state(modem);
                modem_sound_event(modem->sound, MODEM_SOUND_BUSY, NULL, 0);
                modem_send_res(modem, ResNOCARRIER);
                modem->tcpIpConnInProgress = 0;
                break;
            } else if (status == 1) {
                modem->tcpIpConnInProgress = 0;
                modem->tcpIpSocketConnected = true;
                break;
            }

            modem->tcpIpConnCounter++;

            if (status < 0 || (status == 0
                               && modem->tcpIpConnCounter >= MAX(5000u, modem->call_connect_at_ms + 5000u))) {
                plat_netsocket_close(modem->clientsocket);
                modem->clientsocket = -1;
                modem_enter_idle_state(modem);
                modem_send_res(modem, ResNOANSWER);
                modem->tcpIpConnInProgress = 0;
                modem->tcpIpMode           = 0;
                break;
            }
        } while (0);
    }

    if (modem->call_progress_active) {
        modem->call_progress_elapsed_ms++;
        if (modem->tcpIpSocketConnected && !modem->call_answer_sound_started
            && modem->call_progress_elapsed_ms >= modem->call_answer_at_ms) {
            modem_sound_event(modem->sound, MODEM_SOUND_ANSWER, NULL, 0);
            modem->call_answer_sound_started = true;
        }
        if (modem->tcpIpSocketConnected
            && modem->call_progress_elapsed_ms >= modem->call_connect_at_ms) {
            modem->call_progress_active = false;
            modem->tcpIpSocketConnected = false;
            modem_enter_connected_state(modem);
        }
    }

    if (!modem->connected && !modem->call_progress_active
        && modem->waitingclientsocket == -1 && modem->serversocket != -1) {
        modem->waitingclientsocket = plat_netsocket_accept(modem->serversocket);
        if (modem->waitingclientsocket != -1) {
            if (modem->dtrstate == 0 && modem->dtrmode != 0) {
                modem_enter_idle_state(modem);
            } else {
                modem->ringing = true;
                modem_send_res(modem, ResRING);
                modem->char_status ^= CHAR_COM_RI;
                char_update_status(modem->char_port);
                modem->ringtimer            = 3000;
                modem->reg[MREG_RING_COUNT] = 0;
            }
        }
    }
    if (modem->ringing) {
        if (modem->ringtimer <= 0) {
            modem->reg[MREG_RING_COUNT]++;
            if ((modem->reg[MREG_AUTOANSWER_COUNT] > 0) && (modem->reg[MREG_RING_COUNT] >= modem->reg[MREG_AUTOANSWER_COUNT])) {
                modem_accept_incoming_call(modem);
                return;
            }
            modem_send_res(modem, ResRING);
            modem->char_status ^= CHAR_COM_RI;
            char_update_status(modem->char_port);

            modem->ringtimer = 3000;
        }
        --modem->ringtimer;
    }

    if (modem->in_warmup) {
        modem->in_warmup--;
        if (modem->in_warmup == 0) {
            modem->tx_count = 0;
            fifo8_reset(&modem->rx_data);
        }
    } else if (modem->connected && modem->tcpIpMode) {
        if (modem->tx_count) {
            int wouldblock = 0;
            int res        = plat_netsocket_send(modem->clientsocket, modem->tx_pkt_ser_line, modem->tx_count, &wouldblock);

            if (res <= 0 && !wouldblock) {
                /* No bytes sent or error. */
                modem->tx_count = 0;
                modem_enter_idle_state(modem);
                modem_send_res(modem, ResNOCARRIER);
            } else if (res > 0) {
                if (res == modem->tx_count) {
                    modem->tx_count = 0;
                } else {
                    memmove(modem->tx_pkt_ser_line, &modem->tx_pkt_ser_line[res], modem->tx_count - res);
                    modem->tx_count -= res;
                }
            }
        }
        if (modem->connected && !modem->fax_rx_complete_pending
            && !modem->fax_rx_frame_waiting) {
            uint8_t buffer[1024];
            int     wouldblock = 0;
            int     recv       = MIN(modem->rx_data.capacity - modem->rx_data.num, sizeof(buffer));
            int     res        = plat_netsocket_receive(modem->clientsocket, buffer, recv, &wouldblock);

            if (res > 0) {
                if (modem->voice_class && !modem->telnet_mode) {
                    if (voice_deframe(&modem->voice_deframer, buffer, (size_t) res,
                                      modem_voice_frame_in, modem) != 0) {
                        modem->tx_count = 0;
                        modem_enter_idle_state(modem);
                        modem_send_res(modem, ResNOCARRIER);
                    }
                } else if ((modem->fax_support & MODEM_FAX_SUPPORT_CLASS_1)
                    && modem->fax_class == 1 && !modem->telnet_mode) {
                    for (int pos = 0; pos < res; pos++) {
                        fifo8_push(&modem->rx_data, buffer[pos]);
                        if (!modem->fax_rx_frame_waiting && !modem->fax_rx_complete_pending) {
                            const modem_fax_transfer_status_t status =
                                modem_fax_transfer_status(&modem->fax_rx_pending_dle, buffer[pos]);
                            if (status != MODEM_FAX_TRANSFER_ACTIVE) {
                                const bool receive_requested = modem->fax_rx_active;
                                modem->fax_rx_active = false;
                                modem->fax_rx_complete_pending = receive_requested;
                                modem->fax_rx_frame_waiting = !receive_requested;
                                modem->fax_rx_aborted = status == MODEM_FAX_TRANSFER_ABORTED;
                            }
                        }
                    }
                } else if (modem->telnet_mode)
                    modem_process_telnet(modem, buffer, res);
                else
                    fifo8_push_all(&modem->rx_data, buffer, res);
            } else if (res == 0) {
                modem->tx_count = 0;
                modem_enter_idle_state(modem);
                modem_send_res(modem, ResNOCARRIER);
            } else if (!wouldblock) {
                modem->tx_count = 0;
                modem_enter_idle_state(modem);
                modem_send_res(modem, ResNOCARRIER);
            }
        }
    }

    if (modem->mode != MODEM_MODE_FAX_TX && modem->mode != MODEM_MODE_FAX_WAIT
        && modem->mode != MODEM_MODE_VOICE_TX && modem->mode != MODEM_MODE_VOICE_RX
        && modem->mode != MODEM_MODE_VOICE_TR
        && !modem->fax_rx_active && !modem->fax_rx_complete_pending) {
        modem->cmdpause++;
        guard_threshold = (uint32_t) (modem->reg[MREG_GUARD_TIME] * 20);
        if (modem->cmdpause > guard_threshold) {
            if (modem->plusinc == 0) {
                modem->plusinc = 1;
            } else if (modem->plusinc == 4) {
                modem_log(modem->log, "Escape sequence triggered, returning to command mode\n");
                modem->mode = MODEM_MODE_COMMAND;
                modem_send_res(modem, ResOK);
                modem->plusinc = 0;
            }
        }
    }
}

/* Initialize the device for use by the user. */
static uint32_t
modem_parse_ipv4_config(const char *value)
{
    struct in_addr address;

    if (!value || !value[0] || inet_pton(AF_INET, value, &address) != 1)
        return 0;

    return ntohl(address.s_addr);
}

static void *
modem_init(UNUSED(const device_t *info))
{
    modem_t    *modem          = (modem_t *) calloc(1, sizeof(modem_t));
    const char *phonebook_file = NULL;

    if (!modem)
        return NULL;

    memset(modem->mac, 0xfc, 6);

    modem->baudrate    = 115200;
    modem->modem_identity = device_get_config_int("modem_identity");
    modem->sound = modem_sound_init();
    sound_in_add_handler(modem_voice_capture, modem);
    sound_add_handler(modem_voice_playback, modem);
    modem->listen_port = device_get_config_int("listen_port");
    modem->telnet_mode = device_get_config_int("telnet_mode");

    modem->connection_type = device_get_config_int("connection_type");
    modem->fax_support = (uint8_t) device_get_config_int("fax_voice_support");
    modem->ppp_auth_type   = device_get_config_int("ppp_auth_type");
    modem->ppp_encryption  = device_get_config_int("ppp_encryption");
    {
        const char *group = device_get_config_string("ppp_multilink_group");
        if (group)
            strncpy(modem->ppp_multilink_group, group,
                    sizeof(modem->ppp_multilink_group) - 1);
    }

    /* Shared SLIP/PPP credentials */
    {
        const char *u = device_get_config_string("username");
        const char *p = device_get_config_string("password");
        if (u) strncpy(modem->username, u, sizeof(modem->username) - 1);
        if (p) strncpy(modem->password, p, sizeof(modem->password) - 1);
    }
    modem->ppp_wins1 = modem_parse_ipv4_config(device_get_config_string("ppp_wins1"));
    modem->ppp_wins2 = modem_parse_ipv4_config(device_get_config_string("ppp_wins2"));
    modem->ppp_dns1 = modem_parse_ipv4_config(device_get_config_string("ppp_dns1"));
    modem->ppp_dns2 = modem_parse_ipv4_config(device_get_config_string("ppp_dns2"));

    modem->clientsocket = modem->serversocket = modem->waitingclientsocket = -1;

    fifo8_create(&modem->data_pending, 0x40000);
    fifo8_create(&modem->rx_data, 0x40000);
    fifo8_create(&modem->char_tx_data, 0x40000);

    timer_add(&modem->dtr_timer, modem_dtr_callback_timer, modem, 0);
    timer_add(&modem->host_to_serial_timer, host_to_modem_cb, modem, 0);
    timer_add(&modem->cmdpause_timer, modem_cmdpause_timer_callback, modem, 0);
    timer_on_auto(&modem->cmdpause_timer, 1000);
    modem->char_port = char_attach(0, modem_char_read, modem_char_write,
                                   modem_char_status, modem_char_control,
                                   modem_char_port_config, modem);
    modem->log = char_log_open(modem->char_port, "CHAR MODEM");
    modem_log(modem->log, "init()\n");
    /* Initialize CSLIP context (always available, used when selected). */
    modem->cslip_ctx = cslip_init(modem->log);
    if (modem->char_port->com.baud)
        modem->baudrate = modem->char_port->com.baud;
    timer_on_auto(&modem->host_to_serial_timer,
                  (1000000.0 / (double) modem->baudrate) * 9.0);

    modem_reset(modem);
    modem->card = network_attach_modem(modem, modem->mac, modem_rx);

    phonebook_file = device_get_config_string("phonebook_file");
    if (phonebook_file && phonebook_file[0] != 0) {
        modem_read_phonebook_file(modem, phonebook_file);
    }

    return modem;
}

void
modem_close(void *priv)
{
    modem_t *modem = (modem_t *) priv;

    /* Stop callbacks before reset/teardown.  The command-pause timer is
       periodic and rearms itself; the host-to-serial timer also rearms on
       every callback.  Leaving either enabled past this point lets it access
       modem state after the device is freed. */
    timer_disable(&modem->dtr_timer);
    timer_disable(&modem->host_to_serial_timer);
    timer_disable(&modem->cmdpause_timer);

    modem->listen_port = 0;
    modem_reset(modem);
    sound_in_remove_handler(modem_voice_capture, modem);
    sound_remove_handler(modem_voice_playback, modem);

    if (modem->ppp_ctx) {
        ppp_close(modem->ppp_ctx);
        modem->ppp_ctx = NULL;
    }
    if (modem->cslip_ctx) {
        cslip_close(modem->cslip_ctx);
        modem->cslip_ctx = NULL;
    }
    if (modem->slip_auth_ctx) {
        slip_auth_close(modem->slip_auth_ctx);
        modem->slip_auth_ctx = NULL;
    }
    modem_sound_close(modem->sound);

    modem_log(modem->log, "close()\n");
    log_close(modem->log);
    fifo8_destroy(&modem->data_pending);
    fifo8_destroy(&modem->rx_data);
    fifo8_destroy(&modem->char_tx_data);
    netcard_close(modem->card);
    free(priv);
}

// clang-format off
static const device_config_t modem_config[] = {
    {
        .name           = "modem_identity",
        .description    = "Modem Identity",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = MODEM_IDENTITY_GENERIC,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Generic 86Box Modem", .value = MODEM_IDENTITY_GENERIC },
            { .description = "Diamond SupraExpress 56e PRO", .value = MODEM_IDENTITY_SUPRAEXPRESS },
            { .description = "" }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "listen_port",
        .description    = "TCP/IP listening port",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = {
            .min =     0,
            .max = 32767
        },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "phonebook_file",
        .description    = "Phonebook File",
        .type           = CONFIG_FNAME,
        .default_string = NULL,
        .file_filter    = "Text files (*.txt)|*.txt",
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "telnet_mode",
        .description    = "Telnet emulation",
        .type           = CONFIG_BINARY,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "connection_type",
        .description    = "Connection Type",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 1,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "SLIP",                             .value = MODEM_TYPE_SLIP },
            { .description = "PPP",                              .value = MODEM_TYPE_PPP },
            { .description = "CSLIP",                            .value = MODEM_TYPE_CSLIP },
            { .description = "Microsoft RAS (NT 4.0 SP3)",       .value = MODEM_TYPE_RAS },
            { .description = "" }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "username",
        .description    = "Username",
        .type           = CONFIG_STRING,
        .default_string = "",
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "password",
        .description    = "Password",
        .type           = CONFIG_STRING,
        .default_string = "",
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "fax_voice_support",
        .description    = "Fax and Voice AT Class Support",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = MODEM_FAX_SUPPORT_DISABLED },
            { .description = "Class 0",             .value = MODEM_FAX_SUPPORT_CLASS_0 },
            { .description = "Class 1",             .value = MODEM_FAX_SUPPORT_CLASS_1 },
            { .description = "Class 8 (Voice)",     .value = MODEM_FAX_SUPPORT_CLASS_8 },
            { .description = "Class 0 + Class 8",   .value = MODEM_FAX_SUPPORT_CLASS_0 | MODEM_FAX_SUPPORT_CLASS_8 },
            { .description = "Class 0 + Class 1 + Class 8", .value = MODEM_FAX_SUPPORT_CLASS_0 | MODEM_FAX_SUPPORT_CLASS_1 | MODEM_FAX_SUPPORT_CLASS_8 },
            { .description = "" }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "ppp_auth_type",
        .description    = "PPP Authentication",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "None",       .value = 0 },
            { .description = "PAP",        .value = 1 },
            { .description = "CHAP (MD5)", .value = 2 },
            { .description = "MS-CHAP",    .value = 3 },
            { .description = "MS-CHAPv2",  .value = 4 },
            { .description = "CHAP (SHA-1)", .value = 5 },
            { .description = "CHAP (SHA-256)", .value = 6 },
            { .description = "CHAP (SHA-384)", .value = 7 },
            { .description = "CHAP (SHA-512)", .value = 8 },
            { .description = "EAP (MD5-Challenge)", .value = 9 },
            { .description = ""                       }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "ppp_encryption",
        .description    = "Require PPP Encryption",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "None",    .value = 0 },
            { .description = "40-Bit",  .value = 40 },
            { .description = "56-Bit",  .value = 56 },
            { .description = "128-Bit", .value = 128 },
            { .description = ""                   }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "ppp_multilink_group",
        .description    = "PPP Multilink Group (same on each link)",
        .type           = CONFIG_STRING,
        .default_string = "",
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "ppp_wins1",
        .description    = "PPP Primary WINS Server",
        .type           = CONFIG_STRING,
        .default_string = "",
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "ppp_wins2",
        .description    = "PPP Secondary WINS Server",
        .type           = CONFIG_STRING,
        .default_string = "",
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "ppp_dns1",
        .description    = "PPP Primary DNS Server",
        .type           = CONFIG_STRING,
        .default_string = "",
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "ppp_dns2",
        .description    = "PPP Secondary DNS Server",
        .type           = CONFIG_STRING,
        .default_string = "",
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};
// clang-format on

const device_t modem_device = {
    .name          = "Standard Hayes-compliant Modem (char API)",
    .internal_name = "char_modem",
    .flags         = DEVICE_COM | DEVICE_HOTPLUG,
    .local         = 0,
    .init          = modem_init,
    .close         = modem_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = modem_config
};
