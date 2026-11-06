#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "net_slirp_dhcp.h"

#define ETHERNET_HEADER_LENGTH 14
#define IPV4_MIN_HEADER_LENGTH 20
#define UDP_HEADER_LENGTH 8
#define BOOTP_FIXED_LENGTH 236
#define DHCP_COOKIE_LENGTH 4
#define DHCP_OPTIONS_OFFSET (BOOTP_FIXED_LENGTH + DHCP_COOKIE_LENGTH)

#define ETHERTYPE_IPV4 0x0800
#define IP_PROTOCOL_UDP 17
#define DHCP_SERVER_PORT 67
#define DHCP_CLIENT_PORT 68
#define DHCP_BOOTREPLY 2
#define DHCP_OPTION_PAD 0
#define DHCP_OPTION_END 255
#define DHCP_OPTION_MESSAGE_TYPE 53
#define DHCP_OPTION_WINS 44
#define DHCP_OPTION_NODE_TYPE 46
#define DHCP_OFFER 2
#define DHCP_ACK 5
#define DHCP_H_NODE 8

static uint16_t
dhcp_get_be16(const uint8_t *data)
{
    return (uint16_t) (((uint16_t) data[0] << 8) | data[1]);
}

static void
dhcp_put_be16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t) (value >> 8);
    data[1] = (uint8_t) value;
}

static uint32_t
dhcp_pseudo_header_sum(const uint8_t *ip, uint16_t udp_length)
{
    return dhcp_get_be16(ip + 12) + dhcp_get_be16(ip + 14) + dhcp_get_be16(ip + 16)
         + dhcp_get_be16(ip + 18) + IP_PROTOCOL_UDP + udp_length;
}

static uint16_t
dhcp_checksum(uint32_t sum, const uint8_t *data, size_t length)
{
    for (size_t i = 0; i + 1 < length; i += 2)
        sum += dhcp_get_be16(data + i);
    if (length & 1)
        sum += (uint32_t) data[length - 1] << 8;
    while (sum >> 16)
        sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t) ~sum;
}

static void
dhcp_update_checksums(uint8_t *ip, size_t ip_header_length, uint8_t *udp, uint16_t udp_length,
                     uint16_t old_udp_checksum)
{
    dhcp_put_be16(ip + 10, 0);
    dhcp_put_be16(ip + 10, dhcp_checksum(0, ip, ip_header_length));

    if (old_udp_checksum) {
        dhcp_put_be16(udp + 6, 0);
        uint16_t checksum = dhcp_checksum(dhcp_pseudo_header_sum(ip, udp_length), udp, udp_length);
        dhcp_put_be16(udp + 6, checksum ? checksum : 0xFFFF);
    }
}

bool
net_slirp_dhcp_add_wins_options(uint8_t *frame, size_t *frame_length, size_t frame_capacity,
                                uint32_t wins_server_ip)
{
    uint8_t *ip;
    uint8_t *udp;
    uint8_t *bootp;
    uint8_t *options;
    size_t   ip_header_length;
    size_t   ip_length;
    size_t   udp_length;
    size_t   options_length;
    size_t   end_offset = 0;
    size_t   extra = 0;
    bool     has_message_type = false;
    bool     has_wins = false;
    bool     has_node_type = false;
    bool     is_offer_or_ack = false;

    if (!frame || !frame_length || *frame_length > frame_capacity || *frame_length < ETHERNET_HEADER_LENGTH
        || dhcp_get_be16(frame + 12) != ETHERTYPE_IPV4)
        return false;

    ip = frame + ETHERNET_HEADER_LENGTH;
    if ((ip[0] >> 4) != 4)
        return false;
    ip_header_length = (size_t) (ip[0] & 0x0F) * 4;
    if (ip_header_length < IPV4_MIN_HEADER_LENGTH || *frame_length < ETHERNET_HEADER_LENGTH + ip_header_length
        || ip[9] != IP_PROTOCOL_UDP || (dhcp_get_be16(ip + 6) & 0x3FFF))
        return false;

    ip_length = dhcp_get_be16(ip + 2);
    if (ip_length < ip_header_length + UDP_HEADER_LENGTH + DHCP_OPTIONS_OFFSET
        || ip_length > *frame_length - ETHERNET_HEADER_LENGTH)
        return false;

    udp = ip + ip_header_length;
    udp_length = dhcp_get_be16(udp + 4);
    if (dhcp_get_be16(udp) != DHCP_SERVER_PORT || dhcp_get_be16(udp + 2) != DHCP_CLIENT_PORT
        || udp_length < UDP_HEADER_LENGTH + DHCP_OPTIONS_OFFSET || udp_length > ip_length - ip_header_length)
        return false;

    bootp = udp + UDP_HEADER_LENGTH;
    if (bootp[0] != DHCP_BOOTREPLY || memcmp(bootp + BOOTP_FIXED_LENGTH, "\x63\x82\x53\x63", DHCP_COOKIE_LENGTH))
        return false;

    options = bootp + DHCP_OPTIONS_OFFSET;
    options_length = udp_length - UDP_HEADER_LENGTH - DHCP_OPTIONS_OFFSET;
    for (size_t i = 0; i < options_length;) {
        uint8_t option = options[i];
        size_t  length;

        if (option == DHCP_OPTION_PAD) {
            i++;
            continue;
        }
        if (option == DHCP_OPTION_END) {
            end_offset = (size_t) (options + i - frame);
            break;
        }
        if (i + 2 > options_length)
            return false;
        length = options[i + 1];
        if (i + 2 + length > options_length)
            return false;

        if (option == DHCP_OPTION_MESSAGE_TYPE && length == 1) {
            has_message_type = true;
            is_offer_or_ack = options[i + 2] == DHCP_OFFER || options[i + 2] == DHCP_ACK;
        } else if (option == DHCP_OPTION_WINS && length >= 4) {
            has_wins = true;
        } else if (option == DHCP_OPTION_NODE_TYPE && length == 1) {
            has_node_type = true;
        }
        i += 2 + length;
    }
    if (!end_offset || !has_message_type || !is_offer_or_ack)
        return false;

    extra = (has_wins ? 0 : 6) + (has_node_type ? 0 : 3);
    if (!extra || *frame_length > frame_capacity - extra || ip_length + extra > UINT16_MAX
        || udp_length + extra > UINT16_MAX)
        return false;

    memmove(frame + end_offset + extra, frame + end_offset, *frame_length - end_offset);
    size_t at = end_offset;
    if (!has_wins) {
        frame[at++] = DHCP_OPTION_WINS;
        frame[at++] = 4;
        frame[at++] = (uint8_t) (wins_server_ip >> 24);
        frame[at++] = (uint8_t) (wins_server_ip >> 16);
        frame[at++] = (uint8_t) (wins_server_ip >> 8);
        frame[at++] = (uint8_t) wins_server_ip;
    }
    if (!has_node_type) {
        frame[at++] = DHCP_OPTION_NODE_TYPE;
        frame[at++] = 1;
        frame[at++] = DHCP_H_NODE;
    }

    ip_length += extra;
    udp_length += extra;
    dhcp_put_be16(ip + 2, (uint16_t) ip_length);
    dhcp_put_be16(udp + 4, (uint16_t) udp_length);
    dhcp_update_checksums(ip, ip_header_length, udp, (uint16_t) udp_length, dhcp_get_be16(udp + 6));
    *frame_length += extra;
    return true;
}
