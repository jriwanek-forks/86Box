#ifndef NETWORK_NET_SLIRP_DHCP_H
#define NETWORK_NET_SLIRP_DHCP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool net_slirp_dhcp_add_wins_options(uint8_t *frame, size_t *frame_length, size_t frame_capacity,
                                    uint32_t wins_server_ip);

#endif
