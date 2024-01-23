//
// Created by abiel on 12/9/21.
//

#pragma once

#include <cstdint>

/**
* All data transfered over CAN will be sent in big-endian byte order
 * cross-platform version of https://linux.die.net/man/3/htonl
*/
#ifdef __cplusplus
extern "C" {
#endif
uint32_t vanttec_htonl(uint32_t hostlong);

uint16_t vanttec_htons(uint16_t hostshort);

uint32_t vanttec_ntohl(uint32_t netlong);

uint16_t vanttec_ntohs(uint16_t netshort);
#ifdef __cplusplus
}
#endif