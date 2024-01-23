//
// Created by abiel on 12/9/21.
//

#include "CANDeserialization.h"
#include "ByteOrder/inet.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif
float deserialize_float(uint32_t in) {
    bool sgn = in >> 31;
    uint8_t exp = (in >> 23) & 0b011111111;
    uint32_t frac = in & 0x7FFFFF;

    float out = ldexp((float) frac / 0x7FFFFF, exp - 127);
    out = sgn ? -out : out;
    return out;
}

uint16_t deserialize_short(const uint8_t *data) {
    uint16_t out;
    out = data[0] << 8;
    out |= data[1];

    return vanttec_ntohs(out);
}

uint32_t deserialize_long(const uint8_t *data) {
    uint32_t out;
    out = data[0] << (8 * 3);
    out |= data[1] << (8 * 2);
    out |= data[2] << (8);
    out |= data[3] & 0xff;

    return vanttec_ntohl(out);
}
#ifdef __cplusplus
}
#endif
