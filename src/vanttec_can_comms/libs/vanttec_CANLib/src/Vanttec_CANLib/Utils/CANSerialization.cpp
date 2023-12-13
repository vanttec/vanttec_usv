//
// Created by abiel on 12/9/21.
//

#include "CANSerialization.h"
#include "ByteOrder/inet.h"
#include <math.h>

uint32_t serialize_float(float input) {
    uint32_t out = 0;
    out |= (input < 0) << 31; //sgn

    int exp;
    float frac = frexp(input, &exp);
    uint8_t ieeeExp = (uint8_t) exp + 127;
    uint32_t mant = fabs(frac) * 0x7fffff;

    out |= ieeeExp << 23;
    out |= mant & 0x7fffff;
    return out;
}

void serialize_short(uint8_t *data, uint16_t in) {
    in = vanttec_htons(in);
    data[0] = in >> 8;
    data[1] = in & 0xFF;
}

void serialize_long(uint8_t *data, uint32_t in) {
    in = vanttec_htonl(in);
    data[0] = in >> (8 * 3);
    data[1] = in >> (8 * 2);
    data[2] = in >> 8;
    data[3] = in & 0xFF;
}
