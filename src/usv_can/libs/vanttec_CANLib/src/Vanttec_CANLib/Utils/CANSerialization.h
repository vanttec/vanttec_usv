//
// Created by abiel on 12/9/21.
//

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
uint32_t serialize_float(float input);

void serialize_short(uint8_t *data, uint16_t in);

void serialize_long(uint8_t *data, uint32_t in);
#ifdef __cplusplus
}
#endif