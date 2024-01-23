//
// Created by abiel on 12/9/21.
//

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
float deserialize_float(uint32_t in);

uint16_t deserialize_short(const uint8_t *data);

uint32_t deserialize_long(const uint8_t *data);
#ifdef __cplusplus
}
#endif