//
// Created by abiel on 12/9/21.
//

#include "CANMessage.h"
#include "Utils/CANSerialization.h"
#include "Utils/CANDeserialization.h"
#include "math.h"

#ifdef __cplusplus
namespace vanttec {
    uint8_t getId(const CANMessage &message) {
        return can_parse_id(static_cast<const uint8_t *>(message.data), message.len);
    }

    float getFloat(const CANMessage &message) {
        return can_parse_float(message.data, message.len);
    }

    void packFloat(CANMessage &message, uint8_t id, float data) {
        message.len = 5;
        can_pack_float(id, data, message.data, message.len);
    }

    uint16_t getShort(const CANMessage &message) {
        return can_parse_short(message.data, message.len);
    }

    void packShort(CANMessage &message, uint8_t id, uint16_t data) {
        message.len = 3;
        can_pack_short(id, data, message.data, message.len);
    }

    uint32_t getLong(const CANMessage &message) {
        return can_parse_long(message.data, message.len);
    }

    void packLong(CANMessage &message, uint8_t id, uint32_t data) {
        message.len = 5;
        can_pack_long(id, data, message.data, message.len);
    }

    void packByte(CANMessage &message, uint8_t id, uint8_t data){
        message.len = 2;
        can_pack_byte(id, data, message.data, message.len);
    }
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
uint8_t can_parse_id(const uint8_t *data, uint8_t len) {
    if (len == 0 || data == 0) return 0;
    return data[0];
}

uint8_t can_pack_byte(uint8_t id, uint8_t dataIn, uint8_t *data, uint8_t len){
	if(len < 2) return 0;

	data[0] = id;
	data[1] = dataIn;

	return 2;
}

uint8_t can_parse_byte(const uint8_t *data, uint8_t len){
	if(len < 2) return 0;
	return data[1];
}

uint8_t can_pack_float(uint8_t id, float n, uint8_t *data, uint8_t len) {
    return can_pack_long(id, serialize_float(n), data, len);
}

float can_parse_float(const uint8_t *data, uint8_t len) {
    if (data == 0 || len < 5) {
        return NAN;
    }

    return deserialize_float(deserialize_long(data + 1));
}

uint8_t can_pack_short(uint8_t id, uint16_t n, uint8_t *data, uint8_t len) {
    if (data == 0 || len < 3) return 0;

    data[0] = id;
    serialize_short(data + 1, n);
    return 3;
}

uint16_t can_parse_short(const uint8_t *data, uint8_t len) {
    if (len < 3) return 0;

    return deserialize_short(data + 1);
}

uint8_t can_pack_long(uint8_t id, uint32_t n, uint8_t *data, uint8_t len) {
    if (data == 0 || len < 5) return 0;

    data[0] = id;
    serialize_long(data + 1, n);
    return 5;
}

uint32_t can_parse_long(const uint8_t *data, uint8_t len) {
    if (data == 0 || len < 5) return 0;
    return deserialize_long(data + 1);
}
#ifdef __cplusplus
}
#endif
