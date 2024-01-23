//
// Created by abiel on 12/9/21.
//

#include <gtest/gtest.h>
#include "Vanttec_CANLib/Utils/CANSerialization.h"
#include "Vanttec_CANLib/Utils/CANDeserialization.h"

float serializeDeserializeFloat(float in){
    return deserialize_float(serialize_float(in));
}

TEST(CANSerialize, serializeFloat){
    float i = -1000.0;
    while(i < 1000){
        EXPECT_NEAR(serializeDeserializeFloat(i), i, 0.001);
        i += 0.01;
    }
}

TEST(CANSerialize, serializeShort){
    uint8_t data[2];
    for(uint32_t i = 0; i < std::numeric_limits<uint16_t>().max(); i++){
        serialize_short(data, i);
        EXPECT_EQ(deserialize_short(data), i);
    }
}

TEST(CANSerialize, serializeLong){
    uint8_t data[4];
    for(uint64_t i = 0; i < std::numeric_limits<uint32_t>().max(); i += 1024){
        serialize_long(data, i);
        EXPECT_EQ(deserialize_long(data), i);
    }
}