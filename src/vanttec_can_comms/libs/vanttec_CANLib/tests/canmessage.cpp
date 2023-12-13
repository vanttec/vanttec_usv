//
// Created by abiel on 12/9/21.
//

#include <gtest/gtest.h>
#include "Vanttec_CANLib/CANMessage.h"

TEST(CANMessage, Float){
    float i = -100;
    uint8_t id = 0;
    while(i < 100){
        vanttec::CANMessage msg;
        vanttec::packFloat(msg, id, i);
        float out = vanttec::getFloat(msg);

        ASSERT_EQ(id, vanttec::getId(msg));
        ASSERT_NEAR(i, out, 0.001);
        id++;
        i += 0.01;
    }
}

TEST(CANMessage, Short){
    uint8_t id = 0;
    for(uint32_t i = 0; i < std::numeric_limits<uint16_t>().max(); i++){
        vanttec::CANMessage msg;
        vanttec::packShort(msg, id, i);
        uint16_t out = vanttec::getShort(msg);

        ASSERT_EQ(id, vanttec::getId(msg));
        ASSERT_EQ(i, out);
        id++;
    }
}

TEST(CANMessage, Long){
    uint8_t id = 0;
    for(uint64_t i = 0; i < std::numeric_limits<uint32_t>().max(); i += 2048){
        vanttec::CANMessage msg;
        vanttec::packLong(msg, id, i);

        uint32_t out = vanttec::getLong(msg);
        ASSERT_EQ(id, vanttec::getId(msg));
        ASSERT_EQ(i, out);
        id++;
    }
}