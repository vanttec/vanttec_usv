#include <iostream>
#include <cmath>
#include <limits>
#include <random>
#include <numeric>
#include <thread>
#include <chrono>
#include "Vanttec_CANLib/Utils/CANDeserialization.h"
#include "Vanttec_CANLib/Utils/CANSerialization.h"
#include "Vanttec_CANLib_Linux/CANHandler.h"

int main() {
    vanttec::CANHandler handler("can0");
//    std::thread writeThread([&]{
//        for(;;) {
//            handler.write(vanttec::CANMessage());
//            std::this_thread::sleep_for(std::chrono::seconds(10));
//        }
//    });
//
    while(true){
        handler.update_read();
    }

}
