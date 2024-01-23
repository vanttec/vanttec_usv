//
// Created by abiel on 12/9/21.
//

#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <cstdint>
#include <stdexcept>
#include <cstring>

namespace vanttec {
    inline int socketcan_open(const std::string &interface) {
        int ret = 0;
        int fd = -1;
        fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
        if (fd == -1) {
            throw std::runtime_error("Error opening socketcan socket");
        }

        ifreq ifr{};
        strcpy(ifr.ifr_name, interface.c_str());
        ioctl(fd, SIOCGIFINDEX, &ifr);

//        struct can_filter rfilter[numFilters];
//        for(int i = 0; i < numFilters; i++){
//            rfilter[i].can_id = filter[i];
//            rfilter[i].can_mask = filtermask[i];
//        }
//        int ret = setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
//        if(ret < 0){
//            close(fd);
//            throw std::runtime_error("Error configuring SocketCAN filters");
//        }

        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        ret = bind(fd, (struct sockaddr *) &addr, sizeof(addr));
        if (ret < 0) {
            close(fd);
            throw std::runtime_error("Error binding to CAN interface");
        }

        //Set non blocking
        fcntl(fd, F_SETFL, O_NONBLOCK);
        return fd;
    }
}
