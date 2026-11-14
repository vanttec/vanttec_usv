//
// Created by abiel on 12/9/21.
//

#include "CANHandler.h"
#include <cerrno>
#include <cstring>
#include <iostream>

namespace vanttec {
    CANHandler::CANHandler(const std::string &interfaceName) {
        epfd = epoll_create1(0);
        canfd = vanttec::socketcan_open(interfaceName);
        if (epfd == -1 || canfd == -1) {
            if (epfd) close(epfd);
            if (canfd) close(canfd);
            throw std::runtime_error("Could not initialise CANHandler fds");
        }

        int ret;
        epoll_event ev{};

        ev.data.fd = canfd;
        ev.events = EPOLLIN; //Poll for read and write

        ret = epoll_ctl(epfd, EPOLL_CTL_ADD, canfd, &ev);
        if (ret == -1) {
            close(epfd);
            close(canfd);
            throw std::runtime_error("Could not add to epoll");
        }

//        register_parser([](uint8_t id, can_frame frame){
//            std::cout << std::to_string(id) << std::endl;
//        });
    }

    void CANHandler::update_write(){
        CANMessage elem;
        while (true) {
            {
                std::lock_guard<std::mutex> lock(writeQueueMutex);
                if (writeQueue.empty()) break;
                elem = writeQueue.front();
                writeQueue.pop();
            }

            if (elem.len != 0){
                if (elem.len > CAN_MAX_DLEN) {
                    std::cerr << "Ignoring CAN frame with invalid DLC: "
                              << static_cast<int>(elem.len) << std::endl;
                    continue;
                }

                can_frame outFrame{};
                outFrame.can_dlc = elem.len;
                memcpy(outFrame.data, elem.data, elem.len);
                outFrame.can_id = elem.arb_id & CAN_SFF_MASK; // use per-message arb ID

                int retry_count = 0;
                ssize_t bytes_written;
                while ((bytes_written = ::write(canfd, &outFrame, sizeof(can_frame))) !=
                           sizeof(can_frame) &&
                       retry_count < 10) {
                    std::cerr << "CAN write failed (attempt " << (retry_count + 1)
                              << "/10): " << std::strerror(errno) << std::endl;
                    retry_count++;
                }
                if (bytes_written != sizeof(can_frame)) {
                    std::cerr << "Dropping CAN frame after 10 write attempts"
                              << std::endl;
                }
            }
        }
    }

    void CANHandler::write(const vanttec::CANMessage &msg) {
        std::lock_guard<std::mutex> lock(writeQueueMutex);
        writeQueue.push(msg);
    }

    void CANHandler::register_parser(uint32_t filter, const std::function<void(can_frame)> &parser){
        filterMsgParsers[filter].emplace_back(parser);
    }

    void CANHandler::register_parser(const std::function<void(uint8_t, can_frame)> &parser) {
        msgParsers.emplace_back(parser);
    }

    void CANHandler::update_read() {
        // Non-blocking poll: timeout=0 avoids stalling the ROS wall-timer thread
        int rdy = epoll_wait(epfd, evlist, MAX_EVENTS, 0);
        if (rdy == -1) {
            std::cerr << "Error waiting for epoll" << std::endl;
            return;
        }

        can_frame frame{};

        for (int i = 0; i < rdy; i++) {
            if(!(evlist[i].events & EPOLLIN)) continue;
            //Read available
            auto len = read(evlist[i].data.fd, &frame, sizeof(frame));
            if(len < 0) continue;

            // Route by CAN arbitration ID (HAL convention), not data[0] (Vanttec convention)
            auto id = frame.can_id & CAN_SFF_MASK;
            for(auto &parser : msgParsers) parser(static_cast<uint8_t>(id), frame);
            auto it = filterMsgParsers.find(id);
            if(it != filterMsgParsers.end())
                for(auto &parser : it->second) parser(frame);
        }
    }

    CANHandler::~CANHandler() {
        if (epfd)
            close(epfd);

        if (canfd)
            close(canfd);
    }
}