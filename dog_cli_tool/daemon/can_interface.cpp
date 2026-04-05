#include "can_interface.hpp"
#include "log.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <linux/can/raw.h>
#include <mutex>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

std::string ErrnoMsg() {
    return std::strerror(errno);
}

}  // namespace

CANInterface::CANInterface(const char* can_if) : if_name_(can_if ? can_if : "") {
    sockaddr_can addr{};
    ifreq ifr{};

    can_socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        LOG("ERROR", "CAN", if_name_.c_str(), ("socket failed: " + ErrnoMsg()).c_str());
        return;
    }

    std::strncpy(ifr.ifr_name, if_name_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        LOG("ERROR", "CAN", if_name_.c_str(), ("ioctl failed: " + ErrnoMsg()).c_str());
        ::close(can_socket_);
        can_socket_ = -1;
        return;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(can_socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        LOG("ERROR", "CAN", if_name_.c_str(), ("bind failed: " + ErrnoMsg()).c_str());
        ::close(can_socket_);
        can_socket_ = -1;
        return;
    }

    running_ = true;
    can_thread_ = std::thread([this]() {
        struct can_frame frame{};
        while (running_) {
            const int nbytes = ::read(can_socket_, &frame, sizeof(frame));
            if (nbytes < 0) {
                if (errno != EAGAIN && errno != EBADF) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                continue;
            }
            if (nbytes < static_cast<int>(sizeof(frame))) {
                continue;
            }

            std::vector<can_rx_callback_t> callbacks;
            std::vector<void*> user_ptrs;
            {
                std::lock_guard<std::mutex> lock(filter_mutex_);
                for (const auto& info : filter_info_) {
                    bool match = true;
                    const bool frame_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
                    const bool filter_extended = (info.filter.flags & CAN_FILTER_IDE) != 0;
                    if (info.filter.mask != 0 && frame_extended != filter_extended) {
                        match = false;
                    }
                    if (match) {
                        const uint32_t clean_id = frame.can_id & CAN_EFF_MASK;
                        if ((clean_id & info.filter.mask) != (info.filter.id & info.filter.mask)) {
                            match = false;
                        }
                    }
                    if (match && info.callback != nullptr) {
                        callbacks.push_back(info.callback);
                        user_ptrs.push_back(info.user_data);
                    }
                }
            }

            struct can_frame frame_copy = frame;
            for (size_t i = 0; i < callbacks.size(); ++i) {
                callbacks[i](nullptr, &frame_copy, user_ptrs[i]);
            }
        }
    });
}

CANInterface::~CANInterface() {
    running_ = false;
    if (can_socket_ >= 0) {
        ::close(can_socket_);
        can_socket_ = -1;
    }
    if (can_thread_.joinable()) {
        can_thread_.join();
    }
}

const char* CANInterface::GetName() const {
    return if_name_.c_str();
}

int CANInterface::SendMessage(const struct can_frame* frame) {
    if (frame == nullptr || can_socket_ < 0) {
        return -1;
    }
    const int nbytes = ::write(can_socket_, frame, sizeof(*frame));
    if (nbytes != static_cast<int>(sizeof(*frame))) {
        LOG("ERROR", "CAN", if_name_.c_str(), ("write failed: " + ErrnoMsg()).c_str());
        return -1;
    }
    return 0;
}

int CANInterface::SetFilter(struct can_filter filter, can_rx_callback_t callback, void* user_data) {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    filter_info_.push_back({filter, callback, user_data});
    return 0;
}
