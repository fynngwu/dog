#pragma once

#include <atomic>
#include <cstdint>
#include <linux/can.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifndef BIT
#define BIT(n) (1UL << (n))
#endif

/** Filter flag used to match extended 29-bit CAN identifiers. */
#define CAN_FILTER_IDE BIT(0)

/** Forward declaration used by the callback signature. */
struct device;

/**
 * @brief Minimal Linux SocketCAN wrapper with callback-based frame dispatch.
 *
 * The reference implementation binds one SocketCAN interface per USB-CAN dongle
 * and keeps a dedicated RX thread alive for the lifetime of the object.  This
 * class preserves that behavior while adding documentation and a small amount of
 * defensive bookkeeping.
 */
class CANInterface {
public:
    /** Callback signature used when a received frame matches a software filter. */
    using can_rx_callback_t = void (*)(const struct device* dev,
                                       struct can_frame* frame,
                                       void* user_data);

    /**
     * @brief Software filter description.
     *
     * The logic is compatible with the reference implementation: ID/mask are
     * matched in user space after every received frame, and @ref CAN_FILTER_IDE
     * can be used to require extended identifiers.
     */
    struct can_filter {
        uint32_t id = 0;
        uint32_t mask = 0;
        uint8_t flags = 0;
    };

    /**
     * @brief Open and bind a Linux SocketCAN interface.
     *
     * @param can_if Interface name such as {@code candle0}.
     */
    explicit CANInterface(const char* can_if);

    /**
     * @brief Stop the RX thread and close the SocketCAN file descriptor.
     */
    ~CANInterface();

    /**
     * @brief Return the interface name used during construction.
     */
    const char* GetName() const;

    /**
     * @brief Report whether the SocketCAN file descriptor is valid.
     */
    bool IsValid() const { return can_socket_ >= 0; }

    /**
     * @brief Send one raw CAN frame.
     *
     * @param frame Frame to transmit.
     * @return 0 on success, -1 on failure.
     */
    int SendMessage(const struct can_frame* frame);

    /**
     * @brief Register one software RX filter and callback pair.
     *
     * @param filter ID/mask/flag filter.
     * @param callback User callback invoked for every matching frame.
     * @param user_data Opaque pointer forwarded to @p callback.
     * @return Always 0 for API compatibility.
     */
    int SetFilter(struct can_filter filter, can_rx_callback_t callback, void* user_data);

private:
    /** Internal filter/callback bundle. */
    struct can_filter_info {
        struct can_filter filter;
        can_rx_callback_t callback = nullptr;
        void* user_data = nullptr;
    };

    int can_socket_ = -1;
    std::vector<struct can_filter_info> filter_info_;
    std::string if_name_;
    std::thread can_thread_;
    std::atomic<bool> running_{false};
    std::mutex filter_mutex_;
};
