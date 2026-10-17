#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace beckhoff {

struct AdsOptions {
    std::string net_id;
    std::string ip_address;
    std::string client_net_id;
    std::uint16_t port = 851;
    std::uint32_t timeout_ms = 1000;
    std::uint32_t notification_cycle_us = 1000;
    std::uint32_t notification_max_delay_us = 0;
};

// Implementations must stop deliveries before unsubscribe returns. Callbacks may
// run on a transport thread; they must not make synchronous ADS calls or close it.
class Transport {
public:
    using Notification = std::function<void(const std::uint8_t*, std::size_t)>;
    virtual ~Transport() = default;
    virtual std::uint32_t acquire(const std::string& name) = 0;
    virtual void release(std::uint32_t handle) noexcept = 0;
    virtual void read(std::uint32_t handle, void* data, std::size_t size) = 0;
    virtual void write(std::uint32_t handle, const void* data, std::size_t size) = 0;
    virtual std::uint32_t subscribe(std::uint32_t handle, std::size_t size,
                                    Notification callback) = 0;
    virtual void unsubscribe(std::uint32_t notification) noexcept = 0;
};

// Uses Beckhoff's standalone AdsLib; the PLC needs a preconfigured return route.
std::unique_ptr<Transport> makeAdsTransport(const AdsOptions& options);

} // namespace beckhoff
