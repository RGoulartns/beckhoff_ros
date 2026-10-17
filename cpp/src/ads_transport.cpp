#include "beckhoff/transport.hpp"

#include <AdsLib.h>

#include <array>
#include <condition_variable>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace beckhoff {
namespace {

using NetId = std::array<std::uint8_t, 6>;

NetId parseNetId(const std::string& text) {
    NetId result{};
    std::size_t position = 0;
    for (std::size_t part = 0; part < result.size(); ++part) {
        unsigned value = 0;
        const auto start = position;
        while (position < text.size() && text[position] >= '0' && text[position] <= '9') {
            value = value * 10 + static_cast<unsigned>(text[position++] - '0');
            if (value > 255 || position - start > 3) {
                throw std::invalid_argument("AMS Net ID must contain six decimal octets: " + text);
            }
        }
        if (position == start || (part < 5 && (position == text.size() || text[position++] != '.')) ||
            (part == 5 && position != text.size())) {
            throw std::invalid_argument("AMS Net ID must contain six decimal octets: " + text);
        }
        result[part] = static_cast<std::uint8_t>(value);
    }
    if (result == NetId{}) {
        throw std::invalid_argument("AMS Net ID must not be all zero");
    }
    return result;
}

AmsNetId adsNetId(const NetId& net_id) {
    return {net_id[0], net_id[1], net_id[2], net_id[3], net_id[4], net_id[5]};
}

NetId bytes(const AmsNetId& net_id) {
    return {net_id.b[0], net_id.b[1], net_id.b[2], net_id.b[3], net_id.b[4], net_id.b[5]};
}

void check(long status, const char* operation) {
    if (status != 0) {
        throw std::runtime_error(std::string(operation) + " failed with ADS error " + std::to_string(status));
    }
}

std::uint32_t checkedSize(std::size_t size) {
    if (size == 0 || size > std::numeric_limits<std::uint32_t>::max()) {
        throw std::invalid_argument("ADS buffer size must be between 1 and UINT32_MAX bytes");
    }
    return static_cast<std::uint32_t>(size);
}

std::uint32_t hundredNanoseconds(std::uint32_t microseconds) {
    if (microseconds > std::numeric_limits<std::uint32_t>::max() / 10) {
        throw std::invalid_argument("ADS notification interval exceeds the 100 ns timer range");
    }
    return microseconds * 10;
}

struct Route {
    std::string ip;
    std::size_t references = 0;
};

struct Routes {
    std::mutex mutex;
    std::map<NetId, Route> routes;
    NetId local{};
};

// AdsLib itself has process-wide router state. Keep our coordination alive until
// process exit as well, including during destruction of global client objects.
Routes& sharedRoutes() {
    static auto* routes = new Routes;
    return *routes;
}

void acquireRoute(const NetId& target, const std::string& ip,
                  const NetId& requested_local, long port) {
    auto& state = sharedRoutes();
    std::lock_guard<std::mutex> lock(state.mutex);
    AmsAddr local{};
    check(AdsGetLocalAddressEx(port, &local), "Reading local AMS address");
    const auto previous_local = bytes(local.netId);
    if (state.local != NetId{} && state.local != previous_local) {
        throw std::runtime_error("AdsLib's process-wide local AMS Net ID was changed by another client");
    }
    if (state.local != NetId{} && requested_local != NetId{} && requested_local != previous_local) {
        throw std::invalid_argument("All AdsLib clients in a process must use the same local AMS Net ID");
    }
    const auto existing = state.routes.find(target);
    if (existing != state.routes.end()) {
        if (existing->second.ip != ip) {
            throw std::invalid_argument("An AMS Net ID already has a local route to another IP address");
        }
        if (existing->second.references == std::numeric_limits<std::size_t>::max()) {
            throw std::overflow_error("Too many shared ADS routes");
        }
        ++existing->second.references;
        return;
    }

    // Allocate bookkeeping before touching the process-wide router. Only the
    // client-side route is added; the PLC's return route must already exist.
    auto inserted = state.routes.emplace(target, Route{ip, 0});
    bool added = false;
    // The standalone router starts with a default local address. An explicit
    // address may replace that default before our first successful route.
    const bool changed_local = requested_local != NetId{} && previous_local != requested_local;
    try {
        if (changed_local) {
            AdsSetLocalAddress(adsNetId(requested_local));
        }
        check(AdsAddRoute(adsNetId(target), ip.c_str()), "Adding local ADS route");
        added = true;
        check(AdsGetLocalAddressEx(port, &local), "Reading local AMS address");
        if (bytes(local.netId) == NetId{}) {
            throw std::runtime_error("AdsLib did not establish a local AMS Net ID");
        }
        state.local = bytes(local.netId);
        inserted.first->second.references = 1;
    } catch (...) {
        if (added) {
            try {
                AdsDelRoute(adsNetId(target));
            } catch (...) {
            }
        }
        if (changed_local) {
            try {
                AdsSetLocalAddress(adsNetId(previous_local));
            } catch (...) {
            }
        }
        state.routes.erase(inserted.first);
        throw;
    }
}

void releaseRoute(const NetId& target) noexcept {
    try {
        auto& state = sharedRoutes();
        std::lock_guard<std::mutex> lock(state.mutex);
        const auto route = state.routes.find(target);
        if (route != state.routes.end() && --route->second.references == 0) {
            AdsDelRoute(adsNetId(target));
            state.routes.erase(route);
        }
    } catch (...) {
        // Destruction must not throw, including when the connection was lost.
    }
}

struct CallbackEntry {
    std::mutex mutex;
    std::condition_variable quiet;
    Transport::Notification callback;
    std::size_t size;
    std::size_t active = 0;
    bool enabled = true;

    CallbackEntry(Transport::Notification function, std::size_t length)
        : callback(std::move(function)), size(length) {}
};

struct Callbacks {
    std::mutex mutex;
    std::unordered_map<std::uint32_t, std::shared_ptr<CallbackEntry>> entries;
    std::uint64_t next_token = 1;
};

Callbacks& sharedCallbacks() {
    static auto* callbacks = new Callbacks;
    return *callbacks;
}

std::uint32_t registerCallback(const std::shared_ptr<CallbackEntry>& entry) {
    auto& state = sharedCallbacks();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.next_token > std::numeric_limits<std::uint32_t>::max()) {
        throw std::overflow_error("ADS notification tokens exhausted");
    }
    // Never recycle tokens: a delayed server notification cannot reach a newer
    // subscription, even after an unsuccessful server-side unsubscribe.
    const auto token = static_cast<std::uint32_t>(state.next_token++);
    state.entries.emplace(token, entry);
    return token;
}

void stopCallback(std::uint32_t token, const std::shared_ptr<CallbackEntry>& entry) {
    {
        auto& state = sharedCallbacks();
        std::lock_guard<std::mutex> lock(state.mutex);
        state.entries.erase(token);
    }
    std::unique_lock<std::mutex> lock(entry->mutex);
    entry->enabled = false;
    entry->quiet.wait(lock, [&] { return entry->active == 0; });
}

void notificationCallback(const AmsAddr*, const AdsNotificationHeader* sample,
                          std::uint32_t token) noexcept {
    try {
        std::shared_ptr<CallbackEntry> entry;
        {
            auto& state = sharedCallbacks();
            std::lock_guard<std::mutex> lock(state.mutex);
            const auto found = state.entries.find(token);
            if (found == state.entries.end()) {
                return;
            }
            entry = found->second;
        }
        {
            std::lock_guard<std::mutex> lock(entry->mutex);
            if (!entry->enabled || !sample || sample->cbSampleSize != entry->size) {
                return;
            }
            ++entry->active;
        }
        try {
            const auto* data = reinterpret_cast<const std::uint8_t*>(sample + 1);
            entry->callback(data, sample->cbSampleSize);
        } catch (...) {
            // Never let application exceptions escape into the AdsLib thread.
        }
        {
            std::lock_guard<std::mutex> lock(entry->mutex);
            if (--entry->active == 0) {
                entry->quiet.notify_all();
            }
        }
    } catch (...) {
        // This is a C callback boundary, including for allocation/locking errors.
    }
}

class AdsTransport final : public Transport {
public:
    explicit AdsTransport(const AdsOptions& options)
        : target_(parseNetId(options.net_id)), address_{adsNetId(target_), options.port},
          cycle_(hundredNanoseconds(options.notification_cycle_us)),
          max_delay_(hundredNanoseconds(options.notification_max_delay_us)) {
        const auto local = options.client_net_id.empty() ? NetId{} : parseNetId(options.client_net_id);
        if (options.ip_address.empty() || options.ip_address.find('\0') != std::string::npos) {
            throw std::invalid_argument("ADS IP address or hostname must be nonempty and contain no NUL");
        }
        if (options.port == 0 || options.timeout_ms == 0) {
            throw std::invalid_argument("ADS port and timeout must be nonzero");
        }
        port_ = AdsPortOpenEx();
        if (port_ == 0) {
            throw std::runtime_error("AdsPortOpenEx failed: no local ADS ports available");
        }
        try {
            check(AdsSyncSetTimeoutEx(port_, options.timeout_ms), "Setting ADS timeout");
            acquireRoute(target_, options.ip_address, local, port_);
        } catch (...) {
            try {
                AdsPortCloseEx(port_);
            } catch (...) {
            }
            port_ = 0;
            throw;
        }
    }

    ~AdsTransport() override {
        // Users must stop concurrent transport operations before destruction.
        while (!notifications_.empty()) {
            unsubscribe(notifications_.begin()->first);
        }
        for (const auto& handle : handles_) {
            for (std::size_t count = 0; count < handle.second; ++count) {
                releaseRemote(handle.first);
            }
        }
        try {
            AdsPortCloseEx(port_);
        } catch (...) {
        }
        releaseRoute(target_);
    }

    std::uint32_t acquire(const std::string& name) override {
        if (name.empty() || name.find('\0') != std::string::npos ||
            name.size() >= std::numeric_limits<std::uint32_t>::max()) {
            throw std::invalid_argument("ADS symbol name must be nonempty, NUL-free and fit in a 32-bit buffer");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        std::array<std::uint8_t, 4> raw{};
        std::uint32_t read = 0;
        check(AdsSyncReadWriteReqEx2(port_, &address_, ADSIGRP_SYM_HNDBYNAME, 0,
                                    static_cast<std::uint32_t>(raw.size()), raw.data(),
                                    static_cast<std::uint32_t>(name.size() + 1), name.c_str(), &read),
              "Acquiring ADS symbol handle");
        if (read != raw.size()) {
            throw std::runtime_error("ADS symbol handle response has an incorrect length");
        }
        const auto handle = static_cast<std::uint32_t>(raw[0]) |
                            (static_cast<std::uint32_t>(raw[1]) << 8) |
                            (static_cast<std::uint32_t>(raw[2]) << 16) |
                            (static_cast<std::uint32_t>(raw[3]) << 24);
        if (handle == 0) {
            throw std::runtime_error("ADS returned an invalid zero symbol handle");
        }
        try {
            auto& references = handles_[handle];
            if (references == std::numeric_limits<std::size_t>::max()) {
                throw std::overflow_error("Too many references to ADS symbol handle");
            }
            ++references;
        } catch (...) {
            releaseRemote(handle);
            throw;
        }
        return handle;
    }

    void release(std::uint32_t handle) noexcept override {
        try {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto owned = handles_.find(handle);
            if (owned == handles_.end()) {
                return;
            }
            releaseRemote(handle);
            if (--owned->second == 0) {
                handles_.erase(owned);
            }
        } catch (...) {
        }
    }

    void read(std::uint32_t handle, void* data, std::size_t size) override {
        const auto length = checkedSize(size);
        if (!data) {
            throw std::invalid_argument("ADS read buffer must not be null");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        requireHandle(handle);
        std::uint32_t read = 0;
        check(AdsSyncReadReqEx2(port_, &address_, ADSIGRP_SYM_VALBYHND, handle, length, data, &read),
              "Reading ADS symbol");
        if (read != length) {
            throw std::runtime_error("ADS read returned an incorrect number of bytes");
        }
    }

    void write(std::uint32_t handle, const void* data, std::size_t size) override {
        const auto length = checkedSize(size);
        if (!data) {
            throw std::invalid_argument("ADS write buffer must not be null");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        requireHandle(handle);
        check(AdsSyncWriteReqEx(port_, &address_, ADSIGRP_SYM_VALBYHND, handle, length, data),
              "Writing ADS symbol");
    }

    std::uint32_t subscribe(std::uint32_t handle, std::size_t size, Notification callback) override {
        const auto length = checkedSize(size);
        // Pinned standalone AdsLib's NotificationDispatcher has a fixed 4 MiB
        // ring. Leave 64 bytes for stream, timestamp and sample framing so a
        // single sample can fit. Backlogged/batched notifications still depend
        // on application callback throughput and the server's batching policy.
        constexpr std::size_t maximum_sample_size = 4U * 1024U * 1024U - 64U;
        if (size > maximum_sample_size) {
            throw std::invalid_argument("ADS notification sample exceeds AdsLib's 4 MiB ring capacity");
        }
        if (!callback) {
            throw std::invalid_argument("ADS notification callback must not be empty");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        requireHandle(handle);
        auto entry = std::make_shared<CallbackEntry>(std::move(callback), size);
        const auto token = registerCallback(entry);
        std::uint32_t notification = 0;
        try {
            // Allocate local bookkeeping before activating the remote callback.
            auto inserted = notifications_.emplace(token, Subscription{0, entry});
            AdsNotificationAttrib attributes{};
            attributes.cbLength = length;
            attributes.nTransMode = ADSTRANS_SERVERONCHA;
            attributes.nCycleTime = cycle_;
            attributes.nMaxDelay = max_delay_;
            check(AdsSyncAddDeviceNotificationReqEx(port_, &address_, ADSIGRP_SYM_VALBYHND,
                                                    handle, &attributes, notificationCallback,
                                                    token, &notification),
                  "Subscribing to ADS symbol");
            inserted.first->second.handle = notification;
        } catch (...) {
            stopCallback(token, entry);
            notifications_.erase(token);
            if (notification != 0) {
                try {
                    AdsSyncDelDeviceNotificationReqEx(port_, &address_, notification);
                } catch (...) {
                }
            }
            throw;
        }
        return token;
    }

    void unsubscribe(std::uint32_t token) noexcept override {
        try {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto found = notifications_.find(token);
            if (found == notifications_.end()) {
                return;
            }
            stopCallback(token, found->second.entry);
            // Erase locally even if the PLC is disconnected. A late callback
            // has no registry entry and cannot reach released application data.
            const auto handle = found->second.handle;
            notifications_.erase(found);
            AdsSyncDelDeviceNotificationReqEx(port_, &address_, handle);
        } catch (...) {
        }
    }

private:
    struct Subscription {
        std::uint32_t handle;
        std::shared_ptr<CallbackEntry> entry;
    };

    void requireHandle(std::uint32_t handle) const {
        if (handles_.find(handle) == handles_.end()) {
            throw std::invalid_argument("ADS symbol handle does not belong to this transport");
        }
    }

    void releaseRemote(std::uint32_t handle) noexcept {
        const std::array<std::uint8_t, 4> raw{
            static_cast<std::uint8_t>(handle), static_cast<std::uint8_t>(handle >> 8),
            static_cast<std::uint8_t>(handle >> 16), static_cast<std::uint8_t>(handle >> 24)};
        try {
            AdsSyncWriteReqEx(port_, &address_, ADSIGRP_SYM_RELEASEHND, 0,
                              static_cast<std::uint32_t>(raw.size()), raw.data());
        } catch (...) {
        }
    }

    NetId target_;
    AmsAddr address_;
    std::uint32_t cycle_;
    std::uint32_t max_delay_;
    long port_ = 0;
    std::mutex mutex_;
    std::unordered_map<std::uint32_t, std::size_t> handles_;
    std::unordered_map<std::uint32_t, Subscription> notifications_;
};

} // namespace

std::unique_ptr<Transport> makeAdsTransport(const AdsOptions& options) {
    return std::make_unique<AdsTransport>(options);
}

} // namespace beckhoff
