// Exercises the real backend against deterministic ADS API stubs. No PLC or
// AdsLib linkage is needed; headers come from the pinned upstream dependency.
#include "beckhoff/transport.hpp"
#include <AdsLib.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <future>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

void require(bool condition, const char* expression, int line) {
    if (!condition) {
        throw std::runtime_error("ADS transport test line " + std::to_string(line) + ": " + expression);
    }
}
#define CHECK(expression) require(static_cast<bool>(expression), #expression, __LINE__)

template <typename Function>
void fails(Function&& function) {
    bool failed = false;
    try {
        function();
    } catch (const std::exception&) {
        failed = true;
    }
    CHECK(failed);
}

AmsNetId local{127, 0, 0, 1, 1, 1};
long port_seed = 30000;
unsigned opened_ports = 0;
unsigned add_routes = 0;
unsigned del_routes = 0;
unsigned closed_ports = 0;
unsigned released = 0;
unsigned deleted = 0;
unsigned subscriptions = 0;
std::uint32_t expected_sample_size = 4;
bool fail_route = false;
bool fail_acquire = false;
bool short_handle = false;
bool short_read = false;
bool fail_subscribe = false;
PAdsNotificationFuncEx callback = nullptr;
std::uint32_t user_token = 0;
std::uint32_t attrs_cycle = 0;
std::uint32_t attrs_delay = 0;
constexpr std::uint32_t symbol_handle = 0x78563412;
constexpr std::uint32_t server_notification = 77;

struct Sample {
    AdsNotificationHeader header{};
    unsigned char value[4]{};
};

void run() {
    using namespace std::chrono_literals;
    beckhoff::AdsOptions options;
    options.net_id = "192.168.1.2.1.1";
    options.client_net_id = "192.168.1.3.1.1";
    options.ip_address = "192.168.1.2";
    options.notification_cycle_us = 1234;
    options.notification_max_delay_us = 56;

    for (const std::string invalid_id : {"", "1.2.3.4.5", "1.2.3.4.5.256", "1.2.3.4.5.6.7",
                                         "1.2.3.4.-1.6", "1.2.3.4.5. 6", "0.0.0.0.0.0"}) {
        auto invalid = options;
        invalid.net_id = invalid_id;
        fails([&] { beckhoff::makeAdsTransport(invalid); });
    }
    auto invalid = options;
    invalid.notification_cycle_us = std::numeric_limits<std::uint32_t>::max();
    fails([&] { beckhoff::makeAdsTransport(invalid); });
    invalid = options;
    invalid.notification_max_delay_us = std::numeric_limits<std::uint32_t>::max();
    fails([&] { beckhoff::makeAdsTransport(invalid); });
    invalid = options;
    invalid.client_net_id = "1.2.3.4.5.999";
    fails([&] { beckhoff::makeAdsTransport(invalid); });
    CHECK(opened_ports == 0);
    CHECK(add_routes == 0);

    // A failed connection restores the local Net ID and closes its port.
    fail_route = true;
    fails([&] { beckhoff::makeAdsTransport(options); });
    CHECK(local.b[0] == 127);
    CHECK(local.b[3] == 1);
    CHECK(closed_ports == 1);
    fail_route = false;

    auto transport = beckhoff::makeAdsTransport(options);
    CHECK(local.b[0] == 192);
    CHECK(local.b[3] == 3);
    auto shared = beckhoff::makeAdsTransport(options);
    CHECK(add_routes == 2); // One unsuccessful attempt and one shared route.
    invalid = options;
    invalid.client_net_id = "192.168.1.4.1.1";
    fails([&] { beckhoff::makeAdsTransport(invalid); });
    invalid = options;
    invalid.ip_address = "192.168.1.99";
    fails([&] { beckhoff::makeAdsTransport(invalid); });
    CHECK(add_routes == 2);

    fail_acquire = true;
    fails([&] { transport->acquire("MAIN.test"); });
    fail_acquire = false;
    short_handle = true;
    fails([&] { transport->acquire("MAIN.test"); });
    short_handle = false;
    fails([&] { transport->acquire(""); });
    fails([&] { transport->acquire(std::string("bad\0name", 8)); });
    CHECK(released == 0);

    const auto handle = transport->acquire("MAIN.test");
    CHECK(handle == symbol_handle);
    std::uint32_t value = 0;
    transport->read(handle, &value, sizeof(value));
    CHECK(value == 0x2a2a2a2a);
    transport->write(handle, &value, sizeof(value));
    short_read = true;
    fails([&] { transport->read(handle, &value, sizeof(value)); });
    short_read = false;
    fails([&] { transport->read(handle, nullptr, sizeof(value)); });
    fails([&] { transport->write(handle, nullptr, sizeof(value)); });
    fails([&] { transport->read(1, &value, sizeof(value)); });
    fails([&] { transport->read(handle, &value, 0); });
    fails([&] { transport->subscribe(handle, sizeof(value), {}); });

    // The pinned backend reserves stream framing space inside AdsLib's fixed
    // 4 MiB ring. Check both sides without allocating a sample-sized buffer.
    constexpr std::size_t maximum_sample_size = 4U * 1024U * 1024U - 64U;
    const auto before_subscriptions = subscriptions;
    fails([&] { transport->subscribe(handle, maximum_sample_size + 1,
                                      [](const std::uint8_t*, std::size_t) {}); });
    CHECK(subscriptions == before_subscriptions);
    expected_sample_size = static_cast<std::uint32_t>(maximum_sample_size);
    const auto boundary = transport->subscribe(handle, maximum_sample_size,
                                               [](const std::uint8_t*, std::size_t) {});
    CHECK(subscriptions == before_subscriptions + 1);
    transport->unsubscribe(boundary);
    expected_sample_size = 4;
    const auto before_deletions = deleted;

    Sample sample;
    sample.header.cbSampleSize = sizeof(sample.value);
    std::atomic<unsigned> deliveries{0};
    fail_subscribe = true;
    fails([&] {
        transport->subscribe(handle, sizeof(value), [&](const std::uint8_t*, std::size_t) { ++deliveries; });
    });
    fail_subscribe = false;
    CHECK(deleted == before_deletions + 1);
    // Even a partially registered remote notification cannot retain the user
    // callback after the subscribe operation has reported an error.
    callback(nullptr, &sample.header, user_token);
    CHECK(deliveries == 0);

    std::mutex mutex;
    std::condition_variable condition;
    bool entered = false;
    bool finished = false;
    std::atomic<bool> callback_timed_out{false};
    const auto subscription = transport->subscribe(handle, sizeof(value),
        [&](const std::uint8_t*, std::size_t length) {
            if (length != sizeof(value)) {
                callback_timed_out = true;
                return;
            }
            ++deliveries;
            std::unique_lock<std::mutex> lock(mutex);
            entered = true;
            condition.notify_all();
            if (!condition.wait_for(lock, 2s, [&] { return finished; })) {
                callback_timed_out = true;
            }
            throw std::runtime_error("Exception from an application callback");
        });
    CHECK(attrs_cycle == 12340);
    CHECK(attrs_delay == 560);
    const auto saved_callback = callback;
    const auto token = user_token;
    sample.header.cbSampleSize = 3;
    saved_callback(nullptr, &sample.header, token);
    saved_callback(nullptr, nullptr, token);
    CHECK(deliveries == 0);
    sample.header.cbSampleSize = sizeof(sample.value);

    // Futures and bounded callback waits also prevent assertion failures from
    // leaving a blocked joinable worker behind during stack unwinding.
    auto producer = std::async(std::launch::async, [&] { saved_callback(nullptr, &sample.header, token); });
    {
        std::unique_lock<std::mutex> lock(mutex);
        CHECK(condition.wait_for(lock, 2s, [&] { return entered; }));
    }
    auto stopped = std::async(std::launch::async, [&] { transport->unsubscribe(subscription); });
    CHECK(stopped.wait_for(30ms) == std::future_status::timeout);
    {
        std::lock_guard<std::mutex> lock(mutex);
        finished = true;
    }
    condition.notify_all();
    CHECK(stopped.wait_for(2s) == std::future_status::ready);
    stopped.get();
    CHECK(producer.wait_for(2s) == std::future_status::ready);
    producer.get();
    CHECK(!callback_timed_out);
    saved_callback(nullptr, &sample.header, token);
    CHECK(deliveries == 1);
    CHECK(deleted == before_deletions + 2);
    transport->unsubscribe(subscription);
    CHECK(deleted == before_deletions + 2);

    transport->release(handle);
    transport->release(handle);
    CHECK(released == 1);
    fails([&] { transport->read(handle, &value, sizeof(value)); });

    // Successfully acquired handles remain owned when a later acquire fails,
    // and RAII destruction releases them without needing the caller's cleanup.
    auto partial = beckhoff::makeAdsTransport(options);
    CHECK(partial->acquire("MAIN.first") == symbol_handle);
    fail_acquire = true;
    fails([&] { partial->acquire("MAIN.second"); });
    fail_acquire = false;
    partial.reset();
    CHECK(released == 2);
    CHECK(del_routes == 0);
    transport.reset();
    CHECK(del_routes == 0);
    shared.reset();
    CHECK(del_routes == 1);
    CHECK(opened_ports == closed_ports);
}

} // namespace

// These definitions intentionally implement only the ABI used by the backend.
AmsNetId::AmsNetId(std::uint32_t) : b{} {}
AmsNetId::AmsNetId(std::uint8_t a, std::uint8_t c, std::uint8_t d,
                   std::uint8_t e, std::uint8_t f, std::uint8_t g) : b{a, c, d, e, f, g} {}
long AdsPortOpenEx() { ++opened_ports; return ++port_seed; }
long AdsPortCloseEx(long) { ++closed_ports; return 0; }
long AdsGetLocalAddressEx(long port, AmsAddr* result) {
    result->netId = local;
    result->port = static_cast<std::uint16_t>(port);
    return 0;
}
long AdsSyncSetTimeoutEx(long, std::uint32_t timeout) { CHECK(timeout == 1000); return 0; }
void AdsSetLocalAddress(AmsNetId id) { local = id; }
long AdsAddRoute(AmsNetId, const char*) { ++add_routes; return fail_route ? 6 : 0; }
void AdsDelRoute(AmsNetId) { ++del_routes; }
long AdsSyncReadWriteReqEx2(long, const AmsAddr*, std::uint32_t group, std::uint32_t,
                           std::uint32_t length, void* result, std::uint32_t name_length,
                           const void* name, std::uint32_t* read) {
    CHECK(group == ADSIGRP_SYM_HNDBYNAME);
    CHECK(length == 4);
    CHECK(name_length > 1);
    CHECK(static_cast<const char*>(name)[name_length - 1] == '\0');
    if (fail_acquire) return 1808;
    const unsigned char raw[4] = {0x12, 0x34, 0x56, 0x78};
    std::memcpy(result, raw, 4);
    *read = short_handle ? 3 : 4;
    return 0;
}
long AdsSyncReadReqEx2(long, const AmsAddr*, std::uint32_t group, std::uint32_t handle,
                      std::uint32_t length, void* result, std::uint32_t* read) {
    CHECK(group == ADSIGRP_SYM_VALBYHND);
    CHECK(handle == symbol_handle);
    std::memset(result, 42, length);
    *read = short_read ? length - 1 : length;
    return 0;
}
long AdsSyncWriteReqEx(long, const AmsAddr*, std::uint32_t group, std::uint32_t handle,
                      std::uint32_t length, const void* data) {
    if (group == ADSIGRP_SYM_RELEASEHND) {
        const unsigned char raw[4] = {0x12, 0x34, 0x56, 0x78};
        CHECK(length == 4);
        CHECK(std::memcmp(raw, data, 4) == 0);
        ++released;
    } else {
        CHECK(group == ADSIGRP_SYM_VALBYHND);
        CHECK(handle == symbol_handle);
    }
    return 0;
}
long AdsSyncAddDeviceNotificationReqEx(long, const AmsAddr*, std::uint32_t group,
                                       std::uint32_t handle, const AdsNotificationAttrib* attributes,
                                       PAdsNotificationFuncEx function, std::uint32_t token,
                                       std::uint32_t* notification) {
    CHECK(group == ADSIGRP_SYM_VALBYHND);
    CHECK(handle == symbol_handle);
    CHECK(attributes->nTransMode == ADSTRANS_SERVERONCHA);
    CHECK(attributes->cbLength == expected_sample_size);
    ++subscriptions;
    attrs_cycle = attributes->nCycleTime;
    attrs_delay = attributes->nMaxDelay;
    callback = function;
    user_token = token;
    *notification = server_notification;
    return fail_subscribe ? 1808 : 0;
}
long AdsSyncDelDeviceNotificationReqEx(long, const AmsAddr*, std::uint32_t notification) {
    CHECK(notification == server_notification);
    ++deleted;
    return 0;
}

int main() {
    std::mutex watchdog_mutex;
    std::condition_variable watchdog_condition;
    bool completed = false;
    std::thread watchdog([&] {
        std::unique_lock<std::mutex> lock(watchdog_mutex);
        if (!watchdog_condition.wait_for(lock, std::chrono::seconds(10), [&] { return completed; })) {
            std::cerr << "ADS transport test exceeded its 10 second deadline\n";
            std::_Exit(2);
        }
    });
    int result = 0;
    try {
        run();
        std::cout << "ADS transport tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    {
        std::lock_guard<std::mutex> lock(watchdog_mutex);
        completed = true;
    }
    watchdog_condition.notify_all();
    watchdog.join();
    return result;
}
