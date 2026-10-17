#include "beckhoff/beckhoff_plc.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using beckhoff::BeckhoffPLC;
using beckhoff::Json;

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template<class F> void rejects(F&& action, const char* message) {
    try { action(); } catch (const std::exception&) { return; }
    throw std::runtime_error(message);
}

struct State {
    struct Subscription { std::uint32_t handle; beckhoff::Transport::Notification callback; };
    std::vector<std::string> calls;
    std::map<std::uint32_t, std::vector<std::uint8_t>> data;
    std::map<std::uint32_t, Subscription> subscriptions;
    std::size_t acquisitions = 0;
    std::size_t subscription_attempts = 0;
    std::size_t reads = 0;
    std::size_t writes = 0;
    std::size_t fail_acquire = 0;
    std::size_t fail_subscribe = 0;
    bool fail_read = false;
    bool destroyed = false;

    void emit(std::uint32_t handle, const std::uint8_t* bytes, std::size_t size) {
        for (const auto& subscription : subscriptions)
            if (subscription.second.handle == handle)
                subscription.second.callback(bytes, size);
    }
};

class MockTransport final : public beckhoff::Transport {
public:
    explicit MockTransport(std::shared_ptr<State> state) : state_(std::move(state)) {}
    ~MockTransport() override { state_->destroyed = true; }

    std::uint32_t acquire(const std::string& name) override {
        ++state_->acquisitions;
        state_->calls.push_back("acquire:" + name);
        if (state_->fail_acquire == state_->acquisitions) throw std::runtime_error("acquire failure");
        const auto handle = static_cast<std::uint32_t>(state_->acquisitions + 100);
        state_->data.emplace(handle, std::vector<std::uint8_t>(64));
        return handle;
    }
    void release(std::uint32_t handle) noexcept override {
        state_->calls.push_back("release:" + std::to_string(handle));
        state_->data.erase(handle);
    }
    void read(std::uint32_t handle, void* data, std::size_t size) override {
        ++state_->reads;
        if (state_->fail_read) throw std::runtime_error("read failure");
        const auto& bytes = state_->data.at(handle);
        if (bytes.size() < size) throw std::runtime_error("mock short read");
        std::memcpy(data, bytes.data(), size);
    }
    void write(std::uint32_t handle, const void* data, std::size_t size) override {
        ++state_->writes;
        const auto* bytes = static_cast<const std::uint8_t*>(data);
        state_->data.at(handle).assign(bytes, bytes + size);
    }
    std::uint32_t subscribe(std::uint32_t handle, std::size_t, Notification callback) override {
        ++state_->subscription_attempts;
        state_->calls.push_back("subscribe:" + std::to_string(handle));
        if (state_->fail_subscribe == state_->subscription_attempts) throw std::runtime_error("subscribe failure");
        const auto token = static_cast<std::uint32_t>(state_->subscription_attempts + 200);
        state_->subscriptions.emplace(token, State::Subscription{handle, std::move(callback)});
        return token;
    }
    void unsubscribe(std::uint32_t notification) noexcept override {
        state_->calls.push_back("unsubscribe:" + std::to_string(notification));
        state_->subscriptions.erase(notification);
    }
private:
    std::shared_ptr<State> state_;
};

Json configuration() {
    return Json::parse(R"({"PLC": {
        "structures": {"Status": {"count": "uint16", "enabled": "bool"}},
        "variables": {"GVL.status": "Status", "GVL.flag": "bool", "GVL.samples": ["uint16", 2]},
        "variables_to_monitor": ["GVL.status", "GVL.flag"],
        "variables_to_write": ["GVL.status", "GVL.samples"]
    }})");
}

std::unique_ptr<MockTransport> transport(const std::shared_ptr<State>& state) {
    return std::make_unique<MockTransport>(state);
}

void lifecycleAndValues() {
    auto state = std::make_shared<State>();
    {
        BeckhoffPLC plc(configuration(), transport(state));
        check(state->acquisitions == 3 && state->subscription_attempts == 2, "startup acquires and subscribes once");
        const auto& status = plc.variable("GVL.status");
        check(status.name() == "GVL.status" && status.size() == 4 && status.writable(), "variable metadata");
        check(!plc.variable("GVL.flag").writable(), "read-only metadata");
        rejects([&] { plc.variable("missing"); }, "unknown variable must fail");

        auto value = plc.makeVariable("GVL.status");
        value.set("/count", 0x1234);
        value.set("/enabled", true);
        plc.setVariable("GVL.status", value);
        check(state->data.at(101) == std::vector<std::uint8_t>({0x34, 0x12, 1, 0}), "typed write preserves wire layout");
        check(plc.getVariable("GVL.status") == Json::parse(R"({"count":4660,"enabled":true})"), "typed read decodes custom struct");
        plc.setVariable("GVL.status", Json::parse(R"({"count":7,"enabled":false})"));
        check(plc.getVariable("GVL.status")["count"] == 7, "JSON write/read round trip");

        auto samples = plc.makeVariable("GVL.samples");
        samples.assign(Json::array({65535, 42}));
        plc.setVariable("GVL.samples", samples);
        check(plc.getVariable("GVL.samples") == Json::array({65535, 42}), "array Value writes/readback");
        auto separate_array = plc.makeValue(Json::array({"uint16", 2}));
        separate_array.assign(Json::array({19, 23}));
        plc.setVariable("GVL.samples", separate_array);
        check(plc.getVariable("GVL.samples") == Json::array({19, 23}), "equivalent array factory types are compatible");
        auto custom = plc.makeValue("Status");
        custom.assign(Json::parse(R"({"count":33,"enabled":true})"));
        plc.setVariable("GVL.status", custom);
        check(plc.getVariable("GVL.status")["count"] == 33, "custom type factory uses registered layout");

        std::uint8_t bytes[4] = {0xcd, 0xab, 0, 0};
        plc.write(status, bytes, sizeof(bytes));
        std::memset(bytes, 0, sizeof(bytes));
        plc.read(status, bytes, sizeof(bytes));
        check(bytes[0] == 0xcd && bytes[1] == 0xab, "raw read/write round trip");
        const auto reads = state->reads;
        const auto writes = state->writes;
        rejects([&] { plc.read(status, nullptr, status.size()); }, "null read rejected");
        rejects([&] { plc.write(status, nullptr, status.size()); }, "null write rejected");
        rejects([&] { plc.read(status, bytes, sizeof(bytes) - 1); }, "short read rejected");
        rejects([&] { plc.write(status, bytes, sizeof(bytes) + 1); }, "oversized write rejected");
        rejects([&] { plc.write(plc.variable("GVL.flag"), bytes, 1); }, "disabled writes rejected");
        rejects([&] { plc.setVariable("GVL.flag", Json(true)); }, "disabled JSON write rejected");
        auto wrong = plc.makeValue("uint32");
        rejects([&] { plc.setVariable("GVL.status", wrong); }, "wrong Value type rejected despite equal byte size");
        check(state->reads == reads && state->writes == writes, "invalid operations do not reach transport");
        check(state->acquisitions == 3, "cyclic operations reuse cached handles");
    }
    check(state->subscriptions.empty() && state->data.empty() && state->destroyed, "destruction releases transport resources");
    const std::vector<std::string> expected{
        "acquire:GVL.status", "acquire:GVL.flag", "acquire:GVL.samples", "subscribe:101", "subscribe:102",
        "unsubscribe:202", "unsubscribe:201", "release:103", "release:102", "release:101"};
    check(state->calls == expected, "destruction unsubscribes before releasing handles in reverse order");
}

void observers() {
    auto state = std::make_shared<State>();
    BeckhoffPLC plc(configuration(), transport(state));
    state->data.at(101) = {9, 0, 1, 0};
    state->data.at(102) = {1};
    std::vector<std::pair<std::string, Json>> events;
    const auto json_id = plc.addObserver([&](const std::string& name, const Json& value) { events.emplace_back(name, value); });
    check(events.size() == 2 && events[0].first == "GVL.status" && events[0].second["count"] == 9 && events[1].second == true,
          "observer gets initial monitored values in configuration order");
    std::vector<std::uint8_t> raw;
    const auto raw_id = plc.addRawObserver([&](const std::string& name, const std::uint8_t* data, std::size_t size) {
        if (name == "GVL.status") raw.assign(data, data + size);
    }, false);
    check(raw.empty(), "initial delivery is optional");
    const std::uint8_t update[] = {21, 0, 0, 0};
    state->emit(101, update, sizeof(update));
    check(events.size() == 3 && events.back().second["count"] == 21, "notification JSON decode");
    check(raw == std::vector<std::uint8_t>({21, 0, 0, 0}), "raw notification preserves bytes");

    const auto throwing = plc.addObserver([](const std::string&, const Json&) { throw std::runtime_error("observer failure"); }, false);
    state->emit(101, update, sizeof(update));
    check(plc.callbackErrors() == 1 && events.size() == 4, "observer exceptions are contained");
    state->emit(101, update, sizeof(update) - 1);
    state->emit(101, nullptr, sizeof(update));
    check(plc.callbackErrors() == 3 && events.size() == 4, "invalid notification buffers are contained");
    plc.removeObserver(throwing);
    plc.removeObserver(json_id);
    plc.removeObserver(json_id);
    plc.removeObserver(raw_id);
    state->emit(101, update, sizeof(update));
    check(events.size() == 4 && plc.callbackErrors() == 3, "removed observers receive no future notifications");
    rejects([&] { plc.addObserver({}); }, "empty JSON observer rejected");
    rejects([&] { plc.addRawObserver({}); }, "empty raw observer rejected");

    std::size_t initial_calls = 0;
    state->fail_read = true;
    rejects([&] { plc.addObserver([&](const std::string&, const Json&) { ++initial_calls; }); }, "initial read errors propagate");
    state->fail_read = false;
    state->emit(101, update, sizeof(update));
    check(initial_calls == 0, "failed registration rolls back observer");
    const auto id = plc.addRawObserver([&](const std::string&, const std::uint8_t*, std::size_t) { ++initial_calls; }, false);
    plc.readInitialValues();
    check(initial_calls == 2, "explicit initial reads reach current observers");
    plc.removeObserver(id);

    const auto initial_error_id = plc.addRawObserver([](const std::string&, const std::uint8_t*, std::size_t) { throw 1; });
    check(plc.callbackErrors() == 5, "initial callback exceptions are also contained");
    plc.removeObserver(initial_error_id);
}

void validationBeforeConnect() {
    std::vector<Json> invalid;
    invalid.push_back(Json::array());
    invalid.push_back(configuration()); invalid.back()["PLC"]["port"] = 0;
    invalid.push_back(configuration()); invalid.back()["PLC"]["port"] = 65536;
    invalid.push_back(configuration()); invalid.back()["PLC"]["timeout_ms"] = -1;
    invalid.push_back(configuration()); invalid.back()["PLC"]["variables"] = Json::array();
    invalid.push_back(configuration()); invalid.back()["PLC"]["variables"]["GVL.status"] = "Unknown";
    invalid.push_back(configuration()); invalid.back()["PLC"]["variables_to_monitor"] = Json::array({"missing"});
    invalid.push_back(configuration()); invalid.back()["PLC"]["variables_to_write"] = Json::array({"GVL.status", "GVL.status"});
    invalid.push_back(configuration()); invalid.back()["PLC"]["variables_to_monitor"] = "GVL.status";
    invalid.push_back(configuration()); invalid.back()["PLC"]["pack"] = 3;
    invalid.push_back(configuration()); invalid.back()["PLC"]["structures"]["Recursive"] = Json{{"self", "Recursive"}};
    for (const auto& config : invalid) {
        auto state = std::make_shared<State>();
        rejects([&] { BeckhoffPLC plc(config, transport(state)); }, "invalid configuration accepted");
        check(state->acquisitions == 0 && state->subscription_attempts == 0 && state->destroyed,
              "configuration errors precede ADS acquisition");
    }
}

void constructorRollback() {
    auto acquire_failure = std::make_shared<State>();
    acquire_failure->fail_acquire = 2;
    rejects([&] { BeckhoffPLC plc(configuration(), transport(acquire_failure)); }, "acquire failure must propagate");
    check(acquire_failure->calls == std::vector<std::string>({"acquire:GVL.status", "acquire:GVL.flag", "release:101"}),
          "partial acquisition releases previous handles");
    check(acquire_failure->data.empty() && acquire_failure->destroyed, "failed acquisition destroys transport");

    auto subscription_failure = std::make_shared<State>();
    subscription_failure->fail_subscribe = 2;
    rejects([&] { BeckhoffPLC plc(configuration(), transport(subscription_failure)); }, "subscription failure must propagate");
    check(subscription_failure->calls == std::vector<std::string>({
        "acquire:GVL.status", "acquire:GVL.flag", "acquire:GVL.samples", "subscribe:101", "subscribe:102",
        "unsubscribe:201", "release:103", "release:102", "release:101"}), "partial subscription unwinds notifications and handles");
    check(subscription_failure->subscriptions.empty() && subscription_failure->data.empty() && subscription_failure->destroyed,
          "failed subscription leaves no resources");
}

void clientOwnership() {
    auto first_state = std::make_shared<State>();
    auto second_state = std::make_shared<State>();
    BeckhoffPLC first(configuration(), transport(first_state));
    BeckhoffPLC second(configuration(), transport(second_state));
    std::uint8_t bytes[4]{};
    rejects([&] { second.read(first.variable("GVL.status"), bytes, sizeof(bytes)); }, "cross-client symbol read rejected");
    rejects([&] { second.write(first.variable("GVL.status"), bytes, sizeof(bytes)); }, "cross-client symbol write rejected");
    check(first_state->reads == 0 && second_state->reads == 0 && second_state->writes == 0,
          "cross-client symbols do not reach either transport");
}

void quiescentRemoval() {
    auto state = std::make_shared<State>();
    BeckhoffPLC plc(configuration(), transport(state));
    std::mutex mutex;
    std::condition_variable changed;
    bool entered = false;
    bool finish = false;
    std::atomic<std::size_t> calls{0};
    const auto id = plc.addRawObserver([&](const std::string&, const std::uint8_t*, std::size_t) {
        std::unique_lock<std::mutex> lock(mutex);
        ++calls;
        entered = true;
        changed.notify_all();
        changed.wait(lock, [&] { return finish; });
    }, false);
    const std::uint8_t update[] = {1, 0, 0, 0};
    std::thread delivery([&] { state->emit(101, update, sizeof(update)); });
    {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(lock, [&] { return entered; });
    }
    std::promise<void> removing;
    auto removal_started = removing.get_future();
    auto removal = std::async(std::launch::async, [&] {
        removing.set_value();
        plc.removeObserver(id);
    });
    removal_started.wait();
    const auto premature = removal.wait_for(std::chrono::milliseconds(30)) == std::future_status::ready;
    {
        std::lock_guard<std::mutex> lock(mutex);
        finish = true;
    }
    changed.notify_all();
    delivery.join();
    removal.get();
    check(!premature, "observer removal waits for callbacks already running");
    state->emit(101, update, sizeof(update));
    check(calls == 1, "observer removal prevents later deliveries");
}

void removedSnapshotObserver() {
    auto state = std::make_shared<State>();
    BeckhoffPLC plc(configuration(), transport(state));
    std::mutex mutex;
    std::condition_variable changed;
    bool entered = false;
    bool finish = false;
    const auto first = plc.addRawObserver([&](const std::string&, const std::uint8_t*, std::size_t) {
        std::unique_lock<std::mutex> lock(mutex);
        entered = true;
        changed.notify_all();
        changed.wait(lock, [&] { return finish; });
    }, false);
    std::size_t removed_calls = 0;
    const auto removed = plc.addRawObserver([&](const std::string&, const std::uint8_t*, std::size_t) { ++removed_calls; }, false);
    const std::uint8_t update[] = {1, 0, 0, 0};
    std::thread delivery([&] { state->emit(101, update, sizeof(update)); });
    {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(lock, [&] { return entered; });
    }
    plc.removeObserver(removed);
    {
        std::lock_guard<std::mutex> lock(mutex);
        finish = true;
    }
    changed.notify_all();
    delivery.join();
    check(removed_calls == 0, "a removed observer is skipped by notifications holding an older snapshot");
    plc.removeObserver(first);
}

void callbackReentrancy() {
    auto state = std::make_shared<State>();
    BeckhoffPLC plc(configuration(), transport(state));
    BeckhoffPLC::ObserverId id = 0;
    std::size_t callbacks = 0;
    id = plc.addRawObserver([&](const std::string&, const std::uint8_t*, std::size_t) {
        std::uint8_t bytes[4]{};
        rejects([&] { plc.read(plc.variable("GVL.status"), bytes, sizeof(bytes)); }, "synchronous read inside callback rejected");
        rejects([&] { plc.write(plc.variable("GVL.status"), bytes, sizeof(bytes)); }, "synchronous write inside callback rejected");
        rejects([&] { plc.readInitialValues(); }, "initial read inside callback rejected");
        rejects([&] { plc.removeObserver(id); }, "observer removal inside callback rejected");
        rejects([&] { plc.addRawObserver([](const std::string&, const std::uint8_t*, std::size_t) {}, false); },
                "observer addition inside callback rejected");
        ++callbacks;
    }, false);
    const std::uint8_t update[] = {1, 0, 0, 0};
    state->emit(101, update, sizeof(update));
    check(callbacks == 1 && plc.callbackErrors() == 0, "callback reentrancy fails promptly without escaping dispatcher");
    plc.removeObserver(id);
}
} // namespace

int main() {
    try {
        lifecycleAndValues();
        observers();
        validationBeforeConnect();
        constructorRollback();
        clientOwnership();
        quiescentRemoval();
        removedSnapshotObserver();
        callbackReentrancy();
        std::cout << "BeckhoffPLC tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << "BeckhoffPLC test failure: " << error.what() << '\n';
        return 1;
    }
}
