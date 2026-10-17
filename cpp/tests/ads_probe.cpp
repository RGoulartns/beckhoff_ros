#include "beckhoff/beckhoff_plc.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <stdexcept>

namespace {
using beckhoff::Json;
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
Json configuration() {
    return Json::parse(R"({
        "net_id":"127.0.0.1.1.2", "client_net_id":"127.0.0.1.1.1",
        "ip_address":"127.0.0.1", "port":851, "timeout_ms":1000,
        "structures":{"Status":{"count":"uint16","enabled":"bool"}},
        "variables":{"GVL.status":"Status","GVL.counter":"uint32"},
        "variables_to_monitor":["GVL.status"],
        "variables_to_write":["GVL.status","GVL.counter"]
    })");
}
} // namespace

int main() {
    try {
        beckhoff::BeckhoffPLC plc(configuration());
        check(plc.getVariable("GVL.counter") == 42, "initial ADS handle read");
        std::mutex mutex;
        std::condition_variable changed;
        std::size_t callbacks = 0;
        Json received;
        const auto observer = plc.addObserver([&](const std::string& name, const Json& value) {
            std::lock_guard<std::mutex> lock(mutex);
            check(name == "GVL.status", "ADS notification symbol name");
            received = value;
            ++callbacks;
            changed.notify_all();
        });
        check(callbacks == 1 && received["count"] == 7 && received["enabled"] == true,
              "initial ADS observer read");
        plc.setVariable("GVL.status", Json::parse(R"({"count":4660,"enabled":false})"));
        {
            std::unique_lock<std::mutex> lock(mutex);
            check(changed.wait_for(lock, std::chrono::seconds(2), [&] { return callbacks == 2; }),
                  "ADS wire notification was not delivered");
            check(received["count"] == 4660 && received["enabled"] == false, "ADS notification wire decode");
        }
        auto buffer = plc.makeVariable("GVL.status");
        plc.read(plc.variable("GVL.status"), buffer.data(), buffer.size());
        check(buffer.data()[0] == 0x34 && buffer.data()[1] == 0x12 && buffer.data()[2] == 0,
              "raw ADS read bytes");
        plc.setVariable("GVL.counter", Json(123456));
        check(plc.getVariable("GVL.counter") == 123456, "ADS write/read by cached handle");

        // A second client shares the process-wide route and keeps its own ADS port.
        {
            beckhoff::BeckhoffPLC other(configuration());
            check(other.getVariable("GVL.counter") == 123456, "shared-route client read");
        }
        check(plc.getVariable("GVL.counter") == 123456, "shared route survives other client destruction");
        plc.removeObserver(observer);
        check(plc.callbackErrors() == 0, "ADS observer errors");
        std::cout << "ADS localhost integration passed\n";
    } catch (const std::exception& error) {
        std::cerr << "ADS localhost integration failed: " << error.what() << '\n';
        return 1;
    }
}
