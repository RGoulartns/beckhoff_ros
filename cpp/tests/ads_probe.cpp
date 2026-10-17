#include "beckhoff/beckhoff_plc.hpp"
#include "test_struct3.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace {
using beckhoff::Json;
constexpr const char* complex_name = "GVL.complex";
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

struct Observations {
    std::mutex mutex;
    std::condition_variable changed;
    std::size_t callbacks = 0;
    Json received;
    std::vector<std::uint8_t> raw;

    void json(const std::string& name, const Json& value) {
        std::lock_guard<std::mutex> lock(mutex);
        check(name == complex_name, "ADS notification symbol name");
        received = value;
        ++callbacks;
        changed.notify_all();
    }
    void bytes(const std::string& name, const std::uint8_t* data, std::size_t size) {
        std::lock_guard<std::mutex> lock(mutex);
        check(name == complex_name, "raw ADS notification symbol name");
        raw.assign(data, data + size);
        changed.notify_all();
    }
    void expect(std::size_t count, const Json& json, const std::vector<std::uint8_t>& bytes) {
        std::unique_lock<std::mutex> lock(mutex);
        check(changed.wait_for(lock, std::chrono::seconds(2), [&] {
            return callbacks == count && received == json && raw == bytes;
        }), "complex ADS notification was not delivered or decoded correctly");
    }
};
} // namespace

int main(int argc, char** argv) {
    try {
        check(argc == 2, "usage: beckhoff_ads_probe /path/to/test_struct3.json");
        // Captured observer state outlives both clients, including error cleanup.
        Observations observations;
        auto owner = beckhoff::BeckhoffPLC::fromFile(argv[1]);
        auto& plc = *owner;
        check(plc.getVariable("GVL.counter") == 42, "initial ADS handle read");
        check(plc.variable(complex_name).size() == 444, "TestStruct3 native PLC layout size");
        const auto zero = plc.makeVariable(complex_name).toJson();
        const auto expected = test_struct3::value();
        const auto golden = test_struct3::goldenBytes();
        check(plc.getVariable(complex_name) == zero, "initial complex ADS JSON read");

        const auto observer = plc.addObserver([&](const std::string& name, const Json& value) {
            observations.json(name, value);
        });
        const auto raw_observer = plc.addRawObserver([&](const std::string& name, const std::uint8_t* data, std::size_t size) {
            observations.bytes(name, data, size);
        });
        observations.expect(1, zero, std::vector<std::uint8_t>(444));
        plc.setVariable(complex_name, expected);
        observations.expect(2, expected, golden);
        check(plc.getVariable(complex_name) == expected, "nested ADS JSON write/read");

        auto buffer = plc.makeVariable(complex_name);
        plc.read(plc.variable(complex_name), buffer.data(), buffer.size());
        check(std::equal(golden.begin(), golden.end(), buffer.data()), "all 444 raw ADS bytes");
        plc.setVariable("GVL.counter", Json(123456));
        check(plc.getVariable("GVL.counter") == 123456, "ADS write/read by cached handle");

        // The clients share one process-wide route but own distinct ADS ports.
        // The peer deliberately reuses notification token 1 for each ADS port.
        auto second_value = expected;
        second_value["attr1"] = false;
        auto second_bytes = golden;
        second_bytes[0] = 0;
        {
            Observations other_observations;
            auto other_owner = beckhoff::BeckhoffPLC::fromFile(argv[1]);
            auto& other = *other_owner;
            check(other.getVariable("GVL.counter") == 123456, "shared-route client read");
            const auto other_observer = other.addObserver([&](const std::string& name, const Json& value) {
                other_observations.json(name, value);
            });
            const auto other_raw = other.addRawObserver([&](const std::string& name, const std::uint8_t* data, std::size_t size) {
                other_observations.bytes(name, data, size);
            });
            other_observations.expect(1, expected, golden);
            plc.write(plc.variable(complex_name), second_bytes.data(), second_bytes.size());
            observations.expect(3, second_value, second_bytes);
            other_observations.expect(2, second_value, second_bytes);
            check(other.getVariable(complex_name) == second_value, "shared-route complex read");
            other.removeObserver(other_observer);
            other.removeObserver(other_raw);
            check(other.callbackErrors() == 0, "second-client ADS observer errors");
        }
        check(plc.getVariable("GVL.counter") == 123456, "shared route survives other client destruction");
        check(plc.getVariable(complex_name) == second_value, "complex ADS raw write/read");
        plc.removeObserver(observer);
        plc.removeObserver(raw_observer);
        check(plc.callbackErrors() == 0, "ADS observer errors");
        std::cout << "Native ADS localhost TestStruct3 integration passed\n";
    } catch (const std::exception& error) {
        std::cerr << "ADS localhost integration failed: " << error.what() << '\n';
        return 1;
    }
}
