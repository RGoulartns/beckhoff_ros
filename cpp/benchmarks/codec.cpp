#include "beckhoff/types.hpp"
#include "../tests/test_struct3.hpp"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

int main() {
    using namespace beckhoff;
    TypeRegistry types(test_struct3::structures());
    const auto type = types.resolve("TestStruct3");
    const auto object = test_struct3::value();
    Value value(type);
    value.assign(object);
    const auto expected = test_struct3::goldenBytes();
    if (value.size() != expected.size() || !std::equal(expected.begin(), expected.end(), value.data()) || value.toJson() != object)
        throw std::runtime_error("TestStruct3 benchmark input failed its golden-byte/round-trip check");
    constexpr std::size_t iterations = 5000;
    for (std::size_t i = 0; i < 100; ++i) {
        value.assign(object);
        if (value.toJson() != object) throw std::runtime_error("benchmark warmup failed");
    }
    std::vector<std::int64_t> durations(iterations);
    std::uint64_t checksum = 0;
    for (std::size_t i = 0; i < iterations; ++i) {
        const auto start = std::chrono::steady_clock::now();
        value.assign(object);
        {
            const auto decoded = value.toJson();
            checksum += decoded.at("attr2").get<unsigned>();
        }
        durations[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
    }
    std::sort(durations.begin(), durations.end());
    std::cout << Json{{"type", "TestStruct3"}, {"bytes", value.size()}, {"iterations", iterations},
        {"median_ns", durations[iterations / 2]}, {"p95_ns", durations[iterations * 95 / 100]},
        {"checksum", checksum}, {"wire_bytes", std::vector<std::uint8_t>(value.data(), value.data() + value.size())}}.dump() << '\n';
}
