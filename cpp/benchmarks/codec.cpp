#include "beckhoff/types.hpp"
#include <chrono>
#include <cstdint>
#include <iostream>
#include <iomanip>

int main() {
    using namespace beckhoff;
    const auto schema = Json::parse(R"({"CustomStruct1_s":{"attr1":"uint16","attr2":"bool"},"CustomStruct2_s":{"attr1":"uint16","attr2":"CustomStruct1_s","attr3":["bool",2]}})");
    TypeRegistry types(schema);
    const auto type = types.resolve("CustomStruct2_s");
    const auto object = Json::parse(R"({"attr1":42,"attr2":{"attr1":100,"attr2":true},"attr3":[false,true]})");
    Value value(type);
    constexpr std::size_t iterations = 200000;
    std::uint64_t checksum = 0;
    auto start = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < iterations; ++i) {
        value.assign(object);
        const auto decoded = value.toJson();
        checksum += decoded.at("attr1").get<unsigned>();
    }
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
    std::cout << "C++ JSON encode+decode: " << ns / static_cast<double>(iterations)
              << " ns/iteration, iterations=" << iterations << ", checksum=" << checksum << '\n';
    std::cout << "Wire bytes: ";
    for (std::size_t i = 0; i < value.size(); ++i)
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(value.data()[i]);
    std::cout << '\n';
}
