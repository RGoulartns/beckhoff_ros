#pragma once

#include "beckhoff/types.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

// Shared by the type, client, wire integration, and benchmark checks. The byte
// oracle below is deliberately independent of TypeRegistry and Value.
namespace test_struct3 {
using beckhoff::Json;

inline Json structures() {
    return Json::parse(R"({
        "TestStruct1": {"attrbt1":"bool", "attrbt2":["bool",6]},
        "TestStruct2": {"attr1":["TestStruct1",10], "attr2":"uint16"},
        "TestStruct3": {"attr1":"bool", "attr2":"uint16", "attr3":["TestStruct2",6], "attr4":"TestStruct1"}
    })");
}

inline std::uint16_t groupMarker(std::size_t group) {
    return static_cast<std::uint16_t>(0x2100 + group * 0x0137);
}

inline Json leafValue(std::size_t group, std::size_t leaf) {
    // Seven bits encode a different pattern for all 60 nested leaves and the
    // final leaf, exposing reversed fields, wrong strides, and skipped indices.
    const auto bits = (group * 10 + leaf) ^ 0x55U;
    Json flags = Json::array();
    for (std::size_t bit = 0; bit < 6; ++bit)
        flags.push_back(((bits >> (bit + 1)) & 1U) != 0);
    return Json{{"attrbt1", (bits & 1U) != 0}, {"attrbt2", std::move(flags)}};
}

inline Json value() {
    Json groups = Json::array();
    for (std::size_t group = 0; group < 6; ++group) {
        Json leaves = Json::array();
        for (std::size_t leaf = 0; leaf < 10; ++leaf)
            leaves.push_back(leafValue(group, leaf));
        groups.push_back(Json{{"attr1", std::move(leaves)}, {"attr2", groupMarker(group)}});
    }
    return Json{{"attr1", true}, {"attr2", 0xa35c},
                {"attr3", std::move(groups)}, {"attr4", leafValue(6, 0)}};
}

inline std::vector<std::uint8_t> goldenBytes(std::size_t pack = 0) {
    if (pack != 0 && pack != 1 && pack != 2 && pack != 4 && pack != 8 && pack != 16)
        throw std::invalid_argument("Unsupported TestStruct3 packing");
    const bool packed = pack == 1;
    std::vector<std::uint8_t> bytes(packed ? 442 : 444, 0);
    const auto writeUint16 = [&](std::size_t offset, std::uint16_t number) {
        bytes.at(offset) = static_cast<std::uint8_t>(number & 0xffU);
        bytes.at(offset + 1) = static_cast<std::uint8_t>(number >> 8);
    };
    const auto writeLeaf = [&](std::size_t offset, std::size_t index) {
        const auto bits = index ^ 0x55U;
        for (std::size_t bit = 0; bit < 7; ++bit)
            bytes.at(offset + bit) = static_cast<std::uint8_t>((bits >> bit) & 1U);
    };
    bytes[0] = 1;
    writeUint16(packed ? 1 : 2, 0xa35c);
    const std::size_t groupsOffset = packed ? 3 : 4;
    for (std::size_t group = 0; group < 6; ++group) {
        const auto offset = groupsOffset + group * 72;
        for (std::size_t leaf = 0; leaf < 10; ++leaf)
            writeLeaf(offset + leaf * 7, group * 10 + leaf);
        writeUint16(offset + 70, static_cast<std::uint16_t>(0x2100 + group * 0x0137));
    }
    writeLeaf(packed ? 435 : 436, 60);
    return bytes;
}

inline Json configuration(std::size_t pack = 0) {
    return Json{{"PLC", Json{{"pack", pack}, {"structures", structures()},
                             {"variables", Json{{"GVL.complex", "TestStruct3"}}},
                             {"variables_to_monitor", Json::array({"GVL.complex"})},
                             {"variables_to_write", Json::array({"GVL.complex"})}}}};
}
} // namespace test_struct3
