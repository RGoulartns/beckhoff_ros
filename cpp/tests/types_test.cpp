#include "beckhoff/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using beckhoff::Json;
using beckhoff::TypeRegistry;
using beckhoff::Value;

namespace {
int checks = 0;

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

template <typename Function> void rejects(Function&& function, const std::string& message) {
    ++checks;
    try { function(); }
    catch (const std::invalid_argument&) { return; }
    throw std::runtime_error("Expected validation error: " + message);
}

void bytesEqual(const Value& value, std::initializer_list<std::uint8_t> bytes, const std::string& message) {
    require(value.size() == bytes.size() && std::equal(bytes.begin(), bytes.end(), value.data()), message);
}

Json sampleStructures() {
    // Deliberately place a referring structure before its definition.
    return Json::parse(R"({
        "CustomStruct2_s": {"attr1":"uint16", "attr2":"CustomStruct1_s", "attr3":["bool",2]},
        "CustomStruct1_s": {"attr1":"uint16", "attr2":"bool"},
        "Nonalphabetic": {"z":"uint8", "a":"uint32"},
        "Wide": {"b":"bool", "d":"double", "tail":"uint8"},
        "Escapes": {"~a/b":"uint16", "":"bool"}
    })");
}

void layoutTests() {
    const TypeRegistry registry(sampleStructures());
    const auto leaf = registry.resolve("CustomStruct1_s");
    require(leaf->size == 4 && leaf->alignment == 2, "ctypes leaf size/alignment");
    require(leaf->fields[0].offset == 0 && leaf->fields[1].offset == 2, "ctypes leaf offsets");
    const auto nested = registry.resolve("CustomStruct2_s");
    require(nested->size == 8 && nested->alignment == 2, "ctypes nested size/alignment");
    require(nested->fields[0].offset == 0 && nested->fields[1].offset == 2 && nested->fields[2].offset == 6, "ctypes nested offsets");
    const auto array = registry.resolve(Json::array({"CustomStruct2_s", 10}));
    require(array->size == 80 && array->alignment == 2, "sample array stride");
    const auto ordered = registry.resolve("Nonalphabetic");
    require(ordered->fields[0].name == "z" && ordered->fields[1].name == "a", "JSON declaration order retained");
    require(ordered->fields[1].offset == 4 && ordered->size == 8, "ordered struct padding");
    const auto matrix = registry.resolve(Json::array({Json::array({"int16", 2}), 3}));
    require(matrix->size == 12 && matrix->element->count == 2, "nested array type");
    for (const auto pack : {0U, 1U, 2U, 4U, 8U, 16U}) {
        const TypeRegistry packed(sampleStructures(), pack);
        const auto wide = packed.resolve("Wide");
        const std::size_t alignment = pack == 0 || pack >= 8 ? 8 : pack;
        const std::size_t expectedSize = alignment == 1 ? 10 : alignment == 2 ? 12 : alignment == 4 ? 16 : 24;
        require(wide->alignment == alignment && wide->size == expectedSize, "pack_mode alignment/size");
        require(wide->fields[1].offset == alignment, "packed double offset");
    }
    const TypeRegistry packed(sampleStructures(), 1);
    require(packed.resolve("CustomStruct1_s")->size == 3 && packed.resolve("CustomStruct2_s")->size == 7, "nested packed structures");
    const TypeRegistry other(Json::parse(R"({"CustomStruct1_s":{"x":"uint8"}})"));
    require(other.resolve("CustomStruct1_s")->size == 1 && leaf->size == 4, "registries do not leak custom types");
    require(registry.resolve("int") == registry.resolve("int16"), "Python int alias");
    require(registry.resolve("float32") == registry.resolve("real"), "float alias");
}

void serializationTests() {
    const TypeRegistry registry(sampleStructures());
    Value value(registry.resolve("CustomStruct2_s"));
    bytesEqual(value, {0,0,0,0,0,0,0,0}, "zero initialization includes padding");
    require(value.toJson() == Json::parse(R"({"attr1":0,"attr2":{"attr1":0,"attr2":false},"attr3":[false,false]})"), "zero defaults");
    value.assign(Json::parse(R"({"attr3":[true,false],"attr2":{"attr2":true,"attr1":4660},"attr1":43981})"));
    bytesEqual(value, {0xcd,0xab,0x34,0x12,1,0,1,0}, "ctypes-compatible little endian nested bytes");
    const auto json = value.toJson();
    require(json.begin().key() == "attr1", "decoded fields retain declaration order");
    Value duplicate(value.type(), json);
    require(std::equal(value.data(), value.data() + value.size(), duplicate.data()), "binary round trip");
    value.data()[5] = 0xff;
    value.assign(json);
    require(value.data()[5] == 0, "assignment clears padding");
    Value matrix(registry.resolve(Json::array({Json::array({"int16",2}), 2})), Json::array({Json::array({-1, -32768}), Json::array({32767,0})}));
    bytesEqual(matrix, {0xff,0xff,0,0x80,0xff,0x7f,0,0}, "multidimensional signed arrays");

    const std::vector<std::pair<std::string, Json>> integers = {
        {"int8", -128}, {"int8", 127}, {"uint8", 255},
        {"int16", -32768}, {"int16", 32767}, {"uint16", 65535},
        {"int32", std::numeric_limits<std::int32_t>::min()}, {"int32", std::numeric_limits<std::int32_t>::max()},
        {"uint32", std::numeric_limits<std::uint32_t>::max()},
        {"int64", std::numeric_limits<std::int64_t>::min()}, {"int64", std::numeric_limits<std::int64_t>::max()},
        {"uint64", std::numeric_limits<std::uint64_t>::max()}
    };
    for (const auto& pair : integers) {
        Value integer(registry.resolve(pair.first), pair.second);
        require(integer.toJson() == pair.second, "integer boundary round trip: " + pair.first);
    }
    Value signed64(registry.resolve("int64"), std::numeric_limits<std::int64_t>::min());
    bytesEqual(signed64, {0,0,0,0,0,0,0,0x80}, "signed int64 minimum bytes");
    Value real(registry.resolve("real"), 1.0);
    bytesEqual(real, {0,0,0x80,0x3f}, "IEEE float32 bytes");
    Value dbl(registry.resolve("double"), -2.5);
    bytesEqual(dbl, {0,0,0,0,0,0,4,0xc0}, "IEEE float64 bytes");
    Value negativeZero(registry.resolve("real"), -0.0);
    require(std::signbit(negativeZero.toJson().get<float>()), "negative zero round trip");
    Value boolean(registry.resolve("bool"), false);
    boolean.data()[0] = 0xff;
    require(boolean.toJson() == true, "nonzero PLC boolean decodes true");
}

void friendlyValueTests() {
    const TypeRegistry registry(sampleStructures());
    Value value(registry.resolve(Json::array({"CustomStruct2_s", 2})));
    value.set("/1/attr2/attr1", 500);
    value.set("/1/attr3/0", true);
    require(value.get("/1/attr2/attr1") == 500 && value.get("/1/attr3/0") == true, "nested structure/array builder");
    require(value.get("/0/attr2/attr1") == 0, "partial builder preserves defaults");
    const auto saved = value.toJson();
    rejects([&] { value.set("/1/attr2", Json::parse(R"({"attr1":3,"attr2":4})")); }, "nested assignment types");
    require(value.toJson() == saved, "failed set preserves original bytes");
    rejects([&] { value.assign(Json::array({saved[0]})); }, "array assignment shape");
    require(value.toJson() == saved, "failed assign preserves original bytes");
    require(value.get("") == saved, "empty pointer selects entire value");
    value.set("", saved);
    Value escaped(registry.resolve("Escapes"));
    escaped.set("/~0a~1b", 65000);
    escaped.set("/", true);
    require(escaped.get("/~0a~1b") == 65000 && escaped.get("/") == true, "RFC6901 escaping and empty field names");
    for (const auto& path : {"1", "/-", "/01", "/2", "/184467440737095516160", "/0/nope", "/0/attr1/x", "/~2"}) {
        rejects([&] { value.get(path); }, std::string("invalid path ") + path);
    }
}

void validationTests() {
    const TypeRegistry registry;
    rejects([] { TypeRegistry bad(Json::array()); }, "structures must be an object");
    rejects([] { TypeRegistry bad(Json::object(), 3); }, "unsupported pack");
    rejects([] { TypeRegistry bad(Json::parse(R"({"bool":{"x":"uint8"}})")); }, "built-in shadow");
    rejects([] { TypeRegistry bad(Json::parse(R"({"Empty":{}})")); }, "empty structure");
    rejects([] { TypeRegistry bad(Json::parse(R"({"Bad":[]})")); }, "nonobject structure");
    rejects([] { TypeRegistry bad(Json::parse(R"({"Bad":{"x":"missing"}})")); }, "unknown field type");
    rejects([] { TypeRegistry bad(Json::parse(R"({"A":{"b":"B"},"B":{"a":["A",2]}})")); }, "recursive structure");
    rejects([&] { registry.resolve("absent"); }, "unknown type");
    for (const auto& spec : {Json::object(), Json::array(), Json::array({"uint8"}), Json::array({"uint8",1,2}),
                            Json::array({"uint8",0}), Json::array({"uint8",-1}), Json::array({"uint8",1.0}),
                            Json::array({"uint8",std::uint64_t(TypeRegistry::maxBytes)+1}),
                            Json::array({"uint64",TypeRegistry::maxBytes})}) {
        rejects([&] { registry.resolve(spec); }, "invalid type spec");
    }
    require(registry.resolve(Json::array({"uint8",TypeRegistry::maxBytes}))->size == TypeRegistry::maxBytes, "maximum allowed bytes");
    rejects([] { TypeRegistry bad(Json::parse(R"({"Huge":{"a":["uint8",16777216],"b":"bool"}})")); }, "structure overflow");
    auto deep = Json("uint8");
    for (int i = 0; i < 140; ++i) deep = Json::array({deep,1});
    rejects([&] { registry.resolve(deep); }, "excessive nested arrays");
    rejects([&] { Value bad(nullptr); }, "null type");
    for (const auto& pair : std::vector<std::pair<std::string, Json>>{
             {"uint8",-1}, {"uint8",256}, {"int8",-129}, {"int8",128}, {"int16",32768},
             {"uint16",65536}, {"int32",2147483648LL}, {"uint32",4294967296LL},
             {"int64",std::numeric_limits<std::uint64_t>::max()}, {"bool",1}, {"int",1.5},
             {"int",true}, {"real","1"}, {"real",1e100}, {"double",std::numeric_limits<double>::infinity()},
             {"real",std::numeric_limits<double>::quiet_NaN()}}) {
        rejects([&] { Value bad(registry.resolve(pair.first), pair.second); }, "invalid scalar value: " + pair.first);
    }
    const TypeRegistry structs(sampleStructures());
    for (const auto& value : {Json::parse(R"({"attr1":1})"), Json::parse(R"({"attr1":1,"unknown":true})"),
                              Json::parse(R"({"attr1":1,"attr2":true,"extra":0})"), Json::array({1,true})}) {
        rejects([&] { Value bad(structs.resolve("CustomStruct1_s"), value); }, "invalid structure fields");
    }
}
} // namespace

int main() {
    try {
        layoutTests();
        serializationTests();
        friendlyValueTests();
        validationTests();
        std::cout << "Type/value tests passed (" << checks << " checks)\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Type/value test failed after " << checks << " checks: " << error.what() << '\n';
        return 1;
    }
}
