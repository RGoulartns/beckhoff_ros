#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "beckhoff_ros/plc_types.hpp"

using beckhoff_ros::ConfigError;
using beckhoff_ros::PlcType;
using beckhoff_ros::PlcTypePtr;
using beckhoff_ros::TypeRegistry;
using beckhoff_ros::Value;
using beckhoff_ros::ValueError;
using json = nlohmann::ordered_json;

namespace
{

std::string hex(const std::vector<std::uint8_t> & data)
{
  static const char * const digits = "0123456789abcdef";
  std::string out;
  for (const auto byte : data) {
    out += digits[byte >> 4];
    out += digits[byte & 15];
  }
  return out;
}

std::vector<std::uint8_t> bytes(const std::string & hex_text)
{
  std::vector<std::uint8_t> out;
  for (std::size_t i = 0; i < hex_text.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoi(hex_text.substr(i, 2), nullptr, 16)));
  }
  return out;
}

/// The structures of config/params.json
TypeRegistry exampleRegistry()
{
  TypeRegistry registry;
  registry.addStructure(
    "CustomStruct1_s", json::parse(R"({"attr1": "uint16", "attr2": "bool"})"));
  registry.addStructure(
    "CustomStruct2_s",
    json::parse(R"({"attr1": "uint16", "attr2": "CustomStruct1_s", "attr3": ["bool", 2]})"));
  return registry;
}

std::size_t offset(const PlcTypePtr & type, const std::string & field)
{
  for (const auto & f : type->fields()) {
    if (f.name == field) {
      return f.offset;
    }
  }
  throw std::out_of_range(field);
}

}  // namespace

// Expected layouts and bytes come from ctypes, as used by the Python version

TEST(TypeRegistry, HasThePythonVersionTypes)
{
  const TypeRegistry registry;
  const std::pair<const char *, std::size_t> types[] = {
    {"bool", 1}, {"int", 2}, {"int16", 2}, {"uint16", 2}, {"uint8", 1}, {"real", 4},
    {"lreal", 8}, {"dint", 4}, {"ulint", 8}};
  for (const auto & type : types) {
    ASSERT_NE(registry.find(type.first), nullptr) << type.first;
    EXPECT_EQ(registry.get(type.first)->size(), type.second) << type.first;
  }
  EXPECT_EQ(registry.find("BOOL"), nullptr);  // names are case sensitive, like in Python
  EXPECT_THROW(registry.get("unknown"), ConfigError);
}

TEST(TypeRegistry, ExampleStructuresLayout)
{
  const TypeRegistry registry = exampleRegistry();
  const auto s1 = registry.get("CustomStruct1_s");
  EXPECT_EQ(s1->size(), 4u);
  EXPECT_EQ(s1->alignment(), 2u);
  EXPECT_EQ(offset(s1, "attr2"), 2u);

  const auto s2 = registry.get("CustomStruct2_s");
  EXPECT_EQ(s2->size(), 8u);
  EXPECT_EQ(offset(s2, "attr1"), 0u);
  EXPECT_EQ(offset(s2, "attr2"), 2u);
  EXPECT_EQ(offset(s2, "attr3"), 6u);

  const auto obj1 = registry.resolve(json::parse(R"(["CustomStruct2_s", 10])"));
  EXPECT_EQ(obj1->kind(), PlcType::Kind::Array);
  EXPECT_EQ(obj1->length(), 10u);
  EXPECT_EQ(obj1->size(), 80u);
}

TEST(TypeRegistry, AlignmentAndPacking)
{
  const auto fields = json::parse(
    R"({"a": "bool", "b": "lreal", "c": "uint8", "d": ["int16", 3], "e": "dint"})");
  const struct
  {
    std::size_t pack, size, b, c, d, e;
  } cases[] = {
    {0, 32, 8, 16, 18, 24},  // natural alignment (TwinCAT 3 default)
    {1, 20, 1, 9, 10, 16},  // {attribute 'pack_mode' := '1'}
    {2, 22, 2, 10, 12, 18},
  };
  for (const auto & c : cases) {
    TypeRegistry registry;
    const auto type = registry.addStructure("Mixed", fields, c.pack);
    EXPECT_EQ(type->size(), c.size) << "pack " << c.pack;
    EXPECT_EQ(offset(type, "b"), c.b) << "pack " << c.pack;
    EXPECT_EQ(offset(type, "c"), c.c) << "pack " << c.pack;
    EXPECT_EQ(offset(type, "d"), c.d) << "pack " << c.pack;
    EXPECT_EQ(offset(type, "e"), c.e) << "pack " << c.pack;
  }
}

TEST(PlcType, EncodesAndDecodesLikeCtypes)
{
  const TypeRegistry registry = exampleRegistry();
  const auto s2 = registry.get("CustomStruct2_s");
  const Value value = Value::dict(
  {
    {"attr1", 513},
    {"attr2", Value::dict({{"attr1", 65535}, {"attr2", true}})},
    {"attr3", Value::list({false, true})},
  });
  EXPECT_EQ(hex(s2->encode(value)), "0102ffff01000001");
  EXPECT_EQ(s2->decode(bytes("0102ffff01000001")), value);
  EXPECT_EQ(
    s2->decode(bytes("0102ffff01000001")).str(),
    "{'attr1': 513, 'attr2': {'attr1': 65535, 'attr2': True}, 'attr3': [False, True]}");

  // missing array items are zeroed, like ctypes does
  const auto array = PlcType::array(s2, 2);
  EXPECT_EQ(hex(array->encode(Value::list({value}))), "0102ffff010000010000000000000000");

  const auto real = registry.get("real");
  EXPECT_EQ(hex(real->encode(0.1)), "cdcccc3d");
  EXPECT_EQ(real->decode(bytes("cdcccc3d")).str(), "0.10000000149011612");
  const auto lreal = registry.get("lreal");
  EXPECT_EQ(hex(lreal->encode(-1234.5678)), "adfa5c6d454a93c0");
  EXPECT_EQ(lreal->decode(bytes("adfa5c6d454a93c0")).str(), "-1234.5678");

  const auto int16 = registry.get("int");
  EXPECT_EQ(hex(int16->encode(-2)), "feff");
  EXPECT_EQ(int16->decode(bytes("feff")), Value(-2));
  EXPECT_EQ(hex(registry.get("uint64")->encode(18446744073709551615u)), "ffffffffffffffff");
  EXPECT_EQ(registry.get("int64")->decode(bytes("0000000000000080")).str(), "-9223372036854775808");

  // like ctypes, bool accepts numbers (truth value) and integers accept bool
  const auto boolean = registry.get("bool");
  EXPECT_EQ(hex(boolean->encode(5)), "01");
  EXPECT_EQ(hex(boolean->encode(0.0)), "00");
  EXPECT_EQ(boolean->decode(bytes("02")), Value(true));
  EXPECT_EQ(hex(registry.get("uint16")->encode(true)), "0100");
  EXPECT_EQ(hex(real->encode(3)), "00004040");
}

TEST(PlcType, RejectsValuesThatDoNotFit)
{
  const TypeRegistry registry = exampleRegistry();
  const auto s1 = registry.get("CustomStruct1_s");
  EXPECT_THROW(s1->encode(Value::dict({{"attr1", 1}})), ValueError);  // missing field
  EXPECT_NO_THROW(s1->encode(Value::dict({{"attr1", 1}, {"attr2", true}, {"extra", 1}})));
  EXPECT_THROW(s1->encode(Value::list({1, true})), ValueError);
  EXPECT_THROW(s1->encode(Value::dict({{"attr1", 1.5}, {"attr2", true}})), ValueError);
  EXPECT_THROW(s1->encode(Value::dict({{"attr1", "1"}, {"attr2", true}})), ValueError);

  const auto uint16 = registry.get("uint16");
  EXPECT_THROW(uint16->encode(65536), ValueError);
  EXPECT_THROW(uint16->encode(-1), ValueError);
  EXPECT_THROW(registry.get("int8")->encode(128), ValueError);
  EXPECT_THROW(registry.get("int64")->encode(18446744073709551615u), ValueError);
  EXPECT_THROW(registry.get("real")->encode(1e39), ValueError);
  EXPECT_NO_THROW(registry.get("lreal")->encode(1e39));
  EXPECT_THROW(registry.get("bool")->encode("true"), ValueError);
  EXPECT_THROW(registry.get("bool")->encode(Value()), ValueError);

  const auto array = PlcType::array(uint16, 2);
  EXPECT_THROW(array->encode(Value::list({1, 2, 3})), ValueError);
  EXPECT_THROW(array->encode(1), ValueError);
  try {
    array->encode(Value::list({1, 70000}), "GVL.var2");
    FAIL();
  } catch (const ValueError & e) {
    EXPECT_NE(std::string(e.what()).find("GVL.var2[1]"), std::string::npos) << e.what();
  }
}

TEST(TypeRegistry, RejectsInvalidDefinitions)
{
  TypeRegistry registry;
  // structures must be defined before they are used
  EXPECT_THROW(
    registry.addStructure("A", json::parse(R"({"b": "B"})")), ConfigError);
  EXPECT_THROW(registry.addStructure("A", json::parse(R"(["bool"])")), ConfigError);
  EXPECT_THROW(registry.addStructure("A", json::parse(R"({})")), ConfigError);
  EXPECT_THROW(registry.addStructure("A", json::parse(R"({"a": {"b": "bool"}})")), ConfigError);
  EXPECT_THROW(registry.addStructure("A", json::parse(R"({"a": 5})")), ConfigError);
  EXPECT_THROW(registry.addStructure("bool", json::parse(R"({"a": "bool"})")), ConfigError);
  for (const char * spec : {R"(["bool"])", R"(["bool", 0])", R"(["bool", -1])",
      R"(["bool", 1.5])", R"(["bool", 2, 3])", R"([2, "bool"])", R"(5)", R"({"a": "b"})"})
  {
    EXPECT_THROW(registry.resolve(json::parse(spec)), ConfigError) << spec;
  }
  registry.addStructure("A", json::parse(R"({"a": "bool"})"));
  EXPECT_THROW(registry.addStructure("A", json::parse(R"({"a": "bool"})")), ConfigError);
}
