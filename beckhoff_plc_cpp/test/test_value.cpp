#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <sstream>
#include <vector>

#include "beckhoff_plc/value.hpp"

using beckhoff::ConstValueRef;
using beckhoff::Json;
using beckhoff::KeyError;
using beckhoff::RangeError;
using beckhoff::TypeError;
using beckhoff::TypeRegistry;
using beckhoff::Value;
using beckhoff::ValueRef;

namespace {

// The structures of config/params.json plus a few others.
TypeRegistry makeTypes() {
  TypeRegistry types;
  types.define({}, {{"CustomStruct1_s", {{"attr1", "uint16"}, {"attr2", "bool"}}, {}},
                    {"CustomStruct2_s",
                     {{"attr1", "uint16"}, {"attr2", "CustomStruct1_s"}, {"attr3", "ARRAY[0..1] OF bool"}},
                     {}},
                    {"ST_All",
                     {{"b", "BOOL"},
                      {"si", "SINT"},
                      {"us", "USINT"},
                      {"i", "INT"},
                      {"ui", "UINT"},
                      {"di", "DINT"},
                      {"udi", "UDINT"},
                      {"li", "LINT"},
                      {"uli", "ULINT"},
                      {"r", "REAL"},
                      {"lr", "LREAL"},
                      {"s", "STRING(5)"},
                      {"ws", "WSTRING(4)"},
                      {"m", "ARRAY[0..1, 0..2] OF INT"}},
                     {}}});
  return types;
}

const TypeRegistry& types() {
  static const TypeRegistry registry = makeTypes();
  return registry;
}

}  // namespace

TEST(Value, IsZeroInitialized) {
  const Value v(types().parse("CustomStruct2_s"));
  EXPECT_EQ(v.size(), 8u);
  for (std::size_t i = 0; i < v.size(); ++i) EXPECT_EQ(v.data()[i], 0);
  EXPECT_EQ(v.toJson(), Json::parse(R"({"attr1":0,"attr2":{"attr1":0,"attr2":false},"attr3":[false,false]})"));
}

TEST(Value, CreateAndEditCustomStructures) {
  Value obj(types().parse("CustomStruct2_s"));
  obj["attr1"] = 42;
  obj["attr2"]["attr1"] = 7;
  obj["attr2"]["attr2"] = true;
  obj["attr3"] = {true, false};
  EXPECT_EQ(obj["attr1"].as<int>(), 42);
  EXPECT_TRUE(obj["attr2"]["attr2"].as<bool>());
  EXPECT_EQ(obj.at("attr3[0]").as<bool>(), true);
  // Little endian bytes at the TwinCAT offsets
  const std::vector<std::uint8_t> expected = {42, 0, 7, 0, 1, 0, 1, 0};
  EXPECT_EQ(std::vector<std::uint8_t>(obj.data(), obj.data() + obj.size()), expected);
}

TEST(Value, ArraysOfStructures) {
  Value list(types().parse("ARRAY[0..9] OF CustomStruct2_s"));
  ASSERT_EQ(list.length(), 10u);
  list[9]["attr2"]["attr1"] = 0xBEEF;
  EXPECT_EQ(list.data()[9 * 8 + 2], 0xEF);
  EXPECT_EQ(list.data()[9 * 8 + 3], 0xBE);
  EXPECT_THROW(list[10], RangeError);
  EXPECT_THROW(list["attr1"], TypeError);
  EXPECT_THROW(list[0]["nope"], KeyError);
  EXPECT_THROW(list[0][0], TypeError);
}

TEST(Value, ElementaryConversions) {
  Value v(types().parse("ST_All"));
  v["r"] = 2;  // integer to REAL
  EXPECT_FLOAT_EQ(v["r"].as<float>(), 2.0f);
  v["i"] = 3.0;  // integral double to INT
  EXPECT_EQ(v["i"].as<int>(), 3);
  EXPECT_THROW(v["i"] = 3.5, RangeError);  // would drop the fraction
  EXPECT_THROW(v["us"] = 256, RangeError);
  EXPECT_THROW(v["us"] = -1, RangeError);
  EXPECT_THROW(v["i"] = 40000, RangeError);
  EXPECT_THROW(v["r"] = 1e300, RangeError);
  v["ui"] = 65535;
  EXPECT_EQ(v["ui"].as<std::uint16_t>(), 65535);
  EXPECT_THROW(v["ui"].as<std::int16_t>(), RangeError);
  EXPECT_EQ(v["ui"].as<double>(), 65535.0);
  v["li"] = std::numeric_limits<std::int64_t>::min();
  EXPECT_EQ(v["li"].as<std::int64_t>(), std::numeric_limits<std::int64_t>::min());
  EXPECT_THROW(v["li"].as<std::uint64_t>(), RangeError);
  v["uli"] = std::numeric_limits<std::uint64_t>::max();
  EXPECT_EQ(v["uli"].as<std::uint64_t>(), std::numeric_limits<std::uint64_t>::max());
  EXPECT_THROW(v["uli"].as<std::int64_t>(), RangeError);
  EXPECT_THROW(v["di"] = 9.3e18, RangeError);
  v["b"] = 5;  // non-zero is true
  EXPECT_EQ(v.data()[0], 1);
  EXPECT_EQ(v["b"].as<int>(), 1);
  v["lr"] = true;
  EXPECT_EQ(v["lr"].as<double>(), 1.0);
  EXPECT_THROW(v["s"] = 1, TypeError);
  EXPECT_THROW(v["s"].as<int>(), TypeError);
  EXPECT_THROW(v["i"] = "1", TypeError);
  EXPECT_THROW(v["i"].as<std::string>(), TypeError);
  EXPECT_THROW(v.as<int>(), TypeError);
}

TEST(Value, Strings) {
  Value v(types().parse("ST_All"));
  v["s"] = "hello";
  EXPECT_EQ(v["s"].as<std::string>(), "hello");
  EXPECT_THROW(v["s"] = "hello!", RangeError);  // STRING(5)
  v["s"] = std::string("hi");
  EXPECT_EQ(v["s"].as<std::string>(), "hi");
  EXPECT_EQ(v.data()[v.type().field("s").offset + 2], 0);  // rest cleared
  // STRING is Latin-1 in TwinCAT: one byte per character, UTF-8 on the C++ side
  v["s"] = "\xC3\xA9t\xC3\xA9";  // "été"
  EXPECT_EQ(v.data()[v.type().field("s").offset], 0xE9);
  EXPECT_EQ(v["s"].as<std::string>(), "\xC3\xA9t\xC3\xA9");
  EXPECT_EQ(v["s"].toJson(), "\xC3\xA9t\xC3\xA9");
  EXPECT_THROW(v["s"] = "\xE2\x82\xAC", RangeError);  // the euro sign is not Latin-1
  EXPECT_THROW(v["s"] = "\xC3", RangeError);          // invalid UTF-8

  v["ws"] = "a\xE2\x82\xAC";  // "a€": UTF-16
  EXPECT_EQ(v["ws"].as<std::string>(), "a\xE2\x82\xAC");
  v["ws"] = "\xF0\x9F\x98\x80";  // U+1F600 uses a surrogate pair (2 units)
  EXPECT_EQ(v["ws"].as<std::string>(), "\xF0\x9F\x98\x80");
  EXPECT_THROW(v["ws"] = "abcde", RangeError);  // WSTRING(4)
}

TEST(Value, ContainersAndInitializerLists) {
  Value v(types().parse("ST_All"));
  v["m"][1] = {4, 5, 6};
  v["m"][0] = std::vector<int>{1, 2, 3};
  EXPECT_EQ(v["m"][1].as<std::vector<int>>(), (std::vector<int>{4, 5, 6}));
  EXPECT_EQ((v["m"][0].as<std::array<short, 3>>()), (std::array<short, 3>{1, 2, 3}));
  EXPECT_THROW(v["m"][0] = std::vector<int>({1, 2}), RangeError);
  EXPECT_THROW(v["m"][0] = std::vector<int>({1, 2, 70000}), RangeError);
  EXPECT_THROW((v["m"][0].as<std::array<int, 2>>()), RangeError);
  std::vector<bool> flags = {true, false};
  Value pair(types().parse("ARRAY[0..1] OF BOOL"));
  pair = flags;
  EXPECT_EQ(pair.as<std::vector<bool>>(), flags);
  EXPECT_EQ(v["m"].toJson(), Json::parse("[[1,2,3],[4,5,6]]"));
}

TEST(Value, PlainCppStructures) {
  struct CustomStruct1 {
    std::uint16_t attr1;
    bool attr2;
  };
  static_assert(sizeof(CustomStruct1) == 4, "same layout as CustomStruct1_s");
  Value v(types().parse("CustomStruct1_s"));
  v.set(CustomStruct1{513, true});
  EXPECT_EQ(v["attr1"].as<int>(), 513);
  EXPECT_TRUE(v["attr2"].as<bool>());
  const CustomStruct1 back = v.as<CustomStruct1>();
  EXPECT_EQ(back.attr1, 513);
  EXPECT_TRUE(back.attr2);

  enum class Mode : std::int16_t { Off = 0, Auto = 2 };
  Value i(types().parse("INT"));
  i.set(Mode::Auto);
  EXPECT_EQ(i.as<int>(), 2);
  EXPECT_EQ(i.as<Mode>(), Mode::Auto);

  struct Wrong {
    std::uint32_t attr1;
    bool attr2;
  };
  EXPECT_THROW(v.as<Wrong>(), TypeError);
  EXPECT_THROW(v.set(Wrong{}), TypeError);
}

TEST(Value, JsonRoundTrip) {
  const Json json = Json::parse(R"({
    "b": true, "si": -5, "us": 200, "i": -300, "ui": 60000, "di": -70000, "udi": 4000000000,
    "li": -9000000000, "uli": 18000000000000000000, "r": 0.5, "lr": 3.25, "s": "abc", "ws": "x",
    "m": [[1, 2, 3], [4, 5, 6]]})");
  const Value v(types().parse("ST_All"), json);
  EXPECT_EQ(v.toJson(), json);
  EXPECT_EQ(Value(v.typePtr(), v.toJson()), v);
  EXPECT_EQ(v.toString(), json.dump());
  std::ostringstream out;
  out << v;
  EXPECT_EQ(out.str(), json.dump());
}

TEST(Value, FromJsonErrorsNameThePath) {
  Value list(types().parse("ARRAY[0..1] OF CustomStruct2_s"));
  const auto message = [&](const Json& json) -> std::string {
    try {
      list.fromJson(json);
    } catch (const beckhoff::Error& e) {
      return e.what();
    }
    return "no error";
  };
  const Json good = Json::parse(R"({"attr1":1,"attr2":{"attr1":2,"attr2":true},"attr3":[true,false]})");
  Json bad = Json::array({good, good});
  bad[1]["attr2"]["attr1"] = 70000;
  EXPECT_EQ(message(bad), "'[1].attr2.attr1': value 70000 does not fit in UINT");
  bad = Json::array({good, good});
  bad[0]["attr2"].erase("attr2");
  EXPECT_EQ(message(bad), "'[0].attr2': missing field 'attr2' of structure 'CustomStruct1_s'");
  bad = Json::array({good, good});
  bad[0]["extra"] = 1;
  EXPECT_EQ(message(bad), "'[0]': structure 'CustomStruct2_s' has no field 'extra'");
  bad = Json::array({good, good});
  bad[0]["attr3"] = Json::array({true});
  EXPECT_EQ(message(bad), "'[0].attr3': ARRAY[0..1] OF BOOL has 2 elements, got 1");
  bad = Json::array({good, good});
  bad[1]["attr1"] = "one";
  EXPECT_EQ(message(bad), "'[1].attr1': expected a number for UINT, got a string");
  EXPECT_EQ(message(good), "expected an array for ARRAY[0..1] OF CustomStruct2_s, got an object");
  // Strong guarantee: the value is unchanged after an error
  EXPECT_EQ(list.toJson()[0]["attr1"], 0);
}

TEST(Value, FromJsonAcceptsTwinCatCaseInsensitiveNames) {
  Value v(types().parse("CustomStruct1_s"));
  v.fromJson({{"ATTR1", 3}, {"Attr2", true}});
  EXPECT_EQ(v["attr1"].as<int>(), 3);
  EXPECT_EQ(v["ATTR2"].as<bool>(), true);
}

TEST(Value, UpdateChangesOnlyTheGivenMembers) {
  Value v(types().parse("CustomStruct2_s"), {{"attr1", 1}, {"attr2", {{"attr1", 2}, {"attr2", true}}}, {"attr3", {true, true}}});
  v.update({{"attr2", {{"attr1", 20}}}, {"attr3", {false}}});
  EXPECT_EQ(v.toJson(), Json::parse(R"({"attr1":1,"attr2":{"attr1":20,"attr2":true},"attr3":[false,true]})"));
  EXPECT_THROW(v.update({{"attr3", {false, false, false}}}), RangeError);
}

TEST(Value, AssignCopiesAndConverts) {
  Value a(types().parse("CustomStruct2_s"));
  Value b(types().parse("CustomStruct2_s"), {{"attr1", 1}, {"attr2", {{"attr1", 2}, {"attr2", true}}}, {"attr3", {true, false}}});
  a["attr2"] = b["attr2"];  // structure copy
  EXPECT_EQ(a["attr2"], b["attr2"]);
  a["attr1"] = b["attr2"]["attr1"];
  EXPECT_EQ(a["attr1"].as<int>(), 2);

  Value all(types().parse("ST_All"));
  all["lr"] = b["attr1"];  // UINT -> LREAL
  EXPECT_EQ(all["lr"].as<double>(), 1.0);
  all["ws"] = "abc";
  all["s"] = all["ws"];  // WSTRING -> STRING
  EXPECT_EQ(all["s"].as<std::string>(), "abc");
  EXPECT_THROW(all["s"] = all["m"], TypeError);
  EXPECT_THROW(a["attr2"] = all["m"][0], TypeError);
  all["i"] = -1;
  EXPECT_THROW(all["ui"] = all["i"], RangeError);
}

TEST(Value, ReferencesWriteThrough) {
  Value v(types().parse("ARRAY[0..2] OF CustomStruct1_s"));
  ValueRef second = v[1];  // bound once, e.g. outside a control loop
  ValueRef attr1 = second["attr1"];
  attr1 = 11;
  EXPECT_EQ(v[1]["attr1"].as<int>(), 11);
  ValueRef first = v[0];
  first = second;  // assigning a reference copies the contents
  EXPECT_EQ(v[0]["attr1"].as<int>(), 11);
  EXPECT_EQ(first.data(), v.data());
  v[2] = v[2];  // self assignment
  first.clear();
  EXPECT_EQ(v[0]["attr1"].as<int>(), 0);
}

TEST(Value, ValuesAreRegularTypes) {
  Value a(types().parse("CustomStruct1_s"), {{"attr1", 5}, {"attr2", true}});
  Value copy = a;
  copy["attr1"] = 6;
  EXPECT_EQ(a["attr1"].as<int>(), 5);  // deep copy
  EXPECT_NE(a, copy);

  Value list(types().parse("ARRAY[0..1] OF CustomStruct1_s"));
  list[1] = a;
  Value element(list[1]);  // copy of the element, with its type
  EXPECT_EQ(element.type().name(), "CustomStruct1_s");
  Value other(types().parse("INT"));
  other = list.ref()[1];  // assigning a reference replaces the value
  EXPECT_EQ(other.type().name(), "CustomStruct1_s");
  other = Value(types().parse("CustomStruct1_s"), {{"attr1", 1}, {"attr2", false}});
  EXPECT_EQ(other["attr1"].as<int>(), 1);

  Value empty;
  EXPECT_FALSE(empty.valid());
  EXPECT_THROW(empty.type(), TypeError);
  EXPECT_THROW(empty = 1, TypeError);
  EXPECT_EQ(Value(ConstValueRef()).valid(), false);
}

TEST(Value, EqualityIgnoresPaddingAndStringTails) {
  Value a(types().parse("CustomStruct1_s"));
  Value b(types().parse("CustomStruct1_s"));
  b.data()[3] = 0xFF;  // padding byte
  EXPECT_EQ(a, b);
  Value s1(types().parse("STRING(5)"));
  Value s2(types().parse("STRING(5)"));
  s1 = "ab";
  s2 = "abcd";
  s2 = "ab";  // leaves no garbage, but the PLC may
  s2.data()[4] = 'x';
  EXPECT_EQ(s1, s2);
  s2 = "ac";
  EXPECT_NE(s1, s2);
  EXPECT_NE(Value(types().parse("INT")), Value(types().parse("UINT")));
}

TEST(Value, Paths) {
  Value list(types().parse("ARRAY[0..1] OF CustomStruct2_s"));
  list.at("[1].attr2.attr1") = 9;
  EXPECT_EQ(list[1]["attr2"]["attr1"].as<int>(), 9);
  EXPECT_EQ(list[1].at("attr3[1]").type().name(), "BOOL");
  EXPECT_THROW(list.at("[2]"), RangeError);
  EXPECT_THROW(list.at("[1].missing"), KeyError);
  EXPECT_THROW(list.at("[1]attr1"), TypeError);
  EXPECT_THROW(list.at(".attr1"), TypeError);
  EXPECT_THROW(list.at("[x]"), TypeError);
  EXPECT_EQ(list.at("").type().name(), list.type().name());
}
