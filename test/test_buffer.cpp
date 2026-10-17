#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "beckhoff_ros/buffer.hpp"
#include "beckhoff_ros/plc_types.hpp"

using beckhoff_ros::Buffer;
using beckhoff_ros::ConstView;
using beckhoff_ros::Field;
using beckhoff_ros::PlcTypePtr;
using beckhoff_ros::TypeRegistry;
using beckhoff_ros::Value;
using beckhoff_ros::ValueError;
using beckhoff_ros::View;
using json = nlohmann::ordered_json;

namespace
{

std::string hex(const Buffer & buffer)
{
  static const char * const digits = "0123456789abcdef";
  std::string out;
  for (const auto byte : buffer.bytes()) {
    out += digits[byte >> 4];
    out += digits[byte & 15];
  }
  return out;
}

/// RobotStruct of test_plc_types.cpp (layout checked against ctypes there)
struct Types
{
  TypeRegistry registry;
  PlcTypePtr robot;

  Types()
  {
    registry.addStructure("MyStruct1", json::parse(R"({"attr1": "uint16", "attr2": "bool"})"));
    registry.addStructure("MyStruct21", json::parse(R"({"x": "real", "y": "real"})"));
    registry.addStructure(
      "MyStruct22",
      json::parse(R"({"id": "dint", "points": ["MyStruct21", 3], "flags": ["bool", 4]})"));
    robot = registry.addStructure(
      "RobotStruct",
      json::parse(R"({"var1": "bool", "var2": ["MyStruct1", 5], "var3": "MyStruct22"})"));
  }
};

}  // namespace

TEST(Buffer, IsZeroInitialized)
{
  const Types types;
  const Buffer buffer(types.robot);
  EXPECT_EQ(buffer.size(), 56u);
  EXPECT_EQ(&buffer.type(), types.robot.get());
  EXPECT_EQ(buffer.bytes(), std::vector<std::uint8_t>(56, 0));
  EXPECT_THROW(Buffer(nullptr), ValueError);
}

TEST(Buffer, BuildsTheSameDataAsValues)
{
  const Types types;
  Buffer buffer(types.robot);
  buffer["var1"] = true;
  for (int i = 0; i < 5; ++i) {
    buffer["var2"][i]["attr1"] = 100 + i;
    buffer["var2"][i]["attr2"] = i % 2 == 0;
  }
  buffer["var3"]["id"] = -7;
  for (int i = 0; i < 3; ++i) {
    buffer.at("var3.points[" + std::to_string(i) + "].x") = i * 2.0;
    buffer.at("var3.points[" + std::to_string(i) + "].y") = i * 2.0f + 1.5f;
  }
  buffer["var3"]["flags"][0] = true;
  buffer["var3"]["flags"][2] = 1;  // like ctypes.c_bool(1)
  const std::string expected =
    "010064000100650000006600010067000000680001000000f9ffffff000000000000c03f0000004000006040"
    "000080400000b04001000100";
  EXPECT_EQ(hex(buffer), expected);

  // the same, from a Value
  const Value value = buffer.value();
  const Buffer copy(types.robot, value);
  EXPECT_EQ(copy, buffer);
  EXPECT_EQ(hex(copy), expected);
}

TEST(Buffer, ReadsItems)
{
  const Types types;
  Buffer buffer(types.robot);
  buffer["var2"][3]["attr1"] = 65535;
  buffer["var3"]["id"] = -2147483647 - 1;
  buffer["var3"]["points"][1]["y"] = 2.5;
  const Buffer & data = buffer;
  EXPECT_EQ(data["var2"][3]["attr1"].as<int>(), 65535);
  EXPECT_EQ(data["var2"][3]["attr1"].as<std::uint16_t>(), 65535);
  EXPECT_EQ(data["var2"][3]["attr1"].as<double>(), 65535.0);
  EXPECT_THROW(data["var2"][3]["attr1"].as<std::int16_t>(), ValueError);  // out of range
  EXPECT_EQ(data["var3"]["id"].as<std::int64_t>(), -2147483648LL);
  EXPECT_THROW(data["var3"]["id"].as<unsigned>(), ValueError);  // negative
  EXPECT_EQ(data.at("var3.points[1].y").as<float>(), 2.5f);
  EXPECT_THROW(data.at("var3.points[1].y").as<int>(), ValueError);  // REAL as an integer
  EXPECT_THROW(data.at("var3.points[1].y").as<bool>(), ValueError);
  EXPECT_FALSE(data["var1"].as<bool>());
  EXPECT_EQ(data["var1"].as<int>(), 0);
  EXPECT_THROW(data["var2"].as<int>(), ValueError);  // not an elementary type
  EXPECT_EQ(data["var2"][3].value(), Value::dict({{"attr1", 65535}, {"attr2", false}}));
  EXPECT_EQ(data["var2"].size(), 20u);
  EXPECT_EQ(data["var3"]["points"][1].data() - data.data(), 24 + 4 + 8);
}

TEST(Buffer, ChecksTheItems)
{
  const Types types;
  Buffer buffer(types.robot);
  EXPECT_THROW(buffer["var4"], ValueError);           // no such field
  EXPECT_THROW(buffer["var2"]["attr1"], ValueError);  // an array, not a structure
  EXPECT_THROW(buffer["var2"][5], ValueError);        // out of range
  EXPECT_THROW(buffer[0], ValueError);                // a structure, not an array
  EXPECT_THROW(buffer["var1"]["x"], ValueError);
  for (const char * path : {
    "var2[", "var2[]", "var2[x]", "var2[1", "var2..attr1", ".var1", "var2[1]attr1",
    "var3.", "var2[99]", "var9"})
  {
    EXPECT_THROW(buffer.at(path), ValueError) << path;
  }
  EXPECT_EQ(buffer.at("").data(), buffer.data());
  EXPECT_EQ(buffer.at("var2[4].attr2").data() - buffer.data(), 2 + 4 * 4 + 2);
}

TEST(Buffer, ChecksTheValues)
{
  const Types types;
  Buffer buffer(types.robot);
  buffer["var3"]["id"] = 5;
  const auto before = buffer.bytes();
  EXPECT_THROW(buffer["var2"][0]["attr1"] = 65536, ValueError);
  EXPECT_THROW(buffer["var2"][0]["attr1"] = -1, ValueError);
  EXPECT_THROW(buffer["var2"][0]["attr1"] = 1.5, ValueError);  // no silent truncation
  EXPECT_THROW(buffer["var3"]["points"][0]["x"] = 1e300, ValueError);  // out of REAL range
  EXPECT_THROW(buffer["var3"] = Value::dict({{"id", 1}}), ValueError);  // missing fields
  EXPECT_THROW(buffer["var2"] = Value("text"), ValueError);
  EXPECT_THROW(buffer.setValue(Value::dict({{"var1", true}})), ValueError);
  EXPECT_EQ(buffer.bytes(), before);  // unchanged by the failed assignments

  // a structure assigned from a Value: missing array items are zeroed
  buffer["var2"][4]["attr1"] = 9;
  buffer["var2"] = Value::list({Value::dict({{"attr1", 1}, {"attr2", true}})});
  EXPECT_EQ(buffer["var2"][0]["attr1"].as<int>(), 1);
  EXPECT_EQ(buffer["var2"][4]["attr1"].as<int>(), 0);
}

TEST(Buffer, CopiesItemsOfTheSameType)
{
  const Types types;
  Buffer a(types.robot);
  Buffer b(types.robot);
  a["var2"][1]["attr1"] = 7;
  a["var2"][1]["attr2"] = true;
  b["var2"][3] = a["var2"][1];  // copies the data, does not rebind
  EXPECT_EQ(b["var2"][3].value(), Value::dict({{"attr1", 7}, {"attr2", true}}));
  View item = b["var2"][0];
  item = a["var2"][1];
  EXPECT_EQ(b["var2"][0]["attr1"].as<int>(), 7);
  EXPECT_THROW(b["var2"][0] = a["var3"], ValueError);  // another type
  Buffer c = a;  // copy
  EXPECT_EQ(c, a);
  c.clear();
  EXPECT_NE(c, a);
  EXPECT_EQ(c.bytes(), std::vector<std::uint8_t>(56, 0));
}

TEST(Field, ReadsAndWritesAtAFixedOffset)
{
  const Types types;
  const Field<std::uint16_t> attr1(types.robot, "var2[3].attr1");
  const Field<bool> var1(types.robot, "var1");
  const Field<float> y(types.robot, "var3.points[2].y");
  const Field<std::int32_t> id(*types.robot, "var3.id");
  EXPECT_EQ(attr1.offset(), 2u + 3 * 4);
  EXPECT_EQ(y.offset(), 24u + 4 + 2 * 8 + 4);

  Buffer buffer(types.robot);
  attr1.set(buffer, 4242);
  var1.set(buffer, true);
  y.set(buffer, -1.25f);
  id.set(buffer, -5);
  EXPECT_EQ(buffer["var2"][3]["attr1"].as<int>(), 4242);
  EXPECT_TRUE(buffer["var1"].as<bool>());
  EXPECT_EQ(buffer.at("var3.points[2].y").as<double>(), -1.25);
  EXPECT_EQ(buffer["var3"]["id"].as<int>(), -5);
  EXPECT_EQ(attr1.get(buffer), 4242);
  EXPECT_TRUE(var1.get(buffer));
  EXPECT_EQ(y.get(buffer), -1.25f);
  EXPECT_EQ(id.get(buffer), -5);
  buffer.data()[0] = 2;  // any non zero byte is true
  EXPECT_TRUE(var1.get(buffer));
  // from views of the whole data
  const ConstView view = buffer.view();
  EXPECT_EQ(attr1.get(view), 4242);
}

TEST(Field, ChecksTheTypes)
{
  const Types types;
  EXPECT_THROW(Field<float>(types.robot, "var2[3].attr1"), ValueError);  // UINT, not REAL
  EXPECT_THROW(Field<std::int16_t>(types.robot, "var2[3].attr1"), ValueError);
  EXPECT_THROW(Field<std::uint16_t>(types.robot, "var2[3]"), ValueError);  // a structure
  EXPECT_THROW(Field<std::uint16_t>(types.robot, "var2[5].attr1"), ValueError);
  const Field<std::uint16_t> attr1(types.robot, "var2[3].attr1");

  // a buffer of another type
  Buffer other(types.registry.get("MyStruct22"));
  EXPECT_THROW(attr1.get(other), ValueError);
  EXPECT_THROW(attr1.set(other, 1), ValueError);
  Field<bool> unset;
  Buffer buffer(types.robot);
  EXPECT_THROW(unset.get(buffer), ValueError);

  // the same structure loaded by another registry is accepted
  const Types same;
  const Buffer copy(same.robot);
  EXPECT_EQ(attr1.get(copy), 0);

  // elementary types
  const Field<double> lreal(types.registry.get("lreal"), "");
  Buffer number(types.registry.get("lreal"));
  lreal.set(number, 0.1);
  EXPECT_EQ(number.as<double>(), 0.1);
  EXPECT_EQ(number.value(), Value(0.1));
}
