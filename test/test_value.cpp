#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "beckhoff_ros/value.hpp"

using beckhoff_ros::formatPythonFloat;
using beckhoff_ros::formatPythonString;
using beckhoff_ros::parseTypedValue;
using beckhoff_ros::Value;
using beckhoff_ros::ValueError;

// Expected strings are the output of Python's str()/repr()

TEST(Value, FormatsFloatsLikePython)
{
  EXPECT_EQ(formatPythonFloat(0.0), "0.0");
  EXPECT_EQ(formatPythonFloat(-0.0), "-0.0");
  EXPECT_EQ(formatPythonFloat(1.0), "1.0");
  EXPECT_EQ(formatPythonFloat(0.1), "0.1");
  EXPECT_EQ(formatPythonFloat(1e16), "1e+16");
  EXPECT_EQ(formatPythonFloat(1e15), "1000000000000000.0");
  EXPECT_EQ(formatPythonFloat(1e-5), "1e-05");
  EXPECT_EQ(formatPythonFloat(0.0001), "0.0001");
  EXPECT_EQ(formatPythonFloat(123456789.0), "123456789.0");
  EXPECT_EQ(formatPythonFloat(1.5e300), "1.5e+300");
  EXPECT_EQ(formatPythonFloat(5e-324), "5e-324");
  EXPECT_EQ(formatPythonFloat(1e22), "1e+22");
  EXPECT_EQ(formatPythonFloat(2.5e-7), "2.5e-07");
  EXPECT_EQ(formatPythonFloat(12345678901234567.0), "1.2345678901234568e+16");
  EXPECT_EQ(formatPythonFloat(-3.0e-3), "-0.003");
  EXPECT_EQ(formatPythonFloat(std::numeric_limits<double>::infinity()), "inf");
  EXPECT_EQ(formatPythonFloat(-std::numeric_limits<double>::infinity()), "-inf");
  EXPECT_EQ(formatPythonFloat(std::numeric_limits<double>::quiet_NaN()), "nan");
  // a REAL is widened to double, as ctypes does
  EXPECT_EQ(Value(0.1f).str(), "0.10000000149011612");
  EXPECT_EQ(Value(1.401298464324817e-45f).str(), "1.401298464324817e-45");
}

TEST(Value, FormatsLikePython)
{
  EXPECT_EQ(Value().str(), "None");
  EXPECT_EQ(Value(true).str(), "True");
  EXPECT_EQ(Value(false).str(), "False");
  EXPECT_EQ(Value(-42).str(), "-42");
  EXPECT_EQ(Value(std::numeric_limits<std::int64_t>::min()).str(), "-9223372036854775808");
  EXPECT_EQ(Value(std::numeric_limits<std::uint64_t>::max()).str(), "18446744073709551615");
  EXPECT_EQ(Value("text").str(), "text");
  EXPECT_EQ(Value("text").repr(), "'text'");
  EXPECT_EQ(formatPythonString("it's"), "\"it's\"");
  EXPECT_EQ(formatPythonString("q\"x'"), "'q\"x\\''");
  EXPECT_EQ(formatPythonString("a\\b\t\n\x01"), "'a\\\\b\\t\\n\\x01'");

  const Value value = Value::dict(
  {
    {"a", Value::list({1, -2, true, Value(), 1.5})},
    {"it's", Value::dict({{"q\"x", "tab\t"}})},
    {"e", Value::list({})},
    {"f", Value::dict({})},
  });
  EXPECT_EQ(
    value.str(),
    "{'a': [1, -2, True, None, 1.5], \"it's\": {'q\"x': 'tab\\t'}, 'e': [], 'f': {}}");
}

TEST(Value, TypeStringLikePython)
{
  EXPECT_EQ(Value().typeString(), "<class 'NoneType'>");
  EXPECT_EQ(Value(true).typeString(), "<class 'bool'>");
  EXPECT_EQ(Value(1).typeString(), "<class 'int'>");
  EXPECT_EQ(Value(std::numeric_limits<std::uint64_t>::max()).typeString(), "<class 'int'>");
  EXPECT_EQ(Value(1.0).typeString(), "<class 'float'>");
  EXPECT_EQ(Value("x").typeString(), "<class 'str'>");
  EXPECT_EQ(Value::list({}).typeString(), "<class 'list'>");
  EXPECT_EQ(Value::dict({}).typeString(), "<class 'dict'>");
}

TEST(Value, Accessors)
{
  const Value dict = Value::dict({{"attr1", 5}, {"attr2", true}});
  EXPECT_TRUE(dict.isDict());
  EXPECT_EQ(dict["attr1"].asInt(), 5);
  EXPECT_TRUE(dict["attr2"].asBool());
  EXPECT_EQ(dict.find("missing"), nullptr);
  EXPECT_THROW(dict["missing"], ValueError);
  EXPECT_THROW(dict.asList(), ValueError);

  const Value list = Value::list({1, 2.5});
  EXPECT_EQ(list[1].asFloat(), 2.5);
  EXPECT_THROW(list[2], ValueError);

  // items can be changed in place, but not added
  Value robot = Value::dict(
  {
    {"var1", false},
    {"var2", Value::list({Value::dict({{"attr1", 1}}), Value::dict({{"attr1", 2}})})},
  });
  robot["var1"] = true;
  robot["var2"][1]["attr1"] = 42;
  EXPECT_EQ(robot.str(), "{'var1': True, 'var2': [{'attr1': 1}, {'attr1': 42}]}");
  EXPECT_THROW(robot["var3"] = 1, ValueError);
  EXPECT_THROW(robot["var2"][2], ValueError);
  EXPECT_THROW(robot["var1"]["x"], ValueError);
  EXPECT_THROW(robot["var1"][0], ValueError);

  EXPECT_EQ(Value(true).asInt(), 1);  // bool is an int in Python
  EXPECT_EQ(Value(3).asFloat(), 3.0);
  EXPECT_THROW(Value(1.5).asInt(), ValueError);
  EXPECT_THROW(Value(-1).asUInt(), ValueError);
  EXPECT_THROW(Value(std::numeric_limits<std::uint64_t>::max()).asInt(), ValueError);
  EXPECT_EQ(Value(std::numeric_limits<std::uint64_t>::max()).asUInt(), 18446744073709551615u);
  EXPECT_THROW(Value(1).asBool(), ValueError);
}

TEST(Value, ParsesPythonLiterals)
{
  EXPECT_EQ(
    Value::parseLiteral("{'attr1': 5, \"attr2\": True}"),
    Value::dict({{"attr1", 5}, {"attr2", true}}));
  EXPECT_EQ(
    Value::parseLiteral(" [1, -2, 3.5, 1e3, .5, 5., False, None,] "),
    Value::list({1, -2, 3.5, 1000.0, 0.5, 5.0, false, Value()}));
  EXPECT_EQ(Value::parseLiteral("(1, 2)"), Value::list({1, 2}));
  EXPECT_EQ(Value::parseLiteral("(1,)"), Value::list({1}));
  EXPECT_EQ(Value::parseLiteral("(1)"), Value(1));
  EXPECT_EQ(Value::parseLiteral("()"), Value::list({}));
  EXPECT_EQ(Value::parseLiteral("0x1F"), Value(31));
  EXPECT_EQ(Value::parseLiteral("0o17"), Value(15));
  EXPECT_EQ(Value::parseLiteral("0b101"), Value(5));
  EXPECT_EQ(Value::parseLiteral("1_000"), Value(1000));
  EXPECT_EQ(Value::parseLiteral("- 5"), Value(-5));
  EXPECT_EQ(Value::parseLiteral("18446744073709551615"), Value(18446744073709551615u));
  EXPECT_EQ(Value::parseLiteral("'\\x41\\n\\101\\u00e9'"), Value("A\nA\xc3\xa9"));
  EXPECT_EQ(Value::parseLiteral("{'a': 1, 'a': 2}"), Value::dict({{"a", 2}}));
  // JSON is accepted as well
  EXPECT_EQ(
    Value::parseLiteral("{\"a\": [true, false, null]}"),
    Value::dict({{"a", Value::list({true, false, Value()})}}));
  EXPECT_TRUE(std::isinf(Value::parseLiteral("-inf").asFloat()));
  EXPECT_TRUE(std::isnan(Value::parseLiteral("nan").asFloat()));

  for (const char * invalid : {"", "[1, 2", "[,]", "[1,,2]", "{'a' 1}", "{1: 2}", "'abc", "abc",
      "1 2", "--1", "18446744073709551616", "-9223372036854775809", "1e", "0x"})
  {
    EXPECT_THROW(Value::parseLiteral(invalid), ValueError) << invalid;
  }
}

TEST(Value, ParsesServiceValuesLikePython)
{
  // bool(int(value))
  EXPECT_EQ(parseTypedValue("bool", "0"), Value(false));
  EXPECT_EQ(parseTypedValue("bool", "1"), Value(true));
  EXPECT_EQ(parseTypedValue("bool", "-2"), Value(true));
  EXPECT_EQ(parseTypedValue("bool", " 0_0 "), Value(false));
  EXPECT_EQ(parseTypedValue("bool", "99999999999999999999999"), Value(true));
  EXPECT_EQ(parseTypedValue("bool", "True"), Value(true));
  EXPECT_EQ(parseTypedValue("bool", "false"), Value(false));
  EXPECT_THROW(parseTypedValue("bool", "1.0"), ValueError);
  EXPECT_THROW(parseTypedValue("bool", "yes"), ValueError);
  // int(value)
  EXPECT_EQ(parseTypedValue("int", " -42\n"), Value(-42));
  EXPECT_EQ(parseTypedValue("int", "+1_000"), Value(1000));
  EXPECT_EQ(parseTypedValue("int", "007"), Value(7));
  EXPECT_THROW(parseTypedValue("int", "1.5"), ValueError);
  EXPECT_THROW(parseTypedValue("int", "0x10"), ValueError);
  EXPECT_THROW(parseTypedValue("int", ""), ValueError);
  // float(value)
  EXPECT_EQ(parseTypedValue("float", "1.5"), Value(1.5));
  EXPECT_EQ(parseTypedValue("float", " 3 "), Value(3.0));
  EXPECT_EQ(parseTypedValue("float", "1e3"), Value(1000.0));
  EXPECT_EQ(parseTypedValue("float", "-0").str(), "-0.0");
  EXPECT_EQ(parseTypedValue("float", "1e400").str(), "inf");
  EXPECT_EQ(parseTypedValue("float", "-Infinity").str(), "-inf");
  EXPECT_EQ(parseTypedValue("float", "nan").str(), "nan");
  EXPECT_THROW(parseTypedValue("float", "1,5"), ValueError);
  EXPECT_THROW(parseTypedValue("float", "0x10"), ValueError);
  // structures and arrays
  EXPECT_EQ(
    parseTypedValue("dict", "{'attr1': 5, 'attr2': True}"),
    Value::dict({{"attr1", 5}, {"attr2", true}}));
  EXPECT_EQ(parseTypedValue("list", "[1, 2]"), Value::list({1, 2}));
  EXPECT_EQ(parseTypedValue("tuple", "(1, 2)"), Value::list({1, 2}));
  EXPECT_THROW(parseTypedValue("dict", "[1, 2]"), ValueError);
  EXPECT_THROW(parseTypedValue("list", "{}"), ValueError);
  // type names as published in PlcVariable messages
  EXPECT_EQ(parseTypedValue("<class 'int'>", "5"), Value(5));
  EXPECT_EQ(parseTypedValue("<class 'dict'>", "{'a': 1}"), Value::dict({{"a", 1}}));
  // unsupported
  EXPECT_THROW(parseTypedValue("str", "abc"), ValueError);
  EXPECT_THROW(parseTypedValue("eval", "1"), ValueError);
}

TEST(Value, Equality)
{
  EXPECT_EQ(Value(1), Value(1));
  EXPECT_NE(Value(1), Value(1.0));
  EXPECT_NE(Value(1), Value(true));
  EXPECT_NE(Value::dict({{"a", 1}, {"b", 2}}), Value::dict({{"b", 2}, {"a", 1}}));
  EXPECT_NE(
    Value(std::numeric_limits<double>::quiet_NaN()),
    Value(std::numeric_limits<double>::quiet_NaN()));
}
