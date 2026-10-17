#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <type_traits>
#include <utility>

#include "beckhoff_plc/config.hpp"

using beckhoff::Config;
using beckhoff::ConfigError;
using beckhoff::Json;
using beckhoff::NotificationSettings;

namespace {

// The parameters of config/params.json (Python version).
const char* const kPythonParams = R"({
  "PLC": {
    "net_id": "192.168.1.10.1.1",
    "client_net_id": "192.168.1.20.1.1",
    "username": "Administrator",
    "password": "1",
    "structures": {
      "CustomStruct1_s": {"attr1": "uint16", "attr2": "bool"},
      "CustomStruct2_s": {"attr1": "uint16", "attr2": "CustomStruct1_s", "attr3": ["bool", 2]}
    },
    "variables": {
      "GVL.obj1": ["CustomStruct2_s", 10],
      "GVL.obj2": "CustomStruct1_s",
      "GVL.var1": "bool",
      "GVL.var2": ["uint16", 5]
    },
    "variables_to_monitor": ["GVL.obj2", "GVL.var1"],
    "variables_to_write": ["GVL.var1", "GVL.obj1"]
  }
})";

std::string errorOf(const std::string& json) {
  try {
    Config::fromJsonText(json);
  } catch (const ConfigError& e) {
    return e.what();
  }
  return "no error";
}

/// {"PLC": {"net_id": ..., <extra>}}
std::string params(const std::string& extra) {
  return R"({"PLC": {"net_id": "127.0.0.1.1.1")" + (extra.empty() ? "" : ", " + extra) + "}}";
}

template <class J, class = void>
struct AcceptsJson : std::false_type {};
template <class J>
struct AcceptsJson<J, std::void_t<decltype(Config::fromJson(std::declval<const J&>()))>> : std::true_type {};

}  // namespace

// nlohmann::json sorts object members, which would reorder structure fields.
static_assert(AcceptsJson<beckhoff::Json>::value, "ordered JSON is accepted");
static_assert(!AcceptsJson<nlohmann::json>::value, "unordered JSON is rejected at compile time");

TEST(Config, ReadsThePythonParameters) {
  const Config config = Config::fromJsonText(kPythonParams);
  EXPECT_EQ(config.netId, "192.168.1.10.1.1");
  EXPECT_EQ(config.clientNetId, "192.168.1.20.1.1");
  EXPECT_EQ(config.username, "Administrator");
  EXPECT_EQ(config.password, "1");
  EXPECT_EQ(config.port, 851);
  ASSERT_EQ(config.structures.size(), 2u);
  EXPECT_EQ(config.structures[1].name, "CustomStruct2_s");
  ASSERT_EQ(config.structures[1].fields.size(), 3u);
  EXPECT_EQ(config.structures[1].fields[2].name, "attr3");  // file order, not alphabetical
  EXPECT_EQ(config.structures[1].fields[2].type, "ARRAY[0..1] OF bool");
  ASSERT_EQ(config.variables.size(), 4u);
  EXPECT_EQ(config.variables[0].name, "GVL.obj1");
  EXPECT_EQ(config.variables[0].type, "ARRAY[0..9] OF CustomStruct2_s");
  ASSERT_EQ(config.variablesToMonitor.size(), 2u);
  EXPECT_EQ(config.variablesToMonitor[1].name, "GVL.var1");
  EXPECT_FALSE(config.variablesToMonitor[1].notification.has_value());
  EXPECT_EQ(config.variablesToWrite, (std::vector<std::string>{"GVL.var1", "GVL.obj1"}));
  EXPECT_TRUE(config.ignoredKeys.empty());

  const beckhoff::TypeRegistry types = config.createTypeRegistry();
  EXPECT_EQ(types.parse(config.variables[0].type)->size(), 80u);
  EXPECT_EQ(types.parse(config.variables[3].type)->size(), 10u);
}

TEST(Config, ReadsTheRepositoryParameterFile) {
  std::ifstream file(BECKHOFF_PLC_REPO_PARAMS);
  if (!file) GTEST_SKIP() << "not inside the beckhoff_ros repository";
  const Config config = Config::fromFile(BECKHOFF_PLC_REPO_PARAMS);
  EXPECT_EQ(config.netId, "192.168.1.10.1.1");
  EXPECT_EQ(config.variables.size(), 4u);
}

TEST(Config, AcceptsTheParametersObjectOrADocumentHoldingIt) {
  const Json document = Json::parse(kPythonParams);
  EXPECT_EQ(Config::fromJson(document).netId, "192.168.1.10.1.1");
  EXPECT_EQ(Config::fromJson(document["PLC"]).netId, "192.168.1.10.1.1");
  const Json other = {{"MyPLC", document["PLC"]}};
  EXPECT_EQ(Config::fromJson(other, "MyPLC").variables.size(), 4u);
  EXPECT_THROW(Config::fromJson(other), ConfigError);
}

TEST(Config, MultiDimensionalListTypes) {
  const Config config = Config::fromJsonText(params(R"("variables": {"GVL.m": ["INT", 2, 3], "GVL.n": [["INT", 3], 2]})"));
  EXPECT_EQ(config.variables[0].type, "ARRAY[0..1, 0..2] OF INT");
  EXPECT_EQ(config.variables[1].type, "ARRAY[0..1] OF ARRAY[0..2] OF INT");
  const auto types = config.createTypeRegistry();
  EXPECT_TRUE(types.parse(config.variables[0].type)->sameLayout(*types.parse(config.variables[1].type)));
}

TEST(Config, CppExtensions) {
  const Config config = Config::fromJsonText(R"json({
    // comments are allowed
    "net_id": "10.0.0.5.1.1", "ip_address": "plc.local", "port": 852, "timeout_ms": 200,
    "client_ip_address": "10.0.0.9", "route_name": "robot", "create_route": true, "pack_mode": 4,
    "aliases": {"E_State": "INT", "T_Name": "STRING(20)"},
    "structures": {
      "ST_A": {"state": "E_State", "name": "T_Name"},
      "ST_B": [["x", "LREAL"], "y : ARRAY[0..1] OF BOOL;"]
    },
    "pack_modes": {"ST_B": 1},
    "variables": {"MAIN.a": "ST_A", "MAIN.b": "ST_B"},
    "notification": {"mode": "cyclic", "cycle_time_ms": 10, "max_delay_ms": 0.5},
    "variables_to_monitor": ["MAIN.a", {"name": "MAIN.b", "mode": "on_change"}],
    "verify_sizes": false,
    "comment": "unknown keys are collected"
  })json");
  EXPECT_EQ(config.ipAddress, "plc.local");
  EXPECT_EQ(config.port, 852);
  EXPECT_EQ(config.timeout.count(), 200);
  EXPECT_EQ(config.clientIpAddress, "10.0.0.9");
  EXPECT_EQ(config.routeName, "robot");
  EXPECT_TRUE(config.createRoute);
  EXPECT_FALSE(config.verifySizes);
  EXPECT_EQ(config.packMode, 4);
  ASSERT_EQ(config.aliases.size(), 2u);
  EXPECT_EQ(config.structures[1].fields[1].name, "y");
  EXPECT_EQ(config.structures[1].fields[1].type, "ARRAY[0..1] OF BOOL");
  EXPECT_EQ(config.structures[1].packMode, 1);
  EXPECT_FALSE(config.structures[0].packMode.has_value());
  EXPECT_EQ(config.notification.mode, NotificationSettings::Mode::Cyclic);
  EXPECT_EQ(config.notification.cycleTime.count(), 10'000'000);
  EXPECT_EQ(config.notification.maxDelay.count(), 500'000);
  ASSERT_TRUE(config.variablesToMonitor[1].notification.has_value());
  // per-variable settings start from the defaults
  EXPECT_EQ(config.variablesToMonitor[1].notification->mode, NotificationSettings::Mode::OnChange);
  EXPECT_EQ(config.variablesToMonitor[1].notification->cycleTime.count(), 10'000'000);
  EXPECT_EQ(config.ignoredKeys, std::vector<std::string>{"comment"});

  const auto types = config.createTypeRegistry();
  EXPECT_EQ(types.parse("ST_A")->size(), 24u);  // INT + STRING(20) + padding to the 2 byte alignment
  EXPECT_EQ(types.parse("ST_B")->size(), 10u);  // packed
}

TEST(Config, DefaultNotificationSettingsAreThoseOfPyads) {
  const Config config = Config::fromJsonText(params(""));
  EXPECT_EQ(config.notification.mode, NotificationSettings::Mode::OnChange);
  EXPECT_EQ(config.notification.cycleTime.count(), 100);  // 1 * 100 ns
  EXPECT_EQ(config.notification.maxDelay.count(), 100);
}

TEST(Config, ValidationErrors) {
  EXPECT_EQ(errorOf(R"({"PLC": {}})"), "missing parameter 'net_id'");
  EXPECT_EQ(errorOf(R"({"other": {}})"), "missing 'net_id' (or a 'PLC' object holding it)");
  EXPECT_EQ(errorOf(R"({"PLC": {"net_id": "192.168.1.10"}})"),
            "parameter 'net_id': invalid AMS Net ID '192.168.1.10' (expected e.g. 192.168.1.10.1.1)");
  EXPECT_EQ(errorOf(params(R"("variables_to_write": ["GVL.x"])")),
            "parameter 'variables_to_write': 'GVL.x' is not defined in 'variables'");
  EXPECT_EQ(errorOf(params(R"("variables": {"GVL.x": "INT"}, "variables_to_monitor": ["GVL.x", "GVL.x"])")),
            "parameter 'variables_to_monitor': 'GVL.x' is listed more than once");
  EXPECT_EQ(errorOf(params(R"("variables": {"GVL.x": "ST_Missing"})")),
            "parameter 'variables.GVL.x': unknown type 'ST_Missing'");
  EXPECT_EQ(errorOf(params(R"("variables": {"GVL.x": ["INT", 0]})")),
            "parameter 'variables.GVL.x': must be between 1 and 2147483647");
  EXPECT_EQ(errorOf(params(R"("variables": {"GVL.x": 5})")),
            "parameter 'variables.GVL.x': expected a type name or [type, size], got 5");
  EXPECT_EQ(errorOf(params(R"("variables": ["GVL.x"])")), "parameter 'variables': expected an object, got [\"GVL.x\"]");
  EXPECT_EQ(errorOf(params(R"("structures": {"ST_A": {"x": "ST_B"}})")),
            "structure 'ST_A', field 'x': unknown type 'ST_B'");
  EXPECT_EQ(errorOf(params(R"("pack_modes": {"ST_A": 1})")), "parameter 'pack_modes.ST_A': no structure with that name");
  EXPECT_EQ(errorOf(params(R"("port": 70000)")), "parameter 'port': must be between 1 and 65535");
  EXPECT_EQ(errorOf(params(R"("notification": {"mode": "sometimes"})")),
            "parameter 'notification.mode': expected \"on_change\" or \"cyclic\", got \"sometimes\"");
  EXPECT_EQ(errorOf(params(R"("create_route": "yes")")), "parameter 'create_route': expected true or false, got \"yes\"");
  EXPECT_NE(errorOf("{not json").find("invalid JSON"), std::string::npos);
}

TEST(Config, FileErrors) {
  try {
    Config::fromFile("/nonexistent/params.json");
    FAIL();
  } catch (const ConfigError& e) {
    EXPECT_EQ(std::string(e.what()), "cannot open parameter file '/nonexistent/params.json'");
  }
}

TEST(Config, NetIds) {
  EXPECT_TRUE(beckhoff::isValidNetId("192.168.1.10.1.1"));
  EXPECT_TRUE(beckhoff::isValidNetId("5.80.201.232.1.1"));
  EXPECT_FALSE(beckhoff::isValidNetId("192.168.1.10.1"));
  EXPECT_FALSE(beckhoff::isValidNetId("192.168.1.10.1.1.1"));
  EXPECT_FALSE(beckhoff::isValidNetId("192.168.1.256.1.1"));
  EXPECT_FALSE(beckhoff::isValidNetId("192.168..10.1.1"));
  EXPECT_FALSE(beckhoff::isValidNetId("a.b.c.d.e.f"));
  EXPECT_EQ(beckhoff::netIdToIp("192.168.1.10.1.1"), "192.168.1.10");
  EXPECT_THROW(beckhoff::netIdToIp("192.168.1.10"), ConfigError);
}

TEST(Config, ProgrammaticConfiguration) {
  // e.g. filled from ROS parameters instead of a JSON file
  Config config;
  config.netId = "127.0.0.1.1.1";
  config.structures.push_back({"ST_Cmd", {{"enable", "BOOL"}, {"speed", "LREAL"}}, std::nullopt});
  config.variables = {{"GVL.cmd", "ST_Cmd"}, {"GVL.count", "UDINT"}};
  config.variablesToWrite = {"GVL.cmd"};
  config.variablesToMonitor = {{"GVL.count", std::nullopt}};
  EXPECT_NO_THROW(config.validate());
  EXPECT_EQ(config.createTypeRegistry().parse("ST_Cmd")->size(), 16u);
  config.variablesToWrite.push_back("GVL.nope");
  EXPECT_THROW(config.validate(), ConfigError);
}
