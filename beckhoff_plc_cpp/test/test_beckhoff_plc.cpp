// End-to-end tests: BeckhoffPLC and the Beckhoff ADS library talking to a
// fake PLC over TCP (see fake_ads_server.hpp).
#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "beckhoff_plc/beckhoff_plc.hpp"
#include "fake_ads_server.hpp"

using beckhoff::AdsError;
using beckhoff::AdsState;
using beckhoff::BeckhoffPLC;
using beckhoff::ConfigError;
using beckhoff::Json;
using beckhoff::KeyError;
using beckhoff::Notification;
using beckhoff::ObserverId;
using beckhoff::RangeError;
using beckhoff::TypeError;
using beckhoff::Value;
using beckhoff::Variable;
using beckhoff::test::FakeAdsServer;
using namespace std::chrono_literals;

namespace {

constexpr std::uint16_t kRead = 2;
constexpr std::uint16_t kWrite = 3;
constexpr std::uint16_t kReadWrite = 9;

// config/params.json of the Python version, pointing to the fake PLC.
Json pythonParams() {
  return Json::parse(R"({
    "PLC": {
      "net_id": "127.0.0.1.1.1",
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
  })");
}

std::vector<std::uint8_t> bytes(std::initializer_list<int> values) {
  std::vector<std::uint8_t> result;
  for (int v : values) result.push_back(static_cast<std::uint8_t>(v));
  return result;
}

bool waitUntil(const std::function<bool()>& condition, std::chrono::milliseconds timeout = 3s) {
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (condition()) return true;
    std::this_thread::sleep_for(1ms);
  }
  return condition();
}

/// Records the notifications received by a callback.
class Recorder {
 public:
  beckhoff::Callback callback() {
    return [this](const Notification& n) {
      std::lock_guard<std::mutex> lock(mutex_);
      events_.emplace_back(n.name, n.value.toJson());
      times_.push_back(n.time());
    };
  }
  std::vector<std::pair<std::string, Json>> events() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }
  std::size_t count(const std::string& name = "") const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t n = 0;
    for (const auto& event : events_) n += name.empty() || event.first == name;
    return n;
  }
  Json last(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = events_.rbegin(); it != events_.rend(); ++it) {
      if (it->first == name) return it->second;
    }
    return nullptr;
  }
  std::chrono::system_clock::time_point lastTime() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return times_.back();
  }

 private:
  mutable std::mutex mutex_;
  std::vector<std::pair<std::string, Json>> events_;
  std::vector<std::chrono::system_clock::time_point> times_;
};

class PlcTest : public ::testing::Test {
 protected:
  void SetUp() override {
    try {
      server_ = std::make_unique<FakeAdsServer>();
    } catch (const std::exception& e) {
      GTEST_SKIP() << "cannot start the fake ADS server: " << e.what();
    }
    params_ = pythonParams();
    server_->addSymbols(beckhoff::Config::fromJson(params_));
  }

  FakeAdsServer& server() { return *server_; }

  /// The PLC sends the current value of a notification right after it is
  /// added; wait for those before counting notifications.
  static void settle() { std::this_thread::sleep_for(30ms); }

  std::unique_ptr<FakeAdsServer> server_;
  Json params_;
};

}  // namespace

TEST_F(PlcTest, ConnectsWithThePythonParameters) {
  BeckhoffPLC plc(params_);
  EXPECT_EQ(server().openHandles(), 4u);  // one per variable: a single round trip per read or write
  EXPECT_EQ(server().activeNotifications(), 2u);
  EXPECT_EQ(plc.variableNames(), (std::vector<std::string>{"GVL.obj1", "GVL.obj2", "GVL.var1", "GVL.var2"}));
  EXPECT_TRUE(plc.variable("GVL.obj1").isWritable());
  EXPECT_FALSE(plc.variable("GVL.obj1").isMonitored());
  EXPECT_TRUE(plc.variable("GVL.var1").isWritable());
  EXPECT_TRUE(plc.variable("GVL.var1").isMonitored());
  EXPECT_FALSE(plc.hasVariable("GVL.nope"));
  EXPECT_EQ(plc.variable("GVL.obj1").type().name(), "ARRAY[0..9] OF CustomStruct2_s");
  EXPECT_EQ(plc.readState(), AdsState::Run);
  EXPECT_EQ(toString(plc.readState()), "Run");
}

TEST_F(PlcTest, FromFileAndConfig) {
  const std::string path = ::testing::TempDir() + "beckhoff_plc_params.json";
  std::ofstream(path) << params_.dump(2);
  BeckhoffPLC fromFile = BeckhoffPLC::fromFile(path);
  EXPECT_EQ(fromFile.variableNames().size(), 4u);

  const beckhoff::Config config = beckhoff::Config::fromJson(params_);
  BeckhoffPLC plc(config);
  EXPECT_EQ(plc.config().netId, "127.0.0.1.1.1");
  BeckhoffPLC moved = std::move(plc);
  EXPECT_FALSE(moved.getVariable<bool>("GVL.var1"));
  EXPECT_THROW(plc.getVariable("GVL.var1"), beckhoff::Error);  // NOLINT(bugprone-use-after-move)
}

TEST_F(PlcTest, ReadsAndWritesElementaryValues) {
  BeckhoffPLC plc(params_);
  plc.setVariable("GVL.var1", true);
  EXPECT_EQ(server().value("GVL.var1"), bytes({1}));
  EXPECT_TRUE(plc.getVariable<bool>("GVL.var1"));
  EXPECT_EQ(plc.getVariable("GVL.var1").toJson(), true);
  plc.setVariable("GVL.var1", 0);
  EXPECT_FALSE(plc.getVariable<bool>("GVL.var1"));
  plc.setVariable("GVL.var1", Json(true));
  EXPECT_EQ(server().value("GVL.var1"), bytes({1}));

  server().setValue("GVL.var2", bytes({1, 0, 2, 0, 3, 0, 4, 0, 0xFF, 0xFF}));
  EXPECT_EQ(plc.getVariable<std::vector<int>>("GVL.var2"), (std::vector<int>{1, 2, 3, 4, 65535}));
  EXPECT_EQ(plc.getVariable<Json>("GVL.var2"), Json::parse("[1,2,3,4,65535]"));
  EXPECT_THROW(plc.getVariable<std::vector<std::int16_t>>("GVL.var2"), RangeError);
}

TEST_F(PlcTest, WritesCustomStructures) {
  BeckhoffPLC plc(params_);
  Value list = plc.variable("GVL.obj1").createValue();
  list[3]["attr1"] = 42;
  list[3]["attr2"]["attr1"] = 7;
  list[3]["attr2"]["attr2"] = true;
  list[3]["attr3"] = {false, true};
  plc.setVariable("GVL.obj1", list);

  const std::vector<std::uint8_t> stored = server().value("GVL.obj1");
  ASSERT_EQ(stored.size(), 80u);
  EXPECT_EQ(std::vector<std::uint8_t>(stored.begin() + 24, stored.begin() + 32), bytes({42, 0, 7, 0, 1, 0, 0, 1}));
  EXPECT_EQ(plc.getVariable("GVL.obj1"), list);

  // An element of another value, a value created by type name
  Value element = plc.createValue("CustomStruct2_s", {{"attr1", 1}, {"attr2", {{"attr1", 2}, {"attr2", false}}},
                                                       {"attr3", {true, true}}});
  list[9] = element;
  plc.variable("GVL.obj1").write(list);
  EXPECT_EQ(plc.getVariable("GVL.obj1")[9], element);
}

TEST_F(PlcTest, WritesPythonStyleValues) {
  BeckhoffPLC plc(params_);
  Json list = Json::array();
  for (int i = 0; i < 10; ++i) {
    list.push_back({{"attr1", i}, {"attr2", {{"attr1", 100 + i}, {"attr2", i % 2 == 1}}}, {"attr3", {true, false}}});
  }
  plc.setVariable("GVL.obj1", list);
  EXPECT_EQ(plc.getVariable("GVL.obj1").toJson(), list);

  // Incomplete values are rejected before anything is sent
  const std::size_t writes = server().requests(kWrite);
  list[9]["attr2"].erase("attr2");
  try {
    plc.setVariable("GVL.obj1", list);
    FAIL();
  } catch (const KeyError& e) {
    EXPECT_STREQ(e.what(), "'GVL.obj1': '[9].attr2': missing field 'attr2' of structure 'CustomStruct1_s'");
  }
  EXPECT_EQ(server().requests(kWrite), writes);
}

TEST_F(PlcTest, ReadsIntoExistingValuesWithoutAllocating) {
  BeckhoffPLC plc(params_);
  const Variable& obj1 = plc.variable("GVL.obj1");
  Value value = obj1.createValue();
  const std::uint8_t* const buffer = value.data();
  server().setValue("GVL.obj1", std::vector<std::uint8_t>(80, 1));
  obj1.read(value);
  EXPECT_EQ(value.data(), buffer);
  EXPECT_EQ(value[9]["attr1"].as<int>(), 257);
  plc.getVariable("GVL.obj1", value);
  EXPECT_EQ(value.data(), buffer);
  obj1.read(value.ref());  // into memory of the same layout
  EXPECT_THROW(obj1.read(value[0]), TypeError);
}

TEST_F(PlcTest, PlainCppStructuresMirroringThePlc) {
  struct CustomStruct1 {
    std::uint16_t attr1;
    bool attr2;
  };
  struct CustomStruct2 {
    std::uint16_t attr1;
    CustomStruct1 attr2;
    bool attr3[2];
  };
  static_assert(sizeof(CustomStruct2) == 8, "same layout as CustomStruct2_s");

  BeckhoffPLC plc(params_);
  std::array<CustomStruct2, 10> list{};
  list[2] = CustomStruct2{5, {6, true}, {true, true}};
  plc.setVariable("GVL.obj1", list);
  EXPECT_EQ(plc.getVariable("GVL.obj1")[2].toJson(),
            Json::parse(R"({"attr1":5,"attr2":{"attr1":6,"attr2":true},"attr3":[true,true]})"));
  EXPECT_EQ((plc.getVariable<std::array<CustomStruct2, 10>>("GVL.obj1")[2].attr2.attr1), 6);

  server().setValue("GVL.obj2", bytes({0x34, 0x12, 1, 0}));
  const auto s = plc.getVariable<CustomStruct1>("GVL.obj2");
  EXPECT_EQ(s.attr1, 0x1234);
  EXPECT_TRUE(s.attr2);
  EXPECT_THROW(plc.getVariable<CustomStruct2>("GVL.obj2"), TypeError);
}

TEST_F(PlcTest, ReportsMisuse) {
  BeckhoffPLC plc(params_);
  EXPECT_THROW(plc.getVariable("GVL.unknown"), KeyError);
  try {
    plc.setVariable("GVL.var2", {1, 2, 3, 4, 5});
    FAIL();
  } catch (const KeyError& e) {
    EXPECT_STREQ(e.what(), "variable 'GVL.var2' is not writable: add it to 'variables_to_write'");
  }
  EXPECT_THROW(plc.setVariable("GVL.obj1", 5), TypeError);
  EXPECT_THROW(plc.setVariable("GVL.var1", "text"), TypeError);
  EXPECT_THROW(plc.setVariable("GVL.obj1", plc.createValue("CustomStruct1_s")), TypeError);
  EXPECT_THROW(plc.subscribe("GVL.obj1", [](const Notification&) {}), KeyError);
  EXPECT_THROW(plc.createValue("ST_Unknown"), KeyError);
  EXPECT_THROW(plc.addObserver(nullptr), beckhoff::Error);
}

TEST_F(PlcTest, NotificationsDeliverNewValues) {
  BeckhoffPLC plc(params_);
  settle();
  Recorder all;
  Recorder var1;
  plc.addObserver(all.callback());  // delivers the current values first
  ASSERT_EQ(all.events().size(), 2u);
  EXPECT_EQ(all.events()[0].first, "GVL.obj2");
  EXPECT_EQ(all.events()[0].second, Json::parse(R"({"attr1":0,"attr2":false})"));
  EXPECT_EQ(all.events()[1].first, "GVL.var1");
  plc.subscribe("GVL.var1", var1.callback(), false);

  server().setValue("GVL.obj2", bytes({5, 0, 1, 0}));  // changed by the PLC program
  ASSERT_TRUE(waitUntil([&] { return all.last("GVL.obj2") == Json::parse(R"({"attr1":5,"attr2":true})"); }));
  plc.setVariable("GVL.var1", true);  // own writes are notified as well
  ASSERT_TRUE(waitUntil([&] { return var1.last("GVL.var1") == true; }));
  EXPECT_TRUE(waitUntil([&] { return all.last("GVL.var1") == true; }));
  EXPECT_EQ(var1.count("GVL.obj2"), 0u);
  // ADS timestamps
  EXPECT_LT(std::chrono::abs(std::chrono::system_clock::now() - var1.lastTime()), 5s);
}

TEST_F(PlcTest, ObserversCanBeRemoved) {
  BeckhoffPLC plc(params_);
  settle();
  Recorder recorder;
  const ObserverId id = plc.addObserver(recorder.callback(), false);
  server().setValue("GVL.var1", bytes({1}));
  ASSERT_TRUE(waitUntil([&] { return recorder.count() == 1; }));
  plc.removeObserver(id);
  server().setValue("GVL.var1", bytes({0}));
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(recorder.count(), 1u);
}

TEST_F(PlcTest, ReadInitialValuesNotifiesEveryObserver) {
  BeckhoffPLC plc(params_);
  settle();
  Recorder a;
  Recorder b;
  Recorder var1;
  plc.addObserver(a.callback(), false);
  plc.addObserver(b.callback(), false);
  plc.subscribe("GVL.var1", var1.callback(), false);
  plc.readInitialValues();
  EXPECT_EQ(a.count(), 2u);
  EXPECT_EQ(b.count(), 2u);
  EXPECT_EQ(var1.count(), 1u);
}

TEST_F(PlcTest, CallbackExceptionsGoToTheErrorHandler) {
  BeckhoffPLC plc(params_);
  settle();
  std::mutex mutex;
  std::vector<std::string> errors;
  plc.setErrorHandler([&](const std::string& error) {
    std::lock_guard<std::mutex> lock(mutex);
    errors.push_back(error);
  });
  plc.addObserver([](const Notification&) { throw std::runtime_error("boom"); }, false);
  server().setValue("GVL.var1", bytes({1}));
  ASSERT_TRUE(waitUntil([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return !errors.empty();
  }));
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(errors[0], "callback for 'GVL.var1' threw an exception: boom");
}

TEST_F(PlcTest, CallbacksMayUseThePlc) {
  BeckhoffPLC plc(params_);
  settle();
  std::atomic<int> handled{0};
  plc.subscribe(
      "GVL.obj2",
      [&](const Notification& n) {
        if (!n.value["attr2"].as<bool>()) return;
        plc.setVariable("GVL.var1", true);  // e.g. acknowledge a request of the PLC
        plc.getVariable("GVL.obj1");
        ++handled;
      },
      false);
  server().setValue("GVL.obj2", bytes({1, 0, 1, 0}));
  ASSERT_TRUE(waitUntil([&] { return handled > 0; }));
  EXPECT_EQ(server().value("GVL.var1"), bytes({1}));
}

TEST_F(PlcTest, SumCommandsUseASingleRequest) {
  BeckhoffPLC plc(params_);
  server().setValue("GVL.var2", bytes({9, 0, 8, 0, 7, 0, 6, 0, 5, 0}));
  const std::size_t readWrites = server().requests(kReadWrite);
  const std::size_t reads = server().requests(kRead);
  const std::size_t writes = server().requests(kWrite);

  const std::vector<Value> values = plc.getVariables({"GVL.var2", "GVL.var1", "GVL.obj2"});
  EXPECT_EQ(server().requests(kReadWrite), readWrites + 1);
  EXPECT_EQ(server().requests(kRead), reads);
  ASSERT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0].as<std::vector<int>>(), (std::vector<int>{9, 8, 7, 6, 5}));
  EXPECT_EQ(values[2].type().name(), "CustomStruct1_s");

  Json obj1 = plc.variable("GVL.obj1").createValue().toJson();
  obj1[4]["attr1"] = 77;
  plc.setVariables({{"GVL.var1", true}, {"GVL.obj1", obj1}});
  EXPECT_EQ(server().requests(kReadWrite), readWrites + 2);
  EXPECT_EQ(server().requests(kWrite), writes);
  EXPECT_EQ(server().value("GVL.var1"), bytes({1}));
  EXPECT_EQ(plc.getVariable("GVL.obj1")[4]["attr1"].as<int>(), 77);

  Value off = plc.createValue("BOOL");
  Value list = plc.variable("GVL.obj1").createValue();
  plc.setVariables({{"GVL.var1", off}, {"GVL.obj1", list}});
  EXPECT_EQ(server().value("GVL.var1"), bytes({0}));
  EXPECT_THROW(plc.setVariables({{"GVL.var2", Json::array({1, 2, 3, 4, 5})}}), KeyError);

  // Reusing the values of a previous read
  std::vector<Value> reused;
  plc.getVariables({"GVL.var1", "GVL.var2"}, reused);
  const std::uint8_t* const buffer = reused[1].data();
  plc.getVariables({"GVL.var1", "GVL.var2"}, reused);
  EXPECT_EQ(reused[1].data(), buffer);
}

TEST_F(PlcTest, FallsBackWithoutSumCommands) {
  server().setSumCommands(false);
  BeckhoffPLC plc(params_);
  server().setValue("GVL.obj2", bytes({3, 0, 1, 0}));
  const auto values = plc.getVariables({"GVL.obj2", "GVL.var1"});
  EXPECT_EQ(values[0]["attr1"].as<int>(), 3);
  plc.setVariables({{"GVL.var1", true}, {"GVL.obj1", plc.variable("GVL.obj1").createValue().toJson()}});
  EXPECT_EQ(server().value("GVL.var1"), bytes({1}));
}

TEST_F(PlcTest, DetectsStructureDefinitionsThatDoNotMatchThePlc) {
  // e.g. CustomStruct1_s declared with {attribute 'pack_mode' := '1'} in the PLC
  server().addSymbol("GVL.obj2", 3, "CustomStruct1_s");
  try {
    BeckhoffPLC plc(params_);
    FAIL();
  } catch (const ConfigError& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("'GVL.obj2' is CustomStruct1_s (4 bytes) in the configuration but CustomStruct1_s (3 bytes) "
                        "in the PLC. Check the members and pack mode of CustomStruct1_s"),
              std::string::npos)
        << what;
  }
  EXPECT_EQ(server().openHandles(), 0u);  // released again
  EXPECT_EQ(server().activeNotifications(), 0u);
}

TEST_F(PlcTest, SizeVerificationCanBeDisabled) {
  server().addSymbol("GVL.var2", 12, "ARRAY [0..5] OF UINT");
  EXPECT_THROW(BeckhoffPLC{params_}, ConfigError);
  params_["PLC"]["verify_sizes"] = false;
  BeckhoffPLC plc(params_);
  EXPECT_EQ(plc.getVariable("GVL.var2").length(), 5u);
}

TEST_F(PlcTest, ReportsMissingSymbols) {
  params_["PLC"]["variables"]["GVL.missing"] = "INT";
  try {
    BeckhoffPLC plc(params_);
    FAIL();
  } catch (const AdsError& e) {
    EXPECT_EQ(e.code(), 0x710);
    EXPECT_NE(std::string(e.what()).find("variable 'GVL.missing' not found in the PLC"), std::string::npos) << e.what();
  }
  EXPECT_EQ(server().openHandles(), 0u);
}

TEST_F(PlcTest, ReportsAnUnknownAdsPort) {
  params_["PLC"]["port"] = 852;
  try {
    BeckhoffPLC plc(params_);
    FAIL();
  } catch (const AdsError& e) {
    EXPECT_EQ(e.code(), 0x6);
  }
}

TEST_F(PlcTest, ReportsAnUnreachablePlc) {
  params_["PLC"]["net_id"] = "127.0.0.2.1.1";  // nothing listens there
  try {
    BeckhoffPLC plc(params_);
    FAIL();
  } catch (const AdsError& e) {
    EXPECT_NE(std::string(e.what()).find("cannot connect to the PLC at 127.0.0.2"), std::string::npos) << e.what();
  }
}

TEST_F(PlcTest, RenewsHandlesAfterAnOnlineChange) {
  BeckhoffPLC plc(params_);
  server().invalidateHandles();
  plc.setVariable("GVL.var1", true);
  EXPECT_TRUE(plc.getVariable<bool>("GVL.var1"));
  server().invalidateHandles();
  EXPECT_TRUE(plc.getVariables({"GVL.var1", "GVL.var2"})[0].as<bool>());
  server().invalidateHandles();
  plc.setVariables({{"GVL.var1", false}, {"GVL.obj1", plc.variable("GVL.obj1").createValue().toJson()}});
  EXPECT_EQ(server().value("GVL.var1"), bytes({0}));
}

TEST_F(PlcTest, ReconnectsAfterAPlcRestart) {
  // The first request sent on a connection the PLC closed only fails when it times out.
  params_["PLC"]["timeout_ms"] = 300;
  BeckhoffPLC plc(params_);
  settle();
  Recorder recorder;
  plc.addObserver(recorder.callback(), false);
  server().restart();
  std::this_thread::sleep_for(50ms);
  EXPECT_THROW(plc.setVariable("GVL.var1", true), AdsError);  // must not kill the process (SIGPIPE)
  EXPECT_THROW(plc.getVariable("GVL.var1"), AdsError);

  plc.reconnect();
  EXPECT_EQ(server().activeNotifications(), 2u);
  EXPECT_EQ(server().openHandles(), 4u);
  server().setValue("GVL.var1", bytes({1}));
  ASSERT_TRUE(waitUntil([&] { return recorder.last("GVL.var1") == true; }));
  EXPECT_TRUE(plc.getVariable<bool>("GVL.var1"));
}

TEST_F(PlcTest, ReleasesEverythingWhenDestroyed) {
  {
    BeckhoffPLC plc(params_);
    plc.addObserver([](const Notification&) {});
    EXPECT_EQ(server().openHandles(), 4u);
  }
  EXPECT_EQ(server().openHandles(), 0u);
  EXPECT_EQ(server().activeNotifications(), 0u);
}

TEST_F(PlcTest, IsThreadSafe) {
  BeckhoffPLC plc(params_);
  std::atomic<int> notifications{0};
  plc.addObserver([&](const Notification&) { ++notifications; }, false);
  std::atomic<int> failures{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([&, t] {
      try {
        Value list = plc.variable("GVL.obj1").createValue();
        for (int i = 0; i < 200; ++i) {
          list[t]["attr1"] = i;
          plc.setVariable("GVL.obj1", list);
          plc.setVariable("GVL.var1", i % 2 == 0);
          plc.getVariable("GVL.var2");
          plc.getVariables({"GVL.obj2", "GVL.var1"});
        }
      } catch (const std::exception& e) {
        ++failures;
        ADD_FAILURE() << e.what();
      }
    });
  }
  for (std::thread& thread : threads) thread.join();
  EXPECT_EQ(failures, 0);
  EXPECT_GT(notifications, 0);
}

TEST_F(PlcTest, CyclicNotifications) {
  Json monitored = Json::array({"GVL.obj2", {{"name", "GVL.var1"}, {"mode", "cyclic"}, {"cycle_time_ms", 5}}});
  params_["PLC"]["variables_to_monitor"] = monitored;
  BeckhoffPLC plc(params_);
  Recorder recorder;
  plc.subscribe("GVL.var1", recorder.callback(), false);
  EXPECT_TRUE(waitUntil([&] { return recorder.count() >= 10; }));  // without any change
}
