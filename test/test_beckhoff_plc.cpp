// BeckhoffPLC against a simulated PLC (test/ads_simulator.py, started for each test).

#include <gtest/gtest.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "beckhoff_ros/beckhoff_plc.hpp"

using beckhoff_ros::AdsError;
using beckhoff_ros::BeckhoffPLC;
using beckhoff_ros::Batch;
using beckhoff_ros::Buffer;
using beckhoff_ros::ConfigError;
using beckhoff_ros::ConstView;
using beckhoff_ros::Field;
using beckhoff_ros::Pending;
using beckhoff_ros::Value;
using beckhoff_ros::ValueError;
using beckhoff_ros::Variable;
using Json = nlohmann::ordered_json;
using namespace std::chrono_literals;
namespace ads_error = beckhoff_ros::ads_error;

// Counts the memory allocations (of every thread) while counting is enabled.
#if defined(__GNUC__) && !defined(__clang__)
// operator delete frees what this operator new allocated with malloc
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif
namespace
{
std::atomic<bool> g_count_allocations{false};
std::atomic<std::size_t> g_allocations{0};
}  // namespace

void * operator new(std::size_t size)
{
  if (g_count_allocations.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  void * memory = std::malloc(size > 0 ? size : 1);
  if (memory == nullptr) {
    throw std::bad_alloc();
  }
  return memory;
}

void operator delete(void * memory) noexcept
{
  std::free(memory);
}

void operator delete(void * memory, std::size_t) noexcept
{
  std::free(memory);
}

namespace
{

const char * const kConfig =
  R"({
  "net_id": "127.0.0.1.1.1",
  "structures": {
    "CustomStruct1_s": {"attr1": "uint16", "attr2": "bool"},
    "CustomStruct2_s": {"attr1": "uint16", "attr2": "CustomStruct1_s", "attr3": ["bool", 2]}
  },
  "variables": {
    "GVL.obj1": ["CustomStruct2_s", 10],
    "GVL.obj2": "CustomStruct1_s",
    "GVL.var1": "bool",
    "GVL.var2": ["uint16", 5],
    "GVL.position": "lreal"
  },
  "variables_to_monitor": [
    "GVL.obj2", "GVL.var1",
    {"name": "GVL.position", "mode": "cyclic", "cycle_time_ms": 2}
  ],
  "variables_to_write": ["GVL.var1", "GVL.obj1"]
})";

/// ads_simulator.py in a child process, controlled through its standard input.
class Simulator
{
public:
  explicit Simulator(const Json & config, double cycle_ms = 0)
  {
    char path[] = "/tmp/beckhoff_ros_test_XXXXXX";
    const int fd = ::mkstemp(path);
    if (fd < 0) {
      throw std::runtime_error("mkstemp failed");
    }
    path_ = path;
    const std::string text = Json{{"PLC", config}}.dump();
    if (::write(fd, text.data(), text.size()) != static_cast<ssize_t>(text.size())) {
      throw std::runtime_error("write failed");
    }
    ::close(fd);

    int input[2];
    int output[2];
    if (::pipe(input) != 0 || ::pipe(output) != 0) {
      throw std::runtime_error("pipe failed");
    }
    const std::string cycle = std::to_string(cycle_ms);
    pid_ = ::fork();
    if (pid_ == 0) {
      ::dup2(input[0], 0);
      ::dup2(output[1], 1);
      ::close(input[0]);
      ::close(input[1]);
      ::close(output[0]);
      ::close(output[1]);
      ::execlp(
        "python3", "python3", ADS_SIMULATOR, path, "--host", "127.0.0.1", "--port", "0",
        "--cycle-ms", cycle.c_str(), "--control", static_cast<char *>(nullptr));
      ::_exit(127);
    }
    ::close(input[0]);
    ::close(output[1]);
    to_ = input[1];
    from_ = ::fdopen(output[0], "r");
    const std::string line = readLine();  // "ADS simulator listening on 127.0.0.1:PORT"
    const auto colon = line.rfind(':');
    if (line.find("listening") == std::string::npos || colon == std::string::npos) {
      throw std::runtime_error("the simulator did not start: " + line);
    }
    port_ = static_cast<std::uint16_t>(std::stoi(line.substr(colon + 1)));
  }

  ~Simulator()
  {
    const std::string quit = "quit\n";
    (void)!::write(to_, quit.data(), quit.size());
    ::close(to_);
    std::fclose(from_);
    int status = 0;
    ::waitpid(pid_, &status, 0);
    std::remove(path_.c_str());
  }

  std::uint16_t port() const {return port_;}

  std::string command(const std::string & line)
  {
    const std::string text = line + "\n";
    if (::write(to_, text.data(), text.size()) != static_cast<ssize_t>(text.size())) {
      throw std::runtime_error("the simulator is not running");
    }
    return readLine();
  }

  /// Number of requests received for a command (and index group, e.g. "0xF080").
  int requests(int command, const std::string & group)
  {
    const Json counts = Json::parse(this->command("requests"));
    const std::string key = std::to_string(command) + "," + group;
    return counts.contains(key) ? counts[key].get<int>() : 0;
  }

private:
  std::string readLine()
  {
    char buffer[65536];
    if (std::fgets(buffer, sizeof(buffer), from_) == nullptr) {
      throw std::runtime_error("no answer from the simulator");
    }
    std::string line(buffer);
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
      line.pop_back();
    }
    return line;
  }

  std::string path_;
  pid_t pid_ = -1;
  int to_ = -1;
  FILE * from_ = nullptr;
  std::uint16_t port_ = 0;
};

/// Collects the log messages of a BeckhoffPLC.
struct Log
{
  void operator()(BeckhoffPLC::LogLevel level, const std::string & message)
  {
    std::lock_guard<std::mutex> lock(mutex);
    messages.push_back(std::to_string(static_cast<int>(level)) + " " + message);
  }

  bool contains(const std::string & text)
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & message : messages) {
      if (message.find(text) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  std::mutex mutex;
  std::vector<std::string> messages;
};

template<typename Predicate>
bool waitUntil(Predicate predicate, std::chrono::milliseconds timeout = 5000ms)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

class BeckhoffPlcTest : public ::testing::Test
{
protected:
  Json config(const Simulator & simulator)
  {
    Json params = Json::parse(kConfig);
    params["tcp_port"] = simulator.port();
    params["reconnect_interval_ms"] = 100;
    params["health_check_ms"] = 100;
    return params;
  }

  std::unique_ptr<BeckhoffPLC> connect(Simulator & simulator)
  {
    return std::make_unique<BeckhoffPLC>(
      config(simulator), [this](BeckhoffPLC::LogLevel level, const std::string & message) {
        log(level, message);
      });
  }

  /// Waits for the first notifications (the PLC sends the current values when the
  /// notifications are added).
  static void settle() {std::this_thread::sleep_for(50ms);}

  Log log;
};

}  // namespace

TEST_F(BeckhoffPlcTest, ReadsAndWritesValues)
{
  Simulator simulator(Json::parse(kConfig));
  simulator.command("set GVL.var2 [1, 2, 3, 4, 5]");
  auto plc = connect(simulator);
  EXPECT_TRUE(plc->ready());
  EXPECT_EQ(plc->localNetId(), "127.0.0.1.1.1");
  EXPECT_EQ(plc->getVariable("GVL.var2"), Value::list({1, 2, 3, 4, 5}));

  plc->setVariable("GVL.obj2", Value::dict({{"attr1", 7}, {"attr2", true}}));
  EXPECT_EQ(simulator.command("get GVL.obj2"), "{'attr1': 7, 'attr2': True}");
  EXPECT_EQ(plc->getVariable("GVL.obj2"), Value::dict({{"attr1", 7}, {"attr2", true}}));
  EXPECT_THROW(
    plc->setVariable("GVL.obj2", Value::dict({{"attr1", 70000}, {"attr2", true}})),
    ValueError);
  EXPECT_THROW(plc->getVariable("GVL.unknown"), ConfigError);

  // the handles and the sizes of every variable were obtained with one request
  EXPECT_EQ(simulator.requests(9, "0xF082"), 1);
  EXPECT_EQ(simulator.requests(9, "0xF003"), 0);  // no handle request per read
}

TEST_F(BeckhoffPlcTest, ReadsAndWritesBuffers)
{
  Simulator simulator(Json::parse(kConfig));
  auto plc = connect(simulator);
  const Variable & obj1 = plc->variable("GVL.obj1");
  EXPECT_EQ(obj1.size(), 80u);
  EXPECT_TRUE(obj1.writable());
  EXPECT_FALSE(obj1.monitored());
  Buffer data = obj1.buffer();
  data[3]["attr2"]["attr1"] = 42;
  data[3]["attr3"][1] = true;
  const Field<std::uint16_t> attr1(obj1.type(), "[9].attr1");
  attr1.set(data, 999);
  plc->write(obj1, data);
  const Value value = Value::parseLiteral(simulator.command("get GVL.obj1"));
  EXPECT_EQ(value[3]["attr2"]["attr1"], Value(42));
  EXPECT_EQ(value[3]["attr3"], Value::list({false, true}));
  EXPECT_EQ(value[9]["attr1"], Value(999));

  Buffer read = obj1.buffer();
  plc->read(obj1, read);
  EXPECT_EQ(read, data);
  EXPECT_EQ(attr1.get(read), 999);

  // buffers of another type are rejected
  Buffer wrong = plc->buffer("GVL.obj2");
  EXPECT_THROW(plc->read(obj1, wrong), ValueError);
  EXPECT_THROW(plc->write(obj1, wrong), ValueError);
}

TEST_F(BeckhoffPlcTest, ExecutesBatchesInOneRoundTrip)
{
  Simulator simulator(Json::parse(kConfig));
  simulator.command("set GVL.var2 [5, 4, 3, 2, 1]");
  auto plc = connect(simulator);
  const Variable & var1 = plc->variable("GVL.var1");
  const Variable & var2 = plc->variable("GVL.var2");
  const Variable & obj1 = plc->variable("GVL.obj1");
  Buffer var1_in = var1.buffer();
  Buffer var1_out = var1.buffer();
  Buffer var2_out = var2.buffer();
  Buffer obj1_in = obj1.buffer();
  Buffer obj1_out = obj1.buffer();
  Batch batch;
  batch.write(var1, var1_in).write(obj1, obj1_in);
  batch.read(var1, var1_out).read(var2, var2_out).read(obj1, obj1_out);
  const int sum_reads = simulator.requests(9, "0xF080");
  const int sum_writes = simulator.requests(9, "0xF081");
  for (int i = 0; i < 3; ++i) {
    var1_in.view() = i % 2 == 0;
    obj1_in[0]["attr1"] = i;
    plc->execute(batch);
    // the reads see the writes of the same batch
    EXPECT_EQ(var1_out.as<bool>(), i % 2 == 0);
    EXPECT_EQ(obj1_out[0]["attr1"].as<int>(), i);
    EXPECT_EQ(var2_out.value(), Value::list({5, 4, 3, 2, 1}));
  }
  EXPECT_EQ(simulator.requests(9, "0xF080"), sum_reads + 3);
  EXPECT_EQ(simulator.requests(9, "0xF081"), sum_writes + 3);

  // a batch with a single read: a plain read request
  Batch single;
  single.read(var2, var2_out);
  const int reads = simulator.requests(2, "0xF005");
  plc->execute(single);
  EXPECT_EQ(simulator.requests(2, "0xF005"), reads + 1);

  // the batch is not modified while it is executed
  Pending pending = plc->executeAsync(batch);
  EXPECT_THROW(batch.read(var2, var2_out), ConfigError);
  EXPECT_THROW(plc->execute(batch), ConfigError);
  pending.wait();
  EXPECT_NO_THROW(batch.read(var2, var2_out));
}

TEST_F(BeckhoffPlcTest, ReportsTheFailedItemsOfABatch)
{
  Simulator simulator(Json::parse(kConfig));
  Json params = config(simulator);
  params["health_check_ms"] = 0;  // the error itself makes the handles be created again
  auto plc = std::make_unique<BeckhoffPLC>(params);
  const Variable & var1 = plc->variable("GVL.var1");
  const Variable & var2 = plc->variable("GVL.var2");
  Buffer var1_out = var1.buffer();
  Buffer var2_out = var2.buffer();
  Batch batch;
  batch.read(var1, var1_out).read(var2, var2_out);
  // the PLC loses the handles (a new program): the error names the variable
  simulator.command("set GVL.var1 True");
  simulator.command("program");
  try {
    plc->execute(batch);
    FAIL() << "no error";
  } catch (const AdsError & e) {
    EXPECT_EQ(e.code(), static_cast<long>(ads_error::kSymbolNotFound));
    EXPECT_NE(std::string(e.what()).find("GVL.var1"), std::string::npos) << e.what();
  }
  // and the handles are created again
  EXPECT_TRUE(waitUntil([&plc]() {return plc->ready();}));
  plc->execute(batch);
  EXPECT_TRUE(var1_out.as<bool>());
}

TEST_F(BeckhoffPlcTest, ChecksTheConfigurationAgainstThePlc)
{
  // the PLC declares attr1 as UDINT
  Json simulated = Json::parse(kConfig);
  simulated["structures"]["CustomStruct1_s"]["attr1"] = "uint32";
  {
    Simulator simulator(simulated);
    try {
      connect(simulator);
      FAIL() << "no error";
    } catch (const ConfigError & e) {
      const std::string message = e.what();
      EXPECT_NE(
        message.find(
          "GVL.obj2: 4 bytes in the configuration ('CustomStruct1_s'), "
          "8 bytes on the PLC ('CustomStruct1_s')"), std::string::npos) << message;
      EXPECT_NE(message.find("GVL.obj1: 80 bytes"), std::string::npos) << message;
    }
  }
  // a variable that does not exist
  simulated = Json::parse(kConfig);
  simulated["variables"].erase("GVL.var2");
  {
    Simulator simulator(simulated);
    try {
      connect(simulator);
      FAIL() << "no error";
    } catch (const AdsError & e) {
      EXPECT_EQ(e.code(), static_cast<long>(ads_error::kSymbolNotFound));
      EXPECT_NE(std::string(e.what()).find("GVL.var2 (symbol not found)"), std::string::npos)
        << e.what();
    }
    // the handles obtained were released
    EXPECT_EQ(simulator.requests(9, "0xF081"), 1);
  }
}

TEST_F(BeckhoffPlcTest, WorksWithoutSumCommands)
{
  Simulator simulator(Json::parse(kConfig));
  simulator.command("sum off");
  auto plc = connect(simulator);
  EXPECT_TRUE(log.contains("does not support ADS sum commands"));
  const Variable & var1 = plc->variable("GVL.var1");
  const Variable & obj2 = plc->variable("GVL.obj2");
  Buffer var1_in = var1.buffer();
  var1_in.view() = true;
  Buffer obj2_out = obj2.buffer();
  Buffer var1_out = var1.buffer();
  Batch batch;
  batch.write(var1, var1_in).read(var1, var1_out).read(obj2, obj2_out);
  plc->execute(batch);
  EXPECT_TRUE(var1_out.as<bool>());
  EXPECT_EQ(simulator.command("get GVL.var1"), "True");
}

TEST_F(BeckhoffPlcTest, KeepsTheLatestValuesOfTheMonitoredVariables)
{
  Simulator simulator(Json::parse(kConfig));
  simulator.command("set GVL.obj2 {'attr1': 3, 'attr2': True}");
  auto plc = connect(simulator);
  const Variable & obj2 = plc->variable("GVL.obj2");
  Buffer data = obj2.buffer();
  beckhoff_ros::Timestamp time;
  ASSERT_TRUE(plc->latest(obj2, data, &time));  // available right away
  EXPECT_EQ(data.value(), Value::dict({{"attr1", 3}, {"attr2", true}}));
  EXPECT_LT(std::chrono::system_clock::now() - time, 10s);
  EXPECT_THROW(plc->latest(plc->variable("GVL.var2"), data), ConfigError);  // not monitored

  std::uint64_t sequence = 0;
  ASSERT_TRUE(plc->waitForUpdate(obj2, data, sequence, 1s));
  EXPECT_EQ(sequence, 1u);
  settle();
  // the first notification carried the value read: not a new value
  EXPECT_FALSE(plc->waitForUpdate(obj2, data, sequence, 100ms));
  simulator.command("set GVL.obj2 {'attr1': 4, 'attr2': False}");
  ASSERT_TRUE(plc->waitForUpdate(obj2, data, sequence, 2s));
  EXPECT_EQ(data.value(), Value::dict({{"attr1", 4}, {"attr2", false}}));
  ASSERT_TRUE(plc->latest(obj2, data));
  EXPECT_EQ(data["attr1"].as<int>(), 4);

  // cyclic notifications: a value every 2 ms
  const Variable & position = plc->variable("GVL.position");
  Buffer value = position.buffer();
  std::uint64_t position_sequence = 0;
  ASSERT_TRUE(plc->waitForUpdate(position, value, position_sequence, 1s));
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(plc->waitForUpdate(position, value, position_sequence, 1s));
  }
  EXPECT_GE(std::chrono::steady_clock::now() - start, 10ms);
}

TEST_F(BeckhoffPlcTest, CallsTheUpdateCallbacks)
{
  Simulator simulator(Json::parse(kConfig));
  auto plc = connect(simulator);
  settle();
  const Variable & obj2 = plc->variable("GVL.obj2");
  const Field<std::uint16_t> attr1(obj2.type(), "attr1");
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<int> values;
  const auto id = plc->onUpdate(
    obj2, [&](const Variable & variable, const ConstView & data, beckhoff_ros::Timestamp) {
      EXPECT_EQ(variable.name(), "GVL.obj2");
      std::lock_guard<std::mutex> lock(mutex);
      values.push_back(attr1.get(data));
      cv.notify_all();
    });
  for (int i = 1; i <= 3; ++i) {
    simulator.command("set GVL.obj2 {'attr1': " + std::to_string(i) + ", 'attr2': True}");
  }
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(cv.wait_for(lock, 2s, [&values]() {return values.size() >= 3;}));
    EXPECT_EQ(values, (std::vector<int>{1, 2, 3}));
  }
  plc->removeCallback(id);
  simulator.command("set GVL.obj2 {'attr1': 9, 'attr2': True}");
  Buffer data = obj2.buffer();
  ASSERT_TRUE(waitUntil([&]() {return plc->latest(obj2, data) && attr1.get(data) == 9;}));
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(values.size(), 3u);
}

TEST_F(BeckhoffPlcTest, NotifiesTheObservers)
{
  Simulator simulator(Json::parse(kConfig));
  simulator.command("set GVL.var1 True");
  auto plc = connect(simulator);
  std::mutex mutex;
  std::vector<std::string> first;
  std::vector<std::string> second;
  const auto observer = [&mutex](std::vector<std::string> & messages) {
    return [&mutex, &messages](const std::string & name, const Value & value) {
             if (name != "GVL.position") {
               std::lock_guard<std::mutex> lock(mutex);
               messages.push_back(name + " " + value.str());
             }
           };
  };
  const auto get = [&mutex](const std::vector<std::string> & messages) {
    std::lock_guard<std::mutex> lock(mutex);
    return messages;
  };
  const std::vector<std::string> initial{
    "GVL.obj2 {'attr1': 0, 'attr2': False}", "GVL.var1 True"};
  plc->addObserver(observer(first));
  // the current values first
  ASSERT_TRUE(waitUntil([&]() {return get(first).size() == 2;}));
  EXPECT_EQ(get(first), initial);
  simulator.command("set GVL.var1 False");
  simulator.command("set GVL.var1 True");
  ASSERT_TRUE(waitUntil([&]() {return get(first).size() == 4;}));
  EXPECT_EQ(get(first)[2], "GVL.var1 False");
  EXPECT_EQ(get(first)[3], "GVL.var1 True");

  // a second observer gets the values the first one has
  plc->addObserver(observer(second));
  ASSERT_TRUE(waitUntil([&]() {return get(second).size() == 2;}));
  EXPECT_EQ(get(second), initial);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(get(first).size(), 4u);  // no duplicates
  EXPECT_EQ(get(second).size(), 2u);
}

TEST_F(BeckhoffPlcTest, SendsAsynchronousRequests)
{
  Simulator simulator(Json::parse(kConfig));
  auto plc = connect(simulator);
  const Variable & obj1 = plc->variable("GVL.obj1");
  const Variable & var1 = plc->variable("GVL.var1");
  Buffer obj1_in = obj1.buffer();
  obj1_in[1]["attr1"] = 11;
  Buffer var1_in = var1.buffer();
  var1_in.view() = true;
  Buffer obj1_out = obj1.buffer();
  // pipelined: sent one after the other, waited for together
  Pending write = plc->writeAsync(obj1, obj1_in);
  Pending read = plc->readAsync(obj1, obj1_out);
  EXPECT_TRUE(write.valid());
  write.wait();
  read.wait();
  EXPECT_FALSE(read.valid());
  EXPECT_EQ(obj1_out[1]["attr1"].as<int>(), 11);
  EXPECT_TRUE(read.ready());

  // fire and forget
  plc->writeAsync(var1, var1_in);
  EXPECT_TRUE(waitUntil([&simulator]() {return simulator.command("get GVL.var1") == "True";}));

  // moved
  Pending first = plc->readAsync(obj1, obj1_out);
  Pending second = std::move(first);
  EXPECT_FALSE(first.valid());
  EXPECT_TRUE(waitUntil([&second]() {return second.ready();}));
  second.wait();
}

TEST_F(BeckhoffPlcTest, HandlesConcurrentRequests)
{
  Simulator simulator(Json::parse(kConfig));
  auto plc = connect(simulator);
  std::atomic<int> errors{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 6; ++t) {
    threads.emplace_back(
      [&, t]() {
        const Variable & obj1 = plc->variable("GVL.obj1");
        Buffer in = obj1.buffer();
        Buffer out = obj1.buffer();
        for (int i = 0; i < 100; ++i) {
          try {
            in[t]["attr1"] = i;
            Batch batch;
            batch.write(obj1, in).read(obj1, out);
            plc->execute(batch);
            plc->getVariable("GVL.obj2");
          } catch (const std::exception & e) {
            ++errors;
            ADD_FAILURE() << e.what();
          }
        }
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  EXPECT_EQ(errors.load(), 0);
}

TEST_F(BeckhoffPlcTest, ReconnectsWhenTheConnectionIsLost)
{
  Simulator simulator(Json::parse(kConfig));
  auto plc = connect(simulator);
  std::mutex mutex;
  std::vector<std::string> messages;
  plc->addObserver(
    [&](const std::string & name, const Value & value) {
      if (name == "GVL.var1") {
        std::lock_guard<std::mutex> lock(mutex);
        messages.push_back(value.str());
      }
    });
  simulator.command("drop");
  EXPECT_TRUE(waitUntil([&plc]() {return !plc->ready();}));
  try {
    plc->getVariable("GVL.var1");
  } catch (const AdsError & e) {
    EXPECT_EQ(e.code(), static_cast<long>(ads_error::kNotConnected));
  }
  EXPECT_TRUE(waitUntil([&plc]() {return plc->ready();}));
  EXPECT_TRUE(log.contains("Connection to the PLC lost"));
  EXPECT_TRUE(waitUntil([this]() {return log.contains("PLC variables set up again");}));
  plc->setVariable("GVL.var1", true);
  EXPECT_EQ(plc->getVariable("GVL.var1"), Value(true));
  // the notifications work again
  EXPECT_TRUE(
    waitUntil(
      [&]() {
        std::lock_guard<std::mutex> lock(mutex);
        return !messages.empty() && messages.back() == "True";
      }));
}

TEST_F(BeckhoffPlcTest, SetsUpAgainWhenThePlcProgramChanges)
{
  Simulator simulator(Json::parse(kConfig));
  auto plc = connect(simulator);
  const Variable & obj2 = plc->variable("GVL.obj2");
  Buffer data = obj2.buffer();
  std::uint64_t sequence = 0;
  ASSERT_TRUE(plc->waitForUpdate(obj2, data, sequence, 1s));
  simulator.command("program");  // the handles and the notifications are gone
  EXPECT_TRUE(waitUntil([this]() {return log.contains("the PLC program changed");}));
  EXPECT_TRUE(waitUntil([&plc]() {return plc->ready();}));
  simulator.command("set GVL.obj2 {'attr1': 5, 'attr2': True}");
  ASSERT_TRUE(plc->waitForUpdate(obj2, data, sequence, 2s));
  ASSERT_TRUE(waitUntil([&]() {return plc->latest(obj2, data) && data["attr1"].as<int>() == 5;}));
}

TEST_F(BeckhoffPlcTest, WaitsForTheRuntimeToStartAgain)
{
  Simulator simulator(Json::parse(kConfig));
  auto plc = connect(simulator);
  simulator.command("stop");
  EXPECT_TRUE(waitUntil([&plc]() {return !plc->ready();}));
  EXPECT_THROW(plc->getVariable("GVL.var1"), AdsError);
  std::this_thread::sleep_for(300ms);
  simulator.command("start");
  EXPECT_TRUE(waitUntil([&plc]() {return plc->ready();}));
  EXPECT_NO_THROW(plc->getVariable("GVL.var1"));
}

TEST_F(BeckhoffPlcTest, SharesTheConnectionBetweenInstances)
{
  Simulator simulator(Json::parse(kConfig));
  auto first = connect(simulator);
  auto second = connect(simulator);
  first->setVariable("GVL.var1", true);
  EXPECT_EQ(second->getVariable("GVL.var1"), Value(true));
  first.reset();
  second->setVariable("GVL.var1", false);
  EXPECT_EQ(second->getVariable("GVL.var1"), Value(false));
  // a variable of another instance is rejected
  auto third = connect(simulator);
  Buffer data = third->buffer("GVL.var1");
  EXPECT_THROW(second->read(third->variable("GVL.var1"), data), ConfigError);
}

TEST_F(BeckhoffPlcTest, ExecutesABatchInOnePlcCycle)
{
  // requests are processed at the end of the 5 ms PLC cycle they arrive in
  Simulator simulator(Json::parse(kConfig), 5.0);
  auto plc = connect(simulator);
  const Variable & var1 = plc->variable("GVL.var1");
  const Variable & obj1 = plc->variable("GVL.obj1");
  Buffer var1_in = var1.buffer();
  Buffer obj1_out = obj1.buffer();
  Batch batch;
  batch.write(var1, var1_in).read(obj1, obj1_out);
  const auto measure = [](const std::function<void()> & function) {
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 20; ++i) {
      function();
    }
    return (std::chrono::steady_clock::now() - start) / 20;
  };
  const auto batch_time = measure([&]() {plc->execute(batch);});
  const auto sequential_time = measure(
    [&]() {
      plc->write(var1, var1_in);
      plc->read(obj1, obj1_out);
    });
  EXPECT_LT(batch_time, 7500us);
  EXPECT_GT(sequential_time, 8ms);
}

TEST_F(BeckhoffPlcTest, DoesNotAllocateMemoryInLoops)
{
  Simulator simulator(Json::parse(kConfig));
  Json params = config(simulator);
  params["health_check_ms"] = 0;  // a request of the internal thread, that allocates
  BeckhoffPLC plc(params);
  const Variable & obj1 = plc.variable("GVL.obj1");
  const Variable & obj2 = plc.variable("GVL.obj2");
  const Variable & var1 = plc.variable("GVL.var1");
  Buffer obj1_in = obj1.buffer();
  Buffer obj1_out = obj1.buffer();
  Buffer obj2_out = obj2.buffer();
  Buffer var1_in = var1.buffer();
  const Field<std::uint16_t> attr1(obj1.type(), "[2].attr2.attr1");
  Batch batch;
  batch.write(obj1, obj1_in).write(var1, var1_in).read(obj1, obj1_out).read(obj2, obj2_out);
  int total = 0;
  const auto iteration = [&](int i) {
    attr1.set(obj1_in, static_cast<std::uint16_t>(i));
    obj1_in[3]["attr3"][1] = i % 2 == 0;
    var1_in.view() = i % 2 == 0;    // changes a monitored variable: notifications
    plc.execute(batch);
    plc.read(obj2, obj2_out);
    plc.write(var1, var1_in);
    Pending pending = plc.writeAsync(obj1, obj1_in);
    pending.wait();
    plc.latest(obj2, obj2_out);
    total += attr1.get(obj1_out) + obj1_out[2]["attr2"]["attr1"].as<int>();
  };
  for (int i = 0; i < 20; ++i) {  // warm up: the buffers of the client grow to their size
    iteration(i);
  }
  g_allocations = 0;
  g_count_allocations = true;
  for (int i = 0; i < 200; ++i) {
    iteration(i);
  }
  g_count_allocations = false;
  EXPECT_EQ(g_allocations.load(), 0u);
  EXPECT_GT(total, 0);
}
