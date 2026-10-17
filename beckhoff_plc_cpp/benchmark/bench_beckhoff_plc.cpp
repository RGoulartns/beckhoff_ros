// Latency of the C++ BeckhoffPLC, measured with the same operations as
// bench_python.py measures for the Python version (see compare.py).
//   bench_beckhoff_plc <params.json> [--net-id ID] [--iterations N]
// Uses the variables of benchmark/params.json. Prints one JSON object per line.
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

#include "beckhoff_plc/beckhoff_plc.hpp"

namespace {

using Clock = std::chrono::steady_clock;

void report(const std::string& operation, int iterations, const std::function<void()>& run) {
  for (int i = 0; i < std::min(200, iterations / 5 + 1); ++i) run();  // warm up
  std::vector<double> us;
  us.reserve(static_cast<std::size_t>(iterations));
  for (int i = 0; i < iterations; ++i) {
    const auto start = Clock::now();
    run();
    us.push_back(std::chrono::duration<double, std::micro>(Clock::now() - start).count());
  }
  std::sort(us.begin(), us.end());
  const auto percentile = [&](double p) { return us[std::min(us.size() - 1, static_cast<std::size_t>(p * us.size()))]; };
  beckhoff::Json line = {{"op", operation},
                         {"n", us.size()},
                         {"mean_us", std::accumulate(us.begin(), us.end(), 0.0) / static_cast<double>(us.size())},
                         {"p50_us", percentile(0.50)},
                         {"p90_us", percentile(0.90)},
                         {"p99_us", percentile(0.99)}};
  std::cout << line.dump() << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: bench_beckhoff_plc <params.json> [--net-id ID] [--iterations N]\n";
    return 2;
  }
  std::string netId;
  int iterations = 2000;
  for (int i = 2; i + 1 < argc; i += 2) {
    const std::string arg = argv[i];
    if (arg == "--net-id") netId = argv[i + 1];
    if (arg == "--iterations") iterations = std::stoi(argv[i + 1]);
  }
  beckhoff::Config config = beckhoff::Config::fromFile(argv[1]);
  if (!netId.empty()) config.netId = netId;
  beckhoff::BeckhoffPLC plc(config);

  // The values the Python benchmark writes: lists and dicts.
  beckhoff::Json list = beckhoff::Json::array();
  for (int i = 0; i < 10; ++i) {
    list.push_back({{"attr1", i}, {"attr2", {{"attr1", i}, {"attr2", true}}}, {"attr3", {true, false}}});
  }

  // ---- operations measured for both versions
  report("getVariable struct", iterations, [&] { plc.getVariable("GVL.obj2"); });
  report("getVariable 10 structs", iterations, [&] { plc.getVariable("GVL.obj1"); });
  bool flag = false;
  report("setVariable bool", iterations, [&] { plc.setVariable("GVL.var1", flag = !flag); });
  report("setVariable 10 structs (dicts)", iterations, [&] { plc.setVariable("GVL.obj1", list); });
  report("read 3 variables", iterations, [&] { plc.getVariables({"GVL.obj1", "GVL.obj2", "GVL.var1"}); });

  // Write a monitored variable and wait for its notification.
  std::mutex mutex;
  std::condition_variable received;
  int expected = 0;
  bool arrived = false;
  plc.subscribe(
      "GVL.obj2",
      [&](const beckhoff::Notification& n) {
        std::lock_guard<std::mutex> lock(mutex);
        if (n.value["attr1"].as<int>() == expected) {
          arrived = true;
          received.notify_one();
        }
      },
      false);
  report("notification round trip", iterations, [&] {
    std::unique_lock<std::mutex> lock(mutex);
    expected = (expected + 1) % 60000;
    arrived = false;
    const beckhoff::Json value = {{"attr1", expected}, {"attr2", expected % 2 == 0}};
    lock.unlock();
    plc.setVariable("GVL.obj2", value);
    lock.lock();
    if (!received.wait_for(lock, std::chrono::seconds(2), [&] { return arrived; })) {
      throw std::runtime_error("no notification");
    }
  });

  // Conversion between values and bytes only (no ADS request)
  beckhoff::Value obj1 = plc.variable("GVL.obj1").createValue();
  report("convert dicts to bytes (no I/O)", iterations, [&] { obj1.fromJson(list); });
  report("convert bytes to dicts (no I/O)", iterations, [&] { obj1.toJson(); });

  // ---- C++ only: typed values and pre-resolved variables (no allocation)
  beckhoff::Variable& obj1Var = plc.variable("GVL.obj1");
  beckhoff::Variable& obj2Var = plc.variable("GVL.obj2");
  beckhoff::Value obj2 = obj2Var.createValue();
  report("C++ Variable::read into Value", iterations, [&] { obj2Var.read(obj2); });
  report("C++ Variable::write Value", iterations, [&] { obj1Var.write(obj1); });
  std::vector<beckhoff::ValueRef> attrs;
  for (std::size_t i = 0; i < 10; ++i) attrs.push_back(obj1[i]["attr2"]["attr1"]);
  int counter = 0;
  report("C++ set 10 fields (no I/O)", iterations, [&] {
    for (beckhoff::ValueRef& attr : attrs) attr = ++counter & 0xFFFF;
  });
  return 0;
}
