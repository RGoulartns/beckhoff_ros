// Measures the latency of PLC requests with the configuration of the beckhoff_ros node:
//   ros2 run beckhoff_ros benchmark config.json [--read GVL.a] [--write GVL.b] [--count 1000]
// Writes are only done for the variables given with --write: their current value is read
// first and written back, so the PLC keeps its values (but the PLC program sees the writes).

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ads_connection.hpp"
#include "ads_protocol.hpp"
#include "beckhoff_ros/beckhoff_plc.hpp"

namespace
{

using beckhoff_ros::BeckhoffPLC;
using beckhoff_ros::Buffer;
using Clock = std::chrono::steady_clock;

void usage()
{
  std::cerr <<
    "Usage: benchmark CONFIG.json [--read VARIABLE]... [--write VARIABLE]... [--count N]\n"
    "Measures the latency of PLC requests. CONFIG.json is the configuration of the\n"
    "beckhoff_ros node (its \"PLC\" object is used). Without --read, the monitored variables\n"
    "are read. The variables given with --write are read, then written back with the same\n"
    "value.\n";
}

/// Times `count` runs of `function` (after a few warm up runs) and prints statistics.
void measure(const std::string & name, std::size_t count, const std::function<void()> & function)
{
  for (int i = 0; i < 5; ++i) {
    function();
  }
  std::vector<double> times;
  times.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto start = Clock::now();
    function();
    times.push_back(std::chrono::duration<double, std::milli>(Clock::now() - start).count());
  }
  std::sort(times.begin(), times.end());
  double sum = 0;
  for (const double time : times) {
    sum += time;
  }
  const auto percentile = [&times](double p) {
    return times[std::min(times.size() - 1, static_cast<std::size_t>(p * times.size()))];
  };
  std::printf(
    "%-66s %8.3f %8.3f %8.3f %8.3f %8.3f\n", name.c_str(), times.front(), sum / times.size(),
    percentile(0.5), percentile(0.99), times.back());
}

/// The requests of the previous version of beckhoff_ros (Beckhoff's AdsLib, like pyads):
/// a read gets a symbol handle, reads by handle and releases the handle, one request at a time.
class LegacyClient : public beckhoff_ros::ads::PortListener
{
public:
  explicit LegacyClient(const nlohmann::ordered_json & params)
  {
    namespace ads = beckhoff_ros::ads;
    const std::string net_id = params.at("net_id").get<std::string>();
    ads::AmsNetId::parse(net_id, target_.net_id);
    target_.port = params.value("ams_port", ads::kPlcPort);
    ads::ConnectionOptions options;
    options.host = params.value("host", BeckhoffPLC::netId2ip(net_id));
    options.port = params.value("tcp_port", ads::kTcpPort);
    options.local_net_id = params.value("local_net_id", "");
    connection_ = ads::Connection::open(options);  // shared with the BeckhoffPLC
    port_ = connection_->addPort(this);
  }
  ~LegacyClient() override {connection_->removePort(port_);}

  void onNotification(const beckhoff_ros::ads::NotificationSample &) override {}
  void onConnectionChanged(bool, std::uint64_t) override {}
  void onLog(beckhoff_ros::ads::LogLevel, const std::string &) override {}

  /// One request, waiting for its response; returns the response data.
  std::vector<std::uint8_t> request(
    std::uint16_t command, std::uint32_t group, std::uint32_t offset,
    const std::vector<std::uint32_t> & lengths, const std::string & data)
  {
    namespace ads = beckhoff_ros::ads;
    const auto ticket = connection_->reserve(
      port_, Clock::now() + std::chrono::seconds(1));
    std::vector<std::uint8_t> frame;
    ads::FrameWriter writer(frame);
    writer.begin(
      target_, ads::AmsAddress{connection_->localNetId(), port_}, command,
      ticket.invoke_id);
    writer.u32(group);
    writer.u32(offset);
    for (const auto length : lengths) {
      writer.u32(length);
    }
    writer.bytes(data.data(), data.size());
    writer.end();
    if (!connection_->send(frame.data(), frame.size(), connection_->epoch())) {
      connection_->fail(ticket, beckhoff_ros::ads_error::kNotConnected);
    }
    const std::uint32_t error = connection_->wait(ticket);
    std::vector<std::uint8_t> response = connection_->response(ticket);
    connection_->release(ticket);
    if (error != 0 || response.size() < 4 || ads::get32(response.data()) != 0) {
      throw beckhoff_ros::AdsError(
              "Legacy request failed", error != 0 ? error :
              (response.size() < 4 ? beckhoff_ros::ads_error::kInvalidResponse :
              ads::get32(response.data())));
    }
    return response;
  }

  void readByName(const std::string & name, std::uint32_t size)
  {
    namespace ads = beckhoff_ros::ads;
    const auto handle = request(
      ads::kReadWrite, ads::kSymbolHandleByName, 0,
      {4, static_cast<std::uint32_t>(name.size())}, name);
    const std::uint32_t value = ads::get32(handle.data() + 8);
    request(ads::kRead, ads::kSymbolValueByHandle, value, {size}, "");
    request(
      ads::kWrite, ads::kSymbolReleaseHandle, 0, {4},
      std::string(reinterpret_cast<const char *>(handle.data() + 8), 4));
  }

private:
  std::shared_ptr<beckhoff_ros::ads::Connection> connection_;
  beckhoff_ros::ads::AmsAddress target_;
  std::uint16_t port_ = 0;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string config_path;
  std::vector<std::string> reads;
  std::vector<std::string> writes;
  std::size_t count = 1000;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if ((argument == "--read" || argument == "--write" || argument == "--count") && i + 1 < argc) {
      const std::string value = argv[++i];
      if (argument == "--read") {
        reads.push_back(value);
      } else if (argument == "--write") {
        writes.push_back(value);
      } else {
        count = std::max<std::size_t>(1, std::strtoul(value.c_str(), nullptr, 10));
      }
    } else if (config_path.empty() && argument[0] != '-') {
      config_path = argument;
    } else {
      usage();
      return 2;
    }
  }
  if (config_path.empty()) {
    usage();
    return 2;
  }

  try {
    std::ifstream file(config_path);
    if (!file) {
      throw std::runtime_error("Unable to open " + config_path);
    }
    const auto params = nlohmann::ordered_json::parse(file).at("PLC");
    BeckhoffPLC plc(params, [](BeckhoffPLC::LogLevel level, const std::string & message) {
        if (level >= BeckhoffPLC::LogLevel::Warn) {
          std::cerr << message << std::endl;
        }
      });
    if (reads.empty()) {
      for (const auto & variable : plc.variables()) {
        if (variable.monitored()) {
          reads.push_back(variable.name());
        }
      }
    }
    if (reads.empty() && writes.empty()) {
      throw std::runtime_error("Nothing to measure: give variables with --read or --write");
    }

    std::vector<const beckhoff_ros::Variable *> read_variables;
    std::vector<Buffer> read_buffers;
    for (const auto & name : reads) {
      read_variables.push_back(&plc.variable(name));
      read_buffers.push_back(plc.variable(name).buffer());
    }
    std::vector<const beckhoff_ros::Variable *> write_variables;
    std::vector<Buffer> write_buffers;
    for (const auto & name : writes) {
      write_variables.push_back(&plc.variable(name));
      write_buffers.push_back(plc.variable(name).buffer());
      plc.read(*write_variables.back(), write_buffers.back());  // written back unchanged
    }

    std::printf(
      "%zu runs per test, times in ms\n%-66s %8s %8s %8s %8s %8s\n", count, "", "min", "mean",
      "median", "p99", "max");
    for (std::size_t i = 0; i < reads.size(); ++i) {
      measure("read " + reads[i], count, [&]() {plc.read(*read_variables[i], read_buffers[i]);});
    }
    for (std::size_t i = 0; i < writes.size(); ++i) {
      measure(
        "write " + writes[i], count, [&]() {
          plc.write(*write_variables[i], write_buffers[i]);
        });
    }

    beckhoff_ros::Batch batch;
    for (std::size_t i = 0; i < writes.size(); ++i) {
      batch.write(*write_variables[i], write_buffers[i]);
    }
    for (std::size_t i = 0; i < reads.size(); ++i) {
      batch.read(*read_variables[i], read_buffers[i]);
    }
    if (reads.size() + writes.size() > 1) {
      measure(
        "batch: " + std::to_string(writes.size()) + " writes + " + std::to_string(reads.size()) +
        " reads (1 round trip)", count, [&]() {plc.execute(batch);});
      measure(
        "one after the other (" + std::to_string(reads.size() + writes.size()) +
        " round trips)", count, [&]() {
          for (std::size_t i = 0; i < writes.size(); ++i) {
            plc.write(*write_variables[i], write_buffers[i]);
          }
          for (std::size_t i = 0; i < reads.size(); ++i) {
            plc.read(*read_variables[i], read_buffers[i]);
          }
        });
    }

    for (std::size_t i = 0; i < reads.size(); ++i) {
      if (read_variables[i]->monitored()) {
        measure(
          "latest " + reads[i] + " (no request)", count, [&]() {
            plc.latest(*read_variables[i], read_buffers[i]);
          });
      }
    }

    // the requests of the previous version
    LegacyClient legacy(params);
    for (std::size_t i = 0; i < reads.size(); ++i) {
      measure(
        "previous version: read " + reads[i] + " (3 round trips)", count, [&]() {
          legacy.readByName(reads[i], static_cast<std::uint32_t>(read_variables[i]->size()));
        });
    }
    if (!writes.empty() && !reads.empty()) {
      measure(
        "previous version: write " + writes[0] + " + read " + reads[0] + " (4 round trips)",
        count, [&]() {
          plc.write(*write_variables[0], write_buffers[0]);
          legacy.readByName(reads[0], static_cast<std::uint32_t>(read_variables[0]->size()));
        });
    }
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
