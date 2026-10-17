// Stand-alone fake PLC for manual tests and benchmarks:
//   fake_ads_server --config ../config/params.json
// then connect with net_id "127.0.0.1.1.1" (C++ or the Python version).
#include <signal.h>

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "beckhoff_plc/config.hpp"
#include "fake_ads_server.hpp"

namespace {
void usage() {
  std::cerr << "usage: fake_ads_server [--config params.json [--key PLC]] [--symbol NAME:SIZE]...\n"
               "                       [--address 127.0.0.1] [--port 48898] [--no-sum-commands]\n";
}
}  // namespace

int main(int argc, char** argv) {
  beckhoff::test::FakeAdsServer::Options options;
  std::string configPath;
  std::string key = "PLC";
  std::vector<std::pair<std::string, std::size_t>> symbols;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool hasValue = i + 1 < argc;
    if (arg == "--config" && hasValue) {
      configPath = argv[++i];
    } else if (arg == "--key" && hasValue) {
      key = argv[++i];
    } else if (arg == "--address" && hasValue) {
      options.address = argv[++i];
    } else if (arg == "--port" && hasValue) {
      options.tcpPort = static_cast<std::uint16_t>(std::stoi(argv[++i]));
    } else if (arg == "--symbol" && hasValue) {
      const std::string symbol = argv[++i];
      const auto colon = symbol.rfind(':');
      if (colon == std::string::npos) {
        usage();
        return 2;
      }
      symbols.emplace_back(symbol.substr(0, colon), std::stoul(symbol.substr(colon + 1)));
    } else if (arg == "--no-sum-commands") {
      options.sumCommands = false;
    } else {
      usage();
      return 2;
    }
  }

  // Handle SIGINT/SIGTERM with sigwait(): block them before any thread starts.
  sigset_t signals;
  sigemptyset(&signals);
  sigaddset(&signals, SIGINT);
  sigaddset(&signals, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &signals, nullptr);

  try {
    beckhoff::test::FakeAdsServer server(options);
    if (!configPath.empty()) server.addSymbols(beckhoff::Config::fromFile(configPath, key));
    for (const auto& [name, size] : symbols) server.addSymbol(name, size);
    std::cout << "fake ADS server listening on " << options.address << ":" << options.tcpPort << std::endl;
    int signal = 0;
    sigwait(&signals, &signal);
    std::cout << "served " << server.requests() << " requests" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "fake_ads_server: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
