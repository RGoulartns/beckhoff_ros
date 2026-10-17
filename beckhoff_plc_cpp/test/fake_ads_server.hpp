#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace beckhoff {
struct Config;

namespace test {

/// Minimal ADS server emulating a TwinCAT PLC runtime, to test and benchmark
/// without hardware. It speaks AMS/TCP on port 48898 (the port the ADS
/// library always connects to) and supports what BeckhoffPLC uses: handles by
/// name, read/write by handle, symbol information, sum read/write, ADS state
/// and device notifications (on change and cyclic). Symbol names are case
/// insensitive, like in TwinCAT.
class FakeAdsServer {
 public:
  struct Options {
    std::string address = "127.0.0.1";
    std::uint16_t tcpPort = 48898;
    std::uint16_t adsPort = 851;  ///< other target ports get "target port not found"
    bool sumCommands = true;      ///< false: answer sum commands with "service not supported"
  };

  FakeAdsServer();
  explicit FakeAdsServer(Options options);  ///< throws std::runtime_error if the port is in use
  ~FakeAdsServer();
  FakeAdsServer(const FakeAdsServer&) = delete;
  FakeAdsServer& operator=(const FakeAdsServer&) = delete;

  /// Adds a PLC variable, zero-initialized.
  void addSymbol(const std::string& name, std::size_t size, const std::string& typeName = "");
  /// Adds every variable of a configuration, with the size of its type.
  void addSymbols(const Config& config);

  std::vector<std::uint8_t> value(const std::string& name) const;
  /// Changes a variable like the PLC program would, sending the on-change
  /// notifications.
  void setValue(const std::string& name, const std::vector<std::uint8_t>& value);

  /// Invalidates all handles, like an online change.
  void invalidateHandles();
  /// Closes all connections and forgets handles and notifications, like a
  /// PLC restart.
  void restart();
  void setSumCommands(bool enabled);

  /// Number of requests received with the given ADS command id (2: read,
  /// 3: write, 6: add notification, 9: read/write, ...), or of all requests.
  std::size_t requests(std::uint16_t command) const;
  std::size_t requests() const;
  std::size_t openHandles() const;
  std::size_t activeNotifications() const;
  std::size_t connections() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace test
}  // namespace beckhoff
