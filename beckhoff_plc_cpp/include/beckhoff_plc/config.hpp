#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "beckhoff_plc/types.hpp"
#include "beckhoff_plc/value.hpp"

namespace beckhoff {

/// How the PLC sends the notifications of a monitored variable.
struct NotificationSettings {
  enum class Mode {
    OnChange,  ///< send the value when it changes (ADSTRANS_SERVERONCHA)
    Cyclic,    ///< send the value periodically (ADSTRANS_SERVERCYCLE)
  };
  Mode mode = Mode::OnChange;
  /// OnChange: how often the PLC checks the variable; Cyclic: the period.
  /// The PLC rounds it up to its task cycle time. ADS resolution is 100 ns.
  /// The default (100 ns, i.e. every PLC cycle) is the pyads default.
  std::chrono::nanoseconds cycleTime{100};
  /// Longest time the PLC may hold back a notification. Default as in pyads.
  std::chrono::nanoseconds maxDelay{100};
};

/// A PLC variable (symbol) and its type.
struct VariableDefinition {
  std::string name;  ///< symbol name, e.g. "GVL.var1" or "MAIN.fbAxis.nState"
  std::string type;  ///< type specification, see TypeRegistry::parse()
};

/// A variable whose changes are sent by the PLC (ADS notifications).
struct MonitoredVariable {
  std::string name;
  /// Uses Config::notification when empty.
  std::optional<NotificationSettings> notification;
};

/// Configuration of a BeckhoffPLC.
///
/// Usually loaded from the JSON parameter file of the Python version (see
/// fromFile()), but it can also be filled in code, e.g. from ROS parameters.
/// The comments give the JSON key of each member; keys marked "C++ only" are
/// extensions ignored by the Python version.
struct Config {
  // ---- Connection
  std::string netId;                        ///< "net_id": AMS Net ID of the PLC (required)
  std::string ipAddress;                    ///< "ip_address" (C++ only): PLC address; default: first 4 bytes of net_id
  std::uint16_t port = 851;                 ///< "port" (C++ only): ADS port (851: TwinCAT 3 PLC1, 801: TwinCAT 2)
  std::string clientNetId;                  ///< "client_net_id": local AMS Net ID (process-wide); default: <local IP>.1.1
  std::chrono::milliseconds timeout{5000};  ///< "timeout_ms" (C++ only): ADS request timeout

  // ---- Route creation (BeckhoffPLC::createRoute())
  std::string username;         ///< "username": PLC user
  std::string password;         ///< "password"
  std::string clientIpAddress;  ///< "client_ip_address" (C++ only): default: first 4 bytes of client_net_id
  std::string routeName;        ///< "route_name" (C++ only): default "beckhoff_plc"
  bool createRoute = false;     ///< "create_route" (C++ only): add the route on the PLC when connecting

  // ---- Types
  int packMode = 8;                          ///< "pack_mode" (C++ only): default structure alignment (TwinCAT 3: 8)
  std::vector<AliasDefinition> aliases;      ///< "aliases" (C++ only): {"E_State": "INT"}
  std::vector<StructDefinition> structures;  ///< "structures" (pack mode per structure: "pack_modes", C++ only)

  // ---- Variables
  std::vector<VariableDefinition> variables;          ///< "variables"
  std::vector<MonitoredVariable> variablesToMonitor;  ///< "variables_to_monitor"
  std::vector<std::string> variablesToWrite;          ///< "variables_to_write"
  NotificationSettings notification;                  ///< "notification" (C++ only): defaults for monitored variables
  /// "verify_sizes" (C++ only): compare the size of every variable with the
  /// PLC symbol when connecting (catches wrong structure definitions early).
  bool verifySizes = true;

  /// Keys of the JSON object that were not recognized (typos?). Informative.
  std::vector<std::string> ignoredKeys;

  /// Reads the configuration from JSON. `json` is either the parameters
  /// object (what the Python version receives) or a document holding it
  /// under `key` ("PLC" by default, as in the provided params.json).
  static Config fromJson(const Json& json, const std::string& key = "PLC");
  /// Same, parsing JSON text (comments are allowed).
  static Config fromJsonText(std::string_view text, const std::string& key = "PLC");
  /// Same, reading a file (comments are allowed).
  static Config fromFile(const std::string& path, const std::string& key = "PLC");

  /// nlohmann::json sorts object members alphabetically, which would change
  /// the order (and thus the layout) of the structure members: parse with
  /// beckhoff::Json (nlohmann::ordered_json) or use fromJsonText()/fromFile().
  template <class J, std::enable_if_t<std::is_same_v<J, nlohmann::json>, int> = 0>
  static Config fromJson(const J& json, const std::string& key = "PLC") = delete;

  /// Builds the types (structures and aliases). Throws ConfigError.
  TypeRegistry createTypeRegistry() const;
  /// Checks names, types and lists. Throws ConfigError. Called by the
  /// fromXxx() functions and by BeckhoffPLC.
  void validate() const;
};

/// IP address encoded in an AMS Net ID: its first four bytes
/// ("192.168.1.10.1.1" -> "192.168.1.10"), like netId2ip() in Python.
std::string netIdToIp(const std::string& netId);

/// True for a valid AMS Net ID: six numbers from 0 to 255 separated by dots.
bool isValidNetId(std::string_view netId);

}  // namespace beckhoff
