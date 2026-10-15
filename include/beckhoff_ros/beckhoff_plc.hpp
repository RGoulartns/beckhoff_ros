#ifndef BECKHOFF_ROS__BECKHOFF_PLC_HPP_
#define BECKHOFF_ROS__BECKHOFF_PLC_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

#include "beckhoff_ros/plc_types.hpp"
#include "beckhoff_ros/value.hpp"

namespace beckhoff_ros
{

/// Raised when an ADS request fails (pyads.ADSError in the Python version).
class AdsError : public std::runtime_error
{
public:
  /// `what` describes the failed operation; the ADS error code and its meaning are appended.
  AdsError(const std::string & what, long code);
  long code() const {return code_;}

private:
  long code_;
};

/// Short description of an ADS return code, e.g. "symbol not found" for 1808.
std::string adsErrorText(long code);

/// Communication with a Beckhoff PLC (TwinCAT 3) through ADS, using Beckhoff's AdsLib.
///
/// The structures (DUTs) used by the PLC program and the variables to access are described at
/// runtime by the "PLC" object of the JSON configuration file, so the same code works for any
/// PLC project:
///   net_id                AMS Net Id of the PLC (its IP address followed by ".1.1")
///   client_net_id         AMS Net Id of this computer (only used by createRoute)
///   username, password    PLC credentials (only used by createRoute)
///   structures            {"StructName": {"field": type, ...}, ...}, in dependency order
///   variables             {"GVL.name": type, ...}
///   variables_to_monitor  variables whose changes are notified to the observers
///   variables_to_write    variables that can be written with setVariable
///   pack                  optional, structure packing (0, the default, is TwinCAT's default
///                         layout; 1 corresponds to {attribute 'pack_mode' := '1'})
/// where a type is either a type name ("bool", "uint16", "real", "StructName", ...) or an array
/// given as [type name, size].
///
/// PLC values are exchanged as Value objects: structures are dicts, arrays are lists and
/// elementary types are bool/int/float.
///
/// This is the C++ port of beckhoff_ros/beckhoff_plc.py.
class BeckhoffPLC
{
public:
  /// Receives the name of a monitored variable and its new value.
  using Observer = std::function<void (const std::string & source, const Value & msg)>;

  enum class LogLevel { Debug, Info, Warn, Error };
  using Logger = std::function<void (LogLevel level, const std::string & message)>;

  /// Loads the structures and variables, connects to the PLC (TwinCAT 3 PLC port 851), adds
  /// notifications for variables_to_monitor and gets handles for variables_to_write.
  /// Throws ConfigError for invalid parameters and AdsError if the PLC cannot be reached or a
  /// variable does not exist on the PLC. Messages are logged to stderr unless a logger is given.
  explicit BeckhoffPLC(const nlohmann::ordered_json & params, Logger logger = nullptr);
  ~BeckhoffPLC();

  BeckhoffPLC(const BeckhoffPLC &) = delete;
  BeckhoffPLC & operator=(const BeckhoffPLC &) = delete;

  /// Adds an observer, then reads the monitored variables and notifies their current values.
  /// Changes of monitored variables are notified from an AdsLib thread; notifications received
  /// before the first observer is added are dropped.
  void addObserver(Observer observer);

  /// Calls every observer with the given variable name and value.
  void notify(const std::string & source, const Value & msg);

  /// Writes a variable listed in variables_to_write. Structures are given as dicts and arrays as
  /// lists, e.g. setVariable("GVL.obj2", Value::dict({{"attr1", 5}, {"attr2", true}})).
  /// Throws ConfigError if the variable is not writable, ValueError if the value does not match
  /// its type and AdsError if the write fails. Thread safe.
  void setVariable(const std::string & name, const Value & value);

  /// Reads a variable listed in variables. Throws ConfigError if the variable is unknown and
  /// AdsError if the read fails. Thread safe.
  Value getVariable(const std::string & name);

  /// Reads every variable of variables_to_monitor and notifies its value to the observers.
  void readInitialValues();

  /// Adds a route for this computer (client_net_id) to the PLC, using username and password.
  /// Throws AdsError if the PLC refuses it.
  void createRoute();
  /// Same, for the given parameters (it can be used before a BeckhoffPLC is created).
  static void createRoute(const nlohmann::ordered_json & params);

  /// IP address contained in an AMS Net Id: "192.168.1.10.1.1" -> "192.168.1.10".
  static std::string netId2ip(const std::string & net_id);

  /// Known type names, including the structures of the configuration.
  const TypeRegistry & types() const {return types_;}
  /// Type of a variable listed in variables, or nullptr.
  PlcTypePtr variableType(const std::string & name) const;

private:
  struct Ads;  // AdsLib connection, handles and notifications

  void loadStructures(const nlohmann::ordered_json & structures);
  void loadVariables(const nlohmann::ordered_json & variables);
  /// Handles a notification of variables_to_monitor[index] (called from an AdsLib thread).
  void updateCB(std::size_t index, const std::uint8_t * data, std::size_t size);
  const PlcTypePtr & requireVariable(const std::string & name) const;
  void log(LogLevel level, const std::string & message) const;

  nlohmann::ordered_json params_;
  std::size_t pack_ = 0;
  TypeRegistry types_;
  std::map<std::string, PlcTypePtr> variables_;
  std::vector<std::string> monitored_;
  Logger logger_;

  std::recursive_mutex observers_mutex_;  // also serializes the notifications
  std::vector<Observer> observers_;

  std::unique_ptr<Ads> ads_;  // last member: destroyed first, before what notifications use
};

}  // namespace beckhoff_ros

#endif  // BECKHOFF_ROS__BECKHOFF_PLC_HPP_
