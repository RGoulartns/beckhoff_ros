#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "beckhoff_plc/config.hpp"
#include "beckhoff_plc/errors.hpp"
#include "beckhoff_plc/types.hpp"
#include "beckhoff_plc/value.hpp"

namespace beckhoff {

class Variable;

namespace detail {
class PlcCore;

/// Zero-initialized temporary buffer, on the stack for small values.
class ScratchBuffer {
 public:
  explicit ScratchBuffer(std::size_t size) {
    if (size <= sizeof(local_)) {
      std::memset(local_, 0, size);
      data_ = local_;
    } else {
      heap_.assign(size, 0);
      data_ = heap_.data();
    }
  }
  ScratchBuffer(const ScratchBuffer&) = delete;
  ScratchBuffer& operator=(const ScratchBuffer&) = delete;
  std::uint8_t* data() noexcept { return data_; }

 private:
  alignas(std::max_align_t) std::uint8_t local_[256];
  std::vector<std::uint8_t> heap_;
  std::uint8_t* data_;
};
}  // namespace detail

/// ADS timestamp: 100 ns intervals since 1601-01-01 UTC (Windows FILETIME).
std::chrono::system_clock::time_point fileTimeToSystemTime(std::uint64_t fileTime);

/// A new value of a monitored variable.
struct Notification {
  /// Name of the variable (as in the configuration).
  const std::string& name;
  /// The received value. It points into a buffer that is only valid during
  /// the callback: copy it (Value(n.value), n.value.as<T>()) to keep it.
  ConstValueRef value;
  /// When the PLC sampled the value (see fileTimeToSystemTime()). For the
  /// initial values read when an observer is added, the time of the read.
  std::uint64_t timestamp;
  const Variable& variable;

  std::chrono::system_clock::time_point time() const { return fileTimeToSystemTime(timestamp); }
  template <class T>
  T as() const {
    return value.as<T>();
  }
};

/// Function called with the new values of monitored variables.
///
/// Callbacks run in the notification thread of the ADS library (and, for
/// initial values, in the thread that adds the observer or calls
/// readInitialValues()). The callbacks of one BeckhoffPLC are never run
/// concurrently. Keep them short: they delay the following notifications.
/// They may read and write variables, but must not destroy the BeckhoffPLC
/// or call reconnect().
///
/// Right after connecting, the same value may be delivered twice: the PLC
/// also sends the current value of a variable when its notification is added.
using Callback = std::function<void(const Notification&)>;

/// Identifies an observer, see BeckhoffPLC::removeObserver().
using ObserverId = std::uint64_t;

/// ADS state of the PLC runtime.
enum class AdsState : std::uint16_t {
  Invalid = 0,
  Idle = 1,
  Reset = 2,
  Init = 3,
  Start = 4,
  Run = 5,
  Stop = 6,
  SaveConfig = 7,
  LoadConfig = 8,
  PowerFailure = 9,
  PowerGood = 10,
  Error = 11,
  Shutdown = 12,
  Suspend = 13,
  Resume = 14,
  Config = 15,
  Reconfig = 16,
  Stopping = 17,
  Incompatible = 18,
  Exception = 19,
};
std::string toString(AdsState state);

/// A PLC variable listed in the configuration. Each variable keeps an ADS
/// handle, so reading or writing it takes a single ADS round trip. Obtain
/// it once with BeckhoffPLC::variable() to avoid looking it up by name on
/// every access (valid as long as the BeckhoffPLC exists).
class Variable {
 public:
  Variable(const Variable&) = delete;
  Variable& operator=(const Variable&) = delete;

  const std::string& name() const noexcept { return name_; }
  const Type& type() const noexcept { return *type_; }
  const TypePtr& typePtr() const noexcept { return type_; }
  std::size_t size() const noexcept { return type_->size(); }
  /// Listed in "variables_to_write".
  bool isWritable() const noexcept { return writable_; }
  /// Listed in "variables_to_monitor".
  bool isMonitored() const noexcept { return monitored_; }

  /// Zero-initialized value of this variable's type.
  Value createValue() const { return Value(type_); }

  /// Reads the current value.
  Value read() const;
  /// Reads into `out`, reusing its memory if it already has this type (no
  /// allocation, the fastest way to read a variable repeatedly).
  void read(Value& out) const;
  /// Reads into existing memory of the same layout.
  void read(ValueRef out) const;
  /// Reads as T: a number, std::string, std::vector, Json, or a C++ struct
  /// with the same memory layout (see ConstValueRef::as()).
  template <class T>
  T read() const;

  /// Writes the variable (it must be listed in "variables_to_write"). Accepts
  /// a Value/ValueRef of the same type, numbers, strings, std::vector, a C++
  /// struct with the same memory layout, or JSON (the Python dict/list form):
  ///     plc.variable("GVL.obj2").write({{"attr1", 5}, {"attr2", true}});
  template <class T>
  void write(const T& value);
  void write(const Json& value);

  /// Raw access: `size` must be equal to size().
  void readRaw(void* data, std::size_t size) const;
  void writeRaw(const void* data, std::size_t size);

  /// Calls `callback` with every new value (monitored variables only).
  /// With `initialValue`, the current value is read and delivered first.
  ObserverId subscribe(Callback callback, bool initialValue = true);

 private:
  friend class detail::PlcCore;
  Variable(detail::PlcCore& core, std::string name, TypePtr type, bool writable, bool monitored,
           NotificationSettings notification);
  void writeValue(ConstValueRef value);
  void checkSize(std::size_t size) const;
  void checkWritable() const;

  detail::PlcCore* core_;
  std::string name_;
  TypePtr type_;
  bool writable_;
  bool monitored_;
  NotificationSettings notification_;
  // Protected by the request mutex of the core.
  mutable std::uint32_t handle_ = 0;
  std::uint32_t notificationHandle_ = 0;
  std::uint32_t notificationId_ = 0;
};

/// Communication with a Beckhoff PLC over ADS, configured with the same
/// parameters (JSON file) as the Python BeckhoffPLC class.
///
///     auto plc = beckhoff::BeckhoffPLC::fromFile("params.json");
///     plc.addObserver([](const beckhoff::Notification& n) {
///       std::cout << n.name << " = " << n.value << "\n";
///     });
///     beckhoff::Value obj = plc.createValue("CustomStruct2_s");
///     obj["attr1"] = 42;
///     plc.setVariable("GVL.obj1", ...);
///     bool b = plc.getVariable<bool>("GVL.var1");
///
/// The constructor connects to the PLC, gets a handle for every configured
/// variable (checking its size against the PLC symbol) and registers the
/// notifications of the monitored variables. All member functions are
/// thread-safe; requests are serialized (the ADS library handles one request
/// at a time per connection).
///
/// On POSIX systems the first BeckhoffPLC sets SIGPIPE to "ignore" if it
/// still has its default action, like Python does: the ADS library would
/// otherwise terminate the process when writing to a connection the PLC
/// closed (e.g. while it restarts).
class BeckhoffPLC {
 public:
  explicit BeckhoffPLC(Config config);
  /// Same parameters as the Python version: the content of the "PLC" object
  /// of params.json (or the whole document).
  explicit BeckhoffPLC(const Json& params);
  /// Use beckhoff::Json (ordered) instead: nlohmann::json sorts object
  /// members, which would reorder structure fields. See Config::fromJson().
  template <class J, std::enable_if_t<std::is_same_v<J, nlohmann::json>, int> = 0>
  explicit BeckhoffPLC(const J& params) = delete;
  /// Loads the parameter file (e.g. config/params.json) and connects.
  static BeckhoffPLC fromFile(const std::string& path, const std::string& key = "PLC");

  ~BeckhoffPLC();
  BeckhoffPLC(BeckhoffPLC&&) noexcept;
  BeckhoffPLC& operator=(BeckhoffPLC&&) noexcept;

  // ---- Variables ----------------------------------------------------------

  /// Configured variable (KeyError if unknown).
  Variable& variable(const std::string& name);
  const Variable& variable(const std::string& name) const;
  bool hasVariable(const std::string& name) const noexcept;
  /// Configured variables, in configuration order.
  std::vector<std::string> variableNames() const;

  /// Reads a variable (one ADS round trip).
  Value getVariable(const std::string& name) const { return variable(name).read(); }
  /// Reads into `out`, reusing its memory (see Variable::read()).
  void getVariable(const std::string& name, Value& out) const { variable(name).read(out); }
  /// Reads a variable as T, e.g. getVariable<bool>("GVL.var1").
  template <class T>
  T getVariable(const std::string& name) const {
    return variable(name).read<T>();
  }

  /// Reads several variables in a single ADS round trip (sum command).
  std::vector<Value> getVariables(const std::vector<std::string>& names) const;
  /// Same, reusing the memory of `out` (no allocation once the types match).
  void getVariables(const std::vector<std::string>& names, std::vector<Value>& out) const;

  /// Writes a variable listed in "variables_to_write" (one ADS round trip).
  /// See Variable::write() for the accepted values; with JSON it accepts the
  /// values of the Python version:
  ///     plc.setVariable("GVL.var1", true);
  ///     plc.setVariable("GVL.obj2", {{"attr1", 5}, {"attr2", true}});
  template <class T>
  void setVariable(const std::string& name, const T& value) {
    variable(name).write(value);
  }
  void setVariable(const std::string& name, const Json& value) { variable(name).write(value); }

  /// Writes several variables in a single ADS round trip (sum command):
  ///     plc.setVariables({{"GVL.var1", true}, {"GVL.var2", {1, 2, 3, 4, 5}}});
  void setVariables(const Json& values);
  void setVariables(const std::vector<std::pair<std::string, ConstValueRef>>& values);

  // ---- Notifications ------------------------------------------------------

  /// Calls `callback` with every new value of every monitored variable. With
  /// `initialValues`, the current values are read and delivered to this
  /// observer first (the Python version reads them for all observers).
  ObserverId addObserver(Callback callback, bool initialValues = true);
  /// Calls `callback` with the new values of one monitored variable.
  ObserverId subscribe(const std::string& name, Callback callback, bool initialValue = true);
  /// Removes an observer or subscription. When called outside of a callback,
  /// the callback is not running anymore when this returns.
  void removeObserver(ObserverId id);
  /// Reads all monitored variables and delivers their values to all
  /// observers, like readInitialValues() in Python.
  void readInitialValues();
  /// Called with the errors that cannot be thrown to the caller (exceptions
  /// thrown by callbacks, malformed notifications). Default: print to stderr.
  void setErrorHandler(std::function<void(const std::string&)> handler);

  // ---- Types --------------------------------------------------------------

  /// Zero-initialized value of a type (see TypeRegistry::parse()):
  ///     auto obj = plc.createValue("CustomStruct2_s");
  ///     auto list = plc.createValue("ARRAY[0..9] OF CustomStruct2_s");
  Value createValue(std::string_view type) const;
  /// Value initialized from JSON: plc.createValue("CustomStruct1_s", {{"attr1", 5}, {"attr2", true}})
  Value createValue(std::string_view type, const Json& json) const;
  /// Built-in types plus the structures and aliases of the configuration.
  const TypeRegistry& types() const;
  const Config& config() const;

  // ---- Connection ---------------------------------------------------------

  /// ADS state of the PLC (Run, Stop, ...).
  AdsState readState() const;
  /// Reconnects: new connection, handles and notifications (e.g. after the
  /// PLC was restarted or a new program was downloaded). Observers are kept.
  void reconnect();

  /// Adds a route to this computer on the PLC (like the TwinCAT "Add Route"
  /// dialog), using net_id/ip_address, client_net_id/client_ip_address,
  /// username, password and route_name. Not needed if the route exists.
  static void createRoute(const Config& config);
  /// IP address part of an AMS Net ID ("192.168.1.10.1.1" -> "192.168.1.10").
  static std::string netId2ip(const std::string& netId) { return netIdToIp(netId); }
  /// Verbosity of the messages the ADS library prints to stderr:
  /// 0 verbose, 1 info, 2 warnings (default), 3 errors, 4 nothing.
  static void setAdsLogLevel(int level);

 private:
  std::unique_ptr<detail::PlcCore> core_;
};

// ---------------------------------------------------------------------------
// Template implementations

template <class T>
T Variable::read() const {
  using U = std::decay_t<T>;
  if constexpr (std::is_same_v<U, Value>) {
    return read();
  } else if constexpr (detail::isRawCopyable<U> && std::is_default_constructible_v<U>) {
    checkSize(sizeof(U));
    U result;
    readRaw(static_cast<void*>(&result), sizeof(U));
    return result;
  } else {
    detail::ScratchBuffer buffer(size());
    readRaw(buffer.data(), size());
    return ConstValueRef(type(), buffer.data()).template as<U>();
  }
}

template <class T>
void Variable::write(const T& value) {
  using U = std::decay_t<T>;
  checkWritable();
  if constexpr (std::is_base_of_v<ConstValueRef, U> || std::is_same_v<U, Value>) {
    writeValue(static_cast<ConstValueRef>(value));
  } else if constexpr (std::is_same_v<U, Json>) {
    write(static_cast<const Json&>(value));
  } else if constexpr (detail::isRawCopyable<U>) {
    checkSize(sizeof(U));
    writeRaw(static_cast<const void*>(&value), sizeof(U));
  } else {
    detail::ScratchBuffer buffer(size());
    ValueRef(type(), buffer.data()).set(value);
    writeRaw(buffer.data(), size());
  }
}

}  // namespace beckhoff
