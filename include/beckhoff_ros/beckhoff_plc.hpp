#ifndef BECKHOFF_ROS__BECKHOFF_PLC_HPP_
#define BECKHOFF_ROS__BECKHOFF_PLC_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"

#include "beckhoff_ros/ads_error.hpp"
#include "beckhoff_ros/buffer.hpp"
#include "beckhoff_ros/plc_types.hpp"
#include "beckhoff_ros/value.hpp"

namespace beckhoff_ros
{

class BeckhoffPLC;
class Batch;

/// Time stamp of a value sent by the PLC (PLC clock).
using Timestamp = std::chrono::system_clock::time_point;

namespace detail
{
struct RequestId
{
  std::uint32_t slot = 0;
  std::uint32_t invoke_id = 0;
};
}  // namespace detail

/// A variable of the configuration ("variables"). Obtained with BeckhoffPLC::variable() and
/// valid as long as its BeckhoffPLC.
class Variable
{
public:
  const std::string & name() const {return name_;}
  const PlcTypePtr & type() const {return type_;}
  std::size_t size() const {return type_->size();}
  /// Listed in variables_to_monitor: notifications keep its latest value (BeckhoffPLC::latest).
  bool monitored() const {return monitored_;}
  /// Listed in variables_to_write (the variables the ROS service can write).
  bool writable() const {return writable_;}
  /// Zero initialized Buffer for this variable.
  Buffer buffer() const {return Buffer(type_);}

private:
  friend class BeckhoffPLC;
  friend class Batch;
  friend class Pending;
  Variable(
    std::string name, PlcTypePtr type, std::size_t index, const BeckhoffPLC * owner,
    bool monitored, bool writable)
  : name_(std::move(name)), type_(std::move(type)), index_(index), owner_(owner),
    monitored_(monitored), writable_(writable) {}

  std::string name_;
  PlcTypePtr type_;
  std::size_t index_;
  const BeckhoffPLC * owner_;
  bool monitored_;
  bool writable_;
};

/// Result of an asynchronous operation (BeckhoffPLC::readAsync, writeAsync, executeAsync).
///
/// wait() returns when the responses arrive (or throws). If a Pending is destroyed without
/// being waited for, the request completes in the background and its errors are logged, which
/// allows "fire and forget" writes. A Pending must not outlive its BeckhoffPLC, nor the buffers
/// and the Batch it uses.
class Pending
{
public:
  Pending() = default;
  Pending(Pending && other) noexcept;
  Pending & operator=(Pending && other) noexcept;
  ~Pending();

  /// False once wait() has been called (or for a default constructed / moved from Pending).
  bool valid() const {return plc_ != nullptr;}
  /// True if wait() would not block: the responses arrived, or their timeout elapsed.
  bool ready() const;
  /// Waits for the responses, copies the data read to the buffers and throws AdsError if a
  /// request failed (the data of the successful reads of a batch is copied anyway).
  void wait();

private:
  friend class BeckhoffPLC;
  enum class Kind : std::uint8_t { Read, Write, Batch };

  void detach() noexcept;

  BeckhoffPLC * plc_ = nullptr;
  Kind kind_ = Kind::Read;
  const Variable * variable_ = nullptr;  // Read, Write
  Buffer * buffer_ = nullptr;            // Read
  Batch * batch_ = nullptr;              // Batch
  detail::RequestId request_;            // Read, Write
  std::uint64_t setup_ = 0;              // setup of the handles used
};

/// Reads and writes executed together: the writes, then the reads, are sent at once (several
/// variables are combined in an ADS sum command), and the PLC processes them in this order. A
/// batch costs a single round trip to the PLC.
///
/// Build a batch once and execute it repeatedly: no memory is allocated after its first
/// execution. The buffers are referenced, not copied: they must stay valid, and the data to
/// write is taken from them when the batch is executed.
class Batch
{
public:
  Batch() = default;
  Batch(const Batch &) = delete;
  Batch & operator=(const Batch &) = delete;

  /// Writes the content of `buffer` to `variable`.
  Batch & write(const Variable & variable, const Buffer & buffer);
  /// Reads `variable` into `buffer`.
  Batch & read(const Variable & variable, Buffer & buffer);
  void clear();
  bool empty() const {return writes_.empty() && reads_.empty();}

private:
  friend class BeckhoffPLC;
  friend class Pending;

  struct Item
  {
    const Variable * variable;
    const Buffer * source;    // write
    Buffer * destination;     // read
  };
  struct Request
  {
    bool write;
    bool sum;  // sum command (or single read/write)
    std::size_t begin;
    std::size_t end;
    detail::RequestId id;
  };

  void checkIdle() const;

  std::vector<Item> writes_;
  std::vector<Item> reads_;
  std::vector<Request> requests_;
  std::vector<std::uint8_t> frames_;
  const BeckhoffPLC * plc_ = nullptr;
  bool pending_ = false;
};

/// Communication with a Beckhoff PLC (TwinCAT 3) through ADS.
///
/// The structures (DUTs) used by the PLC program and the variables to access are described at
/// runtime by the "PLC" object of the JSON configuration file (see README), so the same code
/// works for any PLC project.
///
/// Two APIs are available:
///  - Values (getVariable, setVariable, observers): structures are Value dicts, arrays are
///    lists. Convenient, but every access converts the data and allocates memory.
///  - Buffers (read, write, Batch, latest, onUpdate): the data is exchanged as is, typed
///    accessors (View, Field) read and write its items. Nothing is allocated once the buffers
///    exist, which suits control loops.
///
/// Every variable gets a symbol handle when the BeckhoffPLC is created, so a read or a write
/// is a single request, and a Batch combines any number of them. Requests are pipelined: they
/// can be sent from several threads at the same time, without waiting for each other.
/// Monitored variables are updated by ADS notifications: latest() returns their last value
/// without any request.
///
/// The connection is re-established when it is lost, and the handles and notifications are
/// created again (also when the PLC program changes); requests fail with AdsError meanwhile.
class BeckhoffPLC
{
public:
  /// Receives the name of a monitored variable and its new value.
  using Observer = std::function<void (const std::string & source, const Value & msg)>;
  /// Receives the data of a monitored variable sent by a notification.
  using UpdateCallback =
    std::function<void (const Variable & variable, const ConstView & data, Timestamp time)>;
  using CallbackId = std::size_t;

  enum class LogLevel { Debug, Info, Warn, Error };
  using Logger = std::function<void (LogLevel level, const std::string & message)>;

  /// Loads the configuration, connects to the PLC, gets the handles of the variables and adds
  /// the notifications of the monitored variables. Throws ConfigError for invalid parameters
  /// (or variables that do not match the PLC) and AdsError if the PLC cannot be reached or a
  /// variable does not exist on the PLC. Messages are logged to stderr unless a logger is
  /// given; it is called from internal threads.
  explicit BeckhoffPLC(const nlohmann::ordered_json & params, Logger logger = nullptr);
  ~BeckhoffPLC();

  BeckhoffPLC(const BeckhoffPLC &) = delete;
  BeckhoffPLC & operator=(const BeckhoffPLC &) = delete;

  // ---------------------------------------------------------------------------------------
  // Variables

  /// Variable listed in "variables"; throws ConfigError if there is none with this name.
  const Variable & variable(const std::string & name) const;
  /// Same, nullptr if there is none.
  const Variable * findVariable(const std::string & name) const;
  /// Every variable, in the order of the configuration.
  const std::vector<Variable> & variables() const;
  /// Zero initialized Buffer for a variable.
  Buffer buffer(const std::string & name) const {return variable(name).buffer();}
  /// Type of a variable, or nullptr.
  PlcTypePtr variableType(const std::string & name) const;
  /// Known type names, including the structures of the configuration.
  const TypeRegistry & types() const;

  // ---------------------------------------------------------------------------------------
  // Buffers

  /// Reads a variable (one round trip). Throws AdsError if it fails, ValueError if the buffer
  /// is not of the type of the variable. Thread safe, like all the functions below.
  void read(const Variable & variable, Buffer & buffer);
  /// Writes a variable (one round trip).
  void write(const Variable & variable, const Buffer & buffer);
  /// Executes the writes then the reads of a batch (one round trip).
  void execute(Batch & batch);

  /// Same, without waiting for the response: returns when the request is sent. The data to
  /// write is copied before returning; the data read is copied to the buffer by wait().
  Pending readAsync(const Variable & variable, Buffer & buffer);
  Pending writeAsync(const Variable & variable, const Buffer & buffer);
  Pending executeAsync(Batch & batch);

  /// Copies the last value of a monitored variable (sent by the PLC with a notification) to
  /// `buffer`, without any request. Returns false if no value was received yet (or the
  /// connection is lost). `time` receives the PLC time of the value.
  bool latest(const Variable & variable, Buffer & buffer, Timestamp * time = nullptr) const;
  /// Waits until a monitored variable has a value newer than `sequence` (0 for any value),
  /// copies it to `buffer` and updates `sequence`. Returns false on timeout. With a cyclic
  /// notification, a loop calling it runs at the rate of the PLC.
  bool waitForUpdate(
    const Variable & variable, Buffer & buffer, std::uint64_t & sequence,
    std::chrono::nanoseconds timeout, Timestamp * time = nullptr) const;
  /// Calls `callback` with each notification of a monitored variable. The callback runs on
  /// the ADS receive thread: it must return quickly, and must not wait for requests (async
  /// requests can be sent, but not waited for) nor add or remove callbacks.
  CallbackId onUpdate(const Variable & variable, UpdateCallback callback);
  void removeCallback(CallbackId id);

  // ---------------------------------------------------------------------------------------
  // Values

  /// Adds an observer: it receives the current value of every monitored variable, then every
  /// change. Observers are called from an internal thread, one at a time.
  void addObserver(Observer observer);

  /// Writes a variable. Structures are given as dicts and arrays as lists, e.g.
  /// setVariable("GVL.obj2", Value::dict({{"attr1", 5}, {"attr2", true}})).
  /// Throws ConfigError if the variable is unknown, ValueError if the value does not match its
  /// type and AdsError if the write fails.
  void setVariable(const std::string & name, const Value & value);

  /// Reads a variable. Throws ConfigError if the variable is unknown and AdsError if the read
  /// fails.
  Value getVariable(const std::string & name);

  // ---------------------------------------------------------------------------------------
  // Connection

  /// True when connected, with valid handles (requests can be sent).
  bool ready() const;
  /// AMS Net Id of this computer, as used for the connection (see "local_net_id").
  std::string localNetId() const;

  /// Adds a route for this computer (client_net_id) to the PLC, using username and password.
  /// Throws AdsError if the PLC refuses it.
  void createRoute();
  /// Same, for the given parameters (it can be used before a BeckhoffPLC is created).
  static void createRoute(const nlohmann::ordered_json & params);

  /// IP address contained in an AMS Net Id: "192.168.1.10.1.1" -> "192.168.1.10".
  static std::string netId2ip(const std::string & net_id);

private:
  friend class Pending;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace beckhoff_ros

#endif  // BECKHOFF_ROS__BECKHOFF_PLC_HPP_
