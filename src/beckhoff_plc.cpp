#include "beckhoff_ros/beckhoff_plc.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ads_connection.hpp"
#include "ads_protocol.hpp"

namespace beckhoff_ros
{

namespace
{

using ads::Clock;
using ads::get32;
using Json = nlohmann::ordered_json;

// Route name used by createRoute (the same as the Python version)
constexpr const char * kRouteName = "jetson-to-plc";
// Read length of the symbol information (AdsSymbolEntry: 30 bytes, then the name, the type
// name and the comment). Only the size and the type name are used.
constexpr std::uint32_t kSymbolInfoLength = 512;
constexpr std::size_t kSymbolInfoChunk = 100;  // variables per request
constexpr std::size_t kMaxQueuedEvents = 10000;
constexpr auto kReleaseTimeout = std::chrono::milliseconds(500);

const std::set<std::string> kKnownParameters = {
  "net_id", "host", "tcp_port", "ams_port", "local_net_id", "client_net_id", "username",
  "password", "timeout_ms", "reconnect", "reconnect_interval_ms", "health_check_ms",
  "check_sizes", "notification", "receive_thread", "pack", "structures", "variables",
  "variables_to_monitor", "variables_to_write"};

const Json & require(const Json & params, const std::string & key)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    throw ConfigError("Missing PLC parameter '" + key + "'");
  }
  return *it;
}

std::string requireString(const Json & params, const std::string & key)
{
  const auto & value = require(params, key);
  if (value.is_string()) {
    return value.get<std::string>();
  }
  if (value.is_number()) {  // e.g. a numeric password
    return value.dump();
  }
  throw ConfigError("PLC parameter '" + key + "' must be a string");
}

std::string optionalString(const Json & params, const std::string & key)
{
  return params.contains(key) ? requireString(params, key) : std::string();
}

bool optionalBool(const Json & params, const std::string & key, bool fallback)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return fallback;
  }
  if (!it->is_boolean()) {
    throw ConfigError("PLC parameter '" + key + "' must be true or false");
  }
  return it->get<bool>();
}

double optionalNumber(
  const Json & params, const std::string & key, double fallback, double min, double max,
  const std::string & where = "PLC parameter")
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return fallback;
  }
  if (!it->is_number() || it->get<double>() < min || it->get<double>() > max) {
    throw ConfigError(
            where + " '" + key + "' must be a number between " + Json(min).dump() + " and " +
            Json(max).dump());
  }
  return it->get<double>();
}

std::vector<Json> optionalList(const Json & params, const std::string & key)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return {};
  }
  if (!it->is_array()) {
    throw ConfigError("PLC parameter '" + key + "' must be a list of variable names");
  }
  return std::vector<Json>(it->begin(), it->end());
}

ads::AmsNetId parseNetId(const std::string & net_id, const std::string & key)
{
  ads::AmsNetId result;
  if (!ads::AmsNetId::parse(net_id, result)) {
    throw ConfigError(
            "Invalid AMS Net Id '" + net_id + "' for '" + key +
            "' (expected 6 numbers, e.g. 192.168.1.10.1.1)");
  }
  return result;
}

/// Notification attributes, in the units of ADS (100 ns). The defaults are those of pyads:
/// the PLC checks the value every task cycle and sends it right away.
struct NotificationSettings
{
  std::uint32_t mode = ads::kTransModeOnChange;
  std::uint32_t cycle_time = 1;
  std::uint32_t max_delay = 1;
};

NotificationSettings parseNotificationSettings(
  const Json & spec, const NotificationSettings & defaults, const std::string & where)
{
  if (!spec.is_object()) {
    throw ConfigError(where + " must be a dictionary");
  }
  for (auto it = spec.begin(); it != spec.end(); ++it) {
    if (it.key() != "mode" && it.key() != "cycle_time_ms" && it.key() != "max_delay_ms" &&
      it.key() != "name")
    {
      throw ConfigError(where + ": unknown key '" + it.key() + "'");
    }
  }
  NotificationSettings settings = defaults;
  if (spec.contains("mode")) {
    const auto & mode = spec["mode"];
    if (mode == "on_change") {
      settings.mode = ads::kTransModeOnChange;
    } else if (mode == "cyclic") {
      settings.mode = ads::kTransModeCyclic;
    } else {
      throw ConfigError(where + ": 'mode' must be \"on_change\" or \"cyclic\"");
    }
  }
  constexpr double kMaxMs = 400000.0;  // fits 32 bits in 100 ns units
  const auto units = [](double ms) {return static_cast<std::uint32_t>(ms * 10000.0 + 0.5);};
  if (spec.contains("cycle_time_ms")) {
    settings.cycle_time = std::max<std::uint32_t>(
      1, units(optionalNumber(spec, "cycle_time_ms", 0, 0, kMaxMs, where)));
  }
  if (spec.contains("max_delay_ms")) {
    settings.max_delay = units(optionalNumber(spec, "max_delay_ms", 0, 0, kMaxMs, where));
  }
  return settings;
}

Timestamp toTimestamp(std::uint64_t filetime)
{
  if (filetime < ads::kFileTimeUnixEpoch) {
    return Timestamp();
  }
  const std::chrono::nanoseconds since_epoch((filetime - ads::kFileTimeUnixEpoch) * 100);
  return Timestamp(std::chrono::duration_cast<Timestamp::duration>(since_epoch));
}

std::uint64_t nowFileTime()
{
  const auto since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  return ads::kFileTimeUnixEpoch + static_cast<std::uint64_t>(since_epoch / 100);
}

/// Result of a response starting with an ADS result code (Write, AddNotification, ...).
std::uint32_t resultOf(std::uint32_t error, const std::uint8_t * data, std::size_t size)
{
  if (error != 0) {
    return error;
  }
  return size < 4 ? ads_error::kInvalidResponse : get32(data);
}

std::uint32_t resultOf(std::uint32_t error, const std::vector<std::uint8_t> & data)
{
  return resultOf(error, data.data(), data.size());
}

/// Data of a Read or ReadWrite response (result, length, data).
std::uint32_t readData(
  std::uint32_t error, const std::vector<std::uint8_t> & response, const std::uint8_t *& data,
  std::size_t & size)
{
  const std::uint32_t result = resultOf(error, response);
  if (result != 0) {
    return result;
  }
  if (response.size() < 8) {
    return ads_error::kInvalidResponse;
  }
  size = get32(response.data() + 4);
  if (size > response.size() - 8) {
    return ads_error::kInvalidResponse;
  }
  data = response.data() + 8;
  return 0;
}

/// Errors meaning that the handles and notifications of the client are no longer valid.
bool invalidatesHandles(std::uint32_t error)
{
  return error == ads_error::kSymbolNotFound || error == ads_error::kSymbolVersionInvalid ||
         error == ads_error::kTargetPortNotFound || error == ads_error::kInvalidIndexOffset ||
         error == ads_error::kSymbolNotActive;
}

/// Allows a message per second, to avoid flooding the log with repeated errors.
class RateLimiter
{
public:
  /// Returns true if a message can be logged; `suppressed` is then the number of messages
  /// that were not since the previous one.
  bool allow(std::size_t & suppressed)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = Clock::now();
    if (logged_ && now - last_ < std::chrono::seconds(1)) {
      ++suppressed_;
      return false;
    }
    logged_ = true;
    last_ = now;
    suppressed = suppressed_;
    suppressed_ = 0;
    return true;
  }

private:
  std::mutex mutex_;
  bool logged_ = false;
  Clock::time_point last_;
  std::size_t suppressed_ = 0;
};

}  // namespace

// -----------------------------------------------------------------------------------------
// Implementation

struct BeckhoffPLC::Impl : public ads::PortListener
{
  struct VariableState
  {
    Impl * impl = nullptr;
    std::size_t index = 0;
    std::atomic<std::uint32_t> handle{0};
    NotificationSettings notification;
    std::uint32_t notification_handle = 0;  // setup thread only

    // latest value of a monitored variable
    mutable std::mutex mutex;
    mutable std::condition_variable cv;
    std::vector<std::uint8_t> latest;
    std::uint64_t timestamp = 0;  // FILETIME
    std::uint64_t sequence = 0;
    bool valid = false;
    bool from_read = false;  // latest was read by the setup, not notified
    mutable std::size_t waiters = 0;

    // guarded by Impl::callbacks_mutex
    std::vector<std::pair<CallbackId, UpdateCallback>> callbacks;
  };

  /// For the worker thread: a notification to deliver to the observers, or a new observer.
  struct Event
  {
    bool new_observer = false;  // the next of new_observers
    std::size_t variable = 0;
    std::uint64_t sequence = 0;
    std::vector<std::uint8_t> data;
  };

  struct NewObserver
  {
    Observer observer;
    // for the first observer: the latest values (variable, sequence, data)
    std::vector<std::pair<std::size_t, std::uint64_t>> snapshot;
    std::vector<std::vector<std::uint8_t>> snapshot_data;
  };

  /// Events in a ring buffer: once it has grown, queuing a notification only copies its data
  /// (the receive thread does not allocate memory).
  class EventQueue
  {
public:
    bool empty() const {return count_ == 0;}
    std::size_t size() const {return count_;}
    /// The event to fill at the back of the queue.
    Event & push()
    {
      if (count_ == events_.size()) {
        std::vector<Event> grown(std::max<std::size_t>(16, events_.size() * 2));
        for (std::size_t i = 0; i < count_; ++i) {
          grown[i] = std::move(events_[(head_ + i) % events_.size()]);
        }
        events_ = std::move(grown);
        head_ = 0;
      }
      return events_[(head_ + count_++) % events_.size()];
    }
    /// Moves the front event to `event`, which gives its data buffer in exchange.
    void pop(Event & event)
    {
      Event & front = events_[head_];
      event.new_observer = front.new_observer;
      event.variable = front.variable;
      event.sequence = front.sequence;
      std::swap(event.data, front.data);
      head_ = (head_ + 1) % events_.size();
      --count_;
    }

private:
    std::vector<Event> events_;
    std::size_t head_ = 0;
    std::size_t count_ = 0;
  };

  Impl(BeckhoffPLC & owner, const Json & params, Logger logger);
  ~Impl() override;

  // PortListener (receive thread)
  void onNotification(const ads::NotificationSample & sample) override;
  void onConnectionChanged(bool connected, std::uint64_t epoch) override;
  void onLog(ads::LogLevel level, const std::string & message) override;

  void log(LogLevel level, const std::string & message) const;
  void loadConfiguration();
  void connect();

  // requests
  ads::AmsAddress source() const {return ads::AmsAddress{connection->localNetId(), port};}
  std::uint64_t readyEpoch(const char * operation, const std::string & name) const;
  ads::Ticket reserve(
    Clock::time_point deadline, ads::ResponseHook hook = nullptr, void * context = nullptr);
  void begin(ads::FrameWriter & writer, std::uint16_t command, const ads::Ticket & ticket)
  {
    writer.begin(target, source(), command, ticket.invoke_id);
  }
  void send(
    const std::vector<std::uint8_t> & frames, std::uint64_t epoch,
    const ads::Ticket * tickets, std::size_t count);
  void checkVariable(const Variable & variable) const;
  void checkMonitored(const Variable & variable) const;
  void checkBuffer(const Variable & variable, const Buffer & buffer) const;
  void handleRequestError(std::uint32_t error, std::uint64_t setup);

  // setup of the handles and notifications
  void setup();
  void getHandles(std::uint64_t epoch, bool check_sizes);
  void addNotifications(std::uint64_t epoch);
  void readInitialValues(std::uint64_t epoch);
  int readSymbolVersion(std::uint64_t epoch, std::uint32_t & error);
  void releaseAll(std::uint64_t epoch);
  void requestSetup(const std::string & reason);
  void invalidateLatest();
  /// Sends `count` requests (their data written by `build(writer, i)`) and waits for them.
  /// Returns their errors and responses; throws AdsError if there are no free request slots.
  template<typename Build>
  void exchange(
    std::uint64_t epoch, std::size_t count, std::uint16_t command, Build && build,
    std::vector<std::uint32_t> & errors, std::vector<std::vector<std::uint8_t>> & responses,
    std::chrono::milliseconds timeout, ads::ResponseHook hook = nullptr,
    const std::vector<void *> * contexts = nullptr);

  // worker thread
  void startWorker();
  void runWorker();
  void addObserver(NewObserver & observer);
  void processEvent(Event & event);
  void deliver(const Observer & observer, std::size_t variable, const Value & value);
  void healthCheck();

  // detached requests
  static void onDetachedWrite(
    void * context, std::uint32_t error, const std::uint8_t * data, std::size_t size);
  static void onDetachedSumWrite(
    void * context, std::uint32_t error, const std::uint8_t * data, std::size_t size);
  static void onDetachedRead(
    void * context, std::uint32_t error, const std::uint8_t * data, std::size_t size);
  static void onNotificationAdded(
    void * context, std::uint32_t error, const std::uint8_t * data, std::size_t size);
  void logDetachedError(const std::string & what, std::uint32_t error);

  BeckhoffPLC & owner;
  Json params;
  Logger logger;

  // configuration
  std::size_t pack = 0;
  TypeRegistry types;
  std::vector<Variable> variables;
  std::map<std::string, std::size_t> index_by_name;
  std::vector<std::size_t> monitored;
  std::vector<std::unique_ptr<VariableState>> state;
  std::string net_id;
  ads::AmsAddress target;
  ads::ConnectionOptions connection_options;
  std::chrono::milliseconds timeout{1000};
  std::chrono::milliseconds health_check_interval{1000};
  bool check_sizes = true;

  // connection
  std::shared_ptr<ads::Connection> connection;
  std::uint16_t port = 0;
  std::atomic<bool> sum_commands{true};
  std::atomic<std::uint64_t> ready_epoch{0};   // connection epoch of the handles, 0: not ready
  std::atomic<std::uint64_t> setup_count{0};   // incremented by each successful setup
  std::mutex setup_mutex;
  std::uint64_t handles_epoch = 0;  // connection epoch of the current handles (setup thread)
  int symbol_version = -1;          // -1: not supported by the PLC

  std::mutex notifications_mutex;
  std::unordered_map<std::uint32_t, std::size_t> by_notification;

  mutable std::mutex callbacks_mutex;
  CallbackId next_callback = 1;

  // worker thread
  std::thread worker;
  std::mutex events_mutex;
  std::condition_variable events_cv;
  bool stopping = false;
  bool setup_requested = false;
  Clock::time_point next_setup_attempt;
  bool queue_events = false;  // there are observers
  EventQueue events;
  std::deque<NewObserver> new_observers;
  std::size_t dropped_events = 0;
  // worker thread only
  std::vector<Observer> observers;
  std::vector<std::vector<std::uint8_t>> last_data;  // per variable, last value delivered
  std::vector<std::uint64_t> last_sequence;

  mutable RateLimiter detached_errors;
  mutable RateLimiter notification_errors;
};

BeckhoffPLC::Impl::Impl(BeckhoffPLC & plc, const Json & parameters, Logger log_function)
: owner(plc), params(parameters), logger(std::move(log_function))
{
  loadConfiguration();
  connect();
}

BeckhoffPLC::Impl::~Impl()
{
  {
    std::lock_guard<std::mutex> lock(events_mutex);
    stopping = true;
  }
  events_cv.notify_all();
  if (worker.joinable()) {
    worker.join();
  }
  if (connection) {
    try {
      std::lock_guard<std::mutex> lock(setup_mutex);
      if (connection->connected() && handles_epoch == connection->epoch()) {
        releaseAll(handles_epoch);
      }
    } catch (const std::exception & e) {
      log(LogLevel::Debug, std::string("Unable to release the handles: ") + e.what());
    }
    connection->removePort(port);
  }
}

void BeckhoffPLC::Impl::log(LogLevel level, const std::string & message) const
{
  if (logger) {
    try {
      logger(level, message);
    } catch (...) {
    }
    return;
  }
  static const char * const kLevels[] = {"DEBUG", "INFO", "WARN", "ERROR"};
  if (level != LogLevel::Debug) {
    std::cerr << "[beckhoff_plc] [" << kLevels[static_cast<int>(level)] << "] " << message <<
      std::endl;
  }
}

void BeckhoffPLC::Impl::onLog(ads::LogLevel level, const std::string & message)
{
  log(static_cast<LogLevel>(static_cast<int>(level)), message);
}

void BeckhoffPLC::Impl::loadConfiguration()
{
  if (!params.is_object()) {
    throw ConfigError("The PLC parameters must be a JSON object");
  }
  for (auto it = params.begin(); it != params.end(); ++it) {
    if (kKnownParameters.count(it.key()) == 0) {
      log(LogLevel::Warn, "Ignoring unknown PLC parameter '" + it.key() + "'");
    }
  }
  if (params.contains("pack")) {
    const auto & value = params["pack"];
    if (!value.is_number_integer() || value.get<std::int64_t>() < 0) {
      throw ConfigError("PLC parameter 'pack' must be a non negative integer");
    }
    pack = value.get<std::size_t>();
  }

  net_id = requireString(params, "net_id");
  target.net_id = parseNetId(net_id, "net_id");
  target.port = static_cast<std::uint16_t>(
    optionalNumber(params, "ams_port", ads::kPlcPort, 1, 65535));
  connection_options.host = optionalString(params, "host");
  if (connection_options.host.empty()) {
    connection_options.host = netId2ip(net_id);
  }
  connection_options.port = static_cast<std::uint16_t>(
    optionalNumber(params, "tcp_port", ads::kTcpPort, 1, 65535));
  connection_options.local_net_id = optionalString(params, "local_net_id");
  if (!connection_options.local_net_id.empty()) {
    parseNetId(connection_options.local_net_id, "local_net_id");
  }
  if (params.contains("client_net_id")) {
    parseNetId(requireString(params, "client_net_id"), "client_net_id");
  }
  timeout = std::chrono::milliseconds(
    static_cast<std::int64_t>(optionalNumber(params, "timeout_ms", 1000, 1, 3600000)));
  connection_options.connect_timeout = std::max(timeout, std::chrono::milliseconds(1000));
  connection_options.reconnect = optionalBool(params, "reconnect", true);
  connection_options.reconnect_interval = std::chrono::milliseconds(
    static_cast<std::int64_t>(
      optionalNumber(params, "reconnect_interval_ms", 1000, 10, 3600000)));
  health_check_interval = std::chrono::milliseconds(
    static_cast<std::int64_t>(optionalNumber(params, "health_check_ms", 1000, 0, 3600000)));
  check_sizes = optionalBool(params, "check_sizes", true);
  if (params.contains("receive_thread")) {
    const auto & thread = params["receive_thread"];
    if (!thread.is_object()) {
      throw ConfigError("PLC parameter 'receive_thread' must be a dictionary");
    }
    connection_options.receive_thread_priority = static_cast<int>(
      optionalNumber(thread, "priority", 0, 0, 99, "receive_thread parameter"));
    connection_options.receive_thread_cpu = static_cast<int>(
      optionalNumber(thread, "cpu", -1, -1, 1023, "receive_thread parameter"));
  }
  NotificationSettings default_notification;
  if (params.contains("notification")) {
    default_notification = parseNotificationSettings(
      params["notification"], default_notification, "PLC parameter 'notification'");
  }

  // structures and variables (loadStructures + loadVariables in the Python version)
  const Json structures = params.contains("structures") ? params["structures"] : Json::object();
  if (!structures.is_object()) {
    throw ConfigError("PLC parameter 'structures' must be a dictionary");
  }
  for (auto it = structures.begin(); it != structures.end(); ++it) {
    types.addStructure(it.key(), it.value(), pack);
  }
  const Json variable_specs = require(params, "variables");
  if (!variable_specs.is_object()) {
    throw ConfigError("PLC parameter 'variables' must be a dictionary");
  }
  std::vector<std::pair<std::string, PlcTypePtr>> resolved;
  for (auto it = variable_specs.begin(); it != variable_specs.end(); ++it) {
    if (it.value().is_object()) {
      throw ConfigError("Variable " + it.key() + " cannot be a dict (ie, structure)");
    }
    try {
      resolved.emplace_back(it.key(), types.resolve(it.value()));
    } catch (const ConfigError & e) {
      throw ConfigError("Variable " + it.key() + ": " + e.what());
    }
    index_by_name[it.key()] = resolved.size() - 1;
  }

  const auto find = [this](const Json & name, const std::string & list) {
    if (!name.is_string()) {
      throw ConfigError("PLC parameter '" + list + "' must be a list of variable names");
    }
    const auto it = index_by_name.find(name.get<std::string>());
    if (it == index_by_name.end()) {
      throw ConfigError(
              "Variable '" + name.get<std::string>() + "' of '" + list +
              "' is not listed in variables");
    }
    return it->second;
  };
  std::vector<bool> is_monitored(resolved.size(), false);
  std::vector<bool> is_writable(resolved.size(), false);
  std::vector<NotificationSettings> settings(resolved.size(), default_notification);
  for (const auto & entry : optionalList(params, "variables_to_monitor")) {
    std::size_t index = 0;
    if (entry.is_object()) {
      if (!entry.contains("name")) {
        throw ConfigError("A variable of 'variables_to_monitor' has no 'name'");
      }
      index = find(entry["name"], "variables_to_monitor");
      settings[index] = parseNotificationSettings(
        entry, default_notification, "Notification of '" + resolved[index].first + "'");
    } else {
      index = find(entry, "variables_to_monitor");
    }
    if (!is_monitored[index]) {
      is_monitored[index] = true;
      monitored.push_back(index);
    }
  }
  for (const auto & entry : optionalList(params, "variables_to_write")) {
    is_writable[find(entry, "variables_to_write")] = true;
  }

  variables.reserve(resolved.size());
  for (std::size_t i = 0; i < resolved.size(); ++i) {
    variables.push_back(
      Variable(resolved[i].first, resolved[i].second, i, &owner, is_monitored[i], is_writable[i]));
    auto variable_state = std::make_unique<VariableState>();
    variable_state->impl = this;
    variable_state->index = i;
    variable_state->notification = settings[i];
    if (is_monitored[i]) {
      variable_state->latest.assign(resolved[i].second->size(), 0);
    }
    state.push_back(std::move(variable_state));
  }
  last_data.resize(variables.size());
  last_sequence.assign(variables.size(), 0);
}

void BeckhoffPLC::Impl::connect()
{
  connection = ads::Connection::open(
    connection_options, [this](ads::LogLevel level, const std::string & message) {
      log(static_cast<LogLevel>(static_cast<int>(level)), message);
    });
  port = connection->addPort(this);
  const std::string local = connection->localNetId().toString();
  log(
    LogLevel::Debug, "Connected to " + connection_options.host + ":" +
    std::to_string(connection_options.port) + " as " + local + ":" + std::to_string(port));
  if (params.contains("client_net_id") && requireString(params, "client_net_id") != local &&
    connection_options.local_net_id.empty())
  {
    log(
      LogLevel::Warn, "This computer uses the AMS Net Id " + local + " for the connection, "
      "not client_net_id (" + requireString(params, "client_net_id") + "): the PLC needs a "
      "route to " + local + ", or set local_net_id");
  }
}

// -----------------------------------------------------------------------------------------
// Requests

std::uint64_t BeckhoffPLC::Impl::readyEpoch(const char * operation, const std::string & name)
const
{
  const std::uint64_t epoch = ready_epoch.load(std::memory_order_acquire);
  if (epoch == 0 || epoch != connection->epoch()) {
    throw AdsError(
            std::string(operation) + " '" + name + "': the connection to the PLC is not ready" +
            (connection->connected() ? " (setting the handles up)" : ""),
            ads_error::kNotConnected);
  }
  return epoch;
}

ads::Ticket BeckhoffPLC::Impl::reserve(
  Clock::time_point deadline, ads::ResponseHook hook, void * context)
{
  const ads::Ticket ticket = connection->reserve(port, deadline, hook, context);
  if (!ticket) {
    throw AdsError("Too many requests in flight", ads_error::kNoMemory);
  }
  return ticket;
}

void BeckhoffPLC::Impl::send(
  const std::vector<std::uint8_t> & frames, std::uint64_t epoch, const ads::Ticket * tickets,
  std::size_t count)
{
  if (!connection->send(frames.data(), frames.size(), epoch)) {
    for (std::size_t i = 0; i < count; ++i) {
      connection->fail(tickets[i], ads_error::kNotConnected);
    }
  }
}

void BeckhoffPLC::Impl::checkVariable(const Variable & variable) const
{
  if (variable.owner_ != &owner) {
    throw ConfigError("Variable '" + variable.name() + "' belongs to another BeckhoffPLC");
  }
}

void BeckhoffPLC::Impl::checkMonitored(const Variable & variable) const
{
  checkVariable(variable);
  if (!variable.monitored()) {
    throw ConfigError("Variable '" + variable.name() + "' is not listed in variables_to_monitor");
  }
}

void BeckhoffPLC::Impl::checkBuffer(const Variable & variable, const Buffer & buffer) const
{
  if (!buffer.typePtr()) {
    throw ValueError("Empty buffer used for '" + variable.name() + "'");
  }
  detail::checkSameType(buffer.type(), *variable.type());
}

void BeckhoffPLC::Impl::handleRequestError(std::uint32_t error, std::uint64_t setup)
{
  if (invalidatesHandles(error) && setup == setup_count.load()) {
    requestSetup(
      "a request failed with ADS error " + std::to_string(error) + " (" +
      adsErrorText(error) + ")");
  }
}

template<typename Build>
void BeckhoffPLC::Impl::exchange(
  std::uint64_t epoch, std::size_t count, std::uint16_t command, Build && build,
  std::vector<std::uint32_t> & errors, std::vector<std::vector<std::uint8_t>> & responses,
  std::chrono::milliseconds request_timeout, ads::ResponseHook hook,
  const std::vector<void *> * contexts)
{
  errors.assign(count, 0);
  responses.assign(count, {});
  std::vector<ads::Ticket> tickets;
  std::vector<std::uint8_t> frames;
  // at most kWave requests in flight, to leave slots for the other requests
  constexpr std::size_t kWave = ads::Connection::kSlots / 2;
  for (std::size_t first = 0; first < count; first += kWave) {
    const std::size_t last = std::min(count, first + kWave);
    tickets.clear();
    frames.clear();
    const auto deadline = Clock::now() + request_timeout;
    try {
      for (std::size_t i = first; i < last; ++i) {
        tickets.push_back(
          reserve(deadline, hook, contexts != nullptr ? (*contexts)[i] : nullptr));
        ads::FrameWriter writer(frames);
        begin(writer, command, tickets.back());
        build(writer, i);
        writer.end();
      }
    } catch (...) {
      for (const auto & ticket : tickets) {
        connection->release(ticket);
      }
      throw;
    }
    send(frames, epoch, tickets.data(), tickets.size());
    for (std::size_t i = first; i < last; ++i) {
      const ads::Ticket & ticket = tickets[i - first];
      errors[i] = connection->wait(ticket);
      responses[i] = connection->response(ticket);
      connection->release(ticket);
    }
  }
}

// -----------------------------------------------------------------------------------------
// Setup: handles, notifications

void BeckhoffPLC::Impl::setup()
{
  std::lock_guard<std::mutex> lock(setup_mutex);
  if (!connection->connected()) {
    throw AdsError("Unable to set the PLC variables up", ads_error::kNotConnected);
  }
  const std::uint64_t epoch = connection->epoch();
  ready_epoch.store(0);
  invalidateLatest();
  if (handles_epoch == epoch) {  // same connection: the previous handles may still exist
    releaseAll(epoch);
  }
  handles_epoch = epoch;
  {
    std::lock_guard<std::mutex> notifications_lock(notifications_mutex);
    by_notification.clear();
  }
  getHandles(epoch, check_sizes);
  addNotifications(epoch);
  std::uint32_t error = 0;
  symbol_version = readSymbolVersion(epoch, error);
  setup_count.fetch_add(1);
  ready_epoch.store(epoch, std::memory_order_release);
  readInitialValues(epoch);
}

void BeckhoffPLC::Impl::getHandles(std::uint64_t epoch, bool check_sizes_now)
{
  const std::size_t count = variables.size();
  std::vector<std::uint32_t> handles(count, 0);
  std::vector<std::uint32_t> results(count, ads_error::kInvalidResponse);
  std::vector<std::int64_t> plc_sizes(count, -1);
  std::vector<std::string> plc_types(count);
  std::vector<std::uint32_t> errors;
  std::vector<std::vector<std::uint8_t>> responses;

  const auto parse_symbol_info = [&](std::size_t index, const std::uint8_t * data,
    std::size_t size) {
    // AdsSymbolEntry: length, group, offset, size, type, flags, name/type/comment lengths
    if (size >= 30) {
      plc_sizes[index] = get32(data + 12);
      const std::size_t name_length = ads::get16(data + 24);
      const std::size_t type_length = ads::get16(data + 26);
      if (30 + name_length + 1 + type_length <= size) {
        plc_types[index].assign(
          reinterpret_cast<const char *>(data) + 30 + name_length + 1, type_length);
      }
    }
  };

  if (sum_commands.load()) {
    // one sum read-write request per chunk: the symbol information (if checked) and the
    // handle of each variable
    const std::size_t chunk = check_sizes_now ? kSymbolInfoChunk : ads::kMaxSumCommands / 2;
    const std::size_t chunks = (count + chunk - 1) / chunk;
    const std::size_t per_variable = check_sizes_now ? 2 : 1;
    exchange(
      epoch, chunks, ads::kReadWrite, [&](ads::FrameWriter & writer, std::size_t c) {
        const std::size_t begin = c * chunk;
        const std::size_t end = std::min(count, begin + chunk);
        const std::size_t n = (end - begin) * per_variable;
        std::uint32_t read_length = static_cast<std::uint32_t>(n * 8);
        std::uint32_t write_length = static_cast<std::uint32_t>(n * 16);
        for (std::size_t i = begin; i < end; ++i) {
          read_length += 4 + (check_sizes_now ? kSymbolInfoLength : 0);
          write_length +=
          static_cast<std::uint32_t>(variables[i].name().size() * per_variable);
        }
        writer.u32(ads::kSumReadWrite);
        writer.u32(static_cast<std::uint32_t>(n));
        writer.u32(read_length);
        writer.u32(write_length);
        for (std::size_t i = begin; i < end; ++i) {
          const auto length = static_cast<std::uint32_t>(variables[i].name().size());
          if (check_sizes_now) {
            writer.u32(ads::kSymbolInfoByNameEx);
            writer.u32(0);
            writer.u32(kSymbolInfoLength);
            writer.u32(length);
          }
          writer.u32(ads::kSymbolHandleByName);
          writer.u32(0);
          writer.u32(4);
          writer.u32(length);
        }
        for (std::size_t i = begin; i < end; ++i) {
          for (std::size_t k = 0; k < per_variable; ++k) {
            writer.bytes(variables[i].name().data(), variables[i].name().size());
          }
        }
      }, errors, responses, timeout);

    for (std::size_t c = 0; c < chunks; ++c) {
      const std::size_t begin = c * chunk;
      const std::size_t end = std::min(count, begin + chunk);
      const std::size_t n = (end - begin) * per_variable;
      const std::uint8_t * data = nullptr;
      std::size_t size = 0;
      const std::uint32_t result = readData(errors[c], responses[c], data, size);
      if (c == 0 && (result == ads_error::kServiceNotSupported ||
        result == ads_error::kInvalidIndexGroup))
      {
        log(LogLevel::Info, "The PLC does not support ADS sum commands");
        sum_commands.store(false);
        break;
      }
      if (result != 0) {
        throw AdsError("Unable to get the handles of the variables", result);
      }
      if (size < n * 8) {
        throw AdsError("Unable to get the handles of the variables", ads_error::kInvalidResponse);
      }
      std::size_t pos = n * 8;
      for (std::size_t k = 0; k < n; ++k) {
        const std::size_t index = begin + k / per_variable;
        const std::uint32_t sub_result = get32(data + k * 8);
        const std::size_t length = get32(data + k * 8 + 4);
        if (pos + length > size) {
          throw AdsError(
                  "Unable to get the handles of the variables", ads_error::kInvalidResponse);
        }
        const bool is_info = check_sizes_now && k % 2 == 0;
        if (is_info) {
          if (sub_result == 0) {
            parse_symbol_info(index, data + pos, length);
          }
        } else {
          results[index] = sub_result != 0 ? sub_result :
            (length == 4 ? 0 : ads_error::kInvalidResponse);
          if (results[index] == 0) {
            handles[index] = get32(data + pos);
          }
        }
        pos += length;
      }
    }
  }

  if (!sum_commands.load()) {
    // one request per variable, all sent at once
    const std::size_t per_variable = check_sizes_now ? 2 : 1;
    exchange(
      epoch, count * per_variable, ads::kReadWrite,
      [&](ads::FrameWriter & writer, std::size_t k) {
        const auto & name = variables[k / per_variable].name();
        const bool is_info = check_sizes_now && k % 2 == 0;
        writer.u32(is_info ? ads::kSymbolInfoByNameEx : ads::kSymbolHandleByName);
        writer.u32(0);
        writer.u32(is_info ? kSymbolInfoLength : 4);
        writer.u32(static_cast<std::uint32_t>(name.size()));
        writer.bytes(name.data(), name.size());
      }, errors, responses, timeout);
    for (std::size_t k = 0; k < count * per_variable; ++k) {
      const std::size_t index = k / per_variable;
      const std::uint8_t * data = nullptr;
      std::size_t size = 0;
      const std::uint32_t result = readData(errors[k], responses[k], data, size);
      if (check_sizes_now && k % 2 == 0) {
        if (result == 0) {
          parse_symbol_info(index, data, size);
        }
      } else {
        results[index] = result != 0 ? result : (size == 4 ? 0 : ads_error::kInvalidResponse);
        if (results[index] == 0) {
          handles[index] = get32(data);
        }
      }
    }
  }

  for (std::size_t i = 0; i < count; ++i) {
    state[i]->handle.store(handles[i], std::memory_order_relaxed);
  }

  std::string missing;
  std::uint32_t first_error = 0;
  for (std::size_t i = 0; i < count; ++i) {
    if (results[i] != 0) {
      missing += (missing.empty() ? "" : ", ") + variables[i].name() + " (" +
        (adsErrorText(results[i]).empty() ? "error " + std::to_string(results[i]) :
        adsErrorText(results[i])) + ")";
      first_error = first_error != 0 ? first_error : results[i];
    }
  }
  if (!missing.empty()) {
    throw AdsError("Unable to get the handles of the PLC variables " + missing, first_error);
  }

  std::string mismatches;
  for (std::size_t i = 0; i < count; ++i) {
    if (plc_sizes[i] >= 0 && static_cast<std::size_t>(plc_sizes[i]) != variables[i].size()) {
      mismatches += "\n  " + variables[i].name() + ": " + std::to_string(variables[i].size()) +
        " bytes in the configuration ('" + variables[i].type()->name() + "'), " +
        std::to_string(plc_sizes[i]) + " bytes on the PLC" +
        (plc_types[i].empty() ? "" : " ('" + plc_types[i] + "')");
    }
  }
  if (!mismatches.empty()) {
    throw ConfigError(
            "The size of some variables does not match the PLC (check the structures: fields, "
            "types and order, and 'pack'):" + mismatches);
  }
}

void BeckhoffPLC::Impl::onNotificationAdded(
  void * context, std::uint32_t error, const std::uint8_t * data, std::size_t size)
{
  // called by the receive thread before it processes the first notification
  auto * variable_state = static_cast<VariableState *>(context);
  if (resultOf(error, data, size) != 0 || size < 8) {
    return;
  }
  Impl & impl = *variable_state->impl;
  std::lock_guard<std::mutex> lock(impl.notifications_mutex);
  impl.by_notification[get32(data + 4)] = variable_state->index;
}

void BeckhoffPLC::Impl::addNotifications(std::uint64_t epoch)
{
  if (monitored.empty()) {
    return;
  }
  std::vector<void *> contexts;
  for (const auto index : monitored) {
    contexts.push_back(state[index].get());
  }
  std::vector<std::uint32_t> errors;
  std::vector<std::vector<std::uint8_t>> responses;
  exchange(
    epoch, monitored.size(), ads::kAddNotification,
    [&](ads::FrameWriter & writer, std::size_t k) {
      const VariableState & variable_state = *state[monitored[k]];
      writer.u32(ads::kSymbolValueByHandle);
      writer.u32(variable_state.handle.load(std::memory_order_relaxed));
      writer.u32(static_cast<std::uint32_t>(variables[monitored[k]].size()));
      writer.u32(variable_state.notification.mode);
      writer.u32(variable_state.notification.max_delay);
      writer.u32(variable_state.notification.cycle_time);
      const std::uint8_t reserved[16] = {0};
      writer.bytes(reserved, sizeof(reserved));
    }, errors, responses, timeout, &Impl::onNotificationAdded, &contexts);

  std::string failed;
  std::uint32_t first_error = 0;
  for (std::size_t k = 0; k < monitored.size(); ++k) {
    std::uint32_t result = resultOf(errors[k], responses[k]);
    if (result == 0 && responses[k].size() < 8) {
      result = ads_error::kInvalidResponse;
    }
    if (result == 0) {
      state[monitored[k]]->notification_handle = get32(responses[k].data() + 4);
    } else {
      failed += (failed.empty() ? "" : ", ") + variables[monitored[k]].name();
      first_error = first_error != 0 ? first_error : result;
    }
  }
  if (!failed.empty()) {
    throw AdsError("Unable to add the notifications of " + failed, first_error);
  }
}

int BeckhoffPLC::Impl::readSymbolVersion(std::uint64_t epoch, std::uint32_t & error)
{
  std::vector<std::uint32_t> errors;
  std::vector<std::vector<std::uint8_t>> responses;
  exchange(
    epoch, 1, ads::kRead, [](ads::FrameWriter & writer, std::size_t) {
      writer.u32(ads::kSymbolVersion);
      writer.u32(0);
      writer.u32(1);
    }, errors, responses, timeout);
  const std::uint8_t * data = nullptr;
  std::size_t size = 0;
  error = readData(errors[0], responses[0], data, size);
  return error == 0 && size == 1 ? data[0] : -1;
}

void BeckhoffPLC::Impl::readInitialValues(std::uint64_t epoch)
{
  // The PLC sends the current value of a monitored variable right after its notification is
  // added; reading them anyway makes latest() return a value as soon as the setup is done.
  if (monitored.empty()) {
    return;
  }
  std::vector<std::uint32_t> errors;
  std::vector<std::vector<std::uint8_t>> responses;
  try {
    exchange(
      epoch, monitored.size(), ads::kRead, [&](ads::FrameWriter & writer, std::size_t k) {
        writer.u32(ads::kSymbolValueByHandle);
        writer.u32(state[monitored[k]]->handle.load(std::memory_order_relaxed));
        writer.u32(static_cast<std::uint32_t>(variables[monitored[k]].size()));
      }, errors, responses, timeout);
  } catch (const AdsError & e) {
    log(LogLevel::Debug, std::string("Unable to read the initial values: ") + e.what());
    return;
  }
  const std::uint64_t time = nowFileTime();
  for (std::size_t k = 0; k < monitored.size(); ++k) {
    const std::uint8_t * data = nullptr;
    std::size_t size = 0;
    const std::uint32_t result = readData(errors[k], responses[k], data, size);
    VariableState & variable_state = *state[monitored[k]];
    if (result != 0 || size != variable_state.latest.size()) {
      log(
        LogLevel::Warn, "Unable to read the initial value of '" + variables[monitored[k]].name() +
        "' (ADS error " + std::to_string(result) + ")");
      continue;
    }
    std::lock_guard<std::mutex> lock(variable_state.mutex);
    if (!variable_state.valid) {  // a notification may have been received already
      std::memcpy(variable_state.latest.data(), data, size);
      variable_state.timestamp = time;
      ++variable_state.sequence;
      variable_state.valid = true;
      variable_state.from_read = true;
      if (variable_state.waiters > 0) {
        variable_state.cv.notify_all();
      }
    }
  }
}

void BeckhoffPLC::Impl::releaseAll(std::uint64_t epoch)
{
  std::vector<std::uint32_t> errors;
  std::vector<std::vector<std::uint8_t>> responses;
  std::vector<std::uint32_t> notifications;
  for (const auto index : monitored) {
    if (state[index]->notification_handle != 0) {
      notifications.push_back(state[index]->notification_handle);
      state[index]->notification_handle = 0;
    }
  }
  {
    std::lock_guard<std::mutex> lock(notifications_mutex);
    by_notification.clear();
  }
  if (!notifications.empty()) {
    exchange(
      epoch, notifications.size(), ads::kDeleteNotification,
      [&](ads::FrameWriter & writer, std::size_t k) {writer.u32(notifications[k]);},
      errors, responses, kReleaseTimeout);
  }
  std::vector<std::uint32_t> handles;
  for (auto & variable_state : state) {
    const std::uint32_t handle = variable_state->handle.exchange(0);
    if (handle != 0) {
      handles.push_back(handle);
    }
  }
  if (handles.empty()) {
    return;
  }
  if (sum_commands.load() && handles.size() > 1) {
    const std::size_t chunk = ads::kMaxSumCommands;
    exchange(
      epoch, (handles.size() + chunk - 1) / chunk, ads::kReadWrite,
      [&](ads::FrameWriter & writer, std::size_t c) {
        const std::size_t begin = c * chunk;
        const std::size_t end = std::min(handles.size(), begin + chunk);
        const auto n = static_cast<std::uint32_t>(end - begin);
        writer.u32(ads::kSumWrite);
        writer.u32(n);
        writer.u32(n * 4);
        writer.u32(n * 16);
        for (std::size_t i = begin; i < end; ++i) {
          writer.u32(ads::kSymbolReleaseHandle);
          writer.u32(0);
          writer.u32(4);
        }
        for (std::size_t i = begin; i < end; ++i) {
          writer.u32(handles[i]);
        }
      }, errors, responses, kReleaseTimeout);
  } else {
    exchange(
      epoch, handles.size(), ads::kWrite, [&](ads::FrameWriter & writer, std::size_t k) {
        writer.u32(ads::kSymbolReleaseHandle);
        writer.u32(0);
        writer.u32(4);
        writer.u32(handles[k]);
      }, errors, responses, kReleaseTimeout);
  }
}

void BeckhoffPLC::Impl::invalidateLatest()
{
  for (const auto index : monitored) {
    std::lock_guard<std::mutex> lock(state[index]->mutex);
    state[index]->valid = false;
  }
}

void BeckhoffPLC::Impl::requestSetup(const std::string & reason)
{
  ready_epoch.store(0);
  bool first = false;
  {
    std::lock_guard<std::mutex> lock(events_mutex);
    first = !setup_requested;
    setup_requested = true;
    next_setup_attempt = Clock::now();
  }
  events_cv.notify_all();
  if (first) {
    log(LogLevel::Info, "Setting the PLC variables up again: " + reason);
  }
}

// -----------------------------------------------------------------------------------------
// Receive thread

void BeckhoffPLC::Impl::onNotification(const ads::NotificationSample & sample)
{
  std::size_t index = 0;
  {
    std::lock_guard<std::mutex> lock(notifications_mutex);
    const auto it = by_notification.find(sample.handle);
    if (it == by_notification.end()) {
      return;  // e.g. a notification deleted since
    }
    index = it->second;
  }
  VariableState & variable_state = *state[index];
  const Variable & variable = variables[index];
  if (sample.size != variable_state.latest.size()) {
    std::size_t suppressed = 0;
    if (notification_errors.allow(suppressed)) {
      log(
        LogLevel::Warn, "Ignoring a notification of '" + variable.name() + "': " +
        std::to_string(sample.size) + " bytes received, " +
        std::to_string(variable_state.latest.size()) + " expected");
    }
    return;
  }
  std::uint64_t sequence = 0;
  bool new_value = true;
  {
    std::lock_guard<std::mutex> lock(variable_state.mutex);
    // the first notification carries the current value, which the setup may have read
    new_value = !variable_state.valid || !variable_state.from_read ||
      std::memcmp(variable_state.latest.data(), sample.data, sample.size) != 0;
    variable_state.from_read = false;
    variable_state.timestamp = sample.timestamp;
    variable_state.valid = true;
    if (new_value) {
      std::memcpy(variable_state.latest.data(), sample.data, sample.size);
      sequence = ++variable_state.sequence;
      if (variable_state.waiters > 0) {
        variable_state.cv.notify_all();
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(callbacks_mutex);
    if (!variable_state.callbacks.empty()) {
      const ConstView data(*variable.type(), sample.data);
      const Timestamp time = toTimestamp(sample.timestamp);
      for (const auto & callback : variable_state.callbacks) {
        try {
          callback.second(variable, data, time);
        } catch (const std::exception & e) {
          log(
            LogLevel::Error, "Error in the update callback of '" + variable.name() + "': " +
            e.what());
        } catch (...) {
          log(
            LogLevel::Error, "Unknown error in the update callback of '" + variable.name() +
            "'");
        }
      }
    }
  }
  if (!new_value) {
    return;
  }
  bool queued = false;
  bool dropped = false;
  {
    std::lock_guard<std::mutex> lock(events_mutex);
    if (queue_events) {
      if (events.size() >= kMaxQueuedEvents) {
        ++dropped_events;
        dropped = true;
      } else {
        Event & event = events.push();
        event.new_observer = false;
        event.variable = index;
        event.sequence = sequence;
        event.data.assign(sample.data, sample.data + sample.size);
        queued = true;
      }
    }
  }
  if (queued) {
    events_cv.notify_all();
  } else if (dropped) {
    std::size_t suppressed = 0;
    if (notification_errors.allow(suppressed)) {
      log(LogLevel::Warn, "The observers are too slow, notifications are dropped");
    }
  }
}

void BeckhoffPLC::Impl::onConnectionChanged(bool connected, std::uint64_t /*epoch*/)
{
  if (!connected) {
    ready_epoch.store(0);
    invalidateLatest();
  } else {
    requestSetup("reconnected to the PLC");
  }
}

void BeckhoffPLC::Impl::logDetachedError(const std::string & what, std::uint32_t error)
{
  std::size_t suppressed = 0;
  if (detached_errors.allow(suppressed)) {
    log(
      LogLevel::Error, AdsError(what, error).what() +
      (suppressed > 0 ? " (" + std::to_string(suppressed) + " similar errors not logged)" : ""));
  }
  if (invalidatesHandles(error)) {
    requestSetup("an asynchronous request failed with ADS error " + std::to_string(error));
  }
}

void BeckhoffPLC::Impl::onDetachedWrite(
  void * context, std::uint32_t error, const std::uint8_t * data, std::size_t size)
{
  auto * variable_state = static_cast<VariableState *>(context);
  const std::uint32_t result = resultOf(error, data, size);
  if (result != 0) {
    variable_state->impl->logDetachedError(
      "Asynchronous write of '" + variable_state->impl->variables[variable_state->index].name() +
      "' failed", result);
  }
}

void BeckhoffPLC::Impl::onDetachedSumWrite(
  void * context, std::uint32_t error, const std::uint8_t * data, std::size_t size)
{
  auto * impl = static_cast<Impl *>(context);
  std::uint32_t result = resultOf(error, data, size);
  if (result == 0 && size >= 8) {
    const std::size_t length = std::min<std::size_t>(get32(data + 4), size - 8);
    for (std::size_t pos = 0; result == 0 && pos + 4 <= length; pos += 4) {
      result = get32(data + 8 + pos);
    }
  }
  if (result != 0) {
    impl->logDetachedError("Asynchronous write of a batch failed", result);
  }
}

void BeckhoffPLC::Impl::onDetachedRead(
  void * context, std::uint32_t error, const std::uint8_t * data, std::size_t size)
{
  // the data is not used, but an error may invalidate the handles
  const std::uint32_t result = resultOf(error, data, size);
  if (invalidatesHandles(result)) {
    static_cast<Impl *>(context)->logDetachedError("Asynchronous read failed", result);
  }
}

// -----------------------------------------------------------------------------------------
// Worker thread: observers, setup after a reconnection, health check

void BeckhoffPLC::Impl::startWorker()
{
  worker = std::thread(&Impl::runWorker, this);
}

void BeckhoffPLC::Impl::runWorker()
{
  auto next_health_check = Clock::now() + health_check_interval;
  std::string last_error;
  Event event;
  std::unique_lock<std::mutex> lock(events_mutex);
  while (!stopping) {
    const auto now = Clock::now();
    // the setup goes first: the observers may be too slow to ever empty the queue
    if (setup_requested && now >= next_setup_attempt && connection->connected()) {
      lock.unlock();
      std::string error;
      try {
        setup();
        log(LogLevel::Info, "PLC variables set up again");
      } catch (const std::exception & e) {
        error = e.what();
        if (error != last_error) {  // the setup is retried every second: log changes only
          log(LogLevel::Warn, "Unable to set the PLC variables up: " + error);
        }
      }
      last_error = error;
      lock.lock();
      if (error.empty() && ready_epoch.load() != 0) {
        setup_requested = false;
      } else if (!error.empty()) {
        next_setup_attempt = Clock::now() + connection_options.reconnect_interval;
      }
      next_health_check = Clock::now() + health_check_interval;
      continue;
    }
    if (health_check_interval.count() > 0 && now >= next_health_check) {
      lock.unlock();
      healthCheck();
      lock.lock();
      next_health_check = Clock::now() + health_check_interval;
      continue;
    }
    if (!events.empty()) {
      events.pop(event);
      NewObserver observer;
      if (event.new_observer) {
        observer = std::move(new_observers.front());
        new_observers.pop_front();
      }
      lock.unlock();
      if (event.new_observer) {
        addObserver(observer);
      } else {
        processEvent(event);
      }
      lock.lock();
      continue;
    }
    auto wake_up = Clock::now() + std::chrono::hours(1);
    if (setup_requested && connection->connected()) {
      wake_up = std::min(wake_up, next_setup_attempt);
    }
    if (health_check_interval.count() > 0) {
      wake_up = std::min(wake_up, next_health_check);
    }
    events_cv.wait_until(lock, wake_up);
  }
}

void BeckhoffPLC::Impl::addObserver(NewObserver & observer)
{
  for (std::size_t k = 0; k < observer.snapshot.size(); ++k) {
    const std::size_t index = observer.snapshot[k].first;
    last_data[index] = std::move(observer.snapshot_data[k]);
    last_sequence[index] = observer.snapshot[k].second;
  }
  observers.push_back(std::move(observer.observer));
  for (const auto index : monitored) {
    if (!last_data[index].empty()) {
      deliver(observers.back(), index, variables[index].type()->decode(last_data[index]));
    }
  }
}

void BeckhoffPLC::Impl::processEvent(Event & event)
{
  if (event.sequence <= last_sequence[event.variable]) {
    return;  // already delivered with the initial values
  }
  last_sequence[event.variable] = event.sequence;
  std::swap(last_data[event.variable], event.data);  // event.data gets the previous buffer
  const Value value = variables[event.variable].type()->decode(last_data[event.variable]);
  for (const auto & observer : observers) {
    deliver(observer, event.variable, value);
  }
}

void BeckhoffPLC::Impl::deliver(
  const Observer & observer, std::size_t variable, const Value & value)
{
  try {
    observer(variables[variable].name(), value);
  } catch (const std::exception & e) {
    log(
      LogLevel::Error, "Error in an observer of '" + variables[variable].name() + "': " +
      e.what());
  } catch (...) {
    log(LogLevel::Error, "Unknown error in an observer of '" + variables[variable].name() + "'");
  }
}

void BeckhoffPLC::Impl::healthCheck()
{
  const std::uint64_t epoch = ready_epoch.load();
  if (epoch == 0 || !connection->connected()) {
    return;
  }
  const std::uint64_t setup = setup_count.load();
  std::uint32_t error = 0;
  int version = -1;
  try {
    version = readSymbolVersion(epoch, error);
  } catch (const AdsError &) {
    return;
  }
  if (error == ads_error::kTimeout || error == ads_error::kNotConnected) {
    return;  // the connection takes care of it
  }
  if (error != 0 && invalidatesHandles(error)) {
    handleRequestError(error, setup);
  } else if (version >= 0 && symbol_version >= 0 && version != symbol_version &&
    setup == setup_count.load())
  {
    requestSetup(
      "the PLC program changed (symbol version " + std::to_string(symbol_version) + " -> " +
      std::to_string(version) + ")");
  }
}

// -----------------------------------------------------------------------------------------
// BeckhoffPLC

BeckhoffPLC::BeckhoffPLC(const nlohmann::ordered_json & params, Logger logger)
: impl_(std::make_unique<Impl>(*this, params, std::move(logger)))
{
  impl_->setup();
  impl_->startWorker();
}

BeckhoffPLC::~BeckhoffPLC() = default;

const Variable & BeckhoffPLC::variable(const std::string & name) const
{
  const Variable * found = findVariable(name);
  if (found == nullptr) {
    throw ConfigError("Variable '" + name + "' is not listed in variables");
  }
  return *found;
}

const Variable * BeckhoffPLC::findVariable(const std::string & name) const
{
  const auto it = impl_->index_by_name.find(name);
  return it == impl_->index_by_name.end() ? nullptr : &impl_->variables[it->second];
}

const std::vector<Variable> & BeckhoffPLC::variables() const
{
  return impl_->variables;
}

PlcTypePtr BeckhoffPLC::variableType(const std::string & name) const
{
  const Variable * found = findVariable(name);
  return found == nullptr ? nullptr : found->type();
}

const TypeRegistry & BeckhoffPLC::types() const
{
  return impl_->types;
}

bool BeckhoffPLC::ready() const
{
  const std::uint64_t epoch = impl_->ready_epoch.load();
  return epoch != 0 && epoch == impl_->connection->epoch() && impl_->connection->connected();
}

std::string BeckhoffPLC::localNetId() const
{
  return impl_->connection->localNetId().toString();
}

void BeckhoffPLC::read(const Variable & variable, Buffer & buffer)
{
  readAsync(variable, buffer).wait();
}

void BeckhoffPLC::write(const Variable & variable, const Buffer & buffer)
{
  writeAsync(variable, buffer).wait();
}

void BeckhoffPLC::execute(Batch & batch)
{
  executeAsync(batch).wait();
}

Pending BeckhoffPLC::readAsync(const Variable & variable, Buffer & buffer)
{
  impl_->checkVariable(variable);
  impl_->checkBuffer(variable, buffer);
  const std::uint64_t setup = impl_->setup_count.load();
  const std::uint64_t epoch = impl_->readyEpoch("Unable to read", variable.name());
  const ads::Ticket ticket = impl_->reserve(Clock::now() + impl_->timeout);
  thread_local std::vector<std::uint8_t> frames;
  frames.clear();
  ads::FrameWriter writer(frames);
  impl_->begin(writer, ads::kRead, ticket);
  writer.u32(ads::kSymbolValueByHandle);
  writer.u32(impl_->state[variable.index_]->handle.load(std::memory_order_relaxed));
  writer.u32(static_cast<std::uint32_t>(variable.size()));
  writer.end();
  impl_->send(frames, epoch, &ticket, 1);

  Pending pending;
  pending.plc_ = this;
  pending.kind_ = Pending::Kind::Read;
  pending.variable_ = &variable;
  pending.buffer_ = &buffer;
  pending.request_ = detail::RequestId{ticket.slot, ticket.invoke_id};
  pending.setup_ = setup;
  return pending;
}

Pending BeckhoffPLC::writeAsync(const Variable & variable, const Buffer & buffer)
{
  impl_->checkVariable(variable);
  impl_->checkBuffer(variable, buffer);
  const std::uint64_t setup = impl_->setup_count.load();
  const std::uint64_t epoch = impl_->readyEpoch("Unable to write", variable.name());
  const ads::Ticket ticket = impl_->reserve(Clock::now() + impl_->timeout);
  thread_local std::vector<std::uint8_t> frames;
  frames.clear();
  ads::FrameWriter writer(frames);
  impl_->begin(writer, ads::kWrite, ticket);
  writer.u32(ads::kSymbolValueByHandle);
  writer.u32(impl_->state[variable.index_]->handle.load(std::memory_order_relaxed));
  writer.u32(static_cast<std::uint32_t>(variable.size()));
  writer.bytes(buffer.data(), buffer.size());
  writer.end();
  impl_->send(frames, epoch, &ticket, 1);

  Pending pending;
  pending.plc_ = this;
  pending.kind_ = Pending::Kind::Write;
  pending.variable_ = &variable;
  pending.request_ = detail::RequestId{ticket.slot, ticket.invoke_id};
  pending.setup_ = setup;
  return pending;
}

Pending BeckhoffPLC::executeAsync(Batch & batch)
{
  batch.checkIdle();
  if (batch.empty()) {
    return Pending();
  }
  if (batch.plc_ != this) {
    throw ConfigError("The batch contains variables of another BeckhoffPLC");
  }
  const std::uint64_t setup = impl_->setup_count.load();
  const std::uint64_t epoch = impl_->readyEpoch("Unable to execute", "batch");
  const bool sum = impl_->sum_commands.load();

  // plan the requests: the writes first, then the reads; several items in a sum command
  batch.requests_.clear();
  for (const bool write : {true, false}) {
    const auto & items = write ? batch.writes_ : batch.reads_;
    if (items.size() == 1 || (!sum && !items.empty())) {
      for (std::size_t i = 0; i < items.size(); ++i) {
        batch.requests_.push_back(Batch::Request{write, false, i, i + 1, {}});
      }
    } else {
      for (std::size_t begin = 0; begin < items.size(); begin += ads::kMaxSumCommands) {
        batch.requests_.push_back(
          Batch::Request{write, true, begin,
            std::min(items.size(), begin + ads::kMaxSumCommands), {}});
      }
    }
  }

  const auto deadline = Clock::now() + impl_->timeout;
  batch.frames_.clear();
  std::size_t reserved = 0;
  try {
    for (auto & request : batch.requests_) {
      const ads::Ticket ticket = impl_->reserve(deadline);
      request.id = detail::RequestId{ticket.slot, ticket.invoke_id};
      ++reserved;
      const auto & items = request.write ? batch.writes_ : batch.reads_;
      ads::FrameWriter writer(batch.frames_);
      if (!request.sum) {
        const Batch::Item & item = items[request.begin];
        impl_->begin(writer, request.write ? ads::kWrite : ads::kRead, ticket);
        writer.u32(ads::kSymbolValueByHandle);
        writer.u32(impl_->state[item.variable->index_]->handle.load(std::memory_order_relaxed));
        writer.u32(static_cast<std::uint32_t>(item.variable->size()));
        if (request.write) {
          writer.bytes(item.source->data(), item.source->size());
        }
      } else {
        const auto n = static_cast<std::uint32_t>(request.end - request.begin);
        std::uint32_t data_size = 0;
        for (std::size_t i = request.begin; i < request.end; ++i) {
          data_size += static_cast<std::uint32_t>(items[i].variable->size());
        }
        impl_->begin(writer, ads::kReadWrite, ticket);
        writer.u32(request.write ? ads::kSumWrite : ads::kSumRead);
        writer.u32(n);
        writer.u32(request.write ? n * 4 : n * 4 + data_size);  // read length
        writer.u32(request.write ? n * 12 + data_size : n * 12);  // write length
        for (std::size_t i = request.begin; i < request.end; ++i) {
          writer.u32(ads::kSymbolValueByHandle);
          writer.u32(
            impl_->state[items[i].variable->index_]->handle.load(std::memory_order_relaxed));
          writer.u32(static_cast<std::uint32_t>(items[i].variable->size()));
        }
        if (request.write) {
          for (std::size_t i = request.begin; i < request.end; ++i) {
            writer.bytes(items[i].source->data(), items[i].source->size());
          }
        }
      }
      writer.end();
    }
  } catch (...) {
    for (std::size_t i = 0; i < reserved; ++i) {
      impl_->connection->release(
        ads::Ticket{batch.requests_[i].id.slot, batch.requests_[i].id.invoke_id});
    }
    throw;
  }

  thread_local std::vector<ads::Ticket> tickets;
  tickets.clear();
  for (const auto & request : batch.requests_) {
    tickets.push_back(ads::Ticket{request.id.slot, request.id.invoke_id});
  }
  impl_->send(batch.frames_, epoch, tickets.data(), tickets.size());

  batch.pending_ = true;
  Pending pending;
  pending.plc_ = this;
  pending.kind_ = Pending::Kind::Batch;
  pending.batch_ = &batch;
  pending.setup_ = setup;
  return pending;
}

bool BeckhoffPLC::latest(const Variable & variable, Buffer & buffer, Timestamp * time) const
{
  impl_->checkMonitored(variable);
  impl_->checkBuffer(variable, buffer);
  const auto & variable_state = *impl_->state[variable.index_];
  std::lock_guard<std::mutex> lock(variable_state.mutex);
  if (!variable_state.valid) {
    return false;
  }
  std::memcpy(buffer.data(), variable_state.latest.data(), variable_state.latest.size());
  if (time != nullptr) {
    *time = toTimestamp(variable_state.timestamp);
  }
  return true;
}

bool BeckhoffPLC::waitForUpdate(
  const Variable & variable, Buffer & buffer, std::uint64_t & sequence,
  std::chrono::nanoseconds timeout, Timestamp * time) const
{
  impl_->checkMonitored(variable);
  impl_->checkBuffer(variable, buffer);
  const auto & variable_state = *impl_->state[variable.index_];
  timeout = std::min<std::chrono::nanoseconds>(timeout, std::chrono::hours(24));
  std::unique_lock<std::mutex> lock(variable_state.mutex);
  ++variable_state.waiters;
  const bool updated = variable_state.cv.wait_for(
    lock, timeout, [&]() {return variable_state.valid && variable_state.sequence > sequence;});
  --variable_state.waiters;
  if (!updated) {
    return false;
  }
  std::memcpy(buffer.data(), variable_state.latest.data(), variable_state.latest.size());
  sequence = variable_state.sequence;
  if (time != nullptr) {
    *time = toTimestamp(variable_state.timestamp);
  }
  return true;
}

BeckhoffPLC::CallbackId BeckhoffPLC::onUpdate(const Variable & variable, UpdateCallback callback)
{
  impl_->checkMonitored(variable);
  if (!callback) {
    throw ConfigError("onUpdate: no callback");
  }
  std::lock_guard<std::mutex> lock(impl_->callbacks_mutex);
  const CallbackId id = impl_->next_callback++;
  impl_->state[variable.index_]->callbacks.emplace_back(id, std::move(callback));
  return id;
}

void BeckhoffPLC::removeCallback(CallbackId id)
{
  std::lock_guard<std::mutex> lock(impl_->callbacks_mutex);
  for (auto & variable_state : impl_->state) {
    auto & callbacks = variable_state->callbacks;
    callbacks.erase(
      std::remove_if(
        callbacks.begin(), callbacks.end(),
        [id](const auto & callback) {return callback.first == id;}),
      callbacks.end());
  }
}

void BeckhoffPLC::addObserver(Observer observer)
{
  if (!observer) {
    throw ConfigError("addObserver: no observer");
  }
  Impl::NewObserver added;
  added.observer = std::move(observer);
  {
    std::lock_guard<std::mutex> lock(impl_->events_mutex);
    if (!impl_->queue_events) {
      // the first observer: start from the latest values, notifications are queued from now on
      for (const auto index : impl_->monitored) {
        const auto & variable_state = *impl_->state[index];
        std::lock_guard<std::mutex> state_lock(variable_state.mutex);
        if (variable_state.valid) {
          added.snapshot.emplace_back(index, variable_state.sequence);
          added.snapshot_data.push_back(variable_state.latest);
        }
      }
      impl_->queue_events = true;
    }
    impl_->new_observers.push_back(std::move(added));
    impl_->events.push().new_observer = true;
  }
  impl_->events_cv.notify_all();
}

void BeckhoffPLC::setVariable(const std::string & name, const Value & value)
{
  const Variable & target = variable(name);
  Buffer buffer(target.type());
  target.type()->encode(value, buffer.data(), name);
  write(target, buffer);
}

Value BeckhoffPLC::getVariable(const std::string & name)
{
  const Variable & source = variable(name);
  Buffer buffer(source.type());
  read(source, buffer);
  return buffer.value();
}

void BeckhoffPLC::createRoute()
{
  createRoute(impl_->params);
}

void BeckhoffPLC::createRoute(const nlohmann::ordered_json & params)
{
  const std::string net_id = requireString(params, "net_id");
  const std::string client_net_id = requireString(params, "client_net_id");
  const ads::AmsNetId client = parseNetId(client_net_id, "client_net_id");
  parseNetId(net_id, "net_id");
  std::string host = params.contains("host") ? requireString(params, "host") : "";
  if (host.empty()) {
    host = netId2ip(net_id);
  }
  const std::uint32_t error = ads::addRoute(
    host, client, netId2ip(client_net_id), kRouteName, requireString(params, "username"),
    requireString(params, "password"));
  if (error != 0) {
    throw AdsError("Unable to add a route to " + client_net_id + " on the PLC " + net_id, error);
  }
}

std::string BeckhoffPLC::netId2ip(const std::string & net_id)
{
  std::string ip = net_id;
  for (int i = 0; i < 2; ++i) {
    const auto dot = ip.rfind('.');
    if (dot != std::string::npos) {
      ip = ip.substr(0, dot);
    }
  }
  return ip;
}

// -----------------------------------------------------------------------------------------
// Pending

Pending::Pending(Pending && other) noexcept
{
  *this = std::move(other);
}

Pending & Pending::operator=(Pending && other) noexcept
{
  if (this != &other) {
    detach();
    plc_ = other.plc_;
    kind_ = other.kind_;
    variable_ = other.variable_;
    buffer_ = other.buffer_;
    batch_ = other.batch_;
    request_ = other.request_;
    setup_ = other.setup_;
    other.plc_ = nullptr;
  }
  return *this;
}

Pending::~Pending()
{
  detach();
}

void Pending::detach() noexcept
{
  if (plc_ == nullptr) {
    return;
  }
  auto & impl = *plc_->impl_;
  if (kind_ == Kind::Batch) {
    for (const auto & request : batch_->requests_) {
      if (!request.write) {
        impl.connection->detach(
          ads::Ticket{request.id.slot, request.id.invoke_id}, &BeckhoffPLC::Impl::onDetachedRead,
          &impl);
      } else if (request.sum) {
        impl.connection->detach(
          ads::Ticket{request.id.slot, request.id.invoke_id},
          &BeckhoffPLC::Impl::onDetachedSumWrite, &impl);
      } else {
        impl.connection->detach(
          ads::Ticket{request.id.slot, request.id.invoke_id}, &BeckhoffPLC::Impl::onDetachedWrite,
          impl.state[batch_->writes_[request.begin].variable->index_].get());
      }
    }
    batch_->pending_ = false;
  } else if (kind_ == Kind::Write) {
    impl.connection->detach(
      ads::Ticket{request_.slot, request_.invoke_id}, &BeckhoffPLC::Impl::onDetachedWrite,
      impl.state[variable_->index_].get());
  } else {
    impl.connection->detach(
      ads::Ticket{request_.slot, request_.invoke_id}, &BeckhoffPLC::Impl::onDetachedRead, &impl);
  }
  plc_ = nullptr;
}

bool Pending::ready() const
{
  if (plc_ == nullptr) {
    return true;
  }
  auto & connection = *plc_->impl_->connection;
  if (kind_ == Kind::Batch) {
    return std::all_of(
      batch_->requests_.begin(), batch_->requests_.end(), [&connection](const auto & request) {
        return connection.completed(ads::Ticket{request.id.slot, request.id.invoke_id});
      });
  }
  return connection.completed(ads::Ticket{request_.slot, request_.invoke_id});
}

void Pending::wait()
{
  if (plc_ == nullptr) {
    return;
  }
  BeckhoffPLC::Impl & impl = *plc_->impl_;
  ads::Connection & connection = *impl.connection;
  plc_ = nullptr;  // waited for: nothing to detach anymore

  if (kind_ != Kind::Batch) {
    const ads::Ticket ticket{request_.slot, request_.invoke_id};
    const std::uint32_t error = connection.wait(ticket);
    const auto & response = connection.response(ticket);
    std::uint32_t result = 0;
    if (kind_ == Kind::Write) {
      result = resultOf(error, response);
    } else {
      const std::uint8_t * data = nullptr;
      std::size_t size = 0;
      result = readData(error, response, data, size);
      if (result == 0 && size != buffer_->size()) {
        result = ads_error::kInvalidSize;
      }
      if (result == 0) {
        std::memcpy(buffer_->data(), data, size);
      }
    }
    connection.release(ticket);
    if (result != 0) {
      impl.handleRequestError(result, setup_);
      throw AdsError(
              std::string(kind_ == Kind::Write ? "Unable to write '" : "Unable to read '") +
              variable_->name() + "'", result);
    }
    return;
  }

  Batch & batch = *batch_;
  std::uint32_t first_error = 0;
  std::string failed;
  const auto fail = [&](const Variable & variable, bool write, std::uint32_t result) {
    if (first_error == 0) {
      first_error = result;
      failed = std::string(write ? "Unable to write '" : "Unable to read '") +
        variable.name() + "'";
    }
  };
  for (const auto & request : batch.requests_) {
    const ads::Ticket ticket{request.id.slot, request.id.invoke_id};
    const std::uint32_t error = connection.wait(ticket);
    const auto & response = connection.response(ticket);
    const auto & items = request.write ? batch.writes_ : batch.reads_;
    if (!request.sum) {
      const Batch::Item & item = items[request.begin];
      std::uint32_t result = 0;
      if (request.write) {
        result = resultOf(error, response);
      } else {
        const std::uint8_t * data = nullptr;
        std::size_t size = 0;
        result = readData(error, response, data, size);
        if (result == 0 && size != item.destination->size()) {
          result = ads_error::kInvalidSize;
        }
        if (result == 0) {
          std::memcpy(item.destination->data(), data, size);
        }
      }
      if (result != 0) {
        fail(*item.variable, request.write, result);
      }
    } else {
      const std::size_t n = request.end - request.begin;
      const std::uint8_t * data = nullptr;
      std::size_t size = 0;
      std::uint32_t result = readData(error, response, data, size);
      if (result == 0 && size < n * 4) {
        result = ads_error::kInvalidResponse;
      }
      if (result != 0) {
        fail(*items[request.begin].variable, request.write, result);
      } else {
        std::size_t pos = n * 4;
        for (std::size_t i = request.begin; i < request.end; ++i) {
          const std::uint32_t sub_result = get32(data + (i - request.begin) * 4);
          if (request.write) {
            if (sub_result != 0) {
              fail(*items[i].variable, true, sub_result);
            }
            continue;
          }
          const std::size_t item_size = items[i].destination->size();
          if (pos + item_size > size) {
            fail(*items[i].variable, false, ads_error::kInvalidResponse);
            break;
          }
          if (sub_result == 0) {
            std::memcpy(items[i].destination->data(), data + pos, item_size);
          } else {
            fail(*items[i].variable, false, sub_result);
          }
          pos += item_size;
        }
      }
    }
    connection.release(ticket);
  }
  batch.pending_ = false;
  if (first_error != 0) {
    impl.handleRequestError(first_error, setup_);
    throw AdsError(failed, first_error);
  }
}

// -----------------------------------------------------------------------------------------
// Batch

Batch & Batch::write(const Variable & variable, const Buffer & buffer)
{
  checkIdle();
  if (plc_ != nullptr && variable.owner_ != plc_) {
    throw ConfigError("Variable '" + variable.name() + "' belongs to another BeckhoffPLC");
  }
  if (!buffer.typePtr()) {
    throw ValueError("Empty buffer used for '" + variable.name() + "'");
  }
  detail::checkSameType(buffer.type(), *variable.type());
  plc_ = variable.owner_;
  writes_.push_back(Item{&variable, &buffer, nullptr});
  return *this;
}

Batch & Batch::read(const Variable & variable, Buffer & buffer)
{
  checkIdle();
  if (plc_ != nullptr && variable.owner_ != plc_) {
    throw ConfigError("Variable '" + variable.name() + "' belongs to another BeckhoffPLC");
  }
  if (!buffer.typePtr()) {
    throw ValueError("Empty buffer used for '" + variable.name() + "'");
  }
  detail::checkSameType(buffer.type(), *variable.type());
  plc_ = variable.owner_;
  reads_.push_back(Item{&variable, nullptr, &buffer});
  return *this;
}

void Batch::clear()
{
  checkIdle();
  writes_.clear();
  reads_.clear();
  plc_ = nullptr;
}

void Batch::checkIdle() const
{
  if (pending_) {
    throw ConfigError("The batch is being executed (wait for its Pending first)");
  }
}

}  // namespace beckhoff_ros
