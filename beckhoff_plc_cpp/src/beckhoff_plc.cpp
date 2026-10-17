#include "beckhoff_plc/beckhoff_plc.hpp"

#include <algorithm>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <unordered_set>

#ifndef _WIN32
#include <signal.h>
#endif

// Beckhoff ADS library (https://github.com/Beckhoff/ADS)
#include "AdsLib.h"
#include "Log.h"

namespace beckhoff {

namespace detail {

namespace {

constexpr std::uint64_t kUnixEpochFileTime = 116444736000000000ULL;  // 1970-01-01 in 100 ns since 1601
constexpr std::size_t kMaxSumCommands = 500;  // Beckhoff's recommended maximum per sum command

using FileTimeDuration = std::chrono::duration<std::int64_t, std::ratio<1, 10000000>>;

std::uint64_t nowFileTime() {
  const auto sinceEpoch = std::chrono::system_clock::now().time_since_epoch();
  return kUnixEpochFileTime +
         static_cast<std::uint64_t>(std::chrono::duration_cast<FileTimeDuration>(sinceEpoch).count());
}

std::uint32_t ticks(std::chrono::nanoseconds duration) {
  return static_cast<std::uint32_t>(std::clamp<std::int64_t>(duration.count() / 100, 0, 0xFFFFFFFF));
}

std::uint32_t size32(std::size_t size) { return static_cast<std::uint32_t>(size); }

std::uint32_t loadLe32(const std::uint8_t* data) { return load<std::uint32_t>(data); }
void storeLe32(std::uint8_t* data, std::uint32_t value) { store(data, value); }

/// The handle does not refer to the symbol anymore (online change, new program).
bool isHandleError(long code) {
  return code == ADSERR_DEVICE_SYMBOLNOTFOUND || code == ADSERR_DEVICE_SYMBOLVERSIONINVALID ||
         code == ADSERR_DEVICE_SYMBOLNOTACTIVE;
}

bool isNotSupported(long code) { return code == ADSERR_DEVICE_SRVNOTSUPP || code == ADSERR_DEVICE_INVALIDGRP; }

/// The PLC cannot be reached: further requests would fail the same way.
bool isConnectionError(long code) {
  return code == -1 || code == ADSERR_CLIENT_SYNCTIMEOUT || code == GLOBALERR_MISSING_ROUTE ||
         code == GLOBALERR_TARGET_PORT || code == ADSERR_CLIENT_PORTNOTOPEN;
}

void defaultErrorHandler(const std::string& message) { std::cerr << "[beckhoff_plc] " << message << std::endl; }

/// The ADS library writes to its sockets without MSG_NOSIGNAL: writing to a
/// connection the PLC closed (e.g. while it restarts) raises SIGPIPE, which
/// terminates the process by default. Python (pyads) ignores SIGPIPE; do the
/// same, unless the application installed its own handler.
void ignoreSigpipe() {
#ifndef _WIN32
  static std::once_flag once;
  std::call_once(once, [] {
    struct sigaction current {};
    if (sigaction(SIGPIPE, nullptr, &current) == 0 && !(current.sa_flags & SA_SIGINFO) &&
        current.sa_handler == SIG_DFL) {
      struct sigaction ignore {};
      ignore.sa_handler = SIG_IGN;
      sigemptyset(&ignore.sa_mask);
      sigaction(SIGPIPE, &ignore, nullptr);
    }
  });
#endif
}

template <class E>
[[noreturn]] void rethrowFor(const E& error, const std::string& name) {
  throw E("'" + name + "': " + error.what());
}

}  // namespace

struct ObserverEntry {
  ObserverId id = 0;
  std::shared_ptr<const Callback> callback;
  const Variable* variable = nullptr;  // nullptr: every monitored variable
};

/// Routes the notifications of the ADS library (which only carry a 32-bit
/// user handle) to the PlcCore and Variable they belong to.
class NotificationRegistry {
 public:
  static NotificationRegistry& instance() {
    // Never destroyed: ADS threads may still deliver notifications while
    // static objects are destroyed.
    static NotificationRegistry* registry = new NotificationRegistry();
    return *registry;
  }

  std::uint32_t add(PlcCore* core, Variable* variable) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::uint32_t id;
    do {
      id = next_++;
    } while (id == 0 || targets_.count(id));
    targets_[id] = Target{core, variable};
    return id;
  }

  /// Stops routing notifications to `core`. When this returns, no callback of
  /// `core` is running anymore (unless called from one of them).
  void removeAll(const PlcCore* core) {
    std::unique_lock<std::mutex> lock(mutex_);
    for (auto it = targets_.begin(); it != targets_.end();) {
      it = it->second.core == core ? targets_.erase(it) : std::next(it);
    }
    if (dispatching_ == core) return;
    idle_.wait(lock, [&] { return busy_.count(core) == 0; });
  }

  static bool isDispatching(const PlcCore* core) { return dispatching_ == core; }

  static void onNotification(const AmsAddr*, const AdsNotificationHeader* header, std::uint32_t id) {
    instance().dispatch(header, id);
  }

 private:
  struct Target {
    PlcCore* core = nullptr;
    Variable* variable = nullptr;
  };

  void dispatch(const AdsNotificationHeader* header, std::uint32_t id);

  std::mutex mutex_;
  std::condition_variable idle_;
  std::unordered_map<std::uint32_t, Target> targets_;
  std::unordered_map<const PlcCore*, int> busy_;  // callbacks running, per core
  std::uint32_t next_ = 1;
  static thread_local const PlcCore* dispatching_;
};

thread_local const PlcCore* NotificationRegistry::dispatching_ = nullptr;

/// Implementation of BeckhoffPLC.
class PlcCore {
 public:
  explicit PlcCore(Config cfg);
  ~PlcCore();
  PlcCore(const PlcCore&) = delete;
  PlcCore& operator=(const PlcCore&) = delete;

  const Config config;
  const TypeRegistry types;
  std::vector<std::unique_ptr<Variable>> variables;
  std::vector<Variable*> monitored;  // in "variables_to_monitor" order

  Variable& variable(const std::string& name) const {
    const auto it = byName_.find(name);
    if (it == byName_.end()) throw KeyError("variable '" + name + "' is not defined in 'variables'");
    return *it->second;
  }
  bool hasVariable(const std::string& name) const { return byName_.count(name) != 0; }

  // ---- requests (thread-safe; the standalone AdsLib supports a single
  // pending request per AMS port, so they are serialized)
  void read(const Variable& v, void* data, std::size_t size);
  void write(const Variable& v, const void* data, std::size_t size);
  void readMany(const std::vector<const Variable*>& vars, const std::vector<std::uint8_t*>& outputs);
  void writeMany(const std::vector<const Variable*>& vars, const std::vector<const std::uint8_t*>& inputs);
  AdsState readState();
  void reconnect();

  // ---- notifications
  void dispatch(const Variable& v, const std::uint8_t* data, std::uint32_t size, std::uint64_t timestamp) noexcept;
  ObserverId addObserver(Callback callback, const Variable* variable, bool initialValues);
  void removeObserver(ObserverId id);
  void deliverCurrentValues(std::optional<ObserverId> only);
  void setErrorHandler(std::function<void(const std::string&)> handler);
  void reportError(const std::string& message) noexcept;

 private:
  static Config validated(Config config) {
    config.validate();
    return config;
  }
  void connect();
  void disconnect() noexcept;
  void acquireHandle(const Variable& v);
  void refreshHandle(const Variable& v);
  std::string checkSymbol(const Variable& v);
  void addNotification(Variable& v);
  void checkConnected() const;
  void checkWritable(const Variable& v) const;
  std::shared_ptr<const std::vector<ObserverEntry>> observers() {
    std::lock_guard<std::mutex> lock(observersMutex_);
    return observers_;
  }
  void invoke(const ObserverEntry& observer, const Notification& notification) noexcept;

  std::unordered_map<std::string, Variable*> byName_;

  std::recursive_mutex requestMutex_;  // ADS requests and connection state
  long port_ = 0;
  AmsAddr addr_{};
  bool routeAdded_ = false;
  bool connected_ = false;
  bool sumCommands_ = true;
  std::vector<std::uint8_t> symbolBuffer_;
  std::vector<std::uint8_t> sumRequest_;
  std::vector<std::uint8_t> sumResponse_;

  std::recursive_mutex callbackMutex_;  // callbacks never run concurrently
  std::mutex observersMutex_;
  std::shared_ptr<const std::vector<ObserverEntry>> observers_;
  ObserverId nextObserverId_ = 1;
  std::function<void(const std::string&)> errorHandler_ = defaultErrorHandler;
};

void NotificationRegistry::dispatch(const AdsNotificationHeader* header, std::uint32_t id) {
  Target target;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = targets_.find(id);
    if (it == targets_.end()) return;
    target = it->second;
    ++busy_[target.core];
  }
  const PlcCore* previous = dispatching_;
  dispatching_ = target.core;
  target.core->dispatch(*target.variable, reinterpret_cast<const std::uint8_t*>(header + 1), header->cbSampleSize,
                        header->nTimeStamp);
  dispatching_ = previous;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = busy_.find(target.core);
    if (--it->second == 0) {
      busy_.erase(it);
      idle_.notify_all();
    }
  }
}

// ---------------------------------------------------------------------------
// PlcCore: construction and connection

PlcCore::PlcCore(Config cfg)
    : config(validated(std::move(cfg))),
      types(config.createTypeRegistry()),
      observers_(std::make_shared<const std::vector<ObserverEntry>>()) {
  std::unordered_set<std::string> writable(config.variablesToWrite.begin(), config.variablesToWrite.end());
  std::unordered_map<std::string, NotificationSettings> monitoredSettings;
  for (const MonitoredVariable& m : config.variablesToMonitor) {
    monitoredSettings[m.name] = m.notification.value_or(config.notification);
  }
  for (const VariableDefinition& definition : config.variables) {
    const auto settings = monitoredSettings.find(definition.name);
    const bool isMonitored = settings != monitoredSettings.end();
    variables.emplace_back(new Variable(*this, definition.name, types.parse(definition.type),
                                        writable.count(definition.name) != 0, isMonitored,
                                        isMonitored ? settings->second : config.notification));
    byName_[definition.name] = variables.back().get();
  }
  for (const MonitoredVariable& m : config.variablesToMonitor) monitored.push_back(byName_.at(m.name));

  ignoreSigpipe();
  try {
    std::lock_guard<std::recursive_mutex> lock(requestMutex_);
    for (Variable* v : monitored) v->notificationId_ = NotificationRegistry::instance().add(this, v);
    connect();
  } catch (...) {
    NotificationRegistry::instance().removeAll(this);
    throw;
  }
}

PlcCore::~PlcCore() {
  // First make sure no callback runs (they may use the request mutex), then
  // release everything on the PLC.
  NotificationRegistry::instance().removeAll(this);
  disconnect();
}

void PlcCore::connect() {
  std::lock_guard<std::recursive_mutex> lock(requestMutex_);
  try {
    if (config.createRoute) BeckhoffPLC::createRoute(config);
    if (!config.clientNetId.empty()) bhf::ads::SetLocalAddress(AmsNetId(config.clientNetId));

    const std::string ip = config.ipAddress.empty() ? netIdToIp(config.netId) : config.ipAddress;
    const AmsNetId netId(config.netId);
    const long status = bhf::ads::AddLocalRoute(netId, ip.c_str());
    if (status) throw AdsError(status, "cannot connect to the PLC at " + ip + " (AMS Net ID " + config.netId + ")");
    routeAdded_ = true;

    port_ = AdsPortOpenEx();
    if (!port_) throw AdsError(ADSERR_CLIENT_PORTNOTOPEN, "cannot open an AMS port (too many open ports?)");
    const long timeoutStatus = AdsSyncSetTimeoutEx(port_, static_cast<std::uint32_t>(config.timeout.count()));
    if (timeoutStatus) throw AdsError(timeoutStatus, "setting the ADS timeout");
    addr_ = AmsAddr{netId, config.port};
    connected_ = true;

    std::vector<std::string> problems;
    for (const auto& v : variables) {
      acquireHandle(*v);
      if (config.verifySizes) {
        std::string problem = checkSymbol(*v);
        if (!problem.empty()) problems.push_back(std::move(problem));
      }
    }
    if (!problems.empty()) {
      std::string message = "the configuration does not match the PLC:";
      for (const std::string& problem : problems) message += "\n  - " + problem;
      throw ConfigError(message);
    }
    for (Variable* v : monitored) addNotification(*v);
  } catch (...) {
    disconnect();
    throw;
  }
}

void PlcCore::disconnect() noexcept {
  std::lock_guard<std::recursive_mutex> lock(requestMutex_);
  // Best effort: stop sending requests as soon as the PLC does not answer.
  bool alive = connected_;
  const auto request = [&](long status) {
    if (isConnectionError(status)) alive = false;
  };
  for (Variable* v : monitored) {
    if (v->notificationHandle_ && alive) {
      request(AdsSyncDelDeviceNotificationReqEx(port_, &addr_, v->notificationHandle_));
    }
    v->notificationHandle_ = 0;
  }
  for (const auto& v : variables) {
    if (v->handle_ && alive) {
      const std::uint32_t handle = v->handle_;
      request(AdsSyncWriteReqEx(port_, &addr_, ADSIGRP_SYM_RELEASEHND, 0, sizeof(handle), &handle));
    }
    v->handle_ = 0;
  }
  if (port_) {
    // Closing the port deletes the notifications left; do not wait for a dead PLC.
    if (!alive) AdsSyncSetTimeoutEx(port_, 50);
    AdsPortCloseEx(port_);
    port_ = 0;
  }
  if (routeAdded_) {
    bhf::ads::DelLocalRoute(AmsNetId(config.netId));
    routeAdded_ = false;
  }
  connected_ = false;
}

void PlcCore::reconnect() {
  if (NotificationRegistry::isDispatching(this)) {
    throw Error("reconnect() cannot be called from a notification callback");
  }
  // Stop the notifications first, without holding the request mutex (a
  // running callback may be waiting for it).
  NotificationRegistry::instance().removeAll(this);
  std::lock_guard<std::recursive_mutex> lock(requestMutex_);
  disconnect();
  for (Variable* v : monitored) v->notificationId_ = NotificationRegistry::instance().add(this, v);
  connect();
}

void PlcCore::acquireHandle(const Variable& v) {
  std::uint32_t handle = 0;
  std::uint32_t bytesRead = 0;
  const long status = AdsSyncReadWriteReqEx2(port_, &addr_, ADSIGRP_SYM_HNDBYNAME, 0, sizeof(handle), &handle,
                                             size32(v.name_.size()), v.name_.data(), &bytesRead);
  if (status == ADSERR_DEVICE_SYMBOLNOTFOUND) throw AdsError(status, "variable '" + v.name_ + "' not found in the PLC");
  if (isConnectionError(status)) {
    throw AdsError(status, "no answer from the PLC at " + config.netId + ":" + std::to_string(config.port) +
                               " (is the PLC running? does it have a route to this computer? see create_route)");
  }
  if (status) throw AdsError(status, "getting a handle for '" + v.name_ + "'");
  if (bytesRead != sizeof(handle)) {
    throw AdsError(ADSERR_CLIENT_SYNCRESINVALID, "getting a handle for '" + v.name_ + "': invalid response");
  }
  v.handle_ = handle;
}

void PlcCore::refreshHandle(const Variable& v) {
  const std::uint32_t old = v.handle_;
  AdsSyncWriteReqEx(port_, &addr_, ADSIGRP_SYM_RELEASEHND, 0, sizeof(old), &old);
  v.handle_ = 0;
  acquireHandle(v);
}

/// Compares the configured size with the PLC symbol. Returns a description of
/// the mismatch, if any.
std::string PlcCore::checkSymbol(const Variable& v) {
  symbolBuffer_.resize(0xFFFF);
  std::uint32_t bytesRead = 0;
  const long status =
      AdsSyncReadWriteReqEx2(port_, &addr_, ADSIGRP_SYM_INFOBYNAMEEX, 0, size32(symbolBuffer_.size()),
                             symbolBuffer_.data(), size32(v.name_.size()), v.name_.data(), &bytesRead);
  if (isNotSupported(status)) return {};  // nothing to compare with
  if (status) throw AdsError(status, "reading the symbol information of '" + v.name_ + "'");
  AdsSymbolEntry entry;
  if (bytesRead < sizeof(entry)) {
    throw AdsError(ADSERR_CLIENT_SYNCRESINVALID, "reading the symbol information of '" + v.name_ + "'");
  }
  std::memcpy(&entry, symbolBuffer_.data(), sizeof(entry));
  if (entry.size == 0 || entry.size == v.size()) return {};  // 0: bit symbols, nothing to compare

  std::string plcType;
  const std::size_t typeStart = sizeof(entry) + entry.nameLength + 1;
  if (typeStart + entry.typeLength <= bytesRead) {
    plcType.assign(reinterpret_cast<const char*>(symbolBuffer_.data()) + typeStart, entry.typeLength);
  }
  std::string message = "'" + v.name_ + "' is " + v.type().name() + " (" + std::to_string(v.size()) +
                        " bytes) in the configuration but " + (plcType.empty() ? "" : plcType + " ") + "(" +
                        std::to_string(entry.size) + " bytes) in the PLC";
  const Type* type = &v.type();
  while (type->isArray()) type = &type->elementType();
  if (type->isStruct()) {
    std::string layout = type->layout();
    for (std::size_t pos = 0; (pos = layout.find('\n', pos)) != std::string::npos; pos += 5) {
      layout.replace(pos, 1, "\n    ");
    }
    message += ". Check the members and pack mode of " + type->name() + ":\n    " + layout;
  }
  return message;
}

void PlcCore::addNotification(Variable& v) {
  AdsNotificationAttrib attrib{};
  attrib.cbLength = size32(v.size());
  attrib.nTransMode = v.notification_.mode == NotificationSettings::Mode::Cyclic ? ADSTRANS_SERVERCYCLE
                                                                                   : ADSTRANS_SERVERONCHA;
  attrib.nMaxDelay = ticks(v.notification_.maxDelay);
  attrib.nCycleTime = ticks(v.notification_.cycleTime);
  std::uint32_t handle = 0;
  const long status = AdsSyncAddDeviceNotificationReqEx(port_, &addr_, ADSIGRP_SYM_VALBYHND, v.handle_, &attrib,
                                                        &NotificationRegistry::onNotification, v.notificationId_,
                                                        &handle);
  if (status) throw AdsError(status, "adding the notification of '" + v.name_ + "'");
  v.notificationHandle_ = handle;
}

void PlcCore::checkConnected() const {
  if (!connected_) throw AdsError(ADSERR_CLIENT_PORTNOTOPEN, "not connected to the PLC (call reconnect())");
}

void PlcCore::checkWritable(const Variable& v) const { v.checkWritable(); }

// ---------------------------------------------------------------------------
// PlcCore: requests

void PlcCore::read(const Variable& v, void* data, std::size_t size) {
  std::lock_guard<std::recursive_mutex> lock(requestMutex_);
  checkConnected();
  std::uint32_t bytesRead = 0;
  long status = AdsSyncReadReqEx2(port_, &addr_, ADSIGRP_SYM_VALBYHND, v.handle_, size32(size), data, &bytesRead);
  if (isHandleError(status)) {
    refreshHandle(v);
    status = AdsSyncReadReqEx2(port_, &addr_, ADSIGRP_SYM_VALBYHND, v.handle_, size32(size), data, &bytesRead);
  }
  if (status) throw AdsError(status, "reading '" + v.name_ + "'");
  if (bytesRead != size) {
    throw AdsError(ADSERR_DEVICE_INVALIDSIZE, "reading '" + v.name_ + "': received " + std::to_string(bytesRead) +
                                                  " bytes instead of " + std::to_string(size));
  }
}

void PlcCore::write(const Variable& v, const void* data, std::size_t size) {
  checkWritable(v);
  std::lock_guard<std::recursive_mutex> lock(requestMutex_);
  checkConnected();
  long status = AdsSyncWriteReqEx(port_, &addr_, ADSIGRP_SYM_VALBYHND, v.handle_, size32(size), data);
  if (isHandleError(status)) {
    refreshHandle(v);
    status = AdsSyncWriteReqEx(port_, &addr_, ADSIGRP_SYM_VALBYHND, v.handle_, size32(size), data);
  }
  if (status) throw AdsError(status, "writing '" + v.name_ + "'");
}

void PlcCore::readMany(const std::vector<const Variable*>& vars, const std::vector<std::uint8_t*>& outputs) {
  std::lock_guard<std::recursive_mutex> lock(requestMutex_);
  checkConnected();
  for (std::size_t start = 0; start < vars.size(); start += kMaxSumCommands) {
    const std::size_t count = std::min(kMaxSumCommands, vars.size() - start);
    if (count == 1 || !sumCommands_) {
      for (std::size_t i = start; i < start + count; ++i) read(*vars[i], outputs[i], vars[i]->size());
      continue;
    }
    // Request: {index group, index offset, length} per variable.
    // Response: one result code per variable, then the data of every variable.
    sumRequest_.resize(12 * count);
    std::size_t responseSize = 4 * count;
    for (std::size_t i = 0; i < count; ++i) {
      const Variable& v = *vars[start + i];
      storeLe32(&sumRequest_[12 * i], ADSIGRP_SYM_VALBYHND);
      storeLe32(&sumRequest_[12 * i + 4], v.handle_);
      storeLe32(&sumRequest_[12 * i + 8], size32(v.size()));
      responseSize += v.size();
    }
    sumResponse_.resize(responseSize);
    std::uint32_t bytesRead = 0;
    const long status = AdsSyncReadWriteReqEx2(port_, &addr_, ADSIGRP_SUMUP_READ, size32(count),
                                               size32(responseSize), sumResponse_.data(), size32(sumRequest_.size()),
                                               sumRequest_.data(), &bytesRead);
    if (isNotSupported(status)) {
      sumCommands_ = false;  // very old runtime: one request per variable
      for (std::size_t i = start; i < start + count; ++i) read(*vars[i], outputs[i], vars[i]->size());
      continue;
    }
    if (status) throw AdsError(status, "reading " + std::to_string(count) + " variables");
    if (bytesRead != responseSize) {
      throw AdsError(ADSERR_DEVICE_INVALIDSIZE, "reading " + std::to_string(count) + " variables: received " +
                                                    std::to_string(bytesRead) + " bytes instead of " +
                                                    std::to_string(responseSize));
    }
    std::size_t offset = 4 * count;
    for (std::size_t i = 0; i < count; ++i) {
      const Variable& v = *vars[start + i];
      const long result = static_cast<long>(loadLe32(&sumResponse_[4 * i]));
      if (result == 0) {
        std::memcpy(outputs[start + i], &sumResponse_[offset], v.size());
      } else if (isHandleError(result)) {
        read(v, outputs[start + i], v.size());  // gets a new handle
      } else {
        throw AdsError(result, "reading '" + v.name_ + "'");
      }
      offset += v.size();
    }
  }
}

void PlcCore::writeMany(const std::vector<const Variable*>& vars, const std::vector<const std::uint8_t*>& inputs) {
  for (const Variable* v : vars) checkWritable(*v);
  std::lock_guard<std::recursive_mutex> lock(requestMutex_);
  checkConnected();
  for (std::size_t start = 0; start < vars.size(); start += kMaxSumCommands) {
    const std::size_t count = std::min(kMaxSumCommands, vars.size() - start);
    if (count == 1 || !sumCommands_) {
      for (std::size_t i = start; i < start + count; ++i) write(*vars[i], inputs[i], vars[i]->size());
      continue;
    }
    // Request: {index group, index offset, length} per variable, then the data.
    // Response: one result code per variable.
    std::size_t requestSize = 12 * count;
    for (std::size_t i = 0; i < count; ++i) requestSize += vars[start + i]->size();
    sumRequest_.resize(requestSize);
    std::size_t offset = 12 * count;
    for (std::size_t i = 0; i < count; ++i) {
      const Variable& v = *vars[start + i];
      storeLe32(&sumRequest_[12 * i], ADSIGRP_SYM_VALBYHND);
      storeLe32(&sumRequest_[12 * i + 4], v.handle_);
      storeLe32(&sumRequest_[12 * i + 8], size32(v.size()));
      std::memcpy(&sumRequest_[offset], inputs[start + i], v.size());
      offset += v.size();
    }
    sumResponse_.resize(4 * count);
    std::uint32_t bytesRead = 0;
    const long status = AdsSyncReadWriteReqEx2(port_, &addr_, ADSIGRP_SUMUP_WRITE, size32(count),
                                               size32(sumResponse_.size()), sumResponse_.data(), size32(requestSize),
                                               sumRequest_.data(), &bytesRead);
    if (isNotSupported(status)) {
      sumCommands_ = false;
      for (std::size_t i = start; i < start + count; ++i) write(*vars[i], inputs[i], vars[i]->size());
      continue;
    }
    if (status) throw AdsError(status, "writing " + std::to_string(count) + " variables");
    if (bytesRead != sumResponse_.size()) {
      throw AdsError(ADSERR_DEVICE_INVALIDSIZE, "writing " + std::to_string(count) + " variables: invalid response");
    }
    for (std::size_t i = 0; i < count; ++i) {
      const Variable& v = *vars[start + i];
      const long result = static_cast<long>(loadLe32(&sumResponse_[4 * i]));
      if (isHandleError(result)) {
        write(v, inputs[start + i], v.size());  // gets a new handle
      } else if (result) {
        throw AdsError(result, "writing '" + v.name_ + "'");
      }
    }
  }
}

AdsState PlcCore::readState() {
  std::lock_guard<std::recursive_mutex> lock(requestMutex_);
  checkConnected();
  std::uint16_t adsState = 0;
  std::uint16_t deviceState = 0;
  const long status = AdsSyncReadStateReqEx(port_, &addr_, &adsState, &deviceState);
  if (status) throw AdsError(status, "reading the PLC state");
  return static_cast<AdsState>(adsState);
}

// ---------------------------------------------------------------------------
// PlcCore: notifications

void PlcCore::dispatch(const Variable& v, const std::uint8_t* data, std::uint32_t size,
                       std::uint64_t timestamp) noexcept {
  if (size != v.size()) {
    reportError("notification of '" + v.name_ + "' has " + std::to_string(size) + " bytes instead of " +
                std::to_string(v.size()));
    return;
  }
  std::lock_guard<std::recursive_mutex> lock(callbackMutex_);
  const auto list = observers();
  const Notification notification{v.name_, ConstValueRef(v.type(), data), timestamp, v};
  for (const ObserverEntry& observer : *list) {
    if (!observer.variable || observer.variable == &v) invoke(observer, notification);
  }
}

void PlcCore::invoke(const ObserverEntry& observer, const Notification& notification) noexcept {
  try {
    (*observer.callback)(notification);
  } catch (const std::exception& e) {
    reportError("callback for '" + notification.name + "' threw an exception: " + e.what());
  } catch (...) {
    reportError("callback for '" + notification.name + "' threw an exception");
  }
}

ObserverId PlcCore::addObserver(Callback callback, const Variable* variable, bool initialValues) {
  if (!callback) throw Error("empty callback");
  if (variable && !variable->isMonitored()) {
    throw KeyError("variable '" + variable->name_ + "' is not monitored: add it to 'variables_to_monitor'");
  }
  ObserverId id;
  {
    std::lock_guard<std::mutex> lock(observersMutex_);
    id = nextObserverId_++;
    auto list = std::make_shared<std::vector<ObserverEntry>>(*observers_);
    list->push_back(ObserverEntry{id, std::make_shared<const Callback>(std::move(callback)), variable});
    observers_ = std::move(list);
  }
  if (initialValues) {
    try {
      deliverCurrentValues(id);
    } catch (...) {
      removeObserver(id);  // the caller never gets the id
      throw;
    }
  }
  return id;
}

void PlcCore::removeObserver(ObserverId id) {
  {
    std::lock_guard<std::mutex> lock(observersMutex_);
    auto list = std::make_shared<std::vector<ObserverEntry>>(*observers_);
    list->erase(std::remove_if(list->begin(), list->end(), [&](const ObserverEntry& o) { return o.id == id; }),
                list->end());
    observers_ = std::move(list);
  }
  // Wait for a callback running in another thread (the observer list it
  // uses may still contain the removed observer).
  std::lock_guard<std::recursive_mutex> wait(callbackMutex_);
}

/// Reads the monitored variables and delivers them to one observer (or all).
void PlcCore::deliverCurrentValues(std::optional<ObserverId> only) {
  std::vector<const Variable*> targets;
  if (only) {
    const auto list = observers();
    const auto it = std::find_if(list->begin(), list->end(), [&](const ObserverEntry& o) { return o.id == *only; });
    if (it == list->end()) return;
    if (it->variable) {
      targets.push_back(it->variable);
    } else {
      targets.assign(monitored.begin(), monitored.end());
    }
  } else {
    targets.assign(monitored.begin(), monitored.end());
  }
  if (targets.empty()) return;

  std::vector<Value> values;
  std::vector<std::uint8_t*> outputs;
  values.reserve(targets.size());
  for (const Variable* v : targets) {
    values.push_back(v->createValue());
    outputs.push_back(values.back().data());
  }
  readMany(targets, outputs);
  const std::uint64_t timestamp = nowFileTime();

  std::lock_guard<std::recursive_mutex> lock(callbackMutex_);
  const auto list = observers();
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const Notification notification{targets[i]->name_, values[i].ref(), timestamp, *targets[i]};
    for (const ObserverEntry& observer : *list) {
      if (only && observer.id != *only) continue;
      if (!observer.variable || observer.variable == targets[i]) invoke(observer, notification);
    }
  }
}

void PlcCore::setErrorHandler(std::function<void(const std::string&)> handler) {
  std::lock_guard<std::mutex> lock(observersMutex_);
  errorHandler_ = handler ? std::move(handler) : defaultErrorHandler;
}

void PlcCore::reportError(const std::string& message) noexcept {
  try {
    std::function<void(const std::string&)> handler;
    {
      std::lock_guard<std::mutex> lock(observersMutex_);
      handler = errorHandler_;
    }
    handler(message);
  } catch (...) {
  }
}

}  // namespace detail

// ---------------------------------------------------------------------------
// Free functions

std::chrono::system_clock::time_point fileTimeToSystemTime(std::uint64_t fileTime) {
  const detail::FileTimeDuration sinceEpoch(static_cast<std::int64_t>(fileTime - detail::kUnixEpochFileTime));
  return std::chrono::system_clock::time_point(
      std::chrono::duration_cast<std::chrono::system_clock::duration>(sinceEpoch));
}

std::string toString(AdsState state) {
  static const char* const names[] = {"Invalid",  "Idle",     "Reset",  "Init",    "Start",        "Run",
                                      "Stop",     "SaveConfig", "LoadConfig", "PowerFailure", "PowerGood",
                                      "Error",    "Shutdown", "Suspend", "Resume", "Config",       "Reconfig",
                                      "Stopping", "Incompatible", "Exception"};
  const auto index = static_cast<std::size_t>(state);
  return index < sizeof(names) / sizeof(names[0]) ? names[index] : "Unknown (" + std::to_string(index) + ")";
}

// ---------------------------------------------------------------------------
// Variable

Variable::Variable(detail::PlcCore& core, std::string name, TypePtr type, bool writable, bool monitored,
                   NotificationSettings notification)
    : core_(&core),
      name_(std::move(name)),
      type_(std::move(type)),
      writable_(writable),
      monitored_(monitored),
      notification_(notification) {}

Value Variable::read() const {
  Value value(type_);
  core_->read(*this, value.data(), value.size());
  return value;
}

void Variable::read(Value& out) const {
  if (!out.valid() || !out.type().sameLayout(*type_)) out = Value(type_);
  core_->read(*this, out.data(), out.size());
}

void Variable::read(ValueRef out) const {
  if (!out.type().sameLayout(*type_)) {
    throw TypeError("cannot read '" + name_ + "' (" + type_->name() + ") into a value of type '" + out.type().name() +
                    "'");
  }
  core_->read(*this, out.data(), size());
}

void Variable::readRaw(void* data, std::size_t size) const {
  checkSize(size);
  core_->read(*this, data, size);
}

void Variable::writeRaw(const void* data, std::size_t size) {
  checkSize(size);
  core_->write(*this, data, size);
}

void Variable::write(const Json& value) {
  checkWritable();
  detail::ScratchBuffer buffer(size());
  try {
    ValueRef(*type_, buffer.data()).fromJson(value);
  } catch (const KeyError& e) {
    detail::rethrowFor(e, name_);
  } catch (const TypeError& e) {
    detail::rethrowFor(e, name_);
  } catch (const RangeError& e) {
    detail::rethrowFor(e, name_);
  }
  core_->write(*this, buffer.data(), size());
}

void Variable::writeValue(ConstValueRef value) {
  if (value.type().sameLayout(*type_)) {
    core_->write(*this, value.data(), size());
    return;
  }
  detail::ScratchBuffer buffer(size());
  ValueRef(*type_, buffer.data()).assign(value);
  core_->write(*this, buffer.data(), size());
}

void Variable::checkSize(std::size_t size) const {
  if (size != type_->size()) detail::throwRawSize(*type_, size);
}

void Variable::checkWritable() const {
  if (!writable_) throw KeyError("variable '" + name_ + "' is not writable: add it to 'variables_to_write'");
}

ObserverId Variable::subscribe(Callback callback, bool initialValue) {
  return core_->addObserver(std::move(callback), this, initialValue);
}

// ---------------------------------------------------------------------------
// BeckhoffPLC

BeckhoffPLC::BeckhoffPLC(Config config) : core_(std::make_unique<detail::PlcCore>(std::move(config))) {}

BeckhoffPLC::BeckhoffPLC(const Json& params) : BeckhoffPLC(Config::fromJson(params)) {}

BeckhoffPLC BeckhoffPLC::fromFile(const std::string& path, const std::string& key) {
  return BeckhoffPLC(Config::fromFile(path, key));
}

BeckhoffPLC::~BeckhoffPLC() = default;
BeckhoffPLC::BeckhoffPLC(BeckhoffPLC&&) noexcept = default;
BeckhoffPLC& BeckhoffPLC::operator=(BeckhoffPLC&&) noexcept = default;

namespace {
detail::PlcCore& checked(const std::unique_ptr<detail::PlcCore>& core) {
  if (!core) throw Error("use of a moved-from BeckhoffPLC");
  return *core;
}
}  // namespace

Variable& BeckhoffPLC::variable(const std::string& name) { return checked(core_).variable(name); }

const Variable& BeckhoffPLC::variable(const std::string& name) const { return checked(core_).variable(name); }

bool BeckhoffPLC::hasVariable(const std::string& name) const noexcept { return core_ && core_->hasVariable(name); }

std::vector<std::string> BeckhoffPLC::variableNames() const {
  std::vector<std::string> names;
  for (const auto& v : checked(core_).variables) names.push_back(v->name());
  return names;
}

std::vector<Value> BeckhoffPLC::getVariables(const std::vector<std::string>& names) const {
  std::vector<Value> values;
  getVariables(names, values);
  return values;
}

void BeckhoffPLC::getVariables(const std::vector<std::string>& names, std::vector<Value>& out) const {
  detail::PlcCore& core = checked(core_);
  std::vector<const Variable*> vars;
  vars.reserve(names.size());
  for (const std::string& name : names) vars.push_back(&core.variable(name));
  out.resize(names.size());
  std::vector<std::uint8_t*> outputs(names.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (!out[i].valid() || !out[i].type().sameLayout(vars[i]->type())) out[i] = vars[i]->createValue();
    outputs[i] = out[i].data();
  }
  core.readMany(vars, outputs);
}

void BeckhoffPLC::setVariables(const Json& values) {
  detail::PlcCore& core = checked(core_);
  if (!values.is_object()) throw TypeError("setVariables() expects an object {\"variable name\": value, ...}");
  std::vector<const Variable*> vars;
  std::vector<Value> buffers;
  buffers.reserve(values.size());
  for (const auto& item : values.items()) {
    const Variable& v = core.variable(item.key());
    if (!v.isWritable()) throw KeyError("variable '" + v.name() + "' is not writable: add it to 'variables_to_write'");
    buffers.push_back(v.createValue());
    try {
      buffers.back().fromJson(item.value());
    } catch (const KeyError& e) {
      detail::rethrowFor(e, v.name());
    } catch (const TypeError& e) {
      detail::rethrowFor(e, v.name());
    } catch (const RangeError& e) {
      detail::rethrowFor(e, v.name());
    }
    vars.push_back(&v);
  }
  std::vector<const std::uint8_t*> inputs;
  for (const Value& buffer : buffers) inputs.push_back(buffer.data());
  core.writeMany(vars, inputs);
}

void BeckhoffPLC::setVariables(const std::vector<std::pair<std::string, ConstValueRef>>& values) {
  detail::PlcCore& core = checked(core_);
  std::vector<const Variable*> vars;
  std::vector<const std::uint8_t*> inputs;
  std::vector<Value> converted;
  converted.reserve(values.size());
  for (const auto& [name, value] : values) {
    const Variable& v = core.variable(name);
    if (value.type().sameLayout(v.type())) {
      inputs.push_back(value.data());
    } else {
      converted.push_back(v.createValue());
      converted.back().ref().assign(value);
      inputs.push_back(converted.back().data());
    }
    vars.push_back(&v);
  }
  core.writeMany(vars, inputs);
}

ObserverId BeckhoffPLC::addObserver(Callback callback, bool initialValues) {
  return checked(core_).addObserver(std::move(callback), nullptr, initialValues);
}

ObserverId BeckhoffPLC::subscribe(const std::string& name, Callback callback, bool initialValue) {
  return variable(name).subscribe(std::move(callback), initialValue);
}

void BeckhoffPLC::removeObserver(ObserverId id) { checked(core_).removeObserver(id); }

void BeckhoffPLC::readInitialValues() { checked(core_).deliverCurrentValues(std::nullopt); }

void BeckhoffPLC::setErrorHandler(std::function<void(const std::string&)> handler) {
  checked(core_).setErrorHandler(std::move(handler));
}

Value BeckhoffPLC::createValue(std::string_view type) const { return Value(checked(core_).types.parse(type)); }

Value BeckhoffPLC::createValue(std::string_view type, const Json& json) const {
  return Value(checked(core_).types.parse(type), json);
}

const TypeRegistry& BeckhoffPLC::types() const { return checked(core_).types; }

const Config& BeckhoffPLC::config() const { return checked(core_).config; }

AdsState BeckhoffPLC::readState() const { return checked(core_).readState(); }

void BeckhoffPLC::reconnect() { checked(core_).reconnect(); }

void BeckhoffPLC::createRoute(const Config& config) {
  if (!isValidNetId(config.clientNetId)) {
    throw ConfigError("creating a route needs 'client_net_id' (the AMS Net ID of this computer)");
  }
  const std::string plcIp = config.ipAddress.empty() ? netIdToIp(config.netId) : config.ipAddress;
  const std::string clientIp = config.clientIpAddress.empty() ? netIdToIp(config.clientNetId) : config.clientIpAddress;
  const std::string routeName = config.routeName.empty() ? "beckhoff_plc" : config.routeName;
  const std::string context = "adding a route to " + clientIp + " (" + config.clientNetId + ") on the PLC at " + plcIp;
  long status;
  try {
    status = bhf::ads::AddRemoteRoute(plcIp, AmsNetId(config.clientNetId), clientIp, routeName, config.username,
                                      config.password);
  } catch (const Socket::TimeoutEx&) {
    throw AdsError(ADSERR_CLIENT_SYNCTIMEOUT, context + ": no answer");
  } catch (const std::exception& e) {
    throw AdsError(-1, context + ": " + e.what());
  }
  if (status) throw AdsError(status, context + " (check username and password)");
}

void BeckhoffPLC::setAdsLogLevel(int level) { Logger::logLevel = static_cast<std::size_t>(std::clamp(level, 0, 4)); }

}  // namespace beckhoff
