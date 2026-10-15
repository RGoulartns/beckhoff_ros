#include "beckhoff_ros/beckhoff_plc.hpp"

#include <AdsDevice.h>
#include <AdsLib.h>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace beckhoff_ros
{

namespace
{

// Route name used by createRoute (the same as the Python version)
constexpr const char * kRouteName = "jetson-to-plc";

const nlohmann::ordered_json & require(
  const nlohmann::ordered_json & params, const std::string & key)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    throw ConfigError("Missing PLC parameter '" + key + "'");
  }
  return *it;
}

std::string requireString(const nlohmann::ordered_json & params, const std::string & key)
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

std::vector<std::string> requireStringList(
  const nlohmann::ordered_json & params, const std::string & key)
{
  const auto & value = require(params, key);
  if (!value.is_array() ||
    !std::all_of(value.begin(), value.end(), [](const auto & item) {return item.is_string();}))
  {
    throw ConfigError("PLC parameter '" + key + "' must be a list of variable names");
  }
  return value.get<std::vector<std::string>>();
}

std::string describeAdsError(const std::string & what, long code)
{
  const std::string text = adsErrorText(code);
  return what + " (ADS error " + std::to_string(code) + (text.empty() ? "" : ": " + text) + ")";
}

AmsNetId parseNetId(const std::string & net_id)
{
  std::uint8_t bytes[6];
  std::size_t count = 0;
  std::size_t pos = 0;
  while (count < 6) {
    std::size_t end = pos;
    unsigned value = 0;
    while (end < net_id.size() && end - pos < 3 && net_id[end] >= '0' && net_id[end] <= '9') {
      value = value * 10 + static_cast<unsigned>(net_id[end++] - '0');
    }
    if (end == pos || value > 255) {
      break;
    }
    bytes[count++] = static_cast<std::uint8_t>(value);
    pos = end;
    if (count < 6 && (pos >= net_id.size() || net_id[pos++] != '.')) {
      break;
    }
  }
  if (count != 6 || pos != net_id.size()) {
    throw ConfigError(
            "Invalid AMS Net Id '" + net_id + "' (expected 6 numbers, e.g. 192.168.1.10.1.1)");
  }
  return AmsNetId(bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5]);
}

/// Notifications are dispatched by AdsLib to a plain function with a 32 bit user value.
/// This registry maps that value to the BeckhoffPLC and the variable. The mutex is held while a
/// notification is handled, so unregistering waits until running callbacks are finished.
struct NotificationTarget
{
  BeckhoffPLC * plc;
  std::size_t index;
};

std::mutex & registryMutex()
{
  static std::mutex mutex;
  return mutex;
}

std::unordered_map<std::uint32_t, NotificationTarget> & registry()
{
  static std::unordered_map<std::uint32_t, NotificationTarget> targets;
  return targets;
}

std::uint32_t registerTarget(BeckhoffPLC * plc, std::size_t index)
{
  static std::uint32_t next_id = 0;
  std::lock_guard<std::mutex> lock(registryMutex());
  auto & targets = registry();
  do {
    ++next_id;
  } while (next_id == 0 || targets.count(next_id) != 0);
  targets[next_id] = NotificationTarget{plc, index};
  return next_id;
}

}  // namespace

AdsError::AdsError(const std::string & what, long code)
: std::runtime_error(describeAdsError(what, code)), code_(code)
{
}

std::string adsErrorText(long code)
{
  switch (code) {
    case -1: return "request failed, the connection may be lost";
    case GLOBALERR_TARGET_PORT: return "target port not found or PLC not reachable";
    case GLOBALERR_MISSING_ROUTE: return "target machine not found, possibly missing ADS routes";
    case GLOBALERR_NO_MEMORY: return "no memory";
    case GLOBALERR_TCP_SEND: return "TCP send error";
    case ROUTERERR_PORTALREADYINUSE: return "port already in use";
    case ROUTERERR_NOMOREQUEUES: return "maximum number of ports reached";
    case ADSERR_DEVICE_ERROR: return "device error";
    case ADSERR_DEVICE_SRVNOTSUPP: return "service is not supported by the server";
    case ADSERR_DEVICE_INVALIDGRP: return "invalid index group";
    case ADSERR_DEVICE_INVALIDOFFSET: return "invalid index offset";
    case ADSERR_DEVICE_INVALIDACCESS: return "reading/writing not permitted";
    case ADSERR_DEVICE_INVALIDSIZE:
      return "parameter size not correct, check the variable type and structure definitions";
    case ADSERR_DEVICE_INVALIDDATA: return "invalid parameter value(s)";
    case ADSERR_DEVICE_NOTREADY: return "device is not in a ready state";
    case ADSERR_DEVICE_BUSY: return "device is busy";
    case ADSERR_DEVICE_NOMEMORY: return "out of memory";
    case ADSERR_DEVICE_INVALIDPARM: return "invalid parameter value(s)";
    case ADSERR_DEVICE_NOTFOUND: return "not found";
    case ADSERR_DEVICE_SYMBOLNOTFOUND: return "symbol not found";
    case ADSERR_DEVICE_SYMBOLVERSIONINVALID: return "symbol version invalid (online change?)";
    case ADSERR_DEVICE_INVALIDSTATE: return "server is in invalid state";
    case ADSERR_DEVICE_TRANSMODENOTSUPP: return "transmission mode not supported";
    case ADSERR_DEVICE_NOTIFYHNDINVALID: return "notification handle is invalid";
    case ADSERR_DEVICE_NOMOREHDLS: return "no more notification handles";
    case ADSERR_DEVICE_INVALIDWATCHSIZE: return "notification size too large";
    case ADSERR_DEVICE_TIMEOUT: return "device timeout";
    case ADSERR_DEVICE_SYMBOLNOTACTIVE: return "symbol not active (online change?)";
    case ADSERR_DEVICE_ACCESSDENIED: return "access denied";
    case ADSERR_CLIENT_ERROR: return "client error";
    case ADSERR_CLIENT_INVALIDPARM: return "invalid parameter at service call";
    case ADSERR_CLIENT_SYNCTIMEOUT:
      return "timeout elapsed, check the ADS routes of the PLC and this computer";
    case ADSERR_CLIENT_PORTNOTOPEN: return "ADS port not opened";
    case ADSERR_CLIENT_NOAMSADDR: return "no AMS address";
    case ADSERR_CLIENT_SYNCRESINVALID: return "invalid response received";
    default: return "";
  }
}

/// AdsLib objects. The members are destroyed in reverse order: the notification callbacks are
/// unregistered first, then the notifications and handles are deleted and the port closed.
struct BeckhoffPLC::Ads
{
  struct Notification
  {
    AdsHandle symbol;
    AdsHandle notification;  // deleted before its symbol handle is released
  };

  /// Removes the registered notification callbacks, waiting for the running ones to finish.
  struct Callbacks
  {
    std::vector<std::uint32_t> ids;
    ~Callbacks()
    {
      std::lock_guard<std::mutex> lock(registryMutex());
      for (const auto id : ids) {
        registry().erase(id);
      }
    }
  };

  Ads(const std::string & ip, const AmsNetId & net_id)
  : device(ip, net_id, AMSPORT_R0_PLC_TC3) {}

  static void onNotification(
    const AmsAddr * /*address*/, const AdsNotificationHeader * header, std::uint32_t user)
  {
    std::lock_guard<std::mutex> lock(registryMutex());
    const auto it = registry().find(user);
    if (it == registry().end()) {
      return;
    }
    BeckhoffPLC * plc = it->second.plc;
    try {
      // the sample data follows the header
      plc->updateCB(
        it->second.index, reinterpret_cast<const std::uint8_t *>(header + 1),
        header->cbSampleSize);
    } catch (const std::exception & e) {
      plc->log(LogLevel::Error, std::string("Error handling a PLC notification: ") + e.what());
    } catch (...) {
      plc->log(LogLevel::Error, "Unknown error handling a PLC notification");
    }
  }

  AdsHandle getHandle(const std::string & name) const
  {
    try {
      return device.GetHandle(name);
    } catch (const AdsException & e) {
      throw AdsError("Unable to get a handle for '" + name + "'", e.errorCode);
    }
  }

  /// AdsLib allows a single pending request per port: requests are serialized.
  std::mutex request_mutex;
  AdsDevice device;
  std::map<std::string, AdsHandle> write_handles;
  std::vector<Notification> notifications;
  Callbacks callbacks;
};

BeckhoffPLC::BeckhoffPLC(const nlohmann::ordered_json & params, Logger logger)
: params_(params), logger_(std::move(logger))
{
  if (!params_.is_object()) {
    throw ConfigError("The PLC parameters must be a JSON object");
  }
  if (params_.contains("pack")) {
    const auto & pack = params_["pack"];
    if (!pack.is_number_integer() || pack.get<std::int64_t>() < 0) {
      throw ConfigError("PLC parameter 'pack' must be a non negative integer");
    }
    pack_ = pack.get<std::size_t>();
  }
  const std::string net_id = requireString(params_, "net_id");
  const AmsNetId ams_net_id = parseNetId(net_id);

  loadStructures(require(params_, "structures"));
  loadVariables(require(params_, "variables"));
  monitored_ = requireStringList(params_, "variables_to_monitor");
  const auto variables_to_write = requireStringList(params_, "variables_to_write");
  for (const auto & name : monitored_) {
    requireVariable(name);
  }
  for (const auto & name : variables_to_write) {
    requireVariable(name);
  }

  // like pyads.Connection(net_id, PORT_TC3PLC1).open(): the PLC IP is part of its net id
  const std::string ip = netId2ip(net_id);
  try {
    ads_ = std::make_unique<Ads>(ip, ams_net_id);
  } catch (const AdsException & e) {
    throw AdsError("Unable to connect to the PLC " + net_id + " at " + ip, e.errorCode);
  }

  // variable monitoring uses notification callbacks
  for (std::size_t index = 0; index < monitored_.size(); ++index) {
    const std::string & name = monitored_[index];
    AdsNotificationAttrib attributes{};
    attributes.cbLength = static_cast<std::uint32_t>(variables_.at(name)->size());
    // same attributes as pyads.NotificationAttrib: on change, max delay and cycle time 1e-4 ms
    attributes.nTransMode = ADSTRANS_SERVERONCHA;
    attributes.nMaxDelay = 1;
    attributes.nCycleTime = 1;

    AdsHandle symbol = ads_->getHandle(name);
    // registered first: the PLC sends the current value right after adding the notification
    ads_->callbacks.ids.push_back(registerTarget(this, index));
    try {
      AdsHandle notification = ads_->device.GetHandle(
        ADSIGRP_SYM_VALBYHND, *symbol, attributes, &Ads::onNotification,
        ads_->callbacks.ids.back());
      ads_->notifications.push_back(Ads::Notification{std::move(symbol), std::move(notification)});
    } catch (const AdsException & e) {
      throw AdsError("Unable to add a notification for '" + name + "'", e.errorCode);
    }
  }

  // handles used to speed up communication
  for (const auto & name : variables_to_write) {
    ads_->write_handles.erase(name);  // AdsHandle is not assignable
    ads_->write_handles.emplace(name, ads_->getHandle(name));
  }
}

BeckhoffPLC::~BeckhoffPLC()
{
  // stop notifications, delete them, release the handles and close the connection
  ads_.reset();
}

void BeckhoffPLC::addObserver(Observer observer)
{
  {
    std::lock_guard<std::recursive_mutex> lock(observers_mutex_);
    observers_.push_back(std::move(observer));
  }
  readInitialValues();
}

void BeckhoffPLC::notify(const std::string & source, const Value & msg)
{
  std::lock_guard<std::recursive_mutex> lock(observers_mutex_);
  for (const auto & observer : observers_) {
    observer(source, msg);
  }
}

void BeckhoffPLC::updateCB(std::size_t index, const std::uint8_t * data, std::size_t size)
{
  std::lock_guard<std::recursive_mutex> lock(observers_mutex_);
  if (observers_.empty()) {
    return;
  }
  const std::string & name = monitored_.at(index);
  const PlcTypePtr & type = variables_.at(name);
  if (size != type->size()) {
    log(
      LogLevel::Warn, "Ignoring notification of '" + name + "': " + std::to_string(size) +
      " bytes received, " + std::to_string(type->size()) + " expected");
    return;
  }
  notify(name, type->decode(data));
}

void BeckhoffPLC::setVariable(const std::string & name, const Value & value)
{
  const PlcTypePtr & type = requireVariable(name);
  const std::vector<std::uint8_t> data = type->encode(value, name);

  std::lock_guard<std::mutex> lock(ads_->request_mutex);
  const auto handle = ads_->write_handles.find(name);
  if (handle == ads_->write_handles.end()) {
    throw ConfigError("Variable '" + name + "' is not listed in variables_to_write");
  }
  const long error = ads_->device.WriteReqEx(
    ADSIGRP_SYM_VALBYHND, *handle->second, data.size(), data.data());
  if (error != 0) {
    throw AdsError("Unable to write '" + name + "'", error);
  }
}

Value BeckhoffPLC::getVariable(const std::string & name)
{
  const PlcTypePtr & type = requireVariable(name);
  std::vector<std::uint8_t> data(type->size());
  {
    std::lock_guard<std::mutex> lock(ads_->request_mutex);
    // like pyads' read_by_name: get a handle, read by handle and release the handle
    const AdsHandle handle = ads_->getHandle(name);
    std::uint32_t bytes_read = 0;
    const long error = ads_->device.ReadReqEx2(
      ADSIGRP_SYM_VALBYHND, *handle, data.size(), data.data(), &bytes_read);
    if (error != 0) {
      throw AdsError("Unable to read '" + name + "'", error);
    }
    if (bytes_read != data.size()) {
      throw AdsError(
              "Unable to read '" + name + "': " + std::to_string(bytes_read) + " bytes read, " +
              std::to_string(data.size()) + " expected", ADSERR_DEVICE_INVALIDSIZE);
    }
  }
  return type->decode(data);
}

void BeckhoffPLC::readInitialValues()
{
  for (const auto & name : monitored_) {
    // holding the lock keeps a concurrent notification from being overtaken by an older value
    std::lock_guard<std::recursive_mutex> lock(observers_mutex_);
    notify(name, getVariable(name));
  }
}

void BeckhoffPLC::loadStructures(const nlohmann::ordered_json & structures)
{
  if (!structures.is_object()) {
    throw ConfigError("PLC parameter 'structures' must be a dictionary");
  }
  for (auto it = structures.begin(); it != structures.end(); ++it) {
    // structures are added to the type registry for future reference
    types_.addStructure(it.key(), it.value(), pack_);
  }
}

void BeckhoffPLC::loadVariables(const nlohmann::ordered_json & variables)
{
  if (!variables.is_object()) {
    throw ConfigError("PLC parameter 'variables' must be a dictionary");
  }
  for (auto it = variables.begin(); it != variables.end(); ++it) {
    if (it.value().is_object()) {
      throw ConfigError("Variable " + it.key() + " cannot be a dict (ie, structure)");
    }
    try {
      variables_[it.key()] = types_.resolve(it.value());
    } catch (const ConfigError & e) {
      throw ConfigError("Variable " + it.key() + ": " + e.what());
    }
  }
}

PlcTypePtr BeckhoffPLC::variableType(const std::string & name) const
{
  const auto it = variables_.find(name);
  return it == variables_.end() ? nullptr : it->second;
}

const PlcTypePtr & BeckhoffPLC::requireVariable(const std::string & name) const
{
  const auto it = variables_.find(name);
  if (it == variables_.end()) {
    throw ConfigError("Variable '" + name + "' is not listed in variables");
  }
  return it->second;
}

void BeckhoffPLC::createRoute()
{
  createRoute(params_);
}

void BeckhoffPLC::createRoute(const nlohmann::ordered_json & params)
{
  const std::string net_id = requireString(params, "net_id");
  const std::string client_net_id = requireString(params, "client_net_id");
  const AmsNetId client = parseNetId(client_net_id);
  parseNetId(net_id);

  bhf::ads::SetLocalAddress(client);
  const long error = bhf::ads::AddRemoteRoute(
    netId2ip(net_id), client, netId2ip(client_net_id), kRouteName,
    requireString(params, "username"), requireString(params, "password"));
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

void BeckhoffPLC::log(LogLevel level, const std::string & message) const
{
  if (logger_) {
    logger_(level, message);
    return;
  }
  static const char * const kLevels[] = {"DEBUG", "INFO", "WARN", "ERROR"};
  std::cerr << "[beckhoff_plc] [" << kLevels[static_cast<int>(level)] << "] " << message <<
    std::endl;
}

}  // namespace beckhoff_ros
