#include "beckhoff_plc/config.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_set>

#include "util.hpp"

namespace beckhoff {

namespace {

std::string describe(const Json& json) {
  std::string text = json.dump(-1, ' ', false, Json::error_handler_t::replace);
  return text.size() > 60 ? text.substr(0, 57) + "..." : text;
}

[[noreturn]] void invalid(const std::string& key, const std::string& message) {
  throw ConfigError("parameter '" + key + "': " + message);
}

std::string getString(const Json& json, const std::string& key) {
  if (!json.is_string()) invalid(key, "expected a string, got " + describe(json));
  return json.get<std::string>();
}

/// Username/password may be numbers in hand written files ("password": 1).
std::string getText(const Json& json, const std::string& key) {
  if (json.is_number_integer()) return json.dump();
  return getString(json, key);
}

bool getBool(const Json& json, const std::string& key) {
  if (!json.is_boolean()) invalid(key, "expected true or false, got " + describe(json));
  return json.get<bool>();
}

std::int64_t getInteger(const Json& json, const std::string& key, std::int64_t min, std::int64_t max) {
  if (!json.is_number_integer()) invalid(key, "expected an integer, got " + describe(json));
  const bool tooLarge = json.is_number_unsigned() && json.get<std::uint64_t>() > static_cast<std::uint64_t>(max);
  const std::int64_t value = tooLarge ? max : json.get<std::int64_t>();
  if (tooLarge || value < min || value > max) {
    invalid(key, "must be between " + std::to_string(min) + " and " + std::to_string(max));
  }
  return value;
}

/// Milliseconds (possibly fractional) to nanoseconds, limited to what fits
/// in the 32-bit 100 ns fields of ADS notifications.
std::chrono::nanoseconds getMilliseconds(const Json& json, const std::string& key) {
  if (!json.is_number()) invalid(key, "expected a number of milliseconds, got " + describe(json));
  const double ms = json.get<double>();
  if (!std::isfinite(ms) || ms < 0 || ms > 429496.0) invalid(key, "must be between 0 and 429496 ms");
  return std::chrono::nanoseconds(static_cast<std::int64_t>(std::llround(ms * 1e6)));
}

const Json& getObject(const Json& json, const std::string& key) {
  if (!json.is_object()) invalid(key, "expected an object, got " + describe(json));
  return json;
}

const Json& getArray(const Json& json, const std::string& key) {
  if (!json.is_array()) invalid(key, "expected a list, got " + describe(json));
  return json;
}

/// Type specification: "UINT", "CustomStruct1_s", "ARRAY[0..4] OF UINT" or
/// the Python list form [type, size] (and [type, size1, size2, ...] for
/// multi-dimensional arrays).
std::string typeSpec(const Json& json, const std::string& key) {
  if (json.is_string()) {
    std::string spec = json.get<std::string>();
    if (spec.find_first_not_of(" \t") == std::string::npos) invalid(key, "empty type");
    return spec;
  }
  if (json.is_array()) {
    if (json.size() < 2) invalid(key, "expected [type, size], got " + describe(json));
    std::string ranges;
    for (std::size_t i = 1; i < json.size(); ++i) {
      const std::int64_t size = getInteger(json[i], key, 1, 0x7FFFFFFF);
      ranges += (i > 1 ? ", 0.." : "0..") + std::to_string(size - 1);
    }
    return "ARRAY[" + ranges + "] OF " + typeSpec(json[0], key);
  }
  invalid(key, "expected a type name or [type, size], got " + describe(json));
}

std::string trim(std::string_view text) {
  const std::size_t start = text.find_first_not_of(" \t\r\n");
  if (start == std::string_view::npos) return {};
  const std::size_t end = text.find_last_not_of(" \t\r\n");
  return std::string(text.substr(start, end - start + 1));
}

/// Structure members: {"attr1": "uint16", ...} (Python format) or a list of
/// ["attr1", "uint16"] pairs / "attr1 : UINT;" declarations, which keeps the
/// member order in formats whose maps are unordered (e.g. ROS parameters).
std::vector<FieldDefinition> structFields(const Json& json, const std::string& key) {
  std::vector<FieldDefinition> fields;
  if (json.is_object()) {
    for (const auto& item : json.items()) {
      fields.push_back({item.key(), typeSpec(item.value(), key + "." + item.key())});
    }
  } else if (json.is_array()) {
    for (const Json& member : json) {
      if (member.is_array() && member.size() == 2 && member[0].is_string()) {
        const std::string name = member[0].get<std::string>();
        fields.push_back({name, typeSpec(member[1], key + "." + name)});
      } else if (member.is_string()) {
        const std::string declaration = member.get<std::string>();
        const std::size_t colon = declaration.find(':');
        if (colon == std::string::npos) invalid(key, "expected 'name : type', got '" + declaration + "'");
        std::string type = trim(declaration.substr(colon + 1));
        if (!type.empty() && type.back() == ';') type = trim(std::string_view(type).substr(0, type.size() - 1));
        fields.push_back({trim(declaration.substr(0, colon)), type});
      } else {
        invalid(key, "expected [\"name\", type] or \"name : type\", got " + describe(member));
      }
    }
  } else {
    invalid(key, "each structure must be an object of {field: type} (got " + describe(json) + ")");
  }
  if (fields.empty()) invalid(key, "structure has no fields");
  return fields;
}

NotificationSettings notificationSettings(const Json& json, const std::string& key, NotificationSettings settings,
                                          bool allowName) {
  for (const auto& item : json.items()) {
    const std::string& name = item.key();
    const std::string itemKey = key + "." + name;
    if (name == "mode") {
      const std::string mode = getString(item.value(), itemKey);
      if (util::iequals(mode, "on_change")) {
        settings.mode = NotificationSettings::Mode::OnChange;
      } else if (util::iequals(mode, "cyclic")) {
        settings.mode = NotificationSettings::Mode::Cyclic;
      } else {
        invalid(itemKey, "expected \"on_change\" or \"cyclic\", got \"" + mode + "\"");
      }
    } else if (name == "cycle_time_ms") {
      settings.cycleTime = getMilliseconds(item.value(), itemKey);
    } else if (name == "max_delay_ms") {
      settings.maxDelay = getMilliseconds(item.value(), itemKey);
    } else if (!(allowName && name == "name")) {
      invalid(itemKey, "unknown setting (expected mode, cycle_time_ms or max_delay_ms)");
    }
  }
  return settings;
}

}  // namespace

bool isValidNetId(std::string_view netId) {
  int parts = 0;
  std::size_t pos = 0;
  while (true) {
    const std::size_t start = pos;
    unsigned value = 0;
    while (pos < netId.size() && netId[pos] >= '0' && netId[pos] <= '9' && pos - start < 3) {
      value = value * 10 + static_cast<unsigned>(netId[pos++] - '0');
    }
    if (pos == start || value > 255) return false;
    ++parts;
    if (pos == netId.size()) return parts == 6;
    if (netId[pos] != '.' || parts == 6) return false;
    ++pos;
  }
}

std::string netIdToIp(const std::string& netId) {
  if (!isValidNetId(netId)) throw ConfigError("invalid AMS Net ID '" + netId + "'");
  std::size_t pos = 0;
  for (int i = 0; i < 4; ++i) pos = netId.find('.', pos) + 1;
  return netId.substr(0, pos - 1);
}

Config Config::fromJson(const Json& document, const std::string& key) {
  const Json* json = &document;
  if (!document.is_object()) throw ConfigError("the PLC parameters must be a JSON object");
  if (!document.contains("net_id") && !key.empty()) {
    if (!document.contains(key)) throw ConfigError("missing 'net_id' (or a '" + key + "' object holding it)");
    json = &document[key];
    if (!json->is_object()) invalid(key, "expected an object");
  }

  Config config;
  std::vector<std::pair<std::string, int>> packModes;
  for (const auto& item : json->items()) {
    const std::string& name = item.key();
    const Json& value = item.value();
    if (name == "net_id") {
      config.netId = getString(value, name);
    } else if (name == "ip_address") {
      config.ipAddress = getString(value, name);
    } else if (name == "port") {
      config.port = static_cast<std::uint16_t>(getInteger(value, name, 1, 65535));
    } else if (name == "client_net_id") {
      config.clientNetId = getString(value, name);
    } else if (name == "timeout_ms") {
      config.timeout = std::chrono::milliseconds(getInteger(value, name, 1, 0xFFFFFFFF));
    } else if (name == "username") {
      config.username = getText(value, name);
    } else if (name == "password") {
      config.password = getText(value, name);
    } else if (name == "client_ip_address") {
      config.clientIpAddress = getString(value, name);
    } else if (name == "route_name") {
      config.routeName = getString(value, name);
    } else if (name == "create_route") {
      config.createRoute = getBool(value, name);
    } else if (name == "pack_mode") {
      config.packMode = static_cast<int>(getInteger(value, name, 0, 8));
    } else if (name == "aliases") {
      for (const auto& alias : getObject(value, name).items()) {
        config.aliases.push_back({alias.key(), typeSpec(alias.value(), name + "." + alias.key())});
      }
    } else if (name == "structures") {
      for (const auto& structure : getObject(value, name).items()) {
        config.structures.push_back(
            {structure.key(), structFields(structure.value(), name + "." + structure.key()), std::nullopt});
      }
    } else if (name == "pack_modes") {
      for (const auto& structure : getObject(value, name).items()) {
        packModes.emplace_back(structure.key(),
                               static_cast<int>(getInteger(structure.value(), name + "." + structure.key(), 0, 8)));
      }
    } else if (name == "variables") {
      for (const auto& variable : getObject(value, name).items()) {
        config.variables.push_back({variable.key(), typeSpec(variable.value(), name + "." + variable.key())});
      }
    } else if (name == "variables_to_monitor") {
      for (const Json& entry : getArray(value, name)) {
        if (entry.is_string()) {
          config.variablesToMonitor.push_back({entry.get<std::string>(), std::nullopt});
        } else if (entry.is_object() && entry.contains("name") && entry["name"].is_string()) {
          // C++ only: {"name": "GVL.x", "mode": "cyclic", "cycle_time_ms": 10}
          config.variablesToMonitor.push_back({entry["name"].get<std::string>(), NotificationSettings{}});
        } else {
          invalid(name, "expected a variable name or {\"name\": ..., <notification settings>}, got " +
                            describe(entry));
        }
      }
    } else if (name == "variables_to_write") {
      for (const Json& entry : getArray(value, name)) config.variablesToWrite.push_back(getString(entry, name));
    } else if (name == "notification") {
      config.notification = notificationSettings(getObject(value, name), name, NotificationSettings{}, false);
    } else if (name == "verify_sizes") {
      config.verifySizes = getBool(value, name);
    } else {
      config.ignoredKeys.push_back(name);
    }
  }

  // Per-variable notification settings start from the (now known) defaults.
  if (json->contains("variables_to_monitor")) {
    std::size_t index = 0;
    for (const Json& entry : (*json)["variables_to_monitor"]) {
      if (entry.is_object()) {
        config.variablesToMonitor[index].notification =
            notificationSettings(entry, "variables_to_monitor." + entry["name"].get<std::string>(),
                                 config.notification, true);
      }
      ++index;
    }
  }

  for (const auto& [structName, packMode] : packModes) {
    bool found = false;
    for (StructDefinition& structure : config.structures) {
      if (util::iequals(structure.name, structName)) {
        structure.packMode = packMode;
        found = true;
      }
    }
    if (!found) invalid("pack_modes." + structName, "no structure with that name");
  }

  config.validate();
  return config;
}

Config Config::fromJsonText(std::string_view text, const std::string& key) {
  Json json;
  try {
    json = Json::parse(text.begin(), text.end(), nullptr, true, /*ignore_comments=*/true);
  } catch (const Json::exception& e) {
    throw ConfigError(std::string("invalid JSON: ") + e.what());
  }
  return fromJson(json, key);
}

Config Config::fromFile(const std::string& path, const std::string& key) {
  std::ifstream file(path);
  if (!file) throw ConfigError("cannot open parameter file '" + path + "'");
  std::stringstream text;
  text << file.rdbuf();
  try {
    return fromJsonText(text.str(), key);
  } catch (const ConfigError& e) {
    throw ConfigError(path + ": " + e.what());
  }
}

TypeRegistry Config::createTypeRegistry() const {
  try {
    TypeRegistry registry(packMode);
    registry.define(aliases, structures);
    return registry;
  } catch (const ConfigError&) {
    throw;
  } catch (const Error& e) {
    throw ConfigError(e.what());
  }
}

void Config::validate() const {
  if (netId.empty()) throw ConfigError("missing parameter 'net_id'");
  if (!isValidNetId(netId)) invalid("net_id", "invalid AMS Net ID '" + netId + "' (expected e.g. 192.168.1.10.1.1)");
  if (!clientNetId.empty() && !isValidNetId(clientNetId)) {
    invalid("client_net_id", "invalid AMS Net ID '" + clientNetId + "'");
  }
  if (port == 0) invalid("port", "must not be 0");
  if (timeout.count() <= 0) invalid("timeout_ms", "must be positive");

  const TypeRegistry types = createTypeRegistry();

  std::unordered_set<std::string> names;
  for (const VariableDefinition& variable : variables) {
    if (variable.name.empty()) invalid("variables", "empty variable name");
    if (!names.insert(variable.name).second) invalid("variables", "'" + variable.name + "' is defined more than once");
    try {
      types.parse(variable.type);
    } catch (const Error& e) {
      invalid("variables." + variable.name, e.what());
    }
  }
  const auto checkList = [&](const char* list, const std::vector<std::string>& entries) {
    std::unordered_set<std::string> seen;
    for (const std::string& name : entries) {
      if (!names.count(name)) {
        invalid(list, "'" + name + "' is not defined in 'variables'");
      }
      if (!seen.insert(name).second) invalid(list, "'" + name + "' is listed more than once");
    }
  };
  std::vector<std::string> monitored;
  for (const MonitoredVariable& variable : variablesToMonitor) monitored.push_back(variable.name);
  checkList("variables_to_monitor", monitored);
  checkList("variables_to_write", variablesToWrite);
}

}  // namespace beckhoff
