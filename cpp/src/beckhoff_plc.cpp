#include "beckhoff/beckhoff_plc.hpp"
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace beckhoff {
namespace {
const Json& configuration(const Json& parameters) {
    if (!parameters.is_object()) throw std::invalid_argument("PLC configuration must be an object");
    const auto& config = parameters.contains("PLC") ? parameters.at("PLC") : parameters;
    if (!config.is_object()) throw std::invalid_argument("PLC configuration must be an object");
    return config;
}
std::uint32_t number(const Json& config, const char* key, std::uint32_t fallback,
                     std::uint32_t maximum = std::numeric_limits<std::uint32_t>::max()) {
    if (!config.contains(key)) return fallback;
    const auto& value = config.at(key);
    if (!value.is_number_integer() || (value.is_number_integer() && !value.is_number_unsigned() && value.get<std::int64_t>() < 0))
        throw std::invalid_argument(std::string(key) + " must be a nonnegative integer");
    auto result = value.get<std::uint64_t>();
    if (result > maximum) throw std::invalid_argument(std::string(key) + " is out of range");
    return static_cast<std::uint32_t>(result);
}
bool equivalent(const Type& a, const Type& b) {
    if (&a == &b) return true;
    if (a.kind != b.kind || a.size != b.size || a.alignment != b.alignment) return false;
    if (a.kind == Type::Kind::Array)
        return a.count == b.count && equivalent(*a.element, *b.element);
    if (a.kind == Type::Kind::Structure) {
        if (a.fields.size() != b.fields.size()) return false;
        for (std::size_t i = 0; i < a.fields.size(); ++i) {
            const auto& left = a.fields[i];
            const auto& right = b.fields[i];
            if (left.name != right.name || left.offset != right.offset || !equivalent(*left.type, *right.type)) return false;
        }
    }
    return true;
}
thread_local bool insideObserver = false;
void outsideObserver() {
    if (insideObserver) throw std::logic_error("queue synchronous I/O and observer registry changes outside callbacks");
}
struct Entry {
    std::uint64_t id;
    BeckhoffPLC::RawObserver callback;
    std::mutex mutex;
    std::condition_variable quiet;
    std::size_t active = 0;
    bool enabled = true;
    Entry(std::uint64_t token, BeckhoffPLC::RawObserver function) : id(token), callback(std::move(function)) {}
    void stop() {
        std::unique_lock<std::mutex> lock(mutex);
        enabled = false;
        quiet.wait(lock, [&] { return active == 0; });
    }
};
using Entries = std::vector<std::shared_ptr<Entry>>;
struct Events {
    std::mutex edit;
    std::shared_ptr<const Entries> observers = std::make_shared<const Entries>();
    std::atomic<std::uint64_t> errors{0};
    std::uint64_t next_id = 1;
    void call(Entry& entry, const std::string& name,
              const std::uint8_t* data, std::size_t size) noexcept {
        {
            std::lock_guard<std::mutex> lock(entry.mutex);
            if (!entry.enabled) return;
            ++entry.active;
        }
        const auto previous = insideObserver;
        insideObserver = true;
        try { entry.callback(name, data, size); } catch (...) { ++errors; }
        insideObserver = previous;
        {
            std::lock_guard<std::mutex> lock(entry.mutex);
            if (--entry.active == 0) entry.quiet.notify_all();
        }
    }
    void deliver(const std::string& name, const std::uint8_t* data, std::size_t size) noexcept {
        const auto snapshot = std::atomic_load(&observers);
        for (const auto& entry : *snapshot) call(*entry, name, data, size);
    }
};
}

struct BeckhoffPLC::Impl {
    TypeRegistry types;
    AdsOptions options;
    std::vector<Variable> variables;
    std::unordered_map<std::string, std::size_t> indices;
    std::vector<std::size_t> monitored;
    std::vector<std::uint32_t> handles;
    std::vector<std::uint32_t> notifications;
    std::unique_ptr<Transport> transport;
    std::shared_ptr<Events> events = std::make_shared<Events>();

    explicit Impl(const Json& config)
        : types(config.value("structures", Json::object()), number(config, "pack", 0)) {
        options.net_id = config.value("net_id", std::string{});
        options.client_net_id = config.value("client_net_id", std::string{});
        options.ip_address = config.value("ip_address", std::string{});
        // Support existing parameter files that omit the gateway/IP. An explicit
        // address is preferred when the AMS Net ID differs from the network IP.
        if (options.ip_address.empty()) {
            const auto last = options.net_id.rfind('.');
            if (last != std::string::npos) {
                const auto previous = options.net_id.rfind('.', last - 1);
                if (previous != std::string::npos) options.ip_address = options.net_id.substr(0, previous);
            }
        }
        options.port = static_cast<std::uint16_t>(number(config, "port", 851, 65535));
        options.timeout_ms = number(config, "timeout_ms", 1000);
        options.notification_cycle_us = number(config, "notification_cycle_us", 1000, UINT32_MAX / 10);
        options.notification_max_delay_us = number(config, "notification_max_delay_us", 0, UINT32_MAX / 10);
        if (!options.port || !options.timeout_ms) throw std::invalid_argument("port and timeout_ms must be positive");
        const auto& specs = config.at("variables");
        if (!specs.is_object()) throw std::invalid_argument("variables must be an object");
        variables.reserve(specs.size());
        for (auto it = specs.begin(); it != specs.end(); ++it) {
            if (it.key().empty() || it.key().find('\0') != std::string::npos)
                throw std::invalid_argument("variable names must be nonempty and contain no NUL");
            Variable variable;
            variable.owner_ = this;
            variable.name_ = it.key();
            variable.type_ = types.resolve(it.value());
            indices.emplace(it.key(), variables.size());
            variables.push_back(std::move(variable));
        }
        auto names = [&](const char* key, bool monitor) {
            const auto list = config.value(key, Json::array());
            if (!list.is_array()) throw std::invalid_argument(std::string(key) + " must be an array");
            std::unordered_set<std::string> seen;
            for (const auto& item : list) {
                if (!item.is_string()) throw std::invalid_argument(std::string(key) + " must contain names");
                const auto name = item.get<std::string>();
                if (!seen.insert(name).second) throw std::invalid_argument(std::string(key) + " contains duplicate " + name);
                const auto index = indices.at(name);
                if (monitor) monitored.push_back(index);
                else variables[index].writable_ = true;
            }
        };
        names("variables_to_monitor", true);
        names("variables_to_write", false);
        handles.reserve(variables.size());
        notifications.reserve(monitored.size());
    }
    void connect(std::unique_ptr<Transport> supplied) {
        transport = supplied ? std::move(supplied) : makeAdsTransport(options);
        for (auto& variable : variables) {
            const auto handle = transport->acquire(variable.name_);
            handles.push_back(handle);
            variable.handle_ = handle;
        }
        for (const auto index : monitored) {
            const auto& variable = variables[index];
            auto state = events;
            auto name = variable.name_;
            const auto size = variable.size();
            auto token = transport->subscribe(variable.handle_, size,
                [state, name, size](const std::uint8_t* data, std::size_t length) {
                    if (length != size || !data) { ++state->errors; return; }
                    state->deliver(name, data, length);
                });
            notifications.push_back(token);
        }
    }
    ~Impl() {
        if (!transport) return;
        for (auto it = notifications.rbegin(); it != notifications.rend(); ++it) transport->unsubscribe(*it);
        for (auto it = handles.rbegin(); it != handles.rend(); ++it) transport->release(*it);
    }
};

BeckhoffPLC::BeckhoffPLC(const Json& parameters, std::unique_ptr<Transport> transport)
    : impl_(std::make_unique<Impl>(configuration(parameters))) { impl_->connect(std::move(transport)); }
BeckhoffPLC::~BeckhoffPLC() = default;

std::unique_ptr<BeckhoffPLC> BeckhoffPLC::fromFile(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open PLC parameters: " + path);
    return std::make_unique<BeckhoffPLC>(Json::parse(input));
}
const BeckhoffPLC::Variable& BeckhoffPLC::variable(const std::string& name) const {
    return impl_->variables.at(impl_->indices.at(name));
}
Value BeckhoffPLC::makeValue(const Json& spec) const { return Value(impl_->types.resolve(spec)); }
Value BeckhoffPLC::makeVariable(const std::string& name) const { return Value(variable(name).type()); }
Json BeckhoffPLC::getVariable(const std::string& name) {
    const auto& symbol = variable(name);
    Value value(symbol.type());
    read(symbol, value.data(), value.size());
    return value.toJson();
}
void BeckhoffPLC::setVariable(const std::string& name, const Json& json) {
    const auto& symbol = variable(name);
    Value value(symbol.type(), json);
    write(symbol, value.data(), value.size());
}
void BeckhoffPLC::setVariable(const std::string& name, const Value& value) {
    const auto& symbol = variable(name);
    if (!equivalent(*symbol.type(), *value.type())) throw std::invalid_argument("value type differs from variable type");
    write(symbol, value.data(), value.size());
}
void BeckhoffPLC::read(const Variable& symbol, void* buffer, std::size_t size) {
    outsideObserver();
    if (symbol.owner_ != impl_.get() || !buffer || size != symbol.size())
        throw std::invalid_argument("read needs a local variable and an exact-size buffer");
    impl_->transport->read(symbol.handle_, buffer, size);
}
void BeckhoffPLC::write(const Variable& symbol, const void* buffer, std::size_t size) {
    outsideObserver();
    if (symbol.owner_ != impl_.get() || !buffer || size != symbol.size())
        throw std::invalid_argument("write needs a local variable and an exact-size buffer");
    if (!symbol.writable_) throw std::invalid_argument("variable is not in variables_to_write");
    impl_->transport->write(symbol.handle_, buffer, size);
}
BeckhoffPLC::ObserverId BeckhoffPLC::addRawObserver(RawObserver observer, bool initial) {
    outsideObserver();
    if (!observer) throw std::invalid_argument("observer must be callable");
    auto state = impl_->events;
    ObserverId id;
    std::shared_ptr<Entry> entry;
    {
        std::lock_guard<std::mutex> lock(state->edit);
        auto updated = std::make_shared<Entries>(*std::atomic_load(&state->observers));
        id = state->next_id++;
        entry = std::make_shared<Entry>(id, std::move(observer));
        updated->push_back(entry);
        std::atomic_store(&state->observers, std::shared_ptr<const Entries>(std::move(updated)));
    }
    try {
        if (initial) for (const auto index : impl_->monitored) {
            const auto& symbol = impl_->variables[index];
            Value value(symbol.type());
            read(symbol, value.data(), value.size());
            state->call(*entry, symbol.name(), value.data(), value.size());
        }
    } catch (...) { removeObserver(id); throw; }
    return id;
}
BeckhoffPLC::ObserverId BeckhoffPLC::addObserver(Observer observer, bool initial) {
    outsideObserver();
    if (!observer) throw std::invalid_argument("observer must be callable");
    auto types = std::make_shared<std::unordered_map<std::string, std::shared_ptr<const Type>>>();
    for (const auto index : impl_->monitored) {
        const auto& symbol = impl_->variables[index];
        types->emplace(symbol.name(), symbol.type());
    }
    return addRawObserver([types, observer = std::move(observer)](const std::string& name, const std::uint8_t* data, std::size_t size) {
        Value value(types->at(name));
        std::memcpy(value.data(), data, size);
        observer(name, value.toJson());
    }, initial);
}
void BeckhoffPLC::removeObserver(ObserverId id) {
    outsideObserver();
    auto state = impl_->events;
    std::lock_guard<std::mutex> lock(state->edit);
    auto updated = std::make_shared<Entries>(*std::atomic_load(&state->observers));
    const auto found = std::find_if(updated->begin(), updated->end(), [id](const auto& e) { return e->id == id; });
    if (found == updated->end()) return;
    auto entry = *found;
    updated->erase(found);
    std::atomic_store(&state->observers, std::shared_ptr<const Entries>(std::move(updated)));
    entry->stop();
}
void BeckhoffPLC::readInitialValues() {
    outsideObserver();
    for (const auto index : impl_->monitored) {
        const auto& symbol = impl_->variables[index];
        Value value(symbol.type());
        read(symbol, value.data(), value.size());
        impl_->events->deliver(symbol.name(), value.data(), value.size());
    }
}
std::uint64_t BeckhoffPLC::callbackErrors() const noexcept { return impl_->events->errors.load(); }
} // namespace beckhoff
