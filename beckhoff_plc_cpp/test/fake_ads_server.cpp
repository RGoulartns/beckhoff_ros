#include "fake_ads_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

#include "beckhoff_plc/config.hpp"

namespace beckhoff {
namespace test {

namespace {

// AMS command ids
constexpr std::uint16_t kReadDeviceInfo = 1;
constexpr std::uint16_t kRead = 2;
constexpr std::uint16_t kWrite = 3;
constexpr std::uint16_t kReadState = 4;
constexpr std::uint16_t kWriteControl = 5;
constexpr std::uint16_t kAddNotification = 6;
constexpr std::uint16_t kDelNotification = 7;
constexpr std::uint16_t kDeviceNotification = 8;
constexpr std::uint16_t kReadWrite = 9;

// Index groups
constexpr std::uint32_t kSymHandleByName = 0xF003;
constexpr std::uint32_t kSymValueByHandle = 0xF005;
constexpr std::uint32_t kSymReleaseHandle = 0xF006;
constexpr std::uint32_t kSymInfoByNameEx = 0xF009;
constexpr std::uint32_t kSumRead = 0xF080;
constexpr std::uint32_t kSumWrite = 0xF081;

// ADS return codes
constexpr std::uint32_t kErrTargetPort = 0x6;
constexpr std::uint32_t kErrServiceNotSupported = 0x701;
constexpr std::uint32_t kErrInvalidGroup = 0x702;
constexpr std::uint32_t kErrInvalidSize = 0x705;
constexpr std::uint32_t kErrSymbolNotFound = 0x710;
constexpr std::uint32_t kErrNotificationHandle = 0x714;

constexpr std::uint32_t kTransServerCycle = 3;
constexpr auto kInitialNotificationDelay = std::chrono::milliseconds(2);  // TwinCAT: next PLC cycle

using NetId = std::array<std::uint8_t, 6>;
using Bytes = std::vector<std::uint8_t>;

std::uint16_t get16(const std::uint8_t* p) { return static_cast<std::uint16_t>(p[0] | (p[1] << 8)); }
std::uint32_t get32(const std::uint8_t* p) {
  return static_cast<std::uint32_t>(p[0]) | static_cast<std::uint32_t>(p[1]) << 8 |
         static_cast<std::uint32_t>(p[2]) << 16 | static_cast<std::uint32_t>(p[3]) << 24;
}
void put16(Bytes& out, std::uint16_t v) {
  out.push_back(static_cast<std::uint8_t>(v));
  out.push_back(static_cast<std::uint8_t>(v >> 8));
}
void put32(Bytes& out, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
}
void put64(Bytes& out, std::uint64_t v) {
  for (int i = 0; i < 8; ++i) out.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
}
std::uint32_t size32(std::size_t size) { return static_cast<std::uint32_t>(size); }

std::string lower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

std::uint64_t fileTimeNow() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return 116444736000000000ULL + static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() / 100);
}

struct AmsHeader {
  NetId targetNetId{};
  std::uint16_t targetPort = 0;
  NetId sourceNetId{};
  std::uint16_t sourcePort = 0;
  std::uint16_t command = 0;
  std::uint16_t flags = 0;
  std::uint32_t length = 0;
  std::uint32_t error = 0;
  std::uint32_t invokeId = 0;

  static AmsHeader parse(const std::uint8_t* p) {
    AmsHeader h;
    std::copy(p, p + 6, h.targetNetId.begin());
    h.targetPort = get16(p + 6);
    std::copy(p + 8, p + 14, h.sourceNetId.begin());
    h.sourcePort = get16(p + 14);
    h.command = get16(p + 16);
    h.flags = get16(p + 18);
    h.length = get32(p + 20);
    h.error = get32(p + 24);
    h.invokeId = get32(p + 28);
    return h;
  }
};

/// AMS/TCP frame: 6 byte TCP header, 32 byte AMS header, data.
Bytes makeFrame(const NetId& targetNetId, std::uint16_t targetPort, const NetId& sourceNetId, std::uint16_t sourcePort,
                std::uint16_t command, std::uint16_t flags, std::uint32_t error, std::uint32_t invokeId,
                const Bytes& data) {
  Bytes frame;
  frame.reserve(38 + data.size());
  put16(frame, 0);
  put32(frame, size32(32 + data.size()));
  frame.insert(frame.end(), targetNetId.begin(), targetNetId.end());
  put16(frame, targetPort);
  frame.insert(frame.end(), sourceNetId.begin(), sourceNetId.end());
  put16(frame, sourcePort);
  put16(frame, command);
  put16(frame, flags);
  put32(frame, size32(data.size()));
  put32(frame, error);
  put32(frame, invokeId);
  frame.insert(frame.end(), data.begin(), data.end());
  return frame;
}

}  // namespace

struct FakeAdsServer::Impl {
  struct Symbol {
    std::string name;
    std::string type;
    Bytes data;
  };
  struct Connection {
    int fd = -1;
    std::mutex writeMutex;
    std::atomic<bool> open{true};
    std::thread thread;
  };
  struct Notification {
    std::shared_ptr<Connection> connection;
    NetId clientNetId{};
    std::uint16_t clientPort = 0;
    NetId serverNetId{};
    std::uint16_t serverPort = 0;
    std::string symbol;  // key in `symbols`
    std::uint32_t length = 0;
    std::uint32_t mode = 0;
    std::chrono::steady_clock::duration cycle{};
    std::chrono::steady_clock::time_point next{};
    bool initialPending = true;
  };
  using Outgoing = std::vector<std::pair<std::shared_ptr<Connection>, Bytes>>;

  explicit Impl(Options o) : options(std::move(o)) {}

  Options options;
  mutable std::mutex mutex;
  std::unordered_map<std::string, Symbol> symbols;  // key: lower-case name
  std::unordered_map<std::uint32_t, std::string> handles;  // empty string: invalidated
  std::map<std::uint32_t, Notification> notifications;
  std::vector<std::shared_ptr<Connection>> connectionList;
  std::uint32_t nextHandle = 0x1000;
  std::uint32_t nextNotification = 0x100;
  std::array<std::size_t, 16> counts{};
  std::size_t total = 0;

  int listenFd = -1;
  std::atomic<bool> running{true};
  std::thread acceptThread;
  std::thread timerThread;
  std::condition_variable timerCv;

  void start() {
    listenFd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd < 0) throw std::runtime_error("socket() failed");
    const int one = 1;
    ::setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(options.tcpPort);
    if (::inet_pton(AF_INET, options.address.c_str(), &address.sin_addr) != 1) {
      ::close(listenFd);
      throw std::runtime_error("invalid address " + options.address);
    }
    if (::bind(listenFd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
        ::listen(listenFd, 16) != 0) {
      const std::string error = std::strerror(errno);
      ::close(listenFd);
      throw std::runtime_error("cannot listen on " + options.address + ":" + std::to_string(options.tcpPort) + ": " +
                               error);
    }
    acceptThread = std::thread([this] { acceptLoop(); });
    timerThread = std::thread([this] { timerLoop(); });
  }

  void stop() {
    running = false;
    timerCv.notify_all();
    if (acceptThread.joinable()) acceptThread.join();
    if (timerThread.joinable()) timerThread.join();
    ::close(listenFd);
    std::vector<std::shared_ptr<Connection>> list;
    {
      std::lock_guard<std::mutex> lock(mutex);
      list = connectionList;
    }
    for (auto& c : list) {
      c->open = false;
      ::shutdown(c->fd, SHUT_RDWR);
    }
    for (auto& c : list) {
      if (c->thread.joinable()) c->thread.join();
      ::close(c->fd);
    }
  }

  void acceptLoop() {
    while (running) {
      pollfd p{listenFd, POLLIN, 0};
      if (::poll(&p, 1, 50) <= 0) continue;
      const int fd = ::accept(listenFd, nullptr, nullptr);
      if (fd < 0) continue;
      const int one = 1;
      ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
      auto connection = std::make_shared<Connection>();
      connection->fd = fd;
      std::lock_guard<std::mutex> lock(mutex);
      connectionList.push_back(connection);
      connection->thread = std::thread([this, connection] { serve(connection); });
    }
  }

  static bool readFully(int fd, std::uint8_t* data, std::size_t size) {
    while (size > 0) {
      const ssize_t n = ::recv(fd, data, size, 0);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) return false;
      data += n;
      size -= static_cast<std::size_t>(n);
    }
    return true;
  }

  static void send(Connection& c, const Bytes& frame) {
    std::lock_guard<std::mutex> lock(c.writeMutex);
    std::size_t sent = 0;
    while (c.open && sent < frame.size()) {
      const ssize_t n = ::send(c.fd, frame.data() + sent, frame.size() - sent, MSG_NOSIGNAL);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) {
        c.open = false;
        return;
      }
      sent += static_cast<std::size_t>(n);
    }
  }

  void serve(const std::shared_ptr<Connection>& c) {
    Bytes buffer;
    while (running && c->open) {
      std::uint8_t tcpHeader[6];
      if (!readFully(c->fd, tcpHeader, sizeof(tcpHeader))) break;
      const std::uint32_t length = get32(tcpHeader + 2);
      if (length < 32 || length > 64 * 1024 * 1024) break;
      buffer.resize(length);
      if (!readFully(c->fd, buffer.data(), length)) break;
      const AmsHeader h = AmsHeader::parse(buffer.data());
      if (h.length != length - 32) break;
      if (h.flags & 0x0001) continue;  // a response: nothing to do

      Outgoing outgoing;
      std::uint32_t amsError = 0;
      const Bytes response = process(h, buffer.data() + 32, c, outgoing, amsError);
      send(*c, makeFrame(h.sourceNetId, h.sourcePort, h.targetNetId, h.targetPort, h.command, 0x0005, amsError,
                         h.invokeId, response));
      sendAll(outgoing);
    }
    c->open = false;
    ::shutdown(c->fd, SHUT_RDWR);
    std::lock_guard<std::mutex> lock(mutex);
    for (auto it = notifications.begin(); it != notifications.end();) {
      it = it->second.connection == c ? notifications.erase(it) : std::next(it);
    }
  }

  static void sendAll(const Outgoing& outgoing) {
    for (const auto& [connection, frame] : outgoing) send(*connection, frame);
  }

  // ---- requests (called with `mutex` held)

  Bytes process(const AmsHeader& h, const std::uint8_t* d, const std::shared_ptr<Connection>& connection,
                Outgoing& outgoing, std::uint32_t& amsError) {
    std::lock_guard<std::mutex> lock(mutex);
    ++total;
    if (h.command < counts.size()) ++counts[h.command];
    Bytes out;
    if (h.targetPort != options.adsPort) {
      amsError = kErrTargetPort;
      return out;
    }
    const std::uint32_t n = h.length;
    switch (h.command) {
      case kReadDeviceInfo: {
        put32(out, 0);
        out.push_back(3);  // version 3.1.4024
        out.push_back(1);
        put16(out, 4024);
        const char name[16] = "FakeAdsServer";
        out.insert(out.end(), name, name + sizeof(name));
        return out;
      }
      case kReadState:
        put32(out, 0);
        put16(out, 5);  // RUN
        put16(out, 0);
        return out;
      case kWriteControl:
        put32(out, 0);
        return out;
      case kRead: {
        Bytes payload;
        std::uint32_t result = kErrInvalidSize;
        if (n >= 12) {
          const std::uint32_t group = get32(d), offset = get32(d + 4), length = get32(d + 8);
          result = readValue(group, offset, length, payload);
        }
        put32(out, result);
        put32(out, size32(payload.size()));
        out.insert(out.end(), payload.begin(), payload.end());
        return out;
      }
      case kWrite: {
        std::uint32_t result = kErrInvalidSize;
        if (n >= 12 && n >= 12 + get32(d + 8)) result = write(get32(d), get32(d + 4), d + 12, get32(d + 8), outgoing);
        put32(out, result);
        return out;
      }
      case kReadWrite: {
        Bytes payload;
        std::uint32_t result = kErrInvalidSize;
        if (n >= 16 && n >= 16 + get32(d + 12)) {
          const std::uint32_t readLength = get32(d + 8);
          result = readWrite(get32(d), get32(d + 4), d + 16, get32(d + 12), payload, outgoing);
          if (result == 0 && payload.size() > readLength) {
            result = kErrInvalidSize;
            payload.clear();
          }
        }
        put32(out, result);
        put32(out, size32(payload.size()));
        out.insert(out.end(), payload.begin(), payload.end());
        return out;
      }
      case kAddNotification: {
        std::uint32_t result = kErrInvalidSize;
        std::uint32_t id = 0;
        if (n >= 24) {
          result = addNotification(h, get32(d), get32(d + 4), get32(d + 8), get32(d + 12), get32(d + 20), connection,
                                   id);
        }
        put32(out, result);
        put32(out, id);
        return out;
      }
      case kDelNotification: {
        std::uint32_t result = kErrInvalidSize;
        if (n >= 4) result = notifications.erase(get32(d)) ? 0 : kErrNotificationHandle;
        put32(out, result);
        return out;
      }
      default:
        put32(out, kErrServiceNotSupported);
        return out;
    }
  }

  Symbol* symbolByHandle(std::uint32_t handle) {
    const auto it = handles.find(handle);
    if (it == handles.end() || it->second.empty()) return nullptr;
    const auto symbol = symbols.find(it->second);
    return symbol == symbols.end() ? nullptr : &symbol->second;
  }

  std::uint32_t readValue(std::uint32_t group, std::uint32_t offset, std::uint32_t length, Bytes& payload) {
    if (group != kSymValueByHandle) return kErrInvalidGroup;
    const Symbol* s = symbolByHandle(offset);
    if (!s) return kErrSymbolNotFound;
    if (length > s->data.size()) return kErrInvalidSize;
    payload.assign(s->data.begin(), s->data.begin() + length);
    return 0;
  }

  std::uint32_t write(std::uint32_t group, std::uint32_t offset, const std::uint8_t* data, std::uint32_t length,
                      Outgoing& outgoing) {
    if (group == kSymReleaseHandle) {
      if (length < 4) return kErrInvalidSize;
      return handles.erase(get32(data)) ? 0 : kErrSymbolNotFound;
    }
    if (group != kSymValueByHandle) return kErrInvalidGroup;
    Symbol* s = symbolByHandle(offset);
    if (!s) return kErrSymbolNotFound;
    if (length != s->data.size()) return kErrInvalidSize;
    store(*s, Bytes(data, data + length), outgoing);
    return 0;
  }

  std::uint32_t readWrite(std::uint32_t group, std::uint32_t offset, const std::uint8_t* data, std::uint32_t length,
                          Bytes& payload, Outgoing& outgoing) {
    if (group == kSymHandleByName || group == kSymInfoByNameEx) {
      std::string name(reinterpret_cast<const char*>(data), length);
      while (!name.empty() && name.back() == '\0') name.pop_back();
      const auto it = symbols.find(lower(name));
      if (it == symbols.end()) return kErrSymbolNotFound;
      if (group == kSymHandleByName) {
        const std::uint32_t handle = nextHandle++;
        handles[handle] = it->first;
        put32(payload, handle);
        return 0;
      }
      const Symbol& s = it->second;
      put32(payload, size32(30 + s.name.size() + 1 + s.type.size() + 1 + 1));
      put32(payload, 0x4040);  // index group of PLC memory
      put32(payload, 0);
      put32(payload, size32(s.data.size()));
      put32(payload, 65);  // ADST_BIGTYPE
      put32(payload, 0);
      put16(payload, static_cast<std::uint16_t>(s.name.size()));
      put16(payload, static_cast<std::uint16_t>(s.type.size()));
      put16(payload, 0);
      payload.insert(payload.end(), s.name.begin(), s.name.end());
      payload.push_back(0);
      payload.insert(payload.end(), s.type.begin(), s.type.end());
      payload.push_back(0);
      payload.push_back(0);
      return 0;
    }
    if (group == kSumRead || group == kSumWrite) {
      if (!options.sumCommands) return kErrServiceNotSupported;
      const std::uint32_t count = offset;
      if (length < 12ULL * count) return kErrInvalidSize;
      Bytes results;
      Bytes values;
      std::size_t dataOffset = 12ULL * count;
      for (std::uint32_t i = 0; i < count; ++i) {
        const std::uint32_t subGroup = get32(data + 12 * i);
        const std::uint32_t subOffset = get32(data + 12 * i + 4);
        const std::uint32_t subLength = get32(data + 12 * i + 8);
        if (group == kSumRead) {
          Bytes value;
          const std::uint32_t result = readValue(subGroup, subOffset, subLength, value);
          value.resize(subLength);  // failed reads still take their room
          put32(results, result);
          values.insert(values.end(), value.begin(), value.end());
        } else {
          if (dataOffset + subLength > length) return kErrInvalidSize;
          put32(results, write(subGroup, subOffset, data + dataOffset, subLength, outgoing));
          dataOffset += subLength;
        }
      }
      payload = results;
      payload.insert(payload.end(), values.begin(), values.end());
      return 0;
    }
    return kErrInvalidGroup;
  }

  std::uint32_t addNotification(const AmsHeader& h, std::uint32_t group, std::uint32_t offset, std::uint32_t length,
                                std::uint32_t mode, std::uint32_t cycleTime,
                                const std::shared_ptr<Connection>& connection, std::uint32_t& id) {
    if (group != kSymValueByHandle) return kErrInvalidGroup;
    const auto handle = handles.find(offset);
    if (handle == handles.end() || handle->second.empty()) return kErrSymbolNotFound;
    if (length > symbols.at(handle->second).data.size()) return kErrInvalidSize;
    Notification notification;
    notification.connection = connection;
    notification.clientNetId = h.sourceNetId;
    notification.clientPort = h.sourcePort;
    notification.serverNetId = h.targetNetId;
    notification.serverPort = h.targetPort;
    notification.symbol = handle->second;
    notification.length = length;
    notification.mode = mode;
    notification.cycle = std::max<std::chrono::steady_clock::duration>(std::chrono::nanoseconds(100ULL * cycleTime),
                                                                       std::chrono::milliseconds(1));
    notification.next = std::chrono::steady_clock::now() + kInitialNotificationDelay;
    id = nextNotification++;
    notifications.emplace(id, std::move(notification));
    timerCv.notify_all();
    return 0;
  }

  Bytes notificationFrame(std::uint32_t id, const Notification& n) const {
    const Bytes& value = symbols.at(n.symbol).data;
    Bytes data;
    put32(data, 24 + n.length);  // length of what follows
    put32(data, 1);              // stamps
    put64(data, fileTimeNow());
    put32(data, 1);  // samples
    put32(data, id);
    put32(data, n.length);
    data.insert(data.end(), value.begin(), value.begin() + n.length);
    return makeFrame(n.clientNetId, n.clientPort, n.serverNetId, n.serverPort, kDeviceNotification, 0x0004, 0, 0, data);
  }

  void store(Symbol& symbol, Bytes value, Outgoing& outgoing) {
    if (symbol.data == value) return;
    symbol.data = std::move(value);
    const std::string key = lower(symbol.name);
    for (const auto& [id, n] : notifications) {
      if (n.symbol == key && n.mode != kTransServerCycle && !n.initialPending) {
        outgoing.emplace_back(n.connection, notificationFrame(id, n));
      }
    }
  }

  /// Sends the initial notifications (shortly after they were added) and the
  /// cyclic ones.
  void timerLoop() {
    std::unique_lock<std::mutex> lock(mutex);
    while (running) {
      const auto now = std::chrono::steady_clock::now();
      auto wake = now + std::chrono::milliseconds(50);
      Outgoing outgoing;
      for (auto& [id, n] : notifications) {
        if (now >= n.next && (n.initialPending || n.mode == kTransServerCycle)) {
          outgoing.emplace_back(n.connection, notificationFrame(id, n));
          n.initialPending = false;
          n.next = n.mode == kTransServerCycle ? std::max(n.next + n.cycle, now) : now + std::chrono::hours(24);
        }
        if (n.initialPending || n.mode == kTransServerCycle) wake = std::min(wake, n.next);
      }
      if (!outgoing.empty()) {
        lock.unlock();
        sendAll(outgoing);
        lock.lock();
        continue;
      }
      timerCv.wait_until(lock, wake);
    }
  }
};

FakeAdsServer::FakeAdsServer() : FakeAdsServer(Options{}) {}

FakeAdsServer::FakeAdsServer(Options options) : impl_(std::make_unique<Impl>(std::move(options))) { impl_->start(); }

FakeAdsServer::~FakeAdsServer() { impl_->stop(); }

void FakeAdsServer::addSymbol(const std::string& name, std::size_t size, const std::string& typeName) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->symbols[lower(name)] = Impl::Symbol{name, typeName, Bytes(size, 0)};
}

void FakeAdsServer::addSymbols(const Config& config) {
  const TypeRegistry types = config.createTypeRegistry();
  for (const VariableDefinition& variable : config.variables) {
    const TypePtr type = types.parse(variable.type);
    addSymbol(variable.name, type->size(), type->name());
  }
}

std::vector<std::uint8_t> FakeAdsServer::value(const std::string& name) const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->symbols.at(lower(name)).data;
}

void FakeAdsServer::setValue(const std::string& name, const std::vector<std::uint8_t>& value) {
  Impl::Outgoing outgoing;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl::Symbol& symbol = impl_->symbols.at(lower(name));
    if (value.size() != symbol.data.size()) throw std::invalid_argument("wrong size for " + name);
    impl_->store(symbol, value, outgoing);
  }
  Impl::sendAll(outgoing);
}

void FakeAdsServer::invalidateHandles() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  for (auto& handle : impl_->handles) handle.second.clear();
}

void FakeAdsServer::restart() {
  std::vector<std::shared_ptr<Impl::Connection>> list;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    list = impl_->connectionList;
    impl_->handles.clear();
    impl_->notifications.clear();
  }
  for (auto& c : list) {
    c->open = false;
    ::shutdown(c->fd, SHUT_RDWR);
  }
}

void FakeAdsServer::setSumCommands(bool enabled) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->options.sumCommands = enabled;
}

std::size_t FakeAdsServer::requests(std::uint16_t command) const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return command < impl_->counts.size() ? impl_->counts[command] : 0;
}

std::size_t FakeAdsServer::requests() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->total;
}

std::size_t FakeAdsServer::openHandles() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->handles.size();
}

std::size_t FakeAdsServer::activeNotifications() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->notifications.size();
}

std::size_t FakeAdsServer::connections() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return static_cast<std::size_t>(std::count_if(impl_->connectionList.begin(), impl_->connectionList.end(),
                                                [](const auto& c) { return c->open.load(); }));
}

}  // namespace test
}  // namespace beckhoff
