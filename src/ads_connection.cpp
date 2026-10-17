#include "ads_connection.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "beckhoff_ros/ads_error.hpp"

namespace beckhoff_ros
{
namespace ads
{

namespace
{

static_assert(Connection::kSlots == 256, "the invoke id holds the slot index in its low byte");

constexpr std::size_t kInitialReceiveBuffer = 64 * 1024;
// The receive thread wakes up at least that often, to expire detached requests.
constexpr int kReceiveTimeoutMs = 50;
// Dead connections are detected after a few seconds, with or without traffic.
constexpr int kKeepAliveIdleS = 2;
constexpr int kKeepAliveIntervalS = 1;
constexpr int kKeepAliveCount = 3;
constexpr unsigned kUserTimeoutMs = 5000;
constexpr int kSendTimeoutMs = 3000;

std::string errnoText(int error)
{
  char buffer[256] = {0};
  // GNU strerror_r returns a char *, POSIX's returns an int: support both
  const auto result = strerror_r(error, buffer, sizeof(buffer));
  if constexpr (std::is_same<decltype(result), char *>::value) {
    return result;
  } else {
    return result == 0 ? std::string(buffer) : "error " + std::to_string(error);
  }
}

bool sendAll(int fd, const std::uint8_t * data, std::size_t size)
{
  while (size > 0) {
    const ssize_t sent = ::send(fd, data, size, MSG_NOSIGNAL);
    if (sent < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    data += sent;
    size -= static_cast<std::size_t>(sent);
  }
  return true;
}

void setOption(int fd, int level, int name, int value)
{
  // best effort: the connection works without these options
  (void)setsockopt(fd, level, name, &value, sizeof(value));
}

}  // namespace

std::shared_ptr<Connection> Connection::open(
  const ConnectionOptions & options, const LogFunction & log)
{
  static std::mutex mutex;
  static std::map<std::string, std::weak_ptr<Connection>> connections;

  const std::string key =
    options.host + ":" + std::to_string(options.port) + "/" + options.local_net_id;
  std::lock_guard<std::mutex> lock(mutex);
  for (auto it = connections.begin(); it != connections.end(); ) {
    it = it->second.expired() ? connections.erase(it) : std::next(it);
  }
  auto & entry = connections[key];
  if (auto existing = entry.lock()) {
    return existing;
  }
  std::shared_ptr<Connection> connection(new Connection(options));
  connection->start(log);
  entry = connection;
  return connection;
}

Connection::Connection(const ConnectionOptions & options)
: options_(options)
{
  if (!options_.local_net_id.empty()) {
    if (!AmsNetId::parse(options_.local_net_id, local_net_id_)) {
      throw std::invalid_argument("invalid AMS Net Id '" + options_.local_net_id + "'");
    }
    local_net_id_given_ = true;
  }
  free_slots_.reserve(kSlots);
  for (std::size_t i = kSlots; i > 0; --i) {
    free_slots_.push_back(static_cast<std::uint32_t>(i - 1));
  }
  rx_.resize(kInitialReceiveBuffer);
}

Connection::~Connection()
{
  stopping_.store(true);
  {
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (fd_ >= 0) {
      ::shutdown(fd_, SHUT_RDWR);  // wakes the receive thread up
    }
  }
  {
    std::lock_guard<std::mutex> lock(stop_mutex_);
  }
  stop_cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int Connection::connectSocket(std::string & error)
{
  addrinfo hints{};
  hints.ai_family = AF_INET;  // the AMS Net Id of this computer is made of its IPv4 address
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;
  addrinfo * addresses = nullptr;
  const std::string service = std::to_string(options_.port);
  const int status = ::getaddrinfo(options_.host.c_str(), service.c_str(), &hints, &addresses);
  if (status != 0) {
    error = "unknown host '" + options_.host + "': " + ::gai_strerror(status);
    return -1;
  }
  std::unique_ptr<addrinfo, decltype(& ::freeaddrinfo)> guard(addresses, &::freeaddrinfo);

  error = "no address";
  for (const addrinfo * address = addresses; address != nullptr; address = address->ai_next) {
    const int fd = ::socket(address->ai_family, address->ai_socktype | SOCK_CLOEXEC, 0);
    if (fd < 0) {
      error = errnoText(errno);
      continue;
    }
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    bool connected = ::connect(fd, address->ai_addr, address->ai_addrlen) == 0;
    if (!connected && errno != EINPROGRESS) {
      error = errnoText(errno);
      ::close(fd);
      continue;
    }
    if (!connected) {
      // wait for the connection, checking regularly if the connection is being destroyed
      const auto deadline = Clock::now() + options_.connect_timeout;
      error = "timeout";
      while (!stopping_.load()) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
          deadline - Clock::now()).count();
        if (remaining <= 0) {
          break;
        }
        pollfd entry{fd, POLLOUT, 0};
        const int ready = ::poll(&entry, 1, static_cast<int>(std::min<long long>(remaining, 100)));
        if (ready > 0) {
          int socket_error = 0;
          socklen_t length = sizeof(socket_error);
          if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &length) != 0) {
            socket_error = errno;
          }
          connected = socket_error == 0;
          error = connected ? "" : errnoText(socket_error);
          break;
        }
        if (ready < 0 && errno != EINTR) {
          error = errnoText(errno);
          break;
        }
      }
      if (!connected) {
        ::close(fd);
        continue;
      }
    }
    ::fcntl(fd, F_SETFL, flags);

    // requests are small and must not wait (several requests can be sent back to back)
    setOption(fd, IPPROTO_TCP, TCP_NODELAY, 1);
    setOption(fd, SOL_SOCKET, SO_KEEPALIVE, 1);
#ifdef TCP_KEEPIDLE
    setOption(fd, IPPROTO_TCP, TCP_KEEPIDLE, kKeepAliveIdleS);
    setOption(fd, IPPROTO_TCP, TCP_KEEPINTVL, kKeepAliveIntervalS);
    setOption(fd, IPPROTO_TCP, TCP_KEEPCNT, kKeepAliveCount);
#endif
#ifdef TCP_USER_TIMEOUT
    setOption(fd, IPPROTO_TCP, TCP_USER_TIMEOUT, static_cast<int>(kUserTimeoutMs));
#endif
    timeval receive_timeout{0, kReceiveTimeoutMs * 1000};
    (void)setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout));
    timeval send_timeout{kSendTimeoutMs / 1000, (kSendTimeoutMs % 1000) * 1000};
    (void)setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
    return fd;
  }
  return -1;
}

void Connection::start(const LogFunction & log)
{
  std::string error;
  const int fd = connectSocket(error);
  if (fd < 0) {
    throw AdsError(
            "Unable to connect to the PLC at " + options_.host + ":" +
            std::to_string(options_.port) + " (" + error + ")", ads_error::kNotConnected);
  }
  if (!local_net_id_given_) {
    // like AdsLib: the local IP address of the connection, followed by ".1.1"
    sockaddr_in local{};
    socklen_t length = sizeof(local);
    if (::getsockname(fd, reinterpret_cast<sockaddr *>(&local), &length) != 0 ||
      local.sin_family != AF_INET)
    {
      ::close(fd);
      throw AdsError(
              "Unable to get the local address of the connection to the PLC",
              ads_error::kClientError);
    }
    const std::uint32_t ip = ntohl(local.sin_addr.s_addr);
    local_net_id_.bytes = {
      static_cast<std::uint8_t>(ip >> 24), static_cast<std::uint8_t>(ip >> 16),
      static_cast<std::uint8_t>(ip >> 8), static_cast<std::uint8_t>(ip), 1, 1};
  }
  fd_ = fd;
  epoch_.store(1);
  connected_.store(true);
  thread_ = std::thread(&Connection::run, this);

  if (options_.receive_thread_priority > 0) {
    sched_param parameters{};
    parameters.sched_priority = options_.receive_thread_priority;
    const int result = pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &parameters);
    if (result != 0 && log) {
      log(
        LogLevel::Warn, "Unable to set the priority of the ADS receive thread to SCHED_FIFO " +
        std::to_string(options_.receive_thread_priority) + ": " + errnoText(result) +
        " (it needs CAP_SYS_NICE or an rtprio limit, see README)");
    }
  }
  if (options_.receive_thread_cpu >= 0) {
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(options_.receive_thread_cpu, &cpus);
    const int result = pthread_setaffinity_np(thread_.native_handle(), sizeof(cpus), &cpus);
    if (result != 0 && log) {
      log(
        LogLevel::Warn, "Unable to run the ADS receive thread on CPU " +
        std::to_string(options_.receive_thread_cpu) + ": " + errnoText(result));
    }
  }
}

std::uint16_t Connection::addPort(PortListener * listener)
{
  std::lock_guard<std::mutex> lock(ports_mutex_);
  std::uint16_t port = kFirstLocalPort;
  while (std::any_of(
      ports_.begin(), ports_.end(), [port](const Port & entry) {return entry.port == port;}))
  {
    ++port;
  }
  ports_.push_back(Port{port, listener});
  return port;
}

void Connection::removePort(std::uint16_t port)
{
  {
    std::lock_guard<std::mutex> lock(ports_mutex_);
    ports_.erase(
      std::remove_if(
        ports_.begin(), ports_.end(), [port](const Port & entry) {return entry.port == port;}),
      ports_.end());
  }
  for (auto & slot : slots_) {
    std::unique_lock<std::mutex> lock(slot.mutex);
    if (slot.state != SlotState::Free && slot.port == port) {
      freeSlot(slot, lock);
    }
  }
}

Connection::Slot * Connection::slotOf(const Ticket & ticket)
{
  if (!ticket || ticket.slot >= kSlots) {
    return nullptr;
  }
  return &slots_[ticket.slot];
}

Ticket Connection::reserve(
  std::uint16_t port, Clock::time_point deadline, ResponseHook hook, void * hook_context)
{
  std::uint32_t index = 0;
  {
    std::lock_guard<std::mutex> lock(free_mutex_);
    if (free_slots_.empty()) {
      return Ticket{};
    }
    index = free_slots_.back();
    free_slots_.pop_back();
  }
  Slot & slot = slots_[index];
  std::lock_guard<std::mutex> lock(slot.mutex);
  slot.generation = slot.generation % 0xFFFFFFu + 1;  // 1 .. 2^24 - 1: never 0
  slot.invoke_id = (slot.generation << 8) | index;
  slot.state = SlotState::Pending;
  slot.detached = false;
  slot.port = port;
  slot.error = 0;
  slot.deadline = deadline;
  slot.hook = hook;
  slot.hook_context = hook_context;
  slot.data.clear();
  return Ticket{index, slot.invoke_id};
}

bool Connection::send(const std::uint8_t * data, std::size_t size, std::uint64_t epoch)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (fd_ < 0 || epoch != epoch_.load(std::memory_order_relaxed)) {
    return false;
  }
  if (!sendAll(fd_, data, size)) {
    // a frame may have been sent partially: the stream is lost, the receive thread reconnects
    ::shutdown(fd_, SHUT_RDWR);
    return false;
  }
  return true;
}

void Connection::complete(
  Slot & slot, std::unique_lock<std::mutex> & lock, std::uint32_t error,
  const std::uint8_t * data, std::size_t size)
{
  slot.error = error;
  if (size > 0) {
    slot.data.assign(data, data + size);
  } else {
    slot.data.clear();
  }
  if (slot.hook != nullptr) {
    slot.hook(slot.hook_context, error, data, size);
  }
  slot.state = SlotState::Completed;
  if (slot.detached) {
    freeSlot(slot, lock);
  } else {
    slot.cv.notify_all();
  }
}

void Connection::freeSlot(Slot & slot, std::unique_lock<std::mutex> & lock)
{
  if (slot.detached) {
    detached_count_.fetch_sub(1);
  }
  slot.state = SlotState::Free;
  slot.detached = false;
  slot.invoke_id = 0;
  slot.hook = nullptr;
  slot.hook_context = nullptr;
  const auto index = static_cast<std::uint32_t>(&slot - slots_.data());
  slot.cv.notify_all();  // a misused ticket must not wait forever
  lock.unlock();
  std::lock_guard<std::mutex> free_lock(free_mutex_);
  free_slots_.push_back(index);
}

void Connection::fail(const Ticket & ticket, std::uint32_t error)
{
  Slot * slot = slotOf(ticket);
  if (slot == nullptr) {
    return;
  }
  std::unique_lock<std::mutex> lock(slot->mutex);
  if (slot->invoke_id == ticket.invoke_id && slot->state == SlotState::Pending) {
    complete(*slot, lock, error, nullptr, 0);
  }
}

std::uint32_t Connection::wait(const Ticket & ticket)
{
  Slot * slot = slotOf(ticket);
  if (slot == nullptr) {
    return ads_error::kClientInvalidParameter;
  }
  std::unique_lock<std::mutex> lock(slot->mutex);
  if (slot->invoke_id != ticket.invoke_id) {
    return ads_error::kNotConnected;
  }
  if (slot->state == SlotState::Pending && inReceiveThread()) {
    // the response could only be read by this thread
    complete(*slot, lock, ads_error::kClientError, nullptr, 0);
    return slot->error;
  }
  slot->cv.wait_until(
    lock, slot->deadline, [&]() {
      return slot->state != SlotState::Pending || slot->invoke_id != ticket.invoke_id;
    });
  if (slot->invoke_id != ticket.invoke_id) {
    return ads_error::kNotConnected;  // freed by removePort()
  }
  if (slot->state == SlotState::Pending) {
    complete(*slot, lock, ads_error::kTimeout, nullptr, 0);
  }
  return slot->error;
}

bool Connection::completed(const Ticket & ticket)
{
  Slot * slot = slotOf(ticket);
  if (slot == nullptr) {
    return true;
  }
  std::lock_guard<std::mutex> lock(slot->mutex);
  return slot->invoke_id != ticket.invoke_id || slot->state != SlotState::Pending ||
         Clock::now() >= slot->deadline;
}

void Connection::release(const Ticket & ticket)
{
  Slot * slot = slotOf(ticket);
  if (slot == nullptr) {
    return;
  }
  std::unique_lock<std::mutex> lock(slot->mutex);
  if (slot->invoke_id == ticket.invoke_id && slot->state != SlotState::Free) {
    freeSlot(*slot, lock);
  }
}

void Connection::detach(const Ticket & ticket, ResponseHook hook, void * hook_context)
{
  Slot * slot = slotOf(ticket);
  if (slot == nullptr) {
    return;
  }
  std::unique_lock<std::mutex> lock(slot->mutex);
  if (slot->invoke_id != ticket.invoke_id || slot->state == SlotState::Free) {
    return;
  }
  if (hook != nullptr) {
    slot->hook = hook;
    slot->hook_context = hook_context;
  }
  if (slot->state == SlotState::Completed) {
    if (hook != nullptr) {
      hook(hook_context, slot->error, slot->data.data(), slot->data.size());
    }
    freeSlot(*slot, lock);
  } else if (!slot->detached) {
    slot->detached = true;
    detached_count_.fetch_add(1);
  }
}

void Connection::run()
{
  receive_thread_id_.store(std::this_thread::get_id());
  auto last_expiry = Clock::now();
  bool reconnect_failed = false;
  while (!stopping_.load()) {
    if (!connected_.load()) {
      {
        std::unique_lock<std::mutex> lock(stop_mutex_);
        stop_cv_.wait_for(
          lock, options_.reconnect ? options_.reconnect_interval : std::chrono::hours(24),
          [this]() {return stopping_.load();});
      }
      if (stopping_.load() || !options_.reconnect) {
        continue;
      }
      std::string error;
      const int fd = connectSocket(error);
      if (fd < 0) {
        if (!reconnect_failed) {
          log(LogLevel::Info, "Unable to reconnect to the PLC (" + error + "), retrying");
          reconnect_failed = true;
        }
        continue;
      }
      reconnect_failed = false;
      std::uint64_t epoch = 0;
      {
        std::lock_guard<std::mutex> lock(send_mutex_);
        fd_ = fd;
        epoch = epoch_.fetch_add(1) + 1;
        connected_.store(true);
      }
      rx_begin_ = rx_end_ = 0;
      log(LogLevel::Info, "Reconnected to the PLC at " + options_.host);
      broadcastConnection(true, epoch);
      continue;
    }
    receive();
    if (detached_count_.load() > 0) {
      const auto now = Clock::now();
      if (now - last_expiry >= std::chrono::milliseconds(kReceiveTimeoutMs)) {
        last_expiry = now;
        expireDetached();
      }
    }
  }
  if (connected_.load()) {
    disconnect("connection closed");
  }
}

bool Connection::receive()
{
  // room for at least a small frame
  if (rx_.size() - rx_end_ < 4096) {
    if (rx_begin_ > 0) {
      std::memmove(rx_.data(), rx_.data() + rx_begin_, rx_end_ - rx_begin_);
      rx_end_ -= rx_begin_;
      rx_begin_ = 0;
    }
    if (rx_.size() - rx_end_ < 4096) {
      rx_.resize(rx_.size() * 2);
    }
  }
  const ssize_t received = ::recv(fd_, rx_.data() + rx_end_, rx_.size() - rx_end_, 0);
  if (received <= 0) {
    if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      return true;  // timeout: nothing received
    }
    disconnect(received == 0 ? "connection closed by the PLC" : errnoText(errno));
    return false;
  }
#ifdef TCP_QUICKACK
  // acknowledge immediately: the PLC may wait for it before sending the next response
  setOption(fd_, IPPROTO_TCP, TCP_QUICKACK, 1);
#endif
  rx_end_ += static_cast<std::size_t>(received);

  while (rx_end_ - rx_begin_ >= kTcpHeaderSize) {
    const std::uint8_t * frame = rx_.data() + rx_begin_;
    const std::size_t length = get32(frame + 2);
    if (length > kMaxFrameSize) {
      disconnect("invalid AMS/TCP frame (length " + std::to_string(length) + ")");
      return false;
    }
    const std::size_t total = kTcpHeaderSize + length;
    if (rx_end_ - rx_begin_ < total) {
      if (rx_.size() - rx_begin_ < total) {  // make room for the whole frame
        std::memmove(rx_.data(), rx_.data() + rx_begin_, rx_end_ - rx_begin_);
        rx_end_ -= rx_begin_;
        rx_begin_ = 0;
        if (rx_.size() < total) {
          rx_.resize(total);
        }
      }
      break;
    }
    if (get16(frame) == 0) {  // other values are commands of a local AMS router: ignored
      handleFrame(frame + kTcpHeaderSize, length);
    }
    rx_begin_ += total;
  }
  if (rx_begin_ == rx_end_) {
    rx_begin_ = rx_end_ = 0;
  }
  return true;
}

void Connection::handleFrame(const std::uint8_t * frame, std::size_t size)
{
  if (size < kAmsHeaderSize) {
    log(LogLevel::Warn, "Ignoring a frame too short for an AMS header");
    return;
  }
  const AmsHeader header = AmsHeader::decode(frame);
  if (header.length > size - kAmsHeaderSize) {
    log(LogLevel::Warn, "Ignoring a frame shorter than the length of its AMS header");
    return;
  }
  const std::uint8_t * data = frame + kAmsHeaderSize;

  if ((header.flags & kFlagResponse) != 0) {
    Slot & slot = slots_[header.invoke_id % kSlots];
    std::unique_lock<std::mutex> lock(slot.mutex);
    if (header.invoke_id == 0 || slot.invoke_id != header.invoke_id ||
      slot.state != SlotState::Pending)
    {
      unexpected_responses_.fetch_add(1);  // e.g. a request that timed out
      return;
    }
    complete(slot, lock, header.error, data, header.length);
    return;
  }

  if (header.command == kNotification) {
    bool valid = true;
    {
      std::lock_guard<std::mutex> lock(ports_mutex_);
      const auto port = std::find_if(
        ports_.begin(), ports_.end(),
        [&header](const Port & entry) {return entry.port == header.target.port;});
      if (port == ports_.end()) {
        return;
      }
      PortListener * listener = port->listener;
      valid = parseNotification(
        data, header.length,
        [listener](const NotificationSample & sample) {listener->onNotification(sample);});
    }
    if (!valid) {
      log(LogLevel::Warn, "Ignoring the rest of a malformed device notification");
    }
  }
  // other requests of the PLC are not expected
}

void Connection::disconnect(const std::string & reason)
{
  {
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (fd_ >= 0) {
      ::shutdown(fd_, SHUT_RDWR);
      ::close(fd_);
      fd_ = -1;
    }
    connected_.store(false);
  }
  rx_begin_ = rx_end_ = 0;
  failAll(ads_error::kNotConnected);
  if (!stopping_.load()) {
    log(
      LogLevel::Warn, "Connection to the PLC lost (" + reason + ")" +
      (options_.reconnect ? ", reconnecting" : ""));
    broadcastConnection(false, epoch_.load());
  }
}

void Connection::failAll(std::uint32_t error)
{
  for (auto & slot : slots_) {
    std::unique_lock<std::mutex> lock(slot.mutex);
    if (slot.state == SlotState::Pending) {
      complete(slot, lock, error, nullptr, 0);
    }
  }
}

void Connection::expireDetached()
{
  const auto now = Clock::now();
  for (auto & slot : slots_) {
    std::unique_lock<std::mutex> lock(slot.mutex);
    if (slot.state == SlotState::Pending && slot.detached && now >= slot.deadline) {
      complete(slot, lock, ads_error::kTimeout, nullptr, 0);
    }
  }
}

void Connection::broadcastConnection(bool connected, std::uint64_t epoch)
{
  std::lock_guard<std::mutex> lock(ports_mutex_);
  for (const auto & port : ports_) {
    port.listener->onConnectionChanged(connected, epoch);
  }
}

void Connection::log(LogLevel level, const std::string & message)
{
  std::lock_guard<std::mutex> lock(ports_mutex_);
  for (const auto & port : ports_) {
    port.listener->onLog(level, message);
  }
}

}  // namespace ads
}  // namespace beckhoff_ros
