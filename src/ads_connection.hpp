#ifndef ADS_CONNECTION_HPP_
#define ADS_CONNECTION_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ads_protocol.hpp"

namespace beckhoff_ros
{
namespace ads
{

using Clock = std::chrono::steady_clock;

enum class LogLevel { Debug, Info, Warn, Error };
using LogFunction = std::function<void (LogLevel level, const std::string & message)>;

struct ConnectionOptions
{
  std::string host;  // IP address (or host name) of the PLC
  std::uint16_t port = kTcpPort;
  /// AMS Net Id of this computer; empty: the local IP address of the connection + ".1.1".
  std::string local_net_id;
  std::chrono::milliseconds connect_timeout{3000};
  /// Re-establish the connection when it is lost.
  bool reconnect = true;
  std::chrono::milliseconds reconnect_interval{1000};
  /// SCHED_FIFO priority of the receive thread if > 0.
  int receive_thread_priority = 0;
  /// CPU the receive thread runs on if >= 0.
  int receive_thread_cpu = -1;
};

/// Receives what the PLC sends to an AMS port of this client. The functions are called by the
/// receive thread: they must return quickly and must not wait for ADS responses.
class PortListener
{
public:
  virtual ~PortListener() = default;
  /// One sample of a device notification.
  virtual void onNotification(const NotificationSample & sample) = 0;
  /// The connection was lost, or established again with the given epoch.
  virtual void onConnectionChanged(bool connected, std::uint64_t epoch) = 0;
  virtual void onLog(LogLevel level, const std::string & message) = 0;
};

/// A request waiting for its response.
struct Ticket
{
  std::uint32_t slot = 0;
  std::uint32_t invoke_id = 0;  // 0: no request
  explicit operator bool() const {return invoke_id != 0;}
};

/// Called with the outcome of a request: its response (error is then the error code of the AMS
/// header) or a client error (ads_error::kTimeout, kNotConnected). It is called before wait()
/// returns, by the receive thread or by the thread detecting the error; `data` is only valid
/// during the call.
using ResponseHook =
  void (*)(void * context, std::uint32_t error, const std::uint8_t * data, std::size_t size);

/// AMS/TCP connection to a TwinCAT router.
///
/// Requests are pipelined: any number of threads can have requests in flight (up to kSlots),
/// several frames can be sent at once, and responses are matched to their request by invoke id.
/// A receive thread reads the responses and the device notifications. When the connection is
/// lost, the requests in flight fail with ads_error::kNotConnected and the connection is
/// re-established in the background (ConnectionOptions::reconnect); its epoch is then
/// incremented, so that frames built for the previous connection (with its symbol and
/// notification handles) are never sent on the new one.
class Connection
{
public:
  static constexpr std::size_t kSlots = 256;

  /// Connects to the PLC, or returns the connection already opened by this process with the
  /// same host and local AMS Net Id (TwinCAT accepts a single connection per route). Throws
  /// AdsError if the connection cannot be established. `log` reports problems found while
  /// opening (e.g. the receive thread priority cannot be set).
  static std::shared_ptr<Connection> open(
    const ConnectionOptions & options, const LogFunction & log = nullptr);

  ~Connection();
  Connection(const Connection &) = delete;
  Connection & operator=(const Connection &) = delete;

  const AmsNetId & localNetId() const {return local_net_id_;}
  const ConnectionOptions & options() const {return options_;}
  /// Incremented each time the connection is established.
  std::uint64_t epoch() const {return epoch_.load(std::memory_order_acquire);}
  bool connected() const {return connected_.load(std::memory_order_acquire);}
  /// True if called from the receive thread (which must not wait for responses).
  bool inReceiveThread() const
  {
    return std::this_thread::get_id() == receive_thread_id_.load(std::memory_order_relaxed);
  }

  /// Registers a listener and returns the AMS port it owns: the source port of the requests it
  /// sends, and the target port of the notifications it receives.
  std::uint16_t addPort(PortListener * listener);
  /// Unregisters a port and frees its requests in flight, without calling their hooks. Waits for
  /// running listener calls and hooks to return, so it must not be called from them.
  void removePort(std::uint16_t port);

  /// Reserves a slot for a request of `port` whose response is expected before `deadline`.
  /// Returns an invalid ticket if all the slots are used. The frame of the request must use
  /// the invoke id of the ticket.
  Ticket reserve(
    std::uint16_t port, Clock::time_point deadline, ResponseHook hook = nullptr,
    void * hook_context = nullptr);

  /// Sends complete frames (one or more), all at once, if the connection epoch is `epoch`.
  /// Returns false if it is not (or the connection is lost); the tickets of the frames must
  /// then be failed by the caller.
  bool send(const std::uint8_t * data, std::size_t size, std::uint64_t epoch);

  /// Completes a request with an error (e.g. it could not be sent).
  void fail(const Ticket & ticket, std::uint32_t error);

  /// Waits for the response of a request, until its deadline. Returns the error of the AMS
  /// header of the response, or ads_error::kTimeout / kNotConnected. The data of the response
  /// is then available with response() until the ticket is released.
  std::uint32_t wait(const Ticket & ticket);
  /// True if the request is completed (wait() would not block).
  bool completed(const Ticket & ticket);
  const std::vector<std::uint8_t> & response(const Ticket & ticket) const
  {
    return slots_[ticket.slot].data;
  }
  /// Frees the slot of a request; a response arriving later is ignored.
  void release(const Ticket & ticket);
  /// The response of the request will not be waited for: its slot is freed when it arrives,
  /// or when its deadline is reached. `hook` (if not null) replaces the hook of the request and
  /// is called with its outcome, possibly right away if it is already completed.
  void detach(const Ticket & ticket, ResponseHook hook = nullptr, void * hook_context = nullptr);

  /// Number of frames received with an unknown invoke id (e.g. after a timeout).
  std::uint64_t unexpectedResponses() const {return unexpected_responses_.load();}

private:
  enum class SlotState { Free, Pending, Completed };

  struct Slot
  {
    std::mutex mutex;
    std::condition_variable cv;
    SlotState state = SlotState::Free;
    bool detached = false;
    std::uint32_t generation = 0;
    std::uint32_t invoke_id = 0;
    std::uint16_t port = 0;
    std::uint32_t error = 0;
    Clock::time_point deadline;
    ResponseHook hook = nullptr;
    void * hook_context = nullptr;
    std::vector<std::uint8_t> data;
  };

  struct Port
  {
    std::uint16_t port;
    PortListener * listener;
  };

  explicit Connection(const ConnectionOptions & options);
  int connectSocket(std::string & error);
  void start(const LogFunction & log);
  void run();
  bool receive();
  void handleFrame(const std::uint8_t * frame, std::size_t size);
  void complete(
    Slot & slot, std::unique_lock<std::mutex> & lock, std::uint32_t error,
    const std::uint8_t * data, std::size_t size);
  void freeSlot(Slot & slot, std::unique_lock<std::mutex> & lock);
  void disconnect(const std::string & reason);
  void failAll(std::uint32_t error);
  void expireDetached();
  void broadcastConnection(bool connected, std::uint64_t epoch);
  void log(LogLevel level, const std::string & message);
  Slot * slotOf(const Ticket & ticket);

  const ConnectionOptions options_;
  AmsNetId local_net_id_;
  bool local_net_id_given_ = false;

  // socket: written by the receive thread under send_mutex_
  std::mutex send_mutex_;
  int fd_ = -1;
  std::atomic<std::uint64_t> epoch_{0};
  std::atomic<bool> connected_{false};

  std::array<Slot, kSlots> slots_;
  std::mutex free_mutex_;
  std::vector<std::uint32_t> free_slots_;
  std::atomic<std::size_t> detached_count_{0};

  std::mutex ports_mutex_;  // held while calling listeners
  std::vector<Port> ports_;

  // receive thread
  std::thread thread_;
  std::atomic<std::thread::id> receive_thread_id_{};
  std::atomic<bool> stopping_{false};
  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;
  std::vector<std::uint8_t> rx_;
  std::size_t rx_begin_ = 0;
  std::size_t rx_end_ = 0;
  std::atomic<std::uint64_t> unexpected_responses_{0};
};

/// Adds a route to this computer on a PLC (TwinCAT's "Add Route" dialog, or
/// AdsLib's AddRemoteRoute): the PLC at `plc_host` will accept connections from `route_net_id`
/// at `route_address` (an IP address or host name of this computer). Returns the ADS error
/// code (0 on success, e.g. 0x704 for wrong credentials, ads_error::kTimeout without answer).
std::uint32_t addRoute(
  const std::string & plc_host, const AmsNetId & route_net_id,
  const std::string & route_address, const std::string & route_name,
  const std::string & username, const std::string & password,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(5000),
  std::uint16_t udp_port = kUdpPort);

}  // namespace ads
}  // namespace beckhoff_ros

#endif  // ADS_CONNECTION_HPP_
