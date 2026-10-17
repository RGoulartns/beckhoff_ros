// The ADS client (src/ads_connection.hpp) against a scripted server.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ads_connection.hpp"
#include "ads_protocol.hpp"
#include "beckhoff_ros/ads_error.hpp"

namespace ads = beckhoff_ros::ads;
namespace ads_error = beckhoff_ros::ads_error;
using namespace std::chrono_literals;

namespace
{

std::string hex(const std::uint8_t * data, std::size_t size)
{
  static const char * const digits = "0123456789abcdef";
  std::string out;
  for (std::size_t i = 0; i < size; ++i) {
    out += digits[data[i] >> 4];
    out += digits[data[i] & 15];
  }
  return out;
}

std::string hex(const std::vector<std::uint8_t> & data) {return hex(data.data(), data.size());}

std::vector<std::uint8_t> u32s(std::initializer_list<std::uint32_t> values)
{
  std::vector<std::uint8_t> out(values.size() * 4);
  std::size_t i = 0;
  for (const auto value : values) {
    ads::put32(out.data() + 4 * i++, value);
  }
  return out;
}

struct Frame
{
  ads::AmsHeader header;
  std::vector<std::uint8_t> data;
  std::vector<std::uint8_t> raw;  // the whole AMS/TCP frame
};

/// A TCP server playing the PLC, driven by the test.
class FakePlc
{
public:
  FakePlc()
  {
    listener_ = ::socket(AF_INET, SOCK_STREAM, 0);
    const int one = 1;
    ::setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    EXPECT_EQ(::bind(listener_, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
    EXPECT_EQ(::listen(listener_, 4), 0);
    socklen_t length = sizeof(address);
    ::getsockname(listener_, reinterpret_cast<sockaddr *>(&address), &length);
    port_ = ntohs(address.sin_port);
  }

  ~FakePlc()
  {
    closeClient();
    ::close(listener_);
  }

  std::uint16_t port() const {return port_;}

  bool accept(std::chrono::milliseconds timeout = 3000ms)
  {
    pollfd entry{listener_, POLLIN, 0};
    if (::poll(&entry, 1, static_cast<int>(timeout.count())) <= 0) {
      return false;
    }
    closeClient();
    client_ = ::accept(listener_, nullptr, nullptr);
    return client_ >= 0;
  }

  void closeClient()
  {
    if (client_ >= 0) {
      ::shutdown(client_, SHUT_RDWR);
      ::close(client_);
      client_ = -1;
    }
  }

  bool receive(std::uint8_t * data, std::size_t size, std::chrono::milliseconds timeout)
  {
    while (size > 0) {
      pollfd entry{client_, POLLIN, 0};
      if (::poll(&entry, 1, static_cast<int>(timeout.count())) <= 0) {
        return false;
      }
      const ssize_t received = ::recv(client_, data, size, 0);
      if (received <= 0) {
        return false;
      }
      data += received;
      size -= static_cast<std::size_t>(received);
    }
    return true;
  }

  bool receiveFrame(Frame & frame, std::chrono::milliseconds timeout = 3000ms)
  {
    std::uint8_t tcp[6];
    if (!receive(tcp, sizeof(tcp), timeout)) {
      return false;
    }
    const std::uint32_t length = ads::get32(tcp + 2);
    frame.raw.assign(tcp, tcp + 6);
    frame.raw.resize(6 + length);
    if (!receive(frame.raw.data() + 6, length, timeout) || length < 32) {
      return false;
    }
    frame.header = ads::AmsHeader::decode(frame.raw.data() + 6);
    frame.data.assign(frame.raw.begin() + 38, frame.raw.end());
    return true;
  }

  static std::vector<std::uint8_t> frameOf(
    const ads::AmsHeader & header, const std::vector<std::uint8_t> & data)
  {
    std::vector<std::uint8_t> out(38 + data.size());
    ads::put16(out.data(), 0);
    ads::put32(out.data() + 2, static_cast<std::uint32_t>(32 + data.size()));
    ads::AmsHeader copy = header;
    copy.length = static_cast<std::uint32_t>(data.size());
    copy.encode(out.data() + 6);
    std::copy(data.begin(), data.end(), out.begin() + 38);
    return out;
  }

  /// The response to a request.
  static std::vector<std::uint8_t> responseTo(
    const Frame & request, const std::vector<std::uint8_t> & data, std::uint32_t error = 0)
  {
    ads::AmsHeader header;
    header.target = request.header.source;
    header.source = request.header.target;
    header.command = request.header.command;
    header.flags = 0x0005;
    header.error = error;
    header.invoke_id = request.header.invoke_id;
    return frameOf(header, data);
  }

  void sendRaw(const std::vector<std::uint8_t> & data)
  {
    ASSERT_EQ(
      ::send(client_, data.data(), data.size(), MSG_NOSIGNAL),
      static_cast<ssize_t>(data.size()));
  }

  void respond(
    const Frame & request, const std::vector<std::uint8_t> & data, std::uint32_t error = 0)
  {
    sendRaw(responseTo(request, data, error));
  }

private:
  int listener_ = -1;
  int client_ = -1;
  std::uint16_t port_ = 0;
};

/// Records what the connection reports to its port.
class Listener : public ads::PortListener
{
public:
  void onNotification(const ads::NotificationSample & sample) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    samples.push_back(
      std::to_string(sample.handle) + "@" + std::to_string(sample.timestamp) + "=" +
      hex(sample.data, sample.size));
    if (on_notification) {
      on_notification();
    }
    cv.notify_all();
  }
  void onConnectionChanged(bool connected, std::uint64_t epoch) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    changes.push_back((connected ? "connected " : "lost ") + std::to_string(epoch));
    cv.notify_all();
  }
  void onLog(ads::LogLevel, const std::string & message) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    logs.push_back(message);
  }

  template<typename Predicate>
  bool waitFor(Predicate predicate, std::chrono::milliseconds timeout = 3000ms)
  {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, timeout, predicate);
  }

  /// Copy of a list, taken under the lock.
  std::vector<std::string> get(const std::vector<std::string> & list)
  {
    std::lock_guard<std::mutex> lock(mutex);
    return list;
  }

  std::mutex mutex;
  std::condition_variable cv;
  std::vector<std::string> samples;
  std::vector<std::string> changes;
  std::vector<std::string> logs;
  std::function<void()> on_notification;
};

class ConnectionTest : public ::testing::Test
{
protected:
  void open(const std::string & local_net_id = "")
  {
    ads::ConnectionOptions options;
    options.host = "127.0.0.1";
    options.port = plc.port();
    options.local_net_id = local_net_id;
    options.reconnect_interval = 50ms;
    connection = ads::Connection::open(options);
    ASSERT_TRUE(plc.accept());
    port = connection->addPort(&listener);
  }

  void TearDown() override
  {
    if (connection) {
      connection->removePort(port);
    }
  }

  /// Builds and sends a Read request; returns its ticket.
  ads::Ticket sendRead(
    std::uint32_t group, std::uint32_t offset, std::uint32_t length,
    std::chrono::milliseconds timeout = 2000ms)
  {
    const ads::Ticket ticket = connection->reserve(port, ads::Clock::now() + timeout);
    EXPECT_TRUE(ticket);
    std::vector<std::uint8_t> frame;
    ads::FrameWriter writer(frame);
    writer.begin(target, source(), ads::kRead, ticket.invoke_id);
    writer.u32(group);
    writer.u32(offset);
    writer.u32(length);
    writer.end();
    EXPECT_TRUE(connection->send(frame.data(), frame.size(), connection->epoch()));
    return ticket;
  }

  ads::AmsAddress source() const {return ads::AmsAddress{connection->localNetId(), port};}

  FakePlc plc;
  Listener listener;
  std::shared_ptr<ads::Connection> connection;
  std::uint16_t port = 0;
  ads::AmsAddress target{{{192, 168, 1, 10, 1, 1}}, 851};
};

}  // namespace

TEST(AmsNetId, ParsesAndFormats)
{
  ads::AmsNetId net_id;
  ASSERT_TRUE(ads::AmsNetId::parse("192.168.1.10.1.1", net_id));
  EXPECT_EQ(net_id.toString(), "192.168.1.10.1.1");
  ASSERT_TRUE(ads::AmsNetId::parse("0.0.0.0.0.255", net_id));
  EXPECT_EQ(net_id.bytes[5], 255);
  for (const char * text : {
    "", "1.2.3.4.5", "1.2.3.4.5.6.7", "1.2.3.4.5.256", "1.2.3.4.5.", "a.2.3.4.5.6",
    "1..3.4.5.6", "1.2.3.4.5.0006", " 1.2.3.4.5.6"})
  {
    EXPECT_FALSE(ads::AmsNetId::parse(text, net_id)) << text;
  }
}

TEST(Notifications, AreParsed)
{
  // 2 stamps: the first with 2 samples, the second with 1
  std::vector<std::uint8_t> data = u32s({0, 2});
  const auto append = [&data](const std::vector<std::uint8_t> & bytes) {
    data.insert(data.end(), bytes.begin(), bytes.end());
  };
  std::vector<std::uint8_t> stamp(12);
  ads::put64(stamp.data(), 1000);
  ads::put32(stamp.data() + 8, 2);
  append(stamp);
  append(u32s({7, 2}));
  append({0xAA, 0xBB});
  append(u32s({8, 0}));
  ads::put64(stamp.data(), 2000);
  ads::put32(stamp.data() + 8, 1);
  append(stamp);
  append(u32s({9, 4, 0x01020304}));
  ads::put32(data.data(), static_cast<std::uint32_t>(data.size() - 4));

  std::vector<std::string> samples;
  const auto collect = [&samples](const ads::NotificationSample & sample) {
    samples.push_back(
      std::to_string(sample.handle) + "@" + std::to_string(sample.timestamp) + "=" +
      hex(sample.data, sample.size));
  };
  EXPECT_TRUE(ads::parseNotification(data.data(), data.size(), collect));
  EXPECT_EQ(samples, (std::vector<std::string>{"7@1000=aabb", "8@1000=", "9@2000=04030201"}));

  // shorter than its length: rejected
  samples.clear();
  data.resize(data.size() - 1);
  EXPECT_FALSE(ads::parseNotification(data.data(), data.size(), collect));
  EXPECT_TRUE(samples.empty());
  // a truncated sample: the complete samples before it are parsed
  ads::put32(data.data(), static_cast<std::uint32_t>(data.size() - 4));
  EXPECT_FALSE(ads::parseNotification(data.data(), data.size(), collect));
  EXPECT_EQ(samples, (std::vector<std::string>{"7@1000=aabb", "8@1000="}));
  EXPECT_FALSE(ads::parseNotification(data.data(), 7, collect));
}

TEST_F(ConnectionTest, DerivesTheLocalNetIdFromTheLocalAddress)
{
  open();
  EXPECT_EQ(connection->localNetId().toString(), "127.0.0.1.1.1");
  EXPECT_EQ(port, ads::kFirstLocalPort);
  EXPECT_TRUE(connection->connected());
  EXPECT_EQ(connection->epoch(), 1u);
}

TEST_F(ConnectionTest, UsesTheGivenLocalNetId)
{
  open("10.1.2.3.4.5");
  EXPECT_EQ(connection->localNetId().toString(), "10.1.2.3.4.5");
}

TEST_F(ConnectionTest, SendsAdsFrames)
{
  open("10.1.2.3.4.5");
  const ads::Ticket ticket = sendRead(0xF005, 0x1234, 6);
  Frame request;
  ASSERT_TRUE(plc.receiveFrame(request));
  // AMS/TCP header, AMS header (target, source, command 2, flags 4, length 12, error 0,
  // invoke id) and the read request (index group, offset, length)
  std::vector<std::uint8_t> invoke(4);
  ads::put32(invoke.data(), ticket.invoke_id);
  EXPECT_EQ(
    hex(request.raw),
    "00002c000000" "c0a8010a0101" "5303" "0a0102030405" "3075" "0200" "0400" "0c000000"
    "00000000" + hex(invoke) + "05f00000" "34120000" "06000000");

  plc.respond(request, {0, 0, 0, 0, 6, 0, 0, 0, 1, 2, 3, 4, 5, 6});
  EXPECT_EQ(connection->wait(ticket), 0u);
  EXPECT_EQ(hex(connection->response(ticket)), "0000000006000000010203040506");
  connection->release(ticket);
}

TEST_F(ConnectionTest, PipelinesRequests)
{
  open();
  // three requests sent at once, answered in another order
  std::vector<ads::Ticket> tickets;
  std::vector<std::uint8_t> frames;
  for (std::uint32_t i = 0; i < 3; ++i) {
    tickets.push_back(connection->reserve(port, ads::Clock::now() + 2s));
    ads::FrameWriter writer(frames);
    writer.begin(target, source(), ads::kRead, tickets.back().invoke_id);
    writer.u32(0xF005);
    writer.u32(i);
    writer.u32(4);
    writer.end();
  }
  ASSERT_TRUE(connection->send(frames.data(), frames.size(), connection->epoch()));
  std::vector<Frame> requests(3);
  for (auto & request : requests) {
    ASSERT_TRUE(plc.receiveFrame(request));
  }
  for (const int i : {2, 0, 1}) {
    plc.respond(requests[i], u32s({0, 4, 100u + i}));
  }
  for (std::uint32_t i = 0; i < 3; ++i) {
    EXPECT_EQ(connection->wait(tickets[i]), 0u);
    EXPECT_EQ(ads::get32(connection->response(tickets[i]).data() + 8), 100 + i);
    connection->release(tickets[i]);
  }
}

TEST_F(ConnectionTest, ReceivesFragmentedAndCoalescedFrames)
{
  open();
  const ads::Ticket first = sendRead(0xF005, 1, 4);
  const ads::Ticket second = sendRead(0xF005, 2, 4);
  Frame request1;
  Frame request2;
  ASSERT_TRUE(plc.receiveFrame(request1));
  ASSERT_TRUE(plc.receiveFrame(request2));
  // the first response byte by byte, the second in the same packet as its last byte
  const auto response1 = FakePlc::responseTo(request1, u32s({0, 4, 11}));
  const auto response2 = FakePlc::responseTo(request2, u32s({0, 4, 22}));
  for (std::size_t i = 0; i + 1 < response1.size(); ++i) {
    plc.sendRaw({response1[i]});
    std::this_thread::sleep_for(1ms);
  }
  std::vector<std::uint8_t> rest{response1.back()};
  rest.insert(rest.end(), response2.begin(), response2.end());
  plc.sendRaw(rest);
  EXPECT_EQ(connection->wait(first), 0u);
  EXPECT_EQ(connection->wait(second), 0u);
  EXPECT_EQ(ads::get32(connection->response(first).data() + 8), 11u);
  EXPECT_EQ(ads::get32(connection->response(second).data() + 8), 22u);
  connection->release(first);
  connection->release(second);
}

TEST_F(ConnectionTest, ReportsAmsErrors)
{
  open();
  const ads::Ticket ticket = sendRead(0xF005, 1, 4);
  Frame request;
  ASSERT_TRUE(plc.receiveFrame(request));
  plc.respond(request, {}, ads_error::kTargetPortNotFound);
  EXPECT_EQ(connection->wait(ticket), ads_error::kTargetPortNotFound);
  connection->release(ticket);
}

TEST_F(ConnectionTest, TimesOutAndIgnoresLateResponses)
{
  open();
  const auto start = ads::Clock::now();
  const ads::Ticket ticket = sendRead(0xF005, 1, 4, 100ms);
  Frame request;
  ASSERT_TRUE(plc.receiveFrame(request));
  EXPECT_FALSE(connection->completed(ticket));
  EXPECT_EQ(connection->wait(ticket), ads_error::kTimeout);
  EXPECT_GE(ads::Clock::now() - start, 100ms);
  EXPECT_TRUE(connection->completed(ticket));
  connection->release(ticket);

  plc.respond(request, u32s({0, 4, 1}));  // too late
  const ads::Ticket next = sendRead(0xF005, 2, 4);
  Frame next_request;
  ASSERT_TRUE(plc.receiveFrame(next_request));
  plc.respond(next_request, u32s({0, 4, 2}));
  EXPECT_EQ(connection->wait(next), 0u);
  EXPECT_EQ(ads::get32(connection->response(next).data() + 8), 2u);
  connection->release(next);
  EXPECT_EQ(connection->unexpectedResponses(), 1u);
}

TEST_F(ConnectionTest, ReconnectsWhenTheConnectionIsLost)
{
  open();
  const ads::Ticket ticket = sendRead(0xF005, 1, 4);
  Frame request;
  ASSERT_TRUE(plc.receiveFrame(request));
  const std::uint64_t epoch = connection->epoch();
  plc.closeClient();
  EXPECT_EQ(connection->wait(ticket), ads_error::kNotConnected);
  connection->release(ticket);
  EXPECT_TRUE(listener.waitFor([this]() {return !listener.changes.empty();}));
  EXPECT_EQ(listener.get(listener.changes).front(), "lost 1");

  // frames of the previous connection are never sent on the new one
  ASSERT_TRUE(plc.accept());
  EXPECT_TRUE(listener.waitFor([this]() {return listener.changes.size() == 2;}));
  EXPECT_EQ(listener.get(listener.changes).back(), "connected 2");
  EXPECT_TRUE(connection->connected());
  std::vector<std::uint8_t> frame(38, 0);
  EXPECT_FALSE(connection->send(frame.data(), frame.size(), epoch));
  const ads::Ticket next = sendRead(0xF005, 2, 4);
  Frame next_request;
  ASSERT_TRUE(plc.receiveFrame(next_request));
  plc.respond(next_request, u32s({0, 4, 5}));
  EXPECT_EQ(connection->wait(next), 0u);
  connection->release(next);
}

TEST_F(ConnectionTest, DispatchesNotificationsToTheirPort)
{
  open();
  Listener other;
  const std::uint16_t other_port = connection->addPort(&other);
  EXPECT_EQ(other_port, ads::kFirstLocalPort + 1);

  std::vector<std::uint8_t> data = u32s({0, 1});
  std::vector<std::uint8_t> stamp(12);
  ads::put64(stamp.data(), 1234);
  ads::put32(stamp.data() + 8, 2);
  data.insert(data.end(), stamp.begin(), stamp.end());
  for (const std::uint32_t handle : {5u, 6u}) {
    const auto sample = u32s({handle, 2});
    data.insert(data.end(), sample.begin(), sample.end());
    data.push_back(static_cast<std::uint8_t>(handle));
    data.push_back(0);
  }
  ads::put32(data.data(), static_cast<std::uint32_t>(data.size() - 4));
  ads::AmsHeader header;
  header.target = ads::AmsAddress{connection->localNetId(), other_port};
  header.source = target;
  header.command = ads::kNotification;
  header.flags = 0x0004;
  plc.sendRaw(FakePlc::frameOf(header, data));
  ASSERT_TRUE(other.waitFor([&other]() {return other.samples.size() == 2;}));
  EXPECT_EQ(other.get(other.samples), (std::vector<std::string>{"5@1234=0500", "6@1234=0600"}));
  EXPECT_TRUE(listener.get(listener.samples).empty());
  connection->removePort(other_port);

  // malformed: ignored after the valid samples, with a warning
  header.target.port = port;
  data.resize(data.size() - 1);
  ads::put32(data.data(), static_cast<std::uint32_t>(data.size() - 4));
  plc.sendRaw(FakePlc::frameOf(header, data));
  ASSERT_TRUE(listener.waitFor([this]() {return listener.samples.size() == 1;}));
  EXPECT_TRUE(listener.waitFor([this]() {return !listener.logs.empty();}));
}

TEST_F(ConnectionTest, CallsTheHooksOfDetachedRequests)
{
  open();
  struct Outcome
  {
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<std::string> calls;
  } outcome;
  const ads::ResponseHook hook = [](void * context, std::uint32_t error,
    const std::uint8_t * data, std::size_t size) {
    auto * result = static_cast<Outcome *>(context);
    std::lock_guard<std::mutex> lock(result->mutex);
    result->calls.push_back(std::to_string(error) + ":" + hex(data, size));
    result->cv.notify_all();
  };
  const ads::Ticket answered = sendRead(0xF005, 1, 4);
  const ads::Ticket expiring = sendRead(0xF005, 2, 4, 200ms);
  connection->detach(answered, hook, &outcome);
  connection->detach(expiring, hook, &outcome);
  Frame request;
  ASSERT_TRUE(plc.receiveFrame(request));
  plc.respond(request, u32s({0, 4, 3}));
  std::unique_lock<std::mutex> lock(outcome.mutex);
  ASSERT_TRUE(outcome.cv.wait_for(lock, 2s, [&outcome]() {return outcome.calls.size() == 2;}));
  EXPECT_EQ(outcome.calls[0], "0:000000000400000003000000");
  EXPECT_EQ(outcome.calls[1], std::to_string(ads_error::kTimeout) + ":");
  lock.unlock();

  // a request detached after its completion
  const ads::Ticket completed = sendRead(0xF005, 3, 4);
  Frame request3;
  ASSERT_TRUE(plc.receiveFrame(request3));
  ASSERT_TRUE(plc.receiveFrame(request3));  // (the request of `expiring` came first)
  plc.respond(request3, u32s({0, 4, 9}));
  while (!connection->completed(completed)) {
    std::this_thread::sleep_for(1ms);
  }
  connection->detach(completed, hook, &outcome);
  EXPECT_EQ(outcome.calls.size(), 3u);
}

TEST_F(ConnectionTest, DoesNotWaitInTheReceiveThread)
{
  open();
  const ads::Ticket pending = sendRead(0xF005, 1, 4);
  std::atomic<std::uint32_t> result{0};
  listener.on_notification = [&]() {result = connection->wait(pending);};
  ads::AmsHeader header;
  header.target = source();
  header.source = target;
  header.command = ads::kNotification;
  header.flags = 0x0004;
  // length, 1 stamp: timestamp 0, 1 sample: handle 77, no data
  plc.sendRaw(FakePlc::frameOf(header, u32s({24, 1, 0, 0, 1, 77, 0})));
  ASSERT_TRUE(listener.waitFor([this]() {return listener.samples.size() == 1;}));
  EXPECT_EQ(result.load(), ads_error::kClientError);
  connection->release(pending);
}

TEST_F(ConnectionTest, SharesTheConnectionToAPlc)
{
  open();
  ads::ConnectionOptions options;
  options.host = "127.0.0.1";
  options.port = plc.port();
  EXPECT_EQ(ads::Connection::open(options), connection);
}

TEST_F(ConnectionTest, HandlesManyThreads)
{
  open();
  std::atomic<bool> serving{true};
  std::thread server([&]() {
      Frame request;
      while (serving && plc.receiveFrame(request, 200ms)) {
        // echo the offset as the value read
        plc.respond(request, u32s({0, 4, ads::get32(request.data.data() + 4)}));
      }
    });
  std::atomic<int> failures{0};
  std::vector<std::thread> clients;
  for (int t = 0; t < 8; ++t) {
    clients.emplace_back(
      [&, t]() {
        for (std::uint32_t i = 0; i < 300; ++i) {
          const std::uint32_t value = static_cast<std::uint32_t>(t) * 1000 + i;
          const ads::Ticket ticket = sendRead(0xF005, value, 4);
          if (connection->wait(ticket) != 0 ||
          ads::get32(connection->response(ticket).data() + 8) != value)
          {
            ++failures;
          }
          connection->release(ticket);
        }
      });
  }
  for (auto & client : clients) {
    client.join();
  }
  serving = false;
  server.join();
  EXPECT_EQ(failures.load(), 0);
}

TEST(Route, IsAddedOverUdp)
{
  // a UDP server playing the PLC
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ASSERT_EQ(::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
  socklen_t length = sizeof(address);
  ::getsockname(fd, reinterpret_cast<sockaddr *>(&address), &length);
  const std::uint16_t udp_port = ntohs(address.sin_port);

  std::string request_hex;
  std::thread plc([&]() {
      std::uint8_t buffer[1024];
      sockaddr_in client{};
      socklen_t client_length = sizeof(client);
      const ssize_t size = ::recvfrom(
        fd, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr *>(&client), &client_length);
      request_hex = hex(buffer, static_cast<std::size_t>(size));
      // cookie, invoke id, service 0x80000006, AMS address, 1 tag: result 0x704
      std::vector<std::uint8_t> response = u32s({0x71146603, 0, 0x80000006});
      const std::uint8_t ams_address[8] = {192, 168, 1, 10, 1, 1, 0, 0};
      response.insert(response.end(), ams_address, ams_address + 8);
      const auto tags = u32s({1, 0x00040001, 0x704});
      response.insert(response.end(), tags.begin(), tags.end());
      ::sendto(
        fd, response.data(), response.size(), 0,
        reinterpret_cast<sockaddr *>(&client), client_length);
    });
  ads::AmsNetId net_id;
  ads::AmsNetId::parse("192.168.1.20.1.1", net_id);
  const std::uint32_t result = ads::addRoute(
    "127.0.0.1", net_id, "192.168.1.20", "route", "Administrator", "1", 2000ms, udp_port);
  plc.join();
  ::close(fd);
  EXPECT_EQ(result, 0x704u);
  // the same bytes as AdsLib's AddRemoteRoute
  EXPECT_EQ(
    request_hex,
    "03661471" "00000000" "06000000" "c0a801140101" "0000" "05000000"
    "0c00" "0600" "726f75746500"
    "0700" "0600" "c0a801140101"
    "0d00" "0e00" "41646d696e6973747261746f7200"
    "0200" "0200" "3100"
    "0500" "0d00" "3139322e3136382e312e323000");
}

TEST(Route, TimesOutWithoutAnswer)
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ASSERT_EQ(::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
  socklen_t length = sizeof(address);
  ::getsockname(fd, reinterpret_cast<sockaddr *>(&address), &length);
  ads::AmsNetId net_id;
  EXPECT_EQ(
    ads::addRoute("127.0.0.1", net_id, "a", "", "u", "p", 100ms, ntohs(address.sin_port)),
    ads_error::kTimeout);
  ::close(fd);
}
