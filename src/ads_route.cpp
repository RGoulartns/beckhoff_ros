// Route management over UDP (port 48899), as done by AdsLib's AddRemoteRoute.

#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ads_connection.hpp"
#include "ads_protocol.hpp"
#include "beckhoff_ros/ads_error.hpp"

namespace beckhoff_ros
{
namespace ads
{

namespace
{

constexpr std::uint32_t kUdpCookie = 0x71146603;
constexpr std::uint32_t kServiceAddRoute = 6;
constexpr std::uint32_t kServiceResponse = 0x80000000;

constexpr std::uint16_t kTagPassword = 2;
constexpr std::uint16_t kTagComputerName = 5;
constexpr std::uint16_t kTagNetId = 7;
constexpr std::uint16_t kTagRouteName = 12;
constexpr std::uint16_t kTagUserName = 13;
constexpr std::uint16_t kTagResult = 1;

void append16(std::vector<std::uint8_t> & out, std::uint16_t value)
{
  out.resize(out.size() + 2);
  put16(out.data() + out.size() - 2, value);
}

void append32(std::vector<std::uint8_t> & out, std::uint32_t value)
{
  out.resize(out.size() + 4);
  put32(out.data() + out.size() - 4, value);
}

void appendTag(std::vector<std::uint8_t> & out, std::uint16_t tag, const std::string & text)
{
  append16(out, tag);
  append16(out, static_cast<std::uint16_t>(text.size() + 1));
  out.insert(out.end(), text.begin(), text.end());
  out.push_back(0);
}

struct SocketGuard
{
  int fd;
  ~SocketGuard() {::close(fd);}
};

void appendTag(std::vector<std::uint8_t> & out, std::uint16_t tag, const AmsNetId & net_id)
{
  append16(out, tag);
  append16(out, static_cast<std::uint16_t>(net_id.bytes.size()));
  out.insert(out.end(), net_id.bytes.begin(), net_id.bytes.end());
}

}  // namespace

std::uint32_t addRoute(
  const std::string & plc_host, const AmsNetId & route_net_id,
  const std::string & route_address, const std::string & route_name,
  const std::string & username, const std::string & password,
  std::chrono::milliseconds timeout, std::uint16_t udp_port)
{
  for (const std::string * text : {&route_address, &route_name, &username, &password}) {
    if (text->size() >= 0xFFFF) {
      return ads_error::kInvalidParameter;
    }
  }
  std::vector<std::uint8_t> request;
  append32(request, kUdpCookie);
  append32(request, 0);  // invoke id
  append32(request, kServiceAddRoute);
  request.insert(request.end(), route_net_id.bytes.begin(), route_net_id.bytes.end());
  append16(request, 0);  // AMS port
  append32(request, 5);  // number of tags
  appendTag(request, kTagRouteName, route_name.empty() ? route_address : route_name);
  appendTag(request, kTagNetId, route_net_id);
  appendTag(request, kTagUserName, username);
  appendTag(request, kTagPassword, password);
  appendTag(request, kTagComputerName, route_address);

  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  addrinfo * addresses = nullptr;
  if (::getaddrinfo(
      plc_host.c_str(), std::to_string(udp_port).c_str(), &hints, &addresses) != 0)
  {
    return ads_error::kTargetMachineNotFound;
  }
  std::unique_ptr<addrinfo, decltype(& ::freeaddrinfo)> guard(addresses, &::freeaddrinfo);
  const int fd = ::socket(addresses->ai_family, addresses->ai_socktype | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    return ads_error::kClientError;
  }
  const SocketGuard socket_guard{fd};

  if (::sendto(
      fd, request.data(), request.size(), 0, addresses->ai_addr, addresses->ai_addrlen) !=
    static_cast<ssize_t>(request.size()))
  {
    return ads_error::kClientError;
  }

  const auto deadline = Clock::now() + timeout;
  std::vector<std::uint8_t> response(2048);
  while (true) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - Clock::now()).count();
    if (remaining <= 0) {
      return ads_error::kTimeout;
    }
    pollfd entry{fd, POLLIN, 0};
    if (::poll(&entry, 1, static_cast<int>(remaining)) <= 0) {
      continue;
    }
    const ssize_t size = ::recv(fd, response.data(), response.size(), 0);
    if (size < 12 || get32(response.data()) != kUdpCookie || get32(response.data() + 4) != 0 ||
      get32(response.data() + 8) != (kServiceResponse | kServiceAddRoute))
    {
      continue;  // not the answer to this request
    }
    // AMS address of the PLC, number of tags, tags
    std::size_t pos = 12 + 8;
    if (static_cast<std::size_t>(size) < pos + 4) {
      return ads_error::kInvalidResponse;
    }
    std::uint32_t count = get32(response.data() + pos);
    pos += 4;
    while (count-- > 0 && static_cast<std::size_t>(size) >= pos + 4) {
      const std::uint16_t tag = get16(response.data() + pos);
      const std::uint16_t length = get16(response.data() + pos + 2);
      pos += 4;
      if (static_cast<std::size_t>(size) < pos + length) {
        break;
      }
      if (tag == kTagResult && length == 4) {
        return get32(response.data() + pos);
      }
      pos += length;
    }
    return ads_error::kInvalidResponse;
  }
}

}  // namespace ads
}  // namespace beckhoff_ros
