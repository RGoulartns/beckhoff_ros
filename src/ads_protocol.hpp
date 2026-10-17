#ifndef ADS_PROTOCOL_HPP_
#define ADS_PROTOCOL_HPP_

// ADS/AMS protocol definitions (see Beckhoff's "ADS specification"): AMS/TCP framing, AMS
// header, ADS commands and the index groups used by this package. All the fields are little
// endian, whatever the host.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace beckhoff_ros
{
namespace ads
{

inline void put16(std::uint8_t * out, std::uint16_t value)
{
  out[0] = static_cast<std::uint8_t>(value);
  out[1] = static_cast<std::uint8_t>(value >> 8);
}

inline void put32(std::uint8_t * out, std::uint32_t value)
{
  for (int i = 0; i < 4; ++i) {
    out[i] = static_cast<std::uint8_t>(value >> (8 * i));
  }
}

inline void put64(std::uint8_t * out, std::uint64_t value)
{
  for (int i = 0; i < 8; ++i) {
    out[i] = static_cast<std::uint8_t>(value >> (8 * i));
  }
}

inline std::uint16_t get16(const std::uint8_t * in)
{
  return static_cast<std::uint16_t>(in[0] | (in[1] << 8));
}

inline std::uint32_t get32(const std::uint8_t * in)
{
  std::uint32_t value = 0;
  for (int i = 3; i >= 0; --i) {
    value = (value << 8) | in[i];
  }
  return value;
}

inline std::uint64_t get64(const std::uint8_t * in)
{
  std::uint64_t value = 0;
  for (int i = 7; i >= 0; --i) {
    value = (value << 8) | in[i];
  }
  return value;
}

constexpr std::uint16_t kTcpPort = 48898;    // AMS/TCP
constexpr std::uint16_t kUdpPort = 48899;    // route management
constexpr std::uint16_t kPlcPort = 851;      // TwinCAT 3 PLC runtime 1
constexpr std::uint16_t kFirstLocalPort = 30000;  // AMS ports of this client (like AdsLib)

// ADS commands
constexpr std::uint16_t kReadDeviceInfo = 1;
constexpr std::uint16_t kRead = 2;
constexpr std::uint16_t kWrite = 3;
constexpr std::uint16_t kReadState = 4;
constexpr std::uint16_t kWriteControl = 5;
constexpr std::uint16_t kAddNotification = 6;
constexpr std::uint16_t kDeleteNotification = 7;
constexpr std::uint16_t kNotification = 8;
constexpr std::uint16_t kReadWrite = 9;

// AMS header state flags
constexpr std::uint16_t kFlagResponse = 0x0001;
constexpr std::uint16_t kFlagCommand = 0x0004;

// Index groups of the PLC symbol server
constexpr std::uint32_t kSymbolHandleByName = 0xF003;  // read-write: name -> handle
constexpr std::uint32_t kSymbolValueByHandle = 0xF005;
constexpr std::uint32_t kSymbolReleaseHandle = 0xF006;  // write: handle
constexpr std::uint32_t kSymbolVersion = 0xF008;        // 1 byte, changes with the program
constexpr std::uint32_t kSymbolInfoByNameEx = 0xF009;   // read-write: name -> symbol entry
// Sum commands: several sub-commands in one request (TwinCAT 2.11 and later)
constexpr std::uint32_t kSumRead = 0xF080;
constexpr std::uint32_t kSumWrite = 0xF081;
constexpr std::uint32_t kSumReadWrite = 0xF082;
constexpr std::size_t kMaxSumCommands = 500;  // sub-commands per request

// Notification transmission modes
constexpr std::uint32_t kTransModeCyclic = 3;    // ADSTRANS_SERVERCYCLE
constexpr std::uint32_t kTransModeOnChange = 4;  // ADSTRANS_SERVERONCHA

constexpr std::size_t kTcpHeaderSize = 6;
constexpr std::size_t kAmsHeaderSize = 32;
constexpr std::size_t kFrameHeaderSize = kTcpHeaderSize + kAmsHeaderSize;
/// Frames larger than that are considered a protocol error.
constexpr std::size_t kMaxFrameSize = 64u * 1024u * 1024u;

/// Windows FILETIME (100 ns since 1601-01-01) of the Unix epoch.
constexpr std::uint64_t kFileTimeUnixEpoch = 116444736000000000ull;

struct AmsNetId
{
  std::array<std::uint8_t, 6> bytes{};

  /// Parses "192.168.1.10.1.1"; returns false if the text is not a valid AMS Net Id.
  static bool parse(const std::string & text, AmsNetId & net_id);
  std::string toString() const;

  friend bool operator==(const AmsNetId & lhs, const AmsNetId & rhs)
  {
    return lhs.bytes == rhs.bytes;
  }
  friend bool operator!=(const AmsNetId & lhs, const AmsNetId & rhs) {return !(lhs == rhs);}
};

struct AmsAddress
{
  AmsNetId net_id;
  std::uint16_t port = 0;
};

struct AmsHeader
{
  AmsAddress target;
  AmsAddress source;
  std::uint16_t command = 0;
  std::uint16_t flags = 0;
  std::uint32_t length = 0;  // of the data following the header
  std::uint32_t error = 0;
  std::uint32_t invoke_id = 0;

  /// Writes the 32 bytes of the header.
  void encode(std::uint8_t * out) const;
  static AmsHeader decode(const std::uint8_t * in);
};

/// Appends AMS/TCP frames to a byte vector, without allocating once it has grown enough.
class FrameWriter
{
public:
  explicit FrameWriter(std::vector<std::uint8_t> & out)
  : out_(out) {}

  /// Starts a request; its data is appended with the functions below until end().
  void begin(
    const AmsAddress & target, const AmsAddress & source, std::uint16_t command,
    std::uint32_t invoke_id)
  {
    start_ = out_.size();
    out_.resize(start_ + kFrameHeaderSize);
    header_.target = target;
    header_.source = source;
    header_.command = command;
    header_.flags = kFlagCommand;
    header_.error = 0;
    header_.invoke_id = invoke_id;
  }

  void u32(std::uint32_t value)
  {
    const std::size_t at = out_.size();
    out_.resize(at + 4);
    put32(out_.data() + at, value);
  }

  void bytes(const void * data, std::size_t size)
  {
    if (size > 0) {
      const std::size_t at = out_.size();
      out_.resize(at + size);
      std::memcpy(out_.data() + at, data, size);
    }
  }

  /// Completes the frame started by begin().
  void end()
  {
    const std::size_t length = out_.size() - start_ - kFrameHeaderSize;
    std::uint8_t * frame = out_.data() + start_;
    put16(frame, 0);  // reserved
    put32(frame + 2, static_cast<std::uint32_t>(kAmsHeaderSize + length));
    header_.length = static_cast<std::uint32_t>(length);
    header_.encode(frame + kTcpHeaderSize);
  }

private:
  std::vector<std::uint8_t> & out_;
  std::size_t start_ = 0;
  AmsHeader header_;
};

/// One sample of a device notification.
struct NotificationSample
{
  std::uint32_t handle;
  std::uint64_t timestamp;  // FILETIME
  const std::uint8_t * data;
  std::uint32_t size;
};

/// Calls `on_sample(const NotificationSample &)` for each sample of the data of a device
/// notification (AdsNotificationStream). Returns false if the data is malformed (the samples
/// before the error have been processed).
template<typename OnSample>
bool parseNotification(const std::uint8_t * data, std::size_t size, OnSample && on_sample)
{
  if (size < 8) {
    return false;
  }
  const std::size_t length = get32(data);
  if (length > size - 4) {
    return false;
  }
  const std::uint8_t * pos = data + 8;
  const std::uint8_t * const end = data + 4 + length;
  for (std::uint32_t stamps = get32(data + 4); stamps > 0; --stamps) {
    if (end - pos < 12) {
      return false;
    }
    const std::uint64_t timestamp = get64(pos);
    std::uint32_t samples = get32(pos + 8);
    pos += 12;
    for (; samples > 0; --samples) {
      if (end - pos < 8) {
        return false;
      }
      const std::uint32_t handle = get32(pos);
      const std::uint32_t sample_size = get32(pos + 4);
      pos += 8;
      if (static_cast<std::size_t>(end - pos) < sample_size) {
        return false;
      }
      on_sample(NotificationSample{handle, timestamp, pos, sample_size});
      pos += sample_size;
    }
  }
  return true;
}

}  // namespace ads
}  // namespace beckhoff_ros

#endif  // ADS_PROTOCOL_HPP_
