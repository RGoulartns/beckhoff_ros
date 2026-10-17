#include "ads_protocol.hpp"

#include <string>

namespace beckhoff_ros
{
namespace ads
{

bool AmsNetId::parse(const std::string & text, AmsNetId & net_id)
{
  AmsNetId result;
  std::size_t pos = 0;
  for (std::size_t i = 0; i < result.bytes.size(); ++i) {
    if (i > 0) {
      if (pos >= text.size() || text[pos] != '.') {
        return false;
      }
      ++pos;
    }
    unsigned value = 0;
    std::size_t digits = 0;
    while (pos < text.size() && digits < 3 && text[pos] >= '0' && text[pos] <= '9') {
      value = value * 10 + static_cast<unsigned>(text[pos++] - '0');
      ++digits;
    }
    if (digits == 0 || value > 255) {
      return false;
    }
    result.bytes[i] = static_cast<std::uint8_t>(value);
  }
  if (pos != text.size()) {
    return false;
  }
  net_id = result;
  return true;
}

std::string AmsNetId::toString() const
{
  std::string text;
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i > 0) {
      text += '.';
    }
    text += std::to_string(bytes[i]);
  }
  return text;
}

void AmsHeader::encode(std::uint8_t * out) const
{
  std::memcpy(out, target.net_id.bytes.data(), 6);
  put16(out + 6, target.port);
  std::memcpy(out + 8, source.net_id.bytes.data(), 6);
  put16(out + 14, source.port);
  put16(out + 16, command);
  put16(out + 18, flags);
  put32(out + 20, length);
  put32(out + 24, error);
  put32(out + 28, invoke_id);
}

AmsHeader AmsHeader::decode(const std::uint8_t * in)
{
  AmsHeader header;
  std::memcpy(header.target.net_id.bytes.data(), in, 6);
  header.target.port = get16(in + 6);
  std::memcpy(header.source.net_id.bytes.data(), in + 8, 6);
  header.source.port = get16(in + 14);
  header.command = get16(in + 16);
  header.flags = get16(in + 18);
  header.length = get32(in + 20);
  header.error = get32(in + 24);
  header.invoke_id = get32(in + 28);
  return header;
}

}  // namespace ads
}  // namespace beckhoff_ros
