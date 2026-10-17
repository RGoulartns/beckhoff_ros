#include "beckhoff_ros/buffer.hpp"

#include <algorithm>
#include <cstring>
#include <string>
#include <string_view>
#include <utility>

namespace beckhoff_ros
{

namespace detail
{

Location locateField(const PlcType & type, const char * name, std::size_t length)
{
  if (type.kind() == PlcType::Kind::Struct) {
    for (const auto & field : type.fields()) {
      if (field.name.size() == length && std::memcmp(field.name.data(), name, length) == 0) {
        return Location{field.type.get(), field.offset};
      }
    }
    throw ValueError(
            "'" + type.name() + "' has no field " + formatPythonString(std::string(name, length)));
  }
  throw ValueError(
          "'" + type.name() + "' is not a structure (looking for field " +
          formatPythonString(std::string(name, length)) + ")");
}

Location locateIndex(const PlcType & type, std::size_t index)
{
  if (type.kind() != PlcType::Kind::Array) {
    throw ValueError(
            "'" + type.name() + "' is not an array (looking for index " + std::to_string(index) +
            ")");
  }
  if (index >= type.length()) {
    throw ValueError(
            "index " + std::to_string(index) + " is out of range for '" + type.name() + "'");
  }
  return Location{type.elementType().get(), index * type.elementType()->size()};
}

Location locatePath(const PlcType & root, std::string_view path)
{
  Location location{&root, 0};
  std::size_t pos = 0;
  while (pos < path.size()) {
    Location item{};
    if (path[pos] == '[') {
      const std::size_t close = path.find(']', pos);
      std::size_t index = 0;
      bool valid = close != std::string_view::npos && close > pos + 1;
      for (std::size_t i = pos + 1; valid && i < close; ++i) {
        valid = path[i] >= '0' && path[i] <= '9' && index < (std::size_t(1) << 48);
        index = index * 10 + static_cast<std::size_t>(path[i] - '0');
      }
      if (!valid) {
        throw ValueError("invalid index in path '" + std::string(path) + "'");
      }
      item = locateIndex(*location.type, index);
      pos = close + 1;
    } else {
      if (path[pos] == '.') {
        if (pos == 0) {
          throw ValueError("invalid path '" + std::string(path) + "'");
        }
        ++pos;
      } else if (pos > 0) {
        throw ValueError("invalid path '" + std::string(path) + "' (expected '.' or '[')");
      }
      const std::size_t end = std::min(path.find_first_of(".[", pos), path.size());
      if (end == pos) {
        throw ValueError("invalid path '" + std::string(path) + "' (missing field name)");
      }
      item = locateField(*location.type, path.data() + pos, end - pos);
      pos = end;
    }
    location = Location{item.type, location.offset + item.offset};
  }
  return location;
}

void checkSameType(const PlcType & type, const PlcType & expected)
{
  if (&type != &expected && (type.name() != expected.name() || type.size() != expected.size())) {
    throw ValueError(
            "data of type '" + type.name() + "' used where '" + expected.name() +
            "' is expected");
  }
}

void throwNotPrimitive(const PlcType & type)
{
  throw ValueError("'" + type.name() + "' is not an elementary type");
}

void throwNotConvertible(const PlcType & type, const Value & value, const char * to)
{
  throw ValueError(
          "the value " + value.repr() + " of '" + type.name() + "' cannot be converted to " + to);
}

}  // namespace detail

const View & View::operator=(const ConstView & other) const
{
  detail::checkSameType(other.type(), *type_);
  std::memmove(data(), other.data(), type_->size());
  return *this;
}

Buffer::Buffer(PlcTypePtr type)
: type_(std::move(type))
{
  if (!type_) {
    throw ValueError("Buffer: no type");
  }
  data_.assign(type_->size(), 0);
}

Buffer::Buffer(PlcTypePtr type, const Value & value)
: Buffer(std::move(type))
{
  setValue(value);
}

void Buffer::clear()
{
  std::fill(data_.begin(), data_.end(), 0);
}

bool operator==(const Buffer & lhs, const Buffer & rhs)
{
  if (!lhs.type_ || !rhs.type_) {
    return !lhs.type_ && !rhs.type_;
  }
  return (lhs.type_ == rhs.type_ || lhs.type_->name() == rhs.type_->name()) &&
         lhs.data_ == rhs.data_;
}

}  // namespace beckhoff_ros
