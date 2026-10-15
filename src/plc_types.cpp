#include "beckhoff_ros/plc_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace beckhoff_ros
{

namespace
{

// Type names that can be used in the configuration file (BeckhoffPLC.cTypes in the Python
// version). Add names here as needed.
const std::pair<const char *, Primitive> kPrimitiveTypes[] = {
  // names available in the Python version
  {"bool", Primitive::Bool},
  {"int", Primitive::Int16},
  {"int16", Primitive::Int16},
  {"uint16", Primitive::UInt16},
  {"uint8", Primitive::UInt8},
  {"real", Primitive::Real32},
  // other fixed size integer and floating point types
  {"int8", Primitive::Int8},
  {"int32", Primitive::Int32},
  {"uint32", Primitive::UInt32},
  {"int64", Primitive::Int64},
  {"uint64", Primitive::UInt64},
  {"float", Primitive::Real32},
  {"double", Primitive::Real64},
  // IEC 61131-3 names of the same types
  {"sint", Primitive::Int8},
  {"usint", Primitive::UInt8},
  {"byte", Primitive::UInt8},
  {"uint", Primitive::UInt16},
  {"word", Primitive::UInt16},
  {"dint", Primitive::Int32},
  {"udint", Primitive::UInt32},
  {"dword", Primitive::UInt32},
  {"lint", Primitive::Int64},
  {"ulint", Primitive::UInt64},
  {"lword", Primitive::UInt64},
  {"lreal", Primitive::Real64},
  // insert type
};

std::size_t primitiveSize(Primitive primitive)
{
  switch (primitive) {
    case Primitive::Bool:
    case Primitive::Int8:
    case Primitive::UInt8:
      return 1;
    case Primitive::Int16:
    case Primitive::UInt16:
      return 2;
    case Primitive::Int32:
    case Primitive::UInt32:
    case Primitive::Real32:
      return 4;
    case Primitive::Int64:
    case Primitive::UInt64:
    case Primitive::Real64:
      return 8;
  }
  return 0;
}

std::size_t alignUp(std::size_t value, std::size_t alignment)
{
  return (value + alignment - 1) / alignment * alignment;
}

template<typename T>
T loadLittleEndian(const std::uint8_t * data)
{
  using U = typename std::make_unsigned<T>::type;
  U value = 0;
  for (std::size_t i = 0; i < sizeof(U); ++i) {
    value = static_cast<U>(value | static_cast<U>(static_cast<U>(data[i]) << (8 * i)));
  }
  return static_cast<T>(value);
}

template<typename T>
void storeLittleEndian(std::uint8_t * out, T value)
{
  using U = typename std::make_unsigned<T>::type;
  const auto bits = static_cast<U>(value);
  for (std::size_t i = 0; i < sizeof(U); ++i) {
    out[i] = static_cast<std::uint8_t>(bits >> (8 * i));
  }
}

[[noreturn]] void throwTypeMismatch(
  const std::string & path, const char * expected, const std::string & type_name,
  const Value & value)
{
  throw ValueError(
          path + ": expected " + expected + " for '" + type_name + "', got " + value.repr());
}

template<typename T>
T toInteger(const Value & value, const std::string & path, const std::string & type_name)
{
  // like ctypes: int (and bool) are accepted, float is not
  if (!value.isInt() && !value.isBool()) {
    throwTypeMismatch(path, "an int", type_name, value);
  }
  // unlike ctypes, out of range values are rejected instead of silently truncated
  using Limits = std::numeric_limits<T>;
  const bool negative = value.asFloat() < 0;
  const bool in_range = negative ?
    std::is_signed<T>::value && value.asInt() >= static_cast<std::int64_t>(Limits::min()) :
    value.asUInt() <= static_cast<std::uint64_t>(Limits::max());
  if (!in_range) {
    throw ValueError(
            path + ": " + value.repr() + " is out of range for '" + type_name + "' (" +
            std::to_string(Limits::min()) + ".." + std::to_string(Limits::max()) + ")");
  }
  return negative ? static_cast<T>(value.asInt()) : static_cast<T>(value.asUInt());
}

double toReal(const Value & value, const std::string & path, const std::string & type_name)
{
  if (!value.isFloat() && !value.isInt() && !value.isBool()) {
    throwTypeMismatch(path, "a float", type_name, value);
  }
  return value.asFloat();
}

void encodePrimitive(
  Primitive primitive, const std::string & type_name, const Value & value, std::uint8_t * out,
  const std::string & path)
{
  switch (primitive) {
    case Primitive::Bool: {
        // like ctypes.c_bool, numbers are converted to their truth value
        bool flag = false;
        if (value.isBool()) {
          flag = value.asBool();
        } else if (value.isInt()) {
          flag = value != Value(0);
        } else if (value.isFloat()) {
          flag = value.asFloat() != 0.0;
        } else {
          throwTypeMismatch(path, "a bool", type_name, value);
        }
        out[0] = flag ? 1 : 0;
        return;
      }
    case Primitive::Int8:
      storeLittleEndian(out, toInteger<std::int8_t>(value, path, type_name));
      return;
    case Primitive::UInt8:
      storeLittleEndian(out, toInteger<std::uint8_t>(value, path, type_name));
      return;
    case Primitive::Int16:
      storeLittleEndian(out, toInteger<std::int16_t>(value, path, type_name));
      return;
    case Primitive::UInt16:
      storeLittleEndian(out, toInteger<std::uint16_t>(value, path, type_name));
      return;
    case Primitive::Int32:
      storeLittleEndian(out, toInteger<std::int32_t>(value, path, type_name));
      return;
    case Primitive::UInt32:
      storeLittleEndian(out, toInteger<std::uint32_t>(value, path, type_name));
      return;
    case Primitive::Int64:
      storeLittleEndian(out, toInteger<std::int64_t>(value, path, type_name));
      return;
    case Primitive::UInt64:
      storeLittleEndian(out, toInteger<std::uint64_t>(value, path, type_name));
      return;
    case Primitive::Real32: {
        const double number = toReal(value, path, type_name);
        if (std::isfinite(number) &&
          std::fabs(number) > static_cast<double>(std::numeric_limits<float>::max()))
        {
          throw ValueError(
                  path + ": " + value.repr() + " is out of range for '" + type_name + "'");
        }
        const auto real = static_cast<float>(number);
        std::uint32_t bits = 0;
        std::memcpy(&bits, &real, sizeof(bits));
        storeLittleEndian(out, bits);
        return;
      }
    case Primitive::Real64: {
        const double real = toReal(value, path, type_name);
        std::uint64_t bits = 0;
        std::memcpy(&bits, &real, sizeof(bits));
        storeLittleEndian(out, bits);
        return;
      }
  }
}

Value decodePrimitive(Primitive primitive, const std::uint8_t * data)
{
  switch (primitive) {
    case Primitive::Bool: return Value(data[0] != 0);
    case Primitive::Int8: return Value(loadLittleEndian<std::int8_t>(data));
    case Primitive::UInt8: return Value(loadLittleEndian<std::uint8_t>(data));
    case Primitive::Int16: return Value(loadLittleEndian<std::int16_t>(data));
    case Primitive::UInt16: return Value(loadLittleEndian<std::uint16_t>(data));
    case Primitive::Int32: return Value(loadLittleEndian<std::int32_t>(data));
    case Primitive::UInt32: return Value(loadLittleEndian<std::uint32_t>(data));
    case Primitive::Int64: return Value(loadLittleEndian<std::int64_t>(data));
    case Primitive::UInt64: return Value(loadLittleEndian<std::uint64_t>(data));
    case Primitive::Real32: {
        const auto bits = loadLittleEndian<std::uint32_t>(data);
        float real = 0;
        std::memcpy(&real, &bits, sizeof(real));
        return Value(real);
      }
    case Primitive::Real64: {
        const auto bits = loadLittleEndian<std::uint64_t>(data);
        double real = 0;
        std::memcpy(&real, &bits, sizeof(real));
        return Value(real);
      }
  }
  return Value();
}

}  // namespace

PlcTypePtr PlcType::primitive(std::string name, Primitive primitive)
{
  std::shared_ptr<PlcType> type(new PlcType());
  type->kind_ = Kind::Primitive;
  type->name_ = std::move(name);
  type->primitive_ = primitive;
  type->size_ = primitiveSize(primitive);
  type->alignment_ = type->size_;
  return type;
}

PlcTypePtr PlcType::array(PlcTypePtr element, std::size_t length)
{
  std::shared_ptr<PlcType> type(new PlcType());
  type->kind_ = Kind::Array;
  type->name_ = element->name() + "[" + std::to_string(length) + "]";
  type->size_ = element->size() * length;
  type->alignment_ = element->alignment();
  type->length_ = length;
  type->element_ = std::move(element);
  return type;
}

PlcTypePtr PlcType::structure(
  std::string name, const std::vector<std::pair<std::string, PlcTypePtr>> & fields,
  std::size_t pack)
{
  std::shared_ptr<PlcType> type(new PlcType());
  type->kind_ = Kind::Struct;
  type->name_ = std::move(name);
  // Same algorithm as ctypes: each field is aligned to its natural alignment (limited by
  // pack), the structure is aligned to its largest field alignment.
  std::size_t offset = 0;
  std::size_t alignment = 1;
  for (const auto & field : fields) {
    std::size_t field_alignment = field.second->alignment();
    if (pack > 0) {
      field_alignment = std::min(field_alignment, pack);
    }
    offset = alignUp(offset, field_alignment);
    type->fields_.push_back({field.first, field.second, offset});
    offset += field.second->size();
    alignment = std::max(alignment, field_alignment);
  }
  type->alignment_ = alignment;
  type->size_ = alignUp(offset, alignment);
  return type;
}

Value PlcType::decode(const std::uint8_t * data) const
{
  switch (kind_) {
    case Kind::Primitive:
      return decodePrimitive(primitive_, data);
    case Kind::Array: {
        Value::List list;
        list.reserve(length_);
        for (std::size_t i = 0; i < length_; ++i) {
          list.push_back(element_->decode(data + i * element_->size()));
        }
        return Value(std::move(list));
      }
    case Kind::Struct: {
        Value::Dict dict;
        dict.reserve(fields_.size());
        for (const auto & field : fields_) {
          dict.emplace_back(field.name, field.type->decode(data + field.offset));
        }
        return Value(std::move(dict));
      }
  }
  return Value();
}

Value PlcType::decode(const std::vector<std::uint8_t> & data) const
{
  if (data.size() != size_) {
    throw ValueError(
            "'" + name_ + "' needs " + std::to_string(size_) + " bytes, got " +
            std::to_string(data.size()));
  }
  return decode(data.data());
}

std::vector<std::uint8_t> PlcType::encode(const Value & value, const std::string & name) const
{
  std::vector<std::uint8_t> data(size_, 0);
  encodeInto(value, data.data(), name);
  return data;
}

void PlcType::encodeInto(const Value & value, std::uint8_t * out, const std::string & path) const
{
  switch (kind_) {
    case Kind::Primitive:
      encodePrimitive(primitive_, name_, value, out, path);
      return;
    case Kind::Array: {
        if (!value.isList()) {
          throwTypeMismatch(path, "a list", name_, value);
        }
        const auto & items = value.asList();
        if (items.size() > length_) {
          throw ValueError(
                  path + ": too many items for '" + name_ + "' (" + std::to_string(items.size()) +
                  ")");
        }
        for (std::size_t i = 0; i < items.size(); ++i) {
          element_->encodeInto(
            items[i], out + i * element_->size(), path + "[" + std::to_string(i) + "]");
        }
        return;
      }
    case Kind::Struct: {
        if (!value.isDict()) {
          throwTypeMismatch(path, "a dict", name_, value);
        }
        for (const auto & field : fields_) {
          const Value * item = value.find(field.name);
          if (item == nullptr) {
            throw ValueError(
                    path + ": missing field " + formatPythonString(field.name) + " of '" + name_ +
                    "'");
          }
          field.type->encodeInto(
            *item, out + field.offset, path + "[" + formatPythonString(field.name) + "]");
        }
        return;
      }
  }
}

TypeRegistry::TypeRegistry()
{
  for (const auto & entry : kPrimitiveTypes) {
    types_[entry.first] = PlcType::primitive(entry.first, entry.second);
  }
}

PlcTypePtr TypeRegistry::find(const std::string & name) const
{
  const auto it = types_.find(name);
  return it == types_.end() ? nullptr : it->second;
}

PlcTypePtr TypeRegistry::get(const std::string & name) const
{
  auto type = find(name);
  if (!type) {
    throw ConfigError("unknown type '" + name + "'");
  }
  return type;
}

PlcTypePtr TypeRegistry::resolve(const nlohmann::ordered_json & spec) const
{
  if (spec.is_string()) {
    return get(spec.get<std::string>());
  }
  if (spec.is_array() && spec.size() == 2 && spec[0].is_string() &&
    spec[1].is_number_integer() && spec[1].get<std::int64_t>() > 0)
  {
    return PlcType::array(get(spec[0].get<std::string>()), spec[1].get<std::size_t>());
  }
  throw ConfigError(
          "invalid type " + spec.dump() + " (expected a type name or [type name, size])");
}

PlcTypePtr TypeRegistry::addStructure(
  const std::string & name, const nlohmann::ordered_json & fields, std::size_t pack)
{
  if (!fields.is_object()) {
    throw ConfigError("Each structure listed must be a Dictionary ('" + name + "' is not)");
  }
  if (fields.empty()) {
    throw ConfigError("Structure '" + name + "' has no fields");
  }
  std::vector<std::pair<std::string, PlcTypePtr>> members;
  for (auto it = fields.begin(); it != fields.end(); ++it) {
    if (!it.value().is_string() && !it.value().is_array()) {
      throw ConfigError(
              "Unable to create CType struct for '" + name +
              "'. Fields must be either str or a list");
    }
    try {
      members.emplace_back(it.key(), resolve(it.value()));
    } catch (const ConfigError & e) {
      throw ConfigError(
              "Unable to create CType struct for '" + name + "', field '" + it.key() + "': " +
              e.what() + " (structures can only use types defined before them)");
    }
  }
  auto type = PlcType::structure(name, members, pack);
  add(name, type);
  return type;
}

void TypeRegistry::add(const std::string & name, PlcTypePtr type)
{
  if (!types_.emplace(name, std::move(type)).second) {
    throw ConfigError("Type '" + name + "' is already defined");
  }
}

}  // namespace beckhoff_ros
