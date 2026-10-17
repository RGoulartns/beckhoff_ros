#include "beckhoff_plc/value.hpp"

#include <ostream>

#include "util.hpp"

namespace beckhoff {

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "ADS data is little endian; big endian hosts are not supported"
#endif

namespace detail {

void throwOutOfRange(const std::string& value, const Type& plcType, const char* cppType, bool writing) {
  if (writing) throw RangeError("value " + value + " does not fit in " + plcType.name());
  throw RangeError(plcType.name() + " value " + value + " does not fit in " + cppType);
}

void throwNotNumeric(const Type& type) {
  throw TypeError("'" + type.name() + "' is not a number or BOOL");
}

void throwInvalidRef() { throw TypeError("empty value (no type)"); }

void throwRawSize(const Type& type, std::size_t size) {
  throw TypeError("size mismatch: '" + type.name() + "' has " + std::to_string(type.size()) +
                  " bytes but the C++ type has " + std::to_string(size) +
                  " (check the members and the alignment/#pragma pack of the C++ struct)");
}

}  // namespace detail

namespace {

bool isAscii(std::string_view text) {
  for (char c : text) {
    if (static_cast<unsigned char>(c) >= 0x80) return false;
  }
  return true;
}

/// Contents of a STRING up to the NUL terminator, still Latin-1 encoded.
std::string_view rawString(const Type& type, const std::uint8_t* data) {
  const char* text = reinterpret_cast<const char*>(data);
  const std::size_t max = type.stringLength();
  std::size_t length = 0;
  while (length < max && text[length] != '\0') ++length;
  return {text, length};
}

std::size_t wstringUnits(const Type& type, const std::uint8_t* data) {
  const std::size_t max = type.stringLength();
  std::size_t units = 0;
  while (units < max && (data[2 * units] != 0 || data[2 * units + 1] != 0)) ++units;
  return units;
}

std::string readString(const Type& type, const std::uint8_t* data) {
  if (type.primitive() == Primitive::String) {
    const std::string_view raw = rawString(type, data);
    return isAscii(raw) ? std::string(raw) : util::latin1ToUtf8(raw);
  }
  return util::utf16ToUtf8(data, wstringUnits(type, data));
}

/// Writes UTF-8 text into a STRING (as Latin-1) or WSTRING (as UTF-16).
void writeString(const Type& type, std::uint8_t* data, std::string_view utf8) {
  if (!type.isString()) throw TypeError("'" + type.name() + "' is not a STRING or WSTRING");
  const std::size_t max = type.stringLength();
  if (type.primitive() == Primitive::String) {
    std::string latin1;
    if (!isAscii(utf8)) {
      latin1 = util::utf8ToLatin1(utf8);
      utf8 = latin1;
    }
    if (utf8.size() > max) {
      throw RangeError("string of " + std::to_string(utf8.size()) + " characters does not fit in " + type.name());
    }
    std::memset(data, 0, type.size());
    std::memcpy(data, utf8.data(), utf8.size());
  } else {
    const std::u16string units = util::utf8ToUtf16(utf8);
    if (units.size() > max) {
      throw RangeError("string of " + std::to_string(units.size()) + " UTF-16 units does not fit in " + type.name());
    }
    std::memset(data, 0, type.size());
    for (std::size_t i = 0; i < units.size(); ++i) {
      data[2 * i] = static_cast<std::uint8_t>(units[i] & 0xFF);
      data[2 * i + 1] = static_cast<std::uint8_t>(units[i] >> 8);
    }
  }
}

/// Position inside a value, for error messages ("attr2.attr1", "[3].attr1").
struct Path {
  const Path* parent = nullptr;
  std::string_view field;
  std::size_t index = 0;

  std::string str() const {
    if (!parent) return {};
    std::string prefix = parent->str();
    if (field.empty()) return prefix + "[" + std::to_string(index) + "]";
    return prefix.empty() ? std::string(field) : prefix + "." + std::string(field);
  }
  std::string where() const {
    const std::string path = str();
    return path.empty() ? std::string() : "'" + path + "': ";
  }
};

const char* jsonTypeName(const Json& json) {
  if (json.is_number_float()) return "a floating point number";
  if (json.is_number()) return "an integer";
  if (json.is_boolean()) return "a boolean";
  if (json.is_string()) return "a string";
  if (json.is_array()) return "an array";
  if (json.is_object()) return "an object";
  return "null";
}

Json toJsonImpl(const Type& type, const std::uint8_t* data) {
  switch (type.kind()) {
    case Type::Kind::Primitive:
      switch (type.primitive()) {
        case Primitive::Bool: return data[0] != 0;
        case Primitive::Int8: return detail::load<std::int8_t>(data);
        case Primitive::UInt8: return detail::load<std::uint8_t>(data);
        case Primitive::Int16: return detail::load<std::int16_t>(data);
        case Primitive::UInt16: return detail::load<std::uint16_t>(data);
        case Primitive::Int32: return detail::load<std::int32_t>(data);
        case Primitive::UInt32: return detail::load<std::uint32_t>(data);
        case Primitive::Int64: return detail::load<std::int64_t>(data);
        case Primitive::UInt64: return detail::load<std::uint64_t>(data);
        case Primitive::Real32: return detail::load<float>(data);
        case Primitive::Real64: return detail::load<double>(data);
        case Primitive::String:
        case Primitive::WString: return readString(type, data);
      }
      break;
    case Type::Kind::Array: {
      Json array = Json::array();
      const Type& element = type.elementType();
      for (std::size_t i = 0; i < type.length(); ++i) array.push_back(toJsonImpl(element, data + i * element.size()));
      return array;
    }
    case Type::Kind::Struct: {
      Json object = Json::object();
      for (const Field& f : type.fields()) object[f.name] = toJsonImpl(*f.type, data + f.offset);
      return object;
    }
  }
  return nullptr;
}

void fromJsonImpl(const Type& type, std::uint8_t* data, const Json& json, bool partial, const Path& path) {
  switch (type.kind()) {
    case Type::Kind::Primitive:
      try {
        if (type.isString()) {
          if (!json.is_string()) throw TypeError("expected a string, got " + std::string(jsonTypeName(json)));
          writeString(type, data, json.get_ref<const std::string&>());
        } else if (json.is_boolean()) {
          detail::writeArithmetic(type, data, json.get<bool>());
        } else if (json.is_number_unsigned()) {
          detail::writeArithmetic(type, data, json.get<std::uint64_t>());
        } else if (json.is_number_integer()) {
          detail::writeArithmetic(type, data, json.get<std::int64_t>());
        } else if (json.is_number_float()) {
          detail::writeArithmetic(type, data, json.get<double>());
        } else {
          throw TypeError("expected a number for " + type.name() + ", got " + jsonTypeName(json));
        }
      } catch (const TypeError& e) {
        throw TypeError(path.where() + e.what());
      } catch (const RangeError& e) {
        throw RangeError(path.where() + e.what());
      }
      return;

    case Type::Kind::Array: {
      if (!json.is_array()) {
        throw TypeError(path.where() + "expected an array for " + type.name() + ", got " + jsonTypeName(json));
      }
      const std::size_t length = type.length();
      if (json.size() > length || (!partial && json.size() != length)) {
        throw RangeError(path.where() + type.name() + " has " + std::to_string(length) + " elements, got " +
                         std::to_string(json.size()));
      }
      const Type& element = type.elementType();
      for (std::size_t i = 0; i < json.size(); ++i) {
        const Path child{&path, {}, i};
        fromJsonImpl(element, data + i * element.size(), json[i], partial, child);
      }
      return;
    }

    case Type::Kind::Struct: {
      if (!json.is_object()) {
        throw TypeError(path.where() + "expected an object for structure '" + type.name() + "', got " +
                        jsonTypeName(json));
      }
      for (const auto& item : json.items()) {
        if (!type.findField(item.key())) {
          throw KeyError(path.where() + "structure '" + type.name() + "' has no field '" + item.key() + "'");
        }
      }
      for (const Field& f : type.fields()) {
        auto it = json.find(f.name);
        if (it == json.end()) {
          // TwinCAT names are case-insensitive
          for (auto candidate = json.begin(); candidate != json.end(); ++candidate) {
            if (util::iequals(candidate.key(), f.name)) {
              it = candidate;
              break;
            }
          }
        }
        if (it == json.end()) {
          if (partial) continue;
          throw KeyError(path.where() + "missing field '" + f.name + "' of structure '" + type.name() + "'");
        }
        const Path child{&path, f.name, 0};
        fromJsonImpl(*f.type, data + f.offset, *it, partial, child);
      }
      return;
    }
  }
}

/// Resolves a path such as "attr2.attr1", "attr3[1]" or "[3].attr2".
std::pair<const Type*, std::size_t> navigate(const Type& root, std::string_view path) {
  const Type* type = &root;
  std::size_t offset = 0;
  std::size_t pos = 0;
  const auto fail = [&](const std::string& message) -> TypeError {
    return TypeError("invalid path '" + std::string(path) + "': " + message);
  };
  while (pos < path.size()) {
    if (path[pos] == '[') {
      const std::size_t close = path.find(']', pos);
      if (close == std::string_view::npos || close == pos + 1) throw fail("expected '[index]'");
      std::size_t index = 0;
      for (std::size_t i = pos + 1; i < close; ++i) {
        if (path[i] < '0' || path[i] > '9') throw fail("invalid index");
        if (index > (std::numeric_limits<std::size_t>::max() - 9) / 10) throw fail("index too large");
        index = index * 10 + static_cast<std::size_t>(path[i] - '0');
      }
      if (!type->isArray()) throw fail("'" + type->name() + "' is not an array");
      if (index >= type->length()) {
        throw RangeError("invalid path '" + std::string(path) + "': index " + std::to_string(index) +
                         " out of range for " + type->name());
      }
      type = &type->elementType();
      offset += index * type->size();
      pos = close + 1;
    } else {
      if (path[pos] == '.') {
        if (pos == 0) throw fail("unexpected '.'");
        ++pos;
      } else if (pos != 0) {
        throw fail("expected '.' or '['");
      }
      const std::size_t start = pos;
      while (pos < path.size() && util::isIdentifierChar(path[pos])) ++pos;
      if (pos == start) throw fail("expected a field name");
      const std::string_view name = path.substr(start, pos - start);
      if (!type->isStruct()) throw fail("'" + type->name() + "' is not a structure");
      const Field* f = type->findField(name);
      if (!f) {
        throw KeyError("invalid path '" + std::string(path) + "': structure '" + type->name() + "' has no field '" +
                       std::string(name) + "'");
      }
      offset += f->offset;
      type = f->type.get();
    }
  }
  return {type, offset};
}

bool equalImpl(const Type& type, const std::uint8_t* a, const std::uint8_t* b) {
  switch (type.kind()) {
    case Type::Kind::Primitive:
      switch (type.primitive()) {
        case Primitive::Bool: return (a[0] != 0) == (b[0] != 0);
        case Primitive::String: return rawString(type, a) == rawString(type, b);
        case Primitive::WString: {
          const std::size_t units = wstringUnits(type, a);
          return units == wstringUnits(type, b) && std::memcmp(a, b, 2 * units) == 0;
        }
        default: return std::memcmp(a, b, type.size()) == 0;
      }
    case Type::Kind::Array: {
      const Type& element = type.elementType();
      for (std::size_t i = 0; i < type.length(); ++i) {
        if (!equalImpl(element, a + i * element.size(), b + i * element.size())) return false;
      }
      return true;
    }
    case Type::Kind::Struct:
      for (const Field& f : type.fields()) {
        if (!equalImpl(*f.type, a + f.offset, b + f.offset)) return false;
      }
      return true;
  }
  return false;
}

}  // namespace

// ---------------------------------------------------------------------------
// ConstValueRef

ConstValueRef ConstValueRef::operator[](std::size_t index) const {
  const Type& t = type();
  if (!t.isArray()) throw TypeError("'" + t.name() + "' is not an array");
  if (index >= t.length()) {
    throw RangeError("index " + std::to_string(index) + " out of range for " + t.name() + " (indices are zero-based)");
  }
  const Type& element = t.elementType();
  return {element, data_ + index * element.size()};
}

ConstValueRef ConstValueRef::at(std::string_view path) const {
  const auto [t, offset] = navigate(type(), path);
  return {*t, data_ + offset};
}

Json ConstValueRef::toJson() const { return toJsonImpl(type(), data_); }

std::string ConstValueRef::toString(int indent) const {
  return toJson().dump(indent, ' ', false, Json::error_handler_t::replace);
}

std::string ConstValueRef::stringValue() const {
  if (!type().isString()) throw TypeError("'" + type().name() + "' is not a STRING or WSTRING");
  return readString(*type_, data_);
}

void ConstValueRef::checkRawSize(std::size_t size) const {
  if (size != type().size()) detail::throwRawSize(*type_, size);
}

// ---------------------------------------------------------------------------
// ValueRef

ValueRef ValueRef::operator[](std::size_t index) const {
  const ConstValueRef element = ConstValueRef::operator[](index);
  return {element.type(), const_cast<std::uint8_t*>(element.data())};
}

ValueRef ValueRef::at(std::string_view path) const {
  const auto [t, offset] = navigate(type(), path);
  return {*t, data() + offset};
}

void ValueRef::assign(ConstValueRef other) const {
  const Type& to = type();
  const Type& from = other.type();
  if (to.sameLayout(from)) {
    if (data_ != other.data()) std::memmove(data(), other.data(), to.size());
  } else if (to.isString() && from.isString()) {
    writeString(to, data(), readString(from, other.data()));
  } else if (to.isPrimitive() && from.isPrimitive() && !to.isString() && !from.isString()) {
    switch (from.primitive()) {
      case Primitive::Real32:
      case Primitive::Real64: set(other.as<double>()); break;
      case Primitive::Int8:
      case Primitive::Int16:
      case Primitive::Int32:
      case Primitive::Int64: set(other.as<std::int64_t>()); break;
      case Primitive::Bool: set(other.as<bool>()); break;
      default: set(other.as<std::uint64_t>()); break;
    }
  } else {
    throw TypeError("cannot assign a value of type '" + from.name() + "' to '" + to.name() + "'");
  }
}

void ValueRef::fromJson(const Json& json) const {
  std::vector<std::uint8_t> scratch(data_, data_ + type().size());  // unchanged on error
  fromJsonImpl(*type_, scratch.data(), json, false, Path{});
  std::memcpy(data(), scratch.data(), scratch.size());
}

void ValueRef::update(const Json& json) const {
  std::vector<std::uint8_t> scratch(data_, data_ + type().size());
  fromJsonImpl(*type_, scratch.data(), json, true, Path{});
  std::memcpy(data(), scratch.data(), scratch.size());
}

void ValueRef::clear() const noexcept {
  if (valid()) std::memset(data(), 0, type_->size());
}

void ValueRef::setString(std::string_view text) const { writeString(type(), data(), text); }

// ---------------------------------------------------------------------------
// Value

Value::Value(TypePtr type) : type_(std::move(type)) {
  if (!type_) throw TypeError("cannot create a value without a type");
  bytes_.assign(type_->size(), 0);
}

Value::Value(TypePtr type, const Json& json) : Value(std::move(type)) { fromJson(json); }

Value::Value(ConstValueRef value) {
  if (!value.valid()) return;
  type_ = value.typePtr();
  bytes_.assign(value.data(), value.data() + type_->size());
}

bool operator==(const ConstValueRef& a, const ConstValueRef& b) {
  if (!a.valid() || !b.valid()) return a.valid() == b.valid();
  return a.type().sameLayout(b.type()) && equalImpl(a.type(), a.data(), b.data());
}

std::ostream& operator<<(std::ostream& out, const ConstValueRef& value) {
  return out << (value.valid() ? value.toString() : std::string("<empty>"));
}

std::ostream& operator<<(std::ostream& out, const Value& value) { return out << value.ref(); }

}  // namespace beckhoff
