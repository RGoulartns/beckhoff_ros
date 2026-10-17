#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <iosfwd>
#include <limits>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <nlohmann/json.hpp>

#include "beckhoff_plc/errors.hpp"
#include "beckhoff_plc/types.hpp"

namespace beckhoff {

/// JSON type used by this library. It keeps the order of object members,
/// which matters for structure definitions (members are laid out in order).
using Json = nlohmann::ordered_json;

class ConstValueRef;
class ValueRef;
class Value;

namespace detail {

template <class>
inline constexpr bool alwaysFalse = false;

template <class T>
struct IsStdVector : std::false_type {};
template <class T, class A>
struct IsStdVector<std::vector<T, A>> : std::true_type {};

template <class T>
struct IsStdArray : std::false_type {};
template <class T, std::size_t N>
struct IsStdArray<std::array<T, N>> : std::true_type {};

template <class T>
inline constexpr bool isValueType = std::is_base_of_v<ConstValueRef, T> || std::is_same_v<T, Value>;

template <class T>
inline constexpr bool isStringLike = std::is_convertible_v<const T&, std::string_view> && !std::is_same_v<T, Json>;

/// Plain C++ objects that are copied byte by byte (structures mirroring a PLC
/// structure, enums, ...). Arithmetic types are converted instead.
template <class T>
inline constexpr bool isRawCopyable = std::is_trivially_copyable_v<T> && !std::is_arithmetic_v<T> &&
                                      !std::is_pointer_v<T> && !isValueType<T> && !IsStdArray<T>::value;

template <class T>
const char* cppTypeName() {
  if constexpr (std::is_same_v<T, bool>) return "bool";
  else if constexpr (std::is_floating_point_v<T>) return sizeof(T) == 4 ? "float" : "double";
  else if constexpr (std::is_signed_v<T>) return sizeof(T) == 1 ? "int8_t" : sizeof(T) == 2 ? "int16_t" : sizeof(T) == 4 ? "int32_t" : "int64_t";
  else return sizeof(T) == 1 ? "uint8_t" : sizeof(T) == 2 ? "uint16_t" : sizeof(T) == 4 ? "uint32_t" : "uint64_t";
}

[[noreturn]] void throwOutOfRange(const std::string& value, const Type& plcType, const char* cppType, bool writing);
[[noreturn]] void throwNotNumeric(const Type& type);
[[noreturn]] void throwInvalidRef();
[[noreturn]] void throwRawSize(const Type& type, std::size_t size);

/// Integer range check (std::in_range from C++20).
template <class To, class From>
constexpr bool inRange(From value) noexcept {
  if constexpr (std::is_signed_v<From> == std::is_signed_v<To>) {
    if constexpr (sizeof(To) >= sizeof(From)) {
      return true;
    } else {
      return value >= std::numeric_limits<To>::lowest() && value <= std::numeric_limits<To>::max();
    }
  } else if constexpr (std::is_signed_v<From>) {
    if constexpr (sizeof(To) >= sizeof(From)) {
      return value >= 0;
    } else {
      return value >= 0 && static_cast<std::make_unsigned_t<From>>(value) <= std::numeric_limits<To>::max();
    }
  } else {
    if constexpr (sizeof(To) > sizeof(From)) {
      return true;
    } else {
      return value <= static_cast<std::make_unsigned_t<To>>(std::numeric_limits<To>::max());
    }
  }
}

/// Converts between arithmetic types, refusing conversions that would change
/// the value (overflow, fractional part dropped, ...). Any non-zero value
/// converts to `true`.
template <class To, class From>
To numericCast(From value, const Type& plcType, bool writing) {
  if constexpr (std::is_same_v<To, bool>) {
    return value != From{};
  } else if constexpr (std::is_same_v<From, bool>) {
    return static_cast<To>(value ? 1 : 0);
  } else if constexpr (std::is_integral_v<To> && std::is_integral_v<From>) {
    if (!inRange<To>(value)) throwOutOfRange(std::to_string(value), plcType, cppTypeName<To>(), writing);
    return static_cast<To>(value);
  } else if constexpr (std::is_integral_v<To>) {
    // floating point -> integer: only integral values that fit
    const double v = static_cast<double>(value);
    constexpr int digits = std::numeric_limits<To>::digits;
    const double limit = std::ldexp(1.0, digits);
    const double lower = std::is_signed_v<To> ? -limit : 0.0;
    if (!std::isfinite(v) || std::trunc(v) != v || v < lower || v >= limit) {
      throwOutOfRange(std::to_string(v), plcType, cppTypeName<To>(), writing);
    }
    return static_cast<To>(v);
  } else if constexpr (std::is_floating_point_v<From> && sizeof(To) < sizeof(From)) {
    if (std::isfinite(value) && std::fabs(value) > static_cast<From>(std::numeric_limits<To>::max())) {
      throwOutOfRange(std::to_string(value), plcType, cppTypeName<To>(), writing);
    }
    return static_cast<To>(value);
  } else {
    return static_cast<To>(value);
  }
}

template <class T>
T load(const std::uint8_t* data) noexcept {
  T value;
  std::memcpy(&value, data, sizeof(T));
  return value;
}

template <class T>
void store(std::uint8_t* data, T value) noexcept {
  std::memcpy(data, &value, sizeof(T));
}

template <class T>
T readArithmetic(const Type& type, const std::uint8_t* data) {
  if (type.isPrimitive()) {
    switch (type.primitive()) {
      case Primitive::Bool: return numericCast<T>(data[0] != 0, type, false);
      case Primitive::Int8: return numericCast<T>(load<std::int8_t>(data), type, false);
      case Primitive::UInt8: return numericCast<T>(load<std::uint8_t>(data), type, false);
      case Primitive::Int16: return numericCast<T>(load<std::int16_t>(data), type, false);
      case Primitive::UInt16: return numericCast<T>(load<std::uint16_t>(data), type, false);
      case Primitive::Int32: return numericCast<T>(load<std::int32_t>(data), type, false);
      case Primitive::UInt32: return numericCast<T>(load<std::uint32_t>(data), type, false);
      case Primitive::Int64: return numericCast<T>(load<std::int64_t>(data), type, false);
      case Primitive::UInt64: return numericCast<T>(load<std::uint64_t>(data), type, false);
      case Primitive::Real32: return numericCast<T>(load<float>(data), type, false);
      case Primitive::Real64: return numericCast<T>(load<double>(data), type, false);
      case Primitive::String:
      case Primitive::WString: break;
    }
  }
  throwNotNumeric(type);
}

template <class T>
void writeArithmetic(const Type& type, std::uint8_t* data, T value) {
  if (type.isPrimitive()) {
    switch (type.primitive()) {
      case Primitive::Bool: data[0] = numericCast<bool>(value, type, true) ? 1 : 0; return;
      case Primitive::Int8: return store(data, numericCast<std::int8_t>(value, type, true));
      case Primitive::UInt8: return store(data, numericCast<std::uint8_t>(value, type, true));
      case Primitive::Int16: return store(data, numericCast<std::int16_t>(value, type, true));
      case Primitive::UInt16: return store(data, numericCast<std::uint16_t>(value, type, true));
      case Primitive::Int32: return store(data, numericCast<std::int32_t>(value, type, true));
      case Primitive::UInt32: return store(data, numericCast<std::uint32_t>(value, type, true));
      case Primitive::Int64: return store(data, numericCast<std::int64_t>(value, type, true));
      case Primitive::UInt64: return store(data, numericCast<std::uint64_t>(value, type, true));
      case Primitive::Real32: return store(data, numericCast<float>(value, type, true));
      case Primitive::Real64: return store(data, numericCast<double>(value, type, true));
      case Primitive::String:
      case Primitive::WString: break;
    }
  }
  throwNotNumeric(type);
}

}  // namespace detail

/// Read-only view of a PLC value: a type plus a pointer to the bytes laid out
/// as in the PLC. Cheap to copy. It does not own the memory, which must
/// outlive the view (the Value it comes from, or the notification buffer for
/// the duration of a callback).
class ConstValueRef {
 public:
  ConstValueRef() = default;
  ConstValueRef(const Type& type, const std::uint8_t* data) noexcept : type_(&type), data_(data) {}

  /// False for a default constructed (null) view.
  bool valid() const noexcept { return type_ != nullptr && data_ != nullptr; }
  const Type& type() const {
    if (!type_) detail::throwInvalidRef();
    return *type_;
  }
  TypePtr typePtr() const { return type().shared_from_this(); }
  const std::uint8_t* data() const noexcept { return data_; }
  /// Size in bytes.
  std::size_t size() const noexcept { return type_ ? type_->size() : 0; }
  /// Number of elements of an array (TypeError for other types).
  std::size_t length() const { return type().length(); }
  /// True if this is a structure with a field of that name.
  bool has(std::string_view field) const noexcept { return type_ && type_->findField(field) != nullptr; }

  /// Structure member (KeyError if there is no such field).
  ConstValueRef operator[](std::string_view field) const {
    const Field& f = type().field(field);
    return {*f.type, data_ + f.offset};
  }
  /// Array element, zero-based whatever the declared lower bound is.
  ConstValueRef operator[](std::size_t index) const;
  /// Nested element by path, e.g. "attr2.attr1", "attr3[1]" or "[3].attr2".
  ConstValueRef at(std::string_view path) const;

  /// Converts the value to T:
  ///  - arithmetic types (bool, int, double, ...): checked conversion of an
  ///    elementary value; throws RangeError if the value does not fit in T
  ///  - std::string: contents of a STRING (raw bytes) or WSTRING (UTF-8)
  ///  - std::vector<E> / std::array<E, N>: array elements converted to E
  ///  - Json: same as toJson()
  ///  - any other trivially copyable type (a C++ struct mirroring the PLC
  ///    structure, an enum, ...): byte copy; sizeof(T) must equal size()
  template <class T>
  T as() const;

  /// JSON representation: structures become objects, arrays become arrays,
  /// strings are converted to UTF-8 (the same shape as the Python version's
  /// dicts and lists).
  Json toJson() const;
  /// JSON text (compact when indent < 0).
  std::string toString(int indent = -1) const;

 protected:
  std::string stringValue() const;
  void checkRawSize(std::size_t size) const;

  const Type* type_ = nullptr;
  const std::uint8_t* data_ = nullptr;
};

/// Mutable view of a PLC value. Like a C++ reference it is bound when it is
/// created and assigning to it writes to the referenced bytes:
///
///     value["attr1"] = 42;              // elementary value
///     value["attr3"] = {true, false};   // array from a list
///     value["attr2"] = other["attr2"];  // copy of another value
class ValueRef : public ConstValueRef {
 public:
  ValueRef() = default;
  ValueRef(const Type& type, std::uint8_t* data) noexcept : ConstValueRef(type, data) {}
  ValueRef(const ValueRef&) = default;

  std::uint8_t* data() const noexcept { return const_cast<std::uint8_t*>(data_); }

  ValueRef operator[](std::string_view field) const {
    const Field& f = type().field(field);
    return {*f.type, data() + f.offset};
  }
  ValueRef operator[](std::size_t index) const;
  ValueRef at(std::string_view path) const;

  /// Sets the value from T: the inverse of ConstValueRef::as(). Values of
  /// other PLC types (Value, ValueRef, ...) are copied, converting between
  /// elementary types when needed.
  template <class T>
  void set(const T& value) const;

  /// Copies the contents of another value (see set()).
  ValueRef& operator=(const ValueRef& other) {
    assign(other);
    return *this;
  }
  template <class T>
  ValueRef& operator=(const T& value) {
    set(value);
    return *this;
  }
  /// Array elements from a list: `value["attr3"] = {true, false};`
  template <class T>
  ValueRef& operator=(std::initializer_list<T> list) {
    setElements(list.begin(), list.size());
    return *this;
  }

  /// Copies another value. Both must have the same layout, except elementary
  /// values (converted when the value fits) and strings (copied if they fit).
  void assign(ConstValueRef other) const;

  /// Sets the whole value from JSON (the same shape as toJson()). Every
  /// structure member and array element must be given. On error the value
  /// is left unchanged.
  void fromJson(const Json& json) const;
  /// Like fromJson() but members that are not given keep their value and
  /// arrays may be shorter (only the first elements are changed).
  void update(const Json& json) const;
  /// Sets all bytes to zero.
  void clear() const noexcept;

 private:
  void setString(std::string_view text) const;
  template <class E>
  void setElements(const E* elements, std::size_t count) const;
};

/// A PLC value that owns its memory: use it to create objects of custom
/// structures/types, read variables into, or write variables from.
///
///     beckhoff::Value obj = plc.createValue("CustomStruct2_s");  // zero-initialized
///     obj["attr1"] = 42;
///     obj["attr2"]["attr2"] = true;
///     obj["attr3"] = {true, false};
///     plc.setVariable("GVL.obj", obj);
class Value {
 public:
  /// Empty value (no type).
  Value() = default;
  /// Zero-initialized value of the given type.
  explicit Value(TypePtr type);
  /// Value of the given type initialized from JSON (see ValueRef::fromJson()):
  ///     Value v(type, {{"attr1", 5}, {"attr2", true}});
  Value(TypePtr type, const Json& json);
  /// Copy of the referenced value.
  explicit Value(ConstValueRef value);

  Value(const Value&) = default;
  Value(Value&&) noexcept = default;
  Value& operator=(const Value&) = default;
  Value& operator=(Value&&) noexcept = default;

  /// Assigning a PLC value (ValueRef, ConstValueRef) replaces this value with
  /// a copy, including its type. Anything else (numbers, strings, vectors,
  /// JSON, C++ structs) sets the contents, keeping the type (see ValueRef::set()).
  template <class T, std::enable_if_t<!std::is_same_v<T, Value>, int> = 0>
  Value& operator=(const T& value) {
    if constexpr (std::is_base_of_v<ConstValueRef, T>) {
      *this = Value(ConstValueRef(value));
    } else {
      ref().set(value);
    }
    return *this;
  }
  template <class T>
  Value& operator=(std::initializer_list<T> list) {
    ref() = list;
    return *this;
  }

  bool valid() const noexcept { return type_ != nullptr; }
  const Type& type() const { return ref().type(); }
  const TypePtr& typePtr() const noexcept { return type_; }
  std::uint8_t* data() noexcept { return bytes_.data(); }
  const std::uint8_t* data() const noexcept { return bytes_.data(); }
  std::size_t size() const noexcept { return bytes_.size(); }
  std::size_t length() const { return ref().length(); }
  bool has(std::string_view field) const noexcept { return ref().has(field); }

  ValueRef ref() noexcept { return type_ ? ValueRef(*type_, bytes_.data()) : ValueRef(); }
  ConstValueRef ref() const noexcept { return type_ ? ConstValueRef(*type_, bytes_.data()) : ConstValueRef(); }
  operator ValueRef() noexcept { return ref(); }
  operator ConstValueRef() const noexcept { return ref(); }

  ValueRef operator[](std::string_view field) { return ref()[field]; }
  ConstValueRef operator[](std::string_view field) const { return ref()[field]; }
  ValueRef operator[](std::size_t index) { return ref()[index]; }
  ConstValueRef operator[](std::size_t index) const { return ref()[index]; }
  ValueRef at(std::string_view path) { return ref().at(path); }
  ConstValueRef at(std::string_view path) const { return ref().at(path); }

  template <class T>
  T as() const {
    return ref().template as<T>();
  }
  template <class T>
  void set(const T& value) {
    ref().set(value);
  }

  void fromJson(const Json& json) { ref().fromJson(json); }
  void update(const Json& json) { ref().update(json); }
  Json toJson() const { return ref().toJson(); }
  std::string toString(int indent = -1) const { return ref().toString(indent); }
  void clear() noexcept { ref().clear(); }

 private:
  TypePtr type_;
  std::vector<std::uint8_t> bytes_;
};

/// Same type layout and same contents (padding bytes and bytes after the end
/// of a string are ignored).
bool operator==(const ConstValueRef& a, const ConstValueRef& b);
inline bool operator!=(const ConstValueRef& a, const ConstValueRef& b) { return !(a == b); }

/// Prints the value as compact JSON.
std::ostream& operator<<(std::ostream& out, const ConstValueRef& value);
std::ostream& operator<<(std::ostream& out, const Value& value);

// ---------------------------------------------------------------------------
// Template implementations

template <class T>
T ConstValueRef::as() const {
  using U = std::remove_cv_t<T>;
  if constexpr (std::is_arithmetic_v<U>) {
    return detail::readArithmetic<U>(type(), data_);
  } else if constexpr (std::is_same_v<U, std::string>) {
    return stringValue();
  } else if constexpr (std::is_same_v<U, Json>) {
    return toJson();
  } else if constexpr (detail::IsStdVector<U>::value) {
    const std::size_t n = length();
    U result;
    result.reserve(n);
    for (std::size_t i = 0; i < n; ++i) result.push_back((*this)[i].template as<typename U::value_type>());
    return result;
  } else if constexpr (detail::IsStdArray<U>::value) {
    U result{};
    if (length() != result.size()) {
      throw RangeError("cannot read " + type().name() + " into an array of " + std::to_string(result.size()) +
                       " elements");
    }
    for (std::size_t i = 0; i < result.size(); ++i) result[i] = (*this)[i].template as<typename U::value_type>();
    return result;
  } else if constexpr (detail::isRawCopyable<U> && std::is_default_constructible_v<U>) {
    checkRawSize(sizeof(U));
    U result;
    std::memcpy(static_cast<void*>(&result), data_, sizeof(U));
    return result;
  } else {
    static_assert(detail::alwaysFalse<T>, "unsupported type for ConstValueRef::as<T>()");
  }
}

template <class T>
void ValueRef::set(const T& value) const {
  using U = std::decay_t<T>;
  if constexpr (std::is_arithmetic_v<U>) {
    detail::writeArithmetic(type(), data(), value);
  } else if constexpr (detail::isStringLike<U>) {
    setString(std::string_view(value));
  } else if constexpr (std::is_same_v<U, Json>) {
    fromJson(value);
  } else if constexpr (std::is_base_of_v<ConstValueRef, U> || std::is_same_v<U, Value>) {
    assign(static_cast<ConstValueRef>(value));
  } else if constexpr (detail::IsStdVector<U>::value || detail::IsStdArray<U>::value) {
    const std::size_t n = length();
    if (value.size() != n) {
      throw RangeError(type().name() + " has " + std::to_string(n) + " elements, got " +
                       std::to_string(value.size()));
    }
    for (std::size_t i = 0; i < n; ++i) (*this)[i].set(static_cast<typename U::value_type>(value[i]));
  } else if constexpr (detail::isRawCopyable<U>) {
    checkRawSize(sizeof(U));
    std::memcpy(data(), static_cast<const void*>(&value), sizeof(U));
  } else {
    static_assert(detail::alwaysFalse<T>, "unsupported type for ValueRef::set()");
  }
}

template <class E>
void ValueRef::setElements(const E* elements, std::size_t count) const {
  const std::size_t n = length();
  if (count != n) {
    throw RangeError(type().name() + " has " + std::to_string(n) + " elements, got " + std::to_string(count));
  }
  for (std::size_t i = 0; i < n; ++i) (*this)[i].set(elements[i]);
}

}  // namespace beckhoff
