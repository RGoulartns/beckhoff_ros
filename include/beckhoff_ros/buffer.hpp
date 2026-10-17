#ifndef BECKHOFF_ROS__BUFFER_HPP_
#define BECKHOFF_ROS__BUFFER_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "beckhoff_ros/plc_types.hpp"
#include "beckhoff_ros/value.hpp"

namespace beckhoff_ros
{

namespace detail
{

/// Primitive stored as the C++ type T (Field<T>).
template<typename T>
struct PrimitiveOf;
template<>
struct PrimitiveOf<bool> {static constexpr Primitive value = Primitive::Bool;};
template<>
struct PrimitiveOf<std::int8_t> {static constexpr Primitive value = Primitive::Int8;};
template<>
struct PrimitiveOf<std::uint8_t> {static constexpr Primitive value = Primitive::UInt8;};
template<>
struct PrimitiveOf<std::int16_t> {static constexpr Primitive value = Primitive::Int16;};
template<>
struct PrimitiveOf<std::uint16_t> {static constexpr Primitive value = Primitive::UInt16;};
template<>
struct PrimitiveOf<std::int32_t> {static constexpr Primitive value = Primitive::Int32;};
template<>
struct PrimitiveOf<std::uint32_t> {static constexpr Primitive value = Primitive::UInt32;};
template<>
struct PrimitiveOf<std::int64_t> {static constexpr Primitive value = Primitive::Int64;};
template<>
struct PrimitiveOf<std::uint64_t> {static constexpr Primitive value = Primitive::UInt64;};
template<>
struct PrimitiveOf<float> {static constexpr Primitive value = Primitive::Real32;};
template<>
struct PrimitiveOf<double> {static constexpr Primitive value = Primitive::Real64;};

template<typename T>
struct IsFieldType : std::integral_constant<
    bool,
    std::is_same<T, bool>::value ||
    std::is_same<T, std::int8_t>::value || std::is_same<T, std::uint8_t>::value ||
    std::is_same<T, std::int16_t>::value || std::is_same<T, std::uint16_t>::value ||
    std::is_same<T, std::int32_t>::value || std::is_same<T, std::uint32_t>::value ||
    std::is_same<T, std::int64_t>::value || std::is_same<T, std::uint64_t>::value ||
    std::is_same<T, float>::value || std::is_same<T, double>::value>
{};

/// Position of an item inside the data of a type.
struct Location
{
  const PlcType * type;
  std::size_t offset;
};

Location locateField(const PlcType & type, const char * name, std::size_t length);
Location locateIndex(const PlcType & type, std::size_t index);
Location locatePath(const PlcType & root, std::string_view path);
/// Throws ValueError unless `type` is the same type as `expected` (the same object, or a type
/// with the same name and size, e.g. from another BeckhoffPLC loaded from the same file).
void checkSameType(const PlcType & type, const PlcType & expected);
[[noreturn]] void throwNotPrimitive(const PlcType & type);
[[noreturn]] void throwNotConvertible(const PlcType & type, const Value & value, const char * to);

template<typename T>
inline T loadLittleEndian(const std::uint8_t * data)
{
  T value;
  std::memcpy(&value, data, sizeof(T));
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  auto * bytes = reinterpret_cast<unsigned char *>(&value);
  for (std::size_t i = 0; i < sizeof(T) / 2; ++i) {
    std::swap(bytes[i], bytes[sizeof(T) - 1 - i]);
  }
#endif
  return value;
}

template<typename T>
inline void storeLittleEndian(std::uint8_t * data, T value)
{
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  auto * bytes = reinterpret_cast<unsigned char *>(&value);
  for (std::size_t i = 0; i < sizeof(T) / 2; ++i) {
    std::swap(bytes[i], bytes[sizeof(T) - 1 - i]);
  }
#endif
  std::memcpy(data, &value, sizeof(T));
}

}  // namespace detail

/// Read-only view of PLC data of a known type: a whole variable, or an item of it (structure
/// field, array element). It does not own the data.
///
/// Items are reached by field name and index, like Value: view["var2"][3]["attr1"], or with a
/// path: view.at("var2[3].attr1"). Looking up a field compares its name with the fields of the
/// structure: in a loop, prefer a Field<T>, resolved once.
class ConstView
{
public:
  ConstView() = default;
  ConstView(const PlcType & type, const std::uint8_t * data)
  : type_(&type), data_(data) {}

  const PlcType & type() const {return *type_;}
  const std::uint8_t * data() const {return data_;}
  std::size_t size() const {return type_->size();}

  /// Field of a structure; throws ValueError if this is not a structure or has no such field.
  ConstView operator[](std::string_view field) const
  {
    const auto location = detail::locateField(*type_, field.data(), field.size());
    return ConstView(*location.type, data_ + location.offset);
  }
  /// Element of an array; throws ValueError if this is not an array or index is out of range.
  ConstView operator[](std::size_t index) const
  {
    const auto location = detail::locateIndex(*type_, index);
    return ConstView(*location.type, data_ + location.offset);
  }
  /// Item at a path made of field names and indexes, e.g. "var2[3].attr1" ("[3].attr1" for an
  /// array). Throws ValueError if there is no such item.
  ConstView at(std::string_view path) const
  {
    const auto location = detail::locatePath(*type_, path);
    return ConstView(*location.type, data_ + location.offset);
  }

  /// Elementary value converted to T (bool, an integer or a floating point type). Throws
  /// ValueError if this is not an elementary value or if it does not fit T: integers are range
  /// checked, REAL/LREAL values cannot be read as integers or bool.
  template<typename T>
  T as() const;

  /// Converts the data to a Value: a dict for a structure, a list for an array.
  Value value() const {return type_->decode(data_);}

protected:
  const PlcType * type_ = nullptr;
  const std::uint8_t * data_ = nullptr;
};

/// Modifiable view of PLC data: like ConstView, and its items can be assigned.
///
///   buffer["var1"] = true;
///   buffer["var2"][3]["attr1"] = 42;
///   buffer["var3"] = Value::dict({...});
///
/// Assignments are checked like setVariable: a ValueError is thrown (and the data is not
/// changed) if the value does not fit the type, e.g. 70000 for a UINT, or 1.5 for an INT.
class View : public ConstView
{
public:
  View() = default;
  View(const PlcType & type, std::uint8_t * data)
  : ConstView(type, data) {}
  View(const View &) = default;

  std::uint8_t * data() const {return const_cast<std::uint8_t *>(data_);}

  View operator[](std::string_view field) const
  {
    const auto location = detail::locateField(*type_, field.data(), field.size());
    return View(*location.type, data() + location.offset);
  }
  View operator[](std::size_t index) const
  {
    const auto location = detail::locateIndex(*type_, index);
    return View(*location.type, data() + location.offset);
  }
  View at(std::string_view path) const
  {
    const auto location = detail::locatePath(*type_, path);
    return View(*location.type, data() + location.offset);
  }

  /// Sets an elementary value.
  template<
    typename T, typename std::enable_if<std::is_arithmetic<T>::value, int>::type = 0>
  const View & operator=(T value) const
  {
    set(Value(value));
    return *this;
  }
  /// Sets the item from a Value: a dict for a structure (with every field), a list for an
  /// array (missing elements are zeroed).
  const View & operator=(const Value & value) const
  {
    set(value);
    return *this;
  }
  /// Copies the data of an item of the same type.
  const View & operator=(const ConstView & other) const;
  const View & operator=(const View & other) const
  {
    return *this = static_cast<const ConstView &>(other);
  }

  void set(const Value & value) const {type_->encode(value, data(), "value");}
};

/// Data of a PLC variable (or of any PLC type): its bytes, as transmitted by ADS.
///
/// Reading and writing variables with Buffers avoids the conversions to and from Value: the
/// data is copied as is, and no memory is allocated once the Buffer exists.
class Buffer
{
public:
  Buffer() = default;
  /// Zero initialized data of `type`.
  explicit Buffer(PlcTypePtr type);
  /// Data of `type` holding `value`; throws ValueError if it does not fit.
  Buffer(PlcTypePtr type, const Value & value);

  const PlcType & type() const {return *type_;}
  const PlcTypePtr & typePtr() const {return type_;}
  std::uint8_t * data() {return data_.data();}
  const std::uint8_t * data() const {return data_.data();}
  std::size_t size() const {return data_.size();}
  const std::vector<std::uint8_t> & bytes() const {return data_;}

  View view() {return View(*type_, data_.data());}
  ConstView view() const {return ConstView(*type_, data_.data());}
  View operator[](std::string_view field) {return view()[field];}
  ConstView operator[](std::string_view field) const {return view()[field];}
  View operator[](std::size_t index) {return view()[index];}
  ConstView operator[](std::size_t index) const {return view()[index];}
  View at(std::string_view path) {return view().at(path);}
  ConstView at(std::string_view path) const {return view().at(path);}

  /// Elementary value of the buffer (for variables of an elementary type).
  template<typename T>
  T as() const {return view().template as<T>();}

  /// Converts the data to a Value.
  Value value() const {return type_->decode(data_.data());}
  /// Replaces the data; throws ValueError (and keeps the data) if the value does not fit.
  void setValue(const Value & value) {type_->encode(value, data_.data(), type_->name());}
  /// Sets every byte to zero.
  void clear();

  friend bool operator==(const Buffer & lhs, const Buffer & rhs);
  friend bool operator!=(const Buffer & lhs, const Buffer & rhs) {return !(lhs == rhs);}

private:
  PlcTypePtr type_;
  std::vector<std::uint8_t> data_;
};

/// Typed accessor of an elementary item, resolved once: reading or writing it is a copy at a
/// fixed offset, without name lookups, conversions or allocations.
///
///   const beckhoff_ros::Field<std::uint16_t> attr1(robot.type(), "var2[3].attr1");
///   attr1.set(buffer, 42);
///   std::uint16_t value = attr1.get(buffer);
///
/// T is the C++ type of the PLC type: bool (BOOL), std::int8_t (SINT), std::uint8_t (USINT,
/// BYTE), std::int16_t (INT), std::uint16_t (UINT, WORD), std::int32_t (DINT),
/// std::uint32_t (UDINT, DWORD), std::int64_t (LINT), std::uint64_t (ULINT, LWORD),
/// float (REAL) or double (LREAL).
template<typename T>
class Field
{
  static_assert(
    detail::IsFieldType<T>::value,
    "Field<T>: T must be bool, std::int8_t ... std::uint64_t, float or double");

public:
  Field() = default;
  /// The item at `path` (e.g. "var2[3].attr1", or "" for an elementary type) of `type`. Throws
  /// ValueError if there is no such item or if its type does not correspond to T.
  Field(const PlcType & type, std::string_view path)
  : root_(&type)
  {
    const auto location = detail::locatePath(type, path);
    if (location.type->kind() != PlcType::Kind::Primitive ||
      location.type->primitiveType() != detail::PrimitiveOf<T>::value)
    {
      throwWrongType(*location.type, path);
    }
    offset_ = location.offset;
  }
  Field(const PlcTypePtr & type, std::string_view path)
  : Field(* type, path) {}

  /// Value in `buffer`, which must hold data of the type the field was created for (throws
  /// ValueError otherwise).
  T get(const Buffer & buffer) const {return get(buffer.view());}
  void set(Buffer & buffer, T value) const {set(buffer.view(), value);}
  /// Same, for a view of the whole data (e.g. the data given to a notification callback).
  T get(const ConstView & data) const
  {
    check(data.type());
    if constexpr (std::is_same<T, bool>::value) {
      return data.data()[offset_] != 0;
    } else {
      return detail::loadLittleEndian<T>(data.data() + offset_);
    }
  }
  void set(const View & data, T value) const
  {
    check(data.type());
    if constexpr (std::is_same<T, bool>::value) {
      data.data()[offset_] = value ? 1 : 0;
    } else {
      detail::storeLittleEndian<T>(data.data() + offset_, value);
    }
  }

  std::size_t offset() const {return offset_;}
  explicit operator bool() const {return root_ != nullptr;}

private:
  void check(const PlcType & type) const
  {
    if (&type != root_) {
      if (root_ == nullptr) {
        throw ValueError("Field used before being initialized");
      }
      detail::checkSameType(type, *root_);
    }
  }

  [[noreturn]] static void throwWrongType(const PlcType & type, std::string_view path);

  const PlcType * root_ = nullptr;
  std::size_t offset_ = 0;
};

template<typename T>
void Field<T>::throwWrongType(const PlcType & type, std::string_view path)
{
  throw ValueError(
          "Field '" + std::string(path) + "' is of type '" + type.name() +
          "', which does not correspond to the C++ type of the Field");
}

template<typename T>
T ConstView::as() const
{
  static_assert(std::is_arithmetic<T>::value, "as<T>(): T must be an arithmetic type");
  if (type_->kind() != PlcType::Kind::Primitive) {
    detail::throwNotPrimitive(*type_);
  }
  const Value value = type_->decode(data_);
  if constexpr (std::is_same<T, bool>::value) {
    if (value.isFloat()) {
      detail::throwNotConvertible(*type_, value, "bool");
    }
    return value.isBool() ? value.asBool() : value != Value(0);
  } else if constexpr (std::is_integral<T>::value) {
    if (value.isFloat()) {
      detail::throwNotConvertible(*type_, value, "an integer");
    }
    if (value.isBool()) {
      return static_cast<T>(value.asBool() ? 1 : 0);
    }
    if (value.asFloat() < 0) {
      const std::int64_t number = value.asInt();
      if (!std::is_signed<T>::value ||
        number < static_cast<std::int64_t>(std::numeric_limits<T>::min()))
      {
        detail::throwNotConvertible(*type_, value, "the requested integer type");
      }
      return static_cast<T>(number);
    }
    const std::uint64_t number = value.asUInt();
    if (number > static_cast<std::uint64_t>(std::numeric_limits<T>::max())) {
      detail::throwNotConvertible(*type_, value, "the requested integer type");
    }
    return static_cast<T>(number);
  } else {
    const double number = value.asFloat();
    if (std::is_same<T, float>::value && std::isfinite(number) &&
      std::fabs(number) > static_cast<double>(std::numeric_limits<float>::max()))
    {
      detail::throwNotConvertible(*type_, value, "float");
    }
    return static_cast<T>(number);
  }
}

}  // namespace beckhoff_ros

#endif  // BECKHOFF_ROS__BUFFER_HPP_
