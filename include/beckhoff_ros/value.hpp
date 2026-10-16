#ifndef BECKHOFF_ROS__VALUE_HPP_
#define BECKHOFF_ROS__VALUE_HPP_

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace beckhoff_ros
{

/// Raised when a value cannot be parsed or does not fit a PLC data type
/// (the Python version raised TypeError/ValueError in these cases).
class ValueError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Dynamically typed value exchanged with the PLC.
///
/// It models the Python objects the Python version of this package worked with: None, bool,
/// int, float, str, list and dict (insertion ordered). PLC structures are converted to dicts,
/// arrays to lists and elementary types to bool/int/float, exactly like the Python version did.
/// str(), repr() and typeString() produce the same text as Python's str(), repr() and
/// str(type()), so the messages published by the node are unchanged.
class Value
{
public:
  using List = std::vector<Value>;
  using Dict = std::vector<std::pair<std::string, Value>>;

  enum class Type { None, Bool, Int, Float, String, List, Dict };

  Value() = default;
  Value(std::nullptr_t) {}  // NOLINT(runtime/explicit)
  Value(bool value)  // NOLINT(runtime/explicit)
  : data_(value) {}

  template<
    typename T,
    typename std::enable_if<
      std::is_integral<T>::value && !std::is_same<T, bool>::value, int>::type = 0>
  Value(T value)  // NOLINT(runtime/explicit)
  {
    if (std::is_signed<T>::value ||
      static_cast<std::uint64_t>(value) <=
      static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
    {
      data_ = static_cast<std::int64_t>(value);
    } else {
      data_ = static_cast<std::uint64_t>(value);
    }
  }

  Value(double value)  // NOLINT(runtime/explicit)
  : data_(value) {}
  Value(float value)  // NOLINT(runtime/explicit)
  : data_(static_cast<double>(value)) {}
  Value(const char * value)  // NOLINT(runtime/explicit)
  : data_(std::string(value)) {}
  Value(std::string value)  // NOLINT(runtime/explicit)
  : data_(std::move(value)) {}
  Value(List value)  // NOLINT(runtime/explicit)
  : data_(std::move(value)) {}
  Value(Dict value)  // NOLINT(runtime/explicit)
  : data_(std::move(value)) {}

  /// Builds a list, e.g. Value::list({1, 2, 3}).
  static Value list(std::initializer_list<Value> items);
  /// Builds a dict, e.g. Value::dict({{"attr1", 5}, {"attr2", true}}).
  static Value dict(std::initializer_list<std::pair<std::string, Value>> items);

  /// Parses a Python literal, like ast.literal_eval() (dict, list, tuple, str, int, float,
  /// True/False/None). JSON (true/false/null, double quoted strings) is accepted as well,
  /// so does inf/nan. Tuples become lists.
  static Value parseLiteral(const std::string & text);

  Type type() const;
  bool isNone() const {return type() == Type::None;}
  bool isBool() const {return type() == Type::Bool;}
  bool isInt() const {return type() == Type::Int;}
  bool isFloat() const {return type() == Type::Float;}
  bool isString() const {return type() == Type::String;}
  bool isList() const {return type() == Type::List;}
  bool isDict() const {return type() == Type::Dict;}

  bool asBool() const;
  /// Integer value; throws ValueError if it does not fit an int64.
  std::int64_t asInt() const;
  /// Integer value; throws ValueError if it is negative.
  std::uint64_t asUInt() const;
  /// Numeric value as double (bool, int and float are accepted, like Python's float()).
  double asFloat() const;
  const std::string & asString() const;
  const List & asList() const;
  List & asList();
  const Dict & asDict() const;
  Dict & asDict();

  /// Dict lookup; nullptr if this is not a dict or the key does not exist.
  const Value * find(const std::string & key) const;
  /// Dict item; throws ValueError if the key does not exist (items are not added, so a
  /// misspelled field name is an error), e.g. robot["var2"][3]["attr1"] = 42.
  const Value & operator[](const std::string & key) const;
  Value & operator[](const std::string & key);
  /// List item; throws ValueError if out of range.
  const Value & operator[](std::size_t index) const;
  Value & operator[](std::size_t index);

  /// Same text as Python's str(value).
  std::string str() const;
  /// Same text as Python's repr(value).
  std::string repr() const;
  /// Same text as Python's str(type(value)), e.g. "<class 'dict'>".
  std::string typeString() const;

  /// Exact structural comparison (same types, same order of dict items).
  friend bool operator==(const Value & lhs, const Value & rhs);
  friend bool operator!=(const Value & lhs, const Value & rhs) {return !(lhs == rhs);}

private:
  // Integers are stored as int64, except values above INT64_MAX that are stored as uint64.
  std::variant<
    std::monostate, bool, std::int64_t, std::uint64_t, double, std::string, List, Dict> data_;
};

/// Converts the textual value of a WritePlcVariable request to a Value, according to its type.
///
/// The Python version evaluated `locate(type)(value)`. Supported types:
///  - "bool": like Python's bool(int(value)); "true"/"false" are accepted as well
///  - "int": like Python's int(value)
///  - "float": like Python's float(value)
///  - "dict", "list", "tuple": the value is parsed as a Python literal (see parseLiteral), which
///    allows writing structures and arrays
/// The type may also be given as published in PlcVariable messages, e.g. "<class 'dict'>".
/// Throws ValueError for unsupported types or invalid values.
Value parseTypedValue(const std::string & type, const std::string & text);

/// Formats a double like Python's repr(float).
std::string formatPythonFloat(double value);

/// Formats a string like Python's repr(str).
std::string formatPythonString(const std::string & value);

}  // namespace beckhoff_ros

#endif  // BECKHOFF_ROS__VALUE_HPP_
