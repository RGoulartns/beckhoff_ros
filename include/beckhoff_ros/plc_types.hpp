#ifndef BECKHOFF_ROS__PLC_TYPES_HPP_
#define BECKHOFF_ROS__PLC_TYPES_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"

#include "beckhoff_ros/value.hpp"

namespace beckhoff_ros
{

/// Raised for invalid structure/variable definitions in the configuration
/// (the Python version raised TypeError/KeyError in these cases).
class ConfigError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Elementary PLC data types (the ctypes types of the Python version).
enum class Primitive
{
  Bool,     // ctypes.c_bool
  Int8,     // ctypes.c_int8
  UInt8,    // ctypes.c_uint8
  Int16,    // ctypes.c_int16
  UInt16,   // ctypes.c_uint16
  Int32,    // ctypes.c_int32
  UInt32,   // ctypes.c_uint32
  Int64,    // ctypes.c_int64
  UInt64,   // ctypes.c_uint64
  Real32,   // ctypes.c_float
  Real64,   // ctypes.c_double
};

class PlcType;
using PlcTypePtr = std::shared_ptr<const PlcType>;

/// Field of a structure, with its byte offset inside the structure.
struct PlcField
{
  std::string name;
  PlcTypePtr type;
  std::size_t offset;
};

/// Description and memory layout of a PLC data type: an elementary type, an array or a
/// structure (DUT).
///
/// This is the C++ counterpart of the ctypes types the Python version created at runtime
/// (ctypes primitives, `ctype * size` arrays and ctypes.Structure subclasses). The memory layout
/// follows the same rules as ctypes: with pack == 0 every member is naturally aligned, which is
/// also how TwinCAT 3 lays out structures by default; pack > 0 limits the alignment like
/// `_pack_` (TwinCAT's {attribute 'pack_mode' := '<pack>'}).
/// Data is always encoded little endian, as transmitted by ADS.
class PlcType
{
public:
  enum class Kind { Primitive, Array, Struct };

  static PlcTypePtr primitive(std::string name, Primitive primitive);
  /// Array of `length` elements (`ctype * length` in the Python version).
  static PlcTypePtr array(PlcTypePtr element, std::size_t length);
  /// Structure with the given fields, in order (a ctypes.Structure with `_fields_` and `_pack_`).
  static PlcTypePtr structure(
    std::string name, const std::vector<std::pair<std::string, PlcTypePtr>> & fields,
    std::size_t pack = 0);

  Kind kind() const {return kind_;}
  const std::string & name() const {return name_;}
  /// Size in bytes (ctypes.sizeof).
  std::size_t size() const {return size_;}
  std::size_t alignment() const {return alignment_;}

  /// Only for Kind::Primitive.
  Primitive primitiveType() const {return primitive_;}
  /// Only for Kind::Array.
  const PlcTypePtr & elementType() const {return element_;}
  std::size_t length() const {return length_;}
  /// Only for Kind::Struct.
  const std::vector<PlcField> & fields() const {return fields_;}

  /// Converts PLC data to a Value (cType2Python in the Python version): structures become
  /// dicts, arrays become lists and elementary types become bool/int/float.
  /// `data` must hold size() bytes.
  Value decode(const std::uint8_t * data) const;
  Value decode(const std::vector<std::uint8_t> & data) const;

  /// Converts a Value to PLC data (python2CType in the Python version). Structures are given as
  /// dicts containing every field (other keys are ignored), arrays as lists with at most
  /// length() items (missing items are zeroed, like ctypes does). Throws ValueError if the value
  /// does not fit this type; `name` is used in its message.
  std::vector<std::uint8_t> encode(const Value & value, const std::string & name = "value") const;

private:
  PlcType() = default;
  void encodeInto(const Value & value, std::uint8_t * out, const std::string & path) const;

  Kind kind_ = Kind::Primitive;
  std::string name_;
  std::size_t size_ = 0;
  std::size_t alignment_ = 1;
  Primitive primitive_ = Primitive::Bool;
  PlcTypePtr element_;
  std::size_t length_ = 0;
  std::vector<PlcField> fields_;
};

/// Type names available in the configuration (BeckhoffPLC.cTypes in the Python version).
///
/// It starts with the elementary types (see plc_types.cpp to add more names) and grows with
/// the structures defined in the configuration file.
class TypeRegistry
{
public:
  TypeRegistry();

  /// Type registered under `name`, or nullptr.
  PlcTypePtr find(const std::string & name) const;
  /// Type registered under `name`; throws ConfigError if there is none.
  PlcTypePtr get(const std::string & name) const;

  /// Resolves a type specification of the configuration file: either a type name ("uint16")
  /// or an array given as [type name, size] (["uint16", 5]).
  PlcTypePtr resolve(const nlohmann::ordered_json & spec) const;

  /// Creates a structure from its JSON definition ({"field": type specification, ...}) and
  /// registers it (createCTypeStruct + loadStructures in the Python version). Fields are laid
  /// out in the order they appear in the file, so only structures defined earlier can be used.
  PlcTypePtr addStructure(
    const std::string & name, const nlohmann::ordered_json & fields, std::size_t pack = 0);

  /// Registers a type under a name; throws ConfigError if the name is already used.
  void add(const std::string & name, PlcTypePtr type);

private:
  std::map<std::string, PlcTypePtr> types_;
};

}  // namespace beckhoff_ros

#endif  // BECKHOFF_ROS__PLC_TYPES_HPP_
