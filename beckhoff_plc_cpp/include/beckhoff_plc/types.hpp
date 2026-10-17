#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace beckhoff {

class Type;

/// Types are immutable and shared by values, variables and registries.
using TypePtr = std::shared_ptr<const Type>;

/// Elementary PLC data types (IEC 61131-3 / TwinCAT).
enum class Primitive : std::uint8_t {
  Bool,     ///< BOOL (1 byte)
  Int8,     ///< SINT
  UInt8,    ///< USINT, BYTE
  Int16,    ///< INT
  UInt16,   ///< UINT, WORD
  Int32,    ///< DINT
  UInt32,   ///< UDINT, DWORD, TIME, TOD, DATE, DT
  Int64,    ///< LINT
  UInt64,   ///< ULINT, LWORD, LTIME
  Real32,   ///< REAL
  Real64,   ///< LREAL
  String,   ///< STRING(n): up to n Latin-1 characters plus a NUL terminator
  WString,  ///< WSTRING(n): up to n UTF-16 code units plus a NUL terminator
};

/// A member of a structure type.
struct Field {
  std::string name;
  TypePtr type;
  std::size_t offset = 0;  ///< byte offset inside the structure
};

/// Declaration of a structure member: field name and type specification
/// (see TypeRegistry::parse()).
struct FieldDefinition {
  std::string name;
  std::string type;
};

/// Declaration of a structure (DUT). Fields are laid out in declaration order.
struct StructDefinition {
  std::string name;
  std::vector<FieldDefinition> fields;
  /// Alignment of this structure, like {attribute 'pack_mode' := 'n'} in
  /// TwinCAT. Uses the registry default when empty.
  std::optional<int> packMode;
};

/// Declaration of an alternative name for a type, e.g. an enumeration
/// ("E_State" -> "INT") or `TYPE T_Name : STRING(50); END_TYPE`.
struct AliasDefinition {
  std::string name;
  std::string type;
};

/// Description of a PLC data type and of its memory layout.
///
/// A type is either a primitive (BOOL, INT, REAL, STRING(n), ...), an array
/// (ARRAY[a..b] OF T) or a structure. Layouts follow the TwinCAT rules: every
/// member is aligned to min(natural alignment, pack mode) and a structure is
/// padded to a multiple of its alignment.
class Type : public std::enable_shared_from_this<Type> {
 public:
  enum class Kind : std::uint8_t { Primitive, Array, Struct };

  /// Elementary type. STRING/WSTRING get the default length (80), see
  /// makeString()/makeWString() for other lengths.
  static TypePtr makePrimitive(Primitive primitive);
  /// STRING(length): length characters plus the NUL terminator.
  static TypePtr makeString(std::size_t length = 80);
  /// WSTRING(length): length UTF-16 code units plus the NUL terminator.
  static TypePtr makeWString(std::size_t length = 80);
  /// ARRAY[lowerBound..lowerBound + length - 1] OF element. The lower bound is
  /// informative only: element access is always zero-based.
  static TypePtr makeArray(TypePtr element, std::size_t length, std::int64_t lowerBound = 0);
  /// Structure with the given members (in memory order). packMode is 1, 2, 4
  /// or 8 (0 is treated as 1, like in TwinCAT).
  static TypePtr makeStruct(std::string name, const std::vector<std::pair<std::string, TypePtr>>& fields,
                            int packMode = 8);

  Kind kind() const noexcept { return kind_; }
  bool isPrimitive() const noexcept { return kind_ == Kind::Primitive; }
  bool isArray() const noexcept { return kind_ == Kind::Array; }
  bool isStruct() const noexcept { return kind_ == Kind::Struct; }
  /// STRING(n) or WSTRING(n).
  bool isString() const noexcept {
    return kind_ == Kind::Primitive && (primitive_ == Primitive::String || primitive_ == Primitive::WString);
  }

  /// TwinCAT style name: "UINT", "STRING(80)", "ARRAY[0..9] OF ST_Foo", "ST_Foo".
  const std::string& name() const noexcept { return name_; }
  /// Size in bytes (including padding).
  std::size_t size() const noexcept { return size_; }
  /// Alignment in bytes.
  std::size_t alignment() const noexcept { return alignment_; }

  /// Kind of primitive. Throws TypeError if this is not a primitive.
  Primitive primitive() const;
  /// Maximum number of characters of a STRING/WSTRING. Throws TypeError otherwise.
  std::size_t stringLength() const;

  /// Element type of an array. Throws TypeError if this is not an array.
  const Type& elementType() const;
  const TypePtr& elementTypePtr() const;
  /// Number of elements of an array. Throws TypeError if this is not an array.
  std::size_t length() const;
  /// Declared lower bound of an array (informative).
  std::int64_t lowerBound() const;

  /// Members of a structure. Throws TypeError if this is not a structure.
  const std::vector<Field>& fields() const;
  /// Member by name (exact match first, then case-insensitive like TwinCAT).
  /// Returns nullptr if not found or if this is not a structure.
  const Field* findField(std::string_view name) const noexcept;
  /// Like findField() but throws TypeError/KeyError instead of returning nullptr.
  const Field& field(std::string_view name) const;
  /// Pack mode of a structure.
  int packMode() const;

  /// True if both types have the same memory layout and interpretation
  /// (same primitives, array lengths, member names and offsets).
  bool sameLayout(const Type& other) const noexcept;

  /// Multi-line description of the memory layout (offsets, sizes); handy to
  /// debug mismatches with the PLC.
  std::string layout() const;

  struct Private;
  explicit Type(const Private&) {}  // use the make*() factories

 private:
  Kind kind_ = Kind::Primitive;
  Primitive primitive_ = Primitive::Bool;
  std::string name_;
  std::size_t size_ = 0;
  std::size_t alignment_ = 1;
  std::size_t length_ = 0;  // array length or string length
  TypePtr element_;
  std::int64_t lowerBound_ = 0;
  std::vector<Field> fields_;
  int packMode_ = 8;
};

/// Set of named types: the built-in elementary types plus the structures and
/// aliases defined by the user (usually loaded from the parameter file).
///
/// Name lookups are case-insensitive, like in TwinCAT. Besides the IEC names
/// (BOOL, INT, UDINT, LREAL, ...) the names used by the Python version are
/// accepted (bool, int, int16, uint16, uint8, real) as well as C-like ones
/// (int8 .. int64, uint8 .. uint64, float, double). Note that, as in TwinCAT
/// and in the Python version, "int" is a 16-bit integer.
class TypeRegistry {
 public:
  explicit TypeRegistry(int defaultPackMode = 8);

  /// Pack mode used by structures that do not specify one (TwinCAT 3 uses 8).
  int defaultPackMode() const noexcept { return defaultPackMode_; }
  void setDefaultPackMode(int packMode);

  /// Named type (built-in, structure or alias), or nullptr if unknown.
  TypePtr find(std::string_view name) const;
  bool contains(std::string_view name) const { return find(name) != nullptr; }

  /// Parses a type specification:
  ///  - a type name: "UINT", "bool", "CustomStruct1_s"
  ///  - a string: "STRING" (80 characters), "STRING(20)", "STRING[20]", "WSTRING(10)"
  ///  - an array: "ARRAY[0..9] OF CustomStruct1_s", "ARRAY[1..2, 1..3] OF INT"
  ///    (multi-dimensional arrays are arrays of arrays, last index fastest)
  /// Throws ConfigError on syntax errors and KeyError on unknown type names.
  TypePtr parse(std::string_view spec) const;

  /// Defines a structure. Referenced types must already be known.
  TypePtr defineStruct(const StructDefinition& definition);
  /// Defines an alias for a type specification.
  TypePtr defineAlias(const std::string& name, std::string_view spec);
  /// Defines several aliases and structures at once. They may reference each
  /// other in any order (dependencies are resolved automatically). Either all
  /// definitions are added or, on error, none.
  void define(const std::vector<AliasDefinition>& aliases, const std::vector<StructDefinition>& structures);

  /// User defined structures, in definition order.
  std::vector<TypePtr> structures() const;

 private:
  struct Entry {
    TypePtr type;
    bool builtin = false;
  };
  struct Resolver;
  void addNamed(const std::string& name, TypePtr type);

  std::unordered_map<std::string, Entry> named_;  // key: lower-case name
  std::vector<TypePtr> structures_;
  int defaultPackMode_ = 8;
};

}  // namespace beckhoff
