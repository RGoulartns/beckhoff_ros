#include "beckhoff_plc/types.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <sstream>

#include "beckhoff_plc/errors.hpp"
#include "util.hpp"

namespace beckhoff {

struct Type::Private {};

namespace {

// ADS transfers lengths as 32-bit values; keep well below that.
constexpr std::size_t kMaxTypeSize = 0x7FFFFFFF;

const char* primitiveName(Primitive primitive) {
  switch (primitive) {
    case Primitive::Bool: return "BOOL";
    case Primitive::Int8: return "SINT";
    case Primitive::UInt8: return "USINT";
    case Primitive::Int16: return "INT";
    case Primitive::UInt16: return "UINT";
    case Primitive::Int32: return "DINT";
    case Primitive::UInt32: return "UDINT";
    case Primitive::Int64: return "LINT";
    case Primitive::UInt64: return "ULINT";
    case Primitive::Real32: return "REAL";
    case Primitive::Real64: return "LREAL";
    case Primitive::String: return "STRING";
    case Primitive::WString: return "WSTRING";
  }
  return "?";
}

std::size_t primitiveSize(Primitive primitive) {
  switch (primitive) {
    case Primitive::Bool:
    case Primitive::Int8:
    case Primitive::UInt8: return 1;
    case Primitive::Int16:
    case Primitive::UInt16: return 2;
    case Primitive::Int32:
    case Primitive::UInt32:
    case Primitive::Real32: return 4;
    case Primitive::Int64:
    case Primitive::UInt64:
    case Primitive::Real64: return 8;
    case Primitive::String:
    case Primitive::WString: break;
  }
  return 0;
}

int normalizePackMode(int packMode) {
  if (packMode == 0) return 1;  // TwinCAT: 0 and 1 both mean "no gaps"
  if (packMode == 1 || packMode == 2 || packMode == 4 || packMode == 8) return packMode;
  throw ConfigError("invalid pack mode " + std::to_string(packMode) + " (valid values: 0, 1, 2, 4, 8)");
}

std::size_t alignUp(std::size_t value, std::size_t alignment) { return (value + alignment - 1) / alignment * alignment; }

/// Recursive descent parser for type specifications (see TypeRegistry::parse()).
class SpecParser {
 public:
  using Lookup = std::function<TypePtr(std::string_view)>;

  SpecParser(std::string_view text, const Lookup& lookup) : text_(text), lookup_(lookup) {}

  TypePtr parse() {
    TypePtr type = parseType();
    skipSpace();
    if (pos_ != text_.size()) fail("unexpected '" + std::string(text_.substr(pos_)) + "'");
    return type;
  }

 private:
  TypePtr parseType() {
    if (keyword("ARRAY")) return parseArray();
    if (keyword("STRING")) return Type::makeString(stringLength());
    if (keyword("WSTRING")) return Type::makeWString(stringLength());
    const std::string name = identifier();
    if (name.empty()) fail(pos_ < text_.size() ? "expected a type name" : "empty type");
    TypePtr type = lookup_(name);
    if (!type) throw KeyError("unknown type '" + name + "'");
    return type;
  }

  TypePtr parseArray() {
    expect('[');
    std::vector<std::pair<std::int64_t, std::int64_t>> ranges;
    do {
      const std::int64_t lower = integer();
      expect('.');
      expect('.');
      const std::int64_t upper = integer();
      if (upper < lower) {
        fail("invalid array range " + std::to_string(lower) + ".." + std::to_string(upper));
      }
      ranges.emplace_back(lower, upper);
    } while (consume(','));
    expect(']');
    if (!keyword("OF")) fail("expected 'OF'");
    TypePtr type = parseType();
    // ARRAY[a..b, c..d] OF T is an array of arrays: the last index varies fastest.
    for (auto range = ranges.rbegin(); range != ranges.rend(); ++range) {
      const auto length = static_cast<std::uint64_t>(range->second - range->first) + 1;
      if (length > kMaxTypeSize) fail("array too large");
      type = Type::makeArray(type, static_cast<std::size_t>(length), range->first);
    }
    return type;
  }

  std::size_t stringLength() {
    char close;
    if (consume('(')) {
      close = ')';
    } else if (consume('[')) {
      close = ']';
    } else {
      return 80;  // TwinCAT default
    }
    const std::int64_t length = integer();
    expect(close);
    if (length < 1 || static_cast<std::uint64_t>(length) > kMaxTypeSize / 2) {
      fail("invalid string length " + std::to_string(length));
    }
    return static_cast<std::size_t>(length);
  }

  void skipSpace() {
    while (pos_ < text_.size() && (text_[pos_] == ' ' || text_[pos_] == '\t' || text_[pos_] == '\n' ||
                                   text_[pos_] == '\r')) {
      ++pos_;
    }
  }

  bool consume(char c) {
    skipSpace();
    if (pos_ < text_.size() && text_[pos_] == c) {
      ++pos_;
      return true;
    }
    return false;
  }

  void expect(char c) {
    if (!consume(c)) fail(std::string("expected '") + c + "'");
  }

  /// Case-insensitive keyword that is not the prefix of a longer identifier.
  bool keyword(std::string_view word) {
    skipSpace();
    if (text_.size() - pos_ < word.size() || !util::iequals(text_.substr(pos_, word.size()), word)) return false;
    const std::size_t end = pos_ + word.size();
    if (end < text_.size() && (util::isIdentifierChar(text_[end]) || text_[end] == '.')) return false;
    pos_ = end;
    return true;
  }

  std::string identifier() {
    skipSpace();
    const std::size_t start = pos_;
    while (pos_ < text_.size() && (util::isIdentifierChar(text_[pos_]) || text_[pos_] == '.')) ++pos_;
    const std::string_view name = text_.substr(start, pos_ - start);
    if (!name.empty() && !util::isIdentifier(name)) {
      pos_ = start;
      fail("invalid type name '" + std::string(name) + "'");
    }
    return std::string(name);
  }

  std::int64_t integer() {
    skipSpace();
    bool negative = false;
    if (pos_ < text_.size() && (text_[pos_] == '-' || text_[pos_] == '+')) negative = text_[pos_++] == '-';
    if (pos_ >= text_.size() || text_[pos_] < '0' || text_[pos_] > '9') fail("expected an integer");
    std::int64_t value = 0;
    while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') {
      if (value > (std::numeric_limits<std::int64_t>::max() - 9) / 10) fail("integer too large");
      value = value * 10 + (text_[pos_++] - '0');
    }
    return negative ? -value : value;
  }

  [[noreturn]] void fail(const std::string& message) const {
    throw ConfigError("invalid type '" + std::string(text_) + "': " + message);
  }

  std::string_view text_;
  const Lookup& lookup_;
  std::size_t pos_ = 0;
};

}  // namespace

// ---------------------------------------------------------------------------
// Type

TypePtr Type::makePrimitive(Primitive primitive) {
  if (primitive == Primitive::String) return makeString();
  if (primitive == Primitive::WString) return makeWString();
  // Elementary types are singletons shared by all registries.
  static const auto cache = [] {
    std::array<TypePtr, static_cast<std::size_t>(Primitive::Real64) + 1> types;
    for (std::size_t i = 0; i < types.size(); ++i) {
      auto type = std::make_shared<Type>(Private{});
      type->primitive_ = static_cast<Primitive>(i);
      type->name_ = primitiveName(type->primitive_);
      type->size_ = type->alignment_ = primitiveSize(type->primitive_);
      types[i] = std::move(type);
    }
    return types;
  }();
  return cache[static_cast<std::size_t>(primitive)];
}

TypePtr Type::makeString(std::size_t length) {
  if (length < 1 || length > kMaxTypeSize - 1) throw ConfigError("invalid STRING length " + std::to_string(length));
  auto type = std::make_shared<Type>(Private{});
  type->primitive_ = Primitive::String;
  type->name_ = "STRING(" + std::to_string(length) + ")";
  type->length_ = length;
  type->size_ = length + 1;
  type->alignment_ = 1;
  return type;
}

TypePtr Type::makeWString(std::size_t length) {
  if (length < 1 || length > kMaxTypeSize / 2 - 1) throw ConfigError("invalid WSTRING length " + std::to_string(length));
  auto type = std::make_shared<Type>(Private{});
  type->primitive_ = Primitive::WString;
  type->name_ = "WSTRING(" + std::to_string(length) + ")";
  type->length_ = length;
  type->size_ = 2 * (length + 1);
  type->alignment_ = 2;
  return type;
}

TypePtr Type::makeArray(TypePtr element, std::size_t length, std::int64_t lowerBound) {
  if (!element) throw ConfigError("array element type is missing");
  if (length < 1) throw ConfigError("arrays need at least one element");
  if (length > kMaxTypeSize / element->size()) throw ConfigError("array of " + element->name() + " is too large");
  auto type = std::make_shared<Type>(Private{});
  type->kind_ = Kind::Array;
  type->name_ = "ARRAY[" + std::to_string(lowerBound) + ".." +
                std::to_string(lowerBound + static_cast<std::int64_t>(length) - 1) + "] OF " + element->name();
  type->length_ = length;
  type->lowerBound_ = lowerBound;
  type->size_ = length * element->size();
  type->alignment_ = element->alignment();
  type->element_ = std::move(element);
  return type;
}

TypePtr Type::makeStruct(std::string name, const std::vector<std::pair<std::string, TypePtr>>& fields,
                         int packMode) {
  if (name.empty()) throw ConfigError("structure name is empty");
  if (fields.empty()) throw ConfigError("structure '" + name + "' has no fields");
  const int pack = normalizePackMode(packMode);

  auto type = std::make_shared<Type>(Private{});
  type->kind_ = Kind::Struct;
  type->packMode_ = pack;
  std::size_t offset = 0;
  std::size_t alignment = 1;
  for (const auto& [fieldName, fieldType] : fields) {
    if (fieldName.empty()) throw ConfigError("structure '" + name + "' has a field without name");
    if (!fieldType) throw ConfigError("structure '" + name + "', field '" + fieldName + "': type is missing");
    for (const Field& previous : type->fields_) {
      if (util::iequals(previous.name, fieldName)) {
        throw ConfigError("structure '" + name + "' has more than one field named '" + fieldName + "'");
      }
    }
    // Each member is aligned to min(natural alignment, pack mode).
    const std::size_t fieldAlignment = std::min(fieldType->alignment(), static_cast<std::size_t>(pack));
    offset = alignUp(offset, fieldAlignment);
    type->fields_.push_back(Field{fieldName, fieldType, offset});
    offset += fieldType->size();
    if (offset > kMaxTypeSize) throw ConfigError("structure '" + name + "' is too large");
    alignment = std::max(alignment, fieldAlignment);
  }
  type->alignment_ = alignment;
  type->size_ = alignUp(offset, alignment);  // trailing padding
  type->name_ = std::move(name);
  return type;
}

Primitive Type::primitive() const {
  if (!isPrimitive()) throw TypeError("'" + name_ + "' is not an elementary type");
  return primitive_;
}

std::size_t Type::stringLength() const {
  if (!isString()) throw TypeError("'" + name_ + "' is not a STRING or WSTRING");
  return length_;
}

const Type& Type::elementType() const { return *elementTypePtr(); }

const TypePtr& Type::elementTypePtr() const {
  if (!isArray()) throw TypeError("'" + name_ + "' is not an array");
  return element_;
}

std::size_t Type::length() const {
  if (!isArray()) throw TypeError("'" + name_ + "' is not an array");
  return length_;
}

std::int64_t Type::lowerBound() const {
  if (!isArray()) throw TypeError("'" + name_ + "' is not an array");
  return lowerBound_;
}

const std::vector<Field>& Type::fields() const {
  if (!isStruct()) throw TypeError("'" + name_ + "' is not a structure");
  return fields_;
}

const Field* Type::findField(std::string_view name) const noexcept {
  for (const Field& f : fields_) {
    if (f.name == name) return &f;
  }
  for (const Field& f : fields_) {
    if (util::iequals(f.name, name)) return &f;
  }
  return nullptr;
}

const Field& Type::field(std::string_view name) const {
  if (!isStruct()) throw TypeError("'" + name_ + "' is not a structure (looking for field '" + std::string(name) + "')");
  if (const Field* f = findField(name)) return *f;
  throw KeyError("structure '" + name_ + "' has no field '" + std::string(name) + "'");
}

int Type::packMode() const {
  if (!isStruct()) throw TypeError("'" + name_ + "' is not a structure");
  return packMode_;
}

bool Type::sameLayout(const Type& other) const noexcept {
  if (this == &other) return true;
  if (kind_ != other.kind_ || size_ != other.size_) return false;
  switch (kind_) {
    case Kind::Primitive:
      return primitive_ == other.primitive_ && length_ == other.length_;
    case Kind::Array:
      return length_ == other.length_ && element_->sameLayout(*other.element_);
    case Kind::Struct:
      if (fields_.size() != other.fields_.size()) return false;
      for (std::size_t i = 0; i < fields_.size(); ++i) {
        const Field& a = fields_[i];
        const Field& b = other.fields_[i];
        if (a.offset != b.offset || !util::iequals(a.name, b.name) || !a.type->sameLayout(*b.type)) return false;
      }
      return true;
  }
  return false;
}

std::string Type::layout() const {
  std::ostringstream out;
  out << name_ << ": " << size_ << " bytes, alignment " << alignment_;
  if (isStruct()) {
    out << ", pack mode " << packMode_;
    for (const Field& f : fields_) {
      out << "\n  +" << f.offset << " " << f.name << " : " << f.type->name() << " (" << f.type->size() << " bytes)";
    }
  }
  return out.str();
}

// ---------------------------------------------------------------------------
// TypeRegistry

TypeRegistry::TypeRegistry(int defaultPackMode) : defaultPackMode_(normalizePackMode(defaultPackMode)) {
  struct Builtin {
    Primitive primitive;
    std::initializer_list<const char*> names;
  };
  const Builtin builtins[] = {
      {Primitive::Bool, {"BOOL"}},
      {Primitive::Int8, {"SINT", "int8", "int8_t"}},
      {Primitive::UInt8, {"USINT", "BYTE", "uint8", "uint8_t"}},
      {Primitive::Int16, {"INT", "int16", "int16_t"}},
      {Primitive::UInt16, {"UINT", "WORD", "uint16", "uint16_t"}},
      {Primitive::Int32, {"DINT", "int32", "int32_t"}},
      {Primitive::UInt32,
       {"UDINT", "DWORD", "uint32", "uint32_t", "TIME", "TOD", "TIME_OF_DAY", "DATE", "DT", "DATE_AND_TIME"}},
      {Primitive::Int64, {"LINT", "int64", "int64_t"}},
      {Primitive::UInt64,
       {"ULINT", "LWORD", "uint64", "uint64_t", "LTIME", "LDATE", "LTOD", "LTIME_OF_DAY", "LDT", "LDATE_AND_TIME"}},
      {Primitive::Real32, {"REAL", "float", "float32"}},
      {Primitive::Real64, {"LREAL", "double", "float64"}},
  };
  for (const Builtin& builtin : builtins) {
    for (const char* name : builtin.names) {
      named_[util::toLower(name)] = Entry{Type::makePrimitive(builtin.primitive), true};
    }
  }
  named_["string"] = Entry{Type::makeString(), true};
  named_["wstring"] = Entry{Type::makeWString(), true};
}

void TypeRegistry::setDefaultPackMode(int packMode) { defaultPackMode_ = normalizePackMode(packMode); }

TypePtr TypeRegistry::find(std::string_view name) const {
  const auto it = named_.find(util::toLower(name));
  return it == named_.end() ? nullptr : it->second.type;
}

TypePtr TypeRegistry::parse(std::string_view spec) const {
  const SpecParser::Lookup lookup = [this](std::string_view name) { return find(name); };
  return SpecParser(spec, lookup).parse();
}

void TypeRegistry::addNamed(const std::string& name, TypePtr type) {
  named_[util::toLower(name)] = Entry{std::move(type), false};
}

TypePtr TypeRegistry::defineStruct(const StructDefinition& definition) {
  define({}, {definition});
  return find(definition.name);
}

TypePtr TypeRegistry::defineAlias(const std::string& name, std::string_view spec) {
  define({AliasDefinition{name, std::string(spec)}}, {});
  return find(name);
}

std::vector<TypePtr> TypeRegistry::structures() const { return structures_; }

/// Resolves a set of definitions that may reference each other in any order.
struct TypeRegistry::Resolver {
  enum class State { Pending, Active, Done };
  struct Definition {
    const AliasDefinition* alias = nullptr;
    const StructDefinition* structure = nullptr;
    State state = State::Pending;
    const std::string& name() const { return alias ? alias->name : structure->name; }
  };

  TypeRegistry& registry;
  std::unordered_map<std::string, Definition> definitions;  // key: lower-case name
  std::vector<std::string> chain;                            // definitions being resolved

  TypePtr lookup(std::string_view name) {
    if (TypePtr type = registry.find(name)) return type;
    const auto it = definitions.find(util::toLower(name));
    return it == definitions.end() ? nullptr : resolve(it->second);
  }

  TypePtr parse(const std::string& spec, const std::string& context) {
    const SpecParser::Lookup lookupFn = [this](std::string_view name) { return lookup(name); };
    try {
      return SpecParser(spec, lookupFn).parse();
    } catch (const Error& e) {
      throw ConfigError(context + ": " + e.what());
    }
  }

  TypePtr resolve(Definition& definition) {
    if (definition.state == State::Done) return registry.find(definition.name());
    if (definition.state == State::Active) {
      std::string cycle;
      for (const std::string& name : chain) cycle += name + " -> ";
      throw ConfigError("recursive type definition: " + cycle + definition.name());
    }
    definition.state = State::Active;
    chain.push_back(definition.name());

    TypePtr type;
    if (definition.alias) {
      type = parse(definition.alias->type, "alias '" + definition.alias->name + "'");
    } else {
      const StructDefinition& s = *definition.structure;
      std::vector<std::pair<std::string, TypePtr>> fields;
      fields.reserve(s.fields.size());
      for (const FieldDefinition& f : s.fields) {
        fields.emplace_back(f.name, parse(f.type, "structure '" + s.name + "', field '" + f.name + "'"));
      }
      type = Type::makeStruct(s.name, fields, s.packMode.value_or(registry.defaultPackMode()));
      registry.structures_.push_back(type);
    }
    registry.addNamed(definition.name(), type);

    chain.pop_back();
    definition.state = State::Done;
    return type;
  }
};

void TypeRegistry::define(const std::vector<AliasDefinition>& aliases,
                          const std::vector<StructDefinition>& structures) {
  TypeRegistry work = *this;  // all or nothing
  Resolver resolver{work, {}, {}};

  const auto declare = [&](const std::string& name, Resolver::Definition definition, const char* what) {
    if (!util::isIdentifier(name)) throw ConfigError("invalid " + std::string(what) + " name '" + name + "'");
    for (const char* keyword : {"ARRAY", "OF", "STRING", "WSTRING"}) {
      if (util::iequals(name, keyword)) throw ConfigError("'" + name + "' is a reserved word");
    }
    if (const auto it = work.named_.find(util::toLower(name)); it != work.named_.end()) {
      throw ConfigError(std::string(what) + " '" + name + "' conflicts with " +
                        (it->second.builtin ? "a built-in type" : "an existing type"));
    }
    if (!resolver.definitions.emplace(util::toLower(name), definition).second) {
      throw ConfigError("type '" + name + "' is defined more than once");
    }
  };
  for (const AliasDefinition& alias : aliases) declare(alias.name, {&alias, nullptr}, "alias");
  for (const StructDefinition& structure : structures) {
    if (structure.fields.empty()) throw ConfigError("structure '" + structure.name + "' has no fields");
    if (structure.packMode) {
      try {
        normalizePackMode(*structure.packMode);
      } catch (const ConfigError& e) {
        throw ConfigError("structure '" + structure.name + "': " + e.what());
      }
    }
    declare(structure.name, {nullptr, &structure}, "structure");
  }

  // Resolve in declaration order so that structures() keeps it when possible.
  for (const AliasDefinition& alias : aliases) resolver.resolve(resolver.definitions.at(util::toLower(alias.name)));
  for (const StructDefinition& structure : structures) {
    resolver.resolve(resolver.definitions.at(util::toLower(structure.name)));
  }
  *this = std::move(work);
}

}  // namespace beckhoff
