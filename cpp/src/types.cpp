#include "beckhoff/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace beckhoff {
namespace {
constexpr std::size_t maxDepth = 128;

[[noreturn]] void invalid(const std::string& message) {
    throw std::invalid_argument(message);
}

std::size_t checkedSize(std::size_t size) {
    if (size == 0 || size > TypeRegistry::maxBytes) {
        invalid("PLC type size must be between 1 byte and 16 MiB");
    }
    return size;
}

std::size_t aligned(std::size_t value, std::size_t alignment) {
    const auto extra = (alignment - value % alignment) % alignment;
    if (value > TypeRegistry::maxBytes - extra) invalid("PLC structure size exceeds 16 MiB");
    return value + extra;
}

std::uint64_t readUnsigned(const std::uint8_t* bytes, std::size_t size) {
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < size; ++i) value |= std::uint64_t(bytes[i]) << (8U * i);
    return value;
}

void writeUnsigned(std::uint8_t* bytes, std::size_t size, std::uint64_t value) {
    for (std::size_t i = 0; i < size; ++i) bytes[i] = static_cast<std::uint8_t>(value >> (8U * i));
}

std::int64_t signedNumber(const Json& value, std::size_t size) {
    if (!value.is_number_integer()) invalid("Expected a signed integer");
    std::int64_t number;
    if (value.is_number_unsigned()) {
        const auto unsignedValue = value.get<std::uint64_t>();
        if (unsignedValue > std::uint64_t(std::numeric_limits<std::int64_t>::max())) invalid("Signed integer out of range");
        number = static_cast<std::int64_t>(unsignedValue);
    } else {
        number = value.get<std::int64_t>();
    }
    if (size < 8) {
        const auto limit = std::int64_t(1) << (size * 8U - 1U);
        if (number < -limit || number >= limit) invalid("Signed integer out of range");
    }
    return number;
}

std::uint64_t unsignedNumber(const Json& value, std::size_t size) {
    if (!value.is_number_integer()) invalid("Expected an unsigned integer");
    if (!value.is_number_unsigned() && value.get<std::int64_t>() < 0) invalid("Unsigned integer out of range");
    const auto number = value.get<std::uint64_t>();
    if (size < 8 && number >= (std::uint64_t(1) << (size * 8U))) invalid("Unsigned integer out of range");
    return number;
}

void encode(const Type& type, const Json& value, std::uint8_t* bytes) {
    switch (type.kind) {
    case Type::Kind::Boolean:
        if (!value.is_boolean()) invalid("Expected a boolean");
        bytes[0] = value.get<bool>() ? 1 : 0;
        return;
    case Type::Kind::SignedInteger:
        writeUnsigned(bytes, type.size, static_cast<std::uint64_t>(signedNumber(value, type.size)));
        return;
    case Type::Kind::UnsignedInteger:
        writeUnsigned(bytes, type.size, unsignedNumber(value, type.size));
        return;
    case Type::Kind::Floating: {
        if (!value.is_number()) invalid("Expected a floating point number");
        const double number = value.get<double>();
        if (!std::isfinite(number)) invalid("Floating point number must be finite");
        if (type.size == 4) {
            if (std::abs(number) > std::numeric_limits<float>::max()) invalid("Float32 out of range");
            const auto narrowed = static_cast<float>(number);
            std::uint32_t bits;
            std::memcpy(&bits, &narrowed, sizeof(bits));
            writeUnsigned(bytes, 4, bits);
        } else {
            std::uint64_t bits;
            std::memcpy(&bits, &number, sizeof(bits));
            writeUnsigned(bytes, 8, bits);
        }
        return;
    }
    case Type::Kind::Structure:
        if (!value.is_object() || value.size() != type.fields.size()) invalid("Structure requires exactly its declared fields");
        for (const auto& field : type.fields) {
            const auto it = value.find(field.name);
            if (it == value.end()) invalid("Missing structure field: " + field.name);
            encode(*field.type, *it, bytes + field.offset);
        }
        return;
    case Type::Kind::Array:
        if (!value.is_array() || value.size() != type.count) invalid("Array length does not match its type");
        for (std::size_t i = 0; i < type.count; ++i) encode(*type.element, value[i], bytes + i * type.element->size);
        return;
    }
    invalid("Unsupported PLC type");
}

Json decode(const Type& type, const std::uint8_t* bytes) {
    switch (type.kind) {
    case Type::Kind::Boolean:
        return bytes[0] != 0;
    case Type::Kind::SignedInteger: {
        const auto bits = readUnsigned(bytes, type.size);
        // Avoid implementation-defined conversion of out-of-range unsigned values.
        const auto sign = std::uint64_t(1) << (type.size * 8U - 1U);
        if ((bits & sign) == 0) return static_cast<std::int64_t>(bits);
        const auto mask = type.size == 8 ? ~std::uint64_t(0) : (std::uint64_t(1) << (type.size * 8U)) - 1U;
        return -1 - static_cast<std::int64_t>((~bits) & mask);
    }
    case Type::Kind::UnsignedInteger:
        return readUnsigned(bytes, type.size);
    case Type::Kind::Floating: {
        if (type.size == 4) {
            const auto bits = static_cast<std::uint32_t>(readUnsigned(bytes, 4));
            float number;
            std::memcpy(&number, &bits, sizeof(number));
            return number;
        }
        const auto bits = readUnsigned(bytes, 8);
        double number;
        std::memcpy(&number, &bits, sizeof(number));
        return number;
    }
    case Type::Kind::Structure: {
        auto result = Json::object();
        for (const auto& field : type.fields) result[field.name] = decode(*field.type, bytes + field.offset);
        return result;
    }
    case Type::Kind::Array: {
        auto result = Json::array();
        for (std::size_t i = 0; i < type.count; ++i) result.push_back(decode(*type.element, bytes + i * type.element->size));
        return result;
    }
    }
    invalid("Unsupported PLC type");
}

struct Selection { const Type* type; std::size_t offset; };

Selection select(const Type& root, const std::string& path) {
    Selection current{&root, 0};
    if (path.empty()) return current;
    if (path[0] != '/') invalid("Value path must be an RFC 6901 JSON pointer");
    std::size_t begin = 1;
    while (true) {
        const auto end = path.find('/', begin);
        const auto raw = path.substr(begin, end == std::string::npos ? end : end - begin);
        std::string token;
        for (std::size_t i = 0; i < raw.size(); ++i) {
            if (raw[i] != '~') token.push_back(raw[i]);
            else {
                if (++i == raw.size() || (raw[i] != '0' && raw[i] != '1')) invalid("Invalid JSON pointer escape");
                token.push_back(raw[i] == '0' ? '~' : '/');
            }
        }
        if (current.type->kind == Type::Kind::Structure) {
            const auto& fields = current.type->fields;
            const auto found = std::find_if(fields.begin(), fields.end(), [&](const Type::Field& field) { return field.name == token; });
            if (found == fields.end()) invalid("Unknown structure field: " + token);
            current.offset += found->offset;
            current.type = found->type.get();
        } else if (current.type->kind == Type::Kind::Array) {
            if (token.empty() || (token.size() > 1 && token.front() == '0')) invalid("Invalid array index");
            std::size_t index = 0;
            for (const char digit : token) {
                if (digit < '0' || digit > '9' || index > (std::numeric_limits<std::size_t>::max() - std::size_t(digit - '0')) / 10) invalid("Invalid array index");
                index = index * 10 + std::size_t(digit - '0');
            }
            if (index >= current.type->count) invalid("Array index out of range");
            current.offset += index * current.type->element->size;
            current.type = current.type->element.get();
        } else {
            invalid("Cannot traverse a scalar value");
        }
        if (end == std::string::npos) return current;
        begin = end + 1;
    }
}
} // namespace

TypeRegistry::TypeRegistry(const Json& structures, std::size_t pack) : structures_(structures), pack_(pack) {
    static_assert(sizeof(float) == 4 && sizeof(double) == 8 && std::numeric_limits<float>::is_iec559 && std::numeric_limits<double>::is_iec559, "IEEE-754 floats are required");
    if (!structures_.is_object()) invalid("structures must be a JSON object");
    if (pack != 0 && pack != 1 && pack != 2 && pack != 4 && pack != 8 && pack != 16) invalid("pack must be 0, 1, 2, 4, 8 or 16");
    const auto add = [&](const std::string& name, Type::Kind kind, std::size_t size, std::size_t alignment) {
        auto type = std::make_shared<Type>();
        type->name = name;
        type->kind = kind;
        type->size = size;
        type->alignment = alignment;
        types_.emplace(name, std::move(type));
    };
    add("bool", Type::Kind::Boolean, 1, 1);
    add("int8", Type::Kind::SignedInteger, 1, alignof(std::int8_t));
    add("uint8", Type::Kind::UnsignedInteger, 1, alignof(std::uint8_t));
    add("int16", Type::Kind::SignedInteger, 2, alignof(std::int16_t));
    add("uint16", Type::Kind::UnsignedInteger, 2, alignof(std::uint16_t));
    add("int32", Type::Kind::SignedInteger, 4, alignof(std::int32_t));
    add("uint32", Type::Kind::UnsignedInteger, 4, alignof(std::uint32_t));
    add("int64", Type::Kind::SignedInteger, 8, alignof(std::int64_t));
    add("uint64", Type::Kind::UnsignedInteger, 8, alignof(std::uint64_t));
    add("real", Type::Kind::Floating, 4, alignof(float));
    add("double", Type::Kind::Floating, 8, alignof(double));
    types_.emplace("int", types_.at("int16"));
    types_.emplace("float", types_.at("real"));
    types_.emplace("float32", types_.at("real"));
    types_.emplace("float64", types_.at("double"));
    for (const auto& item : structures_.items()) {
        if (types_.count(item.key())) invalid("Custom structure shadows a built-in type: " + item.key());
        if (!item.value().is_object() || item.value().empty()) invalid("Each structure must be a nonempty object: " + item.key());
    }
    for (const auto& item : structures_.items()) build(item.key(), 0);
}

std::shared_ptr<const Type> TypeRegistry::resolve(const Json& spec) const {
    return resolveImpl(spec, 0);
}

std::shared_ptr<const Type> TypeRegistry::resolveImpl(const Json& spec, std::size_t depth) const {
    if (depth > maxDepth) invalid("PLC type nesting exceeds 128 levels");
    if (spec.is_string()) return build(spec.get<std::string>(), depth + 1);
    if (!spec.is_array() || spec.size() != 2 || !spec[1].is_number_integer()) invalid("Type must be a name or [type, count]");
    const auto countValue = unsignedNumber(spec[1], 8);
    if (countValue == 0 || countValue > maxBytes) invalid("Array count must be between 1 and 16 MiB");
    auto element = resolveImpl(spec[0], depth + 1);
    const auto count = static_cast<std::size_t>(countValue);
    if (element->size > maxBytes / count) invalid("Array size exceeds 16 MiB");
    auto type = std::make_shared<Type>();
    type->kind = Type::Kind::Array;
    type->name = element->name + "[" + std::to_string(count) + "]";
    type->size = checkedSize(element->size * count);
    type->alignment = element->alignment;
    type->count = count;
    type->element = std::move(element);
    return type;
}

std::shared_ptr<const Type> TypeRegistry::build(const std::string& name, std::size_t depth) const {
    if (depth > maxDepth) invalid("PLC type nesting exceeds 128 levels");
    const auto cached = types_.find(name);
    if (cached != types_.end()) return cached->second;
    const auto definition = structures_.find(name);
    if (definition == structures_.end()) invalid("Unknown PLC type: " + name);
    if (std::find(building_.begin(), building_.end(), name) != building_.end()) invalid("Recursive PLC structure: " + name);
    building_.push_back(name);
    try {
        auto type = std::make_shared<Type>();
        type->kind = Type::Kind::Structure;
        type->name = name;
        std::size_t offset = 0;
        for (const auto& item : definition->items()) {
            auto fieldType = resolveImpl(item.value(), depth + 1);
            const auto alignment = pack_ == 0 ? fieldType->alignment : std::min(pack_, fieldType->alignment);
            offset = aligned(offset, alignment);
            type->alignment = std::max(type->alignment, alignment);
            type->fields.push_back({item.key(), fieldType, offset});
            if (fieldType->size > maxBytes - offset) invalid("PLC structure size exceeds 16 MiB");
            offset += fieldType->size;
        }
        type->size = checkedSize(aligned(offset, type->alignment));
        types_.emplace(name, type);
        building_.pop_back();
        return type;
    } catch (...) {
        building_.pop_back();
        throw;
    }
}

Value::Value(std::shared_ptr<const Type> type) : type_(std::move(type)) {
    if (!type_) invalid("Value requires a type");
    bytes_.resize(checkedSize(type_->size), 0);
}

Value::Value(std::shared_ptr<const Type> type, const Json& value) : Value(std::move(type)) {
    assign(value);
}

Json Value::toJson() const { return decode(*type_, bytes_.data()); }

void Value::assign(const Json& value) {
    std::vector<std::uint8_t> next(bytes_.size(), 0);
    encode(*type_, value, next.data());
    bytes_.swap(next);
}

void Value::set(const std::string& path, const Json& value) {
    const auto selected = select(*type_, path);
    std::vector<std::uint8_t> next(selected.type->size, 0);
    encode(*selected.type, value, next.data());
    std::copy(next.begin(), next.end(), bytes_.begin() + static_cast<std::ptrdiff_t>(selected.offset));
}

Json Value::get(const std::string& path) const {
    const auto selected = select(*type_, path);
    return decode(*selected.type, bytes_.data() + selected.offset);
}

} // namespace beckhoff
