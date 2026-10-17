#pragma once

#include <nlohmann/json.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace beckhoff {

// JSON object insertion order is the PLC structure's field order.
using Json = nlohmann::ordered_json;

struct Type {
    enum class Kind { Boolean, SignedInteger, UnsignedInteger, Floating, Structure, Array };
    struct Field {
        std::string name;
        std::shared_ptr<const Type> type;
        std::size_t offset;
    };

    Kind kind = Kind::Boolean;
    std::string name;
    std::size_t size = 0;
    std::size_t alignment = 1;
    std::vector<Field> fields;
    std::shared_ptr<const Type> element;
    std::size_t count = 0;
};

// Default packing matches Python ctypes.Structure(_pack_=0). A nonzero pack
// caps member alignment; set the same TwinCAT pack_mode on the PLC DUTs.
class TypeRegistry {
public:
    explicit TypeRegistry(const Json& structures = Json::object(), std::size_t pack = 0);
    std::shared_ptr<const Type> resolve(const Json& spec) const;
    static constexpr std::size_t maxBytes = 16U * 1024U * 1024U;

private:
    std::shared_ptr<const Type> resolveImpl(const Json& spec, std::size_t depth) const;
    std::shared_ptr<const Type> build(const std::string& name, std::size_t depth) const;
    Json structures_;
    std::size_t pack_;
    mutable std::unordered_map<std::string, std::shared_ptr<const Type>> types_;
    mutable std::vector<std::string> building_;
};

// A zero-initialized, owning PLC byte buffer. Integers and IEEE floats are
// explicitly serialized little endian; padding remains zero after assign().
class Value {
public:
    explicit Value(std::shared_ptr<const Type> type);
    Value(std::shared_ptr<const Type> type, const Json& value);
    const std::shared_ptr<const Type>& type() const noexcept { return type_; }
    std::uint8_t* data() noexcept { return bytes_.data(); }
    const std::uint8_t* data() const noexcept { return bytes_.data(); }
    std::size_t size() const noexcept { return bytes_.size(); }
    Json toJson() const;
    void assign(const Json& value);
    // RFC 6901 JSON pointer: "/field/0", with ~0 and ~1 escaping. Empty path
    // selects the entire value. set() updates only the selected subtree.
    void set(const std::string& path, const Json& value);
    Json get(const std::string& path) const;

private:
    std::shared_ptr<const Type> type_;
    std::vector<std::uint8_t> bytes_;
};

} // namespace beckhoff
