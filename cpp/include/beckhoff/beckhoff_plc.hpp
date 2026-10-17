#pragma once

#include "beckhoff/transport.hpp"
#include "beckhoff/types.hpp"
#include <functional>
#include <memory>
#include <string>

namespace beckhoff {

// ROS-independent, synchronous ADS client. Construct/register outside the cyclic
// loop. Serialize shutdown with application operations; callbacks must not call
// synchronous ADS methods or destroy this object (queue work to another thread).
class BeckhoffPLC {
    struct Impl;
public:
    class Variable {
    public:
        const std::string& name() const noexcept { return name_; }
        const std::shared_ptr<const Type>& type() const noexcept { return type_; }
        std::size_t size() const noexcept { return type_->size; }
        bool writable() const noexcept { return writable_; }
    private:
        friend struct Impl;
        friend class BeckhoffPLC;
        Variable() = default;
        const Impl* owner_ = nullptr;
        std::string name_;
        std::shared_ptr<const Type> type_;
        std::uint32_t handle_ = 0;
        bool writable_ = false;
    };
    using Observer = std::function<void(const std::string&, const Json&)>;
    using RawObserver = std::function<void(const std::string&, const std::uint8_t*, std::size_t)>;
    using ObserverId = std::uint64_t;

    explicit BeckhoffPLC(const Json& parameters, std::unique_ptr<Transport> transport = {});
    static std::unique_ptr<BeckhoffPLC> fromFile(const std::string& path);
    ~BeckhoffPLC();
    BeckhoffPLC(const BeckhoffPLC&) = delete;
    BeckhoffPLC& operator=(const BeckhoffPLC&) = delete;
    BeckhoffPLC(BeckhoffPLC&&) = delete;
    BeckhoffPLC& operator=(BeckhoffPLC&&) = delete;

    const Variable& variable(const std::string& name) const;
    Value makeValue(const Json& type_spec) const;
    Value makeVariable(const std::string& name) const;
    Json getVariable(const std::string& name);
    void setVariable(const std::string& name, const Json& value);
    void setVariable(const std::string& name, const Value& value);

    // No JSON conversion, name lookup, or wrapper allocation on these paths.
    // Supply exactly size() bytes. Wire layout remains the caller's responsibility.
    void read(const Variable& variable, void* buffer, std::size_t size);
    void write(const Variable& variable, const void* buffer, std::size_t size);

    // Registration allocates. Raw notification buffers are borrowed only for the
    // callback duration. Changes arrive on the ADS dispatcher thread; initial
    // values run on the calling thread and may overlap change notifications.
    // Registry changes and synchronous I/O are forbidden inside observers.
    ObserverId addObserver(Observer observer, bool read_initial = true);
    ObserverId addRawObserver(RawObserver observer, bool read_initial = true);
    // Waits for active invocations, so captured state can be destroyed afterward.
    // Call outside callbacks; registration/removal are serialized.
    void removeObserver(ObserverId id);
    void readInitialValues();
    std::uint64_t callbackErrors() const noexcept;

private:
    std::unique_ptr<Impl> impl_;
};
} // namespace beckhoff
