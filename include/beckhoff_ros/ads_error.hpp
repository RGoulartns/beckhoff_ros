#ifndef BECKHOFF_ROS__ADS_ERROR_HPP_
#define BECKHOFF_ROS__ADS_ERROR_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>

namespace beckhoff_ros
{

/// ADS return codes (see Beckhoff's "ADS Return Codes"), plus the client side errors.
namespace ads_error
{
constexpr std::uint32_t kNone = 0x0;
constexpr std::uint32_t kInternal = 0x1;
constexpr std::uint32_t kTargetPortNotFound = 0x6;     // e.g. the PLC runtime is not running
constexpr std::uint32_t kTargetMachineNotFound = 0x7;  // e.g. missing route
constexpr std::uint32_t kDeviceError = 0x700;
constexpr std::uint32_t kServiceNotSupported = 0x701;
constexpr std::uint32_t kInvalidIndexGroup = 0x702;
constexpr std::uint32_t kInvalidIndexOffset = 0x703;
constexpr std::uint32_t kInvalidAccess = 0x704;
constexpr std::uint32_t kInvalidSize = 0x705;
constexpr std::uint32_t kInvalidData = 0x706;
constexpr std::uint32_t kNotReady = 0x707;
constexpr std::uint32_t kBusy = 0x708;
constexpr std::uint32_t kNoMemory = 0x70A;
constexpr std::uint32_t kInvalidParameter = 0x70B;
constexpr std::uint32_t kNotFound = 0x70C;
constexpr std::uint32_t kSymbolNotFound = 0x710;
constexpr std::uint32_t kSymbolVersionInvalid = 0x711;
constexpr std::uint32_t kInvalidState = 0x712;
constexpr std::uint32_t kTransModeNotSupported = 0x713;
constexpr std::uint32_t kNotificationHandleInvalid = 0x714;
constexpr std::uint32_t kNoMoreHandles = 0x716;
constexpr std::uint32_t kInvalidWatchSize = 0x717;
constexpr std::uint32_t kDeviceTimeout = 0x719;
constexpr std::uint32_t kSymbolNotActive = 0x722;
constexpr std::uint32_t kAccessDenied = 0x723;
constexpr std::uint32_t kClientError = 0x740;
constexpr std::uint32_t kClientInvalidParameter = 0x741;
/// No response before the timeout.
constexpr std::uint32_t kTimeout = 0x745;
/// The connection to the PLC is not established (lost, or being re-established).
constexpr std::uint32_t kNotConnected = 0x748;
/// The response is not valid.
constexpr std::uint32_t kInvalidResponse = 0x754;
}  // namespace ads_error

/// Raised when an ADS request fails (pyads.ADSError in the Python version).
class AdsError : public std::runtime_error
{
public:
  /// `what` describes the failed operation; the ADS error code and its meaning are appended.
  AdsError(const std::string & what, long code);
  long code() const {return code_;}

private:
  long code_;
};

/// Short description of an ADS return code, e.g. "symbol not found" for 1808.
std::string adsErrorText(long code);

}  // namespace beckhoff_ros

#endif  // BECKHOFF_ROS__ADS_ERROR_HPP_
