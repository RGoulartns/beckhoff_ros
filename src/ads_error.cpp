#include "beckhoff_ros/ads_error.hpp"

#include <string>

namespace beckhoff_ros
{

namespace
{

std::string describe(const std::string & what, long code)
{
  const std::string text = adsErrorText(code);
  return what + " (ADS error " + std::to_string(code) + (text.empty() ? "" : ": " + text) + ")";
}

}  // namespace

AdsError::AdsError(const std::string & what, long code)
: std::runtime_error(describe(what, code)), code_(code)
{
}

std::string adsErrorText(long code)
{
  using namespace ads_error;  // NOLINT(build/namespaces)
  switch (code) {
    case kInternal: return "internal error";
    case 0x4: return "mailbox full, too many ADS requests at once";
    case kTargetPortNotFound: return "target port not found, the PLC runtime may not be running";
    case kTargetMachineNotFound: return "target machine not found, possibly missing ADS routes";
    case 0x8: return "unknown command ID";
    case 0xD: return "port not connected";
    case 0xE: return "invalid AMS length";
    case 0xF: return "invalid AMS Net ID";
    case 0x12: return "port disabled";
    case 0x13: return "port already connected";
    case 0x19: return "no memory";
    case 0x1A: return "TCP send error";
    case 0x1B: return "host unreachable";
    case 0x502: return "router mailbox full, too many ADS requests at once";
    case 0x506: return "port already in use";
    case 0x508: return "maximum number of ports reached";
    case 0x50A: return "router not active";
    case kDeviceError: return "device error";
    case kServiceNotSupported: return "service is not supported by the server";
    case kInvalidIndexGroup: return "invalid index group";
    case kInvalidIndexOffset: return "invalid index offset";
    case kInvalidAccess: return "reading/writing not permitted";
    case kInvalidSize:
      return "parameter size not correct, check the variable type and structure definitions";
    case kInvalidData: return "invalid parameter value(s)";
    case kNotReady: return "device is not in a ready state";
    case kBusy: return "device is busy";
    case 0x709: return "invalid operating system context";
    case kNoMemory: return "out of memory";
    case kInvalidParameter: return "invalid parameter value(s)";
    case kNotFound: return "not found";
    case 0x70D: return "syntax error";
    case 0x70E: return "objects do not match";
    case 0x70F: return "object already exists";
    case kSymbolNotFound: return "symbol not found";
    case kSymbolVersionInvalid: return "symbol version invalid (online change?)";
    case kInvalidState: return "server is in invalid state";
    case kTransModeNotSupported: return "transmission mode not supported";
    case kNotificationHandleInvalid: return "notification handle is invalid";
    case 0x715: return "notification client not registered";
    case kNoMoreHandles: return "no more notification handles";
    case kInvalidWatchSize: return "notification size too large";
    case 0x718: return "device not initialized";
    case kDeviceTimeout: return "device timeout";
    case 0x71E: return "request pending";
    case 0x71F: return "request aborted";
    case 0x721: return "invalid array index";
    case kSymbolNotActive: return "symbol not active (online change?)";
    case kAccessDenied: return "access denied";
    case 0x724: return "missing license";
    case 0x72C: return "exception";
    case kClientError: return "client error";
    case kClientInvalidParameter: return "invalid parameter at service call";
    case kTimeout: return "timeout elapsed, check the ADS routes of the PLC and this computer";
    case kNotConnected: return "not connected to the PLC";
    case kInvalidResponse: return "invalid response received";
    default: return "";
  }
}

}  // namespace beckhoff_ros
