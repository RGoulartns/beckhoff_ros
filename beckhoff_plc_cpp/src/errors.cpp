#include "beckhoff_plc/errors.hpp"

#include <cstdio>

namespace beckhoff {

namespace {

// Source: Beckhoff Information System, "ADS Return Codes".
const char* describe(long code) {
  switch (code) {
    case -1: return "request could not be sent (connection lost or AMS port busy)";
    case 0x0: return "no error";
    case 0x1: return "internal error";
    case 0x2: return "no real time";
    case 0x3: return "allocation locked, memory error";
    case 0x4: return "mailbox full, the ADS message could not be sent";
    case 0x5: return "wrong HMSG";
    case 0x6: return "target port not found, the ADS server is not started, not reachable or not installed";
    case 0x7: return "target computer not found, AMS route was not found";
    case 0x8: return "unknown command ID";
    case 0x9: return "invalid task ID";
    case 0xA: return "no IO";
    case 0xB: return "unknown AMS command";
    case 0xC: return "Win32 error";
    case 0xD: return "port not connected";
    case 0xE: return "invalid AMS length";
    case 0xF: return "invalid AMS Net ID";
    case 0x10: return "installation level is too low (TwinCAT 2 license error)";
    case 0x11: return "no debugging available";
    case 0x12: return "port disabled, TwinCAT system service not started";
    case 0x13: return "port already connected";
    case 0x14: return "AMS sync Win32 error";
    case 0x15: return "AMS sync timeout";
    case 0x16: return "AMS sync error";
    case 0x17: return "no index map for AMS sync available";
    case 0x18: return "invalid AMS port";
    case 0x19: return "no memory";
    case 0x1A: return "TCP send error";
    case 0x1B: return "host unreachable";
    case 0x1C: return "invalid AMS fragment";
    case 0x1D: return "TLS send error, secure ADS connection failed";
    case 0x1E: return "access denied, secure ADS access denied";
    case 0x500: return "router: locked memory cannot be allocated";
    case 0x501: return "router: the router memory size could not be changed";
    case 0x502: return "router: the mailbox has reached the maximum number of possible messages";
    case 0x503: return "router: the debug mailbox has reached the maximum number of possible messages";
    case 0x504: return "router: the port type is unknown";
    case 0x505: return "router: the router is not initialized";
    case 0x506: return "router: the port number is already assigned (a route to this AMS Net ID with another IP address exists)";
    case 0x507: return "router: the port is not registered";
    case 0x508: return "router: the maximum number of ports has been reached";
    case 0x509: return "router: the port is invalid";
    case 0x50A: return "router: the router is not active";
    case 0x50B: return "router: the mailbox has reached the maximum number for fragmented messages";
    case 0x50C: return "router: a fragment timeout has occurred";
    case 0x50D: return "router: the port is removed";
    case 0x700: return "general device error";
    case 0x701: return "service is not supported by the server";
    case 0x702: return "invalid index group";
    case 0x703: return "invalid index offset";
    case 0x704: return "reading or writing not permitted";
    case 0x705: return "parameter size not correct";
    case 0x706: return "invalid data values";
    case 0x707: return "device is not ready to operate";
    case 0x708: return "device is busy";
    case 0x709: return "invalid operating system context";
    case 0x70A: return "insufficient memory";
    case 0x70B: return "invalid parameter values";
    case 0x70C: return "not found (files, ...)";
    case 0x70D: return "syntax error in file or command";
    case 0x70E: return "objects do not match";
    case 0x70F: return "object already exists";
    case 0x710: return "symbol not found";
    case 0x711: return "invalid symbol version (e.g. after an online change)";
    case 0x712: return "device (server) is in an invalid state";
    case 0x713: return "AdsTransMode not supported";
    case 0x714: return "notification handle is invalid";
    case 0x715: return "notification client not registered";
    case 0x716: return "no further handle available";
    case 0x717: return "notification size too large";
    case 0x718: return "device not initialized";
    case 0x719: return "device has a timeout";
    case 0x71A: return "interface query failed";
    case 0x71B: return "wrong interface requested";
    case 0x71C: return "class ID is invalid";
    case 0x71D: return "object ID is invalid";
    case 0x71E: return "request pending";
    case 0x71F: return "request is aborted";
    case 0x720: return "signal warning";
    case 0x721: return "invalid array index";
    case 0x722: return "symbol not active";
    case 0x723: return "access denied";
    case 0x724: return "missing license";
    case 0x725: return "license expired";
    case 0x726: return "license exceeded";
    case 0x727: return "invalid license";
    case 0x728: return "license problem: system ID is invalid";
    case 0x729: return "license not limited in time";
    case 0x72A: return "licensing problem: time in the future";
    case 0x72B: return "license period too long";
    case 0x72C: return "exception at system startup";
    case 0x72D: return "license file read twice";
    case 0x72E: return "invalid signature";
    case 0x72F: return "invalid certificate";
    case 0x730: return "public key not known from OEM";
    case 0x731: return "license not valid for this system ID";
    case 0x732: return "demo license prohibited";
    case 0x733: return "invalid function ID";
    case 0x734: return "outside the valid range";
    case 0x735: return "invalid alignment";
    case 0x736: return "invalid platform level";
    case 0x737: return "context: forward to passive level";
    case 0x738: return "context: forward to dispatch level";
    case 0x739: return "context: forward to real time";
    case 0x740: return "general client error";
    case 0x741: return "service contains an invalid parameter";
    case 0x742: return "polling list is empty";
    case 0x743: return "var connection already in use";
    case 0x744: return "the called ID is already in use";
    case 0x745: return "timeout, the remote terminal is not responding (check the AMS routes and the firewall)";
    case 0x746: return "error in Win32 subsystem";
    case 0x747: return "invalid client timeout value";
    case 0x748: return "port not open";
    case 0x749: return "no AMS address";
    case 0x750: return "internal error in ADS sync";
    case 0x751: return "hash table overflow";
    case 0x752: return "key not found in the table";
    case 0x753: return "no symbols in the cache";
    case 0x754: return "invalid response received";
    case 0x755: return "sync port is locked";
    case 0x1000: return "internal error in the real-time system";
    case 0x1001: return "timer value is not valid";
    case 0x1002: return "task pointer has the invalid value 0";
    case 0x1003: return "stack pointer has the invalid value 0";
    case 0x1004: return "the requested task priority is already assigned";
    case 0x1005: return "no free TCB (task control block) available";
    case 0x1006: return "no free semaphores available";
    case 0x1007: return "no free space available in the queue";
    case 0x100D: return "an external synchronization interrupt is already applied";
    case 0x100E: return "no external sync interrupt applied";
    case 0x100F: return "application of the external synchronization interrupt has failed";
    case 0x1010: return "call of a service function in the wrong context";
    case 0x1017: return "Intel VT-x extension is not supported";
    case 0x1018: return "Intel VT-x extension is not enabled in the BIOS";
    case 0x1019: return "missing function in Intel VT-x extension";
    case 0x101A: return "activation of Intel VT-x fails";
    case 0x274C: return "connection timeout";
    case 0x274D: return "connection refused";
    case 0x2751: return "no route to host";
    default: return nullptr;
  }
}

std::string hex(long code) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), code < 0 ? "%ld" : "0x%lX", code);
  return buffer;
}

}  // namespace

std::string adsErrorString(long code) {
  const char* text = describe(code);
  return "ADS error " + hex(code) + ": " + (text ? text : "unknown error");
}

AdsError::AdsError(long code, const std::string& context)
    : Error(context + " (" + adsErrorString(code) + ")"), code_(code) {}

}  // namespace beckhoff
