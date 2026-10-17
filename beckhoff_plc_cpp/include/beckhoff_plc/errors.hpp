#pragma once

#include <stdexcept>
#include <string>

namespace beckhoff {

/// Base class of every exception thrown by this library.
class Error : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// Invalid configuration: malformed parameter file, bad structure definition,
/// type sizes that do not match the PLC symbols, ...
class ConfigError : public Error {
 public:
  using Error::Error;
};

/// Unknown name: variable, structure field or type that was not defined, or a
/// variable used in a way the configuration does not allow (e.g. writing a
/// variable that is not listed in "variables_to_write").
class KeyError : public Error {
 public:
  using Error::Error;
};

/// The operation does not apply to the value's type (e.g. reading a structure
/// as an integer, or writing a value of another type to a variable).
class TypeError : public Error {
 public:
  using Error::Error;
};

/// A value does not fit: numeric overflow, string too long, array index out
/// of bounds, wrong number of array elements, ...
class RangeError : public Error {
 public:
  using Error::Error;
};

/// An ADS request failed. code() holds the ADS return code.
class AdsError : public Error {
 public:
  AdsError(long code, const std::string& context);

  /// ADS return code (see adsErrorString()).
  long code() const noexcept { return code_; }

 private:
  long code_;
};

/// Human readable description of an ADS return code.
std::string adsErrorString(long code);

}  // namespace beckhoff
