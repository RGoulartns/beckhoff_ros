// Internal helpers shared by the implementation files (not installed).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace beckhoff {
namespace util {

inline char lowerChar(char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c; }

inline std::string toLower(std::string_view text) {
  std::string result(text);
  for (char& c : result) c = lowerChar(c);
  return result;
}

/// Case-insensitive (ASCII) comparison, the way TwinCAT compares identifiers.
inline bool iequals(std::string_view a, std::string_view b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (lowerChar(a[i]) != lowerChar(b[i])) return false;
  }
  return true;
}

inline bool isIdentifierStart(char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_'; }
inline bool isIdentifierChar(char c) { return isIdentifierStart(c) || (c >= '0' && c <= '9'); }

/// True for "Name", "_name1" and namespaced names such as "Tc2_MC2.ST_AxisStatus".
bool isIdentifier(std::string_view text);

/// STRING contents (TwinCAT uses Latin-1 / Windows-1252) to UTF-8.
std::string latin1ToUtf8(std::string_view latin1);
/// UTF-8 to Latin-1; throws RangeError for characters that Latin-1 cannot represent.
std::string utf8ToLatin1(std::string_view utf8);
/// UTF-16 (little endian, as stored in a WSTRING) to UTF-8.
std::string utf16ToUtf8(const std::uint8_t* data, std::size_t units);
/// UTF-8 to UTF-16 code units; throws RangeError on invalid UTF-8.
std::u16string utf8ToUtf16(std::string_view utf8);

}  // namespace util
}  // namespace beckhoff
