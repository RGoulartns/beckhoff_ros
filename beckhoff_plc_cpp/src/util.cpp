#include "util.hpp"

#include <cstdio>

#include "beckhoff_plc/errors.hpp"

namespace beckhoff {
namespace util {

namespace {

void appendUtf8(std::string& out, char32_t cp) {
  if (cp < 0x80) {
    out += static_cast<char>(cp);
  } else if (cp < 0x800) {
    out += static_cast<char>(0xC0 | (cp >> 6));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  } else if (cp < 0x10000) {
    out += static_cast<char>(0xE0 | (cp >> 12));
    out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  } else {
    out += static_cast<char>(0xF0 | (cp >> 18));
    out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
    out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  }
}

[[noreturn]] void invalidUtf8(std::size_t position) {
  throw RangeError("invalid UTF-8 sequence at byte " + std::to_string(position));
}

/// Decodes the code point starting at text[pos] and advances pos.
char32_t decodeUtf8(std::string_view text, std::size_t& pos) {
  const auto byte = [&](std::size_t i) { return static_cast<unsigned char>(text[i]); };
  const std::size_t start = pos;
  const unsigned char lead = byte(pos++);
  if (lead < 0x80) return lead;

  std::size_t extra;
  char32_t cp;
  char32_t minimum;
  if ((lead & 0xE0) == 0xC0) {
    extra = 1, cp = lead & 0x1F, minimum = 0x80;
  } else if ((lead & 0xF0) == 0xE0) {
    extra = 2, cp = lead & 0x0F, minimum = 0x800;
  } else if ((lead & 0xF8) == 0xF0) {
    extra = 3, cp = lead & 0x07, minimum = 0x10000;
  } else {
    invalidUtf8(start);
  }
  for (std::size_t i = 0; i < extra; ++i, ++pos) {
    if (pos >= text.size() || (byte(pos) & 0xC0) != 0x80) invalidUtf8(start);
    cp = (cp << 6) | (byte(pos) & 0x3F);
  }
  if (cp < minimum || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) invalidUtf8(start);
  return cp;
}

}  // namespace

bool isIdentifier(std::string_view text) {
  bool expectStart = true;
  for (char c : text) {
    if (expectStart) {
      if (!isIdentifierStart(c)) return false;
      expectStart = false;
    } else if (c == '.') {
      expectStart = true;
    } else if (!isIdentifierChar(c)) {
      return false;
    }
  }
  return !expectStart;
}

std::string latin1ToUtf8(std::string_view latin1) {
  std::string out;
  out.reserve(latin1.size());
  for (char c : latin1) appendUtf8(out, static_cast<unsigned char>(c));
  return out;
}

std::string utf8ToLatin1(std::string_view utf8) {
  std::string out;
  out.reserve(utf8.size());
  for (std::size_t pos = 0; pos < utf8.size();) {
    const char32_t cp = decodeUtf8(utf8, pos);
    if (cp > 0xFF) {
      char buffer[64];
      std::snprintf(buffer, sizeof(buffer), "character U+%04X", static_cast<unsigned>(cp));
      throw RangeError(std::string(buffer) + " cannot be stored in a STRING (Latin-1), use a WSTRING");
    }
    out += static_cast<char>(cp);
  }
  return out;
}

std::string utf16ToUtf8(const std::uint8_t* data, std::size_t units) {
  const auto unit = [&](std::size_t i) {
    return static_cast<char16_t>(data[2 * i] | (data[2 * i + 1] << 8));
  };
  std::string out;
  out.reserve(units);
  for (std::size_t i = 0; i < units; ++i) {
    const char16_t u = unit(i);
    if (u >= 0xD800 && u <= 0xDBFF && i + 1 < units && unit(i + 1) >= 0xDC00 && unit(i + 1) <= 0xDFFF) {
      appendUtf8(out, 0x10000 + ((static_cast<char32_t>(u) - 0xD800) << 10) + (unit(i + 1) - 0xDC00));
      ++i;
    } else if (u >= 0xD800 && u <= 0xDFFF) {
      appendUtf8(out, 0xFFFD);  // unpaired surrogate
    } else {
      appendUtf8(out, u);
    }
  }
  return out;
}

std::u16string utf8ToUtf16(std::string_view utf8) {
  std::u16string out;
  out.reserve(utf8.size());
  for (std::size_t pos = 0; pos < utf8.size();) {
    const char32_t cp = decodeUtf8(utf8, pos);
    if (cp < 0x10000) {
      out += static_cast<char16_t>(cp);
    } else {
      out += static_cast<char16_t>(0xD800 + ((cp - 0x10000) >> 10));
      out += static_cast<char16_t>(0xDC00 + ((cp - 0x10000) & 0x3FF));
    }
  }
  return out;
}

}  // namespace util
}  // namespace beckhoff
