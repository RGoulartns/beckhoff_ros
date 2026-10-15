#include "beckhoff_ros/value.hpp"

#include <cctype>
#include <charconv>
#include <clocale>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>

namespace beckhoff_ros
{

namespace
{

const char * typeName(Value::Type type)
{
  switch (type) {
    case Value::Type::None: return "NoneType";
    case Value::Type::Bool: return "bool";
    case Value::Type::Int: return "int";
    case Value::Type::Float: return "float";
    case Value::Type::String: return "str";
    case Value::Type::List: return "list";
    case Value::Type::Dict: return "dict";
  }
  return "unknown";
}

std::string typeError(const char * expected, const Value & value)
{
  return std::string("expected ") + expected + ", got " + typeName(value.type()) + " (" +
         value.repr() + ")";
}

bool isWhitespace(char c)
{
  return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f';
}

std::string strip(const std::string & text)
{
  std::size_t begin = 0;
  std::size_t end = text.size();
  while (begin < end && isWhitespace(text[begin])) {
    ++begin;
  }
  while (end > begin && isWhitespace(text[end - 1])) {
    --end;
  }
  return text.substr(begin, end - begin);
}

std::string toLower(std::string text)
{
  for (auto & c : text) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return text;
}

bool isDigit(char c, int base)
{
  if (base == 16) {
    return std::isxdigit(static_cast<unsigned char>(c)) != 0;
  }
  return c >= '0' && c < static_cast<char>('0' + base);
}

int digitValue(char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  return std::tolower(static_cast<unsigned char>(c)) - 'a' + 10;
}

/// Converts a run of digits (already validated, no underscores) to an integer Value.
Value makeInteger(bool negative, const std::string & digits, int base)
{
  std::uint64_t magnitude = 0;
  for (char c : digits) {
    const auto digit = static_cast<std::uint64_t>(digitValue(c));
    if (magnitude > (std::numeric_limits<std::uint64_t>::max() - digit) / base) {
      throw ValueError("integer '" + digits + "' is too large");
    }
    magnitude = magnitude * base + digit;
  }
  if (!negative) {
    return Value(magnitude);
  }
  constexpr auto kMinMagnitude = static_cast<std::uint64_t>(1) << 63;
  if (magnitude > kMinMagnitude) {
    throw ValueError("integer '-" + digits + "' is too small");
  }
  if (magnitude == kMinMagnitude) {
    return Value(std::numeric_limits<std::int64_t>::min());
  }
  return Value(-static_cast<std::int64_t>(magnitude));
}

/// Converts a validated decimal floating point text ("-1.5e3", no underscores) to double.
/// Out of range values become +-inf or 0, like Python's float().
double toDouble(std::string text)
{
  // strtod honours the C locale decimal point, make sure it matches.
  const char * decimal_point = std::localeconv()->decimal_point;
  if (decimal_point != nullptr && decimal_point[0] != '\0' && decimal_point[0] != '.') {
    for (auto & c : text) {
      if (c == '.') {
        c = decimal_point[0];
      }
    }
  }
  return std::strtod(text.c_str(), nullptr);
}

/// Scans digits with optional single underscores between them (Python numeric literal rules).
/// Returns the digits without underscores; empty if there are no digits at `pos`.
std::string scanDigits(const std::string & text, std::size_t & pos, int base)
{
  std::string digits;
  while (pos < text.size()) {
    if (isDigit(text[pos], base)) {
      digits += text[pos++];
    } else if (
      text[pos] == '_' && !digits.empty() && pos + 1 < text.size() &&
      isDigit(text[pos + 1], base))
    {
      ++pos;
    } else {
      break;
    }
  }
  return digits;
}

/// Parses a number (without sign) at `pos`: decimal/hex/octal/binary integers, decimal floats,
/// inf and nan. Decimal integers are returned as floats if `as_float`. Returns false (pos
/// unchanged) if there is no number at `pos`.
bool scanNumber(
  const std::string & text, std::size_t & pos, bool negative, Value & result,
  bool as_float = false)
{
  const std::size_t start = pos;
  // inf / infinity / nan (case insensitive)
  for (const char * word : {"infinity", "inf", "nan"}) {
    const std::string w(word);
    if (toLower(text.substr(pos, w.size())) == w &&
      (pos + w.size() == text.size() ||
      !std::isalnum(static_cast<unsigned char>(text[pos + w.size()]))))
    {
      pos += w.size();
      const double value = w == "nan" ? std::nan("") : std::numeric_limits<double>::infinity();
      result = Value(negative ? -value : value);
      return true;
    }
  }
  // 0x / 0o / 0b prefixed integers
  if (pos + 1 < text.size() && text[pos] == '0') {
    const char prefix = static_cast<char>(std::tolower(static_cast<unsigned char>(text[pos + 1])));
    const int base = prefix == 'x' ? 16 : prefix == 'o' ? 8 : prefix == 'b' ? 2 : 0;
    if (base != 0) {
      pos += 2;
      if (pos < text.size() && text[pos] == '_') {
        ++pos;
      }
      const std::string digits = scanDigits(text, pos, base);
      if (digits.empty()) {
        pos = start;
        return false;
      }
      result = makeInteger(negative, digits, base);
      return true;
    }
  }
  // decimal integer or float
  std::string mantissa = scanDigits(text, pos, 10);
  bool is_float = false;
  std::string fraction;
  if (pos < text.size() && text[pos] == '.') {
    std::size_t after_dot = pos + 1;
    fraction = scanDigits(text, after_dot, 10);
    if (mantissa.empty() && fraction.empty()) {
      pos = start;
      return false;
    }
    pos = after_dot;
    is_float = true;
  } else if (mantissa.empty()) {
    pos = start;
    return false;
  }
  std::string exponent;
  if (pos < text.size() && (text[pos] == 'e' || text[pos] == 'E')) {
    std::size_t exp_pos = pos + 1;
    std::string sign;
    if (exp_pos < text.size() && (text[exp_pos] == '+' || text[exp_pos] == '-')) {
      sign = text[exp_pos++];
    }
    const std::string exp_digits = scanDigits(text, exp_pos, 10);
    if (!exp_digits.empty()) {
      exponent = "e" + sign + exp_digits;
      pos = exp_pos;
      is_float = true;
    }
  }
  if (!is_float && !as_float) {
    result = makeInteger(negative, mantissa, 10);
    return true;
  }
  const std::string number = (negative ? "-" : "") + (mantissa.empty() ? "0" : mantissa) + "." +
    (fraction.empty() ? "0" : fraction) + exponent;
  result = Value(toDouble(number));
  return true;
}

/// Recursive descent parser for Python literals (and JSON).
class LiteralParser
{
public:
  explicit LiteralParser(const std::string & text)
  : text_(text) {}

  Value parse()
  {
    Value value = parseValue(0);
    skipWhitespace();
    if (pos_ != text_.size()) {
      fail("unexpected '" + text_.substr(pos_, 10) + "'");
    }
    return value;
  }

private:
  static constexpr int kMaxDepth = 512;

  [[noreturn]] void fail(const std::string & what) const
  {
    const std::string text = text_.size() > 100 ? text_.substr(0, 100) + "..." : text_;
    throw ValueError(
            "invalid literal at position " + std::to_string(pos_) + ": " + what + " in '" + text +
            "'");
  }

  void skipWhitespace()
  {
    while (pos_ < text_.size() && isWhitespace(text_[pos_])) {
      ++pos_;
    }
  }

  bool consume(char c)
  {
    skipWhitespace();
    if (pos_ < text_.size() && text_[pos_] == c) {
      ++pos_;
      return true;
    }
    return false;
  }

  Value parseValue(int depth)
  {
    if (depth > kMaxDepth) {
      fail("too deeply nested");
    }
    skipWhitespace();
    if (pos_ >= text_.size()) {
      fail("unexpected end of input");
    }
    const char c = text_[pos_];
    if (c == '{') {
      ++pos_;
      return parseDict(depth);
    }
    if (c == '[') {
      ++pos_;
      return parseSequence(depth, ']');
    }
    if (c == '(') {
      ++pos_;
      return parseSequence(depth, ')');
    }
    if (c == '\'' || c == '"') {
      return Value(parseString());
    }
    return parseScalar();
  }

  Value parseDict(int depth)
  {
    Value::Dict dict;
    while (!consume('}')) {  // also accepts a trailing comma
      skipWhitespace();
      if (pos_ >= text_.size() || (text_[pos_] != '\'' && text_[pos_] != '"')) {
        fail("expected a string key");
      }
      std::string key = parseString();
      if (!consume(':')) {
        fail("expected ':'");
      }
      Value value = parseValue(depth + 1);
      bool replaced = false;
      for (auto & item : dict) {  // like Python, a repeated key keeps the last value
        if (item.first == key) {
          item.second = std::move(value);
          replaced = true;
          break;
        }
      }
      if (!replaced) {
        dict.emplace_back(std::move(key), std::move(value));
      }
      if (!consume(',')) {
        if (!consume('}')) {
          fail("expected ',' or '}'");
        }
        break;
      }
    }
    return Value(std::move(dict));
  }

  Value parseSequence(int depth, char close)
  {
    Value::List list;
    std::size_t commas = 0;
    while (!consume(close)) {  // also accepts a trailing comma
      list.push_back(parseValue(depth + 1));
      if (consume(',')) {
        ++commas;
        continue;
      }
      if (!consume(close)) {
        fail(std::string("expected ',' or '") + close + "'");
      }
      break;
    }
    if (close == ')' && list.size() == 1 && commas == 0) {
      return std::move(list.front());  // "(x)" is just x, "(x,)" is a tuple
    }
    return Value(std::move(list));
  }

  void appendUtf8(std::string & out, std::uint32_t code_point)
  {
    if (code_point < 0x80) {
      out += static_cast<char>(code_point);
    } else if (code_point < 0x800) {
      out += static_cast<char>(0xC0 | (code_point >> 6));
      out += static_cast<char>(0x80 | (code_point & 0x3F));
    } else if (code_point < 0x10000) {
      out += static_cast<char>(0xE0 | (code_point >> 12));
      out += static_cast<char>(0x80 | ((code_point >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (code_point & 0x3F));
    } else if (code_point < 0x110000) {
      out += static_cast<char>(0xF0 | (code_point >> 18));
      out += static_cast<char>(0x80 | ((code_point >> 12) & 0x3F));
      out += static_cast<char>(0x80 | ((code_point >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (code_point & 0x3F));
    } else {
      fail("invalid unicode escape");
    }
  }

  std::uint32_t parseHex(std::size_t count)
  {
    if (pos_ + count > text_.size()) {
      fail("truncated escape sequence");
    }
    std::uint32_t value = 0;
    for (std::size_t i = 0; i < count; ++i) {
      const char c = text_[pos_++];
      if (!isDigit(c, 16)) {
        fail("invalid escape sequence");
      }
      value = value * 16 + static_cast<std::uint32_t>(digitValue(c));
    }
    return value;
  }

  std::string parseString()
  {
    const char quote = text_[pos_++];
    std::string out;
    while (true) {
      if (pos_ >= text_.size()) {
        fail("unterminated string");
      }
      const char c = text_[pos_++];
      if (c == quote) {
        return out;
      }
      if (c != '\\') {
        out += c;
        continue;
      }
      if (pos_ >= text_.size()) {
        fail("unterminated string");
      }
      const char e = text_[pos_++];
      switch (e) {
        case '\\': out += '\\'; break;
        case '\'': out += '\''; break;
        case '"': out += '"'; break;
        case '/': out += '/'; break;
        case 'n': out += '\n'; break;
        case 'r': out += '\r'; break;
        case 't': out += '\t'; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case 'v': out += '\v'; break;
        case 'a': out += '\a'; break;
        case 'x': appendUtf8(out, parseHex(2)); break;
        case 'u': {
            std::uint32_t code_point = parseHex(4);
            // JSON encodes characters outside the BMP as surrogate pairs
            if (code_point >= 0xD800 && code_point < 0xDC00 &&
              text_.compare(pos_, 2, "\\u") == 0)
            {
              pos_ += 2;
              const std::uint32_t low = parseHex(4);
              if (low < 0xDC00 || low > 0xDFFF) {
                fail("invalid surrogate pair");
              }
              code_point = 0x10000 + ((code_point - 0xD800) << 10) + (low - 0xDC00);
            }
            appendUtf8(out, code_point);
            break;
          }
        case 'U': appendUtf8(out, parseHex(8)); break;
        default:
          if (e >= '0' && e <= '7') {  // octal escape, up to 3 digits
            std::uint32_t code_point = static_cast<std::uint32_t>(e - '0');
            for (int i = 0; i < 2 && pos_ < text_.size() && isDigit(text_[pos_], 8); ++i) {
              code_point = code_point * 8 + static_cast<std::uint32_t>(text_[pos_++] - '0');
            }
            appendUtf8(out, code_point);
          } else {  // Python keeps unknown escapes as they are
            out += '\\';
            out += e;
          }
      }
    }
  }

  Value parseScalar()
  {
    const std::size_t start = pos_;
    bool negative = false;
    bool has_sign = false;
    if (text_[pos_] == '+' || text_[pos_] == '-') {
      negative = text_[pos_] == '-';
      has_sign = true;
      ++pos_;
      skipWhitespace();
    }
    Value number;
    if (pos_ < text_.size() && scanNumber(text_, pos_, negative, number)) {
      if (pos_ < text_.size() && (std::isalnum(static_cast<unsigned char>(text_[pos_])) ||
        text_[pos_] == '_' || text_[pos_] == '.'))
      {
        fail("invalid number");
      }
      return number;
    }
    if (has_sign) {
      pos_ = start;
      fail("expected a number");
    }
    std::size_t end = pos_;
    while (end < text_.size() &&
      (std::isalnum(static_cast<unsigned char>(text_[end])) || text_[end] == '_'))
    {
      ++end;
    }
    const std::string word = text_.substr(pos_, end - pos_);
    if (word == "True" || word == "true") {
      pos_ = end;
      return Value(true);
    }
    if (word == "False" || word == "false") {
      pos_ = end;
      return Value(false);
    }
    if (word == "None" || word == "null") {
      pos_ = end;
      return Value();
    }
    fail(word.empty() ? "unexpected character" : "unknown name '" + word + "'");
  }

  const std::string & text_;
  std::size_t pos_ = 0;
};

/// Validates the text given to Python's int(): base 10, surrounding whitespace, sign and
/// underscores allowed. Returns the digits (without underscores).
std::string pythonIntDigits(const std::string & text, bool & negative)
{
  const std::string s = strip(text);
  std::size_t pos = 0;
  negative = false;
  if (pos < s.size() && (s[pos] == '+' || s[pos] == '-')) {
    negative = s[pos++] == '-';
  }
  const std::string digits = scanDigits(s, pos, 10);
  if (digits.empty() || pos != s.size()) {
    throw ValueError("invalid literal for int() with base 10: " + formatPythonString(text));
  }
  return digits;
}

/// Python's int(text).
Value pythonInt(const std::string & text)
{
  bool negative = false;
  const std::string digits = pythonIntDigits(text, negative);
  return makeInteger(negative, digits, 10);
}

/// Python's float(text).
Value pythonFloat(const std::string & text)
{
  const std::string s = strip(text);
  std::size_t pos = 0;
  bool negative = false;
  if (pos < s.size() && (s[pos] == '+' || s[pos] == '-')) {
    negative = s[pos++] == '-';
  }
  Value number;
  // float() only accepts decimal numbers (no 0x/0o/0b prefixes)
  const bool prefixed = pos + 1 < s.size() && s[pos] == '0' &&
    std::string("xXoObB").find(s[pos + 1]) != std::string::npos;
  if (prefixed || !scanNumber(s, pos, negative, number, true) || pos != s.size()) {
    throw ValueError("could not convert string to float: " + formatPythonString(text));
  }
  return number;
}

}  // namespace

Value Value::list(std::initializer_list<Value> items)
{
  return Value(List(items));
}

Value Value::dict(std::initializer_list<std::pair<std::string, Value>> items)
{
  return Value(Dict(items));
}

Value Value::parseLiteral(const std::string & text)
{
  return LiteralParser(text).parse();
}

Value::Type Value::type() const
{
  switch (data_.index()) {
    case 0: return Type::None;
    case 1: return Type::Bool;
    case 2:
    case 3: return Type::Int;
    case 4: return Type::Float;
    case 5: return Type::String;
    case 6: return Type::List;
    default: return Type::Dict;
  }
}

bool Value::asBool() const
{
  if (const auto * value = std::get_if<bool>(&data_)) {
    return *value;
  }
  throw ValueError(typeError("bool", *this));
}

std::int64_t Value::asInt() const
{
  if (const auto * value = std::get_if<std::int64_t>(&data_)) {
    return *value;
  }
  if (const auto * value = std::get_if<bool>(&data_)) {
    return *value ? 1 : 0;
  }
  if (std::holds_alternative<std::uint64_t>(data_)) {
    throw ValueError("integer " + str() + " does not fit in int64");
  }
  throw ValueError(typeError("int", *this));
}

std::uint64_t Value::asUInt() const
{
  if (const auto * value = std::get_if<std::uint64_t>(&data_)) {
    return *value;
  }
  const std::int64_t value = asInt();
  if (value < 0) {
    throw ValueError("integer " + str() + " is negative");
  }
  return static_cast<std::uint64_t>(value);
}

double Value::asFloat() const
{
  if (const auto * value = std::get_if<double>(&data_)) {
    return *value;
  }
  if (const auto * value = std::get_if<std::uint64_t>(&data_)) {
    return static_cast<double>(*value);
  }
  if (isInt() || isBool()) {
    return static_cast<double>(asInt());
  }
  throw ValueError(typeError("float", *this));
}

const std::string & Value::asString() const
{
  if (const auto * value = std::get_if<std::string>(&data_)) {
    return *value;
  }
  throw ValueError(typeError("str", *this));
}

const Value::List & Value::asList() const
{
  if (const auto * value = std::get_if<List>(&data_)) {
    return *value;
  }
  throw ValueError(typeError("list", *this));
}

Value::List & Value::asList()
{
  if (auto * value = std::get_if<List>(&data_)) {
    return *value;
  }
  throw ValueError(typeError("list", *this));
}

const Value::Dict & Value::asDict() const
{
  if (const auto * value = std::get_if<Dict>(&data_)) {
    return *value;
  }
  throw ValueError(typeError("dict", *this));
}

Value::Dict & Value::asDict()
{
  if (auto * value = std::get_if<Dict>(&data_)) {
    return *value;
  }
  throw ValueError(typeError("dict", *this));
}

const Value * Value::find(const std::string & key) const
{
  if (const auto * dict = std::get_if<Dict>(&data_)) {
    for (const auto & item : *dict) {
      if (item.first == key) {
        return &item.second;
      }
    }
  }
  return nullptr;
}

const Value & Value::operator[](const std::string & key) const
{
  if (const auto * value = find(key)) {
    return *value;
  }
  asDict();  // throws if this is not a dict
  throw ValueError("key " + formatPythonString(key) + " not found");
}

const Value & Value::operator[](std::size_t index) const
{
  const auto & list = asList();
  if (index >= list.size()) {
    throw ValueError("list index " + std::to_string(index) + " out of range");
  }
  return list[index];
}

std::string Value::str() const
{
  if (const auto * value = std::get_if<std::string>(&data_)) {
    return *value;
  }
  return repr();
}

std::string Value::repr() const
{
  switch (data_.index()) {
    case 0: return "None";
    case 1: return std::get<bool>(data_) ? "True" : "False";
    case 2: return std::to_string(std::get<std::int64_t>(data_));
    case 3: return std::to_string(std::get<std::uint64_t>(data_));
    case 4: return formatPythonFloat(std::get<double>(data_));
    case 5: return formatPythonString(std::get<std::string>(data_));
    case 6: {
        std::string out = "[";
        for (const auto & item : std::get<List>(data_)) {
          out += (out.size() > 1 ? ", " : "") + item.repr();
        }
        return out + "]";
      }
    default: {
        std::string out = "{";
        for (const auto & item : std::get<Dict>(data_)) {
          out += (out.size() > 1 ? ", " : "") + formatPythonString(item.first) + ": " +
            item.second.repr();
        }
        return out + "}";
      }
  }
}

std::string Value::typeString() const
{
  return std::string("<class '") + typeName(type()) + "'>";
}

bool operator==(const Value & lhs, const Value & rhs)
{
  return lhs.data_ == rhs.data_;
}

Value parseTypedValue(const std::string & type, const std::string & text)
{
  std::string name = strip(type);
  // accept the type strings published in PlcVariable messages, e.g. "<class 'int'>"
  const std::string prefix = "<class '";
  const std::string suffix = "'>";
  if (name.size() > prefix.size() + suffix.size() && name.compare(0, prefix.size(), prefix) == 0 &&
    name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
  {
    name = name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
  }

  if (name == "bool") {
    // The Python version used bool(int(value)): "0" -> False, any other integer -> True
    const std::string word = toLower(strip(text));
    if (word == "true" || word == "false") {
      return Value(word == "true");
    }
    bool negative = false;
    const std::string digits = pythonIntDigits(text, negative);
    return Value(digits.find_first_not_of('0') != std::string::npos);
  }
  if (name == "int") {
    return pythonInt(text);
  }
  if (name == "float") {
    return pythonFloat(text);
  }
  if (name == "dict" || name == "list" || name == "tuple") {
    Value value = Value::parseLiteral(text);
    if (name == "dict" ? !value.isDict() : !value.isList()) {
      throw ValueError(
              "value " + formatPythonString(text) + " is not a " + name + " (it is a " +
              typeName(value.type()) + ")");
    }
    return value;
  }
  throw ValueError(
          "unsupported type " + formatPythonString(type) +
          " (supported types: bool, int, float, dict, list, tuple)");
}

std::string formatPythonFloat(double value)
{
  if (std::isnan(value)) {
    return "nan";
  }
  if (std::isinf(value)) {
    return value < 0 ? "-inf" : "inf";
  }
  // Shortest representation that round trips, e.g. "-1.2345e+17" (same digits Python uses)
  char buffer[64];
  const auto result = std::to_chars(
    buffer, buffer + sizeof(buffer), value, std::chars_format::scientific);
  const std::string scientific(buffer, result.ptr);

  const bool negative = scientific[0] == '-';
  const std::size_t exponent_pos = scientific.find('e');
  std::string digits;
  for (std::size_t i = negative ? 1 : 0; i < exponent_pos; ++i) {
    if (scientific[i] != '.') {
      digits += scientific[i];
    }
  }
  while (digits.size() > 1 && digits.back() == '0') {
    digits.pop_back();
  }
  const int exponent = std::atoi(scientific.c_str() + exponent_pos + 1);
  // value = 0.<digits> * 10^decimal_point
  const int decimal_point = exponent + 1;
  const int num_digits = static_cast<int>(digits.size());

  std::string out = negative ? "-" : "";
  if (decimal_point > -4 && decimal_point <= 16) {
    if (decimal_point <= 0) {
      out += "0." + std::string(static_cast<std::size_t>(-decimal_point), '0') + digits;
    } else if (decimal_point >= num_digits) {
      out += digits + std::string(static_cast<std::size_t>(decimal_point - num_digits), '0') + ".0";
    } else {
      out += digits.substr(0, static_cast<std::size_t>(decimal_point)) + "." +
        digits.substr(static_cast<std::size_t>(decimal_point));
    }
  } else {
    out += digits.substr(0, 1);
    if (num_digits > 1) {
      out += "." + digits.substr(1);
    }
    const int e = decimal_point - 1;
    const std::string e_digits = std::to_string(e < 0 ? -e : e);
    out += std::string(e < 0 ? "e-" : "e+") + (e_digits.size() < 2 ? "0" : "") + e_digits;
  }
  return out;
}

std::string formatPythonString(const std::string & value)
{
  const bool has_single = value.find('\'') != std::string::npos;
  const bool has_double = value.find('"') != std::string::npos;
  const char quote = has_single && !has_double ? '"' : '\'';
  std::string out(1, quote);
  for (const char c : value) {
    const auto u = static_cast<unsigned char>(c);
    if (c == quote || c == '\\') {
      out += '\\';
      out += c;
    } else if (c == '\t') {
      out += "\\t";
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if (u < 0x20 || u == 0x7F) {
      char escaped[5];
      std::snprintf(escaped, sizeof(escaped), "\\x%02x", u);
      out += escaped;
    } else {
      out += c;
    }
  }
  return out + quote;
}

}  // namespace beckhoff_ros
