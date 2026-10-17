#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

#include "beckhoff_plc/errors.hpp"
#include "beckhoff_plc/types.hpp"

using beckhoff::ConfigError;
using beckhoff::KeyError;
using beckhoff::Primitive;
using beckhoff::StructDefinition;
using beckhoff::Type;
using beckhoff::TypeError;
using beckhoff::TypePtr;
using beckhoff::TypeRegistry;

namespace {

struct Expected {
  const char* spec;
  const char* name;
  std::size_t size;
};

}  // namespace

TEST(Types, ElementaryTypes) {
  const TypeRegistry types;
  const Expected cases[] = {
      {"BOOL", "BOOL", 1},          {"SINT", "SINT", 1},     {"int8", "SINT", 1},     {"USINT", "USINT", 1},
      {"BYTE", "USINT", 1},         {"INT", "INT", 2},       {"UINT", "UINT", 2},     {"WORD", "UINT", 2},
      {"DINT", "DINT", 4},          {"int32", "DINT", 4},    {"UDINT", "UDINT", 4},   {"DWORD", "UDINT", 4},
      {"TIME", "UDINT", 4},         {"TOD", "UDINT", 4},     {"DATE", "UDINT", 4},    {"DT", "UDINT", 4},
      {"LINT", "LINT", 8},          {"int64", "LINT", 8},    {"ULINT", "ULINT", 8},   {"LWORD", "ULINT", 8},
      {"LTIME", "ULINT", 8},        {"REAL", "REAL", 4},     {"float", "REAL", 4},    {"LREAL", "LREAL", 8},
      {"double", "LREAL", 8},       {"STRING", "STRING(80)", 81}, {"STRING(20)", "STRING(20)", 21},
      {"STRING[20]", "STRING(20)", 21}, {"WSTRING", "WSTRING(80)", 162}, {"WSTRING(10)", "WSTRING(10)", 22},
  };
  for (const Expected& c : cases) {
    SCOPED_TRACE(c.spec);
    const TypePtr type = types.parse(c.spec);
    EXPECT_EQ(type->name(), c.name);
    EXPECT_EQ(type->size(), c.size);
    EXPECT_TRUE(type->isPrimitive());
  }
}

TEST(Types, PythonVersionNames) {
  // BeckhoffPLC.cTypes of beckhoff_plc.py
  const TypeRegistry types;
  EXPECT_EQ(types.parse("bool")->primitive(), Primitive::Bool);
  EXPECT_EQ(types.parse("int")->primitive(), Primitive::Int16);  // ctypes.c_int16, like IEC INT
  EXPECT_EQ(types.parse("int16")->primitive(), Primitive::Int16);
  EXPECT_EQ(types.parse("uint16")->primitive(), Primitive::UInt16);
  EXPECT_EQ(types.parse("uint8")->primitive(), Primitive::UInt8);
  EXPECT_EQ(types.parse("real")->primitive(), Primitive::Real32);
}

TEST(Types, NamesAreCaseInsensitive) {
  TypeRegistry types;
  types.defineStruct({"ST_Point", {{"x", "lreal"}, {"y", "LReal"}}, {}});
  EXPECT_EQ(types.parse("st_point"), types.parse("ST_POINT"));
  EXPECT_EQ(types.parse("Bool"), types.parse("BOOL"));
  EXPECT_EQ(types.parse("array[0..1] of st_point")->size(), 32u);
  EXPECT_TRUE(types.contains("string"));
}

TEST(Types, Arrays) {
  const TypeRegistry types;
  const TypePtr a = types.parse("ARRAY[0..9] OF UINT");
  EXPECT_TRUE(a->isArray());
  EXPECT_EQ(a->length(), 10u);
  EXPECT_EQ(a->size(), 20u);
  EXPECT_EQ(a->elementType().name(), "UINT");
  EXPECT_EQ(a->name(), "ARRAY[0..9] OF UINT");

  const TypePtr b = types.parse("array [ 1 .. 10 ] of uint");
  EXPECT_EQ(b->length(), 10u);
  EXPECT_EQ(b->lowerBound(), 1);
  EXPECT_TRUE(b->sameLayout(*a));

  EXPECT_EQ(types.parse("ARRAY[-5..5] OF SINT")->length(), 11u);

  // Multi-dimensional arrays are arrays of arrays, the last index varies fastest.
  const TypePtr m = types.parse("ARRAY[1..2, 0..2] OF INT");
  EXPECT_EQ(m->length(), 2u);
  EXPECT_EQ(m->elementType().length(), 3u);
  EXPECT_EQ(m->size(), 12u);
  EXPECT_TRUE(m->sameLayout(*types.parse("ARRAY[0..1] OF ARRAY[0..2] OF INT")));

  const TypePtr s = types.parse("ARRAY[0..2] OF STRING(10)");
  EXPECT_EQ(s->size(), 33u);
  EXPECT_EQ(s->alignment(), 1u);
}

TEST(Types, StructLayoutFollowsPackMode) {
  // STRUCT a : BOOL; b : LREAL; c : INT; END_STRUCT with each pack mode.
  struct Case {
    int packMode;
    std::size_t b, c, size, alignment;
  };
  for (const Case& c : {Case{0, 1, 9, 11, 1}, Case{1, 1, 9, 11, 1}, Case{2, 2, 10, 12, 2}, Case{4, 4, 12, 16, 4},
                        Case{8, 8, 16, 24, 8}}) {
    SCOPED_TRACE(c.packMode);
    TypeRegistry types;
    const TypePtr t = types.defineStruct({"ST_Test", {{"a", "BOOL"}, {"b", "LREAL"}, {"c", "INT"}}, c.packMode});
    EXPECT_EQ(t->field("a").offset, 0u);
    EXPECT_EQ(t->field("b").offset, c.b);
    EXPECT_EQ(t->field("c").offset, c.c);
    EXPECT_EQ(t->size(), c.size);
    EXPECT_EQ(t->alignment(), c.alignment);
  }
}

// The layout rules are those of C compilers (and of Python ctypes, which the
// Python version uses): check them against the compiler with #pragma pack.
#pragma pack(push, 1)
struct Pack1 {
  bool a;
  double b;
  std::int16_t c;
  float d;
  char e[6];
  std::int8_t f[2][3];
  std::uint32_t g;
};
#pragma pack(pop)
#pragma pack(push, 2)
struct Pack2 {
  bool a;
  double b;
  std::int16_t c;
  float d;
  char e[6];
  std::int8_t f[2][3];
  std::uint32_t g;
};
#pragma pack(pop)
#pragma pack(push, 4)
struct Pack4 {
  bool a;
  double b;
  std::int16_t c;
  float d;
  char e[6];
  std::int8_t f[2][3];
  std::uint32_t g;
};
#pragma pack(pop)
struct Natural {
  bool a;
  double b;
  std::int16_t c;
  float d;
  char e[6];
  std::int8_t f[2][3];
  std::uint32_t g;
};

template <class S>
void expectSameLayoutAsCompiler(int packMode) {
  SCOPED_TRACE(packMode);
  TypeRegistry types;
  const TypePtr t = types.defineStruct({"ST_Mixed",
                                        {{"a", "BOOL"},
                                         {"b", "LREAL"},
                                         {"c", "INT"},
                                         {"d", "REAL"},
                                         {"e", "STRING(5)"},
                                         {"f", "ARRAY[0..1, 0..2] OF SINT"},
                                         {"g", "UDINT"}},
                                        packMode});
  EXPECT_EQ(t->field("a").offset, offsetof(S, a));
  EXPECT_EQ(t->field("b").offset, offsetof(S, b));
  EXPECT_EQ(t->field("c").offset, offsetof(S, c));
  EXPECT_EQ(t->field("d").offset, offsetof(S, d));
  EXPECT_EQ(t->field("e").offset, offsetof(S, e));
  EXPECT_EQ(t->field("f").offset, offsetof(S, f));
  EXPECT_EQ(t->field("g").offset, offsetof(S, g));
  EXPECT_EQ(t->size(), sizeof(S));
  EXPECT_EQ(t->alignment(), alignof(S));
}

TEST(Types, LayoutMatchesTheCompiler) {
  expectSameLayoutAsCompiler<Pack1>(1);
  expectSameLayoutAsCompiler<Pack2>(2);
  expectSameLayoutAsCompiler<Pack4>(4);
  if (alignof(Natural) != 8) GTEST_SKIP() << "this platform aligns double to " << alignof(Natural) << " bytes";
  expectSameLayoutAsCompiler<Natural>(8);
}

TEST(Types, ParamsJsonStructures) {
  // config/params.json: same sizes as the ctypes structures of the Python version
  TypeRegistry types;
  types.define({}, {{"CustomStruct1_s", {{"attr1", "uint16"}, {"attr2", "bool"}}, {}},
                    {"CustomStruct2_s", {{"attr1", "uint16"}, {"attr2", "CustomStruct1_s"}, {"attr3", "ARRAY[0..1] OF bool"}}, {}}});
  const TypePtr s1 = types.parse("CustomStruct1_s");
  const TypePtr s2 = types.parse("CustomStruct2_s");
  EXPECT_EQ(s1->size(), 4u);
  EXPECT_EQ(s2->size(), 8u);
  EXPECT_EQ(s2->field("attr2").offset, 2u);
  EXPECT_EQ(s2->field("attr3").offset, 6u);
  EXPECT_EQ(types.parse("ARRAY[0..9] OF CustomStruct2_s")->size(), 80u);
  ASSERT_EQ(types.structures().size(), 2u);
  EXPECT_EQ(types.structures()[0]->name(), "CustomStruct1_s");
}

TEST(Types, NestedStructuresKeepTheirOwnPackMode) {
  TypeRegistry types;
  types.define({}, {{"ST_Inner", {{"a", "BOOL"}, {"b", "DINT"}}, 1},  // packed: 5 bytes, alignment 1
                    {"ST_Outer", {{"x", "BOOL"}, {"inner", "ST_Inner"}, {"y", "DINT"}}, 8}});
  const TypePtr outer = types.parse("ST_Outer");
  EXPECT_EQ(types.parse("ST_Inner")->size(), 5u);
  EXPECT_EQ(outer->field("inner").offset, 1u);
  EXPECT_EQ(outer->field("y").offset, 8u);
  EXPECT_EQ(outer->size(), 12u);
}

TEST(Types, DefinitionsMayComeInAnyOrder) {
  TypeRegistry types;
  types.define({{"T_Points", "ARRAY[0..2] OF ST_Point"}, {"E_State", "INT"}},
               {{"ST_Path", {{"points", "T_Points"}, {"state", "E_State"}}, {}},
                {"ST_Point", {{"x", "REAL"}, {"y", "REAL"}}, {}}});
  EXPECT_EQ(types.parse("ST_Path")->size(), 28u);
  EXPECT_EQ(types.parse("E_State")->name(), "INT");
  EXPECT_EQ(types.parse("t_points")->length(), 3u);
}

TEST(Types, DefinitionErrors) {
  const auto expectError = [](const std::vector<StructDefinition>& structures, const std::string& message) {
    SCOPED_TRACE(message);
    TypeRegistry types;
    try {
      types.define({}, structures);
      ADD_FAILURE() << "no exception";
    } catch (const ConfigError& e) {
      EXPECT_NE(std::string(e.what()).find(message), std::string::npos) << e.what();
    }
  };
  expectError({{"ST_A", {{"x", "ST_Missing"}}, {}}}, "structure 'ST_A', field 'x': unknown type 'ST_Missing'");
  expectError({{"ST_A", {{"b", "ST_B"}}, {}}, {"ST_B", {{"a", "ST_A"}}, {}}}, "recursive type definition: ST_A -> ST_B -> ST_A");
  expectError({{"ST_A", {{"x", "INT"}}, {}}, {"st_a", {{"x", "INT"}}, {}}}, "defined more than once");
  expectError({{"ST_A", {{"x", "INT"}, {"X", "INT"}}, {}}}, "more than one field named 'X'");
  expectError({{"ST_A", {}, {}}}, "has no fields");
  expectError({{"ST A", {{"x", "INT"}}, {}}}, "invalid structure name");
  expectError({{"Array", {{"x", "INT"}}, {}}}, "reserved word");
  expectError({{"DWORD", {{"x", "INT"}}, {}}}, "conflicts with a built-in type");
  expectError({{"ST_A", {{"x", "INT"}}, 3}}, "invalid pack mode 3");
  expectError({{"ST_A", {{"x", "ARRAY[1..0] OF INT"}}, {}}}, "invalid array range");
}

TEST(Types, FailedDefinitionsChangeNothing) {
  TypeRegistry types;
  EXPECT_THROW(types.define({}, {{"ST_Good", {{"x", "INT"}}, {}}, {"ST_Bad", {{"x", "ST_Missing"}}, {}}}), ConfigError);
  EXPECT_FALSE(types.contains("ST_Good"));
  EXPECT_TRUE(types.structures().empty());
}

TEST(Types, ParseErrors) {
  const TypeRegistry types;
  for (const char* spec : {"", "  ", "INT INT", "ARRAY[0..] OF INT", "ARRAY[0..1] INT", "ARRAY[0..1 OF INT",
                           "STRING(0)", "STRING(x)", "ST..A", "INT[3]"}) {
    SCOPED_TRACE(spec);
    EXPECT_THROW(types.parse(spec), ConfigError);
  }
  EXPECT_THROW(types.parse("ST_Unknown"), KeyError);
  EXPECT_THROW(types.parse("ARRAY[0..1] OF ST_Unknown"), KeyError);
}

TEST(Types, FieldLookup) {
  TypeRegistry types;
  const TypePtr t = types.defineStruct({"ST_A", {{"Speed", "LREAL"}, {"speed2", "LREAL"}}, {}});
  EXPECT_EQ(t->findField("Speed")->offset, 0u);
  EXPECT_EQ(t->findField("SPEED")->offset, 0u);  // case-insensitive, like TwinCAT
  EXPECT_EQ(t->findField("missing"), nullptr);
  EXPECT_THROW(t->field("missing"), KeyError);
  EXPECT_THROW(types.parse("INT")->field("x"), TypeError);
  EXPECT_EQ(types.parse("INT")->findField("x"), nullptr);
  EXPECT_THROW(types.parse("INT")->length(), TypeError);
  EXPECT_THROW(t->primitive(), TypeError);
}

TEST(Types, SameLayout) {
  TypeRegistry a;
  TypeRegistry b;
  const TypePtr ta = a.defineStruct({"ST_X", {{"x", "INT"}, {"y", "BOOL"}}, {}});
  const TypePtr tb = b.defineStruct({"ST_X", {{"X", "int16"}, {"Y", "bool"}}, {}});
  const TypePtr other = b.defineStruct({"ST_Y", {{"x", "INT"}, {"z", "BOOL"}}, {}});
  EXPECT_TRUE(ta->sameLayout(*tb));
  EXPECT_FALSE(ta->sameLayout(*other));  // different field name
  EXPECT_FALSE(a.parse("ARRAY[0..1] OF INT")->sameLayout(*a.parse("ARRAY[0..2] OF INT")));
  EXPECT_FALSE(a.parse("INT")->sameLayout(*a.parse("UINT")));
  EXPECT_FALSE(a.parse("STRING(10)")->sameLayout(*a.parse("STRING(11)")));
}

TEST(Types, LayoutDescription) {
  TypeRegistry types;
  const TypePtr t = types.defineStruct({"ST_A", {{"a", "BOOL"}, {"b", "LREAL"}}, 4});
  const std::string layout = t->layout();
  EXPECT_NE(layout.find("ST_A: 12 bytes, alignment 4, pack mode 4"), std::string::npos) << layout;
  EXPECT_NE(layout.find("+4 b : LREAL (8 bytes)"), std::string::npos) << layout;
}
