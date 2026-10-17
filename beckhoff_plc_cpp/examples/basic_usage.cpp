// Tour of the BeckhoffPLC API with the variables of config/params.json.
//
//   basic_usage <params.json> [--net-id ID]
//
// Without a PLC, run it against the fake one:
//   ./build/test/fake_ads_server --config ../config/params.json &
//   ./build/examples/basic_usage ../config/params.json --net-id 127.0.0.1.1.1
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "beckhoff_plc/beckhoff_plc.hpp"

// A C++ struct with the same memory layout as CustomStruct1_s can be read and
// written directly (fastest: no conversion at all).
struct CustomStruct1 {
  std::uint16_t attr1;
  bool attr2;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: basic_usage <params.json> [--net-id ID]\n";
    return 2;
  }
  try {
    // Same parameter file as the Python version (fields of "PLC").
    beckhoff::Config config = beckhoff::Config::fromFile(argv[1]);
    if (argc > 3 && std::string(argv[2]) == "--net-id") config.netId = argv[3];
    beckhoff::BeckhoffPLC plc(config);

    // Like addObserver() in Python: called with the current values of the
    // monitored variables, then whenever they change.
    plc.addObserver([](const beckhoff::Notification& n) {
      std::cout << "changed: " << n.name << " = " << n.value << std::endl;
    });

    // Create a value of a custom structure and fill it field by field.
    beckhoff::Value obj = plc.createValue("CustomStruct2_s");
    obj["attr1"] = 42;
    obj["attr2"]["attr1"] = 7;
    obj["attr2"]["attr2"] = true;
    obj["attr3"] = {true, false};

    // Or from JSON, with the dicts and lists of the Python version.
    beckhoff::Value other = plc.createValue(
        "CustomStruct2_s", {{"attr1", 1}, {"attr2", {{"attr1", 2}, {"attr2", false}}}, {"attr3", {false, true}}});

    // GVL.obj1 is ARRAY[0..9] OF CustomStruct2_s
    beckhoff::Value list = plc.variable("GVL.obj1").createValue();
    list[0] = obj;
    list[1] = other;
    plc.setVariable("GVL.obj1", list);
    plc.setVariable("GVL.var1", true);
    std::cout << "GVL.obj1[0] = " << plc.getVariable("GVL.obj1")[0] << std::endl;

    // Typed reads.
    const bool var1 = plc.getVariable<bool>("GVL.var1");
    const auto var2 = plc.getVariable<std::vector<int>>("GVL.var2");
    const auto obj2 = plc.getVariable<CustomStruct1>("GVL.obj2");
    std::cout << "GVL.var1 = " << var1 << ", GVL.var2[0] = " << var2.at(0) << ", GVL.obj2.attr1 = " << obj2.attr1
              << std::endl;

    // Fast path for control loops: look up the variable once, reuse the value
    // (no allocation) and read several variables in a single ADS request.
    beckhoff::Variable& obj1 = plc.variable("GVL.obj1");
    beckhoff::ValueRef speed = list[2]["attr1"];  // bound once
    std::vector<beckhoff::Value> values;
    for (int cycle = 0; cycle < 3; ++cycle) {
      speed = cycle;
      obj1.write(list);
      plc.getVariables({"GVL.obj2", "GVL.var1", "GVL.var2"}, values);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "GVL.var2 = " << values[2] << std::endl;
  } catch (const beckhoff::Error& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
