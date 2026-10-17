#include "beckhoff/beckhoff_plc.hpp"
#include <iostream>

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            std::cerr << "Usage: beckhoff_example params.json\nReads the configured variables_to_monitor; no writes.\n";
            return 2;
        }
        auto plc = beckhoff::BeckhoffPLC::fromFile(argv[1]);
        plc->addObserver([](const std::string& name, const beckhoff::Json& value) {
            std::cout << name << " = " << value.dump() << '\n';
        });
        // Zero defaults, then edit fields using JSON pointers. This stays local.
        auto value = plc->makeValue("CustomStruct1_s");
        value.set("/attr1", 42);
        value.set("/attr2", true);
        std::cout << "Local custom value: " << value.toJson().dump() << '\n';
        // Explicit write example: plc->setVariable("GVL.var1", true);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
