# beckhoff_ros

Beckhoff ADS + ROS2 (C++)

The BeckhoffPLC class expands [Beckhoff's ADS library](https://github.com/Beckhoff/ADS) (the
library pyads wraps) by allowing the usage of any data types: the custom DUTs (structures) of the
PLC program, lists of them, structures containing lists and other structures, etc. \
Users must specify the custom DUTs and the variables in a JSON file. Everything is loaded at
runtime, so the same package can be used with any PLC project without code changes.

This is the C++ port of the Python version (branch `main`). It uses the same configuration file,
topic and service.

## Build

Requirements:

- ROS 2 Jazzy or Kilted (Ubuntu 24.04)
- the `beckhoff_interfaces` package (see [ROS interface](#ros-interface)), in the same workspace
- Beckhoff's ADS library: downloaded automatically at build time if it is not installed (see below)

```bash
cd ~/ros2_ws/src
git clone -b cpp https://github.com/RGoulartns/beckhoff_ros.git
# + beckhoff_interfaces
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build
```

Beckhoff's ADS library is taken, in order of preference, from:

1. an installation made with its CMake build (`cmake -S ADS -B build && cmake --install build`;
   CMake package `Ads`, or `ads` before version 113.0.35)
2. an installation made with its meson build, or Beckhoff's `libads-dev` package
3. otherwise CMake downloads its sources from https://github.com/Beckhoff/ADS (needs git and
   internet) and builds them with this package. CMake options:
   - `-DBECKHOFF_ROS_ADS_GIT_TAG=<branch, tag or commit>` (default: commit `24e598a`, version
     113.0.36)
   - `-DBECKHOFF_ROS_ADS_GIT_REPOSITORY=<url>` to use a mirror
   - `-DFETCHCONTENT_SOURCE_DIR_BECKHOFF_ADS=/path/to/ADS` to use a local copy (offline builds)
   - `-DBECKHOFF_ROS_FETCH_ADS=OFF` to never download it

## Configuration

The PLC is described by the `PLC` object of a JSON file, `config/params.json` by default:

```json
{
    "PLC": {
        "net_id": "192.168.1.10.1.1",
        "client_net_id": "192.168.1.20.1.1",
        "username": "Administrator",
        "password": "1",
        "structures": {
            "CustomStruct1_s": {
                "attr1": "uint16",
                "attr2": "bool"
            },
            "CustomStruct2_s": {
                "attr1": "uint16",
                "attr2": "CustomStruct1_s",
                "attr3": ["bool", 2]
            }
        },
        "variables": {
            "GVL.obj1": ["CustomStruct2_s", 10],
            "GVL.obj2": "CustomStruct1_s",
            "GVL.var1": "bool",
            "GVL.var2": ["uint16", 5]
        },
        "variables_to_monitor": ["GVL.obj2", "GVL.var1"],
        "variables_to_write": ["GVL.var1", "GVL.obj1"]
    }
}
```

- `net_id`: AMS Net Id of the PLC. Its first 4 numbers are the IP address of the PLC.
- `client_net_id`, `username`, `password`: only used by `BeckhoffPLC::createRoute()`, which adds
  a route to this computer on the PLC. The PLC needs that route, which can also be added with
  TwinCAT.
- `structures`: the DUTs, with their fields in the same order as in the PLC program. A structure
  can only use the structures defined before it.
- `variables`: the PLC variables used, with their type.
- `variables_to_monitor`: variables published when they change.
- `variables_to_write`: variables that can be written.
- `pack` (optional): structure packing. `0`, the default, is TwinCAT 3's default layout; use `1`
  for structures declared with `{attribute 'pack_mode' := '1'}`.

A type is a type name or an array given as `[type name, size]`. Type names are the structures and:

| Type names                | PLC type      | Size (bytes) |
|---------------------------|---------------|--------------|
| `bool`                    | BOOL          | 1            |
| `int8`, `sint`            | SINT          | 1            |
| `uint8`, `usint`, `byte`  | USINT, BYTE   | 1            |
| `int`, `int16`            | INT           | 2            |
| `uint16`, `uint`, `word`  | UINT, WORD    | 2            |
| `int32`, `dint`           | DINT          | 4            |
| `uint32`, `udint`, `dword`| UDINT, DWORD  | 4            |
| `int64`, `lint`           | LINT          | 8            |
| `uint64`, `ulint`, `lword`| ULINT, LWORD  | 8            |
| `real`, `float`           | REAL          | 4            |
| `lreal`, `double`         | LREAL         | 8            |

More names can be added to the table at the top of `src/plc_types.cpp`.

If a structure does not match its PLC declaration, the PLC rejects the requests with ADS error
1797 (parameter size not correct).

## Usage

```bash
ros2 launch beckhoff_ros beckhoff_ros_launch.py config_file:=/path/to/my_plc.json
```

Without `config_file`, the `config/params.json` installed with the package is used. The node can
also be started with `ros2 run beckhoff_ros beckhoff_ros_node --ros-args -p config_file:=...`.

### ROS interface

- `variable_state` (`beckhoff_interfaces/msg/PlcVariable`): published with the initial value of
  each monitored variable and every time one of them changes. `type` and `value` are formatted
  like the Python version did (Python's `str(type(value))` and `str(value)`):

  ```
  name: GVL.obj2
  type: <class 'dict'>
  value: "{'attr1': 7, 'attr2': True}"
  ```

- `write_variable` (`beckhoff_interfaces/srv/WritePlcVariable`): writes a variable listed in
  `variables_to_write`; `result` is `true` on success. `type` tells how `value` is read:

  | `type`                    | `value`                                        | Example                         |
  |---------------------------|------------------------------------------------|---------------------------------|
  | `bool`                    | an integer (0 is false), `true` or `false`     | `1`                             |
  | `int`                     | an integer                                     | `-5`                            |
  | `float`                   | a number                                       | `1.5`                           |
  | `dict`                    | a structure, as a Python literal or JSON       | `{'attr1': 5, 'attr2': True}`   |
  | `list`, `tuple`           | an array, as a Python literal or JSON          | `[1, 2, 3]`                     |

  The type strings of the published messages (e.g. `<class 'dict'>`) are accepted as well. Every
  field of a structure must be given; arrays can be given with fewer items (the others are set to
  zero).

  ```bash
  ros2 service call /write_variable beckhoff_interfaces/srv/WritePlcVariable \
    "{variable: {name: 'GVL.var1', type: 'bool', value: '1'}}"
  ```

`beckhoff_interfaces` is expected to define:

```
# msg/PlcVariable.msg
string name
string type
string value

# srv/WritePlcVariable.srv
PlcVariable variable
---
bool result
```

### C++ API

`BeckhoffPLC` does not depend on ROS and can be used by other packages
(`find_package(beckhoff_ros)`, `target_link_libraries(<target> beckhoff_ros::beckhoff_ros)`):

```cpp
#include <fstream>
#include <iostream>
#include "beckhoff_ros/beckhoff_plc.hpp"

using beckhoff_ros::Value;

std::ifstream file("params.json");
beckhoff_ros::BeckhoffPLC plc(nlohmann::ordered_json::parse(file)["PLC"]);

plc.addObserver([](const std::string & name, const Value & value) {
    std::cout << name << " " << value.str() << std::endl;
  });

plc.setVariable("GVL.var1", true);
plc.setVariable("GVL.obj1", Value::list({Value::dict({
    {"attr1", 1},
    {"attr2", Value::dict({{"attr1", 2}, {"attr2", true}})},
    {"attr3", Value::list({true, false})}})}));

const Value obj2 = plc.getVariable("GVL.obj2");
std::uint64_t attr1 = obj2["attr1"].asUInt();

// read, change some fields, write back
Value obj1 = plc.getVariable("GVL.obj1");
obj1[3]["attr2"]["attr1"] = 42;
obj1[3]["attr3"][0] = true;
plc.setVariable("GVL.obj1", obj1);

// from a Python literal or JSON text
plc.setVariable("GVL.obj2", Value::parseLiteral("{'attr1': 5, 'attr2': True}"));
```

Structures are `Value` dicts, arrays are lists and the other types are bool, int or float.
Every field of a structure must be given (in any order); arrays can have fewer items (the others
are set to zero). `operator[]` only accesses existing items, so a misspelled field name throws.
`setVariable` throws `ValueError` (with the path of the wrong item) before anything is written if
a value does not fit its type.

## Tests

`colcon test --packages-select beckhoff_ros` runs unit tests and tests the node against
`test/ads_simulator.py`, a minimal TwinCAT ADS server that listens on 127.0.0.1:48898. The
simulator can also be used to try the node without a PLC:

```bash
python3 test/ads_simulator.py my_plc.json --host 127.0.0.1 --value "GVL.var1=True"
```

(with `"net_id": "127.0.0.1.1.1"` in the configuration given to the node).

## Differences with the Python version

- The configuration file is given with the `config_file` parameter (launch argument) instead of
  the hard coded `INSERT_PATH`. The ROS parameters that were declared but not used, and the
  untested `params.yaml`, were removed: ROS parameters cannot describe nested structures.
- Fixed: the node crashed at startup when variables were monitored (the publisher was created
  after the initial values were notified).
- Fixed: changes of monitored variables of elementary types were published as `{}`, and arrays of
  elementary types were read as `[{}, {}, ...]`.
- Fixed: `result` of `write_variable` is set (the Python version assigned `None`).
- `write_variable` can also write structures and arrays (`dict`, `list` and `tuple` types).
- Values that do not fit their type are rejected: out of range integers were silently truncated by
  ctypes, and invalid values could lead to partial writes.
- Requests to the PLC are serialized: the ADS library only supports one pending request at a time
  and the node handles service calls on several threads.
- More elementary types, and the `pack` option.
- Made for ROS 2 Jazzy and Kilted (the Python version targeted Foxy).

Like the Python version, the connection is not re-established if it is lost (the node has to be
restarted) and routes are not created automatically.
