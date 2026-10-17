# beckhoff_ros

Beckhoff ADS + ROS2 (C++)

The BeckhoffPLC class communicates with a Beckhoff PLC (TwinCAT 3) through ADS, using any data
type: the custom DUTs (structures) of the PLC program, arrays of them, structures containing arrays
and other structures, etc. \
Users must specify the custom DUTs and the variables in a JSON file. Everything is loaded at
runtime, so the same package can be used with any PLC project without code changes.

This is the C++ port of the Python version (branch `main`). It uses the same configuration file,
topic and service, and is made for control loops: see [Performance](#performance).

## Build

Requirements: ROS 2 Jazzy or Kilted (Ubuntu 24.04) and the `beckhoff_interfaces` package (see
[ROS interface](#ros-interface)), in the same workspace. The ADS client is part of this package:
there is nothing else to install.

```bash
cd ~/ros2_ws/src
git clone -b cpp https://github.com/RGoulartns/beckhoff_ros.git
# + beckhoff_interfaces
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build
```

The package is built in `Release` mode unless another `CMAKE_BUILD_TYPE` is given.

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
- `structures`: the DUTs, with their fields in the same order as in the PLC program. A structure
  can only use the structures defined before it.
- `variables`: the PLC variables used, with their type. Any symbol can be used, e.g.
  `"GVL.robot.var2[3]": "MyStruct1"` for a part of a structure.
- `variables_to_monitor`: variables whose changes are sent by the PLC (ADS notifications): they
  are published by the node, and their latest value is always available without a request
  (`BeckhoffPLC::latest`). An entry can also set its notification, see `notification`:
  `{"name": "GVL.status", "mode": "cyclic", "cycle_time_ms": 1}`.
- `variables_to_write`: variables that can be written by the ROS service.
- `client_net_id`, `username`, `password`: only used by `BeckhoffPLC::createRoute()`, which adds
  a route to this computer on the PLC. The PLC needs that route, which can also be added with
  TwinCAT.

Optional parameters:

| Parameter               | Default            | Description                                                                 |
|-------------------------|--------------------|-----------------------------------------------------------------------------|
| `pack`                  | `0`                | structure packing: `0` is TwinCAT 3's default layout, use `1` for `{attribute 'pack_mode' := '1'}` |
| `notification`          | on change          | default notification of the monitored variables: `{"mode": "on_change" or "cyclic", "cycle_time_ms": ..., "max_delay_ms": ...}`. The PLC checks the value (on change) or sends it (cyclic) every `cycle_time_ms`, by default every task cycle |
| `timeout_ms`            | `1000`             | timeout of the requests                                                     |
| `local_net_id`          | IP address + `.1.1` | AMS Net Id of this computer: the PLC must have a route to it               |
| `host`, `tcp_port`      | from `net_id`, `48898` | address of the PLC, if it is not the one of its Net Id (e.g. a tunnel)  |
| `ams_port`              | `851`              | AMS port of the PLC runtime (`852` for the second one...)                   |
| `reconnect`             | `true`             | re-establish the connection when it is lost                                 |
| `reconnect_interval_ms` | `1000`             | between two connection attempts                                             |
| `health_check_ms`       | `1000`             | period of the check of the PLC program (see [Connection](#connection)), `0` to disable |
| `check_sizes`           | `true`             | compare the sizes of the variables with the PLC at startup                  |
| `receive_thread`        |                    | `{"priority": 80, "cpu": 3}`: SCHED_FIFO priority and CPU of the thread receiving the responses (see [Real-time](#real-time)) |

A type is a type name or an array given as `[type name, size]`. Type names are the structures and:

| Type names                | PLC type      | Size (bytes) | C++ type        |
|---------------------------|---------------|--------------|-----------------|
| `bool`                    | BOOL          | 1            | `bool`          |
| `int8`, `sint`            | SINT          | 1            | `std::int8_t`   |
| `uint8`, `usint`, `byte`  | USINT, BYTE   | 1            | `std::uint8_t`  |
| `int`, `int16`            | INT           | 2            | `std::int16_t`  |
| `uint16`, `uint`, `word`  | UINT, WORD    | 2            | `std::uint16_t` |
| `int32`, `dint`           | DINT          | 4            | `std::int32_t`  |
| `uint32`, `udint`, `dword`| UDINT, DWORD  | 4            | `std::uint32_t` |
| `int64`, `lint`           | LINT          | 8            | `std::int64_t`  |
| `uint64`, `ulint`, `lword`| ULINT, LWORD  | 8            | `std::uint64_t` |
| `real`, `float`           | REAL          | 4            | `float`         |
| `lreal`, `double`         | LREAL         | 8            | `double`        |

More names can be added to the table at the top of `src/plc_types.cpp`.

When the BeckhoffPLC starts, the size of each variable is compared with the one the PLC reports:
if a structure does not match its PLC declaration (missing field, wrong type or order, packing),
the error tells which variables differ.

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

The node logs the published values at the debug level
(`--ros-args --log-level beckhoff_ros_node:=debug`).

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

## C++ API

`BeckhoffPLC` does not depend on ROS and can be used by other packages
(`find_package(beckhoff_ros)`, `target_link_libraries(<target> beckhoff_ros::beckhoff_ros)`).
It offers two ways to access the variables:

- Values: structures are `Value` dicts, arrays are lists, the other types are bool, int or
  float. Convenient, but each access converts the data and allocates memory.
- Buffers: the data is exchanged as it is in the PLC, its items are reached with views and typed
  fields. Nothing is converted or allocated once the buffers exist: this is the API for control
  loops.

### Values

```cpp
#include <fstream>
#include <iostream>
#include "beckhoff_ros/beckhoff_plc.hpp"

using beckhoff_ros::Value;

std::ifstream file("params.json");
beckhoff_ros::BeckhoffPLC plc(nlohmann::ordered_json::parse(file)["PLC"]);

// called with the current value of each monitored variable, then with every change
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

Every field of a structure must be given (in any order); arrays can have fewer items (the others
are set to zero). `operator[]` only accesses existing items, so a misspelled field name throws.
`setVariable` throws `ValueError` (with the path of the wrong item) before anything is written if
a value does not fit its type, `AdsError` if the PLC returns an error.

### Buffers

```cpp
using beckhoff_ros::Buffer;
using beckhoff_ros::Field;

const beckhoff_ros::Variable & command = plc.variable("GVL.robot_command");  // a RobotStruct
const beckhoff_ros::Variable & status = plc.variable("GVL.robot_status");

// the data of the variables (zero initialized)
Buffer command_data = command.buffer();
Buffer status_data = status.buffer();

// items, by name and index, like Values (each [] looks the field up)
command_data["var1"] = true;
command_data["var2"][3]["attr1"] = 42;
command_data.at("var3.points[1].x") = 0.5f;
command_data["var2"][0] = Value::dict({{"attr1", 7}, {"attr2", true}});  // from a Value
float x = status_data["var3"]["points"][1]["x"].as<float>();

// typed fields, resolved once: a copy at a fixed offset
const Field<std::uint16_t> attr1(command.type(), "var2[3].attr1");
const Field<float> y(status.type(), "var3.points[1].y");
attr1.set(command_data, 42);
float value = y.get(status_data);

plc.write(command, command_data);  // one request
plc.read(status, status_data);     // one request
```

Assignments are checked like `setVariable`: a `ValueError` is thrown, and the data is not changed,
if the value does not fit (e.g. 70000 for a UINT, 1.5 for an INT). A `Field<T>` checks that the
item is of the C++ type `T` (see the table of types) when it is created. Buffers convert to and
from Values with `value()` and `setValue()`.

A `Batch` executes several writes then several reads with a single round trip to the PLC. Build
it once; executing it does not allocate memory:

```cpp
beckhoff_ros::Batch cycle;
cycle.write(command, command_data).read(status, status_data).read(other, other_data);

while (running) {
  attr1.set(command_data, setpoint);
  plc.execute(cycle);  // sends the commands, reads the status (after the writes)
  use(y.get(status_data));
}
```

Every request has an asynchronous version that returns once the request is sent:

```cpp
beckhoff_ros::Pending write = plc.writeAsync(command, command_data);  // the data is copied
beckhoff_ros::Pending batch = plc.executeAsync(cycle);
// ... do something else ...
write.wait();  // throws AdsError if the request failed
batch.wait();  // the data read is copied to the buffers here

plc.writeAsync(command, command_data);  // not waited for: errors are logged
```

The monitored variables are sent by the PLC when they change (or periodically), so their latest
value is available without any request:

```cpp
plc.latest(status, status_data);  // false if no value was received yet

// wait for a new value: with a cyclic notification, a loop that runs at the PLC rate
std::uint64_t sequence = 0;
while (plc.waitForUpdate(status, status_data, sequence, std::chrono::milliseconds(100))) {
  ...
}

// or a callback, called by the thread receiving the notifications
plc.onUpdate(status, [&](const beckhoff_ros::Variable &, const beckhoff_ros::ConstView & data,
  beckhoff_ros::Timestamp time) {
    float y_value = y.get(data);
  });
```

All the functions can be called from several threads; requests of different threads are sent
without waiting for each other. Callbacks of `onUpdate` must return quickly and must not wait for
requests (they would wait for the thread running them); observers run on another internal thread.

### Real-time

- In a control loop, use buffers and fields created beforehand: `read`, `write`, `execute`, the
  asynchronous versions, `latest`, `waitForUpdate`, field and view accesses do not allocate
  memory (`test_beckhoff_plc.cpp` checks it). The Value API, observers, error paths and the
  setup after a reconnection allocate.
- Give the thread receiving the responses and notifications a real-time priority, so that a busy
  computer does not delay them: `"receive_thread": {"priority": 80, "cpu": 3}` in the
  configuration. This needs the permission to use real-time priorities, e.g. a line
  `<user> - rtprio 99` in `/etc/security/limits.conf` (log in again) or `CAP_SYS_NICE`;
  otherwise a warning is logged and the default priority is kept. The loop itself should run in a
  real-time thread as well.

### Connection

The connection is checked continuously:

- when it is lost (PLC reboot, network failure), the requests in flight fail with `AdsError`
  (code `ads_error::kNotConnected`), and it is re-established in the background;
- when the PLC program changes (new download) or the runtime restarts, the symbol handles and the
  notifications are lost: they are created again (detected by the requests, and by
  `health_check_ms`).

Requests fail with `AdsError` while this happens; `ready()` tells if they can be sent. Several
BeckhoffPLC objects of a process share the connection to a PLC (TwinCAT accepts a single
connection per route).

## Performance

A request to the PLC costs a round trip: the network time, plus the time for the PLC to process
it, which is in step with its task cycle (so typically up to a PLC cycle). The number of round
trips is what matters:

| Operation                                     | Previous versions (and pyads) | Now                       |
|-----------------------------------------------|-------------------------------|---------------------------|
| read a variable                               | 3 (get handle, read, release) | 1                         |
| write a variable                              | 1                             | 1                         |
| write a variable and read another one back    | 4                             | 1 with a `Batch`          |
| read or write N variables                     | N or 3N                       | 1 (ADS sum command)       |
| read a monitored variable                     | 3                             | 0 (`latest`)              |

Use the benchmark tool to measure them with your PLC (it reads the variables given with `--read`,
and writes back the current value of the variables given with `--write`):

```bash
ros2 run beckhoff_ros benchmark my_plc.json --read GVL.status --write GVL.command --count 1000
```

With the simulator below and a PLC cycle of 1 ms (`--cycle-ms 1`, `config/params.json`):

```
1000 runs per test, times in ms
                                                                   min     mean   median      p99
read GVL.obj2                                                    0.717    1.000    0.998    1.150
write GVL.var1                                                   0.559    1.013    1.001    1.374
batch: 2 writes + 2 reads (1 round trip)                         0.267    1.015    1.000    1.818
one after the other (4 round trips)                              3.336    4.039    4.000    5.038
latest GVL.obj2 (no request)                                     0.000    0.000    0.000    0.000
previous version: read GVL.obj2 (3 round trips)                  2.738    3.010    2.999    3.217
previous version: write GVL.var1 + read GVL.obj2 (4 round trips) 3.555    4.036    4.000    5.055
```

Against a server answering at once on the loopback interface, a round trip takes about 0.08 ms
(median): this is what the client and the operating system add to the time of the PLC.

## Tests

`colcon test --packages-select beckhoff_ros` runs unit tests, tests the ADS client against a
scripted server, and tests BeckhoffPLC and the node against `test/ads_simulator.py`, a TwinCAT ADS
server simulator. The simulator can also be used to try the node or a program without a PLC:

```bash
ros2 run beckhoff_ros ads_simulator.py my_plc.json --host 127.0.0.1 --value "GVL.var1=True"
```

(with `"net_id": "127.0.0.1.1.1"` in the configuration given to the node). With `--cycle-ms 1`,
requests are processed at the end of a 1 ms PLC cycle, like TwinCAT does.

## Differences with the Python version

- Requests use symbol handles created at startup, sum commands and pipelining (see
  [Performance](#performance)); the Buffer API gives access to the data without conversions.
- The connection is re-established when it is lost, and the handles and notifications are
  created again when the PLC program changes. The sizes of the variables are checked at startup.
- The configuration file is given with the `config_file` parameter (launch argument) instead of
  the hard coded `INSERT_PATH`. The ROS parameters that were declared but not used, and the
  untested `params.yaml`, were removed: ROS parameters cannot describe nested structures.
- Fixed: the node crashed at startup when variables were monitored (the publisher was created
  after the initial values were notified).
- Fixed: changes of monitored variables of elementary types were published as `{}`, and arrays of
  elementary types were read as `[{}, {}, ...]`.
- Fixed: `result` of `write_variable` is set (the Python version assigned `None`).
- `write_variable` can also write structures and arrays (`dict`, `list` and `tuple` types).
  `variables_to_write` limits what the service can write; the C++ API can write any variable.
- Values that do not fit their type are rejected: out of range integers were silently truncated by
  ctypes, and invalid values could lead to partial writes.
- More elementary types, the `pack` option, cyclic notifications.
- Made for ROS 2 Jazzy and Kilted (the Python version targeted Foxy).

Like the Python version, routes are not created automatically (see `createRoute()`).
