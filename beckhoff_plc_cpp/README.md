# beckhoff_plc (C++)

C++ version of `beckhoff_ros/beckhoff_plc.py`: communication with a Beckhoff
PLC over ADS, with custom structures (DUTs) loaded at runtime from the **same
JSON parameter file** as the Python version. It is a plain CMake library with no
ROS dependency, built on [Beckhoff's ADS library](https://github.com/Beckhoff/ADS)
(the one pyads uses on Linux).

- Any PLC type: elementary types, `STRING(n)`/`WSTRING(n)`, arrays (also
  multi-dimensional), nested structures, aliases/enums, TwinCAT pack modes.
- A simple API to create and edit values of custom structures, plus direct
  access with plain C++ structs and JSON in the Python dict/list form.
- Lower latency than the Python version (see [Performance](#performance)): ADS
  handles cached for every variable, sum commands to read/write several
  variables in one request, no allocation on the fast paths.
- Checks the configured layouts against the PLC symbols when connecting, renews
  handles after an online change, `reconnect()` after a PLC restart, thread-safe.

## Usage

```cpp
#include <beckhoff_plc/beckhoff_plc.hpp>

// Same parameters as the Python version (config/params.json)
beckhoff::BeckhoffPLC plc = beckhoff::BeckhoffPLC::fromFile("config/params.json");

// Python: plc.addObserver(fn). Called with the current values of
// "variables_to_monitor", then whenever they change.
plc.addObserver([](const beckhoff::Notification& n) {
  std::cout << n.name << " = " << n.value << "\n";  // {"attr1":5,"attr2":true}
});

// Create a value of a custom structure (zero-initialized) and edit it
beckhoff::Value obj = plc.createValue("CustomStruct2_s");
obj["attr1"] = 42;
obj["attr2"]["attr2"] = true;  // nested structure
obj["attr3"] = {true, false};  // array

beckhoff::Value list = plc.variable("GVL.obj1").createValue();  // ARRAY[0..9] OF CustomStruct2_s
list[0] = obj;
plc.setVariable("GVL.obj1", list);

// The Python way: dicts and lists as JSON
plc.setVariable("GVL.var1", true);
plc.setVariable("GVL.obj2", {{"attr1", 5}, {"attr2", true}});  // if GVL.obj2 is writable
beckhoff::Value value = plc.createValue("CustomStruct1_s", {{"attr1", 5}, {"attr2", true}});

// Reading
beckhoff::Value current = plc.getVariable("GVL.obj1");
uint16_t a = current[0]["attr1"].as<uint16_t>();
bool b = plc.getVariable<bool>("GVL.var1");
std::vector<int> v = plc.getVariable<std::vector<int>>("GVL.var2");
beckhoff::Json j = current.toJson();  // what the Python version returns
```

Values convert safely: writing `70000` into a `UINT` or reading a `REAL` holding
`2.5` as an `int` throws `beckhoff::RangeError` instead of wrapping around or
truncating. Errors derive from `beckhoff::Error` (`ConfigError`, `KeyError`,
`TypeError`, `RangeError`, `AdsError` with the ADS code).

### Plain C++ structs

A C++ struct with the same memory layout as the PLC structure is read and
written byte by byte (the size is checked):

```cpp
struct CustomStruct1 {  // TwinCAT 3 default alignment = C++ default alignment
  uint16_t attr1;
  bool attr2;
};
auto s = plc.getVariable<CustomStruct1>("GVL.obj2");
```

### Real-time loops

Look things up once, then reuse them: no name lookup, no allocation, one ADS
round trip per call.

```cpp
beckhoff::Variable& cmd = plc.variable("GVL.obj1");
beckhoff::Value cmdValue = cmd.createValue();
beckhoff::ValueRef speed = cmdValue[0]["attr1"];  // bound once

std::vector<beckhoff::Value> state;  // reused buffers
while (running) {
  speed = 10;
  cmd.write(cmdValue);
  plc.getVariables({"GVL.obj2", "GVL.var1", "GVL.var2"}, state);  // 1 request for 3 variables
}
```

`plc.subscribe("GVL.var1", callback)` delivers the notifications of a single
variable; `removeObserver(id)` removes an observer or subscription.

## Parameter file

The Python format is used as is (`config/params.json`); the parameters may be
the whole file or the `"PLC"` object. Struct members keep the file order
(that is their memory order). Optional keys only read by the C++ version:

| Key | Default | |
|---|---|---|
| `net_id` | required | AMS Net ID of the PLC |
| `client_net_id` | `<local IP>.1.1` | AMS Net ID of this computer |
| `username`, `password` | | for `create_route` / `BeckhoffPLC::createRoute()` |
| `structures` | | `{"Name": {"field": type, ...}}`, or a list `[["field", type], "field : TYPE;"]` |
| `variables` | | `{"GVL.var": type}` |
| `variables_to_monitor` | `[]` | names, or `{"name": ..., "mode": ..., "cycle_time_ms": ..., "max_delay_ms": ...}` |
| `variables_to_write` | `[]` | writing other variables throws `KeyError` (like Python) |
| `ip_address` | from `net_id` | PLC address (IP or host name) |
| `port` | `851` | ADS port (TwinCAT 3 PLC1; TwinCAT 2: `801`) |
| `timeout_ms` | `5000` | ADS request timeout |
| `pack_mode` | `8` | structure alignment, as `{attribute 'pack_mode'}` (TwinCAT 3 default: 8) |
| `pack_modes` | | per structure: `{"ST_Packed": 1}` |
| `aliases` | | `{"E_State": "INT", "T_Name": "STRING(50)"}` |
| `notification` | on change, 0.1 µs, 0.1 µs (pyads) | `{"mode": "on_change" \| "cyclic", "cycle_time_ms": 10, "max_delay_ms": 0}` |
| `verify_sizes` | `true` | compare each variable's size with the PLC symbol when connecting |
| `create_route` | `false` | add the route on the PLC when connecting (`client_ip_address`, `route_name`) |

Types: the Python names (`bool`, `int`, `int16`, `uint16`, `uint8`, `real`),
the TwinCAT ones (`BOOL`, `SINT`..`ULINT`, `BYTE`..`LWORD`, `REAL`, `LREAL`,
`TIME`, `STRING(20)`, `WSTRING`, ...) and C-like ones (`int32`, `float`, `double`);
`int` is 16 bits, as in TwinCAT and the Python version. Arrays are written
`["type", size]` (Python), `["type", rows, columns]`, or
`"ARRAY[0..9] OF ST_Foo"` / `"ARRAY[1..2, 1..3] OF INT"` (TwinCAT). Indices
are zero-based in C++, whatever the declared lower bound. Structures may
reference each other in any order. Names are case-insensitive, like in TwinCAT.

The configuration can also be filled in code (`beckhoff::Config`), e.g. from
ROS parameters.

## From Python to C++

| Python | C++ |
|---|---|
| `BeckhoffPLC(params)` | `BeckhoffPLC(params)` (`beckhoff::Json`), `BeckhoffPLC(config)`, `BeckhoffPLC::fromFile(path)` |
| `addObserver(fn)` | `addObserver(callback)`; also `subscribe(name, callback)`, `removeObserver(id)` |
| `setVariable(name, value)` | `setVariable(name, value)`: Value, number, string, vector, JSON, C++ struct |
| `getVariable(name)` | `getVariable(name)`, `getVariable<T>(name)`, `getVariable(name, value)` |
| | `getVariables(names)`, `setVariables({...})`: one ADS request |
| `readInitialValues()` | `readInitialValues()` |
| `createRoute()` | `BeckhoffPLC::createRoute(config)` or `"create_route": true` |
| `netId2ip(netId)` | `BeckhoffPLC::netId2ip(netId)` |
| `instantiateCStruct(dict, type)` | `plc.createValue("Type", json)` |

Differences:
- `addObserver()` delivers the current values to the new observer only
  (Python sends them to every observer); `readInitialValues()` sends them to all.
- Every variable gets a handle, so `getVariable()` is one ADS round trip
  (Python: get handle, read, release handle).
- Missing structure members or array elements in JSON values are errors
  (`Value::update()` changes only the given members).
- Two issues of the Python version (with pyads 3.6) do not exist here: reading
  an array of elementary types (`GVL.var2`) returns `[{}, {}, {}, {}, {}]`, and
  notifications of elementary variables (`GVL.var1`) deliver `{}`.

## Threads

All `BeckhoffPLC` functions are thread-safe. Requests are serialized: the ADS
library handles one pending request per connection, so with the Python version
concurrent calls fail (`ADSError: Unknown Error (-1)`: 684 of 1200 writes from
4 threads in a test), e.g. in a ROS node with a multi-threaded executor. Callbacks run in the ADS notification
thread, one at a time; keep them short. They may call `getVariable()` /
`setVariable()`, but not destroy the `BeckhoffPLC` or call `reconnect()`.

## Building

Linux (or another POSIX system); CMake >= 3.18 and a C++17 compiler. The ADS
library (pinned commit) and, when not installed, nlohmann_json (`apt install
nlohmann-json3-dev`) and GoogleTest are downloaded by CMake.

```sh
cd beckhoff_plc_cpp
cmake -B build && cmake --build build
ctest --test-dir build       # unit tests + end-to-end tests against a fake PLC
cmake --install build --prefix /usr/local
```

Then `find_package(beckhoff_plc)` and link `beckhoff_plc::beckhoff_plc`.
Offline builds: `-DFETCHCONTENT_SOURCE_DIR_BECKHOFF_ADS=/path/to/ADS` (a
checkout of https://github.com/Beckhoff/ADS), or
`-DBECKHOFF_PLC_USE_SYSTEM_ADS=ON` for an installed ADS library.

### In a ROS 2 workspace

colcon does not look for packages inside another package, so with this
repository in `src/beckhoff_ros`, expose the library with a symlink (or
`colcon build --paths src/beckhoff_ros src/beckhoff_ros/beckhoff_plc_cpp ...`):

```sh
ln -s beckhoff_ros/beckhoff_plc_cpp src/beckhoff_plc
colcon build
```

In the node's package: `<depend>beckhoff_plc</depend>` in package.xml and

```cmake
find_package(beckhoff_plc REQUIRED)
target_link_libraries(my_node beckhoff_plc::beckhoff_plc)
```

```cpp
// in the node
const auto file = declare_parameter<std::string>("config_file");
plc_ = std::make_unique<beckhoff::BeckhoffPLC>(beckhoff::Config::fromFile(file));
plc_->addObserver([this](const beckhoff::Notification& n) {
  beckhoff_interfaces::msg::PlcVariable msg;
  msg.name = n.name;
  msg.type = n.value.type().name();
  msg.value = n.value.toString();  // JSON
  publisher_->publish(msg);
});
```

## Performance

Median latency of each operation against the same local fake PLC, three runs
of 2000 iterations (`benchmark/compare.py`; Python: the unchanged
`beckhoff_plc.py` with pyads 3.6, Python 3.13; 4 vCPU Xeon VM). Both go
through Beckhoff's ADS library, so a request costs the same ~75 µs here (the
loopback round trip); the difference is the number of requests and the work
done around them. Over a real network each saved round trip is worth more.

| Operation | Python | C++ | |
|---|---:|---:|---:|
| getVariable (struct) | 283 µs | 78 µs | 3.6x |
| getVariable (10 structs) | 308 µs | 75 µs | 4.1x |
| setVariable (bool) | 89 µs | 74 µs | 1.2x |
| setVariable (10 structs, dicts) | 141 µs | 74 µs | 1.9x |
| read 3 variables (C++: `getVariables`) | 890 µs | 77 µs | 11.5x |
| write + notification received | 223 µs | 130 µs | 1.7x |
| dicts to bytes (no I/O) | 38 µs | 1.4 µs | 27x |
| bytes to dicts (no I/O) | 21 µs | 6.8 µs | 3.1x |
| set 10 fields of a Value (no I/O) | | 0.06 µs | |

```sh
cmake -B build -DBECKHOFF_PLC_BUILD_BENCHMARKS=ON && cmake --build build
python3 benchmark/compare.py --build build --python <python with pyads>
```

## Tests

`ctest` runs unit tests (types and layouts, checked against the compiler's
`#pragma pack` layouts; values; parameters) and end-to-end tests where the
real ADS library talks to `test/fake_ads_server`, a small ADS server that
emulates a TwinCAT runtime (handles, symbol information, sum commands,
notifications, online changes, restarts). The fake server also runs on its
own, to try the library or the Python version without a PLC:

```sh
./build/test/fake_ads_server --config ../config/params.json   # then use net_id 127.0.0.1.1.1
```
