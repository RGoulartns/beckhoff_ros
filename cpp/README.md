# Standalone BeckhoffPLC for C++

C++17 ADS client with runtime JSON structures, nested arrays, cached symbol
handles and on-change observers. It has no ROS or Python runtime dependency.
The existing Python ROS package stays available alongside this library.

The native transport includes Beckhoff's `AdsLib.h` and calls its C/C++ API
directly: `AdsSyncReadWriteReqEx2` for symbol handles, `AdsSyncReadReqEx2` and
`AdsSyncWriteReqEx` for I/O, and the native notification API for subscriptions.
The call path is `BeckhoffPLC -> AdsTransport -> Beckhoff AdsLib -> ADS/TCP`.
The type registry computes layouts and serializes values in C++.

## Build and test

Requires a C++17 compiler, CMake 3.24+, threads and Internet access on the first
configure. CMake downloads pinned, SHA-256-checked nlohmann/json 3.11.3 and
Beckhoff AdsLib sources. Existing `nlohmann_json` CMake packages are supported.

```sh
cmake -S cpp -B build-cpp -DCMAKE_BUILD_TYPE=Release
cmake --build build-cpp -j4
ctest --test-dir build-cpp --output-on-failure
```

Default tests exercise codecs, client behavior and the ADS API boundary without
hardware. To additionally exercise real AdsLib TCP traffic, configure with a
Python 3 interpreter for the independent, standard-library-only ADS test peer:

```sh
cmake -S cpp -B build-cpp -DCMAKE_BUILD_TYPE=Release \
  -DBECKHOFF_PLC_ADS_INTEGRATION_TEST=ON \
  -DPython3_EXECUTABLE="$(command -v python)"
cmake --build build-cpp -j4
ctest --test-dir build-cpp --output-on-failure
```

The integration test uses localhost TCP port 48898. It verifies reads, writes,
wire notifications, shared routes and resource cleanup. Python is only a test
helper dependency for this optional suite; it requires no third-party packages.
The C++ build and all integration tests work without pyads installed.
`-DBECKHOFF_PLC_WITH_ADS=OFF` builds codecs and injectable client
support without AdsLib; a real connection then requires an injected `Transport`.

## Complex structure validation

[examples/test_struct3.json](examples/test_struct3.json) contains the complete
`TestStruct1` / `TestStruct2` / `TestStruct3` configuration. Its network addresses
point to the localhost test peer; replace them for a real PLC. The schema is:

```json
{
  "TestStruct1": {"attrbt1": "bool", "attrbt2": ["bool", 6]},
  "TestStruct2": {"attr1": ["TestStruct1", 10], "attr2": "uint16"},
  "TestStruct3": {
    "attr1": "bool", "attr2": "uint16",
    "attr3": ["TestStruct2", 6], "attr4": "TestStruct1"
  }
}
```

Default layout on x86-64:

| Type | Bytes | Member offsets |
| --- | ---: | --- |
| TestStruct1 | 7 | `attrbt1=0`, `attrbt2=1` |
| TestStruct2 | 72 | `attr1=0`, `attr2=70` |
| TestStruct3 | 444 | `attr1=0`, `attr2=2`, `attr3=4`, `attr4=436` |

Tests exercise all 60 nested leaves and the trailing `TestStruct1`, with unique
boolean patterns and distinct `uint16` values for each group. Expected bytes
are computed independently of the codec. Checks include every field/byte,
padding, native and packed layouts (`TestStruct3` is 442 bytes with `pack=1`),
nested builders, invalid shapes/ranges, mocked I/O and observers. The native ADS
integration test loads the JSON parameter file, exchanges the full 444-byte
payload over TCP and verifies notifications and cleanup. This is a local protocol
test; it does not establish behavior or timing on a physical PLC.

For example, after loading this configuration, a nested value can be built as:

```cpp
auto value = plc->makeValue("TestStruct3"); // All fields initially zero/false.
value.set("/attr2", 42);
value.set("/attr3/5/attr1/9/attrbt2/5", true);
value.set("/attr4/attrbt1", true);
plc->setVariable("GVL.complex", value);
```

## JSON configuration and layouts

`BeckhoffPLC::fromFile("config/params.json")` accepts the repository's existing
JSON document with its outer `PLC` key. The constructor also accepts the inner
object. Parse with **`beckhoff::Json` (`nlohmann::ordered_json`)**: declaration
order determines struct layout. Converting a sorted JSON map to ordered JSON
after parsing cannot recover the original field order.

```json
{
  "PLC": {
    "net_id": "192.168.1.10.1.1",
    "ip_address": "192.168.1.10",
    "client_net_id": "192.168.1.20.1.1",
    "port": 851,
    "timeout_ms": 1000,
    "notification_cycle_us": 1000,
    "notification_max_delay_us": 0,
    "pack": 0,
    "structures": {
      "Status": {"count": "uint16", "enabled": "bool"},
      "Command": {"status": "Status", "targets": ["real", 3]}
    },
    "variables": {"GVL.command": "Command", "GVL.status": "Status"},
    "variables_to_monitor": ["GVL.status"],
    "variables_to_write": ["GVL.command"]
  }
}
```

Types are names or `[type, count]`; nested array specs such as
`[["uint16", 3], 2]` also work. Forward structure references are allowed; recursive
types, unknown names, zero-length arrays, empty structs and oversized types fail
before symbol acquisition. The maximum type size is 16 MiB and nesting depth is
bounded. The ADS backend imposes a smaller notification limit to fit AdsLib's
4 MiB dispatcher ring; high traffic can still exhaust that shared ring.

Builtins: `bool`, `int`/`int16`, `uint16`, `uint8`, `real` (float32), plus `int8`,
`int32`, `uint32`, `int64`, `uint64`, `double` (float64). Aliases `float`,
`float32`, `float64` are accepted. PLC STRING/WSTRING and variable-length types
are not implemented.

Default `pack: 0` matches native Python `ctypes` alignment on the build platform.
`pack: 1`, `2`, `4`, `8` or `16` caps alignment for every custom struct. Match the
actual TwinCAT DUT packing; native C++ layout alone does not establish a match.
Integers and IEEE-754 floating point bytes are little endian, with zero padding
when constructing or assigning a value. In the original sample, `CustomStruct1_s`
is 4 bytes, `CustomStruct2_s` is 8 bytes and `GVL.obj1` is 80 bytes with default
packing on x86-64. Type metadata exposes field offsets, alignment and size.

`ip_address` is the PLC/gateway IP or hostname and can differ from its AMS Net ID.
If omitted, the first four Net ID octets are used for Python compatibility.
The default ADS port is 851. All AdsLib clients in one process share the same
local AMS Net ID; explicitly conflicting client IDs/routes are rejected.

Configure a **return ADS route on the PLC** for `client_net_id` and the client's
reachable IP before connecting. Cloud execution does not imply access to a PLC
on your LAN. Legacy `username`/`password` keys are ignored: the client does not
automatically change PLC routes or send administrator credentials. Configure
routes using TwinCAT/Beckhoff tools separately. Symbols are resolved once at
construction; recreate the client after a PLC download invalidates handles.

## Values and observers

```cpp
#include <beckhoff/beckhoff_plc.hpp>

auto plc = beckhoff::BeckhoffPLC::fromFile("params.json");

// A zero-initialized custom value: edit fields with JSON pointers.
auto command = plc->makeValue("Command");
command.set("/status/count", 42);
command.set("/status/enabled", true);
command.set("/targets/0", 1.5);
plc->setVariable("GVL.command", command);

// Or assign the full JSON object; missing/extra fields and range errors throw.
auto value = plc->getVariable("GVL.status");
auto observer = plc->addObserver([](const std::string& name,
                                   const beckhoff::Json& value) {
    // Hand off to your application (e.g. a queue read by a ROS executor).
});
plc->removeObserver(observer); // Waits until active invocations finish.
```

`makeVariable(name)` constructs a value for a configured variable, including
arrays. `assign(json)` requires all fields and an exact array length; `set(path,
json)` updates one subtree and leaves other fields at their existing values.
Paths follow RFC 6901 (`/array/0/field`, `~0` for `~`, `~1` for `/`). Failed
assignments leave the old value intact. Boolean input must be a JSON boolean;
out-of-range numbers are rejected instead of wrapping. This is deliberately
stricter than Python's implicit ctypes conversions.

`addObserver` and `addRawObserver` return removal IDs. Registration reads initial
monitored values by default; pass `false` to skip that read. Initial delivery
runs on the caller's thread and may overlap async delivery on AdsLib's thread.
Keep observers brief and make shared state thread-safe. Raw byte pointers are
valid only during the callback. Observer exceptions are contained and counted
by `callbackErrors()`. Do not destroy the client inside callbacks; synchronous
I/O and observer registration/removal inside callbacks are rejected. Queue that
work to another thread. Removal waits for active invocations; stop other client
operations before destruction. No implicit reconnection is attempted.

## Latency-sensitive loops

Resolve symbols and allocate buffers before entering the loop:

```cpp
const auto& symbol = plc->variable("GVL.command");
auto buffer = plc->makeVariable("GVL.command");
buffer.set("/status/enabled", true); // Prepare outside the cyclic loop.
plc->write(symbol, buffer.data(), buffer.size());
plc->read(symbol, buffer.data(), buffer.size());
```

The raw path does no name lookup, JSON conversion or allocation in this wrapper.
It uses caller-owned buffers, cached read/write handles and synchronous ADS
calls. Reads/writes on an ADS transport are serialized. Raw writes bypass value
validation; callers must supply the exact wire layout. JSON getters, setters,
builders and decoded observers allocate and should stay outside tight loops.

ADS/TCP, OS scheduling, mutexes and AdsLib internals do **not** give hard realtime
guarantees. A timeout can block an I/O call. Compare latency and jitter on your
target PLC, network, payload and scheduling policy before using it in a control
loop. The native benchmark uses the same complete `TestStruct3` payload and
reports median/p95 encode+decode CPU times after warmup:

```sh
build-cpp/beckhoff_codec_benchmark
```

An optional historical comparison imports the original Python class. **Only this
comparison** requires `pyads==3.5.0` to import that legacy module; the native client,
native benchmark and test server do not use it:

```sh
python cpp/benchmarks/compare_python.py build-cpp/beckhoff_codec_benchmark
```

It checks all 444 bytes against the original Python encoder and times the same
complex struct encode/decode. Neither benchmark measures PLC/network latency.

## Use from another project or ROS C++ node

```cmake
add_subdirectory(path/to/beckhoff_ros/cpp beckhoff-build)
target_link_libraries(your_node PRIVATE BeckhoffPLC::beckhoff_plc)
```

Or install to a local prefix and consume the exported package:

```sh
cmake --install build-cpp --prefix "$PWD/install-cpp"
```

```cmake
find_package(BeckhoffPLC CONFIG REQUIRED)
target_link_libraries(your_node PRIVATE BeckhoffPLC::beckhoff_plc)
```

Set `CMAKE_PREFIX_PATH` to that prefix. Keep ROS publishers/services and message
conversion in your node; the library neither initializes ROS nor depends on it.
The existing Python ROS node is not migrated by this addition.
