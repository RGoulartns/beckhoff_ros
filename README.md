# beckhoff_ros

Beckhoff ADS + ROS2

The BeckhoffPLC class expands pyads capabilities by allowing the usage of any data types. \
A list of primitive data types are listed in the 'Beckhoff.cTypes' structure (expand it as needed). This class also allows 'lists of lists' and 'list of custom DUT'. \
Users must specify the custom DUTs in the params.yaml (or json.. if used). All variables defined in the parameters file are loaded at runtime.

Note: All reads/writes are done using byte arrays of ctypes instead of 'pyads.PLCTYPE'.

## C++ version

[beckhoff_plc_cpp](beckhoff_plc_cpp/README.md) is a C++ version of the BeckhoffPLC class: a ROS-agnostic CMake library that reads the same parameter file (config/params.json), with lower latency, for real-time applications.
