# Latency benchmark, compared with the Python version by compare.py.
add_executable(bench_beckhoff_plc bench_beckhoff_plc.cpp)
target_link_libraries(bench_beckhoff_plc PRIVATE beckhoff_plc)
