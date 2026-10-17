"""Compares the latency of the C++ and the Python BeckhoffPLC.

Starts the fake PLC (fake_ads_server) with the variables of
benchmark/params.json (those of config/params.json), runs bench_beckhoff_plc (C++) and bench_python.py (the
unchanged Python class, needs pyads) against it and prints a table.

    cmake -B build -DBECKHOFF_PLC_BUILD_BENCHMARKS=ON && cmake --build build
    python3 benchmark/compare.py --build build --python /path/to/python-with-pyads
"""
import argparse
import json
import os
import signal
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NET_ID = "127.0.0.1.1.1"


def run(command):
    output = subprocess.run(command, check=True, capture_output=True, text=True).stdout
    return {line["op"]: line for line in map(json.loads, output.splitlines()) if line}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", default=os.path.join(HERE, "..", "build"), help="CMake build directory")
    parser.add_argument("--python", default=sys.executable, help="Python interpreter with pyads installed")
    parser.add_argument("--params", default=os.path.join(HERE, "params.json"))
    parser.add_argument("--iterations", type=int, default=2000)
    args = parser.parse_args()

    server = subprocess.Popen([os.path.join(args.build, "test", "fake_ads_server"), "--config", args.params],
                              stdout=subprocess.PIPE, text=True)
    try:
        server.stdout.readline()  # "fake ADS server listening on ..."
        common = [args.params, "--net-id", NET_ID, "--iterations", str(args.iterations)]
        cpp = run([os.path.join(args.build, "benchmark", "bench_beckhoff_plc")] + common)
        python = run([args.python, os.path.join(HERE, "bench_python.py")] + common)
    finally:
        server.send_signal(signal.SIGINT)
        server.wait()

    print("| Operation | Python p50 | C++ p50 | Python p99 | C++ p99 | Speed-up (p50) |")
    print("|---|---:|---:|---:|---:|---:|")
    for op, c in cpp.items():
        p = python.get(op)
        if p:
            print(f"| {op} | {p['p50_us']:.1f} µs | {c['p50_us']:.1f} µs | {p['p99_us']:.1f} µs | "
                  f"{c['p99_us']:.1f} µs | {p['p50_us'] / c['p50_us']:.1f}x |")
        else:
            print(f"| {op} | - | {c['p50_us']:.2f} µs | - | {c['p99_us']:.2f} µs | |")


if __name__ == "__main__":
    main()
