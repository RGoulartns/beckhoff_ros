"""Latency of the Python BeckhoffPLC (beckhoff_ros/beckhoff_plc.py, unchanged).

Measures the same operations as bench_beckhoff_plc.cpp; see compare.py.
Needs pyads. Prints one JSON object per line.

    python3 bench_python.py <params.json> [--net-id ID] [--iterations N]
"""
import argparse
import json
import os
import sys
import threading
import time


def report(operation, iterations, run):
    for _ in range(min(200, iterations // 5 + 1)):  # warm up
        run()
    us = []
    for _ in range(iterations):
        start = time.perf_counter()
        run()
        us.append((time.perf_counter() - start) * 1e6)
    us.sort()

    def percentile(p):
        return us[min(len(us) - 1, int(p * len(us)))]

    print(json.dumps({"op": operation, "n": len(us), "mean_us": sum(us) / len(us),
                      "p50_us": percentile(0.50), "p90_us": percentile(0.90),
                      "p99_us": percentile(0.99)}), flush=True)


def main():
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    parser = argparse.ArgumentParser()
    parser.add_argument("params")
    parser.add_argument("--net-id")
    parser.add_argument("--iterations", type=int, default=2000)
    parser.add_argument("--repo", default=repo, help="root of the beckhoff_ros repository")
    args = parser.parse_args()

    sys.path.insert(0, args.repo)
    sys.dont_write_bytecode = True  # keep the repository clean
    from beckhoff_ros.beckhoff_plc import BeckhoffPLC

    with open(args.params) as file:
        params = json.load(file)["PLC"]
    if args.net_id:
        params["net_id"] = args.net_id
    plc = BeckhoffPLC(params)

    values = [{"attr1": i, "attr2": {"attr1": i, "attr2": True}, "attr3": [True, False]}
              for i in range(10)]
    n = args.iterations

    report("getVariable struct", n, lambda: plc.getVariable("GVL.obj2"))
    report("getVariable 10 structs", n, lambda: plc.getVariable("GVL.obj1"))
    flag = [False]

    def toggle():
        flag[0] = not flag[0]
        plc.setVariable("GVL.var1", flag[0])

    report("setVariable bool", n, toggle)
    report("setVariable 10 structs (dicts)", n, lambda: plc.setVariable("GVL.obj1", values))

    def read_three():
        for name in ("GVL.obj1", "GVL.obj2", "GVL.var1"):
            plc.getVariable(name)

    report("read 3 variables", n, read_three)

    # Write a monitored variable and wait for its notification.
    received = threading.Event()
    expected = [0]

    def observer(name, value):
        if name == "GVL.obj2" and value["attr1"] == expected[0]:
            received.set()

    plc.addObserver(observer)

    def round_trip():
        expected[0] = (expected[0] + 1) % 60000
        received.clear()
        plc.setVariable("GVL.obj2", {"attr1": expected[0], "attr2": expected[0] % 2 == 0})
        if not received.wait(2):
            raise RuntimeError("no notification")

    report("notification round trip", n, round_trip)

    # Conversion between values and bytes only (no ADS request)
    ctype = plc.variables["GVL.obj1"]
    report("convert dicts to bytes (no I/O)", n, lambda: bytearray(plc.python2CType(ctype, values)))
    raw = ctype.from_buffer(bytearray(plc.python2CType(ctype, values)))
    report("convert bytes to dicts (no I/O)", n, lambda: plc.cType2Python(raw))
    os._exit(0)  # BeckhoffPLC never closes its connection


if __name__ == "__main__":
    main()
