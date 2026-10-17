"""Optional historical comparison against the original Python class.

Usage: python cpp/benchmarks/compare_python.py /path/to/beckhoff_codec_benchmark
Requires pyads only to import that legacy module; never opens a PLC connection.
The C++ library, C++ benchmark and integration tests do not use this dependency.
This measures serialization CPU cost, not ADS round-trip or realtime guarantees.
"""
import copy
import importlib.util
import json
from pathlib import Path
import statistics
import subprocess
import sys
import time


def main():
    if importlib.util.find_spec("pyads") is None:
        raise SystemExit("Only this optional legacy comparison needs pyads: install pyads==3.5.0, "
                         "or run beckhoff_codec_benchmark directly for the native C++ benchmark.")
    module_path = Path(__file__).resolve().parents[2] / "beckhoff_ros" / "beckhoff_plc.py"
    spec = importlib.util.spec_from_file_location("original_beckhoff_plc", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class OfflinePLC(module.BeckhoffPLC):
        def __del__(self):
            pass

    plc = object.__new__(OfflinePLC)
    plc.pack = 0
    parameters = Path(__file__).resolve().parents[1] / "examples" / "test_struct3.json"
    structures = json.loads(parameters.read_text())["PLC"]["structures"]
    plc.loadStructures(copy.deepcopy(structures))

    def leaf(group, item):
        index = (group * 10 + item) ^ 0x55
        return {"attrbt1": bool(index & 1),
                "attrbt2": [bool((index >> (bit + 1)) & 1) for bit in range(6)]}

    ctype = plc.loadVariables({"value": "TestStruct3"})["value"]
    value = {
        "attr1": True,
        "attr2": 0xA35C,
        "attr3": [{"attr1": [leaf(group, item) for item in range(10)],
                   "attr2": 0x2100 + group * 0x0137} for group in range(6)],
        "attr4": leaf(6, 0),
    }
    expected = bytes(plc.python2CType(ctype, value))
    if plc.cType2Python(ctype.from_buffer_copy(expected)) != value:
        raise RuntimeError("original Python codec failed TestStruct3 round trip")
    output = subprocess.run([sys.argv[1]], check=True, text=True, capture_output=True).stdout
    native = json.loads(output)
    actual = bytes(native["wire_bytes"])
    if actual != expected:
        raise RuntimeError(f"C++ and Python bytes differ: {actual.hex()} != {expected.hex()}")
    iterations = 1000
    checksum = 0
    for _ in range(100):
        plc.cType2Python(plc.python2CType(ctype, value))
    durations = []
    for _ in range(iterations):
        start = time.perf_counter_ns()
        encoded = plc.python2CType(ctype, value)
        decoded = plc.cType2Python(encoded)
        checksum += decoded["attr2"]
        del decoded, encoded
        durations.append(time.perf_counter_ns() - start)
    durations.sort()
    print(f"TestStruct3: {native['bytes']} bytes, 6 groups x 10 nested TestStruct1 values plus attr4")
    print(f"C++ JSON encode+decode: median={native['median_ns']} ns, p95={native['p95_ns']} ns, "
          f"iterations={native['iterations']}, checksum={native['checksum']}")
    print(f"Python ctypes encode+decode: median={statistics.median(durations):.0f} ns, "
          f"p95={durations[iterations * 95 // 100]} ns, iterations={iterations}, checksum={checksum}")
    print("C++ bytes match the original Python encoder; no PLC/network latency measured.")


if __name__ == "__main__":
    main()
