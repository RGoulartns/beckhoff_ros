"""Compare the same local struct encode/decode against the original Python class.

Usage: python cpp/benchmarks/compare_python.py /path/to/beckhoff_codec_benchmark
Requires pyads for importing the original module; never opens a PLC connection.
This measures serialization CPU cost, not ADS round-trip or realtime guarantees.
"""
import copy
import importlib.util
from pathlib import Path
import subprocess
import sys
import time


def main():
    module_path = Path(__file__).resolve().parents[2] / "beckhoff_ros" / "beckhoff_plc.py"
    spec = importlib.util.spec_from_file_location("original_beckhoff_plc", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class OfflinePLC(module.BeckhoffPLC):
        def __del__(self):
            pass

    plc = object.__new__(OfflinePLC)
    plc.pack = 0
    structures = {
        "CustomStruct1_s": {"attr1": "uint16", "attr2": "bool"},
        "CustomStruct2_s": {
            "attr1": "uint16", "attr2": "CustomStruct1_s", "attr3": ["bool", 2]
        },
    }
    plc.loadStructures(copy.deepcopy(structures))
    ctype = plc.loadVariables({"value": "CustomStruct2_s"})["value"]
    value = {"attr1": 42, "attr2": {"attr1": 100, "attr2": True}, "attr3": [False, True]}
    expected = bytes(plc.python2CType(ctype, value))
    assert plc.cType2Python(ctype.from_buffer_copy(expected)) == value
    output = subprocess.run([sys.argv[1]], check=True, text=True, capture_output=True).stdout
    actual = next(line.split(": ", 1)[1] for line in output.splitlines() if line.startswith("Wire bytes: "))
    assert actual == expected.hex(), (actual, expected.hex())
    print(output, end="")
    iterations = 20000
    checksum = 0
    start = time.perf_counter_ns()
    for _ in range(iterations):
        encoded = plc.python2CType(ctype, value)
        decoded = plc.cType2Python(encoded)
        checksum += decoded["attr1"]
    elapsed = time.perf_counter_ns() - start
    print(f"Python ctypes encode+decode: {elapsed / iterations:.1f} ns/iteration, "
          f"iterations={iterations}, checksum={checksum}")
    print("C++ bytes match the original Python encoder; no PLC/network latency measured.")


if __name__ == "__main__":
    main()
