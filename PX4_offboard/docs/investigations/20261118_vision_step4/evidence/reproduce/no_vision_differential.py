#!/usr/bin/env python3
"""基线与当前纯C++控制轨迹逐字节比较；没有ROS/硬件连接，不修改工程源码。"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile

BASELINE = "2948bfe4e6a26b5ee58bf02e4e9988e484fad81e"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="必须是尚不存在的证据目录")
    args = parser.parse_args()
    root = Path(subprocess.check_output(["git", "rev-parse", "--show-toplevel"], text=True).strip())
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    summary = {"baseline": BASELINE, "results": {}}
    with tempfile.TemporaryDirectory(prefix="fc01-step4-baseline-") as directory:
        baseline = Path(directory)
        data = subprocess.check_output(["git", "archive", BASELINE, "native"], cwd=root)
        with tarfile.open(fileobj=io.BytesIO(data)) as archive:
            for member in archive.getmembers():
                parts = Path(member.name).parts
                if (Path(member.name).is_absolute() or ".." in parts or
                        not (member.isfile() or member.isdir())):
                    raise ValueError("unsafe baseline archive member")
            archive.extractall(baseline)
        for name in ("control", "runtime", "publication"):
            traces = []
            item = {}
            for label, tree in (("baseline", baseline), ("current", root)):
                binary = output / (name + "-" + label)
                source = tree / ("native/tests/test_stage2_" + name + "_trace.cpp")
                argv = ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic", "-pthread",
                        "-I", str(tree / "native/src/jikl_fc01_system/include"),
                        "-I", str(tree / "native/tests"), str(source), "-o", str(binary)]
                built = subprocess.run(argv, capture_output=True, timeout=120)
                (output / (name + "-" + label + "-build.log")).write_bytes(built.stdout + built.stderr)
                built.check_returncode()
                run = subprocess.run([str(binary)], capture_output=True, timeout=60)
                (output / (name + "-" + label + ".trace")).write_bytes(run.stdout)
                (output / (name + "-" + label + "-stderr.log")).write_bytes(run.stderr)
                run.check_returncode()
                traces.append(run.stdout)
                item[label] = {"sha256": hashlib.sha256(run.stdout).hexdigest(),
                               "lines": len(run.stdout.splitlines()), "bytes": len(run.stdout)}
            item["byte_identical"] = traces[0] == traces[1]
            summary["results"][name] = item
    summary["all_byte_identical"] = all(item["byte_identical"] for item in summary["results"].values())
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    if not summary["all_byte_identical"]:
        raise SystemExit("FC01_STEP4_NO_VISION_DIFFERENTIAL_FAILED")
    print("FC01_STEP4_NO_VISION_DIFFERENTIAL_PASS", sum(item["current"]["lines"] for item in summary["results"].values()))


if __name__ == "__main__":
    main()
