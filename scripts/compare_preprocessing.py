#!/usr/bin/env python3
"""Paired SCARED comparison: baseline none versus configured preprocessing.

Only preprocessing varies. Every call uses identical matcher/depth flags.
Missing ground truth, zero scored points and failed runs are errors, never
zero-error results. Raw benchmark JSON is retained in the comparison report.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

METRICS = ("mae_3d_m", "rmse_3d_m", "mae_z_m", "coverage", "valid_fraction",
           "within_1mm", "within_2mm", "within_5mm", "preprocessing_ms",
           "matching_ms", "reconstruction_ms")


def run_case(bench, keyframe, config, common_args, positionals=()):
    command = [str(bench), *common_args]
    if config is not None:
        command.append("--preprocessing-config=" + str(config))
    command.append(str(keyframe))
    command.extend(positionals)
    try:
        proc = subprocess.run(command, capture_output=True, text=True)
        if proc.returncode:
            return {"status": "failed", "command": command,
                    "error": proc.stderr.strip() or "benchmark failed"}
        data = json.loads(proc.stdout)
        if not data.get("has_ground_truth"):
            raise ValueError("point_cloud.obj is required for a DoD comparison")
        if data.get("matched_points", 0) <= 0:
            raise ValueError("no scored points; errors are undefined")
        for key in METRICS:
            value = data.get(key)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("missing or non-finite metric: " + key)
        return {"status": "ok", "command": command, "result": data}
    except (OSError, ValueError) as error:
        return {"status": "failed", "command": command, "error": str(error)}


def compare(bench, keyframes, variants, common_args, positionals=()):
    rows = []
    for keyframe in keyframes:
        baseline = run_case(bench, keyframe, None, common_args, positionals)
        rows.append({"keyframe": str(keyframe), "variant": "none", **baseline})
        for name, config in variants:
            case = run_case(bench, keyframe, config, common_args, positionals)
            if case["status"] == "ok" and baseline["status"] == "ok":
                original = baseline["result"]
                result = case["result"]
                b_settings = dict(original.get("settings", {}))
                v_settings = dict(result.get("settings", {}))
                b_settings.pop("preprocessing", None)
                v_settings.pop("preprocessing", None)
                if not b_settings or b_settings != v_settings:
                    case = {"status": "failed", "command": case["command"],
                            "error": "non-preprocessing settings differ from baseline",
                            "result": result}
                else:
                    case["delta_vs_none"] = {
                        key: result[key] - original[key] for key in METRICS}
            elif baseline["status"] != "ok" and case["status"] == "ok":
                case["status"] = "failed"
                case["error"] = "baseline failed; paired comparison is unavailable"
            rows.append({"keyframe": str(keyframe), "variant": name, **case})
    return rows


def write_reports(prefix, report):
    prefix = Path(prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    Path(str(prefix) + ".json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    fields = ["keyframe", "variant", "status", "error", *METRICS,
              *("delta_" + key for key in METRICS)]
    with Path(str(prefix) + ".csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for case in report["runs"]:
            row = {key: case.get(key, "") for key in fields[:4]}
            if case["status"] == "ok":
                row.update({key: case["result"][key] for key in METRICS})
                row.update({"delta_" + key: value for key, value in
                            case.get("delta_vs_none", {}).items()})
            writer.writerow(row)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("keyframes", nargs="+", type=Path)
    parser.add_argument("--bench", required=True, type=Path)
    parser.add_argument("--variant", action="append", required=True,
                        metavar="NAME=YAML", help="repeat for each preprocessing chain")
    parser.add_argument("--bench-arg", action="append", default=[],
                        help="common flag; use --bench-arg=--block-size=5")
    parser.add_argument("--disparities", type=int, default=160)
    parser.add_argument("--min-depth-m", type=float, default=0.02)
    parser.add_argument("--max-depth-m", type=float, default=0.30)
    parser.add_argument("--output", type=Path, default=Path("out/preprocessing-comparison"))
    args = parser.parse_args(argv)
    if (args.disparities <= 0 or args.disparities % 16 or
            not math.isfinite(args.min_depth_m) or
            not math.isfinite(args.max_depth_m) or
            not 0 < args.min_depth_m < args.max_depth_m):
        parser.error("use positive disparities divisible by 16 and 0 < min < max depth")
    variants = []
    names = {"none"}
    for value in args.variant:
        name, sep, path = value.partition("=")
        if not sep or not name or name in names or not Path(path).is_file():
            parser.error("variant needs a unique NAME=existing.yaml; none is reserved")
        names.add(name)
        variants.append((name, Path(path).resolve()))
    # Prevent a common option from overriding the baseline or injecting an
    # extra keyframe. Reconstruction flags are validated by the benchmark.
    for flag in args.bench_arg:
        if not flag.startswith("--") or flag.startswith("--preprocessing-"):
            parser.error("bench-arg must be a non-preprocessing --flag")
    bench = args.bench.resolve()
    if not bench.is_file():
        parser.error("benchmark executable does not exist")
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    positionals = [str(args.disparities), "-", str(args.min_depth_m),
                   str(args.max_depth_m)]
    rows = compare(bench, [p.resolve() for p in args.keyframes], variants,
                   args.bench_arg, positionals)
    report = {"status": "complete" if all(r["status"] == "ok" for r in rows) else "failed",
              "benchmark": str(bench), "benchmark_sha256": digest(bench),
              "common_args": args.bench_arg,
              "common_positionals": positionals,
              "variants": [{"name": name, "config": str(path),
                            "sha256": digest(path), "yaml": path.read_text()}
                           for name, path in variants], "runs": rows}
    write_reports(args.output, report)
    print(f"{report['status']}: {args.output}.json, {args.output}.csv")
    return 0 if report["status"] == "complete" else 1


if __name__ == "__main__":
    sys.exit(main())
