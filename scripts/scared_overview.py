#!/usr/bin/env python3
"""Run scared_dataset_benchmark over many keyframes and report the results.

Usage:
    scripts/scared_overview.py [--bench PATH] [--disparities N] [--png-dir DIR]
                               [--json-dir DIR] [--depth-range MIN_M MAX_M]
                               [--bench-arg=FLAG ...]
                               [--csv PATH] [--xlsx PATH]
                               [--view] [--view-out DIR] [--no-open]
                               [--viewer-stride N] [--serve [PORT]] [--bind ADDR]
                               KEYFRAME_DIR...
    scripts/scared_overview.py --fit-offset [--bench PATH] [--disparities N]
                               [--depth-range MIN_M MAX_M] [--bench-arg=FLAG ...]
                               KEYFRAME_DIR...

Each KEYFRAME_DIR must hold Left_Image.png, Right_Image.png and
endoscope_calibration.yaml. Accuracy columns are filled in only for keyframes
that also contain point_cloud.obj.

The benchmark runs once per keyframe; every output is produced from that one
list of results. A Markdown table is always printed to stdout (nothing else
goes there). Optional outputs: per-keyframe JSON (--json-dir), depth images
(--png-dir), a CSV table (--csv, standard library only), an XLSX workbook
(--xlsx, needs the ``openpyxl`` package; the tool exits with status 2 before
running any benchmark when it is missing) and an offline web viewer site
(--view).

Tuning flags: every ``--bench-arg=FLAG`` (repeatable) is forwarded unchanged,
in the order given, to each benchmark call, e.g.
``--bench-arg=--speckle-window=800 --bench-arg=--disparity-offset=0.1``. Use
the ``=`` form; ``--bench-arg --speckle-window=800`` (two words) is accepted
too but the ``=`` form is the documented one because argparse would otherwise
read the benchmark flag as an option of this tool. The benchmark validates
the flags itself: a rejected flag makes that keyframe a ``failed`` row whose
``Result.error`` carries the benchmark's message (also echoed on stderr). The
flags are listed on the XLSX ``run`` sheet as ``bench_arg`` rows.

Offset fitting: ``--fit-offset`` runs the benchmark on the given keyframes
(only those with ground truth count), combines the per-keyframe
``fit_disparity_offset_px`` into one recommended disparity offset weighted by
matched points, and runs a leave-one-dataset-out check: for each ``dataset_N``
(the path component), the offset fitted on the other datasets is applied to
its keyframes by re-running them with ``--disparity-offset=<fit>`` (after any
``--bench-arg``), and the point-weighted MAE 3D and median signed z bias
before/after are printed. With keyframes from a single dataset only the
offset is printed and the check is reported as needing two datasets. The
mode writes no files: ``--csv``, ``--xlsx``, ``--png-dir``, ``--json-dir``
and the viewer options are rejected together with it.

Viewer: ``--view`` (or ``--view-out DIR`` / ``--serve``) builds a
self-contained site in DIR (default out/viewer) from the template in
scripts/cloud_viewer/site/ with one frame per successful keyframe (failed
keyframes are left out) and its metrics, prints
``viewer: <index.html> (<size> MB, N frames)`` to stderr and opens the site
in the default browser via file:// unless ``--no-open`` is given. The benchmark
is asked for the frames with ``--viewer-frame=PATH --viewer-stride=N``
(``--viewer-stride``, default 4, samples every N-th pixel). ``--serve [PORT]``
(default port 8765) serves the site over HTTP on ``--bind`` (default
127.0.0.1) instead, blocks until Ctrl-C and opens the http URL unless
``--no-open``; a port already in use exits with status 2.

Module structure (for tests and further writers):
    collect(args, extra_args)    -> list of Result(label, status, json, error)
    write_markdown(results, out) -> the stdout table
    write_csv(results, path)
    write_xlsx(results, path, run_info)
    run_info(args, results)      -> dict recorded on the XLSX "run" sheet
    COLUMNS                      -> shared column table for CSV and XLSX
    viewer_frames(results, site) -> manifest entries for the viewer site
    build_site (scripts/cloud_viewer/build_site.py) -> site, manifest, serve
    dataset_id(kf)               -> "dataset_N" path component of a keyframe
    combined_offset(results)     -> point-weighted recommended offset
    fit_offset(args, out)        -> the --fit-offset mode (prints, no files)
"""
import argparse
import collections
import csv
import datetime
import json
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cloud_viewer import build_site  # noqa: E402

DEFAULT_BENCH = "build-opencv/src/cloud_point/scared_dataset_benchmark"
DEFAULT_VIEW_OUT = "out/viewer"
DEFAULT_SERVE_PORT = 8765
DEFAULT_VIEWER_STRIDE = 4

OPENPYXL_MISSING = ("error: --xlsx needs the 'openpyxl' package "
                    "(pip install openpyxl, or apt install python3-openpyxl)")

# One benchmark run. status is "ok" or "failed"; json is the parsed benchmark
# output for "ok" and None for "failed"; error is the benchmark's stderr for
# "failed" (e.g. its usage message for a rejected flag) and None for "ok".
Result = collections.namedtuple("Result", "label status json error", defaults=(None,))


# --------------------------------------------------------------------------
# Collect
# --------------------------------------------------------------------------

def keyframe_label(kf):
    return "/".join(kf.rstrip("/").split("/")[-2:])


def run_benchmark(bench, kf, disparities, png_dir, json_dir, depth_range,
                  extra_args=()):
    """Run the benchmark on one keyframe and return a Result.

    extra_args are optional benchmark flags placed before the positional
    arguments (the benchmark parses flags first, positionals stay compatible).
    """
    label = keyframe_label(kf)
    cmd = [bench] + list(extra_args) + [kf, str(disparities)]
    if png_dir:
        os.makedirs(png_dir, exist_ok=True)
        cmd.append(os.path.join(png_dir, label.replace("/", "_") + ".png"))
    else:
        cmd.append("-")
    if depth_range:
        cmd += [str(depth_range[0]), str(depth_range[1])]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        print(f"{label}: benchmark failed\n{proc.stderr}", file=sys.stderr)
        return Result(label, "failed", None, proc.stderr.strip())
    result = json.loads(proc.stdout)
    if json_dir:
        os.makedirs(json_dir, exist_ok=True)
        with open(os.path.join(json_dir, label.replace("/", "_") + ".json"), "w") as f:
            json.dump(result, f, indent=2)
    return Result(label, "ok", result)


def collect(args, extra_args=()):
    """Run the benchmark once per keyframe and return the list of Results.

    extra_args is a sequence of benchmark flags used for every keyframe, or a
    callable keyframe_dir -> sequence for per-keyframe flags.
    """
    def flags(kf):
        return list(extra_args(kf)) if callable(extra_args) else list(extra_args)
    return [run_benchmark(args.bench, kf, args.disparities, args.png_dir,
                          args.json_dir, args.depth_range, flags(kf))
            for kf in args.keyframes]


# --------------------------------------------------------------------------
# Shared column table (CSV and XLSX)
# --------------------------------------------------------------------------

def _to_mm(v):
    return None if v is None else round(v * 1000.0, 4)


def _to_pct(v):
    return None if v is None else round(v * 100.0, 3)


def _to_ms(v):
    return None if v is None else round(float(v), 3)


def _to_px(v):
    return None if v is None else round(float(v), 4)


def _metric(key, transform=None):
    """Getter for a benchmark JSON value; None when failed or absent."""
    def get(r):
        if r.json is None:
            return None
        v = r.json.get(key)
        return transform(v) if transform else v
    return get


def _nested(outer, key, transform=None):
    """Getter for r.json[outer][key]; None when failed or either is absent."""
    def get(r):
        if r.json is None:
            return None
        inner = r.json.get(outer)
        v = inner.get(key) if isinstance(inner, dict) else None
        return transform(v) if transform and v is not None else v
    return get


# (header, getter(Result) -> value or None, Excel number format or None).
# Units: lengths in mm, fractions in percent, times in ms. None means an empty
# cell in both CSV and XLSX; the getter (not the writer) applies rounding so
# the two outputs hold the same numbers.
COLUMNS = [
    ("keyframe",          lambda r: r.label,                     None),
    ("status",            lambda r: r.status,                    None),
    ("has_ground_truth",  _metric("has_ground_truth"),           None),
    ("valid_points_pct",  _metric("valid_fraction", _to_pct),    "0.0"),
    ("depth_p5_mm",       _metric("z_p05_m", _to_mm),            "0.00"),
    ("depth_median_mm",   _metric("z_median_m", _to_mm),         "0.00"),
    ("depth_p95_mm",      _metric("z_p95_m", _to_mm),            "0.00"),
    ("matching_ms",       _metric("matching_ms", _to_ms),        "0"),
    ("reconstruction_ms", _metric("reconstruction_ms", _to_ms),  "0"),
    ("coverage_pct",      _metric("coverage", _to_pct),          "0.0"),
    ("mae_3d_mm",         _metric("mae_3d_m", _to_mm),           "0.00"),
    ("mae_z_mm",          _metric("mae_z_m", _to_mm),            "0.00"),
    ("rmse_3d_mm",        _metric("rmse_3d_m", _to_mm),          "0.00"),
    ("median_3d_mm",      _metric("median_3d_m", _to_mm),        "0.00"),
    ("within_1mm_pct",    _metric("within_1mm", _to_pct),        "0.0"),
    ("within_2mm_pct",    _metric("within_2mm", _to_pct),        "0.0"),
    ("within_5mm_pct",    _metric("within_5mm", _to_pct),        "0.0"),
    # Diagnostics (ground truth only): bias, fitted offset, tail, coverage loss.
    ("bias_z_median_mm",       _metric("bias_z_median_m", _to_mm),                     "0.000"),
    ("fit_disparity_offset_px", _metric("fit_disparity_offset_px", _to_px),        "0.0000"),
    ("tail_over_5mm_pct",      _metric("tail_over_5mm", _to_pct),                      "0.000"),
    ("tail_over_20mm_pct",     _metric("tail_over_20mm", _to_pct),                     "0.000"),
    ("max_3d_mm",              _metric("max_3d_m", _to_mm),                            "0.00"),
    ("lost_search_range_pct",  _nested("coverage_loss", "out_of_search_range", _to_pct), "0.00"),
    ("lost_matcher_pct",       _nested("coverage_loss", "matcher_rejected", _to_pct),   "0.00"),
]

HEADERS = [c[0] for c in COLUMNS]


def table_row(result):
    """Values of one table row in COLUMNS order (None = empty cell)."""
    return [getter(result) for _, getter, _ in COLUMNS]


# --------------------------------------------------------------------------
# Writers
# --------------------------------------------------------------------------

def fmt_mm(v):
    return "" if v is None else f"{v * 1000:.2f}"


def fmt_pct(v):
    return "" if v is None else f"{v * 100:.1f}"


def write_markdown(results, out=None):
    out = sys.stdout if out is None else out
    print("| keyframe | valid % | z p5 / median / p95 (mm) | match ms | GT | coverage % | MAE3D mm | RMSE3D mm | median mm | <1 mm % | <2 mm % | <5 mm % |", file=out)
    print("|---|---|---|---|---|---|---|---|---|---|---|---|", file=out)
    for res in results:
        label, r = res.label, res.json
        if r is None:
            print(f"| {label} | failed | | | | | | | | | | |", file=out)
            continue
        z = f"{r['z_p05_m']*1000:.0f} / {r['z_median_m']*1000:.0f} / {r['z_p95_m']*1000:.0f}"
        gt = r.get("has_ground_truth", False)
        print("| {} | {} | {} | {:.0f} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
            label, fmt_pct(r["valid_fraction"]), z, r["matching_ms"],
            "yes" if gt else "no",
            fmt_pct(r.get("coverage")), fmt_mm(r.get("mae_3d_m")),
            fmt_mm(r.get("rmse_3d_m")), fmt_mm(r.get("median_3d_m")),
            fmt_pct(r.get("within_1mm")), fmt_pct(r.get("within_2mm")),
            fmt_pct(r.get("within_5mm"))), file=out)


def _csv_cell(v):
    if v is None:
        return ""
    if isinstance(v, bool):
        return "true" if v else "false"
    return v


def write_csv(results, path):
    """RFC 4180 CSV (UTF-8, CRLF, header row) with COLUMNS."""
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(HEADERS)
        for r in results:
            w.writerow([_csv_cell(v) for v in table_row(r)])


def require_openpyxl():
    """Exit with status 2 and a hint if openpyxl cannot be imported."""
    try:
        import openpyxl  # noqa: F401
    except ImportError:
        print(OPENPYXL_MISSING, file=sys.stderr)
        sys.exit(2)


def git_commit():
    try:
        proc = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True,
                              text=True, cwd=os.path.dirname(os.path.abspath(__file__)))
    except OSError:
        return ""
    return proc.stdout.strip() if proc.returncode == 0 else ""


def run_info(args, results):
    """Run parameters recorded on the XLSX 'run' sheet (ordered dict)."""
    depth_range = args.depth_range
    if depth_range is None:
        for r in results:
            if r.json is not None and "min_depth_m" in r.json:
                depth_range = (r.json["min_depth_m"], r.json["max_depth_m"])
                break
    info = collections.OrderedDict()
    info["num_disparities"] = args.disparities
    info["min_depth_m"] = None if depth_range is None else float(depth_range[0])
    info["max_depth_m"] = None if depth_range is None else float(depth_range[1])
    info["timestamp_utc"] = (datetime.datetime.now(datetime.timezone.utc)
                             .replace(microsecond=0).isoformat())
    info["git_commit"] = git_commit()
    info["benchmark"] = args.bench
    info["bench_args"] = list(getattr(args, "bench_args", None) or [])
    info["keyframes"] = list(args.keyframes)
    return info


def write_xlsx(results, path, info):
    """Workbook with a 'metrics' sheet (COLUMNS) and a 'run' sheet (info)."""
    import openpyxl
    from openpyxl.styles import Font
    from openpyxl.utils import get_column_letter

    wb = openpyxl.Workbook()
    ws = wb.active
    ws.title = "metrics"
    ws.append(HEADERS)
    for cell in ws[1]:
        cell.font = Font(bold=True)
    for r in results:
        ws.append(table_row(r))
    for col, (_, _, number_format) in enumerate(COLUMNS, start=1):
        if number_format:
            for row in range(2, ws.max_row + 1):
                ws.cell(row=row, column=col).number_format = number_format
        ws.column_dimensions[get_column_letter(col)].width = max(
            12, len(HEADERS[col - 1]) + 2)
    ws.column_dimensions["A"].width = max(
        ws.column_dimensions["A"].width, *(len(r.label) + 2 for r in results))
    ws.freeze_panes = "A2"
    ws.auto_filter.ref = f"A1:{get_column_letter(len(COLUMNS))}{max(ws.max_row, 2)}"

    run = wb.create_sheet("run")
    run.append(["key", "value"])
    for cell in run[1]:
        cell.font = Font(bold=True)
    for key, value in info.items():
        if key == "keyframes":
            for kf in value:
                run.append(["keyframe", kf])
        elif key == "bench_args":
            # One row per forwarded benchmark flag, in the order given.
            for flag in value:
                run.append(["bench_arg", flag])
        else:
            run.append([key, value])
    run.column_dimensions["A"].width = 18
    run.column_dimensions["B"].width = 60
    wb.save(path)


# --------------------------------------------------------------------------
# Viewer site
# --------------------------------------------------------------------------

def viewer_frame_id(kf):
    """Frame id for a keyframe dir: its label with '/' -> '_', sanitised."""
    return build_site.frame_id(keyframe_label(kf))


def viewer_extra_args(site_dir, stride):
    """Benchmark flags (per keyframe) that make it write a viewer frame."""
    def flags(kf):
        return [f"--viewer-frame={build_site.frame_path(site_dir, viewer_frame_id(kf))}",
                f"--viewer-stride={stride}"]
    return flags


def viewer_frames(results, site_dir, err=None):
    """Manifest frame entries for the successful results whose frame exists.

    Failed keyframes are skipped (they were already reported). A successful
    run without a frame file is reported on ``err`` and skipped too.
    """
    err = sys.stderr if err is None else err
    frames = collections.OrderedDict()
    for res in results:
        label, status, r = res.label, res.status, res.json
        if status != "ok" or r is None:
            continue
        fid = build_site.frame_id(label)
        path = r.get("viewer_frame") or build_site.frame_path(site_dir, fid)
        if not os.path.isfile(path):
            print(f"{label}: no viewer frame written ({path}), skipped in the site",
                  file=err)
            continue
        frames[fid] = build_site.frame_entry(fid, label, r)
    return list(frames.values())


def write_viewer(results, site_dir, err=None):
    """Write the manifest for ``results`` into the prepared site; print a summary.

    Returns the path of index.html.
    """
    err = sys.stderr if err is None else err
    frames = viewer_frames(results, site_dir, err)
    index, size = build_site.build_site(site_dir, frames)
    print(build_site.describe(index, size, len(frames)), file=err)
    return index


# --------------------------------------------------------------------------
# Offset fitting (--fit-offset)
# --------------------------------------------------------------------------

_DATASET_RE = re.compile(r"^(test_)?dataset_\d+$")


def dataset_id(kf):
    """The ``dataset_N`` path component of a keyframe dir.

    Falls back to the keyframe's parent directory name when no component
    matches, so any two-level layout still groups by parent.
    """
    parts = [p for p in kf.replace("\\", "/").rstrip("/").split("/") if p]
    for p in reversed(parts):
        if _DATASET_RE.match(p):
            return p
    return parts[-2] if len(parts) >= 2 else parts[0] if parts else ""


def _fit_rows(results):
    """(Result, matched_points, current offset, delta) for fittable results.

    A result is fittable when it succeeded, has ground truth and reports
    ``fit_disparity_offset_px`` with a positive ``matched_points``.
    """
    rows = []
    for r in results:
        j = r.json
        if r.status != "ok" or j is None or not j.get("has_ground_truth"):
            continue
        delta = j.get("fit_disparity_offset_px")
        weight = j.get("matched_points") or 0
        if delta is None or weight <= 0:
            continue
        current = float((j.get("settings") or {}).get("disparity_offset_px", 0.0))
        rows.append((r, float(weight), current, float(delta)))
    return rows


def combined_offset(results):
    """Point-weighted recommended disparity offset over ``results``.

    Each fittable keyframe recommends ``current offset + fitted delta``; these
    are averaged weighted by ``matched_points``. Returns (offset_px, weight)
    or (None, 0) when nothing is fittable.
    """
    rows = _fit_rows(results)
    weight = sum(w for _, w, _, _ in rows)
    if weight <= 0:
        return None, 0
    total = sum(w * (cur + delta) for _, w, cur, delta in rows)
    return total / weight, weight


def _weighted_metric(results, key):
    """Point-weighted mean of json[key] over ok GT results, or None."""
    num = den = 0.0
    for r in results:
        j = r.json
        if r.status != "ok" or j is None or key not in j:
            continue
        w = float(j.get("matched_points") or 0)
        num += w * float(j[key])
        den += w
    return num / den if den > 0 else None


def dataset_summary(results):
    """{'mae_mm', 'bias_mm'} for one dataset (point-weighted; None if empty)."""
    return {"mae_mm": _to_mm(_weighted_metric(results, "mae_3d_m")),
            "bias_mm": _to_mm(_weighted_metric(results, "bias_z_median_m"))}


def _fmt(v, spec):
    return "" if v is None else format(v, spec)


def fit_offset(args, out=None, err=None):
    """Run the --fit-offset mode: print the fit and the holdout check.

    Writes no files. Returns the process exit status (0, or 1 when no
    ground-truth keyframe could be fitted).
    """
    out = sys.stdout if out is None else out
    err = sys.stderr if err is None else err
    bench_args = list(args.bench_args or [])

    def run(kf, extra=()):
        return run_benchmark(args.bench, kf, args.disparities, None, None,
                             args.depth_range, bench_args + list(extra))

    results = [run(kf) for kf in args.keyframes]
    rows = _fit_rows(results)
    for r in results:
        if r.status == "ok" and r.json is not None and not r.json.get("has_ground_truth"):
            print(f"{r.label}: no ground truth, not used for the fit", file=err)
    if not rows:
        print("error: --fit-offset needs at least one successful keyframe with "
              "ground truth", file=err)
        return 1

    print("| keyframe | dataset | matched pts | current offset (px) "
          "| fitted delta (px) | recommended (px) | MAE3D mm | bias z mm |", file=out)
    print("|---|---|---|---|---|---|---|---|", file=out)
    by_dataset = collections.OrderedDict()
    for r, w, cur, delta in rows:
        ds = dataset_id(r.json.get("keyframe_dir") or r.label)
        by_dataset.setdefault(ds, []).append(r)
        print("| {} | {} | {:.0f} | {:+.4f} | {:+.4f} | {:+.4f} | {} | {} |".format(
            r.label, ds, w, cur, delta, cur + delta,
            _fmt(_to_mm(r.json.get("mae_3d_m")), ".3f"),
            _fmt(_to_mm(r.json.get("bias_z_median_m")), ".3f")), file=out)

    offset, weight = combined_offset(results)
    print("", file=out)
    print("recommended disparity offset: {:+.4f} px (point-weighted over {} "
          "keyframe{} from {} dataset{}, {:.0f} matched points)".format(
              offset, len(rows), "s" if len(rows) != 1 else "",
              len(by_dataset), "s" if len(by_dataset) != 1 else "", weight), file=out)
    print("apply with: --bench-arg=--disparity-offset={:.4f} "
          "(config: cloud_point.disparity_offset_px)".format(offset), file=out)

    if len(by_dataset) < 2:
        print("", file=out)
        print("holdout check skipped: the leave-one-dataset-out check needs "
              "keyframes from at least two datasets (got {}: {})".format(
                  len(by_dataset), ", ".join(by_dataset)), file=out)
        return 0

    print("", file=out)
    print("leave-one-dataset-out check (each dataset re-run with the offset "
          "fitted on the others):", file=out)
    print("| held-out dataset | keyframes | fit from others (px) | MAE3D before mm "
          "| MAE3D after mm | bias z before mm | bias z after mm | abs bias decreased |",
          file=out)
    print("|---|---|---|---|---|---|---|---|", file=out)
    for ds, ds_results in by_dataset.items():
        others = [r for other, rs in by_dataset.items() if other != ds for r in rs]
        fit, _ = combined_offset(others)
        before = dataset_summary(ds_results)
        kfs = [r.json.get("keyframe_dir") or r.label for r in ds_results]
        rerun = [run(kf, [f"--disparity-offset={fit:.6f}"]) for kf in kfs]
        after = dataset_summary(rerun)
        if after["bias_mm"] is None or before["bias_mm"] is None:
            verdict = "failed"
        else:
            verdict = "yes" if abs(after["bias_mm"]) < abs(before["bias_mm"]) else "no"
        print("| {} | {} | {:+.4f} | {} | {} | {} | {} | {} |".format(
            ds, len(ds_results), fit,
            _fmt(before["mae_mm"], ".3f"), _fmt(after["mae_mm"], ".3f"),
            _fmt(before["bias_mm"], ".3f"), _fmt(after["bias_mm"], ".3f"),
            verdict), file=out)
    return 0


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def _join_bench_arg_pairs(argv):
    """Turn ``--bench-arg FLAG`` (two words) into ``--bench-arg=FLAG``.

    argparse would otherwise treat a FLAG starting with ``--`` as an option
    of this tool. The ``=`` form is the documented one.
    """
    joined, i = [], 0
    while i < len(argv):
        a = argv[i]
        if a == "--bench-arg" and i + 1 < len(argv):
            joined.append(f"--bench-arg={argv[i + 1]}")
            i += 2
        else:
            joined.append(a)
            i += 1
    return joined


def parse_args(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--bench", default=DEFAULT_BENCH)
    ap.add_argument("--disparities", type=int, default=160)
    ap.add_argument("--png-dir")
    ap.add_argument("--json-dir")
    ap.add_argument("--depth-range", nargs=2, type=float, metavar=("MIN_M", "MAX_M"),
                    help="depth filter in metres (default: builder defaults 0.01..10)")
    ap.add_argument("--bench-arg", dest="bench_args", action="append", default=[],
                    metavar="FLAG",
                    help="benchmark tuning flag forwarded unchanged to every "
                         "benchmark call, e.g. --bench-arg=--speckle-window=800; "
                         "repeatable, forwarded in the order given (use the = form)")
    ap.add_argument("--fit-offset", action="store_true",
                    help="fit a disparity offset from the ground-truth keyframes "
                         "and print a leave-one-dataset-out check; writes no files")
    ap.add_argument("--csv", metavar="PATH", help="write the metrics table as CSV")
    ap.add_argument("--xlsx", metavar="PATH",
                    help="write the metrics table as an XLSX workbook (needs openpyxl)")
    view = ap.add_argument_group("viewer site")
    view.add_argument("--view", action="store_true",
                      help="build the offline viewer site and open it in the browser")
    view.add_argument("--view-out", metavar="DIR",
                      help=f"site directory (default {DEFAULT_VIEW_OUT}; implies --view)")
    view.add_argument("--no-open", action="store_true",
                      help="build (or serve) the site without opening a browser")
    view.add_argument("--viewer-stride", type=int, default=DEFAULT_VIEWER_STRIDE,
                      metavar="N", help="sample every N-th pixel for the viewer "
                      f"frames (default {DEFAULT_VIEWER_STRIDE})")
    view.add_argument("--serve", nargs="?", type=int, const=DEFAULT_SERVE_PORT,
                      metavar="PORT", help="serve the site over HTTP on PORT "
                      f"(default {DEFAULT_SERVE_PORT}) until Ctrl-C instead of "
                      "opening it via file:// (implies --view); give the port "
                      "or put --serve after the keyframes so that a keyframe "
                      "path is not taken as PORT")
    view.add_argument("--bind", default="127.0.0.1", metavar="ADDR",
                      help="address --serve listens on (default 127.0.0.1)")
    ap.add_argument("keyframes", nargs="+")
    args = ap.parse_args(_join_bench_arg_pairs(sys.argv[1:] if argv is None else list(argv)))
    if args.viewer_stride < 1:
        ap.error("--viewer-stride must be >= 1")
    if args.serve is not None and not 0 < args.serve < 65536:
        ap.error("--serve PORT must be between 1 and 65535")
    args.view = args.view or args.view_out is not None or args.serve is not None
    if args.fit_offset:
        writers = [name for name, value in [("--csv", args.csv), ("--xlsx", args.xlsx),
                                            ("--png-dir", args.png_dir),
                                            ("--json-dir", args.json_dir),
                                            ("--view/--view-out/--serve", args.view)]
                   if value]
        if writers:
            ap.error("--fit-offset writes no files; drop " + ", ".join(writers))
    if args.view_out is None:
        args.view_out = DEFAULT_VIEW_OUT
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.fit_offset:
        return fit_offset(args)
    if args.xlsx:
        require_openpyxl()

    bench_args = list(args.bench_args)
    extra_args = bench_args
    if args.view:
        # The template is copied and frames/ created before the benchmark
        # runs so that it can write its frame files straight into the site.
        site_dir = build_site.prepare_site(args.view_out)
        viewer_flags = viewer_extra_args(site_dir, args.viewer_stride)

        def extra_args(kf):  # noqa: F811 - tuning flags first, then viewer flags
            return bench_args + viewer_flags(kf)

    results = collect(args, extra_args)

    write_markdown(results)
    if args.csv:
        write_csv(results, args.csv)
        print(f"wrote {args.csv}", file=sys.stderr)
    if args.xlsx:
        write_xlsx(results, args.xlsx, run_info(args, results))
        print(f"wrote {args.xlsx}", file=sys.stderr)
    if args.view:
        index = write_viewer(results, site_dir)
        if args.serve is not None:
            return build_site.serve(site_dir, args.serve, args.bind,
                                    open_browser=not args.no_open)
        if not args.no_open:
            build_site.open_in_browser(index)
    return 0


if __name__ == "__main__":
    sys.exit(main())
