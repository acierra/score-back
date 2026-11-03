"""Build, serve and open the offline point-cloud viewer site.

The site is the template in ``site/`` next to this file (index.html, viewer.js,
viewer.css, vendor/) plus a ``frames/`` directory that holds one
``frames/<id>.js`` per frame (written by ``scared_dataset_benchmark
--viewer-frame=PATH``) and ``frames/manifest.js``, which lists the frames and
their metrics:

    CloudViewer.setManifest({"title": ..., "generated": ..., "frames": [
        {"id": ..., "label": ..., "has_gt": ..., "metrics": {...}}, ...]});

Only the standard library is used, so this module works wherever
``scared_overview.py`` runs. Everything is importable and side-effect free
except ``serve()`` (blocks) and ``open_in_browser()``.
"""
import datetime
import errno
import http.server
import json
import os
import pathlib
import re
import shutil
import sys
import webbrowser

TEMPLATE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "site")
MANIFEST_NAME = "manifest.js"
DEFAULT_TITLE = "SCARED reconstruction viewer"

# Benchmark JSON keys copied into a frame's manifest entry (SI units as the
# benchmark emits them). Absent or null values are left out.
METRIC_KEYS = (
    "valid_fraction", "z_p05_m", "z_median_m", "z_p95_m",
    "matching_ms", "reconstruction_ms",
    "coverage", "mae_3d_m", "mae_z_m", "rmse_3d_m", "median_3d_m",
    "within_1mm", "within_2mm", "within_5mm",
)

FRAME_ID_RE = re.compile(r"^[A-Za-z0-9_.-]+$")


def frame_id(label):
    """Turn a keyframe label such as ``dataset_1/keyframe_1`` into a frame id.

    Path separators become ``_`` and any other character outside
    ``[A-Za-z0-9_.-]`` becomes ``-``; the result matches the id regex used by
    the C++ writer and the viewer.
    """
    fid = re.sub(r"[/\\]+", "_", str(label).strip())
    fid = re.sub(r"[^A-Za-z0-9_.-]", "-", fid)
    fid = fid.strip("._-") or "frame"
    assert FRAME_ID_RE.match(fid), fid
    return fid


def frame_path(site_dir, fid):
    """Absolute path of the frame script for id ``fid`` inside ``site_dir``."""
    return os.path.join(os.path.abspath(site_dir), "frames", fid + ".js")


def prepare_site(site_dir):
    """Copy the template into ``site_dir`` (created if needed) and add frames/.

    Existing frame files are kept, so repeated builds into the same directory
    add up; the manifest is rewritten by :func:`write_manifest`.
    """
    if not os.path.isfile(os.path.join(TEMPLATE_DIR, "index.html")):
        raise FileNotFoundError(f"viewer template not found at {TEMPLATE_DIR}")
    site_dir = os.path.abspath(site_dir)
    shutil.copytree(TEMPLATE_DIR, site_dir, dirs_exist_ok=True)
    os.makedirs(os.path.join(site_dir, "frames"), exist_ok=True)
    return site_dir


def filter_metrics(benchmark_json):
    """The manifest ``metrics`` dict: METRIC_KEYS present with non-null values."""
    if not benchmark_json:
        return {}
    return {k: benchmark_json[k] for k in METRIC_KEYS
            if benchmark_json.get(k) is not None}


def frame_entry(fid, label, benchmark_json):
    """One manifest frame entry from a benchmark JSON dict."""
    return {
        "id": fid,
        "label": label,
        "has_gt": bool(benchmark_json.get("has_ground_truth", False)),
        "metrics": filter_metrics(benchmark_json),
    }


def make_manifest(frames, title=DEFAULT_TITLE, generated=None):
    if generated is None:
        generated = (datetime.datetime.now(datetime.timezone.utc)
                     .replace(microsecond=0).isoformat())
    return {"title": title, "generated": generated, "frames": list(frames)}


def write_manifest(site_dir, manifest):
    """Write ``frames/manifest.js`` (``CloudViewer.setManifest(<json>);``)."""
    for f in manifest["frames"]:
        if not FRAME_ID_RE.match(str(f.get("id", ""))):
            raise ValueError(f"invalid frame id {f.get('id')!r}")
    path = os.path.join(site_dir, "frames", MANIFEST_NAME)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("CloudViewer.setManifest(")
        fh.write(json.dumps(manifest, indent=1))
        fh.write(");\n")
    return path


def read_manifest(site_dir):
    """Parse ``frames/manifest.js`` back into a dict (for tests and tooling)."""
    with open(os.path.join(site_dir, "frames", MANIFEST_NAME), encoding="utf-8") as fh:
        text = fh.read().strip()
    prefix, suffix = "CloudViewer.setManifest(", ");"
    if not (text.startswith(prefix) and text.endswith(suffix)):
        raise ValueError("not a manifest.js file")
    return json.loads(text[len(prefix):-len(suffix)])


def site_size_bytes(site_dir):
    """Total size of all regular files under ``site_dir``."""
    total = 0
    for p in pathlib.Path(site_dir).rglob("*"):
        if p.is_file():
            total += p.stat().st_size
    return total


def build_site(site_dir, frames, title=DEFAULT_TITLE, generated=None):
    """Prepare the site directory and write the manifest for ``frames``.

    ``frames`` is a list of manifest frame entries (see :func:`frame_entry`)
    whose ``frames/<id>.js`` files are already in place (or will be). Returns
    ``(index_html_path, size_bytes)``.
    """
    site_dir = prepare_site(site_dir)
    write_manifest(site_dir, make_manifest(frames, title, generated))
    return os.path.join(site_dir, "index.html"), site_size_bytes(site_dir)


def describe(index_path, size_bytes, n_frames):
    """One-line summary, e.g. ``viewer: /x/index.html (3.2 MB, 2 frames)``."""
    plural = "" if n_frames == 1 else "s"
    return (f"viewer: {index_path} ({size_bytes / 1e6:.1f} MB, "
            f"{n_frames} frame{plural})")


def file_uri(path):
    return pathlib.Path(os.path.abspath(path)).as_uri()


def open_in_browser(path, out=None):
    """Open ``path`` (a file or an http URL) in the default browser.

    Returns True when a browser was launched. Never raises: when no browser
    can be found, prints a note with the location and returns False.
    """
    out = sys.stderr if out is None else out
    url = path if "://" in str(path) else file_uri(path)
    try:
        ok = bool(webbrowser.open(url))
    except Exception:  # webbrowser.Error or a broken launcher
        ok = False
    if not ok:
        print(f"note: could not open a browser; the viewer is at {url}", file=out)
    return ok


class _QuietHandler(http.server.SimpleHTTPRequestHandler):
    """Serves the site directory; logs one short line per request to stderr."""

    def log_message(self, fmt, *args):
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))


def make_server(site_dir, port, bind="127.0.0.1"):
    """Create (but do not run) the HTTP server for ``site_dir``.

    Raises OSError (errno EADDRINUSE) when the port is taken.
    """
    site_dir = os.path.abspath(site_dir)

    class Handler(_QuietHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=site_dir, **kwargs)

    return http.server.ThreadingHTTPServer((bind, port), Handler)


def serve(site_dir, port, bind="127.0.0.1", open_browser=False, out=None,
          server=None):
    """Serve ``site_dir`` over HTTP until Ctrl-C.

    Prints the URL, optionally opens it in the browser, then blocks. Returns 0
    on a clean shutdown. A port that is already in use prints an error and
    returns 2 (the caller exits with it) without serving.
    """
    out = sys.stderr if out is None else out
    if server is None:
        try:
            server = make_server(site_dir, port, bind)
        except OSError as e:
            if e.errno == errno.EADDRINUSE:
                print(f"error: port {port} on {bind} is already in use",
                      file=out)
                return 2
            raise
    host = server.server_address[0]
    url = f"http://{host}:{server.server_address[1]}/index.html"
    print(f"serving {os.path.abspath(site_dir)} at {url} (Ctrl-C to stop)",
          file=out)
    if open_browser:
        open_in_browser(url, out)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0
