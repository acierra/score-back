---
type: Reference
title: Build & Testing
description: Meson build system, dependencies, build targets, Linux/Windows build instructions, Docker, CI pipelines (Gitea + GitHub Actions), Doxygen, test suite overview, and SCARED dataset E2E test guidance.
tags: [build, testing, meson, docker, ci, doxygen, scared]
sources:
  - id: openwiki-source-29d7e503d29ef2bf863bceaf
    resource: repo://.gitea/workflows/wiki-sync.yaml
  - id: openwiki-source-f9c9abf37ae4501f1a4539da
    resource: repo://scripts/sync_gitea_wiki.sh
generated: { by: "opencode", at: "2026-10-12T11:40:45.370Z" }
verified:
  - by: openwiki/0.5.0
    at: 2026-10-12T11:40:45.370Z
---

# Build & Testing

## Build system

The project uses **Meson** with **Ninja** and requires a **C++23** compiler (GCC or Clang). The root `meson.build` declares the project and dependencies; `src/meson.build` and `tests/meson.build` define build targets.

### Dependencies

All dependencies are resolved via Meson wrap files in `subprojects/` or system packages:

| Dependency | Wrap file | Purpose |
|---|---|---|
| nlohmann_json | `nlohmann_json.wrap` | JSON parsing |
| asio | `asio.wrap` | TCP networking (header-only) |
| glog | `glog.wrap` (CMake subproject) | Logging |
| yaml-cpp | `yaml-cpp.wrap` | Config file parsing |
| aklomp-base64 | `aklomp-base64.wrap` | Base64 encode/decode |
| GoogleTest | `gtest.wrap` | Unit testing (gtest + gmock) |

The `rpc/` directory is a git submodule pointing to [json-rpc-cxx](https://github.com/jsonrpcx/json-rpc-cxx), providing `jsonrpccxx/` headers used by the client. It is included via `include_directories` in the root `meson.build`.

### Build targets

Defined in `src/meson.build`:

**Shared libraries:**
| Library | Sources | Notes |
|---|---|---|
| `libcloud_point_rpc` | `rpc_coder.cpp`, `rpc_dto.cpp`, `rpc_server.cpp`, `server_api.cpp`, `service.cpp` | Core library, installed with `install_rpath: '$ORIGIN'` |
| `libcloud_point_rpc_cli` | `cli.cpp` | CLI client logic |
| `test_cloud_point` | `test_api.cpp` | Test API library |
| `cloud_point_compute` | `src/cloud_point/*.cpp` | OpenCV compute library (optional — only built when `opencv4` is found). Links against `libcloud_point_rpc` + OpenCV. Sources: `image`, `cpu_stereo_matcher`, `gpu_stereo_matcher`, `stereo_matcher_factory`, `stereo_rectifier`, `point_cloud_builder`, `cloud_point_client`, `scared_dataset_loader`. |

**Executables:**
| Executable | Source | Description |
|---|---|---|
| `cloud_point_rpc_server` | `server_main.cpp` | Standalone mock server |
| `cloud_point_rpc_cli` | `main.cpp` | Interactive CLI client |
| `minimal_client` | `minimal_client.cpp` | Minimal client sending a hardcoded `ping` |
| `scared_dataset_server` | `cloud_point/scared_dataset_server.cpp` | RPC server backed by a SCARED dataset keyframe directory (requires opencv4) |

### Linux build

```bash
git submodule init && git submodule update
meson setup build
meson compile -C build
```

Run the server:
```bash
./build/src/cloud_point_rpc_server config.yaml
```

Run the CLI client:
```bash
./build/src/cloud_point_rpc_cli config.yaml
```

### Windows build

Windows requires static linking and a Python venv for Meson:

```powershell
git submodule init
git submodule update
python3 -m venv .\venv
.\venv\Scripts\Activate.ps1
pip install meson cmake
meson setup -Ddefault_library=static build
meson compile -C build
# To get DLLs on PATH:
meson devenv -C build
```

The root `meson.build` adds a `devenv` on Windows that appends subproject DLL directories to `PATH` when `default_library=shared`. When building static, the build defines `BASE64_STATIC_DEFINE` and `YAML_CPP_STATIC_DEFINE`.

### Clean build

```bash
meson compile --clean -C build
```

## Configuration

The server reads a YAML config file. Sample: `config.yml`:

```yaml
server:
  ip: "127.0.0.1"
  port: 9095
```

Full config schema (parsed by `ConfigLoader` in `include/cloud_point_rpc/config.hpp`):

| Section | Field | Type | Default | Description |
|---|---|---|---|---|
| `server.ip` | string | `127.0.0.1` | Server bind address |
| `server.port` | int | `8080` | Server listen port |
| `test_data.intrinsic_params` | list of double | empty (fallback to identity 3×3) | Camera intrinsic parameters |
| `test_data.extrinsic_params` | list of double | empty (fallback to identity 4×4) | Camera extrinsic parameters |
| `test_data.cloud_point` | list of lists of double | empty (fallback to 3 sample points) | Point cloud data |

The `test_data` section is only used by the standalone mock server (`server_main.cpp`). The C API path (`crpc_init`) loads config for server address but does not use `test_data` — Unity provides its own handlers.

## Docker

The `Dockerfile` uses Ubuntu 24.04 and builds the CLI client. It installs build dependencies, copies the project, runs `meson setup build && meson compile -C build`, and starts the CLI by default.

```bash
docker build -t cloud-point-rpc .
docker run --network=host -it cloud-point-rpc
```

Mount a custom config:
```bash
docker run --network=host -it -v $(pwd)/my_config.yaml:/app/config.yaml cloud-point-rpc
```

> The server is not configured to run inside Docker — only the CLI client. The `--network=host` flag simplifies connectivity to a server running on the host.

## CI

`.gitea/workflows/test.yaml` defines a Gitea Actions workflow named "Verification" that runs on push to `master`:

1. Install build tools (cmake, make, ninja, gcc)
2. Install Meson via pip in a venv
3. Checkout with submodules
4. `meson setup build && meson compile -C build -j2`
5. `meson test -C build`

`.github/workflows/openwiki-update.yml` defines a GitHub Actions workflow that runs on a daily schedule (`0 8 * * *`) and on manual dispatch. It installs OpenWiki, runs `openwiki code --update --print` using the OpenRouter provider, and opens a pull request with the regenerated `openwiki/` content. This is how the repository wiki stays current without manual intervention.

`.gitea/workflows/wiki-sync.yaml` publishes the checked-in `openwiki/` pages to the repository's Gitea wiki. It runs on manual dispatch or on pushes to `master` that change `openwiki/**`, `scripts/sync_gitea_wiki.sh`, or the workflow itself. The job checks out the repository and runs the sync script; it supplies `REPO_URL` and uses the `WIKI_TOKEN` secret when configured, falling back to the Gitea job token. The token must have permission to write to the wiki repository.

The script clones the wiki Git repository, copies Markdown pages from `openwiki/`, maps the configured home page (by default `index.md`) to `Home.md`, and rewrites links for Gitea wiki pages and links back to repository files or images. It excludes `INSTRUCTIONS.md`. Before the first run, create at least one page in the Gitea wiki so its Git repository exists. To preview changes without pushing, run `scripts/sync_gitea_wiki.sh --dry-run`; by default it preserves wiki-only pages such as `_Sidebar.md`. Pass `--prune` to remove Markdown pages in the wiki that are absent from `openwiki/`.

## Doxygen

The `Doxyfile` configures Doxygen to generate API documentation from `openwiki/`, `docs/`, `include/`, `src/`, `README.md`, and `API.md`. HTML output goes to `html/` and LaTeX output to `latex/` (both git-ignored). Run with:

```bash
doxygen Doxyfile
```

## Testing

All tests are in `tests/` and compiled into a single `unit_tests` executable (defined in `tests/meson.build`) linked against `cloud_point_rpc_dep`, `cloud_point_rpc_cli_dep`, `cloud_point_rpc_test_dep`, and GoogleTest/GMock.

### Run tests

```bash
meson test -C build          # all tests
meson test -C build -v       # verbose
meson test -C build unit_tests  # explicit
```

### Test suites

| File | Area | Description |
|---|---|---|
| `test_rpc.cpp` | RPC server | Basic request/response, method dispatch |
| `test_rpc_edge_cases.cpp` | RPC server | Edge cases: invalid JSON, missing fields, non-object requests |
| `test_tcp.cpp` | TCP server | TCP connection and message round-trip |
| `test_tcp_edge_cases.cpp` | TCP server | TCP edge cases |
| `test_integration.cpp` | Integration | Full server+client stack with mock data, real TCP |
| `test_cli.cpp` | CLI | CLI client menu and output |
| `test_c_api.cpp` | C API | `crpc_test_*` functions, callback registration, auto-call |
| `test_c_api_edge_cases.cpp` | C API | Multiple methods, removal, scheduling |
| `test_base64.cpp` | Base64 | Encode/decode round-trip |
| `test_base64_edge_cases.cpp` | Base64 | Edge cases (empty input, binary with nulls) |
| `test_serialize.cpp` | Serialization | `serialize`/`deserialize` for numeric types, `inplace_size_embedding` |
| `test_serialize_image.cpp` | Serialization | Image serialization round-trip |
| `test_service.cpp` | Service | Default fallbacks, configured data, empty data, stereo calibration + image pair mocks |
| `test_stereo_matcher.cpp` | Stereo matching | CPU/GPU stereo matcher factory, disparity output, `num_disparities` validation |
| `test_stereo_rectifier.cpp` | Stereo rectification | `StereoRectifier` rectified image pair dimensions and validity |
| `test_point_cloud_builder.cpp` | Point cloud | `PointCloudBuilder` SGBM → reproject → NaN filter pipeline |
| `test_cloud_point_client.cpp` | CloudPointClient | End-to-end facade: connect, compute_cloud, PLY export |
| `test_scared_dataset.cpp` | SCARED E2E | Full stereo pipeline against real SCARED endoscopic data (skipped unless `SCARED_KEYFRAME_DIR` is set) |
| `test_image.cpp` | Image | `Image`/`ImageFactory` construction, Mat dimensions, pixel round-trip |

### Test conventions

- All test fixtures initialize Google Logging in `SetUp()` with `FLAGS_logtostderr = true`.
- Integration tests (`test_integration.cpp`) create a temporary `config.yaml`, start a real `TcpServer` in a thread, and connect via `TCPConnector`/`RpcClient`. Includes `ClientRetrieveRemoteMethods` which verifies the auto-registered `get-available-methods` method.
- C API tests use `crpc_test_init()` / `crpc_test_deinit()` and verify callback invocation via `std::promise`/`std::future`.
- Stereo pipeline tests (`test_stereo_matcher`, `test_stereo_rectifier`, `test_point_cloud_builder`, `test_cloud_point_client`) require OpenCV and are only compiled when `opencv4` is found.

### Linting

The project uses clang-format with LLVM base style and 4-space indent (`.clang-format`):

```bash
ninja -C build clang-format
# or
find src include tests -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
```

## SCARED Dataset E2E Test

The `ScaredDatasetTest.ComputeCloudFromRealData` test (compiled when OpenCV is
found) validates the full stereo pipeline against real endoscopic data from the
[SCARED dataset](https://huggingface.co/datasets/maxhallan7/scared).

The test is **skipped in CI** (no dataset on CI runners). To run it locally:

```bash
export SCARED_KEYFRAME_DIR=/path/to/test_dataset_8/keyframe_0
./build/tests/unit_tests '--gtest_filter=ScaredDataset*'
# or via meson (the test shows as skipped when env var is absent):
meson test -C build -v
```

The test asserts:
- Cloud has more than 50,000 valid points.
- Median z is in `[0.02, 0.20]` m (20 mm – 200 mm, typical endoscopy range).

**Important:** the YAML file stores `T` in millimetres (baseline ≈ −4.35 mm).
`ScaredDatasetLoader` divides `T` by 1000 before populating
`StereoCalibrationRPC.translation` (which is in metres on the wire).
If the median depth looks ~1000× too large, the mm→m conversion is missing.

The SCARED rig has fx ≈ 1024 px and B ≈ 4.35 mm, giving a maximum disparity
of only ~127 px at ~35 mm depth. `scared_dataset_server` and the E2E test
both use `num_disparities = 160` via the `CloudPointClient` constructor's
new `num_disparities` parameter (default 128 — backward-compatible).

## Source references

- `meson.build` — Root build config, dependency declarations
- `src/meson.build` — Library and executable targets
- `tests/meson.build` — Test executable definition
- `config.yml` — Sample config
- `Dockerfile` — Container build
- `.gitea/workflows/test.yaml` — CI pipeline
- `.gitea/workflows/wiki-sync.yaml` — Publish OpenWiki pages to the Gitea wiki
- `scripts/sync_gitea_wiki.sh` — Clone, adapt, and push wiki page content
- `.clang-format` — Code formatting config
