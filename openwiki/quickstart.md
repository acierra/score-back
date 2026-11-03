---
type: Quickstart
title: Cloud Point RPC Quickstart
description: Entry point for the CloudPointRPC code wiki. Covers what the project is, repository layout, build/run instructions, and links to all major documentation sections.
tags: [quickstart, overview, navigation]
verified:
  - by: openwiki/0.5.0
    at: 2026-10-12T11:40:45.370Z
sources:
  - id: openwiki-source-29d7e503d29ef2bf863bceaf
    resource: repo://.gitea/workflows/wiki-sync.yaml
  - id: openwiki-source-f9c9abf37ae4501f1a4539da
    resource: repo://scripts/sync_gitea_wiki.sh
generated: { by: "opencode", at: "2026-10-12T11:40:45.370Z" }
---

# Cloud Point RPC — Quickstart

## What is this?

Cloud Point RPC is a **C++23 JSON-RPC 2.0** server and client implementation designed to bridge a C++ backend with a **Unity Scene** over TCP. Unity serves stereo camera data (`get-stereo-calibration`, `get-image-pair`) over the embedded RPC server; the C++ `CloudPointClient` fetches calibration once on `connect()`, then on each `compute_cloud()` call retrieves a synchronised image pair, runs stereo rectification + SGBM disparity + `cv::reprojectImageTo3D`, and returns a filtered `PointCloud`. A C API (`server_api.h`) allows Unity to embed the server, register custom RPC handlers, and manage the server lifecycle from native code.

The server side with C-API is fully implemented. The C++ stereo point-cloud client (`CloudPointClient`, `StereoRectifier`, `PointCloudBuilder`) is implemented. The remaining item is the Unity-side C# implementation; see [docs/unity-integration.md](../docs/unity-integration.md).

## Repository layout

| Path | Purpose |
|---|---|
| `include/cloud_point_rpc/` | C++ public headers: TCP server/client, RPC server/client, config, serialization, service, coder |
| `include/server_api.h` | C API for embedding the server in Unity/native consumers |
| `include/test_api.h` | C API for test-driven method registration and scheduled calls |
| `include/export.h` | Cross-platform shared-library export macros (`CRPC_EXPORT`) |
| `src/` | Implementation files and executable entrypoints |
| `tests/` | GTest/GMock unit and integration tests (single `unit_tests` executable) |
| `rpc/` | Git submodule — [json-rpc-cxx](https://github.com/jsonrpcx/json-rpc-cxx) providing `jsonrpccxx` headers |
| `subprojects/` | Meson wrap dependencies (asio, nlohmann_json, glog, yaml-cpp, base64, gtest) |
| `include/cloud_point/` | OpenCV compute library headers: `StereoRectifier`, `PointCloudBuilder`, `CloudPointClient` |
| `src/cloud_point/` | OpenCV compute library implementation (optional, requires opencv4) |
| `docs/` | PlantUML communication model diagram and Unity integration design spec |
| `config.yml` | Sample server configuration (IP and port) |
| `Dockerfile` | Container image for the CLI client |
| `.gitea/workflows/test.yaml` | CI pipeline (build + test on push to master) |
| `.gitea/workflows/wiki-sync.yaml`, `scripts/sync_gitea_wiki.sh` | Publish the generated OpenWiki pages to the repository's Gitea wiki |
| `.github/workflows/openwiki-update.yml` | Scheduled GitHub Actions workflow that refreshes OpenWiki docs daily and opens a PR |
| `Doxyfile` | Doxygen config — generates HTML/LaTeX API docs from `openwiki/`, `docs/`, `include/`, `src/`, `README.md`, `API.md` (output in `html/` and `latex/`, git-ignored) |

## Build and run

```bash
git submodule init && git submodule update
meson setup build
meson compile -C build
```

Start the test server (uses mock camera data from `config.yml`):

```bash
./build/src/cloud_point_rpc_server config.yaml
```

Run the interactive CLI client:

```bash
./build/src/cloud_point_rpc_cli config.yaml
```

CLI menu options (options 4 and 5 are hidden when built without opencv4):

| Option | Action |
|--------|--------|
| 4 | Compute point cloud — prints point count and bounding box |
| 5 | Compute point cloud and save to `output.ply` |

Run all tests:

```bash
meson test -C build -v
```

For Windows build instructions and Docker usage, see [Build & Testing](build-and-testing.md).

## Documentation sections

- [Architecture](architecture.md) — Layered design, TCP framing, threading model, communication flow
- [RPC Protocol](rpc-protocol.md) — JSON-RPC 2.0 methods, request/response format, error codes, Base64 encoding
- [C API](c-api.md) — C interface for Unity integration, `rpc_string` memory management, test API
- [Build & Testing](build-and-testing.md) — Meson build system, dependencies, config, Docker, CI, test suite overview

## Publishing the wiki

The Gitea Wiki Sync workflow publishes `openwiki/` pages to this repository's Gitea wiki on demand or after relevant pushes to `master`. Preview a local publication with `scripts/sync_gitea_wiki.sh --dry-run`. See [Build & Testing](build-and-testing.md#ci) for the token, first-run setup, link rewriting, and pruning behavior.

## Key concepts

- **Namespace**: All C++ code lives in `score` (renamed from `cloud_point_rpc` early in development).
- **Wire framing**: Every TCP message is prefixed with an 8-byte little-endian `uint64_t` payload size, then the JSON-RPC payload follows. See [Architecture → Wire framing](architecture.md#wire-framing).
- **Two server entrypoints**: `server_main.cpp` is a standalone executable with mock data; `server_api.cpp` provides the embeddable C API that Unity uses to start the server and register callbacks.
- **Base64**: Image pixel payloads (`get-image-pair`) are Base64-encoded for ASCII-safe transport over JSON. Calibration arrays are plain JSON doubles — not Base64. Encoding on the Unity side; decoding in `Base64RPCCoder` on the client side per API.md.
- **Persistent connections**: `TcpServer::handle_client` loops per connection — multiple RPC round-trips share one TCP connection without reconnecting.
- **std::expected**: The `cloud_point` compute library uses `std::expected<PointCloud, Error>` (C++23) as its return type. Callers check `has_value()` before accessing the result.
