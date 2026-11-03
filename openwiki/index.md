---
okf_version: "0.2"
---

# Files

- [Architecture](architecture.md) - Layered architecture for JSON-RPC 2.0 over TCP, including the OpenCV compute layer, RPC/server/transport layers, threading model, and the SCARED dataset validation server.
- [Build & Testing](build-and-testing.md) - Meson build system, dependencies, build targets, Linux/Windows build instructions, Docker, CI pipelines (Gitea + GitHub Actions), Doxygen, test suite overview, and SCARED dataset E2E test guidance.
- [C API for Unity Integration](c-api.md) - C API surface for embedding the RPC server in Unity or other native consumers — lifecycle functions, rpc_string memory management, method registration, and the test API for handler testing.
- [Cloud Point RPC Quickstart](quickstart.md) - Entry point for the CloudPointRPC code wiki. Covers what the project is, repository layout, build/run instructions, and links to all major documentation sections.
- [RPC Protocol](rpc-protocol.md) - JSON-RPC 2.0 wire protocol over TCP — request/response formats, error codes, all RPC methods (get-stereo-calibration, get-image-pair, get-available-methods, legacy methods), and handler registration.
