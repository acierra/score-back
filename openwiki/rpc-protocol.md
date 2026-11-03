---
type: Protocol
title: RPC Protocol
description: JSON-RPC 2.0 wire protocol over TCP — request/response formats, error codes, all RPC methods (get-stereo-calibration, get-image-pair, get-available-methods, legacy methods), and handler registration.
tags: [rpc, json-rpc, protocol, api, wire-format]
generated: { by: "opencode", at: "2026-10-12T11:40:45.370Z" }
verified:
  - by: openwiki/0.5.0
    at: 2026-10-12T11:40:45.370Z
sources:
  - id: openwiki-source-4bbac091a25773429cb5718a
    resource: repo://include/cloud_point_rpc/rpc_dto.hpp
  - id: openwiki-source-3c77e0f2f7c4637ea432ebef
    resource: repo://src/rpc_server.cpp
  - id: openwiki-source-7190a628f0f2e262d1a9e7fa
    resource: repo://tests/test_rpc_edge_cases.cpp
---

# RPC Protocol

## JSON-RPC 2.0

The server implements **JSON-RPC 2.0** over TCP with length-prefixed framing (see [Architecture → Wire framing](architecture.md#wire-framing)).

Batch requests are not supported — only single request objects are processed.

### Request format

```json
{
  "jsonrpc": "2.0",
  "method": "<method_name>",
  "params": {},
  "id": <integer|string>
}
```

The `params` field is accepted but currently ignored by all implemented methods. The `id` field is required; requests without it receive a `-32600 Invalid Request` error.

### Success response

```json
{
  "jsonrpc": "2.0",
  "result": <method_specific_result>,
  "id": <matching_request_id>
}
```

### Error response

```json
{
  "jsonrpc": "2.0",
  "error": {
    "code": <integer>,
    "message": "<string>"
  },
  "id": <matching_request_id>
}
```

### Error codes

| Code | Meaning | When |
|---|---|---|
| `-32700` | Parse error | Request is not valid JSON |
| `-32600` | Invalid Request | Missing `jsonrpc`, `method`, or `id` fields, or `jsonrpc != "2.0"` |
| `-32601` | Method not found | No handler registered for the requested method name |
| `-32000` | Server error | Handler threw an exception |

> **Known issue**: Non-object JSON (arrays, strings, numbers, null) causes a `nlohmann::json::type_error` instead of a clean `-32600` response. See `tests/test_rpc_edge_cases.cpp`.

Source: `src/rpc_server.cpp` — `RpcServer::process()` and `create_error()`.

## Methods

### `get-stereo-calibration`

Returns full stereo rig calibration: intrinsics for both cameras, stereo rotation and translation, and sensor resolution. Called once by `CloudPointClient::connect()`.

**Request:**
```json
{"jsonrpc": "2.0", "method": "get-stereo-calibration", "id": 1}
```

**Response:**
```json
{
  "left":  { "camera_matrix": [9 doubles], "dist_coeffs": [5 doubles] },
  "right": { "camera_matrix": [9 doubles], "dist_coeffs": [5 doubles] },
  "rotation":    [9 doubles],
  "translation": [3 doubles],
  "image_size":  { "width": 640, "height": 480 }
}
```

All matrices are row-major. `camera_matrix` is the 3×3 intrinsic matrix; `rotation` is the 3×3 left-to-right rotation (OpenCV convention); `translation` is in metres. Calibration arrays are **plain JSON doubles — not Base64**.

Test mock defaults: fx=fy=800, cx=320, cy=240, zero distortion, R=I, T=[-0.06, 0, 0], 640×480.

---

### `get-image-pair`

Returns a synchronised stereo frame as two base64-encoded images. Called on each `CloudPointClient::compute_cloud()`.

**Request:**
```json
{"jsonrpc": "2.0", "method": "get-image-pair", "id": 2}
```

**Response:**
```json
{
  "frame": 42,
  "left":  { "width": 640, "height": 480, "type": "BGR", "data": "<base64>" },
  "right": { "width": 640, "height": 480, "type": "BGR", "data": "<base64>" }
}
```

`data` is Base64-encoded raw pixel bytes (standard alphabet, no line breaks). `type` is `"BGR"` (3 ch), `"RGBA"` (4 ch), or `"DEPTH"` (1 ch float32). Unity must flip GPU readback vertically before encoding (wire format is top-left-origin row-major).

DTOs: `StereoCalibrationDto` and `ImagePairDto` in `include/cloud_point_rpc/rpc_dto.hpp`. Decoded client-side via `Base64RPCCoder`.

---

### `get-available-methods`

Returns the names of all methods registered on the server. Auto-registered in the `RpcServer` constructor — not added by the application. Useful for client-side discovery.

**Request:**
```json
{"jsonrpc": "2.0", "method": "get-available-methods", "id": 5}
```

**Response:**
```json
{"jsonrpc": "2.0", "result": ["get-available-methods", "get-stereo-calibration", "get-image-pair"], "id": 5}
```

Result is a JSON array of method-name strings, including `get-available-methods` itself. Tested in `tests/test_integration.cpp` (`ClientRetrieveRemoteMethods`).

Source: `src/rpc_server.cpp` — `RpcServer::RpcServer()` constructor; `get_method_names()`.

---

### `get-intrinsic-params` *(legacy)*

Retrieves left-camera intrinsic parameters as a flat 3×3 matrix (row-major, 9 doubles). Not used by `CloudPointClient`; kept for backward compatibility.

**Request:**
```json
{"jsonrpc": "2.0", "method": "get-intrinsic-params", "id": 3}
```

**Response:**
```json
{"jsonrpc": "2.0", "result": [fx, 0, cx, 0, fy, cy, 0, 0, 1], "id": 3}
```

Result: 9 plain JSON doubles. **Not Base64.**

### `get-extrinsic-params` *(legacy)*

Retrieves left-camera extrinsic matrix as a flat 4×4 matrix (row-major, 16 doubles). Not used by `CloudPointClient`; kept for backward compatibility.

**Request:**
```json
{"jsonrpc": "2.0", "method": "get-extrinsic-params", "id": 4}
```

**Response:**
```json
{"jsonrpc": "2.0", "result": [16 doubles], "id": 4}
```

Result: 16 plain JSON doubles. **Not Base64.**

### `get-cloud-point`

Retrieves the current field-of-view point cloud.

**Request:**
```json
{"jsonrpc": "2.0", "method": "get-cloud-point", "id": 3}
```

**Response:**
```json
{"jsonrpc": "2.0", "result": {"width": int, "height": int, "data": <base64-encoded-array>}, "id": 3}
```

Result type: `matrix WxH` (list of `[x, y, z]` points), Base64-encoded.

> **Note**: The standalone mock server (`server_main.cpp`) returns these as raw JSON arrays (not Base64-encoded) via `Service`. The Base64 encoding is expected to be implemented on the Unity side per `API.md`.

## Handler registration

Handlers are registered with `RpcServer::register_method()`. Two forms exist:

### C++ handler

```cpp
<!-- openwiki: broken internal link [const json& params] file "const json& params" does not exist. Fix the href or restore the target, then delete this comment. -->
rpc_server.register_method("get-intrinsic-params", [&](const json& params) {
    return service.get_intrinsic_params();  // returns vector<double>
});
```

The handler returns `std::variant<nlohmann::json, std::string>`. If a `json` is returned, it is placed directly in the `result` field. If a `std::string` is returned, it is placed as-is.

### C callback handler

```cpp
rpc_string* my_callback(rpc_string* params_json) {
    // params_json->s contains the JSON params as a string
    // return a result string (JSON or raw)
    return crpc_str_create("result_data", 11);
}

rpc_server.register_method("my-method", my_callback);
```

The C callback receives the JSON `params` as a string in `rpc_string->s`. The return value's string is parsed as JSON if possible; otherwise it is used as a raw string in the `result` field.

Source: `src/rpc_server.cpp` — `register_method(name, callback_t)` overload.

## Base64 encoding

`include/cloud_point_rpc/rpc_coder.hpp`, `src/rpc_coder.cpp`

- `Base64RPCCoder` implements the `IRPCCoder` interface with `encode()` and `decode()` methods.
- Uses the [aklomp/base64](https://github.com/aklomp/base64) library (Meson wrap `subprojects/aklomp-base64.wrap`).
- `encode(vector<char>) → string` and `decode(string) → vector<char>`.
- Includes overflow protection: throws `std::length_error` if input exceeds safe size limits.

> Per `API.md`: **only image pixel data** (`get-image-pair` `data` fields) uses Base64. Calibration arrays (`camera_matrix`, `dist_coeffs`, `rotation`, `translation`) and legacy params are plain JSON doubles — never Base64. Unity encodes images before sending; the C++ client decodes them via `Base64RPCCoder`.

## Client-side usage

`include/cloud_point_rpc/rpc_client.hpp`

`RpcClient` wraps `jsonrpccxx::JsonRpcClient` and provides typed access:

```cpp
score::TCPConnector connector(ip, port);
score::RpcClient client(connector);

auto intrinsic = client.get_intrinsic_params();   // vector<double>
auto extrinsic = client.get_extrinsic_params();   // vector<double>
auto cloud     = client.get_cloud_point();        // vector<vector<double>>
```

The interactive CLI (`src/cli.cpp`) and `minimal_client` (`src/minimal_client.cpp`) demonstrate client usage.
