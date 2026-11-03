---
type: API
title: C API for Unity Integration
description: C API surface for embedding the RPC server in Unity or other native consumers — lifecycle functions, rpc_string memory management, method registration, and the test API for handler testing.
tags: [c-api, unity, server-api, test-api, rpc_string, integration]
verified:
  - by: openwiki/0.5.0
    at: 2026-10-12T11:40:45.370Z
sources:
  - id: openwiki-source-57a2b8ed5ffb1454a6d4e159
    resource: repo://include/server_api.h
  - id: openwiki-source-d53a3dd812e6cd06aa3a676b
    resource: repo://include/test_api.h
  - id: openwiki-source-6a27631dfafd447a04ce33ed
    resource: repo://src/server_api.cpp
  - id: openwiki-source-18c61a32d76a956498fa032e
    resource: repo://src/test_api.cpp
---

# C API for Unity Integration

The C API allows Unity (or any C/C++ consumer) to embed the RPC server as a shared library, register custom RPC handlers as C function pointers, and manage the server lifecycle without touching C++ directly.

> **Unity C# design spec**: For the complete Unity-side implementation guide — including `rpc_string` ownership rules, IL2CPP callback pinning, main-thread dispatcher design, and shutdown ordering — see [docs/unity-integration.md](../docs/unity-integration.md). The C API described here is the C++ side of the interface; the C# side is the Unity project's responsibility.

Two C API surfaces exist:

| API | Header | Library | Purpose |
|---|---|---|---|
| Server API | `include/server_api.h` | `libcloud_point_rpc` | Start/stop the TCP server, register RPC methods |
| Test API | `include/test_api.h` | `test_cloud_point` | Internal test harness: register methods, schedule calls, auto-call loop |

Both APIs use the `rpc_string` type and `callback_t` function pointer typedef.

## Export macros

`include/export.h` defines `CRPC_EXPORT`. On Windows, it resolves to `__declspec(dllexport)` when `CRPC_SERVER_API_EXPORT` is defined (set in `src/meson.build`) and `__declspec(dllimport)` otherwise. On GCC/Clang, it uses `__attribute__((visibility("default")))`. This allows the same headers to be used when building the library and when consuming it.

## `rpc_string`

Defined in `include/cloud_point_rpc/rpc_server.hpp` inside an `extern "C"` block:

```c
struct rpc_string {
    std::string s;  // C++ std::string, but the struct is C-ABI compatible
};
```

Although the struct contains a `std::string`, it is allocated and managed by the library. Consumers interact with it through opaque pointers and accessor functions:

| Function | Description |
|---|---|
| `crpc_str_create(data, size)` | Allocate a new `rpc_string` with the given data. Tracked by internal GC. |
| `crpc_str_destroy(ptr)` | Manually free a `rpc_string`. |
| `crpc_str_get_data(ptr)` | Get the raw C string pointer. |
| `crpc_str_get_size(ptr)` | Get the string length. |

### Garbage collector

`src/server_api.cpp` maintains a static `std::list<std::unique_ptr<rpc_string>> gc` protected by `gc_mtx`. All `crpc_str_create` allocations are tracked in this list. `crpc_deinit()` clears the entire list. `crpc_str_destroy` removes a specific entry. This prevents memory leaks if Unity forgets to call `destroy`, though manual destruction is recommended to avoid excessive memory usage.

> **Important**: `rpc_string` pointers returned from callbacks are owned by the library's GC. Do not `free()` them — use `crpc_str_destroy()`.

## Server API (`server_api.h`)

### Lifecycle

```c
// Initialize and start the server from a config file
crpc_init("config.yaml");

// Or initialize with an explicit address (no config file needed)
crpc_init_with_address("127.0.0.1", 9095);

// ... register methods and serve ...

// Stop server and free all GC-tracked rpc_strings
crpc_deinit();
```

- `crpc_init(config_path)` — Loads YAML config via `ConfigLoader`, creates a `TcpServer` with the configured IP/port, and starts it. Initializes Google Logging if not already initialized. See [Architecture → Config](architecture.md#config).
- `crpc_init_with_address(ip, port)` — Same but without a config file. Used when the consumer wants to set the address directly.
- `crpc_deinit()` — Stops the server (resets the `TcpServer` unique_ptr) and clears the `rpc_string` GC list.

### Registering methods

```c
rpc_string* my_handler(rpc_string* params_json) {
    // params_json->s contains the JSON params as a string
    // Build your result (JSON or raw string)
    return crpc_str_create("{\"key\":\"value\"}", 15);
}

rpc_string method_name;
method_name.s = "my-method";
crpc_add_method(my_handler, &method_name);
```

- `callback_t` is `rpc_string* (*)(rpc_string*)` — a C function pointer.
- The callback receives the JSON `params` object serialized as a string in `rpc_string->s`.
- The callback returns a `rpc_string*` whose string is parsed as JSON if possible, or used as a raw string in the `result` field. See [RPC Protocol → Handler registration](rpc-protocol.md#handler-registration).
- `crpc_add_method` is guarded by `server_mtx` and registers the callback on the global `RpcServer`.

### Global state

`server_api.cpp` uses file-level statics: `rpc_server` (the global `RpcServer`), `server` (the `TcpServer` unique_ptr), `gc` (the string GC list), and two mutexes (`gc_mtx`, `server_mtx`). This means only one server instance is supported per process.

## Test API (`test_api.h`)

The test API is built into a separate shared library (`test_cloud_point`) and provides a `TestThread` class (in `src/test_api.cpp`) that runs a background `std::jthread` for testing registered methods without a real TCP connection.

### Lifecycle

```c
crpc_test_init();       // Start the test thread + Google Logging
// ... register methods, schedule calls ...
crpc_test_deinit();     // Stop thread, call crpc_deinit(), reset state
```

### Methods

| Function | Description |
|---|---|
| `crpc_test_add_method(cb, name)` | Register a method on the test `RpcServer`. Duplicates are ignored. |
| `crpc_test_remove_method(name)` | Remove a registered method. Returns 0 on success, -1 if not found. |
| `crpc_test_schedule_call(name)` | Enqueue a one-shot call to the named method (processed by the test thread). |
| `crpc_test_change_duration(ms)` | Set the auto-call sleep interval (default 50ms). |
| `crpc_test_duration()` | Get the current sleep interval. |
| `crpc_test_auto_call(state)` | Enable (1) or disable (0) auto-calling registered methods on each sleep cycle. |

### Test thread behavior

The `TestThread::routine()` loop:
1. If there are queued one-shot calls, process them (build a JSON-RPC request and call `server.process()`).
2. If auto-call is enabled and methods exist, call the next method in round-robin order.
3. If auto-call is enabled and the queue is empty, wait on a condition variable for the configured duration (or until stop is requested).
4. Stop when `jthread` stop is requested via `crpc_test_deinit()`.

> **Note**: The test API does **not** start a TCP server. It processes JSON-RPC requests directly through `RpcServer::process()`, logging results. It is designed for testing handler registration and callback behavior in C, as demonstrated in `tests/test_c_api.cpp` and `tests/test_c_api_edge_cases.cpp`.

## Source references

- `include/server_api.h` — Server C API declarations
- `src/server_api.cpp` — Server C API implementation, `rpc_string` GC
- `include/test_api.h` — Test C API declarations
- `src/test_api.cpp` — `TestThread` implementation and test C API
- `include/export.h` — `CRPC_EXPORT` macro
- `include/cloud_point_rpc/rpc_server.hpp` — `rpc_string` struct and `callback_t` typedef