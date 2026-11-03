# Unity Integration Design

This document specifies the C# architecture the Unity side (the
`UnityLaparoscopicSceneSimulator` project) must implement to serve stereo
images and calibration to the C++ `CloudPointClient` over the embedded RPC
server. It is a design spec — the C# code lives in the Unity repository.

The existing prototypes there (`Assets/Scripts/CrpcApi.cs`,
`Assets/Scripts/RpcTest.cs`) are a starting point but contain several
correctness bugs called out explicitly below.

## Data flow

```
Unity process (server)                     C++ client process
──────────────────────                     ──────────────────
CloudPointServer (C#)
  crpc_init / crpc_add_method
  get-stereo-calibration  ◄────────────── CloudPointClient::connect()   (once)
  get-image-pair          ◄────────────── CloudPointClient::compute_cloud()
                                             │ rectify → SGBM → reproject
                                             ▼
                                          PointCloud → caller
```

Unity is a **data source only**. The point cloud is computed and consumed on
the client side. See [API.md](../API.md) for the wire schemas.

## 1. `CloudPointServer` facade

A single MonoBehaviour owning the server lifecycle
(`Assets/Scripts/CloudPointRpc/CloudPointServer.cs`):

- `Awake()` — call `crpc_init(configPath)` (config.yaml under
  `Application.streamingAssetsPath`, contains `server: {ip, port}`), then
  register handlers.
- `OnDestroy()` / `OnApplicationQuit()` — shut down in the order specified in
  §3 (Shutdown ordering).
- JSON: use Newtonsoft Json.NET (`com.unity.nuget.newtonsoft-json`).
  `JsonUtility` cannot serialize dictionaries or nested arrays.

### Callback registration rules (IL2CPP + GC safety)

- Handler methods must be **static** and annotated with
  `[AOT.MonoPInvokeCallback(typeof(RpcStringCallback))]` — instance methods
  crash under IL2CPP.
- The delegate instance passed to `crpc_add_method` must be stored in a
  **static field** for the lifetime of the server. The current `RpcTest.cs`
  passes a method group directly; the marshalled thunk can be garbage
  collected while C++ still holds the function pointer — a latent crash.

```csharp
private static readonly CrpcTestApi.RpcStringCallback s_calibrationCb = OnGetStereoCalibration;
// ...
CrpcExtensions.CrpcAddMethod(s_calibrationCb, "get-stereo-calibration");
```

## 2. `rpc_string` ownership rules

The C API (`server_api.h`) uses `rpc_string*` in both directions with
**different ownership**:

| Pointer | Owner | C# obligation |
|---|---|---|
| Handler **input** (`params` string) | C++ — a stack object inside `RpcServer::process` | Copy the data out immediately. **Never** call `crpc_str_destroy` on it. |
| Handler **return** value | Transfers to C++ — `rpc_server.cpp` destroys it after parsing | Create with `crpc_str_create`, return the handle, and **relinquish** C#-side ownership. |

Both rules are violated by the current `RpcTest.cs`:
it `Dispose()`s the input handle (destroying an object it doesn't own) and
returns a handle whose C# finalizer will later call `crpc_str_destroy` on
memory C++ has already freed — a double-destroy race.

Spec: split the wrapper into two types.

```csharp
/// Non-owning view over a C++-owned rpc_string. No finalizer, no Dispose.
readonly ref struct BorrowedRpcString { /* Data property only */ }

/// C#-created rpc_string. Dispose() destroys it; Release() transfers
/// ownership (nulls the handle and calls GC.SuppressFinalize).
sealed class OwnedRpcString : IDisposable
{
    public IntPtr Release() { var h = _handle; _handle = IntPtr.Zero;
                              GC.SuppressFinalize(this); return h; }
}
```

Handlers end with `return new OwnedRpcString(json).Release();`.

## 3. Main-thread dispatcher

RPC callbacks arrive on a C++ per-client thread; Unity APIs are
main-thread-only. The callback must block until the main thread produces the
result. The `task.Wait()` idea in `RpcTest.cs` is directionally right but must
be hardened (`Assets/Scripts/CloudPointRpc/MainThreadDispatcher.cs`):

- Per request, create
  `new TaskCompletionSource<string>(TaskCreationOptions.RunContinuationsAsynchronously)`
  and enqueue `(Func<string> handler, tcs)` into a `ConcurrentQueue`.
- `Update()` drains the queue: run the handler, `tcs.TrySetResult(json)`
  (or `TrySetException`).
- The callback thread waits with a **bounded timeout**:
  `tcs.Task.Wait(TimeoutMs)` with ~5000 ms. On timeout, return a JSON string
  with an `"error"` field (the C++ side passes the string through as the
  result). An unbounded wait deadlocks permanently when the editor is paused
  or the component is destroyed.
- Note that `server_api.cpp` holds a global `server_mtx` while dispatching, so
  **one stuck handler stalls every client** — another reason the timeout is
  mandatory.

### Shutdown ordering (deadlock hazard)

`crpc_deinit()` joins server threads. A server thread may be blocked inside a
callback waiting for the main thread — which is the thread calling
`crpc_deinit()`. Required order in `OnDestroy`:

1. Set a `volatile bool _shuttingDown` — new callbacks fail-fast with an error
   result; pending queue entries get `TrySetCanceled()`.
2. Drain/clear the queue.
3. Only then call `crpc_deinit()`.

## 4. Required handlers

### `get-stereo-calibration`

Derive intrinsics from the rendering camera, extrinsics from the two camera
transforms (schema in API.md):

- `fx = width / (2 * tan(hFov / 2))`, `fy = height / (2 * tan(vFov / 2))`
  where `vFov = Camera.fieldOfView` (degrees → radians) and
  `hFov = 2 * atan(tan(vFov/2) * aspect)`. Use the render-texture resolution,
  not the screen resolution.
- `cx = width / 2`, `cy = height / 2`, skew 0. `dist_coeffs` = five zeros
  (ideal pinhole).
- Extrinsics: right camera relative to left in OpenCV convention
  (`x_r = R·x_l + T`), converted per §5. For the standard parallel rig this is
  `R = I`, `T = [-baseline, 0, 0]` with
  `baseline = Vector3.Distance(left.position, right.position)` in meters.
- `image_size` = the RenderTexture size served by `get-image-pair`.

### `get-image-pair`

Both eyes must be captured on the **same rendered frame**:

- Give both cameras `targetTexture` RenderTextures; inside the (main-thread)
  handler call `leftCam.Render(); rightCam.Render();` then read both back.
- Readback: synchronous `Texture2D.ReadPixels` inside the handler is
  acceptable and simplest — the RPC thread is blocked waiting anyway.
  `AsyncGPUReadback` is the documented optimization: issue the request in the
  handler and complete the `TaskCompletionSource` from the readback callback
  (the dispatcher design above already supports deferred completion).
- **Vertical flip**: GPU readbacks are bottom-up; the wire format is
  top-left-origin row-major (API.md). Flip rows before encoding.
- Encode with `Convert.ToBase64String`; fill the `get-image-pair` schema with
  `type: "RGBA"` (or convert to BGR to save 25% payload).
- Include a monotonically increasing `frame` counter
  (`Time.frameCount` is fine).

### Legacy methods

`get-intrinsic-params` / `get-extrinsic-params` may be kept, backed by the
left camera, for backward compatibility. They are not used by
`CloudPointClient`.

## 5. Coordinate conventions

- Unity: left-handed, +Y up, +Z forward. OpenCV camera frame: right-handed,
  +X right, **+Y down**, +Z forward.
- Conversion with `S = diag(1, -1, 1)`:
  `R_cv = S · R_unity · S`, `t_cv = S · t_unity`.
- The reconstructed cloud is in the **OpenCV left-camera frame**. Consumers
  that need Unity world space must apply the inverse conversion plus the left
  camera pose; if needed, add an optional `left_pose` (16 doubles, row-major
  4x4) field to `get-stereo-calibration`.
- All matrices row-major flattened; all lengths in meters (1 Unity unit = 1 m);
  disparity in pixels.

## 6. Performance notes

- 1920x1080 RGBA is 8.3 MB raw ≈ 11 MB base64 per eye, ~22 MB per
  `get-image-pair` — of the order 100–300 ms per fetch on loopback. For
  interactive rates use 960x540 and/or BGR.
- `ReadPixels` stalls the GPU pipeline (~1–5 ms at 1080p): fine for on-demand
  capture; do not capture every frame unconditionally.
- Base64 of large buffers allocates heavily; reuse buffers
  (`Convert.TryToBase64Chars`) where practical.
- The server serializes requests globally (`server_mtx`), so handlers need no
  reentrancy protection — but a slow handler blocks all clients (see §3).

## 7. Verification checklist

1. Start the Unity scene (server on the configured port).
2. From this repo: `./build/src/cloud_point_rpc_cli config.yaml`, option
   `4` (compute-cloud) — expect a plausible point count and bounding box.
3. Option `5` writes a PLY; inspect it in MeshLab against the visible scene
   geometry (a plane at 1.5 m should reconstruct at z ≈ 1.5).
4. Kill the client mid-request and re-connect — the server must keep serving
   (per-client threads are independent; the TCP loop tolerates EOF).
