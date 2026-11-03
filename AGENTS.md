# Cloud Point RPC Agent Guide

This repository contains a C++23 implementation of a JSON RPC protocol for communicating with a Unity Scene.
Agents working on this codebase must adhere to the following guidelines and conventions.

## 1. Build, Lint, and Test

The project uses the **Meson** build system.

### Build Commands
- **Setup Build Directory:**
  ```bash
  meson setup build
  ```
- **Compile:**
  ```bash
  meson compile -C build
  ```
- **Clean:**
  ```bash
  meson compile --clean -C build
  ```

### Testing
- **Run All Tests:**
  ```bash
  meson test -C build
  ```
- **Run Specific Test:**
  To run a single test suite or case, use the test name defined in `meson.build`:
  ```bash
  meson test -C build <test_name>
  ```
  *Example: `meson test -C build unit_tests`*

- **Verbose Output:**
  ```bash
  meson test -C build -v
  ```

### Linting & Formatting
- **Format Code:**
  Use `clang-format` with the project's configuration (assumed Google style if not present).
  ```bash
  ninja -C build clang-format
  # OR manual execution
  find src include tests -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
  ```
- **Static Analysis:**
  If configured, run clang-tidy:
  ```bash
  ninja -C build clang-tidy
  ```

## 2. Code Style & Conventions

Adhere strictly to **Modern C++23** standards.

### General Guidelines
- **Standard:** C++23. Use concepts, ranges, `std::expected`, and smart pointers. Avoid raw `new`/`delete`.
- **Memory Management:** Use `std::unique_ptr` and `std::shared_ptr`.
- **Const Correctness:** Use `const` (and `constexpr`/`consteval`) whenever possible.
- **Includes:** Use absolute paths for project headers (e.g., `#include "rpc/server.hpp"`).
- **Auto:** Use `auto` when the type is obvious from the right-hand side or is a complex template type.

### Naming Conventions
- **Files:** `snake_case.cpp`, `snake_case.hpp`.
- **Classes/Structs:** `PascalCase`.
- **Functions/Methods:** `snake_case`.
- **Variables:** `snake_case`.
- **Private Members:** `snake_case_` (trailing underscore).
- **Constants:** `kPascalCase` or `ALL_CAPS` for macros.
- **Namespaces:** `score` (primary project namespace).
- **Interfaces:** `IPascalCase`.

### Project Structure
- `include/cloud_point_rpc/`: Public header files (RPC server/client, TCP, config, serialization, coder, DTOs).
- `include/cloud_point/`: OpenCV compute library headers (StereoRectifier, PointCloudBuilder, CloudPointClient). Optional; requires opencv4.
- `src/`: Implementation files and executable entrypoints.
- `src/cloud_point/`: OpenCV compute library implementation (optional, requires opencv4).
- `tests/`: Unit and integration tests.
- `subprojects/`: Meson wrap files for dependencies.
- `meson.build`: Build configuration.

### Error Handling
- Use **Exceptions** (`std::runtime_error`, `std::invalid_argument`) for invariant violations and logical errors.
- Use **`std::optional`** or **`std::expected`** for recoverable runtime errors.
- **JSON RPC Errors:** Ensure all RPC handlers catch exceptions and return valid JSON-RPC error objects containing `code`, `message`, and `data`.

### Documentation
- Use **Doxygen** style comments for public APIs.
- Document thread safety guarantees.

### Example Class

```cpp
namespace score {

/// @brief Manages camera parameters.
class CameraController {
 public:
  struct Config {
    int width;
    int height;
  };

  explicit CameraController(const Config& config);
  
  /// @brief Retrieves current intrinsic parameters.
  /// @return A 3x3 matrix.
  [[nodiscard]] std::vector<double> get_intrinsic_params() const;

 private:
  Config config_;
  mutable std::mutex mutex_;
  std::vector<double> cached_intrinsics_;
};

} // namespace score
```

### Implementation Details
- **JSON Library:** Use `nlohmann/json` (likely via `subprojects/nlohmann_json.wrap`).
- **Concurrency:** Use `std::jthread` (auto-joining) over `std::thread`.
- **Error handling:** Use `std::expected<T, E>` (C++23) for recoverable errors in the `cloud_point` compute library.
- **RPC Methods** (served by Unity or the C++ mock in `server_main.cpp`):
  - `get-available-methods` — list registered method names.
  - `get-stereo-calibration` — full stereo rig calibration (intrinsics, R, T, image size).
  - `get-image-pair` — synchronised stereo frame as two base64-encoded images.
  - `get-intrinsic-params` *(legacy)* — left-camera intrinsic matrix (9 doubles).
  - `get-extrinsic-params` *(legacy)* — left-camera extrinsic matrix (16 doubles).
  - Ensure thread safety if the RPC server is multi-threaded.

## 3. Workflow & Git

### Commit Messages
- Use conventional commits format: `<type>(<scope>): <subject>`
  - Types: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`.
  - Example: `feat(rpc): add handler for get-cloud-point`

### Pull Requests
- Ensure `meson test -C build` passes before requesting review.
- Keep PRs small and focused on a single logical change.

## 4. Cursor & Copilot Rules

*(No specific rules found in .cursor/rules/ or .github/copilot-instructions.md.)*

- **Context:** Always read related header files before modifying source files.
- **Verification:** Write unit tests for new features in `tests/`.
- **Refactoring:** When refactoring, ensure existing behavior is preserved via tests.
- **Dependencies:** Do not introduce new dependencies without updating `meson.build` and `subprojects/`.

## OpenWiki

This repository has documentation located in the /openwiki directory.

Start here:
- [OpenWiki quickstart](openwiki/quickstart.md)

OpenWiki includes repository overview, architecture notes, workflows, domain concepts, operations, integrations, testing guidance, and source maps.

When working in this repository, read the OpenWiki quickstart first, then follow its links to the relevant architecture, workflow, domain, operation, and testing notes.

<!-- OPENWIKI:START -->

## OpenWiki

This repository has a generated `openwiki/` evidence index. It is optional just-in-time context, not required startup reading.

- Treat source code and tests as authoritative. A brief's unknowns and review items are verification gaps, not automatic requirements.
- Prefer the narrowest quiet validation that proves the changed behavior. Preserve complete failure output.

The scheduled OpenWiki GitHub Actions workflow refreshes the repository wiki. Do not hand-edit generated OpenWiki pages unless explicitly asked; prefer updating source code/docs and letting OpenWiki regenerate.

<!-- OPENWIKI:END -->
