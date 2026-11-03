#pragma once
#if defined(_WIN32)
#ifdef CRPC_SERVER_API_EXPORT
#define CRPC_EXPORT __declspec(dllexport)
#else
#define CRPC_EXPORT __declspec(dllimport)
#endif
#elif defined(__GNUC__) // GCC, Clang, etc.
  // Linux, macOS, etc.
  #define CRPC_EXPORT __attribute__((visibility("default")))
#else
  #define CRPC_EXPORT
  #pragma warning Unknown dynamic link import/export semantics.
#endif