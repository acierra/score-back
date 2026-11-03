#ifndef CRPC_TEST_API
#define CRPC_TEST_API

#include <cstdint>
#include "export.h"
#ifdef __cplusplus
extern "C" {
#endif //cpp

struct CRPC_EXPORT rpc_string;

/// @brief callback function intendent to handle json and retrieve string data
typedef rpc_string*(*callback_t)(rpc_string*);

/// @brief call it to initialize library thread;
/// by default would call available methods one by one
CRPC_EXPORT void crpc_test_init();

/// @brief please, call it, when you're done working with the lib
/// @note blocking operation
CRPC_EXPORT void crpc_test_deinit();

/// @brief crpc_test_add_method will add to the available set/map
/// @note blocking operation
CRPC_EXPORT void crpc_test_add_method(callback_t cb, rpc_string* name);

/// @brief crpc_test_remove_method would
/// @param name pointer to name string; memory management not transferred
CRPC_EXPORT int crpc_test_remove_method(rpc_string* name);

/// @brief crpc_schedule_call would add to the execution queue
/// may be called one after current
/// @note blocking operation
CRPC_EXPORT void crpc_test_schedule_call(rpc_string* name);

/// @brief crpc_test_change_duration will change duration between regular calls
/// @note blocking operation
CRPC_EXPORT void crpc_test_change_duration(uint64_t duration_ms);

/// @note blocking operation
CRPC_EXPORT uint64_t crpc_test_duration();

/// @brief crpc_test_auto_call change internal state of auto call
/// @param state 0 - turn off auto call on sleep; 1 - turn on auto call on sleep
CRPC_EXPORT void crpc_test_auto_call(uint32_t state);
#ifdef __cplusplus
}
#endif //cpp

#endif //CRPC_SERVER_API