#ifndef CRPC_SERVER_API
#define CRPC_SERVER_API

#include <cstdint>
#include "export.h"


#ifdef __cplusplus
extern "C" {
#endif //cpp
/**
 * @brief basically just std::string wrapper:
 * struct rpc_string {
 *   std::string s;
 * };
 * has internal gc and would be automatically deallocated on deinit call,
 * but it is better to call destroy manually, to prevent exceeding memory usage
 */
struct CRPC_EXPORT rpc_string;

CRPC_EXPORT const char* crpc_str_get_data(const rpc_string*);
CRPC_EXPORT uint64_t crpc_str_get_size(const rpc_string*);

CRPC_EXPORT rpc_string* crpc_str_create(const char* data, uint64_t size);
CRPC_EXPORT void crpc_str_destroy(rpc_string*);

typedef rpc_string*(*callback_t)(rpc_string*);

CRPC_EXPORT void crpc_init(const char* config_path);
CRPC_EXPORT void crpc_init_with_address(const char* ip, int port);
CRPC_EXPORT void crpc_deinit();

CRPC_EXPORT rpc_string* crpc_get_method_name_by_id(uint64_t id);
CRPC_EXPORT uint64_t crpc_get_methods_count();
CRPC_EXPORT void crpc_add_method(callback_t cb, rpc_string* name);

#ifdef __cplusplus
}
#endif //cpp

#endif //CRPC_SERVER_API