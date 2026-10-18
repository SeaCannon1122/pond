#pragma once

#include <stdint.h>

struct pond_api;

typedef enum pond_result
{
    POND_SUCCESS,
    POND_ERROR,
} pond_result;

typedef pond_result (*pfn_pond_module_on_startup)(pond_api* api, uint32_t argc, void** argv);
typedef void (*pfn_pond_module_on_shutdown)(pond_api* api);
typedef void (*pfn_pond_module_on_frame)(pond_api* api);

typedef struct pond_module_metadata
{
    uint8_t* name;
    uint8_t* info;
    pfn_pond_module_on_startup on_startup;
    pfn_pond_module_on_shutdown on_shutdown;
    pfn_pond_module_on_frame on_frame;
} pond_module_metadata;

#define POND_MODULE(_code_prefix) pond_module_##_code_prefix##_metadata
#define EXTERN_POND_MODULE(_code_prefix) extern pond_module_metadata POND_MODULE(_code_prefix)

#define POND_MODULE_DECLARE(_code_prefix, _name, _info) \
pond_module_metadata POND_MODULE(_code_prefix) = {\
    .name = (uint8_t*)_name,\
    .info = (uint8_t*)_info,\
    .on_startup = pond_module_##_code_prefix##_on_startup,\
    .on_shutdown = pond_module_##_code_prefix##_on_shutdown,\
    .on_frame = pond_module_##_code_prefix##_on_frame,\
};

typedef struct pond_bundle_metadata
{
    uint8_t* info;
    uint32_t module_count;
    pond_module_metadata* modules;
} pond_bundle_metadata;

#if defined(_WIN32)
    #define POND_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
    #define POND_EXPORT __attribute__((visibility("default")))
#else
    #define POND_EXPORT
#endif

#define POND_BUNDLE_DECLARE(_info, ...)\
pond_module_metadata pond_bundle_module_metadata_list[] = {__VA_ARGS__};\
POND_EXPORT pond_bundle_metadata pond_bundle_metadata_ = {\
    .info = (uint8_t*)_info,\
    .module_count = sizeof(pond_bundle_module_metadata_list) / sizeof(pond_bundle_module_metadata_list[0]),\
    .modules = pond_bundle_module_metadata_list\
};
