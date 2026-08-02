#pragma once

#include <stdint.h>
#include <stddef.h>

typedef enum pond_topic_assertion
{
    POND_TOPIC_ASSERTION_SAME_THREAD,
    POND_TOPIC_ASSERTION_SINGLE_RECEIVER,
    POND_TOPIC_ASSERTION_SIGNLE_DISTRIBUTER,
} pond_topic_assertion;

typedef enum pond_result
{
    POND_SUCCESS,
    POND_ERROR,
} pond_result;

typedef struct pond_api pond_api;

typedef void (*pfn_pond_receiver_callback)(pond_api* api, void* callback_pointer, void* data);

typedef void (*pfn_pond_module_shutdown)(uint32_t ctx);
typedef void (*pfn_pond_module_log)(uint32_t ctx, uint8_t* format, ...);
typedef void (*pfn_pond_module_set_user_ptr)(uint32_t ctx, void* ptr);
typedef void* (*pfn_pond_module_get_user_ptr)(uint32_t ctx);

typedef int32_t (*pfn_pond_create_distributor)(uint32_t ctx, uint8_t* topic);
typedef void (*pfn_pond_destroy_distributor)(uint32_t ctx, uint32_t distributor);
typedef void (*pfn_pond_distribute)(uint32_t ctx, uint32_t distributor, void* data);

typedef int32_t (*pfn_pond_create_receiver)(uint32_t ctx, uint8_t* topic, pfn_pond_receiver_callback callback, void* callback_pointer);
typedef void (*pfn_pond_destroy_receiver)(uint32_t ctx, uint32_t receiver);

typedef struct pond_api
{
    uint32_t ctx;
    
    pfn_pond_module_shutdown module_shutdown;
    pfn_pond_module_log log;
    pfn_pond_module_set_user_ptr module_set_user_ptr;
    pfn_pond_module_get_user_ptr module_get_user_ptr;

    pfn_pond_create_distributor create_distributor;
    pfn_pond_destroy_distributor destroy_distributor;
    pfn_pond_distribute distribute;

    pfn_pond_create_receiver create_receiver;
    pfn_pond_destroy_receiver destroy_receiver;
} pond_api;

typedef pond_result (*pfn_pond_module_on_startup)(pond_api* api);
typedef void (*pfn_pond_module_on_shutdown)(pond_api* api);

typedef pond_result (*pfn_pond_module_on_activate)(pond_api* api);
typedef void (*pfn_pond_module_on_deactivate)(pond_api* api);

typedef void (*pfn_pond_module_on_frame)(pond_api* api);

typedef struct pond_module_metadata
{
    uint8_t* name;
    uint8_t* info;
    pfn_pond_module_on_startup on_startup;
    pfn_pond_module_on_shutdown on_shutdown;
    pfn_pond_module_on_activate on_activate;
    pfn_pond_module_on_deactivate on_deactivate;
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
    .on_activate = pond_module_##_code_prefix##_on_activate,\
    .on_deactivate = pond_module_##_code_prefix##_on_deactivate,\
    .on_frame = pond_module_##_code_prefix##_on_frame,\
};

typedef struct pond_bundle_metadata
{
    uint8_t* info;
    uint32_t module_count;
    pond_module_metadata modules[];
} pond_bundle_metadata;

#if defined(_WIN32)
    #define POND_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
    #define POND_EXPORT __attribute__((visibility("default")))
#else
    #define POND_EXPORT
#endif

#define POND_BUNDLE_DECLARE(_info, _count, ...)\
POND_EXPORT pond_bundle_metadata pond_bundle_metadata_ = {\
    .info = (uint8_t*)_info,\
    .module_count = _count,\
    .modules = {__VA_ARGS__},\
};
