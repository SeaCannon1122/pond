#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

typedef enum pond_result
{
    POND_SUCCESS,
    POND_ERROR,
} pond_result;

typedef enum pond_parameter_type
{
    POND_PARAMETER_DONT_CARE,
    POND_PARAMETER_STRING,
    POND_PARAMETER_STRING_ARRAY,
    POND_PARAMETER_INT,
    POND_PARAMETER_INT_ARRAY,
    POND_PARAMETER_DOUBLE,
    POND_PARAMETER_DOUBLE_ARRAY,
    POND_PARAMETER_BOOL,
    POND_PARAMETER_BOOL_ARRAY
} pond_parameter_type;

typedef struct pond_parameter
{
    pond_parameter_type type;
    uint32_t array_length;
    union
    {
        uint8_t* String;
        uint8_t** StringArray;
        int Int;
        int* IntArray;
        double Double;
        double* DoubleArray;
        bool Bool;
        bool* BoolArray;
    } value;
} pond_parameter;

static pond_parameter* pond_malloc_parameter_int(int32_t value)
{
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter));
    p->type = POND_PARAMETER_INT;
    p->value.Int = value;
    return p;
}

static pond_parameter* pond_malloc_parameter_double(double value)
{
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter));
    p->type = POND_PARAMETER_DOUBLE;
    p->value.Double = value;
    return p;
}

static pond_parameter* pond_malloc_parameter_bool(bool value)
{
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter));
    p->type = POND_PARAMETER_BOOL;
    p->value.Bool = value;
    return p;
}

static pond_parameter* pond_malloc_parameter_string(uint8_t* value)
{
    size_t size = strlen((char*)value) + 1;
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter) + size);
    p->type = POND_PARAMETER_STRING;
    p->value.String = (uint8_t*)p + sizeof(pond_parameter);
    memcpy(p->value.String, value, size);
    return p;
}

static pond_parameter* pond_malloc_parameter_int_array(int32_t* value, uint32_t length)
{
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter) + sizeof(int32_t) * length);
    p->type = POND_PARAMETER_INT_ARRAY;
    p->array_length = length;
    p->value.IntArray = (int32_t*)((uint8_t*)p + sizeof(pond_parameter));
    memcpy(p->value.IntArray, value, sizeof(int32_t) * length);
    return p;
}

static pond_parameter* pond_malloc_parameter_double_array(double* value, uint32_t length)
{
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter) + sizeof(double) * length);
    p->type = POND_PARAMETER_DOUBLE_ARRAY;
    p->array_length = length;
    p->value.DoubleArray = (double*)((uint8_t*)p + sizeof(pond_parameter));
    memcpy(p->value.DoubleArray, value, sizeof(double) * length);
    return p;
}

static pond_parameter* pond_malloc_parameter_bool_array(bool* value, uint32_t length)
{
    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter) + sizeof(bool) * length);
    p->type = POND_PARAMETER_BOOL_ARRAY;
    p->array_length = length;
    p->value.BoolArray = (bool*)((uint8_t*)p + sizeof(pond_parameter));
    memcpy(p->value.BoolArray, value, sizeof(bool) * length);
    return p;
}

static pond_parameter* pond_malloc_parameter_string_array(uint8_t** value, uint32_t length)
{
    size_t total_size = 0;
    for (uint32_t i = 0; i < length; i++) total_size += strlen((char*)value[i]) + 1;

    pond_parameter* p = (pond_parameter*)malloc(sizeof(pond_parameter) + sizeof(uint8_t*) * length + total_size);
    p->type = POND_PARAMETER_STRING_ARRAY;
    p->array_length = length;
    p->value.StringArray = (uint8_t**)((uint8_t*)p + sizeof(pond_parameter));

    for (uint32_t i = 0, offset = 0; i < length; i++)
    {
        p->value.StringArray[i] = (uint8_t*)((uint8_t*)p + sizeof(pond_parameter) + sizeof(uint8_t*) * length + offset);
        size_t len = strlen((char*)value[i])+1;
        memcpy(p->value.StringArray[i], value[i], len);
        offset += len;
    }

    return p;
}

typedef struct pond_api pond_api;

typedef struct pond_dds_slot_info
{
    uint8_t* topic;
    uint8_t* type;
} pond_dds_slot_info;

typedef void (*pfn_pond_receiver_callback)(pond_api* api, void* callback_pointer, void** slot_data);

typedef void (*pfn_pond_shutdown)(void* ctx);
typedef void (*pfn_pond_log)(void* ctx, uint8_t* format, ...);
typedef void (*pfn_pond_set_user_ptr)(void* ctx, void* ptr);
typedef void* (*pfn_pond_get_user_ptr)(void* ctx);

typedef void (*pfn_pond_set_parameter)(void* ctx, uint8_t* name, pond_parameter* parameter);
typedef pond_parameter* (*pfn_pond_get_parameter)(void* ctx, uint8_t* name);

typedef int32_t (*pfn_pond_create_distributor)(void* ctx, pond_dds_slot_info* slots, uint32_t slot_count);
typedef void (*pfn_pond_destroy_distributor)(void* ctx, uint32_t distributor);
typedef void (*pfn_pond_distribute)(void* ctx, uint32_t distributor, void** slot_data);

typedef int32_t (*pfn_pond_create_receiver)(void* ctx, pond_dds_slot_info* slots, uint32_t slot_count, pfn_pond_receiver_callback callback, void* callback_pointer);
typedef void (*pfn_pond_destroy_receiver)(void* ctx, uint32_t receiver);

typedef struct pond_api
{
    void* ctx;
    
    pfn_pond_shutdown shutdown;
    pfn_pond_log log;
    pfn_pond_set_user_ptr set_user_ptr;
    pfn_pond_get_user_ptr get_user_ptr;

    pfn_pond_set_parameter set_parameter;
    pfn_pond_get_parameter get_parameter;

    pfn_pond_create_distributor create_distributor;
    pfn_pond_destroy_distributor destroy_distributor;
    pfn_pond_distribute distribute;

    pfn_pond_create_receiver create_receiver;
    pfn_pond_destroy_receiver destroy_receiver;
} pond_api;

typedef pond_result (*pfn_pond_module_on_startup)(pond_api* api);
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
