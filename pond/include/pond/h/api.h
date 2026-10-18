#pragma once

#include <stdint.h>

struct pond_parameter;
typedef struct pond_api pond_api;

typedef struct pond_dds_slot_info
{
    uint8_t* channel;
    uint8_t* type;
    void* data_ptr;
} pond_dds_slot_info;

typedef void (*pfn_pond_shutdown)(void* ctx);
typedef void (*pfn_pond_log)(void* ctx, uint8_t* format, ...);
typedef void (*pfn_pond_set_user_ptr)(void* ctx, void* ptr);
typedef void* (*pfn_pond_get_user_ptr)(void* ctx);

typedef void (*pfn_pond_set_parameter)(void* ctx, uint8_t* name, pond_parameter* parameter);
typedef pond_parameter* (*pfn_pond_get_parameter)(void* ctx, uint8_t* name);

typedef int32_t (*pfn_pond_create_distributor)(void* ctx, pond_dds_slot_info* slots, uint32_t slot_count);
typedef void (*pfn_pond_destroy_distributor)(void* ctx, uint32_t distributor);
typedef void (*pfn_pond_distribute)(void* ctx, uint32_t distributor, void** slot_data);

typedef void (*pfn_pond_callback_function)(pond_api* api, void* callback_pointer, void** slot_data);

typedef int32_t (*pfn_pond_create_callback)(void* ctx, pond_dds_slot_info* slots, uint32_t slot_count, pfn_pond_callback_function callback, void* callback_pointer);
typedef void (*pfn_pond_destroy_callback)(void* ctx, uint32_t callback);

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

    pfn_pond_create_callback create_callback;
    pfn_pond_destroy_callback destroy_callback;
} pond_api;