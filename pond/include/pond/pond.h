#pragma once

#include <stdint.h>
#include <stddef.h>

typedef enum pond_topic_assertion
{
    POND_TOPIC_ASSERTION_SAME_THREAD,
    POND_TOPIC_ASSERTION_SINGLE_RECEIVER,
    POND_TOPIC_ASSERTION_SIGNLE_DISTRIBUTER,
} pond_topic_assertion;

typedef struct pond_module_api pond_module_api;

typedef void (*pfn_pond_receiver_callback)(pond_module_api* api, void* callback_pointer, void* data);

typedef void (*pfn_pond_module_shutdown)(uint32_t ctx);
typedef void (*pfn_pond_module_set_user_ptr)(uint32_t ctx, void* ptr);
typedef void* (*pfn_pond_module_get_user_ptr)(uint32_t ctx);

typedef int32_t (*pfn_pond_create_distributor)(uint32_t ctx, uint8_t* topic);
typedef void (*pfn_pond_destroy_distributor)(uint32_t ctx, uint32_t distributor);
typedef void (*pfn_pond_distribute)(uint32_t ctx, uint32_t distributor, void* data);

typedef int32_t (*pfn_pond_create_receiver)(uint32_t ctx, uint8_t* topic, pfn_pond_receiver_callback callback, void* callback_pointer);
typedef void (*pfn_pond_destroy_receiver)(uint32_t ctx, uint32_t receiver);

typedef struct pond_module_api
{
    uint32_t ctx;
    
    pfn_pond_module_shutdown module_shutdown;
    pfn_pond_module_set_user_ptr module_set_user_ptr;
    pfn_pond_module_get_user_ptr module_get_user_ptr;

    pfn_pond_create_distributor create_distributor;
    pfn_pond_destroy_distributor destroy_distributor;
    pfn_pond_distribute distribute;

    pfn_pond_create_receiver create_receiver;
    pfn_pond_destroy_receiver destroy_receiver;
} pond_module_api;

typedef enum pond_result
{
    POND_SUCCESS,
    POND_ERROR,
} pond_result;

extern "C"
{
    pond_result pond_module_on_startup(pond_module_api* api);
    void pond_module_on_shutdown(pond_module_api* api);

    pond_result pond_module_on_activate(pond_module_api* api);
    void pond_module_on_deactivate(pond_module_api* api);

    void pond_module_on_frame(pond_module_api* api);
};