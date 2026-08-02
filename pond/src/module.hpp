#pragma once

#include <pond/pond.h>
#include <set>

class module
{
    void* lib_handle;
    pond_api native_api;
    struct
    {
        pond_result (*pond_module_on_startup)(pond_api* api);
        void (*pond_module_on_shutdown)(pond_api* api);
        pond_result (*pond_module_on_activate)(pond_api* api);
        void (*pond_module_on_deactivate)(pond_api* api);
        void (*pond_module_on_frame)(pond_api* api);
    } module_api;
    std::set<int32_t> distributors;
    std::set<int32_t> receivers;
};