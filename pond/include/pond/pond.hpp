#pragma once

#include "hpp/module_base.hpp"

#include <thread>
#include <chrono>

namespace pond
{
    inline double get_time() {return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }
    inline void sleep(double duration) {std::this_thread::sleep_for(std::chrono::duration<double>(duration));}
}

#define POND_MODULE_CPP_DECLARE(cls, _name, _info)\
pond_result pond_module_##cls##_on_startup(pond_api* api, uint32_t argc, void** argv)\
{\
    cls* module = new cls();\
    api->set_user_ptr(api->ctx, module);\
    module->init(api);\
    std::vector<void*> args(argc);\
    for (uint32_t i = 0; i < argc; i++) args[i] = argv[i];\
\
    return module->onStartup(args);\
}\
\
void pond_module_##cls##_on_shutdown(pond_api* api)\
{\
    cls* module = (cls*)api->get_user_ptr(api->ctx);\
    module->onShutdown();\
    delete module;\
}\
\
void pond_module_##cls##_on_frame(pond_api* api)\
{\
    cls* module = (cls*)api->get_user_ptr(api->ctx);\
    module->onFrame();\
}\
POND_MODULE_DECLARE(cls, _name, _info)
