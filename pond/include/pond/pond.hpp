#pragma once

#include <stdint.h>
#include <string>
#include <memory>
#include <functional>
#include "ring_buffer_queue.hpp"
#include "pond.h"

namespace pond
{
    class ManagedMessage
    {
    public:
        virtual ~ManagedMessage() = default;
        virtual ManagedMessage* clone() const = 0;
    };

    class Distributor
    {
    friend class ModuleBase;
    public:
        void destroy();
        void distribute(void* data);
    private:
        int32_t _id;
        pond_api _pond_api; 
    };

    class Receiver
    {
    friend class ModuleBase;
    public:
        void destroy();
    private:
        int32_t _id;
        pond_api _pond_api;
        std::unique_ptr<std::function<void(void*)>> _callback;
    };

    class ModuleBase
    {
    public:
        virtual pond_result onStartup();
        virtual void onShutdown();
        virtual pond_result onActivate();
        virtual void onDeactivate();
        virtual void onFrame();
        void shutdown();
        Distributor createDistributor(const std::string& topic);
        Receiver createReceiver(const std::string& topic, std::function<void(void*)> callback);

        pond_api _pond_api; 
    };
}

#define POND_LOG(format, ...) _pond_api.log(_pond_api.ctx, (uint8_t*)format, ##__VA_ARGS__)

#define POND_MODULE_CPP_DECLARE(cls, _name, _info)\
pond_result pond_module_##cls##_on_startup(pond_api* api)\
{\
    cls* module = new cls();\
    api->module_set_user_ptr(api->ctx, module);\
    module->_pond_api = *api;\
    return module->onStartup();\
}\
\
void pond_module_##cls##_on_shutdown(pond_api* api)\
{\
    cls* module = (cls*)api->module_get_user_ptr(api->ctx);\
    module->onShutdown();\
    delete module;\
}\
\
pond_result pond_module_##cls##_on_activate(pond_api* api)\
{\
    cls* module = (cls*)api->module_get_user_ptr(api->ctx);\
    return module->onActivate();\
}\
\
void pond_module_##cls##_on_deactivate(pond_api* api)\
{\
    cls* module = (cls*)api->module_get_user_ptr(api->ctx);\
    module->onDeactivate();\
}\
\
void pond_module_##cls##_on_frame(pond_api* api)\
{\
    cls* module = (cls*)api->module_get_user_ptr(api->ctx);\
    module->onFrame();\
}\
POND_MODULE_DECLARE(cls, _name, _info)

#define POND_MODULE_CPP_MAKE_IMPLEMENTATION() \
namespace pond\
{\
\
pond_result ModuleBase::onStartup() {return POND_SUCCESS;}\
void ModuleBase::onShutdown() {}\
pond_result ModuleBase::onActivate() {return POND_SUCCESS;}\
void ModuleBase::onDeactivate() {}\
void ModuleBase::onFrame() {}\
\
void ModuleBase::shutdown()\
{\
    _pond_api.module_shutdown(_pond_api.ctx);\
}\
\
Distributor ModuleBase::createDistributor(const std::string& topic)\
{\
    Distributor d;\
    d._pond_api = _pond_api;\
    d._id = _pond_api.create_distributor(_pond_api.ctx, (uint8_t*)topic.c_str());\
    return d;\
}\
\
void Distributor::destroy()\
{\
    _pond_api.destroy_distributor(_pond_api.ctx, _id);\
}\
\
void Distributor::distribute(void* data)\
{\
    _pond_api.distribute(_pond_api.ctx, _id, data);\
}\
\
void pond_cpp_callback(pond_api* api, void* callback_pointer, void* data)\
{\
    auto* callback = static_cast<std::function<void(void*)>*>(callback_pointer);\
    (*callback)(data);\
}\
\
Receiver ModuleBase::createReceiver(const std::string& topic, std::function<void(void*)> callback)\
{\
    Receiver r;\
    r._pond_api = _pond_api;\
    r._callback = std::make_unique<std::function<void(void*)>>(std::move(callback));\
    r._id = _pond_api.create_receiver(_pond_api.ctx, (uint8_t*)topic.c_str(), pond_cpp_callback, r._callback.get());\
    return r;\
}\
\
void Receiver::destroy()\
{\
    _pond_api.destroy_receiver(_pond_api.ctx, _id);\
    _callback.reset();\
}\
}
