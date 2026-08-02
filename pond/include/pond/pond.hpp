#pragma once

#include <stdint.h>
#include <string>
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
        pond_module_api _p_m_api; 
    };

    class Receiver
    {
    friend class ModuleBase;
    public:
        void destroy();
    private:
        int32_t _id;
        pond_module_api _p_m_api;
        std::function<void(void*)> _callback;
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
        Receiver createReceiver(const std::string& topic, std::function<void(void*)>);

        pond_module_api _p_m_api; 
    };

}

#define POND_MODULE_CPP_DECLARE(cls)\
namespace pond\
{\
pond_result pond_module_on_startup(pond_module_api* api)\
{\
    cls* module = new cls();\
    api->module_set_user_ptr(api->ctx, module);\
    module->_p_m_api = *api;\
    return module->onStartup();\
}\
\
void pond_module_on_shutdown(pond_module_api* api)\
{\
    cls* module = (cls*)api->module_get_user_ptr(api->ctx);\
    module->onShutdown();\
    delete module;\
}\
\
pond_result pond_module_on_activate(pond_module_api* api)\
{\
    cls* module = (cls*)api->module_get_user_ptr(api->ctx);\
    return module->onActivate();\
}\
\
void pond_module_on_deactivate(pond_module_api* api)\
{\
    cls* module = (cls*)api->module_get_user_ptr(api->ctx);\
    module->onDeactivate();\
}\
\
void pond_module_on_frame(pond_module_api* api)\
{\
    cls* module = (cls*)api->module_get_user_ptr(api->ctx);\
    module->onFrame();\
}\
\
pond_result ModuleBase::onStartup() {return POND_SUCCESS;}\
void ModuleBase::onShutdown() {}\
pond_result ModuleBase::onActivate() {return POND_SUCCESS;}\
void ModuleBase::onDeactivate() {}\
void ModuleBase::onFrame() {}\
\
void ModuleBase::shutdown()\
{\
    _p_m_api.module_shutdown(_p_m_api.ctx);\
}\
\
Distributor ModuleBase::createDistributor(const std::string& topic)\
{\
    Distributor d;\
    d._p_m_api = _p_m_api;\
    d._id = _p_m_api.create_distributor(_p_m_api.ctx, (uint8_t*)topic.c_str());\
    return d;\
}\
\
void Distributor::destroy()\
{\
    _p_m_api.destroy_distributor(_p_m_api.ctx, _id);\
}\
\
void Distributor::distribute(void* data)\
{\
    _p_m_api.distribute(_p_m_api.ctx, _id, data);\
}\
\

void pond_cpp_callback(pond_module_api* api, void* callback_pointer, void* data)
{

}

struct

Receiver ModuleBase::createReceiver(const std::string& topic, std::function<void(void*)>)\
{\
    Receiver r;\
    r._p_m_api = _p_m_api;\
    r._id = _p_m_api.create_receiver(_p_m_api.ctx, (uint8_t*)topic.c_str(), pond_cpp_callback);\
    return d;\
}\
Receiver ModuleBase::createReceiver(const std::string& topic)\
{\
    Distributor d;\
    
}\
\
void Receiver::destroy()\
{\
    _p_m_api.destroy_receiver(_p_m_api.ctx, _id);\
}\
}