#pragma once

#include <optional>
#include <stdint.h>
#include <string>
#include <memory>
#include <functional>
#include <unordered_map>
#include "ring_buffer_queue.hpp"
#include "pond.h"
#include <array>

namespace pond
{
    class Message
    {
    public:
        virtual ~Message() = default;
    };

    class _UntypedParameterBase
    {
    friend class ModuleBase;
    public:

    #define PARAM_FUNCTION_DECL(_type, _type_name)\
        std::optional<_type> get##_type_name(const _type& def, bool default_on_fail = true);\
        std::optional<std::vector<_type>> get##_type_name##Array(const std::vector<_type>& def, uint32_t min_length = 0, uint32_t max_length = 0, bool default_on_fail = true);\
        void set##_type_name(const _type& value);\
        void set##_type_name##Array(const std::vector<_type>& value);

        PARAM_FUNCTION_DECL(std::string, String)
        PARAM_FUNCTION_DECL(int32_t, Int)
        PARAM_FUNCTION_DECL(double, Double)
        PARAM_FUNCTION_DECL(bool, Bool)

    private:

        bool check_array_length(uint32_t length, uint32_t min, uint32_t max)
        {
            if (min == max && min != 0 && length != min)
            {
                api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s'.length != %d", name->c_str(), min);
                return false;
            }
            if (length < min)
            {
                api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s'.length < %d", name->c_str(), min);
                return false;
            }
            if (length > max && max != 0)
            {
                api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s'.length > %d", name->c_str(), max);
                return false;
            }
            return true;
        }

        _UntypedParameterBase(std::string* _name, pond_api* _api) : api(_api), name(_name) {}
        pond_api* api;
        std::string* name;
    };

    template<typename... Args>
    class Distributor
    {
    friend class ModuleBase;
    public:
        void destroy();

        void distribute(Args&... args)
        {
            void* data[] = { static_cast<void*>(&args)... };
            api->distribute(api->ctx, id, data);
        }
    private:
        int32_t id;
        pond_api* api; 
    };

    template<typename... Args>
    class Receiver
    {
    friend class ModuleBase;
    public:
        void destroy();
    private:
        int32_t id;
        pond_api* api;
        std::unique_ptr<std::function<void(Args& ...)>> callback;
        std::unique_ptr<std::function<void(void**)>> sub_callback;
    };

    void pond_cpp_callback(pond_api* api, void* callback_pointer, void** data);

    class ModuleBase
    {
    public:
        virtual pond_result onStartup();
        virtual void onShutdown();
        virtual void onFrame();
        void shutdown();

    #define CREATE_TOPIC_ARRAYS\
        std::array<std::string, sizeof...(Args)> type_name_strings = {std::string(typeid(Args).name()) + "_" + std::to_string(typeid(Args).hash_code())...};\
        std::array<uint8_t*, sizeof...(Args)> topic_names;\
        std::array<uint8_t*, sizeof...(Args)> type_names;\
        \
        for (int i = 0; i < sizeof...(Args); i++)\
        {\
            topic_names[i] = topics[i].c_str();\
            type_names[i] = type_name_strings[i].c_str();\
        }

        template<typename... Args>
        Distributor<Args...> createDistributor(const std::array<std::string, sizeof...(Args)>& topics)
        {
            CREATE_TOPIC_ARRAYS

            Distributor<Args...> d;
            d.api = &_pond_api;
            d.id = d.api->create_distributor(d.api->ctx, topic_names.data(), sizeof...(Args), type_names.data());
            return d;
        }

        template<typename... Args>
        Receiver<Args...> createReceiver(const std::array<std::string, sizeof...(Args)>& topics, std::function<void(Args& ...)> callback)
        {
            CREATE_TOPIC_ARRAYS

            Receiver<Args...> r;

            r.api = &_pond_api;
            r.callback = std::make_unique<std::function<void(Args& ...)>>(std::move(callback));
            r.sub_callback = std::make_unique<std::function<void(void**)>>([callback = &(*r.callback)](void** data){
            
                auto call_callback = [&]<std::size_t... I>(std::index_sequence<I...>) {
                    *callback(*static_cast<Args*>(data[I])...);
                };
                call_callback(std::index_sequence_for<Args...>{});
            });

            r.id = r.api->create_receiver(
                r.api->ctx, 
                topic_names.data(), 
                sizeof...(Args), 
                type_names.data(), 
                pond_cpp_callback,
                &(*r.sub_callback)
            );
            return r;
        }

        _UntypedParameterBase parameter(const std::string& name);

        pond_api _pond_api; 
    };
}

#define POND_LOG(format, ...) _pond_api.log(_pond_api.ctx, (uint8_t*)format, ##__VA_ARGS__)

#define POND_MODULE_CPP_DECLARE(cls, _name, _info)\
pond_result pond_module_##cls##_on_startup(pond_api* api)\
{\
    cls* module = new cls();\
    api->set_user_ptr(api->ctx, module);\
    module->_pond_api = *api;\
\
    return module->onStartup();\
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

#ifdef POND_MODULE_CPP_MAKE_IMPLEMENTATION
namespace pond
{

#define PARAM_GET_ARRAY_BLOCK(_type, _type_name, _caster)\
if (check_array_length(p->array_length, min_length, max_length))\
{\
    _type vec(p->array_length);\
    for (int i = 0; i < p->array_length; i++) vec[i] = _caster p->value._type_name[i];\
    return vec;\
}\

#define PARAM_GET(_type, _type_name, _pond_type_name, _success_block, ...)\
std::optional<_type> _UntypedParameterBase::get##_type_name(const _type& def, ##__VA_ARGS__ , bool default_on_fail)\
{\
    if (pond_parameter* p = api->get_parameter(api->ctx, (uint8_t*)name->c_str()))\
    {\
        if (p->type == _pond_type_name)\
        {\
            _success_block\
        }\
        else api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s'.type != '"#_type_name"'", name->c_str());\
    }\
    else if (!default_on_fail) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s' not set", name->c_str());\
\
    if (default_on_fail) return def;\
    else return std::nullopt; \
}

#define PARAM_SET_SINGLE_AND_ARRAY(_type, _type_name, _pond_type_name, _set_type, _assign_suffix)\
void _UntypedParameterBase::set##_type_name(const _type& value)\
{\
    pond_parameter p;\
    p.type = _pond_type_name;\
    p.value._type_name = (_set_type) value _assign_suffix;\
    api->set_parameter(api->ctx, (uint8_t*)name->c_str(), &p);\
}\
\
void _UntypedParameterBase::set##_type_name##Array(const std::vector<_type>& value)\
{\
    pond_parameter p;\
    p.type = _pond_type_name##_ARRAY;\
    p.value._type_name##Array = new _set_type[value.size()];\
    for (int i = 0; i < value.size(); i++) p.value._type_name##Array[i] = (_set_type)value[i] _assign_suffix;\
\
    api->set_parameter(api->ctx, (uint8_t*)name->c_str(), &p);\
\
    delete p.value._type_name##Array;\
}

#define PARAM_FUNCTION(_type, _set_type, _type_name, _pond_type_name, _caster, _assign_suffix)\
PARAM_GET(_type, _type_name, _pond_type_name, return _caster p->value._type_name;)\
PARAM_GET(std::vector<_type>, _type_name##Array, _pond_type_name##_ARRAY, PARAM_GET_ARRAY_BLOCK(std::vector<_type>, _type_name##Array, _caster), uint32_t min_length, uint32_t max_length)\
PARAM_SET_SINGLE_AND_ARRAY(_type, _type_name, _pond_type_name, _set_type, _assign_suffix)

PARAM_FUNCTION(int32_t, int32_t, Int, POND_PARAMETER_INT,,)
PARAM_FUNCTION(std::string, uint8_t*, String, POND_PARAMETER_STRING, (char*), .c_str())
PARAM_FUNCTION(double, double, Double, POND_PARAMETER_DOUBLE,,)
PARAM_FUNCTION(bool, bool, Bool, POND_PARAMETER_BOOL,,)

_UntypedParameterBase ModuleBase::parameter(const std::string& name)
{
    return _UntypedParameterBase((std::string*)&name, &_pond_api);
}

pond_result ModuleBase::onStartup() {return POND_SUCCESS;}
void ModuleBase::onShutdown() {}
void ModuleBase::onFrame() {}

void ModuleBase::shutdown()
{
    _pond_api.shutdown(_pond_api.ctx);
}

void pond_cpp_callback(pond_api* api, void* callback_pointer, void** data)
{
    auto* callback = static_cast<std::function<void(void**)>*>(callback_pointer);
    (*callback)(data);
}

}
#endif