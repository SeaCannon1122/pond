#pragma once

#include "pond.h"
#include <unordered_set>
#include <optional>
#include <stdint.h>
#include <string>
#include <memory>
#include <functional>
#include <type_traits>
#include <array>
#include <vector>
#include <stddef.h>

namespace pond
{

    #define _CPP_PARAM_REF(p_ptr, extra_ptr) (*(c_type*extra_ptr)((uint8_t*)(p_ptr) + val_offset))

    template<typename T, typename c_type>
    class _ParameterBase
    {
    friend class _UntypedParameterBase;
    public:

        std::optional<T> getStrict(const std::unordered_set<T>& allowed_values = {}, bool log = true)
        {
            pond_parameter* p = api->get_parameter(api->ctx, (uint8_t*)name->c_str());
            if (p != NULL)
            {
                if (p->type == pt)
                {
                    T value{_CPP_PARAM_REF(p, )};
                    
                    if (auto it = allowed_values.find(value); it != allowed_values.end() || allowed_values.size() == 0)
                    {
                        free(p);
                        return value;
                    }
                    else if (log)
                    {
                        std::string list; list.reserve(1000);
                        for (auto& v : allowed_values)
                        {
                            list.append(", ");
                            if constexpr (std::is_same_v<T, std::string>) { list.append("'");list.append(v);list.append("'"); }
                            else if constexpr (std::is_same_v<T, bool>) list.append(v ? "true" : "false");
                            else list.append(std::to_string(v));
                        }

                        std::string string_value;
                        if constexpr (std::is_same_v<T, std::string>) string_value = "'" + value + "'";
                        else if constexpr (std::is_same_v<T, bool>) string_value = (value ? "true" : "false");
                        else string_value = std::to_string(value);

                        api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s' (%s) value must be in { %s }", name->c_str(), string_value.c_str(), &list.c_str()[2]);
                    }   
                }
                else if (log) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s'.type != '%s'", name->c_str(), type_name);
                free(p);
            }
            else if (log) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s' not set", name->c_str());
            
            return std::nullopt;
        }

        void set(const T& val)
        {
            pond_parameter* p;
            if constexpr (std::is_same_v<T, std::string>)
            {
                p = (pond_parameter*)malloc(sizeof(pond_parameter) + val.size() + 1);
                p->value.String = (uint8_t*)p + sizeof(pond_parameter);
                memcpy(p->value.String, val.c_str(), val.size() + 1);
            }
            else
            {
                p = (pond_parameter*)malloc(sizeof(pond_parameter));
                _CPP_PARAM_REF(p, ) = val;
            }
            
            p->type = pt;
            api->set_parameter(api->ctx, (uint8_t*)name->c_str(), p);
        }

        T get(const T& default_val, const std::unordered_set<T>& allowed_values = {}, bool set_default = true)
        {
            if (auto ret = getStrict(allowed_values, false)) return std::move(ret.value());
            else
            {
                if (set_default) set(default_val);
                return default_val;
            }
        }
    private:
        _ParameterBase(std::string* _name, pond_api* _api, const char* _type_name, size_t _val_offset, pond_parameter_type _pt)
            : api(_api), name(_name), type_name(_type_name), val_offset(_val_offset), pt(_pt) {}
        pond_api* api;
        std::string* name;
        const char* type_name;
        size_t val_offset;
        pond_parameter_type pt;
    };

    template<typename T, typename c_type>
    class _ParameterArrayBase
    {
    friend class _UntypedParameterBase;
    public:

        std::optional<std::vector<T>> getStrict(uint32_t min_length = 0, uint32_t max_length = 0, bool log = true)
        {
            pond_parameter* p = api->get_parameter(api->ctx, (uint8_t*)name->c_str());
            if (p != NULL)
            {
                if (p->type == pt)
                {
                    if (check_array_length(p->array_length, min_length, max_length, log))
                    {
                        std::vector<T> vec(p->array_length);
                        for (int i = 0; i < vec.size(); i++) vec[i] = T{_CPP_PARAM_REF(p, *)[i]};
                        free(p);
                        return vec;
                    }
                }
                else if (log) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s'.type != '%s'", name->c_str(), type_name);
                free(p);
            }
            else if (log) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s' not set", name->c_str());
        
            return std::nullopt;
        }

        void set(const std::vector<T>& val)
        {
            pond_parameter* p;
            if constexpr (std::is_same_v<T, std::string>)
            {
                size_t total_size = 0;
                for (auto& s : val) total_size += s.size() + 1;

                p = (pond_parameter*)malloc(sizeof(pond_parameter) + sizeof(uint8_t*) * val.size() + total_size);
                p->value.StringArray = (uint8_t**)((uint8_t*)p + sizeof(pond_parameter));

                for (uint32_t i = 0, offset = 0; i < val.size(); i++)
                {
                    p->value.StringArray[i] = (uint8_t*)((uint8_t*)p + sizeof(pond_parameter) + sizeof(uint8_t*) * val.size() + offset);
                    memcpy(p->value.StringArray[i], val[i].c_str(), val[i].size()+1);
                    offset += val[i].size()+1;
                }
            }
            else
            {
                p = (pond_parameter*)malloc(sizeof(pond_parameter) + sizeof(c_type) * val.size());
                _CPP_PARAM_REF(p, *) = (c_type*)((uint8_t*)p + sizeof(pond_parameter));
                for (uint32_t i = 0; i < val.size(); i++) _CPP_PARAM_REF(p, *)[i] = val[i];
            }
            
            p->type = pt;
            p->array_length = val.size();
            api->set_parameter(api->ctx, (uint8_t*)name->c_str(), p);
        }

        std::vector<T> get(const std::vector<T>& default_val, uint32_t min_length = 0, uint32_t max_length = 0, bool set_default = true)
        {
            if (auto ret = getStrict(min_length, max_length, false)) return std::move(ret.value());
            else
            {
                if (set_default) set(default_val);
                return default_val;
            }
        }

    private:
        bool check_array_length(uint32_t length, uint32_t min, uint32_t max, bool log)
        {
            if (min == max && min != 0 && length != min)
            {
                if (log) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s'.length != %d", name->c_str(), min);
                return false;
            }
            if (length < min)
            {
                if (log) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s'.length < %d", name->c_str(), min);
                return false;
            }
            if (length > max && max != 0)
            {
                if (log) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s'.length > %d", name->c_str(), max);
                return false;
            }
            return true;
        }

        _ParameterArrayBase(std::string* _name, pond_api* _api, const char* _type_name, size_t _val_offset, pond_parameter_type _pt)
            : api(_api), name(_name), type_name(_type_name), val_offset(_val_offset), pt(_pt) {}
        pond_api* api;
        std::string* name;
        const char* type_name;
        size_t val_offset;
        pond_parameter_type pt;
    };

    class _UntypedParameterBase
    {
    friend class ModuleBase;
    public:

    _ParameterBase<int32_t, int32_t> asInt();
    _ParameterArrayBase<int32_t, int32_t> asIntArray();
    _ParameterBase<double, double> asDouble();
    _ParameterArrayBase<double, double> asDoubleArray();
    _ParameterBase<bool, bool> asBool();
    _ParameterArrayBase<bool, bool> asBoolArray();
    _ParameterBase<std ::string, char*> asString();
    _ParameterArrayBase<std ::string, char*> asStringArray();

    private:

        _UntypedParameterBase(std::string* _name, pond_api* _api) : api(_api), name(_name) {}
        pond_api* api;
        std::string* name;
    };

    template<typename... Args>
    class Distributor
    {
    friend class ModuleBase;
    public:
        void destroy()
        {
            if (id == -1) return;
            api->destroy_distributor(api->ctx, id);
        }

        void distribute(Args&... args)
        {
            if (id == -1) return;
            void* data[] = { static_cast<void*>(&args)... };
            api->distribute(api->ctx, id, data);
        }
    private:
        int32_t id;
        pond_api* api; 
    };

    template<typename... Args>
    class ReceiverCallback
    {
    public:
        explicit ReceiverCallback(std::function<void(Args&...)> callback) : callback_(std::move(callback)) {}

        static void trampoline(pond_api* api, void* callback_pointer, void** data)
        {
            static_cast<ReceiverCallback*>(callback_pointer)->invoke(data, std::index_sequence_for<Args...>{});
        }

    private:
        std::function<void(Args&...)> callback_;

        template<std::size_t... I>
        void invoke(void** data, std::index_sequence<I...>)
        {
            callback_(*static_cast<std::remove_reference_t<Args>*>(data[I])...);
        }
    };

    template<typename... Args>
    class Receiver
    {
    friend class ModuleBase;
    public:
        void destroy()
        {
            if (id == -1) return;
            api->destroy_receiver(api->ctx, id);
        }
    private:
        int32_t id;
        pond_api* api;
        std::shared_ptr<ReceiverCallback<Args...>> callback;
    };

    class ModuleBase
    {
    public:
        virtual pond_result onStartup(const std::vector<void*>& args);
        virtual void onShutdown();
        virtual void onFrame();
        void shutdown();

        template<typename... Args>
        Distributor<Args...> createDistributor(const std::array<std::string, sizeof...(Args)>& topics)
        {
            std::array<std::string, sizeof...(Args)> type_name_strings = {std::string(typeid(Args).name()) + "_" + std::to_string(typeid(Args).hash_code())...};
            std::array<pond_dds_slot_info, sizeof...(Args)> slot_infos;
            
            for (int i = 0; i < sizeof...(Args); i++)
            {
                slot_infos[i].topic = (uint8_t*)topics[i].c_str();
                slot_infos[i].type = (uint8_t*)type_name_strings[i].c_str();
            }

            Distributor<Args...> d;
            d.api = &_pond_api;
            d.id = d.api->create_distributor(d.api->ctx, slot_infos.data(), sizeof...(Args));
            return d;
        }

        template<typename... Args, typename Callback>
        Receiver<Args...> createReceiver(const std::array<std::string, sizeof...(Args)>& topics, Callback&& callback)
        {
            std::array<std::string, sizeof...(Args)> type_name_strings = {std::string(typeid(Args).name()) + "_" + std::to_string(typeid(Args).hash_code())...};
            std::array<pond_dds_slot_info, sizeof...(Args)> slot_infos;
            
            for (int i = 0; i < sizeof...(Args); i++)
            {
                slot_infos[i].topic = (uint8_t*)topics[i].c_str();
                slot_infos[i].type = (uint8_t*)type_name_strings[i].c_str();
            }

            Receiver<Args...> r;

            r.api = &_pond_api;
            r.callback =  std::make_shared<ReceiverCallback<Args...>>(
                std::function<void(Args&...)>(
                    std::forward<Callback>(callback)
                )
            );

            r.id = r.api->create_receiver(
                r.api->ctx, 
                slot_infos.data(), 
                sizeof...(Args), 
                ReceiverCallback<Args...>::trampoline,
                &(*r.callback)
            );
            return r;
        }

        _UntypedParameterBase parameter(const std::string& name);

        pond_api _pond_api; 
    };
}

#define POND_LOG(format, ...) _pond_api.log(_pond_api.ctx, (uint8_t*)format, ##__VA_ARGS__)

#define POND_MODULE_CPP_DECLARE(cls, _name, _info)\
pond_result pond_module_##cls##_on_startup(pond_api* api, uint32_t argc, void** argv)\
{\
    cls* module = new cls();\
    api->set_user_ptr(api->ctx, module);\
    module->_pond_api = *api;\
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

#ifdef POND_MODULE_CPP_MAKE_IMPLEMENTATION
namespace pond
{

_UntypedParameterBase ModuleBase::parameter(const std::string& name)
{
    return _UntypedParameterBase((std::string*)&name, &_pond_api);
}

#define TYPED_PARAM_IMPL(_type, _param_type, _param_name, _c_type)\
    _ParameterBase<_type, _c_type> _UntypedParameterBase::as##_param_name()\
    {\
        return _ParameterBase<_type, _c_type>(name, api, #_param_name, offsetof(pond_parameter, value._param_name), _param_type);\
    }\
    _ParameterArrayBase<_type, _c_type> _UntypedParameterBase::as##_param_name##Array()\
    {\
        return _ParameterArrayBase<_type, _c_type>(name, api, #_param_name"Array", offsetof(pond_parameter, value._param_name##Array), _param_type);\
    }\

    TYPED_PARAM_IMPL(int32_t, POND_PARAMETER_INT, Int, int32_t)
    TYPED_PARAM_IMPL(double, POND_PARAMETER_DOUBLE, Double, double)
    TYPED_PARAM_IMPL(bool, POND_PARAMETER_BOOL, Bool, bool)
    TYPED_PARAM_IMPL(std::string, POND_PARAMETER_STRING, String, char*)

pond_result ModuleBase::onStartup(const std::vector<void*>& args) {return POND_SUCCESS;}
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