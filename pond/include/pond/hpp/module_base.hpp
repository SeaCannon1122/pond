#pragma once

#include "parameter.hpp"
#include "dds.hpp"

#include <functional>

namespace pond
{
    class ModuleBase
    {
    public:
        virtual pond_result onStartup(const std::vector<void*>& args);
        virtual void onShutdown();
        virtual void onFrame();
        void shutdown();

        Distributor createDistributor(const ChannelsInfo& info)
        {
            return Distributor(&_pond_api, info);
        }

        template<typename... Args>
        Distributor createDistributor(const std::array<std::string, sizeof...(Args)>& channels)
        {
            ChannelsInfo info; auto it = channels.begin();
            (info.channel<Args>(*it++),...);

            return Distributor(&_pond_api, info);
        }

        Distributor createDistributor(const std::vector<std::string>& channels)
        {
            ChannelsInfo info;
            for (auto& c : channels) info.channel<void>(c);
            return Distributor(&_pond_api, info);
        }

        template<typename... Args>
        DistributorTyped<Args...> createDistributorTyped(const std::array<std::string, sizeof...(Args)>& channels)
        {
            return DistributorTyped<Args...>(&_pond_api, channels);
        }

        template<size_t i> using argtype_void = void*;

        template<size_t... I>
        static void typed_receiver_callback_entrypoint(pond_api* api, void* callback_pointer, void** data)
        {
            std::function<void(argtype_void<I>...)>* function = (std::function<void(argtype_void<I>...)>*)callback_pointer;
            (*function)(data[I]...);
        };

        template <size_t N, size_t... I>
        struct sequence_callback_generator : sequence_callback_generator<N - 1, N - 1, I...> {};

        template <size_t... I>
        struct sequence_callback_generator<0, I...>
        {
            static constexpr auto callback = typed_receiver_callback_entrypoint<I...>;
        };

        template<typename... Args, typename Callback>
        Receiver createReceiver(const std::array<std::string, sizeof...(Args)>& channels, const Callback& callback)
        {
            ChannelsInfo info; auto it = channels.begin();
            (info.channel<Args>(*it++),...);

            auto cb = std::make_shared<std::function<void(Args*...)>>(callback);

            return Receiver(&_pond_api, info, *(std::shared_ptr<void>*)&cb, sequence_callback_generator<sizeof...(Args)>::callback);
        }

        static void raw_receiver_callback_entrypoint(pond_api* api, void* callback_pointer, void** data)
        {
            std::function<void(void**)>* function = (std::function<void(void**)>*)callback_pointer;
            (*function)(data);
        }

        template<typename T, typename Callback>
        Receiver createReceiver(const ChannelsInfo& info, const Callback& callback)
        {
            auto cb = std::make_shared<std::function<void(T**)>>(callback);

            return Receiver(&_pond_api, info, *(std::shared_ptr<void>*)&cb, raw_receiver_callback_entrypoint);
        }

        Receiver createReceiver(const std::vector<std::string>& channels, const std::function<void(void**)>& callback)
        {
            auto cb = std::make_shared<std::function<void(void**)>>(callback);
            ChannelsInfo info;
            for (auto& c : channels) info.channel<void>(c);

            return Receiver(&_pond_api, info, *(std::shared_ptr<void>*)&cb, raw_receiver_callback_entrypoint);
        }

        _UntypedParameterBase parameter(const std::string& name);

        pond_api _pond_api; 
    };
}

#define POND_LOG(format, ...) _pond_api.log(_pond_api.ctx, (uint8_t*)format, ##__VA_ARGS__)
#define POND_LOG_RETURN(format, ...) {POND_LOG(format, ##__VA_ARGS__); return;}
#define POND_LOG_RETURN_ERROR(format, ...) {POND_LOG(format, ##__VA_ARGS__); return POND_ERROR;}
#define POND_LOG_RETURN_FALSE(format, ...) {POND_LOG(format, ##__VA_ARGS__); return false;}

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
        return _ParameterBase<_type, _c_type>(name, api, offsetof(pond_parameter, value._param_name), _param_type);\
    }\
    _ParameterArrayBase<_type, _c_type> _UntypedParameterBase::as##_param_name##Array()\
    {\
        return _ParameterArrayBase<_type, _c_type>(name, api, offsetof(pond_parameter, value._param_name##Array), _param_type##_ARRAY);\
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

}
#endif