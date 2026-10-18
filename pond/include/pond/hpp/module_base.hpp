#pragma once

#include "parameter.hpp"
#include "dds.hpp"

#include <functional>

namespace pond
{
    class ModuleBase : public ParameterSpace
    {
    public:
        void init(pond_api *_api) {_pond_api = *_api; ParameterSpace::api = &_pond_api; ParameterSpace::_param_prefix = "";}

        virtual pond_result onStartup(const std::vector<void*>& args) {return POND_SUCCESS;}
        virtual void onShutdown() {}
        virtual void onFrame() {}
        void shutdown() {_pond_api.shutdown(_pond_api.ctx);}

        Distributor createDistributor(const ChannelsInfo& info)
        {
            return Distributor(&_pond_api, info);
        }

        template<typename... Channel_Types>
        Distributor createDistributor(const std::array<std::string, sizeof...(Channel_Types)>& channels)
        {
            ChannelsInfo info; auto it = channels.begin();
            (info.channel<Channel_Types>(*it++),...);

            return Distributor(&_pond_api, info);
        }

        Distributor createDistributor(const std::vector<std::string>& channels)
        {
            ChannelsInfo info;
            for (auto& c : channels) info.channel<void>(c);
            return Distributor(&_pond_api, info);
        }

        template<typename... Channel_Types>
        DistributorTyped<Channel_Types...> createDistributorTyped(const std::array<std::string, sizeof...(Channel_Types)>& channels)
        {
            return DistributorTyped<Channel_Types...>(&_pond_api, channels);
        }

        template<size_t i> using argtype_void = void*;

        template<size_t... I>
        static void typed_callback_callback_entrypoint(pond_api* api, void* callback_pointer, void** data)
        {
            std::function<void(argtype_void<I>...)>* function = (std::function<void(argtype_void<I>...)>*)callback_pointer;
            (*function)(data[I]...);
        };

        template <size_t N, size_t... I>
        struct sequence_callback_generator : sequence_callback_generator<N - 1, N - 1, I...> {};

        template <size_t... I>
        struct sequence_callback_generator<0, I...>
        {
            static constexpr auto callback = typed_callback_callback_entrypoint<I...>;
        };

        template<typename... Channel_Types, typename Callback_Type>
        Callback createCallback(const std::array<std::string, sizeof...(Channel_Types)>& channels, const Callback_Type& callback)
        {
            ChannelsInfo info; auto it = channels.begin();
            (info.channel<Channel_Types>(*it++),...);

            auto cb = std::make_shared<std::function<void(Channel_Types*...)>>(callback);

            return Callback(&_pond_api, info, *(std::shared_ptr<void>*)&cb, sequence_callback_generator<sizeof...(Channel_Types)>::callback);
        }

        static void raw_callback_callback_entrypoint(pond_api* api, void* callback_pointer, void** data)
        {
            std::function<void(void**)>* function = (std::function<void(void**)>*)callback_pointer;
            (*function)(data);
        }

        template<typename Channel_Type, typename Callback_Type>
        Callback createCallback(const ChannelsInfo& info, const Callback_Type& callback)
        {
            auto cb = std::make_shared<std::function<void(Channel_Type**)>>(callback);

            return Callback(&_pond_api, info, *(std::shared_ptr<void>*)&cb, raw_callback_callback_entrypoint);
        }

        Callback createCallback(const std::vector<std::string>& channels, const std::function<void(void**)>& callback)
        {
            auto cb = std::make_shared<std::function<void(void**)>>(callback);
            ChannelsInfo info;
            for (auto& c : channels) info.channel<void>(c);

            return Callback(&_pond_api, info, *(std::shared_ptr<void>*)&cb, raw_callback_callback_entrypoint);
        }

        pond_api _pond_api; 
    };
}

#define POND_LOG(format, ...) _pond_api.log(_pond_api.ctx, (uint8_t*)format, ##__VA_ARGS__)
#define POND_LOG_RETURN(format, ...) {POND_LOG(format, ##__VA_ARGS__); return;}
#define POND_LOG_RETURN_ERROR(format, ...) {POND_LOG(format, ##__VA_ARGS__); return POND_ERROR;}
#define POND_LOG_RETURN_FALSE(format, ...) {POND_LOG(format, ##__VA_ARGS__); return false;}

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
