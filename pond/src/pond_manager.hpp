#pragma once

#include <memory>
#include <pond/pond.h>
#include "slot_array.hpp"
#include <string>
#include <mutex>
#include <vector>
#include <shared_mutex>
#include <sys/types.h>
#include <unordered_map>
#include <thread>
#include <stdarg.h>
#include <atomic>

typedef struct PondManager PondManager;

namespace pond_internal
{

struct Receiver
{
    std::string module_name;
    std::vector<std::string> topics;
    std::vector<std::string> topic_types;
    uint32_t discovery_id;
    pond_api* api;

    std::shared_mutex mutex;
    std::atomic<bool> active;
    pfn_pond_receiver_callback callback;
    void* callback_pointer;
};

struct ReceiverConnection
{
    std::shared_ptr<Receiver> receiver;
    std::vector<uint32_t> indices;
    std::vector<void*> handle_array;
};

struct Distributor
{
    std::string module_name;
    std::vector<std::string> topics;
    std::vector<std::string> topic_types;
    uint32_t discovery_id;

    SlotArray<ReceiverConnection> connections;

    std::mutex new_connections_mutex;
    SlotArray<ReceiverConnection> new_connections;
};

struct Module;

struct ModuleContext
{
    PondManager* manager;
    Module* module;
};

struct Module
{
    std::string name;
    std::unordered_map<std::string, std::string> topic_mappings;
    pond_api native_api;
    ModuleContext context;

    void* lib_handle;
    pond_module_metadata module_api;
    void* user_pointer;
    
    SlotArray<std::shared_ptr<Distributor>> distributors;
    SlotArray<std::shared_ptr<Receiver>> receivers;

    std::atomic<bool> should_shutdown;
    std::atomic<bool> alive;
};

struct Thread
{
    std::string name;
    std::thread thread;

    std::mutex mutex;
    SlotArray<std::shared_ptr<pond_internal::Module>> modules;

    struct
    {
        std::atomic<bool> is;
        std::shared_ptr<pond_internal::Module> module;
    } load_request;

    struct
    {
        std::atomic<bool> is;
        std::string name;
    } shutdown_request;
    
    std::atomic<bool> shutdown_thread;
};

}

class PondManager
{
public:
    PondManager();
    ~PondManager();

    std::string load_module(
        const std::string& name,
        const std::string& bundle_name,
        const std::string& module_name,
        const std::string& thread_name
    );

    std::string shutdown_module(const std::string& name);

    std::string print_modules();

    void api_shutdown(pond_internal::Module* module);
    void api_log(pond_internal::Module* module, uint8_t* format, va_list args);
    void api_set_user_ptr(pond_internal::Module* module, void* ptr);
    void* api_get_user_ptr(pond_internal::Module* module);

    void api_set_parameter(pond_internal::Module* module, uint8_t* name, pond_parameter* parameter);
    pond_parameter* api_get_parameter(pond_internal::Module* module, uint8_t* name);

    int32_t api_create_distributor(pond_internal::Module* module, uint8_t** topics, uint32_t topic_count, uint8_t** topic_type_names);
    void api_destroy_distributor(pond_internal::Module* module, uint32_t distributor);
    void api_distribute(pond_internal::Module* module, uint32_t distributor, void** data);

    int32_t api_create_receiver(pond_internal::Module* module, uint8_t** topics, uint32_t topic_count, uint8_t** topic_type_names, pfn_pond_receiver_callback callback, void* callback_pointer);
    void api_destroy_receiver(pond_internal::Module* module, uint32_t receiver);

private:

    void log(const std::string& message);

    bool load_module_library(
        const std::string& bundle_name,
        const std::string& module_name,
        pond_internal::Module& module,
        std::string& message
    );

    bool try_connect_receiver(std::shared_ptr<pond_internal::Distributor>& d, std::shared_ptr<pond_internal::Receiver>& r, bool to_new_connections);
    void thread_function(pond_internal::Thread* thread);
    void cleanup_module(pond_internal::Module* module);

    struct
    {
        std::shared_mutex receiver_mutex;
        SlotArray<std::shared_ptr<pond_internal::Receiver>> receivers;

        std::shared_mutex distributor_mutex;
        SlotArray<std::shared_ptr<pond_internal::Distributor>> distributors;
    } discovery;
    
    SlotArray<std::shared_ptr<pond_internal::Thread>> threads;
};