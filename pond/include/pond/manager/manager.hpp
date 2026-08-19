#pragma once

#include <pond/pond.h>

#include <memory>
#include <string>
#include <mutex>
#include <vector>
#include <shared_mutex>
#include <sys/types.h>
#include <unordered_map>
#include <thread>
#include <stdarg.h>
#include <atomic>
#include <unordered_set>

#include "slot_array.hpp"

typedef struct PondManager PondManager;

namespace pond_internal
{

struct Slot
{
    std::string type;
    std::string topic;
};

struct Receiver
{
    std::string module_name;
    std::vector<Slot> slots;
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
    std::vector<Slot> slots;
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
    std::string thread_name;
    uint32_t discovery_id;
    std::unordered_map<std::string, std::string> topic_mappings;
    pond_api native_api;
    ModuleContext context;
    std::vector<void*> args;

    std::unordered_map<std::string, pond_parameter*> parameters;
    std::mutex parameter_mutex;

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
    uint32_t id;
    std::thread thread;

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
        const std::string& thread_name,
        const std::unordered_map<std::string, pond_parameter*>& parameters = {},
        const std::unordered_map<std::string, std::string>& topic_mappings = {},
        const std::vector<void*>& args = {}
    );

    std::string shutdown_module(const std::string& name);

    std::string print_modules();

    void api_shutdown(pond_internal::Module* module);
    void api_log(pond_internal::Module* module, uint8_t* format, va_list args);
    void api_set_user_ptr(pond_internal::Module* module, void* ptr);
    void* api_get_user_ptr(pond_internal::Module* module);

    void api_set_parameter(pond_internal::Module* module, uint8_t* name, pond_parameter* parameter);
    pond_parameter* api_get_parameter(pond_internal::Module* module, uint8_t* name);

    int32_t api_create_distributor(pond_internal::Module* module, pond_dds_slot_info* slots, uint32_t slot_count);
    void api_destroy_distributor(pond_internal::Module* module, uint32_t distributor);
    void api_distribute(pond_internal::Module* module, uint32_t distributor, void** slot_data);

    int32_t api_create_receiver(pond_internal::Module* module, pond_dds_slot_info* slots, uint32_t slot_count, pfn_pond_receiver_callback callback, void* callback_pointer);
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
    } dds_discovery;
    
    SlotArray<std::shared_ptr<pond_internal::Thread>> threads;

    struct
    { 
        std::mutex module_mutex;
        SlotArray<std::shared_ptr<pond_internal::Module>> modules;
    } module_discovery;
    
};