#pragma once

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
#include <set>
#include <atomic>

typedef struct PondManager PondManager;

namespace pond_internal
{

struct Module;

struct ModuleContext
{
    PondManager* manager;
    Module* module;
};

struct Module
{
    // init static
    std::string name;
    std::unordered_map<std::string, std::string> topic_mappings;
    pond_api native_api;
    ModuleContext context;

    // lib stuff
    void* lib_handle;
    pond_module_metadata module_api;
    
    // thread managed
    std::set<int32_t> distributors;
    std::set<int32_t> receivers;

    // state
    std::atomic<bool> should_shutdown;
    std::atomic<bool> alive;
};

struct Thread
{
    std::mutex mutex;
    std::string name;
    std::thread thread;
    std::set<uint32_t> modules;
};

struct Distributor
{
    std::string name;
};

struct Receiver
{
    std::string name;
    pfn_pond_receiver_callback callback;
    void* callback_pointer;
};

struct Topic
{
    int distributor_count;
    std::string name;
    SlotArray<uint32_t> subscriber_indicies;
};
}

class PondManager
{
public:
    PondManager();
    ~PondManager();
    void log(const std::string& message);

    void load_module(
        const std::string& name,
        const std::string& bundle_name,
        const std::string& module_name,
        const std::string& thread_name
    );

    void shutdown_module(const std::string& name);

    void api_module_shutdown(pond_internal::Module* module);
    void api_module_log(pond_internal::Module* module, uint8_t* format, va_list args);
    void api_module_set_user_ptr(pond_internal::Module* module, void* ptr);
    void* api_module_get_user_ptr(pond_internal::Module* module);

    int32_t api_create_distributor(pond_internal::Module* module, uint8_t* topic);
    void api_destroy_distributor(pond_internal::Module* module, uint32_t distributor);
    void api_distribute(pond_internal::Module* module, uint32_t distributor, void* data);

    int32_t api_create_receiver(pond_internal::Module* module, uint8_t* topic, pfn_pond_receiver_callback callback, void* callback_pointer);
    void api_destroy_receiver(pond_internal::Module* module, uint32_t receiver);

private:

    bool load_module_library(
        const std::string& bundle_name,
        const std::string& module_name,
        pond_internal::Module* module
    );

    void unload_module_library(pond_internal::Module* module);

    void thread_function(pond_internal::Thread* thread);

    std::shared_mutex distrubution_mutex;
    SlotArray<pond_internal::Topic*> topics;
    SlotArray<pond_internal::Receiver> receivers;
    SlotArray<uint32_t> distributor_topic_indicies;
    
    SlotArray<pond_internal::Module*> modules;
    SlotArray<pond_internal::Thread*> threads;
    std::unordered_map<std::string, uint32_t> thread_names_map;
    std::unordered_map<std::string, uint32_t> module_names_map;
};