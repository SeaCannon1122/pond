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

#include "slot_array.hpp"

#define MAX_THREAD_FRAME_TIME 10.0

#ifndef MAX_SLOT_COUNT
#define MAX_SLOT_COUNT 32
#endif

typedef struct PondManager PondManager;

namespace pond_internal
{

struct Slot
{
    std::string type;
    std::string channel;
};

struct Callback
{
    std::string module_name;
    Slot slots[MAX_SLOT_COUNT];
    uint32_t slot_count;
    uint32_t discovery_id;
    pond_api* api;

    std::shared_mutex mutex;
    std::atomic<bool> active;
    pfn_pond_callback_function callback;
    void* callback_pointer;
};

struct CallbackConnection
{
    std::shared_ptr<Callback> callback;
    uint32_t slot_count;
    uint32_t indices[MAX_SLOT_COUNT];
    void* handle_array[MAX_SLOT_COUNT];
};

struct Distributor
{
    std::string module_name;
    Slot slots[MAX_SLOT_COUNT];
    uint32_t slot_count;
    uint32_t discovery_id;

    std::vector<CallbackConnection> connections;

    std::mutex new_connections_mutex;
    std::vector<CallbackConnection> new_connections;
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
    std::unordered_map<std::string, std::string> channel_mappings;
    std::string channel_namespace;
    pond_api native_api;
    ModuleContext context;
    std::vector<void*> args;

    std::unordered_map<std::string, pond_parameter*> parameters;
    std::mutex parameter_mutex;

    void* lib_handle;
    pond_module_metadata module_api;
    void* user_pointer;
    
    SlotArray<std::shared_ptr<Distributor>> distributors;
    SlotArray<std::shared_ptr<Callback>> callbacks;

    std::atomic<bool> should_shutdown;
    std::atomic<bool> alive;
};

struct Thread
{
    std::string name;
    uint32_t id;
    std::thread thread;
    std::atomic<double> frame_time;
    double last_time;

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
    PondManager() = delete;
    PondManager(bool connect_log, bool distribute_log);
    ~PondManager();

    std::string load_module(
        const std::string& name,
        const std::string& bundle_name,
        const std::string& module_name,
        const std::string& thread_name,
        const std::unordered_map<std::string, pond_parameter*>& parameters = {},
        const std::unordered_map<std::string, std::string>& channel_mappings = {},
        const std::string& channel_namespace = "",
        const std::vector<void*>& args = {}
    );

    std::string shutdown_module(const std::string& name);

    std::string print_modules();

    void set_thread_frame_time(const std::string& thread_name, double frame_time);

    // api functions

    static void api_shutdown(pond_internal::ModuleContext* ctx);
    static void api_log(pond_internal::ModuleContext* ctx, uint8_t* format, ...);
    static void api_set_user_ptr(pond_internal::ModuleContext* ctx, void* ptr);
    static void* api_get_user_ptr(pond_internal::ModuleContext* ctx);

    static void api_set_parameter(pond_internal::ModuleContext* ctx, uint8_t* name, pond_parameter* parameter);
    static pond_parameter* api_get_parameter(pond_internal::ModuleContext* ctx, uint8_t* name);

    static int32_t api_create_distributor(pond_internal::ModuleContext* ctx, pond_dds_slot_info* slots, uint32_t slot_count);
    static void api_destroy_distributor(pond_internal::ModuleContext* ctx, uint32_t distributor);
    static void api_distribute(pond_internal::ModuleContext* ctx, uint32_t distributor, void** slot_data);

    static int32_t api_create_callback(pond_internal::ModuleContext* ctx, pond_dds_slot_info* slots, uint32_t slot_count, pfn_pond_callback_function callback, void* callback_pointer);
    static void api_destroy_callback(pond_internal::ModuleContext* ctx, uint32_t callback);

private:

    void log(const std::string& message);

    bool load_module_library(
        const std::string& bundle_name,
        const std::string& module_name,
        pond_internal::Module& module,
        std::string& message
    );

    bool construct_slots(pond_internal::Module* module, pond_internal::Slot* slots, pond_dds_slot_info* c_slots, uint32_t c_slot_count);
    bool try_connect_callback(std::shared_ptr<pond_internal::Distributor>& d, std::shared_ptr<pond_internal::Callback>& r, bool to_new_connections);
    void thread_function(pond_internal::Thread* thread);
    void cleanup_module(pond_internal::Module* module);

    struct
    {
        std::shared_mutex callback_mutex;
        SlotArray<std::shared_ptr<pond_internal::Callback>> callbacks;

        std::shared_mutex distributor_mutex;
        SlotArray<std::shared_ptr<pond_internal::Distributor>> distributors;
    } dds_discovery;
    
    std::unordered_map<std::string, double> set_thread_frame_times;
    SlotArray<std::shared_ptr<pond_internal::Thread>> threads;

    struct
    { 
        std::mutex module_mutex;
        SlotArray<std::shared_ptr<pond_internal::Module>> modules;
    } module_discovery;

    bool connect_log;
    bool distribute_log;
};