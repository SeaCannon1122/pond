#include "pond/pond.h"
#include "pond_manager.hpp"
#include <dlfcn.h>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdlib.h>
#include <sstream>
#include <thread>
#include <vector>
#include <filesystem>

void _pond_shutdown(pond_internal::ModuleContext* ctx)
{
    ctx->manager->api_shutdown(ctx->module);
}

void _pond_log(pond_internal::ModuleContext* ctx, uint8_t* format, ...)
{
    va_list args;
    va_start(args, format);
    ctx->manager->api_log(ctx->module, format, args);
    va_end(args);
}

void _pond_set_user_ptr(pond_internal::ModuleContext* ctx, void* ptr)
{
    ctx->manager->api_set_user_ptr(ctx->module, ptr);
}

void* _pond_get_user_ptr(pond_internal::ModuleContext* ctx)
{
    return ctx->manager->api_get_user_ptr(ctx->module);
}

void _pond_set_parameter(pond_internal::ModuleContext* ctx, uint8_t* name, pond_parameter* parameter)
{
    ctx->manager->api_set_parameter(ctx->module, name, parameter);
}

pond_parameter* _pond_get_parameter(pond_internal::ModuleContext* ctx, uint8_t* name)
{
    return ctx->manager->api_get_parameter(ctx->module, name);
}

int32_t _pond_create_distributor(pond_internal::ModuleContext* ctx, uint8_t** topics, uint32_t topic_count, uint8_t** topic_type_names)
{
    return ctx->manager->api_create_distributor(ctx->module, topics, topic_count, topic_type_names);
}

void _pond_destroy_distributor(pond_internal::ModuleContext* ctx, uint32_t distributor)
{
    ctx->manager->api_destroy_distributor(ctx->module, distributor);
}

void _pond_distribute(pond_internal::ModuleContext* ctx, uint32_t distributor, void** data)
{
    ctx->manager->api_distribute(ctx->module,distributor, data);
}

int32_t _pond_create_receiver(pond_internal::ModuleContext* ctx, uint8_t** topics, uint32_t topic_count, uint8_t** topic_type_names, pfn_pond_receiver_callback callback, void* callback_pointer)
{
    return ctx->manager->api_create_receiver(ctx->module, topics, topic_count, topic_type_names, callback, callback_pointer);
}

void _pond_destroy_receiver(pond_internal::ModuleContext* ctx, uint32_t receiver)
{
    ctx->manager->api_destroy_receiver(ctx->module, receiver);
}

std::vector<std::string> get_bundle_paths()
{
    std::vector<std::string> paths;

    const char* env = getenv("POND_BUNDLE_PATH");
    if (!env) return paths;

#ifdef _WIN32
    char separator = ';';
#else
    char separator = ':';
#endif

    std::stringstream ss(env);
    std::string path;

    while (std::getline(ss, path, separator)) if (!path.empty()) paths.push_back(path);
    return paths;
}

PondManager::PondManager()
{
    log("Constructed");
}

PondManager::~PondManager()
{
    for (auto& t : threads) t->shutdown_thread.store(true);
    for (auto& t : threads) t->thread.join();

    log("Deconstructed");
}

void PondManager::log(const std::string& message)
{
    printf(("[POND_MANAGER] " + message + "\n").c_str());
    fflush(stdout);
}

bool PondManager::load_module_library(
    const std::string& bundle_name,
    const std::string& module_name,
    pond_internal::Module& module,
    std::string& message
)
{
    #if defined(_WIN32)
    std::string library_name = bundle_name + ".dll";
#elif defined(__linux__)
    std::string library_name = "lib" + bundle_name + ".so";
#else
    std::string library_name = bundle_name;
#endif
    std::vector<std::string> bundle_paths = get_bundle_paths();

    for (auto& path : bundle_paths)
    {
        std::string library_path = (path + (path[path.length()-1] == '/' ? library_name : "/" + library_name));
        
        if (!std::filesystem::exists(library_path)) continue;

        module.lib_handle = dlopen(library_path.c_str(), RTLD_NOW);
        if (module.lib_handle == NULL)
        {
            message = "Could not open '" + library_path + "' with error: " + std::string(dlerror());
            return false;
        }

        pond_bundle_metadata* bundle_metadata = (pond_bundle_metadata*)dlsym(module.lib_handle, "pond_bundle_metadata_");
        if (bundle_metadata == NULL)
        {
            message = "Could not retrieve symbol 'pond_bundle_metadata_' from '" + library_path + "' with error: " + std::string(dlerror());
            dlclose(module.lib_handle);
            return false;
        }

        uint32_t i = 0;
        for (; i < bundle_metadata->module_count; i++)
        {
            if (std::string((const char*)bundle_metadata->modules[i].name) != module_name) continue;

            module.module_api = bundle_metadata->modules[i];
            return true;
        }

        message = "Bundle '" + bundle_name + "' does not contain module '" + module_name + "'";
        dlclose(module.lib_handle);
        return false;
    }

    message = "Could not find bundle '" + bundle_name + "'";
    return false;
}

#define LOG_RETURN(message) {std::string _m = message; log(_m); return _m;}

std::string PondManager::load_module(
    const std::string& name,
    const std::string& bundle_name,
    const std::string& module_name,
    const std::string& thread_name,
    const std::unordered_map<std::string, pond_parameter*>& parameters,
    const std::unordered_map<std::string, std::string>& topic_mappings
)
{
    {
        std::lock_guard<std::mutex> lock(module_discovery.module_mutex);
        for (auto& m : module_discovery.modules) if (m->name == module_name)
            LOG_RETURN("Module with name '" + name + "' already exists on thread '" + m->thread_name + "'");
    }
    
    auto module =  std::make_shared<pond_internal::Module>();

    std::string load_message;
    if (!load_module_library(bundle_name, module_name, *module, load_message)) LOG_RETURN(load_message);

    module->name = name;
    module->alive.store(true);
    module->should_shutdown.store(false);
    module->topic_mappings = topic_mappings;
    module->parameters = parameters;
    module->thread_name = thread_name;

    module->context.module = module.get();
    module->context.manager = this;
    module->native_api.ctx = &module->context;

    module->native_api.create_distributor = (pfn_pond_create_distributor)_pond_create_distributor;
    module->native_api.destroy_distributor = (pfn_pond_destroy_distributor)_pond_destroy_distributor;
    module->native_api.distribute = (pfn_pond_distribute)_pond_distribute;

    module->native_api.create_receiver = (pfn_pond_create_receiver)_pond_create_receiver;
    module->native_api.destroy_receiver = (pfn_pond_destroy_receiver)_pond_destroy_receiver;
    
    module->native_api.shutdown = (pfn_pond_shutdown)_pond_shutdown;
    module->native_api.log = (pfn_pond_log)_pond_log;
    module->native_api.set_user_ptr = (pfn_pond_set_user_ptr)_pond_set_user_ptr;
    module->native_api.get_user_ptr = (pfn_pond_get_user_ptr)_pond_get_user_ptr;

    module->native_api.get_parameter = (pfn_pond_get_parameter)_pond_get_parameter;
    module->native_api.set_parameter = (pfn_pond_set_parameter)_pond_set_parameter;

    std::shared_ptr<pond_internal::Thread> thread;
    for (auto& t : threads) if (t->name == thread_name) { thread = t; break; }
    if (!thread)
    {
        thread = std::make_shared<pond_internal::Thread>();
        thread->name = thread_name;
        thread->shutdown_thread.store(false);
        thread->load_request.is.store(false);
        thread->shutdown_request.is.store(false);
        thread->thread = std::thread(
            &PondManager::thread_function,
            this,
            thread.get()
        );

        thread->id = threads.insert(thread);
    }

    thread->load_request.module = std::move(module);
    thread->load_request.is.store(true);
    while (thread->load_request.is.load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));

    LOG_RETURN("Loaded module  '" + name + "'  of type  '" + bundle_name + "/" + module_name + "'");
}

std::string PondManager::print_modules()
{
    std::string str;
    str.reserve(10000);
    
    std::lock_guard<std::mutex> lock(module_discovery.module_mutex);

    for (auto& m : module_discovery.modules)
    {
        str.append("  '");
        str.append(m->name);
        str.append("' on '");
        str.append(m->thread_name);
        str.append("'\n");
    }

    return str;
}

std::string PondManager::shutdown_module(const std::string& name)
{
    std::string thread_name = "";
    {
        std::lock_guard<std::mutex> lock(module_discovery.module_mutex);

        for (auto& m : module_discovery.modules) if (m->name == name)
        {
            thread_name = m->thread_name;
            break;
        }
    }

    if (thread_name == "") LOG_RETURN("Could not shutdown non-existent module '" + name + "'");

    for (auto& t : threads)
    {
        if (t->name == thread_name)
        {
            t->shutdown_request.name = name;
            t->shutdown_request.is.store(true);

            while (t->shutdown_request.is.load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
            LOG_RETURN("Shut down module '" + name + "'");
        }
    }
    
}

void PondManager::cleanup_module(pond_internal::Module* module)
{
    for (int i = 0; i < module->distributors.get_length(); i++) 
        if (module->distributors.is_used(i)) 
            api_destroy_distributor(module, i);

    for (int i = 0; i < module->receivers.get_length(); i++)
        if (module->receivers.is_used(i)) 
            api_destroy_receiver(module, i);

    dlclose(module->lib_handle);
}

void PondManager::thread_function(pond_internal::Thread* thread)
{
    bool shutdown = false;
    while (!shutdown)
    {
        if (thread->load_request.is.load())
        {
            log("[" + thread->name + "] Starting module '" + thread->load_request.module->name + "' ...");
            if (thread->load_request.module->module_api.on_startup(&thread->load_request.module->native_api) == POND_SUCCESS)
            {
                log("[" + thread->name + "] Started module '" + thread->load_request.module->name + "'");
                thread->modules.insert(thread->load_request.module);
                std::lock_guard<std::mutex> lock(module_discovery.module_mutex);
                thread->load_request.module->discovery_id = module_discovery.modules.insert(thread->load_request.module);
            }
            else
            {
                log("[" + thread->name + "] Error on startup of module '" + thread->load_request.module->name + "'");
                cleanup_module(thread->load_request.module.get());
            }
            
            thread->load_request.module.reset();
            thread->load_request.is.store(false);
        }

        if (thread->shutdown_request.is.load())
        {
            for (auto& m : thread->modules)
            {
                if (m->name == thread->shutdown_request.name)
                {
                    m->should_shutdown.store(true);
                    break;
                }
            }
                
            thread->shutdown_request.is.store(false);
        }

        if (shutdown = thread->shutdown_thread.load()) for (auto& m : thread->modules) m->should_shutdown.store(true);

        for (int i = 0; i < thread->modules.get_length(); i++) if (thread->modules.is_used(i))
        {
            if (thread->modules[i]->should_shutdown.load())
            {
                log("[" + thread->name + "] Shutting down module '" + thread->modules[i]->name + "' ...");
                thread->modules[i]->module_api.on_shutdown(&thread->modules[i]->native_api);
                log("[" + thread->name + "] Shut down module '" + thread->modules[i]->name + "'");
                cleanup_module(thread->modules[i].get());
                
                {
                    std::lock_guard<std::mutex> lock(module_discovery.module_mutex);
                    module_discovery.modules.release_slot(thread->modules[i]->discovery_id);
                }
                
                thread->modules.release_slot(i);
            }
                
        }

        for (auto& m : thread->modules) m->module_api.on_frame(&m->native_api);
    }

    log("[" + thread->name + "] exited");
}