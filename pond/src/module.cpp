#include "pond/pond.h"
#include "pond_manager.hpp"
#include <dlfcn.h>
#include <mutex>
#include <shared_mutex>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <filesystem>

void _pond_shutdown(pond_internal::ModuleContext* ctx)
{
    ctx->manager->api_module_shutdown(ctx->module);
}

void _pond_log(pond_internal::ModuleContext* ctx, uint8_t* format, ...)
{
    va_list args;
    va_start(args, format);
    ctx->manager->api_module_log(ctx->module, format, args);
    va_end(args);
}

void _pond_set_user_ptr(pond_internal::ModuleContext* ctx, void* ptr)
{
    ctx->manager->api_module_set_user_ptr(ctx->module, ptr);
}

void* _pond_get_user_ptr(pond_internal::ModuleContext* ctx)
{
    return ctx->manager->api_module_get_user_ptr(ctx->module);
}

int32_t _pond_create_distributor(pond_internal::ModuleContext* ctx, uint8_t* topic)
{
    return ctx->manager->api_create_distributor(ctx->module, topic);
}

void _pond_destroy_distributor(pond_internal::ModuleContext* ctx, uint32_t distributor)
{
    ctx->manager->api_destroy_distributor(ctx->module, distributor);
}

void _pond_distribute(pond_internal::ModuleContext* ctx, uint32_t distributor, void* data)
{
    ctx->manager->api_distribute(ctx->module,distributor, data);
}

int32_t _pond_create_receiver(pond_internal::ModuleContext* ctx, uint8_t* topic, pfn_pond_receiver_callback callback, void* callback_pointer)
{
    return ctx->manager->api_create_receiver(ctx->module, topic, callback, callback_pointer);
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

bool PondManager::load_module_library(
    const std::string& bundle_name,
    const std::string& module_name,
    pond_internal::Module* module
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

        module->lib_handle = dlopen(library_path.c_str(), RTLD_NOW);
        if (module->lib_handle == NULL)
        {
            log("Could not open '" + library_path + "' with error: " + std::string(dlerror()));
            return false;
        }

        pond_bundle_metadata* bundle_metadata = (pond_bundle_metadata*)dlsym(module->lib_handle, "pond_bundle_metadata_");
        if (bundle_metadata == NULL)
        {
            log("Could not retrieve symbol 'pond_bundle_metadata_' from '" + library_path + "' with error: " + std::string(dlerror()));
            dlclose(module->lib_handle);
            return false;
        }

        uint32_t i = 0;
        for (; i < bundle_metadata->module_count; i++)
        {
            if (std::string((const char*)bundle_metadata->modules[i].name) != module_name) continue;

            module->module_api = bundle_metadata->modules[i];
            return true;
        }

        log("Bundle '" + bundle_name + "' does not contain module '" + module_name + "'");
        dlclose(module->lib_handle);
        return false;
    }

    log("Could not find bundle '" + bundle_name + "'");
}

void PondManager::unload_module_library(pond_internal::Module* module)
{
    dlclose(module->lib_handle);
}

void PondManager::load_module(
    const std::string& name,
    const std::string& bundle_name,
    const std::string& module_name,
    const std::string& thread_name
)
{
    pond_internal::Module* module = new pond_internal::Module();

    if (!load_module_library(bundle_name, module_name, module)) delete module;

    auto thread_map_it = thread_names_map.find(thread_name);
    if (thread_map_it == thread_names_map.end())
    {
        uint32_t thread_id = threads.get_slot();
        thread_names_map[thread_name] = thread_id;
        pond_internal::Thread* thread = new pond_internal::Thread();
        thread->name = thread_name;
        thread->thread = std::thread(&PondManager::thread_function, this, thread);

        threads[thread_id] = thread;
    }

    module->alive.store(true);
    module->should_shutdown.store(false);

    module->native_api.create_distributor = (pfn_pond_create_distributor)_pond_create_distributor;
    module->native_api.destroy_distributor = (pfn_pond_destroy_distributor)_pond_destroy_distributor;
    module->native_api.distribute = (pfn_pond_distribute)_pond_distribute;

    module->native_api.create_receiver = (pfn_pond_create_receiver)_pond_create_receiver;
    module->native_api.destroy_receiver = (pfn_pond_destroy_receiver)_pond_destroy_receiver;
    
    module->native_api.shutdown = (pfn_pond_shutdown)_pond_shutdown;
    module->native_api.log = (pfn_pond_log)_pond_log;
    module->native_api.set_user_ptr = (pfn_pond_set_user_ptr)_pond_set_user_ptr;
    module->native_api.get_user_ptr = (pfn_pond_get_user_ptr)_pond_get_user_ptr;

    pond_internal::Thread* thread = threads[thread_map_it->second];
    {
        std::lock_guard<std::mutex> thread_lock(thread->mutex);
    }
    
    log("Loaded module  '" + name + "'  of type  '" + bundle_name + "/" + module_name + "'");
}

void PondManager::shutdown_module(const std::string& name)
{
    log("Unloaded module '" + name + "'");
}

