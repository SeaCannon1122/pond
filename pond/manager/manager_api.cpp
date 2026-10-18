#include <pond_manager/manager.hpp>
#include <string>
#include <time.h>
#include <unordered_set>

void PondManager::api_shutdown(pond_internal::ModuleContext* ctx)
{
    ctx->module->should_shutdown.store(true);
}

void PondManager::api_log(pond_internal::ModuleContext* ctx, uint8_t* format, ...)
{
    va_list args;
    va_start(args, format);

    uint8_t buffer[10000];
    vsnprintf((char*)buffer, sizeof(buffer), (char*)format, args);

    va_end(args);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    uint8_t* start = buffer;
    for (int i = 0;; i++) if (buffer[i] == '\n' || buffer[i] == 0)
    {
        bool end = (buffer[i] == 0);
        buffer[i] = 0;

        printf("[%s] [%lld.%06lld] %s\n", ctx->module->name.c_str(), (long long)ts.tv_sec, (long long)ts.tv_nsec / 1000, start);
        start = &buffer[i+1];

        if (end) break;
    }

    fflush(stdout);
}

void PondManager::api_set_user_ptr(pond_internal::ModuleContext* ctx, void* ptr)
{
    ctx->module->user_pointer = ptr;
}

void* PondManager::api_get_user_ptr(pond_internal::ModuleContext* ctx)
{
    return ctx->module->user_pointer;
}

void PondManager::api_set_parameter(pond_internal::ModuleContext* ctx, uint8_t* name, pond_parameter* parameter)
{
    std::lock_guard<std::mutex> lock(ctx->module->parameter_mutex);

    if (parameter)
    {
        if (auto it = ctx->module->parameters.find(std::string((char*)name)); it != ctx->module->parameters.end()) free(it->second);
        ctx->module->parameters[std::string((char*)name)] = parameter;
    }
    else ctx->module->parameters.erase(std::string((char*)name));
}

pond_parameter* PondManager::api_get_parameter(pond_internal::ModuleContext* ctx, uint8_t* name)
{
    std::lock_guard<std::mutex> lock(ctx->module->parameter_mutex);

    auto it = ctx->module->parameters.find(std::string((char*)name));

    if (it == ctx->module->parameters.end()) return NULL;
    else
    {
        pond_parameter* p = it->second;
        switch (p->type)
        {
        case POND_PARAMETER_INT:            return pond_malloc_parameter_int            (p->value.Int);
        case POND_PARAMETER_INT_ARRAY:      return pond_malloc_parameter_int_array      (p->value.IntArray, p->array_length);
        case POND_PARAMETER_DOUBLE:         return pond_malloc_parameter_double         (p->value.Double);
        case POND_PARAMETER_DOUBLE_ARRAY:   return pond_malloc_parameter_double_array   (p->value.DoubleArray, p->array_length);
        case POND_PARAMETER_BOOL:           return pond_malloc_parameter_bool           (p->value.Bool);
        case POND_PARAMETER_BOOL_ARRAY:     return pond_malloc_parameter_bool_array     (p->value.BoolArray, p->array_length);
        case POND_PARAMETER_STRING:         return pond_malloc_parameter_string         (p->value.String);
        case POND_PARAMETER_STRING_ARRAY:   return pond_malloc_parameter_string_array   (p->value.StringArray, p->array_length);
        default: return NULL;
        }
    }
}

bool PondManager::construct_slots(pond_internal::Module* module, pond_internal::Slot* slots, pond_dds_slot_info* c_slots, uint32_t c_slot_count)
{
    if (c_slot_count > MAX_SLOT_COUNT)
    {
        log("In module " + module->name + ": tried connection with slot count ("+std::to_string(c_slot_count)+") > MAX_SLOT_COUNT ("+std::to_string(MAX_SLOT_COUNT)+")");
        return false;
    }

    for (uint32_t i = 0; i < c_slot_count; i++)
    {
        slots[i].type = (c_slots[i].type != NULL ? std::string((char*)c_slots[i].type) : "");
        slots[i].channel = std::string((char*)c_slots[i].channel);

        if (slots[i].channel == "")
        {
            log("In module " + module->name + ": cannot communicate on channel without name");
            return false;
        }

        auto it = module->channel_mappings.find(slots[i].channel);
        if (it != module->channel_mappings.end())
        {
            if (it->second == "")
            {
                log("In module " + module->name + ": cannot map channel '"+ slots[i].channel +"' to channel without name");
                return false;
            }

            slots[i].channel = it->second;
        }

        if (slots[i].channel[0] != '/') slots[i].channel = module->channel_namespace + slots[i].channel;
    }

    return true;
}

bool PondManager::try_connect_callback(std::shared_ptr<pond_internal::Distributor>& d, std::shared_ptr<pond_internal::Callback>& r, bool to_new_connections)
{
    pond_internal::CallbackConnection connection;
    connection.slot_count = r->slot_count;

    for (int i = 0; i < r->slot_count; i++)
    {
        for (int j = 0; ; j++)
        {
            if (j == d->slot_count) return false;

            if (d->slots[j].channel == r->slots[i].channel)
            {
                if (d->slots[j].type != "" && r->slots[i].type != "" && d->slots[j].type != r->slots[i].type)
                {
                    log("Mismatch of types on channel '" + d->slots[j].channel + "': '" + d->module_name + "' (" + d->slots[j].type + ") -> '" + r->slots[i].channel + "': '" + r->module_name + "' (" + r->slots[i].type + ")");
                    return false;
                }
                
                connection.indices[i] = j;
                break;
            }
        }
    }

    connection.callback = r;

    if (to_new_connections)
    {
        std::lock_guard<std::mutex> lock(d->new_connections_mutex);
        d->new_connections.emplace_back(std::move(connection));
    }
    else d->connections.emplace_back(std::move(connection));
    
    return true;
}

int32_t PondManager::api_create_distributor(pond_internal::ModuleContext* ctx, pond_dds_slot_info* slots, uint32_t slot_count)
{
    auto d = std::make_shared<pond_internal::Distributor>();
    if (!ctx->manager->construct_slots(ctx->module, d->slots, slots, slot_count)) return -1;
    d->module_name = ctx->module->name;
    d->slot_count = slot_count;

    std::unordered_set<std::string> all_channels;
    for (uint32_t i = 0; i < d->slot_count; i++)
    {
        if (all_channels.find(d->slots[i].channel) != all_channels.end())
        {
            ctx->manager->log("In module " + ctx->module->name + ": can't publish on the same channel (" + d->slots[i].channel + ") multiple times in parallel");
            return -1;
        }
        all_channels.insert(d->slots[i].channel);
    }
    
    if (ctx->manager->connect_log)
    {
        std::string channel_array_string;
        channel_array_string.reserve(1000);
        for (uint32_t i = 0; i < d->slot_count; i++)
        {
            channel_array_string.append(", ");
            channel_array_string.append(d->slots[i].channel);
        }
        ctx->module->native_api.log(ctx->module->native_api.ctx, (uint8_t*)"Distributing on channels { %s }", &(channel_array_string.c_str()[2]));
    }

    {
        std::shared_lock<std::shared_mutex> lock(ctx->manager->dds_discovery.callback_mutex);
        for (auto& r : ctx->manager->dds_discovery.callbacks)
            if (ctx->manager->try_connect_callback(d, r, false)) if (ctx->manager->connect_log)
            {
                std::string channel_array_string;
                channel_array_string.reserve(1000);
                for (uint32_t i = 0; i < r->slot_count; i++)
                {
                    channel_array_string.append(", ");
                    channel_array_string.append(r->slots[i].channel);
                }
                ctx->module->native_api.log(ctx->module->native_api.ctx, (uint8_t*)"Connected to callback on module '%s' on channels { %s }", r->module_name.c_str(), &(channel_array_string.c_str()[2]));
            }
    }

    {
        std::lock_guard<std::shared_mutex> lock(ctx->manager->dds_discovery.distributor_mutex);
        d->discovery_id = ctx->manager->dds_discovery.distributors.insert(d);
    }

    return (int32_t)ctx->module->distributors.emplace(d);
}

void PondManager::api_destroy_distributor(pond_internal::ModuleContext* ctx, uint32_t distributor)
{
    if (!ctx->module->distributors.is_used(distributor))
    {
        ctx->manager->log("Invalid distributor " + std::to_string(distributor) + " in module '" + ctx->module->name + "'");
        return;
    }
    auto d = &*ctx->module->distributors[distributor];

    {
        std::lock_guard<std::shared_mutex> lock(ctx->manager->dds_discovery.distributor_mutex);
        ctx->manager->dds_discovery.distributors.release_slot(d->discovery_id);
    }

    ctx->module->distributors.release_slot(distributor);
}

void PondManager::api_distribute(pond_internal::ModuleContext* ctx, uint32_t distributor, void** slot_data)
{
    if (!ctx->module->distributors.is_used(distributor))
    {
        ctx->manager->log("Invalid distributor " + std::to_string(distributor) + " in module '" + ctx->module->name + "'");
        return;
    }
    auto d = &*ctx->module->distributors[distributor];

    {
        std::lock_guard<std::mutex> lock(d->new_connections_mutex);

        for (auto& connection : d->new_connections) d->connections.emplace_back(std::move(connection));
        d->new_connections.clear();
    }

    for (uint32_t i = 0; i < d->connections.size();)
    {
        auto& connection = d->connections[i];
        std::shared_lock<std::shared_mutex> lock(connection.callback->mutex);

        for (int i = 0; i < connection.slot_count; i++) connection.handle_array[i] = slot_data[connection.indices[i]];
        
        if (!connection.callback->active.load())
        {
            d->connections.erase(d->connections.begin() + i);
            continue;
        }

        connection.callback->callback(connection.callback->api, connection.callback->callback_pointer, connection.handle_array);
        i++;
    }
}

int32_t PondManager::api_create_callback(pond_internal::ModuleContext* ctx, pond_dds_slot_info* slots, uint32_t slot_count, pfn_pond_callback_function callback, void* callback_pointer)
{
    auto r = std::make_shared<pond_internal::Callback>();
    if (!ctx->manager->construct_slots(ctx->module, r->slots, slots, slot_count)) return -1;
    r->slot_count = slot_count;
    r->module_name = ctx->module->name;
    r->api = &ctx->module->native_api;
    r->active.store(true);
    r->callback = callback;
    r->callback_pointer = callback_pointer;

    if (ctx->manager->connect_log)
    {
        std::string channel_array_string;
        channel_array_string.reserve(1000);
        for (uint32_t i = 0; i < r->slot_count; i++)
        {
            channel_array_string.append(", ");
            channel_array_string.append(r->slots[i].channel);
        }
        ctx->module->native_api.log(ctx->module->native_api.ctx, (uint8_t*)"Receiving on channels { %s }", &(channel_array_string.c_str()[2]));
    }

    {
        std::shared_lock<std::shared_mutex> lock(ctx->manager->dds_discovery.distributor_mutex);
        for (auto& d : ctx->manager->dds_discovery.distributors)
            if (ctx->manager->try_connect_callback(d, r, true)) if (ctx->manager->connect_log)
            {
                std::string channel_array_string;
                channel_array_string.reserve(1000);
                for (uint32_t i = 0; i < r->slot_count; i++)
                {
                    channel_array_string.append(", ");
                    channel_array_string.append(r->slots[i].channel);
                }
                ctx->module->native_api.log(ctx->module->native_api.ctx, (uint8_t*)"Connected to distributor on module '%s' on channels { %s }", d->module_name.c_str(), &(channel_array_string.c_str()[2]));
            }
    }
    {
        std::lock_guard<std::shared_mutex> lock(ctx->manager->dds_discovery.callback_mutex);
        r->discovery_id = ctx->manager->dds_discovery.callbacks.insert(r);
    }

    return (int32_t)ctx->module->callbacks.emplace(r);
}

void PondManager::api_destroy_callback(pond_internal::ModuleContext* ctx, uint32_t callback)
{
    if (!ctx->module->callbacks.is_used(callback))
    {
        ctx->manager->log("Invalid callback " + std::to_string(callback) + " in module '" + ctx->module->name + "'");
        return;
    }
    auto r = &*ctx->module->callbacks[callback];

    {
        std::lock_guard<std::shared_mutex> lock(ctx->manager->dds_discovery.callback_mutex);
        ctx->manager->dds_discovery.callbacks.release_slot(r->discovery_id);
    }

    r->active.store(false);
    {std::lock_guard<std::shared_mutex> lock(r->mutex);}

    ctx->module->callbacks.release_slot(callback);
}