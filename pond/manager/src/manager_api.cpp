#include <pond_manager/manager.hpp>
#include <time.h>
#include <unordered_set>

void PondManager::api_shutdown(pond_internal::Module* module)
{
    module->should_shutdown.store(true);
}

void PondManager::api_log(pond_internal::Module* module, uint8_t* format, va_list args)
{
    uint8_t buffer[10000];
    vsnprintf((char*)buffer, sizeof(buffer), (char*)format, args);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    uint8_t* start = buffer;
    for (int i = 0;; i++) if (buffer[i] == '\n' || buffer[i] == 0)
    {
        bool end = (buffer[i] == 0);
        buffer[i] = 0;

        printf("[%s] [%lld.%06lld] %s\n", module->name.c_str(), (long long)ts.tv_sec, (long long)ts.tv_nsec / 1000, start);
        start = &buffer[i+1];

        if (end) break;
    }

    fflush(stdout);
}

void PondManager::api_set_user_ptr(pond_internal::Module* module, void* ptr)
{
    module->user_pointer = ptr;
}

void* PondManager::api_get_user_ptr(pond_internal::Module* module)
{
    return module->user_pointer;
}

void PondManager::api_set_parameter(pond_internal::Module* module, uint8_t* name, pond_parameter* parameter)
{
    std::lock_guard<std::mutex> lock(module->parameter_mutex);

    if (parameter)
    {
        if (auto it = module->parameters.find(std::string((char*)name)); it != module->parameters.end()) free(it->second);
        module->parameters[std::string((char*)name)] = parameter;
    }
    else module->parameters.erase(std::string((char*)name));
}

pond_parameter* PondManager::api_get_parameter(pond_internal::Module* module, uint8_t* name)
{
    std::lock_guard<std::mutex> lock(module->parameter_mutex);

    auto it = module->parameters.find(std::string((char*)name));

    if (it == module->parameters.end()) return NULL;
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

bool PondManager::construct_slots(pond_internal::Module* module, std::vector<pond_internal::Slot>& slots, pond_dds_slot_info* c_slots, uint32_t c_slot_count)
{
    slots.resize(c_slot_count);
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

bool PondManager::try_connect_receiver(std::shared_ptr<pond_internal::Distributor>& d, std::shared_ptr<pond_internal::Receiver>& r, bool to_new_connections)
{
    pond_internal::ReceiverConnection connection;
    connection.indices.resize(r->slots.size());
    connection.handle_array.resize(r->slots.size());

    for (int i = 0; i < r->slots.size(); i++)
    {
        for (int j = 0; ; j++)
        {
            if (j == d->slots.size()) return false;

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

    connection.receiver = r;

    if (to_new_connections)
    {
        std::lock_guard<std::mutex> lock(d->new_connections_mutex);
        d->new_connections.emplace(std::move(connection));
    }
    else d->connections.emplace(std::move(connection));
    
    return true;
}

int32_t PondManager::api_create_distributor(pond_internal::Module* module, pond_dds_slot_info* slots, uint32_t slot_count)
{
    auto d = std::make_shared<pond_internal::Distributor>();
    if (!construct_slots(module, d->slots, slots, slot_count)) return -1;
    d->module_name = module->name;

    std::unordered_set<std::string> all_channels;
    for (auto& s : d->slots)
    {
        if (all_channels.find(s.channel) != all_channels.end())
        {
            log("In module " + module->name + ": can't publish on the same channel (" + s.channel + ") multiple times in parallel");
            return -1;
        }
        all_channels.insert(s.channel);
    }
    
    if (connect_log)
    {
        std::string channel_array_string;
        channel_array_string.reserve(1000);
        for (auto& s: d->slots)
        {
            channel_array_string.append(", ");
            channel_array_string.append(s.channel);
        }
        module->native_api.log(module->native_api.ctx, (uint8_t*)"Distributing on channels { %s }", &(channel_array_string.c_str()[2]));
    }

    {
        std::shared_lock<std::shared_mutex> lock(dds_discovery.receiver_mutex);
        for (auto& r : dds_discovery.receivers)
            if (try_connect_receiver(d, r, false)) if (connect_log)
            {
                std::string channel_array_string;
                channel_array_string.reserve(1000);
                for (auto& s: r->slots)
                {
                    channel_array_string.append(", ");
                    channel_array_string.append(s.channel);
                }
                module->native_api.log(module->native_api.ctx, (uint8_t*)"Connected to receiver on module '%s' on channels { %s }", r->module_name.c_str(), &(channel_array_string.c_str()[2]));
            }
    }

    {
        std::lock_guard<std::shared_mutex> lock(dds_discovery.distributor_mutex);
        d->discovery_id = dds_discovery.distributors.insert(d);
    }

    return (int32_t)module->distributors.emplace(d);
}

void PondManager::api_destroy_distributor(pond_internal::Module* module, uint32_t distributor)
{
    if (!module->distributors.is_used(distributor))
    {
        log("Invalid distributor " + std::to_string(distributor) + " in module '" + module->name + "'");
        return;
    }
    auto d = &*module->distributors[distributor];

    {
        std::lock_guard<std::shared_mutex> lock(dds_discovery.distributor_mutex);
        dds_discovery.distributors.release_slot(d->discovery_id);
    }

    module->distributors.release_slot(distributor);
}

void PondManager::api_distribute(pond_internal::Module* module, uint32_t distributor, void** slot_data)
{
    if (!module->distributors.is_used(distributor))
    {
        log("Invalid distributor " + std::to_string(distributor) + " in module '" + module->name + "'");
        return;
    }
    auto d = &*module->distributors[distributor];

    {
        std::lock_guard<std::mutex> lock(d->new_connections_mutex);
        for (int i = 0; i < d->new_connections.get_length(); i++) if (d->new_connections.is_used(i))
        {
            d->connections.insert(d->new_connections[i]);
            d->new_connections.release_slot(i);
        }
    }

    for (auto& connection : d->connections)
    {
        for (int i = 0; i < connection.handle_array.size(); i++) connection.handle_array[i] = slot_data[connection.indices[i]];
        
        if (!connection.receiver->active.load()) continue;
        std::shared_lock<std::shared_mutex> lock(connection.receiver->mutex);
        if (!connection.receiver->active.load()) continue;
        
        if (distribute_log)
        {
            std::string channel_array_string;
            channel_array_string.reserve(1000);
            for (auto& s: connection.receiver->slots)
            {
                channel_array_string.append(", ");
                channel_array_string.append(s.channel);
            }
            module->native_api.log(module->native_api.ctx, (uint8_t*)"Distributing to module '%s' on channels { %s }", connection.receiver->module_name.c_str(), &(channel_array_string.c_str()[2]));
        }

        connection.receiver->callback(connection.receiver->api, connection.receiver->callback_pointer, connection.handle_array.data());
    }
}

int32_t PondManager::api_create_receiver(pond_internal::Module* module, pond_dds_slot_info* slots, uint32_t slot_count, pfn_pond_receiver_callback callback, void* callback_pointer)
{
    auto r = std::make_shared<pond_internal::Receiver>();
    if (!construct_slots(module, r->slots, slots, slot_count)) return -1;
    r->module_name = module->name;
    r->api = &module->native_api;
    r->active.store(true);
    r->callback = callback;
    r->callback_pointer = callback_pointer;

    if (connect_log)
    {
        std::string channel_array_string;
        channel_array_string.reserve(1000);
        for (auto& s: r->slots)
        {
            channel_array_string.append(", ");
            channel_array_string.append(s.channel);
        }
        module->native_api.log(module->native_api.ctx, (uint8_t*)"Receiving on channels { %s }", &(channel_array_string.c_str()[2]));
    }

    {
        std::shared_lock<std::shared_mutex> lock(dds_discovery.distributor_mutex);
        for (auto& d : dds_discovery.distributors)
            if (try_connect_receiver(d, r, true)) if (connect_log)
            {
                std::string channel_array_string;
                channel_array_string.reserve(1000);
                for (auto& s: r->slots)
                {
                    channel_array_string.append(", ");
                    channel_array_string.append(s.channel);
                }
                module->native_api.log(module->native_api.ctx, (uint8_t*)"Connected to distributor on module '%s' on channels { %s }", d->module_name.c_str(), &(channel_array_string.c_str()[2]));
            }
    }
    {
        std::lock_guard<std::shared_mutex> lock(dds_discovery.receiver_mutex);
        r->discovery_id = dds_discovery.receivers.insert(r);
    }

    return (int32_t)module->receivers.emplace(r);
}

void PondManager::api_destroy_receiver(pond_internal::Module* module, uint32_t receiver)
{
    if (!module->receivers.is_used(receiver))
    {
        log("Invalid receiver " + std::to_string(receiver) + " in module '" + module->name + "'");
        return;
    }
    auto r = &*module->receivers[receiver];

    {
        std::lock_guard<std::shared_mutex> lock(dds_discovery.receiver_mutex);
        dds_discovery.receivers.release_slot(r->discovery_id);
    }

    r->active.store(false);
    {std::lock_guard<std::shared_mutex> lock(r->mutex);}

    module->receivers.release_slot(receiver);
}