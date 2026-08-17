#include <pond/pond.h>
#include "pond_manager.hpp"
#include <mutex>
#include <stdio.h>
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

    if (parameter) module->parameters[std::string((char*)name)] = parameter;
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

void construct_slots(pond_internal::Module* module, std::vector<pond_internal::Slot>& slots, pond_dds_slot_info* c_slots, uint32_t c_slot_count)
{
    slots.resize(c_slot_count);
    for (uint32_t i = 0; i < c_slot_count; i++)
    {
        slots[i].type = std::string((char*)c_slots[i].type);
        slots[i].topic = std::string((char*)c_slots[i].topic);

        auto it = module->topic_mappings.find(slots[i].topic);
        if (it != module->topic_mappings.end()) slots[i].topic = it->second;
    }
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

            if (d->slots[j].topic == r->slots[i].topic)
            {
                if (d->slots[j].type != "" && r->slots[i].type != "" && d->slots[j].type != r->slots[i].type)
                {
                    log("Mismatch of types on topic '" + d->slots[j].topic + "': '" + d->module_name + "' (" + d->slots[j].type + ") -> '" + r->slots[i].topic + "': '" + r->module_name + "' (" + r->slots[i].type + ")");
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
    construct_slots(module, d->slots, slots, slot_count);
    d->module_name = module->name;

    std::unordered_set<std::string> all_topics;
    for (auto& s : d->slots)
    {
        if (all_topics.find(s.topic) != all_topics.end())
        {
            log("In module " + module->name + ": can't publish on the same topic (" + s.topic + ") multiple times in parallel");
            return -1;
        }
        all_topics.insert(s.topic);
    }

    {
        std::shared_lock<std::shared_mutex> lock(dds_discovery.receiver_mutex);
        for (auto& r : dds_discovery.receivers)
            if (!try_connect_receiver(d, r, false)) return -1;
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
        for (int i = 0; i < d->slots.size(); i++) connection.handle_array[i] = slot_data[connection.indices[i]];
        
        std::shared_lock<std::shared_mutex> lock(connection.receiver->mutex);
        if (!connection.receiver->active.load()) continue;
        connection.receiver->callback(connection.receiver->api, connection.receiver->callback_pointer, connection.handle_array.data());
    }
}

int32_t PondManager::api_create_receiver(pond_internal::Module* module, pond_dds_slot_info* slots, uint32_t slot_count, pfn_pond_receiver_callback callback, void* callback_pointer)
{
    auto r = std::make_shared<pond_internal::Receiver>();
    construct_slots(module, r->slots, slots, slot_count);
    r->module_name = module->name;
    r->api = &module->native_api;
    r->active.store(true);
    r->callback = callback;
    r->callback_pointer = callback_pointer;

    {
        std::shared_lock<std::shared_mutex> lock(dds_discovery.distributor_mutex);
        for (auto& d : dds_discovery.distributors) try_connect_receiver(d, r, true);
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