#include "pond_manager.hpp"
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

void PondManager::api_shutdown(pond_internal::Module* module)
{
    module->should_shutdown.store(true);
}

void PondManager::api_log(pond_internal::Module* module, uint8_t* format, va_list args)
{

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

}

pond_parameter* PondManager::api_get_parameter(pond_internal::Module* module, uint8_t* name)
{
    return NULL;
}


bool PondManager::try_connect_receiver(std::shared_ptr<pond_internal::Distributor>& d, std::shared_ptr<pond_internal::Receiver>& r, bool to_new_connections)
{
    pond_internal::ReceiverConnection connection;
    connection.indices.resize(r->topics.size());
    connection.handle_array.resize(r->topics.size());

    for (int i = 0; i < r->topics.size(); i++)
    {
        for (int j = 0; ; j++)
        {
            if (j == d->topics.size()) return false;

            if (d->topics[j] == r->topics[i])
            {
                if (d->topic_types[j] != "" && r->topic_types[i] != "" && d->topic_types[j] != r->topic_types[i])
                {
                    log("Mismatch of types on topic '" + d->topics[j] + "': '" + d->module_name + "' (" + d->topic_types[j] + ") -> '" + r->module_name + "' (" + r->topic_types[j] + ")");
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

int32_t PondManager::api_create_distributor(pond_internal::Module* module, uint8_t** topics, uint32_t topic_count, uint8_t** topic_type_names)
{
    auto d = std::make_shared<pond_internal::Distributor>(); 
    d->topics.resize(topic_count);
    d->topic_types.resize(topic_count);
    d->module_name = module->name;

    for (int i = 0; i < topic_count; i++)
    {
        d->topics[i] = std::string((char*)topics[i]);
        for (auto& t : module->topic_mappings) if (d->topics[i] == t.first) d->topics[i] = t.second;
        d->topic_types[i] = std::string((char*)topic_type_names[i]);
    }

    {
        std::shared_lock<std::shared_mutex> lock(discovery.receiver_mutex);
        for (auto& r : discovery.receivers) try_connect_receiver(d, r, false);
    }

    {
        std::lock_guard<std::shared_mutex> lock(discovery.distributor_mutex);
        d->discovery_id = discovery.distributors.insert(d);
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
        std::lock_guard<std::shared_mutex> lock(discovery.distributor_mutex);
        discovery.distributors.release_slot(d->discovery_id);
    }

    module->distributors.release_slot(distributor);
}

void PondManager::api_distribute(pond_internal::Module* module, uint32_t distributor, void** data)
{
    if (!module->distributors.is_used(distributor))
    {
        log("Invalid distributor " + std::to_string(distributor) + " in module '" + module->name + "'");
        return;
    }
    auto d = &*module->distributors[distributor];

    {
        std::lock_guard<std::mutex> lock(d->new_connections_mutex);
        for (auto& connection : d->new_connections) d->connections.insert(connection);
    }

    for (auto& connection : d->connections)
    {
        for (int i = 0; i < d->topics.size(); i++) connection.handle_array[i] = data[connection.indices[i]];
        
        std::shared_lock<std::shared_mutex> lock(connection.receiver->mutex);
        if (!connection.receiver->active.load()) continue;
        connection.receiver->callback(connection.receiver->api, connection.receiver->callback_pointer, connection.handle_array.data());
    }
}

int32_t PondManager::api_create_receiver(pond_internal::Module* module, uint8_t** topics, uint32_t topic_count, uint8_t** topic_type_names, pfn_pond_receiver_callback callback, void* callback_pointer)
{
    auto r = std::make_shared<pond_internal::Receiver>();
    r->topics.resize(topic_count);
    r->topic_types.resize(topic_count);
    r->module_name = module->name;
    r->api = &module->native_api;

    for (int i = 0; i < topic_count; i++)
    {
        r->topics[i] = std::string((char*)topics[i]);
        for (auto& t : module->topic_mappings) if (r->topics[i] == t.first) r->topics[i] = t.second;
        r->topic_types[i] = std::string((char*)topic_type_names[i]);
    }

    {
        std::shared_lock<std::shared_mutex> lock(discovery.distributor_mutex);
        for (auto& d : discovery.distributors) try_connect_receiver(d, r, true);
    }

    {
        std::lock_guard<std::shared_mutex> lock(discovery.receiver_mutex);
        r->discovery_id = discovery.receivers.insert(r);
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
        std::lock_guard<std::shared_mutex> lock(discovery.receiver_mutex);
        discovery.receivers.release_slot(r->discovery_id);
    }

    r->active.store(false);
    {std::lock_guard<std::shared_mutex> lock(r->mutex);}

    module->receivers.release_slot(receiver);
}
