#include "pond_manager.hpp"
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <type_traits>
#include <vector>

void PondManager::api_module_shutdown(pond_internal::Module* module)
{

}

void PondManager::api_module_log(pond_internal::Module* module, uint8_t* format, va_list args)
{

}

void PondManager::api_module_set_user_ptr(pond_internal::Module* module, void* ptr)
{

}

void* PondManager::api_module_get_user_ptr(pond_internal::Module* module)
{

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
                if (d->topic_types[j] == r->topic_types[i])
                {
                    connection.indices[i] = j;
                    break;
                }
                else
                {
                    log("Mismatch of types on topic '" + d->topics[j] + "': '" + d->module_name + "' (" + d->topic_types[j] + ") -> '" + r->module_name + "' (" + r->topic_types[j] + ")");
                    return false;
                }
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
        d->topic_types[i] = std::string((char*)topic_type_names[i]);
    }

    {
        std::shared_lock<std::shared_mutex> lock(discovery.receiver_mutex);
        for (auto& r : discovery.receivers) try_connect_receiver(d, r, false);
    }

    discovery.distributor_mutex.lock();
    d->discovery_id = discovery.distributors.insert(d);
    discovery.distributor_mutex.unlock();
}

void PondManager::api_destroy_distributor(pond_internal::Module* module, uint32_t distributor)
{
    discovery.distributor_mutex.lock();
    discovery.distributor_mutex.unlock();
}

void PondManager::api_distribute(pond_internal::Module* module, uint32_t distributor, void** data)
{
    auto d = module->distributors[distributor];

    {
        std::lock_guard<std::mutex> lock(d->new_connections_mutex);
        for ()
    }

    d->connections_mutex.lock_shared();
    auto connections = d->connections;
    d->connections_mutex.unlock_shared();
}

int32_t PondManager::api_create_receiver(pond_internal::Module* module, uint8_t** topics, uint32_t topic_count, uint8_t** topic_type_names, pfn_pond_receiver_callback callback, void* callback_pointer)
{
    auto r = std::make_shared<pond_internal::Receiver>();
    r->topics.resize(topic_count);
    r->topic_types.resize(topic_count);
    r->module_name = module->name;

    for (int i = 0; i < topic_count; i++)
    {
        r->topics[i] = std::string((char*)topics[i]);
        r->topic_types[i] = std::string((char*)topic_type_names[i]);
    }

    {
        std::shared_lock<std::shared_mutex> lock(discovery.receiver_mutex);
        for (auto& d : discovery.distributors) try_connect_receiver(d, r, true);
    }


}

void PondManager::api_destroy_receiver(pond_internal::Module* module, uint32_t receiver)
{

}
