#include "pond_manager.hpp"

PondManager::PondManager()
{
    topics.create(32);
    distributor_topic_indicies.create(32);

    printf("[POND MANAGER] Started\n"); fflush(stdout);
}

PondManager::~PondManager()
{
    topics.destroy();
    distributor_topic_indicies.destroy();

    printf("[POND MANAGER] Stopped\n"); fflush(stdout);
}

void PondManager::load_module(
    const std::string& runtime_name,
    const std::string& bundle_name,
    const std::string& module_name,
    const std::string& thread_name
)
{

}

void PondManager::unload_module(const std::string& runtime_name)
{
    
}
