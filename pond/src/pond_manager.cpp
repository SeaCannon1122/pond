#include "pond_manager.hpp"


PondManager::PondManager()
{
    topics.create(32);
    distributor_topic_indicies.create(32);

    log("Started");
}

PondManager::~PondManager()
{
    topics.destroy();
    distributor_topic_indicies.destroy();

    log("Stopped");
}

void PondManager::log(const std::string& message)
{
    printf(("[POND_MANAGER] " + message + "\n").c_str());
    fflush(stdout);
}