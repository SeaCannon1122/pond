#include "pond_manager.hpp"


PondManager::PondManager()
{


    log("Started");
}

PondManager::~PondManager()
{


    log("Stopped");
}

void PondManager::log(const std::string& message)
{
    printf(("[POND_MANAGER] " + message + "\n").c_str());
    fflush(stdout);
}