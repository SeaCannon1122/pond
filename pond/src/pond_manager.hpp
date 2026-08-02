#pragma once

#include <pond/pond.h>
#include "module.hpp"
#include "slot_array.hpp"
#include <string>
#include <mutex>
#include <unordered_map>
#include <thread>

namespace pond_interal
{
    class thread
    {
        std::mutex mutex;
        SlotArray<module*> modules;
        std::thread thread;
    };

    class receiver
    {
        pfn_pond_receiver_callback callback;
        void* callback_pointer;
    };

    class topic
    {
        int distributor_count;
        std::string name;
        SlotArray<uint32_t> subscriber_indicies;
    };
}

class PondManager
{
public:
    PondManager();
    ~PondManager();
    void load_module(
        const std::string& runtime_name,
        const std::string& bundle_name,
        const std::string& module_name,
        const std::string& thread_name
    );
    void unload_module(const std::string& runtime_name);
private:

    std::mutex dds_mutex;
    SlotArray<pond_interal::topic*> topics;
    SlotArray<pond_interal::receiver> receivers;
    SlotArray<uint32_t> distributor_topic_indicies;
    
    
    
    std::unordered_map<std::string, uint32_t> module_names_map;
};