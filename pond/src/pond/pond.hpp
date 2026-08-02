#pragma once

#include <slot_array.hpp>
#include <atomic>

typedef struct managed_subscriber
{
    std::atomic<int>* mutex;
    SlotArray<>

} managed_subscriber;

typedef struct managed_topic
{
    uint8_t name[POND_MAX_TOPIC_NAME_LENGTH];
    SlotArray<uint32_t> subscriber_indicies;
} managed_topic;

class Pond
{
public:
    Pond();
    ~Pond();
private:
    SlotArray<managed_topic> topics;
    SlotArray<managed_subscriber> subscribers;
    SlotArray<uint32_t> publisher_topic_indicies;
};