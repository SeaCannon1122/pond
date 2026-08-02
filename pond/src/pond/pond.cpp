#include <pond.hpp>

Pond::Pond()
{
    topics.create(32);
    publisher_topic_indicies.create(32);
}

Pond::~Pond()
{
    for ()
    topics.destroy();
    publisher_topic_indicies.destroy()
}
