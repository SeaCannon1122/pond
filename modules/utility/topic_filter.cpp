#include <pond/pond.hpp>

class TopicFilter : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;

    int32_t distributor, receiver;
    uint32_t counter = 0;
    uint32_t rate;
};

POND_MODULE_CPP_DECLARE(TopicFilter, "topic_filter", "filters the rate of data flow")

void callback(pond_api* api, TopicFilter* filter, void** slot_data)
{
    if (filter->counter == 0) api->distribute(api->ctx, filter->distributor, slot_data);

    filter->counter = (filter->counter + 1) % filter->rate; 
}

pond_result TopicFilter::onStartup(const std::vector<void*>& args)
{
    auto topics_in = parameter("topics_in").asStringArray().get({"in"}, 1);
    auto topics_out = parameter("topics_out").asStringArray().get({"out"}, topics_in.size(), topics_in.size());
    rate = parameter("rate").asInt().get(1);

    std::vector<pond_dds_slot_info> infos_in(topics_in.size()), infos_out(topics_in.size());

    for (int i = 0; i < topics_in.size(); i++)
    {
        infos_in[i].topic = (uint8_t*)topics_in[i].c_str();
        infos_in[i].type = (uint8_t*)"";
        infos_out[i].topic = (uint8_t*)topics_out[i].c_str();
        infos_out[i].type = (uint8_t*)"";
    }

    distributor = _pond_api.create_distributor(_pond_api.ctx, infos_out.data(), topics_in.size());
    receiver = _pond_api.create_receiver(_pond_api.ctx, infos_in.data(), topics_in.size(), (pfn_pond_receiver_callback)callback, (void*)this);
    return POND_SUCCESS;
}

void TopicFilter::onShutdown()
{
    _pond_api.destroy_receiver(_pond_api.ctx, receiver);
    _pond_api.destroy_distributor(_pond_api.ctx, distributor);
}