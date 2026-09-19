#include <pond/pond.hpp>

class ChannelFilter : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;

    pond::Distributor distributor;
    pond::Receiver receiver;
    uint32_t counter = 0;
    uint32_t rate;
};

POND_MODULE_CPP_DECLARE(ChannelFilter, "channel_filter", "filters the rate of data flow")

pond_result ChannelFilter::onStartup(const std::vector<void*>& args)
{
    auto channels_in = parameter("channels_in").asStringArray().getStrict(1);
    if (!channels_in) return POND_ERROR;

    auto channels_out = parameter("channels_out").asStringArray().getStrict(channels_in->size(), channels_in->size());
    if (!channels_in) return POND_ERROR;

    rate = parameter("rate").asInt().get(1);

    distributor = createDistributor(*channels_out);
    receiver = createReceiver(*channels_in, [this](void** data) {distributor.distribute_raw(data);});
    
    return POND_SUCCESS;
}

void ChannelFilter::onShutdown()
{
    receiver.destroy();
    distributor.destroy();
}