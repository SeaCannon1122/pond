#include <pond/pond.hpp>

class ControlFlowManager : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:

    std::vector<pond::Distributor> distributors;
    std::vector<void*> data_addresses;
    std::vector<void**> data_address_pointers;

    bool frame_time_log;
};

POND_MODULE_CPP_DECLARE(ControlFlowManager, "control_flow_manager", "connecting motors and controllers")

pond_result ControlFlowManager::onStartup(const std::vector<void*>& args)
{
    auto stage_names = parameter("stage_names").asStringArray().getStrict();
    auto data_names = parameter("data_names").asStringArray().getStrict();
    if (!stage_names || !data_names) return POND_ERROR;

    distributors.resize(stage_names->size());
    data_addresses.resize(data_names->size());
    data_address_pointers.resize(data_names->size());

    for (uint32_t i = 0; i < stage_names->size(); i++)
    {
        pond::ChannelsInfo info;
        for (auto& name : *data_names) info.channel<void>(stage_names->at(i) + "/" + name);

        distributors[i] = createDistributor(info);
    }

    for (uint32_t i = 0; i < data_names->size(); i++) data_address_pointers[i] = &data_addresses[i];

    frame_time_log = parameter("frame_time_log").asBool().get(false);

    return POND_SUCCESS;
}

void ControlFlowManager::onShutdown()
{
    for (uint32_t i = 0; i < distributors.size(); i++) distributors[i].destroy();
}

void ControlFlowManager::onFrame()
{
    double start = pond::get_monotonic_time();

    // reset interfaces;
    std::fill(data_addresses.begin(), data_addresses.end(), nullptr);

    // call stages
    for (auto& d : distributors) d.distribute_raw(data_address_pointers.data());

    double end = pond::get_monotonic_time();

    if (frame_time_log) POND_LOG("Frame took %f ms", (end-start)*1000.0);
}
