#include "pond/hpp/dds.hpp"
#include <pond/pond.hpp>

class ControlFlowManager : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:

    std::vector<pond::Distributor> stage_distributors;
    std::vector<pond::DistributorTyped<void>> data_distributors;
    std::vector<void*> data_addresses;
    std::vector<void**> data_address_pointers;
    std::vector<void**> out_data_address_pointers;

    bool frame_time_log;
};

POND_MODULE_CPP_DECLARE(ControlFlowManager, "control_flow_manager", "connecting motors and controllers")

pond_result ControlFlowManager::onStartup(const std::vector<void*>& args)
{
    auto stage_names = parameter("stage_names").asStringArray().getStrict();
    auto data_ids = parameter("data_ids").asStringArray().getStrict();
    auto data_out = parameter("data_out").asBoolArray().get({});
    if (!stage_names || !data_ids) return POND_ERROR;

    stage_distributors.resize(stage_names->size());
    data_distributors.reserve(data_ids->size());
    out_data_address_pointers.reserve(data_ids->size());
    data_addresses.resize(data_ids->size());
    data_address_pointers.resize(data_ids->size());

    for (uint32_t i = 0; i < stage_names->size(); i++)
    {
        pond::ChannelsInfo info;
        for (auto& name : *data_ids) info.channel<void>(stage_names->at(i) + "/" + name);

        stage_distributors[i] = createDistributor(info);
    }

    for (uint32_t i = 0; i < data_ids->size(); i++)
    {
        data_address_pointers[i] = &data_addresses[i];

        if (data_out.size() > i) if (data_out[i])
        {
            out_data_address_pointers.push_back(&data_addresses[i]);
            data_distributors.emplace_back(createDistributorTyped<void>({data_ids->at(i) + "/out"}));
        }
    }

    frame_time_log = parameter("frame_time_log").asBool().get(false);

    return POND_SUCCESS;
}

void ControlFlowManager::onShutdown()
{
    for (uint32_t i = 0; i < stage_distributors.size(); i++) stage_distributors[i].destroy();
    for (uint32_t i = 0; i < data_distributors.size(); i++) data_distributors[i].destroy();
}

void ControlFlowManager::onFrame()
{
    double start = pond::get_monotonic_time();

    // reset interfaces;
    std::fill(data_addresses.begin(), data_addresses.end(), nullptr);

    // call stages
    for (auto& d : stage_distributors) d.distribute_raw(data_address_pointers.data());

    double end = pond::get_monotonic_time();

    if (frame_time_log) POND_LOG("Frame took %f ms", (end-start)*1000.0);

    for (uint32_t i = 0; i < data_distributors.size(); i++) if (*out_data_address_pointers[i]) data_distributors[i].distribute(*out_data_address_pointers[i]);
}
