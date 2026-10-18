#include <pond/pond.hpp>
#include <pond_data_types/command_types.hpp>

enum GaitType
{
    TROT,
    WALK,
    AMBLE
};

class GaitScheduler : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
    std::atomic<int> gait_type{TROT};
    std::atomic<double> gait_duration{1.0};
    pond::Callback gait_type_callback, gait_duration_callback, schedule_callback, twist_callback;

    std::vector<double> phases, partial_durations;
};

POND_MODULE_CPP_DECLARE(GaitScheduler, "gait_scheduler", "scheduling different gates like trot, walk etc.")

pond_result GaitScheduler::onStartup(const std::vector<void*>& args)
{
    gait_type_callback = createCallback<std::string>({"gait_type"}, 
        [this](std::string* type_string){
            if (*type_string == "trot") gait_type.store(TROT); 
            else if (*type_string == "walk") gait_type.store(WALK);
            else if (*type_string == "amble") gait_type.store(AMBLE);
        }
    );

    gait_duration_callback = createCallback<double>({"gait_duration"},
        [this](double* duration) {gait_duration.store(*duration);}
    );

    phases.resize(4);
    partial_durations.resize(2);

    schedule_callback = createCallback<std::vector<double>*, std::vector<double>*, TwistCommand*>(
        {"schedule_gaits/gait_phases", "schedule_gaits/partial_gait_durations", "schedule_gaits/twist_command"},
        [this](std::vector<double>** gait_phases, std::vector<double>** partial_gait_durations, TwistCommand** twist_command)
        {
            *gait_phases = &phases;
            *partial_gait_durations = &partial_durations;

            double duration = gait_duration.load();
            partial_durations[0] = duration/2.0;
            partial_durations[1] = duration/2.0;

            switch (gait_type.load())
            {
                case TROT: {
                    gait_phases[0]
                } break;
                case WALK:
                case AMBLE:
            }
        }
    );

    return POND_SUCCESS;
}

void GaitScheduler::onShutdown()
{
    schedule_callback.destroy();
    gait_type_callback.destroy();
    gait_duration_callback.destroy();
}
