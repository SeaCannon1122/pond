#include "pond_data_types/transform_types.hpp"
#include <mutex>
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
    std::atomic<double> gait_duration{1.5};
    pond::Callback gait_type_callback, gait_duration_callback, schedule_callback, twist_callback, pose_callback;

    std::vector<double> phases, partial_durations;
    TwistCommand twist_cmd, twist_cmd_received;
    FrameTransform pose_tf, pose_tf_received;
    std::mutex cmd_mutex;

    double cmd_timeout;
};

POND_MODULE_CPP_DECLARE(GaitScheduler, "gait_scheduler", "scheduling different gates like trot, walk etc.")

pond_result GaitScheduler::onStartup(const std::vector<void*>& args)
{
    cmd_timeout = parameter("timeout").asDouble().get(0.5);

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

    twist_callback = createCallback<TwistCommand>({"cmd_vel"}, [this](TwistCommand* cmd) {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        twist_cmd_received = *cmd;
    });

    pose_callback = createCallback<FrameTransform>({"robot_pose"}, [this](FrameTransform* tf) {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        pose_tf_received = *tf;
    });

    phases.resize(4);
    // first stance, then swing
    partial_durations.resize(2);

    schedule_callback = createCallback<std::vector<double>*, std::vector<double>*, TwistCommand*, FrameTransform*>(
        {"schedule_gaits/gait_phases", "schedule_gaits/partial_gait_durations", "schedule_gaits/twist_command", "schedule_gaits/robot_pose"},
        [this](std::vector<double>** phases_out, std::vector<double>** partial_durations_out, TwistCommand** twist_cmd_out, FrameTransform** pose_tf_out)
        {
            *phases_out = &phases;
            *partial_durations_out = &partial_durations;
            *twist_cmd_out = &twist_cmd;
            *pose_tf_out = &pose_tf;

            double duration = gait_duration.load();
            double time_now = pond::get_time();

            // get input

            cmd_mutex.lock();

            bool twist_usable = (std::abs(twist_cmd_received.lin[0]) > 0.001 | std::abs(twist_cmd_received.lin[1]) > 0.001 | std::abs(twist_cmd_received.ang[2]) > 0.001) && twist_cmd_received.stamp.time + cmd_timeout > time_now;
            if (twist_usable) twist_cmd = twist_cmd_received;
            else
            {
                twist_cmd.lin[0] = 0.1;
                twist_cmd.lin[1] = 0.0;
                twist_cmd.ang[2] = 0.0;
            }

            if (pose_tf_received.stamp.time + cmd_timeout > time_now) pose_tf = pose_tf_received;
            
            cmd_mutex.unlock();

            // calculate phases

            if (twist_usable)
            {
                for (uint32_t i = 0; i < 4; i++) phases[i] = time_now;

                switch (gait_type.load())
                {
                    case TROT: {
                        partial_durations[0] = duration/2.0;
                        partial_durations[1] = duration/2.0;

                        phases[1] += duration/2.0;
                        phases[3] += duration/2.0;
                    } break;
                    case WALK: {
                        partial_durations[0] = 3*duration/4.0;
                        partial_durations[1] = duration/4.0;

                        phases[1] += 3.0*duration/4.0;
                        phases[2] += duration/2.0;
                        phases[3] += duration/4.0;
                    } break;
                    case AMBLE: {
                        phases[1] += duration/2.0;
                        phases[3] += duration/2.0;
                    } break;
                }

                for (uint32_t i = 0; i < 4; i++) phases[i] = fmod(phases[i], duration);
            }
            else
            {
                partial_durations[0] = 1.0;
                partial_durations[1] = 1.0;

                for (uint32_t i = 0; i < 4; i++) phases[i] = 0.5;
            }

        }
    );

    return POND_SUCCESS;
}

void GaitScheduler::onShutdown()
{
    schedule_callback.destroy();
    twist_callback.destroy();
    pose_callback.destroy();
    gait_type_callback.destroy();
    gait_duration_callback.destroy();
}
