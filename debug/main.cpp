#include "pond/pond.h"
#include <pond_manager/manager.hpp>

#include <chrono>
#include <signal.h>

std::atomic<bool> is_running{true};
int32_t int_counter = 0;

void signalHandler(int signal)
{
    if (signal != SIGINT && signal != SIGTERM)return;
    printf(" INTERRUPT\n"); fflush(stdout);
    is_running.store(false);

    int_counter++;
    if (int_counter == 10) exit(187);
}

int main()
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    {
        PondManager pm(false, false);

        pm.set_thread_frame_time("robot_state_thread", 1.0);

        pm.load_module(
            "robot_state_tracker",
            "robot_state",
            "state_tracker",
            "robot_state_thread",
            {{"description_path", pond_malloc_parameter_string((uint8_t*)"/home/pilot/.cache/robot.urdf")}}
        );

        pm.set_thread_frame_time("arm_thread", 0.1);

        uint8_t* joint_names[] = {(uint8_t*)"arm_segment_0_joint", (uint8_t*)"arm_segment_1_joint", (uint8_t*)"arm_segment_2_joint"};

        pm.load_module(
            "arm_controller",
            "controllers",
            "arm_controller",
            "arm_thread",
            {
                {"arm_joint_names", pond_malloc_parameter_string_array(joint_names, 3)},
                {"gripper_joint_name", pond_malloc_parameter_string((uint8_t*)"gripper_joint")},
                {"target_link_name", pond_malloc_parameter_string((uint8_t*)"gripper_target_link")}
            }
        );

        while (is_running.load()) std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}