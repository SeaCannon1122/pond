#pragma once
#include "pond.hpp"
#include "data_types/robot_state_types.hpp"

namespace pond
{
    class ModuleBaseTF : public ModuleBase
    {
    public: 
        virtual pond_result onStartupTF(const std::vector<void*>& args);
        virtual void onShutdownTF();

        virtual pond_result onStartup(const std::vector<void*>& args) override;
        virtual void onShutdown() override;

        bool tfGetTransform(const std::string& source_frame, const std::string& target_frame, double time_point, GetFrameTransformRequest& request, bool verbose = true);
        bool tfGetJointInfo(const std::string& joint_name, GetJointInfoRequest& request, bool verbose = true);
        void tfSetJointState(const std::string& joint_name, double angle, double time);
        void tfSetJointStates(std::vector<JointState>& joint_states);

    private:
        Distributor<GetFrameTransformRequest> tf_request_distributor;
        Distributor<GetJointInfoRequest> joint_request_distributor;
        Distributor<std::vector<JointState>> joint_state_distributor;
        std::vector<JointState> joint_state;
    };
}

#ifdef POND_MODULE_CPP_MAKE_IMPLEMENTATION
namespace pond
{
    pond_result ModuleBaseTF::onStartupTF(const std::vector<void*>& args) {return POND_SUCCESS;}
    void ModuleBaseTF::onShutdownTF() {}

    pond_result ModuleBaseTF::onStartup(const std::vector<void*>& args)
    {
        tf_request_distributor = createDistributor<GetFrameTransformRequest>({"get_robot_transform"});
        joint_request_distributor = createDistributor<GetJointInfoRequest>({"get_robot_joint_info"});
        joint_state_distributor = createDistributor<std::vector<JointState>>({"joint_states"});
        joint_state.resize(1);
        if (onStartupTF(args) != POND_SUCCESS)
        {
            joint_state_distributor.destroy();
            joint_request_distributor.destroy();
            tf_request_distributor.destroy();
            return POND_ERROR;
        }
        return POND_SUCCESS;
    }

    void ModuleBaseTF::onShutdown()
    {
        onShutdownTF();
        joint_state_distributor.destroy();
        joint_request_distributor.destroy();
        tf_request_distributor.destroy();
    }

    bool ModuleBaseTF::tfGetTransform(const std::string& source_frame, const std::string& target_frame, double time_point, GetFrameTransformRequest& request, bool verbose)
    {
        request.source_frame = source_frame;
        request.target_frame = target_frame;
        request.time_point = time_point;

        tf_request_distributor.distribute(request);
        if (verbose && !request.fufilled) POND_LOG("Failed to get transform from '%s' to '%s' at timepoint %f", source_frame.c_str(), target_frame.c_str(), time_point);
        return request.fufilled;
    }

    bool ModuleBaseTF::tfGetJointInfo(const std::string& joint_name, GetJointInfoRequest& request, bool verbose)
    {
        request.joint_name = joint_name;
        joint_request_distributor.distribute(request);
        if (verbose && !request.fufilled) POND_LOG("Failed to get info about joint '%s'", joint_name.c_str());
        return request.fufilled;
    }

    void ModuleBaseTF::tfSetJointState(const std::string& joint_name, double angle, double time)
    {
        joint_state[0].time = time;
        joint_state[0].hw_time = time;
        joint_state[0].joint_name = joint_name;
        joint_state[0].angle = angle;
        joint_state_distributor.distribute(joint_state);
    }

    void ModuleBaseTF::tfSetJointStates(std::vector<JointState>& joint_states)
    {
        joint_state_distributor.distribute(joint_states);
    }
}

#endif