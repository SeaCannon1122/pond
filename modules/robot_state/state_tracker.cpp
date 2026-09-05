#include <vector>
#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/pond.hpp>
#include <pond/data_types/robot_state_types.hpp>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <deque>
#include <fstream>
#include <sstream>
#include <shared_mutex>

struct Joint;

struct Link
{
    std::string name;
    std::shared_mutex tf_mutex;

    Link* parent_link;
    std::deque<std::tuple<double, Sophus::SE3d>> past_parent_transforms;
    Joint* parent_joint;

    std::vector<Link*> child_links;
    uint32_t level;
};

struct Joint
{
    std::string name;
    std::shared_mutex tf_mutex;
    
    bool is_static;
    Link* parent_link;
    Link* child_link;

    Sophus::SE3d tf;
    Eigen::Vector3d axis;
    std::deque<std::tuple<double, double>> past_states;
};

class StateTracker : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    pond::Receiver<GetFrameTransformRequest> tf_request_receiver;
    pond::Receiver<GetJointInfoRequest> joint_request_receiver;

    pond::Receiver<std::vector<JointState>> joint_states_receiver;
    
    pond::Distributor<std::vector<FrameTransform>> tf_distributor;
    pond::Distributor<std::vector<FrameTransform>> tf_static_distributor;

    std::string description;
    pond::Distributor<std::string> description_distributor;

    urdf::ModelInterfaceSharedPtr model;

    std::vector<Joint> joints;
    std::vector<Link> links;
    Link* root;
    std::unordered_map<std::string, Joint*> joints_map;
    std::unordered_map<std::string, Link*> links_map;

    double tf_store_duration;
};

POND_MODULE_CPP_DECLARE(StateTracker, "state_tracker", "tracks the transform state of the robot and makes it available to other modules")

POND_BUNDLE_DECLARE(
    "template bundle info", 
    1,
    POND_MODULE(StateTracker),
)

pond_result StateTracker::onStartup(const std::vector<void*>& args)
{
    bool verbose_model_info = parameter("verbose_model_info").asBool().get(false);

    auto description_path_o = parameter("description_path").asString().getStrict();
    if (!description_path_o) return POND_ERROR;
    
    tf_store_duration = parameter("tf_store_duration").asDouble().get(5);

    std::ifstream file(*description_path_o);

    if (!file.is_open())
    {
        POND_LOG("ERROR: Could not open file '%s'", *description_path_o->c_str());
        return POND_ERROR;
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();

    if (file.bad())
    {
        POND_LOG("ERROR: Could not read file '%s' as string", *description_path_o->c_str());
        return POND_ERROR;
    }

    description = buffer.str();

    if (!(model = urdf::parseURDF(description)))
    {
        POND_LOG("ERROR: Failed to parse urdf model");
        return POND_ERROR;
    }

    links = std::vector<Link>(model->links_.size());
    uint32_t links_i = 0;
    for (auto& l : model->links_) 
    {
        links_map[l.second->name] = &links[links_i];
        links_i++;
    }

    root = links_map[model->getRoot()->name];
    
    
    joints = std::vector<Joint>(model->joints_.size());
    uint32_t joints_i = 0;
    for (auto& j : model->joints_) 
    {
        joints_map[j.second->name] = &joints[joints_i];
        Joint* joint = &joints[joints_i];

        joint->name = j.second->name;
        joint->child_link = links_map[j.second->child_link_name];
        joint->parent_link = links_map[j.second->parent_link_name];
        joint->is_static = (j.second->type == urdf::Joint::FIXED);
        joint->axis = Eigen::Vector3d(j.second->axis.x, j.second->axis.y, j.second->axis.z).normalized();
        
        joint->tf = Sophus::SE3d(
            Eigen::Quaterniond(
                j.second->parent_to_joint_origin_transform.rotation.w,
                j.second->parent_to_joint_origin_transform.rotation.x,
                j.second->parent_to_joint_origin_transform.rotation.y,
                j.second->parent_to_joint_origin_transform.rotation.z
            ),
            Eigen::Vector3d(
                j.second->parent_to_joint_origin_transform.position.x,
                j.second->parent_to_joint_origin_transform.position.y,
                j.second->parent_to_joint_origin_transform.position.z
            )
        );

        if (verbose_model_info) POND_LOG("Joint: %s", joint->name.c_str());
        joints_i++;
    }

    for (auto& l : model->links_) 
    {
        Link* link = links_map[l.second->name];

        link->name = l.second->name;
        link->child_links.resize(l.second->child_joints.size());
        
        for (uint32_t i = 0; i < link->child_links.size(); i++)
            link->child_links[i] = links_map[l.second->child_joints[i]->child_link_name];
        
        if (l.second->parent_joint)
        {
            link->parent_link = links_map[l.second->parent_joint->parent_link_name];
            link->parent_joint = joints_map[l.second->parent_joint->name];

            if (link->parent_joint->is_static)
                link->past_parent_transforms.push_back({0, link->parent_joint->tf});
        }
        else
        {
            if (link != root)
            {
                POND_LOG("ERROR: Found unconnnected non root link '%s'", link->name.c_str());
                return POND_ERROR;
            }

            link->parent_link = NULL;
            link->parent_joint = NULL;
        }
        if (verbose_model_info) POND_LOG("Link: %s", link->name.c_str());
    }

    auto rec_set_link_level = [](Link* link, uint32_t level, auto&& self) -> void
    {
        link->level = level;
        for (auto& l : link->child_links) self(l, level+1, self);
    };

    rec_set_link_level(root, 0, rec_set_link_level);

    description_distributor = createDistributor<std::string>({"robot_description"});

    tf_request_receiver = createReceiver<GetFrameTransformRequest>({"get_robot_transform"}, [this](GetFrameTransformRequest* request) {

        auto source_it = links_map.find(request->source_frame);
        if (source_it == links_map.end()) return;

        auto target_it = links_map.find(request->target_frame);
        if (target_it == links_map.end()) return;

        Link* source = source_it->second; Link* target = target_it->second;
        
        Link* source_traverse = source; Link* target_traverse = target;
        Sophus::SE3d source_to_common, target_to_common;
        
        auto update_tf = [](Link* link, double time_point, Sophus::SE3d& tf) -> bool
        {
            std::shared_lock<std::shared_mutex> lock(link->tf_mutex);

            if (link->past_parent_transforms.empty()) return false;
            if (time_point == 0 && link->parent_joint->is_static)
            {
                tf = std::get<1>(link->past_parent_transforms.back()) * tf;
                return true;
            }
            if (time_point > std::get<0>(link->past_parent_transforms.back())) return false;
            
            auto it = link->past_parent_transforms.rbegin();
            for (;; it++)
            {
                if (it == link->past_parent_transforms.rend()) return false;
                if (time_point == std::get<0>(*it))
                {
                    tf = std::get<1>(*it) * tf;
                    return true;
                }
                if (time_point > std::get<0>(*it)) break;
            }

            auto& tf_older = *it;
            it--;
            auto& tf_newer = *it;

            double alpha = (time_point - std::get<0>(tf_older)) / (std::get<0>(tf_newer) - std::get<0>(tf_older));
                
            tf = (std::get<1>(tf_older) * Sophus::SE3d::exp(
                alpha * (std::get<1>(tf_older).inverse() * std::get<1>(tf_newer)).log()
            )) * tf;
            return true;
        };

        while (source_traverse->level > target_traverse->level)
        {
            if (!update_tf(source_traverse, request->time_point, source_to_common)) return;
            source_traverse = source_traverse->parent_link;
        }

        while (target_traverse->level > source_traverse->level)
        {
            if (!update_tf(target_traverse, request->time_point, target_to_common)) return;
            target_traverse = target_traverse->parent_link;
        }

        while (target_traverse != source_traverse)
        {
            if (!update_tf(source_traverse, request->time_point, source_to_common)) return;
            source_traverse = source_traverse->parent_link;

            if (!update_tf(target_traverse, request->time_point, target_to_common)) return;
            target_traverse = target_traverse->parent_link;
        }

        request->fufilled = true;
        request->tf = target_to_common.inverse() * source_to_common;
        
    });

    joint_request_receiver = createReceiver<GetJointInfoRequest>({"get_robot_joint_info"}, [this](GetJointInfoRequest* request) {
        const auto& joint = joints_map.find(request->joint_name);
        if (joint == joints_map.end()) return;

        request->parent_link_name = joint->second->parent_link->name;
        request->child_link_name = joint->second->child_link->name;
        request->is_static = joint->second->is_static;
        request->fufilled = true;
        request->tf = joint->second->tf;
    });

    tf_distributor = createDistributor<std::vector<FrameTransform>>({"tf"});
    tf_static_distributor = createDistributor<std::vector<FrameTransform>>({"tf_static"});

    joint_states_receiver = createReceiver<std::vector<JointState>>({"joint_states"}, [this](std::vector<JointState>* states){

        std::vector<FrameTransform> tfs; tfs.reserve(states->size());

        for (auto& state : *states)
        {
            const auto& joint = joints_map.find(state.joint_name);
            if (joint == joints_map.end()) return;

            FrameTransform tf;
            tf.stamp.frame_id = joint->second->parent_link->name;
            tf.stamp.time = state.time;
            tf.stamp.hw_time = state.hw_time;
            tf.child_frame_id = joint->second->child_link->name;
            tf.tf = joint->second->tf * Sophus::SE3d(Sophus::SO3d::exp(joint->second->axis * state.angle),Eigen::Vector3d::Zero());

            tfs.push_back(tf);

            std::unique_lock<std::shared_mutex> lock(joint->second->child_link->tf_mutex);
            joint->second->child_link->past_parent_transforms.push_back({state.time, tf.tf});
            while (state.time - std::get<0>(joint->second->child_link->past_parent_transforms.front()) > tf_store_duration) joint->second->child_link->past_parent_transforms.pop_front();
        }

        tf_distributor.distribute(tfs);
    });

    return POND_SUCCESS;
}

void StateTracker::onShutdown()
{
    joint_states_receiver.destroy();
    tf_request_receiver.destroy();
    joint_request_receiver.destroy();
    tf_distributor.destroy();
    tf_static_distributor.destroy();
    description_distributor.destroy();
}

void StateTracker::onFrame()
{
    std::vector<FrameTransform> static_tfs; static_tfs.reserve(100);
    double time = pond::get_time();

    for (auto& joint : joints) if (joint.is_static) static_tfs.push_back(
        {
            .stamp={.time = time, .hw_time = time, .frame_id = joint.parent_link->name}, 
            .child_frame_id = joint.child_link->name,
            .tf = joint.tf
        }
    );

    tf_static_distributor.distribute(static_tfs);
    description_distributor.distribute(description);
}
