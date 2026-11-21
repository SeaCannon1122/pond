#include <pond/hpp/module_base_tf.hpp>

struct SegmentData
{
    Eigen::Vector3d offset;

    double mass;
    Eigen::Vector3d com;
    Eigen::Matrix3d inertia;

    double joint_min, joint_max, effort_max;
};

struct LegConfig
{
    Eigen::Vector3d origin_pos;

    SegmentData segments[3];
};

class QuadrupedRobotConfig
{
public:
    bool init(pond::ModuleBaseTF& module)
    {
        base_link_name = module.parameter("base_link_name").asString().get("base_link");
        auto joint_names_o = module.parameter("joint_names").asStringArray().getStrict(12, 12); if (!joint_names_o) return false;
        auto foot_link_names_o = module.parameter("foot_link_names").asStringArray().getStrict(4, 4); if (!foot_link_names_o) return false;
        joint_names = *joint_names_o; foot_link_names = *foot_link_names_o;

        GetLinkInfoRequest link_request;
        if (!module.tfGetLinkInfo(base_link_name, link_request)) return false;
        if (!link_request.mass || !link_request.inertia) {module._pond_api.log(module._pond_api.ctx, (uint8_t *)"No mass/inertia set for %s", link_request.link_name.c_str()); return false;}
        
        body.mass = *link_request.mass;
        body.inertia = *link_request.inertia;

        for (uint32_t l = 0; l < 4; l++)
        {
            GetJointInfoRequest joint_requests[3];
            GetFrameTransformRequest tf_requests[4];

            for (uint32_t i = 0; i < 3; i++)
            {
                if (!module.tfGetJointInfo(joint_names[3*l+i], joint_requests[i])) return false;
                if (!joint_requests[i].min_angle || !joint_requests[i].max_angle || !joint_requests[i].max_effort) {module._pond_api.log(module._pond_api.ctx, (uint8_t *)"No joint position limits set for %s", joint_requests[i].joint_name.c_str()); return false;}
            
                legs[l].segments[i].joint_min = *joint_requests[i].min_angle;
                legs[l].segments[i].joint_max = *joint_requests[i].max_angle;
                legs[l].segments[i].effort_max = *joint_requests[i].max_effort;
            }

            if (!module.tfGetTransform(joint_requests[1].parent_link_name, joint_requests[0].child_link_name, 0, tf_requests[0])) return false;
            if (!module.tfGetTransform(joint_requests[2].parent_link_name, joint_requests[1].child_link_name, 0, tf_requests[1])) return false;
            if (!module.tfGetTransform(foot_link_names[l], joint_requests[2].child_link_name, 0, tf_requests[2])) return false;

            legs[l].segments[0].offset = (tf_requests[0].tf * joint_requests[1].tf).translation();
            legs[l].segments[1].offset = (tf_requests[1].tf * joint_requests[2].tf).translation();
            legs[l].segments[2].offset = (tf_requests[2].tf).translation();

            if (!module.tfGetTransform(joint_requests[0].parent_link_name, base_link_name, 0, tf_requests[3])) return false;
            legs[l].origin_pos = tf_requests[3].tf * joint_requests[0].tf * Eigen::Vector3d::Zero();

            GetLinkInfoRequest link_requests[3];
            for (uint32_t i = 0; i < 3; i++)
            {
                if (!module.tfGetLinkInfo(joint_requests[i].child_link_name, link_requests[i])) return false;
                if (!link_requests[i].mass || !link_requests[i].inertia_tf || !link_requests[i].inertia) {module._pond_api.log(module._pond_api.ctx, (uint8_t *)"No mass/inertia set for %s", link_requests[i].link_name.c_str()); return false;}
                
                legs[l].segments[i].mass = *link_requests[i].mass;
                legs[l].segments[i].com = link_requests[i].inertia_tf->translation();
                legs[l].segments[i].inertia = *link_requests[i].inertia;
            }
        }

        return true;
    }

    struct
    {
        double mass;
        Eigen::Matrix3d inertia;
    } body;

    LegConfig legs[4];

    std::string base_link_name;
    std::vector<std::string> joint_names, foot_link_names;
};