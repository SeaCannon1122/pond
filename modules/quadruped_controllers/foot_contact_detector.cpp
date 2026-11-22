#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/trajectory_types.hpp>
#include <sophus/so3.hpp>

#include "pond/h/module.h"
#include "quadruped_robot_config.hpp"

using Vector12 = Eigen::Vector<double, 12>;
using Vector18 = Eigen::Vector<double, 18>;
using Matrix18x18 = Eigen::Matrix<double, 18, 18>;
using Matrix3x18 = Eigen::Matrix<double, 3, 18>;

struct FullBodyDynamics
{
    Matrix18x18 M;
    Matrix18x18 C;
    Vector18 Ct_qdot;
    Vector18 g;
};

class FootContactDetector : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    FullBodyDynamics computeFullBodyDynamics(
        TrajectoryPoint& body_trajectory,
        std::array<Eigen::Vector3d, 4>& joint_positions,
        std::array<Eigen::Vector3d, 4>& joint_velocities
    );

    QuadrupedRobotConfig cfg;

    pond::Callback stage_callback;
};

POND_MODULE_CPP_DECLARE(FootContactDetector, "foot_contact_detector", "detects foot touchdown and lift")

pond_result FootContactDetector::onStartupTF(const std::vector<void*>& args)
{
    if (!cfg.init(*this)) return POND_ERROR;

    stage_callback = createCallback<std::array<Eigen::Vector3d, 4>*, std::array<Eigen::Vector3d, 4>*, TrajectoryPoint*>(
        {"detect_foot_contacts/joint_positions", "detect_foot_contacts/joint_velocities", "detect_foot_contacts/robot_trajectory_point"},
        [this](std::array<Eigen::Vector3d, 4>** joint_positions_in, std::array<Eigen::Vector3d, 4>** joint_velocities_in, TrajectoryPoint** trajectory_point_in)
        {
            if (*joint_positions_in == NULL) POND_LOG_RETURN("detect_foot_contacts/joint_positions is NULL");
            if (*joint_velocities_in == NULL) POND_LOG_RETURN("detect_foot_contacts/joint_velocities is NULL");
            if (*trajectory_point_in == NULL) POND_LOG_RETURN("detect_foot_contacts/robot_trajectory_point is NULL");

            FullBodyDynamics dynamics = computeFullBodyDynamics(**trajectory_point_in, **joint_positions_in, **joint_velocities_in); 
        }
    );

    return POND_SUCCESS;
}

void FootContactDetector::onShutdownTF()
{
    stage_callback.destroy();
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
    return (Eigen::Matrix3d() <<   0.0, -v.z(),   v.y(),
                                 v.z(),    0.0,  -v.x(),
                                -v.y(),  v.x(),     0.0).finished();
}

FullBodyDynamics FootContactDetector::computeFullBodyDynamics(
    TrajectoryPoint& body_trajectory,
    std::array<Eigen::Vector3d, 4>& joint_positions,
    std::array<Eigen::Vector3d, 4>& joint_velocities
)
{
    FullBodyDynamics dyn; dyn.M.setZero(); dyn.C.setZero(); dyn.Ct_qdot.setZero(); dyn.g.setZero();

    Eigen::Vector3d gravity_body = body_trajectory.pose.so3().inverse() * Eigen::Vector3d(0.0, 0.0, -9.81);

    // 1. Trunk (body 0, CoM at origin)
    {
        Matrix3x18 Jv = Matrix3x18::Zero(), Jw = Matrix3x18::Zero();
        Jv.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        Jw.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

        dyn.M += cfg.body.mass * Jv.transpose() * Jv + Jw.transpose() * cfg.body.inertia * Jw;
        dyn.g -= Jv.transpose() * (cfg.body.mass * gravity_body);
    }

    // 2. Four legs (3 links each): compute M, g, and dM/dq once per link
    std::array<Matrix18x18, 18> dM; dM.fill(Matrix18x18::Zero());

    for (uint32_t leg = 0; leg < 4; ++leg)
    {
        uint32_t col_offset = 6 + 3 * leg;

        Sophus::SO3d R[3];
        Eigen::Vector3d p[3];
        p[0] = cfg.legs[leg].origin_pos;
        R[0] = Sophus::SO3d::rotX(joint_positions[leg][0]);
        p[1] = p[0] + R[0] * cfg.legs[leg].segments[0].offset;
        R[1] = R[0] * Sophus::SO3d::rotY(joint_positions[leg][1]);
        p[2] = p[1] + R[1] * cfg.legs[leg].segments[1].offset;
        R[2] = R[1] * Sophus::SO3d::rotY(joint_positions[leg][2]);

        Eigen::Vector3d axis[3] = {Eigen::Vector3d::UnitX(), R[0] * Eigen::Vector3d::UnitY(), R[1] * Eigen::Vector3d::UnitY()};

        for (uint32_t b = 0; b < 3; b++)
        {
            auto& seg = cfg.legs[leg].segments[b];
            Eigen::Vector3d p_com = p[b] + R[b] * seg.com;
            Eigen::Matrix3d I_body = R[b].matrix() * seg.inertia * R[b].matrix().transpose();

            Matrix3x18 Jv = Matrix3x18::Zero(), Jw = Matrix3x18::Zero();
            Jv.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            Jv.block<3, 3>(0, 3) = -skew(p_com);
            Jw.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
            for (uint32_t i = 0; i <= b; i++)
            {
                Jv.col(col_offset + i) = axis[i].cross(p_com - p[i]);
                Jw.col(col_offset + i) = axis[i];
            }

            dyn.M += seg.mass * Jv.transpose() * Jv + Jw.transpose() * I_body * Jw;
            dyn.g -= Jv.transpose() * (seg.mass * gravity_body);

            for (uint32_t k = 0; k <= b; k++)
            {
                Eigen::Vector3d dp_com = axis[k].cross(p_com - p[k]);
                Eigen::Matrix3d A_k = skew(axis[k]);
                Eigen::Matrix3d dI_body = A_k * I_body - I_body * A_k;

                Matrix3x18 dJv = Matrix3x18::Zero(), dJw = Matrix3x18::Zero();
                dJv.block<3, 3>(0, 3) = -skew(dp_com);

                for (int i = 0; i <= b; ++i)
                {
                    Eigen::Vector3d da_i = (i > k) ? axis[k].cross(axis[i]) : Eigen::Vector3d::Zero();
                    Eigen::Vector3d dp_i = (i > k) ? axis[k].cross(p[i] - p[k]) : Eigen::Vector3d::Zero();
                    dJv.col(col_offset + i) = da_i.cross(p_com - p[i]) + axis[i].cross(dp_com - dp_i);
                    dJw.col(col_offset + i) = da_i;
                }

                dM[col_offset + k] +=
                    seg.mass * (dJv.transpose() * Jv + Jv.transpose() * dJv) +
                    dJw.transpose() * I_body * Jw +
                    Jw.transpose() * dI_body * Jw +
                    Jw.transpose() * I_body * dJw;
            }
        }
    }

    // 3. Joint-coordinate Christoffel part of C (diagnostics only, NOT complete for the base rows).
    //    The observer uses Ct_qdot below.
    for (uint32_t i = 0; i < 18; i++) for (uint32_t j = 0; j < 18; j++)
    {
        double cij = 0.0;
        for (uint32_t k = 0; k < 12; k++) cij += (dM[k](i, j) + dM[j](i, k) - dM[i](j, k)) * joint_positions[k/3][k%3];
        dyn.C(i, j) = 0.5 * cij;
    }

    Vector18 q_dot; q_dot << body_trajectory.lin_vel, body_trajectory.ang_vel, joint_velocities[0], joint_velocities[1], joint_velocities[2], joint_velocities[3];

    // 4. Non-force momentum rate, so that  p_dot = S^T tau + Ct_qdot - g + tau_d,  p = M * q_dot
    Vector18 p_total = dyn.M * q_dot;
    Eigen::Vector3d p_v = p_total.head<3>();          // linear momentum (body frame)
    Eigen::Vector3d p_w = p_total.segment<3>(3);      // angular momentum about body origin

    // Base rows: Newton-Euler in a rotating, translating frame
    dyn.Ct_qdot.head<3>()     = -body_trajectory.ang_vel.cross(p_v);
    dyn.Ct_qdot.segment<3>(3) = -body_trajectory.ang_vel.cross(p_w) - body_trajectory.lin_vel.cross(p_v);

    // Joint rows: 0.5 * nu^T (dM/dq_j) nu, with the FULL twist (includes base-joint coupling)
    for (uint32_t j = 6; j < 18; j++) dyn.Ct_qdot[j] = 0.5 * q_dot.dot(dM[j] * q_dot);

    return dyn;
}