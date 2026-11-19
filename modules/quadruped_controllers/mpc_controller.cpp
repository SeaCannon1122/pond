#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/command_types.hpp>
#include <pond_data_types/trajectory_types.hpp>

#include <qpOASES.hpp>

using Vector13 = Eigen::Vector<double, 13>;
using Matrix13 = Eigen::Matrix<double, 13, 13>;
using Matrix13x12 = Eigen::Matrix<double, 13, 12>;
using MatrixXdRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

static_assert(std::is_same_v<qpOASES::real_t, double>, "qpOASES::real_t IS NOT double");

const int state_dim = 13, input_dim = 12;
const double G_ACCEL = 9.81;

class MPCController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
    virtual void onFrame() override;

private:
    pond::Callback update_callback, frame_state_callback;
    pond::DistributorTyped<std::vector<Eigen::Vector3d>> force_distributor;
    std::vector<Eigen::Vector3d> forces;

    TwistCommand twist_cmd_rec;
    FrameTransform pose_cmd_rec;
    std::array<double, 2> partial_durations_rec = {1.0, 0.0};
    std::array<double, 4> periods_rec = {0.0, 0.0, 0.0, 0.0};
    TrajectoryPoint current_trajectory_point_rec;
    std::mutex cmd_mutex, state_mutex;

    std::vector<std::string> foot_link_names; std::string reference_link;

    bool debug_log;

    double robot_mass, robot_mu;
    std::vector<double> robot_f_range;
    Eigen::Matrix3d robot_I_body;

    double mpc_alpha, mpc_dt; uint32_t mpc_N, mpc_max_iter;

    uint32_t nV;

    // work quantities
    std::vector<Matrix13> A_powers;
    std::vector<Vector13> x_ref_sequence;
    std::vector<Matrix13x12> B_hat_sequence;

    MatrixXdRM H, B_qp, A, Q;
    Eigen::VectorXd g, solution, errors, lbA, ubA, lb, ub;

    std::vector<std::array<bool, 4>> foot_contacts;
};

POND_MODULE_CPP_DECLARE(MPCController, "mpc_controller", "model predictive control")

pond_result MPCController::onStartupTF(const std::vector<void*>& args)
{
    auto foot_link_names_o = parameter("foot_link_names").asStringArray().getStrict(4, 4);
    auto reference_link_o = parameter("reference_link").asString().getStrict();
    if (!foot_link_names_o || !reference_link_o) return POND_ERROR;

    reference_link = *reference_link_o; foot_link_names = *foot_link_names_o;

    debug_log     = parameter("debug_log")    .asBool()       .get(false);
    robot_mass    = parameter("robot.mass")   .asDouble()     .get(5);
    robot_mu      = parameter("robot.mu")     .asDouble()     .get(0.6);
    robot_f_range = parameter("robot.f_range").asDoubleArray().get({0.0, 300.0}, 2, 2);

    auto I_body_vec = parameter("robot.I_body").asDoubleArray().get({0.5, 0.8, 0.3}, 3, 3);
    robot_I_body = Eigen::Map<Eigen::Vector3d>(I_body_vec.data()).asDiagonal();

    mpc_dt       = parameter("mpc.dt")      .asDouble().get(0.03);
    mpc_N        = parameter("mpc.N")       .asInt()   .get(12);
    mpc_max_iter = parameter("mpc.max_iter").asInt()   .get(100);
    mpc_alpha    = parameter("mpc.alpha")   .asDouble().get(0.0001);

    // Q-Matrix diagonal weights Θ(3), P(3), ω(3), v(3), g(1)
    auto Q_weights_vec = parameter("mpc.Q_weights").asDoubleArray().get({0.25, 0.25, 10.0, 2.0, 2.0, 50.0, 0.0, 0.0, 0.3, 0.2, 0.2, 0.1, 0.0}, 13, 13);
    Q = Eigen::Map<Vector13>(Q_weights_vec.data()).replicate(mpc_N, 1).asDiagonal();

    nV = input_dim * mpc_N;
    uint32_t max_nC = 4*6*mpc_N;

    foot_contacts.resize(mpc_N); x_ref_sequence.resize(mpc_N); B_hat_sequence.resize(mpc_N);

    H.resize(nV, nV);
    g.resize(nV);
    B_qp.resize(state_dim * mpc_N, nV);
    A_powers.resize(mpc_N+1);

    solution.resize(nV);
    errors.resize(state_dim * mpc_N);

    A.resize(max_nC, nV);
    lbA.resize(max_nC); ubA.resize(max_nC); lb.resize(nV); ub.resize(nV);
        
    forces.resize(4);
    force_distributor = createDistributorTyped<std::vector<Eigen::Vector3d>>({"ground_forces"});

    update_callback = createCallback<std::vector<double>*, std::vector<double>*, TwistCommand*, FrameTransform*>(
        {"update_mpc/gait_periods", "update_mpc/partial_gait_durations", "update_mpc/cmd_vel", "update_mpc/cmd_pose"},
        [this](std::vector<double>** periods_in, std::vector<double>** partial_durations_in, TwistCommand** twist_cmd_in, FrameTransform** pose_tf_in)
        {
            if (*periods_in == NULL)            POND_LOG_RETURN("update_mpc/gait_periods is NULL");
            if (*partial_durations_in == NULL)  POND_LOG_RETURN("update_mpc/partial_gait_durations is NULL");
            if (*twist_cmd_in == NULL)          POND_LOG_RETURN("update_mpc/cmd_vel is NULL");
            if (*pose_tf_in == NULL)            POND_LOG_RETURN("update_mpc/cmd_pose is NULL");

            std::lock_guard<std::mutex> lock(cmd_mutex);

            for (uint32_t i = 0; i < 2; i++) partial_durations_rec[i] = partial_durations_in[0][0][i];
            for (uint32_t i = 0; i < 4; i++) periods_rec[i] = periods_in[0][0][i];

            twist_cmd_rec = **twist_cmd_in;
            pose_cmd_rec = **pose_tf_in;
        }
    );

    frame_state_callback = createCallback<TrajectoryPoint>({reference_link+"_trajectory_point"}, [this](TrajectoryPoint* point_in) {
        std::lock_guard<std::mutex> lock(state_mutex);
        current_trajectory_point_rec = *point_in;
    });

    return POND_SUCCESS;
}

void MPCController::onShutdownTF()
{
    update_callback.destroy();
    frame_state_callback.destroy();
    force_distributor.destroy();
}

void MPCController::onFrame()
{
    double time_now = pond::get_time();

    cmd_mutex.lock();

    auto twist_cmd = twist_cmd_rec;
    auto pose_cmd = pose_cmd_rec;
    double gait_duration = partial_durations_rec[0] + partial_durations_rec[1];
    double stance_duration = partial_durations_rec[0];
    auto gait_periods = periods_rec;

    cmd_mutex.unlock();

    state_mutex.lock();
    TrajectoryPoint current_state = current_trajectory_point_rec;
    state_mutex.unlock();

    if (current_state.stamp.time == 0 || twist_cmd.stamp.time == 0) return;

    Eigen::Matrix3d current_rotation = current_state.pose.so3().matrix();
    Eigen::Vector3d current_angles = { // roll pitch yaw
        std::atan2(current_rotation(2,1), current_rotation(2,2)),
        -std::asin(std::clamp(current_rotation(2,0), -1.0, 1.0)),
        0
    };

    Eigen::Matrix3d pose_cmd_rotation = pose_cmd.tf.so3().matrix();
    Eigen::Vector3d pose_cmd_angles = pose_cmd_rotation.eulerAngles(2, 1, 0).reverse();

    GetFrameTransformRequest tf_request;
    std::array<Eigen::Vector3d, 4> foot_r_world;
    for (uint32_t i = 0; i < 4; i++)
    {
        if (!tfGetTransform(foot_link_names[i], reference_link, 0, tf_request)) return;
        foot_r_world[i] = current_rotation * tf_request.tf.translation();
    }

    double start_time = pond::get_monotonic_time();

    Matrix13 A_c = Matrix13::Zero();
    A_c.block<3,3>(0,6) = Sophus::SO3d::rotZ(twist_cmd.ang[2] * mpc_dt * (double)mpc_N / 2.0).matrix().transpose();
    A_c.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    A_c(11,12) = -1.0; 

    Matrix13 A_hat = Matrix13::Identity() + mpc_dt*A_c + 0.5*mpc_dt*mpc_dt*A_c*A_c;

    A_powers[0] = Matrix13::Identity();
    for (uint32_t n = 0; n < mpc_N; n++) A_powers[n+1] = A_powers[n] * A_hat;
    
    double yaw_integrated = 0;
    auto yaw_integrated_rot = Sophus::SO3d::rotZ(yaw_integrated);
    auto rotated_cmd_lin = yaw_integrated_rot * twist_cmd.lin;
    Eigen::Vector3d pos_integrated = Eigen::Vector3d::Zero();
    
    for (uint32_t n = 0; n < mpc_N; n++)
    {
        Eigen::Vector3d delta_pos = rotated_cmd_lin * mpc_dt;

        Matrix13x12 B_c = Matrix13x12::Zero();

        Eigen::Matrix3d I_hat_inv = (yaw_integrated_rot.matrix() * robot_I_body * yaw_integrated_rot.matrix().transpose()).inverse();

        for (uint32_t i = 0; i < 4; i++)
        {
            auto& r = foot_r_world[i];

            Eigen::Matrix3d r_cross; r_cross << 0, -r[2], r[1], r[2], 0, -r[0], -r[1], r[0], 0;
            
            B_c.block<3,3>(6, 3*i) = I_hat_inv * r_cross;
            B_c.block<3,3>(9, 3*i).setIdentity();
            B_c.block<3,3>(9, 3*i) /= robot_mass;

            r -= delta_pos;
        }
        
        yaw_integrated += twist_cmd.ang[2] * mpc_dt;
        pos_integrated += delta_pos;
        yaw_integrated_rot = Sophus::SO3d::rotZ(yaw_integrated);
        rotated_cmd_lin = yaw_integrated_rot * twist_cmd.lin;

        B_hat_sequence[n] = mpc_dt*B_c + 0.5*mpc_dt*mpc_dt*A_c*B_c;

        x_ref_sequence[n] <<
            pose_cmd_angles[0], pose_cmd_angles[1], yaw_integrated,
            pos_integrated[0], pos_integrated[1], pose_cmd.tf.translation().z(),
            twist_cmd.ang,
            rotated_cmd_lin[0], rotated_cmd_lin[1], 0.0,
            G_ACCEL;
    }

    Vector13 x_current; x_current << current_angles[0], current_angles[1], 0, 0, 0, current_state.pose.translation().z(), current_state.ang_vel, current_state.lin_vel, G_ACCEL;

    B_qp.setZero();
    for (uint32_t n = 0; n < mpc_N; n++) for (uint32_t j = 0; j <= n; j++)
        B_qp.block<state_dim, input_dim>(state_dim * n, input_dim * j) = A_powers[n - j] * B_hat_sequence[j];

    for (uint32_t n = 0; n < mpc_N; n++) errors.segment<state_dim>(state_dim * n) = A_powers[n + 1] * x_current - x_ref_sequence[n];

    H.setZero();
    H.selfadjointView<Eigen::Lower>().rankUpdate((Q.cwiseSqrt() * B_qp).transpose(), 2.0);
    H.triangularView<Eigen::StrictlyUpper>() = H.triangularView<Eigen::StrictlyLower>().transpose();
    H.diagonal().array() += 2.0 * mpc_alpha;

    g.noalias() = 2.0 * B_qp.transpose() * Q * errors;

    uint32_t nC = 0;

    // Count constraints for active feet only
    for (uint32_t n = 0; n < mpc_N; n++) for (uint32_t leg = 0; leg < 4; leg++)
        if (foot_contacts[n][leg] = (std::fmod(gait_periods[leg] + (double)n * mpc_dt, gait_duration) <= stance_duration))
            nC += 6;

    // Quick check for NaN in H and g or no stance feet
    if (bool H_nan = H.hasNaN(), g_nan = g.hasNaN(); H_nan || g_nan || nC == 0)
    {
        if (H_nan) POND_LOG("ERROR H contains NaN!");
        if (g_nan) POND_LOG("ERROR g contains NaN!");

        for (uint32_t i = 0; i < 4; i++) forces[i].setZero();
        force_distributor.distribute(&forces);

        return;
    }

    A.setConstant(0.0); 
    lbA.setConstant(-qpOASES::INFTY); ubA.setConstant(+qpOASES::INFTY);
    lb .setConstant(-qpOASES::INFTY); ub .setConstant(+qpOASES::INFTY);

    uint32_t constraint_idx = 0;
    
    for (uint32_t n = 0; n < mpc_N; n++) for (uint32_t leg = 0; leg < 4; leg++)
    {   
        uint32_t idx = n * 12 + leg * 3;

        if (foot_contacts[n][leg])
        {
            // 0)  f_z ≥ f_min      1)  f_z ≤ f_max  
            // 2)  f_x ≤ μ·f_z      3) -f_x ≤ μ·f_z
            // 4)  f_y ≤ μ·f_z      5) -f_y ≤ μ·f_z

            Eigen::Vector3d A_vals[] = {
                {0, 0, 1}, {0, 0, 1}, 
                {1, 0, -robot_mu}, {-1, 0, -robot_mu},
                {0, 1, -robot_mu}, {0, -1, -robot_mu}
            };

            Eigen::Vector2d bounds[] = {
                {robot_f_range[0], qpOASES::INFTY}, {-qpOASES::INFTY, robot_f_range[1]},
                {-qpOASES::INFTY, 0}, {-qpOASES::INFTY, 0},
                {-qpOASES::INFTY, 0}, {-qpOASES::INFTY, 0},
            };

            for (uint32_t i = 0; i < 6; i++, constraint_idx++)
            {
                A.block<1, 3>(constraint_idx, idx) = A_vals[i];
                lbA[constraint_idx] = bounds[i][0]; ubA[constraint_idx] = bounds[i][1];
            }
        }
        else lb[idx] = ub[idx] = lb[idx+1] = ub[idx+1] = lb[idx+2] = ub[idx+2] = 0.0; // Set force for airborne feet to 0
    }

    // === 3. SOLVE QP ===
    double qp_start_time = pond::get_monotonic_time();

    qpOASES::QProblem qp_solver(nV, nC);
    qpOASES::int_t nWSR = mpc_max_iter;
    
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = (debug_log ? qpOASES::PL_DEBUG_ITER : qpOASES::PL_NONE);
    options.enableRegularisation = qpOASES::BT_TRUE;  // DEBUG: Enable for stability
    options.enableFlippingBounds = qpOASES::BT_FALSE;
    qp_solver.setOptions(options);

    qpOASES::returnValue status = qp_solver.init(
        H.data(), g.data(), A.data(),
        lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR,
        NULL, NULL, NULL, NULL, NULL, NULL
    );
    
    if (status == qpOASES::SUCCESSFUL_RETURN || status == qpOASES::RET_MAX_NWSR_REACHED) qp_solver.getPrimalSolution(solution.data());
    else
    {
        POND_LOG("ERROR QP failed! Status: %d - %s", status, qpOASES::MessageHandling::getErrorCodeMessage(status));
        POND_LOG("Trying unconstrained Eigen fallback solution...");

        Eigen::LLT<Eigen::MatrixXd> llt(H + 1e-4 * Eigen::MatrixXd::Identity(nV, nV));
        
        if (llt.info() == Eigen::Success)
        {
            solution = llt.solve(-g);
            double fallback_norm = solution.norm();
            POND_LOG("Eigen fallback succeeded, solution norm: %.4f", fallback_norm);
        }
        else POND_LOG("ERROR Eigen LLT decomposition failed");
    }
    
    if (double solution_norm = solution.norm(); solution_norm < 1e-10) POND_LOG("Solution is all zeros (norm=%.2e)", solution_norm);
    
    if (debug_log) POND_LOG("QP solve time: %.2f ms (%d iterations)", (pond::get_monotonic_time() - qp_start_time)*1000.0, nWSR);

    for (uint32_t i = 0; i < 4; i++) forces[i] = solution.segment<3>(3 * i);
    force_distributor.distribute(&forces);

    if (debug_log) POND_LOG("MPC iteration complete: %.2f ms", (pond::get_monotonic_time() - start_time)*1000.0);
}