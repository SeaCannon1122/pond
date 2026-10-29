#include "pond/hpp/module_base.hpp"
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/command_types.hpp>

#include <qpOASES.hpp>

using Matrix13 = Eigen::Matrix<double, 13, 13>;
using Matrix13x12 = Eigen::Matrix<double, 13, 12>;
using Matrix12 = Eigen::Matrix<double, 12, 12>;
using Vector13 = Eigen::Matrix<double, 13, 1>;

class MPCController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
    virtual void onFrame() override;

    pond::Callback force_callback, twist_callback, orientation_callback, phases_callback;
    pond::Distributor force_distributor;
    std::vector<Eigen::Vector3d> forces;
    std::vector<bool> phases_buf; std::mutex phases_mutex;

    Sophus::SO3d orientation_received;

    TwistCommand twist_cmd, twist_cmd_received;
    std::mutex cmd_mutex;
    double cmd_timeout;
    uint32_t iteration_count = 0;

private:
    // Robot parameters (constant, static)
    double robot_m_;               // Total mass [kg]
    double robot_mu_;              // Friction coefficient
    double robot_f_min_;           // Minimum vertical force [N]
    double robot_f_max_;           // Maximum vertical force [N]
    Eigen::Matrix3d robot_I_body_; // Inertia tensor in body frame [kg·m²]

    // MPC parameters (constant for chosen gait)
    double mpc_dt_;    // Time step Δt [s] (0.03-0.05s)
    int mpc_N_;        // Horizon length k (10-16)
    int mpc_max_iter_; // Maximum QP iterations
    double mpc_tol_;   // Convergence tolerance

    // Q and R matrices (constant, diagonal)
    Matrix13 mpc_Q_; // State weighting (13x13, diagonal)
    Matrix12 mpc_R_; // Force weighting (12x12, R = α·I)
    double mpc_alpha_;      // α for R = α·I (Paper Eq. 28)

    // Common dimensions / buffer limits
    static constexpr int MAX_N = 32; // max supported horizon length for buffers
    static constexpr double G_ACCEL = 9.81;

    // Cached discrete A matrix
    Eigen::MatrixXd A_hat_cached_;
    bool A_hat_valid_ = false;

    // Preallocated buffers to avoid per-cycle allocations (dynamic to avoid huge stack objects)
    Eigen::MatrixXd B_qp_buffer_;
    Eigen::MatrixXd H_buffer_;
    Eigen::VectorXd g_buffer_;
    std::array<Matrix13, MAX_N> A_powers_buffer_;
};

POND_MODULE_CPP_DECLARE(MPCController, "mpc_controller", "model predictive control")

pond_result MPCController::onStartupTF(const std::vector<void*>& args)
{
    cmd_timeout = parameter("timeout").asDouble().get(0.5);
    robot_m_ = parameter("robot.mass").asDouble().get(5);
    robot_mu_ = parameter("robot.mu").asDouble().get(0.6); // Friction coefficient μ - typically 0.6 for rubber feet
    auto f_range = parameter("robot.f_range").asDoubleArray().get({0.0, 300.0}, 2, 2);
    robot_f_min_ = f_range[0]; robot_f_max_ = f_range[1];

    // Inertia tensor diagonal [Ixx, Iyy, Izz] in kg·m²
    // Cheetah 3 typical: [0.5, 0.8, 0.3]
    auto I_body_vec = parameter("robot.I_body").asDoubleArray().get({0.5, 0.8, 0.3}, 3, 3);
    robot_I_body_ = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) robot_I_body_(i, i) = I_body_vec[i];

    mpc_dt_ = parameter("mpc.dt").asDouble().get(0.03);           // Time step Δt [s] - Paper: 0.03-0.05s
    mpc_N_ = parameter("mpc.N").asInt().get(12);                  // Horizon length k - Paper: 10-16 steps
    mpc_max_iter_ = parameter("mpc.max_iter").asInt().get(100);
    mpc_tol_ = parameter("mpc.tol").asDouble().get(1e-4);         // Convergence tolerance
    mpc_alpha_ = parameter("mpc.alpha").asDouble().get(0.01);     // α for R = α·I (Paper Equation 28) - Force weighting
    mpc_R_ = mpc_alpha_ * Eigen::MatrixXd::Identity(12, 12);

    // Q-Matrix diagonal weights (13 states: Θ(3), P(3), ω(3), v(3), g(1))
    // Paper IV.B: only xy-velocity, xy-position, z, yaw, yaw-rate ≠ 0, rest 0
    auto Q_weights_vec = parameter("mpc.Q_weights").asDoubleArray().get(
        {
            0.0,  // φ (roll) - always 0 in reference
            0.0,  // θ (pitch) - always 0 in reference
            0.5,  // ψ (yaw) - important
            1.0,  // x position [m]
            1.0,  // y position [m]
            10.0, // z position [m] - VERY important for height control!
            0.05, // ω_x (roll rate) [rad/s] - always 0 in reference
            0.05, // ω_y (pitch rate) [rad/s] - always 0 in reference
            0.02, // ω_z (yaw rate) [rad/s] - important
            0.1,  // v_x (x velocity) [m/s] - important
            0.1,  // v_y (y velocity) [m/s] - important
            1.0,  // v_z (z velocity) [m/s] - always 0 in reference
            0.0   // g (gravity state) - constant
        }, 13, 13
    );

    mpc_Q_ = Matrix13::Zero();
    for (int i = 0; i < 13; ++i) mpc_Q_(i, i) = Q_weights_vec[i];

    forces.resize(4);
    force_distributor = createDistributor<std::vector<Eigen::Vector3d> >({"ground_forces"});

    force_callback = createCallback<double>({"force"}, [this](double* in){
        for (uint32_t i = 0; i < 4; i++) forces[i] = Eigen::Vector3d(0, 0, -*in);
        force_distributor.distribute(&forces);
    });

    phases_callback = createCallback<std::vector<bool>>({"gait_phases"}, [this](std::vector<bool>* in){
        std::lock_guard<std::mutex> lock(phases_mutex);
        phases_buf = *in;
    });

    twist_callback = createCallback<TwistCommand>({"cmd_vel"}, [this](TwistCommand* cmd) {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        twist_cmd_received = *cmd;
    });

    return POND_SUCCESS;
}

void MPCController::onShutdownTF()
{
    twist_callback.destroy();
    force_callback.destroy();
    phases_callback.destroy();
    force_distributor.destroy();
}

void MPCController::onFrame()
{
    double time_now = pond::get_time();

    cmd_mutex.lock();

    bool twist_usable = (std::abs(twist_cmd_received.lin[0]) > 0.001 | std::abs(twist_cmd_received.lin[1]) > 0.001 | std::abs(twist_cmd_received.ang[2]) > 0.001) && twist_cmd_received.stamp.time + cmd_timeout > time_now;
    if (twist_usable) twist_cmd = twist_cmd_received;
    else
    {
        twist_cmd.lin[0] = 0.0;
        twist_cmd.lin[1] = 0.0;
        twist_cmd.ang[2] = 0.0;
    }

    cmd_mutex.unlock();

    phases_mutex.lock();
    std::vector<bool> phases = {true, true, true, true};
    phases_mutex.unlock();

    try {
        POND_LOG("Iteration %d", iteration_count);
        
        double start_time = pond::get_monotonic_time();

        // === 2. REFERENCE GENERATION ===
        POND_LOG("Generating reference trajectory...");
        
        std::vector<Eigen::Vector<double, 13>> x_ref_sequence; x_ref_sequence.resize(mpc_N_, Eigen::Vector<double, 13>::Zero());
        std::vector<Matrix13x12> B_hat_sequence;
        
        double yaw_integrated = 0;
        Eigen::Vector2d pos_integrated = Eigen::Vector2d::Zero();
        
        for (uint32_t n = 0; n < mpc_N_; n++)
        {
            // Build reference
            x_ref_sequence[n](2) = yaw_integrated;
            x_ref_sequence[n](3) = pos_integrated[0];
            x_ref_sequence[n](4) = pos_integrated[1];
            x_ref_sequence[n](5) = 0.4;
            x_ref_sequence[n](8) = twist_cmd.ang[2];
            x_ref_sequence[n](9) = twist_cmd.lin[0];
            x_ref_sequence[n](10) = twist_cmd.lin[1];
            x_ref_sequence[n](12) = G_ACCEL;

            // Compute B_hat
            const std::array<Eigen::Vector3d, 4> r_i_body =
            {
                Eigen::Vector3d(0.3, 0.2, -0.4),
                Eigen::Vector3d(0.3, -0.2, -0.4),
                Eigen::Vector3d(-0.3, 0.2, -0.4),
                Eigen::Vector3d(-0.3, -0.2, -0.4)
            };
            
            auto yaw_rot = Sophus::SO3d::rotZ(yaw_integrated);

            std::array<Eigen::Vector3d, 4> r_i_world;
            for (int leg = 0; leg < 4; leg++) r_i_world[leg] = yaw_rot * r_i_body[leg];
            
            Matrix13x12 B_hat = computeBhat(yaw_integrated, r_i_world);
            B_hat_sequence.push_back(B_hat);
            
            // Update for next timestep
            yaw_integrated += twist_cmd.ang[2] * mpc_dt_;
            
            Eigen::Vector3d v_world = yaw_rot * twist_cmd.lin;
            pos_integrated[0] += v_world[0] * mpc_dt_;
            pos_integrated[1] += v_world[1] * mpc_dt_;
        }

        POND_LOG("Generated %td references", x_ref_sequence.size());

        // === 3. CURRENT STATE ===
        POND_LOG("Building current state...");

        Eigen::VectorXd x_current = Eigen::VectorXd::Zero(13);
        x_current(2) = 0; // yaw, set to zero for now, may be changed when there is a good odometry system
        x_current(5) = 0.4;
        x_current(9) = twist_cmd.lin[0];
        x_current(10) = twist_cmd.lin[1];
        x_current(12) = G_ACCEL;
        
        POND_LOG("Current state: yaw=%.3f, vx=%.3f, vy=%.3f", x_current(2), x_current(9), x_current(10));
        
        // === 4. BUILD AND SOLVE QP ===
        POND_LOG("Computing H and g...");

        Eigen::MatrixXd H = computeH(B_hat_sequence);
        Eigen::VectorXd g = computeg(x_current, B_hat_sequence, x_ref_sequence);

        POND_LOG("H: %dx%d, g: %td", H.rows(), H.cols(), g.size());

        // Quick sanity check
        if (H.rows() == 0 || H.cols() == 0) POND_LOG_RETURN("H is empty!");

        if (g.size() == 0) POND_LOG_RETURN("g is empty!");

        POND_LOG("Solving QP with constraints...");
        Eigen::VectorXd U_opt = solveQPWithConstraints(H, g, phases);

        POND_LOG("Got solution of size %td", U_opt.size());

        // === 5. PUBLISH FORCES ===
        if (U_opt.size() >= 12)
        {
            Eigen::Map<Eigen::Matrix<double, 3, 4>> forces_mat(U_opt.data());
            for (uint32_t i = 0; i < 4; i++) forces[i] = forces_mat.col(i);
        }
        else POND_LOG("Solution too small: %td < 12", U_opt.size());

        iteration_count++;

        POND_LOG("MPC iteration complete: %.2f ms", (pond::get_monotonic_time() - start_time)/1000.0);

    } catch (const std::exception &e) {POND_LOG("MPC error: %s", e.what());}
}