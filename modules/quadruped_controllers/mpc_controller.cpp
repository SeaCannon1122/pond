#include <Eigen/src/Core/Matrix.h>
#include <pond/pond.hpp>
#include <pond_data_types/command_types.hpp>

#include <qpOASES.hpp>
#include <sophus/so3.hpp>

class MPCController : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;

    pond::Callback force_callback, twist_callback, orientation_callback;
    pond::Distributor force_distributor;
    std::vector<Eigen::Vector3d> forces;

    Sophus::SO3d orientation_received;

    TwistCommand twist_cmd, twist_cmd_received;
    std::mutex cmd_mutex;
    double cmd_timeout;
private:

    // Robot parameters (constant, static)
  double robot_m_;               // Total mass [kg]
  double robot_g_;               // Gravitational acceleration [m/s²]
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
  Eigen::MatrixXd mpc_Q_; // State weighting (13x13, diagonal)
  Eigen::MatrixXd mpc_R_; // Force weighting (12x12, R = α·I)
  double mpc_alpha_;      // α for R = α·I (Paper Eq. 28)

  // Common dimensions / buffer limits
  static constexpr int STATE_DIM = 13;
  static constexpr int INPUT_DIM = 12;
  static constexpr int MAX_N = 32; // max supported horizon length for buffers

  using Matrix13 = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
  using Matrix13x12 = Eigen::Matrix<double, STATE_DIM, INPUT_DIM>;
  using Matrix12 = Eigen::Matrix<double, INPUT_DIM, INPUT_DIM>;
  using Vector13 = Eigen::Matrix<double, STATE_DIM, 1>;

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

pond_result MPCController::onStartup(const std::vector<void*>& args)
{
    cmd_timeout = parameter("timeout").asDouble().get(0.5);
    
    forces.resize(4);
    force_distributor = createDistributor<std::vector<Eigen::Vector3d> >({"ground_forces"});

    force_callback = createCallback<double>({"force"}, [this](double* in){
        for (uint32_t i = 0; i < 4; i++) forces[i] = Eigen::Vector3d(0, 0, -*in);
        force_distributor.distribute(&forces);
    });

    twist_callback = createCallback<TwistCommand>({"cmd_vel"}, [this](TwistCommand* cmd) {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        twist_cmd_received = *cmd;
    });

    return POND_SUCCESS;
}

void MPCController::onShutdown()
{
    twist_callback.destroy();
    force_callback.destroy();
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

    static std::vector<bool> foot_contact = {true, true, true, true};
    
    try {
        POND_LOG("Iteration %d", iteration_count_);
        
        // === 2. REFERENCE GENERATION ===
        POND_LOG("Generating reference trajectory...");
        
        std::vector<Eigen::VectorXd> x_ref_sequence;
        std::vector<Matrix13x12> B_hat_sequence;
        
        double yaw_integrated = 0;
        Eigen::Vector2d pos_integrated = Eigen::Vector2d::Zero();
        
        POND_LOG("Starting from: yaw=%.3f, x=%.3f, y=%.3f", 
                   yaw_integrated, pos_integrated[0], pos_integrated[1]);
        
        for (int n = 0; n < mpc_N_; ++n) {
            // Build reference
            Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(13);
            x_ref(2) = yaw_integrated;
            x_ref(3) = pos_integrated[0];
            x_ref(4) = pos_integrated[1];
            x_ref(5) = 0.4;
            x_ref(8) = twist_cmd.ang[2];
            x_ref(9) = twist_cmd.lin[0];
            x_ref(10) = twist_cmd.lin[1];
            x_ref(12) = robot_g_;
            
            x_ref_sequence.push_back(x_ref);
            
            // Compute B_hat
            static const std::array<Eigen::Vector3d, 4> r_i_body = {
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
        x_current(2) = this->yaw;
        x_current(5) = 0.4;
        x_current(9) = this->odometry_data_->twist.twist.linear.x;
        x_current(10) = this->odometry_data_->twist.twist.linear.y;
        x_current(12) = robot_g_;
        
        POND_LOG("Current state: yaw=%.3f, vx=%.3f, vy=%.3f",
                   x_current(2), x_current(9), x_current(10));
        
        // === 4. BUILD AND SOLVE QP ===
        POND_LOG("Computing H and g...");
        
        Eigen::MatrixXd H = computeH(B_hat_sequence);
        Eigen::VectorXd g = computeg(x_current, B_hat_sequence, x_ref_sequence);
        
        POND_LOG("H: %dx%d, g: %td", 
                   H.rows(), H.cols(), g.size());
        
        // Quick sanity check
        if (H.rows() == 0 || H.cols() == 0) POND_LOG_RETURN("H is empty!");
        
        if (g.size() == 0) POND_LOG_RETURN("g is empty!");
        
        POND_LOG("Solving QP with constraints...");
        Eigen::VectorXd U_opt = solveQPWithConstraints(H, g, foot_contact);
        
        POND_LOG("Got solution of size %td", U_opt.size());
        
        // === 5. PUBLISH FORCES ===
        if (U_opt.size() >= 12) {
            Eigen::Map<Eigen::Matrix<double, 3, 4>> forces(U_opt.data());
            publishForces(forces);
            POND_LOG("Published forces");
        } else {
            POND_LOG("Solution too small: %td < 12", U_opt.size());
        }
        
        iteration_count_++;
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time);
        
        POND_LOG("MPC iteration complete: %.2f ms", duration.count()/1000.0);
        
    } catch (const std::exception &e) {
        POND_LOG("MPC error: %s", e.what());
    }
}