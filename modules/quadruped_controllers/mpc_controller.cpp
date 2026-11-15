#include <Eigen/src/Core/Matrix.h>
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/command_types.hpp>

#include <qpOASES.hpp>
#include <qpOASES/Types.hpp>

using Vector12 = Eigen::Vector<double, 12>;
using Vector13 = Eigen::Vector<double, 13>;
using Matrix13 = Eigen::Matrix<double, 13, 13>;
using Matrix13x12 = Eigen::Matrix<double, 13, 12>;
using MatrixXdRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

const int state_dim = 13, input_dim = 12;
const double G_ACCEL = 9.81;

class MPCController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
    virtual void onFrame() override;

    pond::Callback update_callback;
    pond::DistributorTyped<std::vector<Eigen::Vector3d>> force_distributor;
    std::vector<Eigen::Vector3d> forces;

    TwistCommand twist_cmd_rec, last_twist_cmd{.lin = Eigen::Vector3d(-99999, 0, 0)};
    std::array<double, 2> partial_durations_rec = {1.0, 0.0};
    std::array<double, 4> periods_rec = {0.0, 0.0, 0.0, 0.0};
    std::mutex mutex;

private:
    bool debug_log;
    // Robot parameters (constant, static)
    double robot_mass, robot_mu; // robot mass, Friction coefficient
    std::vector<double> robot_f_range; // Minimum / Maximum vertical force
    Eigen::Matrix3d robot_I_body_; // Inertia tensor diagonal [Ixx, Iyy, Izz], Cheetah 3 typical: [0.5, 0.8, 0.3]

    // MPC parameters (constant for chosen gait)
    double mpc_alpha_, mpc_dt_, mpc_N_, mpc_max_iter_; // α for R = α·I (Paper Eq. 28), Time step Δt (0.03-0.05s), Horizon length (10-16), Maximum QP iterations

    // Q and R matrix diagonals
    Vector13 mpc_Q_diag_; Vector12 mpc_R_diag_; // State weighting (13x13, diagonal) Force weighting (12x12, R = α·I)

    uint32_t nV;

    // work quantities
    Matrix13 A_c, A_hat;
    std::vector<Matrix13> A_powers;
    Matrix13x12 B_c;
    std::vector<Vector13> x_ref_sequence;
    std::vector<Matrix13x12> B_hat_sequence;

    MatrixXdRM H, B_qp, A;
    Eigen::VectorXd g, solution;

    std::vector<std::array<bool, 4>> foot_contacts;

    std::vector<qpOASES::real_t> x_opt, lbA, ubA, lb, ub;
};

POND_MODULE_CPP_DECLARE(MPCController, "mpc_controller", "model predictive control")

pond_result MPCController::onStartupTF(const std::vector<void*>& args)
{
    debug_log = parameter("debug_log").asBool().get(false);
    robot_mass = parameter("robot.mass").asDouble().get(5);
    robot_mu = parameter("robot.mu").asDouble().get(0.6); // Friction coefficient μ - typically 0.6 for rubber feet
    robot_f_range = parameter("robot.f_range").asDoubleArray().get({0.0, 300.0}, 2, 2);

    auto I_body_vec = parameter("robot.I_body").asDoubleArray().get({0.5, 0.8, 0.3}, 3, 3);
    robot_I_body_ = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) robot_I_body_(i, i) = I_body_vec[i];

    mpc_dt_ = parameter("mpc.dt").asDouble().get(0.03);           // Time step Δt [s] - Paper: 0.03-0.05s
    mpc_N_ = parameter("mpc.N").asInt().get(12);                  // Horizon length k - Paper: 10-16 steps
    mpc_max_iter_ = parameter("mpc.max_iter").asInt().get(100);
    mpc_alpha_ = parameter("mpc.alpha").asDouble().get(0.0001);     // α for R = α·I (Paper Equation 28) - Force weighting
    mpc_R_diag_.setOnes(); mpc_R_diag_ *= mpc_alpha_;

    nV = input_dim * mpc_N_;
    uint32_t max_nC = 4*6*mpc_N_;

    foot_contacts.resize(mpc_N_); x_ref_sequence.resize(mpc_N_); B_hat_sequence.resize(mpc_N_);

    H.resize(nV, nV);
    g.resize(nV);
    B_qp.resize(state_dim * mpc_N_, nV);
    A_powers.resize(mpc_N_+1);

    solution.resize(nV);
    x_opt.resize(nV);

    A.resize(max_nC, nV);
    lbA.resize(max_nC); ubA.resize(max_nC); lb.resize(nV); ub.resize(nV);
        
    // Q-Matrix diagonal weights (13 states: Θ(3), P(3), ω(3), v(3), g(1))
    // Paper IV.B: only xy-velocity, xy-position, z, yaw, yaw-rate ≠ 0, rest 0
    auto Q_weights_vec = parameter("mpc.Q_weights").asDoubleArray().get(
        {
            0.8,  // φ (roll) - always 0 in reference
            0.8,  // θ (pitch) - always 0 in reference
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
        }, state_dim, state_dim
    );

    for (uint32_t i = 0; i < 13; i++) mpc_Q_diag_[i] = Q_weights_vec[i];

    forces.resize(4);
    force_distributor = createDistributorTyped<std::vector<Eigen::Vector3d>>({"ground_forces"});

    update_callback = createCallback<std::vector<double>*, std::vector<double>*, TwistCommand*, FrameTransform*>(
        {"update_mpc/gait_periods", "update_mpc/partial_gait_durations", "update_mpc/twist_command", "update_mpc/robot_pose"},
        [this](std::vector<double>** periods_in, std::vector<double>** partial_durations_in, TwistCommand** twist_cmd_in, FrameTransform** pose_tf_in)
        {
            if (*periods_in == NULL) POND_LOG_RETURN("update_mpc/gait_periods is NULL");
            if (*partial_durations_in == NULL) POND_LOG_RETURN("update_mpc/partial_gait_durations is NULL");
            if (*twist_cmd_in == NULL) POND_LOG_RETURN("update_mpc/twist_command is NULL");
            if (*pose_tf_in == NULL) POND_LOG_RETURN("update_mpc/robot_pose is NULL");

            std::lock_guard<std::mutex> lock(mutex);

            for (uint32_t i = 0; i < 2; i++) partial_durations_rec[i] = partial_durations_in[0][0][i];
            for (uint32_t i = 0; i < 4; i++) periods_rec[i] = periods_in[0][0][i];

            twist_cmd_rec = **twist_cmd_in;
        }
    );

    return POND_SUCCESS;
}

void MPCController::onShutdownTF()
{
    update_callback.destroy();
    force_distributor.destroy();
}

void MPCController::onFrame()
{
    double time_now = pond::get_time();

    mutex.lock();

    auto twist_cmd = twist_cmd_rec;
    double gait_duration = partial_durations_rec[0] + partial_durations_rec[1];
    double stance_duration = partial_durations_rec[0];
    auto gait_periods = periods_rec;

    mutex.unlock();

    bool new_twist = ((twist_cmd.lin - last_twist_cmd.lin).norm() + (twist_cmd.ang - last_twist_cmd.ang).norm() > 0.001);
    if (new_twist) last_twist_cmd = twist_cmd;

    try {
        double start_time = pond::get_monotonic_time();

        if (new_twist)
        {
            A_c.setZero();
            A_c.block<3,3>(0,6) = Sophus::SO3d::rotZ(twist_cmd.ang[2] * mpc_dt_ * (double)mpc_N_ / 2.0).matrix().transpose();
            A_c.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
            A_c(11,12) = -1.0; 

            A_hat = Matrix13::Identity() + mpc_dt_*A_c + 0.5*mpc_dt_*mpc_dt_*A_c*A_c;

            A_powers[0] = Matrix13::Identity();
            for (uint32_t n = 0; n < mpc_N_; n++) A_powers[n+1] = A_powers[n] * A_hat;
        }
        
        double yaw_integrated = 0;
        Eigen::Vector3d pos_integrated = Eigen::Vector3d::Zero();
        
        for (uint32_t n = 0; n < mpc_N_; n++)
        {
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

            // calculate B_c
            B_c.setZero();

            Eigen::Matrix3d I_hat_inv = (yaw_rot.matrix() * robot_I_body_ * yaw_rot.matrix().transpose()).inverse();

            for (uint32_t i = 0; i < 4; i++)
            {
                const Eigen::Vector3d& r = r_i_world[i];
                Eigen::Matrix3d r_cross;
                r_cross <<    0, -r[2],  r[1],
                           r[2],     0, -r[0],
                          -r[1],  r[0],     0;
                
                B_c.block<3,3>(6, 3*i) = I_hat_inv * r_cross;
                B_c.block<3,3>(9, 3*i).setIdentity();
                B_c.block<3,3>(9, 3*i) /= robot_mass;
            }
            
            B_hat_sequence[n] = mpc_dt_*B_c + 0.5*mpc_dt_*mpc_dt_*A_c*B_c;
            
            // integrate timestep
            yaw_integrated += twist_cmd.ang[2] * mpc_dt_;
            pos_integrated += yaw_rot * twist_cmd.lin * mpc_dt_;

            auto rotated_cmd_lin = Sophus::SO3d::rotZ(yaw_integrated) * twist_cmd.lin;

            // Build reference
            x_ref_sequence[n] = Vector13{
                0.0, 0.0, yaw_integrated, // rotation
                pos_integrated[0], pos_integrated[1], 0.4, // position
                0.0, 0.0, twist_cmd.ang[2], //  ang vel
                rotated_cmd_lin[0], rotated_cmd_lin[1], 0.0, // lin vel
                G_ACCEL // gravitational acceleration
            };
        }

        H.setZero(); g.setZero(); B_qp.setZero();
        
        for (uint32_t n = 0; n < mpc_N_; n++) for (uint32_t j = 0; j <= n; j++)
            B_qp.block<state_dim, input_dim>(state_dim * n, input_dim * j) = A_powers[n-j] * B_hat_sequence[j];

        Vector13 x_current{
            0.0, 0.0, 0.0, // rotation
            0.0, 0.0, 0.4, // position
            0.0, 0.0, 0.0, //  ang vel
            0, 0, 0.0, // lin vel
            G_ACCEL // gravitational acceleration
        };

        // 5. BUILD H with symmetric updates
        // R-contribution first (block diagonal)
        for (uint32_t blk = 0; blk < mpc_N_; blk++) H.block<input_dim, input_dim>(input_dim * blk, input_dim * blk).diagonal() = 2.0 * mpc_R_diag_;
        
        // Q-contribution: Use symmetry - only fill lower triangular matrix
        for (uint32_t n = 0; n < mpc_N_; n++)
        {
            Eigen::VectorXd x_pred = A_powers[n+1] * x_current;

            for (uint32_t s = 0; s < state_dim; s++)
            {
                double q = mpc_Q_diag_[s];
                if (q == 0.0) continue;
                
                Eigen::RowVectorXd v_row = B_qp.row(state_dim * n + s);

                double err = x_pred(s) - x_ref_sequence[n](s);
                if (std::abs(err) > 1e-10) g.noalias() += 2.0 * q * err * v_row.transpose();

                // Symmetric update: H += 2q * (v^T * v)
                // Only fill lower triangular matrix
                for (uint32_t col = 0; col < nV; col++) if (v_row[col] != 0.0)
                    for (uint32_t row = col; row < nV; row++) H(row, col) += 2.0 * q * v_row[col] * v_row[row];
            }
        }
    
        // Mirror upper triangular matrix
        H.triangularView<Eigen::StrictlyUpper>() = H.triangularView<Eigen::StrictlyLower>().transpose();

        uint32_t nC = 0;

        // Count constraints for active feet only
        for (uint32_t n = 0; n < mpc_N_; n++) for (uint32_t leg = 0; leg < 4; leg++)
        {
            if (std::fmod(gait_periods[leg] + (double)n * mpc_dt_, gait_duration) <= stance_duration)
            {
                foot_contacts[n][leg] = true;
                nC += 6;  // 6 constraints per foot
            }
            else foot_contacts[n][leg] = false;
        }

        // Quick check for NaN in H and g
        if (bool H_nan = H.hasNaN(), g_nan = g.hasNaN(); H_nan || g_nan || nC == 0)
        {
            if (H_nan) POND_LOG("ERROR H contains NaN!");
            if (g_nan) POND_LOG("ERROR g contains NaN!");

            for (uint32_t i = 0; i < 4; i++) forces[i].setZero();
            force_distributor.distribute(&forces);

            return;
        }

        std::fill(A.data(), A.data() + nC*nV, 0.0);
        std::fill(lbA.begin(), lbA.begin() + nC, -qpOASES::INFTY);
        std::fill(ubA.begin(), ubA.begin() + nC, +qpOASES::INFTY);
        std::fill(lb.begin(), lb.begin() + nV, -qpOASES::INFTY);
        std::fill(ub.begin(), ub.begin() + nV, +qpOASES::INFTY);

        int constraint_idx = 0;
        
        for (uint32_t n = 0; n < mpc_N_; n++) for (uint32_t leg = 0; leg < 4; leg++)
        {   
            int idx = n * 12 + leg * 3;

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
  
        // MIT CONSTRAINTS
        qpOASES::QProblem qp_solver(nV, nC);
        qpOASES::int_t nWSR = mpc_max_iter_;
        
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = (debug_log ? qpOASES::PL_DEBUG_ITER : qpOASES::PL_NONE);
        options.enableRegularisation = qpOASES::BT_TRUE;  // DEBUG: Enable for stability
        options.enableFlippingBounds = qpOASES::BT_FALSE;
        qp_solver.setOptions(options);

        // KORREKTER INIT-AUFRUF mit allen 14 Parametern
        qpOASES::returnValue status = qp_solver.init(
            H.data(),             // Hessian matrix
            g.data(),             // Gradient vector
            A.data(),             // Constraint matrix
            lb.data(),            // Lower bounds (variables)
            ub.data(),            // Upper bounds (variables)
            lbA.data(),           // Lower bounds (constraints)
            ubA.data(),           // Upper bounds (constraints)
            nWSR,                 // Max iterations
            nullptr,              // CPU time
            nullptr,              // xOpt (initial primal solution)
            nullptr,              // yOpt (initial dual solution)
            nullptr,              // guessedBounds
            nullptr,              // guessedConstraints
            nullptr               // R (Cholesky factor)
        );
        
        if (status == qpOASES::SUCCESSFUL_RETURN || status == qpOASES::RET_MAX_NWSR_REACHED)
        {
            qp_solver.getPrimalSolution(x_opt.data());
            
            for (uint32_t i = 0; i < nV; i++) solution(i) = x_opt[i];
        }
        else // === 4. ERROR HANDLING AND FALLBACK ===
        {
            POND_LOG("ERROR QP failed! Status: %d - %s", status, qpOASES::MessageHandling::getErrorCodeMessage(status));
            
            // Additional debug for common errors
            if      (status == qpOASES::RET_INIT_FAILED)               POND_LOG("ERROR INIT_FAILED: Check problem data");
            else if (status == qpOASES::RET_INIT_FAILED_INFEASIBILITY) POND_LOG("ERROR INFEASIBLE: Constraints cannot be satisfied");
            else if (status == qpOASES::RET_INVALID_ARGUMENTS)         POND_LOG("ERROR INVALID_ARGUMENTS: Check matrix dimensions and values");
            
            // Fallback to Eigen solver (unconstrained)
            POND_LOG("Trying Eigen fallback solution...");
            try {
                Eigen::LLT<Eigen::MatrixXd> llt(H + 1e-4 * Eigen::MatrixXd::Identity(nV, nV));
                if (llt.info() == Eigen::Success)
                {
                    solution = llt.solve(-g);
                    double fallback_norm = solution.norm();
                    POND_LOG("Eigen fallback succeeded, solution norm: %.4f", fallback_norm);
                }
                else POND_LOG("ERROR Eigen LLT decomposition failed");
        
            } catch (const std::exception& e) {POND_LOG("ERROR Eigen fallback failed: %s", e.what());}
        }
        
        // Check if solution is all zeros
        if (double solution_norm = solution.norm(); solution_norm < 1e-10) POND_LOG("Solution is all zeros (norm=%.2e)", solution_norm);
        
        if (debug_log) POND_LOG("QP solve time: %.2f ms (%d iterations)", (pond::get_monotonic_time() - qp_start_time)*1000.0, nWSR);

        // === 5. DISTRIBUTE FORCES ===
        Eigen::Map<Eigen::Matrix<double, 3, 4>> forces_mat(solution.data());
        for (uint32_t i = 0; i < 4; i++) forces[i] = forces_mat.col(i);
        force_distributor.distribute(&forces);

        if (debug_log) POND_LOG("MPC iteration complete: %.2f ms", (pond::get_monotonic_time() - start_time)*1000.0);

    } catch (const std::exception &e) {POND_LOG("MPC error: %s", e.what());}
}