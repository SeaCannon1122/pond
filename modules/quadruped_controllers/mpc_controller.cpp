#include "pond/hpp/module_base.hpp"
#include <array>
#include <atomic>
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/command_types.hpp>

#include <qpOASES.hpp>

using Matrix13 = Eigen::Matrix<double, 13, 13>;
using Matrix13x12 = Eigen::Matrix<double, 13, 12>;
using Matrix12 = Eigen::Matrix<double, 12, 12>;
using Vector13 = Eigen::Matrix<double, 13, 1>;

struct HGSolution { Eigen::MatrixXd H; Eigen::VectorXd g;};

class MPCController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
    virtual void onFrame() override;

    Matrix13x12 computeBhat(double psi, const std::array<Eigen::Vector3d, 4> &r_i);
    Eigen::MatrixXd computeAhat();
    
    Eigen::MatrixXd computeH(const std::vector<Matrix13x12> &B_hat_sequence);
    Eigen::VectorXd computeg(const Eigen::VectorXd &x0, const std::vector<Matrix13x12> &B_hat_sequence, const std::vector<Eigen::Vector<double, 13>> &x_ref_sequence);
    Eigen::VectorXd solveQPWithConstraints(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const std::array<bool, 4>& foot_contact);

    pond::Callback twist_callback, orientation_callback, update_callback;
    pond::DistributorTyped<std::vector<Eigen::Vector3d>> force_distributor;
    std::vector<Eigen::Vector3d> forces;
    std::atomic<uint32_t> phases_buf;

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

    qpOASES::QProblem qp_solver_{0, 0};
    bool qp_initialized_ = false;
    int qp_nv_ = 0;
    int qp_nc_ = 0;
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
    force_distributor = createDistributorTyped<std::vector<Eigen::Vector3d>>({"ground_forces"});

    update_callback = createCallback<std::vector<bool>*>({"update_mpc/gait_phases"}, [this](std::vector<bool>** phases_in){
        if (!*phases_in) POND_LOG_RETURN("update_mpc/gait_phases is NULL");

        uint32_t phase_int = 0;
        if (phases_in[0][0][0]) phase_int |= 0b0001;
        if (phases_in[0][0][1]) phase_int |= 0b0010;
        if (phases_in[0][0][2]) phase_int |= 0b0100;
        if (phases_in[0][0][3]) phase_int |= 0b1000;

        phases_buf.store(phase_int);
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
    update_callback.destroy();
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

    uint32_t phases_int = phases_buf.load();
    std::array<bool, 4> foot_contacts = {
        phases_int & 0b0001 ? true : false,
        phases_int & 0b0100 ? true : false,
        phases_int & 0b0010 ? true : false,
        phases_int & 0b1000 ? true : false,
    };

    try {
        double start_time = pond::get_monotonic_time();

        std::vector<Eigen::Vector<double, 13>> x_ref_sequence(mpc_N_, Eigen::Vector<double, 13>::Zero());
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

        // === 3. CURRENT STATE ===

        Eigen::VectorXd x_current = Eigen::VectorXd::Zero(13);
        x_current(2) = 0; // yaw, set to zero for now, may be changed when there is a good odometry system
        x_current(5) = 0.4;
        x_current(9) = twist_cmd.lin[0];
        x_current(10) = twist_cmd.lin[1];
        x_current(12) = G_ACCEL;
          
        // === 4. BUILD AND SOLVE QP ===

        Eigen::MatrixXd H = computeH(B_hat_sequence);
        Eigen::VectorXd g = computeg(x_current, B_hat_sequence, x_ref_sequence);

        // Quick sanity check
        if (H.rows() == 0 || H.cols() == 0) POND_LOG_RETURN("H is empty!");

        if (g.size() == 0) POND_LOG_RETURN("g is empty!");

        Eigen::VectorXd U_opt = solveQPWithConstraints(H, g, foot_contacts);

        // === 5. PUBLISH FORCES ===
        if (U_opt.size() >= 12)
        {
            Eigen::Map<Eigen::Matrix<double, 3, 4>> forces_mat(U_opt.data());
            for (uint32_t i = 0; i < 4; i++) forces[i] = forces_mat.col(i);
            force_distributor.distribute(&forces);
        }
        else POND_LOG("Solution too small: %td < 12", U_opt.size());

        iteration_count++;

        POND_LOG("MPC iteration complete: %.2f ms", (pond::get_monotonic_time() - start_time)/1000.0);

    } catch (const std::exception &e) {POND_LOG("MPC error: %s", e.what());}
}

Matrix13x12 MPCController::computeBhat(double psi, const std::array<Eigen::Vector3d, 4>& r_i) {
    // CACHED VERSION - 10x faster with same psi
    
    // 1. CACHE for rotation matrix and LDLT
    static double cached_psi = -1000.0;
    static Eigen::Matrix3d cached_R_z;
    static Eigen::LDLT<Eigen::Matrix3d> cached_I_hat_ldlt;
    
    if (std::abs(psi - cached_psi) > 1e-6) {
        // Only recompute if psi changes significantly
        cached_psi = psi;
        
        cached_R_z << cos(psi), -sin(psi), 0,
                    sin(psi),  cos(psi), 0,
                    0,         0,        1;
        
        Eigen::Matrix3d I_hat = cached_R_z * robot_I_body_ * cached_R_z.transpose();
        cached_I_hat_ldlt.compute(I_hat);
    }
    
    // 2. Build B_c matrix with pre-allocated blocks
    static Matrix13x12 B_c;  // Static for reuse
    B_c.setZero();
    
    // UNROLLED LOOP for all 4 legs
    for (uint32_t i = 0; i < 4; i++)
    {
            const Eigen::Vector3d& r = r_i[i];
            Eigen::Matrix3d r_cross;
            r_cross << 0, -r[2],  r[1],
                    r[2],     0, -r[0],
                -r[1],  r[0],     0;
            
            B_c.block<3,3>(6, 3*i) = cached_I_hat_ldlt.solve(r_cross);
            B_c.block<3,3>(9, 3*i).setIdentity();
            B_c.block<3,3>(9, 3*i) /= robot_m_;
    }
    
    return mpc_dt_ * B_c;
}

Eigen::MatrixXd MPCController::computeAhat() {
  // Cached Â (depends only on mpc_dt_)
  if (A_hat_valid_) {
    return A_hat_cached_;
  }

  Eigen::MatrixXd A_c = Eigen::MatrixXd::Zero(13, 13);
  // Block for Ṗ = v
  A_c.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();

  A_hat_cached_ = Eigen::MatrixXd::Identity(13, 13) + mpc_dt_ * A_c;
  A_hat_valid_ = true;
  return A_hat_cached_;
}

Eigen::MatrixXd MPCController::computeH(const std::vector<Matrix13x12>& B_hat_sequence)
{
    // OPTIMIZED: Uses symmetry and block-wise updates
    const int state_dim = 13;
    const int input_dim = 12;
    const int H_dim = input_dim * mpc_N_;
    
    // 1. STATIC PRE-ALLOCATION (most important performance gain)
    static Eigen::MatrixXd H_buffer;
    static Eigen::MatrixXd B_qp_buffer;
    
    if (H_buffer.rows() != H_dim || H_buffer.cols() != H_dim)
    {
        H_buffer.resize(H_dim, H_dim);
        B_qp_buffer.resize(state_dim * mpc_N_, H_dim);
    }
    
    H_buffer.setZero();
    B_qp_buffer.setZero();
    
    // 2. A^k PRE-COMPUTE with optimization for small N
    static Eigen::MatrixXd A_hat;
    static bool A_hat_computed = false;
    
    if (!A_hat_computed)
    {
        Eigen::MatrixXd A_c = Eigen::MatrixXd::Zero(13, 13);
        A_c.block<3,3>(3, 9) = Eigen::Matrix3d::Identity();
        A_hat = Eigen::MatrixXd::Identity(13, 13) + mpc_dt_ * A_c;
        A_hat_computed = true;
    }
    
    // Static A_powers buffer
    static std::vector<Eigen::MatrixXd> A_powers(mpc_N_);
    
    // Only recompute if necessary
    static int last_mpc_N = 0;
    if (last_mpc_N != mpc_N_)
    {
        last_mpc_N = mpc_N_;
        A_powers.resize(mpc_N_);
        A_powers[0] = Eigen::MatrixXd::Identity(13, 13);
        for (int i = 1; i < mpc_N_; ++i) A_powers[i] = A_powers[i-1] * A_hat;
    }
    
    // 3. OPTIMIZED B_qp COMPUTATION
    // Exploit symmetry: only compute lower triangular matrix
    for (int i = 0; i < mpc_N_; ++i)
    {
        // Block for j = i (Diagonal)
        B_qp_buffer.block(state_dim * i, input_dim * i, state_dim, input_dim) = A_powers[i] * B_hat_sequence[i];
        
        // Only compute lower triangular matrix (j < i)
        for (int j = 0; j < i; ++j) B_qp_buffer.block(state_dim * i, input_dim * j, state_dim, input_dim) = A_powers[i] * B_hat_sequence[j];
    }
    
    // 4. EFFICIENTLY USE DIAGONAL Q AND R
    static const Eigen::VectorXd q_diag = mpc_Q_.diagonal();
    static const Eigen::VectorXd r_diag = mpc_R_.diagonal();
    
    // 5. BUILD H with symmetric updates
    // R-contribution first (block diagonal)
    for (int blk = 0; blk < mpc_N_; ++blk) H_buffer.block(input_dim * blk, input_dim * blk, input_dim, input_dim).diagonal() = 2.0 * r_diag;
    
    // Q-contribution: Use symmetry - only fill lower triangular matrix
    for (int i = 0; i < mpc_N_; ++i) for (int s = 0; s < state_dim; ++s)
    {
        double q = q_diag[s];
        if (q == 0.0) continue;
        
        Eigen::RowVectorXd v_row = B_qp_buffer.row(state_dim * i + s);
        
        // Symmetric update: H += 2q * (v^T * v)
        // Only fill lower triangular matrix
        for (int col = 0; col < H_dim; ++col) if (v_row[col] != 0.0)
            for (int row = col; row < H_dim; ++row) H_buffer(row, col) += 2.0 * q * v_row[col] * v_row[row];
    }
  
  // Mirror upper triangular matrix
  H_buffer.triangularView<Eigen::StrictlyUpper>() = H_buffer.triangularView<Eigen::StrictlyLower>().transpose();
  
  return H_buffer;
}

Eigen::VectorXd MPCController::computeg(const Eigen::VectorXd& x0, const std::vector<Matrix13x12>& B_hat_sequence, const std::vector<Eigen::Vector<double, 13>>& x_ref_sequence)
{  
  const int state_dim = 13;
  const int input_dim = 12;
  const int H_dim = input_dim * mpc_N_;
  
  // 1. STATIC BUFFER REUSE
  static Eigen::VectorXd g_buffer;
  static Eigen::MatrixXd B_qp_buffer;
  
  if (g_buffer.size() != H_dim)
  {
    g_buffer.resize(H_dim);
    B_qp_buffer.resize(state_dim * mpc_N_, H_dim);
  }
  
  g_buffer.setZero();
  
  // 2. B_qp IS SHARED WITH computeH() - no double computation!
  // Here we need to either reuse B_qp_buffer from computeH()
  // or recompute. Better: combine computeH() and computeg().
  
  // Optimization: Combine H and g computation
  static bool buffers_computed = false;
  static std::vector<Eigen::MatrixXd> A_powers(mpc_N_);
  
  if (!buffers_computed)
  {
    // Compute A^k (once only)
    Eigen::MatrixXd A_hat = computeAhat();
    A_powers.resize(mpc_N_);
    A_powers[0] = Eigen::MatrixXd::Identity(13, 13);
    for (int i = 1; i < mpc_N_; ++i) A_powers[i] = A_powers[i-1] * A_hat;
    
    // B_qp berechnen
    for (int i = 0; i < mpc_N_; ++i) for (int j = 0; j <= i; ++j)
      B_qp_buffer.block(state_dim * i, input_dim * j, state_dim, input_dim) = A_powers[i] * B_hat_sequence[j];
    
    buffers_computed = true;
  }
  
  // 3. ONLY COMPUTE RELEVANT STATES (not all 13)
  // Your Q-Matrix has only few non-zero entries:
  // Indizes: 2,3,4,5,8,9,10,12 sind ≠ 0 (8 von 13)
  static const std::array<int, 8> relevant_states = {2, 3, 4, 5, 8, 9, 10, 12};
  static const Eigen::VectorXd q_diag = mpc_Q_.diagonal();
  
  // 4. OPTIMIZED ERROR COMPUTATION
    for (int i = 0; i < mpc_N_; ++i)
    {
        Eigen::VectorXd x_pred = A_powers[i] * x0;
        const Eigen::VectorXd& x_ref = x_ref_sequence[i];
        
        // Only process relevant states
        for (int s_idx = 0; s_idx < 8; ++s_idx)
        {
            int s = relevant_states[s_idx];
            double q = q_diag[s];
            if (q == 0.0) continue;
            
            double err = x_pred(s) - x_ref(s);
            if (std::abs(err) < 1e-10) continue;  // Skip small errors
            
            Eigen::RowVectorXd v = B_qp_buffer.row(state_dim * i + s);
            g_buffer.noalias() += 2.0 * q * err * v;
        }
    }
  
  return g_buffer;
}

Eigen::VectorXd MPCController::solveQPWithConstraints(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const std::array<bool, 4>& foot_contact)
{
    // Check dimensions
    if (H.rows() != 12 * mpc_N_ || H.cols() != 12 * mpc_N_)
    {
        POND_LOG("ERROR H has wrong dimensions! Expected %dx%d, got %ldx%ld", 12*mpc_N_, 12*mpc_N_, H.rows(), H.cols());
        return Eigen::VectorXd::Zero(12 * mpc_N_);
    }
    
    if (g.size() != 12 * mpc_N_)
    {
        POND_LOG("ERROR g has wrong size! Expected %d, got %td", 12*mpc_N_, g.size());
        return Eigen::VectorXd::Zero(12 * mpc_N_);
    }
    
    int nV = H.rows();
    
    // Quick check for NaN in H and g
    if (H.hasNaN()) {POND_LOG("ERROR H contains NaN!"); return Eigen::VectorXd::Zero(nV);}
    if (g.hasNaN()) {POND_LOG("ERROR g contains NaN!"); return Eigen::VectorXd::Zero(nV);}
    
    // === 1. CONSTRAINT MATRIX A ===
    // Count constraints for active feet only
    int nC = 0;
    int feet_on_ground = 0;
    
    for (int n = 0; n < mpc_N_; ++n) for (int leg = 0; leg < 4; ++leg) if (foot_contact[leg])
    {
        nC += 6;  // 6 constraints per foot
        feet_on_ground++;
    }
    
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nC, nV);
    std::vector<qpOASES::real_t> lbA(nC), ubA(nC);
    
    int constraint_idx = 0;
    
    for (int n = 0; n < mpc_N_; ++n) for (int leg = 0; leg < 4; ++leg)
    {
        if (!foot_contact[leg]) continue;
        
        int var_idx = n * 12 + leg * 3;
        
        // 1) f_z lower bound: f_z ≥ f_min
        A(constraint_idx, var_idx + 2) = 1.0;
        lbA[constraint_idx] = robot_f_min_;
        ubA[constraint_idx] = 1e20;
        constraint_idx++;
        
        // 2) f_z upper bound: f_z ≤ f_max  
        A(constraint_idx, var_idx + 2) = 1.0;
        lbA[constraint_idx] = -1e20;
        ubA[constraint_idx] = robot_f_max_;
        constraint_idx++;
        
        // 3) f_x ≤ μ·f_z
        A(constraint_idx, var_idx) = 1.0;
        A(constraint_idx, var_idx + 2) = -robot_mu_;
        lbA[constraint_idx] = -1e20; ubA[constraint_idx] = 0.0;
        constraint_idx++;
        
        // 4) -f_x ≤ μ·f_z
        A(constraint_idx, var_idx) = -1.0;
        A(constraint_idx, var_idx + 2) = -robot_mu_;
        lbA[constraint_idx] = -1e20; ubA[constraint_idx] = 0.0;
        constraint_idx++;
        
        // 5) f_y ≤ μ·f_z
        A(constraint_idx, var_idx + 1) = 1.0;
        A(constraint_idx, var_idx + 2) = -robot_mu_;
        lbA[constraint_idx] = -1e20; ubA[constraint_idx] = 0.0;
        constraint_idx++;
        
        // 6) -f_y ≤ μ·f_z
        A(constraint_idx, var_idx + 1) = -1.0;
        A(constraint_idx, var_idx + 2) = -robot_mu_;
        lbA[constraint_idx] = -1e20; ubA[constraint_idx] = 0.0;
        constraint_idx++;
    }
    
    // === 2. SETUP QP DATA ===

    std::vector<qpOASES::real_t> H_qp(nV * nV);
    Eigen::Map<Eigen::MatrixXd>(H_qp.data(), nV, nV) = H;
    for (int i = 0; i < nV; ++i) H_qp[i * nV + i] += 1e-6;  // Regularization
    
    std::vector<qpOASES::real_t> g_qp(nV);
    Eigen::Map<Eigen::VectorXd>(g_qp.data(), nV) = g;
    
    std::vector<qpOASES::real_t> A_qp(nC * nV);
    if (nC > 0) Eigen::Map<Eigen::MatrixXd>(A_qp.data(), nC, nV) = A;
    
    std::vector<qpOASES::real_t> lb(nV, -1e20), ub(nV, +1e20);
    
    // Set force for airborne feet to 0
    int airborne_feet_count = 0;
    for (int n = 0; n < mpc_N_; ++n) for (int leg = 0; leg < 4; ++leg)
    {
        if (foot_contact[leg]) continue;

        airborne_feet_count++;
        int idx = n * 12 + leg * 3;
        lb[idx] = ub[idx] = 0.0;      // f_x = 0
        lb[idx+1] = ub[idx+1] = 0.0;  // f_y = 0
        lb[idx+2] = ub[idx+2] = 0.0;  // f_z = 0
    }
    
    // === 3. SOLVE QP ===

    Eigen::VectorXd solution = Eigen::VectorXd::Zero(nV);
    
    qpOASES::int_t nWSR = 100;
    qpOASES::returnValue status;
    
    double start_time = pond::get_monotonic_time();
    
    if (nC > 0)
    {
        // MIT CONSTRAINTS
        qpOASES::QProblem qp_solver(nV, nC);
        
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_HIGH;  // DEBUG: Use HIGH for debugging
        options.enableRegularisation = qpOASES::BT_TRUE;  // DEBUG: Enable for stability
        options.enableFlippingBounds = qpOASES::BT_FALSE;
        qp_solver.setOptions(options);
        
        // KORREKTER INIT-AUFRUF mit allen 14 Parametern
        status = qp_solver.init(
            H_qp.data(),          // Hessian matrix
            g_qp.data(),          // Gradient vector
            A_qp.data(),          // Constraint matrix
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
            std::vector<qpOASES::real_t> x_opt(nV);
            qp_solver.getPrimalSolution(x_opt.data());
            
            for (int i = 0; i < nV; ++i) solution(i) = x_opt[i];
            
            qpOASES::real_t obj_val = qp_solver.getObjVal();
        }
        else POND_LOG("ERROR QP with constraints failed: %d", status);
    }
    else
    {
        // OHNE CONSTRAINTS (nur bounds)

        qpOASES::QProblemB qp_b_solver(nV);
        
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_HIGH;
        options.enableRegularisation = qpOASES::BT_TRUE;
        qp_b_solver.setOptions(options);
        
        status = qp_b_solver.init(
            H_qp.data(),          // Hessian matrix
            g_qp.data(),          // Gradient vector
            lb.data(),            // Lower bounds
            ub.data(),            // Upper bounds
            nWSR,                 // Max iterations
            nullptr               // CPU time
        );
        
        if (status == qpOASES::SUCCESSFUL_RETURN || status == qpOASES::RET_MAX_NWSR_REACHED)
        {
            std::vector<qpOASES::real_t> x_opt(nV);
            qp_b_solver.getPrimalSolution(x_opt.data());
            
            for (int i = 0; i < nV; ++i) solution(i) = x_opt[i];
            
            qpOASES::real_t obj_val = qp_b_solver.getObjVal();
        }
        else POND_LOG("ERROR QP without constraints failed: %d", status);
    }
    
    double end_time = pond::get_monotonic_time();
    
    // === 4. ERROR HANDLING AND FALLBACK ===
    if (status != qpOASES::SUCCESSFUL_RETURN && status != qpOASES::RET_MAX_NWSR_REACHED)
    {
        const char* error_msg = qpOASES::MessageHandling::getErrorCodeMessage(status);
        POND_LOG("ERROR QP failed! Status: %d - %s", status, error_msg);
        
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
                POND_LOG("Eigen fallback succeeded");
                double fallback_norm = solution.norm();
                POND_LOG("Fallback solution norm: %.4f", fallback_norm);
            }
            else POND_LOG("ERROR Eigen LLT decomposition failed");
    
        } catch (const std::exception& e) {POND_LOG("ERROR Eigen fallback failed: %s", e.what());}
    }
    
    // Validate solution size
    if (solution.size() != 12 * mpc_N_)
    {
        POND_LOG("ERROR Solution has wrong size: %td != %d", solution.size(), 12 * mpc_N_);
        solution.setZero();
    }
    
    // Check if solution is all zeros
    double solution_norm = solution.norm();
    if (solution_norm < 1e-10) POND_LOG("Solution is all zeros (norm=%.2e)", solution_norm);
    
    POND_LOG("QP solve time: %.2f ms", (end_time - start_time)/1000.0);
    
    return solution;
}