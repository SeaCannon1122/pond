#include <pond/pond.hpp>

class FootContactController : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
};

POND_MODULE_CPP_DECLARE(FootContactController, "foot_contact_controller", "detects foot touchdown and lift")

pond_result FootContactController::onStartup(const std::vector<void*>& args)
{
    return POND_SUCCESS;
}

void FootContactController::onShutdown()
{
}

#pragma once

#include <Eigen/Dense>
#include <cmath>

class MomentumObserver
{
public:
    static constexpr int NV = 18;
    static constexpr int NJ = 12;

    using Vector18d = Eigen::Matrix<double, NV, 1>;
    using Matrix18d = Eigen::Matrix<double, NV, NV>;
    using Vector12d = Eigen::Matrix<double, NJ, 1>;

    MomentumObserver(double dt, double cutoff_frequency)
    {
        dt_ = dt;
        lambda_ = 2.0 * M_PI * cutoff_frequency;

        gamma_ = std::exp(-lambda_ * dt_);

        beta_ = (1.0 - gamma_) /
                (gamma_ * dt_);

        r_.setZero();
        initialized_ = false;
    }

    Vector18d update(Matrix18d& M, Vector18d& v, Matrix18d& C, Vector18d& g, Vector12d& motor_torques)
    {
        Vector18d p = M * v;    // Generalized momentum

        // Map actuated motor torques into generalized coordinates
        Vector18d tau_generalized = mapMotorTorques(motor_torques);

        // C^T * v
        Vector18d coriolis_transpose_velocity = C.transpose() * v;

        // Observer input from the discrete-time derivation
        Vector18d x =
            beta_ * p
            + tau_generalized
            + coriolis_transpose_velocity
            - g;

        // Initialization
        if (!initialized_)
        {
            r_ = beta_ * p;
            initialized_ = true;
        }

        // Update internal observer state
        r_ += (1.0 - gamma_) * x;

        // Estimated external generalized disturbance
        Vector18d tau_disturbance =
            beta_ * p - r_;

        return tau_disturbance;
    }

    void reset()
    {
        r_.setZero();
        initialized_ = false;
    }

    const Vector18d& getInternalState() const
    {
        return r_;
    }

    double getGamma() const
    {
        return gamma_;
    }

    double getBeta() const
    {
        return beta_;
    }

private:

    Vector18d mapMotorTorques(
        const Vector12d& motor_torques)
    {
        Vector18d generalized_torques =
            Vector18d::Zero();

        // Floating base unactuated
        // 12 joint torques map to generalized indices 6-17
        generalized_torques.segment<NJ>(6) =
            motor_torques;

        return generalized_torques;
    }

    double dt_;
    double lambda_;
    double gamma_;
    double beta_;

    Vector18d r_;

    bool initialized_;
};