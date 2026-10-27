#include "trajectory_function.hpp"

#include <pond_data_types/trajectory_types.hpp>

class DynamicTrajectory
{    
public: 
    double T_sw, T_st, T_0, T_1;
    double delta_y;

    double v;
    Eigen::Matrix3d v_rot_mat;

    double step_length;
    double j;
    double stance_amplitude;

    bool init(double T_st_, double T_sw_, double A_, double T_0_, double T_1_, double delta_y_, double j_, Eigen::Vector2d v_)
    {
        T_sw = T_sw_; T_st = T_st_;
        T_0 = T_0_; T_1 = T_1_;
        delta_y = delta_y_;

        v = -v_.norm();
        v_rot_mat = Eigen::AngleAxisd(atan2(v_.y(), v_.x()), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        
        step_length = v * T_st;
        stance_amplitude = A_;
        j = j_;

        // Stance

        stance_function.f_x.make_polynomial({-step_length / 2.0, v});
        stance_function.f_y.make_polynomial({0});
        stance_function.f_z.make_cosine(stance_amplitude, -stance_amplitude, M_PI/2.0, -v*M_PI/step_length);

        // Swing I

        // x
        if (!swing_I_function.f_x.make_polynomial_through_points(

            eq_point<0>(0.0,        stance_function.f_x.eval_d<0>(T_st)),
            eq_point<1>(0.0,        stance_function.f_x.eval_d<1>(T_st)),
            eq_point<2>(0.0,        stance_function.f_x.eval_d<2>(T_st)),
            eq_point<3>(0.0,        stance_function.f_x.eval_d<3>(T_st)),

            eq_point<0>(T_sw / 2.0, 0.0),
            eq_point<2>(T_sw / 2.0, 0.0),
            eq_point<1>(T_0,        0.0)

        )) return false;

        // y
        swing_I_function.f_y.make_polynomial({0});

        // z
        if (!swing_I_function.f_z.make_polynomial_through_points(

            eq_point<0>(0.0,        stance_function.f_z.eval_d<0>(T_st)),
            eq_point<1>(0.0,        stance_function.f_z.eval_d<1>(T_st)),
            eq_point<2>(0.0,        stance_function.f_z.eval_d<2>(T_st)),
            eq_point<3>(0.0,        stance_function.f_z.eval_d<3>(T_st)),

            eq_point<0>(T_sw / 2.0, delta_y + stance_amplitude),
            eq_point<1>(T_sw / 2.0, 0.0),
            eq_point<2>(T_sw / 2.0, 0.0),
            eq_point<3>(T_sw / 2.0, j)

        )) return false;

        // Swing II
        
        // x
        if (!swing_II_function.f_x.make_polynomial_through_points(

            eq_point<0>(T_sw,        stance_function.f_x.eval_d<0>(0.0)),
            eq_point<1>(T_sw,        stance_function.f_x.eval_d<1>(0.0)),
            eq_point<2>(T_sw,        stance_function.f_x.eval_d<2>(0.0)),
            eq_point<3>(T_sw,        stance_function.f_x.eval_d<3>(0.0)),

            eq_point<0>(T_sw / 2.0, 0.0),
            eq_point<2>(T_sw / 2.0, 0.0),
            eq_point<1>(T_1,        0.0)

        )) return false;

        // y
        swing_II_function.f_y.make_polynomial({0});

        // z
        if (!swing_II_function.f_z.make_polynomial_through_points(

            eq_point<0>(T_sw,        stance_function.f_z.eval_d<0>(0.0)),
            eq_point<1>(T_sw,        stance_function.f_z.eval_d<1>(0.0)),
            eq_point<2>(T_sw,        stance_function.f_z.eval_d<2>(0.0)),
            eq_point<3>(T_sw,        stance_function.f_z.eval_d<3>(0.0)),

            eq_point<0>(T_sw / 2.0, delta_y + stance_amplitude),
            eq_point<1>(T_sw / 2.0, 0.0),
            eq_point<2>(T_sw / 2.0, 0.0),
            eq_point<3>(T_sw / 2.0, j)

        )) return false;

        return true;
    }

    void get_state(double t, TrajectoryState& point)
    {
        // Normalize time to a single gait cycle [0.0, 1.0]
        t = std::fmod(t, T_st + T_sw);

        // Select the appropriate gait phase based on the duty cycle
        if (t <= T_st)
        {
            point.pos = stance_function.eval_d<0>(t);
            point.vel = stance_function.eval_d<1>(t);
            point.acc = stance_function.eval_d<2>(t);
        }
        else if(t < T_st + T_sw/2.0)
        {
            point.pos = swing_I_function.eval_d<0>(t - T_st);
            point.vel = swing_I_function.eval_d<1>(t - T_st);
            point.acc = swing_I_function.eval_d<2>(t - T_st);
        }
        else
        {
            point.pos = swing_II_function.eval_d<0>(t - T_st);
            point.vel = swing_II_function.eval_d<1>(t - T_st);
            point.acc = swing_II_function.eval_d<2>(t - T_st);
        }

        // Transform local trajectory state into the global movement direction
        point.pos = v_rot_mat * point.pos;
        point.vel = v_rot_mat * point.vel;
        point.acc = v_rot_mat * point.acc;
    }

    trajectory_function_3d stance_function;
    trajectory_function_3d swing_I_function;
    trajectory_function_3d swing_II_function;
};