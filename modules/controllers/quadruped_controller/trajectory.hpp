#include "trajectory_function.hpp"

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

    bool init(double A_, double T_0_, double T_1_, double delta_y_, double j_, Eigen::Vector2d v_)
    {
        T_sw = 0.5; T_st = 0.5;
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

    void get_state(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc)
    {
        // Normalize time to a single gait cycle [0.0, 1.0]
        t = std::fmod(t, 1.0);

        // Select the appropriate gait phase based on the duty cycle
        if (t <= T_st)
        {
            pos = stance_function.eval_d<0>(t);
            vel = stance_function.eval_d<1>(t);
            acc = stance_function.eval_d<2>(t);
        }
        else if(t < T_st + T_sw/2.0)
        {
            pos = swing_I_function.eval_d<0>(t - T_st);
            vel = swing_I_function.eval_d<1>(t - T_st);
            acc = swing_I_function.eval_d<2>(t - T_st);
        }
        else
        {
            pos = swing_II_function.eval_d<0>(t - T_st);
            vel = swing_II_function.eval_d<1>(t - T_st);
            acc = swing_II_function.eval_d<2>(t - T_st);
        }

        // Transform local trajectory state into the global movement direction
        pos = v_rot_mat * pos;
        vel = v_rot_mat * vel;
        acc = v_rot_mat * acc;
    }

    trajectory_function_3d stance_function;
    trajectory_function_3d swing_I_function;
    trajectory_function_3d swing_II_function;
};