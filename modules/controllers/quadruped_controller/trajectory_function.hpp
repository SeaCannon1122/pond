#include <array>
#include <stdint.h>
#include <Eigen/Eigen>

template<uint32_t order>
double d_factor(double n) {return n * d_factor<order - 1>(n - 1.0);}

template<>
double d_factor<0>(double n) {return 1.0;}

template<uint32_t order>
double power(double base) {return base * d_factor<order - 1>(base);}

template<>
double power<0>(double base) {return 1.0;}

template<uint32_t d_order>
struct eq_point
{
    
    eq_point(double t, double res) : t_(t), res_(res) {}

    double t_, res_;
};

class trajectory_function
{
public:

    enum class type
    {
        POLYNOMIAL,
        SINE,
        COSINE,
    } type;

    /*
        polynomial: c_0 + c_1*t + c_2*t^2 + ...

        sine:       c_0 + c_1*sin(c_2 + c_3*t)

        cosine:     c_0 + c_1*cos(c_2 + c_3*t)
    */
    std::vector<double> coeffs;

    void make_polynomial(const std::vector<double> coeffs_)           { coeffs=coeffs_; type = type::POLYNOMIAL;}
    void make_sine(double c_0, double c_1, double c_2, double c_3)    { coeffs={c_0, c_1, c_2, c_3}; type = type::SINE;}
    void make_cosine(double c_0, double c_1, double c_2, double c_3)  { coeffs={c_0, c_1, c_2, c_3}; type = type::COSINE;}

    template<uint32_t order>
    double eval_d(double t)
    {
        switch (type)
        {
            case type::POLYNOMIAL:
            {
                double result = 0, power = 1.;

                for (uint32_t i = order; i < coeffs.size(); i++, power*=t) result += coeffs[i] * power * d_factor<order>(i);

                return result;
            }
            case type::SINE:
            case type::COSINE:
            {
                uint32_t count = order + (type == type::SINE ? 0 : 1);

                double inner = coeffs[2] + coeffs[3] * t;
                
                double trig = ((count%4) / 2 ? -1.f : 1.f) * (count%2 ? cos(inner) : sin(inner));

                return (order == 0 ? coeffs[0] : 0.0) + coeffs[1] * power<order>(coeffs[3]) * trig;
            }
        }

        return 0.0;
    }

    template<uint32_t c_count, uint32_t d_order>
    Eigen::RowVector<double, c_count> polynomial_row(double t)
    {
        Eigen::RowVector<double, c_count> row;
        for (uint32_t i = 0; i < d_order && i < c_count; i++) row[i] = 0.0;

        double power = 1.0;
        for (uint32_t i = d_order; i < c_count; i++, power*=t) row[i] = power * d_factor<d_order>(i);

        return row;
    }

    template<uint32_t ...d_orders>
    bool make_polynomial_through_points(eq_point<d_orders>... points)
    {
        Eigen::Matrix<double, sizeof...(d_orders), sizeof...(d_orders)> M;
        Eigen::Vector<double, sizeof...(d_orders)> b;

        uint32_t M_i = 0, b_i = 0;
        ((M.row(M_i++) = polynomial_row<sizeof...(d_orders), d_orders>(points.t_), b[b_i++] = points.res_), ...);

        Eigen::FullPivLU<Eigen::Matrix<double, sizeof...(d_orders), sizeof...(d_orders)>> lu(M);

        if (lu.rank() != sizeof...(d_orders)) return false;

        Eigen::Vector<double, sizeof...(d_orders)> x = lu.solve(b);
        coeffs.assign(x.data(), x.data() + x.size());
        type = type::POLYNOMIAL;

        return true;
    }
};

struct trajectory_function_3d
{
    template<uint32_t order>
    Eigen::Vector3d eval_d(double t) { return {f_x.eval_d<order>(t), f_y.eval_d<order>(t), f_z.eval_d<order>(t)};}

    trajectory_function f_x;
    trajectory_function f_y;
    trajectory_function f_z;
};