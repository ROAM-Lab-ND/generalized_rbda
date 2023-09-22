#include "gtest/gtest.h"

#include "Utils/SpatialInertia.h"
#include "Utils/SpatialTransforms.h"

#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <casadi/casadi.hpp>

using namespace grbda;

// TODO(@MatthewChignoli): Code is based off of http://docs.ros.org/en/melodic/api/pinocchio/html/casadi-basic_8cpp_source.html

namespace casadi
{
    // Copy Eigen matrix to casadi matrix
    template <typename MT, typename Scalar>
    inline void copy(Eigen::MatrixBase<MT> const &src,
                     ::casadi::Matrix<Scalar> &dst)
    {
        Eigen::DenseIndex const m = src.rows();
        Eigen::DenseIndex const n = src.cols();

        dst.resize(m, n);

        for (Eigen::DenseIndex i = 0; i < m; ++i)
            for (Eigen::DenseIndex j = 0; j < n; ++j)
                dst(i, j) = src(i, j);
    }

    // Copy casadi matrix to Eigen matrix
    template <typename MT, typename Scalar>
    inline void copy(::casadi::Matrix<Scalar> const &src,
                     Eigen::MatrixBase<MT> &dst)
    {
        Eigen::DenseIndex const m = src.size1();
        Eigen::DenseIndex const n = src.size2();

        dst.resize(m, n);

        for (Eigen::DenseIndex i = 0; i < m; ++i)
            for (Eigen::DenseIndex j = 0; j < n; ++j)
                dst(i, j) = src(i, j);
    }
}

GTEST_TEST(Derivatives, SpatialMotionTransform)
{
    // Compute

    // TODO(@MatthewChignoli): template the test on SX and MX?
    using SX = casadi::SX;

    // So I think that if we have a revolute Z joint, then the partial with repsect to q of Xup should be [0;0;1;0;0;0]

    SX cs_q = SX::sym("q");
    spatial::Transform<SX> XJ = spatial::jointXform(spatial::JointType::Revolute,
                                                    ori::CoordinateAxis::Z, cs_q);

    SVec<SX> v1;
    for (int i = 0; i < 6; ++i)
    {
        v1(i) = 1.;
    }

    SVec<SX> v2 = XJ.transformMotionVector(v1);

    // // Copy the result from Eigen matrices to casadi matrix
    casadi::SX cs_v2 = casadi::SX(casadi::Sparsity::dense(v2.rows(), 1));
    casadi::copy(v2, cs_v2);

    // Display the resulting casadi matrix
    std::cout << "v2 = " << cs_v2 << std::endl;

    // Do some AD
    casadi::SX dv2_dq = jacobian(cs_v2, cs_q);

    // Display the resulting jacobian
    std::cout << "dv2/dq = " << dv2_dq << std::endl;

    // Create a function
    casadi::Function fun("fun", casadi::SXVector{cs_q}, casadi::SXVector{cs_v2, dv2_dq});
    std::cout << "fun = " << fun << std::endl;

    // Evaluate the function
    casadi::DMVector res = fun(casadi::DMVector{std::vector<double>{1.}});
    std::cout << "fun(q)=" << res << std::endl;

    // Compare to finite difference
    double eps = 1e-6;
    casadi::DMVector res_plus = fun(casadi::DMVector{std::vector<double>{1. + eps}});
    // std::cout << "fun(q + eps)=" << res_plus << std::endl;

    casadi::DMVector res_minus = fun(casadi::DMVector{std::vector<double>{1. - eps}});
    // std::cout << "fun(q - eps)=" << res_minus << std::endl;

    casadi::DM res_fd = (res_plus[0] - res_minus[0]) / (2. * eps);

    std::cout << "fun(q + eps) - fun(q - eps) / (2 * eps) = " << res_fd << std::endl;


    
}
