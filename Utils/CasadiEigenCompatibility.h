#ifndef GRBDA_CASADI_EIGEN_COMPATIBILITY_H
#define GRBDA_CASADI_EIGEN_COMPATIBILITY_H

#include <eigen3/Eigen/Dense>
#include <casadi/casadi.hpp>

// This code is based the workaround implemented by Pinocchio to make CasADi compatible with Eigen
// https://github.com/stack-of-tasks/pinocchio/blob/master/src/autodiff/casadi.hpp

namespace casadi
{
    inline bool operator||(const bool x, const casadi::Matrix<SXElem> & /*y*/)
    {
        return x;
    }

    inline bool operator&&(const bool x, const casadi::Matrix<SXElem> & /*y*/)
    {
        return x;
    }

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
                dst(i, j) = (typename MT::Scalar)src(i, j);
    }

    // Copy std::vector of Eigen matrix to std::vector casadi matrix
    template <typename MT>
    inline void copy(std::vector<MT> const &src,
                     std::vector<::casadi::SX> &dst)
    {
        static_assert(std::is_base_of<Eigen::MatrixBase<MT>, MT>::value,
                      "MT must be an Eigen matrix type");

        for (const auto &vec : src)
        {
            SX cs_vec = SX(Sparsity::dense(vec.rows(), vec.cols()));
            casadi::copy(vec, cs_vec);
            dst.push_back(cs_vec);
        }
    }
}

namespace Eigen
{
    /// @brief Eigen::NumTraits<> specialization for casadi::SX
    ///
    template <typename Scalar>
    struct NumTraits<casadi::Matrix<Scalar>>
    {
        using Real = casadi::Matrix<Scalar>;
        using NonInteger = casadi::Matrix<Scalar>;
        using Literal = casadi::Matrix<Scalar>;
        using Nested = casadi::Matrix<Scalar>;

        enum
        {
            // does not support complex Base types
            IsComplex = 0,
            // does not support integer Base types
            IsInteger = 0,
            // only support signed Base types
            IsSigned = 1,
            // must initialize an AD<Base> object
            RequireInitialization = 1,
            // computational cost of the corresponding operations
            ReadCost = 1,
            AddCost = 2,
            MulCost = 2
        };

        static casadi::Matrix<Scalar> epsilon()
        {
            return casadi::Matrix<Scalar>(std::numeric_limits<double>::epsilon());
        }

        static casadi::Matrix<Scalar> dummy_precision()
        {
            return casadi::Matrix<Scalar>(NumTraits<double>::dummy_precision());
        }

        static casadi::Matrix<Scalar> highest()
        {
            return casadi::Matrix<Scalar>(std::numeric_limits<double>::max());
        }

        static casadi::Matrix<Scalar> lowest()
        {
            return casadi::Matrix<Scalar>(std::numeric_limits<double>::min());
        }

        static int digits10()
        {
            return std::numeric_limits<double>::digits10;
        }
    };
} // namespace Eigen

#endif // GRBDA_CASADI_EIGEN_COMPATIBILITY_H
