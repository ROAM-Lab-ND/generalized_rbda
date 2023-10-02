/*! @file cTypes.h
 *  @brief Common types that are only valid in C++
 *
 *  This file contains types which are only used in C++ code.  This includes
 * Eigen types, template types, aliases, ...
 */

#ifndef GRBDA_CPPTYPES_H
#define GRBDA_CPPTYPES_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <casadi/casadi.hpp>

// TODO(@MatthewChignoli): Eigen and Casadi compatibility is mostly taken from Pinnocchio. Need to make sure to give them credit

// TODO(@MatthewChignoli): Should probably move this to somewhere else
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

namespace grbda
{

    // Rotation Matrix
    template <typename T>
    using RotMat = typename Eigen::Matrix<T, 3, 3>;

    // 2x1 Vector
    template <typename T>
    using Vec2 = typename Eigen::Matrix<T, 2, 1>;

    // 3x1 Vector
    template <typename T>
    using Vec3 = typename Eigen::Matrix<T, 3, 1>;

    // 7x1 Vector
    template <typename T>
    using Vec7 = typename Eigen::Matrix<T, 7, 1>;

    // 2x2 Matrix
    template <typename T>
    using Mat2 = typename Eigen::Matrix<T, 2, 2>;

    // 3x3 Matrix
    template <typename T>
    using Mat3 = typename Eigen::Matrix<T, 3, 3>;

    // 4x4 Matrix
    template <typename T>
    using Mat4 = typename Eigen::Matrix<T, 4, 4>;

    // 4x1 Vector
    template <typename T>
    using Quat = typename Eigen::Matrix<T, 4, 1>;

    // Spatial Vector (6x1, all subspaces)
    template <typename T>
    using SVec = typename Eigen::Matrix<T, 6, 1>;

    // 6x6 Matrix
    template <typename T>
    using Mat6 = typename Eigen::Matrix<T, 6, 6>;

    // 10x1 Vector
    template <typename T>
    using MassProperties = typename Eigen::Matrix<T, 10, 1>;

    // Dynamically sized vector
    template <typename T>
    using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

    // Dynamically sized matrix
    template <typename T>
    using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

    // Dynamically sized matrix with spatial vector columns
    template <typename T>
    using D6Mat = typename Eigen::Matrix<T, 6, Eigen::Dynamic>;

    // Dynamically sized matrix with cartesian vector columns
    template <typename T>
    using D3Mat = typename Eigen::Matrix<T, 3, Eigen::Dynamic>;

} // namespace grbda

#endif // PROJECT_CPPTYPES_H
