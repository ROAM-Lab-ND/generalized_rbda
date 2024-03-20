/*! @file cTypes.h
 *  @brief Common types that are only valid in C++
 *
 *  This file contains types which are only used in C++ code.  This includes
 * Eigen types, template types, aliases, ...
 */

#ifndef GRBDA_CPPTYPES_H
#define GRBDA_CPPTYPES_H

#include <vector>
#include "CasadiEigenCompatibility.h"

namespace grbda
{

    // Rotation Matrix
    template <typename T>
    using RotMat = typename Eigen::Matrix<T, 3, 3>;

    // 1x1 Vector
    template <typename T>
    using Vec1 = typename Eigen::Matrix<T, 1, 1>;

    // 2x1 Vector
    template <typename T>
    using Vec2 = typename Eigen::Matrix<T, 2, 1>;

    // 3x1 Vector
    template <typename T>
    using Vec3 = typename Eigen::Matrix<T, 3, 1>;

    // 7x1 Vector
    template <typename T>
    using Vec7 = typename Eigen::Matrix<T, 7, 1>;

    // 10x1 Vector
    template <typename T>
    using Vec10 = typename Eigen::Matrix<T, 10, 1>;

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
