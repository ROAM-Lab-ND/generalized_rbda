#ifndef GRBDA_CASADI_DERIVATIVES_H
#define GRBDA_CASADI_DERIVATIVES_H

#include <casadi/casadi.hpp>
#include <stdexcept>
#include "grbda/Utils/cppTypes.h"
#include "grbda/Utils/SpatialTransforms.h"

namespace grbda
{
namespace casadi_derivatives
{

/// @brief Cross-product matrix (skew-symmetric) for CasADi SX
inline casadi::SX crossMatrix(const casadi::SX& v)
{
    casadi::SX crm = casadi::SX::zeros(3, 3);
    crm(0, 1) = -v(2);
    crm(0, 2) =  v(1);
    crm(1, 0) =  v(2);
    crm(1, 2) = -v(0);
    crm(2, 0) = -v(1);
    crm(2, 1) =  v(0);
    return crm;
}

/// @brief Spatial cross-product matrix (6x6) for motion vectors
inline casadi::SX spatialCrossMatrix(const casadi::SX& v)
{
    casadi::SX angular = v(casadi::Slice(0, 3));
    casadi::SX linear = v(casadi::Slice(3, 6));
    
    casadi::SX crm_ang = crossMatrix(angular);
    casadi::SX crm_lin = crossMatrix(linear);
    
    casadi::SX result = casadi::SX::zeros(6, 6);
    result(casadi::Slice(0, 3), casadi::Slice(0, 3)) = crm_ang;
    result(casadi::Slice(3, 6), casadi::Slice(0, 3)) = crm_lin;
    result(casadi::Slice(3, 6), casadi::Slice(3, 6)) = crm_ang;
    
    return result;
}

/// @brief Rotation matrix from axis-angle using CasADi (symbolic)
inline casadi::SX rotationMatrix(char axis, const casadi::SX& angle)
{
    casadi::SX c = cos(angle);
    casadi::SX s = sin(angle);
    casadi::SX R = casadi::SX::eye(3);

    // NOTE: Using frame transformation convention (passive rotation) to match coordinateRotation
    // This is the TRANSPOSE of the standard active rotation matrix
    if (axis == 'X' || axis == 'x')
    {
        R(1, 1) =  c; R(1, 2) =  s;
        R(2, 1) = -s; R(2, 2) =  c;
    }
    else if (axis == 'Y' || axis == 'y')
    {
        R(0, 0) =  c; R(0, 2) = -s;
        R(2, 0) =  s; R(2, 2) =  c;
    }
    else if (axis == 'Z' || axis == 'z')
    {
        R(0, 0) =  c; R(0, 1) =  s;
        R(1, 0) = -s; R(1, 1) =  c;
    }
    else
        throw std::runtime_error("rotationMatrix: invalid axis '" + std::string(1, axis) + "'");

    return R;
}

/// @brief Spatial transform matrix from rotation (6x6)
inline casadi::SX spatialRotation(char axis, const casadi::SX& angle)
{
    casadi::SX R = rotationMatrix(axis, angle);
    casadi::SX X = casadi::SX::zeros(6, 6);
    X(casadi::Slice(0, 3), casadi::Slice(0, 3)) = R;
    X(casadi::Slice(3, 6), casadi::Slice(3, 6)) = R;
    return X;
}

/// @brief Joint motion subspace for revolute joint
inline casadi::SX revoluteMotionSubspace(char axis)
{
    casadi::SX S = casadi::SX::zeros(6, 1);
    if (axis == 'X' || axis == 'x')
        S(0) = 1.0;
    else if (axis == 'Y' || axis == 'y')
        S(1) = 1.0;
    else if (axis == 'Z' || axis == 'z')
        S(2) = 1.0;
    else
        throw std::runtime_error("revoluteMotionSubspace: invalid axis '" + std::string(1, axis) + "'");
    return S;
}

/// @brief Transform a motion vector using spatial transform
/// Formula: m_out = [E * m_angular, -E * [r]× * m_angular + E * m_linear]
/// where E is rotation matrix and r is translation vector
inline casadi::SX spatialTransformMotionVector(const casadi::SX& E, const casadi::SX& r, const casadi::SX& m_in)
{
    casadi::SX m_angular = m_in(casadi::Slice(0, 3));
    casadi::SX m_linear = m_in(casadi::Slice(3, 6));

    casadi::SX r_cross = crossMatrix(r);

    casadi::SX m_out_angular = mtimes(E, m_angular);
    casadi::SX m_out_linear = -mtimes(E, mtimes(r_cross, m_angular)) + mtimes(E, m_linear);

    return vertcat(m_out_angular, m_out_linear);
}

} // namespace casadi_derivatives
} // namespace grbda

#endif // GRBDA_CASADI_DERIVATIVES_H
