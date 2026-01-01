#ifndef GRBDA_JOINT_DERIVATIVES_H
#define GRBDA_JOINT_DERIVATIVES_H

#include "grbda/Utils/cppTypes.h"

namespace grbda
{

    /// @brief Contract 3D tensor S_q with a vector
    /// @details Computes result(:,i) = S_q[:,:,i] * vec
    /// where S_q is represented as a vector of matrices: S_q[i] is (spatial_dim x nv)
    /// This corresponds to the MATLAB contract() function in ID_derivatives.m
    /// @param S_q Vector of nv matrices, each of size (spatial_dim x nv)
    /// @param vec Vector of size nv
    /// @param spatial_dim The spatial dimension (6 for single joints, 6*num_bodies for cluster joints)
    /// @return Matrix of size (spatial_dim x nv)
    template <typename Scalar>
    DMat<Scalar> contractSqWithVector(const std::vector<DMat<Scalar>> &S_q,
                                       const DVec<Scalar> &vec,
                                       int spatial_dim)
    {
        if (S_q.empty())
        {
            return DMat<Scalar>::Zero(spatial_dim, vec.size());
        }

        const int nv = vec.size();
        DMat<Scalar> result = DMat<Scalar>::Zero(S_q[0].rows(), nv);

        for (int i = 0; i < nv; ++i)
        {
            result.col(i) = S_q[i] * vec; // S_q[i] is (spatial_dim x nv), vec is (nv x 1)
        }

        return result;
    }

    /// @brief Contract transpose of 3D tensor S_q with a vector
    /// @details Computes result(i,j) = S_q[:,:,i]^T * vec for each output column j
    /// This corresponds to the MATLAB contractT() function in ID_derivatives.m
    /// @param S_q Vector of nv matrices, each of size (spatial_dim x nv)
    /// @param vec Vector of size spatial_dim
    /// @return Matrix of size (nv x nv)
    template <typename Scalar>
    DMat<Scalar> contractSqTransposeWithVector(const std::vector<DMat<Scalar>> &S_q,
                                                const DVec<Scalar> &vec)
    {
        const int nv = S_q.size();
        if (nv == 0)
        {
            return DMat<Scalar>::Zero(0, 0);
        }

        DMat<Scalar> result = DMat<Scalar>::Zero(nv, nv);

        for (int i = 0; i < nv; ++i)
        {
            // S_q[i] is (spatial_dim x nv), vec is (spatial_dim x 1)
            // S_q[i].transpose() is (nv x spatial_dim)
            // result is (nv x nv), so result.col(i) is (nv x 1)
            result.col(i) = S_q[i].transpose() * vec;
        }

        return result;
    }

} // namespace grbda

#endif // GRBDA_JOINT_DERIVATIVES_H
