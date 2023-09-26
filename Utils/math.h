#ifndef GRBDA_MAT_H
#define GRBDA_MATH_H

#include "cppTypes.h"

namespace grbda
{
    namespace math
    {
        template <typename T>
        bool isPositiveDefinite(const Eigen::MatrixBase<T> &A)
        {
            // Symmetry check
            if (!A.isApprox(A.transpose()))
            {
                return false;
            }

            // Positive eignvalue check
            Eigen::LLT<T> llt_of_A(A);
            if (llt_of_A.info() == Eigen::NumericalIssue)
            {
                return false;
            }

            return true;
        }

        inline DMat<double> matrixInverse(const DMat<double> &mat)
        {
            return mat.inverse();
        }

        inline DMat<casadi::SX> matrixInverse(const DMat<casadi::SX> &mat)
        {
            casadi::SX cs_mat(mat.rows(), mat.cols());
            casadi::copy(mat, cs_mat);

            casadi::SX cs_mat_inv = casadi::SX::inv(cs_mat);

            DMat<casadi::SX> mat_out(mat.rows(), mat.cols());
            casadi::copy(cs_mat_inv, mat_out);

            return mat_out;
        }
    }
}

#endif // GRBDA_MATH_H
