#ifndef GRBDA_MATH_H
#define GRBDA_MATH_H

#include "cppTypes.h"

namespace grbda
{
    namespace math
    {

        /*!
         * Square a number
         */
        template <typename T>
        T square(T a)
        {
            return a * a;
        }

        /*!
         * Square root of a number
         */
        inline double sqrt(double a)
        {
            return std::sqrt(a);
        }

        inline casadi::SX sqrt(casadi::SX a)
        {
            return casadi::SX::sqrt(a);
        }

        /*!
         * Convert radians to degrees
         */
        template <typename T>
        T rad2deg(T rad)
        {
            return rad * T(180) / T(M_PI);
        }

        /*!
         * Convert degrees to radians
         */
        template <typename T>
        T deg2rad(T deg)
        {
            return deg * T(M_PI) / T(180);
        }

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

        // TODO(@MatthewChignoli): Or I could specialize these functions for double vs casadi::SX and for the double one I can call Eigen::orthogonalDecomposition.pseudoInverse()
        // TODO(@MatthewChignoli): Unit test against Eigen::orthogonalDecomposition.pseudoInverse()
        // Applicable when mat has linearly independent columns
        template <typename T>
        DMat<typename T::Scalar> matrixLeftPseudoInverse(const Eigen::MatrixBase<T> &mat)
        {
            const DMat<typename T::Scalar> tmp = mat.transpose() * mat;
            return matrixInverse(tmp) * mat.transpose();
        }

        // Applicable when mat has linearly independent rows
        template <typename T>
        DMat<typename T::Scalar> matrixRightPseudoInverse(const Eigen::MatrixBase<T> &mat)
        {
            const DMat<typename T::Scalar> tmp = mat * mat.transpose();
            return mat.transpose() * matrixInverse(tmp);
        }

        inline DVec<double> solveLinearSystem(const DMat<double> &A, const DVec<double> &b)
        {
            return A.colPivHouseholderQr().solve(b);
        }

        // TODO(@MatthewChignoli): Unit test this
        inline DVec<casadi::SX> solveLinearSystem(const DMat<casadi::SX> &A,
                                                  const DVec<casadi::SX> &b)
        {
            casadi::SX cs_A(A.rows(), A.cols());
            casadi::copy(A, cs_A);

            casadi::SX cs_b(b.rows(), b.cols());
            casadi::copy(b, cs_b);

            casadi::SX cs_x = casadi::SX::solve(cs_A, cs_b);

            DVec<casadi::SX> x(cs_x.size1());
            casadi::copy(cs_x, x);

            return x;
        }

        class CasadiLLT
        {
        public:
            CasadiLLT() {}
            CasadiLLT(const DMat<casadi::SX> &mat)
            {
                Ainv_.resize(mat.rows(), mat.cols());

                casadi::SX cs_mat(mat.rows(), mat.cols());
                casadi::copy(mat, cs_mat);

                casadi::SX cs_mat_inv = casadi::SX::inv(cs_mat);

                DMat<casadi::SX> mat_out(mat.rows(), mat.cols());
                casadi::copy(cs_mat_inv, Ainv_);
            }

            template <typename Derived>
            DMat<casadi::SX> solve(const Eigen::MatrixBase<Derived> &b) const
            {
                return Ainv_ * b;
            }

        private:
            DMat<casadi::SX> Ainv_;
        };

        class CasadiInverse
        {
        public:
            CasadiInverse() {}
            CasadiInverse(const DMat<casadi::SX> &mat) {}
        };
    }
}

#endif // GRBDA_MATH_H
