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
         * Generate a random number
         */
        template <typename T>
        T random()
        {
            return T(std::rand()) / T(RAND_MAX);
        }

        /*!
         * Generate a std::vector of random numbers
         */
        template <typename T>
        std::vector<T> random(int n)
        {
            std::vector<T> v(n);
            for (int i = 0; i < n; ++i)
            {
                v[i] = random<T>();
            }
            return v;
        }

        /*!
         * Generate a std::vector of zeros
         */
        template <typename T>
        std::vector<T> zeros(int n)
        {
            std::vector<T> v(n);
            for (int i = 0; i < n; ++i)
            {
                v[i] = T(0);
            }
            return v;
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

        /*!
         * Check if an Eigen matrix is positive definite
         */
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

        /*!
         * Overloaded functions for taking the inverse of a Eigen matrix
         * - Standard Eigen matrices are inverted using the .inverse() function
         * - Casadi matrices are symbolically inverted using the casadi::SX::inv() function
         */
        template <typename T>
        DMat<typename T::Scalar> matrixInverse(const Eigen::MatrixBase<T> &mat)
        {
            return mat.inverse();
        }

        template <>
        inline DMat<casadi::SX>
        matrixInverse<DMat<casadi::SX>>(const Eigen::MatrixBase<DMat<casadi::SX>> &mat)
        {
            casadi::SX cs_mat(mat.rows(), mat.cols());
            casadi::copy(mat, cs_mat);

            casadi::SX cs_mat_inv = casadi::SX::inv(cs_mat);

            DMat<casadi::SX> mat_out(mat.rows(), mat.cols());
            casadi::copy(cs_mat_inv, mat_out);

            return mat_out;
        }

        /*!
         * Left pseudo inverse of an Eigen matrix.
         * Applicable when mat has linearly independent columns
         */
        template <typename T>
        DMat<typename T::Scalar> matrixLeftPseudoInverse(const Eigen::MatrixBase<T> &mat)
        {
            return mat.completeOrthogonalDecomposition().pseudoInverse();
        }

        template <>
        inline DMat<casadi::SX>
        matrixLeftPseudoInverse<DMat<casadi::SX>>(const Eigen::MatrixBase<DMat<casadi::SX>> &mat)
        {
            const DMat<casadi::SX> tmp = mat.transpose() * mat;
            return matrixInverse(tmp) * mat.transpose();
        }

        /*!
         * Right pseudo inverse of an Eigen matrix.
         * Applicable when mat has linearly independent rows
         */
        template <typename T>
        DMat<typename T::Scalar> matrixRightPseudoInverse(const Eigen::MatrixBase<T> &mat)
        {
            return mat.completeOrthogonalDecomposition().pseudoInverse();
        }

        template <>
        inline DMat<casadi::SX>
        matrixRightPseudoInverse<DMat<casadi::SX>>(const Eigen::MatrixBase<DMat<casadi::SX>> &mat)
        {
            const DMat<casadi::SX> tmp = mat * mat.transpose();
            return mat.transpose() * matrixInverse(tmp);
        }

        /*!
         * Solve a linear system of Eigen matrices
         */
        template <typename T>
        DVec<typename T::Scalar> solveLinearSystem(const Eigen::MatrixBase<T> &A,
                                                   const DVec<typename T::Scalar> &b)
        {
            return A.colPivHouseholderQr().solve(b);
        }

        template <>
        inline DVec<casadi::SX> solveLinearSystem(const Eigen::MatrixBase<DMat<casadi::SX>> &A,
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

        /*!
         * Analaogous to Eigen::LLT, but for casadi::SX matrices
         */
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

        template <typename Scalar>
        struct CorrectMatrixLltType
        {
            using type = Eigen::LLT<DMat<Scalar>>;
        };

        // Specialization for casadi::SX
        template <>
        struct CorrectMatrixLltType<casadi::SX>
        {
            using type = math::CasadiLLT;
        };

        /*!
         * Analaogous to Eigen::ColPivHouseholderQR, but for casadi::SX matrices
         */
        class CasadiInverse
        {
        public:
            CasadiInverse() {}
            CasadiInverse(const DMat<casadi::SX> &mat)
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

        template <typename Scalar>
        struct CorrectMatrixInverseType
        {
            using type = Eigen::ColPivHouseholderQR<DMat<Scalar>>;
        };

        // Specialization for casadi::SX
        template <>
        struct CorrectMatrixInverseType<casadi::SX>
        {
            using type = math::CasadiInverse;
        };
    }
}

#endif // GRBDA_MATH_H
