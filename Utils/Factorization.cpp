#include "Factorization.h"

#include "math.h"

namespace grbda
{

    namespace factorization
    {

        template <typename Scalar>
        LTL<Scalar>::LTL(DMat<Scalar> H, const std::vector<int> &expanded_tree_parent_indices)
            : expanded_tree_parent_indices_(expanded_tree_parent_indices)
        {
            for (int k = H.rows() - 1; k > -1; k--)
            {
                H(k, k) = sqrt(H(k, k));
                int i = parent(k);
                while (i != -1)
                {
                    H(k, i) = H(k, i) / H(k, k);
                    i = parent(i);
                }

                i = parent(k);
                while (i != -1)
                {
                    int j = i;
                    while (j != -1)
                    {
                        H(i, j) = H(i, j) - H(k, i) * H(k, j);
                        j = parent(j);
                    }
                    i = parent(i);
                }
            }
            this->DMat<Scalar>::operator=(H.template triangularView<Eigen::Lower>());
        }

        template <typename Scalar>
        DVec<Scalar> LTL<Scalar>::product(const DVec<Scalar> &x) const
        {
            const int n = x.rows();
            DVec<Scalar> y = DVec<Scalar>::Zero(n);
            for (int i = n - 1; i > -1; i--)
            {
                y(i) = (*this)(i, i) * x(i);
                int j = parent(i);
                while (j != -1)
                {
                    y(i) += (*this)(i, j) * x(j);
                    j = parent(j);
                }
            }
            return y;
        }

        template <typename Scalar>
        DVec<Scalar> LTL<Scalar>::transposeProduct(const DVec<Scalar> &x) const
        {
            const int n = x.rows();
            DVec<Scalar> y = DVec<Scalar>::Zero(n);
            for (int i = 0; i < n; i++)
            {
                y(i) = (*this)(i, i) * x(i);
                int j = parent(i);
                while (j != -1)
                {
                    y(j) = y(j) + (*this)(i, j) * x(i);
                    j = parent(j);
                }
            }
            return y;
        }

        template <typename Scalar>
        DMat<Scalar> LTL<Scalar>::transposeMatrixProduct(const DMat<Scalar> &X) const
        {
            DMat<Scalar> Y = DMat<Scalar>(this->cols(), X.cols());
            for (int i = 0; i < X.cols(); i++)
            {
                Y.col(i) = transposeProduct(X.col(i));
            }
            return Y;
        }

        template <typename Scalar>
        void LTL<Scalar>::inverseProductInSitu(DVec<Scalar> &x) const
        {
            for (int i = 0; i < x.rows(); i++)
            {
                int j = parent(i);
                while (j != -1)
                {
                    x(i) -= (*this)(i, j) * x(j);
                    j = parent(j);
                }
                x(i) /= (*this)(i, i);
            }
        }

        template <typename Scalar>
        DVec<Scalar> LTL<Scalar>::inverseProduct(const DVec<Scalar> &x) const
        {
            DVec<Scalar> y = x;
            inverseProductInSitu(y);
            return y;
        }

        template <typename Scalar>
        void LTL<Scalar>::inverseTransposeProductInSitu(DVec<Scalar> &x) const
        {
            for (int i = x.rows() - 1; i > -1; i--)
            {
                x(i) /= (*this)(i, i);
                int j = parent(i);
                while (j != -1)
                {
                    x(j) -= (*this)(i, j) * x(i);
                    j = parent(j);
                }
            }
        }

        template <typename Scalar>
        DVec<Scalar> LTL<Scalar>::inverseTransposeProduct(const DVec<Scalar> &x) const
        {
            DVec<Scalar> y = x;
            inverseTransposeProductInSitu(y);
            return y;
        }

        template <typename Scalar>
        DMat<Scalar> LTL<Scalar>::inverseTransposeMatrixProduct(const DMat<Scalar> &X) const
        {
            DMat<Scalar> Y = X;
            for (int i = 0; i < X.cols(); i++)
                Y.col(i) = inverseTransposeProduct(Y.col(i));
            return Y;
        }

        template <typename Scalar>
        DVec<Scalar> LTL<Scalar>::solve(const DVec<Scalar> &x) const
        {
            return inverseProduct(inverseTransposeProduct(x));
        }

        template class LTL<double>;
        template class LTL<casadi::SX>;

    }

} // namespace grbda
