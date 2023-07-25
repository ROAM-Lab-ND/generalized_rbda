#include "Factorization.h"

namespace grbda
{

    namespace factorization
    {

        LTL::LTL(DMat<double> H, const std::vector<RigidBodyTreeNode> &nodes) : nodes_(nodes)
        {
            // ISSUE #14
            for (const RigidBodyTreeNode &node : nodes)
            {
                if (node.joint_->numVelocities() > 1)
                {
                    std::cout << "Warning: slow factorization due to multi-dof joint" << std::endl;
                    this->DMat<double>::operator=(DMat<double>::Zero(0, 0));
                    return;
                }
            }

            for (int k = H.rows() - 1; k > -1; k--)
            {
                H(k, k) = std::sqrt(H(k, k));
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
            this->DMat<double>::operator=(H.triangularView<Eigen::Lower>());
        }

        DVec<double> LTL::product(const DVec<double> &x) const
        {
            const int n = x.rows();
            DVec<double> y = DVec<double>::Zero(n);
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

        DVec<double> LTL::transposeProduct(const DVec<double> &x) const
        {
            const int n = x.rows();
            DVec<double> y = DVec<double>::Zero(n);
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

        DMat<double> LTL::transposeMatrixProduct(const DMat<double> &X) const
        {
            DMat<double> Y = DMat<double>(cols(), X.cols());
            for (int i = 0; i < X.cols(); i++)
            {
                Y.col(i) = transposeProduct(X.col(i));
            }
            return Y;
        }

        void LTL::inverseProductInSitu(DVec<double> &x) const
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

        DVec<double> LTL::inverseProduct(const DVec<double> &x) const
        {
            DVec<double> y = x;
            inverseProductInSitu(y);
            return y;
        }

        void LTL::inverseTransposeProductInSitu(DVec<double> &x) const
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

        DVec<double> LTL::inverseTransposeProduct(const DVec<double> &x) const
        {
            DVec<double> y = x;
            inverseTransposeProductInSitu(y);
            return y;
        }

        DMat<double> LTL::inverseTransposeMatrixProduct(const DMat<double> &X) const
        {
            DMat<double> Y = X;
            for (int i = 0; i < X.cols(); i++)
                Y.col(i) = inverseTransposeProduct(Y.col(i));
            return Y;
        }

        DVec<double> LTL::solve(const DVec<double> &x) const
        {
            return inverseProduct(inverseTransposeProduct(x));
        }

    }

} // namespace grbda
