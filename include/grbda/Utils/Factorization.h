#ifndef GRBDA_DYNAMICS_FACTORIZATION_H
#define GRBDA_DYNAMICS_FACTORIZATION_H

#include "cppTypes.h"

namespace grbda
{

    namespace factorization
    {

        // From Featherstone Ch 6.5
        template <typename Scalar>
        class LTL : public DMat<Scalar>
        {
        public:
            LTL(DMat<Scalar> H, const std::vector<int> &expanded_tree_parent_indices);

            DVec<Scalar> product(const DVec<Scalar> &x) const;

            DVec<Scalar> transposeProduct(const DVec<Scalar> &x) const;
            DMat<Scalar> transposeMatrixProduct(const DMat<Scalar> &X) const;

            void inverseProductInSitu(DVec<Scalar> &x) const;
            DVec<Scalar> inverseProduct(const DVec<Scalar> &x) const;

            void inverseTransposeProductInSitu(DVec<Scalar> &x) const;
            DVec<Scalar> inverseTransposeProduct(const DVec<Scalar> &x) const;
            DMat<Scalar> inverseTransposeMatrixProduct(const DMat<Scalar> &X) const;

            DVec<Scalar> solve(const DVec<Scalar> &x) const;

        private:
            int parent(const int i) const { return expanded_tree_parent_indices_[i]; }
            std::vector<int> expanded_tree_parent_indices_;
        };

    }

} // namespace grbda

#endif // GRBDA_DYNAMICS_FACTORIZATION_H
