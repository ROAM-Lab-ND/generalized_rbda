#ifndef GRBDA_DYNAMICS_FACTORIZATION_H
#define GRBDA_DYNAMICS_FACTORIZATION_H

#include "cppTypes.h"

namespace grbda
{

    namespace factorization
    {

        // From Featherstone Ch 6.5
        class LTL : public DMat<double>
        {
        public:
            LTL(DMat<double> H, const std::vector<int> &expanded_tree_parent_indices);

            DVec<double> product(const DVec<double> &x) const;

            DVec<double> transposeProduct(const DVec<double> &x) const;
            DMat<double> transposeMatrixProduct(const DMat<double> &X) const;

            void inverseProductInSitu(DVec<double> &x) const;
            DVec<double> inverseProduct(const DVec<double> &x) const;

            void inverseTransposeProductInSitu(DVec<double> &x) const;
            DVec<double> inverseTransposeProduct(const DVec<double> &x) const;
            DMat<double> inverseTransposeMatrixProduct(const DMat<double> &X) const;

            DVec<double> solve(const DVec<double> &x) const;

        private:
            int parent(const int i) const { return expanded_tree_parent_indices_[i]; }
            std::vector<int> expanded_tree_parent_indices_;
        };

    }

} // namespace grbda

#endif // GRBDA_DYNAMICS_FACTORIZATION_H
