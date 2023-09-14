#ifndef GRBDA_DYNAMICS_FACTORIZATION_H

#include "Dynamics/Nodes/RigidBodyTreeNode.h"

namespace grbda
{

    namespace factorization
    {

        // From Featherstone Ch 6.5
        class LTL : public DMat<double>
        {
        public:
            LTL(DMat<double> H, const std::vector<std::shared_ptr<RigidBodyTreeNode>> &nodes);

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
            int parent(const int i) const { return nodes_[i]->parent_index_; }
            std::vector<std::shared_ptr<RigidBodyTreeNode>> nodes_;
        };

    }

} // namespace grbda

#endif // GRBDA_DYNAMICS_FACTORIZATION_H
