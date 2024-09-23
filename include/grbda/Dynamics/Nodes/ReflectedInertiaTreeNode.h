#ifndef GRBDA_REFLECTED_INERTIA_TREE_NODE_H
#define GRBDA_REFLECTED_INERTIA_TREE_NODE_H

#include "grbda/Dynamics/Nodes/TreeNode.h"
#include "grbda/Dynamics/Joints/Joint.h"

namespace grbda
{

    template <typename Scalar = double>
    struct ReflectedInertiaTreeNode : TreeNode<Scalar>
    {
        ReflectedInertiaTreeNode(const int index, const Body<Scalar> &link,
                                 const std::shared_ptr<Joints::Base<Scalar>> &joint,
                                 const int parent_index,
                                 const int position_index, const int velocity_index,
                                 const int motion_subspace_index);

        void updateKinematics() override;
        const DVec<Scalar> &vJ() const override { return vJ_; }
        const DMat<Scalar> &S() const override { return joint_->S(); }
        const DVec<Scalar> &cJ() const override { return cJ_; }

        const spatial::Transform<Scalar> &getAbsoluteTransformForBody(const Body<Scalar> &body) override;
        DVec<Scalar> getVelocityForBody(const Body<Scalar> &body) override;
        DVec<Scalar> getAccelerationForBody(const Body<Scalar> &body) override;
        void applyForceToBody(const SVec<Scalar> &force, const Body<Scalar> &body) override;

        const Body<Scalar> link_;
        const std::shared_ptr<Joints::Base<Scalar>> joint_;

        DVec<Scalar> vJ_;
        DVec<Scalar> cJ_ = DVec<Scalar>::Zero(6);
        const spatial::Transform<Scalar> Xtree_;

        Mat6<Scalar> IA_;    // articulated body inertia
        SVec<Scalar> pA_;    // articulated body bias force
        D6Mat<Scalar> U_;    // helper variable for ABA
        DMat<Scalar> D_inv_; // helper variable for ABA
        DVec<Scalar> u_;     // helper variable for ABA

        Mat6<Scalar> ChiUp_; // Articulated transform
        DMat<Scalar> qdd_for_subtree_due_to_subtree_root_joint_qdd;
        DMat<Scalar> K_;
    };

} // namespace grbda

#endif // GRBDA_REFLECTED_INERTIA_TREE_NODE_H
