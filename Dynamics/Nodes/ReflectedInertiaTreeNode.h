#pragma once

#include <optional>

#include "TreeNode.h"
#include "Dynamics/Joints/Joint.h"

namespace grbda
{

    using namespace spatial;

    struct ReflectedInertiaTreeNode : TreeNode
    {
        ReflectedInertiaTreeNode(const int index, const Body &link,
                                 const std::shared_ptr<Joints::Base> &joint, const int parent_index,
                                 const int position_index, const int velocity_index);

        void updateKinematics() override;
        const DVec<double> &vJ() const override { return vJ_; }
        const DMat<double> &S() const override { return joint_->S(); }
        const DVec<double> &cJ() const override { return cJ_; }

        const SpatialTransform &getAbsoluteTransformForBody(const Body &body) override;
        DVec<double> getVelocityForBody(const Body &body) override;
        void applyForceToBody(const SVec<double> &force, const Body &body) override;

        const Body link_;
        std::shared_ptr<Joints::Base> joint_;

        DVec<double> vJ_;
        const SpatialTransform Xtree_;

        Mat6<double> IA_;    // articulated body inertia
        SVec<double> pA_;    // articulated body bias force
        D6Mat<double> U_;    // helper variable for ABA
        DMat<double> D_inv_; // helper variable for ABA
        DVec<double> u_;     // helper variable for ABA
        Mat6<double> ChiUp_; // Articulated transform
        DMat<double> qdd_for_subtree_due_to_subtree_root_joint_qdd;

    };

} // namespace grbda
