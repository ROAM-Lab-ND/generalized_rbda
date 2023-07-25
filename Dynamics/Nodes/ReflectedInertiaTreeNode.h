#pragma once

#include <optional>

#include "TreeNode.h"
#include "Dynamics/Joints/Joint.h"

namespace grbda
{

    using namespace spatial;

    struct ReflectedInertiaTreeNode : TreeNode<ReflectedInertiaTreeNode>
    {
        ReflectedInertiaTreeNode(const int index, const Body &link,
                                 const std::shared_ptr<Joints::Base> &joint, const int parent_index,
                                 const int position_index, const int velocity_index);

        void updateKinematics();
        const DVec<double> &vJ() const { return vJ_; }
        const DMat<double> &S() const { return joint_->S(); }
        const DMat<double> &S_ring() const { return joint_->S_ring(); }

        const SpatialTransform &getAbsoluteTransformForBody(const Body &body) const;
        DVec<double> getVelocityForBody(const Body &body) const;
        void applyForceToBody(const SVec<double> &force, const Body &body);

        const Body link_;
        std::shared_ptr<Joints::Base> joint_;

        DVec<double> vJ_;
        const SpatialTransform Xtree_;

        Mat6<double> IA_;    // articulated body inertia
        SVec<double> pA_;    // articulated body bias force
        D6Mat<double> U_;    // helper variable for ABA
        DMat<double> D_inv_; // helper variable for ABA
        DVec<double> u_;     // helper variable for ABA
    };

} // namespace grbda
