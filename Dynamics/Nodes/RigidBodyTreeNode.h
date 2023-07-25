#pragma once

#include "TreeNode.h"
#include "Dynamics/Joints/Joint.h"

namespace grbda
{

    using namespace spatial;

    struct RigidBodyTreeNode : TreeNode<RigidBodyTreeNode>
    {
        RigidBodyTreeNode(const Body &body, const std::shared_ptr<Joints::Base> &joint,
                          const int position_index, const int velocity_index);

        void updateKinematics();
        const DVec<double> &vJ() const { return vJ_; }
        const DMat<double> &S() const { return joint_->S(); }
        const DMat<double> &S_ring() const { return joint_->S_ring(); }

        const SpatialTransform &getAbsoluteTransformForBody(const Body &body) const;
        DVec<double> getVelocityForBody(const Body &body) const;
        void applyForceToBody(const SVec<double> &force, const Body &body);

        const Body body_;
        std::shared_ptr<Joints::Base> joint_;

        DVec<double> vJ_;
        const SpatialTransform Xtree_;
    };

} // namespace grbda
