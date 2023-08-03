#pragma once

#include "TreeNode.h"
#include "Dynamics/Joints/Joint.h"

namespace grbda
{

    using namespace spatial;

    struct RigidBodyTreeNode : TreeNode
    {
        RigidBodyTreeNode(const Body &body, const std::shared_ptr<Joints::Base> &joint,
                          const int position_index, const int velocity_index,
                          const int motion_subspace_index);

        void updateKinematics() override;
        const DVec<double> &vJ() const override { return vJ_; }
        const DMat<double> &S() const override { return joint_->S(); }
        const DMat<double> &S_ring() const override { return joint_->S_ring(); }

        const SpatialTransform &getAbsoluteTransformForBody(const Body &body) override;
        DVec<double> getVelocityForBody(const Body &body) override;
        void applyForceToBody(const SVec<double> &force, const Body &body) override;

        const Body body_;
        std::shared_ptr<Joints::Base> joint_;

        DVec<double> vJ_;
        const SpatialTransform Xtree_;
    };

} // namespace grbda
