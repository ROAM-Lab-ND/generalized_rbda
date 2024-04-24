#ifndef GRBDA_RIGID_BODY_TREE_NODE_H
#define GRBDA_RIGID_BODY_TREE_NODE_H

#include "grbda/Dynamics/Nodes/TreeNode.h"
#include "grbda/Dynamics/Joints/Joint.h"

namespace grbda
{

    template <typename Scalar = double>
    struct RigidBodyTreeNode : TreeNode<Scalar>
    {
        RigidBodyTreeNode(const Body<Scalar> &body,
                          const std::shared_ptr<Joints::Base<Scalar>> &joint,
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

        const Body<Scalar> body_;
        std::shared_ptr<Joints::Base<Scalar>> joint_;

        DVec<Scalar> vJ_;
        DVec<Scalar> cJ_ = DVec<Scalar>::Zero(6);
        const spatial::Transform<Scalar> Xtree_;
    };

} // namespace grbda

#endif // GRBDA_RIGID_BODY_TREE_NODE_H
