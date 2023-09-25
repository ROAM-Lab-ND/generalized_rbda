#include "RigidBodyTreeNode.h"

namespace grbda
{

    RigidBodyTreeNode::RigidBodyTreeNode(const Body<> &body,
                                         const std::shared_ptr<Joints::Base<>> &joint,
                                         const int position_index, const int velocity_index,
                                         const int motion_subspace_index)
        : TreeNode(body.index_, body.name_, body.parent_index_, 1,
                   motion_subspace_index, 6,
                   position_index, joint->numPositions(),
                   velocity_index, joint->numVelocities()),
          body_(body), joint_(joint), Xtree_(body.Xtree_)
    {
        I_ = body.inertia_.getMatrix();
        Xup_.appendTransformWithClusterAncestorSubIndex(spatial::Transform{}, 0);
        Xa_.appendTransform(spatial::Transform{});
    }

    void RigidBodyTreeNode::updateKinematics()
    {
        joint_->updateKinematics(joint_state_.position, joint_state_.velocity);
        Xup_[0] = joint_->XJ() * Xtree_;
        vJ_ = joint_->S() * joint_state_.velocity;
    }

    const spatial::Transform<> &RigidBodyTreeNode::getAbsoluteTransformForBody(const Body<> &body)
    {
        return Xa_[0];
    };

    DVec<double> RigidBodyTreeNode::getVelocityForBody(const Body<> &body)
    {
        return v_;
    };

    void RigidBodyTreeNode::applyForceToBody(const SVec<double> &force, const Body<> &body)
    {
        f_ext_.segment<6>(0) = force;
    }

} // namespace grbda
