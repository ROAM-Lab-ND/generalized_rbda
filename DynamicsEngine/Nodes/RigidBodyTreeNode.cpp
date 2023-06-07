#include "RigidBodyTreeNode.h"

RigidBodyTreeNode::RigidBodyTreeNode(const Body &body, const std::shared_ptr<Joints::Base> &joint,
                                     const int position_index, const int velocity_index)
    : TreeNode(body.index_, body.name_, body.parent_index_, 6, 1, position_index, joint->numPositions(), velocity_index, joint->numVelocities()),
      body_(body), joint_(joint), Xtree_(body.Xtree_)
{
    I_ = body.inertia_.getMatrix();
    Xup_.appendSpatialTransformWithClusterAncestorSubIndex(SpatialTransform{}, 0);
    Xa_.appendSpatialTransform(SpatialTransform{});
}

void RigidBodyTreeNode::updateKinematics()
{
    joint_->updateKinematics(q_, qd_);
    Xup_[0] = joint_->XJ() * Xtree_;
    vJ_ = joint_->S() * qd_;
}

const SpatialTransform &RigidBodyTreeNode::getAbsoluteTransformForBody(const Body &body)
{
    return Xa_[0];
};

DVec<double> RigidBodyTreeNode::getVelocityForBody(const Body &body)
{
    return v_;
};

void RigidBodyTreeNode::applyForceToBody(const SVec<double> &force, const Body &body)
{
    f_ext_.segment<6>(0) = force;
}
