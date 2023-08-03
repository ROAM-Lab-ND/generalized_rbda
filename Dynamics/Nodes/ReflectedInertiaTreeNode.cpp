#include "ReflectedInertiaTreeNode.h"

namespace grbda
{

    ReflectedInertiaTreeNode::ReflectedInertiaTreeNode(const int index, const Body &link,
                                                       const std::shared_ptr<Joints::Base> &joint,
                                                       const int parent_index,
                                                       const int position_index,
                                                       const int velocity_index,
                                                       const int motion_subspace_index)
        : TreeNode(index, link.name_, parent_index, 1,
                   motion_subspace_index, 6,
                   position_index, joint->numPositions(),
                   velocity_index, joint->numVelocities()),
          link_(link), joint_(joint), Xtree_(link.Xtree_)
    {
        I_ = link.inertia_.getMatrix();
        Xup_.appendSpatialTransformWithClusterAncestorSubIndex(SpatialTransform{}, 0);
        Xa_.appendSpatialTransform(SpatialTransform{});
    }

    void ReflectedInertiaTreeNode::updateKinematics()
    {
        joint_->updateKinematics(joint_state_.position, joint_state_.velocity);
        Xup_[0] = joint_->XJ() * Xtree_;
        vJ_ = joint_->S() * joint_state_.velocity;
    }

    const SpatialTransform &ReflectedInertiaTreeNode::getAbsoluteTransformForBody(const Body &body)
    {
        return Xa_[0];
    };

    DVec<double> ReflectedInertiaTreeNode::getVelocityForBody(const Body &body)
    {
        return v_;
    };

    void ReflectedInertiaTreeNode::applyForceToBody(const SVec<double> &force, const Body &body)
    {
        f_ext_.segment<6>(0) = force;
    }

} // namespace grbda
