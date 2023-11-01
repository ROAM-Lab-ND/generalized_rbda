#include "grbda/Dynamics/Nodes/ReflectedInertiaTreeNode.h"

namespace grbda
{

    template <typename Scalar>
    ReflectedInertiaTreeNode<Scalar>::ReflectedInertiaTreeNode(
        const int index, const Body<Scalar> &link,
        const std::shared_ptr<Joints::Base<Scalar>> &joint,
        const int parent_index, const int position_index, const int velocity_index,
        const int motion_subspace_index)
        : TreeNode<Scalar>(index, link.name_, parent_index, 1,
                           motion_subspace_index, 6,
                           position_index, joint->numPositions(),
                           velocity_index, joint->numVelocities()),
          link_(link), joint_(joint), Xtree_(link.Xtree_)
    {
        this->I_ = link.inertia_.getMatrix();
        this->Xup_.appendTransformWithClusterAncestorSubIndex(spatial::Transform<Scalar>{}, 0);
        this->Xa_.appendTransform(spatial::Transform<Scalar>{});
    }

    template <typename Scalar>
    void ReflectedInertiaTreeNode<Scalar>::updateKinematics()
    {
        joint_->updateKinematics(this->joint_state_.position, this->joint_state_.velocity);
        this->Xup_[0] = joint_->XJ() * this->Xtree_;
        this->vJ_ = joint_->S() * this->joint_state_.velocity;
    }

    template <typename Scalar>
    const spatial::Transform<Scalar> &
    ReflectedInertiaTreeNode<Scalar>::getAbsoluteTransformForBody(const Body<Scalar> &body)
    {
        return this->Xa_[0];
    };

    template <typename Scalar>
    DVec<Scalar> ReflectedInertiaTreeNode<Scalar>::getVelocityForBody(const Body<Scalar> &body)
    {
        return this->v_;
    };

    template <typename Scalar>
    void ReflectedInertiaTreeNode<Scalar>::applyForceToBody(const SVec<Scalar> &force,
                                                            const Body<Scalar> &body)
    {
        this->f_ext_.template segment<6>(0) = force;
    }

    template struct ReflectedInertiaTreeNode<double>;
    template struct ReflectedInertiaTreeNode<float>;
    template struct ReflectedInertiaTreeNode<casadi::SX>;

} // namespace grbda
