#include "grbda/Dynamics/Nodes/RigidBodyTreeNode.h"

namespace grbda
{

    template <typename Scalar>
    RigidBodyTreeNode<Scalar>::RigidBodyTreeNode(const Body<Scalar> &body,
                                                 const std::shared_ptr<Joints::Base<Scalar>> &joint,
                                                 const int position_index, const int velocity_index,
                                                 const int motion_subspace_index)
        : TreeNode<Scalar>(body.index_, body.name_, body.parent_index_, 1,
                           motion_subspace_index, 6,
                           position_index, joint->numPositions(),
                           velocity_index, joint->numVelocities()),
          body_(body), joint_(joint), Xtree_(body.Xtree_)
    {
        this->I_ = body.inertia_.getMatrix();
        this->Xup_.appendTransformWithClusterAncestorSubIndex(spatial::Transform<Scalar>{}, 0);
        this->Xa_.appendTransform(spatial::Transform<Scalar>{});
    }

    template <typename Scalar>
    void RigidBodyTreeNode<Scalar>::updateKinematics()
    {
        joint_->updateKinematics(this->joint_state_.position, this->joint_state_.velocity);
        this->Xup_[0] = joint_->XJ() * this->Xtree_;
        this->vJ_ = joint_->S() * this->joint_state_.velocity;
    }

    template <typename Scalar>
    const spatial::Transform<Scalar> &
    RigidBodyTreeNode<Scalar>::getAbsoluteTransformForBody(const Body<Scalar> &body)
    {
        return this->Xa_[0];
    };

    template <typename Scalar>
    DVec<Scalar> RigidBodyTreeNode<Scalar>::getVelocityForBody(const Body<Scalar> &body)
    {
        return this->v_;
    };

    template <typename Scalar>
    DVec<Scalar> RigidBodyTreeNode<Scalar>::getAccelerationForBody(const Body<Scalar> &body)
    {
        return this->a_;
    };

    template <typename Scalar>
    void RigidBodyTreeNode<Scalar>::applyForceToBody(const SVec<Scalar> &force,
                                                     const Body<Scalar> &body)
    {
        this->f_ext_.template segment<6>(0) = force;
    }

    template struct RigidBodyTreeNode<double>;
    template struct RigidBodyTreeNode<float>;
    template struct RigidBodyTreeNode<casadi::SX>;

} // namespace grbda
