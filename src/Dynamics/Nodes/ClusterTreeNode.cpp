#include "grbda/Dynamics/Nodes/ClusterTreeNode.h"

namespace grbda
{

    template <typename Scalar>
    ClusterTreeNode<Scalar>::ClusterTreeNode(
        int index, std::string name, std::vector<Body<Scalar>> &bodies,
        ClusterJointPtr joint, int parent_index, int num_parent_bodies,
        int position_index, int velocity_index, int motion_ss_index)
        : TreeNode<Scalar>(index, name, parent_index, num_parent_bodies,
                           motion_ss_index, 6 * bodies.size(),
                           position_index, joint->numPositions(),
                           velocity_index, joint->numVelocities()),
          bodies_(bodies), joint_(joint)
    {
        for (size_t i = 0; i < bodies.size(); i++)
        {
            this->I_.template block<6, 6>(6 * i, 6 * i) = bodies[i].inertia_.getMatrix();
            this->Xup_.appendTransformWithClusterAncestorSubIndex(
                spatial::Transform<Scalar>{}, bodies[i].cluster_ancestor_sub_index_within_cluster_);
            this->Xa_.appendTransform(spatial::Transform<Scalar>{});
        }
    }

    template <typename Scalar>
    void ClusterTreeNode<Scalar>::updateKinematics()
    {
        joint_->updateKinematics(this->joint_state_);
        joint_->computeSpatialTransformFromParentToCurrentCluster(this->Xup_);
    }

    template <typename Scalar>
    void ClusterTreeNode<Scalar>::updateDinv(const DMat<Scalar> &D)
    {
        D_inv_ = InverseType(D);
    }

    template <typename Scalar>
    const spatial::Transform<Scalar> &
    ClusterTreeNode<Scalar>::getAbsoluteTransformForBody(const Body<Scalar> &body)
    {
        return this->Xa_.getTransformForOutputBody(body.sub_index_within_cluster_);
    }

    template <typename Scalar>
    DVec<Scalar> ClusterTreeNode<Scalar>::getVelocityForBody(const Body<Scalar> &body)
    {
        return this->v_.template segment<6>(6 * body.sub_index_within_cluster_);
    }

    template <typename Scalar>
    DVec<Scalar> ClusterTreeNode<Scalar>::getAccelerationForBody(const Body<Scalar> &body)
    {
        return this->a_.template segment<6>(6 * body.sub_index_within_cluster_);
    }

    template <typename Scalar>
    void ClusterTreeNode<Scalar>::applyForceToBody(const SVec<Scalar> &force,
                                                   const Body<Scalar> &body)
    {
        this->f_ext_.template segment<6>(6 * body.sub_index_within_cluster_) += force;
    }

    template <typename Scalar>
    bool ClusterTreeNode<Scalar>::containsBody(int body_index) const
    {
        for (const auto &body : bodies_)
        {
            if (body.index_ == body_index)
                return true;
        }
        return false;
    }

    template <typename Scalar>
    std::vector<std::pair<Body<Scalar>, JointPtr<Scalar>>>
    ClusterTreeNode<Scalar>::bodiesAndJoints() const
    {
        std::vector<JointPtr<Scalar>> single_joints = joint_->singleJoints();
        std::vector<std::pair<Body<Scalar>, JointPtr<Scalar>>> bodies_and_joints;
        for (int i = 0; i < (int)bodies_.size(); i++)
        {
            Body<Scalar> body = bodies_[i];
            JointPtr<Scalar> joint = single_joints[i]->clone();
            std::pair<Body<Scalar>, JointPtr<Scalar>> body_and_joint = std::make_pair(body, joint);
            bodies_and_joints.push_back(body_and_joint);
        }
        return bodies_and_joints;
    }

    template <typename Scalar>
    std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
    ClusterTreeNode<Scalar>::bodiesJointsAndReflectedInertias() const
    {
        return joint_->bodiesJointsAndReflectedInertias();
    }

    template struct ClusterTreeNode<double>;
    template struct ClusterTreeNode<float>;
    template struct ClusterTreeNode<casadi::SX>;
    template struct ClusterTreeNode<std::complex<double>>;

} // namespace grbda
