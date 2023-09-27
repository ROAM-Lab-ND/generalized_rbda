#include "ClusterTreeNode.h"

#include "Utils/math.h"

namespace grbda
{

    template <typename Scalar, typename InverseType>
    ClusterTreeNode<Scalar, InverseType>::ClusterTreeNode(int index, std::string name,
                                                          std::vector<Body<Scalar>> &bodies,
                                                          ClusterJointPtr joint,
                                                          int parent_index, int num_parent_bodies,
                                                          int position_index, int velocity_index,
                                                          int motion_ss_index)
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

    template <typename Scalar, typename InverseType>
    void ClusterTreeNode<Scalar, InverseType>::updateKinematics()
    {
        joint_->updateKinematics(this->joint_state_);
        joint_->computeSpatialTransformFromParentToCurrentCluster(this->Xup_);
    }

    template <typename Scalar, typename InverseType>
    void ClusterTreeNode<Scalar, InverseType>::updateDinv(const DMat<Scalar> &D)
    {
        D_inv_ = InverseType(D);
    }

    template <typename Scalar, typename InverseType>
    const spatial::Transform<Scalar> &
    ClusterTreeNode<Scalar, InverseType>::getAbsoluteTransformForBody(const Body<Scalar> &body)
    {
        return this->Xa_.getTransformForOutputBody(body.sub_index_within_cluster_);
    }

    template <typename Scalar, typename InverseType>
    DVec<Scalar> ClusterTreeNode<Scalar, InverseType>::getVelocityForBody(const Body<Scalar> &body)
    {
        return this->v_.template segment<6>(6 * body.sub_index_within_cluster_);
    }

    template <typename Scalar, typename InverseType>
    void ClusterTreeNode<Scalar, InverseType>::applyForceToBody(const SVec<Scalar> &force,
                                                                const Body<Scalar> &body)
    {
        this->f_ext_.template segment<6>(6 * body.sub_index_within_cluster_) += force;
    }

    template <typename Scalar, typename InverseType>
    bool ClusterTreeNode<Scalar, InverseType>::containsBody(int body_index) const
    {
        for (const auto &body : bodies_)
        {
            if (body.index_ == body_index)
                return true;
        }
        return false;
    }

    template <typename Scalar, typename InverseType>
    std::vector<std::pair<Body<Scalar>, JointPtr<Scalar>>>
    ClusterTreeNode<Scalar, InverseType>::bodiesAndJoints() const
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

    template <typename Scalar, typename InverseType>
    std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
    ClusterTreeNode<Scalar, InverseType>::bodiesJointsAndReflectedInertias() const
    {
        return joint_->bodiesJointsAndReflectedInertias();
    }

    template class ClusterTreeNode<double, Eigen::ColPivHouseholderQR<DMat<double>>>;
    template class ClusterTreeNode<casadi::SX, math::CasadiInverse>;

} // namespace grbda
