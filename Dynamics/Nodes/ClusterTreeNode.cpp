#include "ClusterTreeNode.h"

namespace grbda
{

    ClusterTreeNode::ClusterTreeNode(int index, std::string name, std::vector<Body<>> &bodies,
                                     ClusterJointPtr joint, int parent_index, int num_parent_bodies,
                                     int position_index, int velocity_index, int motion_ss_index)
        : TreeNode(index, name, parent_index, num_parent_bodies,
                   motion_ss_index, 6 * bodies.size(),
                   position_index, joint->numPositions(),
                   velocity_index, joint->numVelocities()),
          bodies_(bodies), joint_(joint)
    {
        for (size_t i = 0; i < bodies.size(); i++)
        {
            I_.block<6, 6>(6 * i, 6 * i) = bodies[i].inertia_.getMatrix();
            Xup_.appendTransformWithClusterAncestorSubIndex(
                spatial::Transform{}, bodies[i].cluster_ancestor_sub_index_within_cluster_);
            Xa_.appendTransform(spatial::Transform{});
        }
    }

    void ClusterTreeNode::updateKinematics()
    {
        joint_->updateKinematics(joint_state_);
        joint_->computeSpatialTransformFromParentToCurrentCluster(Xup_);
    }

    void ClusterTreeNode::updateDinv(const DMat<double> &D)
    {
        D_inv_ = Eigen::ColPivHouseholderQR<DMat<double>>(D);
    }

    const spatial::Transform<> &ClusterTreeNode::getAbsoluteTransformForBody(const Body<> &body)
    {
        return Xa_.getTransformForOutputBody(body.sub_index_within_cluster_);
    }

    DVec<double> ClusterTreeNode::getVelocityForBody(const Body<> &body)
    {
        return v_.segment<6>(6 * body.sub_index_within_cluster_);
    }

    void ClusterTreeNode::applyForceToBody(const SVec<double> &force, const Body<> &body)
    {
        f_ext_.segment<6>(6 * body.sub_index_within_cluster_) += force;
    }

    bool ClusterTreeNode::containsBody(int body_index) const
    {
        for (const auto &body : bodies_)
        {
            if (body.index_ == body_index)
                return true;
        }
        return false;
    }

    std::vector<std::pair<Body<>, JointPtr<double>>>
    ClusterTreeNode::bodiesAndJoints() const
    {
        std::vector<JointPtr<double>> single_joints = joint_->singleJoints();
        std::vector<std::pair<Body<>, JointPtr<double>>> bodies_and_joints;
        for (int i = 0; i < (int)bodies_.size(); i++)
        {
            Body<> body = bodies_[i];
            JointPtr<double> joint = single_joints[i]->clone();
            std::pair<Body<>, JointPtr<double>> body_and_joint = std::make_pair(body, joint);
            bodies_and_joints.push_back(body_and_joint);
        }
        return bodies_and_joints;
    }

    std::vector<std::tuple<Body<>, JointPtr<double>, DMat<double>>>
    ClusterTreeNode::bodiesJointsAndReflectedInertias() const
    {
        return joint_->bodiesJointsAndReflectedInertias();
    }

} // namespace grbda
