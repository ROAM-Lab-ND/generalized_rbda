#include "ClusterTreeNode.h"

namespace grbda
{

    ClusterTreeNode::ClusterTreeNode(int index, std::string name, std::vector<Body> &bodies,
                                     std::shared_ptr<GeneralizedJoints::Base> joint,
                                     int parent_index, int num_parent_bodies,
                                     int position_index, int velocity_index)
        : TreeNode(index, name, parent_index, 6 * bodies.size(), num_parent_bodies,
                   position_index, joint->numPositions(),
                   velocity_index, joint->numVelocities()),
          bodies_(bodies), joint_(joint)
    {
        for (size_t i = 0; i < bodies.size(); i++)
        {
            I_.block<6, 6>(6 * i, 6 * i) = bodies[i].inertia_.getMatrix();
            Xup_.appendSpatialTransformWithClusterAncestorSubIndex(
                SpatialTransform{}, bodies[i].cluster_ancestor_sub_index_within_cluster_);
            Xa_.appendSpatialTransform(SpatialTransform{});
        }
    }

    void ClusterTreeNode::updateKinematics()
    {
        joint_->updateKinematics(joint_state_);
        joint_->computeSpatialTransformFromParentToCurrentCluster(Xup_);
    }

    const SpatialTransform &ClusterTreeNode::getAbsoluteTransformForBody(const Body &body)
    {
        return Xa_.getTransformForOutputBody(body.sub_index_within_cluster_);
    }

    DVec<double> ClusterTreeNode::getVelocityForBody(const Body &body)
    {
        return v_.segment<6>(6 * body.sub_index_within_cluster_);
    }

    void ClusterTreeNode::applyForceToBody(const SVec<double> &force, const Body &body)
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

    std::vector<std::pair<Body, JointPtr>>
    ClusterTreeNode::bodiesAndJoints() const
    {
        std::vector<JointPtr> single_joints = joint_->singleJoints();
        std::vector<std::pair<Body, JointPtr>> bodies_and_joints;
        for (int i = 0; i < (int)bodies_.size(); i++)
        {
            std::pair<Body, JointPtr> body_and_joint = std::make_pair(bodies_[i], single_joints[i]);
            bodies_and_joints.push_back(body_and_joint);
        }
        return bodies_and_joints;
    }

    std::vector<std::tuple<Body, JointPtr, DMat<double>>>
    ClusterTreeNode::bodiesJointsAndReflectedInertias() const
    {
        return joint_->bodiesJointsAndReflectedInertias();
    }

} // namespace grbda
