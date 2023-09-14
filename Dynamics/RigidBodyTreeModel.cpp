#include "RigidBodyTreeModel.h"

namespace grbda
{

    RigidBodyTreeModel::RigidBodyTreeModel(const ClusterTreeModel &cluster_tree_model,
                                           const FwdDynMethod fd_method)
    {
        forward_dynamics_method_ = fd_method;
        gravity_ = cluster_tree_model.getGravity();

        extractRigidBodiesAndJointsFromClusterModel(cluster_tree_model);
        extractLoopClosureFunctionsFromClusterModel(cluster_tree_model);
        extractContactPointsFromClusterModel(cluster_tree_model);
    }

    void RigidBodyTreeModel::extractRigidBodiesAndJointsFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {
        int body_index = 0;
        for (const auto &cluster : cluster_tree_model.clusters())
        {
            for (const auto &body_and_joint : cluster->bodiesAndJoints())
            {
                const auto &body = body_and_joint.first;
                const auto &joint = body_and_joint.second;
                auto node = std::make_shared<RigidBodyTreeNode>(body, joint,
                                                                position_index_, velocity_index_,
                                                                motion_subspace_index_);
                rigid_body_nodes_.push_back(node);
                nodes_.push_back(node);
                body_name_to_body_index_[body.name_] = body_index++;

                position_index_ += joint->numPositions();
                velocity_index_ += joint->numVelocities();
                motion_subspace_index_ += 6;
            }
        }
        H_ = DMat<double>::Zero(velocity_index_, velocity_index_);
        C_ = DVec<double>::Zero(velocity_index_);

        extractExpandedTreeConnectivity();
    }

    void RigidBodyTreeModel::extractExpandedTreeConnectivity()
    {
        // Forward pass to initialize the expanded parent indices
        for (const auto &node : rigid_body_nodes_)
        {
            for (int i = 0; i < node->joint_->numVelocities(); i++)
            {
                expanded_tree_parent_indices_.push_back(node->parent_index_);
            }
        }

        // Backward pass to account for multi-dof joints
        for (int i = rigid_body_nodes_.size() - 1; i > -1; i--)
        {
            const auto &node = rigid_body_nodes_[i];

            int extra_dofs = 0;
            int j = node->parent_index_;
            while (j > -1)
            {
                const auto &parent_node = rigid_body_nodes_[j];
                extra_dofs += parent_node->joint_->numVelocities() - 1;
                j = parent_node->parent_index_;
            }

            for (int k = 0; k < node->joint_->numVelocities(); k++)
            {
                expanded_tree_parent_indices_.at(node->index_ + extra_dofs + k) = node->parent_index_ + extra_dofs + k;
            }
        }
    }

    void RigidBodyTreeModel::extractLoopClosureFunctionsFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {

        for (const auto &cluster : cluster_tree_model.clusters())
        {
            loop_constraints_.push_back(cluster->joint_->cloneLoopConstraint());
        }
    }

    void RigidBodyTreeModel::extractContactPointsFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {
        contact_name_to_contact_index_ = cluster_tree_model.contact_name_to_contact_index_;

        int contact_point_index = 0;
        for (const auto &contact_point : cluster_tree_model.contactPoints())
        {
            contact_points_.push_back(contact_point);
            if (contact_point.is_end_effector_)
            {
                num_end_effectors_++;

                ContactPoint &end_effector = contact_points_.back();
                end_effector.supporting_nodes_.clear();
                end_effector.ChiUp_.clear();

                // Keep track of which nodes support this new end effector
                int i = getNodeContainingBody(end_effector.body_index_)->index_;
                while (i > -1)
                {
                    auto &node = rigid_body_nodes_[i];
                    node->supported_end_effectors_.push_back(contact_point_index);
                    end_effector.supporting_nodes_.push_back(i);
                    i = node->parent_index_;
                }

                // Initialize the force propagators for this end effector
                for (int j = 0; j < (int)rigid_body_nodes_.size(); j++)
                {
                    end_effector.ChiUp_.push_back(DMat<double>::Zero(0, 0));
                }

                // Get the nearest shared supporting node for every existing end effector
                for (int k = 0; k < (int)contact_points_.size() - 1; k++)
                {
                    if (!contact_points_[k].is_end_effector_)
                        continue;
                    std::pair<int, int> cp_pair(k, contact_point_index);
                    const int nearest_shared_support = getNearestSharedSupportingNode(cp_pair);
                    rigid_body_nodes_[nearest_shared_support]->nearest_supported_ee_pairs_.push_back(cp_pair);
                }
            }
            contact_point_index++;
        }
    }

    void RigidBodyTreeModel::initializeState(const DVec<double> &q, const DVec<double> &qd)
    {
        for (auto &node : rigid_body_nodes_)
        {
            node->joint_state_.position = q.segment(node->position_index_, node->num_positions_);
            node->joint_state_.velocity = qd.segment(node->velocity_index_, node->num_velocities_);
        }

        q_ = q;
        qd_ = qd;

        initializeExternalForces();
    }

    void RigidBodyTreeModel::updateLoopConstraints()
    {
#ifdef DEBUG_MODE
        if (q_.size() != getNumPositions() || qd_.size() != getNumDegreesOfFreedom())
            throw std::runtime_error("State is not initialized");
#endif
        if (loop_constraints_updated_)
            return;

        loop_constraints_.update(q_, qd_);
        loop_constraints_updated_ = true;
    }

    Vec3<double> RigidBodyTreeModel::getPosition(const string &body_name)
    {
        // TODO(@MatthewChignoli): Helper function that gets node given the name?
        const int &body_idx = body_name_to_body_index_.at(body_name);
        const TreeNodePtr rigid_body_node = getNodeContainingBody(body_idx);

        forwardKinematics();
        const SpatialTransform &Xa = rigid_body_node->Xa_[0];
        const Mat6<double> Xai = invertSXform(Xa.toMatrix().cast<double>());
        Vec3<double> link_pos = sXFormPoint(Xai, Vec3<double>::Zero());
        return link_pos;
    }

    Mat3<double> RigidBodyTreeModel::getOrientation(const string &body_name)
    {
        const int &body_idx = body_name_to_body_index_.at(body_name);
        const TreeNodePtr rigid_body_node = getNodeContainingBody(body_idx);

        forwardKinematics();
        const SpatialTransform &Xa = rigid_body_node->Xa_[0];
        Mat3<double> Rai = Xa.getRotation();
        Rai.transposeInPlace();
        return Rai;
    }

    Vec3<double> RigidBodyTreeModel::getLinearVelocity(const string &body_name)
    {
        const int &body_idx = body_name_to_body_index_.at(body_name);
        const TreeNodePtr rigid_body_node = getNodeContainingBody(body_idx);

        forwardKinematics();
        const Mat3<double> Rai = getOrientation(body_name);
        const SVec<double> v = rigid_body_node->v_.head<6>();
        return Rai * spatialToLinearVelocity(v, Vec3<double>::Zero());
    }

    Vec3<double> RigidBodyTreeModel::getAngularVelocity(const string &body_name)
    {
        const int &body_idx = body_name_to_body_index_.at(body_name);
        const TreeNodePtr rigid_body_node = getNodeContainingBody(body_idx);

        forwardKinematics();
        const Mat3<double> Rai = getOrientation(body_name);
        const SVec<double> v = rigid_body_node->v_.head<6>();
        return Rai * v.head<3>();
    }

    DMat<double> RigidBodyTreeModel::getMassMatrix()
    {
        updateLoopConstraints();
        compositeRigidBodyAlgorithm();
        return loop_constraints_.G_transpose() * H_ * loop_constraints_.G();
    }

    DVec<double> RigidBodyTreeModel::getBiasForceVector()
    {
        updateLoopConstraints();
        updateBiasForceVector();
        if (loop_constraints_.g().norm() > 1e-12)
        {
            compositeRigidBodyAlgorithm();
        }
        return loop_constraints_.G_transpose() * (C_ + H_ * loop_constraints_.g());
    }

} // namespace grbda
