#include "RigidBodyTreeModel.h"

namespace grbda
{

    template <typename Scalar>
    RigidBodyTreeModel<Scalar>::RigidBodyTreeModel(
        const ClusterTreeModel<Scalar> &cluster_tree_model, const FwdDynMethod fd_method)
    {
        forward_dynamics_method_ = fd_method;
        this->gravity_ = cluster_tree_model.getGravity();

        extractRigidBodiesAndJointsFromClusterModel(cluster_tree_model);
        extractLoopClosureFunctionsFromClusterModel(cluster_tree_model);
        extractContactPointsFromClusterModel(cluster_tree_model);
    }

    template <typename Scalar>
    void RigidBodyTreeModel<Scalar>::extractRigidBodiesAndJointsFromClusterModel(
        const ClusterTreeModel<Scalar> &cluster_tree_model)
    {
        int body_index = 0;
        for (const auto &cluster : cluster_tree_model.clusters())
        {
            for (const auto &body_and_joint : cluster->bodiesAndJoints())
            {
                const Body<Scalar> &body = body_and_joint.first;
                JointPtr<Scalar> joint = body_and_joint.second;
                auto node = std::make_shared<RigidBodyTreeNode<Scalar>>(
                    body, joint, this->position_index_, this->velocity_index_,
                    this->motion_subspace_index_);
                rigid_body_nodes_.push_back(node);
                this->nodes_.push_back(node);
                body_name_to_body_index_[body.name_] = body_index++;

                this->position_index_ += joint->numPositions();
                this->velocity_index_ += joint->numVelocities();
                this->motion_subspace_index_ += 6;
            }
        }
        this->H_ = DMat<Scalar>::Zero(this->velocity_index_, this->velocity_index_);
        this->C_ = DVec<Scalar>::Zero(this->velocity_index_);

        extractExpandedTreeConnectivity();
    }

    template <typename Scalar>
    void RigidBodyTreeModel<Scalar>::extractExpandedTreeConnectivity()
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

    template <typename Scalar>
    void RigidBodyTreeModel<Scalar>::extractLoopClosureFunctionsFromClusterModel(
        const ClusterTreeModel<Scalar> &cluster_tree_model)
    {

        for (const auto &cluster : cluster_tree_model.clusters())
        {
            loop_constraints_.push_back(cluster->joint_->cloneLoopConstraint());
        }
    }

    template <typename Scalar>
    void RigidBodyTreeModel<Scalar>::extractContactPointsFromClusterModel(
        const ClusterTreeModel<Scalar> &cluster_tree_model)
    {
        this->contact_name_to_contact_index_ = cluster_tree_model.contact_name_to_contact_index_;

        int contact_point_index = 0;
        for (const auto &contact_point : cluster_tree_model.contactPoints())
        {
            this->contact_points_.push_back(contact_point);
            if (contact_point.is_end_effector_)
            {
                this->num_end_effectors_++;

                ContactPoint<Scalar> &end_effector = this->contact_points_.back();
                end_effector.supporting_nodes_.clear();
                end_effector.ChiUp_.clear();

                // Keep track of which nodes support this new end effector
                int i = this->getNodeContainingBody(end_effector.body_index_)->index_;
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
                    end_effector.ChiUp_.push_back(DMat<Scalar>::Zero(0, 0));
                }

                // Get the nearest shared supporting node for every existing end effector
                for (int k = 0; k < (int)this->contact_points_.size() - 1; k++)
                {
                    if (!this->contact_points_[k].is_end_effector_)
                        continue;
                    std::pair<int, int> cp_pair(k, contact_point_index);
                    const int nearest_shared_support = this->getNearestSharedSupportingNode(cp_pair);
                    rigid_body_nodes_[nearest_shared_support]->nearest_supported_ee_pairs_.push_back(cp_pair);
                }
            }
            contact_point_index++;
        }
    }

    template <typename Scalar>
    void RigidBodyTreeModel<Scalar>::setState(const DVec<Scalar> &q, const DVec<Scalar> &qd)
    {
        for (auto &node : rigid_body_nodes_)
        {
            node->joint_state_.position = q.segment(node->position_index_, node->num_positions_);
            node->joint_state_.velocity = qd.segment(node->velocity_index_, node->num_velocities_);
        }

        q_ = q;
        qd_ = qd;

        this->setExternalForces();
    }

    template <typename Scalar>
    void RigidBodyTreeModel<Scalar>::updateLoopConstraints()
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

    template <typename Scalar>
    Vec3<Scalar> RigidBodyTreeModel<Scalar>::getPosition(const std::string &body_name)
    {
        const int &body_idx = body_name_to_body_index_.at(body_name);
        const TreeNodePtr<Scalar> rigid_body_node = this->getNodeContainingBody(body_idx);

        this->forwardKinematics();
        const spatial::Transform<Scalar> &Xa = rigid_body_node->Xa_[0];
        const Mat6<Scalar> Xai = spatial::invertSXform(Xa.toMatrix().template cast<Scalar>());
        Vec3<Scalar> link_pos = spatial::sXFormPoint(Xai, Vec3<Scalar>::Zero());
        return link_pos;
    }

    template <typename Scalar>
    Mat3<Scalar> RigidBodyTreeModel<Scalar>::getOrientation(const std::string &body_name)
    {
        const int &body_idx = body_name_to_body_index_.at(body_name);
        const TreeNodePtr<Scalar> rigid_body_node = this->getNodeContainingBody(body_idx);

        this->forwardKinematics();
        const spatial::Transform<Scalar> &Xa = rigid_body_node->Xa_[0];
        Mat3<Scalar> Rai = Xa.getRotation();
        Rai.transposeInPlace();
        return Rai;
    }

    template <typename Scalar>
    Vec3<Scalar> RigidBodyTreeModel<Scalar>::getLinearVelocity(const std::string &body_name)
    {
        const int &body_idx = body_name_to_body_index_.at(body_name);
        const TreeNodePtr<Scalar> rigid_body_node = this->getNodeContainingBody(body_idx);

        this->forwardKinematics();
        const Mat3<Scalar> Rai = getOrientation(body_name);
        const SVec<Scalar> v = rigid_body_node->v_.template head<6>();
        return Rai * spatial::spatialToLinearVelocity(v, Vec3<Scalar>::Zero());
    }

    template <typename Scalar>
    Vec3<Scalar> RigidBodyTreeModel<Scalar>::getAngularVelocity(const std::string &body_name)
    {
        const int &body_idx = body_name_to_body_index_.at(body_name);
        const TreeNodePtr<Scalar> rigid_body_node = this->getNodeContainingBody(body_idx);

        this->forwardKinematics();
        const Mat3<Scalar> Rai = getOrientation(body_name);
        const SVec<Scalar> v = rigid_body_node->v_.template head<6>();
        return Rai * v.template head<3>();
    }

    template <typename Scalar>
    DMat<Scalar> RigidBodyTreeModel<Scalar>::getMassMatrix()
    {
        updateLoopConstraints();
        this->compositeRigidBodyAlgorithm();
        return loop_constraints_.G_transpose() * this->H_ * loop_constraints_.G();
    }

    template <typename Scalar>
    DVec<Scalar> RigidBodyTreeModel<Scalar>::getBiasForceVector()
    {
        updateLoopConstraints();
        this->updateBiasForceVector();
        this->compositeRigidBodyAlgorithm();
        return loop_constraints_.G_transpose() * (this->C_ + this->H_ * loop_constraints_.g());
    }

    template class RigidBodyTreeModel<double>;
    template class RigidBodyTreeModel<float>;
    template class RigidBodyTreeModel<casadi::SX>;

} // namespace grbda
