#include "grbda/Dynamics/TreeModel.h"

namespace grbda
{

    template <typename Scalar>
    void TreeModel<Scalar>::forwardKinematics()
    {
        if (kinematics_updated_)
            return;

        for (auto &node : nodes_)
        {
            node->updateKinematics();

            if (node->parent_index_ >= 0)
            {
                const auto parent_node = nodes_[node->parent_index_];
                node->v_ = node->Xup_.transformMotionVector(parent_node->v_) + node->vJ();
                node->Xa_ = node->Xup_ * parent_node->Xa_;
            }
            else
            {
                node->v_ = node->vJ();
                node->Xa_ = node->Xup_.toAbsolute();
            }

            node->avp_ = spatial::generalMotionCrossProduct(node->v_, node->vJ());
        }

        kinematics_updated_ = true;
    }

    template <typename Scalar>
    void TreeModel<Scalar>::contactPointForwardKinematics()
    {
        if (contact_point_kinematics_updated_)
            return;

        for (auto &cp : contact_points_)
        {
            const auto &body = getBody(cp.body_index_);
            const auto node = this->getNodeContainingBody(cp.body_index_);

            const auto &Xa = node->getAbsoluteTransformForBody(body);
            const SVec<Scalar> v_body = node->getVelocityForBody(body);

            cp.position_ = Xa.inverseTransformPoint(cp.local_offset_);
            cp.velocity_ = spatial::spatialToLinearVelocity(Xa.inverseTransformMotionVector(v_body),
                                                            cp.position_);
        }

        contact_point_kinematics_updated_ = true;
    }

    template <typename Scalar>
    void TreeModel<Scalar>::updateContactPointJacobians()
    {
        if (contact_jacobians_updated_)
            return;

        for (auto &contact_point : contact_points_)
        {
            contactJacobianWorldFrame(contact_point.name_);
        }
        contact_jacobians_updated_ = true;
    }

    template <typename Scalar>
    void TreeModel<Scalar>::compositeRigidBodyAlgorithm()
    {
        if (mass_matrix_updated_)
            return;

        forwardKinematics();

        // Forward Pass
        for (auto &node : nodes_)
            node->Ic_ = node->I_;

        // Backward Pass
        for (int i = (int)nodes_.size() - 1; i >= 0; i--)
        {
            auto &node_i = nodes_[i];
            const int vel_idx_i = node_i->velocity_index_;
            const int num_vel_i = node_i->num_velocities_;

            if (node_i->parent_index_ >= 0)
            {
                auto parent_node = nodes_[node_i->parent_index_];
                parent_node->Ic_ += node_i->Xup_.inverseTransformSpatialInertia(node_i->Ic_);
            }

            DMat<Scalar> F = node_i->Ic_ * node_i->S();
            H_.block(vel_idx_i, vel_idx_i, num_vel_i, num_vel_i) = node_i->S().transpose() * F;

            int j = i;
            while (nodes_[j]->parent_index_ > -1)
            {
                F = nodes_[j]->Xup_.inverseTransformForceSubspace(F);

                j = nodes_[j]->parent_index_;
                const int vel_idx_j = nodes_[j]->velocity_index_;
                const int num_vel_j = nodes_[j]->num_velocities_;

                H_.block(vel_idx_i, vel_idx_j, num_vel_i, num_vel_j) =
                    F.transpose() * nodes_[j]->S();
                H_.block(vel_idx_j, vel_idx_i, num_vel_j, num_vel_i) =
                    H_.block(vel_idx_i, vel_idx_j, num_vel_i, num_vel_j).transpose();
            }
        }

        mass_matrix_updated_ = true;
    }

    template <typename Scalar>
    void TreeModel<Scalar>::updateBiasForceVector()
    {
        if (bias_force_updated_)
            return;

        C_ = recursiveNewtonEulerAlgorithm(DVec<Scalar>::Zero(getNumDegreesOfFreedom()));

        bias_force_updated_ = true;
    }

    template <typename Scalar>
    DVec<Scalar> TreeModel<Scalar>::recursiveNewtonEulerAlgorithm(const DVec<Scalar> &qdd)
    {
        forwardKinematics();

        DVec<Scalar> tau = DVec<Scalar>::Zero(qdd.rows());

        // Forward Pass
        for (auto &node : nodes_)
        {
            const int vel_idx = node->velocity_index_;
            const int num_vel = node->num_velocities_;

            if (node->parent_index_ >= 0)
            {
                auto parent_node = nodes_[node->parent_index_];
                node->a_ = node->Xup_.transformMotionVector(parent_node->a_) +
                           node->S() * qdd.segment(vel_idx, num_vel) +
                           node->cJ() + node->avp_;
            }
            else
            {
                node->a_ = node->Xup_.transformMotionVector(-gravity_) +
                           node->S() * qdd.segment(vel_idx, num_vel) +
                           node->cJ() + node->avp_;
            }

            node->f_ = node->I_ * node->a_ +
                       spatial::generalForceCrossProduct(node->v_,
                                                         DVec<Scalar>(node->I_ * node->v_));
        }

        // Account for external forces in bias force
        for (int index : indices_of_nodes_experiencing_external_forces_)
        {
            auto node = nodes_[index];
            node->f_ -= node->Xa_.transformExternalForceVector(node->f_ext_);
        }

        // Backward Pass
        for (int i = (int)nodes_.size() - 1; i >= 0; i--)
        {
            auto &node = nodes_[i];
            const int vel_idx = node->velocity_index_;
            const int num_vel = node->num_velocities_;

            tau.segment(vel_idx, num_vel) = node->S().transpose() * node->f_;

            if (node->parent_index_ >= 0)
            {
                auto &parent_node = nodes_[node->parent_index_];
                parent_node->f_ += node->Xup_.inverseTransformForceVector(node->f_);
            }
        }

        return tau;
    }

    template <typename Scalar>
    void TreeModel<Scalar>::setExternalForces(
        const std::vector<ExternalForceAndBodyIndexPair<Scalar>> &force_and_body_index_pairs)
    {
        // Clear previous external forces
        for (const int index : indices_of_nodes_experiencing_external_forces_)
            nodes_[index]->f_ext_.setZero();

        // Apply forces to nodes
        indices_of_nodes_experiencing_external_forces_.clear();
        for (const auto &force_and_body_index : force_and_body_index_pairs)
        {
            const auto &force = force_and_body_index.force_;
            const int body_index = force_and_body_index.index_;

            const auto &body = getBody(body_index);
            const auto node = this->getNodeContainingBody(body_index);
            node->applyForceToBody(force, body);

            // Add index to vector if vector does not already contain this cluster
            if (!vectorContainsIndex(indices_of_nodes_experiencing_external_forces_, node->index_))
                indices_of_nodes_experiencing_external_forces_.push_back(node->index_);
        }

        resetCache();
    }

    template <typename Scalar>
    void TreeModel<Scalar>::resetCache()
    {
        kinematics_updated_ = false;
        contact_point_kinematics_updated_ = false;
        mass_matrix_updated_ = false;
        bias_force_updated_ = false;
        contact_jacobians_updated_ = false;
    }

    template <typename Scalar>
    int TreeModel<Scalar>::getNearestSharedSupportingNode(const std::pair<int, int> &cp_indices)
    {
        const ContactPoint<Scalar> &cp_i = contact_points_[cp_indices.first];
        const ContactPoint<Scalar> &cp_j = contact_points_[cp_indices.second];
        return greatestCommonElement(cp_i.supporting_nodes_, cp_j.supporting_nodes_);
    }

    template <typename Scalar>
    bool TreeModel<Scalar>::vectorContainsIndex(const std::vector<int> vec, const int index)
    {
        return std::find(vec.begin(), vec.end(), index) != vec.end();
    }

    template class TreeModel<double>;
    template class TreeModel<float>;
    template class TreeModel<casadi::SX>;

} // namespace grbda
