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
    void TreeModel<Scalar>::forwardAccelerationKinematics(const DVec<Scalar> &qdd)
    {
        forwardKinematics();
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
        }
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
    void TreeModel<Scalar>::contactPointForwardAccelerationKinematics(const DVec<Scalar> &qdd)
    {
        contactPointForwardKinematics();

        for (auto &cp : contact_points_)
        {
            const auto &body = getBody(cp.body_index_);
            const auto node = this->getNodeContainingBody(cp.body_index_);

            const auto &Xa = node->getAbsoluteTransformForBody(body);
            const SVec<Scalar> v_body = node->getVelocityForBody(body);
            const SVec<Scalar> v_body_in_world = Xa.inverseTransformMotionVector(v_body);
            const SVec<Scalar> a_body = node->getAccelerationForBody(body);
            const SVec<Scalar> a_body_in_world = Xa.inverseTransformMotionVector(a_body) + gravity_;
            cp.acceleration_ = spatial::spatialToLinearAcceleration(a_body_in_world,
                                                                    v_body_in_world,
                                                                    cp.position_);
        }
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

        // Forward Pass: Initialize composite inertias to local inertias
        for (auto &node : nodes_)
            node->Ic_ = node->I_;

        // Backward Pass
        for (int i = (int)nodes_.size() - 1; i >= 0; i--)
        {
            auto &node_i = nodes_[i];
            const int vel_idx_i = node_i->velocity_index_;
            const int num_vel_i = node_i->num_velocities_;

            // Accumulate composite inertia to parent using block-diagonal structure
            // For cluster B connected to cluster A at body k, we add:
            // I_A[k,k] += sum over all bodies j in B of: X_j^{-T} * I_B[j,j] * X_j^{-1}
            // where X_j is the transform from body j in B to body k in A
            if (node_i->parent_index_ >= 0)
            {
                auto parent_node = nodes_[node_i->parent_index_];
                node_i->Xup_.accumulateBlockDiagonalInertia(node_i->Ic_, parent_node->Ic_);
            }

            // Diagonal block: H_ii = S_i^T * Ic_i * S_i
            // Compute F = Ic * S exploiting block-diagonal structure of Ic
            DMat<Scalar> F = node_i->Xup_.blockDiagonalInertiaTimesMotionSubspace(
                node_i->Ic_, node_i->S());
            H_.block(vel_idx_i, vel_idx_i, num_vel_i, num_vel_i) = node_i->S().transpose() * F;

            // Off-diagonal blocks: H_ij = S_j^T * X_ij^{-T} * Ic_i * S_i
            // F is transformed through the chain from node i to ancestor j
            int j = i;
            while (nodes_[j]->parent_index_ > -1)
            {
                // Transform F from current frame to parent frame using block-wise transform
                F = nodes_[j]->Xup_.transformForceSubspaceToParent(F);

                j = nodes_[j]->parent_index_;
                const int vel_idx_j = nodes_[j]->velocity_index_;
                const int num_vel_j = nodes_[j]->num_velocities_;

                // H_ij = F^T * S_j
                H_.block(vel_idx_i, vel_idx_j, num_vel_i, num_vel_j) =
                    F.transpose() * nodes_[j]->S();
                H_.block(vel_idx_j, vel_idx_i, num_vel_j, num_vel_i) =
                    H_.block(vel_idx_i, vel_idx_j, num_vel_i, num_vel_j).transpose();
            }
        }

        mass_matrix_updated_ = true;
    }

    template <typename Scalar>
    void TreeModel<Scalar>::compositeRigidBodyAlgorithmWorldFrame()
    {
        if (mass_matrix_updated_)
            return;

        forwardKinematics();

        const int n = (int)nodes_.size();

        // Storage for world-frame quantities
        // Ic_world[i] stores the composite inertia for node i in world frame
        std::vector<DMat<Scalar>> Ic_world(n);
        // S_world[i] stores the motion subspace for node i in world frame
        std::vector<DMat<Scalar>> S_world(n);

        // Forward Pass: Transform inertias and motion subspaces to world frame
        for (int i = 0; i < n; i++)
        {
            auto &node = nodes_[i];
            // Transform local inertia to world frame
            Ic_world[i] = node->Xa_.transformBlockDiagonalInertiaToWorld(node->I_);
            // Transform motion subspace to world frame
            S_world[i] = node->Xa_.transformMotionSubspaceToWorld(node->S());
        }

        int N_total = getNumDegreesOfFreedom();
        DMat<Scalar> F_tmp = DMat<Scalar>::Zero(6, N_total);

        for( auto & node : nodes_)
        {
            node->num_subtree_velocities_ = 0;
        }

        // Backward Pass: Accumulate composite inertias and compute H
        for (int i = n - 1; i >= 0; i--)
        {
            auto &node_i = nodes_[i];

            const int vel_idx_i = node_i->velocity_index_;
            const int num_vel_i = node_i->num_velocities_;

            // Accumulate composite inertia to parent - direct addition in world frame!
            if (node_i->parent_index_ >= 0)
            {
                for(int j = 0 ; j < node_i->Xup_.getNumOutputBodies(); j++)
                {
                    const int output_body = j;
                    const int parent_subindex = node_i->Xup_.transform_and_parent_subindex(j).second;
                    Ic_world[node_i->parent_index_].template block<6, 6>(6 * parent_subindex, 6 * parent_subindex).noalias() +=
                        Ic_world[i].template block<6, 6>(6 * output_body, 6 * output_body);
                }
            }

            // Compute F = Ic_world * S_world (both in world frame, compatible!)
            // For block-diagonal Ic, we can exploit the structure
            const int num_bodies = node_i->Xa_.getNumOutputBodies();
            DMat<Scalar> F = DMat<Scalar>::Zero(6 * num_bodies, num_vel_i);
            for (int body = 0; body < num_bodies; body++)
            {
                F.template middleRows<6>(6 * body).noalias() =
                    Ic_world[i].template block<6, 6>(6 * body, 6 * body) *
                    S_world[i].template middleRows<6>(6 * body);
            }

            // Diagonal block: H_ii = S_world^T * F
            H_.block(vel_idx_i, vel_idx_i, num_vel_i, num_vel_i) =
                S_world[i].transpose() * F;
            
            int idx = 0;
            F_tmp.middleCols(vel_idx_i, num_vel_i).setZero();
            while(idx < F.rows())
            {
                F_tmp.middleCols(vel_idx_i, num_vel_i) += F.template middleRows<6>(idx);
                idx += 6;
            }
            int j = i;
            // Off-diagonal blocks: H_ij = S_world_j^T * Ic_world
            while (nodes_[j]->parent_index_ > -1)
            {
                j = nodes_[j]->parent_index_;
                nodes_[j]->num_subtree_velocities_ += num_vel_i;
            }
        }
        
        for (int i = n - 1; i >= 0; i--)
        {
            auto &node_i = nodes_[i];

            const int vel_idx_i = node_i->velocity_index_;
            const int num_vel_i = node_i->num_velocities_;

            const auto F_subtree = F_tmp.middleCols(vel_idx_i+num_vel_i, node_i->num_subtree_velocities_);
            H_.block(vel_idx_i, vel_idx_i+num_vel_i, num_vel_i, node_i->num_subtree_velocities_) = S_world[i].transpose() * F_subtree;
        }

        H_.template triangularView<Eigen::StrictlyLower>() =
            H_.template triangularView<Eigen::StrictlyUpper>().transpose();

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
        forwardAccelerationKinematics(qdd);

        DVec<Scalar> tau = DVec<Scalar>::Zero(qdd.rows());

        // Forward Pass
        for (auto &node : nodes_)
        {
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
    template class TreeModel<std::complex<double>>;
    template class TreeModel<float>;
    template class TreeModel<casadi::SX>;

} // namespace grbda
