#include "ReflectedInertiaTreeModel.h"

namespace grbda
{

    ReflectedInertiaTreeModel::ReflectedInertiaTreeModel(
        const ClusterTreeModel &cluster_tree_model,
        RotorInertiaApproximation rotor_inertia_approximation)
        : TreeModel(), rotor_inertia_approximation_(rotor_inertia_approximation)
    {
        gravity_ = cluster_tree_model.getGravity();

        extractRigidBodiesAndJointsFromClusterModel(cluster_tree_model);
        extractIndependentCoordinatesFromClusterModel(cluster_tree_model);
        extractContactPointsFromClusterModel(cluster_tree_model);
    }

    void ReflectedInertiaTreeModel::extractRigidBodiesAndJointsFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {
        for (const auto &cluster : cluster_tree_model.clusters())
        {
            const int &nv = cluster->num_velocities_;
            DMat<double> cluster_reflected_inertia = DMat<double>::Zero(nv, nv);
            for (const auto &link_joint_and_reflected_inertia :
                 cluster->bodiesJointsAndReflectedInertias())
            {
                const auto &link = std::get<0>(link_joint_and_reflected_inertia);
                const auto link_joint = std::get<1>(link_joint_and_reflected_inertia);
                const auto &reflected_inertia = std::get<2>(link_joint_and_reflected_inertia);

                const int node_index = (int)reflected_inertia_nodes_.size();
                const int parent_node_index = getIndexOfParentNodeForBody(link.parent_index_);
                auto node = std::make_shared<ReflectedInertiaTreeNode>(node_index, link, link_joint,
                                                                       parent_node_index,
                                                                       position_index_,
                                                                       velocity_index_,
                                                                       motion_subspace_index_);
                reflected_inertia_nodes_.push_back(node);
                nodes_.push_back(node);

                cluster_reflected_inertia += reflected_inertia;

                position_index_ += link_joint->numPositions();
                velocity_index_ += link_joint->numVelocities();
                motion_subspace_index_ += 6;
            }
            reflected_inertia_ = appendEigenMatrix(reflected_inertia_, cluster_reflected_inertia);
        }

        H_ = DMat<double>::Zero(velocity_index_, velocity_index_);
        C_ = DVec<double>::Zero(velocity_index_);

        // Resize system matrices
        for (auto &node : reflected_inertia_nodes_)
            node->qdd_for_subtree_due_to_subtree_root_joint_qdd
                .setZero(getNumDegreesOfFreedom(), node->num_velocities_);
    }

    void ReflectedInertiaTreeModel::extractIndependentCoordinatesFromClusterModel(
        const ClusterTreeModel &cluster_tree_model)
    {
        spanning_tree_to_independent_coords_conversion_ = DMat<double>::Zero(0, 0);
        for (const auto &cluster : cluster_tree_model.clusters())
        {
            const auto joint = cluster->joint_;
            spanning_tree_to_independent_coords_conversion_ =
                appendEigenMatrix(spanning_tree_to_independent_coords_conversion_,
                                  joint->spanningTreeToIndependentCoordsConversion());
        }

        const int num_bodies = cluster_tree_model.getNumBodies();
        independent_coord_indices_ =
            spanning_tree_to_independent_coords_conversion_.template cast<int>() *
            DVec<int>::LinSpaced(num_bodies, 0, num_bodies - 1);
    }

    void ReflectedInertiaTreeModel::extractContactPointsFromClusterModel(
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
                    auto &node = reflected_inertia_nodes_[i];
                    node->supported_end_effectors_.push_back(contact_point_index);
                    end_effector.supporting_nodes_.push_back(i);
                    i = node->parent_index_;
                }

                // Initialize the force propagators for this end effector
                for (int j = 0; j < (int)reflected_inertia_nodes_.size(); j++)
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
                    reflected_inertia_nodes_[nearest_shared_support]->nearest_supported_ee_pairs_.push_back(cp_pair);
                }
            }
            contact_point_index++;
        }
    }

    void ReflectedInertiaTreeModel::setIndependentStates(const DVec<double> &y,
                                                                const DVec<double> &yd)
    {
        for (auto &node : reflected_inertia_nodes_)
        {
            node->joint_state_.position = y.segment(node->position_index_, node->num_positions_);
            node->joint_state_.velocity = yd.segment(node->velocity_index_, node->num_velocities_);
        }

        setExternalForces();
    }

    void ReflectedInertiaTreeModel::resetCache()
    {
        TreeModel::resetCache();
        articulated_bodies_updated_ = false;
        force_propagators_updated_ = false;
        qdd_effects_updated_ = false;
    }

    DMat<double> ReflectedInertiaTreeModel::getMassMatrix()
    {
        compositeRigidBodyAlgorithm();

        switch (rotor_inertia_approximation_)
        {
        case RotorInertiaApproximation::NONE:
            return H_;
        case RotorInertiaApproximation::DIAGONAL:
            return H_ + DMat<double>(reflected_inertia_.diagonal().asDiagonal());
        case RotorInertiaApproximation::BLOCK_DIAGONAL:
            return H_ + reflected_inertia_;
        default:
            throw std::runtime_error("Invalid rotor inertia approximation");
        }
    }

    DVec<double> ReflectedInertiaTreeModel::getBiasForceVector()
    {
        updateBiasForceVector();
        return C_;
    }

    // NOTE: The following relationship is true for the rigid body tree model, but not the reflected
    // inertia model: body.index_ = nodes[body.index_].index_
    const Body &ReflectedInertiaTreeModel::getBody(int spanning_tree_index) const
    {
        for (const auto &link_node : reflected_inertia_nodes_)
            if (link_node->link_.index_ == spanning_tree_index)
                return link_node->link_;
        throw std::runtime_error("That body does not exist in the link and rotor tree model");
    }

    const TreeNodePtr ReflectedInertiaTreeModel::getNodeContainingBody(int spanning_tree_index)
    {
        for (const auto &link_node : reflected_inertia_nodes_)
            if (link_node->link_.index_ == spanning_tree_index)
                return link_node;
        throw std::runtime_error("That body does not exist in the link and rotor tree model");
    }

    int ReflectedInertiaTreeModel::getIndexOfParentNodeForBody(const int spanning_tree_index)
    {
        if (spanning_tree_index == -1)
            return -1;
        else
            return getNodeContainingBody(spanning_tree_index)->index_;
    }

    const D6Mat<double> &
    ReflectedInertiaTreeModel::contactJacobianWorldFrame(const std::string &cp_name)
    {
        forwardKinematics();

        ContactPoint &cp = contact_points_[contact_name_to_contact_index_.at(cp_name)];

        const size_t &i = cp.body_index_;
        const auto node_i = getNodeContainingBody(i);
        const spatial::Transform Xa = node_i->Xa_[0];
        const Mat3<double> R_link_to_world = Xa.getRotation().transpose();
        Mat6<double> Xout = spatial::createSXform(R_link_to_world, cp.local_offset_);

        int j = node_i->index_;
        while (j > -1)
        {
            const auto node_j = reflected_inertia_nodes_[j];
            const int &vel_idx = node_j->velocity_index_;
            const int &num_vel = node_j->num_velocities_;

            const D6Mat<double> &S = node_j->S();
            cp.jacobian_.middleCols(vel_idx, num_vel) = Xout * S;

            const Mat6<double> Xup = node_j->Xup_[0].toMatrix();
            Xout = Xout * Xup;

            j = node_j->parent_index_;
        }

        return cp.jacobian_;
    }

    D6Mat<double> ReflectedInertiaTreeModel::contactJacobianBodyFrame(const std::string &cp_name)
    {
        forwardKinematics();

        D6Mat<double> J = D6Mat<double>::Zero(6, getNumDegreesOfFreedom());

        ContactPoint &cp = contact_points_[contact_name_to_contact_index_.at(cp_name)];
        const size_t &i = cp.body_index_;
        const auto node_i = getNodeContainingBody(i);
        Mat6<double> Xout = spatial::createSXform(Mat3<double>::Identity(), cp.local_offset_);

        int j = node_i->index_;
        while (j > -1)
        {
            const auto node_j = reflected_inertia_nodes_[j];
            const int &vel_idx = node_j->velocity_index_;
            const int &num_vel = node_j->num_velocities_;

            const D6Mat<double> &S = node_j->S();
            J.middleCols(vel_idx, num_vel) = Xout * S;

            const Mat6<double> Xup = node_j->Xup_[0].toMatrix();
            Xout = Xout * Xup;

            j = node_j->parent_index_;
        }

        return J;
    }

    DVec<double> ReflectedInertiaTreeModel::forwardDynamics(const DVec<double> &tau)
    {
        switch (rotor_inertia_approximation_)
        {
        case RotorInertiaApproximation::NONE:
            return forwardDynamicsABA(tau, false);
        case RotorInertiaApproximation::DIAGONAL:
            return forwardDynamicsABA(tau, true);
        case RotorInertiaApproximation::BLOCK_DIAGONAL:
            return forwardDynamicsHinv(tau);
        default:
            throw std::runtime_error("Invalid rotor inertia approximation");
        }
    }

    DVec<double> ReflectedInertiaTreeModel::inverseDynamics(const DVec<double> &ydd)
    {
        switch (rotor_inertia_approximation_)
        {
        case RotorInertiaApproximation::NONE:
            return recursiveNewtonEulerAlgorithm(ydd);
        case RotorInertiaApproximation::DIAGONAL:
            return recursiveNewtonEulerAlgorithm(ydd) +
                   reflected_inertia_.diagonal().asDiagonal() * ydd;
        case RotorInertiaApproximation::BLOCK_DIAGONAL:
            return recursiveNewtonEulerAlgorithm(ydd) + reflected_inertia_ * ydd;
        default:
            throw std::runtime_error("Invalid rotor inertia approximation");
        }
    }

    DMat<double> ReflectedInertiaTreeModel::inverseOperationalSpaceInertiaMatrix()
    {
        switch (rotor_inertia_approximation_)
        {
        case RotorInertiaApproximation::NONE:
            return inverseOpSpaceInertiaEFPA(false);
        case RotorInertiaApproximation::DIAGONAL:
            return inverseOpSpaceInertiaEFPA(true);
        case RotorInertiaApproximation::BLOCK_DIAGONAL:
            return inverseOpSpaceInertiaHinv();
        default:
            throw std::runtime_error("Invalid rotor inertia approximation");
        }
    }

    DVec<double> ReflectedInertiaTreeModel::forwardDynamicsHinv(const DVec<double> &tau)
    {
        compositeRigidBodyAlgorithm();
        updateBiasForceVector();
        DVec<double> qdd_ref_inertia = (H_ + reflected_inertia_).inverse() * (tau - C_);
        return qdd_ref_inertia;
    }

    DVec<double> ReflectedInertiaTreeModel::forwardDynamicsABA(const DVec<double> &tau,
                                                               bool use_reflected_inertia)
    {
        // Forward dynamics via Articulated Body Algorithm
        forwardKinematics();
        updateArticulatedBodies(use_reflected_inertia);

        DVec<double> qdd = DVec<double>::Zero(getNumDegreesOfFreedom());

        // Forward Pass - Articulated body bias force
        for (auto &link_node : reflected_inertia_nodes_)
        {
            link_node->pA_ = spatial::generalForceCrossProduct(link_node->v_, DVec<double>(link_node->I_ * link_node->v_));
        }

        // Account for external forces in bias force
        for (int link_node_index : indices_of_nodes_experiencing_external_forces_)
        {
            auto &link_node = reflected_inertia_nodes_[link_node_index];
            link_node->pA_ -= link_node->Xa_.transformExternalForceVector(link_node->f_ext_);
        }

        // Backward pass - Gauss principal of least constraint
        for (int i = (int)reflected_inertia_nodes_.size() - 1; i >= 0; i--)
        {
            auto &link_node = reflected_inertia_nodes_[i];
            const int vel_idx = link_node->velocity_index_;
            const int num_vel = link_node->num_velocities_;
            const auto joint = link_node->joint_;

            link_node->u_ = tau.segment(vel_idx, num_vel) - joint->S().transpose() * link_node->pA_;

            // Articulated body bias force recursion
            if (link_node->parent_index_ >= 0)
            {
                auto parent_link_node = reflected_inertia_nodes_[link_node->parent_index_];

                Mat6<double> Ia =
                    link_node->IA_ - link_node->U_ * link_node->D_inv_ * link_node->U_.transpose();

                SVec<double> pa =
                    link_node->pA_ + Ia * (link_node->cJ() + link_node->avp_) +
                    link_node->U_ * link_node->D_inv_ * link_node->u_;

                parent_link_node->pA_ += link_node->Xup_.inverseTransformForceVector(pa);
            }
        }

        // Forward Pass - Joint accelerations
        for (auto &link_node : reflected_inertia_nodes_)
        {
            const int vel_idx = link_node->velocity_index_;
            const int num_vel = link_node->num_velocities_;
            const auto joint = link_node->joint_;

            SVec<double> a_temp;
            if (link_node->parent_index_ >= 0)
            {
                const auto parent_link_node = reflected_inertia_nodes_[link_node->parent_index_];
                a_temp = link_node->Xup_.transformMotionVector(parent_link_node->a_) +
                         link_node->cJ() + link_node->avp_;
            }
            else
            {
                a_temp = link_node->Xup_.transformMotionVector(-gravity_) +
                         link_node->cJ() + link_node->avp_;
            }
            qdd.segment(vel_idx, num_vel) =
                link_node->D_inv_ * (link_node->u_ - link_node->U_.transpose() * a_temp);
            link_node->a_ = a_temp + joint->S() * qdd.segment(vel_idx, num_vel);
        }

        return qdd;
    }

    void ReflectedInertiaTreeModel::updateArticulatedBodies(bool use_reflected_inertia)
    {
        if (articulated_bodies_updated_)
            return;

        forwardKinematics();

        // Forward pass
        for (auto &link_node : reflected_inertia_nodes_)
        {
            link_node->IA_ = link_node->I_;
        }

        // Backward pass (Gauss principal of least constraint)
        for (int i = (int)reflected_inertia_nodes_.size() - 1; i >= 0; i--)
        {
            auto &link_node = reflected_inertia_nodes_[i];
            const auto joint = link_node->joint_;

            const int vel_idx = link_node->velocity_index_;
            const int num_vel = link_node->num_velocities_;

            link_node->U_ = link_node->IA_ * joint->S();
            DMat<double> D = joint->S().transpose() * link_node->U_;
            if (use_reflected_inertia)
                D += reflected_inertia_.block(vel_idx, vel_idx, num_vel, num_vel);
            link_node->D_inv_ = D.inverse();

            // Articulated body inertia recursion
            if (link_node->parent_index_ >= 0)
            {
                auto parent_link_node = reflected_inertia_nodes_[link_node->parent_index_];
                DMat<double> Ia =
                    link_node->IA_ - link_node->U_ * link_node->D_inv_ * link_node->U_.transpose();
                parent_link_node->IA_ += link_node->Xup_.inverseTransformSpatialInertia(Ia);
            }
        }

        articulated_bodies_updated_ = true;
    }

    double ReflectedInertiaTreeModel::applyTestForce(
        const std::string &contact_point_name, const Vec3<double> &force, DVec<double> &dstate_out)
    {
        switch (rotor_inertia_approximation_)
        {
        case RotorInertiaApproximation::NONE:
            return applyTestForceEFPA(force, contact_point_name, dstate_out, false);
        case RotorInertiaApproximation::DIAGONAL:
            return applyTestForceEFPA(force, contact_point_name, dstate_out, true);
        case RotorInertiaApproximation::BLOCK_DIAGONAL:
            return applyTestForceHinv(force, contact_point_name, dstate_out);
        default:
            throw std::runtime_error("Unknown rotor inertia approximation");
        }
    }

    double ReflectedInertiaTreeModel::applyTestForceEFPA(const Vec3<double> &force,
                                                         const std::string &cp_name,
                                                         DVec<double> &dstate_out,
                                                         bool use_reflected_inertia)
    {
        const int contact_point_index = contact_name_to_contact_index_.at(cp_name);
        const ContactPoint &contact_point = contact_points_[contact_point_index];

        forwardKinematics();
        updateArticulatedBodies(use_reflected_inertia);
        updateForcePropagators(use_reflected_inertia);
        updateQddEffects(use_reflected_inertia);

        dstate_out = DVec<double>::Zero(getNumDegreesOfFreedom());

        SVec<double> f = localCartesianForceAtPointToWorldPluckerForceOnCluster(force,
                                                                                contact_point);
        double lambda_inv = 0.;

        // from tips to base
        int j = getNodeContainingBody(contact_point.body_index_)->index_;
        while (j > -1)
        {
            const auto &node = reflected_inertia_nodes_[j];
            const int vel_idx = node->velocity_index_;
            const int num_vel = node->num_velocities_;
            const auto joint = node->joint_;

            DVec<double> tmp = joint->S().transpose() * f;
            lambda_inv += tmp.dot(node->D_inv_ * tmp);

            dstate_out +=
                node->qdd_for_subtree_due_to_subtree_root_joint_qdd * node->D_inv_ * tmp;

            f = node->ChiUp_.transpose() * f;

            j = node->parent_index_;
        }

        return lambda_inv;
    }

    double ReflectedInertiaTreeModel::applyTestForceHinv(const Vec3<double> &force,
                                                         const std::string &cp_name,
                                                         DVec<double> &dstate_out)
    {
        const D3Mat<double> J = contactJacobianWorldFrame(cp_name).bottomRows<3>();
        const DMat<double> H = getMassMatrix();
        const DMat<double> H_inv = H.inverse();
        const DMat<double> inv_ops_inertia = J * H_inv * J.transpose();
        dstate_out = H_inv * (J.transpose() * force);
        return force.dot(inv_ops_inertia * force);
    }

    void ReflectedInertiaTreeModel::updateForcePropagators(bool use_reflected_inertia)
    {
        if (force_propagators_updated_)
            return;

        updateArticulatedBodies(use_reflected_inertia);

        for (auto &node : reflected_inertia_nodes_)
        {
            const auto joint = node->joint_;
            const DMat<double> Xup = node->Xup_.toMatrix();
            node->ChiUp_ = Xup - joint->S() * node->D_inv_ * node->U_.transpose() * Xup;
        }

        force_propagators_updated_ = true;
    }

    void ReflectedInertiaTreeModel::updateQddEffects(bool use_reflected_inertia)
    {
        if (qdd_effects_updated_)
            return;

        updateForcePropagators(use_reflected_inertia);

        for (auto &node : reflected_inertia_nodes_)
        {
            const int &vel_idx = node->velocity_index_;
            const int &num_vel = node->num_velocities_;
            const auto joint = node->joint_;

            node->qdd_for_subtree_due_to_subtree_root_joint_qdd
                .middleRows(vel_idx, num_vel)
                .setIdentity();

            // Compute Psi
            const D6Mat<double> &S = joint->S();
            D6Mat<double> Psi = S.transpose().completeOrthogonalDecomposition().pseudoInverse();

            D6Mat<double> F =
                (node->ChiUp_.transpose() - node->Xup_.toMatrix().transpose()) * Psi;

            int j = node->parent_index_;
            while (j > -1)
            {
                auto parent_node = reflected_inertia_nodes_[j];
                const auto parent_joint = parent_node->joint_;

                parent_node->qdd_for_subtree_due_to_subtree_root_joint_qdd
                    .middleRows(vel_idx, num_vel) = F.transpose() * parent_joint->S();

                F = parent_node->ChiUp_.transpose() * F;
                j = parent_node->parent_index_;
            }
        }

        qdd_effects_updated_ = true;
    }

    SVec<double> ReflectedInertiaTreeModel::localCartesianForceAtPointToWorldPluckerForceOnCluster(
        const Vec3<double> &force, const ContactPoint &contact_point)
    {
        const auto node = getNodeContainingBody(contact_point.body_index_);
        const auto &Xa = node->Xa_[0];
        Mat3<double> Rai = Xa.getRotation().transpose();
        spatial::Transform X_cartesian_to_plucker{Rai, contact_point.local_offset_};

        SVec<double> spatial_force = SVec<double>::Zero();
        spatial_force.tail<3>() = force;

        return X_cartesian_to_plucker.inverseTransformForceVector(spatial_force);
    }

    DMat<double> ReflectedInertiaTreeModel::inverseOpSpaceInertiaEFPA(bool use_reflected_inertia)
    {
        // Based on the EFPA from "https://www3.nd.edu/~pwensing/Papers/WensingFeatherstoneOrin12-ICRA.pdf"

        forwardKinematics();
        updateArticulatedBodies(use_reflected_inertia);
        updateForcePropagators(use_reflected_inertia);

        // Reset Force Propagators for the end-effectors
        for (ContactPoint &cp : contact_points_)
        {
            if (!cp.is_end_effector_)
                continue;
            const auto &node = getNodeContainingBody(cp.body_index_);
            cp.ChiUp_[node->index_] = spatial::createSXform(Mat3<double>::Identity(),
                                                            cp.local_offset_);
        }

        // Backward Pass to compute K and propagate the force propagators for the end-effectors
        for (int i = (int)reflected_inertia_nodes_.size() - 1; i >= 0; i--)
        {
            auto &node = reflected_inertia_nodes_[i];
            const DMat<double> &S = node->S();
            node->K_ = S * node->D_inv_ * S.transpose();

            const int &parent_index = node->parent_index_;
            if (parent_index >= 0)
            {
                for (const int &cp_index : node->supported_end_effectors_)
                {
                    ContactPoint &cp = contact_points_[cp_index];
                    cp.ChiUp_[parent_index] = cp.ChiUp_[i] * node->ChiUp_;
                }
            }
        }

        // TODO(@MatthewChignoli): Remove the assumption that every operational space has size 6
        const int num_bodies = reflected_inertia_nodes_.size();
        DMat<double> lambda_inv = DMat<double>::Zero(6 * num_end_effectors_,
                                                     6 * num_end_effectors_);
        DMat<double> lambda_inv_tmp = DMat<double>::Zero(6 * num_bodies,
                                                         6 * num_end_effectors_);

        // Forward Pass
        DMat<double> lambda_inv_prev;
        for (auto &node : reflected_inertia_nodes_)
        {
            const int &node_index = node->index_;          // "i" in Table 1 of the paper
            const int &parent_index = node->parent_index_; // "p(i)"" in Table 1 of the paper

            const int &mss_index = node->motion_subspace_index_;
            const int &mss_dim = node->motion_subspace_dimension_;

            for (const int &cp_index : node->supported_end_effectors_)
            {
                const ContactPoint &contact_point = contact_points_[cp_index];
                const int &k = contact_point.end_effector_index_; // "k" in Table 1 of the paper

                const int ee_output_dim = 6;

                if (parent_index > -1)
                {
                    const auto &parent_node = reflected_inertia_nodes_[parent_index];
                    const int &parent_mss_index = parent_node->motion_subspace_index_;
                    const int &parent_mss_dim = parent_node->motion_subspace_dimension_;
                    lambda_inv_prev = lambda_inv_tmp.block(parent_mss_index, 6 * k,
                                                           parent_mss_dim, ee_output_dim);
                }
                else
                {
                    lambda_inv_prev = DMat<double>::Zero(mss_dim, ee_output_dim);
                }

                lambda_inv_tmp.block(mss_index, 6 * k, mss_dim, ee_output_dim) =
                    node->ChiUp_ * lambda_inv_prev +
                    node->K_ * contact_point.ChiUp_[node_index].transpose();
            }

            for (const std::pair<int, int> &cp_pair : node->nearest_supported_ee_pairs_)
            {
                const ContactPoint &cp1 = contact_points_[cp_pair.first];
                const ContactPoint &cp2 = contact_points_[cp_pair.second];

                const int &k1 = cp1.end_effector_index_; // "k1" in Table 1 of the paper
                const int &k2 = cp2.end_effector_index_; // "k2" in Table 1 of the paper

                const int ee1_output_dim = 6;
                const int ee2_output_dim = 6;

                lambda_inv.block(6 * k1, 6 * k2, ee1_output_dim, ee2_output_dim) =
                    cp1.ChiUp_[node_index] *
                    lambda_inv_tmp.block(mss_index, 6 * k2, mss_dim, ee2_output_dim);

                lambda_inv.block(6 * k2, 6 * k1, ee2_output_dim, ee1_output_dim) =
                    lambda_inv.block(6 * k1, 6 * k2, ee1_output_dim, ee2_output_dim).transpose();
            }
        }

        // And now do the diagonal blocks
        for (int i = 0; i < (int)contact_points_.size(); i++)
        {
            const ContactPoint &cp = contact_points_[i];

            if (!cp.is_end_effector_)
                continue;

            const int &k = cp.end_effector_index_; // "k" in Table 1 of the paper
            const int &ee_output_dim = 6;

            const int node_index = getNodeContainingBody(cp.body_index_)->index_;
            const auto &node = reflected_inertia_nodes_[node_index];
            const int &mss_index = node->motion_subspace_index_;
            const int &mss_dim = node->motion_subspace_dimension_;

            lambda_inv.block(6 * k, 6 * k, ee_output_dim, ee_output_dim) =
                cp.ChiUp_[node_index] *
                lambda_inv_tmp.block(mss_index, 6 * k, mss_dim, ee_output_dim);
        }
        return lambda_inv;
    }

    DMat<double> ReflectedInertiaTreeModel::inverseOpSpaceInertiaHinv()
    {
        const DMat<double> H = getMassMatrix();
        DMat<double> J_stacked = DMat<double>::Zero(6 * getNumEndEffectors(),
                                                    getNumDegreesOfFreedom());
        int ee_cnt = 0;
        for (int i = 0; i < (int)contact_points_.size(); i++)
        {
            const ContactPoint &cp = contact_points_[i];
            if (!cp.is_end_effector_)
                continue;
            J_stacked.middleRows<6>(6 * ee_cnt++) = contactJacobianBodyFrame(cp.name_);
        }
        return J_stacked * H.inverse() * J_stacked.transpose();
    }

} // namespace grbda
