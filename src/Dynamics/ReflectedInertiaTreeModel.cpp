#include "grbda/Dynamics/ReflectedInertiaTreeModel.h"

namespace grbda
{

    template <typename Scalar>
    ReflectedInertiaTreeModel<Scalar>::ReflectedInertiaTreeModel(
        const ClusterTreeModel<Scalar> &cluster_tree_model,
        RotorInertiaApproximation rotor_inertia_approximation)
        : TreeModel<Scalar>(), rotor_inertia_approximation_(rotor_inertia_approximation)
    {
        this->gravity_ = cluster_tree_model.getGravity();

        extractRigidBodiesAndJointsFromClusterModel(cluster_tree_model);
        extractIndependentCoordinatesFromClusterModel(cluster_tree_model);
        extractContactPointsFromClusterModel(cluster_tree_model);
    }

    template <typename Scalar>
    void ReflectedInertiaTreeModel<Scalar>::extractRigidBodiesAndJointsFromClusterModel(
        const ClusterTreeModel<Scalar> &cluster_tree_model)
    {
        for (const auto &cluster : cluster_tree_model.clusters())
        {
            const int &nv = cluster->num_velocities_;
            DMat<Scalar> cluster_reflected_inertia = DMat<Scalar>::Zero(nv, nv);
            for (const auto &link_joint_and_reflected_inertia :
                 cluster->bodiesJointsAndReflectedInertias())
            {
                const Body<Scalar> &link = std::get<0>(link_joint_and_reflected_inertia);
                const JointPtr<Scalar> link_joint = std::get<1>(link_joint_and_reflected_inertia);
                const DMat<Scalar> &ref_inertia = std::get<2>(link_joint_and_reflected_inertia);

                const int node_index = (int)reflected_inertia_nodes_.size();
                const int parent_node_index = getIndexOfParentNodeForBody(link.parent_index_);
                auto node = std::make_shared<ReflectedInertiaTreeNode<Scalar>>(
                    node_index, link, link_joint, parent_node_index,
                    this->position_index_, this->velocity_index_, this->motion_subspace_index_);
                reflected_inertia_nodes_.push_back(node);
                this->nodes_.push_back(node);

                cluster_reflected_inertia += ref_inertia;

                this->position_index_ += link_joint->numPositions();
                this->velocity_index_ += link_joint->numVelocities();
                this->motion_subspace_index_ += 6;
            }
            reflected_inertia_ = appendEigenMatrix(reflected_inertia_, cluster_reflected_inertia);
        }
        diag_reflected_inertia_ = reflected_inertia_.diagonal().asDiagonal();

        this->H_ = DMat<Scalar>::Zero(this->velocity_index_, this->velocity_index_);
        this->C_ = DVec<Scalar>::Zero(this->velocity_index_);

        // Resize system matrices
        for (auto &node : reflected_inertia_nodes_)
            node->qdd_for_subtree_due_to_subtree_root_joint_qdd
                .setZero(this->getNumDegreesOfFreedom(), node->num_velocities_);
    }

    template <typename Scalar>
    void ReflectedInertiaTreeModel<Scalar>::extractIndependentCoordinatesFromClusterModel(
        const ClusterTreeModel<Scalar> &cluster_tree_model)
    {
        spanning_tree_to_independent_coords_conversion_ = DMat<int>::Zero(0, 0);
        for (const auto &cluster : cluster_tree_model.clusters())
        {
            const auto joint = cluster->joint_;

            if (joint->spanningTreeToIndependentCoordsConversion().size() == 0)
            {
                throw std::runtime_error("Detected a joint that does not have a spanning tree to independent coordinates conversion matrix");
            }

            spanning_tree_to_independent_coords_conversion_ =
                appendEigenMatrix(spanning_tree_to_independent_coords_conversion_,
                                  joint->spanningTreeToIndependentCoordsConversion());
        }

        const int num_bodies = cluster_tree_model.getNumBodies();
        independent_coord_indices_ =
            spanning_tree_to_independent_coords_conversion_ *
            DVec<int>::LinSpaced(num_bodies, 0, num_bodies - 1);

        body_name_to_body_spanning_index_ = cluster_tree_model.body_name_to_body_index_;
    }

    template <typename Scalar>
    void ReflectedInertiaTreeModel<Scalar>::extractContactPointsFromClusterModel(
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
                    auto &node = reflected_inertia_nodes_[i];
                    node->supported_end_effectors_.push_back(contact_point_index);
                    end_effector.supporting_nodes_.push_back(i);
                    i = node->parent_index_;
                }

                // Initialize the force propagators for this end effector
                for (int j = 0; j < (int)reflected_inertia_nodes_.size(); j++)
                {
                    end_effector.ChiUp_.push_back(DMat<Scalar>::Zero(0, 0));
                }

                // Get the nearest shared supporting node for every existing end effector
                for (int k = 0; k < (int)this->contact_points_.size() - 1; k++)
                {
                    if (!this->contact_points_[k].is_end_effector_)
                        continue;
                    std::pair<int, int> cp_pair(k, contact_point_index);
                    const int nearest_shared_support =
                        this->getNearestSharedSupportingNode(cp_pair);
                    reflected_inertia_nodes_[nearest_shared_support]->nearest_supported_ee_pairs_.push_back(cp_pair);
                }
            }
            contact_point_index++;
        }
    }

    template <typename Scalar>
    void ReflectedInertiaTreeModel<Scalar>::setIndependentStates(const DVec<Scalar> &y,
                                                                 const DVec<Scalar> &yd)
    {
        for (auto &node : reflected_inertia_nodes_)
        {
            node->joint_state_.position = y.segment(node->position_index_, node->num_positions_);
            node->joint_state_.velocity = yd.segment(node->velocity_index_, node->num_velocities_);
        }

        this->setExternalForces();
    }

    template <typename Scalar>
    void ReflectedInertiaTreeModel<Scalar>::resetCache()
    {
        TreeModel<Scalar>::resetCache();
        articulated_bodies_updated_ = false;
        force_propagators_updated_ = false;
        qdd_effects_updated_ = false;
    }

    template <typename Scalar>
    DMat<Scalar> ReflectedInertiaTreeModel<Scalar>::getMassMatrix()
    {
        this->compositeRigidBodyAlgorithm();

        switch (rotor_inertia_approximation_)
        {
        case RotorInertiaApproximation::NONE:
            return this->H_;
        case RotorInertiaApproximation::DIAGONAL:
            return this->H_ + diag_reflected_inertia_;
        case RotorInertiaApproximation::BLOCK_DIAGONAL:
            return this->H_ + reflected_inertia_;
        default:
            throw std::runtime_error("Invalid rotor inertia approximation");
        }
    }

    template <typename Scalar>
    DVec<Scalar> ReflectedInertiaTreeModel<Scalar>::getBiasForceVector()
    {
        this->updateBiasForceVector();
        return this->C_;
    }

    // NOTE: The following relationship is true for the rigid body tree model, but not the reflected
    // inertia model: body.index_ = nodes[body.index_].index_
    template <typename Scalar>
    const Body<Scalar> &ReflectedInertiaTreeModel<Scalar>::getBody(int spanning_tree_index) const
    {
        for (const auto &link_node : reflected_inertia_nodes_)
            if (link_node->link_.index_ == spanning_tree_index)
                return link_node->link_;
        throw std::runtime_error("That body does not exist in the link and rotor tree model");
    }

    template <typename Scalar>
    const TreeNodePtr<Scalar>
    ReflectedInertiaTreeModel<Scalar>::getNodeContainingBody(int spanning_tree_index)
    {
        for (const auto &link_node : reflected_inertia_nodes_)
            if (link_node->link_.index_ == spanning_tree_index)
                return link_node;
        throw std::runtime_error("That body does not exist in the link and rotor tree model");
    }

    template <typename Scalar>
    int ReflectedInertiaTreeModel<Scalar>::getIndexOfParentNodeForBody(const int spanning_tree_index)
    {
        if (spanning_tree_index == -1)
            return -1;
        else
            return this->getNodeContainingBody(spanning_tree_index)->index_;
    }

    template <typename Scalar>
    const D6Mat<Scalar> &
    ReflectedInertiaTreeModel<Scalar>::contactJacobianWorldFrame(const std::string &cp_name)
    {
        this->forwardKinematics();

        ContactPoint<Scalar> &cp =
            this->contact_points_[this->contact_name_to_contact_index_.at(cp_name)];

        const size_t &i = cp.body_index_;
        const TreeNodePtr<Scalar> node_i = this->getNodeContainingBody(i);
        const spatial::Transform<Scalar> Xa = node_i->Xa_[0];
        const Mat3<Scalar> R_link_to_world = Xa.getRotation().transpose();
        Mat6<Scalar> Xout = spatial::createSXform(R_link_to_world, cp.local_offset_);

        int j = node_i->index_;
        while (j > -1)
        {
            const auto node_j = reflected_inertia_nodes_[j];
            const int &vel_idx = node_j->velocity_index_;
            const int &num_vel = node_j->num_velocities_;

            const D6Mat<Scalar> &S = node_j->S();
            cp.jacobian_.middleCols(vel_idx, num_vel) = Xout * S;

            const Mat6<Scalar> Xup = node_j->Xup_[0].toMatrix();
            Xout = Xout * Xup;

            j = node_j->parent_index_;
        }

        return cp.jacobian_;
    }

    template <typename Scalar>
    D6Mat<Scalar> ReflectedInertiaTreeModel<Scalar>::contactJacobianBodyFrame(const std::string &cp_name)
    {
        this->forwardKinematics();

        D6Mat<Scalar> J = D6Mat<Scalar>::Zero(6, this->getNumDegreesOfFreedom());

        ContactPoint<Scalar> &cp = this->contact_points_[this->contact_name_to_contact_index_.at(cp_name)];
        const size_t &i = cp.body_index_;
        const auto node_i = this->getNodeContainingBody(i);
        Mat6<Scalar> Xout = spatial::createSXform(Mat3<Scalar>::Identity(), cp.local_offset_);

        int j = node_i->index_;
        while (j > -1)
        {
            const auto node_j = reflected_inertia_nodes_[j];
            const int &vel_idx = node_j->velocity_index_;
            const int &num_vel = node_j->num_velocities_;

            const D6Mat<Scalar> &S = node_j->S();
            J.middleCols(vel_idx, num_vel) = Xout * S;

            const Mat6<Scalar> Xup = node_j->Xup_[0].toMatrix();
            Xout = Xout * Xup;

            j = node_j->parent_index_;
        }

        return J;
    }

    template <typename Scalar>
    DVec<Scalar> ReflectedInertiaTreeModel<Scalar>::forwardDynamics(const DVec<Scalar> &tau)
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

    template <typename Scalar>
    DVec<Scalar> ReflectedInertiaTreeModel<Scalar>::inverseDynamics(const DVec<Scalar> &ydd)
    {
        switch (rotor_inertia_approximation_)
        {
        case RotorInertiaApproximation::NONE:
            return this->recursiveNewtonEulerAlgorithm(ydd);
        case RotorInertiaApproximation::DIAGONAL:
            return this->recursiveNewtonEulerAlgorithm(ydd) + diag_reflected_inertia_ * ydd;
        case RotorInertiaApproximation::BLOCK_DIAGONAL:
            return this->recursiveNewtonEulerAlgorithm(ydd) + reflected_inertia_ * ydd;
        default:
            throw std::runtime_error("Invalid rotor inertia approximation");
        }
    }

    template <typename Scalar>
    DMat<Scalar> ReflectedInertiaTreeModel<Scalar>::inverseOperationalSpaceInertiaMatrix()
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

    template <typename Scalar>
    DVec<Scalar> ReflectedInertiaTreeModel<Scalar>::forwardDynamicsHinv(const DVec<Scalar> &tau)
    {
        this->compositeRigidBodyAlgorithm();
        this->updateBiasForceVector();
        const DMat<Scalar> H_reflected = this->H_ + reflected_inertia_;
        DVec<Scalar> qdd_ref_inertia = matrixInverse(H_reflected) * (tau - this->C_);
        return qdd_ref_inertia;
    }

    template <typename Scalar>
    DVec<Scalar> ReflectedInertiaTreeModel<Scalar>::forwardDynamicsABA(const DVec<Scalar> &tau,
                                                                       bool use_reflected_inertia)
    {
        // Forward dynamics via Articulated Body Algorithm
        this->forwardKinematics();
        updateArticulatedBodies(use_reflected_inertia);

        DVec<Scalar> qdd = DVec<Scalar>::Zero(this->getNumDegreesOfFreedom());

        // Forward Pass - Articulated body bias force
        for (auto &link_node : reflected_inertia_nodes_)
        {
            link_node->pA_ = spatial::generalForceCrossProduct(link_node->v_, DVec<Scalar>(link_node->I_ * link_node->v_));
        }

        // Account for external forces in bias force
        for (int link_node_index : this->indices_of_nodes_experiencing_external_forces_)
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

                Mat6<Scalar> Ia =
                    link_node->IA_ - link_node->U_ * link_node->D_inv_ * link_node->U_.transpose();

                SVec<Scalar> pa =
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

            SVec<Scalar> a_temp;
            if (link_node->parent_index_ >= 0)
            {
                const auto parent_link_node = reflected_inertia_nodes_[link_node->parent_index_];
                a_temp = link_node->Xup_.transformMotionVector(parent_link_node->a_) +
                         link_node->cJ() + link_node->avp_;
            }
            else
            {
                a_temp = link_node->Xup_.transformMotionVector(-this->gravity_) +
                         link_node->cJ() + link_node->avp_;
            }
            qdd.segment(vel_idx, num_vel) =
                link_node->D_inv_ * (link_node->u_ - link_node->U_.transpose() * a_temp);
            link_node->a_ = a_temp + joint->S() * qdd.segment(vel_idx, num_vel);
        }

        return qdd;
    }

    template <typename Scalar>
    void ReflectedInertiaTreeModel<Scalar>::updateArticulatedBodies(bool use_reflected_inertia)
    {
        if (articulated_bodies_updated_)
            return;

        this->forwardKinematics();

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
            DMat<Scalar> D = joint->S().transpose() * link_node->U_;
            if (use_reflected_inertia)
                D += reflected_inertia_.block(vel_idx, vel_idx, num_vel, num_vel);
            link_node->D_inv_ = matrixInverse(D);

            // Articulated body inertia recursion
            if (link_node->parent_index_ >= 0)
            {
                auto parent_link_node = reflected_inertia_nodes_[link_node->parent_index_];
                DMat<Scalar> Ia =
                    link_node->IA_ - link_node->U_ * link_node->D_inv_ * link_node->U_.transpose();
                parent_link_node->IA_ += link_node->Xup_.inverseTransformSpatialInertia(Ia);
            }
        }

        articulated_bodies_updated_ = true;
    }

    template <typename Scalar>
    Scalar ReflectedInertiaTreeModel<Scalar>::applyTestForce(
        const std::string &contact_point_name, const Vec3<Scalar> &force, DVec<Scalar> &dstate_out)
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

    template <typename Scalar>
    Scalar ReflectedInertiaTreeModel<Scalar>::applyTestForceEFPA(const Vec3<Scalar> &force,
                                                                 const std::string &cp_name,
                                                                 DVec<Scalar> &dstate_out,
                                                                 bool use_reflected_inertia)
    {
        const int contact_point_index = this->contact_name_to_contact_index_.at(cp_name);
        const ContactPoint<Scalar>  &contact_point = this->contact_points_[contact_point_index];

        this->forwardKinematics();
        updateArticulatedBodies(use_reflected_inertia);
        updateForcePropagators(use_reflected_inertia);
        updateQddEffects(use_reflected_inertia);

        dstate_out = DVec<Scalar>::Zero(this->getNumDegreesOfFreedom());

        SVec<Scalar> f = localCartesianForceAtPointToWorldPluckerForceOnCluster(force,
                                                                                contact_point);
        Scalar lambda_inv = 0.;

        // from tips to base
        int j = this->getNodeContainingBody(contact_point.body_index_)->index_;
        while (j > -1)
        {
            const auto &node = reflected_inertia_nodes_[j];
            const int vel_idx = node->velocity_index_;
            const int num_vel = node->num_velocities_;
            const auto joint = node->joint_;

            DVec<Scalar> tmp = joint->S().transpose() * f;
            lambda_inv += tmp.dot(node->D_inv_ * tmp);

            dstate_out +=
                node->qdd_for_subtree_due_to_subtree_root_joint_qdd * node->D_inv_ * tmp;

            f = node->ChiUp_.transpose() * f;

            j = node->parent_index_;
        }

        return lambda_inv;
    }

    template <typename Scalar>
    Scalar ReflectedInertiaTreeModel<Scalar>::applyTestForceHinv(const Vec3<Scalar> &force,
                                                                 const std::string &cp_name,
                                                                 DVec<Scalar> &dstate_out)
    {
        const D3Mat<Scalar> J = contactJacobianWorldFrame(cp_name).template bottomRows<3>();
        const DMat<Scalar> H = getMassMatrix();
        const DMat<Scalar> H_inv = matrixInverse(H);
        const DMat<Scalar> inv_ops_inertia = J * H_inv * J.transpose();
        dstate_out = H_inv * (J.transpose() * force);
        return force.dot(inv_ops_inertia * force);
    }

    template <typename Scalar>
    void ReflectedInertiaTreeModel<Scalar>::updateForcePropagators(bool use_reflected_inertia)
    {
        if (force_propagators_updated_)
            return;

        updateArticulatedBodies(use_reflected_inertia);

        for (auto &node : reflected_inertia_nodes_)
        {
            const auto joint = node->joint_;
            const DMat<Scalar> Xup = node->Xup_.toMatrix();
            node->ChiUp_ = Xup - joint->S() * node->D_inv_ * node->U_.transpose() * Xup;
        }

        force_propagators_updated_ = true;
    }

    template <typename Scalar>
    void ReflectedInertiaTreeModel<Scalar>::updateQddEffects(bool use_reflected_inertia)
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
            const DMat<Scalar> &ST = joint->S().transpose();
            D6Mat<Scalar> Psi = matrixRightPseudoInverse(ST);

            D6Mat<Scalar> F =
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

    template <typename Scalar>
    SVec<Scalar>
    ReflectedInertiaTreeModel<Scalar>::localCartesianForceAtPointToWorldPluckerForceOnCluster(
        const Vec3<Scalar> &force, const ContactPoint<Scalar> &contact_point)
    {
        const auto node = this->getNodeContainingBody(contact_point.body_index_);
        const auto &Xa = node->Xa_[0];
        Mat3<Scalar> Rai = Xa.getRotation().transpose();
        spatial::Transform X_cartesian_to_plucker{Rai, contact_point.local_offset_};

        SVec<Scalar> spatial_force = SVec<Scalar>::Zero();
        spatial_force.template tail<3>() = force;

        return X_cartesian_to_plucker.inverseTransformForceVector(spatial_force);
    }

    template <typename Scalar>
    DMat<Scalar>
    ReflectedInertiaTreeModel<Scalar>::inverseOpSpaceInertiaEFPA(bool use_reflected_inertia)
    {
        // Based on the EFPA from "https://www3.nd.edu/~pwensing/Papers/WensingFeatherstoneOrin12-ICRA.pdf"

        this->forwardKinematics();
        updateArticulatedBodies(use_reflected_inertia);
        updateForcePropagators(use_reflected_inertia);

        // Reset Force Propagators for the end-effectors
        for (ContactPoint<Scalar> &cp : this->contact_points_)
        {
            if (!cp.is_end_effector_)
                continue;
            const auto &node = this->getNodeContainingBody(cp.body_index_);
            cp.ChiUp_[node->index_] = spatial::createSXform(Mat3<Scalar>::Identity(),
                                                            cp.local_offset_);
        }

        // Backward Pass to compute K and propagate the force propagators for the end-effectors
        for (int i = (int)reflected_inertia_nodes_.size() - 1; i >= 0; i--)
        {
            auto &node = reflected_inertia_nodes_[i];
            const DMat<Scalar> &S = node->S();
            node->K_ = S * node->D_inv_ * S.transpose();

            const int &parent_index = node->parent_index_;
            if (parent_index >= 0)
            {
                for (const int &cp_index : node->supported_end_effectors_)
                {
                    ContactPoint<Scalar> &cp = this->contact_points_[cp_index];
                    cp.ChiUp_[parent_index] = cp.ChiUp_[i] * node->ChiUp_;
                }
            }
        }

        const int num_bodies = reflected_inertia_nodes_.size();
        DMat<Scalar> lambda_inv = DMat<Scalar>::Zero(6 * this->num_end_effectors_,
                                                     6 * this->num_end_effectors_);
        DMat<Scalar> lambda_inv_tmp = DMat<Scalar>::Zero(6 * num_bodies,
                                                         6 * this->num_end_effectors_);

        // Forward Pass
        DMat<Scalar> lambda_inv_prev;
        for (auto &node : reflected_inertia_nodes_)
        {
            const int &node_index = node->index_;          // "i" in Table 1 of the paper
            const int &parent_index = node->parent_index_; // "p(i)"" in Table 1 of the paper

            const int &mss_index = node->motion_subspace_index_;
            const int &mss_dim = node->motion_subspace_dimension_;

            for (const int &cp_index : node->supported_end_effectors_)
            {
                const ContactPoint<Scalar> &contact_point = this->contact_points_[cp_index];
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
                    lambda_inv_prev = DMat<Scalar>::Zero(mss_dim, ee_output_dim);
                }

                lambda_inv_tmp.block(mss_index, 6 * k, mss_dim, ee_output_dim) =
                    node->ChiUp_ * lambda_inv_prev +
                    node->K_ * contact_point.ChiUp_[node_index].transpose();
            }

            for (const std::pair<int, int> &cp_pair : node->nearest_supported_ee_pairs_)
            {
                const ContactPoint<Scalar> &cp1 = this->contact_points_[cp_pair.first];
                const ContactPoint<Scalar> &cp2 = this->contact_points_[cp_pair.second];

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
        for (int i = 0; i < (int)this->contact_points_.size(); i++)
        {
            const ContactPoint<Scalar> &cp = this->contact_points_[i];

            if (!cp.is_end_effector_)
                continue;

            const int &k = cp.end_effector_index_; // "k" in Table 1 of the paper
            const int &ee_output_dim = 6;

            const int node_index = this->getNodeContainingBody(cp.body_index_)->index_;
            const auto &node = reflected_inertia_nodes_[node_index];
            const int &mss_index = node->motion_subspace_index_;
            const int &mss_dim = node->motion_subspace_dimension_;

            lambda_inv.block(6 * k, 6 * k, ee_output_dim, ee_output_dim) =
                cp.ChiUp_[node_index] *
                lambda_inv_tmp.block(mss_index, 6 * k, mss_dim, ee_output_dim);
        }
        return lambda_inv;
    }

    template <typename Scalar>
    DMat<Scalar> ReflectedInertiaTreeModel<Scalar>::inverseOpSpaceInertiaHinv()
    {
        const DMat<Scalar> H = getMassMatrix();
        DMat<Scalar> J_stacked = DMat<Scalar>::Zero(6 * this->getNumEndEffectors(),
                                                    this->getNumDegreesOfFreedom());
        int ee_cnt = 0;
        for (int i = 0; i < (int)this->contact_points_.size(); i++)
        {
            const ContactPoint<Scalar> &cp = this->contact_points_[i];
            if (!cp.is_end_effector_)
                continue;
            J_stacked.template middleRows<6>(6 * ee_cnt++) = contactJacobianBodyFrame(cp.name_);
        }
        return J_stacked * matrixInverse(H) * J_stacked.transpose();
    }

    template class ReflectedInertiaTreeModel<double>;
template class ReflectedInertiaTreeModel<std::complex<double>>;
    template class ReflectedInertiaTreeModel<casadi::SX>;

} // namespace grbda
