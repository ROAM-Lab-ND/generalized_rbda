/*! @file ClusterTreeModel.cpp
 *
 */

#include "grbda/Dynamics/ClusterTreeModel.h"

namespace grbda
{

    template <typename Scalar>
    const D6Mat<Scalar> &
    ClusterTreeModel<Scalar>::contactJacobianWorldFrame(const std::string &cp_name)
    {
        this->forwardKinematics();

        ContactPoint<Scalar> &cp = this->contact_points_[this->contact_name_to_contact_index_.at(cp_name)];
        const size_t i = cp.body_index_;
        const Body<Scalar> &body_i = body(i);
        const auto &cluster_i = getClusterContainingBody(body_i);
        const int &subindex_within_cluster_i = body_i.sub_index_within_cluster_;

        const spatial::Transform<Scalar> Xa = cluster_i->Xa_[subindex_within_cluster_i];
        const Mat3<Scalar> &R_link_to_world = Xa.getRotation().transpose();
        Mat6<Scalar> Xout = spatial::createSXform(R_link_to_world, cp.local_offset_);

        int j = (int)i;
        while (j > -1)
        {
            const Body<Scalar> &body_j = body(j);
            const auto &cluster_j = getClusterContainingBody(body_j);
            const int &subindex_within_cluster_j = body_j.sub_index_within_cluster_;
            const int &vel_idx = cluster_j->velocity_index_;
            const int &num_vel = cluster_j->num_velocities_;

            D6Mat<Scalar> S = cluster_j->S().template middleRows<6>(6 * subindex_within_cluster_j);
            cp.jacobian_.middleCols(vel_idx, num_vel) = Xout * S;

            Mat6<Scalar> Xup = cluster_j->Xup_[subindex_within_cluster_j].toMatrix();
            Xout = Xout * Xup;

            j = body_j.cluster_ancestor_index_;
        }

        return cp.jacobian_;
    }

    template <typename Scalar>
    D6Mat<Scalar> ClusterTreeModel<Scalar>::contactJacobianBodyFrame(const std::string &cp_name)
    {
        this->forwardKinematics();

        D6Mat<Scalar> J = D6Mat<Scalar>::Zero(6, this->getNumDegreesOfFreedom());

        ContactPoint<Scalar> &cp = this->contact_points_[this->contact_name_to_contact_index_.at(cp_name)];
        Mat6<Scalar> Xout = spatial::createSXform(Mat3<Scalar>::Identity(), cp.local_offset_);

        int j = cp.body_index_;
        while (j > -1)
        {
            const Body<Scalar> &body_j = body(j);
            const auto &cluster_j = getClusterContainingBody(body_j);
            const int &subindex_within_cluster_j = body_j.sub_index_within_cluster_;
            const int &vel_idx = cluster_j->velocity_index_;
            const int &num_vel = cluster_j->num_velocities_;

            D6Mat<Scalar> S = cluster_j->S().template middleRows<6>(6 * subindex_within_cluster_j);
            J.middleCols(vel_idx, num_vel) = Xout * S;

            Mat6<Scalar> Xup = cluster_j->Xup_[subindex_within_cluster_j].toMatrix();
            Xout = Xout * Xup;

            j = body_j.cluster_ancestor_index_;
        }

        return J;
    }

    template <typename Scalar>
    DVec<Scalar> ClusterTreeModel<Scalar>::inverseDynamics(const DVec<Scalar> &qdd)
    {
        return this->recursiveNewtonEulerAlgorithm(qdd);
    }

    template <typename Scalar>
    DVec<Scalar> ClusterTreeModel<Scalar>::forwardDynamics(const DVec<Scalar> &tau)
    {
        DVec<Scalar> qdd = DVec<Scalar>::Zero(this->getNumDegreesOfFreedom());

        // Forward dynamics via Articulated Body Algorithm
        this->forwardKinematics();
        updateArticulatedBodies();

        // Forward Pass - Articulated body bias force
        for (auto &cluster : cluster_nodes_)
        {
            cluster->pA_ = spatial::generalForceCrossProduct(cluster->v_, DVec<Scalar>(cluster->I_ * cluster->v_));
        }

        // Account for external forces in bias force
        for (int cluster_index : this->indices_of_nodes_experiencing_external_forces_)
        {
            auto &cluster = cluster_nodes_[cluster_index];
            cluster->pA_ -= cluster->Xa_.transformExternalForceVector(cluster->f_ext_);
        }

        // Backward pass - Gauss principal of least constraint
        for (int i = (int)cluster_nodes_.size() - 1; i >= 0; i--)
        {
            auto &cluster = cluster_nodes_[i];
            const int vel_idx = cluster->velocity_index_;
            const int num_vel = cluster->num_velocities_;
            const auto joint = cluster->joint_;

            cluster->u_ = tau.segment(vel_idx, num_vel) - joint->S().transpose() * cluster->pA_;
            cluster->D_inv_u_ = cluster->D_inv_.solve(cluster->u_);

            // Articulated body bias force recursion
            if (cluster->parent_index_ >= 0)
            {
                auto parent_cluster = cluster_nodes_[cluster->parent_index_];

                const DVec<Scalar> pa = cluster->pA_ +
                                        cluster->Ia_ * (cluster->cJ() + cluster->avp_) +
                                        cluster->U_ * cluster->D_inv_u_;

                parent_cluster->pA_ += cluster->Xup_.inverseTransformForceVector(pa);
            }
        }

        // Forward Pass - Joint accelerations
        for (auto &cluster : cluster_nodes_)
        {
            const int vel_idx = cluster->velocity_index_;
            const int num_vel = cluster->num_velocities_;
            const auto joint = cluster->joint_;

            DVec<Scalar> a_temp;
            if (cluster->parent_index_ >= 0)
            {
                const auto parent_cluster = cluster_nodes_[cluster->parent_index_];
                a_temp = cluster->Xup_.transformMotionVector(parent_cluster->a_) +
                         cluster->cJ() + cluster->avp_;
            }
            else
            {
                a_temp = cluster->Xup_.transformMotionVector(-this->gravity_) +
                         cluster->cJ() + cluster->avp_;
            }
            qdd.segment(vel_idx, num_vel) = cluster->D_inv_u_ - cluster->D_inv_UT_ * a_temp;
            cluster->a_ = a_temp + joint->S() * qdd.segment(vel_idx, num_vel);
        }

        return qdd;
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::updateArticulatedBodies()
    {
        if (articulated_bodies_updated_)
            return;

        this->forwardKinematics();

        // Forward pass
        for (auto &cluster : cluster_nodes_)
        {
            cluster->IA_ = cluster->I_;
        }

        // Backward pass (Gauss principal of least constraint)
        for (int i = (int)cluster_nodes_.size() - 1; i >= 0; i--)
        {
            auto &cluster = cluster_nodes_[i];
            const auto joint = cluster->joint_;
            cluster->U_ = cluster->IA_ * joint->S();
            const DMat<Scalar> D = joint->S().transpose() * cluster->U_;
            cluster->updateDinv(D);
            cluster->D_inv_UT_ = cluster->D_inv_.solve(cluster->U_.transpose());

            // Articulated body inertia recursion
            if (cluster->parent_index_ >= 0)
            {
                auto parent_cluster = cluster_nodes_[cluster->parent_index_];
                cluster->Ia_ = cluster->IA_ - cluster->U_ * cluster->D_inv_UT_;
                parent_cluster->IA_ += cluster->Xup_.inverseTransformSpatialInertia(cluster->Ia_);
            }
        }

        articulated_bodies_updated_ = true;
    }

    template <typename Scalar>
    Scalar ClusterTreeModel<Scalar>::applyTestForce(const std::string &contact_point_name,
                                                    const Vec3<Scalar> &force,
                                                    DVec<Scalar> &dstate_out)
    {
        const int contact_point_index = this->contact_name_to_contact_index_.at(contact_point_name);
        const ContactPoint<Scalar> &contact_point = this->contact_points_[contact_point_index];

        this->forwardKinematics();
        updateArticulatedBodies();
        updateForcePropagators();
        updateQddEffects();

        dstate_out = DVec<Scalar>::Zero(this->getNumDegreesOfFreedom());

        DVec<Scalar> f = localCartesianForceAtPointToWorldPluckerForceOnCluster(force,
                                                                                contact_point);
        Scalar lambda_inv = 0.;

        // from tips to base
        int j = getIndexOfClusterContainingBody(contact_point.body_index_);
        while (j > -1)
        {
            const auto &cluster = cluster_nodes_[j];
            const int vel_idx = cluster->velocity_index_;
            const int num_vel = cluster->num_velocities_;
            const auto joint = cluster->joint_;

            DVec<Scalar> tmp = joint->S().transpose() * f;
            lambda_inv += tmp.dot(DVec<Scalar>(cluster->D_inv_.solve(tmp)));

            dstate_out +=
                cluster->qdd_for_subtree_due_to_subtree_root_joint_qdd * cluster->D_inv_.solve(tmp);

            f = cluster->ChiUp_.transpose() * f;

            j = cluster->parent_index_;
        }

        return lambda_inv;
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::updateForcePropagators()
    {
        if (force_propagators_updated_)
            return;

        updateArticulatedBodies();

        for (auto &cluster : cluster_nodes_)
        {
            const int &mss_dim = cluster->motion_subspace_dimension_;
            const DMat<Scalar> L = DMat<Scalar>::Identity(mss_dim, mss_dim) -
                                   cluster->S() * cluster->D_inv_UT_;
            cluster->ChiUp_ = cluster->Xup_.rightMultiplyMotionTransform(L);
        }

        force_propagators_updated_ = true;
    }

    template <typename Scalar>
    void ClusterTreeModel<Scalar>::updateQddEffects()
    {
        if (qdd_effects_updated_)
            return;

        updateForcePropagators();

        for (auto &cluster : cluster_nodes_)
        {
            const int &vel_idx = cluster->velocity_index_;
            const int &num_vel = cluster->num_velocities_;

            cluster->qdd_for_subtree_due_to_subtree_root_joint_qdd
                .middleRows(vel_idx, num_vel)
                .setIdentity();

            // Compute Psi
            const DMat<Scalar> &ST = cluster->S().transpose();
            DMat<Scalar> Psi = matrixRightPseudoInverse(ST);

            DMat<Scalar> F =
                (cluster->ChiUp_.transpose() - cluster->Xup_.toMatrix().transpose()) * Psi;

            int j = cluster->parent_index_;
            while (j > -1)
            {
                auto parent_cluster = cluster_nodes_[j];

                parent_cluster->qdd_for_subtree_due_to_subtree_root_joint_qdd
                    .middleRows(vel_idx, num_vel) = F.transpose() * parent_cluster->S();

                F = parent_cluster->ChiUp_.transpose() * F;
                j = parent_cluster->parent_index_;
            }
        }

        qdd_effects_updated_ = true;
    }

    template <typename Scalar>
    DMat<Scalar> ClusterTreeModel<Scalar>::inverseOperationalSpaceInertiaMatrix()
    {
        // Based on the EFPA from "https://www3.nd.edu/~pwensing/Papers/WensingFeatherstoneOrin12-ICRA.pdf"

        typedef typename CorrectMatrixLltType<Scalar>::type LltType;

        this->forwardKinematics();
        for (auto &cluster : cluster_nodes_)
        {
            cluster->IA_ = cluster->I_;
        }

        // Reset Force Propagators for the end-effectors
        for (ContactPoint<Scalar> &cp : this->contact_points_)
        {
            if (!cp.is_end_effector_)
                continue;

            const Body<Scalar> &body = bodies_[cp.body_index_];
            const auto &cluster = getClusterContainingBody(cp.body_index_);

            DMat<Scalar> &ChiUp = cp.ChiUp_[cluster->index_];
            ChiUp = DMat<Scalar>::Zero(6, cluster->motion_subspace_dimension_);
            const Mat6<Scalar> X_offset = spatial::createSXform(Mat3<Scalar>::Identity(),
                                                                cp.local_offset_);
            ChiUp.template middleCols<6>(6 * body.sub_index_within_cluster_) = X_offset;
        }

        // Backward Pass to compute K and propagate the force propagators for the end-effectors
        for (int i = (int)cluster_nodes_.size() - 1; i >= 0; i--)
        {
            auto &cluster = cluster_nodes_[i];

            const DMat<Scalar> &S = cluster->S();
            const DMat<Scalar> ST = S.transpose();
            cluster->K_ = S * (LltType(ST * cluster->IA_ * S).solve(ST));

            const int &mss_dim = cluster->motion_subspace_dimension_;
            cluster->L_ = DMat<Scalar>::Identity(mss_dim, mss_dim) - cluster->K_ * cluster->IA_;
            cluster->ChiUp_ = cluster->Xup_.rightMultiplyMotionTransform(cluster->L_);

            const int &parent_index = cluster->parent_index_;
            if (parent_index >= 0)
            {
                auto &parent_cluster = cluster_nodes_[parent_index];
                parent_cluster->IA_ +=
                    cluster->Xup_.inverseTransformSpatialInertia(cluster->L_.transpose() *
                                                                 cluster->IA_);

                for (const int &cp_index : cluster->supported_end_effectors_)
                {
                    ContactPoint<Scalar> &cp = this->contact_points_[cp_index];
                    cp.ChiUp_[parent_index] = cp.ChiUp_[i] * cluster->ChiUp_;
                }
            }
        }

        const int num_bodies = bodies_.size();
        DMat<Scalar> lambda_inv = DMat<Scalar>::Zero(6 * this->num_end_effectors_,
                                                     6 * this->num_end_effectors_);
        DMat<Scalar> lambda_inv_tmp = DMat<Scalar>::Zero(6 * num_bodies,
                                                         6 * this->num_end_effectors_);

        // Forward Pass
        DMat<Scalar> lambda_inv_prev;
        for (auto &cluster : cluster_nodes_)
        {
            const int &cluster_index = cluster->index_;       // "i" in Table 1 of the paper
            const int &parent_index = cluster->parent_index_; // "p(i)"" in Table 1 of the paper

            const int &mss_index = cluster->motion_subspace_index_;
            const int &mss_dim = cluster->motion_subspace_dimension_;

            for (const int &cp_index : cluster->supported_end_effectors_)
            {
                const ContactPoint<Scalar> &contact_point = this->contact_points_[cp_index];
                const int &k = contact_point.end_effector_index_; // "k" in Table 1 of the paper

                const int ee_output_dim = 6;

                if (parent_index > -1)
                {
                    const auto &parent_cluster = cluster_nodes_[parent_index];
                    const int &parent_mss_index = parent_cluster->motion_subspace_index_;
                    const int &parent_mss_dim = parent_cluster->motion_subspace_dimension_;
                    lambda_inv_prev = lambda_inv_tmp.block(parent_mss_index, 6 * k,
                                                           parent_mss_dim, ee_output_dim);
                }
                else
                {
                    lambda_inv_prev = DMat<Scalar>::Zero(mss_dim, ee_output_dim);
                }

                lambda_inv_tmp.block(mss_index, 6 * k, mss_dim, ee_output_dim) =
                    cluster->ChiUp_ * lambda_inv_prev +
                    cluster->K_ * contact_point.ChiUp_[cluster_index].transpose();
            }

            for (const std::pair<int, int> &cp_pair : cluster->nearest_supported_ee_pairs_)
            {
                const ContactPoint<Scalar> &cp1 = this->contact_points_[cp_pair.first];
                const ContactPoint<Scalar> &cp2 = this->contact_points_[cp_pair.second];

                const int &k1 = cp1.end_effector_index_; // "k1" in Table 1 of the paper
                const int &k2 = cp2.end_effector_index_; // "k2" in Table 1 of the paper

                const int ee1_output_dim = 6;
                const int ee2_output_dim = 6;

                lambda_inv.block(6 * k1, 6 * k2, ee1_output_dim, ee2_output_dim) =
                    cp1.ChiUp_[cluster_index] *
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

            const int cluster_index = getIndexOfClusterContainingBody(cp.body_index_);
            const auto &cluster = cluster_nodes_[cluster_index];
            const int &mss_index = cluster->motion_subspace_index_;
            const int &mss_dim = cluster->motion_subspace_dimension_;

            lambda_inv.block(6 * k, 6 * k, ee_output_dim, ee_output_dim) =
                cp.ChiUp_[cluster_index] *
                lambda_inv_tmp.block(mss_index, 6 * k, mss_dim, ee_output_dim);
        }

        return lambda_inv;
    }

    template class ClusterTreeModel<double>;
    template class ClusterTreeModel<float>;
    template class ClusterTreeModel<casadi::SX>;

} // namespace grbda
