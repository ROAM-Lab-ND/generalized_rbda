/*! @file ClusterTreeModel.cpp
 *
 */

#include "ClusterTreeModel.h"

namespace grbda
{

    using namespace ori;
    using namespace spatial;

    const D6Mat<double> &ClusterTreeModel::contactJacobian(const std::string &cp_name)
    {
        forwardKinematics();

        ContactPoint &cp = contact_points_[contact_name_to_contact_index_.at(cp_name)];
        const size_t i = cp.body_index_;
        const Body &body_i = body(i);
        const auto &cluster_i = getClusterContainingBody(body_i);
        const int &subindex_within_cluster_i = body_i.sub_index_within_cluster_;

        const SpatialTransform Xa = cluster_i->Xa_[subindex_within_cluster_i];
        const Mat3<double> &R_link_to_world = Xa.getRotation().transpose();
        Mat6<double> Xout = createSXform(R_link_to_world, cp.local_offset_);

        int j = (int)i;
        while (j > -1)
        {
            const Body &body_j = body(j);
            const auto &cluster_j = getClusterContainingBody(body_j);
            const int &subindex_within_cluster_j = body_j.sub_index_within_cluster_;
            const int &vel_idx = cluster_j->velocity_index_;
            const int &num_vel = cluster_j->num_velocities_;

            D6Mat<double> S = cluster_j->S().middleRows<6>(6 * subindex_within_cluster_j);
            cp.jacobian_.middleCols(vel_idx, num_vel) = Xout * S;

            Mat6<double> Xup = cluster_j->Xup_[subindex_within_cluster_j].toMatrix();
            Xout = Xout * Xup;

            j = body_j.cluster_ancestor_index_;
        }

        return cp.jacobian_;
    }

    const D6Mat<double> &ClusterTreeModel::bodyJacobian(const std::string &cp_name)
    {
        forwardKinematics();

        ContactPoint &cp = contact_points_[contact_name_to_contact_index_.at(cp_name)];
        Mat6<double> Xout = createSXform(Mat3<double>::Identity(), cp.local_offset_);

        int j = cp.body_index_;
        while (j > -1)
        {
            const Body &body_j = body(j);
            const auto &cluster_j = getClusterContainingBody(body_j);
            const int &subindex_within_cluster_j = body_j.sub_index_within_cluster_;
            const int &vel_idx = cluster_j->velocity_index_;
            const int &num_vel = cluster_j->num_velocities_;

            D6Mat<double> S = cluster_j->S().middleRows<6>(6 * subindex_within_cluster_j);
            cp.jacobian_.middleCols(vel_idx, num_vel) = Xout * S;

            Mat6<double> Xup = cluster_j->Xup_[subindex_within_cluster_j].toMatrix();
            Xout = Xout * Xup;

            j = body_j.cluster_ancestor_index_;
        }

        return cp.jacobian_;
    }

    DVec<double> ClusterTreeModel::inverseDynamics(const DVec<double> &qdd)
    {
        return recursiveNewtonEulerAlgorithm(qdd);
    }

    DVec<double> ClusterTreeModel::forwardDynamics(const DVec<double> &tau)
    {
        DVec<double> qdd = DVec<double>::Zero(getNumDegreesOfFreedom());

        // Forward dynamics via Articulated Body Algorithm
#ifdef TIMING_STATS
        timing_statistics_.zero();
        timer_.start();
#endif
        forwardKinematics();
#ifdef TIMING_STATS
        timing_statistics_.forward_kinematics_time = timer_.getMs();
        timer_.start();
#endif
        updateArticulatedBodies();
#ifdef TIMING_STATS
        timing_statistics_.update_articulated_bodies_time = timer_.getMs();
        timer_.start();
#endif

        // Forward Pass - Articulated body bias force
        for (auto &cluster : cluster_nodes_)
        {
            cluster->pA_ = generalForceCrossProduct(cluster->v_, DVec<double>(cluster->I_ * cluster->v_));
        }
#ifdef TIMING_STATS
        timing_statistics_.forward_pass1_time = timer_.getMs();
        timer_.start();
#endif

        // Account for external forces in bias force
        for (int cluster_index : indices_of_nodes_experiencing_external_forces_)
        {
            auto &cluster = cluster_nodes_[cluster_index];
            cluster->pA_ -= cluster->Xa_.transformExternalForceVector(cluster->f_ext_);
        }
#ifdef TIMING_STATS
        timing_statistics_.external_force_time = timer_.getMs();
        timer_.start();
#endif

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

                const DVec<double> pa = cluster->pA_ + cluster->Ia_ * cluster->c_ +
                                        cluster->U_ * cluster->D_inv_u_;

                parent_cluster->pA_ += cluster->Xup_.inverseTransformForceVector(pa);
            }
        }
#ifdef TIMING_STATS
        timing_statistics_.backward_pass_time = timer_.getMs();
        timer_.start();
#endif

        // Forward Pass - Joint accelerations
        for (auto &cluster : cluster_nodes_)
        {
            const int vel_idx = cluster->velocity_index_;
            const int num_vel = cluster->num_velocities_;
            const auto joint = cluster->joint_;

            DVec<double> a_temp;
            if (cluster->parent_index_ >= 0)
            {
                const auto parent_cluster = cluster_nodes_[cluster->parent_index_];
                a_temp = cluster->Xup_.transformMotionVector(parent_cluster->a_) + cluster->c_;
            }
            else
            {
                a_temp = cluster->Xup_.transformMotionVector(-gravity_) + cluster->c_;
            }
            qdd.segment(vel_idx, num_vel) = cluster->D_inv_u_ - cluster->D_inv_UT_ * a_temp;
            cluster->a_ = a_temp + joint->S() * qdd.segment(vel_idx, num_vel);
        }
#ifdef TIMING_STATS
        timing_statistics_.forward_pass2_time = timer_.getMs();
#endif

        return qdd;
    }

    void ClusterTreeModel::updateArticulatedBodies()
    {
        if (articulated_bodies_updated_)
            return;

        forwardKinematics();

        // Forward pass
#ifdef TIMING_STATS
        double start_time_IA = timer_.getMs();
#endif
        for (auto &cluster : cluster_nodes_)
        {
            cluster->IA_ = cluster->I_;
        }
#ifdef TIMING_STATS
        timing_statistics_.reset_IA_time += timer_.getMs() - start_time_IA;
#endif

        // Backward pass (Gauss principal of least constraint)
        for (int i = (int)cluster_nodes_.size() - 1; i >= 0; i--)
        {
            auto &cluster = cluster_nodes_[i];
            const auto joint = cluster->joint_;
#ifdef TIMING_STATS
            double start_time_D = timer_.getMs();
#endif
            cluster->U_ = cluster->IA_ * joint->S();
            const DMat<double> D = joint->S().transpose() * cluster->U_;
            cluster->updateDinv(D);
            cluster->D_inv_UT_ = cluster->D_inv_.solve(cluster->U_.transpose());
#ifdef TIMING_STATS
            timing_statistics_.update_and_solve_D_time += timer_.getMs() - start_time_D;
#endif

            // Articulated body inertia recursion
            if (cluster->parent_index_ >= 0)
            {
#ifdef TIMING_STATS
                double start_time_Ia = timer_.getMs();
#endif
                auto parent_cluster = cluster_nodes_[cluster->parent_index_];
                cluster->Ia_ = cluster->IA_ - cluster->U_ * cluster->D_inv_UT_;
                parent_cluster->IA_ += cluster->Xup_.inverseTransformSpatialInertia(cluster->Ia_);
#ifdef TIMING_STATS
                timing_statistics_.invert_xform_spatial_inertia_time += timer_.getMs() - start_time_Ia;
#endif
            }
        }

        articulated_bodies_updated_ = true;
    }

    double ClusterTreeModel::applyTestForce(const string &cp_name,
                                            const Vec3<double> &force_ics_at_contact,
                                            DVec<double> &dstate_out)
    {
        return applyLocalFrameTestForceAtContactPoint(force_ics_at_contact, cp_name, dstate_out);
    }

    double
    ClusterTreeModel::applyLocalFrameTestForceAtContactPoint(const Vec3<double> &force,
                                                             const std::string &contact_point_name,
                                                             DVec<double> &dstate_out)
    {
        const int contact_point_index = contact_name_to_contact_index_.at(contact_point_name);
        const ContactPoint &contact_point = contact_points_[contact_point_index];

        forwardKinematics();
        updateArticulatedBodies();
        updateForcePropagators();
        updateQddEffects();

        dstate_out = DVec<double>::Zero(getNumDegreesOfFreedom());

        DVec<double> f = localCartesianForceAtPointToWorldPluckerForceOnCluster(force,
                                                                                contact_point);
        double lambda_inv = 0.;

        // from tips to base
        int j = getIndexOfClusterContainingBody(contact_point.body_index_);
        while (j > -1)
        {
            const auto &cluster = cluster_nodes_[j];
            const int vel_idx = cluster->velocity_index_;
            const int num_vel = cluster->num_velocities_;
            const auto joint = cluster->joint_;

            DVec<double> tmp = joint->S().transpose() * f;
            lambda_inv += tmp.dot(cluster->D_inv_.solve(tmp));

            dstate_out +=
                cluster->qdd_for_subtree_due_to_subtree_root_joint_qdd * cluster->D_inv_.solve(tmp);

            f = cluster->ChiUp_.transpose() * f;

            j = cluster->parent_index_;
        }

        return lambda_inv;
    }

    void ClusterTreeModel::updateForcePropagators()
    {
        if (force_propagators_updated_)
            return;

        updateArticulatedBodies();

        for (auto &cluster : cluster_nodes_)
        {
            const auto joint = cluster->joint_;
            const DMat<double> Xup = cluster->Xup_.toMatrix();
            cluster->ChiUp_ = Xup - joint->S() * cluster->D_inv_UT_ * Xup;
        }

        force_propagators_updated_ = true;
    }

    void ClusterTreeModel::updateQddEffects()
    {
        if (qdd_effects_updated_)
            return;

        updateForcePropagators();

        for (auto &cluster : cluster_nodes_)
        {
            const int &vel_idx = cluster->velocity_index_;
            const int &num_vel = cluster->num_velocities_;
            const auto joint = cluster->joint_;

            cluster->qdd_for_subtree_due_to_subtree_root_joint_qdd
                .middleRows(vel_idx, num_vel)
                .setIdentity();

            // Compute Psi
            const DMat<double> &S = joint->S();
            DMat<double> Psi = S.transpose().completeOrthogonalDecomposition().pseudoInverse();

            DMat<double> F =
                (cluster->ChiUp_.transpose() - cluster->Xup_.toMatrix().transpose()) * Psi;

            int j = cluster->parent_index_;
            while (j > -1)
            {
                auto parent_cluster = cluster_nodes_[j];
                const auto parent_joint = parent_cluster->joint_;

                parent_cluster->qdd_for_subtree_due_to_subtree_root_joint_qdd
                    .middleRows(vel_idx, num_vel) = F.transpose() * parent_joint->S();

                F = parent_cluster->ChiUp_.transpose() * F;
                j = parent_cluster->parent_index_;
            }
        }

        qdd_effects_updated_ = true;
    }

    // TODO(@MatthewChignoli): We don't want every contact point to be a computed as part of the operatational space, so how can we do that selectively?
    DMat<double> ClusterTreeModel::inverseOperationalSpaceInertiaMatrices()
    {
        // Based on the EFPA from "https://www3.nd.edu/~pwensing/Papers/WensingFeatherstoneOrin12-ICRA.pdf"

        forwardKinematics();
        updateArticulatedBodies();
        updateForcePropagators();

        // Reset Force Propagators for the end-effectors
        for (ContactPoint &cp : contact_points_)
        {
            const Body &body = bodies_[cp.body_index_];
            const auto &cluster = getClusterContainingBody(cp.body_index_);

            DMat<double> &ChiUp = cp.ChiUp_[cluster->index_];
            ChiUp = DMat<double>::Zero(6, cluster->motion_subspace_dimension_);
            const Mat6<double> X_offset = createSXform(Mat3<double>::Identity(), cp.local_offset_);
            ChiUp.middleCols<6>(6 * body.sub_index_within_cluster_) = X_offset;
        }

        // Backward Pass to compute K and propagate the force propagators for the end-effectors
        for (int i = (int)cluster_nodes_.size() - 1; i >= 0; i--)
        {
            auto &cluster = cluster_nodes_[i];
            const DMat<double> &S = cluster->joint_->S();
            cluster->K_ = S * cluster->D_inv_.solve(S.transpose());

            const int &parent_index = cluster->parent_index_;
            if (parent_index >= 0)
            {
                for (const int &cp_index : cluster->supported_contact_points_)
                {
                    ContactPoint &cp = contact_points_[cp_index];
                    cp.ChiUp_[parent_index] = cp.ChiUp_[i] * cluster->ChiUp_;
                }
            }
        }

        // TODO(@MatthewChignoli): Right now, this steps makes two assumptions. (1)  Every contact point is considered as part of the operational space and (2) every operational space has dimension 6
        const int num_bodies = bodies_.size();
        const int num_contacts = contact_points_.size();

        DMat<double> lambda_inv = DMat<double>::Zero(6 * num_contacts, 6 * num_contacts);
        DMat<double> lambda_inv_tmp = DMat<double>::Zero(6 * num_bodies, 6 * num_contacts);

        // Forward Pass
        DMat<double> lambda_inv_prev;
        for (auto &cluster : cluster_nodes_)
        {
            const int &cluster_index = cluster->index_;       // "i" in Table 1 of the paper
            const int &parent_index = cluster->parent_index_; // "p(i)"" in Table 1 of the paper

            // TODO(@MatthewChignoli): We assume op space dim = motion subspace dim
            const int &mss_index = cluster->motion_subspace_index_;
            const int &mss_dim = cluster->motion_subspace_dimension_;

            for (const int &cp_index : cluster->supported_contact_points_)
            {
                // cp_index is "k" in Table 1 of the paper

                const ContactPoint &contact_point = contact_points_[cp_index];
                // TODO(@MatthewChignoli): Here is an instance of where we assume all output dimensions are 6
                const int cp_output_dim = 6;

                if (parent_index > -1)
                {
                    const auto &parent_cluster = cluster_nodes_[parent_index];
                    const int &parent_mss_index = parent_cluster->motion_subspace_index_;
                    const int &parent_mss_dim = parent_cluster->motion_subspace_dimension_;
                    // TODO(@MatthewChignoli): Here is an instance of where we assume all output dimensions are 6
                    lambda_inv_prev = lambda_inv_tmp.block(parent_mss_index, 6 * cp_index,
                                                           parent_mss_dim, cp_output_dim);
                }
                else
                    lambda_inv_prev = DMat<double>::Zero(mss_dim, cp_output_dim);

                // TODO(@MatthewChignoli): Here is an instance of where we assume all output dimensions are 6
                DMatRef<double> lambda_inv_tmp_ik = lambda_inv_tmp.block(mss_index, 6 * cp_index,
                                                                         mss_dim, cp_output_dim);
                lambda_inv_tmp_ik = cluster->ChiUp_ * lambda_inv_prev +
                                    cluster->K_ * contact_point.ChiUp_[cluster_index].transpose();
            }

            for (const std::pair<int, int> &cp_pair : cluster->nearest_supported_cp_pairs_)
            {
                const int &cp1_index = cp_pair.first;  // "k1" in Table 1 of the paper
                const int &cp2_index = cp_pair.second; // "k2" in Table 1 of the paper

                // TODO(@MatthewChignoli): Here is an instance of where we assume all output dimensions are 6
                const int cp1_output_dim = 6;
                const int cp2_output_dim = 6;

                // TODO(@MatthewChignoli): Here is an instance of where we assume all output dimensions are 6
                DMatRef<double> lambda_inv_k1k2 = lambda_inv.block(6 * cp1_index, 6 * cp2_index,
                                                                   cp1_output_dim, cp2_output_dim);
                const DMat<double> &ChiUp_k1i = contact_points_[cp1_index].ChiUp_[cluster_index];
                DMatRef<double> lambda_inv_tmp_ik2 = lambda_inv_tmp.block(mss_index, 6 * cp2_index,
                                                                          mss_dim, cp2_output_dim);
                lambda_inv_k1k2 = ChiUp_k1i * lambda_inv_tmp_ik2;

                // TODO(@MatthewChignoli): Here is an instance of where we assume all output dimensions are 6
                DMatRef<double> lambda_inv_k2k1 = lambda_inv.block(6 * cp2_index, 6 * cp1_index,
                                                                   cp2_output_dim, cp1_output_dim);
                lambda_inv_k2k1 = lambda_inv_k1k2.transpose();
            }
        }

        // And now do the diagonal blocks
        for (int k = 0; k < (int)contact_points_.size(); k++)
        {
            const ContactPoint &cp = contact_points_[k];
            // TODO(@MatthewChignoli): Here is an instance of where we assume all output dimensions are 6
            const int &cp_output_dim = 6;

            const int cluster_index = getIndexOfClusterContainingBody(cp.body_index_);
            const auto &cluster = cluster_nodes_[cluster_index];
            const int &mss_index = cluster->motion_subspace_index_;
            const int &mss_dim = cluster->motion_subspace_dimension_;

            // TODO(@MatthewChignoli): Here is an instance of where we assume all output dimensions are 6
            DMatRef<double> lambda_inv_kk = lambda_inv.block(6 * k, 6 * k,
                                                             cp_output_dim, cp_output_dim);
            const DMat<double> &ChiUp_kp = cp.ChiUp_[cluster_index];
            DMatRef<double> lambda_inv_tmp_pk = lambda_inv_tmp.block(mss_index, 6 * k,
                                                                     mss_dim, cp_output_dim);
            lambda_inv_kk = ChiUp_kp * lambda_inv_tmp_pk;
        }
        return lambda_inv;
    }

} // namespace grbda
