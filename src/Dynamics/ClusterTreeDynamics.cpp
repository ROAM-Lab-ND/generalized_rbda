/*! @file ClusterTreeModel.cpp
 *
 */

#include "grbda/Dynamics/ClusterTreeModel.h"
#include <chrono>

// Profiling accumulators for ID derivatives breakdown
namespace {
    thread_local double prof_fwd_kin_us = 0;
    thread_local double prof_fwd_casadi_us = 0;
    thread_local double prof_fwd_other_us = 0;
    thread_local double prof_bwd_casadi_us = 0;
    thread_local double prof_bwd_other_us = 0;
    thread_local double prof_bwd_prop_us = 0;
    thread_local int prof_count = 0;
    thread_local int prof_fwd_casadi_calls = 0;
    thread_local int prof_bwd_casadi_calls = 0;
    thread_local bool prof_enabled = false;
}

namespace grbda
{
    // Call this to enable profiling
    void enableIDDerivativesProfiling() {
        prof_enabled = true;
        prof_count = 0;
        prof_fwd_casadi_calls = 0;
        prof_bwd_casadi_calls = 0;
    }

    // Get profiling data without printing (returns averages per call)
    // Returns: {fwd_kin, fwd_casadi, fwd_other, bwd_casadi, bwd_other, bwd_prop, total}
    std::vector<double> getIDDerivativesProfilingData() {
        if (prof_count == 0) return {0, 0, 0, 0, 0, 0, 0};
        double total = prof_fwd_kin_us + prof_fwd_casadi_us + prof_fwd_other_us +
                      prof_bwd_casadi_us + prof_bwd_other_us + prof_bwd_prop_us;
        return {
            prof_fwd_kin_us / prof_count,
            prof_fwd_casadi_us / prof_count,
            prof_fwd_other_us / prof_count,
            prof_bwd_casadi_us / prof_count,
            prof_bwd_other_us / prof_count,
            prof_bwd_prop_us / prof_count,
            total / prof_count
        };
    }

    void resetIDDerivativesProfiling() {
        prof_fwd_kin_us = prof_fwd_casadi_us = prof_fwd_other_us = 0;
        prof_bwd_casadi_us = prof_bwd_other_us = prof_bwd_prop_us = 0;
        prof_fwd_casadi_calls = prof_bwd_casadi_calls = 0;
        prof_count = 0;
        prof_enabled = false;
    }

    // Call this to print and reset profiling results
    void printIDDerivativesProfiling() {
        if (prof_count > 0) {
            double total = prof_fwd_kin_us + prof_fwd_casadi_us + prof_fwd_other_us +
                          prof_bwd_casadi_us + prof_bwd_other_us + prof_bwd_prop_us;
            int fwd_calls_per = prof_fwd_casadi_calls / prof_count;
            int bwd_calls_per = prof_bwd_casadi_calls / prof_count;
            std::cout << "\n=== ID Derivatives Profiling (" << prof_count << " calls) ===" << std::endl;
            std::cout << "Forward kinematics:    " << (prof_fwd_kin_us / prof_count) << " us/call ("
                      << (100.0 * prof_fwd_kin_us / total) << "%)" << std::endl;
            std::cout << "Forward pass CasADi:   " << (prof_fwd_casadi_us / prof_count) << " us/call ("
                      << (100.0 * prof_fwd_casadi_us / total) << "%) [" << fwd_calls_per << " fcn calls]" << std::endl;
            std::cout << "Forward pass other:    " << (prof_fwd_other_us / prof_count) << " us/call ("
                      << (100.0 * prof_fwd_other_us / total) << "%)" << std::endl;
            std::cout << "Backward pass CasADi:  " << (prof_bwd_casadi_us / prof_count) << " us/call ("
                      << (100.0 * prof_bwd_casadi_us / total) << "%) [" << bwd_calls_per << " fcn calls]" << std::endl;
            std::cout << "Backward pass other:   " << (prof_bwd_other_us / prof_count) << " us/call ("
                      << (100.0 * prof_bwd_other_us / total) << "%)" << std::endl;
            std::cout << "Backward propagate:    " << (prof_bwd_prop_us / prof_count) << " us/call ("
                      << (100.0 * prof_bwd_prop_us / total) << "%)" << std::endl;
            std::cout << "Total:                 " << (total / prof_count) << " us/call" << std::endl;
            if (fwd_calls_per + bwd_calls_per > 0) {
                double casadi_total = prof_fwd_casadi_us + prof_bwd_casadi_us;
                double casadi_per_fcn = casadi_total / (prof_fwd_casadi_calls + prof_bwd_casadi_calls);
                std::cout << "CasADi avg per fcn:    " << casadi_per_fcn << " us/fcn" << std::endl;
            }
        }
        prof_fwd_kin_us = prof_fwd_casadi_us = prof_fwd_other_us = 0;
        prof_bwd_casadi_us = prof_bwd_other_us = prof_bwd_prop_us = 0;
        prof_fwd_casadi_calls = prof_bwd_casadi_calls = 0;
        prof_count = 0;
        prof_enabled = false;
    }

    template <typename Scalar, typename OriTpl>
    const D6Mat<Scalar> &
    ClusterTreeModel<Scalar, OriTpl>::contactJacobianWorldFrame(const std::string &cp_name)
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

    template <typename Scalar, typename OriTpl>
    D6Mat<Scalar>
    ClusterTreeModel<Scalar, OriTpl>::contactJacobianBodyFrame(const std::string &cp_name)
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

    template <typename Scalar, typename OriTpl>
    DVec<Scalar> ClusterTreeModel<Scalar, OriTpl>::inverseDynamics(const DVec<Scalar> &qdd)
    {
        return this->recursiveNewtonEulerAlgorithm(qdd);
    }

    template <typename Scalar, typename OriTpl>
    DVec<Scalar> ClusterTreeModel<Scalar, OriTpl>::forwardDynamics(const DVec<Scalar> &tau)
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

    template <typename Scalar, typename OriTpl>
    void ClusterTreeModel<Scalar, OriTpl>::updateArticulatedBodies()
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

    template <typename Scalar, typename OriTpl>
    Scalar ClusterTreeModel<Scalar, OriTpl>::applyTestForce(const std::string &contact_point_name,
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
            // CRITICAL FIX: Use transpose()*vec instead of dot() to avoid complex conjugation
            // Eigen's dot(a,b) computes conj(a)^T * b, but we need a^T * b for complex-step
            lambda_inv += (tmp.transpose() * DVec<Scalar>(cluster->D_inv_.solve(tmp)))(0);

            dstate_out +=
                cluster->qdd_for_subtree_due_to_subtree_root_joint_qdd * cluster->D_inv_.solve(tmp);

            f = cluster->ChiUp_.transpose() * f;

            j = cluster->parent_index_;
        }

        return lambda_inv;
    }

    template <typename Scalar, typename OriTpl>
    void ClusterTreeModel<Scalar, OriTpl>::updateForcePropagators()
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

    template <typename Scalar, typename OriTpl>
    void ClusterTreeModel<Scalar, OriTpl>::updateQddEffects()
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

    template <typename Scalar, typename OriTpl>
    DMat<Scalar> ClusterTreeModel<Scalar, OriTpl>::inverseOperationalSpaceInertiaMatrix()
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
                    lambda_inv_prev = DMat<Scalar>::Zero(6, ee_output_dim);
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

    template <typename Scalar, typename OriTpl>
    std::pair<DMat<Scalar>, DMat<Scalar>> ClusterTreeModel<Scalar, OriTpl>::firstOrderInverseDynamicsDerivatives(const DVec<Scalar> &qdd)
    {
        using clock = std::chrono::high_resolution_clock;
        auto t0 = clock::now();

        const auto [q, qd] = this->getState();
        this->forwardAccelerationKinematics(qdd);
        updateArticulatedBodies();

        auto t1 = clock::now();
        if (prof_enabled) prof_fwd_kin_us += std::chrono::duration<double, std::micro>(t1 - t0).count();

        const int nDOF = this->getNumDegreesOfFreedom();
        const int nClusters = static_cast<int>(cluster_nodes_.size());
        DMat<Scalar> dtau_dq = DMat<Scalar>::Zero(nDOF, nDOF);
        DMat<Scalar> dtau_dq_dot = DMat<Scalar>::Zero(nDOF, nDOF);

        double fwd_casadi_local = 0, fwd_other_local = 0;

        // Forward Pass - compute Psi_dot, Psi_ddot, Upsilon_dot, M_cup, B_cup, F for each cluster
        for (auto &cluster : cluster_nodes_)
        {
            auto tf0 = clock::now();

            const int mss_dim = cluster->motion_subspace_dimension_;
            const int num_vel = cluster->num_velocities_;
            const DMat<Scalar> &S = cluster->S();
            const DMat<Scalar> &I = cluster->I_;
            const DVec<Scalar> &v = cluster->v_;

            // Get parent velocity and acceleration
            DVec<Scalar> v_parent_up, a_parent_up;
            if (cluster->parent_index_ >= 0)
            {
                const auto &parent_cluster = cluster_nodes_[cluster->parent_index_];
                v_parent_up = cluster->Xup_.transformMotionVector(parent_cluster->v_);
                a_parent_up = cluster->Xup_.transformMotionVector(parent_cluster->a_);
            }
            else
            {
                v_parent_up = DVec<Scalar>::Zero(mss_dim);
                a_parent_up = cluster->Xup_.transformMotionVector(-this->getGravity());
            }

            // Compute alpha = d(S*qd)/dq and beta = d(S*qdd)/dq using efficient contractions
            // Only compute for joints with configuration-dependent S (e.g., GenericJoint with CasADi)
            const DVec<Scalar> cluster_qd = qd.segment(cluster->velocity_index_, num_vel);
            const DVec<Scalar> cluster_qdd = qdd.segment(cluster->velocity_index_, num_vel);

            DMat<Scalar> alpha, beta, Sdotqd_q;
            const bool has_config_dependent_S = cluster->joint_->hasConfigurationDependentS();
            double iter_casadi_time = 0;

            if (has_config_dependent_S) {
                auto tf1 = clock::now();
                alpha = cluster->joint_->evalSTimesVec_dq(cluster_qd);
                beta = cluster->joint_->evalSTimesVec_dq(cluster_qdd);
                Sdotqd_q = cluster->joint_->getSdotqd_q();
                auto tf2 = clock::now();

                iter_casadi_time = std::chrono::duration<double, std::micro>(tf2 - tf1).count();
                if (prof_enabled) {
                    fwd_casadi_local += iter_casadi_time;
                    prof_fwd_casadi_calls += 3;  // evalSTimesVec_dq x2 + getSdotqd_q
                }
            }

            auto tf3 = clock::now();
            if (prof_enabled) {
                // Forward other = total cluster time minus CasADi time for this iteration
                fwd_other_local += std::chrono::duration<double, std::micro>(tf3 - tf0).count() - iter_casadi_time;
            }

            // Psi_dot = crm(v_parent_up) * S + alpha
            // Use optimized motionCrossTimesMatrix to avoid building full cross-product matrix
            cluster->Psi_dot_ = spatial::motionCrossTimesMatrix(v_parent_up, S);
            if (has_config_dependent_S) {
                cluster->Psi_dot_ += alpha;
            }

            // Cache crm(v)*S since it's used in both Psi_ddot and Upsilon_dot
            const DMat<Scalar> crm_v_S = spatial::motionCrossTimesMatrix(v, S);

            // Psi_ddot = crm(a_parent_up)*S + crm(v_parent_up)*Psi_dot + Sdotqd_q + beta + crm(v)*alpha
            cluster->Psi_ddot_ = spatial::motionCrossTimesMatrix(a_parent_up, S);
            cluster->Psi_ddot_ += spatial::motionCrossTimesMatrix(v_parent_up, cluster->Psi_dot_);
            if (has_config_dependent_S) {
                cluster->Psi_ddot_ += Sdotqd_q + beta;
                cluster->Psi_ddot_ += spatial::motionCrossTimesMatrix(v, alpha);
            }

            // Upsilon_dot = crm(v)*S + Psi_dot + S_ring (reuse cached crm_v_S)
            cluster->Upsilon_dot_ = crm_v_S;
            cluster->Upsilon_dot_ += cluster->Psi_dot_ + cluster->S_ring();

            // M_cup = I (will accumulate children's contributions)
            cluster->M_cup_ = I;

            // B_cup = crf(v)*I - I*crm(v) + icrf(I*v)
            // Use fused spatialInertiaCrossTerms to compute crf(v)*I - I*crm(v) in one pass
            const DVec<Scalar> Iv = I * v;
            cluster->B_cup_ = spatial::spatialInertiaCrossTerms(I, v);
            spatial::addSwappedForceCrossMatrixInPlace(cluster->B_cup_, Iv);

            // F = I*a + crf(v)*I*v
            cluster->F_.noalias() = I * cluster->a_;
            cluster->F_ += spatial::generalForceCrossProduct(v, Iv);

            if (prof_enabled) {
                fwd_other_local += std::chrono::duration<double, std::micro>(clock::now() - tf3).count();
            }
        }

        if (prof_enabled) {
            prof_fwd_casadi_us += fwd_casadi_local;
            prof_fwd_other_us += fwd_other_local;
        }

        double bwd_casadi_local = 0, bwd_other_local = 0, bwd_prop_local = 0;

        // Backward Pass - compute derivatives and propagate M_cup, B_cup, F to parents
        for (int i = nClusters - 1; i >= 0; i--)
        {
            auto tb0 = clock::now();
            double iter_casadi_us = 0;  // CasADi time for this iteration only

            auto &cluster_i = cluster_nodes_[i];
            const int ii = cluster_i->velocity_index_;
            const int num_vel_i = cluster_i->num_velocities_;
            const int mss_dim_i = cluster_i->motion_subspace_dimension_;

            // Cache references
            const DMat<Scalar> &M_cup = cluster_i->M_cup_;
            const DMat<Scalar> &B_cup = cluster_i->B_cup_;
            const DVec<Scalar> &F = cluster_i->F_;
            const DMat<Scalar> &S_i = cluster_i->S();

            // Compute t1, t2, t3, t4 once
            // M_cup and B_cup are block-diagonal, use optimized block-diagonal multiplication
            // The blockDiagonalInertiaTimesMotionSubspace method has a fast path for single-body clusters
            DMat<Scalar> t1 = cluster_i->Xup_.blockDiagonalInertiaTimesMotionSubspace(M_cup, S_i);
            DMat<Scalar> t2 = cluster_i->Xup_.blockDiagonalInertiaTimesMotionSubspace(B_cup, S_i);
            t2.noalias() += cluster_i->Xup_.blockDiagonalInertiaTimesMotionSubspace(M_cup, cluster_i->Upsilon_dot_);
            DMat<Scalar> t3 = cluster_i->Xup_.blockDiagonalInertiaTimesMotionSubspace(B_cup, cluster_i->Psi_dot_);
            t3.noalias() += cluster_i->Xup_.blockDiagonalInertiaTimesMotionSubspace(M_cup, cluster_i->Psi_ddot_);
            t3 += spatial::swappedForceCrossTimesMatrix(F, S_i);
            DMat<Scalar> t4 = cluster_i->Xup_.blockDiagonalInertiaTimesMotionSubspace(B_cup.transpose(), S_i);

            auto tb1 = clock::now();

            // Walk from cluster i to root
            // Use optimized path for single-body clusters (most common case)
            if (mss_dim_i == 6)
            {
                // Single-body cluster: use Transform directly for efficiency
                int j = i;
                while (j >= 0)
                {
                    auto &cluster_j = cluster_nodes_[j];
                    const int jj = cluster_j->velocity_index_;
                    const int num_vel_j = cluster_j->num_velocities_;
                    const DMat<Scalar> &S_j = cluster_j->S();

                    // dtau_dq(ii, jj) = t1^T * Psi_ddot_j + t4^T * Psi_dot_j
                    dtau_dq.block(ii, jj, num_vel_i, num_vel_j).noalias() =
                        t1.transpose() * cluster_j->Psi_ddot_ + t4.transpose() * cluster_j->Psi_dot_;

                    if (j < i)
                    {
                        dtau_dq.block(jj, ii, num_vel_j, num_vel_i).noalias() = S_j.transpose() * t3;
                    }
                    else  // j == i (diagonal block)
                    {
                        // Only compute S^T derivative for joints with config-dependent S
                        if (cluster_i->joint_->hasConfigurationDependentS()) {
                            auto tc0 = clock::now();
                            DMat<Scalar> st_dq = cluster_i->joint_->evalSTTimesVec_dq(F);
                            dtau_dq.block(ii, ii, num_vel_i, num_vel_i) += st_dq;
                            if (prof_enabled) {
                                iter_casadi_us = std::chrono::duration<double, std::micro>(clock::now() - tc0).count();
                                prof_bwd_casadi_calls++;
                            }
                        }
                    }

                    dtau_dq_dot.block(jj, ii, num_vel_j, num_vel_i).noalias() = S_j.transpose() * t2;
                    dtau_dq_dot.block(ii, jj, num_vel_i, num_vel_j).noalias() =
                        t1.transpose() * cluster_j->Upsilon_dot_ + t4.transpose() * S_j;

                    // Transform t1, t2, t3, t4 to parent frame using batched transform
                    // This computes E^T and r_hat*E^T only once for all 4 matrices
                    if (cluster_j->parent_index_ >= 0)
                    {
                        const auto &X = cluster_j->Xup_[0];
                        X.inverseTransformForceSubspace4(t1, t2, t3, t4);
                    }
                    j = cluster_j->parent_index_;
                }
            }
            else
            {
                // Multi-body cluster: use GeneralizedTransform
                int j = i;
                while (j >= 0)
                {
                    auto &cluster_j = cluster_nodes_[j];
                    const int jj = cluster_j->velocity_index_;
                    const int num_vel_j = cluster_j->num_velocities_;
                    const DMat<Scalar> &S_j = cluster_j->S();

                    dtau_dq.block(ii, jj, num_vel_i, num_vel_j).noalias() =
                        t1.transpose() * cluster_j->Psi_ddot_ + t4.transpose() * cluster_j->Psi_dot_;

                    if (j < i)
                    {
                        dtau_dq.block(jj, ii, num_vel_j, num_vel_i).noalias() = S_j.transpose() * t3;
                    }
                    else
                    {
                        // Only compute S^T derivative for joints with config-dependent S
                        if (cluster_i->joint_->hasConfigurationDependentS()) {
                            auto tc0 = clock::now();
                            DMat<Scalar> st_dq = cluster_i->joint_->evalSTTimesVec_dq(F);
                            dtau_dq.block(ii, ii, num_vel_i, num_vel_i) += st_dq;
                            if (prof_enabled) {
                                iter_casadi_us = std::chrono::duration<double, std::micro>(clock::now() - tc0).count();
                                prof_bwd_casadi_calls++;
                            }
                        }
                    }

                    dtau_dq_dot.block(jj, ii, num_vel_j, num_vel_i).noalias() = S_j.transpose() * t2;
                    dtau_dq_dot.block(ii, jj, num_vel_i, num_vel_j).noalias() =
                        t1.transpose() * cluster_j->Upsilon_dot_ + t4.transpose() * S_j;

                    // Transform t1, t2, t3, t4 to parent frame using batched transform
                    // This shares E^T and r_hat*E^T computation across all 4 matrices per body
                    if (cluster_j->parent_index_ >= 0)
                    {
                        cluster_j->Xup_.inverseTransformForceSubspace4(t1, t2, t3, t4);
                    }
                    j = cluster_j->parent_index_;
                }
            }

            auto tb2 = clock::now();

            // Propagate M_cup, B_cup, F to parent
            // Use batched inertia accumulation to share E^T and r_hat computation
            if (cluster_i->parent_index_ >= 0)
            {
                auto &parent_cluster = cluster_nodes_[cluster_i->parent_index_];
                cluster_i->Xup_.accumulateBlockDiagonalInertia2(
                    M_cup, parent_cluster->M_cup_,
                    B_cup, parent_cluster->B_cup_);
                parent_cluster->F_ += cluster_i->Xup_.inverseTransformForceVector(F);
            }

            auto tb3 = clock::now();

            if (prof_enabled) {
                // tb0->tb1: t1-t4 computation (backward other)
                // tb1->tb2: walk to root including CasADi call
                // tb2->tb3: M_cup/B_cup/F propagation (backward propagate)
                bwd_other_local += std::chrono::duration<double, std::micro>(tb1 - tb0).count();
                bwd_other_local += std::chrono::duration<double, std::micro>(tb2 - tb1).count() - iter_casadi_us;
                bwd_casadi_local += iter_casadi_us;
                bwd_prop_local += std::chrono::duration<double, std::micro>(tb3 - tb2).count();
            }
        }

        if (prof_enabled) {
            prof_bwd_casadi_us += bwd_casadi_local;
            prof_bwd_other_us += bwd_other_local;
            prof_bwd_prop_us += bwd_prop_local;
            prof_count++;
        }

        return {dtau_dq, dtau_dq_dot};
    }


    template class ClusterTreeModel<double>;
    template class ClusterTreeModel<std::complex<double>>;
    template class ClusterTreeModel<float>;
    template class ClusterTreeModel<casadi::SX>;

} // namespace grbda
