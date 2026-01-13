/*! @file ClusterTreeModel.cpp
 *
 */

#include "grbda/Dynamics/ClusterTreeModel.h"
#include "grbda/Utils/JointDerivatives.h"

namespace grbda
{

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
        const auto [q, qd] = this->getState();
        this->forwardAccelerationKinematics(qdd);
        updateArticulatedBodies();
        DMat<Scalar> dtau_dq = DMat<Scalar>::Zero(this->getNumDegreesOfFreedom(), this->getNumDegreesOfFreedom());
        DMat<Scalar> dtau_dq_dot = DMat<Scalar>::Zero(this->getNumDegreesOfFreedom(), this->getNumDegreesOfFreedom());
        //Forward Pass
        for (auto &cluster : cluster_nodes_)
        {
            // Get parent velocity and acceleration
            // For root cluster (parent_index_ == -1): parent is ground with v=0, a=-gravity
            // For other clusters: parent is the actual parent cluster
            DVec<Scalar> v_parent, a_parent;
            if (cluster->parent_index_ >= 0)
            {
                auto &parent_cluster = cluster_nodes_[cluster->parent_index_];
                v_parent = parent_cluster->v_;
                a_parent = parent_cluster->a_;
            }
            else
            {
                v_parent = DVec<Scalar>::Zero(6);
                a_parent = -this->getGravity();
            }

            const auto v_parent_up = cluster->Xup_.transformMotionVector(v_parent);
            const auto a_parent_up = cluster->Xup_.transformMotionVector(a_parent);

            // Compute alpha = contract(S_q, qd) - corresponds to MATLAB ID_derivatives.m line 32
            const DVec<Scalar> cluster_qd = qd.segment(cluster->velocity_index_, cluster->num_velocities_);
            const DMat<Scalar> alpha = contractSqWithVector(cluster->joint_->getSq(), cluster_qd, cluster->S().rows());

            cluster->Psi_dot_ =
            spatial::generalMotionCrossMatrix(v_parent_up) * cluster->S() + alpha;

            // Compute beta = contract(S_q, qdd) - corresponds to MATLAB ID_derivatives.m line 33
            const DVec<Scalar> cluster_qdd = qdd.segment(cluster->velocity_index_, cluster->num_velocities_);
            const DMat<Scalar> beta = contractSqWithVector(cluster->joint_->getSq(), cluster_qdd, cluster->S().rows());

            // Compute new_part = Sdotqd_q + beta + crm(v)*alpha - corresponds to MATLAB ID_derivatives.m line 36
            const DMat<Scalar> Sdotqd_q = cluster->joint_->getSdotqd_q();
            const DMat<Scalar> crm_v_alpha = spatial::generalMotionCrossMatrix(cluster->v_) * alpha;
            const DMat<Scalar> new_part = Sdotqd_q + beta + crm_v_alpha;

            // Debug: Check intermediate values
            if constexpr (std::is_same_v<Scalar, double>) {
                if (!new_part.allFinite()) {
                    std::cout << "[DEBUG new_part] Cluster " << cluster->velocity_index_ << std::endl;
                    std::cout << "  Sdotqd_q: " << Sdotqd_q.rows() << "x" << Sdotqd_q.cols() << " finite=" << Sdotqd_q.allFinite() << std::endl;
                    std::cout << "  beta: " << beta.rows() << "x" << beta.cols() << " finite=" << beta.allFinite() << std::endl;
                    std::cout << "  crm_v_alpha: " << crm_v_alpha.rows() << "x" << crm_v_alpha.cols() << " finite=" << crm_v_alpha.allFinite() << std::endl;
                    std::cout << "  Sdotqd_q+beta finite: " << (Sdotqd_q + beta).allFinite() << std::endl;
                    std::cout << "  beta+crm_v_alpha finite: " << (beta + crm_v_alpha).allFinite() << std::endl;
                }
            }

            cluster->Psi_ddot_ =
            (spatial::generalMotionCrossMatrix(a_parent_up) * cluster->S()).eval()
            + spatial::generalMotionCrossMatrix(v_parent_up) * cluster->Psi_dot_
            + new_part;

            // Debug: Check Psi_ddot_ for NaN in forward pass
            if constexpr (std::is_same_v<Scalar, double>) {
                if (!cluster->Psi_ddot_.allFinite()) {
                    std::cout << "[DEBUG FORWARD] Cluster " << cluster->velocity_index_
                              << " Psi_ddot_ contains NaN!" << std::endl;
                    std::cout << "  alpha finite: " << alpha.allFinite() << std::endl;
                    std::cout << "  beta finite: " << beta.allFinite() << std::endl;
                    std::cout << "  Sdotqd_q finite: " << Sdotqd_q.allFinite() << std::endl;
                    std::cout << "  new_part finite: " << new_part.allFinite() << std::endl;
                    std::cout << "  crm(a_parent_up)*S finite: " << (spatial::generalMotionCrossMatrix(a_parent_up) * cluster->S()).allFinite() << std::endl;
                    std::cout << "  crm(v_parent_up)*Psi_dot finite: " << (spatial::generalMotionCrossMatrix(v_parent_up) * cluster->Psi_dot_).allFinite() << std::endl;
                }
            }

            cluster->Upsilon_dot_ = (spatial::generalMotionCrossMatrix(cluster->v_) * cluster->S()).eval()
            + cluster->Psi_dot_ + cluster->S_ring();

            cluster->M_cup_ = cluster->I_;

#ifdef GRBDA_DEBUG_DERIVATIVES
            // Print body mass from inertia matrix
            if (cluster->I_.rows() >= 6 && cluster->I_.cols() >= 6) {
                std::cout << "[DEBUG] Forward pass - Body mass from I_[3,3] = " << cluster->I_(3,3) << "\n";
            }
#endif

            cluster->B_cup_ = spatial::generalForceCrossMatrix(cluster->v_) * cluster->I_
            - cluster->I_ * spatial::generalMotionCrossMatrix(cluster->v_)
            + spatial::generalSwappedForceCrossMatrix(DVec<Scalar>(cluster->I_ * cluster->v_));

#ifdef GRBDA_DEBUG_DERIVATIVES
            // Debug BC computation for Body 1 (floating base - parent_index == -1)
            if (cluster->parent_index_ == -1) {
                std::cout << "\n[DEBUG] Body 1 B_cup FRESH (recomputed, before accumulation):\n";
                for (int row = 0; row < std::min(3, (int)cluster->B_cup_.rows()); row++) {
                    std::cout << "   ";
                    for (int col = 0; col < std::min(6, (int)cluster->B_cup_.cols()); col++) {
                        std::cout << " " << std::setw(12) << std::setprecision(6) << std::scientific << cluster->B_cup_(row, col);
                    }
                    std::cout << "\n";
                }
            }
#endif

            cluster->F_ = cluster->I_ * cluster->a_ + spatial::generalForceCrossMatrix(cluster->v_) * cluster->I_ * cluster->v_;
            cluster->F_ = cluster->I_ * cluster->a_ + spatial::generalForceCrossMatrix(cluster->v_) * cluster->I_ * cluster->v_;
            
        }
        //Backward Pass
        for (int i = (int)cluster_nodes_.size() - 1; i >= 0; i--)
        {
            auto &cluster_i = cluster_nodes_[i];
            const int &ii = cluster_i->velocity_index_;

#ifdef GRBDA_DEBUG_DERIVATIVES
            // Print detailed M_cup comparison for Body 0 (floating base)
            if (i == 0) {
                std::cout << "\n[DEBUG] Body 0 (Floating Base) M_cup Analysis:\n";
                std::cout << "  C++ M_cup:\n";
                for (int row = 0; row < 6; row++) {
                    std::cout << "    [";
                    for (int col = 0; col < 6; col++) {
                        std::cout << std::setw(12) << std::setprecision(4) << std::fixed << cluster_i->M_cup_(row, col);
                        if (col < 5) std::cout << ", ";
                    }
                    std::cout << "]\n";
                }
                std::cout << "\n  MATLAB expected IC{1}:\n";
                std::cout << "    [ 1.6972, -0.9437, -2.5452, -0.0000, -1.4635,  0.5622]\n";
                std::cout << "    [-0.9437,  6.9044, -0.5035,  1.4641,  0.0000, -3.6315]\n";
                std::cout << "    [-2.5452, -0.5035,  5.5872, -0.5622,  3.6315,  0.0000]\n";
                std::cout << "    [ 0.0000,  1.4641, -0.5622,  3.0000,  0.0000, -0.0000]\n";
                std::cout << "    [-1.4635,  0.0000,  3.6315,  0.0000,  3.0000, -0.0000]\n";
                std::cout << "    [ 0.5622, -3.6315, -0.0000, -0.0000, -0.0000,  3.0000]\n";

                std::cout << "\n  Ratios (C++/MATLAB) for key elements:\n";
                std::cout << "    Upper-left 3x3 (rotational inertia):\n";
                std::cout << "      [0,0]: " << cluster_i->M_cup_(0,0)/1.6972 << "\n";
                std::cout << "      [0,1]: " << cluster_i->M_cup_(0,1)/(-0.9437) << "\n";
                std::cout << "      [1,1]: " << cluster_i->M_cup_(1,1)/6.9044 << "\n";
                std::cout << "    Lower-right 3x3 (mass):\n";
                std::cout << "      [3,3]: " << cluster_i->M_cup_(3,3)/3.0 << " (should be 1.0)\n";
                std::cout << "      [4,4]: " << cluster_i->M_cup_(4,4)/3.0 << " (should be 1.0)\n";
            }
#endif

            DMat<Scalar> t1 = cluster_i->M_cup_ * cluster_i->S();
            DMat<Scalar> t2 = DMat<Scalar>(cluster_i->B_cup_ * cluster_i->S()) + DMat<Scalar>(cluster_i->M_cup_ * cluster_i->Upsilon_dot_);
            DMat<Scalar> t3 = DMat<Scalar>(cluster_i->B_cup_ * cluster_i->Psi_dot_) + DMat<Scalar>(cluster_i->M_cup_ * cluster_i->Psi_ddot_)
            + DMat<Scalar>(spatial::generalSwappedForceCrossMatrix(cluster_i->F_)*cluster_i->S());
            DMat<Scalar> t4 = cluster_i->B_cup_.transpose() * cluster_i->S();

            // Debug: Check t1-t4 for NaN (only for double type)
            if constexpr (std::is_same_v<Scalar, double>) {
                if (!t1.allFinite() || !t2.allFinite() || !t3.allFinite() || !t4.allFinite()) {
                    std::cout << "[DEBUG] Cluster " << i << " t-matrices contain NaN:" << std::endl;
                    std::cout << "  t1 finite: " << t1.allFinite() << std::endl;
                    std::cout << "  t2 finite: " << t2.allFinite() << std::endl;
                    std::cout << "  t3 finite: " << t3.allFinite() << std::endl;
                    std::cout << "  t4 finite: " << t4.allFinite() << std::endl;
                    std::cout << "  M_cup_ finite: " << cluster_i->M_cup_.allFinite() << std::endl;
                    std::cout << "  B_cup_ finite: " << cluster_i->B_cup_.allFinite() << std::endl;
                    std::cout << "  S() finite: " << cluster_i->S().allFinite() << std::endl;
                    std::cout << "  Upsilon_dot_ finite: " << cluster_i->Upsilon_dot_.allFinite() << std::endl;
                    std::cout << "  Psi_dot_ finite: " << cluster_i->Psi_dot_.allFinite() << std::endl;
                    std::cout << "  Psi_ddot_ finite: " << cluster_i->Psi_ddot_.allFinite() << std::endl;
                    std::cout << "  F_ finite: " << cluster_i->F_.allFinite() << std::endl;
                }
            }

#ifdef GRBDA_DEBUG_DERIVATIVES
            // Debug output for comparing with MATLAB
            if (i == 0) { // Body 1 (floating base) - index 0
                std::cout << "\n[DEBUG] Body 1 Backward Pass tmp matrices:\n";
                std::cout << "tmp1 (IC*S) size: " << t1.rows() << "x" << t1.cols() << "\n";
                std::cout << "First 3 columns:\n";
                for (int row = 0; row < std::min(6, (int)t1.rows()); row++) {
                    std::cout << "  ";
                    for (int col = 0; col < std::min(3, (int)t1.cols()); col++) {
                        std::cout << std::setw(18) << std::setprecision(10) << std::scientific << t1(row, col) << " ";
                    }
                    std::cout << "\n";
                }

                std::cout << "\ntmp2 (BC*S + IC*Upsilond) first 3 cols:\n";
                for (int row = 0; row < std::min(6, (int)t2.rows()); row++) {
                    std::cout << "  ";
                    for (int col = 0; col < std::min(3, (int)t2.cols()); col++) {
                        std::cout << std::setw(18) << std::setprecision(10) << std::scientific << t2(row, col) << " ";
                    }
                    std::cout << "\n";
                }

                std::cout << "\ntmp3 (BC*Psid + IC*Psidd + icrf(f)*S) first 3 cols:\n";
                for (int row = 0; row < std::min(6, (int)t3.rows()); row++) {
                    std::cout << "  ";
                    for (int col = 0; col < std::min(3, (int)t3.cols()); col++) {
                        std::cout << std::setw(18) << std::setprecision(10) << std::scientific << t3(row, col) << " ";
                    }
                    std::cout << "\n";
                }

                std::cout << "\ntmp4 (BC^T*S) first 3 cols:\n";
                for (int row = 0; row < std::min(6, (int)t4.rows()); row++) {
                    std::cout << "  ";
                    for (int col = 0; col < std::min(3, (int)t4.cols()); col++) {
                        std::cout << std::setw(18) << std::setprecision(10) << std::scientific << t4(row, col) << " ";
                    }
                    std::cout << "\n";
                }
                std::cout << std::endl;
            }
#endif

            int j = i;

            while (j >= 0)
            {
                auto &cluster_j = cluster_nodes_[j];
                const int &jj = cluster_j->velocity_index_;

                DMat<Scalar> block_val = t1.transpose() * cluster_j->Psi_ddot_ + t4.transpose() * cluster_j->Psi_dot_;

                // Debug: Check for NaN in block assignment
                if constexpr (std::is_same_v<Scalar, double>) {
                    if (!block_val.allFinite()) {
                        std::cout << "[DEBUG] NaN in dtau_dq block(" << ii << "," << jj << ") for clusters i=" << i << ", j=" << j << std::endl;
                        std::cout << "  t1.transpose() * Psi_ddot finite: " << (t1.transpose() * cluster_j->Psi_ddot_).allFinite() << std::endl;
                        std::cout << "  t4.transpose() * Psi_dot finite: " << (t4.transpose() * cluster_j->Psi_dot_).allFinite() << std::endl;
                        std::cout << "  cluster_j->Psi_ddot_ finite: " << cluster_j->Psi_ddot_.allFinite() << std::endl;
                        std::cout << "  cluster_j->Psi_dot_ finite: " << cluster_j->Psi_dot_.allFinite() << std::endl;
                    }
                }

                dtau_dq.block(ii,jj,cluster_i->num_velocities_,cluster_j->num_velocities_) = block_val;
                
                if (j < i)
                {
                    dtau_dq.block(jj,ii,cluster_j->num_velocities_,cluster_i->num_velocities_) = cluster_j->S().transpose() * t3;
                }
                else // j == i, diagonal block
                {
                    // Add the configuration-dependent term: contractT(S_q, F)
                    // Corresponds to MATLAB ID_derivatives.m line 72
                    auto S_q_i = cluster_i->joint_->getSq();
                    auto contract_result = contractSqTransposeWithVector(S_q_i, cluster_i->F_);

                    // Debug: Check for NaN (only for double type)
                    if constexpr (std::is_same_v<Scalar, double>) {
                        if (!contract_result.allFinite()) {
                            std::cout << "[DEBUG backward pass] contractSqTransposeWithVector returned NaN for cluster " << i << std::endl;
                            std::cout << "  F_ finite: " << cluster_i->F_.allFinite() << std::endl;
                            std::cout << "  S_q size: " << S_q_i.size() << std::endl;
                            for (size_t k = 0; k < S_q_i.size(); ++k) {
                                if (!S_q_i[k].allFinite()) {
                                    std::cout << "  S_q[" << k << "] contains NaN/Inf!" << std::endl;
                                }
                            }
                        }
                    }

                    dtau_dq.block(ii,ii,cluster_i->num_velocities_,cluster_i->num_velocities_) += contract_result;
                }

                dtau_dq_dot.block(jj,ii,cluster_j->num_velocities_,cluster_i->num_velocities_) = cluster_j->S().transpose() * t2;
                dtau_dq_dot.block(ii,jj,cluster_i->num_velocities_,cluster_j->num_velocities_) =
                t1.transpose() * cluster_j->Upsilon_dot_ + t4.transpose() * cluster_j->S();

                if (cluster_j->parent_index_ >= 0)
                {
                    t1 = cluster_j->Xup_.inverseTransformForceSubspace(t1);
                    t2 = cluster_j->Xup_.inverseTransformForceSubspace(t2);
                    t3 = cluster_j->Xup_.inverseTransformForceSubspace(t3);
                    t4 = cluster_j->Xup_.inverseTransformForceSubspace(t4);
                }
                j = cluster_j->parent_index_;
            }
            if (cluster_i->parent_index_ >= 0)
            {
                auto &parent_cluster = cluster_nodes_[cluster_i->parent_index_];

                const auto X = cluster_i->Xup_.toMatrix();

#ifdef GRBDA_DEBUG_DERIVATIVES
                std::cout << "\n[DEBUG] Accumulating from body " << i << " to parent " << cluster_i->parent_index_ << "\n";

                // For body 2 (leaf), print M_cup to verify it's just the single-body inertia
                if (i == 2) {
                    std::cout << "  Body 2 M_cup (should be single-body, mass=1.0):\n";
                    std::cout << "    Mass ([3,3]): " << cluster_i->M_cup_(3,3) << " (expect 1.0)\n";
                    std::cout << "    Rotational inertia ([0,0]): " << cluster_i->M_cup_(0,0) << " (expect 0.0025)\n";

                    std::cout << "\n  Xup matrix for body 2 (FULL 6x6):\n";
                    for (int row = 0; row < 6; row++) {
                        std::cout << "    [";
                        for (int col = 0; col < 6; col++) {
                            std::cout << std::setw(10) << std::setprecision(4) << std::scientific << X(row, col);
                            if (col < 5) std::cout << ", ";
                        }
                        std::cout << "]\n";
                    }

                    std::cout << "\n  C++ Xup translation vector r:\n";
                    auto r_vec = cluster_i->Xup_[0].getTranslation();
                    std::cout << "    r = [" << r_vec(0) << ", " << r_vec(1) << ", " << r_vec(2) << "]\n";

                    std::cout << "\n  C++ M_cup[2] BEFORE transform (FULL 6x6):\n";
                    for (int row = 0; row < 6; row++) {
                        std::cout << "    [";
                        for (int col = 0; col < 6; col++) {
                            std::cout << std::setw(10) << std::setprecision(4) << std::scientific << cluster_i->M_cup_(row, col);
                            if (col < 5) std::cout << ", ";
                        }
                        std::cout << "]\n";
                    }

                    auto M_child_transformed_debug = (X.transpose() * cluster_i->M_cup_ * X).eval();
                    std::cout << "\n  Transformed M_cup[2] (X^T * M * X) first 3 rows:\n";
                    for (int row = 0; row < 3; row++) {
                        std::cout << "    [";
                        for (int col = 0; col < 6; col++) {
                            std::cout << std::setw(12) << std::setprecision(6) << std::scientific << M_child_transformed_debug(row, col);
                            if (col < 5) std::cout << ", ";
                        }
                        std::cout << "]\n";
                    }
                    std::cout << "  Expected MATLAB Xup{3}' * IC{3} * Xup{3} first row:\n";
                    std::cout << "    [1.087e-01, -2.188e-01, -3.790e-01, 0, -2.449e-01, 1.414e-01]\n";
                }

                // For body 1, print full Xup and M_cup to compare with MATLAB
                if (i == 1) {
                    std::cout << "  Xup matrix for body " << i << " (first 3 rows):\n";
                    for (int row = 0; row < 3; row++) {
                        std::cout << "    [";
                        for (int col = 0; col < 6; col++) {
                            std::cout << std::setw(10) << std::setprecision(4) << std::scientific << X(row, col);
                            if (col < 5) std::cout << ", ";
                        }
                        std::cout << "]\n";
                    }
                    std::cout << "\n  Child M_cup[1] (6x6) FULL MATRIX:\n";
                    for (int row = 0; row < 6; row++) {
                        std::cout << "    [";
                        for (int col = 0; col < 6; col++) {
                            std::cout << std::setw(12) << std::setprecision(6) << std::scientific << cluster_i->M_cup_(row, col);
                            if (col < 5) std::cout << ", ";
                        }
                        std::cout << "]\n";
                    }

                    std::cout << "\n  Transformed M (X^T * M_cup[1] * X) FULL MATRIX:\n";
                    auto M_trans_full = (X.transpose() * cluster_i->M_cup_ * X).eval();
                    for (int row = 0; row < 6; row++) {
                        std::cout << "    [";
                        for (int col = 0; col < 6; col++) {
                            std::cout << std::setw(12) << std::setprecision(6) << std::scientific << M_trans_full(row, col);
                            if (col < 5) std::cout << ", ";
                        }
                        std::cout << "]\n";
                    }
                }

                std::cout << "  Child M_cup[" << i << "][0,0] = " << cluster_i->M_cup_(0,0) << "\n";
                std::cout << "  Parent M_cup[" << cluster_i->parent_index_ << "][0,0] (before) = " << parent_cluster->M_cup_(0,0) << "\n";

                auto M_transformed = X.transpose() * cluster_i->M_cup_ * X;
                std::cout << "  X^T * M_cup[" << i << "] * X [0,0] = " << M_transformed(0,0) << "\n";
#endif

                // Use X^T * M * X formula for spatial inertia (matching MATLAB)
                auto M_child_transformed = (X.transpose() * cluster_i->M_cup_ * X).eval();
                auto B_child_transformed = (X.transpose() * cluster_i->B_cup_ * X).eval();
                auto F_child_transformed = cluster_i->Xup_.inverseTransformForceVector(cluster_i->F_);

                parent_cluster->M_cup_ += M_child_transformed;
                parent_cluster->B_cup_ += B_child_transformed;
                parent_cluster->F_     += F_child_transformed;

#ifdef GRBDA_DEBUG_DERIVATIVES
                std::cout << "  Parent M_cup[" << cluster_i->parent_index_ << "][0,0] (after) = " << parent_cluster->M_cup_(0,0) << "\n";

                // For floating base (parent index 0), print full M_cup after final accumulation
                if (cluster_i->parent_index_ == 0 && i == 1) {
                    std::cout << "\n[DEBUG] M_cup for Body 1 (floating base) AFTER full accumulation:\n";
                    std::cout << "  First column: ";
                    for (int row = 0; row < 6; row++) {
                        std::cout << parent_cluster->M_cup_(row, 0) << " ";
                    }
                    std::cout << "\n  MATLAB expected IC{1} first column: 1.6972 -0.9437 -2.5452 0.0 -1.4635 0.5622\n";
                }
#endif
            }
        }
        return {dtau_dq, dtau_dq_dot};
    }
    

    template class ClusterTreeModel<double>;
    template class ClusterTreeModel<std::complex<double>>;
    template class ClusterTreeModel<float>;     
    template class ClusterTreeModel<casadi::SX>;

} // namespace grbda
