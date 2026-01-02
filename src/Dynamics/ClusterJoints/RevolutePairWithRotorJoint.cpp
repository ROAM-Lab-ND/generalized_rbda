#include "grbda/Dynamics/ClusterJoints/RevolutePairWithRotorJoint.h"
#include "grbda/Utils/CasadiDerivatives.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar>
        RevolutePairWithRotor<Scalar>::RevolutePairWithRotor(
            ProximalTransmission &module_1, DistalTransmission &module_2)
            : Base<Scalar>(4, 2, 2),
              link1_(module_1.body_), link2_(module_2.body_),
              rotor1_(module_1.rotor_), rotor2_(module_2.rotor_),
              link1_index_(link1_.sub_index_within_cluster_),
              link2_index_(link2_.sub_index_within_cluster_),
              rotor1_index_(rotor1_.sub_index_within_cluster_),
              rotor2_index_(rotor2_.sub_index_within_cluster_),
              axis1_(module_1.joint_axis_),
              axis2_(module_2.joint_axis_),
              X_tree_internal_(module_2.body_.Xtree_),
              q_cache_(2),
              qd_cache_(2),
              casadi_functions_initialized_(false)
        {
            using Rev = Joints::Revolute<Scalar>;

            link1_joint_ = this->single_joints_.emplace_back(new Rev(module_1.joint_axis_));
            rotor1_joint_ = this->single_joints_.emplace_back(new Rev(module_1.rotor_axis_));
            rotor2_joint_ = this->single_joints_.emplace_back(new Rev(module_2.rotor_axis_));
            link2_joint_ = this->single_joints_.emplace_back(new Rev(module_2.joint_axis_));

            this->spanning_tree_to_independent_coords_conversion_ = DMat<int>::Zero(2, 4);
            this->spanning_tree_to_independent_coords_conversion_(0, link1_index_) = 1;
            this->spanning_tree_to_independent_coords_conversion_(1, link2_index_) = 1;

            Vec2<Scalar> gear_ratios{module_1.gear_ratio_, module_2.gear_ratio_};
            Eigen::DiagonalMatrix<Scalar, 2> rotor_matrix(gear_ratios);
            Mat2<Scalar> belt_matrix;
            belt_matrix << beltMatrixRowFromBeltRatios(module_1.belt_ratios_), 0,
                beltMatrixRowFromBeltRatios(module_2.belt_ratios_);
            Mat2<Scalar> ratio_product = rotor_matrix * belt_matrix;

            DMat<Scalar> G = DMat<Scalar>::Zero(4, 2);
            G(link1_index_, 0) = 1.;
            G(rotor1_index_, 0) = ratio_product(0, 0);
            G(rotor2_index_, 0) = ratio_product(1, 0);
            G(rotor2_index_, 1) = ratio_product(1, 1);
            G(link2_index_, 1) = 1.;

            DMat<Scalar> K = DMat<Scalar>::Zero(2, 4);
            int cnstr1_index = rotor1_index_ > rotor2_index_;
            int cnstr2_index = rotor2_index_ > rotor1_index_;
            K(cnstr1_index, rotor1_index_) = -1.;
            K(cnstr1_index, link1_index_) = G(rotor1_index_, 0);
            K(cnstr2_index, rotor2_index_) = -1.;
            K(cnstr2_index, link1_index_) = G(rotor2_index_, 0);
            K(cnstr2_index, link2_index_) = G(rotor2_index_, 1);

            this->loop_constraint_ = std::make_shared<LoopConstraint::Static<Scalar>>(G, K);

            X_intra_S_span_ = DMat<Scalar>::Zero(24, 4);
            X_intra_S_span_ring_ = DMat<Scalar>::Zero(24, 4);

            const DMat<Scalar> &link1_S = link1_joint_->S();
            X_intra_S_span_.template block<6, 1>(6 * link1_index_, link1_index_) = link1_S;
            const DMat<Scalar> &rotor1_S = rotor1_joint_->S();
            X_intra_S_span_.template block<6, 1>(6 * rotor1_index_, rotor1_index_) = rotor1_S;
            const DMat<Scalar> &rotor2_S = rotor2_joint_->S();
            X_intra_S_span_.template block<6, 1>(6 * rotor2_index_, rotor2_index_) = rotor2_S;
            const DMat<Scalar> &link2_S = link2_joint_->S();
            X_intra_S_span_.template block<6, 1>(6 * link2_index_, link2_index_) = link2_S;

            this->S_ = X_intra_S_span_ * this->loop_constraint_->G();
        }

        template <typename Scalar>
        void RevolutePairWithRotor<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            const DVec<Scalar> &q = spanning_joint_state.position;
            const DVec<Scalar> &qd = spanning_joint_state.velocity;

            // Cache INDEPENDENT coordinates for derivative methods (not spanning tree!)
            q_cache_ = joint_state.position;
            qd_cache_ = joint_state.velocity;

            link1_joint_->updateKinematics(q.template segment<1>(link1_index_),
                                           qd.template segment<1>(link1_index_));
            rotor1_joint_->updateKinematics(q.template segment<1>(rotor1_index_),
                                            qd.template segment<1>(rotor1_index_));
            rotor2_joint_->updateKinematics(q.template segment<1>(rotor2_index_),
                                            qd.template segment<1>(rotor2_index_));
            link2_joint_->updateKinematics(q.template segment<1>(link2_index_),
                                           qd.template segment<1>(link2_index_));

            X21_ = link2_joint_->XJ() * link2_.Xtree_;

            const DVec<Scalar> v2_relative = link2_joint_->S() * qd[link2_index_];

            X_intra_S_span_.template block<6, 1>(6 * link2_index_, link1_index_) =
                X21_.transformMotionSubspace(link1_joint_->S());

            // CRITICAL FIX: Recompute entire S matrix after updating X_intra_S_span_
            // Both columns of S depend on X_intra_S_span_, not just column 0!
            // S = X_intra_S_span * G where G is the loop constraint matrix
            this->S_ = X_intra_S_span_ * this->loop_constraint_->G();

            X_intra_S_span_ring_.template block<6, 1>(6 * link2_index_, link1_index_) =
                -spatial::generalMotionCrossMatrix(v2_relative) *
                X_intra_S_span_.template block<6, 1>(6 * link2_index_, link1_index_);

            this->vJ_ = X_intra_S_span_ * qd;
            this->cJ_ = X_intra_S_span_ring_ * qd;
            this->S_ring_ = X_intra_S_span_ring_ * this->loop_constraint_->G(); //+X_intra*S_span_*G_dot_;
        }

        template <typename Scalar>
        void RevolutePairWithRotor<Scalar>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 4)
                throw std::runtime_error("[RevolutePairWithRotor] Xup must have 24 rows");
#endif

            Xup[link1_index_] = link1_joint_->XJ() * link1_.Xtree_;
            Xup[rotor1_index_] = rotor1_joint_->XJ() * rotor1_.Xtree_;
            Xup[rotor2_index_] = rotor2_joint_->XJ() * rotor2_.Xtree_;
            Xup[link2_index_] = link2_joint_->XJ() * link2_.Xtree_ * Xup[link1_index_];
        }

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        RevolutePairWithRotor<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> bodies_joints_and_ref_inertias;

            DMat<Scalar> S_dependent_1 = this->S_.template middleRows<6>(6 * rotor1_index_);
            Mat6<Scalar> Ir1 = rotor1_.inertia_.getMatrix();
            DMat<Scalar> ref_inertia_1 = S_dependent_1.transpose() * Ir1 * S_dependent_1;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(link1_, link1_joint_,
                                                                      ref_inertia_1));

            DMat<Scalar> S_dependent_2 = this->S_.template middleRows<6>(6 * rotor2_index_);
            Mat6<Scalar> Ir2 = rotor2_.inertia_.getMatrix();
            DMat<Scalar> ref_inertia_2 = S_dependent_2.transpose() * Ir2 * S_dependent_2;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(link2_, link2_joint_,
                                                                      ref_inertia_2));

            return bodies_joints_and_ref_inertias;
        }

        template <typename Scalar>
        char RevolutePairWithRotor<Scalar>::axisToChar(ori::CoordinateAxis axis) const
        {
            switch (axis)
            {
            case ori::CoordinateAxis::X:
                return 'X';
            case ori::CoordinateAxis::Y:
                return 'Y';
            case ori::CoordinateAxis::Z:
                return 'Z';
            default:
                throw std::runtime_error("Unknown axis");
            }
        }

        template <typename Scalar>
        void RevolutePairWithRotor<Scalar>::initializeCasadiFunctions() const
        {
            using namespace casadi;
            using namespace casadi_derivatives;

            // Symbolic variables for joint positions and velocities
            SX q1 = SX::sym("q1");
            SX q2 = SX::sym("q2");
            SX qd1 = SX::sym("qd1");
            SX qd2 = SX::sym("qd2");

            // Joint axes
            char ax1 = axisToChar(axis1_);
            char ax2 = axisToChar(axis2_);

            // Motion subspaces for the revolute joints
            SX S1 = revoluteMotionSubspace(ax1);
            SX S2 = revoluteMotionSubspace(ax2);

            // Spatial rotation transforms
            // NOTE: q1 and q2 are INDEPENDENT coordinates, not spanning tree coordinates
            // For RevolutePairWithRotor, the spanning tree has: link1, rotor1, rotor2, link2
            // The mapping is: q_spanning[link1] = q1, q_spanning[link2] = q2 (from G matrix)
            //
            // In updateKinematics, we compute: X21 = spatialRotation(ax2, q_spanning[link2]) * Xtree
            // Since q_spanning[link2] = q2, we have: X21 = spatialRotation(ax2, q2) * Xtree
            SX XJ1 = spatialRotation(ax1, q1);
            SX XJ2 = spatialRotation(ax2, q2);

            // CRITICAL: When Xtree has a translation, we need to account for matrix multiplication properly
            // The formula is: X_intra[link2, link1] = XJ2 * Xtree * S1
            // where XJ2 depends on q2, and Xtree is a 6x6 constant matrix
            //
            // The derivative is: ∂(XJ2 * Xtree * S1)/∂q2
            //
            // Method: Embed Xtree as a numeric DM matrix (6x6), compute jacobian of XJ2*Xtree
            // with respect to each element, then multiply result by S1
            //
            // Note: We can't use SX(DM) for the full Xtree*S1 product because CasADi loses symbolic
            // dependency. Instead, we embed Xtree as DM and let CasADi differentiate XJ2.

            // Convert X_tree_internal to CasADi DM
            DMat<double> X_internal_eigen = X_tree_internal_.toMatrix().template cast<double>();

            std::vector<double> X_internal_vec(36);
            // Fill in row-major order (Eigen convention)
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    X_internal_vec[i * 6 + j] = X_internal_eigen(i, j);
                }
            }
            DM X_internal_dm = DM(reshape(DM(X_internal_vec), 6, 6));

            // Compute S1 as DM
            DM S1_dm = DM::zeros(6, 1);
            if (ax1 == 'X' || ax1 == 'x') S1_dm(0) = 1.0;
            else if (ax1 == 'Y' || ax1 == 'y') S1_dm(1) = 1.0;
            else if (ax1 == 'Z' || ax1 == 'z') S1_dm(2) = 1.0;

            // Pre-compute constant vector v = Xtree * S1
            DM constant_vec = mtimes(X_internal_dm, S1_dm);
            constant_vec_ = constant_vec;

            // CRITICAL: Compute the full symbolic expression XJ2 * Xtree * S1 first,
            // THEN take derivatives. This ensures correct jacobian computation.

            // Convert Xtree to symbolic SX (not DM) so derivatives work correctly
            SX X_tree_sx = SX::zeros(6, 6);
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    X_tree_sx(i, j) = X_internal_eigen(i, j);
                }
            }

            // Convert S1 to symbolic SX
            SX S1_sx = SX::zeros(6, 1);
            if (ax1 == 'X' || ax1 == 'x') S1_sx(0) = 1.0;
            else if (ax1 == 'Y' || ax1 == 'y') S1_sx(1) = 1.0;
            else if (ax1 == 'Z' || ax1 == 'z') S1_sx(2) = 1.0;

            // Compute the full expression: result = XJ2 * Xtree * S1
            SX intermediate = mtimes(X_tree_sx, S1_sx);
            SX result = mtimes(XJ2, intermediate);  // 6x1 vector

            // Now compute jacobians of the final 6x1 vector
            SX dResult_dq1 = jacobian(result, q1);  // 6x1 jacobian
            SX dResult_dq2 = jacobian(result, q2);  // 6x1 jacobian

            // Create Functions
            f_dS_dq1_ = Function("dS_dq1", {q1, q2}, {dResult_dq1});
            f_dS_dq2_ = Function("dS_dq2", {q1, q2}, {dResult_dq2});

            // Also store constant_vec for reference (though not used in new approach)
            constant_vec_ = constant_vec;

            // For Sdotqd_qd: the time derivative Sdot*qd
            // Ṡ[:,0] = ∂result/∂q1 * q̇1 + ∂result/∂q2 * q̇2
            // where result = XJ2 * Xtree * S1 (already computed above)
            SX Sdot_col0 = dResult_dq1 * qd1 + dResult_dq2 * qd2;
            SX Sdotqd = Sdot_col0 * qd1;

            // Compute ∂(Ṡqd)/∂qd using CasADi's automatic differentiation
            SX dSdotqd_dqd1 = jacobian(Sdotqd, qd1);
            SX dSdotqd_dqd2 = jacobian(Sdotqd, qd2);

            f_Sdotqd_q_ = Function("Sdotqd_q", {q1, q2, qd1, qd2}, {SX::zeros(6, 2)});  // No q-dependence in second-order term
            f_Sdotqd_qd_ = Function("Sdotqd_qd", {q1, q2, qd1, qd2}, {horzcat(dSdotqd_dqd1, dSdotqd_dqd2)});

            casadi_functions_initialized_ = true;
        }

        template <typename Scalar>
        std::vector<DMat<Scalar>> RevolutePairWithRotor<Scalar>::getSq() const
        {
            if (!casadi_functions_initialized_)
            {
                initializeCasadiFunctions();
            }

            const int nv = 2;
            const int spatial_dim = 24; // 4 bodies * 6 DOF
            std::vector<DMat<Scalar>> S_q(nv);

            // Get q from cache (set by updateKinematics)
            const DVec<Scalar> &q = q_cache_;

            // Evaluate CasADi functions
            std::vector<casadi::DM> input = {
                casadi::DM(static_cast<double>(q(0))),
                casadi::DM(static_cast<double>(q(1)))
            };

            std::vector<casadi::DM> dS_dq1_result = f_dS_dq1_(input);
            std::vector<casadi::DM> dS_dq2_result = f_dS_dq2_(input);

            // The functions now return the full derivative vectors directly (6x1)
            casadi::DM dS_link2_col0_dq1 = dS_dq1_result[0];  // 6x1 vector
            casadi::DM dS_link2_col0_dq2 = dS_dq2_result[0];  // 6x1 vector

            // Convert to Eigen matrices
            S_q[0] = DMat<Scalar>::Zero(spatial_dim, nv);
            S_q[1] = DMat<Scalar>::Zero(spatial_dim, nv);


            // CRITICAL FIX: S = X_intra_S_span * G, so ∂S/∂qi = (∂X_intra_S_span/∂qi) * G
            // The CasADi function returns ∂(X_intra_S_span[link2, link1])/∂qi (a 6x1 vector)
            // We need to compute the full derivative by multiplying by G

            // Create ∂X_intra_S_span/∂qi (24x4 matrix, mostly zero)
            DMat<Scalar> dX_intra_dq1 = DMat<Scalar>::Zero(24, 4);
            DMat<Scalar> dX_intra_dq2 = DMat<Scalar>::Zero(24, 4);

            // Only the [link2, link1] block is non-zero (6 rows, 1 column)
            for (int i = 0; i < 6; i++)
            {
                dX_intra_dq1(6 * link2_index_ + i, link1_index_) = static_cast<Scalar>(
                    static_cast<double>(dS_link2_col0_dq1(i)));
                dX_intra_dq2(6 * link2_index_ + i, link1_index_) = static_cast<Scalar>(
                    static_cast<double>(dS_link2_col0_dq2(i)));
            }

            // Now compute ∂S/∂qi = (∂X_intra_S_span/∂qi) * G
            // This gives us derivatives for BOTH columns of S, not just column 0
            const DMat<Scalar> &G = this->loop_constraint_->G();
            S_q[0] = dX_intra_dq1 * G;  // 24x2 matrix
            S_q[1] = dX_intra_dq2 * G;  // 24x2 matrix


            return S_q;
        }

        template <typename Scalar>
        DMat<Scalar> RevolutePairWithRotor<Scalar>::getSdotqd_q() const
        {
            if (!casadi_functions_initialized_)
            {
                initializeCasadiFunctions();
            }

            const DVec<Scalar> &q = q_cache_;
            const DVec<Scalar> &qd = qd_cache_;

            std::vector<casadi::DM> input = {
                casadi::DM(static_cast<double>(q(0))),
                casadi::DM(static_cast<double>(q(1))),
                casadi::DM(static_cast<double>(qd(0))),
                casadi::DM(static_cast<double>(qd(1)))
            };

            std::vector<casadi::DM> result = f_Sdotqd_q_(input);
            casadi::DM Sdotqd_q = result[0];  // 6x2 matrix (only link2 rows)

            DMat<Scalar> output = DMat<Scalar>::Zero(24, 2);
            // No q-dependence in second-order term, returns zeros
            return output;
        }

        template <typename Scalar>
        DMat<Scalar> RevolutePairWithRotor<Scalar>::getSdotqd_qd() const
        {
            if (!casadi_functions_initialized_)
            {
                initializeCasadiFunctions();
            }

            const DVec<Scalar> &q = q_cache_;
            const DVec<Scalar> &qd = qd_cache_;

            std::vector<casadi::DM> input = {
                casadi::DM(static_cast<double>(q(0))),
                casadi::DM(static_cast<double>(q(1))),
                casadi::DM(static_cast<double>(qd(0))),
                casadi::DM(static_cast<double>(qd(1)))
            };

            std::vector<casadi::DM> result = f_Sdotqd_qd_(input);
            casadi::DM Sdotqd_qd = result[0];  // 6x2 matrix (link2 rows only)

            DMat<Scalar> output = DMat<Scalar>::Zero(24, 2);
            // Fill in rows corresponding to link2
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    output(6 * link2_index_ + i, j) = static_cast<Scalar>(
                        static_cast<double>(Sdotqd_qd(i, j)));
                }
            }

            return output;
        }

        template class RevolutePairWithRotor<double>;

        // Complex-step specialization: derivative methods return zero since
        // complex-step doesn't use analytical derivatives
        template <>
        void RevolutePairWithRotor<std::complex<double>>::initializeCasadiFunctions() const
        {
            casadi_functions_initialized_ = true;
        }

        template <>
        std::vector<DMat<std::complex<double>>>
        RevolutePairWithRotor<std::complex<double>>::getSq() const
        {
            return std::vector<DMat<std::complex<double>>>(2, DMat<std::complex<double>>::Zero(24, 2));
        }

        template <>
        DMat<std::complex<double>>
        RevolutePairWithRotor<std::complex<double>>::getSdotqd_q() const
        {
            return DMat<std::complex<double>>::Zero(24, 2);
        }

        template <>
        DMat<std::complex<double>>
        RevolutePairWithRotor<std::complex<double>>::getSdotqd_qd() const
        {
            return DMat<std::complex<double>>::Zero(24, 2);
        }

        template class RevolutePairWithRotor<std::complex<double>>;
        template class RevolutePairWithRotor<float>;
        template class RevolutePairWithRotor<casadi::SX>;

    }

} // namespace grbda
