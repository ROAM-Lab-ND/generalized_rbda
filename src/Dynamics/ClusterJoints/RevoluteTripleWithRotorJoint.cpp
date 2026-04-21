#include "grbda/Dynamics/ClusterJoints/RevoluteTripleWithRotorJoint.h"
#include "grbda/Utils/CasadiDerivatives.h"
#include "grbda/Utils/IDDerivProfile.h"

#include <chrono>

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar>
        RevoluteTripleWithRotor<Scalar>::RevoluteTripleWithRotor(
            const ProximalTransmission &module_1,
            const IntermediateTransmission &module_2,
            const DistalTransmission &module_3)
            : Base<Scalar>(6, 3, 3), link_1_(module_1.body_), link_2_(module_2.body_),
              link_3_(module_3.body_), rotor_1_(module_1.rotor_), rotor_2_(module_2.rotor_),
              rotor_3_(module_3.rotor_),
              axis1_(module_1.joint_axis_), axis2_(module_2.joint_axis_), axis3_(module_3.joint_axis_),
              X_tree_2_(module_2.body_.Xtree_), X_tree_3_(module_3.body_.Xtree_)
        {
            using Rev = Joints::Revolute<Scalar>;

            link_1_joint_ = this->single_joints_.emplace_back(new Rev(module_1.joint_axis_));
            link_2_joint_ = this->single_joints_.emplace_back(new Rev(module_2.joint_axis_));
            link_3_joint_ = this->single_joints_.emplace_back(new Rev(module_3.joint_axis_));

            rotor_1_joint_ = this->single_joints_.emplace_back(new Rev(module_1.rotor_axis_));
            rotor_2_joint_ = this->single_joints_.emplace_back(new Rev(module_2.rotor_axis_));
            rotor_3_joint_ = this->single_joints_.emplace_back(new Rev(module_3.rotor_axis_));

            this->spanning_tree_to_independent_coords_conversion_ = DMat<int>::Zero(3, 6);
            this->spanning_tree_to_independent_coords_conversion_.template topLeftCorner<3, 3>().setIdentity();

            Vec3<Scalar> gear_ratios{module_1.gear_ratio_, module_2.gear_ratio_, module_3.gear_ratio_};
            Eigen::DiagonalMatrix<Scalar, 3> rotor_matrix(gear_ratios);

            DMat<Scalar> belt_matrix = DMat<Scalar>::Zero(3, 3);
            belt_matrix << beltMatrixRowFromBeltRatios(module_1.belt_ratios_), 0., 0.,
                beltMatrixRowFromBeltRatios(module_2.belt_ratios_), 0.,
                beltMatrixRowFromBeltRatios(module_3.belt_ratios_);

            DMat<Scalar> G = DMat<Scalar>::Zero(6, 3);
            G.template topRows<3>().setIdentity();
            G.template bottomRows<3>() = rotor_matrix * belt_matrix;

            DMat<Scalar> K = DMat<Scalar>::Zero(3, 6);
            K.template leftCols(3) = -G.bottomRows(3);
            K.template rightCols(3).setIdentity();
            this->loop_constraint_ = std::make_shared<LoopConstraint::Static<Scalar>>(G, K);

            X_intra_S_span_ = DMat<Scalar>::Zero(36, 6);
            X_intra_S_span_ring_ = DMat<Scalar>::Zero(36, 6);

            X_intra_S_span_.template block<6, 1>(0, 0) = link_1_joint_->S();
            X_intra_S_span_.template block<6, 1>(6, 1) = link_2_joint_->S();
            X_intra_S_span_.template block<6, 1>(12, 2) = link_3_joint_->S();
            X_intra_S_span_.template block<6, 1>(18, 3) = rotor_1_joint_->S();
            X_intra_S_span_.template block<6, 1>(24, 4) = rotor_2_joint_->S();
            X_intra_S_span_.template block<6, 1>(30, 5) = rotor_3_joint_->S();

            this->S_ = X_intra_S_span_ * this->loop_constraint_->G();
        }

        template <typename Scalar>
        void RevoluteTripleWithRotor<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            const DVec<Scalar> &q = spanning_joint_state.position;
            const DVec<Scalar> &qd = spanning_joint_state.velocity;

            // Cache INDEPENDENT coordinates for derivative methods
            q_cache_ = joint_state.position;
            qd_cache_ = joint_state.velocity;
            S_q_cache_valid_ = false;  // state changed, invalidate derivative cache

            link_1_joint_->updateKinematics(q.template segment<1>(0), qd.template segment<1>(0));
            link_2_joint_->updateKinematics(q.template segment<1>(1), qd.template segment<1>(1));
            link_3_joint_->updateKinematics(q.template segment<1>(2), qd.template segment<1>(2));
            rotor_1_joint_->updateKinematics(q.template segment<1>(3), qd.template segment<1>(3));
            rotor_2_joint_->updateKinematics(q.template segment<1>(4), qd.template segment<1>(4));
            rotor_3_joint_->updateKinematics(q.template segment<1>(5), qd.template segment<1>(5));

            X21_ = link_2_joint_->XJ() * link_2_.Xtree_;
            X32_ = link_3_joint_->XJ() * link_3_.Xtree_;
            X31_ = X32_ * X21_;

            const DVec<Scalar> v2_relative1 = link_2_joint_->S() * qd[1];
            const DMat<Scalar> X21_S1 = X21_.transformMotionSubspace(link_1_joint_->S());
            const DVec<Scalar> v3_relative1 = X32_.transformMotionVector(v2_relative1) +
                                              link_3_joint_->S() * qd[2];
            const DMat<Scalar> X31_S1 = X31_.transformMotionSubspace(link_1_joint_->S());
            const DVec<Scalar> v3_relative2 = link_3_joint_->S() * qd[2];
            const DMat<Scalar> X32_S2 = X32_.transformMotionSubspace(link_2_joint_->S());

            X_intra_S_span_.template block<6, 1>(6, 0) = X21_S1;
            X_intra_S_span_.template block<6, 1>(12, 0) = X31_S1;
            X_intra_S_span_.template block<6, 1>(12, 1) = X32_S2;

            this->S_.template topLeftCorner<18, 3>() =
                X_intra_S_span_.template topLeftCorner<18, 3>();

            X_intra_S_span_ring_.template block<6, 1>(6, 0) =
                -spatial::generalMotionCrossMatrix(v2_relative1) * X21_S1;
            X_intra_S_span_ring_.template block<6, 1>(12, 0) =
                -spatial::generalMotionCrossMatrix(v3_relative1) * X31_S1;
            X_intra_S_span_ring_.template block<6, 1>(12, 1) =
                -spatial::generalMotionCrossMatrix(v3_relative2) * X32_S2;

            this->vJ_ = X_intra_S_span_ * qd;
            this->cJ_ = X_intra_S_span_ring_ * qd;
            this->S_ring_ = X_intra_S_span_ring_ * this->loop_constraint_->G(); //+X_intra*S_span_*G_dot_;
        }

        template <typename Scalar>
        void RevoluteTripleWithRotor<Scalar>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 6)
                throw std::runtime_error("[RevoluteTripleWithRotor] Xup must have 36 rows");
#endif

            Xup[0] = link_1_joint_->XJ() * link_1_.Xtree_;
            Xup[1] = X21_ * Xup[0];
            Xup[2] = X31_ * Xup[0];
            Xup[3] = rotor_1_joint_->XJ() * rotor_1_.Xtree_;
            Xup[4] = rotor_2_joint_->XJ() * rotor_2_.Xtree_;
            Xup[5] = rotor_3_joint_->XJ() * rotor_3_.Xtree_;
        }

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        RevoluteTripleWithRotor<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> bodies_joints_and_ref_inertias;

            const DMat<Scalar> S_dependent_1 = this->S_.template middleRows<6>(18);
            const Mat6<Scalar> Ir1 = rotor_1_.inertia_.getMatrix();
            const DMat<Scalar> ref_inertia_1 = S_dependent_1.transpose() * Ir1 * S_dependent_1;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(link_1_, link_1_joint_,
                                                                     ref_inertia_1));

            const DMat<Scalar> S_dependent_2 = this->S_.template middleRows<6>(24);
            const Mat6<Scalar> Ir2 = rotor_2_.inertia_.getMatrix();
            const DMat<Scalar> ref_inertia_2 = S_dependent_2.transpose() * Ir2 * S_dependent_2;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(link_2_, link_2_joint_,
                                                                     ref_inertia_2));

            const DMat<Scalar> S_dependent_3 = this->S_.template middleRows<6>(30);
            const Mat6<Scalar> Ir3 = rotor_3_.inertia_.getMatrix();
            const DMat<Scalar> ref_inertia_3 = S_dependent_3.transpose() * Ir3 * S_dependent_3;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(link_3_, link_3_joint_,
                                                                     ref_inertia_3));

            return bodies_joints_and_ref_inertias;
        }

        template <typename Scalar>
        char RevoluteTripleWithRotor<Scalar>::axisToChar(ori::CoordinateAxis axis) const
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
        void RevoluteTripleWithRotor<Scalar>::initializeCasadiFunctions() const
        {
            if (casadi_functions_initialized_) return;

            using namespace casadi;
            using namespace casadi_derivatives;

            SX q1 = SX::sym("q1");
            SX q2 = SX::sym("q2");
            SX q3 = SX::sym("q3");
            SX qd1 = SX::sym("qd1");
            SX qd2 = SX::sym("qd2");
            SX qd3 = SX::sym("qd3");

            char ax1 = axisToChar(axis1_);
            char ax2 = axisToChar(axis2_);
            char ax3 = axisToChar(axis3_);

            SX S1 = revoluteMotionSubspace(ax1);
            SX S2 = revoluteMotionSubspace(ax2);
            SX S3 = revoluteMotionSubspace(ax3);

            // Convert Xtree matrices to CasADi SX
            DMat<double> Xtree2_eigen = X_tree_2_.toMatrix().template cast<double>();
            DMat<double> Xtree3_eigen = X_tree_3_.toMatrix().template cast<double>();

            SX Xtree2_sx = SX::zeros(6, 6);
            SX Xtree3_sx = SX::zeros(6, 6);

            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    Xtree2_sx(i, j) = Xtree2_eigen(i, j);
                    Xtree3_sx(i, j) = Xtree3_eigen(i, j);
                }
            }

            // Compute spatial rotations
            SX XJ2 = spatialRotation(ax2, q2);
            SX XJ3 = spatialRotation(ax3, q3);

            // Compute X21 = XJ2 * Xtree2
            SX X21 = mtimes(XJ2, Xtree2_sx);

            // Compute X32 = XJ3 * Xtree3
            SX X32 = mtimes(XJ3, Xtree3_sx);

            // Compute X31 = X32 * X21
            SX X31 = mtimes(X32, X21);

            // Motion subspaces for link2 and link3
            // Link2, column 0: X21 * S1
            SX S_link2_col0 = mtimes(X21, S1);

            // Link3, column 0: X31 * S1
            SX S_link3_col0 = mtimes(X31, S1);

            // Link3, column 1: X32 * S2
            SX S_link3_col1 = mtimes(X32, S2);

            // Compute jacobians for link2
            SX dS_link2_col0_dq1 = jacobian(S_link2_col0, q1);
            SX dS_link2_col0_dq2 = jacobian(S_link2_col0, q2);
            SX dS_link2_col0_dq3 = jacobian(S_link2_col0, q3);

            // Compute jacobians for link3 column 0
            SX dS_link3_col0_dq1 = jacobian(S_link3_col0, q1);
            SX dS_link3_col0_dq2 = jacobian(S_link3_col0, q2);
            SX dS_link3_col0_dq3 = jacobian(S_link3_col0, q3);

            // Compute jacobians for link3 column 1
            SX dS_link3_col1_dq1 = jacobian(S_link3_col1, q1);
            SX dS_link3_col1_dq2 = jacobian(S_link3_col1, q2);
            SX dS_link3_col1_dq3 = jacobian(S_link3_col1, q3);

            // Combine into output matrices
            // Link2 derivatives: 6x3 matrix for each q
            SX dS_link2_dq = horzcat(dS_link2_col0_dq1, dS_link2_col0_dq2, dS_link2_col0_dq3);

            // Link3 derivatives: 6x3 matrix for each q (combining both columns)
            SX dS_link3_col0_dq = horzcat(dS_link3_col0_dq1, dS_link3_col0_dq2, dS_link3_col0_dq3);
            SX dS_link3_col1_dq = horzcat(dS_link3_col1_dq1, dS_link3_col1_dq2, dS_link3_col1_dq3);

            // Create functions
            f_dS_link2_dq_ = Function("dS_link2_dq", {q1, q2, q3}, {dS_link2_dq});
            f_dS_link3_dq_ = Function("dS_link3_dq", {q1, q2, q3}, {dS_link3_col0_dq, dS_link3_col1_dq});

            // For Sdotqd derivatives
            // Ṡ = dS/dq1*q̇1 + dS/dq2*q̇2 + dS/dq3*q̇3
            // We need to track derivatives for all configuration-dependent terms

            // For Sdotqd derivatives
            // Ṡ = dS/dq1*q̇1 + dS/dq2*q̇2 + dS/dq3*q̇3
            // We need to compute Sdotqd for each link separately

            // Link2: Only has configuration-dependent column 0
            SX Sdot_link2_col0 = dS_link2_col0_dq1 * qd1 + dS_link2_col0_dq2 * qd2 + dS_link2_col0_dq3 * qd3;
            SX Sdotqd_link2 = Sdot_link2_col0 * qd1;

            // Link3: Has configuration-dependent columns 0 and 1
            SX Sdot_link3_col0 = dS_link3_col0_dq1 * qd1 + dS_link3_col0_dq2 * qd2 + dS_link3_col0_dq3 * qd3;
            SX Sdot_link3_col1 = dS_link3_col1_dq1 * qd1 + dS_link3_col1_dq2 * qd2 + dS_link3_col1_dq3 * qd3;
            SX Sdotqd_link3 = Sdot_link3_col0 * qd1 + Sdot_link3_col1 * qd2;

            // Compute ∂(Ṡqd)/∂q for each link separately
            SX dSdotqd_link2_dq1 = jacobian(Sdotqd_link2, q1);
            SX dSdotqd_link2_dq2 = jacobian(Sdotqd_link2, q2);
            SX dSdotqd_link2_dq3 = jacobian(Sdotqd_link2, q3);

            SX dSdotqd_link3_dq1 = jacobian(Sdotqd_link3, q1);
            SX dSdotqd_link3_dq2 = jacobian(Sdotqd_link3, q2);
            SX dSdotqd_link3_dq3 = jacobian(Sdotqd_link3, q3);

            // Compute ∂(Ṡqd)/∂qd for each link separately
            SX dSdotqd_link2_dqd1 = jacobian(Sdotqd_link2, qd1);
            SX dSdotqd_link2_dqd2 = jacobian(Sdotqd_link2, qd2);
            SX dSdotqd_link2_dqd3 = jacobian(Sdotqd_link2, qd3);

            SX dSdotqd_link3_dqd1 = jacobian(Sdotqd_link3, qd1);
            SX dSdotqd_link3_dqd2 = jacobian(Sdotqd_link3, qd2);
            SX dSdotqd_link3_dqd3 = jacobian(Sdotqd_link3, qd3);

            // Create functions that return separate results for link2 and link3
            f_Sdotqd_q_ = Function("Sdotqd_q", {q1, q2, q3, qd1, qd2, qd3},
                                   {horzcat(dSdotqd_link2_dq1, dSdotqd_link2_dq2, dSdotqd_link2_dq3),
                                    horzcat(dSdotqd_link3_dq1, dSdotqd_link3_dq2, dSdotqd_link3_dq3)});
            f_Sdotqd_qd_ = Function("Sdotqd_qd", {q1, q2, q3, qd1, qd2, qd3},
                                    {horzcat(dSdotqd_link2_dqd1, dSdotqd_link2_dqd2, dSdotqd_link2_dqd3),
                                     horzcat(dSdotqd_link3_dqd1, dSdotqd_link3_dqd2, dSdotqd_link3_dqd3)});

            casadi_functions_initialized_ = true;
        }

        template <typename Scalar>
        std::vector<DMat<Scalar>> RevoluteTripleWithRotor<Scalar>::getSq() const
        {
            const int nv = 3;
            const int spatial_dim = 36;

            // Return cached result if available and state unchanged
            if (S_q_cache_valid_ && (int)S_q_cache_.size() == nv) {
                return S_q_cache_;
            }

            initializeCasadiFunctions();

            std::vector<casadi::DM> input = {
                casadi::DM(static_cast<double>(q_cache_(0))),
                casadi::DM(static_cast<double>(q_cache_(1))),
                casadi::DM(static_cast<double>(q_cache_(2)))
            };

            const auto t_casadi_s_start = std::chrono::high_resolution_clock::now();
            auto res_link2 = f_dS_link2_dq_(input);
            auto res_link3 = f_dS_link3_dq_(input);
            const double casadi_s_us = std::chrono::duration<double, std::micro>(
                std::chrono::high_resolution_clock::now() - t_casadi_s_start).count();
            profiling::addCasadiSUs(casadi_s_us);

            casadi::DM dS_link2 = res_link2[0];  // 6x3 matrix
            casadi::DM dS_link3_col0 = res_link3[0];  // 6x3 matrix
            casadi::DM dS_link3_col1 = res_link3[1];  // 6x3 matrix

            // Create ∂X_intra_S_span/∂qi (36x6 matrix, mostly zero)
            std::vector<DMat<Scalar>> dX_intra_dq(nv);
            for (int i = 0; i < nv; ++i) {
                dX_intra_dq[i] = DMat<Scalar>::Zero(spatial_dim, 6);
            }

            // Fill in link2 derivatives (rows 6-11, column 0)
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < nv; ++j) {
                    dX_intra_dq[j](6 + i, 0) = static_cast<Scalar>(static_cast<double>(dS_link2(i, j)));
                }
            }

            // Fill in link3 column 0 derivatives (rows 12-17, column 0)
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < nv; ++j) {
                    dX_intra_dq[j](12 + i, 0) = static_cast<Scalar>(static_cast<double>(dS_link3_col0(i, j)));
                }
            }

            // Fill in link3 column 1 derivatives (rows 12-17, column 1)
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < nv; ++j) {
                    dX_intra_dq[j](12 + i, 1) = static_cast<Scalar>(static_cast<double>(dS_link3_col1(i, j)));
                }
            }

            // Compute ∂S/∂qi = (∂X_intra_S_span/∂qi) * G
            const DMat<Scalar> &G = this->loop_constraint_->G();
            S_q_cache_.resize(nv);
            for (int i = 0; i < nv; ++i) {
                S_q_cache_[i] = dX_intra_dq[i] * G;
            }

            S_q_cache_valid_ = true;
            return S_q_cache_;
        }

        template <typename Scalar>
        DMat<Scalar> RevoluteTripleWithRotor<Scalar>::getSdotqd_q() const
        {
            initializeCasadiFunctions();
            const int nv = 3;
            const int spatial_dim = 36;

            std::vector<casadi::DM> input = {
                casadi::DM(static_cast<double>(q_cache_(0))),
                casadi::DM(static_cast<double>(q_cache_(1))),
                casadi::DM(static_cast<double>(q_cache_(2))),
                casadi::DM(static_cast<double>(qd_cache_(0))),
                casadi::DM(static_cast<double>(qd_cache_(1))),
                casadi::DM(static_cast<double>(qd_cache_(2)))
            };

            const auto t_casadi_sdotq_start = std::chrono::high_resolution_clock::now();
            std::vector<casadi::DM> result = f_Sdotqd_q_(input);
            const double casadi_sdotq_us = std::chrono::duration<double, std::micro>(
                std::chrono::high_resolution_clock::now() - t_casadi_sdotq_start).count();
            profiling::addCasadiSdotqdQUs(casadi_sdotq_us);
            casadi::DM Sdotqd_q_link2 = result[0];  // 6x3 matrix for link2
            casadi::DM Sdotqd_q_link3 = result[1];  // 6x3 matrix for link3

            DMat<Scalar> output = DMat<Scalar>::Zero(spatial_dim, nv);

            // Link2 contribution (rows 6-11)
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < nv; ++j) {
                    output(6 + i, j) = static_cast<Scalar>(static_cast<double>(Sdotqd_q_link2(i, j)));
                }
            }

            // Link3 contribution (rows 12-17)
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < nv; ++j) {
                    output(12 + i, j) = static_cast<Scalar>(static_cast<double>(Sdotqd_q_link3(i, j)));
                }
            }

            return output;
        }

        template <typename Scalar>
        DMat<Scalar> RevoluteTripleWithRotor<Scalar>::getSdotqd_qd() const
        {
            initializeCasadiFunctions();
            const int nv = 3;
            const int spatial_dim = 36;

            std::vector<casadi::DM> input = {
                casadi::DM(static_cast<double>(q_cache_(0))),
                casadi::DM(static_cast<double>(q_cache_(1))),
                casadi::DM(static_cast<double>(q_cache_(2))),
                casadi::DM(static_cast<double>(qd_cache_(0))),
                casadi::DM(static_cast<double>(qd_cache_(1))),
                casadi::DM(static_cast<double>(qd_cache_(2)))
            };

            const auto t_casadi_sdotqd_start = std::chrono::high_resolution_clock::now();
            std::vector<casadi::DM> result = f_Sdotqd_qd_(input);
            const double casadi_sdotqd_us = std::chrono::duration<double, std::micro>(
                std::chrono::high_resolution_clock::now() - t_casadi_sdotqd_start).count();
            profiling::addCasadiSdotqdQdUs(casadi_sdotqd_us);
            casadi::DM Sdotqd_qd_link2 = result[0];  // 6x3 matrix for link2
            casadi::DM Sdotqd_qd_link3 = result[1];  // 6x3 matrix for link3

            DMat<Scalar> output = DMat<Scalar>::Zero(spatial_dim, nv);

            // Link2 contribution (rows 6-11)
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < nv; ++j) {
                    output(6 + i, j) = static_cast<Scalar>(static_cast<double>(Sdotqd_qd_link2(i, j)));
                }
            }

            // Link3 contribution (rows 12-17)
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < nv; ++j) {
                    output(12 + i, j) = static_cast<Scalar>(static_cast<double>(Sdotqd_qd_link3(i, j)));
                }
            }

            return output;
        }

        // Complex specializations
        template <>
        void RevoluteTripleWithRotor<std::complex<double>>::initializeCasadiFunctions() const
        {
            casadi_functions_initialized_ = true;
        }

        template <>
        std::vector<DMat<std::complex<double>>>
        RevoluteTripleWithRotor<std::complex<double>>::getSq() const
        {
            return std::vector<DMat<std::complex<double>>>(3, DMat<std::complex<double>>::Zero(36, 3));
        }

        template <>
        DMat<std::complex<double>>
        RevoluteTripleWithRotor<std::complex<double>>::getSdotqd_q() const
        {
            return DMat<std::complex<double>>::Zero(36, 3);
        }

        template <>
        DMat<std::complex<double>>
        RevoluteTripleWithRotor<std::complex<double>>::getSdotqd_qd() const
        {
            return DMat<std::complex<double>>::Zero(36, 3);
        }

        template class RevoluteTripleWithRotor<double>;
        template class RevoluteTripleWithRotor<std::complex<double>>;
        template class RevoluteTripleWithRotor<casadi::SX>;

    }

} // namespace grbda
