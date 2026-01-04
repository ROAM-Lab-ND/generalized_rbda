#include "grbda/Dynamics/ClusterJoints/RevolutePairJoint.h"
#include "grbda/Utils/CasadiDerivatives.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar>
        RevolutePair<Scalar>::RevolutePair(Body<Scalar> &link_1, Body<Scalar> &link_2,
                                           ori::CoordinateAxis joint_axis_1,
                                           ori::CoordinateAxis joint_axis_2)
            : Base<Scalar>(2, 2, 2), link_1_(link_1), link_2_(link_2),
              axis1_(joint_axis_1), axis2_(joint_axis_2)
        {
            using Rev = Joints::Revolute<Scalar>;
            link_1_joint_ =  this->single_joints_.emplace_back(new Rev(joint_axis_1));
            link_2_joint_ =  this->single_joints_.emplace_back(new Rev(joint_axis_2));

            this->spanning_tree_to_independent_coords_conversion_ = DMat<int>::Identity(2, 2);

            DMat<Scalar> G = DMat<Scalar>::Zero(2, 2);
            G << 1., 0.,
                0., 1.;
            const DMat<Scalar> K = DMat<Scalar>::Identity(0, 2);
            this->loop_constraint_ = std::make_shared<LoopConstraint::Static<Scalar>>(G, K);

            X_intra_S_span_ = DMat<Scalar>::Zero(12, 2);
            X_intra_S_span_ring_ = DMat<Scalar>::Zero(12, 2);

            X_intra_S_span_.template block<6, 1>(0, 0) = link_1_joint_->S();
            X_intra_S_span_.template block<6, 1>(6, 1) = link_2_joint_->S();

            this->S_ = X_intra_S_span_ * this->loop_constraint_->G();
        }

        template <typename Scalar>
        void RevolutePair<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            const DVec<Scalar> &q = spanning_joint_state.position;
            const DVec<Scalar> &qd = spanning_joint_state.velocity;

            // Cache INDEPENDENT coordinates for derivative methods (not spanning tree!)
            q_cache_ = joint_state.position;
            qd_cache_ = joint_state.velocity;

            link_1_joint_->updateKinematics(q.template segment<1>(0), qd.template segment<1>(0));
            link_2_joint_->updateKinematics(q.template segment<1>(1), qd.template segment<1>(1));

            X21_ = link_2_joint_->XJ() * link_2_.Xtree_;
            const DVec<Scalar> v2_relative = link_2_joint_->S() * qd[1];
            X_intra_S_span_.template block<6, 1>(6, 0) =
                X21_.transformMotionSubspace(link_1_joint_->S());
            this->S_.template block<6, 1>(6, 0) = X21_.transformMotionSubspace(link_1_joint_->S());

            X_intra_S_span_ring_.template block<6, 1>(6, 0) =
                -spatial::generalMotionCrossMatrix(v2_relative) *
                X_intra_S_span_.template block<6, 1>(6, 0);

            this->vJ_ = X_intra_S_span_ * qd;
            this->cJ_ = X_intra_S_span_ring_ * qd;
            this->S_ring_ = X_intra_S_span_ring_ * this->loop_constraint_->G(); //+X_intra*S_span_*G_dot_;
        }

        template <typename Scalar>
        void RevolutePair<Scalar>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 2)
                throw std::runtime_error("[RevolutePair] Xup must have 12 rows");
#endif

            Xup[0] = link_1_joint_->XJ() * link_1_.Xtree_;
            Xup[1] = link_2_joint_->XJ() * link_2_.Xtree_ * Xup[0];
        }

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        RevolutePair<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> bodies_joints_and_reflected_inertias;

            const DMat<Scalar> reflected_inertia_1 = DMat<Scalar>::Zero(this->numVelocities(),
                                                                        this->numVelocities());
            bodies_joints_and_reflected_inertias.push_back(
                std::make_tuple(link_1_, link_1_joint_, reflected_inertia_1));

            const DMat<Scalar> reflected_inertia_2 = DMat<Scalar>::Zero(this->numVelocities(),
                                                                        this->numVelocities());
            bodies_joints_and_reflected_inertias.push_back(
                std::make_tuple(link_2_, link_2_joint_, reflected_inertia_2));

            return bodies_joints_and_reflected_inertias;
        }

        template <typename Scalar>
        char RevolutePair<Scalar>::axisToChar(ori::CoordinateAxis axis) const
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
        void RevolutePair<Scalar>::initializeCasadiFunctions() const
        {
            if (casadi_functions_initialized_) return;

            using namespace casadi;
            using namespace casadi_derivatives;

            SX q1 = SX::sym("q1");
            SX q2 = SX::sym("q2");
            SX qd1 = SX::sym("qd1");
            SX qd2 = SX::sym("qd2");

            char ax1 = axisToChar(axis1_);
            char ax2 = axisToChar(axis2_);

            SX S1_sym = revoluteMotionSubspace(ax1);
            SX S2_sym = revoluteMotionSubspace(ax2);

            DMat<double> Xtree2_eigen = link_2_.Xtree_.toMatrix().template cast<double>();
            SX Xtree2_sx = SX::zeros(6, 6);
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    Xtree2_sx(i, j) = Xtree2_eigen(i, j);
                }
            }

            SX XJ2_sym = spatialRotation(ax2, q2);
            SX X21_sym = mtimes(XJ2_sym, Xtree2_sx);
            SX S_body2_col0 = mtimes(X21_sym, S1_sym);

            SX dS_body2_col0_dq1 = jacobian(S_body2_col0, q1);
            SX dS_body2_col0_dq2 = jacobian(S_body2_col0, q2);

            // For Sdotqd_q and Sdotqd_qd: compute Ṡ*qd where Ṡ = dS/dq1*q̇1 + dS/dq2*q̇2
            SX Sdot_col0 = dS_body2_col0_dq1 * qd1 + dS_body2_col0_dq2 * qd2;
            SX Sdotqd = Sdot_col0 * qd1;  // Only column 0 contributes (column 1 is constant)

            // Compute ∂(Ṡqd)/∂q
            SX dSdotqd_dq1 = jacobian(Sdotqd, q1);
            SX dSdotqd_dq2 = jacobian(Sdotqd, q2);

            // Compute ∂(Ṡqd)/∂qd
            SX dSdotqd_dqd1 = jacobian(Sdotqd, qd1);
            SX dSdotqd_dqd2 = jacobian(Sdotqd, qd2);

            f_dS_dq1_ = Function("dS_dq1", {q1, q2}, {dS_body2_col0_dq1});
            f_dS_dq2_ = Function("dS_dq2", {q1, q2}, {dS_body2_col0_dq2});
            f_Sdotqd_q_ = Function("Sdotqd_q", {q1, q2, qd1, qd2}, {horzcat(dSdotqd_dq1, dSdotqd_dq2)});
            f_Sdotqd_qd_ = Function("Sdotqd_qd", {q1, q2, qd1, qd2}, {horzcat(dSdotqd_dqd1, dSdotqd_dqd2)});

            casadi_functions_initialized_ = true;
        }

        template <typename Scalar>
        std::vector<DMat<Scalar>> RevolutePair<Scalar>::getSq() const
        {
            initializeCasadiFunctions();
            const int nv = 2;
            const int spatial_dim = 12;

            std::vector<casadi::DM> input = {
                casadi::DM(static_cast<double>(q_cache_(0))),
                casadi::DM(static_cast<double>(q_cache_(1)))
            };
            auto res_dq1 = f_dS_dq1_(input);
            auto res_dq2 = f_dS_dq2_(input);

            // Create ∂X_intra_S_span/∂qi (12x2 matrix, mostly zero)
            DMat<Scalar> dX_intra_dq1 = DMat<Scalar>::Zero(spatial_dim, 2);
            DMat<Scalar> dX_intra_dq2 = DMat<Scalar>::Zero(spatial_dim, 2);

            // Only the [link2, link1] block is non-zero (rows 6-11, column 0)
            for (int i = 0; i < 6; ++i) {
                dX_intra_dq1(6 + i, 0) = static_cast<Scalar>(static_cast<double>(res_dq1[0](i)));
                dX_intra_dq2(6 + i, 0) = static_cast<Scalar>(static_cast<double>(res_dq2[0](i)));
            }

            // Compute ∂S/∂qi = (∂X_intra_S_span/∂qi) * G
            const DMat<Scalar> &G = this->loop_constraint_->G();
            std::vector<DMat<Scalar>> S_q(nv);
            S_q[0] = dX_intra_dq1 * G;  // 12x2 matrix
            S_q[1] = dX_intra_dq2 * G;  // 12x2 matrix

            return S_q;
        }

        template <typename Scalar>
        DMat<Scalar> RevolutePair<Scalar>::getSdotqd_q() const
        {
            initializeCasadiFunctions();
            const int nv = 2;
            const int spatial_dim = 12;

            std::vector<casadi::DM> input = {
                casadi::DM(static_cast<double>(q_cache_(0))),
                casadi::DM(static_cast<double>(q_cache_(1))),
                casadi::DM(static_cast<double>(qd_cache_(0))),
                casadi::DM(static_cast<double>(qd_cache_(1)))
            };

            std::vector<casadi::DM> result = f_Sdotqd_q_(input);
            casadi::DM Sdotqd_q_result = result[0];  // 6x2 matrix

            DMat<Scalar> output = DMat<Scalar>::Zero(spatial_dim, nv);

            // Fill in the link2 block (rows 6-11)
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < nv; ++j) {
                    output(6 + i, j) = static_cast<Scalar>(static_cast<double>(Sdotqd_q_result(i, j)));
                }
            }

            return output;
        }

        template <typename Scalar>
        DMat<Scalar> RevolutePair<Scalar>::getSdotqd_qd() const
        {
            initializeCasadiFunctions();
            const int nv = 2;
            const int spatial_dim = 12;
            std::vector<casadi::DM> input = {
                casadi::DM(static_cast<double>(q_cache_(0))),
                casadi::DM(static_cast<double>(q_cache_(1))),
                casadi::DM(static_cast<double>(qd_cache_(0))),
                casadi::DM(static_cast<double>(qd_cache_(1)))
            };
            auto res = f_Sdotqd_qd_(input);
            DMat<Scalar> result = DMat<Scalar>::Zero(spatial_dim, nv);
            for (int i = 0; i < 6; ++i) {
                result(6 + i, 0) = static_cast<Scalar>(static_cast<double>(res[0](i, 0)));
                result(6 + i, 1) = static_cast<Scalar>(static_cast<double>(res[0](i, 1)));
            }
            return result;
        }

        template <>
        void RevolutePair<std::complex<double>>::initializeCasadiFunctions() const
        {
            casadi_functions_initialized_ = true;
        }

        template <>
        std::vector<DMat<std::complex<double>>>
        RevolutePair<std::complex<double>>::getSq() const
        {
            return std::vector<DMat<std::complex<double>>>(2, DMat<std::complex<double>>::Zero(12, 2));
        }

        template <>
        DMat<std::complex<double>>
        RevolutePair<std::complex<double>>::getSdotqd_q() const
        {
            return DMat<std::complex<double>>::Zero(12, 2);
        }

        template <>
        DMat<std::complex<double>>
        RevolutePair<std::complex<double>>::getSdotqd_qd() const
        {
            return DMat<std::complex<double>>::Zero(12, 2);
        }

        template class RevolutePair<double>;
        template class RevolutePair<std::complex<double>>;
        template class RevolutePair<float>;
        template class RevolutePair<casadi::SX>;
    }

} // namespace grbda
