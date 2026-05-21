#ifndef GRBDA_GENERALIZED_JOINT_GENERIC_H
#define GRBDA_GENERALIZED_JOINT_GENERIC_H

#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"

namespace grbda
{

    namespace LoopConstraint
    {
        template <typename Scalar = double>
        struct GenericImplicit : Base<Scalar>
        {
            using SX = casadi::SX;
            using SymPhiFcn = std::function<DVec<SX>(const JointCoordinate<SX> &)>;

            GenericImplicit(std::vector<bool> is_coordinate_independent, SymPhiFcn phi_fcn);

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<GenericImplicit<Scalar>>(*this);
            }

            GenericImplicit<double> copyAsDouble() const
            {
                return GenericImplicit<double>(this->is_coordinate_independent_, phi_sym_);
            }

            GenericImplicit<SX> copyAsSymbolic() const
            {
                return GenericImplicit<SX>(this->is_coordinate_independent_, phi_sym_);
            }

            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const override;
            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override;
            void updateBiases(const JointState<Scalar> &joint_state) override;

            bool isValidSpanningPosition(const JointCoordinate<Scalar> &joint_pos) const;
            
            const std::vector<bool>& isCoordinateIndependent() const;

            void createRandomStateHelpers() override;

            // Symbolic phi function accessor (for creating complex-typed constraints)
            const SymPhiFcn& getSymbolicPhi() const { return phi_sym_; }

            // Coordinate permutation matrix: q_span = coord_map * [y; q_dep]
            // coord_map^T extracts [y; q_dep] from q_span, so ydot = (coord_map^T * qd_span).head(nv)
            const DMat<double>& getCoordMap() const { return coord_map_; }

            // dG/dq CasADi function accessor (for computing G_dot = dG/dt in S_ring)
            // Returns the Jacobian of vec(G) w.r.t. q, shape (n_G_elements, n_q)
            const casadi::Function& getdGdqFcn() const { return dG_dq_fcn_; }

            // G CasADi function accessor (for evaluating G matrix)
            // Returns G matrix, shape (n_spanning, n_independent)
            const casadi::Function& getGFcn() const { return G_fcn_; }

            // g CasADi function accessor (for evaluating explicit constraint bias)
            // Returns g vector, shape (n_spanning, 1), takes {q, v} as inputs
            const casadi::Function& getgFcn() const { return g_fcn_; }

            // K CasADi function accessor (for computing constraint Jacobian analytically)
            // Returns K = dphi/dq, shape (n_constraints, n_spanning)
            const casadi::Function& getKFcn() const { return K_fcn_; }

        private:
            // Basic CasADi function evaluation (real-valued)
            static DMat<double> runCasadiFcnReal(const casadi::Function &fcn,
                                                  const DVec<double> &arg);
            static DMat<double> runCasadiFcnReal(const casadi::Function &fcn,
                                                  const DVec<double> &pos,
                                                  const DVec<double> &vel);

            // Complex-step aware evaluation methods (non-static, use derivative functions)
            DMat<Scalar> evalK(const JointCoordinate<Scalar> &joint_pos) const;
            DMat<Scalar> evalG(const JointCoordinate<Scalar> &joint_pos) const;
            DMat<Scalar> evalk(const JointState<Scalar> &joint_state) const;
            DMat<Scalar> evalg(const JointState<Scalar> &joint_state) const;

            // Legacy static methods for phi evaluation
            static DMat<Scalar> runCasadiFcn(const casadi::Function &fcn,
                                             const JointCoordinate<Scalar> &arg);
            static DMat<Scalar> runCasadiFcn(const casadi::Function &fcn,
                                             const JointState<Scalar> &args);

            const std::vector<bool> is_coordinate_independent_;
            DMat<double> coord_map_;   // permutation: q_span = coord_map * [y; q_dep]
            SymPhiFcn phi_sym_;

            casadi::Function cs_phi_fcn_;
            casadi::Function K_fcn_;
            casadi::Function G_fcn_;
            casadi::Function k_fcn_;
            casadi::Function g_fcn_;

            // Derivative functions for complex-step support
            // dK/dq: for each q_i, gives the Jacobian of K w.r.t. q_i
            casadi::Function dK_dq_fcn_;
            // dG/dq: for each q_i, gives the Jacobian of G w.r.t. q_i
            casadi::Function dG_dq_fcn_;

            // dk/dq and dk/dv: Jacobians of k w.r.t. position and velocity
            casadi::Function dk_dq_fcn_;
            casadi::Function dk_dv_fcn_;
            // dg/dq and dg/dv: Jacobians of g w.r.t. position and velocity
            casadi::Function dg_dq_fcn_;
            casadi::Function dg_dv_fcn_;
        };
    }

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class Generic : public Base<Scalar>
        {
        public:
            Generic(const std::vector<Body<Scalar>> &bodies,
                    const std::vector<JointPtr<Scalar>> &joints,
                    std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint);

            Generic(const std::vector<Body<Scalar>> &bodies,
                    const std::vector<JointPtr<Scalar>> &joints,
                    std::shared_ptr<LoopConstraint::GenericImplicit<Scalar>> loop_constraint);

            ClusterJointTypes type() const override { return ClusterJointTypes::Generic; }

            JointState<double> randomJointState(bool enforce_position_constraint = true) const override;

            void updateKinematics(const JointState<Scalar> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<Scalar> &Xup) const override;

            // Motion subspace derivatives for configuration-dependent kinematics
            DMat<Scalar> getSdotqd_q() const override;
            DMat<Scalar> getSdotqd_qd() const override;

            // GenericJoint has configuration-dependent S (uses CasADi)
            bool hasConfigurationDependentS() const override { return generic_constraint_ != nullptr; }

            // Contraction-based derivatives (efficient, avoids materializing S_q tensor)
            DMat<Scalar> evalSTimesVec_dq(const DVec<Scalar>& b) const override;
            DMat<Scalar> evalSTTimesVec_dq(const DVec<Scalar>& F) const override;

            // Access to GenericImplicit constraint for complex-step differentiation
            std::shared_ptr<LoopConstraint::GenericImplicit<Scalar>> getGenericConstraint() const {
                return generic_constraint_;
            }

        protected:
            // Protected members for derived classes (e.g., FourBar) to access
            DMat<Scalar> S_spanning_;
            DMat<Scalar> X_intra_;
            DMat<Scalar> X_intra_ring_;
            mutable DVec<Scalar> q_spanning_;
            mutable DVec<Scalar> qd_spanning_;

        private:
            void initialize(const std::vector<JointPtr<Scalar>> &joints,
                            std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint);

            JointCoordinate<double> findRootsForPhi() const;
        
            void extractConnectivity();

            bool bodyInCurrentCluster(const int body_index) const;
            const Body<Scalar> &getBody(const int body_index) const;

            const std::vector<Body<Scalar>> bodies_;
            std::shared_ptr<LoopConstraint::GenericImplicit<Scalar>> generic_constraint_;

            DMat<bool> connectivity_;

            // Cached intermediates for derivative evaluation
            mutable DMat<Scalar> S_implicit_;

            void initializeDerivativeFunctions() const;

            // CasADi functions for computing dG/dq and Sdotqd derivatives
            mutable casadi::Function dG_dq_fcn_;
            mutable casadi::Function dSdotqd_dq_fcn_;

            // CasADi functions for efficient contraction-based derivatives
            // d(S*b)/dq: inputs {q_span, b}, outputs (mss_dim x nv) matrix
            mutable casadi::Function dSb_dy_fcn_;
            // d(S^T*F)/dq: inputs {q_span, F}, outputs (nv x nv) matrix
            mutable casadi::Function dSTF_dy_fcn_;

            // Pre-allocated work vectors for low-level CasADi evaluation (avoids allocation overhead)
            mutable std::vector<double> dSb_work_w_;
            mutable std::vector<casadi_int> dSb_work_iw_;
            mutable std::vector<double> dSb_arg_buf_;   // concatenated input buffer [q; b]
            mutable std::vector<double> dSb_res_buf_;   // output buffer
            mutable std::vector<double> dSTF_work_w_;
            mutable std::vector<casadi_int> dSTF_work_iw_;
            mutable std::vector<double> dSTF_arg_buf_;  // concatenated input buffer [q; F]
            mutable std::vector<double> dSTF_res_buf_;  // output buffer
            mutable std::vector<double> dSdotqd_work_w_;
            mutable std::vector<casadi_int> dSdotqd_work_iw_;
            mutable std::vector<double> dSdotqd_arg_buf_;  // concatenated input buffer [q; ydot]
            mutable std::vector<double> dSdotqd_res_buf_;  // output buffer

            mutable bool derivative_functions_initialized_ = false;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINT_GENERIC_H
