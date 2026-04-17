#ifndef GRBDA_GENERALIZED_JOINT_GENERIC_H
#define GRBDA_GENERALIZED_JOINT_GENERIC_H

#include "grbda/Codegen/CasadiGen.h"
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

            // Native phi function type - works with any scalar type (double, complex, SX)
            // This enables machine-precision complex-step differentiation
            using NativePhiFcn = std::function<DVec<Scalar>(const JointCoordinate<Scalar> &)>;

            // Constructor with symbolic phi only (legacy, uses Taylor expansion for complex)
            GenericImplicit(std::vector<bool> is_coordinate_independent, SymPhiFcn phi_fcn,
                           const CasadiHelperFunctions<double> &kg_dGdq_codegen = {});

            // Constructor with both symbolic and native phi (enables exact complex evaluation)
            GenericImplicit(std::vector<bool> is_coordinate_independent, SymPhiFcn phi_sym,
                           NativePhiFcn phi_native,
                           const CasadiHelperFunctions<double> &kg_dGdq_codegen = {});

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
            void updateBiasGOnly(const JointState<Scalar> &joint_state);
            void updateGAndg(const JointState<Scalar> &joint_state);
            void updateGAndgFromIndependentVelocity(const JointCoordinate<Scalar> &joint_pos,
                                                    const JointCoordinate<Scalar> &independent_vel);

            // Override to use native phi when available for machine-precision validation
            bool isValidSpanningPosition(const JointCoordinate<Scalar> &joint_pos) const;
            
            const std::vector<bool>& isCoordinateIndependent() const;

            void createRandomStateHelpers() override;

            // Check if native phi is available (for complex-step support)
            bool hasNativePhi() const { return has_native_phi_; }

            // Solve constraints phi(y, q_dep) = 0 for q_dep given (possibly complex) independent coords y
            // Uses Newton iteration with native phi for machine-precision complex-step differentiation
            // Returns the full spanning coordinates q = [q_ind, q_dep] in proper order
            // q_dep_init is the initial guess for dependent coordinates (usually the real solution)
            DVec<Scalar> solveConstraintsComplex(const DVec<Scalar>& y_independent,
                                                  const DVec<Scalar>& q_dep_init,
                                                  int max_iters = 10,
                                                  double tol = 1e-12) const;

            // Native phi function for use with complex-step differentiation
            // Returns empty function if not available
            const NativePhiFcn& nativePhi() const { return phi_native_; }

            // Symbolic phi function accessor (for creating complex-typed constraints)
            const SymPhiFcn& getSymbolicPhi() const { return phi_sym_; }

            // dG/dq CasADi function accessor (for computing G_dot = dG/dt in S_ring)
            // Returns the Jacobian of vec(G) w.r.t. q, shape (n_G_elements, n_q)
            const casadi::Function& getdGdqFcn() const { return dG_dq_fcn_; }

            // Combined G and dG/dq evaluator to reduce CasADi call boundary overhead.
            // Returns [G, dG_dq] with shapes (n_spanning, n_independent) and (n_G_elements, n_q).
            const casadi::Function& getGAnddGdqFcn() const { return G_dG_dq_fcn_; }

            // d²G/dq² CasADi function accessor (for Taylor series expansion in complex-step)
            // Returns the Hessian of vec(G) w.r.t. q, shape (n_G_elements * n_q, n_q)
            const casadi::Function& getd2Gdq2Fcn() const { return d2G_dq2_fcn_; }

            // Combined dG/dq and d²G/dq² evaluator to reduce repeated calls at identical q.
            // Returns [dG_dq, d2G_dq2] with shapes (n_G_elements, n_q) and (n_G_elements*n_q, n_q).
            const casadi::Function& getdGdqAndd2Gdq2Fcn() const { return dG_dq_d2G_dq2_fcn_; }

            // G CasADi function accessor (for evaluating G matrix)
            // Returns G matrix, shape (n_spanning, n_independent)
            const casadi::Function& getGFcn() const { return G_fcn_; }

            // g CasADi function accessor (for evaluating constraint bias)
            // Returns g vector/matrix in spanning coordinates, shape (n_spanning, 1)
            const casadi::Function& getgFcn() const { return g_fcn_; }

            // K CasADi function accessor (for computing constraint Jacobian analytically)
            // Returns K = dphi/dq, shape (n_constraints, n_spanning)
            const casadi::Function& getKFcn() const { return K_fcn_; }

            // Cached dG/dq from the last double-valued updateJacobians() call.
            bool hasCacheddGdq() const { return dG_dq_cache_valid_; }
            const DMat<double>& getCacheddGdq() const { return dG_dq_cache_; }

            // Evaluate dG/dq via optional AOT codegen helper. Returns true if used.
            bool evalDGdqCodegen(const DVec<double> &q, DMat<double> &dG_dq) const;

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
            SymPhiFcn phi_sym_;
            NativePhiFcn phi_native_;  // Optional native phi for complex-step support
            bool has_native_phi_ = false;

            casadi::Function K_fcn_;
            casadi::Function G_fcn_;
            casadi::Function Gg_fcn_;
            casadi::Function Gg_from_independent_vel_fcn_;
            casadi::Function Gg_dGdq_from_independent_vel_fcn_;
            casadi::Function KG_fcn_;
            casadi::Function k_fcn_;
            casadi::Function g_fcn_;
            casadi::Function kg_fcn_;

            // Derivative functions for complex-step support
            // dK/dq: for each q_i, gives the Jacobian of K w.r.t. q_i
            casadi::Function dK_dq_fcn_;
            // dG/dq: for each q_i, gives the Jacobian of G w.r.t. q_i
            casadi::Function dG_dq_fcn_;
            // Combined evaluator for [G, dG/dq] at the same q.
            casadi::Function G_dG_dq_fcn_;
            // Combined evaluator for [K, G, dG/dq] at the same q (double hot path).
            casadi::Function KG_dG_dq_fcn_;
            // d²G/dq²: Hessian of vec(G) w.r.t. q (for Taylor series in complex-step)
            casadi::Function d2G_dq2_fcn_;
            // Combined evaluator for [dG/dq, d²G/dq²] at the same q.
            casadi::Function dG_dq_d2G_dq2_fcn_;
            // dk/dq and dk/dv: Jacobians of k w.r.t. position and velocity
            casadi::Function dk_dq_fcn_;
            casadi::Function dk_dv_fcn_;
            // dg/dq and dg/dv: Jacobians of g w.r.t. position and velocity
            casadi::Function dg_dq_fcn_;
            casadi::Function dg_dv_fcn_;
            // Optional ahead-of-time generated code for [K, G, dG/dq] on Tello Generic constraints.
            CasadiHelperFunctions<double> kg_dGdq_codegen_;
            mutable std::vector<double> kg_dGdq_codegen_k_buf_;
            mutable std::vector<double> kg_dGdq_codegen_g_buf_;
            mutable std::vector<double> kg_dGdq_codegen_dG_buf_;
            mutable std::vector<grbda_int_T> kg_dGdq_codegen_iw_;
            mutable std::vector<double> kg_dGdq_codegen_w_;

            // Cache populated in updateJacobians(double): d(vec(G))/dq at the current q.
            mutable DMat<double> dG_dq_cache_;
            mutable bool dG_dq_cache_valid_ = false;
            mutable DVec<double> dG_dq_cache_key_;
            mutable bool dG_dq_cache_key_valid_ = false;
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

            JointState<double> randomJointState() const override;

            void updateKinematics(const JointState<Scalar> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<Scalar> &Xup) const override;

            // Motion subspace derivatives for configuration-dependent kinematics
            std::vector<DMat<Scalar>> getSq() const override;
            DMat<Scalar> getSdotqd_q() const override;
            DMat<Scalar> getSdotqd_qd() const override;

            // Access to GenericImplicit constraint for complex-step differentiation
            std::shared_ptr<LoopConstraint::GenericImplicit<Scalar>> getGenericConstraint() const {
                return generic_constraint_;
            }

        protected:
            // Protected members for derived classes (e.g., FourBar) to access
            DMat<Scalar> S_spanning_;
            DMat<Scalar> X_intra_;
            DMat<Scalar> X_intra_ring_;
            mutable DVec<Scalar> q_cache_;
            mutable DVec<Scalar> qd_cache_;

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
            mutable std::vector<int> body_vel_offsets_;
            mutable std::vector<int> body_num_vel_;
            mutable std::vector<int> body_num_pos_;
            mutable std::vector<DMat<Scalar>> body_S_cache_;
            mutable std::vector<std::vector<std::pair<int, int>>> body_affected_pairs_;

            // Cached intermediates for derivative evaluation
            mutable DMat<Scalar> S_implicit_;
            mutable std::vector<DMat<Scalar>> S_q_cache_;
            mutable bool S_q_cache_valid_ = false;
            mutable DMat<Scalar> Sdotqd_q_cache_;
            mutable bool Sdotqd_q_cache_valid_ = false;
            mutable DVec<Scalar> Sdotqd_q_key_q_cache_;
            mutable DVec<Scalar> Sdotqd_q_key_qd_cache_;
            mutable DMat<Scalar> dG_dq_cache_;
            mutable bool dG_dq_cache_valid_ = false;
            mutable DVec<Scalar> ydot_independent_cache_;
            mutable bool ydot_independent_cache_valid_ = false;
            mutable DVec<Scalar> raw_q_input_cache_;
            mutable DVec<Scalar> raw_qd_input_cache_;
            mutable bool raw_input_cache_valid_ = false;
            mutable bool raw_pos_input_spanning_ = false;
            mutable bool raw_vel_input_spanning_ = false;

            void initializeDerivativeFunctions() const;

            Generic<casadi::SX> copyAsSymbolic() const;

            // CasADi functions for computing dG/dq and Sdotqd derivatives
            mutable casadi::Function dG_dq_fcn_;
            mutable casadi::Function dSdotqd_q_analytic_fcn_;
            mutable casadi::Function dSdotqd_dq_fcn_;
            mutable casadi::Function dSdotqd_dqd_fcn_;
            mutable bool derivative_functions_initialized_ = false;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINT_GENERIC_H
