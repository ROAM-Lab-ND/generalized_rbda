#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"
#include "grbda/Dynamics/ClusterJoints/LazyGenericJoint.h"
#include "grbda/Dynamics/ClusterJoints/PrecompiledGenericJoint.h"
#include "grbda/Utils/IDDerivProfile.h"
#include "grbda/Utils/Utilities.h"

#include <chrono>

namespace grbda
{

#include <iomanip>
    namespace LoopConstraint
    {
        template <typename Scalar>
        GenericImplicit<Scalar>::GenericImplicit(std::vector<bool> is_coordinate_independent,
                                                 SymPhiFcn phi_fcn,
                                                 const CasadiHelperFunctions<double> &kg_dGdq_codegen)
            : is_coordinate_independent_(is_coordinate_independent), phi_sym_(phi_fcn)
        {
            // Separate coordinates into independent and dependent
            int state_dim = is_coordinate_independent.size();
            std::vector<int> ind_coords, dep_coords;
            for (int i = 0; i < state_dim; i++)
            {
                if (is_coordinate_independent[i])
                    ind_coords.push_back(i);
                else
                    dep_coords.push_back(i);
            }
            int ind_dim = ind_coords.size();
            int dep_dim = dep_coords.size();

            // Debug output for coordinate sizes
            std::cout << "[GenericImplicit] state_dim=" << state_dim
                      << ", ind_dim=" << ind_dim << ", dep_dim=" << dep_dim << std::endl;
            if (state_dim == 0 || ind_dim + dep_dim != state_dim) {
                std::cerr << "[GenericImplicit] Invalid coordinate sizes!" << std::endl;
            }

            // The coordinate map is a matrix that maps the stacked indepedent
            // coordinates [y;q_dep] to the spanning coordinate vector q such that
            // q = coord_map * [y;q_dep]
            SX coord_map = SX::zeros(state_dim, state_dim);
            for (int i = 0; i < ind_dim; i++)
            {
                coord_map(ind_coords[i], i) = 1;
            }
            for (int i = 0; i < dep_dim; i++)
            {
                coord_map(dep_coords[i], i + ind_dim) = 1;
            }

            // Symbolic state
            SX cs_q_sym = SX::sym("q", state_dim, 1);
            DVec<SX> q_sym(state_dim);
            if (cs_q_sym.size2() == 0 || state_dim == 0) {
                std::cerr << "[GenericImplicit] cs_q_sym has zero size!" << std::endl;
            }
            casadi::copy(cs_q_sym, q_sym);
            JointCoordinate<SX> joint_pos_sym(q_sym, true);

            SX cs_v_sym = SX::sym("v", state_dim, 1);
            DVec<SX> v_sym(state_dim);
            if (cs_v_sym.size2() == 0 || state_dim == 0) {
                std::cerr << "[GenericImplicit] cs_v_sym has zero size!" << std::endl;
            }
            casadi::copy(cs_v_sym, v_sym);

            // Implicit constraint violation function
            DVec<SX> phi_sym = phi_fcn(joint_pos_sym);
            const int constraint_dim = phi_sym.rows();
            if (constraint_dim == 0) {
                std::cerr << "[GenericImplicit] phi_sym has zero rows!" << std::endl;
            }
            SX cs_phi_sym = casadi::SX(casadi::Sparsity::dense(constraint_dim, 1));
            casadi::copy(phi_sym, cs_phi_sym);
            casadi::Function cs_phi_fcn = casadi::Function("phi", {cs_q_sym}, {cs_phi_sym});

            // Implicit constraint jacobian
            SX cs_K_sym = jacobian(cs_phi_sym, cs_q_sym);

            // Implict constraint bias
            SX cs_Kdot_sym = SX(constraint_dim, state_dim);
            for (size_t i = 0; i < state_dim; i++)
            {
                casadi::Slice all = casadi::Slice();
                cs_Kdot_sym(all, i) = jtimes(cs_K_sym(all, i), cs_q_sym, cs_v_sym);
            }
            SX cs_k_sym = -casadi::SX::mtimes(cs_Kdot_sym, cs_v_sym);

            // Explicit constraint jacobian
            SX cs_Ki_sym = SX(constraint_dim, ind_coords.size());
            for (size_t i = 0; i < ind_coords.size(); i++)
            {
                cs_Ki_sym(casadi::Slice(), i) = cs_K_sym(casadi::Slice(), ind_coords[i]);
            }
            SX cs_Kd_sym = SX(constraint_dim, dep_coords.size());
            for (size_t i = 0; i < dep_coords.size(); i++)
            {
                cs_Kd_sym(casadi::Slice(), i) = cs_K_sym(casadi::Slice(), dep_coords[i]);
            }
            SX cs_G_sym = SX::zeros(state_dim, ind_dim);
            casadi::Slice ind_slice = casadi::Slice(0, ind_dim);
            cs_G_sym(ind_slice, ind_slice) = SX::eye(ind_dim);
            casadi::Slice dep_slice = casadi::Slice(ind_dim, ind_dim + dep_dim);
            if (cs_Kd_sym.size2() == 0 || cs_Ki_sym.size2() == 0) {
                std::cerr << "[GenericImplicit] cs_Kd_sym or cs_Ki_sym has zero size!" << std::endl;
            }
            
            // For 2x2 matrices, use analytical formula for complex-step safety
            // For larger matrices, use solve which is more numerically stable
            if (dep_dim == 2 && ind_coords.size() == 1) {
                // 2x2 system: inv([[a,b],[c,d]]) * [[e],[f]] = (1/det) * [[d,-b],[-c,a]] * [[e],[f]]
                // This is complex-step safe (pure algebraic operations)
                SX a = cs_Kd_sym(0, 0);
                SX b = cs_Kd_sym(0, 1);
                SX c = cs_Kd_sym(1, 0);
                SX d = cs_Kd_sym(1, 1);
                SX det = a*d - b*c;
                SX e = cs_Ki_sym(0, 0);
                SX f = cs_Ki_sym(1, 0);
                SX inv_Kd_e = (d*e - b*f) / det;
                SX inv_Kd_f = (-c*e + a*f) / det;
                std::vector<SX> col_vec = {-inv_Kd_e, -inv_Kd_f};
                cs_G_sym(dep_slice, casadi::Slice()) = SX::vertcat(col_vec);
            } else {
                // General case: use solve() which is more numerically stable
                cs_G_sym(dep_slice, casadi::Slice()) = -SX::solve(cs_Kd_sym, cs_Ki_sym);
            }
            cs_G_sym = SX::mtimes(coord_map, cs_G_sym);

            // Explicit constraints bias
            SX cs_g_sym = SX::zeros(state_dim, 1);
            if (cs_Kd_sym.size2() == 0) {
                std::cerr << "[GenericImplicit] cs_Kd_sym has zero size for bias!" << std::endl;
            }
            
            // For 2x2 matrices, use analytical formula for complex-step safety
            if (dep_dim == 2) {
                // 2x2 system: inv([[a,b],[c,d]]) * [[e],[f]] = (1/det) * [[d,-b],[-c,a]] * [[e],[f]]
                SX a = cs_Kd_sym(0, 0);
                SX b = cs_Kd_sym(0, 1);
                SX c = cs_Kd_sym(1, 0);
                SX d = cs_Kd_sym(1, 1);
                SX det = a*d - b*c;
                SX e = cs_k_sym(0);
                SX f = cs_k_sym(1);
                SX inv_Kd_e = (d*e - b*f) / det;
                SX inv_Kd_f = (-c*e + a*f) / det;
                std::vector<SX> col_vec = {inv_Kd_e, inv_Kd_f};
                cs_g_sym(dep_slice) = SX::vertcat(col_vec);
            } else {
                // General case: use solve()
                cs_g_sym(dep_slice) = SX::solve(cs_Kd_sym, cs_k_sym);
            }
            cs_g_sym = SX::mtimes(coord_map, cs_g_sym);

            // Assign member variables using casadi functions
            this->phi_ = [cs_phi_fcn](const JointCoordinate<Scalar> &joint_pos)
            {
                return runCasadiFcn(cs_phi_fcn, joint_pos);
            };

            this->K_ = DMat<Scalar>::Zero(constraint_dim, state_dim);
            K_fcn_ = casadi::Function("K", {cs_q_sym}, {cs_K_sym});

            this->G_ = DMat<Scalar>::Zero(state_dim, ind_dim);
            G_fcn_ = casadi::Function("G", {cs_q_sym}, {cs_G_sym});

            this->k_ = DVec<Scalar>::Zero(constraint_dim);
            k_fcn_ = casadi::Function("k", {cs_q_sym, cs_v_sym}, {cs_k_sym});

            this->g_ = DVec<Scalar>::Zero(state_dim);
            g_fcn_ = casadi::Function("g", {cs_q_sym, cs_v_sym}, {cs_g_sym});

            // Create derivative functions for complex-step support
            // dK/dq: Jacobian of each element of K w.r.t. q
            SX dK_dq_sym = jacobian(SX::vec(cs_K_sym), cs_q_sym);
            dK_dq_fcn_ = casadi::Function("dK_dq", {cs_q_sym}, {dK_dq_sym});

            // dG/dq: Jacobian of each element of G w.r.t. q
            SX dG_dq_sym = jacobian(SX::vec(cs_G_sym), cs_q_sym);
            dG_dq_fcn_ = casadi::Function("dG_dq", {cs_q_sym}, {dG_dq_sym});

            // d²G/dq²: Hessian of vec(G) w.r.t. q (for Taylor series in complex-step)
            // Shape: (n_G_elements * n_q, n_q) - Jacobian of dG/dq w.r.t. q
            SX d2G_dq2_sym = jacobian(SX::vec(dG_dq_sym), cs_q_sym);
            d2G_dq2_fcn_ = casadi::Function("d2G_dq2", {cs_q_sym}, {d2G_dq2_sym});

            // dk/dq and dk/dv: Jacobians of k w.r.t. position and velocity
            SX dk_dq_sym = jacobian(cs_k_sym, cs_q_sym);
            SX dk_dv_sym = jacobian(cs_k_sym, cs_v_sym);
            dk_dq_fcn_ = casadi::Function("dk_dq", {cs_q_sym, cs_v_sym}, {dk_dq_sym});
            dk_dv_fcn_ = casadi::Function("dk_dv", {cs_q_sym, cs_v_sym}, {dk_dv_sym});

            // dg/dq and dg/dv: Jacobians of g w.r.t. position and velocity
            SX dg_dq_sym = jacobian(cs_g_sym, cs_q_sym);
            SX dg_dv_sym = jacobian(cs_g_sym, cs_v_sym);
            dg_dq_fcn_ = casadi::Function("dg_dq", {cs_q_sym, cs_v_sym}, {dg_dq_sym});
            dg_dv_fcn_ = casadi::Function("dg_dv", {cs_q_sym, cs_v_sym}, {dg_dv_sym});
        }

        // Constructor with both symbolic and native phi functions
        // The native phi enables machine-precision complex-step differentiation
        template <typename Scalar>
        GenericImplicit<Scalar>::GenericImplicit(std::vector<bool> is_coordinate_independent,
                                                 SymPhiFcn phi_sym, NativePhiFcn phi_native,
                                                 const CasadiHelperFunctions<double> &kg_dGdq_codegen)
            : GenericImplicit(is_coordinate_independent, phi_sym, kg_dGdq_codegen)
        {
            phi_native_ = phi_native;
            has_native_phi_ = static_cast<bool>(phi_native_);

            // Override phi_ to use native phi for complex types (CasADi doesn't support complex)
            // Also use native phi for double types when available for better numerical accuracy
            // The native C++ implementation using std::sin/std::cos is more precise than
            // CasADi's symbolic evaluation, which may have truncation in constant terms
            if constexpr (std::is_same_v<Scalar, std::complex<double>> || std::is_same_v<Scalar, double>) {
                if (!has_native_phi_) {
                    return;
                }
                this->phi_ = [this](const JointCoordinate<Scalar> &joint_pos) -> DVec<Scalar>
                {
                    return phi_native_(joint_pos);
                };
            }
        }

        template <typename Scalar>
        DVec<Scalar> GenericImplicit<Scalar>::gamma(const JointCoordinate<Scalar> &joint_pos) const
        {
            throw std::runtime_error("GenericImplicit::gamma() not implemented");
        }

        template <typename Scalar>
        void GenericImplicit<Scalar>::updateJacobians(const JointCoordinate<Scalar> &joint_pos)
        {
            this->K_ = evalK(joint_pos);
            this->G_ = evalG(joint_pos);
        }

        template <typename Scalar>
        void GenericImplicit<Scalar>::updateBiases(const JointState<Scalar> &joint_state)
        {
            this->k_ = evalk(joint_state);
            this->g_ = evalg(joint_state);
        }

        template <typename Scalar>
        void GenericImplicit<Scalar>::updateBiasGOnly(const JointState<Scalar> &joint_state)
        {
            // Only update g (explicit bias), not k (implicit bias)
            this->g_ = evalg(joint_state);
        }

        template <typename Scalar>
        void GenericImplicit<Scalar>::updateGAndg(const JointState<Scalar> &joint_state)
        {
            // Update both G (Jacobian) and g (bias)
            updateJacobians(joint_state.position);
            this->g_ = evalg(joint_state);
        }

        template <typename Scalar>
        void GenericImplicit<Scalar>::updateGAndgFromIndependentVelocity(
            const JointCoordinate<Scalar> &joint_pos,
            const JointCoordinate<Scalar> &independent_vel)
        {
            // G is defined at spanning configuration and maps independent velocity
            // to spanning velocity. g(q, qd_span) must be evaluated with spanning qd.
            updateJacobians(joint_pos);

            DVec<Scalar> spanning_vel_data = this->G_ * independent_vel;
            JointCoordinate<Scalar> spanning_vel(spanning_vel_data, true);
            JointState<Scalar> joint_state(joint_pos, spanning_vel);
            this->g_ = evalg(joint_state);
        }

        template <typename Scalar>
        const std::vector<bool> &GenericImplicit<Scalar>::isCoordinateIndependent() const
        {
            return is_coordinate_independent_;
        }

        // Override isValidSpanningPosition to use native phi when available
        // This ensures consistency with the Newton solver that uses native phi
        template <typename Scalar>
        bool GenericImplicit<Scalar>::isValidSpanningPosition(const JointCoordinate<Scalar> &joint_pos) const
        {
            if (!joint_pos.isSpanning()) {
                return false;
            }

            DVec<Scalar> violation;

            // Use native phi when available for machine-precision validation
            // This is critical for complex-step differentiation where Newton solver
            // converges to machine precision using native phi
            if (has_native_phi_) {
                violation = phi_native_(joint_pos);
            } else {
                violation = this->phi_(joint_pos);
            }

            // Tolerance for constraint validation - Newton solver can achieve machine precision
            // when properly converged with native phi, but CasADi phi may have small offsets
            const double tol = has_native_phi_ ? 1e-8 : 2e-2;
            return nearZeroDefaultTrue(violation, static_cast<Scalar>(tol));
        }

        template <typename Scalar>
        DMat<Scalar> GenericImplicit<Scalar>::runCasadiFcn(const casadi::Function &fcn,
                                                           const JointCoordinate<Scalar> &arg)
        {
            // For complex types, extract real part for CasADi evaluation
            // (CasADi functions are real-valued)
            if constexpr (std::is_same_v<Scalar, std::complex<double>>) {
                DVec<double> arg_real(arg.size());
                for (int i = 0; i < arg.size(); ++i) {
                    arg_real(i) = arg(i).real();
                }
                
                casadi::DM arg_dm;
                casadi::copy(arg_real, arg_dm);
                
                const auto t0 = std::chrono::high_resolution_clock::now();
                casadi::DM res_dm = fcn(arg_dm)[0];
                const auto t1 = std::chrono::high_resolution_clock::now();
                profiling::addCasadiUs(std::chrono::duration<double, std::micro>(t1 - t0).count());
                
                // Convert result through double then cast to complex
                DMat<double> res_double(res_dm.size1(), res_dm.size2());
                casadi::copy(res_dm, res_double);
                
                // Cast to complex (imaginary part is zero, but that's OK for constraint evaluation)
                DMat<std::complex<double>> res = res_double.template cast<std::complex<double>>();
                return res;
            } else {
                // For real types, use standard copy
                casadi::DM arg_dm;
                casadi::copy(arg, arg_dm);
                
                const auto t0 = std::chrono::high_resolution_clock::now();
                casadi::DM res_dm = fcn(arg_dm)[0];
                const auto t1 = std::chrono::high_resolution_clock::now();
                profiling::addCasadiUs(std::chrono::duration<double, std::micro>(t1 - t0).count());
                
                // Convert through double to handle float specialization
                DMat<double> res_double(res_dm.size1(), res_dm.size2());
                casadi::copy(res_dm, res_double);
                
                // Cast to target scalar type
                DMat<Scalar> res = res_double.template cast<Scalar>();
                return res;
            }
        }

        template <typename Scalar>
        DMat<Scalar> GenericImplicit<Scalar>::runCasadiFcn(const casadi::Function &fcn,
                                                           const JointState<Scalar> &args)
        {
            // For complex types, extract real parts for CasADi evaluation
            // (CasADi functions are real-valued)
            if constexpr (std::is_same_v<Scalar, std::complex<double>>) {
                DVec<double> pos_real(args.position.size());
                for (int i = 0; i < args.position.size(); ++i) {
                    pos_real(i) = args.position(i).real();
                }
                
                DVec<double> vel_real(args.velocity.size());
                for (int i = 0; i < args.velocity.size(); ++i) {
                    vel_real(i) = args.velocity(i).real();
                }
                
                casadi::DM pos_dm, vel_dm;
                casadi::copy(pos_real, pos_dm);
                casadi::copy(vel_real, vel_dm);
                
                std::vector<casadi::DM> arg_vec = {pos_dm, vel_dm};
                const auto t0 = std::chrono::high_resolution_clock::now();
                std::vector<casadi::DM> res_vec = fcn(arg_vec);
                const auto t1 = std::chrono::high_resolution_clock::now();
                profiling::addCasadiUs(std::chrono::duration<double, std::micro>(t1 - t0).count());
                casadi::DM res_dm = res_vec[0];
                
                // Convert result through double then cast to complex
                DMat<double> res_double(res_dm.size1(), res_dm.size2());
                casadi::copy(res_dm, res_double);
                
                // Cast to complex (imaginary part is zero, but that's OK for constraint evaluation)
                DMat<std::complex<double>> res = res_double.template cast<std::complex<double>>();
                return res;
            } else {
                // For real types, use standard copy
                casadi::DM pos_dm, vel_dm;
                casadi::copy(args.position, pos_dm);
                casadi::copy(args.velocity, vel_dm);
                
                std::vector<casadi::DM> arg_vec = {pos_dm, vel_dm};
                const auto t0 = std::chrono::high_resolution_clock::now();
                std::vector<casadi::DM> res_vec = fcn(arg_vec);
                const auto t1 = std::chrono::high_resolution_clock::now();
                profiling::addCasadiUs(std::chrono::duration<double, std::micro>(t1 - t0).count());
                casadi::DM res_dm = res_vec[0];
                
                // Convert through double to handle float specialization
                DMat<double> res_double(res_dm.size1(), res_dm.size2());
                casadi::copy(res_dm, res_double);
                
                // Cast to target scalar type
                DMat<Scalar> res = res_double.template cast<Scalar>();
                return res;
            }
        }

        // Helper function to evaluate CasADi function with real-valued inputs (single arg)
        template <typename Scalar>
        DMat<double> GenericImplicit<Scalar>::runCasadiFcnReal(const casadi::Function &fcn,
                                                               const DVec<double> &arg)
        {
            casadi::DM arg_dm;
            casadi::copy(arg, arg_dm);
            const auto t0 = std::chrono::high_resolution_clock::now();
            casadi::DM res_dm = fcn(arg_dm)[0];
            const auto t1 = std::chrono::high_resolution_clock::now();
            profiling::addCasadiUs(std::chrono::duration<double, std::micro>(t1 - t0).count());
            DMat<double> res(res_dm.size1(), res_dm.size2());
            casadi::copy(res_dm, res);
            return res;
        }

        // Helper function to evaluate CasADi function with real-valued inputs (two args)
        template <typename Scalar>
        DMat<double> GenericImplicit<Scalar>::runCasadiFcnReal(const casadi::Function &fcn,
                                                               const DVec<double> &pos,
                                                               const DVec<double> &vel)
        {
            casadi::DM pos_dm, vel_dm;
            casadi::copy(pos, pos_dm);
            casadi::copy(vel, vel_dm);
            std::vector<casadi::DM> arg_vec = {pos_dm, vel_dm};
            const auto t0 = std::chrono::high_resolution_clock::now();
            std::vector<casadi::DM> res_vec = fcn(arg_vec);
            const auto t1 = std::chrono::high_resolution_clock::now();
            profiling::addCasadiUs(std::chrono::duration<double, std::micro>(t1 - t0).count());
            casadi::DM res_dm = res_vec[0];
            DMat<double> res(res_dm.size1(), res_dm.size2());
            casadi::copy(res_dm, res);
            return res;
        }

        // Complex-step aware evaluation of K
        // Uses TAYLOR SERIES EXPANSION to avoid finite-difference errors:
        //   K(q + i*δq) = K(q) + i * (dK/dq @ δq)
        //
        // This achieves machine precision for complex-step differentiation by using
        // CasADi's analytical derivatives instead of numerical finite differences.
        template <typename Scalar>
        DMat<Scalar> GenericImplicit<Scalar>::evalK(const JointCoordinate<Scalar> &joint_pos) const
        {
            if constexpr (std::is_same_v<Scalar, std::complex<double>>) {
                const int n = joint_pos.size();

                // TAYLOR SERIES EXPANSION using CasADi symbolic derivatives
                // K(q + i*δq) = K(q_real) + i * (dK/dq @ q_imag)
                // This avoids finite-difference errors that limit accuracy to ~1e-4
                DVec<double> q_real(n), q_imag(n);
                for (int i = 0; i < n; ++i) {
                    q_real(i) = joint_pos(i).real();
                    q_imag(i) = joint_pos(i).imag();
                }

                // Evaluate K at real part
                DMat<double> K_real = runCasadiFcnReal(K_fcn_, q_real);
                const int rows = K_real.rows();
                const int cols = K_real.cols();

                // Evaluate dK/dq at real part (vectorized K, so output is (rows*cols) x n)
                DMat<double> dK_dq = runCasadiFcnReal(dK_dq_fcn_, q_real);

                // Compute imaginary part: dK/dq @ q_imag
                // dK_dq is (rows*cols) x n, q_imag is n x 1
                // Result is (rows*cols) x 1, reshape to rows x cols
                DVec<double> K_imag_vec = dK_dq * q_imag;

                // Build complex result
                // CasADi vec() uses column-major: element K[i,j] is at index j*rows + i
                DMat<std::complex<double>> K_complex(rows, cols);
                for (int i = 0; i < rows; ++i) {
                    for (int j = 0; j < cols; ++j) {
                        int idx = j * rows + i;  // Column-major indexing
                        K_complex(i, j) = std::complex<double>(K_real(i, j), K_imag_vec(idx));
                    }
                }
                return K_complex;
            } else {
                // For real types, use standard evaluation
                return runCasadiFcn(K_fcn_, joint_pos);
            }
        }

        // Complex-step aware evaluation of G
        // G is computed from K via implicit function theorem: G = [I; -Kd^{-1} * Ki]
        //
        // Uses TAYLOR SERIES EXPANSION to avoid finite-difference errors:
        //   G(q + i*δq) = G(q) + i * (dG/dq @ δq)
        //
        // This achieves machine precision for complex-step differentiation by using
        // CasADi's analytical derivatives instead of numerical finite differences.
        template <typename Scalar>
        DMat<Scalar> GenericImplicit<Scalar>::evalG(const JointCoordinate<Scalar> &joint_pos) const
        {
            if constexpr (std::is_same_v<Scalar, std::complex<double>>) {
                const int n = joint_pos.size();

                // TAYLOR SERIES EXPANSION using CasADi symbolic derivatives
                // G(q + i*δq) = G(q_real) + i * (dG/dq @ q_imag)
                // This avoids finite-difference errors that limit accuracy to ~1e-4
                DVec<double> q_real(n), q_imag(n);
                for (int i = 0; i < n; ++i) {
                    q_real(i) = joint_pos(i).real();
                    q_imag(i) = joint_pos(i).imag();
                }

                DMat<double> G_real = runCasadiFcnReal(G_fcn_, q_real);
                const int rows = G_real.rows();
                const int cols = G_real.cols();

                DMat<double> dG_dq = runCasadiFcnReal(dG_dq_fcn_, q_real);
                // dG_dq has shape (n_G_elements, n_q) where n_G_elements = rows * cols
                // dG_dq @ q_imag gives the change in vec(G) for imaginary perturbation
                DVec<double> G_imag_vec = dG_dq * q_imag;

                DMat<std::complex<double>> G_complex(rows, cols);
                for (int i = 0; i < rows; ++i) {
                    for (int j = 0; j < cols; ++j) {
                        // CasADi vec() uses column-major: element G[i,j] is at index j*rows + i
                        int idx = j * rows + i;
                        G_complex(i, j) = std::complex<double>(G_real(i, j), G_imag_vec(idx));
                    }
                }

                // Debug: check if G has non-zero imaginary parts
                double max_G_imag = 0.0;
                for (int i = 0; i < rows; ++i) {
                    for (int j = 0; j < cols; ++j) {
                        max_G_imag = std::max(max_G_imag, std::abs(G_complex(i,j).imag()));
                    }
                }
                if (max_G_imag > 1e-25) {
                    std::cout << "[DEBUG evalG Taylor] G has imag, max|G_imag|=" << max_G_imag << std::endl;
                    std::cout << "  q_imag norm=" << q_imag.norm() << std::endl;
                }

                return G_complex;
            } else {
                // For real types, use standard CasADi evaluation
                return runCasadiFcn(G_fcn_, joint_pos);
            }
        }

        // Complex-step aware evaluation of k (constraint bias)
        // k = -K̇ · v where K̇ = dK/dt = sum_j (dK/dq_j * qd_j)
        //
        // Uses Taylor expansion: k(q + i*dq, v + i*dv) ≈ k(q,v) + i * (dk/dq * dq + dk/dv * dv)
        //
        // For velocity derivatives (dtau/dqdot), position has no imaginary part (dq=0),
        // so the expansion becomes: k(q, v + i*dv) = k(q,v) + i * (dk/dv * dv)
        // This is EXACT (not approximate) since k is linear in v.
        template <typename Scalar>
        DMat<Scalar> GenericImplicit<Scalar>::evalk(const JointState<Scalar> &joint_state) const
        {
            if constexpr (std::is_same_v<Scalar, std::complex<double>>) {
                const int n_pos = joint_state.position.size();
                const int n_vel = joint_state.velocity.size();

                // Taylor expansion approach using CasADi symbolic derivatives
                DVec<double> q_real(n_pos), q_imag(n_pos);
                for (int i = 0; i < n_pos; ++i) {
                    q_real(i) = joint_state.position(i).real();
                    q_imag(i) = joint_state.position(i).imag();
                }
                DVec<double> v_real(n_vel), v_imag(n_vel);
                for (int i = 0; i < n_vel; ++i) {
                    v_real(i) = joint_state.velocity(i).real();
                    v_imag(i) = joint_state.velocity(i).imag();
                }

                DMat<double> k_real = runCasadiFcnReal(k_fcn_, q_real, v_real);
                DMat<double> dk_dq = runCasadiFcnReal(dk_dq_fcn_, q_real, v_real);
                DMat<double> dk_dv = runCasadiFcnReal(dk_dv_fcn_, q_real, v_real);

                DVec<double> k_imag = dk_dq * q_imag + dk_dv * v_imag;

                const int rows = k_real.rows();
                DMat<std::complex<double>> k_complex(rows, 1);
                for (int i = 0; i < rows; ++i) {
                    k_complex(i, 0) = std::complex<double>(k_real(i, 0), k_imag(i));
                }
                return k_complex;
            } else {
                return runCasadiFcn(k_fcn_, joint_state);
            }
        }

        // Complex-step aware evaluation of g (explicit constraint bias)
        // g = [0; Kd^{-1} · k] mapped to spanning coordinates
        //
        // Uses Taylor expansion: g(q + i*dq, v + i*dv) ≈ g(q,v) + i * (dg/dq * dq + dg/dv * dv)
        //
        // For velocity derivatives (dtau/dqdot), position has no imaginary part (dq=0),
        // so the expansion becomes: g(q, v + i*dv) = g(q,v) + i * (dg/dv * dv)
        // This is EXACT since g depends linearly on v (through k which is linear in v).
        template <typename Scalar>
        DMat<Scalar> GenericImplicit<Scalar>::evalg(const JointState<Scalar> &joint_state) const
        {
            if constexpr (std::is_same_v<Scalar, std::complex<double>>) {
                const int n_pos = joint_state.position.size();
                const int n_vel = joint_state.velocity.size();

                // Taylor expansion approach using CasADi symbolic derivatives
                DVec<double> q_real(n_pos), q_imag(n_pos);
                for (int i = 0; i < n_pos; ++i) {
                    q_real(i) = joint_state.position(i).real();
                    q_imag(i) = joint_state.position(i).imag();
                }
                DVec<double> v_real(n_vel), v_imag(n_vel);
                for (int i = 0; i < n_vel; ++i) {
                    v_real(i) = joint_state.velocity(i).real();
                    v_imag(i) = joint_state.velocity(i).imag();
                }

                DMat<double> g_real = runCasadiFcnReal(g_fcn_, q_real, v_real);
                DMat<double> dg_dq = runCasadiFcnReal(dg_dq_fcn_, q_real, v_real);
                DMat<double> dg_dv = runCasadiFcnReal(dg_dv_fcn_, q_real, v_real);

                DVec<double> g_imag = dg_dq * q_imag + dg_dv * v_imag;

                const int rows = g_real.rows();
                DMat<std::complex<double>> g_complex(rows, 1);
                for (int i = 0; i < rows; ++i) {
                    g_complex(i, 0) = std::complex<double>(g_real(i, 0), g_imag(i));
                }
                return g_complex;
            } else {
                return runCasadiFcn(g_fcn_, joint_state);
            }
        }

        template <typename Scalar>
        void GenericImplicit<Scalar>::createRandomStateHelpers()
        {
            if (this->random_state_helpers_.created)
            {
                return;
            }
            this->random_state_helpers_.created = true;

            // Create symbolic generic implicit loop constraint
            GenericImplicit<SX> symbolic = copyAsSymbolic();

            // Root finding
            {
                SX cs_q_sym = SX::sym("q", this->numSpanningPos());
                DVec<SX> q_sym(this->numSpanningPos());
                casadi::copy(cs_q_sym, q_sym);

                // Compuate constraint violation
                JointCoordinate<SX> joint_pos(q_sym, true);
                DVec<SX> phi_sx = symbolic.phi(joint_pos);
                SX cs_phi_sym = casadi::SX(casadi::Sparsity::dense(phi_sx.rows(), 1));
                casadi::copy(phi_sx, cs_phi_sym);

                // Slice depending on independent coordinate
                std::vector<int> ind_coords, dep_coords;
                for (int i = 0; i < this->numSpanningPos(); i++)
                {
                    if (this->is_coordinate_independent_[i])
                        ind_coords.push_back(i);
                    else
                        dep_coords.push_back(i);
                }

                // Create rootfinder problem
                casadi::SXDict rootfinder_problem;
                rootfinder_problem["x"] = cs_q_sym(dep_coords);
                rootfinder_problem["p"] = cs_q_sym(ind_coords);
                rootfinder_problem["g"] = cs_phi_sym;
                casadi::Dict options;
                options["expand"] = true;   
                options["error_on_fail"] = true;  // Fail fast on non-convergence
                this->random_state_helpers_.phi_root_finder = casadi::rootfinder("solver", "newton",
                                                                                 rootfinder_problem,
                                                                                 options);
            }

            // Explicit constraint Jacobian
            {
                SX cs_q_sym = SX::sym("q", this->numSpanningPos());
                DVec<SX> q_sym(this->numSpanningPos());
                casadi::copy(cs_q_sym, q_sym);
                JointCoordinate<SX> joint_pos(q_sym, true);
                symbolic.updateJacobians(joint_pos);
                DMat<SX> G = symbolic.G();
                SX G_sym = casadi::SX(casadi::Sparsity::dense(G.rows(), G.cols()));
                casadi::copy(G, G_sym);
                this->random_state_helpers_.G = casadi::Function("G", {cs_q_sym}, {G_sym}, {"q"}, {"G"});
            }
        }

        // Solve constraints phi(y, q_dep) = 0 for q_dep given (possibly complex) independent coords y
        // Uses Newton iteration with native phi for machine-precision complex-step differentiation
        // Returns the full spanning coordinates q = [q_ind, q_dep] in proper order
        template <typename Scalar>
        DVec<Scalar> GenericImplicit<Scalar>::solveConstraintsComplex(
            const DVec<Scalar>& y_independent,
            const DVec<Scalar>& q_dep_init,
            int max_iters,
            double tol) const
        {
            // This function only makes sense for complex types - used for complex-step differentiation
            if constexpr (std::is_same_v<Scalar, std::complex<double>>) {
                if (!has_native_phi_) {
                    throw std::runtime_error(
                        "solveConstraintsComplex requires native phi function for complex-step support");
                }

                const int n = is_coordinate_independent_.size();

                // Identify independent and dependent coordinate indices
                std::vector<int> ind_coords, dep_coords;
                for (int i = 0; i < n; i++) {
                    if (is_coordinate_independent_[i])
                        ind_coords.push_back(i);
                    else
                        dep_coords.push_back(i);
                }
                const int ind_dim = ind_coords.size();
                const int dep_dim = dep_coords.size();

                if (y_independent.size() != ind_dim) {
                    throw std::runtime_error("y_independent size mismatch");
                }
                if (q_dep_init.size() != dep_dim) {
                    throw std::runtime_error("q_dep_init size mismatch");
                }

                // Initialize dependent coordinates
                DVec<Scalar> q_dep = q_dep_init;

                // For complex-step differentiation, we need to find q_dep such that:
                //   phi(y_independent, q_dep) = 0
                //
                // When y_independent = y_real + i*h*e_j (perturbing the j-th independent coord),
                // and assuming phi(y_real, q_dep_real) = 0, the solution has form:
                //   q_dep = q_dep_real + i * q_dep_imag
                //
                // To first order (which is exact for analytic functions):
                //   phi(y_real + i*h*e_j, q_dep_real + i*q_dep_imag)
                //     ≈ phi(y_real, q_dep_real) + i * (dPhi/dy * h * e_j + dPhi/dq_dep * q_dep_imag)
                //     = 0 + i * (Ki * h * e_j + Kd * q_dep_imag) = 0
                //
                // So: q_dep_imag = -Kd^{-1} * Ki * h * e_j
                //
                // This is exactly the first-order formula using G = -Kd^{-1}*Ki, but we compute it
                // via Newton iteration to handle the general case correctly.

                const int m = (int)phi_native_(JointCoordinate<Scalar>(DVec<Scalar>::Zero(n), true)).size();

                // Helper lambda to compute K using CasADi K_fcn_ (analytically, no finite differences)
                auto computeK = [this, n, m](const DVec<double>& q_spanning_real) -> DMat<double> {
                    std::vector<double> q_vec(n);
                    for (int j = 0; j < n; ++j) {
                        q_vec[j] = q_spanning_real(j);
                    }
                    casadi::DM q_dm(q_vec);
                    casadi::DM K_dm = K_fcn_(casadi::DMVector{q_dm})[0];

                    DMat<double> K_real(m, n);
                    for (int i = 0; i < m; ++i) {
                        for (int j = 0; j < n; ++j) {
                            K_real(i, j) = static_cast<double>(K_dm(i, j));
                        }
                    }
                    return K_real;
                };

                // Extract real parts of the input
                DVec<double> y_ind_real(ind_dim), q_dep_real(dep_dim);
                DVec<double> y_ind_imag(ind_dim), q_dep_imag(dep_dim);
                for (int i = 0; i < ind_dim; ++i) {
                    y_ind_real(i) = y_independent(i).real();
                    y_ind_imag(i) = y_independent(i).imag();
                }
                for (int i = 0; i < dep_dim; ++i) {
                    q_dep_real(i) = q_dep_init(i).real();
                    q_dep_imag(i) = 0.0;  // Start with zero imaginary part
                }

                // Build full spanning coordinate vector (real)
                DVec<double> q_spanning_real(n);
                for (int i = 0; i < ind_dim; ++i) {
                    q_spanning_real(ind_coords[i]) = y_ind_real(i);
                }
                for (int i = 0; i < dep_dim; ++i) {
                    q_spanning_real(dep_coords[i]) = q_dep_real(i);
                }

                // First, do Newton iteration on real parts if needed
                for (int iter = 0; iter < max_iters; ++iter) {
                    // Evaluate phi at real point
                    DVec<Scalar> q_spanning_c(n);
                    for (int j = 0; j < n; ++j) {
                        q_spanning_c(j) = Scalar(q_spanning_real(j), 0.0);
                    }
                    JointCoordinate<Scalar> jc(q_spanning_c, true);
                    DVec<Scalar> phi_c = phi_native_(jc);

                    double phi_norm = 0.0;
                    for (int i = 0; i < m; ++i) {
                        phi_norm += phi_c(i).real() * phi_c(i).real();
                    }
                    phi_norm = std::sqrt(phi_norm);

                    if (phi_norm < tol) {
                        break;
                    }

                    // Compute K using CasADi (analytically)
                    DMat<double> K_real = computeK(q_spanning_real);

                    // Extract Kd
                    DMat<double> Kd_real(m, dep_dim);
                    for (int i = 0; i < dep_dim; ++i) {
                        Kd_real.col(i) = K_real.col(dep_coords[i]);
                    }

                    // Newton step for real part
                    Eigen::PartialPivLU<DMat<double>> lu(Kd_real);
                    DVec<double> phi_real_vec(m);
                    for (int i = 0; i < m; ++i) {
                        phi_real_vec(i) = phi_c(i).real();
                    }
                    DVec<double> delta = -lu.solve(phi_real_vec);

                    for (int i = 0; i < dep_dim; ++i) {
                        q_dep_real(i) += delta(i);
                        q_spanning_real(dep_coords[i]) = q_dep_real(i);
                    }
                }

                // Now compute the imaginary part of q_dep using the implicit function theorem
                // q_dep_imag = -Kd^{-1} * Ki * y_ind_imag
                // where Ki is the Jacobian of phi w.r.t. independent coords

                // Compute K at the converged real point using CasADi (analytically)
                DMat<double> K_real = computeK(q_spanning_real);

                // Extract Ki (columns for independent coords) and Kd (columns for dependent coords)
                DMat<double> Ki_real(m, ind_dim), Kd_real(m, dep_dim);
                for (int i = 0; i < ind_dim; ++i) {
                    Ki_real.col(i) = K_real.col(ind_coords[i]);
                }
                for (int i = 0; i < dep_dim; ++i) {
                    Kd_real.col(i) = K_real.col(dep_coords[i]);
                }

                // Compute q_dep_imag = -Kd^{-1} * Ki * y_ind_imag
                Eigen::PartialPivLU<DMat<double>> lu(Kd_real);
                DVec<double> rhs = Ki_real * y_ind_imag;
                q_dep_imag = -lu.solve(rhs);

                // Build final complex spanning coordinates
                for (int i = 0; i < dep_dim; ++i) {
                    q_dep(i) = Scalar(q_dep_real(i), q_dep_imag(i));
                }

                // Build final spanning coordinates
                DVec<Scalar> q_spanning(n);
                for (int i = 0; i < ind_dim; ++i) {
                    q_spanning(ind_coords[i]) = y_independent(i);
                }
                for (int i = 0; i < dep_dim; ++i) {
                    q_spanning(dep_coords[i]) = q_dep(i);
                }

                return q_spanning;
            } else {
                // For non-complex types, this function should not be called
                throw std::runtime_error(
                    "solveConstraintsComplex is only implemented for std::complex<double>");
            }
        }

        template struct GenericImplicit<double>;
        template struct GenericImplicit<std::complex<double>>;
        template struct GenericImplicit<float>;
        template struct GenericImplicit<casadi::SX>;
    }

    namespace ClusterJoints
    {

        template <typename Scalar>
        Generic<Scalar>::Generic(const std::vector<Body<Scalar>> &bodies,
                                 const std::vector<JointPtr<Scalar>> &joints,
                                 std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint)
            : Base<Scalar>((int)bodies.size(),
                           loop_constraint->isExplicit() ? loop_constraint->numIndependentPos()
                                                         : loop_constraint->numSpanningPos(),
                           loop_constraint->numIndependentVel()),
              bodies_(bodies)
        {
            initialize(joints, loop_constraint);
        }

        template <typename Scalar>
        Generic<Scalar>::Generic(const std::vector<Body<Scalar>> &bodies,
                                 const std::vector<JointPtr<Scalar>> &joints,
                                 std::shared_ptr<LoopConstraint::GenericImplicit<Scalar>> loop_constraint)
            : Base<Scalar>((int)bodies.size(),
                           loop_constraint->isExplicit() ? loop_constraint->numIndependentPos()
                                                         : loop_constraint->numSpanningPos(),
                           loop_constraint->numIndependentVel()),
              bodies_(bodies)
        {
            generic_constraint_ = loop_constraint;
            initialize(joints, loop_constraint);
        }

        template <typename Scalar>
        void
        Generic<Scalar>::initialize(const std::vector<JointPtr<Scalar>> &joints,
                                    std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint)
        {
            this->loop_constraint_ = loop_constraint;
            // Preserve GenericImplicit access even when this class is constructed
            // through the Base<Scalar> loop-constraint overload.
            generic_constraint_ = std::dynamic_pointer_cast<LoopConstraint::GenericImplicit<Scalar>>(loop_constraint);
            if (!generic_constraint_) {
                if constexpr (std::is_same_v<Scalar, double>) {
                    if (auto lazy_constraint = std::dynamic_pointer_cast<LoopConstraint::LazyGenericImplicit<Scalar>>(loop_constraint)) {
                        generic_constraint_ = std::make_shared<LoopConstraint::GenericImplicit<Scalar>>(lazy_constraint->copyAsDouble());
                    } else if (auto precompiled_constraint = std::dynamic_pointer_cast<LoopConstraint::PrecompiledGenericImplicit<Scalar>>(loop_constraint)) {
                        generic_constraint_ = std::make_shared<LoopConstraint::GenericImplicit<Scalar>>(precompiled_constraint->copyAsDouble());
                    }
                }
            }

            for (auto &joint : joints)
                this->single_joints_.push_back(joint);

            extractConnectivity();

            S_spanning_ = DMat<Scalar>::Zero(0, 0);
            for (auto &joint : joints)
                S_spanning_ = appendEigenMatrix(S_spanning_, joint->S());

            X_intra_ = DMat<Scalar>::Identity(6 * this->num_bodies_, 6 * this->num_bodies_);
            X_intra_ring_ = DMat<Scalar>::Zero(6 * this->num_bodies_, 6 * this->num_bodies_);
        }

        template <typename Scalar>
        JointCoordinate<double> Generic<Scalar>::findRootsForPhi() const
        {
            using DM = casadi::DM;

            using LoopConstraintPtr = const std::shared_ptr<LoopConstraint::Base<Scalar>>;
            LoopConstraintPtr &loop_constraint = this->loop_constraint_;

            const int n_ind = loop_constraint->numIndependentPos();
            const int n_span = loop_constraint->numSpanningPos();
            // TODO(@MatthewChignoli): Make this an input parameter
            double ind_range = 1.0;
            double dep_range = 0.1;

            DM q_ind, q_dep;
            bool solve_success = false;
            int num_attempts = 0;
            while (!solve_success && num_attempts++ < 45)
            {
                q_ind = ind_range * (2. * DM::rand(n_ind) - 1.);
                DM q_dep_guess = dep_range * (2. * DM::rand(n_span - n_ind) - 1.);

                casadi::DMDict arg;
                arg["p"] = q_ind;
                arg["x0"] = q_dep_guess;

                try
                {
                    q_dep = loop_constraint->random_state_helpers_.phi_root_finder(arg).at("x");
                    solve_success = true;
                }
                catch (const std::exception &e)
                {
                    solve_success = false;
                }
            }

            if (!solve_success)
            {
                throw std::runtime_error("Failed to find valid roots for implicit loop constraint");
            }

            DM q_dm(n_span, 1);
            int ind_cnt = 0, dep_cnt = 0;
            for (int i = 0; i < n_span; i++)
            {
                if (generic_constraint_->isCoordinateIndependent()[i])
                {
                    q_dm(i) = q_ind(ind_cnt++);
                }
                else
                {
                    q_dm(i) = q_dep(dep_cnt++);
                }
            }
            DVec<double> q(n_span);
            casadi::copy(q_dm, q);

            return JointCoordinate<double>(q, true);
        }

        template <typename Scalar>
        JointState<double> Generic<Scalar>::randomJointState() const
        {
            if (this->loop_constraint_->isExplicit())
               return Base<Scalar>::randomJointState();

            // Try to get constraint pointer (works for GenericImplicit, LazyGenericImplicit, and PrecompiledGenericImplicit)
            auto lazy_constraint = std::dynamic_pointer_cast<LoopConstraint::LazyGenericImplicit<Scalar>>(this->loop_constraint_);
            auto precompiled_constraint = std::dynamic_pointer_cast<LoopConstraint::PrecompiledGenericImplicit<Scalar>>(this->loop_constraint_);

            if (!generic_constraint_ && !lazy_constraint && !precompiled_constraint)
            {
                throw std::runtime_error("GenericImplicit loop constraint not set");
            }

            const int n_span = this->loop_constraint_->numSpanningPos();
            const int n_ind = this->loop_constraint_->numIndependentPos();
            const int n_dep = n_span - n_ind;

            // Build independent mask (works for all constraint types)
            std::vector<bool> ind_mask;
            if (generic_constraint_) {
                ind_mask = generic_constraint_->isCoordinateIndependent();
            } else if (lazy_constraint) {
                ind_mask = lazy_constraint->isCoordinateIndependent();
            } else if (precompiled_constraint) {
                ind_mask = precompiled_constraint->isCoordinateIndependent();
            }
            if ((int)ind_mask.size() != n_span) {
                ind_mask.assign(n_span, false);
                for (int i = 0; i < n_span; ++i) ind_mask[i] = (i < n_ind);
            }

            // Initialize q with random independent and small dependent values
            DVec<double> q_span = DVec<double>::Zero(n_span);
            for (int i = 0; i < n_span; ++i) {
                if (ind_mask[i]) q_span(i) = 0.3 * (2.0 * ((double)rand() / RAND_MAX) - 1.0);
                else q_span(i) = 0.01 * (2.0 * ((double)rand() / RAND_MAX) - 1.0);
            }

            // Use native phi if available for better accuracy
            // Only works when Scalar=double because the native phi function is templated on Scalar
            bool use_native = false;
            std::function<DVec<double>(const DVec<double>&)> phi_native_double;
            if constexpr (std::is_same_v<Scalar, double>) {
                if (generic_constraint_) {
                    use_native = generic_constraint_->hasNativePhi();
                    if (use_native) {
                        phi_native_double = [this](const DVec<double>& q) -> DVec<double> {
                            JointCoordinate<double> jc(q, true);
                            return generic_constraint_->nativePhi()(jc);
                        };
                    }
                } else if (lazy_constraint) {
                    use_native = lazy_constraint->hasNativePhi();
                    if (use_native) {
                        phi_native_double = [&lazy_constraint](const DVec<double>& q) -> DVec<double> {
                            JointCoordinate<double> jc(q, true);
                            return lazy_constraint->nativePhi()(jc);
                        };
                    }
                } else if (precompiled_constraint) {
                    use_native = precompiled_constraint->hasNativePhi();
                    if (use_native) {
                        phi_native_double = [&precompiled_constraint](const DVec<double>& q) -> DVec<double> {
                            JointCoordinate<double> jc(q, true);
                            return precompiled_constraint->nativePhi()(jc);
                        };
                    }
                }
            }
            auto numerical_lc = generic_constraint_ ? generic_constraint_->copyAsDouble() :
                                (lazy_constraint ? lazy_constraint->copyAsDouble() : precompiled_constraint->copyAsDouble());
            auto phi_eval = [&numerical_lc, use_native, &phi_native_double](const DVec<double> &q) -> DVec<double> {
                if (use_native) {
                    return phi_native_double(q);
                }
                JointCoordinate<double> jc(q, true);
                return numerical_lc.phi(jc);
            };

            // Newton solver with adaptive damping for robust convergence to machine precision
            // Uses undamped Newton when close to solution, damped when far
            const int max_iters = 200;
            const double tol_tight = 1e-12;  // Target machine precision
            const double tol_accept = 1e-10; // Acceptance tolerance
            const double h = 1e-8;           // FD step for Jacobian

            // Build dep indices
            std::vector<int> dep_idx; dep_idx.reserve(n_dep);
            for (int i = 0; i < n_span; ++i) if (!ind_mask[i]) dep_idx.push_back(i);

            bool converged = false;
            double best_phi_norm = 1e10;
            DVec<double> best_q_span = q_span;

            for (int attempt = 0; attempt < 30 && !converged; ++attempt) {
                double damping = 0.5;  // Start with damping for stability
                for (int iter = 0; iter < max_iters; ++iter) {
                    DVec<double> phi = phi_eval(q_span);
                    double phi_norm = phi.norm();

                    // Track best solution found
                    if (phi_norm < best_phi_norm) {
                        best_phi_norm = phi_norm;
                        best_q_span = q_span;
                    }

                    if (phi_norm < tol_tight) { converged = true; break; }

                    // Use undamped Newton when close to solution
                    if (phi_norm < 1e-3) damping = 1.0;
                    else if (phi_norm < 1e-2) damping = 0.8;
                    else damping = 0.5;

                    DMat<double> J(phi.size(), n_dep);
                    for (int j = 0; j < n_dep; ++j) {
                        DVec<double> q_pert = q_span;
                        q_pert(dep_idx[j]) += h;
                        J.col(j) = (phi_eval(q_pert) - phi) / h;
                    }
                    Eigen::CompleteOrthogonalDecomposition<DMat<double>> cod(J);
                    DVec<double> dx = cod.solve(-phi);
                    for (int j = 0; j < n_dep; ++j) q_span(dep_idx[j]) += damping * dx(j);
                }
                if (!converged) {
                    // reinitialize dependents with wider range
                    for (int i = 0; i < n_span; ++i) if (!ind_mask[i]) q_span(i) = 0.3 * (2.0 * ((double)rand() / RAND_MAX) - 1.0);
                }
            }

            // Use best solution found if not converged
            if (!converged) {
                q_span = best_q_span;
                converged = (best_phi_norm < tol_accept);
            }

            // Final phi check - use native phi for validation too when available
            DVec<double> phi_final = phi_eval(q_span);
            double final_phi_norm = phi_final.norm();

            // Validate using the same phi function that Newton used (native if available)
            // This ensures consistency between Newton convergence and validation
            bool is_valid = (final_phi_norm < 1e-8);  // Use our own tolerance since we know phi

            if (!converged || !is_valid) {
                std::cerr << "[Newton debug] converged=" << converged
                          << ", best_phi_norm=" << best_phi_norm
                          << ", final_phi_norm=" << final_phi_norm
                          << ", is_valid=" << is_valid
                          << ", use_native=" << use_native << std::endl;
                throw std::runtime_error("Failed to sample valid spanning state for implicit constraint");
            }

            // Return with independent velocities
            DVec<double> ydot = DVec<double>::Random(this->loop_constraint_->numIndependentVel());
            JointCoordinate<double> pos(q_span, true);
            JointCoordinate<double> vel(ydot, false);
            return JointState<double>(pos, vel);
        }

        template <typename Scalar>
        void Generic<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            const DVec<Scalar> &q = spanning_joint_state.position;
            const DVec<Scalar> &qd = spanning_joint_state.velocity;

            // Cache state for derivative computation
            q_cache_ = q;
            qd_cache_ = qd;
            S_q_cache_valid_ = false; // state changed, invalidate derivative cache
            Sdotqd_q_cache_valid_ = false;

            int pos_idx = 0;
            int vel_idx = 0;
            for (int i = 0; i < this->num_bodies_; i++)
            {
                const auto &body = bodies_[i];
                auto joint = this->single_joints_[i];

                const int num_pos = joint->numPositions();
                const int num_vel = joint->numVelocities();

                joint->updateKinematics(q.segment(pos_idx, num_pos), qd.segment(vel_idx, num_vel));

                int k = i;
                for (int j = i - 1; j >= 0; j--)
                {
                    if (connectivity_(i, j))
                    {
                        const auto &body_k = bodies_[k];
                        const auto joint_k = this->single_joints_[k];

                        const Mat6<Scalar> Xup_prev = X_intra_.template block<6, 6>(6 * i, 6 * k);
                        const Mat6<Scalar> Xint = (joint_k->XJ() * body_k.Xtree_).toMatrix();
                        X_intra_.template block<6, 6>(6 * i, 6 * j) = Xup_prev * Xint;

                        k = j;
                    }
                }

                pos_idx += num_pos;
                vel_idx += num_vel;
            }

            S_implicit_ = X_intra_ * S_spanning_;
            this->S_ = S_implicit_ * this->loop_constraint_->G();
            this->vJ_ = S_implicit_ * qd;

            for (int i = 0; i < this->num_bodies_; i++)
            {
                SVec<Scalar> v_relative = SVec<Scalar>::Zero();
                for (int j = i - 1; j >= 0; j--)
                {
                    if (connectivity_(i, j))
                    {
                        const Mat6<Scalar> Xup = X_intra_.template block<6, 6>(6 * i, 6 * j);

                        const SVec<Scalar> v_parent = Xup * this->vJ_.template segment<6>(6 * j);
                        const SVec<Scalar> v_child = this->vJ_.template segment<6>(6 * i);
                        v_relative = v_child - v_parent;

                        X_intra_ring_.template block<6, 6>(6 * i, 6 * j) =
                            -spatial::motionCrossMatrix(v_relative) * Xup;
                    }
                }
            }

            this->cJ_ = X_intra_ring_ * this->S_spanning_ * qd +
                        S_implicit_ * this->loop_constraint_->g();

            // S_ring = dS/dt = d(X_intra * S_spanning * G)/dt
            //        = dX_intra/dt * S_spanning * G + X_intra * S_spanning * dG/dt
            // where dG/dt = sum_j (dG/dq_j * qd_j)
            //
            // For GenericImplicit constraints, we compute dG/dt using the constraint's
            // dG/dq CasADi function which is properly initialized in the constructor.
            DMat<Scalar> S_ring_term1 = X_intra_ring_ * this->S_spanning_ * this->loop_constraint_->G();
            DMat<Scalar> S_ring_term2 = DMat<Scalar>::Zero(S_ring_term1.rows(), S_ring_term1.cols());

            // Compute G_dot for both double and complex types
            // For complex types, use the real part of q to evaluate dG/dq (valid for small imaginary parts)
            if constexpr (std::is_same_v<Scalar, double> || std::is_same_v<Scalar, std::complex<double>>) {
                if (generic_constraint_ && q_cache_.size() > 0) {
                    // Get the constraint's dG/dq function (initialized in constructor)
                    const casadi::Function& dG_dq_fcn = generic_constraint_->getdGdqFcn();

                    // Evaluate dG/dq at current position (real part only for complex types)
                    std::vector<double> q_vec(q_cache_.size());
                    for (int i = 0; i < q_cache_.size(); ++i) {
                        if constexpr (std::is_same_v<Scalar, std::complex<double>>) {
                            q_vec[i] = std::real(q_cache_(i));
                        } else {
                            q_vec[i] = q_cache_(i);
                        }
                    }
                    casadi::DM q_dm(q_vec);
                    casadi::DM dG_dq_dm = dG_dq_fcn(casadi::DMVector{q_dm})[0];

                    // dG_dq_dm has shape (n_G_elements, n_q) where n_G_elements = G.rows() * G.cols()
                    // G_dot = sum_j (dG/dq_j * qd_j) = dG_dq * qd (matrix-vector product)
                    const int n_q = q_cache_.size();
                    const int n_G_rows = this->loop_constraint_->G().rows();
                    const int n_G_cols = this->loop_constraint_->G().cols();
                    const int n_G_elements = n_G_rows * n_G_cols;

                    // Compute G_dot_vec = dG_dq * qd
                    DVec<Scalar> G_dot_vec = DVec<Scalar>::Zero(n_G_elements);
                    for (int i = 0; i < n_G_elements; ++i) {
                        for (int j = 0; j < n_q; ++j) {
                            G_dot_vec(i) += Scalar(static_cast<double>(dG_dq_dm(i, j))) * qd(j);
                        }
                    }

                    // Reshape G_dot_vec to G_dot matrix (column-major order)
                    DMat<Scalar> G_dot(n_G_rows, n_G_cols);
                    for (int col = 0; col < n_G_cols; ++col) {
                        for (int row = 0; row < n_G_rows; ++row) {
                            G_dot(row, col) = G_dot_vec(col * n_G_rows + row);
                        }
                    }

                    // Second term: X_intra * S_spanning * G_dot
                    S_ring_term2 = S_implicit_ * G_dot;
                }
            }

            this->S_ring_ = S_ring_term1 + S_ring_term2;
        }

        template <typename Scalar>
        void Generic<Scalar>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
        {
            for (int i = 0; i < this->num_bodies_; i++)
            {
                const auto &body = bodies_[i];
                const auto joint = this->single_joints_[i];
                Xup[i] = joint->XJ() * body.Xtree_;
                for (int j = i - 1; j >= 0; j--)
                    if (connectivity_(i, j))
                    {
                        Xup[i] = Xup[i] * Xup[j];
                        break;
                    }
            }
        }

        template <typename Scalar>
        void Generic<Scalar>::extractConnectivity()
        {
            connectivity_ = DMat<bool>::Zero(this->num_bodies_, this->num_bodies_);
            for (int i = 0; i < this->num_bodies_; i++)
            {
                int j = i;
                while (bodyInCurrentCluster(bodies_[j].parent_index_))
                {
                    const Body<Scalar> &parent_body = getBody(bodies_[j].parent_index_);
                    j = parent_body.sub_index_within_cluster_;
                    connectivity_(i, j) = true;
                }
            }
        }

        template <typename Scalar>
        bool Generic<Scalar>::bodyInCurrentCluster(const int body_index) const
        {
            for (const auto &body : bodies_)
                if (body.index_ == body_index)
                    return true;
            return false;
        }

        template <typename Scalar>
        const Body<Scalar> &Generic<Scalar>::getBody(const int body_index) const
        {
            for (const auto &body : bodies_)
                if (body.index_ == body_index)
                    return body;
            throw std::runtime_error("Body is not in the current cluster");
}

        template <typename Scalar>
        void Generic<Scalar>::initializeDerivativeFunctions() const
        {
            if (derivative_functions_initialized_ || !generic_constraint_) {
                return;
            }
            derivative_functions_initialized_ = true;

            // Only initialize for double type
            if constexpr (!std::is_same_v<Scalar, double>) {
                return;
            }

            // Create symbolic constraint to compute dG/dq
            using SX = casadi::SX;
            auto symbolic_constraint = generic_constraint_->copyAsSymbolic();

            const int n_span_pos = this->loop_constraint_->numSpanningPos();
            const int n_span_vel = this->loop_constraint_->numSpanningVel();

            // Symbolic spanning positions and velocities
            SX q_span_sx = SX::sym("q_span", n_span_pos);
            DVec<SX> q_span_vec(n_span_pos);
            casadi::copy(q_span_sx, q_span_vec);
            JointCoordinate<SX> joint_pos_sx(q_span_vec, true);

            SX qd_span_sx = SX::sym("qd_span", n_span_vel);
            DVec<SX> qd_span_vec(n_span_vel);
            casadi::copy(qd_span_sx, qd_span_vec);
            JointCoordinate<SX> vel_pos_sx(qd_span_vec, false);

            // Update constraint Jacobians and biases with symbolic state
            symbolic_constraint.updateJacobians(joint_pos_sx);
            DMat<SX> G_sx = symbolic_constraint.G();
            JointState<SX> joint_state_sx(joint_pos_sx, vel_pos_sx);
            symbolic_constraint.updateBiases(joint_state_sx);
            DMat<SX> g_sx = symbolic_constraint.g();

            // Convert to CasADi matrices
            SX G_casadi = SX::zeros(G_sx.rows(), G_sx.cols());
            casadi::copy(G_sx, G_casadi);
            SX g_casadi = SX::zeros(g_sx.rows(), g_sx.cols());
            casadi::copy(g_sx, g_casadi);

            // Compute dG/dq using CasADi automatic differentiation
            std::vector<SX> dG_dq_vec;
            for (int i = 0; i < n_span_pos; ++i) {
                SX dG_dqi = jacobian(G_casadi, q_span_sx(i));
                dG_dq_vec.push_back(dG_dqi);
            }
            SX dG_dq_stacked = SX::vertcat(dG_dq_vec);
            dG_dq_fcn_ = casadi::Function("dG_dq", {q_span_sx}, {dG_dq_stacked});

            // Compute jacobians of g with respect to q and qd
            SX dg_dq_sx = jacobian(g_casadi, q_span_sx);
            SX dg_dqd_sx = jacobian(g_casadi, qd_span_sx);

            // Create functions
            dSdotqd_dq_fcn_ = casadi::Function("dSdotqd_dq", {q_span_sx, qd_span_sx}, {dg_dq_sx});
            dSdotqd_dqd_fcn_ = casadi::Function("dSdotqd_dqd", {q_span_sx, qd_span_sx}, {dg_dqd_sx});
        }


        template <typename Scalar>
        std::vector<DMat<Scalar>> Generic<Scalar>::getSq() const
        {
            struct ScopedGetSqTimer {
                bool enabled;
                std::chrono::high_resolution_clock::time_point start;
                ~ScopedGetSqTimer() {
                    if (enabled) {
                        const auto end = std::chrono::high_resolution_clock::now();
                        profiling::addGetSqInternalUs(
                            std::chrono::duration<double, std::micro>(end - start).count());
                    }
                }
            } timer{profiling::isEnabled() && std::is_same_v<Scalar, double>,
                    std::chrono::high_resolution_clock::now()};

            const int mss_dim = this->num_bodies_ * 6;
            const int nv = this->num_velocities_;
            const int n_span_vel = this->loop_constraint_->numSpanningVel();
            static const bool force_uncached_getsq = []() {
                const char *env = std::getenv("GRBDA_ID_DERIV_FORCE_GETSQ_UNCACHED");
                return env != nullptr && env[0] != '0';
            }();

            if constexpr (std::is_same_v<Scalar, double>) {
                // Reuse cached result when possible to avoid repeated CasADi evaluation
                if (!force_uncached_getsq && S_q_cache_valid_ && (int)S_q_cache_.size() == nv) {
                    return S_q_cache_;
                }

                S_q_cache_.assign(nv, DMat<Scalar>::Zero(mss_dim, nv));

                if (!generic_constraint_) {
                    S_q_cache_valid_ = true;
                    return S_q_cache_;
                }

                // Safety check: ensure state has been cached
                if (q_cache_.size() == 0 || S_implicit_.size() == 0) {
                    S_q_cache_valid_ = true;
                    return S_q_cache_;
                }

                const DMat<Scalar>& S_implicit = S_implicit_;
                const DMat<Scalar>& G = this->loop_constraint_->G();

                // Debug: Check if S_implicit contains NaN
                if (!S_implicit.allFinite()) {
                    std::cout << "[DEBUG getSq] S_implicit contains NaN/Inf!" << std::endl;
                    std::cout << "  X_intra_ finite: " << X_intra_.allFinite() << std::endl;
                    std::cout << "  S_spanning_ finite: " << S_spanning_.allFinite() << std::endl;
                }

                casadi::DM q_dm(q_cache_.size());
                casadi::copy(q_cache_, q_dm);

                // Use the constraint's dG/dq function (properly initialized in constructor)
                // instead of the joint's dG_dq_fcn_ which has issues with symbolic propagation
                const casadi::Function& dG_dq_constraint = generic_constraint_->getdGdqFcn();
                casadi::DMVector result = dG_dq_constraint(casadi::DMVector{q_dm});
                casadi::DM dG_dq_stacked_dm = result[0];

                const int n_span = G.rows();
                const int n_indep = G.cols();

                // The full derivative is: dS/dy_j = sum_k (dS/dq_k * G_kj)
                // where dS/dq_k = dX_intra/dq_k * S_spanning * G + X_intra * S_spanning * dG/dq_k
                //
                // First, compute dS/dq_k for each spanning coordinate k
                // Then contract with G to get dS/dy_j

                // Extract dG/dq matrices for all spanning coordinates
                // dG_dq_stacked_dm has shape (n_span * n_indep, n_span_pos) from CasADi jacobian
                // CasADi stores column-major, so column k contains dG/dq_k flattened
                const int n_span_pos = q_cache_.size();
                std::vector<DMat<Scalar>> dG_dq_k(n_span_pos);
                for (int k = 0; k < n_span_pos; ++k) {
                    dG_dq_k[k].resize(n_span, n_indep);
                    for (int row = 0; row < n_span; ++row) {
                        for (int col = 0; col < n_indep; ++col) {
                            // CasADi jacobian(vec(G), q) has shape (n_G_elements, n_q)
                            // vec(G) is column-major, so element G[row,col] is at index col*n_span + row
                            // dG[row,col]/dq[k] is at dG_dq_stacked_dm(col*n_span + row, k)
                            int vec_idx = col * n_span + row;
                            dG_dq_k[k](row, col) = static_cast<double>(dG_dq_stacked_dm(vec_idx, k));
                        }
                    }
                }

                // Compute d(X_intra * S_spanning)/dq_k with finite differences.
                // This is slower than the handcrafted path logic but substantially more robust
                // for complex implicit clusters such as Tello's differential mechanisms.
                std::vector<DMat<Scalar>> dXintra_Sspan_dq(n_span_pos, DMat<Scalar>::Zero(mss_dim, n_span_vel));

                std::vector<JointPtr<Scalar>> joints_local;
                joints_local.reserve(this->num_bodies_);
                std::vector<int> pos_offsets(this->num_bodies_);
                std::vector<int> vel_offsets(this->num_bodies_);
                std::vector<int> num_pos_per_joint(this->num_bodies_);
                std::vector<int> num_vel_per_joint(this->num_bodies_);

                int pos_cursor = 0;
                int vel_cursor = 0;
                for (int b = 0; b < this->num_bodies_; ++b) {
                    auto joint_clone = this->single_joints_[b]->clone();
                    const int num_pos_b = joint_clone->numPositions();
                    const int num_vel_b = joint_clone->numVelocities();
                    joints_local.push_back(joint_clone);
                    pos_offsets[b] = pos_cursor;
                    vel_offsets[b] = vel_cursor;
                    num_pos_per_joint[b] = num_pos_b;
                    num_vel_per_joint[b] = num_vel_b;
                    pos_cursor += num_pos_b;
                    vel_cursor += num_vel_b;
                }

                DMat<Scalar> S_spanning_local = DMat<Scalar>::Zero(mss_dim, n_span_vel);
                DMat<Scalar> X_intra_local = DMat<Scalar>::Identity(mss_dim, mss_dim);

                auto evalXintraSspanning = [&](const DVec<Scalar>& q_span_eval) -> DMat<Scalar> {
                    S_spanning_local.setZero();
                    X_intra_local.setIdentity();

                    for (int b = 0; b < this->num_bodies_; ++b) {
                        auto joint_b = joints_local[b];
                        const int num_pos_b = num_pos_per_joint[b];
                        const int num_vel_b = num_vel_per_joint[b];
                        const int pos_idx_b = pos_offsets[b];
                        const int vel_idx_b = vel_offsets[b];

                        DVec<Scalar> qd_zero = DVec<Scalar>::Zero(num_vel_b);
                        joint_b->updateKinematics(q_span_eval.segment(pos_idx_b, num_pos_b), qd_zero);
                        S_spanning_local.block(6 * b, vel_idx_b, 6, num_vel_b) = joint_b->S();

                        int k = b;
                        for (int j = b - 1; j >= 0; --j) {
                            if (connectivity_(b, j)) {
                                const auto& body_k = bodies_[k];
                                const auto joint_k = joints_local[k];
                                const Mat6<Scalar> Xup_prev = X_intra_local.template block<6, 6>(6 * b, 6 * k);
                                const Mat6<Scalar> Xint = (joint_k->XJ() * body_k.Xtree_).toMatrix();
                                X_intra_local.template block<6, 6>(6 * b, 6 * j) = Xup_prev * Xint;
                                k = j;
                            }
                        }
                    }

                    return X_intra_local * S_spanning_local;
                };

                const Scalar h = Scalar(1e-8);
                for (int k = 0; k < n_span_pos; ++k) {
                    DVec<Scalar> q_plus = q_cache_;
                    DVec<Scalar> q_minus = q_cache_;
                    q_plus(k) += h;
                    q_minus(k) -= h;
                    const DMat<Scalar> xs_plus = evalXintraSspanning(q_plus);
                    const DMat<Scalar> xs_minus = evalXintraSspanning(q_minus);
                    dXintra_Sspan_dq[k] = (xs_plus - xs_minus) / (Scalar(2) * h);
                }

                // Now compute dS/dy_j = sum_k (dS/dq_k * G_kj)
                // where dS/dq_k = dXintra_Sspan_dq[k] * G + S_implicit * dG_dq_k[k]

                for (int j = 0; j < nv; ++j) {
                    DMat<Scalar> dS_dyj = DMat<Scalar>::Zero(mss_dim, nv);

                    for (int k = 0; k < n_span_pos; ++k) {
                        // Term 1: dX_intra/dq_k * S_spanning * G * G_kj
                        DMat<Scalar> term1 = dXintra_Sspan_dq[k] * G * G(k, j);

                        // Term 2: X_intra * S_spanning * dG/dq_k * G_kj = S_implicit * dG/dq_k * G_kj
                        DMat<Scalar> term2 = S_implicit * dG_dq_k[k] * G(k, j);

                        dS_dyj += term1 + term2;
                    }

                    S_q_cache_[j] = dS_dyj;
                }

                S_q_cache_valid_ = true;
                return S_q_cache_;
            } else if constexpr (std::is_same_v<Scalar, std::complex<double>>) {
                // Complex-type implementation for complex-step differentiation
                // Uses TAYLOR SERIES EXPANSION to avoid finite-difference errors:
                //   f(q + i*δq) ≈ f(q) + i*(df/dq @ δq)
                //
                // For dG/dq, we use:
                //   dG/dq(q + i*δq) ≈ dG/dq(q) + i*(d²G/dq² @ δq)

                if (!generic_constraint_) {
                    return std::vector<DMat<Scalar>>(nv, DMat<Scalar>::Zero(mss_dim, nv));
                }

                // Safety check: ensure state has been cached
                if (q_cache_.size() == 0 || S_implicit_.size() == 0) {
                    return std::vector<DMat<Scalar>>(nv, DMat<Scalar>::Zero(mss_dim, nv));
                }

                std::vector<DMat<Scalar>> S_q_result(nv, DMat<Scalar>::Zero(mss_dim, nv));

                const DMat<Scalar>& S_implicit = S_implicit_;
                const DMat<Scalar>& G = this->loop_constraint_->G();
                const int n_span = G.rows();
                const int n_indep = G.cols();
                const int n_span_pos = q_cache_.size();

                // Precompute per-body velocity offsets once to avoid repeated O(n)
                // scans inside the dX/dq assembly loops.
                std::vector<int> body_vel_offset(this->num_bodies_, 0);
                std::vector<int> body_num_vel(this->num_bodies_, 0);
                int vel_cursor_pre = 0;
                for (int b = 0; b < this->num_bodies_; ++b) {
                    body_vel_offset[b] = vel_cursor_pre;
                    body_num_vel[b] = this->single_joints_[b]->numVelocities();
                    vel_cursor_pre += body_num_vel[b];
                }

                // Extract real and imaginary parts of q_cache_
                DVec<double> q_real(n_span_pos), q_imag(n_span_pos);
                for (int i = 0; i < n_span_pos; ++i) {
                    q_real(i) = q_cache_(i).real();
                    q_imag(i) = q_cache_(i).imag();
                }

                // Get CasADi derivative functions from constraint
                const casadi::Function& dG_dq_fcn = generic_constraint_->getdGdqFcn();
                const casadi::Function& d2G_dq2_fcn = generic_constraint_->getd2Gdq2Fcn();

                // Evaluate dG/dq at real part using CasADi
                casadi::DM q_dm(n_span_pos);
                casadi::copy(q_real, q_dm);

                casadi::DMVector dG_dq_result = dG_dq_fcn(casadi::DMVector{q_dm});
                casadi::DM dG_dq_stacked_dm = dG_dq_result[0];

                // Evaluate d²G/dq² at real part using CasADi
                casadi::DMVector d2G_dq2_result = d2G_dq2_fcn(casadi::DMVector{q_dm});
                casadi::DM d2G_dq2_stacked_dm = d2G_dq2_result[0];

                // Build complex dG/dq_k matrices using Taylor series:
                // dG/dq(q + i*δq) ≈ dG/dq(q) + i*(d²G/dq² @ δq)
                //
                // d²G/dq² has shape (n_G_elements * n_span_pos, n_span_pos)
                // It's the Jacobian of vec(dG/dq) w.r.t. q
                // dG_dq has shape (n_G_elements, n_span_pos)
                // So d²G/dq² @ δq gives the change in dG/dq for imaginary perturbation δq

                const int n_G_elements = n_span * n_indep;

                // Extract dG/dq_k for each k (real part)
                std::vector<DMat<Scalar>> dG_dq_k(n_span_pos);
                for (int k = 0; k < n_span_pos; ++k) {
                    dG_dq_k[k].resize(n_span, n_indep);
                    for (int row = 0; row < n_span; ++row) {
                        for (int col = 0; col < n_indep; ++col) {
                            // CasADi jacobian(vec(G), q) has shape (n_G_elements, n_q)
                            // vec(G) is column-major: element G[row,col] at index col*n_span + row
                            int vec_idx = col * n_span + row;
                            double dG_real = static_cast<double>(dG_dq_stacked_dm(vec_idx, k));

                            // Compute imaginary part from d²G/dq²
                            // d²G/dq² has shape (n_G_elements * n_q, n_q)
                            // d(dG[row,col]/dq_k)/dq_j is at d2G_dq2(vec_idx * n_q + k, j)
                            // We need sum_j d(dG[row,col]/dq_k)/dq_j * q_imag(j)
                            double dG_imag = 0.0;
                            for (int j = 0; j < n_span_pos; ++j) {
                                // Index into d²G/dq²: row index is (element of dG/dq), col is q_j
                                // dG/dq has shape (n_G_elements, n_q), so dG/dq[vec_idx, k]
                                // is at linear index vec_idx * n_span_pos + k in vec(dG/dq)
                                // d(vec(dG/dq))/dq has shape (n_G_elements * n_q, n_q)
                                int d2_row_idx = vec_idx * n_span_pos + k;
                                dG_imag += static_cast<double>(d2G_dq2_stacked_dm(d2_row_idx, j)) * q_imag(j);
                            }

                            dG_dq_k[k](row, col) = Scalar(dG_real, dG_imag);
                        }
                    }
                }

                // Compute dX_intra/dq_k * S_spanning for each spanning coordinate k
                // This uses the existing complex-valued X_intra_ and S_spanning_
                std::vector<DMat<Scalar>> dXintra_Sspan_dq(n_span_pos, DMat<Scalar>::Zero(mss_dim, n_span_vel));

                // Iterate over spanning coordinates (each corresponds to a body's joint)
                int pos_idx = 0;
                for (int m = 0; m < this->num_bodies_; ++m) {
                    const auto& joint_m = this->single_joints_[m];
                    const int num_pos_m = joint_m->numPositions();

                    // Get the joint axis/motion subspace for body m
                    const DMat<Scalar>& S_m = joint_m->S();  // 6 x num_vel_m

                    // For each position DOF of this joint (usually 1 for revolute)
                    for (int local_k = 0; local_k < num_pos_m; ++local_k) {
                        int k = pos_idx + local_k;  // Global spanning coordinate index

                        // For revolute joints, the axis is the motion subspace
                        SVec<Scalar> axis_m = S_m.col(std::min(local_k, (int)S_m.cols() - 1));

                        for (int i = 0; i < this->num_bodies_; ++i) {
                            if (i != m && !connectivity_(i, m)) continue;

                            for (int j = 0; j < this->num_bodies_; ++j) {
                                if (i == j) continue;

                                bool m_in_path = false;

                                if (i == m) {
                                    if (j != m && (j < m || connectivity_(m, j))) {
                                        m_in_path = true;
                                    }
                                } else if (connectivity_(i, m)) {
                                    if (j == m) {
                                        m_in_path = false;
                                    } else if (j < m && connectivity_(m, j)) {
                                        m_in_path = true;
                                    } else if (j < m) {
                                        Mat6<Scalar> X_mj = X_intra_.template block<6,6>(6*m, 6*j);
                                        if (std::abs(X_mj.norm()) > 1e-10) {
                                            m_in_path = true;
                                        }
                                    }
                                }

                                if (m_in_path) {
                                    Mat6<Scalar> X_im;
                                    if (i == m) {
                                        X_im = Mat6<Scalar>::Identity();
                                    } else {
                                        X_im = X_intra_.template block<6,6>(6*i, 6*m);
                                    }
                                    Mat6<Scalar> X_mj = X_intra_.template block<6,6>(6*m, 6*j);

                                    SVec<Scalar> X_im_s = X_im * axis_m;
                                    Mat6<Scalar> X_ij = X_im * X_mj;
                                    Mat6<Scalar> dX_ij_dqm = -spatial::motionCrossMatrix(X_im_s) * X_ij;

                                    const int vel_idx_j = body_vel_offset[j];
                                    const int num_vel_j = body_num_vel[j];

                                    DMat<Scalar> S_span_j = S_spanning_.block(6*j, vel_idx_j, 6, num_vel_j);
                                    DMat<Scalar> contrib = dX_ij_dqm * S_span_j;

                                    dXintra_Sspan_dq[k].block(6*i, vel_idx_j, 6, num_vel_j) += contrib;
                                }
                            }
                        }
                    }
                    pos_idx += num_pos_m;
                }

                // Now compute dS/dy_j = sum_k (dS/dq_k * G_kj)
                for (int j = 0; j < nv; ++j) {
                    DMat<Scalar> dS_dyj = DMat<Scalar>::Zero(mss_dim, nv);

                    for (int k = 0; k < n_span_pos; ++k) {
                        // Term 1: dX_intra/dq_k * S_spanning * G * G_kj
                        DMat<Scalar> term1 = dXintra_Sspan_dq[k] * G * G(k, j);

                        // Term 2: S_implicit * dG/dq_k * G_kj
                        DMat<Scalar> term2 = S_implicit * dG_dq_k[k] * G(k, j);

                        dS_dyj += term1 + term2;
                    }

                    S_q_result[j] = dS_dyj;
                }

                return S_q_result;
            } else {
                return std::vector<DMat<Scalar>>(nv, DMat<Scalar>::Zero(mss_dim, nv));
            }
        }

        template <typename Scalar>

        DMat<Scalar> Generic<Scalar>::getSdotqd_q() const
        {
            const int mss_dim = this->num_bodies_ * 6;
            const int nv = this->num_velocities_;
            const char *profile_fd_env = std::getenv("GRBDA_PROFILE_SDOTQD_FD");
            const bool enable_profiling = (profile_fd_env != nullptr && profile_fd_env[0] != '0');

            // Fast production path: S_ring stores dS/dq contracted with generalized velocity.
            // Keep this as default for throughput-sensitive workloads.
            if constexpr (std::is_same_v<Scalar, double> || std::is_same_v<Scalar, float>) {
                const char *disable_fast = std::getenv("GRBDA_DISABLE_FAST_SDOTQD_Q");
                const bool fast_path_enabled = (disable_fast == nullptr || disable_fast[0] == '0');
                if (fast_path_enabled) {
                    // For implicit joints, S_ring_ can introduce a measurable bias in d(tau)/dq.
                    // Default to the accurate FD path unless explicitly overridden.
                    if (!generic_constraint_) {
                        return this->S_ring_;
                    }

                    const char *enable_fast_implicit = std::getenv("GRBDA_ENABLE_FAST_SDOTQD_Q_IMPLICIT");
                    if (enable_fast_implicit != nullptr && enable_fast_implicit[0] != '0') {
                        return this->S_ring_;
                    }
                }
            }

            // Explicit constraints (or missing implicit constraint handle) have no extra
            // configuration-dependent bias term beyond the standard explicit-joint path.
            if (!generic_constraint_) {
                if (enable_profiling) {
                    std::cerr << "[SDotqdQFD] No generic_constraint. Is it explicit? " 
                              << (this->loop_constraint_ ? (this->loop_constraint_->isExplicit() ? "yes" : "no") : "null")
                              << " nv=" << nv << std::endl;
                }
                return DMat<Scalar>::Zero(mss_dim, nv);
            }

            // Need a valid cached state from updateKinematics.
            if (q_cache_.size() == 0 || S_implicit_.size() == 0) {
                if (enable_profiling) {
                    std::cerr << "[SDotqdQFD] Invalid cache state (q_size=" << q_cache_.size() 
                              << ", S_size=" << S_implicit_.size() << ")" << std::endl;
                }
                return DMat<Scalar>::Zero(mss_dim, nv);
            }

            // Use cache if available (state hasn't changed since last computation)
            if (Sdotqd_q_cache_valid_ && Sdotqd_q_cache_.rows() == mss_dim && Sdotqd_q_cache_.cols() == nv) {
                if (enable_profiling) {
                    std::cerr << "[SDotqdQFD] Using cached result" << std::endl;
                }
                return Sdotqd_q_cache_;
            }

            if (enable_profiling) {
                std::cerr << "[SDotqdQFD] Computing FD for nv=" << nv << " mss_dim=" << mss_dim << std::endl;
            }

            // For implicit joints, compute d(cJ)/dy directly via finite differences,
            // where cJ = X_intra_ring * S_spanning * qd_span + S_implicit * g(q_span, qd_span).
            // This captures all chain-rule paths through X_intra, X_intra_ring, and g.
            if constexpr (std::is_same_v<Scalar, double> || std::is_same_v<Scalar, std::complex<double>>) {
                const auto t_fd_start = std::chrono::high_resolution_clock::now();
                const DMat<Scalar>& G_base = this->loop_constraint_->G();
                const int n_span_vel = this->loop_constraint_->numSpanningVel();
                const DVec<Scalar> ydot_independent =
                    G_base.colPivHouseholderQr().solve(qd_cache_);

                double time_clone_us = 0.0;
                double time_constraint_us = 0.0;
                double time_kin_us = 0.0;
                double time_xring_us = 0.0;
                double time_bias_us = 0.0;

                // Hoist reusable state outside perturbation loop to avoid repeated
                // heap churn in cold FD calls.
                const auto t_clone_start = std::chrono::high_resolution_clock::now();
                auto lc_local = generic_constraint_->clone();
                std::vector<JointPtr<Scalar>> joints_local;
                joints_local.reserve(this->num_bodies_);
                std::vector<int> pos_offsets(this->num_bodies_);
                std::vector<int> vel_offsets(this->num_bodies_);
                std::vector<int> num_pos_per_joint(this->num_bodies_);
                std::vector<int> num_vel_per_joint(this->num_bodies_);

                int pos_cursor = 0;
                int vel_cursor = 0;
                for (int i = 0; i < this->num_bodies_; ++i) {
                    auto joint_clone = this->single_joints_[i]->clone();
                    const int num_pos_i = joint_clone->numPositions();
                    const int num_vel_i = joint_clone->numVelocities();
                    joints_local.push_back(joint_clone);
                    pos_offsets[i] = pos_cursor;
                    vel_offsets[i] = vel_cursor;
                    num_pos_per_joint[i] = num_pos_i;
                    num_vel_per_joint[i] = num_vel_i;
                    pos_cursor += num_pos_i;
                    vel_cursor += num_vel_i;
                }
                const auto t_clone_end = std::chrono::high_resolution_clock::now();
                time_clone_us += std::chrono::duration<double, std::micro>(t_clone_end - t_clone_start).count();

                DMat<Scalar> S_spanning_local = DMat<Scalar>::Zero(mss_dim, n_span_vel);
                DMat<Scalar> X_intra_local = DMat<Scalar>::Identity(mss_dim, mss_dim);
                DMat<Scalar> X_intra_ring_local = DMat<Scalar>::Zero(mss_dim, mss_dim);
                DMat<Scalar> S_implicit_local(mss_dim, n_span_vel);
                DVec<Scalar> qd_span_local(n_span_vel);
                DVec<Scalar> vJ_local(mss_dim);

                auto evaluate_cJ_term = [&](const DVec<Scalar>& q_span) -> DVec<Scalar> {
                    const auto t_constraint_start = std::chrono::high_resolution_clock::now();
                    JointCoordinate<Scalar> pos_coord(q_span, true);
                    lc_local->updateJacobians(pos_coord);
                    const DMat<Scalar> G_local = lc_local->G();
                    qd_span_local.noalias() = G_local * ydot_independent;
                    const auto t_kin_start = std::chrono::high_resolution_clock::now();
                    time_constraint_us += std::chrono::duration<double, std::micro>(t_kin_start - t_constraint_start).count();

                    S_spanning_local.setZero();
                    X_intra_local.setIdentity();
                    for (int i = 0; i < this->num_bodies_; ++i) {
                        auto joint_i = joints_local[i];
                        const int num_pos_i = num_pos_per_joint[i];
                        const int num_vel_i = num_vel_per_joint[i];
                        const int pos_idx = pos_offsets[i];
                        const int vel_idx = vel_offsets[i];

                        joint_i->updateKinematics(q_span.segment(pos_idx, num_pos_i),
                                                  qd_span_local.segment(vel_idx, num_vel_i));

                        S_spanning_local.block(6 * i, vel_idx, 6, num_vel_i) = joint_i->S();

                        int k = i;
                        for (int j = i - 1; j >= 0; --j) {
                            if (connectivity_(i, j)) {
                                const auto& body_k = bodies_[k];
                                const auto joint_k = joints_local[k];

                                const Mat6<Scalar> Xup_prev = X_intra_local.template block<6, 6>(6 * i, 6 * k);
                                const Mat6<Scalar> Xint = (joint_k->XJ() * body_k.Xtree_).toMatrix();
                                X_intra_local.template block<6, 6>(6 * i, 6 * j) = Xup_prev * Xint;
                                k = j;
                            }
                        }
                    }
                    const auto t_xring_start = std::chrono::high_resolution_clock::now();
                    time_kin_us += std::chrono::duration<double, std::micro>(t_xring_start - t_kin_start).count();

                    S_implicit_local.noalias() = X_intra_local * S_spanning_local;
                    vJ_local.noalias() = S_implicit_local * qd_span_local;

                    X_intra_ring_local.setZero();
                    for (int i = 0; i < this->num_bodies_; ++i) {
                        SVec<Scalar> v_relative = SVec<Scalar>::Zero();
                        for (int j = i - 1; j >= 0; --j) {
                            if (connectivity_(i, j)) {
                                const Mat6<Scalar> Xup = X_intra_local.template block<6, 6>(6 * i, 6 * j);
                                const SVec<Scalar> v_parent = Xup * vJ_local.template segment<6>(6 * j);
                                const SVec<Scalar> v_child = vJ_local.template segment<6>(6 * i);
                                v_relative = v_child - v_parent;
                                X_intra_ring_local.template block<6, 6>(6 * i, 6 * j) =
                                    -spatial::motionCrossMatrix(v_relative) * Xup;
                            }
                        }
                    }

                    JointCoordinate<Scalar> vel_coord(qd_span_local, true);
                    JointState<Scalar> js(pos_coord, vel_coord);
                    const auto t_bias_start = std::chrono::high_resolution_clock::now();
                    time_xring_us += std::chrono::duration<double, std::micro>(t_bias_start - t_xring_start).count();

                    lc_local->updateBiases(js);
                    const auto t_bias_end = std::chrono::high_resolution_clock::now();
                    time_bias_us += std::chrono::duration<double, std::micro>(t_bias_end - t_bias_start).count();

                    DVec<Scalar> cJ_term = X_intra_ring_local * S_spanning_local * qd_span_local;
                    cJ_term.noalias() += S_implicit_local * lc_local->g();
                    return cJ_term;
                };

                DMat<Scalar> out = DMat<Scalar>::Zero(mss_dim, nv);
                const DMat<Scalar>& G = this->loop_constraint_->G();
                const Scalar h = 1e-6;

                const auto t_loop_start = std::chrono::high_resolution_clock::now();
                for (int j = 0; j < nv; ++j) {
                    const DVec<Scalar> direction = G.col(j);

                    const DVec<Scalar> c_plus = evaluate_cJ_term(q_cache_ + h * direction);
                    const DVec<Scalar> c_minus = evaluate_cJ_term(q_cache_ - h * direction);
                    out.col(j) = (c_plus - c_minus) / (2.0 * h);
                }
                const auto t_loop_end = std::chrono::high_resolution_clock::now();

                // Cache the result
                Sdotqd_q_cache_ = out;
                Sdotqd_q_cache_valid_ = true;

                // Print profiling breakdown if enabled (using stderr for unbuffered output)
                if (enable_profiling) {
                    const double total_loop_us = std::chrono::duration<double, std::micro>(t_loop_end - t_loop_start).count();
                    const double per_pert_us = total_loop_us / (2 * nv);
                    std::cerr << "[SDotqdQFDProfile] clone_us=" << std::fixed << std::setprecision(2) 
                              << (time_clone_us / (2 * nv))
                              << " constraint_us=" << (time_constraint_us / (2 * nv))
                              << " kin_us=" << (time_kin_us / (2 * nv))
                              << " xring_us=" << (time_xring_us / (2 * nv))
                              << " bias_us=" << (time_bias_us / (2 * nv))
                              << " per_pert_us=" << per_pert_us
                              << std::defaultfloat << std::endl;
                }

                return Sdotqd_q_cache_;
            }

            return DMat<Scalar>::Zero(mss_dim, nv);
        }

        template <typename Scalar>
        DMat<Scalar> Generic<Scalar>::getSdotqd_qd() const
        {
            // The derivative of S(q) * qd w.r.t. qd is S(q)
            return this->S_;
        }
        template class Generic<double>;
        template class Generic<std::complex<double>>;
        template class Generic<float>;
        template class Generic<casadi::SX>;
    }

} // namespace grbda

