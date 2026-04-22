        // Evaluate d(S*b)/d(independent coords) using CasADi function object
        template <typename Scalar>
        DMat<Scalar> Generic<Scalar>::evalSvec_deriv(const DVec<Scalar>& b) const {
            if constexpr (std::is_same_v<Scalar, double>) {
                initializeDerivativeFunctions();
                if (!derivative_functions_initialized_ || jacobian_S_times_b_fcn_.is_null() || q_cache_.size() == 0) {
                    return DMat<Scalar>::Zero(this->num_bodies_ * 6, this->num_velocities_);
                }
                casadi::DM q_dm(q_cache_.size());
                casadi::DM b_dm(b.size());
                casadi::copy(q_cache_, q_dm);
                casadi::copy(b, b_dm);
                casadi::DM result_dm = jacobian_S_times_b_fcn_(casadi::DMVector{q_dm, b_dm})[0];
                DMat<Scalar> out(result_dm.size1(), result_dm.size2());
                casadi::copy(result_dm, out);
                return out;
            } else {
                return DMat<Scalar>::Zero(this->num_bodies_ * 6, this->num_velocities_);
            }
        }

        // Evaluate d(S^T*F)/d(independent coords) using CasADi function object
        template <typename Scalar>
        DMat<Scalar> Generic<Scalar>::evalSTvec_deriv(const DVec<Scalar>& F) const {
            if constexpr (std::is_same_v<Scalar, double>) {
                initializeDerivativeFunctions();
                if (!derivative_functions_initialized_ || jacobian_ST_times_F_fcn_.is_null() || q_cache_.size() == 0) {
                    return DMat<Scalar>::Zero(this->num_velocities_, this->num_velocities_);
                }
                casadi::DM q_dm(q_cache_.size());
                casadi::DM F_dm(F.size());
                casadi::copy(q_cache_, q_dm);
                casadi::copy(F, F_dm);
                casadi::DM result_dm = jacobian_ST_times_F_fcn_(casadi::DMVector{q_dm, F_dm})[0];
                DMat<Scalar> out(result_dm.size1(), result_dm.size2());
                casadi::copy(result_dm, out);
                return out;
            } else {
                return DMat<Scalar>::Zero(this->num_velocities_, this->num_velocities_);
            }
        }
#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"
#include "grbda/Utils/Utilities.h"

namespace grbda
{

    namespace LoopConstraint
    {
        template <typename Scalar>
        GenericImplicit<Scalar>::GenericImplicit(std::vector<bool> is_coordinate_independent,
                                                 SymPhiFcn phi_fcn)
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
            coord_map_ = DMat<double>::Zero(state_dim, state_dim);
            for (int i = 0; i < ind_dim; i++)
            {
                coord_map(ind_coords[i], i) = 1;
                coord_map_(ind_coords[i], i) = 1.0;
            }
            for (int i = 0; i < dep_dim; i++)
            {
                coord_map(dep_coords[i], i + ind_dim) = 1;
                coord_map_(dep_coords[i], i + ind_dim) = 1.0;
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
            
            
            cs_G_sym(dep_slice, casadi::Slice()) = -SX::solve(cs_Kd_sym, cs_Ki_sym);
            cs_G_sym = SX::mtimes(coord_map, cs_G_sym);

            // Explicit constraints bias
            SX cs_g_sym = SX::zeros(state_dim, 1);
            if (cs_Kd_sym.size2() == 0) {
                std::cerr << "[GenericImplicit] cs_Kd_sym has zero size for bias!" << std::endl;
            }
            
            
            cs_g_sym(dep_slice) = SX::solve(cs_Kd_sym, cs_k_sym);
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
                                                 SymPhiFcn phi_sym, NativePhiFcn phi_native)
            : GenericImplicit(is_coordinate_independent, phi_sym)
        {
            phi_native_ = phi_native;
            has_native_phi_ = true;

            // Override phi_ to use native phi for complex types (CasADi doesn't support complex)
            // Also use native phi for double types when available for better numerical accuracy
            // The native C++ implementation using std::sin/std::cos is more precise than
            // CasADi's symbolic evaluation, which may have truncation in constant terms
            if constexpr (std::is_same_v<Scalar, std::complex<double>> || std::is_same_v<Scalar, double>) {
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
                
                casadi::DM res_dm = fcn(arg_dm)[0];
                
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
                
                casadi::DM res_dm = fcn(arg_dm)[0];
                
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
                std::vector<casadi::DM> res_vec = fcn(arg_vec);
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
                std::vector<casadi::DM> res_vec = fcn(arg_vec);
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
            casadi::DM res_dm = fcn(arg_dm)[0];
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
            std::vector<casadi::DM> res_vec = fcn(arg_vec);
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
                // if (max_G_imag > 1e-25) {
                //     std::cout << "[DEBUG evalG Taylor] G has imag, max|G_imag|=" << max_G_imag << std::endl;
                //     std::cout << "  q_imag norm=" << q_imag.norm() << std::endl;
                // }

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

            if (!generic_constraint_)
            {
                throw std::runtime_error("GenericImplicit loop constraint not set");
            }

            const int n_span = this->loop_constraint_->numSpanningPos();
            const int n_ind = this->loop_constraint_->numIndependentPos();
            const int n_dep = n_span - n_ind;

            // Build independent mask
            std::vector<bool> ind_mask = generic_constraint_->isCoordinateIndependent();
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
                use_native = generic_constraint_->hasNativePhi();
                if (use_native) {
                    phi_native_double = [this](const DVec<double>& q) -> DVec<double> {
                        JointCoordinate<double> jc(q, true);
                        return generic_constraint_->nativePhi()(jc);
                    };
                }
            }
            auto numerical_lc = generic_constraint_->copyAsDouble();
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

            using SX = casadi::SX;

            const int mss_dim = this->num_bodies_ * 6;
            const int n_span_pos = this->loop_constraint_->numSpanningPos();
            const int n_span_vel = this->loop_constraint_->numSpanningVel();

            const int nv = this->num_velocities_;

            // Symbolic spanning positions and independent velocities.
            // qd_span is NOT an independent symbolic variable — it is determined by
            // the velocity constraint: qd_span = G(q_span) * ydot.
            // Using ydot as the velocity input ensures the Jacobian d(cJ)/dq_span
            // is taken holding ydot fixed (correct), capturing how qd_span also
            // changes with q_span through G.
            SX q_span_sx = SX::sym("q_span", n_span_pos);
            DVec<SX> q_span_vec(n_span_pos);
            casadi::copy(q_span_sx, q_span_vec);

            SX ydot_sx = SX::sym("ydot", nv);

            // Get G and g symbolically by calling CasADi functions with SX inputs.
            // Calling symbolic_constraint.updateJacobians/updateBiases with SX inputs
            // fails because runCasadiFcn converts symbolic variables to NaN via DM cast.
            // Instead, call the underlying CasADi functions directly with SX arguments,
            // which performs symbolic substitution and returns correct SX expressions.
            SX G_casadi = generic_constraint_->getGFcn()(casadi::SXVector{q_span_sx})[0];

            // Express spanning velocity via the constraint: qd_span = G(q_span) * ydot
            SX qd_span_constrained = SX::mtimes(G_casadi, ydot_sx);  // n_span_vel x 1
            DVec<SX> qd_span_vec(n_span_vel);
            for (int i = 0; i < n_span_vel; ++i) qd_span_vec(i) = qd_span_constrained(i, 0);

            SX g_casadi = generic_constraint_->getgFcn()(casadi::SXVector{q_span_sx, qd_span_constrained})[0];

            // Compute dG/dq using CasADi automatic differentiation
            std::vector<SX> dG_dq_vec;
            for (int i = 0; i < n_span_pos; ++i) {
                SX dG_dqi = jacobian(G_casadi, q_span_sx(i));
                dG_dq_vec.push_back(dG_dqi);
            }
            SX dG_dq_stacked = SX::vertcat(dG_dq_vec);
            dG_dq_fcn_ = casadi::Function("dG_dq", {q_span_sx}, {dG_dq_stacked});

            // Build symbolic cJ = X_intra_ring * S_spanning * qd_span + X_intra * S_spanning * g
            // by constructing SX-typed joint clones and running the same logic as updateKinematics_vJ.

            // Build symbolic cJ using cloneAsSymbolic() on each sub-joint.
            // Guarded with if constexpr: bodies_[].Xtree_ and S_spanning_ are DMat<Scalar>,
            // so .cast<SX>() inside would fail to instantiate for Scalar=complex<double>.
            if constexpr (std::is_same_v<Scalar, double>) {
                std::vector<std::shared_ptr<Joints::Base<SX>>> joints_sx;
                for (int i = 0; i < this->num_bodies_; ++i)
                    joints_sx.push_back(this->single_joints_[i]->cloneAsSymbolic());

                // Drive symbolic joints with q_span_vec / qd_span_vec (already Eigen<SX>)
                int pos_idx2 = 0, vel_idx2 = 0;
                for (int i = 0; i < this->num_bodies_; ++i) {
                    const int npos = joints_sx[i]->numPositions();
                    const int nvel = joints_sx[i]->numVelocities();
                    joints_sx[i]->updateKinematics(
                        q_span_vec.segment(pos_idx2, npos),
                        qd_span_vec.segment(vel_idx2, nvel));
                    pos_idx2 += npos;
                    vel_idx2 += nvel;
                }

                // Build X_intra_sx (same connectivity loop as updateKinematics_vJ)
                DMat<SX> X_intra_sx = DMat<SX>::Identity(mss_dim, mss_dim);
                for (int i = 0; i < this->num_bodies_; ++i) {
                    int k = i;
                    for (int j = i - 1; j >= 0; --j) {
                        if (connectivity_(i, j)) {
                            DMat<SX> Xup_prev = X_intra_sx.block(6 * i, 6 * k, 6, 6);
                            Mat6<SX> XJ_k = joints_sx[k]->XJ().toMatrix();
                            Mat6<SX> Xtree_k = bodies_[k].Xtree_.toMatrix().template cast<SX>();
                            X_intra_sx.block(6 * i, 6 * j, 6, 6) = Xup_prev * XJ_k * Xtree_k;
                            k = j;
                        }
                    }
                }

                DMat<SX> S_spanning_sx = S_spanning_.template cast<SX>();
                DMat<SX> S_implicit_sx = X_intra_sx * S_spanning_sx;
                DVec<SX> vJ_sx = S_implicit_sx * qd_span_vec;

                // --- Jacobian-based product derivatives ---

                // Build X_intra_ring_sx
                DMat<SX> X_intra_ring_sx = DMat<SX>::Zero(mss_dim, mss_dim);
                for (int i = 0; i < this->num_bodies_; ++i) {
                    for (int j = i - 1; j >= 0; --j) {
                        if (connectivity_(i, j)) {
                            DMat<SX> Xup = X_intra_sx.block(6 * i, 6 * j, 6, 6);
                            SVec<SX> v_parent = Xup * vJ_sx.template segment<6>(6 * j);
                            SVec<SX> v_child = vJ_sx.template segment<6>(6 * i);
                            X_intra_ring_sx.block(6 * i, 6 * j, 6, 6) =
                                -spatial::motionCrossMatrix(v_child - v_parent) * Xup;
                        }
                    }
                }

                DVec<SX> g_sx_vec(g_casadi.size1());
                for (int r = 0; r < (int)g_casadi.size1(); ++r) g_sx_vec(r) = g_casadi(r, 0);

                DMat<SX> S_full_sx = S_implicit_sx * G_casadi; // S = X_intra * S_spanning * G

                // --- jacobian_S_times_b: d(S*b)/dq_span, then contract with G to get d(S*b)/d(independent coords) ---
                SX b_sx = SX::sym("b", nv, 1);
                SX S_times_b = SX::mtimes(S_full_sx, b_sx); // mss_dim x 1
                SX dS_times_b_dq = jacobian(S_times_b, q_span_sx); // mss_dim x n_span_pos
                SX dS_times_b_dy = SX::mtimes(dS_times_b_dq, G_casadi); // mss_dim x nv
                jacobian_S_times_b_fcn_ = casadi::Function("jacobian_S_times_b", {q_span_sx, b_sx}, {dS_times_b_dy});

                // --- jacobian_ST_times_F: d(S^T*F)/dq_span, then contract with G to get d(S^T*F)/d(independent coords) ---
                SX F_sx = SX::sym("F", mss_dim, 1);
                SX S_T = SX::transpose(S_full_sx); // nv x mss_dim
                SX S_T_times_F = SX::mtimes(S_T, F_sx); // nv x 1
                SX dST_times_F_dq = jacobian(S_T_times_F, q_span_sx); // nv x n_span_pos
                SX dST_times_F_dy = SX::mtimes(dST_times_F_dq, G_casadi); // nv x nv
                jacobian_ST_times_F_fcn_ = casadi::Function("jacobian_ST_times_F", {q_span_sx, F_sx}, {dST_times_F_dy});

                // --- cJ and Sdotqd derivatives (unchanged) ---
                DVec<SX> cJ_sx = X_intra_ring_sx * S_spanning_sx * qd_span_vec
                                + S_implicit_sx * g_sx_vec;

                SX cJ_casadi = SX::zeros(mss_dim, 1);
                casadi::copy(cJ_sx, cJ_casadi);
                SX dcJ_dq_sx = jacobian(cJ_casadi, q_span_sx);        // mss_dim x n_span_pos
                SX dcJ_dy_sx = SX::mtimes(dcJ_dq_sx, G_casadi);       // mss_dim x nv

                dSdotqd_dq_fcn_ = casadi::Function("dSdotqd_dq",
                    {q_span_sx, ydot_sx}, {dcJ_dy_sx});
            }
            // end if constexpr (std::is_same_v<Scalar, double>)
        }


        template <typename Scalar>
        std::vector<DMat<Scalar>> Generic<Scalar>::getSq() const
        {
            const int mss_dim = this->num_bodies_ * 6;
            const int nv = this->num_velocities_;
            const int n_span_vel = this->loop_constraint_->numSpanningVel();

            if constexpr (std::is_same_v<Scalar, double>) {
                initializeDerivativeFunctions();
                if (S_q_cache_valid_ && (int)S_q_cache_.size() == nv) {
                    return S_q_cache_;
                }
                S_q_cache_.assign(nv, DMat<Scalar>::Zero(mss_dim, nv));
                if (!generic_constraint_) {
                    S_q_cache_valid_ = true;
                    return S_q_cache_;
                }
                if (q_cache_.size() == 0 || !derivative_functions_initialized_ || S_implicit_.size() == 0) {
                    S_q_cache_valid_ = true;
                    return S_q_cache_;
                }
                // Use new contraction function for each basis vector
                for (int j = 0; j < nv; ++j) {
                    DVec<Scalar> b = DVec<Scalar>::Zero(nv);
                    b(j) = Scalar(1);
                    S_q_cache_[j] = evalSvec_deriv(b);
                }
                S_q_cache_valid_ = true;
                return S_q_cache_;
            } else if constexpr (std::is_same_v<Scalar, casadi::SX>) {
                // Symbolic: return zeros (or throw if needed)
                return std::vector<DMat<Scalar>>(nv, DMat<Scalar>::Zero(mss_dim, nv));
            } else if constexpr (std::is_same_v<Scalar, std::complex<double>>) {
                throw std::runtime_error("getSq is not implemented for complex types due to CasADi limitations with symbolic derivatives in complex mode.");
            } else {
                return std::vector<DMat<Scalar>>(nv, DMat<Scalar>::Zero(mss_dim, nv));
            }
        }

        template <typename Scalar>
        DMat<Scalar> Generic<Scalar>::getSdotqd_q() const
        {
            const int mss_dim = this->num_bodies_ * 6;
            const int nv = this->num_velocities_;

            if (!generic_constraint_)
            {
                return DMat<Scalar>::Zero(mss_dim, nv);
            }

            // Need a valid cached state from updateKinematics.
            if (q_cache_.size() == 0 || S_implicit_.size() == 0) {
                return DMat<Scalar>::Zero(mss_dim, nv);
            }

            // For implicit joints, compute d(cJ)/dy directly via finite differences,
            // where cJ = X_intra_ring * S_spanning * qd_span + S_implicit * g(q_span, qd_span).
            // This captures all chain-rule paths through X_intra, X_intra_ring, and g.
            if constexpr (std::is_same_v<Scalar, double> ) {
                const DMat<Scalar>& G_base = this->loop_constraint_->G();

                // coord_map^T * qd_span = [ydot; qdot_dep], so ydot is the first nv entries
                const DMat<double>& coord_map = generic_constraint_->getCoordMap();
                const DVec<Scalar> ydot_independent =
                    (coord_map.transpose() * qd_cache_).head(nv);

                DMat<Scalar> out = DMat<Scalar>::Zero(mss_dim, nv);

                initializeDerivativeFunctions();

                if (q_cache_.size() == 0 || qd_cache_.size() == 0 ||
                    !derivative_functions_initialized_ || dSdotqd_dq_fcn_.is_null())
                    return DMat<Scalar>::Zero(mss_dim, nv);


                // --- CasADi symbolic derivative ---
                casadi::DM q_dm(q_cache_.size());
                casadi::DM ydot_dm(ydot_independent.size());
                casadi::copy(q_cache_, q_dm);
                casadi::copy(ydot_independent, ydot_dm);

                casadi::DM result_dm = dSdotqd_dq_fcn_(casadi::DMVector{q_dm, ydot_dm})[0];

                casadi::copy(result_dm, out);
                
                return out;
            }

            return DMat<Scalar>::Zero(mss_dim, nv);
        }

        template <typename Scalar>
        DMat<Scalar> Generic<Scalar>::getSdotqd_qd() const
        {
            std::cout << "[DEBUG getSdotqd_qd] Called with Scalar = " << typeid(Scalar).name() << std::endl;
            throw std::runtime_error("getSdotqd_qd is not implemented yet");
        }
        template class Generic<double>;
        template class Generic<std::complex<double>>;
        template class Generic<float>;
        template class Generic<casadi::SX>;
    }

} // namespace grbda

