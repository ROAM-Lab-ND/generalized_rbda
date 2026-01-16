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
        }

        template <typename Scalar>
        DVec<Scalar> GenericImplicit<Scalar>::gamma(const JointCoordinate<Scalar> &joint_pos) const
        {
            throw std::runtime_error("GenericImplicit::gamma() not implemented");
        }

        template <typename Scalar>
        void GenericImplicit<Scalar>::updateJacobians(const JointCoordinate<Scalar> &joint_pos)
        {
            this->K_ = runCasadiFcn(K_fcn_, joint_pos);
            this->G_ = runCasadiFcn(G_fcn_, joint_pos);
        }

        template <typename Scalar>
        void GenericImplicit<Scalar>::updateBiases(const JointState<Scalar> &joint_state)
        {
            this->k_ = runCasadiFcn(k_fcn_, joint_state);
            this->g_ = runCasadiFcn(g_fcn_, joint_state);
        }

        template <typename Scalar>
        const std::vector<bool> &GenericImplicit<Scalar>::isCoordinateIndependent() const
        {
            return is_coordinate_independent_;
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

            auto numerical_lc = generic_constraint_->copyAsDouble();
            auto phi_eval = [&](const DVec<double> &q) {
                return numerical_lc.phi(JointCoordinate<double>(q, true));
            };

            // Dampened Newton
            const int max_iters = 100;
            const double tol_accept = 2e-2;
            const double h = 1e-7;
            const double damping = 0.5;
            
            // Build dep indices
            std::vector<int> dep_idx; dep_idx.reserve(n_dep);
            for (int i = 0; i < n_span; ++i) if (!ind_mask[i]) dep_idx.push_back(i);

            bool converged = false;
            for (int attempt = 0; attempt < 20 && !converged; ++attempt) {
                for (int iter = 0; iter < max_iters; ++iter) {
                    DVec<double> phi = phi_eval(q_span);
                    if (phi.norm() < tol_accept) { converged = true; break; }
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
                    // reinitialize dependents slightly differently
                    for (int i = 0; i < n_span; ++i) if (!ind_mask[i]) q_span(i) = 0.02 * (2.0 * ((double)rand() / RAND_MAX) - 1.0);
                }
            }

            if (!converged || !numerical_lc.isValidSpanningPosition(JointCoordinate<double>(q_span, true))) {
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
            this->S_ring_ = X_intra_ring_ * this->S_spanning_ * this->loop_constraint_->G(); //+X_intra*S_panning_*G_dot_;
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
            const int mss_dim = this->num_bodies_ * 6;
            const int nv = this->num_velocities_;
            const int n_span_vel = this->loop_constraint_->numSpanningVel();

            if constexpr (std::is_same_v<Scalar, double>) {
                initializeDerivativeFunctions();

                // Reuse cached result when possible to avoid repeated CasADi evaluation
                if (S_q_cache_valid_ && (int)S_q_cache_.size() == nv) {
                    return S_q_cache_;
                }

                S_q_cache_.assign(nv, DMat<Scalar>::Zero(mss_dim, nv));

                if (!generic_constraint_) {
                    S_q_cache_valid_ = true;
                    return S_q_cache_;
                }

                // Safety check: ensure state has been cached
                if (q_cache_.size() == 0 || !derivative_functions_initialized_ || S_implicit_.size() == 0) {
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

                casadi::DMVector result = dG_dq_fcn_(casadi::DMVector{q_dm});
                casadi::DM dG_dq_stacked_dm = result[0];

                const int n_span = G.rows();
                const int n_indep = G.cols();

                // Debug: Check if CasADi returned NaN/Inf
                bool has_nan = false;
                for (int i = 0; i < dG_dq_stacked_dm.size1(); ++i) {
                    double val = static_cast<double>(dG_dq_stacked_dm(i));
                    if (!std::isfinite(val)) {
                        has_nan = true;
                        std::cout << "[DEBUG getSq] CasADi element " << i << " = " << val << std::endl;
                    }
                }
                if (has_nan) {
                    std::cout << "[DEBUG getSq] CasADi dG_dq_fcn returned NaN/Inf!" << std::endl;
                    std::cout << "  q_cache size=" << q_cache_.size() << ": " << q_cache_.transpose() << std::endl;
                    std::cout << "  dG_dq size=" << dG_dq_stacked_dm.size1() << std::endl;
                }

                for (int qi = 0; qi < nv; ++qi) {
                    if (qi < n_span_vel) {
                        DMat<Scalar> dG_dqi(n_span, n_indep);
                        for (int row = 0; row < n_span; ++row) {
                            for (int col = 0; col < n_indep; ++col) {
                                int idx = qi * n_span * n_indep + row * n_indep + col;
                                dG_dqi(row, col) = static_cast<double>(dG_dq_stacked_dm(idx));
                            }
                        }
                        S_q_cache_[qi] = S_implicit * dG_dqi;

                        // Debug: Check if result contains NaN
                        if (!S_q_cache_[qi].allFinite()) {
                            std::cout << "[DEBUG getSq] S_q[" << qi << "] contains NaN/Inf after multiplication!" << std::endl;
                            std::cout << "  dG_dqi finite: " << dG_dqi.allFinite() << std::endl;
                        }
                    }
                }

                S_q_cache_valid_ = true;
                return S_q_cache_;
            } else {
                return std::vector<DMat<Scalar>>(nv, DMat<Scalar>::Zero(mss_dim, nv));
            }
        }

        template <typename Scalar>

        DMat<Scalar> Generic<Scalar>::getSdotqd_q() const
        {
            // For implicit joints, S_ring_ encodes dS/dq * G
            // We need to return the full matrix, not S_ring_ * qd
            // The correct return is S_ring_ itself, which is (spatial_dim x nv)
            return this->S_ring_;
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

