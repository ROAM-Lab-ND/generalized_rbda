#include "grbda/Dynamics/ClusterJoints/FourBarJoint.h"
#include <iostream>

namespace grbda
{
    namespace LoopConstraint
    {
        template <typename Scalar>
        FourBar<Scalar>::FourBar(std::vector<Scalar> path1_link_lengths,
                                 std::vector<Scalar> path2_link_lengths,
                                 Vec2<Scalar> offset, int independent_coordinate)
            : links_in_path1_(path1_link_lengths.size()),
              links_in_path2_(path2_link_lengths.size()),
              path1_link_lengths_(path1_link_lengths),
              path2_link_lengths_(path2_link_lengths),
              offset_(offset), independent_coordinate_(independent_coordinate)
        {
            if (links_in_path1_ + links_in_path2_ != 3)
            {
                throw std::runtime_error("FourBar: Must contain 3 links");
            }

            this->phi_ = [this, offset](const JointCoordinate<Scalar> &joint_pos)
            {
                DVec<Scalar> phi = DVec<Scalar>::Zero(2);

                DVec<Scalar> path1_joints(2), path2_joints(1);
                path1_joints << joint_pos(0), joint_pos(2);
                path2_joints << joint_pos(1);

                Scalar cumulative_angle = 0.;
                DVec<Scalar> path1 = DVec<Scalar>::Zero(2);
                for (size_t i = 0; i < links_in_path1_; i++)
                {
                    cumulative_angle += path1_joints(i);
                    path1(0) += path1_link_lengths_[i] * cos(cumulative_angle);
                    path1(1) += path1_link_lengths_[i] * sin(cumulative_angle);
                }

                cumulative_angle = 0.;
                DVec<Scalar> path2 = offset;
                for (size_t i = 0; i < links_in_path2_; i++)
                {
                    cumulative_angle += path2_joints(i);
                    path2(0) += path2_link_lengths_[i] * cos(cumulative_angle);
                    path2(1) += path2_link_lengths_[i] * sin(cumulative_angle);
                }

                phi = path1 - path2;
                return phi;
            };

            this->G_ = DMat<Scalar>::Zero(3, 1);
            this->g_ = DVec<Scalar>::Zero(3);

            this->K_ = DMat<Scalar>::Zero(2, 3);
            this->k_ = DVec<Scalar>::Zero(2);

            switch (independent_coordinate_)
            {
            case 0:
                indepenent_coordinate_map_ << 1., 0., 0.,
                    0., 1., 0.,
                    0., 0., 1.;
                break;
            case 1:
                indepenent_coordinate_map_ << 0., 1., 0.,
                    1., 0., 0.,
                    0., 0., 1.;
                break;
            case 2:
                indepenent_coordinate_map_ << 0., 1., 0.,
                    0., 0., 1.,
                    1., 0., 0.;
                break;
            default:
                throw std::runtime_error("FourBar: Invalid independent coordinate");
            }
        }

        template <typename Scalar>
        void FourBar<Scalar>::updateJacobians(const JointCoordinate<Scalar> &joint_pos)
        {
            updateImplicitJacobian(joint_pos);
            updateExplicitJacobian(this->K_);
        }

        // Override isValidSpanningPosition to use tight tolerance
        // FourBar phi uses standard C++ trig functions (cos, sin) which work correctly
        // with complex types and can achieve machine precision
        template <typename Scalar>
        bool FourBar<Scalar>::isValidSpanningPosition(const JointCoordinate<Scalar> &joint_pos) const
        {
            if (!joint_pos.isSpanning()) {
                return false;
            }

            DVec<Scalar> violation = this->phi_(joint_pos);

            // Tight tolerance - FourBar constraints can achieve machine precision
            // since they use standard C++ trig functions, not CasADi
            const double tol = 1e-8;
            return nearZeroDefaultTrue(violation, static_cast<Scalar>(tol));
        }

        template <typename Scalar>
        void FourBar<Scalar>::updateImplicitJacobian(const JointCoordinate<Scalar> &joint_pos)
        {
            DVec<Scalar> q1(2), q2(1);
            q1 << joint_pos(0), joint_pos(2);
            q2 << joint_pos(1);
            
            Scalar cumulative_angle = 0.;
            DMat<Scalar> K1 = DMat<Scalar>::Zero(2, links_in_path1_);
            for (size_t i = 0; i < path1_link_lengths_.size(); i++)
            {
                cumulative_angle += q1(i);
                for (size_t j = 0; j <= i; j++)
                {
                    K1(0, j) += -path1_link_lengths_[i] * sin(cumulative_angle);
                    K1(1, j) += path1_link_lengths_[i] * cos(cumulative_angle);
                }
            }

            cumulative_angle = 0.;
            DMat<Scalar> K2 = DMat<Scalar>::Zero(2, links_in_path2_);
            for (size_t i = 0; i < links_in_path2_; i++)
            {
                cumulative_angle += q2(i);
                for (size_t j = 0; j <= i; j++)
                {
                    K2(0, j) += path2_link_lengths_[i] * sin(cumulative_angle);
                    K2(1, j) += -path2_link_lengths_[i] * cos(cumulative_angle);
                }
            }

            this->K_ << K1.col(0), K2.col(0), K1.col(1);
        }

        template <typename Scalar>
        void FourBar<Scalar>::updateExplicitJacobian(const DMat<Scalar> &K)
        {
            DVec<Scalar> Ki(2, 1);
            DMat<Scalar> Kd(2, 2);
            int dep_counter = 0;
            for (int i = 0; i < 3; i++)
            {
                if (i == independent_coordinate_)
                {
                    Ki = K.col(i);
                }
                else
                {
                    Kd.col(dep_counter) = K.col(i);
                    dep_counter++;
                }
            }

            Kd_inv_ = InverseType(Kd);
            this->G_(0, 0) = 1.;
            this->G_.template bottomRows<2>() = -Kd_inv_.solve(Ki);
            this->G_ = indepenent_coordinate_map_ * this->G_;
        }

        template <typename Scalar>
        void FourBar<Scalar>::updateBiases(const JointState<Scalar> &joint_state)
        {
            const JointCoordinate<Scalar> &joint_pos = joint_state.position;
            const JointCoordinate<Scalar> &joint_vel = joint_state.velocity;

            DVec<Scalar> q1(2), q2(1), qd1(2), qd2(1);
            q1 << joint_pos(0), joint_pos(2);
            q2 << joint_pos(1);
            qd1 << joint_vel(0), joint_vel(2);
            qd2 << joint_vel(1);

            // Update k
            Scalar cumulative_angle = 0.;
            Scalar cumulative_velocity = 0.;
            DMat<Scalar> Kd1 = DMat<Scalar>::Zero(2, links_in_path1_);
            for (size_t i = 0; i < path1_link_lengths_.size(); i++)
            {
                cumulative_angle += q1(i);
                cumulative_velocity += qd1(i);
                for (size_t j = 0; j <= i; j++)
                {
                    Kd1(0, j) += -path1_link_lengths_[i] *
                                 cumulative_velocity * cos(cumulative_angle);
                    Kd1(1, j) += -path1_link_lengths_[i] *
                                 cumulative_velocity * sin(cumulative_angle);
                }
            }

            cumulative_angle = 0.;
            cumulative_velocity = 0.;
            DMat<Scalar> Kd2 = DMat<Scalar>::Zero(2, links_in_path2_);
            for (size_t i = 0; i < links_in_path2_; i++)
            {
                cumulative_angle += q2(i);
                cumulative_velocity += qd2(i);
                for (size_t j = 0; j <= i; j++)
                {
                    Kd2(0, j) += path2_link_lengths_[i] *
                                 cumulative_velocity * cos(cumulative_angle);
                    Kd2(1, j) += path2_link_lengths_[i] *
                                 cumulative_velocity * sin(cumulative_angle);
                }
            }

            DMat<Scalar> Kdot(2, 3);
            Kdot << Kd1.col(0), Kd2.col(0), Kd1.col(1);
            this->k_ = -Kdot * joint_vel;

            // Update g
            this->g_(0) = 0.;
            this->g_.template bottomRows<2>() = Kd_inv_.solve(this->k_);
            this->g_ = indepenent_coordinate_map_ * this->g_;
        }

        // TODO(@MatthewChignoli): This is the same as generic joint, so do we need it? Probably not. In fact, we can probably deprecate this entire class.
        template <typename Scalar>
        void FourBar<Scalar>::createRandomStateHelpers()
        {
            if (this->random_state_helpers_.created)
            {
                return;
            }
            this->random_state_helpers_.created = true;

            using SX = casadi::SX;

            // Create symbolic four bar loop constraint
            std::vector<SX> path1_link_lengths_sym, path2_link_lengths_sym;
            for (size_t i = 0; i < path1_link_lengths_.size(); i++)
            {
                if constexpr (std::is_same<Scalar, casadi::SX>::value)
                {
                    path1_link_lengths_sym.push_back(path1_link_lengths_[i]);
                }
                else
                {
                    using std::real;
                    path1_link_lengths_sym.push_back(real(path1_link_lengths_[i])); 
                }

            }
            for (size_t i = 0; i < path2_link_lengths_.size(); i++)
            {
                if constexpr (std::is_same<Scalar, casadi::SX>::value)
                {
                    path2_link_lengths_sym.push_back(path2_link_lengths_[i]);
                }
                else
                {
                    using std::real;
                    path2_link_lengths_sym.push_back(real(path2_link_lengths_[i]));
                }
            }
            Vec2<SX> offset_sym;
            if constexpr (std::is_same<Scalar, casadi::SX>::value)
            {
                offset_sym = Vec2<SX>{offset_[0], offset_[1]};
            }
            else
            {
                using std::real;
                offset_sym = Vec2<SX>{real(offset_[0]), real(offset_[1])};
            }
            FourBar<SX> symbolic = FourBar<SX>(path1_link_lengths_sym,
                                               path2_link_lengths_sym,
                                               offset_sym, independent_coordinate_);

            // Root finding
            {
                SX cs_q_sym = SX::sym("q", this->numSpanningPos());
                DVec<SX> q_sym(this->numSpanningPos());
                casadi::copy(cs_q_sym, q_sym);

                // Compute constraint violation
                JointCoordinate<SX> joint_pos(q_sym, true);
                DVec<SX> phi_sx = symbolic.phi(joint_pos);
                SX cs_phi_sym = casadi::SX(casadi::Sparsity::dense(phi_sx.rows(), 1));
                casadi::copy(phi_sx, cs_phi_sym);

                // Slice depending on independent coordinate
                casadi::Slice ind_slice, dep_slice;
                switch (independent_coordinate_)
                {
                case 0:
                    ind_slice = casadi::Slice(0);
                    dep_slice = casadi::Slice(1, 3);
                    break;
                case 1:
                    ind_slice = casadi::Slice(1);
                    dep_slice = casadi::Slice(0, 3, 2);
                    break;
                case 2:
                    ind_slice = casadi::Slice(2);
                    dep_slice = casadi::Slice(0, 2);
                    break;
                default:
                    throw std::runtime_error("FourBar: Invalid independent coordinate");
                }

                // Create rootfinder problem
                casadi::SXDict rootfinder_problem;
                rootfinder_problem["x"] = cs_q_sym(dep_slice);
                rootfinder_problem["p"] = cs_q_sym(ind_slice);
                rootfinder_problem["g"] = cs_phi_sym;
                casadi::Dict options;
                options["expand"] = true;
                options["error_on_fail"] = true;
                this->random_state_helpers_.phi_root_finder = casadi::rootfinder("solver", "newton",
                                                                                 rootfinder_problem,
                                                                                 options);
            }

            // Explicit constraint jacobian
            {
                SX cs_q_sym = SX::sym("q", this->numSpanningPos());
                DVec<SX> q_sym(this->numSpanningPos());
                casadi::copy(cs_q_sym, q_sym);
                JointCoordinate<SX> joint_pos(q_sym, false);
                symbolic.updateJacobians(joint_pos);
                DMat<SX> G = symbolic.G();
                SX G_sym = casadi::SX(casadi::Sparsity::dense(G.rows(), G.cols()));
                casadi::copy(G, G_sym);
                this->random_state_helpers_.G = casadi::Function("G", {cs_q_sym}, {G_sym}, {"q"}, {"G"});
            }
        }

        template struct FourBar<double>;
        template struct FourBar<std::complex<double>>;
        template struct FourBar<casadi::SX>;

    } // namespace LoopConstraint

    namespace ClusterJoints
    {
        template <typename Scalar>
        JointState<double> FourBar<Scalar>::randomJointState(bool enforce_position_constraint) const
        {
            if (!enforce_position_constraint)
                return Base<Scalar>::randomJointState();

            using DM = casadi::DM;

            // Create Helper functions
            four_bar_constraint_->createRandomStateHelpers();

            // Random independent position coordinate
            const int n_ind = four_bar_constraint_->numIndependentPos();
            const int n_span = four_bar_constraint_->numSpanningPos();
            double ind_range = 1.0;
            double dep_range = 0.1;
            DM q_ind, q_dep;

            // Call the rootfinder to get dependent position coordinates
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
                    q_dep = four_bar_constraint_->random_state_helpers_.phi_root_finder(arg).at("x");
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

            DM q_dm;
            switch (four_bar_constraint_->independent_coordinate())
            {
            case 0:
                q_dm = DM::vertcat({q_ind, q_dep});
                break;
            case 1:
                q_dm = DM::vertcat({q_dep(0), q_ind, q_dep(1)});
                break;
            case 2:
                q_dm = DM::vertcat({q_dep(0), q_dep(1), q_ind});
                break;
            }
            DVec<double> q(four_bar_constraint_->numSpanningPos());
            casadi::copy(q_dm, q);
            JointCoordinate<double> joint_pos(q, true);

            // Random independent joint velocity
            DVec<double> v = DVec<double>::Random(four_bar_constraint_->numIndependentVel());
            JointCoordinate<double> joint_vel(v, false);

            return JointState<double>(joint_pos, joint_vel);
        }

        template <typename Scalar>
        std::vector<DMat<Scalar>> FourBar<Scalar>::getSq() const
        {
            using std::sin;
            using std::cos;

            // Get dimensions
            const int mss_dim = this->num_bodies_ * 6;  // motion subspace spatial dimension
            const int nv = this->num_velocities_;       // number of independent velocities (1 for FourBar)
            const int n_span = four_bar_constraint_->numSpanningPos();  // 3 for FourBar

            // Initialize result: S_q[i] is the derivative of S w.r.t. the i-th independent coordinate
            std::vector<DMat<Scalar>> S_q(nv, DMat<Scalar>::Zero(mss_dim, nv));

            // S = X_intra(q) * S_spanning * G(q)
            //
            // dS/dy_i = sum_j (dS/dq_j * G(j, yi))  [chain rule with G mapping y -> q]
            //
            // dS/dq_j = dX_intra/dq_j * S_spanning * G + X_intra * S_spanning * dG/dq_j
            //
            // Term 1: dX_intra/dq_j * S_spanning * G
            //   This involves the derivative of the intra-cluster transform.
            //   X_intra depends on joint transforms which depend on q.
            //
            // Term 2: X_intra * S_spanning * dG/dq_j
            //   This involves the derivative of the constraint Jacobian.

            // Get constraint parameters
            const auto& K = four_bar_constraint_->K();
            const auto& G = four_bar_constraint_->G();
            const int ind_coord = four_bar_constraint_->independent_coordinate();
            const auto& path1_lengths = four_bar_constraint_->path1LinkLengths();
            const auto& path2_lengths = four_bar_constraint_->path2LinkLengths();
            const auto& coord_map = four_bar_constraint_->independentCoordinateMap();

            if (this->q_cache_.size() == 0) {
                return S_q;
            }
            const DVec<Scalar>& q = this->q_cache_;

            DVec<Scalar> q1(2), q2(1);
            q1 << q(0), q(2);
            q2 << q(1);

            Scalar angle_sum = q1(0) + q1(1);

            // ===== Term 2: Compute X_intra * S_spanning * dG/dq =====
            // dK/dq for each spanning coordinate
            DMat<Scalar> dK_dq0 = DMat<Scalar>::Zero(2, 3);
            dK_dq0(0, 0) = -path1_lengths[0] * cos(q1(0)) - path1_lengths[1] * cos(angle_sum);
            dK_dq0(1, 0) = -path1_lengths[0] * sin(q1(0)) - path1_lengths[1] * sin(angle_sum);
            dK_dq0(0, 2) = -path1_lengths[1] * cos(angle_sum);
            dK_dq0(1, 2) = -path1_lengths[1] * sin(angle_sum);

            DMat<Scalar> dK_dq1 = DMat<Scalar>::Zero(2, 3);
            dK_dq1(0, 1) = path2_lengths[0] * cos(q2(0));
            dK_dq1(1, 1) = path2_lengths[0] * sin(q2(0));

            DMat<Scalar> dK_dq2 = DMat<Scalar>::Zero(2, 3);
            dK_dq2(0, 0) = -path1_lengths[1] * cos(angle_sum);
            dK_dq2(1, 0) = -path1_lengths[1] * sin(angle_sum);
            dK_dq2(0, 2) = -path1_lengths[1] * cos(angle_sum);
            dK_dq2(1, 2) = -path1_lengths[1] * sin(angle_sum);

            std::array<DMat<Scalar>, 3> dK_dq = {dK_dq0, dK_dq1, dK_dq2};

            // Extract Ki and Kd
            DVec<Scalar> Ki(2);
            DMat<Scalar> Kd(2, 2);
            int dep_col = 0;
            for (int i = 0; i < 3; i++) {
                if (i == ind_coord) {
                    Ki = K.col(i);
                } else {
                    Kd.col(dep_col++) = K.col(i);
                }
            }

            DVec<Scalar> Kd_inv_Ki = four_bar_constraint_->KdInverse().solve(Ki);

            // Compute dG/dq_j for each spanning coordinate
            std::array<DVec<Scalar>, 3> dG_dq;
            for (int j = 0; j < n_span; ++j) {
                DVec<Scalar> dKi_dqj(2);
                DMat<Scalar> dKd_dqj(2, 2);
                dep_col = 0;
                for (int i = 0; i < 3; i++) {
                    if (i == ind_coord) {
                        dKi_dqj = dK_dq[j].col(i);
                    } else {
                        dKd_dqj.col(dep_col++) = dK_dq[j].col(i);
                    }
                }

                DVec<Scalar> d_Kdinv_Ki_dqj = four_bar_constraint_->KdInverse().solve(
                    dKi_dqj - dKd_dqj * Kd_inv_Ki);

                DVec<Scalar> dG_dqj_before_map(3);
                dG_dqj_before_map << Scalar(0), -d_Kdinv_Ki_dqj;
                dG_dq[j] = coord_map * dG_dqj_before_map;
            }

            // ===== Term 1: Compute dX_intra/dq * S_spanning * G =====
            // For FourBar with 3 revolute joints:
            // X_intra has structure based on tree connectivity.
            // Joint i affects X_intra blocks downstream of joint i.
            //
            // For revolute joint at angle q_j:
            // dX_intra_block/dq_j = -crm([0;0;1;0;0;0]) * X_intra_block
            //
            // We compute this using the structure of the FourBar mechanism.

            const DMat<Scalar>& S_spanning = this->S_spanning_;
            const DMat<Scalar>& X_intra = this->X_intra_;
            DMat<Scalar> S_implicit = X_intra * S_spanning;

            // dX_intra/dq_j for each spanning coordinate
            // For a revolute joint, dXJ/dq = -crm([0;0;1;0;0;0]) * XJ = -crm(s_axis) * XJ
            // where s_axis is the joint axis in spatial coordinates.
            //
            // For FourBar mechanism with 3 revolute joints along z-axis:
            // dX_intra[i,j]/dq_k depends on which joint k affects the transform from j to i.
            //
            // Since computing this analytically is complex, we use the fact that
            // for revolute joints, dX/dq * v = crm(X * s_axis) * X * v = crm(s) * (X * v)
            // where s is the joint axis expressed in the current frame.

            // For simplicity, compute dS/dq_j directly using the S_ring structure.
            // Actually, S_ring = dX_intra/dt * S_spanning * G, where dt involves qd.
            // We need dX_intra/dq_j, not dX_intra/dt.
            //
            // For each revolute joint j, dX_intra/dq_j affects blocks (i,k) where
            // joint j is on the path from body k to body i.
            //
            // For FourBar topology, this requires knowing the tree structure.
            // Let's use the fact that X_intra_ring encodes this information scaled by velocity.

            // Alternative: Use chain rule through the motion subspace
            // For revolute joints: dS/dq = crm(s) * S where s is the unit axis
            // This gives us a way to compute dX_intra_S_span/dq without full X_intra derivatives.

            // Actually, for implicit constraints with configuration-dependent G:
            // The key insight is that both terms contribute.
            //
            // Term 1 (dX_intra/dq contribution) is captured in S_ring but scaled by velocity.
            // For per-position derivatives, we need to unscale.
            //
            // Let's compute Term 1 using the revolute joint axis structure:
            // For joint j at position q_j, dXJ_j/dq_j = -crm(axis) * XJ_j
            // Then dX_intra/dq_j propagates through the tree.

            // ===== Compute dX_intra/dq_j for the FourBar topology =====
            // PlanarLegLinkage FourBar structure:
            // - Body 0 (shank_driver): parent outside cluster
            // - Body 1 (shank_support): parent outside cluster
            // - Body 2 (foot): parent = body 0 (shank_driver)
            //
            // X_intra structure:
            // - X_intra[2,0] = XJ_foot * Xtree_foot (depends on q[2], the foot joint angle)
            // - Other blocks don't depend on positions within the cluster
            //
            // For revolute joint: dXJ/dq = -crm(axis) * XJ
            // where axis = [0,0,1,0,0,0] for z-axis revolute
            //
            // dX_intra[2,0]/dq[2] = -crm(axis) * X_intra[2,0]
            // dX_intra[2,0]/dq[0] = 0 (joint 0 is outside the path from body 0 to body 2)
            // dX_intra[2,0]/dq[1] = 0 (joint 1 is outside the path from body 0 to body 2)

            // S_spanning structure for 3 revolute joints:
            // S_spanning = diag([s0, s1, s2]) where s_i = [0,0,1,0,0,0]^T for z-axis
            // It's an 18x3 matrix with 6x1 blocks on the diagonal

            // X_intra * S_spanning structure:
            // Row block i corresponds to body i, column j corresponds to joint j
            // (X_intra * S_spanning)[body i, joint j] = X_intra[i,j] * s_j

            // For the FourBar:
            // (X_intra * S_spanning)[0,0] = I * s0 = s0 (body 0, joint 0)
            // (X_intra * S_spanning)[0,1] = 0 (body 0, joint 1 - no connectivity)
            // (X_intra * S_spanning)[0,2] = 0 (body 0, joint 2 - no connectivity)
            // (X_intra * S_spanning)[1,0] = 0 (body 1, joint 0 - no connectivity)
            // (X_intra * S_spanning)[1,1] = I * s1 = s1 (body 1, joint 1)
            // (X_intra * S_spanning)[1,2] = 0 (body 1, joint 2 - no connectivity)
            // (X_intra * S_spanning)[2,0] = X_intra[2,0] * s0 (body 2, joint 0)
            // (X_intra * S_spanning)[2,1] = 0 (body 2, joint 1 - no connectivity in standard FourBar)
            // (X_intra * S_spanning)[2,2] = I * s2 = s2 (body 2, joint 2)

            // d(X_intra * S_spanning)/dq[2]:
            // Only affects rows corresponding to body 2, column 0:
            // d(X_intra[2,0] * s0)/dq[2] = dX_intra[2,0]/dq[2] * s0 = -crm(axis) * X_intra[2,0] * s0

            // Get the current X_intra[2,0] block (rows 12-17, cols 0-5)
            Mat6<Scalar> X_intra_20 = X_intra.template block<6, 6>(12, 0);

            // Revolute joint axis (z-axis)
            SVec<Scalar> axis;
            axis << Scalar(0), Scalar(0), Scalar(1), Scalar(0), Scalar(0), Scalar(0);

            // d(X_intra * S_spanning)/dq[2] (only body 2, joint 0 is affected)
            // = -crm(axis) * X_intra[2,0] * S_spanning column 0
            SVec<Scalar> X_intra_20_s0 = X_intra_20 * axis;  // X_intra[2,0] * s0
            SVec<Scalar> dXintra_Sspan_dq2_block = -spatial::motionCrossProduct(axis, X_intra_20_s0);

            // Build full dX_intra_S_span/dq matrices
            std::array<DMat<Scalar>, 3> dXintra_Sspan_dq;
            for (int j = 0; j < n_span; ++j) {
                dXintra_Sspan_dq[j] = DMat<Scalar>::Zero(mss_dim, n_span);
            }
            // Only q[2] affects X_intra_S_spanning (at block [body 2, joint 0])
            dXintra_Sspan_dq[2].template block<6, 1>(12, 0) = dXintra_Sspan_dq2_block;

            // Final S_q computation combining both terms
            for (int yi = 0; yi < nv; ++yi) {
                // Term 2: X_intra * S_spanning * sum_j(dG/dq_j * G(j, yi))
                DVec<Scalar> dG_dy(n_span);
                dG_dy.setZero();
                for (int j = 0; j < n_span; ++j) {
                    dG_dy += dG_dq[j] * G(j, yi);
                }
                DVec<Scalar> term2 = S_implicit * dG_dy;

                // Term 1: sum_j(dX_intra_S_span/dq_j * G * G(j, yi))
                DVec<Scalar> term1 = DVec<Scalar>::Zero(mss_dim);
                for (int j = 0; j < n_span; ++j) {
                    // dX_intra_S_span/dq_j * G gives (mss_dim x 1) vector
                    // Scale by G(j, yi) to get contribution from spanning coord j
                    DVec<Scalar> dXS_G = dXintra_Sspan_dq[j] * G;
                    term1 += dXS_G * G(j, yi);
                }

                S_q[yi].col(yi) = term1 + term2;
            }

            return S_q;
        }

        template class FourBar<double>;
        template class FourBar<std::complex<double>>;
        template class FourBar<casadi::SX>;
    }
}
