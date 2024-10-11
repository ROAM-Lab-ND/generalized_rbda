#include "grbda/Dynamics/ClusterJoints/FourBarJoint.h"

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
                path1_link_lengths_sym.push_back(path1_link_lengths_[i]);
            }
            for (size_t i = 0; i < path2_link_lengths_.size(); i++)
            {
                path2_link_lengths_sym.push_back(path2_link_lengths_[i]);
            }
            Vec2<SX> offset_sym{offset_[0], offset_[1]};
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
        template struct FourBar<casadi::SX>;

    } // namespace LoopConstraint

    namespace ClusterJoints
    {
        template <typename Scalar>
        JointState<double> FourBar<Scalar>::randomJointState() const
        {
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

        template class FourBar<double>;
        template class FourBar<casadi::SX>;
    }
}
