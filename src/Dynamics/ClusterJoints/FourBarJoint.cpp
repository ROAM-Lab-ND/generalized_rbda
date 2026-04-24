#include "grbda/Dynamics/ClusterJoints/FourBarJoint.h"
#include <iostream>

namespace grbda
{
    namespace LoopConstraint
    {
        // ---------------------------------------------------------------------------
        // Factory helpers for GenericImplicit constructor
        // ---------------------------------------------------------------------------

        namespace
        {
            // Build the is_coordinate_independent mask (3 coords, one independent)
            std::vector<bool> makeFourBarIndMask(int ind_coord)
            {
                std::vector<bool> mask = {false, false, false};
                mask[ind_coord] = true;
                return mask;
            }

            // Build the symbolic phi (always SX; captures link lengths as SX constants)
            template <typename Scalar>
            std::function<DVec<casadi::SX>(const JointCoordinate<casadi::SX> &)>
            makeFourBarSymPhi(const std::vector<Scalar> &p1, const std::vector<Scalar> &p2,
                              const Vec2<Scalar> &off, size_t n1, size_t n2)
            {
                using SX = casadi::SX;

                std::vector<SX> p1_sx, p2_sx;
                Vec2<SX> off_sx;

                if constexpr (std::is_same_v<Scalar, SX>)
                {
                    p1_sx = p1;
                    p2_sx = p2;
                    off_sx << off[0], off[1];
                }
                else
                {
                    using std::real;
                    for (const auto &x : p1) p1_sx.push_back(SX(real(x)));
                    for (const auto &x : p2) p2_sx.push_back(SX(real(x)));
                    off_sx << SX(real(off[0])), SX(real(off[1]));
                }

                return [p1_sx, p2_sx, off_sx, n1, n2](const JointCoordinate<SX> &jp) -> DVec<SX>
                {
                    DVec<SX> pj1(2), pj2(1);
                    pj1 << jp(0), jp(2);
                    pj2 << jp(1);

                    SX ca = SX(0.);
                    DVec<SX> path1 = DVec<SX>::Zero(2);
                    for (size_t i = 0; i < n1; i++)
                    {
                        ca = ca + pj1(i);
                        path1(0) = path1(0) + p1_sx[i] * cos(ca);
                        path1(1) = path1(1) + p1_sx[i] * sin(ca);
                    }

                    DVec<SX> path2(2);
                    path2 << off_sx[0], off_sx[1];
                    ca = SX(0.);
                    for (size_t i = 0; i < n2; i++)
                    {
                        ca = ca + pj2(i);
                        path2(0) = path2(0) + p2_sx[i] * cos(ca);
                        path2(1) = path2(1) + p2_sx[i] * sin(ca);
                    }

                    DVec<SX> phi = path1 - path2;
                    return phi;
                };
            }

            // Build the native phi (works with any Scalar, including complex and SX)
            template <typename Scalar>
            std::function<DVec<Scalar>(const JointCoordinate<Scalar> &)>
            makeFourBarNativePhi(std::vector<Scalar> p1, std::vector<Scalar> p2,
                                 Vec2<Scalar> off, size_t n1, size_t n2)
            {
                return [p1, p2, off, n1, n2](const JointCoordinate<Scalar> &jp) -> DVec<Scalar>
                {
                    using std::cos;
                    using std::sin;

                    DVec<Scalar> pj1(2), pj2(1);
                    pj1 << jp(0), jp(2);
                    pj2 << jp(1);

                    Scalar ca = Scalar(0.);
                    DVec<Scalar> path1 = DVec<Scalar>::Zero(2);
                    for (size_t i = 0; i < n1; i++)
                    {
                        ca += pj1(i);
                        path1(0) += p1[i] * cos(ca);
                        path1(1) += p1[i] * sin(ca);
                    }

                    DVec<Scalar> path2 = off;
                    ca = Scalar(0.);
                    for (size_t i = 0; i < n2; i++)
                    {
                        ca += pj2(i);
                        path2(0) += p2[i] * cos(ca);
                        path2(1) += p2[i] * sin(ca);
                    }

                    return path1 - path2;
                };
            }
        } // anonymous namespace

        // ---------------------------------------------------------------------------
        // FourBar constructor
        // ---------------------------------------------------------------------------

        template <typename Scalar>
        FourBar<Scalar>::FourBar(std::vector<Scalar> path1_link_lengths,
                                 std::vector<Scalar> path2_link_lengths,
                                 Vec2<Scalar> offset, int independent_coordinate)
            : GenericImplicit<Scalar>(
                  makeFourBarIndMask(independent_coordinate),
                  makeFourBarSymPhi<Scalar>(path1_link_lengths, path2_link_lengths, offset,
                                            path1_link_lengths.size(),
                                            path2_link_lengths.size()),
                  makeFourBarNativePhi<Scalar>(path1_link_lengths, path2_link_lengths, offset,
                                               path1_link_lengths.size(),
                                               path2_link_lengths.size())),
              links_in_path1_(path1_link_lengths.size()),
              links_in_path2_(path2_link_lengths.size()),
              path1_link_lengths_(path1_link_lengths),
              path2_link_lengths_(path2_link_lengths),
              offset_(offset),
              independent_coordinate_(independent_coordinate)
        {
            if (links_in_path1_ + links_in_path2_ != 3)
            {
                throw std::runtime_error("FourBar: Must contain 3 links");
            }

            // Restore phi_ to the native computation so it works for all scalar types
            // (GenericImplicit sets phi_ to a CasADi-backed version that only handles real inputs)
            this->phi_ = makeFourBarNativePhi<Scalar>(path1_link_lengths_, path2_link_lengths_,
                                                      offset_, links_in_path1_, links_in_path2_);

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

        template <typename Scalar>
        void FourBar<Scalar>::createRandomStateHelpers()
        {
            if (this->random_state_helpers_.created)
            {
                return;
            }
            this->random_state_helpers_.created = true;

            using SX = casadi::SX;

            // Build a symbolic phi using the factory (avoids constructing a full FourBar<SX>)
            auto sym_phi = makeFourBarSymPhi<Scalar>(path1_link_lengths_, path2_link_lengths_,
                                                      offset_, links_in_path1_, links_in_path2_);

            // Root finding
            {
                SX cs_q_sym = SX::sym("q", this->numSpanningPos());
                DVec<SX> q_sym(this->numSpanningPos());
                casadi::copy(cs_q_sym, q_sym);

                JointCoordinate<SX> joint_pos(q_sym, true);
                DVec<SX> phi_sx = sym_phi(joint_pos);
                SX cs_phi_sym = casadi::SX(casadi::Sparsity::dense(phi_sx.rows(), 1));
                casadi::copy(phi_sx, cs_phi_sym);

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

            // Explicit constraint jacobian for random state generation
            {
                SX cs_q_sym = SX::sym("q", this->numSpanningPos());
                DVec<SX> q_sym(this->numSpanningPos());
                casadi::copy(cs_q_sym, q_sym);

                // Use a temporary FourBar<SX> only for G (updateJacobians is analytic)
                std::vector<SX> path1_sx, path2_sx;
                Vec2<SX> offset_sx;
                if constexpr (std::is_same_v<Scalar, SX>)
                {
                    path1_sx = path1_link_lengths_;
                    path2_sx = path2_link_lengths_;
                    offset_sx << offset_[0], offset_[1];
                }
                else
                {
                    using std::real;
                    for (const auto &l : path1_link_lengths_) path1_sx.push_back(SX(real(l)));
                    for (const auto &l : path2_link_lengths_) path2_sx.push_back(SX(real(l)));
                    offset_sx << SX(real(offset_[0])), SX(real(offset_[1]));
                }
                FourBar<SX> symbolic(path1_sx, path2_sx, offset_sx, independent_coordinate_);

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

        template class FourBar<double>;
        template class FourBar<std::complex<double>>;
        template class FourBar<casadi::SX>;
    }
}
