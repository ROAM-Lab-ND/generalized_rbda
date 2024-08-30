#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"
#include "grbda/Utils/Utilities.h"

namespace grbda
{

    namespace LoopConstraint
    {
        template <typename Scalar>
        GenericImplicit<Scalar>::GenericImplicit(std::vector<bool> is_coordinate_independent,
                                                 SymPhiFcn phi_fcn) : phi_sym_(phi_fcn)
        {
            this->is_coordinate_independent_ = is_coordinate_independent;

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
            casadi::copy(cs_q_sym, q_sym);
            JointCoordinate<SX> joint_pos_sym(q_sym, true);

            SX cs_v_sym = SX::sym("v", state_dim, 1);
            DVec<SX> v_sym(state_dim);
            casadi::copy(cs_v_sym, v_sym);

            // Implicit constraint violation function
            DVec<SX> phi_sym = phi_fcn(joint_pos_sym);
            const int constraint_dim = phi_sym.rows();
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
            cs_G_sym(dep_slice, casadi::Slice()) = -SX::mtimes(SX::inv(cs_Kd_sym), cs_Ki_sym);
            cs_G_sym = SX::mtimes(coord_map, cs_G_sym);

            // Explicit constraints bias
            SX cs_g_sym = SX::zeros(state_dim, 1);
            cs_g_sym(dep_slice) = SX::mtimes(SX::inv(cs_Kd_sym), cs_k_sym);
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
        DMat<Scalar> GenericImplicit<Scalar>::runCasadiFcn(const casadi::Function &fcn,
                                                           const JointCoordinate<Scalar> &arg)
        {
            using CasadiScalar = typename std::conditional<std::is_same<Scalar, casadi::SX>::value, casadi::SX, casadi::DM>::type;
            using CasadiResult = typename std::conditional<std::is_same<Scalar, float>::value, double, Scalar>::type;

            CasadiScalar arg_cs(arg.rows());
            casadi::copy(arg, arg_cs);
            CasadiScalar res_cs = fcn(arg_cs)[0];
            DMat<CasadiResult> res(res_cs.size1(), res_cs.size2());
            casadi::copy(res_cs, res);

            return res.template cast<Scalar>();
        }

        template <typename Scalar>
        DMat<Scalar> GenericImplicit<Scalar>::runCasadiFcn(const casadi::Function &fcn,
                                                           const JointState<Scalar> &args)
        {
            using CasadiScalar = typename std::conditional<std::is_same<Scalar, casadi::SX>::value, casadi::SX, casadi::DM>::type;
            using CasadiResult = typename std::conditional<std::is_same<Scalar, float>::value, double, Scalar>::type;

            std::vector<CasadiScalar> args_cs(2);
            args_cs[0] = CasadiScalar(args.position.rows());
            casadi::copy(args.position, args_cs[0]);
            args_cs[1] = CasadiScalar(args.velocity.rows());
            casadi::copy(args.velocity, args_cs[1]);

            CasadiScalar res_cs = fcn(args_cs)[0];
            DMat<CasadiResult> res(res_cs.size1(), res_cs.size2());
            casadi::copy(res_cs, res);

            return res.template cast<Scalar>();
        }

        template <typename Scalar>
        void GenericImplicit<Scalar>::createRandomStateHelpers()
        {
            if (this->random_state_helpers_.created)
            {
                return;
            }
            this->random_state_helpers_.created = true;

            using SX = casadi::SX;

            // Create symbolic generic implicit loop constraint
            GenericImplicit<SX> symbolic = GenericImplicit<SX>(this->is_coordinate_independent_, phi_sym_);

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
                options["error_on_fail"] = true;
                this->random_state_helpers_.phi_root_finder = casadi::rootfinder("solver", "newton",
                                                                           rootfinder_problem,
                                                                           options);
            }

            // Explicit constraint Jacobian
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

        template struct GenericImplicit<double>;
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
                           loop_constraint->numIndependentPos(),
                           loop_constraint->numIndependentVel()),
              bodies_(bodies)
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
        JointState<double> Generic<Scalar>::randomJointState() const
        {
            using DM = casadi::DM;

            // Create Helper functions
            this->loop_constraint_->createRandomStateHelpers();

            // Random independent position coordinate
            const int n_ind = this->loop_constraint_->numIndependentPos();
            const int n_span = this->loop_constraint_->numSpanningPos();
            double range = 6.28;
            DM q_ind = range * (2. * DM::rand(n_ind) - 1.);
            DM q_dep_guess = range * (2. * DM::rand(n_span - n_ind) - 1.);

            // Call the rootfinder to get dependent position coordinates
            casadi::DMDict arg;
            arg["p"] = q_ind;
            arg["x0"] = q_dep_guess;
            DM q_dep = this->loop_constraint_->random_state_helpers_.phi_root_finder(arg).at("x");

            DM q_dm(n_span, 1);
            int ind_cnt = 0, dep_cnt = 0;
            for (int i = 0; i < n_span; i++)
            {
                if (this->loop_constraint_->is_coordinate_independent_[i])
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
            JointCoordinate<double> joint_pos(q, true);

            // Random independent joint velocity
            DVec<double> v = DVec<double>::Random(this->loop_constraint_->numIndependentVel());
            JointCoordinate<double> joint_vel(v, false);

            return JointState<double>(joint_pos, joint_vel);
        }

        template <typename Scalar>
        void Generic<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            const DVec<Scalar> &q = spanning_joint_state.position;
            const DVec<Scalar> &qd = spanning_joint_state.velocity;

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

            const DMat<Scalar> S_implicit = X_intra_ * S_spanning_;
            this->S_ = S_implicit * this->loop_constraint_->G();
            this->vJ_ = S_implicit * qd;

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
                        S_implicit * this->loop_constraint_->g();
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

        template class Generic<double>;
        template class Generic<float>;
        template class Generic<casadi::SX>;
    }

} // namespace grbda
