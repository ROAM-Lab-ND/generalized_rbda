#include "grbda/Dynamics/ClusterJoints/LoopConstraint.h"
#include "grbda/Utils/Utilities.h"

namespace grbda
{
    // TODO(@MatthewChignoli): Template so that we can use any MatrixBase
    casadi::DM toCasadiArg(const DVec<double> &vec)
    {
        casadi::DM cs_vec(vec.rows(), vec.cols());
        casadi::copy(vec, cs_vec);
        return cs_vec;
    }

    casadi::SX toCasadiArg(const DVec<casadi::SX> &vec)
    {
        casadi::SX cs_vec(vec.rows(), vec.cols());
        casadi::copy(vec, cs_vec);
        return cs_vec;
    }

    casadi::DMVector toCasadiArgVector(const JointState<double> &joint_state)
    {
        casadi::DMVector cs_vec(2);
        cs_vec[0] = toCasadiArg(joint_state.position);
        cs_vec[1] = toCasadiArg(joint_state.velocity);
        return cs_vec;
    }

    casadi::SXVector toCasadiArgVector(const JointState<casadi::SX> &joint_state)
    {
        casadi::SXVector cs_vec(2);
        cs_vec[0] = toCasadiArg(joint_state.position);
        cs_vec[1] = toCasadiArg(joint_state.velocity);
        return cs_vec;
    }

    // TODO(@MatthewChignoli): No need for separate vec and mat functions
    DVec<casadi::SX> toEigenVec(const casadi::SX &vec)
    {
        DVec<casadi::SX> vec_out(vec.rows());
        casadi::copy(vec, vec_out);
        return vec_out;
    }

    DVec<double> toEigenVec(const casadi::DM &vec)
    {
        DVec<double> vec_out(vec.rows());
        casadi::copy(vec, vec_out);
        return vec_out;
    }

    DMat<casadi::SX> toEigenMat(const casadi::SX &mat)
    {
        DMat<casadi::SX> mat_out(mat.size1(), mat.size2());
        casadi::copy(mat, mat_out);
        return mat_out;
    }

    DMat<double> toEigenMat(const casadi::DM &mat)
    {
        DMat<double> mat_out(mat.size1(), mat.size2());
        casadi::copy(mat, mat_out);
        return mat_out;
    }

    namespace LoopConstraint
    {

        template <typename Scalar>
        GenericImplicit<Scalar>::GenericImplicit(int state_dim, SymPhiFcn phi_fcn)
        {
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

            // Assign member variables using casadi functions
            this->phi_ = [cs_phi_fcn](const JointCoordinate<Scalar> &joint_pos)
            {
                return toEigenVec(cs_phi_fcn(toCasadiArg(joint_pos))[0]);
            };

            this->K_ = DMat<Scalar>::Zero(constraint_dim, state_dim);
            K_fcn_ = casadi::Function("K", {cs_q_sym}, {cs_K_sym});

            this->k_ = DVec<Scalar>::Zero(constraint_dim);
            k_fcn_ = casadi::Function("k", {cs_q_sym, cs_v_sym}, {cs_k_sym});

            // TODO(@MatthewChignoli): Still need lambda functions for G and g
        }

        template <typename Scalar>
        DVec<Scalar> GenericImplicit<Scalar>::gamma(const JointCoordinate<Scalar> &joint_pos) const
        {
            throw std::runtime_error("GenericImplicit::gamma() not implemented");
        }

        template <typename Scalar>
        void GenericImplicit<Scalar>::updateJacobians(const JointCoordinate<Scalar> &joint_pos)
        {
            this->K_ = toEigenMat(K_fcn_(toCasadiArg(joint_pos))[0]);
        }

        template <typename Scalar>
        void GenericImplicit<Scalar>::updateBiases(const JointState<Scalar> &joint_state)
        {
            this->k_ = toEigenVec(k_fcn_(toCasadiArgVector(joint_state))[0]);
        }

        template struct GenericImplicit<double>;
        // template struct GenericImplicit<float>;
        template struct GenericImplicit<casadi::SX>;

        template <typename Scalar>
        Static<Scalar>::Static(DMat<Scalar> G, DMat<Scalar> K)
        {
            this->G_ = G;
            this->g_ = DVec<Scalar>::Zero(G.rows());

            this->K_ = K;
            this->k_ = DVec<Scalar>::Zero(K.rows());
        }

        template <typename Scalar>
        DVec<Scalar> Static<Scalar>::gamma(const JointCoordinate<Scalar> &joint_pos) const
        {
            return this->G_ * joint_pos;
        }

        template struct Static<double>;
        template struct Static<float>;
        template struct Static<casadi::SX>;

        template <typename Scalar>
        DVec<Scalar> Collection<Scalar>::gamma(const DVec<Scalar> y) const
        {
            DVec<Scalar> spanning_pos(span_pos_cnt_);

            int span_pos_cnt = 0;
            int ind_pos_cnt = 0;
            for (auto constraint : *this)
            {
                const int n_span_pos = constraint->numSpanningPos();
                const int n_ind_pos = constraint->numIndependentPos();

                JointCoordinate<Scalar> position(y.segment(ind_pos_cnt, n_ind_pos), false);
                spanning_pos.segment(span_pos_cnt, n_span_pos) = constraint->gamma(position);

                span_pos_cnt += n_span_pos;
                ind_pos_cnt += n_ind_pos;
            }
            return spanning_pos;
        }

        template <typename Scalar>
        const DMat<Scalar> &Collection<Scalar>::G_transpose() const
        {
            if (!G_transpose_computed_)
            {
                G_transpose_ = G_.transpose();
                G_transpose_computed_ = true;
            }
            return G_transpose_;
        }

        template <typename Scalar>
        const DMat<Scalar> &Collection<Scalar>::G_pinv() const
        {
            if (!G_pinv_computed_)
            {
                G_pinv_ = matrixLeftPseudoInverse(G_);
                G_pinv_computed_ = true;
            }

            return G_pinv_;
        }

        template <typename Scalar>
        const DMat<Scalar> &Collection<Scalar>::G_tranpose_pinv() const
        {
            if (!G_tranpose_pinv_computed_)
            {
                G_tranpose_pinv_ = matrixRightPseudoInverse(G_transpose());
                G_tranpose_pinv_computed_ = true;
            }
            return G_tranpose_pinv_;
        }

        template <typename Scalar>
        const DMat<Scalar> &Collection<Scalar>::K_transpose() const
        {
            if (!K_transpose_computed_)
            {
                K_transpose_ = K_.transpose();
                K_transpose_computed_ = true;
            }
            return K_transpose_;
        }

        template <typename Scalar>
        void Collection<Scalar>::push_back(const std::shared_ptr<Base<Scalar>> loop_constraint)
        {
            std::vector<std::shared_ptr<Base<Scalar>>>::push_back(loop_constraint);

            span_pos_cnt_ += loop_constraint->numSpanningPos();

            G_ = DMat<Scalar>::Zero(G_.rows() + loop_constraint->G().rows(),
                                    G_.cols() + loop_constraint->G().cols());
            g_ = DVec<Scalar>::Zero(G_.rows());

            K_ = DMat<Scalar>::Zero(K_.rows() + loop_constraint->K().rows(),
                                    K_.cols() + loop_constraint->K().cols());
            k_ = DVec<Scalar>::Zero(K_.rows());
        }

        template <typename Scalar>
        void Collection<Scalar>::update(DVec<Scalar> q)
        {
            int pos_cnt = 0;
            int span_vel_cnt = 0;
            int ind_vel_cnt = 0;
            int cnstr_cnt = 0;

            for (auto constraint : *this)
            {
                const int n_span_pos = constraint->numSpanningPos();
                const int n_span_vel = constraint->numSpanningVel();
                const int n_ind_vel = constraint->numIndependentVel();
                const int n_cnstr = constraint->numConstraints();

                JointCoordinate<Scalar> position(q.segment(pos_cnt, n_span_pos), true);
                constraint->updateJacobians(position);

                G_.block(span_vel_cnt, ind_vel_cnt, n_span_vel, n_ind_vel) = constraint->G();
                K_.block(cnstr_cnt, span_vel_cnt, n_cnstr, n_span_vel) = constraint->K();

                pos_cnt += n_span_pos;
                span_vel_cnt += n_span_vel;
                ind_vel_cnt += n_ind_vel;
                cnstr_cnt += n_cnstr;
            }

            resetCache();
        }

        template <typename Scalar>
        void Collection<Scalar>::update(DVec<Scalar> q, DVec<Scalar> qd)
        {
            int pos_cnt = 0;
            int span_vel_cnt = 0;
            int ind_vel_cnt = 0;
            int cnstr_cnt = 0;

            for (auto constraint : *this)
            {
                const int n_span_pos = constraint->numSpanningPos();
                const int n_span_vel = constraint->numSpanningVel();
                const int n_ind_vel = constraint->numIndependentVel();
                const int n_cnstr = constraint->numConstraints();

                JointCoordinate<Scalar> position(q.segment(pos_cnt, n_span_pos), true);
                JointCoordinate<Scalar> velocity(qd.segment(span_vel_cnt, n_span_vel), true);
                JointState<Scalar> state(position, velocity);
                constraint->updateJacobians(position);
                constraint->updateBiases(state);

                G_.block(span_vel_cnt, ind_vel_cnt, n_span_vel, n_ind_vel) = constraint->G();
                g_.segment(span_vel_cnt, n_span_vel) = constraint->g();

                K_.block(cnstr_cnt, span_vel_cnt, n_cnstr, n_span_vel) = constraint->K();
                k_.segment(cnstr_cnt, n_cnstr) = constraint->k();

                pos_cnt += n_span_pos;
                span_vel_cnt += n_span_vel;
                ind_vel_cnt += n_ind_vel;
                cnstr_cnt += n_cnstr;
            }

            resetCache();
        }

        template <typename Scalar>
        void Collection<Scalar>::resetCache()
        {
            G_transpose_computed_ = false;
            G_pinv_computed_ = false;
            G_tranpose_pinv_computed_ = false;
            K_transpose_computed_ = false;
        }

        template struct Collection<double>;
        template struct Collection<float>;
        template struct Collection<casadi::SX>;

    }

}
