#include "LoopConstraint.h"

#include "Utils/math.h"
#include <casadi/casadi.hpp>

namespace grbda
{

    namespace LoopConstraint
    {

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

        template class Static<double>;
        template class Static<casadi::SX>;

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
                const DMat<Scalar> GT_G = G_.transpose() * G_;
                G_pinv_ = math::matrixInverse(GT_G) * G_.transpose();
                G_pinv_computed_ = true;
            }

            return G_pinv_;
        }

        template <typename Scalar>
        const DMat<Scalar> &Collection<Scalar>::G_tranpose_pinv() const
        {
            if (!G_tranpose_pinv_computed_)
            {
                const DMat<Scalar> GT_G = G_.transpose() * G_;
                G_tranpose_pinv_ = G_ * math::matrixInverse(GT_G);
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

        template class Collection<double>;
        template class Collection<casadi::SX>;

    }

}
