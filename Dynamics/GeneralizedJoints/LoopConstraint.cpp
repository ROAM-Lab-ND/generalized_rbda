#include "LoopConstraint.h"

namespace grbda
{

    namespace LoopConstraint
    {

        Static::Static(DMat<double> G, DMat<double> K)
        {
            G_ = G;
            g_ = DVec<double>::Zero(G.rows());

            K_ = K;
            k_ = DVec<double>::Zero(K.rows());
        }

        DVec<double> Static::gamma(const JointCoordinate &joint_pos) const
        {
            return G_ * joint_pos;
        }

        DVec<double> Collection::gamma(const DVec<double> y) const
        {
            DVec<double> spanning_pos(span_pos_cnt_);

            int span_pos_cnt = 0;
            int ind_pos_cnt = 0;
            for (auto constraint : *this)
            {
                const int n_span_pos = constraint->numSpanningPos();
                const int n_ind_pos = constraint->numIndependentPos();

                JointCoordinate position(y.segment(ind_pos_cnt, n_ind_pos), false);
                spanning_pos.segment(span_pos_cnt, n_span_pos) = constraint->gamma(position);

                span_pos_cnt += n_span_pos;
                ind_pos_cnt += n_ind_pos;
            }
            return spanning_pos;
        }

        const DMat<double> &Collection::G_transpose() const
        {
            if (!G_transpose_computed_)
            {
                G_transpose_ = G_.transpose();
                G_transpose_computed_ = true;
            }
            return G_transpose_;
        }

        const DMat<double> &Collection::G_pinv() const
        {
            if (!G_pinv_computed_)
            {
                G_pinv_ = G_.completeOrthogonalDecomposition().pseudoInverse();
                G_pinv_computed_ = true;
            }

            return G_pinv_;
        }
        const DMat<double> &Collection::G_tranpose_pinv() const
        {
            if (!G_tranpose_pinv_computed_)
            {
                G_tranpose_pinv_ = G_transpose().completeOrthogonalDecomposition().pseudoInverse();
                G_tranpose_pinv_computed_ = true;
            }
            return G_tranpose_pinv_;
        }

        const DMat<double> &Collection::K_transpose() const
        {
            if (!K_transpose_computed_)
            {
                K_transpose_ = K_.transpose();
                K_transpose_computed_ = true;
            }
            return K_transpose_;
        }

        void Collection::push_back(const std::shared_ptr<Base> loop_constraint)
        {
            std::vector<std::shared_ptr<Base>>::push_back(loop_constraint);

            span_pos_cnt_ += loop_constraint->numSpanningPos();

            G_ = DMat<double>::Zero(G_.rows() + loop_constraint->G().rows(),
                                    G_.cols() + loop_constraint->G().cols());
            g_ = DVec<double>::Zero(G_.rows());

            K_ = DMat<double>::Zero(K_.rows() + loop_constraint->K().rows(),
                                    K_.cols() + loop_constraint->K().cols());
            k_ = DVec<double>::Zero(K_.rows());
        }

        void Collection::update(DVec<double> q, DVec<double> qd)
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

                JointCoordinate position(q.segment(pos_cnt, n_span_pos), true);
                JointCoordinate velocity(qd.segment(span_vel_cnt, n_span_vel), true);
                JointState state(position, velocity);
                constraint->updateConstraintFromJointPos(position);
                constraint->updateConstraintFromJointState(state);

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

        void Collection::resetCache()
        {
            G_transpose_computed_ = false;
            G_pinv_computed_ = false;
            G_tranpose_pinv_computed_ = false;
            K_transpose_computed_ = false;
        }

    }

}
