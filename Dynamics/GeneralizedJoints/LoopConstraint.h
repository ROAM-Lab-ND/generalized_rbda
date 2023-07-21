#pragma once

// #include <memory>
// #include "Dynamics/Body.h"
// #include "Dynamics/Joints/Joint.h"
// #include "Utils/Utilities/SpatialTransforms.h"
#include "Utils/cppTypes.h"
#include <iostream>

// TODO(@MatthewChignoli): The constraints are the abstract class. Then we can have stuff inherit from them.

// TODO(@MatthewChignoli): And then we need an explicit constraint collection class (just a std::Vector)

// TODO(@MatthewChignoli): Should probably make a namespace...

namespace grbda
{

    namespace LoopConstraint
    {

        struct Base
        {
            virtual ~Base() {}

            virtual std::shared_ptr<Base> clone() const = 0;

            virtual int numSpanningPos() const { return G_.rows(); }
            virtual int numIndependentPos() const { return G_.cols(); }
            int numSpanningVel() const { return G_.rows(); }
            int numIndependentVel() const { return G_.cols(); }
            int numConstraints() const { return K_.rows(); }

            virtual void updateJacobians(const JointCoordinate &joint_pos) = 0;
            virtual void updateBiases(const JointState &joint_state) = 0;

            virtual DVec<double> gamma(const JointCoordinate &joint_pos) const = 0;
            const DMat<double> &G() const { return G_; }
            const DVec<double> &g() const { return g_; }

            const DMat<double> &K() const { return K_; }
            const DVec<double> &k() const { return k_; }

        protected:
            DMat<double> G_;
            DVec<double> g_;

            DMat<double> K_;
            DVec<double> k_;
        };

        struct Static : Base
        {
            Static(DMat<double> G, DMat<double> K)
            {
                G_ = G;
                g_ = DVec<double>::Zero(G.rows());

                K_ = K;
                k_ = DVec<double>::Zero(K.rows());
            }

            virtual std::shared_ptr<Base> clone() const
            {
                return std::make_shared<Static>(*this);
            }

            void updateJacobians(const JointCoordinate &joint_pos) override {}
            void updateBiases(const JointState &joint_state) override {}

            DVec<double> gamma(const JointCoordinate &joint_pos) const override
            {
                return G_ * joint_pos;
            }
        };

        // TODO(@MatthewChignoli): I think a more efficient way would be to bind references of G to blocks of the big G collection matrix, but let's worry about that later
        struct Collection : std::vector<std::shared_ptr<Base>>
        {
            DVec<double> gamma(const DVec<double> y) const
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

            const DMat<double> &G() const { return G_; }
            const DVec<double> &g() const { return g_; }

            const DMat<double> &K() const { return K_; }
            const DVec<double> &k() const { return k_; }

            const DMat<double> &G_transpose() const
            {
                if (!G_transpose_computed_)
                {
                    G_transpose_ = G_.transpose();
                    G_transpose_computed_ = true;
                }
                return G_transpose_;
            }

            const DMat<double> &G_pinv() const
            {
                if (!G_pinv_computed_)
                {
                    G_pinv_ = G_.completeOrthogonalDecomposition().pseudoInverse();
                    G_pinv_computed_ = true;
                }

                return G_pinv_;
            }
            const DMat<double> &G_tranpose_pinv() const
            {
                if (!G_tranpose_pinv_computed_)
                {
                    G_tranpose_pinv_ = G_transpose().completeOrthogonalDecomposition().pseudoInverse();
                    G_tranpose_pinv_computed_ = true;
                }
                return G_tranpose_pinv_;
            }

            const DMat<double> &K_transpose() const
            {
                if (!K_transpose_computed_)
                {
                    K_transpose_ = K_.transpose();
                    K_transpose_computed_ = true;
                }
                return K_transpose_;
            }

            void push_back(const std::shared_ptr<Base> loop_constraint)
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

            void update(DVec<double> q, DVec<double> qd)
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

                G_transpose_computed_ = false;
                G_pinv_computed_ = false;
                G_tranpose_pinv_computed_ = false;
                K_transpose_computed_ = false;

            }

        private:
            int span_pos_cnt_ = 0;
            DMat<double> G_ = DMat<double>::Zero(0, 0);
            DVec<double> g_ = DVec<double>::Zero(0);

            DMat<double> K_ = DMat<double>::Zero(0, 0);
            DVec<double> k_ = DVec<double>::Zero(0);

            mutable bool G_transpose_computed_ = false;
            mutable DMat<double> G_transpose_;
            mutable bool G_pinv_computed_ = false;
            mutable DMat<double> G_pinv_;
            mutable bool G_tranpose_pinv_computed_ = false;
            mutable DMat<double> G_tranpose_pinv_;
            mutable bool K_transpose_computed_ = false;
            mutable DMat<double> K_transpose_;
        };
    }

}
