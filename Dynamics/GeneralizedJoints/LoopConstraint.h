#ifndef GRBDA_LOOP_CONSTRAINT_H
#define GRBDA_LOOP_CONSTRAINT_H

#include <memory>
#include "Utils/cppTypes.h"

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
            Static(DMat<double> G, DMat<double> K);

            std::shared_ptr<Base> clone() const override { return std::make_shared<Static>(*this); }

            void updateJacobians(const JointCoordinate &joint_pos) override {}
            void updateBiases(const JointState &joint_state) override {}

            DVec<double> gamma(const JointCoordinate &joint_pos) const override;
        };

        struct Collection : std::vector<std::shared_ptr<Base>>
        {
            DVec<double> gamma(const DVec<double> y) const;

            const DMat<double> &G() const { return G_; }
            const DVec<double> &g() const { return g_; }

            const DMat<double> &K() const { return K_; }
            const DVec<double> &k() const { return k_; }

            const DMat<double> &G_transpose() const;
            const DMat<double> &G_pinv() const;
            const DMat<double> &G_tranpose_pinv() const;
            const DMat<double> &K_transpose() const;

            void push_back(const std::shared_ptr<Base> loop_constraint);

            void update(DVec<double> q);
            void update(DVec<double> q, DVec<double> qd);

        private:
            void resetCache();

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

#endif // GRBDA_LOOP_CONSTRAINT_H
