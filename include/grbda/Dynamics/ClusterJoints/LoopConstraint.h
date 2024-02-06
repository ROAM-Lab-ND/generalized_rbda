#ifndef GRBDA_LOOP_CONSTRAINT_H
#define GRBDA_LOOP_CONSTRAINT_H

#include <memory>
#include "grbda/Utils/StateRepresentation.h"
#include "grbda/Utils/Utilities.h"

namespace grbda
{

    namespace LoopConstraint
    {

        template <typename Scalar = double>
        struct Base
        {
            virtual ~Base() {}

            virtual std::shared_ptr<Base<Scalar>> clone() const = 0;

            virtual int numSpanningPos() const { return G_.rows(); }
            virtual int numIndependentPos() const { return G_.cols(); }
            int numSpanningVel() const { return G_.rows(); }
            int numIndependentVel() const { return G_.cols(); }
            int numConstraints() const { return K_.rows(); }

            bool isValidSpanningPosition(const JointCoordinate<Scalar> &joint_pos) const;
            bool isValidSpanningVelocity(const JointCoordinate<Scalar> &joint_vel) const;

            virtual void updateJacobians(const JointCoordinate<Scalar> &joint_pos) = 0;
            virtual void updateBiases(const JointState<Scalar> &joint_state) = 0;
            
            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const
            {
                return gamma_(joint_pos);
            }
            const DMat<Scalar> &G() const { return G_; }
            const DVec<Scalar> &g() const { return g_; }

            DVec<Scalar> phi(const JointCoordinate<Scalar> &joint_pos) const
            {
                return phi_(joint_pos);
            }
            const DMat<Scalar> &K() const { return K_; }
            const DVec<Scalar> &k() const { return k_; }

        protected:
            std::function<DVec<Scalar>(const JointCoordinate<Scalar> &)> gamma_;
            DMat<Scalar> G_;
            DVec<Scalar> g_;

            std::function<DVec<Scalar>(const JointCoordinate<Scalar> &)> phi_;
            DMat<Scalar> K_;
            DVec<Scalar> k_;
        };

        // TODO(@MatthewChignoli): Should we have a middle layer of abstraction for implicit vs explicit constraints? The independent coordinate mapping is one example of something that could be abstracted for all implicit constraints.

        template <typename Scalar = double>
        struct Static : Base<Scalar>
        {
            Static(DMat<Scalar> G, DMat<Scalar> K);

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<Static<Scalar>>(*this);
            }

            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override {}
            void updateBiases(const JointState<Scalar> &joint_state) override {}
        };

        template <typename Scalar = double>
        struct Collection : std::vector<std::shared_ptr<Base<Scalar>>>
        {
            DVec<Scalar> gamma(const DVec<Scalar> y) const;

            const DMat<Scalar> &G() const { return G_; }
            const DVec<Scalar> &g() const { return g_; }

            const DMat<Scalar> &K() const { return K_; }
            const DVec<Scalar> &k() const { return k_; }

            const DMat<Scalar> &G_transpose() const;
            const DMat<Scalar> &G_pinv() const;
            const DMat<Scalar> &G_tranpose_pinv() const;
            const DMat<Scalar> &K_transpose() const;

            void push_back(const std::shared_ptr<Base<Scalar>> loop_constraint);

            void update(DVec<Scalar> q);
            void update(DVec<Scalar> q, DVec<Scalar> qd);

        private:
            void resetCache();

            int span_pos_cnt_ = 0;
            DMat<Scalar> G_ = DMat<Scalar>::Zero(0, 0);
            DVec<Scalar> g_ = DVec<Scalar>::Zero(0);

            DMat<Scalar> K_ = DMat<Scalar>::Zero(0, 0);
            DVec<Scalar> k_ = DVec<Scalar>::Zero(0);

            mutable bool G_transpose_computed_ = false;
            mutable DMat<Scalar> G_transpose_;
            mutable bool G_pinv_computed_ = false;
            mutable DMat<Scalar> G_pinv_;
            mutable bool G_tranpose_pinv_computed_ = false;
            mutable DMat<Scalar> G_tranpose_pinv_;
            mutable bool K_transpose_computed_ = false;
            mutable DMat<Scalar> K_transpose_;
        };
    }

}

#endif // GRBDA_LOOP_CONSTRAINT_H
