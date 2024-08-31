#ifndef GRBDA_LOOP_CONSTRAINT_H
#define GRBDA_LOOP_CONSTRAINT_H

#include <memory>
#include "grbda/Utils/Utilities.h"
#include "grbda/Utils/StateRepresentation.h"

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

            bool isExplicit() const { return phi_ == nullptr; }
            bool isValidSpanningPosition(const JointCoordinate<Scalar> &joint_pos) const;
            bool isValidSpanningVelocity(const JointCoordinate<Scalar> &joint_vel) const;

            virtual void updateJacobians(const JointCoordinate<Scalar> &joint_pos) = 0;
            virtual void updateBiases(const JointState<Scalar> &joint_state) = 0;

            DVec<Scalar> phi(const JointCoordinate<Scalar> &joint_pos) const { return phi_(joint_pos); }
            virtual DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const = 0;
            const DMat<Scalar> &G() const { return G_; }
            const DVec<Scalar> &g() const { return g_; }

            const DMat<Scalar> &K() const { return K_; }
            const DVec<Scalar> &k() const { return k_; }

            // TODO(@MatthewChignoli): I actually think we want to keep this, but as a private member
            std::vector<bool> is_coordinate_independent_;

            // TODO(@MatthewChignoli): This is a mess
            virtual void createRandomStateHelpers()
            {
                throw std::runtime_error("Random state helpers not setup for this loop constraint type");
            }
            struct
            {
                bool created = false;
                casadi::Function phi_root_finder;
                casadi::Function G;
            } random_state_helpers_;

        protected:
            std::function<DVec<Scalar>(const JointCoordinate<Scalar> &)> phi_;

            DMat<Scalar> G_;
            DVec<Scalar> g_;

            DMat<Scalar> K_;
            DVec<Scalar> k_;
        };

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

            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const override;
        };

        // TODO(@MatthewChignoli): Move this back to four bar file?
        template <typename Scalar = double>
        struct FourBar : Base<Scalar>
        {
            typedef typename CorrectMatrixInverseType<Scalar>::type InverseType;

            FourBar(std::vector<Scalar> path1_link_lengths, std::vector<Scalar> path2_link_lengths,
                    Vec2<Scalar> offset, int independent_coordinate);

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<FourBar<Scalar>>(*this);
            }

            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const override
            {
                throw std::runtime_error("FourBar: Explicit constraint does not exist");
            }

            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override;
            void updateBiases(const JointState<Scalar> &joint_state) override;

            void createRandomStateHelpers() override;

            const int& independent_coordinate() const { return independent_coordinate_; }

        private:
            void updateImplicitJacobian(const JointCoordinate<Scalar> &joint_pos);
            void updateExplicitJacobian(const DMat<Scalar> &K);

            const size_t links_in_path1_;
            const size_t links_in_path2_;
            const std::vector<Scalar> path1_link_lengths_;
            const std::vector<Scalar> path2_link_lengths_;
            const Vec2<Scalar> offset_;

            const int independent_coordinate_;
            // The independent coordinate map is a 3x3 matrix that maps the stacked indepedent
            // coordinates [y;q_dep] to the spanning coordinate vector [q1;q2;q3]
            Mat3<Scalar> indepenent_coordinate_map_;

            InverseType Kd_inv_;
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
