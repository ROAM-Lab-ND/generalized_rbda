#ifndef GRBDA_FOUR_BAR_JOINT_H
#define GRBDA_FOUR_BAR_JOINT_H

#include "grbda/Utils/Utilities.h"
#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"

namespace grbda
{

    namespace LoopConstraint
    {
        // TODO(@MatthewChignoli): Later we can worry about how we handle the fixed offset with this loop constraint compared to the generic one
        // TODO(@MatthewChignoli): Missing the fixed offset between the joints of link 1 and link 3
        template <typename Scalar = double>
        struct FourBar : Base<Scalar>
        {
            typedef typename CorrectMatrixInverseType<Scalar>::type InverseType;
            typedef typename CorrectMatrixFullPivLuType<Scalar>::type FullPivLuType;

            // TODO(@MatthewChignoli): Pass by reference?
            FourBar(std::vector<Scalar> path1_lengths, std::vector<Scalar> path2_lengths,
                    Vec2<Scalar> offset);

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<FourBar<Scalar>>(*this);
            }

            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override;

            void updateBiases(const JointState<Scalar> &joint_state) override;

            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const override { return DVec<Scalar>::Zero(0); }

        private:
            void updateImplicitJacobian(const JointCoordinate<Scalar> &joint_pos);
            void updateExplicitJacobian(const DMat<Scalar> &K);

            // TODO(@MatthewChignoli): The naming here is pretty confusing
            const size_t path1_size_;
            const size_t path2_size_;
            const std::vector<Scalar> path1_lengths_;
            const std::vector<Scalar> path2_lengths_;

            // TODO(@MatthewChignoli): Perhaps I should just make a factorization info struct
            InverseType K11inv_;
            // Note: Q2T_ is the transpose of Q2 but is of type InverseType and Q1inv_ is the
            // inverse of Q1 but is of type DMat<Scalar> because Eigen treats LU factorization 
            // in the form K = P^{-1}*L*U*Q^{-1} while Featherstone (our reference) treats it in 
            // the form K = Q1*L*U*Q2
            InverseType Q2T_;
            DMat<Scalar> Q1inv_;
        };
    }

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class FourBar : public Base<Scalar>
        {
        public:
            // TODO(@MatthewChignoli): Figure out later exactly what we need
            FourBar(Body<Scalar> &link_1, Body<Scalar> &link_2,
                    ori::CoordinateAxis joint_axis_1, ori::CoordinateAxis joint_axis_2) {}

            virtual ~FourBar() {}

            void updateKinematics(const JointState<Scalar> &joint_state) override {}
        };

    } // namespace ClusterJoints

} // namespace grbda

#endif // GRBDA_FOUR_BAR_JOINT_H
