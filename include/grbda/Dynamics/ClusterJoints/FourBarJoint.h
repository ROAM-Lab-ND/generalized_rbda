#ifndef GRBDA_FOUR_BAR_JOINT_H
#define GRBDA_FOUR_BAR_JOINT_H

#include "grbda/Utils/Utilities.h"
// #include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"
#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"

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

            FourBar(std::vector<Scalar> path1_link_lengths, std::vector<Scalar> path2_link_lengths,
                    Vec2<Scalar> offset, int independent_coordinate);

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<FourBar<Scalar>>(*this);
            }

            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override;

            void updateBiases(const JointState<Scalar> &joint_state) override;

            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const override
            {
                throw std::runtime_error("FourBar: Explicit constraint does not exist");
            }

            void createRandomStateHelpers();
            struct
            {
                bool created = false;
                casadi::Function phi_root_finder;
                casadi::Function G;
            } random_state_helpers_;

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
    }

    namespace ClusterJoints
    {

        // TODO(@MatthewChignoli): Before specializing, use the generic joint first
        template <typename Scalar = double>
        class FourBar : public Generic<Scalar>
        {
        public:
            // TODO(@MatthewChignoli): For now, assume a certain type of path
            // using LinkJointPair = std::pair<Body<Scalar>, JointPtr<Scalar>>;
            // FourBar(std::vector<LinkJointPair> link_joint_pairs) : Base<Scalar>(3, 1, 1)
            // {
            // }
            // virtual ~FourBar() {}

            FourBar(const std::vector<Body<Scalar>> &bodies,
                    const std::vector<JointPtr<Scalar>> &joints,
                    std::shared_ptr<LoopConstraint::FourBar<Scalar>> loop_constraint)
                : Generic<Scalar>(bodies, joints, loop_constraint),
                  four_bar_constraint_(loop_constraint) {}

            virtual ~FourBar() {}

            ClusterJointTypes type() const override { return ClusterJointTypes::FourBar; }

            // void updateKinematics(const JointState<Scalar> &joint_state) override
            // {
            //     const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            //     const DVec<Scalar> &q = spanning_joint_state.position;
            //     const DVec<Scalar> &qd = spanning_joint_state.velocity;

            //     // Need to compute: S, vJ, cJ
            // }

            JointState<double> randomJointState() const override;

        private:
            std::shared_ptr<LoopConstraint::FourBar<Scalar>> four_bar_constraint_;
        };

    } // namespace ClusterJoints

} // namespace grbda

#endif // GRBDA_FOUR_BAR_JOINT_H
