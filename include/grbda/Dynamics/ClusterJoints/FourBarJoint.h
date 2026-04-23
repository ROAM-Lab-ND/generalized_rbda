#ifndef GRBDA_FOUR_BAR_JOINT_H
#define GRBDA_FOUR_BAR_JOINT_H

#include "grbda/Utils/Utilities.h"
#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"

namespace grbda
{

    namespace LoopConstraint
    {
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

            // Override to use tighter tolerance - FourBar phi uses standard C++ trig functions
            // which work correctly with complex types and can achieve machine precision
            bool isValidSpanningPosition(const JointCoordinate<Scalar> &joint_pos) const;

            void createRandomStateHelpers() override;

            const int& independent_coordinate() const { return independent_coordinate_; }

            // Accessors for computing dG/dq analytically
            const std::vector<Scalar>& path1LinkLengths() const { return path1_link_lengths_; }
            const std::vector<Scalar>& path2LinkLengths() const { return path2_link_lengths_; }
            size_t linksInPath1() const { return links_in_path1_; }
            size_t linksInPath2() const { return links_in_path2_; }
            const Mat3<Scalar>& independentCoordinateMap() const { return indepenent_coordinate_map_; }
            const InverseType& KdInverse() const { return Kd_inv_; }

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

        template <typename Scalar = double>
        class FourBar : public Generic<Scalar>
        {
        public:
            FourBar(const std::vector<Body<Scalar>> &bodies,
                    const std::vector<JointPtr<Scalar>> &joints,
                    std::shared_ptr<LoopConstraint::FourBar<Scalar>> loop_constraint)
                : Generic<Scalar>(bodies, joints, loop_constraint),
                  four_bar_constraint_(loop_constraint) {}

            virtual ~FourBar() {}

            ClusterJointTypes type() const override { return ClusterJointTypes::FourBar; }

            JointState<double> randomJointState(bool enforce_position_constraint = true) const override;

            // Override getSq to compute dS/dq analytically for FourBar
            // This is required for correct analytical derivative computation
            std::vector<DMat<Scalar>> getSq() const override;

        private:
            std::shared_ptr<LoopConstraint::FourBar<Scalar>> four_bar_constraint_;
        };

    } // namespace ClusterJoints

} // namespace grbda

#endif // GRBDA_FOUR_BAR_JOINT_H
