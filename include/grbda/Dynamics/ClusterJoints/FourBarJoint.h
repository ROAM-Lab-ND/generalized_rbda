#ifndef GRBDA_FOUR_BAR_JOINT_H
#define GRBDA_FOUR_BAR_JOINT_H

#include "grbda/Utils/Utilities.h"
#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"

namespace grbda
{

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

            JointState<double> randomJointState() const override;

        private:
            std::shared_ptr<LoopConstraint::FourBar<Scalar>> four_bar_constraint_;
        };

    } // namespace ClusterJoints

} // namespace grbda

#endif // GRBDA_FOUR_BAR_JOINT_H
