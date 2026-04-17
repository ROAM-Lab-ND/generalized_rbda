#ifndef GRBDA_TELLO_DIFFERENTIAL_JOINTS_H
#define GRBDA_TELLO_DIFFERENTIAL_JOINTS_H

#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"

namespace grbda
{

namespace ClusterJoints
{

template <typename Scalar = double>
class TelloHipDifferential : public Generic<Scalar>
{
public:
    TelloHipDifferential(const std::vector<Body<Scalar>> &bodies,
                         const std::vector<JointPtr<Scalar>> &joints,
                         std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint)
        : Generic<Scalar>(bodies, joints, loop_constraint)
    {
    }

    ClusterJointTypes type() const override { return ClusterJointTypes::TelloHipDifferential; }
};

template <typename Scalar = double>
class TelloKneeAnkleDifferential : public Generic<Scalar>
{
public:
    TelloKneeAnkleDifferential(const std::vector<Body<Scalar>> &bodies,
                               const std::vector<JointPtr<Scalar>> &joints,
                               std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint)
        : Generic<Scalar>(bodies, joints, loop_constraint)
    {
    }

    ClusterJointTypes type() const override { return ClusterJointTypes::TelloKneeAnkleDifferential; }
};

} // namespace ClusterJoints

} // namespace grbda

#endif // GRBDA_TELLO_DIFFERENTIAL_JOINTS_H
