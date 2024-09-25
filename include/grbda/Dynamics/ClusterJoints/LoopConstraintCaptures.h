#ifndef GRBDA_LOOP_CONSTRAINT_CAPTURE_H
#define GRBDA_LOOP_CONSTRAINT_CAPTURE_H

#include <vector>
#include "grbda/Dynamics/Body.h"

namespace grbda
{

    // TODO(@MatthewChignoli): Rename these
    struct ConstraintCapture
    {
        std::vector<Body<casadi::SX>> nca_to_predecessor_subchain;
        std::vector<Body<casadi::SX>> nca_to_successor_subchain;
    };

    struct PositionConstraintCapture : public ConstraintCapture
    {
        urdf::Pose predecessor_to_joint_origin_transform;
        urdf::Pose successor_to_joint_origin_transform;
    };

    struct RollingConstraintCapture : public ConstraintCapture
    {
        double ratio;
    };

}

#endif // GRBDA_LOOP_CONSTRAINT_CAPTURE_H
