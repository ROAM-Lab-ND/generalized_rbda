#ifndef GRBDA_LOOP_CONSTRAINT_CAPTURE_H
#define GRBDA_LOOP_CONSTRAINT_CAPTURE_H

#include <vector>
#include "grbda/Dynamics/Body.h"

namespace grbda
{

    struct ConstraintCapture
    {
        std::vector<Body<casadi::SX>> nca_to_parent_subtree;
        std::vector<Body<casadi::SX>> nca_to_child_subtree;
    };

    struct LoopConstraintCapture : public ConstraintCapture
    {
        urdf::Pose parent_to_constraint_origin_transform;
        urdf::Pose child_to_constraint_origin_transform;
    };

    struct JointConstraintCapture : public ConstraintCapture
    {
        double ratio;
    };

}

#endif // GRBDA_LOOP_CONSTRAINT_CAPTURE_H
