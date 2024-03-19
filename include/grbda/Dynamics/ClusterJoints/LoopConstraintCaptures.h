#ifndef GRBDA_LOOP_CONSTRAINT_CAPTURE_H
#define GRBDA_LOOP_CONSTRAINT_CAPTURE_H

#include <vector>
#include "grbda/Dynamics/Body.h"

namespace grbda
{

    struct ConstraintCapture
    {
        std::vector<Body<casadi::SX>> nca_to_predecessor_subtree;
        std::vector<Body<casadi::SX>> nca_to_successor_subtree;
    };

    struct PositionConstraintCapture : public ConstraintCapture
    {
        urdf::Pose predecessor_to_constraint_origin_transform;
        urdf::Pose successor_to_constraint_origin_transform;
    };

    struct RollingConstraintCapture : public ConstraintCapture
    {
        double ratio;
    };

}

#endif // GRBDA_LOOP_CONSTRAINT_CAPTURE_H
