#pragma once

#include "GeneralizedJoint.h"

namespace GeneralizedJoints
{

class Revolute : public Base
{
public:
    Revolute(const Body &body, CoordinateAxis joint_axis);
    virtual ~Revolute() {}

    GeneralizedJointTypes type() const override { return GeneralizedJointTypes::Revolute; }

    void updateKinematics(const DVec<double> &y, const DVec<double> &yd) override;

    void computeSpatialTransformFromParentToCurrentCluster(
        GeneralizedSpatialTransform &Xup) const override;

    std::vector<std::tuple<Body, JointPtr, DMat<double>>>
    bodiesJointsAndReflectedInertias() const override;

private:
    const Body body_;
};

}
