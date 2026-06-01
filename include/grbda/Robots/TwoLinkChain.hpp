#ifndef GRBDA_ROBOTS_TWO_LINK_CHAIN_H
#define GRBDA_ROBOTS_TWO_LINK_CHAIN_H

#include "grbda/Robots/Robot.h"
#include "grbda/Dynamics/ClusterTreeModel.h"

namespace grbda
{

    class TwoLinkChain : public Robot<>
    {
    public:
        TwoLinkChain() {}

        ClusterTreeModel<> buildClusterTreeModel() const override
        {
            ClusterTreeModel<> model;
            using Revolute = ClusterJoints::Revolute<>;

            Mat3<> I3 = Mat3<>::Identity();

            Mat3<> I1;
            I1 << 0.05, 0., 0.,
                  0., 0.03, 0.,
                  0., 0., 0.01;
            Vec3<> com1(0.15, 0., 0.);
            SpatialInertia<> link1_inertia(2.0, com1, I1);

            Mat3<> I2;
            I2 << 0.02, 0., 0.,
                  0., 0.015, 0.,
                  0., 0., 0.01;
            Vec3<> com2(0.1, 0., 0.);
            SpatialInertia<> link2_inertia(1.5, com2, I2);

            spatial::Transform<> Xtree1(I3, Vec3<>::Zero());
            model.template appendBody<Revolute>(
                "link1", link1_inertia, "ground", Xtree1,
                ori::CoordinateAxis::Z, "joint1");

            Vec3<> r2(0.3, 0., 0.);
            spatial::Transform<> Xtree2(I3, r2);
            model.template appendBody<Revolute>(
                "link2", link2_inertia, "link1", Xtree2,
                ori::CoordinateAxis::Z, "joint2");

            return model;
        }
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_TWO_LINK_CHAIN_H
