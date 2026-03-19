#ifndef GRBDA_ROBOTS_TWO_LINK_CHAIN_H
#define GRBDA_ROBOTS_TWO_LINK_CHAIN_H

#include "grbda/Robots/Robot.h"
#include "grbda/Dynamics/ClusterTreeModel.h"

namespace grbda
{

    /**
     * @brief Templated Two-Link Chain robot class for complex-step differentiation
     *
     * This is a 2-DOF fixed-base serial chain with two revolute joints.
     * Unlike DoublePendulum, this has CoM offset from joint origins to ensure
     * non-degenerate dynamics (non-zero derivative matrices).
     */
    template <typename Scalar = double>
    class TwoLinkChain : public Robot<Scalar>
    {
    public:
        TwoLinkChain() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override
        {
            ClusterTreeModel<Scalar> model;
            using Revolute = ClusterJoints::Revolute<Scalar>;

            Mat3<Scalar> I3 = Mat3<Scalar>::Identity();

            // Link 1 inertial parameters
            // mass = 2.0, CoM at [0.15, 0, 0] (offset from joint)
            Mat3<Scalar> I1;
            I1 << Scalar(0.05), Scalar(0), Scalar(0),
                  Scalar(0), Scalar(0.03), Scalar(0),
                  Scalar(0), Scalar(0), Scalar(0.01);
            Vec3<Scalar> com1(Scalar(0.15), Scalar(0), Scalar(0));  // CoM offset along link
            SpatialInertia<Scalar> link1_inertia(Scalar(2.0), com1, I1);

            // Link 2 inertial parameters
            // mass = 1.5, CoM at [0.1, 0, 0] (offset from joint)
            Mat3<Scalar> I2;
            I2 << Scalar(0.02), Scalar(0), Scalar(0),
                  Scalar(0), Scalar(0.015), Scalar(0),
                  Scalar(0), Scalar(0), Scalar(0.01);
            Vec3<Scalar> com2(Scalar(0.1), Scalar(0), Scalar(0));  // CoM offset along link
            SpatialInertia<Scalar> link2_inertia(Scalar(1.5), com2, I2);

            // Joint 1: Revolute about Z-axis, attached to ground at origin
            spatial::Transform<Scalar> Xtree1(I3, Vec3<Scalar>::Zero());

            model.template appendBody<Revolute>(
                "link1", link1_inertia, "ground", Xtree1,
                ori::CoordinateAxis::Z, "joint1");

            // Joint 2: Revolute about Z-axis, offset from link1 by [0.3, 0, 0]
            Vec3<Scalar> r2(Scalar(0.3), Scalar(0), Scalar(0));
            spatial::Transform<Scalar> Xtree2(I3, r2);

            model.template appendBody<Revolute>(
                "link2", link2_inertia, "link1", Xtree2,
                ori::CoordinateAxis::Z, "joint2");

            return model;
        }
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_TWO_LINK_CHAIN_H
