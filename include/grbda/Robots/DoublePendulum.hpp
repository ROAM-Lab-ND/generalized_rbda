#ifndef GRBDA_ROBOTS_DOUBLE_PENDULUM_H
#define GRBDA_ROBOTS_DOUBLE_PENDULUM_H

#include "grbda/Robots/Robot.h"
#include "grbda/Dynamics/ClusterTreeModel.h"

namespace grbda
{

    /**
     * @brief Templated Double Pendulum robot class for complex-step differentiation
     *
     * This is a 2-DOF fixed-base serial chain with two revolute joints.
     * The robot matches the structure in double_pendulum.urdf but is templated
     * to support complex numbers for complex-step derivative verification.
     */
    template <typename Scalar = double>
    class DoublePendulum : public Robot<Scalar>
    {
    public:
        DoublePendulum() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override
        {
            ClusterTreeModel<Scalar> model;
            using Revolute = ClusterJoints::Revolute<Scalar>;

            Mat3<Scalar> I3 = Mat3<Scalar>::Identity();

            // Link 1 inertial parameters (from URDF)
            // mass = 2.0, inertia = [0.05, 0.03, 0.01]
            Mat3<Scalar> I1;
            I1 << Scalar(0.05), Scalar(0), Scalar(0),
                  Scalar(0), Scalar(0.03), Scalar(0),
                  Scalar(0), Scalar(0), Scalar(0.01);
            Vec3<Scalar> com1(Scalar(0), Scalar(0), Scalar(0));
            SpatialInertia<Scalar> link1_inertia(Scalar(2.0), com1, I1);

            // Link 2 inertial parameters (from URDF)
            // mass = 1.0, inertia = [0.01, 0.005, 0.2]
            Mat3<Scalar> I2;
            I2 << Scalar(0.01), Scalar(0), Scalar(0),
                  Scalar(0), Scalar(0.005), Scalar(0),
                  Scalar(0), Scalar(0), Scalar(0.2);
            Vec3<Scalar> com2(Scalar(0), Scalar(0), Scalar(0));
            SpatialInertia<Scalar> link2_inertia(Scalar(1.0), com2, I2);

            // Joint 1: Revolute about Z-axis, attached to ground at origin
            spatial::Transform<Scalar> Xtree1(I3, Vec3<Scalar>::Zero());

            model.template appendBody<Revolute>(
                "link1", link1_inertia, "ground", Xtree1,
                ori::CoordinateAxis::Z, "joint1");

            // Joint 2: Revolute about Z-axis, offset from link1 by [0.1, 0, 0]
            Vec3<Scalar> r2(Scalar(0.1), Scalar(0), Scalar(0));
            spatial::Transform<Scalar> Xtree2(I3, r2);

            model.template appendBody<Revolute>(
                "link2", link2_inertia, "link1", Xtree2,
                ori::CoordinateAxis::Z, "joint2");

            return model;
        }
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_DOUBLE_PENDULUM_H
