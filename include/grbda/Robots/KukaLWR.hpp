#ifndef GRBDA_ROBOTS_KUKA_LWR_H
#define GRBDA_ROBOTS_KUKA_LWR_H

#include "grbda/Robots/Robot.h"
#include "grbda/Dynamics/ClusterTreeModel.h"

namespace grbda
{

    /**
     * @brief Templated KUKA LWR 4+ robot class for complex-step differentiation
     *
     * This is a 7-DOF fixed-base serial chain manipulator.
     * The robot matches the structure in kuka_lwr_4plus.urdf but is templated
     * to support complex numbers for complex-step derivative verification.
     */
    template <typename Scalar = double>
    class KukaLWR : public Robot<Scalar>
    {
    public:
        KukaLWR() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override
        {
            ClusterTreeModel<Scalar> model;
            using Revolute = ClusterJoints::Revolute<Scalar>;

            Mat3<Scalar> I3 = Mat3<Scalar>::Identity();

            // Link inertial parameters from URDF
            // Links 1-5: mass = 2.0
            // Link 6: mass = 1.0
            // Link 7: mass = 0.2

            // Common inertia for links 1-4
            Mat3<Scalar> I_link_1_4;
            I_link_1_4 << Scalar(0.0136666666667), Scalar(0), Scalar(0),
                          Scalar(0), Scalar(0.0118666666667), Scalar(0),
                          Scalar(0), Scalar(0), Scalar(0.003);

            // Inertia for link 5
            Mat3<Scalar> I_link_5;
            I_link_5 << Scalar(0.0126506666667), Scalar(0), Scalar(0),
                        Scalar(0), Scalar(0.0108506666667), Scalar(0),
                        Scalar(0), Scalar(0), Scalar(0.003);

            // Inertia for link 6
            Mat3<Scalar> I_link_6;
            I_link_6 << Scalar(0.00260416666667), Scalar(0), Scalar(0),
                        Scalar(0), Scalar(0.00260416666667), Scalar(0),
                        Scalar(0), Scalar(0), Scalar(0.00260416666667);

            // Inertia for link 7
            Mat3<Scalar> I_link_7;
            I_link_7 << Scalar(6.66666666667e-05), Scalar(0), Scalar(0),
                        Scalar(0), Scalar(6.66666666667e-05), Scalar(0),
                        Scalar(0), Scalar(0), Scalar(0.00012);

            // ================================================================
            // Link 1: Joint about Z, origin at [0, 0, 0.11]
            // ================================================================
            {
                Vec3<Scalar> com1(Scalar(0), Scalar(0), Scalar(0.130));
                SpatialInertia<Scalar> link1_inertia(Scalar(2.0), com1, I_link_1_4);
                Vec3<Scalar> r1(Scalar(0), Scalar(0), Scalar(0.11));
                spatial::Transform<Scalar> Xtree1(I3, r1);

                model.template appendBody<Revolute>(
                    "lwr_arm_1_link", link1_inertia, "ground", Xtree1,
                    ori::CoordinateAxis::Z, "lwr_arm_0_joint");
            }

            // ================================================================
            // Link 2: Joint about Y, origin at [0, 0, 0.20]
            // ================================================================
            {
                Vec3<Scalar> com2(Scalar(0), Scalar(-0.06), Scalar(0.07));
                SpatialInertia<Scalar> link2_inertia(Scalar(2.0), com2, I_link_1_4);
                Vec3<Scalar> r2(Scalar(0), Scalar(0), Scalar(0.20));
                spatial::Transform<Scalar> Xtree2(I3, r2);

                model.template appendBody<Revolute>(
                    "lwr_arm_2_link", link2_inertia, "lwr_arm_1_link", Xtree2,
                    ori::CoordinateAxis::Y, "lwr_arm_1_joint");
            }

            // ================================================================
            // Link 3: Joint about Z, origin at [0, 0, 0.20]
            // ================================================================
            {
                Vec3<Scalar> com3(Scalar(0), Scalar(-0.06), Scalar(0.130));
                SpatialInertia<Scalar> link3_inertia(Scalar(2.0), com3, I_link_1_4);
                Vec3<Scalar> r3(Scalar(0), Scalar(0), Scalar(0.20));
                spatial::Transform<Scalar> Xtree3(I3, r3);

                model.template appendBody<Revolute>(
                    "lwr_arm_3_link", link3_inertia, "lwr_arm_2_link", Xtree3,
                    ori::CoordinateAxis::Z, "lwr_arm_2_joint");
            }

            // ================================================================
            // Link 4: Joint about Y, origin at [0, 0, 0.20]
            // ================================================================
            {
                Vec3<Scalar> com4(Scalar(0), Scalar(0.06), Scalar(0.07));
                SpatialInertia<Scalar> link4_inertia(Scalar(2.0), com4, I_link_1_4);
                Vec3<Scalar> r4(Scalar(0), Scalar(0), Scalar(0.20));
                spatial::Transform<Scalar> Xtree4(I3, r4);

                model.template appendBody<Revolute>(
                    "lwr_arm_4_link", link4_inertia, "lwr_arm_3_link", Xtree4,
                    ori::CoordinateAxis::Y, "lwr_arm_3_joint");
            }

            // ================================================================
            // Link 5: Joint about Z, origin at [0, 0, 0.20]
            // ================================================================
            {
                Vec3<Scalar> com5(Scalar(0), Scalar(0), Scalar(0.124));
                SpatialInertia<Scalar> link5_inertia(Scalar(2.0), com5, I_link_5);
                Vec3<Scalar> r5(Scalar(0), Scalar(0), Scalar(0.20));
                spatial::Transform<Scalar> Xtree5(I3, r5);

                model.template appendBody<Revolute>(
                    "lwr_arm_5_link", link5_inertia, "lwr_arm_4_link", Xtree5,
                    ori::CoordinateAxis::Z, "lwr_arm_4_joint");
            }

            // ================================================================
            // Link 6: Joint about Y, origin at [0, 0, 0.19]
            // ================================================================
            {
                Vec3<Scalar> com6(Scalar(0), Scalar(0), Scalar(0));
                SpatialInertia<Scalar> link6_inertia(Scalar(1.0), com6, I_link_6);
                Vec3<Scalar> r6(Scalar(0), Scalar(0), Scalar(0.19));
                spatial::Transform<Scalar> Xtree6(I3, r6);

                model.template appendBody<Revolute>(
                    "lwr_arm_6_link", link6_inertia, "lwr_arm_5_link", Xtree6,
                    ori::CoordinateAxis::Y, "lwr_arm_5_joint");
            }

            // ================================================================
            // Link 7: Joint about Z, origin at [0, 0, 0.078]
            // ================================================================
            {
                Vec3<Scalar> com7(Scalar(0), Scalar(0), Scalar(0));
                SpatialInertia<Scalar> link7_inertia(Scalar(0.2), com7, I_link_7);
                Vec3<Scalar> r7(Scalar(0), Scalar(0), Scalar(0.078));
                spatial::Transform<Scalar> Xtree7(I3, r7);

                model.template appendBody<Revolute>(
                    "lwr_arm_7_link", link7_inertia, "lwr_arm_6_link", Xtree7,
                    ori::CoordinateAxis::Z, "lwr_arm_6_joint");
            }

            return model;
        }
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_KUKA_LWR_H
