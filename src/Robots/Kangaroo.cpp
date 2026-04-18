#include "grbda/Robots/Kangaroo.hpp"

namespace grbda
{
    template <typename Scalar>
    ClusterTreeModel<Scalar> Kangaroo<Scalar>::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;
        using RevJoint = Joints::Revolute<Scalar>;
        using CoordAxis = ori::CoordinateAxis;

        ClusterTreeModel<Scalar> model{};

        // Set gravity
        model.setGravity(Vec3<Scalar>{0., 0., grav});

        // =====================================================================
        // Hip ground (floating base)
        // From Gepetto URDF: hip_ground is the base link
        // =====================================================================
        const std::string hip_ground_name = base;
        const std::string hip_ground_parent_name = "ground";
        const SpatialInertia<Scalar> hip_ground_spatial_inertia =
            SpatialInertia<Scalar>{hip_ground_mass, hip_ground_CoM, hip_ground_inertia};

        model.template appendBody<Free<Scalar>>(
            hip_ground_name, hip_ground_spatial_inertia,
            hip_ground_parent_name, spatial::Transform<Scalar>{},
            "hip-ground-to-ground");

        // =====================================================================
        // Hip part (hip Z rotation - yaw)
        // Joint: free_zhip (revolute about Z)
        // From URDF: origin xyz="0.0055... 0.0715... -0.935..." relative to hip_ground
        // =====================================================================
        const std::string hip_part_name = "hip_part";
        const std::string hip_part_parent_name = hip_ground_name;

        // Transform computed from URDF origins
        const Vec3<Scalar> hip_part_offset{
            Scalar(0.0055 - 0.018),      // -0.0125
            Scalar(0.0715 - 0.2765),     // -0.205
            Scalar(-0.935 + 0.94)};      // 0.005

        const spatial::Transform<Scalar> hip_part_Xtree(
            Mat3<Scalar>::Identity(), hip_part_offset);

        const SpatialInertia<Scalar> hip_part_spatial_inertia =
            SpatialInertia<Scalar>{hip_part_mass, hip_part_CoM, hip_part_inertia};

        model.template appendBody<Revolute<Scalar>>(
            hip_part_name, hip_part_spatial_inertia,
            hip_part_parent_name, hip_part_Xtree,
            CoordAxis::Z, "free_zhip");

        // =====================================================================
        // Lower hip joint (connects to hip universal mechanism)
        // From URDF: lower_hip is revolute about Y
        // This represents the pitch component of the hip differential
        // =====================================================================
        const std::string lower_hip_name = "lower_hip_link";
        const std::string lower_hip_parent_name = hip_part_name;

        // Position from URDF: the lower_hip joint connects hip_part to universal_hip_y
        const Vec3<Scalar> lower_hip_offset{Scalar(0.0), Scalar(0.0), Scalar(-0.1)};
        const spatial::Transform<Scalar> lower_hip_Xtree(
            Mat3<Scalar>::Identity(), lower_hip_offset);

        // Inertia for the lower hip link (approximated)
        const SpatialInertia<Scalar> lower_hip_spatial_inertia =
            SpatialInertia<Scalar>{Scalar(0.2), Vec3<Scalar>::Zero(),
                                   Mat3<Scalar>::Identity() * Scalar(0.001)};

        model.template appendBody<Revolute<Scalar>>(
            lower_hip_name, lower_hip_spatial_inertia,
            lower_hip_parent_name, lower_hip_Xtree,
            CoordAxis::Y, "lower_hip");

        // =====================================================================
        // Universal hip Y (roll component)
        // From URDF: free_universal_yhip is revolute about X
        // =====================================================================
        const std::string universal_hip_name = "universal_hip_link";
        const std::string universal_hip_parent_name = lower_hip_name;

        const Vec3<Scalar> universal_hip_offset{Scalar(0.0), Scalar(0.0), Scalar(0.0)};
        const spatial::Transform<Scalar> universal_hip_Xtree(
            Mat3<Scalar>::Identity(), universal_hip_offset);

        const SpatialInertia<Scalar> universal_hip_spatial_inertia =
            SpatialInertia<Scalar>{Scalar(0.1), Vec3<Scalar>::Zero(),
                                   Mat3<Scalar>::Identity() * Scalar(0.0005)};

        model.template appendBody<Revolute<Scalar>>(
            universal_hip_name, universal_hip_spatial_inertia,
            universal_hip_parent_name, universal_hip_Xtree,
            CoordAxis::X, "free_universal_yhip");

        // =====================================================================
        // Part 1 (femur/upper leg)
        // Connected via upper_knee joint (revolute about Z in local frame)
        // =====================================================================
        const std::string part_1_name = "part_1";
        const std::string part_1_parent_name = universal_hip_name;

        // From URDF: part_1 connects to universal_hip via implicit connection
        const Vec3<Scalar> part_1_offset{Scalar(0.0), Scalar(0.0), Scalar(-0.05)};
        const spatial::Transform<Scalar> part_1_Xtree(
            Mat3<Scalar>::Identity(), part_1_offset);

        const SpatialInertia<Scalar> part_1_spatial_inertia =
            SpatialInertia<Scalar>{part_1_mass, part_1_CoM, part_1_inertia};

        model.template appendBody<Revolute<Scalar>>(
            part_1_name, part_1_spatial_inertia,
            part_1_parent_name, part_1_Xtree,
            CoordAxis::Z, "femur_rotation");

        // =====================================================================
        // Part 2 (upper knee/thigh link)
        // From URDF: upper_knee joint, revolute about Z
        // Origin: xyz="0 0.04675 -0.0715" rpy="..." relative to part_1
        // =====================================================================
        const std::string part_2_name = "part_2";
        const std::string part_2_parent_name = part_1_name;

        const Vec3<Scalar> part_2_offset{Scalar(0.0), Scalar(0.047), Scalar(-0.072)};
        const spatial::Transform<Scalar> part_2_Xtree(
            Mat3<Scalar>::Identity(), part_2_offset);

        const SpatialInertia<Scalar> part_2_spatial_inertia =
            SpatialInertia<Scalar>{part_2_mass, part_2_CoM, part_2_inertia};

        model.template appendBody<Revolute<Scalar>>(
            part_2_name, part_2_spatial_inertia,
            part_2_parent_name, part_2_Xtree,
            CoordAxis::Z, "upper_knee");

        // =====================================================================
        // Part 3 (lower knee/shin link)
        // From URDF: lower_knee joint, revolute about Z
        // Origin: xyz="-0.217 -0.336 0" relative to part_2
        // =====================================================================
        const std::string part_3_name = "part_3";
        const std::string part_3_parent_name = part_2_name;

        const Vec3<Scalar> part_3_offset{Scalar(-0.217), Scalar(-0.336), Scalar(0.0)};
        const spatial::Transform<Scalar> part_3_Xtree(
            Mat3<Scalar>::Identity(), part_3_offset);

        const SpatialInertia<Scalar> part_3_spatial_inertia =
            SpatialInertia<Scalar>{part_3_mass, part_3_CoM, part_3_inertia};

        model.template appendBody<Revolute<Scalar>>(
            part_3_name, part_3_spatial_inertia,
            part_3_parent_name, part_3_Xtree,
            CoordAxis::Z, "lower_knee");

        // =====================================================================
        // Universal foot (ankle mechanism)
        // From URDF: universal_foot_up then free_ankle_universalx
        // This is a 2-DOF ankle (pitch + roll)
        // =====================================================================
        const std::string ankle_pitch_name = "ankle_pitch_link";
        const std::string ankle_pitch_parent_name = part_3_name;

        const Vec3<Scalar> ankle_pitch_offset{Scalar(0.217), Scalar(-0.336), Scalar(0.0)};
        const spatial::Transform<Scalar> ankle_pitch_Xtree(
            Mat3<Scalar>::Identity(), ankle_pitch_offset);

        const SpatialInertia<Scalar> ankle_pitch_spatial_inertia =
            SpatialInertia<Scalar>{Scalar(0.01), Vec3<Scalar>::Zero(),
                                   Mat3<Scalar>::Identity() * Scalar(0.00001)};

        model.template appendBody<Revolute<Scalar>>(
            ankle_pitch_name, ankle_pitch_spatial_inertia,
            ankle_pitch_parent_name, ankle_pitch_Xtree,
            CoordAxis::Y, "universal_foot_up");

        // =====================================================================
        // Foot part (final link)
        // =====================================================================
        const std::string foot_name = "foot_part";
        const std::string foot_parent_name = ankle_pitch_name;

        const Vec3<Scalar> foot_offset{Scalar(0.0), Scalar(0.0), Scalar(0.0)};
        const spatial::Transform<Scalar> foot_Xtree(
            Mat3<Scalar>::Identity(), foot_offset);

        const SpatialInertia<Scalar> foot_spatial_inertia =
            SpatialInertia<Scalar>{foot_part_mass, foot_part_CoM, foot_part_inertia};

        model.template appendBody<Revolute<Scalar>>(
            foot_name, foot_spatial_inertia,
            foot_parent_name, foot_Xtree,
            CoordAxis::X, "free_ankle_universalx");

        // =====================================================================
        // Contact points on the foot
        // =====================================================================
        model.appendContactPoint(foot_name, Vec3<Scalar>{Scalar(0.05), Scalar(0.0), Scalar(0.0)},
                                 "toe-contact");
        model.appendContactPoint(foot_name, Vec3<Scalar>{Scalar(-0.05), Scalar(0.0), Scalar(0.0)},
                                 "heel-contact");

        // =====================================================================
        // CLOSED-LOOP CONSTRAINTS
        //
        // The full Kangaroo leg from Gepetto example-parallel-robots has:
        // - 6 prismatic motors: motor_hipz, motor_hip_xy1, motor_hip_xy2,
        //                       motor_knee, motor_ankle1, motor_ankle2
        // - 11 closed-loop constraints (all 6D type)
        //
        // Constraint Structure (from robot.yaml):
        // ----------------------------------------
        // closedloop0:  [closedloop0_B (part_7), closedloop0_A (foot_part)]
        // closedloop2:  [closedloop2_A (cylinder_8), closedloop2_B (cylinder_5)]
        // closedloop3:  [closedloop3_B (cylinder_10), closedloop3_A (cylinder)]
        // closedloop4:  [closedloop4_B (cylinder_9), closedloop4_A (cylinder_2)]
        // closedloop5:  [closedloop5_A (hip_rotation_motor), closedloop5_B (holder_hip)]
        // closedloop6:  [closedloop6_A (part_4), closedloop6_B (part_6)]
        // closedloop7:  [closedloop7_B (part_1), closedloop7_A (part_5)]
        // closedloop8:  [closedloop8_B (cylinder_6), closedloop8_A (cylinder_3)]
        // closedloop9:  [closedloop9_A (part_7_4), closedloop9_B (transmission_ankle)]
        // closedloop10: [closedloop10_A (part_7_3), closedloop10_B (transmission_ankle2)]
        // closedloop11: [closedloop11_B (part_7_2), closedloop11_A (foot_part)]
        //
        // To implement these as GenericImplicit constraints:
        //
        // 1. For each closure pair (frame_A, frame_B), compute forward kinematics
        //    from their common ancestor to each frame.
        //
        // 2. The constraint is: T_A(q) = T_B(q), which gives 6 equations:
        //    - Position: p_A(q) - p_B(q) = 0  (3 equations)
        //    - Rotation: log(R_A(q)^T * R_B(q)) = 0  (3 equations, using axis-angle)
        //
        // 3. Identify independent coordinates (motor positions) and dependent
        //    coordinates (linkage joint angles) based on which coordinates can
        //    be freely set vs. which are determined by the constraints.
        //
        // 4. Create constraint phi function in CasADi symbolic form and native
        //    C++ form (for complex-step differentiation support).
        //
        // Example for a planar 4-bar (like closedloop6/7 for knee):
        //   phi = [ L1*cos(q1) + L2*cos(q1+q2) - L3*cos(q3) - L4 ]
        //         [ L1*sin(q1) + L2*sin(q1+q2) - L3*sin(q3)      ]
        //
        // The existing Cassie implementation uses FourBar constraint which is
        // a specialized 2D constraint. For Kangaroo's 6D constraints, use
        // GenericImplicit with the full 6D loop closure equations.
        //
        // Reference: See Tello.cpp for GenericImplicit usage pattern.
        // =====================================================================

        return model;
    }

    // Template instantiations
    template class Kangaroo<double>;
    template class Kangaroo<float>;
    template class Kangaroo<casadi::SX>;

} // namespace grbda
