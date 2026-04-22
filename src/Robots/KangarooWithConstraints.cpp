#include "grbda/Robots/KangarooWithConstraints.hpp"
#include "grbda/Dynamics/ClusterJoints/FourBarJoint.h"
#include <complex>

namespace grbda
{
    template <typename Scalar>
    ClusterTreeModel<Scalar> KangarooWithConstraints<Scalar>::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;
        using RevJoint = Joints::Revolute<Scalar>;
        using CoordAxis = ori::CoordinateAxis;

        ClusterTreeModel<Scalar> model{};
        model.setGravity(Vec3<Scalar>{0., 0., grav});

        // Hip ground (floating base)
        const std::string hip_ground_name = base;
        const SpatialInertia<Scalar> hip_ground_spatial_inertia =
            SpatialInertia<Scalar>{hip_ground_mass, hip_ground_CoM, hip_ground_inertia};

        model.template appendBody<Free<Scalar>>(
            hip_ground_name, hip_ground_spatial_inertia,
            "ground", spatial::Transform<Scalar>{},
            "hip-ground-to-ground");

        // Hip part (yaw)
        const std::string hip_part_name = "hip_part";
        const Vec3<Scalar> hip_part_offset{Scalar(-0.0125), Scalar(-0.205), Scalar(0.005)};
        const spatial::Transform<Scalar> hip_part_Xtree(Mat3<Scalar>::Identity(), hip_part_offset);
        const SpatialInertia<Scalar> hip_part_spatial_inertia =
            SpatialInertia<Scalar>{hip_part_mass, hip_part_CoM, hip_part_inertia};

        model.template appendBody<Revolute<Scalar>>(
            hip_part_name, hip_part_spatial_inertia,
            hip_ground_name, hip_part_Xtree,
            CoordAxis::Z, "free_zhip");

        // Lower hip (pitch)
        const std::string lower_hip_name = "lower_hip_link";
        const Vec3<Scalar> lower_hip_offset{Scalar(0.0), Scalar(0.0), Scalar(-0.1)};
        const spatial::Transform<Scalar> lower_hip_Xtree(Mat3<Scalar>::Identity(), lower_hip_offset);
        const SpatialInertia<Scalar> lower_hip_spatial_inertia =
            SpatialInertia<Scalar>{Scalar(0.2), Vec3<Scalar>::Zero(),
                                   Mat3<Scalar>::Identity() * Scalar(0.001)};

        model.template appendBody<Revolute<Scalar>>(
            lower_hip_name, lower_hip_spatial_inertia,
            hip_part_name, lower_hip_Xtree,
            CoordAxis::Y, "lower_hip");

        // Universal hip (roll)
        const std::string universal_hip_name = "universal_hip_link";
        const spatial::Transform<Scalar> universal_hip_Xtree(Mat3<Scalar>::Identity(), Vec3<Scalar>::Zero());
        const SpatialInertia<Scalar> universal_hip_spatial_inertia =
            SpatialInertia<Scalar>{Scalar(0.1), Vec3<Scalar>::Zero(),
                                   Mat3<Scalar>::Identity() * Scalar(0.0005)};

        model.template appendBody<Revolute<Scalar>>(
            universal_hip_name, universal_hip_spatial_inertia,
            lower_hip_name, universal_hip_Xtree,
            CoordAxis::X, "free_universal_yhip");

        // Part 1 (femur)
        const std::string part_1_name = "part_1";
        const Vec3<Scalar> part_1_offset{Scalar(0.0), Scalar(0.0), Scalar(-0.05)};
        const spatial::Transform<Scalar> part_1_Xtree(Mat3<Scalar>::Identity(), part_1_offset);
        const SpatialInertia<Scalar> part_1_spatial_inertia =
            SpatialInertia<Scalar>{part_1_mass, part_1_CoM, part_1_inertia};

        model.template appendBody<Revolute<Scalar>>(
            part_1_name, part_1_spatial_inertia,
            universal_hip_name, part_1_Xtree,
            CoordAxis::Z, "femur_rotation");

        // KNEE 4-BAR MECHANISM
        // Register bodies for the 4-bar cluster
        const std::string part_2_name = "part_2";
        const Vec3<Scalar> part_2_offset{Scalar(0.0), Scalar(0.047), Scalar(-0.072)};
        const spatial::Transform<Scalar> part_2_Xtree(Mat3<Scalar>::Identity(), part_2_offset);
        const SpatialInertia<Scalar> part_2_spatial_inertia =
            SpatialInertia<Scalar>{part_2_mass, part_2_CoM,
                                   Mat3<Scalar>::Identity() * Scalar(0.002)};

        auto part_2_body = model.registerBody(part_2_name, part_2_spatial_inertia,
                                               part_1_name, part_2_Xtree);

        const std::string part_4_name = "part_4";
        const Vec3<Scalar> part_4_offset{Scalar(0.046), Scalar(-0.0195), Scalar(0.0)};
        const spatial::Transform<Scalar> part_4_Xtree(Mat3<Scalar>::Identity(), part_4_offset);
        const SpatialInertia<Scalar> part_4_spatial_inertia =
            SpatialInertia<Scalar>{part_4_mass, part_4_CoM,
                                   Mat3<Scalar>::Identity() * Scalar(0.00001)};

        auto part_4_body = model.registerBody(part_4_name, part_4_spatial_inertia,
                                               part_2_name, part_4_Xtree);

        const std::string part_5_name = "part_5";
        const Vec3<Scalar> part_5_offset{Scalar(0.0195), Scalar(-0.046), Scalar(0.0)};
        const spatial::Transform<Scalar> part_5_Xtree(Mat3<Scalar>::Identity(), part_5_offset);
        const SpatialInertia<Scalar> part_5_spatial_inertia =
            SpatialInertia<Scalar>{part_5_mass, part_5_CoM,
                                   Mat3<Scalar>::Identity() * Scalar(0.00002)};

        auto part_5_body = model.registerBody(part_5_name, part_5_spatial_inertia,
                                               part_4_name, part_5_Xtree);

        // Create joints for the 4-bar
        std::vector<Body<Scalar>> fourbar_bodies = {part_2_body, part_4_body, part_5_body};
        std::vector<JointPtr<Scalar>> fourbar_joints = {
            std::make_shared<RevJoint>(CoordAxis::Z, "upper_knee"),
            std::make_shared<RevJoint>(CoordAxis::Z, "4barlink_knee"),
            std::make_shared<RevJoint>(CoordAxis::Z, "sherical_4bar")
        };

        // FourBar constraint parameters
        // For a valid 4-bar: path1 = L1 + L2, path2 = L3, offset = L4 (ground link)
        // Must satisfy Grashof criterion for proper closure
        // Using a crank-rocker style 4-bar:
        std::vector<Scalar> path1_link_lengths = {Scalar(0.1), Scalar(0.1)};  // L1, L2
        std::vector<Scalar> path2_link_lengths = {Scalar(0.14)};               // L3
        Vec2<Scalar> offset{Scalar(0.06), Scalar(0.0)};                        // L4 (ground)
        const int independent_coordinate = 0;

        auto fourbar_constraint = std::make_shared<LoopConstraint::FourBar<Scalar>>(
            path1_link_lengths, path2_link_lengths, offset, independent_coordinate);

        model.template appendRegisteredBodiesAsCluster<ClusterJoints::FourBar<Scalar>>(
            "knee-4bar", fourbar_bodies, fourbar_joints, fourbar_constraint);

        // Part 3 (shin)
        const std::string part_3_name = "part_3";
        const Vec3<Scalar> part_3_offset{Scalar(-0.217), Scalar(-0.336), Scalar(0.0)};
        const spatial::Transform<Scalar> part_3_Xtree(Mat3<Scalar>::Identity(), part_3_offset);
        const SpatialInertia<Scalar> part_3_spatial_inertia =
            SpatialInertia<Scalar>{part_3_mass, part_3_CoM,
                                   Mat3<Scalar>::Identity() * Scalar(0.0003)};

        model.template appendBody<Revolute<Scalar>>(
            part_3_name, part_3_spatial_inertia,
            part_2_name, part_3_Xtree,
            CoordAxis::Z, "lower_knee");

        // Ankle and foot
        const std::string ankle_name = "ankle_link";
        const Vec3<Scalar> ankle_offset{Scalar(0.217), Scalar(-0.336), Scalar(0.0)};
        const spatial::Transform<Scalar> ankle_Xtree(Mat3<Scalar>::Identity(), ankle_offset);
        const SpatialInertia<Scalar> ankle_spatial_inertia =
            SpatialInertia<Scalar>{Scalar(0.01), Vec3<Scalar>::Zero(),
                                   Mat3<Scalar>::Identity() * Scalar(0.00001)};

        model.template appendBody<Revolute<Scalar>>(
            ankle_name, ankle_spatial_inertia,
            part_3_name, ankle_Xtree,
            CoordAxis::Y, "ankle_pitch");

        const std::string foot_name = "foot_part";
        const spatial::Transform<Scalar> foot_Xtree(Mat3<Scalar>::Identity(), Vec3<Scalar>::Zero());
        const SpatialInertia<Scalar> foot_spatial_inertia =
            SpatialInertia<Scalar>{foot_part_mass, foot_part_CoM,
                                   Mat3<Scalar>::Identity() * Scalar(0.00005)};

        model.template appendBody<Revolute<Scalar>>(
            foot_name, foot_spatial_inertia,
            ankle_name, foot_Xtree,
            CoordAxis::X, "ankle_roll");

        // Contact points
        model.appendContactPoint(foot_name, Vec3<Scalar>{Scalar(0.05), Scalar(0.0), Scalar(0.0)},
                                 "toe-contact");
        model.appendContactPoint(foot_name, Vec3<Scalar>{Scalar(-0.05), Scalar(0.0), Scalar(0.0)},
                                 "heel-contact");

        return model;
    }

    template class KangarooWithConstraints<double>;
    template class KangarooWithConstraints<casadi::SX>;
    template class KangarooWithConstraints<std::complex<double>>;

} // namespace grbda
