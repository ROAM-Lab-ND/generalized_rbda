#include "grbda/Robots/Cassie.hpp"

namespace grbda
{
    template <typename Scalar>
    ClusterTreeModel<Scalar> Cassie<Scalar>::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;
        using RevJoint = Joints::Revolute<Scalar>;
        using CoordAxis = ori::CoordinateAxis;

        ClusterTreeModel<Scalar> model{};
        model.setGravity(Vec3<Scalar>{0., 0., grav});

        const std::string pelvis_name = base;
        const std::string ground_name = "ground";
        const SpatialInertia<Scalar> pelvis_spatial_inertia =
            SpatialInertia<Scalar>{pelvis_mass, pelvis_CoM, pelvis_inertia};

        model.template appendBody<Free<Scalar>>(
            pelvis_name, pelvis_spatial_inertia,
            ground_name, spatial::Transform<Scalar>{},
            "pelvis-to-ground");

        const std::vector<std::string> sides = {"left", "right"};

        for (size_t i = 0; i < sides.size(); ++i) {
            const std::string& side = sides[i];
            const Scalar side_sign = (i == 0) ? Scalar(1.) : Scalar(-1.);

            const std::string hip_roll_name = side + "-hip-roll";
            Vec3<Scalar> hip_roll_pos = hip_roll_joint_pos;
            hip_roll_pos(1) *= side_sign;
            const spatial::Transform<Scalar> hip_roll_Xtree(Mat3<Scalar>::Identity(), hip_roll_pos);
            const SpatialInertia<Scalar> hip_roll_inertia(
                hip_roll_mass, hip_roll_CoM, Mat3<Scalar>::Identity() * Scalar(0.0035));
            model.template appendBody<Revolute<Scalar>>(
                hip_roll_name, hip_roll_inertia,
                pelvis_name, hip_roll_Xtree,
                CoordAxis::X, side + "-hip-roll-joint");

            const std::string hip_yaw_name = side + "-hip-yaw";
            Vec3<Scalar> hip_yaw_pos = hip_yaw_joint_offset;
            hip_yaw_pos(1) *= side_sign;
            const spatial::Transform<Scalar> hip_yaw_Xtree(Mat3<Scalar>::Identity(), hip_yaw_pos);
            const SpatialInertia<Scalar> hip_yaw_inertia(
                hip_yaw_mass, hip_yaw_CoM, Mat3<Scalar>::Identity() * Scalar(0.0025));
            model.template appendBody<Revolute<Scalar>>(
                hip_yaw_name, hip_yaw_inertia,
                hip_roll_name, hip_yaw_Xtree,
                CoordAxis::Z, side + "-hip-yaw-joint");

            const std::string hip_pitch_name = side + "-hip-pitch";
            Vec3<Scalar> hip_pitch_pos = hip_pitch_joint_offset;
            hip_pitch_pos(1) *= side_sign;
            const spatial::Transform<Scalar> hip_pitch_Xtree(Mat3<Scalar>::Identity(), hip_pitch_pos);
            const SpatialInertia<Scalar> hip_pitch_inertia(
                hip_pitch_mass, hip_pitch_CoM, Mat3<Scalar>::Identity() * Scalar(0.01));
            model.template appendBody<Revolute<Scalar>>(
                hip_pitch_name, hip_pitch_inertia,
                hip_yaw_name, hip_pitch_Xtree,
                CoordAxis::Y, side + "-hip-pitch-joint");

            const std::string achilles_name = side + "-achilles-rod";
            const std::string knee_name = side + "-knee";
            const std::string foot_name = side + "-foot";

            // Lower-leg branch offsets from Cassie MJCF (left leg mirrored for right).
            const Vec3<Scalar> achilles_location{Scalar(0.0), Scalar(0.0), Scalar(0.045) * side_sign};
            const Vec3<Scalar> knee_location{Scalar(0.12), Scalar(0.0), Scalar(0.0045) * side_sign};
            const Vec3<Scalar> foot_location{Scalar(0.408), Scalar(-0.04), Scalar(0.0)};

            const spatial::Transform<Scalar> achilles_Xtree(Mat3<Scalar>::Identity(), achilles_location);
            const spatial::Transform<Scalar> knee_Xtree(Mat3<Scalar>::Identity(), knee_location);
            const spatial::Transform<Scalar> foot_Xtree(Mat3<Scalar>::Identity(), foot_location);

            const SpatialInertia<Scalar> achilles_inertia(
                achilles_mass, achilles_CoM, Mat3<Scalar>::Identity() * Scalar(5e-6));
            const SpatialInertia<Scalar> knee_inertia(
                knee_mass, knee_CoM, Mat3<Scalar>::Identity() * Scalar(0.0015));
            const SpatialInertia<Scalar> foot_inertia(
                foot_mass, foot_CoM, Mat3<Scalar>::Identity() * Scalar(2e-4));

            auto achilles = model.registerBody(achilles_name, achilles_inertia, hip_pitch_name, achilles_Xtree);
            auto knee = model.registerBody(knee_name, knee_inertia, hip_pitch_name, knee_Xtree);
            auto foot = model.registerBody(foot_name, foot_inertia, knee_name, foot_Xtree);

            std::vector<Body<Scalar>> leg_bodies = {achilles, knee, foot};
            std::vector<JointPtr<Scalar>> leg_joints = {
                std::make_shared<RevJoint>(CoordAxis::Z, side + "-achilles-to-knee"),
                std::make_shared<RevJoint>(CoordAxis::Z, side + "-knee-to-foot"),
                std::make_shared<RevJoint>(CoordAxis::Z, side + "-foot-link")};

            // Cassie MJCF equality/connect anchors:
            // - plantar-rod to foot:   0.35012
            // - achilles-rod to spring: 0.5012
            // The four-bar abstraction uses these as closure lengths.
            std::vector<Scalar> path1_link_lengths = {Scalar(0.35012), Scalar(0.15108)};
            std::vector<Scalar> path2_link_lengths = {Scalar(0.35012)};
            Vec2<Scalar> offset{Scalar(0.15108), Scalar(0.0)};
            const int independent_coordinate = 0;

            auto leg_constraint = std::make_shared<LoopConstraint::FourBar<Scalar>>(
                path1_link_lengths, path2_link_lengths, offset, independent_coordinate);

            model.template appendRegisteredBodiesAsCluster<ClusterJoints::FourBar<Scalar>>(
                side + "-lower-leg-loop", leg_bodies, leg_joints, leg_constraint);

            model.appendContactPoint(foot_name, Vec3<Scalar>{Scalar(0.069746), Scalar(-0.010224), Scalar(0.0)},
                                     side + "-toe-contact");
            model.appendContactPoint(foot_name, Vec3<Scalar>{Scalar(-0.052821), Scalar(0.092622), Scalar(0.0)},
                                     side + "-heel-contact");
        }

        return model;
    }

    template class Cassie<double>;
    template class Cassie<std::complex<double>>;
    template class Cassie<casadi::SX>;

} // namespace grbda