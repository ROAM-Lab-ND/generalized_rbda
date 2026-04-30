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

            // ---------------------------------------------------------------
            // Hip serial chain (3 revolute clusters)
            // ---------------------------------------------------------------

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

            // ---------------------------------------------------------------
            // Upper four-bar cluster (parent = hip-pitch)
            //
            // Bodies:
            //   [0] knee+shin     — child of hip-pitch  (path1 link 1, dep, L=0.077)
            //   [1] achilles-rod  — child of hip-pitch  (path2, ind, L=0.5012)
            //   [2] tarsus        — child of knee+shin  (path1 link 2, dep, L=0.422)
            //
            // independent_coordinate = 1 (achilles, passive — knee/tarsus are dependent)
            // Kd = [dPhi/dKnee, dPhi/dTarsus] uses L1 and L2 -> well-conditioned.
            //
            // All Xtrees use R=Identity so joint angles measured from x-axis, matching phi.
            // tarsus Xtree = shin_on_knee + tarsus_on_shin = (0.49544, 0.06741, 0)
            // offset = achilles_pivot - knee_pivot in hip-pitch XY = (-0.12, 0)
            // ---------------------------------------------------------------

            const std::string achilles_name = side + "-achilles-rod";
            const std::string knee_shin_name = side + "-knee-shin";
            const std::string tarsus_name = side + "-tarsus";

            // Xtree for achilles-rod: (0, 0, ±0.045) from hip-pitch
            const Vec3<Scalar> achilles_pos{Scalar(0.), Scalar(0.), Scalar(0.045) * side_sign};
            const spatial::Transform<Scalar> achilles_Xtree(Mat3<Scalar>::Identity(), achilles_pos);

            // Xtree for knee+shin: (0.12, 0, ±0.0045) from hip-pitch
            const Vec3<Scalar> knee_shin_pos{Scalar(0.12), Scalar(0.), Scalar(0.0045) * side_sign};
            const spatial::Transform<Scalar> knee_shin_Xtree(Mat3<Scalar>::Identity(), knee_shin_pos);

            // Xtree for tarsus+heel-spring: shin joint position in knee+shin body frame.
            // L1 = |shin_on_knee| = 0.077m (knee pivot to shin joint).
            // L2 = |(shin_to_tarsus) + (heel_on_tarsus)| = 0.422m (shin joint to heel-spring
            //      pivot, straight-line across the rigid tarsus body).
            const Vec3<Scalar> tarsus_pos{Scalar(0.06068), Scalar(0.04741), Scalar(0.)};
            const spatial::Transform<Scalar> tarsus_Xtree(Mat3<Scalar>::Identity(), tarsus_pos);

            const SpatialInertia<Scalar> achilles_inertia(
                achilles_mass, achilles_CoM, Mat3<Scalar>::Identity() * Scalar(4.5e-3));
            const SpatialInertia<Scalar> knee_shin_inertia(
                knee_shin_mass, knee_shin_CoM, Mat3<Scalar>::Identity() * Scalar(0.005));
            const SpatialInertia<Scalar> tarsus_inertia(
                tarsus_mass, tarsus_CoM, Mat3<Scalar>::Identity() * Scalar(0.014));

            auto knee_shin_body = model.registerBody(knee_shin_name, knee_shin_inertia,
                                                     hip_pitch_name, knee_shin_Xtree);
            auto achilles_body = model.registerBody(achilles_name, achilles_inertia,
                                                    hip_pitch_name, achilles_Xtree);
            auto tarsus_body = model.registerBody(tarsus_name, tarsus_inertia,
                                                  knee_shin_name, tarsus_Xtree);

            std::vector<Body<Scalar>> upper_bodies = {knee_shin_body, achilles_body, tarsus_body};
            std::vector<JointPtr<Scalar>> upper_joints = {
                std::make_shared<RevJoint>(CoordAxis::Z, side + "-knee-joint"),
                std::make_shared<RevJoint>(CoordAxis::Z, side + "-achilles-joint"),
                std::make_shared<RevJoint>(CoordAxis::Z, side + "-shin-joint")};

            // path1 = {knee+shin (q[0], dep), tarsus (q[2], dep)}, path2 = {achilles (q[1], ind)}
            // independent = 1 (achilles, passive)
            // L1 = |shin_on_knee| = 0.077005 m (knee+shin arm = knee pivot to shin joint)
            // L2 = |shin_to_tarsus + heel_on_tarsus| = 0.422204 m (shin joint to heel-spring pivot)
            // L3 = 0.5012 m (achilles rod arm to closure point)
            // offset = achilles_pivot - knee_pivot in hip-pitch XY = (0-0.12, 0-0) = (-0.12, 0)
            std::vector<Scalar> upper_path1 = {Scalar(0.077005), Scalar(0.422204)};
            std::vector<Scalar> upper_path2 = {Scalar(0.5012)};
            Vec2<Scalar> upper_offset{Scalar(-0.12), Scalar(0.0)};

            auto upper_constraint = std::make_shared<LoopConstraint::FourBar<Scalar>>(
                upper_path1, upper_path2, upper_offset, 1);

            model.template appendRegisteredBodiesAsCluster<ClusterJoints::FourBar<Scalar>>(
                side + "-upper-leg-loop", upper_bodies, upper_joints, upper_constraint);

            // ---------------------------------------------------------------
            // Lower four-bar cluster (parent = tarsus+heel-spring body)
            //
            // Bodies:
            //   [0] foot-crank   — child of tarsus  (path1 link 1, L=0.055)
            //   [1] foot         — child of tarsus  (path2, zero-length rocker)
            //   [2] plantar-rod  — child of foot-crank (path1 link 2, L=0.35012)
            //
            // independent_coordinate = 1 (foot joint, actuated)
            //
            // offset = foot_pivot - foot_crank_pivot on tarsus
            //        = (0.408,-0.04) - (0.058,-0.034) = (0.35, -0.006)
            // ---------------------------------------------------------------

            const std::string foot_crank_name = side + "-foot-crank";
            const std::string plantar_rod_name = side + "-plantar-rod";
            const std::string foot_name = side + "-foot";

            // Xtree for foot-crank: (0.058, -0.034, ±0.02275) from tarsus
            const Vec3<Scalar> foot_crank_pos{Scalar(0.058), Scalar(-0.034),
                                              Scalar(0.02275) * side_sign};
            const spatial::Transform<Scalar> foot_crank_Xtree(Mat3<Scalar>::Identity(),
                                                               foot_crank_pos);

            // Xtree for foot: (0.408, -0.04, 0) from tarsus
            const Vec3<Scalar> foot_pos{Scalar(0.408), Scalar(-0.04), Scalar(0.)};
            const spatial::Transform<Scalar> foot_Xtree(Mat3<Scalar>::Identity(), foot_pos);

            // Xtree for plantar-rod: (0.055, 0, ∓0.00791) from foot-crank
            const Vec3<Scalar> plantar_pos{Scalar(0.055), Scalar(0.),
                                           Scalar(-0.00791) * side_sign};
            const spatial::Transform<Scalar> plantar_Xtree(Mat3<Scalar>::Identity(), plantar_pos);

            const SpatialInertia<Scalar> foot_crank_inertia(
                foot_crank_mass, foot_crank_CoM, Mat3<Scalar>::Identity() * Scalar(5e-5));
            const SpatialInertia<Scalar> foot_inertia(
                foot_mass, foot_CoM, Mat3<Scalar>::Identity() * Scalar(2e-4));
            const SpatialInertia<Scalar> plantar_rod_inertia(
                plantar_rod_mass, plantar_rod_CoM, Mat3<Scalar>::Identity() * Scalar(1.8e-3));

            auto foot_crank_body = model.registerBody(foot_crank_name, foot_crank_inertia,
                                                      tarsus_name, foot_crank_Xtree);
            auto foot_body = model.registerBody(foot_name, foot_inertia,
                                                tarsus_name, foot_Xtree);
            auto plantar_rod_body = model.registerBody(plantar_rod_name, plantar_rod_inertia,
                                                       foot_crank_name, plantar_Xtree);

            std::vector<Body<Scalar>> lower_bodies = {foot_crank_body, foot_body, plantar_rod_body};
            std::vector<JointPtr<Scalar>> lower_joints = {
                std::make_shared<RevJoint>(CoordAxis::Z, side + "-foot-crank-joint"),
                std::make_shared<RevJoint>(CoordAxis::Z, side + "-foot-joint"),
                std::make_shared<RevJoint>(CoordAxis::Z, side + "-plantar-joint")};

            // path1 = {foot-crank (q[0]), plantar-rod (q[2])}, path2 = {foot (q[1], L=0)}
            // independent = 1 (foot joint, actuated)
            std::vector<Scalar> lower_path1 = {Scalar(0.055), Scalar(0.35012)};
            std::vector<Scalar> lower_path2 = {Scalar(0.)};
            Vec2<Scalar> lower_offset{Scalar(0.35), Scalar(-0.006)};

            auto lower_constraint = std::make_shared<LoopConstraint::FourBar<Scalar>>(
                lower_path1, lower_path2, lower_offset, 1);

            model.template appendRegisteredBodiesAsCluster<ClusterJoints::FourBar<Scalar>>(
                side + "-lower-leg-loop", lower_bodies, lower_joints, lower_constraint);

            model.appendContactPoint(foot_name,
                Vec3<Scalar>{Scalar(0.069746), Scalar(-0.010224), Scalar(0.)},
                side + "-toe-contact");
            model.appendContactPoint(foot_name,
                Vec3<Scalar>{Scalar(-0.052821), Scalar(0.092622), Scalar(0.)},
                side + "-heel-contact");
        }

        return model;
    }

    template class Cassie<double>;
    template class Cassie<std::complex<double>>;
    template class Cassie<casadi::SX>;

} // namespace grbda
