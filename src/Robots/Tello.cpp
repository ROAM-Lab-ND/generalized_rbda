#include "grbda/Robots/Tello.hpp"

namespace grbda
{

    template <typename Scalar>
    ClusterTreeModel<Scalar> Tello<Scalar>::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;

        using RevJoint = Joints::Revolute<Scalar>;
        using CoordAxis = ori::CoordinateAxis;
        using LoopConstraintType = LoopConstraint::GenericImplicit<Scalar>;

        ClusterTreeModel<Scalar> model{};

        // Set gravity in z direction
        model.setGravity(Vec3<Scalar>{0., 0., grav});

        // Torso
        /*const std::string torso_name = base;
        const std::string torso_parent_name = "ground";
        const SpatialInertia<Scalar> torso_spatial_inertia =
            SpatialInertia<Scalar>{torso_mass, torso_CoM, torso_inertia};
        model.appendBody<Free<Scalar>>(torso_name, torso_spatial_inertia,
                                 torso_parent_name, spatial::Transform{});*/

        std::vector<std::string> sides = {"left", "right"};
        const std::string hip_clamp_parent_name = base;
        const std::string hip_clamp_rotor_parent_name = base;

        for (size_t i(0); i < 2; i++)
        {
            const std::string side = sides[i];

            // Hip clamp
            const Mat3<Scalar> R_hip_clamp = i == 0 ? R_left_hip_clamp : R_right_hip_clamp;
            const Vec3<Scalar> p_hip_clamp = i == 0 ? p_left_hip_clamp : p_right_hip_clamp;
            const spatial::Transform<Scalar> hip_clamp_Xtree = spatial::Transform(R_hip_clamp,
                                                                            p_hip_clamp);
            const std::string hip_clamp_name = side + "-hip-clamp";
            const SpatialInertia<Scalar> hip_clamp_spatial_inertia =
                SpatialInertia<Scalar>{hip_clamp_mass, hip_clamp_CoM, hip_clamp_inertia};
            auto hip_clamp = model.registerBody(hip_clamp_name, hip_clamp_spatial_inertia,
                                                hip_clamp_parent_name, hip_clamp_Xtree);

            // Hip clamp rotor
            const Mat3<Scalar> R_hip_clamp_rotor = i == 0 ? R_left_hip_clamp_rotor
                                                          : R_right_hip_clamp_rotor;
            const Vec3<Scalar> p_hip_clamp_rotor = i == 0 ? p_left_hip_clamp_rotor
                                                          : p_right_hip_clamp_rotor;
            const spatial::Transform<Scalar> hip_clamp_rotor_Xtree = spatial::Transform(
                R_hip_clamp_rotor, p_hip_clamp_rotor);
            const std::string hip_clamp_rotor_name = side + "-hip-clamp-rotor";
            const SpatialInertia<Scalar> hip_clamp_rotor_spatial_inertia =
                SpatialInertia<Scalar>{hip_clamp_rotor_mass, hip_clamp_rotor_CoM,
                                       hip_clamp_rotor_inertia};
            auto hip_clamp_rotor = model.registerBody(hip_clamp_rotor_name,
                                                      hip_clamp_rotor_spatial_inertia,
                                                      hip_clamp_rotor_parent_name,
                                                      hip_clamp_rotor_Xtree);

            // Hip clamp cluster
            const std::string hip_clamp_cluster_name = side + "-hip-clamp";
            const std::string hip_clamp_joint_name = base + "-to-" + side + "-hip-clamp";
            const std::string hip_clamp_rotor_joint_name = base + "-to-" + side + "-hip-clamp-rotor";
            GearedTransmissionModule<Scalar> hip_clamp_module{hip_clamp, hip_clamp_rotor,
                                                              hip_clamp_joint_name,
                                                              hip_clamp_rotor_joint_name,
                                                              ori::CoordinateAxis::Z,
                                                              ori::CoordinateAxis::Z,
                                                              gear_ratio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor<Scalar>>(
                hip_clamp_cluster_name, hip_clamp_module);

            // Hip differential rotor 1
            const Mat3<Scalar> R_hip_rotor_1 = i == 0 ? R_left_hip_rotor_1 : R_right_hip_rotor_1;
            const Vec3<Scalar> p_hip_rotor_1 = i == 0 ? p_left_hip_rotor_1 : p_right_hip_rotor_1;
            const spatial::Transform<Scalar> hip_rotor_1_Xtree = spatial::Transform(R_hip_rotor_1,
                                                                              p_hip_rotor_1);
            const std::string hip_rotor_1_name = side + "-hip-rotor-1";
            const std::string hip_rotor_1_parent_name = side + "-hip-clamp";
            const SpatialInertia<Scalar> hip_rotor_1_spatial_inertia =
                SpatialInertia<Scalar>{hip_rotor_1_mass, hip_rotor_1_CoM, hip_rotor_1_inertia};
            auto hip_rotor_1 = model.registerBody(hip_rotor_1_name, hip_rotor_1_spatial_inertia,
                                                  hip_rotor_1_parent_name, hip_rotor_1_Xtree);

            // Hip differential rotor 2
            const Mat3<Scalar> R_hip_rotor_2 = i == 0 ? R_left_hip_rotor_2 : R_right_hip_rotor_2;
            const Vec3<Scalar> p_hip_rotor_2 = i == 0 ? p_left_hip_rotor_2 : p_right_hip_rotor_2;
            const spatial::Transform<Scalar> hip_rotor_2_Xtree = spatial::Transform(R_hip_rotor_2,
                                                                              p_hip_rotor_2);
            const std::string hip_rotor_2_name = side + "-hip-rotor-2";
            const std::string hip_rotor_2_parent_name = side + "-hip-clamp";
            const SpatialInertia<Scalar> hip_rotor_2_spatial_inertia =
                SpatialInertia<Scalar>{hip_rotor_2_mass, hip_rotor_2_CoM, hip_rotor_2_inertia};
            auto hip_rotor_2 = model.registerBody(hip_rotor_2_name, hip_rotor_2_spatial_inertia,
                                                  hip_rotor_2_parent_name, hip_rotor_2_Xtree);

            // Gimbal
            const Mat3<Scalar> R_gimbal = i == 0 ? R_left_gimbal : R_right_gimbal;
            const Vec3<Scalar> p_gimbal = i == 0 ? p_left_gimbal : p_right_gimbal;
            const spatial::Transform<Scalar> gimbal_Xtree = spatial::Transform(R_gimbal, p_gimbal);
            const std::string gimbal_name = side + "-gimbal";
            const std::string gimbal_parent_name = side + "-hip-clamp";
            const SpatialInertia<Scalar> gimbal_spatial_inertia =
                SpatialInertia<Scalar>{gimbal_mass, gimbal_CoM, gimbal_inertia};
            auto gimbal = model.registerBody(gimbal_name, gimbal_spatial_inertia,
                                             gimbal_parent_name, gimbal_Xtree);

            //
            const Mat3<Scalar> R_thigh = i == 0 ? R_left_thigh : R_right_thigh;
            const Vec3<Scalar> p_thigh = i == 0 ? p_left_thigh : p_right_thigh;
            const spatial::Transform<Scalar> thigh_Xtree = spatial::Transform(R_thigh, p_thigh);
            const std::string thigh_name = side + "-thigh";
            const std::string thigh_parent_name = side + "-gimbal";
            const SpatialInertia<Scalar> thigh_spatial_inertia =
                SpatialInertia<Scalar>{thigh_mass, thigh_CoM, thigh_inertia};
            auto thigh = model.registerBody(thigh_name, thigh_spatial_inertia,
                                            thigh_parent_name, thigh_Xtree);

            // Hip differential cluster
            std::vector<Body<Scalar>> bodies_in_hip_diff_cluster = {hip_rotor_1, hip_rotor_2,
                                                                    gimbal, thigh};

            const std::string hip_differential_cluster_name = side + "-hip-differential";
            const std::string hip_rotor1_joint_name = side + "-hip-clamp-to-hip-rotor-1";
            const std::string hip_rotor2_joint_name = side + "-hip-clamp-to-hip-rotor-2";
            const std::string gimbal_joint_name = side + "-hip-clamp-to-gimbal";
            const std::string thigh_joint_name = side + "-gimbal-to-thigh";
            TelloDifferentialModule<Scalar> hip_differential_module{
                hip_rotor_1, hip_rotor_2, gimbal, thigh,
                hip_rotor1_joint_name, hip_rotor2_joint_name, gimbal_joint_name, thigh_joint_name,
                ori::CoordinateAxis::Z, ori::CoordinateAxis::Z,
                ori::CoordinateAxis::X, ori::CoordinateAxis::Y, gear_ratio};

            std::vector<JointPtr<Scalar>> joints_in_hip_diff_cluster = {
                std::make_shared<RevJoint>(CoordAxis::Z, hip_rotor1_joint_name),
                std::make_shared<RevJoint>(CoordAxis::Z, hip_rotor2_joint_name),
                std::make_shared<RevJoint>(CoordAxis::X, gimbal_joint_name),
                std::make_shared<RevJoint>(CoordAxis::Y, thigh_joint_name)};

            std::function<DVec<casadi::SX>(const JointCoordinate<casadi::SX> &)>
                hip_diff_phi = [](const JointCoordinate<casadi::SX> &q)
            {
                double N = 6.0;
                DVec<casadi::SX> out = DVec<casadi::SX>(2);
                casadi::SX ql_1 = q(0);
                casadi::SX ql_2 = q(1);
                casadi::SX y_1 = q(2) / N;
                casadi::SX y_2 = q(3) / N;

                out[0] = (57 * sin(y_1)) / 2500 - (49 * cos(ql_1)) / 5000 - (399 * sin(ql_1)) / 20000 - (8 * cos(y_1) * cos(ql_2)) / 625 - (57 * cos(ql_1) * sin(ql_2)) / 2500 - (7 * sin(y_1) * sin(ql_1)) / 625 + (7 * sin(ql_1) * sin(ql_2)) / 625 - (8 * cos(ql_1) * sin(y_1) * sin(ql_2)) / 625 + 3021 / 160000;

                out[1] = (57 * sin(y_2)) / 2500 - (49 * cos(ql_1)) / 5000 + (399 * sin(ql_1)) / 20000 - (8 * cos(y_2) * cos(ql_2)) / 625 - (57 * cos(ql_1) * sin(ql_2)) / 2500 + (7 * sin(y_2) * sin(ql_1)) / 625 - (7 * sin(ql_1) * sin(ql_2)) / 625 - (8 * cos(ql_1) * sin(y_2) * sin(ql_2)) / 625 + 3021 / 160000;

                return out;
            };
            std::vector<bool> hip_diff_independent_coordinates = {true, true, false, false};

            std::shared_ptr<LoopConstraintType> hip_diff_loop_constraint;
            hip_diff_loop_constraint = std::make_shared<LoopConstraintType>(
                hip_diff_independent_coordinates, hip_diff_phi);

            model.template appendRegisteredBodiesAsCluster<ClusterJoints::Generic<Scalar>>(
                hip_differential_cluster_name, bodies_in_hip_diff_cluster,
                joints_in_hip_diff_cluster, hip_diff_loop_constraint);

            // Knee-ankle differential rotor 1
            const Mat3<Scalar> R_knee_ankle_rotor_1 = i == 0 ? R_left_knee_ankle_rotor_1
                                                             : R_right_knee_ankle_rotor_1;
            const Vec3<Scalar> p_knee_ankle_rotor_1 = i == 0 ? p_left_knee_ankle_rotor_1
                                                             : p_right_knee_ankle_rotor_1;
            const spatial::Transform<Scalar> knee_ankle_rotor_1_Xtree =
                spatial::Transform(R_knee_ankle_rotor_1, p_knee_ankle_rotor_1);
            const std::string knee_ankle_rotor_1_name = side + "-knee-ankle-rotor-1";
            const std::string knee_ankle_rotor_1_parent_name = side + "-thigh";
            const SpatialInertia<Scalar> knee_ankle_rotor_1_spatial_inertia =
                SpatialInertia<Scalar>{knee_ankle_rotor_1_mass, knee_ankle_rotor_1_CoM,
                                       knee_ankle_rotor_1_inertia};
            auto knee_ankle_rotor_1 = model.registerBody(knee_ankle_rotor_1_name,
                                                         knee_ankle_rotor_1_spatial_inertia,
                                                         knee_ankle_rotor_1_parent_name,
                                                         knee_ankle_rotor_1_Xtree);

            // Knee-ankle differential rotor 2
            const Mat3<Scalar> R_knee_ankle_rotor_2 = i == 0 ? R_left_knee_ankle_rotor_2
                                                             : R_right_knee_ankle_rotor_2;
            const Vec3<Scalar> p_knee_ankle_rotor_2 = i == 0 ? p_left_knee_ankle_rotor_2
                                                             : p_right_knee_ankle_rotor_2;
            const spatial::Transform<Scalar> knee_ankle_rotor_2_Xtree =
                spatial::Transform(R_knee_ankle_rotor_2, p_knee_ankle_rotor_2);
            const std::string knee_ankle_rotor_2_name = side + "-knee-ankle-rotor-2";
            const std::string knee_ankle_rotor_2_parent_name = side + "-thigh";
            const SpatialInertia<Scalar> knee_ankle_rotor_2_spatial_inertia =
                SpatialInertia<Scalar>{knee_ankle_rotor_2_mass, knee_ankle_rotor_2_CoM,
                                       knee_ankle_rotor_2_inertia};
            auto knee_ankle_rotor_2 = model.registerBody(knee_ankle_rotor_2_name,
                                                         knee_ankle_rotor_2_spatial_inertia,
                                                         knee_ankle_rotor_2_parent_name,
                                                         knee_ankle_rotor_2_Xtree);

            // Shin
            const Mat3<Scalar> R_shin = i == 0 ? R_left_shin : R_right_shin;
            const Vec3<Scalar> p_shin = i == 0 ? p_left_shin : p_right_shin;
            const spatial::Transform<Scalar> shin_Xtree = spatial::Transform(R_shin, p_shin);
            const std::string shin_name = side + "-shin";
            const std::string shin_parent_name = side + "-thigh";
            const SpatialInertia<Scalar> shin_spatial_inertia =
                SpatialInertia<Scalar>{shin_mass, shin_CoM, shin_inertia};
            auto shin = model.registerBody(shin_name, shin_spatial_inertia,
                                           shin_parent_name, shin_Xtree);

            // Foot
            const Mat3<Scalar> R_foot = i == 0 ? R_left_foot : R_right_foot;
            const Vec3<Scalar> p_foot = i == 0 ? p_left_foot : p_right_foot;
            const spatial::Transform<Scalar> foot_Xtree = spatial::Transform(R_foot, p_foot);
            const std::string foot_name = side + "-foot";
            const std::string foot_parent_name = side + "-shin";
            const SpatialInertia<Scalar> foot_spatial_inertia =
                SpatialInertia<Scalar>{foot_mass, foot_CoM, foot_inertia};
            auto foot = model.registerBody(foot_name, foot_spatial_inertia,
                                           foot_parent_name, foot_Xtree);

            // Knee-ankle differential cluster
            std::vector<Body<Scalar>> bodies_in_knee_ankle_diff_cluster = {knee_ankle_rotor_1,
                                                                           knee_ankle_rotor_2, shin, foot};

            const std::string knee_ankle_differential_cluster_name = side + "-knee-ankle-differential";
            const std::string knee_ankle_rotor1_joint_name = side + "-thigh-to-knee-ankle-rotor-1";
            const std::string knee_ankle_rotor2_joint_name = side + "-thigh-to-knee-ankle-rotor-2";
            const std::string shin_joint_name = side + "-thigh-to-shin";
            const std::string foot_joint_name = side + "-shin-to-foot";
            TelloDifferentialModule<Scalar> knee_ankle_module{
                knee_ankle_rotor_1, knee_ankle_rotor_2, shin, foot,
                knee_ankle_rotor1_joint_name, knee_ankle_rotor2_joint_name,
                shin_joint_name, foot_joint_name,
                ori::CoordinateAxis::Z, ori::CoordinateAxis::Z,
                ori::CoordinateAxis::Y, ori::CoordinateAxis::Y, gear_ratio};

            std::vector<JointPtr<Scalar>> joints_in_knee_ankle_diff_cluster = {
                std::make_shared<RevJoint>(CoordAxis::Z, knee_ankle_rotor1_joint_name),
                std::make_shared<RevJoint>(CoordAxis::Z, knee_ankle_rotor2_joint_name),
                std::make_shared<RevJoint>(CoordAxis::Y, shin_joint_name),
                std::make_shared<RevJoint>(CoordAxis::Y, foot_joint_name)};

            std::function<DVec<casadi::SX>(const JointCoordinate<casadi::SX> &)>
                knee_ankle_diff_phi = [](const JointCoordinate<casadi::SX> &q)
            {
                double N = 6.0;
                DVec<casadi::SX> out = DVec<casadi::SX>(2);
                casadi::SX ql_1 = q(0);
                casadi::SX ql_2 = q(1);
                casadi::SX y_1 = q(2) / N;
                casadi::SX y_2 = q(3) / N;

                out[0] = (21 * cos(y_1 / 2 - y_2 / 2 + (1979 * 3.1415) / 4500)) / 6250 - (13 * cos(y_1 / 2 - y_2 / 2 + (493 * 3.1415) / 1500)) / 625 - (273 * cos(3.1415 / 9)) / 12500 - (7 * sin(y_1 / 2 - y_2 / 2 + ql_2 + (231 * 3.1415) / 500)) / 2500 + (91 * sin(ql_2 + (2 * 3.1415) / 15)) / 5000 - (147 * sin(ql_2 + 3.1415 / 45)) / 50000 + 163349 / 6250000;

                out[1] = ql_1 - y_2 / 2 - y_1 / 2;

                return out;
            };
            std::vector<bool> knee_ankle_diff_independent_coordinates = {true, true, false, false};
            
            std::shared_ptr<LoopConstraintType> knee_ankle_diff_loop_constraint;
            knee_ankle_diff_loop_constraint = std::make_shared<LoopConstraintType>(
                knee_ankle_diff_independent_coordinates, knee_ankle_diff_phi);

            model.template appendRegisteredBodiesAsCluster<ClusterJoints::Generic<Scalar>>(
                knee_ankle_differential_cluster_name, bodies_in_knee_ankle_diff_cluster,
                joints_in_knee_ankle_diff_cluster, knee_ankle_diff_loop_constraint);

            // Append contact points for the feet
            const std::string toe_contact_name = side + "-toe_contact";
            const std::string heel_contact_name = side + "-heel_contact";
            if (i == 0)
                model.appendEndEffector(foot_name, Vec3<Scalar>(_footToeLength, 0, -_footHeight),
                                        toe_contact_name);
            else
                model.appendContactPoint(foot_name, Vec3<Scalar>(-_footToeLength, 0, -_footHeight),
                                         toe_contact_name);
            model.appendContactPoint(foot_name, Vec3<Scalar>(-_footHeelLength, 0, -_footHeight),
                                     heel_contact_name);
        }

        return model;
    }

    template class Tello<double>;
    template class Tello<casadi::SX>;

} // namespace grbda
