#include "Tello.hpp"

namespace grbda
{

    ClusterTreeModel Tello::buildClusterTreeModel() const
    {
        ClusterTreeModel model{};

        // Set gravity in z direction
        model.setGravity(Vec3<double>{0., 0., grav});

        // Torso
        const std::string torso_name = "torso";
        const std::string torso_parent_name = "ground";
        const SpatialInertia<double> torso_spatial_inertia = SpatialInertia<double>{torso_mass,
            torso_CoM, torso_inertia};
        auto torso = model.registerBody(torso_name, torso_spatial_inertia, torso_parent_name,
                                        SpatialTransform{});
        auto torso_generalized_joint = std::make_shared<GeneralizedJoints::Free>(torso);
        model.appendRegisteredBodiesAsCluster(torso_name, torso_generalized_joint);

        std::vector<std::string> sides = {"left","right"};
        const std::string hip_clamp_parent_name = "torso";
        const std::string hip_clamp_rotor_parent_name = "torso";

        for (size_t i(0); i < 2; i++)
        {
            const std::string side = sides[i];

            // Hip clamp
            const SpatialTransform hip_clamp_Xtree = i == 0 ? SpatialTransform(R_left_hip_clamp,
                                                                               p_left_hip_clamp)
                                                            : SpatialTransform(R_right_hip_clamp,
                                                                               p_right_hip_clamp);
            const std::string hip_clamp_name = side + "-hip-clamp";
            const SpatialInertia<double> hip_clamp_spatial_inertia =
                SpatialInertia<double>{hip_clamp_mass, hip_clamp_CoM, hip_clamp_inertia};
            auto hip_clamp = model.registerBody(hip_clamp_name, hip_clamp_spatial_inertia,
                                                hip_clamp_parent_name, hip_clamp_Xtree);

            // Hip clamp rotor
            const Mat3<double> R_hip_clamp_rotor = i == 0 ? R_left_hip_clamp_rotor
                                                          : R_right_hip_clamp_rotor;
            const Vec3<double> p_hip_clamp_rotor = i == 0 ? p_left_hip_clamp_rotor
                                                          : p_right_hip_clamp_rotor;
            const SpatialTransform hip_clamp_rotor_Xtree = SpatialTransform(R_hip_clamp_rotor,
                                                                            p_hip_clamp_rotor);
            const std::string hip_clamp_rotor_name = side + "-hip-clamp-rotor";
            const SpatialInertia<double> hip_clamp_rotor_spatial_inertia =
                SpatialInertia<double>{hip_clamp_rotor_mass, hip_clamp_rotor_CoM,
                                       hip_clamp_rotor_inertia};
            auto hip_clamp_rotor = model.registerBody(hip_clamp_rotor_name,
                                                      hip_clamp_rotor_spatial_inertia,
                                                      hip_clamp_rotor_parent_name,
                                                      hip_clamp_rotor_Xtree);

            // Hip clamp cluster
            const std::string hip_clamp_cluster_name = side + "-hip-clamp";
            auto hip_clamp_generalized_joint = std::make_shared<GeneralizedJoints::RevoluteWithRotor>(
                hip_clamp, hip_clamp_rotor, CoordinateAxis::Z, CoordinateAxis::Z, gear_ratio);
            model.appendRegisteredBodiesAsCluster(hip_clamp_cluster_name, hip_clamp_generalized_joint);

            // Hip differential rotor 1
            const SpatialTransform hip_rotor_1_Xtree = i == 0 ? SpatialTransform(R_left_hip_rotor_1,
                                                                                 p_left_hip_rotor_1)
                                                              : SpatialTransform(R_right_hip_rotor_1,
                                                                                 p_right_hip_rotor_1);
            const std::string hip_rotor_1_name = side + "-hip-rotor-1";
            const std::string hip_rotor_1_parent_name = side + "-hip-clamp";
            const SpatialInertia<double> hip_rotor_1_spatial_inertia =
                SpatialInertia<double>{hip_rotor_1_mass, hip_rotor_1_CoM, hip_rotor_1_inertia};
            auto hip_rotor_1 = model.registerBody(hip_rotor_1_name, hip_rotor_1_spatial_inertia,
                                                  hip_rotor_1_parent_name, hip_rotor_1_Xtree);

            // Hip differential rotor 2
            const SpatialTransform hip_rotor_2_Xtree = i == 0 ? SpatialTransform(R_left_hip_rotor_2,
                                                                                 p_left_hip_rotor_2)
                                                              : SpatialTransform(R_right_hip_rotor_2,
                                                                                 p_right_hip_rotor_2);
            const std::string hip_rotor_2_name = side + "-hip-rotor-2";
            const std::string hip_rotor_2_parent_name = side + "-hip-clamp";
            const SpatialInertia<double> hip_rotor_2_spatial_inertia =
                SpatialInertia<double>{hip_rotor_2_mass, hip_rotor_2_CoM, hip_rotor_2_inertia};
            auto hip_rotor_2 = model.registerBody(hip_rotor_2_name, hip_rotor_2_spatial_inertia,
                                                  hip_rotor_2_parent_name, hip_rotor_2_Xtree);

            // Gimbal
            const SpatialTransform gimbal_Xtree = i == 0 ? SpatialTransform(R_left_gimbal,
                                                                            p_left_gimbal)
                                                         : SpatialTransform(R_right_gimbal,
                                                                            p_right_gimbal);
            const std::string gimbal_name = side + "-gimbal";
            const std::string gimbal_parent_name = side + "-hip-clamp";
            const SpatialInertia<double> gimbal_spatial_inertia =
                SpatialInertia<double>{gimbal_mass, gimbal_CoM, gimbal_inertia};
            auto gimbal = model.registerBody(gimbal_name, gimbal_spatial_inertia,
                                             gimbal_parent_name, gimbal_Xtree);

            // Thigh
            const SpatialTransform thigh_Xtree = i == 0 ? SpatialTransform(R_left_thigh,
                                                                           p_left_thigh)
                                                        : SpatialTransform(R_right_thigh,
                                                                           p_right_thigh);
            const std::string thigh_name = side + "-thigh";
            const std::string thigh_parent_name = side + "-gimbal";
            const SpatialInertia<double> thigh_spatial_inertia =
                SpatialInertia<double>{thigh_mass, thigh_CoM, thigh_inertia};
            auto thigh = model.registerBody(thigh_name, thigh_spatial_inertia,
                                            thigh_parent_name, thigh_Xtree);

            // Hip differential cluster
            const std::string hip_differential_cluster_name = side + "-hip-differential";
            auto hip_differential_generalized_joint = std::make_shared<GeneralizedJoints::TelloHipDifferential>(
                hip_rotor_1, hip_rotor_2, gimbal, thigh, CoordinateAxis::Z, CoordinateAxis::Z,
                CoordinateAxis::X, CoordinateAxis::Y, gear_ratio);
            model.appendRegisteredBodiesAsCluster(hip_differential_cluster_name,
                                                  hip_differential_generalized_joint);

            // Knee-ankle differential rotor 1
            const Mat3<double> R_knee_ankle_rotor_1 = i == 0 ? R_left_knee_ankle_rotor_1
                                                             : R_right_knee_ankle_rotor_1;
            const Vec3<double> p_knee_ankle_rotor_1 = i == 0 ? p_left_knee_ankle_rotor_1
                                                             : p_right_knee_ankle_rotor_1;
            const SpatialTransform knee_ankle_rotor_1_Xtree = SpatialTransform(R_knee_ankle_rotor_1,
                                                                               p_knee_ankle_rotor_1);
            const std::string knee_ankle_rotor_1_name = side + "-knee-ankle-rotor-1";
            const std::string knee_ankle_rotor_1_parent_name = side + "-thigh";
            const SpatialInertia<double> knee_ankle_rotor_1_spatial_inertia = SpatialInertia<double>{
                knee_ankle_rotor_1_mass, knee_ankle_rotor_1_CoM, knee_ankle_rotor_1_inertia};
            auto knee_ankle_rotor_1 = model.registerBody(knee_ankle_rotor_1_name,
                                                         knee_ankle_rotor_1_spatial_inertia,
                                                         knee_ankle_rotor_1_parent_name,
                                                         knee_ankle_rotor_1_Xtree);

            // Knee-ankle differential rotor 2
            const Mat3<double> R_knee_ankle_rotor_2 = i == 0 ? R_left_knee_ankle_rotor_2
                                                             : R_right_knee_ankle_rotor_2;
            const Vec3<double> p_knee_ankle_rotor_2 = i == 0 ? p_left_knee_ankle_rotor_2
                                                             : p_right_knee_ankle_rotor_2;
            const SpatialTransform knee_ankle_rotor_2_Xtree = SpatialTransform(R_knee_ankle_rotor_2,
                                                                               p_knee_ankle_rotor_2);
            const std::string knee_ankle_rotor_2_name = side + "-knee-ankle-rotor-2";
            const std::string knee_ankle_rotor_2_parent_name = side + "-thigh";
            const SpatialInertia<double> knee_ankle_rotor_2_spatial_inertia = SpatialInertia<double>{
                knee_ankle_rotor_2_mass, knee_ankle_rotor_2_CoM, knee_ankle_rotor_2_inertia};
            auto knee_ankle_rotor_2 = model.registerBody(knee_ankle_rotor_2_name,
                                                         knee_ankle_rotor_2_spatial_inertia,
                                                         knee_ankle_rotor_2_parent_name,
                                                         knee_ankle_rotor_2_Xtree);

            // Shin
            const SpatialTransform shin_Xtree = i == 0 ? SpatialTransform(R_left_shin, p_left_shin)
                                                       : SpatialTransform(R_right_shin, p_right_shin);
            const std::string shin_name = side + "-shin";
            const std::string shin_parent_name = side + "-thigh";
            const SpatialInertia<double> shin_spatial_inertia =
                SpatialInertia<double>{shin_mass, shin_CoM, shin_inertia};
            auto shin = model.registerBody(shin_name, shin_spatial_inertia,
                                           shin_parent_name, shin_Xtree);

            // Foot
            const SpatialTransform foot_Xtree = i == 0 ? SpatialTransform(R_left_foot, p_left_foot)
                                                       : SpatialTransform(R_right_foot, p_right_foot);
            const std::string foot_name = side + "-foot";
            const std::string foot_parent_name = side + "-shin";
            const SpatialInertia<double> foot_spatial_inertia =
                SpatialInertia<double>{foot_mass, foot_CoM, foot_inertia};
            auto foot = model.registerBody(foot_name, foot_spatial_inertia,
                                           foot_parent_name, foot_Xtree);

            // Hip differential cluster
            const std::string knee_ankle_differential_cluster_name = side + "-knee-ankle-differential";
            auto knee_ankle_differential_generalized_joint = std::make_shared<GeneralizedJoints::TelloKneeAnkleDifferential>(
                knee_ankle_rotor_1, knee_ankle_rotor_2, shin, foot,
                CoordinateAxis::Z, CoordinateAxis::Z, CoordinateAxis::Y, CoordinateAxis::Y,
                gear_ratio);
            model.appendRegisteredBodiesAsCluster(knee_ankle_differential_cluster_name,
                                                  knee_ankle_differential_generalized_joint);

            // Append contact points for the feet
            const std::string toe_contact_name = side + "-toe_contact";
            const std::string heel_contact_name = side + "-heel_contact";
            model.appendContactPoint(foot_name, Vec3<double>(_footToeLength, 0, -_footHeight),
                                     toe_contact_name);
            model.appendContactPoint(foot_name, Vec3<double>(-_footHeelLength, 0, -_footHeight),
                                     heel_contact_name);
        }

        return model;
    }

} // namespace grbda
