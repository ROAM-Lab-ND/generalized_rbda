#ifndef GRBDA_ROBOTS_TELLO_LEG_H
#define GRBDA_ROBOTS_TELLO_LEG_H

#include "grbda/Robots/Tello.hpp"

namespace grbda
{

    class TelloLeg : public Tello
    {
    public:
        ClusterTreeModel<> buildClusterTreeModel() const override
        {
            using namespace ClusterJoints;
            typedef spatial::Transform<> Xform;

            ClusterTreeModel<> model{};

            // Hip clamp
            const std::string hip_clamp_parent_name = "ground";
            const std::string hip_clamp_rotor_parent_name = "ground";
            const Mat3<double> R_hip_clamp = R_right_hip_clamp;
            const Vec3<double> p_hip_clamp = p_right_hip_clamp;
            const Xform hip_clamp_Xtree = spatial::Transform(R_hip_clamp, p_hip_clamp);
            const std::string hip_clamp_name = "hip-clamp";
            const SpatialInertia<double> hip_clamp_spatial_inertia =
                SpatialInertia<double>{hip_clamp_mass, hip_clamp_CoM, hip_clamp_inertia};
            auto hip_clamp = model.registerBody(hip_clamp_name, hip_clamp_spatial_inertia,
                                                hip_clamp_parent_name, hip_clamp_Xtree);

            // Hip clamp rotor
            const Mat3<double> R_hip_clamp_rotor = R_right_hip_clamp_rotor;
            const Vec3<double> p_hip_clamp_rotor = p_right_hip_clamp_rotor;
            const Xform hip_clamp_rotor_Xtree = spatial::Transform(R_hip_clamp_rotor,
                                                                   p_hip_clamp_rotor);
            const std::string hip_clamp_rotor_name = "hip-clamp-rotor";
            const SpatialInertia<double> hip_clamp_rotor_spatial_inertia =
                SpatialInertia<double>{hip_clamp_rotor_mass, hip_clamp_rotor_CoM,
                                       hip_clamp_rotor_inertia};
            auto hip_clamp_rotor = model.registerBody(hip_clamp_rotor_name,
                                                      hip_clamp_rotor_spatial_inertia,
                                                      hip_clamp_rotor_parent_name,
                                                      hip_clamp_rotor_Xtree);

            // Hip clamp cluster
            const std::string hip_clamp_cluster_name = "hip-clamp";
            GearedTransmissionModule<> hip_clamp_module{hip_clamp, hip_clamp_rotor,
                                                        ori::CoordinateAxis::Z,
                                                        ori::CoordinateAxis::Z,
                                                        gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor<>>(hip_clamp_cluster_name,
                                                                       hip_clamp_module);

            // Hip differential rotor 1
            const Mat3<double> R_hip_rotor_1 = R_right_hip_rotor_1;
            const Vec3<double> p_hip_rotor_1 = p_right_hip_rotor_1;
            const Xform hip_rotor_1_Xtree = spatial::Transform(R_hip_rotor_1, p_hip_rotor_1);
            const std::string hip_rotor_1_name = "hip-rotor-1";
            const std::string hip_rotor_1_parent_name = "hip-clamp";
            const SpatialInertia<double> hip_rotor_1_spatial_inertia =
                SpatialInertia<double>{hip_rotor_1_mass, hip_rotor_1_CoM, hip_rotor_1_inertia};
            auto hip_rotor_1 = model.registerBody(hip_rotor_1_name, hip_rotor_1_spatial_inertia,
                                                  hip_rotor_1_parent_name, hip_rotor_1_Xtree);

            // Hip differential rotor 2
            const Mat3<double> R_hip_rotor_2 = R_right_hip_rotor_2;
            const Vec3<double> p_hip_rotor_2 = p_right_hip_rotor_2;
            const Xform hip_rotor_2_Xtree = spatial::Transform(R_hip_rotor_2, p_hip_rotor_2);
            const std::string hip_rotor_2_name = "hip-rotor-2";
            const std::string hip_rotor_2_parent_name = "hip-clamp";
            const SpatialInertia<double> hip_rotor_2_spatial_inertia =
                SpatialInertia<double>{hip_rotor_2_mass, hip_rotor_2_CoM, hip_rotor_2_inertia};
            auto hip_rotor_2 = model.registerBody(hip_rotor_2_name, hip_rotor_2_spatial_inertia,
                                                  hip_rotor_2_parent_name, hip_rotor_2_Xtree);

            // Gimbal
            const Mat3<double> R_gimbal = R_right_gimbal;
            const Vec3<double> p_gimbal = p_right_gimbal;
            const Xform gimbal_Xtree = spatial::Transform(R_gimbal, p_gimbal);
            const std::string gimbal_name = "gimbal";
            const std::string gimbal_parent_name = "hip-clamp";
            const SpatialInertia<double> gimbal_spatial_inertia =
                SpatialInertia<double>{gimbal_mass, gimbal_CoM, gimbal_inertia};
            auto gimbal = model.registerBody(gimbal_name, gimbal_spatial_inertia,
                                             gimbal_parent_name, gimbal_Xtree);

            //
            const Mat3<double> R_thigh = R_right_thigh;
            const Vec3<double> p_thigh = p_right_thigh;
            const Xform thigh_Xtree = spatial::Transform(R_thigh, p_thigh);
            const std::string thigh_name = "thigh";
            const std::string thigh_parent_name = "gimbal";
            const SpatialInertia<double> thigh_spatial_inertia =
                SpatialInertia<double>{thigh_mass, thigh_CoM, thigh_inertia};
            auto thigh = model.registerBody(thigh_name, thigh_spatial_inertia,
                                            thigh_parent_name, thigh_Xtree);

            // Hip differential cluster
            const std::string hip_differential_cluster_name = "hip-differential";
            TelloDifferentialModule<> hip_differential_module{
                hip_rotor_1, hip_rotor_2, gimbal, thigh,
                ori::CoordinateAxis::Z, ori::CoordinateAxis::Z,
                ori::CoordinateAxis::X, ori::CoordinateAxis::Y, gear_ratio};
            model.appendRegisteredBodiesAsCluster<TelloHipDifferential<double>>(
                hip_differential_cluster_name, hip_differential_module);

            // Knee-ankle differential rotor 1
            const Mat3<double> R_knee_ankle_rotor_1 = R_right_knee_ankle_rotor_1;
            const Vec3<double> p_knee_ankle_rotor_1 = p_right_knee_ankle_rotor_1;
            const Xform knee_ankle_rotor_1_Xtree =
                spatial::Transform(R_knee_ankle_rotor_1, p_knee_ankle_rotor_1);
            const std::string knee_ankle_rotor_1_name = "knee-ankle-rotor-1";
            const std::string knee_ankle_rotor_1_parent_name = "thigh";
            const SpatialInertia<double> knee_ankle_rotor_1_spatial_inertia = SpatialInertia<double>{
                knee_ankle_rotor_1_mass, knee_ankle_rotor_1_CoM, knee_ankle_rotor_1_inertia};
            auto knee_ankle_rotor_1 = model.registerBody(knee_ankle_rotor_1_name,
                                                         knee_ankle_rotor_1_spatial_inertia,
                                                         knee_ankle_rotor_1_parent_name,
                                                         knee_ankle_rotor_1_Xtree);

            // Knee-ankle differential rotor 2
            const Mat3<double> R_knee_ankle_rotor_2 = R_right_knee_ankle_rotor_2;
            const Vec3<double> p_knee_ankle_rotor_2 = p_right_knee_ankle_rotor_2;
            const Xform knee_ankle_rotor_2_Xtree =
                spatial::Transform(R_knee_ankle_rotor_2, p_knee_ankle_rotor_2);
            const std::string knee_ankle_rotor_2_name = "knee-ankle-rotor-2";
            const std::string knee_ankle_rotor_2_parent_name = "thigh";
            const SpatialInertia<double> knee_ankle_rotor_2_spatial_inertia = SpatialInertia<double>{
                knee_ankle_rotor_2_mass, knee_ankle_rotor_2_CoM, knee_ankle_rotor_2_inertia};
            auto knee_ankle_rotor_2 = model.registerBody(knee_ankle_rotor_2_name,
                                                         knee_ankle_rotor_2_spatial_inertia,
                                                         knee_ankle_rotor_2_parent_name,
                                                         knee_ankle_rotor_2_Xtree);

            // Shin
            const Mat3<double> R_shin = R_right_shin;
            const Vec3<double> p_shin = p_right_shin;
            const Xform shin_Xtree = spatial::Transform(R_shin, p_shin);
            const std::string shin_name = "shin";
            const std::string shin_parent_name = "thigh";
            const SpatialInertia<double> shin_spatial_inertia =
                SpatialInertia<double>{shin_mass, shin_CoM, shin_inertia};
            auto shin = model.registerBody(shin_name, shin_spatial_inertia,
                                           shin_parent_name, shin_Xtree);

            // Foot
            const Mat3<double> R_foot = R_right_foot;
            const Vec3<double> p_foot = p_right_foot;
            const Xform foot_Xtree = spatial::Transform(R_foot, p_foot);
            const std::string foot_name = "foot";
            const std::string foot_parent_name = "shin";
            const SpatialInertia<double> foot_spatial_inertia =
                SpatialInertia<double>{foot_mass, foot_CoM, foot_inertia};
            auto foot = model.registerBody(foot_name, foot_spatial_inertia,
                                           foot_parent_name, foot_Xtree);

            // Knee-ankle differential cluster
            const std::string knee_ankle_differential_cluster_name = "knee-ankle-differential";
            TelloDifferentialModule<> knee_ankle_module{
                knee_ankle_rotor_1, knee_ankle_rotor_2, shin, foot,
                ori::CoordinateAxis::Z, ori::CoordinateAxis::Z,
                ori::CoordinateAxis::Y, ori::CoordinateAxis::Y, gear_ratio};
            model.appendRegisteredBodiesAsCluster<TelloKneeAnkleDifferential<double>>(
                knee_ankle_differential_cluster_name, knee_ankle_module);

            // Append contact points for the feet
            const std::string toe_contact_name = "toe_contact";
            const std::string heel_contact_name = "heel_contact";
            model.appendContactPoint(foot_name, Vec3<double>(-_footToeLength, 0, -_footHeight),
                                     toe_contact_name);
            model.appendContactPoint(foot_name, Vec3<double>(-_footHeelLength, 0, -_footHeight),
                                     heel_contact_name);

            return model;
        }
    };
} // namespace grbda

#endif // GRBDA_ROBOTS_TELLO_LEG_H
