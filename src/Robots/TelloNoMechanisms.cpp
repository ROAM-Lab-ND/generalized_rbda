#include "grbda/Robots/TelloNoMechanisms.hpp"

namespace grbda
{

    template <typename Scalar>
    ClusterTreeModel<Scalar> TelloNoMechanisms<Scalar>::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;
        typedef spatial::Transform<Scalar> Xform;
        typedef GearedTransmissionModule<Scalar> TransmissionModule;
        typedef RevoluteWithRotor<Scalar> RevWithRotor;

        ClusterTreeModel<Scalar> model{};

        // Set gravity in z direction
        model.setGravity(Vec3<Scalar>{0., 0., this->grav});

        // Torso
        const std::string torso_name = this->base;
        const std::string torso_parent_name = "ground";
        const SpatialInertia<Scalar> torso_spatial_inertia =
            SpatialInertia<Scalar>{this->torso_mass, this->torso_CoM, this->torso_inertia};
        model.template appendBody<Free<Scalar>>(torso_name, torso_spatial_inertia,
                                                torso_parent_name, Xform{},
                                                "torso-to-ground");

        std::vector<std::string> sides = {"left", "right"};
        const std::string hip_clamp_parent_name = this->base;
        const std::string hip_clamp_rotor_parent_name = this->base;

        for (size_t i(0); i < 2; i++)
        {
            const std::string side = sides[i];

            // Hip clamp (same as full Tello - RevoluteWithRotor)
            const Mat3<Scalar> R_hip_clamp = i == 0 ? this->R_left_hip_clamp : this->R_right_hip_clamp;
            const Vec3<Scalar> p_hip_clamp = i == 0 ? this->p_left_hip_clamp : this->p_right_hip_clamp;
            const Xform hip_clamp_Xtree = Xform(R_hip_clamp, p_hip_clamp);
            const std::string hip_clamp_name = side + "-hip-clamp";
            const SpatialInertia<Scalar> hip_clamp_spatial_inertia =
                SpatialInertia<Scalar>{this->hip_clamp_mass, this->hip_clamp_CoM, this->hip_clamp_inertia};
            auto hip_clamp = model.registerBody(hip_clamp_name, hip_clamp_spatial_inertia,
                                                hip_clamp_parent_name, hip_clamp_Xtree);

            // Hip clamp rotor
            const Mat3<Scalar> R_hip_clamp_rotor = i == 0 ? this->R_left_hip_clamp_rotor
                                                          : this->R_right_hip_clamp_rotor;
            const Vec3<Scalar> p_hip_clamp_rotor = i == 0 ? this->p_left_hip_clamp_rotor
                                                          : this->p_right_hip_clamp_rotor;
            const Xform hip_clamp_rotor_Xtree = Xform(R_hip_clamp_rotor, p_hip_clamp_rotor);
            const std::string hip_clamp_rotor_name = side + "-hip-clamp-rotor";
            const SpatialInertia<Scalar> hip_clamp_rotor_spatial_inertia =
                SpatialInertia<Scalar>{this->hip_clamp_rotor_mass, this->hip_clamp_rotor_CoM,
                                       this->hip_clamp_rotor_inertia};
            auto hip_clamp_rotor = model.registerBody(hip_clamp_rotor_name,
                                                      hip_clamp_rotor_spatial_inertia,
                                                      hip_clamp_rotor_parent_name,
                                                      hip_clamp_rotor_Xtree);

            // Hip clamp cluster
            const std::string hip_clamp_cluster_name = side + "-hip-clamp";
            const std::string hip_clamp_joint_name = this->base + "-to-" + side + "-hip-clamp";
            const std::string hip_clamp_rotor_joint_name = this->base + "-to-" + side + "-hip-clamp-rotor";
            TransmissionModule hip_clamp_module{hip_clamp, hip_clamp_rotor,
                                                hip_clamp_joint_name,
                                                hip_clamp_rotor_joint_name,
                                                ori::CoordinateAxis::Z,
                                                ori::CoordinateAxis::Z,
                                                this->gear_ratio};
            model.template appendRegisteredBodiesAsCluster<RevWithRotor>(
                hip_clamp_cluster_name, hip_clamp_module);

            // Gimbal with rotor
            // Both gimbal and hip_rotor_1 have parent hip-clamp (same parent cluster)
            const Mat3<Scalar> R_gimbal = i == 0 ? this->R_left_gimbal : this->R_right_gimbal;
            const Vec3<Scalar> p_gimbal = i == 0 ? this->p_left_gimbal : this->p_right_gimbal;
            const Xform gimbal_Xtree = Xform(R_gimbal, p_gimbal);
            const std::string gimbal_name = side + "-gimbal";
            const std::string gimbal_parent_name = side + "-hip-clamp";
            const SpatialInertia<Scalar> gimbal_spatial_inertia =
                SpatialInertia<Scalar>{this->gimbal_mass, this->gimbal_CoM, this->gimbal_inertia};
            auto gimbal = model.registerBody(gimbal_name, gimbal_spatial_inertia,
                                             gimbal_parent_name, gimbal_Xtree);

            // Hip rotor 1 (for gimbal)
            const Mat3<Scalar> R_hip_rotor_1 = i == 0 ? this->R_left_hip_rotor_1 : this->R_right_hip_rotor_1;
            const Vec3<Scalar> p_hip_rotor_1 = i == 0 ? this->p_left_hip_rotor_1 : this->p_right_hip_rotor_1;
            const Xform hip_rotor_1_Xtree = Xform(R_hip_rotor_1, p_hip_rotor_1);
            const std::string hip_rotor_1_name = side + "-hip-rotor-1";
            const std::string hip_rotor_1_parent_name = side + "-hip-clamp";
            const SpatialInertia<Scalar> hip_rotor_1_spatial_inertia =
                SpatialInertia<Scalar>{this->hip_rotor_1_mass, this->hip_rotor_1_CoM, this->hip_rotor_1_inertia};
            auto hip_rotor_1 = model.registerBody(hip_rotor_1_name, hip_rotor_1_spatial_inertia,
                                                  hip_rotor_1_parent_name, hip_rotor_1_Xtree);

            // Gimbal cluster (RevoluteWithRotor)
            const std::string gimbal_cluster_name = side + "-gimbal";
            const std::string gimbal_joint_name = side + "-hip-clamp-to-gimbal";
            const std::string hip_rotor1_joint_name = side + "-hip-clamp-to-hip-rotor-1";
            TransmissionModule gimbal_module{gimbal, hip_rotor_1,
                                             gimbal_joint_name, hip_rotor1_joint_name,
                                             ori::CoordinateAxis::X, ori::CoordinateAxis::Z,
                                             this->gear_ratio};
            model.template appendRegisteredBodiesAsCluster<RevWithRotor>(
                gimbal_cluster_name, gimbal_module);

            // Thigh with rotor
            // In the no-mechanisms version, place rotor on gimbal (same parent cluster as thigh)
            const Mat3<Scalar> R_thigh = i == 0 ? this->R_left_thigh : this->R_right_thigh;
            const Vec3<Scalar> p_thigh = i == 0 ? this->p_left_thigh : this->p_right_thigh;
            const Xform thigh_Xtree = Xform(R_thigh, p_thigh);
            const std::string thigh_name = side + "-thigh";
            const std::string thigh_parent_name = side + "-gimbal";
            const SpatialInertia<Scalar> thigh_spatial_inertia =
                SpatialInertia<Scalar>{this->thigh_mass, this->thigh_CoM, this->thigh_inertia};
            auto thigh = model.registerBody(thigh_name, thigh_spatial_inertia,
                                            thigh_parent_name, thigh_Xtree);

            // Hip rotor 2 (for thigh) - now on gimbal instead of hip-clamp
            // This ensures both thigh and its rotor have parent in the gimbal cluster
            const Mat3<Scalar> R_hip_rotor_2 = i == 0 ? this->R_left_hip_rotor_2 : this->R_right_hip_rotor_2;
            const Vec3<Scalar> p_hip_rotor_2 = i == 0 ? this->p_left_hip_rotor_2 : this->p_right_hip_rotor_2;
            const Xform hip_rotor_2_Xtree = Xform(R_hip_rotor_2, p_hip_rotor_2);
            const std::string hip_rotor_2_name = side + "-hip-rotor-2";
            const std::string hip_rotor_2_parent_name = side + "-gimbal";  // Changed from hip-clamp
            const SpatialInertia<Scalar> hip_rotor_2_spatial_inertia =
                SpatialInertia<Scalar>{this->hip_rotor_2_mass, this->hip_rotor_2_CoM, this->hip_rotor_2_inertia};
            auto hip_rotor_2 = model.registerBody(hip_rotor_2_name, hip_rotor_2_spatial_inertia,
                                                  hip_rotor_2_parent_name, hip_rotor_2_Xtree);

            // Thigh cluster (RevoluteWithRotor)
            const std::string thigh_cluster_name = side + "-thigh";
            const std::string thigh_joint_name = side + "-gimbal-to-thigh";
            const std::string hip_rotor2_joint_name = side + "-gimbal-to-hip-rotor-2";
            TransmissionModule thigh_module{thigh, hip_rotor_2,
                                            thigh_joint_name, hip_rotor2_joint_name,
                                            ori::CoordinateAxis::Y, ori::CoordinateAxis::Z,
                                            this->gear_ratio};
            model.template appendRegisteredBodiesAsCluster<RevWithRotor>(
                thigh_cluster_name, thigh_module);

            // Shin with rotor
            // Both shin and knee_ankle_rotor_1 have parent thigh (same parent cluster)
            const Mat3<Scalar> R_shin = i == 0 ? this->R_left_shin : this->R_right_shin;
            const Vec3<Scalar> p_shin = i == 0 ? this->p_left_shin : this->p_right_shin;
            const Xform shin_Xtree = Xform(R_shin, p_shin);
            const std::string shin_name = side + "-shin";
            const std::string shin_parent_name = side + "-thigh";
            const SpatialInertia<Scalar> shin_spatial_inertia =
                SpatialInertia<Scalar>{this->shin_mass, this->shin_CoM, this->shin_inertia};
            auto shin = model.registerBody(shin_name, shin_spatial_inertia,
                                           shin_parent_name, shin_Xtree);

            // Knee-ankle rotor 1 (for shin)
            const Mat3<Scalar> R_knee_ankle_rotor_1 = i == 0 ? this->R_left_knee_ankle_rotor_1
                                                             : this->R_right_knee_ankle_rotor_1;
            const Vec3<Scalar> p_knee_ankle_rotor_1 = i == 0 ? this->p_left_knee_ankle_rotor_1
                                                             : this->p_right_knee_ankle_rotor_1;
            const Xform knee_ankle_rotor_1_Xtree = Xform(R_knee_ankle_rotor_1, p_knee_ankle_rotor_1);
            const std::string knee_ankle_rotor_1_name = side + "-knee-ankle-rotor-1";
            const std::string knee_ankle_rotor_1_parent_name = side + "-thigh";
            const SpatialInertia<Scalar> knee_ankle_rotor_1_spatial_inertia =
                SpatialInertia<Scalar>{this->knee_ankle_rotor_1_mass, this->knee_ankle_rotor_1_CoM,
                                       this->knee_ankle_rotor_1_inertia};
            auto knee_ankle_rotor_1 = model.registerBody(knee_ankle_rotor_1_name,
                                                         knee_ankle_rotor_1_spatial_inertia,
                                                         knee_ankle_rotor_1_parent_name,
                                                         knee_ankle_rotor_1_Xtree);

            // Shin cluster (RevoluteWithRotor)
            const std::string shin_cluster_name = side + "-shin";
            const std::string shin_joint_name = side + "-thigh-to-shin";
            const std::string knee_ankle_rotor1_joint_name = side + "-thigh-to-knee-ankle-rotor-1";
            TransmissionModule shin_module{shin, knee_ankle_rotor_1,
                                           shin_joint_name, knee_ankle_rotor1_joint_name,
                                           ori::CoordinateAxis::Y, ori::CoordinateAxis::Z,
                                           this->gear_ratio};
            model.template appendRegisteredBodiesAsCluster<RevWithRotor>(
                shin_cluster_name, shin_module);

            // Foot with rotor
            // Place rotor on shin (same parent cluster as foot)
            const Mat3<Scalar> R_foot = i == 0 ? this->R_left_foot : this->R_right_foot;
            const Vec3<Scalar> p_foot = i == 0 ? this->p_left_foot : this->p_right_foot;
            const Xform foot_Xtree = Xform(R_foot, p_foot);
            const std::string foot_name = side + "-foot";
            const std::string foot_parent_name = side + "-shin";
            const SpatialInertia<Scalar> foot_spatial_inertia =
                SpatialInertia<Scalar>{this->foot_mass, this->foot_CoM, this->foot_inertia};
            auto foot = model.registerBody(foot_name, foot_spatial_inertia,
                                           foot_parent_name, foot_Xtree);

            // Knee-ankle rotor 2 (for foot) - on shin instead of thigh
            const Mat3<Scalar> R_knee_ankle_rotor_2 = i == 0 ? this->R_left_knee_ankle_rotor_2
                                                             : this->R_right_knee_ankle_rotor_2;
            const Vec3<Scalar> p_knee_ankle_rotor_2 = i == 0 ? this->p_left_knee_ankle_rotor_2
                                                             : this->p_right_knee_ankle_rotor_2;
            const Xform knee_ankle_rotor_2_Xtree = Xform(R_knee_ankle_rotor_2, p_knee_ankle_rotor_2);
            const std::string knee_ankle_rotor_2_name = side + "-knee-ankle-rotor-2";
            const std::string knee_ankle_rotor_2_parent_name = side + "-shin";  // Changed from thigh
            const SpatialInertia<Scalar> knee_ankle_rotor_2_spatial_inertia =
                SpatialInertia<Scalar>{this->knee_ankle_rotor_2_mass, this->knee_ankle_rotor_2_CoM,
                                       this->knee_ankle_rotor_2_inertia};
            auto knee_ankle_rotor_2 = model.registerBody(knee_ankle_rotor_2_name,
                                                         knee_ankle_rotor_2_spatial_inertia,
                                                         knee_ankle_rotor_2_parent_name,
                                                         knee_ankle_rotor_2_Xtree);

            // Foot cluster (RevoluteWithRotor)
            const std::string foot_cluster_name = side + "-foot";
            const std::string foot_joint_name = side + "-shin-to-foot";
            const std::string knee_ankle_rotor2_joint_name = side + "-shin-to-knee-ankle-rotor-2";
            TransmissionModule foot_module{foot, knee_ankle_rotor_2,
                                           foot_joint_name, knee_ankle_rotor2_joint_name,
                                           ori::CoordinateAxis::Y, ori::CoordinateAxis::Z,
                                           this->gear_ratio};
            model.template appendRegisteredBodiesAsCluster<RevWithRotor>(
                foot_cluster_name, foot_module);

            // Append contact points for the feet
            const std::string toe_contact_name = side + "-toe_contact";
            const std::string heel_contact_name = side + "-heel_contact";
            if (i == 0)
                model.appendEndEffector(foot_name, Vec3<Scalar>(this->_footToeLength, 0, -this->_footHeight),
                                        toe_contact_name);
            else
                model.appendContactPoint(foot_name, Vec3<Scalar>(-this->_footToeLength, 0, -this->_footHeight),
                                         toe_contact_name);
            model.appendContactPoint(foot_name, Vec3<Scalar>(-this->_footHeelLength, 0, -this->_footHeight),
                                     heel_contact_name);
        }

        return model;
    }

    template class TelloNoMechanisms<double>;
    template class TelloNoMechanisms<std::complex<double>>;
    template class TelloNoMechanisms<casadi::SX>;

} // namespace grbda
