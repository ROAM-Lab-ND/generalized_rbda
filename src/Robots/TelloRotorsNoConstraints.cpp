#include "grbda/Robots/TelloRotorsNoConstraints.hpp"

namespace grbda
{

    template <typename Scalar>
    ClusterTreeModel<Scalar> TelloRotorsNoConstraints<Scalar>::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;
        typedef spatial::Transform<Scalar> Xform;
        typedef GearedTransmissionModule<Scalar> GearedTransModule;
        typedef RevoluteWithRotor<Scalar> RevoluteWithRotor;

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

        for (size_t i(0); i < 2; i++)
        {
            const std::string side = sides[i];

            // ===== Hip Clamp RevoluteWithRotor Cluster =====
            const Mat3<Scalar> R_hip_clamp = i == 0 ? this->R_left_hip_clamp : this->R_right_hip_clamp;
            const Vec3<Scalar> p_hip_clamp = i == 0 ? this->p_left_hip_clamp : this->p_right_hip_clamp;
            const Xform hip_clamp_Xtree = Xform(R_hip_clamp, p_hip_clamp);
            const std::string hip_clamp_name = side + "-hip-clamp";
            const SpatialInertia<Scalar> hip_clamp_spatial_inertia =
                SpatialInertia<Scalar>{this->hip_clamp_mass, this->hip_clamp_CoM, this->hip_clamp_inertia};
            auto hip_clamp = model.registerBody(hip_clamp_name, hip_clamp_spatial_inertia,
                                                this->base, hip_clamp_Xtree);

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
                                                      this->base, hip_clamp_rotor_Xtree);

            const std::string hip_clamp_cluster_name = side + "-hip-clamp";
            const std::string hip_clamp_joint_name = this->base + "-to-" + side + "-hip-clamp";
            const std::string hip_clamp_rotor_joint_name = this->base + "-to-" + side + "-hip-clamp-rotor";
            GearedTransModule hip_clamp_module{hip_clamp, hip_clamp_rotor,
                                               hip_clamp_joint_name,
                                               hip_clamp_rotor_joint_name,
                                               ori::CoordinateAxis::Z,
                                               ori::CoordinateAxis::Z,
                                               this->gear_ratio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(
                hip_clamp_cluster_name, hip_clamp_module);

            // ===== Gimbal RevoluteWithRotor Cluster (INDEPENDENT) =====
            const Mat3<Scalar> R_gimbal = i == 0 ? this->R_left_gimbal : this->R_right_gimbal;
            const Vec3<Scalar> p_gimbal = i == 0 ? this->p_left_gimbal : this->p_right_gimbal;
            const Xform gimbal_Xtree = Xform(R_gimbal, p_gimbal);
            const std::string gimbal_name = side + "-gimbal";
            const SpatialInertia<Scalar> gimbal_spatial_inertia =
                SpatialInertia<Scalar>{this->gimbal_mass, this->gimbal_CoM, this->gimbal_inertia};
            auto gimbal = model.registerBody(gimbal_name, gimbal_spatial_inertia,
                                             hip_clamp_name, gimbal_Xtree);

            // Gimbal rotor
            const Mat3<Scalar> R_gimbal_rotor = i == 0 ? this->R_left_hip_rotor_1 : this->R_right_hip_rotor_1;
            const Vec3<Scalar> p_gimbal_rotor = i == 0 ? this->p_left_hip_rotor_1 : this->p_right_hip_rotor_1;
            const Xform gimbal_rotor_Xtree = Xform(R_gimbal_rotor, p_gimbal_rotor);
            const std::string gimbal_rotor_name = side + "-gimbal-rotor";
            const SpatialInertia<Scalar> gimbal_rotor_spatial_inertia =
                SpatialInertia<Scalar>{this->hip_rotor_1_mass, this->hip_rotor_1_CoM,
                                       this->hip_rotor_1_inertia};
            auto gimbal_rotor = model.registerBody(gimbal_rotor_name, gimbal_rotor_spatial_inertia,
                                                   hip_clamp_name, gimbal_rotor_Xtree);

            const std::string gimbal_cluster_name = side + "-gimbal";
            const std::string gimbal_joint_name = hip_clamp_name + "-to-" + side + "-gimbal";
            const std::string gimbal_rotor_joint_name = hip_clamp_name + "-to-" + side + "-gimbal-rotor";
            GearedTransModule gimbal_module{gimbal, gimbal_rotor,
                                            gimbal_joint_name,
                                            gimbal_rotor_joint_name,
                                            ori::CoordinateAxis::X,
                                            ori::CoordinateAxis::X,
                                            this->gear_ratio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(
                gimbal_cluster_name, gimbal_module);

            // ===== Thigh RevoluteWithRotor Cluster (INDEPENDENT) =====
            const Mat3<Scalar> R_thigh = i == 0 ? this->R_left_thigh : this->R_right_thigh;
            const Vec3<Scalar> p_thigh = i == 0 ? this->p_left_thigh : this->p_right_thigh;
            const Xform thigh_Xtree = Xform(R_thigh, p_thigh);
            const std::string thigh_name = side + "-thigh";
            const SpatialInertia<Scalar> thigh_spatial_inertia =
                SpatialInertia<Scalar>{this->thigh_mass, this->thigh_CoM, this->thigh_inertia};
            auto thigh = model.registerBody(thigh_name, thigh_spatial_inertia,
                                            gimbal_name, thigh_Xtree);

            // Thigh rotor
            const Mat3<Scalar> R_thigh_rotor = i == 0 ? this->R_left_hip_rotor_2 : this->R_right_hip_rotor_2;
            const Vec3<Scalar> p_thigh_rotor = i == 0 ? this->p_left_hip_rotor_2 : this->p_right_hip_rotor_2;
            const Xform thigh_rotor_Xtree = Xform(R_thigh_rotor, p_thigh_rotor);
            const std::string thigh_rotor_name = side + "-thigh-rotor";
            const SpatialInertia<Scalar> thigh_rotor_spatial_inertia =
                SpatialInertia<Scalar>{this->hip_rotor_2_mass, this->hip_rotor_2_CoM,
                                       this->hip_rotor_2_inertia};
            auto thigh_rotor = model.registerBody(thigh_rotor_name, thigh_rotor_spatial_inertia,
                                                  gimbal_name, thigh_rotor_Xtree);

            const std::string thigh_cluster_name = side + "-thigh";
            const std::string thigh_joint_name = gimbal_name + "-to-" + side + "-thigh";
            const std::string thigh_rotor_joint_name = gimbal_name + "-to-" + side + "-thigh-rotor";
            GearedTransModule thigh_module{thigh, thigh_rotor,
                                           thigh_joint_name,
                                           thigh_rotor_joint_name,
                                           ori::CoordinateAxis::Y,
                                           ori::CoordinateAxis::Y,
                                           this->gear_ratio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(
                thigh_cluster_name, thigh_module);

            // ===== Shin RevoluteWithRotor Cluster (INDEPENDENT) =====
            const Mat3<Scalar> R_shin = i == 0 ? this->R_left_shin : this->R_right_shin;
            const Vec3<Scalar> p_shin = i == 0 ? this->p_left_shin : this->p_right_shin;
            const Xform shin_Xtree = Xform(R_shin, p_shin);
            const std::string shin_name = side + "-shin";
            const SpatialInertia<Scalar> shin_spatial_inertia =
                SpatialInertia<Scalar>{this->shin_mass, this->shin_CoM, this->shin_inertia};
            auto shin = model.registerBody(shin_name, shin_spatial_inertia,
                                           thigh_name, shin_Xtree);

            // Shin rotor
            const Mat3<Scalar> R_shin_rotor = i == 0 ? this->R_left_knee_ankle_rotor_1
                                                      : this->R_right_knee_ankle_rotor_1;
            const Vec3<Scalar> p_shin_rotor = i == 0 ? this->p_left_knee_ankle_rotor_1
                                                      : this->p_right_knee_ankle_rotor_1;
            const Xform shin_rotor_Xtree = Xform(R_shin_rotor, p_shin_rotor);
            const std::string shin_rotor_name = side + "-shin-rotor";
            const SpatialInertia<Scalar> shin_rotor_spatial_inertia =
                SpatialInertia<Scalar>{this->knee_ankle_rotor_1_mass, this->knee_ankle_rotor_1_CoM,
                                       this->knee_ankle_rotor_1_inertia};
            auto shin_rotor = model.registerBody(shin_rotor_name, shin_rotor_spatial_inertia,
                                                 thigh_name, shin_rotor_Xtree);

            const std::string shin_cluster_name = side + "-shin";
            const std::string shin_joint_name = thigh_name + "-to-" + side + "-shin";
            const std::string shin_rotor_joint_name = thigh_name + "-to-" + side + "-shin-rotor";
            GearedTransModule shin_module{shin, shin_rotor,
                                          shin_joint_name,
                                          shin_rotor_joint_name,
                                          ori::CoordinateAxis::Y,
                                          ori::CoordinateAxis::Y,
                                          this->gear_ratio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(
                shin_cluster_name, shin_module);

            // ===== Foot RevoluteWithRotor Cluster (INDEPENDENT) =====
            const Mat3<Scalar> R_foot = i == 0 ? this->R_left_foot : this->R_right_foot;
            const Vec3<Scalar> p_foot = i == 0 ? this->p_left_foot : this->p_right_foot;
            const Xform foot_Xtree = Xform(R_foot, p_foot);
            const std::string foot_name = side + "-foot";
            const SpatialInertia<Scalar> foot_spatial_inertia =
                SpatialInertia<Scalar>{this->foot_mass, this->foot_CoM, this->foot_inertia};
            auto foot = model.registerBody(foot_name, foot_spatial_inertia,
                                           shin_name, foot_Xtree);

            // Foot rotor
            const Mat3<Scalar> R_foot_rotor = i == 0 ? this->R_left_knee_ankle_rotor_2
                                                      : this->R_right_knee_ankle_rotor_2;
            const Vec3<Scalar> p_foot_rotor = i == 0 ? this->p_left_knee_ankle_rotor_2
                                                      : this->p_right_knee_ankle_rotor_2;
            const Xform foot_rotor_Xtree = Xform(R_foot_rotor, p_foot_rotor);
            const std::string foot_rotor_name = side + "-foot-rotor";
            const SpatialInertia<Scalar> foot_rotor_spatial_inertia =
                SpatialInertia<Scalar>{this->knee_ankle_rotor_2_mass, this->knee_ankle_rotor_2_CoM,
                                       this->knee_ankle_rotor_2_inertia};
            auto foot_rotor = model.registerBody(foot_rotor_name, foot_rotor_spatial_inertia,
                                                 shin_name, foot_rotor_Xtree);

            const std::string foot_cluster_name = side + "-foot";
            const std::string foot_joint_name = shin_name + "-to-" + side + "-foot";
            const std::string foot_rotor_joint_name = shin_name + "-to-" + side + "-foot-rotor";
            GearedTransModule foot_module{foot, foot_rotor,
                                          foot_joint_name,
                                          foot_rotor_joint_name,
                                          ori::CoordinateAxis::Y,
                                          ori::CoordinateAxis::Y,
                                          this->gear_ratio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(
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

    template class TelloRotorsNoConstraints<double>;
    template class TelloRotorsNoConstraints<std::complex<double>>;
    template class TelloRotorsNoConstraints<casadi::SX>;

} // namespace grbda
