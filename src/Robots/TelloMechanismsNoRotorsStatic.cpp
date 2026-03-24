#include "grbda/Robots/TelloMechanismsNoRotorsStatic.hpp"

namespace grbda
{

    template <typename Scalar>
    ClusterTreeModel<Scalar> TelloMechanismsNoRotorsStatic<Scalar>::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;

        using RevJoint = Joints::Revolute<Scalar>;
        using CoordAxis = ori::CoordinateAxis;
        typedef spatial::Transform<Scalar> Xform;

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

        // Zero inertia for virtual rotor bodies (negligible mass)
        const Scalar virtual_mass = Scalar(1e-9);
        const Vec3<Scalar> virtual_CoM = Vec3<Scalar>::Zero();
        const Mat3<Scalar> virtual_inertia = Mat3<Scalar>::Identity() * Scalar(1e-12);
        const SpatialInertia<Scalar> virtual_rotor_inertia =
            SpatialInertia<Scalar>{virtual_mass, virtual_CoM, virtual_inertia};

        for (size_t i(0); i < 2; i++)
        {
            const std::string side = sides[i];

            // Hip clamp - plain Revolute (no rotor, matching TelloNoRotors)
            const Mat3<Scalar> R_hip_clamp = i == 0 ? this->R_left_hip_clamp : this->R_right_hip_clamp;
            const Vec3<Scalar> p_hip_clamp = i == 0 ? this->p_left_hip_clamp : this->p_right_hip_clamp;
            const Xform hip_clamp_Xtree = Xform(R_hip_clamp, p_hip_clamp);
            const std::string hip_clamp_name = side + "-hip-clamp";
            const SpatialInertia<Scalar> hip_clamp_spatial_inertia =
                SpatialInertia<Scalar>{this->hip_clamp_mass, this->hip_clamp_CoM, this->hip_clamp_inertia};
            const std::string hip_clamp_joint_name = this->base + "-to-" + side + "-hip-clamp";
            model.template appendBody<Revolute<Scalar>>(hip_clamp_name, hip_clamp_spatial_inertia,
                                                        hip_clamp_parent_name, hip_clamp_Xtree,
                                                        CoordAxis::Z, hip_clamp_joint_name);

            // Virtual hip rotor 1 (zero inertia, provides joint coordinate slot)
            const Mat3<Scalar> R_hip_rotor_1 = i == 0 ? this->R_left_hip_rotor_1 : this->R_right_hip_rotor_1;
            const Vec3<Scalar> p_hip_rotor_1 = i == 0 ? this->p_left_hip_rotor_1 : this->p_right_hip_rotor_1;
            const Xform hip_rotor_1_Xtree = Xform(R_hip_rotor_1, p_hip_rotor_1);
            const std::string hip_rotor_1_name = side + "-hip-rotor-1";
            const std::string hip_rotor_1_parent_name = side + "-hip-clamp";
            auto hip_rotor_1 = model.registerBody(hip_rotor_1_name, virtual_rotor_inertia,
                                                  hip_rotor_1_parent_name, hip_rotor_1_Xtree);

            // Virtual hip rotor 2 (zero inertia)
            const Mat3<Scalar> R_hip_rotor_2 = i == 0 ? this->R_left_hip_rotor_2 : this->R_right_hip_rotor_2;
            const Vec3<Scalar> p_hip_rotor_2 = i == 0 ? this->p_left_hip_rotor_2 : this->p_right_hip_rotor_2;
            const Xform hip_rotor_2_Xtree = Xform(R_hip_rotor_2, p_hip_rotor_2);
            const std::string hip_rotor_2_name = side + "-hip-rotor-2";
            const std::string hip_rotor_2_parent_name = side + "-hip-clamp";
            auto hip_rotor_2 = model.registerBody(hip_rotor_2_name, virtual_rotor_inertia,
                                                  hip_rotor_2_parent_name, hip_rotor_2_Xtree);

            // Gimbal
            const Mat3<Scalar> R_gimbal = i == 0 ? this->R_left_gimbal : this->R_right_gimbal;
            const Vec3<Scalar> p_gimbal = i == 0 ? this->p_left_gimbal : this->p_right_gimbal;
            const Xform gimbal_Xtree = Xform(R_gimbal, p_gimbal);
            const std::string gimbal_name = side + "-gimbal";
            const std::string gimbal_parent_name = side + "-hip-clamp";
            const SpatialInertia<Scalar> gimbal_spatial_inertia =
                SpatialInertia<Scalar>{this->gimbal_mass, this->gimbal_CoM, this->gimbal_inertia};
            auto gimbal = model.registerBody(gimbal_name, gimbal_spatial_inertia,
                                             gimbal_parent_name, gimbal_Xtree);

            // Thigh
            const Mat3<Scalar> R_thigh = i == 0 ? this->R_left_thigh : this->R_right_thigh;
            const Vec3<Scalar> p_thigh = i == 0 ? this->p_left_thigh : this->p_right_thigh;
            const Xform thigh_Xtree = Xform(R_thigh, p_thigh);
            const std::string thigh_name = side + "-thigh";
            const std::string thigh_parent_name = side + "-gimbal";
            const SpatialInertia<Scalar> thigh_spatial_inertia =
                SpatialInertia<Scalar>{this->thigh_mass, this->thigh_CoM, this->thigh_inertia};
            auto thigh = model.registerBody(thigh_name, thigh_spatial_inertia,
                                            thigh_parent_name, thigh_Xtree);

            // Hip differential cluster with STATIC constraints (constant gear ratio approximation)
            // Instead of complex four-bar linkage: q_gimbal ≈ gear_ratio * q_rotor1, q_thigh ≈ gear_ratio * q_rotor2
            std::vector<Body<Scalar>> bodies_in_hip_diff_cluster = {hip_rotor_1, hip_rotor_2,
                                                                    gimbal, thigh};

            const std::string hip_differential_cluster_name = side + "-hip-differential";
            const std::string hip_rotor1_joint_name = side + "-hip-clamp-to-hip-rotor-1";
            const std::string hip_rotor2_joint_name = side + "-hip-clamp-to-hip-rotor-2";
            const std::string gimbal_joint_name = side + "-hip-clamp-to-gimbal";
            const std::string thigh_joint_name = side + "-gimbal-to-thigh";

            std::vector<JointPtr<Scalar>> joints_in_hip_diff_cluster = {
                std::make_shared<RevJoint>(CoordAxis::Z, hip_rotor1_joint_name),
                std::make_shared<RevJoint>(CoordAxis::Z, hip_rotor2_joint_name),
                std::make_shared<RevJoint>(CoordAxis::X, gimbal_joint_name),
                std::make_shared<RevJoint>(CoordAxis::Y, thigh_joint_name)};

            // Static constraint matrices (linear approximation of differential)
            // Spanning coordinates: [y_1, y_2, q_gimbal, q_thigh]^T
            // Independent: [y_1, y_2]^T
            // G maps independent to spanning: [1 0; 0 1; gr 0; 0 gr] where gr = gear_ratio
            const Scalar gr = this->gear_ratio;
            DMat<Scalar> G_hip = DMat<Scalar>::Zero(4, 2);
            G_hip(0, 0) = 1.0;
            G_hip(1, 1) = 1.0;
            G_hip(2, 0) = gr;  // gimbal couples to rotor 1
            G_hip(3, 1) = gr;  // thigh couples to rotor 2

            // K relates spanning velocities: K such that K * spanning_v ≈ 0 (constraint manifold)
            DMat<Scalar> K_hip = DMat<Scalar>::Zero(2, 4);
            K_hip(0, 0) = -gr;
            K_hip(0, 2) = 1.0;
            K_hip(1, 1) = -gr;
            K_hip(1, 3) = 1.0;

            std::shared_ptr<LoopConstraint::Static<Scalar>> hip_diff_loop_constraint;
            hip_diff_loop_constraint = std::make_shared<LoopConstraint::Static<Scalar>>(G_hip, K_hip);

            model.template appendRegisteredBodiesAsCluster<ClusterJoints::Generic<Scalar>>(
                hip_differential_cluster_name, bodies_in_hip_diff_cluster,
                joints_in_hip_diff_cluster, hip_diff_loop_constraint);

            // Virtual knee-ankle rotor 1 (zero inertia)
            const Mat3<Scalar> R_knee_ankle_rotor_1 = i == 0 ? this->R_left_knee_ankle_rotor_1
                                                             : this->R_right_knee_ankle_rotor_1;
            const Vec3<Scalar> p_knee_ankle_rotor_1 = i == 0 ? this->p_left_knee_ankle_rotor_1
                                                             : this->p_right_knee_ankle_rotor_1;
            const Xform knee_ankle_rotor_1_Xtree = Xform(R_knee_ankle_rotor_1, p_knee_ankle_rotor_1);
            const std::string knee_ankle_rotor_1_name = side + "-knee-ankle-rotor-1";
            const std::string knee_ankle_rotor_1_parent_name = side + "-thigh";
            auto knee_ankle_rotor_1 = model.registerBody(knee_ankle_rotor_1_name,
                                                         virtual_rotor_inertia,
                                                         knee_ankle_rotor_1_parent_name,
                                                         knee_ankle_rotor_1_Xtree);

            // Virtual knee-ankle rotor 2 (zero inertia)
            const Mat3<Scalar> R_knee_ankle_rotor_2 = i == 0 ? this->R_left_knee_ankle_rotor_2
                                                             : this->R_right_knee_ankle_rotor_2;
            const Vec3<Scalar> p_knee_ankle_rotor_2 = i == 0 ? this->p_left_knee_ankle_rotor_2
                                                             : this->p_right_knee_ankle_rotor_2;
            const Xform knee_ankle_rotor_2_Xtree = Xform(R_knee_ankle_rotor_2, p_knee_ankle_rotor_2);
            const std::string knee_ankle_rotor_2_name = side + "-knee-ankle-rotor-2";
            const std::string knee_ankle_rotor_2_parent_name = side + "-thigh";
            auto knee_ankle_rotor_2 = model.registerBody(knee_ankle_rotor_2_name,
                                                         virtual_rotor_inertia,
                                                         knee_ankle_rotor_2_parent_name,
                                                         knee_ankle_rotor_2_Xtree);

            // Shin
            const Mat3<Scalar> R_shin = i == 0 ? this->R_left_shin : this->R_right_shin;
            const Vec3<Scalar> p_shin = i == 0 ? this->p_left_shin : this->p_right_shin;
            const Xform shin_Xtree = Xform(R_shin, p_shin);
            const std::string shin_name = side + "-shin";
            const std::string shin_parent_name = side + "-thigh";
            const SpatialInertia<Scalar> shin_spatial_inertia =
                SpatialInertia<Scalar>{this->shin_mass, this->shin_CoM, this->shin_inertia};
            auto shin = model.registerBody(shin_name, shin_spatial_inertia,
                                           shin_parent_name, shin_Xtree);

            // Foot
            const Mat3<Scalar> R_foot = i == 0 ? this->R_left_foot : this->R_right_foot;
            const Vec3<Scalar> p_foot = i == 0 ? this->p_left_foot : this->p_right_foot;
            const Xform foot_Xtree = Xform(R_foot, p_foot);
            const std::string foot_name = side + "-foot";
            const std::string foot_parent_name = side + "-shin";
            const SpatialInertia<Scalar> foot_spatial_inertia =
                SpatialInertia<Scalar>{this->foot_mass, this->foot_CoM, this->foot_inertia};
            auto foot = model.registerBody(foot_name, foot_spatial_inertia,
                                           foot_parent_name, foot_Xtree);

            // Knee-ankle differential cluster with STATIC constraints
            std::vector<Body<Scalar>> bodies_in_knee_ankle_diff_cluster = {knee_ankle_rotor_1,
                                                                           knee_ankle_rotor_2,
                                                                           shin,
                                                                           foot};

            const std::string knee_ankle_differential_cluster_name = side + "-knee-ankle-differential";
            const std::string knee_ankle_rotor1_joint_name = side + "-thigh-to-knee-ankle-rotor-1";
            const std::string knee_ankle_rotor2_joint_name = side + "-thigh-to-knee-ankle-rotor-2";
            const std::string shin_joint_name = side + "-thigh-to-shin";
            const std::string foot_joint_name = side + "-shin-to-foot";

            std::vector<JointPtr<Scalar>> joints_in_knee_ankle_diff_cluster = {
                std::make_shared<RevJoint>(CoordAxis::Z, knee_ankle_rotor1_joint_name),
                std::make_shared<RevJoint>(CoordAxis::Z, knee_ankle_rotor2_joint_name),
                std::make_shared<RevJoint>(CoordAxis::Y, shin_joint_name),
                std::make_shared<RevJoint>(CoordAxis::Y, foot_joint_name)};

            // Static constraint for knee-ankle (same structure as hip)
            DMat<Scalar> G_knee = DMat<Scalar>::Zero(4, 2);
            G_knee(0, 0) = 1.0;
            G_knee(1, 1) = 1.0;
            G_knee(2, 0) = gr;
            G_knee(3, 1) = gr;

            DMat<Scalar> K_knee = DMat<Scalar>::Zero(2, 4);
            K_knee(0, 0) = -gr;
            K_knee(0, 2) = 1.0;
            K_knee(1, 1) = -gr;
            K_knee(1, 3) = 1.0;

            std::shared_ptr<LoopConstraint::Static<Scalar>> knee_ankle_diff_loop_constraint;
            knee_ankle_diff_loop_constraint = std::make_shared<LoopConstraint::Static<Scalar>>(G_knee, K_knee);

            model.template appendRegisteredBodiesAsCluster<ClusterJoints::Generic<Scalar>>(
                knee_ankle_differential_cluster_name, bodies_in_knee_ankle_diff_cluster,
                joints_in_knee_ankle_diff_cluster, knee_ankle_diff_loop_constraint);
        }

        return model;
    }

    template class TelloMechanismsNoRotorsStatic<double>;
    template class TelloMechanismsNoRotorsStatic<std::complex<double>>;
    template class TelloMechanismsNoRotorsStatic<casadi::SX>;

} // namespace grbda
