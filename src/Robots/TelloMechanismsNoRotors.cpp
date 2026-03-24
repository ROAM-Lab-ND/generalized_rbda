#include "grbda/Robots/TelloMechanismsNoRotors.hpp"

namespace grbda
{

    template <typename Scalar>
    ClusterTreeModel<Scalar> TelloMechanismsNoRotors<Scalar>::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;

        using RevJoint = Joints::Revolute<Scalar>;
        using CoordAxis = ori::CoordinateAxis;
        using LoopConstraintType = LoopConstraint::GenericImplicit<Scalar>;
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

        // Zero inertia for virtual rotor bodies (negligible mass to avoid singularity)
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

            // Virtual hip rotor 1 (zero inertia, provides joint coordinate for constraint)
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

            // Hip differential cluster - same structure as full Tello but with virtual rotors
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

            // Same constraint as full Tello - four-bar linkage relating rotors to links
            std::function<DVec<casadi::SX>(const JointCoordinate<casadi::SX> &)>
                hip_diff_phi = [](const JointCoordinate<casadi::SX> &q)
            {
                double N = 6.0;
                DVec<casadi::SX> out = DVec<casadi::SX>(2);
                // q(0), q(1) are independent (rotors), q(2), q(3) are dependent (links)
                casadi::SX y_1 = q(0) / N;  // rotor 1 post-gearbox (independent)
                casadi::SX y_2 = q(1) / N;  // rotor 2 post-gearbox (independent)
                casadi::SX ql_1 = q(2);     // gimbal angle (dependent)
                casadi::SX ql_2 = q(3);     // thigh angle (dependent)

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

            // Knee-ankle differential cluster - same structure as full Tello
            std::vector<Body<Scalar>> bodies_in_knee_ankle_diff_cluster = {knee_ankle_rotor_1,
                                                                           knee_ankle_rotor_2, shin, foot};

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

            // Same constraint as full Tello
            std::function<DVec<casadi::SX>(const JointCoordinate<casadi::SX> &)>
                knee_ankle_diff_phi = [](const JointCoordinate<casadi::SX> &q)
            {
                double N = 6.0;
                DVec<casadi::SX> out = DVec<casadi::SX>(2);
                // q(0), q(1) are independent (rotors), q(2), q(3) are dependent (links)
                casadi::SX y_1 = q(0) / N;  // rotor 1 post-gearbox (independent)
                casadi::SX y_2 = q(1) / N;  // rotor 2 post-gearbox (independent)
                casadi::SX ql_1 = q(2);     // shin angle (dependent)
                casadi::SX ql_2 = q(3);     // foot angle (dependent)

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

    template class TelloMechanismsNoRotors<double>;
    template class TelloMechanismsNoRotors<std::complex<double>>;
    template class TelloMechanismsNoRotors<casadi::SX>;

} // namespace grbda
