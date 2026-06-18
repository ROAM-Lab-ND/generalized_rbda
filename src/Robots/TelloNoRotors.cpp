#include "grbda/Robots/TelloNoRotors.hpp"

namespace grbda
{

    template <typename Scalar>
    ClusterTreeModel<Scalar> TelloNoRotors<Scalar>::buildClusterTreeModel() const
    {
        using namespace ClusterJoints;
        typedef spatial::Transform<Scalar> Xform;

        ClusterTreeModel<Scalar> model{};

        // Set gravity in z direction
        model.setGravity(Vec3<Scalar>{0., 0., this->grav});

        // Torso (floating base)
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

            // Hip clamp - plain Revolute about Z
            const Mat3<Scalar> R_hip_clamp = i == 0 ? this->R_left_hip_clamp : this->R_right_hip_clamp;
            const Vec3<Scalar> p_hip_clamp = i == 0 ? this->p_left_hip_clamp : this->p_right_hip_clamp;
            const Xform hip_clamp_Xtree = Xform(R_hip_clamp, p_hip_clamp);
            const std::string hip_clamp_name = side + "-hip-clamp";
            const std::string hip_clamp_parent_name = this->base;
            const SpatialInertia<Scalar> hip_clamp_spatial_inertia =
                SpatialInertia<Scalar>{this->hip_clamp_mass, this->hip_clamp_CoM, this->hip_clamp_inertia};
            const std::string hip_clamp_joint_name = this->base + "-to-" + side + "-hip-clamp";
            model.template appendBody<Revolute<Scalar>>(hip_clamp_name, hip_clamp_spatial_inertia,
                                                        hip_clamp_parent_name, hip_clamp_Xtree,
                                                        ori::CoordinateAxis::Z, hip_clamp_joint_name);

            // Gimbal - plain Revolute about X
            const Mat3<Scalar> R_gimbal = i == 0 ? this->R_left_gimbal : this->R_right_gimbal;
            const Vec3<Scalar> p_gimbal = i == 0 ? this->p_left_gimbal : this->p_right_gimbal;
            const Xform gimbal_Xtree = Xform(R_gimbal, p_gimbal);
            const std::string gimbal_name = side + "-gimbal";
            const std::string gimbal_parent_name = side + "-hip-clamp";
            const SpatialInertia<Scalar> gimbal_spatial_inertia =
                SpatialInertia<Scalar>{this->gimbal_mass, this->gimbal_CoM, this->gimbal_inertia};
            const std::string gimbal_joint_name = side + "-hip-clamp-to-gimbal";
            model.template appendBody<Revolute<Scalar>>(gimbal_name, gimbal_spatial_inertia,
                                                        gimbal_parent_name, gimbal_Xtree,
                                                        ori::CoordinateAxis::X, gimbal_joint_name);

            // Thigh - plain Revolute about Y
            const Mat3<Scalar> R_thigh = i == 0 ? this->R_left_thigh : this->R_right_thigh;
            const Vec3<Scalar> p_thigh = i == 0 ? this->p_left_thigh : this->p_right_thigh;
            const Xform thigh_Xtree = Xform(R_thigh, p_thigh);
            const std::string thigh_name = side + "-thigh";
            const std::string thigh_parent_name = side + "-gimbal";
            const SpatialInertia<Scalar> thigh_spatial_inertia =
                SpatialInertia<Scalar>{this->thigh_mass, this->thigh_CoM, this->thigh_inertia};
            const std::string thigh_joint_name = side + "-gimbal-to-thigh";
            model.template appendBody<Revolute<Scalar>>(thigh_name, thigh_spatial_inertia,
                                                        thigh_parent_name, thigh_Xtree,
                                                        ori::CoordinateAxis::Y, thigh_joint_name);

            // Shin - plain Revolute about Y
            const Mat3<Scalar> R_shin = i == 0 ? this->R_left_shin : this->R_right_shin;
            const Vec3<Scalar> p_shin = i == 0 ? this->p_left_shin : this->p_right_shin;
            const Xform shin_Xtree = Xform(R_shin, p_shin);
            const std::string shin_name = side + "-shin";
            const std::string shin_parent_name = side + "-thigh";
            const SpatialInertia<Scalar> shin_spatial_inertia =
                SpatialInertia<Scalar>{this->shin_mass, this->shin_CoM, this->shin_inertia};
            const std::string shin_joint_name = side + "-thigh-to-shin";
            model.template appendBody<Revolute<Scalar>>(shin_name, shin_spatial_inertia,
                                                        shin_parent_name, shin_Xtree,
                                                        ori::CoordinateAxis::Y, shin_joint_name);

            // Foot - plain Revolute about Y
            const Mat3<Scalar> R_foot = i == 0 ? this->R_left_foot : this->R_right_foot;
            const Vec3<Scalar> p_foot = i == 0 ? this->p_left_foot : this->p_right_foot;
            const Xform foot_Xtree = Xform(R_foot, p_foot);
            const std::string foot_name = side + "-foot";
            const std::string foot_parent_name = side + "-shin";
            const SpatialInertia<Scalar> foot_spatial_inertia =
                SpatialInertia<Scalar>{this->foot_mass, this->foot_CoM, this->foot_inertia};
            const std::string foot_joint_name = side + "-shin-to-foot";
            model.template appendBody<Revolute<Scalar>>(foot_name, foot_spatial_inertia,
                                                        foot_parent_name, foot_Xtree,
                                                        ori::CoordinateAxis::Y, foot_joint_name);

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

    template class TelloNoRotors<double>;
    template class TelloNoRotors<std::complex<double>>;
    template class TelloNoRotors<casadi::SX>;

} // namespace grbda
