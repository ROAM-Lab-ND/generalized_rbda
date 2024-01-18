#ifndef GRBDA_ROBOTS_SINGLE_LEG_HOPPER_H
#define GRBDA_ROBOTS_SINGLE_LEG_HOPPER_H

#include "grbda/Robots/Robot.h"

namespace grbda
{

    // 
    // TODO(@MatthewChignoli): Template on Scalar
    // TODO(@MatthewChignoli): This is the robot from 2.74. Maybe the name should reflect that. Although it is not floating base. Also I should get parameters for the robot from the homework/matlab code. And then unit test it against the matlab code.
    class SingleLegHopper : public Robot<double>
    {
    public:
        SingleLegHopper() 
        {
            // TODO(@MatthewChignoli): Get correct values later

            // Thigh
            Mat3<double> thigh_rotiatonal_inertia;
            thigh_rotiatonal_inertia << 0.1, 0., 0.,
                0., 0.1, 0.,
                0., 0., 0.1;
            Vec3<double> thigh_com{0., 0., 0.};
            thigh_spatial_inertia_ = SpatialInertia<double>(1., thigh_com, thigh_rotiatonal_inertia);

            // Shank Driver
            Mat3<double> shank_driver_rotiatonal_inertia;
            shank_driver_rotiatonal_inertia << 0.1, 0., 0.,
                0., 0.1, 0.,
                0., 0., 0.1;
            Vec3<double> shank_driver_com{0., 0., 0.};
            shank_driver_spatial_inertia_ = SpatialInertia<double>(1., shank_driver_com, shank_driver_rotiatonal_inertia);

            // Shank Support
            Mat3<double> shank_support_rotiatonal_inertia;
            shank_support_rotiatonal_inertia << 0.1, 0., 0.,
                0., 0.1, 0.,
                0., 0., 0.1;
            Vec3<double> shank_support_com{0., 0., 0.};
            shank_support_spatial_inertia_ = SpatialInertia<double>(1., shank_support_com, shank_support_rotiatonal_inertia);

            // Foot
            Mat3<double> foot_rotiatonal_inertia;
            foot_rotiatonal_inertia << 0.1, 0., 0.,
                0., 0.1, 0.,
                0., 0., 0.1;
            Vec3<double> foot_com{0., 0., 0.};
            foot_spatial_inertia_ = SpatialInertia<double>(1., foot_com, foot_rotiatonal_inertia);

        }

        ClusterTreeModel<> buildClusterTreeModel() const override
        {
            using SpatialTransform = spatial::Transform<>;
            Mat3<double> I3 = Mat3<double>::Identity();
            ori::CoordinateAxis common_axis = ori::CoordinateAxis::Z;

            ClusterTreeModel<> model{};

            // Thigh
            SpatialTransform thigh_Xtree{I3, thigh_location_};
            Body<> thigh = model.registerBody(thigh_name_, thigh_spatial_inertia_,
                                              thigh_parent_name_, thigh_Xtree);
            using RevoluteCluster = ClusterJoints::Revolute<>;
            model.appendRegisteredBodiesAsCluster<RevoluteCluster>(thigh_name_, thigh, common_axis);

            // Shank Driver
            SpatialTransform shank_driver_Xtree{I3, shank_driver_location_};
            Body<> shank_driver = model.registerBody(
                shank_driver_name_, shank_driver_spatial_inertia_,
                shank_driver_parent_name_, shank_driver_Xtree);
            JointPtr<double> driver_joint = std::make_shared<Joints::Revolute<>>(common_axis);

            // Shank Support
            SpatialTransform shank_support_Xtree{I3, shank_support_location_};
            Body<> shank_support = model.registerBody(
                shank_support_name_, shank_support_spatial_inertia_,
                shank_support_parent_name_, shank_support_Xtree);
            JointPtr<double> support_joint = std::make_shared<Joints::Revolute<>>(common_axis);

            // Foot
            SpatialTransform foot_Xtree{I3, foot_location_};
            Body<> foot = model.registerBody(foot_name_, foot_spatial_inertia_,
                                             foot_parent_name_, foot_Xtree);
            JointPtr<double> foot_joint = std::make_shared<Joints::Revolute<>>(common_axis);

            // Loop Constraint
            std::vector<double> path1_link_lengths, path2_link_lengths;
            path1_link_lengths.push_back(foot_location_[0]);
            path1_link_lengths.push_back(foot_joint_to_constraint_length_);
            path2_link_lengths.push_back(support_joint_to_constraint_length_);
            Vec2<double> offset{shank_driver_location_[0] - shank_support_location_[0], 0.};
            int indepedent_coord = 0.;
            std::shared_ptr<LoopConstraint::FourBar<double>> loop_constraint =
                std::make_shared<LoopConstraint::FourBar<double>>(path1_link_lengths, path2_link_lengths, offset, indepedent_coord);

            // Lower Leg Cluster
            std::vector<Body<double>> lower_leg_bodies = {shank_driver, shank_support, foot};
            std::vector<JointPtr<double>> lower_leg_joints = {driver_joint, support_joint, foot_joint};
            using FourBarCluster = ClusterJoints::FourBar<double>;
            model.appendRegisteredBodiesAsCluster<FourBarCluster>(
                lower_leg_cluster_name_, lower_leg_bodies, lower_leg_joints, loop_constraint);

            return model;
        }

    private:
        // Thigh Cluster
        const std::string thigh_cluster_name_ = "thigh-cluster";

        // Thigh
        const std::string thigh_name_ = "thigh";
        const std::string thigh_parent_name_ = "ground";
        Vec3<double> thigh_location_{0., 0., 0.};
        SpatialInertia<double> thigh_spatial_inertia_;

        // Lower Leg Cluster
        const std::string lower_leg_cluster_name_ = "lower-leg-cluster";

        // Shank Driver
        const std::string shank_driver_name_ = "shank-driver";
        const std::string shank_driver_parent_name_ = "thigh";
        Vec3<double> shank_driver_location_{0.5, 0., 0.};
        SpatialInertia<double> shank_driver_spatial_inertia_;

        // Shank Support
        const std::string shank_support_name_ = "shank-support";
        const std::string shank_support_parent_name_ = "thigh";
        Vec3<double> shank_support_location_{2.5, 0., 0.};
        SpatialInertia<double> shank_support_spatial_inertia_;

        // Foot
        const std::string foot_name_ = "foot";
        const std::string foot_parent_name_ = "shank-driver";
        Vec3<double> foot_location_{4.5, 0., 0.};
        SpatialInertia<double> foot_spatial_inertia_;

        // TODO(@MatthewChignoli): Better way to encode this? Probably using xtree so that it can match the URDF?
        const double foot_joint_to_constraint_length_ = 5.;
        const double support_joint_to_constraint_length_ = 3.;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_SINGLE_LEG_HOPPER_H
