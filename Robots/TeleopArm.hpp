#ifndef GRBDA_ROBOTS_TELEOP_ARM_H
#define GRBDA_ROBOTS_TELEOP_ARM_H

#include "Robot.h"

namespace grbda
{

    class TeleopArm : public Robot<double>
    {
    public:
        TeleopArm();

        ClusterTreeModel<> buildClusterTreeModel() const override;

    private:
        // Base Cluster
        const std::string base_cluster_name_ = "base-cluster";

        // Base
        const std::string base_name_ = "base";
        const std::string base_parent_name_ = "ground";
        Vec3<double> base_location_{0, 0, 0.051};
        SpatialInertia<double> base_spatial_inertia_;

        // Base Rotor
        const std::string base_rotor_name_ = "base-rotor";
        const std::string base_rotor_parent_name_ = "ground";
        Vec3<double> base_rotor_location_{0, 0, 0};
        SpatialInertia<double> base_rotor_spatial_inertia_;
        double base_rotor_gear_ratio_ = 6.0;

        // Shoulder Rx Cluster
        const std::string shoulder_rx_cluster_name_ = "shoulder-rx-cluster";

        // Shoulder Rx
        const std::string shoulder_rx_link_name_ = "shoulder-rx-link";
        const std::string shoulder_rx_link_parent_name_ = "base";
        Vec3<double> shoulder_rx_link_location_{0, 0, 0.106};
        SpatialInertia<double> shoulder_rx_link_spatial_inertia_;

        // Shoulder Rx Rotor
        const std::string shoulder_rx_rotor_name_ = "shoulder-rx-rotor";
        const std::string shoulder_rx_rotor_parent_name_ = "base";
        Vec3<double> shoulder_rx_rotor_location_{0, 0, 0};
        SpatialInertia<double> shoulder_rx_rotor_spatial_inertia_;
        double shoulder_rx_rotor_gear_ratio_ = 6.0;

        // Shoulder Ry Cluster
        const std::string shoulder_ry_cluster_name_ = "shoulder-ry-cluster";

        // Shoulder Ry
        const std::string shoulder_ry_link_name_ = "shoulder-ry-link";
        const std::string shoulder_ry_link_parent_name_ = "shoulder-rx-link";
        Vec3<double> shoulder_ry_link_location_{0, 0, 0.071};
        SpatialInertia<double> shoulder_ry_link_spatial_inertia_;

        // Shoulder Ry Rotor
        const std::string shoulder_ry_rotor_name_ = "shoulder-ry-rotor";
        const std::string shoulder_ry_rotor_parent_name_ = "shoulder-rx-link";
        Vec3<double> shoulder_ry_rotor_location_{0, 0, 0};
        SpatialInertia<double> shoulder_ry_rotor_spatial_inertia_;
        double shoulder_ry_rotor_gear_ratio_ = 6.0;

        // Upper Arm Cluster
        const std::string upper_arm_cluster_name_ = "upper-arm-cluster";

        // Upper Link
        const std::string upper_link_name_ = "upper-link";
        const std::string upper_link_parent_name_ = "shoulder-ry-link";
        const Vec3<double> upper_link_location_{0, -0.0095, 0.3855};
        SpatialInertia<double> upper_link_spatial_inertia_;

        // Wrist Pitch Link
        const std::string wrist_pitch_link_name_ = "wrist-pitch-link";
        const std::string wrist_pitch_link_parent_name_ = "upper-link";
        const Vec3<double> wrist_pitch_link_location_{0, 0, 0.362};
        SpatialInertia<double> wrist_pitch_link_spatial_inertia_;

        // Wrist Roll Link
        const std::string wrist_roll_link_name_ = "wrist-roll-link";
        const std::string wrist_roll_link_parent_name_ = "wrist-pitch-link";
        const Vec3<double> wrist_roll_link_location_{0, 0.004, 0.03574};
        SpatialInertia<double> wrist_roll_link_spatial_inertia_;

        // Elbow Rotor
        const std::string elbow_rotor_name_ = "elbow-rotor";
        const std::string elbow_rotor_parent_name_ = "shoulder-ry-link";
        const Vec3<double> elbow_rotor_location_{0., 0., 0.};
        SpatialInertia<double> elbow_rotor_spatial_inertia_;
        const double elbow_rotor_gear_ratio_ = 6.0;
        const double elbow_rotor_belt_ratio_ = 1.0;

        // Wrist Pitch Rotor
        const std::string wrist_pitch_rotor_name_ = "wrist-pitch-rotor";
        const std::string wrist_pitch_rotor_parent_name_ = "shoulder-ry-link";
        const Vec3<double> wrist_pitch_rotor_location_{0., 0., 0.};
        SpatialInertia<double> wrist_pitch_rotor_spatial_inertia_;
        const double wrist_pitch_rotor_gear_ratio_ = 6.0;
        const double wrist_pitch_rotor_belt_ratio_ = 1.0;

        // Wrist Roll Rotor
        const std::string wrist_roll_rotor_name_ = "wrist-roll-rotor";
        const std::string wrist_roll_rotor_parent_name_ = "shoulder-ry-link";
        const Vec3<double> wrist_roll_rotor_location_{0., 0., 0.};
        SpatialInertia<double> wrist_roll_rotor_spatial_inertia_;
        const double wrist_roll_rotor_gear_ratio_ = 6.0;
        const double wrist_roll_rotor_belt_ratio_ = 1.0;

        // Gripper Cluster
        const std::string gripper_cluster_name_ = "gripper-cluster";

        // Gripper
        const std::string gripper_name_ = "gripper";
        const std::string gripper_parent_name_ = "wrist-roll-link";
        Vec3<double> gripper_location_{0.0004, 0.0375, 0.070995};
        SpatialInertia<double> gripper_spatial_inertia_;

        // Gripper Rotor
        const std::string gripper_rotor_name_ = "gripper-rotor";
        const std::string gripper_rotor_parent_name_ = "wrist-roll-link";
        Vec3<double> gripper_rotor_location_{0, 0, 0};
        SpatialInertia<double> gripper_rotor_spatial_inertia_;
        double gripper_rotor_gear_ratio_ = 2.0;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_TELEOP_ARM_H
