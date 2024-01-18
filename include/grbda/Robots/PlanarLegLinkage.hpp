#ifndef GRBDA_ROBOTS_SINGLE_LEG_HOPPER_H
#define GRBDA_ROBOTS_SINGLE_LEG_HOPPER_H

#include "grbda/Robots/Robot.h"

namespace grbda
{
    template <typename Scalar = double>
    class PlanarLegLinkage : public Robot<Scalar>
    {
    public:
        PlanarLegLinkage();
        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;

    private:
        // Thigh Cluster
        const std::string thigh_cluster_name_ = "thigh-cluster";

        // Thigh
        const std::string thigh_name_ = "thigh";
        const std::string thigh_parent_name_ = "ground";
        Vec3<Scalar> thigh_location_{0., 0., 0.};
        SpatialInertia<Scalar> thigh_spatial_inertia_;

        // Lower Leg Cluster
        const std::string lower_leg_cluster_name_ = "lower-leg-cluster";

        // Shank Driver
        const std::string shank_driver_name_ = "shank-driver";
        const std::string shank_driver_parent_name_ = "thigh";
        Vec3<Scalar> shank_driver_location_{0.5, 0., 0.};
        SpatialInertia<Scalar> shank_driver_spatial_inertia_;

        // Shank Support
        const std::string shank_support_name_ = "shank-support";
        const std::string shank_support_parent_name_ = "thigh";
        Vec3<Scalar> shank_support_location_{2.5, 0., 0.};
        SpatialInertia<Scalar> shank_support_spatial_inertia_;

        // Foot
        const std::string foot_name_ = "foot";
        const std::string foot_parent_name_ = "shank-driver";
        Vec3<Scalar> foot_location_{4.5, 0., 0.};
        SpatialInertia<Scalar> foot_spatial_inertia_;
        const Scalar foot_length_ = 3.0;

        // TODO(@MatthewChignoli): Better way to encode this? Probably using xtree so that it can match the URDF?
        const Scalar foot_joint_to_constraint_length_ = 5.;
        const Scalar support_joint_to_constraint_length_ = 3.;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_SINGLE_LEG_HOPPER_H
