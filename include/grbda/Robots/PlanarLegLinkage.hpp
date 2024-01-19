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
        const std::string shank_driver_name_ = "driving_shank";
        const std::string shank_driver_parent_name_ = "thigh";
        Vec3<Scalar> shank_driver_location_{.011, 0., 0.};
        SpatialInertia<Scalar> shank_driver_spatial_inertia_;

        // Shank Support
        const std::string shank_support_name_ = "supporting_shank";
        const std::string shank_support_parent_name_ = "thigh";
        Vec3<Scalar> shank_support_location_{.042, 0., 0.};
        SpatialInertia<Scalar> shank_support_spatial_inertia_;

        // Foot
        const std::string foot_name_ = "foot";
        const std::string foot_parent_name_ = "driving_shank";
        Vec3<Scalar> foot_location_{.096, 0., 0.};
        SpatialInertia<Scalar> foot_spatial_inertia_;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_SINGLE_LEG_HOPPER_H
