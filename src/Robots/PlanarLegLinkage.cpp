#include "grbda/Robots/PlanarLegLinkage.hpp"

namespace grbda
{
    template <typename Scalar>
    PlanarLegLinkage<Scalar>::PlanarLegLinkage()
    {
        // Thigh
        Mat3<Scalar> thigh_rotiatonal_inertia = Mat3<Scalar>::Zero();
        thigh_rotiatonal_inertia(2, 2) = 45.389e-6;
        Vec3<Scalar> thigh_com{.0364, 0., 0.};
        thigh_spatial_inertia_ = SpatialInertia<Scalar>(0.23, thigh_com, thigh_rotiatonal_inertia);

        // Shank Driver
        Mat3<Scalar> shank_driver_rotiatonal_inertia = Mat3<Scalar>::Zero();
        shank_driver_rotiatonal_inertia(2, 2) = 3.257e-6;
        Vec3<Scalar> shank_driver_com{.048, 0., 0.};
        shank_driver_spatial_inertia_ = SpatialInertia<Scalar>(.004, shank_driver_com, shank_driver_rotiatonal_inertia);

        // Shank Support
        Mat3<Scalar> shank_support_rotiatonal_inertia = Mat3<Scalar>::Zero();
        shank_support_rotiatonal_inertia(2, 2) = 22.918e-6;
        Vec3<Scalar> shank_support_com{.04, 0., 0.};
        shank_support_spatial_inertia_ = SpatialInertia<Scalar>(0.225, shank_support_com, shank_support_rotiatonal_inertia);

        // Foot
        Mat3<Scalar> foot_rotiatonal_inertia = Mat3<Scalar>::Zero();
        foot_rotiatonal_inertia(2, 2) = 22.176e-6;
        Vec3<Scalar> foot_com{.0635, 0., 0.};
        foot_spatial_inertia_ = SpatialInertia<Scalar>(.017, foot_com, foot_rotiatonal_inertia);
    }

    template <typename Scalar>
    ClusterTreeModel<Scalar> PlanarLegLinkage<Scalar>::buildClusterTreeModel() const
    {
        using SpatialTransform = spatial::Transform<Scalar>;
        Mat3<Scalar> I3 = Mat3<Scalar>::Identity();
        ori::CoordinateAxis common_axis = ori::CoordinateAxis::Z;

        ClusterTreeModel<Scalar> model{};

        // Thigh
        SpatialTransform thigh_Xtree{I3, thigh_location_};
        Body<Scalar> thigh = model.registerBody(thigh_name_, thigh_spatial_inertia_,
                                                thigh_parent_name_, thigh_Xtree);
        using RevoluteCluster = ClusterJoints::Revolute<Scalar>;
        model.template appendRegisteredBodiesAsCluster<RevoluteCluster>(thigh_name_,
                                                                        thigh, common_axis);

        // Shank Driver
        SpatialTransform shank_driver_Xtree{I3, shank_driver_location_};
        Body<Scalar> shank_driver = model.registerBody(
            shank_driver_name_, shank_driver_spatial_inertia_,
            shank_driver_parent_name_, shank_driver_Xtree);
        JointPtr<Scalar> driver_joint = std::make_shared<Joints::Revolute<Scalar>>(common_axis);

        // Shank Support
        SpatialTransform shank_support_Xtree{I3, shank_support_location_};
        Body<Scalar> shank_support = model.registerBody(
            shank_support_name_, shank_support_spatial_inertia_,
            shank_support_parent_name_, shank_support_Xtree);
        JointPtr<Scalar> support_joint = std::make_shared<Joints::Revolute<Scalar>>(common_axis);

        // Foot
        SpatialTransform foot_Xtree{I3, foot_location_};
        Body<Scalar> foot = model.registerBody(foot_name_, foot_spatial_inertia_,
                                               foot_parent_name_, foot_Xtree);
        JointPtr<Scalar> foot_joint = std::make_shared<Joints::Revolute<Scalar>>(common_axis);

        // Loop Constraint
        // NOTE: Linkage is a parallelogram, so path1_lengths[0] = path2_lengths[1] and
        // path1_lengths[1] = offset[0]
        std::vector<Scalar> path1_link_lengths, path2_link_lengths;
        path1_link_lengths.push_back(foot_location_[0]);
        path1_link_lengths.push_back(shank_support_location_[0] - shank_driver_location_[0]);
        path2_link_lengths.push_back(path1_link_lengths[0]);
        Vec2<Scalar> offset{path1_link_lengths[1], 0.};
        int indepedent_coord = 0.;
        std::shared_ptr<LoopConstraint::FourBar<Scalar>> loop_constraint =
            std::make_shared<LoopConstraint::FourBar<Scalar>>(path1_link_lengths, path2_link_lengths, offset, indepedent_coord);

        // Lower Leg Cluster
        std::vector<Body<Scalar>> lower_leg_bodies = {shank_driver, shank_support, foot};
        std::vector<JointPtr<Scalar>> lower_leg_joints = {driver_joint, support_joint, foot_joint};
        using FourBarCluster = ClusterJoints::FourBar<Scalar>;
        model.template appendRegisteredBodiesAsCluster<FourBarCluster>(
            lower_leg_cluster_name_, lower_leg_bodies, lower_leg_joints, loop_constraint);

        // End-effector
        std::string end_effector_name = "end-effector";
        Vec3<Scalar> end_effector_offset{path1_link_lengths[1] + .096, 0., 0.};
        model.appendEndEffector(foot_name_, end_effector_offset, end_effector_name);

        return model;
    }

    template class PlanarLegLinkage<double>;
    template class PlanarLegLinkage<casadi::SX>;
}
