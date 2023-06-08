#include "Tello.hpp"

ClusterTreeModel Tello::buildClusterTreeModel() const
{
    ClusterTreeModel model{};

    // Set gravity in z direction
    model.setGravity(Vec3<double>{0., 0., grav});

    // First Rotor
    const std::string rotor_8_name = "hip-roll";
    const std::string rotor_8_parent_name = "ground"; // TODO: hip-clamp
    const SpatialTransform rotor_8_Xtree = SpatialTransform(R8, p8);
    const SpatialInertia<double> rotor_8_spatial_inertia = SpatialInertia<double>{rotor_8_mass, rotor_8_CoM, rotor_8_inertia};
    auto rotor_8 = model.registerBody(rotor_8_name, rotor_8_spatial_inertia, rotor_8_parent_name, rotor_8_Xtree);
    
    // Second Rotor
    const std::string rotor_9_name = "hip-pitch";
    const std::string rotor_9_parent_name = "ground"; // TODO: hip-clamp
    const SpatialTransform rotor_9_Xtree = SpatialTransform(R9, p9);
    const SpatialInertia<double> rotor_9_spatial_inertia = SpatialInertia<double>{rotor_9_mass, rotor_9_CoM, rotor_9_inertia};
    auto rotor_9 = model.registerBody(rotor_9_name, rotor_9_spatial_inertia, rotor_9_parent_name, rotor_9_Xtree);

    // First Link
    const std::string link_3_name = "gimbal";
    const std::string link_3_parent_name = "ground"; // TODO: hip-clamp
    const SpatialTransform link_3_Xtree = SpatialTransform(R3, p3);
    const SpatialInertia<double> link_3_spatial_inertia = SpatialInertia<double>{link_3_mass, link_3_CoM, link_3_inertia};
    auto link_3 = model.registerBody(link_3_name, link_3_spatial_inertia, link_3_parent_name, link_3_Xtree);

    // Second Link
    const std::string link_4_name = "thigh";
    const std::string link_4_parent_name = "gimbal";
    const SpatialTransform link_4_Xtree = SpatialTransform(R4, p4);
    const SpatialInertia<double> link_4_spatial_inertia = SpatialInertia<double>{link_4_mass, link_4_CoM, link_4_inertia};
    auto link_4 = model.registerBody(link_4_name, link_4_spatial_inertia, link_4_parent_name, link_4_Xtree);
    
    // Body Cluster
    const std::string body_cluster_name = "hip-differential";

    auto joint = std::make_shared<GeneralizedJoints::TelloHipDifferential>(
	    rotor_8, rotor_9, link_3, link_4, CoordinateAxis::Z, CoordinateAxis::Z,
	    CoordinateAxis::X, CoordinateAxis::Y);

    model.appendRegisteredBodiesAsCluster(body_cluster_name, joint);

    return model;
}
