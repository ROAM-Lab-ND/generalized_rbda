#include "Tello.hpp"

namespace grbda
{

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
	const std::string body_cluster_name_2 = "hip-differential";

	auto joint_2 = std::make_shared<GeneralizedJoints::TelloHipDifferential>(
		rotor_8, rotor_9, link_3, link_4, CoordinateAxis::Z, CoordinateAxis::Z,
		CoordinateAxis::X, CoordinateAxis::Y);

	model.appendRegisteredBodiesAsCluster(body_cluster_name_2, joint_2);

	// Knee-ankle differential left rotor
	const std::string rotor_10_name = "knee-ankle-left-motor";
	const std::string rotor_10_parent_name = "thigh";
	const SpatialTransform rotor_10_Xtree = SpatialTransform(R10, p10);
	const SpatialInertia<double> rotor_10_spatial_inertia = SpatialInertia<double>{rotor_10_mass, rotor_10_CoM, rotor_10_inertia};
	auto rotor_10 = model.registerBody(rotor_10_name, rotor_10_spatial_inertia, rotor_10_parent_name, rotor_10_Xtree);
	
	// Knee-ankle differential right motor
	const std::string rotor_11_name = "knee-ankle-right-motor";
	const std::string rotor_11_parent_name = "thigh"; // TODO: hip-clamp
	const SpatialTransform rotor_11_Xtree = SpatialTransform(R11, p11);
	const SpatialInertia<double> rotor_11_spatial_inertia = SpatialInertia<double>{rotor_11_mass, rotor_11_CoM, rotor_11_inertia};
	auto rotor_11 = model.registerBody(rotor_11_name, rotor_11_spatial_inertia, rotor_11_parent_name, rotor_11_Xtree);

	// Shin
	const std::string link_5_name = "shin";
	const std::string link_5_parent_name = "thigh"; // TODO: hip-clamp
	const SpatialTransform link_5_Xtree = SpatialTransform(R5, p5);
	const SpatialInertia<double> link_5_spatial_inertia = SpatialInertia<double>{link_5_mass, link_5_CoM, link_5_inertia};
	auto link_5 = model.registerBody(link_5_name, link_5_spatial_inertia, link_5_parent_name, link_5_Xtree);

	// Foot
	const std::string link_6_name = "foot";
	const std::string link_6_parent_name = "shin";
	const SpatialTransform link_6_Xtree = SpatialTransform(R6, p6);
	const SpatialInertia<double> link_6_spatial_inertia = SpatialInertia<double>{link_6_mass, link_6_CoM, link_6_inertia};
	auto link_6 = model.registerBody(link_6_name, link_6_spatial_inertia, link_6_parent_name, link_6_Xtree);
	
	// Body Cluster
	const std::string body_cluster_name_3 = "knee-ankle-differential";

	auto joint_3 = std::make_shared<GeneralizedJoints::TelloKneeAnkleDifferential>(
		rotor_10, rotor_11, link_5, link_6, CoordinateAxis::Z, CoordinateAxis::Z,
		CoordinateAxis::Y, CoordinateAxis::Y);

	model.appendRegisteredBodiesAsCluster(body_cluster_name_3, joint_3);

	return model;
    }

} // namespace grbda
