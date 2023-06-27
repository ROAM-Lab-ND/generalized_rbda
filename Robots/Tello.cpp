#include "Tello.hpp"

namespace grbda
{

    ClusterTreeModel Tello::buildClusterTreeModel() const
    {
	ClusterTreeModel model{};

	// Set gravity in z direction
	model.setGravity(Vec3<double>{0., 0., grav});

	// Hip clamp rotor
	const std::string hip_clamp_rotor_name = "hip-clamp-rotor";
	const std::string hip_clamp_rotor_parent_name = "ground";
	const SpatialTransform hip_clamp_rotor_Xtree = SpatialTransform(R_hip_clamp, p_hip_clamp);
	const SpatialInertia<double> hip_clamp_rotor_spatial_inertia = SpatialInertia<double>{hip_clamp_rotor_mass, hip_clamp_rotor_CoM, hip_clamp_rotor_inertia};
	auto hip_clamp_rotor = model.registerBody(hip_clamp_rotor_name, hip_clamp_rotor_spatial_inertia, hip_clamp_rotor_parent_name, hip_clamp_rotor_Xtree);

	// Hip clamp
	const std::string hip_clamp_name = "hip-clamp";
	const std::string hip_clamp_parent_name = "ground";
	const SpatialTransform hip_clamp_Xtree = SpatialTransform(R_hip_clamp_rotor, p_hip_clamp_rotor);
	const SpatialInertia<double> hip_clamp_spatial_inertia = SpatialInertia<double>{hip_clamp_mass, hip_clamp_CoM, hip_clamp_inertia};
	auto hip_clamp = model.registerBody(hip_clamp_name, hip_clamp_spatial_inertia, hip_clamp_parent_name, hip_clamp_Xtree);

	// Body Cluster 1
	const std::string body_cluster_name_1 = "hip-clamp";
	const int gear_ratio = 1;
	auto joint_1 = std::make_shared<GeneralizedJoints::RevoluteWithRotor>(
		hip_clamp_rotor, hip_clamp, CoordinateAxis::Z, CoordinateAxis::Z, gear_ratio);
	model.appendRegisteredBodiesAsCluster(body_cluster_name_1, joint_1);

	// Hip differential left rotor
	const std::string hip_rotor_left_name = "hip-left-rotor";
	const std::string hip_rotor_left_parent_name = "hip-clamp";
	const SpatialTransform hip_rotor_left_Xtree = SpatialTransform(R_hip_rotor_left, p_hip_rotor_left);
	const SpatialInertia<double> hip_rotor_left_spatial_inertia = SpatialInertia<double>{hip_rotor_left_mass, hip_rotor_left_CoM, hip_rotor_left_inertia};
	auto hip_rotor_left = model.registerBody(hip_rotor_left_name, hip_rotor_left_spatial_inertia, hip_rotor_left_parent_name, hip_rotor_left_Xtree);
	
	// Hip differential right rotor
	const std::string hip_rotor_right_name = "hip-right-rotor";
	const std::string hip_rotor_right_parent_name = "hip-clamp";
	const SpatialTransform hip_rotor_right_Xtree = SpatialTransform(R_hip_rotor_right, p_hip_rotor_right);
	const SpatialInertia<double> hip_rotor_right_spatial_inertia = SpatialInertia<double>{hip_rotor_right_mass, hip_rotor_right_CoM, hip_rotor_right_inertia};
	auto hip_rotor_right = model.registerBody(hip_rotor_right_name, hip_rotor_right_spatial_inertia, hip_rotor_right_parent_name, hip_rotor_right_Xtree);

	// Gimbal
	const std::string gimbal_name = "gimbal";
	const std::string gimbal_parent_name = "hip-clamp";
	const SpatialTransform gimbal_Xtree = SpatialTransform(R_gimbal, p_gimbal);
	const SpatialInertia<double> gimbal_spatial_inertia = SpatialInertia<double>{gimbal_mass, gimbal_CoM, gimbal_inertia};
	auto gimbal = model.registerBody(gimbal_name, gimbal_spatial_inertia, gimbal_parent_name, gimbal_Xtree);

	// Thigh
	const std::string thigh_name = "thigh";
	const std::string thigh_parent_name = "gimbal";
	const SpatialTransform thigh_Xtree = SpatialTransform(R_thigh, p_thigh);
	const SpatialInertia<double> thigh_spatial_inertia = SpatialInertia<double>{thigh_mass, thigh_CoM, thigh_inertia};
	auto thigh = model.registerBody(thigh_name, thigh_spatial_inertia, thigh_parent_name, thigh_Xtree);
	
	// Body Cluster 2
	const std::string body_cluster_name_2 = "hip-differential";
	auto joint_2 = std::make_shared<GeneralizedJoints::TelloHipDifferential>(
		hip_rotor_left, hip_rotor_right, gimbal, thigh,
		CoordinateAxis::Z, CoordinateAxis::Z,
		CoordinateAxis::X, CoordinateAxis::Y);
	model.appendRegisteredBodiesAsCluster(body_cluster_name_2, joint_2);

	// Knee-ankle differential left rotor
	const std::string knee_ankle_rotor_left_name = "knee-ankle-left-motor";
	const std::string knee_ankle_rotor_left_parent_name = "thigh";
	const SpatialTransform knee_ankle_rotor_left_Xtree = SpatialTransform(R_knee_ankle_rotor_left, p_knee_ankle_rotor_left);
	const SpatialInertia<double> knee_ankle_rotor_left_spatial_inertia = SpatialInertia<double>{knee_ankle_rotor_left_mass, knee_ankle_rotor_left_CoM, knee_ankle_rotor_left_inertia};
	auto knee_ankle_rotor_left = model.registerBody(knee_ankle_rotor_left_name, knee_ankle_rotor_left_spatial_inertia, knee_ankle_rotor_left_parent_name, knee_ankle_rotor_left_Xtree);
	
	// Knee-ankle differential right motor
	const std::string knee_ankle_rotor_right_name = "knee-ankle-right-motor";
	const std::string knee_ankle_rotor_right_parent_name = "thigh";
	const SpatialTransform knee_ankle_rotor_right_Xtree = SpatialTransform(R_knee_ankle_rotor_right, p_knee_ankle_rotor_right);
	const SpatialInertia<double> knee_ankle_rotor_right_spatial_inertia = SpatialInertia<double>{knee_ankle_rotor_right_mass, knee_ankle_rotor_right_CoM, knee_ankle_rotor_right_inertia};
	auto knee_ankle_rotor_right = model.registerBody(knee_ankle_rotor_right_name, knee_ankle_rotor_right_spatial_inertia, knee_ankle_rotor_right_parent_name, knee_ankle_rotor_right_Xtree);

	// Shin
	const std::string shin_name = "shin";
	const std::string shin_parent_name = "thigh";
	const SpatialTransform shin_Xtree = SpatialTransform(R_shin, p_shin);
	const SpatialInertia<double> shin_spatial_inertia = SpatialInertia<double>{shin_mass, shin_CoM, shin_inertia};
	auto shin = model.registerBody(shin_name, shin_spatial_inertia, shin_parent_name, shin_Xtree);

	// Foot
	const std::string foot_name = "foot";
	const std::string foot_parent_name = "shin";
	const SpatialTransform foot_Xtree = SpatialTransform(R_foot, p_foot);
	const SpatialInertia<double> foot_spatial_inertia = SpatialInertia<double>{foot_mass, foot_CoM, foot_inertia};
	auto foot = model.registerBody(foot_name, foot_spatial_inertia, foot_parent_name, foot_Xtree);
	
	// Body Cluster 3
	const std::string body_cluster_name_3 = "knee-ankle-differential";
	auto joint_3 = std::make_shared<GeneralizedJoints::TelloKneeAnkleDifferential>(
		knee_ankle_rotor_left, knee_ankle_rotor_right, shin, foot,
		CoordinateAxis::Z, CoordinateAxis::Z,
		CoordinateAxis::Y, CoordinateAxis::Y);
	model.appendRegisteredBodiesAsCluster(body_cluster_name_3, joint_3);

	return model;
    }

} // namespace grbda
