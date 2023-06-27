#pragma once

#include "Robot.h"

namespace grbda
{

    class Tello : public Robot
    {
    public:
	Tello() {}

	ClusterTreeModel buildClusterTreeModel() const override;

    private:
	const double grav = 9.81;

	const Mat3<double> R_hip_clamp = Mat3<double>::Identity();
	const Vec3<double> p_hip_clamp = Vec3<double>{0., 126e-3, -87e-3};

	const Mat3<double> R_hip_clamp_rotor = Mat3<double>::Identity();
	const Vec3<double> p_hip_clamp_rotor = Vec3<double>{0., 126e-3, -26e-3};

	const Mat3<double> R_gimbal = Mat3<double>::Identity();
	const Vec3<double> p_gimbal = Vec3<double>{0., 0., -142.5e-3};

	const Mat3<double> R_thigh = Mat3<double>::Identity();
	const Vec3<double> p_thigh = Vec3<double>{0., 0., 0.};

	const Mat3<double> R_hip_rotor_left = (Mat3<double>() << -1., 0., 0., 0., 0., 1., 0., 1., 0.).finished();
	const Vec3<double> p_hip_rotor_left = Vec3<double>{0., 0.04, 0.};

	const Mat3<double> R_hip_rotor_right = R_hip_rotor_left;
	const Vec3<double> p_hip_rotor_right = Vec3<double>{0., -0.04, 0.};

	const Mat3<double> R_shin = Mat3<double>::Identity();
	const Vec3<double> p_shin = Vec3<double>{0., 0., -226.8e-3};

	const Mat3<double> R_foot = Mat3<double>::Identity();
	const Vec3<double> p_foot = Vec3<double>{0., 0., -260e-3};

	const Mat3<double> R_knee_ankle_rotor_left = R_hip_rotor_left;
	const Vec3<double> p_knee_ankle_rotor_left = Vec3<double>{0., 26.55e-3, 0.};

	const Mat3<double> R_knee_ankle_rotor_right = R_hip_rotor_left;
	const Vec3<double> p_knee_ankle_rotor_right = Vec3<double>{0., -26.55e-3, 0.};

	const double hip_clamp_mass = 1.3289;
	const Vec3<double> hip_clamp_CoM = Vec3<double>{-0.0010, 0., -0.0069};
	const Mat3<double> hip_clamp_inertia = (Mat3<double>() << 0.0032, 0., 0.0001, 0., 0.0033, 0., 0.0001, 0., 0.0027).finished();

	const double hip_clamp_rotor_mass = 1.;
	const Vec3<double> hip_clamp_rotor_CoM = Vec3<double>{0., 0., 0.};
	const Mat3<double> hip_clamp_rotor_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();

	const double gimbal_mass = 0.4433;
	const Vec3<double> gimbal_CoM = Vec3<double>{-0.0027, 0., 0.0258};
	const Mat3<double> gimbal_inertia = (Mat3<double>() << 0.0018, 0., 0., 0., 0.0017, 0., 0., 0., 0.0015).finished();
	
	const double thigh_mass = 1.5424;
	const Vec3<double> thigh_CoM = Vec3<double>{0.003, -0.0001, -0.0323};
	const Mat3<double> thigh_inertia = (Mat3<double>() << 0.0103, 0., -0.0005, 0., 0.0097, 0., -0.0005, 0., 0.0027).finished();

	const double hip_rotor_left_mass = 1.;
	const Vec3<double> hip_rotor_left_CoM = Vec3<double>{0., 0., 0.};
	const Mat3<double> hip_rotor_left_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();

	const double hip_rotor_right_mass = 1.;
	const Vec3<double> hip_rotor_right_CoM = Vec3<double>{0., 0., 0.};
	const Mat3<double> hip_rotor_right_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();

	const double shin_mass = 0.3072;
	const Vec3<double> shin_CoM = Vec3<double>{0.0047, -0.0003, -0.1043};
	const Mat3<double> shin_inertia = (Mat3<double>() << 0.0054 , -0., -0.0002, -0., 0.0054, 0., -0.0002, 0., 0.0001).finished();
	
	const double foot_mass = 0.1025;
	const Vec3<double> foot_CoM = Vec3<double>{0.0042, -0., -0.0251};
	const Mat3<double> foot_inertia = (Mat3<double>() << 0.094e-3, -0., -0.0038e-3, -0., 0.1773e-3, 0., -0.0038e-3, 0., 0.0901e-3).finished();

	const double knee_ankle_rotor_left_mass = 1.;
	const Vec3<double> knee_ankle_rotor_left_CoM = Vec3<double>{0., 0., 0.};
	const Mat3<double> knee_ankle_rotor_left_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();

	const double knee_ankle_rotor_right_mass = 1.;
	const Vec3<double> knee_ankle_rotor_right_CoM = Vec3<double>{0., 0., 0.};
	const Mat3<double> knee_ankle_rotor_right_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();
    };

} // namespace grbda
