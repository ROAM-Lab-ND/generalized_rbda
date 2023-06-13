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

	const Mat3<double> R3 = Mat3<double>::Identity();
	const Vec3<double> p3 = Vec3<double>{0., 0., -142.5e-3};

	const Mat3<double> R4 = Mat3<double>::Identity();
	const Vec3<double> p4 = Vec3<double>{0., 0., 0.};

	const Mat3<double> R8 = (Mat3<double>() << -1, 0, 0, 0, 0, 1, 0, 1, 0).finished();
	const Vec3<double> p8 = Vec3<double>{0, 0.04, 0};

	const Mat3<double> R9 = R8;
	const Vec3<double> p9 = Vec3<double>{0, -0.04, 0};

	const double link_3_mass = 0.4433;
	const Vec3<double> link_3_CoM = Vec3<double>{-0.0027, 0., 0.0258};
	const Mat3<double> link_3_inertia = (Mat3<double>() << 0.0018, 0., 0., 0., 0.0017, 0., 0., 0., 0.0015).finished();
	
	const double link_4_mass = 1.5424;
	const Vec3<double> link_4_CoM = Vec3<double>{0.003, -0.0001, -0.0323};
	const Mat3<double> link_4_inertia = (Mat3<double>() << 0.0103, 0., -0.0005, 0., 0.0097, 0., -0.0005, 0., 0.0027).finished();

	const double rotor_8_mass = 1.;
	const Vec3<double> rotor_8_CoM = Vec3<double>{0., 0., 0.};
	const Mat3<double> rotor_8_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();

	const double rotor_9_mass = 1.;
	const Vec3<double> rotor_9_CoM = Vec3<double>{0., 0., 0.};
	const Mat3<double> rotor_9_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();
    };

} // namespace grbda
