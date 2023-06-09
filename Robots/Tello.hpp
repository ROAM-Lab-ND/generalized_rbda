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
	double grav = 9.81;

	Mat3<double> R3 = Mat3<double>::Identity();
	Vec3<double> p3 = Vec3<double>{0., 0., -142.5e-3};

	Mat3<double> R4 = Mat3<double>::Identity();
	Vec3<double> p4 = Vec3<double>{0., 0., 0.};

	Mat3<double> R8 = (Mat3<double>() << -1, 0, 0, 0, 0, 1, 0, 1, 0).finished();
	Vec3<double> p8 = Vec3<double>{0, 0.04, 0};

	Mat3<double> R9 = R8;
	Vec3<double> p9 = Vec3<double>{0, -0.04, 0};

	double link_3_mass = 0.4433;
	Vec3<double> link_3_CoM = Vec3<double>{-0.0027, 0., 0.0258};
	Mat3<double> link_3_inertia = (Mat3<double>() << 0.0018, 0., 0., 0., 0.0017, 0., 0., 0., 0.0015).finished();
	
	double link_4_mass = 1.5424;
	Vec3<double> link_4_CoM = Vec3<double>{0.003, -0.0001, -0.0323};
	Mat3<double> link_4_inertia = (Mat3<double>() << 0.0103, 0., -0.0005, 0., 0.0097, 0., -0.0005, 0., 0.0027).finished();

	double rotor_8_mass = 1.;
	Vec3<double> rotor_8_CoM = Vec3<double>{0., 0., 0.};
	Mat3<double> rotor_8_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();

	double rotor_9_mass = 1.;
	Vec3<double> rotor_9_CoM = Vec3<double>{0., 0., 0.};
	Mat3<double> rotor_9_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();
    };

} // namespace grbda
