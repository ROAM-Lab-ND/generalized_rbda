#ifndef GRBDA_ROBOTS_TELLO_WITH_ARMS_H
#define GRBDA_ROBOTS_TELLO_WITH_ARMS_H

#include "Robot.h"

namespace grbda
{

    class TelloWithArms : public Robot
    {
    public:
        TelloWithArms()
        {
            _shoulderRyRotInertia << 0.0013678, 0.0000266, 0.0000021,
                0.0000266, 0.0007392, -0.0000012,
                0.0000021, -0.0000012, 0.000884;
            _shoulderRxRotInertia << 0.0011524, 0.0000007, 0.0000396,
                0.0000007, 0.0011921, 0.0000014,
                0.0000396, 0.0000014, 0.0012386;
            _shoulderRzRotInertia << 0.0012713, 0.000001, -0.000008,
                0.000001, 0.0017477, -0.0000225,
                -0.000008, -0.0000225, 0.0008191;
            _elbowRotInertia << 0.001570, 0.0000002, 0.0000335,
                0.0000002, 0.0016167, 0.000003,
                0.0000335, 0.000003, 0.0000619;

            _smallRotorRotationalInertiaZ << 1.084e-4, 0, 0,
                0, 1.084e-4, 0,
                0, 0, 1.6841e-4;

            Mat3<double> RY = coordinateRotation<double>(CoordinateAxis::Y, M_PI / 2);
            Mat3<double> RX = coordinateRotation<double>(CoordinateAxis::X, -M_PI / 2);

            Mat3<double> smallRotorRotationalInertiaX = RY.transpose() * _smallRotorRotationalInertiaZ * RY;
            Mat3<double> smallRotorRotationalInertiaY = RX.transpose() * _smallRotorRotationalInertiaZ * RX;

            _smallRotorRotInertiaX = smallRotorRotationalInertiaX;
            _smallRotorRotInertiaY = smallRotorRotationalInertiaY;
            _smallRotorRotInertiaZ = _smallRotorRotationalInertiaZ;

        }

        ClusterTreeModel buildClusterTreeModel() const override;

    private:
    	const double grav = 9.81;
    
    	// Left leg
    	const Mat3<double> R_left_hip_clamp = Mat3<double>::Identity();
    	const Vec3<double> p_left_hip_clamp = Vec3<double>{0., 126e-3, -87e-3};
    
    	const Mat3<double> R_left_gimbal = Mat3<double>::Identity();
    	const Vec3<double> p_left_gimbal = Vec3<double>{0., 0., -142.5e-3};
    
    	const Mat3<double> R_left_thigh = Mat3<double>::Identity();
    	const Vec3<double> p_left_thigh = Vec3<double>::Zero();
    
    	const Mat3<double> R_left_shin = Mat3<double>::Identity();
    	const Vec3<double> p_left_shin = Vec3<double>{0., 0., -226.8e-3};
    
    	const Mat3<double> R_left_foot = Mat3<double>::Identity();
    	const Vec3<double> p_left_foot = Vec3<double>{0., 0., -260e-3};
    
    	const Mat3<double> R_left_hip_clamp_rotor = Mat3<double>::Identity();
    	const Vec3<double> p_left_hip_clamp_rotor = Vec3<double>{0., 126e-3, -26e-3};
    
    	const Mat3<double> R_left_hip_rotor_1 = (Mat3<double>() << -1., 0., 0., 0., 0., 1., 0., 1., 0.).finished();
    	const Vec3<double> p_left_hip_rotor_1 = Vec3<double>{0., 0.04, 0.};
    
    	const Mat3<double> R_left_hip_rotor_2 = R_left_hip_rotor_1;
    	const Vec3<double> p_left_hip_rotor_2 = Vec3<double>{0., -0.04, 0.};
    
    	const Mat3<double> R_left_knee_ankle_rotor_1 = R_left_hip_rotor_1;
    	const Vec3<double> p_left_knee_ankle_rotor_1 = Vec3<double>{0., 26.55e-3, 0.};
    
    	const Mat3<double> R_left_knee_ankle_rotor_2 = R_left_hip_rotor_1;
    	const Vec3<double> p_left_knee_ankle_rotor_2 = Vec3<double>{0., -26.55e-3, 0.};
    
    	// Right leg
    	const Mat3<double> R_right_hip_clamp = Mat3<double>::Identity();
    	const Vec3<double> p_right_hip_clamp = Vec3<double>{0., -126e-3, -87e-3};
    
    	const Mat3<double> R_right_gimbal = Mat3<double>::Identity();
    	const Vec3<double> p_right_gimbal = Vec3<double>{0., 0., -142.5e-3};
    
    	const Mat3<double> R_right_thigh = Mat3<double>::Identity();
    	const Vec3<double> p_right_thigh = Vec3<double>::Zero();
    
    	const Mat3<double> R_right_shin = Mat3<double>::Identity();
    	const Vec3<double> p_right_shin = Vec3<double>{0., 0., -226.8e-3};
    
    	const Mat3<double> R_right_foot = Mat3<double>::Identity();
    	const Vec3<double> p_right_foot = Vec3<double>{0., 0., -260e-3};
    
    	const Mat3<double> R_right_hip_clamp_rotor = Mat3<double>::Identity();
    	const Vec3<double> p_right_hip_clamp_rotor = Vec3<double>{0., -126e-3, -26e-3};
    
    	const Mat3<double> R_right_hip_rotor_1 = (Mat3<double>() << -1., 0., 0., 0., 0., 1., 0., 1., 0.).finished();
    	const Vec3<double> p_right_hip_rotor_1 = Vec3<double>{0., 0.04, 0.};
    
    	const Mat3<double> R_right_hip_rotor_2 = R_right_hip_rotor_1;
    	const Vec3<double> p_right_hip_rotor_2 = Vec3<double>{0., -0.04, 0.};
    
    	const Mat3<double> R_right_knee_ankle_rotor_1 = R_right_hip_rotor_1;
    	const Vec3<double> p_right_knee_ankle_rotor_1 = Vec3<double>{0., 26.55e-3, 0.};
    
    	const Mat3<double> R_right_knee_ankle_rotor_2 = R_right_hip_rotor_1;
    	const Vec3<double> p_right_knee_ankle_rotor_2 = Vec3<double>{0., -26.55e-3, 0.};
    
    	const double torso_mass = 2.3008;
    	const Vec3<double> torso_CoM = Vec3<double>{0.0073, -0.0013, -0.0023};
    	const Mat3<double> torso_inertia = (Mat3<double>() << 0.0366, 0., -0.0006, 0., 0.0142, -0.0002, -0.0006, -0.0002, 0.0291).finished();
    
        const double gear_ratio = 6.0;

    	const double hip_clamp_mass = 1.3289;
    	const Vec3<double> hip_clamp_CoM = Vec3<double>{-0.0010, 0., -0.0069};
    	const Mat3<double> hip_clamp_inertia = (Mat3<double>() << 0.0032, 0., 0.0001, 0., 0.0033, 0., 0.0001, 0., 0.0027).finished();
    
    	const double gimbal_mass = 0.4433;
    	const Vec3<double> gimbal_CoM = Vec3<double>{-0.0027, 0., 0.0258};
    	const Mat3<double> gimbal_inertia = (Mat3<double>() << 0.0018, 0., 0., 0., 0.0017, 0., 0., 0., 0.0015).finished();
    	
    	const double thigh_mass = 1.5424;
    	const Vec3<double> thigh_CoM = Vec3<double>{0.003, -0.0001, -0.0323};
    	const Mat3<double> thigh_inertia = (Mat3<double>() << 0.0103, 0., -0.0005, 0., 0.0097, 0., -0.0005, 0., 0.0027).finished();
    
    	const double shin_mass = 0.3072;
    	const Vec3<double> shin_CoM = Vec3<double>{0.0047, -0.0003, -0.1043};
    	const Mat3<double> shin_inertia = (Mat3<double>() << 0.0054 , -0., -0.0002, -0., 0.0054, 0., -0.0002, 0., 0.0001).finished();
    	
    	const double foot_mass = 0.1025;
    	const Vec3<double> foot_CoM = Vec3<double>{0.0042, -0., -0.0251};
    	const Mat3<double> foot_inertia = (Mat3<double>() << 0.094e-3, -0., -0.0038e-3, -0., 0.1773e-3, 0., -0.0038e-3, 0., 0.0901e-3).finished();
    
    	const double hip_clamp_rotor_mass = 1.;
    	const Vec3<double> hip_clamp_rotor_CoM = Vec3<double>{0., 0., 0.};
    	const Mat3<double> hip_clamp_rotor_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();
    
    	const double hip_rotor_1_mass = 1.;
    	const Vec3<double> hip_rotor_1_CoM = Vec3<double>{0., 0., 0.};
    	const Mat3<double> hip_rotor_1_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();
    
    	const double hip_rotor_2_mass = 1.;
    	const Vec3<double> hip_rotor_2_CoM = Vec3<double>{0., 0., 0.};
    	const Mat3<double> hip_rotor_2_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();
    
    	const double knee_ankle_rotor_1_mass = 1.;
    	const Vec3<double> knee_ankle_rotor_1_CoM = Vec3<double>{0., 0., 0.};
    	const Mat3<double> knee_ankle_rotor_1_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();
    
    	const double knee_ankle_rotor_2_mass = 1.;
    	const Vec3<double> knee_ankle_rotor_2_CoM = Vec3<double>{0., 0., 0.};
    	const Mat3<double> knee_ankle_rotor_2_inertia = (Mat3<double>() << 0., 0., 0., 0., 0., 0., 0., 0., 0.003).finished();
    
    	const double _footToeLength = 0.1;
        const double _footHeelLength = 0.05;
        const double _footHeight = 0.041;
    
        // Arms
        double _shoulderRyMass = 0.788506;
        double _shoulderRxMass = 0.80125;
        double _shoulderRzMass = 0.905588;
        double _elbowMass = 0.34839;

        Vec3<double> _shoulderRyLocation = Vec3<double>(0.01346, -0.17608, 0.24657);
        Vec3<double> _shoulderRxLocation = Vec3<double>(0.0, -0.0575, 0.0);
        Vec3<double> _shoulderRzLocation = Vec3<double>(0.0, 0.0, -0.10250);
        Vec3<double> _elbowLocation = Vec3<double>(0.0, 0.0, -0.1455);
    
        Vec3<double> _shoulderRyCOM = Vec3<double>(0.009265, 0.052623, -0.0001249);
        Vec3<double> _shoulderRxCOM = Vec3<double>(0.0006041, 0.0001221, -0.082361);
        Vec3<double> _shoulderRzCOM = Vec3<double>(0.0001703, -0.016797, -0.060);
        Vec3<double> _elbowCOM = Vec3<double>(-0.0059578, 0.000111, -0.0426735);
    
        Vec3<double> _smallRotorCOM = Vec3<double>(0., 0., 0.);

        double _shoulderRxGearRatio = 6.0;
        double _shoulderRzGearRatio = 6.0;
        double _shoulderRyGearRatio = 6.0;
        double _elbowGearRatio = 9.0;
    
        Vec3<double> _shoulderRyRotorLocation = Vec3<double>(0.01346, -0.16, 0.24657);
        Vec3<double> _shoulderRxRotorLocation = Vec3<double>(0, -0.0575, 0);
        Vec3<double> _shoulderRzRotorLocation = Vec3<double>(0., 0., -0.1025);
        Vec3<double> _elbowRotorLocation = Vec3<double>(0., 0.0325, -0.06);
    
        Mat3<double> _shoulderRyRotInertia;
        Mat3<double> _shoulderRxRotInertia;
        Mat3<double> _shoulderRzRotInertia;
        Mat3<double> _elbowRotInertia;
    
        Mat3<double> _smallRotorRotationalInertiaZ;

        Mat3<double> _smallRotorRotInertiaX;
        Mat3<double> _smallRotorRotInertiaY;
        Mat3<double> _smallRotorRotInertiaZ;

        double _lowerArmLength = 0.27;

        template <typename T>
        Vec3<double> withLeftRightSigns(const Eigen::MatrixBase<T> &v, int side) const
        {
            static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                          "Must have 3x1 matrix");
            switch (side)
            {
            case 0: // right
                return Vec3<double>(v[0], v[1], v[2]);
            case 1: // left
                return Vec3<double>(v[0], -v[1], v[2]);
            default:
                throw std::runtime_error("Invalid side id!");
            }
        }

        std::string withLeftRightSigns(const std::string &s, int side) const
        {
            switch (side)
            {
            case 0: // right
                return "right_" + s;
            case 1: // left
                return "left_" + s;
            default:
                throw std::runtime_error("Invalid side id!");
            }
        }

        SpatialInertia<double> withLeftRightSigns(const SpatialInertia<double> &I, int side) const
        {
            switch (side)
            {
            case -1: // right
            case 0:  // right
                return I.flipAlongAxis(ori::CoordinateAxis::Y);
            case 1: // left
                return I;
            default:
                throw std::runtime_error("Invalid side id!");
            }
        }
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_TELLO_WITH_ARMS_H
