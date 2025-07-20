#ifndef GRBDA_ROBOTS_TELLO_H
#define GRBDA_ROBOTS_TELLO_H

#include "grbda/Robots/Robot.h"

namespace grbda
{

    template <typename Scalar>
    class Tello : public Robot<Scalar>
    {
    public:
        Tello() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;

    protected:
        const std::string base = "torso";

        const Scalar grav = -9.81;
    
        const Mat3<Scalar> R_down = (Mat3<Scalar>() << 1., 0., 0., 0., -1., 0., 0., 0., -1.).finished();
        const Mat3<Scalar> R_left = (Mat3<Scalar>() << -1., 0., 0., 0., 0., 1., 0., 1., 0.).finished();
        const Mat3<Scalar> R_right = (Mat3<Scalar>() << 1., 0., 0., 0., 0., 1., 0., -1., 0.).finished();

        // Left leg
        const Mat3<Scalar> R_left_hip_clamp = Mat3<Scalar>::Identity();
        const Vec3<Scalar> p_left_hip_clamp = Vec3<Scalar>{0., 126e-3, -87e-3};
    
        const Mat3<Scalar> R_left_gimbal = Mat3<Scalar>::Identity();
        const Vec3<Scalar> p_left_gimbal = Vec3<Scalar>{0., 0., -142.5e-3};
    
        const Mat3<Scalar> R_left_thigh = Mat3<Scalar>::Identity();
        const Vec3<Scalar> p_left_thigh = Vec3<Scalar>::Zero();
    
        const Mat3<Scalar> R_left_shin = Mat3<Scalar>::Identity();
        const Vec3<Scalar> p_left_shin = Vec3<Scalar>{0., 0., -226.8e-3};
    
        const Mat3<Scalar> R_left_foot = Mat3<Scalar>::Identity();
        const Vec3<Scalar> p_left_foot = Vec3<Scalar>{0., 0., -260e-3};
    
        const Mat3<Scalar> R_left_hip_clamp_rotor = R_down;
        const Vec3<Scalar> p_left_hip_clamp_rotor = Vec3<Scalar>{0., 126e-3, -26e-3};
    
        const Mat3<Scalar> R_left_hip_rotor_1 = R_left;
        const Vec3<Scalar> p_left_hip_rotor_1 = Vec3<Scalar>{0., 0.04, 0.};
    
        const Mat3<Scalar> R_left_hip_rotor_2 = R_right;
        const Vec3<Scalar> p_left_hip_rotor_2 = Vec3<Scalar>{0., -0.04, 0.};
    
        const Mat3<Scalar> R_left_knee_ankle_rotor_1 = R_right;
        const Vec3<Scalar> p_left_knee_ankle_rotor_1 = Vec3<Scalar>{0., 26.55e-3, 0.};
    
        const Mat3<Scalar> R_left_knee_ankle_rotor_2 = R_left;
        const Vec3<Scalar> p_left_knee_ankle_rotor_2 = Vec3<Scalar>{0., -26.55e-3, 0.};
    
        // Right leg
        const Mat3<Scalar> R_right_hip_clamp = Mat3<Scalar>::Identity();
        const Vec3<Scalar> p_right_hip_clamp = Vec3<Scalar>{0., -126e-3, -87e-3};
    
        const Mat3<Scalar> R_right_gimbal = Mat3<Scalar>::Identity();
        const Vec3<Scalar> p_right_gimbal = Vec3<Scalar>{0., 0., -142.5e-3};
    
        const Mat3<Scalar> R_right_thigh = Mat3<Scalar>::Identity();
        const Vec3<Scalar> p_right_thigh = Vec3<Scalar>::Zero();
    
        const Mat3<Scalar> R_right_shin = Mat3<Scalar>::Identity();
        const Vec3<Scalar> p_right_shin = Vec3<Scalar>{0., 0., -226.8e-3};
    
        const Mat3<Scalar> R_right_foot = Mat3<Scalar>::Identity();
        const Vec3<Scalar> p_right_foot = Vec3<Scalar>{0., 0., -260e-3};
    
        const Mat3<Scalar> R_right_hip_clamp_rotor = R_down;
        const Vec3<Scalar> p_right_hip_clamp_rotor = Vec3<Scalar>{0., -126e-3, -26e-3};
    
        const Mat3<Scalar> R_right_hip_rotor_1 = R_left;
        const Vec3<Scalar> p_right_hip_rotor_1 = Vec3<Scalar>{0., 0.04, 0.};
    
        const Mat3<Scalar> R_right_hip_rotor_2 = R_right;
        const Vec3<Scalar> p_right_hip_rotor_2 = Vec3<Scalar>{0., -0.04, 0.};
    
        const Mat3<Scalar> R_right_knee_ankle_rotor_1 = R_right;
        const Vec3<Scalar> p_right_knee_ankle_rotor_1 = Vec3<Scalar>{0., 26.55e-3, 0.};
    
        const Mat3<Scalar> R_right_knee_ankle_rotor_2 = R_left;
        const Vec3<Scalar> p_right_knee_ankle_rotor_2 = Vec3<Scalar>{0., -26.55e-3, 0.};
    
        const Scalar torso_mass = 2.3008;
        const Vec3<Scalar> torso_CoM = Vec3<Scalar>{0.0073, -0.0013, -0.0023};
        const Mat3<Scalar> torso_inertia = (Mat3<Scalar>() << 0.0366, 0., -0.0006, 0., 0.0142, -0.0002, -0.0006, -0.0002, 0.0291).finished();
    
        const Scalar gear_ratio = 6.0;
    
        const Scalar hip_clamp_mass = 1.3289;
        const Vec3<Scalar> hip_clamp_CoM = Vec3<Scalar>{-0.0010, 0., -0.0069};
        const Mat3<Scalar> hip_clamp_inertia = (Mat3<Scalar>() << 0.0032, 0., 0.0001, 0., 0.0033, 0., 0.0001, 0., 0.0027).finished();
    
        const Scalar gimbal_mass = 0.4433;
        const Vec3<Scalar> gimbal_CoM = Vec3<Scalar>{-0.0027, 0., 0.0258};
        const Mat3<Scalar> gimbal_inertia = (Mat3<Scalar>() << 0.0018, 0., 0., 0., 0.0017, 0., 0., 0., 0.0015).finished();
        
        const Scalar thigh_mass = 1.5424;
        const Vec3<Scalar> thigh_CoM = Vec3<Scalar>{0.003, -0.0001, -0.0323};
        const Mat3<Scalar> thigh_inertia = (Mat3<Scalar>() << 0.0103, 0., -0.0005, 0., 0.0097, 0., -0.0005, 0., 0.0027).finished();
    
        const Scalar shin_mass = 0.3072;
        const Vec3<Scalar> shin_CoM = Vec3<Scalar>{0.0047, -0.0003, -0.1043};
        const Mat3<Scalar> shin_inertia = (Mat3<Scalar>() << 0.0054 , -0., -0.0002, -0., 0.0054, 0., -0.0002, 0., 0.0001).finished();
        
        const Scalar foot_mass = 0.1025;
        const Vec3<Scalar> foot_CoM = Vec3<Scalar>{0.0042, -0., -0.0251};
        const Mat3<Scalar> foot_inertia = (Mat3<Scalar>() << 0.094e-3, -0., -0.0038e-3, -0., 0.1773e-3, 0., -0.0038e-3, 0., 0.0901e-3).finished();
    
        const Scalar rotor_mass = 0.07;
        const Vec3<Scalar> rotor_CoM = Vec3<Scalar>::Zero();
        const Mat3<Scalar> rotor_inertia = (Mat3<Scalar>() << 2.5984e-5, 0., 0., 0., 2.5984e-5, 0., 0., 0., 5.1512e-5).finished();
    
        const Scalar hip_clamp_rotor_mass = rotor_mass;
        const Vec3<Scalar> hip_clamp_rotor_CoM = rotor_CoM;
        const Mat3<Scalar> hip_clamp_rotor_inertia = rotor_inertia;
    
        const Scalar hip_rotor_1_mass = rotor_mass;
        const Vec3<Scalar> hip_rotor_1_CoM = rotor_CoM;
        const Mat3<Scalar> hip_rotor_1_inertia = rotor_inertia;
    
        const Scalar hip_rotor_2_mass = rotor_mass;
        const Vec3<Scalar> hip_rotor_2_CoM = rotor_CoM;
        const Mat3<Scalar> hip_rotor_2_inertia = rotor_inertia;
    
        const Scalar knee_ankle_rotor_1_mass = rotor_mass;
        const Vec3<Scalar> knee_ankle_rotor_1_CoM = rotor_CoM;
        const Mat3<Scalar> knee_ankle_rotor_1_inertia = rotor_inertia;
    
        const Scalar knee_ankle_rotor_2_mass = rotor_mass;
        const Vec3<Scalar> knee_ankle_rotor_2_CoM = rotor_CoM;
        const Mat3<Scalar> knee_ankle_rotor_2_inertia = rotor_inertia;
    
        const Scalar _footToeLength = 0.1;
        const Scalar _footHeelLength = 0.05;
        const Scalar _footHeight = 0.041;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_TELLO_H
