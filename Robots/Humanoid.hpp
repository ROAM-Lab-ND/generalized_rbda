#ifndef GRBDA_ROBOTS_HUMANOID_H
#define GRBDA_ROBOTS_HUMANOID_H

#include "Humanoid.h"

namespace grbda
{

    class Humanoid : public Robot
    {
    public:
        Humanoid() {}

        ClusterTreeModel buildClusterTreeModel() const override;

    private:
        const double grav = -9.81;

        // Orientation and position of left leg
        const Mat3<double> R_left_hip_p = Mat3<double>::Identity();
        const Vec3<double> p_left_hip_p = Vec3<double>{0., 0.096, 0.};
        const Mat3<double> R_left_hip_p_rotor = R_left_hip_p;
        const Vec3<double> p_left_hip_p_rotor = p_left_hip_p;

        const Mat3<double> R_left_hip_r = Mat3<double>::Identity();
        const Vec3<double> p_left_hip_r = Vec3<double>{0., -2.77e-17, 0.};
        const Mat3<double> R_left_hip_r_rotor = R_left_hip_r;
        const Vec3<double> p_left_hip_r_rotor = p_left_hip_r;

        const Mat3<double> R_left_hip_y = Mat3<double>::Identity();
        const Vec3<double> p_left_hip_y = Vec3<double>{0., -2.77e-17, 0.};
        const Mat3<double> R_left_hip_y_rotor = R_left_hip_y;
        const Vec3<double> p_left_hip_y_rotor = p_left_hip_y;
        
        const Mat3<double> R_left_knee = Mat3<double>::Identity();
        const Vec3<double> p_left_knee = Vec3<double>{-0.02, -2.77e-17, -0.389};
        const Mat3<double> R_left_knee_rotor = R_left_knee;
        const Vec3<double> p_left_knee_rotor = p_left_knee;

        const Mat3<double> R_left_ankle_r = Mat3<double>::Identity();
        const Vec3<double> p_left_ankle_r = Vec3<double>{0.04, 1.52e-16, -0.357};
        const Mat3<double> R_left_ankle_r_rotor = R_left_ankle_r;
        const Vec3<double> p_left_ankle_r_rotor = p_left_ankle_r;

        const Mat3<double> R_left_ankle_p = Mat3<double>::Identity();
        const Vec3<double> p_left_ankle_p = Vec3<double>{-4.86e-17, 3.61e-16, -2.22e-16};
        const Mat3<double> R_left_ankle_p_rotor = R_left_ankle_p;
        const Vec3<double> p_left_ankle_p_rotor = p_left_ankle_p;

        // Orientation and position of right leg
        const Mat3<double> R_right_hip_p = Mat3<double>::Identity();
        const Vec3<double> p_right_hip_p = Vec3<double>{0. -0.096, 0.};
        const Mat3<double> R_right_hip_p_rotor = R_right_hip_p;
        const Vec3<double> p_right_hip_p_rotor = p_right_hip_p;

        const Mat3<double> R_right_hip_r = Mat3<double>::Identity();
        const Vec3<double> p_right_hip_r = Vec3<double>{0., 2.77e-17, 0.};
        const Mat3<double> R_right_hip_r_rotor = R_right_hip_r;
        const Vec3<double> p_right_hip_r_rotor = p_right_hip_r;

        const Mat3<double> R_right_hip_y = Mat3<double>::Identity();
        const Vec3<double> p_right_hip_y = Vec3<double>{0., 2.77e-17, 0.};
        const Mat3<double> R_right_hip_y_rotor = R_right_hip_y;
        const Vec3<double> p_right_hip_y_rotor = p_right_hip_y;
        
        const Mat3<double> R_right_knee = Mat3<double>::Identity();
        const Vec3<double> p_right_knee = Vec3<double>{-0.02, 2.77e-17, -0.389};
        const Mat3<double> R_right_knee_rotor = R_right_knee;
        const Vec3<double> p_right_knee_rotor = p_right_knee;

        const Mat3<double> R_right_ankle_r = Mat3<double>::Identity();
        const Vec3<double> p_right_ankle_r = Vec3<double>{0.04, 1.11e-16, -0.357};
        const Mat3<double> R_right_ankle_r_rotor = R_right_ankle_r;
        const Vec3<double> p_right_ankle_r_rotor = p_right_ankle_r;

        const Mat3<double> R_right_ankle_p = Mat3<double>::Identity();
        const Vec3<double> p_right_ankle_p = Vec3<double>{-1.734e-17, -4.44e-16, -1.11e-16};
        const Mat3<double> R_right_ankle_p_rotor = R_right_ankle_p;
        const Vec3<double> p_right_ankle_p_rotor = p_right_ankle_p;

        // Orientation and position of body
        const Mat3<double> R_waist_y = Mat3<double>::Identity();
        const Vec3<double> p_waist_y = Vec3<double>{0., 0., 0.192};
        const Mat3<double> R_waist_y_rotor = R_waist_y;
        const Vec3<double> p_waist_y_rotor = p_waist_y;

        const Mat3<double> R_waist_p = Mat3<double>::Identity();
        const Vec3<double> p_waist_p = Vec3<double>{0., 0., -1.11e-16};
        const Mat3<double> R_waist_p_rotor = R_waist_p;
        const Vec3<double> p_waist_p_rotor = p_waist_p;

        const Mat3<double> R_waist_r = Mat3<double>::Identity();
        const Vec3<double> p_waist_r = Vec3<double>{0., 0., -1.11e-16};
        const Mat3<double> R_waist_r_rotor = R_waist_r;
        const Vec3<double> p_waist_r_rotor = p_waist_r;

        const Mat3<double> R_neck_y = Mat3<double>::Identity();
        const Vec3<double> p_neck_y = Vec3<double>{-0.003, 0., 0.453};
        const Mat3<double> R_neck_y_rotor = R_neck_y;
        const Vec3<double> p_neck_y_rotor = p_neck_y;

        const Mat3<double> R_neck_r = Mat3<double>::Identity();
        const Vec3<double> p_neck_r = Vec3<double>{8.24e-18, 0., 2.22e-16};
        const Mat3<double> R_neck_r_rotor = R_neck_r;
        const Vec3<double> p_neck_r_rotor = p_neck_r;

        const Mat3<double> R_neck_p = Mat3<double>::Identity();
        const Vec3<double> p_neck_p = Vec3<double>{8.24e-18, 0., 2.22e-16};
        const Mat3<double> R_neck_p_rotor = R_neck_p;
        const Vec3<double> p_neck_p_rotor = p_neck_p;
        
        // Orientation and position of left arm
        const Mat3<double> R_left_shoulder_p = Mat3<double>::Identity();
        const Vec3<double> p_left_shoulder_p = Vec3<double>{0., 0.24, 0.33};
        const Mat3<double> R_left_shoulder_p_rotor = R_left_shoulder_p;
        const Vec3<double> p_left_shoulder_p_rotor = p_left_shoulder_p;

        const Mat3<double> R_left_shoulder_r = Mat3<double>::Identity();
        const Vec3<double> p_left_shoulder_r = Vec3<double>{0., 2.78e-17, -5.55e-16};
        const Mat3<double> R_left_shoulder_r_rotor = R_left_shoulder_r;
        const Vec3<double> p_left_shoulder_r_rotor = p_left_shoulder_r;

        const Mat3<double> R_left_shoulder_y = Mat3<double>::Identity();
        const Vec3<double> p_left_shoulder_y = Vec3<double>{0., 2.78e-17, -5.55e-16};
        const Mat3<double> R_left_shoulder_y_rotor = R_left_shoulder_y;
        const Vec3<double> p_left_shoulder_y_rotor = p_left_shoulder_y;

        const Mat3<double> R_left_elbow_p = Mat3<double>::Identity();
        const Vec3<double> p_left_elbow_p = Vec3<double>{0.004, -2.78e-17, -0.305};
        const Mat3<double> R_left_elbow_p_rotor = R_left_elbow_p;
        const Vec3<double> p_left_elbow_p_left_rotor = p_left_elbow_p;

        const Mat3<double> R_left_elbow_y = Mat3<double>::Identity();
        const Vec3<double> p_left_elbow_y = Vec3<double>{-0.004, 3.89e-16, -0.239};
        const Mat3<double> R_left_elbow_y_rotor = R_left_elbow_y;
        const Vec3<double> p_left_elbow_y_rotor = p_left_elbow_y;

        const Mat3<double> R_left_wrist_r = Mat3<double>::Identity();
        const Vec3<double> p_left_wrist_r = Vec3<double>{0., -2.78e-16, -1.80e-16};
        const Mat3<double> R_left_wrist_r_rotor = R_left_wrist_r;
        const Vec3<double> p_left_wrist_r_rotor = p_left_wrist_r;

        const Mat3<double> R_left_wrist_y = Mat3<double>::Identity();
        const Vec3<double> p_left_wrist_y = Vec3<double>{0., -2.78e-16, -1.80e-16};
        const Mat3<double> R_left_wrist_y_rotor = R_left_wrist_y;
        const Vec3<double> p_left_wrist_y_rotor = p_left_wrist_y;

        // Orientation and position of right arm
        const Mat3<double> R_right_shoulder_p = Mat3<double>::Identity();
        const Vec3<double> p_right_shoulder_p = Vec3<double>{0., -0.24, 0.33};
        const Mat3<double> R_right_shoulder_p_rotor = R_right_shoulder_p;
        const Vec3<double> p_right_shoulder_p_right_rotor = p_right_shoulder_p;

        const Mat3<double> R_right_shoulder_r = Mat3<double>::Identity();
        const Vec3<double> p_right_shoulder_r = Vec3<double>{0., -2.78e-17, -7.77e-16};
        const Mat3<double> R_right_shoulder_r_rotor = R_right_shoulder_r;
        const Vec3<double> p_right_shoulder_r_rotor = p_right_shoulder_r;

        const Mat3<double> R_right_shoulder_y = Mat3<double>::Identity();
        const Vec3<double> p_right_shoulder_y = Vec3<double>{0., -2.78e-17, -7.77e-16};
        const Mat3<double> R_right_shoulder_y_rotor = R_right_shoulder_y;
        const Vec3<double> p_right_shoulder_y_rotor = p_right_shoulder_y;

        const Mat3<double> R_right_elbow_p = Mat3<double>::Identity();
        const Vec3<double> p_right_elbow_p = Vec3<double>{0.004, 2.78e-17, -0.305};
        const Mat3<double> R_right_elbow_p_rotor = R_right_elbow_p;
        const Vec3<double> p_right_elbow_p_rotor = p_right_elbow_p;

        const Mat3<double> R_right_elbow_y = Mat3<double>::Identity();
        const Vec3<double> p_right_elbow_y = Vec3<double>{-0.004, -5e-16, -0.239};
        const Mat3<double> R_right_elbow_y_rotor = R_right_elbow_y;
        const Vec3<double> p_right_elbow_y_rotor = p_right_elbow_y;

        const Mat3<double> R_right_wrist_r = Mat3<double>::Identity();
        const Vec3<double> p_right_wrist_r = Vec3<double>{0., 2.78e-16, -1.80e-16};
        const Mat3<double> R_right_wrist_r_rotor = R_right_wrist_r;
        const Vec3<double> p_right_wrist_r_rotor = p_right_wrist_r;

        const Mat3<double> R_right_wrist_y = Mat3<double>::Identity();
        const Vec3<double> p_right_wrist_y = Vec3<double>{0., 2.78e-16, -1.80e-16};
        const Mat3<double> R_right_wrist_y_rotor = R_right_wrist_y;
        const Vec3<double> p_right_wrist_y_rotor = p_right_wrist_y;

        // mass, CoM and inertia values
        const double gear_ratio = 6.;
        const double rotor_mass = 0.07;
        const Vec3<double> rotor_CoM = Vec3<doube>::Zero();
        const Mat3<double> rotor_inertia = (Mat3<double>() << 2e-5, 0., 0., 0., 2e-5, 0., 0., 0., 5e-5).finished();

        const double pelvis_mass = 10.;
        const Vec3<double> pelvis_CoM = Vec3<double>{-0.01, 0., 0.034};
        const Mat3<double> pelvis_inertia = (Mat3<double>() << 0.089583, 0., 0., 0., 0.089583, 0., 0., 0., 0.1125).finished();

        const double hip_p_mass = 1.;
        const Vec3<double> hip_p_CoM = Vec3<double>{0., 0., 0.};
        const Mat3<double> hip_p_inertia = (Mat3<double>() << 0.00196, 0., 0., 0., 0.00196, 0., 0., 0., 0.00196).finished();

        const double hip_r_mass = 1.; 
        const Vec3<double> hip_r_CoM = Vec3<double>{0., 0., 0.};
        const Mat3<double> hip_r_inertia = (Mat3<double>() << 0.00196, 0., 0., 0., 0.00196, 0., 0., 0., 0.00196).finished();

        const double hip_y_mass = 3.; 
        const Vec3<double> hip_y_CoM = Vec3<double>{0.01, 0., -0.22};
        const Mat3<double> hip_y_inertia = (Mat3<double>() << 0.031925, 0., 0., 0., 0.034525, 0., 0., 0., 0.00865).finished();

        const double knee_mass = 3.; 
        const Vec3<double> knee_CoM = Vec3<double>{0.04, 0., -0.16};
        const Mat3<double> knee_inertia = (Mat3<double>() << 0.031925, 0., 0., 0., 0.034525, 0., 0., 0., 0.00865).finished();

        const double ankle_r_mass = 1.; 
        const Vec3<double> ankle_r_CoM = Vec3<double>{0., 0., 0.};
        const Mat3<double> knee_inertia = (Mat3<double>() << 0.00064, 0., 0., 0., 0.00064, 0., 0., 0., 0.00064).finished();

        const double ankle_p_mass = 1.5; 
        const Vec3<double> ankle_p_CoM = Vec3<double>{0.03, 0., -0.07};
        const Mat3<double> knee_inertia = (Mat3<double>() << 0.001417, 0., 0., 0., 0.005617, 0., 0., 0., 0.006217).finished();
        
        const double waist_y_mass = 1.; 
        const Vec3<double> waist_y_CoM = Vec3<double>{0., 0., -0.07};
        const Mat3<double> waist_y_inertia = (Mat3<double>() << 0.00173, 0., 0., 0., 0.00173, 0., 0., 0., 0.0032).finished();

        const double waist_p_mass = 1.; 
        const Vec3<double> waist_p_CoM = Vec3<double>{0., 0., 0.};
        const Mat3<double> waist_p_inertia = (Mat3<double>() << 0.002425, 0., 0., 0., 0.002425, 0., 0., 0., 0.002425).finished();

        const double waist_r_mass = 10.; 
        const Vec3<double> waist_r_CoM = Vec3<double>{0.02, 0., 0.24};
        const Mat3<double> waist_r_inertia = (Mat3<double>() << 0.157083, 0., 0., 0., 0.101083, 0., 0., 0., 0.1367).finished();

        const double neck_y_mass = 0.5; 
        const Vec3<double> neck_y_CoM = Vec3<double>{0., 0., -0.05};
        const Mat3<double> neck_y_inertia = (Mat3<double>() << 0.000729167, 0., 0., 0., 0.000729167, 0., 0., 0., 0.000625).finished();

        const double neck_r_mass = 0.5; 
        const Vec3<double> neck_r_CoM = Vec3<double>{0., 0., 0.};
        const Mat3<double> neck_r_inertia = (Mat3<double>() << 0.0005, 0., 0., 0., 0.0005, 0., 0., 0., 0.0005).finished();

        const double neck_p_mass = 2.; 
        const Vec3<double> neck_p_CoM = Vec3<double>{0.01, 0., 0.11};
        const Mat3<double> neck_p_inertia = (Mat3<double>() << 0.00968, 0., 0., 0., 0.00968, 0., 0., 0., 0.00968).finished();

        const double shoulder_p_mass = 1.; 
        const Vec3<double> shoulder_p_CoM = Vec3<double>{0., 0., 0.};
        const Mat3<double> shoulder_p_inertia = (Mat3<double>() << 0.00196, 0., 0., 0., 0.00196, 0., 0., 0., 0.00196).finished();

        const double shoulder_r_mass = 1.; 
        const Vec3<double> shoulder_r_CoM = Vec3<double>{0., 0., 0.};
        const Mat3<double> shoulder_r_inertia = (Mat3<double>() << 0.00196, 0., 0., 0., 0.00196, 0., 0., 0., 0.00196).finished();

        const double shoulder_y_mass = 2.; 
        const Vec3<double> shoulder_y_CoM = Vec3<double>{-0.01, 0., -0.19};
        const Mat3<double> shoulder_y_inertia = (Mat3<double>() << 0.01365, 0., 0., 0., 0.0146, 0., 0., 0., 0.00635).finished();

        const double elbow_p_mass = 1.; 
        const Vec3<double> elbow_p_CoM = Vec3<double>{-0.02, 0., -0.1};
        const Mat3<double> elbow_p_inertia = (Mat3<double>() << 0.010675, 0., 0., 0., 0.010675, 0., 0., 0., 0.0027).finished();

        const double elbow_y_mass = 1.; 
        const Vec3<double> elbow_y_CoM = Vec3<double>{0., 0., 0.};
        const Mat3<double> elbow_y_inertia = (Mat3<double>() << 0.00064, 0., 0., 0., 0.00064, 0., 0., 0., 0.00064).finished();

        const double wrist_r_mass = 0.5; 
        const Vec3<double> wrist_r_CoM = Vec3<double>{0., 0., 0.};
        const Mat3<double> wrist_r_inertia = (Mat3<double>() << 0.00032, 0., 0., 0., 0.00032, 0., 0., 0., 0.00032).finished();

        const double wrist_y_mass = 0.5; 
        const Vec3<double> left_wrist_y_CoM = Vec3<double>{0., -0.01, -0.06};
        const Vec3<double> right_wrist_y_CoM = Vec3<double>{0., 0.01, -0.06};
        const Mat3<double> wrist_y_inertia = (Mat3<double>() << 0.0004625, 0., 0., 0., 0.0007625, 0., 0., 0., 0.0004625).finished();
    }
}
