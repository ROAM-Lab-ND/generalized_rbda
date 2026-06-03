#ifndef GRBDA_ROBOTS_CASSIE_H
#define GRBDA_ROBOTS_CASSIE_H

#include "grbda/Robots/Robot.h"

namespace grbda
{
    /**
     * Cassie biped robot from Agility Robotics.
     *
     * Model topology follows the Cassie MJCF in MuJoCo Menagerie
     * (agility_cassie/cassie.xml). Each leg has two FourBar clusters:
     *
     *   Upper four-bar (parent = hip-pitch):
     *     path1: knee+shin (link 1) -> tarsus+heel-spring (link 2)
     *     path2: achilles-rod (closure via achilles tip to heel-spring pivot)
     *
     *   Lower four-bar (parent = tarsus+heel-spring body):
     *     path1: foot-crank (link 1) -> plantar-rod (link 2)
     *     path2: foot (zero-length rocker; closure at foot pivot)
     */
    template <typename Scalar>
    class Cassie : public Robot<Scalar>
    {
    public:

        Cassie() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;

    protected:
        const std::string base = "pelvis";
        const Scalar grav = -9.806;

        // Pelvis
        const Scalar pelvis_mass = 10.33;
        const Vec3<Scalar> pelvis_CoM = Vec3<Scalar>{0.05066, 0.000346, 0.02841};
        const Mat3<Scalar> pelvis_inertia = (Mat3<Scalar>() <<
            0.085821,  1.276e-05, -0.00016022,
            1.276e-05,  0.049222, -0.000414,
           -0.00016022, -0.000414,  0.08626).finished();

        // Hip-roll (symmetric across sides — CoM z is 0 in MJCF xyaxes frame)
        const Scalar hip_roll_mass = 1.82;
        // CoM in hip-roll body frame. y-component is identical for both sides in the MJCF.
        const Vec3<Scalar> hip_roll_CoM = Vec3<Scalar>{-0.01793, 0.0001, -0.04428};
        // fullinertia = Ixx Iyy Izz Ixy Ixz Iyz (symmetric, left side; right side flips Ixy,Iyz signs)
        const Mat3<Scalar> hip_roll_inertia_left = (Mat3<Scalar>() <<
            0.003431, -6.65e-07, -0.00084,
           -6.65e-07,  0.003793,  3.99e-06,
           -0.00084,   3.99e-06,  0.002135).finished();
        const Mat3<Scalar> hip_roll_inertia_right = (Mat3<Scalar>() <<
            0.003431,  6.65e-07, -0.00084,
            6.65e-07,  0.003793, -3.99e-06,
           -0.00084,  -3.99e-06,  0.002135).finished();

        // Hip-yaw (CoM y flips sign between sides)
        const Scalar hip_yaw_mass = 1.171;
        const Vec3<Scalar> hip_yaw_CoM_left  = Vec3<Scalar>{0.0, -1e-05, -0.034277};
        const Vec3<Scalar> hip_yaw_CoM_right = Vec3<Scalar>{0.0,  1e-05, -0.034277};
        const Mat3<Scalar> hip_yaw_inertia_left = (Mat3<Scalar>() <<
            0.002443, -4e-08,     2.462e-07,
           -4e-08,     0.002803, -2.71e-08,
            2.462e-07,-2.71e-08,  0.000842).finished();
        const Mat3<Scalar> hip_yaw_inertia_right = (Mat3<Scalar>() <<
            0.002443,  4e-08,     2.462e-07,
            4e-08,     0.002803,  2.71e-08,
            2.462e-07, 2.71e-08,  0.000842).finished();

        // Hip-pitch (CoM z flips sign between sides)
        const Scalar hip_pitch_mass = 5.52;
        const Vec3<Scalar> hip_pitch_CoM_left  = Vec3<Scalar>{0.05946, 5e-05, -0.03581};
        const Vec3<Scalar> hip_pitch_CoM_right = Vec3<Scalar>{0.05946, 5e-05,  0.03581};
        const Mat3<Scalar> hip_pitch_inertia_left = (Mat3<Scalar>() <<
            0.010898, -0.0002669, -5.721e-05,
           -0.0002669,  0.029714,  9.17e-06,
           -5.721e-05,  9.17e-06,  0.030257).finished();
        const Mat3<Scalar> hip_pitch_inertia_right = (Mat3<Scalar>() <<
            0.010898, -0.0002669,  5.721e-05,
           -0.0002669,  0.029714, -9.17e-06,
            5.721e-05, -9.17e-06,  0.030257).finished();

        const Vec3<Scalar> hip_roll_joint_pos = Vec3<Scalar>{0.021, 0.135, 0.0};
        const Vec3<Scalar> hip_yaw_joint_offset = Vec3<Scalar>{0.0, 0.0, -0.07};
        const Vec3<Scalar> hip_pitch_joint_offset = Vec3<Scalar>{0.0, 0.0, -0.09};

        // Gear ratios
        const Scalar hip_roll_gear = 25.0;
        const Scalar hip_yaw_gear = 25.0;
        const Scalar hip_pitch_gear = 16.0;
        const Scalar knee_gear = 16.0;
        const Scalar foot_gear = 50.0;

        // Achilles-rod (symmetric across sides; CoM on x-axis only)
        const Scalar achilles_mass = 0.1567;
        const Vec3<Scalar> achilles_CoM = Vec3<Scalar>{0.24719, 0.0, 0.0};
        const Mat3<Scalar> achilles_inertia = (Mat3<Scalar>() <<
            3.754e-06, -3.74e-08, -1.61e-08,
           -3.74e-08,   0.004487,  0.0,
           -1.61e-08,   0.0,       0.004488).finished();

        // knee+knee-spring+shin combined (single rigid body in knee body frame).
        // Lumps knee (0.7578 kg), knee-spring (0.186 kg), shin (0.577 kg).
        // CoM and inertia are in the knee body frame; z-component of CoM flips between sides.
        const Scalar knee_shin_mass = 1.5208;
        const Vec3<Scalar> knee_shin_CoM_left  = Vec3<Scalar>{0.12170, 0.04491, -0.00101};
        const Vec3<Scalar> knee_shin_CoM_right = Vec3<Scalar>{0.12170, 0.04491,  0.00101};
        // Lumped inertia computed via parallel-axis theorem from MJCF values.
        // Left and right differ only in Ixz and Iyz sign.
        const Mat3<Scalar> knee_shin_inertia_left = (Mat3<Scalar>() <<
            0.002032, -0.000641, -5.8e-05,
           -0.000641,  0.016670, -2.1e-05,
           -5.8e-05,  -2.1e-05,   0.017630).finished();
        const Mat3<Scalar> knee_shin_inertia_right = (Mat3<Scalar>() <<
            0.002032, -0.000641,  5.8e-05,
           -0.000641,  0.016670,  2.1e-05,
            5.8e-05,   2.1e-05,   0.017630).finished();

        // tarsus+heel-spring combined (single rigid body in tarsus body frame).
        // Lumps tarsus (0.782 kg) and heel-spring (0.126 kg).
        // CoM z-component flips between sides.
        const Scalar tarsus_mass = 0.9080;
        const Vec3<Scalar> tarsus_CoM_left  = Vec3<Scalar>{0.10461, -0.03028, -0.00100};
        const Vec3<Scalar> tarsus_CoM_right = Vec3<Scalar>{0.10461, -0.03028,  0.00100};
        const Mat3<Scalar> tarsus_inertia_left = (Mat3<Scalar>() <<
            0.000453,  0.000257, -5.2e-05,
            0.000257,  0.013817, -4.9e-05,
           -5.2e-05,  -4.9e-05,  0.013897).finished();
        const Mat3<Scalar> tarsus_inertia_right = (Mat3<Scalar>() <<
            0.000453,  0.000257,  5.2e-05,
            0.000257,  0.013817,  4.9e-05,
            5.2e-05,   4.9e-05,   0.013897).finished();

        // Foot-crank (CoM z flips between sides)
        const Scalar foot_crank_mass = 0.1261;
        const Vec3<Scalar> foot_crank_CoM_left  = Vec3<Scalar>{0.00493,  2e-05, -0.00215};
        const Vec3<Scalar> foot_crank_CoM_right = Vec3<Scalar>{0.00493,  2e-05,  0.00215};
        const Mat3<Scalar> foot_crank_inertia_left = (Mat3<Scalar>() <<
            2.6941e-05, -2.1e-09,  -3.9623e-06,
           -2.1e-09,     4.9621e-05, -1.09e-08,
           -3.9623e-06, -1.09e-08,   6.3362e-05).finished();
        const Mat3<Scalar> foot_crank_inertia_right = (Mat3<Scalar>() <<
            2.6941e-05, -2.1e-09,   3.9623e-06,
           -2.1e-09,     4.9621e-05, 1.09e-08,
            3.9623e-06,  1.09e-08,   6.3362e-05).finished();

        // Plantar-rod (symmetric; CoM on x-axis only)
        const Scalar plantar_rod_mass = 0.1186;
        const Vec3<Scalar> plantar_rod_CoM = Vec3<Scalar>{0.17792, 0.0, 0.0};
        const Mat3<Scalar> plantar_rod_inertia = (Mat3<Scalar>() <<
            2.779e-06, -2.34e-08, -8.1e-09,
           -2.34e-08,   0.001774,  0.0,
           -8.1e-09,    0.0,       0.001775).finished();

        // Foot (CoM z flips between sides)
        const Scalar foot_mass = 0.1498;
        const Vec3<Scalar> foot_CoM_left  = Vec3<Scalar>{0.00474, 0.02748, -0.00014};
        const Vec3<Scalar> foot_CoM_right = Vec3<Scalar>{0.00474, 0.02748,  0.00014};
        const Mat3<Scalar> foot_inertia_left = (Mat3<Scalar>() <<
            0.00017388,  0.00011814,  1.36e-06,
            0.00011814,  0.00016793, -4e-07,
            1.36e-06,   -4e-07,       0.00033261).finished();
        const Mat3<Scalar> foot_inertia_right = (Mat3<Scalar>() <<
            0.00017388,  0.00011814, -1.36e-06,
            0.00011814,  0.00016793,  4e-07,
           -1.36e-06,    4e-07,       0.00033261).finished();
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_CASSIE_H
