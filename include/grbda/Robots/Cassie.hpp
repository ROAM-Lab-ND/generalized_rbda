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
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Cassie() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;

    protected:
        const std::string base = "pelvis";
        const Scalar grav = -9.806;

        // Pelvis
        const Scalar pelvis_mass = 10.33;
        const Vec3<Scalar> pelvis_CoM = Vec3<Scalar>{0.05066, 0.000346, 0.02841};
        const Mat3<Scalar> pelvis_inertia = (Mat3<Scalar>() <<
            0.085821, 1.276e-05, -0.00016022,
            1.276e-05, 0.049222, -0.000414,
            -0.00016022, -0.000414, 0.08626).finished();

        // Hip links
        const Scalar hip_roll_mass = 1.82;
        const Vec3<Scalar> hip_roll_CoM = Vec3<Scalar>{-0.01793, 0.0001, -0.04428};

        const Scalar hip_yaw_mass = 1.171;
        const Vec3<Scalar> hip_yaw_CoM = Vec3<Scalar>{0.0, -1e-05, -0.034277};

        const Scalar hip_pitch_mass = 5.52;
        const Vec3<Scalar> hip_pitch_CoM = Vec3<Scalar>{0.05946, 5e-05, -0.03581};

        const Vec3<Scalar> hip_roll_joint_pos = Vec3<Scalar>{0.021, 0.135, 0.0};
        const Vec3<Scalar> hip_yaw_joint_offset = Vec3<Scalar>{0.0, 0.0, -0.07};
        const Vec3<Scalar> hip_pitch_joint_offset = Vec3<Scalar>{0.0, 0.0, -0.09};

        // Gear ratios
        const Scalar hip_roll_gear = 25.0;
        const Scalar hip_yaw_gear = 25.0;
        const Scalar hip_pitch_gear = 16.0;
        const Scalar knee_gear = 16.0;
        const Scalar foot_gear = 50.0;

        // Upper four-bar cluster bodies
        // achilles-rod: pivot at (0,0,0.045) on hip-pitch, rod length 0.5012
        const Scalar achilles_mass = 0.1567;
        const Vec3<Scalar> achilles_CoM = Vec3<Scalar>{0.24719, 0.0, 0.0};

        // knee+knee-spring+shin combined (single rigid body, pivot at knee joint on hip-pitch)
        // Lumps knee (0.7578 kg), knee-spring (0.186 kg), shin (0.577 kg)
        const Scalar knee_shin_mass = 1.5208;
        const Vec3<Scalar> knee_shin_CoM = Vec3<Scalar>{0.12170, 0.04491, -0.00101};

        // tarsus+heel-spring combined (single rigid body, pivot at shin joint on knee+shin body)
        // Lumps tarsus (0.782 kg) and heel-spring (0.126 kg)
        const Scalar tarsus_mass = 0.9080;
        const Vec3<Scalar> tarsus_CoM = Vec3<Scalar>{0.10461, -0.03028, -0.00100};

        // Lower four-bar cluster bodies (children of tarsus+heel-spring)
        const Scalar foot_crank_mass = 0.1261;
        const Vec3<Scalar> foot_crank_CoM = Vec3<Scalar>{0.00493, 2e-05, -0.00215};

        const Scalar plantar_rod_mass = 0.1186;
        const Vec3<Scalar> plantar_rod_CoM = Vec3<Scalar>{0.17792, 0.0, 0.0};

        const Scalar foot_mass = 0.1498;
        const Vec3<Scalar> foot_CoM = Vec3<Scalar>{0.00474, 0.02748, -0.00014};
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_CASSIE_H
