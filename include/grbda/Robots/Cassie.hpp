#ifndef GRBDA_ROBOTS_CASSIE_H
#define GRBDA_ROBOTS_CASSIE_H

#include "grbda/Robots/Robot.h"

namespace grbda
{
    /**
     * Cassie biped robot from Agility Robotics.
     *
     * Source-driven benchmark model built from the Cassie MJCF layout in
     * MuJoCo Menagerie (`agility_cassie/cassie.xml`).
     *
     * Notes:
     * - Pelvis and hip inertias/offsets are lifted from Cassie MJCF.
     * - Lower-leg closure is represented with the existing FourBar cluster type,
     *   parameterized from Cassie linkage anchor lengths (0.35012 and 0.5012).
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

        const Scalar pelvis_mass = 10.33;
        const Vec3<Scalar> pelvis_CoM = Vec3<Scalar>{0.05066, 0.000346, 0.02841};
        const Mat3<Scalar> pelvis_inertia = (Mat3<Scalar>() <<
            0.085821, 1.276e-05, -0.00016022,
            1.276e-05, 0.049222, -0.000414,
            -0.00016022, -0.000414, 0.08626).finished();

        const Scalar hip_roll_mass = 1.82;
        const Vec3<Scalar> hip_roll_CoM = Vec3<Scalar>{-0.01793, 0.0001, -0.04428};

        const Scalar hip_yaw_mass = 1.171;
        const Vec3<Scalar> hip_yaw_CoM = Vec3<Scalar>{0.0, -1e-05, -0.034277};

        const Scalar hip_pitch_mass = 5.52;
        const Vec3<Scalar> hip_pitch_CoM = Vec3<Scalar>{0.05946, 5e-05, -0.03581};

        const Vec3<Scalar> hip_roll_joint_pos = Vec3<Scalar>{0.021, 0.135, 0.0};
        const Vec3<Scalar> hip_yaw_joint_offset = Vec3<Scalar>{0.0, 0.0, -0.07};
        const Vec3<Scalar> hip_pitch_joint_offset = Vec3<Scalar>{0.0, 0.0, -0.09};

        const Scalar hip_roll_gear = 25.0;
        const Scalar hip_yaw_gear = 25.0;
        const Scalar hip_pitch_gear = 16.0;
        const Scalar knee_gear = 16.0;
        const Scalar foot_gear = 50.0;

        const Scalar achilles_mass = 0.1567;
        const Vec3<Scalar> achilles_CoM = Vec3<Scalar>{0.24719, 0.0, 0.0};

        const Scalar knee_mass = 0.7578;
        const Vec3<Scalar> knee_CoM = Vec3<Scalar>{0.023, 0.03207, -0.002181};

        const Scalar foot_mass = 0.1498;
        const Vec3<Scalar> foot_CoM = Vec3<Scalar>{0.00474, 0.02748, -0.00014};
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_CASSIE_H