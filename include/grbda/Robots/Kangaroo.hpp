#ifndef GRBDA_ROBOTS_KANGAROO_H
#define GRBDA_ROBOTS_KANGAROO_H

#include "grbda/Robots/Robot.h"

namespace grbda
{
    /**
     * Kangaroo humanoid robot from PAL Robotics (open-chain version)
     *
     * Based on:
     * - PAL Robotics Kangaroo description: https://github.com/pal-robotics/kangaroo_robot
     * - Gepetto parallel robots: https://github.com/Gepetto/example-parallel-robots
     */
    template <typename Scalar>
    class Kangaroo : public Robot<Scalar>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Kangaroo() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;

    protected:
        const std::string base = "hip_ground";
        const Scalar grav = -9.81;

        // Hip ground inertial properties from Gepetto URDF
        const Scalar hip_ground_mass = 1.5865591647149870358;
        const Vec3<Scalar> hip_ground_CoM = Vec3<Scalar>{
            -0.0014786115366831747986,
            0.1276542468606598757,
            0.024304995635825776823};
        const Mat3<Scalar> hip_ground_inertia = (Mat3<Scalar>() <<
            0.0088382827967698161797, 0.0002748776218202433853, -2.920764549681984781e-05,
            0.0002748776218202433853, 0.0025754201921633630527, 6.8858407992517019366e-05,
            -2.920764549681984781e-05, 6.8858407992517019366e-05, 0.010752178054608726651).finished();

        // Hip part
        const Scalar hip_part_mass = 0.1490882064698049736;
        const Vec3<Scalar> hip_part_CoM = Vec3<Scalar>{
            3.4156793823035547231e-08,
            0.0030510835733530833336,
            -0.083365236807579123912};
        const Mat3<Scalar> hip_part_inertia = (Mat3<Scalar>() <<
            0.00035636864137915798224, 1.6418959534102587027e-10, -6.6325805549172388776e-11,
            1.6418959534102587027e-10, 0.00032228379449719354345, -1.6655006554546136825e-05,
            -6.6325805549172388776e-11, -1.6655006554546136825e-05, 4.9201065872666322333e-05).finished();

        // Part 1 (femur)
        const Scalar part_1_mass = 0.24432174426363428843;
        const Vec3<Scalar> part_1_CoM = Vec3<Scalar>{
            0.0067026039127809581425,
            0.016607315554245553196,
            -0.045523329198624416791};
        const Mat3<Scalar> part_1_inertia = (Mat3<Scalar>() <<
            0.00038002986328721221352, -4.5343822480551211444e-05, 0.00019115477962150834884,
            -4.5343822480551211444e-05, 0.00063262805206548233892, 7.0096670808240417656e-05,
            0.00019115477962150834884, 7.0096670808240417656e-05, 0.00036002210019491686028).finished();

        // Part 2 (thigh)
        const Scalar part_2_mass = 0.41058769355584682869;
        const Vec3<Scalar> part_2_CoM = Vec3<Scalar>{
            0.0018628398299891867629,
            -0.017878001168809584588,
            -0.13478999979714813949};

        // Part 3 (shin)
        const Scalar part_3_mass = 0.18697741691093650556;
        const Vec3<Scalar> part_3_CoM = Vec3<Scalar>{
            -0.0099474959730298698401,
            0.015679430879779682523,
            0.055687419481867003174};

        // Foot
        const Scalar foot_part_mass = 0.11291432905905610398;
        const Vec3<Scalar> foot_part_CoM = Vec3<Scalar>{
            -0.0071619814024688813028,
            -0.00053847577685908939695,
            0.011291730689193687093};
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_KANGAROO_H
