#ifndef GRBDA_ROBOTS_KANGAROO_WITH_CONSTRAINTS_H
#define GRBDA_ROBOTS_KANGAROO_WITH_CONSTRAINTS_H

#include "grbda/Robots/Robot.h"

namespace grbda
{
    /**
     * Kangaroo humanoid robot with closed-loop knee 4-bar constraint
     *
     * This version includes the knee 4-bar linkage mechanism from the
     * Gepetto example-parallel-robots model. The 4-bar connects:
     * - part_1 (femur) via closedloop7_B frame
     * - part_2 via upper_knee joint
     * - part_4 via 4barlink_knee joint
     * - part_5 via sherical_4bar joint back to closedloop7_A on part_5
     *
     * The constraint reduces 3 joint DOF to 1 independent coordinate.
     */
    template <typename Scalar>
    class KangarooWithConstraints : public Robot<Scalar>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        KangarooWithConstraints() {}

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;

    protected:
        const std::string base = "hip_ground";
        const Scalar grav = -9.81;

        // Hip ground inertial properties
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

        // Part 4 (4-bar link 1)
        const Scalar part_4_mass = 0.011311035579799526632;
        const Vec3<Scalar> part_4_CoM = Vec3<Scalar>{
            0.019483726843524864364,
            -0.046047631733748345395,
            0.0};

        // Part 5 (4-bar link 2)
        const Scalar part_5_mass = 0.011107100826766714077;
        const Vec3<Scalar> part_5_CoM = Vec3<Scalar>{
            0.0,
            4.1792121910006938696e-17,
            -0.070709999999999995079};

        // Foot
        const Scalar foot_part_mass = 0.11291432905905610398;
        const Vec3<Scalar> foot_part_CoM = Vec3<Scalar>{
            -0.0071619814024688813028,
            -0.00053847577685908939695,
            0.011291730689193687093};

        // 4-bar linkage geometry from URDF
        // closedloop7_B on part_1: xyz="0.0839 -0.0077 -0.0715"
        // closedloop7_A on part_5: xyz="0 0 -0.1414"
        // 4barlink_knee on part_2: xyz="0.046 -0.0195 0"
        // sherical_4bar on part_4: xyz="0.0195 -0.046 0"
        const Scalar L_closure7_B = 0.0839;  // x-offset from part_1 to closure frame
        const Scalar L_part_5 = 0.1414;      // length of part_5 link
        const Scalar L_4bar_link = 0.050;    // approximate 4-bar link length
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_KANGAROO_WITH_CONSTRAINTS_H
