#ifndef GRBDA_ROBOTS_KANGAROO_H
#define GRBDA_ROBOTS_KANGAROO_H

#include "grbda/Robots/Robot.h"

namespace grbda
{
    /**
     * Kangaroo humanoid robot from PAL Robotics
     *
     * Features:
     * - Serial-parallel hybrid leg mechanism with linear actuators
     * - 6 actuators per leg (hip_z, hip_xy1, hip_xy2, knee, ankle1, ankle2)
     * - 11 closed-loop constraints per leg for full parallel mechanism
     * - Floating base (6 DOF)
     *
     * Based on:
     * - PAL Robotics Kangaroo description: https://github.com/pal-robotics/kangaroo_robot
     * - Gepetto parallel robots: https://github.com/Gepetto/example-parallel-robots
     *
     * The leg mechanism uses linear actuators near the pelvis with motion
     * transferred through a complex system of parallel linkages to achieve
     * low inertia at the end-effectors and robust impact handling.
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

        // =====================================================================
        // Hip ground (base) inertial properties from Gepetto URDF
        // =====================================================================
        const Scalar hip_ground_mass = 1.5865591647149870358;
        const Vec3<Scalar> hip_ground_CoM = Vec3<Scalar>{
            -0.0014786115366831747986,
            0.1276542468606598757,
            0.024304995635825776823};
        const Mat3<Scalar> hip_ground_inertia = (Mat3<Scalar>() <<
            0.0088382827967698161797, 0.0002748776218202433853, -2.920764549681984781e-05,
            0.0002748776218202433853, 0.0025754201921633630527, 6.8858407992517019366e-05,
            -2.920764549681984781e-05, 6.8858407992517019366e-05, 0.010752178054608726651).finished();

        // =====================================================================
        // Hip part (rotating about Z from hip_ground)
        // =====================================================================
        const Scalar hip_part_mass = 0.1490882064698049736;
        const Vec3<Scalar> hip_part_CoM = Vec3<Scalar>{
            3.4156793823035547231e-08,
            0.0030510835733530833336,
            -0.083365236807579123912};
        const Mat3<Scalar> hip_part_inertia = (Mat3<Scalar>() <<
            0.00035636864137915798224, 1.6418959534102587027e-10, -6.6325805549172388776e-11,
            1.6418959534102587027e-10, 0.00032228379449719354345, -1.6655006554546136825e-05,
            -6.6325805549172388776e-11, -1.6655006554546136825e-05, 4.9201065872666322333e-05).finished();

        // Hip part transform from hip_ground
        const Vec3<Scalar> hip_part_pos = Vec3<Scalar>{
            0.0055000000000023745172 - 0.018000000000009067774,
            0.071500000000000035749 - 0.27650000000001295763,
            -0.93500000000000038636 + 0.9400000000000158229};

        // =====================================================================
        // Part 1 (upper leg link)
        // =====================================================================
        const Scalar part_1_mass = 0.24432174426363428843;
        const Vec3<Scalar> part_1_CoM = Vec3<Scalar>{
            0.0067026039127809581425,
            0.016607315554245553196,
            -0.045523329198624416791};
        const Mat3<Scalar> part_1_inertia = (Mat3<Scalar>() <<
            0.00038002986328721221352, -4.5343822480551211444e-05, 0.00019115477962150834884,
            -4.5343822480551211444e-05, 0.00063262805206548233892, 7.0096670808240417656e-05,
            0.00019115477962150834884, 7.0096670808240417656e-05, 0.00036002210019491686028).finished();

        // =====================================================================
        // Part 2 (mid leg link)
        // =====================================================================
        const Scalar part_2_mass = 0.41058769355584682869;
        const Vec3<Scalar> part_2_CoM = Vec3<Scalar>{
            0.0018628398299891867629,
            -0.017878001168809584588,
            -0.13478999979714813949};
        const Mat3<Scalar> part_2_inertia = (Mat3<Scalar>() <<
            0.0024807227695316702089, -8.8419693915619252527e-06, 6.1050621820855792428e-05,
            -8.8419693915619252527e-06, 0.002509655792655584541, 0.00052169893684878131106,
            6.1050621820855792428e-05, 0.00052169893684878131106, 0.00012497096972285892227).finished();

        // =====================================================================
        // Part 3 (lower leg link)
        // =====================================================================
        const Scalar part_3_mass = 0.18697741691093650556;
        const Vec3<Scalar> part_3_CoM = Vec3<Scalar>{
            -0.0099474959730298698401,
            0.015679430879779682523,
            0.055687419481867003174};
        const Mat3<Scalar> part_3_inertia = (Mat3<Scalar>() <<
            0.00025376131655679006285, -2.4556108001626116814e-05, -5.2067851851680689693e-05,
            -2.4556108001626116814e-05, 0.00034866085095936891011, 3.1006802741406406266e-05,
            -5.2067851851680689693e-05, 3.1006802741406406266e-05, 0.00016119645298810645816).finished();

        // =====================================================================
        // Foot part
        // =====================================================================
        const Scalar foot_part_mass = 0.11291432905905610398;
        const Vec3<Scalar> foot_part_CoM = Vec3<Scalar>{
            -0.0071619814024688813028,
            -0.00053847577685908939695,
            0.011291730689193687093};
        const Mat3<Scalar> foot_part_inertia = (Mat3<Scalar>() <<
            2.9706279932379499627e-05, -1.4680115626376891992e-07, 5.1195645166037379656e-06,
            -1.4680115626376891992e-07, 7.6890866395668632684e-05, 3.3336113831889025426e-07,
            5.1195645166037379656e-06, 3.3336113831889025426e-07, 6.3853389929295598152e-05).finished();

        // =====================================================================
        // Universal foot (ankle joint)
        // =====================================================================
        const Scalar universal_foot_mass = 0.0064135211594024193679;
        const Vec3<Scalar> universal_foot_CoM = Vec3<Scalar>{
            6.7228290681873054761e-09,
            -1.3877787807814456755e-17,
            -6.5993277170931378462e-05};

        // =====================================================================
        // Motor masses (prismatic actuators)
        // =====================================================================
        const Scalar motor_mass = 0.5;  // Approximate motor mass

        // =====================================================================
        // Joint limits from PAL Robotics URDF (converted to radians)
        // =====================================================================
        const Scalar deg_to_rad = 0.01745329251994329577;
        const Scalar leg_1_min = -15.0 * deg_to_rad;  // hip yaw
        const Scalar leg_1_max = 40.0 * deg_to_rad;
        const Scalar leg_2_min = -42.5 * deg_to_rad;  // hip pitch
        const Scalar leg_2_max = 38.0 * deg_to_rad;
        const Scalar leg_3_min = -27.0 * deg_to_rad;  // hip roll
        const Scalar leg_3_max = 27.0 * deg_to_rad;

        // Linear actuator limits
        const Scalar leg_min_length = 0.131973;
        const Scalar leg_max_length = 0.71416639;

        // =====================================================================
        // Gear ratios / transmission multipliers from PAL URDF mimic joints
        // These approximate the parallel linkage kinematics
        // =====================================================================
        const Scalar femur_multiplier = 1.69435;
        const Scalar femur_offset = -0.096145;
        const Scalar knee_multiplier = 3.38877;
        const Scalar knee_offset = -0.19229;
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_KANGAROO_H
