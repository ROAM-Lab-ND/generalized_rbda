#ifndef GRBDA_ROBOTS_MIT_HUMANOID_H
#define GRBDA_ROBOTS_MIT_HUMANOID_H

#include "Robot.h"

namespace grbda
{

    template <typename Scalar = double, typename OrientationRepresentation = ori_representation::QuaternionRepresentation< Quat<Scalar> >>
    class MIT_Humanoid : public Robot<Scalar>
    {
    public:
        MIT_Humanoid()
        {
            _torsoRotInertia << 0.172699, 0.001419, 0.004023,
                0.001419, 0.105949, -0.001672,
                0.004023, -0.001672, 0.091906;
            _hipRzRotInertia << 0.0015373, 0.0000011, 0.0005578,
                0.0000011, 0.0014252, 0.0000024,
                0.0005578, 0.0000024, 0.0012028;
            _hipRxRotInertia << 0.0017535, -0.0000063, -0.000080,
                -0.0000063, 0.003338, -0.000013,
                -0.000080, -0.000013, 0.0019927;
            _hipRyRotInertia << 0.0243761, 0.0000996, 0.0006548,
                0.0000996, 0.0259015, 0.0026713,
                0.0006548, 0.0026713, 0.0038929;
            _kneeRotInertia << 0.003051, 0.000000, 0.0000873,
                0.000000, 0.003033, 0.0000393,
                0.0000873, 0.0000393, 0.0002529;
            _ankleRotInertia << 0.0000842, 0.000000, -0.0000488,
                0.000000, 0.0007959, -0.000000,
                -0.0000488, -0.000000, 0.0007681;
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

            _largeRotorRotationalInertiaZ << 3.443e-4, 0, 0,
                0, 3.443e-4, 0,
                0, 0, 5.548e-4;
            _smallRotorRotationalInertiaZ << 1.084e-4, 0, 0,
                0, 1.084e-4, 0,
                0, 0, 1.6841e-4;

            Mat3<Scalar> RY = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::Y, M_PI / 2);
            Mat3<Scalar> RX = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::X, -M_PI / 2);

            Mat3<Scalar> smallRotorRotationalInertiaX = RY.transpose() * _smallRotorRotationalInertiaZ * RY;
            Mat3<Scalar> smallRotorRotationalInertiaY = RX.transpose() * _smallRotorRotationalInertiaZ * RX;
            Mat3<Scalar> largeRotorRotationalInertiaY = RX.transpose() * _largeRotorRotationalInertiaZ * RX;

            _smallRotorRotInertiaX = smallRotorRotationalInertiaX;
            _smallRotorRotInertiaY = smallRotorRotationalInertiaY;
            _smallRotorRotInertiaZ = _smallRotorRotationalInertiaZ;
            _largeRotorRotInertiaY = largeRotorRotationalInertiaY;
            _largeRotorRotInertiaZ = _largeRotorRotationalInertiaZ;
        }

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;

    protected:
        //
        Scalar _torsoMass = 8.52; // from measuring the total weight of the humanoid

        Scalar _hipRzMass = 0.84563;
        Scalar _hipRxMass = 1.908683;
        Scalar _hipRyMass = 2.64093;
        Scalar _kneeMass = 0.3543355;
        Scalar _ankleMass = 0.280951;

        //
        Scalar _shoulderRyMass = 0.788506;
        Scalar _shoulderRxMass = 0.80125;
        Scalar _shoulderRzMass = 0.905588;
        Scalar _elbowMass = 0.34839;

        //
        Scalar _hipRzPitch = -0.174533;
        Scalar _hipRxPitch = 0.436332;
        Scalar _hipRyPitch = -(_hipRxPitch + _hipRzPitch);

        //
        Vec3<Scalar> _hipRzLocation = Vec3<Scalar>(-0.00565, -0.082, -0.05735);
        Vec3<Scalar> _hipRxLocation = Vec3<Scalar>(-0.06435, 0.0, -.07499);
        Vec3<Scalar> _hipRyLocation = Vec3<Scalar>(0.071, 0.0018375, 0.0);
        Vec3<Scalar> _kneeLocation = Vec3<Scalar>(0.0, 0.0, -0.267);
        Vec3<Scalar> _ankleLocation = Vec3<Scalar>(0.0, 0.0, -0.2785);

        Vec3<Scalar> _shoulderRyLocation = Vec3<Scalar>(0.01346, -0.17608, 0.24657);
        Vec3<Scalar> _shoulderRxLocation = Vec3<Scalar>(0.0, -0.0575, 0.0);
        Vec3<Scalar> _shoulderRzLocation = Vec3<Scalar>(0.0, 0.0, -0.10250);
        Vec3<Scalar> _elbowLocation = Vec3<Scalar>(0.0, 0.0, -0.1455);

        //
        Vec3<Scalar> _torsoCOM = Vec3<Scalar>(0.009896, 0.004771, 0.100522);

        Vec3<Scalar> _hipRzCOM = Vec3<Scalar>(-0.064842, -0.000036, -0.063090);
        Vec3<Scalar> _hipRxCOM = Vec3<Scalar>(0.067232, -0.013018, 0.0001831);
        Vec3<Scalar> _hipRyCOM = Vec3<Scalar>(0.0132054, 0.0269864, -0.096021);
        Vec3<Scalar> _kneeCOM = Vec3<Scalar>(0.00528, 0.0014762, -0.13201);
        Vec3<Scalar> _ankleCOM = Vec3<Scalar>(0.022623, 0.0, -0.012826);

        Vec3<Scalar> _shoulderRyCOM = Vec3<Scalar>(0.009265, 0.052623, -0.0001249);
        Vec3<Scalar> _shoulderRxCOM = Vec3<Scalar>(0.0006041, 0.0001221, -0.082361);
        Vec3<Scalar> _shoulderRzCOM = Vec3<Scalar>(0.0001703, -0.016797, -0.060);
        Vec3<Scalar> _elbowCOM = Vec3<Scalar>(-0.0059578, 0.000111, -0.0426735);

        Vec3<Scalar> _smallRotorCOM = Vec3<Scalar>(0., 0., 0.);
        Vec3<Scalar> _largeRotorCOM = Vec3<Scalar>(0., 0., 0.);

        //
        Scalar _hipRzGearRatio = 6.0;
        Scalar _hipRxGearRatio = 6.0;
        Scalar _hipRyGearRatio = 6.0;
        Scalar _kneeGearRatio = 6.0;
        Scalar _kneeBeltRatio = 2.0;
        Scalar _ankleGearRatio = 6.0;
        Scalar _ankleBeltRatio = 2.0;

        Scalar _shoulderRxGearRatio = 6.0;
        Scalar _shoulderRzGearRatio = 6.0;
        Scalar _shoulderRyGearRatio = 6.0;
        Scalar _elbowGearRatio = 9.0;

        //
        Vec3<Scalar> _hipRzRotorLocation = Vec3<Scalar>(-0.00842837, -0.082, -0.041593);
        Vec3<Scalar> _hipRxRotorLocation = Vec3<Scalar>(-0.0827, 0.0, -0.066436);
        Vec3<Scalar> _hipRyRotorLocation = Vec3<Scalar>(0.071, 0.024, 0.0);
        Vec3<Scalar> _kneeRotorLocation = Vec3<Scalar>(0.013, -0.0497, -0.0178);
        Vec3<Scalar> _ankleRotorLocation = Vec3<Scalar>(0., 0., 0.);

        Vec3<Scalar> _shoulderRyRotorLocation = Vec3<Scalar>(0.01346, -0.16, 0.24657);
        Vec3<Scalar> _shoulderRxRotorLocation = Vec3<Scalar>(0, -0.0575, 0);
        Vec3<Scalar> _shoulderRzRotorLocation = Vec3<Scalar>(0., 0., -0.1025);
        Vec3<Scalar> _elbowRotorLocation = Vec3<Scalar>(0., 0.0325, -0.06);

        //
        Mat3<Scalar> _torsoRotInertia;

        Mat3<Scalar> _hipRzRotInertia;
        Mat3<Scalar> _hipRxRotInertia;
        Mat3<Scalar> _hipRyRotInertia;
        Mat3<Scalar> _kneeRotInertia;
        Mat3<Scalar> _ankleRotInertia;

        Mat3<Scalar> _shoulderRyRotInertia;
        Mat3<Scalar> _shoulderRxRotInertia;
        Mat3<Scalar> _shoulderRzRotInertia;
        Mat3<Scalar> _elbowRotInertia;

        Mat3<Scalar> _largeRotorRotationalInertiaZ;
        Mat3<Scalar> _smallRotorRotationalInertiaZ;

        Mat3<Scalar> _smallRotorRotInertiaX;
        Mat3<Scalar> _smallRotorRotInertiaY;
        Mat3<Scalar> _smallRotorRotInertiaZ;
        Mat3<Scalar> _largeRotorRotInertiaY;
        Mat3<Scalar> _largeRotorRotInertiaZ;

        //
        Scalar _torsoLength = .1;
        Scalar _torsoWidth = .164;
        Scalar _torsoHeight = 0.30392;

        Scalar _thighLength = 0.267;
        Scalar _footToeLength = 0.1;
        Scalar _footHeelLength = 0.05;
        Scalar _footHeight = 0.041;

        Scalar _lowerArmLength = 0.27;

        template <typename T>
        Vec3<Scalar> withLeftRightSigns(const Eigen::MatrixBase<T> &v, int side) const
        {
            static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                          "Must have 3x1 matrix");
            switch (side)
            {
            case 0: // right
                return Vec3<Scalar>(v[0], v[1], v[2]);
            case 1: // left
                return Vec3<Scalar>(v[0], -v[1], v[2]);
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

        SpatialInertia<Scalar> withLeftRightSigns(const SpatialInertia<Scalar> &I, int side) const
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

#endif // GRBDA_ROBOTS_MIT_HUMANOID_H
