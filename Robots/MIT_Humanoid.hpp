#pragma once

#include "Robot.h"

namespace grbda
{

    class MIT_Humanoid : public Robot
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

            Mat3<double> RY = coordinateRotation<double>(CoordinateAxis::Y, M_PI / 2);
            Mat3<double> RX = coordinateRotation<double>(CoordinateAxis::X, -M_PI / 2);

            Mat3<double> smallRotorRotationalInertiaX = RY.transpose() * _smallRotorRotationalInertiaZ * RY;
            Mat3<double> smallRotorRotationalInertiaY = RX.transpose() * _smallRotorRotationalInertiaZ * RX;
            Mat3<double> largeRotorRotationalInertiaY = RX.transpose() * _largeRotorRotationalInertiaZ * RX;

            _smallRotorRotInertiaX = smallRotorRotationalInertiaX;
            _smallRotorRotInertiaY = smallRotorRotationalInertiaY;
            _smallRotorRotInertiaZ = _smallRotorRotationalInertiaZ;
            _largeRotorRotInertiaY = largeRotorRotationalInertiaY;
            _largeRotorRotInertiaZ = _largeRotorRotationalInertiaZ;
        }

        ClusterTreeModel buildClusterTreeModel() const override;

    private:
        //
        double _torsoMass = 8.52; // from measuring the total weight of the humanoid

        double _hipRzMass = 0.84563;
        double _hipRxMass = 1.908683;
        double _hipRyMass = 2.64093;
        double _kneeMass = 0.3543355;
        double _ankleMass = 0.280951;

        //
        double _shoulderRyMass = 0.788506;
        double _shoulderRxMass = 0.80125;
        double _shoulderRzMass = 0.905588;
        double _elbowMass = 0.34839;

        //
        double _hipRzPitch = -0.174533;
        double _hipRxPitch = 0.436332;
        double _hipRyPitch = -(_hipRxPitch + _hipRzPitch);

        //
        Vec3<double> _hipRzLocation = Vec3<double>(-0.00565, -0.082, -0.05735);
        Vec3<double> _hipRxLocation = Vec3<double>(-0.06435, 0.0, -.07499);
        Vec3<double> _hipRyLocation = Vec3<double>(0.071, 0.0018375, 0.0);
        Vec3<double> _kneeLocation = Vec3<double>(0.0, 0.0, -0.267);
        Vec3<double> _ankleLocation = Vec3<double>(0.0, 0.0, -0.2785);

        Vec3<double> _shoulderRyLocation = Vec3<double>(0.01346, -0.17608, 0.24657);
        Vec3<double> _shoulderRxLocation = Vec3<double>(0.0, -0.0575, 0.0);
        Vec3<double> _shoulderRzLocation = Vec3<double>(0.0, 0.0, -0.10250);
        Vec3<double> _elbowLocation = Vec3<double>(0.0, 0.0, -0.1455);

        //
        Vec3<double> _torsoCOM = Vec3<double>(0.009896, 0.004771, 0.100522);

        Vec3<double> _hipRzCOM = Vec3<double>(-0.064842, -0.000036, -0.063090);
        Vec3<double> _hipRxCOM = Vec3<double>(0.067232, -0.013018, 0.0001831);
        Vec3<double> _hipRyCOM = Vec3<double>(0.0132054, 0.0269864, -0.096021);
        Vec3<double> _kneeCOM = Vec3<double>(0.00528, 0.0014762, -0.13201);
        Vec3<double> _ankleCOM = Vec3<double>(0.022623, 0.0, -0.012826);

        Vec3<double> _shoulderRyCOM = Vec3<double>(0.009265, 0.052623, -0.0001249);
        Vec3<double> _shoulderRxCOM = Vec3<double>(0.0006041, 0.0001221, -0.082361);
        Vec3<double> _shoulderRzCOM = Vec3<double>(0.0001703, -0.016797, -0.060);
        Vec3<double> _elbowCOM = Vec3<double>(-0.0059578, 0.000111, -0.0426735);

        Vec3<double> _smallRotorCOM = Vec3<double>(0., 0., 0.);
        Vec3<double> _largeRotorCOM = Vec3<double>(0., 0., 0.);

        //
        double _hipRzGearRatio = 6.0;
        double _hipRxGearRatio = 6.0;
        double _hipRyGearRatio = 6.0;
        double _kneeGearRatio = 12.0;
        double _kneeBeltRatio = 2.0;
        double _ankleGearRatio = 12.0;
        double _ankleBeltRatio = 2.0;

        double _shoulderRxGearRatio = 6.0;
        double _shoulderRzGearRatio = 6.0;
        double _shoulderRyGearRatio = 6.0;
        double _elbowGearRatio = 9.0;

        //
        Vec3<double> _hipRzRotorLocation = Vec3<double>(-0.00842837, -0.082, -0.041593);
        Vec3<double> _hipRxRotorLocation = Vec3<double>(-0.0827, 0.0, -0.066436);
        Vec3<double> _hipRyRotorLocation = Vec3<double>(0.071, 0.024, 0.0);
        Vec3<double> _kneeRotorLocation = Vec3<double>(0.013, -0.0497, -0.0178);
        Vec3<double> _ankleRotorLocation = Vec3<double>(0., 0., 0.);

        Vec3<double> _shoulderRyRotorLocation = Vec3<double>(0.01346, -0.16, 0.24657);
        Vec3<double> _shoulderRxRotorLocation = Vec3<double>(0, -0.0575, 0);
        Vec3<double> _shoulderRzRotorLocation = Vec3<double>(0., 0., -0.1025);
        Vec3<double> _elbowRotorLocation = Vec3<double>(0., 0.0325, -0.06);

        //
        Mat3<double> _torsoRotInertia;

        Mat3<double> _hipRzRotInertia;
        Mat3<double> _hipRxRotInertia;
        Mat3<double> _hipRyRotInertia;
        Mat3<double> _kneeRotInertia;
        Mat3<double> _ankleRotInertia;

        Mat3<double> _shoulderRyRotInertia;
        Mat3<double> _shoulderRxRotInertia;
        Mat3<double> _shoulderRzRotInertia;
        Mat3<double> _elbowRotInertia;

        Mat3<double> _largeRotorRotationalInertiaZ;
        Mat3<double> _smallRotorRotationalInertiaZ;

        Mat3<double> _smallRotorRotInertiaX;
        Mat3<double> _smallRotorRotInertiaY;
        Mat3<double> _smallRotorRotInertiaZ;
        Mat3<double> _largeRotorRotInertiaY;
        Mat3<double> _largeRotorRotInertiaZ;

        //
        double _torsoLength = .1;
        double _torsoWidth = .164;
        double _torsoHeight = 0.30392;

        double _thighLength = 0.267;
        double _footToeLength = 0.1;
        double _footHeelLength = 0.05;
        double _footHeight = 0.041;

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
}
