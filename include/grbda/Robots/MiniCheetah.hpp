#ifndef GRBDA_ROBOTS_MINI_CHEETAH_H
#define GRBDA_ROBOTS_MINI_CHEETAH_H

#include "grbda/Robots/Robot.h"

namespace grbda
{

    template <typename Scalar = double,
              typename OrientationRepresentation = ori_representation::Quaternion>
    class MiniCheetah : public Robot<Scalar>
    {
    public:
        MiniCheetah()
        {
            Mat3<Scalar> RY = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::Y, M_PI / 2);
            Mat3<Scalar> RX = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::X, M_PI / 2);

            _bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
            _bodyRotationalInertia = _bodyRotationalInertia * 1e-6;

            _abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
            _abadRotationalInertia = _abadRotationalInertia * 1e-6;

            _hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
            _hipRotationalInertia = _hipRotationalInertia * 1e-6;

            _kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
            _kneeRotationalInertiaRotated = _kneeRotationalInertiaRotated * 1e-6;
            _kneeRotationalInertia = RY * _kneeRotationalInertiaRotated * RY.transpose();

            _rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
            _rotorRotationalInertiaZ = 1e-6 * _rotorRotationalInertiaZ;

            _rotorRotationalInertiaX = RY * _rotorRotationalInertiaZ * RY.transpose();
            _rotorRotationalInertiaY = RX * _rotorRotationalInertiaZ * RX.transpose();
        }

        ClusterTreeModel<Scalar> buildClusterTreeModel() const override;

    private:
        //
        Scalar _bodyMass = 3.3;
        Scalar _abadMass = 0.54;
        Scalar _hipMass = 0.634;
        Scalar _kneeMass = 0.064;

        //
        Scalar _bodyLength = 0.19 * 2;
        Scalar _bodyWidth = 0.049 * 2;
        Scalar _bodyHeight = 0.05 * 2;
        Scalar _abadLinkLength = 0.062;
        Scalar _hipLinkLength = 0.209;
        Scalar _kneeLinkLength = 0.195;
        Scalar _kneeLinkY_offset = 0.004;

        //
        Scalar _abadGearRatio = 6;
        Scalar _hipGearRatio = 6;
        Scalar _kneeGearRatio = 9.33;

        //
        Vec3<Scalar> _bodyCOM = Vec3<Scalar>(0, 0, 0);
        Vec3<Scalar> _abadCOM = Vec3<Scalar>(0, 0.036, 0);
        Vec3<Scalar> _hipCOM = Vec3<Scalar>(0, 0.016, -0.02);
        Vec3<Scalar> _kneeCOM = Vec3<Scalar>(0, 0, -0.061);
        Vec3<Scalar> _rotorCOM = Vec3<Scalar>(0, 0, 0);

        //
        Mat3<Scalar> _bodyRotationalInertia;
        Mat3<Scalar> _abadRotationalInertia;
        Mat3<Scalar> _hipRotationalInertia;
        Mat3<Scalar> _kneeRotationalInertia, _kneeRotationalInertiaRotated;

        Mat3<Scalar> _rotorRotationalInertiaX;
        Mat3<Scalar> _rotorRotationalInertiaY;
        Mat3<Scalar> _rotorRotationalInertiaZ;

        //
        Vec3<Scalar> _abadRotorLocation = Vec3<Scalar>(0.125, 0.049, 0);
        Vec3<Scalar> _abadLocation = Vec3<Scalar>(0.38, 0.1, 0) * 0.5;
        Vec3<Scalar> _hipLocation = Vec3<Scalar>(0, 0.062, 0);
        Vec3<Scalar> _hipRotorLocation = Vec3<Scalar>(0, 0.04, 0);
        Vec3<Scalar> _kneeLocation = Vec3<Scalar>(0, 0, -0.209);
        Vec3<Scalar> _kneeRotorLocation = Vec3<Scalar>(0, 0, 0);

        template <typename T>
        Vec3<Scalar> withLegSigns(const Eigen::MatrixBase<T> &v, int side) const
        {
            static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                          "Must have 3x1 matrix");
            switch (side)
            {
            case 0:
                return Vec3<Scalar>(v[0], v[1], v[2]);
            case 1:
                return Vec3<Scalar>(v[0], -v[1], v[2]);
            case 2:
                return Vec3<Scalar>(-v[0], v[1], v[2]);
            case 3:
                return Vec3<Scalar>(-v[0], -v[1], v[2]);
            default:
                throw std::runtime_error("Invalid leg id!");
            }
        }

        std::string withLegSigns(const std::string &s, int side) const
        {
            switch (side)
            {
            case 0:
                return "FL_" + s;
            case 1:
                return "FR_" + s;
            case 2:
                return "HL_" + s;
            case 3:
                return "HR_" + s;
            default:
                throw std::runtime_error("Invalid leg id!");
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

};

#endif // GRBDA_ROBOTS_MINI_CHEETAH_H
