#ifndef GRBDA_ROBOTS_MINI_CHEETAH_H
#define GRBDA_ROBOTS_MINI_CHEETAH_H

#include "Robot.h"

namespace grbda
{

    class MiniCheetah : public Robot<double>
    {
    public:
        MiniCheetah()
        {
            Mat3<double> RY = ori::coordinateRotation<double>(ori::CoordinateAxis::Y, M_PI / 2);
            Mat3<double> RX = ori::coordinateRotation<double>(ori::CoordinateAxis::X, M_PI / 2);

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

        ClusterTreeModel<> buildClusterTreeModel() const override;

    private:
        //
        double _bodyMass = 3.3;
        double _abadMass = 0.54;
        double _hipMass = 0.634;
        double _kneeMass = 0.064;

        //
        double _bodyLength = 0.19 * 2;
        double _bodyWidth = 0.049 * 2;
        double _bodyHeight = 0.05 * 2;
        double _abadLinkLength = 0.062;
        double _hipLinkLength = 0.209;
        double _kneeLinkLength = 0.195;
        double _kneeLinkY_offset = 0.004;

        //
        double _abadGearRatio = 6;
        double _hipGearRatio = 6;
        double _kneeGearRatio = 9.33;

        //
        Vec3<double> _bodyCOM = Vec3<double>(0, 0, 0);
        Vec3<double> _abadCOM = Vec3<double>(0, 0.036, 0);
        Vec3<double> _hipCOM = Vec3<double>(0, 0.016, -0.02);
        Vec3<double> _kneeCOM = Vec3<double>(0, 0, -0.061);
        Vec3<double> _rotorCOM = Vec3<double>(0, 0, 0);

        //
        Mat3<double> _bodyRotationalInertia;
        Mat3<double> _abadRotationalInertia;
        Mat3<double> _hipRotationalInertia;
        Mat3<double> _kneeRotationalInertia, _kneeRotationalInertiaRotated;

        Mat3<double> _rotorRotationalInertiaX;
        Mat3<double> _rotorRotationalInertiaY;
        Mat3<double> _rotorRotationalInertiaZ;

        //
        Vec3<double> _abadRotorLocation = Vec3<double>(0.125, 0.049, 0);
        Vec3<double> _abadLocation = Vec3<double>(0.38, 0.1, 0) * 0.5;
        Vec3<double> _hipLocation = Vec3<double>(0, 0.062, 0);
        Vec3<double> _hipRotorLocation = Vec3<double>(0, 0.04, 0);
        Vec3<double> _kneeLocation = Vec3<double>(0, 0, -0.209);
        Vec3<double> _kneeRotorLocation = Vec3<double>(0, 0, 0);

        template <typename T>
        Vec3<double> withLegSigns(const Eigen::MatrixBase<T> &v, int side) const
        {
            static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                          "Must have 3x1 matrix");
            switch (side)
            {
            case 0:
                return Vec3<double>(v[0], -v[1], v[2]);
            case 1:
                return Vec3<double>(v[0], v[1], v[2]);
            case 2:
                return Vec3<double>(-v[0], -v[1], v[2]);
            case 3:
                return Vec3<double>(-v[0], v[1], v[2]);
            default:
                throw std::runtime_error("Invalid leg id!");
            }
        }

        std::string withLegSigns(const std::string &s, int side) const
        {
            switch (side)
            {
            case 0:
                return "FR_" + s;
            case 1:
                return "FL_" + s;
            case 2:
                return "HR_" + s;
            case 3:
                return "HL_" + s;
            default:
                throw std::runtime_error("Invalid leg id!");
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

};

#endif // GRBDA_ROBOTS_MINI_CHEETAH_H
